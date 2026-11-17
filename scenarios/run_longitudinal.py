"""Configured CARLA-SUMO longitudinal experiments. See docs/longitudinal.md."""
import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers"))
from acc import ACC, SpeedPID, arbitrate
from longitudinal_metrics import evaluate


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    if cfg["step_s"] != 0.05:
        raise ValueError("This benchmark requires a 0.05 s step")
    names = set()
    for case in cfg["cases"]:
        if case.get('demo_mode','acc') not in ('acc','baseline'):
            raise ValueError('Scene mode must be acc or baseline')
        if case["name"] in names or not case["name"].replace("_", "").isalnum():
            raise ValueError("Case names must be unique alphanumeric identifiers")
        names.add(case["name"])
        if case["kind"] not in ("follow", "slowdown", "hard_brake", "cutin", "cutout", "adjacent"):
            raise ValueError("Unknown scenario kind")
        for key in ("ego_speed_mps", "lead_speed_mps", "offset_m", "duration_s"):
            if not math.isfinite(case[key]) or case[key] <= 0:
                raise ValueError("Positive finite " + key + " required")
        if not 0 <= case["trigger_s"] < case["duration_s"] - 2:
            raise ValueError("Trigger must leave time to observe the event")
        if abs(case["duration_s"] / cfg["step_s"] - round(case["duration_s"] / cfg["step_s"])) > 1e-6:
            raise ValueError("Duration must contain a whole number of steps")
        if case["kind"] == "slowdown" and not 0 <= case["slow_speed_mps"] < case["lead_speed_mps"] - 1:
            raise ValueError("Slowdown requires a measurable lead speed change")
        if case["kind"] in ("cutin", "cutout") and not 0 < case.get("lane_change_s", 2) < case["duration_s"] - case["trigger_s"]:
            raise ValueError("Lane change must fit in the observation window")
    return cfg


def run_case(cfg, mode, gains, camera=None, trace_path=None):
    distance_reference=cfg.get('aeb_distance_reference','center')
    if distance_reference not in ('center','body'):
        raise ValueError('Unknown AEB distance reference')
    if distance_reference=='body' and cfg.get('target_policy')!='route':
        raise ValueError('Body-gap experiment requires route target selection')
    if cfg.get('aeb_closing_guard',False) and distance_reference!='body':
        raise ValueError('Closing-guard experiment requires body-gap distance')
    if cfg.get('backend','python')!='python' and (cfg.get('lqr_schedule') or cfg.get('steering_filter_alpha',.4)!=.4):
        raise ValueError('Experimental scheduling/filter ablations require the Python backend')
    # Import CARLA only when executing a simulator experiment.
    from run_cosim_aeb import (carla, CarlaSimulation, SumoSimulation,
                               SimulationSynchronization, AEBController, bridge)
    from stanley_pid_controller import StanleyController
    from route_tracking import RouteTracking
    sim = CarlaSimulation("127.0.0.1", 2000, cfg["step_s"])
    sim.client.set_timeout(30)
    world = sim.world
    road = world.get_map()
    initial = world.get_settings()
    if not road.name.endswith("Town04") or initial.synchronous_mode or list(world.get_actors().filter("vehicle.*")):
        raise RuntimeError("An idle asynchronous Town04 with no vehicles is required")
    settings = world.get_settings()
    settings.no_rendering_mode = camera is None
    owned, contacts, rows = [], [], []
    native = None
    sync = sumo = None
    trace = trace_path.open("w", newline="", encoding="utf-8") if trace_path else None
    writer = None
    try:
        world.apply_settings(settings)
        random.seed(cfg["seed"])
        sumo = SumoSimulation(str(bridge / "examples" / "Town04.sumocfg"), cfg["step_s"])
        sync = SimulationSynchronization(sumo, sim, tls_manager="none")
        start = road.get_spawn_points()[cfg.get("spawn_index",269)]
        wp = road.get_waypoint(start.location)
        lead_wp = wp.get_left_lane() if cfg["kind"] in ("cutin", "adjacent") else wp
        target = lead_wp.next(cfg["offset_m"])[0].transform
        target.location.z += 0.28
        bp = world.get_blueprint_library().find("vehicle.tesla.model3")
        for role, pose in (("acc_ego", start), ("acc_lead", target)):
            bp.set_attribute("role_name", role)
            owned.append(world.spawn_actor(bp, pose))
        ego, lead = owned
        sensor = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                   carla.Transform(), attach_to=ego)
        owned.append(sensor)
        sensor.listen(lambda e: contacts.append((e.frame, e.other_actor.id)))
        if camera:
            owned.append(camera(world, ego))
        for _ in range(10):
            sync.tick()
        for vehicle, speed in ((ego, cfg["ego_speed_mps"]), (lead, cfg["lead_speed_mps"])):
            fwd = vehicle.get_transform().get_forward_vector()
            vehicle.set_target_velocity(carla.Vector3D(fwd.x * speed, fwd.y * speed, 0))
        sync.tick()
        origin = lead.get_transform()
        forward, right = origin.get_forward_vector(), origin.get_right_vector()
        kinematic = cfg["kind"] in ("cutin", "cutout")
        if kinematic:
            lead.set_simulate_physics(False)
        pid, lead_pid = SpeedPID(), SpeedPID()
        steering, lead_steering = StanleyController(k=0.08), StanleyController(k=0.08)
        lateral_mode = cfg.get("lateral_controller","stanley")
        route = RouteTracking(road,ego,"lqr" if lateral_mode=="lqr" else "stanley",cfg["ego_speed_mps"],
                              schedule=cfg.get('lqr_schedule',False),filter_alpha=cfg.get('steering_filter_alpha',.4))
        lead_route = RouteTracking(road,lead,"stanley",cfg["lead_speed_mps"]) if cfg.get("curved") else None
        runtime_gains=dict(gains,lateral_mode=int(lateral_mode=="lqr"),lqr_speed=max(cfg["ego_speed_mps"],1),
                           lqr_wheelbase=route.wheelbase,lqr_dt=cfg["step_s"])
        if lateral_mode=="lqr":
            cfg['lqr_model']=dict(design_speed_mps=runtime_gains['lqr_speed'],wheelbase_m=route.wheelbase,
                                  dt_s=cfg['step_s'],position_weight=.2)
        if cfg.get("backend","python")=="cpp":
            from control_native import NativeController
            native=NativeController(runtime_gains)
        elif cfg.get("backend")=="ros2":
            from control_ros import RosController
            native=RosController(runtime_gains)
        acc = ACC(**gains)
        aeb = AEBController(closing_guard=cfg.get('aeb_closing_guard',False))
        if cfg.get('full_brake_at_onset',False):
            aeb.partial_brake_strength = aeb.full_brake_strength
        if cfg.get('full_brake_at_warning',False):
            aeb.ttc_full = aeb.ttc_warning
        latched = 0.0
        previous_location = None
        initial_frame = world.get_snapshot().frame
        for tick in range(round(cfg["duration_s"] / cfg["step_s"])):
            t = tick * cfg["step_s"]
            pose, target_pose = ego.get_transform(), lead.get_transform()
            heading = math.radians(pose.rotation.yaw)
            dx, dy = target_pose.location.x - pose.location.x, target_pose.location.y - pose.location.y
            longitudinal = dx * math.cos(heading) + dy * math.sin(heading)
            lateral = -dx * math.sin(heading) + dy * math.cos(heading)
            yaw = math.radians(target_pose.rotation.yaw - pose.rotation.yaw)
            half_length = abs(math.cos(yaw)) * lead.bounding_box.extent.x + abs(math.sin(yaw)) * lead.bounding_box.extent.y
            half_width = abs(math.sin(yaw)) * lead.bounding_box.extent.x + abs(math.cos(yaw)) * lead.bounding_box.extent.y
            gap = longitudinal - ego.bounding_box.extent.x - half_length
            chord_gap = gap
            path_gap = route.state(lead)["station"]-route.state(ego)["station"]-ego.bounding_box.extent.x-lead.bounding_box.extent.x
            if cfg.get("curved"):
                gap=path_gap
            clearance = abs(lateral) - ego.bounding_box.extent.y - half_width
            speed, lead_speed = ego.get_velocity().length(), lead.get_velocity().length()
            overrides = None
            if kinematic and previous_location is not None:
                velocity = carla.Vector3D(
                    (target_pose.location.x - previous_location.x) / cfg["step_s"],
                    (target_pose.location.y - previous_location.y) / cfg["step_s"], 0)
                overrides = {lead.id: velocity}
                lead_speed = velocity.length()
            previous_location = target_pose.location
            distance, closing = aeb.detect_front_vehicle(ego, world, velocity_overrides=overrides)
            if cfg.get("target_policy")=="route":
                distance,actor_id,closing=route.nearest_target(ego,world,aeb.detection_range,
                                                              body_gap=distance_reference=='body')
                aeb.last_target_id=actor_id
            requested = aeb.update_state(distance, closing)
            target_id = aeb.last_target_id
            # ACC only follows the designated lead; unexpected traffic is reported.
            following = target_id == lead.id
            reference = acc.target_speed(speed, cfg["ego_speed_mps"], gap if following else None, closing) if mode == "acc" else cfg["ego_speed_mps"]
            throttle, brake = pid.update(reference, speed, cfg["step_s"])
            throttle, brake, latched = arbitrate(throttle, brake, requested, latched)
            ego_wp = road.get_waypoint(pose.location)
            steer, cross_track, heading_error = route.update(ego)
            if lateral_mode == "legacy":
                steer = steering.compute_steering_angle(ego, ego_wp.next(6)[0])[0]
            backend_error=backend_us=0.
            if native:
                values=dict(route.control_input,dt=cfg["step_s"],speed=speed,cruise=cfg["ego_speed_mps"],
                            gap=gap,closing=closing,aeb_request=requested,
                            has_target=int(following),acc_enabled=int(mode=="acc"))
                expected=dict(reference=reference,throttle=throttle,brake=brake,steer=steer,latched_brake=latched)
                started=time.perf_counter()
                try:
                    if cfg.get("backend")=="ros2":
                        native.set_scene(pose,route.points)
                    computed=native.step(values)
                except Exception as exc:
                    replies=sim.client.apply_batch_sync([carla.command.ApplyVehicleControl(ego.id,
                                                 carla.VehicleControl(throttle=0.,brake=1.))],False)
                    accepted=all(not reply.error for reply in replies)
                    raise RuntimeError('Control backend failed; safe_brake_batch_accepted='+str(accepted)+
                                       '; '+repr(exc)) from exc
                backend_us=(time.perf_counter()-started)*1e6
                backend_error=max(abs(computed[k]-expected[k]) for k in expected)
                if backend_error>1e-9:
                    raise RuntimeError("Native/Python command mismatch")
                reference,throttle,brake,steer,latched=(computed[k] for k in ("reference","throttle","brake","steer","latched_brake"))
            commands = [carla.command.ApplyVehicleControl(ego.id,
                        carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))]
            if kinematic:
                fraction = max(0, min(1, (t - cfg["trigger_s"]) / cfg.get("lane_change_s", 2)))
                shift = (3.5 if cfg["kind"] == "cutin" else -3.5) * fraction
                lead.set_transform(carla.Transform(carla.Location(
                    x=origin.location.x + forward.x * cfg["lead_speed_mps"] * t + right.x * shift,
                    y=origin.location.y + forward.y * cfg["lead_speed_mps"] * t + right.y * shift,
                    z=origin.location.z), origin.rotation))
            else:
                target_speed = cfg["lead_speed_mps"]
                if cfg["kind"] == "slowdown" and t >= cfg["trigger_s"]:
                    target_speed = max(cfg["slow_speed_mps"], target_speed - (t - cfg["trigger_s"]) * 1.0)
                lt, lb = lead_pid.update(target_speed, lead_speed, cfg["step_s"])
                if cfg["kind"] in ("hard_brake", "adjacent") and t >= cfg["trigger_s"]:
                    lt, lb = 0.0, 1.0
                lead_waypoint = road.get_waypoint(target_pose.location).next(6)[0]
                ls = lead_route.update(lead)[0] if lead_route else lead_steering.compute_steering_angle(lead, lead_waypoint)[0]
                commands.append(carla.command.ApplyVehicleControl(lead.id,
                                carla.VehicleControl(throttle=lt, brake=lb, steer=ls)))
            replies = sim.client.apply_batch_sync(commands,False)
            if any(reply.error for reply in replies):
                raise RuntimeError("Vehicle control batch failed")
            observed = t > cfg["trigger_s"]
            if cfg["kind"] == "follow":
                observed = observed and following and abs(lead_speed - cfg["lead_speed_mps"]) < 1
            elif cfg["kind"] == "cutin":
                observed = observed and clearance <= 0
            elif cfg["kind"] == "cutout":
                observed = observed and clearance > 0 and not following
            elif cfg["kind"] == "slowdown":
                observed = observed and lead_speed < cfg["lead_speed_mps"] - 1
            elif cfg["kind"] in ("hard_brake", "adjacent"):
                observed = observed and lead_speed < 1
            rows.append(dict(time_s=round(t, 4), frame=world.get_snapshot().frame,
                             ego_speed_mps=speed, lead_speed_mps=lead_speed, gap_m=gap,
                             longitudinal_m=longitudinal, lateral_clearance_m=clearance,
                             target_id=target_id, lead_id=lead.id, target_speed_mps=reference,
                             throttle=throttle, brake=brake, aeb_brake=requested,
                             latched_brake=latched, cross_track_m=cross_track,
                             path_error_m=cross_track, heading_error_rad=heading_error, steer=steer,
                             control_state="AEB" if latched>0 else "FOLLOW" if mode=="acc" and following else "CRUISE",
                             ego_yaw_rad=heading,
                             lead_path_error_m=lead_route.state(lead)["error"] if lead_route else 0.,
                             chord_gap_m=chord_gap,path_gap_m=path_gap,
                             path_distance_m=path_gap+ego.bounding_box.extent.x+lead.bounding_box.extent.x,
                             backend_error_max=backend_error,backend_compute_us=backend_us,
                             event_observed=observed, sumo_background=len(sync.sumo2carla_ids)))
            rows[-1].update(aeb_distance_m=distance,aeb_closing_mps=closing,
                            aeb_ttc_s=aeb.calculate_ttc(distance,closing) if distance is not None else None,
                            aeb_level=aeb.state)
            if cfg.get('target_policy')=='route':
                rows[-1].update(aeb_center_distance_m=route.target_geometry['center_distance_m'],
                                aeb_body_gap_m=route.target_geometry['body_gap_m'])
            if lateral_mode=="lqr":
                rows[-1].update(control_cross_track_m=route.control_input['cross_track'],
                    control_path_heading_rad=route.control_input['path_heading'],
                    control_curvature_inv_m=route.control_input['curvature'],
                    control_max_angle_rad=route.control_input['max_angle'],control_closing_mps=closing,
                    control_has_target=following)
            if camera and hasattr(camera,"on_sample"):
                camera.on_sample(rows[-1])
            if trace:
                if writer is None:
                    writer = csv.DictWriter(trace, fieldnames=list(rows[-1]))
                    writer.writeheader()
                writer.writerow(rows[-1])
                trace.flush()
            sync.tick()
        events = [{"time_s": round((frame - initial_frame) * cfg["step_s"], 4),
                   "other_id": actor} for frame, actor in contacts]
        return rows, events
    finally:
        if native:
            native.close()
        if trace:
            trace.close()
        try:
            for actor in owned:
                if actor.type_id.startswith("sensor.") and actor.is_listening:
                    actor.stop()
            ids = [actor.id for actor in reversed(owned)]
            if sync:
                ids.extend(sync.sumo2carla_ids.values())
            if ids:
                responses = sim.client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)
                errors = [r.error for r in responses if r.has_error()]
                if errors:
                    raise RuntimeError("Actor cleanup failed: " + "; ".join(errors))
                if sync:
                    sync.sumo2carla_ids.clear()
        finally:
            try:
                if sync:
                    sync.close()
                    sim.client.get_trafficmanager().set_synchronous_mode(False)
                elif sumo:
                    sumo.close()
            finally:
                world.apply_settings(initial)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs" / "longitudinal.json"))
    parser.add_argument("--case")
    parser.add_argument("--split", choices=("development", "holdout"))
    parser.add_argument("--mode", choices=("acc", "baseline", "both", "scene"), default="both")
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "longitudinal")
    parser.add_argument("--gap-gain", type=float)
    parser.add_argument("--relative-gain", type=float)
    parser.add_argument("--lateral", choices=("legacy","stanley","lqr"),default="stanley")
    parser.add_argument("--target-policy",choices=("corridor","route"))
    parser.add_argument("--backend",choices=("python","cpp","ros2"),default="python")
    args = parser.parse_args()
    if args.backend!="python" and args.lateral=="legacy":
        parser.error("The native core implements Stanley and LQR lateral control")
    config = load_config(args.config)
    gains = dict(config["gains"])
    for key in ("gap_gain", "relative_gain"):
        if getattr(args, key) is not None:
            gains[key] = getattr(args, key)
    cases = [c for c in config["cases"] if (not args.case or c["name"] == args.case)
             and (not args.split or c["split"] == args.split)]
    if not cases:
        parser.error("No matching cases")
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"gains": gains, "runs": {}}
    errors = False
    for case in cases:
        cfg = dict(case, step_s=config["step_s"], seed=config["seed"],lateral_controller=args.lateral,
                   target_policy=args.target_policy or case.get("target_policy","corridor"),backend=args.backend)
        modes=(case.get('demo_mode','acc'),) if args.mode=='scene' else (('baseline','acc') if args.mode=='both' else (args.mode,))
        for mode in modes:
            name = case["name"] + "_" + mode
            report["runs"][name] = dict(config=cfg, mode=mode, status="running")
            (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            try:
                rows, events = run_case(cfg, mode, gains, trace_path=args.output / (name + ".csv"))
                result = dict(config=cfg, mode=mode, metrics=evaluate(rows, events, cfg), collisions=events)
                print(name, result["metrics"], flush=True)
            except Exception as exc:
                result = dict(config=cfg, mode=mode, execution_error=repr(exc))
                errors = True
                print(name, repr(exc), flush=True)
            report["runs"][name] = result
            (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            if errors:
                raise SystemExit("Execution failed; saved error in summary.json. Repair environment before retrying.")


if __name__ == "__main__":
    main()
