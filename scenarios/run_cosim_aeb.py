"""Run the CARLA–SUMO AEB comparison on Town04."""

import argparse
import csv
import json
import math
import os
import random
import sys
from itertools import product
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
try:
    bridge = Path(os.environ["CARLA_HOME"]) / "Co-Simulation" / "Sumo"
    sumo_home = Path(os.environ["SUMO_HOME"])
except KeyError as error:
    raise SystemExit("Set CARLA_HOME and SUMO_HOME as shown in the README.") from error
os.environ["PATH"] = str(sumo_home / "bin") + os.pathsep + os.environ["PATH"]
sys.path[:0] = [str(bridge), str(sumo_home / "tools"), str(REPO / "controllers")]

import carla
import sumolib
import traci

# The CARLA 0.9.13 bridge expects this attribute on TraCI.
traci.sumolib = sumolib
from run_synchronization import SimulationSynchronization
from stanley_pid_controller import AEBController
from sumo_integration.carla_simulation import CarlaSimulation
from sumo_integration.sumo_simulation import SumoSimulation


STEP = 0.05
SPEED = 12.0
SEED = 42
CUTIN_OFFSETS = (6.0, 12.0, 18.0)
OUTPUT = REPO / "output" / "cosim-aeb"


def run_case(sim, enabled, adjacent=False, cutin_offset=None, predict_cutin=True, attach_camera=None,
             *, ego_speed_mps=SPEED, cutin_duration_s=2.0):
    cutin = cutin_offset is not None
    world = sim.world
    road = world.get_map()
    start = road.get_spawn_points()[269]
    waypoint = road.get_waypoint(start.location)
    if (waypoint.road_id, waypoint.lane_id) != (47, 6):
        raise RuntimeError("Unexpected Town04 start waypoint")
    target_waypoint = waypoint
    if adjacent or cutin:
        target_waypoint = waypoint.get_left_lane()
        if (target_waypoint is None or target_waypoint.lane_type != carla.LaneType.Driving
                or target_waypoint.road_id != waypoint.road_id
                or target_waypoint.lane_id * waypoint.lane_id <= 0):
            raise RuntimeError("Expected an adjacent lane traveling in the same direction")
    lead_start = target_waypoint.next(cutin_offset if cutin else (6.0 if adjacent else 22.0))[0].transform
    lead_start.location.z += 0.28

    existing = list(world.get_actors().filter("vehicle.*"))
    if world.get_settings().synchronous_mode or any(
        actor.attributes.get("role_name") != "camera_preview"
        or actor.get_location().distance(start.location) < 150.0 for actor in existing
    ):
        raise RuntimeError("The CARLA world is busy or the camera preview is near the test road")

    # Keep the parked Pygame preview outside the synchronization and cleanup.
    sim._active_actors = {actor.id for actor in existing}
    random.seed(SEED)
    sumo = SumoSimulation(str(bridge / "examples" / "Town04.sumocfg"), STEP)
    sync = None
    owned = []
    collisions = []
    rows = []
    try:
        sync = SimulationSynchronization(sumo, sim, tls_manager="none")
        blueprint = world.get_blueprint_library().find("vehicle.tesla.model3")
        for name, transform in [("aeb_ego", start), ("aeb_lead", lead_start)]:
            blueprint.set_attribute("role_name", name)
            if attach_camera:
                blueprint.set_attribute("color", "29,88,167" if name == "aeb_ego" else "218,121,26")
            owned.append(world.spawn_actor(blueprint, transform))
        ego, lead = owned
        if attach_camera:
            owned.append(attach_camera(world, ego))
        sensor = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                   carla.Transform(), attach_to=ego)
        owned.append(sensor)
        sensor.listen(lambda event: collisions.append((event.frame, event.other_actor.id)))
        for _ in range(10):
            sync.tick()
        for vehicle, speed in ((ego, ego_speed_mps), (lead, SPEED)):
            forward = vehicle.get_transform().get_forward_vector()
            vehicle.set_target_velocity(carla.Vector3D(forward.x * speed, forward.y * speed, 0))
        sync.tick()
        if cutin:
            lead.set_simulate_physics(False)
            cutin_start = lead.get_transform()
            forward = cutin_start.get_forward_vector()
            right = cutin_start.get_right_vector()
            ego_start = ego.get_location()
            lateral = ((ego_start.x - cutin_start.location.x) * right.x
                       + (ego_start.y - cutin_start.location.y) * right.y)

        aeb = AEBController(predict_cutin=predict_cutin)
        body_length = ego.bounding_box.extent.x + lead.bounding_box.extent.x
        previous_lead_location = None
        for tick in range(120):
            seconds = round(tick * STEP, 3)
            ego_speed = ego.get_velocity().length()
            lead_speed = lead.get_velocity().length()
            ego_pose = ego.get_transform()
            lead_location = lead.get_location()
            lead_waypoint = road.get_waypoint(lead_location)
            center_distance = ego_pose.location.distance(lead_location)
            dx = lead_location.x - ego_pose.location.x
            dy = lead_location.y - ego_pose.location.y
            ego_forward = ego_pose.get_forward_vector()
            ego_right = ego_pose.get_right_vector()
            velocity_overrides = None
            if cutin and previous_lead_location is not None:
                lead_velocity = carla.Vector3D(
                    (lead_location.x - previous_lead_location.x) / STEP,
                    (lead_location.y - previous_lead_location.y) / STEP, 0.0,
                )
                velocity_overrides = {lead.id: lead_velocity}
                lead_speed = lead_velocity.length()
            previous_lead_location = lead_location
            distance, closing_speed = aeb.detect_front_vehicle(ego, world, velocity_overrides=velocity_overrides)
            ttc = aeb.calculate_ttc(distance, closing_speed) if distance is not None else math.inf
            requested = aeb.update_state(distance, closing_speed)
            brake = requested if enabled else 0.0
            ego.apply_control(carla.VehicleControl(
                throttle=0.0 if brake else min(0.7, max(0.0, 0.2 + 0.25 * (ego_speed_mps - ego_speed))),
                brake=brake,
            ))
            if cutin:
                fraction = min(1.0, max(0.0, (seconds - 1.0) / cutin_duration_s))
                location = carla.Location(
                    x=cutin_start.location.x + forward.x * 8.0 * seconds + right.x * lateral * fraction,
                    y=cutin_start.location.y + forward.y * 8.0 * seconds + right.y * lateral * fraction,
                    z=cutin_start.location.z)
                lead.set_transform(carla.Transform(location, cutin_start.rotation))
            else:
                lead.apply_control(carla.VehicleControl(
                    throttle=min(0.7, max(0.0, 0.2 + 0.25 * (SPEED - lead_speed))) if seconds < 1.0 else 0.0,
                    brake=0.0 if seconds < 1.0 else 1.0,
                ))
            rows.append(dict(time_s=seconds, frame=world.get_snapshot().frame,
                             ego_speed_mps=ego_speed, lead_speed_mps=lead_speed,
                             lead_id=lead.id, aeb_target_id=aeb.last_target_id,
                             predicted_corridor_entry_s=aeb.last_target_entry_s,
                             lead_lane_id=lead_waypoint.lane_id,
                             lead_longitudinal_m=dx * ego_forward.x + dy * ego_forward.y,
                             lead_lateral_m=dx * ego_right.x + dy * ego_right.y,
                             lead_gap_m=None if adjacent or cutin else center_distance - body_length,
                             detected_distance_m=distance,
                             closing_speed_mps=closing_speed, ttc_s=ttc, state=aeb.state,
                             brake=brake, sumo_background=len(sync.sumo2carla_ids)))
            sync.tick()

        first_brake = next((r for r in rows if r["brake"] > 0), None)
        first_collision_frame = min((frame for frame, _ in collisions), default=None)
        pre_collision = next((r for r in reversed(rows)
                              if first_collision_frame is not None and r["frame"] < first_collision_frame), None)
        return rows, {
            "scenario": "kinematic_cutin" if cutin else ("adjacent_lane_braking" if adjacent else "same_lane_braking"),
            "aeb_enabled": enabled,
            "target_detection": "predictive" if predict_cutin else "overlap_only",
            "first_aeb_brake_s": first_brake["time_s"] if first_brake else None,
            "brake_target_is_lead": first_brake["aeb_target_id"] == lead.id if first_brake else None,
            "corridor_entry_prediction_at_brake_s": first_brake["predicted_corridor_entry_s"] if first_brake else None,
            "lead_brake_s": None if cutin else 1.0,
            "cutin_model": "kinematic, physics disabled, velocity estimated from successive positions" if cutin else None,
            "cutin_start_s": 1.0 if cutin else None,
            "cutin_initial_longitudinal_offset_m": cutin_offset if cutin else None,
            "lead_collision": any(other == lead.id for _, other in collisions),
            "ego_collision_events": len(collisions),
            "non_lead_collision_events": sum(other != lead.id for _, other in collisions),
            "first_collision_s": round((first_collision_frame - rows[0]["frame"]) * STEP, 3)
            if first_collision_frame is not None else None,
            "ego_speed_before_first_collision_kmh": round(pre_collision["ego_speed_mps"] * 3.6, 2)
            if pre_collision else None,
            "min_lead_gap_m": None if adjacent or cutin else round(min(r["lead_gap_m"] for r in rows), 3),
            "adjacent_lane_preserved": all(r["lead_lane_id"] == target_waypoint.lane_id for r in rows)
            if adjacent else None,
            "adjacent_target_selected": any(r["aeb_target_id"] == lead.id for r in rows)
            if adjacent else None,
            "peak_sumo_background": max(r["sumo_background"] for r in rows),
            "target_selected_before_cutin": any(r["aeb_target_id"] == lead.id for r in rows if r["time_s"] < 1.0) if cutin else None,
            "first_target_detection_s": next((r["time_s"] for r in rows if r["aeb_target_id"] == lead.id), None),
            "sumo_end_time_s": traci.simulation.getTime(),
        }
    finally:
        for actor in reversed(owned):
            if actor.is_alive:
                actor.destroy()
        if sync is not None:
            sync.close()
            sim.client.get_trafficmanager().set_synchronous_mode(False)
        else:
            sumo.close()


def sweep_pairs():
    for offset, speed, duration in product(CUTIN_OFFSETS, (10.0, 12.0, 14.0), (1.5, 2.0)):
        yield f"cutin_{offset:g}m_{speed:g}mps_{duration:g}s", {
            "enabled": True, "cutin_offset": offset, "ego_speed_mps": speed,
            "cutin_duration_s": duration,
        }


def summarize_sweep(results):
    outcomes = {
        (False, False): "neither_collided",
        (True, False): "collision_avoided",
        (True, True): "both_collided",
        (False, True): "collision_introduced",
    }
    pairs = []
    for name, parameters in sweep_pairs():
        baseline = results[name + "_overlap_only"]
        predictive = results[name + "_predictive"]
        valid = all(run["peak_sumo_background"] and not run["non_lead_collision_events"]
                    for run in (baseline, predictive))
        pairs.append({
            "case": name,
            "initial_offset_m": parameters["cutin_offset"],
            "ego_target_speed_mps": parameters["ego_speed_mps"],
            "cutin_duration_s": parameters["cutin_duration_s"],
            "outcome": outcomes[baseline["lead_collision"], predictive["lead_collision"]]
            if valid else "invalid",
            "baseline_first_brake_s": baseline["first_aeb_brake_s"],
            "predictive_first_brake_s": predictive["first_aeb_brake_s"],
            "baseline_first_collision_s": baseline["first_collision_s"],
            "predictive_first_collision_s": predictive["first_collision_s"],
            "baseline_ego_speed_before_collision_kmh": baseline["ego_speed_before_first_collision_kmh"],
            "predictive_ego_speed_before_collision_kmh": predictive["ego_speed_before_first_collision_kmh"],
        })
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", action="store_true", help="Run the paired cut-in parameter matrix.")
    sweep = parser.parse_args().sweep
    sim = CarlaSimulation("127.0.0.1", 2000, STEP)
    sim.client.set_timeout(20.0)
    world = sim.world
    if world.get_map().name.split("/")[-1] != "Town04":
        raise RuntimeError("Load Town04 before starting the regression.")
    initial_settings = world.get_settings()
    if initial_settings.synchronous_mode:
        raise RuntimeError("CARLA already has a synchronous session.")
    settings = world.get_settings()
    settings.no_rendering_mode = True
    output = OUTPUT / "sweep" if sweep else OUTPUT
    output.mkdir(parents=True, exist_ok=True)
    cases = {} if sweep else {
        "same_lane_off": {"enabled": False},
        "same_lane_on": {"enabled": True},
        "adjacent_lane": {"enabled": True, "adjacent": True},
    }
    cutins = sweep_pairs() if sweep else (
        (f"cutin_{offset:g}m", {"enabled": True, "cutin_offset": offset}) for offset in CUTIN_OFFSETS
    )
    for name, parameters in cutins:
        for label, predict in (("overlap_only", False), ("predictive", True)):
            cases[f"{name}_{label}"] = dict(parameters, predict_cutin=predict)
    results = {}
    try:
        world.apply_settings(settings)
        for name, parameters in cases.items():
            sim = CarlaSimulation("127.0.0.1", 2000, STEP)
            sim.client.set_timeout(20.0)
            rows, result = run_case(sim, **parameters)
            with (output / (name + ".csv")).open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            results[name] = result
            print(f"{name}: first brake {result['first_aeb_brake_s']} s, "
                  f"collision events {result['ego_collision_events']}", flush=True)
    finally:
        world.apply_settings(initial_settings)
    report = {
        "carla_version": sim.client.get_server_version(), "map": "Town04",
        "step_s": STEP, "bridge_seed": SEED, "cases": results,
    }
    if sweep:
        report["pairs"] = summarize_sweep(results)
        with (output / "comparison.csv").open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(report["pairs"][0]))
            writer.writeheader()
            writer.writerows(report["pairs"])
    else:
        report["initial_ego_speed_mps"] = SPEED
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Summary: {output / 'summary.json'}")
    if sweep:
        return

    off, on, adjacent = (results[name] for name in ("same_lane_off", "same_lane_on", "adjacent_lane"))
    if (not all(case["peak_sumo_background"] for case in results.values()) or not off["lead_collision"]
            or on["ego_collision_events"] or not on["brake_target_is_lead"]
            or on["first_aeb_brake_s"] < on["lead_brake_s"] or on["min_lead_gap_m"] <= 0):
        raise RuntimeError("AEB comparison did not meet the regression criteria")
    if (not adjacent["adjacent_lane_preserved"] or adjacent["adjacent_target_selected"]
            or adjacent["first_aeb_brake_s"] is not None or adjacent["ego_collision_events"]):
        raise RuntimeError("Adjacent-lane case did not meet the regression criteria")
    for offset in CUTIN_OFFSETS:
        baseline = results[f"cutin_{offset:g}m_overlap_only"]
        predictive = results[f"cutin_{offset:g}m_predictive"]
        if (predictive["target_selected_before_cutin"] or not predictive["brake_target_is_lead"]
                or not baseline["brake_target_is_lead"]
                or not 1.0 <= predictive["first_aeb_brake_s"] < baseline["first_aeb_brake_s"]):
            raise RuntimeError(f"Cut-in at {offset:g} m did not trigger earlier braking.")
        # The 6 m case measures mitigation at a short offset, including any collision.
        if offset != 6.0 and predictive["ego_collision_events"]:
            raise RuntimeError(f"Unexpected predictive cut-in collision at {offset:g} m.")
    if not results["cutin_12m_overlap_only"]["lead_collision"]:
        raise RuntimeError("The 12 m overlap-only collision was not reproduced.")


if __name__ == "__main__":
    main()
