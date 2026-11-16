"""Matched CARLA lateral-controller experiments on purpose-built OpenDRIVE lanes."""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import carla
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers"))
from acc import SpeedPID
from stanley_pid_controller import StanleyController

DT = 0.05
# name, signed curvature (1/m) of the two 32 m segments, requested speed (m/s)
CASES = [
    ("straight", 0, 0, 8), ("arc", 1/65, 1/65, 8), ("s_bend", 1/55, -1/55, 8),
    ("holdout_arc", -1/52, -1/52, 10), ("holdout_s", -1/50, 1/42, 10),
]


def test_map():
    roads = []
    for i, (_, first, second, _) in enumerate(CASES, 1):
        x, y, heading, s = 0., i * 180., 0., 0.
        geoms = []
        for length, k in [(40, 0), (32, first), (32, second), (30, 0)]:
            kind = '<arc curvature="{}"/>'.format(k) if k else '<line/>'
            geoms.append(f'<geometry s="{s}" x="{x}" y="{y}" hdg="{heading}" length="{length}">{kind}</geometry>')
            end = heading + k * length
            x += (math.sin(end)-math.sin(heading))/k if k else length*math.cos(heading)
            y += (math.cos(heading)-math.cos(end))/k if k else length*math.sin(heading)
            heading, s = end, s + length
        lane = '<lane id="{}" type="driving" level="false"><link/><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>'
        roads.append(f'<road name="case{i}" length="{s}" id="{i}" junction="-1"><planView>{"".join(geoms)}</planView>'
                     '<lanes><laneSection s="0"><left>'+lane.format(1)+'</left><center><lane id="0" type="none" level="false"/>'
                     '</center><right>'+lane.format(-1)+lane.format(-2)+'</right></laneSection></lanes></road>')
    return '<OpenDRIVE><header revMajor="1" revMinor="4"/>'+''.join(roads)+'</OpenDRIVE>'


def project(xy, starts, vectors, lengths2):
    delta = xy - starts
    t = np.clip(np.sum(delta*vectors, axis=1)/lengths2, 0, 1)
    offsets = delta - t[:, None]*vectors
    index = int(np.argmin(np.sum(offsets**2, axis=1)))
    vx, vy = vectors[index]
    dx, dy = offsets[index]
    signed_distance = math.copysign(math.hypot(dx, dy), vx*dy-vy*dx)
    return index, t[index], signed_distance


def run(world, road, case, road_id, mode, folder, camera_hook=None):
    name, _, _, target_speed = case
    stations = np.arange(1., 133., 0.5)
    path = [road.get_waypoint_xodr(road_id, -1, float(s)) for s in stations]
    if any(w is None for w in path):
        raise RuntimeError("Missing reference lane waypoint")
    points = np.array([(w.transform.location.x, w.transform.location.y) for w in path])
    if mode == "stanley":
        np.savetxt(folder / (name + "_reference.csv"), np.column_stack((stations, points)),
                   delimiter=",", header="station_m,x,y", comments="")
    vectors = np.diff(points, axis=0)
    lengths2 = np.sum(vectors**2, axis=1)
    initial = path[2].transform
    initial.location += carla.Location(z=0.35)
    vehicle = world.spawn_actor(world.get_blueprint_library().find("vehicle.tesla.model3"), initial)
    camera = None
    try:
        capture = None
        if camera_hook:
            camera, capture = camera_hook(world, vehicle)
        for _ in range(12):
            world.tick()
        vehicle.set_target_velocity(initial.get_forward_vector()*target_speed)
        pid, stanley = SpeedPID(), StanleyController(k=0.3, max_steer=0.7)
        physics = vehicle.get_physics_control().wheels
        # 0.9.13 returns wheel positions in world centimetres. All starts face +x.
        rear_x = physics[2].position.x/100. - vehicle.get_location().x
        wheelbase = abs(physics[0].position.x - physics[2].position.x)/100.
        max_angle = math.radians(physics[0].max_steer_angle)
        if not 1 < wheelbase < 5 or not -3 < rear_x < 0 or max_angle <= 0:
            raise RuntimeError("Invalid vehicle geometry")
        stanley.steer_alpha = 1.0
        previous = 0.0
        rows = []
        for tick in range(int(32/DT)):
            transform = vehicle.get_transform()
            pos = transform.location
            speed = vehicle.get_velocity().length()
            idx, fraction, error = project(np.array([pos.x, pos.y]), points[:-1], vectors, lengths2)
            station = stations[idx] + fraction*0.5
            target = min(len(path)-1, idx + round((3 + 0.4*speed)/0.5))
            if mode == "stanley":
                steer = stanley.compute_steering_angle(vehicle, path[target])[0]
            else:
                # Geometric pure pursuit in a rear-axle frame; CARLA +y is right.
                axle = transform.transform(carla.Location(x=rear_x))
                dx, dy = points[target] - np.array([axle.x, axle.y])
                right = transform.get_right_vector()
                lateral = dx*right.x + dy*right.y
                steer = math.atan2(2*wheelbase*lateral, dx*dx + dy*dy)/max_angle
            steer = 0.4 * max(-0.7, min(0.7, steer)) + 0.6 * previous
            previous = steer
            throttle, brake = pid.update(target_speed, speed, DT)
            ref = points[idx] + fraction*vectors[idx]
            rows.append((tick*DT, station, pos.x, pos.y, *ref, error, speed, steer))
            if capture:
                capture(world.get_snapshot().frame, rows[-1])
            if station >= 121:
                break
            vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer))
            world.tick()
        with (folder / (name+'_'+mode+'.csv')).open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow('time_s station_m x y ref_x ref_y error_m speed_mps steer'.split())
            writer.writerows(rows)
        if station < 121:
            raise RuntimeError(name + " did not finish the route")
        active = np.array([r for r in rows if 35 <= r[1] <= 117])
        error, commands = active[:, 6], active[:, 8]
        outside = abs(error) > 1.75
        return dict(case=name, mode=mode, frames=len(active), rmse_m=float(np.sqrt(np.mean(error**2))),
                    p95_m=float(np.percentile(abs(error), 95)), peak_m=float(max(abs(error))),
                    center_exits=int(outside[0])+int(np.sum(outside[1:] & ~outside[:-1])),
                    steer_rate_rms=float(np.sqrt(np.mean((np.diff(commands)/DT)**2))),
                    mean_speed_mps=float(active[:, 7].mean()))
    finally:
        if camera:
            camera.stop()
            camera.destroy()
        vehicle.destroy()
        world.tick()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    folder = args.output
    folder.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(90)
    original = client.get_world()
    original_map = original.get_map().name
    actors = original.get_actors()
    if (original.get_settings().synchronous_mode or 'OpenDriveMap' in original_map
            or any(actors.filter(pattern) for pattern in ('vehicle.*', 'walker.*', 'sensor.*'))):
        raise RuntimeError("Requires an idle, asynchronous stock map")
    xml = test_map()
    (folder/'lateral.xodr').write_text(xml)
    try:
        world = client.generate_opendrive_world(xml)
        settings = world.get_settings()
        settings.fixed_delta_seconds = DT
        settings.synchronous_mode = True
        world.apply_settings(settings)
        road = world.get_map()
        results = []
        for i, case in enumerate(CASES, 1):
            for mode in ('stanley', 'pursuit'):
                result = run(world, road, case, i, mode, folder)
                results.append(result)
                print(result, flush=True)
        (folder/'summary.json').write_text(json.dumps(results, indent=2))
    finally:
        client.load_world(original_map.rsplit('/', 1)[-1])


if __name__ == '__main__':
    main()
