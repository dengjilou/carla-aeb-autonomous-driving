"""Record real CARLA RGB frames with synchronized lateral-error telemetry."""
import json
import argparse
import queue
from pathlib import Path

import carla
import imageio_ffmpeg
import pygame

from run_lateral import CASES, DT, run, test_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument('--fixed', action='store_true')
    comparison.add_argument('--lqr', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('output/lateral-camera'))
    args = parser.parse_args()
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    client = carla.Client('localhost', 2000)
    client.set_timeout(90)
    original = client.get_world()
    name = original.get_map().name
    if original.get_settings().synchronous_mode or any(original.get_actors().filter(p) for p in ('vehicle.*','walker.*','sensor.*')):
        raise RuntimeError('Requires an idle stock map')
    captures = []
    summaries = []
    try:
        world = client.generate_opendrive_world(test_map())
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        road = world.get_map()
        points = [road.get_waypoint_xodr(len(CASES), -1, float(s)).transform
                  for s in range(1, 133)]
        for offset, color in ((0, carla.Color(0,230,200)), (-1.75, carla.Color(255,210,30)),
                              (1.75, carla.Color(255,210,30))):
            for first, second in zip(points, points[1:]):
                a = first.location + first.get_right_vector()*offset + carla.Location(z=0.08)
                b = second.location + second.get_right_vector()*offset + carla.Location(z=0.08)
                world.debug.draw_line(a,b,thickness=0.05,color=color,life_time=45)
        modes = ('stanley_fixed','pursuit','lqr') if args.lqr else ('stanley','stanley_fixed') if args.fixed else ('stanley','pursuit')
        for mode in modes:
            frames = []
            def attach(world, vehicle):
                incoming = queue.Queue()
                bp = world.get_blueprint_library().find('sensor.camera.rgb')
                bp.set_attribute('image_size_x','640')
                bp.set_attribute('image_size_y','360')
                bp.set_attribute('fov','90')
                cam = world.spawn_actor(bp, carla.Transform(carla.Location(x=-7,z=6),
                                         carla.Rotation(pitch=-30)), attach_to=vehicle)
                cam.listen(lambda img: incoming.put((img.frame, bytes(img.raw_data))))
                def collect(frame_id, row):
                    while True:
                        number, data = incoming.get(timeout=10)
                        if number == frame_id:
                            frames.append((data, row))
                            return
                        if number > frame_id:
                            raise RuntimeError('Skipped camera frame')
                return cam, collect
            summaries.append(run(world, road, CASES[-1], len(CASES), mode, output, attach, gain=2, lqr_weight=1))
            captures.append(frames)
    finally:
        client.load_world(name.rsplit('/',1)[-1])
    pygame.init()
    size = (640*len(captures), 480)
    display = pygame.display.set_mode(size)
    pygame.display.set_caption('CARLA lateral comparison | actual camera recording')
    font = pygame.font.Font(None, 28)
    writer = imageio_ffmpeg.write_frames(str(output/'lateral_comparison.mp4'), size, fps=20,
                                        codec='libx264', pix_fmt_in='rgb24', quality=7)
    writer.send(None)
    count = min(map(len,captures))
    snapshot = round(6.75/DT)
    try:
        for tick in range(count):
            display.fill((17,25,36))
            labels = ('Stanley k=2','Pure Pursuit','LQR Q=diag(1,1), R=1') if args.lqr else ('Original Stanley','Corrected Stanley (k=2)') if args.fixed else ('Stanley','Geometric Pure Pursuit')
            for x, label, frames in zip(range(0,size[0],640),labels,captures):
                data, row = frames[tick]
                display.blit(pygame.image.frombuffer(data,(640,360),'BGRA'), (x,42))
                for y, text in [(10,label+' | target 10 m/s'),
                                (410,'t={:.2f}s   speed={:.2f}m/s   error={:+.3f}m'.format(row[0],row[7],row[6])),
                                (442,'steer={:+.3f}   center outside lane: {}'.format(row[8],abs(row[6])>1.75))]:
                    display.blit(font.render(text,True,(235,240,245)), (x+12,y))
            pygame.display.flip()
            pygame.event.pump()
            writer.send(pygame.image.tostring(display,'RGB'))
            if tick==snapshot:
                pygame.image.save(display, str(output/'lateral_comparison.png'))
    finally:
        writer.close()
        pygame.quit()
    (output/'capture.json').write_text(json.dumps(dict(frames=count,step_s=DT,results=summaries),indent=2))
    print('Recorded {} camera frames per controller'.format(count))


if __name__ == '__main__':
    main()

