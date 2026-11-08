"""Record the 12 m Town04 cut-in comparison from live CARLA camera frames."""

import json

import imageio_ffmpeg
import pygame

from run_cosim_aeb import CarlaSimulation, OUTPUT, REPO, STEP, carla, run_case


SIZE = (1280, 720)
CAMERA_SIZE = (608, 342)
FPS = 30
LAST_TICK = 110  # Keep a margin for CARLA's asynchronous camera delivery.
TEXT = (229, 237, 247)
MUTED = (163, 182, 204)
AMBER = (255, 178, 87)
TEAL = (96, 227, 196)


def capture_case(predict):
    images = {}

    def attach_camera(world, ego):
        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(CAMERA_SIZE[0]))
        blueprint.set_attribute("image_size_y", str(CAMERA_SIZE[1]))
        blueprint.set_attribute("fov", "90")
        pose = carla.Transform(carla.Location(x=-8.5, z=5.2), carla.Rotation(pitch=-16))
        camera = world.spawn_actor(blueprint, pose, attach_to=ego)
        camera.listen(lambda image: images.__setitem__(image.frame, image))
        return camera

    sim = CarlaSimulation("127.0.0.1", 2000, STEP)
    sim.client.set_timeout(20)
    if sim.world.get_map().name.split("/")[-1] != "Town04" or sim.world.get_settings().no_rendering_mode:
        raise RuntimeError("The demo requires Town04 with CARLA rendering enabled.")
    rows, summary = run_case(sim, True, cutin_offset=12.0, predict_cutin=predict, attach_camera=attach_camera)
    rows = rows[:LAST_TICK + 1]
    if any(row["frame"] not in images for row in rows):
        raise RuntimeError("CARLA did not deliver every RGB frame for the demo.")
    return rows, summary, images


def compose(canvas, fonts, cases, tick, end=False):
    def label(value, x, y, size="body", color=TEXT):
        canvas.blit(fonts[size].render(value, True, color), (x, y))

    canvas.fill((10, 19, 32))
    label("CARLA + SUMO   /   AUTOMATED AEB COMPARISON", 24, 18, "small", TEAL)
    label("12 m cut-in on Town04", 24, 44, "title")
    time_s = cases[0][0][tick]["time_s"]
    label(f"SIM {time_s:04.2f} s   |   0.25x replay", 1000, 54, "body", MUTED)

    for x, (rows, summary, images), title, accent in zip(
        (24, 648), cases, ("OVERLAP-ONLY BASELINE", "PREDICTIVE DETECTION"), (AMBER, TEAL)
    ):
        row = rows[tick]
        pygame.draw.rect(canvas, (23, 37, 55), (x, 99, 608, 504), border_radius=8)
        label(title, x + 15, 110, "heading", accent)
        frame = images[row["frame"]]
        canvas.blit(pygame.image.frombuffer(frame.raw_data, CAMERA_SIZE, "BGRA"), (x, 142))
        lead_detected = row["aeb_target_id"] == row["lead_id"]
        brake = row["brake"]
        label(f"Ego {row['ego_speed_mps'] * 3.6:04.1f} km/h", x + 15, 496, "heading")
        label("Cut-in target: tracked" if lead_detected else "Cut-in target: searching",
              x + 15, 528, "body", accent if lead_detected else MUTED)
        label(f"AEB command: {brake:.0%}", x + 330, 528, "body", TEXT)
        pygame.draw.rect(canvas, (49, 65, 84), (x + 330, 554, 258, 8), border_radius=4)
        if brake:
            pygame.draw.rect(canvas, accent, (x + 330, 554, round(258 * brake), 8), border_radius=4)
        collision_time = summary["first_collision_s"]
        if collision_time is not None and time_s >= collision_time:
            status = f"COLLISION AT {collision_time:.2f} s"
        elif end:
            status = "NO COLLISION"
        elif time_s >= summary["first_aeb_brake_s"]:
            status = f"BRAKING STARTED AT {summary['first_aeb_brake_s']:.2f} s"
        elif time_s >= summary["cutin_start_s"]:
            status = "CUT-IN IN PROGRESS"
        else:
            status = "CUT-IN STARTS AT 1.00 s"
        label(status, x + 15, 571, "small", accent)

    pygame.draw.rect(canvas, (49, 65, 84), (24, 623, 1232, 4))
    pygame.draw.rect(canvas, TEAL, (24, 623, round(1232 * tick / LAST_TICK), 4))
    label("Same scenario, initial speeds and AEB thresholds", 24, 648, "heading")
    label("Blue: ego     Amber: cut-in vehicle     SUMO: background traffic", 24, 680, "body", MUTED)
    if end:
        baseline = cases[0][1]["first_aeb_brake_s"]
        prediction = cases[1][1]["first_aeb_brake_s"]
        label(f"First brake {baseline:.2f} s to {prediction:.2f} s", 948, 649, "heading", TEAL)


def main():
    cases = [capture_case(False), capture_case(True)]
    names = ("cutin_12m_overlap_only", "cutin_12m_predictive")
    reference = json.loads((REPO / "docs" / "cosim_aeb_reference.json").read_text(encoding="utf-8"))
    results = {name: case[1] for name, case in zip(names, cases)}
    for name, result in results.items():
        for metric in ("first_aeb_brake_s", "first_collision_s", "lead_collision"):
            if result[metric] != reference["cases"][name][metric]:
                raise RuntimeError(f"Rendered case {name} did not reproduce {metric}: {result[metric]}")
    if results["cutin_12m_predictive"]["ego_collision_events"]:
        raise RuntimeError("The predictive run recorded an ego collision.")

    OUTPUT.mkdir(parents=True, exist_ok=True)
    video = OUTPUT / "cutin_12m_comparison.mp4"
    pygame.font.init()
    fonts = {name: pygame.font.Font(None, size) for name, size in
             (("title", 42), ("heading", 28), ("body", 24), ("small", 22))}
    canvas = pygame.Surface(SIZE)
    writer = imageio_ffmpeg.write_frames(str(video), SIZE, fps=FPS, codec="libx264",
                                         quality=7, output_params=["-movflags", "+faststart"])
    writer.send(None)
    try:
        for tick in range(LAST_TICK + 1):
            compose(canvas, fonts, cases, tick, end=tick == LAST_TICK)
            pixels = pygame.image.tostring(canvas, "RGB")
            # 20 simulation frames/s displayed at quarter speed in a 30 fps MP4.
            copies = 3 * FPS if tick == 0 else (6 * FPS if tick == LAST_TICK else 6)
            for _ in range(copies):
                writer.send(pixels)
    finally:
        writer.close()
    (OUTPUT / "cutin_12m_demo.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(video)


if __name__ == "__main__":
    main()
