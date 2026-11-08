"""Record the held-out following comparison from real CARLA RGB frames."""
import json
import pygame
import imageio_ffmpeg
from run_longitudinal import ROOT, load_config, run_case


def main():
    import carla
    config = load_config(ROOT / "configs/longitudinal.json")
    case = next(c for c in config["cases"] if c["name"] == "follow_holdout")
    cfg = dict(case, step_s=config["step_s"], seed=config["seed"])
    output = ROOT / "output/longitudinal-video"
    output.mkdir(parents=True, exist_ok=True)
    captures = []
    for mode in ("baseline", "acc"):
        frames = {}
        def attach(world, ego):
            bp = world.get_blueprint_library().find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", "480")
            bp.set_attribute("image_size_y", "270")
            camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=-9, z=5),
                                       carla.Rotation(pitch=-15)), attach_to=ego)
            camera.listen(lambda image: frames.__setitem__(image.frame, bytes(image.raw_data)))
            return camera
        rows, events = run_case(cfg, mode, config["gains"], camera=attach)
        captures.append((rows[:-10], frames, events))
    pygame.init()
    size = (1008, 464)
    canvas = pygame.Surface(size)
    font = pygame.font.Font(None, 27)
    writer = imageio_ffmpeg.write_frames(str(output / "acc_comparison.mp4"), size,
                                        fps=20, codec="libx264", pix_fmt_in="rgb24", quality=7)
    writer.send(None)
    try:
        for tick in range(min(len(c[0]) for c in captures)):
            canvas.fill((14, 24, 37))
            canvas.blit(font.render("CARLA-SUMO | Held-out following | Real simulator frames", True, (230, 238, 245)), (18, 15))
            for x, title, (rows, frames, events) in zip((12, 508), ("Constant speed + AEB", "Time-headway ACC + AEB"), captures):
                row = rows[tick]
                if row["frame"] not in frames:
                    raise RuntimeError("Missing RGB frame {}".format(row["frame"]))
                frame = pygame.image.frombuffer(frames[row["frame"]], (480, 270), "BGRA")
                canvas.blit(frame, (x, 80))
                for y, text in [(51, title), (365, "Time {:.2f}s | Ego {:.1f} km/h".format(row["time_s"], row["ego_speed_mps"] * 3.6)),
                                (395, "Gap {:.1f}m | Brake {:.0%}".format(row["gap_m"], row["brake"])),
                                (425, "Emergency intervention: {}".format(row["latched_brake"] > 0))]:
                    canvas.blit(font.render(text, True, (220, 235, 240)), (x, y))
            writer.send(pygame.image.tostring(canvas, "RGB"))
        pygame.image.save(canvas, str(output / "acc_comparison.png"))
    finally:
        writer.close()
        pygame.quit()
    (output / "capture.json").write_text(json.dumps({
        "config": cfg, "gains": config["gains"], "frames_per_mode": len(captures[0][0]),
        "last_rows": [c[0][-1] for c in captures], "collision_events": [c[2] for c in captures],
    }, indent=2) + "\n", encoding="utf-8")
    print(output / "acc_comparison.mp4")


if __name__ == "__main__":
    main()
