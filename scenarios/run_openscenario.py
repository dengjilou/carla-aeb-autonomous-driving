"""Run the pinned ScenarioRunner example and observe real lead speed."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    import carla
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-speed", type=float, default=10)
    parser.add_argument("--trigger", type=float, default=4)
    parser.add_argument("--output", type=Path, default=Path("output/openscenario"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    sr = Path(os.environ["SCENARIO_RUNNER_ROOT"])
    args.output.mkdir(parents=True, exist_ok=True)
    client = carla.Client("localhost", 2000)
    client.set_timeout(10)
    world = client.get_world()
    if not world.get_map().name.endswith("Town04") or world.get_settings().synchronous_mode or list(world.get_actors().filter("vehicle.*")):
        raise RuntimeError("ScenarioRunner requires an idle asynchronous Town04")
    command = [sys.executable, str(sr / "scenario_runner.py"),
               "--openscenario", str(root / "scenarios/openscenario/lead_slowdown.xosc"),
               "--openscenarioparams", "initialLeadSpeed:{},triggerTime:{}".format(args.initial_speed, args.trigger),
               "--sync", "--timeout", "30", "--output", "--json", "--outputDir", str(args.output.resolve())]
    rows = []
    started = time.time()
    with (args.output / "runner.log").open("w", encoding="utf-8") as log:
        process = subprocess.Popen(command, cwd=str(sr), stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 120
        try:
            while process.poll() is None:
                if time.monotonic() > deadline:
                    raise TimeoutError("ScenarioRunner exceeded 120 s")
                try:
                    snapshot = world.wait_for_tick(1)
                    lead = next((a for a in world.get_actors().filter("vehicle.*")
                                 if a.attributes.get("role_name") == "adversary"), None)
                    if lead:
                        rows.append(dict(frame=snapshot.frame, world_time_s=snapshot.timestamp.elapsed_seconds,
                                         lead_speed_mps=lead.get_velocity().length()))
                except RuntimeError:
                    # The runner can finish while this observer is waiting for a tick.
                    continue
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=15)
    with (args.output / "observed.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=("frame", "world_time_s", "lead_speed_mps"))
        writer.writeheader()
        writer.writerows(rows)
    runner_reports = [p for p in args.output.glob("*.json")
                      if p.name != "observation.json" and p.stat().st_mtime >= started]
    success = any(json.loads(p.read_text(encoding="utf-8")).get("success") is True for p in runner_reports)
    result = dict(returncode=process.returncode, scenario_success=success, parameters=vars(args).copy(), samples=len(rows),
                  max_speed_mps=max((r["lead_speed_mps"] for r in rows), default=None),
                  final_speed_mps=rows[-1]["lead_speed_mps"] if rows else None)
    result["parameters"]["output"] = str(args.output)
    (args.output / "observation.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    if process.returncode or not success or len(rows) < 10 or result["max_speed_mps"] < args.initial_speed - 2 or not 2 < result["final_speed_mps"] < 6:
        raise SystemExit("Scenario execution/speed-change verification failed; inspect runner.log and observed.csv")
    print(result)


if __name__ == "__main__":
    main()
