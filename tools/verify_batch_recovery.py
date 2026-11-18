"""Exercise batch recovery against an idle CARLA Town04 and local SUMO."""
import argparse
import json
from pathlib import Path
import shutil
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scenarios"))
import run_longitudinal as runner


def check_events(reference, recovered):
    keys = ("valid", "verdict", "collided", "first_lead_selected_s", "event_observed_s",
            "first_aeb_s", "first_collision_s")
    assert reference.keys() == recovered.keys()
    for name in reference:
        for key in keys:
            expected = reference[name]["metrics"][key]
            actual = recovered[name]["metrics"][key]
            if expected != actual:
                assert (key.endswith("_s") and None not in (expected, actual)
                        and abs(expected - actual) <= 0.050001), (name, key, expected, actual)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="New directory for independent runs")
    folder = parser.parse_args().output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    config = runner.load_config(ROOT / "configs/longitudinal.json")
    names = {"follow_holdout", "hard_brake_holdout", "short_cutin_failure"}
    config["cases"] = [c for c in config["cases"] if c["name"] in names]
    config_path = folder / "cases.json"
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")

    def run(directory):
        args = ["runner", "--resume", "--config", str(config_path),
                "--output", str(folder / directory), "--mode", "acc"]
        with patch.object(sys, "argv", args):
            runner.main()
        return json.loads((folder / directory / "summary.json").read_text())["runs"]

    full = run("full")
    simulate = runner.run_case

    class InjectedInterrupt(KeyboardInterrupt):
        pass

    def stop_before_braking(case, *args, **kwargs):
        if case["kind"] == "hard_brake":
            raise InjectedInterrupt()
        return simulate(case, *args, **kwargs)

    try:
        with patch.object(runner, "run_case", stop_before_braking):
            run("resumed")
    except InjectedInterrupt:
        pass
    else:
        raise AssertionError("Expected interruption at the hard-brake case")

    first = folder / "resumed/follow_holdout_acc.csv"
    original_hash = runner.digest(first)
    check_events(full, run("resumed"))
    assert runner.digest(first) == original_hash

    damaged = folder / "damaged"
    shutil.copytree(str(folder / "resumed"), str(damaged))
    target = damaged / "short_cutin_failure_acc.csv"
    with target.open("ab") as stream:
        stream.write(b"corrupt")
    with patch.object(runner, "run_case", wraps=simulate) as calls:
        check_events(full, run("damaged"))
        assert calls.call_count == 1 and calls.call_args[0][0]["name"] == "short_cutin_failure"
    restored = {path.name: runner.digest(path) for path in damaged.glob("*.csv")}
    with patch.object(runner, "run_case", side_effect=AssertionError("Unexpected rerun")):
        run("damaged")
        run("damaged")
    assert restored == {path.name: runner.digest(path) for path in damaged.glob("*.csv")}
    print("Verified recovery, valid function failures, selective repair and repeated resume.")


if __name__ == "__main__":
    main()
