"""Audit clocks and actual controls before comparing braking physics settings."""
import argparse
import hashlib
import json
from pathlib import Path

from report_independent_aeb import comfort
from report_longitudinal import replay

ROOT = Path(__file__).resolve().parents[1]
GROUPS = ("a", "b", "substep-a", "substep-b", "substep3125-a", "substep3125-b",
          "holdout-a", "holdout-b")


def audit_samples(rows, config):
    dt = config["step_s"]
    expected_substep = config.get("physics_substep_s", .01)
    for i, row in enumerate(rows):
        assert int(row["frame"]) == int(row["snapshot_frame"]), "Snapshot frame mismatch"
        assert abs(float(row["time_s"]) - i * dt) < 1e-6
        assert abs(float(row["delta_s"]) - dt) < 1e-6
        assert row["substepping"] == "True"
        assert 1 <= int(row["max_substeps"]) <= 16
        assert abs(float(row["max_substep_delta_time"]) - expected_substep) < 1e-9
        assert expected_substep * int(row["max_substeps"]) >= dt - 1e-9
        if i:
            previous = rows[i-1]
            assert int(row["frame"]) == int(previous["frame"]) + 1, "Skipped frame"
            assert abs(float(row["elapsed_s"]) - float(previous["elapsed_s"]) - dt) < 1e-5
            for name in ("brake", "throttle"):
                assert abs(float(row["applied_"+name]) - float(previous[name])) < 1e-6, "Control timing mismatch"
    acceleration = [(float(b["ego_speed_mps"]) - float(a["ego_speed_mps"])) / dt
                    for a, b in zip(rows, rows[1:])]
    peak = min(range(len(acceleration)), key=acceleration.__getitem__) + 1
    sample = rows[peak]
    vector_error = max(abs((float(b["velocity_"+axis+"_mps"]) - float(a["velocity_"+axis+"_mps"])) / dt
                           - float(b["acceleration_"+axis+"_mps2"]))
                       for a, b in zip(rows, rows[1:]) for axis in ("x", "y", "z"))
    return dict(peak_time_s=float(sample["time_s"]),
                peak_fd_deceleration_mps2=-min(acceleration),
                api_acceleration_at_peak_mps2=float(sample["longitudinal_acceleration_mps2"]),
                applied_brake_at_peak=float(sample["applied_brake"]),
                previous_requested_brake=float(rows[peak-1]["brake"]),
                vector_acceleration_max_error_mps2=vector_error,
                stopped_s=next((float(r["time_s"]) for r in rows
                               if float(r["time_s"]) > config["trigger_s"] and float(r["ego_speed_mps"]) < .1), None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "docs/brake_physics_manifest.json").read_text())
    raw = (ROOT / "configs/brake_physics_holdout.json").read_bytes().replace(b"\r\n", b"\n")
    assert hashlib.sha256(raw).hexdigest() == manifest["holdout_config_lf_sha256"]
    frozen = json.loads(raw)
    expected = {case["name"]+"_baseline": case for case in frozen["cases"]}
    results, traces, runs = {}, {}, {}
    for group in GROUPS:
        folder = args.input / group
        report = json.loads((folder / "summary.json").read_text())
        if group.startswith("holdout"):
            assert set(report["runs"]) == set(expected)
            assert report["gains"] == frozen["gains"]
        results[group], traces[group], runs[group] = {}, {}, report["runs"]
        for name, run in report["runs"].items():
            cfg = run["config"]
            assert cfg["step_s"] == .05 and cfg["brake_diagnostics"]
            assert cfg["backend"] == "python" and cfg["lateral_controller"] == "lqr"
            if group.startswith("holdout"):
                assert all(cfg[key] == value for key, value in expected[name].items())
            rows = replay(folder / (name+".csv"), run)
            traces[group][name] = rows
            endpoint = run["metrics"]["first_collision_s"] or cfg["duration_s"]
            results[group][name] = dict(metrics=run["metrics"], diagnostics=audit_samples(rows, cfg),
                                       full_precontact=comfort(rows, endpoint))
            if not group.startswith("holdout"):
                results[group][name]["fixed_4_35s_window"] = comfort(rows, 4.35)
    repeat_deltas = {}
    for left, right in zip(GROUPS[::2], GROUPS[1::2]):
        assert runs[left].keys() == runs[right].keys()
        for name, run in runs[left].items():
            other = runs[right][name]
            assert run["config"] == other["config"]
            a, b = run["metrics"], other["metrics"]
            assert all(a[key] == b[key] for key in ("valid", "verdict", "collided"))
            delta = {}
            for key in ("first_lead_selected_s", "first_aeb_s", "first_collision_s", "event_observed_s"):
                assert (a[key] is None) == (b[key] is None)
                delta[key] = abs(a[key]-b[key]) if a[key] is not None else 0
                assert delta[key] <= .0500001
            repeat_deltas[left+"/"+name] = delta
    args.output.mkdir(parents=True, exist_ok=True)
    output = dict(manifest=manifest, results=results, repeat_event_deltas=repeat_deltas)
    (args.output / "reference.json").write_text(json.dumps(output, indent=2, allow_nan=False)+"\n")
    if args.plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for group, label in (("a", "10 ms"), ("substep-a", "5 ms"), ("substep3125-a", "3.125 ms")):
            rows = traces[group]["brake_0_warning_baseline"]
            times = [float(row["time_s"]) for row in rows]
            axes[0].plot(times, [float(r["ego_speed_mps"]) for r in rows], label=label)
            axes[1].plot(times, [float(r["longitudinal_acceleration_mps2"]) for r in rows])
            axes[2].plot(times, [float(r["applied_brake"]) for r in rows])
        for ax, label in zip(axes, ("Speed (m/s)", "API longitudinal a (m/s²)", "Applied brake")):
            ax.set_ylabel(label)
            ax.axvline(4.35, color="black", linestyle="--", alpha=.5)
            ax.grid(alpha=.2)
        axes[0].legend()
        axes[1].set_ylim(-32, 5)
        axes[-1].set_xlim(2, 5.5)
        axes[-1].set_xlabel("Simulation time (s), same 50 ms control step")
        fig.tight_layout()
        fig.savefig(args.output / "substep_comparison.png", dpi=140)
    print("Verified", sum(len(r) for r in runs.values()), "runs and", len(repeat_deltas), "repeat pairs")


if __name__ == "__main__":
    main()
