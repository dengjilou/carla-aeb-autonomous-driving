"""Regenerate a readable report; optional trace plots require matplotlib."""
import argparse
import csv
import json
from pathlib import Path
from longitudinal_metrics import evaluate


def replay(path, run):
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    typed = [dict(
        **{k: float(row[k]) for k in ("time_s", "aeb_brake", "cross_track_m", "gap_m", "ego_speed_mps",
                                     "lead_speed_mps", "lateral_clearance_m", "longitudinal_m")},
        **{k: int(row[k]) if row[k] else None for k in ("lead_id", "target_id", "sumo_background")},
        event_observed=row["event_observed"] == "True",
    ) for row in rows]
    if evaluate(typed, run["collisions"], run["config"]) != run["metrics"]:
        raise ValueError("Trace and summary disagree: " + str(path))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--traces", type=Path)
    parser.add_argument("--plots", action="store_true")
    args = parser.parse_args()
    if args.plots and not args.traces:
        parser.error("--plots requires --traces")
    data = json.loads(args.input.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=True)
    lines = ["# Longitudinal experiment report", "",
             "Ground-truth targets; fixed 50 ms step. Kinematic cut-in/out results are tagged in each configuration.",
             "Gap RMSE uses the last 5 seconds with the lead selected, before contact. "
             "Jerk uses speed finite differences, excludes the first second and contact frames.", "",
             "| Case | Verdict | Collision | AEB onset (s) | Gap RMSE (m) | Speed RMSE (m/s) | Jerk RMS (m/s³) |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for name, run in data["runs"].items():
        if "metrics" not in run:
            lines.append("| {} | ERROR | {} | | | | |".format(name, run.get("execution_error", "Interrupted")))
            continue
        m = run["metrics"]
        values = [name, m["verdict"], m["collided"], m["first_aeb_s"],
                  m["steady_gap_rmse_m"], m["steady_speed_rmse_mps"], m["jerk_rms_mps3"]]
        lines.append("| " + " | ".join("{:.3f}".format(v) if isinstance(v, float) else str(v) for v in values) + " |")
        if args.traces:
            rows = replay(args.traces / (name + ".csv"), run)
        if args.plots:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            t = [float(r["time_s"]) for r in rows]
            fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
            for field, label in (("ego_speed_mps", "Ego"), ("lead_speed_mps", "Lead"), ("target_speed_mps", "Speed reference")):
                axes[0].plot(t, [float(r[field]) for r in rows], label=label)
            axes[0].set_ylabel("Speed (m/s)")
            axes[1].plot(t, [float(r["gap_m"]) for r in rows], label="Projected bumper gap")
            axes[1].plot(t, [8 + 1.8 * float(r["ego_speed_mps"]) for r in rows], label="Desired gap")
            axes[1].set_ylabel("Distance (m)")
            for field in ("throttle", "brake"):
                axes[2].plot(t, [float(r[field]) for r in rows], label=field)
            axes[2].set_ylabel("Actuator command")
            axes[2].set_xlabel("Simulation time (s)")
            for ax in axes:
                ax.grid(alpha=.25)
                ax.legend()
                if m["first_collision_s"] is not None:
                    ax.axvline(m["first_collision_s"], color="red", linestyle="--")
            fig.suptitle(name)
            fig.tight_layout()
            fig.savefig(str(args.output / (name + ".png")), dpi=140)
            plt.close(fig)
    if args.plots:
        for name, run in data["runs"].items():
            if "metrics" in run:
                lines.extend(["", "## " + name, "", "![](" + name + ".png)"])
    (args.output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(args.output / "report.md")


if __name__ == "__main__":
    main()
