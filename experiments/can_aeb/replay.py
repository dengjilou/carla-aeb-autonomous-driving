"""Offline teaching CAN trace. No CAN interface or vehicle connection."""
import argparse
import csv
import io
import json
import sys
import zipfile
from pathlib import Path

import cantools

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "controllers"))
from acc import SpeedPID

MSG = cantools.database.load_file(HERE / "aeb.dbc").get_message_by_name("BrakeStatus")
PERIOD_MS = 50


def encode(rows):
    pid = SpeedPID()
    lines = []
    originals = []
    for n, row in enumerate(rows):
        speed = float(row["ego_speed_mps"])
        _, ordinary = pid.update(float(row["target_speed_mps"]), speed, PERIOD_MS/1000)
        final = float(row["brake"])
        if abs(max(ordinary, float(row["latched_brake"])) - final) > 1e-8:
            raise ValueError("Reconstructed PID disagrees with the post-arbitration brake")
        sig = dict(Speed=speed, OrdinaryBrake=ordinary, FinalBrake=final,
                   AebRequest=float(row["aeb_brake"]), AebLevel=int(row["aeb_level"]), Counter=n%16)
        payload = MSG.encode(sig)
        lines.append(f"({float(row['time_s']):.3f}) simcan0 {MSG.frame_id:03X}#{payload.hex().upper()}")
        originals.append(sig)
    return lines, originals


def decode(lines, observation_end_ms=None):
    previous = None
    first_aeb = None
    issues, signals = [], []
    for line in lines:
        stamp, _, message = line.split()
        frame_id, hexdata = message.split("#")
        ms = round(1000*float(stamp.strip("()")))
        if observation_end_ms is not None and ms > observation_end_ms:
            raise ValueError("Observation end precedes a logged frame")
        if int(frame_id,16) != MSG.frame_id:
            continue
        sig = MSG.decode(bytes.fromhex(hexdata))
        if previous is not None:
            delta = ms-previous[0]
            step = (sig["Counter"]-previous[1])%16
            labels = []
            if delta >= 3*PERIOD_MS:
                labels.append("timeout")
            if step == 0 and delta < PERIOD_MS:
                labels.append("duplicate")
            elif step != 1:
                labels.append("counter_gap")
            for reason in labels:
                issues.append(dict(id=frame_id,observed_ms=ms,reason=reason))
        previous = ms,sig["Counter"]
        if first_aeb is None and sig["AebRequest"] > 0:
            first_aeb = ms/1000
        signals.append(sig)
    if observation_end_ms is not None and previous is not None:
        deadline = previous[0] + 3*PERIOD_MS
        if observation_end_ms >= deadline:
            issues.append(dict(id=f"{MSG.frame_id:03X}",observed_ms=observation_end_ms,
                               deadline_ms=deadline,reason="terminal_timeout"))
    return dict(frames=len(signals),first_aeb_s=first_aeb,issues=issues),signals


def study(output):
    output.mkdir(parents=True,exist_ok=True)
    result={}
    with zipfile.ZipFile(HERE.parents[1]/"docs/independent_aeb_traces.zip") as archive:
        for run in ("a","b"):
            for policy in ("staged","warning"):
                name=f"{run}/brake_0_{policy}_baseline.csv"
                rows=list(csv.DictReader(io.StringIO(archive.read(name).decode())))
                lines,source=encode(rows)
                label=f"{run}_{policy}"
                (output/(label+".log")).write_text("\n".join(lines)+"\n")
                report,signals=decode(lines)
                report["max_abs_error"]={k:max(abs(a[k]-b[k]) for a,b in zip(signals,source))
                                         for k in source[0]}
                truth=next(float(r["time_s"]) for r in rows if float(r["aeb_brake"])>0)
                if report["issues"] or report["first_aeb_s"]!=truth or max(report["max_abs_error"].values())>.00500001:
                    raise ValueError(label)
                result[label]=report
                if label=="a_staged":
                    for fault, altered in dict(drop=lines[:9]+lines[10:],repeat=lines[:10]+lines[9:],
                                                timeout=lines[:9]+lines[12:]).items():
                        (output/(fault+".log")).write_text("\n".join(altered)+"\n")
                        result[fault]=decode(altered)[0]
                    # The complete source run supplies the observation end independently of the truncated trace.
                    end_ms=round(1000*float(rows[-1]["time_s"]))
                    truncated=lines[:-3]
                    (output/"terminal.log").write_text("\n".join(truncated)+"\n")
                    result["terminal"]=dict(observation_end_ms=end_ms,**decode(truncated,end_ms)[0])
    (output/"results.json").write_text(json.dumps(result,indent=2)+"\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    operation=p.add_mutually_exclusive_group(required=True)
    operation.add_argument("output",nargs="?",type=Path,help="Build the reference logs and report")
    operation.add_argument("--decode",type=Path,metavar="LOG",help="Audit a previously saved log on its own")
    p.add_argument("--end-ms",type=int,help="Independent observation end, on the same clock as the trace")
    args=p.parse_args()
    if args.decode is None:
        if args.end_ms is not None:
            p.error("--end-ms requires --decode")
        study(args.output)
    else:
        with args.decode.open() as log:
            report,_=decode(log,args.end_ms)
        print(json.dumps(report,indent=2))
