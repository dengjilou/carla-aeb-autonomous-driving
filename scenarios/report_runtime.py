"""Audit native/ROS closed-loop logs against the frozen Python baseline."""
import argparse
import json
import math
from pathlib import Path
from report_integrated import differences
from report_longitudinal import replay

ROOT=Path(__file__).resolve().parents[1]


def audit(root,backend):
    previous=json.loads((ROOT/'docs/integrated_reference.json').read_text())['stanley']
    curved=json.loads((ROOT/'docs/curved_reference.json').read_text())
    result=dict(backend=backend,step_s=.05,command_tolerance=1e-9,groups={})
    for name,baseline in [('straight',previous),('curved',curved['route']),('stress',curved['aeb-route'])]:
        folder=root/(backend+'-'+name)
        first=json.loads((folder/'summary.json').read_text())
        repeat_folder=root/(backend+'-'+name+'-repeat')
        second=json.loads((repeat_folder/'summary.json').read_text())
        if set(first['runs'])!=set(baseline['runs']) or set(second['runs'])!=set(first['runs']):
            raise ValueError('Incomplete cases: '+name)
        errors=[]
        durations=[]
        frames=0
        for path,data in [(folder,first),(repeat_folder,second)]:
            for case,run in data['runs'].items():
                rows=replay(path/(case+'.csv'),run)
                for row in rows:
                    error=float(row['backend_error_max'])
                    duration=float(row['backend_compute_us'])
                    if not math.isfinite(error) or error>1e-9:
                        raise ValueError('Command parity failed '+case)
                    if float(row['latched_brake'])>0:
                        assert float(row['throttle'])==0
                        assert float(row['brake'])>=float(row['latched_brake'])
                    assert int(row['sumo_background'])>0
                    errors.append(error)
                    durations.append(duration)
                frames+=len(rows)
        changes=differences(baseline,first)
        repeated=differences(first,second)
        if changes or repeated:
            raise ValueError('Safety/event regression: '+str(changes+repeated))
        result['groups'][name]=dict(first=first,repeat=second,baseline_changes=changes,
            repeat_changes=repeated,frames=frames,command_abs_error_max=max(errors),
            compute_us_median=sorted(durations)[len(durations)//2],
            compute_us_p95=sorted(durations)[math.ceil(.95*len(durations))-1])
    result['total_runs']=sum(2*len(g['first']['runs']) for g in result['groups'].values())
    result['total_frames']=sum(g['frames'] for g in result['groups'].values())
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'output')
    parser.add_argument('--backend',choices=('native','ros'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=audit(args.input,args.backend)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='groups'}))
    print({k:{f:v for f,v in g.items() if f not in ('first','repeat')} for k,g in report['groups'].items()})


if __name__=='__main__':
    main()
