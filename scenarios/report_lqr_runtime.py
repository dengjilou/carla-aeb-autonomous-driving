"""Audit fixed-gain Eigen/ROS LQR migration against actual Python LQR runs."""
import argparse
import json
import math
from pathlib import Path
from report_integrated import differences
from report_longitudinal import replay

ROOT=Path(__file__).resolve().parents[1]


def audit(root):
    evidence=dict(step_s=.05,command_tolerance=1e-9,groups={})
    for group,count in [('straight',10),('curved',7),('stress',1)]:
        runs={}
        stats={}
        for label in ('python','cpp','cpp-repeat','ros','ros-repeat'):
            backend,suffix=(label.split('-')[0],'-repeat' if label.endswith('repeat') else '')
            folder=root/(backend+'-'+group+suffix)
            data=json.loads((folder/'summary.json').read_text())
            assert len(data['runs'])==count
            frames=0
            errors=[]
            for name,run in data['runs'].items():
                rows=replay(folder/(name+'.csv'),run)
                frames+=len(rows)
                for row in rows:
                    error=float(row['backend_error_max'])
                    assert math.isfinite(error) and error<=1e-9
                    assert int(row['sumo_background'])>0
                    if float(row['latched_brake'])>0:
                        assert float(row['throttle'])==0 and float(row['brake'])>=float(row['latched_brake'])
                    errors.append(error)
            runs[label]=data
            stats[label]=dict(frames=frames,command_abs_error_max=max(errors))
        changes={label:differences(runs['python'],data) for label,data in runs.items() if label!='python'}
        repeats={label:differences(runs[label],runs[label+'-repeat']) for label in ('cpp','ros')}
        assert not any(changes.values()),changes
        assert not any(repeats.values()),repeats
        evidence['groups'][group]=dict(runs=runs,stats=stats,python_changes=changes,repeat_changes=repeats)
    old=json.loads((ROOT/'docs/integrated_reference.json').read_text())['lqr']
    evidence['historical_straight_changes']=differences(old,evidence['groups']['straight']['runs']['python'])
    assert not evidence['historical_straight_changes']
    evidence['total_runs']=90
    evidence['total_frames']=sum(s['frames'] for g in evidence['groups'].values() for s in g['stats'].values())
    return evidence


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'output/lqr-runtime')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    data=audit(args.input)
    args.output.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in data.items() if k!='groups'}))
    print({g:v['stats'] for g,v in data['groups'].items()})


if __name__=='__main__':
    main()
