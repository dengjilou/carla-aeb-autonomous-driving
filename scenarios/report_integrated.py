"""Offline integration report and regression audit from original frame logs."""
import argparse
import json
from pathlib import Path
from report_longitudinal import replay


def differences(first, second):
    changed=[]
    for name,run in second['runs'].items():
        a,b=first['runs'][name]['metrics'],run['metrics']
        fields=['valid','verdict','collided','aeb_for_other_target','resumed_cruise']
        fields += ['first_aeb_s','first_collision_s','first_lead_selected_s','event_observed_s']
        for field in fields:
            x,y=a[field],b[field]
            if isinstance(x,float) and isinstance(y,float):
                unequal=abs(x-y)>.050001
            else:
                unequal=x!=y
            if unequal:
                changed.append(dict(case=name,metric=field,before=x,after=y))
    return changed


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    args=parser.parse_args()
    evidence={}
    for label in ('legacy','stanley','repeat','lqr'):
        folder=args.input/label
        data=json.loads((folder/'summary.json').read_text())
        expected=10 if label=='lqr' else 20
        assert len(data['runs'])==expected
        for name,run in data['runs'].items():
            if 'metrics' not in run:
                raise ValueError('Unfinished run '+label+'/'+name)
            rows=replay(folder/(name+'.csv'),run)
            assert all(float(r['latched_brake'])==0 or
                       (float(r['throttle'])==0 and float(r['brake'])>=float(r['latched_brake']))
                       for r in rows)
            assert all(float(r['sumo_background'])>0 for r in rows)
        evidence[label]=data
    evidence['legacy_changes']=differences(evidence['legacy'],evidence['stanley'])
    historical=json.loads((Path(__file__).resolve().parents[1]/'docs/longitudinal_reference.json').read_text())
    evidence['historical_changes']=differences(historical,evidence['legacy'])
    evidence['repeat_changes']=differences(evidence['stanley'],evidence['repeat'])
    lqr_base={'runs':{n:r for n,r in evidence['stanley']['runs'].items() if n.endswith('_acc')}}
    evidence['lqr_changes']=differences(lqr_base,evidence['lqr'])
    prior=json.loads((Path(__file__).resolve().parents[1]/'docs/cosim_aeb_reference.json').read_text())['cases']
    current=json.loads((args.input/'aeb/summary.json').read_text())['cases']
    assert len(current)==9
    evidence['aeb_9_changes']=[]
    for name,row in current.items():
        for field in ('lead_collision','first_aeb_brake_s','first_collision_s'):
            if row[field]!=prior[name][field]:
                evidence['aeb_9_changes'].append(dict(case=name,metric=field,before=prior[name][field],after=row[field]))
    evidence['aeb_9']=current
    evidence['video']=json.loads((args.input/'video/capture.json').read_text())
    lines=['# CARLA-SUMO integrated regression','',
           'Town04 straight-road scenes, 50 ms step, truth-state targets. Physics lead for follow/braking; kinematic cut-in/out.',
           'Path errors use the fixed reference route before first contact. AEB retains priority and stopping latch.','',
           '| Scene | Result | Collision | AEB s | Gap RMSE m | Path peak m |',
           '|---|---|---|---|---|---|']
    for name,run in evidence['stanley']['runs'].items():
        m=run['metrics']
        values=[name,m['verdict'],m['collided'],m['first_aeb_s'],m['steady_gap_rmse_m'],m['path_peak_m']]
        lines.append('| '+' | '.join('{:.4f}'.format(v) if isinstance(v,float) else str(v) for v in values)+' |')
    checks=('historical_changes','legacy_changes','repeat_changes','lqr_changes','aeb_9_changes')
    for key in checks:
        lines += ['', '## '+key,'',json.dumps(evidence[key],ensure_ascii=False,indent=2)]
    lines += ['', '## Camera recording','', '[Six-scene actual camera video](video/integrated_demo.mp4)']
    (args.input/'report.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    (args.input/'reference.json').write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    print('Changes:',{k:len(evidence[k]) for k in checks})
    print(args.input/'report.md')
    if any(evidence[k] for k in checks):
        raise SystemExit("Regression differences recorded. Inspect report.md before publishing.")


if __name__=='__main__':
    main()
