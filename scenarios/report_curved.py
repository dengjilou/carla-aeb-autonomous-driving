"""Offline curved-road validation, input checks and repeat/regression comparison."""
import argparse
import json
from pathlib import Path
from report_longitudinal import replay
from report_integrated import differences


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    root=parser.parse_args().input
    evidence={}
    for label in ('corridor','route','repeat','aeb-corridor','aeb-route','aeb-repeat','straight'):
        data=json.loads((root/label/'summary.json').read_text())
        expected=20 if label=='straight' else 1 if label.startswith('aeb-') else 7
        assert len(data['runs'])==expected
        for name,run in data['runs'].items():
            rows=replay(root/label/(name+'.csv'),run)
            assert all(float(r['latched_brake'])==0 or
                       (float(r['throttle'])==0 and float(r['brake'])>=float(r['latched_brake']))
                       for r in rows)
            if run['config'].get('curved'):
                assert run['metrics']['valid'],(label,name,run['metrics']['invalid_reasons'])
                if run['config']['kind']=='hard_brake':
                    onset=next((float(r['time_s']) for r in rows if
                                float(r['time_s'])>run['config']['trigger_s'] and
                                float(r['lead_speed_mps'])<run['config']['lead_speed_mps']-1),None)
                    run['input_braking_observed_s']=onset
                    assert onset is not None
                    contact=run['metrics']['first_collision_s']
                    assert contact is None or onset<contact
        evidence[label]=data
    assert not differences(evidence['route'],evidence['repeat'])
    assert not differences(evidence['aeb-route'],evidence['aeb-repeat'])
    historical=json.loads((Path(__file__).resolve().parents[1]/'docs/integrated_reference.json').read_text())['stanley']
    assert not differences(historical,evidence['straight'])
    for name,run in evidence['route']['runs'].items():
        if name!='holdout_extreme_brake_acc':
            assert run['metrics']['verdict']=='pass'
    times=('first_aeb_s','first_collision_s','first_lead_selected_s','event_observed_s')
    shifts=[]
    for name,run in evidence['route']['runs'].items():
        a,b=run['metrics'],evidence['repeat']['runs'][name]['metrics']
        shifts.extend(abs(a[k]-b[k]) for k in times if a[k] is not None and b[k] is not None)
    evidence['repeat_max_event_shift_s']=max(shifts)
    evidence['video']=json.loads((root/'video/capture.json').read_text())
    (root/'reference.json').write_text(json.dumps(evidence,indent=2))
    lines=['# Curved-road joint validation','',
           'Town04, physical lead, truth-state route projection. Controller gains remain frozen.','',
           '| Case | Policy | Result | Coverage | Ego peak m | Lead peak m | Turn deg | AEB s |',
           '|---|---|---|---|---|---|---|---|']
    for label in ('corridor','route','aeb-route'):
        for name,run in evidence[label]['runs'].items():
            m=run['metrics']
            values=[name,label,m['verdict'],m['target_coverage'],m['path_peak_m'],
                    m['lead_path_peak_m'],m['turn_deg'],m['first_aeb_s']]
            lines.append('| '+' | '.join('{:.4f}'.format(v) if isinstance(v,float) else str(v) for v in values)+' |')
    lines+=['','All frame metrics replayed. Route repeat event maximum shift: '+str(max(shifts))+' s.',
            'Twenty straight-road regression outcomes match the frozen integrated reference.']
    (root/'report.md').write_text('\n'.join(lines)+'\n')
    print(root/'report.md')
    print('Verified 24 curved runs, 20 straight runs and physical lead braking before contact.')


if __name__=='__main__':
    main()
