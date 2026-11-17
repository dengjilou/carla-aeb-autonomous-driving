"""Recompute AEB distance-reference results and independently audit threshold decisions."""
import argparse
import json
import math
from pathlib import Path
import zipfile
from report_longitudinal import replay


def audit_run(folder,name,run):
    rows=replay(folder/(name+'.csv'),run)
    level=0
    for r in rows:
        field='aeb_'+('body_gap' if run['config']['aeb_distance_reference']=='body' else 'center_distance')+'_m'
        assert r['aeb_distance_m']==r[field]
        if not r['aeb_distance_m']:
            level=0
        else:
            distance,closing=float(r['aeb_distance_m']),float(r['aeb_closing_mps'])
            ttc=distance/closing if closing>.1 else math.inf
            assert math.isclose(float(r['aeb_ttc_s']),ttc,abs_tol=1e-12)
            full_threshold = 2.5 if run['config'].get('full_brake_at_warning',False) else .6
            target=3 if ttc<=full_threshold else 2 if ttc<=1.6 else 1 if ttc<=2.5 else 0
            allow_distance=(not run['config'].get('aeb_closing_guard',False) or distance<=0
                            or (closing>.1 and distance/closing<=5.))
            if allow_distance:
                if distance<10:
                    target=3
                elif distance<15:
                    target=max(target,2)
            level=max(level,target)
        assert level==int(r['aeb_level'])
        partial_request = 1. if run['config'].get('full_brake_at_onset',False) else .4
        assert float(r['aeb_brake'])==(0.,0.,partial_request,1.)[level]
        if float(r['latched_brake'])>0:
            assert float(r['throttle'])==0 and float(r['brake'])>=float(r['latched_brake'])
    def first(predicate):
        row=next((r for r in rows if predicate(r)),None)
        snapshot={k:float(row[k]) if row[k] else None for k in (
            'time_s','ego_speed_mps','lead_speed_mps','aeb_distance_m','aeb_center_distance_m',
            'aeb_body_gap_m','aeb_closing_mps','aeb_ttc_s')} if row else None
        return {k:v if v is None or math.isfinite(v) else None for k,v in snapshot.items()} if snapshot else None
    return dict(first_brake=first(lambda r:float(r['aeb_brake'])>0),
                first_full=first(lambda r:float(r['aeb_brake'])==1),frames=len(rows))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True,help='Contains a, b, equal-a and equal-b directories')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plots',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    data={}
    for label,count in (('a',28),('b',28),('equal-a',4),('equal-b',4)):
        folder=args.input/label
        report=json.loads((folder/'summary.json').read_text())
        assert len(report['runs'])==count
        assert {p.stem for p in folder.glob('*.csv')}==set(report['runs'])
        evidence={name:audit_run(folder,name,run) for name,run in report['runs'].items()}
        data[label]=dict(report,frame_audit=evidence)
    repeat_deltas={}
    for left,right in (('a','b'),('equal-a','equal-b')):
        for name,run in data[left]['runs'].items():
            m,other=run['metrics'],data[right]['runs'][name]['metrics']
            assert all(m[k]==other[k] for k in ('valid','verdict','collided'))
            deltas={}
            for key in ('first_aeb_s','first_collision_s','event_observed_s'):
                assert (m[key] is None)==(other[key] is None)
                deltas[key]=abs(m[key]-other[key]) if m[key] is not None else 0
                assert deltas[key]<=.0500001
            a=data[left]['frame_audit'][name]['first_full']
            b=data[right]['frame_audit'][name]['first_full']
            assert (a is None)==(b is None)
            deltas['first_full_s']=abs(a['time_s']-b['time_s']) if a else 0
            assert deltas['first_full_s']<=.0500001
            repeat_deltas[name]=deltas
    result=dict(groups=data,repeat_event_deltas=repeat_deltas)
    (args.output/'aeb_gap_reference.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with zipfile.ZipFile(args.output/'aeb_gap_traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for label in data:
            for path in sorted((args.input/label).glob('*.csv'))+[args.input/label/'summary.json']:
                archive.write(path,label+'/'+path.name)
    print('PASS: 64 traces, distance/TTC/state/latch audits and 32 repeat pairs')
    if not args.plots:
        return
    import csv
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for ref in ('center','body'):
        rows=list(csv.DictReader((args.input/'a'/('diagnosis_stress_'+ref+'_baseline.csv')).open()))
        times=[float(r['time_s']) for r in rows]
        for ax,key in zip(axes,('aeb_body_gap_m','ego_speed_mps','aeb_brake')):
            ax.plot(times,[float(r[key]) if r[key] else math.nan for r in rows],label=ref)
    for ax,label in zip(axes,('Projected body gap (m)','Speed (m/s)','AEB request')):
        ax.set_ylabel(label); ax.grid(alpha=.3)
    axes[0].legend(); axes[-1].set_xlabel('Time (s), including post-contact frames')
    fig.tight_layout()
    (args.output/'images').mkdir(exist_ok=True)
    fig.savefig(args.output/'images/aeb_gap.png',dpi=140)


if __name__=='__main__':
    main()
