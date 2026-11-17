"""Audit the frozen closing-window experiment, including safety regressions."""
import argparse
import json
from pathlib import Path
import zipfile
from report_aeb_gap import audit_run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True,help='Contains a and b run directories')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plots',action='store_true')
    args=parser.parse_args()
    data={}
    for label in ('a','b'):
        folder=args.input/label
        report=json.loads((folder/'summary.json').read_text())
        assert len(report['runs'])==40
        assert {p.stem for p in folder.glob('*.csv')}==set(report['runs'])
        data[label]=dict(report,frame_audit={n:audit_run(folder,n,r) for n,r in report['runs'].items()})
    repeats={}
    for name,run in data['a']['runs'].items():
        m,other=run['metrics'],data['b']['runs'][name]['metrics']
        assert all(m[k]==other[k] for k in ('valid','verdict','collided'))
        delta={}
        for key in ('first_aeb_s','first_collision_s','event_observed_s'):
            assert (m[key] is None)==(other[key] is None)
            delta[key]=abs(m[key]-other[key]) if m[key] is not None else 0
            assert delta[key]<=.0500001
        left,right=(data[label]['frame_audit'][name]['first_full'] for label in ('a','b'))
        assert (left is None)==(right is None)
        delta['first_full_s']=abs(left['time_s']-right['time_s']) if left else 0
        assert delta['first_full_s']<=.0500001
        repeats[name]=delta
    comparisons={}
    regressions=[]
    collision_regressions=[]
    for name,run in data['a']['runs'].items():
        if run['config']['aeb_closing_guard']:
            continue
        guarded=data['a']['runs'][name.replace('_legacy_','_guard_')]
        old,new=run['metrics'],guarded['metrics']
        comparisons[name]=dict(legacy=old,guard=new,split=run['config']['split'])
        if ((old['verdict']=='pass' and new['verdict']!='pass') or
                (not old['collided'] and new['collided'])):
            regressions.append(name)
        if not old['collided'] and new['collided']:
            collision_regressions.append(name)
    args.output.mkdir(parents=True,exist_ok=True)
    result=dict(groups=data,comparisons=comparisons,repeat_event_deltas=repeats,
                regressions=regressions,collision_regressions=collision_regressions,
                promotion_eligible=not regressions)
    (args.output/'aeb_closing_reference.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with zipfile.ZipFile(args.output/'aeb_closing_traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for label in data:
            for path in sorted((args.input/label).glob('*.csv'))+[args.input/label/'summary.json']:
                archive.write(path,label+'/'+path.name)
    print('AUDIT PASS: 80 traces and 40 repeat pairs; this is not a functional pass claim')
    print('Verdict regressions:',regressions,'; collision regressions:',collision_regressions)
    if not args.plots:
        return
    import csv
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for label in ('legacy','guard'):
        rows=list(csv.DictReader((args.input/'a'/('equal_4_body_'+label+'_acc.csv')).open()))
        t=[float(r['time_s']) for r in rows]
        for ax,key in zip(axes,('ego_speed_mps','aeb_brake','brake')):
            ax.plot(t,[float(r[key]) for r in rows],label=label)
    for ax,title in zip(axes,('Ego speed (m/s)','AEB request','Applied brake')):
        ax.set_ylabel(title); ax.grid(alpha=.3); ax.legend()
    axes[-1].set_xlabel('Simulation time (s)')
    fig.tight_layout()
    (args.output/'images').mkdir(exist_ok=True)
    fig.savefig(args.output/'images/aeb_closing.png',dpi=140)


if __name__=='__main__':
    main()
