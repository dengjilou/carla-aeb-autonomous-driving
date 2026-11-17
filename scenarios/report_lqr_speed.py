"""Recompute metrics, audit repeats and plot speed scheduling experiments offline."""
import argparse
import json
import math
from pathlib import Path
import zipfile
from report_longitudinal import replay

EXTRA=('first_path_exit_s','path_abs_p95_m','steer_rate_peak_per_s')


def audit(folder):
    data=json.loads((folder/'summary.json').read_text())
    traces={}
    for name,run in data['runs'].items():
        base=dict(run,metrics={k:v for k,v in run['metrics'].items() if k not in EXTRA})
        rows=replay(folder/(name+'.csv'),base)
        contact=run['metrics']['first_collision_s']
        before=[r for r in rows if contact is None or float(r['time_s'])<contact]
        errors=sorted(abs(float(r['path_error_m'])) for r in before)
        values=(next((float(r['time_s']) for r in before if abs(float(r['path_error_m']))>1),None),
                errors[math.ceil(.95*len(errors))-1],
                max(abs(float(b['steer'])-float(a['steer']))/.05 for a,b in zip(before,before[1:])))
        assert all(run['metrics'][key]==value for key,value in zip(EXTRA,values))
        traces[name]=rows
    repeats={}
    for name,run in data['runs'].items():
        if not name.endswith('_0'):
            continue
        other=data['runs'][name[:-1]+'1']['metrics']
        m=run['metrics']
        assert all(m[k]==other[k] for k in ('collided','valid','verdict'))
        deltas={}
        for k in ('first_collision_s','first_aeb_s','first_path_exit_s','event_observed_s'):
            assert (m[k] is None)==(other[k] is None)
            deltas[k]=abs(m[k]-other[k]) if m[k] is not None else 0
            assert deltas[k]<=.0500001
        repeats[name[:-2]]=deltas
    return data,traces,repeats


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnosis',type=Path,required=True)
    parser.add_argument('--validation',type=Path,required=True)
    parser.add_argument('--filter-validation',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plots',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    report={}
    all_traces={}
    groups=(('diagnosis',args.diagnosis),('validation',args.validation),('filter_validation',args.filter_validation))
    for label,folder in groups:
        data,traces,repeats=audit(folder)
        report[label]=dict(data,repeat_event_deltas=repeats)
        all_traces[label]=traces
    assert len(report['diagnosis']['runs'])==8
    assert len(report['validation']['runs'])==28
    assert len(report['filter_validation']['runs'])==28
    (args.output/'lqr_speed_reference.json').write_text(json.dumps(report,indent=2)+'\n')
    with zipfile.ZipFile(args.output/'lqr_speed_traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for label,folder in groups:
            for path in sorted(folder.glob('*.csv'))+[folder/'summary.json']:
                archive.write(path,label+'/'+path.name)
    print('PASS: 64 traces recomputed; 32 pairs match collision/validity and events within one step')
    if not args.plots:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for variant in ('fixed','scheduled','fixed_no_filter','scheduled_no_filter'):
        rows=all_traces['diagnosis']['holdout_extreme_brake_'+variant+'_0']
        t=[float(r['time_s']) for r in rows]
        for ax,key in zip(axes,('path_error_m','ego_speed_mps','steer')):
            ax.plot(t,[float(r[key]) for r in rows],label=variant)
    axes[0].axhline(1,color='gray',ls='--'); axes[0].axhline(-1,color='gray',ls='--')
    for ax,label in zip(axes,('Center path error (m)','Speed (m/s)','Normalized steering')):
        ax.set_ylabel(label); ax.grid(alpha=.3)
    axes[0].legend(); axes[-1].set_xlabel('Simulation time (s), including post-contact frames')
    figure.tight_layout()
    (args.output/'images').mkdir(exist_ok=True)
    figure.savefig(args.output/'images/lqr_speed.png',dpi=140)


if __name__=='__main__':
    main()
