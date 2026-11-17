"""Controlled speed scheduling / steering-filter ablation on existing physical scenes."""
import argparse
import json
import math
from pathlib import Path
from run_longitudinal import ROOT, load_config, run_case
from longitudinal_metrics import evaluate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--phase',choices=('diagnosis','validation'),default='diagnosis')
    parser.add_argument('--filter-alpha',type=float,choices=(.4,1.),default=.4)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    config=load_config(ROOT/'configs/curved.json')
    suffix='_no_filter' if args.filter_alpha==1. else ''
    variants=[('fixed'+suffix,False,args.filter_alpha),('scheduled'+suffix,True,args.filter_alpha)]
    if args.phase=='diagnosis':
        variants=[('fixed',False,.4),('scheduled',True,.4),
                  ('fixed_no_filter',False,1.),('scheduled_no_filter',True,1.)]
    cases=[c for c in config['cases'] if args.phase=='validation' or c['name']=='holdout_extreme_brake']
    report=dict(phase=args.phase,step_s=config['step_s'],gains=config['gains'],runs={})
    for case in cases:
        for label,schedule,alpha in variants:
            for repeat in range(2):
                name=case['name']+'_'+label+'_'+str(repeat)
                cfg=dict(case,step_s=config['step_s'],seed=config['seed'],backend='python',
                         lateral_controller='lqr',lqr_schedule=schedule,steering_filter_alpha=alpha)
                mode=case.get('demo_mode','acc') if args.phase=='diagnosis' else 'acc'
                try:
                    rows,events=run_case(cfg,mode,config['gains'],trace_path=args.output/(name+'.csv'))
                    metrics=evaluate(rows,events,cfg)
                    before=[r for r in rows if metrics['first_collision_s'] is None or r['time_s']<metrics['first_collision_s']]
                    first_exit=next((r['time_s'] for r in before if abs(r['path_error_m'])>1),None)
                    errors=sorted(abs(r['path_error_m']) for r in before)
                    metrics.update(first_path_exit_s=first_exit,
                        path_abs_p95_m=errors[math.ceil(.95*len(errors))-1],
                        steer_rate_peak_per_s=max(abs(b['steer']-a['steer'])/cfg['step_s'] for a,b in zip(before,before[1:])))
                    result=dict(config=cfg,mode=mode,metrics=metrics,collisions=events)
                except Exception as exc:
                    report['runs'][name]=dict(config=cfg,execution_error=repr(exc))
                    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
                    raise
                report['runs'][name]=result
                (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
                print(name,metrics['verdict'],'peak',round(metrics['path_peak_m'],4),
                      'exit',first_exit,'contact',metrics['first_collision_s'],flush=True)


if __name__=='__main__':
    main()
