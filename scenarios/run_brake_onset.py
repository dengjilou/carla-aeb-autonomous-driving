"""Paired closed-loop diagnostic: change only the brake request at partial activation."""
import argparse
import json
from pathlib import Path

from longitudinal_metrics import evaluate
from run_longitudinal import ROOT, load_config, run_case


CASES = ('unseen_18_34_body_guard', 'unseen_22_34_body_guard',
         'fresh_brake_20_30_body_guard', 'fresh_brake_24_38_body_guard',
         'equal_4_body_guard', 'regression_holdout_extreme_brake_body_guard',
         'diagnosis_stress_body_guard')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--warning', action='store_true')
    args = parser.parse_args()
    folder = args.output
    folder.mkdir(parents=True, exist_ok=False)
    config = load_config(ROOT/'configs/aeb_closing.json')
    cases = {case['name']: case for case in config['cases']}
    results = {'gains': config['gains'], 'runs': {}}
    for name in CASES:
        for repeat in (1, 2):
            for policy in (('warning',) if args.warning else ('staged', 'immediate')):
                case = dict(cases[name], seed=config['seed'], step_s=config['step_s'],
                            lateral_controller='lqr', backend='python',
                            full_brake_at_onset=(policy == 'immediate'),
                            full_brake_at_warning=(policy == 'warning'))
                key = '{}_{}_{}'.format(name, policy, repeat)
                mode = case['demo_mode']
                rows, contacts = run_case(case, mode, config['gains'], trace_path=folder/(key+'.csv'))
                metrics = evaluate(rows, contacts, case)
                results['runs'][key] = dict(config=case, mode=mode, collisions=contacts, metrics=metrics)
                (folder/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
                print(key, metrics['verdict'], metrics['first_aeb_s'], metrics['first_collision_s'], flush=True)


if __name__ == '__main__':
    main()
