"""Audit recorded pairs, including equality of the input history through first AEB."""
import argparse
import csv
import json
from pathlib import Path

from report_aeb_gap import audit_run
from run_brake_onset import CASES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--warning-folder', type=Path, required=True)
    args = parser.parse_args()
    folder = args.folder
    runs = json.loads((folder/'summary.json').read_text())['runs']
    assert len(runs) == 4*len(CASES)
    warning_runs = json.loads((args.warning_folder/'summary.json').read_text())['runs']
    assert len(warning_runs) == 2*len(CASES)
    assert not set(runs).intersection(warning_runs)
    runs.update(warning_runs)
    compared = {}
    for name in CASES:
        repeats = []
        for repeat in (1, 2):
            outcomes = {}
            histories = []
            configs = []
            for policy in ('staged', 'immediate', 'warning'):
                key = '{}_{}_{}'.format(name, policy, repeat)
                run = runs[key]
                cfg = run['config'].copy()
                assert cfg.pop('full_brake_at_onset') == (policy == 'immediate')
                assert cfg.pop('full_brake_at_warning',False) == (policy == 'warning')
                configs.append(cfg)
                trace_folder = args.warning_folder if policy == 'warning' else folder
                events = audit_run(trace_folder, key, run)
                metrics = run['metrics']
                with (trace_folder/(key+'.csv')).open() as file:
                    rows = list(csv.DictReader(file))
                prefix = []
                for row in rows:
                    prefix.append(tuple(row[k] for k in ('time_s','ego_speed_mps','lead_speed_mps',
                                                         'path_error_m','aeb_distance_m','aeb_closing_mps',
                                                         'steer')))
                    assert row['target_id'] == row['lead_id'] or not row['target_id']
                    if float(row['aeb_brake']) > 0:
                        break
                histories.append(prefix)
                contact = metrics['first_collision_s']
                last_before_contact = next((r for r in reversed(rows)
                                            if contact is not None and float(r['time_s']) < contact), None)
                outcomes[policy] = dict(verdict=metrics['verdict'],valid=metrics['valid'],
                    collision=metrics['collided'],contact_s=metrics['first_collision_s'],
                    first_brake=events['first_brake'],first_full_request=events['first_full'],
                    approx_precontact_closing_mps=(float(last_before_contact['aeb_closing_mps'])
                        if last_before_contact and last_before_contact['target_id']==last_before_contact['lead_id']
                        else None),
                    min_gap_m=metrics.get('min_path_gap_m',metrics['min_projected_gap_m']),
                    precontact_kmh=metrics['pre_contact_speed_kmh'])
            assert configs[0] == configs[1] == configs[2], name
            assert histories[0] == histories[1], name
            common_length = min(map(len,histories))
            assert histories[0][:common_length] == histories[2][:common_length], name
            repeats.append(outcomes)
        assert repeats[0] == repeats[1], name
        compared[name] = repeats[0]
    print(json.dumps(compared, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
