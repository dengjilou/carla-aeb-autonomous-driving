"""Recompute the 31 m / 39 m AEB diagnostic from two runs of the same cases."""
import argparse
import json
from pathlib import Path

from report_aeb_gap import audit_run

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='directory containing a and b')
    args = parser.parse_args()
    config = json.loads((ROOT / 'configs/offset_check.json').read_text())
    cases = {c['name'] + '_' + c['demo_mode']: c for c in config['cases']}
    first_run = {}
    for repeat in ('a', 'b'):
        folder = args.input / repeat
        summary = json.loads((folder / 'summary.json').read_text())
        assert summary['gains'] == config['gains'] and summary['runs'].keys() == cases.keys()
        for name, case in cases.items():
            run = summary['runs'][name]
            expected = dict(case, step_s=config['step_s'], seed=config['seed'],
                            backend='python', lateral_controller='lqr')
            assert all(run['config'][k] == v for k, v in expected.items()), name
            assert run['mode'] == case['demo_mode'], name
            audit = audit_run(folder, name, run)
            metrics = run['metrics']
            assert metrics['valid'], (name, metrics['invalid_reasons'])
            full = audit['first_full']
            events = (metrics['first_aeb_s'], full['time_s'] if full else None,
                      metrics['first_collision_s'])
            if repeat == 'a':
                first_run[name] = events
            else:
                for a, b in zip(first_run[name], events):
                    assert (a is None) == (b is None), (name, a, b)
                    assert a is None or abs(a - b) <= config['step_s'] + 1e-9, (name, a, b)
            print(repeat, name, 'brake/full/contact (s)', events,
                  'min projected gap (m)', round(metrics['min_projected_gap_m'], 3))


if __name__ == '__main__':
    main()
