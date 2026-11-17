"""Audit the predeclared independent AEB set and compare comfort on common windows."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import zipfile

from report_aeb_gap import audit_run

ROOT = Path(__file__).resolve().parents[1]


def configuration_sha256(raw):
    return hashlib.sha256(raw.replace(b'\r\n',b'\n')).hexdigest()


def comfort(rows, end):
    speeds = [float(row['ego_speed_mps']) for row in rows if float(row['time_s']) < end]
    acceleration = [(b-a)/.05 for a,b in zip(speeds,speeds[1:])]
    jerk = [(b-a)/.05 for a,b in zip(acceleration,acceleration[1:])][18:]
    return dict(window_end_s=end,peak_deceleration_mps2=max(0,-min(acceleration)),
                jerk_rms_mps3=math.sqrt(statistics.mean(value*value for value in jerk)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plots',action='store_true')
    args = parser.parse_args()
    manifest = json.loads((ROOT/'docs/independent_aeb_manifest.json').read_text())
    config_path = ROOT/'configs/independent_aeb.json'
    assert configuration_sha256(config_path.read_bytes()) == manifest['config_lf_sha256']
    config = json.loads(config_path.read_text())
    cases = {case['name']:case for case in config['cases']}
    groups = {}
    traces = {}
    for label in ('a','b'):
        folder = args.input/label
        data = json.loads((folder/'summary.json').read_text())
        assert len(data['runs']) == 32
        assert {p.stem for p in folder.glob('*.csv')} == set(data['runs'])
        events = {}
        for name,run in data['runs'].items():
            frozen = cases[run['config']['name']]
            assert all(run['config'][key] == value for key,value in frozen.items())
            assert run['mode'] == frozen['demo_mode']
            assert run['config']['step_s'] == .05 and run['config']['seed'] == 42
            assert run['config']['backend'] == 'python' and run['config']['lateral_controller'] == 'lqr'
            assert data['gains'] == config['gains']
            events[name] = audit_run(folder,name,run)
            with (folder/(name+'.csv')).open() as file:
                traces[label,name] = list(csv.DictReader(file))
        groups[label] = dict(data,frame_audit=events)
    repeat_deltas = {}
    for name,run in groups['a']['runs'].items():
        left,right = run['metrics'],groups['b']['runs'][name]['metrics']
        assert all(left[key] == right[key] for key in ('valid','verdict','collided'))
        delta = {}
        for key in ('first_aeb_s','first_collision_s','event_observed_s'):
            assert (left[key] is None) == (right[key] is None)
            delta[key] = abs(left[key]-right[key]) if left[key] is not None else 0
            assert delta[key] <= .0500001
        a,b = (groups[label]['frame_audit'][name]['first_full'] for label in ('a','b'))
        assert (a is None) == (b is None)
        delta['first_full_s'] = abs(a['time_s']-b['time_s']) if a else 0
        assert delta['first_full_s'] <= .0500001
        repeat_deltas[name] = delta
    comparisons = {}
    for name,run in groups['a']['runs'].items():
        if run['config']['full_brake_at_warning']:
            continue
        other_name = name.replace('_staged_','_warning_')
        other = groups['a']['runs'][other_name]
        configs = []
        for item in (run,other):
            normalized = item['config'].copy()
            normalized.pop('name')
            normalized.pop('full_brake_at_warning')
            configs.append(normalized)
        assert configs[0] == configs[1]
        onset = [item['metrics']['first_aeb_s'] for item in (run,other)]
        intervention = min((value for value in onset if value is not None),default=run['config']['duration_s'])
        columns = ('time_s','ego_speed_mps','lead_speed_mps','aeb_distance_m','aeb_closing_mps','steer','path_error_m')
        prefixes = [[tuple(row[key] for key in columns) for row in traces['a',key]
                     if float(row['time_s']) <= intervention] for key in (name,other_name)]
        assert prefixes[0] == prefixes[1]
        contacts = [item['metrics']['first_collision_s'] for item in (run,other)]
        end = min([value for value in contacts if value is not None]+[run['config']['duration_s']])
        comparisons[run['config']['name'].removesuffix('_staged')] = dict(
            kind=run['config']['kind'],
            target_motion='kinematic' if run['config']['kind'] in ('cutin','cutout') else 'physics',
            staged=run['metrics'],warning=other['metrics'],
            common_window_comfort={label:comfort(traces['a',key],end) for label,key in
                                   (('staged',name),('warning',other_name))})
    result = dict(manifest=manifest,groups=groups,comparisons=comparisons,
                  repeat_event_deltas=repeat_deltas,personal_mastery='pending user demonstration')
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'independent_aeb_reference.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with zipfile.ZipFile(args.output/'independent_aeb_traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for label in groups:
            for path in sorted((args.input/label).glob('*.csv'))+[args.input/label/'summary.json']:
                archive.write(path,label+'/'+path.name)
    print('AUDIT PASS: 64 runs, 32 repeat pairs and frozen configuration verified')
    for name,pair in comparisons.items():
        print(name,pair['staged']['verdict'],pair['warning']['verdict'],
              'contacts',pair['staged']['collided'],pair['warning']['collided'])
    if args.plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        names = ['brake_'+str(index) for index in range(4)]
        figure,axis = plt.subplots(figsize=(10,4))
        for offset,policy in ((-.18,'staged'),(.18,'warning')):
            values = [comparisons[name]['common_window_comfort'][policy]['jerk_rms_mps3'] for name in names]
            axis.bar([index+offset for index in range(4)],values,width=.35,label=policy)
        axis.set_xticks(range(4))
        axis.set_xticklabels(['17 m/s, 31 m','21 m/s, 37 m','23 m/s, 33 m','27 m/s, 41 m'])
        axis.set_ylabel('Jerk RMS (m/s³)')
        axis.set_title('Common pre-contact observation window')
        axis.legend()
        axis.grid(axis='y',alpha=.25)
        figure.tight_layout()
        (args.output/'images').mkdir(exist_ok=True)
        figure.savefig(args.output/'images/independent_aeb_comfort.png',dpi=140)


if __name__ == '__main__':
    main()
