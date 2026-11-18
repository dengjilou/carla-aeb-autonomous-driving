import sys, json, shutil, hashlib
from pathlib import Path
from unittest.mock import patch
import carla
root=Path(__file__).resolve().parents[1]
folder=Path(sys.argv[1]).resolve()
folder.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(root/'scenarios'))
import run_longitudinal as runner
client=carla.Client('localhost',2000)
client.set_timeout(60)
world=client.get_world()
assert not world.get_settings().synchronous_mode and not list(world.get_actors().filter('vehicle.*'))
assert world.get_map().name.endswith('Town04'), 'Start an idle Town04 first'

config=runner.load_config(root/'configs/longitudinal.json')
config['cases']=[c for c in config['cases'] if c['name'] in ('follow_holdout','hard_brake_holdout','short_cutin_failure')]
cfg=folder/'cases.json'
cfg.write_text(json.dumps(config,indent=2))
def run(name):
    sys.argv=['runner','--resume','--config',str(cfg),'--output',str(folder/name),'--mode','acc']
    runner.main()
def report(name):
    return json.loads((folder/name/'summary.json').read_text())['runs']
run('full')
original=runner.run_case
def abort(cfg,*args,**kwargs):
    if cfg['kind']=='hard_brake':
        raise KeyboardInterrupt()
    return original(cfg,*args,**kwargs)
try:
    with patch.object(runner,'run_case',abort): run('resumed')
except KeyboardInterrupt: pass
get_hash=lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
first=folder/'resumed/follow_holdout_acc.csv'
saved=get_hash(first)
run('resumed')
assert get_hash(first)==saved
for k,a in report('full').items():
    b=report('resumed')[k]['metrics']
    for key in ('valid','verdict','collided','first_aeb_s','first_collision_s','first_lead_selected_s','event_observed_s'):
        x,y=a['metrics'][key],b[key]
        assert x==y or (key.endswith('_s') and x is not None and y is not None and abs(x-y)<=0.050001),(k,key,x,y)
shutil.copytree(str(folder/'resumed'),str(folder/'damaged'))
with (folder/'damaged/short_cutin_failure_acc.csv').open('ab') as f: f.write(b'corrupt')
run('damaged')
hashes={p.name:get_hash(p) for p in (folder/'damaged').glob('*.csv')}
for _ in range(2): run('damaged')
assert hashes=={p.name:get_hash(p) for p in (folder/'damaged').glob('*.csv')}
print('VERIFIED recovery, retained first trace, corruption repair and idempotence',flush=True)

