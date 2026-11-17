"""Controlled real-node pause: timeout, accepted emergency command, recovery."""
import argparse
import json
import subprocess
from pathlib import Path
from unittest.mock import patch
from run_longitudinal import run_case, load_config

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-pid',type=int,required=True,help='PID of this project control_node in Ubuntu-24.04')
    parser.add_argument('--output',type=Path,default=ROOT/'output/ros-timeout')
    args=parser.parse_args()
    # Guard against freezing an unrelated process.
    def wsl(*command):
        return subprocess.run(['wsl','-d','Ubuntu-24.04','-u','root','--',*command],
                              check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    identity=wsl('ps','-p',str(args.node_pid),'-o','comm=').stdout.decode().strip()
    if identity!='control_node':
        raise ValueError('Expected the project control_node, got '+identity)
    import control_ros
    from control_native import PythonController
    original=control_ros.RosController
    class PausedController(original):
        def step(self,values):
            if self.sequence==20:
                wsl('kill','-STOP',str(args.node_pid))
            return super().step(values)
    config=load_config(ROOT/'configs/longitudinal.json')
    case=next(c for c in config['cases'] if c['name']=='follow_holdout')
    cfg=dict(case,step_s=config['step_s'],seed=config['seed'],lateral_controller='stanley',backend='ros2')
    args.output.mkdir(parents=True,exist_ok=True)
    result=dict(injection='SIGSTOP control_node before frame 20',physical_stopping_test=False)
    try:
        with patch.object(control_ros,'RosController',PausedController):
            try:
                run_case(cfg,'acc',config['gains'],trace_path=args.output/'partial.csv')
            except RuntimeError as exc:
                result['execution_error']=str(exc)
            else:
                raise AssertionError('Timeout was not detected')
    finally:
        wsl('kill','-CONT',str(args.node_pid))
    assert 'safe_brake_batch_accepted=True' in result['execution_error']
    assert 'exceeded 0.8 s' in result['execution_error']
    # A fresh session must operate after the paused node resumes.
    client=original()
    reference=PythonController()
    row=dict(dt=.05,speed=10.,cruise=12.,gap=26.,closing=0.,heading=0.,
             path_heading=0.,cross_track=0.,max_angle=1.2,aeb_request=0.,
             has_target=1,acc_enabled=1)
    try:
        for _ in range(20):
            expected=reference.step(row)
            actual=client.step(row)
            assert max(abs(actual[k]-expected[k]) for k in expected)<=1e-9
        result['recovery_frames']=20
    finally:
        client.close()
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))


if __name__=='__main__':
    main()
