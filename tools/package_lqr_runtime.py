"""Package verified LQR migration results, diagnostic replay and plots."""
import csv
import json
import shutil
import sys
import zipfile
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'controllers'),str(ROOT/'scenarios')]
from control_native import NativeController, PythonController
from lqr_tracking import LQR
from report_lqr_runtime import audit
from report_integrated import differences
from report_longitudinal import replay


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def numerical_check():
    maximum=dict(gain_abs_error=0.,riccati_abs_error=0.,riccati_residual=0.,closed_loop_spectral_radius=0.)
    for speed in (.5,4.,8.,12.,25.):
        for weight in (.05,.2,1.):
            native=NativeController(dict(lateral_mode=1,lqr_speed=speed,lqr_wheelbase=2.9,lqr_position_weight=weight))
            try:
                gain,p=native.lqr_solution()
                gain=np.array(gain).reshape(1,2)
                p=np.array(p).reshape(2,2)
                py=LQR(speed,2.9,position_weight=weight)
                values=dict(gain_abs_error=float(abs(gain-py.k).max()),riccati_abs_error=float(abs(p-py.p).max()),
                    riccati_residual=float(abs(p-py.q-py.a.T@p@(py.a-py.b@gain)).max()),
                    closed_loop_spectral_radius=float(max(abs(np.linalg.eigvals(py.a-py.b@gain)))))
                for key,value in values.items():
                    maximum[key]=max(maximum[key],value)
            finally:
                native.close()
    assert maximum['gain_abs_error']<=1e-9 and maximum['riccati_abs_error']<=1e-8
    assert maximum['riccati_residual']<=1e-8 and maximum['closed_loop_spectral_radius']<1
    return dict(combinations=15,**maximum)


def replay_commands(path,run):
    model=run['config']['lqr_model']
    config=dict(lateral_mode=1,lqr_speed=model['design_speed_mps'],lqr_wheelbase=model['wheelbase_m'],
                lqr_dt=model['dt_s'],lqr_position_weight=model['position_weight'])
    controllers=[PythonController(config),NativeController(config)]
    maximum=0.
    count=0
    try:
        with path.open(newline='',encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                values=dict(dt=run['config']['step_s'],speed=float(row['ego_speed_mps']),
                    cruise=run['config']['ego_speed_mps'],gap=float(row['gap_m']),
                    closing=float(row['control_closing_mps']),heading=float(row['ego_yaw_rad']),
                    path_heading=float(row['control_path_heading_rad']),cross_track=float(row['control_cross_track_m']),
                    max_angle=float(row['control_max_angle_rad']),curvature=float(row['control_curvature_inv_m']),
                    aeb_request=float(row['aeb_brake']),has_target=int(row['control_has_target']=='True'),
                    acc_enabled=int(run['mode']=='acc'))
                expected={k:float(row[v]) for k,v in dict(reference='target_speed_mps',throttle='throttle',
                    brake='brake',steer='steer',latched_brake='latched_brake').items()}
                for controller in controllers:
                    actual=controller.step(values)
                    maximum=max(maximum,max(abs(expected[k]-actual[k]) for k in expected))
                count+=1
    finally:
        for controller in controllers:
            controller.close()
    assert maximum<=1e-9
    return dict(frames=count,python_and_native_abs_error_max=maximum)


def plots(output,evidence):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    old=read(ROOT/'docs/curved_reference.json')['route']['runs']
    current=evidence['groups']['curved']['runs']['python']['runs']
    names=[n for n in current if 'extreme' not in n]
    labels=['Dev follow','Dev slow','Dev brake','Hold follow','Hold slow','Hold brake']
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    x=np.arange(len(names))
    for ax,field,title in zip(axes,('path_rmse_m','path_peak_m'),('Path RMSE (m)','Peak path error (m)')):
        ax.bar(x-.18,[old[n]['metrics'][field] for n in names],.36,label='Stanley',color='#edb44c')
        ax.bar(x+.18,[current[n]['metrics'][field] for n in names],.36,label='Fixed-gain LQR',color='#33a8b5')
        ax.set_ylabel(title)
        ax.grid(axis='y',alpha=.25)
        ax.legend()
    axes[-1].set_xticks(x,labels)
    fig.suptitle('Physical CARLA-SUMO bends: vehicle-center path error before contact')
    fig.tight_layout()
    fig.savefig(ROOT/'docs/images/lqr_runtime_comparison.png',dpi=150)
    plt.close(fig)
    run=evidence['groups']['stress']['runs']['python']['runs']['holdout_extreme_brake_baseline']
    rows=replay(output/'python-stress/holdout_extreme_brake_baseline.csv',run)
    before=[r for r in rows if float(r['time_s'])<run['metrics']['first_collision_s']]
    first_out=next(float(r['time_s']) for r in before if abs(float(r['path_error_m']))>1.)
    evidence['stress_diagnosis']=dict(first_path_error_over_1m_s=first_out,
        first_full_aeb_s=next(float(r['time_s']) for r in rows if float(r['aeb_brake'])>=1.),
        first_contact_s=run['metrics']['first_collision_s'],valid=False)
    fig,axes=plt.subplots(3,1,figsize=(9,6),sharex=True)
    t=[float(r['time_s']) for r in rows]
    for ax,field,label in zip(axes,('path_error_m','ego_speed_mps','aeb_brake'),
                             ('Center path error (m)','Ego speed (m/s)','AEB request')):
        ax.plot(t,[float(r[field]) for r in rows],label=label,color='#168b9a')
        ax.axvline(run['metrics']['first_collision_s'],color='#d65c54',ls='--',label='First contact')
        ax.grid(alpha=.25)
        ax.legend()
    axes[0].axhline(-1,color='#777777',ls=':')
    axes[0].axhline(1,color='#777777',ls=':')
    axes[-1].set_xlabel('Simulation time (s)')
    fig.suptitle('Invalid high-speed stress case: lane departure precedes contact')
    fig.tight_layout()
    fig.savefig(ROOT/'docs/images/lqr_runtime_stress.png',dpi=150)
    plt.close(fig)


def main():
    output=ROOT/'output/lqr-runtime'
    evidence=audit(output)
    evidence['numerical']=numerical_check()
    video=read(output/'video/capture.json')
    evidence['video']=video
    evidence['csv_command_replay']={}
    for name,run in video.items():
        replay(output/'video'/(name+'.csv'),run)
        group='stress' if run['mode']=='baseline' else 'curved'
        baseline=evidence['groups'][group]['runs']['python']['runs'][name+'_'+run['mode']]
        assert not differences({'runs':{name:baseline}},{'runs':{name:run}})
        evidence['csv_command_replay'][name]=replay_commands(output/'video'/(name+'.csv'),run)
    evidence['legacy_bag_replay']=read(ROOT/'output/lqr-legacy-bag-check.json')
    evidence['lqr_bag_replay']=read(output/'bag-replay.json')
    assert evidence['legacy_bag_replay']['matched_frames']==360
    assert evidence['lqr_bag_replay']['matched_frames']==960
    assert evidence['lqr_bag_replay']['command_abs_error_max']<=1e-9
    old=read(ROOT/'docs/integrated_reference.json')['stanley']
    evidence['stanley_regression']={}
    for backend in ('cpp','ros2'):
        folder=output/('stanley-'+backend)
        result=read(folder/'summary.json')
        assert not differences(old,result)
        for name,run in result['runs'].items():
            replay(folder/(name+'.csv'),run)
        evidence['stanley_regression'][backend]=result
    plots(output,evidence)
    (ROOT/'docs/lqr_runtime_reference.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    shutil.copyfile(output/'video/integrated_demo.mp4',ROOT/'docs/lqr_runtime_demo.mp4')
    shutil.copyfile(output/'video/integrated_demo.png',ROOT/'docs/images/lqr_runtime_camera.png')
    with zipfile.ZipFile(ROOT/'docs/lqr_runtime_traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(output.rglob('*')):
            if path.is_file() and path.suffix in ('.csv','.json','.db3','.yaml') and 'from-zip' not in path.parts:
                archive.write(path,path.relative_to(output))
    print(json.dumps(evidence['numerical']))
    print('CSV command replay:',evidence['csv_command_replay'])
    print('Stress diagnosis:',evidence['stress_diagnosis'])
    print('Packaged',evidence['total_runs'],'runs and',evidence['total_frames'],'frames.')


if __name__=='__main__':
    main()
