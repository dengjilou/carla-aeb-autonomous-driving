"""Recheck runtime evidence and package original traces and ROS recordings."""
import json
import shutil
import sys
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scenarios'))
from report_runtime import audit
from report_integrated import differences
from report_longitudinal import replay


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    output=ROOT/'output'
    evidence=dict(native=audit(output,'native'),ros=audit(output,'ros'))
    prior=read(ROOT/'docs/cosim_aeb_reference.json')['cases']
    aeb=read(output/'runtime-aeb-9/summary.json')
    assert len(aeb['cases'])==9
    assert not [(n,k) for n,r in aeb['cases'].items()
                for k in ('lead_collision','first_aeb_brake_s','first_collision_s') if r[k]!=prior[n][k]]
    evidence['aeb_9']=aeb
    video=read(output/'ros-video/capture.json')
    curved=read(ROOT/'docs/curved_reference.json')
    for name,run in video.items():
        replay(output/'ros-video'/(name+'.csv'),run)
        reference=curved['aeb-route' if run['mode']=='baseline' else 'route']['runs'][name+'_'+run['mode']]
        assert not differences({'runs':{name:reference}},{'runs':{name:run}})
    evidence['video']=video
    evidence['bag_replay']=read(output/'rosbag-replay.json')
    assert evidence['bag_replay']['matched_frames']==360
    assert evidence['bag_replay']['command_abs_error_max']<=1e-9
    evidence['timeout']=read(output/'ros-timeout/summary.json')
    assert 'safe_brake_batch_accepted=True' in evidence['timeout']['execution_error']
    assert evidence['timeout']['recovery_frames']==20
    evidence['environment']=dict(carla='0.9.13',sumo='1.18.0',step_s=.05,ros='Jazzy',
        wsl='Ubuntu 24.04',simulation_python='3.7.9',offline_python='3.12',standard='C++17',
        targeting='simulator truth; Python geometry and AEB trigger; C++ control and arbitration')
    (ROOT/'docs/runtime_reference.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    shutil.copyfile(output/'ros-video/integrated_demo.mp4',ROOT/'docs/runtime_demo.mp4')
    shutil.copyfile(output/'ros-video/integrated_demo.png',ROOT/'docs/images/runtime_camera.png')
    shutil.copyfile(output/'native-gdb.log',ROOT/'docs/native_gdb.txt')
    folders=[output/(backend+'-'+group+suffix)
             for backend in ('native','ros') for group in ('straight','curved','stress')
             for suffix in ('','-repeat')]
    folders += [output/p for p in ('runtime-aeb-9','ros-video','ros-timeout','rosbag-follow','rosbag-curved')]
    with zipfile.ZipFile(ROOT/'docs/runtime_traces.zip','w',zipfile.ZIP_DEFLATED) as archive:
        for folder in folders:
            for path in sorted(folder.rglob('*')):
                if path.is_file() and path.suffix in ('.csv','.json','.db3','.yaml'):
                    archive.write(path,path.relative_to(output))
        archive.write(output/'rosbag-replay.json','rosbag-replay.json')
    print('Packaged 112 closed-loop comparisons, 9 legacy cases, 4 camera scenes, timeout and rosbag evidence.')


if __name__=='__main__':
    main()
