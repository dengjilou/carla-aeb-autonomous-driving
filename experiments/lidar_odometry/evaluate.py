"""First-frame-aligned KITTI odometry evaluation. All pose errors use camera 0."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def transforms(values):
    matrices=np.tile(np.eye(4),(len(values),1,1))
    matrices[:,:3]=values.reshape(-1,3,4)
    return matrices


def segments_100m(gt,est):
    distance=np.r_[0,np.linalg.norm(np.diff(gt[:,:3,3],axis=0),axis=1).cumsum()]
    segments=[]
    for i in range(0,len(gt),10):
        j=int(np.searchsorted(distance,distance[i]+100,side='right'))
        if j>=len(gt):
            break
        rel_gt=np.linalg.inv(gt[i])@gt[j]
        rel_est=np.linalg.inv(est[i])@est[j]
        drift=np.linalg.norm((np.linalg.inv(rel_est)@rel_gt)[:3,3])
        segments.append((i,j,float(drift)))  # m per nominal 100 m = percent
    if not segments:
        raise ValueError("No 100 m segment")
    return distance,segments


def evaluate(dataset,seq,csv_file):
    table=np.genfromtxt(csv_file,delimiter=",",names=True)
    if not np.array_equal(table['frame'],np.arange(len(table))):
        raise ValueError("Noncontiguous estimate")
    calib=(dataset/"sequences"/seq/"calib.txt").read_text().splitlines()
    tr=transforms(np.fromstring(next(s[3:] for s in calib if s.startswith("Tr:")),sep=" ")[None,:])[0]
    lidar=transforms(np.stack([table[f"T{r}{c}"] for r in range(3) for c in range(4)],axis=1))
    est=tr@lidar@np.linalg.inv(tr)
    gt=transforms(np.loadtxt(dataset/"poses"/(seq+".txt")))
    if len(gt)!=len(table):
        raise ValueError("Expected complete sequence")
    gt=np.linalg.inv(gt[0])@gt
    error=np.linalg.norm(est[:,:3,3]-gt[:,:3,3],axis=1)
    distance,segments=segments_100m(gt,est)
    worst=max(segments,key=lambda s:s[2])
    return dict(frames=len(gt),registration_failures=int(sum(table['converged'][1:]==0)),
                coverage=float(sum(table['converged'][1:])/(len(gt)-1)),
                gt_distance_m=float(distance[-1]),translation_rmse_m=float(np.sqrt(np.mean(error**2))),
                drift_100m_pct=float(np.mean([s[2] for s in segments])),segments=len(segments),
                registration_p95_ms=float(np.percentile(table['registration_ms'][1:],95)),
                frame_p95_ms=float(np.percentile(table['frame_ms'][1:],95)),
                worst_segment=dict(start=worst[0],end=worst[1],drift_pct=worst[2])),gt,est,error,table,lidar


def figure(dataset,seq,runs,out):
    fig,axes=plt.subplots(1,2,figsize=(10,4.3))
    result={}
    for i,(label,file) in enumerate(runs):
        metrics,gt,est,err,_,_=evaluate(dataset,seq,file)
        result[label]=metrics
        if i==0:
            axes[0].plot(gt[:,0,3],gt[:,2,3],color='k',ls='--',lw=1,label='KITTI truth')
        axes[0].plot(est[:,0,3],est[:,2,3],lw=1,label=label)
        axes[1].plot(err,lw=1,label=label)
    axes[0].set(xlabel='Camera x (m)',ylabel='Camera z (m)',aspect='equal')
    axes[0].set_adjustable('datalim')
    axes[1].set(xlabel='Frame',ylabel='First-frame-aligned position error (m)')
    for ax in axes: ax.legend(); ax.grid(alpha=.2)
    fig.suptitle(f'KITTI odometry | sequence {seq}')
    fig.tight_layout();fig.savefig(out/f'{seq}_trajectory.png',dpi=150);plt.close(fig)
    return result


def overlay(dataset,seq,i,table,lidar,out):
    def points(index):
        path=dataset/'sequences'/seq/'velodyne'/f'{index:06}.bin'
        xyz=np.fromfile(path,'<f4').reshape(-1,4)[:,:3]
        return xyz[(np.abs(xyz[:,0])<20)&(np.abs(xyz[:,1])<20)&(xyz[:,2]>-1.5)][::4]
    prev,curr=points(i-1),points(i)
    rel=np.linalg.inv(lidar[i-1])@lidar[i]
    aligned=curr@rel[:3,:3].T+rel[:3,3]
    fig,axes=plt.subplots(1,2,figsize=(9.5,4.6),sharey=True)
    for ax,cloud,title in zip(axes,[curr,aligned],['Before registration','After registration']):
        ax.scatter(prev[:,0],prev[:,1],s=.7,color='#17766c',label='previous')
        ax.scatter(cloud[:,0],cloud[:,1],s=.7,color='#c96a35',alpha=.58,label='current')
        ax.set(aspect='equal',xlabel='Laser x (m)',title=title)
        ax.legend(markerscale=4)
    axes[0].set_ylabel('Laser y (m)')
    fig.suptitle(f'KITTI {seq} | scan {i} to {i-1} | ICP fitness {table["fitness"][i]:.3f}')
    fig.tight_layout();fig.savefig(out/'scan_pair.png',dpi=150);plt.close(fig)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path);p.add_argument('out',type=Path)
    p.add_argument('stage',choices=['develop','holdout']);a=p.parse_args()
    if a.stage=='develop':
        result=figure(a.dataset,'05',[(m,a.out/f'05_{m}.csv') for m in ['identity','motion']],a.out)
        mode=min(result,key=lambda m:(result[m]['registration_failures'],result[m]['drift_100m_pct'],m))
        (a.out/'development.json').write_text(json.dumps(dict(selection=mode,metrics=result),indent=2))
        print(mode)
    else:
        mode=json.loads((a.out/'development.json').read_text())['selection']
        path=a.out/f'07_{mode}.csv'
        result=figure(a.dataset,'07',[(mode,path)],a.out)[mode]
        _,gt,est,_,table,lidar=evaluate(a.dataset,'07',path)
        start,end=(result['worst_segment'][k] for k in ('start','end'))
        i=start+1+int(np.argmax(table['fitness'][start+1:end+1]))
        result['overlay_frame']=i
        actual_step=np.linalg.inv(gt[i-1])@gt[i]
        estimated_step=np.linalg.inv(est[i-1])@est[i]
        delta=np.linalg.inv(estimated_step)@actual_step
        result['overlay_diagnostics']=dict(
            converged=bool(table['converged'][i]),filtered_points=int(table['points'][i]),
            fitness_m2=float(table['fitness'][i]),
            scan_step_translation_error_m=float(np.linalg.norm(delta[:3,3])),
            scan_step_rotation_error_deg=float(np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))))
        overlay(a.dataset,'07',i,table,lidar,a.out)
        np.savez_compressed(a.out/'07_replay.npz',gt=gt,est=est,lidar=lidar,start=start,end=end)
        (a.out/'holdout.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
