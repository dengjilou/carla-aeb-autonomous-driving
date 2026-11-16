"""Recompute diagnosis, parameter selection, validation and recovery evidence."""
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from report_lateral import trace, verify

ROOT=Path(__file__).resolve().parents[1]
NAMES=['diag-base','diag-noff','diag-nofilter','diag-q02','diag-q005',
       'refine-new','refine-new-repeat','recover-offset','recover-offset-repeat',
       'recover-heading','recover-heading-repeat']


def main():
    evidence={}
    for name in NAMES:
        folder=ROOT/'output'/name
        rows=verify(folder)
        cfg=json.loads((folder/'config.json').read_text())
        for row in rows:
            raw=trace(folder,row['case']+'_'+row['mode'])
            previous=np.r_[0,raw['steer'][:-1]]
            expected=cfg['steer_alpha']*np.clip(raw['raw_steer'],-.7,.7)+(1-cfg['steer_alpha'])*previous
            np.testing.assert_allclose(raw['steer'],expected,atol=1e-9)
            straight=raw[raw['station_m']<35]
            good=(abs(straight['error_m'])<=.1)&(abs(straight['body_heading_error'])<=math.radians(2))
            starts=[i for i in range(len(good)-20) if all(good[i:i+21])]
            recovered=float(straight['time_s'][starts[0]]) if starts else None
            assert recovered==row['recovery_s']
            np.testing.assert_allclose(max(abs(straight['error_m'])),row['recovery_peak_m'])
            if name.startswith('recover-offset'):
                np.testing.assert_allclose(raw['error_m'][0],.5,atol=.001)
            if name.startswith('recover-heading'):
                np.testing.assert_allclose(raw['body_heading_error'][0],math.radians(3),atol=.001)
            if name.startswith('diag'):
                # Regression for the rear axle falling behind the reference endpoint.
                assert abs(raw['rear_error'][0])<.001 and abs(raw['raw_steer'][0])<.001
        evidence[name]=dict(config=cfg,results=rows)
    choices=[]
    for name,q in [('diag-base',1),('diag-q02',.2),('diag-q005',.05)]:
        rows=evidence[name]['results']
        choices.append(dict(q=q,max_rmse=max(r['rmse_m'] for r in rows),
                            exits=sum(r['center_exits'] for r in rows),
                            steering_sum=sum(r['steer_rate_rms'] for r in rows)))
    acceptable=[r for r in choices if r['max_rmse']<=.1 and r['exits']==0]
    assert min(acceptable,key=lambda r:r['steering_sum'])['q']==.2
    evidence['selection']=choices
    repeats=[]
    for name in ['refine-new','recover-offset','recover-heading']:
        a,b=evidence[name]['results'],evidence[name+'-repeat']['results']
        assert len(a)==len(b)==8
        assert [(r['case'],r['mode'],r['center_exits'],r['recovery_exits']) for r in a]==[(r['case'],r['mode'],r['center_exits'],r['recovery_exits']) for r in b]
        for x,y in zip(a,b):
            assert (x['recovery_s'] is None)==(y['recovery_s'] is None)
            if x['recovery_s'] is not None:
                assert abs(x['recovery_s']-y['recovery_s'])<=.050001
        repeats.append(dict(name=name,rmse_shift=max(abs(x['rmse_m']-y['rmse_m']) for x,y in zip(a,b))))
    evidence['repeats']=repeats
    evidence['async_control_comparison']={}
    for name in ['recover-offset-async','recover-offset-repeat-async']:
        evidence['async_control_comparison'][name]=verify(ROOT/'output'/name)
    (ROOT/'docs/refinement_reference.json').write_text(json.dumps(evidence,indent=2))
    diagnostic=trace(ROOT/'output/diag-base','s_bend_lqr')
    fig,axes=plt.subplots(4,1,figsize=(9,9),sharex=True,constrained_layout=True)
    for field,ax,label in [('curvature',axes[0],'Curvature (1/m)'),('heading_error',axes[1],'Rear heading error (rad)')]:
        ax.plot(diagnostic['time_s'],diagnostic[field]);ax.set_ylabel(label)
    for field in ('ff_rad','feedback_rad'):
        axes[2].plot(diagnostic['time_s'],diagnostic[field],label=field)
    axes[2].set_ylabel('Steering angle (rad)')
    axes[2].legend()
    for field in ('raw_steer','steer'):
        axes[3].plot(diagnostic['time_s'],diagnostic[field],label=field)
    wheel_axis=axes[3].twinx()
    wheel_axis.plot(diagnostic['time_s'],diagnostic['wheel_angle_rad'],color='gray',alpha=.55,label='Measured wheel angle')
    wheel_axis.set_ylabel('Wheel angle (rad)')
    axes[3].set(xlabel='Simulation time (s)',ylabel='Normalized command')
    axes[3].legend()
    for ax in axes:ax.grid(True,alpha=.2)
    fig.savefig(ROOT/'docs/images/lqr_diagnosis.png',dpi=140)
    plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(9,6.4),constrained_layout=True)
    for mode,label,color in [('lqr','LQR q=1','#ad542c'),('lqr_smooth','LQR q=0.2','#007f7f')]:
        data=trace(ROOT/'output/refine-new','refine_s_'+mode)
        axes[0].plot(data['station_m'],data['error_m'],label=label,color=color)
        axes[1].plot(data['station_m'],data['steer'],label=label,color=color)
    axes[0].set_ylabel('Signed center error (m)')
    axes[0].legend()
    axes[1].set(xlabel='OpenDRIVE station (m)',ylabel='Normalized steering')
    for ax in axes:ax.set_xlim(35,117);ax.grid(True,alpha=.2)
    fig.savefig(ROOT/'docs/images/lqr_refinement.png',dpi=160)
    print('Verified selection, initial-state regression, all traces and repeats:',repeats)


if __name__=='__main__':
    main()
