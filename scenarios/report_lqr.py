"""Verify LQR development selection, repeated validation and three-controller plots."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from report_lateral import verify, trace

ROOT=Path(__file__).resolve().parents[1]


def main():
    names=['lqr-q005','lqr-q02','lqr-q1','lqr-known','lqr-new','lqr-new-repeat']
    evidence={name:verify(ROOT/'output'/name) for name in names}
    selection=[]
    for name, weight in zip(names[:3],[0.05,0.2,1.]):
        rows=evidence[name]
        assert {r['case'] for r in rows}=={'straight','arc','s_bend'}
        selection.append(dict(weight=weight, exits=sum(r['center_exits'] for r in rows),
                              rmse_sum=sum(r['rmse_m'] for r in rows),
                              steering_sum=sum(r['steer_rate_rms'] for r in rows)))
    assert min(selection,key=lambda r:(r['exits'],r['rmse_sum'],r['steering_sum']))['weight']==1
    for name in names:
        for row in evidence[name]:
            raw=trace(ROOT/'output'/name,row['case']+'_'+row['mode'])
            active=raw[(raw['station_m']>=35)&(raw['station_m']<=117)]
            np.testing.assert_allclose(np.percentile(active['compute_us'],95),row['compute_p95_us'],atol=1e-6)
    first,repeat=evidence['lqr-new'],evidence['lqr-new-repeat']
    assert len(first)==len(repeat)==6
    assert [(r['case'],r['mode'],r['center_exits']) for r in first]==[(r['case'],r['mode'],r['center_exits']) for r in repeat]
    evidence['selection']=selection
    evidence['validation_max_rmse_shift_m']=max(abs(a['rmse_m']-b['rmse_m']) for a,b in zip(first,repeat))
    (ROOT/'docs/lqr_reference.json').write_text(json.dumps(evidence,indent=2))
    fig,axes=plt.subplots(2,1,figsize=(8.4,6.4),constrained_layout=True)
    folder=ROOT/'output/lqr-new'
    ref=trace(folder,'lqr_new_s_reference')
    origin=ref['y'][0]
    axes[0].plot(ref['x'],ref['y']-origin,'k--',label='Reference')
    for mode,label,color in [('stanley_fixed','Stanley k=2','#bc6a2a'),('pursuit','Pure Pursuit','#426db4'),('lqr','Kinematic LQR','#007f7a')]:
        data=trace(folder,'lqr_new_s_'+mode)
        axes[0].plot(data['x'],data['y']-origin,color=color,label=label)
        axes[1].plot(data['station_m'],data['error_m'],color=color,label=label)
    axes[0].set(xlabel='World x (m)',ylabel='Relative y (m)',title='New S-bend | target 11 m/s | frozen parameters')
    axes[0].axis('equal')
    axes[0].legend()
    axes[1].set(xlim=(35,117),xlabel='OpenDRIVE station (m)',ylabel='Signed center error (m)')
    for ax in axes:
        ax.grid(True,alpha=.2)
    fig.savefig(ROOT/'docs/images/lqr_comparison.png',dpi=160)
    print('Verified all CSV metrics. Repeat RMSE shift:',evidence['validation_max_rmse_shift_m'])


if __name__=='__main__':
    main()
