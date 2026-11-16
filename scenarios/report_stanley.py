"""Recompute the Stanley ablation, gain selection and frozen validation evidence."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from report_lateral import verify, trace

ROOT = Path(__file__).resolve().parents[1]


def main():
    folders = ['stanley-ablation','stanley-gain-1','stanley-gain-2',
               'stanley-known','stanley-new','stanley-new-repeat']
    evidence = {name: verify(ROOT/'output'/name) for name in folders}
    candidates = []
    for name, gain in [('stanley-ablation',0.3),('stanley-gain-1',1),('stanley-gain-2',2)]:
        rows = [r for r in evidence[name] if r['mode']=='stanley_fixed']
        assert {r['case'] for r in rows}=={'straight','arc','s_bend'}
        candidates.append(dict(gain=gain, exits=sum(r['center_exits'] for r in rows),
                               rmse_sum=sum(r['rmse_m'] for r in rows),
                               steer_rate_sum=sum(r['steer_rate_rms'] for r in rows)))
    chosen = min(candidates,key=lambda r:(r['exits'],r['rmse_sum'],r['steer_rate_sum']))
    assert chosen['gain']==2
    a, b = evidence['stanley-new'], evidence['stanley-new-repeat']
    assert [(r['case'],r['mode'],r['center_exits']) for r in a]==[(r['case'],r['mode'],r['center_exits']) for r in b]
    shift=max(abs(x['rmse_m']-y['rmse_m']) for x,y in zip(a,b))
    evidence['selection']=candidates
    evidence['validation_max_rmse_shift_m']=shift
    (ROOT/'docs/stanley_reference.json').write_text(json.dumps(evidence,indent=2))
    fig, axes=plt.subplots(2,1,figsize=(8.4,6.4),constrained_layout=True)
    folder=ROOT/'output/stanley-known'
    reference=trace(folder,'holdout_s_reference')
    origin=reference['y'][0]
    axes[0].plot(reference['x'],reference['y']-origin,'k--',label='Reference')
    for mode,label,color in [('stanley','Original Stanley','#b75327'),
                             ('stanley_fixed','Corrected Stanley k=2','#007b7f')]:
        data=trace(folder,'holdout_s_'+mode)
        axes[0].plot(data['x'],data['y']-origin,color=color,label=label)
        axes[1].plot(data['station_m'],data['error_m'],color=color,label=label)
    axes[0].set(xlabel='World x (m)',ylabel='Relative y (m)',title='Known S-bend | target 10 m/s | same vehicle and PID')
    axes[0].axis('equal')
    axes[0].legend()
    for value in [-1.75,1.75]:
        axes[1].axhline(value,color='gray',linestyle=':')
    axes[1].set(xlim=(35,117),xlabel='OpenDRIVE station (m)',ylabel='Signed center error (m)')
    for ax in axes:
        ax.grid(True,alpha=.2)
    fig.savefig(ROOT/'docs/images/stanley_correction.png',dpi=160)
    print('Verified all traces. Validation max repeated RMSE shift:',shift)


if __name__=='__main__':
    main()
