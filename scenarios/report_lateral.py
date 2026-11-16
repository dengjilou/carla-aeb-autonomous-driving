"""Compare two completed runs, check logged metrics and plot a held-out S-bend."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def trace(folder, name):
    return np.genfromtxt(folder / (name+'.csv'), names=True, delimiter=',')


def verify(folder):
    results = json.loads((folder/'summary.json').read_text())
    for row in results:
        raw = trace(folder, row['case']+'_'+row['mode'])
        active = raw[(raw['station_m'] >= 35) & (raw['station_m'] <= 117)]
        error = np.hypot(active['x']-active['ref_x'], active['y']-active['ref_y'])
        np.testing.assert_allclose(error, abs(active['error_m']), atol=1e-6)
        assert len(active) == row['frames'] and raw['station_m'][-1] >= 121
        outside = error > 1.75
        exits = int(outside[0])+np.count_nonzero(outside[1:] & ~outside[:-1])
        assert exits == row['center_exits']
        measured = [np.sqrt(np.mean(error**2)), np.percentile(error, 95),
                    np.sqrt(np.mean((np.diff(active['steer'])/np.diff(active['time_s']))**2))]
        expected = [row[k] for k in ('rmse_m','p95_m','steer_rate_rms')]
        np.testing.assert_allclose(measured, expected, atol=1e-6)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('first', type=Path)
    parser.add_argument('repeat', type=Path)
    parser.add_argument('image', type=Path)
    args = parser.parse_args()
    first, repeat = verify(args.first), verify(args.repeat)
    keys = lambda rows: [(r['case'], r['mode'], r['center_exits']) for r in rows]
    assert len(first) == 10 and keys(first) == keys(repeat)
    print('Max repeated RMSE shift (m):', max(abs(a['rmse_m']-b['rmse_m']) for a,b in zip(first,repeat)))
    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.4), constrained_layout=True)
    ref = trace(args.first, 'holdout_s_reference')
    origin = ref['y'][0]
    axes[0].plot(ref['x'], ref['y']-origin, '--', color='black', label='Reference')
    for mode, label, color in [('stanley','Stanley','#bb5c2c'),('pursuit','Pure Pursuit','#007f83')]:
        data = trace(args.first, 'holdout_s_'+mode)
        axes[0].plot(data['x'], data['y']-origin, color=color, label=label)
        axes[1].plot(data['station_m'], data['error_m'], color=color, label=label)
    axes[0].axis('equal')
    axes[0].set(xlabel='World x (m)', ylabel='Relative y (m)', title='Held-out S-bend | target speed 10 m/s')
    axes[0].legend()
    for limit in (-1.75, 1.75):
        axes[1].axhline(limit, color='gray', linestyle=':')
    axes[1].set(xlim=(35,117), xlabel='OpenDRIVE station (m)', ylabel='Signed lateral error (m)')
    for ax in axes:
        ax.grid(True, alpha=.2)
    fig.savefig(args.image, dpi=160)


if __name__ == '__main__':
    main()
