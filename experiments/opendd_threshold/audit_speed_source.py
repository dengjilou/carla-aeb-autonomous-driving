"""Replay the frozen rdb5 candidate decisions with three speed measurements."""
import argparse
import json
from bisect import bisect_right
from collections import defaultdict
from itertools import pairwise
from math import ceil, floor, hypot
from pathlib import Path

from mine import ROOT, find_visits, mine, read_frames, speed_drop

COHORTS = {'development': range(333,343), 'earlier': range(343,363), 'heldout': range(363,393)}


def resample(points):
    # Fixed grid relative to recording time zero; no endpoint extrapolation.
    times = [p[0] for p in points]
    for tick in range(ceil(20 * times[0]), floor(20 * times[-1]) + 1):
        t = tick / 20
        i = min(bisect_right(times, t) - 1, len(points) - 2)
        t0, x, y = points[i]
        t1, xx, yy = points[i+1]
        weight = (t-t0) / (t1-t0)
        yield t, x + weight*(xx-x), y + weight*(yy-y)


def best_drop(points, start, stop, rules, grid):
    duration = rules['speed']['drop_window_s']
    gap = rules['max_frame_gap_s']
    peaks = []
    # Break gaps before both interpolation and differencing.
    cuts = [0] + [i for i in range(1,len(points)) if points[i][0]-points[i-1][0] > gap] + [len(points)]
    for a,b in pairwise(cuts):
        segment = points[a:b]
        if len(segment)<2 or segment[-1][0]<start or segment[0][0]>stop:
            continue
        samples = []
        for (t,x,y),(tt,xx,yy) in pairwise(resample(segment) if grid else segment):
            middle = (t+tt)/2
            if start <= middle <= stop:
                samples.append((middle, hypot(xx-x,yy-y)/(tt-t)))
        peak = speed_drop(samples,duration,gap)
        if peak:
            peaks.append(peak)
    return max(peaks,key=lambda p:p['drop_mps'],default=None)


def audit(database, numbers=COHORTS['heldout']):
    config = json.loads((ROOT/'zones.json').read_text(encoding='utf-8'))
    rules,zone = config['rules'],config['sites']['rdb5']
    threshold = rules['speed']['review_drop_mps']
    records = []
    for number in numbers:
        recording,frames,classes = read_frames(database,f'rdb5_{number}')
        visits = find_visits(frames,classes,zone,rules['max_frame_gap_s'])
        pairs = mine(frames,visits,rules)
        vehicles = {e[k] for e in pairs for k in ('first','second')}
        tracks = defaultdict(list)
        for t,frame in frames.items():
            for obj,p in frame.items():
                if obj in vehicles:
                    tracks[obj].append((t,p[0],p[1]))
        for e in pairs:
            peaks = {m:[] for m in ('V','native_xy','xy_50ms')}
            for k in ('first','second'):
                peaks['V'].append(e[k+'_speed_drop'])
                enter,gate = e[k+'_enter'],e[k+'_gate_s']
                start = (enter if gate is None else gate)-rules['speed']['pre_gate_s']
                points = tracks[e[k]]
                for method,grid in [('native_xy',False),('xy_50ms',True)]:
                    peaks[method].append(best_drop(points,start,enter,rules,grid))
            speed = {m:any(p and p['drop_mps']>=threshold for p in v) for m,v in peaks.items()}
            review = {m:e['path_label']=='entry_and_circulating' and (e['pet_hit'] or hit) for m,hit in speed.items()}
            if review['V']!=e['merge_review_hit'] or speed['V']!=e['speed_review_hit']:
                raise ValueError('Frozen decision mismatch')
            records.append(dict(recording=recording,id=e['id'],vehicles=[e['first'],e['second']],
                                path=e['path_label'],pet_s=e['pet_s'],time_hit=e['pet_hit'],peaks=peaks,
                                speed_hit=speed,review=review))
    return records


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('database',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--cohort',choices=COHORTS,default='heldout')
    args=parser.parse_args()
    rows=audit(args.database,COHORTS[args.cohort])
    totals={m:{key:sum(r[key][m] for r in rows) for key in ('speed_hit','review')}
            for m in ('V','native_xy','xy_50ms')}
    archived={'heldout':'heldout_rdb5_363_392','earlier':'earlier_rdb5_343_362'}
    if args.cohort in archived:
        saved=json.loads((ROOT/'speed_evaluation.json').read_text())[archived[args.cohort]]
        if (len(rows),totals['V']['speed_hit'],totals['V']['review']) != (
                saved['pairs'],saved['speed_hits'],saved['merge_review_hits']):
            raise ValueError('Source cohort differs from the archived evaluation')
    args.output.write_text(json.dumps(dict(pairs=len(rows),totals=totals,rows=rows),indent=2)+'\n',encoding='utf-8')
    print(totals)
