#!/usr/bin/env python3
"""Analyze A3d-0 smoke log for spawn + mission→ENU mapping."""
from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args()
    out = Path(args.out_dir)
    log = (out / 'launch_mission.log').read_text(errors='replace')

    key = []
    for l in log.splitlines():
        if any(
            s in l
            for s in (
                'go_point_unified mission:',
                'unified ENU target:',
                'SETTLED',
                'TakeOff',
                'no more commands',
                'Failed to load',
            )
        ):
            key.append(l[l.find('control') :] if 'control' in l else l[-180:])
    (out / 'key_events.txt').write_text('\n'.join(key) + '\n')

    enu = re.findall(
        r'unified ENU target:\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', log
    )
    mission = re.findall(
        r'go_point_unified mission:\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', log
    )
    summary = [
        f'missions={mission}',
        f'enu_targets={enu}',
    ]

    gt = []
    gp = out / 'gt_samples.csv'
    if gp.exists() and gp.stat().st_size > 10:
        with gp.open() as f:
            for row in csv.DictReader(f):
                gt.append((float(row['x']), float(row['y']), float(row['z'])))
    if gt:
        xs = [g[0] for g in gt]
        ys = [g[1] for g in gt]
        zs = [g[2] for g in gt]
        summary += [
            f'gt_n={len(gt)}',
            f'gt_x=[{min(xs):.3f},{max(xs):.3f}]',
            f'gt_y=[{min(ys):.3f},{max(ys):.3f}]',
            f'gt_z=[{min(zs):.3f},{max(zs):.3f}]',
            f'gt_dx={max(xs) - min(xs):.3f} gt_dy={max(ys) - min(ys):.3f}',
        ]
    else:
        summary.append('gt_n=0')

    ok = True
    reasons = []
    if 'no more commands to execute' not in log:
        ok = False
        reasons.append('plan_incomplete')
    if 'Failed to load' in log:
        ok = False
        reasons.append('component_load_fail')
    if len(mission) < 3:
        ok = False
        reasons.append('missing_mission_cmds')

    # Expect: mission (0,0,60) → ENU (-190,-190,~75.75)
    #         mission (20,0,60) → ENU (-170,-190,~75.75)
    if enu:
        e0 = list(map(float, enu[0]))
        if abs(e0[0] - (-190)) > 5 or abs(e0[1] - (-190)) > 5:
            ok = False
            reasons.append(f'first_enu_unexpected={e0}')
        if len(enu) >= 2:
            e1 = list(map(float, enu[1]))
            if abs(e1[0] - (-170)) > 8 or abs(e1[1] - (-190)) > 8:
                ok = False
                reasons.append(f'hop_enu_unexpected={e1}')
            # Hop should be +20 m in world X
            if abs((e1[0] - e0[0]) - 20.0) > 1.0:
                ok = False
                reasons.append(f'hop_delta_x_unexpected={e1[0] - e0[0]}')
    else:
        ok = False
        reasons.append('no_enu_targets')

    summary.append('PASS' if ok else 'FAIL')
    summary.append('reasons=' + (','.join(reasons) if reasons else 'none'))
    print('\n'.join(summary))


if __name__ == '__main__':
    main()
