#!/usr/bin/env python3
"""Analyze A3d speed smoke: cruise speed, settle, SLAM health."""
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

import numpy as np


def load(path: Path, cols: int):
    if not path.exists() or path.stat().st_size < 10:
        return np.zeros((0, cols))
    rows = []
    with path.open() as f:
        for row in csv.DictReader(f):
            rows.append([float(row[k]) for k in row.keys()])
    return np.asarray(rows, dtype=float) if rows else np.zeros((0, cols))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--target-v', type=float, default=5.0)
    args = ap.parse_args()
    out = Path(args.out_dir)
    log = (out / 'launch_mission.log').read_text(errors='replace')

    odom = load(out / 'odom.csv', 7)
    vel = load(out / 'body_velocity.csv', 4)
    gt = load(out / 'gt.csv', 4)

    # Prefer odom twist for ground-speed (public frame)
    if len(odom):
        spd = np.linalg.norm(odom[:, 4:6], axis=1)
        # cruise samples: ignore takeoff/settle by speed > 1.5
        cruise = spd[spd > 1.5]
        peak = float(spd.max()) if len(spd) else float('nan')
        med_cruise = float(np.median(cruise)) if len(cruise) else float('nan')
        p90 = float(np.percentile(cruise, 90)) if len(cruise) else float('nan')
    else:
        peak = med_cruise = p90 = float('nan')
        cruise = np.array([])

    if len(vel):
        bspd = np.linalg.norm(vel[:, 1:3], axis=1)
        b_cruise = bspd[bspd > 1.5]
        b_med = float(np.median(b_cruise)) if len(b_cruise) else float('nan')
        b_peak = float(bspd.max()) if len(bspd) else float('nan')
    else:
        b_med = b_peak = float('nan')

    settles = re.findall(
        r'unified SETTLED id=(\d+) e=\(([-\d.eE+]+),([-\d.eE+]+),([-\d.eE+]+)\) vh=([-\d.eE+]+)',
        log,
    )
    settle_xy = []
    settle_vh = []
    for s in settles:
        ex, ey = float(s[1]), float(s[2])
        settle_xy.append(math.hypot(ex, ey))
        settle_vh.append(abs(float(s[4])))

    # LocalSLAM / visual health proxies from log
    n_feat_lines = len(re.findall(r'n matches:|features|visual updates', log, re.I))
    enable_true = 'localslam-> ~/enable true' in log or 'Activate SLAM visual' in log
    stop_ok = 'no more commands to execute' in log
    rejected = log.count('REJECTED') + log.count('traj REJECTED')
    failed = log.count('FAILED') + log.count('Failed to load')

    # GT path length / duration if available
    if len(gt) > 1:
        dxy = np.linalg.norm(np.diff(gt[:, 1:3], axis=0), axis=1).sum()
        dur = float(gt[-1, 0] - gt[0, 0])
    elif len(odom) > 1:
        dxy = np.linalg.norm(np.diff(odom[:, 1:3], axis=0), axis=1).sum()
        dur = float(odom[-1, 0] - odom[0, 0])
    else:
        dxy = dur = float('nan')

    # Drift proxy: final odom vs first (mission returns to origin) — use GT if present
    final_err = float('nan')
    if len(gt) > 10:
        # public GT after SLAM: expect near start XY at end
        final_err = float(np.linalg.norm(gt[-1, 1:3] - gt[0, 1:3]))
    elif len(odom) > 10:
        final_err = float(np.linalg.norm(odom[-1, 1:3] - odom[0, 1:3]))

    tgt = args.target_v
    reasons = []
    ok = True
    if not stop_ok:
        ok = False
        reasons.append('plan_incomplete')
    if failed:
        ok = False
        reasons.append(f'failures={failed}')
    if rejected:
        ok = False
        reasons.append(f'traj_rejected={rejected}')
    if not enable_true:
        reasons.append('slam_not_enabled')  # soft
    if not (len(cruise) > 20 and med_cruise == med_cruise):
        ok = False
        reasons.append('insufficient_cruise_samples')
    else:
        # Accept if median cruise within 20% of target, or peak reaches ≥0.85*target
        if med_cruise < 0.8 * tgt and peak < 0.85 * tgt:
            ok = False
            reasons.append(f'cruise_too_slow med={med_cruise:.2f} peak={peak:.2f}')
        if med_cruise > 1.25 * tgt:
            reasons.append(f'cruise_faster_than_target med={med_cruise:.2f}')
        if peak > 1.4 * tgt:
            ok = False
            reasons.append(f'excessive_speed_peak={peak:.2f}')
    if settle_xy:
        if max(settle_xy) > 3.0:
            ok = False
            reasons.append(f'settle_xy_large={max(settle_xy):.2f}')
        if max(settle_vh) > 1.0:
            reasons.append(f'settle_vh={max(settle_vh):.2f}')  # soft warn

    lines = [
        f'target_v={tgt}',
        f'odom_n={len(odom)} vel_n={len(vel)} gt_n={len(gt)}',
        f'odom_cruise_median={med_cruise}',
        f'odom_cruise_p90={p90}',
        f'odom_speed_peak={peak}',
        f'body_cruise_median={b_med}',
        f'body_speed_peak={b_peak}',
        f'path_len_xy≈{dxy}',
        f'duration_s≈{dur}',
        f'final_xy_err_proxy={final_err}',
        f'settles={len(settles)} max_settle_xy={max(settle_xy) if settle_xy else None} max_settle_vh={max(settle_vh) if settle_vh else None}',
        f'slam_enabled={enable_true} feat_log_hits={n_feat_lines}',
        f'PASS' if ok else 'FAIL',
        f'reasons={",".join(reasons) if reasons else "none"}',
        f'validated_cruise_mps={med_cruise if ok else "n/a"}',
    ]
    text = '\n'.join(lines) + '\n'
    (out / 'speed_verdict.txt').write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
