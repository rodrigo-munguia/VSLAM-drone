#!/usr/bin/env python3
"""A3c offline ATE from recorder NPZ/CSV. SE(3)-free translational RMSE vs GT.

Uses timestamp association with max_dt. Does not Umeyama-align segments after
loop commits (preserves correction effect). Optional single rigid XY yaw+trans
alignment of entire trajectory using the first N seconds (shared policy).
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def load_xyz(csv_path: Path) -> np.ndarray:
    if not csv_path.exists():
        return np.zeros((0, 4))
    rows = []
    with csv_path.open() as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append([float(row['t']), float(row['x']), float(row['y']), float(row['z'])])
    return np.array(rows, dtype=np.float64) if rows else np.zeros((0, 4))


def associate(est: np.ndarray, gt: np.ndarray, max_dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Nearest GT stamp for each est sample within max_dt."""
    if len(est) == 0 or len(gt) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    gt_t = gt[:, 0]
    e_xyz, g_xyz = [], []
    for row in est:
        j = np.searchsorted(gt_t, row[0])
        cands = []
        if j < len(gt):
            cands.append(j)
        if j > 0:
            cands.append(j - 1)
        best = None
        best_dt = max_dt
        for k in cands:
            dt = abs(gt_t[k] - row[0])
            if dt <= best_dt:
                best_dt = dt
                best = k
        if best is not None:
            e_xyz.append(row[1:4])
            g_xyz.append(gt[best, 1:4])
    return np.asarray(e_xyz), np.asarray(g_xyz)


def yaw_xy_align(est: np.ndarray, gt: np.ndarray):
    """Rigid yaw+XY translation aligning est→gt (Z unchanged)."""
    if len(est) < 2:
        return est.copy(), {'yaw': 0.0, 'tx': 0.0, 'ty': 0.0}
    e = est[:, :2]
    g = gt[:, :2]
    mu_e = e.mean(axis=0)
    mu_g = g.mean(axis=0)
    E = e - mu_e
    G = g - mu_g
    H = E.T @ G
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_g - R @ mu_e
    out = est.copy()
    out[:, :2] = (R @ e.T).T + t
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    return out, {'yaw': yaw, 'tx': float(t[0]), 'ty': float(t[1])}


def metrics(est: np.ndarray, gt: np.ndarray) -> dict:
    if len(est) == 0:
        return {
            'n': 0,
            'ate_rmse': None,
            'ate_rmse_xy': None,
            'ate_rmse_z': None,
            'final_err': None,
            'final_err_xy': None,
            'max_err': None,
            'max_err_xy': None,
            'mean_err': None,
        }
    d = est - gt
    nrm = np.linalg.norm(d, axis=1)
    nrm_xy = np.linalg.norm(d[:, :2], axis=1)
    nrm_z = np.abs(d[:, 2])
    return {
        'n': int(len(est)),
        'ate_rmse': float(np.sqrt((nrm ** 2).mean())),
        'ate_rmse_xy': float(np.sqrt((nrm_xy ** 2).mean())),
        'ate_rmse_z': float(np.sqrt((nrm_z ** 2).mean())),
        'final_err': float(nrm[-1]),
        'final_err_xy': float(nrm_xy[-1]),
        'max_err': float(nrm.max()),
        'max_err_xy': float(nrm_xy.max()),
        'mean_err': float(nrm.mean()),
    }


def path_vs_gt(path_xyz: np.ndarray, gt: np.ndarray) -> dict:
    """Match global keyframes to nearest GT by XY (no stamps on path)."""
    if path_xyz is None or len(path_xyz) == 0 or len(gt) == 0:
        return metrics(np.zeros((0, 3)), np.zeros((0, 3)))
    gxy = gt[:, 1:3]
    e, g = [], []
    for p in path_xyz:
        d = np.linalg.norm(gxy - p[:2], axis=1)
        k = int(np.argmin(d))
        e.append(p)
        g.append(gt[k, 1:4])
    return metrics(np.asarray(e), np.asarray(g))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--max-dt', type=float, default=0.15)
    ap.add_argument('--align-sec', type=float, default=15.0,
                    help='Use first align-sec of associated samples for yaw+XY align')
    args = ap.parse_args()
    d = Path(args.run_dir)
    gt = load_xyz(d / 'gt.csv')
    odom = load_xyz(d / 'odom.csv')
    e, g = associate(odom, gt, args.max_dt)
    report = {'max_dt': args.max_dt, 'n_gt': len(gt), 'n_odom': len(odom)}

    # raw (no align)
    report['localslam_raw'] = metrics(e, g)

    # shared align policy: fit on early window, apply to all
    if len(e) > 5:
        # time from odom association approximate: use index fraction for first align_sec
        # Prefer true time: rebuild with times
        e_t, g_t = [], []
        gt_t = gt[:, 0]
        for row in odom:
            j = np.searchsorted(gt_t, row[0])
            cands = [c for c in (j, j - 1) if 0 <= c < len(gt)]
            best = None
            best_dt = args.max_dt
            for k in cands:
                dt = abs(gt_t[k] - row[0])
                if dt <= best_dt:
                    best_dt = dt
                    best = k
            if best is not None:
                e_t.append([row[0], *row[1:4]])
                g_t.append([gt[best, 0], *gt[best, 1:4]])
        e_t = np.asarray(e_t)
        g_t = np.asarray(g_t)
        t0 = e_t[0, 0]
        mask = e_t[:, 0] <= (t0 + args.align_sec)
        if mask.sum() >= 5:
            _, align = yaw_xy_align(e_t[mask, 1:4], g_t[mask, 1:4])
        else:
            _, align = yaw_xy_align(e_t[:, 1:4], g_t[:, 1:4])
        e_al, _ = yaw_xy_align(e_t[:, 1:4], g_t[:, 1:4])  # full fit for reference
        # apply early-window transform manually
        yaw = align['yaw']
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        e_fixed = e_t[:, 1:4].copy()
        e_fixed[:, :2] = (R @ e_t[:, 1:3].T).T + np.array([align['tx'], align['ty']])
        report['align'] = align
        report['localslam_aligned'] = metrics(e_fixed, g_t[:, 1:4])
        # error vs distance traveled
        if len(e_fixed) > 1:
            step = np.linalg.norm(np.diff(g_t[:, 1:4], axis=0), axis=1)
            dist = np.concatenate([[0.0], np.cumsum(step)])
            err = np.linalg.norm(e_fixed - g_t[:, 1:4], axis=1)
            report['drift_vs_distance'] = {
                'dist_final_m': float(dist[-1]),
                'err_at_25pct': float(err[np.searchsorted(dist, 0.25 * dist[-1])]),
                'err_at_50pct': float(err[np.searchsorted(dist, 0.50 * dist[-1])]),
                'err_at_75pct': float(err[np.searchsorted(dist, 0.75 * dist[-1])]),
                'err_final': float(err[-1]),
            }
    else:
        report['localslam_aligned'] = report['localslam_raw']

    # Global path final
    gpath = d / 'global_path_final.npy'
    if gpath.exists():
        gp = np.load(gpath)
        report['globalslam_path_vs_nearest_gt'] = path_vs_gt(gp, gt)
        # also apply same early yaw align if available
        if 'align' in report:
            yaw = report['align']['yaw']
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([[c, -s], [s, c]])
            gp2 = gp.copy()
            gp2[:, :2] = (R @ gp[:, :2].T).T + np.array(
                [report['align']['tx'], report['align']['ty']]
            )
            report['globalslam_path_aligned_vs_nearest_gt'] = path_vs_gt(gp2, gt)

    commits_csv = d / 'commits.csv'
    if commits_csv.exists():
        with commits_csv.open() as f:
            report['commits'] = list(csv.DictReader(f))

    out = d / 'ate_summary.json'
    out.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
