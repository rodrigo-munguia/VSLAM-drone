#!/usr/bin/env python3
"""Offline A3e KF66 significance audit (hist path / GT / index sep)."""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

DEFAULT = Path(__file__).resolve().parents[2] / (
    "docs/evidence/gslam_visual_loop_a3e/raw/run_T_hybrid"
)


def load_admits(stream: Path) -> dict[int, dict]:
    admits: dict[int, dict] = {}
    with stream.open() as f:
        for line in f:
            o = json.loads(line)
            if o.get("event") in ("ekf_admit", "hybrid_admit", "absolute_admit"):
                admits[int(o["idx_kf"])] = o
    return admits


def chain_path(admits: dict[int, dict], i0: int, i1: int, key: str = "t_c2n") -> float:
    s = 0.0
    for i in range(i0, i1):
        if i not in admits or (i + 1) not in admits:
            continue
        a = np.asarray(admits[i][key][:3], dtype=float)
        b = np.asarray(admits[i + 1][key][:3], dtype=float)
        s += float(np.linalg.norm(b - a))
    return s


def nearest_gt(gt: np.ndarray, t: float):
    i = int(np.argmin(np.abs(gt[:, 0] - t)))
    return gt[i], abs(gt[i, 0] - t)


def main() -> int:
    run = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT
    admits = load_admits(run / "ba_d5" / "kf_pose_stream.jsonl")
    cur, matched = 76, 66
    if cur not in admits or matched not in admits:
        print("MISSING: admit records for KF66/KF76")
        return 1

    gt_rows = []
    with (run / "gt.csv").open() as f:
        for row in csv.DictReader(f):
            gt_rows.append(
                (float(row["t"]), float(row["x"]), float(row["y"]), float(row["z"]))
            )
    gt = np.asarray(gt_rows)

    t66 = admits[matched]["source_time_ms"] / 1000.0
    t76 = admits[cur]["source_time_ms"] / 1000.0
    g66, dt66 = nearest_gt(gt, t66)
    g76, dt76 = nearest_gt(gt, t76)
    mask = (gt[:, 0] >= t66) & (gt[:, 0] <= t76)
    seg = gt[mask]
    gt_path = (
        float(np.linalg.norm(np.diff(seg[:, 1:4], axis=0), axis=1).sum())
        if len(seg) > 1
        else float("nan")
    )

    print("A3e KF66 significance")
    print(f"  current_kf={cur} keyframe_id_admit={admits[cur]['keyframe_id']}")
    print(f"  matched_kf={matched} keyframe_id_admit={admits[matched]['keyframe_id']}")
    print(f"  commit_keyframe_id=6897 (from commits.csv / GlobalLoopCommitted)")
    print(f"  idx_sep={cur - matched}")
    print(f"  time_sep_s={t76 - t66:.3f}")
    print(f"  hist_path_t_c2n_m={chain_path(admits, matched, cur):.3f}")
    print(f"  hist_path_r_N_m={chain_path(admits, matched, cur, 'r_N'):.3f}")
    print(
        "  euclid_t_c2n_m="
        f"{np.linalg.norm(np.asarray(admits[cur]['t_c2n']) - np.asarray(admits[matched]['t_c2n'])):.3f}"
    )
    print(f"  GT_spatial_sep_m={np.linalg.norm(g76[1:4] - g66[1:4]):.3f} (dt66={dt66:.3f} dt76={dt76:.3f})")
    print(f"  GT_path_m={gt_path:.3f}")
    print(f"  matched_pos_t_c2n={admits[matched]['t_c2n']}")
    print(f"  current_pos_t_c2n={admits[cur]['t_c2n']}")
    print("  note: travel_dis=839.3 m is D (since last commit), NOT hist_path")
    ba = 10
    print(f"  ba_window=BA_max_n_kf_optimized={ba} → ba_limit≈{cur + 1 - ba}")
    print("  classification: short-history continuous overlap just outside BA window")
    return 0


if __name__ == "__main__":
    sys.exit(main())
