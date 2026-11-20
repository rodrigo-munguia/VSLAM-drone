#!/usr/bin/env python3
"""KF-Z-A3 offline analysis: ATE Z/XY, admit diag, BA-D5 Z, landmarks, plots."""
from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from a3c_analyze_ate import associate, load_xyz, metrics, yaw_xy_align


def load_txyz(csv_path: Path) -> np.ndarray:
    return load_xyz(csv_path)


def z_metrics(est: np.ndarray, gt: np.ndarray, max_dt: float = 0.15) -> dict:
    e, g = associate(est, gt, max_dt)
    if len(e) == 0:
        return {"n": 0}
    ez = e[:, 2] - g[:, 2]
    exy = np.linalg.norm(e[:, :2] - g[:, :2], axis=1)
    e3 = np.linalg.norm(e - g, axis=1)
    return {
        "n": int(len(e)),
        "ate_rmse": float(np.sqrt(np.mean(e3**2))),
        "ate_rmse_xy": float(np.sqrt(np.mean(exy**2))),
        "ate_rmse_z": float(np.sqrt(np.mean(ez**2))),
        "max_abs_z": float(np.max(np.abs(ez))),
        "final_z_err": float(ez[-1]),
        "mean_abs_z": float(np.mean(np.abs(ez))),
        "max_err": float(e3.max()),
        "final_err": float(e3[-1]),
    }


def align_full(est: np.ndarray, gt: np.ndarray):
    e, g = associate(est, gt, 0.15)
    if len(e) < 10:
        return e, g, {}
    n0 = max(50, len(e) // 20)
    _, al = yaw_xy_align(e[:n0], g[:n0])
    yaw, tx, ty = al["yaw"], al["tx"], al["ty"]
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    e2 = e.copy()
    e2[:, :2] = (R @ e[:, :2].T).T + np.array([tx, ty])
    return e2, g, al


def path_length_xy(xyz: np.ndarray) -> np.ndarray:
    if len(xyz) < 2:
        return np.zeros(len(xyz))
    d = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def parse_admit_diag(log_path: Path) -> list[dict]:
    if not log_path.exists():
        return []
    rows = []
    pat = re.compile(
        r"kf_z_admit mode=(\S+) idx=(\d+) id=(\d+) "
        r"z_ekf=([-\d.eE]+) z_full_rel=([-\d.eE]+) "
        r"z_admit=([-\d.eE]+) z_gs_prev=([-\d.eE]+) dz_admit_ekf=([-\d.eE]+)"
    )
    for line in log_path.read_text(errors="replace").splitlines():
        m = pat.search(line)
        if not m:
            continue
        rows.append(
            {
                "mode": m.group(1),
                "idx": int(m.group(2)),
                "id": int(m.group(3)),
                "z_ekf": float(m.group(4)),
                "z_full_rel": float(m.group(5)),
                "z_admit": float(m.group(6)),
                "z_gs_prev": float(m.group(7)),
                "dz_init": float(m.group(8)),
            }
        )
    return rows


def parse_ba_d5(ba_dir: Path) -> dict:
    stream = ba_dir / "kf_pose_stream.jsonl"
    admits = []
    commits = []
    if not stream.exists():
        return {"admits": admits, "ba_commits": commits, "dz_ba": []}
    for line in stream.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        ev = o.get("event", "")
        if ev in ("ekf_admit", "rel_admit", "hybrid_admit"):
            admits.append(o)
        elif ev == "ba_commit":
            commits.append(o)
    # Pair ba_commit with last prior admit/pose for same idx_kf in stream order
    last_z: dict[int, float] = {}
    dz_ba = []
    for line in stream.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue
        idx = int(o.get("idx_kf", -1))
        t = o.get("t_c2n")
        if not t or idx < 0:
            continue
        z = float(t[2])
        ev = o.get("event", "")
        if ev == "ba_commit":
            z_pre = last_z.get(idx)
            if z_pre is not None:
                dz_ba.append(
                    {
                        "idx_kf": idx,
                        "keyframe_id": o.get("keyframe_id"),
                        "ba_id": o.get("ba_id"),
                        "z_pre": z_pre,
                        "z_post": z,
                        "dz_ba": z - z_pre,
                    }
                )
            last_z[idx] = z
        else:
            last_z[idx] = z
    return {"admits": admits, "ba_commits": commits, "dz_ba": dz_ba}


def landmark_z_stats(xyz: np.ndarray) -> dict:
    if xyz is None or len(xyz) == 0:
        return {"n": 0}
    z = xyz[:, 2]
    med = float(np.median(z))
    return {
        "n": int(len(z)),
        "z_median": med,
        "z_p05": float(np.percentile(z, 5)),
        "z_p95": float(np.percentile(z, 95)),
        "z_min": float(z.min()),
        "z_max": float(z.max()),
        "n_elevated_med_plus15": int(np.sum(z > med + 15)),
        "n_deep_med_minus50": int(np.sum(z < med - 50)),
    }


def analyze_run(run_dir: Path) -> dict:
    gt = load_txyz(run_dir / "gt.csv")
    odom = load_txyz(run_dir / "odom.csv")
    gkf = load_txyz(run_dir / "global_kf.csv")
    if len(gkf) == 0 and (run_dir / "global_path_final.npy").exists():
        # fallback untimed
        xyz = np.load(run_dir / "global_path_final.npy")
        gkf = np.column_stack([np.arange(len(xyz), dtype=np.float64), xyz])

    e_loc, g_loc, al = align_full(odom, gt)
    loc = z_metrics(
        np.column_stack([np.arange(len(e_loc)), e_loc]) if len(e_loc) else np.zeros((0, 4)),
        np.column_stack([np.arange(len(g_loc)), g_loc]) if len(g_loc) else np.zeros((0, 4)),
        max_dt=1e9,
    )
    # Recompute properly with associate using original stamps after align
    e0, g0 = associate(odom, gt, 0.15)
    if len(e0) >= 10:
        n0 = max(50, len(e0) // 20)
        _, al = yaw_xy_align(e0[:n0], g0[:n0])
        yaw, tx, ty = al["yaw"], al["tx"], al["ty"]
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        e_al = e0.copy()
        e_al[:, :2] = (R @ e0[:, :2].T).T + np.array([tx, ty])
        ez = e_al[:, 2] - g0[:, 2]
        exy = np.linalg.norm(e_al[:, :2] - g0[:, :2], axis=1)
        e3 = np.linalg.norm(e_al - g0, axis=1)
        loc = {
            "n": int(len(e_al)),
            "ate_rmse": float(np.sqrt(np.mean(e3**2))),
            "ate_rmse_xy": float(np.sqrt(np.mean(exy**2))),
            "ate_rmse_z": float(np.sqrt(np.mean(ez**2))),
            "max_abs_z": float(np.max(np.abs(ez))),
            "final_z_err": float(ez[-1]),
            "mean_abs_z": float(np.mean(np.abs(ez))),
            "max_err": float(e3.max()),
            "final_err": float(e3[-1]),
            "align": al,
        }

    # Global: timestamp-matched if stamps look real (not 0..N)
    glob = {"n": 0, "note": "no global_kf"}
    if len(gkf) and len(gt):
        # If t looks like indices (0..n), treat as untimed nearest-XY fallback
        if gkf[-1, 0] < 100 and gkf[0, 0] < 1.0 and len(gkf) > 5:
            # untimed fallback — report separately
            gxy = gt[:, 1:3]
            errs_z, errs_xy, errs = [], [], []
            for p in gkf[:, 1:4]:
                j = int(np.argmin(np.linalg.norm(gxy - p[:2], axis=1)))
                errs_z.append(p[2] - gt[j, 3])
                errs_xy.append(np.linalg.norm(p[:2] - gt[j, 1:3]))
                errs.append(np.linalg.norm(p - gt[j, 1:4]))
            errs_z = np.asarray(errs_z)
            errs_xy = np.asarray(errs_xy)
            errs = np.asarray(errs)
            glob = {
                "n": int(len(errs)),
                "method": "nearest_gt_xy_untimed",
                "ate_rmse": float(np.sqrt(np.mean(errs**2))),
                "ate_rmse_xy": float(np.sqrt(np.mean(errs_xy**2))),
                "ate_rmse_z": float(np.sqrt(np.mean(errs_z**2))),
                "max_abs_z": float(np.max(np.abs(errs_z))),
                "final_z_err": float(errs_z[-1]),
                "mean_abs_z": float(np.mean(np.abs(errs_z))),
            }
        else:
            e_g, g_g = associate(gkf, gt, 0.25)
            if len(e_g) >= 2:
                n0 = max(5, len(e_g) // 20)
                _, al_g = yaw_xy_align(e_g[:n0], g_g[:n0])
                yaw, tx, ty = al_g["yaw"], al_g["tx"], al_g["ty"]
                c, s = np.cos(yaw), np.sin(yaw)
                R = np.array([[c, -s], [s, c]])
                e_al = e_g.copy()
                e_al[:, :2] = (R @ e_g[:, :2].T).T + np.array([tx, ty])
                ez = e_al[:, 2] - g_g[:, 2]
                exy = np.linalg.norm(e_al[:, :2] - g_g[:, :2], axis=1)
                e3 = np.linalg.norm(e_al - g_g, axis=1)
                dist = path_length_xy(g_g)
                glob = {
                    "n": int(len(e_al)),
                    "method": "timestamp_match+early_yaw_xy_align",
                    "ate_rmse": float(np.sqrt(np.mean(e3**2))),
                    "ate_rmse_xy": float(np.sqrt(np.mean(exy**2))),
                    "ate_rmse_z": float(np.sqrt(np.mean(ez**2))),
                    "max_abs_z": float(np.max(np.abs(ez))),
                    "final_z_err": float(ez[-1]),
                    "mean_abs_z": float(np.mean(np.abs(ez))),
                    "max_err": float(e3.max()),
                    "final_err": float(e3[-1]),
                    "align": al_g,
                    "z_err_series": ez.tolist(),
                    "dist_series": dist.tolist(),
                    "z_est_series": e_al[:, 2].tolist(),
                    "z_gt_series": g_g[:, 2].tolist(),
                }

    admits = parse_admit_diag(run_dir / "launch_mission.log")
    ba = parse_ba_d5(run_dir / "ba_d5")

    dz_init = [a["dz_init"] for a in admits]
    adj_jump = []
    for i in range(1, len(admits)):
        adj_jump.append(admits[i]["z_admit"] - admits[i - 1]["z_admit"])

    dz_ba = [d["dz_ba"] for d in ba["dz_ba"]]

    # Landmarks from final snap if present
    lm = None
    for name in ("global_landmarks_final.npy",):
        p = run_dir / name
        if p.exists():
            lm = np.load(p)
    # from trajectories — recorder may not save final lm; use last commit empty
    # parse PointCloud not available; use landmark height from ba if needed
    lm_stats = landmark_z_stats(lm) if lm is not None else {"n": 0, "note": "no final landmark dump"}

    # Try load from any commit landmarks
    lm_files = sorted(run_dir.glob("commit*_landmarks_post.npy"))
    if not lm_files:
        lm_files = sorted(run_dir.glob("*landmarks*.npy"))
    # Also save from recorder — check if we can get from npz; else leave

    ba_reject = 0
    log = (run_dir / "launch_mission.log").read_text(errors="replace") if (run_dir / "launch_mission.log").exists() else ""
    ba_reject = len(re.findall(r"BA update rejected \(Delta_norm gate\)", log))
    ba_delta_large = len(re.findall(r"Delta_norm for n-kf optimized too large", log))

    # Hybrid mode check from ready line
    hybrid_ready = None
    m = re.search(r"hybrid_z_abs=(\d)", log)
    if m:
        hybrid_ready = int(m.group(1))

    return {
        "run_dir": str(run_dir),
        "hybrid_z_abs_ready": hybrid_ready,
        "n_gt": int(len(gt)),
        "n_odom": int(len(odom)),
        "n_gkf": int(len(gkf)),
        "localslam": loc,
        "globalslam": {k: v for k, v in glob.items() if not k.endswith("_series")},
        "globalslam_series": {
            "z_err": glob.get("z_err_series"),
            "dist": glob.get("dist_series"),
            "z_est": glob.get("z_est_series"),
            "z_gt": glob.get("z_gt_series"),
        },
        "admit": {
            "n": len(admits),
            "modes": sorted({a["mode"] for a in admits}),
            "mean_abs_dz_init": float(np.mean(np.abs(dz_init))) if dz_init else None,
            "max_abs_dz_init": float(np.max(np.abs(dz_init))) if dz_init else None,
            "mean_abs_adj_z_jump": float(np.mean(np.abs(adj_jump))) if adj_jump else None,
            "max_abs_adj_z_jump": float(np.max(np.abs(adj_jump))) if adj_jump else None,
            "rows": admits,
        },
        "ba": {
            "n_ba_commit_events": len(ba["ba_commits"]),
            "n_dz_ba_pairs": len(dz_ba),
            "mean_abs_dz_ba": float(np.mean(np.abs(dz_ba))) if dz_ba else None,
            "max_abs_dz_ba": float(np.max(np.abs(dz_ba))) if dz_ba else None,
            "ba_update_rejected_lines": ba_reject,
            "delta_norm_large_lines": ba_delta_large,
            "dz_ba": ba["dz_ba"],
        },
        "landmarks": lm_stats,
        "gt_path_m": float(path_length_xy(gt[:, 1:3])[-1]) if len(gt) > 1 else 0.0,
    }


def plot_compare(ev: Path, r: dict, h: dict):
    def series(d, key):
        s = d.get("globalslam_series", {})
        return s.get(key) or []

    # 1 Z vs distance Local+Global
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, d, title in [(axes[0], r, "Run R"), (axes[1], h, "Run H")]:
        dist = series(d, "dist")
        zest = series(d, "z_est")
        zgt = series(d, "z_gt")
        if dist and zest:
            ax.plot(dist, zgt, "k-", lw=1.0, label="GT Z")
            ax.plot(dist, zest, "C2-", lw=1.0, label="Global Z")
        ax.set_title(title)
        ax.set_xlabel("Distance [m]")
        ax.set_ylabel("Z [m]")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.suptitle("KF-Z-A3 Global vs GT Z")
    fig.tight_layout()
    fig.savefig(ev / "z_vs_distance.svg")
    plt.close(fig)

    # 2 Z error vs distance
    fig, ax = plt.subplots(figsize=(8, 4))
    for d, lab, col in [(r, "R", "C3"), (h, "H", "C2")]:
        dist = series(d, "dist")
        ze = series(d, "z_err")
        if dist and ze:
            ax.plot(dist, ze, color=col, lw=1.0, label=lab)
    ax.set_xlabel("Distance [m]")
    ax.set_ylabel("Global Z error [m]")
    ax.set_title("KF-Z-A3 Global Z error vs distance")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(ev / "z_error_vs_distance.svg")
    plt.close(fig)

    # 3 Admit Z ladder
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, d, title in [(axes[0], r, "Run R"), (axes[1], h, "Run H")]:
        rows = d.get("admit", {}).get("rows", [])
        if rows:
            idx = [a["idx"] for a in rows]
            ax.plot(idx, [a["z_ekf"] for a in rows], "C0-", label="z_ekf", lw=0.8)
            ax.plot(idx, [a["z_full_rel"] for a in rows], "C3--", label="z_full_rel", lw=0.8)
            ax.plot(idx, [a["z_admit"] for a in rows], "C2-", label="z_admit", lw=1.0)
        ax.set_title(title)
        ax.set_xlabel("KF index")
        ax.set_ylabel("Z [m]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    fig.suptitle("KF-Z-A3 admit Z (ekf / full-rel / admitted)")
    fig.tight_layout()
    fig.savefig(ev / "admit_z_ladder.svg")
    plt.close(fig)

    # 4 BA dz hist
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.5), sharey=True)
    for ax, d, title in [(axes[0], r, "Run R"), (axes[1], h, "Run H")]:
        dz = [x["dz_ba"] for x in d.get("ba", {}).get("dz_ba", [])]
        if dz:
            ax.hist(dz, bins=40, color="C4", alpha=0.8)
        ax.set_title(f"{title} (n={len(dz)})")
        ax.set_xlabel("dz_BA [m]")
        ax.grid(True, alpha=0.3)
    fig.suptitle("KF-Z-A3 BA-induced ΔZ")
    fig.tight_layout()
    fig.savefig(ev / "ba_dz_hist.svg")
    plt.close(fig)

    # 5 XY trajectories — load from csv
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    for ax, name, title in [
        (axes[0], "run_R", "Run R"),
        (axes[1], "run_H", "Run H"),
    ]:
        rd = ev / "raw" / name
        gt = load_txyz(rd / "gt.csv")
        gkf = load_txyz(rd / "global_kf.csv")
        if len(gt):
            ax.plot(gt[:, 1], gt[:, 2], "k-", lw=1.0, label="GT")
        if len(gkf):
            ax.plot(gkf[:, 1], gkf[:, 2], "C2-", lw=0.9, label="Global")
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
    fig.suptitle("KF-Z-A3 Global XY vs GT")
    fig.tight_layout()
    fig.savefig(ev / "xy_trajectory.svg")
    plt.close(fig)

    # 6 landmark placeholder if missing
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.text(
        0.5,
        0.5,
        "Landmark height: see admit/BA-D5 + REPORT\n"
        "(PointCloud final dump optional)",
        ha="center",
        va="center",
    )
    ax.set_axis_off()
    fig.savefig(ev / "landmark_height_note.svg")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-dir", type=Path, required=True)
    args = ap.parse_args()
    ev = args.evidence_dir
    r = analyze_run(ev / "raw" / "run_R")
    h = analyze_run(ev / "raw" / "run_H")
    # Strip heavy rows from JSON
    def slim(d):
        out = json.loads(json.dumps(d))
        if "admit" in out and "rows" in out["admit"]:
            out["admit"]["n_rows_saved"] = len(out["admit"]["rows"])
            # keep first/last few
            rows = out["admit"]["rows"]
            out["admit"]["rows_head"] = rows[:5]
            out["admit"]["rows_tail"] = rows[-5:]
            del out["admit"]["rows"]
        if "ba" in out and "dz_ba" in out["ba"]:
            out["ba"]["dz_ba_n"] = len(out["ba"]["dz_ba"])
            out["ba"]["dz_ba_sample"] = out["ba"]["dz_ba"][:20]
            del out["ba"]["dz_ba"]
        if "globalslam_series" in out:
            del out["globalslam_series"]
        return out

    summary = {"run_R": slim(r), "run_H": slim(h)}
    (ev / "analysis_summary.json").write_text(json.dumps(summary, indent=2))
    # Save full admit CSVs
    for name, d in [("run_R", r), ("run_H", h)]:
        rows = d.get("admit", {}).get("rows", [])
        with (ev / "raw" / name / "admit_z.csv").open("w", newline="") as f:
            fields = [
                "mode",
                "idx",
                "id",
                "z_ekf",
                "z_full_rel",
                "z_admit",
                "z_gs_prev",
                "dz_init",
            ]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        with (ev / "raw" / name / "ba_dz.csv").open("w", newline="") as f:
            fields = ["idx_kf", "keyframe_id", "ba_id", "z_pre", "z_post", "dz_ba"]
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for row in d.get("ba", {}).get("dz_ba", []):
                w.writerow(row)

    plot_compare(ev, r, h)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
