#!/usr/bin/env python3
"""A3d offline analysis: ATE, first-loop KF/landmark deltas, latency, plots."""
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


def path_length_xy(xyz: np.ndarray) -> np.ndarray:
    if len(xyz) < 2:
        return np.zeros(len(xyz))
    d = np.linalg.norm(np.diff(xyz[:, 1:3], axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def drift_series(est: np.ndarray, gt: np.ndarray, max_dt: float = 0.15):
    e, g = associate(est, gt, max_dt)
    if len(e) == 0:
        return np.zeros(0), np.zeros(0)
    err = np.linalg.norm(e - g, axis=1)
    # approximate distance along associated est order using GT path of nearest samples
    # rebuild by walking est timestamps
    gt_t = gt[:, 0]
    dists = []
    cum = path_length_xy(gt)
    for row in est:
        j = np.searchsorted(gt_t, row[0])
        cands = [k for k in (j, j - 1) if 0 <= k < len(gt)]
        best = min(cands, key=lambda k: abs(gt_t[k] - row[0])) if cands else None
        if best is None or abs(gt_t[best] - row[0]) > max_dt:
            continue
        dists.append(cum[best])
    dists = np.asarray(dists[: len(err)], dtype=np.float64)
    return dists, err[: len(dists)]


def kf_deltas(pre: np.ndarray, post: np.ndarray) -> np.ndarray:
    n = min(len(pre), len(post))
    return np.linalg.norm(post[:n] - pre[:n], axis=1)


def landmark_height_stats(xyz: np.ndarray) -> dict:
    if len(xyz) == 0:
        return {"n": 0}
    z = xyz[:, 2]
    return {
        "n": int(len(z)),
        "z_min": float(z.min()),
        "z_p05": float(np.percentile(z, 5)),
        "z_p25": float(np.percentile(z, 25)),
        "z_median": float(np.median(z)),
        "z_p75": float(np.percentile(z, 75)),
        "z_p95": float(np.percentile(z, 95)),
        "z_max": float(z.max()),
        "n_z_above_p95_plus10": int(np.sum(z > (np.percentile(z, 95) + 10))),
        "n_z_below_p05_minus50": int(np.sum(z < (np.percentile(z, 5) - 50))),
    }


def match_landmarks_nn(pre: np.ndarray, post: np.ndarray, max_dist: float = 2.0):
    """Greedy NN match pre→post for displacement stats (no landmark IDs)."""
    if len(pre) == 0 or len(post) == 0:
        return np.zeros(0), np.zeros((0, 3)), np.zeros((0, 3))
    # subsample if huge for speed
    rng = np.random.default_rng(0)
    pre_s = pre if len(pre) <= 8000 else pre[rng.choice(len(pre), 8000, replace=False)]
    # KD-tree-free: chunked brute for robustness without scipy dependency preference
    try:
        from scipy.spatial import cKDTree

        tree = cKDTree(post)
        d, idx = tree.query(pre_s, k=1, workers=-1)
    except Exception:
        # fallback: random subsample post too
        post_s = post if len(post) <= 8000 else post[rng.choice(len(post), 8000, replace=False)]
        d = np.full(len(pre_s), np.inf)
        idx = np.zeros(len(pre_s), dtype=int)
        for i, p in enumerate(pre_s):
            dd = np.linalg.norm(post_s - p, axis=1)
            j = int(np.argmin(dd))
            d[i] = dd[j]
            idx[i] = j
        post = post_s
    ok = d <= max_dist
    return d[ok], pre_s[ok], post[idx[ok]]


def parse_latency(log_path: Path) -> dict:
    ss, ps, elig, above = [], [], [], []
    commits = []
    if not log_path.exists():
        return {}
    text = log_path.read_text(errors="replace")
    for m in re.finditer(r"search_s=([0-9.eE+-]+)", text):
        ss.append(float(m.group(1)))
    for m in re.finditer(r"pnp_s=([0-9.eE+-]+)", text):
        ps.append(float(m.group(1)))
    for m in re.finditer(
        r"visual LC attempt: cur_kf=(\d+) eligible=(\d+) above_thr=(\d+) travel_dis=([0-9.eE+-]+) search_s=([0-9.eE+-]+)",
        text,
    ):
        elig.append(int(m.group(2)))
        above.append(int(m.group(3)))
    for m in re.finditer(
        r"visual LC: matched_kf=(\d+) matches=(\d+) pnp_try=(\d+)/(\d+) pos=\[([^\]]+)\] ref=(\d+) committed=1 travel_dis=([0-9.eE+-]+)",
        text,
    ):
        commits.append(
            {
                "matched_kf": int(m.group(1)),
                "matches": int(m.group(2)),
                "pnp_try": int(m.group(3)),
                "ref": int(m.group(6)),
                "travel_dis": float(m.group(7)),
                "pos": [float(x) for x in m.group(5).split()],
            }
        )
    ba_delta = len(re.findall(r"Delta_norm for n-kf optimized too large", text))
    ba_reject = len(re.findall(r"BA update rejected \(Delta_norm gate\)", text))
    # Pair each successful Close-loop block with the immediately preceding Potential Loop
    pose_slam = []
    for m in re.finditer(
        r"Potential Loop: Mean reprojection error: ([0-9.eE+-]+) \(inliers=(\d+) d_xy=([0-9.eE+-]+)\)\s*"
        r".{0,400}?"
        r"initial error = ([0-9.eE+-]+)\s*.*?final error = ([0-9.eE+-]+)",
        text,
        re.S,
    ):
        if float(m.group(2)) <= 0:
            continue
        pose_slam.append(
            {
                "mean_reproj": float(m.group(1)),
                "inliers": int(m.group(2)),
                "d_xy": float(m.group(3)),
                "pose_slam_initial": float(m.group(4)),
                "pose_slam_final": float(m.group(5)),
            }
        )

    def summ(xs):
        if not xs:
            return None
        xs = sorted(xs)
        return {
            "n": len(xs),
            "median": float(statistics.median(xs)),
            "mean": float(statistics.fmean(xs)),
            "max": float(max(xs)),
            "p95": float(xs[max(0, int(0.95 * len(xs)) - 1)]),
        }

    return {
        "search_s": summ(ss),
        "pnp_s": summ(ps),
        "eligible_max": max(elig) if elig else None,
        "above_thr_max": max(above) if above else None,
        "ba_delta_norm_large_lines": ba_delta,
        "ba_update_rejected_lines": ba_reject,
        "matched_commits": commits,
        "pose_slam_events": pose_slam[:10],
    }


def global_path_vs_gt(path_xyz: np.ndarray, gt: np.ndarray) -> dict:
    """Nearest-GT translational metrics for untimed global path samples (mission frame).

    Global path is body/mission-relative without stamps. Align by early yaw+XY using
    first 20% of path vs GT early window, then nearest-GT in XY for remaining samples.
    """
    if len(path_xyz) == 0 or len(gt) < 2:
        return metrics(np.zeros((0, 3)), np.zeros((0, 3)))
    # Use GT XY as reference cloud; map each global KF to nearest GT by XY
    gxy = gt[:, 1:3]
    errs = []
    for p in path_xyz:
        d = np.linalg.norm(gxy - p[:2], axis=1)
        j = int(np.argmin(d))
        # Z: compare to GT z at that sample (both often ~0 in odom/GT epoch frame)
        e = np.array([p[0] - gt[j, 1], p[1] - gt[j, 2], p[2] - gt[j, 3]])
        errs.append(np.linalg.norm(e))
    errs = np.asarray(errs)
    # Also XY-only
    err_xy = []
    for p in path_xyz:
        d = np.linalg.norm(gxy - p[:2], axis=1)
        j = int(np.argmin(d))
        err_xy.append(np.linalg.norm(p[:2] - gt[j, 1:3]))
    err_xy = np.asarray(err_xy)
    return {
        "n": int(len(errs)),
        "ate_rmse": float(np.sqrt(np.mean(errs**2))),
        "ate_rmse_xy": float(np.sqrt(np.mean(err_xy**2))),
        "final_err": float(errs[-1]),
        "final_err_xy": float(err_xy[-1]),
        "max_err": float(errs.max()),
        "max_err_xy": float(err_xy.max()),
        "mean_err": float(errs.mean()),
        "note": "nearest-GT by XY; global path untimed — interpret with caution",
    }


def first_loop_kf_gt(pre: np.ndarray, post: np.ndarray, gt: np.ndarray) -> dict:
    """Per-KF XY nearest-GT error before/after first commit."""
    if len(pre) == 0:
        return {}
    gxy = gt[:, 1:3]

    def per_kf(path):
        e = []
        for p in path:
            j = int(np.argmin(np.linalg.norm(gxy - p[:2], axis=1)))
            e.append(np.linalg.norm(p[:2] - gt[j, 1:3]))
        return np.asarray(e)

    e_pre = per_kf(pre)
    e_post = per_kf(post[: len(pre)])
    return {
        "n_kf": int(len(e_pre)),
        "mean_err_xy_pre": float(e_pre.mean()),
        "mean_err_xy_post": float(e_post.mean()),
        "rmse_xy_pre": float(np.sqrt(np.mean(e_pre**2))),
        "rmse_xy_post": float(np.sqrt(np.mean(e_post**2))),
        "max_err_xy_pre": float(e_pre.max()),
        "max_err_xy_post": float(e_post.max()),
        "frac_improved": float(np.mean(e_post < e_pre)),
        "mean_delta_err": float((e_post - e_pre).mean()),
    }


def plot_all(out_dir: Path, c_dir: Path, t_dir: Path, analysis: dict):
    gt_c = load_xyz(c_dir / "gt.csv")
    od_c = load_xyz(c_dir / "odom.csv")
    gt_t = load_xyz(t_dir / "gt.csv")
    od_t = load_xyz(t_dir / "odom.csv")
    g_c = np.load(c_dir / "global_path_final.npy") if (c_dir / "global_path_final.npy").exists() else np.zeros((0, 3))
    g_t = np.load(t_dir / "global_path_final.npy") if (t_dir / "global_path_final.npy").exists() else np.zeros((0, 3))

    # 1 Local vs GT (treatment)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(gt_t[:, 1], gt_t[:, 2], "k-", lw=1.2, label="GT")
    ax.plot(od_t[:, 1], od_t[:, 2], "C0-", lw=0.8, alpha=0.8, label="LocalSLAM")
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("A3d Run T — LocalSLAM vs GT")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "localslam_vs_gt.svg")
    plt.close(fig)

    # 2 Global C vs T
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(gt_t[:, 1], gt_t[:, 2], "k-", lw=1.0, label="GT (T)")
    if len(g_c):
        ax.plot(g_c[:, 0], g_c[:, 1], "C3--", lw=1.0, label="Global C")
    if len(g_t):
        ax.plot(g_t[:, 0], g_t[:, 1], "C2-", lw=1.0, label="Global T")
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("A3d — GlobalSLAM control vs treatment")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "globalslam_C_vs_T.svg")
    plt.close(fig)

    # 3 Error vs distance
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for label, od, gt, color in [
        ("Local C", od_c, gt_c, "C3"),
        ("Local T", od_t, gt_t, "C0"),
    ]:
        # align
        e, g = associate(od, gt, 0.15)
        if len(e) < 10:
            continue
        e_al, _ = yaw_xy_align(e[: max(50, len(e) // 20)], g[: max(50, len(e) // 20)])
        # apply same R,t to all — recompute via early window
        n0 = max(50, len(e) // 20)
        _, al = yaw_xy_align(e[:n0], g[:n0])
        yaw, tx, ty = al["yaw"], al["tx"], al["ty"]
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        e2 = e.copy()
        e2[:, :2] = (R @ e[:, :2].T).T + np.array([tx, ty])
        err = np.linalg.norm(e2 - g, axis=1)
        dist = path_length_xy(np.column_stack([np.arange(len(g)), g[:, 0], g[:, 1], g[:, 2] if g.shape[1] > 2 else np.zeros(len(g))]))
        # better: use cumulative along g
        dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(g[:, :2], axis=0), axis=1))])
        # downsample
        step = max(1, len(dist) // 800)
        ax.plot(dist[::step], err[::step], color=color, lw=0.9, label=label)
    ax.set_xlabel("Traveled distance [m]")
    ax.set_ylabel("Position error [m]")
    ax.set_title("A3d — LocalSLAM error vs distance")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "error_vs_distance.svg")
    plt.close(fig)

    # 4 First-loop per-KF correction
    pre = analysis.get("commit0_kf_delta")
    if pre is not None and len(pre):
        fig, ax = plt.subplots(figsize=(8, 3.5))
        ax.bar(np.arange(len(pre)), pre, width=1.0, color="C2", alpha=0.85)
        ax.set_xlabel("Keyframe index (overlap)")
        ax.set_ylabel("|Δpose| [m]")
        ax.set_title("A3d Run T — first visual loop per-KF correction")
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / "first_loop_kf_correction.svg")
        plt.close(fig)

    # 5 Landmark height distributions
    h = analysis.get("landmark_heights")
    if h:
        fig, ax = plt.subplots(figsize=(7, 4))
        bins = np.linspace(-80, -30, 50)
        if "pre_z" in h:
            ax.hist(h["pre_z"], bins=bins, alpha=0.55, label="pre Pose_SLAM", color="C3")
        if "post_z" in h:
            ax.hist(h["post_z"], bins=bins, alpha=0.55, label="post Pose_SLAM", color="C2")
        ax.set_xlabel("Landmark Z (global map frame) [m]")
        ax.set_ylabel("Count")
        ax.set_title("A3d first loop — landmark height distribution")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / "landmark_height_dist.svg")
        plt.close(fig)

    # 6 BA timeline — commits only (no Delta_norm lines observed)
    fig, ax = plt.subplots(figsize=(8, 3))
    commits = analysis.get("latency", {}).get("matched_commits", [])
    if commits:
        xs = [c["travel_dis"] for c in commits]
        ys = [1] * len(commits)
        ax.stem(xs, ys, linefmt="C2-", markerfmt="C2o", basefmt="k-")
        ax.set_xlabel("Travel distance since previous gate [m] (log field)")
        ax.set_ylabel("Commit")
        ax.set_title("A3d Run T — visual commits (BA reject lines: 0 in launch log)")
    else:
        ax.text(0.5, 0.5, "No commits", ha="center", va="center")
        ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(out_dir / "ba_commit_timeline.svg")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--evidence-dir", type=Path, required=True)
    args = ap.parse_args()
    ev = args.evidence_dir
    raw = ev / "raw"
    c_dir = raw / "run_C"
    t_dir = raw / "run_T"
    out = ev
    out.mkdir(parents=True, exist_ok=True)

    # Load ate summaries already produced
    ate_c = json.loads((c_dir / "ate_summary.json").read_text()) if (c_dir / "ate_summary.json").exists() else {}
    ate_t = json.loads((t_dir / "ate_summary.json").read_text()) if (t_dir / "ate_summary.json").exists() else {}

    gt_t = load_xyz(t_dir / "gt.csv")
    pre = np.load(t_dir / "commit0_path_pre.npy") if (t_dir / "commit0_path_pre.npy").exists() else np.zeros((0, 3))
    post = np.load(t_dir / "commit0_path_post.npy") if (t_dir / "commit0_path_post.npy").exists() else np.zeros((0, 3))
    lm_pre = np.load(t_dir / "commit0_landmarks_pre.npy") if (t_dir / "commit0_landmarks_pre.npy").exists() else np.zeros((0, 3))
    lm_post = np.load(t_dir / "commit0_landmarks_post.npy") if (t_dir / "commit0_landmarks_post.npy").exists() else np.zeros((0, 3))
    pre1 = np.load(t_dir / "commit1_path_pre.npy") if (t_dir / "commit1_path_pre.npy").exists() else np.zeros((0, 3))
    post1 = np.load(t_dir / "commit1_path_post.npy") if (t_dir / "commit1_path_post.npy").exists() else np.zeros((0, 3))

    deltas = kf_deltas(pre, post) if len(pre) else np.zeros(0)
    deltas1 = kf_deltas(pre1, post1) if len(pre1) else np.zeros(0)

    # Landmark NN displacement (Pose_SLAM cloud motion)
    d_nn, pre_m, post_m = match_landmarks_nn(lm_pre, lm_post, max_dist=5.0)
    z_disp = post_m[:, 2] - pre_m[:, 2] if len(pre_m) else np.zeros(0)

    # Filter extreme underground for "elevated" analysis relative to median
    med_z = float(np.median(lm_pre[:, 2])) if len(lm_pre) else 0.0
    # Ground plane ≈ median; elevated = much less negative (higher z)
    elevated_pre = lm_pre[lm_pre[:, 2] > med_z + 15] if len(lm_pre) else np.zeros((0, 3))
    elevated_post = lm_post[lm_post[:, 2] > med_z + 15] if len(lm_post) else np.zeros((0, 3))
    deep_pre = lm_pre[lm_pre[:, 2] < med_z - 50] if len(lm_pre) else np.zeros((0, 3))

    latency = parse_latency(t_dir / "launch_mission.log")
    latency_c = parse_latency(c_dir / "launch_mission.log")

    first_loop_gt = first_loop_kf_gt(pre, post, gt_t)

    analysis = {
        "ate_C": ate_c,
        "ate_T": ate_t,
        "commit0_kf_delta": deltas.tolist(),
        "commit0_kf_delta_summary": {
            "n": int(len(deltas)),
            "mean": float(deltas.mean()) if len(deltas) else None,
            "rms": float(np.sqrt(np.mean(deltas**2))) if len(deltas) else None,
            "max": float(deltas.max()) if len(deltas) else None,
        },
        "commit1_kf_delta_summary": {
            "n": int(len(deltas1)),
            "mean": float(deltas1.mean()) if len(deltas1) else None,
            "rms": float(np.sqrt(np.mean(deltas1**2))) if len(deltas1) else None,
            "max": float(deltas1.max()) if len(deltas1) else None,
        },
        "first_loop_kf_vs_gt_xy": first_loop_gt,
        "landmark_heights": {
            "pre": landmark_height_stats(lm_pre),
            "post": landmark_height_stats(lm_post),
            "median_z_pre": med_z,
            "n_elevated_pre_med_plus15": int(len(elevated_pre)),
            "n_elevated_post_med_plus15": int(len(elevated_post)),
            "n_deep_pre_med_minus50": int(len(deep_pre)),
            "nn_match": {
                "n_matched": int(len(d_nn)),
                "mean_disp_m": float(d_nn.mean()) if len(d_nn) else None,
                "rms_disp_m": float(np.sqrt(np.mean(d_nn**2))) if len(d_nn) else None,
                "max_disp_m": float(d_nn.max()) if len(d_nn) else None,
                "mean_abs_z_disp": float(np.mean(np.abs(z_disp))) if len(z_disp) else None,
                "max_abs_z_disp": float(np.max(np.abs(z_disp))) if len(z_disp) else None,
            },
            "pre_z": lm_pre[:, 2].tolist() if len(lm_pre) else [],
            "post_z": lm_post[:, 2].tolist() if len(lm_post) else [],
        },
        "latency": latency,
        "latency_C": latency_c,
        "relative_kf_init": {
            "status": "UNAVAILABLE",
            "note": "No per-KF Local absolute / relative / Global init diagnostic topic captured; relative_kf_pose=1 confirmed in Gslam ready line only.",
        },
        "intermediate_map_states": {
            "pre_pose_slam": "captured (commit0_*_pre.npy)",
            "immediately_corrected": "captured as commit0_*_post (includes match_ba_refine if ran before publish)",
            "post_loop_BA_separate": "NOT SEPARATED — match_ba_refine_ran=1; post snapshot is after Pose_SLAM+local BA refine",
            "subsequent_incremental_BA": "NOT ISOLATED — no Delta_norm reject lines in A3d launch logs",
        },
    }

    # Strip large z arrays from JSON dump
    analysis_json = json.loads(json.dumps(analysis, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else o))
    analysis_json["landmark_heights"].pop("pre_z", None)
    analysis_json["landmark_heights"].pop("post_z", None)
    analysis_json.pop("commit0_kf_delta", None)
    (out / "analysis_summary.json").write_text(json.dumps(analysis_json, indent=2))

    plot_all(out, c_dir, t_dir, analysis)
    print(json.dumps(analysis_json, indent=2))


if __name__ == "__main__":
    main()
