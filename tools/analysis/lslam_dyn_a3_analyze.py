#!/usr/bin/env python3
"""LSLAM-DYNAMIC-CONSISTENCY-A3 offline analysis.

Consumes per-run raw/ folders under docs/evidence/lslam_dynamic_consistency_a3/
and writes tables/, figures/, and summary JSON for REPORT.md.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None


RUNS = [
    ("A3a", "run_a3a_jac_off_sigma001"),
    ("A3b", "run_a3b_jac_on_sigma001"),
    ("A3c", "run_a3c_jac_on_sigma050"),
]


def load_xyz(csv_path: Path) -> np.ndarray:
    if not csv_path.exists():
        return np.zeros((0, 4))
    rows = []
    with csv_path.open() as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                rows.append(
                    [float(row["t"]), float(row["x"]), float(row["y"]), float(row["z"])]
                )
            except (KeyError, ValueError):
                continue
    return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 4))


def load_att(csv_path: Path) -> np.ndarray:
    """t, roll, pitch, yaw if present."""
    if not csv_path.exists():
        return np.zeros((0, 4))
    rows = []
    with csv_path.open() as f:
        r = csv.DictReader(f)
        fields = r.fieldnames or []
        roll_k = next((k for k in ("roll", "phi", "r") if k in fields), None)
        pitch_k = next((k for k in ("pitch", "theta", "p") if k in fields), None)
        yaw_k = next((k for k in ("yaw", "psi", "y") if k in fields), None)
        if not (roll_k and pitch_k and yaw_k and "t" in fields):
            return np.zeros((0, 4))
        for row in r:
            try:
                rows.append(
                    [
                        float(row["t"]),
                        float(row[roll_k]),
                        float(row[pitch_k]),
                        float(row[yaw_k]),
                    ]
                )
            except ValueError:
                continue
    return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 4))


def associate(est: np.ndarray, gt: np.ndarray, max_dt: float = 0.05):
    if len(est) == 0 or len(gt) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros((0,))
    gt_t = gt[:, 0]
    e_xyz, g_xyz, ts = [], [], []
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
            ts.append(row[0])
    return np.asarray(e_xyz), np.asarray(g_xyz), np.asarray(ts)


def yaw_xy_align(est: np.ndarray, gt: np.ndarray):
    if len(est) < 2:
        return est.copy(), {"yaw": 0.0, "tx": 0.0, "ty": 0.0}
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
    return out, {"yaw": yaw, "tx": float(t[0]), "ty": float(t[1])}


def rmse(a: np.ndarray) -> float:
    if len(a) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.sum(a * a, axis=1))))


def load_diag(path: Path) -> np.ndarray:
    if not path.exists() or path.stat().st_size == 0:
        return np.zeros((0,))
    # return structured via csv DictReader later
    return path


def parse_diag(path: Path):
    rows = []
    if not path.exists():
        return rows
    with path.open() as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows


def stage_correction_stats(diag_rows):
    """Aggregate |delta_pos| and |delta_att| for after-phase by stage."""
    out = {}
    for stage in ("PRED", "AHRS", "VIS", "ALT", "RANGE"):
        dxy = []
        datt = []
        for row in diag_rows:
            if row.get("stage") != stage or row.get("phase") != "after":
                continue
            try:
                dx = float(row["dx"])
                dy = float(row["dy"])
                dz = float(row["dz"])
                dphi = float(row["dphi"])
                dtheta = float(row["dtheta"])
                dpsi = float(row["dpsi"])
            except (KeyError, ValueError):
                continue
            dxy.append(math.hypot(dx, dy))
            datt.append(math.sqrt(dphi * dphi + dtheta * dtheta + dpsi * dpsi))
        out[stage] = {
            "n": len(dxy),
            "horiz_corr_mean": float(np.mean(dxy)) if dxy else float("nan"),
            "horiz_corr_p95": float(np.percentile(dxy, 95)) if dxy else float("nan"),
            "horiz_corr_max": float(np.max(dxy)) if dxy else float("nan"),
            "att_corr_mean": float(np.mean(datt)) if datt else float("nan"),
            "att_corr_p95": float(np.percentile(datt, 95)) if datt else float("nan"),
            "att_corr_max": float(np.max(datt)) if datt else float("nan"),
        }
    return out


def detect_braking_windows(gt: np.ndarray, v_thresh: float = 3.5, win_s: float = 8.0):
    """Identify deceleration near corners from GT speed."""
    if len(gt) < 10:
        return []
    t = gt[:, 0]
    xy = gt[:, 1:3]
    dt = np.diff(t)
    dt[dt <= 0] = np.nan
    v = np.zeros(len(gt))
    dxy = np.diff(xy, axis=0)
    speed = np.linalg.norm(dxy, axis=1) / dt
    v[1:] = np.nan_to_num(speed, nan=0.0)
    # smooth
    if len(v) > 5:
        kernel = np.ones(5) / 5.0
        v_s = np.convolve(v, kernel, mode="same")
    else:
        v_s = v
    # find local minima after high speed
    windows = []
    i = 20
    while i < len(v_s) - 20:
        if v_s[i] < v_thresh and np.max(v_s[max(0, i - 40) : i]) > 4.0:
            t0 = t[i] - win_s * 0.4
            t1 = t[i] + win_s * 0.6
            # avoid duplicates
            if not windows or t0 > windows[-1][1] + 5.0:
                windows.append((float(t0), float(t1), float(t[i])))
                i += 40
                continue
        i += 1
    return windows


def window_metrics(est, gt, ts, t0, t1):
    mask = (ts >= t0) & (ts <= t1)
    if not np.any(mask):
        return {}
    err = est[mask] - gt[mask]
    horiz = np.linalg.norm(err[:, :2], axis=1)
    # along-track: project onto GT velocity direction
    # approximate using consecutive GT
    return {
        "peak_horiz_err": float(np.max(horiz)),
        "mean_horiz_err": float(np.mean(horiz)),
        "osc_amp": float(np.percentile(horiz, 95) - np.percentile(horiz, 5))
        if len(horiz) > 5
        else float("nan"),
        "n": int(len(horiz)),
    }


def analyze_run(raw: Path) -> dict:
    meta = {}
    meta_path = raw / "run_meta.txt"
    if meta_path.exists():
        for line in meta_path.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                meta[k.strip()] = v.strip()

    gt = load_xyz(raw / "gt.csv")
    if len(gt) == 0:
        gt = load_xyz(raw / "gt_pose.csv")
    odom = load_xyz(raw / "odom.csv")
    if len(odom) == 0:
        odom = load_xyz(raw / "odometry.csv")
    gslam = load_xyz(raw / "global_kf.csv")
    if len(gslam) == 0:
        gslam = load_xyz(raw / "globalslam_pose.csv")

    # prefer ate summary if present
    ate = {}
    ate_path = raw / "ate_summary.json"
    if ate_path.exists():
        try:
            ate = json.loads(ate_path.read_text())
        except Exception:
            ate = {}

    e, g, ts = associate(odom, gt, max_dt=0.08)
    align = {}
    if len(e) > 10:
        e_al, align = yaw_xy_align(e, g)
        err = e_al - g
        local_ate = {
            "n": len(e),
            "ate_rmse": rmse(err),
            "horiz_rmse": float(np.sqrt(np.mean(np.sum(err[:, :2] ** 2, axis=1)))),
            "z_rmse": float(np.sqrt(np.mean(err[:, 2] ** 2))),
            "peak_horiz": float(np.max(np.linalg.norm(err[:, :2], axis=1))),
        }
    else:
        e_al, err = e, np.zeros((0, 3))
        local_ate = {
            "n": 0,
            "ate_rmse": float("nan"),
            "horiz_rmse": float("nan"),
            "z_rmse": float("nan"),
            "peak_horiz": float("nan"),
        }

    ge, gg, gts = associate(gslam, gt, max_dt=0.08)
    if len(ge) > 10:
        ge_al, _ = yaw_xy_align(ge, gg)
        gerr = ge_al - gg
        global_ate = {
            "n": len(ge),
            "ate_rmse": rmse(gerr),
            "horiz_rmse": float(np.sqrt(np.mean(np.sum(gerr[:, :2] ** 2, axis=1)))),
        }
    else:
        global_ate = {"n": 0, "ate_rmse": float("nan"), "horiz_rmse": float("nan")}

    diag_rows = parse_diag(raw / "lslam_dynamic_diag.csv")
    stage = stage_correction_stats(diag_rows)

    brakes = detect_braking_windows(gt) if len(gt) else []
    brake_metrics = []
    for t0, t1, tc in brakes:
        m = window_metrics(e_al, g, ts, t0, t1) if len(e_al) else {}
        # stage corrections inside window using mono_ms is hard; use meas_time if ms
        m["t0"] = t0
        m["t1"] = t1
        m["t_center"] = tc
        brake_metrics.append(m)

    # pred_dt stats
    pred_dts = []
    for row in diag_rows:
        if row.get("stage") == "PRED" and row.get("phase") == "before":
            try:
                pred_dts.append(float(row["pred_dt"]))
            except (KeyError, ValueError):
                pass

    return {
        "run_id": raw.name,
        "meta": meta,
        "mission_done": meta.get("mission_done_marker") == "1",
        "local_ate": local_ate,
        "global_ate": global_ate,
        "align": align,
        "stage_corrections": stage,
        "braking_windows": brake_metrics,
        "pred_dt": {
            "n": len(pred_dts),
            "median": float(np.median(pred_dts)) if pred_dts else float("nan"),
            "p95": float(np.percentile(pred_dts, 95)) if pred_dts else float("nan"),
        },
        "diag_rows": len(diag_rows),
        "ate_file": ate,
        "traj": {
            "gt_n": len(gt),
            "odom_n": len(odom),
            "gslam_n": len(gslam),
            "gt": gt.tolist() if len(gt) < 50000 else [],
            "odom": odom.tolist() if len(odom) < 50000 else [],
        },
        "_arrays": {
            "gt": gt,
            "e_al": e_al,
            "g": g,
            "ts": ts,
            "brakes": brakes,
            "err": err,
        },
    }


def plot_trajs(results, fig_dir: Path):
    if plt is None:
        return
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    colors = {"A3a": "C0", "A3b": "C1", "A3c": "C2"}
    gt_plotted = False
    for label, r in results.items():
        arr = r.get("_arrays", {})
        gt = arr.get("gt")
        e = arr.get("e_al")
        if gt is not None and len(gt) and not gt_plotted:
            ax.plot(gt[:, 1], gt[:, 2], "k-", lw=1.5, label="GT", alpha=0.7)
            gt_plotted = True
        if e is not None and len(e):
            ax.plot(e[:, 0], e[:, 1], color=colors.get(label, None), lw=1.0, label=f"{label} Local")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("LocalSLAM vs GT (yaw+XY aligned)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "traj_local_vs_gt.png", dpi=140)
    plt.close(fig)


def plot_braking(results, fig_dir: Path):
    if plt is None:
        return
    # pick first braking window from first run that has one
    ref = None
    for label, r in results.items():
        brakes = r.get("_arrays", {}).get("brakes") or []
        if brakes:
            ref = brakes[0]
            break
    if ref is None:
        return
    t0, t1, _ = ref
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    colors = {"A3a": "C0", "A3b": "C1", "A3c": "C2"}
    for label, r in results.items():
        arr = r["_arrays"]
        ts = arr.get("ts")
        err = arr.get("err")
        gt = arr.get("gt")
        if ts is None or err is None or len(ts) == 0:
            continue
        mask = (ts >= t0) & (ts <= t1)
        if not np.any(mask):
            continue
        horiz = np.linalg.norm(err[mask, :2], axis=1)
        axes[0].plot(ts[mask] - t0, horiz, color=colors[label], label=label)
        if gt is not None and len(gt):
            gmask = (gt[:, 0] >= t0) & (gt[:, 0] <= t1)
            if np.any(gmask) and label == "A3a":
                dxy = np.diff(gt[gmask, 1:3], axis=0)
                dt = np.diff(gt[gmask, 0])
                sp = np.linalg.norm(dxy, axis=1) / np.maximum(dt, 1e-6)
                axes[1].plot(gt[gmask, 0][1:] - t0, sp, "k-", label="GT speed")
        # stage VIS/PRED/AHRS horiz corrections near window (meas_time_ms if comparable)
    axes[0].set_ylabel("horiz err [m]")
    axes[0].legend()
    axes[0].set_title(f"Braking window [{t0:.1f},{t1:.1f}]")
    axes[1].set_ylabel("speed [m/s]")
    axes[1].legend()
    # stage corrections from A3a diag if available — plotted in separate figure
    axes[2].set_ylabel("(see stage fig)")
    axes[2].set_xlabel("t - t0 [s]")
    fig.tight_layout()
    fig.savefig(fig_dir / "braking_horiz_err.png", dpi=140)
    plt.close(fig)


def plot_stage_series(raw: Path, fig_dir: Path, tag: str):
    if plt is None:
        return
    rows = parse_diag(raw / "lslam_dynamic_diag.csv")
    if not rows:
        return
    series = {s: {"t": [], "dxy": [], "datt": []} for s in ("PRED", "AHRS", "VIS")}
    t0 = None
    for row in rows:
        st = row.get("stage")
        if st not in series or row.get("phase") != "after":
            continue
        try:
            t = float(row["mono_ms"]) * 1e-3
            dxy = math.hypot(float(row["dx"]), float(row["dy"]))
            datt = math.sqrt(
                float(row["dphi"]) ** 2 + float(row["dtheta"]) ** 2 + float(row["dpsi"]) ** 2
            )
        except (KeyError, ValueError):
            continue
        if t0 is None:
            t0 = t
        series[st]["t"].append(t - t0)
        series[st]["dxy"].append(dxy)
        series[st]["datt"].append(datt)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for st, col in (("PRED", "C0"), ("AHRS", "C1"), ("VIS", "C2")):
        if series[st]["t"]:
            axes[0].plot(series[st]["t"], series[st]["dxy"], ".", ms=1, color=col, label=st, alpha=0.5)
            axes[1].plot(series[st]["t"], series[st]["datt"], ".", ms=1, color=col, label=st, alpha=0.5)
    axes[0].set_ylabel("|Δxy| [m]")
    axes[1].set_ylabel("|Δatt| [rad]")
    axes[1].set_xlabel("t - t0 [s] (steady clock)")
    axes[0].legend(markerscale=4)
    axes[0].set_title(f"{tag} stage-tagged corrections")
    fig.tight_layout()
    fig.savefig(fig_dir / f"stage_corrections_{tag}.png", dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--evidence-dir",
        type=Path,
        default=Path("docs/evidence/lslam_dynamic_consistency_a3"),
    )
    args = ap.parse_args()
    evid = args.evidence_dir
    raw_root = evid / "raw"
    fig_dir = evid / "figures"
    tab_dir = evid / "tables"
    fig_dir.mkdir(parents=True, exist_ok=True)
    tab_dir.mkdir(parents=True, exist_ok=True)

    results = {}
    summaries = {}
    for label, run_id in RUNS:
        raw = raw_root / run_id
        if not raw.exists():
            summaries[label] = {"missing": True, "run_id": run_id}
            continue
        r = analyze_run(raw)
        results[label] = r
        # strip heavy arrays for JSON
        s = {k: v for k, v in r.items() if not k.startswith("_") and k != "traj"}
        s["traj_counts"] = {
            "gt_n": r["traj"]["gt_n"],
            "odom_n": r["traj"]["odom_n"],
            "gslam_n": r["traj"]["gslam_n"],
        }
        summaries[label] = s
        plot_stage_series(raw, fig_dir, label)

    if results:
        plot_trajs(results, fig_dir)
        plot_braking(results, fig_dir)

    out_json = tab_dir / "summary.json"
    out_json.write_text(json.dumps(summaries, indent=2, default=str))

    # comparison CSV
    with (tab_dir / "mission_level.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "cfg",
                "mission_done",
                "local_ate_rmse",
                "local_horiz_rmse",
                "local_peak_horiz",
                "global_ate_rmse",
                "diag_rows",
                "vis_horiz_corr_p95",
                "ahrs_horiz_corr_p95",
                "pred_horiz_corr_p95",
                "vis_att_corr_p95",
            ]
        )
        for label, _ in RUNS:
            s = summaries.get(label, {})
            if s.get("missing"):
                w.writerow([label, "MISSING"] + [""] * 9)
                continue
            sc = s.get("stage_corrections", {})
            w.writerow(
                [
                    label,
                    s.get("mission_done"),
                    s.get("local_ate", {}).get("ate_rmse"),
                    s.get("local_ate", {}).get("horiz_rmse"),
                    s.get("local_ate", {}).get("peak_horiz"),
                    s.get("global_ate", {}).get("ate_rmse"),
                    s.get("diag_rows"),
                    sc.get("VIS", {}).get("horiz_corr_p95"),
                    sc.get("AHRS", {}).get("horiz_corr_p95"),
                    sc.get("PRED", {}).get("horiz_corr_p95"),
                    sc.get("VIS", {}).get("att_corr_p95"),
                ]
            )
    print(f"wrote {out_json}")
    print(json.dumps({k: {kk: vv for kk, vv in v.items() if kk != "meta"} for k, v in summaries.items()}, indent=2, default=str)[:4000])


if __name__ == "__main__":
    main()
