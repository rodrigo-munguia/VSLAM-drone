#!/usr/bin/env python3
"""LSLAM-DYNAMIC-CONSISTENCY-A3.1 offline analysis for rect380 A3b run."""
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
except Exception:
    plt = None


def load_csv(path: Path, cols=None) -> np.ndarray:
    if not path.exists() or path.stat().st_size == 0:
        return np.zeros((0, 1))
    rows = []
    with path.open() as f:
        r = csv.DictReader(f)
        fields = cols or r.fieldnames or []
        for row in r:
            try:
                rows.append([float(row[c]) for c in fields])
            except (KeyError, ValueError):
                continue
    return np.asarray(rows, float) if rows else np.zeros((0, len(fields)))


def assoc(est, gt, max_dt=0.08):
    if len(est) == 0 or len(gt) == 0:
        return np.zeros((0, est.shape[1] - 1 if est.ndim == 2 else 0)), np.zeros((0, 3)), np.zeros(0)
    gt_t = gt[:, 0]
    E, G, T = [], [], []
    for row in est:
        j = np.searchsorted(gt_t, row[0])
        cands = [k for k in (j - 1, j) if 0 <= k < len(gt)]
        best, bd = None, max_dt
        for k in cands:
            dt = abs(gt_t[k] - row[0])
            if dt <= bd:
                bd, best = dt, k
        if best is not None:
            E.append(row[1:])
            G.append(gt[best, 1:4])
            T.append(row[0])
    return np.asarray(E), np.asarray(G), np.asarray(T)


def yaw_xy_align(est_xyz, gt_xyz):
    e, g = est_xyz[:, :2], gt_xyz[:, :2]
    if len(e) < 2:
        return est_xyz.copy()
    mu_e, mu_g = e.mean(0), g.mean(0)
    H = (e - mu_e).T @ (g - mu_g)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_g - R @ mu_e
    out = est_xyz.copy()
    out[:, :2] = (R @ e.T).T + t
    return out


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def gt_speed(gt):
    t, xy = gt[:, 0], gt[:, 1:3]
    dt = np.diff(t)
    dt[dt <= 0] = np.nan
    sp = np.linalg.norm(np.diff(xy, axis=0), axis=1) / dt
    v = np.zeros(len(gt))
    v[1:] = np.nan_to_num(sp, nan=0.0)
    return np.convolve(v, np.ones(9) / 9.0, mode="same")


def find_brakes(gt, v_hi=4.5, v_lo=2.0, min_sep=15.0):
    v, t = gt_speed(gt), gt[:, 0]
    wins, i = [], 100
    while i < len(v) - 100:
        if v[i] < v_lo and np.max(v[i - 120 : i]) > v_hi:
            j = i + int(np.argmin(v[i : i + 80]))
            tc = t[j]
            if not wins or tc - wins[-1][2] > min_sep:
                wins.append(
                    (
                        tc - 10.0,
                        tc + 12.0,
                        tc,
                        float(np.max(v[i - 120 : i])),
                        float(v[j]),
                    )
                )
            i = j + 80
            continue
        i += 1
    if len(wins) < 2:
        # fallback lower thresholds
        return find_brakes(gt, v_hi=3.8, v_lo=1.8, min_sep=12.0) if v_hi > 3.9 else wins
    return wins


def nearest_row(arr, t, max_dt=0.05):
    if len(arr) == 0:
        return None
    j = np.searchsorted(arr[:, 0], t)
    cands = [k for k in (j - 1, j) if 0 <= k < len(arr)]
    best, bd = None, max_dt
    for k in cands:
        dt = abs(arr[k, 0] - t)
        if dt <= bd:
            bd, best = dt, k
    return None if best is None else arr[best]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--evidence-dir",
        type=Path,
        default=Path("docs/evidence/lslam_dynamic_consistency_a3_1"),
    )
    ap.add_argument(
        "--run-id",
        default="run_a3_1_jac_on_sigma001_rect380",
    )
    ap.add_argument(
        "--a3b-ref",
        type=Path,
        default=Path(
            "docs/evidence/lslam_dynamic_consistency_a3/raw/run_a3b_jac_on_sigma001"
        ),
    )
    args = ap.parse_args()
    evid = args.evidence_dir
    raw = evid / "raw" / args.run_id
    fig = evid / "figures"
    tab = evid / "tables"
    fig.mkdir(parents=True, exist_ok=True)
    tab.mkdir(parents=True, exist_ok=True)

    gt = load_csv(raw / "gt.csv", ["t", "x", "y", "z"])
    odom = load_csv(raw / "odom.csv", ["t", "x", "y", "z"])
    odom_full = load_csv(
        raw / "odom_full.csv",
        ["t", "x", "y", "z", "roll_i", "pitch_i", "yaw_i", "vx", "vy", "vz", "wx", "wy", "wz"],
    )
    gt_att = load_csv(
        raw / "gt_att.csv",
        ["t", "roll_i", "pitch_i", "yaw_i", "roll_ros", "pitch_ros", "yaw_ros", "wx", "wy", "wz"],
    )
    ahrs = load_csv(raw / "ahrs.csv", ["t", "roll", "pitch", "yaw", "time_ms"])
    gkf = load_csv(raw / "global_kf.csv", ["t", "x", "y", "z"])

    ate_path = raw / "ate_summary.json"
    ate = json.loads(ate_path.read_text()) if ate_path.exists() else {}

    v = gt_speed(gt) if len(gt) else np.zeros(0)
    brakes = find_brakes(gt) if len(gt) else []
    print(
        f"GT n={len(gt)} vmax={v.max() if len(v) else None:.3f} "
        f"v95={np.percentile(v,95) if len(v) else None:.3f} brakes={len(brakes)}"
    )

    e, g, ts = assoc(odom, gt, max_dt=0.1)
    e_al = yaw_xy_align(e, g) if len(e) else e
    err = e_al - g if len(e) else np.zeros((0, 3))
    horiz = np.linalg.norm(err[:, :2], axis=1) if len(err) else np.zeros(0)

    # Attitude errors (internal convention)
    att_err = []
    if len(odom_full) and len(gt_att):
        for row in odom_full[::2]:
            gta = nearest_row(gt_att, row[0], max_dt=0.05)
            if gta is None:
                continue
            dphi = wrap(row[4] - gta[1])
            dth = wrap(row[5] - gta[2])
            dpsi = wrap(row[6] - gta[3])
            att_err.append([row[0], dphi, dth, dpsi, math.sqrt(dphi * dphi + dth * dth + dpsi * dpsi)])
    att_err = np.asarray(att_err) if att_err else np.zeros((0, 5))

    # Image-time attitude: VIS before rows vs GT/AHRS
    diag_path = raw / "lslam_dynamic_diag.csv"
    img_att = []
    last_ahrs_meas = None
    pred_err_inc = []
    stage_help = {s: {"pos_help": 0, "pos_hurt": 0, "att_help": 0, "att_hurt": 0, "n": 0} for s in ("PRED", "AHRS", "VIS")}
    if diag_path.exists():
        prev_pred_before = None
        with diag_path.open() as f:
            for r in csv.DictReader(f):
                st, ph = r["stage"], r["phase"]
                mt = float(r["meas_time_ms"]) * 1e-3
                if st == "AHRS" and ph == "before":
                    last_ahrs_meas = {
                        "t": mt,
                        "phi": float(r["phi"]),  # state before = not measurement; see note
                    }
                if st == "VIS" and ph == "before":
                    gta = nearest_row(gt_att, mt, max_dt=0.08) if len(gt_att) else None
                    ahr = nearest_row(ahrs, mt, max_dt=0.08) if len(ahrs) else None
                    # estimated attitude used for projection ≈ state before VIS
                    ephi, eth, epsi = float(r["phi"]), float(r["theta"]), float(r["psi"])
                    rec = {
                        "t": mt,
                        "est_roll": ephi,
                        "est_pitch": eth,
                        "est_yaw": epsi,
                        "ahrs_age_ms": None,
                        "gt_err": None,
                        "ahrs_err": None,
                        "ahrs_vs_gt": None,
                    }
                    if last_ahrs_meas is not None:
                        rec["ahrs_age_ms"] = (mt - last_ahrs_meas["t"]) * 1000.0
                    if gta is not None:
                        dphi = wrap(ephi - gta[1])
                        dth = wrap(eth - gta[2])
                        dpsi = wrap(epsi - gta[3])
                        rec["gt_err"] = math.sqrt(dphi * dphi + dth * dth + dpsi * dpsi)
                    if ahr is not None and gta is not None:
                        # AHRS msg already internal convention
                        dphi = wrap(ahr[1] - gta[1])
                        dth = wrap(ahr[2] - gta[2])
                        dpsi = wrap(ahr[3] - gta[3])
                        rec["ahrs_vs_gt"] = math.sqrt(dphi * dphi + dth * dth + dpsi * dpsi)
                        dphi = wrap(ephi - ahr[1])
                        dth = wrap(eth - ahr[2])
                        dpsi = wrap(epsi - ahr[3])
                        rec["ahrs_err"] = math.sqrt(dphi * dphi + dth * dth + dpsi * dpsi)
                    img_att.append(rec)

                # Prediction error increment vs GT displacement
                if st == "PRED" and ph == "before":
                    prev_pred_before = r
                if st == "PRED" and ph == "after" and prev_pred_before is not None:
                    t0 = float(prev_pred_before["meas_time_ms"]) * 1e-3
                    # use mono-adjacent; better: associate positions with GT at before/after via state time ≈ last meas
                    # Use position before/after and GT nearest to same stamps if we treat meas_time as proxy
                    xb, yb = float(prev_pred_before["x"]), float(prev_pred_before["y"])
                    xa, ya = float(r["x"]), float(r["y"])
                    pred_d = np.array([xa - xb, ya - yb])
                    # GT displacement over pred_dt
                    dt = float(prev_pred_before["pred_dt"])
                    g0 = nearest_row(gt, t0, max_dt=0.1)
                    g1 = nearest_row(gt, t0 + dt, max_dt=0.1) if dt > 0 else None
                    if g0 is not None and g1 is not None and dt > 1e-4:
                        gt_d = np.array([g1[1] - g0[1], g1[2] - g0[2]])
                        inc = pred_d - gt_d
                        # along-track from GT motion
                        nrm = np.linalg.norm(gt_d) + 1e-9
                        tang = gt_d / nrm
                        along = float(inc @ tang)
                        cross = float(inc[0] * tang[1] - inc[1] * tang[0])
                        pred_err_inc.append(
                            {
                                "t": t0,
                                "dt": dt,
                                "along": along,
                                "cross": cross,
                                "norm": float(np.linalg.norm(inc)),
                                "pred_norm": float(np.linalg.norm(pred_d)),
                                "gt_norm": float(np.linalg.norm(gt_d)),
                            }
                        )
                    prev_pred_before = None

                # Does update help position/attitude error?
                if ph == "after" and st in stage_help:
                    # before state reconstructed from after - delta
                    xa, ya = float(r["x"]), float(r["y"])
                    xb, yb = xa - float(r["dx"]), ya - float(r["dy"])
                    t_m = float(r["meas_time_ms"]) * 1e-3
                    gta = nearest_row(gt, t_m, max_dt=0.1)
                    if gta is not None:
                        eb = math.hypot(xb - gta[1], yb - gta[2])
                        ea = math.hypot(xa - gta[1], ya - gta[2])
                        stage_help[st]["n"] += 1
                        if ea < eb - 1e-6:
                            stage_help[st]["pos_help"] += 1
                        elif ea > eb + 1e-6:
                            stage_help[st]["pos_hurt"] += 1
                    if len(gt_att):
                        gatt = nearest_row(gt_att, t_m, max_dt=0.08)
                        if gatt is not None:
                            phi_a, th_a, ps_a = float(r["phi"]), float(r["theta"]), float(r["psi"])
                            phi_b = wrap(phi_a - float(r["dphi"]))
                            th_b = wrap(th_a - float(r["dtheta"]))
                            ps_b = wrap(ps_a - float(r["dpsi"]))
                            eb = math.sqrt(
                                wrap(phi_b - gatt[1]) ** 2
                                + wrap(th_b - gatt[2]) ** 2
                                + wrap(ps_b - gatt[3]) ** 2
                            )
                            ea = math.sqrt(
                                wrap(phi_a - gatt[1]) ** 2
                                + wrap(th_a - gatt[2]) ** 2
                                + wrap(ps_a - gatt[3]) ** 2
                            )
                            if ea < eb - 1e-6:
                                stage_help[st]["att_help"] += 1
                            elif ea > eb + 1e-6:
                                stage_help[st]["att_hurt"] += 1

    # Braking event metrics
    brake_metrics = []
    for win in brakes:
        t0, t1, tc, v_b, v_m = win
        m = (ts >= t0) & (ts <= t1) if len(ts) else np.zeros(0, dtype=bool)
        if m.sum() < 5:
            continue
        h = horiz[m]
        gi = np.searchsorted(gt[:, 0], tc)
        gi = min(max(gi, 1), len(gt) - 2)
        tang = gt[gi, 1:3] - gt[gi - 1, 1:3]
        tang = tang / (np.linalg.norm(tang) + 1e-9)
        along = (err[m, :2] @ tang)
        # attitude err in window
        if len(att_err):
            am = (att_err[:, 0] >= t0) & (att_err[:, 0] <= t1)
            att_peak = float(att_err[am, 4].max()) if am.any() else None
            att_mean = float(att_err[am, 4].mean()) if am.any() else None
        else:
            att_peak = att_mean = None
        # image-time att in window
        img_w = [x for x in img_att if t0 <= x["t"] <= t1 and x["gt_err"] is not None]
        img_peak = max(x["gt_err"] for x in img_w) if img_w else None
        # pred err in window
        pe = [p for p in pred_err_inc if t0 <= p["t"] <= t1]
        brake_metrics.append(
            {
                "t_center_rel": float(tc - gt[0, 0]),
                "v_before": v_b,
                "v_min": v_m,
                "peak_horiz": float(h.max()),
                "osc_amp": float(np.percentile(h, 95) - np.percentile(h, 5)),
                "peak_along_abs": float(np.max(np.abs(along))),
                "signed_along_at_peak": float(along[int(np.argmax(h))]),
                "att_err_peak": att_peak,
                "att_err_mean": att_mean,
                "img_att_err_peak": img_peak,
                "pred_along_mean": float(np.mean([p["along"] for p in pe])) if pe else None,
                "pred_along_p95": float(np.percentile([p["along"] for p in pe], 95)) if pe else None,
                "n_pred_samples": len(pe),
            }
        )

    # KF relative pose — fix association using source_time_ms as seconds (sim)
    kf_csv = raw / "map_mgmt_audit" / "per_keyframe.csv"
    kf_rel = {"cruise": [], "brake": [], "note": ""}
    if kf_csv.exists() and len(gt):
        krows = list(csv.DictReader(kf_csv.open()))
        kpos = []
        for r in krows:
            try:
                # source_time_ms is sim ms
                t = float(r["source_time_ms"]) * 1e-3
                kpos.append([t, float(r["est_x"]), float(r["est_y"]), float(r["est_z"])])
            except Exception:
                continue
        kpos = np.asarray(kpos) if kpos else np.zeros((0, 4))
        # Verify clock domain overlap
        if len(kpos):
            overlap = (kpos[0, 0] >= gt[0, 0] - 5) and (kpos[-1, 0] <= gt[-1, 0] + 5)
            kf_rel["note"] = (
                f"kf_t=[{kpos[0,0]:.2f},{kpos[-1,0]:.2f}] gt_t=[{gt[0,0]:.2f},{gt[-1,0]:.2f}] "
                f"overlap={overlap} n={len(kpos)}"
            )
            ke, kg, kt = assoc(kpos, gt, max_dt=0.25)
            if len(ke) > 3:
                # Align absolute then evaluate relative increments (removes common yaw/trans)
                ke_al = yaw_xy_align(ke, kg)
                vs = gt_speed(gt)
                for i in range(1, len(ke_al)):
                    er = ke_al[i, :2] - ke_al[i - 1, :2]
                    gr = kg[i, :2] - kg[i - 1, :2]
                    terr = float(np.linalg.norm(er - gr))
                    # rotation about z from 2D headings
                    eh = math.atan2(er[1], er[0]) if np.linalg.norm(er) > 1e-3 else 0.0
                    gh = math.atan2(gr[1], gr[0]) if np.linalg.norm(gr) > 1e-3 else 0.0
                    rerr = abs(wrap(eh - gh))
                    gi = np.searchsorted(gt[:, 0], kt[i])
                    gi = min(max(gi, 0), len(gt) - 1)
                    spd = vs[gi]
                    item = {
                        "trans_err": terr,
                        "rot_err": rerr,
                        "base_est": float(np.linalg.norm(er)),
                        "base_gt": float(np.linalg.norm(gr)),
                        "speed": float(spd),
                    }
                    if spd < 2.5:
                        kf_rel["brake"].append(item)
                    elif spd > 4.0:
                        kf_rel["cruise"].append(item)

    def summarize_rel(items):
        if not items:
            return None
        te = [x["trans_err"] for x in items]
        re = [x["rot_err"] for x in items]
        return {
            "n": len(items),
            "trans_mean": float(np.mean(te)),
            "trans_p95": float(np.percentile(te, 95)),
            "rot_mean_deg": float(np.mean(re) * 180 / np.pi),
            "rot_p95_deg": float(np.percentile(re, 95) * 180 / np.pi),
        }

    # Landmarks near corners vs cruise
    births_path = raw / "landmark_audit" / "landmark_births.jsonl"
    birth_stats = {"n_acc": 0, "n_rej": 0, "depth_mean": None, "reproj_p95": None, "depth_gt80": 0}
    if births_path.exists():
        depths, reproj = [], []
        for line in births_path.open():
            b = json.loads(line)
            if b.get("event") == "birth_accepted":
                birth_stats["n_acc"] += 1
                if b.get("depth_curr") is not None:
                    depths.append(b["depth_curr"])
                    if b["depth_curr"] > 80:
                        birth_stats["depth_gt80"] += 1
                if b.get("reproj_rms_px") is not None:
                    reproj.append(b["reproj_rms_px"])
            elif b.get("event") == "birth_rejected":
                birth_stats["n_rej"] += 1
        if depths:
            birth_stats["depth_mean"] = float(np.mean(depths))
            birth_stats["depth_p95"] = float(np.percentile(depths, 95))
        if reproj:
            birth_stats["reproj_p95"] = float(np.percentile(reproj, 95))

    # A3b 100m reference
    a3b_ref = {}
    if (args.a3b_ref / "ate_summary.json").exists():
        a3b_ref = json.loads((args.a3b_ref / "ate_summary.json").read_text())

    summary = {
        "run_id": args.run_id,
        "mission_done": (raw / "run_meta.txt").read_text().find("mission_done_marker=1") >= 0
        if (raw / "run_meta.txt").exists()
        else None,
        "vmax_gt": float(v.max()) if len(v) else None,
        "v95_gt": float(np.percentile(v, 95)) if len(v) else None,
        "reached_5mps": bool(v.max() >= 4.8) if len(v) else False,
        "n_brakes": len(brakes),
        "brakes": brake_metrics,
        "ate": ate.get("localslam_aligned"),
        "ate_raw": ate.get("localslam_raw"),
        "gslam": ate.get("globalslam_path_aligned_vs_nearest_gt")
        or ate.get("globalslam_path_vs_nearest_gt"),
        "peak_horiz": float(horiz.max()) if len(horiz) else None,
        "att_rmse_rad": float(np.sqrt(np.mean(att_err[:, 4] ** 2))) if len(att_err) else None,
        "att_rmse_deg": float(np.sqrt(np.mean(att_err[:, 4] ** 2)) * 180 / np.pi)
        if len(att_err)
        else None,
        "roll_rmse_deg": float(np.sqrt(np.mean(att_err[:, 1] ** 2)) * 180 / np.pi)
        if len(att_err)
        else None,
        "pitch_rmse_deg": float(np.sqrt(np.mean(att_err[:, 2] ** 2)) * 180 / np.pi)
        if len(att_err)
        else None,
        "yaw_rmse_deg": float(np.sqrt(np.mean(att_err[:, 3] ** 2)) * 180 / np.pi)
        if len(att_err)
        else None,
        "img_att": {
            "n": len(img_att),
            "gt_err_median": float(np.median([x["gt_err"] for x in img_att if x["gt_err"] is not None]))
            if any(x["gt_err"] is not None for x in img_att)
            else None,
            "gt_err_p95": float(np.percentile([x["gt_err"] for x in img_att if x["gt_err"] is not None], 95))
            if any(x["gt_err"] is not None for x in img_att)
            else None,
            "ahrs_vs_gt_median": float(
                np.median([x["ahrs_vs_gt"] for x in img_att if x["ahrs_vs_gt"] is not None])
            )
            if any(x["ahrs_vs_gt"] is not None for x in img_att)
            else None,
            "ahrs_age_ms_p95": float(
                np.percentile([x["ahrs_age_ms"] for x in img_att if x["ahrs_age_ms"] is not None], 95)
            )
            if any(x["ahrs_age_ms"] is not None for x in img_att)
            else None,
        },
        "pred_err_inc": {
            "n": len(pred_err_inc),
            "along_mean": float(np.mean([p["along"] for p in pred_err_inc])) if pred_err_inc else None,
            "along_p95": float(np.percentile([p["along"] for p in pred_err_inc], 95))
            if pred_err_inc
            else None,
            "norm_p95": float(np.percentile([p["norm"] for p in pred_err_inc], 95))
            if pred_err_inc
            else None,
        },
        "stage_help": stage_help,
        "kf_rel_note": kf_rel["note"],
        "kf_rel_cruise": summarize_rel(kf_rel["cruise"]),
        "kf_rel_brake": summarize_rel(kf_rel["brake"]),
        "births": birth_stats,
        "recorder_counts": {
            "gt": len(gt),
            "gt_att": len(gt_att),
            "ahrs": len(ahrs),
            "odom_full": len(odom_full),
            "gkf": len(gkf),
        },
        "a3b_100m_ref_ate": a3b_ref.get("localslam_aligned"),
    }
    (tab / "summary.json").write_text(json.dumps(summary, indent=2))

    # Figures
    if plt is not None and len(gt):
        fig1, ax = plt.subplots(figsize=(8, 8))
        ax.plot(gt[:, 1], gt[:, 2], "k-", lw=1.5, label="GT")
        if len(e_al):
            ax.plot(e_al[:, 0], e_al[:, 1], "C1-", lw=1, label="Local A3.1")
        ax.set_aspect("equal", adjustable="box")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_title("A3.1 LocalSLAM vs GT (380 m rectangle)")
        fig1.tight_layout()
        fig1.savefig(fig / "traj_local_vs_gt.png", dpi=140)
        plt.close(fig1)

        fig2, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        t0 = gt[0, 0]
        axes[0].plot(gt[:, 0] - t0, v, "k-")
        axes[0].axhline(5.0, color="r", ls="--", label="5 m/s")
        axes[0].set_ylabel("GT speed")
        axes[0].legend()
        if len(ts):
            axes[1].plot(ts - t0, horiz, "C0-", lw=0.8)
        axes[1].set_ylabel("horiz err [m]")
        if len(att_err):
            axes[2].plot(att_err[:, 0] - t0, att_err[:, 4] * 180 / np.pi, "C2-", lw=0.7)
        axes[2].set_ylabel("|att err| [deg]")
        if pred_err_inc:
            axes[3].plot(
                [p["t"] - t0 for p in pred_err_inc[::5]],
                [p["along"] for p in pred_err_inc[::5]],
                ".",
                ms=1,
                color="C3",
            )
        axes[3].set_ylabel("pred along err inc")
        axes[3].set_xlabel("t - t0 [s]")
        for ax, wins in zip(axes, [brakes] * 4):
            for a, b, c, *_ in brakes:
                ax.axvspan(a - t0, b - t0, color="r", alpha=0.1)
            ax.grid(True, alpha=0.3)
        axes[0].set_title("A3.1 braking characterization")
        fig2.tight_layout()
        fig2.savefig(fig / "braking_timeseries.png", dpi=140)
        plt.close(fig2)

        # zoom first brake
        if brakes:
            a, b, c, *_ = brakes[0]
            fig3, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
            mgt = (gt[:, 0] >= a) & (gt[:, 0] <= b)
            axes[0].plot(gt[mgt, 0] - c, v[mgt], "k-", label="GT speed")
            if len(odom_full):
                mo = (odom_full[:, 0] >= a) & (odom_full[:, 0] <= b)
                spd_e = np.linalg.norm(odom_full[mo, 7:9], axis=1)
                axes[0].plot(odom_full[mo, 0] - c, spd_e, "C1-", label="|v_est| xy")
            axes[0].legend()
            axes[0].set_ylabel("speed")
            if len(gt_att) and len(odom_full):
                mg = (gt_att[:, 0] >= a) & (gt_att[:, 0] <= b)
                mo = (odom_full[:, 0] >= a) & (odom_full[:, 0] <= b)
                axes[1].plot(gt_att[mg, 0] - c, gt_att[mg, 1] * 180 / np.pi, "k-", label="GT roll")
                axes[1].plot(odom_full[mo, 0] - c, odom_full[mo, 4] * 180 / np.pi, "C1-", label="est roll")
                axes[1].plot(gt_att[mg, 0] - c, gt_att[mg, 2] * 180 / np.pi, "k--", label="GT pitch")
                axes[1].plot(odom_full[mo, 0] - c, odom_full[mo, 5] * 180 / np.pi, "C2--", label="est pitch")
                axes[1].legend(fontsize=8)
            axes[1].set_ylabel("att [deg]")
            mh = (ts >= a) & (ts <= b) if len(ts) else np.zeros(0, bool)
            if mh.any():
                axes[2].plot(ts[mh] - c, horiz[mh], "C0-")
            axes[2].set_ylabel("horiz err")
            axes[2].set_xlabel("t - t_brake [s]")
            for ax in axes:
                ax.grid(True, alpha=0.3)
            axes[0].set_title("First braking window detail")
            fig3.tight_layout()
            fig3.savefig(fig / "brake0_detail.png", dpi=140)
            plt.close(fig3)

    print(json.dumps({k: summary[k] for k in summary if k != "brakes"}, indent=2, default=str)[:4500])
    print("brakes:", json.dumps(brake_metrics, indent=2)[:2000])


if __name__ == "__main__":
    main()
