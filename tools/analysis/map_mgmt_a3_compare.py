#!/usr/bin/env python3
"""Compare A2 (1px) vs A3a (10px) vs A3b (20px) map-mgmt exclusion ablation."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

import numpy as np


def load_summary(run_dir: Path) -> dict:
    for p in (
        run_dir / "summary.json",
        run_dir / "analysis_tmp" / "tables" / "summary.json",
        run_dir / "tables" / "summary.json",
    ):
        if p.exists():
            return json.loads(p.read_text())
    raise FileNotFoundError(f"no summary.json under {run_dir}")


def load_enriched(run_dir: Path) -> list[dict]:
    for p in (
        run_dir / "per_keyframe_enriched.csv",
        run_dir / "analysis_tmp" / "tables" / "per_keyframe_enriched.csv",
    ):
        if p.exists():
            with p.open() as f:
                return list(csv.DictReader(f))
    # fallback: rebuild via audit only missing
    return []


def load_kf_audit(run_dir: Path) -> list[dict]:
    p = run_dir / "map_mgmt_audit" / "per_keyframe.csv"
    if not p.exists():
        return []
    with p.open() as f:
        return list(csv.DictReader(f))


def fmean(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return float(np.mean(xs)) if xs else None


def fmed(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return float(np.median(xs)) if xs else None


def lc_stats(run_dir: Path) -> dict:
    log = run_dir / "launch_mission.log"
    if not log.exists():
        return {}
    text = log.read_text(errors="ignore")
    attempts = len(re.findall(r"visual LC attempt", text))
    rejects = len(re.findall(r"PnP reject", text))
    commits = len(re.findall(r"GlobalLoopCommitted source=visual_loop|committed=1", text))
    ms = [int(x) for x in re.findall(r"matches=(\d+)", text)]
    return {
        "lc_attempts": attempts,
        "pnp_rejects": rejects,
        "lc_commits": commits,
        "match_count_mean": float(np.mean(ms)) if ms else None,
        "match_count_max": int(max(ms)) if ms else None,
    }


def ate_metrics(run_dir: Path) -> dict:
    for name in ("ate_summary.json", "ate_summary.json.print"):
        p = run_dir / name
        if not p.exists():
            continue
        txt = p.read_text()
        start = txt.find("{")
        if start < 0:
            continue
        try:
            ate = json.loads(txt[start:])
        except Exception:
            continue
        out = {}
        loc = ate.get("localslam_raw") or {}
        glob_ = ate.get("globalslam_path_vs_nearest_gt") or {}
        out["local_ate_rmse"] = loc.get("ate_rmse")
        out["local_ate_xy"] = loc.get("ate_rmse_xy")
        out["local_ate_z"] = loc.get("ate_rmse_z")
        out["global_ate_rmse"] = glob_.get("ate_rmse")
        out["global_ate_xy"] = glob_.get("ate_rmse_xy")
        out["global_ate_z"] = glob_.get("ate_rmse_z")
        return out
    return {}


def mission_done(run_dir: Path) -> bool:
    meta = run_dir / "run_meta.txt"
    if meta.exists() and "mission_done_marker=1" in meta.read_text():
        return True
    log = run_dir / "launch_mission.log"
    return log.exists() and "no more commands to execute" in log.read_text(errors="ignore")


def effective_excl(kf_rows: list[dict]) -> float | None:
    if not kf_rows:
        return None
    return float(kf_rows[0].get("min_distance_px", "nan"))


def ba_accept_rate(kf_rows: list[dict]) -> float | None:
    ran = [r for r in kf_rows if int(float(r.get("ba_ran", 0))) == 1]
    if not ran:
        return None
    ok = sum(1 for r in ran if int(float(r.get("ba_accepted", 0))) == 1)
    return ok / len(ran)


def collect_run(label: str, excl: int, run_dir: Path) -> dict:
    summary = load_summary(run_dir)
    kf = load_kf_audit(run_dir)
    enr = load_enriched(run_dir)
    seg = summary.get("segment_summary", {})
    explore = seg.get("explore", {})
    revisit = seg.get("revisit", {})
    life = summary.get("lifecycle", {})
    ba = summary.get("ba", {}) or {}
    ate = ate_metrics(run_dir)
    lc = lc_stats(run_dir)

    births = summary.get("n_births_logged", 0)
    deletes = summary.get("n_deletes_logged", 0)
    imm_cull_frac = (deletes / births) if births else None

    # min-distance rejection rate from audit
    rej = sum(int(float(r.get("n_rejected_min_distance", 0))) for r in kf)
    inliers_surv = sum(int(float(r.get("n_triang_submitted", 0))) for r in kf)
    rej_rate = rej / (rej + inliers_surv) if (rej + inliers_surv) else None

    supported = sum(int(float(r.get("n_birth_in_supported_cell", 0))) for r in kf)
    supported_frac = supported / births if births else None

    comb_cov = fmean([float(r.get("grid_combined_occ", 0)) for r in kf])
    # occupancy as fraction of 48 cells
    comb_cov_frac = (comb_cov / 48.0) if comb_cov is not None else None

    init_t = fmed([float(r.get("init_time_s", 0)) for r in kf])
    ba_t = fmed([float(r.get("ba_time_s", 0)) for r in kf if int(float(r.get("ba_ran", 0))) == 1])
    upd_t = fmed([float(r.get("update_time_s", 0)) for r in kf])
    assoc = fmean([float(r.get("n_matched_existing", 0)) for r in kf])

    reuse_vals = []
    for r in enr or []:
        v = r.get("reuse_ratio")
        if v not in (None, "", "nan"):
            try:
                fv = float(v)
                if math.isfinite(fv):
                    reuse_vals.append(fv)
            except ValueError:
                pass
    if not reuse_vals and kf:
        for r in kf:
            a = float(r.get("n_matched_existing", 0))
            b = float(r.get("n_births", 0))
            if a + b > 0:
                reuse_vals.append(a / (a + b))

    return {
        "label": label,
        "exclusion_px": excl,
        "run_dir": str(run_dir),
        "mission_completed": mission_done(run_dir),
        "effective_exclusion_px": effective_excl(kf),
        "path_m": summary.get("path_m"),
        "n_keyframes": summary.get("n_keyframes"),
        "total_births": births,
        "births_per_kf": summary.get("segment_summary", {}).get("all", {}).get("births_per_kf"),
        "births_per_m_explore": explore.get("births_per_m"),
        "births_per_m_revisit": revisit.get("births_per_m"),
        "n_kf_explore": explore.get("n_kf"),
        "n_kf_revisit": revisit.get("n_kf"),
        "path_explore": explore.get("path_m"),
        "path_revisit": revisit.get("path_m"),
        "final_landmarks": summary.get("final_landmarks"),
        "total_deletes": deletes,
        "immediate_cull_frac": imm_cull_frac,
        "subsequent_reuse_frac": life.get("fraction_subsequent_reuse_nm_gt2"),
        "assoc_per_kf": assoc,
        "reuse_ratio": fmean(reuse_vals),
        "min_distance_rejects": rej,
        "min_distance_reject_rate": rej_rate,
        "births_in_supported_cells_frac": supported_frac,
        "combined_coverage_cells": comb_cov,
        "combined_coverage_frac": comb_cov_frac,
        "median_init_time_s": init_t,
        "median_ba_time_s": ba_t,
        "median_update_time_s": upd_t,
        "ba_accept_rate": ba_accept_rate(kf),
        "ba_corr_window": ba.get("corr_time_vs_window_pts"),
        "ba_corr_map": ba.get("corr_time_vs_map_size"),
        "mean_ba_time_s": ba.get("mean_ba_time_s"),
        "mean_window_pts": ba.get("mean_window_pts"),
        "accounting_ok": summary.get("accounting_identity_ok"),
        **ate,
        **lc,
        "hist_visible_explore": explore.get("hist_visible_per_kf"),
        "hist_visible_revisit": revisit.get("hist_visible_per_kf"),
        "assoc_explore": explore.get("assoc_per_kf"),
        "assoc_revisit": revisit.get("assoc_per_kf"),
        "births_per_kf_explore": explore.get("births_per_kf"),
        "births_per_kf_revisit": revisit.get("births_per_kf"),
        "enriched": enr,
        "kf_audit": kf,
    }


def pct_change(new, base):
    if base is None or new is None or base == 0:
        return None
    return 100.0 * (new - base) / base


def write_comparison_table(rows: list[dict], path: Path):
    keys = [
        "label",
        "mission_completed",
        "effective_exclusion_px",
        "path_m",
        "n_keyframes",
        "total_births",
        "births_per_kf",
        "births_per_m_explore",
        "births_per_m_revisit",
        "final_landmarks",
        "total_deletes",
        "immediate_cull_frac",
        "subsequent_reuse_frac",
        "assoc_per_kf",
        "reuse_ratio",
        "min_distance_reject_rate",
        "births_in_supported_cells_frac",
        "combined_coverage_cells",
        "median_init_time_s",
        "median_ba_time_s",
        "median_update_time_s",
        "local_ate_rmse",
        "global_ate_rmse",
        "ba_accept_rate",
        "match_count_mean",
        "match_count_max",
        "lc_commits",
        "lc_attempts",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in keys})


def svg_multi_line(path: Path, series_by_label, title, ylabel, w=920, h=340):
    """series_by_label: {label: (xs, ys, color)}"""
    pad_l, pad_r, pad_t, pad_b = 60, 140, 30, 40
    all_x = [x for xs, _, _ in series_by_label.values() for x in xs]
    all_y = [y for _, ys, _ in series_by_label.values() for y in ys]
    if not all_x or not all_y:
        return
    xmin, xmax = min(all_x), max(all_x)
    ymin, ymax = min(all_y), max(all_y)
    if ymax <= ymin:
        ymax = ymin + 1

    def X(x):
        return pad_l + (x - xmin) / (xmax - xmin) * (w - pad_l - pad_r)

    def Y(y):
        return pad_t + (1 - (y - ymin) / (ymax - ymin)) * (h - pad_t - pad_b)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{w/2}" y="18" text-anchor="middle" font-size="14">{title}</text>',
        f'<text x="14" y="{h/2}" transform="rotate(-90 14 {h/2})" font-size="11">{ylabel}</text>',
        f'<text x="{(w-pad_r+pad_l)/2}" y="{h-8}" text-anchor="middle" font-size="11">GT path [m]</text>',
        f'<line x1="{pad_l}" y1="{h-pad_b}" x2="{w-pad_r}" y2="{h-pad_b}" stroke="#333"/>',
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{h-pad_b}" stroke="#333"/>',
    ]
    for i, (lab, (xs, ys, col)) in enumerate(series_by_label.items()):
        pts = " ".join(f"{X(x):.1f},{Y(y):.1f}" for x, y in zip(xs, ys))
        parts.append(
            f'<polyline fill="none" stroke="{col}" stroke-width="1.6" points="{pts}"/>'
        )
        parts.append(
            f'<text x="{w-pad_r+8}" y="{pad_t+16+i*14}" font-size="11" fill="{col}">{lab}</text>'
        )
    parts.append("</svg>")
    path.write_text("\n".join(parts))


def bar_svg(path: Path, labels, groups, title, w=720, h=320):
    """groups: [(name, values, color)] values aligned with labels"""
    pad_l, pad_r, pad_t, pad_b = 60, 20, 40, 60
    n = len(labels)
    g = len(groups)
    ymax = max(max(v) for _, v, _ in groups) * 1.15 if groups else 1
    bw = (w - pad_l - pad_r) / max(n, 1)
    bar_w = bw / (g + 1)

    def Y(y):
        return pad_t + (1 - y / ymax) * (h - pad_t - pad_b)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{w/2}" y="18" text-anchor="middle" font-size="14">{title}</text>',
        f'<line x1="{pad_l}" y1="{h-pad_b}" x2="{w-pad_r}" y2="{h-pad_b}" stroke="#333"/>',
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{h-pad_b}" stroke="#333"/>',
    ]
    for i, lab in enumerate(labels):
        x0 = pad_l + i * bw
        for j, (name, vals, col) in enumerate(groups):
            v = vals[i]
            x = x0 + (j + 0.5) * bar_w
            y = Y(v)
            parts.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w*0.9:.1f}" height="{h-pad_b-y:.1f}" fill="{col}"/>'
            )
        parts.append(
            f'<text x="{x0+bw/2:.1f}" y="{h-pad_b+16}" text-anchor="middle" font-size="11">{lab}</text>'
        )
    for j, (name, _, col) in enumerate(groups):
        parts.append(
            f'<text x="{pad_l+j*120}" y="{h-8}" font-size="11" fill="{col}">{name}</text>'
        )
    parts.append("</svg>")
    path.write_text("\n".join(parts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a2-dir", type=Path, required=True)
    ap.add_argument("--a3a-dir", type=Path, required=True)
    ap.add_argument("--a3b-dir", type=Path, required=True)
    ap.add_argument("--evidence-dir", type=Path, required=True)
    args = ap.parse_args()

    ev = args.evidence_dir
    fig = ev / "figures"
    tab = ev / "tables"
    fig.mkdir(parents=True, exist_ok=True)
    tab.mkdir(parents=True, exist_ok=True)

    runs = [
        collect_run("A2_1px", 1, args.a2_dir),
        collect_run("A3a_10px", 10, args.a3a_dir),
        collect_run("A3b_20px", 20, args.a3b_dir),
    ]
    # strip heavy fields for json
    slim = []
    for r in runs:
        s = {k: v for k, v in r.items() if k not in ("enriched", "kf_audit")}
        slim.append(s)
    (tab / "comparison_summary.json").write_text(json.dumps(slim, indent=2))
    write_comparison_table(slim, tab / "comparison_table.csv")

    base = slim[0]
    pct_rows = []
    for r in slim[1:]:
        pct_rows.append(
            {
                "label": r["label"],
                "pct_total_births": pct_change(r["total_births"], base["total_births"]),
                "pct_final_landmarks": pct_change(
                    r["final_landmarks"], base["final_landmarks"]
                ),
                "pct_births_per_m_explore": pct_change(
                    r["births_per_m_explore"], base["births_per_m_explore"]
                ),
                "pct_births_per_m_revisit": pct_change(
                    r["births_per_m_revisit"], base["births_per_m_revisit"]
                ),
                "pct_immediate_culls": pct_change(
                    r["total_deletes"], base["total_deletes"]
                ),
                "pct_subsequent_reuse": pct_change(
                    r["subsequent_reuse_frac"], base["subsequent_reuse_frac"]
                ),
                "pct_assoc_per_kf": pct_change(r["assoc_per_kf"], base["assoc_per_kf"]),
                "pct_median_init_time": pct_change(
                    r["median_init_time_s"], base["median_init_time_s"]
                ),
                "pct_median_update_time": pct_change(
                    r["median_update_time_s"], base["median_update_time_s"]
                ),
                "pct_global_ate": pct_change(
                    r.get("global_ate_rmse"), base.get("global_ate_rmse")
                ),
                "pct_local_ate": pct_change(
                    r.get("local_ate_rmse"), base.get("local_ate_rmse")
                ),
            }
        )
    with (tab / "pct_change_vs_a2.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(pct_rows[0].keys()))
        w.writeheader()
        w.writerows(pct_rows)

    colors = {"A2_1px": "#1f77b4", "A3a_10px": "#ff7f0e", "A3b_20px": "#2ca02c"}

    # Fig1 map growth
    series = {}
    for r in runs:
        enr = r["enriched"]
        if not enr:
            continue
        xs = [float(x["cum_path_m"]) for x in enr]
        ys = [float(x["n_anchors_after"]) for x in enr]
        series[r["label"]] = (xs, ys, colors[r["label"]])
    svg_multi_line(fig / "map_growth_vs_path.svg", series, "Global map growth vs GT path", "landmarks")

    # Fig2 births and assoc — plot births for each
    series_b = {}
    for r in runs:
        enr = r["enriched"]
        if not enr:
            continue
        xs = [float(x["cum_path_m"]) for x in enr]
        # rolling mean 5
        b = np.array([float(x["n_births"]) for x in enr], dtype=float)
        a = np.array([float(x["n_matched_existing"]) for x in enr], dtype=float)
        kernel = np.ones(5) / 5
        if len(b) >= 5:
            b = np.convolve(b, kernel, mode="same")
            a = np.convolve(a, kernel, mode="same")
        series_b[r["label"] + " births"] = (xs, b.tolist(), colors[r["label"]])
    svg_multi_line(
        fig / "births_vs_path.svg",
        series_b,
        "New landmark births vs path (5-KF smooth)",
        "births/KF",
    )

    # Fig3 bar birth/retain/cull
    labels = [r["label"] for r in slim]
    bar_svg(
        fig / "birth_cull_retain.svg",
        labels,
        [
            ("births", [r["total_births"] for r in slim], "#1f77b4"),
            ("deletes", [r["total_deletes"] for r in slim], "#d62728"),
            ("final", [r["final_landmarks"] for r in slim], "#2ca02c"),
        ],
        "Births, deletes, final map size",
    )

    # Fig4 coverage / nn
    bar_svg(
        fig / "coverage_supported.svg",
        labels,
        [
            (
                "supported birth frac",
                [
                    (r["births_in_supported_cells_frac"] or 0) * 100
                    for r in slim
                ],
                "#ff7f0e",
            ),
            (
                "comb coverage % cells",
                [(r["combined_coverage_frac"] or 0) * 100 for r in slim],
                "#9467bd",
            ),
            (
                "min-dis reject %",
                [(r["min_distance_reject_rate"] or 0) * 100 for r in slim],
                "#8c564b",
            ),
        ],
        "Coverage and exclusion rejection (%)",
    )

    # Fig5 ATE
    bar_svg(
        fig / "trajectory_ate.svg",
        labels,
        [
            ("Local ATE", [r.get("local_ate_rmse") or 0 for r in slim], "#1f77b4"),
            ("Global ATE", [r.get("global_ate_rmse") or 0 for r in slim], "#ff7f0e"),
        ],
        "Trajectory ATE RMSE [m]",
    )

    # Fig6 compute
    bar_svg(
        fig / "compute_times.svg",
        labels,
        [
            ("median init [s]", [r.get("median_init_time_s") or 0 for r in slim], "#1f77b4"),
            ("median BA [s]", [r.get("median_ba_time_s") or 0 for r in slim], "#ff7f0e"),
            ("median update [s]", [r.get("median_update_time_s") or 0 for r in slim], "#2ca02c"),
        ],
        "Computational times (median)",
    )

    print(json.dumps({"runs": slim, "pct_vs_a2": pct_rows}, indent=2))


if __name__ == "__main__":
    main()
