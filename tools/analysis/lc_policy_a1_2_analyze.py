#!/usr/bin/env python3
"""Offline A1.2 Vgraph integrity + performance analysis from runtime dumps."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


def pct(xs: List[float], p: float) -> float:
    if not xs:
        return float("nan")
    a = sorted(xs)
    i = int(round(p * (len(a) - 1)))
    return a[max(0, min(len(a) - 1, i))]


def load_csv(path: Path) -> List[dict]:
    if not path.is_file():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def reconstruct_reference(obs_by_kf: Dict[int, set], n_kf: int) -> np.ndarray:
    V = np.zeros((n_kf, n_kf), dtype=np.float64)
    for i in range(n_kf):
        for j in range(i + 1, n_kf):
            shared = len(obs_by_kf.get(i, set()) & obs_by_kf.get(j, set()))
            if shared:
                V[i, j] = shared
                V[j, i] = shared
    return V


def load_snapshot(prefix: Path) -> dict:
    meta = load_csv(Path(str(prefix) + "_meta.csv"))
    if not meta:
        return {}
    m = meta[0]
    n_kf = int(m["n_kf"])
    obs_rows = load_csv(Path(str(prefix) + "_observations.csv"))
    lm_rows = load_csv(Path(str(prefix) + "_landmarks.csv"))
    edge_rows = load_csv(Path(str(prefix) + "_vgraph_edges.csv"))

    active = {int(r["landmark_id"]) for r in lm_rows}
    obs_by_kf: Dict[int, set] = defaultdict(set)
    obs_entries = 0
    dup_same_kf = 0
    unresolvable = 0
    uv_mismatch = 0
    for r in obs_rows:
        obs_entries += 1
        kf = int(r["kf_idx"])
        lid = int(r["landmark_id"])
        if lid not in active:
            unresolvable += 1
            continue
        if lid in obs_by_kf[kf]:
            dup_same_kf += 1
        obs_by_kf[kf].add(lid)

    # n_kf_matched vs distinct observers
    observers: Dict[int, set] = defaultdict(set)
    for kf, lids in obs_by_kf.items():
        for lid in lids:
            observers[lid].add(kf)
    n_kf_matched = {int(r["landmark_id"]): int(r["n_kf_matched"]) for r in lm_rows}
    counter_mismatch = 0
    for lid, obs_set in observers.items():
        if n_kf_matched.get(lid, -1) != len(obs_set):
            counter_mismatch += 1

    V_maint = np.zeros((n_kf, n_kf), dtype=np.float64)
    for r in edge_rows:
        i, j = int(r["i"]), int(r["j"])
        w = float(r["weight"])
        if 0 <= i < n_kf and 0 <= j < n_kf:
            V_maint[i, j] = w
            if i != j:
                V_maint[j, i] = w

    V_ref = reconstruct_reference(obs_by_kf, n_kf)
    diff = np.abs(V_maint - V_ref)
    asym = np.abs(V_maint - V_maint.T)
    diag_nz = int(np.count_nonzero(np.diag(V_maint)))
    missing = 0
    stale = 0
    weight_mis = 0
    for i in range(n_kf):
        for j in range(i + 1, n_kf):
            a, b = V_maint[i, j], V_ref[i, j]
            if abs(a - b) > 1e-9:
                weight_mis += 1
                if a == 0 and b > 0:
                    missing += 1
                elif a > 0 and b == 0:
                    stale += 1
    return {
        "boundary": m.get("boundary", ""),
        "n_kf": n_kf,
        "n_lm": int(m["n_lm"]),
        "n_obs": int(m["n_obs"]),
        "obs_entries": obs_entries,
        "dup_same_kf": dup_same_kf,
        "unresolvable": unresolvable,
        "counter_mismatch": counter_mismatch,
        "diag_nonzero": diag_nz,
        "asymmetric_pairs": int(np.sum(asym > 1e-9) // 2),
        "missing_edges": missing,
        "stale_edges": stale,
        "weight_mismatches": weight_mis,
        "max_abs_error": float(diff.max()) if diff.size else 0.0,
        "nonzero_edges_maint": int(np.count_nonzero(np.triu(V_maint, 1))),
        "nonzero_edges_ref": int(np.count_nonzero(np.triu(V_ref, 1))),
        "ok": weight_mis == 0 and diag_nz == 0 and dup_same_kf == 0
        and unresolvable == 0 and counter_mismatch == 0,
        "V_maint": V_maint,
        "V_ref": V_ref,
        "prefix": str(prefix),
    }


def heatmap_svg(path: Path, title: str, V: np.ndarray, max_n: int = 80) -> None:
    n = V.shape[0]
    step = max(1, int(math.ceil(n / max_n)))
    idx = list(range(0, n, step))
    m = len(idx)
    cell = max(2, min(8, 480 // max(m, 1)))
    w = 40 + m * cell
    h = 60 + m * cell
    mx = float(V.max()) if V.size else 1.0
    if mx <= 0:
        mx = 1.0
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="8" y="18" font-size="12">{title}</text>',
        f'<text x="8" y="34" font-size="10" fill="#666">n_kf={n} stride={step} (runtime)</text>',
    ]
    for a, i in enumerate(idx):
        for b, j in enumerate(idx):
            v = float(V[i, j])
            if v <= 0:
                fill = "#f2f2f2"
            else:
                t = min(1.0, v / mx)
                r = int(255 * (1 - t))
                g = int(255 * (1 - 0.55 * t))
                fill = f"#{r:02x}{g:02x}ff"
            x = 30 + b * cell
            y = 50 + a * cell
            lines.append(
                f'<rect x="{x}" y="{y}" width="{cell}" height="{cell}" fill="{fill}"/>'
            )
    lines.append("</svg>")
    path.write_text("\n".join(lines))


def signed_diff_svg(path: Path, title: str, A: np.ndarray, B: np.ndarray, max_n: int = 80) -> None:
    D = B - A
    n = D.shape[0]
    step = max(1, int(math.ceil(n / max_n)))
    idx = list(range(0, n, step))
    m = len(idx)
    cell = max(2, min(8, 480 // max(m, 1)))
    w = 40 + m * cell
    h = 60 + m * cell
    mx = float(np.max(np.abs(D))) if D.size else 1.0
    if mx <= 0:
        mx = 1.0
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="8" y="18" font-size="12">{title}</text>',
        '<text x="8" y="34" font-size="10" fill="#666">blue=+ connectivity, red=− (runtime)</text>',
    ]
    for a, i in enumerate(idx):
        for b, j in enumerate(idx):
            v = float(D[i, j])
            t = min(1.0, abs(v) / mx)
            if abs(v) < 1e-12:
                fill = "#f2f2f2"
            elif v > 0:
                fill = f"#{int(255*(1-t)):02x}{int(255*(1-0.4*t)):02x}ff"
            else:
                fill = f"#ff{int(255*(1-0.4*t)):02x}{int(255*(1-t)):02x}"
            x = 30 + b * cell
            y = 50 + a * cell
            lines.append(
                f'<rect x="{x}" y="{y}" width="{cell}" height="{cell}" fill="{fill}"/>'
            )
    lines.append("</svg>")
    path.write_text("\n".join(lines))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--evidence-dir", required=True)
    args = ap.parse_args()
    run = Path(args.run_dir)
    evid = Path(args.evidence_dir)
    tables = evid / "tables"
    figures = evid / "figures"
    snaps_out = evid / "snapshots"
    tables.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)
    snaps_out.mkdir(parents=True, exist_ok=True)

    vrt = run / "vgraph_runtime"
    idx_rows = load_csv(vrt / "snapshot_index.csv")
    # Sample snapshots: all special boundaries + every Nth ordinary_update.
    selected = []
    ordinary_i = 0
    for row in idx_rows:
        b = row.get("boundary", "")
        keep = b in {
            "after_delete",
            "after_ba",
            "pre_loop_commit",
            "post_loop_refine",
            "authoritative_commit",
            "initial_mapping",
            "mission_end",
        }
        if b == "ordinary_update":
            ordinary_i += 1
            keep = ordinary_i <= 3 or ordinary_i % 8 == 0
        if keep:
            selected.append(row)
    # Always include first/last if present
    if idx_rows:
        if idx_rows[0] not in selected:
            selected.insert(0, idx_rows[0])
        if idx_rows[-1] not in selected:
            selected.append(idx_rows[-1])

    comparisons = []
    snap_objs = []
    for row in selected:
        prefix = Path(row["prefix"])
        # Prefer run-local path; fall back to evidence copy
        if not Path(str(prefix) + "_meta.csv").is_file():
            alt = snaps_out / Path(prefix).name
            if Path(str(alt) + "_meta.csv").is_file():
                prefix = alt
        s = load_snapshot(prefix)
        if not s:
            continue
        snap_objs.append(s)
        comparisons.append({k: v for k, v in s.items() if k not in ("V_maint", "V_ref")})

    with (tables / "vgraph_reference_comparison.csv").open("w", newline="") as f:
        if comparisons:
            w = csv.DictWriter(f, fieldnames=list(comparisons[0].keys()))
            w.writeheader()
            w.writerows(comparisons)
        else:
            f.write("boundary,ok\n")

    # Timing
    reb = load_csv(vrt / "vgraph_rebuild_timing.csv")
    with (tables / "vgraph_rebuild_timing.csv").open("w", newline="") as f:
        if reb:
            w = csv.DictWriter(f, fieldnames=list(reb[0].keys()))
            w.writeheader()
            w.writerows(reb)

    overhead = load_csv(vrt / "gmap_update_rebuild_overhead.csv")
    with (tables / "gmap_update_rebuild_overhead.csv").open("w", newline="") as f:
        if overhead:
            w = csv.DictWriter(f, fieldnames=list(overhead[0].keys()))
            w.writeheader()
            w.writerows(overhead)

    loop_oh = load_csv(vrt / "loop_transaction_rebuild_overhead.csv")
    with (tables / "loop_transaction_rebuild_overhead.csv").open("w", newline="") as f:
        if loop_oh:
            w = csv.DictWriter(f, fieldnames=list(loop_oh[0].keys()))
            w.writeheader()
            w.writerows(loop_oh)

    excl = load_csv(vrt / "visibility_candidate_exclusions.csv")
    with (tables / "visibility_candidate_exclusions.csv").open("w", newline="") as f:
        if excl:
            w = csv.DictWriter(f, fieldnames=list(excl[0].keys()))
            w.writeheader()
            w.writerows(excl)

    ba = load_csv(vrt / "ba_window_composition.csv")
    with (tables / "ba_window_composition.csv").open("w", newline="") as f:
        if ba:
            w = csv.DictWriter(f, fieldnames=list(ba[0].keys()))
            w.writeheader()
            w.writerows(ba)

    # Funnel from m7 runtime
    funnel = load_csv(run / "m7_runtime" / "detection_funnel.csv")
    with (tables / "detection_funnel.csv").open("w", newline="") as f:
        if funnel:
            w = csv.DictWriter(f, fieldnames=list(funnel[0].keys()))
            w.writeheader()
            w.writerows(funnel)

    # Figures from selected snapshots
    by_boundary = defaultdict(list)
    for s in snap_objs:
        by_boundary[s["boundary"]].append(s)

    def pick(name: str):
        xs = by_boundary.get(name, [])
        return xs[-1] if xs else None

    for name, label in [
        ("ordinary_update", "mid_ordinary"),
        ("after_delete", "after_delete"),
        ("pre_loop_commit", "pre_loop"),
        ("post_loop_refine", "post_loop"),
        ("authoritative_commit", "auth_commit"),
    ]:
        s = pick(name)
        if s is None and name == "ordinary_update" and snap_objs:
            # mid mission by KF count
            s = sorted(snap_objs, key=lambda x: x["n_kf"])[len(snap_objs) // 2]
        if s is None:
            continue
        heatmap_svg(
            figures / f"vgraph_{label}.svg",
            f"Runtime Vgraph ({s['boundary']}) n={s['n_kf']}",
            s["V_maint"],
        )

    pre = pick("pre_loop_commit")
    post = pick("post_loop_refine")
    if pre and post and pre["V_maint"].shape == post["V_maint"].shape:
        heatmap_svg(figures / "vgraph_before_first_post_loop.svg",
                     "Before post-loop refine", pre["V_maint"])
        heatmap_svg(figures / "vgraph_after_first_post_loop.svg",
                     "After post-loop refine", post["V_maint"])
        signed_diff_svg(
            figures / "vgraph_signed_diff_first_loop.svg",
            "Signed visibility difference (post − pre)",
            pre["V_maint"], post["V_maint"],
        )

    # Cost vs map size
    if reb:
        xs = [float(r["n_kf"]) for r in reb]
        ys = [float(r["duration_ms"]) for r in reb]
        # bin by KF
        bins = defaultdict(list)
        for x, y in zip(xs, ys):
            bins[int(x // 25) * 25].append(y)
        w, h = 720, 360
        pad_l, pad_b, pad_t = 55, 45, 40
        lines = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
            '<rect width="100%" height="100%" fill="white"/>',
            '<text x="12" y="22" font-size="13">Runtime reconstruction duration vs KF count</text>',
        ]
        if bins:
            ks = sorted(bins)
            meds = [pct(bins[k], 0.5) for k in ks]
            ymax = max(meds + [1.0]) * 1.15
            plot_w = w - pad_l - 20
            plot_h = h - pad_t - pad_b
            x0, y0 = pad_l, pad_t + plot_h
            lines.append(f'<line x1="{x0}" y1="{y0}" x2="{x0+plot_w}" y2="{y0}" stroke="#333"/>')
            lines.append(f'<line x1="{x0}" y1="{pad_t}" x2="{x0}" y2="{y0}" stroke="#333"/>')
            pts = []
            xmin, xmax = min(ks), max(ks) or 1
            for k, med in zip(ks, meds):
                x = x0 + ((k - xmin) / max(1, xmax - xmin)) * plot_w
                y = y0 - (med / ymax) * plot_h
                pts.append((x, y))
                lines.append(f'<circle cx="{x}" cy="{y}" r="3.5" fill="#1f77b4"/>')
            for i in range(len(pts) - 1):
                lines.append(
                    f'<line x1="{pts[i][0]}" y1="{pts[i][1]}" x2="{pts[i+1][0]}" '
                    f'y2="{pts[i+1][1]}" stroke="#1f77b4" stroke-width="2"/>'
                )
        lines.append("</svg>")
        (figures / "reconstruction_duration_vs_map_size.svg").write_text("\n".join(lines))

    if overhead:
        fracs = [float(r["rebuild_frac"]) for r in overhead]
        nreb = [int(float(r["n_rebuilds"])) for r in overhead]
        # simple strip chart of rebuild fraction
        w, h = 720, 280
        lines = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
            '<rect width="100%" height="100%" fill="white"/>',
            '<text x="12" y="22" font-size="13">Aggregate rebuild fraction of GMAP update</text>',
        ]
        if fracs:
            ymax = max(fracs + [0.01]) * 1.1
            for i, fr in enumerate(fracs[:: max(1, len(fracs)//200)]):
                x = 40 + i * max(1, (w - 60) // max(1, len(fracs[:: max(1, len(fracs)//200)])))
                bar_h = (fr / ymax) * (h - 60)
                lines.append(
                    f'<rect x="{x}" y="{h-40-bar_h}" width="2" height="{bar_h}" fill="#ff7f0e"/>'
                )
        lines.append(
            f'<text x="12" y="{h-12}" font-size="11">median_frac={pct(fracs,0.5):.4f} '
            f'p95={pct(fracs,0.95):.4f} median_n_rebuilds={pct([float(x) for x in nreb],0.5):.1f}</text>'
        )
        lines.append("</svg>")
        (figures / "aggregate_rebuild_overhead.svg").write_text("\n".join(lines))

    if excl:
        # stacked totals
        sums = Counter()
        for r in excl:
            sums["v0"] += int(float(r["n_eligible_v0"]))
            sums["v1"] += int(float(r["n_excluded_v1"]))
            sums["v2_5"] += int(float(r["n_excluded_v2_5"]))
            sums["v6_20"] += int(float(r["n_excluded_v6_20"]))
            sums["v21p"] += int(float(r["n_excluded_v21p"]))
        labels = ["V=0 eligible", "excl V=1", "excl V=2–5", "excl V=6–20", "excl V≥21"]
        keys = ["v0", "v1", "v2_5", "v6_20", "v21p"]
        vals = [sums[k] for k in keys]
        mx = max(vals + [1])
        w, h = 720, 320
        lines = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
            '<rect width="100%" height="100%" fill="white"/>',
            '<text x="12" y="22" font-size="13">Visibility weight bins across CLOOP queries</text>',
        ]
        bw = 90
        for i, (lab, v) in enumerate(zip(labels, vals)):
            bh = (v / mx) * 220
            x = 60 + i * 120
            lines.append(
                f'<rect x="{x}" y="{260-bh}" width="{bw}" height="{bh}" fill="#2ca02c"/>'
            )
            lines.append(
                f'<text x="{x+bw/2}" y="280" font-size="10" text-anchor="middle">{lab}</text>'
            )
            lines.append(
                f'<text x="{x+bw/2}" y="{250-bh}" font-size="10" text-anchor="middle">{v}</text>'
            )
        lines.append("</svg>")
        (figures / "visibility_exclusion_bins.svg").write_text("\n".join(lines))

    # Trajectory figure if CSV present
    gt = run / "gt_xy.csv"
    loc = run / "localslam_xy.csv"
    glo = run / "globalslam_xy.csv"
    # recorder may use different names — probe
    cand = {
        "gt": list(run.glob("*gt*.csv")) + list(run.glob("ground*.csv")),
        "local": list(run.glob("*local*.csv")),
        "global": list(run.glob("*global*.csv")),
    }

    def read_xy(paths):
        for p in paths:
            rows = load_csv(p)
            if not rows:
                continue
            keys = rows[0].keys()
            xk = next((k for k in keys if k.lower() in ("x", "px", "pos_x")), None)
            yk = next((k for k in keys if k.lower() in ("y", "py", "pos_y")), None)
            if xk and yk:
                xs = [float(r[xk]) for r in rows]
                ys = [float(r[yk]) for r in rows]
                return xs, ys, p.name
        return [], [], ""

    gtx, gty, _ = read_xy(cand["gt"] or [gt])
    lx, ly, _ = read_xy(cand["local"] or [loc])
    gx, gy, _ = read_xy(cand["global"] or [glo])
    if gtx or lx or gx:
        allx = gtx + lx + gx
        ally = gty + ly + gy
        xmin, xmax = min(allx), max(allx)
        ymin, ymax = min(ally), max(ally)
        pad = 5
        w, h = 720, 720

        def mapx(x):
            return 40 + (x - xmin + pad) / (xmax - xmin + 2 * pad) * (w - 80)

        def mapy(y):
            return h - 40 - (y - ymin + pad) / (ymax - ymin + 2 * pad) * (h - 80)

        def poly(xs, ys, color):
            if len(xs) < 2:
                return ""
            pts = " ".join(f"{mapx(x):.1f},{mapy(y):.1f}" for x, y in zip(xs, ys))
            return f'<polyline fill="none" stroke="{color}" stroke-width="1.5" points="{pts}"/>'

        svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
            '<rect width="100%" height="100%" fill="white"/>',
            '<text x="12" y="22" font-size="13">GT / LocalSLAM / GlobalSLAM XY</text>',
            poly(gtx, gty, "#111"),
            poly(lx, ly, "#1f77b4"),
            poly(gx, gy, "#d62728"),
            "</svg>",
        ]
        (figures / "trajectories_xy.svg").write_text("\n".join(svg))

    # Summary JSON
    reb_ms = [float(r["duration_ms"]) for r in reb] if reb else []
    ordinary = [r for r in reb if r.get("reason", "").startswith("new_") or r.get("reason") == "deleted_points"]
    postloop = [r for r in reb if "post_loop" in r.get("reason", "")]
    summary = {
        "n_snapshots": len(comparisons),
        "snapshots_ok": sum(1 for c in comparisons if c.get("ok")),
        "snapshots_fail": sum(1 for c in comparisons if not c.get("ok")),
        "rebuild_count": len(reb),
        "rebuild_med_ms": pct(reb_ms, 0.5) if reb_ms else None,
        "rebuild_p95_ms": pct(reb_ms, 0.95) if reb_ms else None,
        "rebuild_max_ms": max(reb_ms) if reb_ms else None,
        "ordinary_rebuild_med_ms": pct([float(r["duration_ms"]) for r in ordinary], 0.5) if ordinary else None,
        "postloop_rebuild_med_ms": pct([float(r["duration_ms"]) for r in postloop], 0.5) if postloop else None,
        "gmap_update_rows": len(overhead),
        "median_n_rebuilds_per_update": pct(
            [float(r["n_rebuilds"]) for r in overhead], 0.5) if overhead else None,
        "median_rebuild_frac": pct(
            [float(r["rebuild_frac"]) for r in overhead], 0.5) if overhead else None,
        "p95_rebuild_frac": pct(
            [float(r["rebuild_frac"]) for r in overhead], 0.95) if overhead else None,
        "visibility_queries": len(excl),
        "ba_window_rows": len(ba),
        "loop_txn_rows": len(loop_oh),
        "insert_flush": load_csv(vrt / "observation_insert_results.csv")[-1]
        if load_csv(vrt / "observation_insert_results.csv") else None,
    }
    (tables / "runtime_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
