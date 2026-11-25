#!/usr/bin/env python3
"""GSLAM-MAP-MGMT-A2 offline analysis from instrumented mission outputs."""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None


def load_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def fget(row: dict, key: str, default: float = 0.0) -> float:
    v = row.get(key, "")
    if v is None or v == "" or v == "nan":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def iget(row: dict, key: str, default: int = 0) -> int:
    return int(round(fget(row, key, float(default))))


def load_xyz_csv(path: Path) -> np.ndarray:
    if not path.exists():
        return np.zeros((0, 4))
    rows = []
    with path.open() as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(
                [float(row["t"]), float(row["x"]), float(row["y"]), float(row["z"])]
            )
    return np.asarray(rows, dtype=np.float64) if rows else np.zeros((0, 4))


def associate_gt(est_xyz: np.ndarray, gt: np.ndarray, max_dt: float = 0.2):
    """Nearest GT for each (t,x,y,z) est row."""
    if len(est_xyz) == 0 or len(gt) == 0:
        return np.full((len(est_xyz), 3), np.nan)
    gt_t = gt[:, 0]
    out = np.full((len(est_xyz), 3), np.nan)
    for i, row in enumerate(est_xyz):
        j = np.searchsorted(gt_t, row[0])
        cands = [c for c in (j - 1, j) if 0 <= c < len(gt)]
        best = None
        best_dt = max_dt
        for k in cands:
            dt = abs(gt_t[k] - row[0])
            if dt <= best_dt:
                best_dt = dt
                best = k
        if best is not None:
            out[i] = gt[best, 1:4]
    return out


def cum_path(xyz: np.ndarray) -> np.ndarray:
    if len(xyz) == 0:
        return np.zeros(0)
    d = np.zeros(len(xyz))
    for i in range(1, len(xyz)):
        if np.any(np.isnan(xyz[i])) or np.any(np.isnan(xyz[i - 1])):
            d[i] = d[i - 1]
        else:
            d[i] = d[i - 1] + float(np.linalg.norm(xyz[i] - xyz[i - 1]))
    return d


def footprint_disk(xy: np.ndarray, radius_m: float, res: float = 2.0):
    """Approximate nadir footprint as disk on z=0 plane (diagnostic)."""
    if np.any(np.isnan(xy)):
        return set()
    cells = set()
    r2 = radius_m * radius_m
    n = int(math.ceil(radius_m / res))
    cx, cy = xy[0], xy[1]
    for ix in range(-n, n + 1):
        for iy in range(-n, n + 1):
            dx = ix * res
            dy = iy * res
            if dx * dx + dy * dy <= r2:
                cells.add((int(math.floor((cx + dx) / res)), int(math.floor((cy + dy) / res))))
    return cells


def classify_overlap(overlap: float, t_low: float = 0.15, t_high: float = 0.45) -> str:
    if overlap < t_low:
        return "explore"
    if overlap < t_high:
        return "partial"
    return "revisit"


def savefig(path: Path):
    if plt is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--evidence-dir", type=Path, required=True)
    ap.add_argument("--footprint-radius-m", type=float, default=25.0)
    ap.add_argument("--horizon-kf", type=int, default=20)
    args = ap.parse_args()

    run = args.run_dir
    ev = args.evidence_dir
    fig = ev / "figures"
    tab = ev / "tables"
    fig.mkdir(parents=True, exist_ok=True)
    tab.mkdir(parents=True, exist_ok=True)

    audit = run / "map_mgmt_audit"
    kf_rows = load_csv(audit / "per_keyframe.csv")
    births = load_csv(audit / "per_landmark_birth.csv")
    deletes = load_csv(audit / "per_landmark_delete.csv")
    ba_rows = load_csv(audit / "ba_events.csv")

    gt = load_xyz_csv(run / "gt_pose.csv")
    if len(gt) == 0:
        gt = load_xyz_csv(run / "gt.csv")

    # Build est time from wall_s if source_time unreliable; prefer source_time_ms.
    est = []
    for r in kf_rows:
        t_ms = fget(r, "source_time_ms", -1)
        t = t_ms / 1000.0 if t_ms > 0 else fget(r, "wall_s")
        est.append([t, fget(r, "est_x"), fget(r, "est_y"), fget(r, "est_z")])
    est_a = np.asarray(est, dtype=np.float64) if est else np.zeros((0, 4))
    gt_xyz = associate_gt(est_a, gt)
    # Prefer GT for path / footprint; fall back to est.
    path_xyz = np.where(np.isnan(gt_xyz), est_a[:, 1:4], gt_xyz)
    cum = cum_path(path_xyz)

    # Historical footprint overlap (vs footprints older than horizon_kf).
    footprints = []
    for i in range(len(kf_rows)):
        footprints.append(footprint_disk(path_xyz[i, :2], args.footprint_radius_m))

    hist_overlap = np.zeros(len(kf_rows))
    recent_overlap = np.zeros(len(kf_rows))
    labels = []
    for i in range(len(kf_rows)):
        cur = footprints[i]
        if not cur:
            labels.append("explore")
            continue
        hist_u = set()
        recent_u = set()
        for j in range(i):
            if i - j > args.horizon_kf:
                hist_u |= footprints[j]
            else:
                recent_u |= footprints[j]
        hist_overlap[i] = len(cur & hist_u) / max(1, len(cur))
        recent_overlap[i] = len(cur & recent_u) / max(1, len(cur))
        labels.append(classify_overlap(hist_overlap[i]))

    # Per-KF derived table
    out_rows = []
    for i, r in enumerate(kf_rows):
        births_i = iget(r, "n_births")
        assoc_i = iget(r, "n_matched_existing")
        denom = assoc_i + births_i
        reuse = assoc_i / denom if denom > 0 else float("nan")
        out_rows.append(
            {
                "kf_idx": iget(r, "kf_idx"),
                "keyframe_id": r.get("keyframe_id", ""),
                "cum_path_m": float(cum[i]) if i < len(cum) else 0.0,
                "gt_x": float(path_xyz[i, 0]),
                "gt_y": float(path_xyz[i, 1]),
                "gt_z": float(path_xyz[i, 2]),
                "hist_overlap": float(hist_overlap[i]),
                "recent_overlap": float(recent_overlap[i]),
                "segment": labels[i],
                "n_births": births_i,
                "n_matched_existing": assoc_i,
                "reuse_ratio": reuse,
                "n_hist_visible": iget(r, "n_hist_visible"),
                "n_excluded_age": iget(r, "n_excluded_age"),
                "n_eligible_visible": iget(r, "n_eligible_visible"),
                "n_rejected_min_distance": iget(r, "n_rejected_min_distance"),
                "n_rejected_birth_gate": iget(r, "n_rejected_birth_gate"),
                "n_anchors_after": iget(r, "n_anchors_after"),
                "n_deleted": iget(r, "n_deleted"),
                "grid_assoc_occ": iget(r, "grid_assoc_occ"),
                "grid_birth_occ": iget(r, "grid_birth_occ"),
                "n_birth_in_supported_cell": iget(r, "n_birth_in_supported_cell"),
                "mean_nn_px_birth_to_assoc": fget(r, "mean_nn_px_birth_to_assoc", -1),
                "ba_n_pts": iget(r, "ba_n_pts"),
                "ba_time_s": fget(r, "ba_time_s"),
                "update_time_s": fget(r, "update_time_s"),
            }
        )

    with (tab / "per_keyframe_enriched.csv").open("w", newline="") as f:
        if out_rows:
            w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
            w.writeheader()
            w.writerows(out_rows)

    # Segment summary
    by_seg = defaultdict(list)
    for row in out_rows:
        by_seg[row["segment"]].append(row)

    def seg_stats(rows):
        if not rows:
            return {}
        births = np.array([r["n_births"] for r in rows], dtype=float)
        assoc = np.array([r["n_matched_existing"] for r in rows], dtype=float)
        hist_v = np.array([r["n_hist_visible"] for r in rows], dtype=float)
        path_d = rows[-1]["cum_path_m"] - rows[0]["cum_path_m"]
        path_d = max(path_d, 1e-6)
        reuse = []
        for r in rows:
            if math.isfinite(r["reuse_ratio"]):
                reuse.append(r["reuse_ratio"])
        return {
            "n_kf": len(rows),
            "path_m": float(path_d),
            "births_total": float(births.sum()),
            "births_per_kf": float(births.mean()),
            "births_per_m": float(births.sum() / path_d),
            "assoc_per_kf": float(assoc.mean()),
            "hist_visible_per_kf": float(hist_v.mean()),
            "reuse_ratio_mean": float(np.mean(reuse)) if reuse else None,
            "min_distance_rejects_total": float(
                sum(r["n_rejected_min_distance"] for r in rows)
            ),
            "birth_gate_rejects_total": float(
                sum(r["n_rejected_birth_gate"] for r in rows)
            ),
            "birth_in_supported_cell_total": float(
                sum(r["n_birth_in_supported_cell"] for r in rows)
            ),
            "births_total_for_supported_frac": float(births.sum()),
        }

    summary = {s: seg_stats(rows) for s, rows in by_seg.items()}
    summary["all"] = seg_stats(out_rows)

    # Accounting check
    accounting = []
    for r in kf_rows:
        before = iget(r, "n_anchors_before")
        after = iget(r, "n_anchors_after")
        births_i = iget(r, "n_births")
        deleted = iget(r, "n_deleted")
        expect = before + births_i - deleted
        accounting.append(
            {
                "kf_idx": iget(r, "kf_idx"),
                "before": before,
                "births": births_i,
                "deleted": deleted,
                "after": after,
                "expect": expect,
                "ok": int(expect == after),
            }
        )
    with (tab / "landmark_accounting.csv").open("w", newline="") as f:
        if accounting:
            w = csv.DictWriter(f, fieldnames=list(accounting[0].keys()))
            w.writeheader()
            w.writerows(accounting)
    acct_ok = all(a["ok"] == 1 for a in accounting) if accounting else False

    # Lifecycle: subsequent reuse from snapshots / n_kf_matched at end
    snap_path = audit / "anchor_snapshot.jsonl"
    final_nm = {}
    if snap_path.exists():
        last = None
        with snap_path.open() as f:
            for line in f:
                line = line.strip()
                if line:
                    last = json.loads(line)
        if last:
            for a in last.get("anchors", []):
                sid = a.get("sid", -1)
                final_nm[sid] = a.get("nm", 0)

    birth_sids = []
    for b in births:
        sid = int(float(b.get("stable_id", -1)))
        birth_sids.append(sid)
    deleted_sids = {int(float(d.get("stable_id", -1))) for d in deletes}

    # Subsequent reuse: nm>2 means more than birth pair (birth sets nm=2)
    n_birth = len(births)
    n_deleted = len(deletes)
    n_with_subsequent = sum(1 for s in birth_sids if final_nm.get(s, 0) > 2)
    n_only_birth_pair = sum(
        1
        for s in birth_sids
        if s not in deleted_sids and final_nm.get(s, 2) <= 2
    )

    # Near-duplicate indicator from final snapshot (spatial NN < 1 m)
    nn_close = 0
    nn_total = 0
    if snap_path.exists() and last:
        pts = np.array([[a["x"], a["y"], a["z"]] for a in last["anchors"]], dtype=float)
        if len(pts) > 1:
            # sample up to 2000 for speed
            idx = np.arange(len(pts))
            if len(pts) > 2000:
                rng = np.random.default_rng(0)
                idx = rng.choice(len(pts), 2000, replace=False)
            sub = pts[idx]
            for i in range(len(sub)):
                d = np.linalg.norm(sub - sub[i], axis=1)
                d[i] = np.inf
                nn_total += 1
                if d.min() < 1.0:
                    nn_close += 1

    # LC events from log
    lc_events = []
    log = run / "launch_mission.log"
    if log.exists():
        text = log.read_text(errors="ignore")
        for line in text.splitlines():
            if "GlobalLoopCommitted" in line or "visual LC: matched" in line:
                lc_events.append(line.strip())

    # BA vs map size
    ba_corr = None
    if ba_rows:
        n_pts = np.array([fget(r, "n_pts") for r in ba_rows])
        n_anc = np.array([fget(r, "n_anchors_total") for r in ba_rows])
        t_ba = np.array([fget(r, "ba_time_s") for r in ba_rows])
        if len(n_pts) > 2:
            ba_corr = {
                "corr_time_vs_window_pts": float(np.corrcoef(n_pts, t_ba)[0, 1]),
                "corr_time_vs_map_size": float(np.corrcoef(n_anc, t_ba)[0, 1]),
                "mean_ba_time_s": float(t_ba.mean()),
                "mean_window_pts": float(n_pts.mean()),
                "final_map_size": float(n_anc[-1]) if len(n_anc) else None,
            }

    # Figures
    if plt is not None and out_rows:
        x = [r["cum_path_m"] for r in out_rows]
        y_map = [r["n_anchors_after"] for r in out_rows]
        y_b = [r["n_births"] for r in out_rows]
        y_a = [r["n_matched_existing"] for r in out_rows]
        y_h = [r["n_hist_visible"] for r in out_rows]
        y_o = [r["hist_overlap"] for r in out_rows]

        plt.figure(figsize=(9, 4))
        plt.plot(x, y_map, label="landmarks")
        for i, lab in enumerate(labels):
            if lab == "revisit" and (i == 0 or labels[i - 1] != "revisit"):
                plt.axvline(x[i], color="C1", alpha=0.35, linewidth=1)
        plt.xlabel("GT path [m]")
        plt.ylabel("Global landmarks")
        plt.title("Map growth vs path")
        plt.legend()
        savefig(fig / "map_growth_vs_path.png")

        plt.figure(figsize=(9, 4))
        plt.plot(x, y_b, label="births/KF", alpha=0.7)
        plt.plot(x, y_a, label="assoc existing/KF", alpha=0.7)
        plt.xlabel("GT path [m]")
        plt.ylabel("count")
        plt.title("Births vs existing associations")
        plt.legend()
        savefig(fig / "births_vs_assoc.png")

        plt.figure(figsize=(9, 4))
        plt.plot(x, y_h, label="hist visible (age-excluded)")
        plt.plot(x, y_o, label="hist footprint overlap")
        plt.xlabel("GT path [m]")
        plt.title("Age-horizon exclusion vs geographic revisit")
        plt.legend()
        savefig(fig / "horizon_vs_revisit.png")

        if ba_rows:
            plt.figure(figsize=(7, 4))
            plt.scatter(
                [fget(r, "n_anchors_total") for r in ba_rows],
                [fget(r, "ba_time_s") for r in ba_rows],
                s=12,
                alpha=0.6,
                label="vs map size",
            )
            plt.scatter(
                [fget(r, "n_pts") for r in ba_rows],
                [fget(r, "ba_time_s") for r in ba_rows],
                s=12,
                alpha=0.6,
                label="vs window pts",
            )
            plt.xlabel("count")
            plt.ylabel("BA time [s]")
            plt.legend()
            plt.title("BA workload")
            savefig(fig / "ba_workload.png")

        # Top-down path + segment colors
        plt.figure(figsize=(6, 6))
        colors = {"explore": "C0", "partial": "C2", "revisit": "C1"}
        for lab in ("explore", "partial", "revisit"):
            xs = [r["gt_x"] for r in out_rows if r["segment"] == lab]
            ys = [r["gt_y"] for r in out_rows if r["segment"] == lab]
            if xs:
                plt.scatter(xs, ys, s=8, c=colors[lab], label=lab)
        plt.axis("equal")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("GT KF positions by footprint class")
        plt.legend()
        savefig(fig / "gt_segments.png")

    # ATE if present
    ate = {}
    ate_print = run / "ate_summary.json.print"
    ate_json = run / "ate_summary.json"
    for p in (ate_json, ate_print):
        if p.exists():
            try:
                txt = p.read_text()
                # file may be print + json
                start = txt.find("{")
                if start >= 0:
                    ate = json.loads(txt[start:])
                    break
            except Exception:
                pass

    results = {
        "n_keyframes": len(kf_rows),
        "n_births_logged": n_birth,
        "n_deletes_logged": n_deleted,
        "final_landmarks": out_rows[-1]["n_anchors_after"] if out_rows else 0,
        "path_m": float(cum[-1]) if len(cum) else 0.0,
        "accounting_identity_ok": acct_ok,
        "accounting_mismatches": sum(1 for a in accounting if a["ok"] == 0),
        "segment_summary": summary,
        "lifecycle": {
            "fraction_subsequent_reuse_nm_gt2": (
                n_with_subsequent / n_birth if n_birth else None
            ),
            "fraction_only_birth_pair_surviving": (
                n_only_birth_pair / n_birth if n_birth else None
            ),
            "n_birth": n_birth,
            "n_with_subsequent": n_with_subsequent,
            "n_only_birth_pair_surviving": n_only_birth_pair,
        },
        "spatial_nn_lt_1m_fraction_sampled": (
            nn_close / nn_total if nn_total else None
        ),
        "ba": ba_corr,
        "lc_log_lines": lc_events[:20],
        "n_lc_log_lines": len(lc_events),
        "ate": ate,
        "footprint_radius_m": args.footprint_radius_m,
        "horizon_kf": args.horizon_kf,
        "thresholds": {"explore_lt": 0.15, "revisit_ge": 0.45},
    }

    (tab / "summary.json").write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
