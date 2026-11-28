#!/usr/bin/env python3
"""Offline analysis for GSLAM-MAP-ARCH-M7.7 Webots run."""
from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def _svg_polyline(xs, ys, w=720, h=420, pad=40, stroke="#1f77b4", label="") -> str:
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if len(xs) == 0:
        return ""
    xmin, xmax = float(np.min(xs)), float(np.max(xs))
    ymin, ymax = float(np.min(ys)), float(np.max(ys))
    if xmax <= xmin:
        xmax = xmin + 1.0
    if ymax <= ymin:
        ymax = ymin + 1.0

    def sx(x):
        return pad + (x - xmin) / (xmax - xmin) * (w - 2 * pad)

    def sy(y):
        return h - pad - (y - ymin) / (ymax - ymin) * (h - 2 * pad)

    pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in zip(xs, ys))
    return (
        f'<polyline fill="none" stroke="{stroke}" stroke-width="1.5" points="{pts}"/>'
        + (f'<text x="{pad}" y="16" font-size="12" fill="{stroke}">{label}</text>' if label else "")
    )


def write_svg(path: Path, body: str, w=720, h=420, title="") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">'
        f'<rect width="100%" height="100%" fill="white"/>'
        f'<text x="12" y="18" font-size="14" font-family="sans-serif">{title}</text>'
        f"{body}</svg>\n"
    )


def read_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    a = np.asarray(xs, dtype=float)
    return float(np.percentile(a, p))


def write_csv(path: Path, rows: list[dict], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fields = fieldnames or list(rows[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def parse_run_meta(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text(errors="replace").splitlines():
        if "=" in line and not line.startswith("---"):
            k, _, v = line.partition("=")
            out[k.strip()] = v.strip()
    return out


def analyze(run_dir: Path, evid: Path) -> dict:
    tables = evid / "tables"
    figures = evid / "figures"
    tables.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)

    audit = run_dir / "map_mgmt_audit"
    idinv = run_dir / "identity_invariant"
    m7 = run_dir / "m7_runtime"
    log = (run_dir / "launch_mission.log").read_text(errors="replace")
    meta = parse_run_meta(run_dir / "run_meta.txt")

    pk = read_csv(audit / "per_keyframe.csv")
    births = read_csv(audit / "per_landmark_birth.csv")
    deletes = read_csv(audit / "per_landmark_delete.csv")
    inv_s = read_csv(idinv / "invariant_samples.csv")
    inv_v = read_csv(idinv / "invariant_violations.csv")
    extracts = read_csv(m7 / "detection_extract.csv")
    id_snaps = read_csv(m7 / "identity_snapshots.csv")
    events = read_csv(m7 / "runtime_events.csv")
    mem = read_csv(m7 / "memory_samples.csv")

    # --- Loop / sync from launch log ---
    proposals_submitted = re.findall(
        r"submitted proposal_id=(\d+).*?closing=(\d+).*?matched_kf=(\d+)", log
    )
    # fallback looser
    if not proposals_submitted:
        proposals_submitted = [
            (m.group(1), m.group(2) if m.lastindex and m.lastindex >= 2 else "", "")
            for m in re.finditer(r"submitted proposal_id=(\d+)", log)
        ]

    commits = []
    for m in re.finditer(
        r"Authoritative loop committed proposal=(\d+)\s+reval=(\S+)\s+closing=(\d+)\s+"
        r"delta_L=\[([-\d.eE+]+)\s+([-\d.eE+]+)\]\s+ba_refine=(\d+)",
        log,
    ):
        commits.append(
            {
                "proposal_id": int(m.group(1)),
                "reval": m.group(2),
                "closing_kf": int(m.group(3)),
                "delta_x": float(m.group(4)),
                "delta_y": float(m.group(5)),
                "ba_refine": int(m.group(6)),
            }
        )

    rejects = []
    for m in re.finditer(
        r"Authoritative loop proposal (\d+) (\S+) \(([^)]*)\)", log
    ):
        rejects.append(
            {
                "proposal_id": int(m.group(1)),
                "state": m.group(2),
                "reason": m.group(3),
            }
        )

    sync_lines = {
        "SYNC_REQUEST": len(re.findall(r"SYNC_REQUEST", log)),
        "LOCAL_APPLIED": len(re.findall(r"LOCAL_APPLIED", log)),
        "REFERENCE_REBASED": len(re.findall(r"REFERENCE_REBASED", log)),
        "SYNC_COMPLETE": len(re.findall(r"SYNC_COMPLETE", log)),
        "FIRST_POST_SYNC_KF": len(re.findall(r"FIRST_POST_SYNC_KF", log)),
        "CoordPause": len(re.findall(r"CoordPause", log)),
        "pause_not_acknowledged": len(re.findall(r"pause_not_acknowledged", log)),
    }

    # Same-KF alignment residuals from log if present
    align_residuals = []
    for m in re.finditer(
        r"SYNC_REQUEST.*?closing_kf[=:](\d+).*?delta_L=\(([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+)\)",
        log,
        re.S,
    ):
        align_residuals.append(
            {
                "closing_kf": int(m.group(1)),
                "dx": float(m.group(2)),
                "dy": float(m.group(3)),
                "dz": float(m.group(4)),
            }
        )

    # Identity preservation across commit
    content_rows = []
    by_prop: dict[str, dict[str, dict]] = defaultdict(dict)
    for row in id_snaps:
        by_prop[row["proposal_id"]][row["stage"]] = row
    for pid, stages in by_prop.items():
        start = stages.get("proposal_consume_start")
        commit = stages.get("authoritative_commit")
        if not start or not commit:
            content_rows.append(
                {
                    "proposal_id": pid,
                    "has_start": int(start is not None),
                    "has_commit": int(commit is not None),
                    "intervening_kf_preserved": "",
                    "intervening_lm_preserved": "",
                    "allocator_monotonic": "",
                    "note": "incomplete_pair",
                }
            )
            continue
        skf = set(start["kf_ids"].split(";")) - {""}
        ckf = set(commit["kf_ids"].split(";")) - {""}
        slm = set(start["lm_ids"].split(";")) - {""}
        clm = set(commit["lm_ids"].split(";")) - {""}
        # Intervening = in commit not in start must be allowed; start subset of commit for KFs/landmarks
        # that were not legitimately deleted. We check start KFs ⊆ commit KFs (closing+history preserved)
        # and start landmarks that remain active ⊆ commit (cannot distinguish deletes without delete log).
        kf_lost = sorted(skf - ckf)
        # For landmarks, lost IDs may be legitimate culls during correction workspace — track counts.
        lm_lost = sorted(slm - clm)
        alloc_ok = int(commit["next_landmark_id"]) >= int(start["next_landmark_id"])
        content_rows.append(
            {
                "proposal_id": pid,
                "has_start": 1,
                "has_commit": 1,
                "start_n_kf": start["n_kf"],
                "commit_n_kf": commit["n_kf"],
                "start_n_lm": start["n_lm"],
                "commit_n_lm": commit["n_lm"],
                "start_n_obs": start["n_obs"],
                "commit_n_obs": commit["n_obs"],
                "kf_lost_count": len(kf_lost),
                "lm_lost_count": len(lm_lost),
                "kf_gained": len(ckf - skf),
                "lm_gained": len(clm - slm),
                "allocator_monotonic": alloc_ok,
                "start_next_id": start["next_landmark_id"],
                "commit_next_id": commit["next_landmark_id"],
                "note": "ok" if not kf_lost and alloc_ok else "investigate",
            }
        )

    # Extract timing
    timing_rows = []
    for site in sorted({r.get("site", "") for r in extracts}):
        sub = [r for r in extracts if r.get("site") == site]
        wait = [float(r["wait_ns"]) / 1e6 for r in sub]
        clone = [float(r["clone_ns"]) / 1e6 for r in sub]
        total = [w + c for w, c in zip(wait, clone)]
        n_lm = [float(r["n_lm"]) for r in sub]
        timing_rows.append(
            {
                "site": site,
                "n": len(sub),
                "wait_ms_med": pct(wait, 50),
                "wait_ms_p95": pct(wait, 95),
                "wait_ms_max": max(wait) if wait else float("nan"),
                "clone_ms_med": pct(clone, 50),
                "clone_ms_p95": pct(clone, 95),
                "clone_ms_max": max(clone) if clone else float("nan"),
                "total_ms_med": pct(total, 50),
                "total_ms_p95": pct(total, 95),
                "total_ms_max": max(total) if total else float("nan"),
                "n_lm_med": pct(n_lm, 50),
                "n_lm_max": max(n_lm) if n_lm else float("nan"),
            }
        )

    # ATE
    ate = {}
    ate_path = run_dir / "ate_summary.json"
    if not ate_path.is_file():
        # printed path from runner
        alt = run_dir / "ate_summary.json.print"
        if alt.is_file():
            try:
                ate = json.loads(alt.read_text())
            except Exception:
                ate = {"raw": alt.read_text()[:2000]}
    else:
        ate = json.loads(ate_path.read_text())

    # Trajectories (SVG)
    traj = run_dir / "trajectories.npz"
    if traj.is_file():
        data = np.load(traj)
        body = []
        colors = {"gt": "#222", "odom": "#1f77b4", "final_global_path": "#d62728"}
        labels = {"gt": "GT", "odom": "LocalSLAM", "final_global_path": "GlobalSLAM"}
        for key in ("gt", "odom", "final_global_path"):
            if key not in data.files:
                continue
            arr = data[key]
            if arr.ndim != 2 or arr.shape[1] < 2:
                continue
            body.append(
                _svg_polyline(
                    arr[:, 0],
                    arr[:, 1],
                    stroke=colors[key],
                    label=labels[key],
                )
            )
        write_svg(
            figures / "trajectories_xy.svg",
            "".join(body),
            title="M7.7 trajectories (XY)",
            w=720,
            h=720,
        )

    if pk and "n_anchors_after" in pk[0]:
        xs = list(range(len(pk)))
        ys = [float(r["n_anchors_after"]) for r in pk]
        write_svg(
            figures / "landmark_count_evolution.svg",
            _svg_polyline(xs, ys, label="n_anchors_after"),
            title="Landmark count evolution",
        )

    # KF processing latency as backlog proxy (no explicit buffer occupancy column)
    if pk and "update_time_s" in pk[0]:
        xs = list(range(len(pk)))
        ys = [float(r["update_time_s"]) for r in pk]
        write_svg(
            figures / "kf_buffer_occupancy.svg",
            _svg_polyline(xs, ys, label="update_time_s (proxy)"),
            title="GMAP update duration (backlog proxy)",
        )

    if extracts:
        # scatter-like: downsample polyline of sorted by n_lm
        clo = [r for r in extracts if r.get("site") == "cloop_detection"]
        if clo:
            order = sorted(range(len(clo)), key=lambda i: float(clo[i]["n_lm"]))
            xs = [float(clo[i]["n_lm"]) for i in order]
            ys = [
                (float(clo[i]["wait_ns"]) + float(clo[i]["clone_ns"])) / 1e6
                for i in order
            ]
            write_svg(
                figures / "detection_clone_vs_map_size.svg",
                _svg_polyline(xs, ys, label="cloop extract total ms"),
                title="Detection extract duration vs map size",
            )
            xs = list(range(len(clo)))
            wait = [float(r["wait_ns"]) / 1e6 for r in clo]
            clone = [float(r["clone_ns"]) / 1e6 for r in clo]
            write_svg(
                figures / "map_lock_wait_hold.svg",
                _svg_polyline(xs, wait, stroke="#ff7f0e", label="wait_ms")
                + _svg_polyline(xs, clone, stroke="#2ca02c", label="clone_ms"),
                title="Map-lock wait vs clone hold",
            )

    if mem:
        xs = list(range(len(mem)))
        ys = [float(r["rss_kb"]) / 1024.0 for r in mem]
        write_svg(
            figures / "process_memory.svg",
            _svg_polyline(xs, ys, label=f"RSS MiB pid={mem[0].get('pid','?')}"),
            title="Shared component-container RSS",
        )

    # Local XY error vs time from npz if present
    if traj.is_file():
        data = np.load(traj)
        if "gt" in data.files and "odom" in data.files:
            g = data["gt"]
            o = data["odom"]
            gx = np.interp(o[:, 0], g[:, 0], g[:, 1])
            gy = np.interp(o[:, 0], g[:, 0], g[:, 2])
            err = np.hypot(o[:, 1] - gx, o[:, 2] - gy)
            t0 = o[0, 0]
            write_svg(
                figures / "trajectory_error_vs_time.svg",
                _svg_polyline(o[:, 0] - t0, err, label="Local XY err"),
                title="Trajectory error vs time",
            )

    # Event timeline
    marks = []
    for i, c in enumerate(commits):
        marks.append(
            f'<line x1="{40 + i * 40}" y1="40" x2="{40 + i * 40}" y2="120" '
            f'stroke="#d62728" stroke-width="2"/>'
            f'<text x="{40 + i * 40}" y="140" font-size="10" '
            f'text-anchor="middle">p{c["proposal_id"]} kf{c["closing_kf"]}</text>'
        )
    if not marks:
        marks.append(
            '<text x="40" y="80" font-size="12">'
            "No authoritative commits in this run</text>"
        )
    write_svg(
        figures / "loop_sync_timeline.svg",
        "".join(marks),
        title="Loop correction / sync timeline",
        h=180,
    )
    write_svg(
        figures / "post_loop_observation_ba.svg",
        '<text x="40" y="80" font-size="12">'
        "Post-loop matching/BA not exercised (no commit)</text>",
        title="Post-loop observation insertion / BA",
        h=180,
    )

    # Tables
    write_csv(tables / "runtime_timing.csv", timing_rows)
    write_csv(tables / "authoritative_commits.csv", commits)
    write_csv(tables / "loop_proposals.csv", [
        {
            "proposal_id": p[0] if isinstance(p, tuple) else p,
            "closing": p[1] if isinstance(p, tuple) and len(p) > 1 else "",
            "matched": p[2] if isinstance(p, tuple) and len(p) > 2 else "",
        }
        for p in proposals_submitted
    ])
    write_csv(tables / "map_identity_events.csv", content_rows)
    write_csv(
        tables / "map_invariants.csv",
        [
            {
                "invariant_samples": len(inv_s),
                "invariant_violations": len(inv_v),
                "violation_detail_rows": len(inv_v),
            }
        ],
    )
    write_csv(tables / "memory_samples.csv", mem)
    write_csv(tables / "post_loop_refinement.csv", [
        {
            "proposal_id": c["proposal_id"],
            "ba_refine": c["ba_refine"],
            "reval": c["reval"],
        }
        for c in commits
    ])
    write_csv(tables / "sync_events.csv", [
        {"event": k, "count": v} for k, v in sync_lines.items()
    ])

    # Reject reasons
    reject_counter = Counter(r["reason"] for r in rejects)

    n_kf = len(pk)
    n_birth = len(births)
    n_del = len(deletes)
    retained = None
    if pk and "n_landmarks" in pk[-1]:
        retained = int(float(pk[-1]["n_landmarks"]))
    elif pk and "n_active_landmarks" in pk[-1]:
        retained = int(float(pk[-1]["n_active_landmarks"]))

    summary = {
        "run_id": meta.get("run_id", run_dir.name),
        "mission_done_marker": meta.get("mission_done_marker"),
        "git_head": meta.get("git_head"),
        "git_describe": meta.get("git_describe"),
        "loaded_matches_expected": meta.get("loaded_matches_expected"),
        "effective_param_verified": meta.get("effective_param_verified"),
        "feedback_enable_verified": meta.get("feedback_enable_verified"),
        "n_kf_admitted": n_kf,
        "n_landmark_births": n_birth,
        "n_landmark_deletes": n_del,
        "n_landmarks_retained_last_kf": retained,
        "invariant_samples": len(inv_s),
        "invariant_violations": len(inv_v),
        "proposals_submitted": len(proposals_submitted),
        "proposals_rejected_logged": len(rejects),
        "reject_reasons": dict(reject_counter),
        "authoritative_commits": len(commits),
        "commits": commits,
        "sync_event_counts": sync_lines,
        "extract_timing": timing_rows,
        "content_preservation": content_rows,
        "memory_n_samples": len(mem),
        "memory_rss_mib_med": pct(
            [float(r["rss_kb"]) / 1024.0 for r in mem], 50
        )
        if mem
        else None,
        "memory_rss_mib_max": max((float(r["rss_kb"]) / 1024.0 for r in mem), default=None),
        "ate": ate,
        "align_residuals_parsed": align_residuals,
        "runtime_events": len(events),
        "identity_snapshots": len(id_snaps),
    }
    (tables / "mission_metrics.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--evidence-dir", required=True)
    args = ap.parse_args()
    s = analyze(Path(args.run_dir), Path(args.evidence_dir))
    print(json.dumps({k: s[k] for k in s if k not in ("commits", "content_preservation", "extract_timing", "ate")}, indent=2))


if __name__ == "__main__":
    main()
