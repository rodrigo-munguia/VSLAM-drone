#!/usr/bin/env python3
"""GSLAM-VISUAL-LOOP-A3f — static mission / ENU / 40 m revisit checks."""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "control_plan_a3f_rect380_revisit40.txt"
ORIG = ROOT / "control_plan_a3d0_rect380.txt"
SPAWN = (-190.0, -190.0, 15.75)


def mission_to_gps(mx: float, my: float, mz: float):
    sx, sy, sz = SPAWN
    return (sx + mx, sy - my, sz + mz)


def parse_pointas(path: Path):
    pts = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line.startswith("pointa,"):
            continue
        parts = line.split(",")
        pts.append(tuple(float(x) for x in parts[1:5]))
    return pts


def main() -> int:
    errs: list[str] = []
    if not PLAN.is_file():
        print(f"FAIL: missing {PLAN}")
        return 1
    if not ORIG.is_file():
        print(f"FAIL: missing original {ORIG}")
        return 1

    pts = parse_pointas(PLAN)
    expect = [
        (0, 0, 60, 0),
        (380, 0, 60, 0),
        (380, -380, 60, 0),
        (0, -380, 60, 0),
        (0, 0, 60, 0),
        (40, 0, 60, 0),
    ]
    if pts != expect:
        errs.append(f"waypoints mismatch: got {pts} expect {expect}")

    text = PLAN.read_text()
    for forbidden in ("RecHome", "GoHome", "ActivateCloseLoop", "diagonal"):
        if forbidden in text:
            errs.append(f"forbidden token {forbidden!r} in plan")

    orig_pts = parse_pointas(ORIG)
    if pts[:5] != orig_pts:
        errs.append("first five pointa must match original rect380 plan")

    # Axis-aligned segments only; final hop exactly 40 m.
    enu = [mission_to_gps(p[0], p[1], p[2]) for p in pts]
    for i in range(1, len(enu)):
        dx = enu[i][0] - enu[i - 1][0]
        dy = enu[i][1] - enu[i - 1][1]
        if abs(dx) > 1e-9 and abs(dy) > 1e-9:
            errs.append(f"diagonal segment {i-1}->{i}: d=({dx},{dy})")
    last_h = math.hypot(enu[-1][0] - enu[-2][0], enu[-1][1] - enu[-2][1])
    if abs(last_h - 40.0) > 1e-9:
        errs.append(f"final segment {last_h} m != 40 m")

    final = enu[-1]
    if abs(final[0] + 150) > 1e-9 or abs(final[1] + 190) > 1e-9:
        errs.append(f"final ENU {final[:2]} != (-150,-190)")

    # Corridor overlap: revisit x ∈ [-190,-150], y=-190 matches first 40 m of leg1.
    first40_end = mission_to_gps(40, 0, 60)
    if first40_end[:2] != (-150.0, -190.0):
        errs.append("first-40 corridor end mismatch")

    # Terrain footprint (ElevationGrid ±250).
    for name, p in [("home", enu[0]), ("revisit_end", enu[-1]), ("NE", enu[2])]:
        if abs(p[0]) > 250 or abs(p[1]) > 250:
            errs.append(f"{name} ENU {p[:2]} outside ±250 terrain")

    if errs:
        print("FAIL:")
        for e in errs:
            print(" ", e)
        return 1

    print("PASS a3f mission static checks")
    print(f"  plan={PLAN.name}")
    print(f"  spawn={SPAWN}")
    print(f"  final_ENU=({final[0]:.1f},{final[1]:.1f},{final[2]:.2f})")
    print(f"  revisit_horiz_m={last_h:.1f}")
    print("  corridor=first 40 m of leg1 (y=-190, x -190→-150)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
