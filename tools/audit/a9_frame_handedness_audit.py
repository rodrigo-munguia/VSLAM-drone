#!/usr/bin/env python3
"""Temporary diagnostic: Plot2 XY projection viewing direction.

After the from-above fix:
  sx = cx + x_pub * s
  sy = cy - y_pub * s
  => right=+X, up=+Y, look=-Z

Run: python3 tools/a9_frame_handedness_audit.py
"""
from __future__ import annotations

import numpy as np


def public_xy_to_screen(x: float, y: float, cx: float = 0.0, cy: float = 0.0, s: float = 1.0):
    return cx + x * s, cy - y * s


def main() -> None:
    print("=== Plot2 top XY (from-above convention) ===")
    print("sx = cx + x_pub * s")
    print("sy = cy - y_pub * s")
    right = np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 1.0, 0.0])
    look = -np.cross(right, up)
    print(f"right × up = {np.cross(right, up)}")
    print(f"look = -(right × up) = {look}")
    assert abs(look[2] + 1.0) < 1e-12
    print("VERDICT: viewing direction = -Z (from above)")
    M = np.array([[1.0, 0.0], [0.0, 1.0]])
    print(f"map (x,y)->(sr,su) det = {np.linalg.det(M):.1f}")

    print("\n=== Asymmetric labelled reference ===")
    for name, p in [("O", (0, 0)), ("+X", (1, 0)), ("+Y", (0, 1)), ("off (1,2)", (1, 2))]:
        print(f"  {name} {p} -> {public_xy_to_screen(*p)}")


if __name__ == "__main__":
    main()
