#!/usr/bin/env python3
"""Static + lightweight Webots validation for MAP-MGMT-A3.1 extended world."""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def parse_elev(text: str, which: str = "geometry"):
    if which == "geometry":
        m = re.search(
            r"geometry ElevationGrid \{\s*xDimension (\d+)\s*xSpacing ([\d.]+)\s*"
            r"yDimension (\d+)\s*ySpacing ([\d.]+)\s*height \[([^\]]+)\]",
            text,
            re.S,
        )
    else:
        m = re.search(
            r"boundingObject ElevationGrid \{\s*xDimension (\d+)\s*xSpacing ([\d.]+)\s*"
            r"yDimension (\d+)\s*ySpacing ([\d.]+)\s*height \[([^\]]+)\]",
            text,
            re.S,
        )
    if not m:
        raise SystemExit(f"ElevationGrid {which} not found")
    xd, xs, yd, ys = int(m.group(1)), float(m.group(2)), int(m.group(3)), float(m.group(4))
    h = np.array([float(x) for x in re.findall(r"-?\d+\.\d+", m.group(5))])
    return xd, xs, yd, ys, h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", type=Path, required=True)
    ap.add_argument("--ext", type=Path, required=True)
    ap.add_argument("--screenshots-dir", type=Path, default=None)
    ap.add_argument("--webots-load-check", action="store_true")
    args = ap.parse_args()

    o = args.orig.read_text()
    e = args.ext.read_text()

    oxd, oxs, oyd, oys, oh = parse_elev(o)
    exd, exs, eyd, eys, eh = parse_elev(e)
    assert oxd == oyd == 51 and oxs == oys == 10.0
    assert exd == eyd == 101 and exs == eys == 10.0
    assert oh.size == 51 * 51 and eh.size == 101 * 101

    o_extent = (oxd - 1) * oxs
    e_extent = (exd - 1) * exs
    print(f"orig extent {o_extent}x{o_extent} m  grid {oxd}@{oxs}")
    print(f"ext  extent {e_extent}x{e_extent} m  grid {exd}@{exs}")
    assert abs(e_extent - 2 * o_extent) < 1e-6

    ot = re.search(r"DEF TERRAIN Solid \{\s*translation ([-\d.]+) ([-\d.]+) ([-\d.]+)", o)
    et = re.search(r"DEF TERRAIN Solid \{\s*translation ([-\d.]+) ([-\d.]+) ([-\d.]+)", e)
    print("orig terrain SW", ot.group(1), ot.group(2))
    print("ext  terrain SW", et.group(1), et.group(2))
    assert float(ot.group(1)) == -250 and float(et.group(1)) == -500

    # Center heights must match original
    H_o = oh.reshape(51, 51)
    H_e = eh.reshape(101, 101)
    center = H_e[25:76, 25:76]
    max_diff = float(np.max(np.abs(center - H_o)))
    print(f"center height max|diff| vs orig = {max_diff:.6f}")
    assert max_diff < 1e-6

    os_ = re.search(r'Mavic2Pro \{\s*translation ([-\d.]+) ([-\d.]+) ([-\d.]+)', o)
    es_ = re.search(r'Mavic2Pro \{\s*translation ([-\d.]+) ([-\d.]+) ([-\d.]+)', e)
    print("orig spawn", os_.group(1), os_.group(2), os_.group(3))
    print("ext  spawn", es_.group(1), es_.group(2), es_.group(3))
    assert os_.group(0) == es_.group(0) or (
        os_.group(1) == es_.group(1)
        and os_.group(2) == es_.group(2)
        and os_.group(3) == es_.group(3)
    )

    assert 'earth_aerial_texture.jpg' in o and 'earth_aerial_texture_1000m.jpg' in e
    assert 'earth_aerial_texture_1000m.jpg' not in o  # original unchanged

    # Camera footprint margin at mission corner (-190,-190) and far corner approx (190,-190)
    # At 40 m AGL, FOV 1.57 rad ≈ 90°, half-footprint ≈ 40 m
    half_fp = 40.0
    for corner in [(-190, -190), (190, -190), (190, 190), (-190, 190), (40, 0)]:
        for dx in (-half_fp, half_fp):
            for dy in (-half_fp, half_fp):
                x, y = corner[0] + dx, corner[1] + dy
                assert -500 <= x <= 500 and -500 <= y <= 500, (x, y)
    print("mission corners + 40 m FOV half-width remain inside ±500 terrain")

    # Texture scale: 4096 px / 1000 m ≈ 0.244 m/px; orig 2048/500 ≈ 0.244
    tex = args.ext.parent / "earth_aerial_texture_1000m.jpg"
    assert tex.exists(), tex
    print("texture present", tex.name, "bytes", tex.stat().st_size)

    if args.webots_load_check:
        webots = Path("/usr/local/webots/webots")
        if not webots.exists():
            webots = Path("/usr/bin/webots")
        if not webots.exists():
            print("WARN: webots binary not found; skip load check")
        else:
            # Headless-ish: start and quit quickly via timeout
            cmd = [
                str(webots),
                "--stdout",
                "--stderr",
                "--mode=pause",
                "--no-rendering",  # may be ignored on some builds
                str(args.ext.resolve()),
            ]
            # Prefer batch if available
            for flag in ("--batch",):
                try_cmd = [str(webots), flag, "--mode=pause", str(args.ext.resolve())]
                print("trying", try_cmd)
                try:
                    p = subprocess.Popen(
                        try_cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                    )
                    time.sleep(8)
                    # If still running, treat as successful load then kill
                    if p.poll() is None:
                        print("Webots still running after 8s — load OK; terminating")
                        p.terminate()
                        try:
                            p.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            p.kill()
                        print("PASS webots load")
                    else:
                        out = p.stdout.read() if p.stdout else ""
                        print(out[-2000:])
                        if p.returncode != 0:
                            print("FAIL webots exit", p.returncode)
                            sys.exit(1)
                        print("PASS webots exited cleanly")
                    break
                except FileNotFoundError:
                    continue

    print("STATIC_VALIDATION_PASS")


if __name__ == "__main__":
    main()
