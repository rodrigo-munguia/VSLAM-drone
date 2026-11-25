#!/usr/bin/env python3
"""Validate G6d launch precedence: CLI > YAML > node default.

Does not start Webots or enable a live mission. Uses a temporary YAML with
non-default values, then applies explicit CLI-style overrides.
"""

from __future__ import annotations

import os
import sys
import tempfile
import textwrap

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
SCRIPTS = os.path.join(
    ROOT, 'src', 'ekf_monoslam_bringup', 'scripts')
sys.path.insert(0, SCRIPTS)

from g6d_launch_policy import (  # noqa: E402
    resolve_gslam_g6d_params,
    resolve_slam_feedback,
)


def expect(cond: bool, msg: str, failures: list) -> None:
    if not cond:
        failures.append(msg)
        print(f'FAIL: {msg}')
    else:
        print(f'OK: {msg}')


def main() -> int:
    failures: list[str] = []

    # Non-default YAML fixture (differs from shipped + node defaults).
    yaml_gslam = {
        'Enable_global_ba': False,
        'Enable_tag_loop_closure': True,
        'Enable_visual_loop_closure': False,
        'Apply_ba_corrections': False,
        'Apply_tag_loop_corrections': True,  # shipped YAML is false
        'Apply_visual_loop_corrections': False,
        'Apply_global_corrections': False,
        'Coordinate_mode': 'required',
    }
    yaml_control = {
        'Slam_feedback': True,  # shipped is false
        'Slam_feedback_interface': 'legacy',  # shipped is odometry
    }

    # 1) Omitted CLI → YAML wins (must not stomp Apply_tag / Slam_feedback).
    r = resolve_gslam_g6d_params(yaml_gslam)
    expect(r['Enable_global_ba'] is False, 'omitted enable_ba keeps YAML false', failures)
    expect(r['Apply_tag_loop_corrections'] is True,
           'omitted apply_tag keeps YAML true (not hard-forced false)', failures)
    expect(r['_needs_coordination'] is False,
           'A4.1: Local feedback does not require CoordPause', failures)
    expect(r.get('_needs_local_feedback') is True,
           'YAML Apply_tag implies local feedback', failures)
    expect(r['Coordinate_mode'] == 'offline_bypass',
           'A4.1: feedback forces Coordinate_mode=offline_bypass', failures)
    c = resolve_slam_feedback(yaml_control)
    expect(c['Slam_feedback'] is True, 'omitted slam_feedback keeps YAML true', failures)
    expect(c['Slam_feedback_interface'] == 'legacy',
           'omitted slam_feedback_interface keeps YAML legacy', failures)

    # 2) Explicit CLI overrides YAML.
    r2 = resolve_gslam_g6d_params(
        yaml_gslam,
        enable_ba=True,
        apply_ba=False,
        apply_tag=False,
    )
    expect(r2['Enable_global_ba'] is True, 'CLI enable_ba overrides YAML false', failures)
    expect(r2['Apply_tag_loop_corrections'] is False,
           'CLI apply_tag false overrides YAML true', failures)
    expect(r2['_needs_coordination'] is False,
           'all Apply false → no coordination', failures)
    c2 = resolve_slam_feedback(
        yaml_control, slam_feedback='false', slam_feedback_interface='odometry')
    expect(c2['Slam_feedback'] is False, 'CLI slam_feedback overrides YAML', failures)
    expect(c2['Slam_feedback_interface'] == 'odometry',
           'CLI slam_feedback_interface overrides YAML', failures)

    # 3) Missing YAML keys → node defaults.
    r3 = resolve_gslam_g6d_params({})
    expect(r3['Enable_global_ba'] is True, 'node default Enable_global_ba true', failures)
    expect(r3['Enable_visual_loop_closure'] is False,
           'node default Enable_visual_loop_closure false', failures)
    expect(r3['Apply_ba_corrections'] is False,
           'node default Apply_ba_corrections false', failures)
    c3 = resolve_slam_feedback({})
    expect(c3['Slam_feedback'] is False, 'node default Slam_feedback false', failures)

    # 4) Master true forces Apply for enabled; conflicting apply false fails.
    r4 = resolve_gslam_g6d_params(
        {'Enable_global_ba': True, 'Enable_tag_loop_closure': True,
         'Enable_visual_loop_closure': False,
         'Apply_ba_corrections': False, 'Apply_tag_loop_corrections': False},
        global_corrections=True,
    )
    expect(r4['Apply_ba_corrections'] is True, 'master forces Apply_ba', failures)
    expect(r4['Apply_tag_loop_corrections'] is True, 'master forces Apply_tag', failures)
    expect(r4['Apply_visual_loop_corrections'] is False,
           'master does not enable visual Apply without visual Enable', failures)
    try:
        resolve_gslam_g6d_params(
            {'Enable_global_ba': True, 'Enable_tag_loop_closure': True,
             'Enable_visual_loop_closure': False},
            global_corrections=True,
            apply_tag=False,
        )
        expect(False, 'master + apply_tag false must raise', failures)
    except RuntimeError as ex:
        expect('conflicts with' in str(ex) or 'Contradictory' in str(ex),
               f'master conflict raises clearly ({ex})', failures)

    # 5) Master false clears master flag only; YAML per-source Apply kept.
    r5 = resolve_gslam_g6d_params(
        {'Apply_tag_loop_corrections': True, 'Enable_tag_loop_closure': True,
         'Apply_global_corrections': True, 'Enable_global_ba': True},
        global_corrections=False,
    )
    expect(r5['Apply_global_corrections'] is False,
           'explicit master false clears Apply_global_corrections', failures)
    expect(r5['Apply_tag_loop_corrections'] is True,
           'master false does not wipe YAML Apply_tag', failures)

    # 6) Dual loop rejected.
    try:
        resolve_gslam_g6d_params(
            {}, enable_tag=True, enable_vis=True)
        expect(False, 'dual loop must raise', failures)
    except RuntimeError as ex:
        expect('both be true' in str(ex), f'dual loop error ({ex})', failures)

    # Write a temp YAML snapshot for operator evidence (not installed).
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'precedence_fixture.yaml')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(textwrap.dedent("""\
                /gslam:
                  ros__parameters:
                    Enable_global_ba: false
                    Apply_tag_loop_corrections: true
                /control:
                  ros__parameters:
                    Slam_feedback: true
                    Slam_feedback_interface: legacy
                """))
        expect(os.path.isfile(path), f'temp fixture written {path}', failures)

    if failures:
        print('G6d precedence FAIL:')
        for x in failures:
            print(' ', x)
        return 1
    print('G6d precedence smoke PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
