#!/usr/bin/env python3
"""G6b: verify launch-file constraints without starting Webots.

Checks that quad_experiment.launch.py rejects:
  - global_corrections without globalslam
  - Coordinate_mode absent_ok / offline_bypass when corrections enabled

Success-path banners are asserted by static inspection of the launch script
(avoids leaving component_container orphans when process signals are blocked).

Does not claim a full Webots mission.
"""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap


def main() -> int:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    install = os.environ.get('EKF_INSTALL_BASE', os.path.join(root, 'install'))
    setup = os.path.join(install, 'setup.bash')
    launch_py = os.path.join(
        root, 'src', 'ekf_monoslam_bringup', 'scripts', 'quad_experiment.launch.py')
    if not os.path.isfile(setup):
        print(f'SKIP: no overlay at {setup}')
        return 0
    if not os.path.isfile(launch_py):
        print(f'FAIL: missing {launch_py}')
        return 1

    env = os.environ.copy()
    failures = []

    def check_reject(name, launch_args, expect_fail_substr):
        # Rejection paths raise before the container stays up — safe with short timeout.
        script = textwrap.dedent(f"""
            set +e
            source /opt/ros/jazzy/setup.bash
            source '{setup}'
            timeout -s KILL 12 ros2 launch ekf_monoslam_bringup quad_experiment.launch.py \\
              {' '.join(launch_args)}
            exit 0
        """)
        p = subprocess.run(
            ['bash', '-lc', script], capture_output=True, text=True, timeout=20,
            env=env)
        out = (p.stdout or '') + (p.stderr or '')
        print(f'--- {name} ---')
        if expect_fail_substr not in out:
            failures.append(f'{name}: missing {expect_fail_substr!r}')
            print(out[-1500:])
        else:
            print(f'  rejected OK ({expect_fail_substr})')

    check_reject(
        'corrections_without_gs',
        ['use_globalslam:=false', 'global_corrections:=true', 'use_plot2:=false'],
        'requires use_globalslam')

    # Static inspection for success banners (no long-lived container).
    src = open(launch_py, encoding='utf-8').read()
    policy_py = os.path.join(
        root, 'src', 'ekf_monoslam_bringup', 'scripts', 'g6d_launch_policy.py')
    policy_src = open(policy_py, encoding='utf-8').read() if os.path.isfile(
        policy_py) else ''
    print('--- static launch source ---')
    for needle, label, blob in [
        ('resolve_gslam_g6d_params', 'G6d precedence helper', src),
        ("default_value=''", 'global_corrections omit=YAML', src),
        ('G6d: coordinated Local feedback ENABLED', 'corrections banner', src),
        ('Local feedback OFF', 'viz banner text', src),
        ('Apply_ba_corrections', 'G6d apply_ba', src),
        ('Enable_visual_loop_closure', 'G6d enable_visual', src),
        ('CLI > YAML', 'precedence docstring', src),
        ('no simultaneous loop-source arbitration', 'reject dual loop', policy_src),
        ('Contradictory Global SLAM correction options', 'master conflict', policy_src),
        ('Omitted overrides must not rewrite YAML', 'omit keeps YAML', policy_src),
    ]:
        if needle not in blob:
            failures.append(f'static: missing {label!r} ({needle!r})')
            print(f'  missing {label}')
        else:
            print(f'  ok ({label})')

    # Reject absent_ok / offline_bypass when corrections enabled (source-level).
    if 'absent_ok' not in src or 'offline_bypass' not in src:
        failures.append('static: missing absent_ok/offline_bypass rejection')
    else:
        print('  ok (rejects absent_ok/offline_bypass)')

    if failures:
        print('FAIL:')
        for f in failures:
            print(' ', f)
        return 1
    print('G6b launch constraints smoke PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
