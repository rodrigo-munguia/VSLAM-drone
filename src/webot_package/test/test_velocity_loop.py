# CTRL-S3: deterministic velocity → accel and precedence checks.
import math

from webot_package.webot_robot_driver import (
    K_ACCEL_XY_MAX,
    K_DIAG_TIMEOUT,
    K_VEL_P,
    body_accel_to_enu,
    enu_accel_to_body,
    next_att_source,
    velocity_to_accel,
)


def test_zero_velocity_error_zero_accel():
    ax, ay = velocity_to_accel(1.0, -0.5, 1.0, -0.5)
    assert abs(ax) < 1e-12 and abs(ay) < 1e-12


def test_velocity_error_signs():
    ax, ay = velocity_to_accel(1.0, 0.0, 0.0, 0.0)
    assert ax > 0.0 and abs(ay) < 1e-12
    ax, ay = velocity_to_accel(-1.0, 0.0, 0.0, 0.0)
    assert ax < 0.0 and abs(ay) < 1e-12
    ax, ay = velocity_to_accel(0.0, 1.0, 0.0, 0.0)
    assert ay > 0.0 and abs(ax) < 1e-12
    ax, ay = velocity_to_accel(0.0, -1.0, 0.0, 0.0)
    assert ay < 0.0 and abs(ax) < 1e-12


def test_yaw_body_enu_roundtrip():
    yaw = 0.7
    a_bx, a_by = 1.2, -0.4
    a_e, a_n = body_accel_to_enu(a_bx, a_by, yaw)
    bx2, by2 = enu_accel_to_body(a_e, a_n, yaw)
    assert abs(bx2 - a_bx) < 1e-12 and abs(by2 - a_by) < 1e-12
    # yaw=0: body ≡ ENU
    ae, an = body_accel_to_enu(0.5, -0.3, 0.0)
    assert abs(ae - 0.5) < 1e-12 and abs(an + 0.3) < 1e-12


def test_accel_saturation():
    ax, ay = velocity_to_accel(10.0, 0.0, 0.0, 0.0, kv=5.0)
    assert abs(math.hypot(ax, ay) - K_ACCEL_XY_MAX) < 1e-12
    ax, ay = velocity_to_accel(10.0, 10.0, 0.0, 0.0, kv=5.0)
    assert abs(math.hypot(ax, ay) - K_ACCEL_XY_MAX) < 1e-9


def test_nonfinite_commands():
    ax, ay = velocity_to_accel(float('nan'), 1.0, 0.0, 0.0)
    assert abs(ax) < 1e-12 and abs(ay - K_VEL_P) < 1e-12
    ax, ay = velocity_to_accel(0.0, float('inf'), 0.0, 0.0)
    assert abs(ax) < 1e-12 and abs(ay) < 1e-12
    ax, ay = velocity_to_accel(1.0, 0.0, float('nan'), 0.0)
    assert abs(ax - K_VEL_P) < 1e-12


def test_zero_command_braking():
    # v_d=0, v≠0 → opposing accel (brake), not zero / not level-only.
    ax, ay = velocity_to_accel(0.0, 0.0, 0.4, -0.2)
    assert ax < 0.0 and ay > 0.0
    assert abs(ax - K_VEL_P * (0.0 - 0.4)) < 1e-12
    assert abs(ay - K_VEL_P * (0.0 - (-0.2))) < 1e-12
    # Saturated brake still opposes motion.
    ax, ay = velocity_to_accel(0.0, 0.0, 5.0, 0.0)
    assert ax < 0.0 and abs(ax + K_ACCEL_XY_MAX) < 1e-12


def test_command_source_precedence_timeout():
    assert next_att_source('velocity', 0.0, 100.0) == 'velocity'
    assert next_att_source('attitude', 10.0, 10.0 + K_DIAG_TIMEOUT - 0.01) == 'attitude'
    assert next_att_source('attitude', 10.0, 10.0 + K_DIAG_TIMEOUT + 0.01) == 'velocity'
    assert next_att_source('accel', 10.0, 10.0 + K_DIAG_TIMEOUT + 0.01) == 'velocity'
