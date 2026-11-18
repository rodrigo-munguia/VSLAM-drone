# CTRL-S2: deterministic accel→attitude and collective compensation checks.
import math

from webot_package.webot_robot_driver import (
    G_ACCEL,
    K_ACCEL_XY_MAX,
    K_TILT_MAX,
    accel_to_attitude,
    collective_tilt_compensation,
    enu_accel_to_body,
)


def test_zero_accel_level():
    r, p = accel_to_attitude(0.0, 0.0, 0.0)
    assert abs(r) < 1e-12 and abs(p) < 1e-12
    r, p = accel_to_attitude(0.0, 0.0, 1.2)
    assert abs(r) < 1e-12 and abs(p) < 1e-12


def test_east_north_signs_yaw0():
    # A3-verified: +a_E → +pitch; +a_N → −roll (yaw=0, body≡ENU).
    r, p = accel_to_attitude(1.0, 0.0, 0.0)
    assert p > 0.0 and abs(r) < 1e-9
    r, p = accel_to_attitude(-1.0, 0.0, 0.0)
    assert p < 0.0 and abs(r) < 1e-9
    r, p = accel_to_attitude(0.0, 1.0, 0.0)
    assert r < 0.0 and abs(p) < 1e-9
    r, p = accel_to_attitude(0.0, -1.0, 0.0)
    assert r > 0.0 and abs(p) < 1e-9


def test_yaw_rotates_mapping():
    # Facing +90° (north): +a_E is body −Y → +roll; +a_N is body +X → +pitch.
    r, p = accel_to_attitude(1.0, 0.0, math.pi / 2)
    assert r > 0.0 and abs(p) < 1e-6
    r, p = accel_to_attitude(0.0, 1.0, math.pi / 2)
    assert p > 0.0 and abs(r) < 1e-6


def test_small_angle_magnitude():
    a = 0.8
    r, p = accel_to_attitude(a, 0.0, 0.0)
    assert abs(p - math.atan2(a, G_ACCEL)) < 1e-12
    assert abs(r) < 1e-12


def test_accel_and_tilt_limits():
    r, p = accel_to_attitude(100.0, 0.0, 0.0)
    a_bx, _ = enu_accel_to_body(K_ACCEL_XY_MAX, 0.0, 0.0)
    a_cap = min(a_bx, G_ACCEL * math.tan(K_TILT_MAX))
    assert abs(p - math.atan2(a_cap, G_ACCEL)) < 1e-9
    alpha = math.acos(math.cos(r) * math.cos(p))
    assert alpha <= K_TILT_MAX + 1e-9
    # Combined: large equal components must not exceed tilt_max.
    r, p = accel_to_attitude(20.0, 20.0, 0.0)
    alpha = math.acos(max(-1.0, min(1.0, math.cos(r) * math.cos(p))))
    assert alpha <= K_TILT_MAX + 1e-6


def test_nonfinite_safe():
    assert accel_to_attitude(float('nan'), 0.0, 0.0) == (0.0, 0.0)
    assert accel_to_attitude(0.0, float('inf'), 0.0) == (0.0, 0.0)
    assert accel_to_attitude(1.0, 0.0, float('nan')) == (0.0, 0.0)


def test_collective_compensation():
    assert abs(collective_tilt_compensation(0.0, 0.0) - 1.0) < 1e-12
    # At 12° pitch-only: cosα=cos(12°), factor = 1/sqrt(cosα) > 1
    th = math.radians(12.0)
    f = collective_tilt_compensation(0.0, th)
    expect = 1.0 / math.sqrt(math.cos(th))
    assert abs(f - expect) < 1e-12
    assert f > 1.0
    # Bounded: absurd angles still use tilt_max floor (no blow-up).
    f_big = collective_tilt_compensation(1.5, 1.5)
    f_floor = 1.0 / math.sqrt(math.cos(K_TILT_MAX))
    assert abs(f_big - f_floor) < 1e-12
    assert math.isfinite(collective_tilt_compensation(float('nan'), 0.0))
