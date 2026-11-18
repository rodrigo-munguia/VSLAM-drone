# CTRL-R4b: deterministic tests for unified ENU position/velocity cascade.
import math

import pytest

from webot_package.unified_position_control import (
    A_MAX_XY,
    A_MAX_Z,
    G_ACCEL,
    HOVER_OMEGA,
    K_POS_XY,
    K_POS_Z,
    K_VEL_XY,
    THRUST_COEFF,
    V_MAX_XY,
    V_MAX_Z,
    VEHICLE_MASS,
    desired_force,
    force_to_collective_omega,
    hover_force,
    position_error,
    position_to_velocity_sp,
    unified_step,
    velocity_to_accel_sp,
)


def test_zero_error_zero_commands():
    e = position_error((1, 2, 3), (1, 2, 3))
    assert e == (0.0, 0.0, 0.0)
    v_sp = position_to_velocity_sp(e)
    assert v_sp == (0.0, 0.0, 0.0)
    a_sp = velocity_to_accel_sp(v_sp, (0, 0, 0))
    assert a_sp == (0.0, 0.0, 0.0)
    f = desired_force(a_sp)
    assert abs(f[0]) < 1e-12 and abs(f[1]) < 1e-12
    assert abs(f[2] - VEHICLE_MASS * G_ACCEL) < 1e-9


def test_enu_axis_signs():
    e = position_error((2, 0, 0), (0, 0, 0))
    v = position_to_velocity_sp(e)
    assert v[0] > 0 and abs(v[1]) < 1e-12
    e = position_error((0, -3, 0), (0, 0, 0))
    v = position_to_velocity_sp(e)
    assert v[1] < 0 and abs(v[0]) < 1e-12
    e = position_error((0, 0, 5), (0, 0, 0))
    v = position_to_velocity_sp(e)
    assert v[2] > 0


def test_position_to_velocity_gain():
    e = (2.0, 0.0, 0.0)
    v = position_to_velocity_sp(e)
    assert abs(v[0] - K_POS_XY * 2.0) < 1e-12


def test_velocity_to_accel_gain():
    a = velocity_to_accel_sp((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert abs(a[0] - K_VEL_XY * 1.0) < 1e-12


def test_horizontal_velocity_saturation():
    e = (100.0, 0.0, 0.0)
    v = position_to_velocity_sp(e)
    assert abs(v[0]) == pytest.approx(V_MAX_XY)
    e = (100.0, 100.0, 0.0)
    v = position_to_velocity_sp(e)
    assert math.hypot(v[0], v[1]) == pytest.approx(V_MAX_XY)


def test_vertical_velocity_saturation():
    e = (0.0, 0.0, 100.0)
    v = position_to_velocity_sp(e)
    assert abs(v[2]) == pytest.approx(V_MAX_Z)


def test_accel_saturation():
    a = velocity_to_accel_sp((50.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert abs(a[0]) == pytest.approx(A_MAX_XY)
    a = velocity_to_accel_sp((0.0, 0.0, 50.0), (0.0, 0.0, 0.0))
    assert abs(a[2]) == pytest.approx(A_MAX_Z)


def test_gravity_hover_thrust():
    f = hover_force()
    assert abs(f[2] - VEHICLE_MASS * G_ACCEL) < 1e-12
    omega, thrust, sat = force_to_collective_omega(f, 0.0, 0.0)
    assert not sat
    assert abs(thrust - f[2]) < 1e-6
    # ω ≈ sqrt(T/(4c)) near hover command
    expect = math.sqrt(f[2] / (4.0 * THRUST_COEFF))
    assert abs(omega - expect) < 1e-6
    assert abs(omega - HOVER_OMEGA) < 2.0  # model mass vs 68.5 calibration


def test_tilt_increases_collective():
    f = hover_force()
    w0, _, _ = force_to_collective_omega(f, 0.0, 0.0)
    w_tilt, _, _ = force_to_collective_omega(f, 0.0, math.radians(10.0))
    assert w_tilt > w0


def test_nonfinite_safe():
    assert position_error((1, 2, float('nan')), (0, 0, 0)) == (0.0, 0.0, 0.0)
    assert position_to_velocity_sp((float('inf'), 0, 0)) == (0.0, 0.0, 0.0)
    assert velocity_to_accel_sp((0, 0, 0), (float('nan'), 0, 0)) == (0.0, 0.0, 0.0)
    w, _, _ = force_to_collective_omega((0, 0, float('nan')), 0.0, 0.0)
    assert math.isfinite(w) and w > 0


def test_unified_step_hold():
    p = (1.0, 2.0, 10.0)
    v_sp, a_sp, f_d, omega, sat = unified_step(p, p, (0, 0, 0), 0.0, 0.0)
    assert v_sp == (0.0, 0.0, 0.0)
    assert a_sp == (0.0, 0.0, 0.0)
    assert abs(f_d[2] - VEHICLE_MASS * G_ACCEL) < 1e-9
    assert math.isfinite(omega) and not sat


def test_unified_step_climb_command():
    p_d = (0.0, 0.0, 12.0)
    p = (0.0, 0.0, 10.0)
    v_sp, a_sp, f_d, omega, _sat = unified_step(p_d, p, (0, 0, 0), 0.0, 0.0)
    assert v_sp[2] > 0
    assert a_sp[2] > 0
    assert f_d[2] > VEHICLE_MASS * G_ACCEL
    assert omega > HOVER_OMEGA - 5.0
