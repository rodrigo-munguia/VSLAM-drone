# CTRL-R4c: trajectory feedforward + lifecycle helpers (offline).
import math

from webot_package.jerk_limited_trajectory import JerkLimitedTrajectory
from webot_package.unified_position_control import (
    K_POS_XY,
    K_VEL_XY,
    TRAJ_A_MAX,
    TRAJ_J_MAX,
    TRAJ_STILL_SPEED,
    TRAJ_V_MAX_XY,
    measured_speed,
    position_error,
    position_to_velocity_sp,
    unified_step,
    velocity_to_accel_sp,
)


def test_hold_unchanged_with_zero_ff():
    p = (1.0, 2.0, 10.0)
    v_sp, a_sp, f_d, omega, sat = unified_step(p, p, (0, 0, 0), 0.0, 0.0)
    assert v_sp == (0.0, 0.0, 0.0)
    assert a_sp == (0.0, 0.0, 0.0)
    assert math.isfinite(omega) and not sat


def test_velocity_ff_added_once():
    e = (1.0, 0.0, 0.0)
    v_d = (0.5, 0.0, 0.0)
    v_sp = position_to_velocity_sp(e, v_d=v_d)
    assert abs(v_sp[0] - (0.5 + K_POS_XY * 1.0)) < 1e-12


def test_accel_ff_added_once():
    v_sp = (1.0, 0.0, 0.0)
    v = (0.0, 0.0, 0.0)
    a_d = (0.3, 0.0, 0.0)
    a_sp = velocity_to_accel_sp(v_sp, v, a_d=a_d)
    assert abs(a_sp[0] - (0.3 + K_VEL_XY * 1.0)) < 1e-12


def test_no_double_ff_in_unified_step():
    # On perfect track: e=0, v=v_d → a_sp should equal a_d (not 2*a_d)
    p_d = (0.0, 0.0, 10.0)
    p = (0.0, 0.0, 10.0)
    v_d = (1.0, 0.0, 0.0)
    a_d = (0.4, 0.0, 0.0)
    v = v_d
    _v_sp, a_sp, _f, _w, _s = unified_step(
        p_d, p, v, 0.0, 0.0, v_d=v_d, a_d=a_d
    )
    assert abs(a_sp[0] - a_d[0]) < 1e-12
    assert abs(a_sp[1]) < 1e-12


def test_traj_sample_to_terminal_hold():
    traj = JerkLimitedTrajectory(
        (0, 0, 10), (5, 0, 10),
        v_max=TRAJ_V_MAX_XY, a_max=TRAJ_A_MAX, j_max=TRAJ_J_MAX,
    )
    mid = traj.sample(0.5 * traj.duration)
    assert mid.velocity[0] > 0.0
    end = traj.sample(traj.duration)
    assert end.complete
    assert abs(end.position[0] - 5.0) < 1e-9
    assert end.velocity == (0.0, 0.0, 0.0)
    assert end.acceleration == (0.0, 0.0, 0.0)
    # After complete, tracker uses pf with zero FF
    _vs, a_sp, *_ = unified_step(
        end.position, end.position, (0, 0, 0), 0.0, 0.0,
        v_d=(0, 0, 0), a_d=(0, 0, 0),
    )
    assert a_sp == (0.0, 0.0, 0.0)


def test_still_speed_gate():
    assert measured_speed((0.1, 0.0, 0.0)) < TRAJ_STILL_SPEED
    assert measured_speed((0.3, 0.0, 0.0)) > TRAJ_STILL_SPEED


def test_traj_limits_within_controller():
    # R5a traj axis limits must not exceed tracker sats
    from webot_package.unified_position_control import (
        A_MAX_XY, TRAJ_V_MAX_Z, V_MAX_XY, V_MAX_Z,
    )
    assert TRAJ_V_MAX_XY <= V_MAX_XY + 1e-12
    assert TRAJ_V_MAX_Z <= V_MAX_Z + 1e-12
    assert TRAJ_A_MAX <= A_MAX_XY + 1e-12


def test_init_from_measured_position():
    p0 = (1.5, -2.0, 12.0)
    pf = (1.5 + 5.0, -2.0, 12.0)
    traj = JerkLimitedTrajectory(
        p0, pf, v_max=TRAJ_V_MAX_XY, a_max=TRAJ_A_MAX, j_max=TRAJ_J_MAX
    )
    s0 = traj.sample(0.0)
    assert s0.position == p0
    assert s0.velocity == (0.0, 0.0, 0.0)


def test_nonfinite_target_raises():
    try:
        JerkLimitedTrajectory((0, 0, 0), (float('nan'), 0, 0),
                              v_max=TRAJ_V_MAX_XY, a_max=TRAJ_A_MAX, j_max=TRAJ_J_MAX)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_position_error_along_traj():
    traj = JerkLimitedTrajectory(
        (0, 0, 5), (5, 0, 5), v_max=TRAJ_V_MAX_XY, a_max=TRAJ_A_MAX, j_max=TRAJ_J_MAX
    )
    s = traj.sample(traj.duration * 0.25)
    # Vehicle lagging: p behind p_d
    p = (s.position[0] - 0.2, 0.0, 5.0)
    e = position_error(s.position, p)
    assert e[0] > 0.0
