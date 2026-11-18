# CTRL-R4a: deterministic offline tests for path-based jerk-limited trajectories.
import math

import pytest

from webot_package.jerk_limited_trajectory import (
    DEFAULT_A_MAX,
    DEFAULT_DT,
    DEFAULT_J_MAX,
    DEFAULT_V_MAX,
    JerkLimitedTrajectory,
)


def _near(a, b, tol=1e-9):
    return abs(a - b) <= tol


def _vec_near(u, v, tol=1e-9):
    return all(_near(u[i], v[i], tol) for i in range(3))


def _max_abs_jerk(traj, n=2000):
    """Finite-diff path jerk from acceleration samples (bound check)."""
    if traj.duration <= 0.0:
        return 0.0
    dt = traj.duration / n
    prev_a = traj.sample(0.0).path_accel
    jmax = 0.0
    for i in range(1, n + 1):
        a = traj.sample(i * dt).path_accel
        j = abs(a - prev_a) / dt
        jmax = max(jmax, j)
        prev_a = a
    return jmax


# --- A. Zero-distance -------------------------------------------------------

def test_zero_distance():
    traj = JerkLimitedTrajectory((1.0, 2.0, 3.0), (1.0, 2.0, 3.0))
    assert traj.duration == 0.0
    assert traj.path_length == 0.0
    assert traj.is_complete(0.0)
    s0 = traj.sample(0.0)
    assert _vec_near(s0.position, (1.0, 2.0, 3.0))
    assert _vec_near(s0.velocity, (0.0, 0.0, 0.0))
    assert _vec_near(s0.acceleration, (0.0, 0.0, 0.0))
    assert s0.complete
    s1 = traj.sample(1.0)
    assert _vec_near(s1.position, (1.0, 2.0, 3.0))
    assert s1.complete


# --- B. Short trajectory ----------------------------------------------------

def test_short_no_cruise():
    # ~0.5 m: below distance to reach vmax with defaults
    p0 = (0.0, 0.0, 0.0)
    pf = (0.5, 0.0, 0.0)
    traj = JerkLimitedTrajectory(p0, pf)
    assert traj.cruise_speed < DEFAULT_V_MAX - 1e-6
    assert traj.duration > 0.0

    times = [i * traj.duration / 400 for i in range(401)]
    samples = traj.sample_series(times)
    for s in samples:
        assert s.path_speed <= traj.cruise_speed + 1e-6
        assert abs(s.path_accel) <= traj.peak_accel + 1e-6
        assert s.position[0] >= -1e-9
        assert s.position[0] <= 0.5 + 1e-6

    end = traj.sample(traj.duration)
    assert _vec_near(end.position, pf, 1e-9)
    assert _vec_near(end.velocity, (0.0, 0.0, 0.0), 1e-9)
    assert _vec_near(end.acceleration, (0.0, 0.0, 0.0), 1e-9)
    assert end.complete


# --- C. Long trajectory -----------------------------------------------------

def test_long_with_cruise():
    traj = JerkLimitedTrajectory((0.0, 0.0, 0.0), (50.0, 0.0, 0.0))
    assert traj.cruise_speed == pytest.approx(DEFAULT_V_MAX, rel=0, abs=1e-12)
    assert traj.duration > 50.0 / DEFAULT_V_MAX  # accel/decel overhead

    n = 5000
    dt = traj.duration / n
    saw_cruise = False
    v_peak = 0.0
    a_peak = 0.0
    for i in range(n + 1):
        s = traj.sample(i * dt)
        v_peak = max(v_peak, s.path_speed)
        a_peak = max(a_peak, abs(s.path_accel))
        if abs(s.path_speed - DEFAULT_V_MAX) < 1e-6 and abs(s.path_accel) < 1e-6:
            saw_cruise = True
        assert s.path_speed <= DEFAULT_V_MAX + 1e-6
        assert abs(s.path_accel) <= DEFAULT_A_MAX + 1e-6

    assert saw_cruise
    assert v_peak == pytest.approx(DEFAULT_V_MAX, abs=1e-6)
    assert a_peak <= DEFAULT_A_MAX + 1e-6

    # Jerk bound: closed-form |j|<=j_max; FD estimate slightly above at edges
    j_fd = _max_abs_jerk(traj, n=4000)
    assert j_fd <= DEFAULT_J_MAX * 1.05 + 0.5  # loose FD tolerance

    end = traj.sample(traj.duration + 1.0)
    assert _vec_near(end.position, (50.0, 0.0, 0.0), 1e-9)
    assert _vec_near(end.velocity, (0.0, 0.0, 0.0), 1e-9)
    assert _vec_near(end.acceleration, (0.0, 0.0, 0.0), 1e-9)


def test_long_sample_jerk_closed_form():
    traj = JerkLimitedTrajectory((0.0, 0.0, 0.0), (50.0, 0.0, 0.0))
    for i in range(1001):
        s = traj.sample(i * traj.duration / 1000)
        assert abs(s.path_jerk) <= DEFAULT_J_MAX + 1e-9


# --- D. 3-D diagonal --------------------------------------------------------

def test_diagonal_3d():
    p0 = (1.0, -2.0, 5.0)
    pf = (11.0, 8.0, 15.0)  # delta = (10, 10, 10), L=10*sqrt(3)
    traj = JerkLimitedTrajectory(p0, pf)
    L = traj.path_length
    assert L == pytest.approx(10.0 * math.sqrt(3.0), rel=1e-12)
    u = traj.direction
    assert _near(_norm := math.sqrt(u[0] ** 2 + u[1] ** 2 + u[2] ** 2), 1.0)
    # direction along (1,1,1)
    assert _near(u[0], u[1], 1e-12) and _near(u[1], u[2], 1e-12)

    mid = traj.sample(0.5 * traj.duration)
    # Position on the line segment
    for k in range(3):
        assert min(p0[k], pf[k]) - 1e-6 <= mid.position[k] <= max(p0[k], pf[k]) + 1e-6
    # Vector bounds
    spd = math.sqrt(sum(c * c for c in mid.velocity))
    acc = math.sqrt(sum(c * c for c in mid.acceleration))
    assert spd <= DEFAULT_V_MAX + 1e-6
    assert acc <= DEFAULT_A_MAX + 1e-6

    end = traj.sample(traj.duration)
    assert _vec_near(end.position, pf, 1e-9)
    assert _vec_near(end.velocity, (0.0, 0.0, 0.0), 1e-9)


def test_negative_and_vertical():
    traj = JerkLimitedTrajectory((0.0, 0.0, 10.0), (0.0, 0.0, 0.0))
    assert traj.direction[2] == pytest.approx(-1.0)
    end = traj.sample(traj.duration)
    assert _vec_near(end.position, (0.0, 0.0, 0.0), 1e-9)
    assert end.velocity[2] == pytest.approx(0.0, abs=1e-9)


# --- E. Continuity and determinism ------------------------------------------

def test_continuity_across_phases():
    traj = JerkLimitedTrajectory((0.0, 0.0, 0.0), (50.0, 0.0, 0.0))
    # Dense samples: jumps in p,v,a must be consistent with dt and limits
    n = 8000
    dt = traj.duration / n
    prev = traj.sample(0.0)
    for i in range(1, n + 1):
        cur = traj.sample(i * dt)
        # Position continuous
        assert abs(cur.path_s - prev.path_s) <= DEFAULT_V_MAX * dt + 1e-6
        # Velocity continuous (accel-limited)
        assert abs(cur.path_speed - prev.path_speed) <= DEFAULT_A_MAX * dt + 1e-5
        # Acceleration continuous (jerk-limited)
        assert abs(cur.path_accel - prev.path_accel) <= DEFAULT_J_MAX * dt + 1e-5
        prev = cur


def test_determinism_and_order_independence():
    traj = JerkLimitedTrajectory((0.0, 0.0, 0.0), (5.0, 0.0, 0.0))
    times = [0.0, 0.1, traj.duration * 0.5, traj.duration, traj.duration + 2.0]
    a = [traj.sample(t) for t in times]
    b = [traj.sample(t) for t in reversed(times)]
    b.reverse()
    for sa, sb in zip(a, b):
        assert _vec_near(sa.position, sb.position, 0.0)
        assert _vec_near(sa.velocity, sb.velocity, 0.0)
        assert _vec_near(sa.acceleration, sb.acceleration, 0.0)
    # Same timestamp twice
    t = 0.37 * traj.duration
    s1 = traj.sample(t)
    s2 = traj.sample(t)
    assert s1.position == s2.position
    assert s1.velocity == s2.velocity
    assert s1.acceleration == s2.acceleration


def test_before_start_and_after_end():
    traj = JerkLimitedTrajectory((0.0, 0.0, 0.0), (5.0, 1.0, -2.0))
    s_neg = traj.sample(-1.0)
    assert _vec_near(s_neg.position, traj.p0)
    assert _vec_near(s_neg.velocity, (0.0, 0.0, 0.0))
    s_end = traj.sample(traj.duration + 100.0)
    assert _vec_near(s_end.position, traj.pf)
    assert _vec_near(s_end.velocity, (0.0, 0.0, 0.0))
    assert _vec_near(s_end.acceleration, (0.0, 0.0, 0.0))
    assert s_end.complete


# --- F. Invalid inputs ------------------------------------------------------

def test_invalid_limits():
    with pytest.raises(ValueError):
        JerkLimitedTrajectory((0, 0, 0), (1, 0, 0), v_max=0.0)
    with pytest.raises(ValueError):
        JerkLimitedTrajectory((0, 0, 0), (1, 0, 0), a_max=-1.0)
    with pytest.raises(ValueError):
        JerkLimitedTrajectory((0, 0, 0), (1, 0, 0), j_max=float('nan'))
    with pytest.raises(ValueError):
        JerkLimitedTrajectory((0, 0, 0), (1, 0, 0), dt=-0.01)


def test_invalid_positions():
    with pytest.raises(ValueError):
        JerkLimitedTrajectory((0, 0), (1, 0, 0))
    with pytest.raises(ValueError):
        JerkLimitedTrajectory((0, 0, float('inf')), (1, 0, 0))


def test_nonfinite_sample_time():
    traj = JerkLimitedTrajectory((0, 0, 0), (1, 0, 0))
    with pytest.raises(ValueError):
        traj.sample(float('nan'))


def test_very_short_positive():
    traj = JerkLimitedTrajectory((0, 0, 0), (1e-6, 0, 0))
    assert traj.duration >= 0.0
    end = traj.sample(traj.duration)
    assert _vec_near(end.position, (1e-6, 0, 0), 1e-12)
    assert _vec_near(end.velocity, (0, 0, 0), 1e-9)


def test_defaults_match_r4a_table():
    assert DEFAULT_V_MAX == 3.0
    assert DEFAULT_A_MAX == 1.2
    assert DEFAULT_J_MAX == 3.0
    assert DEFAULT_DT == 0.008
