# CTRL-R5/R5a: profile feasibility + independent XY/Z path-speed limits.
import math

import pytest

from webot_package.traj_profile import (
    PROFILE_A,
    PROFILE_B,
    PROFILE_C,
    PROFILE_R5A,
    default_profile,
    feasibility,
    feasibility_xyz,
    path_speed_limit,
    profile_feasibility,
    resolve_effective_limits,
    segment_feasibility,
)
from webot_package.unified_position_control import (
    A_MAX_XY,
    TRAJ_A_MAX,
    TRAJ_J_MAX,
    TRAJ_V_MAX_XY,
    TRAJ_V_MAX_Z,
    V_MAX_XY,
    V_MAX_Z,
)


def test_arm_a_feasible_historical_r4e():
    # Historical isotropic 1.5 still ok under production tracker.
    ok, msg = profile_feasibility(PROFILE_A)
    assert ok, msg
    assert PROFILE_A.v_max == pytest.approx(1.5)


def test_arm_b_infeasible_isotropic_on_z():
    # Isotropic 3.0 exceeds tracker Z (1.5) even with raised XY tracker.
    ok, msg = profile_feasibility(PROFILE_B)
    assert not ok
    assert PROFILE_B.v_max > V_MAX_Z


def test_arm_c_infeasible_same_vmax():
    ok, msg = profile_feasibility(PROFILE_C)
    assert not ok


def test_legacy_feasibility_rejects_above_tracker():
    ok, msg = feasibility(3.0, 0.8, 2.0, v_sat_xy=2.0)
    assert not ok


def test_path_speed_pure_xy():
    v = path_speed_limit((10.0, 0.0, 0.0), 3.0, 1.5)
    assert v == pytest.approx(3.0)


def test_path_speed_pure_z():
    v = path_speed_limit((0.0, 0.0, 10.0), 3.0, 1.5)
    assert v == pytest.approx(1.5)


def test_path_speed_mixed_3d():
    # 45° in XZ: u_xy = |uz| = √2/2 → v_path = √2 * min(3, 1.5) = 1.5√2
    v = path_speed_limit((1.0, 0.0, 1.0), 3.0, 1.5)
    assert v == pytest.approx(1.5 * math.sqrt(2.0))


def test_path_speed_zero_length():
    assert path_speed_limit((0.0, 0.0, 0.0), 3.0, 1.5) == 0.0


def test_path_speed_invalid():
    assert math.isnan(path_speed_limit((1, 0, 0), -1.0, 1.5))
    assert math.isnan(path_speed_limit((1, 0, 0), float("nan"), 1.5))


def test_horiz_3_rejected_with_tracker_2():
    ok, msg = feasibility_xyz(3.0, 1.5, 0.8, 2.0, tracker_v_xy=2.0, tracker_v_z=1.5)
    assert not ok
    assert "v_max_xy" in msg


def test_horiz_3_feasible_with_tracker_3_5():
    ok, msg = feasibility_xyz(3.0, 1.5, 0.8, 2.0, tracker_v_xy=3.5, tracker_v_z=1.5)
    assert ok, msg


def test_vertical_constrained_to_1_5():
    v = path_speed_limit((0, 0, 50), 3.0, 1.5)
    assert v == pytest.approx(1.5)
    ok, msg, vp = segment_feasibility(
        (0, 0, 0), (0, 0, 50), 3.0, 1.5, 0.8, 2.0,
        tracker_v_xy=3.5, tracker_v_z=1.5,
    )
    assert ok, msg
    assert vp == pytest.approx(1.5)


def test_mixed_segment_respects_both():
    ok, msg, vp = segment_feasibility(
        (0, 0, 10), (40, 0, 20), 3.0, 1.5, 0.8, 2.0,
        tracker_v_xy=3.5, tracker_v_z=1.5,
    )
    assert ok, msg
    L = math.hypot(40.0, 10.0)
    u_xy = 40.0 / L
    uz = 10.0 / L
    assert vp * u_xy <= 3.0 + 1e-9
    assert vp * uz <= 1.5 + 1e-9


def test_zero_length_segment_valid():
    ok, msg, vp = segment_feasibility(
        (1, 2, 3), (1, 2, 3), 3.0, 1.5, 0.8, 2.0,
        tracker_v_xy=3.5, tracker_v_z=1.5,
    )
    assert ok, msg
    assert vp == 0.0


def test_r5a_profile_feasible_as_production_default():
    ok, msg = profile_feasibility(PROFILE_R5A)
    assert ok, msg
    ok2, msg2 = profile_feasibility(default_profile())
    assert ok2, msg2
    assert default_profile() is PROFILE_R5A


def test_resolve_effective_limits_precedence():
    # Inherit isotropic when xy/z ≤0
    assert resolve_effective_limits(1.5, -1.0, -1.0) == (1.5, 1.5)
    # Override axes independently
    assert resolve_effective_limits(1.5, 3.0, 1.5) == (3.0, 1.5)
    assert resolve_effective_limits(1.5, 3.0, -1.0) == (3.0, 1.5)


def test_amax_ok():
    assert PROFILE_R5A.a_max <= A_MAX_XY


def test_production_defaults_are_r5a():
    # A3g: validated 5.0 / 5.5 cruise defaults (prior 3.0 / 3.5 via V3 / launch flags).
    assert TRAJ_V_MAX_XY == pytest.approx(5.0)
    assert TRAJ_V_MAX_Z == pytest.approx(1.5)
    assert TRAJ_A_MAX == pytest.approx(0.8)
    assert TRAJ_J_MAX == pytest.approx(2.0)
    assert V_MAX_XY == pytest.approx(5.5)
    assert V_MAX_Z == pytest.approx(1.5)
    assert PROFILE_R5A.effective_v_xy() == pytest.approx(TRAJ_V_MAX_XY)
    assert PROFILE_R5A.effective_v_z() == pytest.approx(TRAJ_V_MAX_Z)
    assert TRAJ_V_MAX_XY <= V_MAX_XY + 1e-12


def test_prior_3ms_profile_still_selectable():
    from webot_package.traj_profile import PROFILE_R5A_V3, PROFILES
    assert PROFILE_R5A_V3.effective_v_xy() == pytest.approx(3.0)
    assert PROFILES["V3"].effective_v_xy() == pytest.approx(3.0)
    assert PROFILES["R5a_horiz3"].effective_v_xy() == pytest.approx(3.0)
    assert PROFILES["V5"].effective_v_xy() == pytest.approx(5.0)
