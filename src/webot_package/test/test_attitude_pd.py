# CTRL-S1: deterministic checks for attitude PD and setpoint saturation.
import math

from webot_package.webot_robot_driver import (
    attitude_pd,
    saturate_attitude_setpoint,
    K_ATT_LIMIT,
)


def test_zero_error_no_proportional():
    assert attitude_pd(0.0, 0.0, 0.0, 50.0, 1.0) == 0.0
    assert attitude_pd(0.1, 0.1, 0.0, 50.0, 1.0) == 0.0


def test_roll_restoring_signs():
    # Positive measured roll vs level → positive u (legacy leveler).
    assert attitude_pd(0.1, 0.0, 0.0, 50.0, 1.0) > 0.0
    assert attitude_pd(-0.1, 0.0, 0.0, 50.0, 1.0) < 0.0
    # Desire above measured → negative u (drive toward +desired).
    assert attitude_pd(0.0, 0.1, 0.0, 50.0, 1.0) < 0.0


def test_pitch_restoring_signs():
    assert attitude_pd(0.05, 0.0, 0.0, 30.0, 1.0) > 0.0
    assert attitude_pd(-0.05, 0.0, 0.0, 30.0, 1.0) < 0.0
    assert attitude_pd(0.0, -0.05, 0.0, 30.0, 1.0) > 0.0


def test_rate_damping_term():
    u = attitude_pd(0.0, 0.0, 0.2, 50.0, 1.0)
    assert abs(u - 0.2) < 1e-9


def test_attitude_limits():
    lim = K_ATT_LIMIT
    assert saturate_attitude_setpoint(0.0) == 0.0
    assert saturate_attitude_setpoint(1.0) == lim
    assert saturate_attitude_setpoint(-1.0) == -lim
    assert abs(saturate_attitude_setpoint(math.radians(5.0)) - math.radians(5.0)) < 1e-12
    assert saturate_attitude_setpoint(float('nan')) == 0.0
    assert saturate_attitude_setpoint(float('inf')) == 0.0


def test_nonfinite_measured_safe():
    assert math.isfinite(attitude_pd(float('nan'), 0.1, 0.0, 50.0, 1.0))
