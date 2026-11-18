# Copyright 2026 EKF-MonoSLAM-Range-To-Base contributors.
#
# Portions of the polynomial evaluation (j, a, v, x under constant jerk)
# follow the structure of PX4 Autopilot VelocitySmoothing::evaluatePoly:
#   https://github.com/PX4/PX4-Autopilot (BSD-3-Clause)
# Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# The rest-to-rest path-based phase planner (scalar s(t) along u) is an
# independent closed-form implementation for stop-and-go segments; it is
# not a line-by-line port of PX4 PositionSmoothing / VelocitySmoothing
# realtime state machines.

"""Path-based jerk-limited stop-to-stop trajectory (CTRL-R4a).

Generates continuous position, velocity, and acceleration references for a
straight-line segment between two 3-D waypoints with zero terminal velocity
and acceleration, subject to scalar path limits on speed, acceleration, and
jerk (rate of change of acceleration).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

Vec3 = Tuple[float, float, float]

# Default design constraints from CTRL-R3 / R4a (configurable per instance).
DEFAULT_V_MAX = 3.0  # m/s
DEFAULT_A_MAX = 1.2  # m/s²
DEFAULT_J_MAX = 3.0  # m/s³
DEFAULT_DT = 0.008  # s (Webots basicTimeStep); informational for R4b

_EPS_LEN = 1e-12
_EPS_TIME = 1e-15


def _finite3(v: Sequence[float]) -> bool:
    return len(v) == 3 and all(math.isfinite(x) for x in v)


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(u: Vec3, s: float) -> Vec3:
    return (u[0] * s, u[1] * s, u[2] * s)


def _norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _eval_poly(j: float, a0: float, v0: float, x0: float, t: float) -> Tuple[float, float, float, float]:
    """Constant-jerk polynomial (PX4 VelocitySmoothing::evaluatePoly form).

    Returns (jerk, accel, vel, pos) at local time t >= 0.
    """
    t2 = t * t
    t3 = t2 * t
    a = a0 + j * t
    v = v0 + a0 * t + 0.5 * j * t2
    x = x0 + v0 * t + 0.5 * a0 * t2 + (1.0 / 6.0) * j * t3
    return j, a, v, x


@dataclass(frozen=True)
class _Phase:
    duration: float
    jerk: float  # constant path jerk during phase


@dataclass(frozen=True)
class TrajectorySample:
    """Sample of the desired trajectory at one time."""

    position: Vec3
    velocity: Vec3
    acceleration: Vec3
    jerk: Vec3
    path_s: float
    path_speed: float
    path_accel: float
    path_jerk: float
    progress: float  # s/L in [0, 1], or 1 if L=0
    complete: bool


class JerkLimitedTrajectory:
    """Straight-line jerk-limited S-curve from p0 to pf (rest-to-rest).

    Path length L = ||pf - p0||. Unit direction u = (pf-p0)/L when L > 0.
    Scalar profile s(t), s_dot, s_ddot with |s_dot|<=v_max, |s_ddot|<=a_max,
    |s_dddot|<=j_max, then:
      p_d = p0 + u * s,  v_d = u * s_dot,  a_d = u * s_ddot.
    """

    def __init__(
        self,
        p0: Sequence[float],
        pf: Sequence[float],
        v_max: float = DEFAULT_V_MAX,
        a_max: float = DEFAULT_A_MAX,
        j_max: float = DEFAULT_J_MAX,
        dt: float = DEFAULT_DT,
    ) -> None:
        if not _finite3(p0):
            raise ValueError('p0 must be a finite length-3 sequence')
        if not _finite3(pf):
            raise ValueError('pf must be a finite length-3 sequence')
        for name, val in (('v_max', v_max), ('a_max', a_max), ('j_max', j_max)):
            if not math.isfinite(val) or val <= 0.0:
                raise ValueError(f'{name} must be finite and > 0, got {val!r}')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f'dt must be finite and > 0, got {dt!r}')

        self._p0: Vec3 = (float(p0[0]), float(p0[1]), float(p0[2]))
        self._pf: Vec3 = (float(pf[0]), float(pf[1]), float(pf[2]))
        self._v_max = float(v_max)
        self._a_max = float(a_max)
        self._j_max = float(j_max)
        self._dt = float(dt)

        delta = _sub(self._pf, self._p0)
        self._L = _norm(delta)
        if self._L <= _EPS_LEN:
            self._L = 0.0
            self._u: Vec3 = (0.0, 0.0, 0.0)
            self._phases: Tuple[_Phase, ...] = ()
            self._duration = 0.0
            self._v_cruise = 0.0
            self._a_peak = 0.0
            return

        self._u = _scale(delta, 1.0 / self._L)
        self._phases, self._v_cruise, self._a_peak = self._plan_phases(
            self._L, self._v_max, self._a_max, self._j_max
        )
        self._duration = sum(p.duration for p in self._phases)

    @property
    def p0(self) -> Vec3:
        return self._p0

    @property
    def pf(self) -> Vec3:
        return self._pf

    @property
    def path_length(self) -> float:
        return self._L

    @property
    def direction(self) -> Vec3:
        return self._u

    @property
    def duration(self) -> float:
        """Total trajectory duration T [s]."""
        return self._duration

    @property
    def v_max(self) -> float:
        return self._v_max

    @property
    def a_max(self) -> float:
        return self._a_max

    @property
    def j_max(self) -> float:
        return self._j_max

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def cruise_speed(self) -> float:
        """Planned peak path speed (may be < v_max on short segments)."""
        return self._v_cruise

    @property
    def peak_accel(self) -> float:
        """Planned peak |path acceleration| (may be < a_max on short segments)."""
        return self._a_peak

    def is_complete(self, t: float) -> bool:
        if not math.isfinite(t):
            raise ValueError(f't must be finite, got {t!r}')
        return t >= self._duration - _EPS_TIME

    def sample(self, t: float) -> TrajectorySample:
        if not math.isfinite(t):
            raise ValueError(f't must be finite, got {t!r}')

        if self._L <= _EPS_LEN:
            return TrajectorySample(
                position=self._pf,
                velocity=(0.0, 0.0, 0.0),
                acceleration=(0.0, 0.0, 0.0),
                jerk=(0.0, 0.0, 0.0),
                path_s=0.0,
                path_speed=0.0,
                path_accel=0.0,
                path_jerk=0.0,
                progress=1.0,
                complete=True,
            )

        if t <= 0.0:
            return TrajectorySample(
                position=self._p0,
                velocity=(0.0, 0.0, 0.0),
                acceleration=(0.0, 0.0, 0.0),
                jerk=(0.0, 0.0, 0.0),
                path_s=0.0,
                path_speed=0.0,
                path_accel=0.0,
                path_jerk=0.0,
                progress=0.0,
                complete=False,
            )

        if t >= self._duration:
            return TrajectorySample(
                position=self._pf,
                velocity=(0.0, 0.0, 0.0),
                acceleration=(0.0, 0.0, 0.0),
                jerk=(0.0, 0.0, 0.0),
                path_s=self._L,
                path_speed=0.0,
                path_accel=0.0,
                path_jerk=0.0,
                progress=1.0,
                complete=True,
            )

        s, sd, sdd, sddd = self._eval_scalar(t)
        # Clamp tiny negatives from FP at phase boundaries.
        if s < 0.0:
            s = 0.0
        if s > self._L:
            s = self._L
        progress = s / self._L if self._L > 0.0 else 1.0
        return TrajectorySample(
            position=_add(self._p0, _scale(self._u, s)),
            velocity=_scale(self._u, sd),
            acceleration=_scale(self._u, sdd),
            jerk=_scale(self._u, sddd),
            path_s=s,
            path_speed=sd,
            path_accel=sdd,
            path_jerk=sddd,
            progress=progress,
            complete=False,
        )

    def sample_series(
        self, times: Iterable[float]
    ) -> Tuple[TrajectorySample, ...]:
        return tuple(self.sample(t) for t in times)

    # --- planning ---------------------------------------------------------

    @staticmethod
    def _accel_half_distance(
        v_peak: float, a_max: float, j_max: float
    ) -> Tuple[float, float, float, float, float]:
        """Distance and times to accelerate 0→v_peak under a_max, j_max.

        Returns (s, T_jerk_up, T_const_acc, T_jerk_down, a_peak).
        """
        if v_peak <= 0.0:
            return 0.0, 0.0, 0.0, 0.0, 0.0

        v_tri = a_max * a_max / j_max
        if v_peak <= v_tri + 1e-14:
            a_peak = math.sqrt(v_peak * j_max)
            t_j = a_peak / j_max
            s = j_max * t_j ** 3  # = a_peak^3 / j_max^2
            return s, t_j, 0.0, t_j, a_peak

        t_j = a_max / j_max
        t_a = (v_peak - a_max * a_max / j_max) / a_max
        s = 0.5 * v_peak * (2.0 * t_j + t_a)
        return s, t_j, t_a, t_j, a_max

    @classmethod
    def _plan_phases(
        cls, L: float, v_max: float, a_max: float, j_max: float
    ) -> Tuple[Tuple[_Phase, ...], float, float]:
        """Build 7-phase rest-to-rest profile for path length L."""
        s_half_vmax, _, _, _, _ = cls._accel_half_distance(v_max, a_max, j_max)

        if 2.0 * s_half_vmax <= L + 1e-12:
            v_c = v_max
            s_half, t1, t2, t3, a_peak = cls._accel_half_distance(
                v_c, a_max, j_max
            )
            t_cruise = max(0.0, (L - 2.0 * s_half) / v_c)
        else:
            # Short: no cruise. Either trapezoid at a_max or triangle.
            s_tri_amax = a_max ** 3 / (j_max * j_max)  # one accel half
            if L * 0.5 <= s_tri_amax + 1e-12:
                # Reduce peak accel: 2 * a^3/j^2 = L
                a_peak = (0.5 * L * j_max * j_max) ** (1.0 / 3.0)
                a_peak = min(a_peak, a_max)
                v_c = a_peak * a_peak / j_max
                s_half, t1, t2, t3, a_peak = cls._accel_half_distance(
                    v_c, a_peak, j_max
                )
            else:
                # Trapezoidal accel at a_max; solve v from s = L/2
                s = 0.5 * L
                # v^2 + (a^2/j) v - 2 a s = 0
                b = a_max * a_max / j_max
                disc = b * b + 8.0 * a_max * s
                v_c = 0.5 * (-b + math.sqrt(max(0.0, disc)))
                v_c = min(v_c, v_max)
                s_half, t1, t2, t3, a_peak = cls._accel_half_distance(
                    v_c, a_max, j_max
                )
            t_cruise = 0.0

        # Numerical cleanup
        t1 = max(0.0, t1)
        t2 = max(0.0, t2)
        t3 = max(0.0, t3)
        t_cruise = max(0.0, t_cruise)

        j = j_max
        phases = (
            _Phase(t1, +j),  # accel jerk up
            _Phase(t2, 0.0),  # const +a
            _Phase(t3, -j),  # accel jerk down → a=0
            _Phase(t_cruise, 0.0),  # cruise
            _Phase(t1, -j),  # decel jerk down
            _Phase(t2, 0.0),  # const -a
            _Phase(t3, +j),  # decel jerk up → a=0
        )
        return phases, v_c, a_peak

    def _eval_scalar(self, t: float) -> Tuple[float, float, float, float]:
        """Evaluate (s, s_dot, s_ddot, s_dddot) at time t in (0, T)."""
        j_cur = 0.0
        a = 0.0
        v = 0.0
        x = 0.0
        t_remain = t
        for phase in self._phases:
            if phase.duration <= _EPS_TIME:
                continue
            if t_remain <= phase.duration + _EPS_TIME:
                tau = min(max(t_remain, 0.0), phase.duration)
                j_cur, a, v, x = _eval_poly(phase.jerk, a, v, x, tau)
                return x, v, a, j_cur
            j_cur, a, v, x = _eval_poly(
                phase.jerk, a, v, x, phase.duration
            )
            t_remain -= phase.duration
        return x, v, a, 0.0
