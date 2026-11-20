# CTRL-R4b: unified 3-D position/velocity → acceleration → thrust (ENU).
# Pure helpers; no ROS/Webots imports.

"""Unified ENU position/velocity cascade for Webots Mavic (R4b/R4c).

Equations (East-North-Up):
  e_p = p_d - p
  v_sp = sat(v_d + K_p * e_p)     # R4c: v_d traj FF; hold uses v_d=0
  a_sp = sat(a_d + K_v (v_sp - v))  # R4c: a_d traj FF once; hold a_d=0
  f_d  = m * (a_sp + g * e_z)     # desired force [N] (gravity once)
  collective ω from |f| and tilt via T = 4 c ω²
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]

G_ACCEL = 9.81
# Webots Mavic2Pro: hover ω≈68.5 → T≈4.88 N → m_eq≈0.50 kg (A3).
VEHICLE_MASS = 0.50  # kg (effective hover mass)
THRUST_COEFF = 2.6e-4  # N/(rad/s)² per propeller (PROTO thrustConstants)
N_ROTORS = 4
HOVER_OMEGA = 68.5  # rad/s (legacy takeoff arm)
MOTOR_OMEGA_MAX = 576.0

# Initial design gains (not a tune campaign).
K_POS_XY = 0.45  # 1/s
K_POS_Z = 0.70  # 1/s
K_VEL_XY = 2.0  # 1/s
K_VEL_Z = 2.2  # 1/s
V_MAX_XY = 5.5  # m/s (A3g production: margin above traj XY 5.0)
V_MAX_Z = 1.5  # m/s
A_MAX_XY = 2.5  # m/s² (match S2)
A_MAX_Z = 2.0  # m/s²
TILT_MAX = math.radians(12.0)


def _finite3(v: Sequence[float]) -> bool:
    return len(v) >= 3 and all(math.isfinite(float(v[i])) for i in range(3))


def clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def position_error(p_d: Sequence[float], p: Sequence[float]) -> Vec3:
    if not (_finite3(p_d) and _finite3(p)):
        return (0.0, 0.0, 0.0)
    return (p_d[0] - p[0], p_d[1] - p[1], p_d[2] - p[2])


def position_to_velocity_sp(
    e_p: Sequence[float],
    v_d: Sequence[float] = (0.0, 0.0, 0.0),
    kp_xy: float = K_POS_XY,
    kp_z: float = K_POS_Z,
    v_max_xy: float = V_MAX_XY,
    v_max_z: float = V_MAX_Z,
) -> Vec3:
    """v_sp = sat(v_d + K_p e_p) with horizontal-norm and vertical limits."""
    if not _finite3(e_p):
        return (0.0, 0.0, 0.0)
    vd = v_d if _finite3(v_d) else (0.0, 0.0, 0.0)
    vx = vd[0] + kp_xy * e_p[0]
    vy = vd[1] + kp_xy * e_p[1]
    vz = vd[2] + kp_z * e_p[2]
    vh = math.hypot(vx, vy)
    if vh > v_max_xy > 0.0:
        s = v_max_xy / vh
        vx *= s
        vy *= s
    vz = clamp(vz, -abs(v_max_z), abs(v_max_z))
    if not all(math.isfinite(c) for c in (vx, vy, vz)):
        return (0.0, 0.0, 0.0)
    return (vx, vy, vz)


def velocity_to_accel_sp(
    v_sp: Sequence[float],
    v: Sequence[float],
    a_d: Sequence[float] = (0.0, 0.0, 0.0),
    kv_xy: float = K_VEL_XY,
    kv_z: float = K_VEL_Z,
    a_max_xy: float = A_MAX_XY,
    a_max_z: float = A_MAX_Z,
) -> Vec3:
    """a_sp = sat(a_d + K_v (v_sp - v)); a_d applied exactly once."""
    if not (_finite3(v_sp) and _finite3(v)):
        return (0.0, 0.0, 0.0)
    ad = a_d if _finite3(a_d) else (0.0, 0.0, 0.0)
    ax = ad[0] + kv_xy * (v_sp[0] - v[0])
    ay = ad[1] + kv_xy * (v_sp[1] - v[1])
    az = ad[2] + kv_z * (v_sp[2] - v[2])
    ah = math.hypot(ax, ay)
    if ah > a_max_xy > 0.0:
        s = a_max_xy / ah
        ax *= s
        ay *= s
    az = clamp(az, -abs(a_max_z), abs(a_max_z))
    if not all(math.isfinite(c) for c in (ax, ay, az)):
        return (0.0, 0.0, 0.0)
    return (ax, ay, az)


def desired_force(
    a_sp: Sequence[float],
    mass: float = VEHICLE_MASS,
    g: float = G_ACCEL,
) -> Vec3:
    """f_d = m (a_sp + g e_z) [N], ENU."""
    if not _finite3(a_sp) or not math.isfinite(mass) or mass <= 0.0:
        return (0.0, 0.0, mass * g if math.isfinite(mass) and mass > 0 else VEHICLE_MASS * g)
    return (mass * a_sp[0], mass * a_sp[1], mass * (a_sp[2] + g))


def force_to_collective_omega(
    f_d: Sequence[float],
    roll: float,
    pitch: float,
    c: float = THRUST_COEFF,
    n_rotors: int = N_ROTORS,
    omega_max: float = MOTOR_OMEGA_MAX,
    tilt_max: float = TILT_MAX,
) -> Tuple[float, float, bool]:
    """Map desired ENU force to collective rotor ω [rad/s].

    Uses world-vertical force demand T_z = f_d_z and measured tilt:
      T_body = T_z / max(cos α, cos tilt_max)
      ω = sqrt(T_body / (n c))
    Returns (omega, thrust_N, saturated).
    """
    if not _finite3(f_d) or not (math.isfinite(roll) and math.isfinite(pitch)):
        hover_t = VEHICLE_MASS * G_ACCEL
        w = math.sqrt(hover_t / (n_rotors * c)) if c > 0 else HOVER_OMEGA
        return w, hover_t, False
    if not (math.isfinite(c) and c > 0.0 and n_rotors > 0):
        return HOVER_OMEGA, VEHICLE_MASS * G_ACCEL, True

    # Required vertical thrust in world (positive up).
    t_z = max(0.0, float(f_d[2]))
    cos_a = math.cos(roll) * math.cos(pitch)
    cos_floor = math.cos(tilt_max)
    cos_a = clamp(cos_a, cos_floor, 1.0)
    t_body = t_z / cos_a
    # Also ensure horizontal demand is feasible under tilt_max (soft).
    t_h = math.hypot(f_d[0], f_d[1])
    t_req = math.hypot(t_h, t_z)
    if t_req > t_body:
        t_body = t_req

    omega_sq = t_body / (n_rotors * c)
    if not math.isfinite(omega_sq) or omega_sq < 0.0:
        omega_sq = 0.0
    omega = math.sqrt(omega_sq)
    saturated = omega > omega_max
    if saturated:
        omega = omega_max
    return omega, n_rotors * c * omega * omega, saturated


def hover_force(mass: float = VEHICLE_MASS, g: float = G_ACCEL) -> Vec3:
    return (0.0, 0.0, mass * g)


def unified_step(
    p_d: Sequence[float],
    p: Sequence[float],
    v: Sequence[float],
    roll: float,
    pitch: float,
    v_d: Sequence[float] = (0.0, 0.0, 0.0),
    a_d: Sequence[float] = (0.0, 0.0, 0.0),
    v_max_xy: float = V_MAX_XY,
    v_max_z: float = V_MAX_Z,
) -> Tuple[Vec3, Vec3, Vec3, float, bool]:
    """One cascade step (hold: v_d=a_d=0; traj: feedforward once).

    Returns (v_sp, a_sp, f_d, collective_omega, saturated).
    ``v_max_xy`` / ``v_max_z`` are tracker velocity-reference saturations
    (production default: R5a 3.5 / 1.5 m/s).
    """
    e = position_error(p_d, p)
    v_sp = position_to_velocity_sp(
        e, v_d=v_d, v_max_xy=v_max_xy, v_max_z=v_max_z
    )
    a_sp = velocity_to_accel_sp(v_sp, v, a_d=a_d)
    f_d = desired_force(a_sp)
    omega, _thrust, sat = force_to_collective_omega(f_d, roll, pitch)
    return v_sp, a_sp, f_d, omega, sat


# Trajectory limits (A3g production: validated 5 m/s cruise). Independent XY/Z;
# isotropic TRAJ_V_MAX is legacy-only (prefer TRAJ_V_MAX_XY / TRAJ_V_MAX_Z).
# Prior 3 m/s: launch --traj-v-max-xy 3.0 --tracker-v-max-xy 3.5
TRAJ_V_MAX_XY = 5.0  # m/s
TRAJ_V_MAX_Z = 1.5  # m/s
TRAJ_V_MAX = TRAJ_V_MAX_XY  # legacy isotropic alias (horizontal)
TRAJ_A_MAX = 0.8  # m/s²
TRAJ_J_MAX = 2.0  # m/s³
TRAJ_STILL_SPEED = 0.25  # m/s — reject rest-to-rest start if faster


def measured_speed(v: Sequence[float]) -> float:
    if not _finite3(v):
        return float('inf')
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
