# CTRL-R5/R5a: trajectory profiles + XY/Z path-speed feasibility.

"""Trajectory limits, path-speed projection, and tracker feasibility gate."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from webot_package.unified_position_control import (
    A_MAX_XY,
    A_MAX_Z,
    TRAJ_A_MAX,
    TRAJ_J_MAX,
    TRAJ_V_MAX_XY,
    TRAJ_V_MAX_Z,
    V_MAX_XY,
    V_MAX_Z,
)

_EPS = 1e-12


@dataclass(frozen=True)
class TrajProfile:
    name: str
    v_max: float  # legacy isotropic (R5 Arm A/B/C)
    a_max: float
    j_max: float
    v_max_xy: Optional[float] = None  # None → use v_max
    v_max_z: Optional[float] = None

    def effective_v_xy(self) -> float:
        return float(self.v_max if self.v_max_xy is None else self.v_max_xy)

    def effective_v_z(self) -> float:
        return float(self.v_max if self.v_max_z is None else self.v_max_z)


# R5 ablation arms (isotropic scalar; historical). Production = PROFILE_R5A.
PROFILE_A = TrajProfile("A_baseline", 1.5, 0.8, 2.0)
PROFILE_B = TrajProfile("B_fast", 3.0, 0.8, 2.0)
PROFILE_C = TrajProfile("C_fast_smooth", 3.0, 0.8, 1.0)
# A3g production: independent XY/Z traj (tracker XY 5.5 / Z 1.5).
PROFILE_R5A = TrajProfile(
    "R5a_horiz5",
    TRAJ_V_MAX_XY,
    TRAJ_A_MAX,
    TRAJ_J_MAX,
    v_max_xy=TRAJ_V_MAX_XY,
    v_max_z=TRAJ_V_MAX_Z,
)
# Prior 3 m/s cruise (selectable via launch flags; not module default).
PROFILE_R5A_V3 = TrajProfile(
    "R5a_horiz3",
    3.0,
    TRAJ_A_MAX,
    TRAJ_J_MAX,
    v_max_xy=3.0,
    v_max_z=TRAJ_V_MAX_Z,
)

PROFILES = {
    "A": PROFILE_A,
    "A_baseline": PROFILE_A,
    "B": PROFILE_B,
    "B_fast": PROFILE_B,
    "C": PROFILE_C,
    "C_fast_smooth": PROFILE_C,
    "R5A": PROFILE_R5A,
    "R5a_horiz5": PROFILE_R5A,
    "R5a_horiz3": PROFILE_R5A_V3,
    "V3": PROFILE_R5A_V3,
    "V5": PROFILE_R5A,
}


def default_profile() -> TrajProfile:
    """Production defaults (R5a; also module TRAJ_*_XY/Z constants)."""
    return PROFILE_R5A


def path_speed_limit(
    displacement: Sequence[float],
    v_max_xy: float,
    v_max_z: float,
) -> float:
    """Scalar path speed limit from independent XY/Z component limits.

    For unit direction u = Δp / L (L>0):
      v_max_path = min( v_max_xy / u_xy , v_max_z / |u_z| )
    omitting a constraint when its denominator is ~0.
    Zero-length → 0. Nonfinite / non-positive limits → nan.
    """
    if len(displacement) < 3:
        return float("nan")
    dx, dy, dz = (float(displacement[0]), float(displacement[1]),
                  float(displacement[2]))
    if not all(math.isfinite(c) for c in (dx, dy, dz, v_max_xy, v_max_z)):
        return float("nan")
    if not (v_max_xy > 0.0 and v_max_z > 0.0):
        return float("nan")
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    if L < _EPS:
        return 0.0
    ux, uy, uz = dx / L, dy / L, dz / L
    u_xy = math.hypot(ux, uy)
    limits = []
    if u_xy > _EPS:
        limits.append(v_max_xy / u_xy)
    if abs(uz) > _EPS:
        limits.append(v_max_z / abs(uz))
    if not limits:
        return 0.0
    v = min(limits)
    return v if math.isfinite(v) else float("nan")


def feasibility_xyz(
    v_max_xy: float,
    v_max_z: float,
    a_max: float,
    j_max: float,
    tracker_v_xy: float = V_MAX_XY,
    tracker_v_z: float = V_MAX_Z,
    a_sat_xy: float = A_MAX_XY,
    a_sat_z: float = A_MAX_Z,
) -> Tuple[bool, str]:
    """Profile vs tracker: component traj limits must not exceed tracker sats."""
    vals = (v_max_xy, v_max_z, a_max, j_max, tracker_v_xy, tracker_v_z)
    if not all(math.isfinite(v) for v in vals):
        return False, "nonfinite trajectory/tracker limits"
    if not (v_max_xy > 0.0 and v_max_z > 0.0 and a_max > 0.0 and j_max > 0.0):
        return False, "trajectory limits must be positive"
    if not (tracker_v_xy > 0.0 and tracker_v_z > 0.0):
        return False, "tracker velocity limits must be positive"
    if v_max_xy > tracker_v_xy + 1e-12:
        return (
            False,
            "traj v_max_xy=%.3f exceeds tracker_v_max_xy=%.3f "
            "(do not clip after planning)" % (v_max_xy, tracker_v_xy),
        )
    if v_max_z > tracker_v_z + 1e-12:
        return (
            False,
            "traj v_max_z=%.3f exceeds tracker_v_max_z=%.3f" % (
                v_max_z, tracker_v_z),
        )
    if a_max > a_sat_xy + 1e-12:
        return (
            False,
            "traj a_max=%.3f exceeds frozen tracker A_MAX_XY=%.3f" % (
                a_max, a_sat_xy),
        )
    if a_max > a_sat_z + 1e-12:
        return (
            False,
            "traj a_max=%.3f exceeds frozen tracker A_MAX_Z=%.3f" % (
                a_max, a_sat_z),
        )
    return True, "feasible under tracker saturations"


def segment_feasibility(
    p0: Sequence[float],
    pf: Sequence[float],
    v_max_xy: float,
    v_max_z: float,
    a_max: float,
    j_max: float,
    tracker_v_xy: float = V_MAX_XY,
    tracker_v_z: float = V_MAX_Z,
) -> Tuple[bool, str, float]:
    """Profile+segment gate. Returns (ok, message, v_max_path)."""
    ok, msg = feasibility_xyz(
        v_max_xy, v_max_z, a_max, j_max, tracker_v_xy, tracker_v_z
    )
    if not ok:
        return False, msg, float("nan")
    if len(p0) < 3 or len(pf) < 3:
        return False, "nonfinite or short endpoints", float("nan")
    disp = (float(pf[0]) - float(p0[0]),
            float(pf[1]) - float(p0[1]),
            float(pf[2]) - float(p0[2]))
    if not all(math.isfinite(c) for c in disp):
        return False, "nonfinite displacement", float("nan")
    v_path = path_speed_limit(disp, v_max_xy, v_max_z)
    if not math.isfinite(v_path):
        return False, "invalid path speed limit", float("nan")
    # Zero-length: valid hold (no motion).
    L = math.sqrt(disp[0] ** 2 + disp[1] ** 2 + disp[2] ** 2)
    if L < _EPS:
        return True, "zero-length segment", 0.0
    # Component cruise speeds under path limit.
    u_xy = math.hypot(disp[0], disp[1]) / L
    uz = abs(disp[2]) / L
    vh = v_path * u_xy
    vz = v_path * uz
    if vh > tracker_v_xy + 1e-9:
        return False, "planned vh=%.3f exceeds tracker xy" % vh, v_path
    if vz > tracker_v_z + 1e-9:
        return False, "planned vz=%.3f exceeds tracker z" % vz, v_path
    return True, "segment feasible v_path=%.3f" % v_path, v_path


def feasibility(
    v_max: float,
    a_max: float,
    j_max: float,
    v_sat_xy: float = V_MAX_XY,
    v_sat_z: float = V_MAX_Z,
    a_sat_xy: float = A_MAX_XY,
    a_sat_z: float = A_MAX_Z,
) -> Tuple[bool, str]:
    """Legacy isotropic gate (R5): scalar v_max applies to both XY and Z."""
    return feasibility_xyz(
        v_max, v_max, a_max, j_max,
        tracker_v_xy=v_sat_xy, tracker_v_z=v_sat_z,
        a_sat_xy=a_sat_xy, a_sat_z=a_sat_z,
    )


def profile_feasibility(
    profile: TrajProfile,
    tracker_v_xy: float = V_MAX_XY,
    tracker_v_z: float = V_MAX_Z,
) -> Tuple[bool, str]:
    return feasibility_xyz(
        profile.effective_v_xy(),
        profile.effective_v_z(),
        profile.a_max,
        profile.j_max,
        tracker_v_xy=tracker_v_xy,
        tracker_v_z=tracker_v_z,
    )


def resolve_profile(name: Optional[str]) -> TrajProfile:
    if not name:
        return default_profile()
    key = str(name).strip()
    if key in PROFILES:
        return PROFILES[key]
    raise KeyError("unknown traj profile %r; expected one of %s" % (
        name, sorted(set(PROFILES))))


def resolve_effective_limits(
    traj_v_max: float,
    traj_v_max_xy: float,
    traj_v_max_z: float,
) -> Tuple[float, float]:
    """Parameter precedence for R5 → R5a migration.

    - ``traj_v_max``: legacy isotropic default (R4e / R5).
    - ``traj_v_max_xy`` / ``traj_v_max_z``: if >0, override that axis;
      if ≤0, inherit ``traj_v_max``.
    """
    v_iso = float(traj_v_max)
    v_xy = float(traj_v_max_xy)
    v_z = float(traj_v_max_z)
    eff_xy = v_xy if v_xy > 0.0 else v_iso
    eff_z = v_z if v_z > 0.0 else v_iso
    return eff_xy, eff_z
