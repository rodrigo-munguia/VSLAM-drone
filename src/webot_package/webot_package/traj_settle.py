# CTRL-R4d: settle detection for trajectory completion (pure, testable).

"""Physical settle check after planned trajectory time elapses."""

from __future__ import annotations

import math
from typing import Sequence

# Provisional acceptance (R4d task).
SETTLE_XY_M = 0.5
SETTLE_Z_M = 0.3
SETTLE_VH_MPS = 0.2
SETTLE_VZ_MPS = 0.2
SETTLE_DWELL_S = 0.5


def position_errors(p_d: Sequence[float], p: Sequence[float]):
    return (p_d[0] - p[0], p_d[1] - p[1], p_d[2] - p[2])


def in_settle_band(
    p_d: Sequence[float],
    p: Sequence[float],
    v: Sequence[float],
    xy_tol: float = SETTLE_XY_M,
    z_tol: float = SETTLE_Z_M,
    vh_tol: float = SETTLE_VH_MPS,
    vz_tol: float = SETTLE_VZ_MPS,
) -> bool:
    if not (len(p_d) >= 3 and len(p) >= 3 and len(v) >= 3):
        return False
    if not all(math.isfinite(c) for c in (*p_d[:3], *p[:3], *v[:3])):
        return False
    ex, ey, ez = position_errors(p_d, p)
    vh = math.hypot(v[0], v[1])
    return (
        math.hypot(ex, ey) <= xy_tol
        and abs(ez) <= z_tol
        and vh <= vh_tol
        and abs(v[2]) <= vz_tol
    )


class SettleWatch:
    """Accumulates dwell while continuously inside the settle band."""

    def __init__(self, dwell_s: float = SETTLE_DWELL_S):
        self.dwell_s = float(dwell_s)
        self._accum = 0.0

    def reset(self) -> None:
        self._accum = 0.0

    @property
    def accumulated(self) -> float:
        return self._accum

    def update(self, inside: bool, dt: float) -> bool:
        """Return True once dwell satisfied. Leaving band resets dwell."""
        if not inside:
            self._accum = 0.0
            return False
        self._accum += max(0.0, float(dt))
        return self._accum >= self.dwell_s
