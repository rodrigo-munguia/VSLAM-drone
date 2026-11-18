# CTRL-R4d: acknowledged trajectory request/status state machine (pure).

"""Request identity, accept/reject, settle, cancel — no ROS dependency."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

from webot_package.traj_settle import SettleWatch, in_settle_band

STATUS_IDLE = 0
STATUS_ACCEPTED = 1
STATUS_RUNNING = 2
STATUS_SETTLED = 3
STATUS_REJECTED = 4
STATUS_CANCELLED = 5
STATUS_FAILED = 6

STILL_SPEED = 0.25  # matches TRAJ_STILL_SPEED


@dataclass
class RequestResult:
    accepted: bool
    request_id: int
    status: int
    message: str


@dataclass
class StatusSnapshot:
    request_id: int
    status: int
    ex: float = 0.0
    ey: float = 0.0
    ez: float = 0.0
    vh: float = 0.0
    vz: float = 0.0
    message: str = ""


@dataclass
class TrajRequestManager:
    """Owns one active request identity and settle dwell."""

    still_speed: float = STILL_SPEED
    active_id: int = 0
    status: int = STATUS_IDLE
    target: Optional[Tuple[float, float, float]] = None
    traj_time_elapsed: bool = False
    settle: SettleWatch = field(default_factory=SettleWatch)
    message: str = ""
    # Exclusive ownership: True while unified mission control owns motors.
    unified_owner: bool = False

    def reset_idle(self) -> None:
        self.active_id = 0
        self.status = STATUS_IDLE
        self.target = None
        self.traj_time_elapsed = False
        self.settle.reset()
        self.message = ""

    def enter_unified(self) -> None:
        self.unified_owner = True

    def exit_unified(self) -> None:
        self.unified_owner = False
        self.reset_idle()

    def handle_request(
        self,
        request_id: int,
        target: Sequence[float],
        speed: float,
        unified_mode: bool,
        traj_running: bool,
    ) -> RequestResult:
        """Accept or reject a waypoint request. Does not start motion itself."""
        rid = int(request_id)
        if not unified_mode:
            self.status = STATUS_REJECTED
            self.message = "not in unified mode"
            return RequestResult(False, rid, STATUS_REJECTED, self.message)
        if not all(_finite(c) for c in target[:3]):
            self.status = STATUS_REJECTED
            self.message = "nonfinite target"
            return RequestResult(False, rid, STATUS_REJECTED, self.message)

        # Idempotent: same id already settled → success without restart.
        if rid == self.active_id and self.status == STATUS_SETTLED:
            return RequestResult(True, rid, STATUS_SETTLED, "already settled")

        # Duplicate while running/accepted: do not restart.
        if rid == self.active_id and self.status in (
            STATUS_ACCEPTED, STATUS_RUNNING
        ):
            return RequestResult(True, rid, self.status, "duplicate ignored")

        # Another request while one is active.
        if traj_running or self.status in (STATUS_ACCEPTED, STATUS_RUNNING):
            self.message = "busy: trajectory already active"
            # Do not overwrite active_id / running status.
            return RequestResult(False, rid, STATUS_REJECTED, self.message)

        if speed > self.still_speed:
            self.active_id = rid
            self.status = STATUS_REJECTED
            self.target = None
            self.traj_time_elapsed = False
            self.settle.reset()
            self.message = "moving start rejected"
            return RequestResult(False, rid, STATUS_REJECTED, self.message)

        self.active_id = rid
        self.target = (float(target[0]), float(target[1]), float(target[2]))
        self.traj_time_elapsed = False
        self.settle.reset()
        self.status = STATUS_ACCEPTED
        self.message = "accepted"
        self.unified_owner = True
        return RequestResult(True, rid, STATUS_ACCEPTED, self.message)

    def mark_running(self) -> None:
        if self.status in (STATUS_ACCEPTED, STATUS_RUNNING):
            self.status = STATUS_RUNNING
            self.message = "running"

    def mark_traj_time_done(self) -> None:
        self.traj_time_elapsed = True

    def cancel(self) -> StatusSnapshot:
        if self.status in (STATUS_ACCEPTED, STATUS_RUNNING):
            self.status = STATUS_CANCELLED
            self.message = "cancelled"
            self.traj_time_elapsed = False
            self.settle.reset()
        return self.snapshot()

    def update_settle(
        self,
        p: Sequence[float],
        v: Sequence[float],
        dt: float,
    ) -> StatusSnapshot:
        """After planned time elapsed, accumulate dwell; return current status."""
        if self.status not in (STATUS_ACCEPTED, STATUS_RUNNING, STATUS_SETTLED):
            return self.snapshot(p=p, v=v)
        if self.status == STATUS_SETTLED:
            return self.snapshot(p=p, v=v)
        if self.target is None or not self.traj_time_elapsed:
            return self.snapshot(p=p, v=v)

        inside = in_settle_band(self.target, p, v)
        if self.settle.update(inside, dt):
            self.status = STATUS_SETTLED
            self.message = "settled"
        else:
            self.status = STATUS_RUNNING
            self.message = "settling" if inside else "running"
        return self.snapshot(p=p, v=v)

    def snapshot(
        self,
        p: Optional[Sequence[float]] = None,
        v: Optional[Sequence[float]] = None,
    ) -> StatusSnapshot:
        ex = ey = ez = vh = vz = 0.0
        if self.target is not None and p is not None and len(p) >= 3:
            ex = self.target[0] - float(p[0])
            ey = self.target[1] - float(p[1])
            ez = self.target[2] - float(p[2])
        if v is not None and len(v) >= 3:
            vh = (float(v[0]) ** 2 + float(v[1]) ** 2) ** 0.5
            vz = float(v[2])
        return StatusSnapshot(
            request_id=self.active_id,
            status=self.status,
            ex=ex, ey=ey, ez=ez, vh=vh, vz=vz,
            message=self.message,
        )


def mission_to_gps_enu(
    spawn_x: float, spawn_y: float, spawn_z: float,
    mission_x: float, mission_y: float, mission_z: float,
) -> Tuple[float, float, float]:
    """Convert ControlQuad mission args to absolute GPS ENU.

    Legacy ControlQuad GPS path stores y = -gps_y relative to spawn, so
    mission_y = -(gps_y - spawn_y). Invert for the driver ENU contract:
      gps = spawn + (mission_x, -mission_y, mission_z)
    Do NOT apply an extra publish-time Y negation.
    """
    return (
        float(spawn_x) + float(mission_x),
        float(spawn_y) - float(mission_y),
        float(spawn_z) + float(mission_z),
    )


def _finite(x: float) -> bool:
    return x == x and abs(x) != float("inf")
