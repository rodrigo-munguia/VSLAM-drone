# CTRL-R4d: ACK request/status identity + ownership tests.
from webot_package.traj_request_manager import (
    STATUS_ACCEPTED,
    STATUS_CANCELLED,
    STATUS_REJECTED,
    STATUS_RUNNING,
    STATUS_SETTLED,
    TrajRequestManager,
    mission_to_gps_enu,
)


def test_accept_waypoint():
    m = TrajRequestManager()
    r = m.handle_request(1, (5, 0, 10), speed=0.0, unified_mode=True, traj_running=False)
    assert r.accepted
    assert r.request_id == 1
    assert r.status == STATUS_ACCEPTED
    m.mark_running()
    assert m.status == STATUS_RUNNING


def test_reject_not_unified():
    m = TrajRequestManager()
    r = m.handle_request(1, (5, 0, 10), speed=0.0, unified_mode=False, traj_running=False)
    assert not r.accepted
    assert r.status == STATUS_REJECTED


def test_reject_moving_start():
    m = TrajRequestManager()
    r = m.handle_request(2, (5, 0, 10), speed=0.5, unified_mode=True, traj_running=False)
    assert not r.accepted
    assert r.status == STATUS_REJECTED
    assert "moving" in r.message


def test_matching_request_id_on_settle():
    m = TrajRequestManager()
    m.handle_request(7, (5, 0, 10), 0.0, True, False)
    m.mark_running()
    m.mark_traj_time_done()
    snap = m.update_settle((5.0, 0.0, 10.0), (0.0, 0.0, 0.0), 0.6)
    assert snap.request_id == 7
    assert snap.status == STATUS_SETTLED


def test_stale_completion_does_not_match_new_id():
    m = TrajRequestManager()
    m.handle_request(1, (0, 0, 10), 0.0, True, False)
    m.mark_running()
    m.mark_traj_time_done()
    m.update_settle((0, 0, 10), (0, 0, 0), 0.6)
    assert m.status == STATUS_SETTLED
    # New waypoint: previous settle must not satisfy id=2 until new settle.
    r = m.handle_request(2, (5, 0, 10), 0.0, True, False)
    assert r.accepted
    assert r.request_id == 2
    assert m.status == STATUS_ACCEPTED
    assert m.status != STATUS_SETTLED


def test_completion_vs_physical_settle():
    m = TrajRequestManager()
    m.handle_request(3, (5, 0, 10), 0.0, True, False)
    m.mark_running()
    # Traj time not elapsed → not settled even if in band.
    snap = m.update_settle((5, 0, 10), (0, 0, 0), 1.0)
    assert snap.status != STATUS_SETTLED
    m.mark_traj_time_done()
    # In band but dwell incomplete.
    snap = m.update_settle((5, 0, 10), (0, 0, 0), 0.2)
    assert snap.status == STATUS_RUNNING
    snap = m.update_settle((5, 0, 10), (0, 0, 0), 0.4)
    assert snap.status == STATUS_SETTLED


def test_dwell_reset_on_leave():
    m = TrajRequestManager()
    m.handle_request(4, (0, 0, 10), 0.0, True, False)
    m.mark_running()
    m.mark_traj_time_done()
    m.update_settle((0, 0, 10), (0, 0, 0), 0.4)
    assert m.settle.accumulated >= 0.4
    m.update_settle((1.0, 0, 10), (0, 0, 0), 0.1)  # leave XY band
    assert m.settle.accumulated == 0.0
    assert m.status == STATUS_RUNNING


def test_cancel():
    m = TrajRequestManager()
    m.handle_request(5, (1, 0, 10), 0.0, True, False)
    m.mark_running()
    snap = m.cancel()
    assert snap.status == STATUS_CANCELLED
    assert snap.request_id == 5


def test_duplicate_does_not_restart():
    m = TrajRequestManager()
    m.handle_request(9, (2, 0, 10), 0.0, True, False)
    m.mark_running()
    target_before = m.target
    r = m.handle_request(9, (99, 0, 10), 0.0, True, True)
    assert r.accepted
    assert r.status == STATUS_RUNNING
    assert m.target == target_before


def test_busy_rejects_new_id():
    m = TrajRequestManager()
    m.handle_request(1, (0, 0, 10), 0.0, True, False)
    m.mark_running()
    r = m.handle_request(2, (5, 0, 10), 0.0, True, True)
    assert not r.accepted
    assert r.status == STATUS_REJECTED
    assert m.active_id == 1


def test_enu_mission_mapping():
    gx, gy, gz = mission_to_gps_enu(10.0, 20.0, 15.0, 5.0, 3.0, 5.0)
    assert gx == 15.0
    assert gy == 17.0  # spawn_y - mission_y
    assert gz == 20.0


def test_exclusive_ownership_flag():
    m = TrajRequestManager()
    assert not m.unified_owner
    m.enter_unified()
    assert m.unified_owner
    m.handle_request(1, (0, 0, 10), 0.0, True, False)
    assert m.unified_owner
    m.exit_unified()
    assert not m.unified_owner
    assert m.status == 0
