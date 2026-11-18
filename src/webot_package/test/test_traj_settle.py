# CTRL-R4d: settle band + dwell tests.
import math

from webot_package.traj_settle import SettleWatch, in_settle_band


def test_in_band_nominal():
    assert in_settle_band((0, 0, 10), (0.1, 0.1, 10.1), (0.05, 0.0, 0.05))


def test_out_of_band_xy():
    assert not in_settle_band((0, 0, 10), (0.6, 0.0, 10.0), (0.0, 0.0, 0.0))


def test_out_of_band_z():
    assert not in_settle_band((0, 0, 10), (0.0, 0.0, 10.4), (0.0, 0.0, 0.0))


def test_out_of_band_speed():
    assert not in_settle_band((0, 0, 10), (0.0, 0.0, 10.0), (0.3, 0.0, 0.0))


def test_dwell_requires_continuous_time():
    w = SettleWatch(dwell_s=0.5)
    assert not w.update(True, 0.2)
    assert not w.update(True, 0.2)
    assert w.update(True, 0.2)
    assert w.accumulated >= 0.5


def test_dwell_resets_when_leaving_band():
    w = SettleWatch(dwell_s=0.5)
    assert not w.update(True, 0.4)
    assert not w.update(False, 0.1)
    assert w.accumulated == 0.0
    assert not w.update(True, 0.4)
    assert w.update(True, 0.2)


def test_traj_time_alone_is_not_settled():
    # Physical settling is separate from planned duration (manager enforces).
    assert in_settle_band((5, 0, 10), (5.0, 0.0, 10.0), (0.0, 0.0, 0.0))
    assert not in_settle_band((5, 0, 10), (4.0, 0.0, 10.0), (0.5, 0.0, 0.0))
