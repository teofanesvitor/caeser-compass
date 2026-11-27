import pytest

from caesar.core.trajectory import WellTrajectory, target_reached


def test_vertical_hold_minimum_curvature():
    well = WellTrajectory([0, 100], [0, 0], [0, 0])
    assert well.tvd[-1] == pytest.approx(100)
    assert well.north[-1] == pytest.approx(0)
    assert well.east[-1] == pytest.approx(0)
    assert well.dls[-1] == pytest.approx(0)


def test_initial_offsets_are_applied():
    well = WellTrajectory([0, 100], [0, 0], [0, 0],
                          tvd_offset=120, north_offset=10, east_offset=-5)
    assert well.tvd[0] == 120
    assert well.north[0] == 10
    assert well.east[0] == -5


def test_target_tolerance():
    point = {"verticalDepth": 100.0001, "ns": 20, "ew": -10}
    target = {"tvd": 100, "ns": 20, "ew": -10}
    assert target_reached(point, target, tolerance=1e-3)
