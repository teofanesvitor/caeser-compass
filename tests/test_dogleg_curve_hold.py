import pytest

from caesar.methods.base import build_segment
from caesar.methods.dogleg_tf import solve_curve_hold


START = {
    "depth": 1000.0,
    "inclination": 0.0,
    "azimuth": 0.0,
    "verticalDepth": 1000.0,
    "ns": 0.0,
    "ew": 0.0,
    "displacement": 0.0,
}


def test_curve_hold_optimizes_single_target_using_dls():
    curve = build_segment(START, 1200.0, 20.0, 90.0, "reference curve")[-1]
    target_point = build_segment(
        curve, 1500.0, 20.0, 90.0, "reference hold"
    )[-1]
    target = {
        "name": "T1",
        "tvd": target_point["verticalDepth"],
        "ns": target_point["ns"],
        "ew": target_point["ew"],
    }

    points = solve_curve_hold(START, target, dls=3.0)

    assert len(points) == 3
    assert points[1]["sectionType"] == "Dogleg Toolface Curve"
    assert points[2]["sectionType"] == "Dogleg Toolface Hold"
    assert points[-1]["verticalDepth"] == pytest.approx(target["tvd"], abs=1e-3)
    assert points[-1]["ns"] == pytest.approx(target["ns"], abs=1e-3)
    assert points[-1]["ew"] == pytest.approx(target["ew"], abs=1e-3)
    assert points[1]["dls"] == pytest.approx(3.0, abs=1e-3)


def test_curve_hold_rejects_invalid_dls():
    with pytest.raises(ValueError):
        solve_curve_hold(
            START,
            {"tvd": 1400.0, "ns": 200.0, "ew": 100.0},
            dls=0.0,
        )
