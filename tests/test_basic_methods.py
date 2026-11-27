import pytest

from caesar.methods.hold import calculate_hold
from caesar.methods.nudge import calculate_trajectory as calculate_nudge
from caesar.methods.dogleg_tf import calculate_trajectory as calculate_dogleg_toolface


START = {
    "depth": 1000.0,
    "inclination": 0.0,
    "azimuth": 0.0,
    "verticalDepth": 1000.0,
    "ns": 0.0,
    "ew": 0.0,
    "displacement": 0.0,
}


def test_hold_extends_current_tangent():
    points = calculate_hold(START, 1100.0)
    assert len(points) == 2
    assert points[0]["depth"] == 1000.0
    assert points[-1]["depth"] == 1100.0
    assert points[-1]["verticalDepth"] == pytest.approx(1100.0)
    assert points[-1]["ns"] == pytest.approx(0.0)
    assert points[-1]["ew"] == pytest.approx(0.0)
    assert points[-1]["dls"] == pytest.approx(0.0)


def test_nudge_md_inc_azi_builds_segment():
    points = calculate_nudge({
        "last_data": START,
        "method": "MdIncAzi",
        "constraints": {"final_md": 1100.0, "final_inc": 10.0, "final_azi": 90.0},
    })
    endpoint = points[-1]
    assert endpoint["depth"] == 1100.0
    assert endpoint["inclination"] == 10.0
    assert endpoint["azimuth"] == 90.0
    assert endpoint["ew"] > 0.0
    assert endpoint["verticalDepth"] < 1100.0


def test_nudge_rejects_unported_modes_explicitly():
    with pytest.raises(NotImplementedError):
        calculate_nudge({
            "last_data": START,
            "method": "curve",
            "constraints": {},
        })


def test_dogleg_toolface_builds_expected_vertical_build():
    points = calculate_dogleg_toolface({
        "last_data": START,
        "values": {"tool_face": 90.0, "dls": 3.0, "extension": 30.0},
    })
    endpoint = points[-1]
    assert endpoint["depth"] == 1030.0
    assert endpoint["inclination"] == pytest.approx(3.0, abs=1e-8)
    assert endpoint["azimuth"] == pytest.approx(90.0, abs=1e-8)
    assert endpoint["dls"] == pytest.approx(3.0, abs=1e-7)


def test_hold_rejects_non_increasing_md():
    with pytest.raises(ValueError):
        calculate_hold(START, 1000.0)


def test_hold_solver_projects_target_onto_current_tangent():
    from caesar.methods.hold import solve

    target = {"tvd": 1200.0, "ns": 0.0, "ew": 0.0}
    points = solve(START, target)
    assert points[-1]["depth"] == pytest.approx(1200.0)
    assert points[-1]["verticalDepth"] == pytest.approx(target["tvd"])


def test_hold_solver_rejects_target_off_tangent_by_projection():
    from caesar.methods.hold import solve

    target = {"tvd": 1200.0, "ns": 20.0, "ew": 0.0}
    points = solve(START, target)
    # The candidate is projected onto the tangent; the planner's positional
    # evaluator is responsible for rejecting the 20 m cross-track miss.
    assert points[-1]["ns"] == pytest.approx(0.0)
    assert points[-1]["verticalDepth"] == pytest.approx(1200.0)
