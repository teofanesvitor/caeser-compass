import pytest
from caesar.planner.kop import prepare_last_data_at_kop
from caesar.planner.models import PlanningConstraints
from caesar.planner.multi_target import MultiTargetPlanner, Target

def test_prepare_last_data_at_kop_projects_hold_in_3d():
    start = {"depth": 1000.0, "inclination": 30.0, "azimuth": 90.0,
             "verticalDepth": 900.0, "ns": 10.0, "ew": 20.0}
    result = prepare_last_data_at_kop(start, 1120.0)
    assert result["depth"] == pytest.approx(1120)
    assert result["verticalDepth"] == pytest.approx(900 + 120 * 3**0.5 / 2)
    assert result["ns"] == pytest.approx(10)
    assert result["ew"] == pytest.approx(80)
    assert result["sectionType"] == "Hold to KOP"

def test_kop_is_applied_before_solver_and_included_in_path():
    start = {"depth": 1000.0, "inclination": 0.0, "azimuth": 0.0,
             "verticalDepth": 1000.0, "ns": 0.0, "ew": 0.0, "dls": 0.0}
    target = Target("T1", 1300.0, 0.0, 0.0, inclination=0.0, azimuth=0.0)
    received = {}
    def solver(last_data, target, alignment_target):
        received.update(last_data)
        return [last_data, dict(last_data, depth=1300.0, verticalDepth=1300.0, dls=0.0)]
    result = MultiTargetPlanner(PlanningConstraints(max_dls=3)).plan(
        start, [target], {"Optimum Align": solver}, kop_by_method={"Optimum Align": 1100.0})
    assert received["depth"] == pytest.approx(1100)
    assert result["best"].points[1]["sectionType"] == "Hold to KOP"
    assert result["best"].final_md == pytest.approx(1300)

def test_method_without_kop_receives_original_survey():
    start = {"depth": 1000.0, "inclination": 0.0, "azimuth": 0.0,
             "verticalDepth": 1000.0, "ns": 0.0, "ew": 0.0, "dls": 0.0}
    target = Target("T1", 1200.0, 0.0, 0.0, inclination=0.0, azimuth=0.0)
    received = {}
    def solver(last_data, target, alignment_target):
        received.update(last_data)
        return [last_data, dict(last_data, depth=1200.0, verticalDepth=1200.0)]
    MultiTargetPlanner(PlanningConstraints(max_dls=3)).plan(start, [target], {"Hold": solver})
    assert received["depth"] == pytest.approx(1000)

def test_invalid_kop_rejects_only_that_method():
    start = {"depth": 1000.0, "inclination": 0.0, "azimuth": 0.0,
             "verticalDepth": 1000.0, "ns": 0.0, "ew": 0.0, "dls": 0.0}
    target = Target("T1", 1200.0, 0.0, 0.0, inclination=0.0, azimuth=0.0)
    def solver(last_data, target, alignment_target):
        return [last_data, dict(last_data, depth=1200.0, verticalDepth=1200.0)]
    result = MultiTargetPlanner(PlanningConstraints(max_dls=3)).plan(
        start, [target], {"bad-kop": solver, "no-kop": solver}, kop_by_method={"bad-kop": 900.0})
    assert result["best"].methods == ("no-kop",)
    assert result["rejected"][0]["method"] == "bad-kop"
