import pytest

from caesar.planner.models import PlanningConstraints
from caesar.planner.multi_target import MultiTargetPlanner, Target


def point(md, tvd, ns, ew, inc=10, azi=20, dls=1):
    return {"depth": md, "verticalDepth": tvd, "ns": ns, "ew": ew,
            "inclination": inc, "azimuth": azi, "dls": dls}


def test_dedicated_terminal_solver_is_used_for_single_target():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    target = Target("T-final", 1100, 50, 0)
    calls = []

    def terminal(last_data, target, alignment_target):
        calls.append(alignment_target)
        return [last_data, point(1200, 1100, 50, 0)]

    result = planner.plan(start, [target], {}, terminal_solvers={"Dogleg TF Curve-Hold": terminal})
    assert calls == [None]
    assert result["best"].methods == ("Dogleg TF Curve-Hold",)


def test_terminal_solver_is_only_used_on_last_target():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    first, final = Target("T1", 1100, 50, 0), Target("T2", 1200, 100, 0)
    calls = []

    def intermediate(last_data, target, alignment_target):
        assert alignment_target["name"] == "T2"
        return [last_data, point(1100, 1100, 50, 0, inc=10, azi=20)]

    def terminal(last_data, target, alignment_target):
        calls.append(target["name"])
        return [last_data, point(1300, 1200, 100, 0)]

    result = planner.plan(start, [first, final], {"Optimum Align": intermediate},
                          terminal_solvers={"Dogleg TF Curve-Hold": terminal})
    assert calls == ["T2"]
    assert result["best"].methods == ("Optimum Align", "Dogleg TF Curve-Hold")


def test_rejects_physical_discontinuity_at_segment_start():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    target = Target("T1", 1100, 50, 0)

    def discontinuous(last_data, target, alignment_target):
        wrong_start = point(1000, 901, 0, 0)
        return [wrong_start, point(1200, 1100, 50, 0)]

    with pytest.raises(ValueError, match="descontinuidade de posição"):
        planner.plan(start, [target], {"bad": discontinuous})
