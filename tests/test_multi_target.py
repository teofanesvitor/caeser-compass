import pytest

from caesar.planner.models import PlanningConstraints
from caesar.planner.multi_target import MultiTargetPlanner, Target


def point(md, tvd, ns, ew, inc=10, azi=20, dls=1):
    return {
        "depth": md, "verticalDepth": tvd, "ns": ns, "ew": ew,
        "inclination": inc, "azimuth": azi, "dls": dls,
    }


def test_intermediate_target_is_passed_as_alignment_reference():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    first, second = Target("T1", 1200, 100, 0), Target("T2", 1500, 250, 50)
    received = []

    def solver(last_data, target, alignment_target):
        received.append(alignment_target)
        return [last_data, point(1300, target["tvd"], target["ns"], target["ew"])]

    result = planner.plan(start, [first, second], {"Optimum Align": solver})
    assert received[0]["name"] == "T2"
    assert received[1] is None
    assert result["best"].reached_targets == ["T1", "T2"]


def test_global_choice_uses_complete_final_md():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    targets = [Target("T1", 1100, 50, 0), Target("T2", 1300, 100, 0)]

    def route_a(last_data, target, alignment_target):
        if target["name"] == "T1":
            return [last_data, point(1100, 1100, 50, 0)]
        return [last_data, point(1600, 1300, 100, 0)]

    def route_b(last_data, target, alignment_target):
        if target["name"] == "T1":
            return [last_data, point(1200, 1100, 50, 0)]
        return [last_data, point(1400, 1300, 100, 0)]

    result = planner.plan(start, targets, {"A": route_a, "B": route_b})
    assert result["best"].final_md == 1400
    assert result["best"].methods == ("B", "B")


def test_rejects_missed_target():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)

    def miss(last_data, target, alignment_target):
        return [last_data, point(1200, 1100, 60, 0)]

    with pytest.raises(ValueError, match="Nenhuma trajetória viável"):
        planner.plan(start, [Target("T1", 1100, 50, 0)], {"miss": miss})
