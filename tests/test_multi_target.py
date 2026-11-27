import math
import pytest

from caesar.planner.models import PlanningConstraints
from caesar.planner.multi_target import MultiTargetPlanner, Target


def point(md, tvd, ns, ew, inc=10, azi=20, dls=1):
    return {
        "depth": md, "verticalDepth": tvd, "ns": ns, "ew": ew,
        "inclination": inc, "azimuth": azi, "dls": dls,
    }


def aimed_point(md, target, following):
    dns, dew = following.ns - target.ns, following.ew - target.ew
    dtvd = following.tvd - target.tvd
    horizontal = math.hypot(dns, dew)
    inc = math.degrees(math.atan2(horizontal, dtvd))
    azi = math.degrees(math.atan2(dew, dns)) % 360
    return point(md, target.tvd, target.ns, target.ew, inc, azi)


def test_intermediate_target_must_aim_at_following_target():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    first, second = Target("T1", 1200, 100, 0), Target("T2", 1500, 250, 50)
    received = []

    def solver(last_data, target, alignment_target):
        received.append(alignment_target)
        end = aimed_point(1300, first, second) if alignment_target else point(
            1400, second.tvd, second.ns, second.ew
        )
        return [last_data, end]

    result = planner.plan(start, [first, second], {"Optimum Align": solver})
    assert received[0]["name"] == "T2"
    assert received[1] is None
    assert result["best"].reached_targets == ["T1", "T2"]


def test_global_choice_uses_complete_final_md():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    t1, t2 = Target("T1", 1100, 50, 0), Target("T2", 1300, 100, 0)

    def route_a(last_data, target, alignment_target):
        if target["name"] == "T1":
            end = aimed_point(1100, t1, t2)
        else:
            end = point(last_data["depth"] + 500, t2.tvd, t2.ns, t2.ew)
        return [last_data, end]

    def route_b(last_data, target, alignment_target):
        if target["name"] == "T1":
            end = aimed_point(1200, t1, t2)
        else:
            end = point(last_data["depth"] + 300, t2.tvd, t2.ns, t2.ew)
        return [last_data, end]

    result = planner.plan(start, [t1, t2], {"A": route_a, "B": route_b})
    assert result["best"].final_md == 1400
    assert result["best"].methods == ("A", "B")


def test_rejects_missed_target():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)

    def miss(last_data, target, alignment_target):
        return [last_data, point(1200, 1100, 60, 0)]

    with pytest.raises(ValueError, match="Nenhuma trajetória viável"):
        planner.plan(start, [Target("T1", 1100, 50, 0)], {"miss": miss})


def test_rejects_target_reached_with_wrong_tangent():
    planner = MultiTargetPlanner(PlanningConstraints(max_dls=3))
    start = point(1000, 900, 0, 0)
    t1, t2 = Target("T1", 1100, 50, 0), Target("T2", 1300, 100, 0)

    def wrong_orientation(last_data, target, alignment_target):
        return [last_data, point(1200, t1.tvd, t1.ns, t1.ew, inc=10, azi=270)]

    with pytest.raises(ValueError, match="alinhamento incorreto"):
        planner.plan(start, [t1, t2], {"wrong": wrong_orientation})
