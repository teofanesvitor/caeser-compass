import pytest

from caesar.planner.models import PlanningConstraints
from caesar.planner.evaluator import evaluate_candidate, select_shortest


def point(md, tvd, ns, ew, dls=0, inc=0):
    return {"depth": md, "verticalDepth": tvd, "ns": ns, "ew": ew,
            "dls": dls, "inclination": inc, "extension": md}


def test_selects_shortest_valid_candidate():
    target = {"tvd": 100, "ns": 20, "ew": 0}
    limits = PlanningConstraints(max_dls=3)
    longer = evaluate_candidate("long", [point(300, 100, 20, 0)], target, limits)
    shorter = evaluate_candidate("short", [point(250, 100, 20, 0)], target, limits)
    assert select_shortest([longer, shorter]).method == "short"


def test_rejects_dls_violation():
    target = {"tvd": 100, "ns": 20, "ew": 0}
    result = evaluate_candidate("bad", [point(250, 100, 20, 0, dls=4)],
                                target, PlanningConstraints(max_dls=3))
    assert not result.valid
    assert any("DLS" in reason for reason in result.reasons)
