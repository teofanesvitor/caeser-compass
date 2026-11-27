import json

import pytest

from caesar.planner import json_runner


def test_slant_hold_lengths_are_expanded_as_independent_candidates():
    constraints = json_runner.PlanningConstraints(max_dls=3.0)
    solvers = json_runner._build_solver(
        "Slant",
        {
            "type": "slant",
            "params": {
                "method_flags": {
                    "kop": True, "dls": False,
                    "inclination": False, "hold_len": False,
                },
                "values": {"kop": 1200.0},
                "hold_lengths": [50.0, 75.0, 100.0],
            },
        },
        constraints,
    )
    assert list(solvers) == [
        "Slant [HOLD_LEN=50]",
        "Slant [HOLD_LEN=75]",
        "Slant [HOLD_LEN=100]",
    ]


def test_auto_role_routes_curve_hold_only_to_terminal_pool():
    assert json_runner._method_roles("dogleg_curve_hold", "auto") == {"terminal"}
    assert json_runner._method_roles("optimum_align", "auto") == {"intermediate", "terminal"}


def test_method_pool_builds_intermediate_and_terminal_candidates():
    constraints = json_runner.PlanningConstraints(max_dls=3.0)
    intermediate, terminal, expanded = json_runner._build_method_pool(
        {
            "Optimum Align": {
                "type": "optimum_align",
                "params": {"dogleg1": 2.0, "dogleg2": 2.0},
            },
            "Dogleg Curve-Hold": {
                "type": "dogleg_curve_hold",
                "params": {"dls": 2.5},
            },
            "Hold": {"type": "hold", "params": {}},
        },
        constraints,
    )
    assert set(intermediate) == {"Optimum Align", "Hold"}
    assert set(terminal) == {"Optimum Align", "Dogleg Curve-Hold", "Hold"}
    assert expanded["Optimum Align"] == ["Optimum Align"]


def test_invalid_role_is_rejected():
    with pytest.raises(ValueError, match="role inválido"):
        json_runner._method_roles("hold", "somewhere")


def test_json_safe_converts_tuples_and_nested_values():
    converted = json_runner._json_safe({
        "methods": ("Optimum Align", "Hold"),
        "points": [{"depth": 1200.0}],
    })
    assert converted == {
        "methods": ["Optimum Align", "Hold"],
        "points": [{"depth": 1200.0}],
    }
    json.dumps(converted, allow_nan=False)
