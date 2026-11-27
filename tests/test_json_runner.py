import json

from caesar.planner import json_runner


def test_slant_hold_lengths_are_expanded_as_independent_global_candidates():
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
