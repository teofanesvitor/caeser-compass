import json

import pytest

from caesar.planner import json_runner


def test_numeric_lists_and_ranges_expand_cartesian_product():
    variants = list(json_runner._expand_parameters({
        "dls": {"min": 1.0, "max": 2.0, "step": 0.5},
        "extension": [100.0, 200.0],
        "fixed": 7.0,
    }))
    assert len(variants) == 6
    assert variants[0][0] == {"dls": 1.0, "extension": 100.0, "fixed": 7.0}
    assert variants[-1][0] == {"dls": 2.0, "extension": 200.0, "fixed": 7.0}


def test_slant_hold_lengths_are_expanded_as_independent_candidates():
    constraints = json_runner.PlanningConstraints(max_dls=3.0)
    intermediate, terminal, expanded, _ = json_runner._build_method_pool(
        {
            "Slant": {
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
        },
        constraints,
    )
    assert len(intermediate) == 3
    assert len(terminal) == 3
    assert len(expanded["Slant"]) == 3
    assert all("HOLD_LEN=" in name for name in intermediate)


def test_auto_role_routes_curve_hold_only_to_terminal_pool():
    assert json_runner._method_roles("dogleg_curve_hold", "auto") == {"terminal"}
    assert json_runner._method_roles("optimum_align", "auto") == {"intermediate", "terminal"}


def test_method_pool_expands_parameter_ranges_and_external_kop():
    constraints = json_runner.PlanningConstraints(max_dls=3.0)
    intermediate, terminal, expanded, kops = json_runner._build_method_pool(
        {
            "Optimum Align": {
                "type": "optimum_align",
                "params": {
                    "dogleg1": [1.0, 2.0],
                    "dogleg2": 2.0,
                },
            },
            "Dogleg Curve-Hold": {
                "type": "dogleg_curve_hold",
                "params": {"dls": {"min": 2.0, "max": 3.0, "step": 0.5}},
            },
            "Hold": {"type": "hold", "params": {}},
        },
        constraints,
        kop_by_method={"Optimum Align": {"min": 1000.0, "max": 1100.0, "step": 100.0}},
    )
    assert len(expanded["Optimum Align"]) == 4
    assert len(kops) == 4
    assert set(intermediate) == set(expanded["Optimum Align"]) | {"Hold"}
    assert set(terminal) == set(expanded["Optimum Align"]) | {
        "Dogleg Curve-Hold [params.dls=2]",
        "Dogleg Curve-Hold [params.dls=2.5]",
        "Dogleg Curve-Hold [params.dls=3]",
        "Hold",
    }


def test_candidate_limit_prevents_accidental_combinatorial_explosion():
    constraints = json_runner.PlanningConstraints(max_dls=3.0)
    with pytest.raises(ValueError, match="mais de 2 candidatos"):
        json_runner._build_method_pool(
            {
                "Optimum Align": {
                    "type": "optimum_align",
                    "params": {"dogleg1": [1.0, 2.0, 3.0], "dogleg2": 2.0},
                },
            },
            constraints,
            max_candidates=2,
        )


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
