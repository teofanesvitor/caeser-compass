import pytest

from caesar.methods import optimum_align


def test_optimum_align_adapter_uses_next_target_direction(monkeypatch):
    start = {
        "depth": 1000.0, "inclination": 10.0, "azimuth": 20.0,
        "verticalDepth": 900.0, "ns": 0.0, "ew": 0.0,
    }
    target = {"name": "T1", "tvd": 1200.0, "ns": 100.0, "ew": 0.0}
    following = {"name": "T2", "tvd": 1500.0, "ns": 250.0, "ew": 50.0}
    captured = {}

    def fake_calculate(params, progress):
        captured.update(params)
        return [{"depth": 1400.0, "inclination": 1.0, "azimuth": 2.0}]

    monkeypatch.setattr(optimum_align, "calculate_optimum_align", fake_calculate)
    result = optimum_align.solve_optimum_align(
        start, target, following, dogleg1=2.0, dogleg2=2.0
    )

    plan = captured["plan"]
    assert plan["inclination"] == pytest.approx(27.818, abs=0.01)
    assert plan["azimuth"] == pytest.approx(18.435, abs=0.01)
    assert result[0] == start
    assert result[1]["depth"] == 1400.0


def test_final_target_requires_explicit_orientation():
    start = {
        "depth": 1000.0, "inclination": 10.0, "azimuth": 20.0,
        "verticalDepth": 900.0, "ns": 0.0, "ew": 0.0,
    }
    with pytest.raises(ValueError, match="último alvo"):
        optimum_align.solve_optimum_align(
            start, {"name": "Tfinal", "tvd": 1200, "ns": 100, "ew": 0},
            dogleg1=2, dogleg2=2
        )
