from caesar.methods import slant
from caesar.planner.models import PlanningConstraints


def test_slant_optimizer_sweeps_hold_and_selects_shortest_valid(monkeypatch):
    start = {
        "depth": 1000.0, "inclination": 0.0, "azimuth": 0.0,
        "verticalDepth": 900.0, "ns": 0.0, "ew": 0.0,
    }
    target = {"tvd": 1100.0, "ns": 50.0, "ew": 0.0}
    received = []

    def fake_slant(params, progress):
        hold = params["values"]["hold_len"]
        received.append(hold)
        return [{
            "depth": 1300.0 - hold,
            "extension": 100.0,
            "inclination": 20.0,
            "azimuth": 0.0,
            "verticalDepth": target["tvd"],
            "ns": target["ns"],
            "ew": target["ew"],
            "dls": 2.0,
        }]

    monkeypatch.setattr(slant, "calculate_slant", fake_slant)
    result = slant.optimize_slant_hold_lengths(
        start,
        target,
        {"kop": True, "dls": False, "inclination": False, "hold_len": False},
        {"kop": 1050.0},
        [50.0, 100.0],
        PlanningConstraints(max_dls=3.0),
    )

    assert received == [50.0, 100.0]
    assert result["best"].final_md == 1200.0


def test_slant_optimizer_rejects_invalid_known_parameter_count():
    start = {
        "depth": 1000.0, "inclination": 0.0, "azimuth": 0.0,
        "verticalDepth": 900.0, "ns": 0.0, "ew": 0.0,
    }
    try:
        slant.optimize_slant_hold_lengths(
            start, {"tvd": 1100, "ns": 50, "ew": 0},
            {"kop": False, "dls": False, "inclination": False, "hold_len": False},
            {}, [50], PlanningConstraints(max_dls=3),
        )
    except ValueError as exc:
        assert "exatamente mais um" in str(exc)
    else:
        raise AssertionError("Era esperado rejeitar a quantidade de parâmetros.")
