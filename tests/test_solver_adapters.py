import pytest

from caesar.planner.models import PlanningConstraints
from caesar.planner import solver_adapters as adapters


def test_optimum_align_adapter_forwards_configuration_and_next_target(monkeypatch):
    captured = {}
    expected = [{"depth": 10.0}]
    def fake(last_data, target, alignment_target, **kwargs):
        captured.update(last_data=last_data, target=target,
                        alignment_target=alignment_target, **kwargs)
        return expected
    monkeypatch.setattr(adapters, "solve_optimum_align", fake)
    solver = adapters.optimum_align_solver(
        dogleg1=2.0, dogleg2=3.0, project_back=[{"method": "extension"}]
    )
    result = solver({"depth": 1}, {"tvd": 2}, {"tvd": 3})
    assert result is expected
    assert captured["dogleg1"] == 2.0
    assert captured["dogleg2"] == 3.0
    assert captured["alignment_target"] == {"tvd": 3}
    assert captured["project_back"] == [{"method": "extension"}]


def test_slant_adapter_returns_selected_candidate_points(monkeypatch):
    points = [{"depth": 100.0}, {"depth": 200.0}]
    captured = {}
    def fake(**kwargs):
        captured.update(kwargs)
        return {"best": type("Candidate", (), {"points": points})()}
    monkeypatch.setattr(adapters, "optimize_slant_hold_lengths", fake)
    constraints = PlanningConstraints(max_dls=3.0)
    solver = adapters.slant_solver(
        method_flags={"kop": True, "dls": False, "inclination": False, "hold_len": False},
        values={"kop": 120.0}, hold_lengths=[50.0, 75.0], constraints=constraints,
    )
    result = solver({"depth": 100.0}, {"tvd": 200.0}, None)
    assert result == points
    assert captured["method_flags"]["kop"] is True
    assert captured["hold_lengths"] == (50.0, 75.0)
    assert captured["constraints"] is constraints


def test_nudge_adapter_binds_md_inc_azi(monkeypatch):
    captured = {}
    def fake(last_data, target, alignment_target, **kwargs):
        captured.update(kwargs)
        return [last_data]
    monkeypatch.setattr(adapters, "solve_nudge", fake)
    solver = adapters.nudge_solver(final_md=1500, final_inc=25, final_azi=90)
    solver({}, {}, {})
    assert captured == {"final_md": 1500, "final_inc": 25, "final_azi": 90}


def test_dogleg_toolface_adapter_binds_original_payload(monkeypatch):
    captured = {}
    def fake(params):
        captured.update(params)
        return [{"depth": 100.0}]
    monkeypatch.setattr(adapters, "calculate_dogleg_toolface", fake)
    solver = adapters.dogleg_toolface_solver(tool_face=90, dls=3, extension=30)
    solver({"depth": 100.0}, {}, None)
    assert captured["values"] == {"tool_face": 90, "dls": 3, "extension": 30}


def test_dogleg_curve_hold_adapter_forwards_dls_and_options(monkeypatch):
    captured = {}
    def fake(last_data, target, alignment_target, *, dls, **kwargs):
        captured.update(dls=dls, **kwargs)
        return [last_data, target]
    monkeypatch.setattr(adapters, "solve_dogleg_curve_hold", fake)
    solver = adapters.dogleg_curve_hold_solver(dls=2.5, position_tolerance=0.01)
    solver({}, {}, {},)
    assert captured == {"dls": 2.5, "position_tolerance": 0.01}
