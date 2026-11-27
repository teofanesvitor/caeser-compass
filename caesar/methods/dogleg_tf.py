"""Dogleg Toolface trajectory adapter."""
from __future__ import annotations
from math import acos, atan2, cos, degrees, radians, sin
from .base import build_segment


def solve_tool_face(last_data: dict, values: dict) -> tuple[float, float]:
    """Compute final INC/AZI from toolface, DLS and measured-depth extension."""
    required = ("tool_face", "dls", "extension")
    missing = [key for key in required if key not in values]
    if missing:
        raise KeyError(f"values sem campos obrigatórios: {', '.join(missing)}")
    inc0, azi0 = radians(float(last_data["inclination"])), radians(float(last_data["azimuth"]))
    tf, dls, extension = radians(float(values["tool_face"])), float(values["dls"]), float(values["extension"])
    if extension <= 0 or dls < 0:
        raise ValueError("Extension deve ser positiva e DLS não pode ser negativo.")
    beta = radians(dls) * extension / 30.0
    inc_arg = cos(inc0) * cos(beta) - sin(inc0) * sin(beta) * cos(tf)
    inc = acos(max(-1.0, min(1.0, inc_arg)))
    azi = azi0 + atan2(sin(beta) * sin(tf),
                       sin(inc0) * cos(beta) + cos(inc0) * sin(beta) * cos(tf))
    return degrees(inc), degrees(azi) % 360.0


def calculate_trajectory(params: dict, progress=None) -> list[dict]:
    try:
        last_data, values = params["last_data"], params["values"]
    except KeyError as exc:
        raise KeyError(f"Parâmetro obrigatório ausente: {exc.args[0]}") from exc
    inc, azi = solve_tool_face(last_data, values)
    final_md = float(last_data["depth"]) + float(values["extension"])
    return build_segment(last_data, final_md, inc, azi, "Dogleg Toolface")


def solve(last_data: dict, target: dict, alignment_target=None, *,
          tool_face: float, dls: float, extension: float) -> list[dict]:
    """Expose Dogleg Toolface through the multi-target solver contract."""
    return calculate_trajectory({
        "last_data": last_data,
        "values": {"tool_face": tool_face, "dls": dls, "extension": extension},
    })
