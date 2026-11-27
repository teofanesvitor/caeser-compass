"""Constant-orientation Hold section."""
from __future__ import annotations
from math import cos, radians, sin
from .base import build_segment


def calculate_hold(last_data: dict, final_md: float) -> list[dict]:
    """Extend the current tangent to an absolute measured depth."""
    return build_segment(last_data, final_md, last_data["inclination"],
                         last_data["azimuth"], "Hold")


def solve(last_data: dict, target: dict, alignment_target=None) -> list[dict]:
    """Project a target onto the current tangent and return a Hold candidate."""
    required = ("tvd", "ns", "ew")
    missing = [key for key in required if key not in target]
    if missing:
        raise KeyError(f"target sem campos obrigatórios: {', '.join(missing)}")

    inc, azi = radians(float(last_data["inclination"])), radians(float(last_data["azimuth"]))
    direction = (cos(inc), sin(inc) * cos(azi), sin(inc) * sin(azi))
    delta = (
        float(target["tvd"]) - float(last_data["verticalDepth"]),
        float(target["ns"]) - float(last_data["ns"]),
        float(target["ew"]) - float(last_data["ew"]),
    )
    extension = sum(a * b for a, b in zip(delta, direction))
    if extension <= 0:
        raise ValueError("O alvo não está à frente do ponto inicial na direção do Hold.")
    return calculate_hold(last_data, float(last_data["depth"]) + extension)
