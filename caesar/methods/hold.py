"""Constant-orientation Hold section."""
from __future__ import annotations
from .base import build_segment


def calculate_hold(last_data: dict, final_md: float) -> list[dict]:
    """Extend the current tangent to an absolute measured depth."""
    return build_segment(last_data, final_md, last_data["inclination"],
                         last_data["azimuth"], "Hold")


def solve(last_data: dict, target: dict | None = None, *, final_md: float) -> list[dict]:
    return calculate_hold(last_data, final_md)
