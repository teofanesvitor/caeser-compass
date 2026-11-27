"""Shared utilities for trajectory construction methods."""
from __future__ import annotations

from typing import Protocol

from caesar.core.trajectory import WellTrajectory


class TrajectoryMethod(Protocol):
    name: str

    def solve(self, last_data: dict, target: dict, **parameters) -> list[dict]:
        """Return ordered trajectory points, including the initial state."""
        ...


def build_segment(last_data: dict, final_md: float, final_inc: float, final_azi: float, section_type: str) -> list[dict]:
    """Build one minimum-curvature segment in the shared planner schema."""
    required = ("depth", "inclination", "azimuth", "verticalDepth", "ns", "ew")
    missing = [key for key in required if key not in last_data]
    if missing:
        raise KeyError(f"last_data sem campos obrigatórios: {', '.join(missing)}")

    md0, md1 = float(last_data["depth"]), float(final_md)
    inc0, azi0 = float(last_data["inclination"]), float(last_data["azimuth"]) % 360.0
    inc1, azi1 = float(final_inc), float(final_azi) % 360.0
    if md1 <= md0:
        raise ValueError("O MD final deve ser maior que o MD inicial.")
    if not 0.0 <= inc1 <= 180.0:
        raise ValueError("A inclinação final deve estar entre 0 e 180 graus.")

    trajectory = WellTrajectory(
        [md0, md1], [inc0, inc1], [azi0, azi1],
        tvd_offset=float(last_data["verticalDepth"]),
        north_offset=float(last_data["ns"]),
        east_offset=float(last_data["ew"]),
        vs_azimuth=last_data.get("vs_azimuth"),
    )
    survey = trajectory.results()
    start = dict(last_data)
    start.setdefault("extension", 0.0)
    end = {
        "depth": md1, "extension": md1 - md0,
        "inclination": inc1, "azimuth": azi1,
        "verticalDepth": float(survey["TVD"][1]),
        "ns": float(survey["North"][1]), "ew": float(survey["East"][1]),
        "displacement": float(survey["Horizontal Displacement"][1]),
        "dls": float(survey["DLS (deg/30m)"][1]),
        "build": float(survey["Build (deg/30m)"][1]),
        "turn": float(survey["Turn (deg/30m)"][1]),
        "toolface": float(survey["Toolface (deg)"][1]),
        "sectionType": section_type, "target": "",
    }
    return [start, end]
