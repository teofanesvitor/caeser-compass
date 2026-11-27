"""Shared planning models."""
from dataclasses import dataclass, field
from typing import Callable


@dataclass(frozen=True)
class PlanningConstraints:
    max_dls: float
    max_inclination: float = 90.0
    position_tolerance: float = 1e-3
    minimum_extension: float = 0.0
    maximum_extension: float | None = None


@dataclass
class CandidateResult:
    method: str
    points: list[dict]
    valid: bool
    reasons: list[str] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    @property
    def final_md(self) -> float:
        return float(self.points[-1]["depth"]) if self.points else float("inf")


Validator = Callable[[list[dict], PlanningConstraints], list[str]]
