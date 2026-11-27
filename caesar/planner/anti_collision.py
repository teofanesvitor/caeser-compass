"""Geometric anti-collision screening for directional well trajectories.

This module performs deterministic 3D polyline clearance checks. It is not an
ISCWSA uncertainty model and must not be interpreted as a Separation Factor.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Any, Iterable, Mapping

_POSITION_KEYS = ("verticalDepth", "ns", "ew")


def _xyz(point: Mapping[str, Any]) -> tuple[float, float, float]:
    try:
        values = tuple(float(point[key]) for key in _POSITION_KEYS)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Cada station precisa de verticalDepth, ns e ew.") from exc
    if not all(isfinite(value) for value in values):
        raise ValueError("Coordenadas de survey devem ser finitas.")
    return values


def _md(point: Mapping[str, Any]) -> float:
    try:
        value = float(point["depth"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Cada station precisa do MD em 'depth'.") from exc
    if not isfinite(value):
        raise ValueError("MD deve ser finito.")
    return value


def _closest_segments(p1, q1, p2, q2):
    """Closest points on two finite 3D segments; returns (distance, s, t)."""
    u = tuple(q1[i] - p1[i] for i in range(3))
    v = tuple(q2[i] - p2[i] for i in range(3))
    w = tuple(p1[i] - p2[i] for i in range(3))
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    a, b, c = dot(u, u), dot(u, v), dot(v, v)
    d, e = dot(u, w), dot(v, w)
    eps = 1e-12

    if a <= eps and c <= eps:
        s = t = 0.0
    elif a <= eps:
        s, t = 0.0, min(1.0, max(0.0, e / c))
    elif c <= eps:
        t, s = 0.0, min(1.0, max(0.0, -d / a))
    else:
        denom = a * c - b * b
        s = min(1.0, max(0.0, (b * e - c * d) / denom)) if denom > eps else 0.0
        t = (b * s + e) / c
        if t < 0.0:
            t = 0.0
            s = min(1.0, max(0.0, -d / a))
        elif t > 1.0:
            t = 1.0
            s = min(1.0, max(0.0, (b - d) / a))

    c1 = tuple(p1[i] + s * u[i] for i in range(3))
    c2 = tuple(p2[i] + t * v[i] for i in range(3))
    distance = sum((c1[i] - c2[i]) ** 2 for i in range(3)) ** 0.5
    return distance, s, t


@dataclass(frozen=True)
class ReferenceWell:
    """Reference well survey in the same coordinate frame as the planned well."""
    name: str
    survey: tuple[Mapping[str, Any], ...]
    coordinate_system: str
    vertical_datum: str

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ReferenceWell":
        return cls(
            name=str(data["name"]),
            survey=tuple(data["survey"]),
            coordinate_system=str(data["coordinate_system"]),
            vertical_datum=str(data["vertical_datum"]),
        )

    def validate(self) -> None:
        if not self.name.strip():
            raise ValueError("Poço de referência precisa de identificação.")
        if len(self.survey) < 2:
            raise ValueError(f"{self.name}: survey precisa de pelo menos dois stations.")
        mds = [_md(point) for point in self.survey]
        if any(b <= a for a, b in zip(mds, mds[1:])):
            raise ValueError(f"{self.name}: MD deve ser estritamente crescente.")
        for point in self.survey:
            _xyz(point)


@dataclass(frozen=True)
class AntiCollisionConfig:
    minimum_clearance_m: float
    coordinate_system: str
    vertical_datum: str

    def __post_init__(self):
        if not isfinite(float(self.minimum_clearance_m)) or self.minimum_clearance_m < 0:
            raise ValueError("minimum_clearance_m deve ser finito e não negativo.")
        if not self.coordinate_system or not self.vertical_datum:
            raise ValueError("Informe coordinate_system e vertical_datum.")


class AntiCollisionChecker:
    """Hard geometric-clearance constraint against any number of reference wells."""

    def __init__(self, config: AntiCollisionConfig, reference_wells: Iterable[ReferenceWell]):
        self.config = config
        self.reference_wells = tuple(reference_wells)
        names = [well.name for well in self.reference_wells]
        if len(names) != len(set(names)):
            raise ValueError("Os nomes dos poços de referência devem ser únicos.")
        for well in self.reference_wells:
            well.validate()
            if well.coordinate_system != config.coordinate_system:
                raise ValueError(
                    f"{well.name}: sistema de coordenadas incompatível "
                    f"({well.coordinate_system!r} != {config.coordinate_system!r})."
                )
            if well.vertical_datum != config.vertical_datum:
                raise ValueError(
                    f"{well.name}: datum vertical incompatível "
                    f"({well.vertical_datum!r} != {config.vertical_datum!r})."
                )

    def evaluate(self, candidate: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
        planned = tuple(candidate)
        if len(planned) < 2:
            raise ValueError("Trajetória planejada precisa de pelo menos dois stations.")
        planned_md = [_md(point) for point in planned]
        if any(b <= a for a, b in zip(planned_md, planned_md[1:])):
            raise ValueError("MD da trajetória planejada deve ser estritamente crescente.")
        planned_xyz = [_xyz(point) for point in planned]

        closest = None
        for well in self.reference_wells:
            survey = well.survey
            for i in range(len(planned) - 1):
                for j in range(len(survey) - 1):
                    distance, s, t = _closest_segments(
                        planned_xyz[i], planned_xyz[i + 1],
                        _xyz(survey[j]), _xyz(survey[j + 1]),
                    )
                    if closest is None or distance < closest["distance_m"]:
                        closest = {
                            "reference_well": well.name,
                            "distance_m": distance,
                            "planned_md": planned_md[i] + s * (planned_md[i + 1] - planned_md[i]),
                            "reference_md": _md(survey[j]) + t * (_md(survey[j + 1]) - _md(survey[j])),
                        }

        if closest is None:
            return {
                "checked": False,
                "valid": True,
                "reason": "Nenhum poço de referência informado.",
                "minimum_clearance_m": self.config.minimum_clearance_m,
                "uncertainty_evaluated": False,
            }

        closest["checked"] = True
        closest["minimum_clearance_m"] = float(self.config.minimum_clearance_m)
        closest["valid"] = closest["distance_m"] >= self.config.minimum_clearance_m
        closest["uncertainty_evaluated"] = False
        closest["method"] = "3D piecewise-linear survey clearance"
        if not closest["valid"]:
            closest["reason"] = (
                f"afastamento de {closest['distance_m']:.3f} m abaixo do mínimo "
                f"de {self.config.minimum_clearance_m:.3f} m em relação ao poço "
                f"{closest['reference_well']}."
            )
        return closest
