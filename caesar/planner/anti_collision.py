"""Conservative 3D geometric anti-collision screening.

Minimum Curvature arcs are sampled adaptively. The closest distance is
computed between finite 3D chords, then reduced by a geometric sampling
error bound before clearance is accepted. This is not an ISCWSA/SF model.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import acos, ceil, cos, isfinite, pi, sin, sqrt
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


def _direction(point: Mapping[str, Any]):
    try:
        inc, azi = float(point["inclination"]), float(point["azimuth"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("INC/AZI devem ser informados em conjunto.") from exc
    if not isfinite(inc) or not isfinite(azi) or not 0 <= inc <= 180:
        raise ValueError("INC/AZI devem ser finitos e INC deve estar entre 0° e 180°.")
    inc, azi = inc * pi / 180.0, azi * pi / 180.0
    return (cos(inc), sin(inc) * cos(azi), sin(inc) * sin(azi))


def _survey_mode(survey) -> str:
    flags = [("inclination" in p and "azimuth" in p) for p in survey]
    partial = [("inclination" in p) != ("azimuth" in p) for p in survey]
    if any(partial):
        raise ValueError("INC e AZI devem ser informados em conjunto em cada station.")
    if any(flags) and not all(flags):
        raise ValueError("Survey não pode misturar stations com e sem INC/AZI.")
    return "minimum_curvature" if all(flags) else "linear_chord"


def _segment_steps(length: float, beta: float, max_step_m: float,
                   max_chord_error_m: float) -> int:
    steps = max(1, int(ceil(length / max_step_m)))
    if beta <= 1e-12:
        return steps
    radius = length / beta
    ratio = min(1.0, max(0.0, max_chord_error_m / radius))
    max_angle = 2.0 * acos(max(-1.0, min(1.0, 1.0 - ratio)))
    if max_angle > 1e-12:
        steps = max(steps, int(ceil(beta / max_angle)))
    return steps


def _densify(survey, max_step_m: float, coordinate_tolerance_m: float,
             max_chord_error_m: float = 0.02):
    """Return (MD, XYZ) samples; retain every original station boundary."""
    mode = _survey_mode(survey)
    result = []
    for index in range(len(survey) - 1):
        p0, p1 = survey[index], survey[index + 1]
        md0, md1 = _md(p0), _md(p1)
        length = md1 - md0
        if length <= 0:
            raise ValueError("MD deve ser estritamente crescente.")
        xyz0, xyz1 = _xyz(p0), _xyz(p1)
        beta = 0.0
        if mode == "minimum_curvature":
            u0, u1 = _direction(p0), _direction(p1)
            dot = max(-1.0, min(1.0, sum(a*b for a, b in zip(u0, u1))))
            beta = acos(dot)
            if pi - beta < 1e-7:
                raise ValueError("Dogleg próximo de 180°: arco Minimum Curvature é geometricamente ambíguo.")
        steps = _segment_steps(length, beta, max_step_m, max_chord_error_m)
        for k in range(steps + 1):
            if index and k == 0:
                continue
            f = k / steps
            if mode == "minimum_curvature":
                if beta <= 1e-12:
                    pos = tuple(xyz0[d] + length * f * u0[d] for d in range(3))
                else:
                    c0 = (cos((1.0-f)*beta) - cos(beta)) / (beta * sin(beta))
                    c1 = (1.0 - cos(f*beta)) / (beta * sin(beta))
                    pos = tuple(xyz0[d] + length * (c0*u0[d] + c1*u1[d]) for d in range(3))
            else:
                pos = tuple(xyz0[d] + f*(xyz1[d]-xyz0[d]) for d in range(3))
            result.append((md0 + f*length, pos))

    if mode == "minimum_curvature":
        for p0, p1 in zip(survey, survey[1:]):
            xyz0, xyz1 = _xyz(p0), _xyz(p1)
            length = _md(p1) - _md(p0)
            u0, u1 = _direction(p0), _direction(p1)
            beta = acos(max(-1.0, min(1.0, sum(x*y for x, y in zip(u0, u1)))))
            if beta <= 1e-12:
                predicted = tuple(xyz0[d] + length*u0[d] for d in range(3))
            else:
                factor = (1.0-cos(beta))/(beta*sin(beta))
                predicted = tuple(xyz0[d] + length*factor*(u0[d]+u1[d]) for d in range(3))
            residual = sqrt(sum((predicted[d]-xyz1[d])**2 for d in range(3)))
            if residual > coordinate_tolerance_m:
                raise ValueError(
                    f"Coordenadas do station MD={_md(p1):g} m incompatíveis com "
                    f"INC/AZI pela Minimum Curvature (resíduo={residual:.3f} m)."
                )
    return result, mode


def _closest_segments(p1, q1, p2, q2):
    """Exact closest points on two finite 3D segments; returns distance, s, t."""
    u = tuple(q1[i] - p1[i] for i in range(3))
    v = tuple(q2[i] - p2[i] for i in range(3))
    w = tuple(p1[i] - p2[i] for i in range(3))
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    a, b, c = dot(u, u), dot(u, v), dot(v, v)
    d, e = dot(u, w), dot(v, w)
    eps = 1e-14
    if a <= eps and c <= eps:
        s = t = 0.0
    elif a <= eps:
        s, t = 0.0, min(1.0, max(0.0, e/c))
    elif c <= eps:
        t, s = 0.0, min(1.0, max(0.0, -d/a))
    else:
        denom = a*c - b*b
        if denom <= eps*a*c:
            # Parallel/near-parallel: project one endpoint, then clamp/reproject.
            s = min(1.0, max(0.0, -d/a))
            t = min(1.0, max(0.0, (b*s+e)/c))
            s = min(1.0, max(0.0, (b*t-d)/a))
        else:
            s = min(1.0, max(0.0, (b*e-c*d)/denom))
            t = (b*s+e)/c
            if t < 0.0:
                t = 0.0
                s = min(1.0, max(0.0, -d/a))
            elif t > 1.0:
                t = 1.0
                s = min(1.0, max(0.0, (b-d)/a))
    c1 = tuple(p1[i] + s*u[i] for i in range(3))
    c2 = tuple(p2[i] + t*v[i] for i in range(3))
    return sqrt(sum((c1[i]-c2[i])**2 for i in range(3))), s, t


@dataclass(frozen=True)
class ReferenceWell:
    """Reference well survey in the same coordinate frame as the planned well."""
    name: str
    survey: tuple[Mapping[str, Any], ...]
    coordinate_system: str
    vertical_datum: str
    uncertainty: Mapping[str, Any] | None = None

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ReferenceWell":
        return cls(
            name=str(data["name"]), survey=tuple(data["survey"]),
            coordinate_system=str(data["coordinate_system"]),
            vertical_datum=str(data["vertical_datum"]),
            uncertainty=data.get("uncertainty"),
        )

    def validate(self) -> None:
        if not self.name.strip():
            raise ValueError("Poço de referência precisa de identificação.")
        if len(self.survey) < 2:
            raise ValueError(f"{self.name}: survey precisa de pelo menos dois stations.")
        mds = [_md(p) for p in self.survey]
        if any(b <= a for a, b in zip(mds, mds[1:])):
            raise ValueError(f"{self.name}: MD deve ser estritamente crescente.")
        for point in self.survey:
            _xyz(point)
        _survey_mode(self.survey)


@dataclass(frozen=True)
class AntiCollisionConfig:
    minimum_clearance_m: float
    coordinate_system: str
    vertical_datum: str
    max_step_m: float = 2.0
    coordinate_tolerance_m: float = 1.0
    max_chord_error_m: float = 0.02

    def __post_init__(self):
        for name, value, allow_zero in (
            ("minimum_clearance_m", self.minimum_clearance_m, True),
            ("max_step_m", self.max_step_m, False),
            ("coordinate_tolerance_m", self.coordinate_tolerance_m, True),
            ("max_chord_error_m", self.max_chord_error_m, False),
        ):
            if not isfinite(float(value)) or (value < 0 if allow_zero else value <= 0):
                qualifier = "não negativo" if allow_zero else "maior que zero"
                raise ValueError(f"{name} deve ser finito e {qualifier}.")
        if not self.coordinate_system or not self.vertical_datum:
            raise ValueError("Informe coordinate_system e vertical_datum.")


class AntiCollisionChecker:
    """Conservative hard-clearance constraint against any number of reference wells."""

    def __init__(self, config: AntiCollisionConfig, reference_wells: Iterable[ReferenceWell]):
        self.config = config
        self.reference_wells = tuple(reference_wells)
        if not self.reference_wells:
            raise ValueError("Informe pelo menos um poço de referência para ativar a anticolisão.")
        names = [well.name for well in self.reference_wells]
        if len(names) != len(set(names)):
            raise ValueError("Os nomes dos poços de referência devem ser únicos.")
        for well in self.reference_wells:
            well.validate()
            if well.coordinate_system != config.coordinate_system:
                raise ValueError(f"{well.name}: sistema de coordenadas incompatível.")
            if well.vertical_datum != config.vertical_datum:
                raise ValueError(f"{well.name}: datum vertical incompatível.")

    def evaluate(self, candidate: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
        planned = tuple(candidate)
        if len(planned) < 2:
            raise ValueError("Trajetória planejada precisa de pelo menos dois stations.")
        mds = [_md(p) for p in planned]
        if any(b <= a for a, b in zip(mds, mds[1:])):
            raise ValueError("MD da trajetória planejada deve ser estritamente crescente.")
        for point in planned:
            _xyz(point)

        planned_samples, planned_mode = _densify(
            planned, self.config.max_step_m, self.config.coordinate_tolerance_m,
            self.config.max_chord_error_m,
        )
        per_well = []
        for well in self.reference_wells:
            ref_samples, ref_mode = _densify(
                well.survey, self.config.max_step_m, self.config.coordinate_tolerance_m,
                self.config.max_chord_error_m,
            )
            closest = None
            for i in range(len(planned_samples)-1):
                md_a, xyz_a = planned_samples[i]
                md_b, xyz_b = planned_samples[i+1]
                for j in range(len(ref_samples)-1):
                    rmd_a, rxyz_a = ref_samples[j]
                    rmd_b, rxyz_b = ref_samples[j+1]
                    distance, s, t = _closest_segments(xyz_a, xyz_b, rxyz_a, rxyz_b)
                    if closest is None or distance < closest["distance_m"]:
                        closest = {
                            "reference_well": well.name,
                            "distance_m": distance,
                            "planned_md": md_a + s*(md_b-md_a),
                            "reference_md": rmd_a + t*(rmd_b-rmd_a),
                        }
            if closest is None:
                raise ValueError(f"Não foi possível avaliar a geometria de {well.name}.")
            # Each sampled arc is within max_chord_error_m of its chord.
            error_bound = self.config.max_chord_error_m * (
                int(planned_mode == "minimum_curvature")
                + int(ref_mode == "minimum_curvature")
            )
            closest.update({
                "clearance_lower_bound_m": max(0.0, closest["distance_m"] - error_bound),
                "sampling_error_bound_m": error_bound,
                "valid": closest["distance_m"] - error_bound >= self.config.minimum_clearance_m,
                "minimum_clearance_m": float(self.config.minimum_clearance_m),
                "planned_geometry": planned_mode,
                "reference_geometry": ref_mode,
                "max_sampling_step_m": float(self.config.max_step_m),
            })
            per_well.append(closest)

        per_well.sort(key=lambda item: item["clearance_lower_bound_m"])
        closest = per_well[0]
        uncertainty_provided = any(
            well.uncertainty is not None or any("uncertainty" in p for p in well.survey)
            for well in self.reference_wells
        ) or any("uncertainty" in p for p in planned)
        valid = all(item["valid"] for item in per_well)
        report = {
            **closest,
            "checked": True,
            "valid": valid,
            "uncertainty_provided": uncertainty_provided,
            "uncertainty_evaluated": False,
            "method": "adaptive Minimum Curvature chord approximation" if (
                planned_mode == "minimum_curvature" and
                all(item["reference_geometry"] == "minimum_curvature" for item in per_well)
            ) else "adaptive sampled survey geometry (linear fallback where INC/AZI unavailable)",
            "reference_wells_checked": len(per_well),
            "reference_results": per_well,
        }
        if not valid:
            violations = [item for item in per_well if not item["valid"]]
            report["reason"] = "; ".join(
                f"afastamento conservador de {item['clearance_lower_bound_m']:.3f} m "
                f"abaixo do mínimo de {self.config.minimum_clearance_m:.3f} m "
                f"em relação ao poço {item['reference_well']} "
                f"(distância amostrada={item['distance_m']:.3f} m, "
                f"MD planejado={item['planned_md']:.2f} m, "
                f"MD referência={item['reference_md']:.2f} m)"
                for item in violations
            )
        return report
