"""Sequential multi-target trajectory planning with geometric alignment checks."""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, degrees, hypot, sqrt
from typing import Callable, Iterable, Mapping, Any

from .models import PlanningConstraints


@dataclass(frozen=True)
class Target:
    name: str
    tvd: float
    ns: float
    ew: float
    inclination: float | None = None
    azimuth: float | None = None
    tolerance: float | None = None

    def as_solver_input(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "name": self.name, "tvd": self.tvd, "ns": self.ns, "ew": self.ew
        }
        if self.inclination is not None:
            result["inclination"] = self.inclination
        if self.azimuth is not None:
            result["azimuth"] = self.azimuth
        if self.tolerance is not None:
            result["tolerance"] = self.tolerance
        return result


@dataclass
class MultiTargetCandidate:
    methods: tuple[str, ...]
    points: list[dict[str, Any]]
    reached_targets: list[str] = field(default_factory=list)

    @property
    def final_md(self) -> float:
        return float(self.points[-1]["depth"]) if self.points else float("inf")


Solver = Callable[..., Iterable[dict[str, Any]]]


def _position_error(point: Mapping[str, Any], target: Target) -> float:
    return max(
        abs(float(point["verticalDepth"]) - target.tvd),
        abs(float(point["ns"]) - target.ns),
        abs(float(point["ew"]) - target.ew),
    )


def _angle_difference(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def _required_alignment(current: Target, following: Target) -> tuple[float, float]:
    """Return tangent INC/AZI from current target toward the following target."""
    delta_tvd = following.tvd - current.tvd
    delta_ns = following.ns - current.ns
    delta_ew = following.ew - current.ew
    horizontal = hypot(delta_ns, delta_ew)
    distance = sqrt(horizontal**2 + delta_tvd**2)
    if distance <= 1e-12:
        raise ValueError(
            f"Alvos {current.name} e {following.name} possuem a mesma posição."
        )
    inclination = degrees(atan2(horizontal, delta_tvd))
    azimuth = degrees(atan2(delta_ew, delta_ns)) % 360.0 if horizontal > 1e-12 else 0.0
    return inclination, azimuth


def _validate_path(points: list[dict[str, Any]], constraints: PlanningConstraints) -> list[str]:
    reasons: list[str] = []
    for index, point in enumerate(points):
        dls = point.get("dls")
        inc = point.get("inclination")
        if dls is not None and abs(float(dls)) > constraints.max_dls + 1e-9:
            reasons.append(f"DLS acima do limite no station {index}")
        if inc is not None and float(inc) > constraints.max_inclination + 1e-9:
            reasons.append(f"inclinação acima do limite no station {index}")
    return reasons


class MultiTargetPlanner:
    """Plans ordered targets, requiring each intermediate target to aim at the next.

    Solver contract:
        solve(last_data, target, alignment_target) -> ordered trajectory points

    alignment_target is the next target for intermediate targets. The solver
    uses this as the Optimum Align orientation reference. The planner then
    independently verifies the resulting tangent direction.
    """

    def __init__(
        self,
        constraints: PlanningConstraints,
        alignment_tolerance_deg: float = 0.1,
    ):
        if alignment_tolerance_deg < 0:
            raise ValueError("A tolerância angular não pode ser negativa.")
        self.constraints = constraints
        self.alignment_tolerance_deg = float(alignment_tolerance_deg)

    def _validate_arrival_alignment(
        self, endpoint: Mapping[str, Any], target: Target, following: Target | None
    ) -> None:
        if following is not None:
            required_inc, required_azi = _required_alignment(target, following)
            description = f"alvo {target.name} em direção a {following.name}"
        elif target.inclination is not None or target.azimuth is not None:
            if target.inclination is None or target.azimuth is None:
                raise ValueError(
                    f"O alvo final {target.name} exige INC e AZI informados em conjunto."
                )
            required_inc, required_azi = target.inclination, target.azimuth
            description = f"orientação final do alvo {target.name}"
        else:
            return

        actual_inc = float(endpoint["inclination"])
        actual_azi = float(endpoint["azimuth"]) % 360.0
        inc_error = abs(actual_inc - required_inc)
        azi_error = _angle_difference(actual_azi, required_azi)
        tolerance = self.alignment_tolerance_deg
        if inc_error > tolerance or azi_error > tolerance:
            raise ValueError(
                f"alinhamento incorreto em {description}: "
                f"INC erro={inc_error:.4g}°, AZI erro={azi_error:.4g}° "
                f"(tolerância={tolerance:g}°)"
            )

    def plan(
        self,
        last_data: dict[str, Any],
        targets: Iterable[Target],
        solvers: Mapping[str, Solver],
    ) -> dict[str, Any]:
        ordered_targets = tuple(targets)
        if not ordered_targets:
            raise ValueError("Informe pelo menos um alvo.")
        if not solvers:
            raise ValueError("Informe pelo menos um método.")

        branches = [MultiTargetCandidate((), [dict(last_data)])]
        rejected: list[dict[str, str]] = []

        for index, target in enumerate(ordered_targets):
            following = ordered_targets[index + 1] if index + 1 < len(ordered_targets) else None
            alignment_target = following.as_solver_input() if following else None
            next_branches: list[MultiTargetCandidate] = []

            for branch in branches:
                start = dict(branch.points[-1])
                for method, solve in solvers.items():
                    try:
                        segment = [dict(p) for p in solve(
                            last_data=start,
                            target=target.as_solver_input(),
                            alignment_target=alignment_target,
                        )]
                        if not segment:
                            raise ValueError("método retornou trajetória vazia")
                        if float(segment[0]["depth"]) != float(start["depth"]):
                            raise ValueError("segmento não começa no MD do estado inicial")

                        combined = branch.points + segment[1:]
                        endpoint = segment[-1]
                        tolerance = (
                            target.tolerance if target.tolerance is not None
                            else self.constraints.position_tolerance
                        )
                        error = _position_error(endpoint, target)
                        if error > tolerance:
                            raise ValueError(
                                f"alvo {target.name} não atingido; erro máximo={error:.6g}"
                            )

                        self._validate_arrival_alignment(endpoint, target, following)
                        violations = _validate_path(combined, self.constraints)
                        if violations:
                            raise ValueError("; ".join(violations))

                        next_branches.append(MultiTargetCandidate(
                            methods=branch.methods + (method,),
                            points=combined,
                            reached_targets=branch.reached_targets + [target.name],
                        ))
                    except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError) as exc:
                        rejected.append({
                            "target": target.name, "method": method, "reason": str(exc)
                        })

            if not next_branches:
                raise ValueError(
                    f"Nenhuma trajetória viável após o alvo {target.name}. "
                    f"Rejeições: {rejected}"
                )
            branches = next_branches

        best = min(branches, key=lambda candidate: candidate.final_md)
        return {
            "best": best,
            "candidates": branches,
            "rejected": rejected,
            "target_order": [target.name for target in ordered_targets],
        }
