"""Sequential multi-target trajectory planning.

Each intermediate target is intercepted while the solver receives the
following target as its alignment reference. All feasible branches are kept
so a locally short segment cannot discard a globally shorter complete path.
"""
from __future__ import annotations

from dataclasses import dataclass, field
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
    """Plans ordered targets and passes the next target as alignment reference.

    Solver contract:
        solve(last_data, target, alignment_target) -> ordered trajectory points

    alignment_target is the next target for intermediate targets. For the
    final target it is None; any final orientation constraint stays in target.
    """

    def __init__(self, constraints: PlanningConstraints):
        self.constraints = constraints

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
            alignment_target = (
                ordered_targets[index + 1].as_solver_input()
                if index + 1 < len(ordered_targets) else None
            )
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
