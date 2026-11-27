"""Sequential multi-target trajectory planning with geometric alignment checks."""
from __future__ import annotations
from dataclasses import dataclass, field
from math import atan2, degrees, hypot, sqrt
from typing import Callable, Iterable, Mapping, Any
from .models import PlanningConstraints
from .kop import prepare_last_data_at_kop

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
        result = {"name": self.name, "tvd": self.tvd, "ns": self.ns, "ew": self.ew}
        if self.inclination is not None: result["inclination"] = self.inclination
        if self.azimuth is not None: result["azimuth"] = self.azimuth
        if self.tolerance is not None: result["tolerance"] = self.tolerance
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
    return max(abs(float(point["verticalDepth"]) - target.tvd),
               abs(float(point["ns"]) - target.ns), abs(float(point["ew"]) - target.ew))

def _angle_difference(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

def _required_alignment(current: Target, following: Target) -> tuple[float, float | None]:
    dtvd, dns, dew = following.tvd-current.tvd, following.ns-current.ns, following.ew-current.ew
    horizontal = hypot(dns, dew)
    if sqrt(horizontal**2 + dtvd**2) <= 1e-12:
        raise ValueError(f"Alvos {current.name} e {following.name} possuem a mesma posição.")
    return degrees(atan2(horizontal, dtvd)), degrees(atan2(dew, dns)) % 360.0 if horizontal > 1e-12 else None

def _validate_path(points: list[dict[str, Any]], constraints: PlanningConstraints) -> list[str]:
    reasons = []
    for index, point in enumerate(points):
        dls, inc = point.get("dls"), point.get("inclination")
        if dls is not None and abs(float(dls)) > constraints.max_dls + 1e-9:
            reasons.append(f"DLS acima do limite no station {index}")
        if inc is not None and float(inc) > constraints.max_inclination + 1e-9:
            reasons.append(f"inclinação acima do limite no station {index}")
    return reasons

def _validate_segment_start(segment_start: Mapping[str, Any], expected: Mapping[str, Any],
                            position_tolerance: float, angle_tolerance: float = 1e-6) -> None:
    required = ("depth", "verticalDepth", "ns", "ew", "inclination", "azimuth")
    missing = [key for key in required if key not in segment_start or key not in expected]
    if missing:
        raise ValueError("estado inicial incompleto para continuidade física: " + ", ".join(missing))
    if abs(float(segment_start["depth"])-float(expected["depth"])) > 1e-8:
        raise ValueError("segmento não começa no MD do estado inicial")
    for key in ("verticalDepth", "ns", "ew"):
        if abs(float(segment_start[key])-float(expected[key])) > position_tolerance:
            raise ValueError(f"descontinuidade de posição no início do segmento ({key})")
    if abs(float(segment_start["inclination"])-float(expected["inclination"])) > angle_tolerance:
        raise ValueError("descontinuidade de inclinação no início do segmento")
    if _angle_difference(segment_start["azimuth"], expected["azimuth"]) > angle_tolerance:
        raise ValueError("descontinuidade de azimute no início do segmento")

class MultiTargetPlanner:
    """Sequential planner with optional terminal solvers and per-method KOPs."""
    def __init__(self, constraints: PlanningConstraints, alignment_tolerance_deg: float = 0.1):
        if alignment_tolerance_deg < 0: raise ValueError("A tolerância angular não pode ser negativa.")
        self.constraints = constraints
        self.alignment_tolerance_deg = float(alignment_tolerance_deg)

    def _validate_arrival_alignment(self, endpoint: Mapping[str, Any], target: Target,
                                    following: Target | None) -> None:
        if following is not None:
            required_inc, required_azi = _required_alignment(target, following)
            description = f"alvo {target.name} em direção a {following.name}"
            if target.inclination is not None and abs(target.inclination-required_inc) > self.alignment_tolerance_deg:
                raise ValueError(f"INC informada para {target.name} conflita com o alinhamento para {following.name}.")
            if target.azimuth is not None and required_azi is not None and _angle_difference(target.azimuth, required_azi) > self.alignment_tolerance_deg:
                raise ValueError(f"AZI informada para {target.name} conflita com o alinhamento para {following.name}.")
        elif target.inclination is not None or target.azimuth is not None:
            if target.inclination is None or target.azimuth is None:
                raise ValueError(f"O alvo final {target.name} exige INC e AZI informados em conjunto.")
            required_inc, required_azi = target.inclination, target.azimuth
            description = f"orientação final do alvo {target.name}"
        else:
            return
        inc_error = abs(float(endpoint["inclination"])-required_inc)
        azi_error = _angle_difference(endpoint["azimuth"], required_azi) if required_azi is not None else 0.0
        if inc_error > self.alignment_tolerance_deg or azi_error > self.alignment_tolerance_deg:
            raise ValueError(f"alinhamento incorreto em {description}: INC erro={inc_error:.4g}°, AZI erro={azi_error:.4g}° (tolerância={self.alignment_tolerance_deg:g}°)")

    def plan(self, last_data: dict[str, Any], targets: Iterable[Target],
             solvers: Mapping[str, Solver], *,
             terminal_solvers: Mapping[str, Solver] | None = None,
             kop_by_method: Mapping[str, float] | None = None) -> dict[str, Any]:
        """KOPs are optional absolute MD values, applied per method on the first leg."""
        ordered_targets = tuple(targets)
        if not ordered_targets: raise ValueError("Informe pelo menos um alvo.")
        if not solvers and not terminal_solvers: raise ValueError("Informe pelo menos um método.")
        if len(ordered_targets) > 1 and not solvers:
            raise ValueError("Informe métodos para os alvos intermediários.")
        if terminal_solvers is not None and not terminal_solvers:
            raise ValueError("terminal_solvers foi informado, mas está vazio.")
        kop_by_method = dict(kop_by_method or {})
        unknown = set(kop_by_method) - (set(solvers) | set(terminal_solvers or {}))
        if unknown:
            raise ValueError("KOP informado para método inexistente: " + ", ".join(sorted(unknown)))
        branches = [MultiTargetCandidate((), [dict(last_data)])]
        rejected = []
        for index, target in enumerate(ordered_targets):
            following = ordered_targets[index+1] if index+1 < len(ordered_targets) else None
            alignment_target = following.as_solver_input() if following else None
            active_solvers = terminal_solvers if following is None and terminal_solvers is not None else solvers
            next_branches = []
            for branch in branches:
                start = dict(branch.points[-1])
                for method, solve in active_solvers.items():
                    try:
                        solver_start, prefix = start, []
                        if index == 0 and method in kop_by_method:
                            solver_start = prepare_last_data_at_kop(start, kop_by_method[method])
                            if float(solver_start["depth"]) > float(start["depth"]) + 1e-8:
                                prefix = [solver_start]
                        segment = [dict(p) for p in solve(last_data=solver_start,
                            target=target.as_solver_input(), alignment_target=alignment_target)]
                        if len(segment) < 2:
                            raise ValueError("método deve retornar ponto inicial e pelo menos um ponto final")
                        _validate_segment_start(segment[0], solver_start, self.constraints.position_tolerance)
                        combined = branch.points + prefix + segment[1:]
                        endpoint = segment[-1]
                        tolerance = target.tolerance if target.tolerance is not None else self.constraints.position_tolerance
                        error = _position_error(endpoint, target)
                        if error > tolerance:
                            raise ValueError(f"alvo {target.name} não atingido; erro máximo={error:.6g}")
                        self._validate_arrival_alignment(endpoint, target, following)
                        violations = _validate_path(combined, self.constraints)
                        if violations: raise ValueError("; ".join(violations))
                        next_branches.append(MultiTargetCandidate(
                            branch.methods+(method,), combined, branch.reached_targets+[target.name]))
                    except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError) as exc:
                        rejected.append({"target": target.name, "method": method, "reason": str(exc)})
            if not next_branches:
                raise ValueError(f"Nenhuma trajetória viável após o alvo {target.name}. Rejeições: {rejected}")
            branches = next_branches
        best = min(branches, key=lambda candidate: candidate.final_md)
        return {"best": best, "candidates": branches, "rejected": rejected,
                "target_order": [target.name for target in ordered_targets]}
