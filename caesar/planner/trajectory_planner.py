"""Orchestrates method solvers, validation and minimum-MD selection."""
from __future__ import annotations

from .evaluator import evaluate_candidate, select_shortest
from .models import PlanningConstraints


class TrajectoryPlanner:
    def __init__(self, constraints: PlanningConstraints, validators=()):
        self.constraints = constraints
        self.validators = tuple(validators)

    def compare(self, target: dict, candidates: dict[str, callable]):
        """Run candidate callables; each receives no args and returns trajectory points."""
        evaluated = []
        errors = {}
        for method, solve in candidates.items():
            try:
                points = solve()
                evaluated.append(evaluate_candidate(
                    method, points, target, self.constraints, self.validators
                ))
            except (ValueError, RuntimeError, ArithmeticError) as exc:
                errors[method] = str(exc)
        if not evaluated or not any(item.valid for item in evaluated):
            detail = "; ".join(f"{name}: {reason}" for name, reason in errors.items())
            raise ValueError("Nenhuma solução válida. " + detail)
        best = select_shortest(evaluated)
        return {"best": best, "candidates": evaluated, "errors": errors}
