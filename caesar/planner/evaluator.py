"""Candidate validation and ranking."""
from __future__ import annotations

from .models import CandidateResult, PlanningConstraints
from caesar.core.trajectory import target_reached


def evaluate_candidate(method, points, target, constraints: PlanningConstraints, validators=()):
    reasons = []
    if not points:
        reasons.append("A trajetória não retornou pontos.")
        return CandidateResult(method, points, False, reasons)

    final = points[-1]
    if not target_reached(final, target, constraints.position_tolerance):
        reasons.append("O ponto final não atingiu o alvo dentro da tolerância.")

    for index, point in enumerate(points):
        if float(point.get("dls", 0.0)) > constraints.max_dls + 1e-9:
            reasons.append(f"DLS acima do limite no ponto {index}.")
        if float(point.get("inclination", 0.0)) > constraints.max_inclination + 1e-9:
            reasons.append(f"Inclinação acima do limite no ponto {index}.")

    for validator in validators:
        reasons.extend(validator(points, constraints))

    extensions = [float(p.get("extension", 0.0)) for p in points]
    if any(length < constraints.minimum_extension for length in extensions):
        reasons.append("Trecho abaixo da extensão mínima.")
    if constraints.maximum_extension is not None and any(
        length > constraints.maximum_extension for length in extensions
    ):
        reasons.append("Trecho acima da extensão máxima.")

    return CandidateResult(
        method=method, points=points, valid=not reasons, reasons=reasons,
        metrics={"final_md": float(final["depth"]),
                 "max_dls": max(float(p.get("dls", 0.0)) for p in points),
                 "max_inclination": max(float(p.get("inclination", 0.0)) for p in points)}
    )


def select_shortest(candidates):
    valid = [candidate for candidate in candidates if candidate.valid]
    if not valid:
        raise ValueError("Nenhuma trajetória candidata atende às restrições.")
    return min(valid, key=lambda candidate: candidate.final_md)
