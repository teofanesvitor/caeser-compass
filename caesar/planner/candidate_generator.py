"""Candidate generation utilities."""
from __future__ import annotations


def inclusive_values(start: float, stop: float, step: float):
    if step <= 0:
        raise ValueError("step deve ser positivo.")
    if stop < start:
        raise ValueError("stop deve ser maior ou igual a start.")
    count = int((stop - start) // step)
    values = [start + i * step for i in range(count + 1)]
    if not values or values[-1] < stop - 1e-9:
        values.append(stop)
    return values


def hold_length_candidates(minimum: float, maximum: float, step: float):
    """Generate fixed first-Hold lengths for Slant candidate sweeps."""
    return inclusive_values(minimum, maximum, step)
