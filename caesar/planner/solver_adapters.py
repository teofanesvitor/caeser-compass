"""Adapters that expose validated trajectory methods to MultiTargetPlanner.

The numerical methods remain untouched. These wrappers only bind each method's
configuration and normalize its call/return contract.
"""
from __future__ import annotations

from typing import Any, Callable, Iterable, Mapping

from caesar.methods.dogleg_tf import calculate_trajectory as calculate_dogleg_toolface
from caesar.methods.dogleg_tf import solve as solve_dogleg_curve_hold
from caesar.methods.hold import solve as solve_hold
from caesar.methods.nudge import solve as solve_nudge
from caesar.methods.optimum_align import solve_optimum_align
from caesar.methods.slant import optimize_slant_hold_lengths
from caesar.planner.models import PlanningConstraints

Solver = Callable[..., Iterable[dict[str, Any]]]


def hold_solver() -> Solver:
    """Return the Hold method using its native multi-target interface."""
    return solve_hold


def optimum_align_solver(
    *, dogleg1: float, dogleg2: float, project_back=None, progress=None
) -> Solver:
    """Bind Optimum Align's fixed DLS inputs without changing its algorithm."""
    def solve(last_data, target, alignment_target=None):
        return solve_optimum_align(
            last_data, target, alignment_target,
            dogleg1=dogleg1, dogleg2=dogleg2,
            project_back=project_back, progress=progress,
        )
    return solve


def slant_solver(
    *,
    method_flags: Mapping[str, bool],
    values: Mapping[str, float],
    hold_lengths: Iterable[float],
    constraints: PlanningConstraints,
) -> Solver:
    """Bind Slant's two-known-parameter model and HOLD_LEN candidate sweep."""
    flags = dict(method_flags)
    known_values = dict(values)
    holds = tuple(float(value) for value in hold_lengths)

    def solve(last_data, target, alignment_target=None):
        result = optimize_slant_hold_lengths(
            last_data=last_data,
            target=target,
            method_flags=flags,
            values=known_values,
            hold_lengths=holds,
            constraints=constraints,
        )
        # optimize_slant_hold_lengths already evaluates and selects the
        # shortest valid Slant candidate; return its full station sequence.
        return [dict(point) for point in result["best"].points]
    return solve


def nudge_solver(*, final_md: float, final_inc: float, final_azi: float) -> Solver:
    """Bind the supported Nudge MdIncAzi mode."""
    def solve(last_data, target, alignment_target=None):
        return solve_nudge(
            last_data, target, alignment_target,
            final_md=final_md, final_inc=final_inc, final_azi=final_azi,
        )
    return solve


def dogleg_toolface_solver(
    *, tool_face: float, dls: float, extension: float
) -> Solver:
    """Bind the explicit Toolface/DLS/extension Dogleg method."""
    values = {"tool_face": tool_face, "dls": dls, "extension": extension}

    def solve(last_data, target, alignment_target=None):
        return calculate_dogleg_toolface(
            {"last_data": dict(last_data), "values": dict(values)}
        )
    return solve


def dogleg_curve_hold_solver(*, dls: float, **solver_options) -> Solver:
    """Bind the terminal Curve-Hold solver driven by a requested DLS."""
    def solve(last_data, target, alignment_target=None):
        return solve_dogleg_curve_hold(
            last_data, target, alignment_target, dls=dls, **solver_options
        )
    return solve
