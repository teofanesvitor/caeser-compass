"""Dogleg Toolface and single-target Curve-Hold trajectory solver."""
from __future__ import annotations

from math import atan2, cos, degrees, hypot, pi, radians, sin
import numpy as np
from scipy.optimize import least_squares

from caesar.core.trajectory import spatial_angle
from .base import build_segment


def solve_tool_face(last_data: dict, values: dict) -> tuple[float, float]:
    """Compute final INC/AZI from a supplied toolface, DLS and extension."""
    required = ("tool_face", "dls", "extension")
    missing = [key for key in required if key not in values]
    if missing:
        raise KeyError(f"values sem campos obrigatórios: {', '.join(missing)}")
    inc0 = radians(float(last_data["inclination"]))
    azi0 = radians(float(last_data["azimuth"]))
    tf = radians(float(values["tool_face"]))
    dls, extension = float(values["dls"]), float(values["extension"])
    if extension <= 0 or dls < 0:
        raise ValueError("Extension deve ser positiva e DLS não pode ser negativo.")
    beta = radians(dls) * extension / 30.0
    inc_arg = cos(inc0) * cos(beta) - sin(inc0) * sin(beta) * cos(tf)
    inc = np.arccos(np.clip(inc_arg, -1.0, 1.0))
    azi = azi0 + atan2(
        sin(beta) * sin(tf),
        sin(inc0) * cos(beta) + cos(inc0) * sin(beta) * cos(tf),
    )
    return degrees(float(inc)), degrees(azi) % 360.0


def calculate_trajectory(params: dict, progress=None) -> list[dict]:
    """Build a dogleg segment from explicit toolface, DLS and extension."""
    try:
        last_data, values = params["last_data"], params["values"]
    except KeyError as exc:
        raise KeyError(f"Parâmetro obrigatório ausente: {exc.args[0]}") from exc
    inc, azi = solve_tool_face(last_data, values)
    final_md = float(last_data["depth"]) + float(values["extension"])
    return build_segment(last_data, final_md, inc, azi, "Dogleg Toolface")


def _curve_hold_residual(x, inc0, azi0, delta_tvd, delta_ns, delta_ew, dls):
    """Position residual for one constant-DLS curve followed by a tangent hold."""
    inc1, azi1, hold_length = x
    dogleg = spatial_angle(inc0, azi0, inc1, azi1)
    curve_length = 30.0 * dogleg / radians(dls)
    gamma = (
        curve_length / 2.0
        if abs(dogleg) < 1e-12
        else curve_length / dogleg * np.tan(dogleg / 2.0)
    )
    tvd = gamma * (cos(inc0) + cos(inc1)) + hold_length * cos(inc1)
    north = (
        gamma * (sin(inc0) * cos(azi0) + sin(inc1) * cos(azi1))
        + hold_length * sin(inc1) * cos(azi1)
    )
    east = (
        gamma * (sin(inc0) * sin(azi0) + sin(inc1) * sin(azi1))
        + hold_length * sin(inc1) * sin(azi1)
    )
    return np.array(
        [tvd - delta_tvd, north - delta_ns, east - delta_ew],
        dtype=float,
    )


def solve_curve_hold(
    last_data: dict,
    target: dict,
    *,
    dls: float,
    dls_step: float = 0.1,
    max_iter: int = 300,
    position_tolerance: float = 1e-3,
) -> list[dict]:
    """Optimize a Curve-Hold path to one target using DLS as the main input.

    The requested DLS is tried first. If it cannot close the target, nearby
    DLS values are tested in the same outward sweep used by the original
    COMPASS JavaScript implementation. Curve endpoint INC/AZI and hold length
    are solved internally.
    """
    required_target = ("tvd", "ns", "ew")
    missing = [key for key in required_target if key not in target]
    if missing:
        raise KeyError(f"target sem campos obrigatórios: {', '.join(missing)}")
    dls = float(dls)
    if dls <= 0 or dls_step <= 0:
        raise ValueError("DLS e dls_step devem ser positivos.")
    if max_iter < 1:
        raise ValueError("max_iter deve ser maior que zero.")

    inc0 = radians(float(last_data["inclination"]))
    azi0 = radians(float(last_data["azimuth"]))
    delta_tvd = float(target["tvd"]) - float(last_data["verticalDepth"])
    delta_ns = float(target["ns"]) - float(last_data["ns"])
    delta_ew = float(target["ew"]) - float(last_data["ew"])
    horizontal = hypot(delta_ns, delta_ew)
    if hypot(horizontal, delta_tvd) <= position_tolerance:
        raise ValueError("O alvo coincide com a posição inicial.")
    inc_guess = atan2(horizontal, delta_tvd)
    azi_guess = atan2(delta_ew, delta_ns) % (2.0 * pi)

    dls_candidates = [dls]
    for i in range(1, max_iter + 1):
        dls_candidates.append(dls + i * dls_step)
        lower = dls - i * dls_step
        if lower > 0:
            dls_candidates.append(lower)

    best = None
    for candidate_dls in dls_candidates:
        for inc_factor in (1.0, 0.5, 1.5, 0.2):
            for azi_shift in (0.0, 0.3, -0.3):
                x0 = [
                    min(inc_guess * inc_factor, radians(120.0)),
                    (azi_guess + azi_shift) % (2.0 * pi),
                    max(horizontal * 0.3, 25.0),
                ]
                result = least_squares(
                    _curve_hold_residual,
                    x0=x0,
                    bounds=(
                        [0.0, 0.0, 1e-7],
                        [radians(120.0), 2.0 * pi, 1e7],
                    ),
                    args=(
                        inc0, azi0, delta_tvd, delta_ns, delta_ew,
                        candidate_dls,
                    ),
                    method="trf",
                    ftol=1e-11,
                    xtol=1e-11,
                    gtol=1e-11,
                    max_nfev=1000,
                )
                error = float(np.max(np.abs(result.fun)))
                if best is None or error < best[0]:
                    best = (error, result.x.copy(), candidate_dls)
                if result.success and error <= position_tolerance:
                    break
            if best is not None and best[0] <= position_tolerance:
                break
        if best is not None and best[0] <= position_tolerance:
            break

    if best is None or best[0] > position_tolerance:
        error = float("inf") if best is None else best[0]
        raise RuntimeError(
            f"Curve-Hold não fechou o alvo; erro máximo: {error:.6g} m."
        )

    _, solution, used_dls = best
    inc1, azi1, hold_length = map(float, solution)
    dogleg = spatial_angle(inc0, azi0, inc1, azi1)
    curve_length = 30.0 * dogleg / radians(used_dls)
    if curve_length <= 1e-8 or hold_length <= 0:
        raise RuntimeError("Curve-Hold encontrou extensão de curva/hold inválida.")

    start_md = float(last_data["depth"])
    curve = build_segment(
        last_data,
        start_md + curve_length,
        degrees(inc1),
        degrees(azi1),
        "Dogleg Toolface Curve",
    )[-1]
    hold = build_segment(
        curve,
        float(curve["depth"]) + hold_length,
        degrees(inc1),
        degrees(azi1),
        "Dogleg Toolface Hold",
    )[-1]
    hold["target"] = str(target.get("name", ""))
    return [dict(last_data), curve, hold]


def solve(
    last_data: dict,
    target: dict,
    alignment_target=None,
    *,
    dls: float,
    **kwargs,
) -> list[dict]:
    """Multi-target planner adapter for the single-target Curve-Hold solver."""
    return solve_curve_hold(last_data, target, dls=dls, **kwargs)
