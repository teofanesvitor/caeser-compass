from math import atan2, cos, hypot, isfinite, sin
import json

import numpy as np
from scipy.optimize import least_squares

from caesar.core.trajectory import (
    WellTrajectory,
    deg2rad,
    rad2deg,
    normalize_angle_deg,
    target_reached,
)

MIN_LEN = 50.0
KOP_FACTOR = 0.3
MAX_INC = 90.0
MAX_DLS = 30.0
BIG = 1e8
ABS_TOL = 1e-6


def _require_keys(data, required, label):
    missing = [key for key in required if key not in data]
    if missing:
        missing_str = ", ".join(missing)
        raise KeyError(f"{label} sem campos obrigatorios: {missing_str}")


def _validate_numeric(value, label, min_value=None, max_value=None, allow_zero=True):
    if value is None or not isfinite(value):
        raise ValueError(f"{label} deve ser numerico e finito.")

    if min_value is not None:
        if allow_zero:
            if value < min_value:
                raise ValueError(f"{label} deve ser >= {min_value}.")
        elif value <= min_value:
            raise ValueError(f"{label} deve ser > {min_value}.")

    if max_value is not None and value > max_value:
        raise ValueError(f"{label} deve ser <= {max_value}.")


def _build_target(last_data, target_raw):
    delta_ns = target_raw["ns"] - last_data["ns"]
    delta_ew = target_raw["ew"] - last_data["ew"]

    return {
        "tvd": target_raw["tvd"] - last_data["verticalDepth"],
        "departure": hypot(delta_ns, delta_ew),
        "delta_ns": delta_ns,
        "delta_ew": delta_ew,
    }


def _initial_guess(target, known):

    inc0 = known.get("inclination") or rad2deg(
        atan2(target["departure"], target["tvd"])
    )

    inc0 = min(max(inc0, 0.0), MAX_INC)

    dls0 = known.get("dls") or 1.0
    _validate_numeric(dls0, "DLS inicial", min_value=0.0, max_value=MAX_DLS, allow_zero=False)

    r10 = 30.0 / deg2rad(dls0)
    inc0_rad = deg2rad(inc0)
    cos_inc0 = max(cos(inc0_rad), ABS_TOL)

    hold0 = max(
        MIN_LEN,
        (target["tvd"] - r10 * sin(inc0_rad) * 1.5) / cos_inc0,
    )
    kop0 = max(MIN_LEN, KOP_FACTOR * max(target["tvd"], 0.0))

    return {
        "kop": kop0,
        "inc": inc0,
        "hold": hold0,
        "r1": r10,
    }


def _resolve_parameters(last_data, flags, known, guesses):
    fixed = {}
    free = {}
    section_parts = ["Slant"]

    if sum(bool(value) for value in flags.values()) != 2:
        raise ValueError(
            "O metodo slant precisa de exatamente 2 parametros conhecidos entre: "
            "kop, dls, inclination e hold_len."
        )

    for key in ("kop", "dls", "inclination", "hold_len"):
        if key not in flags:
            raise KeyError(f"Campo de metodo ausente: {key}")

    if flags["kop"]:
        _validate_numeric(known.get("kop"), "KOP", min_value=last_data["depth"], allow_zero=False)
        fixed["kop"] = known["kop"] - last_data["depth"]
        section_parts.append("KOP")
    else:
        free["kop"] = guesses["kop"]

    if flags["inclination"]:
        _validate_numeric(known.get("inclination"), "Inclination", min_value=0.0, max_value=MAX_INC)
        fixed["inc"] = known["inclination"]
        section_parts.append("INC")
    else:
        free["inc"] = guesses["inc"]

    if flags["hold_len"]:
        _validate_numeric(known.get("hold_len"), "Hold length", min_value=0.0)
        fixed["hold"] = known["hold_len"]
        section_parts.append("HOLD LEN")
    else:
        free["hold"] = guesses["hold"]

    if flags["dls"]:
        _validate_numeric(known.get("dls"), "DLS", min_value=0.0, max_value=MAX_DLS, allow_zero=False)
        fixed["r1"] = 30.0 / deg2rad(known["dls"])
        section_parts.append("DLS")
    else:
        free["r1"] = guesses["r1"]

    return fixed, free, " ".join(section_parts)


def _residuals(values, fixed, idx, target):
    kop = fixed["kop"] if "kop" in fixed else values[idx["kop"]]
    inc = fixed["inc"] if "inc" in fixed else values[idx["inc"]]
    hold = fixed["hold"] if "hold" in fixed else values[idx["hold"]]
    r1 = fixed["r1"] if "r1" in fixed else values[idx["r1"]]

    if r1 <= 0 or inc < 0 or inc > MAX_INC or hold < 0 or kop < 0:
        return [BIG, BIG]

    inc_rad = deg2rad(inc)

    tvd_build = r1 * sin(inc_rad)
    dep_build = r1 * (1 - cos(inc_rad))

    tvd_hold = hold * cos(inc_rad)
    dep_hold = hold * sin(inc_rad)

    return [
        kop + tvd_build + tvd_hold - target["tvd"],
        dep_build + dep_hold - target["departure"],
    ]


def _solve_geometry(fixed, free, target):
    idx = {name: position for position, name in enumerate(free)}
    x0 = np.array([free[name] for name in free], dtype=float)

    lower_bounds = []
    upper_bounds = []
    for name in free:
        if name == "inc":
            lower_bounds.append(0.0)
            upper_bounds.append(MAX_INC)
        elif name == "r1":
            lower_bounds.append(30.0 / deg2rad(MAX_DLS))
            upper_bounds.append(np.inf)
        else:
            lower_bounds.append(0.0)
            upper_bounds.append(np.inf)

    result = least_squares(
        lambda values: _residuals(values, fixed, idx, target),
        x0,
        bounds=(np.array(lower_bounds), np.array(upper_bounds)),
        xtol=1e-10,
        ftol=1e-10,
        gtol=1e-10,
    )

    if not result.success:
        raise RuntimeError(f"Solver nao convergiu: {result.message}")

    residual = _residuals(result.x, fixed, idx, target)
    if max(abs(value) for value in residual) > 1e-3:
        raise RuntimeError(
            "Solver convergiu para uma solucao numerica, mas nao fechou a geometria "
            f"(residuos={residual})."
        )

    solution = {name: result.x[idx[name]] for name in free}

    return {
        "kop": fixed["kop"] if "kop" in fixed else solution["kop"],
        "inc": fixed["inc"] if "inc" in fixed else solution["inc"],
        "hold": fixed["hold"] if "hold" in fixed else solution["hold"],
        "r1": fixed["r1"] if "r1" in fixed else solution["r1"],
    }


def _build_results(last_data, target_raw, section_type, md, azimuth):
    incs = [
        last_data["inclination"],
        last_data["inclination"],
        md["inc"],
        md["inc"],
    ]
    azis = [
        last_data["azimuth"],
        last_data["azimuth"],
        azimuth,
        azimuth,
    ]
    measured_depths = [
        last_data["depth"],
        last_data["depth"] + md["kop"],
        last_data["depth"] + md["kop"] + md["build_len"],
        last_data["depth"] + md["kop"] + md["build_len"] + md["hold"],
    ]

    traj = WellTrajectory(measured_depths, incs, azis)
    survey = traj.results()
    results = []

    for index in range(1, len(measured_depths)):
        point = {
            "depth": survey["MD"][index],
            "extension": measured_depths[index] - measured_depths[index - 1],
            "inclination": survey["Inclination"][index],
            "azimuth": survey["Azimuth"][index],
            "verticalDepth": survey["TVD"][index] + last_data["verticalDepth"],
            "ns": survey["North"][index] + last_data["ns"],
            "ew": survey["East"][index] + last_data["ew"],
            "displacement": survey["Horizontal Displacement"][index],
            "dls": survey["DLS (deg/30m)"][index],
            "build": survey["Build (deg/30m)"][index],
            "turn": survey["Turn (deg/30m)"][index],
            "toolface": survey["Toolface (deg)"][index],
            "sectionType": section_type,
            "target": "",
        }

        if target_reached(point, target_raw):
            point["target"] = target_raw.get("name", "")

        results.append(point)

    return results


def calculate_slant(params, progress):
    _require_keys(params, ("last_data", "target", "method", "values"), "params")

    last_data = params["last_data"]
    target_raw = params["target"]
    flags = params["method"]
    known = params["values"]

    _require_keys(last_data, ("depth", "inclination", "azimuth", "verticalDepth", "ns", "ew"), "last_data")
    _require_keys(target_raw, ("tvd", "ns", "ew"), "target")

    _validate_numeric(last_data["depth"], "Depth atual", min_value=0.0)
    _validate_numeric(last_data["inclination"], "Inclination atual", min_value=0.0, max_value=MAX_INC)
    _validate_numeric(last_data["verticalDepth"], "TVD atual", min_value=0.0)
    _validate_numeric(target_raw["tvd"], "Target TVD", min_value=0.0)

    target = _build_target(last_data, target_raw)
    if target["tvd"] < 0:
        raise ValueError("O target TVD precisa estar abaixo ou no mesmo TVD do ponto atual.")

    azimuth = normalize_angle_deg(rad2deg(atan2(target["delta_ew"], target["delta_ns"])))
    guesses = _initial_guess(target, known)
    fixed, free, section_type = _resolve_parameters(last_data, flags, known, guesses)
    solved = _solve_geometry(fixed, free, target)

    dls = rad2deg(30.0 / solved["r1"])
    build_len = solved["inc"] * 30.0 / dls if dls > 0 else 0.0

    return _build_results(
        last_data,
        target_raw,
        section_type,
        {
            "kop": solved["kop"],
            "inc": solved["inc"],
            "hold": solved["hold"],
            "build_len": build_len,
        },
        azimuth,
    )


def optimize_slant_hold_lengths(
    last_data,
    target,
    method_flags,
    values,
    hold_lengths,
    constraints,
):
    """Evaluate fixed first-hold lengths and return the shortest valid Slant.

    Slant requires exactly two known parameters. HOLD_LEN is fixed for each
    iteration; the caller must provide exactly one additional known parameter.
    """
    from caesar.planner.evaluator import evaluate_candidate, select_shortest

    if not hold_lengths:
        raise ValueError("Informe pelo menos um comprimento de hold.")
    flags = dict(method_flags)
    flags["hold_len"] = True
    if sum(bool(value) for value in flags.values()) != 2:
        raise ValueError(
            "Com HOLD_LEN fixo, informe exatamente mais um parâmetro conhecido "
            "entre KOP, DLS e INC."
        )

    candidates = []
    errors = {}
    for hold_length in hold_lengths:
        try:
            known = dict(values)
            known["hold_len"] = float(hold_length)
            segment = calculate_slant(
                {
                    "last_data": dict(last_data),
                    "target": dict(target),
                    "method": flags,
                    "values": known,
                },
                None,
            )
            points = [dict(last_data)] + [dict(point) for point in segment]
            candidates.append(evaluate_candidate(
                method=f"Slant HOLD={float(hold_length):g}",
                points=points,
                target=target,
                constraints=constraints,
            ))
        except (KeyError, TypeError, ValueError, RuntimeError, ArithmeticError) as exc:
            errors[str(hold_length)] = str(exc)

    valid = [candidate for candidate in candidates if candidate.valid]
    if not valid:
        raise ValueError(
            "Nenhuma solução Slant válida para os holds avaliados. "
            f"Rejeições: {errors}; inválidas: "
            f"{[item.reasons for item in candidates if not item.valid]}"
        )
    best = select_shortest(valid)
    return {"best": best, "candidates": candidates, "errors": errors}
