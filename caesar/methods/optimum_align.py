import json
from math import pi, sin

import numpy as np
from scipy.optimize import least_squares

from caesar.core.trajectory import (
    WellTrajectory,
    deg2rad,
    gamma_factor,
    normalize_angle_deg,
    rad2deg,
    spatial_angle,
    target_reached,
)


POSITION_TOL = 1e-3
ARCSIN_EPS = 1e-12
BOUNDS = ([0.0, 0.0, 25.0], [pi, 2 * pi, 1e6])


def target_direction_guess(delta_tvd, delta_n, delta_e):
    # Geometric guess that points directly from the current point to the target.
    horizontal = np.hypot(delta_n, delta_e)
    return np.arctan2(horizontal, delta_tvd), np.arctan2(delta_e, delta_n)


def _build_angle_from_extension(inclination_deg, build_deg_per_30m, extension):
    return inclination_deg - extension * build_deg_per_30m / 30.0


def _build_angle_from_tvd(inclination_deg, build_deg_per_30m, dtvd):
    if build_deg_per_30m == 0:
        return inclination_deg

    # Back-calculate the previous inclination for a fixed-curvature section.
    radius = 30.0 / deg2rad(build_deg_per_30m)
    arg = sin(deg2rad(inclination_deg)) - dtvd / radius
    return rad2deg(np.arcsin(np.clip(arg, -1.0 + ARCSIN_EPS, 1.0 - ARCSIN_EPS)))


def _extension_from_build(inclination_final_deg, inclination_initial_deg, build_deg_per_30m):
    if build_deg_per_30m == 0:
        raise ValueError("Build nao pode ser zero")
    return 30.0 * (inclination_final_deg - inclination_initial_deg) / build_deg_per_30m


def _extension_from_incazi(inc_i_deg, azi_i_deg, inc_f_deg, azi_f_deg, build_deg_per_30m):
    if build_deg_per_30m == 0:
        raise ValueError("Build nao pode ser zero")

    eps = spatial_angle(
        deg2rad(inc_i_deg),
        deg2rad(azi_i_deg),
        deg2rad(inc_f_deg),
        deg2rad(azi_f_deg),
    )
    return 30.0 * eps / deg2rad(build_deg_per_30m)


def _surface_extension(vertical_depth):
    return vertical_depth


def _segment_delta(inc_i_deg, azi_i_deg, inc_f_deg, azi_f_deg, length):
    # Use the same trajectory engine as the final build so project-back points
    # are derived from the exact same geometry model.
    segment = WellTrajectory([0.0, length], [inc_i_deg, inc_f_deg], [azi_i_deg, azi_f_deg])
    return segment.tvd[1], segment.north[1], segment.east[1]


def _normalize_project_back_requests(project_back):
    # Accept either a single object or a list and flatten optional nested values.
    if project_back is None:
        return []
    requests = project_back if isinstance(project_back, list) else [project_back]

    normalized = []
    for request in requests:
        if not isinstance(request, dict) or "method" not in request:
            raise ValueError("Cada item de project_back deve ser um objeto com method")

        merged = dict(request)
        if isinstance(merged.get("values"), dict):
            values = merged.pop("values")
            merged.update(values)

        if merged.pop("enabled", True):
            normalized.append(merged)

    return normalized


def _apply_project_back(request, current_target, theta_target, phi_target):
    # Each project-back request is solved backwards from the current target,
    # producing the previous point plus the forward section needed later.
    method = request["method"]

    if method == "extension":
        extension = request["extension"]
        prev_theta = _build_angle_from_extension(theta_target, request["dls"], extension)
        prev_phi = phi_target
        section_type = "Back to Extension"
    elif method == "tvd":
        dtvd = current_target["tvd"] - request["vertical_depth"]
        prev_theta = _build_angle_from_tvd(theta_target, request["dls"], dtvd)
        unit_dtvd, _, _ = _segment_delta(prev_theta, phi_target, theta_target, phi_target, 1.0)
        if abs(unit_dtvd) < 1e-12:
            raise ValueError("Nao foi possivel calcular a extensao do project back por TVD")
        extension = dtvd / unit_dtvd
        prev_phi = phi_target
        section_type = "Back to TVD"
    elif method == "inc":
        prev_theta = request["inclination"]
        extension = _extension_from_build(theta_target, prev_theta, request["dls"])
        prev_phi = phi_target
        section_type = "Back to INC"
    elif method == "incazi":
        prev_theta = request["inclination"]
        prev_phi = request["azimuth"]
        extension = _extension_from_incazi(prev_theta, prev_phi, theta_target, phi_target, request["dls"])
        section_type = "Back to INC AZI"
    else:
        raise ValueError(f"Metodo de project_back invalido: {method}")

    dtvd, dns, dew = _segment_delta(prev_theta, prev_phi, theta_target, phi_target, extension)
    next_target = {
        "tvd": current_target["tvd"] - dtvd,
        "ns": current_target["ns"] - dns,
        "ew": current_target["ew"] - dew,
    }
    segment = {
        "extension": extension,
        "inclination": theta_target,
        "azimuth": phi_target,
        "sectionType": section_type,
    }
    return next_target, prev_theta, prev_phi, segment


def _extract_inputs(params):
    # Keep compatibility with both the new "plan" payload and the legacy layout.
    if "plan" in params:
        plan = params["plan"]
        return (
            params["last_data"],
            plan["target"],
            plan["inclination"],
            plan["azimuth"],
            plan.get("dogleg1"),
            plan.get("dogleg2"),
            _normalize_project_back_requests(plan.get("project_back")),
        )

    return (
        params["last_data"],
        params["targets"]["target"],
        params["targets"]["inclination"],
        params["targets"]["azimuth"],
        params["values"]["dogleg1"],
        params["values"]["dogleg2"],
        _normalize_project_back_requests(params.get("project_back")),
    )


def _curve_hold_curve_residuals(x, theta0, phi0, theta_f, phi_f, dls1, dls2, delta_tvd, delta_n, delta_e):
    # Three residuals for the three unknowns:
    # intermediate inclination, intermediate azimuth and hold length.
    theta, phi, hold_length = x

    eps1 = spatial_angle(theta0, phi0, theta, phi)
    eps2 = spatial_angle(theta, phi, theta_f, phi_f)

    l1 = 30.0 * eps1 / deg2rad(dls1)
    l2 = 30.0 * eps2 / deg2rad(dls2)
    g1 = gamma_factor(l1, eps1)
    g2 = gamma_factor(l2, eps2)

    tvd = (g1 * (np.cos(theta0) + np.cos(theta)) +
           hold_length * np.cos(theta) +
           g2 * (np.cos(theta) + np.cos(theta_f)))
    north = (g1 * (np.sin(theta0) * np.cos(phi0) + np.sin(theta) * np.cos(phi)) +
             hold_length * np.sin(theta) * np.cos(phi) +
             g2 * (np.sin(theta) * np.cos(phi) + np.sin(theta_f) * np.cos(phi_f)))
    east = (g1 * (np.sin(theta0) * np.sin(phi0) + np.sin(theta) * np.sin(phi)) +
            hold_length * np.sin(theta) * np.sin(phi) +
            g2 * (np.sin(theta) * np.sin(phi) + np.sin(theta_f) * np.sin(phi_f)))

    return np.array([
        tvd - delta_tvd,
        north - delta_n,
        east - delta_e],
        dtype=float
    )


def _solve_with_dls(theta0_deg, phi0_deg, theta_f_deg, phi_f_deg, dls1, dls2, delta_tvd, delta_n, delta_e):
    # Solve one curve-hold-curve candidate with a fixed DLS pair.
    theta0 = deg2rad(theta0_deg)
    phi0 = deg2rad(phi0_deg)
    theta_f = deg2rad(theta_f_deg)
    phi_f = deg2rad(phi_f_deg)
    theta_guess, phi_guess = target_direction_guess(delta_tvd, delta_n, delta_e)

    x0 = [
        0.5 * (theta_guess + theta_f) % pi,
        phi_guess % (2 * pi),
        max(np.hypot(delta_n, delta_e) * 0.3, 25.0),
    ]

    res = least_squares(
        _curve_hold_curve_residuals,
        x0=x0,
        bounds=BOUNDS,
        args=(theta0, phi0, theta_f, phi_f, dls1, dls2, delta_tvd, delta_n, delta_e),
        method="trf",
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
        max_nfev=2000,
    )

    if not res.success:
        return None

    error = float(np.max(np.abs(res.fun)))
    theta_opt, phi_opt, hold_length_opt = res.x
    return (
        error,
        rad2deg(theta_opt),
        normalize_angle_deg(rad2deg(phi_opt)),
        hold_length_opt,
        dls1,
        dls2,
    )


def solve_curve_hold_curve_with_dls_relax(
    theta0_deg,
    phi0_deg,
    theta_f_deg,
    phi_f_deg,
    dls1,
    dls2,
    delta_tvd,
    delta_n,
    delta_e,
    step=0.1,
    max_iter=200,
    residual_tol=POSITION_TOL,
):
    # If the initial DLS does not close the target, keep increasing both values
    # and keep the best converged candidate found along the sweep.
    best = None

    for i in range(max_iter):
        candidate = _solve_with_dls(
            theta0_deg,
            phi0_deg,
            theta_f_deg,
            phi_f_deg,
            dls1 + i * step,
            dls2 + i * step,
            delta_tvd,
            delta_n,
            delta_e,
        )
        if candidate is None:
            continue

        if best is None or candidate[0] < best[0]:
            best = candidate

        if candidate[0] <= residual_tol:
            break

    if best is None:
        raise RuntimeError("Solver nao convergiu mesmo aumentando o DLS")

    _, theta, phi, hold_length, best_dls1, best_dls2 = best
    return theta, phi, hold_length, best_dls1, best_dls2


def arc_lengths(theta0_deg, phi0_deg, theta_deg, phi_deg, theta_f_deg, phi_f_deg, dls1, dls2):
    theta0 = deg2rad(theta0_deg)
    phi0 = deg2rad(phi0_deg)
    theta = deg2rad(theta_deg)
    phi = deg2rad(phi_deg)
    theta_f = deg2rad(theta_f_deg)
    phi_f = deg2rad(phi_f_deg)

    eps1 = spatial_angle(theta0, phi0, theta, phi)
    eps2 = spatial_angle(theta, phi, theta_f, phi_f)
    return 30.0 * eps1 / deg2rad(dls1), 30.0 * eps2 / deg2rad(dls2)


def _build_results(start, lengths, inc, azi, section_types, target, include_start=True):
    md = np.cumsum(lengths)
    well = WellTrajectory(md, inc, azi)

    well.tvd += start["verticalDepth"]
    well.north += start["ns"]
    well.east += start["ew"]
    well.vs += start.get("displacement", 0.0)

    results = []

    # --- Start opcional ---
    if include_start:
        results.append({
            "depth": md[0],
            "extension": 0.0,
            "inclination": inc[0],
            "azimuth": azi[0],
            "verticalDepth": well.tvd[0],
            "ns": well.north[0],
            "ew": well.east[0],
            "dls": well.dls[0],
            "displacement": well.vs[0],
            "toolface": well.toolface[0],
            "build": well.build[0],
            "turn": well.turn[0],
            "sectionType": "Surface",
            "target": "",
        })

    # --- Construção dos pontos ---
    for i in range(1, len(md)):
        point = {
            "depth": md[i],
            "extension": lengths[i],
            "inclination": inc[i],
            "azimuth": azi[i],
            "verticalDepth": well.tvd[i],
            "ns": well.north[i],
            "ew": well.east[i],
            "dls": well.dls[i],
            "displacement": well.vs[i],
            "toolface": well.toolface[i],
            "build": well.build[i],
            "turn": well.turn[i],
            "sectionType": section_types[i - 1] if i > 0 else section_types[0],
            "target": "",
        }

        if target_reached(point, target):
            point["target"] = target.get("name", "")

        results.append(point)

    return results


def _surface_start(anchor):
    # Surface starts at TVD/MD zero but keeps the horizontal offset of the
    # previous back-to section, matching the planning convention in use here.
    return {
        "depth": 0.0,
        "inclination": 0.0,
        "azimuth": 0.0,
        "verticalDepth": 0.0,
        "ns": anchor["ns"],
        "ew": anchor["ew"],
        "displacement": np.hypot(anchor["ns"], anchor["ew"]),
    }


def calculate_optimum_align(params, progress):
    # Main flow:
    # 1. Read target / DLS / project-back requests.
    # 2. Move the target backwards through project-back sections.
    # 3. If surface is terminal, build only the back-to sections.
    # 4. Otherwise, solve optimum align up to the earliest back-to point.
    last, target, theta_f, phi_f, dls1, dls2, project_back_requests = _extract_inputs(params)

    current_target = {"tvd": target["tvd"], "ns": target["ns"], "ew": target["ew"]}
    theta_chc = theta_f
    phi_chc = phi_f
    segments = []

    surface_terminal = bool(project_back_requests) and project_back_requests[-1]["method"] == "surface"
    for request in (project_back_requests[:-1] if surface_terminal else project_back_requests):

        current_target, theta_chc, phi_chc, segment = _apply_project_back(
            request,
            current_target,
            theta_chc,
            phi_chc,
        )
        segments.append(segment)

    if surface_terminal:
        # Surface closes the plan without running optimum align.
        segments.append({
            "extension": _surface_extension(current_target["tvd"]),
            "inclination": 0.0,
            "azimuth": 0.0,
            "sectionType": "Back to Surface",
        })
        start = _surface_start(current_target)
        ordered = list(reversed(segments))
        lengths = [0.0] + [segment["extension"] for segment in ordered]
        inc = [0.0] + [segment["inclination"] for segment in ordered]
        azi = [0.0] + [segment["azimuth"] for segment in ordered]
        section_types = [segment["sectionType"] for segment in ordered]
        results = _build_results(
            start,
            lengths,
            inc,
            azi,
            section_types,
            target,
            include_start=True
        )

        return results

    if dls1 is None or dls2 is None:
        raise ValueError("dogleg1 e dogleg2 sao obrigatorios quando ha optimum align")

    # Solve the remaining path from the current well state to the earliest
    # project-back point that still needs to be reached.
    theta, phi, hold_length, dls1, dls2 = solve_curve_hold_curve_with_dls_relax(
        last["inclination"],
        last["azimuth"],
        theta_chc,
        phi_chc,
        dls1,
        dls2,
        current_target["tvd"] - last["verticalDepth"],
        current_target["ns"] - last["ns"],
        current_target["ew"] - last["ew"],
    )

    l1, l2 = arc_lengths(
        last["inclination"],
        last["azimuth"],
        theta,
        phi,
        theta_chc,
        phi_chc,
        dls1,
        dls2,
    )

    lengths = [last["depth"], l1, hold_length, l2]
    inc = [last["inclination"], theta, theta, theta_chc]
    azi = [last["azimuth"], phi, phi, phi_chc]
    section_types = ["OPT ALN", "OPT ALN", "OPT ALN"]

    for segment in reversed(segments):
        # Project-back sections were solved backwards, so they are appended
        # in reverse order to rebuild the forward path.
        lengths.append(segment["extension"])
        inc.append(segment["inclination"])
        azi.append(segment["azimuth"])
        section_types.append(segment["sectionType"])

    return _build_results(last, lengths, inc, azi, section_types, target, include_start=False)


def solve_optimum_align(
    last_data,
    target,
    alignment_target=None,
    *,
    dogleg1,
    dogleg2,
    project_back=None,
    progress=None,
):
    """Adapter for sequential planning.

    For an intermediate target, its arrival tangent is set toward the next
    target. KOP is intentionally absent: last_data must already be prepared
    at the physical KOP by the planner.
    """
    if alignment_target is not None:
        delta_tvd = float(alignment_target["tvd"]) - float(target["tvd"])
        delta_ns = float(alignment_target["ns"]) - float(target["ns"])
        delta_ew = float(alignment_target["ew"]) - float(target["ew"])
        horizontal = float(np.hypot(delta_ns, delta_ew))
        if horizontal < 1e-12 and abs(delta_tvd) < 1e-12:
            raise ValueError("O alvo seguinte coincide com o alvo atual.")
        inclination = rad2deg(np.arctan2(horizontal, delta_tvd))
        azimuth = (
            normalize_angle_deg(rad2deg(np.arctan2(delta_ew, delta_ns)))
            if horizontal > 1e-12 else float(target.get("azimuth", last_data["azimuth"]))
        )
        if target.get("inclination") is not None and abs(float(target["inclination"]) - inclination) > 0.1:
            raise ValueError("INC informada conflita com o alinhamento para o próximo alvo.")
        if target.get("azimuth") is not None and horizontal > 1e-12:
            azi_error = abs((float(target["azimuth"]) - azimuth + 180.0) % 360.0 - 180.0)
            if azi_error > 0.1:
                raise ValueError("AZI informada conflita com o alinhamento para o próximo alvo.")
    else:
        if target.get("inclination") is None or target.get("azimuth") is None:
            raise ValueError(
                "O último alvo precisa informar INC e AZI finais quando não há próximo alvo."
            )
        inclination = float(target["inclination"])
        azimuth = float(target["azimuth"])

    params = {
        "last_data": dict(last_data),
        "plan": {
            "target": dict(target),
            "inclination": inclination,
            "azimuth": azimuth,
            "dogleg1": float(dogleg1),
            "dogleg2": float(dogleg2),
            "project_back": project_back,
        },
    }
    points = calculate_optimum_align(params, progress)
    result = [dict(point) for point in points]
    if not result or float(result[0]["depth"]) != float(last_data["depth"]):
        result.insert(0, dict(last_data))
    return result
