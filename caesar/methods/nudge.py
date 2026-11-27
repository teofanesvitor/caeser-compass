"""Nudge / fixed-MD orientation-change adapter."""
from __future__ import annotations
from .base import build_segment


def calculate_trajectory(params: dict, progress=None) -> list[dict]:
    """Calculate the supported MdIncAzi mode using Minimum Curvature."""
    try:
        last_data, method, constraints = params["last_data"], params["method"], params["constraints"]
    except KeyError as exc:
        raise KeyError(f"Parâmetro obrigatório ausente: {exc.args[0]}") from exc
    if method not in ("MdIncAzi", "md-inc-azi", "MD_INC_AZI"):
        raise NotImplementedError(
            f"Modo Nudge ainda não portado: {method!r}. O adaptador MdIncAzi está disponível."
        )
    required = ("final_md", "final_inc", "final_azi")
    missing = [key for key in required if key not in constraints]
    if missing:
        raise KeyError(f"constraints sem campos obrigatórios: {', '.join(missing)}")
    return build_segment(last_data, constraints["final_md"], constraints["final_inc"],
                         constraints["final_azi"], "Nudge MdIncAzi")


def solve(last_data: dict, target: dict | None = None, **parameters) -> list[dict]:
    method = parameters.pop("method", "MdIncAzi")
    return calculate_trajectory({"last_data": last_data, "method": method,
                                  "constraints": parameters})
