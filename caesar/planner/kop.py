"""Preparação geométrica do ponto de início para planejamento no KOP."""
from __future__ import annotations
from math import cos, radians, sin, isfinite
from typing import Any

def prepare_last_data_at_kop(last_data: dict[str, Any], kop_md: float) -> dict[str, Any]:
    """Projeta o último survey em hold até o MD absoluto do KOP."""
    required = ("depth", "inclination", "azimuth", "verticalDepth", "ns", "ew")
    missing = [key for key in required if key not in last_data]
    if missing:
        raise KeyError(f"last_data sem campos obrigatórios: {', '.join(missing)}")
    try:
        kop_md, depth, inc, azi, tvd, ns, ew = map(float, (
            kop_md, last_data["depth"], last_data["inclination"], last_data["azimuth"],
            last_data["verticalDepth"], last_data["ns"], last_data["ew"]))
    except (TypeError, ValueError) as exc:
        raise ValueError("Os dados do survey e o KOP devem ser numéricos.") from exc
    if not all(isfinite(v) for v in (kop_md, depth, inc, azi, tvd, ns, ew)):
        raise ValueError("Os dados do survey e o KOP devem ser finitos.")
    if kop_md < depth:
        raise ValueError("O KOP não pode ser anterior ao último survey.")
    if not 0.0 <= inc <= 180.0:
        raise ValueError("Inclinação deve estar entre 0 e 180 graus.")
    extension = kop_md - depth
    inc_rad, azi_rad = radians(inc), radians(azi)
    result = dict(last_data)
    result.update({
        "depth": kop_md, "inclination": inc, "azimuth": azi % 360.0,
        "verticalDepth": tvd + extension * cos(inc_rad),
        "ns": ns + extension * sin(inc_rad) * cos(azi_rad),
        "ew": ew + extension * sin(inc_rad) * sin(azi_rad),
        "dls": 0.0 if extension > 0 else last_data.get("dls", 0.0),
        "extension": extension,
        "sectionType": "Hold to KOP" if extension > 0 else last_data.get("sectionType", "Survey"),
    })
    return result
