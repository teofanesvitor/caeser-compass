"""JSON interface for running complete multi-target plans."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from caesar.planner import (
    MultiTargetPlanner, PlanningConstraints, Target,
    hold_solver, optimum_align_solver, slant_solver, nudge_solver,
    dogleg_toolface_solver, dogleg_curve_hold_solver,
)


def _build_solver(name: str, config: dict[str, Any], constraints: PlanningConstraints):
    kind = config["type"]
    p = dict(config.get("params", {}))
    if kind == "hold":
        return {name: hold_solver()}
    if kind == "optimum_align":
        return {name: optimum_align_solver(**p)}
    if kind == "slant":
        holds = p.pop("hold_lengths")
        if not holds:
            raise ValueError(f"{name}: informe hold_lengths no Slant.")
        # Each HOLD_LEN is a separate branch so later targets can choose
        # the globally shortest complete route, not only Slant's local best.
        result = {}
        for hold in holds:
            label = f"{name} [HOLD_LEN={float(hold):g}]"
            result[label] = slant_solver(
                method_flags=p["method_flags"], values=p["values"],
                hold_lengths=[float(hold)], constraints=constraints,
            )
        return result
    if kind == "nudge":
        return {name: nudge_solver(**p)}
    if kind == "dogleg_toolface":
        return {name: dogleg_toolface_solver(**p)}
    if kind == "dogleg_curve_hold":
        return {name: dogleg_curve_hold_solver(**p)}
    raise ValueError(f"{name}: tipo de método desconhecido: {kind}")


def _json_safe(value):
    if hasattr(value, "__dataclass_fields__"):
        return {key: _json_safe(getattr(value, key)) for key in value.__dataclass_fields__}
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "item"):
        return value.item()
    return value


def run_plan(config: dict[str, Any]) -> dict[str, Any]:
    """Run a plan described by the documented JSON structure."""
    required = ("last_data", "targets", "constraints", "solvers")
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError("JSON sem campos obrigatórios: " + ", ".join(missing))

    constraints = PlanningConstraints(**config["constraints"])
    planner = MultiTargetPlanner(
        constraints,
        alignment_tolerance_deg=float(config.get("alignment_tolerance_deg", 0.1)),
    )
    targets = [Target(**item) for item in config["targets"]]

    solvers = {}
    for name, method_config in config["solvers"].items():
        solvers.update(_build_solver(name, method_config, constraints))

    terminal = {}
    for name, method_config in config.get("terminal_solvers", {}).items():
        terminal.update(_build_solver(name, method_config, constraints))

    kop_by_method = dict(config.get("kop_by_method", {}))
    # A configured Slant KOP applies to each expanded HOLD_LEN branch.
    for base_name, method_config in config["solvers"].items():
        if method_config.get("type") == "slant" and base_name in kop_by_method:
            value = kop_by_method.pop(base_name)
            for solver_name in solvers:
                if solver_name.startswith(base_name + " [HOLD_LEN="):
                    kop_by_method[solver_name] = value

    result = planner.plan(
        last_data=config["last_data"],
        targets=targets,
        solvers=solvers,
        terminal_solvers=terminal or None,
        kop_by_method=kop_by_method,
    )
    best = result["best"]
    return _json_safe({
        "best": best,
        "candidates": result["candidates"],
        "rejected": result["rejected"],
        "target_order": result["target_order"],
        "summary": {
            "methods": best.methods,
            "final_md": best.final_md,
            "reached_targets": best.reached_targets,
            "station_count": len(best.points),
        },
    })


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Planejador direcional multialvo via arquivo JSON."
    )
    parser.add_argument("input", help="Arquivo JSON de entrada.")
    parser.add_argument("-o", "--output", help="Arquivo JSON de saída; padrão: stdout.")
    args = parser.parse_args(argv)

    with Path(args.input).open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    result = run_plan(config)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)


if __name__ == "__main__":
    main()
