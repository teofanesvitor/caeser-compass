"""JSON interface for automatic multi-target method and parameter selection."""
from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
from typing import Any

from caesar.planner import (
    MultiTargetPlanner, PlanningConstraints, Target,
    hold_solver, optimum_align_solver, slant_solver, nudge_solver,
    dogleg_toolface_solver, dogleg_curve_hold_solver,
)
from caesar.planner.anti_collision import (
    AntiCollisionChecker, AntiCollisionConfig, ReferenceWell,
)
from caesar.planner.candidate_generator import inclusive_values


def _is_range(value: Any) -> bool:
    return isinstance(value, dict) and set(value) == {"min", "max", "step"}


def _is_numeric_sweep(value: Any) -> bool:
    if _is_range(value):
        return True
    return (
        isinstance(value, list)
        and bool(value)
        and all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value)
    )


def _sweep_values(value: Any) -> list[Any]:
    if _is_range(value):
        return inclusive_values(
            float(value["min"]), float(value["max"]), float(value["step"])
        )
    if isinstance(value, list):
        if not value:
            raise ValueError("Uma lista de variação não pode ser vazia.")
        return list(value)
    return [value]


def _expand_parameters(value: Any, path: str = ""):
    """Expand numeric lists/ranges recursively, preserving ordinary structures."""
    if _is_numeric_sweep(value):
        options = _sweep_values(value)
        for option in options:
            label = f"{path}={option:g}" if path and isinstance(option, (int, float)) else (
                f"{path}={option}" if path else ""
            )
            yield option, ([label] if label else [])
        return
    if isinstance(value, dict):
        keys = list(value)
        expanded = []
        for key in keys:
            expanded.append(list(_expand_parameters(value[key], f"{path}.{key}".strip("."))))
        for combination in itertools.product(*expanded):
            result = {}
            labels = []
            for key, (item, item_labels) in zip(keys, combination):
                result[key] = item
                labels.extend(item_labels)
            yield result, labels
        return
    if isinstance(value, list):
        # Lists of objects/strings are configuration data, not parameter sweeps.
        yield value, []
        return
    yield value, []


def _build_solver(name: str, config: dict[str, Any], constraints: PlanningConstraints):
    kind = config["type"]
    p = dict(config.get("params", {}))
    if kind == "hold":
        return {name: hold_solver()}
    if kind == "optimum_align":
        return {name: optimum_align_solver(**p)}
    if kind == "slant":
        holds = p.pop("hold_lengths")
        if isinstance(holds, (int, float)):
            holds = [holds]
        if not holds:
            raise ValueError(f"{name}: informe params.hold_lengths.")
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


def _method_roles(kind: str, role: str) -> set[str]:
    if role == "auto":
        return {"terminal"} if kind == "dogleg_curve_hold" else {"intermediate", "terminal"}
    if role == "both":
        return {"intermediate", "terminal"}
    if role in {"intermediate", "terminal"}:
        return {role}
    raise ValueError(f"role inválido: {role!r}; use auto, both, intermediate ou terminal.")


def _build_method_pool(
    methods: dict[str, Any],
    constraints: PlanningConstraints,
    *,
    kop_by_method: dict[str, Any] | None = None,
    max_candidates: int = 250,
):
    intermediate, terminal = {}, {}
    expanded_names: dict[str, list[str]] = {}
    expanded_kops: dict[str, float] = {}
    kop_by_method = dict(kop_by_method or {})
    candidate_count = 0

    for base_name, config in methods.items():
        kind = config["type"]
        role = config.get("role", "auto")
        roles = _method_roles(kind, role)
        parameter_variants = list(_expand_parameters(config.get("params", {})))
        kop_options = _sweep_values(kop_by_method[base_name]) if base_name in kop_by_method else [None]

        for params, labels in parameter_variants:
            variant_name = base_name
            if labels:
                variant_name += " [" + ", ".join(labels) + "]"
            variant_config = {**config, "params": params}
            built = _build_solver(variant_name, variant_config, constraints)

            for kop in kop_options:
                kop_suffix = f" [KOP={float(kop):g}]" if kop is not None else ""
                for built_name, solver in built.items():
                    candidate_name = built_name + kop_suffix
                    candidate_count += 1
                    if candidate_count > max_candidates:
                        raise ValueError(
                            f"A busca gerou mais de {max_candidates} candidatos. "
                            "Reduza os intervalos ou aumente max_method_candidates."
                        )
                    if "intermediate" in roles:
                        intermediate[candidate_name] = solver
                    if "terminal" in roles:
                        terminal[candidate_name] = solver
                    if kop is not None:
                        expanded_kops[candidate_name] = float(kop)
                    expanded_names.setdefault(base_name, []).append(candidate_name)

    unknown_kops = set(kop_by_method) - set(methods)
    if unknown_kops:
        raise ValueError("KOP informado para método inexistente: " + ", ".join(sorted(unknown_kops)))
    return intermediate, terminal, expanded_names, expanded_kops


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
    required = ("last_data", "targets", "constraints")
    missing = [key for key in required if key not in config]
    if missing:
        raise ValueError("JSON sem campos obrigatórios: " + ", ".join(missing))

    constraints = PlanningConstraints(**config["constraints"])
    planner = MultiTargetPlanner(
        constraints,
        alignment_tolerance_deg=float(config.get("alignment_tolerance_deg", 0.1)),
    )
    targets = [Target(**item) for item in config["targets"]]

    if "methods" in config:
        if "solvers" in config or "terminal_solvers" in config:
            raise ValueError("Use 'methods' ou a configuração legada 'solvers'/'terminal_solvers', não ambas.")
        solvers, terminal, expanded, kop_by_method = _build_method_pool(
            config["methods"],
            constraints,
            kop_by_method=config.get("kop_by_method", {}),
            max_candidates=int(config.get("max_method_candidates", 250)),
        )
        if not terminal:
            raise ValueError("Nenhum método habilitado para o último alvo. Revise methods[*].role.")
        if len(targets) > 1 and not solvers:
            raise ValueError("Nenhum método habilitado para os alvos intermediários. Revise methods[*].role.")
        selection_mode = "automatic_parameter_sweep"
    else:
        if "solvers" not in config:
            raise ValueError("JSON precisa informar 'methods' (seleção automática) ou 'solvers' (modo legado).")
        solvers = {}
        for name, method_config in config["solvers"].items():
            solvers.update(_build_solver(name, method_config, constraints))
        terminal = {}
        for name, method_config in config.get("terminal_solvers", {}).items():
            terminal.update(_build_solver(name, method_config, constraints))
        kop_by_method = dict(config.get("kop_by_method", {}))
        selection_mode = "manual_pool"

    anti_collision_checker = None
    anti_collision_config = config.get("anti_collision")
    if anti_collision_config is not None:
        reference_wells = [
            ReferenceWell.from_dict(item)
            for item in anti_collision_config.get("reference_wells", [])
        ]
        anti_collision_checker = AntiCollisionChecker(
            AntiCollisionConfig(
                minimum_clearance_m=float(anti_collision_config["minimum_clearance_m"]),
                coordinate_system=str(anti_collision_config["coordinate_system"]),
                vertical_datum=str(anti_collision_config["vertical_datum"]),
                max_step_m=float(anti_collision_config.get("max_step_m", 2.0)),
                coordinate_tolerance_m=float(anti_collision_config.get("coordinate_tolerance_m", 1.0)),
                max_chord_error_m=float(anti_collision_config.get("max_chord_error_m", 0.02)),
            ),
            reference_wells,
        )

    result = planner.plan(
        last_data=config["last_data"],
        targets=targets,
        solvers=solvers,
        terminal_solvers=terminal or None,
        kop_by_method=kop_by_method,
        anti_collision_checker=anti_collision_checker,
    )
    best = result["best"]
    return _json_safe({
        "best": best,
        "candidates": result["candidates"],
        "rejected": result["rejected"],
        "target_order": result["target_order"],
        "anti_collision": result.get("anti_collision"),
        "selection": {
            "mode": selection_mode,
            "criterion": "menor MD final entre trajetórias completas viáveis",
            "methods": list(best.methods),
            "candidate_method_count": len(set(solvers) | set(terminal)),
        },
        "summary": {
            "methods": best.methods,
            "final_md": best.final_md,
            "reached_targets": best.reached_targets,
            "station_count": len(best.points),
        },
    })


def main(argv=None):
    parser = argparse.ArgumentParser(description="Planejador direcional multialvo via arquivo JSON.")
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
