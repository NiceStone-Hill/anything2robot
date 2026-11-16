"""Extended Pattern Search with fixed mechanical motors, then equal-length bend polish."""

import shutil
from dataclasses import asdict, replace

import numpy as np
from common import dump, read
from routing import validate_routes
from voxel_tool.stages.placement.eps import (
    EPSWeights,
    optimize_placement_eps,
    save_eps_result,
)
from voxel_tool.stages.placement.planner import save_placement_result
from voxel_tool.stages.placement.semantic import (
    SemanticEvaluation,
    SemanticPlacementEvaluator,
)
from voxel_tool.stages.routing.incremental import IncrementalVoxelRouter
from voxel_tool.stages.routing.polish import (
    polish_routing_bends,
    save_routing_polish_report,
)
from voxel_tool.stages.routing.router import load_netlist_mapping, save_routing_result
from voxel_tool.stages.semantics.component_regions import load_component_pose_regions


def evaluator_for(config, scene, space, initial, semantics=None):
    fixed = {
        p.instance_id: p
        for p in initial.placements
        if p.component_id in config["motor_mapping"]
    }
    original = {p.instance_id: p for p in initial.placements}
    free = space["usable"] & (
        space["part_ids"] == scene["parts"].index(config["placement_part"]) + 1
    )

    def evaluate(candidate):
        violations = []
        for p in candidate.placements:
            before = original[p.instance_id]
            if p.instance_id in fixed:
                if p != fixed[p.instance_id]:
                    violations.append("fixed_motor_moved:" + p.instance_id)
                continue
            # Upstream EPS permutes voxel shapes without rotating interface metadata.
            # Only retain the certified orientation; never accept a mismatched pin pose.
            if (
                p.voxel_shape != before.voxel_shape
                or p.rotation_matrix_local_to_shell
                != before.rotation_matrix_local_to_shell
            ):
                violations.append("uncertified_rotation:" + p.instance_id)
                continue
            start = np.array(p.voxel_start)
            stop = start + p.voxel_shape
            if (
                np.any(start < 0)
                or np.any(stop > free.shape)
                or not free[
                    tuple(slice(int(a), int(b)) for a, b in zip(start, stop))
                ].all()
            ):
                violations.append("outside_available_cavity:" + p.instance_id)
        cost = 0.0
        if semantics is not None:
            movable = replace(
                candidate,
                placements=tuple(
                    p for p in candidate.placements if p.instance_id not in fixed
                ),
            )
            assessment = semantics(movable)
            violations.extend(assessment.violations)
            cost = assessment.soft_cost
        return SemanticEvaluation(not violations, cost, tuple(violations))

    return evaluate


def optimize_layout(config, scene, space, grid, placement, routing, topology, out):
    semantics = None
    if (out / "layout_constraints_raw.json").is_file():
        from semantic_bridge import validate_proposal

        movable = [
            c
            for c in read(out / "components.json")["components"]
            if c["id"] not in config["motor_mapping"]
        ]
        proposal, _ = validate_proposal(
            read(out / "layout_constraints_raw.json"), movable, config
        )
        mirror_center = tuple(
            grid.origin + (np.array(grid.labels.shape) - 1) * grid.pitch / 2
        )
        semantics = SemanticPlacementEvaluator(
            load_component_pose_regions(out / "candidate_regions.json"),
            proposal,
            mirror_center_mm=mirror_center,
        )
    evaluator = evaluator_for(config, scene, space, placement, semantics)
    for filename in ["placement.json", "routing.json"]:
        shutil.copy2(out / filename, out / ("initial_" + filename))
    router = IncrementalVoxelRouter(
        grid,
        topology,
        load_netlist_mapping(out / "circuit_mapping.json"),
        component_schema_path=out / "components.json",
        terminal_access_mode="strict",
    )
    print(
        "EPS: optimize wire length, congestion and compactness; motors locked",
        flush=True,
    )
    result = optimize_placement_eps(
        grid,
        placement,
        router,
        weights=EPSWeights(),
        step_schedule=(4, 2, 1),
        max_evaluations=config.get("eps_max_evaluations", 160),
        candidate_batch=256,
        max_accepts_per_step=5,
        semantic_evaluator=evaluator,
    )
    if semantics is not None:
        assessment = evaluator(result.placement)
        dump(out / "semantic_final_validation.json", asdict(assessment))
        if not assessment.feasible:
            raise ValueError("EPS final placement violates restricted schema")
    save_eps_result(result, out)
    save_routing_result(result.routing, out / "routing_before_polish.json")
    print(
        "EPS:",
        result.initial_score.total,
        "->",
        result.final_score.total,
        "accepted moves:",
        len(result.accepted_moves),
        flush=True,
    )
    print(
        "Polish: reduce bends with identical endpoints and branch lengths", flush=True
    )
    polished, report = polish_routing_bends(
        grid,
        result.placement,
        result.routing,
        max_states_per_branch=config.get("polish_max_states_per_branch", 250000),
    )
    save_routing_polish_report(report, out / "routing_polish_report.json")
    save_placement_result(result.placement, out / "placement.json")
    save_routing_result(polished, out / "routing.json")
    validate_routes(grid, result.placement, polished, out)
    dump(
        out / "optimization_summary.json",
        {
            "method": "Extended Pattern Search + equal-length direction-aware bend refinement",
            "eps": result.to_dict(),
            "polish": report.to_dict(),
            "fixed_motor_ids": [
                p.instance_id
                for p in placement.placements
                if p.component_id in config["motor_mapping"]
            ],
            "orientation_scope": "Certified orientations retained; translations, jumps and feasible swaps are eligible. Shape-only rotations rejected to preserve pin transforms.",
            "limits": "Finite local search; does not certify globally minimal length or bend count. Motion and real cable bend radius not modeled.",
        },
    )
    print(
        "Bends:", report.total_bends_before, "->", report.total_bends_after, flush=True
    )
    return result.placement, polished
