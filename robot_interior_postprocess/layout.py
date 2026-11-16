import hashlib
import json
from dataclasses import asdict

import numpy as np
from common import dump
from voxel_tool.domain.labels import VoxelLabel
from voxel_tool.domain.voxel import VoxelGrid
from voxel_tool.stages.placement.planner import (
    Placement,
    PlacementResult,
    save_placement_result,
)
from voxel_tool.stages.placement.semantic import (
    SemanticPlacementEvaluator,
    plan_from_pose_regions,
)
from voxel_tool.stages.semantics.component_regions import (
    ComponentRegionPolicy,
    compile_component_pose_regions,
    load_component_pose_regions,
    save_component_pose_regions,
)


def grid_from(space, free):
    labels = np.full(free.shape, int(VoxelLabel.EXTERIOR_AIR), np.uint8)
    labels[space["material"] | space["motor_block"]] = int(VoxelLabel.SHELL_MATERIAL)
    labels[free] = int(VoxelLabel.INTERIOR_AIR)
    return VoxelGrid(
        space["material"] | space["motor_block"],
        labels,
        space["pitch"],
        space["origin"],
    )


def fixed_motors(scene, config, space):
    fixed = []
    lookup = {m["id"]: m for m in scene["motors"]}
    for circuit_id, motor_id in config["motor_mapping"].items():
        m = lookup[motor_id]
        a = np.array(m["axial_start_mm"])
        b = np.array(m["axial_end_mm"])
        axis = np.abs((b - a) / np.linalg.norm(b - a))
        if not np.isclose(axis.max(), 1):
            raise ValueError("First adapter requires cardinal motor cylinder axes")
        dimensions = np.full(3, 2 * (m["radius_mm"] + config["motor_clearance_mm"]))
        dimensions[int(np.argmax(axis))] = (
            np.linalg.norm(b - a) + 2 * config["motor_clearance_mm"]
        )
        center = np.asarray(m["center_mm"], dtype=float)
        start = np.floor(
            (center - dimensions / 2 - space["origin"]) / space["pitch"]
        ).astype(int)
        stop = (
            np.ceil(
                (center + dimensions / 2 - space["origin"]) / space["pitch"]
            ).astype(int)
            + 1
        )
        shape = stop - start
        # Mechanical pose stays exact. Only its conservative collision mask is snapped.
        fixed.append(
            Placement(
                circuit_id + "_1",
                circuit_id,
                motor_id,
                tuple(float(x) for x in center),
                tuple(float(x) for x in dimensions),
                tuple(int(x) for x in start),
                tuple(int(x) for x in shape),
                (0.8, 0.2, 0.15),
                ((1, 0, 0), (0, 1, 0), (0, 0, 1)),
            )
        )
        slices = tuple(slice(int(s), int(s + n)) for s, n in zip(start, shape))
        space["motor_block"][slices] = True
        space["usable"][slices] = False
    return fixed


def place_components(config, schema, space, scene, fixed, out):
    part_index = scene["parts"].index(config["placement_part"]) + 1
    free = space["usable"] & (space["part_ids"] == part_index)
    grid = grid_from(space, free)
    movable = [
        c for c in schema["components"] if c["id"] not in config["motor_mapping"]
    ]
    policies = {
        c["id"]: ComponentRegionPolicy(
            c["id"],
            c["interfaces"][0]["id"],
            ((0, 1, 0),),
            clearance_mm=config["component_clearance_mm"],
            interface_access_mm=9,
            max_candidates=128,
            vertical_preference="low" if c["id"] == "BATTERY" else "neutral",
            evidence="Only real cavity cells in selected part, excluding locked motor envelopes",
            pending_constraints=(
                "Measured hardware geometry",
                "Insertion/removal and cover access",
                "Full motion sweep",
            ),
        )
        for c in movable
    }
    from semantic_bridge import prepare_constraints

    proposal = prepare_constraints(config, schema, grid, space, scene, fixed, free, out)
    if proposal is not None:
        policies = {p.component_id: p for p in proposal.policies}
    mirror_center = tuple(
        grid.origin + (np.array(grid.labels.shape) - 1) * grid.pitch / 2
    )
    print("Compile candidate poses", flush=True)
    identity = hashlib.sha256(
        free.tobytes()
        + json.dumps(
            {
                "components": movable,
                "policies": [asdict(p) for p in policies.values()],
                "origin": space["origin"].tolist(),
                "pitch": space["pitch"],
            },
            sort_keys=True,
        ).encode()
    ).hexdigest()
    cache = out / "candidate_cache_key.txt"
    if (
        cache.exists()
        and cache.read_text() == identity
        and (out / "candidate_regions.json").is_file()
    ):
        regions = load_component_pose_regions(out / "candidate_regions.json")
    else:
        regions = compile_component_pose_regions(
            grid, movable, policies=tuple(policies.values())
        )
        save_component_pose_regions(regions, out / "candidate_regions.json")
        cache.write_text(identity)
    print("Joint placement search", flush=True)
    placed = plan_from_pose_regions(
        regions,
        clearance_mm=config["component_clearance_mm"],
        beam_width=32,
        candidates_per_instance=96,
        proposal=proposal,
        mirror_center_mm=mirror_center,
    )
    if proposal is not None:
        semantic = SemanticPlacementEvaluator(
            regions, proposal, mirror_center_mm=mirror_center
        )(placed)
        dump(out / "semantic_initial_validation.json", asdict(semantic))
        if placed.unplaced or not semantic.feasible:
            raise ValueError(
                "No schema-feasible initial layout; inspect candidate regions and semantic_initial_validation.json. Constraints were not relaxed."
            )
    result = PlacementResult(
        tuple(fixed) + placed.placements, placed.unplaced, placed.clearance_mm
    )
    save_placement_result(result, out / "placement.json")
    dump(
        out / "placement_audit.json",
        {
            "fixed_motor_ids": [p.instance_id for p in fixed],
            "movable_ids": [p.instance_id for p in placed.placements],
            "unplaced": list(placed.unplaced),
            "candidate_counts": {
                r.component_id: len(r.candidates) for r in regions.regions
            },
            "unresolved": [
                "Installation/cover access",
                "Switch external actuator opening",
                "Full motion envelope",
            ],
        },
    )
    return result, regions
