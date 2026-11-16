"""Reuse voxel_tool strict-anchor routing; explicit motor access ducts are geometry."""

import itertools

import numpy as np
import trimesh
from common import dump
from layout import grid_from
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
from solid_query import SolidQuery
from space import mask_mesh
from voxel_tool.domain.labels import VoxelLabel
from voxel_tool.stages.routing.router import (
    _a_star,
    load_netlist_mapping,
    route_topology,
    save_routing_result,
)


def dense_path(path):
    result = []
    for a, b in itertools.pairwise(path):
        a = np.array(a)
        b = np.array(b)
        n = int(np.max(np.abs(b - a)))
        result.extend(tuple(v) for v in np.rint(np.linspace(a, b, n + 1)).astype(int))
    return list(dict.fromkeys(result)) if len(path) > 1 else list(path)


def set_port(schema, ref, axis, sign, local_anchor):
    c = next(c for c in schema["components"] if c["id"] == ref)
    tangent = next(i for i in range(3) if i != axis)
    for i, port in enumerate(c["interfaces"]):
        point = np.array(local_anchor, dtype=float).copy()
        point[tangent] = (i - (len(c["interfaces"]) - 1) / 2) * 2.54
        normal = np.zeros(3)
        normal[axis] = sign
        port["position_mm"] = point.tolist()
        port["normal"] = normal.tolist()


def route_layout(config, space, scene, hollows, placement, schema, topology, out):
    pitch = space["pitch"]
    origin = space["origin"]
    shape = space["material"].shape
    fixed_ids = {k + "_1" for k in config["motor_mapping"]}
    free = space["usable"].copy()
    central = (
        space["part_ids"] == scene["parts"].index(config["placement_part"]) + 1
    ) & free
    # Avoid placing the lead's cavity endpoint inside an electronic component.
    occupied = np.zeros(shape, bool)
    for p in placement.placements:
        if p.instance_id not in fixed_ids:
            occupied[
                tuple(
                    slice(int(a), int(a + n))
                    for a, n in zip(p.voxel_start, p.voxel_shape)
                )
            ] = True
    targets = np.argwhere(central & ~binary_dilation(occupied, iterations=2))
    tree = cKDTree(targets)
    modifications = []
    all_channels = np.zeros(shape, bool)
    # Protected material is structural, while motor clearance includes existing air.
    # Do not drill through either; permitted surface cuts remain explicit below.
    forbidden = space["motor_block"] | space["protected_material"] | occupied
    search_allowed = ~binary_dilation(forbidden, iterations=1)
    congestion = np.where(space["material"], 30, 0).astype(np.uint16)
    for fixed in (p for p in placement.placements if p.instance_id in fixed_ids):
        print("Motor access:", fixed.display_name, flush=True)
        m = next(m for m in scene["motors"] if m["id"] == fixed.display_name)
        if len(targets) == 0:
            modifications.append(
                {"motor": fixed.display_name, "status": "no_free_cavity_target"}
            )
            continue
        motor_axis = int(np.argmax(np.abs(m["axis"])))
        found = None
        for axis in [i for i in range(3) if i != motor_axis] + [motor_axis]:
            for sign in (1, -1):
                anchor_index = np.rint(
                    (np.asarray(fixed.center_mm) - origin) / pitch
                ).astype(int)
                anchor_index[axis] = (
                    fixed.voxel_start[axis] + fixed.voxel_shape[axis]
                    if sign > 0
                    else fixed.voxel_start[axis] - 1
                )
                anchor = origin + anchor_index * pitch
                start = anchor_index.copy()
                start[axis] += sign * 2
                if (
                    np.any(start < 0)
                    or np.any(start >= shape)
                    or not search_allowed[tuple(start)]
                ):
                    continue
                # A terminal lead must emerge through actual air before any proposed hole.
                entry = np.rint(
                    np.linspace((anchor - origin) / pitch, start, 7)
                ).astype(int)
                if np.any(space["protected_material"][tuple(entry.T)]):
                    continue
                _, indices = tree.query(start, k=min(64, len(targets)))
                goals = tuple(
                    tuple(int(x) for x in v) for v in targets[np.atleast_1d(indices)]
                )
                path = _a_star(search_allowed, (tuple(start),), goals, congestion, 1.0)
                if path is None:
                    continue
                dense = dense_path(path)
                centerline = np.zeros(shape, bool)
                centerline[tuple(np.array(dense).T)] = True
                # Add source lead cells. They are only used for this component's terminal exit.
                centerline[tuple(entry.T)] = True
                channel = binary_dilation(centerline, iterations=2) & ~forbidden
                if np.any(channel & space["protected_material"]):
                    raise AssertionError("Protected mounting material would be removed")
                found = (axis, sign, channel, dense, anchor)
                break
            if found:
                break
        if found is None:
            print("No structural-safe access:", fixed.display_name, flush=True)
            modifications.append(
                {
                    "motor": fixed.display_name,
                    "status": "no_protected_structure_safe_access",
                    "reason": "No radial port and lead corridor reached the central cavity without touching mounting material",
                }
            )
            continue
        axis, sign, channel, dense, anchor = found
        set_port(
            schema, fixed.component_id, axis, sign, anchor - np.asarray(fixed.center_mm)
        )
        all_channels |= channel
        modifications.append(
            {
                "motor": fixed.display_name,
                "status": "proposed_access_duct",
                "radial_axis": axis,
                "radial_sign": sign,
                "port_anchor_mm": anchor,
                "centerline_mm": origin + np.asarray(dense) * pitch,
                "removed_conservative_material_cells": int(
                    (channel & space["material"]).sum()
                ),
                "static_only": True,
                "requires_flexible_joint_lead": True,
                "service_note": "Provisional cable-tail connector outside fixed motor envelope, not a measured motor pin; tail-to-motor attachment and motion require physical verification",
            }
        )
    print("Access corridors built, exporting modified shells", flush=True)
    if np.any(all_channels):
        # Smooth subtraction tool is smaller than this mask; reserve an additional one-cell
        # inset for the routing centerline. The output never labels the full outside as usable.
        tool = mask_mesh(all_channels, origin, pitch)
        tool.export(out / "meshes/motor_access_duct_tool_mm.stl")
        for name, hollow in list(hollows.items()):
            print("Access Boolean:", name, flush=True)
            refined = trimesh.boolean.difference(
                [hollow, tool], engine="manifold", check_volume=True
            )
            if not refined.is_volume:
                raise ValueError(f"{name}: motor access hole produced invalid shell")
            refined.export(out / "meshes" / f"{name}_hollow_with_access_mm.stl")
            hollows[name] = refined
        # Retain only the core of the physically subtracted duct as free routing space.
        # Reclassify only this bounded duct mask against the actual exported solids.
        # A simple erosion loses the one-cell transition at a component exit.
        indices = np.argwhere(all_channels)
        points = origin + indices * pitch
        clear = np.ones(len(points), bool)
        radius = config["wire_radius_mm"]
        offsets = np.vstack([np.zeros(3), np.eye(3) * radius, -np.eye(3) * radius])
        for name, shell in hollows.items():
            print("Classify actual duct air:", name, flush=True)
            query = SolidQuery(shell)
            for start in range(0, len(points), 128):
                ids = np.arange(start, min(start + 128, len(points)))
                samples = (points[ids, None, :] + offsets[None, :, :]).reshape(-1, 3)
                clear[ids] &= ~query.contains(samples).reshape(-1, len(offsets)).any(
                    axis=1
                )
        core = np.zeros(shape, bool)
        core[tuple(indices[clear].T)] = True
        free |= core
        space["material"] &= ~core
    # These connector locations are not measured pins. Place each provisional
    # cable-tail connector as a group in verified air, without moving the motor.
    for fixed in (p for p in placement.placements if p.instance_id in fixed_ids):
        record = next(r for r in modifications if r["motor"] == fixed.display_name)
        if record["status"] != "proposed_access_duct":
            continue
        component = next(
            c for c in schema["components"] if c["id"] == fixed.component_id
        )
        normal = np.asarray(component["interfaces"][0]["normal"], dtype=float)
        initial = [
            np.asarray(a["position_mm"], dtype=float) for a in component["interfaces"]
        ]
        for step in range(5):
            shift = normal * step * pitch
            indices = np.rint(
                (np.asarray(fixed.center_mm) + np.asarray(initial) + shift - origin)
                / pitch
            ).astype(int)
            if np.any(indices < 0) or np.any(indices >= shape):
                continue
            if all(
                free[tuple(idx)] and not space["motor_block"][tuple(idx)]
                for idx in indices
            ):
                for port, point in zip(component["interfaces"], initial):
                    port["position_mm"] = (point + shift).tolist()
                record["provisional_connector_outward_shift_mm"] = float(step * pitch)
                record["final_connector_center_mm"] = (
                    np.asarray(record["port_anchor_mm"]) + shift
                )
                break
        else:
            record["status"] = "connector_access_unresolved"
    dump(
        out / "motor_access.json",
        {
            "accesses": modifications,
            "note": "Narrow explicit access corridors, including external joint lead sections. No unrestricted known-air routing.",
        },
    )
    dump(out / "components.json", schema)
    grid = grid_from(space, free)
    # voxel_tool expects occupied component envelopes to have interior labels, then
    # excludes all of them in route_topology. This is not placement free-space.
    for p in placement.placements:
        slices = tuple(
            slice(int(a), int(a + n)) for a, n in zip(p.voxel_start, p.voxel_shape)
        )
        grid.labels[slices] = int(VoxelLabel.INTERIOR_AIR)
    print("voxel_tool strict-anchor A*", flush=True)
    routing = route_topology(
        grid,
        placement,
        topology,
        load_netlist_mapping(out / "circuit_mapping.json"),
        component_schema_path=out / "components.json",
        terminal_access_mode="strict",
    )
    save_routing_result(routing, out / "routing.json")
    validate_routes(grid, placement, routing, out)
    return grid, routing


def validate_routes(grid, placement, routing, out):
    # Check every cell along simplified polyline segments, not just A* corner vertices.
    routing_occupied = np.zeros(grid.labels.shape, bool)
    for p in placement.placements:
        routing_occupied[
            tuple(
                slice(int(a), int(a + n)) for a, n in zip(p.voxel_start, p.voxel_shape)
            )
        ] = True
    checks = []
    for r in routing.routes:
        cells = dense_path(r.voxel_path)
        valid = all(
            grid.labels[p] == int(VoxelLabel.INTERIOR_AIR) and not routing_occupied[p]
            for p in cells
        )
        if not valid:
            raise AssertionError("Route crosses blocked cells")
        checks.append(
            {
                "branch": r.branch_id,
                "all_voxel_segments_clear": valid,
                "cells": len(cells),
            }
        )
    dump(
        out / "routing_validation.json",
        {
            "branches": checks,
            "unrouted": list(routing.unrouted),
            "endpoint_fallback_used": any(
                "fallback" in x.source_endpoint_kind
                or "fallback" in x.target_endpoint_kind
                for x in routing.routes
            ),
            "unresolved": [
                "Continuous joint motion",
                "Minimum cable bend radius",
                "Multi-net bundle diameter",
                "Measured motor connector locations",
            ],
        },
    )
