"""Subtract round wire grooves from an actual cactus cover and export its crop.

The trial retains source curvature and exterior; it does not synthesize a plate,
add raceway walls, or replace the electrical routing of the assembled robot.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import heapq
import io
import json
import math
from itertools import pairwise
from pathlib import Path

import manifold3d as md
import numpy as np
import trimesh
from printed_labels import FONT, box, union
from shell_partition.supports import mesh_of, rod, solid

# Local axes: u=-world X, v=world Z, w=world Y (towards the open carrier half).
LOCAL_FROM_WORLD = np.array(
    [[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float
)


class InnerWall:
    """Trace the cover exterior and cavity boundary on a declared visible side."""

    def __init__(self, source):
        self.source = source
        self.cache = {}

    def at(self, u, v):
        key = (round(float(u), 7), round(float(v), 7))
        if key not in self.cache:
            hits = self.source.ray_cast([u, v, -250], [u, v, 50])
            positions = sorted({round(float(h.position[2]), 7) for h in hits})
            if len(positions) < 2 or len(positions) % 2:
                raise ValueError(
                    f"Trial needs one unobstructed wall at {key}; got {positions}"
                )
            # The last interval faces the cavity opening. The first may be an
            # exterior fold hidden behind another layer of the original shell.
            self.cache[key] = positions[-2:]
        return self.cache[key]


def sweep(points, radius):
    return union(
        [rod(a, b, radius) for a, b in pairwise(points)]
        + [
            md.Manifold.sphere(radius, circular_segments=24).translate(p)
            for p in points
        ]
    )


def open_sweep(points, radius, travel=25):
    """Round-bottom channel open towards the cavity, including between samples."""
    pieces = []
    for a, b in pairwise(points):
        segment = sweep([a, b], radius)
        pieces.append((segment + segment.translate([0, 0, travel])).hull())
    return union(pieces)


def engraved_label(wall, label, u, v, depth=0.4, pixel=0.5):
    """Each pixel conforms to the actual inner wall instead of a tangent plane."""
    cells = []
    for index, char in enumerate(label):
        for row, values in enumerate(FONT[char]):
            for col, occupied in enumerate(values):
                if occupied != "1":
                    continue
                x = u + (index * 6 + col) * pixel
                y = v + (6 - row) * pixel
                samples = [
                    wall.at(x + dx, y + dy)[1]
                    for dx in (0, pixel / 2, pixel + 0.015)
                    for dy in (0, pixel / 2, pixel + 0.015)
                ]
                z0, z1 = min(samples) - depth, max(samples) + 0.5
                cells.append(box([pixel + 0.015, pixel + 0.015, z1 - z0], [x, y, z0]))
    return union(cells)


def surface_path(wall, ends, diameter, depth, bounds, mounts, previous):
    """Search the visible inner surface, excluding hardware, mounts and prior wires."""
    from scipy.spatial import cKDTree

    tree = cKDTree(np.vstack(previous)) if previous else None
    radius = diameter / 2 + 0.25

    def point(uv):
        outer, inner = wall.at(*uv)
        return np.array([*uv, inner + radius - depth]), inner - outer

    cache = {}

    def allowed(uv):
        key = tuple(float(v) for v in uv)
        if key in cache:
            return cache[key]
        try:
            p, thickness = point(key)
            okay = thickness >= depth + 1.8
            for lo, hi in bounds:
                okay &= not np.all((p >= lo - 2) & (p <= hi + 2))
            if okay and tree is not None:
                okay &= tree.query(p)[0] >= diameter + 0.8
            if okay:
                # Reserve the complete cutter footprint, including adjacent ribs.
                for du, dv in (
                    (0, 0),
                    (1.7, 0),
                    (-1.7, 0),
                    (0, 1.7),
                    (0, -1.7),
                    (1.2, 1.2),
                    (-1.2, 1.2),
                    (1.2, -1.2),
                    (-1.2, -1.2),
                ):
                    q = p + [du, dv, 0]
                    if mounts.ray_cast(q - [0, 0, depth + 2], q + [0, 0, 25]):
                        okay = False
                        break
            cache[key] = bool(okay)
        except ValueError:
            cache[key] = False
        return cache[key]

    step, u0 = 2.0, -60.0
    v0 = math.floor((min(ends[:, 1]) - 12) / step) * step
    vmax = max(ends[:, 1]) + 12
    uv_of = lambda n: np.array([u0 + n[0] * step, v0 + n[1] * step])
    nodes = [tuple(np.rint((p[:2] - [u0, v0]) / step).astype(int)) for p in ends]
    start, goal = nodes
    queue = [(0.0, start)]
    cost, parent = {start: 0.0}, {}
    for end in ends:
        if not allowed(end[:2]):
            raise ValueError(f"Port projection has no safe inner-wall entry: {end[:2]}")
    while queue:
        _, node = heapq.heappop(queue)
        if node == goal:
            break
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (-1, -1),
            (1, -1),
            (-1, 1),
        ):
            nxt = (node[0] + dx, node[1] + dy)
            uv = uv_of(nxt)
            if not (-60 <= uv[0] <= 60 and v0 <= uv[1] <= vmax) or not allowed(uv):
                continue
            if (
                dx
                and dy
                and not (
                    allowed(uv_of((nxt[0], node[1])))
                    and allowed(uv_of((node[0], nxt[1])))
                )
            ):
                continue
            distance = np.linalg.norm(point(uv)[0] - point(uv_of(node))[0])
            tentative = cost[node] + distance
            if tentative < cost.get(nxt, float("inf")):
                cost[nxt], parent[nxt] = tentative, node
                heapq.heappush(
                    queue, (tentative + np.linalg.norm(uv - uv_of(goal)), nxt)
                )
    if goal not in cost:
        raise ValueError(
            "No inner-wall path avoids hardware, mounts and previous wires"
        )
    path, node = [], goal
    while node != start:
        path.append(uv_of(node))
        node = parent[node]
    path.append(uv_of(start))
    path = [ends[0, :2], *path[::-1], ends[1, :2]]
    # Remove redundant grid turns only where the entire interpolated surface is feasible.
    simple, i = [path[0]], 0
    while i < len(path) - 1:
        for j in range(len(path) - 1, i, -1):
            samples = np.linspace(
                path[i],
                path[j],
                max(2, math.ceil(np.linalg.norm(path[j] - path[i]) / 0.5) + 1),
            )
            if all(allowed(p) for p in samples):
                simple.append(path[j])
                i = j
                break
        else:
            raise ValueError("Surface route refinement failed")
    dense = []
    for a, b in pairwise(simple):
        dense.extend(
            np.linspace(a, b, max(2, math.ceil(np.linalg.norm(b - a) / 0.5) + 1))[:-1]
        )
    dense.append(simple[-1])
    return np.array([point(uv)[0] for uv in dense])


def groove_path(wall, route, diameter, depth, bounds, mounts, previous):
    # Preserve the recorded connector anchors and lead-out normals exactly.
    anchors_world = np.array([route["source_anchor_mm"], route["target_anchor_mm"]])
    normals_world = np.array([route["source_normal"], route["target_normal"]])
    anchors = trimesh.transform_points(anchors_world, LOCAL_FROM_WORLD)
    lead_ends = trimesh.transform_points(
        anchors_world + 5 * normals_world, LOCAL_FROM_WORLD
    )
    cutter_radius = diameter / 2 + 0.25
    center = surface_path(wall, lead_ends, diameter, depth, bounds, mounts, previous)
    cutter = open_sweep(center, cutter_radius)
    whole_path = np.vstack((anchors[0], lead_ends[0], center, lead_ends[1], anchors[1]))
    cable = sweep(whole_path, diameter / 2)
    return center, whole_path, cutter, cable


def source_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clipped_length(points, bounds):
    """Exact centerline length within an AABB, without bridging excluded pieces."""
    total = 0.0
    for a, b in pairwise(np.asarray(points, float)):
        direction = b - a
        lo, hi = 0.0, 1.0
        for axis in range(3):
            if abs(direction[axis]) < 1e-12:
                if not bounds[0][axis] <= a[axis] <= bounds[1][axis]:
                    hi = -1
                    break
            else:
                limits = sorted(
                    (bound[axis] - a[axis]) / direction[axis] for bound in bounds
                )
                lo, hi = max(lo, limits[0]), min(hi, limits[1])
        total += max(0, hi - lo) * np.linalg.norm(direction)
    return float(total)


def export_checked_stl(value, path, forbidden=None):
    """Keep double precision through ASCII STL and verify the actual reloaded file."""
    repairs = []
    tolerance = 0.0001
    # Re-import the native double-precision mesh to normalize Boolean provenance
    # before simplification, while preserving enclosed void boundaries.
    original = value.to_mesh64()
    value = md.Manifold(
        md.Mesh64(
            np.array(
                np.asarray(original.vert_properties)[:, :3],
                dtype=np.float64,
                order="C",
                copy=True,
            ),
            np.array(original.tri_verts, dtype=np.uint64, order="C", copy=True),
        )
    )
    if value.status() != md.Error.NoError:
        raise ValueError(f"Invalid native mesh before STL export: {path.name}")
    for _attempt in range(4):
        refined = value.simplify(tolerance)
        data = refined.to_mesh64()
        candidate = trimesh.Trimesh(
            np.asarray(data.vert_properties)[:, :3],
            np.asarray(data.tri_verts),
            process=True,
        )
        candidate.update_faces(candidate.nondegenerate_faces(height=1e-9))
        candidate.update_faces(candidate.unique_faces())
        candidate.remove_unreferenced_vertices()
        encoded = trimesh.exchange.stl.export_stl_ascii(candidate).encode("ascii")
        checked = trimesh.load(io.BytesIO(encoded), file_type="stl", force="mesh")
        if checked.is_volume and checked.is_watertight:
            path.write_bytes(encoded)
            return {
                "format": "ASCII STL with double-precision vertices",
                "simplification_tolerance_mm": tolerance,
                "reloaded_watertight": True,
                "reloaded_positive_volume": True,
                "faces": len(checked.faces),
                "subtractive_contact_repairs": repairs,
                "maximum_local_export_allowance_mm": len(repairs) * 0.04
                + (_attempt + 1) * tolerance,
            }
        counts = np.bincount(checked.edges_unique_inverse)
        edges = checked.vertices[checked.edges_unique[np.where(counts != 2)[0]]]
        if (
            not len(edges)
            or len(edges) > 100
            or np.linalg.norm(edges[:, 0] - edges[:, 1], axis=1).max() > 2
        ):
            break
        # Sub-voxel Boolean tangencies can create coincident sheets. Open only
        # their tiny contact edges with a 0.04 mm subtraction, never fill a hole.
        repair_tool = union(
            [
                (
                    md.Manifold.sphere(0.04, circular_segments=24).translate(a)
                    + md.Manifold.sphere(0.04, circular_segments=24).translate(b)
                ).hull()
                for a, b in edges
            ]
        )
        if forbidden is not None and (repair_tool ^ forbidden).volume() > 0.001:
            raise ValueError("Export cleanup would affect a protected mounting feature")
        repairs.append({"radius_mm": 0.04, "contact_edges_mm": edges.tolist()})
        value = refined - repair_tool
    raise ValueError(f"Exported STL topology is invalid: {path.name}")


def discard_zero_volume_artifacts(value):
    """Boolean tangencies can leave disconnected, exactly zero-volume triangles."""
    pieces = value.decompose()
    keep = [p for p in pieces if abs(p.volume()) > 1e-8]
    if len(keep) != 1 or keep[0].volume() <= 0:
        # Recomposition can fill negative-oriented enclosed void boundaries.
        return value, 0
    return keep[0], len(pieces) - len(keep)


def generate(source_dir, output):
    source_dir, output = Path(source_dir).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new or empty output directory")
    source_path = source_dir / "meshes/base_piece_1.stl"
    exterior_path = source_dir / "meshes/base_solid_assembly_mm.stl"
    raw = trimesh.load(source_path, force="mesh")
    raw.apply_transform(LOCAL_FROM_WORLD)
    source = solid(raw)
    wall = InnerWall(source)
    original_hollow = trimesh.load(
        source_dir / "meshes/base_hollow_assembly_mm.stl", force="mesh"
    )
    original_hollow.apply_transform(LOCAL_FROM_WORLD)
    added_mounts = source - solid(original_hollow)
    planning_bounds = []
    for placement in json.loads((source_dir / "carrier_placement.json").read_text())[
        "placements"
    ]:
        b = np.asarray(placement["bounds_mm"])
        m = trimesh.creation.box(b[1] - b[0])
        m.apply_translation(b.mean(axis=0))
        m.apply_transform(LOCAL_FROM_WORLD)
        planning_bounds.append(m.bounds)
    routing_path = source_dir / "electrical/attempt_1/routing.json"
    routing = json.loads(routing_path.read_text())
    by_branch = {r["branch_id"]: r for r in routing["routes"]}
    tools, cables, records = [], [], []
    for i, (branch, source_label, target_label) in enumerate(
        (("P1-:1", "REG.GND", "U1.GND"), ("P1+:1", "REG.VOUT", "S1.P")), 1
    ):
        print(f"Plan real connection: {branch}", flush=True)
        route = by_branch[branch]
        diameter, depth = 2.0, 1.0
        center, whole_path, tool, cable = groove_path(
            wall,
            route,
            diameter,
            depth,
            planning_bounds,
            added_mounts,
            [np.asarray(r["centerline_local_mm"]) for r in records],
        )
        groove_mount_overlap = float((source ^ tool ^ added_mounts).volume())
        if groove_mount_overlap > 0.001:
            raise ValueError(
                f"{branch} groove removes existing mounting material: {groove_mount_overlap}"
            )
        tools.append(tool)
        cables.append(cable)
        length = float(np.linalg.norm(np.diff(center, axis=0), axis=1).sum())
        records.append(
            {
                "id": f"W0{i}",
                "start": source_label,
                "end": target_label,
                "source_branch_id": branch,
                "net": route["net_name"],
                "source_instance": route["source_instance"],
                "source_pin": route["source_pin"],
                "target_instance": route["target_instance"],
                "target_pin": route["target_pin"],
                "source_anchor_world_mm": route["source_anchor_mm"],
                "target_anchor_world_mm": route["target_anchor_mm"],
                "endpoint_accuracy": routing["endpoint_accuracy"],
                "whole_path_local_mm": whole_path.tolist(),
                "whole_geometric_path_mm": float(
                    np.linalg.norm(np.diff(whole_path, axis=0), axis=1).sum()
                ),
                "diameter_assumption_mm": diameter,
                "cutter_radius_mm": diameter / 2 + 0.25,
                "nominal_embed_along_w_mm": depth,
                "radial_clearance_mm": 0.25,
                "centerline_local_mm": center.tolist(),
                "whole_groove_centerline_mm": length,
                "trial_blank_with_25mm_end_tails_mm": math.ceil(length + 50),
            }
        )
        label_specs = (
            [("W01 REG.GND", -12, 66), ("TO U1.GND", -12, 72), ("W01 U1.GND", 14, 114)]
            if i == 1
            else [("W02 REG.VOUT", -8, 56), ("TO S1.P", -8, 50), ("W02 S1.P", -10, -80)]
        )
        records[-1]["labels"] = [
            {"text": t, "local_uv_mm": [x, y]} for t, x, y in label_specs
        ]
        for text, x, y in label_specs:
            label = engraved_label(wall, text, x, y)
            label_overlap = float((source ^ label ^ added_mounts).volume())
            if label_overlap > 0.001:
                raise ValueError(
                    f"Label {text} removes existing mounting material: {label_overlap}"
                )
            tools.append(label)
    tool = union(tools)
    removed = source ^ tool
    modified = source - tool
    # This is an intersection crop of the actual curved shell, with no added material.
    crop = box([63, 65, 88], [-35, 45, -150])
    before_crop, after_crop = source ^ crop, modified ^ crop
    after_crop, numerical_artifacts = discard_zero_volume_artifacts(after_crop)
    if not all(
        mesh_of(s).is_volume for s in (source, modified, before_crop, after_crop)
    ):
        raise ValueError(
            "Source, modified shell and crop must remain closed positive solids"
        )
    if sum(p.volume() > 1e-8 for p in after_crop.decompose()) != 1:
        raise ValueError("Selected shell crop is disconnected")
    added = float((modified - source).volume())
    intersections = {
        r["id"]: float((c ^ modified).volume()) for r, c in zip(records, cables)
    }
    wire_crossing = float((cables[0] ^ cables[1]).volume())
    if wire_crossing > 0.001:
        raise ValueError(f"Different electrical nets intersect: {wire_crossing}")
    if added > 1e-5 or any(v > 0.001 for v in intersections.values()):
        raise ValueError(
            f"Unexpected added material or cable collisions: {added}, {intersections}"
        )
    # Include installed carrier envelopes, exact carrier plates and the other shells.
    obstacle_checks = {}
    placements = json.loads((source_dir / "carrier_placement.json").read_text())[
        "placements"
    ]
    obstacles = []
    for placement in placements:
        bounds = np.asarray(placement["bounds_mm"])
        envelope = box((bounds[1] - bounds[0]).tolist(), bounds[0].tolist())
        obstacles.append((placement["id"], envelope.transform(LOCAL_FROM_WORLD[:3])))
    for path in sorted((source_dir / "meshes").glob("*_carrier.stl")):
        part = trimesh.load(path, force="mesh")
        part.apply_transform(LOCAL_FROM_WORLD)
        obstacles.append((path.stem, solid(part)))
    for path in sorted((source_dir / "meshes").glob("*_piece_*.stl")):
        if path == source_path:
            continue
        part = trimesh.load(path, force="mesh")
        part.apply_transform(LOCAL_FROM_WORLD)
        obstacles.append((path.stem, solid(part)))
    for r, cable in zip(records, cables):
        for name, obstacle in obstacles:
            volume = float((cable ^ obstacle).volume())
            obstacle_checks[f"{r['id']}:{name}"] = volume
            if volume > 0.001:
                raise ValueError(
                    f"New route {r['id']} collides with {name}: {volume:.3f} mm3"
                )
    original_hollow = trimesh.load(
        source_dir / "meshes/base_hollow_assembly_mm.stl", force="mesh"
    )
    original_hollow.apply_transform(LOCAL_FROM_WORLD)
    added_mounts = source - solid(original_hollow)
    mount_cut_volume = float((removed ^ added_mounts).volume())
    if mount_cut_volume > 0.001:
        raise ValueError(
            f"Grooves would cut an existing mount/closure feature: {mount_cut_volume}"
        )
    # Test vertical lay-in/removal in the open half-shell with conservative cable columns.
    lay_in = {}
    for r in records:
        p = np.asarray(r["centerline_local_mm"])
        volume = float(
            (open_sweep(p, r["diameter_assumption_mm"] / 2) ^ modified).volume()
        )
        lay_in[r["id"]] = volume
        if volume > 0.001:
            raise ValueError(
                f"{r['id']} cannot be laid in from the cover opening: {volume}"
            )
    # Report a sampled Euclidean exterior clearance, not merely axis thickness.
    outside = trimesh.load(exterior_path, force="mesh")
    outside.apply_transform(LOCAL_FROM_WORLD)
    rm = mesh_of(removed)
    samples = np.vstack((rm.vertices, rm.triangles_center))
    _, distances, _ = trimesh.proximity.closest_point(outside, samples)
    remaining_wall = float(distances.min())
    if remaining_wall < 1.5:
        raise ValueError(
            f"Sampled remaining wall {remaining_wall:.3f} mm is below 1.5 mm"
        )
    # Preserve all pre-existing features: no cut is allowed inside declared protected cells.
    masks = np.load(source_dir / "space_masks.npz")
    world_samples = trimesh.transform_points(samples, np.linalg.inv(LOCAL_FROM_WORLD))
    indices = np.rint((world_samples - masks["origin"]) / float(masks["pitch"])).astype(
        int
    )
    shape = masks["protected_material"].shape
    valid = np.all((indices >= 0) & (indices < shape), axis=1)
    ids = indices[valid]
    protected_count = int(masks["protected_material"][tuple(ids.T)].sum())
    motor_count = int(masks["motor_block"][tuple(ids.T)].sum())
    if protected_count or motor_count:
        raise ValueError("Trial removal touches structural/motor protection cells")
    # Both crops use exactly the same rigid print transform; curvature is preserved.
    print_from_local = np.eye(4)
    print_from_local[:3, 3] = -mesh_of(before_crop).bounds[0]
    print_from_world = print_from_local @ LOCAL_FROM_WORLD
    source_manifest = json.loads((source_dir / "manifest.json").read_text())
    report = {
        "status": "subtractive_static_trial_geometry_passed_physical_fit_pending",
        "source_manifest_status": source_manifest["status"],
        "source_complete_assemblable": source_manifest["complete_assemblable"],
        "source": str(source_path),
        "source_sha256": source_hash(source_path),
        "routing_source": str(routing_path),
        "routing_sha256": source_hash(routing_path),
        "units": "mm",
        "method": "subtract from existing curved interior; no added walls or plate",
        "removed_volume_mm3": float(removed.volume()),
        "added_volume_mm3": added,
        "discarded_zero_volume_boolean_artifacts": numerical_artifacts,
        "remaining_wall_sampled_min_mm": remaining_wall,
        "wall_check_samples": len(samples),
        "minimum_wall_requirement_mm": 1.5,
        "wall_check_scope": "nearest distance of removed-surface vertices/centroids to original exterior; sampled, not continuous thickness/strength proof",
        "protected_cell_hits": protected_count,
        "motor_cell_hits": motor_count,
        "cable_vs_modified_shell_mm3": intersections,
        "different_net_cable_intersection_mm3": wire_crossing,
        "cable_vs_installed_obstacles_mm3": obstacle_checks,
        "existing_mount_material_removed_mm3": mount_cut_volume,
        "vertical_lay_in_intersection_mm3": lay_in,
        "crop_bounds_local_mm": mesh_of(before_crop).bounds.tolist(),
        "print_from_world": print_from_world.tolist(),
        "trial_scope": "Two source-netlist branches rerouted along the cactus carrier half interior; endpoint identity and model anchors preserved; anchors are not measured pins",
        "pending": [
            "printed fit and deburring",
            "actual cable bend ratings",
            "retention clips",
            "replacement of full electrical routing",
            "whole-robot dynamic and electrical validation",
        ],
    }
    output.mkdir(parents=True, exist_ok=True)
    export_report = {}
    for name, value in (
        ("cactus_inner_groove_sample_mm.stl", after_crop),
        ("cactus_original_sample_reference_mm.stl", before_crop),
    ):
        export_report[name] = export_checked_stl(
            value.transform(print_from_local[:3]),
            output / name,
            added_mounts.transform(print_from_local[:3]),
        )
    world_from_local = np.linalg.inv(LOCAL_FROM_WORLD)
    full = mesh_of(modified)
    full.apply_transform(world_from_local)
    export_report["base_piece_1_inner_grooves_world_mm.stl"] = export_checked_stl(
        modified.transform(world_from_local[:3]),
        output / "base_piece_1_inner_grooves_world_mm.stl",
        added_mounts.transform(world_from_local[:3]),
    )
    report["stl_export_checks"] = export_report
    export_allowance = max(
        v["maximum_local_export_allowance_mm"] for v in export_report.values()
    )
    report["remaining_wall_conservative_estimate_after_export_mm"] = (
        remaining_wall - export_allowance
    )
    if remaining_wall - export_allowance < 1.5:
        raise ValueError("Remaining wall estimate after export cleanup is below 1.5 mm")
    for r in records:
        p = np.asarray(r["centerline_local_mm"])
        r["sample_path_length_mm"] = clipped_length(
            p, [[-35, 45, -150], [28, 110, -62]]
        )
        r["sample_trial_blank_mm"] = math.ceil(r["sample_path_length_mm"] + 50)
    spec = {
        "schema_version": 1,
        "units": "mm",
        "routes": records,
        "diameter_provenance": "2mm diameter retained from source routing radius_mm=1; verify actual wire OD",
        "ends": "Source netlist instance/pin IDs and model anchors; REG abbreviates REGULATOR_1, U1 means U1_1, S1 means S1_1",
        "cut_list_scope": "unterminated fitting samples only; complete geometric lengths also recorded, maintenance/termination allowance not certified",
    }
    for filename, data in (("groove_spec.json", spec), ("validation.json", report)):
        (output / filename).write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n"
        )
    with (output / "wire_trial_lengths.csv").open(
        "w", newline="", encoding="utf-8-sig"
    ) as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "id",
                "start",
                "end",
                "diameter_assumption_mm",
                "embed_along_w_mm",
                "sample_path_mm",
                "sample_trial_blank_mm",
                "status",
            ]
        )
        for r in records:
            writer.writerow(
                [
                    r["id"],
                    r["start"],
                    r["end"],
                    r["diameter_assumption_mm"],
                    r["nominal_embed_along_w_mm"],
                    round(r["sample_path_length_mm"], 3),
                    r["sample_trial_blank_mm"],
                    "fit_trial_only_25mm_each_end_no_termination_allowance",
                ]
            )
    scene = trimesh.Scene()
    for path in sorted((source_dir / "meshes").glob("*_piece_*.stl")):
        part = full.copy() if path == source_path else trimesh.load(path, force="mesh")
        part.visual.face_colors = (
            [50, 170, 175, 255] if path == source_path else [190, 199, 205, 95]
        )
        scene.add_geometry(part, node_name=path.stem)
    for r, cable in zip(records, cables):
        m = mesh_of(cable)
        m.apply_transform(world_from_local)
        m.visual.face_colors = (
            [235, 155, 40, 255] if r["id"] == "W01" else [190, 75, 110, 255]
        )
        scene.add_geometry(m, node_name=r["id"] + "_" + r["start"] + "_to_" + r["end"])
    for placement in placements:
        b = np.asarray(placement["bounds_mm"])
        proxy = trimesh.creation.box(b[1] - b[0])
        proxy.apply_translation(b.mean(axis=0))
        proxy.visual.face_colors = [95, 115, 150, 130]
        scene.add_geometry(proxy, node_name=placement["id"] + "_planning_envelope")
    scene.export(output / "cactus_shell_context.glb")
    render_preview(
        source_dir,
        source,
        modified,
        before_crop,
        after_crop,
        crop,
        cables,
        output,
        report["remaining_wall_conservative_estimate_after_export_mm"],
    )
    (output / "README.md").write_text(guide(report, records), encoding="utf-8")
    # Printable STL package contains the modified sample only, no reference duplicates.
    import zipfile

    with zipfile.ZipFile(
        output / "cactus_groove_print_trial.zip", "w", compression=zipfile.ZIP_DEFLATED
    ) as z:
        for filename in (
            "cactus_inner_groove_sample_mm.stl",
            "README.md",
            "wire_trial_lengths.csv",
            "validation.json",
            "preview.png",
        ):
            z.write(output / filename, filename)
    if source_hash(source_path) != report["source_sha256"]:
        raise AssertionError("Source shell unexpectedly changed")
    print(
        json.dumps(
            {
                "output": str(output),
                "remaining_wall_sampled_mm": remaining_wall,
                "sample_bounds_mm": mesh_of(
                    after_crop.transform(print_from_local[:3])
                ).bounds.tolist(),
                "source_unchanged": True,
                "status": report["status"],
            },
            indent=2,
        )
    )


def render_preview(
    source_dir, source, modified, before, after, crop, cables, output, wall
):
    from PIL import Image, ImageDraw, ImageFont

    im = Image.new("RGB", (1800, 1120), "#f2f5f6")
    draw = ImageDraw.Draw(im)
    fp = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    title, font, small = [ImageFont.truetype(fp, size) for size in (30, 21, 17)]
    draw.text(
        (40, 25),
        "CACTUS / GROOVES CUT INTO THE EXISTING INNER SHELL",
        font=title,
        fill="#163849",
    )
    draw.text(
        (40, 72),
        "Actual netlist anchors -> carrier-half inner grooves -> actual destination anchors.",
        font=font,
        fill="#426270",
    )

    def render(items, rect, basis):
        triangles, normals, colors = [], [], []
        for m, color in items:
            triangles.append(m.triangles)
            normals.append(m.face_normals)
            colors.extend([color] * len(m.faces))
        tris, norms = np.vstack(triangles), np.vstack(normals)
        b = np.asarray(basis, float)
        b /= np.linalg.norm(b, axis=1)[:, None]
        pts = tris @ b[:2].T
        lo, hi = pts.min(axis=(0, 1)), pts.max(axis=(0, 1))
        x, y, w, h = rect
        scale = min(w / (hi[0] - lo[0]), h / (hi[1] - lo[1]))
        pts = (pts - (lo + hi) / 2) * [scale, -scale] + [x + w / 2, y + h / 2]
        depth = tris.mean(axis=1) @ np.cross(b[0], b[1])
        light = np.array([-0.4, -0.25, 0.9])
        light /= np.linalg.norm(light)
        for index in np.argsort(depth):
            shade = 0.42 + 0.58 * max(0, float(norms[index] @ light))
            draw.polygon(
                [tuple(p) for p in pts[index]],
                fill=tuple(int(c * shade) for c in colors[index]),
            )

    # Whole cactus context: show its shell geometry and highlight the selected cover.
    context = []
    for p in sorted((source_dir / "meshes").glob("*_piece_*.stl")):
        if p.name == "base_piece_2.stl":
            continue
        m = trimesh.load(p, force="mesh")
        m.apply_transform(LOCAL_FROM_WORLD)
        context.append(
            (m, (70, 174, 176) if p.name == "base_piece_1.stl" else (194, 205, 211))
        )
    # A colored patch floating slightly above the real selected wall locates the crop.
    marked = mesh_of(before).copy()
    marked.vertices[:, 2] += 0.5
    context.append((marked, (238, 143, 53)))
    context.extend(
        (mesh_of(c), color)
        for c, color in zip(cables, [(228, 151, 44), (193, 93, 127)])
    )
    render(context, (50, 170, 460, 690), [[0.92, 0, -0.38], [0.08, 1, 0.2]])
    draw.text(
        (60, 900),
        "Cactus, cover removed / actual wire paths",
        font=small,
        fill="#163849",
    )
    basis = [[1, 0, 0.15], [-0.12, 1, 0.38]]
    render([(mesh_of(before), (185, 198, 206))], (580, 180, 530, 340), basis)
    render([(mesh_of(after), (119, 185, 192))], (1190, 180, 530, 340), basis)
    draw.text(
        (600, 132), "BEFORE: original curved inner wall", font=font, fill="#163849"
    )
    draw.text(
        (1200, 132), "AFTER: net-bound grooves + pin labels", font=font, fill="#163849"
    )
    fitted = [(mesh_of(after), (119, 185, 192))]
    fitted.extend(
        (mesh_of(c ^ crop), color)
        for c, color in zip(cables, [(228, 151, 44), (51, 140, 185), (193, 93, 127)])
    )
    render(fitted, (900, 620, 660, 300), basis)
    draw.text(
        (910, 565),
        "FIT VIEW: W01 REG.GND -> U1.GND / W02 REG.VOUT -> S1.P",
        font=small,
        fill="#163849",
    )
    draw.text(
        (55, 975),
        f"Print the 63 x 65 mm shell crop at 100%. Sampled wall estimate incl. export allowance: {wall:.2f} mm.",
        font=font,
        fill="#163849",
    )
    draw.text(
        (55, 1015),
        "Source model port anchors preserved; physical pin registration, cable retention and bend ratings remain pending.",
        font=small,
        fill="#426270",
    )
    im.save(output / "preview.png")


def guide(report, records):
    rows = "\n".join(
        f"| {r['id']} | {r['diameter_assumption_mm']:g} | {r['nominal_embed_along_w_mm']:g} | {r['sample_path_length_mm']:.1f} | {r['sample_trial_blank_mm']} |"
        for r in records
    )
    return f"""# 仙人掌真实壳体内壁减材槽：局部打印试装

直接从已有承载半壳 `base_piece_1.stl` 的内壁挖出两条圆底凹槽，向原壳体材料内部减材。路径读取现有 routing.json 的实际接线分支、元件位置、接口锚点及引出方向：W01 对应 REGULATOR_1.GND → U1_1.GND；W02 对应 REGULATOR_1.VOUT → S1_1.P。两条线路都位于同一刚体，保留在承载半壳上。样件是该真实壳片的 63 × 65 mm 局部裁切，保留原始曲率、外表面和壁厚。模型单位为毫米。

## 先打印这个文件

`cactus_inner_groove_sample_mm.stl`。不要同时打印 reference 文件，否则会多打一块未开槽的对照件。ZIP 已只包含一个要打印的 STL。

以 100% 比例打印，内侧凹槽朝上（文件 +Z），原曲面外侧朝下。建议 0.4 mm 喷嘴、0.2 mm 层高、3 道外壁、约 20% 填充；PLA 可用于首次静态配合试装。**外侧保留真实曲率，不是平底，需要按切片预览开启仅接触热床的支撑，必要时加 brim。** 槽面和标签面朝上，避免支撑接触槽内。先检查切片预览是否保留槽和文字。清理出口、槽内毛刺后再放线。

完整改动壳片 `base_piece_1_inner_grooves_world_mm.stl` 保留原装配世界坐标，供后续集成；小样使用的刚体变换记录于 validation.json，未缩放和压平曲面。`cactus_shell_context.glb` 包含壳体与完整端口到端口的两条新线路，供观察安装位置，不能作为单件打印文件。

## 放线试装

| 线编号 | 假设外径 mm | 沿开口方向名义嵌入深度 mm | 小样路径长 mm | 建议未端接试验线段 mm |
| --- | --- | --- | --- | --- |
{rows}

试验线段包含两端各 25 mm 外露余量，未计入压接/剥线耗长；不要照此剪断已有插头的成品线。两条线均沿用原布线半径 1 mm，即假设外径 2 mm，需用实际线材核对。线在圆底槽中部分嵌入，槽口开放，方便从内侧放入和取出。groove_spec.json 另记录完整端口到端口几何线长，但维护余量和端接工艺尚未校准。

壳片上直接凹刻 `W01 REG.GND / TO U1.GND`、`W02 REG.VOUT / TO S1.P`，终点附近另刻线号及目标接口。REG 是 REGULATOR_1，U1 是 U1_1，S1 是 S1_1；完整 ID、针脚名、来源分支及坐标保存在 groove_spec.json。全壳片上的槽端通过引线接到原接口锚点；截取小样后槽在部分裁切边缘开口。

这些端口坐标沿用已验证静态布线中的模型接口锚点，仍不是实测连接器针脚；端口引线的真实弯曲半径、插头几何和针序核验保持待完成。这里只替换这两个候选分支，没有把整个旧 routing.json 宣称成新方案。

本轮先验证“在真实壳壁里挖槽”的几何和放线配合；没有压片、螺钉、额外槽壁或机械应力释放。可以先用可移除胶带临时保持线的位置，再观察嵌入深度。正式固定和开合盖线缆管理后续再集成。

## 本次检查

- 修改后的完整壳片与小样均为闭合正体积实体，小样保持一个连通实体。
- 新增材料体积为 {report["added_volume_mm3"]:.6f} mm³，确认是减材槽。
- 对被移除材料表面顶点/三角形中心做外表面最近距离采样，最小剩余壁厚约 **{report["remaining_wall_sampled_min_mm"]:.2f} mm**；这是采样结果，不是连续壁厚或强度认证。
- 导出时会检查重新读取的 STL；微小布尔接触边只允许局部减材修整。计入导出修整余量后，剩余壁厚采样保守估计约 **{report["remaining_wall_conservative_estimate_after_export_mm"]:.2f} mm**。每个文件的修整量与复读检查均记录于 validation.json。
- 两条圆形电缆包络与修改后壳片没有实体相交，已检查沿开口方向放入/取出的保守扫掠，并检查载板、元件包络及其他壳片的碰撞。
- 采样未触及已有结构保护或电机禁入体素。源模型保留不动，SHA256 记录在 validation.json。
- 线材真实弯曲规格、打印配合、粗糙度、固定压片、全电路重布和机器人运动仍待验证。原仙人掌输出也仍是 partial，不能因此宣称整机制造完成。

## 请记录这几项

1. 实际线径与 W01/W02 的 2 mm 假设是否对应，放入是否顺畅。
2. 槽太浅、太深、太宽还是太窄；粗线是否需要过度弯折。
3. 外壁是否透底、打印翘曲，支撑是否容易清理。
4. 凹刻编号是否看得清，单独取出一条线是否方便。

先用这块对应真实接线关系的壳体小样校准槽宽和嵌入深度，再扩展到其他网络和关节线束。
"""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=Path(__file__).parent / "outputs/cactus_restrict_schema_live_verified",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output)
