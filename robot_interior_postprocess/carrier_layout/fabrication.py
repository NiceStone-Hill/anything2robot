"""Exact carrier docking, ownership and straight-path assembly verification."""

from io import BytesIO

import numpy as np
import trimesh
from hardware import as_mesh64 as as_mesh
from hardware import cylinder_at
from shell_partition.geometry import convex_sweep, split_shell
from shell_partition.mounting import box
from shell_partition.supports import (
    connected_material,
    intersection_volume,
    solid,
    union,
)

from .profiles import posed

TOL = 0.02


def export_solid(mesh, path):
    """Validate the actual STL bytes, including float32 collapse of tiny faces.

    Only zero-area/duplicate triangles are removed; no hole filling or unproven
    mesh repair. A substantive volume change or open exported solid is rejected.
    """
    # Binary STL stores float32 coordinates. At world-coordinate offsets this
    # can merge distinct small features. ASCII retains the original coordinates;
    # it must pass exactly the same material checks, without filling any holes.
    from trimesh.exchange.stl import export_stl_ascii

    def decode(data):
        value = trimesh.load(
            BytesIO(data), file_type="stl", force="mesh", process=False
        )
        # Default mesh loading rounds vertices before welding them. Distinct
        # CSG edges can be closer than that threshold; weld exact coordinates.
        vertices, inverse = np.unique(value.vertices, axis=0, return_inverse=True)
        return trimesh.Trimesh(vertices, inverse[value.faces], process=False)

    for ascii_stl in (False, True):

        def encode(value, ascii_stl=ascii_stl):
            return (
                export_stl_ascii(value).encode("ascii")
                if ascii_stl
                else value.export(file_type="stl")
            )

        loaded = decode(encode(mesh))
        loaded.update_faces(loaded.area_faces > 0)
        loaded.update_faces(loaded.unique_faces())
        loaded.remove_unreferenced_vertices()
        data = encode(loaded)
        verified = decode(data)
        if (
            verified.is_volume
            and abs(verified.volume - mesh.volume)
            <= max(0.001, abs(mesh.volume) * 1e-8)
            and sum(m.volume > 1e-6 for m in verified.split(only_watertight=False)) == 1
        ):
            path.write_bytes(data)
            return verified
    raise ValueError(
        f"Exported STL does not preserve one valid material solid: {path.name}"
    )


def docking(profile, pose, owner, normal, seam_offset, seam_keepout):
    """Generate two finite solid rear brackets with blind prototype insert bores.

    Brackets are fused to one shell piece; the carrier is a separate removable
    part. Hole subtraction happens after material union, so it is never refilled.
    """
    transform = pose["transform"]
    inverse = np.linalg.inv(transform)
    width = profile["settings"]["ear_width_mm"]
    plate_bottom = profile["ear_z"] - profile["settings"]["plate_mm"]
    maximum = profile["settings"]["max_anchor_mm"]
    pieces, holes, roots = [], [], []
    for point in profile["ears"]:
        start_local = np.array([*point, plate_bottom - 0.1])
        start = (transform @ np.r_[start_local, 1])[:3]
        hits = owner.ray_cast(start.tolist(), (start - normal * maximum).tolist())
        hits = [
            h for h in hits if np.linalg.norm(np.asarray(h.position) - start) > 0.05
        ]
        if (
            len(hits) < 2
            or np.dot(hits[0].normal, -normal) >= -0.05
            or np.dot(hits[1].normal, -normal) <= 0.05
        ):
            raise ValueError("carrier_has_no_rear_shell_anchor")
        anchor = np.asarray(hits[0].position)
        depth = np.linalg.norm(np.asarray(hits[1].position) - anchor)
        embed = min(0.8, depth * 0.4)
        end = anchor - normal * embed
        if embed < 0.1 or end @ normal >= seam_offset - seam_keepout:
            raise ValueError("carrier_root_invalid_or_crosses_seam")
        local_end = (inverse @ np.r_[end, 1])[:3]
        length = plate_bottom - local_end[2]
        if length < profile["settings"]["dock_depth_mm"]:
            raise ValueError("carrier_dock_has_insufficient_screw_depth")
        # Broad solid rectangular brackets, not the old thin ray-generated rods.
        pieces.append(
            posed(
                box(
                    [point[0] - width / 2, point[1] - width / 2, local_end[2]],
                    [point[0] + width / 2, point[1] + width / 2, plate_bottom],
                ),
                transform,
            )
        )
        holes.append(
            posed(
                cylinder_at(point, 2.1, plate_bottom - 5, plate_bottom + 0.1), transform
            )
        )
        roots.append(
            {
                "anchor_mm": anchor.tolist(),
                "end_mm": end.tolist(),
                "span_mm": float(length),
                "embed_mm": float(embed),
            }
        )
    added, cuts = union(pieces), union(holes)
    # The carrier is removed for insert installation; check this in a separate step.
    return added - cuts, cuts, roots


def build_link(shell, plane, profiles, poses, settings, external_obstacles=()):
    pieces, removed = split_shell(shell, plane, settings["gap_mm"])
    owner, panel = solid(pieces[0]), solid(pieces[1])
    normal = np.asarray(plane["normal"], float)
    lookup = {p["spec"]["id"]: p for p in profiles}
    bodies = {p["id"]: posed(lookup[p["id"]]["body"], p["transform"]) for p in poses}
    carriers = {
        p["id"]: posed(lookup[p["id"]]["carrier"], p["transform"]) for p in poses
    }
    docks, cuts, records, screws, tools, insert_tools = {}, [], [], {}, {}, {}
    screw_parts = {}
    travel = float(np.linalg.norm(shell.extents) * 2 + 100)
    for pose in poses:
        key = pose["id"]
        p = lookup[key]
        added, cut, roots = docking(
            p, pose, owner, normal, plane["offset_mm"], settings["seam_keepout_mm"]
        )
        if intersection_volume(added, panel) > TOL:
            raise ValueError("dock_crosses_split")
        if any(
            intersection_volume(added, q) > TOL
            for q in [
                *bodies.values(),
                *carriers.values(),
                *docks.values(),
                *external_obstacles,
            ]
        ):
            raise ValueError("dock_hits_hardware_carrier_or_neighbor")
        docks[key] = added
        cuts.append(cut)
        z = p["ear_z"]
        bottom = z - p["settings"]["plate_mm"]
        screw_parts[key] = [
            posed(part, pose["transform"])
            for point in p["ears"]
            for part in (
                cylinder_at(point, 1.5, bottom - 4, z),
                cylinder_at(point, 2.9, z, z + 3),
            )
        ]
        screws[key] = union(screw_parts[key])
        tools[key] = posed(
            union([cylinder_at(point, 3.2, z + 3, travel) for point in p["ears"]]),
            pose["transform"],
        )
        insert_tools[key] = posed(
            union(
                [cylinder_at(point, 3.2, bottom + 0.01, travel) for point in p["ears"]]
            ),
            pose["transform"],
        )
        records.append(
            {
                "target_id": key,
                "owner_link": p["spec"]["owner_link"],
                "owner_piece_index": 0,
                "carrier_attachment": "removable_two_bolts",
                "dock_attachment": "fused",
                "roots": roots,
                "world_from_interface": pose["transform"].tolist(),
                "world_normal": normal.tolist(),
                "selected_roll_deg": pose["roll"],
                "local_shell_fastener_centres_mm": p["ears"],
                "component_holes": p["interface"]["record"]["selected_hole_ids"]
                if p["interface"]
                else [],
                "hardware_interface": {
                    **p["interface"]["record"],
                    "world_from_interface": pose["transform"].tolist(),
                }
                if p["interface"]
                else None,
                "pending": p["pending"],
            }
        )
    final = union([owner, *docks.values()]) - union(cuts)
    if not connected_material(final):
        raise ValueError("docks_not_connected_to_owner_material")
    all_hardware = union([*bodies.values(), *carriers.values(), *screws.values()])
    keys = list(bodies)
    for i, key in enumerate(keys):
        first = union([bodies[key], carriers[key], screws[key]])
        for other in keys[i + 1 :]:
            if (
                intersection_volume(
                    first, union([bodies[other], carriers[other], screws[other]])
                )
                > TOL
            ):
                raise ValueError("final_carrier_assemblies_intersect")
    if any(
        intersection_volume(final, q) > TOL
        for q in [*bodies.values(), *carriers.values(), *screws.values()]
    ):
        raise ValueError("final_owner_hits_assembly")
    checks, precedence = [], []
    for pose in poses:
        key = pose["id"]
        assembly = union([bodies[key], carriers[key]])
        obstacles = [final, *external_obstacles]
        swept = solid(convex_sweep(as_mesh(assembly), normal, travel))
        clear = not any(intersection_volume(swept, q) > TOL for q in obstacles)
        # Sweep individual shanks/heads, never a hull bridging separate bolts
        # or artificially tapering a screw head into a clearance bore.
        screw_sweep = union(
            [
                solid(convex_sweep(as_mesh(part), normal, travel))
                for part in screw_parts[key]
            ]
        )
        screw_clear = not any(
            intersection_volume(screw_sweep, q) > TOL for q in [*obstacles, assembly]
        )
        tool_clear = not any(
            intersection_volume(tools[key], q) > TOL for q in [*obstacles, assembly]
        )
        blockers = []
        for other, other_body in bodies.items():
            if other == key:
                continue
            other_assembly = union([other_body, carriers[other], screws[other]])
            if any(
                intersection_volume(path, other_assembly) > TOL
                for path in [swept, screw_sweep, tools[key]]
            ):
                blockers.append(other)
                precedence.append([key, other])
        # Inserts are fitted BEFORE carriers/hardware exist, with the cover absent.
        inserts_clear = not any(
            intersection_volume(insert_tools[key], q) > TOL
            for q in [final, *external_obstacles]
        )
        panel_clear = (
            intersection_volume(
                solid(convex_sweep(as_mesh(assembly + screws[key]), -normal, travel)),
                panel,
            )
            <= TOL
        )
        checks.append(
            {
                "target_id": key,
                "carrier_insertion_clear": clear,
                "carrier_screw_insertion_clear": screw_clear,
                "carrier_tool_clear": tool_clear,
                "dock_insert_installation_clear": inserts_clear,
                "panel_translation_clear": panel_clear,
                "must_precede_carriers": blockers,
                "pcb_fastening_stage": "bench_before_carrier_insertion",
                "path_model": "continuous convex translation; conservative, straight paths only",
            }
        )
    remaining, order = set(bodies), []
    while remaining:
        ready = sorted(
            key
            for key in remaining
            if not any(b == key and a in remaining for a, b in precedence)
        )
        if not ready:
            raise ValueError("cyclic_carrier_insertion_or_tool_dependencies")
        order.extend(ready)
        remaining.difference_update(ready)
    # Existing motor proxy overlap is not reclassified as a hardware collision proof.
    return {
        "pieces": [as_mesh(final), pieces[1]],
        "plane": plane,
        "seam_removed_mm3": removed,
        "records": records,
        "checks": checks,
        "bodies": bodies,
        "carriers": carriers,
        "docks": docks,
        "screws": screws,
        "tools": tools,
        "carrier_order": order,
        "carrier_precedence": precedence,
        "all_paths_clear": all(
            all(
                r[k]
                for k in (
                    "carrier_insertion_clear",
                    "carrier_screw_insertion_clear",
                    "carrier_tool_clear",
                    "dock_insert_installation_clear",
                    "panel_translation_clear",
                )
            )
            for r in checks
        ),
        "all_hardware": all_hardware,
    }
