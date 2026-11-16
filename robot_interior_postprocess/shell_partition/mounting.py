"""Hole-driven PCB supports, ribbed shelves and detachable carrier plates."""

import numpy as np
import trimesh
from hardware import cylinder_at, transform_solid

from .geometry import convex_sweep
from .supports import (
    connected_material,
    intersection_volume,
    local_box,
    mesh_of,
    rod,
    solid,
    union,
    within_half,
)

TOL = 0.01


def root(owner, start, direction, candidate, settings, radius):
    hits = owner.ray_cast(
        start.tolist(), (start + direction * settings["max_anchor_mm"]).tolist()
    )
    hits = [h for h in hits if np.linalg.norm(np.asarray(h.position) - start) > 0.05]
    if (
        len(hits) < 2
        or np.dot(hits[0].normal, direction) >= -0.05
        or np.dot(hits[1].normal, direction) <= 0.05
    ):
        return None
    anchor = np.asarray(hits[0].position)
    depth = np.linalg.norm(np.asarray(hits[1].position) - anchor)
    embed = min(settings["embed_mm"], depth * 0.4)
    if (
        embed < 0.1
        or abs(anchor @ np.asarray(candidate["normal"]) - candidate["offset_mm"])
        < settings["seam_keepout_mm"] + radius
    ):
        return None
    end = anchor + direction * embed
    return {
        "start_mm": start.tolist(),
        "anchor_mm": anchor.tolist(),
        "end_mm": end.tolist(),
        "embed_mm": float(embed),
    }


def box(lo, hi):
    return local_box(np.asarray(hi) - lo, (np.asarray(hi) + lo) / 2, np.eye(3))


def bore(point, profile, top=0):
    insert = profile["insert"]
    # Deep screw relief has a blind bottom; the larger insert pocket opens above.
    return union(
        [
            cylinder_at(
                point,
                insert["hole_diameter_mm"] / 2,
                top - insert["length_mm"],
                top + 0.05,
            ),
            cylinder_at(
                point,
                (profile["shank_diameter_mm"] + 0.2) / 2,
                top - profile["standoff_height_mm"] + profile["bottom_mm"],
                top + 0.05,
            ),
        ]
    )


def screw(point, profile, board_top):
    return union(
        [
            cylinder_at(
                point,
                profile["shank_diameter_mm"] / 2,
                board_top - profile["length_mm"],
                board_top,
            ),
            cylinder_at(
                point,
                profile["head_diameter_mm"] / 2,
                board_top,
                board_top + profile["head_height_mm"],
            ),
        ]
    )


def mounting_options(
    target,
    request,
    interface,
    owner,
    other,
    obstacles,
    wires,
    candidate,
    owner_index,
    gap,
    opening,
    settings,
    wire_relief=None,
    access_obstacles=None,
):
    """Only measured/explicitly registered interface planes determine PCB contact.

    The tool axis is the PCB normal, independent of the shell root direction.
    Final source bodies, screw paths, other hardware and fixed wires are checked.
    """
    transform = interface["transform"]
    access_obstacles = obstacles if access_obstacles is None else access_obstacles
    rotation, origin = transform[:3, :3], transform[:3, 3]
    profile, holes = interface["fastener"], interface["holes"]
    height, radius = profile["standoff_height_mm"], profile["boss_diameter_mm"] / 2
    thickness = settings["plate_mm"]
    lo, hi = interface["outline"].min(0), interface["outline"].max(0)
    margin = radius + 1
    floor_lo, floor_hi = lo - margin, hi + margin
    bosses = union(
        [cylinder_at(h["center_mm"], radius, -height - 0.1, 0) for h in holes]
    )
    bores = union([bore(h["center_mm"], profile) for h in holes])
    screws = union(
        [screw(h["center_mm"], profile, interface["thickness"]) for h in holes]
    )
    screws_world = transform_solid(screws, transform)
    travel = float(
        np.linalg.norm(mesh_of(owner).extents) * 2 + settings["max_anchor_mm"]
    )
    tools = union(
        [
            cylinder_at(
                h["center_mm"],
                profile["tool_radius_mm"],
                interface["thickness"] + profile["head_height_mm"],
                travel,
            )
            for h in holes
        ]
    )
    insert_tools = union(
        [
            cylinder_at(
                h["center_mm"], max(radius, profile["tool_radius_mm"]), 0.01, travel
            )
            for h in holes
        ]
    )
    world_tools = transform_solid(tools, transform)
    world_insert_tools = transform_solid(insert_tools, transform)
    attempts, options = [], []
    for template in request["templates"]:
        if template not in {"hole_standoffs", "ribbed_shelf", "removable_carrier"}:
            continue
        if template == "hole_standoffs":
            directions = [np.array([0.0, 0.0, -1.0])]
        elif template == "ribbed_shelf":
            directions = [
                np.array(v, float)
                for v in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
            ]
        else:
            directions = [np.array([0.0, 0.0, -1.0])]
        variants = [(d, 0, 1) for d in directions]
        if template == "removable_carrier":
            variants = [
                (directions[0], axis, sign) for axis in (0, 1) for sign in (1, -1)
            ]
        for local_direction, carrier_axis, carrier_sign in variants:
            tag = {
                "template": template,
                "attachment_direction_world": (rotation @ local_direction).tolist(),
                "carrier_extension_axis": carrier_axis
                if template == "removable_carrier"
                else None,
                "carrier_extension_sign": carrier_sign
                if template == "removable_carrier"
                else None,
            }
            roots, fixed_parts, removable = [], [], None
            cuts = bores
            secondary_records, secondary_screws, secondary_tools = [], [], []
            secondary_insert_tools = []
            if template == "hole_standoffs":
                fixed_parts.append(bosses)
                starts = [np.array([*h["center_mm"], -height + 0.1]) for h in holes]
            else:
                # All PCB bosses fuse into a real plate, rather than touching the package sides.
                plate = box([*floor_lo, -height - thickness], [*floor_hi, -height])
                fixed_parts.extend([bosses, plate])
                if template == "ribbed_shelf":
                    axis = int(np.argmax(abs(local_direction)))
                    lateral = 1 - axis
                    starts = []
                    for fraction in (0.25, 0.75):
                        p = np.array(
                            [*(floor_lo + floor_hi) / 2, -height - thickness / 2]
                        )
                        p[axis] = (
                            floor_hi[axis] - 1
                            if local_direction[axis] > 0
                            else floor_lo[axis] + 1
                        )
                        p[lateral] = floor_lo[lateral] + fraction * (
                            floor_hi[lateral] - floor_lo[lateral]
                        )
                        starts.append(p)
                else:
                    # Carrier bolts sit outside the board outline, with their own tool axes.
                    # The carrier is preassembled with the PCB, then lowered onto fixed pads.
                    extension = (
                        max(profile["head_diameter_mm"], profile["tool_radius_mm"] * 2)
                        + radius
                        + 2
                        + 2 * profile["min_wall_mm"]
                    )
                    carrier_lo, carrier_hi = floor_lo.copy(), floor_hi.copy()
                    boundary = (
                        floor_hi[carrier_axis]
                        if carrier_sign > 0
                        else floor_lo[carrier_axis]
                    )
                    if carrier_sign > 0:
                        carrier_hi[carrier_axis] += extension
                    else:
                        carrier_lo[carrier_axis] -= extension
                    fixed_parts[1] = box(
                        [*carrier_lo, -height - thickness], [*carrier_hi, -height]
                    )
                    starts = []
                    lateral = 1 - carrier_axis
                    inset = min(
                        2 * radius + 1, (floor_hi[lateral] - floor_lo[lateral]) / 3
                    )
                    for coordinate in [
                        floor_lo[lateral] + inset,
                        floor_hi[lateral] - inset,
                    ]:
                        p = np.zeros(3)
                        p[carrier_axis] = boundary + carrier_sign * extension / 2
                        p[lateral] = coordinate
                        p[2] = -height - thickness - height + 0.1
                        starts.append(p)
                    removable = union(fixed_parts) - bores
                    fixed_parts = []
                    for i, p in enumerate(starts):
                        pad_top = -height - thickness
                        fixed_parts.append(
                            cylinder_at(p[:2], radius + 1, pad_top - height, pad_top)
                        )
                        cuts += bore(p[:2], profile, pad_top)
                        removable -= cylinder_at(
                            p[:2],
                            (profile["shank_diameter_mm"] + 0.3) / 2,
                            pad_top - 0.05,
                            -height + 0.2,
                        )
                        secondary_screws.append(screw(p[:2], profile, -height))
                        secondary_tools.append(
                            cylinder_at(
                                p[:2],
                                profile["tool_radius_mm"],
                                -height + profile["head_height_mm"],
                                travel,
                            )
                        )
                        secondary_insert_tools.append(
                            cylinder_at(
                                p[:2],
                                max(radius, profile["tool_radius_mm"]),
                                pad_top + 0.01,
                                travel,
                            )
                        )
                        # Same profile, but carrier plate replaces the PCB grip thickness.
                        engagement = profile["length_mm"] - thickness
                        if (
                            not profile["min_engagement_mm"]
                            <= min(engagement, profile["insert"]["length_mm"])
                            or engagement > height - profile["bottom_mm"]
                        ):
                            attempts.append(
                                {
                                    **tag,
                                    "reason": "carrier_fastener_length_incompatible",
                                }
                            )
                            starts = []
                            break
                        secondary_records.append(
                            {
                                "id": f"carrier_bolt_{i}",
                                "center_interface_mm": [*p[:2], -height],
                                "thread": profile["thread"],
                                "length_mm": profile["length_mm"],
                            }
                        )
            if not starts:
                continue
            for p in starts:
                start = rotation @ p + origin
                direction = rotation @ local_direction
                found = root(owner, start, direction, candidate, settings, radius)
                if found is None:
                    break
                end = np.asarray(found["end_mm"])
                if template == "ribbed_shelf":
                    # Horizontal shelf extension plus a triangular web below it.
                    end_local = rotation.T @ (end - origin)
                    axis = int(np.argmax(abs(local_direction)))
                    side = np.eye(3)[1 - axis] * thickness / 2
                    top = p.copy()
                    top[2] = -height - thickness
                    root_top = end_local.copy()
                    root_top[2] = -height
                    root_bottom = end_local.copy()
                    root_bottom[2] -= settings["rib_depth_mm"]
                    vertices = np.array(
                        [
                            v + s * side
                            for v in [top, root_top, root_bottom]
                            for s in [-1, 1]
                        ]
                    )
                    web = solid(trimesh.convex.convex_hull(vertices))
                    beam_lo, beam_hi = (
                        np.minimum(p, end_local),
                        np.maximum(p, end_local),
                    )
                    beam_lo[1 - axis] -= thickness / 2
                    beam_hi[1 - axis] += thickness / 2
                    beam_lo[2], beam_hi[2] = -height - thickness, -height + 0.05
                    fixed_parts.extend([box(beam_lo, beam_hi), web])
                else:
                    end_local = rotation.T @ (end - origin)
                    fixed_parts.append(
                        solid(
                            trimesh.creation.cylinder(
                                radius=radius, segment=[p, end_local], sections=32
                            )
                        )
                    )
                roots.append(found)
            if len(roots) != len(starts):
                attempts.append(
                    {
                        **tag,
                        "reason": "interface_roots_not_found",
                        "roots_found": len(roots),
                    }
                )
                continue
            value = transform_solid(union(fixed_parts) - cuts, transform)
            cut_world = transform_solid(cuts, transform)
            carrier = (
                transform_solid(removable, transform) if removable is not None else None
            )
            secondary_world = transform_solid(union(secondary_screws), transform)
            tool_world = union(
                [world_tools, transform_solid(union(secondary_tools), transform)]
            )
            insert_access = union(
                [
                    world_insert_tools,
                    transform_solid(union(secondary_insert_tools), transform),
                ]
            )
            added = union([value, *([carrier] if carrier is not None else [])])
            unrelieved_volume = float(added.volume())
            relieved = False
            if wire_relief is not None and intersection_volume(added, wires) > TOL:
                lands = transform_solid(
                    union(
                        [
                            cylinder_at(
                                h["center_mm"],
                                radius + profile["min_wall_mm"],
                                -height - thickness,
                                0,
                            )
                            for h in holes
                        ]
                    ),
                    transform,
                )
                root_caps = union(
                    [
                        rod(
                            np.asarray(r["anchor_mm"])
                            - (np.asarray(r["end_mm"]) - np.asarray(r["anchor_mm"])),
                            np.asarray(r["end_mm"]),
                            radius,
                        )
                        for r in roots
                    ]
                )
                if intersection_volume(union([lands, root_caps]), wire_relief) > TOL:
                    attempts.append(
                        {**tag, "reason": "wire_relief_would_cut_mounting_land_or_root"}
                    )
                    continue
                value -= wire_relief
                if carrier is not None:
                    carrier -= wire_relief
                added = union([value, *([carrier] if carrier is not None else [])])
                relieved = True
            final = union([owner, value])
            reason = None
            if not within_half(added, candidate, owner_index, gap):
                reason = "mount_crosses_split"
            elif intersection_volume(cut_world, owner) > TOL:
                reason = "mount_bore_would_be_refilled_by_owner"
            elif not connected_material(final):
                reason = "mount_has_disconnected_roots"
            elif carrier is not None and (
                not connected_material(carrier)
                or intersection_volume(carrier, final) > TOL
            ):
                reason = "carrier_disconnected_or_interferes_with_base"
            elif any(
                intersection_volume(added, obstacle) > TOL
                for obstacle in [other, obstacles]
            ):
                reason = "mount_hits_hardware_or_other_piece"
            elif intersection_volume(added, wires) > TOL:
                reason = "mount_blocks_fixed_route"
            elif any(
                intersection_volume(union([screws_world, secondary_world]), obstacle)
                > TOL
                for obstacle in [
                    final,
                    obstacles,
                    wires,
                    *([carrier] if carrier is not None else []),
                ]
            ):
                reason = "mount_screw_hits_material_or_hardware"
            elif any(
                intersection_volume(tool_world, obstacle) > TOL
                for obstacle in [
                    final,
                    access_obstacles,
                    wires,
                    *([carrier] if carrier is not None else []),
                ]
            ):
                reason = "mount_tool_access_blocked"
            elif any(
                intersection_volume(insert_access, obstacle) > TOL
                for obstacle in [final, access_obstacles - interface["body"]]
            ):
                reason = "insert_installation_access_blocked"
            elif carrier is not None:
                # PCB + carrier are installed together; the fasteners are absent at this step.
                install_mesh = mesh_of(union([interface["body"], carrier]))
                swept = solid(convex_sweep(install_mesh, opening, travel))
                if any(
                    intersection_volume(swept, obstacle) > TOL
                    for obstacle in [final, access_obstacles - interface["body"]]
                ):
                    reason = "carrier_assembly_insertion_blocked"
            if reason:
                attempts.append({**tag, "reason": reason})
                continue
            normal = rotation[:, 2].tolist()
            hole_records = []
            for h in holes:
                center = rotation @ np.array([*h["center_mm"], 0]) + origin
                hole_records.append(
                    {
                        "id": h["id"],
                        "center_mm": center.tolist(),
                        "axis": normal,
                        "diameter_mm": h["diameter_mm"],
                        "insert_hole_diameter_mm": profile["insert"][
                            "hole_diameter_mm"
                        ],
                        "insert_depth_mm": profile["insert"]["length_mm"],
                    }
                )
            record = {
                "id": f"{target['id']}:support",
                "target_id": target["id"],
                "owner_link": request["owner_link"],
                "owner_piece_index": owner_index,
                "attachment": "fused",
                "template": template,
                "status": "interface_geometry_checked",
                "roots": roots,
                **tag,
                "contact_model": "registered_pcb_underside_and_mounting_holes",
                "holes": hole_records,
                "hardware_id": interface["record"]["hardware_id"],
                "registration": interface["record"]["registration"],
                "fastener": profile,
                "fastener_count": len(holes) + len(secondary_records),
                "carrier_fasteners": secondary_records,
                "volume_mm3": float(added.volume()),
                "wire_relief": {
                    "applied": relieved,
                    "clearance_beyond_protected_route_mm": settings["wire_relief_mm"],
                    "removed_mm3": max(0.0, unrelieved_volume - float(added.volume())),
                    "mounting_lands_and_roots_preserved": True,
                    "remaining_rib_strength": "pending",
                },
                "fixation": "hole_aligned_heat_set_screw_geometry_checked_physical_fit_pending",
                "strength": "pending",
                "crosses_split": False,
                "checks": {
                    "hole_alignment": True,
                    "screw_length": True,
                    "tool_access": True,
                    "insert_installation_access": True,
                    "owner_connected": True,
                    "carrier_insertion": True if carrier is not None else None,
                },
                "tool_check_scope": "Inserts before hardware; PCB tools before any obstructing later hardware and wiring, with explicit acyclic dependencies. Final screws and printed material still checked against all hardware and protected routes.",
                "release": "Remove screws, then translate PCB/carrier through selected opening; fixed roots remain on owner piece",
                "assembly_steps": [
                    "Open shell",
                    "Install inserts before PCB",
                    "Place PCB on hole-aligned bosses",
                    "Tighten PCB screws",
                ]
                if carrier is None
                else [
                    "Open shell",
                    "Install base and carrier inserts",
                    "Fasten PCB to carrier outside shell",
                    "Insert PCB/carrier assembly",
                    "Tighten carrier screws",
                    "Install wiring along the protected saved routes after carrier installation",
                ],
            }
            option = {
                "solid": value,
                "record": record,
                "occupancy": added,
                "fastener_solid": union([screws_world, secondary_world]),
                "tools": tool_world,
                "insert_tools": insert_access,
            }
            if carrier is not None:
                option["detachable"] = carrier
                option["installation_sweep"] = swept
            options.append(option)
    return options, attempts
