"""Screw-fastened shell pairs with explicit coaxial locating interfaces.

All locations come from the actual cut contour and material. No robot names,
fabricated component holes, or assumption that a thin shell is a threaded boss.
"""

import numpy as np
import trimesh
from hardware import as_mesh64 as as_mesh
from hardware import cylinder_at, number, transform_solid
from shell_partition.geometry import convex_sweep
from shell_partition.supports import (
    connected_material,
    intersection_volume,
    solid,
    union,
)

TOL = 0.02


class ClosureSearchError(ValueError):
    def __init__(self, report):
        self.report = report
        super().__init__(
            f"Insufficient shell connections: {report['connection_count']}/{report['required_count']}"
        )


DEFAULTS = {
    "enabled": True,
    "boss_diameter_mm": 11.0,
    "boss_depth_mm": 8.0,
    "clearance_diameter_mm": 3.4,
    "screw_diameter_mm": 3.0,
    "head_diameter_mm": 5.5,
    "head_height_mm": 3.0,
    "counterbore_diameter_mm": 8.0,
    "tool_radius_mm": 3.2,
    "driver_shaft_length_mm": 75.0,
    "driver_handle_radius_mm": 12.0,
    "driver_handle_length_mm": 45.0,
    "max_head_recess_mm": 12.0,
    "insert_hole_diameter_mm": 4.2,
    "insert_depth_mm": 5.0,
    "insert_tool_radius_mm": 2.3,
    "min_engagement_mm": 3.0,
    "min_wall_mm": 1.0,
    "min_bottom_mm": 1.5,
    "pin_diameter_mm": 7.2,
    "pin_bore_diameter_mm": 5.0,
    "socket_diameter_mm": 7.8,
    "pin_height_mm": 2.0,
    "socket_depth_mm": 2.5,
    "min_cover_web_mm": 1.5,
    "min_root_overlap_mm3": 3.0,
    "min_stop_contact_area_mm2": 5.0,
    "min_connections": 3,
    "max_connections": 6,
    "spacing_mm": 110.0,
    "min_spacing_mm": 24.0,
    "contour_samples": 96,
    "max_candidates": 288,
    "screw_lengths_mm": [8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0, 35.0, 40.0],
    "inset_mm": [6.0, 8.0, 10.0],
}


def settings_for(raw=None):
    raw = raw or {}
    if not isinstance(raw, dict) or raw.keys() - DEFAULTS.keys():
        raise ValueError("Unknown shell-fastening setting")
    p = {**DEFAULTS, **raw}
    if not isinstance(p["enabled"], bool):
        raise TypeError("Shell fastening enabled must be boolean")
    for key in p.keys() - {"enabled", "screw_lengths_mm", "inset_mm"}:
        p[key] = number(p[key], key)
    for key in (
        "min_connections",
        "max_connections",
        "contour_samples",
        "max_candidates",
    ):
        if int(p[key]) != p[key]:
            raise ValueError("Shell fastening counts must be integers")
        p[key] = int(p[key])
    if (
        not 3 <= p["min_connections"] <= p["max_connections"] <= 12
        or not 12 <= p["contour_samples"] <= 256
        or p["max_candidates"] > 1024
    ):
        raise ValueError("Unsupported shell-fastening search budget")
    for key in ("screw_lengths_mm", "inset_mm"):
        if not isinstance(p[key], list) or not 1 <= len(p[key]) <= 32:
            raise ValueError("Finite screw-length and contour-inset lists required")
        p[key] = sorted({number(v, key) for v in p[key]})
    if (
        p["screw_diameter_mm"] != 3.0
        or p["head_diameter_mm"] < 5.5
        or p["head_height_mm"] < 3.0
        or p["max_head_recess_mm"] < p["head_height_mm"] + 0.5
        or p["driver_shaft_length_mm"] < p["max_head_recess_mm"] + 10
        or p["driver_handle_radius_mm"] < p["tool_radius_mm"]
        or p["clearance_diameter_mm"] <= p["screw_diameter_mm"]
        or p["counterbore_diameter_mm"]
        <= max(p["head_diameter_mm"], 2 * p["tool_radius_mm"])
        or p["boss_diameter_mm"]
        < max(p["counterbore_diameter_mm"], p["socket_diameter_mm"])
        + 2 * p["min_wall_mm"]
        or p["boss_depth_mm"] < p["insert_depth_mm"] + p["min_bottom_mm"]
        or p["min_engagement_mm"] > p["insert_depth_mm"]
        or p["pin_bore_diameter_mm"]
        <= max(p["insert_hole_diameter_mm"], 2 * p["insert_tool_radius_mm"])
        or p["pin_diameter_mm"] < p["pin_bore_diameter_mm"] + 2 * p["min_wall_mm"]
        or p["socket_diameter_mm"] <= p["pin_diameter_mm"]
        or p["socket_depth_mm"] <= p["pin_height_mm"]
    ):
        raise ValueError(
            "Incompatible shell screw, boss, insert or locating fit dimensions"
        )
    return p


def local_frame(plane):
    normal = np.asarray(plane["normal"], float)
    if normal.shape != (3,) or not np.isclose(np.linalg.norm(normal), 1):
        raise ValueError("Shell closure requires a unit split normal")
    transform = trimesh.geometry.align_vectors([0, 0, 1], normal)
    transform[:3, 3] = normal * plane["offset_mm"]
    return transform


def outer_envelope(shell, explicit=None):
    if explicit is not None:
        if not explicit.is_volume:
            raise ValueError("Shell fastening exterior envelope must be a closed solid")
        return explicit
    # Only a genuinely separate positive exterior boundary can be filled.
    # A through-hole shell has no such boundary: require its source solid.
    boundaries = shell.split(only_watertight=False)
    exterior = [m for m in boundaries if m.is_volume and m.volume > 0]
    voids = [m for m in boundaries if m.volume < -1e-6]
    if len(exterior) != 1 or not voids:
        raise ValueError(
            "Shell fastening needs an explicit exterior envelope for an open/through-hole shell"
        )
    return exterior[0]


def contour_points(section, p):
    polygons = [np.asarray(poly) for poly in section.to_polygons()]
    rows, perimeter = [], 0.0
    for polygon in polygons:
        if len(polygon) < 3:
            continue
        x, y = polygon.T
        area = float(x @ np.roll(y, -1) - y @ np.roll(x, -1)) / 2
        if area <= 0:
            continue
        edges = np.roll(polygon, -1, axis=0) - polygon
        lengths = np.linalg.norm(edges, axis=1)
        length = float(lengths.sum())
        perimeter += length
        cumulative = np.r_[0, np.cumsum(lengths)]
        for distance in np.linspace(0, length, p["contour_samples"], endpoint=False):
            index = min(
                int(np.searchsorted(cumulative, distance, side="right") - 1),
                len(edges) - 1,
            )
            if lengths[index] < 1e-6:
                continue
            tangent = edges[index] / lengths[index]
            boundary = polygon[index] + tangent * (distance - cumulative[index])
            inward = np.array([-tangent[1], tangent[0]])
            for inset in p["inset_mm"]:
                rows.append(boundary + inward * inset)
    return rows[: p["max_candidates"]], perimeter


def candidate(
    point,
    p,
    gap,
    outer,
    original,
    blockers,
    insert_blockers,
    owner_keepout,
    transform,
    travel,
    required_length=None,
):
    """One exact nominal joint; actual insert/material/process remain unverified."""
    lo, hi = -gap / 2, gap / 2
    radius = p["boss_diameter_mm"] / 2
    hit = outer.ray_cast([*point, hi + 0.001], [*point, travel])
    if len(hit) != 1 or hit[0].normal[2] <= 0.05:
        raise ValueError("no_single_exterior_exit_along_screw_axis")
    exit_z = float(hit[0].position[2])
    choice = None
    lengths = p["screw_lengths_mm"] if required_length is None else [required_length]
    for length in lengths:
        # Pick a real standard length first, then place the bearing seat so the
        # blind insert has valid engagement and the head stays near the mouth.
        seat_min = max(
            hi + p["socket_depth_mm"] + p["min_cover_web_mm"],
            exit_z - p["max_head_recess_mm"],
            length + lo - p["insert_depth_mm"],
        )
        seat_max = min(
            exit_z - p["head_height_mm"] - 0.5,
            length + lo - p["min_engagement_mm"],
        )
        if seat_min > seat_max:
            continue
        seat = float(np.clip(length + lo - 4.0, seat_min, seat_max))
        if seat < hi + p["socket_depth_mm"] + p["min_cover_web_mm"]:
            continue
        wall_probe = cylinder_at(
            point,
            p["counterbore_diameter_mm"] / 2 + p["min_wall_mm"],
            seat - 0.15,
            seat + 0.15,
        )
        if (wall_probe - outer).volume() > TOL:
            continue
        choice = seat, length
        break
    if choice is None:
        raise ValueError("no_standard_length_or_counterbore_wall")
    seat, length = choice
    bottom_probe = cylinder_at(
        point,
        p["insert_hole_diameter_mm"] / 2 + p["min_wall_mm"],
        lo - p["insert_depth_mm"] / 2 - 0.15,
        lo - p["insert_depth_mm"] / 2 + 0.15,
    )
    if (bottom_probe - outer).volume() > TOL:
        raise ValueError("insert_boss_wall_too_thin")
    owner_add = cylinder_at(point, radius, lo - p["boss_depth_mm"], lo) ^ outer
    # A real annular stop bridges the seam gap. Without it, tightening would
    # pull the cover through the nominal gap and change the assembled geometry.
    owner_add += cylinder_at(point, radius, lo - 0.05, hi) ^ outer
    pin = (
        cylinder_at(point, p["pin_diameter_mm"] / 2, lo - 0.1, hi + p["pin_height_mm"])
        ^ outer
    )
    panel_add = cylinder_at(point, radius, hi, exit_z + 0.1) ^ outer
    if any(
        intersection_volume(add, original[i]) < p["min_root_overlap_mm3"]
        for i, add in enumerate((owner_add, panel_add))
    ):
        raise ValueError("shell_boss_has_no_material_root")
    owner_add += pin
    owner_cut = cylinder_at(
        point, p["insert_hole_diameter_mm"] / 2, lo - p["insert_depth_mm"], lo + 0.01
    )
    owner_cut += cylinder_at(
        point, p["pin_bore_diameter_mm"] / 2, lo - 0.01, hi + p["pin_height_mm"] + 0.1
    )
    panel_cut = cylinder_at(point, p["clearance_diameter_mm"] / 2, hi - 0.1, travel)
    panel_cut += cylinder_at(point, p["counterbore_diameter_mm"] / 2, seat, travel)
    panel_cut += cylinder_at(
        point, p["socket_diameter_mm"] / 2, hi - 0.1, hi + p["socket_depth_mm"]
    )
    final = [
        (original[0] + owner_add) - owner_cut,
        (original[1] + panel_add) - panel_cut,
    ]
    stop_contact_area = (
        final[0].slice(hi - 0.0001) ^ final[1].slice(hi + 0.0001)
    ).area()
    if stop_contact_area < p["min_stop_contact_area_mm2"]:
        raise ValueError("shell_shoulder_has_insufficient_mating_contact")
    if intersection_volume(owner_add - owner_cut, owner_keepout) > TOL:
        raise ValueError("shell_boss_blocks_carrier_or_dock_operation")
    shaft = cylinder_at(point, p["screw_diameter_mm"] / 2, seat - length, seat)
    head = cylinder_at(
        point, p["head_diameter_mm"] / 2, seat, seat + p["head_height_mm"]
    )
    tool = cylinder_at(point, p["tool_radius_mm"], seat + p["head_height_mm"], travel)
    # A straight driver holds the screw at the outside mouth. Its entire
    # shaft/handle insertion sweep must remain accessible from outside.
    handle_start = seat + p["head_height_mm"] + p["driver_shaft_length_mm"]
    tool += cylinder_at(
        point,
        p["driver_handle_radius_mm"],
        handle_start,
        travel + p["driver_handle_length_mm"],
    )
    insert_tool = cylinder_at(point, p["insert_tool_radius_mm"], lo + 0.02, travel)
    screw_sweep = union(
        [solid(convex_sweep(as_mesh(v), [0, 0, 1], travel)) for v in (shaft, head)]
    )
    for value in (
        owner_add - owner_cut,
        panel_add - panel_cut,
        shaft + head,
        tool,
        screw_sweep,
    ):
        if intersection_volume(value, blockers) > TOL:
            raise ValueError(
                "shell_joint_or_screw_access_hits_hardware_route_or_neighbor"
            )
    if intersection_volume(insert_tool, insert_blockers + final[0]) > TOL:
        raise ValueError("shell_insert_installation_blocked")
    if intersection_volume(screw_sweep + tool, union(final)) > TOL:
        raise ValueError("shell_screw_or_tool_hits_final_material")
    if intersection_volume(final[0], final[1]) > TOL:
        raise ValueError("shell_locator_interferes_with_socket")
    if any(not connected_material(part) for part in final):
        raise ValueError("shell_joint_disconnects_material")
    center = (transform @ [*point, 0, 1])[:3]
    axis = transform[:3, 2]
    record = {
        "local_cut_xy_mm": point.tolist(),
        "axis_origin_world_mm": center.tolist(),
        "screw_axis_world": axis.tolist(),
        "screw_entry_direction_world": (-axis).tolist(),
        "screw_length_mm": length,
        "screw_diameter_mm": p["screw_diameter_mm"],
        "clearance_diameter_mm": p["clearance_diameter_mm"],
        "counterbore_diameter_mm": p["counterbore_diameter_mm"],
        "fastener_spec": {
            "designation": f"ISO 4762 M3 x {length:g}, pitch 0.5, full thread, steel 12.9",
            "standard": "ISO 4762",
            "thread": "M3",
            "pitch_mm": 0.5,
            "length_under_head_mm": length,
            "head_diameter_mm": 5.5,
            "head_height_mm": 3.0,
            "drive": "hex_socket",
            "hex_key_mm": 2.5,
            "thread_coverage": "full_thread_required",
            "material": "steel",
            "property_class": "12.9",
            "source": "https://eshop.wuerth.de/Hexagon-Socket-Head-Cap-Screw-full-thread-ISO-4762-DIN-912-steel-129-plain-with-full-thread-SCR-CYL-ISO4762-129-HS25-M3X25-25/008393%2025.sku/en/US/EUR/",
        },
        "access": {
            "type": "straight_through_loading_tunnel",
            "mouth_world_mm": (transform @ [*point, exit_z, 1])[:3].tolist(),
            "head_seat_world_mm": (transform @ [*point, seat, 1])[:3].tolist(),
            "loading_start_world_mm": (
                transform @ [*point, exit_z + length + p["head_height_mm"] + 5, 1]
            )[:3].tolist(),
            "insertion_direction_world": (-axis).tolist(),
            "port_diameter_mm": p["counterbore_diameter_mm"],
            "mouth_to_seat_mm": exit_z - seat,
            "driver": "straight 2.5 mm hex driver with magnetic/holding tip; no offset elbow in tunnel",
            "max_driver_shaft_diameter_mm": 2 * p["tool_radius_mm"],
            "driver_shaft_length_mm": p["driver_shaft_length_mm"],
            "max_driver_handle_diameter_mm": 2 * p["driver_handle_radius_mm"],
            "max_driver_handle_length_mm": p["driver_handle_length_mm"],
            "screw_and_driver_sweep_checked": True,
        },
        "head_seat_local_z_mm": seat,
        "clamp_length_mm": seat - lo,
        "engagement_mm": length - (seat - lo),
        "blind_bottom_remaining_mm": p["boss_depth_mm"] - p["insert_depth_mm"],
        "axial_stop_bridges_gap_mm": gap,
        "axial_stop_local_z_mm": hi,
        "axial_stop_contact_area_mm2": stop_contact_area,
        "thread": f"M{p['screw_diameter_mm']:g}",
        "insert": {
            "id": f"prototype_M{p['screw_diameter_mm']:g}_blind_insert_select_and_calibrate",
            "hole_diameter_mm": p["insert_hole_diameter_mm"],
            "depth_mm": p["insert_depth_mm"],
        },
        "locator": {
            "type": "coaxial_annular_spigot",
            "pin_diameter_mm": p["pin_diameter_mm"],
            "socket_diameter_mm": p["socket_diameter_mm"],
            "pin_height_mm": p["pin_height_mm"],
            "socket_depth_mm": p["socket_depth_mm"],
            "radial_clearance_mm": (p["socket_diameter_mm"] - p["pin_diameter_mm"]) / 2,
            "exit_clearance_mm": p["socket_depth_mm"] - p["pin_height_mm"],
        },
        "geometry_checks": {
            "material_roots": True,
            "screw_insertion_clear": True,
            "external_tool_clear": True,
            "insert_installation_clear_before_carriers": True,
            "closed_pair_no_interference": True,
        },
        "process_status": "prototype_uncalibrated",
        "strength": "pending",
    }
    return {
        "point": np.asarray(point),
        "adds": [owner_add, panel_add],
        "cuts": [owner_cut, panel_cut],
        "screw": shaft + head,
        "screw_sweep": screw_sweep,
        "tool": tool,
        "insert_tool": insert_tool,
        "record": record,
    }


def prepare_pair(
    pieces,
    plane,
    shell,
    exterior=None,
    obstacles=(),
    insert_obstacles=(),
    owner_keepouts=(),
    settings=None,
    gap_mm=0.4,
):
    p = settings_for(settings)
    if not p["enabled"]:
        raise ValueError("Shell fastening disabled")
    transform = local_frame(plane)
    inverse = np.linalg.inv(transform)

    def local(value):
        return transform_solid(value, inverse)

    original = [local(solid(mesh)) for mesh in pieces]
    if (
        as_mesh(original[0]).vertices[:, 2].max() > -gap_mm / 2 + 1e-4
        or as_mesh(original[1]).vertices[:, 2].min() < gap_mm / 2 - 1e-4
    ):
        raise ValueError(
            "Undeclared cross-seam features invalidate the original half-space proof"
        )
    outer = local(solid(outer_envelope(shell, exterior)))
    blockers = union([local(o) for o in obstacles])
    insert_blockers = union([local(o) for o in insert_obstacles])
    owner_keepout = union([local(o) for o in owner_keepouts])
    travel = float(
        max(as_mesh(outer).bounds[1, 2] + 20, np.linalg.norm(shell.extents) * 2)
    )
    points, perimeter = contour_points(outer.slice(0), p)
    return {
        "p": p,
        "gap": number(gap_mm, "seam gap"),
        "outer": outer,
        "original": original,
        "blockers": blockers,
        "insert_blockers": insert_blockers,
        "owner_keepout": owner_keepout,
        "transform": transform,
        "travel": travel,
        "points": points,
        "perimeter": perimeter,
    }


def try_candidate(context, point, length=None):
    return candidate(
        np.asarray(point),
        context["p"],
        context["gap"],
        context["outer"],
        context["original"],
        context["blockers"],
        context["insert_blockers"],
        context["owner_keepout"],
        context["transform"],
        context["travel"],
        length,
    )


def farthest_order(points):
    if not len(points):
        return []
    values = np.asarray(points)
    order = [0]
    distance = np.linalg.norm(values - values[0], axis=1)
    while len(order) < len(points):
        distance[order] = -1
        index = int(np.argmax(distance))
        order.append(index)
        distance = np.minimum(distance, np.linalg.norm(values - values[index], axis=1))
    return order


def target_count(context):
    p = context["p"]
    return min(
        p["max_connections"],
        max(p["min_connections"], int(np.ceil(context["perimeter"] / p["spacing_mm"]))),
    )


def fasten_shell_pair(
    pieces,
    plane,
    shell,
    exterior=None,
    obstacles=(),
    insert_obstacles=(),
    owner_keepouts=(),
    settings=None,
    gap_mm=0.4,
):
    context = prepare_pair(
        pieces,
        plane,
        shell,
        exterior,
        obstacles,
        insert_obstacles,
        owner_keepouts,
        settings,
        gap_mm,
    )
    p = context["p"]
    points = context["points"]
    target = target_count(context)
    accepted, rejected = [], []
    # Far-apart contour samples first; then additional insets where needed.
    order = farthest_order(points)
    for index in order:
        point = points[index]
        if any(
            np.linalg.norm(point - feature["point"]) < p["min_spacing_mm"]
            for feature in accepted
        ):
            continue
        try:
            feature = try_candidate(context, point)
        except ValueError as exc:
            rejected.append({"local_xy_mm": point.tolist(), "reason": str(exc)})
            continue
        accepted.append(feature)
        if len(accepted) == target:
            break
    return finish_pair(context, accepted, rejected, target)


def finish_pair(context, accepted, rejected, target):
    p, original, outer, transform = (
        context[k] for k in ("p", "original", "outer", "transform")
    )
    if len(accepted) < p["min_connections"]:
        raise ClosureSearchError(
            {
                "status": "failed",
                "connection_count": len(accepted),
                "required_count": p["min_connections"],
                "target_count": target,
                "candidate_rejections": rejected,
            }
        )
    # Reject collinear screws: coverage must constrain more than one hinge axis.
    xy = np.array([f["point"] for f in accepted])
    if np.linalg.matrix_rank(xy - xy.mean(0), tol=1e-3) != 2:
        raise ValueError("Shell connection positions are collinear")
    final = [
        (original[side] + union([f["adds"][side] for f in accepted]))
        - union([f["cuts"][side] for f in accepted])
        for side in (0, 1)
    ]
    if (
        any(not connected_material(part) for part in final)
        or intersection_volume(final[0], final[1]) > TOL
    ):
        raise ValueError("Combined shell connections are disconnected or interfere")
    # Original pieces stay in disjoint half spaces. Broad shoulder stops end at
    # the cover's inner plane. Only the declared coaxial spigots extend beyond
    # it; their full radius fits a deeper, open-ended socket.
    # Translating the cover along +normal can only reduce spigot penetration.
    record = {
        "status": "geometry_checked",
        "connection_count": len(accepted),
        "target_count": target,
        "owner_piece_index": 0,
        "cover_piece_index": 1,
        "all_pieces_connected": True,
        "separation_method": "original half-spaces plus shoulders ending at the cover inner plane and coaxial spigots inside deeper open-ended clearance sockets; screws removed before +normal withdrawal",
        "screw_removal_before_opening": True,
        "cover_translation_direction_world": transform[:3, 2].tolist(),
        "minimum_locator_release_mm": p["pin_height_mm"] + 0.1,
        "connections": [f["record"] for f in accepted],
        "candidate_rejections": rejected,
        "profile": p,
        "world_from_cut_frame": transform.tolist(),
        "seam_contours_local_mm": [
            np.asarray(poly).tolist() for poly in outer.slice(0).to_polygons()
        ],
        "manufacturing_validation": "pending_insert_selection_print_fit_and_load_test",
    }
    return {
        "pieces": [as_mesh(transform_solid(part, transform)) for part in final],
        "report": record,
        "screws": [transform_solid(f["screw"], transform) for f in accepted],
        "tools": [transform_solid(f["tool"], transform) for f in accepted],
        "screw_sweeps": [
            transform_solid(f["screw_sweep"], transform) for f in accepted
        ],
        "insert_tools": [
            transform_solid(f["insert_tool"], transform) for f in accepted
        ],
        "added": [
            transform_solid(
                union([f["adds"][side] for f in accepted])
                - union([f["cuts"][side] for f in accepted]),
                transform,
            )
            for side in (0, 1)
        ],
    }


def scene_closures(
    out,
    scene,
    shells,
    pieces,
    planes,
    designs,
    profiles,
    poses,
    wire,
    gap_mm,
    raw_settings=None,
):
    """Attach every split pair and retain electronics, routes and carrier tasks."""
    from common import dump

    from .fastener_patterns import discover_groups, fasten_group, policy_for

    raw_settings = dict(raw_settings or {})
    policy = policy_for(raw_settings.pop("pattern_policy", None))
    p = settings_for(raw_settings)
    if not p["enabled"]:
        return {}
    hardware = [d["all_hardware"] for d in designs.values()]
    fixed_docks = [dock for d in designs.values() for dock in d["docks"].values()]
    motor_guards = []
    for motor in scene.get("motors", []):
        a, b = np.array(motor["axial_start_mm"]), np.array(motor["axial_end_mm"])
        axis = (b - a) / np.linalg.norm(b - a)
        motor_guards.append(
            solid(
                trimesh.creation.cylinder(
                    radius=motor["radius_mm"] + 6,
                    segment=np.array([a - axis * 6, b + axis * 6]),
                    sections=32,
                )
            )
        )
    all_keepouts = {}
    for name, design in designs.items():
        keepouts = []
        normal = np.asarray(planes[name]["normal"])
        travel = float(np.linalg.norm(shells[name].extents) * 2 + 100)
        for pose in poses:
            if pose["id"] not in design["bodies"]:
                continue
            profile = next(v for v in profiles if v["spec"]["id"] == pose["id"])
            assembly = design["bodies"][pose["id"]] + design["carriers"][pose["id"]]
            keepouts.append(solid(convex_sweep(as_mesh(assembly), normal, travel)))
            keepouts.append(design["tools"][pose["id"]])
            z = profile["ear_z"]
            bottom = z - profile["settings"]["plate_mm"]
            for point in profile["ears"]:
                keepouts.append(
                    transform_solid(
                        cylinder_at(point, 3.2, bottom + 0.01, travel),
                        pose["transform"],
                    )
                )
                for screw_part in (
                    cylinder_at(point, 1.5, bottom - 4, z),
                    cylinder_at(point, 2.9, z, z + 3),
                ):
                    world = transform_solid(screw_part, pose["transform"])
                    keepouts.append(solid(convex_sweep(as_mesh(world), normal, travel)))
        all_keepouts[name] = keepouts
    result, report, contexts = {}, {}, {}
    for name in scene["parts"]:
        print(f"ASSEMBLY: shell screws and locating sockets {name}", flush=True)
        neighbors = [
            solid(mesh)
            for other, pair in pieces.items()
            if other != name
            for mesh in pair
        ]
        obstacles = [
            *hardware,
            *fixed_docks,
            *motor_guards,
            *neighbors,
        ]
        if wire is not None:
            obstacles.append(wire)
        envelope_path = out / "meshes" / f"{name}_solid_assembly_mm.stl"
        envelope = (
            trimesh.load(envelope_path, force="mesh")
            if envelope_path.exists()
            else None
        )
        contexts[name] = prepare_pair(
            pieces[name],
            planes[name],
            shells[name],
            envelope,
            obstacles,
            [*fixed_docks, *motor_guards, *neighbors],
            all_keepouts.get(name, []),
            p,
            gap_mm,
        )
    for group in discover_groups(contexts, policy):
        names = list(group["registrations"])
        print("ASSEMBLY: jointly plan shell pattern " + ", ".join(names), flush=True)
        other_features = [
            v for other in result.values() for v in [*other["added"], *other["screws"]]
        ]
        for name in names:
            inverse = np.linalg.inv(contexts[name]["transform"])
            contexts[name]["blockers"] += union(
                [transform_solid(v, inverse) for v in other_features]
            )
        try:
            if len(names) > 1:
                result.update(fasten_group(contexts, group))
            else:
                name = names[0]
                ctx = contexts[name]
                accepted, rejected = [], []
                target = target_count(ctx)
                for index in farthest_order(ctx["points"]):
                    point = ctx["points"][index]
                    if any(
                        np.linalg.norm(point - f["point"]) < p["min_spacing_mm"]
                        for f in accepted
                    ):
                        continue
                    try:
                        accepted.append(try_candidate(ctx, point))
                    except ValueError as exc:
                        rejected.append(
                            {"local_xy_mm": point.tolist(), "reason": str(exc)}
                        )
                    if len(accepted) == target:
                        break
                result[name] = finish_pair(ctx, accepted, rejected, target)
        except ClosureSearchError as exc:
            report["shared:" + group["reference"]] = exc.report
            dump(out / "shell_connections.json", {"status": "failed", "links": report})
            raise
        for name in names:
            report[name] = result[name]["report"]
    # New features from a later link may block a previously accepted tool.
    # Recheck the complete scene, not only pairwise generation order.
    for name, value in result.items():
        foreign = [
            solid(mesh)
            for other, data in result.items()
            if other != name
            for mesh in data["pieces"]
        ]
        foreign += [
            s for other, data in result.items() if other != name for s in data["screws"]
        ]
        for tool in [*value["tools"], *value["screw_sweeps"]]:
            if any(intersection_volume(tool, obstacle) > TOL for obstacle in foreign):
                raise ValueError("final_neighbor_shell_blocks_closure_tool:" + name)
    dump(
        out / "shell_connections.json", {"status": "geometry_checked", "links": report}
    )
    return result


def export_connection_map(out, closures):
    """An actual geometric cut-contour map, not an illustrative generated image."""
    from PIL import Image, ImageDraw

    if not closures:
        return
    columns = 2
    rows = (len(closures) + 1) // 2
    canvas = Image.new("RGB", (1440, 550 * rows + 50), "#f4f6f8")
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (22, 16),
        "Shell fastening map | actual cut contours, mm | red: shared screw/locating axes",
        fill="#182a3a",
    )
    for index, (name, data) in enumerate(closures.items()):
        report = data["report"]
        contours = [np.array(poly) for poly in report["seam_contours_local_mm"]]
        points = np.array([r["local_cut_xy_mm"] for r in report["connections"]])
        vertices = np.vstack([*contours, points])
        lo, hi = vertices.min(0), vertices.max(0)
        scale = min(620 / max(hi[0] - lo[0], 1), 420 / max(hi[1] - lo[1], 1))
        center = (lo + hi) / 2
        origin = np.array(
            [(index % columns) * 720 + 360, (index // columns) * 550 + 300]
        )

        def project(values, origin=origin, center=center, scale=scale):
            return (np.asarray(values) - center) * [scale, -scale] + origin

        draw.text(
            ((index % columns) * 720 + 25, (index // columns) * 550 + 65),
            f"{name}: {report['connection_count']} screws / BOTH shell pieces connected",
            fill="#182a3a",
        )
        for contour in contours:
            xy = project(contour)
            draw.line([tuple(v) for v in [*xy, xy[0]]], fill="#627e96", width=2)
        for i, (point, record) in enumerate(zip(points, report["connections"])):
            x, y = project(point)
            radius = max(4, report["profile"]["counterbore_diameter_mm"] / 2 * scale)
            draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                outline="#bb4c3a",
                width=2,
            )
            draw.line((x - 4, y, x + 4, y), fill="#bb4c3a", width=1)
            draw.line((x, y - 4, x, y + 4), fill="#bb4c3a", width=1)
            draw.text(
                (x + radius + 3, y - 6),
                f"{i + 1}: {record['thread']} x {record['screw_length_mm']:g}",
                fill="#872f24",
            )
    canvas.save(out / "shell_fastener_map.png")
