"""Finite support templates with explicit rigid-link and manufacturing-piece ownership.

Legacy envelope pads/trays and explicitly registered PCB interfaces are separate
templates. Hardware holes, gravity, load capacity and servo interfaces are never
inferred. Existing measured mounts remain indivisible fused features.
"""

from __future__ import annotations

from itertools import pairwise, product
from pathlib import Path

import manifold3d as md
import numpy as np
import trimesh

from .io import fingerprint

DEFAULTS = {
    "plate_mm": 2.0,
    "post_radius_mm": 2.0,
    "embed_mm": 1.0,
    "max_anchor_mm": 250.0,
    "fit_mm": 0.8,
    "tray_wall_mm": 2.0,
    "tray_height_mm": 6.0,
    "seam_keepout_mm": 3.0,
    "wire_radius_mm": 1.5,
    "beam_width": 2,
    "max_options_per_target": 3,
    "max_faces": 6,
    "rib_depth_mm": 8.0,
    "wire_relief_mm": 0.5,
}


def solid(mesh):
    value = md.Manifold(
        md.Mesh(
            np.asarray(mesh.vertices, np.float32), np.asarray(mesh.faces, np.uint32)
        )
    )
    if value.status() != md.Error.NoError:
        raise ValueError("Support input must be an oriented manifold")
    return value


def mesh_of(value):
    data = value.to_mesh()
    return trimesh.Trimesh(
        np.asarray(data.vert_properties)[:, :3],
        np.asarray(data.tri_verts),
        process=False,
    )


def union(items):
    return (
        md.Manifold.batch_boolean(list(items), md.OpType.Add)
        if items
        else md.Manifold()
    )


def intersection_volume(a, b):
    return abs(float((a ^ b).volume()))


def connected_material(value):
    # Interior void boundaries have negative signed volume and aren't fragments.
    return (
        not value.is_empty() and sum(p.volume() > 1e-6 for p in value.decompose()) == 1
    )


def rod(a, b, radius):
    return solid(
        trimesh.creation.cylinder(radius=radius, segment=np.array([a, b]), sections=20)
    )


def local_box(size, center, rotation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = rotation @ np.asarray(center)
    return solid(trimesh.creation.box(size, transform=transform))


def load_support_plan(path, targets, shells, sources):
    """Strict input contract. Generated requests and immutable features are separate."""
    import json

    path = Path(path).resolve()
    sources[str(path)] = fingerprint(path)
    raw = json.loads(path.read_text())
    if (
        not isinstance(raw, dict)
        or set(raw)
        - {"schema_version", "units", "settings", "targets", "existing_supports"}
        or raw.get("schema_version") != 1
        or isinstance(raw.get("schema_version"), bool)
        or raw.get("units") != "mm"
        or not isinstance(raw.get("targets"), list)
        or not isinstance(raw.get("settings", {}), dict)
    ):
        raise ValueError(
            "Support plan requires schema_version=1, units=mm and supported fields only"
        )
    settings = {**DEFAULTS, **raw.get("settings", {})}
    if set(settings) != set(DEFAULTS) or any(
        isinstance(v, bool)
        or not isinstance(v, (int, float))
        or not np.isfinite(v)
        or v <= 0
        for v in settings.values()
    ):
        raise ValueError("Unknown support setting or nonpositive/nonfinite value")
    for key, maximum in (
        ("beam_width", 8),
        ("max_options_per_target", 12),
        ("max_faces", 6),
    ):
        if int(settings[key]) != settings[key] or settings[key] > maximum:
            raise ValueError(f"{key} must be an integer in [1,{maximum}]")
        settings[key] = int(settings[key])
    if (
        settings["fit_mm"] > 5
        or settings["embed_mm"] > 5
        or settings["max_anchor_mm"] > 1000
    ):
        raise ValueError("Support clearances/embed/travel exceed prototype bounds")
    lookup = {t["id"]: t for t in targets}
    requests, used, interfaces = [], set(), {}
    for row in raw["targets"]:
        if (
            not isinstance(row, dict)
            or set(row) - {"target_id", "owner_link", "templates", "mounting"}
            or not {"target_id", "owner_link", "templates"} <= set(row)
            or not isinstance(row["target_id"], str)
            or not isinstance(row["owner_link"], str)
            or row["target_id"] not in lookup
        ):
            raise ValueError("Support request must reference a known target")
        key = (row["target_id"], row["owner_link"])
        target = lookup[row["target_id"]]
        if key in used or row["owner_link"] not in target["owners"]:
            raise ValueError("Duplicate support request or cross-rigid-link owner")
        used.add(key)
        templates = row["templates"]
        if (
            not isinstance(templates, list)
            or not templates
            or not all(isinstance(t, str) for t in templates)
            or len(set(templates)) != len(templates)
            or not set(templates)
            <= {
                "bearing_pad",
                "tray",
                "open_tray",
                "hole_standoffs",
                "ribbed_shelf",
                "removable_carrier",
            }
        ):
            raise ValueError("Unsupported support template")
        if target["kind"] == "motor":
            raise ValueError(
                "Motor mounts need explicit interfaces; import an existing measured mount instead"
            )
        interface_templates = {"hole_standoffs", "ribbed_shelf", "removable_carrier"}
        if set(templates) & interface_templates:
            if "mounting" not in row or not set(templates) <= interface_templates:
                raise ValueError(
                    "Interface templates require mounting; mixing envelope fallback is forbidden"
                )
            from hardware import load_interface

            interfaces[row["target_id"]] = load_interface(
                row["mounting"], path.parent, target, sources
            )
        elif "mounting" in row:
            raise ValueError("mounting requires an interface support template")
        requests.append(row)
    existing = []
    rows = raw.get("existing_supports", [])
    if not isinstance(rows, list) or len(rows) > 128:
        raise ValueError(
            "existing_supports requires an array with at most 128 features"
        )
    feature_ids = set()
    for row in rows:
        if (
            not isinstance(row, dict)
            or set(row) != {"id", "target_id", "owner_link", "mesh", "attachment"}
            or row["attachment"] != "fused"
            or not isinstance(row["id"], str)
            or not isinstance(row["target_id"], str)
            or not isinstance(row["owner_link"], str)
            or not isinstance(row["mesh"], str)
            or row["id"] in feature_ids
            or row["target_id"] not in lookup
            or row["owner_link"] not in lookup[row["target_id"]]["owners"]
        ):
            raise ValueError(
                "Existing support requires a unique ID, known target/link, mesh and fused attachment"
            )
        feature_ids.add(row["id"])
        mesh_path = (path.parent / row["mesh"]).resolve()
        sources[str(mesh_path)] = fingerprint(mesh_path)
        mesh = trimesh.load(mesh_path, force="mesh")
        value = solid(mesh)
        if not mesh.is_volume or not connected_material(value):
            raise ValueError(
                "Existing support must contain one connected material region"
            )
        if intersection_volume(value, solid(shells[row["owner_link"]])) < 0.05:
            raise ValueError(
                "Existing fused support has no positive-volume root in its owner shell"
            )
        existing.append(
            {**row, "mesh": mesh, "solid": value, "mesh_source": str(mesh_path)}
        )
    summary = {
        "schema_version": 1,
        "units": "mm",
        "settings": settings,
        "targets": requests,
        "existing_supports": [
            {k: v for k, v in e.items() if k not in {"mesh", "solid"}}
            | {"bounds_mm": e["mesh"].bounds.tolist()}
            for e in existing
        ],
        "hardware_interfaces": [i["record"] for i in interfaces.values()],
        "scope": "Legacy envelope supports or explicitly registered PCB supports; strength and physical calibration pending",
        "gravity_inferred": False,
    }
    return {
        "settings": settings,
        "targets": requests,
        "existing": existing,
        "summary": summary,
        "interfaces": interfaces,
    }


def route_obstacles(path, sources, radius):
    """Immutable swept wire tubes; never edit/reroute an input wire."""
    import json

    path = Path(path)
    if not path.is_file():
        return md.Manifold(), "pending_no_source_routing"
    sources[str(path.resolve())] = fingerprint(path)
    raw = json.loads(path.read_text())
    if raw.get("units") not in {None, "millimeter", "mm"}:
        raise ValueError("Unsupported route units")
    pieces = []
    for route in raw["routes"]:
        points = np.asarray(route["points_mm"], float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("Routes require finite world-mm points")
        for a, b in pairwise(points):
            if np.linalg.norm(b - a) > 1e-6:
                pieces.append(rod(a, b, radius / np.cos(np.pi / 20)))
        for point in points:
            # Circumscribed low-poly sphere conservatively covers the tube joint.
            pieces.append(
                solid(
                    trimesh.creation.icosphere(
                        subdivisions=1, radius=radius * 1.3
                    ).apply_translation(point)
                )
            )
    return union(
        pieces
    ), "protected_fixed_tubes" if pieces else "pending_no_routed_segments"


def within_half(value, candidate, owner_index, gap):
    vertices = mesh_of(value).vertices
    if not len(vertices):
        return False
    projection = vertices @ np.asarray(candidate["normal"])
    boundary = candidate["offset_mm"] + (-1 if owner_index == 0 else 1) * gap / 2
    return bool(
        projection.max() <= boundary + 1e-4
        if owner_index == 0
        else projection.min() >= boundary - 1e-4
    )


def attach_existing(feature, pieces, candidate, gap):
    """An imported mount may be assigned to either half, but is never cut."""
    support = feature["solid"]
    for index in range(2):
        if (
            within_half(support, candidate, index, gap)
            and intersection_volume(support, pieces[index]) >= 0.05
        ):
            return index
    raise ValueError(f"existing_support_crosses_seam_or_loses_root:{feature['id']}")


def support_options(
    target,
    request,
    owner,
    other,
    obstacles,
    wires,
    candidate,
    owner_index,
    gap,
    opening,
    settings,
    interface=None,
    wire_relief=None,
    access_obstacles=None,
):
    """Dispatch registered mounting templates or legacy envelope supports."""
    if interface is not None:
        from .mounting import mounting_options

        return mounting_options(
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
            wire_relief=wire_relief,
            access_obstacles=access_obstacles,
        )
    body = target["mesh"]
    axes = body.bounding_box_oriented.primitive.transform[:3, :3]
    faces = [axes[:, axis] * sign for axis, sign in product(range(3), (-1, 1))]
    faces.sort(key=lambda d: float(d @ np.asarray(opening)))
    options, trace = [], []
    for face_id, downward in enumerate(faces[: settings["max_faces"]]):
        rotation = trimesh.geometry.align_vectors([0, 0, 1], -downward)[:3, :3]
        local = body.vertices @ rotation
        lo, hi = local.min(0), local.max(0)
        center = (lo + hi) / 2
        thickness = settings["plate_mm"]
        variants = [
            (template, scale)
            for template in request["templates"]
            for scale in ((1.0, 0.5) if template in {"tray", "open_tray"} else (1.0,))
        ]
        for template, height_scale in variants:
            tag = {
                "template": template,
                "attachment_direction_world": downward.tolist(),
                "tray_height_scale": height_scale if template == "tray" else None,
            }
            floor_size = hi[:2] - lo[:2]
            if template in {"tray", "open_tray"}:
                floor_size = floor_size + 2 * (
                    settings["fit_mm"] + settings["tray_wall_mm"]
                )
            else:
                floor_size *= 0.85
            floor_center = np.array([*center[:2], lo[2] - thickness / 2])
            if template == "open_tray":
                # Leave no rim overhang beyond the entry end of a sideways cradle.
                # The battery slides out along the shell opening, and a later PCB
                # carrier may occupy space immediately above its saved envelope.
                for axis in range(2):
                    along = float(rotation[:, axis] @ opening)
                    if abs(along) > 0.5:
                        excess = settings["fit_mm"] + settings["tray_wall_mm"]
                        floor_size[axis] -= excess
                        floor_center[axis] -= np.sign(along) * excess / 2
            platform = local_box([*floor_size, thickness], floor_center, rotation)
            additions = [platform]
            if template in {"tray", "open_tray"}:
                wall, fit = settings["tray_wall_mm"], settings["fit_mm"]
                height = (
                    min(settings["tray_height_mm"], (hi[2] - lo[2]) * 0.5)
                    * height_scale
                )
                for axis, sign in product(range(2), (-1, 1)):
                    if (
                        template == "open_tray"
                        and sign * (rotation[:, axis] @ opening) > 0.5
                    ):
                        continue
                    size = [*floor_size, height + 0.25]
                    size[axis] = wall
                    point = np.array([*floor_center[:2], lo[2] + height / 2 - 0.125])
                    point[axis] += sign * ((hi[axis] - lo[axis]) / 2 + fit + wall / 2)
                    additions.append(local_box(size, point, rotation))
            platform_and_walls = union(additions)
            relieved = False
            if template == "open_tray" and wire_relief is not None:
                relieved = intersection_volume(platform_and_walls, wires) > 0.01
                platform_and_walls -= wire_relief
                platform -= wire_relief
            reason = None
            if not within_half(platform_and_walls, candidate, owner_index, gap):
                reason = "support_platform_crosses_split"
            elif intersection_volume(platform_and_walls, obstacles) > 0.01:
                reason = "support_platform_hits_hardware_or_support"
            elif intersection_volume(platform_and_walls, wires) > 0.01:
                reason = "support_platform_blocks_fixed_route"
            elif intersection_volume(platform_and_walls, union([owner, other])) > 0.01:
                reason = "support_platform_intersects_shell"
            if reason:
                trace.append({**tag, "reason": reason})
                continue
            roots, legs = [], []
            for u, v in product((-0.3, 0.3), repeat=2):
                start_local = floor_center.copy()
                start_local[:2] += np.array([u, v]) * floor_size
                start = rotation @ start_local
                found = False
                for a, b in ((0, 0), (0.35, 0), (-0.35, 0), (0, 0.35), (0, -0.35)):
                    direction = downward + a * rotation[:, 0] + b * rotation[:, 1]
                    direction /= np.linalg.norm(direction)
                    hits = owner.ray_cast(
                        start.tolist(),
                        (start + direction * settings["max_anchor_mm"]).tolist(),
                    )
                    hits = [
                        h
                        for h in hits
                        if np.linalg.norm(np.asarray(h.position) - start)
                        > thickness / 2 + 0.01
                    ]
                    if len(hits) < 2:
                        continue
                    if (
                        np.dot(hits[0].normal, direction) >= -0.05
                        or np.dot(hits[1].normal, direction) <= 0.05
                    ):
                        continue
                    anchor = np.asarray(hits[0].position)
                    wall_depth = np.linalg.norm(np.asarray(hits[1].position) - anchor)
                    embed = min(settings["embed_mm"], wall_depth * 0.4)
                    seam_distance = abs(
                        anchor @ np.asarray(candidate["normal"])
                        - candidate["offset_mm"]
                    )
                    if (
                        embed < 0.1
                        or seam_distance
                        < settings["seam_keepout_mm"] + settings["post_radius_mm"]
                    ):
                        continue
                    end = anchor + direction * embed
                    leg = rod(start, end, settings["post_radius_mm"])
                    if (
                        not within_half(leg, candidate, owner_index, gap)
                        or intersection_volume(leg, other) > 0.01
                        or intersection_volume(leg, obstacles) > 0.01
                        or intersection_volume(leg, wires) > 0.01
                        or intersection_volume(leg, platform) < 0.05
                        or intersection_volume(leg, owner) < 0.05
                    ):
                        continue
                    legs.append(leg)
                    roots.append(
                        {
                            "start_mm": start.tolist(),
                            "anchor_mm": anchor.tolist(),
                            "end_mm": end.tolist(),
                            "embed_mm": float(embed),
                        }
                    )
                    found = True
                    break
                if not found:
                    break
            if len(legs) != 4:
                trace.append(
                    {
                        **tag,
                        "reason": "four_owner_piece_roots_not_found",
                        "roots_found": len(legs),
                    }
                )
                continue
            value = union([platform_and_walls, *legs])
            final = union([owner, value])
            if not connected_material(value) or not connected_material(final):
                trace.append({**tag, "reason": "disconnected_support_or_owner"})
                continue
            record = {
                "id": f"{target['id']}:support",
                "target_id": target["id"],
                "owner_link": request["owner_link"],
                "owner_piece_index": owner_index,
                "attachment": "fused",
                "template": template,
                "status": "concept_geometry_checked",
                "release": "Fused support moves with owner piece; hardware retention interface remains pending",
                "roots": roots,
                "attachment_direction_world": downward.tolist(),
                "volume_mm3": float(value.volume()),
                "tray_wall_height_mm": float(height)
                if template in {"tray", "open_tray"}
                else None,
                "wire_relief_applied": relieved,
                "contact_model": "planning_envelope_face",
                "fixation": "pending_measured_hardware_interface_or_retainer",
                "strength": "pending",
                "wire_clear": True,
                "crosses_split": False,
            }
            options.append({"solid": value, "record": record})
            trace.append(
                {**tag, "status": "generated", "volume_mm3": float(value.volume())}
            )
    options.sort(key=lambda item: item["record"]["volume_mm3"])
    return options, trace
