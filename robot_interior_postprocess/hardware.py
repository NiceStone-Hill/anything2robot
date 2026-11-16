"""Explicit, millimetre mounting interfaces. No size fitting or inferred holes."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import manifold3d as md
import numpy as np
import trimesh


def read_source(path, sources):
    path = Path(path).resolve()
    data = path.read_bytes()
    sources[str(path)] = hashlib.sha256(data).hexdigest()
    return json.loads(data)


def number(value, name, positive=True):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or (positive and value <= 0)
    ):
        raise ValueError(f"{name} requires a finite positive number")
    return float(value)


def rigid_transform(value):
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("Mount registration requires a rigid 4x4 transform") from exc
    if (
        matrix.shape != (4, 4)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-7)
        or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-7)
    ):
        raise ValueError(
            "Mount registration must be rigid; scaling and reflection are forbidden"
        )
    return matrix


def transform_solid(value, transform):
    return value.transform(np.asarray(transform)[:3, :].tolist())


def as_mesh(value):
    mesh = value.to_mesh()
    return trimesh.Trimesh(
        np.array(np.asarray(mesh.vert_properties)[:, :3], copy=True),
        np.array(mesh.tri_verts, copy=True),
        process=False,
    )


def as_mesh64(value):
    # Preserve CSG coordinates until the export boundary. Float32 conversion
    # here can collapse close vertices before STL encoding is even attempted.
    mesh = value.to_mesh64()
    return trimesh.Trimesh(
        np.array(np.asarray(mesh.vert_properties)[:, :3], copy=True),
        np.array(mesh.tri_verts, copy=True),
        process=False,
    )


def cylinder_at(point, radius, z0, z1):
    return md.Manifold.cylinder(z1 - z0, radius, circular_segments=48).translate(
        [float(point[0]), float(point[1]), z0]
    )


def load_interface(spec, base, target, sources):
    """Load a separately sourced interface and an explicit per-instance binding.

    The local frame has the PCB underside at z=0 and installation/tool access
    along +z. Bodies may include conservative keepouts or a closed source mesh.
    Registration evidence is always preserved, never upgraded to measurement.
    """
    required = {"definition", "world_from_interface", "registration", "fastener"}
    if (
        not isinstance(spec, dict)
        or not required <= set(spec)
        or set(spec) - required - {"hole_ids"}
    ):
        raise ValueError(
            "mounting requires definition, world_from_interface, registration, fastener"
        )
    if not isinstance(spec["definition"], str):
        raise TypeError("definition must be a local JSON path")
    transform = rigid_transform(spec["world_from_interface"])
    registration = spec["registration"]
    if (
        not isinstance(registration, dict)
        or set(registration) != {"status", "note"}
        or registration["status"] not in {"explicit_candidate", "measured"}
        or not isinstance(registration["note"], str)
        or not registration["note"].strip()
    ):
        raise ValueError("Explicit registration evidence is required")
    path = (Path(base) / spec["definition"]).resolve()
    raw = read_source(path, sources)
    if (
        not isinstance(raw, dict)
        or set(raw)
        - {
            "schema_version",
            "units",
            "id",
            "manufacturer",
            "model",
            "revision",
            "source",
            "board",
            "holes",
            "keepouts",
            "collision_mesh",
            "display_geometry",
        }
        or raw.get("schema_version") != 1
        or isinstance(raw.get("schema_version"), bool)
        or raw.get("units") != "mm"
    ):
        raise ValueError("Hardware definition requires schema_version=1, units=mm")
    for key in ("id", "manufacturer", "model", "revision"):
        if not isinstance(raw.get(key), str) or not raw[key].strip():
            raise ValueError(f"Hardware identity missing: {key}")
    if raw.get("display_geometry", "body") not in {"body", "pcb_only"}:
        raise ValueError("display_geometry must be body or pcb_only")
    source = raw.get("source")
    if (
        not isinstance(source, dict)
        or not isinstance(source.get("url"), str)
        or source.get("evidence")
        not in {"official_cad", "official_drawing", "measured", "synthetic_test"}
    ):
        raise ValueError("Hardware source URL and evidence are required")
    if "file" in source:
        source_path = (path.parent / source["file"]).resolve()
        digest = hashlib.sha256(source_path.read_bytes()).hexdigest()
        if digest != source.get("sha256"):
            raise ValueError("Hardware source SHA256 mismatch")
        sources[str(source_path)] = digest
    board = raw.get("board", {})
    if set(board) != {"outline_mm", "thickness_mm"}:
        raise ValueError("Board requires outline_mm and thickness_mm")
    thickness = number(board["thickness_mm"], "board thickness")
    outline = np.asarray(board["outline_mm"], dtype=float)
    if (
        outline.ndim != 2
        or outline.shape[1] != 2
        or not 3 <= len(outline) <= 256
        or not np.isfinite(outline).all()
    ):
        raise ValueError("Board outline requires 3..256 finite XY points")
    x, y = outline.T
    signed_area = float(x @ np.roll(y, -1) - y @ np.roll(x, -1)) / 2
    if signed_area < 0:
        outline = outline[::-1].copy()
    section = md.CrossSection([outline.tolist()])
    # Accept either contour winding, but reject self intersections rather than
    # silently letting the kernel repair them.
    area = abs(signed_area)
    if area < 1e-6 or not np.isclose(section.area(), area, atol=1e-5):
        raise ValueError("Board outline must be a simple nonzero polygon")
    body = section.extrude(thickness)
    holes = raw.get("holes")
    if not isinstance(holes, list) or not 2 <= len(holes) <= 32:
        raise ValueError("Mounting requires 2..32 explicit through holes")
    ids, normalized = set(), []
    for hole in holes:
        if not isinstance(hole, dict) or set(hole) != {
            "id",
            "center_mm",
            "diameter_mm",
        }:
            raise ValueError("Hole requires id, center_mm, diameter_mm")
        point = np.asarray(hole["center_mm"], float)
        if (
            not isinstance(hole["id"], str)
            or not hole["id"]
            or hole["id"] in ids
            or point.shape != (2,)
            or not np.isfinite(point).all()
        ):
            raise ValueError("Hole IDs must be unique; centres must be finite XY")
        ids.add(hole["id"])
        diameter = number(hole["diameter_mm"], "hole diameter")
        disk = md.CrossSection.circle(diameter / 2, 48).translate(point.tolist())
        if abs((disk - section).area()) > 1e-5:
            raise ValueError("Mount hole lies outside board outline")
        if any(
            np.linalg.norm(point - np.array(h["center_mm"]))
            <= (diameter + h["diameter_mm"]) / 2
            for h in normalized
        ):
            raise ValueError("Mount holes overlap")
        body -= cylinder_at(point, diameter / 2, -0.01, thickness + 0.01)
        normalized.append({**hole, "diameter_mm": diameter})
    sweep_parts = [as_mesh(body)]
    board_mesh = as_mesh(body).apply_transform(transform)
    keepouts = raw.get("keepouts", [])
    if not isinstance(keepouts, list) or len(keepouts) > 512:
        raise ValueError("keepouts must be a finite list")
    for box in keepouts:
        if not isinstance(box, dict) or set(box) != {"id", "min_mm", "max_mm"}:
            raise ValueError("Keepout requires id, min_mm, max_mm")
        lo, hi = np.asarray(box["min_mm"], float), np.asarray(box["max_mm"], float)
        if (
            lo.shape != (3,)
            or hi.shape != (3,)
            or not np.isfinite([lo, hi]).all()
            or np.any(hi <= lo)
        ):
            raise ValueError("Keepout bounds must be finite and ordered")
        keepout = md.Manifold.cube((hi - lo).tolist()).translate(lo.tolist())
        body += keepout
        sweep_parts.append(as_mesh(keepout))
    if "collision_mesh" in raw:
        mesh_spec = raw["collision_mesh"]
        if not isinstance(mesh_spec, dict) or set(mesh_spec) != {
            "file",
            "interface_from_mesh",
            "sha256",
        }:
            raise ValueError(
                "collision_mesh requires file, SHA256 and interface_from_mesh"
            )
        mesh_path = (path.parent / mesh_spec["file"]).resolve()
        sources[str(mesh_path)] = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
        if sources[str(mesh_path)] != mesh_spec["sha256"]:
            raise ValueError("Hardware collision mesh SHA256 mismatch")
        mesh = trimesh.load(mesh_path, force="mesh")
        if not mesh.is_volume:
            raise ValueError("Hardware collision mesh must be a closed oriented solid")
        value = md.Manifold(
            md.Mesh(
                np.asarray(mesh.vertices, np.float32), np.asarray(mesh.faces, np.uint32)
            )
        )
        if value.status() != md.Error.NoError:
            raise ValueError("Invalid hardware collision mesh")
        registered = transform_solid(
            value, rigid_transform(mesh_spec["interface_from_mesh"])
        )
        body += registered
        # Separate the PCB thickness layer from parts above/below it. Cutting
        # through the board plane disconnects leads and components which were
        # fused into one CAD solid. Sweeping their individual hulls preserves
        # underside steps without a whole-body convex hull or an expensive
        # nonconvex Minkowski sum. The union is conservative in every direction.
        # Converted float32 CAD can put the PCB surface a few nanometres
        # beyond its nominal plane. Include that sliver in the board layer,
        # rather than inadvertently reconnecting every underside component.
        plane_margin = 0.00001
        layers = [
            registered.trim_by_plane([0, 0, -1], plane_margin),
            registered.trim_by_plane([0, 0, 1], -plane_margin).trim_by_plane(
                [0, 0, -1], -thickness - plane_margin
            ),
            registered.trim_by_plane([0, 0, 1], thickness + plane_margin),
        ]
        for layer in layers:
            sweep_parts.extend(
                as_mesh(part) for part in layer.decompose() if part.volume() > 0
            )
    selected = normalized
    if "hole_ids" in spec:
        ids = spec["hole_ids"]
        if (
            not isinstance(ids, list)
            or not all(isinstance(i, str) for i in ids)
            or not 2 <= len(ids) <= len(normalized)
            or len(set(ids)) != len(ids)
            or not set(ids) <= {h["id"] for h in normalized}
        ):
            raise ValueError("hole_ids must select at least two unique declared holes")
        selected = [h for h in normalized if h["id"] in ids]
    profile = validate_fastener(spec["fastener"], thickness, selected)
    world = transform_solid(body, transform)
    record = {
        "target_id": target["id"],
        "hardware_id": raw["id"],
        "manufacturer": raw["manufacturer"],
        "model": raw["model"],
        "revision": raw["revision"],
        "source": source,
        "definition": str(path),
        "definition_sha256": sources[str(path)],
        "world_from_interface": transform.tolist(),
        "registration": registration,
        "board": board,
        "holes": normalized,
        "keepouts": keepouts,
        "selected_hole_ids": [h["id"] for h in selected],
        "fastener": profile,
        "collision_model": "explicit_board_and_keepouts_with_source_mesh"
        if "collision_mesh" in raw
        else "explicit_board_and_conservative_keepouts",
        "validation": "Candidate geometry; strength, electrical clearance and physical fit pending",
        "display_geometry": raw.get("display_geometry", "body"),
        "path_model": "Continuous union of conservative per-layer/per-component convex sweeps",
        "sweep_part_count": len(sweep_parts),
    }
    return {
        "record": record,
        "transform": transform,
        "body": world,
        "mesh": as_mesh(world),
        "local_body": body,
        "holes": selected,
        "outline": outline,
        "thickness": thickness,
        "fastener": profile,
        "sweep_parts": [m.copy().apply_transform(transform) for m in sweep_parts],
        "sweep_cache": {},
        "display_mesh": board_mesh
        if raw.get("display_geometry") == "pcb_only"
        else as_mesh(world),
    }


def validate_fastener(profile, thickness, holes):
    keys = {
        "thread",
        "shank_diameter_mm",
        "length_mm",
        "head_diameter_mm",
        "head_height_mm",
        "tool_radius_mm",
        "boss_diameter_mm",
        "standoff_height_mm",
        "min_wall_mm",
        "bottom_mm",
        "min_engagement_mm",
        "insert",
        "process_status",
    }
    if not isinstance(profile, dict) or set(profile) != keys:
        raise ValueError("Incomplete or unknown fastener profile fields")
    if not isinstance(profile["thread"], str) or profile["process_status"] not in {
        "prototype_uncalibrated",
        "coupon_calibrated",
    }:
        raise ValueError("Fastener thread and manufacturing process status required")
    p = dict(profile)
    for key in keys - {"thread", "insert", "process_status"}:
        p[key] = number(profile[key], key)
    insert = profile["insert"]
    if not isinstance(insert, dict) or set(insert) != {
        "id",
        "outer_diameter_mm",
        "hole_diameter_mm",
        "length_mm",
    }:
        raise ValueError("An explicit heat-set insert ID and dimensions are required")
    if not isinstance(insert["id"], str) or not insert["id"]:
        raise ValueError("Insert ID required")
    p["insert"] = {
        "id": insert["id"],
        **{k: number(v, k) for k, v in insert.items() if k != "id"},
    }
    depth = p["length_mm"] - thickness
    if (
        p["shank_diameter_mm"] >= min(h["diameter_mm"] for h in holes)
        or p["head_diameter_mm"] <= max(h["diameter_mm"] for h in holes)
        or p["boss_diameter_mm"]
        < max(insert["hole_diameter_mm"], insert["outer_diameter_mm"])
        + 2 * p["min_wall_mm"]
        or insert["hole_diameter_mm"] <= p["shank_diameter_mm"]
        or insert["outer_diameter_mm"] < insert["hole_diameter_mm"]
        or insert["length_mm"] > p["standoff_height_mm"] - p["bottom_mm"]
        or not p["min_engagement_mm"] <= min(depth, insert["length_mm"])
        or depth > p["standoff_height_mm"] - p["bottom_mm"]
    ):
        raise ValueError(
            "Fastener incompatible with holes, boss wall, engagement or blind-hole bottom"
        )
    return p


def interface_targets(targets, plan):
    """Use registered hardware only for targets which opt into explicit mounting."""
    interfaces = plan.get("interfaces", {})
    return [
        {
            **t,
            "mesh": interfaces[t["id"]]["mesh"],
            "sweep_parts": interfaces[t["id"]]["sweep_parts"],
            "sweep_cache": interfaces[t["id"]]["sweep_cache"],
            "accuracy": "Explicit mounting registration; see hardware_interfaces.json",
            "display_mesh": interfaces[t["id"]]["display_mesh"],
        }
        if t["id"] in interfaces
        else t
        for t in targets
    ]


def eagle_board_geometry(path):
    """Extract explicit board-level holes and a single Dimension-layer outline.

    Pads, vias and holes inside packages are deliberately excluded. Curved board
    edges are sampled at at most five-degree steps; slotted/multi-outline boards
    need a separate importer instead of being silently interpreted here.
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(path).getroot()
    plain = root.find("./drawing/board/plain")
    if plain is None:
        raise ValueError("Expected an XML Eagle board")
    wires = [dict(w.attrib) for w in plain.findall("wire") if w.get("layer") == "20"]
    if not wires:
        raise ValueError("No Dimension-layer board outline")
    segments = [
        (
            np.array([float(w["x1"]), float(w["y1"])]),
            np.array([float(w["x2"]), float(w["y2"])]),
            float(w.get("curve", 0)),
        )
        for w in wires
    ]
    start = segments[0][0].copy()
    current, outline = start.copy(), []
    while segments:
        matches = [
            (i, a, b, angle)
            if np.allclose(a, current, atol=1e-6)
            else (i, b, a, -angle)
            for i, (a, b, angle) in enumerate(segments)
            if np.allclose(a, current, atol=1e-6) or np.allclose(b, current, atol=1e-6)
        ]
        # At the initial vertex both incident edges are possible.
        if not matches or (outline and len(matches) != 1):
            raise ValueError("Ambiguous or disconnected Eagle board outline")
        i, a, b, angle = matches[0]
        segments.pop(i)
        if abs(angle) < 1e-8:
            outline.append(a.tolist())
        else:
            theta = np.deg2rad(angle)
            chord = b - a
            center = (a + b) / 2 + np.array([-chord[1], chord[0]]) / (
                2 * np.tan(theta / 2)
            )
            vec = a - center
            for alpha in np.linspace(
                0, theta, max(2, int(np.ceil(abs(angle) / 5))) + 1
            )[:-1]:
                c, s = np.cos(alpha), np.sin(alpha)
                outline.append((center + np.array([[c, -s], [s, c]]) @ vec).tolist())
        current = b
        if np.allclose(current, start, atol=1e-6) and segments:
            raise ValueError("Multiple Eagle board contours are not supported")
    if not np.allclose(current, start, atol=1e-6):
        raise ValueError("Open Eagle board outline")
    holes = [
        {
            "id": f"H{i + 1}",
            "center_mm": [float(h.get("x")), float(h.get("y"))],
            "diameter_mm": float(h.get("drill")),
        }
        for i, h in enumerate(plain.findall("hole"))
    ]
    return {"outline_mm": outline, "holes": holes}
