"""Read-only carrier-bundle adapter. No cactus names or coordinates in the kernel."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from hardware import cylinder_at

from .geometry import capsule, solid


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def snapshot(directory):
    return {
        str(p.relative_to(directory)): digest(p)
        for p in sorted(directory.rglob("*"))
        if p.is_file()
    }


def safe_path(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f"Bundle file missing or outside source directory: {relative}")
    return path


def load_mesh(path):
    # Exactly weld ASCII coordinates; default trimesh rounding loses tiny CSG edges.
    mesh = trimesh.load(path, force="mesh", process=False)
    vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
    return trimesh.Trimesh(vertices, inverse[mesh.faces], process=False)


def import_connections(source, summary, owners, motor_bindings=None):
    """Import every legacy branch; retain missing physical-owner evidence explicitly."""
    directory = summary.get("artifacts_directory")
    if not directory:
        raise ValueError(
            "No routing input; supply --connections with explicit physical terminals"
        )
    path = safe_path(source, f"{directory}/routing.json")
    data = read(path)
    if not data.get("routes") or "source_anchor_mm" not in data["routes"][0]:
        raise ValueError(
            "This routing format lacks terminal identities/normals; supply --connections"
        )
    cables = []
    for route in data["routes"]:
        ends = {}
        for side in ("source", "target"):
            instance = route[f"{side}_instance"]
            ends[side] = {
                "instance_id": instance,
                "port_id": route[f"{side}_pin"],
                "owner_link": owners.get(instance),
                "position_mm": route[f"{side}_anchor_mm"],
                "normal": route[f"{side}_normal"],
                "evidence": route.get(
                    f"{side}_endpoint_kind", "unmeasured_model_anchor"
                ),
            }
        if motor_bindings:
            from .motors import resolve_endpoint

            ends = {
                side: resolve_endpoint(endpoint, motor_bindings)
                for side, endpoint in ends.items()
            }
        cables.append(
            {
                "id": route["branch_id"],
                "net": route["net_name"],
                **ends,
                "diameter_mm": 2 * summary["radius_mm"],
                "evidence": "legacy planning branch, physical cable and junction design pending",
                "baseline_points_mm": route["points_mm"],
                "baseline_length_mm": route["length_mm"],
            }
        )
    return {
        "schema_version": 1,
        "units": "mm",
        "cables": cables,
        "provenance": str(path),
        "unrouted_input": summary.get("unrouted", []),
    }


def validate_connections(data):
    if (
        data.get("schema_version") != 1
        or data.get("units") != "mm"
        or not isinstance(data.get("cables"), list)
        or not data["cables"]
    ):
        raise ValueError(
            "Connections require schema_version=1, units=mm, nonempty cables"
        )
    ids = set()
    for wire in data["cables"]:
        if not isinstance(wire.get("id"), str) or not wire["id"] or wire["id"] in ids:
            raise ValueError("Cable IDs must be nonempty and unique")
        ids.add(wire["id"])
        diameter = wire.get("diameter_mm")
        if (
            isinstance(diameter, bool)
            or not isinstance(diameter, (int, float))
            or not np.isfinite(diameter)
            or diameter <= 0
        ):
            raise ValueError("Cable diameter must be finite and positive")
        if not wire.get("evidence"):
            raise ValueError("Each cable requires parameter evidence")
        for side in ("source", "target"):
            endpoint = wire[side]
            if not all(endpoint.get(k) for k in ("instance_id", "port_id", "evidence")):
                raise ValueError("Endpoints require instance, port and evidence")
            p, n = (
                np.asarray(endpoint["position_mm"], float),
                np.asarray(endpoint["normal"], float),
            )
            if (
                p.shape != (3,)
                or n.shape != (3,)
                or not np.isfinite([p, n]).all()
                or not np.isclose(np.linalg.norm(n), 1)
            ):
                raise ValueError("Endpoints require finite positions and unit normals")
    return data


def load_bundle(
    directory, connections=None, motor_registration=None, *, geometry_only=False
):
    source = Path(directory).resolve()
    manifest = read(source / "manifest.json")
    if (
        manifest.get("pipeline") != "carrier_assembly_v1"
        or manifest.get("stage") != "completed_with_explicit_blockers"
    ):
        raise ValueError(
            "Source must be a completed carrier candidate, not a failed/interrupted run"
        )
    inventory = read(source / "manufacturing_manifest.json")
    placement = read(source / "carrier_placement.json")
    ownership = read(source / "support_ownership.json")["supports"]
    owners = {r["target_id"]: r["owner_link"] for r in ownership}
    parts = {}
    for record in inventory["printed_parts"]:
        path = safe_path(source, record["file"])
        parts[record["file"]] = solid(load_mesh(path))
    closures = read(source / "shell_connections.json")["links"]
    obstacles = []
    for pose in placement["placements"]:
        lo, hi = np.asarray(pose["bounds_mm"])
        obstacles.append(
            solid(trimesh.creation.box(hi - lo).apply_translation((hi + lo) / 2))
        )
    # Preserve actual motors as well as the protected voxel masks.
    for motor in read(source / "mechanical_scene.json").get("motors", []):
        obstacles.append(
            solid(
                trimesh.creation.cylinder(
                    radius=motor["radius_mm"],
                    sections=32,
                    segment=[motor["axial_start_mm"], motor["axial_end_mm"]],
                )
            )
        )
    # Reserve shell screw and straight-driver envelopes for wiring and lay-in.
    for closure in closures.values():
        transform = np.asarray(closure["world_from_cut_frame"])
        for connection in closure["connections"]:
            point, seat = (
                connection["local_cut_xy_mm"],
                connection["head_seat_local_z_mm"],
            )
            spec, access = connection["fastener_spec"], connection["access"]
            screw = cylinder_at(
                point,
                connection["screw_diameter_mm"] / 2,
                seat - connection["screw_length_mm"],
                seat,
            )
            screw += cylinder_at(
                point, spec["head_diameter_mm"] / 2, seat, seat + spec["head_height_mm"]
            )
            obstacles.append(screw.transform(transform[:3]))
            start = np.asarray(access["head_seat_world_mm"])
            end = np.asarray(access["loading_start_world_mm"])
            obstacles.append(
                capsule(start, end, access["max_driver_shaft_diameter_mm"] / 2)
            )
    surfaces = {}
    for owner, closure in closures.items():
        files = [
            p["file"]
            for p in inventory["printed_parts"]
            if p.get("owner_link") == owner
        ]
        connections_for_link = closure["connections"]
        if not connections_for_link:
            continue
        shell_file = connections_for_link[0]["owner_piece_file"]
        if shell_file not in files:
            raise ValueError(
                "Carrier shell ownership does not match manufacturing inventory"
            )
        base = solid(
            load_mesh(safe_path(source, f"meshes/{owner}_hollow_assembly_mm.stl"))
        )
        outer = load_mesh(safe_path(source, f"meshes/{owner}_solid_assembly_mm.stl"))
        surfaces[owner] = {
            "file": shell_file,
            "shell": parts[shell_file],
            "base": base ^ parts[shell_file],
            "outer": outer,
            "normal": placement["openings"][owner]["normal"],
            "obstacles": [
                *obstacles,
                *[
                    p
                    for name, p in parts.items()
                    if name != shell_file and name not in files
                ],
            ],
            "closed_obstacles": [
                *obstacles,
                *[p for name, p in parts.items() if name != shell_file],
            ],
        }
    with np.load(source / "space_masks.npz") as masks:
        space = {
            k: masks[k].copy()
            for k in ("origin", "pitch", "motor_block", "protected_material")
        }
    if geometry_only:
        return {
            "source": source,
            "parts": parts,
            "surfaces": surfaces,
            "space": space,
            "inventory": inventory,
            "manifest": manifest,
            "connections": None,
            "motor_registration": {"status": "not_loaded_geometry_only"},
        }
    from .motors import motor_terminals

    routing_summary = read(source / "routing_summary.json")
    bindings, motor_report = motor_terminals(
        source, routing_summary, motor_registration
    )
    if connections:
        wire_data = read(connections)
    else:
        wire_data = import_connections(source, routing_summary, owners, bindings)
    return {
        "source": source,
        "parts": parts,
        "surfaces": surfaces,
        "space": space,
        "inventory": inventory,
        "manifest": manifest,
        "connections": validate_connections(wire_data),
        "motor_registration": motor_report,
    }
