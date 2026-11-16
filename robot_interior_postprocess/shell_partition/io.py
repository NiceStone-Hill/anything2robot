"""Read existing result artifacts and render observations without changing them."""

import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw

from .geometry import candidates


def write_json(path, value):
    Path(path).write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )


def fingerprint(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_layout(directory):
    root = Path(directory).resolve()
    sources = {}

    def read(path, mesh=False):
        path = path.resolve()
        if not path.is_relative_to(root):
            raise ValueError("Input artifact escapes layout directory")
        sources[str(path)] = fingerprint(path)
        if not mesh:
            return json.loads(path.read_text())
        value = trimesh.load(path, force="mesh")
        if not value.is_volume:
            raise ValueError(f"Expected closed solid: {path}")
        return value

    manifest = read(root / "manifest.json")
    scene = read(root / "mechanical_scene.json")
    placement = read(root / "placement.json")
    if scene.get("units") != "millimeter" or placement.get("units") != "millimeter":
        raise ValueError("Scene and placement must explicitly use millimeter units")
    config = manifest["config"]
    shells = {}
    for name in scene["parts"]:
        if not isinstance(name, str) or Path(name).name != name or name in {".", ".."}:
            raise ValueError("Unsafe part ID")
        path = root / "meshes" / f"{name}_hollow_with_access_mm.stl"
        if not path.is_file():
            path = root / "meshes" / f"{name}_hollow_assembly_mm.stl"
        shells[name] = read(path, mesh=True)
    targets = []
    for motor in scene["motors"]:
        a, b = np.array(motor["axial_start_mm"]), np.array(motor["axial_end_mm"])
        delta = b - a
        transform = trimesh.geometry.align_vectors(
            [0, 0, 1], delta / np.linalg.norm(delta)
        )
        transform[:3, 3] = (a + b) / 2
        body = trimesh.creation.cylinder(
            radius=motor["radius_mm"],
            height=np.linalg.norm(delta),
            sections=32,
            transform=transform,
        )
        owners = list(dict.fromkeys([motor["parent"], motor["link"]]))
        if not set(owners) <= shells.keys():
            raise ValueError("Motor refers to unknown adjacent links")
        targets.append(
            {
                "id": motor["id"],
                "kind": "motor",
                "owners": owners,
                "mesh": body,
                "accuracy": "Optimization cylinder includes mounting allowance; not hardware CAD",
            }
        )
    for item in placement["placements"]:
        if item["component_id"] in config["motor_mapping"]:
            continue
        owner = item.get("part", config["placement_part"])
        if owner not in shells:
            raise ValueError("Component has an unknown owner link")
        instance = item["instance_id"]
        if (
            not isinstance(instance, str)
            or Path(instance).name != instance
            or instance in {".", ".."}
        ):
            raise ValueError("Unsafe hardware instance ID")
        # Read the exact exported world-space proxy rather than reinterpret old
        # placement versions' size/rotation conventions. Hull is conservative.
        proxy = read(root / "meshes" / f"{instance}_proxy_mm.stl", mesh=True)
        body = trimesh.convex.convex_hull(proxy)
        targets.append(
            {
                "id": item["instance_id"],
                "kind": "electronics",
                "owners": [owner],
                "mesh": body,
                "accuracy": "Saved planning envelope; physical dimensions not reverified",
            }
        )
    ids = [t["id"] for t in targets]
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate hardware target ID")
    catalog = {}
    for name, shell in shells.items():
        motors = [m for m in scene["motors"] if name in (m["link"], m["parent"])]
        catalog[name] = candidates(name, shell, motors)
    context = {
        "schema_version": 1,
        "units": "mm",
        "frame": scene["frame"],
        "product_front_and_gravity": "unknown; do not infer from world axes",
        "parts": [
            {
                "part_id": name,
                "bounds_mm": mesh.bounds.tolist(),
                "candidates": catalog[name],
                "task_target_ids": [t["id"] for t in targets if name in t["owners"]],
            }
            for name, mesh in shells.items()
        ],
        "motors": scene["motors"],
        "targets": [
            {k: v for k, v in t.items() if k != "mesh"}
            | {"bounds_mm": t["mesh"].bounds.tolist()}
            for t in targets
        ],
        "preserved_workflow": "Upstream layout/routing/geometry are immutable inputs. No automatic rerouting or relocation.",
        "missing_evidence": [
            "Actual motor mounting interfaces, roll and axial polarity",
            "Screw and connector coordinates and operation directions",
            "Load paths, physical hand/tool dimensions, full motion envelope",
        ],
    }
    return shells, targets, catalog, context, sources, config


def display_mesh(mesh):
    return (
        mesh.simplify_quadric_decimation(face_count=1800)
        if len(mesh.faces) > 1800
        else mesh.copy()
    )


def render_views(meshes, targets, path, title):
    """Orthographic evidence; hardware overlay is explicitly X-ray, not visibility."""
    image = Image.new("RGB", (1440, 550), "#f4f6f8")
    draw = ImageDraw.Draw(image)
    draw.text((16, 12), title + " | mm | RED hardware overlay = X-RAY", fill="#152435")
    bodies = [(display_mesh(m), "#b8cbdc") for m in meshes]
    bodies += [
        (display_mesh(t["mesh"]), "#85bf90" if t["kind"] == "support" else "#d68666")
        for t in targets
    ]
    views = [
        ([1, 0, 0], [0, 0, 1], "X / Z; looking along Y"),
        ([0, 1, 0], [0, 0, 1], "Y / Z; looking along X"),
        ([1, 0, 0], [0, 1, 0], "X / Y; looking along Z"),
    ]
    vertices = np.vstack([m.vertices for m, _ in bodies])
    for column, (u, v, label) in enumerate(views):
        axes = np.array([u, v]).T
        xy = vertices @ axes
        lo, hi = xy.min(0), xy.max(0)
        scale = 410 / max(hi - lo)
        center = (lo + hi) / 2
        for mesh, color in bodies:
            points = (mesh.vertices @ axes - center) * [scale, -scale] + [
                column * 480 + 240,
                280,
            ]
            depth = mesh.vertices @ np.cross(u, v)
            for face_id in np.argsort(depth[mesh.faces].mean(axis=1)):
                face = mesh.faces[face_id]
                draw.polygon(
                    [tuple(p) for p in points[face]], fill=color, outline="#65798a"
                )
        draw.text((column * 480 + 20, 38), label, fill="#152435")
        draw.text(
            (column * 480 + 20, 510),
            f"span: {hi[0] - lo[0]:.1f} x {hi[1] - lo[1]:.1f} mm",
            fill="#152435",
        )
    image.save(path)


def observations(shells, targets, out):
    directory = out / "observations"
    directory.mkdir()
    paths = [directory / "whole_robot.png"]
    render_views(
        list(shells.values()),
        targets,
        paths[0],
        "Whole robot; assembled world coordinates",
    )
    for i, (name, shell) in enumerate(shells.items()):
        path = directory / f"part_{i}.png"
        local = [t for t in targets if name in t["owners"]]
        render_views(
            [shell],
            local,
            path,
            f"Part {name}; targets: {', '.join(t['id'] for t in local)}",
        )
        paths.append(path)
    return paths
