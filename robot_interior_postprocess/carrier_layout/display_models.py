"""Source-attributed visual CAD, separate from collision and mounting geometry."""

import hashlib

import numpy as np
import trimesh
from hardware import rigid_transform
from scipy.spatial import cKDTree


def connected_face_count(mesh):
    return len(
        trimesh.graph.connected_components(
            mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), engine="scipy"
        )
    )


def export_display_ply(mesh, path):
    """Keep float64 CAD coordinates and face colors across the file boundary.

    Trimesh's default float32 PLY export collapses microscopic source faces,
    causing a later process=True load to remove faces and split source parts.
    """
    vertices = np.empty(len(mesh.vertices), dtype=[(k, "<f8") for k in "xyz"])
    for i, key in enumerate("xyz"):
        vertices[key] = mesh.vertices[:, i]
    faces = np.empty(
        len(mesh.faces),
        dtype=[
            ("count", "u1"),
            ("indices", "<i4", (3,)),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
            ("alpha", "u1"),
        ],
    )
    faces["count"] = 3
    faces["indices"] = mesh.faces
    for i, key in enumerate(("red", "green", "blue", "alpha")):
        faces[key] = mesh.visual.face_colors[:, i]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\nproperty double x\nproperty double y\nproperty double z\n"
        f"element face {len(faces)}\nproperty list uchar int vertex_indices\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\nend_header\n"
    )
    path.write_bytes(header.encode("ascii") + vertices.tobytes() + faces.tobytes())


def presentation_mesh(mesh, budget=16000):
    """Keep every source island, and reject simplification that damages topology.

    A budget is a preference. Source triangles are retained when the preferred
    count cannot preserve connectivity, closed surfaces, holes and dimensions.
    """
    if len(mesh.faces) <= budget:
        return mesh.copy()
    groups = trimesh.graph.connected_components(
        mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), engine="scipy"
    )
    parts = mesh.submesh(groups, append=False, repair=False)
    result = []
    for part in parts:
        desired = min(
            len(part.faces), max(48, round(len(part.faces) * budget / len(mesh.faces)))
        )
        chosen = part
        while desired < len(part.faces):
            trial = part.simplify_quadric_decimation(face_count=desired)
            if (
                len(trial.faces)
                and connected_face_count(trial) == 1
                and trial.is_watertight == part.is_watertight
                and trial.euler_number == part.euler_number
                and np.max(abs(trial.bounds - part.bounds)) < 0.12
                and (
                    not part.is_volume
                    or abs(trial.volume - part.volume)
                    < max(0.01, abs(part.volume) * 0.01)
                )
            ):
                nearest = cKDTree(part.triangles_center).query(trial.triangles_center)[
                    1
                ]
                trial.visual.face_colors = part.visual.face_colors[nearest]
                chosen = trial
                break
            desired = min(len(part.faces), desired * 2)
        result.append(chosen)
    return trimesh.util.concatenate(result)


def load_display(spec, base, sources):
    required = {
        "file",
        "sha256",
        "interface_from_mesh",
        "source_url",
        "label",
        "registration",
    }
    if not isinstance(spec, dict) or set(spec) != required:
        raise ValueError(
            "Display CAD requires file/hash, rigid registration, label and source"
        )
    path = (base / spec["file"]).resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != spec["sha256"]:
        raise ValueError("Display CAD SHA256 mismatch")
    sources[str(path)] = digest
    mesh = trimesh.load(path, force="mesh", process=False)
    if mesh.is_empty or not np.isfinite(mesh.vertices).all():
        raise ValueError("Display CAD must have finite nonempty geometry")
    mesh.apply_transform(rigid_transform(spec["interface_from_mesh"]))
    mesh.metadata["display_reference"] = {
        **spec,
        "scale": 1,
        "validation": "visual reference only; mounting and collision geometry unchanged",
    }
    return mesh
