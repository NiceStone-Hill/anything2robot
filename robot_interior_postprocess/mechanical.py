"""Read only trusted Anything2robot outputs, without importing its GUI environment."""

import pickle
from pathlib import Path

import numpy as np
import trimesh
from common import ROOT, dump
from motor_models import assign_motor_models


class Record:
    def __setstate__(self, state):
        if not isinstance(state, dict):
            raise TypeError("Unsupported legacy object state")
        self.__dict__.update(state)


class LegacyReader(pickle.Unpickler):
    def find_class(self, module, name):
        if module.startswith(
            (
                "modules.",
                "data_struct",
                "mesh_loader",
                "mesh_decomp",
                "interference_removal",
            )
        ):
            return Record
        return super().find_class(module, name)


def inverse_print_transform(csv_path):
    lines = Path(csv_path).read_text().splitlines()
    scale = float(lines[2].split(",")[0])
    translation = np.array([float(v) for v in lines[3].split(",")])
    rotation = np.array([[float(v) for v in row.split(",")] for row in lines[4:7]])
    if scale != 1000 or not np.allclose(rotation.T @ rotation, np.eye(3)):
        raise ValueError("Expected m-to-mm source scale and rigid print rotation")
    # Manufacturing STL already has mm units: p_print = R (p_assembly_mm + t).
    transform = np.eye(4)
    transform[:3, :3] = rotation.T
    transform[:3, 3] = -translation
    return transform


def load_scene(config, out):
    source = ROOT / config["design_result"]
    with (source / "robot_result.pkl").open("rb") as f:
        result = LegacyReader(f).load()
    motors = []
    queue = [result.link_tree]
    index = 0
    while queue:
        node = queue.pop(0)
        queue.extend(node.children)
        link = node.val
        if link.axis is None or np.linalg.norm(link.axis[1]) == 0:
            continue
        motor_start = index
        for axis in link.axis[1:]:
            values = np.asarray(result.motor_results[index])
            a = values[:3] * 10
            b = values[3:6] * 10
            direction = np.asarray(axis, dtype=float)
            direction /= np.linalg.norm(direction)
            motors.append(
                {
                    "id": f"motor_{link.name}" + (f"_axis{index - motor_start + 1}" if len(link.axis) > 2 else ""),
                    "link": link.name,
                    "parent": result.father_link_dict[link.name],
                    "axial_start_mm": a,
                    "axial_end_mm": b,
                    "center_mm": (a + b) / 2,
                    "radius_mm": float(values[6] * 10),
                    "axis": direction,
                    "joint_position_mm": (a + b) / 2,
                    "original_annotation_mm": np.asarray(link.axis[0]) * 10,
                    "geometry_accuracy": "Original optimization cylinder, including library mounting allowance; not exact motor CAD",
                    "electrical_accuracy": "Provisional 3-wire simulator interface, not verified for original physical motor",
                }
            )
            index += 1
    if index != len(result.motor_results):
        raise ValueError("Motor/tree count mismatch")
    assignments = assign_motor_models(motors, config)
    dump(out / "motor_model_assignments.json", assignments)
    meshes = {}
    transforms = {}
    for name in result.link_dict:
        folder = ROOT / config["manufacturing_result"] / "solid_tenon" / name
        transform = inverse_print_transform(folder / f"{name}_replaced.csv")
        mesh = trimesh.load(folder / f"{name}_solid_tenon.stl", force="mesh")
        if not mesh.is_volume:
            raise ValueError(f"{name}: input must bound positive watertight volume")
        if len(mesh.split(only_watertight=False)) != 1:
            raise ValueError(
                f"{name}: this input adapter accepts solid single-boundary source parts only"
            )
        mesh.apply_transform(transform)
        mesh.export(out / "meshes" / f"{name}_solid_assembly_mm.stl")
        meshes[name] = mesh
        transforms[name] = transform
    scene = {
        "units": "millimeter",
        "frame": "Anything2robot design assembly frame",
        "motors": motors,
        "parts": list(meshes),
        "print_to_assembly_mm": transforms,
        "source": str(source),
    }
    dump(out / "mechanical_scene.json", scene)
    return scene, meshes
