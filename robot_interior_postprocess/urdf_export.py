"""Kinematic visualization URDF. Original solid-body inertias are explicitly omitted."""

from xml.etree import ElementTree as ET

import numpy as np
from common import ROOT, dump
from scipy.spatial.transform import Rotation


def matrix_rpy(matrix):
    sy = np.hypot(matrix[0, 0], matrix[1, 0])
    if sy > 1e-8:
        return np.array(
            [
                np.arctan2(matrix[2, 1], matrix[2, 2]),
                np.arctan2(-matrix[2, 0], sy),
                np.arctan2(matrix[1, 0], matrix[0, 0]),
            ]
        )
    return np.array(
        [np.arctan2(-matrix[1, 2], matrix[1, 1]), np.arctan2(-matrix[2, 0], sy), 0.0]
    )


def joint_matrix(joint):
    value = np.eye(4)
    origin = joint.find("origin")
    if origin is not None:
        value[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        value[:3, :3] = Rotation.from_euler(
            "xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]
        ).as_matrix()
    return value


def export_urdf(config, scene, placement, out):
    source = ROOT / config["design_result"] / "urdf"
    tree = ET.parse(next(source.glob("*.urdf")))
    root = tree.getroot()
    root.set("name", "cactus_postprocess_concept")
    for node in root.findall("link/inertial"):
        for link in root.findall("link"):
            if node in list(link):
                link.remove(node)
    for mesh in root.findall(".//mesh"):
        part = mesh.get("filename").rsplit("/", 1)[-1].removesuffix(".stl")
        filename = f"{part}_hollow_with_access_mm.stl"
        if not (out / "meshes" / filename).exists():
            filename = f"{part}_hollow_assembly_mm.stl"
        mesh.set("filename", "meshes/" + filename)
        mesh.set("scale", "0.001 0.001 0.001")
    # Source exporter uses world-space STL vertices plus visual origins. Recover the
    # common design->URDF-root transform from each link's original visual geometry.
    children = {j.find("child").get("link") for j in root.findall("joint")}
    base = next(
        l.get("name") for l in root.findall("link") if l.get("name") not in children
    )
    transforms = {base: np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        remaining = []
        for joint in pending:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                remaining.append(joint)
                continue
            transforms[child] = transforms[parent] @ joint_matrix(joint)
        if len(remaining) == len(pending):
            raise ValueError("Source URDF graph is disconnected or cyclic")
        pending = remaining
    body_link = next(l for l in root.findall("link") if l.get("name") == "BODY")
    body_visual = body_link.find("visual")
    design_to_root = transforms["BODY"] @ joint_matrix(body_visual)
    records = []
    for p in placement.placements:
        is_motor = p.component_id in config["motor_mapping"]
        parent = (
            next(
                m["parent"]
                for m in scene["motors"]
                if m["id"] == config["motor_mapping"][p.component_id]
            )
            if is_motor
            else config["placement_part"]
        )
        world = np.eye(4)
        world[:3, 3] = np.asarray(p.center_mm) / 1000
        if p.rotation_matrix_local_to_shell is not None:
            world[:3, :3] = p.rotation_matrix_local_to_shell
        local = np.linalg.inv(transforms[parent]) @ design_to_root @ world
        link = ET.SubElement(root, "link", name=p.instance_id)
        visual = ET.SubElement(link, "visual")
        geo = ET.SubElement(visual, "geometry")
        ET.SubElement(
            geo,
            "mesh",
            filename=f"meshes/{p.instance_id}_proxy_mm.stl",
            scale="0.001 0.001 0.001",
        )
        # The report meshes are in assembly coordinates. Use a local mesh for URDF.
        from common import read
        from report import component_mesh

        mesh = component_mesh(p, read(out / "components.json"), out)
        local_mm = np.eye(4)
        local_mm[:3, :3] = world[:3, :3].T
        local_mm[:3, 3] = -world[:3, :3].T @ np.asarray(p.center_mm)
        mesh.apply_transform(local_mm)
        mesh.export(out / "meshes" / f"{p.instance_id}_urdf_local_mm.stl")
        visual.find("geometry/mesh").set(
            "filename", f"meshes/{p.instance_id}_urdf_local_mm.stl"
        )
        joint = ET.SubElement(
            root, "joint", name=p.instance_id + "_fixed", type="fixed"
        )
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=p.instance_id)
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(str(float(v)) for v in local[:3, 3]),
            rpy=" ".join(str(float(v)) for v in matrix_rpy(local[:3, :3])),
        )
        records.append(
            {
                "component": p.instance_id,
                "parent": parent,
                "xyz_m": local[:3, 3],
                "rpy": matrix_rpy(local[:3, :3]),
            }
        )
    for node in list(root.findall("gazebo")) + list(root.findall("transmission")):
        root.remove(node)
    ET.indent(tree, space="  ")
    tree.write(out / "cactus_layout.urdf", encoding="utf-8", xml_declaration=True)
    dump(
        out / "urdf_notes.json",
        {
            "scope": "Kinematic visualization only; solid-body inertias, transmissions and simulation plugins removed",
            "original_motion_limits": "Source URDF values retained; not certified against design interference limits",
            "components": records,
        },
    )
