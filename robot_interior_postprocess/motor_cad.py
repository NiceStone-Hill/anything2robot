"""Real motor CAD for visual previews; upstream collision cylinders stay unchanged."""

from pathlib import Path

import numpy as np
import trimesh
from common import dump, read
from motor_models import assign_motor_models

ASSETS = Path(__file__).parent / "assets/motors"
# STEP coordinates: shaft axis X for MG4005V2; Y for Damiao.
SOURCE_AXES = {"MG4005V2": [1, 0, 0], "DM6006": [0, 1, 0],
               "DM8006": [0, 1, 0], "DM10010": [0, 1, 0]}


def convert():
    """Convert downloaded STEP at its original mm scale into Z-axis display meshes."""
    import gmsh
    from cad_models import cad_mesh

    records = []
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    try:
        for source in read(ASSETS / "sources.json")["models"]:
            model_id = source["model_id"]
            mesh = cad_mesh(ASSETS / source["file"], [.25, .28, .32])
            center = mesh.bounds.mean(axis=0)
            mesh.apply_translation(-center)
            transform = trimesh.geometry.align_vectors(SOURCE_AXES[model_id], [0, 0, 1])
            mesh.apply_transform(transform)
            if len(mesh.faces) > 18000:
                mesh = mesh.simplify_quadric_decimation(face_count=18000)
            mesh.visual.face_colors = [90, 98, 110, 255]
            mesh.export(ASSETS / (model_id + ".ply"))
            trimesh.Scene(mesh).export(ASSETS / (model_id + ".glb"))
            records.append({**source, "mesh": model_id + ".ply",
                            "extents_mm": mesh.extents, "faces": len(mesh.faces),
                            "source_center_mm": center, "source_to_display": transform,
                            "registration_note": "Body bounding-box center; shaft axis aligned; roll and axial polarity require assembly verification"})
            print(model_id, mesh.extents, len(mesh.faces), flush=True)
    finally:
        gmsh.finalize()
    dump(ASSETS / "models.json", {"models": records, "units": "millimeter"})


def add_motor_previews(schema, scene, config, out):
    """Use real CAD only for mapped motors, in the existing placement local frame."""
    if not (ASSETS / "models.json").exists():
        return {}, []
    assignments = assign_motor_models(scene["motors"], config)
    dump(out / "motor_model_assignments.json", assignments)
    catalog = {m["model_id"]: m for m in read(ASSETS / "models.json")["models"]}
    motors = {m["id"]: m for m in scene["motors"]}
    meshes, records = {}, []
    for spec in schema["components"]:
        motor = motors.get(config["motor_mapping"].get(spec["id"]))
        if motor is None:
            continue
        model_id = motor["hardware_assignment"]["model_id"]
        if model_id is None or model_id not in catalog:
            records.append({"component": spec["id"], "model": model_id,
                            "status": "original_envelope_no_cad"})
            continue
        record = catalog[model_id]
        mesh = trimesh.load(ASSETS / record["mesh"], force="mesh")
        direction = np.asarray(motor["axial_end_mm"], dtype=float) - motor["axial_start_mm"]
        direction /= np.linalg.norm(direction)
        mesh.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], direction))
        path = out / "cad_meshes" / (spec["id"] + ".ply")
        mesh.export(path)
        spec["visualization"]["mesh"] = str(path.relative_to(out))
        spec["display_name"] = motor["id"] + " / " + model_id
        meshes[spec["id"] + "_1"] = mesh
        records.append({"component": spec["id"], "model": model_id,
                        "status": "downloaded_cad", "size_mm": mesh.extents,
                        "proxy_size_mm": spec["visualization"].get("proxy_size_mm"),
                        "source_page": record["source_page"], "scale": 1,
                        "registration_note": record["registration_note"]})
    return meshes, records


if __name__ == "__main__":
    convert()
