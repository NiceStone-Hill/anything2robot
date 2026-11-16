"""Use source CAD at native scale and assembly registrations; disclose missing CAD."""

import hashlib
from pathlib import Path

import numpy as np
import trimesh
from carrier_layout.display_models import load_display
from physical_wiring.catalog import read, rigid_matrix

ROOT = Path(__file__).resolve().parents[1]


def encoded(mesh):
    return {
        "vertices": mesh.vertices.tolist(),
        "faces": mesh.faces.tolist(),
        "face_colors": mesh.visual.face_colors[:, :3].tolist(),
    }


def config_base(source, config):
    # Input hashes retain the authoritative config location even after a saved snapshot.
    candidates = []
    hashes = read(source / "input_hashes.json")
    for name in hashes:
        path = Path(name)
        if path.suffix != ".json" or not path.is_file():
            continue
        try:
            if read(path) == config:
                candidates.append(path.parent)
        except (ValueError, OSError):
            continue
    if len(set(candidates)) != 1:
        raise ValueError(
            "Cannot resolve original assembly config for CAD registrations"
        )
    return candidates[0]


def load(source, proposals, motor_records):
    source = Path(source)
    config = read(source / "assembly_config_snapshot.json")
    base = config_base(source, config)
    poses = {
        p["id"]: rigid_matrix(p["world_from_interface"])
        for p in read(source / "carrier_placement.json")["placements"]
    }
    sources = {}
    models = []
    records = []
    represented = set()
    for spec in config["components"]:
        key = spec["id"]
        if "display" not in spec:
            records.append(
                {
                    "id": key,
                    "status": "missing_source_CAD",
                    "model": spec.get("component_id"),
                }
            )
            continue
        mesh = load_display(spec["display"], base, sources)
        mesh.apply_transform(poses[key])
        models.append(
            {
                "id": key,
                "label": spec["display"]["label"],
                "kind": "electronics",
                "mesh": encoded(mesh),
                "status": "source_CAD_visual_reference",
                "source_url": spec["display"]["source_url"],
            }
        )
        records.append(
            {
                "id": key,
                "status": "source_CAD_visual_reference",
                "model": spec["display"]["label"],
                "scale": 1,
                "world_from_interface": poses[key].tolist(),
                "registration": spec["display"]["registration"],
                "source_url": spec["display"]["source_url"],
                "source_sha256": spec["display"]["sha256"],
            }
        )
        represented.add(key)
        # The supply proxy reuses the actual battery pose, but its electrical suitability remains unselected.
        if spec.get("component_id") == "BATTERY":
            represented.add("PW_MOTOR_SUPPLY")
    catalog = {
        r["model_id"]: r for r in read(ROOT / "assets/motors/models.json")["models"]
    }
    motor_meshes = {}
    for item in motor_records:
        motor = item["mechanical_envelope"]
        assignment = motor.get("hardware_assignment", {})
        model_id = assignment.get("model_id")
        if model_id not in catalog:
            records.append(
                {
                    "id": item["device"],
                    "status": "missing_source_CAD",
                    "model": model_id,
                }
            )
            continue
        record = catalog[model_id]
        path = ROOT / "assets/motors" / record["mesh"]
        sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        mesh = trimesh.load(path, force="mesh", process=False)
        direction = np.asarray(motor["axial_end_mm"]) - motor["axial_start_mm"]
        direction /= np.linalg.norm(direction)
        transform = trimesh.geometry.align_vectors([0, 0, 1], direction)
        transform[:3, 3] = motor["center_mm"]
        mesh.apply_transform(transform)
        payload = encoded(mesh)
        motor_meshes[item["device"]] = payload
        records.append(
            {
                "id": item["device"],
                "status": "candidate_motor_source_CAD",
                "model": model_id,
                "scale": 1,
                "mesh_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "source_url": record["source_page"],
                "world_from_mesh": transform.tolist(),
                "registration": record["registration_note"],
                "physical_selection_confirmed": False,
            }
        )
        represented.add(item["device"])
    missing = []
    for proposal in proposals:
        if proposal["id"] in represented:
            continue
        missing.append(
            {
                "id": proposal["id"],
                "status": "unselected_component_no_real_CAD",
                "reason": "Exact module/connector not selected; physical model cannot be assigned",
                "bounds_mm": proposal.get("bounds_mm"),
            }
        )
    records += missing
    return (
        models,
        motor_meshes,
        {
            "records": records,
            "source_files": sources,
            "source_CAD_count": sum(
                r["status"]
                in {"source_CAD_visual_reference", "candidate_motor_source_CAD"}
                for r in records
            ),
            "missing_CAD_count": len(
                [
                    r
                    for r in records
                    if r["status"]
                    in {"missing_source_CAD", "unselected_component_no_real_CAD"}
                ]
            ),
            "all_parts_real_CAD": not missing
            and not any(r["status"] == "missing_source_CAD" for r in records),
            "collision_and_hardware_selection_revalidated": False,
        },
    )
