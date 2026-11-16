"""Prepare topology-preserving previews from verified, downloaded CAD sources."""

import hashlib
from pathlib import Path

import trimesh
from carrier_layout.display_models import (
    connected_face_count,
    export_display_ply,
    presentation_mesh,
)
from common import dump, read

ROOT = Path(__file__).parent


def main():
    folder = ROOT / "assets/electronics"
    destination = folder / "presentation"
    sources = {
        r["component"]: r for r in read(destination / "download_sources.json")["assets"]
    }
    catalog = {r["id"]: r for r in read(folder / "converted/models.json")["models"]}
    registrations = {
        "arduino_uno_r3": [
            [0, 1, 0, 33.3817419052124],
            [-1, 0, 0, 26.67],
            [0, 0, 1, 5.365158462734666],
            [0, 0, 0, 1],
        ],
        "regulator_d24v50f5": [
            [1, 0, 0, 8.89],
            [0, 1, 0, 10.16],
            [0, 0, 1, 3.2874],
            [0, 0, 0, 1],
        ],
        "battery_holder_8aa": [
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 35.5],
            [0, 0, 0, 1],
        ],
        "slide_switch": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 4], [0, 0, 0, 1]],
    }
    mapping = {
        "U1_1": "arduino_uno_r3",
        "REGULATOR_1": "regulator_d24v50f5",
        "BATTERY_1": "battery_holder_8aa",
        "S1_1": "slide_switch",
    }
    records = {}
    for name, registration in registrations.items():
        source, record = sources[name], catalog[name]
        native = destination / source["downloaded_file"]
        if (
            hashlib.sha256(native.read_bytes()).hexdigest() != source["sha256"]
            or source["sha256"] != record["source_sha256"]
        ):
            raise ValueError("Native CAD/conversion provenance mismatch")
        mesh = trimesh.load(folder / record["full_mesh"], force="mesh")
        preview = presentation_mesh(mesh)
        target = destination / f"{name}.ply"
        export_display_ply(preview, target)
        preview = trimesh.load(target, force="mesh", process=False)
        before, after = connected_face_count(mesh), connected_face_count(preview)
        if before != after:
            raise ValueError("Presentation lost or fragmented a source part")
        records[name] = {
            "source": source,
            "file": target.name,
            "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            "converted_source_sha256": hashlib.sha256(
                (folder / record["full_mesh"]).read_bytes()
            ).hexdigest(),
            "source_faces": len(mesh.faces),
            "preview_faces": len(preview.faces),
            "source_connected_parts": before,
            "preview_connected_parts": after,
            "interface_from_mesh": registration,
            "scale": 1,
            "omitted_source_objects": record.get("omitted_source_objects", []),
            "registration": "Explicit visual registration; exact board revision, physical retention and CAD collisions remain unverified",
        }
        print(
            name,
            len(mesh.faces),
            "->",
            len(preview.faces),
            "faces;",
            before,
            "parts preserved",
            flush=True,
        )
    dump(
        destination / "models.json",
        {"models": records, "note": "Reference appearance only; not planning geometry"},
    )
    config_path = ROOT / "configs/cactus_carrier_assembly.json"
    config = read(config_path)
    for component in config["components"]:
        name = mapping[component["id"]]
        r = records[name]
        component["display"] = {
            "file": "../assets/electronics/presentation/" + r["file"],
            "sha256": r["sha256"],
            "interface_from_mesh": r["interface_from_mesh"],
            "source_url": r["source"]["url"],
            "label": catalog[name]["name"],
            "registration": r["registration"],
        }
    dump(config_path, config)


if __name__ == "__main__":
    main()
