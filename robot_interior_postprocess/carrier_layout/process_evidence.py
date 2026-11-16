"""Enrich read-only presentation from already recorded evidence artifacts."""

import hashlib
import json
from pathlib import Path

import numpy as np


def attach_evidence(data, output):
    output = Path(output)
    context_path = output / "layout_constraints_context.json"
    if context_path.is_file() and "schema_domains" not in data:
        context = json.loads(context_path.read_text())
        data.setdefault("opening_catalog", context.get("opening_candidates", {}))
        data["schema_domains"] = {
            "owner_cavity_bounds_mm": context.get("owner_cavity_bounds_mm", {}),
            "gravity": context.get("gravity"),
        }
        data.setdefault("presentation_evidence_sources", {})[context_path.name] = (
            hashlib.sha256(context_path.read_bytes()).hexdigest()
        )
    raw_path = output / "layout_constraints_raw.json"
    if raw_path.is_file() and "restrict_schema_raw" not in data:
        data["restrict_schema_raw"] = json.loads(raw_path.read_text())
        data.setdefault("presentation_evidence_sources", {})[raw_path.name] = (
            hashlib.sha256(raw_path.read_bytes()).hexdigest()
        )
    space_path, scene_path = (
        output / "space_masks.npz",
        output / "mechanical_scene.json",
    )
    if (
        "schema_domains" in data
        and "schema_cavity_samples" not in data
        and space_path.is_file()
        and scene_path.is_file()
    ):
        parts = json.loads(scene_path.read_text())["parts"]
        samples = {}
        with np.load(space_path, allow_pickle=False) as space:
            usable, owners = space["usable"], space["part_ids"]
            origin, pitch = space["origin"], float(space["pitch"])
            for index, link in enumerate(parts, 1):
                indices = np.argwhere(usable & (owners == index))
                if len(indices):
                    selected = np.linspace(
                        0, len(indices) - 1, min(len(indices), 1200), dtype=int
                    )
                    samples[link] = {
                        "points_mm": (origin + indices[selected] * pitch).tolist(),
                        "source_voxels": len(indices),
                        "pitch_mm": pitch,
                    }
        data["schema_cavity_samples"] = samples
        data["schema_sample_note"] = (
            "Sampled usable-space voxel centers, not feasible whole-bundle placements. Overlays are X-ray display guides only."
        )
        for path in (space_path, scene_path):
            data.setdefault("presentation_evidence_sources", {})[path.name] = (
                hashlib.sha256(path.read_bytes()).hexdigest()
            )
