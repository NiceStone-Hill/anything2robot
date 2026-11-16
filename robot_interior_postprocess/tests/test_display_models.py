"""Downloaded appearance must preserve parts and never redefine physical geometry."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import trimesh
from carrier_layout.display_models import connected_face_count, presentation_mesh
from carrier_layout.profiles import build_profile


def detailed_pair():
    a = trimesh.creation.box([10, 20, 5])
    for _ in range(3):
        a = a.subdivide()
    b = a.copy().apply_translation([25, 0, 0])
    a.visual.face_colors = [10, 100, 180, 255]
    b.visual.face_colors = [30, 35, 40, 255]
    return trimesh.util.concatenate([a, b])


def test_preview_keeps_every_part_color_hole_topology_and_dimensions():
    source = detailed_pair()
    vertices = source.vertices.copy()
    preview = presentation_mesh(source, budget=80)
    assert connected_face_count(preview) == connected_face_count(source) == 2
    assert preview.is_volume and preview.euler_number == source.euler_number
    assert np.allclose(preview.bounds, source.bounds, atol=0.12)
    assert np.array_equal(source.vertices, vertices)
    assert len(np.unique(preview.visual.face_colors, axis=0)) == 2


def test_bad_decimation_is_rejected_instead_of_exporting_broken_surfaces(monkeypatch):
    source = detailed_pair()

    def broken(mesh, **_):
        return trimesh.Trimesh(mesh.vertices.copy(), mesh.faces[1:].copy())

    monkeypatch.setattr(trimesh.Trimesh, "simplify_quadric_decimation", broken)
    preview = presentation_mesh(source, budget=80)
    assert preview.is_volume and len(preview.faces) == len(source.faces)
    assert connected_face_count(preview) == 2


def test_visual_model_changes_only_display_and_is_fingerprint_checked(tmp_path):
    spec = {
        "id": "part",
        "component_id": "board",
        "owner_link": "torso",
        "mass_kg": None,
        "carrier": {
            "plate_mm": 3,
            "margin_mm": 3,
            "ear_width_mm": 12,
            "dock_depth_mm": 6,
            "max_anchor_mm": 100,
        },
        "envelope": {
            "size_mm": [20, 10, 6],
            "evidence": "test",
            "retention": "pending",
        },
    }
    original = build_profile(spec, tmp_path, {})
    path = tmp_path / "visual.ply"
    detailed_pair().export(path)
    display = {
        "file": path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "interface_from_mesh": np.eye(4).tolist(),
        "source_url": "https://example.com/cad",
        "label": "Reference CAD",
        "registration": "explicit visual reference",
    }
    sources = {}
    updated = build_profile({**spec, "display": display}, tmp_path, sources)
    assert original["body"].volume() == updated["body"].volume()
    assert original["carrier"].volume() == updated["carrier"].volume()
    assert np.array_equal(original["bounds"], updated["bounds"])
    assert np.array_equal(original["collision_boxes"], updated["collision_boxes"])
    assert updated["display"].metadata["display_reference"]["scale"] == 1
    assert str(path) in sources
    with pytest.raises(ValueError, match="SHA256"):
        build_profile(
            {**spec, "display": {**display, "sha256": "invalid"}}, tmp_path, {}
        )


def test_downloaded_component_previews_preserve_pinned_cad_parts():
    root = Path(__file__).resolve().parents[1] / "assets/electronics/presentation"
    records = json.loads((root / "models.json").read_text())["models"]
    assert len(records) == 4
    for record in records.values():
        path = root / record["file"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == record["sha256"]
        mesh = trimesh.load(path, force="mesh", process=False)
        assert connected_face_count(mesh) == record["source_connected_parts"]
