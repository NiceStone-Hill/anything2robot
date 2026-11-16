import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import motor_cad
from common import dump


def test_preview_preserves_scale_and_follows_motor_axis(tmp_path, monkeypatch):
    assets = tmp_path / "assets"
    assets.mkdir()
    trimesh.creation.box(extents=[20, 20, 10]).export(assets / "test.ply")
    dump(assets / "models.json", {"models": [{
        "model_id": "DM6006", "mesh": "test.ply",
        "source_page": "test", "registration_note": "test",
    }]})
    monkeypatch.setattr(motor_cad, "ASSETS", assets)
    (tmp_path / "cad_meshes").mkdir()
    scene = {"motors": [{"id": "m", "axial_start_mm": [100, 0, 0],
                         "axial_end_mm": [150, 0, 0]}]}
    schema = {"components": [{"id": "SERVO1", "visualization": {}}]}
    meshes, records = motor_cad.add_motor_previews(schema, scene, {
        "motor_mapping": {"SERVO1": "m"},
        "motor_model_mapping": {"m": "DM6006"},
    }, tmp_path)
    mesh = meshes["SERVO1_1"]
    assert np.allclose(mesh.extents, [10, 20, 20])
    assert np.allclose(mesh.bounds.mean(axis=0), [0, 0, 0])
    assert records[0]["scale"] == 1
    assert scene["motors"][0]["axial_end_mm"] == [150, 0, 0]


def test_unassigned_motor_keeps_envelope(tmp_path, monkeypatch):
    dump(tmp_path / "models.json", {"models": []})
    monkeypatch.setattr(motor_cad, "ASSETS", tmp_path)
    schema = {"components": [{"id": "S", "visualization": {"mesh": "original.stl"}}]}
    meshes, records = motor_cad.add_motor_previews(schema, {"motors": [{"id": "m"}]},
        {"motor_mapping": {"S": "m"}}, tmp_path)
    assert not meshes
    assert records[0]["status"] == "original_envelope_no_cad"
    assert schema["components"][0]["visualization"]["mesh"] == "original.stl"
