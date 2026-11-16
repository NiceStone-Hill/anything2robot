"""Check actual converted source models and the exported browser payload."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import trimesh

ASSETS = Path(__file__).resolve().parents[1] / "assets/electronics"


def test_candidate_registration_preserves_size_instead_of_shrinking_to_fit():
    from cad_models import fitted_rotation

    size = np.array([127.0, 71.0, 19.0])
    envelope = np.array([118.0, 65.0, 32.0])
    rotation = fitted_rotation(size, envelope)
    assert np.allclose(rotation.T @ rotation, np.eye(3))
    assert np.isclose(np.linalg.det(rotation), 1)
    assert np.allclose(sorted(np.abs(rotation) @ size), sorted(size))
    assert np.any(np.abs(rotation) @ size > envelope)


@pytest.mark.skipif(
    not (ASSETS / "viewer.html").exists(), reason="Convert downloaded CAD first"
)
def test_downloaded_models_really_have_meshes_in_the_browser_view():
    catalog = json.loads((ASSETS / "converted/models.json").read_text())["models"]
    text = (ASSETS / "viewer.html").read_text()
    start = text.index("const scene = ") + len("const scene = ")
    scene = json.loads(text[start : text.index(";\n", start)])
    assert len(scene["components"]) == len(catalog) == 5
    by_id = {c["id"]: c for c in scene["components"]}
    for record in catalog:
        source = ASSETS / record["source"]
        assert (
            hashlib.sha256(source.read_bytes()).hexdigest() == record["source_sha256"]
        )
        mesh = trimesh.load(ASSETS / record["mesh"], force="mesh")
        displayed = by_id[record["id"]]["mesh"]
        assert len(displayed["faces"]) == len(mesh.faces) > 0
        assert np.allclose(np.ptp(displayed["vertices"], axis=0), mesh.extents)
        assert len(displayed["colors"]) == len(displayed["faces"])
