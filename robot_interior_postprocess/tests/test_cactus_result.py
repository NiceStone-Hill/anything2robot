"""Checks the actual exported cactus result when it has been generated locally."""

import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

OUT = Path(__file__).resolve().parents[1] / "outputs/cactus"
pytestmark = pytest.mark.skipif(
    not (OUT / "manifest.json").exists(), reason="Generate cactus output first"
)


def read(name):
    return json.loads((OUT / name).read_text())


def test_full_plan_has_all_components_and_every_net_branch():
    manifest = read("manifest.json")
    assert manifest["status"] == "concept_complete"
    placement = read("placement.json")
    routing = read("routing.json")
    assert len(placement["placements"]) == 7
    assert not placement["unplaced"]
    assert len(routing["routes"]) == 14
    assert not routing["unrouted"]
    assert not read("routing_validation.json")["endpoint_fallback_used"]
    assert read("geometry_validation.json")["all_components_clear"]
    assert read("geometry_validation.json")["all_routes_clear_of_shell"]
    scene = read("mechanical_scene.json")
    config = manifest["config"]
    for motor in scene["motors"]:
        circuit = next(
            k for k, v in config["motor_mapping"].items() if v == motor["id"]
        )
        pose = next(p for p in placement["placements"] if p["component_id"] == circuit)
        assert np.array_equal(pose["center_mm"], motor["center_mm"])


def test_exported_urdf_keeps_motion_joints_and_resolves_all_meshes():
    root = ET.parse(OUT / "cactus_layout.urdf").getroot()
    names = [n.get("name") for n in root.findall("link")]
    assert len(names) == len(set(names))
    assert len([j for j in root.findall("joint") if j.get("type") == "revolute"]) == 3
    assert (
        len([j for j in root.findall("joint") if j.get("name").endswith("_1_fixed")])
        == 7
    )
    for mesh in root.findall(".//mesh"):
        assert (OUT / mesh.get("filename")).is_file()
        assert mesh.get("scale") == "0.001 0.001 0.001"
    assert not root.findall("link/inertial")
