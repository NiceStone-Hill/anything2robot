import sys
from pathlib import Path

import numpy as np
import pytest
import trimesh

HERE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(HERE), "/home/qinxuan/projects/voxel_tool/src"]
from common import cylinder_mask
from layout import fixed_motors
from mechanical import inverse_print_transform
from routing import dense_path
from space import mask_mesh


def test_print_inverse_preserves_assembly_mm(tmp_path):
    rotation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    translation = np.array([14.0, -6.0, 20.0])
    csv = tmp_path / "transform.csv"
    csv.write_text(
        "0,0,0\n1,1,1\n1000,0,0\n14,-6,20\n"
        + "\n".join(",".join(str(x) for x in row) for row in rotation)
    )
    original = np.array([37.0, -10.0, 45.0])
    printed = rotation @ (original + translation)
    restored = inverse_print_transform(csv) @ np.r_[printed, 1]
    assert np.allclose(restored[:3], original)


def test_solid_is_not_placement_air_and_mounts_are_reserved():
    shape = (21, 21, 21)
    origin = np.array([-10.0, -10.0, -10.0])
    pitch = 1.0
    occupied = np.ones(shape, bool)
    motor = cylinder_mask(shape, origin, pitch, [0, 0, -5], [0, 0, 5], 3)
    usable = ~occupied & ~motor
    assert not usable.any()
    assert motor[10, 10, 10]
    assert not motor[0, 0, 0]


def test_fixed_motor_box_is_removed_from_free_space():
    space = {
        "material": np.zeros((40, 40, 40), bool),
        "motor_block": np.zeros((40, 40, 40), bool),
        "usable": np.ones((40, 40, 40), bool),
        "origin": np.array([-20.0, -20.0, -20.0]),
        "pitch": 1.0,
    }
    motor = {
        "id": "m",
        "axial_start_mm": [0, 0, -3],
        "axial_end_mm": [0, 0, 3],
        "center_mm": [0, 0, 0],
        "radius_mm": 2,
    }
    fixed = fixed_motors(
        {"motors": [motor]},
        {"motor_mapping": {"SERVO1": "m"}, "motor_clearance_mm": 1},
        space,
    )
    p = fixed[0]
    s = tuple(slice(a, a + n) for a, n in zip(p.voxel_start, p.voxel_shape))
    assert space["motor_block"][s].all()
    assert not space["usable"][s].any()
    assert np.array_equal(p.center_mm, motor["center_mm"])


def test_cavity_boolean_really_removes_volume():
    mask = np.zeros((21, 21, 21), bool)
    mask[5:16, 5:16, 5:16] = True
    tool = mask_mesh(mask, [-10, -10, -10], 1)
    outer = trimesh.creation.box(extents=[24] * 3)
    shell = trimesh.boolean.difference([outer, tool], engine="manifold")
    assert shell.is_volume
    assert 0 < shell.volume < outer.volume
    assert not shell.contains([[0, 0, 0]])[0]
    assert shell.contains([[11, 0, 0]])[0]


def test_validation_densifies_simplified_route():
    cells = dense_path([(0, 0, 0), (0, 0, 5), (3, 0, 5)])
    assert (0, 0, 3) in cells
    assert (2, 0, 5) in cells
    assert len(cells) == 9


def test_real_eagle_import_preserves_signal_identity_and_adapts_supply(tmp_path):
    from circuit import import_circuit
    from common import read

    config = read(HERE / "configs/cactus.json")
    scene = {"motors": [{"id": i} for i in config["motor_mapping"].values()]}
    _topology, data = import_circuit(config, tmp_path, scene)
    d = {
        n["name"]: {(t["component"], t["pin"]) for t in n["terminals"]}
        for n in data["nets"]
    }
    assert d["U1_D9"] == {("U1", "D9"), ("SERVO2", "SIG")}
    assert d["U1_D10"] == {("U1", "D10"), ("SERVO1", "SIG")}
    assert ("BATTERY", "+") in d["BATTERY_RAW"]
    assert ("REGULATOR", "VIN") in d["BATTERY_RAW"]
    assert ("REGULATOR", "VOUT") in d["P1+"]
    assert all(c["id"] != "P1" for c in data["components"])
    assert ("U1", "GND") in d["P1-"]
    assert len(d["P1-"]) == len(
        next(n for n in data["nets"] if n["name"] == "P1-")["terminals"]
    )


def test_import_rejects_bom_quantity_mismatch(tmp_path):
    import shutil

    from circuit import import_circuit
    from common import ROOT, read

    config = read(HERE / "configs/cactus.json")
    source = ROOT / config["circuit_directory"]
    copied = tmp_path / "inputs"
    copied.mkdir()
    for name in ["Bodacious Bruticus.brd", "Bodacious Bruticus.sch", "bom.csv"]:
        shutil.copy(source / name, copied / name)
    bom = copied / "bom.csv"
    bom.write_text(bom.read_text(encoding="utf-8-sig").replace('"U1",1', '"U1",2'))
    config["circuit_directory"] = str(copied)
    with pytest.raises(ValueError, match="quantity"):
        import_circuit(config, tmp_path, {"motors": []})


def test_fast_solid_query_distinguishes_material_from_real_cavity():
    from solid_query import SolidQuery

    outer = trimesh.creation.box(extents=[20] * 3)
    inner = trimesh.creation.box(extents=[10] * 3)
    shell = trimesh.boolean.difference([outer, inner], engine="manifold")
    points = np.array([[0, 0, 0], [7, 0, 0], [15, 0, 0], [-7, 0, 0]])
    assert SolidQuery(outer).contains(points).tolist() == [True, True, False, True]
    assert SolidQuery(shell).contains(points).tolist() == [False, True, False, True]
    sample = np.random.default_rng(7).uniform(-12, 12, (100, 3))
    assert np.array_equal(SolidQuery(shell).contains(sample), shell.contains(sample))


def test_urdf_rpy_roundtrip_handles_cardinal_and_gimbal_rotations():
    from scipy.spatial.transform import Rotation
    from urdf_export import matrix_rpy

    for angles in [[0, 0, 0], [0, 90, 0], [30, -90, 40], [90, 0, -90]]:
        matrix = Rotation.from_euler("xyz", angles, degrees=True).as_matrix()
        assert np.allclose(
            Rotation.from_euler("xyz", matrix_rpy(matrix)).as_matrix(), matrix
        )


def test_viewer_serializes_fresh_mechanical_numpy_coordinates():
    import json

    from report import joint_payload

    motor = {
        "link": "arm",
        "center_mm": np.array([1.0, 2.0, 3.0]),
        "axis": np.array([0.0, 1.0, 0.0]),
    }
    encoded = joint_payload({"motors": [motor]})
    assert json.loads(encoded) == [
        {"name": "arm", "center": [1.0, 2.0, 3.0], "axis": [0.0, 1.0, 0.0]}
    ]
