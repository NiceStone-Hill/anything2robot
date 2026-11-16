"""STL precision fallback must preserve topology rather than repair material."""

from io import BytesIO

import pytest
import trimesh
from carrier_layout.fabrication import export_solid
from hardware import as_mesh64


def test_float32_collapsed_feature_uses_ascii_without_changing_geometry(tmp_path):
    mesh = trimesh.creation.box(extents=[10, 10, 0.00001])
    mesh.apply_translation([1000, -1000, 1000])
    binary = trimesh.load(
        BytesIO(mesh.export(file_type="stl")), file_type="stl", force="mesh"
    )
    assert not binary.is_volume
    target = tmp_path / "thin_feature.stl"
    result = export_solid(mesh, target)
    assert target.read_bytes().startswith(b"solid ")
    assert result.is_volume
    assert result.volume == pytest.approx(mesh.volume, rel=1e-8)
    assert result.bounds == pytest.approx(mesh.bounds)


def test_open_source_is_rejected_without_writing_or_filling_hole(tmp_path):
    mesh = trimesh.creation.box()
    mesh.update_faces([True] * 11 + [False])
    target = tmp_path / "open.stl"
    with pytest.raises(ValueError, match="Exported STL"):
        export_solid(mesh, target)
    assert not target.exists()


def test_disconnected_solids_remain_rejected(tmp_path):
    first = trimesh.creation.box()
    second = first.copy().apply_translation([5, 0, 0])
    with pytest.raises(ValueError, match="one valid material solid"):
        export_solid(trimesh.util.concatenate([first, second]), tmp_path / "pair.stl")


def test_regular_solid_keeps_binary_export(tmp_path):
    target = tmp_path / "box.stl"
    result = export_solid(trimesh.creation.box(), target)
    assert not target.read_bytes().startswith(b"solid ")
    assert result.is_volume


def test_csg_mesh_conversion_preserves_close_world_vertices_and_writable_arrays():
    import manifold3d as md

    source = md.Manifold.cube([10, 10, 0.00001]).translate([1000, -1000, 1000])
    mesh = as_mesh64(source)
    assert mesh.is_volume
    assert mesh.extents[2] == pytest.approx(0.00001, rel=1e-6)
    assert mesh.vertices.flags.writeable
    assert mesh.faces.flags.writeable
