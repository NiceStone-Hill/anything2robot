"""Snapshots preserve real intermediate states and do not certify missing stages."""

import json

import numpy as np
import pytest
import trimesh
from carrier_layout.pipeline import run_pipeline
from carrier_layout.process_trace import ProcessRecorder


def test_assets_deduplicate_but_transforms_and_geometry_deltas_are_immutable(tmp_path):
    recorder = ProcessRecorder(tmp_path)
    mesh = trimesh.creation.box([10, 20, 30])
    transform = np.eye(4)
    first = recorder.item("board", mesh, "component", "torso", transform)
    recorder.capture("start", "开始", ["原始位置"], [first])
    transform[0, 3] = 7
    second = recorder.item("board", mesh, "component", "torso", transform)
    recorder.capture("move", "移动", ["实际移动 7 mm"], [second])
    second["transform"][0][3] = 999
    assert len(recorder.data["assets"]) == 1
    assert recorder.data["events"][0]["instances"][0]["transform"][0][3] == 0
    assert recorder.data["events"][1]["instances"][0]["transform"][0][3] == 7
    assert recorder.data["events"][1]["delta"]["moved"] == ["board"]
    changed = recorder.item(
        "board", mesh.copy().apply_scale(0.9), "component", "torso", transform
    )
    recorder.capture("cut", "几何变化", ["缩小测试实体"], [changed])
    assert recorder.data["events"][-1]["delta"]["geometry_changed"] == ["board"]
    assert np.allclose(mesh.extents, [10, 20, 30])


def test_bench_display_transition_is_not_reported_as_solver_motion(tmp_path):
    recorder = ProcessRecorder(tmp_path)
    first = recorder.item("carrier", trimesh.creation.box(), "carrier", "torso")
    recorder.capture("bench", "工作台", [], [first], frame="bench_local_mm")
    first["transform"][0][3] = 100
    recorder.capture("layout", "实际布局", [], [first])
    delta = recorder.data["events"][-1]["delta"]
    assert delta["coordinate_frame_changed"] and not delta["moved"]


def test_html_is_offline_script_safe_and_failure_keeps_previous_scene(tmp_path):
    recorder = ProcessRecorder(tmp_path)
    recorder.capture(
        "input",
        "</script><script>bad()</script>",
        ["测试"],
        [recorder.item("part", trimesh.creation.box(), "shell", "torso")],
    )
    recorder.capture("failed", "失败", ["真实报错"], state="failed")
    recorder.finish("failed", {"error": "真实报错"})
    data = json.loads((tmp_path / "process_trace.json").read_text())
    assert data["events"][0]["instances"] == data["events"][1]["instances"]
    page = (tmp_path / "process.html").read_text()
    assert "</script><script>bad()" not in page
    assert "__PROCESS_DATA__" not in page and "https://" not in page
    assert "真实报错" in (tmp_path / "process_changes.md").read_text()


def test_pipeline_failure_is_written_into_the_trace_without_hiding_error(tmp_path):
    file = tmp_path / "config.json"
    file.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "units": "mm",
                "gravity": {"world_m_s2": [0, 0, 0], "evidence": "invalid"},
            }
        )
    )
    out = tmp_path / "out"
    with pytest.raises(ValueError, match="gravity"):
        run_pipeline(file, out)
    data = json.loads((out / "process_trace.json").read_text())
    assert data["status"] == "failed"
    assert data["events"][-1]["stage"] == "failed"
