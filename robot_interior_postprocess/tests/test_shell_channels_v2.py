"""Multi-entry leads, bend-aware routing and evidence-based motor registration."""

import json

import manifold3d as md
import numpy as np
import pytest
from hardware import as_mesh64
from shell_channels.assembly import import_connections
from shell_channels.geometry import Surface, frame
from shell_channels.motors import motor_terminals, resolve_endpoint
from shell_channels.pipeline import DEFAULTS, route_one
from shell_channels.search import SurfaceField


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def mapping_bundle(tmp_path):
    write(
        tmp_path / "mechanical_scene.json",
        {"motors": [{"id": "actuator-42", "parent": "trunk", "link": "limb"}]},
    )
    write(
        tmp_path / "electrical/circuit_audit.json",
        {"motor_mapping": {"MOT_A": "actuator-42"}},
    )
    write(
        tmp_path / "electrical/circuit_mapping.json",
        {"instances": {"MOT_A": "instance-9"}},
    )
    return {"artifacts_directory": "electrical"}


def test_motor_joint_mapping_does_not_guess_body_side(tmp_path):
    summary = mapping_bundle(tmp_path)
    bindings, report = motor_terminals(tmp_path, summary)
    endpoint = {
        "instance_id": "instance-9",
        "port_id": "SIG",
        "position_mm": [9, 8, 7],
        "normal": [0, 0, 1],
        "owner_link": None,
        "evidence": "provisional",
    }
    result = resolve_endpoint(endpoint, bindings)
    assert result["motor_binding"]["motor_id"] == "actuator-42"
    assert result["motor_binding"]["joint_parent"] == "trunk"
    assert result["motor_binding"]["joint_child"] == "limb"
    assert result["owner_link"] is None
    assert result["terminal_status"] == "motor_terminal_registration_required"
    assert report["motors"][0]["registration"] is None
    assert result["position_mm"] == endpoint["position_mm"]


def test_registered_motor_port_replaces_proxy_and_keeps_provenance(tmp_path):
    summary = mapping_bundle(tmp_path)
    registration = {
        "schema_version": 1,
        "units": "mm",
        "motors": [
            {
                "motor_id": "actuator-42",
                "body_owner_link": "limb",
                "evidence": "fixture mount",
                "ports": {
                    "SIG": {
                        "position_mm": [1, 2, 3],
                        "normal": [0, 1, 0],
                        "evidence": "fixture CAD",
                    }
                },
            }
        ],
    }
    path = tmp_path / "ports.json"
    write(path, registration)
    bindings, _ = motor_terminals(tmp_path, summary, path)
    row = {
        "branch_id": "branch",
        "net_name": "signal",
        "length_mm": 12,
        "points_mm": [[0, 0, 0], [9, 8, 7]],
    }
    for side, instance in [("source", "board"), ("target", "instance-9")]:
        row.update(
            {
                f"{side}_instance": instance,
                f"{side}_pin": "SIG",
                f"{side}_anchor_mm": [9, 8, 7],
                f"{side}_normal": [0, 0, 1],
            }
        )
    write(tmp_path / "electrical/routing.json", {"routes": [row]})
    result = import_connections(
        tmp_path, {**summary, "radius_mm": 1}, {"board": "trunk"}, bindings
    )
    endpoint = result["cables"][0]["target"]
    assert endpoint["owner_link"] == "limb"  # cross-joint remains a real cross-joint
    assert endpoint["position_mm"] == [1, 2, 3]
    assert endpoint["legacy_planning_endpoint"]["position_mm"] == [9, 8, 7]
    assert endpoint["evidence"] == "fixture CAD"
    missing = resolve_endpoint({**endpoint, "port_id": "PWR"}, bindings)
    assert missing["owner_link"] is None
    registration["motors"][0]["body_owner_link"] = "unrelated-housing"
    write(path, registration)
    with pytest.raises(ValueError, match="adjacent"):
        motor_terminals(tmp_path, summary, path)
    registration["motors"][0]["body_owner_link"] = "trunk"
    registration["motors"][0]["ports"]["SIG"]["normal"] = [0, 0, 0]
    write(path, registration)
    with pytest.raises(ValueError, match="unit normals"):
        motor_terminals(tmp_path, summary, path)


@pytest.mark.parametrize("normal", [[0, 0, 1], [0.6, 0, 0.8]])
def test_multi_entry_escapes_component_footprint_without_moving_ports(tmp_path, normal):
    world = np.linalg.inv(frame(normal))
    world[:3, 3] = [123, -57, 92]
    shell = (
        md.Manifold.cube([100, 100, 4]).translate([-50, -50, -4]).transform(world[:3])
    )
    outer = (
        md.Manifold.cube([100, 100, 45]).translate([-50, -50, -4]).transform(world[:3])
    )
    # A wide board directly below the left port blocks the old fixed 10 mm entry.
    board = md.Manifold.cube([30, 30, 8]).translate([-35, -15, 2]).transform(world[:3])
    settings = {**DEFAULTS, "entry_search_radius_mm": 32, "entry_candidates": 4}
    surface = Surface(shell, shell, normal, settings, [board])
    wire = {"id": "multi-entry", "diameter_mm": 2}
    for side, uv in [("source", [-20, 0, 20]), ("target", [25, 0, 20])]:
        wire[side] = {
            "instance_id": side,
            "port_id": "p",
            "owner_link": "h",
            "position_mm": (world[:3, :3] @ uv + world[:3, 3]).tolist(),
            "normal": (world[:3, :3] @ [0, 0, 1]).tolist(),
            "evidence": "synthetic",
        }
    descriptor = {"outer": as_mesh64(outer), "closed_obstacles": [board]}
    changed, _, route = route_one(surface, descriptor, wire, [], tmp_path / "shell.stl")
    assert route["entry_search"][0]["rejected"]["unsafe_wall_entry"] > 0
    assert route["entry_offsets_mm"][0] > 15
    assert route["points_mm"][0] == pytest.approx(wire["source"]["position_mm"])
    assert route["points_mm"][-1] == pytest.approx(wire["target"]["position_mm"])
    points = np.asarray(route["points_mm"])
    for side, direction in [
        ("source", points[1] - points[0]),
        ("target", points[-2] - points[-1]),
    ]:
        assert direction / np.linalg.norm(direction) == pytest.approx(
            wire[side]["normal"], abs=1e-8
        )
    assert route["checks"]["export_reloaded_and_rechecked"]
    assert changed.volume() < surface.shell.volume()


def test_bend_aware_search_routes_around_obstacle():
    shell = md.Manifold.cube([80, 80, 4]).translate([-40, -40, -4])
    post = md.Manifold.cube([8, 10, 12]).translate([-4, -5, 0])
    surface = Surface(shell, shell, [0, 0, 1], DEFAULTS, [post])
    ends = [[-25, 0, 0], [25, 0, 0]]
    field = SurfaceField(surface, ends, 2)
    points, expanded = surface.plan(ends, 2, field=field)
    assert expanded > 0
    assert all(field.allowed(p[:2]) for p in points)
    assert np.max(np.abs(points[:, 1])) > 5
    assert points[0, :2] == pytest.approx(ends[0][:2])
    assert points[-1, :2] == pytest.approx(ends[1][:2])
