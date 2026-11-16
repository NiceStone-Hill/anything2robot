"""Evidence extraction, circuit continuity and fail-closed routing contracts."""

import copy
import json

import numpy as np
import pytest
from physical_wiring.catalog import board_ports, build_catalog, rigid_matrix, sha256
from physical_wiring.checks import review
from physical_wiring.pipeline import load_routing_contract
from physical_wiring.registration import compile_connections, template
from physical_wiring.topology import scenario
from shell_channels.assembly import snapshot, validate_connections


@pytest.fixture
def catalog():
    return build_catalog()


def context(ids):
    return {
        "owners": {key: "body" for key in ids},
        "poses": {key: np.eye(4).tolist() for key in ids},
        "placed_instances": ids,
        "links": ["body"],
        "motor_adjacent_links": {},
    }


def synthetic():
    # Synthetic bench fixture, never written as cactus hardware evidence.
    components = [
        {
            "id": key,
            "model": "test fixture",
            "evidence": "fixture",
            "ports": {
                pin: {"kind": "bus", "protocol": "CAN", "conductor_role": pin}
                for pin in ("H", "L")
            },
        }
        for key in ("A", "B")
    ]
    nets = [
        {
            "id": pin,
            "protocol": "CAN",
            "terminals": [{"instance_id": key, "port_id": pin} for key in ("A", "B")],
        }
        for pin in ("H", "L")
    ]
    cables = [
        {
            "id": n["id"],
            "net": n["id"],
            "source": n["terminals"][0],
            "target": n["terminals"][1],
            "bundle_id": "pair",
        }
        for n in nets
    ]
    plan = {
        "schema_version": 1,
        "protocol": "CAN",
        "components": components,
        "nets": nets,
        "cables": cables,
    }
    ctx = context(["A", "B"])
    rotation = [[0, -1, 0, 10], [1, 0, 0, 20], [0, 0, 1, 30], [0, 0, 0, 1]]
    ctx["poses"]["A"] = rotation
    reg = template(plan, ctx, {"profiles": {}})
    reg.update(selection_confirmed=True, motor_bus_voltage_range_v=[12, 12])
    for record in reg["components"].values():
        record.update(
            model="synthetic",
            revision="1",
            selection_confirmed=True,
            selection_evidence="fixture",
            mounting_evidence="fixture",
        )
        for name, pin in record["ports"].items():
            pin.update(
                physical_pin=name,
                connector_id="J1",
                cable_exit_id="J1_EXIT",
                evidence="fixture",
                position_mm=[1, 2, 3],
                normal=[1, 0, 0],
            )
    reg["cables"]["pair"] = {
        "diameter_mm": 2,
        "min_bend_radius_mm": 5,
        "evidence": "fixture",
    }
    return plan, reg, ctx


def test_official_board_pads_are_distinct_and_not_cable_exits(catalog):
    pads = catalog["profiles"]["arduino_uno_rev3e"]["physical_pads"]
    assert len(pads) == 32
    assert {key for key, p in pads.items() if p["board_signal"] == "GND"} == {
        "POWER:6",
        "POWER:7",
        "IOH:7",
    }
    assert all(p["local_cable_exit_mm"] is None for p in pads.values())
    assert pads["POWER:8"]["board_signal"] == "VIN"


def test_mirrored_rotated_pad(tmp_path):
    board = tmp_path / "board.brd"
    board.write_text(
        """<eagle><drawing><board><libraries><library name="L"><packages><package name="P"><pad name="1" x="2" y="3"/></package></packages></library></libraries><elements><element name="J1" library="L" package="P" x="10" y="20" rot="MR90"/></elements><signals><signal name="GND"><contactref element="J1" pad="1"/></signal></signals></board></drawing></eagle>"""
    )
    pad = board_ports(board, ["J1"])["J1:1"]
    assert pad["pad_xy_mm"] == pytest.approx([7, 18])
    assert pad["board_signal"] == "GND"


@pytest.mark.parametrize(
    "value", [np.diag([2, 1, 1, 1]), np.diag([-1, 1, 1, 1]), np.full((4, 4), np.nan)]
)
def test_reject_invalid_transform(value):
    with pytest.raises(ValueError, match="rigid"):
        rigid_matrix(value)


@pytest.mark.parametrize("bus,expected", [("CAN", 29), ("RS485", 28)])
def test_candidates_keep_selection_pending(bus, expected, catalog):
    plan = scenario(bus, ["motor1", "motor2", "motor3"], "board", "regulator", catalog)
    ctx = context(["board", "regulator", "motor1", "motor2", "motor3"])
    result = compile_connections(plan, template(plan, ctx, catalog), ctx)
    assert not plan["review"]["errors"]
    assert not result["can_route"] and result["required_cables"] == expected
    assert result["resolved_cables"] == 0
    assert any(
        p["reason"] == "hardware_scenario_not_selected" for p in result["pending"]
    )
    assert all("SIG" not in c["ports"] for c in plan["components"])
    assert "PW_BUS_INTERFACE" in {c["id"] for c in plan["components"]}


def test_reject_legacy_motor_voltage(catalog):
    plan = scenario("CAN", ["motor"], "board", "regulator", catalog)
    ctx = context(["board", "regulator", "motor"])
    reg = template(plan, ctx, catalog)
    reg["motor_bus_voltage_range_v"] = [5, 5]
    result = compile_connections(plan, reg, ctx)
    assert {"motor_supply_out_of_range", "regulator_supply_out_of_range"} <= {
        e["reason"] for e in result["errors"]
    }


def test_board_registration_cannot_move_signal_to_another_pin(catalog):
    plan = scenario("CAN", ["motor"], "board", "regulator", catalog)
    ctx = context(["board", "regulator", "motor"])
    reg = template(plan, ctx, catalog)
    reg["components"]["board"]["ports"]["POWER:8"]["physical_pin"] = "POWER:5"
    assert any(
        e["reason"] == "official_board_pin_mismatch"
        for e in compile_connections(plan, reg, ctx)["errors"]
    )


def test_continuity_detects_bus_short_through_loom(catalog):
    plan = scenario("CAN", ["motor"], "board", "regulator", catalog)
    loom = next(c for c in plan["components"] if c["id"] == "PW_BUS_LOOM")
    loom["conductive_groups"].append(["HOST:CAN_H", "HOST:CAN_L"])
    assert any(
        e["reason"] == "conductive_path_crosses_bus_polarities"
        for e in review(plan)["errors"]
    )


def test_registered_bundle_transform_and_router_contract():
    plan, reg, ctx = synthetic()
    result = compile_connections(plan, reg, ctx)
    assert result["can_route"] and result["resolved_cables"] == 1
    connections = validate_connections(result["connections"])
    cable = connections["cables"][0]
    assert cable["source"]["position_mm"] == pytest.approx([8, 21, 33])
    assert cable["source"]["normal"] == pytest.approx([0, 1, 0])
    assert len(cable["conductors"]) == 2


def test_bundle_must_share_real_exit():
    plan, reg, ctx = synthetic()
    reg["components"]["A"]["ports"]["L"]["position_mm"] = [9, 9, 9]
    result = compile_connections(plan, reg, ctx)
    assert not result["can_route"]
    assert any(
        p["reason"] == "bundle_conductors_do_not_share_registered_exit"
        for p in result["pending"]
    )


def test_physical_pin_cannot_be_reused():
    plan, reg, ctx = synthetic()
    reg["components"]["A"]["ports"]["L"]["physical_pin"] = "H"
    assert any(
        e["reason"] == "physical_pin_reused"
        for e in compile_connections(plan, reg, ctx)["errors"]
    )


def test_unknown_component_and_bad_normal_rejected():
    plan, reg, ctx = synthetic()
    bad = copy.deepcopy(reg)
    bad["components"]["invented"] = {}
    with pytest.raises(ValueError, match="unknown component"):
        compile_connections(plan, bad, ctx)
    reg["components"]["A"]["ports"]["H"]["normal"] = [0, 0, 0]
    with pytest.raises(ValueError, match="unit normals"):
        compile_connections(plan, reg, ctx)


def test_contract_detects_changed_source_and_connection(tmp_path):
    plan, reg, ctx = synthetic()
    result = compile_connections(plan, reg, ctx)
    source = tmp_path / "assembly"
    source.mkdir()
    (source / "part").write_text("original")
    folder = tmp_path / "output" / "CAN"
    folder.mkdir(parents=True)

    def write(path, data):
        path.write_text(json.dumps(data))

    write(folder.parent / "source_snapshot.json", snapshot(source))
    write(folder / "connections.json", result["connections"])
    write(
        folder / "readiness.json",
        {k: v for k, v in result.items() if k != "connections"},
    )
    write(
        folder / "routing_contract.json",
        {
            "schema_version": 1,
            "pipeline": "physical_wiring_v1",
            "can_route": True,
            "source": str(source),
            "source_snapshot_sha256": sha256(folder.parent / "source_snapshot.json"),
            "readiness_sha256": sha256(folder / "readiness.json"),
            "connections_sha256": sha256(folder / "connections.json"),
        },
    )
    assert load_routing_contract(folder, source)[1] == 5
    (source / "part").write_text("changed")
    with pytest.raises(ValueError, match="Assembly changed"):
        load_routing_contract(folder, source)
    (source / "part").write_text("original")
    (folder / "connections.json").write_text("{}")
    with pytest.raises(ValueError, match="connections changed"):
        load_routing_contract(folder, source)
