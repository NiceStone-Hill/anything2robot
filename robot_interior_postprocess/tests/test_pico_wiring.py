"""Pico migration must change actual pin identities, rails and module requirements."""

import numpy as np
import pytest
from physical_wiring.catalog import build_catalog
from physical_wiring.topology import scenario


@pytest.mark.parametrize("bus", ["CAN", "RS485"])
def test_pico_topology_has_3v3_logic_protected_vsys_and_motor_coverage(bus):
    plan = scenario(
        bus,
        ["M_A", "M_B", "M_C"],
        "PICO",
        "REG",
        build_catalog(),
        controller_profile="raspberry_pi_pico2",
    )
    assert not plan["review"]["errors"]
    parts = {c["id"]: c for c in plan["components"]}
    host = parts["PICO"]
    assert host["profile_id"] == "raspberry_pi_pico2"
    assert host["ports"]["39"]["voltage_range_v"] == [1.8, 5.5]
    assert host["ports"]["36"]["voltage_range_v"] == [3.3, 3.3]
    for spec in host["ports"].values():
        if spec["kind"].startswith("signal"):
            assert spec["voltage_range_v"] == [0, 3.3]
    nets = {n["id"]: n for n in plan["nets"]}
    assert nets["controller_power"]["terminals"] == [
        {"instance_id": "PW_CONTROLLER_SUPPLY", "port_id": "OUT"},
        {"instance_id": "PICO", "port_id": "39"},
    ]
    assert nets["CTRL_REG_input"]["terminals"][0] == {
        "instance_id": "REG",
        "port_id": "VOUT",
    }
    assert nets["bus_interface_power"]["terminals"][0] == {
        "instance_id": "PICO",
        "port_id": "36",
    }
    assert not any(
        "POWER:" in t["port_id"] or "IOH:" in t["port_id"] or "IOL:" in t["port_id"]
        for n in plan["nets"]
        for t in n["terminals"]
        if t["instance_id"] == "PICO"
    )
    for motor in ("M_A", "M_B", "M_C"):
        cables = [c for c in plan["cables"] if c["target"]["instance_id"] == motor]
        assert len(cables) == 4  # power, return and two bus conductors
    if bus == "CAN":
        assert plan["controller_signal_assignment"]["MOSI"] == {
            "gpio": "GP19",
            "physical_pin": 25,
        }


def test_pico_display_does_not_scale_native_official_model():
    from pathlib import Path

    from carrier_layout.display_models import load_display
    from physical_wiring.catalog import read

    root = Path(__file__).resolve().parents[1]
    config = read(root / "configs/cactus_pico2_wiring.json")
    spec = next(c for c in config["components"] if c["id"] == "U1_1")
    sources = {}
    mesh = load_display(spec["display"], root / "configs", sources)
    assert np.allclose(np.sort(mesh.extents[:2]), [21, 52.3], atol=0.2)
    assert mesh.metadata["display_reference"]["scale"] == 1
    assert spec["envelope"]["size_mm"] == [60, 30, 20]
