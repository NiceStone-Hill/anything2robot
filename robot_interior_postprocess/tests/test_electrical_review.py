"""Catch incompatible wiring independently of component and robot names."""

from copy import deepcopy

import pytest
from electrical_review import review


def example():
    return {
        "schema_version": 1,
        "components": [
            {
                "id": "source_17",
                "evidence": "synthetic test",
                "ports": {"out": {"kind": "power_out", "voltage_range_v": [5, 5]}},
            },
            {
                "id": "board_42",
                "evidence": "synthetic test",
                "ports": {"input": {"kind": "power_in", "voltage_range_v": [1.8, 5.5]}},
            },
        ],
        "nets": [
            {
                "id": "rail",
                "voltage_range_v": [5, 5],
                "terminals": [
                    {"instance_id": "source_17", "port_id": "out"},
                    {"instance_id": "board_42", "port_id": "input"},
                ],
            }
        ],
    }


def test_compatible_supply_passes_limited_review_without_claiming_release():
    result = review(example())
    assert result["status"] == "limited_checks_passed"
    assert not result["complete_electrical_design"]


def test_raw_battery_to_low_voltage_controller_is_rejected():
    plan = example()
    plan["components"][0]["ports"]["out"]["voltage_range_v"] = [9, 12]
    plan["nets"][0]["voltage_range_v"] = [9, 12]
    assert any(e["reason"] == "voltage_out_of_range" for e in review(plan)["errors"])


def test_pin_cannot_belong_to_two_nets():
    plan = example()
    net = deepcopy(plan["nets"][0])
    net["id"] = "second_rail"
    plan["nets"].append(net)
    assert any(e["reason"] == "terminal_reused" for e in review(plan)["errors"])


def test_protocol_and_ground_short_are_not_ignored():
    plan = example()
    plan["components"][1]["ports"]["input"] = {"kind": "ground", "protocol": "can"}
    reasons = {e["reason"] for e in review(plan)["errors"]}
    assert {"power_ground_short", "protocol_mismatch"} <= reasons


def test_missing_hardware_and_unwired_required_pins_remain_pending():
    plan = example()
    plan["components"][1]["pending"] = ["connector_measurements"]
    plan["components"][1]["ports"]["enable"] = {"kind": "signal_in"}
    result = review(plan)
    assert result["status"] == "pending"
    assert {p["reason"] for p in result["pending"]} >= {
        "connector_measurements",
        "unconnected_required_terminal",
    }


@pytest.mark.parametrize("value", [[5, 2], [False, 5], [0, float("nan")]])
def test_invalid_voltage_bounds_are_rejected(value):
    plan = example()
    plan["nets"][0]["voltage_range_v"] = value
    with pytest.raises(ValueError):
        review(plan)


def test_assembly_preflight_rejects_invalid_power_before_importing_geometry(tmp_path):
    import json

    from carrier_layout.pipeline import run_pipeline

    plan = example()
    plan["nets"][0]["voltage_range_v"] = [24, 24]
    (tmp_path / "plan.json").write_text(json.dumps(plan))
    config = {
        "schema_version": 1,
        "units": "mm",
        "electrical": {"review_plan": "plan.json"},
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config))
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="Electrical preflight failed"):
        run_pipeline(path, output, record_process=False)
    assert json.loads((output / "manifest.json").read_text())["status"] == "failed"
    assert json.loads((output / "electrical_review.json").read_text())["errors"]
