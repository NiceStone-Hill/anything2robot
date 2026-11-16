"""Expose the planned installed power source and its normalized net connections."""

import csv

from common import dump, read


def export_power_report(config, placement, routing, out):
    topology = read(out / "connectivity.json")
    placed = {p.component_id: p for p in placement.placements}
    nets = [
        {"name": n["name"], "terminals": n["terminals"]}
        for n in topology["nets"]
        if any(t["component"] in {"BATTERY", "REGULATOR"} for t in n["terminals"])
    ]
    raw = next((n for n in nets if n["name"] == "BATTERY_RAW"), {"terminals": []})
    terms = {(t["component"], t["pin"]) for t in raw["terminals"]}
    ground = next(
        (
            n
            for n in nets
            if any(t == {"component": "BATTERY", "pin": "GND"} for t in n["terminals"])
        ),
        {"terminals": []},
    )
    ground_terms = {(t["component"], t["pin"]) for t in ground["terminals"]}
    checks = {
        "battery_placed": "BATTERY" in placed,
        "regulator_placed": "REGULATOR" in placed,
        "battery_regulator_controller_share_ground": {
            ("BATTERY", "GND"),
            ("REGULATOR", "GND"),
            ("U1", "GND"),
        }
        <= ground_terms,
        "battery_supplies_regulator_and_controller": {
            ("BATTERY", "+"),
            ("REGULATOR", "VIN"),
            ("U1", "VIN"),
        }
        <= terms,
        "battery_raw_branches_routed": len(
            [r for r in routing.routes if r.net_name == "BATTERY_RAW"]
        )
        == 2,
    }
    dump(
        out / "power_report.json",
        {
            "checks": checks,
            "power_source_included": all(checks.values()),
            "power_nets": nets,
            "source_adaptation": "Original Tinkercad P1 bench supply is explicitly replaced by an installed BATTERY assembly and REGULATOR; no second bench-supply model is installed.",
            "nominal_concept": "8 AA NiMH cells in a holder (9.6 V nominal) -> regulator VIN and Uno VIN; regulator 5 V output -> S1 -> simulator servo power; common ground.",
            "hardware_status": "Layout topology only. Original mechanical motors are not the Tinkercad three-wire servos; their real supply, protocol, current capacity and compatible power system are not verified.",
            "battery_geometry": next(
                c for c in config["components"] if c["id"] == "BATTERY"
            ),
            "battery_center_mm": placed["BATTERY"].center_mm
            if "BATTERY" in placed
            else None,
            "regulator_center_mm": placed["REGULATOR"].center_mm
            if "REGULATOR" in placed
            else None,
        },
    )
    with (out / "installed_bom.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["Reference", "Quantity", "Description", "Representation"])
        for p in placement.placements:
            writer.writerow(
                [
                    p.component_id,
                    1,
                    p.display_name,
                    "fixed motor envelope; real electrical interface unresolved"
                    if p.component_id in config["motor_mapping"]
                    else "installed assembly planning envelope",
                ]
            )
    return checks
