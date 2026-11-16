"""Audit exported wiring, then explicitly adapt simulator supply to battery power."""

import csv
from collections import Counter
from xml.etree import ElementTree as ET

from common import ROOT, dump
from voxel_tool.circuits.connectivity import load_circuit_topology
from voxel_tool.circuits.normalized import load_physical_topology


def import_circuit(config, out, scene):
    folder = ROOT / config["circuit_directory"]
    brds = list(folder.glob("*.brd"))
    schs = list(folder.glob("*.sch"))
    if len(brds) != 1 or len(schs) != 1:
        raise ValueError("Require exactly one Eagle BRD/SCH pair")
    topology = load_circuit_topology(brds[0], schs[0])
    schroot = ET.parse(schs[0]).getroot()
    board_refs = {e.name for e in topology.board.elements}
    symbolic = []
    for part in schroot.findall(".//schematic/parts/part"):
        if part.attrib["name"] in board_refs:
            continue
        library = next(
            x
            for x in schroot.findall(".//schematic/libraries/library")
            if x.attrib["name"] == part.attrib["library"]
        )
        deviceset = next(
            x
            for x in library.findall("devicesets/deviceset")
            if x.attrib["name"] == part.attrib["deviceset"]
        )
        device = next(
            x
            for x in deviceset.findall("devices/device")
            if x.attrib.get("name", "") == part.attrib.get("device", "")
        )
        if device.attrib.get("package"):
            raise ValueError(
                "Physical SCH part missing from board: " + part.attrib["name"]
            )
        symbolic.append(part.attrib["name"])
    sch_nets = {
        n.attrib["name"]: {
            (p.attrib["part"], p.attrib["pin"])
            for p in n.findall(".//pinref")
            if p.attrib["part"] not in symbolic
        }
        for n in schroot.findall(".//sheets/sheet/nets/net")
    }
    board_nets = {
        n.name: {(t.component, t.pin) for t in n.terminals} for n in topology.nets
    }
    if sch_nets != board_nets:
        raise ValueError("SCH/BRD logical net membership differs")
    with (folder / "bom.csv").open(encoding="utf-8-sig", newline="") as f:
        bom = list(csv.DictReader(f))
    names = []
    for row in bom:
        refs = [v.strip() for v in row["Name"].split(",")]
        if len(refs) != int(row["Quantity"]):
            raise ValueError("BOM ref/quantity mismatch")
        names.extend(refs)
    if Counter(names) != Counter(e.name for e in topology.board.elements):
        raise ValueError("BOM and board component references differ")
    motor_ids = {m["id"] for m in scene["motors"]}
    if set(config["motor_mapping"].values()) != motor_ids or len(
        config["motor_mapping"]
    ) != len(motor_ids):
        raise ValueError("Every mechanical motor needs one unique circuit mapping")
    schematic = [
        {
            "name": n.name,
            "terminals": [
                {"component": t.component, "pad": t.pad, "pin": t.pin}
                for t in n.terminals
            ],
        }
        for n in topology.nets
    ]
    pin_sets = {e.name: set() for e in topology.board.elements if e.name != "P1"}
    pin_sets.update({"BATTERY": {"+", "GND"}, "REGULATOR": {"VIN", "VOUT", "GND"}})
    adapted = []
    for net in topology.nets:
        terms = []
        for t in net.terminals:
            ref, pin = t.component, t.pin
            if ref == "P1":
                ref, pin = "REGULATOR", ("VOUT" if pin == "+" else "GND")
            pin_sets[ref].add(pin)
            entry = {"component": ref, "pin": pin}
            if entry not in terms:
                terms.append(entry)
        if any(t == {"component": "REGULATOR", "pin": "GND"} for t in terms):
            terms.append({"component": "BATTERY", "pin": "GND"})
        terms.sort(
            key=lambda t: (
                t["component"] not in ("REGULATOR", "U1", "S1"),
                t["component"],
            )
        )
        adapted.append({"name": net.name, "terminals": terms})
    pin_sets["U1"].add("VIN")
    adapted.append(
        {
            "name": "BATTERY_RAW",
            "terminals": [
                {"component": "BATTERY", "pin": "+"},
                {"component": "REGULATOR", "pin": "VIN"},
                {"component": "U1", "pin": "VIN"},
            ],
        }
    )
    data = {
        "schema_version": 1,
        "kind": "physical_connectivity",
        "components": [{"id": k, "pins": sorted(v)} for k, v in pin_sets.items()],
        "nets": adapted,
    }
    dump(out / "connectivity.json", data)
    audit = {
        "sch": str(schs[0]),
        "brd": str(brds[0]),
        "bom": bom,
        "source_nets": schematic,
        "symbolic_schematic_only_parts": symbolic,
        "checks": {
            "bom_matches_board": True,
            "schematic_net_names_match_board": True,
            "schematic_terminal_membership_matches_board": True,
        },
        "motor_mapping": config["motor_mapping"],
        "adaptations": [
            "P1 simulator supply is replaced by REGULATOR 5 V output; S1 remains the exported servo-power switch.",
            "BATTERY 9.6 V supplies REGULATOR VIN and Uno VIN; common ground added. Original USB power is not an installed module.",
            "Repeated Uno ground pads consolidated to one logical ground terminal; original pads retained in source audit.",
            "SERVO electrical pins are conceptual placeholders mapped to original fixed mechanical motors; original motor electrical compatibility NOT verified.",
        ],
        "unresolved": [
            "Actual motor supply/control protocol and pin locations",
            "Battery holder dimensions and physical part measurements",
            "Switch external access",
            "Wire gauge, current capacity and full harness topology",
        ],
    }
    dump(out / "circuit_audit.json", audit)
    return load_physical_topology(out / "connectivity.json"), data


def component_schema(config, connectivity, fixed, out, scene):
    specs = []
    byid = {c["id"]: c for c in config["components"]}
    byid.update(
        {
            p.component_id: {
                "id": p.component_id,
                "name": p.display_name,
                "size_mm": list(p.size_mm),
                "color": list(p.color),
                "accuracy": "Fixed mechanical envelope; electrical anchors provisional",
            }
            for p in fixed
        }
    )
    for node in connectivity["components"]:
        ref = node["id"]
        data = byid[ref]
        size = data["size_mm"]
        pins = node["pins"]
        interfaces = []
        # Distinct, explicitly provisional port anchors on the local +Y face.
        for i, pin in enumerate(pins):
            interfaces.append(
                {
                    "id": pin,
                    "kind": "electrical",
                    "position_mm": [
                        (i - (len(pins) - 1) / 2)
                        * min(2.54, size[0] / (len(pins) + 1)),
                        size[1] / 2,
                        0,
                    ],
                    "normal": [0, 1, 0],
                }
            )
        visualization = {"mesh": None, "proxy_size_mm": size}
        if ref in config["motor_mapping"]:
            import numpy as np
            import trimesh

            motor = next(
                m for m in scene["motors"] if m["id"] == config["motor_mapping"][ref]
            )
            center = np.asarray(motor["center_mm"])
            cylinder = trimesh.creation.cylinder(
                radius=motor["radius_mm"],
                sections=32,
                segment=np.vstack(
                    [
                        np.asarray(motor["axial_start_mm"]) - center,
                        np.asarray(motor["axial_end_mm"]) - center,
                    ]
                ),
            )
            relative = f"meshes/{ref}_motor_local_mm.stl"
            cylinder.export(out / relative)
            visualization["mesh"] = relative
        specs.append(
            {
                "id": ref,
                "display_name": data["name"],
                "quantity": 1,
                "body_color": data["color"],
                "accuracy": data["accuracy"],
                "visualization": visualization,
                "interfaces": interfaces,
            }
        )
    value = {
        "schema_version": 1,
        "units": "millimeter",
        "accuracy_note": "Explicit concept proxy anchors; no manufacturer pin-position certification",
        "components": specs,
    }
    dump(out / "components.json", value)
    dump(
        out / "circuit_mapping.json",
        {
            "instances": {c["id"]: c["id"] + "_1" for c in specs},
            "pin_interfaces": {
                n["id"]: {pin: pin for pin in n["pins"]}
                for n in connectivity["components"]
            },
        },
    )
    return value
