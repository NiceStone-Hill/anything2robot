"""Extract exact board pad identity; never equate a PCB pad with a cable exit."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from build_uno_interface import rotation

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text())


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rigid_matrix(value):
    matrix = np.asarray(value, float)
    if (
        matrix.shape != (4, 4)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-7)
        or not np.isclose(np.linalg.det(matrix[:3, :3]), 1)
    ):
        raise ValueError(
            "Registration must be a finite rigid right-handed millimetre transform"
        )
    return matrix


def board_ports(path, references):
    """Eagle element transforms + contactref net identity; separate physical pads."""
    board = ET.parse(path).getroot().find("./drawing/board")
    if board is None:
        raise ValueError("Expected Eagle XML board")
    packages = {
        (lib.get("name"), p.get("name")): p
        for lib in board.findall("./libraries/library")
        for p in lib.findall("./packages/package")
    }
    nets = {}
    for signal in board.findall("./signals/signal"):
        for ref in signal.findall("contactref"):
            key = ref.get("element"), ref.get("pad")
            if key in nets:
                raise ValueError("A physical pad belongs to more than one signal")
            nets[key] = signal.get("name")
    ports = {}
    found = set()
    for element in board.findall("./elements/element"):
        ref = element.get("name")
        if ref not in references:
            continue
        found.add(ref)
        matrix = rotation(element.get("rot"))
        origin = np.array([float(element.get("x")), float(element.get("y"))])
        package = packages[element.get("library"), element.get("package")]
        for pad in list(package.findall("pad")) + list(package.findall("smd")):
            key = f"{ref}:{pad.get('name')}"
            xy = matrix @ [float(pad.get("x")), float(pad.get("y"))] + origin
            signal = nets.get((ref, pad.get("name")))
            ports[key] = {
                "id": key,
                "connector_reference": ref,
                "package": element.get("package"),
                "pad_number": pad.get("name"),
                "board_signal": signal,
                "pad_xy_mm": xy.tolist(),
                "side": "bottom"
                if (element.get("rot") or "").startswith("M")
                else "top",
                "local_cable_exit_mm": None,
                "local_outward_normal": None,
                "geometry_status": "pcb_xy_known_connector_exit_pending",
                "evidence": {
                    "source_sha256": sha256(path),
                    "element": ref,
                    "pad": pad.get("name"),
                    "method": "Eagle package pad rotated/mirrored and translated by its element",
                },
            }
    if found != set(references):
        raise ValueError(
            f"Missing connector references: {sorted(set(references) - found)}"
        )
    return ports


def build_catalog():
    catalog = read(ROOT / "interface_catalog/candidate_profiles.json")
    board = ROOT / "hardware_catalog/sources/UNO-TH_Rev3e.brd"
    expected = read(ROOT / "hardware_catalog/arduino_uno_th_rev3e.json")["source"][
        "sha256"
    ]
    if sha256(board) != expected:
        raise ValueError(
            "Official Uno board source hash changed; review extraction before use"
        )
    pins = board_ports(board, ["POWER", "IOH", "IOL", "AD"])
    aliases = {
        "VIN": "POWER:8",
        "5V": "POWER:5",
        "GND_POWER_1": "POWER:6",
        "GND_POWER_2": "POWER:7",
        "GND_DIGITAL": "IOH:7",
        "MOSI": "IOH:4",
        "MISO": "IOH:5",
        "SCK": "IOH:6",
        "CS": "IOH:3",
        "D2": "IOL:3",
        "D3": "IOL:4",
        "D4": "IOL:5",
        "TX": "IOL:2",
        "RX": "IOL:1",
    }
    catalog["profiles"]["arduino_uno_rev3e"].update(
        physical_pads=pins,
        aliases=aliases,
        source_file=str(board),
        source_sha256=sha256(board),
    )
    for source in catalog["sources"].values():
        if source.get("local_file"):
            path = ROOT / "interface_catalog" / source["local_file"]
            if sha256(path) != source["sha256"]:
                raise ValueError("Interface evidence source hash changed")
    from .pico import catalog_profile

    catalog["profiles"]["raspberry_pi_pico2"] = catalog_profile()
    return catalog
