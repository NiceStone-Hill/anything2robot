"""Build Uno mounting data from the pinned official Eagle board.

XY footprints and hole centres are source data. Heights and solder clearance
are explicitly conservative planning assumptions, not measured component CAD.
"""

import argparse
import hashlib
import json
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from hardware import eagle_board_geometry


def rotation(text):
    match = re.fullmatch(r"(M)?R(-?[\d.]+)", text or "R0")
    if not match:
        raise ValueError(f"Unsupported Eagle rotation: {text}")
    angle = np.deg2rad(float(match[2]))
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]]) @ np.diag([-1 if match[1] else 1, 1])


def pad_bounds(pad):
    center = np.array([float(pad.get("x")), float(pad.get("y"))])
    if pad.tag == "pad":
        radius = max(float(pad.get("diameter", 0)), float(pad.get("drill")) * 2) / 2
        size = np.array([radius, radius])
        if pad.get("shape") in {"long", "offset"}:
            size[0] *= 2
    else:
        size = np.array([float(pad.get("dx")), float(pad.get("dy"))]) / 2
    corners = np.array([[sx, sy] for sx in [-1, 1] for sy in [-1, 1]]) * size
    return corners @ rotation(pad.get("rot")).T + center


def package_outline(package, include_pads=False):
    points = []
    for element in package:
        if element.tag in {"pad", "smd"}:
            if include_pads:
                points.extend(pad_bounds(element))
        elif element.get("layer") in {"21", "22", "51", "52"}:
            if element.tag in {"wire", "rectangle"}:
                a = np.array([float(element.get("x1")), float(element.get("y1"))])
                b = np.array([float(element.get("x2")), float(element.get("y2"))])
                if element.tag == "rectangle":
                    points.extend(
                        [[a[0], a[1]], [a[0], b[1]], [b[0], a[1]], [b[0], b[1]]]
                    )
                else:
                    width = float(element.get("width", 0)) / 2
                    points.extend([a - width, a + width, b - width, b + width])
                    angle = np.deg2rad(float(element.get("curve", 0)))
                    if abs(angle) > 1e-8:
                        chord = b - a
                        center = (a + b) / 2 + np.array([-chord[1], chord[0]]) / (
                            2 * np.tan(angle / 2)
                        )
                        radius = np.linalg.norm(a - center) + width
                        points.extend([center - radius, center + radius])
            elif element.tag == "circle":
                p = np.array([float(element.get("x")), float(element.get("y"))])
                radius = (
                    float(element.get("radius")) + float(element.get("width", 0)) / 2
                )
                points.extend([p - radius, p + radius])
    return np.asarray(points)


def build(path):
    path = Path(path)
    geometry = eagle_board_geometry(path)
    board = ET.parse(path).getroot().find("./drawing/board")
    packages = {
        (library.get("name"), package.get("name")): package
        for library in board.findall("./libraries/library")
        for package in library.findall("./packages/package")
    }
    keepouts, footprint_records = [], []
    for element in board.findall("./elements/element"):
        name, package_name = element.get("name"), element.get("package")
        package = packages[(element.get("library"), package_name)]
        # Testpoints/fiducials/jumpers are copper markings, not tall components.
        if package_name in {"TP-SP", "FIDUCIA-MOUNT", "SJ", "FRAME"}:
            continue
        points = package_outline(package)
        if not len(points):
            points = package_outline(package, include_pads=True)
        if not len(points):
            raise ValueError(f"Physical package has no supported bounds: {name}")
        matrix = rotation(element.get("rot"))
        offset = np.array([float(element.get("x")), float(element.get("y"))])
        points = points @ matrix.T + offset
        lower, upper = points.min(0) - 0.2, points.max(0) + 0.2
        bottom = bool((element.get("rot") or "").startswith("M"))
        z0, z1 = (-4.0, 0.0) if bottom else (1.6, 20.0)
        keepouts.append({"id": name, "min_mm": [*lower, z0], "max_mm": [*upper, z1]})
        footprint_records.append(
            {
                "reference": name,
                "package": package_name,
                "side": "bottom" if bottom else "top",
                "xy_bounds_mm": [lower.tolist(), upper.tolist()],
            }
        )
        for pad in package.findall("pad"):
            # Individual lead/solder envelopes retain the empty area between pins.
            corners = pad_bounds(pad) @ matrix.T + offset
            a, b = corners.min(0) - 0.2, corners.max(0) + 0.2
            keepouts.append(
                {
                    "id": f"{name}:lead:{pad.get('name')}",
                    "min_mm": [*a, -4.0],
                    "max_mm": [*b, 0.0],
                }
            )
        for pad in list(package.findall("pad")) + list(package.findall("smd")):
            corners = pad_bounds(pad) @ matrix.T + offset
            a, b = corners.min(0) - 0.2, corners.max(0) + 0.2
            za, zb = (-0.6, 0.0) if bottom else (1.6, 2.2)
            keepouts.append(
                {
                    "id": f"{name}:solder:{pad.get('name')}",
                    "min_mm": [*a, za],
                    "max_mm": [*b, zb],
                }
            )
    return {
        "schema_version": 1,
        "units": "mm",
        "id": "arduino_uno_th_rev3e_footprints",
        "manufacturer": "Arduino",
        "model": "Uno R3 TH",
        "revision": "Rev3e official PCB",
        "source": {
            "url": "https://docs.arduino.cc/hardware/uno-rev3/",
            "evidence": "official_cad",
            "file": "sources/UNO-TH_Rev3e.brd",
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "notes": "Official board outline, holes, footprint XY and pad locations. Component height cap z=20 mm, bottom lead reach 4 mm, XY envelope margin 0.2 mm are conservative unmeasured planning assumptions. No community-CAD hole coordinates are substituted.",
            "footprints": footprint_records,
        },
        "board": {"outline_mm": geometry["outline_mm"], "thickness_mm": 1.6},
        "holes": geometry["holes"],
        "keepouts": keepouts,
        "display_geometry": "pcb_only",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parent / "hardware_catalog"
    parser.add_argument(
        "--source", type=Path, default=root / "sources/UNO-TH_Rev3e.brd"
    )
    parser.add_argument(
        "--output", type=Path, default=root / "arduino_uno_th_rev3e_footprints.json"
    )
    args = parser.parse_args()
    args.output.write_text(json.dumps(build(args.source), indent=2) + "\n")
