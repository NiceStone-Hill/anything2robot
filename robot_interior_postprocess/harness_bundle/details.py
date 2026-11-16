"""Semantic cable colours and explicit motor coverage, including unresolved tails."""

import colorsys

import numpy as np
from physical_wiring.registration import cable_groups

CATEGORIES = {
    "power": {"label": "供电", "color": "#ff786f"},
    "return": {"label": "回线 / 地", "color": "#70cfff"},
    "communication": {"label": "总线通信", "color": "#78e0a0"},
    "control": {"label": "控制 / 其他", "color": "#c49aff"},
}


def category(members):
    if any(m.get("protocol") for m in members):
        return "communication"
    pins = {
        m[side]["port_id"].upper() for m in members for side in ("source", "target")
    }
    if pins & {"V-", "GND", "GROUND", "RETURN", "VIN-", "OUT-"} or any(
        "GND" in p for p in pins
    ):
        return "return"
    if pins & {"V+", "VIN", "VOUT", "5V", "+5V", "12V", "VIN+", "OUT+"}:
        return "power"
    # The candidate circuit has named supply/fuse/distribution nets as well.
    names = " ".join(m["id"].lower() for m in members)
    if any(word in names for word in ("return", "ground")):
        return "return"
    if any(
        word in names for word in ("power", "supply", "fuse", "distribution", "_input")
    ):
        return "power"
    return "control"


def annotate_cables(cables):
    """Stable across input ordering; colours identify cables, never electrical joins."""
    for i, cable in enumerate(sorted(cables, key=lambda c: c["id"])):
        rgb = colorsys.hsv_to_rgb(
            (i * 0.61803398875 + 0.02) % 1, 0.55 + (i % 2) * 0.18, 0.98
        )
        cable["color"] = "#" + "".join(f"{round(v * 255):02x}" for v in rgb)
        cable["category"] = category(cable["conductors"])


def cylinder_reference(point, motor):
    """Nearest point on the model cylinder; explicitly NOT a connector location."""
    start = np.asarray(motor["axial_start_mm"], float)
    end = np.asarray(motor["axial_end_mm"], float)
    axis = end - start
    length = np.linalg.norm(axis)
    axis /= length
    relative = np.asarray(point) - start
    t = float(relative @ axis)
    radial = relative - t * axis
    r = np.linalg.norm(radial)
    radius = motor["radius_mm"]
    if r < 1e-9:
        direction = np.cross(axis, [1, 0, 0] if abs(axis[0]) < 0.8 else [0, 1, 0])
        direction /= np.linalg.norm(direction)
    else:
        direction = radial / r
    clipped_t = np.clip(t, 0, length)
    clipped_r = min(r, radius)
    if 0 <= t <= length and r < radius:
        if min(t, length - t) < radius - r:
            clipped_t = 0 if t < length / 2 else length
        else:
            clipped_r = radius
    return start + clipped_t * axis + clipped_r * direction


def motor_coverage(plan, cables, ports, mapping, mechanical_scene):
    expected = cable_groups(plan)
    delivered = {c["id"]: c for c in cables}
    motors = {m["id"]: m for m in mechanical_scene["motors"]}
    records = []
    for device, binding in sorted(mapping.items()):
        required = {
            key: members
            for key, members in expected.items()
            if any(
                m[side]["instance_id"] == device
                for m in members
                for side in ("source", "target")
            )
        }
        missing = [
            key
            for key in required
            if key not in delivered or not delivered[key]["segments"]
        ]
        motor = motors[binding["motor_id"]]
        exit_point = ports.get(device, {}).get("position_mm")
        reference = (
            cylinder_reference(exit_point, motor) if exit_point is not None else None
        )
        records.append(
            {
                "device": device,
                "motor_id": binding["motor_id"],
                "mechanical_envelope": motor,
                "expected_cables": sorted(required),
                "mapped_cables": sorted(set(required) & set(delivered)),
                "missing_cables": missing,
                "logical_coverage_complete": bool(required) and not missing,
                "circuits": [
                    {
                        "id": key,
                        "category": category(members),
                        "conductor_count": len(members),
                    }
                    for key, members in required.items()
                ],
                "aggregation_exit_mm": exit_point,
                "envelope_reference_mm": None
                if reference is None
                else reference.tolist(),
                "unresolved_gap_distance_mm": None
                if reference is None
                else float(np.linalg.norm(reference - exit_point)),
                "gap_status": "unrouted_reference_only_not_a_connector_or_cable_path",
                "real_connector_connection_complete": False,
            }
        )
    return {
        "motors": records,
        "motor_count": len(records),
        "expected_motor_cables": sum(len(m["expected_cables"]) for m in records),
        "mapped_motor_cables": sum(len(m["mapped_cables"]) for m in records),
        "logical_coverage_complete": bool(records)
        and all(m["logical_coverage_complete"] for m in records),
        "real_connector_connection_complete": False,
    }
