"""Local carrier bundles and explicitly sourced hardware interfaces."""

from copy import deepcopy

import manifold3d as md
import numpy as np
import trimesh
from hardware import as_mesh, cylinder_at, load_interface, number, transform_solid
from shell_partition.mounting import bore, box, screw
from shell_partition.supports import connected_material, intersection_volume, union


def vector(value, name, unit=False):
    v = np.asarray(value, float)
    if v.shape != (3,) or not np.isfinite(v).all() or np.linalg.norm(v) < 1e-9:
        raise ValueError(f"{name} must be a finite nonzero 3-vector")
    if unit and not np.isclose(np.linalg.norm(v), 1):
        raise ValueError(f"{name} must be a unit vector")
    return v


def safe_id(value):
    if (
        not isinstance(value, str)
        or not value
        or not all(c.isalnum() or c in "_-" for c in value)
    ):
        raise ValueError("IDs must contain only letters, digits, '_' or '-'")
    return value


def build_profile(spec, base, sources):
    """A carrier is generated in hardware coordinates, before choosing a shell.

    Local +Z is its mounting/tool normal, NOT an assumed gravity direction.
    Unknown physical retention is kept in the dossier even when a tray is drawn.
    """
    required = {"id", "owner_link", "component_id", "carrier", "mass_kg"}
    allowed = required | {
        "mounting",
        "envelope",
        "allowed_rolls_deg",
        "ports",
        "centre_of_mass_local_mm",
        "display",
    }
    if not required <= spec.keys() or spec.keys() - allowed:
        raise ValueError("Incomplete/unknown carrier profile fields")
    for key in ("id", "owner_link", "component_id"):
        safe_id(spec[key])
    if ("mounting" in spec) == ("envelope" in spec):
        raise ValueError("Choose exactly one explicit mounting interface or envelope")
    mass = spec["mass_kg"]
    if mass is not None:
        number(mass, "mass_kg")
    if spec.get("centre_of_mass_local_mm") is not None:
        center = np.asarray(spec["centre_of_mass_local_mm"], float)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError("Centre of mass must be an explicit finite local point")
    settings = spec["carrier"]
    keys = {"plate_mm", "margin_mm", "ear_width_mm", "dock_depth_mm", "max_anchor_mm"}
    if set(settings) != keys:
        raise ValueError(
            "Carrier requires explicit plate/margin/ear/dock/anchor parameters"
        )
    settings = {k: number(v, k) for k, v in settings.items()}
    if settings["max_anchor_mm"] > 300 or settings["dock_depth_mm"] < 5:
        raise ValueError("Unsupported carrier docking dimensions")
    pending = []
    interface = None
    if "mounting" in spec:
        binding = deepcopy(spec["mounting"])
        if "world_from_interface" in binding:
            raise ValueError(
                "Carrier profiles contain local interfaces only; remove the saved world_from_interface pose"
            )
        # World pose is chosen later. Do not inherit a saved cactus placement.
        binding["world_from_interface"] = np.eye(4).tolist()
        interface = load_interface(binding, base, {"id": spec["id"]}, sources)
        body = interface["local_body"]
        lo, hi = interface["outline"].min(0), interface["outline"].max(0)
        p = interface["fastener"]
        height = p["standoff_height_mm"]
        bosses = union(
            [
                cylinder_at(h["center_mm"], p["boss_diameter_mm"] / 2, -height - 0.1, 0)
                for h in interface["holes"]
            ]
        )
        cuts = union([bore(h["center_mm"], p) for h in interface["holes"]])
        board_screws = union(
            [
                screw(h["center_mm"], p, interface["thickness"])
                for h in interface["holes"]
            ]
        )
        if intersection_volume(bosses - cuts, body) > 0.01:
            raise ValueError(f"{spec['id']}: PCB standoff hits declared hardware")
        if intersection_volume(board_screws, body) > 0.01:
            raise ValueError(f"{spec['id']}: PCB screw hits declared hardware")
        # PCB screws are operated on the bench, before shell installation.
        tools = union(
            [
                cylinder_at(
                    h["center_mm"],
                    p["tool_radius_mm"],
                    interface["thickness"] + p["head_height_mm"],
                    100,
                )
                for h in interface["holes"]
            ]
        )
        if intersection_volume(tools, body) > 0.01:
            raise ValueError(f"{spec['id']}: PCB bench tool access blocked")
        pending.extend(
            [
                "hardware_revision_and_registration_confirmation",
                "pcb_fastener_process_calibration",
            ]
        )
        display = interface["display_mesh"]
    else:
        envelope = spec["envelope"]
        if set(envelope) != {"size_mm", "evidence", "retention"}:
            raise ValueError("Envelope requires size, evidence and retention status")
        size = np.array([number(v, "envelope size") for v in envelope["size_mm"]])
        if size.shape != (3,):
            raise ValueError("Envelope size must have three dimensions")
        lo, hi = -size[:2] / 2, size[:2] / 2
        body = box([*lo, 0], [*hi, size[2]])
        display = as_mesh(body)
        height = 0.0
        bosses = md.Manifold()
        cuts = md.Manifold()
        board_screws = md.Manifold()
        pending.extend(["measured_hardware_interface", "component_retention"])
    if mass is None:
        pending.append("component_mass_and_centre_of_mass")
    plate = settings["plate_mm"]
    z = -height
    floor_lo, floor_hi = lo - settings["margin_mm"], hi + settings["margin_mm"]
    carrier = box([*floor_lo, z - plate], [*floor_hi, z]) + bosses
    if interface is None:
        # Geometric tray only. It is NOT a substitute for measured retention.
        for axis in (0, 1):
            for sign in (-1, 1):
                a, b = np.r_[floor_lo, z - 0.1], np.r_[floor_hi, 3.0]
                if sign < 0:
                    b[axis] = lo[axis] - 0.5
                else:
                    a[axis] = hi[axis] + 0.5
                carrier += box(a, b)
    # Ears are deliberately outside the complete hardware envelope, not just PCB.
    body_bounds = as_mesh(body).bounds
    width = settings["ear_width_mm"]
    if width < 10:
        raise ValueError(
            "Carrier ears need >=10 mm width for the declared tool profile"
        )
    ear_x = [
        min(floor_lo[0], body_bounds[0, 0]) - width / 2,
        max(floor_hi[0], body_bounds[1, 0]) + width / 2,
    ]
    ear_y = float((lo[1] + hi[1]) / 2)
    ears = [[float(x), ear_y] for x in ear_x]
    # Join both ears to the floor with finite overlap.
    for x, y in ears:
        carrier += box(
            [min(x - width / 2, floor_lo[0]), y - width / 2, z - plate],
            [max(x + width / 2, floor_hi[0]), y + width / 2, z],
        )
    # The carrier-shell joint uses M3 clearance + a blind M3 insert in each dock.
    # These dimensions are a documented prototype profile, not measured fit.
    ear_bores = union([cylinder_at(p, 1.7, z - plate - 0.1, z + 0.1) for p in ears])
    carrier = carrier - cuts - ear_bores
    if not connected_material(carrier):
        raise ValueError("Carrier must be one connected solid after all bores")
    if intersection_volume(carrier, body) > 0.01:
        raise ValueError("Carrier intersects hardware/keepouts")
    screws = union([cylinder_at(p, 2.9, z, z + 3) for p in ears])
    if intersection_volume(screws, body) > 0.01:
        raise ValueError("Carrier screw head intersects hardware")
    pending.extend(["carrier_fastener_process_calibration", "structural_capacity"])
    rolls = spec.get("allowed_rolls_deg", [0, 90, 180, 270])
    if not rolls or any(
        isinstance(r, bool) or r not in (0, 90, 180, 270) for r in rolls
    ):
        raise ValueError(
            "Only explicitly enumerated cardinal in-plane rolls are supported"
        )
    ports = spec.get("ports", {})
    if "display" in spec:
        from .display_models import load_display

        display = load_display(spec["display"], base, sources)
    if not isinstance(ports, dict):
        raise TypeError("Ports must be keyed by explicit pin/interface ID")
    for port_id, port in ports.items():
        if not isinstance(port_id, str) or not port_id.strip():
            raise ValueError("Port ID must be a nonempty string")
        if set(port) != {"position_mm", "normal", "evidence"} or not port["evidence"]:
            raise ValueError("Ports require local position, normal and evidence")
        point = np.asarray(port["position_mm"], float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("Invalid explicit port position")
        vector(port["normal"], "port normal", unit=True)
    return {
        "spec": spec,
        "settings": settings,
        "body": body,
        "display": display,
        "carrier": carrier,
        "board_screws": board_screws,
        "ears": ears,
        "ear_z": z,
        "rolls": tuple(rolls),
        "interface": interface,
        "bounds": as_mesh(union([body, carrier, board_screws, screws])).bounds,
        # Keep thin ears/plate separate from the tall hardware volume. A single
        # full AABB would falsely require empty space above every mounting ear.
        "collision_boxes": [
            as_mesh(v).bounds
            for v in (body, carrier, board_screws, screws)
            if not v.is_empty()
        ],
        "pending": sorted(set(pending)),
    }


def pose_matrix(normal, roll, center, profile):
    """One atomic transform for hardware, holes, ports, carrier and tool axes."""
    rotation = trimesh.geometry.align_vectors([0, 0, 1], normal)[:3, :3]
    angle = np.deg2rad(roll)
    rotation = rotation @ np.array(
        [
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    rotation = np.rint(rotation).astype(float)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(center) - rotation @ profile["bounds"].mean(0)
    return transform


def world_bounds(bounds, transform):
    center = transform[:3, :3] @ bounds.mean(0) + transform[:3, 3]
    extent = abs(transform[:3, :3]) @ (bounds[1] - bounds[0]) / 2
    return np.array([center - extent, center + extent])


def posed(value, transform):
    return transform_solid(value, transform)
