"""Resolve registered cable exits to world coordinates, retaining every missing item."""

from __future__ import annotations

from collections import defaultdict

import numpy as np
from electrical_review import voltage_range

from .catalog import rigid_matrix
from .checks import review


def cable_groups(plan):
    groups = defaultdict(list)
    for cable in plan["cables"]:
        groups[cable.get("bundle_id") or cable["id"]].append(cable)
    return dict(groups)


def template(plan, context, catalog):
    components = {}
    for component in plan["components"]:
        key = component["id"]
        profile = catalog["profiles"].get(component.get("profile_id"), {})
        pads = profile.get("physical_pads", {})
        components[key] = {
            "model": None,
            "revision": None,
            "profile_id": component.get("profile_id"),
            "selection_confirmed": False,
            "selection_evidence": None,
            "owner_link": context["owners"].get(key),
            "mounting_evidence": None,
            "world_from_interface": context["poses"].get(key),
            "ports": {
                name: {
                    "physical_pin": name if name in pads else None,
                    "connector_id": pads.get(name, {}).get("connector_reference"),
                    "cable_exit_id": None,
                    "coordinate_frame": "component",
                    "position_mm": None,
                    "normal": None,
                    "evidence": None,
                    "reference_pad_xy_mm": pads.get(name, {}).get("pad_xy_mm"),
                }
                for name in component["ports"]
            },
        }
    return {
        "schema_version": 1,
        "units": "mm",
        "scenario": plan["protocol"],
        "selection_confirmed": False,
        "motor_bus_voltage_range_v": None,
        "components": components,
        "cables": {
            key: {"diameter_mm": None, "min_bend_radius_mm": None, "evidence": None}
            for key in cable_groups(plan)
        },
    }


def compile_connections(plan, registration, context):
    if (
        registration.get("schema_version") != 1
        or registration.get("units") != "mm"
        or registration.get("scenario") != plan["protocol"]
    ):
        raise ValueError("Registration schema/units/scenario do not match the plan")
    if review(plan)["errors"]:
        raise ValueError(
            "Electrical topology has errors; no physical routing permitted"
        )
    expected = {c["id"]: c for c in plan["components"]}
    supplied = registration.get("components", {})
    if set(supplied) - set(expected):
        raise ValueError("Registration contains unknown component identities")
    if set(registration.get("cables", {})) - set(cable_groups(plan)):
        raise ValueError("Registration contains unknown cable identities")
    pending, errors, endpoints = [], [], {}

    def missing(reason, **data):
        pending.append({"reason": reason, **data})

    if registration.get("selection_confirmed") is not True:
        missing("hardware_scenario_not_selected")
    bounds = registration.get("motor_bus_voltage_range_v")
    if bounds is None:
        missing("motor_supply_voltage_range_missing")
    else:
        bounds = voltage_range(bounds)
        for component in plan["components"]:
            # Power consumers on this rail are reached through the declared
            # closed fuse/switch and distribution groups; no 5 V motor shortcut.
            if (component.get("profile_id") or "").startswith("mg4005v2_"):
                low, high = component["ports"]["V+"]["voltage_range_v"]
                if bounds[0] < low or bounds[1] > high:
                    errors.append(
                        {
                            "instance_id": component["id"],
                            "reason": "motor_supply_out_of_range",
                        }
                    )
            if component.get("profile_id") == "pololu_d24v50f5":
                low, high = component["ports"]["VIN"]["voltage_range_v"]
                if bounds[0] < low or bounds[1] > high:
                    errors.append(
                        {
                            "instance_id": component["id"],
                            "reason": "regulator_supply_out_of_range",
                        }
                    )
    for key, component in expected.items():
        record = supplied.get(key, {})
        if set(record.get("ports", {})) - set(component["ports"]):
            raise ValueError(f"Unknown registered port on {key}")
        if (
            record.get("selection_confirmed") is not True
            or not record.get("model")
            or not record.get("revision")
            or not record.get("selection_evidence")
        ):
            missing("component_model_revision_unconfirmed", instance_id=key)
        if (
            component.get("profile_id")
            and record.get("profile_id") != component["profile_id"]
        ):
            errors.append({"instance_id": key, "reason": "selected_profile_mismatch"})
        if key not in context["placed_instances"]:
            missing("component_not_in_assembly", instance_id=key)
        owner = record.get("owner_link")
        if not owner or not record.get("mounting_evidence"):
            missing("mounting_side_or_evidence_missing", instance_id=key)
        elif owner not in context["links"]:
            errors.append({"instance_id": key, "reason": "unknown_owner_link"})
        elif (
            key in context.get("motor_adjacent_links", {})
            and owner not in context["motor_adjacent_links"][key]
        ):
            errors.append(
                {"instance_id": key, "reason": "motor_owner_not_adjacent_to_joint"}
            )
        elif key in context["owners"] and owner != context["owners"][key]:
            errors.append(
                {"instance_id": key, "reason": "owner_disagrees_with_assembly"}
            )
        transform = record.get("world_from_interface")
        if transform is not None:
            transform = rigid_matrix(transform)
            existing = context["poses"].get(key)
            if existing is not None and not np.allclose(transform, existing, atol=1e-7):
                errors.append(
                    {
                        "instance_id": key,
                        "reason": "registration_pose_disagrees_with_assembly",
                    }
                )
        physical_pins = set()
        for name in component["ports"]:
            pin = record.get("ports", {}).get(name, {})
            identity = (pin.get("connector_id"), pin.get("physical_pin"))
            if (
                component.get("profile_id") == "arduino_uno_rev3e"
                and all(identity)
                and identity != (name.split(":")[0], name)
            ):
                errors.append(
                    {
                        "instance_id": key,
                        "port": name,
                        "reason": "official_board_pin_mismatch",
                    }
                )
            if all(identity):
                if identity in physical_pins:
                    errors.append(
                        {
                            "instance_id": key,
                            "port": name,
                            "reason": "physical_pin_reused",
                        }
                    )
                physical_pins.add(identity)
            required = ("physical_pin", "connector_id", "cable_exit_id", "evidence")
            if (
                not all(pin.get(k) for k in required)
                or pin.get("position_mm") is None
                or pin.get("normal") is None
            ):
                missing(
                    "physical_pin_or_cable_exit_missing", instance_id=key, port=name
                )
                continue
            if pin.get("coordinate_frame") not in {"component", "world"}:
                raise ValueError("Port coordinate_frame must be component or world")
            p, n = (
                np.asarray(pin["position_mm"], float),
                np.asarray(pin["normal"], float),
            )
            if (
                p.shape != (3,)
                or n.shape != (3,)
                or not np.isfinite([p, n]).all()
                or not np.isclose(np.linalg.norm(n), 1)
            ):
                raise ValueError("Ports require finite 3D positions and unit normals")
            if pin["coordinate_frame"] == "component":
                if transform is None:
                    missing(
                        "component_frame_registration_missing",
                        instance_id=key,
                        port=name,
                    )
                    continue
                p, n = (transform @ np.r_[p, 1])[:3], transform[:3, :3] @ n
            endpoints[key, name] = {
                "instance_id": key,
                "port_id": pin["cable_exit_id"],
                "owner_link": owner,
                "position_mm": p.tolist(),
                "normal": n.tolist(),
                "evidence": pin["evidence"],
                "connector_id": pin["connector_id"],
                "physical_pin": pin["physical_pin"],
            }
    cables, outcomes = [], []
    for key, members in cable_groups(plan).items():
        spec = registration.get("cables", {}).get(key, {})
        diameter, bend = spec.get("diameter_mm"), spec.get("min_bend_radius_mm")
        if diameter is None or bend is None or not spec.get("evidence"):
            missing("wire_or_bundle_spec_missing", cable_id=key)
            outcomes.append(
                {
                    "id": key,
                    "status": "pending",
                    "reason": "wire_or_bundle_spec_missing",
                }
            )
            continue
        if any(
            isinstance(x, bool)
            or not isinstance(x, (int, float))
            or not np.isfinite(x)
            or x <= 0
            for x in [diameter, bend]
        ):
            raise ValueError(
                "Cable diameter and bend radius must be finite positive values"
            )
        ends, failure = {}, None
        for side in ("source", "target"):
            terminal_keys = [
                (c[side]["instance_id"], c[side]["port_id"]) for c in members
            ]
            if any(t not in endpoints for t in terminal_keys):
                failure = "endpoint_registration_missing"
                break
            values = [endpoints[t] for t in terminal_keys]
            first = values[0]
            if any(
                v["instance_id"] != first["instance_id"]
                or v["port_id"] != first["port_id"]
                or v["connector_id"] != first["connector_id"]
                or v["owner_link"] != first["owner_link"]
                or not np.allclose(
                    v["position_mm"], first["position_mm"], atol=1e-7, rtol=0
                )
                or not np.allclose(v["normal"], first["normal"], atol=1e-7, rtol=0)
                for v in values[1:]
            ):
                failure = "bundle_conductors_do_not_share_registered_exit"
                break
            ends[side] = first
        if failure:
            missing(failure, cable_id=key)
            outcomes.append({"id": key, "status": "pending", "reason": failure})
            continue
        cables.append(
            {
                "id": key,
                "net": key,
                "diameter_mm": diameter,
                "min_bend_radius_mm": bend,
                "evidence": spec["evidence"],
                **ends,
                "conductors": members,
                "routing_scope": "Static bundle envelope only; electrical, thermal and motion validation remain pending",
            }
        )
        outcomes.append({"id": key, "status": "geometry_input_resolved"})
    # Distinct cables cannot silently leave the same connector exit. A physical
    # junction or an explicitly declared common bundle is required instead.
    exits = {}
    for cable in cables:
        for side in ("source", "target"):
            end = cable[side]
            key = end["instance_id"], end["port_id"]
            if key in exits:
                errors.append(
                    {
                        "reason": "cable_exit_reused_without_bundle",
                        "cables": [exits[key], cable["id"]],
                    }
                )
            exits[key] = cable["id"]
    ready = not errors and not pending and len(cables) == len(cable_groups(plan))
    return {
        "schema_version": 1,
        "units": "mm",
        "can_route": ready,
        "complete_electrical_design": False,
        "complete_assemblable": False,
        "required_cables": len(cable_groups(plan)),
        "resolved_cables": len(cables),
        "errors": errors,
        "pending": pending,
        "cable_outcomes": outcomes,
        "connections": {"schema_version": 1, "units": "mm", "cables": cables},
    }
