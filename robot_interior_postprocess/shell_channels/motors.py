"""Resolve electrical instances to mechanical joints without guessing stator sides."""

from __future__ import annotations

import numpy as np

from .assembly import read, safe_path


def motor_terminals(source, routing, registration_path=None):
    directory = routing.get("artifacts_directory")
    if not directory:
        if registration_path is not None:
            raise ValueError(
                "Motor terminals need an explicit legacy electrical mapping; otherwise supply complete connections"
            )
        return {}, {"motors": [], "note": "No legacy electrical mapping"}
    audit_path = source / directory / "circuit_audit.json"
    mapping_path = source / directory / "circuit_mapping.json"
    if not audit_path.is_file() or not mapping_path.is_file():
        if registration_path is not None:
            raise ValueError(
                "Motor terminals need an explicit legacy electrical mapping"
            )
        return {}, {"motors": [], "note": "Missing explicit motor/instance mapping"}
    audit = read(safe_path(source, f"{directory}/circuit_audit.json"))
    mapping = read(safe_path(source, f"{directory}/circuit_mapping.json"))
    scene = read(source / "mechanical_scene.json")
    motors = {m["id"]: m for m in scene.get("motors", [])}
    registration = {"schema_version": 1, "units": "mm", "motors": []}
    if registration_path is not None:
        registration = read(registration_path)
    elif (source / "motor_terminals.json").is_file():
        registration = read(safe_path(source, "motor_terminals.json"))
    if (
        registration.get("schema_version") != 1
        or registration.get("units") != "mm"
        or not isinstance(registration.get("motors"), list)
    ):
        raise ValueError(
            "Motor terminals require schema_version=1, units=mm, motors list"
        )
    registered = {}
    for record in registration["motors"]:
        key = record["motor_id"]
        if key not in motors or key in registered:
            raise ValueError("Unknown or duplicate motor terminal registration")
        motor = motors[key]
        if record.get("body_owner_link") not in {
            motor["parent"],
            motor["link"],
        } or not record.get("evidence"):
            raise ValueError(
                "Motor body owner must be an explicit adjacent joint link with evidence"
            )
        if not isinstance(record.get("ports"), dict) or not record["ports"]:
            raise ValueError("Motor registration requires explicit ports")
        for port in record["ports"].values():
            p, n = (
                np.asarray(port["position_mm"], float),
                np.asarray(port["normal"], float),
            )
            if (
                p.shape != (3,)
                or n.shape != (3,)
                or not np.isfinite([p, n]).all()
                or not np.isclose(np.linalg.norm(n), 1)
                or not port.get("evidence")
            ):
                raise ValueError(
                    "Motor ports require finite world positions, unit normals and evidence"
                )
        registered[key] = record
    if set(registered) - set(audit.get("motor_mapping", {}).values()):
        raise ValueError("Registered motor has no electrical instance mapping")
    bindings, report = {}, []
    for component, motor_id in audit.get("motor_mapping", {}).items():
        if motor_id not in motors or component not in mapping.get("instances", {}):
            raise ValueError(
                "Electrical motor mapping does not resolve to a mechanical motor and instance"
            )
        instance = mapping["instances"][component]
        if instance in bindings:
            raise ValueError("Duplicate motor instance mapping")
        motor = motors[motor_id]
        record = registered.get(motor_id)
        row = {
            "instance_id": instance,
            "motor_id": motor_id,
            "joint_parent": motor["parent"],
            "joint_child": motor["link"],
            "mapping_evidence": f"{directory}/circuit_audit.json + circuit_mapping.json + mechanical_scene.json",
            "status": "registered"
            if record
            else "body_side_and_physical_terminals_pending",
            "registration": record,
            "electrical_accuracy": motor.get("electrical_accuracy", "unverified"),
        }
        bindings[instance] = row
        report.append(row)
    return bindings, {
        "motors": report,
        "scope": "Electrical identity and adjacent joint links are known; parent/child relation alone does not establish motor-body or connector attachment",
    }


def resolve_endpoint(endpoint, bindings):
    binding = bindings.get(endpoint["instance_id"])
    if not binding:
        return endpoint
    endpoint = {
        **endpoint,
        "motor_binding": {k: v for k, v in binding.items() if k != "registration"},
    }
    registration = binding["registration"]
    port = registration and registration["ports"].get(endpoint["port_id"])
    if port:
        endpoint.update(
            owner_link=registration["body_owner_link"],
            owner_evidence=registration["evidence"],
            position_mm=port["position_mm"],
            normal=port["normal"],
            evidence=port["evidence"],
            terminal_status="registered",
            legacy_planning_endpoint={
                k: endpoint[k] for k in ("position_mm", "normal", "evidence")
            },
        )
    else:
        endpoint.update(
            owner_link=None, terminal_status="motor_terminal_registration_required"
        )
    return endpoint
