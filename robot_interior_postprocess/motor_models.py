"""Assign hardware identities before layout, without claiming mechanical fit."""

MODELS = {
    name: {"model_id": name, "manufacturer": maker}
    for name, maker in [
        ("MG4005V2", "K-Tech"),
        ("DM6006", "DAMIAO"),
        ("DM8006", "DAMIAO"),
        ("DM10010", "DAMIAO"),
    ]
}


def assign_motor_models(motors, config):
    """Match exact motor IDs; upstream size alone is not a model identity."""
    catalog = {**MODELS, **config.get("motor_model_catalog", {})}
    mapping = config.get("motor_model_mapping", {})
    ids = [motor["id"] for motor in motors]
    if len(set(ids)) != len(ids):
        raise ValueError("Motor IDs must be unique before model assignment")
    unknown = set(mapping) - set(ids)
    if unknown:
        raise ValueError(f"Unknown motor IDs in motor_model_mapping: {sorted(unknown)}")
    assignments = []
    for motor in motors:
        model_id = mapping.get(motor["id"])
        if model_id is not None and model_id not in catalog:
            raise ValueError(f"Unknown motor model: {model_id}")
        assignment = {
            "motor_id": motor["id"],
            "model_id": model_id,
            "model": dict(catalog[model_id]) if model_id is not None else None,
            "assignment_source": "explicit_config" if model_id is not None else "unassigned",
            "geometry_status": "original_cylinder_retained",
            "compatibility_status": "unverified",
        }
        motor["hardware_assignment"] = assignment
        assignments.append(assignment)
    return {"schema_version": 1, "assignments": assignments}
