"""Explicit gravity demand reporting; unknown capacity never becomes a pass."""

import numpy as np


def load_report(profiles, poses, designs, gravity):
    lookup = {p["id"]: p for p in poses}
    records = {r["target_id"]: r for d in designs.values() for r in d["records"]}
    g = np.asarray(gravity["world_m_s2"], float)
    rows = []
    for profile in profiles:
        spec = profile["spec"]
        key = spec["id"]
        transform = lookup[key]["transform"]
        normal = transform[:3, 2]
        row = {
            "target_id": key,
            "owner_link": spec["owner_link"],
            "mount_normal_world": normal.tolist(),
            "mount_plane_angle_to_horizontal_deg": float(
                np.rad2deg(
                    np.arccos(np.clip(abs(normal @ (-g / np.linalg.norm(g))), 0, 1))
                )
            ),
            "status": "pending",
            "capacity_validated": False,
            "carrier_and_fastener_mass": "pending_material_and_actual_fastener_properties",
        }
        if spec["mass_kg"] is None or spec.get("centre_of_mass_local_mm") is None:
            row["reason"] = "missing_measured_component_mass_or_local_centre_of_mass"
        else:
            force = spec["mass_kg"] * g
            center = (transform @ np.r_[spec["centre_of_mass_local_mm"], 1])[:3]
            anchor = np.mean([r["anchor_mm"] for r in records[key]["roots"]], axis=0)
            row.update(
                component_gravity_force_n=force.tolist(),
                component_moment_about_anchor_centroid_nm=np.cross(
                    (center - anchor) / 1000, force
                ).tolist(),
                reason="demand_only_missing_material_fastener_capacity_and_carrier_mass",
            )
        rows.append(row)
    return {
        "status": "pending",
        "gravity": gravity,
        "components": rows,
        "scope": "Reference assembly pose, component gravity demand only. Not a stress, joint torque, whole-robot stability or dynamics certificate.",
    }
