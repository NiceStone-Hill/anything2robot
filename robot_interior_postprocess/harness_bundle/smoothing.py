"""Prefer larger physical fillets; keep endpoints and reject new collisions."""

import numpy as np
from hardware import as_mesh64
from harness_design.geometry import curve, wire_solid
from harness_design.pipeline import collision
from shell_channels.assembly import load_mesh
from shell_channels.geometry import check_self_clearance, solid


def refine(
    segments,
    cells,
    domain,
    clearance,
    origin,
    pitch,
    obstacles,
    config,
    out,
    tube_solids,
):
    changes = []
    preferred_factor = config.get("preferred_bend_radius_factor", 4.0)
    for segment in segments:
        radius = segment["diameter_mm"] / 2
        original = np.asarray(segment["points_mm"])
        minimum = segment["assumed_bend_radius_mm"]
        preferred = max(minimum, preferred_factor * radius)
        # Nonincident segments must remain separate; junction fan-out is still pending.
        peers = {
            other["id"]: tube_solids[other["id"]]
            for other in segments
            if other["id"] != segment["id"]
            and not (
                {segment["nodes"][0], segment["nodes"][-1]}
                & {other["nodes"][0], other["nodes"][-1]}
            )
        }
        free = domain & (clearance > radius + config["clearance_mm"] + pitch * 0.87)
        attempts = []
        accepted = None
        for bend in sorted(
            {preferred, (preferred + minimum) / 2, minimum}, reverse=True
        ):
            if bend <= minimum + 1e-6:
                continue
            try:
                points = curve(cells[segment["nodes"]], free, origin, pitch, bend)
                if not np.allclose(points[[0, -1]], original[[0, -1]]):
                    raise ValueError("endpoint changed")
                check_self_clearance(points, 2 * radius)
                guarded = wire_solid(points, radius + config["clearance_mm"])
                hit = collision(guarded, obstacles) or collision(guarded, peers)
                if hit:
                    raise ValueError("collision: " + hit)
                value = wire_solid(points, radius)
                # Validate the exported geometry before replacing this segment.
                temp = out / "meshes" / f"{segment['id']}_smooth_trial.stl"
                as_mesh64(value).export(temp, file_type="stl_ascii")
                reloaded = solid(load_mesh(temp))
                if collision(reloaded, obstacles) or collision(reloaded, peers):
                    raise ValueError("export collision")
                temp.replace(out / segment["file"])
                tube_solids[segment["id"]] = reloaded
                accepted = (points, bend)
                break
            except ValueError as error:
                attempts.append({"radius_mm": bend, "reason": str(error)[:160]})
                (out / "meshes" / f"{segment['id']}_smooth_trial.stl").unlink(
                    missing_ok=True
                )
        if accepted:
            points, bend = accepted
            # Straight segments need no fillet; do not count them as an improvement.
            direction = np.diff(points, axis=0)
            lengths = np.linalg.norm(direction, axis=1)
            direction = direction[lengths > 1e-7] / lengths[lengths > 1e-7, None]
            curved = len(direction) > 1 and np.any(
                np.sum(direction[:-1] * direction[1:], axis=1) < 0.99999
            )
            segment.update(
                points_mm=points.tolist(),
                length_mm=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
                assumed_bend_radius_mm=bend,
                local_bend_passed=True,
                assembly_collision=None,
                exported_solid_collision=None,
            )
            status = (
                "larger_collision_checked_fillets" if curved else "already_straight"
            )
        else:
            status = "kept_previous_checked_geometry"
        segment["smoothing"] = {
            "status": status,
            "previous_radius_mm": minimum,
            "selected_radius_mm": segment["assumed_bend_radius_mm"],
            "rejected_attempts": attempts,
        }
        changes.append({"segment_id": segment["id"], **segment["smoothing"]})
    return changes
