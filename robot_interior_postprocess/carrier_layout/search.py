"""Bounded extended pattern search over complete carrier bundles.

Rectangular masks are conservative coarse checks. Exact CSG and swept-volume
checks remain mandatory after search; a low objective is never a certificate.
"""

from itertools import combinations

import numpy as np

from .profiles import pose_matrix, world_bounds


class Cavity:
    def __init__(self, space, link_index):
        self.origin = np.asarray(space["origin"])
        self.pitch = float(space["pitch"])
        self.free = space["usable"] & (space["part_ids"] == link_index)
        self.prefix = np.pad((~self.free).astype(np.int32), ((1, 0),) * 3)
        for axis in range(3):
            self.prefix = self.prefix.cumsum(axis, dtype=np.int32)

    def clear(self, bounds, clearance=0):
        lo = np.floor((bounds[0] - clearance - self.origin) / self.pitch).astype(int)
        hi = np.ceil((bounds[1] + clearance - self.origin) / self.pitch).astype(int) + 1
        if np.any(lo < 0) or np.any(hi > self.free.shape):
            return False
        count = 0
        for mask in range(8):
            index = tuple(hi[i] if mask & (1 << i) else lo[i] for i in range(3))
            count += (-1 if mask.bit_count() % 2 == 0 else 1) * int(self.prefix[index])
        return count == 0


def rect_overlap(a, b, axes, margin=0):
    delta = np.minimum(a[1, axes], b[1, axes]) - np.maximum(a[0, axes], b[0, axes])
    return bool(np.all(delta + margin > 1e-6))


def make_pose(profile, normal, roll, center):
    transform = pose_matrix(normal, roll, center, profile)
    return {
        "id": profile["spec"]["id"],
        "roll": roll,
        "center": np.asarray(center, float),
        "transform": transform,
        "collision_boxes": [
            world_bounds(b, transform) for b in profile["collision_boxes"]
        ],
        "bounds": world_bounds(profile["bounds"], transform),
    }


def objective(poses, normal, weights, restrictions=None):
    if not poses:
        return 0.0
    axis = int(np.argmax(abs(normal)))
    lateral = [i for i in range(3) if i != axis]
    # Mounting-plane origins, not body centres: differently thick modules can
    # share a carrier plane without forcing their top faces to line up.
    depths = np.array([p["transform"][:3, 3] @ normal for p in poses])
    coplanarity = float(np.ptp(depths))
    boxes = np.array([p["bounds"] for p in poses])
    span = boxes[:, 1].max(0) - boxes[:, 0].min(0)
    # Shorter rear connections are estimated by a separate hard anchoring test
    # in fabrication, rather than an ungrounded reward for being near origin.
    occlusion = 0.0
    for a, b in combinations(poses, 2):
        delta = np.maximum(
            0,
            np.minimum(a["bounds"][1, lateral], b["bounds"][1, lateral])
            - np.maximum(a["bounds"][0, lateral], b["bounds"][0, lateral]),
        )
        area = float(np.prod(delta))
        reference = min(
            float(np.prod(a["bounds"][1, lateral] - a["bounds"][0, lateral])),
            float(np.prod(b["bounds"][1, lateral] - b["bounds"][0, lateral])),
        )
        occlusion += area / max(reference, 1)
    return (
        (restrictions.soft_cost(poses, normal) if restrictions else 0.0)
        + weights["coplanarity"] * coplanarity
        + weights["compactness"] * float(span[lateral].sum())
        + weights.get("projection_overlap", 40.0) * occlusion
    )


def feasible(pose, others, cavity, normal, offset, clearance, seam, restrictions=None):
    if restrictions:
        if restrictions.violations(pose):
            return False
        clearance = restrictions.clearance(pose, others, clearance)
    if not all(cavity.clear(b, clearance) for b in pose["collision_boxes"]):
        return False
    axis = int(np.argmax(abs(normal)))
    front = pose["bounds"][1 if normal[axis] > 0 else 0, axis] * normal[axis]
    if front >= offset - seam:
        return False
    # Only final volume overlap is hard. Projected overlap is a soft preference;
    # exact geometry must subsequently prove an acyclic staged assembly order.
    return not any(
        rect_overlap(a, b, [0, 1, 2], clearance * 2)
        for q in others
        for a in pose["collision_boxes"]
        for b in q["collision_boxes"]
    )


def diverse_beam(states, normal, settings, restrictions=None):
    """Retain spatially different seeds; one good corner must not fill the beam."""
    width = settings["beam_width"]
    unique = {}
    for state in states:
        key = tuple(
            np.rint(np.concatenate([p["bounds"].ravel() for p in state]) * 100).astype(
                int
            )
        )
        unique.setdefault(key, state)
    rows = sorted(
        unique.values(),
        key=lambda s: objective(s, normal, settings["weights"], restrictions),
    )
    if len(rows) <= width:
        return rows
    features = np.array(
        [np.concatenate([p["bounds"].ravel() for p in state]) for state in rows]
    )
    scale = np.maximum(np.ptp(features, axis=0), 1)
    features /= scale
    costs = np.array(
        [objective(s, normal, settings["weights"], restrictions) for s in rows]
    )
    costs = (costs - costs.min()) / max(float(np.ptp(costs)), 1)
    chosen = [0]
    distance = np.linalg.norm(features - features[0], axis=1)
    while len(chosen) < width:
        score = distance - costs * 0.25
        score[chosen] = -np.inf
        index = int(np.argmax(score))
        chosen.append(index)
        distance = np.minimum(
            distance, np.linalg.norm(features - features[index], axis=1)
        )
    return [rows[i] for i in chosen]


def plan_link(
    profiles,
    space,
    link_index,
    normal,
    offset,
    settings,
    on_progress=None,
    *,
    restrictions=None,
):
    cavity = Cavity(space, link_index)
    normal = np.asarray(normal, float)
    axis = int(np.argmax(abs(normal)))
    free_indices = np.argwhere(cavity.free)
    if not len(free_indices):
        return None, {"reason": "no_cavity"}
    stride = max(1, round(settings["seed_pitch_mm"] / cavity.pitch))
    indices = free_indices[(free_indices % stride == 0).all(1)]
    centers = cavity.origin + indices * cavity.pitch
    clearance, seam = settings["clearance_mm"], settings["seam_keepout_mm"]
    candidates = {}
    # Place larger bundles first, independent of IDs and robot-specific labels.
    order = sorted(
        profiles, key=lambda p: -np.prod(np.sort(np.diff(p["bounds"], axis=0)[0])[-2:])
    )
    counts = {}
    for profile in order:
        rows = []
        for roll in profile["rolls"]:
            for center in centers:
                pose = make_pose(profile, normal, roll, center)
                if feasible(
                    pose, [], cavity, normal, offset, clearance, seam, restrictions
                ):
                    rows.append(pose)
        # Depth bands + spatial coverage avoid retaining only one corner.
        rows.sort(key=lambda p: (abs(p["center"][axis]), *p["center"]))
        if len(rows) > settings["max_seed_candidates"]:
            selected = np.linspace(
                0, len(rows) - 1, settings["max_seed_candidates"], dtype=int
            )
            rows = [rows[i] for i in selected]
        candidates[profile["spec"]["id"]] = rows
        counts[profile["spec"]["id"]] = len(rows)
        if not rows:
            return None, {
                "reason": "bundle_has_no_feasible_pose",
                "candidate_counts": counts,
            }
    states = [[]]
    for profile in order:
        expanded = []
        for state in states:
            for pose in candidates[profile["spec"]["id"]]:
                if feasible(
                    pose, state, cavity, normal, offset, clearance, seam, restrictions
                ):
                    expanded.append([*state, pose])
        states = diverse_beam(expanded, normal, settings, restrictions)
        if not states:
            return None, {
                "reason": "no_joint_bundle_packing",
                "failed_component": profile["spec"]["id"],
                "candidate_counts": counts,
            }
    current = states[0]
    initial = objective(current, normal, settings["weights"], restrictions)
    if on_progress:
        on_progress(
            "initial_seed",
            current,
            {
                "cost": initial,
                "candidate_counts": counts,
                "_seed_candidates": states[:3],
                "layout_restrictions_applied": restrictions is not None,
            },
        )
    trace, evaluations, rejected = [], 0, 0
    lookup = {p["spec"]["id"]: p for p in profiles}
    for step in settings["step_mm"]:
        changed = True
        while changed and evaluations < settings["max_evaluations"]:
            changed = False
            best, best_cost = (
                current,
                objective(current, normal, settings["weights"], restrictions),
            )
            for i, pose in enumerate(current):
                profile = lookup[pose["id"]]
                moves = [
                    (pose["roll"], pose["center"] + sign * step * e, "translate")
                    for e in np.eye(3)
                    for sign in (-1, 1)
                ]
                moves += [
                    (roll, pose["center"], "atomic_roll")
                    for roll in profile["rolls"]
                    if roll != pose["roll"]
                ]
                for roll, center, kind in moves:
                    if evaluations >= settings["max_evaluations"]:
                        break
                    evaluations += 1
                    proposed = make_pose(profile, normal, roll, center)
                    others = current[:i] + current[i + 1 :]
                    if not feasible(
                        proposed,
                        others,
                        cavity,
                        normal,
                        offset,
                        clearance,
                        seam,
                        restrictions,
                    ):
                        rejected += 1
                        continue
                    trial = current.copy()
                    trial[i] = proposed
                    cost = objective(trial, normal, settings["weights"], restrictions)
                    if cost < best_cost - 1e-7:
                        best, best_cost = trial, cost
                        move_record = {"id": pose["id"], "kind": kind, "step_mm": step}
            if best is not current:
                old = objective(current, normal, settings["weights"], restrictions)
                current = best
                trace.append({**move_record, "before": old, "after": best_cost})
                if on_progress:
                    on_progress(
                        "accepted_move",
                        current,
                        {
                            **trace[-1],
                            "evaluations": evaluations,
                            "center_mm": next(
                                p["center"]
                                for p in current
                                if p["id"] == move_record["id"]
                            ),
                        },
                    )
                changed = True
    return current, {
        "method": "bounded_extended_pattern_search_carrier_bundles",
        "candidate_counts": counts,
        "evaluations": evaluations,
        "coarse_rejections": rejected,
        "accepted_moves": trace,
        "initial_cost": initial,
        "final_cost": objective(current, normal, settings["weights"], restrictions),
        "objective": "carrier-plane depth spread + projected compactness",
        "orientation_policy": "enumerated cardinal normals and atomic in-plane rolls",
        "layout_restrictions_applied": restrictions is not None,
        "semantic_validation": restrictions.report(current) if restrictions else None,
        "projection_policy": "prefer one layer; overlapping projections require exact acyclic assembly dependencies",
        "wire_objective": "not part of this search; final routed feasibility is a mandatory separate check",
        "_alternate_poses": states,
    }


def projection_metrics(poses, normal):
    axis = int(np.argmax(abs(normal)))
    lateral = [i for i in range(3) if i != axis]
    return {
        "parallel_deviation_deg": 0.0,
        "carrier_plane_depth_spread_mm": float(
            np.ptp([p["transform"][:3, 3] @ normal for p in poses])
        )
        if poses
        else 0,
        "projected_overlap_pairs": [
            [a["id"], b["id"]]
            for a, b in combinations(poses, 2)
            if rect_overlap(a["bounds"], b["bounds"], lateral)
        ],
    }
