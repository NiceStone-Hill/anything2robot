"""Jointly plan corresponding fasteners for approximately similar cut outlines.

Registration is in the two-dimensional split frame, never from robot/link names.
Only the candidate coordinates are normalized; every resulting joint is checked
against its own unscaled shell, hardware, routes, screw and driver sweeps.
"""

import numpy as np
from scipy.spatial import cKDTree

from .closure import (
    ClosureSearchError,
    farthest_order,
    finish_pair,
    target_count,
    try_candidate,
)

DEFAULT_POLICY = {
    "enabled": True,
    "max_normalized_rms": 0.07,
    "max_normalized_p95": 0.20,
    "max_size_ratio": 1.25,
}


def policy_for(raw=None):
    raw = raw or {}
    if not isinstance(raw, dict) or raw.keys() - DEFAULT_POLICY.keys():
        raise ValueError("Unknown shared shell pattern policy")
    p = {**DEFAULT_POLICY, **raw}
    if not isinstance(p["enabled"], bool):
        raise TypeError("Pattern enabled must be boolean")
    for key in p.keys() - {"enabled"}:
        if isinstance(p[key], bool) or not np.isfinite(p[key]) or p[key] <= 0:
            raise ValueError("Positive finite pattern tolerance required")
    if (
        p["max_size_ratio"] < 1
        or p["max_normalized_rms"] > 0.15
        or p["max_normalized_p95"] > 0.3
    ):
        raise ValueError("Pattern similarity tolerance is too permissive")
    return p


def outline_descriptor(context):
    polygons = context["outer"].slice(0).to_polygons()
    vertices = np.concatenate(polygons)
    center = (vertices.min(0) + vertices.max(0)) / 2
    size = np.ptp(vertices, axis=0)
    if np.any(size <= 1e-6):
        raise ValueError("Degenerate split outline")
    samples = []
    for polygon in polygons:
        a = np.asarray(polygon)
        for start, end in zip(a, np.roll(a, -1, axis=0)):
            n = max(1, int(np.ceil(np.linalg.norm((end - start) / size) * 200)))
            samples.extend(
                start + (end - start) * np.linspace(0, 1, n, endpoint=False)[:, None]
            )
    return {
        "center": center,
        "size": size,
        "points": (np.array(samples) - center) / size,
    }


def register_outline(reference, other, policy):
    best = None
    for swap in (False, True):
        for sx in (1, -1):
            for sy in (1, -1):
                matrix = np.array([[0, sx], [sy, 0]]) if swap else np.diag([sx, sy])
                transformed_size = abs(matrix) @ reference["size"]
                ratio = np.maximum(
                    transformed_size / other["size"], other["size"] / transformed_size
                ).max()
                if ratio > policy["max_size_ratio"]:
                    continue
                points = reference["points"] @ matrix.T
                distances = np.r_[
                    cKDTree(points).query(other["points"])[0],
                    cKDTree(other["points"]).query(points)[0],
                ]
                rms = float(np.sqrt(np.mean(distances**2)))
                p95 = float(np.percentile(distances, 95))
                if (
                    rms <= policy["max_normalized_rms"]
                    and p95 <= policy["max_normalized_p95"]
                    and (best is None or rms < best["rms"] - 1e-9)
                ):
                    best = {
                        "matrix": matrix,
                        "rms": rms,
                        "p95": p95,
                        "max_size_ratio": float(ratio),
                    }
    return best


def discover_groups(contexts, policy):
    descriptors = {name: outline_descriptor(c) for name, c in contexts.items()}
    remaining = list(contexts)
    groups = []
    while remaining:
        reference = remaining.pop(0)
        registrations = {
            reference: {
                "matrix": np.eye(2),
                "rms": 0.0,
                "p95": 0.0,
                "max_size_ratio": 1.0,
            }
        }
        if policy["enabled"]:
            for name in list(remaining):
                match = register_outline(
                    descriptors[reference], descriptors[name], policy
                )
                if match is not None:
                    registrations[name] = match
                    remaining.remove(name)
        groups.append(
            {
                "reference": reference,
                "registrations": registrations,
                "descriptors": {n: descriptors[n] for n in registrations},
            }
        )
    return groups


def fasten_group(contexts, group):
    """Accept a hole only if all members can use that location AND screw length.

    No independent fallback: insufficient common geometry must be reported.
    """
    reference = group["reference"]
    names = list(group["registrations"])
    descriptors = group["descriptors"]
    matrices = {n: group["registrations"][n]["matrix"] for n in names}
    normalized = []
    for name in names:
        d = descriptors[name]
        values = (np.asarray(contexts[name]["points"]) - d["center"]) / d["size"]
        normalized.extend(values @ matrices[name])
    values = np.unique(np.round(normalized, 8), axis=0)
    accepted = {n: [] for n in names}
    rejected = {n: [] for n in names}
    target = max(target_count(contexts[n]) for n in names)
    lengths = sorted(
        set.intersection(*(set(contexts[n]["p"]["screw_lengths_mm"]) for n in names))
    )
    for index in farthest_order(values * descriptors[reference]["size"]):
        normalized_point = values[index]
        points = {
            n: descriptors[n]["center"]
            + (matrices[n] @ normalized_point) * descriptors[n]["size"]
            for n in names
        }
        if any(
            any(
                np.linalg.norm(points[n] - f["point"])
                < contexts[n]["p"]["min_spacing_mm"]
                for f in accepted[n]
            )
            for n in names
        ):
            continue
        features = None
        for length in lengths:
            trial = {}
            try:
                for name in names:
                    trial[name] = try_candidate(contexts[name], points[name], length)
            except ValueError as exc:
                rejected[name].append(
                    {
                        "local_xy_mm": points[name].tolist(),
                        "length_mm": length,
                        "reason": str(exc),
                    }
                )
                continue
            features = trial
            break
        if features is None:
            continue
        for name, feature in features.items():
            feature["record"]["shared_pattern_position"] = len(accepted[name]) + 1
            feature["record"]["pattern_normalized_xy"] = normalized_point.tolist()
            accepted[name].append(feature)
        if len(accepted[reference]) == target:
            break
    results = {}
    try:
        for name in names:
            results[name] = finish_pair(
                contexts[name], accepted[name], rejected[name], target
            )
    except (ClosureSearchError, ValueError) as exc:
        raise ClosureSearchError(
            {
                "status": "failed_shared_pattern",
                "members": names,
                "connection_count": len(accepted[reference]),
                "required_count": max(
                    contexts[n]["p"]["min_connections"] for n in names
                ),
                "candidate_rejections": rejected,
                "reason": str(exc),
            }
        ) from exc
    for name in names:
        registration = group["registrations"][name]
        results[name]["report"]["shared_pattern"] = {
            "id": "similar_outline:" + reference,
            "reference_link": reference,
            "members": names,
            "mapping": "normalized bounding box plus planar rotation/reflection",
            "normalized_matrix_from_reference": matrices[name].tolist(),
            "local_outline_center_mm": descriptors[name]["center"].tolist(),
            "local_outline_size_mm": descriptors[name]["size"].tolist(),
            "outline_rms": registration["rms"],
            "outline_p95": registration["p95"],
            "same_corresponding_screw_lengths": True,
            "note": "Approximate shape correspondence; not a claim of identical geometry or interchangeable printed parts",
        }
    return results
