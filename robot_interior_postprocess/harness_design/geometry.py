"""Solid-derived occupancy, bounded A* and constant-radius candidate bends."""

from __future__ import annotations

import heapq
import itertools
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from shell_channels.geometry import dense, rounded


def rasterize(solid, origin, pitch, shape):
    """Even-odd horizontal sections preserve hollow interiors unlike filled voxels."""
    result = np.zeros(shape, bool)
    if solid.is_empty():
        return result
    bounds = np.array(solid.bounding_box()).reshape(2, 3)
    lo = np.maximum(0, np.floor((bounds[0] - origin) / pitch).astype(int) - 1)
    hi = np.minimum(shape, np.ceil((bounds[1] - origin) / pitch).astype(int) + 2)
    if np.any(hi <= lo):
        return result
    xs, _ys = np.meshgrid(
        np.arange(lo[0], hi[0]), np.arange(lo[1], hi[1]), indexing="ij"
    )
    x_values = origin[0] + np.arange(lo[0], hi[0]) * pitch
    y_values = origin[1] + np.arange(lo[1], hi[1]) * pitch
    # Sample centres; distance guard and final exact CSG remain necessary.
    for z in range(lo[2], hi[2]):
        polygons = solid.slice(float(origin[2] + z * pitch)).to_polygons()
        inside = np.zeros(xs.shape, bool)
        for poly in polygons:
            poly = np.asarray(poly)
            a = poly
            b = np.roll(poly, -1, axis=0)
            for j, y in enumerate(y_values):
                crossing = (a[:, 1] > y) != (b[:, 1] > y)
                if not crossing.any():
                    continue
                aa, bb = a[crossing], b[crossing]
                cuts = np.sort(
                    aa[:, 0]
                    + (y - aa[:, 1]) * (bb[:, 0] - aa[:, 0]) / (bb[:, 1] - aa[:, 1])
                )
                inside[:, j] ^= (np.searchsorted(cuts, x_values, side="right") % 2) == 1
        result[lo[0] : hi[0], lo[1] : hi[1], z] = inside
    return result


def largest_region(free):
    components, count = label(free)
    if not count:
        raise ValueError("No connected interior routing volume")
    sizes = np.bincount(components.ravel())
    sizes[0] = 0
    return components == sizes.argmax()


def line_clear(points, free, origin, pitch):
    samples = dense(points, pitch / 3)
    indices = np.rint((samples - origin) / pitch).astype(int)
    return bool(
        np.all(indices >= 0)
        and np.all(indices < free.shape)
        and free[tuple(indices.T)].all()
    )


def search(free, start, end, wall_distance, max_expansions=250000):
    start, end = tuple(start), tuple(end)
    if not free[start] or not free[end]:
        return None
    # Six neighbours avoid cutting corners; smoothing is separately checked.
    neighbors = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
    shape = free.shape

    def heuristic(p):
        return sum(abs(p[i] - end[i]) for i in range(3))

    queue = [(heuristic(start), 0.0, start)]
    costs = {start: 0.0}
    previous = {}
    expanded = 0
    while queue and expanded < max_expansions:
        _, cost, point = heapq.heappop(queue)
        if cost != costs.get(point):
            continue
        if point == end:
            path = [point]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return np.asarray(path[::-1])
        expanded += 1
        for delta in neighbors:
            nxt = tuple(point[i] + delta[i] for i in range(3))
            if any(nxt[i] < 0 or nxt[i] >= shape[i] for i in range(3)) or not free[nxt]:
                continue
            # Preference, not a constraint: keep long runs nearer available walls.
            new = cost + 1.0 + 0.015 * min(float(wall_distance[nxt]), 30.0)
            if new >= costs.get(nxt, math.inf):
                continue
            costs[nxt] = new
            previous[nxt] = point
            heapq.heappush(queue, (new + heuristic(nxt), new, nxt))
    return None


def simplify(points, free, origin, pitch):
    output = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not line_clear([points[i], points[j]], free, origin, pitch):
            j -= 1
        output.append(points[j])
        i = j
    return np.asarray(output)


def curve(path, free, origin, pitch, bend):
    points = origin + np.asarray(path) * pitch
    simplified = simplify(points, free, origin, pitch)
    try:
        candidate = rounded(simplified, bend, min(0.75, pitch / 3))
        if line_clear(candidate, free, origin, pitch):
            return candidate
    except ValueError:
        pass
    return rounded_visibility_path(points, free, origin, pitch, bend)


def reserve(free, points, origin, pitch, radius):
    from scipy.spatial import cKDTree

    output = free.copy()
    samples = dense(points, min(0.5, pitch / 3))
    guard = min(0.5, pitch / 3) / 2
    lo = np.maximum(
        0, np.floor((samples.min(axis=0) - radius - guard - origin) / pitch).astype(int)
    )
    hi = np.minimum(
        free.shape,
        np.ceil((samples.max(axis=0) + radius + guard - origin) / pitch).astype(int)
        + 1,
    )
    if np.any(hi <= lo):
        return output
    slices = tuple(slice(a, b) for a, b in zip(lo, hi))
    local = np.argwhere(output[slices]) + lo
    if len(local):
        distances = cKDTree(samples).query(origin + local * pitch)[0]
        blocked = local[distances < radius + guard]
        output[tuple(blocked.T)] = False
    return output


def guarded_free(domain, occupied, pitch, clearance):
    return domain & (
        distance_transform_edt(~occupied, sampling=pitch) > clearance + pitch * 0.87
    )


def rounded_visibility_path(points, free, origin, pitch, bend, limit=1200):
    """Search waypoint sequences with tangent-length constraints at both corners."""
    stride = max(1, int(np.ceil(len(points) / 48)))
    ids = sorted({0, len(points) - 1, *range(0, len(points), stride)})
    p = np.asarray(points)[ids]
    last = len(p) - 1
    lines = {}
    arcs = {}

    def visible(i, j):
        if (i, j) not in lines:
            lines[i, j] = line_clear([p[i], p[j]], free, origin, pitch)
        return lines[i, j]

    def turn(h, i, j):
        key = (h, i, j)
        if key not in arcs:
            a = p[i] - p[h]
            b = p[j] - p[i]
            angle = math.acos(
                np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)
            )
            trim = bend * math.tan(angle / 2)
            try:
                c = rounded([p[h], p[i], p[j]], bend, min(0.75, pitch / 3))
                valid = line_clear(c, free, origin, pitch)
            except ValueError:
                valid = False
            arcs[key] = (trim, valid)
        return arcs[key]

    queue = []
    counter = itertools.count()
    best = {}
    for j in range(1, last + 1):
        if visible(0, j):
            dist = float(np.linalg.norm(p[j] - p[0]))
            heapq.heappush(
                queue,
                (
                    dist + np.linalg.norm(p[last] - p[j]),
                    next(counter),
                    dist,
                    (0, j),
                    0.0,
                ),
            )
    expanded = 0
    while queue and expanded < limit:
        _, _, cost, sequence, previous_trim = heapq.heappop(queue)
        h, i = sequence[-2:]
        if i == last:
            result = rounded(p[list(sequence)], bend, min(0.75, pitch / 3))
            if line_clear(result, free, origin, pitch):
                return result
            continue
        expanded += 1
        candidates = [j for j in range(i + 1, last + 1) if visible(i, j)]
        # Keep near and far alternatives; very short final legs must not force a kink.
        if len(candidates) > 18:
            candidates = sorted(set(candidates[:6] + candidates[-12:]))
        for j in candidates:
            trim, valid = turn(h, i, j)
            if not valid or previous_trim + trim >= np.linalg.norm(p[i] - p[h]) - 1e-6:
                continue
            new = cost + np.linalg.norm(p[j] - p[i]) + bend * 0.1
            key = (h, i, j)
            if new >= best.get(key, math.inf):
                continue
            best[key] = new
            heapq.heappush(
                queue,
                (
                    new + np.linalg.norm(p[last] - p[j]),
                    next(counter),
                    new,
                    (*sequence, j),
                    trim,
                ),
            )
    raise ValueError(
        "No waypoint sequence satisfies bend radius and free-space constraints"
    )


def wire_solid(points, radius):
    """Discard only collinear samples before CSG; preserve the same polyline."""
    from shell_channels.geometry import sweep

    p = np.asarray(points, float)
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-10]]
    if len(p) > 2:
        a = np.diff(p, axis=0)[:-1]
        b = np.diff(p, axis=0)[1:]
        cross = np.linalg.norm(np.cross(a, b), axis=1)
        keep = (
            cross > 1e-10 * np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
        ) | (np.sum(a * b, axis=1) <= 0)
        p = p[np.r_[True, keep, True]]
    return sweep(p, radius)
