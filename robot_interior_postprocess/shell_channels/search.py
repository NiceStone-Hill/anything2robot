"""Reusable wall samples and radius-aware shortcut search in a surface corridor."""

from __future__ import annotations

import heapq
import math
from itertools import count

import numpy as np

from .geometry import dense, rounded, union


class SurfaceField:
    def __init__(self, surface, endpoints, diameter, previous=(), search_margin=0):
        settings = surface.settings
        radius = diameter / 2 + settings["clearance_mm"]
        depth = settings["depth_mm"]
        step = settings["grid_mm"]
        sample = settings["sample_mm"]
        minimum = settings["min_wall_mm"]
        span = surface.bounds[1, :2] - surface.bounds[0, :2]
        shape = np.ceil(span / step).astype(int) + 1
        if np.prod(shape) > settings["max_grid_cells"]:
            raise ValueError("Surface grid exceeds configured budget")
        lo = surface.bounds[0, :2]
        occupied = union(previous)
        obstacle = surface.obstacles + occupied
        allowed_cache = {}
        ends = np.asarray(endpoints)[:, :2]

        def allowed(uv, search_padding=True):
            key = (*np.round(uv, 6), search_padding)
            if key not in allowed_cache:
                try:
                    p, thickness = surface.point(uv, radius, depth)
                    okay = thickness >= depth + minimum + settings["wall_guard_mm"]
                    # Sample the cutter footprint, not only its centre ray.
                    samples = []
                    # A declared port may start in a narrow, valid passage.
                    # Extra search padding grows away from endpoints; mandatory
                    # cable/cutter clearance and final checks never shrink.
                    ramp = min(
                        1.0,
                        float(np.linalg.norm(ends - uv, axis=1).min())
                        / (2 * settings["bend_radius_mm"]),
                    )
                    for angle in np.linspace(0, 2 * math.pi, 9)[:-1]:
                        offset = (
                            radius + (search_margin * ramp if search_padding else 0)
                        ) * np.array([math.cos(angle), math.sin(angle)])
                        q, thick = surface.point(uv + offset, radius, depth)
                        okay &= thick >= depth + minimum + settings["wall_guard_mm"]
                        samples.append(q)
                    if okay:
                        for q in [p, *samples]:
                            bottom = q - [0, 0, radius]
                            top = q + [0, 0, radius + settings["lay_in_mm"]]
                            if surface.protected.ray_cast(
                                bottom, [q[0], q[1], surface.high]
                            ) or obstacle.ray_cast(bottom, top):
                                okay = False
                                break
                    if okay:
                        okay = surface.mask_hits([p, *samples]) == 0
                    allowed_cache[key] = bool(okay)
                except ValueError:
                    allowed_cache[key] = False
            return allowed_cache[key]

        def uv_of(node):
            return lo + np.asarray(node) * step

        def segment_clear(a, b):
            return all(allowed(p) for p in dense([a, b], sample))

        self.allowed = allowed
        self.segment_clear = segment_clear
        self.uv_of = uv_of
        self.shape, self.lo = shape, lo
        self.surface, self.radius, self.depth = surface, radius, depth
        self.settings = settings


def radius_shortcuts(path, field):
    """Search visibility shortcuts, rejecting each infeasible fillet BEFORE commit.

    A state retains the incoming edge and its already consumed tangent length.
    Hence two neighbouring corners cannot spend the same straight segment twice.
    The grid corridor may be retried with padding if no rounded shortcut exists.
    """
    path = np.asarray(path, float)
    path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-6]]
    radius, step = field.settings["bend_radius_mm"], field.settings["sample_mm"]
    # Bounded waypoint lattice along the grid corridor. This is a proposal
    # search; failure is reported rather than weakening any clearance/radius.
    limit = int(field.settings["max_shortcut_nodes"])
    if len(path) > limit:
        path = path[
            np.unique(np.rint(np.linspace(0, len(path) - 1, limit)).astype(int))
        ]
    last = len(path) - 1
    visible, turns = {}, {}
    checks = 0

    def clear(a, b):
        if (a, b) not in visible:
            visible[a, b] = field.segment_clear(path[a], path[b])
        return visible[a, b]

    def turn(a, b, c, previous_trim):
        u, v = path[b] - path[a], path[c] - path[b]
        lu, lv = np.linalg.norm(u), np.linalg.norm(v)
        u, v = u / lu, v / lv
        angle = math.acos(np.clip(u @ v, -1, 1))
        trim = radius * math.tan(angle / 2)
        if (
            angle > math.pi - 1e-4
            or previous_trim + trim >= lu - 1e-6
            or trim >= lv - 1e-6
        ):
            return trim, False
        if (a, b, c) not in turns:
            if angle < 1e-4:
                okay = True
            else:
                toward = v - (u @ v) * u
                toward /= np.linalg.norm(toward)
                center = path[b] - trim * u + radius * toward
                arc = [
                    center - radius * toward * math.cos(t) + radius * u * math.sin(t)
                    for t in np.linspace(
                        0, angle, max(3, math.ceil(radius * angle / step) + 1)
                    )
                ]
                okay = all(field.allowed(p, search_padding=False) for p in arc)
            turns[a, b, c] = okay
        return trim, turns[a, b, c]

    greedy, b = [0], 0
    while b < last:
        options = [c for c in range(last, b, -1) if clear(b, c)]
        if not options:
            break
        b = options[0]
        greedy.append(b)
    if b == last:
        try:
            uv = rounded(path[greedy], radius, step)
            if all(field.allowed(p, search_padding=False) for p in uv):
                return uv, 0
        except ValueError:
            pass
    serial = count()
    start = (-1, 0, 0.0)
    queue = [(0.0, next(serial), start)]
    cost, parent = {start: 0.0}, {}
    expanded = 0
    while queue:
        priority, _, state = heapq.heappop(queue)
        a, b, previous_trim = state
        expected = cost[state] + 1.5 * np.linalg.norm(path[b] - path[last])
        if priority > expected + 1e-6 and state != start:
            continue
        if b == last:
            indices = [b]
            while state != start:
                state = parent[state]
                indices.append(state[1])
            uv = rounded(path[indices[::-1]], radius, step)
            return uv, expanded
        expanded += 1
        if expanded > field.settings["max_expansions"]:
            raise ValueError("Bend-aware shortcut search budget exhausted")
        for c in range(last, b, -1):
            checks += 1
            if checks > field.settings["max_shortcut_checks"]:
                raise ValueError("Bend-aware shortcut transition budget exhausted")
            if not clear(b, c):
                continue
            trim = 0.0
            if a >= 0:
                trim, okay = turn(a, b, c, previous_trim)
                if (
                    not okay
                    or previous_trim + trim >= np.linalg.norm(path[b] - path[a]) - 1e-6
                ):
                    continue
            nxt = (b, c, round(trim, 7))
            value = cost[state] + np.linalg.norm(path[c] - path[b])
            if value < cost.get(nxt, float("inf")):
                cost[nxt], parent[nxt] = value, state
                heapq.heappush(
                    queue,
                    (
                        value + 1.5 * np.linalg.norm(path[c] - path[last]),
                        next(serial),
                        nxt,
                    ),
                )
    raise ValueError("No clearance-safe shortcut sequence with requested bend radius")


def plan_surface(field, endpoints):
    ends = np.asarray(endpoints)[:, :2]
    surface, settings = field.surface, field.settings
    nodes = [
        tuple(np.rint((p - field.lo) / settings["grid_mm"]).astype(int)) for p in ends
    ]
    start, goal = nodes
    for end, node in zip(ends, nodes):
        if not field.segment_clear(end, field.uv_of(node)):
            raise ValueError("Port lead projection has no safe surface entry")
    # The direct segment has no surface corner and needs no grid exploration.
    if field.segment_clear(*ends):
        return np.array(
            [
                surface.point(p, field.radius, field.depth)[0]
                for p in dense(ends, settings["sample_mm"])
            ]
        ), 0
    queue = [(0.0, start)]
    costs, parent = {start: 0.0}, {}
    expanded = 0
    while queue:
        priority, node = heapq.heappop(queue)
        if (
            priority > costs[node] + np.linalg.norm(field.uv_of(node) - ends[1]) + 1e-6
            and node != start
        ):
            continue
        if node == goal:
            break
        expanded += 1
        if expanded > settings["max_expansions"]:
            raise ValueError("Surface search budget exhausted")
        for delta in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (-1, 1),
            (1, -1),
            (-1, -1),
        ):
            nxt = tuple(np.asarray(node) + delta)
            if np.any(np.asarray(nxt) < 0) or np.any(np.asarray(nxt) >= field.shape):
                continue
            a, b = field.uv_of(node), field.uv_of(nxt)
            if not field.segment_clear(a, b):
                continue
            length = np.linalg.norm(
                surface.point(a, field.radius, field.depth)[0]
                - surface.point(b, field.radius, field.depth)[0]
            )
            cost = costs[node] + length
            if cost < costs.get(nxt, float("inf")):
                costs[nxt], parent[nxt] = cost, node
                heapq.heappush(queue, (cost + np.linalg.norm(b - ends[1]), nxt))
    if goal not in costs:
        raise ValueError("No visible-surface path")
    path, node = [ends[1], field.uv_of(goal)], goal
    while node != start:
        node = parent[node]
        path.append(field.uv_of(node))
    path.append(ends[0])
    uv, bend_expansions = radius_shortcuts(path[::-1], field)
    if not all(field.allowed(p, search_padding=False) for p in uv):
        raise ValueError("Rounded bend enters a forbidden surface region")
    return np.array(
        [surface.point(p, field.radius, field.depth)[0] for p in uv]
    ), expanded + bend_expansions
