"""Visible-surface routing in an explicit opening frame, in millimetres.

One chart covers a surface visible along its opening normal. Re-entrant surfaces
are rejected instead of jumping to another sheet. Final CSG checks are mandatory;
the grid is only a proposal generator, not a manufacturing certificate.
"""

from __future__ import annotations

import math
from itertools import pairwise

import manifold3d as md
import numpy as np
import trimesh
from hardware import as_mesh64


def solid(mesh):
    value = md.Manifold(
        md.Mesh64(
            np.array(mesh.vertices, dtype=np.float64, order="C"),
            np.array(mesh.faces, dtype=np.uint64, order="C"),
        )
    )
    if value.status() != md.Error.NoError or value.volume() <= 0:
        raise ValueError("Expected closed, oriented positive-volume material")
    return value


def union(items):
    items = list(items)
    return md.Manifold.batch_boolean(items, md.OpType.Add) if items else md.Manifold()


def capsule(a, b, radius):
    a, b = np.asarray(a), np.asarray(b)
    ball = md.Manifold.sphere(radius, circular_segments=16)
    if np.linalg.norm(a - b) < 1e-8:
        return ball.translate(a)
    return (ball.translate(a) + ball.translate(b)).hull()


def sweep(points, radius):
    return union(capsule(a, b, radius) for a, b in pairwise(points))


def open_sweep(points, radius, travel):
    return union(
        (s + s.translate([0, 0, travel])).hull()
        for a, b in pairwise(points)
        for s in [capsule(a, b, radius)]
    )


def frame(normal):
    normal = np.asarray(normal, float)
    if (
        normal.shape != (3,)
        or not np.isfinite(normal).all()
        or not np.isclose(np.linalg.norm(normal), 1)
    ):
        raise ValueError("Opening normal must be a finite unit vector")
    # Deterministic right-handed frame, independent of robot/link names.
    reference = np.eye(3)[np.argmin(abs(normal))]
    u = np.cross(normal, reference)
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    matrix = np.eye(4)
    matrix[:3, :3] = np.vstack((u, v, normal))
    return matrix


def dense(points, step):
    output = []
    for a, b in pairwise(np.asarray(points)):
        count = max(1, math.ceil(np.linalg.norm(b - a) / step))
        output.extend(np.linspace(a, b, count + 1)[:-1])
    return np.array([*output, points[-1]])


def simplify_curve(points, tolerance):
    """Bounded chord error reduces CSG segments, retaining both exact endpoints."""
    points = np.asarray(points, float)
    keep, pending = {0, len(points) - 1}, [(0, len(points) - 1)]
    while pending:
        a, b = pending.pop()
        if b <= a + 1:
            continue
        delta = points[b] - points[a]
        norm = float(delta @ delta)
        t = np.clip((points[a + 1 : b] - points[a]) @ delta / max(norm, 1e-20), 0, 1)
        distances = np.linalg.norm(
            points[a + 1 : b] - points[a] - t[:, None] * delta, axis=1
        )
        index = int(np.argmax(distances)) + a + 1
        if distances[index - a - 1] > tolerance:
            keep.add(index)
            pending.extend(((a, index), (index, b)))
    return points[sorted(keep)]


def check_self_clearance(points, diameter):
    """Reject reversals and sampled nonlocal cable self-contact, including leads."""
    from scipy.spatial import cKDTree

    points = np.asarray(points, dtype=float)
    points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-7]]
    directions = np.diff(points, axis=0)
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    if len(directions) > 1 and np.any(
        np.sum(directions[:-1] * directions[1:], axis=1) < -0.99
    ):
        raise ValueError("Cable doubles back on itself at a lead transition")
    samples = dense(points, min(0.5, diameter / 3))
    lengths = np.r_[0, np.cumsum(np.linalg.norm(np.diff(samples, axis=0), axis=1))]
    for a, b in cKDTree(samples).query_pairs(diameter - 1e-4):
        if abs(lengths[b] - lengths[a]) > math.pi * diameter:
            raise ValueError("Cable has sampled nonlocal self-contact")


def rounded(points, radius, step):
    """Tangent circular fillets. Reject short segments instead of shrinking radius."""
    points = np.asarray(points, float)
    points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-7]]
    arcs, trims = {}, np.zeros(len(points))
    for i in range(1, len(points) - 1):
        incoming = points[i] - points[i - 1]
        outgoing = points[i + 1] - points[i]
        incoming /= np.linalg.norm(incoming)
        outgoing /= np.linalg.norm(outgoing)
        angle = math.acos(np.clip(incoming @ outgoing, -1, 1))
        if angle < 1e-4:
            continue
        if angle > math.pi - 1e-4:
            raise ValueError("Path reverses direction")
        trim = radius * math.tan(angle / 2)
        trims[i] = trim
        toward = outgoing - (incoming @ outgoing) * incoming
        toward /= np.linalg.norm(toward)
        start = points[i] - trim * incoming
        center = start + radius * toward
        samples = max(3, math.ceil(radius * angle / step) + 1)
        arcs[i] = np.array(
            [
                center - radius * toward * math.cos(t) + radius * incoming * math.sin(t)
                for t in np.linspace(0, angle, samples)
            ]
        )
    if np.any(
        trims[:-1] + trims[1:] >= np.linalg.norm(np.diff(points, axis=0), axis=1) - 1e-6
    ):
        raise ValueError(
            "Insufficient segment length for requested bend radius: "
            + str(np.round(points, 2).tolist())
        )
    output = [points[0]]
    for i in range(1, len(points) - 1):
        output.extend(arcs.get(i, [points[i]]))
    output.append(points[-1])
    return dense(output, step)


class Surface:
    def __init__(self, shell, base, normal, settings, obstacles=(), masks=None):
        self.local = frame(normal)
        self.world = np.linalg.inv(self.local)
        self.shell = shell.transform(self.local[:3])
        self.base = base.transform(self.local[:3])
        self.protected = self.shell - self.base
        self.obstacles = union(o.transform(self.local[:3]) for o in obstacles)
        self.bounds = as_mesh64(self.base).bounds
        self.low = self.bounds[0, 2] - 5
        self.high = self.bounds[1, 2] + 5
        self.travel = float(self.high - self.low + 5)
        self.settings = settings
        self.masks = masks
        self.cache = {}

    def wall(self, uv):
        key = tuple(np.round(uv, 6))
        if key not in self.cache:
            hits = self.base.ray_cast([*uv, self.low], [*uv, self.high])
            positions = sorted({round(float(h.position[2]), 7) for h in hits})
            # A chart must not silently select a hidden fold/another shell layer.
            if len(positions) != 2:
                raise ValueError("No single visible wall interval")
            self.cache[key] = positions
        return self.cache[key]

    def point(self, uv, radius, depth):
        outer, inner = self.wall(uv)
        return np.array([*uv, inner + radius - depth]), inner - outer

    def mask_hits(self, points):
        if self.masks is None:
            return 0
        points = trimesh.transform_points(points, self.world)
        ids = np.rint(
            (points - self.masks["origin"]) / float(self.masks["pitch"])
        ).astype(int)
        valid = np.all((ids >= 0) & (ids < self.masks["motor_block"].shape), axis=1)
        if not valid.all():
            return len(ids)
        return int(
            (
                self.masks["motor_block"][tuple(ids.T)]
                | self.masks["protected_material"][tuple(ids.T)]
            ).sum()
        )

    def plan(self, endpoints, diameter, previous=(), search_margin=0, field=None):
        from .search import SurfaceField, plan_surface

        field = field or SurfaceField(
            self, endpoints, diameter, previous, search_margin
        )
        return plan_surface(field, endpoints)
