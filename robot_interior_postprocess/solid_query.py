"""Fast point-in-solid checks against exported closed meshes, including cavities."""

import manifold3d as md
import numpy as np


class SolidQuery:
    def __init__(self, mesh):
        self.bounds = np.asarray(mesh.bounds)
        self.solid = md.Manifold(
            md.Mesh(
                np.asarray(mesh.vertices, dtype=np.float32),
                np.asarray(mesh.faces, dtype=np.uint32),
            )
        )
        if self.solid.status() != md.Error.NoError:
            raise ValueError("Occupancy queries require a valid manifold mesh")
        self.vector = np.array([1.0, 0.137123, 0.231731]) * max(
            1.0, np.linalg.norm(self.bounds[1] - self.bounds[0]) * 4
        )

    def contains(self, points):
        points = np.asarray(points, dtype=float)
        result = np.zeros(len(points), bool)
        eligible = np.where(
            np.all(points >= self.bounds[0] - 1e-6, axis=1)
            & np.all(points <= self.bounds[1] + 1e-6, axis=1)
        )[0]
        for i in eligible:
            hits = self.solid.ray_cast(points[i], points[i] + self.vector)
            distances = sorted({round(h.distance, 10) for h in hits})
            result[i] = (
                bool(distances and distances[0] < 1e-8) or len(distances) % 2 == 1
            )
        return result
