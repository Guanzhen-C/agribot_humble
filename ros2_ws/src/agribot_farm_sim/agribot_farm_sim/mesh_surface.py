"""Vertical intersection with inspected OBJ parts for nominal mounting feet."""

from pathlib import Path

import numpy as np


class Surface:
    def __init__(self, triangles):
        self.triangles = np.asarray(triangles, dtype=float).reshape((-1, 3, 3))

    def height(self, x, y):
        triangles = self.triangles
        if not len(triangles):
            raise ValueError("mounting surface has no triangles")
        low, high = triangles[:, :, :2].min(axis=1), triangles[:, :, :2].max(axis=1)
        inside = ((low <= [x, y]) & (high >= [x, y])).all(axis=1)
        a, b, c = triangles[inside].transpose(1, 0, 2)
        ab, ac = b - a, c - a
        determinant = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
        valid = abs(determinant) > 1e-12
        a, ab, ac, determinant = a[valid], ab[valid], ac[valid], determinant[valid]
        dx, dy = x - a[:, 0], y - a[:, 1]
        u = (dx * ac[:, 1] - dy * ac[:, 0]) / determinant
        v = (ab[:, 0] * dy - ab[:, 1] * dx) / determinant
        valid = (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8)
        heights = (a[:, 2] + u * ab[:, 2] + v * ac[:, 2])[valid]
        if not len(heights):
            raise ValueError(f"no mounting surface directly below ({x}, {y})")
        return float(max(heights))


def load_surfaces(path, object_names):
    """Read specified OBJ objects; triangulate the convex exported CAD faces."""
    vertices, faces = [], {name: [] for name in object_names}
    current = None
    with Path(path).open() as stream:
        for line in stream:
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "o":
                current = line[2:].strip()
            elif fields[0] == "v":
                vertices.append(tuple(map(float, fields[1:4])))
            elif fields[0] == "f" and current in faces:
                indices = [int(field.split("/")[0]) for field in fields[1:]]
                indices = [index - 1 if index > 0 else len(vertices) + index for index in indices]
                for i in range(1, len(indices) - 1):
                    faces[current].append((indices[0], indices[i], indices[i + 1]))
    vertices = np.asarray(vertices)
    return {name: Surface(vertices[np.asarray(indices, dtype=int)])
            for name, indices in faces.items()}
