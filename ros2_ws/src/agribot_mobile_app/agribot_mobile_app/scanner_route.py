"""Add scanner-reference guides without changing manually selected anchor poses."""

from copy import deepcopy
import math

import cv2
import numpy as np


def ordered_reference_matches(reference, anchors, maximum_distance=0.75):
    reference = np.asarray(reference, dtype=float)
    anchors = np.asarray(anchors, dtype=float)
    if (reference.ndim != 2 or reference.shape[1] != 2 or len(reference) < 2
            or anchors.ndim != 2 or anchors.shape[1] != 2 or len(anchors) < 2
            or not np.isfinite(reference).all() or not np.isfinite(anchors).all()
            or not math.isfinite(maximum_distance) or maximum_distance <= 0):
        raise ValueError("Finite XY arrays and a positive matching distance are required")
    candidates, costs, parents = [], [], []
    for point in anchors:
        distances = np.linalg.norm(reference - point, axis=1)
        indices = np.flatnonzero(distances <= maximum_distance)
        if not len(indices):
            raise ValueError("A manual point is too far from the scanner reference")
        cost = distances[indices] ** 2
        parent = np.full(len(indices), -1, dtype=int)
        if candidates:
            previous = costs[-1]
            prefix_minimum = np.minimum.accumulate(previous)
            prefix_index = np.zeros(len(previous), dtype=int)
            best = 0
            for index in range(len(previous)):
                if previous[index] < previous[best]:
                    best = index
                prefix_index[index] = best
            positions = np.searchsorted(candidates[-1], indices) - 1
            valid = positions >= 0
            cost[~valid] = np.inf
            cost[valid] += prefix_minimum[positions[valid]]
            parent[valid] = prefix_index[positions[valid]]
        if not np.isfinite(cost).any():
            raise ValueError("Manual point order does not follow the scanner traversal")
        candidates.append(indices)
        costs.append(cost)
        parents.append(parent)
    selected = int(np.argmin(costs[-1]))
    matches = []
    for index in range(len(anchors) - 1, -1, -1):
        matches.append(int(candidates[index][selected]))
        selected = int(parents[index][selected])
    return list(reversed(matches))


def densify_reference_route(reference, poses, *, maximum_distance=0.75,
                            tolerance=0.25, maximum_spacing=12.0,
                            heading_window=0.75, maximum_points=100):
    for value in (tolerance, maximum_spacing, heading_window):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("Densification distances must be positive and finite")
    if maximum_points < 2 or len(poses) > maximum_points:
        raise ValueError("Manual points exceed the route point limit")
    for pose in poses:
        if not all(math.isfinite(pose[key]) for key in ("x", "y", "yaw")):
            raise ValueError("Manual poses must be finite")
    reference = np.asarray(reference, dtype=float)
    matches = ordered_reference_matches(
        reference, [[pose["x"], pose["y"]] for pose in poses], maximum_distance)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
    chosen = set(matches)
    for first, last in zip(matches, matches[1:]):
        points = reference[first:last + 1].astype(np.float32)
        corners = cv2.approxPolyDP(points.reshape(-1, 1, 2), tolerance, False).reshape(-1, 2)
        previous = 0
        for corner in corners[1:-1]:
            candidates = np.flatnonzero(np.all(points[previous + 1:-1] == corner, axis=1))
            if not len(candidates):
                raise ValueError("Cannot map simplified corner to an ordered scanner sample")
            previous += 1 + int(candidates[0])
            chosen.add(first + previous)
    # Split long straight runs by arc length, retaining real samples rather than
    # interpolating XY across a turn or between overlapping outward/return legs.
    for first, last in zip(sorted(chosen), sorted(chosen)[1:]):
        count = int(math.ceil((arc[last] - arc[first]) / maximum_spacing))
        for target in np.linspace(arc[first], arc[last], count + 1)[1:-1]:
            chosen.add(int(np.searchsorted(arc, target)))
    indices = sorted(chosen)
    if len(indices) > maximum_points:
        raise ValueError(f"{len(indices)} points exceed the {maximum_points}-point limit; "
                         "explicitly choose a different density or split the route")
    originals = dict(zip(matches, enumerate(poses)))
    result, anchor_positions = [], []
    for index in indices:
        if index in originals:
            anchor_positions.append(len(result))
            result.append(deepcopy(poses[originals[index][0]]))
            continue
        first = max(matches[0], int(np.searchsorted(arc, arc[index] - heading_window)))
        last = min(matches[-1], int(np.searchsorted(arc, arc[index] + heading_window)))
        direction = reference[last] - reference[first]
        if np.linalg.norm(direction) < 1e-6:
            raise ValueError("Cannot determine scanner heading at an added guide point")
        result.append({"x": float(reference[index, 0]), "y": float(reference[index, 1]),
                       "yaw": math.atan2(direction[1], direction[0])})
    return {"poses": result, "source_sample_indices": indices,
            "manual_anchor_pose_indices": anchor_positions,
            "manual_anchor_reference_indices": matches,
            "manual_anchor_reference_offsets_m": [
                float(np.linalg.norm(reference[index] - [pose["x"], pose["y"]]))
                for pose, index in zip(poses, matches)]}
