#!/usr/bin/env python3

"""Build a Chinese semantic topology from a 2D road-boundary map."""

import argparse
import hashlib
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


class MapTopologyError(RuntimeError):
    pass


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def contains_chinese(text):
    return bool(re.search(r"[\u3400-\u9fff]", str(text)))


class OccupancyMap:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).expanduser().resolve()
        document = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            raise MapTopologyError("map YAML must contain a mapping")
        image_path = Path(str(document.get("image", "")))
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        self.image_path = image_path.resolve()
        self.image = cv2.imread(str(self.image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise MapTopologyError(
                "failed to read map image: {}".format(self.image_path)
            )
        self.resolution = float(document.get("resolution", 0.0))
        self.origin = np.asarray(document.get("origin", []), dtype=np.float64)
        if self.resolution <= 0.0 or self.origin.shape != (3,):
            raise MapTopologyError("map resolution or origin is invalid")
        self.height, self.width = self.image.shape
        yaw = float(self.origin[2])
        self.rotation = np.asarray(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
            dtype=np.float64,
        )
        negate = bool(int(document.get("negate", 0)))
        normalized = self.image.astype(np.float64) / 255.0
        occupancy = normalized if negate else 1.0 - normalized
        self.free = occupancy < float(document.get("free_thresh", 0.196))
        self.occupied = occupancy > float(document.get("occupied_thresh", 0.65))
        self.clearance = cv2.distanceTransform(
            self.free.astype(np.uint8), cv2.DIST_L2, 5
        ) * self.resolution

    def assert_same_geometry(self, other):
        if self.image.shape != other.image.shape:
            raise MapTopologyError("map and road-boundary image sizes differ")
        if abs(self.resolution - other.resolution) > 1e-9:
            raise MapTopologyError("map and road-boundary resolutions differ")
        if not np.allclose(self.origin, other.origin, atol=1e-9):
            raise MapTopologyError("map and road-boundary origins differ")

    def pixel_to_world(self, row, column):
        local = np.asarray(
            [
                (float(column) + 0.5) * self.resolution,
                (float(self.height - 1) - float(row) + 0.5) * self.resolution,
            ],
            dtype=np.float64,
        )
        return self.origin[:2] + self.rotation @ local

    def world_to_pixel(self, point):
        local = self.rotation.T @ (np.asarray(point[:2]) - self.origin[:2])
        column = int(math.floor(local[0] / self.resolution))
        row_from_bottom = int(math.floor(local[1] / self.resolution))
        return self.height - 1 - row_from_bottom, column

    def clearance_at(self, point):
        row, column = self.world_to_pixel(point)
        if not (0 <= row < self.height and 0 <= column < self.width):
            return 0.0
        return float(self.clearance[row, column])


def periodic_gaussian(points, sigma_samples):
    points = np.asarray(points, dtype=np.float64)
    if sigma_samples <= 0.0 or len(points) < 4:
        return points.copy()
    radius = max(1, int(math.ceil(3.0 * sigma_samples)))
    offsets = np.arange(-radius, radius + 1, dtype=np.float64)
    weights = np.exp(-0.5 * (offsets / sigma_samples) ** 2)
    weights /= weights.sum()
    padded = np.concatenate((points[-radius:], points, points[:radius]), axis=0)
    return np.column_stack(
        [np.convolve(padded[:, axis], weights, mode="valid") for axis in range(2)]
    )


def resample_closed(points, spacing):
    if spacing <= 0.0:
        raise MapTopologyError("path sampling distance must be positive")
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise MapTopologyError("closed path must contain at least three 2D points")
    keep = np.concatenate(
        ([True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-9)
    )
    points = points[keep]
    if len(points) >= 2 and np.linalg.norm(points[0] - points[-1]) <= 1e-9:
        points = points[:-1]
    closed = np.vstack((points, points[0]))
    segment_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    if total <= 3.0 * spacing:
        raise MapTopologyError("road centerline is too short")
    targets = np.arange(0.0, total, spacing)
    sampled = np.column_stack(
        [
            np.interp(targets, cumulative, closed[:, axis])
            for axis in range(2)
        ]
    )
    return sampled, total


def interpolate_closed(points, distances):
    points = np.asarray(points, dtype=np.float64)
    closed = np.vstack((points, points[0]))
    segment_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    targets = np.mod(np.asarray(distances, dtype=np.float64), total)
    return np.column_stack(
        [
            np.interp(targets, cumulative, closed[:, axis])
            for axis in range(2)
        ]
    ), total


def component_contours(boundary_map, minimum_pixels):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        boundary_map.occupied.astype(np.uint8), 8
    )
    candidates = []
    for component in range(1, count):
        if int(stats[component, cv2.CC_STAT_AREA]) < minimum_pixels:
            continue
        mask = (labels == component).astype(np.uint8) * 255
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        if not contours:
            continue
        contour = max(contours, key=lambda item: abs(cv2.contourArea(item)))
        area = abs(float(cv2.contourArea(contour)))
        if area > 0.0:
            candidates.append((area, contour[:, 0, :].astype(np.float64)))
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates


def filled_contour(shape, points):
    polygon = np.rint(points).astype(np.int32).reshape(-1, 1, 2)
    line = np.zeros(shape, dtype=np.uint8)
    fill = np.zeros(shape, dtype=np.uint8)
    cv2.polylines(line, [polygon], True, 1, 1)
    cv2.fillPoly(fill, [polygon], 1)
    return line, fill


def select_road_boundaries(boundary_map, arguments):
    candidates = component_contours(
        boundary_map, arguments.minimum_boundary_pixels
    )
    if len(candidates) < 2:
        raise MapTopologyError(
            "road-boundary map must contain two closed boundary components"
        )

    selected = None
    for outer_area, outer in candidates:
        outer_line, outer_fill = filled_contour(boundary_map.image.shape, outer)
        del outer_line
        for inner_area, inner in candidates:
            if inner_area >= outer_area:
                continue
            _, inner_fill = filled_contour(boundary_map.image.shape, inner)
            overlap = int(np.count_nonzero(inner_fill & outer_fill))
            inner_pixels = int(np.count_nonzero(inner_fill))
            if inner_pixels and overlap / inner_pixels >= 0.98:
                selected = (outer, inner)
                break
        if selected is not None:
            break
    if selected is None:
        raise MapTopologyError(
            "no nested outer and inner road boundaries were found"
        )
    return selected


def extract_road_centerline(boundary_map, arguments):
    outer, inner = select_road_boundaries(boundary_map, arguments)
    boundary_step_pixels = arguments.boundary_sample_spacing / boundary_map.resolution
    boundary_sigma = (
        arguments.boundary_smoothing / arguments.boundary_sample_spacing
    )
    smoothed = []
    for contour in (outer, inner):
        sampled, _ = resample_closed(contour, boundary_step_pixels)
        smoothed.append(periodic_gaussian(sampled, boundary_sigma))

    masks = [filled_contour(boundary_map.image.shape, curve) for curve in smoothed]
    fill_sizes = [int(np.count_nonzero(item[1])) for item in masks]
    outer_index, inner_index = np.argsort(fill_sizes)[::-1][:2]
    outer_line, outer_fill = masks[int(outer_index)]
    inner_line, inner_fill = masks[int(inner_index)]
    corridor = outer_fill.astype(bool) & ~inner_fill.astype(bool)
    if np.count_nonzero(corridor) < arguments.minimum_boundary_pixels:
        raise MapTopologyError("the area between road boundaries is empty")

    distance_outer = cv2.distanceTransform(
        (1 - outer_line).astype(np.uint8), cv2.DIST_L2, 5
    )
    distance_inner = cv2.distanceTransform(
        (1 - inner_line).astype(np.uint8), cv2.DIST_L2, 5
    )
    middle_band = (
        corridor
        & (np.abs(distance_outer - distance_inner) <= arguments.middle_band_pixels)
        & (distance_outer > 2.0)
        & (distance_inner > 2.0)
    ).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(middle_band, 8)
    if count <= 1:
        raise MapTopologyError("failed to extract a connected road centerline")
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    component_mask = (labels == component).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        component_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        raise MapTopologyError("road centerline has no closed contour")
    center_pixels = max(
        contours, key=lambda item: cv2.arcLength(item, True)
    )[:, 0, :].astype(np.float64)
    center_world = np.asarray(
        [
            boundary_map.pixel_to_world(point[1], point[0])
            for point in center_pixels
        ]
    )
    center_world, _ = resample_closed(
        center_world, arguments.centerline_sample_spacing
    )
    center_world = periodic_gaussian(
        center_world,
        arguments.centerline_smoothing / arguments.centerline_sample_spacing,
    )
    center_world, _ = resample_closed(
        center_world, arguments.centerline_sample_spacing
    )

    requested_start = np.asarray(arguments.start_position, dtype=np.float64)
    start_index = int(
        np.argmin(np.linalg.norm(center_world - requested_start, axis=1))
    )
    center_world = np.roll(center_world, -start_index, axis=0)
    tangent = center_world[2] - center_world[-2]
    heading = np.asarray(
        [math.cos(math.radians(arguments.start_yaw_deg)),
         math.sin(math.radians(arguments.start_yaw_deg))]
    )
    if float(np.dot(tangent, heading)) < 0.0:
        center_world = np.vstack((center_world[0], center_world[:0:-1]))
    return center_world, corridor


def snap_path_to_safe_space(path, occupancy_map, corridor, arguments):
    projection_clearance = (
        arguments.minimum_centerline_clearance
        + arguments.safety_clearance_margin
    )
    safe = (
        corridor
        & occupancy_map.free
        & (occupancy_map.clearance >= projection_clearance)
    )
    radius = int(math.ceil(arguments.maximum_centerline_snap / occupancy_map.resolution))
    path = np.asarray(path, dtype=np.float64).copy()
    maximum_snap = 0.0
    for _ in range(arguments.safety_iterations):
        snapped = path.copy()
        for index, point in enumerate(path):
            row, column = occupancy_map.world_to_pixel(point)
            if (
                0 <= row < occupancy_map.height
                and 0 <= column < occupancy_map.width
                and safe[row, column]
            ):
                continue
            row_min = max(0, row - radius)
            row_max = min(occupancy_map.height, row + radius + 1)
            col_min = max(0, column - radius)
            col_max = min(occupancy_map.width, column + radius + 1)
            candidates = np.argwhere(safe[row_min:row_max, col_min:col_max])
            if len(candidates) == 0:
                raise MapTopologyError(
                    "centerline point ({:.3f}, {:.3f}) cannot reach safe space".format(
                        point[0], point[1]
                    )
                )
            candidates += np.asarray([row_min, col_min])
            distances = (
                (candidates[:, 0] - row) ** 2
                + (candidates[:, 1] - column) ** 2
            )
            best_row, best_column = candidates[int(np.argmin(distances))]
            replacement = occupancy_map.pixel_to_world(best_row, best_column)
            snap_distance = float(np.linalg.norm(replacement - point))
            if snap_distance > arguments.maximum_centerline_snap + occupancy_map.resolution:
                raise MapTopologyError("centerline requires an excessive safety correction")
            maximum_snap = max(maximum_snap, snap_distance)
            snapped[index] = replacement
        path = periodic_gaussian(
            snapped,
            arguments.safety_smoothing / arguments.centerline_sample_spacing,
        )
        path, _ = resample_closed(path, arguments.centerline_sample_spacing)

    clearances = np.asarray([occupancy_map.clearance_at(point) for point in path])
    if float(clearances.min()) + 1e-6 < arguments.minimum_centerline_clearance:
        raise MapTopologyError(
            "smoothed road centerline does not satisfy {:.3f} m clearance; "
            "minimum is {:.3f} m".format(
                arguments.minimum_centerline_clearance, float(clearances.min())
            )
        )
    return path, clearances, maximum_snap


def load_localizations(path, semantic_metadata):
    if path is None:
        return None
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if document.get("schema_version") != 1:
        raise MapTopologyError("unsupported Chinese localization schema")
    expected_digest = document.get("semantic_metadata_sha256")
    if expected_digest and expected_digest != file_sha256(semantic_metadata):
        raise MapTopologyError("Chinese localization belongs to different semantics")
    translations = document.get("translations")
    if not isinstance(translations, list):
        raise MapTopologyError("Chinese localization has no translations list")
    lookup = {}
    for item in translations:
        if not isinstance(item, dict):
            raise MapTopologyError("Chinese localization contains an invalid item")
        key = (str(item.get("source_caption", "")), str(item.get("source_category", "")))
        caption = str(item.get("caption_zh", "")).strip()
        category = str(item.get("category_zh", "")).strip()
        if not all(key) or not contains_chinese(caption) or not contains_chinese(category):
            raise MapTopologyError("every localized caption and category must be Chinese")
        if key in lookup and lookup[key] != (caption, category):
            raise MapTopologyError("Chinese localization contains a conflicting key")
        lookup[key] = (caption, category)
    return lookup


def uses_embedded_chinese_semantics(document):
    return (
        int(document.get("schema_version", 0)) >= 2
        and document.get("language") == "zh-CN"
    )


def validated_semantic_embedding(item):
    embedding = item.get("semantic_embedding")
    if embedding is None:
        return None
    if not isinstance(embedding, dict):
        raise MapTopologyError("semantic embedding must be an object")
    model = str(embedding.get("model", "")).strip()
    provider = str(embedding.get("provider", "")).strip()
    dimensions = embedding.get("dimensions")
    vector = embedding.get("vector")
    if (
        provider != "ollama_local"
        or not model
        or not isinstance(dimensions, int)
        or dimensions < 64
        or not isinstance(vector, list)
        or len(vector) != dimensions
        or any(
            not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in vector
        )
    ):
        raise MapTopologyError("semantic embedding is invalid")
    norm = math.sqrt(sum(float(value) ** 2 for value in vector))
    if not 0.99 <= norm <= 1.01:
        raise MapTopologyError("semantic embedding must be normalized")
    return {
        "provider": provider,
        "model": model,
        "dimensions": dimensions,
        "text_sha256": str(embedding.get("text_sha256", "")),
        "vector": [float(value) for value in vector],
    }


def direct_landmark_semantics(item):
    if (
        item.get("landmark_usable") is not True
        or item.get("is_static") is not True
        or item.get("is_drivable_surface") is True
    ):
        return None
    caption = str(item.get("caption_zh", item.get("caption", ""))).strip()
    category = str(item.get("category_zh", "")).strip()
    confidence = item.get("semantic_confidence")
    if (
        not contains_chinese(caption)
        or not contains_chinese(category)
        or not isinstance(confidence, (int, float))
        or not math.isfinite(confidence)
        or not 0.0 <= float(confidence) <= 1.0
    ):
        raise MapTopologyError("promoted landmark has invalid Chinese semantics")
    return caption, category, float(confidence)


def semantic_radius(item):
    extent = np.sort(
        np.asarray(item.get("extent", [0.0, 0.0, 0.0]), dtype=np.float64)
    )
    return 0.5 * float(np.linalg.norm(extent[-2:]))


def build_graph(arguments):
    occupancy_map = OccupancyMap(arguments.map_yaml)
    boundary_map = OccupancyMap(arguments.road_boundary_map_yaml)
    occupancy_map.assert_same_geometry(boundary_map)
    centerline, corridor = extract_road_centerline(boundary_map, arguments)
    centerline, centerline_clearance, maximum_snap = snap_path_to_safe_space(
        centerline, occupancy_map, corridor, arguments
    )
    centerline, centerline_length = resample_closed(
        centerline, arguments.centerline_sample_spacing
    )

    place_count = max(3, int(round(centerline_length / arguments.place_spacing)))
    actual_spacing = centerline_length / place_count
    place_distances = np.arange(place_count, dtype=np.float64) * actual_spacing
    place_positions, _ = interpolate_closed(centerline, place_distances)
    before, _ = interpolate_closed(
        centerline, place_distances - arguments.centerline_sample_spacing
    )
    after, _ = interpolate_closed(
        centerline, place_distances + arguments.centerline_sample_spacing
    )
    yaws = np.arctan2(after[:, 1] - before[:, 1], after[:, 0] - before[:, 0])

    semantic_document = json.loads(
        Path(arguments.semantic_metadata).read_text(encoding="utf-8")
    )
    semantic_objects = semantic_document.get("objects")
    if not isinstance(semantic_objects, list):
        raise MapTopologyError("semantic metadata must contain an objects list")
    direct_semantics = uses_embedded_chinese_semantics(semantic_document)
    localization = load_localizations(
        getattr(arguments, "landmark_localization", None),
        arguments.semantic_metadata,
    )
    if direct_semantics and localization is not None:
        raise MapTopologyError(
            "local-model semantics must not be translated a second time"
        )
    if not direct_semantics and localization is None:
        raise MapTopologyError(
            "legacy semantic metadata requires --landmark-localization"
        )
    road_tags = set(arguments.drivable_semantic_tags)
    road_objects = [
        item
        for item in semantic_objects
        if (
            item.get("is_drivable_surface") is True
            if direct_semantics
            else str(item.get("legacy_semantickitti_tag", "")) in road_tags
        )
    ]

    places = []
    for index, (position, yaw) in enumerate(zip(place_positions, yaws)):
        road_support = []
        for item in road_objects:
            center = np.asarray(item.get("center", [math.nan, math.nan]))[:2]
            if not np.isfinite(center).all():
                continue
            distance = max(
                0.0, float(np.linalg.norm(position - center)) - semantic_radius(item)
            )
            if distance <= arguments.road_support_distance:
                road_support.append((distance, int(item["id"])))
        road_support.sort()
        places.append(
            {
                "id": "place_{:03d}".format(index),
                "name": "道路地点{:03d}".format(index),
                "kind": "corridor",
                "position": {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": 0.0,
                },
                "yaw": float(yaw),
                "clearance_m": occupancy_map.clearance_at(position),
                "distance_along_centerline_m": float(place_distances[index]),
                "road_semantic_ids": [item_id for _, item_id in road_support[:8]],
                "landmark_ids": [],
                "semantic_summary": [],
                "topology_evidence": "two_dimensional_map_road_centerline",
            }
        )

    landmarks = []
    for item in semantic_objects:
        source_category = str(item.get("legacy_semantickitti_tag", "unknown"))
        if (
            item.get("is_drivable_surface") is True
            if direct_semantics
            else source_category in road_tags
        ):
            continue
        if int(item.get("num_detections", 0)) < arguments.minimum_landmark_detections:
            continue
        direct_values = direct_landmark_semantics(item) if direct_semantics else None
        if direct_semantics and direct_values is None:
            continue
        center = np.asarray(item.get("center", []), dtype=np.float64)
        if center.shape != (3,) or not np.isfinite(center).all():
            continue
        route_distances = np.linalg.norm(centerline - center[:2], axis=1)
        distance_to_route = float(route_distances.min())
        if distance_to_route > arguments.landmark_attach_radius:
            continue
        source_caption = str(item.get("source_caption", item.get("caption", "object")))
        if direct_semantics:
            caption, category, semantic_confidence = direct_values
        else:
            localized = localization.get((source_caption, source_category))
            if localized is None:
                raise MapTopologyError(
                    "missing Chinese localization for {!r} / {!r}".format(
                        source_caption, source_category
                    )
                )
            caption, category = localized
            semantic_confidence = None
        distances = np.linalg.norm(place_positions - center[:2], axis=1)
        nearest_index = int(np.argmin(distances))
        landmark = {
                "id": "landmark_{:04d}".format(int(item["id"])),
                "semantic_object_id": int(item["id"]),
                "caption": caption,
                "category": category,
                "language": "zh-CN",
                "source_caption_en": source_caption,
                "source_category_en": str(
                    item.get("source_category", source_category)
                ),
                "position": {
                    "x": float(center[0]),
                    "y": float(center[1]),
                    "z": float(center[2]),
                },
                "num_detections": int(item.get("num_detections", 0)),
                "caption_consensus_ratio": float(
                    item.get("caption_consensus_ratio", 0.0)
                ),
                "distance_to_route_m": distance_to_route,
                "nearest_place": places[nearest_index]["id"],
                "distance_to_place_m": float(distances[nearest_index]),
            }
        if direct_semantics:
            landmark.update(
                {
                    "semantic_confidence": semantic_confidence,
                    "semantic_source": str(item.get("semantic_source", "")),
                    "visible_evidence": list(item.get("visible_evidence", [])),
                    "is_static": True,
                }
            )
            embedding = validated_semantic_embedding(item)
            if embedding is not None:
                landmark["semantic_embedding"] = embedding
        landmarks.append(landmark)

    by_place = defaultdict(list)
    for landmark in landmarks:
        by_place[landmark["nearest_place"]].append(landmark)
    for place in places:
        attached = sorted(
            by_place[place["id"]],
            key=lambda item: (
                item["distance_to_place_m"],
                -item["num_detections"],
                item["id"],
            ),
        )
        place["landmark_ids"] = [item["id"] for item in attached]
        summaries = []
        for landmark in sorted(
            attached,
            key=lambda item: (
                -item["num_detections"],
                item["distance_to_place_m"],
                item["id"],
            ),
        ):
            if landmark["caption"] not in summaries:
                summaries.append(landmark["caption"])
            if len(summaries) >= arguments.maximum_place_summaries:
                break
        place["semantic_summary"] = summaries

    connections = []
    path_sample_spacing = min(0.5, actual_spacing / 10.0)
    for index in range(place_count):
        start_distance = index * actual_spacing
        end_distance = (index + 1) * actual_spacing
        samples = np.arange(start_distance, end_distance, path_sample_spacing)
        samples = np.append(samples, end_distance)
        geometry, _ = interpolate_closed(centerline, samples)
        minimum_clearance = min(
            occupancy_map.clearance_at(point) for point in geometry
        )
        source = places[index]
        target = places[(index + 1) % place_count]
        connections.append(
            {
                "id": "connection_{:03d}".format(index),
                "kind": "drivable",
                "source": source["id"],
                "target": target["id"],
                "length_m": float(actual_spacing),
                "minimum_clearance_m": float(minimum_clearance),
                "road_semantic_coverage_ratio": float(
                    (bool(source["road_semantic_ids"]) + bool(target["road_semantic_ids"]))
                    / 2.0
                ),
                "bidirectional": True,
                "executable": True,
                "evidence": "two_dimensional_map_road_centerline",
                "centerline": [
                    {"x": float(point[0]), "y": float(point[1])}
                    for point in geometry
                ],
            }
        )
    for index, landmark in enumerate(sorted(landmarks, key=lambda item: item["id"])):
        connections.append(
            {
                "id": "landmark_connection_{:04d}".format(index),
                "kind": "semantic_association",
                "source": landmark["nearest_place"],
                "target": landmark["id"],
                "length_m": float(landmark["distance_to_place_m"]),
                "bidirectional": True,
                "executable": False,
                "evidence": "nearest_uniform_map_place",
            }
        )

    unsafe_connections = sum(
        item["kind"] == "drivable"
        and item["minimum_clearance_m"] < arguments.minimum_centerline_clearance
        for item in connections
    )
    if unsafe_connections:
        raise MapTopologyError(
            "{} centerline connections fail the clearance contract".format(
                unsafe_connections
            )
        )
    result = {
        "schema_version": 3,
        "frame_id": str(semantic_document.get("frame_id", "map")),
        "language": "zh-CN",
        "source": {
            "map_yaml": str(occupancy_map.yaml_path),
            "map_image": str(occupancy_map.image_path),
            "road_boundary_map_yaml": str(boundary_map.yaml_path),
            "road_boundary_map_image": str(boundary_map.image_path),
            "semantic_metadata": str(Path(arguments.semantic_metadata).resolve()),
            "topology_source": "two_dimensional_map_road_centerline",
            "sha256": {
                "map_yaml": file_sha256(occupancy_map.yaml_path),
                "map_image": file_sha256(occupancy_map.image_path),
                "road_boundary_map_yaml": file_sha256(boundary_map.yaml_path),
                "road_boundary_map_image": file_sha256(boundary_map.image_path),
                "semantic_metadata": file_sha256(arguments.semantic_metadata),
            },
        },
        "parameters": {
            "place_spacing_m": float(actual_spacing),
            "minimum_centerline_clearance_m": arguments.minimum_centerline_clearance,
            "safety_clearance_margin_m": arguments.safety_clearance_margin,
            "centerline_sample_spacing_m": arguments.centerline_sample_spacing,
            "boundary_smoothing_m": arguments.boundary_smoothing,
            "centerline_smoothing_m": arguments.centerline_smoothing,
            "start_position": [float(value) for value in arguments.start_position],
            "start_yaw_deg": arguments.start_yaw_deg,
            "drivable_semantic_tags": sorted(road_tags),
            "semantic_mode": (
                "ollama_chinese_instances" if direct_semantics else "legacy_localization"
            ),
        },
        "statistics": {
            "centerline_length_m": float(centerline_length),
            "centerline_samples": int(len(centerline)),
            "place_spacing_m": float(actual_spacing),
            "maximum_centerline_snap_m": float(maximum_snap),
            "minimum_centerline_clearance_m": float(centerline_clearance.min()),
            "unsafe_connections": int(unsafe_connections),
            "road_semantic_objects": len(road_objects),
            "landmarks": len(landmarks),
            "semantic_nodes": len(places) + len(landmarks),
            "places": len(places),
            "connections": len(connections),
            "drivable_connections": place_count,
            "semantic_associations": len(landmarks),
        },
        "landmarks": landmarks,
        "places": places,
        "connections": connections,
    }
    if localization is not None:
        localization_path = Path(arguments.landmark_localization).resolve()
        result["source"]["landmark_localization"] = str(localization_path)
        result["source"]["sha256"]["landmark_localization"] = file_sha256(
            localization_path
        )
    return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--road-boundary-map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-metadata", required=True, type=Path)
    parser.add_argument("--landmark-localization", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--start-position", nargs=2, type=float, default=[0.0, 0.0])
    parser.add_argument("--start-yaw-deg", type=float, default=90.0)
    parser.add_argument("--place-spacing", type=float, default=10.0)
    parser.add_argument("--boundary-sample-spacing", type=float, default=0.25)
    parser.add_argument("--boundary-smoothing", type=float, default=2.0)
    parser.add_argument("--centerline-sample-spacing", type=float, default=0.25)
    parser.add_argument("--centerline-smoothing", type=float, default=1.0)
    parser.add_argument("--safety-smoothing", type=float, default=0.25)
    parser.add_argument("--safety-iterations", type=int, default=3)
    parser.add_argument("--middle-band-pixels", type=float, default=1.5)
    parser.add_argument("--minimum-boundary-pixels", type=int, default=100)
    parser.add_argument("--minimum-centerline-clearance", type=float, default=0.5)
    parser.add_argument("--safety-clearance-margin", type=float, default=0.15)
    parser.add_argument("--maximum-centerline-snap", type=float, default=1.5)
    parser.add_argument("--minimum-landmark-detections", type=int, default=10)
    parser.add_argument("--landmark-attach-radius", type=float, default=20.0)
    parser.add_argument("--road-support-distance", type=float, default=2.0)
    parser.add_argument(
        "--drivable-semantic-tags", nargs="+", default=["road", "parking"]
    )
    parser.add_argument("--maximum-place-summaries", type=int, default=5)
    return parser.parse_args()


def validate_arguments(arguments):
    positive = [
        arguments.place_spacing,
        arguments.boundary_sample_spacing,
        arguments.boundary_smoothing,
        arguments.centerline_sample_spacing,
        arguments.centerline_smoothing,
        arguments.safety_smoothing,
        arguments.middle_band_pixels,
        arguments.minimum_centerline_clearance,
        arguments.maximum_centerline_snap,
        arguments.landmark_attach_radius,
        arguments.road_support_distance,
    ]
    if any(not math.isfinite(value) or value <= 0.0 for value in positive):
        raise MapTopologyError("distance parameters must be finite and positive")
    if arguments.safety_iterations < 1:
        raise MapTopologyError("safety iterations must be positive")
    if arguments.minimum_boundary_pixels < 10:
        raise MapTopologyError("minimum boundary pixels is too small")
    if arguments.minimum_landmark_detections < 1:
        raise MapTopologyError("minimum landmark detections must be positive")
    if arguments.maximum_place_summaries < 1:
        raise MapTopologyError("maximum place summaries must be positive")
    if (
        not math.isfinite(arguments.safety_clearance_margin)
        or arguments.safety_clearance_margin < 0.0
    ):
        raise MapTopologyError("safety clearance margin must be non-negative")


def main():
    arguments = parse_args()
    validate_arguments(arguments)
    graph = build_graph(arguments)
    output = arguments.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(graph, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(output)
    statistics = graph["statistics"]
    print(
        "Generated a {:.2f} m closed map centerline with {} uniformly spaced "
        "Chinese places, {} Chinese landmarks and {} nearest-place links.".format(
            statistics["centerline_length_m"],
            statistics["places"],
            statistics["landmarks"],
            statistics["semantic_associations"],
        )
    )
    print("Saved map-derived semantic navigation graph to {}".format(output))


if __name__ == "__main__":
    try:
        main()
    except (MapTopologyError, OSError, ValueError, json.JSONDecodeError) as error:
        raise SystemExit("error: {}".format(error)) from error
