#!/usr/bin/env python3

"""Build a semantic navigation graph from a driven trajectory and map."""

import argparse
import hashlib
import json
import math
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
import yaml


class GraphBuildError(RuntimeError):
    pass


class OccupancyMap:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).resolve()
        document = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            raise GraphBuildError("map YAML must contain a mapping")

        image_path = Path(str(document["image"]))
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        self.image_path = image_path.resolve()
        self.image = cv2.imread(str(self.image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise GraphBuildError("failed to read map image: {}".format(self.image_path))

        self.resolution = float(document["resolution"])
        origin = np.asarray(document["origin"], dtype=np.float64)
        if self.resolution <= 0.0 or origin.shape != (3,):
            raise GraphBuildError("map resolution and origin are invalid")
        self.origin = origin
        self.height, self.width = self.image.shape
        yaw = float(origin[2])
        self.rotation = np.asarray(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
            dtype=np.float64,
        )

        negate = bool(int(document.get("negate", 0)))
        normalized = self.image.astype(np.float64) / 255.0
        occupancy = normalized if negate else 1.0 - normalized
        self.free = occupancy < float(document.get("free_thresh", 0.196))
        self.clearance = cv2.distanceTransform(
            self.free.astype(np.uint8), cv2.DIST_L2, 5
        ) * self.resolution

    def world_to_pixel(self, point):
        point = np.asarray(point, dtype=np.float64)
        local = self.rotation.T @ (point[:2] - self.origin[:2])
        column = int(math.floor(local[0] / self.resolution))
        row_from_bottom = int(math.floor(local[1] / self.resolution))
        row = self.height - 1 - row_from_bottom
        return row, column

    def pixel_to_world(self, row, column):
        local = np.asarray(
            [
                (float(column) + 0.5) * self.resolution,
                (float(self.height - 1 - row) + 0.5) * self.resolution,
            ]
        )
        return self.origin[:2] + self.rotation @ local

    def contains_pixel(self, row, column):
        return 0 <= row < self.height and 0 <= column < self.width

    def clearance_at(self, point):
        row, column = self.world_to_pixel(point)
        if not self.contains_pixel(row, column):
            return 0.0
        return float(self.clearance[row, column])

    def snap_to_clearance(self, point, minimum_clearance, maximum_distance):
        row, column = self.world_to_pixel(point)
        if self.contains_pixel(row, column):
            value = float(self.clearance[row, column])
            if value >= minimum_clearance:
                return np.asarray(point[:2], dtype=np.float64), value, 0.0

        radius = int(math.ceil(maximum_distance / self.resolution))
        row_min = max(0, row - radius)
        row_max = min(self.height, row + radius + 1)
        col_min = max(0, column - radius)
        col_max = min(self.width, column + radius + 1)
        if row_min >= row_max or col_min >= col_max:
            return None

        window = self.clearance[row_min:row_max, col_min:col_max]
        candidates = np.argwhere(window >= minimum_clearance)
        if len(candidates) == 0:
            return None
        absolute = candidates + np.asarray([row_min, col_min])
        pixel_distance_squared = (
            (absolute[:, 0] - row) ** 2 + (absolute[:, 1] - column) ** 2
        )
        index = int(np.argmin(pixel_distance_squared))
        best_row, best_column = absolute[index]
        snapped = self.pixel_to_world(int(best_row), int(best_column))
        distance = float(np.linalg.norm(snapped - np.asarray(point[:2])))
        if distance > maximum_distance + self.resolution:
            return None
        return (
            snapped,
            float(self.clearance[int(best_row), int(best_column)]),
            distance,
        )


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_trajectory(path):
    raw = np.loadtxt(path, dtype=np.float64)
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    if raw.shape[1] != 12:
        raise GraphBuildError("trajectory poses must contain 12 values per row")
    transforms = raw.reshape(-1, 3, 4)
    points = transforms[:, :2, 3]
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 2:
        raise GraphBuildError("trajectory contains fewer than two finite poses")
    step = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if float(step.max()) > 2.0:
        raise GraphBuildError("trajectory contains a discontinuity larger than 2 m")
    keep = np.concatenate(([True], step > 1e-4))
    return points[keep]


def resample_polyline(points, spacing):
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    if cumulative[-1] < spacing:
        return points[[0, -1]].copy(), np.asarray([0.0, cumulative[-1]])
    targets = np.arange(0.0, cumulative[-1], spacing)
    if cumulative[-1] - targets[-1] > 1e-6:
        targets = np.append(targets, cumulative[-1])
    sampled = np.column_stack(
        (
            np.interp(targets, cumulative, points[:, 0]),
            np.interp(targets, cumulative, points[:, 1]),
        )
    )
    return sampled, targets


class DisjointSet:
    def __init__(self, size):
        self.parent = list(range(size))
        self.rank = [0] * size

    def find(self, item):
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, first, second):
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root == second_root:
            return
        if self.rank[first_root] < self.rank[second_root]:
            first_root, second_root = second_root, first_root
        self.parent[second_root] = first_root
        if self.rank[first_root] == self.rank[second_root]:
            self.rank[first_root] += 1


def merge_nearby_samples(points, radius):
    if radius <= 0.0:
        return np.arange(len(points), dtype=np.int64), [np.asarray([i]) for i in range(len(points))]
    groups = DisjointSet(len(points))
    buckets = defaultdict(list)
    for index, point in enumerate(points):
        cell = tuple(np.floor(point / radius).astype(np.int64))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for candidate in buckets[(cell[0] + dx, cell[1] + dy)]:
                    if np.linalg.norm(point - points[candidate]) <= radius:
                        groups.union(index, candidate)
        buckets[cell].append(index)

    by_root = defaultdict(list)
    for index in range(len(points)):
        by_root[groups.find(index)].append(index)
    clusters = sorted(
        (np.asarray(indices, dtype=np.int64) for indices in by_root.values()),
        key=lambda indices: int(indices.min()),
    )
    sample_to_cluster = np.empty(len(points), dtype=np.int64)
    for cluster_id, indices in enumerate(clusters):
        sample_to_cluster[indices] = cluster_id
    return sample_to_cluster, clusters


def segment_minimum_clearance(occupancy_map, first, second):
    length = float(np.linalg.norm(second - first))
    count = max(2, int(math.ceil(length / (0.5 * occupancy_map.resolution))) + 1)
    values = [
        occupancy_map.clearance_at(first + ratio * (second - first))
        for ratio in np.linspace(0.0, 1.0, count)
    ]
    return float(min(values))


def graph_components(node_count, edges):
    adjacency = [[] for _ in range(node_count)]
    for edge in edges:
        adjacency[edge[0]].append(edge[1])
        adjacency[edge[1]].append(edge[0])
    unseen = set(range(node_count))
    components = []
    while unseen:
        start = next(iter(unseen))
        queue = deque([start])
        unseen.remove(start)
        component = []
        while queue:
            node = queue.popleft()
            component.append(node)
            for neighbor in adjacency[node]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def node_tangent(sample_index, sample_to_node, node_positions):
    current = int(sample_to_node[sample_index])
    previous = sample_index - 1
    while previous >= 0 and int(sample_to_node[previous]) == current:
        previous -= 1
    following = sample_index + 1
    while following < len(sample_to_node) and int(sample_to_node[following]) == current:
        following += 1
    if previous >= 0 and following < len(sample_to_node):
        vector = node_positions[int(sample_to_node[following])] - node_positions[int(sample_to_node[previous])]
    elif following < len(sample_to_node):
        vector = node_positions[int(sample_to_node[following])] - node_positions[current]
    elif previous >= 0:
        vector = node_positions[current] - node_positions[int(sample_to_node[previous])]
    else:
        vector = np.asarray([1.0, 0.0])
    return float(math.atan2(vector[1], vector[0]))


def build_place_connections(places, place_node_indices, nodes, adjacency, route_edges):
    if len(places) <= 1:
        return []
    node_lookup = {node["id"]: index for index, node in enumerate(nodes)}
    place_at_node = {
        route_node_index: place["id"]
        for place, route_node_index in zip(places, place_node_indices)
    }
    route_edge_at_pair = {}
    for edge in route_edges:
        source = node_lookup[edge["source"]]
        target = node_lookup[edge["target"]]
        route_edge_at_pair[tuple(sorted((source, target)))] = edge

    contracted = {}
    for start_node, source_place in sorted(place_at_node.items()):
        for first_neighbor in sorted(adjacency[start_node]):
            previous = start_node
            current = first_neighbor
            traversed_nodes = [start_node, current]
            traversed_edges = [
                route_edge_at_pair[tuple(sorted((start_node, current)))]
            ]
            visited = {start_node}
            while current not in place_at_node:
                if current in visited:
                    raise GraphBuildError(
                        "a corridor loops without reaching another place node"
                    )
                visited.add(current)
                following = [item for item in adjacency[current] if item != previous]
                if len(following) != 1:
                    raise GraphBuildError(
                        "a non-place route node is not a simple corridor"
                    )
                following_node = following[0]
                traversed_edges.append(
                    route_edge_at_pair[tuple(sorted((current, following_node)))]
                )
                previous, current = current, following_node
                traversed_nodes.append(current)

            target_place = place_at_node[current]
            if target_place == source_place:
                continue
            source, target = sorted((source_place, target_place))
            if source != source_place:
                traversed_nodes.reverse()
                traversed_edges.reverse()
            candidate = {
                "kind": "drivable",
                "source": source,
                "target": target,
                "length_m": float(
                    sum(edge["length_m"] for edge in traversed_edges)
                ),
                "minimum_clearance_m": float(
                    min(edge["minimum_clearance_m"] for edge in traversed_edges)
                ),
                "road_semantic_coverage_ratio": (
                    sum(bool(nodes[index]["road_semantic_ids"]) for index in traversed_nodes)
                    / len(traversed_nodes)
                ),
                "bidirectional": True,
                "executable": True,
                "evidence": "contracted_recorded_vehicle_trajectory",
            }
            corridor_key = tuple(
                sorted(edge["id"] for edge in traversed_edges)
            )
            existing = contracted.get(corridor_key)
            if existing is None or candidate["length_m"] < existing["length_m"]:
                contracted[corridor_key] = candidate

    connections = []
    ordered = sorted(
        contracted.values(),
        key=lambda item: (
            item["source"],
            item["target"],
            item["length_m"],
        ),
    )
    for edge_index, item in enumerate(ordered):
        item["id"] = "connection_{:03d}".format(edge_index)
        connections.append(item)

    place_index = {place["id"]: index for index, place in enumerate(places)}
    components = graph_components(
        len(places),
        [
            (place_index[edge["source"]], place_index[edge["target"]])
            for edge in connections
        ],
    )
    if len(components) != 1:
        raise GraphBuildError("semantic place graph is not connected")
    return connections


def semantic_radius(item):
    extent = np.sort(np.asarray(item.get("extent", [0.0, 0.0, 0.0]), dtype=np.float64))
    return 0.5 * float(np.linalg.norm(extent[-2:]))


def load_semantics(path):
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    objects = document.get("objects")
    if not isinstance(objects, list):
        raise GraphBuildError("semantic metadata must contain an objects list")
    return document, objects


def build_navigation_graph(arguments):
    occupancy_map = OccupancyMap(arguments.map_yaml)
    trajectory = load_trajectory(arguments.trajectory_poses)
    samples, trajectory_distance = resample_polyline(
        trajectory, arguments.sample_spacing
    )

    snapped_samples = []
    snap_distances = []
    for sample in samples:
        result = occupancy_map.snap_to_clearance(
            sample, arguments.minimum_clearance, arguments.maximum_snap_distance
        )
        if result is None:
            raise GraphBuildError(
                "trajectory sample at ({:.3f}, {:.3f}) cannot be snapped to free space".format(
                    sample[0], sample[1]
                )
            )
        snapped_samples.append(result[0])
        snap_distances.append(result[2])
    snapped_samples = np.asarray(snapped_samples)

    sample_to_node, clusters = merge_nearby_samples(
        snapped_samples, arguments.merge_radius
    )
    node_positions = []
    node_clearance = []
    for cluster in clusters:
        center = snapped_samples[cluster].mean(axis=0)
        snapped = occupancy_map.snap_to_clearance(
            center, arguments.minimum_clearance, arguments.maximum_snap_distance
        )
        if snapped is None:
            raise GraphBuildError("a merged route node cannot be placed in free space")
        node_positions.append(snapped[0])
        node_clearance.append(snapped[1])
    node_positions = np.asarray(node_positions)

    edge_support = defaultdict(int)
    for first, second in zip(sample_to_node[:-1], sample_to_node[1:]):
        first = int(first)
        second = int(second)
        if first == second:
            continue
        edge_support[tuple(sorted((first, second)))] += 1
    edge_pairs = sorted(edge_support)
    components = graph_components(len(node_positions), edge_pairs)
    if len(components) != 1:
        raise GraphBuildError("generated route graph is not connected")

    adjacency = [[] for _ in range(len(node_positions))]
    edges = []
    for edge_id, (first, second) in enumerate(edge_pairs):
        adjacency[first].append(second)
        adjacency[second].append(first)
        length = float(np.linalg.norm(node_positions[second] - node_positions[first]))
        edges.append(
            {
                "id": "edge_{:04d}".format(edge_id),
                "source": "route_{:04d}".format(first),
                "target": "route_{:04d}".format(second),
                "length_m": length,
                "minimum_clearance_m": segment_minimum_clearance(
                    occupancy_map, node_positions[first], node_positions[second]
                ),
                "trajectory_traversals": int(edge_support[(first, second)]),
                "bidirectional": True,
                "evidence": "recorded_vehicle_trajectory",
            }
        )

    first_sample = [int(cluster.min()) for cluster in clusters]
    nodes = []
    for node_id, position in enumerate(node_positions):
        degree = len(adjacency[node_id])
        kind = "junction" if degree >= 3 else "terminal" if degree <= 1 else "corridor"
        nodes.append(
            {
                "id": "route_{:04d}".format(node_id),
                "kind": kind,
                "position": {"x": float(position[0]), "y": float(position[1]), "z": 0.0},
                "yaw": node_tangent(
                    first_sample[node_id], sample_to_node, node_positions
                ),
                "degree": degree,
                "clearance_m": float(node_clearance[node_id]),
                "trajectory_distance_m": float(trajectory_distance[first_sample[node_id]]),
                "road_semantic_ids": [],
                "landmark_ids": [],
            }
        )

    semantic_document, semantic_objects = load_semantics(arguments.semantic_metadata)
    road_tags = set(arguments.drivable_semantic_tags)
    road_objects = [
        item
        for item in semantic_objects
        if str(item.get("legacy_semantickitti_tag", "")) in road_tags
    ]
    supported_nodes = 0
    for node_index, node in enumerate(nodes):
        point = node_positions[node_index]
        support = []
        for item in road_objects:
            center = np.asarray(item["center"], dtype=np.float64)[:2]
            distance = max(0.0, float(np.linalg.norm(point - center)) - semantic_radius(item))
            if distance <= arguments.road_support_distance:
                support.append((distance, int(item["id"])))
        support.sort()
        node["road_semantic_ids"] = [item_id for _, item_id in support[:8]]
        if support:
            supported_nodes += 1

    landmarks = []
    for item in semantic_objects:
        if str(item.get("legacy_semantickitti_tag", "")) in road_tags:
            continue
        if int(item.get("num_detections", 0)) < arguments.minimum_landmark_detections:
            continue
        center = np.asarray(item["center"], dtype=np.float64)[:2]
        distances = np.linalg.norm(node_positions - center, axis=1)
        route_node = int(np.argmin(distances))
        route_distance = float(distances[route_node])
        if route_distance > arguments.landmark_attach_radius:
            continue
        landmark_id = "landmark_{:04d}".format(int(item["id"]))
        landmark = {
            "id": landmark_id,
            "semantic_object_id": int(item["id"]),
            "caption": str(item.get("caption", "object")),
            "category": str(item.get("legacy_semantickitti_tag", "unknown")),
            "position": {
                "x": float(center[0]),
                "y": float(center[1]),
                "z": float(item["center"][2]),
            },
            "num_detections": int(item.get("num_detections", 0)),
            "caption_consensus_ratio": float(item.get("caption_consensus_ratio", 0.0)),
            "distance_to_route_m": route_distance,
        }
        landmarks.append(landmark)

    selected_place_nodes = {
        index for index, neighbors in enumerate(adjacency) if len(neighbors) != 2
    }
    last_place_distance = -arguments.place_spacing
    for sample_index, distance in enumerate(trajectory_distance):
        route_node = int(sample_to_node[sample_index])
        if (
            distance - last_place_distance >= arguments.place_spacing
            and route_node not in selected_place_nodes
        ):
            selected_place_nodes.add(route_node)
            last_place_distance = float(distance)

    places = []
    place_node_indices = []
    for place_index, route_node in enumerate(sorted(selected_place_nodes)):
        position = node_positions[route_node]
        nearby = []
        for landmark in landmarks:
            landmark_position = np.asarray(
                [landmark["position"]["x"], landmark["position"]["y"]]
            )
            distance = float(np.linalg.norm(landmark_position - position))
            if distance <= arguments.place_landmark_radius:
                nearby.append(
                    (-landmark["num_detections"], distance, landmark["id"], landmark["caption"])
                )
        nearby.sort()
        nearby = nearby[: arguments.maximum_place_landmarks]
        labels = []
        for _, _, _, caption in nearby:
            if caption not in labels:
                labels.append(caption)
            if len(labels) >= 5:
                break
        places.append(
            {
                "id": "place_{:03d}".format(place_index),
                "kind": nodes[route_node]["kind"],
                "position": nodes[route_node]["position"],
                "yaw": nodes[route_node]["yaw"],
                "clearance_m": nodes[route_node]["clearance_m"],
                "trajectory_distance_m": nodes[route_node]["trajectory_distance_m"],
                "road_semantic_ids": nodes[route_node]["road_semantic_ids"],
                "landmark_ids": [item[2] for item in nearby],
                "semantic_summary": labels,
            }
        )
        place_node_indices.append(route_node)

    place_positions = np.asarray(
        [[place["position"]["x"], place["position"]["y"]] for place in places]
    )
    for landmark in landmarks:
        position = np.asarray(
            [landmark["position"]["x"], landmark["position"]["y"]]
        )
        distances = np.linalg.norm(place_positions - position, axis=1)
        nearest_place = int(np.argmin(distances))
        landmark["nearest_place"] = places[nearest_place]["id"]
        landmark["distance_to_place_m"] = float(distances[nearest_place])

    connections = build_place_connections(
        places, place_node_indices, nodes, adjacency, edges
    )
    for connection_index, landmark in enumerate(
        sorted(landmarks, key=lambda item: item["id"])
    ):
        connections.append(
            {
                "id": "landmark_connection_{:04d}".format(connection_index),
                "kind": "semantic_association",
                "source": landmark["nearest_place"],
                "target": landmark["id"],
                "length_m": float(landmark["distance_to_place_m"]),
                "bidirectional": True,
                "executable": False,
                "evidence": "nearest_place_semantic_association",
            }
        )
    unsafe_connections = sum(
        connection["minimum_clearance_m"] < arguments.minimum_edge_clearance
        for connection in connections
        if connection["kind"] == "drivable"
    )
    result = {
        "schema_version": 3,
        "frame_id": str(semantic_document.get("frame_id", "map")),
        "source": {
            "map_yaml": str(occupancy_map.yaml_path),
            "map_image": str(occupancy_map.image_path),
            "semantic_metadata": str(Path(arguments.semantic_metadata).resolve()),
            "trajectory_poses": str(Path(arguments.trajectory_poses).resolve()),
            "sha256": {
                "map_yaml": file_sha256(occupancy_map.yaml_path),
                "map_image": file_sha256(occupancy_map.image_path),
                "semantic_metadata": file_sha256(arguments.semantic_metadata),
                "trajectory_poses": file_sha256(arguments.trajectory_poses),
            },
        },
        "parameters": {
            "sample_spacing_m": arguments.sample_spacing,
            "merge_radius_m": arguments.merge_radius,
            "minimum_clearance_m": arguments.minimum_clearance,
            "minimum_edge_clearance_m": arguments.minimum_edge_clearance,
            "maximum_snap_distance_m": arguments.maximum_snap_distance,
            "road_support_distance_m": arguments.road_support_distance,
            "drivable_semantic_tags": sorted(road_tags),
            "place_spacing_m": arguments.place_spacing,
        },
        "statistics": {
            "trajectory_length_m": float(trajectory_distance[-1]),
            "trajectory_samples": int(len(samples)),
            "maximum_snap_distance_m": float(max(snap_distances)),
            "road_semantic_objects": len(road_objects),
            "road_semantic_coverage_ratio": supported_nodes / len(nodes),
            "unsafe_connections": int(unsafe_connections),
            "landmarks": len(landmarks),
            "semantic_nodes": len(places) + len(landmarks),
            "places": len(places),
            "connections": len(connections),
            "drivable_connections": sum(
                connection["kind"] == "drivable" for connection in connections
            ),
            "semantic_associations": sum(
                connection["kind"] == "semantic_association"
                for connection in connections
            ),
        },
        "landmarks": landmarks,
        "places": places,
        "connections": connections,
    }
    return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--semantic-metadata", required=True, type=Path)
    parser.add_argument("--trajectory-poses", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--sample-spacing", type=float, default=1.5)
    parser.add_argument("--merge-radius", type=float, default=0.9)
    parser.add_argument("--minimum-clearance", type=float, default=0.5)
    parser.add_argument("--minimum-edge-clearance", type=float, default=0.2)
    parser.add_argument("--maximum-snap-distance", type=float, default=1.0)
    parser.add_argument("--road-support-distance", type=float, default=2.0)
    parser.add_argument(
        "--drivable-semantic-tags", nargs="+", default=["road", "parking"]
    )
    parser.add_argument("--minimum-landmark-detections", type=int, default=10)
    parser.add_argument("--landmark-attach-radius", type=float, default=20.0)
    parser.add_argument("--place-spacing", type=float, default=10.0)
    parser.add_argument("--place-landmark-radius", type=float, default=8.0)
    parser.add_argument("--maximum-place-landmarks", type=int, default=8)
    return parser.parse_args()


def validate_arguments(arguments):
    positive = (
        arguments.sample_spacing,
        arguments.merge_radius,
        arguments.minimum_clearance,
        arguments.minimum_edge_clearance,
        arguments.maximum_snap_distance,
        arguments.road_support_distance,
        arguments.landmark_attach_radius,
        arguments.place_spacing,
        arguments.place_landmark_radius,
    )
    if any(value <= 0.0 for value in positive):
        raise GraphBuildError("distance parameters must be positive")
    if arguments.minimum_landmark_detections < 1 or arguments.maximum_place_landmarks < 1:
        raise GraphBuildError("landmark thresholds must be positive")


def main():
    arguments = parse_args()
    validate_arguments(arguments)
    result = build_navigation_graph(arguments)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(arguments.output)
    stats = result["statistics"]
    print(
        "Generated a semantic graph with {semantic_nodes} nodes: {places} places, "
        "{landmarks} landmarks, {drivable_connections} drivable connections and "
        "{semantic_associations} landmark associations; road semantic "
        "coverage {coverage:.1%}; unsafe connections {unsafe_connections}.".format(
            coverage=stats["road_semantic_coverage_ratio"], **stats
        )
    )
    print("Saved semantic navigation graph to {}".format(arguments.output))


if __name__ == "__main__":
    main()
