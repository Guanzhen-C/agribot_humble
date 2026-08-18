#!/usr/bin/env python3

"""Move recorded semantic places onto a planner-certified Smac path."""

import argparse
import hashlib
import heapq
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class TopologyRefinementError(RuntimeError):
    pass


def strict_json(path, description):
    def reject_constant(value):
        raise ValueError("non-finite JSON number: {}".format(value))

    def reject_duplicate_keys(pairs):
        document = {}
        for key, value in pairs:
            if key in document:
                raise ValueError("duplicate JSON key: {}".format(key))
            document[key] = value
        return document

    try:
        return json.loads(
            Path(path).read_text(encoding="utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (OSError, UnicodeDecodeError, ValueError) as error:
        raise TopologyRefinementError(
            "{} is not strict JSON: {}".format(description, error)
        ) from error


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_number(value, description):
    if isinstance(value, bool):
        raise TopologyRefinementError("{} must be finite".format(description))
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise TopologyRefinementError(
            "{} must be finite".format(description)
        ) from error
    if not math.isfinite(number):
        raise TopologyRefinementError("{} must be finite".format(description))
    return number


class OccupancyMap:
    def __init__(self, yaml_path):
        self.yaml_path = Path(yaml_path).expanduser().resolve()
        document = yaml.safe_load(self.yaml_path.read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            raise TopologyRefinementError("map YAML must contain a mapping")
        image_path = Path(str(document.get("image", "")))
        if not image_path.is_absolute():
            image_path = self.yaml_path.parent / image_path
        self.image_path = image_path.resolve()
        image = cv2.imread(str(self.image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise TopologyRefinementError(
                "failed to read map image: {}".format(self.image_path)
            )
        self.resolution = finite_number(document.get("resolution"), "map resolution")
        origin = np.asarray(document.get("origin"), dtype=np.float64)
        if self.resolution <= 0.0 or origin.shape != (3,):
            raise TopologyRefinementError("map resolution or origin is invalid")
        self.origin = origin
        self.height, self.width = image.shape
        yaw = float(origin[2])
        self.rotation = np.asarray(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
            dtype=np.float64,
        )
        negate = bool(int(document.get("negate", 0)))
        normalized = image.astype(np.float64) / 255.0
        occupancy = normalized if negate else 1.0 - normalized
        free = occupancy < float(document.get("free_thresh", 0.196))
        self.clearance = cv2.distanceTransform(
            free.astype(np.uint8), cv2.DIST_L2, 5
        ) * self.resolution

    def clearance_at(self, point):
        local = self.rotation.T @ (np.asarray(point[:2]) - self.origin[:2])
        column = int(math.floor(local[0] / self.resolution))
        row_from_bottom = int(math.floor(local[1] / self.resolution))
        row = self.height - 1 - row_from_bottom
        if not (0 <= row < self.height and 0 <= column < self.width):
            return 0.0
        return float(self.clearance[row, column])


def validate_graph_sources(graph, graph_path, occupancy_map):
    source = graph.get("source")
    digests = source.get("sha256") if isinstance(source, dict) else None
    if not isinstance(digests, dict):
        raise TopologyRefinementError("recorded semantic graph has no source digests")
    expected_graph_map = digests.get("map_yaml")
    if expected_graph_map is not None and expected_graph_map != file_sha256(
        occupancy_map.yaml_path
    ):
        raise TopologyRefinementError(
            "recorded semantic graph was built from a different map YAML"
        )
    expected_graph_image = digests.get("map_image")
    if expected_graph_image is not None and expected_graph_image != file_sha256(
        occupancy_map.image_path
    ):
        raise TopologyRefinementError(
            "recorded semantic graph was built from a different map image"
        )
    declared_graph = source.get("recorded_navigation_graph")
    if declared_graph and Path(declared_graph).expanduser().resolve() == graph_path:
        raise TopologyRefinementError(
            "input graph is already planner-refined; use its recorded source graph"
        )


def semantic_radius(item):
    extent = np.sort(
        np.asarray(item.get("extent", [0.0, 0.0, 0.0]), dtype=np.float64)
    )
    return 0.5 * float(np.linalg.norm(extent[-2:]))


def validate_reference_source(reference, graph_path, map_path):
    source = reference.get("source")
    if not isinstance(source, dict):
        raise TopologyRefinementError("reference path has no source contract")
    expected_map_digest = source.get("map_yaml_sha256")
    if expected_map_digest != file_sha256(map_path):
        raise TopologyRefinementError(
            "reference path was planned against a different map YAML"
        )
    route_path = Path(str(source.get("semantic_route", ""))).expanduser().resolve()
    if not route_path.is_file():
        raise TopologyRefinementError(
            "reference semantic route does not exist: {}".format(route_path)
        )
    if source.get("semantic_route_sha256") != file_sha256(route_path):
        raise TopologyRefinementError("reference semantic route digest has changed")
    route = strict_json(route_path, "reference semantic route")
    if route.get("graph_sha256") != file_sha256(graph_path):
        raise TopologyRefinementError(
            "reference path was planned from a different semantic graph"
        )


def load_reference_path(path, graph_path, map_path, frame_id):
    document = strict_json(path, "Smac reference path")
    if document.get("schema_version") != 1:
        raise TopologyRefinementError("unsupported Smac reference path schema")
    if str(document.get("frame_id", "")).lstrip("/") != frame_id.lstrip("/"):
        raise TopologyRefinementError("reference path and graph frames differ")
    if document.get("route_waypoint_mode") != "requested_stops":
        raise TopologyRefinementError(
            "reference path must be exported in requested_stops certification mode"
        )
    validate_reference_source(document, graph_path, map_path)
    raw_poses = document.get("poses")
    if not isinstance(raw_poses, list) or len(raw_poses) < 2:
        raise TopologyRefinementError("reference path has fewer than two poses")

    positions = []
    yaws = []
    for index, pose in enumerate(raw_poses):
        if not isinstance(pose, dict) or pose.get("index") != index:
            raise TopologyRefinementError(
                "reference path pose {} has an invalid index".format(index)
            )
        position = pose.get("position")
        if not isinstance(position, dict):
            raise TopologyRefinementError(
                "reference path pose {} has no position".format(index)
            )
        positions.append(
            [
                finite_number(position.get("x"), "reference pose x"),
                finite_number(position.get("y"), "reference pose y"),
            ]
        )
        yaws.append(finite_number(pose.get("yaw"), "reference pose yaw"))
    positions = np.asarray(positions, dtype=np.float64)
    steps = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    if float(steps.max()) > 1.0:
        raise TopologyRefinementError(
            "reference path contains a discontinuity larger than 1 m"
        )
    cumulative = np.concatenate(([0.0], np.cumsum(steps)))
    if cumulative[-1] <= 0.0:
        raise TopologyRefinementError("reference path has zero length")
    return document, positions, np.unwrap(np.asarray(yaws)), cumulative


def index_drivable_graph(graph):
    places = graph.get("places")
    connections = graph.get("connections")
    if graph.get("schema_version") != 3 or not isinstance(places, list):
        raise TopologyRefinementError("semantic graph must use schema version 3")
    if not isinstance(connections, list):
        raise TopologyRefinementError("semantic graph connections must be a list")
    place_by_id = {}
    for place in places:
        place_id = place.get("id") if isinstance(place, dict) else None
        if not isinstance(place_id, str) or not place_id or place_id in place_by_id:
            raise TopologyRefinementError("semantic graph has an invalid place id")
        place_by_id[place_id] = place
    adjacency = {place_id: [] for place_id in place_by_id}
    edge_by_pair = {}
    for connection in connections:
        if not isinstance(connection, dict) or connection.get("kind") != "drivable":
            continue
        source = connection.get("source")
        target = connection.get("target")
        if source not in place_by_id or target not in place_by_id:
            raise TopologyRefinementError("a drivable edge references an unknown place")
        length = finite_number(connection.get("length_m"), "connection length")
        if length <= 0.0:
            raise TopologyRefinementError("a drivable edge has non-positive length")
        pair = tuple(sorted((source, target)))
        if pair in edge_by_pair:
            raise TopologyRefinementError("duplicate drivable connection endpoints")
        edge_by_pair[pair] = connection
        adjacency[source].append((target, length))
        if bool(connection.get("bidirectional", False)):
            adjacency[target].append((source, length))
    return place_by_id, adjacency, edge_by_pair


def shortest_place_path(adjacency, start, goal):
    distances = {start: 0.0}
    predecessor = {}
    pending = [(0.0, start)]
    while pending:
        distance, node = heapq.heappop(pending)
        if distance > distances[node] + 1e-12:
            continue
        if node == goal:
            break
        for neighbor, length in adjacency[node]:
            candidate = distance + length
            if candidate + 1e-12 < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                predecessor[neighbor] = node
                heapq.heappush(pending, (candidate, neighbor))
    if goal not in distances:
        raise TopologyRefinementError(
            "no drivable place path from {} to {}".format(start, goal)
        )
    result = [goal]
    while result[-1] != start:
        result.append(predecessor[result[-1]])
    result.reverse()
    return result


def ordered_place_corridor(adjacency, anchors):
    corridor = []
    anchor_offsets = []
    for index, (start, goal) in enumerate(zip(anchors[:-1], anchors[1:])):
        segment = shortest_place_path(adjacency, start, goal)
        if index == 0:
            corridor.extend(segment)
            anchor_offsets.append(0)
        else:
            corridor.extend(segment[1:])
        anchor_offsets.append(len(corridor) - 1)
    if len(corridor) != len(set(corridor)):
        raise TopologyRefinementError(
            "anchor sequence revisits a place; refine each non-reversing corridor separately"
        )
    return corridor, anchor_offsets


def match_reference_anchors(
    positions, cumulative, place_by_id, anchors, maximum_distance
):
    indices = []
    begin = 0
    for anchor in anchors:
        position = place_by_id[anchor]["position"]
        target = np.asarray(
            [finite_number(position.get("x"), "place x"),
             finite_number(position.get("y"), "place y")]
        )
        distances = np.linalg.norm(positions[begin:] - target, axis=1)
        match = begin + int(np.argmin(distances))
        fit = float(distances[match - begin])
        if fit > maximum_distance:
            raise TopologyRefinementError(
                "anchor {} is {:.3f} m from the reference path, exceeding {:.3f} m".format(
                    anchor, fit, maximum_distance
                )
            )
        indices.append(match)
        begin = match
    if any(second <= first for first, second in zip(indices[:-1], indices[1:])):
        raise TopologyRefinementError("reference anchors are not strictly ordered")
    if indices[0] > 2 or indices[-1] < len(positions) - 3:
        raise TopologyRefinementError(
            "reference path must start at the first anchor and end at the last anchor"
        )
    return np.asarray([cumulative[index] for index in indices])


def interpolate_topology(
    positions, yaws, cumulative, anchor_distances, anchor_offsets, place_count
):
    targets = np.empty(place_count, dtype=np.float64)
    for segment_index in range(len(anchor_offsets) - 1):
        first = anchor_offsets[segment_index]
        last = anchor_offsets[segment_index + 1]
        targets[first:last + 1] = np.linspace(
            anchor_distances[segment_index],
            anchor_distances[segment_index + 1],
            last - first + 1,
        )
    points = np.column_stack(
        (
            np.interp(targets, cumulative, positions[:, 0]),
            np.interp(targets, cumulative, positions[:, 1]),
        )
    )
    angles = np.interp(targets, cumulative, yaws)
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    return targets, points, angles


def refresh_semantic_support(graph, place_by_id):
    metadata_path = Path(str(graph.get("source", {}).get("semantic_metadata", "")))
    expected_digest = graph.get("source", {}).get("sha256", {}).get(
        "semantic_metadata"
    )
    if expected_digest is not None and expected_digest != file_sha256(metadata_path):
        raise TopologyRefinementError("semantic metadata digest has changed")
    metadata = strict_json(metadata_path, "semantic metadata")
    objects = metadata.get("objects")
    if not isinstance(objects, list):
        raise TopologyRefinementError("semantic metadata has no objects list")
    parameters = graph.get("parameters", {})
    road_tags = set(parameters.get("drivable_semantic_tags", ["road", "parking"]))
    support_distance = finite_number(
        parameters.get("road_support_distance_m", 2.0), "road support distance"
    )
    road_objects = [
        item
        for item in objects
        if str(item.get("legacy_semantickitti_tag", "")) in road_tags
    ]
    for place in place_by_id.values():
        point = np.asarray(
            [place["position"]["x"], place["position"]["y"]],
            dtype=np.float64,
        )
        support = []
        for item in road_objects:
            center = np.asarray(item["center"], dtype=np.float64)[:2]
            distance = max(
                0.0, float(np.linalg.norm(point - center)) - semantic_radius(item)
            )
            if distance <= support_distance:
                support.append((distance, int(item["id"])))
        support.sort()
        place["road_semantic_ids"] = [item_id for _, item_id in support[:8]]


def refresh_landmark_associations(
    graph, place_landmark_radius, maximum_place_landmarks
):
    places = graph["places"]
    landmarks = graph.get("landmarks", [])
    positions = np.asarray(
        [[place["position"]["x"], place["position"]["y"]] for place in places]
    )
    landmark_by_id = {}
    for landmark in landmarks:
        landmark_id = landmark.get("id")
        landmark_by_id[landmark_id] = landmark
        point = np.asarray(
            [landmark["position"]["x"], landmark["position"]["y"]]
        )
        distances = np.linalg.norm(positions - point, axis=1)
        nearest = int(np.argmin(distances))
        landmark["nearest_place"] = places[nearest]["id"]
        landmark["distance_to_place_m"] = float(distances[nearest])

    for place, position in zip(places, positions):
        nearby = []
        for landmark in landmarks:
            point = np.asarray(
                [landmark["position"]["x"], landmark["position"]["y"]]
            )
            distance = float(np.linalg.norm(point - position))
            if distance <= place_landmark_radius:
                nearby.append(
                    (
                        -int(landmark.get("num_detections", 0)),
                        distance,
                        landmark["id"],
                        str(landmark.get("caption", "object")),
                    )
                )
        nearby.sort()
        nearby = nearby[:maximum_place_landmarks]
        place["landmark_ids"] = [item[2] for item in nearby]
        labels = []
        for _, _, _, caption in nearby:
            if caption not in labels:
                labels.append(caption)
            if len(labels) >= 5:
                break
        place["semantic_summary"] = labels

    for connection in graph["connections"]:
        if connection.get("kind") != "semantic_association":
            continue
        candidates = [
            node
            for node in (connection.get("source"), connection.get("target"))
            if node in landmark_by_id
        ]
        if len(candidates) != 1:
            raise TopologyRefinementError(
                "semantic association does not contain exactly one landmark"
            )
        landmark = landmark_by_id[candidates[0]]
        connection["source"] = landmark["nearest_place"]
        connection["target"] = landmark["id"]
        connection["length_m"] = float(landmark["distance_to_place_m"])


def refine_graph(arguments):
    graph_path = Path(arguments.graph).expanduser().resolve()
    map_path = Path(arguments.map_yaml).expanduser().resolve()
    reference_path = Path(arguments.reference_path).expanduser().resolve()
    graph = strict_json(graph_path, "semantic navigation graph")
    frame_id = str(graph.get("frame_id", ""))
    if not frame_id:
        raise TopologyRefinementError("semantic graph frame is empty")
    place_by_id, adjacency, edge_by_pair = index_drivable_graph(graph)
    anchors = list(arguments.anchors)
    if len(anchors) < 2 or len(anchors) != len(set(anchors)):
        raise TopologyRefinementError("anchors must contain at least two unique places")
    unknown = [anchor for anchor in anchors if anchor not in place_by_id]
    if unknown:
        raise TopologyRefinementError("unknown anchor: {}".format(unknown[0]))

    occupancy_map = OccupancyMap(map_path)
    validate_graph_sources(graph, graph_path, occupancy_map)
    reference, positions, yaws, cumulative = load_reference_path(
        reference_path, graph_path, map_path, frame_id
    )
    corridor, anchor_offsets = ordered_place_corridor(adjacency, anchors)
    anchor_distances = match_reference_anchors(
        positions,
        cumulative,
        place_by_id,
        anchors,
        arguments.maximum_anchor_fit_distance,
    )
    targets, points, angles = interpolate_topology(
        positions,
        yaws,
        cumulative,
        anchor_distances,
        anchor_offsets,
        len(corridor),
    )

    clearances = np.asarray([occupancy_map.clearance_at(point) for point in points])
    if float(clearances.min()) < arguments.minimum_topology_clearance:
        index = int(np.argmin(clearances))
        raise TopologyRefinementError(
            "refined place {} has only {:.3f} m clearance".format(
                corridor[index], clearances[index]
            )
        )
    for place_id, distance, point, angle, clearance in zip(
        corridor, targets, points, angles, clearances
    ):
        place = place_by_id[place_id]
        place["position"] = {
            "x": float(point[0]),
            "y": float(point[1]),
            "z": 0.0,
        }
        place["yaw"] = float(angle)
        place["clearance_m"] = float(clearance)
        place["trajectory_distance_m"] = float(distance)
        place["topology_evidence"] = "smac_planner_certified_reference"

    minimum_edge_clearance = finite_number(
        graph.get("parameters", {}).get("minimum_edge_clearance_m", 0.0),
        "minimum edge clearance",
    )
    refined_edges = []
    for index, (source, target) in enumerate(zip(corridor[:-1], corridor[1:])):
        edge = edge_by_pair[tuple(sorted((source, target)))]
        first_distance = targets[index]
        second_distance = targets[index + 1]
        inside = (cumulative >= first_distance) & (cumulative <= second_distance)
        samples = np.vstack((points[index], positions[inside], points[index + 1]))
        edge_clearance = min(occupancy_map.clearance_at(point) for point in samples)
        if edge_clearance + 1e-12 < minimum_edge_clearance:
            raise TopologyRefinementError(
                "refined connection {} has only {:.3f} m clearance".format(
                    edge.get("id", "unknown"), edge_clearance
                )
            )
        edge["length_m"] = float(second_distance - first_distance)
        edge["minimum_clearance_m"] = float(edge_clearance)
        edge["evidence"] = "smac_planner_certified_reference"
        edge["planner_certified"] = True
        refined_edges.append(edge)

    refresh_semantic_support(graph, place_by_id)
    for edge in refined_edges:
        edge["road_semantic_coverage_ratio"] = float(
            0.5
            * (
                bool(place_by_id[edge["source"]].get("road_semantic_ids"))
                + bool(place_by_id[edge["target"]].get("road_semantic_ids"))
            )
        )
    refresh_landmark_associations(
        graph,
        arguments.place_landmark_radius,
        arguments.maximum_place_landmarks,
    )

    source = graph.setdefault("source", {})
    digests = source.setdefault("sha256", {})
    source["recorded_navigation_graph"] = str(graph_path)
    source["smac_reference_path"] = str(reference_path)
    digests["recorded_navigation_graph"] = file_sha256(graph_path)
    digests["smac_reference_path"] = file_sha256(reference_path)
    parameters = graph.setdefault("parameters", {})
    parameters["topology_generation"] = "smac_planner_certified_reference"
    parameters["topology_certification_anchors"] = anchors
    parameters["minimum_topology_clearance_m"] = float(
        arguments.minimum_topology_clearance
    )
    statistics = graph.setdefault("statistics", {})
    statistics["planner_certified_places"] = len(corridor)
    statistics["planner_certified_connections"] = len(refined_edges)
    statistics["planner_certified_length_m"] = float(
        anchor_distances[-1] - anchor_distances[0]
    )
    statistics["minimum_planner_certified_clearance_m"] = float(
        min(edge["minimum_clearance_m"] for edge in refined_edges)
    )
    statistics["road_semantic_coverage_ratio"] = sum(
        bool(place.get("road_semantic_ids")) for place in graph["places"]
    ) / len(graph["places"])
    statistics["unsafe_connections"] = sum(
        connection.get("kind") == "drivable"
        and float(connection.get("minimum_clearance_m", 0.0))
        < minimum_edge_clearance
        for connection in graph["connections"]
    )
    graph["planner_certification"] = {
        "planner_id": reference.get("planner_id"),
        "route_id": reference.get("route_id"),
        "route_waypoint_mode": reference.get("route_waypoint_mode"),
        "reference_path_length_m": float(cumulative[-1]),
        "anchors": anchors,
        "corridor_places": corridor,
    }
    return graph


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", required=True, type=Path)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--reference-path", required=True, type=Path)
    parser.add_argument("--anchors", required=True, nargs="+")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--maximum-anchor-fit-distance", type=float, default=0.25)
    parser.add_argument("--minimum-topology-clearance", type=float, default=0.5)
    parser.add_argument("--place-landmark-radius", type=float, default=8.0)
    parser.add_argument("--maximum-place-landmarks", type=int, default=8)
    return parser.parse_args()


def validate_arguments(arguments):
    if (
        arguments.maximum_anchor_fit_distance <= 0.0
        or arguments.minimum_topology_clearance <= 0.0
        or arguments.place_landmark_radius <= 0.0
        or arguments.maximum_place_landmarks < 1
    ):
        raise TopologyRefinementError("refinement thresholds must be positive")
    graph_path = Path(arguments.graph).expanduser().resolve()
    output_path = Path(arguments.output).expanduser().resolve()
    if graph_path == output_path:
        raise TopologyRefinementError(
            "input graph and output must differ to preserve recorded evidence"
        )


def main():
    arguments = parse_args()
    validate_arguments(arguments)
    result = refine_graph(arguments)
    output = Path(arguments.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)
    stats = result["statistics"]
    print(
        "Refined {planner_certified_places} places and "
        "{planner_certified_connections} connections over {planner_certified_length_m:.2f} m; "
        "minimum reference clearance {minimum_planner_certified_clearance_m:.2f} m.".format(
            **stats
        )
    )
    print("Saved planner-certified semantic graph to {}".format(output))


if __name__ == "__main__":
    main()
