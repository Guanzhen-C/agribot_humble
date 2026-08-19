#!/usr/bin/env python3

"""Resolve drivable semantic places into a deterministic route preview."""

import argparse
import hashlib
import heapq
import json
import math
from pathlib import Path


class GraphValidationError(RuntimeError):
    pass


class RoutePlanningError(RuntimeError):
    pass


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_number(value, description):
    if isinstance(value, bool):
        raise GraphValidationError("{} must be a finite number".format(description))
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise GraphValidationError(
            "{} must be a finite number".format(description)
        ) from error
    if not math.isfinite(number):
        raise GraphValidationError("{} must be a finite number".format(description))
    return number


def validate_position(item, node_id):
    position = item.get("position")
    if not isinstance(position, dict):
        raise GraphValidationError("semantic node {} has no position".format(node_id))
    for axis in ("x", "y", "z"):
        finite_number(
            position.get(axis, 0.0), "semantic node {} {}".format(node_id, axis)
        )


class SemanticRouteGraph:
    """A graph of drivable places and place-anchored semantic landmarks."""

    def __init__(self, document):
        if not isinstance(document, dict):
            raise GraphValidationError("navigation graph must be a JSON object")
        if document.get("schema_version") != 3:
            raise GraphValidationError("unsupported navigation graph schema version")
        legacy_fields = sorted(
            field for field in ("nodes", "edges", "place_edges") if field in document
        )
        if legacy_fields:
            raise GraphValidationError(
                "semantic graph contains legacy route fields: {}".format(
                    ", ".join(legacy_fields)
                )
            )
        self.document = document
        self.frame_id = str(document.get("frame_id", ""))
        if not self.frame_id:
            raise GraphValidationError("navigation graph frame_id is empty")

        self.places = self._index_places(document.get("places"))
        self.landmarks = self._index_landmarks(document.get("landmarks"))
        duplicate_ids = sorted(set(self.places).intersection(self.landmarks))
        if duplicate_ids:
            raise GraphValidationError(
                "semantic node id is used by a place and landmark: {}".format(
                    duplicate_ids[0]
                )
            )
        self.nodes = dict(self.places)
        self.nodes.update(self.landmarks)
        self.connections, self.adjacency = self._index_connections(
            document.get("connections")
        )
        self.astar_heuristic_scale = self._compute_astar_heuristic_scale()
        self._validate_connected()

    def _compute_astar_heuristic_scale(self):
        ratios = []
        for connection in self.connections.values():
            if connection.get("kind") != "drivable":
                continue
            source = self.places[connection["source"]]["position"]
            target = self.places[connection["target"]]["position"]
            straight_line = math.hypot(
                float(target["x"]) - float(source["x"]),
                float(target["y"]) - float(source["y"]),
            )
            if straight_line > 1e-12:
                ratios.append(float(connection["length_m"]) / straight_line)
        return min(1.0, min(ratios)) if ratios else 0.0

    def _index_places(self, items):
        if not isinstance(items, list) or not items:
            raise GraphValidationError(
                "navigation graph places must be a non-empty list"
            )
        places = {}
        for index, item in enumerate(items):
            if not isinstance(item, dict):
                raise GraphValidationError("place {} is not an object".format(index))
            place_id = item.get("id")
            if not isinstance(place_id, str) or not place_id:
                raise GraphValidationError("place {} has an invalid id".format(index))
            if place_id in places:
                raise GraphValidationError("duplicate place id: {}".format(place_id))
            if "route_node" in item:
                raise GraphValidationError(
                    "place {} contains a legacy route node".format(place_id)
                )
            validate_position(item, place_id)
            finite_number(item.get("yaw", 0.0), "place {} yaw".format(place_id))
            clearance = finite_number(
                item.get("clearance_m", 0.0),
                "place {} clearance".format(place_id),
            )
            if clearance < 0.0:
                raise GraphValidationError(
                    "place {} clearance is negative".format(place_id)
                )
            places[place_id] = item
        return places

    def _index_landmarks(self, items):
        if not isinstance(items, list):
            raise GraphValidationError("navigation graph landmarks must be a list")
        landmarks = {}
        for index, item in enumerate(items):
            if not isinstance(item, dict):
                raise GraphValidationError(
                    "landmark {} is not an object".format(index)
                )
            landmark_id = item.get("id")
            if not isinstance(landmark_id, str) or not landmark_id:
                raise GraphValidationError(
                    "landmark {} has an invalid id".format(index)
                )
            if landmark_id in landmarks:
                raise GraphValidationError(
                    "duplicate landmark id: {}".format(landmark_id)
                )
            validate_position(item, landmark_id)
            nearest_place = item.get("nearest_place")
            if nearest_place not in self.places:
                raise GraphValidationError(
                    "landmark {} has an unknown nearest place".format(landmark_id)
                )
            distance = finite_number(
                item.get("distance_to_place_m"),
                "landmark {} distance to place".format(landmark_id),
            )
            if distance < 0.0:
                raise GraphValidationError(
                    "landmark {} has a negative place distance".format(landmark_id)
                )
            landmarks[landmark_id] = item
        return landmarks

    def _index_connections(self, items):
        if not isinstance(items, list) or not items:
            raise GraphValidationError(
                "navigation graph connections must be a non-empty list"
            )
        connections = {}
        adjacency = {place_id: [] for place_id in self.places}
        endpoint_pairs = set()
        landmark_associations = {landmark_id: 0 for landmark_id in self.landmarks}
        for index, item in enumerate(items):
            if not isinstance(item, dict):
                raise GraphValidationError(
                    "connection {} is not an object".format(index)
                )
            connection_id = item.get("id")
            if not isinstance(connection_id, str) or not connection_id:
                raise GraphValidationError(
                    "connection {} has an invalid id".format(index)
                )
            if connection_id in connections:
                raise GraphValidationError(
                    "duplicate connection id: {}".format(connection_id)
                )
            source = item.get("source")
            target = item.get("target")
            if source not in self.nodes or target not in self.nodes:
                raise GraphValidationError(
                    "connection {} references an unknown semantic node".format(
                        connection_id
                    )
                )
            if source == target:
                raise GraphValidationError(
                    "connection {} is a self-loop".format(connection_id)
                )
            pair = tuple(sorted((source, target)))
            if pair in endpoint_pairs:
                raise GraphValidationError(
                    "duplicate connection endpoints: {} <-> {}".format(
                        source, target
                    )
                )
            endpoint_pairs.add(pair)
            kind = item.get("kind")
            length = finite_number(
                item.get("length_m"), "connection {} length".format(connection_id)
            )
            if kind == "drivable":
                self._validate_drivable_connection(
                    item, connection_id, source, target, length
                )
                clearance = float(item["minimum_clearance_m"])
            elif kind == "semantic_association":
                landmark_id = self._validate_landmark_connection(
                    item, connection_id, source, target, length
                )
                landmark_associations[landmark_id] += 1
                clearance = None
            else:
                raise GraphValidationError(
                    "connection {} has an unsupported kind".format(connection_id)
                )
            connections[connection_id] = item
            if kind == "drivable":
                adjacency[source].append(
                    (target, length, connection_id, clearance, kind)
                )
                if bool(item.get("bidirectional", False)):
                    adjacency[target].append(
                        (source, length, connection_id, clearance, kind)
                    )
        invalid_associations = sorted(
            landmark_id
            for landmark_id, count in landmark_associations.items()
            if count != 1
        )
        if invalid_associations:
            landmark_id = invalid_associations[0]
            raise GraphValidationError(
                "landmark {} must have exactly one nearest-place association".format(
                    landmark_id
                )
            )
        for neighbors in adjacency.values():
            neighbors.sort(key=lambda entry: (entry[0], entry[2]))
        return connections, adjacency

    def _validate_drivable_connection(
        self, item, connection_id, source, target, length
    ):
        if source not in self.places or target not in self.places:
            raise GraphValidationError(
                "drivable connection {} must join two places".format(connection_id)
            )
        clearance = finite_number(
            item.get("minimum_clearance_m"),
            "connection {} minimum clearance".format(connection_id),
        )
        coverage = finite_number(
            item.get("road_semantic_coverage_ratio"),
            "connection {} road coverage".format(connection_id),
        )
        if (
            length <= 0.0
            or clearance < 0.0
            or not 0.0 <= coverage <= 1.0
            or item.get("executable") is not True
        ):
            raise GraphValidationError(
                "drivable connection {} has invalid metrics or policy".format(
                    connection_id
                )
            )

    def _validate_landmark_connection(
        self, item, connection_id, source, target, length
    ):
        place_ids = [node_id for node_id in (source, target) if node_id in self.places]
        landmark_ids = [
            node_id for node_id in (source, target) if node_id in self.landmarks
        ]
        if len(place_ids) != 1 or len(landmark_ids) != 1:
            raise GraphValidationError(
                "semantic association {} must join one place and one landmark".format(
                    connection_id
                )
            )
        landmark_id = landmark_ids[0]
        landmark = self.landmarks[landmark_id]
        if (
            landmark["nearest_place"] != place_ids[0]
            or length < 0.0
            or item.get("executable") is not False
        ):
            raise GraphValidationError(
                "semantic association {} violates the nearest-place policy".format(
                    connection_id
                )
            )
        if not math.isclose(
            length,
            float(landmark["distance_to_place_m"]),
            rel_tol=1e-9,
            abs_tol=1e-9,
        ):
            raise GraphValidationError(
                "semantic association {} length does not match its landmark".format(
                    connection_id
                )
            )
        return landmark_id

    def _validate_connected(self):
        start = next(iter(self.nodes))
        visited = {start}
        pending = [start]
        undirected = {node_id: set() for node_id in self.nodes}
        for connection in self.connections.values():
            undirected[connection["source"]].add(connection["target"])
            undirected[connection["target"]].add(connection["source"])
        while pending:
            node_id = pending.pop()
            for neighbor in undirected[node_id]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    pending.append(neighbor)
        if len(visited) != len(self.nodes):
            raise GraphValidationError(
                "semantic graph is disconnected: {} of {} nodes reachable".format(
                    len(visited), len(self.nodes)
                )
            )

    def navigation_anchor(self, node_id):
        if node_id in self.places:
            return node_id
        return self.landmarks[node_id]["nearest_place"]

    def resolve_selector(self, selector):
        if selector in self.places:
            place = self.places[selector]
            return {
                "selector": selector,
                "kind": "place",
                "node_id": selector,
                "position": place["position"],
                "semantic_summary": place.get("semantic_summary", []),
                "navigation_anchor_place": selector,
                "navigation_anchor_position": place["position"],
            }
        if selector in self.landmarks:
            landmark = self.landmarks[selector]
            anchor_id = landmark["nearest_place"]
            return {
                "selector": selector,
                "kind": "landmark",
                "node_id": selector,
                "position": landmark["position"],
                "caption": str(landmark.get("caption", "object")),
                "category": str(landmark.get("category", "unknown")),
                "navigation_anchor_place": anchor_id,
                "navigation_anchor_position": self.places[anchor_id]["position"],
            }
        raise RoutePlanningError("unknown semantic node: {}".format(selector))

    def nearest_place(self, x, y, maximum_distance):
        x = finite_number(x, "start x")
        y = finite_number(y, "start y")
        maximum_distance = finite_number(maximum_distance, "maximum start distance")
        if maximum_distance <= 0.0:
            raise RoutePlanningError("maximum start distance must be positive")
        candidates = []
        for place_id, place in self.places.items():
            position = place["position"]
            distance = math.hypot(
                float(position["x"]) - x, float(position["y"]) - y
            )
            candidates.append((distance, place_id))
        distance, place_id = min(candidates)
        if distance > maximum_distance:
            raise RoutePlanningError(
                "nearest semantic place is {:.3f} m away, exceeding {:.3f} m".format(
                    distance, maximum_distance
                )
            )
        return place_id, distance

    def shortest_path(
        self,
        start,
        goal,
        minimum_connection_clearance,
        blocked_nodes=None,
        blocked_connections=None,
    ):
        blocked_nodes = set(blocked_nodes or [])
        blocked_connections = set(blocked_connections or [])
        if start not in self.places or goal not in self.places:
            raise RoutePlanningError(
                "A* endpoints must be drivable semantic places"
            )
        if start in blocked_nodes or goal in blocked_nodes:
            raise RoutePlanningError(
                "route start or goal lies inside a semantic avoidance zone"
            )
        if start == goal:
            return [start], [], 0.0
        def heuristic(node_id):
            position = self.places[node_id]["position"]
            target = self.places[goal]["position"]
            return self.astar_heuristic_scale * math.hypot(
                float(target["x"]) - float(position["x"]),
                float(target["y"]) - float(position["y"]),
            )

        distances = {start: 0.0}
        predecessor = {}
        pending = [(heuristic(start), 0.0, start)]
        while pending:
            _, distance, node_id = heapq.heappop(pending)
            if distance > distances[node_id] + 1e-12:
                continue
            if node_id == goal:
                break
            for neighbor, length, connection_id, clearance, kind in self.adjacency[node_id]:
                if neighbor in blocked_nodes or connection_id in blocked_connections:
                    continue
                if (
                    kind == "drivable"
                    and clearance + 1e-12 < minimum_connection_clearance
                ):
                    continue
                candidate = distance + length
                if candidate + 1e-12 < distances.get(neighbor, math.inf):
                    distances[neighbor] = candidate
                    predecessor[neighbor] = (node_id, connection_id)
                    heapq.heappush(
                        pending,
                        (candidate + heuristic(neighbor), candidate, neighbor),
                    )
        if goal not in distances:
            raise RoutePlanningError(
                "no route from {} to {} with minimum connection clearance "
                "{:.3f} m after applying semantic avoidance constraints".format(
                    start, goal, minimum_connection_clearance
                )
            )

        nodes = [goal]
        connections = []
        current = goal
        while current != start:
            previous, connection_id = predecessor[current]
            nodes.append(previous)
            connections.append(connection_id)
            current = previous
        nodes.reverse()
        connections.reverse()
        return nodes, connections, distances[goal]


def route_yaw(graph, place_ids, index):
    position = graph.places[place_ids[index]]["position"]
    for following in range(index + 1, len(place_ids)):
        target = graph.places[place_ids[following]]["position"]
        dx = float(target["x"]) - float(position["x"])
        dy = float(target["y"]) - float(position["y"])
        if math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
    for previous in range(index - 1, -1, -1):
        source = graph.places[place_ids[previous]]["position"]
        dx = float(position["x"]) - float(source["x"])
        dy = float(position["y"]) - float(source["y"])
        if math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
    return float(graph.places[place_ids[index]].get("yaw", 0.0))


def route_centerline(graph, place_ids, connection_ids):
    if len(connection_ids) != max(0, len(place_ids) - 1):
        raise RoutePlanningError(
            "A* route place and connection counts are inconsistent"
        )
    centerline = []
    for index, connection_id in enumerate(connection_ids):
        connection = graph.connections[connection_id]
        source_id = place_ids[index]
        target_id = place_ids[index + 1]
        if connection.get("kind") != "drivable":
            raise RoutePlanningError(
                "A* route contains a non-drivable connection"
            )
        geometry = connection.get("centerline")
        if not isinstance(geometry, list) or len(geometry) < 2:
            geometry = [
                graph.places[connection["source"]]["position"],
                graph.places[connection["target"]]["position"],
            ]
        if (
            connection["source"] == source_id
            and connection["target"] == target_id
        ):
            oriented = geometry
        elif (
            connection["source"] == target_id
            and connection["target"] == source_id
        ):
            oriented = list(reversed(geometry))
        else:
            raise RoutePlanningError(
                "A* route connection endpoints do not match its places"
            )
        for point in oriented[bool(centerline):]:
            if not isinstance(point, dict):
                raise RoutePlanningError("A* centerline point is invalid")
            centerline.append(
                {
                    "x": finite_number(point.get("x"), "A* centerline x"),
                    "y": finite_number(point.get("y"), "A* centerline y"),
                }
            )
    if len(centerline) < 2:
        raise RoutePlanningError("A* route centerline has fewer than two points")
    return centerline


def point_to_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.hypot(px - ax, py - ay)
    ratio = ((px - ax) * dx + (py - ay) * dy) / length_squared
    ratio = min(1.0, max(0.0, ratio))
    closest_x = ax + ratio * dx
    closest_y = ay + ratio * dy
    return math.hypot(px - closest_x, py - closest_y)


def build_avoidance_constraints(
    graph, avoid_node_ids, avoidance_radius, decay_length=0.5
):
    avoidance_radius = finite_number(avoidance_radius, "avoidance radius")
    if avoidance_radius < 0.0:
        raise RoutePlanningError("avoidance radius must be non-negative")
    decay_length = finite_number(decay_length, "avoidance decay length")
    if decay_length <= 0.0:
        raise RoutePlanningError("avoidance decay length must be positive")
    if not isinstance(avoid_node_ids, list):
        raise RoutePlanningError("avoid node ids must be a list")
    if len(avoid_node_ids) != len(set(avoid_node_ids)):
        raise RoutePlanningError("avoid node ids contain duplicates")
    if any(node_id not in graph.places for node_id in avoid_node_ids):
        raise RoutePlanningError(
            "avoid node ids must contain known drivable semantic places"
        )

    nodes = [graph.resolve_selector(node_id) for node_id in avoid_node_ids]
    centers = [
        (float(node["position"]["x"]), float(node["position"]["y"]))
        for node in nodes
    ]
    blocked_nodes = set()
    for node_id, node in graph.places.items():
        x = float(node["position"]["x"])
        y = float(node["position"]["y"])
        if any(
            math.hypot(x - cx, y - cy) <= avoidance_radius + 1e-12
            for cx, cy in centers
        ):
            blocked_nodes.add(node_id)

    blocked_connections = set()
    for connection_id, connection in graph.connections.items():
        if connection["kind"] != "drivable":
            continue
        source = graph.places[connection["source"]]["position"]
        target = graph.places[connection["target"]]["position"]
        if any(
            point_to_segment_distance(
                cx,
                cy,
                float(source["x"]),
                float(source["y"]),
                float(target["x"]),
                float(target["y"]),
            )
            <= avoidance_radius + 1e-12
            for cx, cy in centers
        ):
            blocked_connections.add(connection_id)

    return {
        "node_ids": list(avoid_node_ids),
        "influence_radius_m": avoidance_radius,
        "decay_length_m": decay_length,
        "nodes": nodes,
        "blocked_node_ids": sorted(blocked_nodes),
        "blocked_connection_ids": sorted(blocked_connections),
    }


def project_navigation_route(graph, semantic_node_ids):
    navigation_place_ids = []
    semantic_to_navigation_index = []
    for node_id in semantic_node_ids:
        anchor = graph.navigation_anchor(node_id)
        if not navigation_place_ids or navigation_place_ids[-1] != anchor:
            navigation_place_ids.append(anchor)
        semantic_to_navigation_index.append(len(navigation_place_ids) - 1)
    return navigation_place_ids, semantic_to_navigation_index


def plan_route(
    graph_document,
    selectors,
    minimum_edge_clearance,
    graph_digest="",
    task_plan=None,
    avoid_node_ids=None,
    avoidance_radius=2.0,
):
    if len(selectors) < 2:
        raise RoutePlanningError("at least a start and goal semantic node are required")
    graph = SemanticRouteGraph(graph_document)
    if minimum_edge_clearance is None:
        minimum_edge_clearance = finite_number(
            graph_document.get("parameters", {}).get(
                "minimum_edge_clearance_m", 0.0
            ),
            "minimum connection clearance",
        )
    minimum_edge_clearance = float(minimum_edge_clearance)
    if not math.isfinite(minimum_edge_clearance) or minimum_edge_clearance < 0.0:
        raise RoutePlanningError(
            "minimum connection clearance must be non-negative"
        )

    stops = [graph.resolve_selector(selector) for selector in selectors]
    if any(stop["kind"] != "place" for stop in stops):
        raise RoutePlanningError(
            "route start and destinations must be drivable semantic places"
        )
    if avoid_node_ids is None:
        avoid_node_ids = (
            task_plan.get("avoid_node_ids", []) if task_plan is not None else []
        )
    avoidance = build_avoidance_constraints(
        graph, list(avoid_node_ids), avoidance_radius
    )
    destination_nodes = set(selectors[1:])
    overlap = sorted(destination_nodes.intersection(avoidance["node_ids"]))
    if overlap:
        raise RoutePlanningError(
            "semantic nodes cannot be both destinations and avoidance constraints: {}".format(
                ", ".join(overlap)
            )
        )

    blocked_nodes = set(avoidance["blocked_node_ids"])
    blocked_connections = set(avoidance["blocked_connection_ids"])
    semantic_node_ids = []
    semantic_connection_ids = []
    astar_cost = 0.0
    semantic_stop_indices = []
    for segment_index, (start, goal) in enumerate(zip(stops[:-1], stops[1:])):
        nodes, connections, cost = graph.shortest_path(
            start["node_id"],
            goal["node_id"],
            minimum_edge_clearance,
            blocked_nodes,
            blocked_connections,
        )
        if segment_index == 0:
            semantic_node_ids.extend(nodes)
            semantic_stop_indices.append(0)
        else:
            semantic_node_ids.extend(nodes[1:])
        semantic_connection_ids.extend(connections)
        astar_cost += cost
        semantic_stop_indices.append(len(semantic_node_ids) - 1)

    navigation_place_ids, semantic_to_navigation_index = project_navigation_route(
        graph, semantic_node_ids
    )
    centerline = route_centerline(
        graph, navigation_place_ids, semantic_connection_ids
    )
    navigation_stop_indices = [
        semantic_to_navigation_index[index] for index in semantic_stop_indices
    ]
    stops_at_index = {}
    for semantic_index, navigation_index, stop in zip(
        semantic_stop_indices, navigation_stop_indices, stops
    ):
        stop["semantic_route_index"] = semantic_index
        stop["navigation_route_index"] = navigation_index
        stops_at_index.setdefault(navigation_index, []).append(stop["selector"])

    poses = []
    supported_places = 0
    for index, place_id in enumerate(navigation_place_ids):
        place = graph.places[place_id]
        if place.get("road_semantic_ids"):
            supported_places += 1
        poses.append(
            {
                "index": index,
                "place_id": place_id,
                "position": {
                    "x": float(place["position"]["x"]),
                    "y": float(place["position"]["y"]),
                    "z": float(place["position"].get("z", 0.0)),
                },
                "yaw": route_yaw(graph, navigation_place_ids, index),
                "stop_selectors": stops_at_index.get(index, []),
            }
        )

    selected_connections = [
        graph.connections[connection_id]
        for connection_id in semantic_connection_ids
    ]
    drivable_connections = [
        connection
        for connection in selected_connections
        if connection["kind"] == "drivable"
    ]
    association_connections = [
        connection
        for connection in selected_connections
        if connection["kind"] == "semantic_association"
    ]
    drivable_length = sum(
        float(connection["length_m"]) for connection in drivable_connections
    )
    association_cost = sum(
        float(connection["length_m"]) for connection in association_connections
    )
    clearances = [
        float(connection["minimum_clearance_m"])
        for connection in drivable_connections
    ]
    if drivable_connections and drivable_length > 0.0:
        covered_length = sum(
            float(connection["length_m"])
            * float(connection["road_semantic_coverage_ratio"])
            for connection in drivable_connections
        )
        road_coverage = covered_length / drivable_length
    else:
        road_coverage = float(
            bool(graph.places[navigation_place_ids[0]].get("road_semantic_ids"))
        )

    request_document = {
        "selectors": selectors,
        "minimum_connection_clearance_m": minimum_edge_clearance,
        "avoid_node_ids": avoidance["node_ids"],
        "avoidance_influence_radius_m": avoidance["influence_radius_m"],
        "avoidance_decay_length_m": avoidance["decay_length_m"],
    }
    route_identity = hashlib.sha256(
        (graph_digest + json.dumps(request_document, sort_keys=True)).encode("utf-8")
    ).hexdigest()[:16]
    contains_landmark_targets = any(stop["kind"] == "landmark" for stop in stops[1:])
    result = {
        "schema_version": 3,
        "route_id": "semantic_route_{}".format(route_identity),
        "frame_id": graph.frame_id,
        "graph_sha256": graph_digest,
        "request": {
            "start": selectors[0],
            "via": selectors[1:-1],
            "goal": selectors[-1],
            "minimum_connection_clearance_m": minimum_edge_clearance,
            "avoid_node_ids": avoidance["node_ids"],
            "avoidance_influence_radius_m": avoidance["influence_radius_m"],
            "avoidance_decay_length_m": avoidance["decay_length_m"],
        },
        "resolved_stops": stops,
        "avoidance_constraints": avoidance,
        "route": {
            "semantic_node_ids": semantic_node_ids,
            "semantic_connection_ids": semantic_connection_ids,
            "navigation_place_ids": navigation_place_ids,
            "poses": poses,
            "centerline": centerline,
        },
        "statistics": {
            "stops": len(stops),
            "route_semantic_nodes": len(semantic_node_ids),
            "route_semantic_connections": len(semantic_connection_ids),
            "route_navigation_places": len(navigation_place_ids),
            "astar_cost_m": astar_cost,
            "search_algorithm": "astar_euclidean_admissible",
            "drivable_route_length_m": drivable_length,
            "semantic_association_cost_m": association_cost,
            "minimum_route_clearance_m": min(clearances) if clearances else None,
            "road_supported_places": supported_places,
            "road_semantic_coverage_ratio": road_coverage,
            "landmark_targets": sum(
                stop["kind"] == "landmark" for stop in stops[1:]
            ),
            "avoided_nodes": len(avoidance["node_ids"]),
            "blocked_nodes": len(blocked_nodes),
            "blocked_connections": len(blocked_connections),
        },
        "execution_policy": {
            "preview_only": True,
            "execution_authorized": False,
            "requires_nav2_path_planning": True,
            "requires_live_collision_checking": True,
            "requires_nav2_proximity_layer": bool(avoidance["node_ids"]),
            "semantic_associations_are_executable": False,
            "landmark_targets_projected_to_nearest_place": contains_landmark_targets,
        },
    }
    if task_plan is not None:
        result["task_plan"] = task_plan
    return result


def validate_task_plan(document, graph_digest, graph):
    if not isinstance(document, dict):
        raise RoutePlanningError("semantic task plan must be a JSON object")
    allowed = {
        "schema_version",
        "task_id",
        "graph_sha256",
        "destination_node_ids",
        "avoid_node_ids",
    }
    unexpected = sorted(set(document) - allowed)
    if unexpected:
        raise RoutePlanningError(
            "semantic task plan contains unsupported fields: {}".format(
                ", ".join(unexpected)
            )
        )
    if document.get("schema_version") != 3:
        raise RoutePlanningError("unsupported semantic task plan schema version")
    task_id = document.get("task_id")
    if not isinstance(task_id, str) or not task_id or len(task_id) > 128:
        raise RoutePlanningError("semantic task plan task_id is invalid")
    task_graph_digest = document.get("graph_sha256")
    if (
        not isinstance(task_graph_digest, str)
        or len(task_graph_digest) != 64
        or any(character not in "0123456789abcdef" for character in task_graph_digest)
    ):
        raise RoutePlanningError("semantic task plan graph_sha256 is invalid")
    if task_graph_digest != graph_digest:
        raise RoutePlanningError("semantic task plan references a stale navigation graph")
    destinations = document.get("destination_node_ids")
    if not isinstance(destinations, list) or not 1 <= len(destinations) <= 16:
        raise RoutePlanningError(
            "semantic task plan must contain 1 to 16 destination node ids"
        )
    if any(
        not isinstance(item, str) or item not in graph.places for item in destinations
    ):
        raise RoutePlanningError(
            "semantic task plan contains an unknown or non-drivable destination place"
        )
    if len(destinations) != len(set(destinations)):
        raise RoutePlanningError("semantic task plan contains duplicate destinations")
    avoid_nodes = document.get("avoid_node_ids", [])
    if not isinstance(avoid_nodes, list) or len(avoid_nodes) > 16:
        raise RoutePlanningError(
            "semantic task plan must contain at most 16 avoid node ids"
        )
    if any(not isinstance(item, str) or item not in graph.places for item in avoid_nodes):
        raise RoutePlanningError(
            "semantic task plan contains an unknown or non-drivable avoidance place"
        )
    if len(avoid_nodes) != len(set(avoid_nodes)):
        raise RoutePlanningError("semantic task plan contains duplicate avoid nodes")
    overlap = sorted(set(destinations).intersection(avoid_nodes))
    if overlap:
        raise RoutePlanningError(
            "semantic task plan uses a node as both destination and avoidance: {}".format(
                ", ".join(overlap)
            )
        )
    return {
        "schema_version": 3,
        "task_id": task_id,
        "graph_sha256": graph_digest,
        "destination_node_ids": destinations,
        "avoid_node_ids": avoid_nodes,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", required=True, type=Path)
    start = parser.add_mutually_exclusive_group(required=True)
    start.add_argument("--start")
    start.add_argument("--start-position", nargs=2, type=float, metavar=("X", "Y"))
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--goal")
    destination.add_argument("--task-plan", type=Path)
    parser.add_argument("--via", action="append", default=[])
    parser.add_argument("--avoid", action="append", default=[])
    parser.add_argument("--avoidance-radius", type=float, default=2.0)
    parser.add_argument("--maximum-start-distance", type=float, default=5.0)
    parser.add_argument("--minimum-edge-clearance", type=float, default=None)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main():
    arguments = parse_args()
    graph_path = arguments.graph.expanduser().resolve()
    if not graph_path.is_file():
        raise RoutePlanningError(
            "navigation graph does not exist: {}".format(graph_path)
        )
    graph_document = json.loads(graph_path.read_text(encoding="utf-8"))
    graph_digest = file_sha256(graph_path)
    graph = SemanticRouteGraph(graph_document)

    start_resolution = {"source": "explicit_selector"}
    start_selector = arguments.start
    if arguments.start_position is not None:
        start_selector, distance = graph.nearest_place(
            arguments.start_position[0],
            arguments.start_position[1],
            arguments.maximum_start_distance,
        )
        start_resolution = {
            "source": "nearest_semantic_place",
            "input_position": {
                "x": arguments.start_position[0],
                "y": arguments.start_position[1],
            },
            "place_id": start_selector,
            "distance_m": distance,
            "maximum_distance_m": arguments.maximum_start_distance,
        }
    else:
        resolved_start = graph.resolve_selector(start_selector)
        if resolved_start["kind"] != "place":
            raise RoutePlanningError("robot start selector must be a semantic place")
        start_resolution = {
            "source": "explicit_selector",
            "place_id": start_selector,
            "position": resolved_start["position"],
        }

    task_plan = None
    if arguments.task_plan is not None:
        if arguments.via or arguments.avoid:
            raise RoutePlanningError(
                "--via and --avoid cannot be combined with --task-plan"
            )
        task_path = arguments.task_plan.expanduser().resolve()
        if not task_path.is_file():
            raise RoutePlanningError(
                "semantic task plan does not exist: {}".format(task_path)
            )
        task_plan = validate_task_plan(
            json.loads(task_path.read_text(encoding="utf-8")), graph_digest, graph
        )
        selectors = [start_selector] + task_plan["destination_node_ids"]
        avoid_node_ids = task_plan["avoid_node_ids"]
    else:
        selectors = [start_selector] + arguments.via + [arguments.goal]
        avoid_node_ids = arguments.avoid

    result = plan_route(
        graph_document,
        selectors,
        arguments.minimum_edge_clearance,
        graph_digest,
        task_plan,
        avoid_node_ids,
        arguments.avoidance_radius,
    )
    result["start_resolution"] = start_resolution
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(arguments.output)
    stats = result["statistics"]
    clearance = stats["minimum_route_clearance_m"]
    clearance_text = "n/a" if clearance is None else "{:.2f} m".format(clearance)
    print(
        "Planned {stops} stops through {route_semantic_nodes} semantic nodes and "
        "{route_navigation_places} navigation places; drivable length "
        "{drivable_route_length_m:.2f} m, A* cost {astar_cost_m:.2f} m, "
        "minimum clearance {clearance}.".format(clearance=clearance_text, **stats)
    )
    print("Saved preview-only route to {}".format(arguments.output))


if __name__ == "__main__":
    try:
        main()
    except (GraphValidationError, RoutePlanningError, json.JSONDecodeError) as error:
        raise SystemExit("error: {}".format(error)) from error
