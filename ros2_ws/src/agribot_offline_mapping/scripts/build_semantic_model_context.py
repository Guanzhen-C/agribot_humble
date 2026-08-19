#!/usr/bin/env python3

"""Export semantic context while restricting model output to drivable places."""

import argparse
import json
from pathlib import Path

from plan_semantic_route import (
    GraphValidationError,
    SemanticRouteGraph,
    file_sha256,
)


def build_model_context(graph_document, graph_digest):
    graph = SemanticRouteGraph(graph_document)
    places = []
    for place_id in sorted(graph.places):
        place = graph.places[place_id]
        places.append(
            {
                "id": place_id,
                "node_type": "place",
                "position": {
                    "x": float(place["position"]["x"]),
                    "y": float(place["position"]["y"]),
                },
                "semantic_summary": [
                    str(item) for item in place.get("semantic_summary", [])
                ],
                "road_semantic_supported": bool(place.get("road_semantic_ids")),
            }
        )

    landmarks = []
    for landmark_id in sorted(graph.landmarks):
        landmark = graph.landmarks[landmark_id]
        landmarks.append(
            {
                "id": landmark_id,
                "node_type": "landmark",
                "position": {
                    "x": float(landmark["position"]["x"]),
                    "y": float(landmark["position"]["y"]),
                },
                "caption": str(landmark.get("caption", "object")),
                "category": str(landmark.get("category", "unknown")),
                "num_detections": int(landmark.get("num_detections", 0)),
                "caption_consensus_ratio": float(
                    landmark.get("caption_consensus_ratio", 0.0)
                ),
                "nearest_place": landmark["nearest_place"],
                "distance_to_place_m": float(landmark["distance_to_place_m"]),
                "navigation_policy": "model_must_output_nearest_place",
            }
        )

    connections = []
    for connection_id in sorted(graph.connections):
        connection = graph.connections[connection_id]
        item = {
            "id": connection_id,
            "kind": connection["kind"],
            "source": connection["source"],
            "target": connection["target"],
            "length_m": float(connection["length_m"]),
            "bidirectional": bool(connection.get("bidirectional", False)),
            "executable": bool(connection.get("executable", False)),
        }
        if connection["kind"] == "drivable":
            item["minimum_raster_clearance_m"] = float(
                connection["minimum_clearance_m"]
            )
            item["road_semantic_coverage_ratio"] = float(
                connection["road_semantic_coverage_ratio"]
            )
        connections.append(item)

    allowed_ids = [item["id"] for item in places]
    return {
        "schema_version": 3,
        "context_type": "agribot_semantic_place_landmark_graph",
        "graph_sha256": graph_digest,
        "frame_id": graph.frame_id,
        "statistics": {
            "semantic_nodes": len(places) + len(landmarks),
            "places": len(places),
            "landmarks": len(landmarks),
            "connections": len(connections),
            "drivable_connections": sum(
                item["kind"] == "drivable" for item in connections
            ),
            "semantic_associations": sum(
                item["kind"] == "semantic_association" for item in connections
            ),
        },
        "planning_contract": {
            "model_may_select": "ordered destination and avoidance place ids only",
            "allowed_output_node_types": ["place"],
            "destination_order": "strictly preserve the user-requested visit order",
            "robot_supplies_start": True,
            "landmarks_are_context_only": True,
            "model_maps_landmarks_to_nearest_place": True,
            "astar_searches_drivable_places_only": True,
            "semantic_associations_are_not_drivable": True,
            "nav2_must_replan_and_collision_check": True,
            "direct_motion_commands_forbidden": True,
            "required_output_schema": "semantic_task_plan.schema.json",
        },
        "allowed_destination_node_ids": allowed_ids,
        "places": places,
        "landmarks": landmarks,
        "connections": connections,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main():
    arguments = parse_args()
    graph_path = arguments.graph.expanduser().resolve()
    if not graph_path.is_file():
        raise GraphValidationError(
            "semantic navigation graph does not exist: {}".format(graph_path)
        )
    graph_document = json.loads(graph_path.read_text(encoding="utf-8"))
    context = build_model_context(graph_document, file_sha256(graph_path))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = arguments.output.with_suffix(arguments.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(context, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(arguments.output)
    stats = context["statistics"]
    print(
        "Exported all {semantic_nodes} semantic nodes: {places} places, "
        "{landmarks} landmarks and {connections} connections.".format(**stats)
    )
    print("Saved provider-neutral model context to {}".format(arguments.output))


if __name__ == "__main__":
    try:
        main()
    except (GraphValidationError, json.JSONDecodeError) as error:
        raise SystemExit("error: {}".format(error)) from error
