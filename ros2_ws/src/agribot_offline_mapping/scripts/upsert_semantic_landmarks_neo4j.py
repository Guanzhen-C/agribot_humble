#!/usr/bin/env python3

"""Incrementally attach Chinese landmarks to existing Neo4j map places."""

import argparse
import json
import math
import os
from pathlib import Path
import re

import numpy as np

from plan_semantic_route import GraphValidationError, SemanticRouteGraph, file_sha256
from semantic_graph_neo4j import (
    DEFAULT_EMBEDDING_DIMENSIONS,
    DEFAULT_EMBEDDING_MODEL,
    DEFAULT_NEO4J_HTTP_URI,
    Neo4jGraphError,
    Neo4jHttpClient,
    assert_graph_version,
    embed_in_batches,
    upsert_incremental_landmarks,
)


DEFAULT_OLLAMA_BASE_URL = "http://172.18.80.26:11434"


def contains_chinese(value):
    return bool(re.search(r"[\u3400-\u9fff]", str(value)))


def load_incremental_landmarks(path, graph_document, maximum_distance):
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise Neo4jGraphError("unsupported incremental landmark schema")
    if document.get("language") != "zh-CN":
        raise Neo4jGraphError("incremental landmarks must declare language zh-CN")
    raw_landmarks = document.get("landmarks")
    if not isinstance(raw_landmarks, list) or not raw_landmarks:
        raise Neo4jGraphError("incremental landmark file has no landmarks")

    places = graph_document.get("places")
    place_positions = np.asarray(
        [
            [float(item["position"]["x"]), float(item["position"]["y"])]
            for item in places
        ],
        dtype=np.float64,
    )
    resolved = []
    seen = set()
    for item in raw_landmarks:
        if not isinstance(item, dict):
            raise Neo4jGraphError("incremental landmark item must be an object")
        landmark_id = str(item.get("id", ""))
        if landmark_id in seen:
            raise Neo4jGraphError("incremental landmark ids must be unique")
        seen.add(landmark_id)
        caption = str(item.get("caption", "")).strip()
        category = str(item.get("category", "")).strip()
        if not contains_chinese(caption) or not contains_chinese(category):
            raise Neo4jGraphError("incremental landmark text must be Chinese")
        position = item.get("position")
        if not isinstance(position, dict):
            raise Neo4jGraphError("incremental landmark position is invalid")
        coordinates = np.asarray(
            [
                float(position.get("x", math.nan)),
                float(position.get("y", math.nan)),
                float(position.get("z", 0.0)),
            ]
        )
        if not np.isfinite(coordinates).all():
            raise Neo4jGraphError("incremental landmark position must be finite")
        distances = np.linalg.norm(place_positions - coordinates[:2], axis=1)
        nearest = int(np.argmin(distances))
        distance = float(distances[nearest])
        if distance > maximum_distance:
            raise Neo4jGraphError(
                "landmark {} is {:.2f} m from the nearest place, beyond {:.2f} m".format(
                    landmark_id, distance, maximum_distance
                )
            )
        resolved.append(
            {
                "id": landmark_id,
                "caption": caption,
                "category": category,
                "language": "zh-CN",
                "position": {
                    "x": float(coordinates[0]),
                    "y": float(coordinates[1]),
                    "z": float(coordinates[2]),
                },
                "nearest_place": places[nearest]["id"],
                "distance_to_place_m": distance,
            }
        )
    return resolved


def atomic_write(path, graph_digest, map_id, landmarks):
    document = {
        "schema_version": 1,
        "language": "zh-CN",
        "map_id": map_id,
        "graph_sha256": graph_digest,
        "landmarks": landmarks,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", required=True, type=Path)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--landmarks", required=True, type=Path)
    parser.add_argument("--resolved-output", type=Path)
    parser.add_argument("--maximum-place-distance", type=float, default=20.0)
    parser.add_argument(
        "--neo4j-http-uri",
        default=os.environ.get("AGRIBOT_NEO4J_HTTP_URI", DEFAULT_NEO4J_HTTP_URI),
    )
    parser.add_argument(
        "--neo4j-user", default=os.environ.get("AGRIBOT_NEO4J_USER", "neo4j")
    )
    parser.add_argument(
        "--neo4j-database", default=os.environ.get("AGRIBOT_NEO4J_DATABASE", "neo4j")
    )
    parser.add_argument("--neo4j-timeout", type=float, default=30.0)
    parser.add_argument(
        "--embedding-base-url",
        default=os.environ.get("AGRIBOT_OLLAMA_URL", DEFAULT_OLLAMA_BASE_URL),
    )
    parser.add_argument("--embedding-model", default=DEFAULT_EMBEDDING_MODEL)
    parser.add_argument(
        "--embedding-dimensions", type=int, default=DEFAULT_EMBEDDING_DIMENSIONS
    )
    parser.add_argument("--embedding-batch-size", type=int, default=10)
    parser.add_argument("--neo4j-batch-size", type=int, default=100)
    parser.add_argument("--skip-embeddings", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_args()
    if not math.isfinite(arguments.maximum_place_distance) or arguments.maximum_place_distance <= 0.0:
        raise Neo4jGraphError("maximum place distance must be positive")
    graph_path = arguments.graph.expanduser().resolve()
    graph_document = json.loads(graph_path.read_text(encoding="utf-8"))
    SemanticRouteGraph(graph_document)
    graph_digest = file_sha256(graph_path)
    landmarks = load_incremental_landmarks(
        arguments.landmarks.expanduser().resolve(),
        graph_document,
        arguments.maximum_place_distance,
    )
    embeddings = None
    if not arguments.skip_embeddings:
        embeddings = embed_in_batches(
            [
                "{}；类别：{}".format(item["caption"], item["category"])
                for item in landmarks
            ],
            arguments.embedding_base_url,
            arguments.embedding_model,
            arguments.embedding_dimensions,
            arguments.embedding_batch_size,
        )
    client = Neo4jHttpClient(
        arguments.neo4j_http_uri,
        arguments.neo4j_user,
        os.environ.get("AGRIBOT_NEO4J_PASSWORD", ""),
        arguments.neo4j_database,
        arguments.neo4j_timeout,
    )
    assert_graph_version(client, arguments.map_id, graph_digest)
    result = upsert_incremental_landmarks(
        client,
        arguments.map_id,
        landmarks,
        embeddings,
        arguments.embedding_model,
        arguments.embedding_dimensions,
        arguments.neo4j_batch_size,
    )
    if arguments.resolved_output is not None:
        atomic_write(
            arguments.resolved_output.expanduser().resolve(),
            graph_digest,
            arguments.map_id,
            landmarks,
        )
    print(
        "Incrementally upserted {upserted} Chinese landmarks into Neo4j map "
        "{map_id}.".format(**result)
    )


if __name__ == "__main__":
    try:
        main()
    except (
        GraphValidationError,
        Neo4jGraphError,
        OSError,
        UnicodeDecodeError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        raise SystemExit("error: {}".format(error)) from error
