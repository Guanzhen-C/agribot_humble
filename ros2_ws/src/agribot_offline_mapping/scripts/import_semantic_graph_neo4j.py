#!/usr/bin/env python3

"""Import one immutable Agribot semantic navigation graph into Neo4j."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

from plan_semantic_route import GraphValidationError, SemanticRouteGraph, file_sha256
from semantic_graph_neo4j import (
    DEFAULT_EMBEDDING_DIMENSIONS,
    DEFAULT_EMBEDDING_MODEL,
    DEFAULT_NEO4J_HTTP_URI,
    Neo4jGraphError,
    Neo4jHttpClient,
    embed_in_batches,
    import_navigation_graph,
)


DEFAULT_BAILIAN_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"


def landmark_embedding_text(item):
    return "{}；类别：{}".format(
        str(item.get("caption", "object")),
        str(item.get("category", "unknown")),
    )


def reusable_embedding(item, model, dimensions):
    embedding = item.get("semantic_embedding")
    if not isinstance(embedding, dict):
        return None
    text = landmark_embedding_text(item)
    expected_digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
    vector = embedding.get("vector")
    if (
        embedding.get("provider") != "alibaba_cloud_bailian"
        or embedding.get("model") != model
        or embedding.get("dimensions") != dimensions
        or embedding.get("text_sha256") != expected_digest
        or not isinstance(vector, list)
        or len(vector) != dimensions
        or any(
            not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in vector
        )
    ):
        return None
    normalized = [float(value) for value in vector]
    norm = math.sqrt(sum(value * value for value in normalized))
    if not 0.99 <= norm <= 1.01:
        return None
    return normalized


def resolve_embeddings(
    landmarks,
    api_key,
    base_url,
    model,
    dimensions,
    batch_size,
):
    embeddings = [None] * len(landmarks)
    missing_indices = []
    missing_texts = []
    for index, item in enumerate(landmarks):
        vector = reusable_embedding(item, model, dimensions)
        if vector is None:
            missing_indices.append(index)
            missing_texts.append(landmark_embedding_text(item))
        else:
            embeddings[index] = vector
    generated = []
    if missing_texts:
        generated = embed_in_batches(
            missing_texts,
            api_key,
            base_url,
            model,
            dimensions,
            batch_size,
        )
        for index, vector in zip(missing_indices, generated):
            embeddings[index] = vector
    if any(vector is None for vector in embeddings):
        raise Neo4jGraphError("failed to resolve every landmark embedding")
    return embeddings, len(landmarks) - len(missing_indices), len(generated)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--graph", required=True, type=Path)
    parser.add_argument("--map-id", required=True)
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
        default=os.environ.get("DASHSCOPE_BASE_URL", DEFAULT_BAILIAN_BASE_URL),
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
    graph_path = arguments.graph.expanduser().resolve()
    if not graph_path.is_file():
        raise Neo4jGraphError("navigation graph does not exist: {}".format(graph_path))
    graph_document = json.loads(graph_path.read_text(encoding="utf-8"))
    SemanticRouteGraph(graph_document)
    graph_digest = file_sha256(graph_path)
    landmarks = graph_document["landmarks"]
    embeddings = None
    if not arguments.skip_embeddings:
        embeddings, reused_count, generated_count = resolve_embeddings(
            landmarks,
            os.environ.get("DASHSCOPE_API_KEY", ""),
            arguments.embedding_base_url,
            arguments.embedding_model,
            arguments.embedding_dimensions,
            arguments.embedding_batch_size,
        )
        print(
            "Resolved {} landmark embeddings with {}: {} reused, {} generated.".format(
                len(landmarks),
                arguments.embedding_model,
                reused_count,
                generated_count,
            )
        )

    client = Neo4jHttpClient(
        arguments.neo4j_http_uri,
        arguments.neo4j_user,
        os.environ.get("AGRIBOT_NEO4J_PASSWORD", ""),
        arguments.neo4j_database,
        arguments.neo4j_timeout,
    )
    result = import_navigation_graph(
        client,
        graph_document,
        graph_digest,
        arguments.map_id,
        embeddings,
        arguments.embedding_model,
        arguments.embedding_dimensions,
        arguments.neo4j_batch_size,
    )
    print(
        "Imported map {map_id}: {places} places, {landmarks} landmarks, "
        "{drivable_connections} drivable connections, graph {graph_sha256}.".format(
            map_id=arguments.map_id, **result
        )
    )
    if embeddings is None:
        print("Vector embeddings were skipped; retrieval will use the full-text index.")


if __name__ == "__main__":
    try:
        main()
    except (
        GraphValidationError,
        Neo4jGraphError,
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as error:
        raise SystemExit("error: {}".format(error)) from error
