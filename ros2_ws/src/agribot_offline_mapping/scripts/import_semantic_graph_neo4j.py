#!/usr/bin/env python3

"""Import one immutable Agribot semantic navigation graph into Neo4j."""

import argparse
import json
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
        embedding_texts = [
            "{}；类别：{}".format(
                str(item.get("caption", "object")),
                str(item.get("category", "unknown")),
            )
            for item in landmarks
        ]
        print(
            "Embedding {} landmarks with {}...".format(
                len(embedding_texts), arguments.embedding_model
            )
        )
        embeddings = embed_in_batches(
            embedding_texts,
            os.environ.get("DASHSCOPE_API_KEY", ""),
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
