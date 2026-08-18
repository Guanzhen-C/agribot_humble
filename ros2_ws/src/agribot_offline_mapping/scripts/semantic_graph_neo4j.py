#!/usr/bin/env python3

"""Import and retrieve Agribot semantic graphs through Neo4j's HTTP API."""

import base64
import json
import math
import re
import socket
import urllib.error
import urllib.parse
import urllib.request


DEFAULT_NEO4J_HTTP_URI = "http://192.168.100.218:7476"
DEFAULT_EMBEDDING_MODEL = "text-embedding-v4"
DEFAULT_EMBEDDING_DIMENSIONS = 1024
VECTOR_INDEX_NAME = "agribot_landmark_embedding"
FULLTEXT_INDEX_NAME = "agribot_landmark_text"
SEMANTIC_STOP_WORDS = {
    "a",
    "an",
    "and",
    "around",
    "at",
    "by",
    "for",
    "in",
    "near",
    "of",
    "place",
    "the",
    "to",
    "with",
}
CHINESE_SEMANTIC_STOP_WORDS = {
    "不要",
    "位置",
    "前往",
    "区域",
    "地点",
    "地方",
    "巡检",
    "旁边",
    "经过",
    "附近",
}


class Neo4jGraphError(RuntimeError):
    pass


def strict_json_loads(value, description):
    def reject_constant(constant):
        raise ValueError("non-finite JSON number: {}".format(constant))

    def reject_duplicate_keys(pairs):
        result = {}
        for key, item in pairs:
            if key in result:
                raise ValueError("duplicate JSON key: {}".format(key))
            result[key] = item
        return result

    try:
        return json.loads(
            value,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise Neo4jGraphError(
            "{} is not one strict JSON document: {}".format(description, error)
        ) from error


def validate_identifier(value, description):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,128}", value):
        raise Neo4jGraphError("{} contains unsupported characters".format(description))
    return value


def validate_http_uri(uri, require_https=False):
    parsed = urllib.parse.urlsplit(uri)
    allowed_schemes = {"https"} if require_https else {"http", "https"}
    if (
        parsed.scheme not in allowed_schemes
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
    ):
        scheme_text = "HTTPS" if require_https else "HTTP(S)"
        raise Neo4jGraphError(
            "URI must be an {} URL without credentials, query, or fragment".format(
                scheme_text
            )
        )
    return uri.rstrip("/")


class Neo4jHttpClient:
    """Minimal fixed-query client; credentials never enter query documents."""

    def __init__(
        self,
        uri,
        user,
        password,
        database="neo4j",
        timeout=30.0,
        opener=None,
    ):
        self.uri = validate_http_uri(uri)
        self.user = str(user)
        self.password = str(password)
        self.database = validate_identifier(database, "Neo4j database")
        self.timeout = float(timeout)
        self.opener = (
            opener
            if opener is not None
            else urllib.request.build_opener(urllib.request.ProxyHandler({})).open
        )
        if not self.user or not self.password:
            raise Neo4jGraphError("Neo4j user and password are required")
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise Neo4jGraphError("Neo4j timeout must be positive")

    @property
    def endpoint(self):
        return "{}/db/{}/tx/commit".format(self.uri, self.database)

    def run(self, statement, parameters=None):
        if not isinstance(statement, str) or not statement.strip():
            raise Neo4jGraphError("Cypher statement is empty")
        authorization = base64.b64encode(
            "{}:{}".format(self.user, self.password).encode("utf-8")
        ).decode("ascii")
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(
                {
                    "statements": [
                        {
                            "statement": statement,
                            "parameters": parameters or {},
                            "resultDataContents": ["row"],
                        }
                    ]
                },
                ensure_ascii=False,
            ).encode("utf-8"),
            headers={
                "Authorization": "Basic {}".format(authorization),
                "Content-Type": "application/json",
                "Accept": "application/json",
                "User-Agent": "agribot-semantic-graph/1",
            },
            method="POST",
        )
        try:
            with self.opener(request, timeout=self.timeout) as response:
                payload = response.read().decode("utf-8")
        except urllib.error.HTTPError as error:
            body = error.read(2048).decode("utf-8", errors="replace")
            raise Neo4jGraphError(
                "Neo4j HTTP request failed with status {}: {}".format(
                    error.code, body[:1000]
                )
            ) from error
        except (urllib.error.URLError, socket.timeout, TimeoutError) as error:
            raise Neo4jGraphError("Neo4j request failed: {}".format(error)) from error

        document = strict_json_loads(payload, "Neo4j response")
        errors = document.get("errors")
        if not isinstance(errors, list):
            raise Neo4jGraphError("Neo4j response has no errors array")
        if errors:
            first = errors[0] if isinstance(errors[0], dict) else {}
            raise Neo4jGraphError(
                "Neo4j query failed: {}: {}".format(
                    first.get("code", "unknown"), first.get("message", "unknown error")
                )
            )
        results = document.get("results")
        if not isinstance(results, list) or len(results) != 1:
            raise Neo4jGraphError("Neo4j response must contain exactly one result")
        result = results[0]
        columns = result.get("columns")
        data = result.get("data")
        if not isinstance(columns, list) or not isinstance(data, list):
            raise Neo4jGraphError("Neo4j result has invalid columns or data")
        rows = []
        for item in data:
            row = item.get("row") if isinstance(item, dict) else None
            if not isinstance(row, list) or len(row) != len(columns):
                raise Neo4jGraphError("Neo4j result row does not match columns")
            rows.append(dict(zip(columns, row)))
        return rows


def call_embeddings(
    texts,
    api_key,
    base_url,
    model=DEFAULT_EMBEDDING_MODEL,
    dimensions=DEFAULT_EMBEDDING_DIMENSIONS,
    timeout=90.0,
    opener=None,
):
    if not isinstance(texts, list) or not texts:
        raise Neo4jGraphError("embedding input must be a non-empty list")
    if any(not isinstance(text, str) or not text.strip() for text in texts):
        raise Neo4jGraphError("embedding input contains an empty text")
    if not isinstance(api_key, str) or not api_key.strip():
        raise Neo4jGraphError(
            "environment variable DASHSCOPE_API_KEY is not configured"
        )
    if not isinstance(dimensions, int) or not 64 <= dimensions <= 4096:
        raise Neo4jGraphError("embedding dimensions must be between 64 and 4096")
    endpoint = validate_http_uri(base_url, require_https=True) + "/embeddings"
    request = urllib.request.Request(
        endpoint,
        data=json.dumps(
            {
                "model": model,
                "input": texts,
                "dimensions": dimensions,
                "encoding_format": "float",
            },
            ensure_ascii=False,
        ).encode("utf-8"),
        headers={
            "Authorization": "Bearer {}".format(api_key.strip()),
            "Content-Type": "application/json",
            "Accept": "application/json",
            "User-Agent": "agribot-semantic-embedding/1",
        },
        method="POST",
    )
    open_request = opener if opener is not None else urllib.request.urlopen
    try:
        with open_request(request, timeout=timeout) as response:
            payload = response.read().decode("utf-8")
    except urllib.error.HTTPError as error:
        body = error.read(2048).decode("utf-8", errors="replace")
        raise Neo4jGraphError(
            "embedding request failed with HTTP {}: {}".format(error.code, body[:1000])
        ) from error
    except (urllib.error.URLError, socket.timeout, TimeoutError) as error:
        raise Neo4jGraphError("embedding request failed: {}".format(error)) from error

    document = strict_json_loads(payload, "embedding API response")
    data = document.get("data")
    if not isinstance(data, list) or len(data) != len(texts):
        raise Neo4jGraphError("embedding response count does not match input count")
    vectors = [None] * len(texts)
    for item in data:
        if not isinstance(item, dict) or not isinstance(item.get("index"), int):
            raise Neo4jGraphError("embedding response contains an invalid item")
        index = item["index"]
        vector = item.get("embedding")
        if (
            not 0 <= index < len(texts)
            or not isinstance(vector, list)
            or len(vector) != dimensions
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in vector
            )
        ):
            raise Neo4jGraphError("embedding response contains an invalid vector")
        vectors[index] = [float(value) for value in vector]
    if any(vector is None for vector in vectors):
        raise Neo4jGraphError("embedding response omitted an input index")
    return vectors


def embed_in_batches(
    texts,
    api_key,
    base_url,
    model=DEFAULT_EMBEDDING_MODEL,
    dimensions=DEFAULT_EMBEDDING_DIMENSIONS,
    batch_size=10,
    **kwargs
):
    if not isinstance(batch_size, int) or not 1 <= batch_size <= 10:
        raise Neo4jGraphError("embedding batch size must be between 1 and 10")
    vectors = []
    for offset in range(0, len(texts), batch_size):
        vectors.extend(
            call_embeddings(
                texts[offset:offset + batch_size],
                api_key,
                base_url,
                model,
                dimensions,
                **kwargs
            )
        )
    return vectors


def create_schema(client, embedding_dimensions):
    statements = [
        "CREATE CONSTRAINT agribot_map_unique IF NOT EXISTS "
        "FOR (n:AgribotMap) REQUIRE n.id IS UNIQUE",
        "CREATE CONSTRAINT agribot_place_unique IF NOT EXISTS "
        "FOR (n:AgribotPlace) REQUIRE (n.map_id, n.id) IS UNIQUE",
        "CREATE CONSTRAINT agribot_landmark_unique IF NOT EXISTS "
        "FOR (n:AgribotLandmark) REQUIRE (n.map_id, n.id) IS UNIQUE",
        "CREATE FULLTEXT INDEX {} IF NOT EXISTS FOR (n:AgribotLandmark) "
        "ON EACH [n.caption, n.category, n.search_text]".format(FULLTEXT_INDEX_NAME),
        "CREATE VECTOR INDEX {} IF NOT EXISTS FOR (n:AgribotLandmark) "
        "ON (n.embedding) OPTIONS {{indexConfig: {{`vector.dimensions`: {}, "
        "`vector.similarity_function`: 'cosine'}}}}".format(
            VECTOR_INDEX_NAME, int(embedding_dimensions)
        ),
    ]
    for statement in statements:
        client.run(statement)


def batches(items, batch_size):
    for offset in range(0, len(items), batch_size):
        yield items[offset:offset + batch_size]


def import_navigation_graph(
    client,
    graph_document,
    graph_digest,
    map_id,
    embeddings=None,
    embedding_model=DEFAULT_EMBEDDING_MODEL,
    embedding_dimensions=DEFAULT_EMBEDDING_DIMENSIONS,
    batch_size=100,
):
    map_id = validate_identifier(map_id, "map id")
    places = graph_document.get("places")
    landmarks = graph_document.get("landmarks")
    connections = graph_document.get("connections")
    if not isinstance(places, list) or not isinstance(landmarks, list):
        raise Neo4jGraphError("navigation graph has invalid places or landmarks")
    if not isinstance(connections, list):
        raise Neo4jGraphError("navigation graph has invalid connections")
    if embeddings is not None and len(embeddings) != len(landmarks):
        raise Neo4jGraphError("landmark embedding count does not match graph")
    if not isinstance(batch_size, int) or batch_size <= 0:
        raise Neo4jGraphError("Neo4j import batch size must be positive")

    create_schema(client, embedding_dimensions)
    client.run(
        "MERGE (m:AgribotMap {id: $map_id}) SET m.graph_sha256 = $graph_sha256, "
        "m.frame_id = $frame_id, m.schema_version = $schema_version, "
        "m.language = $language, "
        "m.place_count = $place_count, m.landmark_count = $landmark_count, "
        "m.connection_count = $connection_count, m.import_ready = false, "
        "m.updated_at = datetime()",
        {
            "map_id": map_id,
            "graph_sha256": graph_digest,
            "frame_id": str(graph_document.get("frame_id", "map")),
            "schema_version": int(graph_document.get("schema_version", 0)),
            "language": str(graph_document.get("language", "zh-CN")),
            "place_count": len(places),
            "landmark_count": len(landmarks),
            "connection_count": len(connections),
        },
    )
    client.run(
        "MATCH (n) WHERE n.map_id = $map_id AND "
        "(n:AgribotPlace OR n:AgribotLandmark) DETACH DELETE n",
        {"map_id": map_id},
    )

    place_rows = []
    for place in places:
        position = place["position"]
        place_rows.append(
            {
                "id": place["id"],
                "name": str(place.get("name", place["id"])),
                "x": float(position["x"]),
                "y": float(position["y"]),
                "z": float(position.get("z", 0.0)),
                "yaw": float(place.get("yaw", 0.0)),
                "clearance_m": float(place.get("clearance_m", 0.0)),
                "semantic_summary": [
                    str(item) for item in place.get("semantic_summary", [])
                ],
            }
        )
    for rows in batches(place_rows, batch_size):
        client.run(
            "UNWIND $rows AS row MATCH (m:AgribotMap {id: $map_id}) "
            "CREATE (p:AgribotPlace {map_id: $map_id, id: row.id, x: row.x, "
            "y: row.y, z: row.z, yaw: row.yaw, clearance_m: row.clearance_m, "
            "name: row.name, language: 'zh-CN', managed_by: 'graph_import', "
            "semantic_summary: row.semantic_summary})-[:IN_MAP]->(m)",
            {"map_id": map_id, "rows": rows},
        )

    landmark_rows = []
    for index, landmark in enumerate(landmarks):
        position = landmark["position"]
        caption = str(landmark.get("caption", "object"))
        category = str(landmark.get("category", "unknown"))
        row = {
            "id": landmark["id"],
            "caption": caption,
            "category": category,
            "search_text": "{}；类别：{}".format(caption, category),
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position.get("z", 0.0)),
            "num_detections": int(landmark.get("num_detections", 0)),
            "caption_consensus_ratio": float(
                landmark.get("caption_consensus_ratio", 0.0)
            ),
            "nearest_place": landmark["nearest_place"],
            "distance_to_place_m": float(landmark["distance_to_place_m"]),
        }
        if embeddings is not None:
            row["embedding"] = embeddings[index]
        landmark_rows.append(row)
    for rows in batches(landmark_rows, batch_size):
        client.run(
            "UNWIND $rows AS row MATCH (m:AgribotMap {id: $map_id}) "
            "MATCH (p:AgribotPlace {map_id: $map_id, id: row.nearest_place}) "
            "CREATE (l:AgribotLandmark {map_id: $map_id, id: row.id, "
            "caption: row.caption, category: row.category, "
            "search_text: row.search_text, "
            "language: 'zh-CN', managed_by: 'graph_import', "
            "x: row.x, y: row.y, z: row.z, num_detections: row.num_detections, "
            "caption_consensus_ratio: row.caption_consensus_ratio})-[:IN_MAP]->(m) "
            "CREATE (l)-[:NEAREST_PLACE {distance_m: row.distance_to_place_m}]->(p) "
            "FOREACH (_ IN CASE WHEN row.embedding IS NULL THEN [] ELSE [1] END | "
            "SET l.embedding = row.embedding, l.embedding_model = $embedding_model, "
            "l.embedding_dimensions = $embedding_dimensions)",
            {
                "map_id": map_id,
                "rows": rows,
                "embedding_model": embedding_model,
                "embedding_dimensions": embedding_dimensions,
            },
        )

    drivable_rows = []
    for connection in connections:
        if connection.get("kind") != "drivable":
            continue
        drivable_rows.append(
            {
                "id": connection["id"],
                "source": connection["source"],
                "target": connection["target"],
                "length_m": float(connection["length_m"]),
                "minimum_clearance_m": float(connection["minimum_clearance_m"]),
                "road_semantic_coverage_ratio": float(
                    connection["road_semantic_coverage_ratio"]
                ),
                "bidirectional": bool(connection.get("bidirectional", False)),
                "executable": bool(connection.get("executable", False)),
            }
        )
    for rows in batches(drivable_rows, batch_size):
        client.run(
            "UNWIND $rows AS row "
            "MATCH (source:AgribotPlace {map_id: $map_id, id: row.source}) "
            "MATCH (target:AgribotPlace {map_id: $map_id, id: row.target}) "
            "CREATE (source)-[:DRIVABLE {id: row.id, length_m: row.length_m, "
            "minimum_clearance_m: row.minimum_clearance_m, "
            "road_semantic_coverage_ratio: row.road_semantic_coverage_ratio, "
            "bidirectional: row.bidirectional, executable: row.executable}]->(target)",
            {"map_id": map_id, "rows": rows},
        )
    client.run("CALL db.awaitIndexes(300)")
    rows = client.run(
        "MATCH (m:AgribotMap {id: $map_id}) "
        "OPTIONAL MATCH (p:AgribotPlace {map_id: $map_id}) "
        "WITH m, count(DISTINCT p) AS places "
        "OPTIONAL MATCH (l:AgribotLandmark {map_id: $map_id}) "
        "WITH m, places, count(DISTINCT l) AS landmarks "
        "OPTIONAL MATCH (:AgribotPlace {map_id: $map_id})-[r:DRIVABLE]->"
        "(:AgribotPlace {map_id: $map_id}) "
        "RETURN m.graph_sha256 AS graph_sha256, places, landmarks, "
        "count(DISTINCT r) AS drivable_connections",
        {"map_id": map_id},
    )
    if len(rows) != 1:
        raise Neo4jGraphError("Neo4j import verification returned no map")
    result = rows[0]
    expected = {
        "graph_sha256": graph_digest,
        "places": len(places),
        "landmarks": len(landmarks),
        "drivable_connections": len(drivable_rows),
    }
    if result != expected:
        raise Neo4jGraphError(
            "Neo4j import verification failed: expected {}, received {}".format(
                expected, result
            )
        )
    client.run(
        "MATCH (m:AgribotMap {id: $map_id}) "
        "SET m.import_ready = true, m.imported_at = datetime()",
        {"map_id": map_id},
    )
    return result


def upsert_incremental_landmarks(
    client,
    map_id,
    landmarks,
    embeddings=None,
    embedding_model=DEFAULT_EMBEDDING_MODEL,
    embedding_dimensions=DEFAULT_EMBEDDING_DIMENSIONS,
    batch_size=100,
):
    map_id = validate_identifier(map_id, "map id")
    if not isinstance(landmarks, list) or not landmarks:
        raise Neo4jGraphError("incremental landmarks must be a non-empty list")
    if embeddings is not None and len(embeddings) != len(landmarks):
        raise Neo4jGraphError("incremental landmark embedding count does not match")
    if not isinstance(batch_size, int) or batch_size <= 0:
        raise Neo4jGraphError("Neo4j import batch size must be positive")
    create_schema(client, embedding_dimensions)

    rows = []
    for index, landmark in enumerate(landmarks):
        position = landmark["position"]
        caption = str(landmark["caption"])
        category = str(landmark["category"])
        row = {
            "id": validate_identifier(landmark["id"], "landmark id"),
            "caption": caption,
            "category": category,
            "search_text": "{}；类别：{}".format(caption, category),
            "x": float(position["x"]),
            "y": float(position["y"]),
            "z": float(position.get("z", 0.0)),
            "nearest_place": validate_identifier(
                landmark["nearest_place"], "nearest place id"
            ),
            "distance_to_place_m": float(landmark["distance_to_place_m"]),
            "embedding": embeddings[index] if embeddings is not None else None,
        }
        if any(
            not math.isfinite(row[key])
            for key in ("x", "y", "z", "distance_to_place_m")
        ):
            raise Neo4jGraphError("incremental landmark contains a non-finite value")
        rows.append(row)

    for row_batch in batches(rows, batch_size):
        client.run(
            "UNWIND $rows AS row "
            "MATCH (m:AgribotMap {id: $map_id}) "
            "MATCH (p:AgribotPlace {map_id: $map_id, id: row.nearest_place}) "
            "MERGE (l:AgribotLandmark {map_id: $map_id, id: row.id}) "
            "SET l.caption = row.caption, l.category = row.category, "
            "l.search_text = row.search_text, l.x = row.x, l.y = row.y, l.z = row.z, "
            "l.language = 'zh-CN', l.managed_by = 'incremental_zh', "
            "l.updated_at = datetime() "
            "MERGE (l)-[:IN_MAP]->(m) "
            "WITH l, p, row OPTIONAL MATCH (l)-[old:NEAREST_PLACE]->() DELETE old "
            "MERGE (l)-[nearest:NEAREST_PLACE]->(p) "
            "SET nearest.distance_m = row.distance_to_place_m "
            "FOREACH (_ IN CASE WHEN row.embedding IS NULL THEN [] ELSE [1] END | "
            "SET l.embedding = row.embedding, l.embedding_model = $embedding_model, "
            "l.embedding_dimensions = $embedding_dimensions)",
            {
                "map_id": map_id,
                "rows": row_batch,
                "embedding_model": embedding_model,
                "embedding_dimensions": embedding_dimensions,
            },
        )
    result = client.run(
        "MATCH (l:AgribotLandmark {map_id: $map_id, managed_by: 'incremental_zh'}) "
        "WHERE l.id IN $landmark_ids "
        "RETURN count(l) AS upserted",
        {"map_id": map_id, "landmark_ids": [row["id"] for row in rows]},
    )
    if len(result) != 1 or result[0].get("upserted") != len(rows):
        raise Neo4jGraphError("incremental landmark verification failed")
    return {"upserted": len(rows), "map_id": map_id}


def assert_graph_version(client, map_id, graph_digest):
    rows = client.run(
        "MATCH (m:AgribotMap {id: $map_id}) "
        "RETURN m.graph_sha256 AS graph_sha256, m.import_ready AS import_ready",
        {"map_id": validate_identifier(map_id, "map id")},
    )
    if len(rows) != 1:
        raise Neo4jGraphError("Neo4j does not contain map {}".format(map_id))
    if rows[0].get("graph_sha256") != graph_digest:
        raise Neo4jGraphError("Neo4j semantic graph is stale for map {}".format(map_id))
    if rows[0].get("import_ready") is not True:
        raise Neo4jGraphError(
            "Neo4j semantic graph import is incomplete for map {}".format(map_id)
        )


def sanitize_fulltext_query(text):
    tokens = re.findall(r"[a-zA-Z0-9]+|[\u3400-\u9fff]", text)
    return " OR ".join(tokens[:32])


def semantic_terms(text):
    terms = set()
    for token in re.findall(r"[a-z0-9]+", text.lower()):
        if token in SEMANTIC_STOP_WORDS:
            continue
        if len(token) > 4 and token.endswith("ies"):
            token = token[:-3] + "y"
        elif len(token) > 3 and token.endswith("s") and not token.endswith("ss"):
            token = token[:-1]
        terms.add(token)
    for sequence in re.findall(r"[\u3400-\u9fff]+", text):
        if sequence in CHINESE_SEMANTIC_STOP_WORDS:
            continue
        if len(sequence) == 1:
            terms.add(sequence)
            continue
        terms.update(
            token
            for token in (
                sequence[index:index + 2] for index in range(len(sequence) - 1)
            )
            if token not in CHINESE_SEMANTIC_STOP_WORDS
        )
    return terms


def _query_vector(client, map_id, embedding, limit):
    if embedding is None:
        return []
    return client.run(
        "CALL db.index.vector.queryNodes($index_name, $search_limit, $embedding) "
        "YIELD node, score WHERE node.map_id = $map_id "
        "MATCH (node)-[rel:NEAREST_PLACE]->(place:AgribotPlace {map_id: $map_id}) "
        "RETURN node.id AS landmark_id, node.caption AS caption, "
        "node.category AS category, node.x AS landmark_x, node.y AS landmark_y, "
        "place.id AS place_id, place.x AS place_x, place.y AS place_y, "
        "rel.distance_m AS distance_to_place_m, score "
        "ORDER BY score DESC LIMIT $limit",
        {
            "index_name": VECTOR_INDEX_NAME,
            "search_limit": max(100, limit * 20),
            "embedding": embedding,
            "map_id": map_id,
            "limit": max(limit * 4, limit),
        },
    )


def _query_fulltext(client, map_id, query_text, limit):
    query = sanitize_fulltext_query(query_text)
    if not query:
        return []
    return client.run(
        "CALL db.index.fulltext.queryNodes($index_name, $query, "
        "{limit: $search_limit}) YIELD node, score "
        "WHERE node.map_id = $map_id "
        "MATCH (node)-[rel:NEAREST_PLACE]->(place:AgribotPlace {map_id: $map_id}) "
        "RETURN node.id AS landmark_id, node.caption AS caption, "
        "node.category AS category, node.x AS landmark_x, node.y AS landmark_y, "
        "place.id AS place_id, place.x AS place_x, place.y AS place_y, "
        "rel.distance_m AS distance_to_place_m, score "
        "ORDER BY score DESC LIMIT $limit",
        {
            "index_name": FULLTEXT_INDEX_NAME,
            "query": query,
            "search_limit": max(100, limit * 20),
            "map_id": map_id,
            "limit": max(limit * 4, limit),
        },
    )


def retrieve_place_candidates(client, map_id, query_text, embedding=None, top_k=5):
    map_id = validate_identifier(map_id, "map id")
    if not isinstance(query_text, str) or not query_text.strip():
        raise Neo4jGraphError("semantic retrieval query is empty")
    if not isinstance(top_k, int) or not 1 <= top_k <= 20:
        raise Neo4jGraphError("retrieval top_k must be between 1 and 20")
    vector_rows = _query_vector(client, map_id, embedding, top_k)
    fulltext_rows = _query_fulltext(client, map_id, query_text, top_k)

    landmarks = {}
    for source, rows in (("vector", vector_rows), ("fulltext", fulltext_rows)):
        for rank, row in enumerate(rows, 1):
            landmark_id = row.get("landmark_id")
            place_id = row.get("place_id")
            if not isinstance(landmark_id, str) or not isinstance(place_id, str):
                raise Neo4jGraphError("Neo4j retrieval returned an invalid semantic id")
            record = landmarks.setdefault(
                landmark_id,
                {
                    "landmark_id": landmark_id,
                    "caption": str(row.get("caption", "object")),
                    "category": str(row.get("category", "unknown")),
                    "landmark_position": {
                        "x": float(row.get("landmark_x", 0.0)),
                        "y": float(row.get("landmark_y", 0.0)),
                    },
                    "place_id": place_id,
                    "place_position": {
                        "x": float(row.get("place_x", 0.0)),
                        "y": float(row.get("place_y", 0.0)),
                    },
                    "distance_to_place_m": float(
                        row.get("distance_to_place_m", 0.0)
                    ),
                    "retrieval_sources": [],
                    "hybrid_score": 0.0,
                },
            )
            record["retrieval_sources"].append(
                {
                    "source": source,
                    "rank": rank,
                    "source_score": float(row.get("score", 0.0)),
                }
            )
            record["hybrid_score"] += 1.0 / (60.0 + rank)

    places = {}
    for landmark in landmarks.values():
        place = places.setdefault(
            landmark["place_id"],
            {
                "place_id": landmark["place_id"],
                "position": landmark["place_position"],
                "hybrid_score": 0.0,
                "evidence_landmarks": [],
            },
        )
        place["hybrid_score"] = max(
            place["hybrid_score"], landmark["hybrid_score"]
        )
        evidence = dict(landmark)
        evidence.pop("place_id")
        evidence.pop("place_position")
        place["evidence_landmarks"].append(evidence)

    query_terms = semantic_terms(query_text)
    for place in places.values():
        place["evidence_landmarks"].sort(
            key=lambda item: (-item["hybrid_score"], item["landmark_id"])
        )
        place["evidence_landmarks"] = place["evidence_landmarks"][:3]
        evidence_text = " ".join(
            "{} {}".format(item["caption"], item["category"])
            for item in place["evidence_landmarks"]
        )
        matched_terms = query_terms.intersection(semantic_terms(evidence_text))
        place["lexical_coverage_ratio"] = (
            float(len(matched_terms)) / float(len(query_terms))
            if query_terms
            else 0.0
        )
    result = sorted(
        places.values(),
        key=lambda item: (
            -item["lexical_coverage_ratio"],
            -item["hybrid_score"],
            item["place_id"],
        ),
    )[:top_k]
    return result
