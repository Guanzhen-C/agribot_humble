import base64
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[1] / "scripts" / "semantic_graph_neo4j.py"
)
SPEC = importlib.util.spec_from_file_location("semantic_graph_neo4j", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Response:
    def __init__(self, document):
        self.payload = json.dumps(document).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def read(self):
        return self.payload


def neo4j_response(columns=None, rows=None, errors=None):
    return {
        "results": [
            {
                "columns": columns or [],
                "data": [{"row": row} for row in (rows or [])],
            }
        ],
        "errors": errors or [],
    }


def test_http_client_uses_fixed_transaction_endpoint_and_basic_auth():
    captured = {}

    def opener(request, timeout):
        captured["url"] = request.full_url
        captured["headers"] = dict(request.header_items())
        captured["document"] = json.loads(request.data)
        captured["timeout"] = timeout
        return Response(neo4j_response(["value"], [[7]]))

    client = MODULE.Neo4jHttpClient(
        "http://192.168.100.218:7476", "neo4j", "secret", opener=opener
    )
    rows = client.run("RETURN $value AS value", {"value": 7})

    assert rows == [{"value": 7}]
    assert captured["url"].endswith("/db/neo4j/tx/commit")
    assert captured["document"]["statements"][0]["parameters"] == {"value": 7}
    expected = base64.b64encode(b"neo4j:secret").decode("ascii")
    assert captured["headers"]["Authorization"] == "Basic " + expected


def test_http_client_rejects_credentials_in_uri_and_query_errors():
    with pytest.raises(MODULE.Neo4jGraphError, match="without credentials"):
        MODULE.Neo4jHttpClient("http://neo4j:secret@host:7474", "u", "p")

    error = {
        "code": "Neo.ClientError.Statement.SyntaxError",
        "message": "bad query",
    }
    client = MODULE.Neo4jHttpClient(
        "http://host:7474",
        "u",
        "p",
        opener=lambda *_args, **_kwargs: Response(
            neo4j_response(errors=[error])
        ),
    )
    with pytest.raises(MODULE.Neo4jGraphError, match="bad query"):
        client.run("BROKEN")


def test_embedding_api_preserves_input_order_and_validates_dimensions():
    captured = {}
    vectors = [[float(index)] * 64 for index in (1, 2)]

    def opener(request, timeout):
        captured["url"] = request.full_url
        captured["document"] = json.loads(request.data)
        return Response({"embeddings": vectors})

    result = MODULE.call_embeddings(
        ["white building", "green hedge"],
        "http://172.18.80.26:11434",
        dimensions=64,
        opener=opener,
    )

    assert all(abs(sum(value * value for value in vector) - 1.0) < 1e-12 for vector in result)
    assert captured["url"].endswith("/api/embed")
    assert captured["document"]["model"] == "qwen3-embedding:8b"


class RetrievalClient:
    def run(self, statement, parameters=None):
        if "db.index.vector.queryNodes" in statement:
            return [
                candidate("landmark_a", "place_001", 0.95),
                candidate("landmark_b", "place_002", 0.90),
            ]
        if "db.index.fulltext.queryNodes" in statement:
            return [
                candidate("landmark_b", "place_002", 8.0),
                candidate("landmark_c", "place_003", 7.0),
            ]
        raise AssertionError(statement)


def candidate(landmark_id, place_id, score, caption=None):
    return {
        "landmark_id": landmark_id,
        "caption": caption or landmark_id,
        "category": "object",
        "landmark_x": 1.0,
        "landmark_y": 2.0,
        "place_id": place_id,
        "place_x": 1.5,
        "place_y": 2.5,
        "distance_to_place_m": 0.7,
        "score": score,
    }


def test_hybrid_retrieval_fuses_ranks_and_returns_places_only():
    result = MODULE.retrieve_place_candidates(
        RetrievalClient(), "map_0811", "white building", [0.0] * 64, 3
    )

    assert [item["place_id"] for item in result] == [
        "place_002",
        "place_001",
        "place_003",
    ]
    assert result[0]["evidence_landmarks"][0]["landmark_id"] == "landmark_b"
    sources = result[0]["evidence_landmarks"][0]["retrieval_sources"]
    assert {source["source"] for source in sources} == {
        "vector",
        "fulltext",
    }


def test_fulltext_sanitization_removes_lucene_control_characters():
    assert MODULE.sanitize_fulltext_query('blue:bike + "motorcycle"') == (
        "blue OR bike OR motorcycle"
    )
    assert MODULE.sanitize_fulltext_query("___") == ""
    assert MODULE.sanitize_fulltext_query("白色建筑附近") == (
        "白 OR 色 OR 建 OR 筑 OR 附 OR 近"
    )
    terms = MODULE.semantic_terms("前往白色高层建筑附近")
    assert {"白色", "高层", "建筑"}.issubset(terms)
    assert "附近" not in terms


class CompoundRetrievalClient:
    def run(self, statement, parameters=None):
        rows = [
            candidate("landmark_bike", "place_007", 0.90, "a blue bicycle"),
            candidate("landmark_other", "place_013", 0.99, "a bicycle"),
            candidate(
                "landmark_motorcycle",
                "place_007",
                0.80,
                "a parked motorcycle",
            ),
        ]
        if "db.index.vector.queryNodes" in statement:
            return rows
        if "db.index.fulltext.queryNodes" in statement:
            return rows
        raise AssertionError(statement)


def test_compound_keywords_prioritize_a_place_with_all_evidence():
    result = MODULE.retrieve_place_candidates(
        CompoundRetrievalClient(),
        "map_0811",
        "place with blue bicycle and motorcycle",
        [0.0] * 64,
        2,
    )

    assert result[0]["place_id"] == "place_007"
    assert result[0]["lexical_coverage_ratio"] == 1.0
    assert result[1]["lexical_coverage_ratio"] < 1.0


def tiny_graph():
    return {
        "schema_version": 3,
        "frame_id": "map",
        "places": [
            {
                "id": "place_000",
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 1.0,
                "semantic_summary": ["a gate"],
            },
            {
                "id": "place_001",
                "position": {"x": 2.0, "y": 0.0, "z": 0.0},
                "yaw": 0.0,
                "clearance_m": 1.0,
                "semantic_summary": ["a building"],
            },
        ],
        "landmarks": [
            {
                "id": "landmark_gate",
                "caption": "a gate",
                "category": "fence",
                "position": {"x": 0.0, "y": 1.0, "z": 0.5},
                "num_detections": 10,
                "caption_consensus_ratio": 0.8,
                "nearest_place": "place_000",
                "distance_to_place_m": 1.0,
            }
        ],
        "connections": [
            {
                "id": "connection_000",
                "kind": "drivable",
                "source": "place_000",
                "target": "place_001",
                "length_m": 2.0,
                "minimum_clearance_m": 0.8,
                "road_semantic_coverage_ratio": 1.0,
                "bidirectional": True,
                "executable": True,
            }
        ],
    }


class ImportClient:
    def __init__(self, verification=None):
        self.statements = []
        self.verification = verification or {
            "graph_sha256": "a" * 64,
            "places": 2,
            "landmarks": 1,
            "drivable_connections": 1,
        }

    def run(self, statement, parameters=None):
        self.statements.append((statement, parameters or {}))
        if "count(DISTINCT p) AS places" in statement:
            return [self.verification]
        return []


def test_import_is_not_marked_ready_until_counts_are_verified():
    client = ImportClient()
    result = MODULE.import_navigation_graph(
        client, tiny_graph(), "a" * 64, "test_map", batch_size=10
    )

    statements = [statement for statement, _ in client.statements]
    mark_incomplete = next(
        index
        for index, statement in enumerate(statements)
        if "m.import_ready = false" in statement
    )
    delete_old = next(
        index for index, statement in enumerate(statements) if "DETACH DELETE" in statement
    )
    assert mark_incomplete < delete_old
    assert "m.import_ready = true" in statements[-1]
    assert result["landmarks"] == 1


def test_failed_import_verification_remains_incomplete():
    client = ImportClient(
        {
            "graph_sha256": "a" * 64,
            "places": 2,
            "landmarks": 0,
            "drivable_connections": 1,
        }
    )

    with pytest.raises(MODULE.Neo4jGraphError, match="verification failed"):
        MODULE.import_navigation_graph(
            client, tiny_graph(), "a" * 64, "test_map", batch_size=10
        )
    assert not any(
        "m.import_ready = true" in statement for statement, _ in client.statements
    )


class VersionClient:
    def __init__(self, row):
        self.row = row

    def run(self, _statement, _parameters=None):
        return [self.row]


def test_graph_version_rejects_an_incomplete_import():
    client = VersionClient(
        {"graph_sha256": "a" * 64, "import_ready": False}
    )
    with pytest.raises(MODULE.Neo4jGraphError, match="incomplete"):
        MODULE.assert_graph_version(client, "test_map", "a" * 64)


class IncrementalClient:
    def __init__(self):
        self.statements = []

    def run(self, statement, parameters=None):
        self.statements.append((statement, parameters or {}))
        if "RETURN count(l) AS upserted" in statement:
            return [{"upserted": len(parameters["landmark_ids"])}]
        return []


def test_incremental_chinese_landmark_is_merged_without_deleting_graph():
    client = IncrementalClient()
    result = MODULE.upsert_incremental_landmarks(
        client,
        "test_map",
        [
            {
                "id": "landmark_zh_gate",
                "caption": "园区北门",
                "category": "入口",
                "position": {"x": 1.0, "y": 2.0, "z": 0.5},
                "nearest_place": "place_001",
                "distance_to_place_m": 0.7,
            }
        ],
        embeddings=[[0.0] * 64],
        embedding_dimensions=64,
    )

    assert result == {"upserted": 1, "map_id": "test_map"}
    statements = [statement for statement, _ in client.statements]
    merge = next(statement for statement in statements if "incremental_zh" in statement)
    assert "MERGE (l:AgribotLandmark" in merge
    assert "NEAREST_PLACE" in merge
    assert "DETACH DELETE" not in merge
