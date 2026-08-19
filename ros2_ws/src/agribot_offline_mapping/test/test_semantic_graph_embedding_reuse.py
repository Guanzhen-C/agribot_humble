import hashlib
import importlib.util
from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / "import_semantic_graph_neo4j.py"
SPEC = importlib.util.spec_from_file_location("import_semantic_graph_neo4j", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def landmark(caption, category, vector=None):
    item = {"caption": caption, "category": category}
    if vector is not None:
        text = "{}；类别：{}".format(caption, category)
        item["semantic_embedding"] = {
            "provider": "alibaba_cloud_bailian",
            "model": "text-embedding-v4",
            "dimensions": 64,
            "text_sha256": hashlib.sha256(text.encode("utf-8")).hexdigest(),
            "vector": vector,
        }
    return item


def test_reuses_precomputed_bailian_vectors_without_an_api_call(monkeypatch):
    vector = [1.0] + [0.0] * 63

    def unexpected_call(*_args, **_kwargs):
        raise AssertionError("embedding API must not be called")

    monkeypatch.setattr(MODULE, "embed_in_batches", unexpected_call)
    result, reused, generated = MODULE.resolve_embeddings(
        [landmark("蓝色门牌入口", "建筑入口", vector)],
        "server-only-api-key",
        "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "text-embedding-v4",
        64,
        10,
    )

    assert result == [vector]
    assert reused == 1
    assert generated == 0


def test_only_generates_vectors_missing_from_legacy_landmarks(monkeypatch):
    reused_vector = [1.0] + [0.0] * 63
    generated_vector = [0.0, 1.0] + [0.0] * 62
    calls = []

    def fake_embed(texts, *_args):
        calls.append(texts)
        return [generated_vector]

    monkeypatch.setattr(MODULE, "embed_in_batches", fake_embed)
    result, reused, generated = MODULE.resolve_embeddings(
        [
            landmark("蓝色门牌入口", "建筑入口", reused_vector),
            landmark("白色立柱", "立柱"),
        ],
        "server-only-api-key",
        "https://dashscope.aliyuncs.com/compatible-mode/v1",
        "text-embedding-v4",
        64,
        10,
    )

    assert calls == [["白色立柱；类别：立柱"]]
    assert result == [reused_vector, generated_vector]
    assert reused == 1
    assert generated == 1
