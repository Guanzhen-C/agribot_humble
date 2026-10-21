import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("batch_prepare_cad", ROOT / "scripts/batch_prepare_cad.py")
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


def test_batch_never_guesses_assembly_among_multiple_parts():
    part = {"format": "step", "member": "part.step", "hashStatus": "verified"}
    assert batch.choose_geometry({"geometryCandidates": [part]}) == part
    iges = {"format": "igs", "member": "part.igs", "hashStatus": "verified"}
    assert batch.choose_geometry({"geometryCandidates": [iges]}) == iges
    assert batch.choose_geometry({"geometryCandidates": [iges, part]}) == part
    for candidates in ([], [part, part], [{"format": "stp"}], [{"format": "obj"}]):
        with pytest.raises(ValueError):
            batch.choose_geometry({"geometryCandidates": candidates})


def test_scene_candidates_cannot_claim_runtime_readiness():
    sources = json.loads((ROOT / "config/scene_sources.json").read_text())["sources"]
    assert len({item["id"] for item in sources}) == len(sources)
    for item in sources:
        assert item["url"].startswith("https://")
        assert item["licenseScope"] and item["evidence"]
        if item["status"] == "integrated":
            assert (ROOT / item["evidence"]).is_file()
            assert item["runtimeScenario"]
        else:
            assert item["runtimeSelectable"] is False
            assert item["pending"]
