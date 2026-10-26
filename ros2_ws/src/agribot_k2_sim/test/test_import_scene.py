import io
import fcntl
import json
from pathlib import Path
import stat
import struct
import zipfile

import numpy as np
from PIL import Image
import pytest

from agribot_k2_sim import import_scene as importer
from agribot_k2_sim.lcc_decode import decode_scene, local_resource, selected_sources


def manifest(count=2):
    return {"version": "0.0.3", "shift": [0, 0, 0], "scale": [1, 1, 1],
            "lodSplats": [count], "root": {"splatFiles": ["tile.sog"],
            "child": {"leaf": {"data": {"3dgs": {"name": 0, "start": 0, "count": count}}}}}}


def test_finest_selection_does_not_stack_coarse_levels():
    data = manifest()
    data["root"]["data"] = {"3dgs": {"name": 9, "start": 0, "count": 500}}
    assert selected_sources(data) == ({0: [(0, 2)]}, 2, 0)
    data["lodSplats"] = [3]
    with pytest.raises(ValueError):
        selected_sources(data)


def test_sog_decodes_all_attributes_and_cache_layout(tmp_path):
    meta = {"version": 2, "count": 2,
            "means": {"files": ["low.png", "high.png"], "mins": [0, 0, 0], "maxs": [np.log(2)]*3},
            "scales": {"files": ["scale.png"], "codebook": [np.log(.1)]},
            "quats": {"files": ["quat.png"]},
            "sh0": {"files": ["color.png"], "codebook": [0.]}}
    with zipfile.ZipFile(tmp_path / "tile.sog", "w") as zf:
        zf.writestr("meta.json", json.dumps(meta))
        for name, array in {
            "low.png": [[[0, 0, 0, 255], [255, 255, 255, 255]]],
            "high.png": [[[0, 0, 0, 255], [255, 255, 255, 255]]],
            "scale.png": [[[0, 0, 0, 255]]*2],
            "quat.png": [[[128, 128, 128, 252]]*2],
            "color.png": [[[0, 0, 0, 200]]*2],
        }.items():
            buffer = io.BytesIO()
            Image.fromarray(np.array(array, np.uint8), "RGBA").save(buffer, format="PNG")
            zf.writestr(name, buffer.getvalue())
    result = decode_scene(tmp_path, manifest(), tmp_path / "gaussians.bin", lambda _: None)
    assert result["render_gaussians"] == 2
    raw = (tmp_path / "gaussians.bin").read_bytes()
    assert raw[:16] == b"LCCGS001" + struct.pack("<Q", 2)
    data = np.frombuffer(raw[16:], "<f4").reshape(2, 13)
    np.testing.assert_allclose(data[:, :3], [[0]*3, [1]*3], atol=1e-6)
    np.testing.assert_allclose(data[:, 9:12], .5)
    assert np.isfinite(data).all()


@pytest.mark.parametrize("name", ["../evil", "/absolute", "sub/../../evil", "sub\\evil"])
def test_reject_unsafe_archive_paths(tmp_path, name):
    archive = tmp_path / "unsafe.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr(name, "bad")
        zf.writestr("scene.lcc2", "{}")
    with pytest.raises(ValueError, match="不安全"):
        importer.extract_archive(archive, tmp_path / "out")
    assert not (tmp_path / "evil").exists()


def test_reject_zip_symlinks_and_missing_or_multiple_manifest(tmp_path):
    for case in ("symlink", "missing", "multiple"):
        archive = tmp_path / (case + ".zip")
        with zipfile.ZipFile(archive, "w") as zf:
            if case == "symlink":
                entry = zipfile.ZipInfo("link")
                entry.external_attr = (stat.S_IFLNK | 0o777) << 16
                zf.writestr(entry, "/etc/passwd")
                zf.writestr("a.lcc2", "{}")
            if case == "multiple":
                zf.writestr("a.lcc2", "{}")
                zf.writestr("b.lcc2", "{}")
        with pytest.raises(ValueError):
            importer.extract_archive(archive, tmp_path / "out")


@pytest.mark.parametrize("name", ["", " ", "a\nb", "a"*49])
def test_reject_empty_long_and_control_names(name):
    with pytest.raises(ValueError):
        importer.validate_name(name)


def fake_prepare(archive, folder, final, checksum):
    for name in ("scene.json", "world.sdf", "map.yaml", "map.pgm", "preview.jpg",
                 "gaussian/gaussians.bin", "gaussian/libK2GaussianVisual.so"):
        target = folder / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps({"spawn": [1, 2, 0]}))


def test_named_registration_cache_reuse_and_duplicate_name_are_atomic(tmp_path, monkeypatch):
    archive = tmp_path / "source.zip"
    archive.write_bytes(b"fixture")
    root = tmp_path / "library"
    calls = []
    def prepare(*args):
        calls.append(args)
        fake_prepare(*args)
    monkeypatch.setattr(importer, "prepare_asset", prepare)
    first = importer.import_scene(archive, "园区一", root)
    second = importer.import_scene(archive, "园区二", root)
    assert len(calls) == 1
    assert first["id"] != second["id"] and first["sceneManifest"] == second["sceneManifest"]
    before = (root / "catalog.json").read_bytes()
    with pytest.raises(ValueError, match="已存在"):
        importer.import_scene(archive, "园区一", root)
    assert (root / "catalog.json").read_bytes() == before
    assert not list(root.glob(".import-*"))


def test_failed_conversion_does_not_publish_catalog_or_keep_partial_asset(tmp_path, monkeypatch):
    archive = tmp_path / "source.zip"
    archive.write_bytes(b"fixture")
    def fail(*_):
        raise ValueError("bad data")
    monkeypatch.setattr(importer, "prepare_asset", fail)
    root = tmp_path / "library"
    with pytest.raises(ValueError, match="bad data"):
        importer.import_scene(archive, "园区", root)
    assert not (root / "catalog.json").exists()
    assert not list(root.glob(".import-*"))


def test_deletion_clears_last_cache_but_preserves_zip_and_allows_same_name_reimport(tmp_path, monkeypatch):
    archive = tmp_path / "source.zip"
    archive.write_bytes(b"fixture")
    root = tmp_path / "library"
    monkeypatch.setattr(importer, "prepare_asset", fake_prepare)
    first = importer.import_scene(archive, "园区", root)
    cache = Path(first["sceneManifest"]).parent
    assert importer.delete_scene(first["id"], root) == first
    assert json.loads((root / "catalog.json").read_text())["scenarios"] == []
    assert not cache.exists() and not list(root.glob(".deleting-*"))
    assert archive.read_bytes() == b"fixture"
    second = importer.import_scene(archive, "园区", root)
    assert second["id"] != first["id"] and cache.exists()


def test_shared_cache_is_deleted_only_after_last_scene(tmp_path, monkeypatch):
    archive = tmp_path / "source.zip"
    archive.write_bytes(b"fixture")
    root = tmp_path / "library"
    monkeypatch.setattr(importer, "prepare_asset", fake_prepare)
    first = importer.import_scene(archive, "园区一", root)
    second = importer.import_scene(archive, "园区二", root)
    importer.delete_scene(first["id"], root)
    assert Path(second["sceneManifest"]).is_file()
    assert json.loads((root / "catalog.json").read_text())["scenarios"] == [second]
    importer.delete_scene(second["id"], root)
    assert not Path(second["sceneManifest"]).parent.exists()


@pytest.mark.parametrize("identifier", ["orchard", "../scan_fixture", "scan_missing"])
def test_delete_rejects_builtin_unknown_and_path_identifiers(tmp_path, identifier):
    path = tmp_path / "catalog.json"
    before = json.dumps({"schemaVersion": "1.0", "scenarios": []})
    path.write_text(before)
    with pytest.raises(ValueError):
        importer.delete_scene(identifier, tmp_path)
    assert path.read_text() == before


def test_delete_rejects_unsafe_cache_and_respects_import_lock(tmp_path):
    root = tmp_path / "library"
    root.mkdir()
    source = tmp_path / "original"
    source.mkdir()
    scene = {"id": "scan_fixture", "imported": True, "sceneManifest": str(source / "scene.json")}
    path = root / "catalog.json"
    before = json.dumps({"schemaVersion": "1.0", "scenarios": [scene]})
    path.write_text(before)
    with (root / ".import.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(ValueError, match="正在更新"):
            importer.delete_scene("scan_fixture", root)
    with pytest.raises(ValueError, match="不安全"):
        importer.delete_scene("scan_fixture", root)
    assert source.is_dir() and path.read_text() == before


def test_delete_restores_cache_if_catalog_publication_fails(tmp_path, monkeypatch):
    archive = tmp_path / "source.zip"
    archive.write_bytes(b"fixture")
    root = tmp_path / "library"
    monkeypatch.setattr(importer, "prepare_asset", fake_prepare)
    scene = importer.import_scene(archive, "园区", root)
    path = root / "catalog.json"
    before = path.read_bytes()
    def fail(*_):
        raise OSError("fixture publication error")
    monkeypatch.setattr(Path, "replace", fail)
    with pytest.raises(OSError, match="publication"):
        importer.delete_scene(scene["id"], root)
    assert path.read_bytes() == before and Path(scene["sceneManifest"]).is_file()
    assert not list(root.glob(".deleting-*")) and not path.with_suffix(".tmp").exists()


def test_resource_rejects_escape_even_if_file_exists(tmp_path):
    root = tmp_path / "nested"
    root.mkdir()
    (tmp_path / "secret").touch()
    with pytest.raises(ValueError):
        local_resource(root, "../secret")


def test_spawn_prefers_road_interior_and_rejects_uneven_support(tmp_path, monkeypatch):
    import yaml
    pixels = np.zeros((71, 71), np.uint8)
    pixels[5:66, 5:66] = 254
    Image.fromarray(pixels).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(yaml.safe_dump({"origin": [0, 0, 0], "resolution": .1}))
    poses = [{"T": [.6, 3.5, 1.5], "R": [1, 0, 0, 0]}]
    monkeypatch.setattr(importer, "ground_at", lambda *_: 0.)
    spawn, ground = importer.choose_spawn(tmp_path, poses, None, 0)
    assert spawn[0] >= 2.0
    assert ground == 0
    monkeypatch.setattr(importer, "ground_at", lambda _, x, y, z: x)
    with pytest.raises(ValueError, match="安全地面"):
        importer.choose_spawn(tmp_path, poses, None, 0)
