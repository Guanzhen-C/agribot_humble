from pathlib import Path
import hashlib
import json

import pytest
import yaml

from agribot_mobile_app.catalog import (
    BagCatalog,
    CatalogError,
    MapCatalog,
    grid_from_nav2_yaml,
    read_pgm,
    validated_identifier,
)


def write_p5(path: Path, width: int, height: int, pixels: bytes):
    path.write_bytes(f"P5\n{width} {height}\n255\n".encode("ascii") + pixels)


def signed(values):
    return [value if value < 128 else value - 256 for value in values]


def test_reads_binary_pgm_and_converts_nav2_rows(tmp_path):
    image = tmp_path / "corridor.pgm"
    write_p5(image, 2, 2, bytes([0, 255, 128, 255]))
    map_yaml = tmp_path / "corridor.yaml"
    map_yaml.write_text(
        yaml.safe_dump(
            {
                "image": image.name,
                "resolution": 0.05,
                "origin": [-2.0, 1.0, 0.1],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        ),
        encoding="utf-8",
    )

    width, height, pixels = read_pgm(image)
    assert (width, height, pixels) == (2, 2, bytes([0, 255, 128, 255]))

    grid = grid_from_nav2_yaml(map_yaml)
    assert (grid.width, grid.height, grid.resolution) == (2, 2, 0.05)
    assert (grid.origin_x, grid.origin_y, grid.origin_yaw) == (-2.0, 1.0, 0.1)
    assert signed(grid.data) == [-1, 0, 100, 0]
    payload = grid.payload("map")
    assert payload["encoding"] == "int8-base64"


def test_map_catalog_only_lists_complete_nav2_maps(tmp_path):
    write_p5(tmp_path / "site.pgm", 1, 1, b"\xff")
    (tmp_path / "site.pcd").write_bytes(b"pcd")
    (tmp_path / "site_georeference.yaml").write_text("schema: 1\n")
    (tmp_path / "site.yaml").write_text(
        "image: site.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    (tmp_path / "unrelated.yaml").write_text("value: 1\n")

    catalog = MapCatalog(tmp_path)
    assert catalog.list()[0]["id"] == "site"
    assert catalog.list()[0]["has_3d"] is True
    assert catalog.list()[0]["has_georeference"] is True
    assert catalog.map_base("site") == tmp_path / "site"


@pytest.mark.parametrize("negate", [0, 1])
def test_map_lookup_matches_original_formula_for_every_gray_value(tmp_path, negate):
    pixels = bytes(range(256)) * 2
    write_p5(tmp_path / "levels.pgm", 256, 2, pixels)
    path = tmp_path / "levels.yaml"
    path.write_text(yaml.safe_dump({"image": "levels.pgm", "resolution": 0.05, "origin": [0, 0, 0], "negate": negate, "occupied_thresh": 0.65, "free_thresh": 0.196}))
    expected = []
    for pixel in pixels:
        probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
        expected.append(100 if probability > 0.65 else 0 if probability < 0.196 else -1)
    assert signed(grid_from_nav2_yaml(path).data) == expected


def test_bag_catalog_requires_metadata_and_rejects_traversal(tmp_path):
    complete = tmp_path / "map_test_20260101_120000"
    complete.mkdir()
    (complete / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    incomplete = tmp_path / "partial"
    incomplete.mkdir()

    catalog = BagCatalog(tmp_path)
    assert [item["id"] for item in catalog.list()] == [complete.name]
    assert catalog.path(complete.name) == complete
    with pytest.raises(CatalogError):
        catalog.path("../outside")


@pytest.mark.parametrize("value", ["", "../map", "map name", "a" * 65])
def test_identifier_validation(value):
    with pytest.raises(CatalogError):
        validated_identifier(value)


def preset_fixture(root):
    (root / "field.yaml").write_text("image: field.pgm\nresolution: 0.05\norigin: [-1, -2, 0]\n")
    (root / "field_georeference.yaml").write_text("map: {id: field}\n")
    (root / "field.pgm").write_bytes(b"not read for route lookup")
    document = {
        "schema_version": 1, "map_id": "field", "frame_id": "map",
        "map_binding": {"georeference_sha256": hashlib.sha256((root / "field_georeference.yaml").read_bytes()).hexdigest(), "resolution": 0.05, "origin": [-1, -2, 0]},
        "routes": [{"id": "scan", "label": "扫描路线", "vehicle_type": "differential", "poses": [{"x": 0, "y": 0, "yaw": 0}, {"x": 1, "y": 2, "yaw": 1}]}],
    }
    path = root / "field_routes.json"
    path.write_text(json.dumps(document))
    return document, path


def test_route_presets_are_map_bound_but_allow_pgm_cleanup(tmp_path):
    preset_fixture(tmp_path)
    catalog = MapCatalog(tmp_path)
    assert catalog.route_presets("field")["routes"][0]["poses"][1]["x"] == 1
    (tmp_path / "field.pgm").write_bytes(b"manually changed image")
    assert len(catalog.route_presets("field")["routes"]) == 1
    assert catalog.list()[0]["has_route_presets"] is True
    (tmp_path / "field_georeference.yaml").write_text("map: {id: changed}\n")
    with pytest.raises(CatalogError, match="地理配准已改变"):
        catalog.route_presets("field")


@pytest.mark.parametrize("change", ["map", "geometry", "frame", "duplicate", "nan", "bool", "too_many", "vehicle", "malformed"])
def test_invalid_route_presets_fail_closed(tmp_path, change):
    doc, path = preset_fixture(tmp_path)
    if change == "map":
        doc["map_id"] = "other"
    elif change == "geometry":
        doc["map_binding"]["origin"][0] = 3
    elif change == "frame":
        doc["frame_id"] = "enu"
    elif change == "duplicate":
        doc["routes"].append(doc["routes"][0])
    elif change == "nan":
        doc["routes"][0]["poses"][0]["yaw"] = float("nan")
    elif change == "bool":
        doc["routes"][0]["poses"][0]["x"] = True
    elif change == "too_many":
        doc["routes"][0]["poses"] *= 56
    elif change == "vehicle":
        doc["routes"][0]["vehicle_type"] = "unknown"
    elif change == "malformed":
        doc["routes"] = [None]
    path.write_text(json.dumps(doc))
    with pytest.raises(CatalogError):
        MapCatalog(tmp_path).route_presets("field")


def test_preset_missing_and_symlink_handling(tmp_path):
    _, path = preset_fixture(tmp_path)
    path.unlink()
    assert MapCatalog(tmp_path).route_presets("field")["routes"] == []
    outside = tmp_path.parent / "external_route.json"
    outside.write_text("{}")
    path.symlink_to(outside)
    with pytest.raises(CatalogError):
        MapCatalog(tmp_path).route_presets("field")
