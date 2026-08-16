from pathlib import Path

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
