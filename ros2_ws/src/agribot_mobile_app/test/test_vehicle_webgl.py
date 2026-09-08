import hashlib
from io import BytesIO
from http import HTTPStatus
from pathlib import Path

import brotli
import pytest

from agribot_mobile_app.vehicle_webgl import (
    VehicleWebGlCatalog,
    VehicleWebGlError,
    VehicleWebGlNotFound,
    VehicleWebGlRangeError,
    accepts_content_encoding,
    parse_single_byte_range,
    plan_asset_response,
    stream_asset_response,
)


def webgl_build(tmp_path: Path) -> Path:
    root = tmp_path / "WebGL"
    build = root / "Build"
    build.mkdir(parents=True)
    (root / "index.html").write_text(
        'companyName: "Test"\n'
        'productName: "Vehicle"\n'
        'productVersion: "1.2.3"\n',
        encoding="utf-8",
    )
    (build / "Vehicle.loader.js").write_text("loader", encoding="utf-8")
    (build / "Vehicle.framework.js.br").write_bytes(b"framework")
    (build / "Vehicle.wasm.br").write_bytes(b"wasm")
    (build / "Vehicle.data.br").write_bytes(b"data")
    return root


def test_manifest_versions_and_describes_external_build(tmp_path):
    catalog = VehicleWebGlCatalog(webgl_build(tmp_path))

    manifest = catalog.manifest()

    assert manifest["available"] is True
    assert manifest["product"] == {
        "company": "Test",
        "name": "Vehicle",
        "version": "1.2.3",
    }
    assert manifest["unity"]["code"] == "Build/Vehicle.wasm.br"
    assert manifest["raw_download"]["header"] == "X-Agribot-Raw-Asset"
    assert manifest["raw_download"]["hash_scope"] == "stored_file_bytes"
    assert manifest["total_bytes"] > 0
    assert len(manifest["version"]) == 16
    wasm = next(
        item for item in manifest["files"] if item["path"].endswith(".wasm.br")
    )
    assert wasm["content_type"] == "application/wasm"
    assert wasm["content_encoding"] == "br"
    assert wasm["sha256"] == hashlib.sha256(b"wasm").hexdigest()


def test_manifest_version_changes_with_content(tmp_path):
    root = webgl_build(tmp_path)
    catalog = VehicleWebGlCatalog(root)
    first = catalog.manifest()["version"]

    (root / "Build" / "Vehicle.data.br").write_bytes(b"new-data")

    assert catalog.manifest()["version"] != first


@pytest.mark.parametrize(
    ("header", "expected"),
    [
        (None, False),
        ("gzip, deflate", False),
        ("gzip, br", True),
        ("br;q=0", False),
        ("*;q=0.5", True),
    ],
)
def test_accept_encoding_negotiation(header, expected):
    assert accepts_content_encoding(header, "br") is expected


def test_browser_falls_back_to_cached_identity_brotli_asset(tmp_path):
    root = webgl_build(tmp_path)
    payload = b"valid wasm bytes" * 128
    compressed = brotli.compress(payload)
    (root / "Build" / "Vehicle.wasm.br").write_bytes(compressed)
    catalog = VehicleWebGlCatalog(root, tmp_path / "decoded-cache")

    compressed_asset = catalog.browser_asset_descriptor(
        "Build/Vehicle.wasm.br",
        "gzip, deflate, br",
    )
    identity_asset = catalog.browser_asset_descriptor(
        "Build/Vehicle.wasm.br",
        "gzip, deflate",
    )
    cached_asset = catalog.browser_asset_descriptor(
        "Build/Vehicle.wasm.br",
        None,
    )

    assert compressed_asset.path == root / "Build" / "Vehicle.wasm.br"
    assert compressed_asset.content_encoding == "br"
    assert identity_asset.path.read_bytes() == payload
    assert identity_asset.content_encoding is None
    assert identity_asset.size == len(payload)
    assert cached_asset.path == identity_asset.path


def test_asset_lookup_rejects_directory_traversal(tmp_path):
    catalog = VehicleWebGlCatalog(webgl_build(tmp_path))

    with pytest.raises(VehicleWebGlError):
        catalog.asset("../secret")
    with pytest.raises(VehicleWebGlError):
        catalog.asset("/etc/passwd")
    with pytest.raises(VehicleWebGlNotFound):
        catalog.asset("Build//Vehicle.wasm.br")
    with pytest.raises(VehicleWebGlNotFound):
        catalog.asset("Build/.hidden")


def test_asset_lookup_is_limited_to_manifest_and_rejects_symlinks(tmp_path):
    root = webgl_build(tmp_path)
    catalog = VehicleWebGlCatalog(root)
    catalog.manifest()
    (root / ".secret").write_text("secret", encoding="utf-8")
    (root / "Build" / "alias.br").symlink_to(
        root / "Build" / "Vehicle.wasm.br"
    )

    with pytest.raises(VehicleWebGlNotFound):
        catalog.asset(".secret")
    with pytest.raises(VehicleWebGlNotFound):
        catalog.asset("Build/alias.br")


@pytest.mark.parametrize(
    ("header", "size", "expected"),
    [
        (None, 10, None),
        ("bytes=0-3", 10, (0, 3)),
        ("bytes=4-", 10, (4, 9)),
        ("bytes=-4", 10, (6, 9)),
        ("bytes=8-99", 10, (8, 9)),
    ],
)
def test_single_byte_range_parsing(header, size, expected):
    assert parse_single_byte_range(header, size) == expected


@pytest.mark.parametrize(
    "header",
    ["bytes=", "bytes=9-4", "bytes=10-", "bytes=0-1,4-5", "items=0-1"],
)
def test_invalid_or_unsupported_ranges_are_rejected(header):
    with pytest.raises(VehicleWebGlRangeError) as caught:
        parse_single_byte_range(header, 10)
    assert caught.value.size == 10


def test_raw_download_preserves_stored_brotli_bytes_without_encoding(tmp_path):
    root = webgl_build(tmp_path)
    stored = b"not-decoded-brotli-bytes"
    (root / "Build" / "Vehicle.wasm.br").write_bytes(stored)
    asset = VehicleWebGlCatalog(root).asset_descriptor("Build/Vehicle.wasm.br")

    browser = plan_asset_response(asset)
    raw = plan_asset_response(asset, raw_download=True)
    output = BytesIO()
    stream_asset_response(raw, output, chunk_size=3)

    assert browser.content_type == "application/wasm"
    assert browser.content_encoding == "br"
    assert raw.content_type == "application/octet-stream"
    assert raw.content_encoding is None
    assert raw.etag != browser.etag
    assert output.getvalue() == stored
    assert asset.sha256 == hashlib.sha256(stored).hexdigest()


def test_partial_and_conditional_asset_responses(tmp_path):
    root = webgl_build(tmp_path)
    stored = b"0123456789"
    (root / "Build" / "Vehicle.data.br").write_bytes(stored)
    asset = VehicleWebGlCatalog(root).asset_descriptor("Build/Vehicle.data.br")

    partial = plan_asset_response(asset, range_header="bytes=2-5")
    output = BytesIO()
    stream_asset_response(partial, output, chunk_size=2)
    assert partial.status == HTTPStatus.PARTIAL_CONTENT
    assert partial.content_range == "bytes 2-5/10"
    assert partial.length == 4
    assert output.getvalue() == b"2345"

    unchanged = plan_asset_response(asset, if_none_match=partial.etag)
    assert unchanged.status == HTTPStatus.NOT_MODIFIED
    assert unchanged.length == 0

    full = plan_asset_response(
        asset,
        range_header="bytes=2-5",
        if_range='"stale"',
    )
    assert full.status == HTTPStatus.OK
    assert full.length == len(stored)


def test_missing_build_is_reported_without_failing_gateway(tmp_path):
    manifest = VehicleWebGlCatalog(tmp_path / "missing").public_manifest()

    assert manifest["available"] is False
    assert manifest["files"] == []
    assert "不存在" in manifest["message"]
