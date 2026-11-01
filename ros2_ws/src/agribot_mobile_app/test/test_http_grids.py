import gzip
from http.client import HTTPConnection
import json
from threading import Thread
from types import SimpleNamespace

import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.gateway_node import GatewayHttpServer


@pytest.mark.parametrize("path", ["/assets/missing.js", "/assets/missing.css", "/downloads/missing.apk"])
def test_missing_static_asset_is_not_cached_as_html(tmp_path, path):
    (tmp_path / "index.html").write_text("<html>console</html>")
    logger = SimpleNamespace(debug=lambda *_: None, error=lambda *_: None)
    gateway = SimpleNamespace(semantic_service_url="", get_logger=lambda: logger)
    server = GatewayHttpServer(("127.0.0.1", 0), gateway, tmp_path)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = HTTPConnection("127.0.0.1", server.server_port, timeout=5)
    try:
        connection.request("GET", path)
        response = connection.getresponse()
        assert response.status == 404
        assert response.getheader("Cache-Control") == "no-store"
        assert "application/json" in response.getheader("Content-Type")
        assert b"<html>" not in response.read()
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


@pytest.mark.parametrize("encoding,compressed", [("gzip, deflate", True), ("GZIP; q=1", True), ("gzip;q=0", False), ("gzip;q=bad", False), ("identity", False)])
def test_large_map_http_compression_preserves_every_grid_byte(tmp_path, encoding, compressed):
    grid = GridData(300, 300, 0.05, -3, 4, 0, bytes([0, 100, 255]) * 30000)
    logger = SimpleNamespace(debug=lambda *_: None, error=lambda *_: None)
    gateway = SimpleNamespace(live_grid=lambda _: grid, semantic_service_url="http://127.0.0.1:8090", get_logger=lambda: logger)
    server = GatewayHttpServer(("127.0.0.1", 0), gateway, tmp_path)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = HTTPConnection("127.0.0.1", server.server_port, timeout=5)
    try:
        connection.request("GET", "/api/v1/grid?layer=map", headers={"Accept-Encoding": encoding})
        response = connection.getresponse()
        payload = response.read()
        assert response.status == 200
        assert int(response.getheader("Content-Length")) == len(payload)
        assert response.getheader("Vary") == "Accept-Encoding"
        assert (response.getheader("Content-Encoding") == "gzip") == compressed
        if compressed:
            assert len(payload) < 2000
            payload = gzip.decompress(payload)
        assert json.loads(payload) == grid.payload("map")
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
