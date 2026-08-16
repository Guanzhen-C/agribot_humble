from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_production_web_bundle_is_checked_in():
    distribution = PACKAGE / "web" / "dist"
    assert (distribution / "index.html").is_file()
    assert (distribution / "manifest.webmanifest").is_file()
    assert (distribution / "sw.js").is_file()
    assert list((distribution / "assets").glob("*.js"))
    assert list((distribution / "assets").glob("*.css"))


def test_frontend_uses_guarded_api_not_raw_velocity():
    source = (PACKAGE / "web" / "src" / "App.jsx").read_text(encoding="utf-8")
    assert "/api/v1/navigation/route" in source
    assert "/api/v1/collection/start" in source
    assert "/api/v1/runtime/start" in source
    assert "/cmd_vel" not in source
