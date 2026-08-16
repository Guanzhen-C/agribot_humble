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


def test_map_view_uses_live_vehicle_and_navigation_outputs():
    view = (PACKAGE / "web" / "src" / "MapView.jsx").read_text(encoding="utf-8")
    config = (PACKAGE / "config" / "mobile_gateway.yaml").read_text(
        encoding="utf-8"
    )
    assert "state?.paths?.history" in view
    assert "state?.paths?.global" in view
    assert "state?.paths?.local" in view
    assert "state?.vehicle?.footprint" in view
    assert "local_plan_topic: /transformed_global_plan" in config
    assert "trajectory_topic: /fastlivo_rtk/path" in config
    assert "footprint_topic: /local_costmap/published_footprint" in config


def test_gateway_checks_sensor_publishers_without_subscribing_to_frames():
    source = (PACKAGE / "agribot_mobile_app" / "gateway_node.py").read_text(
        encoding="utf-8"
    )
    launch = (PACKAGE / "launch" / "mobile_app.launch.py").read_text(
        encoding="utf-8"
    )
    assert "get_publishers_info_by_topic" in source
    assert '"topics"' in source
    assert '"/agribot/mobile_sensor_rates"' not in source
    assert "DiagnosticArray" not in source
    assert "PointCloud2" not in source
    assert "sensor_rate_monitor" not in launch


def test_mobile_launch_isolates_ros_but_keeps_http_on_the_lan():
    launch = (PACKAGE / "launch" / "mobile_app.launch.py").read_text(
        encoding="utf-8"
    )
    config = (PACKAGE / "config" / "mobile_gateway.yaml").read_text(
        encoding="utf-8"
    )
    assert '"ros_localhost_only"' in launch
    assert 'default_value="1"' in launch
    assert '"ROS_LOCALHOST_ONLY"' in launch
    assert "http_host: 0.0.0.0" in config
