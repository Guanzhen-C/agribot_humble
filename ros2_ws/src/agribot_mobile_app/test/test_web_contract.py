from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_production_web_bundle_is_checked_in():
    distribution = PACKAGE / "web" / "dist"
    assert (distribution / "index.html").is_file()
    assert (distribution / "manifest.webmanifest").is_file()
    assert (distribution / "sw.js").is_file()
    assert list((distribution / "assets").glob("*.js"))
    assert list((distribution / "assets").glob("*.css"))


def test_android_package_contains_the_offline_web_interface():
    assets = PACKAGE / "android" / "app" / "src" / "main" / "assets" / "web"
    activity = (
        PACKAGE
        / "android"
        / "app"
        / "src"
        / "main"
        / "java"
        / "com"
        / "guanzhen"
        / "agribot"
        / "MainActivity.java"
    ).read_text(encoding="utf-8")
    assert (assets / "index.html").is_file()
    assert list((assets / "assets").glob("*.js"))
    assert list((assets / "assets").glob("*.css"))
    assert "file:///android_asset/web/index.html" in activity
    assert "gatewayIsReachable" in activity
    assert "setAllowFileAccessFromFileURLs(true)" in activity
    assert "setAllowUniversalAccessFromFileURLs(false)" in activity


def test_frontend_uses_guarded_api_not_raw_velocity():
    source = (PACKAGE / "web" / "src" / "App.jsx").read_text(encoding="utf-8")
    api = (PACKAGE / "web" / "src" / "api.js").read_text(encoding="utf-8")
    gateway = (PACKAGE / "agribot_mobile_app" / "gateway_node.py").read_text(
        encoding="utf-8"
    )
    assert "/api/v1/navigation/route" in source
    assert "/api/v1/semantic/plan" in api
    assert "/api/v1/semantic/route" in source
    assert "/api/v1/semantic/execute" in source
    assert "手动规划" in source
    assert "语义导航" in source
    assert "/api/v1/collection/start" in source
    assert "/api/v1/runtime/start" in source
    assert "/cmd_vel" not in source
    assert "离线地图处理" not in source
    assert "控制口令" not in source
    assert "数据盘可用" not in source
    assert "start_camera" not in source
    assert "enable_ntrip" not in source
    assert '"start_camera:=true"' in gateway
    assert '"enable_ntrip:=false"' in gateway
    assert "manual_required" in source
    assert "/localization/initialization_stage" in gateway
    assert "RTK和视觉均失败后" in gateway
    assert "semantic_runner" not in gateway
    assert '"/api/v1/semantic/route"' in gateway
    assert '"provider": "ollama_local"' in gateway


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


def test_new_managed_task_stops_the_previous_process_group_first():
    gateway = (PACKAGE / "agribot_mobile_app" / "gateway_node.py").read_text(
        encoding="utf-8"
    )
    processes = (PACKAGE / "agribot_mobile_app" / "processes.py").read_text(
        encoding="utf-8"
    )
    assert "_task_transition_lock" in gateway
    assert gateway.count("self._stop_active_tasks()") == 3
    assert "assert_exclusive" not in gateway
    assert "os.killpg" in processes
    assert "旧任务尚未完全退出，禁止启动新任务" in gateway
