import math
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.gateway_node import MobileGateway
from agribot_mobile_app.navigation_health import NavigationInputHealth


def ready_health():
    health = NavigationInputHealth()
    health.record("pose", "map", 0.05, now=100.0)
    health.record("local_costmap", "/map", 0.1, now=100.0)
    return health


def test_global_display_publication_is_not_a_motion_heartbeat():
    health = ready_health()
    health.require(True, now=100.1)
    assert all(v["ready"] for v in health.snapshot(now=100.1).values())


@pytest.mark.parametrize("name,now", [("pose", 101.1), ("local_costmap", 103.1)])
def test_input_age_is_diagnostic_and_does_not_abort_saved_path(name, now):
    health = ready_health()
    other = "local_costmap" if name == "pose" else "pose"
    health.record(other, "map", 0.01, now=now)
    health.require(True, now=now)
    sample = health.snapshot(now=now)[name]
    assert sample["ready"] is False
    assert sample["valid"] is True
    assert "接收中断" in sample["reason"]


def test_old_measurement_age_is_reported_without_aborting_saved_path():
    health = ready_health()
    health.record("pose", "map", 1.2, now=100.0)
    health.require(True, now=100.01)
    assert "测量数据已过期" in health.snapshot(now=100.01)["pose"]["reason"]


def test_long_receipt_gap_does_not_abort_if_fusion_stays_ready():
    health = ready_health()
    health.require(True, now=1000.0)
    assert all(not v["ready"] and v["valid"] for v in health.snapshot(now=1000.0).values())


@pytest.mark.parametrize("age", [math.inf, math.nan, -1.0])
def test_invalid_and_future_measurement_stops(age):
    health = ready_health()
    health.record("pose", "map", age, now=100.0)
    with pytest.raises(RuntimeError, match="时间戳异常"):
        health.require(True, now=100.01)


def test_wrong_frame_and_fusion_not_ready_stop():
    health = ready_health()
    health.record("local_costmap", "odom", 0.0, now=100.0)
    with pytest.raises(RuntimeError, match="坐标系不是map"):
        health.require(True, now=100.01)
    with pytest.raises(RuntimeError, match="融合定位未就绪"):
        ready_health().require(False, now=100.01)


def test_diagnostic_counters_and_runtime_reset():
    health = ready_health()
    health.record("pose", "map", 0.01, now=101.2)
    value = health.snapshot(now=101.3)["pose"]
    assert value["samples"] == 2
    assert value["max_gap_sec"] == pytest.approx(1.2)
    assert value["timeout_gaps"] == 1
    assert value["source_age_sec"] == pytest.approx(0.11)
    with pytest.raises(RuntimeError, match="尚未收到"):
        NavigationInputHealth().require(True, now=101.3)


def test_gateway_observes_local_not_the_cached_global_grid(monkeypatch):
    health = ready_health()
    monkeypatch.setattr("agribot_mobile_app.navigation_health.time.monotonic", lambda: 100.1)
    gateway = object.__new__(MobileGateway)
    gateway._lock = threading.RLock()
    gateway._navigation_health = health
    gateway._state = {"pose": {"frame": "map", "x": 0.0}, "localization": {"fusion_ready": True}}
    local = GridData(2, 2, 1.0, -1.0, -1.0, 0.0, bytes(4))
    gateway._grids = {"local_costmap": local, "global_costmap": object()}
    pose, grid = gateway._saved_observe()
    assert grid is local
    assert pose is not gateway._state["pose"]


def test_recovery_requires_fresh_inputs_and_not_just_a_latched_ready_flag(monkeypatch):
    clock = [101.5]
    monkeypatch.setattr("agribot_mobile_app.navigation_health.time.monotonic", lambda: clock[0])
    gateway = object.__new__(MobileGateway)
    gateway._lock = threading.RLock()
    gateway._navigation_health = ready_health()
    gateway._state = {"localization": {"fusion_ready": True}}
    assert not gateway._saved_recovery_ready()
    gateway._navigation_health.record("pose", "map", .01, now=clock[0])
    assert gateway._saved_recovery_ready()
    gateway._state["localization"]["fusion_ready"] = False
    assert not gateway._saved_recovery_ready()


def test_fusion_readiness_gap_limit_is_four_seconds():
    path = Path(__file__).parents[2] / "agribot_hardware_bringup/differential/config/fastlivo_rtk_fusion.yaml"
    params = yaml.safe_load(path.read_text())["fastlivo_rtk_fusion"]["ros__parameters"]
    assert params["odom_ready_timeout_sec"] == 4.0


def test_small_local_snapshot_refresh_and_global_display_budget():
    root = Path(__file__).parents[2] / "agribot_hardware_bringup/differential/config"
    params = yaml.safe_load((root / "nav2_params_differential_fastlivo_mapped.yaml").read_text())
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    glob = params["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "map"
    assert local["always_send_full_costmap"] is True
    assert local["publish_frequency"] == 2.0
    assert local["update_frequency"] == 5.0
    assert glob["publish_frequency"] == 0.2
    assert glob["update_frequency"] == 2.0
    assert local["width"] * local["height"] / local["resolution"]**2 == pytest.approx(25600)


def test_phone_history_is_bounded_and_does_not_grow_when_stationary(monkeypatch):
    gateway = object.__new__(MobileGateway)
    gateway._state = {"paths": {"history": []}}
    gateway._history_received_at = 0.0
    now = [100.0]
    monkeypatch.setattr("agribot_mobile_app.gateway_node.time.monotonic", lambda: now[0])
    for i in range(1400):
        now[0] += 0.51
        gateway._append_display_history(i*0.05, 0.0)
    points = gateway._state["paths"]["history"]
    assert len(points) <= 600
    assert points[0] == [0.0, 0.0]
    assert points[-1] == [1399*0.05, 0.0]
    now[0] += 1.0
    gateway._append_display_history(*points[-1])
    assert gateway._state["paths"]["history"] == points


def test_saved_path_display_is_bounded_without_modifying_follow_path():
    gateway = object.__new__(MobileGateway)
    gateway._lock = threading.RLock()
    gateway._state = {"paths": {}}
    published = []
    gateway._saved_plan_publisher = SimpleNamespace(publish=published.append)
    gateway._saved_runner = SimpleNamespace(message=lambda p: p)
    gateway._touch = lambda: None
    path = [{"x": i*0.05, "y": 0.0, "yaw": 0.0} for i in range(11285)]
    gateway._saved_publish(path)
    assert len(published[0]) <= 1201
    assert published[0][0] == path[0] and published[0][-1] == path[-1]
    assert len(path) == 11285
