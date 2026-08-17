import importlib.util
from collections import deque
import math
from pathlib import Path

import pytest
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid


SCRIPT_PATH = Path(__file__).parents[1] / "scripts/outdoor_stage_check.py"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "outdoor_stage_check", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def make_checker(module, stage="A"):
    checker = object.__new__(module.OutdoorStageCheck)
    checker.stage = stage
    checker.values = {}
    checker.arrivals = {}
    checker.last_diagnostic = None
    return checker


def healthy_chassis_diagnostic(module):
    status = DiagnosticStatus()
    status.level = DiagnosticStatus.OK
    status.message = "CAN protocol and feedback healthy"
    status.values = [
        KeyValue(key="feedback_fresh", value="true"),
        KeyValue(key="localization_ready", value="true"),
        KeyValue(key="command_active", value="false"),
    ]
    return status


def fill_core_ready_values(checker, stage):
    topics = (
        "/lidar/points",
        "/imu/data",
        "/camera/rgb/image_raw",
        "/rtk/fix",
        "/fastlivo/odometry",
        "/fastlivo_rtk/odometry",
        "/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
    )
    if stage == "B":
        topics += ("/wheel/odometry",)
    checker.values.update({topic: object() for topic in topics})
    checker.values.update(
        {
            "localization_status": (
                "initial localization accepted; map-to-odom correction fixed"
            ),
            "initialization_stage": "ready",
            "initialization_source": "rtk",
            "rtk_seed_ready": True,
            "lidar_ready": True,
            "fusion_ready": True,
            "fixed_active": True,
            "fix_quality": 4,
            "heading_solution": "SOL_COMPUTED,L1_INT",
        }
    )


def test_pose_validation_rejects_nonfinite_and_bad_quaternion():
    module = load_module()
    pose = Pose()
    pose.orientation.w = 1.0
    assert module.pose_is_finite(pose)
    pose.position.x = math.nan
    assert not module.pose_is_finite(pose)
    pose.position.x = 0.0
    pose.orientation.w = 0.0
    assert not module.pose_is_finite(pose)


def test_core_ready_requires_all_localization_gates_and_can_in_stage_b():
    module = load_module()
    stage_a = make_checker(module, "A")
    fill_core_ready_values(stage_a, "A")
    assert stage_a.core_ready()
    stage_a.values["fixed_active"] = False
    assert not stage_a.core_ready()
    stage_a.values["initialization_source"] = "visual"
    assert stage_a.core_ready()

    stage_b = make_checker(module, "B")
    fill_core_ready_values(stage_b, "B")
    assert not stage_b.core_ready()
    stage_b.last_diagnostic = healthy_chassis_diagnostic(module)
    assert stage_b.core_ready()
    stage_b.last_diagnostic.values[-1].value = "true"
    assert not stage_b.core_ready()


def test_topic_rate_uses_recent_arrival_interval():
    module = load_module()
    checker = make_checker(module)
    checker.arrivals["/example"] = deque([10.0, 10.1, 10.2, 10.3])
    assert checker.topic_rate("/example", 20.0) == pytest.approx(10.0)


def test_costmap_bounds_check_uses_metric_origin_and_resolution():
    module = load_module()
    checker = make_checker(module)
    grid = OccupancyGrid()
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -2.0
    grid.info.width = 100
    grid.info.height = 80
    grid.info.resolution = 0.05
    checker.values["/costmap"] = grid
    assert checker.map_contains_pose("/costmap", 0.0, 0.0)
    assert not checker.map_contains_pose("/costmap", 5.0, 0.0)


def test_cli_requires_a_valid_stage_and_sampling_window():
    module = load_module()
    arguments = module.parse_arguments(
        ["--stage", "b", "--timeout", "12", "--sample-duration", "4"]
    )
    assert arguments.stage == "B"
    with pytest.raises(SystemExit):
        module.parse_arguments(
            ["--stage", "A", "--timeout", "3", "--sample-duration", "4"]
        )
