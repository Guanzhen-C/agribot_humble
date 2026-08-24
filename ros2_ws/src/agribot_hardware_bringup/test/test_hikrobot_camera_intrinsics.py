from pathlib import Path

import pytest
import yaml


HARDWARE_PACKAGE = Path(__file__).parents[1]
HIKROBOT_PACKAGE = HARDWARE_PACKAGE.parent / "hikrobot_mvs_ros2"


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_hikrobot_camera_info_and_fastlivo_intrinsics_are_consistent():
    driver = load_yaml(HIKROBOT_PACKAGE / "config" / "mv_cu013_a0uc.yaml")["/**"][
        "ros__parameters"
    ]
    camera_info = load_yaml(
        HIKROBOT_PACKAGE
        / "config"
        / "agribot_hikrobot_right_camera.yaml"
    )
    fastlivo = load_yaml(
        HARDWARE_PACKAGE / "config" / "fastlivo_hikrobot_mv_cu013.yaml"
    )["/**"]["ros__parameters"]["camera"]

    assert driver["camera_info_url"] == (
        "package://hikrobot_mvs_ros2/config/"
        "agribot_hikrobot_right_camera.yaml"
    )
    assert camera_info["camera_name"] == driver["camera_name"]
    assert camera_info["image_width"] == fastlivo["width"] == 1280
    assert camera_info["image_height"] == fastlivo["height"] == 1024
    assert camera_info["distortion_model"] == "plumb_bob"

    matrix = camera_info["camera_matrix"]["data"]
    distortion = camera_info["distortion_coefficients"]["data"]
    assert [fastlivo[key] for key in ("fx", "fy", "cx", "cy")] == pytest.approx(
        [matrix[0], matrix[4], matrix[2], matrix[5]], abs=1.0e-8
    )
    assert [fastlivo[key] for key in ("d0", "d1", "d2", "d3")] == pytest.approx(
        distortion[:4], abs=1.0e-8
    )


def test_ackermann_status_enables_only_completed_camera_calibrations():
    status = load_yaml(
        HARDWARE_PACKAGE
        / "ackermann"
        / "config"
        / "hikrobot_camera_calibration_status.yaml"
    )

    assert status["camera_model"] == "MV-CU013-A0UC"
    assert status["serial_number"] == "DB0447659"
    assert status["lens_installed"] is True
    assert status["intrinsics_calibrated"] is True
    assert status["lidar_camera_extrinsics_calibrated"] is True
    assert status["lidar_camera_extrinsics_source"] == (
        "manual_measurement_2026_08_24"
    )
    assert status["image_time_offset_calibrated"] is True
    assert status["image_time_offset_source"] == (
        "pps_phase_measurement_2026_08_22"
    )
