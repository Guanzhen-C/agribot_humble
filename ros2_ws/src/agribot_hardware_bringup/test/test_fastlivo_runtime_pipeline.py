from pathlib import Path

import yaml


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
FASTLIVO_ROOT = WORKSPACE_SRC / "FAST-LIVO2"


def test_sensor_callbacks_are_decoupled_from_estimation():
    main_source = (FASTLIVO_ROOT / "src/main.cpp").read_text()
    mapper_source = (FASTLIVO_ROOT / "src/LIVMapper.cpp").read_text()

    assert "MultiThreadedExecutor" in main_source
    assert "callback_executor.add_node(mapper.getNode())" in main_source
    assert "rclcpp::spin_some(this->node)" not in mapper_source
    assert "lidarPreprocessLoop" in mapper_source
    assert "max_lidar_input_buffer_size" in mapper_source


def test_c16_runtime_queues_are_bounded_and_lidar_is_reliable():
    parameters = yaml.safe_load(
        (FASTLIVO_ROOT / "config/agribot_c16_astra.yaml").read_text()
    )["/**"]["ros__parameters"]["common"]
    mapper_source = (FASTLIVO_ROOT / "src/LIVMapper.cpp").read_text()

    assert parameters["lidar_subscription_queue_depth"] == 4
    assert parameters["max_lidar_input_buffer_size"] == 4
    assert parameters["max_lidar_buffer_size"] == 2
    assert parameters["max_imu_buffer_size"] == 300
    assert parameters["max_image_buffer_size"] == 3
    assert "lidar_qos.reliable().durability_volatile()" in mapper_source
    assert "minimum_interval - 1e-6" in mapper_source


def test_real_fastlivo_uses_scan_start_cloud_and_measured_image_offset():
    lidar_parameters = yaml.safe_load(
        (WORKSPACE_SRC / "agribot_hardware_bringup/config/c16.yaml").read_text()
    )["lslidar_driver_node"]["ros__parameters"]
    fastlivo_parameters = yaml.safe_load(
        (FASTLIVO_ROOT / "config/agribot_c16_astra.yaml").read_text()
    )["/**"]["ros__parameters"]
    mapper_source = (FASTLIVO_ROOT / "src/LIVMapper.cpp").read_text()

    assert lidar_parameters["use_first_point_time"] is True
    assert fastlivo_parameters["time_offset"]["img_time_offset"] == -0.0033
    assert "meas.lidar_frame_beg_time = lid_header_time_buffer.front()" in mapper_source
    assert "meas.lidar_frame_beg_time +" in mapper_source
    assert "points.back().curvature" in mapper_source
