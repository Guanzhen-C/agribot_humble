import array
import importlib.util
from pathlib import Path
import struct

import pytest
from rclpy.qos import LivelinessPolicy, QoSCompatibility, QoSProfile, ReliabilityPolicy, qos_check_compatible
from sensor_msgs.msg import PointCloud2, PointField


spec = importlib.util.spec_from_file_location(
    "simulation_lidar_time_adapter",
    Path(__file__).resolve().parents[1] / "scripts/simulation_lidar_time_adapter.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_output_is_compatible_with_both_lio_frontends_and_bounded():
    assert module.LOCALIZATION_QOS.depth == 2
    assert module.LOCALIZATION_QOS.reliability == ReliabilityPolicy.RELIABLE
    for reliability in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
        # Resolve middleware-default liveliness for this static check.
        subscription = QoSProfile(
            depth=4, reliability=reliability, liveliness=LivelinessPolicy.AUTOMATIC)
        compatibility, _reason = qos_check_compatible(module.LOCALIZATION_QOS, subscription)
        assert compatibility == QoSCompatibility.OK


@pytest.mark.parametrize("bigendian", [False, True])
def test_same_sampling_instant_and_payload_preserved(bigendian):
    endian = ">" if bigendian else "<"
    cloud = PointCloud2(height=2, width=2, point_step=22, row_step=48,
                        is_bigendian=bigendian)
    cloud.header.frame_id = "lidar_link"
    cloud.header.stamp.sec = 5
    cloud.header.stamp.nanosec = 250_000_000
    cloud.fields = [PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1)]
    cloud.data = array.array("B", bytes(range(96)))
    for row in range(2):
        for col in range(2):
            struct.pack_into(endian + "f", cloud.data, row * 48 + col * 22 + 18, 0.0)
    original = bytes(cloud.data)
    result = module.instantaneous_scan(cloud, 0.1)
    assert cloud.header.stamp.nanosec == 250_000_000
    assert bytes(cloud.data) == original
    assert result.header.frame_id == cloud.header.frame_id
    assert result.header.stamp.sec == 5
    assert result.header.stamp.nanosec == 150_000_000
    modified = set()
    for row in range(2):
        for col in range(2):
            offset = row * 48 + col * 22 + 18
            time = struct.unpack_from(endian + "f", result.data, offset)[0]
            assert time > 0.0
            assert 5.15 + time == pytest.approx(5.25, abs=3e-9)
            modified.update(range(offset, offset + 4))
    assert all(result.data[i] == original[i] for i in range(96) if i not in modified)


def test_empty_cloud_and_invalid_layout():
    empty = PointCloud2()
    assert module.instantaneous_scan(empty, 0.1) is empty
    with pytest.raises(ValueError):
        module.instantaneous_scan(empty, 0.0)
    with pytest.raises(ValueError):
        module.instantaneous_scan(PointCloud2(height=1, width=1), 0.1)


def test_startup_does_not_generate_a_negative_stamp():
    cloud = PointCloud2(height=1, width=1, point_step=4, row_step=4)
    cloud.header.stamp.nanosec = 50_000_000
    cloud.fields = [PointField(name="time", offset=0, datatype=PointField.FLOAT32, count=1)]
    cloud.data = array.array("B", bytes(4))
    result = module.instantaneous_scan(cloud, 0.1)
    assert result.header.stamp.sec == 0 and result.header.stamp.nanosec == 0
    assert struct.unpack("<f", result.data)[0] == pytest.approx(0.05)
