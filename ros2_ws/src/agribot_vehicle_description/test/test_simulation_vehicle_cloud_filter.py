import array
import importlib.util
from pathlib import Path
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField


spec = importlib.util.spec_from_file_location(
    "simulation_vehicle_cloud_filter",
    Path(__file__).resolve().parents[1] / "scripts/simulation_vehicle_cloud_filter.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize("bigendian", [False, True])
def test_self_filter_preserves_external_payload_and_measurement_stamp(bigendian):
    cloud = PointCloud2(height=2, width=2, point_step=22, row_step=48, is_bigendian=bigendian)
    cloud.header.frame_id = "lidar_link"
    cloud.header.stamp.sec = 42
    cloud.header.stamp.nanosec = 12345
    cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate(("x", "y", "z"))]
    cloud.fields.extend([PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
                         PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1)])
    cloud.data = array.array("B", bytes(96))
    endian = ">" if bigendian else "<"
    for index, x in enumerate((0.0, 2.0, float("nan"), 3.0)):
        offset = (index // 2) * 48 + (index % 2) * 22
        struct.pack_into(endian + "ffffHf", cloud.data, offset, x, 0.0, 0.0, 7.0, index, 0.01 * index)
    original = bytes(cloud.data)
    result = module.remove_self_returns(cloud, np.eye(3), [0, 0, 0], [-1, -1, -1], [1, 1, 1])
    assert result.header == cloud.header
    assert result.fields == cloud.fields
    assert (result.height, result.width, result.point_step, result.row_step) == (1, 2, 22, 44)
    assert bytes(result.data) == original[22:44] + original[70:92]
    assert bytes(cloud.data) == original


def test_self_filter_uses_base_frame_rotation_and_translation():
    cloud = PointCloud2(height=1, width=2, point_step=12, row_step=24)
    cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate(("x", "y", "z"))]
    cloud.data = array.array("B", struct.pack("<ffffff", 0, 1, 0, 0, 2, 0))
    rotation = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    result = module.remove_self_returns(cloud, rotation, [1, 0, 0], [-0.5] * 3, [0.5] * 3)
    assert result.width == 1
    assert struct.unpack("<fff", result.data) == (0, 2, 0)


def test_self_filter_does_not_hide_malformed_input():
    cloud = PointCloud2(height=1, width=1)
    with pytest.raises(ValueError, match="layout"):
        module.remove_self_returns(cloud, np.eye(3), [0] * 3, [-1] * 3, [1] * 3)


def test_implement_box_does_not_hide_gap_between_vehicle_and_implement():
    cloud = PointCloud2(height=1, width=4, point_step=12, row_step=48)
    cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate(("x", "y", "z"))]
    cloud.data = array.array("B", struct.pack("<12f", 0, 0, 0, -3, 0, 0, -1.5, 0, 0, 3, 0, 0))
    result = module.remove_self_returns(cloud, np.eye(3), [0, 0, 0], [-1]*3, [1]*3,
                                        [[-4, -1, -1, -2, 1, 1]])
    assert result.width == 2
    assert struct.unpack("<6f", result.data) == (-1.5, 0, 0, 3, 0, 0)
