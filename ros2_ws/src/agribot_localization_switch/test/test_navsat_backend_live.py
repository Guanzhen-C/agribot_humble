"""Run the actual GNSS/INS binary against synthetic sensors, never hardware."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt8
import yaml


def test_real_navsat_map_origin_lever_arm_and_quality_gate(tmp_path):
    prefix = Path(get_package_prefix("agribot_hardware_bringup"))
    binary = prefix/"lib/agribot_hardware_bringup/rtk_eskf_localization"
    if not binary.is_file():
        pytest.skip("Build agribot_hardware_bringup first")
    share = Path(get_package_share_directory("agribot_hardware_bringup"))
    cloud = tmp_path/"map.pcd"
    cloud.write_bytes(b"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n")
    fingerprint = 14695981039346656037
    for byte in cloud.read_bytes():fingerprint = ((fingerprint ^ byte)*1099511628211) & ((1 << 64)-1)
    geo = {"schema_version":1, "created_at_utc":"2026-10-30T00:00:00Z",
        "map":{"id":"map", "pcd":"map.pcd", "fingerprint_fnv1a64":f"{fingerprint:016x}"},
        "reference":{"latitude_deg":38., "longitude_deg":115., "altitude_m":20.},
        "map_from_enu":{"xyz":[4.,5.,0.], "rpy":[0.,0.,0.]},
        "calibration":{"horizontal_rmse_m":.01, "yaw_rmse_deg":.1, "yaw_validation_passed":True,
                       "sample_count":10,"version":"test", "hash":"test"}}
    geofile = tmp_path/"georef.yaml";geofile.write_text(yaml.safe_dump(geo))
    params = {"use_sim_time":False, "required_fix_quality":4,
        "quality_topic":"/switch_test/quality", "imu_topic":"/switch_test/imu",
        "pose_topic":"/switch_test/fix", "rtk_heading_covariance_topic":"/switch_test/heading",
        "output_odom_topic":"/switch_test/odometry", "publish_raw_pose":False,
        "georeference_file":str(geofile), "map_file":str(cloud), "imu_processing_delay_sec":.25}
    config = tmp_path/"override.yaml"
    config.write_text(yaml.safe_dump({"rtk_eskf_localization":{"ros__parameters":params}}))
    log = (tmp_path/"backend.log").open("w+")
    process = subprocess.Popen([str(binary), "--ros-args", "--params-file",str(share/"config/kf_gins_n300pro.yaml"),
        "--params-file",str(share/"differential/config/kf_gins_n300pro.yaml"),
        "--params-file",str(config)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init();node = rclpy.create_node("synthetic_navsat_test")
    imu = node.create_publisher(Imu,"/switch_test/imu",10)
    fix = node.create_publisher(NavSatFix,"/switch_test/fix",10)
    heading = node.create_publisher(PoseWithCovarianceStamped,"/switch_test/heading",10)
    quality = node.create_publisher(UInt8,"/switch_test/quality",10)
    outputs = []
    subscription = node.create_subscription(Odometry,"/switch_test/odometry",outputs.append,100)
    try:
        start = time.monotonic();last_fix = -1.
        while time.monotonic()-start < 6.0:
            elapsed = time.monotonic()-start
            assert process.poll() is None, "NavSat backend exited: "+(tmp_path/"backend.log").read_text()
            stamp = node.get_clock().now().to_msg()
            quality.publish(UInt8(data=5 if 3.0 <= elapsed < 4.5 else 4))
            sample = Imu();sample.header.stamp = stamp;sample.header.frame_id = "imu_link"
            sample.orientation.w = 1.;sample.linear_acceleration.z = 9.80665
            imu.publish(sample)
            if elapsed-last_fix > .1:
                last_fix = elapsed
                yaw = PoseWithCovarianceStamped();yaw.header.stamp = stamp
                yaw.pose.pose.orientation.w = 1.;yaw.pose.covariance[35] = math.radians(1)**2
                heading.publish(yaw)
                measurement = NavSatFix();measurement.header.stamp = stamp;measurement.status.status = 2
                # A stationary base at map(4,5,0), master antenna 0.44m east,
                # 0.32m north and 0.84m above the map reference altitude.
                measurement.latitude = 38.+math.degrees(.32/6378137.)
                measurement.longitude = 115.+math.degrees((.44+(50. if 2.5 < elapsed < 4.5 else 0.))/
                    (6378137.*math.cos(math.radians(38.))))
                measurement.altitude = 20.84
                measurement.position_covariance[0] = measurement.position_covariance[4] = .03**2
                measurement.position_covariance[8] = .06**2
                if 2.5 < elapsed < 3.0:
                    measurement.position_covariance[4] = float("nan")
                fix.publish(measurement)
            rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.009)
        assert len(outputs) > 100
        assert all(abs(sample.pose.pose.position.x-4.) < .3 for sample in outputs)
        last = outputs[-1]
        assert last.header.frame_id == "map" and last.child_frame_id == "base_link"
        assert last.pose.pose.position.x == pytest.approx(4.,abs=.2)
        assert last.pose.pose.position.y == pytest.approx(5.,abs=.2)
        assert last.pose.pose.position.z == pytest.approx(0.,abs=.4)
    finally:
        node.destroy_subscription(subscription);node.destroy_node();rclpy.shutdown()
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        log.close()
