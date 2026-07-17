#!/usr/bin/env python3
"""
Bridge Gazebo IMU data from imu_link frame to base_link frame.

Gazebo's IMU plugin publishes data in the imu_link frame (where gravity
is along the x axis due to the tilted IMU mount). FAST-LIO expects
standard IMU data where gravity is along the negative z axis. This node
transforms the IMU data so that FAST-LIO can initialize correctly.

It also transforms the orientation to remove the IMU mount rotation,
so FAST-LIO sees a near-identity orientation when the robot is level.
"""

import math
import numpy as np
import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def rotation_matrix_from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


def quaternion_from_rotation_matrix(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class ImuFrameBridge(Node):
    def __init__(self):
        super().__init__("imu_frame_bridge")

        # IMU mount rotation: imu_link relative to base_link
        # URDF: rpy=(0, -pi/2, pi)
        self.R_imu_to_base = rotation_matrix_from_rpy(0, -np.pi/2, np.pi)
        # We need base_link->imu_link to transform FROM imu_link TO base_link
        self.R_base_to_imu = self.R_imu_to_base.T  # inverse = transpose for rotation

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Imu, "/imu/data_corrected", imu_qos)
        self.sub = self.create_subscription(Imu, "/imu/data", self.handle_imu, 10)

        self.get_logger().info("Bridging /imu/data (imu_link frame) to /imu/data_corrected (base_link frame)")

    def handle_imu(self, msg):
        # Transform linear acceleration from imu_link to base_link
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        accel_base = self.R_imu_to_base @ accel

        # Transform angular velocity from imu_link to base_link
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        gyro_base = self.R_imu_to_base @ gyro

        # Transform orientation: base_link = imu_link_rot * R_base_to_imu
        # msg.orientation is in imu_link frame (relative to world)
        # We want orientation in base_link frame
        # R_base_in_world = R_imu_in_world @ R_imu_to_base
        R_imu_world = np.array([
            [1-2*(msg.orientation.y**2 + msg.orientation.z**2),
             2*(msg.orientation.x*msg.orientation.y - msg.orientation.w*msg.orientation.z),
             2*(msg.orientation.x*msg.orientation.z + msg.orientation.w*msg.orientation.y)],
            [2*(msg.orientation.x*msg.orientation.y + msg.orientation.w*msg.orientation.z),
             1-2*(msg.orientation.x**2 + msg.orientation.z**2),
             2*(msg.orientation.y*msg.orientation.z - msg.orientation.w*msg.orientation.x)],
            [2*(msg.orientation.x*msg.orientation.z - msg.orientation.w*msg.orientation.y),
             2*(msg.orientation.y*msg.orientation.z + msg.orientation.w*msg.orientation.x),
             1-2*(msg.orientation.x**2 + msg.orientation.y**2)],
        ])
        R_base_world = R_imu_world @ self.R_imu_to_base
        quat_base = quaternion_from_rotation_matrix(R_base_world)

        out = Imu()
        out.header = msg.header
        out.header.frame_id = "base_link"
        out.linear_acceleration.x = accel_base[0]
        out.linear_acceleration.y = accel_base[1]
        out.linear_acceleration.z = accel_base[2]
        out.angular_velocity.x = gyro_base[0]
        out.angular_velocity.y = gyro_base[1]
        out.angular_velocity.z = gyro_base[2]
        out.orientation.x = quat_base[0]
        out.orientation.y = quat_base[1]
        out.orientation.z = quat_base[2]
        out.orientation.w = quat_base[3]
        # Copy covariances
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.orientation_covariance = msg.orientation_covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuFrameBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
