#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SetModelConfiguration
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw))


class AckermannGazeboSim(Node):
    def __init__(self) -> None:
        super().__init__("ackermann_gazebo_sim")

        self.entity_name = self.declare_parameter("entity_name", "ackermann_scout").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/nav2/cmd_vel").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.joint_state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.bridge_cmd_vel_topic = self.declare_parameter("bridge_cmd_vel_topic", "/cmd_vel").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.reference_frame = self.declare_parameter("reference_frame", "world").value

        self.wheelbase = float(self.declare_parameter("wheelbase", 0.498).value)
        self.track_width = float(self.declare_parameter("track_width", 0.58306).value)
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.16459).value)
        self.max_steering_angle = float(self.declare_parameter("max_steering_angle", 0.6).value)
        self.max_steering_rate = float(self.declare_parameter("max_steering_rate", 1.2).value)
        self.max_speed = float(self.declare_parameter("max_speed", 1.0).value)
        self.max_accel = float(self.declare_parameter("max_accel", 1.5).value)
        self.command_timeout = float(self.declare_parameter("command_timeout", 1.5).value)
        self.update_rate = float(self.declare_parameter("update_rate", 30.0).value)
        self.min_speed_for_curvature = float(
            self.declare_parameter("min_speed_for_curvature", 0.05).value
        )
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.publish_odom = bool(self.declare_parameter("publish_odom", True).value)
        self.model_z = float(self.declare_parameter("model_z", 0.24).value)
        self.apply_joint_model_configuration = bool(
            self.declare_parameter("apply_joint_model_configuration", True).value
        )
        self.use_gazebo_cmd_vel_bridge = bool(
            self.declare_parameter("use_gazebo_cmd_vel_bridge", False).value
        )

        self.x = float(self.declare_parameter("initial_x", 2.0).value)
        self.y = float(self.declare_parameter("initial_y", 36.0).value)
        self.yaw = float(self.declare_parameter("initial_yaw", 0.0).value)

        self.current_linear = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_steering = 0.0
        self.last_angular = 0.0

        self.front_left_wheel_pos = 0.0
        self.front_right_wheel_pos = 0.0
        self.rear_left_wheel_pos = 0.0
        self.rear_right_wheel_pos = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.last_wait_log = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.cmd_bridge_pub = None
        if self.use_gazebo_cmd_vel_bridge:
            self.cmd_bridge_pub = self.create_publisher(Twist, self.bridge_cmd_vel_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.entity_client = None
        if not self.use_gazebo_cmd_vel_bridge:
            self.entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.model_config_client = None
        if self.apply_joint_model_configuration and not self.use_gazebo_cmd_vel_bridge:
            self.model_config_client = self.create_client(
                SetModelConfiguration, "/gazebo/set_model_configuration"
            )
        self.entity_future: Optional[rclpy.task.Future] = None
        self.model_future: Optional[rclpy.task.Future] = None

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_callback, 10)
        self.timer = self.create_timer(max(1.0 / self.update_rate, 0.01), self.update)

    def cmd_callback(self, msg: Twist) -> None:
        self.target_linear = clamp(float(msg.linear.x), -self.max_speed, self.max_speed)
        self.target_angular = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()
        if self.cmd_bridge_pub is not None:
            self.cmd_bridge_pub.publish(msg)

    def _services_ready(self) -> bool:
        if self.entity_client is None:
            return True
        if not self.entity_client.service_is_ready():
            now = self.get_clock().now()
            if now - self.last_wait_log > Duration(seconds=5.0):
                self.get_logger().info("Waiting for Gazebo set_entity_state service")
                self.last_wait_log = now
            return False
        if self.model_config_client is None:
            return True
        if self.model_config_client.service_is_ready():
            return True
        now = self.get_clock().now()
        if now - self.last_wait_log > Duration(seconds=5.0):
            self.get_logger().info("Waiting for Gazebo set_model_configuration service")
            self.last_wait_log = now
        return False

    def _approach(self, current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def _target_steering(self) -> float:
        if abs(self.current_linear) < self.min_speed_for_curvature or abs(self.target_angular) < 1e-4:
            return 0.0
        steering = math.atan(self.wheelbase * self.target_angular / self.current_linear)
        return clamp(steering, -self.max_steering_angle, self.max_steering_angle)

    def _split_steering(self, center_angle: float) -> Tuple[float, float]:
        if abs(center_angle) < 1e-5:
            return 0.0, 0.0

        turn_left = center_angle > 0.0
        radius = abs(self.wheelbase / math.tan(center_angle))
        inner_radius = max(radius - self.track_width * 0.5, 1e-4)
        outer_radius = radius + self.track_width * 0.5
        inner_angle = math.atan(self.wheelbase / inner_radius)
        outer_angle = math.atan(self.wheelbase / outer_radius)

        if turn_left:
            return inner_angle, outer_angle
        return -outer_angle, -inner_angle

    def _publish_odom(self, stamp) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.model_z
        msg.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        msg.twist.twist.linear.x = self.current_linear
        msg.twist.twist.angular.z = self.last_angular
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.02
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.02
        self.odom_pub.publish(msg)

    def _publish_joint_states(self, stamp, left_steer: float, right_steer: float) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "front_left_steering_joint",
            "front_right_steering_joint",
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]
        msg.position = [
            left_steer,
            right_steer,
            self.front_left_wheel_pos,
            self.front_right_wheel_pos,
            self.rear_left_wheel_pos,
            self.rear_right_wheel_pos,
        ]
        self.joint_pub.publish(msg)

    def _publish_tf(self, stamp) -> None:
        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.model_z
        transform.transform.rotation = quaternion_from_yaw(self.yaw)
        self.tf_broadcaster.sendTransform(transform)

    def _send_entity_state(self, left_steer: float, right_steer: float) -> None:
        if self.entity_client is None:
            return
        if not self._services_ready():
            return

        if self.entity_future is None or self.entity_future.done():
            linear_world_x = self.current_linear * math.cos(self.yaw)
            linear_world_y = self.current_linear * math.sin(self.yaw)
            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = self.entity_name
            request.state.reference_frame = self.reference_frame
            request.state.pose.position.x = self.x
            request.state.pose.position.y = self.y
            request.state.pose.position.z = self.model_z
            request.state.pose.orientation = quaternion_from_yaw(self.yaw)
            request.state.twist.linear.x = linear_world_x
            request.state.twist.linear.y = linear_world_y
            request.state.twist.angular.z = self.last_angular
            self.entity_future = self.entity_client.call_async(request)

        if (
            self.model_config_client is not None
            and (self.model_future is None or self.model_future.done())
        ):
            request = SetModelConfiguration.Request()
            request.model_name = self.entity_name
            request.urdf_param_name = ""
            request.joint_names = [
                "front_left_steering_joint",
                "front_right_steering_joint",
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
            ]
            request.joint_positions = [
                left_steer,
                right_steer,
                self.front_left_wheel_pos,
                self.front_right_wheel_pos,
                self.rear_left_wheel_pos,
                self.rear_right_wheel_pos,
            ]
            self.model_future = self.model_config_client.call_async(request)

    def update(self) -> None:
        now = self.get_clock().now()
        dt = max((now - self.last_update_time).nanoseconds / 1e9, 0.0)
        self.last_update_time = now
        if dt <= 0.0:
            return

        if now - self.last_cmd_time > Duration(seconds=self.command_timeout):
            linear_target = 0.0
            angular_target = 0.0
        else:
            linear_target = self.target_linear
            angular_target = self.target_angular

        self.current_linear = self._approach(
            self.current_linear, linear_target, self.max_accel * dt
        )
        self.target_angular = angular_target

        steering_target = self._target_steering()
        self.current_steering = self._approach(
            self.current_steering, steering_target, self.max_steering_rate * dt
        )

        if abs(self.current_linear) < 1e-4:
            self.last_angular = 0.0
        else:
            self.last_angular = self.current_linear * math.tan(self.current_steering) / self.wheelbase

        yaw_mid = self.yaw + 0.5 * self.last_angular * dt
        self.x += self.current_linear * math.cos(yaw_mid) * dt
        self.y += self.current_linear * math.sin(yaw_mid) * dt
        self.yaw = wrap_angle(self.yaw + self.last_angular * dt)

        wheel_rotation_delta = (self.current_linear / self.wheel_radius) * dt
        self.front_left_wheel_pos += wheel_rotation_delta
        self.front_right_wheel_pos += wheel_rotation_delta
        self.rear_left_wheel_pos += wheel_rotation_delta
        self.rear_right_wheel_pos += wheel_rotation_delta

        left_steer, right_steer = self._split_steering(self.current_steering)
        stamp = now.to_msg()
        if self.publish_odom:
            self._publish_odom(stamp)
        self._publish_joint_states(stamp, left_steer, right_steer)
        if self.publish_tf:
            self._publish_tf(stamp)
        self._send_entity_state(left_steer, right_steer)


def main() -> None:
    rclpy.init()
    node = AckermannGazeboSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
