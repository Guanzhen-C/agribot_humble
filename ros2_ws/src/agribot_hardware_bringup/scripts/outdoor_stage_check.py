#!/usr/bin/env python3

import argparse
from collections import deque
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Bool, String, UInt8
from tf2_ros import Buffer, TransformException, TransformListener


ACTIVE_GOAL_STATES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}
NAV2_LIFECYCLE_NODES = (
    "map_server",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
)


def transient_qos():
    profile = QoSProfile(depth=1)
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return profile


def planar_speed(message):
    return math.hypot(
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
    )


def pose_is_finite(pose):
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    return all(math.isfinite(value) for value in values) and abs(
        quaternion_norm - 1.0
    ) < 0.05


class OutdoorStageCheck(Node):
    def __init__(self, stage):
        super().__init__("outdoor_stage_check")
        self.stage = stage
        self.started = time.monotonic()
        self.arrivals = {}
        self.values = {}
        self.frames = {}
        self.last_diagnostic = None

        latched = transient_qos()
        self._counted_subscription(
            PointCloud2, "/lidar/points", qos_profile_sensor_data
        )
        self._counted_subscription(Imu, "/imu/data", qos_profile_sensor_data)
        self._counted_subscription(
            Image, "/camera/rgb/image_raw", qos_profile_sensor_data
        )
        self._counted_subscription(
            NavSatFix, "/rtk/fix", qos_profile_sensor_data
        )
        self._counted_subscription(Odometry, "/fastlivo/odometry", 50)
        self._counted_subscription(Odometry, "/fastlivo_rtk/odometry", 50)
        self._counted_subscription(
            PoseWithCovarianceStamped, "/localization_pose", 20
        )
        self._counted_subscription(OccupancyGrid, "/map", latched)
        self._counted_subscription(
            OccupancyGrid, "/global_costmap/costmap", latched
        )
        self._counted_subscription(
            OccupancyGrid, "/local_costmap/costmap", latched
        )

        self.create_subscription(
            String,
            "/localization/status",
            self._value_callback("localization_status"),
            latched,
        )
        self.create_subscription(
            String,
            "/localization/rtk_initializer_status",
            self._value_callback("rtk_initializer_status"),
            latched,
        )
        self.create_subscription(
            Bool,
            "/localization/rtk_seed_ready",
            self._value_callback("rtk_seed_ready"),
            latched,
        )
        self.create_subscription(
            Bool,
            "/localization/lidar_ready",
            self._value_callback("lidar_ready"),
            latched,
        )
        self.create_subscription(
            Bool,
            "/fastlivo_rtk/ready",
            self._value_callback("fusion_ready"),
            latched,
        )
        self.create_subscription(
            Bool,
            "/fastlivo_rtk/fixed_active",
            self._value_callback("fixed_active"),
            latched,
        )
        self.create_subscription(
            UInt8, "/rtk/fix_quality", self._value_callback("fix_quality"), 20
        )
        self.create_subscription(
            String,
            "/rtk/heading_solution",
            self._value_callback("heading_solution"),
            20,
        )
        self.create_subscription(
            Twist, "/nav2/cmd_vel", self._command_callback, 20
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._goal_status_callback("navigate_to_pose"),
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_through_poses/_action/status",
            self._goal_status_callback("navigate_through_poses"),
            10,
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostic_callback, 20
        )
        if stage == "B":
            self._counted_subscription(Odometry, "/wheel/odometry", 50)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _counted_subscription(self, message_type, topic, qos):
        self.arrivals[topic] = deque()

        def callback(message):
            now = time.monotonic()
            arrivals = self.arrivals[topic]
            arrivals.append(now)
            while arrivals and now - arrivals[0] > 10.0:
                arrivals.popleft()
            self.values[topic] = message
            if hasattr(message, "header"):
                self.frames[topic] = getattr(message.header, "frame_id", "")

        self.create_subscription(message_type, topic, callback, qos)

    def _value_callback(self, key):
        def callback(message):
            self.values[key] = message.data

        return callback

    def _command_callback(self, message):
        magnitude = max(abs(message.linear.x), abs(message.angular.z))
        self.values["maximum_command"] = max(
            magnitude, self.values.get("maximum_command", 0.0)
        )

    def _goal_status_callback(self, key):
        def callback(message):
            self.values[f"active_goal_{key}"] = any(
                status.status in ACTIVE_GOAL_STATES
                for status in message.status_list
            )

        return callback

    def _diagnostic_callback(self, message):
        for status in message.status:
            if status.name == "agribot/chassis_can/ackermann":
                self.last_diagnostic = status

    def chassis_diagnostic_ok(self):
        if self.last_diagnostic is None:
            return False
        values = {item.key: item.value for item in self.last_diagnostic.values}
        return (
            self.last_diagnostic.level == DiagnosticStatus.OK
            and values.get("feedback_fresh") == "true"
            and values.get("localization_ready") == "true"
            and values.get("command_active") == "false"
        )

    def topic_rate(self, topic, elapsed):
        arrivals = self.arrivals.get(topic, ())
        if len(arrivals) >= 2:
            return (len(arrivals) - 1) / max(
                arrivals[-1] - arrivals[0], 1.0e-6
            )
        return len(arrivals) / max(elapsed, 1.0e-6)

    def core_ready(self):
        status = self.values.get("localization_status", "")
        heading = self.values.get("heading_solution", "")
        required_topics = (
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
        if self.stage == "B":
            required_topics += ("/wheel/odometry",)
        ready = (
            all(topic in self.values for topic in required_topics)
            and "accepted" in status.lower()
            and "rejected" not in status.lower()
            and self.values.get("rtk_seed_ready") is True
            and self.values.get("lidar_ready") is True
            and self.values.get("fusion_ready") is True
            and self.values.get("fixed_active") is True
            and self.values.get("fix_quality") == 4
            and heading in ("SOL_COMPUTED,L1_INT", "SOL_COMPUTED,NARROW_INT")
        )
        if self.stage == "B":
            ready = ready and self.chassis_diagnostic_ok()
        return ready

    def lifecycle_state(self, node_name):
        client = self.create_client(GetState, f"/{node_name}/get_state")
        if not client.wait_for_service(timeout_sec=1.0):
            return "服务不存在"
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return "查询超时"
        return future.result().current_state.label

    def map_contains_pose(self, topic, x, y):
        grid = self.values.get(topic)
        if grid is None:
            return False
        origin = grid.info.origin.position
        width_m = grid.info.width * grid.info.resolution
        height_m = grid.info.height * grid.info.resolution
        return (
            origin.x <= x < origin.x + width_m
            and origin.y <= y < origin.y + height_m
        )

    def evaluate(self, elapsed):
        checks = []

        def add(name, passed, detail):
            checks.append((name, bool(passed), detail))

        rate_limits = {
            "/lidar/points": 8.0,
            "/imu/data": 80.0,
            "/camera/rgb/image_raw": 8.0,
            "/rtk/fix": 5.0,
            "/fastlivo/odometry": 5.0,
            "/fastlivo_rtk/odometry": 5.0,
        }
        if self.stage == "B":
            rate_limits["/wheel/odometry"] = 5.0
        for topic, minimum in rate_limits.items():
            rate = self.topic_rate(topic, elapsed)
            add(
                f"{topic}频率",
                rate >= minimum,
                f"{rate:.1f} Hz，要求至少{minimum:.1f} Hz",
            )

        expected_frames = {
            "/lidar/points": "lidar_link",
            "/imu/data": "imu_link",
            "/fastlivo/odometry": "odom",
            "/fastlivo_rtk/odometry": "map",
            "/map": "map",
        }
        for topic, expected in expected_frames.items():
            actual = self.frames.get(topic, "未收到")
            add(f"{topic}坐标系", actual == expected, f"{actual}，期望{expected}")

        status = self.values.get("localization_status", "未收到")
        add(
            "NDT/GICP初始重定位",
            "accepted" in status.lower() and "rejected" not in status.lower(),
            status,
        )
        add(
            "RTK粗定位种子",
            self.values.get("rtk_seed_ready") is True,
            str(self.values.get("rtk_seed_ready", "未收到")),
        )
        add(
            "激光重定位就绪",
            self.values.get("lidar_ready") is True,
            str(self.values.get("lidar_ready", "未收到")),
        )
        add(
            "融合定位就绪",
            self.values.get("fusion_ready") is True,
            str(self.values.get("fusion_ready", "未收到")),
        )
        add(
            "当前RTK固定解参与融合",
            self.values.get("fixed_active") is True,
            str(self.values.get("fixed_active", "未收到")),
        )
        add(
            "RTK位置质量",
            self.values.get("fix_quality") == 4,
            str(self.values.get("fix_quality", "未收到")),
        )
        heading = self.values.get("heading_solution", "未收到")
        add(
            "RTK双天线航向",
            heading in ("SOL_COMPUTED,L1_INT", "SOL_COMPUTED,NARROW_INT"),
            heading,
        )

        fused = self.values.get("/fastlivo_rtk/odometry")
        fused_pose_ok = fused is not None and pose_is_finite(fused.pose.pose)
        add(
            "融合位姿数值",
            fused_pose_ok,
            "有限且四元数归一" if fused_pose_ok else "无效或未收到",
        )
        if fused is not None:
            speed = planar_speed(fused)
            add("验收期间车辆静止", speed <= 0.08, f"融合速度{speed:.3f} m/s")

        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as error:
            add("map到base_link TF", False, str(error))
        if transform is not None:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            finite = all(
                math.isfinite(value)
                for value in (
                    translation.x,
                    translation.y,
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                )
            )
            add("map到base_link TF", finite, "可用且数值有限")
            if fused is not None:
                tf_error = math.hypot(
                    translation.x - fused.pose.pose.position.x,
                    translation.y - fused.pose.pose.position.y,
                )
                add(
                    "TF与融合位姿一致",
                    tf_error <= 0.10,
                    f"水平差{tf_error:.3f} m",
                )

        if fused is not None:
            x = fused.pose.pose.position.x
            y = fused.pose.pose.position.y
            for topic, label in (
                ("/global_costmap/costmap", "全局代价地图包含车体"),
                ("/local_costmap/costmap", "局部代价地图包含车体"),
            ):
                add(
                    label,
                    self.map_contains_pose(topic, x, y),
                    f"车辆位置({x:.2f}, {y:.2f})",
                )

        for node_name in NAV2_LIFECYCLE_NODES:
            state = self.lifecycle_state(node_name)
            add(f"{node_name}生命周期", state == "active", state)

        node_names = {
            name for name, _namespace in self.get_node_names_and_namespaces()
        }
        can_running = "ackermann_chassis_can" in node_names
        serial_running = "ackermann_chassis_serial" in node_names
        if self.stage == "A":
            add(
                "阶段A未创建底盘节点",
                not can_running and not serial_running,
                f"CAN={can_running} 串口={serial_running}",
            )
        else:
            add(
                "阶段B仅创建CAN底盘节点",
                can_running and not serial_running,
                f"CAN={can_running} 串口={serial_running}",
            )
            if self.last_diagnostic is None:
                add("CAN诊断", False, "未收到agribot/chassis_can/ackermann")
            else:
                values = {
                    item.key: item.value
                    for item in self.last_diagnostic.values
                }
                add(
                    "CAN反馈、定位门控及静止输出",
                    self.chassis_diagnostic_ok(),
                    f"{self.last_diagnostic.message}; {values}",
                )
            wheel = self.values.get("/wheel/odometry")
            if wheel is not None:
                speed = planar_speed(wheel)
                add("底盘反馈静止", speed <= 0.03, f"轮速{speed:.3f} m/s")

        active_goal = self.values.get(
            "active_goal_navigate_to_pose", False
        ) or self.values.get("active_goal_navigate_through_poses", False)
        add("没有活动导航目标", not active_goal, str(active_goal))
        add(
            "Nav2当前没有非零控制输出",
            self.values.get("maximum_command", 0.0) <= 1.0e-3,
            f"最大绝对值{self.values.get('maximum_command', 0.0):.4f}",
        )
        return checks


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="室外真机阶段A/B一次性安全验收"
    )
    parser.add_argument("--stage", choices=("A", "B", "a", "b"), required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--sample-duration", type=float, default=5.0)
    arguments = parser.parse_args(argv)
    arguments.stage = arguments.stage.upper()
    if (
        arguments.timeout < arguments.sample_duration
        or arguments.sample_duration < 2.0
    ):
        parser.error("timeout必须不小于sample-duration，sample-duration至少为2秒")
    return arguments


def main(args=None):
    cli_args = sys.argv[1:] if args is None else args
    arguments = parse_arguments(cli_args)
    rclpy.init()
    node = OutdoorStageCheck(arguments.stage)
    deadline = time.monotonic() + arguments.timeout
    stable_ready_since = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.core_ready():
                if stable_ready_since is None:
                    stable_ready_since = time.monotonic()
                if (
                    time.monotonic() - stable_ready_since
                    >= arguments.sample_duration
                ):
                    break
            else:
                stable_ready_since = None
        elapsed = time.monotonic() - node.started
        checks = node.evaluate(elapsed)
        failures = 0
        for name, passed, detail in checks:
            marker = "通过" if passed else "失败"
            print(f"[{marker}] {name}: {detail}")
            failures += 0 if passed else 1
        if failures:
            print(
                f"阶段{arguments.stage}验收失败：{failures}项未通过，"
                "禁止下发导航目标。",
                file=sys.stderr,
            )
            return 2
        print(f"阶段{arguments.stage}验收通过。")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
