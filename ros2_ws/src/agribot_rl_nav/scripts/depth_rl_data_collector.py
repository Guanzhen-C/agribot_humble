#!/usr/bin/env python3

import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DepthRLDataCollector(Node):
    def __init__(self) -> None:
        super().__init__("depth_rl_data_collector")
        self.output_dir = Path(self.declare_parameter("output_dir", "").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw").value
        self.rgb_topic = self.declare_parameter("rgb_topic", "/camera/color/image_raw").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.goal_topic = self.declare_parameter("goal_topic", "/current_goal").value
        self.global_plan_topic = self.declare_parameter("global_plan_topic", "/plan").value
        self.action_topic = self.declare_parameter("action_topic", "/cmd_vel").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.sample_hz = float(self.declare_parameter("sample_hz", 10.0).value)
        self.frame_skip = int(self.declare_parameter("frame_skip", 2).value)
        self.max_depth = float(self.declare_parameter("max_depth", 8.0).value)
        self.max_scan_range = float(self.declare_parameter("max_scan_range", 8.0).value)
        self.path_point_count = int(self.declare_parameter("path_point_count", 5).value)
        self.plan_stride = int(self.declare_parameter("plan_stride", 4).value)
        self.goal_distance_clip = float(self.declare_parameter("goal_distance_clip", 40.0).value)
        self.shard_size = int(self.declare_parameter("shard_size", 128).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.depth_image: Optional[np.ndarray] = None
        self.rgb_image: Optional[np.ndarray] = None
        self.scan: Optional[np.ndarray] = None
        self.ground_truth_pose: Optional[np.ndarray] = None
        self.ground_truth_velocity: Optional[np.ndarray] = None
        self.goal_pose: Optional[Tuple[float, float, float]] = None
        self.goal_frame_id: Optional[str] = None
        self.global_plan: List[Tuple[float, float, float]] = []
        self.global_plan_frame_id: Optional[str] = None
        self.action: Optional[np.ndarray] = None

        self.frame_counter = 0
        self.sample_counter = 0
        self.shard_index = 0
        self.buffer = {k: [] for k in (
            "depth", "rgb", "scan", "goal", "path_points", "path_features", "velocity",
            "action", "pose", "goal_map", "meta"
        )}

        self.create_subscription(Image, self.depth_topic, self.handle_depth, 10)
        self.create_subscription(Image, self.rgb_topic, self.handle_rgb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.handle_scan, 10)
        self.create_subscription(Odometry, self.ground_truth_topic, self.handle_ground_truth, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.handle_goal, 10)
        self.create_subscription(NavPath, self.global_plan_topic, self.handle_global_plan, 10)
        self.create_subscription(Twist, self.action_topic, self.handle_action, 10)
        self.create_timer(1.0 / max(self.sample_hz, 1e-3), self.handle_timer)

    def handle_depth(self, msg: Image) -> None:
        if msg.height == 0 or msg.width == 0 or msg.encoding not in ("32FC1", "passthrough"):
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = np.nan_to_num(depth, nan=self.max_depth, posinf=self.max_depth, neginf=0.0)
        self.depth_image = np.clip(depth / self.max_depth, 0.0, 1.0).astype(np.float32)

    def handle_rgb(self, msg: Image) -> None:
        if msg.height == 0 or msg.width == 0 or msg.encoding not in ("rgb8", "bgr8"):
            return
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            rgb = rgb[:, :, ::-1]
        self.rgb_image = rgb.copy()

    def handle_scan(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        max_range = self.max_scan_range if self.max_scan_range > 0.0 else float(msg.range_max)
        max_range = max(max_range, 8.0)
        ranges = np.nan_to_num(ranges, nan=max_range, posinf=max_range, neginf=0.0)
        self.scan = np.clip(ranges / max_range, 0.0, 1.0).astype(np.float32)

    def handle_ground_truth(self, msg: Odometry) -> None:
        self.ground_truth_pose = np.array(
            [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), quaternion_to_yaw(msg.pose.pose.orientation)],
            dtype=np.float32,
        )
        self.ground_truth_velocity = np.array(
            [float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)], dtype=np.float32
        )

    def handle_goal(self, msg: PoseStamped) -> None:
        self.goal_frame_id = msg.header.frame_id or None
        self.goal_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            quaternion_to_yaw(msg.pose.orientation),
        )

    def handle_global_plan(self, msg: NavPath) -> None:
        self.global_plan_frame_id = msg.header.frame_id or None
        self.global_plan = [
            (float(p.pose.position.x), float(p.pose.position.y), quaternion_to_yaw(p.pose.orientation))
            for p in msg.poses
        ]

    def handle_action(self, msg: Twist) -> None:
        self.action = np.array([float(msg.linear.x), float(msg.angular.z)], dtype=np.float32)

    def lookup_robot_pose(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(frame_id, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            quaternion_to_yaw(transform.transform.rotation),
        )

    def point_to_base(self, point_xyyaw: Tuple[float, float, float], frame_id: str) -> Optional[np.ndarray]:
        robot_pose = self.lookup_robot_pose(frame_id)
        if robot_pose is None:
            return None
        rx, ry, ryaw = robot_pose
        px, py, pyaw = point_xyyaw
        dx = px - rx
        dy = py - ry
        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)
        return np.array(
            [cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy, normalize_angle(pyaw - ryaw)],
            dtype=np.float32,
        )

    def build_goal_tensor(self) -> Optional[np.ndarray]:
        if self.goal_pose is None or self.goal_frame_id is None:
            return None
        goal = self.point_to_base(self.goal_pose, self.goal_frame_id)
        if goal is None:
            return None
        goal[0] = float(np.clip(goal[0], -self.goal_distance_clip, self.goal_distance_clip))
        goal[1] = float(np.clip(goal[1], -self.goal_distance_clip, self.goal_distance_clip))
        return goal

    def build_path_tensors(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not self.global_plan or self.global_plan_frame_id is None:
            return None, None
        robot_pose = self.lookup_robot_pose(self.global_plan_frame_id)
        if robot_pose is None:
            return None, None
        robot_xy = np.array(robot_pose[:2], dtype=np.float32)
        plan_xy = np.array([[px, py] for px, py, _ in self.global_plan], dtype=np.float32)
        nearest_idx = int(np.argmin(np.linalg.norm(plan_xy - robot_xy[None, :], axis=1)))
        nearest_local = self.point_to_base(self.global_plan[nearest_idx], self.global_plan_frame_id)
        if nearest_local is None:
            return None, None
        points: List[np.ndarray] = []
        for i in range(self.path_point_count):
            idx = min(nearest_idx + i * self.plan_stride, len(self.global_plan) - 1)
            local = self.point_to_base(self.global_plan[idx], self.global_plan_frame_id)
            if local is None:
                return None, None
            points.append(local[:2])
        while len(points) < self.path_point_count:
            points.append(points[-1].copy())
        lookahead = points[min(1, len(points) - 1)]
        path_points = np.stack(points, axis=0).astype(np.float32)
        path_features = np.array(
            [float(nearest_local[1]), float(nearest_local[2]), float(lookahead[0]), float(lookahead[1]), float(nearest_idx)],
            dtype=np.float32,
        )
        return path_points, path_features

    def is_ready(self) -> bool:
        return all(
            [
                self.depth_image is not None,
                self.rgb_image is not None,
                self.scan is not None,
                self.ground_truth_pose is not None,
                self.ground_truth_velocity is not None,
                self.goal_pose is not None,
                self.goal_frame_id is not None,
                self.global_plan,
                self.global_plan_frame_id is not None,
                self.action is not None,
            ]
        )

    def append_sample(self) -> None:
        goal = self.build_goal_tensor()
        path_points, path_features = self.build_path_tensors()
        if goal is None or path_points is None or path_features is None:
            return
        goal_raw = np.array(self.goal_pose, dtype=np.float32)
        meta = np.array(
            [
                self.get_clock().now().nanoseconds / 1e9,
                float(self.sample_counter),
                1.0,
                float(np.linalg.norm(goal[:2])),
                float(np.linalg.norm(self.ground_truth_velocity)),
            ],
            dtype=np.float32,
        )
        self.buffer["depth"].append(self.depth_image.copy())
        self.buffer["rgb"].append(self.rgb_image.copy())
        self.buffer["scan"].append(self.scan.copy())
        self.buffer["goal"].append(goal)
        self.buffer["path_points"].append(path_points)
        self.buffer["path_features"].append(path_features)
        self.buffer["velocity"].append(self.ground_truth_velocity.copy())
        self.buffer["action"].append(self.action.copy())
        self.buffer["pose"].append(self.ground_truth_pose.copy())
        self.buffer["goal_map"].append(goal_raw)
        self.buffer["meta"].append(meta)
        self.sample_counter += 1
        if len(self.buffer["depth"]) >= self.shard_size:
            self.flush()

    def flush(self) -> None:
        if not self.buffer["depth"]:
            return
        shard_path = self.output_dir / f"depth_rl_shard_{self.shard_index:05d}.npz"
        np.savez_compressed(
            shard_path,
            depth=np.stack(self.buffer["depth"], axis=0),
            rgb=np.stack(self.buffer["rgb"], axis=0),
            scan=np.stack(self.buffer["scan"], axis=0),
            goal=np.stack(self.buffer["goal"], axis=0),
            path_points=np.stack(self.buffer["path_points"], axis=0),
            path_features=np.stack(self.buffer["path_features"], axis=0),
            velocity=np.stack(self.buffer["velocity"], axis=0),
            action=np.stack(self.buffer["action"], axis=0),
            pose=np.stack(self.buffer["pose"], axis=0),
            goal_map=np.stack(self.buffer["goal_map"], axis=0),
            meta=np.stack(self.buffer["meta"], axis=0),
        )
        self.get_logger().info(f"Wrote {shard_path} with {len(self.buffer['depth'])} samples")
        self.shard_index += 1
        for key in self.buffer:
            self.buffer[key].clear()

    def handle_timer(self) -> None:
        if not self.is_ready():
            return
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return
        self.append_sample()

    def destroy_node(self):
        self.flush()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DepthRLDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
