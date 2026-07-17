#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DepthRLPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_rl_policy_node")
        self.model_path = self.declare_parameter("model_path", "").value
        self.depth_topic = self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw").value
        self.ground_truth_topic = self.declare_parameter(
            "ground_truth_topic", "/base_pose_ground_truth"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.global_plan_topic = self.declare_parameter("global_plan_topic", "/plan").value
        self.goal_topic = self.declare_parameter("goal_topic", "/current_goal").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.control_hz = float(self.declare_parameter("control_hz", 10.0).value)
        self.max_depth = float(self.declare_parameter("max_depth", 8.0).value)
        self.max_linear_speed = float(self.declare_parameter("max_linear_speed", 0.55).value)
        self.max_angular_speed = float(self.declare_parameter("max_angular_speed", 1.0).value)
        self.goal_distance_clip = float(self.declare_parameter("goal_distance_clip", 40.0).value)
        self.path_point_count = int(self.declare_parameter("path_point_count", 5).value)
        self.plan_stride = int(self.declare_parameter("plan_stride", 4).value)
        self.seq_len = int(self.declare_parameter("seq_len", 8).value)
        self.chunk_replan_interval = int(self.declare_parameter("chunk_replan_interval", 2).value)
        self.stop_without_plan = bool(self.declare_parameter("stop_without_plan", True).value)
        self.debug_log = bool(self.declare_parameter("debug_log", False).value)

        self.model = torch.jit.load(self.model_path, map_location="cpu")
        self.model.eval()
        self.legacy_single_frame_model = "path_features: Tensor,\n    path_points: Tensor" in self.model.code
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.depth_image: Optional[np.ndarray] = None
        self.velocity_vw: Optional[Tuple[float, float]] = None
        self.goal_map: Optional[Tuple[float, float, float]] = None
        self.goal_frame_id: Optional[str] = None
        self.global_plan: List[Tuple[float, float, float]] = []
        self.global_plan_frame_id: Optional[str] = None
        self.depth_history: Deque[np.ndarray] = deque(maxlen=self.seq_len)
        self.goal_history: Deque[np.ndarray] = deque(maxlen=self.seq_len)
        self.path_points_history: Deque[np.ndarray] = deque(maxlen=self.seq_len)
        self.path_features_history: Deque[np.ndarray] = deque(maxlen=self.seq_len)
        self.velocity_history: Deque[np.ndarray] = deque(maxlen=self.seq_len)
        self.action_queue: Deque[np.ndarray] = deque()
        self.steps_since_replan = self.chunk_replan_interval

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Image, self.depth_topic, self.handle_depth, 10)
        self.create_subscription(Odometry, self.ground_truth_topic, self.handle_odometry, 10)
        self.create_subscription(Path, self.global_plan_topic, self.handle_global_plan, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.handle_goal, 10)
        self.create_timer(1.0 / max(self.control_hz, 1e-3), self.handle_timer)

    def handle_depth(self, msg: Image) -> None:
        if msg.height == 0 or msg.width == 0 or msg.encoding not in ("32FC1", "passthrough"):
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = np.nan_to_num(depth, nan=self.max_depth, posinf=self.max_depth, neginf=0.0)
        self.depth_image = np.clip(depth / self.max_depth, 0.0, 1.0).astype(np.float32)

    def handle_odometry(self, msg: Odometry) -> None:
        self.velocity_vw = (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))

    def handle_goal(self, msg: PoseStamped) -> None:
        self.goal_map = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            quaternion_to_yaw(msg.pose.orientation),
        )
        self.goal_frame_id = msg.header.frame_id or None
        self.action_queue.clear()
        self.steps_since_replan = self.chunk_replan_interval

    def handle_global_plan(self, msg: Path) -> None:
        self.global_plan_frame_id = msg.header.frame_id or None
        self.global_plan = [
            (
                float(p.pose.position.x),
                float(p.pose.position.y),
                quaternion_to_yaw(p.pose.orientation),
            )
            for p in msg.poses
        ]
        self.action_queue.clear()
        self.steps_since_replan = self.chunk_replan_interval

    def lookup_robot_pose(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(frame_id, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        rotation = transform.transform.rotation
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            quaternion_to_yaw(rotation),
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
        if self.goal_map is None or self.goal_frame_id is None:
            return None
        goal = self.point_to_base(self.goal_map, self.goal_frame_id)
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

    def publish_stop(self) -> None:
        self.action_queue.clear()
        self.cmd_pub.publish(Twist())

    def append_history(self, depth, goal, path_points, path_features, velocity) -> None:
        self.depth_history.append(depth.copy())
        self.goal_history.append(goal.copy())
        self.path_points_history.append(path_points.copy())
        self.path_features_history.append(path_features.copy())
        self.velocity_history.append(velocity.copy())

    def pad_history(self) -> None:
        while len(self.depth_history) < self.seq_len:
            self.depth_history.appendleft(self.depth_history[0].copy())
            self.goal_history.appendleft(self.goal_history[0].copy())
            self.path_points_history.appendleft(self.path_points_history[0].copy())
            self.path_features_history.appendleft(self.path_features_history[0].copy())
            self.velocity_history.appendleft(self.velocity_history[0].copy())

    def handle_timer(self) -> None:
        if self.depth_image is None or self.velocity_vw is None:
            self.publish_stop()
            return
        goal = self.build_goal_tensor()
        path_points, path_features = self.build_path_tensors()
        if goal is None or path_points is None or path_features is None:
            if self.stop_without_plan:
                self.publish_stop()
            return

        velocity = np.array(self.velocity_vw, dtype=np.float32)
        if self.legacy_single_frame_model:
            with torch.no_grad():
                action = self.model(
                    torch.from_numpy(self.depth_image[None, ...]),
                    torch.from_numpy(goal[None, :]),
                    torch.from_numpy(path_features[None, :]),
                    torch.from_numpy(path_points[None, ...]),
                    torch.from_numpy(velocity[None, :]),
                ).cpu().numpy()[0]
        else:
            self.append_history(self.depth_image, goal, path_points, path_features, velocity)
            self.pad_history()
            need_replan = (not self.action_queue) or (self.steps_since_replan >= self.chunk_replan_interval)
            if need_replan:
                with torch.no_grad():
                    chunk = self.model(
                        torch.from_numpy(np.stack(list(self.depth_history), axis=0)[None, ...]),
                        torch.from_numpy(np.stack(list(self.goal_history), axis=0)[None, ...]),
                        torch.from_numpy(np.stack(list(self.path_points_history), axis=0)[None, ...]),
                        torch.from_numpy(np.stack(list(self.path_features_history), axis=0)[None, ...]),
                        torch.from_numpy(np.stack(list(self.velocity_history), axis=0)[None, ...]),
                    ).cpu().numpy()
                self.action_queue.clear()
                if chunk.ndim == 2:
                    self.action_queue.append(chunk[0].astype(np.float32, copy=False))
                elif chunk.ndim == 3:
                    for item in chunk[0]:
                        self.action_queue.append(item.astype(np.float32, copy=False))
                else:
                    self.publish_stop()
                    return
                self.steps_since_replan = 0
            if not self.action_queue:
                self.publish_stop()
                return
            action = self.action_queue.popleft()
            self.steps_since_replan += 1

        cmd = Twist()
        cmd.linear.x = float(np.clip(action[0], 0.0, self.max_linear_speed))
        cmd.angular.z = float(np.clip(action[1], -self.max_angular_speed, self.max_angular_speed))
        self.cmd_pub.publish(cmd)
        if self.debug_log:
            self.get_logger().info(
                f"goal=({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}) "
                f"action=({cmd.linear.x:.3f}, {cmd.angular.z:.3f})"
            )


def main() -> None:
    rclpy.init()
    node = DepthRLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
