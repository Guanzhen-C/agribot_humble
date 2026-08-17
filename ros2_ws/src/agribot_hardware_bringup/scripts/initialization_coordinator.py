#!/usr/bin/env python3

"""Serialize RTK, visual and manual initial-pose attempts."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


WAIT_RTK = "wait_rtk"
RTK_REFINING = "rtk_refining"
WAIT_VISUAL = "wait_visual"
VISUAL_REFINING = "visual_refining"
MANUAL_REQUIRED = "manual_required"
MANUAL_REFINING = "manual_refining"
READY = "ready"


@dataclass(frozen=True)
class PolicyAction:
    kind: str
    candidate_index: int = -1


class InitializationPolicy:
    """Pure transition policy used by the ROS node and unit tests."""

    def __init__(
        self,
        *,
        rtk_enabled: bool,
        visual_enabled: bool,
        rtk_wait_sec: float,
        visual_wait_sec: float,
        now: float,
    ):
        self.rtk_enabled = rtk_enabled
        self.visual_enabled = visual_enabled
        self.rtk_wait_sec = rtk_wait_sec
        self.visual_wait_sec = visual_wait_sec
        self.stage = WAIT_RTK if rtk_enabled else WAIT_VISUAL
        self.source = "none"
        self.deadline = now + (
            rtk_wait_sec if rtk_enabled else visual_wait_sec
        )
        self.visual_available = None
        self.visual_candidate_count = 0
        self.visual_candidate_index = -1
        if not rtk_enabled and not visual_enabled:
            self.stage = MANUAL_REQUIRED
            self.deadline = math.inf

    def _begin_visual(self, now: float) -> list[PolicyAction]:
        if not self.visual_enabled or self.visual_available is False:
            self.stage = MANUAL_REQUIRED
            self.source = "none"
            self.deadline = math.inf
            return [PolicyAction("request_visual", 0), PolicyAction("manual_required")]
        self.stage = WAIT_VISUAL
        self.deadline = now + self.visual_wait_sec
        return [PolicyAction("request_visual", 1)]

    def tick(self, now: float) -> list[PolicyAction]:
        if now < self.deadline:
            return []
        if self.stage == WAIT_RTK:
            return self._begin_visual(now)
        if self.stage == WAIT_VISUAL:
            self.stage = MANUAL_REQUIRED
            self.source = "none"
            self.deadline = math.inf
            return [PolicyAction("request_visual", 0), PolicyAction("manual_required")]
        return []

    def set_visual_available(self, available: bool, now: float) -> list[PolicyAction]:
        self.visual_available = available
        if self.stage == WAIT_VISUAL and not available:
            return self._begin_visual(now)
        return []

    def receive_rtk(self, now: float) -> list[PolicyAction]:
        if self.stage != WAIT_RTK:
            return []
        self.stage = RTK_REFINING
        self.source = "rtk"
        self.deadline = math.inf
        return [PolicyAction("forward_rtk")]

    def receive_visual_candidates(self, count: int, now: float) -> list[PolicyAction]:
        if self.stage != WAIT_VISUAL:
            return []
        if count <= 0:
            self.stage = MANUAL_REQUIRED
            self.deadline = math.inf
            return [PolicyAction("request_visual", 0), PolicyAction("manual_required")]
        self.visual_candidate_count = count
        self.visual_candidate_index = 0
        self.stage = VISUAL_REFINING
        self.source = "visual"
        self.deadline = math.inf
        return [PolicyAction("request_visual", 0), PolicyAction("try_visual", 0)]

    def receive_manual(self, now: float) -> list[PolicyAction]:
        if self.stage != MANUAL_REQUIRED:
            return []
        self.stage = MANUAL_REFINING
        self.source = "manual"
        self.deadline = math.inf
        return [PolicyAction("forward_manual")]

    def attempt_result(self, accepted: bool, now: float) -> list[PolicyAction]:
        if self.stage not in (RTK_REFINING, VISUAL_REFINING, MANUAL_REFINING):
            return []
        if accepted:
            self.stage = READY
            self.deadline = math.inf
            return [PolicyAction("request_visual", 0), PolicyAction("ready")]
        if self.stage == RTK_REFINING:
            return self._begin_visual(now)
        if self.stage == VISUAL_REFINING:
            next_index = self.visual_candidate_index + 1
            if next_index < self.visual_candidate_count:
                self.visual_candidate_index = next_index
                self.deadline = math.inf
                return [PolicyAction("try_visual", next_index)]
        self.stage = MANUAL_REQUIRED
        self.source = "none"
        self.deadline = math.inf
        return [PolicyAction("manual_required")]

    def localization_ready(self) -> list[PolicyAction]:
        if self.stage == READY:
            return []
        self.stage = READY
        self.deadline = math.inf
        return [PolicyAction("request_visual", 0), PolicyAction("ready")]


def transient_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class InitializationCoordinator(Node):
    def __init__(self):
        super().__init__("initialization_coordinator")
        rtk_enabled = self.declare_parameter("rtk_enabled", True).value
        visual_enabled = self.declare_parameter("visual_enabled", True).value
        rtk_wait_sec = float(self.declare_parameter("rtk_wait_sec", 20.0).value)
        visual_wait_sec = float(
            self.declare_parameter("visual_wait_sec", 10.0).value
        )
        if min(rtk_wait_sec, visual_wait_sec) <= 0.0:
            raise ValueError("initialization timeouts must be positive")

        self._policy = InitializationPolicy(
            rtk_enabled=bool(rtk_enabled),
            visual_enabled=bool(visual_enabled),
            rtk_wait_sec=rtk_wait_sec,
            visual_wait_sec=visual_wait_sec,
            now=time.monotonic(),
        )
        self._visual_candidates = []
        self._last_detail = ""

        latched = transient_qos()
        self._prior_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/localization/initialpose_prior", 10
        )
        self._visual_request_publisher = self.create_publisher(
            Bool, "/localization/visual_request", latched
        )
        self._stage_publisher = self.create_publisher(
            String, "/localization/initialization_stage", latched
        )
        self._source_publisher = self.create_publisher(
            String, "/localization/initialization_source", latched
        )
        self._status_publisher = self.create_publisher(
            String, "/localization/initialization_status", latched
        )
        self._manual_required_publisher = self.create_publisher(
            Bool, "/localization/manual_required", latched
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization/rtk_initialpose",
            self._handle_rtk,
            10,
        )
        self.create_subscription(
            PoseArray,
            "/localization/visual_candidates",
            self._handle_visual_candidates,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._handle_manual, 10
        )
        self.create_subscription(
            Bool,
            "/localization/visual_available",
            self._handle_visual_available,
            latched,
        )
        self.create_subscription(
            String,
            "/localization/attempt_result",
            self._handle_attempt_result,
            10,
        )
        self.create_subscription(
            Bool,
            "/localization/lidar_ready",
            self._handle_localization_ready,
            latched,
        )
        self.create_subscription(
            String,
            "/localization/visual_status",
            self._handle_visual_status,
            latched,
        )
        self.create_timer(0.2, self._tick)
        self._publish_state()
        if self._policy.stage == WAIT_VISUAL:
            self._publish_visual_request(True)

    def _tick(self):
        self._run_actions(self._policy.tick(time.monotonic()))

    def _handle_rtk(self, message):
        self._run_actions(
            self._policy.receive_rtk(time.monotonic()), rtk_pose=message
        )

    def _handle_visual_candidates(self, message):
        if message.header.frame_id not in ("", "map"):
            self.get_logger().warning(
                f"忽略坐标系为{message.header.frame_id}的视觉候选"
            )
            return
        self._visual_candidates = list(message.poses)
        self._run_actions(
            self._policy.receive_visual_candidates(
                len(self._visual_candidates), time.monotonic()
            )
        )

    def _handle_manual(self, message):
        actions = self._policy.receive_manual(time.monotonic())
        if not actions:
            self.get_logger().warning(
                "自动初始化尚未降级到手动阶段，忽略本次手动初始位姿"
            )
            return
        self._run_actions(actions, manual_pose=message)

    def _handle_visual_available(self, message):
        self._run_actions(
            self._policy.set_visual_available(bool(message.data), time.monotonic())
        )

    def _handle_visual_status(self, message):
        try:
            document = json.loads(message.data)
            self._last_detail = str(document.get("message", message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            self._last_detail = message.data
        self._publish_state()

    def _handle_attempt_result(self, message):
        try:
            result = json.loads(message.data)
            accepted = bool(result["accepted"])
            self._last_detail = str(result.get("reason", ""))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"忽略无效配准结果: {error}")
            return
        self._run_actions(
            self._policy.attempt_result(accepted, time.monotonic())
        )

    def _handle_localization_ready(self, message):
        if message.data:
            self._run_actions(self._policy.localization_ready())

    def _publish_visual_request(self, enabled):
        message = Bool()
        message.data = bool(enabled)
        self._visual_request_publisher.publish(message)

    def _publish_prior(self, source, pose):
        message = PoseWithCovarianceStamped()
        message.header = pose.header
        message.header.frame_id = "map"
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            message.header.stamp = self.get_clock().now().to_msg()
        message.pose = pose.pose
        if not any(message.pose.covariance):
            message.pose.covariance[0] = 1.0
            message.pose.covariance[7] = 1.0
            message.pose.covariance[35] = math.radians(30.0) ** 2
        self._prior_publisher.publish(message)
        self.get_logger().info(f"已提交{source}粗位姿，等待NDT/GICP验收")

    def _publish_visual_candidate(self, index):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose = self._visual_candidates[index]
        pose.pose.covariance[0] = 2.25
        pose.pose.covariance[7] = 2.25
        pose.pose.covariance[35] = math.radians(45.0) ** 2
        self._publish_prior(f"视觉候选{index + 1}", pose)

    def _run_actions(self, actions, *, rtk_pose=None, manual_pose=None):
        for action in actions:
            if action.kind == "request_visual":
                self._publish_visual_request(bool(action.candidate_index))
            elif action.kind == "forward_rtk" and rtk_pose is not None:
                self._publish_prior("RTK", rtk_pose)
            elif action.kind == "try_visual":
                self._publish_visual_candidate(action.candidate_index)
            elif action.kind == "forward_manual" and manual_pose is not None:
                self._publish_prior("手动", manual_pose)
        if actions:
            self._publish_state()

    def _status_text(self):
        labels = {
            WAIT_RTK: "等待RTK固定解粗定位",
            RTK_REFINING: "正在用NDT/GICP精配准RTK粗位姿",
            WAIT_VISUAL: "RTK不可用，正在进行视觉位置识别",
            VISUAL_REFINING: "正在用NDT/GICP精配准视觉候选",
            MANUAL_REQUIRED: "自动初始化失败，请在地图上手动给定初始位姿",
            MANUAL_REFINING: "正在用NDT/GICP精配准手动粗位姿",
            READY: "初始定位已通过NDT/GICP验收",
        }
        text = labels[self._policy.stage]
        if self._last_detail and self._policy.stage != READY:
            text += f"；{self._last_detail}"
        return text

    def _publish_state(self):
        stage = String()
        stage.data = self._policy.stage
        self._stage_publisher.publish(stage)
        source = String()
        source.data = self._policy.source
        self._source_publisher.publish(source)
        status = String()
        status.data = self._status_text()
        self._status_publisher.publish(status)
        manual = Bool()
        manual.data = self._policy.stage == MANUAL_REQUIRED
        self._manual_required_publisher.publish(manual)


def main(args=None):
    rclpy.init(args=args)
    node = InitializationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
