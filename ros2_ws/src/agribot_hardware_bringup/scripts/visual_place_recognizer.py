#!/usr/bin/env python3

"""RDK X5 BPU EigenPlaces coarse-pose provider for initial localization."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


def dequantize(values, quant_info):
    if np.issubdtype(values.dtype, np.floating):
        return values.astype(np.float32, copy=False)
    quant_type = getattr(quant_info.quant_type, "value", quant_info.quant_type)
    if int(quant_type) != 1:
        return values.astype(np.float32)
    scale = np.asarray(quant_info.scale, dtype=np.float32)
    zero = np.asarray(quant_info.zero_point, dtype=np.float32)
    if scale.ndim == 0 or scale.size == 1:
        return (values.astype(np.float32) - zero) * scale
    shape = [1] * values.ndim
    shape[int(quant_info.axis)] = -1
    return (values.astype(np.float32) - zero.reshape(shape)) * scale.reshape(shape)


class EigenPlacesBpu:
    def __init__(self, model_path: Path):
        try:
            import hbm_runtime
        except ImportError as error:
            raise RuntimeError("RDK hbm_runtime不可用") from error
        self.runtime = hbm_runtime.HB_HBMRuntime(str(model_path))
        self.model_name = self.runtime.model_names[0]
        self.input_name = self.runtime.input_names[self.model_name][0]
        self.output_name = self.runtime.output_names[self.model_name][0]
        self.input_shape = self.runtime.input_shapes[self.model_name][self.input_name]
        self.output_quant = self.runtime.output_quants[self.model_name][self.output_name]
        self.runtime.set_scheduling_params(
            priority={self.model_name: 100}, bpu_cores={self.model_name: [0]}
        )
        if list(self.input_shape) != [1, 3, 480, 640]:
            raise RuntimeError(f"BPU模型输入尺寸错误: {self.input_shape}")

    @staticmethod
    def preprocess(bgr_image):
        import cv2

        if bgr_image.shape[:2] != (480, 640):
            bgr_image = cv2.resize(
                bgr_image, (640, 480), interpolation=cv2.INTER_AREA
            )
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(
            rgb.transpose(2, 0, 1)[None], dtype=np.float32
        )

    def infer(self, bgr_image):
        outputs = self.runtime.run(
            {
                self.model_name: {
                    self.input_name: self.preprocess(bgr_image)
                }
            }
        )[self.model_name]
        descriptor = dequantize(
            outputs[self.output_name], self.output_quant
        ).reshape(-1)
        norm = float(np.linalg.norm(descriptor))
        if not math.isfinite(norm) or norm < 1.0e-12:
            raise RuntimeError("BPU输出了无效的视觉描述子")
        return descriptor / norm


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    yaw: float
    score: float
    support: int


def load_database(path: Path):
    with np.load(path, allow_pickle=False) as document:
        required = ("descriptors", "x", "y", "yaw_rad", "pose_frame")
        missing = [name for name in required if name not in document.files]
        if missing:
            raise RuntimeError(f"视觉数据库缺少字段: {', '.join(missing)}")
        pose_frame = str(np.asarray(document["pose_frame"]).item())
        if pose_frame != "base_link":
            raise RuntimeError(f"视觉数据库位姿必须是base_link，实际为{pose_frame}")
        descriptors = np.asarray(document["descriptors"], dtype=np.float32)
        x = np.asarray(document["x"], dtype=np.float64)
        y = np.asarray(document["y"], dtype=np.float64)
        yaw = np.asarray(document["yaw_rad"], dtype=np.float64)
    if descriptors.ndim != 2 or descriptors.shape[0] < 1:
        raise RuntimeError("视觉数据库描述子尺寸无效")
    if any(values.shape != (descriptors.shape[0],) for values in (x, y, yaw)):
        raise RuntimeError("视觉数据库位姿数量与描述子不一致")
    if not all(np.all(np.isfinite(values)) for values in (descriptors, x, y, yaw)):
        raise RuntimeError("视觉数据库包含非有限值")
    norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
    if np.any(norms < 1.0e-12):
        raise RuntimeError("视觉数据库包含零描述子")
    return descriptors / norms, x, y, yaw


def rank_descriptor(descriptor, database, top_k):
    if descriptor.shape != (database.shape[1],):
        raise RuntimeError(
            f"视觉描述子维数不一致: {descriptor.shape}和{database.shape}"
        )
    similarities = database @ descriptor
    count = min(top_k, similarities.size)
    indices = np.argpartition(similarities, -count)[-count:]
    indices = indices[np.argsort(similarities[indices])[::-1]]
    return indices, similarities[indices]


def angle_difference(left, right):
    return math.atan2(math.sin(left - right), math.cos(left - right))


def aggregate_candidates(
    observations,
    x,
    y,
    yaw,
    *,
    minimum_similarity,
    cluster_xy_radius,
    cluster_yaw_rad,
    candidate_limit,
):
    if not observations:
        return []
    records = []
    for frame_index, (indices, scores) in enumerate(observations):
        if not len(indices) or float(scores[0]) < minimum_similarity:
            continue
        retained = scores >= max(minimum_similarity - 0.05, float(scores[0]) - 0.08)
        indices = indices[retained]
        scores = scores[retained]
        weights = np.exp((scores - scores.max()) / 0.02)
        weights /= max(float(weights.sum()), 1.0e-12)
        for index, score, weight in zip(indices, scores, weights):
            records.append(
                {
                    "frame": frame_index,
                    "index": int(index),
                    "score": float(score),
                    "weight": float(weight),
                }
            )
    if not records:
        return []

    clusters = []
    for record in sorted(records, key=lambda value: value["score"], reverse=True):
        index = record["index"]
        selected = None
        for cluster in clusters:
            if (
                math.hypot(x[index] - cluster["center_x"], y[index] - cluster["center_y"])
                <= cluster_xy_radius
                and abs(angle_difference(yaw[index], cluster["center_yaw"]))
                <= cluster_yaw_rad
            ):
                selected = cluster
                break
        if selected is None:
            selected = {
                "records": [],
                "frames": set(),
                "center_x": float(x[index]),
                "center_y": float(y[index]),
                "center_yaw": float(yaw[index]),
            }
            clusters.append(selected)
        selected["records"].append(record)
        selected["frames"].add(record["frame"])
        values = selected["records"]
        total = sum(value["weight"] for value in values)
        selected["center_x"] = sum(
            x[value["index"]] * value["weight"] for value in values
        ) / total
        selected["center_y"] = sum(
            y[value["index"]] * value["weight"] for value in values
        ) / total
        sine = sum(
            math.sin(yaw[value["index"]]) * value["weight"] for value in values
        )
        cosine = sum(
            math.cos(yaw[value["index"]]) * value["weight"] for value in values
        )
        selected["center_yaw"] = math.atan2(sine, cosine)

    minimum_support = max(1, math.ceil(len(observations) * 0.6))
    accepted = []
    for cluster in clusters:
        support = len(cluster["frames"])
        if support < minimum_support:
            continue
        scores = [value["score"] for value in cluster["records"]]
        accepted.append(
            Candidate(
                x=cluster["center_x"],
                y=cluster["center_y"],
                yaw=cluster["center_yaw"],
                score=float(sum(scores) / len(scores)),
                support=support,
            )
        )
    accepted.sort(key=lambda value: (value.support, value.score), reverse=True)
    return accepted[:candidate_limit]


def image_to_bgr(message: Image):
    import cv2

    channels = {
        "bgr8": (3, None),
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
    }
    try:
        channel_count, conversion = channels[message.encoding.lower()]
    except KeyError as error:
        raise RuntimeError(f"不支持相机编码{message.encoding}") from error
    minimum_step = message.width * channel_count
    if message.step < minimum_step:
        raise RuntimeError("相机图像步长小于有效像素宽度")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = rows[:, :minimum_step].reshape(
        message.height, message.width, channel_count
    )
    if conversion is not None:
        image = cv2.cvtColor(image, conversion)
    return np.ascontiguousarray(image)


def transient_qos():
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class VisualPlaceRecognizer(Node):
    def __init__(self):
        super().__init__("visual_place_recognizer")
        self._model_file = Path(
            str(self.declare_parameter("model_file", "").value)
        ).expanduser()
        self._database_file = Path(
            str(self.declare_parameter("database_file", "").value)
        ).expanduser()
        self._image_topic = str(
            self.declare_parameter("image_topic", "/camera/rgb/image_raw").value
        )
        self._minimum_similarity = float(
            self.declare_parameter("minimum_similarity", 0.75).value
        )
        self._top_k = int(self.declare_parameter("top_k", 5).value)
        self._temporal_samples = int(
            self.declare_parameter("temporal_samples", 5).value
        )
        self._candidate_limit = int(
            self.declare_parameter("candidate_limit", 3).value
        )
        self._cluster_xy_radius = float(
            self.declare_parameter("cluster_xy_radius_m", 1.5).value
        )
        self._cluster_yaw_rad = math.radians(
            float(self.declare_parameter("cluster_yaw_deg", 50.0).value)
        )
        inference_period = float(
            self.declare_parameter("inference_period_sec", 0.20).value
        )
        if (
            not 0.0 < self._minimum_similarity <= 1.0
            or min(self._top_k, self._temporal_samples, self._candidate_limit) < 1
            or min(self._cluster_xy_radius, self._cluster_yaw_rad, inference_period)
            <= 0.0
        ):
            raise ValueError("invalid visual relocalization parameters")

        latched = transient_qos()
        self._available_publisher = self.create_publisher(
            Bool, "/localization/visual_available", latched
        )
        self._status_publisher = self.create_publisher(
            String, "/localization/visual_status", latched
        )
        self._candidate_publisher = self.create_publisher(
            PoseArray, "/localization/visual_candidates", 10
        )
        self.create_subscription(
            Bool,
            "/localization/visual_request",
            self._handle_request,
            latched,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image, self._image_topic, self._handle_image, image_qos
        )

        self._latest_image = None
        self._latest_stamp = None
        self._last_processed_stamp = None
        self._requested = False
        self._observations = []
        self._model = None
        self._database = None
        self._x = self._y = self._yaw = None
        try:
            if not self._model_file.is_file():
                raise RuntimeError(f"BPU模型不存在: {self._model_file}")
            if not self._database_file.is_file():
                raise RuntimeError(f"视觉数据库不存在: {self._database_file}")
            self._database, self._x, self._y, self._yaw = load_database(
                self._database_file
            )
            self._model = EigenPlacesBpu(self._model_file)
            if self._database.shape[1] != 512:
                raise RuntimeError(
                    f"视觉数据库描述子维数不是512: {self._database.shape}"
                )
            self._set_available(
                True, f"视觉位置数据库已就绪，共{self._database.shape[0]}个参考位姿"
            )
        except Exception as error:
            self._set_available(False, f"视觉位置识别不可用: {error}")
            self.get_logger().error(str(error))
        self.create_timer(inference_period, self._process_latest)

    def _set_available(self, available, status):
        message = Bool()
        message.data = available
        self._available_publisher.publish(message)
        self._publish_status(status)

    def _publish_status(self, status, **details):
        message = String()
        document = {"message": status, **details}
        message.data = json.dumps(document, ensure_ascii=False, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _handle_request(self, message):
        if not message.data:
            self._requested = False
            self._observations.clear()
            return
        if self._model is None:
            self._publish_empty_candidates("视觉模型或数据库不可用")
            return
        self._requested = True
        self._observations.clear()
        self._last_processed_stamp = None
        self._publish_status("正在采集多帧图像并执行EigenPlaces检索")

    def _handle_image(self, message):
        self._latest_image = message
        self._latest_stamp = (message.header.stamp.sec, message.header.stamp.nanosec)

    def _process_latest(self):
        if (
            not self._requested
            or self._latest_image is None
            or self._latest_stamp == self._last_processed_stamp
        ):
            return
        self._last_processed_stamp = self._latest_stamp
        try:
            started = time.perf_counter()
            descriptor = self._model.infer(image_to_bgr(self._latest_image))
            indices, scores = rank_descriptor(
                descriptor, self._database, self._top_k
            )
            self._observations.append((indices, scores))
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self._publish_status(
                "正在进行视觉位置识别",
                samples=len(self._observations),
                required_samples=self._temporal_samples,
                top1_similarity=round(float(scores[0]), 4),
                inference_ms=round(elapsed_ms, 2),
            )
            if len(self._observations) >= self._temporal_samples:
                self._finish_request()
        except Exception as error:
            self.get_logger().error(f"视觉位置识别失败: {error}")
            self._publish_empty_candidates(f"视觉位置识别失败: {error}")

    def _finish_request(self):
        candidates = aggregate_candidates(
            self._observations,
            self._x,
            self._y,
            self._yaw,
            minimum_similarity=self._minimum_similarity,
            cluster_xy_radius=self._cluster_xy_radius,
            cluster_yaw_rad=self._cluster_yaw_rad,
            candidate_limit=self._candidate_limit,
        )
        message = PoseArray()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for candidate in candidates:
            pose = Pose()
            pose.position.x = candidate.x
            pose.position.y = candidate.y
            pose.orientation.z = math.sin(candidate.yaw / 2.0)
            pose.orientation.w = math.cos(candidate.yaw / 2.0)
            message.poses.append(pose)
        self._candidate_publisher.publish(message)
        self._requested = False
        self._publish_status(
            "视觉候选已生成" if candidates else "视觉检索未形成稳定候选",
            candidate_count=len(candidates),
            candidates=[
                {
                    "x": round(value.x, 3),
                    "y": round(value.y, 3),
                    "yaw_deg": round(math.degrees(value.yaw), 2),
                    "similarity": round(value.score, 4),
                    "support": value.support,
                }
                for value in candidates
            ],
        )

    def _publish_empty_candidates(self, reason):
        message = PoseArray()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        self._candidate_publisher.publish(message)
        self._requested = False
        self._publish_status(reason, candidate_count=0)


def main(args=None):
    rclpy.init(args=args)
    node = VisualPlaceRecognizer()
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
