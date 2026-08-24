#!/usr/bin/env python3

import bisect
import math
import statistics
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2
from std_msgs.msg import Float64, Header


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def percentile(values, fraction):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, round(fraction * (len(ordered) - 1))))
    return ordered[index]


def nearest_residuals(reference_stamps, target_stamps, reference_offset=0.0):
    targets = sorted(target_stamps)
    if not reference_stamps or not targets:
        return []
    residuals = []
    for raw_stamp in reference_stamps:
        stamp = raw_stamp + reference_offset
        index = bisect.bisect_left(targets, stamp)
        candidates = []
        if index < len(targets):
            candidates.append(abs(targets[index] - stamp))
        if index > 0:
            candidates.append(abs(targets[index - 1] - stamp))
        residuals.append(min(candidates))
    return residuals


class TopicTiming:
    def __init__(self, name, topic, minimum_rate_hz, maximum_age_sec, capacity):
        self.name = name
        self.topic = topic
        self.minimum_rate_hz = minimum_rate_hz
        self.maximum_age_sec = maximum_age_sec
        self.samples = deque(maxlen=capacity)
        self.last_stamp = None
        self.regressions = 0
        self.zero_stamps = 0

    def add(self, stamp, receipt):
        if stamp <= 0.0:
            self.zero_stamps += 1
        if self.last_stamp is not None and stamp + 1.0e-9 < self.last_stamp:
            self.regressions += 1
        self.last_stamp = stamp
        self.samples.append((receipt, stamp))

    def trim(self, now, window_sec):
        cutoff = now - window_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def rate_hz(self):
        if len(self.samples) < 2:
            return 0.0
        duration = self.samples[-1][0] - self.samples[0][0]
        return (len(self.samples) - 1) / duration if duration > 0.0 else 0.0

    def ages(self):
        return [receipt - stamp for receipt, stamp in self.samples if stamp > 0.0]

    def stamps(self):
        return [stamp for _, stamp in self.samples if stamp > 0.0]


class SensorTimeSyncMonitor(Node):
    def __init__(self):
        super().__init__("sensor_time_sync_monitor")
        self.window_sec = float(self.declare_parameter("window_sec", 10.0).value)
        self.stale_timeout_sec = float(
            self.declare_parameter("stale_timeout_sec", 3.0).value
        )
        self.stamp_fallback_timeout_sec = float(
            self.declare_parameter("stamp_fallback_timeout_sec", 5.0).value
        )
        capacity = int(self.declare_parameter("sample_capacity", 4000).value)

        self.lidar_topic = self.declare_parameter(
            "lidar_topic", "/lidar/points"
        ).value
        self.lidar_stamp_topic = self.declare_parameter(
            "lidar_stamp_topic", "/time_topic"
        ).value
        self.camera_topic = self.declare_parameter(
            "camera_topic", "/camera/rgb/image_raw"
        ).value
        self.camera_stamp_topic = self.declare_parameter(
            "camera_stamp_topic", "/camera/rgb/frame_stamp"
        ).value

        definitions = {
            "lidar": (
                self.lidar_topic,
                float(self.declare_parameter("lidar_minimum_rate_hz", 8.0).value),
                float(self.declare_parameter("lidar_maximum_age_sec", 0.30).value),
            ),
            "imu": (
                self.declare_parameter("imu_topic", "/imu/data").value,
                float(self.declare_parameter("imu_minimum_rate_hz", 80.0).value),
                float(self.declare_parameter("imu_maximum_age_sec", 0.20).value),
            ),
            "camera": (
                self.camera_topic,
                float(self.declare_parameter("camera_minimum_rate_hz", 8.0).value),
                float(self.declare_parameter("camera_maximum_age_sec", 0.30).value),
            ),
            "rtk": (
                self.declare_parameter("rtk_topic", "/rtk/fix").value,
                float(self.declare_parameter("rtk_minimum_rate_hz", 0.5).value),
                float(self.declare_parameter("rtk_maximum_age_sec", 1.50).value),
            ),
        }
        self.timings = {
            name: TopicTiming(name, topic, rate, age, capacity)
            for name, (topic, rate, age) in definitions.items()
        }

        self.pair_tolerances = {
            "lidar_imu": float(
                self.declare_parameter("lidar_imu_tolerance_sec", 0.020).value
            ),
            "lidar_camera": float(
                self.declare_parameter("lidar_camera_tolerance_sec", 0.080).value
            ),
            "lidar_rtk": float(
                self.declare_parameter("lidar_rtk_tolerance_sec", 0.150).value
            ),
        }
        self.lidar_forward_point_offset_sec = float(
            self.declare_parameter(
                "lidar_forward_point_offset_sec", 0.02504
            ).value
        )
        self.pair_maximum_match_sec = float(
            self.declare_parameter("pair_maximum_match_sec", 0.040).value
        )
        self.pair_minimum_match_ratio = float(
            self.declare_parameter("pair_minimum_match_ratio", 0.80).value
        )

        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.lightweight_stamp_seen = {"lidar": False, "camera": False}
        self.fallback_subscriptions = {}
        self.monitor_started_at = self.get_clock().now().nanoseconds * 1.0e-9
        self.create_subscription(
            Float64,
            self.lidar_stamp_topic,
            lambda message: self.record_lightweight_stamp(
                "lidar", float(message.data)
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Header,
            self.camera_stamp_topic,
            lambda message: self.record_lightweight_stamp(
                "camera", stamp_to_sec(message.stamp)
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            definitions["imu"][0],
            lambda message: self.record("imu", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            definitions["rtk"][0],
            lambda message: self.record("rtk", message),
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self.publish_diagnostics)

    def record(self, name, message):
        receipt = self.get_clock().now().nanoseconds * 1.0e-9
        self.timings[name].add(stamp_to_sec(message.header.stamp), receipt)

    def record_lightweight_stamp(self, name, stamp):
        receipt = self.get_clock().now().nanoseconds * 1.0e-9
        self.timings[name].add(stamp, receipt)
        self.lightweight_stamp_seen[name] = True
        subscription = self.fallback_subscriptions.pop(name, None)
        if subscription is not None:
            self.destroy_subscription(subscription)

    def ensure_stamp_fallbacks(self, now):
        if now - self.monitor_started_at < self.stamp_fallback_timeout_sec:
            return
        if (
            not self.lightweight_stamp_seen["lidar"]
            and "lidar" not in self.fallback_subscriptions
        ):
            self.fallback_subscriptions["lidar"] = self.create_subscription(
                PointCloud2,
                self.lidar_topic,
                lambda message: self.record("lidar", message),
                qos_profile_sensor_data,
            )
            self.get_logger().warning(
                "%s无数据，回退订阅完整点云", self.lidar_stamp_topic
            )
        if (
            not self.lightweight_stamp_seen["camera"]
            and "camera" not in self.fallback_subscriptions
        ):
            self.fallback_subscriptions["camera"] = self.create_subscription(
                Image,
                self.camera_topic,
                lambda message: self.record("camera", message),
                qos_profile_sensor_data,
            )
            self.get_logger().warning(
                "%s无数据，回退订阅完整图像", self.camera_stamp_topic
            )

    @staticmethod
    def value(key, value):
        return KeyValue(key=key, value=str(value))

    def topic_status(self, timing, now):
        status = DiagnosticStatus()
        status.name = f"sensor_time_sync/{timing.name}"
        status.hardware_id = timing.topic
        if not timing.samples or now - timing.samples[-1][0] > self.stale_timeout_sec:
            status.level = DiagnosticStatus.STALE
            status.message = "未收到近期数据"
            return status

        rate = timing.rate_hz()
        ages = timing.ages()
        median_age = statistics.median(ages) if ages else math.nan
        p95_abs_age = percentile([abs(value) for value in ages], 0.95)
        problems = []
        if rate < timing.minimum_rate_hz:
            problems.append("频率偏低")
        if timing.regressions:
            problems.append("时间戳发生倒退")
        if timing.zero_stamps:
            problems.append("出现零时间戳")
        if not math.isnan(p95_abs_age) and p95_abs_age > timing.maximum_age_sec:
            problems.append("时间戳与接收时刻偏差过大")

        status.level = DiagnosticStatus.WARN if problems else DiagnosticStatus.OK
        status.message = "；".join(problems) if problems else "时间戳正常"
        status.values = [
            self.value("topic", timing.topic),
            self.value("samples_in_window", len(timing.samples)),
            self.value("rate_hz", f"{rate:.3f}"),
            self.value("median_receipt_minus_stamp_ms", f"{median_age * 1000.0:.3f}"),
            self.value("p95_abs_receipt_minus_stamp_ms", f"{p95_abs_age * 1000.0:.3f}"),
            self.value("timestamp_regressions", timing.regressions),
            self.value("zero_timestamps", timing.zero_stamps),
        ]
        return status

    def pair_status(
        self,
        pair_name,
        left_name,
        right_name,
        reference_offset_sec=0.0,
        maximum_match_sec=None,
    ):
        status = DiagnosticStatus()
        status.name = f"sensor_time_sync/{pair_name}"
        status.hardware_id = "software_sync"
        raw_residuals = nearest_residuals(
            self.timings[left_name].stamps(),
            self.timings[right_name].stamps(),
            reference_offset_sec,
        )
        if not raw_residuals:
            status.level = DiagnosticStatus.STALE
            status.message = "没有可配对的数据"
            return status

        maximum_match = (
            math.inf if maximum_match_sec is None else maximum_match_sec
        )
        residuals = [
            residual for residual in raw_residuals if residual <= maximum_match
        ]
        unmatched = len(raw_residuals) - len(residuals)
        match_ratio = len(residuals) / len(raw_residuals)
        if not residuals:
            status.level = DiagnosticStatus.WARN
            status.message = "没有同周期可配对的数据"
            status.values = [
                self.value("reference", left_name),
                self.value("target", right_name),
                self.value(
                    "reference_offset_ms",
                    f"{reference_offset_sec * 1000.0:.3f}",
                ),
                self.value("pairs", 0),
                self.value("unmatched_reference_samples", unmatched),
                self.value("match_ratio", f"{match_ratio:.3f}"),
            ]
            return status

        median_residual = statistics.median(residuals)
        p95_residual = percentile(residuals, 0.95)
        tolerance = self.pair_tolerances[pair_name]
        problems = []
        if p95_residual > tolerance:
            problems.append("同周期测量时刻偏差超限")
        if match_ratio < self.pair_minimum_match_ratio:
            problems.append("同周期配对率偏低")
        status.level = (
            DiagnosticStatus.WARN if problems else DiagnosticStatus.OK
        )
        if problems:
            status.message = "；".join(problems)
        elif reference_offset_sec:
            status.message = "前向雷达点与测量时刻正常"
        else:
            status.message = "相邻测量时刻正常"
        status.values = [
            self.value(
                "reference",
                "lidar_forward_point" if reference_offset_sec else left_name,
            ),
            self.value("target", right_name),
            self.value(
                "reference_offset_ms", f"{reference_offset_sec * 1000.0:.3f}"
            ),
            self.value("pairs", len(residuals)),
            self.value("unmatched_reference_samples", unmatched),
            self.value("match_ratio", f"{match_ratio:.3f}"),
            self.value(
                "median_nearest_delta_ms",
                f"{median_residual * 1000.0:.3f}",
            ),
            self.value(
                "p95_nearest_delta_ms",
                f"{p95_residual * 1000.0:.3f}",
            ),
            self.value("tolerance_ms", f"{tolerance * 1000.0:.3f}"),
        ]
        return status

    def publish_diagnostics(self):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        self.ensure_stamp_fallbacks(now)
        for timing in self.timings.values():
            timing.trim(now, self.window_sec)

        statuses = [
            self.topic_status(timing, now) for timing in self.timings.values()
        ]
        statuses.extend(
            [
                self.pair_status("lidar_imu", "lidar", "imu"),
                self.pair_status(
                    "lidar_camera",
                    "lidar",
                    "camera",
                    self.lidar_forward_point_offset_sec,
                    self.pair_maximum_match_sec,
                ),
                self.pair_status(
                    "lidar_rtk",
                    "lidar",
                    "rtk",
                    self.lidar_forward_point_offset_sec,
                    self.pair_maximum_match_sec,
                ),
            ]
        )
        summary = DiagnosticStatus()
        summary.name = "sensor_time_sync/summary"
        summary.hardware_id = "software_sync"
        active = [status for status in statuses if status.level != DiagnosticStatus.STALE]
        summary.level = max(
            (status.level for status in active), default=DiagnosticStatus.STALE
        )
        if not active:
            summary.message = "尚未收到传感器数据"
        elif summary.level == DiagnosticStatus.OK:
            summary.message = "已接收传感器的软同步检查通过"
        else:
            summary.message = "部分软同步指标需要检查"
        statuses.insert(0, summary)

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = statuses
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SensorTimeSyncMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
