"""Warm up the second estimator without cancelling the current navigation goal."""

from collections import deque
import copy
import math
from pathlib import Path
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, UInt8
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .children import EstimatorProcess
from .core import Handover, Sample, finite_pose, motion_discontinuity, sample_health_reason


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def matrix(pose):
    q, p = pose.orientation, pose.position
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [p.x, p.y, p.z]
    if not finite_pose(result):
        raise ValueError("Invalid pose")
    return result


def assign_pose(pose, value):
    pose.position.x, pose.position.y, pose.position.z = map(float, value[:3, 3])
    q = Rotation.from_matrix(value[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


class LocalizationSwitch(Node):
    def __init__(self):
        super().__init__("localization_switch")
        self.lock = threading.RLock()
        self.closing = threading.Event()
        self.map_base = str(self.declare_parameter("map_base", "").value)
        if not self.map_base:
            raise ValueError("map_base is required")
        self.wait_requested = bool(self.declare_parameter("wait_for_request", True).value)
        self.enabled = not self.wait_requested
        self.gnss_holdover_sec = float(self.declare_parameter("navsat_gnss_holdover_sec", 4.0).value)
        if not math.isfinite(self.gnss_holdover_sec) or not .5 <= self.gnss_holdover_sec <= 4.0:
            raise ValueError("navsat_gnss_holdover_sec must be in [0.5, 4.0]")
        self.navsat_initialized = False
        self.last_fix_stamp = -math.inf
        self.handover_xy_variance = float(self.declare_parameter("handover_xy_variance_limit", 1.0).value)
        self.handover_prediction_sec = float(self.declare_parameter("handover_prediction_sec", .5).value)
        stable = float(self.declare_parameter("handover_stable_sec", 1.0).value)
        distance = float(self.declare_parameter("handover_max_distance_m", 1.0).value)
        angle = float(self.declare_parameter("handover_max_angle_deg", 20.0).value)
        for name, value, lower, upper in (
                ("handover_xy_variance_limit", self.handover_xy_variance, .25, 1.0),
                ("handover_prediction_sec", self.handover_prediction_sec, .2, .5),
                ("handover_stable_sec", stable, .5, 5.),
                ("handover_max_distance_m", distance, .1, 1.),
                ("handover_max_angle_deg", angle, 5., 20.)):
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"{name} must be in [{lower}, {upper}]")
        self.core = Handover(stable_sec=stable, max_distance=distance, max_angle_deg=angle)
        self.samples, self.messages = {}, {}
        self.sample_errors = {}
        self.motion_horizon = 4.0
        self.navsat_history = deque(maxlen=500)
        self.raw_history = deque(maxlen=100)
        self.quality, self.quality_at = None, -math.inf
        self.fix_at = self.heading_at = -math.inf
        self.fused_ready, self.fused_ready_at = False, -math.inf
        self.last_output = None
        self.last_stamp = -math.inf
        self.last_source_stamp = None
        self.last_overlap_stamp = None
        self.last_overlap_log = -math.inf
        self.last_health_status = None
        self.last_tick = time.monotonic()
        self.launching = False
        self.launch_error = ""
        log_root = Path.home() / "agribot_mobile_logs" / "localization_switch"
        sim = bool(self.get_parameter("use_sim_time").value)
        self.navsat = EstimatorProcess("navsat", self.map_base, log_root, sim)
        self.fastlivo = EstimatorProcess("fastlivo", self.map_base, log_root, sim)
        self.workers = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.odom_pub = self.create_publisher(Odometry, "/fastlivo_rtk/odometry", 1)
        self.ready_pub = self.create_publisher(Bool, "/fastlivo_rtk/ready", latched)
        self.localization_ready_pub = self.create_publisher(Bool, "/localization/ready", latched)
        self.localization_status_pub = self.create_publisher(String, "/localization/status", latched)
        self.fixed_pub = self.create_publisher(Bool, "/fastlivo_rtk/fixed_active", latched)
        self.mode_pub = self.create_publisher(String, "/localization/active_method", latched)
        self.stage_pub = self.create_publisher(String, "/localization/initialization_stage", latched)
        self.source_pub = self.create_publisher(String, "/localization/initialization_source", latched)
        self.status_pub = self.create_publisher(String, "/localization/initialization_status", latched)
        self.seed_pub = self.create_publisher(PoseWithCovarianceStamped, "/localization/handover_seed", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/localization/navsat/odometry", self.navsat_callback, 1)
        self.create_subscription(Odometry, "/localization/fastlivo/odometry", self.fused_callback, 1)
        self.create_subscription(Odometry, "/fastlivo/odometry", self.raw_callback, 1)
        self.create_subscription(Bool, "/localization/fastlivo/ready", self.fused_flag, latched)
        self.create_subscription(UInt8, "/rtk/fix_quality", self.quality_callback, 1)
        self.create_subscription(NavSatFix, "/rtk/fix", self.fix_callback, 1)
        self.create_subscription(PoseWithCovarianceStamped, "/rtk/heading_with_covariance", self.heading_callback, 1)
        self.create_service(Trigger, "/localization/start", self.start_callback)
        self.create_service(Trigger, "/localization/switch_to_fastlivo", self.switch_callback)
        self.create_timer(.05, self.tick)
        self.create_timer(.2, self.flags)
        self.navsat.start()

    def quality_callback(self, message):
        self.quality, self.quality_at = message.data, time.monotonic()

    def fix_callback(self, message):
        now = time.monotonic()
        stamp = stamp_seconds(message.header.stamp)
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        covariance = message.position_covariance
        if (self.quality == 4 and 0 <= now-self.quality_at <= .3
                and message.status.status >= 0 and -.05 <= age <= .5
                and stamp > self.last_fix_stamp
                and all(math.isfinite(value) for value in covariance)
                and all(0 <= covariance[index] for index in (0, 4, 8))
                and max(covariance[0], covariance[4]) <= .25
                and math.isfinite(message.altitude)
                and -90 <= message.latitude <= 90 and -180 <= message.longitude <= 180):
            self.fix_at, self.last_fix_stamp = now, stamp

    def heading_callback(self, message):
        variance = message.pose.covariance[35]
        age = self.get_clock().now().nanoseconds*1e-9 - stamp_seconds(message.header.stamp)
        if -.05 <= age <= 2.0 and math.isfinite(variance) and 0 <= variance <= math.radians(10)**2:
            self.heading_at = time.monotonic()

    def fused_flag(self, message):
        self.fused_ready, self.fused_ready_at = message.data, time.monotonic()

    def receive(self, name, message):
        if message.header.frame_id.lstrip("/") != "map" or message.child_frame_id != "base_link":
            self.sample_errors[name] = "Localization frame is not map/base_link"
            return
        try:
            twist = message.twist.twist
            covariance = np.array(message.pose.covariance, dtype=float).reshape(6, 6)
            velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
            if not np.isfinite(covariance).all() or not np.isfinite(velocity).all() or not np.isfinite(angular).all():
                self.sample_errors[name] = "Non-finite localization covariance or twist"
                return
            sample = Sample(stamp_seconds(message.header.stamp), time.monotonic(),
                            matrix(message.pose.pose), velocity, angular, covariance)
            if sample.stamp <= self.samples.get(name, sample).stamp and name in self.samples:
                return
            reason = sample_health_reason(sample, sample.received,
                                          self.get_clock().now().nanoseconds*1e-9, 4.0)
            if not reason and name == "fastlivo_rtk" and self.core.mode == "fastlivo_rtk":
                previous = self.samples[name]
                reason = motion_discontinuity(previous, sample, self.motion_horizon)
                if reason:
                    # Repeating an outlier must not enlarge its acceptance
                    # envelope simply because the valid output is now older.
                    self.motion_horizon = min(self.motion_horizon, sample.stamp-previous.stamp)
                else:
                    self.motion_horizon = 4.0
            if reason:
                self.sample_errors[name] = reason
                return
            self.samples[name], self.messages[name] = sample, message
            self.sample_errors.pop(name, None)
            if name == "navsat":
                self.navsat_history.append(sample)
        except (ValueError, TypeError, np.linalg.LinAlgError):
            self.sample_errors[name] = "Invalid localization pose or covariance"
            return

    def navsat_callback(self, message):
        self.receive("navsat", message)

    def fused_callback(self, message):
        self.receive("fastlivo_rtk", message)

    def navsat_at(self, stamp):
        for sample in reversed(self.navsat_history):
            if sample.stamp <= stamp:
                return sample.at(stamp)
        raise ValueError("No matching NavSat history")

    def raw_callback(self, message):
        if message.header.frame_id.lstrip("/") != "odom" or message.child_frame_id != "base_link":
            return
        try:
            self.raw_history.append((stamp_seconds(message.header.stamp), matrix(message.pose.pose)))
        except ValueError:
            return
        if not self.core.requested or self.core.mode == "fastlivo_rtk" or self.fused_ready:
            return
        if not self.navsat_healthy(time.monotonic()):
            return
        try:
            # Predict the timestamped INS state, never relabel an old pose as a
            # current measurement. Prediction is bounded to 350 ms.
            pose = self.navsat_at(stamp_seconds(message.header.stamp))
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = "map"
            seed.header.stamp = message.header.stamp
            assign_pose(seed.pose.pose, pose)
            seed.pose.covariance = list(self.samples["navsat"].covariance.reshape(-1))
            self.seed_pub.publish(seed)
        except ValueError:
            pass

    def navsat_healthy(self, now):
        if not (self.enabled and self.sample_healthy("navsat", now, .5) and self.navsat.alive()):
            return False
        if not self.navsat_initialized:
            if not (self.quality == 4 and 0 <= now-self.quality_at <= .3
                    and 0 <= now-self.fix_at <= .5 and 0 <= now-self.heading_at <= 2.0):
                return False
            self.navsat_initialized = True
        # The backend continues real IMU propagation while rejecting bad GNSS.
        # A single bad/missing observation must not cancel an otherwise healthy INS.
        return 0 <= now-self.fix_at <= self.gnss_holdover_sec

    def sample_healthy(self, name, now, maximum, xy_variance_limit=.25):
        return not (self.sample_errors.get(name) or sample_health_reason(
            self.samples.get(name), now, self.get_clock().now().nanoseconds*1e-9,
            maximum, xy_variance_limit))

    def fused_health_reason(self, now, maximum=4.0):
        if not self.fastlivo.alive():
            return "FAST-LIVO2/RTK process is not running"
        if not self.fused_ready or not 0 <= now-self.fused_ready_at <= 2.0:
            return "FAST-LIVO2/RTK backend is not ready or its heartbeat is stale"
        # Absolute map covariance grows without GNSS. It is an initialization
        # gate, not evidence of a stalled/broken local odometry after takeover.
        limit = self.handover_xy_variance if self.core.mode != "fastlivo_rtk" else None
        return self.sample_errors.get("fastlivo_rtk") or sample_health_reason(
            self.samples.get("fastlivo_rtk"), now,
            self.get_clock().now().nanoseconds*1e-9, maximum, limit)

    def fused_healthy(self, now, maximum=4.0):
        return not self.fused_health_reason(now, maximum)

    def check_handover(self, now, ros_time, navsat_ok):
        # Losing GNSS must not veto a switch while the independent INS still
        # supplies fresh, bounded-uncertainty poses for the overlap comparison.
        overlap_ok = navsat_ok or (self.navsat_initialized and self.navsat.alive()
            and self.sample_healthy("navsat", now, .75, self.handover_xy_variance))
        if not (overlap_ok and self.fused_healthy(now, 1.0)):
            if now-self.last_overlap_log >= 5.0:
                self.last_overlap_log = now
                sample = self.samples.get("fastlivo_rtk")
                age = ros_time-sample.stamp if sample else math.inf
                self.get_logger().info(
                    f"Handover waiting: navsat_ready={navsat_ok}, "
                    f"fused_flag={self.fused_ready}, fused_source_age={age:.3f}s")
            return self.core.consider(None, None, False, now)
        sample = self.samples["fastlivo_rtk"]
        # Validate each measurement once. A normal inter-frame interval is not
        # evidence that the preceding valid overlap was lost.
        if sample.stamp == self.last_overlap_stamp:
            return False
        if self.last_overlap_stamp is not None and sample.stamp-self.last_overlap_stamp > 1.0:
            self.core.consider(None, None, False, now)
        self.last_overlap_stamp = sample.stamp
        try:
            navsat_pose = self.navsat_at(sample.stamp)
            fresh = self.fused_healthy(now, self.handover_prediction_sec)
            output_pose = sample.at(ros_time, maximum=self.handover_prediction_sec) if fresh else sample.pose
        except ValueError:
            return self.core.consider(None, None, False, now)
        switched = self.core.consider(
            navsat_pose, sample.pose, True, now, last_output=self.last_output,
            can_take_over=fresh, output_fused_pose=output_pose)
        if now-self.last_overlap_log >= 5.0 or switched:
            self.last_overlap_log = now
            distance = np.linalg.norm(navsat_pose[:3, 3]-sample.pose[:3, 3])
            self.get_logger().info(
                f"Handover: {self.core.reason}; distance={distance:.3f}m, "
                f"source_age={ros_time-sample.stamp:.3f}s, switched={switched}")
        return switched

    def start_callback(self, _request, response):
        self.enabled = True
        response.success, response.message = True, "NavSat initialization enabled"
        return response

    def switch_callback(self, _request, response):
        with self.lock:
            first = self.core.request()
            if first:
                self.launching = True
                self.worker(self.start_fastlivo)
        response.success = True
        response.message = "FAST-LIVO2/RTK requested; NavSat remains active until stable handover"
        return response

    def worker(self, function):
        thread = threading.Thread(target=function, daemon=True)
        self.workers.append(thread)
        thread.start()

    def start_fastlivo(self):
        try:
            if not self.closing.is_set():
                self.fastlivo.start()
        except Exception as error:
            self.launch_error = str(error)
            self.get_logger().error(f"FAST-LIVO2 warmup failed; keeping NavSat: {error}")
        finally:
            self.launching = False

    def flags(self):
        now = time.monotonic()
        ready = bool(self.navsat_healthy(now) if self.core.mode == "navsat" else self.fused_healthy(now))
        status = self.launch_error or self.core.reason
        if self.core.mode == "navsat" and not self.launch_error:
            if not self.enabled:
                status = "Waiting for initialization request"
            elif not ready:
                status = "Waiting for healthy NavSat INS and valid fixed RTK"
            elif now-self.fix_at > .5:
                status = "NavSat active: short RTK gap, IMU propagation"
        elif self.core.mode == "fastlivo_rtk":
            if not ready:
                status = self.fused_health_reason(now)
            elif not (self.quality == 4 and now-self.quality_at <= .5):
                variance = np.diag(self.samples["fastlivo_rtk"].covariance)[:2]
                status = ("FAST-LIVO2/RTK active: local odometry, no fixed RTK; "
                          f"global XY variance={variance[0]:.3f},{variance[1]:.3f} m^2")
        health_status = (ready, status if not ready else self.core.mode)
        if health_status != self.last_health_status:
            self.last_health_status = health_status
            if ready:
                self.get_logger().info(status)
            else:
                self.get_logger().warning(status)
        self.ready_pub.publish(Bool(data=ready))
        self.localization_ready_pub.publish(Bool(data=ready))
        self.localization_status_pub.publish(String(data=status))
        self.fixed_pub.publish(Bool(data=self.quality == 4 and now-self.quality_at <= .5))
        mode = self.core.mode + ("_warming_fastlivo" if self.core.requested and self.core.mode == "navsat" else "")
        self.mode_pub.publish(String(data=mode))
        self.source_pub.publish(String(data="rtk" if self.core.mode == "navsat" else "navsat_handover"))
        stage = ("ready" if ready else "wait_localization") if self.core.mode == "fastlivo_rtk" else (
            "ready" if ready else ("wait_rtk" if self.enabled else "wait_request"))
        self.stage_pub.publish(String(data=stage))
        self.status_pub.publish(String(data=status))

    def tick(self):
        now = time.monotonic()
        stamp = self.get_clock().now()
        ros_time = stamp.nanoseconds*1e-9
        dt, self.last_tick = now-self.last_tick, now
        navsat_ok = self.navsat_healthy(now) if self.core.mode == "navsat" else False
        switched = False
        if self.core.requested and self.core.mode == "navsat":
            switched = self.check_handover(now, ros_time, navsat_ok)
            if switched:
                self.last_source_stamp = None
                self.worker(self.stop_navsat)
        name = self.core.mode
        healthy = navsat_ok if name == "navsat" else self.fused_healthy(now)
        if not healthy:
            return
        sample = self.samples[name]
        if sample.stamp == self.last_source_stamp:
            return
        self.last_source_stamp = sample.stamp
        target_time = ros_time if 0 <= ros_time-sample.stamp <= .35 else sample.stamp
        pose = sample.at(target_time)
        if name == "fastlivo_rtk":
            if not switched:
                self.core.relax_offset(dt, reference=pose)
            pose = self.core.corrected(pose)
        if target_time <= self.last_stamp:
            return
        message = copy.deepcopy(self.messages[name])
        message.header.frame_id, message.child_frame_id = "map", "base_link"
        message.header.stamp = rclpy.time.Time(seconds=target_time).to_msg()
        assign_pose(message.pose.pose, pose)
        rotation = np.eye(6)
        if name == "fastlivo_rtk":
            rotation[:3, :3] = rotation[3:, 3:] = self.core.offset[:3, :3]
        covariance = rotation @ sample.covariance @ rotation.T
        covariance[:3, :3] += np.eye(3) * (.1*(target_time-sample.stamp))**2
        message.pose.covariance = covariance.reshape(-1).tolist()
        self.odom_pub.publish(message)
        identity = TransformStamped()
        identity.header = message.header
        identity.child_frame_id = "odom"
        identity.transform.rotation.w = 1.0
        base = TransformStamped()
        base.header = copy.deepcopy(message.header)
        base.header.frame_id, base.child_frame_id = "odom", "base_link"
        base.transform.translation.x, base.transform.translation.y, base.transform.translation.z = map(float, pose[:3, 3])
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        base.transform.rotation.x, base.transform.rotation.y, base.transform.rotation.z, base.transform.rotation.w = map(float, q)
        transforms = [identity, base]
        if name == "fastlivo_rtk" and self.raw_history:
            raw_stamp, raw_pose = min(self.raw_history, key=lambda item: abs(item[0]-sample.stamp))
            if abs(raw_stamp-sample.stamp) <= .001:
                camera = TransformStamped()
                camera.header = copy.deepcopy(base.header)
                camera.header.stamp = self.messages[name].header.stamp
                camera.child_frame_id = "camera_init"
                value = self.core.corrected(sample.pose) @ np.linalg.inv(raw_pose)
                camera.transform.translation.x, camera.transform.translation.y, camera.transform.translation.z = map(float, value[:3, 3])
                q = Rotation.from_matrix(value[:3, :3]).as_quat()
                camera.transform.rotation.x, camera.transform.rotation.y, camera.transform.rotation.z, camera.transform.rotation.w = map(float, q)
                transforms.append(camera)
        self.tf.sendTransform(transforms)
        self.last_output, self.last_stamp = pose, target_time

    def stop_navsat(self):
        try:
            self.navsat.stop()
        except Exception as error:
            self.get_logger().error(f"NavSat cleanup failed: {error}")

    def close(self):
        self.closing.set()
        for thread in self.workers:
            thread.join(timeout=35)
        try:
            self.fastlivo.stop()
        finally:
            self.navsat.stop()


def main():
    rclpy.init()
    node = None
    try:
        node = LocalizationSwitch()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
