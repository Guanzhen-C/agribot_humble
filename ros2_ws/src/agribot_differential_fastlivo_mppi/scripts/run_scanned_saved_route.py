#!/usr/bin/env python3
"""Validate a complete saved real route with the actual configured Gazebo car.

Navigation observes KF-GINS and the real Nav2 costmap. Ground truth is used only
for acceptance measurements, never to publish TF, commands or navigation poses.
"""
from array import array
import argparse
import hashlib
import json
import math
import os
from pathlib import Path as FilePath
import shutil
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker, MarkerArray

from agribot_differential_fastlivo_mppi.saved_route_runtime.catalog import GridData
from agribot_differential_fastlivo_mppi.saved_route_runtime.navigation_health import LocalizationUnavailable
from agribot_differential_fastlivo_mppi.saved_route_runtime.saved_path import arc_lengths, forward_index
from agribot_differential_fastlivo_mppi.saved_route_runtime.saved_path_runner import SavedPathRunner
from agribot_differential_fastlivo_mppi.saved_route_runtime.route_preflight import footprint_cells
from fuxi_farm_profile import compare_route_poses


def pose_dict(message):
    p = message.pose.pose
    q = p.orientation
    return {"x": p.position.x, "y": p.position.y, "z": p.position.z,
            "yaw": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}


class Validation(Node):
    def __init__(self, bundle, route_file=None, results_dir=None, plan_manual_points=False):
        super().__init__("scanned_saved_route_validation")
        self.bundle = bundle
        self.output = results_dir or bundle
        self.output.mkdir(parents=True, exist_ok=True)
        self.full_run = route_file is None
        self.route = json.loads((route_file or bundle/"saved_route.json").read_text())
        self.preparation = json.loads((bundle/"preparation.json").read_text())
        self.plan_manual_points = plan_manual_points
        self.planning_verified = False
        self.planning_request = self.planning_handle = self.planning_result = None
        self.planning_started = None
        self.planning_report = None
        self.reference_path = self.route["path"]
        self.physics_model = None
        runtime = FilePath(tempfile.gettempdir())/"configured_ackermann_vehicle.generated.sdf"
        if runtime.is_file():
            plugin = ET.parse(runtime).find("model/plugin[@name='continuous_track_drive']")
            if plugin is not None:
                library = FilePath(plugin.get("filename"))
                shutil.copy2(runtime, self.output/"runtime_vehicle.sdf")
                self.physics_model = {"contact_model": "Gazebo SimpleTrackedVehiclePlugin ODE surface motion",
                                      "library": str(library), "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
                                      "parameters": {key: float(plugin.findtext(key)) for key in
                                                     ("max_linear_speed", "max_angular_speed", "steering_efficiency",
                                                      "max_linear_acceleration", "max_angular_acceleration")}}
        self.footprint = self.preparation.get("navigation_footprint") or [[.7, .53], [.7, -.53], [-.7, -.53], [-.7, .53]]
        self.path = self.route["path"]
        self.arc = arc_lengths(self.path)
        self.anchors = np.asarray(self.route["anchor_path_indices"])
        self.anchor_minima = np.full(len(self.anchors), np.inf)
        self.grid = self.estimate = self.truth = None
        self.pose_wall = 0
        self.started = False
        self.finished = False
        self.outcome = None
        self.terminal_at = None
        self.progress = self.gt_progress = 0
        self.events = []
        self.samples = []
        self.last_print = 0
        self.created_at = time.monotonic()
        self.gt_v = self.gt_w = self.command_v = self.command_w = 0.0
        self.max_tilt = 0.0
        self.gt_roll = self.gt_pitch = 0.0
        self.physical_failure = None
        self.group = ReentrantCallbackGroup()
        lifecycle_names = ["controller_server", "planner_server"]
        if self.plan_manual_points:
            if len(self.route.get("manual_poses", [])) != 104:
                raise ValueError("The complete original 104 manual points are required")
            lifecycle_names.append("fuxi_route/planner_server")
            self.smac_plan = ActionClient(self, ComputePathThroughPoses, "/fuxi_route/compute_path_through_poses", callback_group=self.group)
        self.lifecycle_clients = {name: self.create_client(GetState, f"/{name}/get_state", callback_group=self.group)
                                  for name in lifecycle_names}
        self.lifecycle_futures = {}
        self.lifecycle_active = set()
        self.lifecycle_checked = 0.
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.planned = self.create_publisher(Path, "/validation/full_saved_path", transient)
        self.actual = self.create_publisher(Path, "/validation/actual_path", transient)
        self.markers = self.create_publisher(MarkerArray, "/validation/anchors", transient)
        self.create_subscription(Odometry, "/odometry/filtered_navsat", self.odometry, 10, callback_group=self.group)
        self.create_subscription(Odometry, "/validation/physics_truth", self.ground_truth, 10, callback_group=self.group)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.costmap, transient, callback_group=self.group)
        self.create_subscription(Twist, "/nav2/cmd_vel", self.command, 10, callback_group=self.group)
        self.runner = SavedPathRunner(self, self.observe, self.update, self.publish_path,
                                      self.footprint,
                                      goal_tolerance=self.preparation.get("goal_tolerance_m", .25),
                                      callback_group=self.group)
        self.create_timer(.1, self.tick)
        if not self.plan_manual_points:
            self.publish_path(self.path)
        self.publish_markers()

    def odometry(self, message):
        self.estimate = pose_dict(message)
        self.pose_wall = time.monotonic()

    def ground_truth(self, message):
        self.truth = pose_dict(message)
        self.gt_v = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        self.gt_w = message.twist.twist.angular.z
        q = message.pose.pose.orientation
        angles = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
        self.gt_roll, self.gt_pitch = float(angles[0]), float(angles[1])
        self.max_tilt = max(self.max_tilt, float(max(abs(angles[:2]))))

    def command(self, message):
        self.command_v, self.command_w = message.linear.x, message.angular.z

    def costmap(self, message):
        q = message.info.origin.orientation
        yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        self.grid = GridData(message.info.width, message.info.height, message.info.resolution,
                             message.info.origin.position.x, message.info.origin.position.y,
                             yaw, array("b", message.data).tobytes())

    def observe(self):
        if self.estimate is None or time.monotonic()-self.pose_wall > 4:
            raise LocalizationUnavailable("Waiting for live simulated KF-GINS pose")
        if self.grid is None:
            raise LocalizationUnavailable("Waiting for the real Nav2 local costmap")
        return dict(self.estimate), self.grid

    def update(self, status, message):
        event = {"sim_time": self.get_clock().now().nanoseconds/1e9, "status": status, "message": message}
        self.events.append(event)
        print(json.dumps(event, ensure_ascii=False), flush=True)
        if "停车，规划局部绕行" in message:
            self.snapshot_blockage()
        if status in ("succeeded", "failed", "canceled"):
            self.outcome = status
            self.terminal_at = time.monotonic()

    def snapshot_blockage(self):
        grid = self.grid
        if grid is None:
            return
        footprint = self.footprint
        end = min(len(self.path), int(np.searchsorted(self.arc, self.arc[self.progress]+1.0))+1)
        checks = [{"path_index": i, "pose": self.path[i], **footprint_cells(grid, self.path[i], footprint)}
                  for i in range(self.progress, end)]
        detail = {"sim_time": self.get_clock().now().nanoseconds/1e9, "estimate": self.estimate,
                  "truth": self.truth, "path_index": self.progress, "next_meter_checks": checks}
        serial = len(self.events)
        (self.output/f"blockage_{serial:03d}.json").write_text(json.dumps(detail, ensure_ascii=False, indent=2))
        np.savez_compressed(self.output/f"blockage_{serial:03d}.npz",
                            data=np.frombuffer(grid.data, dtype=np.int8).reshape(grid.height, grid.width),
                            origin=[grid.origin_x, grid.origin_y, grid.origin_yaw], resolution=grid.resolution)

    def publish_path(self, poses):
        # Keep the unmodified full path visible even after local detour requests.
        self.planned.publish(self.runner.message(self.path))

    def publish_markers(self):
        result = MarkerArray()
        for number, index in enumerate(self.anchors, 1):
            p = self.route["manual_poses"][number-1] if self.plan_manual_points else self.path[index]
            item = Marker()
            item.header.frame_id = "map"
            item.ns, item.id = "saved_104", number
            item.type, item.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            item.pose.position.x, item.pose.position.y, item.pose.position.z = p["x"], p["y"], .5
            item.pose.orientation.w = 1.
            item.scale.z = .6
            item.color.r, item.color.g, item.color.b, item.color.a = 1., .6, .05, 1.
            item.text = str(self.route.get("original_anchor_numbers", list(range(1, len(self.anchors)+1)))[number-1])
            result.markers.append(item)
        self.markers.publish(result)

    def plan_preset_points(self):
        if self.planning_verified:
            return True
        if not self.smac_plan.server_is_ready():
            return False
        if self.planning_request is None:
            poses = self.runner.message(self.route["manual_poses"]).poses
            goal = ComputePathThroughPoses.Goal()
            goal.start, goal.goals = poses[0], poses[1:]
            goal.use_start, goal.planner_id = True, "GridBased"
            self.planning_started = time.monotonic()
            self.planning_request = self.smac_plan.send_goal_async(goal)
            self.update("planning", "Smac正在按原104个预设经停点规划；尚未发送运动任务")
            return False
        if self.planning_handle is None:
            if not self.planning_request.done():
                return False
            self.planning_handle = self.planning_request.result()
            if not self.planning_handle.accepted:
                self.update("failed", "Smac拒绝104点规划请求")
                return False
            self.planning_result = self.planning_handle.get_result_async()
        if not self.planning_result.done():
            return False
        response = self.planning_result.result()
        path = []
        for item in response.result.path.poses:
            p, q = item.pose.position, item.pose.orientation
            path.append({"x": p.x, "y": p.y,
                         "yaw": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))})
        self.planning_report = {**compare_route_poses(self.reference_path, path),
            "status": response.status, "wall_time_s": time.monotonic()-self.planning_started,
            "planning_time_s": response.result.planning_time.sec + response.result.planning_time.nanosec/1e9,
            "action": "/fuxi_route/compute_path_through_poses", "requested_manual_points": 104,
            "inputs": self.preparation.get("route_planning")}
        (self.output/"smac_planning.json").write_text(json.dumps(self.planning_report, ensure_ascii=False, indent=2))
        (self.output/"recomputed_route.json").write_text(json.dumps({**self.route, "path": path}, ensure_ascii=False))
        if response.status != GoalStatus.STATUS_SUCCEEDED or not self.planning_report["exactly_equal"]:
            self.update("failed", "Smac新轨迹与真机已验收轨迹不一致，未发送跟踪任务；禁止用旧轨迹冒充新规划")
            return False
        self.path = path
        self.route = {**self.route, "path": path}
        self.planning_verified = True
        self.publish_path(path)
        self.update("planned", "Smac完成104点规划，11285个位姿与真机轨迹逐点完全一致")
        return True

    def tick(self):
        if self.finished:
            return
        wall = time.monotonic()
        if not self.started and wall-self.lifecycle_checked >= .5:
            self.lifecycle_checked = wall
            for name, client in self.lifecycle_clients.items():
                if name in self.lifecycle_active:
                    continue
                future = self.lifecycle_futures.get(name)
                if future is not None:
                    if not future.done():
                        continue
                    response = future.result() if future.exception() is None else None
                    if response and response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                        self.lifecycle_active.add(name)
                        continue
                if client.service_is_ready():
                    self.lifecycle_futures[name] = client.call_async(GetState.Request())
        if self.truth:
            self.gt_progress = forward_index(self.path, self.truth, self.gt_progress, arc=self.arc)
            nearby = abs(self.arc[self.anchors]-self.arc[self.gt_progress]) < 1.5
            xy = (np.array([[p["x"], p["y"]] for p in self.route["manual_poses"]]) if self.plan_manual_points
                  else np.array([[self.path[i]["x"], self.path[i]["y"]] for i in self.anchors]))
            distances = np.linalg.norm(xy-[self.truth["x"], self.truth["y"]], axis=1)
            self.anchor_minima[nearby] = np.minimum(self.anchor_minima[nearby], distances[nearby])
            sim = self.get_clock().now().nanoseconds/1e9
            self.samples.append([sim, self.truth["x"], self.truth["y"], self.truth["z"], self.truth["yaw"],
                                 self.gt_v, self.gt_w, self.gt_progress,
                                 self.estimate["x"] if self.estimate else None,
                                 self.estimate["y"] if self.estimate else None,
                                 self.command_v, self.command_w,
                                 self.gt_roll, self.gt_pitch,
                                 self.estimate["yaw"] if self.estimate else None])
            if self.truth["z"] < -3 or self.max_tilt > math.radians(55):
                self.physical_failure = "Physical vehicle fell or overturned"
                self.runner.stop_event.set()
                if not self.started:
                    self.update("failed", "Physical vehicle fell or overturned before navigation")
        if (not self.started and not self.outcome and self.truth and self.estimate and self.grid
                and not self.physical_failure and len(self.lifecycle_active) == len(self.lifecycle_clients)):
            if (self.runner.follow.server_is_ready() and self.runner.plan.server_is_ready()
                    and (not self.plan_manual_points or self.plan_preset_points())):
                self.started = True
                self.runner.start(self.path, lambda i: setattr(self, "progress", i))
        if wall-self.last_print >= 10:
            self.last_print = wall
            detail = {"started": self.started, "path_index": self.progress, "ground_truth_path_index": self.gt_progress,
                      "sim_time": sim if self.truth else 0., "diagnostic_only": not self.full_run,
                      "anchors_within_075m": int((self.anchor_minima <= .75).sum()),
                      "truth": self.truth, "command": [self.command_v, self.command_w]}
            print(json.dumps(detail, ensure_ascii=False), flush=True)
            (self.output/"live_status.json").write_text(json.dumps(detail, ensure_ascii=False, indent=2))
            if not self.plan_manual_points or self.planning_verified:
                self.publish_path(self.path)
            if self.samples:
                trail = [{"x": p[1], "y": p[2], "yaw": p[4]} for p in self.samples[::10]]
                self.actual.publish(self.runner.message(trail))
        if self.outcome and wall-self.terminal_at > 5 and not self.runner.active:
            self.write_report()
            self.finished = True

    def write_report(self):
        self.preparation = json.loads((self.bundle/"preparation.json").read_text())
        unchanged = all(FilePath(p).is_file() and hashlib.sha256(FilePath(p).read_bytes()).hexdigest() == digest
                        for p, digest in self.preparation["original_source_hashes"].items())
        error = math.hypot(self.truth["x"]-self.path[-1]["x"], self.truth["y"]-self.path[-1]["y"]) if self.truth else None
        passed = bool(self.full_run and not self.physical_failure and len(self.anchors) == 104 and self.outcome == "succeeded" and (self.anchor_minima <= .75).all()
                      and error <= .5 and abs(self.gt_v) < .03 and abs(self.gt_w) < .05 and unchanged
                      and (not self.plan_manual_points or self.planning_verified))
        report = {"passed": passed, "runner_outcome": self.outcome, "events": self.events,
                  "diagnostic_only": not self.full_run,
                  "actual_samples": len(self.samples), "path_samples": len(self.path), "anchor_count": len(self.anchors),
                  "anchor_min_distance_m": [None if not math.isfinite(v) else float(v) for v in self.anchor_minima],
                  "final_position_error_m": error, "final_actual_velocity": [self.gt_v, self.gt_w],
                  "max_roll_pitch_deg": math.degrees(self.max_tilt), "protected_sources_unchanged": unchanged,
                  "physical_failure": self.physical_failure,
                  "smac_planning": self.planning_report,
                  "ground_support": self.preparation.get("ground_support"),
                  "physics_model": self.physics_model or self.preparation.get("physics_model"),
                  "contact_proxy": self.preparation.get("contact_proxy"),
                  "controller_plugin": self.preparation.get("controller_plugin"),
                  "navigation_footprint": self.footprint,
                  "simulation_clearance_m": self.preparation.get("simulation_clearance_m"),
                  "scope": self.preparation["scope"]}
        (self.output/"result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
        np.savetxt(self.output/"actual_trajectory.csv", np.asarray(self.samples, dtype=float), delimiter=",",
                   header="sim_time,x,y,z,yaw,v,w,path_index,kfgins_x,kfgins_y,command_v,command_w,roll,pitch,kfgins_yaw", comments="")
        self.actual.publish(self.runner.message([{"x": p[1], "y": p[2], "yaw": p[4]} for p in self.samples]))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=FilePath)
    parser.add_argument("--route-file", type=FilePath)
    parser.add_argument("--results-dir", type=FilePath)
    parser.add_argument("--keep-result", action="store_true", help="Keep the completed paths visible without sending another task")
    parser.add_argument("--plan-manual-points", action="store_true", help="Recompute the original 104 points with real Smac and require an exact reference match before tracking")
    args, ros = parser.parse_known_args()
    if os.environ.get("ROS_DOMAIN_ID") != "93" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Physical simulation validation requires isolated ROS_DOMAIN_ID=93, ROS_LOCALHOST_ONLY=1")
    rclpy.init(args=ros)
    node = Validation(args.bundle.resolve(), args.route_file, args.results_dir, args.plan_manual_points)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        while rclpy.ok() and (not node.finished or args.keep_result):
            executor.spin_once(timeout_sec=.1)
    finally:
        node.runner.stop_event.set()
        deadline = time.monotonic()+10
        while node.runner.active and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
