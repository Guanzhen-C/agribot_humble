#!/usr/bin/env python3
"""Benchmark saved waypoints using only Nav2 ComputePathThroughPoses, never motion."""

import argparse
import csv
import json
import math
import os
import platform
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from tf2_ros import TransformBroadcaster


def wait(node, predicate, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    raise TimeoutError("Planning benchmark observation timed out")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("preset", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--route-id")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--first", type=int, default=1)
    parser.add_argument("--last", type=int)
    args = parser.parse_args()
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0:
        parser.error("Use an isolated nonzero ROS_DOMAIN_ID for this planning-only fixture")
    document = json.loads(args.preset.read_text(encoding="utf-8"))
    route = next(r for r in document["routes"] if not args.route_id or r["id"] == args.route_id)
    poses = route["poses"][args.first - 1:args.last]
    if len(poses) < 2 or args.repeat < 1:
        parser.error("At least two waypoints and one repetition are required")
    rclpy.init()
    node = Node("capture_route_planning_benchmark")
    broadcaster = TransformBroadcaster(node)

    def transform():
        message = TransformStamped()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.child_frame_id = "base_link"
        message.transform.translation.x = poses[0]["x"]
        message.transform.translation.y = poses[0]["y"]
        message.transform.rotation.z = math.sin(poses[0]["yaw"] / 2)
        message.transform.rotation.w = math.cos(poses[0]["yaw"] / 2)
        broadcaster.sendTransform(message)

    timer = node.create_timer(0.1, transform)
    transform()
    client = ActionClient(node, ComputePathThroughPoses, "compute_path_through_poses")
    lifecycle = node.create_client(GetState, "/planner_server/get_state")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {"map_id": document["map_id"], "route_id": route["id"],
              "ros_domain_id": os.environ["ROS_DOMAIN_ID"], "host_architecture": platform.machine(),
              "motion_action_sent": False, "runs": []}

    def stamped(pose):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.position.x = pose["x"]
        message.pose.position.y = pose["y"]
        message.pose.orientation.z = math.sin(pose["yaw"] / 2)
        message.pose.orientation.w = math.cos(pose["yaw"] / 2)
        return message

    try:
        wait(node, lifecycle.service_is_ready, 30)
        activation_deadline = time.monotonic() + 60.0
        while time.monotonic() < activation_deadline:
            state = lifecycle.call_async(GetState.Request())
            wait(node, state.done, 5)
            if state.result().current_state.id == 3:
                break
            next_poll = time.monotonic() + 0.25
            wait(node, lambda: time.monotonic() >= next_poll, 1.0)
        else:
            raise RuntimeError("Planner did not activate")
        wait(node, client.server_is_ready, 15)
        parameters = node.create_client(GetParameters, "/planner_server/get_parameters")
        wait(node, parameters.service_is_ready, 5)
        parameter_request = GetParameters.Request()
        parameter_request.names = ["GridBased." + name for name in ["max_planning_time", "analytic_expansion_max_length", "analytic_expansion_ratio", "rotation_penalty", "lookup_table_size"]]
        parameter_future = parameters.call_async(parameter_request)
        wait(node, parameter_future.done, 5)
        report["planner_parameters"] = {name: value.double_value for name, value in zip(parameter_request.names, parameter_future.result().values)}
        for attempt in range(args.repeat):
            goal = ComputePathThroughPoses.Goal()
            goal.start = stamped(poses[0])
            goal.use_start = True
            goal.goals = [stamped(pose) for pose in poses[1:]]
            goal.planner_id = "GridBased"
            started = time.monotonic()
            request = client.send_goal_async(goal)
            wait(node, request.done, 10)
            handle = request.result()
            if not handle.accepted:
                raise RuntimeError("Planning request rejected")
            result_future = handle.get_result_async()
            try:
                wait(node, result_future.done, 180)
            except TimeoutError:
                cancel = handle.cancel_goal_async()
                wait(node, cancel.done, 10)
                raise
            response = result_future.result()
            result = response.result
            points = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
            orientations = [math.atan2(2 * (p.pose.orientation.w * p.pose.orientation.z + p.pose.orientation.x * p.pose.orientation.y), 1 - 2 * (p.pose.orientation.y ** 2 + p.pose.orientation.z ** 2)) for p in result.path.poses]
            errors = [min(math.dist((pose["x"], pose["y"]), point) for point in points) for pose in poses] if points else []
            record = {"attempt": attempt + 1, "status": response.status,
                      "success": response.status == GoalStatus.STATUS_SUCCEEDED,
                      "first_waypoint": args.first, "last_waypoint": args.last or len(route["poses"]),
                      "requested_waypoints_including_start": len(poses),
                      "action_wall_time_s": time.monotonic() - started,
                      "nav2_planning_time_s": result.planning_time.sec + result.planning_time.nanosec * 1e-9,
                      "path_pose_count": len(points),
                      "waypoint_max_miss_m": max(errors) if errors else None,
                      "waypoint_miss_m": errors,
                      "path_length_m": sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))}
            report["runs"].append(record)
            print(json.dumps(record), flush=True)
            if points:
                with args.output.with_name(args.output.stem + f"_run{attempt + 1}_path.csv").open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["x", "y", "yaw"])
                    writer.writerows([(*point, yaw) for point, yaw in zip(points, orientations)])
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    except Exception as error:
        report["error"] = str(error)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        raise
    finally:
        node.destroy_timer(timer)
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
