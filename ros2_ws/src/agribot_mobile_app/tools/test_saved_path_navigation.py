#!/usr/bin/env python3
"""Isolated Nav2/MPPI kinematic test. No CAN, sensor drivers or vehicle commands."""

from array import array
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.saved_path_runner import SavedPathRunner
from agribot_mobile_app.route_preflight import footprint_cells


class Simulation(Node):
    def __init__(self):
        super().__init__("saved_path_kinematic_simulation")
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.grid = None
        self.local_grid = None
        self.obstacle = False
        self.stage = "waiting"
        self.updates, self.history = [], []
        self.finished = False
        self.ready = True
        self.scenario = os.environ.get("SAVED_PATH_TEST_SCENARIO", "obstacle")
        if self.scenario == "entry":
            self.x = -1.5
        self.start_time = time.monotonic()
        self.map = self.create_publisher(OccupancyGrid, "/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom = self.create_publisher(Odometry, "/fastlivo_rtk/odometry", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/saved_test/cmd_vel", self.command, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.costmap, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.local_costmap, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.runner = SavedPathRunner(self, self.observe, self.update, lambda p: None,
                                     [[.7,.53],[.7,-.53],[-.7,-.53],[-.7,.53]])
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_map)
        self.publish_map()

    def command(self, message):
        self.v, self.w = message.linear.x, message.angular.z

    def costmap(self, message):
        self.grid = GridData(message.info.width, message.info.height, message.info.resolution,
                             message.info.origin.position.x, message.info.origin.position.y, 0,
                             array("b", message.data).tobytes())

    def observe(self):
        if not self.ready:
            raise RuntimeError("simulation: localization lost")
        if self.local_grid is None:
            raise RuntimeError("simulation: no costmap")
        return {"x": self.x, "y": self.y, "yaw": self.yaw}, self.local_grid

    def local_costmap(self, message):
        self.local_grid = GridData(message.info.width, message.info.height, message.info.resolution,
                                   message.info.origin.position.x, message.info.origin.position.y, 0,
                                   array("b", message.data).tobytes())

    def update(self, status, message):
        self.updates.append({"status":status,"message":message})
        print(status, message, flush=True)
        if status in ("succeeded", "failed", "canceled"):
            self.finished = True

    def publish_map(self):
        pixels = np.zeros((240, 400), dtype=np.int8)
        pixels[:2,:] = pixels[-2:,:] = 100
        pixels[:,:2] = pixels[:,-2:] = 100
        if self.obstacle:
            pixels[112:128,132:148] = 100
            if self.scenario == "wall":
                pixels[:,132:148] = 100
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = .05
        message.info.width, message.info.height = 400, 240
        message.info.origin.position.x, message.info.origin.position.y = -3., -6.
        message.info.origin.orientation.w = 1.
        message.data = array("b", pixels.tobytes())
        self.map.publish(message)

    def advance(self):
        self.x += self.v * math.cos(self.yaw) * .05
        self.y += self.v * math.sin(self.yaw) * .05
        self.yaw += self.w * .05

    def measured_velocity(self):
        return self.v, self.w

    def validation_details(self):
        return {}

    def tick(self):
        self.advance()
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "base_link"
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.measured_velocity()
        self.odom.publish(odom)
        self.history.append([self.x,self.y,self.yaw,self.v,self.w])
        if self.stage == "waiting" and self.grid is not None and self.local_grid is not None and time.monotonic()-self.start_time > 8:
            self.stage = "following"
            end = 3.0 if self.scenario == "entry" else 10.0
            self.runner.start([{"x":float(x),"y":0.,"yaw":0.} for x in np.linspace(0,end,int(end*20)+1)])
        if self.stage == "following" and self.scenario in ("cancel", "stale") and self.x > .15:
            self.stage = "stopping"
            if self.scenario == "cancel":
                self.runner.stop_event.set()
            else:
                self.ready = False
        if self.stage == "following" and self.x > .6 and not self.obstacle and self.scenario in ("obstacle", "wall"):
            self.obstacle = True
            self.publish_map()
            print("Inserted obstacle at x=4,y=0", flush=True)


def main():
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) != 81:
        raise RuntimeError("Test requires isolated ROS_DOMAIN_ID=81")
    rclpy.init()
    node = Simulation()
    deadline = time.monotonic()+float(os.environ.get("SAVED_PATH_TEST_TIMEOUT", "120"))
    try:
        while rclpy.ok() and not node.finished and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        # Continue callbacks while cancellation is acknowledged by Nav2.
        node.runner.stop_event.set()
        end = time.monotonic()+10
        while node.runner.active and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.02)
        end = time.monotonic()+.5
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
        report = {"updates":node.updates,"final":[node.x,node.y],
                  "max_lateral_offset_m":max(abs(p[1]) for p in node.history),
                  "success":any(p["status"]=="succeeded" for p in node.updates),
                  "obstacle_inserted":node.obstacle,
                  "scenario":node.scenario,"final_speed":node.measured_velocity()[0],
                  "scope":os.environ.get("SAVED_PATH_TEST_SCOPE", "real Nav2 planner/controller, ideal kinematic differential simulation; not Gazebo, localization or vehicle acceptance")}
        report.update(node.validation_details())
        footprint=[[.7,.53],[.7,-.53],[-.7,-.53],[-.7,.53]]
        if node.grid:
            report["occupied_footprint_samples"] = sum(
                footprint_cells(node.grid, {"x":p[0],"y":p[1],"yaw":p[2]}, footprint)["occupied_cells"]>0
                for p in node.history)
        prefix=os.environ.get("SAVED_PATH_TEST_PREFIX", "saved_path_sim")
        Path(f"/tmp/{prefix}_{node.scenario}_result.json").write_text(json.dumps(report,indent=2))
        Path(f"/tmp/{prefix}_{node.scenario}_history.json").write_text(json.dumps(node.history))
        print(json.dumps(report,indent=2),flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
