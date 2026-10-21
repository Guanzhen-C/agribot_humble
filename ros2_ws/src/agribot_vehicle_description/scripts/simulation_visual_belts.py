#!/usr/bin/env python3
"""Show CAD tread movement in RViz using measured simulated wheel rotation."""

import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from agribot_vehicle_config_tools.visual_belts import BeltPath, belts
from agribot_vehicle_config_tools.visual_rotors import uri


class VisualBelts(Node):
    def __init__(self):
        super().__init__("simulation_visual_belts")
        config = json.loads(Path(self.declare_parameter("vehicle_config", "").value).read_text(encoding="utf-8"))
        self.items = [(item, BeltPath(item["points"])) for item in belts(config)]
        self.markers = {}
        for item, _ in self.items:
            for part in item["segments"]:
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.ns = "belt_" + item["id"]
                marker.id = part["index"]
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD
                marker.mesh_resource = uri(config, part["meshFile"])
                marker.mesh_use_embedded_materials = True
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                marker.color.r = marker.color.g = marker.color.b = 1.0
                marker.color.a = 1.0
                marker.frame_locked = True
                self.markers[(item["id"], part["index"])] = marker
        self.publisher = self.create_publisher(MarkerArray, "/simulation/track_visuals", 2)
        self.create_subscription(JointState, "/joint_states", self.feedback, 2)

    def feedback(self, message):
        positions = dict(zip(message.name, message.position))
        output = MarkerArray()
        for item, path in self.items:
            angle = positions.get(item["sourceWheel"] + "_wheel_joint")
            if angle is None:
                continue
            for part in item["segments"]:
                x, z, pitch = path.sample(part["distance"] + angle*item["sourceRadius"])
                marker = self.markers[(item["id"], part["index"])]
                marker.header.stamp = message.header.stamp
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, item["centerY"], z
                marker.pose.orientation.y, marker.pose.orientation.w = math.sin(pitch/2), math.cos(pitch/2)
                output.markers.append(marker)
        if output.markers:
            self.publisher.publish(output)


def main():
    rclpy.init()
    node = VisualBelts()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
