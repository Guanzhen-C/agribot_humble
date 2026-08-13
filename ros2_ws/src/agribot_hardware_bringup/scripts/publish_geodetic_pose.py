#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.node import Node


def enu_yaw_from_heading_degrees(heading_degrees: float) -> float:
    return math.atan2(
        math.sin(math.radians(90.0 - heading_degrees)),
        math.cos(math.radians(90.0 - heading_degrees)),
    )


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description=(
            "发布左侧主RTK天线的WGS84经纬度和车头航向，触发地图初始重定位"
        )
    )
    parser.add_argument("latitude", type=float, help="WGS84纬度，单位度")
    parser.add_argument("longitude", type=float, help="WGS84经度，单位度")
    parser.add_argument(
        "heading_degrees",
        type=float,
        help="车头航向，真北为0度、顺时针为正，与/rtk/heading_deg一致",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=float("nan"),
        help="主天线海拔；省略时仅进行水平初始化",
    )
    parser.add_argument(
        "--topic",
        default="/georeference_test/rtk_input",
        help="GeoPoseStamped输入话题",
    )
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=5.0,
        help="等待桥接节点订阅的最长时间",
    )
    return parser.parse_args(argv)


class GeodeticPosePublisher(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("publish_geodetic_pose")
        self.publisher = self.create_publisher(GeoPoseStamped, topic, 10)


def main(args=None) -> None:
    parsed = parse_arguments(sys.argv[1:] if args is None else args)
    if (
        not math.isfinite(parsed.latitude)
        or not math.isfinite(parsed.longitude)
        or not math.isfinite(parsed.heading_degrees)
        or abs(parsed.latitude) > 90.0
        or abs(parsed.longitude) > 180.0
        or parsed.wait_seconds <= 0.0
    ):
        raise SystemExit("经纬度、航向或等待时间无效")

    rclpy.init()
    node = GeodeticPosePublisher(parsed.topic)
    deadline = time.monotonic() + parsed.wait_seconds
    try:
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise SystemExit(
                    f"没有找到{parsed.topic}的订阅者，请先启动地理配准测试"
                )
            rclpy.spin_once(node, timeout_sec=0.05)

        yaw = enu_yaw_from_heading_degrees(parsed.heading_degrees)
        message = GeoPoseStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "wgs84"
        message.pose.position.latitude = parsed.latitude
        message.pose.position.longitude = parsed.longitude
        message.pose.position.altitude = parsed.altitude
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        node.publisher.publish(message)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        print(
            "已发布主天线位置："
            f"lat={parsed.latitude:.9f}, lon={parsed.longitude:.9f}, "
            f"heading={parsed.heading_degrees % 360.0:.3f} deg"
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
