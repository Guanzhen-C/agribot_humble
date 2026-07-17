#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_frame_relay")
        self.declare_parameter("use_sim_time", False)
        self.input_topic = self.declare_parameter("input_topic", "/points").value
        self.output_topic = self.declare_parameter("output_topic", "/kiss/points").value
        self.output_frame_id = self.declare_parameter("output_frame_id", "kiss_lidar").value

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.handle_pointcloud,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Relaying {self.input_topic} to {self.output_topic} with frame_id={self.output_frame_id}"
        )

    def handle_pointcloud(self, msg: PointCloud2) -> None:
        # Publish the same cloud data under a duplicate frame so KISS-ICP can own
        # an independent TF chain without reparenting the robot's existing frames.
        msg.header.frame_id = self.output_frame_id
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
