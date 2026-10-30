"""Exercise the actual executor without HTTP, hardware launch or motion goals."""

import threading
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scout_msgs.msg import ScoutStatus

from agribot_mobile_app.gateway_node import MobileGateway


def test_slow_global_display_cannot_block_pose_chassis_or_action_callbacks():
    rclpy.init()
    gateway = MobileGateway()
    publisher = Node("gateway_isolation_fixture")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(gateway)
    executor.add_node(publisher)
    entered, release, action_called = [threading.Event() for _ in range(3)]
    thread = threading.Thread(target=executor.spin, daemon=True)
    timer = None
    try:
        poses = publisher.create_publisher(Odometry, gateway.pose_topic, 1)
        chassis = publisher.create_publisher(ScoutStatus, "/scout_status", 1)

        def hold_global_group():
            timer.cancel()
            entered.set()
            release.wait(3.0)

        timer = gateway.create_timer(0.05, hold_global_group,
                                     callback_group=gateway._grid_callbacks["global_costmap"])
        thread.start()
        assert entered.wait(2.0)
        action_timer = gateway.create_timer(0.05, action_called.set,
                                            callback_group=gateway._navigation_callbacks)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            pose = Odometry()
            pose.header.frame_id = "map"
            pose.header.stamp = gateway.get_clock().now().to_msg()
            pose.pose.pose.orientation.w = 1.0
            poses.publish(pose)
            chassis.publish(ScoutStatus())
            with gateway._lock:
                pose_received = gateway._navigation_health.snapshot()["pose"]["ready"]
            status = gateway._chassis_feedback.snapshot("ackermann")["chassis_feedback"]
            if pose_received and status["age_sec"] is not None and action_called.is_set():
                break
            time.sleep(0.02)
        assert not release.is_set()
        assert pose_received
        assert status["age_sec"] is not None and status["age_sec"] < 0.5
        assert action_called.is_set()
        action_timer.cancel()
        assert gateway._pose_callbacks is not gateway._chassis_callbacks
        assert gateway._grid_callbacks["local_costmap"] is not gateway._grid_callbacks["global_costmap"]
    finally:
        release.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        gateway.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()
