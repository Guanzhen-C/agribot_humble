"""Bounded integration probe for a running Gazebo demonstration."""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

rclpy.init()
node = Node("check_manipulator_motion")
samples = []
frames = set()


def on_state(msg):
    state = dict(zip(msg.name, msg.position))
    names = ["manipulator_joint" + str(i) for i in range(1, 7)] + [
        "manipulator_finger_joint1", "manipulator_finger_joint2"]
    if all(name in state for name in names):
        samples.append([state[name] for name in names])


node.create_subscription(JointState, "/joint_states", on_state, 10)
node.create_subscription(TFMessage, "/tf", lambda msg: frames.update(t.child_frame_id for t in msg.transforms), 10)
deadline = time.monotonic() + 28
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
ranges = [max(row[i] for row in samples) - min(row[i] for row in samples) for i in range(8)] if samples else []
assert samples and len(samples) > 100, "missing feedback"
assert all(value > 0.05 for value in ranges[:6]), ranges
assert all(value > 0.005 for value in ranges[6:]), ranges
assert all("manipulator_link" + str(i) in frames for i in range(1, 7)), frames
client = node.create_client(SetBool, "/manipulator/demo_enabled")
assert client.wait_for_service(timeout_sec=3)
future = client.call_async(SetBool.Request(data=False))
rclpy.spin_until_future_complete(node, future, timeout_sec=3)
assert future.result() and future.result().success
deadline = time.monotonic() + 6
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
assert max(abs(value) for value in samples[-1]) < 0.03, samples[-1]
print(json.dumps({"samples": len(samples), "joint_ranges_rad": ranges,
                  "final_home_rad": samples[-1], "tf": sorted(frames)}))
node.destroy_node()
rclpy.shutdown()
