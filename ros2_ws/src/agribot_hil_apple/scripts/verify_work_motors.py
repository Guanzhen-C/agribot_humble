"""Explicit simulation fixture, not evidence of physical CAN/HPM operation."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rclpy.init()
    node = Node('hil_work_motor_fixture')
    publisher = node.create_publisher(Float64MultiArray, '/hil/work/target_rpm', 1)
    events = []
    start = time.monotonic()

    def receive(msg):
        events.append(dict(wall=time.monotonic()-start, names=list(msg.name), velocity=list(msg.velocity)))

    node.create_subscription(JointState, '/hil/work/joint_states', receive, 10)
    try:
        while time.monotonic() - start < 10:
            elapsed = time.monotonic() - start
            if elapsed < 1:
                publisher.publish(Float64MultiArray(data=[0., 0.]))
            elif elapsed < 6:
                publisher.publish(Float64MultiArray(data=[900., 800.]))
            # No final zero: motor plugin must stop on the steady-time lease.
            rclpy.spin_once(node, timeout_sec=.04)
        moving = [event for event in events if 4 < event['wall'] < 6]
        stopped = [event for event in events if event['wall'] > 8]
        checks = dict(actual_physics_motion=bool(moving) and all(
            all(math.isfinite(v) and v > 50 for v in event['velocity']) for event in moving),
            stop_on_missing_command=bool(stopped) and all(
                all(abs(v) < .5 for v in event['velocity']) for event in stopped),
            complete_joint_feedback=bool(events) and all(len(event['velocity']) == 2 for event in events))
        report = dict(ok=all(checks.values()), checks=checks, events=events,
                      scope='Gazebo-only shaft-physics fixture; no physical CAN/HPM proof')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(ok=report['ok'], checks=checks, samples=len(events))))
        return 0 if report['ok'] else 1
    finally:
        publisher.publish(Float64MultiArray(data=[0., 0.]))
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
