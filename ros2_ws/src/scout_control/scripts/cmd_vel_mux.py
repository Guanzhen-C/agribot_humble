#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool


@dataclass
class SourceState:
    topic: str
    timeout: float
    priority: int
    latest_msg: Optional[Twist] = None
    latest_time_ns: Optional[int] = None


@dataclass
class LockState:
    topic: str
    priority: int
    locked: bool = False


def zero_twist() -> Twist:
    return Twist()


class CmdVelMux(Node):
    def __init__(self) -> None:
        super().__init__("twist_mux")
        config_file = self.declare_parameter("config_file", "").value
        self.output_topic = self.declare_parameter("output_topic", "/cmd_vel").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)

        if not config_file:
            raise RuntimeError("Parameter 'config_file' is required")

        config = self._load_config(config_file)
        self.sources: Dict[str, SourceState] = {}
        self.locks: Dict[str, LockState] = {}
        self.last_output = zero_twist()

        for entry in config.get("topics", []):
            topic_name = str(entry["topic"])
            state = SourceState(
                topic=topic_name,
                timeout=float(entry.get("timeout", 0.5)),
                priority=int(entry.get("priority", 0)),
            )
            self.sources[topic_name] = state
            self.create_subscription(Twist, topic_name, self._make_source_cb(topic_name), 20)

        for entry in config.get("locks", []):
            topic_name = str(entry["topic"])
            self.locks[topic_name] = LockState(
                topic=topic_name,
                priority=int(entry.get("priority", 255)),
            )
            self.create_subscription(Bool, topic_name, self._make_lock_cb(topic_name), 10)

        self.publisher = self.create_publisher(Twist, self.output_topic, 20)
        self.create_timer(1.0 / max(self.publish_rate, 1e-3), self._publish_selected)

        self.get_logger().info(
            f"cmd_vel mux ready: {len(self.sources)} sources, {len(self.locks)} locks, "
            f"output={self.output_topic}"
        )

    def _load_config(self, config_file: str):
        with Path(config_file).open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        if "twist_mux" in data:
            data = data["twist_mux"].get("ros__parameters", {})

        return data

    def _make_source_cb(self, topic_name: str):
        def callback(msg: Twist) -> None:
            state = self.sources[topic_name]
            state.latest_msg = msg
            state.latest_time_ns = self.get_clock().now().nanoseconds

        return callback

    def _make_lock_cb(self, topic_name: str):
        def callback(msg: Bool) -> None:
            self.locks[topic_name].locked = bool(msg.data)

        return callback

    def _active_lock_priority(self) -> Optional[int]:
        active_priorities = [lock.priority for lock in self.locks.values() if lock.locked]
        return max(active_priorities) if active_priorities else None

    def _active_source(self) -> Optional[SourceState]:
        now_ns = self.get_clock().now().nanoseconds
        active_lock_priority = self._active_lock_priority()
        candidates = []

        for state in self.sources.values():
            if state.latest_msg is None or state.latest_time_ns is None:
                continue
            age = Duration(nanoseconds=now_ns - state.latest_time_ns).nanoseconds / 1e9
            if age > state.timeout:
                continue
            if active_lock_priority is not None and state.priority < active_lock_priority:
                continue
            candidates.append(state)

        if not candidates:
            return None

        return max(candidates, key=lambda item: item.priority)

    def _publish_selected(self) -> None:
        active = self._active_source()
        output = active.latest_msg if active is not None else zero_twist()
        self.publisher.publish(output)
        self.last_output = output


def main() -> None:
    rclpy.init()
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
