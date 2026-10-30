"""Exercise the actual CAN executable against a pseudo-terminal, never hardware."""

import fcntl
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import termios
import time
import tty

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from scout_msgs.msg import ScoutStatus


PACKAGE_ROOT = Path(__file__).parents[1]
DEFAULT_BINARY = (
    PACKAGE_ROOT.parents[1] / "install/agribot_hardware_bringup/lib"
    / "agribot_hardware_bringup/differential_chassis_can_node"
)


def packet(can_id, payload, counter):
    payload = bytearray(payload)
    payload[6] = counter & 15
    checksum = 0
    for value in payload[:7]:
        checksum ^= value
    payload[7] = checksum
    return struct.pack(">BBBI", 0x5A, 8, 0, can_id) + payload + b"\xa5"


def run_case(monkeypatch, read_only, initially_busy=False):
    binary = Path(os.environ.get("CHASSIS_CAN_NODE", DEFAULT_BINARY))
    if not binary.is_file():
        pytest.skip("build differential_chassis_can_node first")
    domain = 142 + os.getpid() % 20
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node(f"can_observer_test_{os.getpid()}", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    messages = []
    diagnostics = []
    subscriptions = [
        node.create_subscription(ScoutStatus, "/scout_status", messages.append, 20),
        node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics.append, 20),
    ]
    command_topic = f"/can_test/case_{os.getpid()}/cmd_vel"
    publisher = node.create_publisher(Twist, command_topic, 10)
    command_message = Twist()
    command_message.linear.x = 0.2
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if initially_busy:
        fcntl.ioctl(slave, termios.TIOCEXCL)
    parameters = {
        "can_transport": "zqwl_cdc", "zqwl_port": path,
        "zqwl_bitrate": "250000", "read_only": str(read_only).lower(),
        "recover_zqwl_startup": "false", "send_rate_hz": "5.0",
        "command_topic": command_topic, "require_localization_ready": "false",
    }
    command = [str(binary), "--ros-args"]
    for name, value in parameters.items():
        command.extend(["-p", f"{name}:={value}"])
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, start_new_session=True,
    )
    wire = bytearray()
    blocked_seen = False
    subscription_count = None
    log = ""

    def drain():
        while True:
            try:
                chunk = os.read(master, 65536)
            except BlockingIOError:
                return
            if not chunk:
                return
            wire.extend(chunk)

    try:
        start = time.monotonic()
        next_frame = start + (1.6 if initially_busy else 0.5)
        counter = 0
        while time.monotonic() - start < 5.0:
            assert process.poll() is None, "CAN node exited unexpectedly"
            elapsed = time.monotonic() - start
            if initially_busy and elapsed >= 1.4:
                fcntl.ioctl(slave, termios.TIOCNXCL)
                initially_busy = False
                blocked_seen = not messages
            if time.monotonic() >= next_frame:
                chassis = bytearray(8)
                chassis[0] = 1
                struct.pack_into("<H", chassis, 2, 754)
                motor = bytearray(8)
                struct.pack_into("<h", motor, 1, 120)
                motor[3] = 75
                motor[5] = 60
                os.write(master, b"".join([
                    packet(0x532, chassis, counter),
                    packet(0x533, motor, counter),
                    packet(0x534, motor, counter),
                ]))
                counter += 1
                next_frame = time.monotonic() + 0.10
            publisher.publish(command_message)
            executor.spin_once(timeout_sec=0.02)
            drain()
        subscription_count = publisher.get_subscription_count()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            log = process.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            log = process.communicate(timeout=3)[0]
        drain()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        os.close(master)
        os.close(slave)
    assert subscriptions
    assert process.returncode == 0, log
    assert messages, log
    assert messages[-1].battery_voltage == pytest.approx(75.4)
    assert messages[-1].motor_states[ScoutStatus.MOTOR_ID_FRONT_LEFT].rpm == 120
    values = {
        v.key: v.value for array in diagnostics for status in array.status
        if status.name == "agribot/chassis_can/differential" for v in status.values
    }
    assert values["read_only"] == str(read_only).lower()
    assert values["feedback_fresh"] == "true"
    return bytes(wire), subscription_count, blocked_seen, log


def test_receive_only_never_transmits_can_frames_even_on_shutdown(monkeypatch):
    wire, subscriptions, _, _ = run_case(monkeypatch, True)
    assert subscriptions == 0
    assert b"\x5a\x08\x00" not in wire


def test_receive_only_waits_for_exclusive_gui_owner_then_recovers(monkeypatch):
    wire, subscriptions, blocked_seen, log = run_case(monkeypatch, True, True)
    assert blocked_seen
    assert "close the desktop CAN GUI" in log
    assert subscriptions == 0
    assert b"\x5a\x08\x00" not in wire


def test_legacy_control_mode_still_sends_commands_on_simulated_transport(monkeypatch):
    wire, subscriptions, _, _ = run_case(monkeypatch, False)
    assert subscriptions == 1
    assert b"\x5a\x08\x00\x00\x00\x05\x14" in wire
