"""Finite real-CAN shredder HIL test; records actual rotor and reverse feedback."""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from agribot_hil_apple.isobus_can_mux import CAN_FRAME
from agribot_hil_apple.tracked_work_wire import decode_drive, decode_work, validate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rclpy.init()
    node = Node('hil_shredder_acceptance')
    drive = node.create_publisher(Twist, '/hil/drive/command', 1)
    work = node.create_publisher(String, '/hil/work/command', 1)
    start = time.monotonic()
    latest, records = {}, []

    def receive(kind, msg):
        now = time.monotonic()-start
        if kind in ('bridge', 'isobus'):
            value = json.loads(msg.data)
        elif kind == 'motors':
            value = dict(names=list(msg.name), position=list(msg.position), velocity=list(msg.velocity))
        else:
            value = dict(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                         speed=msg.twist.twist.linear.x)
        latest[kind] = (now, value)
        records.append(dict(wall=now, kind=kind, value=value))

    node.create_subscription(String, '/hil/tracked_can/status', lambda msg: receive('bridge', msg), 10)
    node.create_subscription(String, '/hil/isobus/events', lambda msg: receive('isobus', msg), 200)
    node.create_subscription(JointState, '/hil/work/joint_states', lambda msg: receive('motors', msg), 20)
    node.create_subscription(Odometry, '/hil/ground_truth', lambda msg: receive('truth', msg), 20)
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
    bus.bind(('can0',))
    phase_at = None
    gateway_at = -math.inf
    checks, fault = {}, ''

    def command(level):
        work.publish(String(data=json.dumps(dict(fan_gear=0, pump_gear=0, shredder_level=level))))

    try:
        while time.monotonic()-start < 50:
            now = time.monotonic()-start
            at, event = latest.get('isobus', (-math.inf, {}))
            if event.get('kind') == 'gateway':
                raw = bytes.fromhex(event['data'])
                if raw[0] == 1 and raw[1] == 1 and raw[2] & 4:
                    gateway_at = at
            status_at, status = latest.get('bridge', (-math.inf, {}))
            healthy = (now-status_at < 1.5 and not status.get('fault', 'missing') and
                       status.get('implement_kind') == 'shredder' and status.get('motion_authorized') and
                       status.get('work_authorized') and status.get('truth_fresh') and
                       status.get('motors_fresh') and now-gateway_at < 1.)
            if phase_at is None:
                drive.publish(Twist()); command(0)
                if now > 3 and healthy:
                    phase_at = now
                    records.append(dict(wall=now, kind='preflight_ready', value=status))
                elif now > 22:
                    raise RuntimeError('Real shredder gateway / actual feedback not ready')
            else:
                elapsed = now-phase_at
                if not healthy:
                    raise RuntimeError('Physical HIL health lost')
                target = Twist()
                if elapsed < 6:
                    target.linear.x = .15; command(3500)
                elif elapsed < 8:
                    command(0)
                elif elapsed < 12:
                    target.linear.x = .10; command(2500)
                elif elapsed < 17:
                    target.linear.x = .10  # Intentionally stop only implement input.
                elif elapsed < 21:
                    command(0)
                else:
                    break
                drive.publish(target)
            for _ in range(256):
                try:
                    raw = bus.recv(CAN_FRAME.size)
                except BlockingIOError:
                    break
                ident, length, data = CAN_FRAME.unpack(raw)
                if ident & 0xe0000000 or length != 8:
                    raise RuntimeError('Unexpected native CAN frame')
                validate(data)
                records.append(dict(wall=now, kind='native_can', value=dict(id=ident, data=data.hex())))
            rclpy.spin_once(node, timeout_sec=.025)
        if phase_at is None:
            raise RuntimeError('No accepted test phase')

        def selected(kind, begin, end):
            return [r['value'] for r in records if r['kind'] == kind and begin < r['wall']-phase_at < end]

        native = selected('native_can', 3, 5.8)
        expired = selected('native_can', 15, 16.8)
        motors = selected('motors', 3, 5.8)
        stopped = selected('motors', 15, 16.8)
        truth = selected('truth', 0, 21)
        final = selected('truth', 19, 21)
        frames = lambda source, ident: [bytes.fromhex(v['data']) for v in source if v['id'] == ident]
        drive_values = [decode_drive(data) for data in frames(native, 0x514)]
        work_values = [decode_work(data) for data in frames(native, 0x580)]
        expired_work = [decode_work(data) for data in frames(expired, 0x580)]
        active_drive = [decode_drive(data) for data in frames(expired, 0x514)]
        feedback = {v['data'] for v in selected('native_can', 2, 6) if v['id'] == 0x5b4
                    and struct.unpack_from('<h', bytes.fromhex(v['data']), 1)[0] > 500}
        returned = {v['data'] for v in selected('isobus', 2, 7)
                    if v.get('kind') == 'feedback' and v.get('id') == 0x5b4}
        checks = dict(
            real_gateway_channel_three_ready=gateway_at > 0,
            native_shredder_percent_command=bool(work_values) and all(v == (0, 0, 3500) for v in work_values),
            actual_cutter_physics=bool(motors) and all(m['names'] == ['work_shredder_joint'] and
                100 < m['velocity'][0] < 140 for m in motors),
            actual_rpm_reverse_isobus_feedback=bool(feedback & returned),
            simultaneous_native_drive=bool(drive_values) and all(v[0] > .1 for v in drive_values),
            work_command_timeout_zero=bool(expired_work) and all(v == (0, 0, 0) for v in expired_work),
            cutter_stops_after_timeout=bool(stopped) and all(abs(m['velocity'][0]) < .5 for m in stopped),
            drive_continues_on_work_timeout=bool(active_drive) and all(v[0] > .05 for v in active_drive),
            actual_chassis_displacement=bool(truth) and truth[-1]['x']-truth[0]['x'] > 1.,
            actual_finish_stop=bool(final) and all(abs(v['speed']) < .025 for v in final))
    except (OSError, RuntimeError, ValueError) as error:
        fault = str(error)
    finally:
        for _ in range(12):
            drive.publish(Twist()); command(0)
            rclpy.spin_once(node, timeout_sec=.03)
        report = dict(ok=bool(checks) and all(checks.values()) and not fault,
                      checks=checks, fault=fault, records=records,
                      scope='Real X5/ZQWL -> HPM6750 -> PC gs_usb -> actual Gazebo cutter and chassis; reverse CAN feedback',
                      limitation='No real branches, fracture, throughput or physical implement calibration')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(ok=report['ok'], checks=checks, fault=fault, records=len(records))))
        bus.close(); node.destroy_node(); rclpy.try_shutdown()
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
