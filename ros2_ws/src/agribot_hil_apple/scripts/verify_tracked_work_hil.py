"""Finite physical-CAN HIL acceptance: drive and spraying, then command loss."""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
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
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rclpy.init()
    node = Node('hil_tracked_spray_acceptance')
    drive = node.create_publisher(Twist, '/hil/drive/command', 1)
    work = node.create_publisher(String, '/hil/work/command', 1)
    start = time.monotonic()
    records = []
    latest = {}

    def receive(kind, msg):
        elapsed = time.monotonic() - start
        if kind in ('bridge', 'isobus'):
            value = json.loads(msg.data)
        elif kind == 'motors':
            value = dict(names=list(msg.name), velocity=list(msg.velocity))
        else:
            value = dict(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                         speed=msg.twist.twist.linear.x)
        latest[kind] = (elapsed, value)
        records.append(dict(wall=elapsed, kind=kind, value=value))

    node.create_subscription(String, '/hil/tracked_can/status', lambda msg: receive('bridge', msg), 10)
    node.create_subscription(String, '/hil/isobus/events', lambda msg: receive('isobus', msg), 100)
    node.create_subscription(JointState, '/hil/work/joint_states', lambda msg: receive('motors', msg), 20)
    node.create_subscription(Odometry, '/hil/ground_truth', lambda msg: receive('truth', msg), 20)
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
    bus.bind(('can0',))
    phase_at = None
    gateway_at = -math.inf
    checks = {}
    fault = ''
    captured = False
    neutral = String(data=json.dumps(dict(fan_gear=0, pump_gear=0, shredder_level=0)))
    try:
        while time.monotonic() - start < 45:
            now = time.monotonic()-start
            node_event = latest.get('isobus', (-math.inf, {}))
            event = node_event[1]
            if event.get('kind') == 'gateway':
                raw = bytes.fromhex(event['data'])
                if raw[0] == 1 and raw[1] == 1 and raw[2] & 3 == 3:
                    gateway_at = node_event[0]
            status = latest.get('bridge', (-math.inf, {}))
            healthy = now - status[0] < 1.5 and not status[1].get('fault', 'missing') and \
                status[1].get('motion_authorized') and status[1].get('work_authorized') and \
                status[1].get('truth_fresh') and status[1].get('motors_fresh') and now-gateway_at < .5
            if phase_at is None:
                drive.publish(Twist()); work.publish(neutral)
                if now > 3 and healthy:
                    phase_at = now
                    records.append(dict(wall=now, kind='preflight_ready', value={}))
                elif now > 20:
                    raise RuntimeError('Physical ISOBUS gateway / simulation feedback not ready')
            else:
                elapsed = now - phase_at
                command = Twist()
                if not healthy:
                    raise RuntimeError('Physical HIL health lost during acceptance')
                if elapsed < 6:
                    command.linear.x = .15
                    work.publish(String(data=json.dumps(dict(fan_gear=3, pump_gear=4, shredder_level=0))))
                elif elapsed < 8:
                    work.publish(neutral)
                elif elapsed < 12:
                    command.linear.x = .10
                    work.publish(String(data=json.dumps(dict(fan_gear=2, pump_gear=3, shredder_level=0))))
                elif elapsed < 16:
                    command.linear.x = .10
                    # Deliberately drop implement input only, keep drive active.
                elif elapsed < 18:
                    work.publish(neutral)
                else:
                    break
                drive.publish(command)
                if args.screenshot and not captured and 3.5 < elapsed < 5.5:
                    args.screenshot.parent.mkdir(parents=True, exist_ok=True)
                    xwd = args.screenshot.with_suffix('.xwd')
                    subprocess.run(['xwd', '-name', 'Gazebo', '-silent', '-out', str(xwd)], check=True, timeout=2)
                    subprocess.run(['ffmpeg', '-loglevel', 'error', '-i', str(xwd), '-frames:v', '1',
                                    str(args.screenshot)], check=True, timeout=2)
                    records.append(dict(wall=now, kind='actual_gazebo_screenshot', value=str(args.screenshot)))
                    captured = True
            for _ in range(128):
                try:
                    raw = bus.recv(CAN_FRAME.size)
                except BlockingIOError:
                    break
                ident, length, data = CAN_FRAME.unpack(raw)
                if ident & 0xe0000000 or length != 8:
                    raise RuntimeError('Unexpected native CAN frame format')
                validate(data)
                records.append(dict(wall=now, kind='native_can', value=dict(id=ident, data=data.hex())))
            rclpy.spin_once(node, timeout_sec=.025)
        if phase_at is None:
            raise RuntimeError('Acceptance never became ready')
        def selected(kind, begin, end):
            return [record['value'] for record in records if record['kind'] == kind
                    and begin < record['wall']-phase_at < end]
        motors = selected('motors', 3, 5.8)
        stopped = selected('motors', 14.5, 15.8)
        native = selected('native_can', 3, 5.8)
        expired = selected('native_can', 14.5, 15.8)
        truths = selected('truth', 0, 18)
        final = selected('truth', 17, 18)
        drive_frames = [decode_drive(bytes.fromhex(v['data'])) for v in native if v['id'] == 0x514]
        work_frames = [decode_work(bytes.fromhex(v['data'])) for v in native if v['id'] == 0x580]
        expired_work = [decode_work(bytes.fromhex(v['data'])) for v in expired if v['id'] == 0x580]
        concurrent_drive = [decode_drive(bytes.fromhex(v['data'])) for v in expired if v['id'] == 0x514]
        checks = dict(
            actual_native_drive=bool(drive_frames) and all(v[0] > .1 for v in drive_frames),
            isobus_converted_to_native_work=bool(work_frames) and all(v == (3, 4, 0) for v in work_frames),
            actual_integrated_motor_motion=bool(motors) and all(all(v > 50 for v in m['velocity']) for m in motors),
            independent_work_command_timeout=bool(expired_work) and all(v == (0, 0, 0) for v in expired_work),
            drive_continues_during_work_timeout=bool(concurrent_drive) and all(v[0] > .05 for v in concurrent_drive),
            actual_motors_stop_on_timeout=bool(stopped) and all(all(abs(v) < .5 for v in m['velocity']) for m in stopped),
            actual_chassis_displacement=bool(truths) and truths[-1]['x']-truths[0]['x'] > 1.,
            chassis_stopped_at_finish=bool(final) and all(abs(v['speed']) < .025 for v in final))
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        fault = str(error)
    finally:
        for _ in range(10):
            drive.publish(Twist()); work.publish(neutral)
            rclpy.spin_once(node, timeout_sec=.025)
        report = dict(ok=bool(checks) and all(checks.values()) and not fault,
                      checks=checks, fault=fault, records=records,
                      scope='Real X5 client -> ZQWL CAN1 -> flashed HPM6750 -> CAN0 PC -> Gazebo actual physics and native feedback')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(ok=report['ok'], checks=checks, fault=fault, records=len(records))))
        bus.close(); node.destroy_node(); rclpy.try_shutdown()
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
