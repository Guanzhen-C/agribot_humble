"""Verify physical hitch/disc motion through the host ISOBUS SIL chain, not real HIL."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal
import socket
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from wire import DRIVE, drive_command


def run(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    env = dict(os.environ, ROS_DOMAIN_ID='74', ROS_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI='http://127.0.0.1:11358', GAZEBO_MODEL_DATABASE_URI='')
    env['GAZEBO_PLUGIN_PATH'] = str(args.plugins)+':'+env.get('GAZEBO_PLUGIN_PATH', '')
    os.environ.update({k: env[k] for k in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')})
    children, files = [], []
    scripts = Path(__file__).resolve().parent
    report = dict(scope='Gazebo + host endpoint SIL, NOT real 6750/X5 HIL', cases={}, samples=[], events=[])

    def start(name, command, pipe=False):
        stream = (args.output/(name+'.log')).open('x')
        files.append(stream)
        process = subprocess.Popen(command, env=env, stdin=subprocess.PIPE if pipe else subprocess.DEVNULL,
                                   stdout=subprocess.PIPE if pipe else stream, stderr=stream,
                                   text=True, bufsize=1, start_new_session=True)
        children.append(process)
        return process

    start('gazebo', ['gzserver', '--verbose', str(args.scene/'world.sdf'),
                     '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'])
    gui = start('gui', ['gzclient', '--verbose']) if args.gui else None
    start('plant', ['/usr/bin/python3', str(scripts/'pc_bridge.py'), '--interface', 'vcan75', '--sil',
                    '--trace', str(args.output/'native.jsonl')])
    start('gateway', ['/usr/bin/python3', str(scripts/'sil_gateway.py'), '--endpoint', str(args.endpoint)])
    client = start('client', [str(args.endpoint), 'client', 'vcan74'], True)
    events = queue.Queue()

    def reader():
        for line in client.stdout:
            events.put(json.loads(line))

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    rclpy.init()
    node = Node('verify_hitch_gazebo_sil')
    measurement = dict(state=None, joints=None, at=-math.inf, ready=False)
    node.create_subscription(Float64MultiArray, '/hil_hitch/plant_state',
                             lambda msg: measurement.update(state=list(msg.data), at=time.monotonic()),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(JointState, '/hil_hitch/joint_states',
                             lambda msg: measurement.update(joints=list(msg.velocity)), 10)
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind(('vcan74',))
    sequence = 0
    started = time.monotonic()

    def capture(label, command, speed=0.):
        if gui is None:
            return
        ids = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--pid', str(gui.pid)],
                                      env=env, text=True).split()
        window = next(identity for identity in ids if 'Gazebo' in subprocess.check_output(
            ['xdotool', 'getwindowname', identity], env=env, text=True))
        xwd, png = args.output/(label+'.xwd'), args.output/(label+'.png')
        errors = []

        def worker():
            try:
                subprocess.run(['xdotool', 'windowactivate', '--sync', window], env=env, check=True, timeout=5)
                time.sleep(.5)
                subprocess.run(['xwd', '-silent', '-id', window, '-out', str(xwd)], env=env, check=True, timeout=5)
                subprocess.run(['ffmpeg', '-v', 'error', '-i', str(xwd), '-frames:v', '1', str(png)], check=True, timeout=5)
            except BaseException as error:
                errors.append(error)
        camera_thread = threading.Thread(target=worker, daemon=True)
        camera_thread.start()
        while camera_thread.is_alive():
            advance(.08, command, speed)
        camera_thread.join()
        if errors:
            raise errors[0]
        report.setdefault('screenshots', []).append(str(png))

    def advance(seconds, command='SET 0 0 0 0', speed=0., until=None):
        nonlocal sequence
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f'Child exited {child.args}: {child.returncode}')
            client.stdin.write(command+'\n'); client.stdin.flush()
            data = drive_command(speed, 0., speed > 0, sequence)
            bus.send(struct.pack('=IB3x8s', DRIVE, 8, data)); sequence = (sequence+1) & 255
            rclpy.spin_once(node, timeout_sec=.02)
            while not events.empty():
                event = events.get_nowait()
                report['events'].append(dict(at=time.monotonic()-started, event=event))
                if event['kind'] == 'network':
                    measurement['ready'] = event['ready']
            if measurement['state'] is not None:
                report['samples'].append(dict(at=time.monotonic()-started,
                                              state=measurement['state'], joints=measurement['joints']))
            if until is not None and until():
                return True
            time.sleep(.01)
        return False

    try:
        assert advance(20, until=lambda: measurement['ready'] and measurement['state'] is not None), \
            'Gazebo/native feedback/ISOBUS not ready'
        advance(.5)
        report['cases']['startup_feedback_and_address_claim'] = True
        assert advance(25, 'SET 1 9000 80 1', until=lambda: abs(measurement['state'][0]-9000) < 120), \
            ('raise', measurement['state'])
        assert measurement['state'][3] > .25, ('transport clearance', measurement['state'])
        report['cases']['raise_actual_joint'] = True
        capture('raised', 'SET 1 9000 80 1')
        advance(3, 'SET 1 9000 80 1', speed=.25)
        assert measurement['state'][2] == 0
        report['cases']['raised_travel_no_tillage'] = True
        advance(1, 'SET 0 0 0 0')
        assert advance(25, 'SET 1 0 70 1', until=lambda: measurement['state'][0] < 100), \
            ('lower', measurement['state'])
        report['cases']['lower_actual_joint'] = True
        advance(1, 'SET 1 0 70 1')
        assert measurement['state'][2] == 0
        report['cases']['stationary_no_tillage'] = True
        advance(16, 'SET 1 0 70 1', speed=.25)
        assert measurement['state'][2] == 1, ('soil engagement', measurement['state'])
        assert measurement['joints'] is not None and len(measurement['joints']) == 17 and \
            sum(abs(v) > .1 for v in measurement['joints'][1:]) >= 12, measurement['joints']
        report['cases']['lowered_travel_rotates_original_discs_and_emits_tillage'] = True
        capture('working', 'SET 1 0 70 1', .25)
        assert advance(20, 'SET 1 6000 80 1', speed=.25,
                       until=lambda: measurement['state'][0] > 4000 and measurement['state'][2] == 0)
        report['cases']['raising_clears_soil_effect'] = True
        capture('raised_after_work', 'SET 1 6000 80 1', .25)
        advance(1, 'SET 0 0 0 0')
        held = measurement['state'][0]
        advance(1, 'SET 0 0 0 0')
        assert abs(measurement['state'][0]-held) < 100
        report['cases']['hydraulic_hold'] = True
        assert abs(measurement['state'][4]) < .02
        report['cases']['stop_chassis'] = True
        report['passed'] = True
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        try:
            client.stdin.write('STOP\n'); client.stdin.flush()
        except (BrokenPipeError, OSError):
            pass
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=2)
        for stream in files:
            stream.close()
        report['child_exit_codes'] = [dict(command=p.args, code=p.returncode) for p in children]
        if any(p.returncode not in (0, -signal.SIGINT) for p in children):
            report['passed'] = False
            report['cleanup_error'] = 'A child failed during execution or cleanup; inspect logs'
        bus.close(); node.destroy_node(); rclpy.try_shutdown(); thread.join(timeout=1)
        (args.output/'result.json').write_text(json.dumps(report, indent=2)+'\n')
    if not report.get('passed'):
        raise RuntimeError(report.get('cleanup_error', 'SIL verification failed'))
    print(json.dumps(dict(passed=report.get('passed', False), cases=report['cases'], output=str(args.output))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--plugins', type=Path, required=True)
    parser.add_argument('--endpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gui', action='store_true')
    run(parser.parse_args())
