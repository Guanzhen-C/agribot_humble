"""PowerShift/CVT verification through ISOBUS; physical and SIL scopes are explicit."""
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
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters, GetParameters
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray, String
from wire import DRIVE, drive_command
from powertrain_wire import MODES


class Bench(Node):
    def __init__(self, args, endpoint=None, bus=None):
        super().__init__('verify_powertrain', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args, self.endpoint, self.bus = args, endpoint, bus
        self.state = self.hitch = None
        self.state_at = -math.inf
        self.events = queue.Queue()
        self.history = []
        self.ready = False
        self.sequence = 0
        self.speed = 0.
        self.mode, self.gear, self.ratio = 0, 0, 0
        self.refresh = True
        self.shift_ms, self.ratio_rate = 600, 150
        self.hitch_mode = 'hold'
        self.samples = (args.output/'samples.jsonl').open('x')
        self.commands = (args.output/'commands.jsonl').open('x')
        self.report = dict(scope='Physical X5/6750/USB-CAN HIL' if args.physical else 'Host endpoint + Gazebo SIL, NOT physical HIL',
                           cases={}, passed=False)
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64MultiArray, '/hil_hitch/powertrain/plant_state', self.plant, feedback_qos)
        self.create_subscription(Float64MultiArray, '/hil_hitch/plant_state', lambda m: setattr(self, 'hitch', list(m.data)), feedback_qos)
        self.create_subscription(String, '/hil_hitch/isobus/events', lambda m: self.events.put(json.loads(m.data)), 100)
        self.velocity_pub = self.create_publisher(Twist, '/nav2/cmd_vel', 1)
        self.profile_pub = self.create_publisher(String, '/hil_hitch/powertrain/command', 1)
        self.hitch_pub = self.create_publisher(String, '/hil_hitch/command', 1)
        self.load_pub = self.create_publisher(Float64, '/hil_hitch/powertrain/load_force', 1)
        self.last_send = 0
        if endpoint:
            threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        for line in self.endpoint.stdout:
            self.events.put(json.loads(line))

    def plant(self, msg):
        self.state, self.state_at = list(msg.data), time.monotonic()
        self.samples.write(json.dumps(dict(at=self.state_at, state=self.state, hitch=self.hitch))+'\n')

    def send(self):
        self.commands.write(json.dumps(dict(at=time.monotonic(), mode=self.mode, speed=self.speed,
            gear=self.gear, ratio=self.ratio, refresh=self.refresh))+'\n')
        if self.endpoint:
            work = 'SET 0 0 0 0' if self.hitch_mode == 'hold' else 'SET 1 0 80 1'
            self.endpoint.stdin.write(work+'\n')
            if self.refresh:
                self.endpoint.stdin.write(f'SETTINGS {self.shift_ms} {self.ratio_rate}\nPOWER {self.mode} {self.gear} 1600 {self.ratio}\n')
            self.endpoint.stdin.flush()
            data = drive_command(self.speed, 0., self.speed > 0, self.sequence)
            self.bus.send(struct.pack('=IB3x8s', DRIVE, 8, data))
            self.sequence = (self.sequence+1) & 255
        else:
            velocity = Twist(); velocity.linear.x = float(self.speed); self.velocity_pub.publish(velocity)
            self.hitch_pub.publish(String(data=json.dumps(dict(mode=self.hitch_mode, position=0, speed=80 if self.hitch_mode != 'hold' else 0))))
            if self.refresh:
                self.profile_pub.publish(String(data=json.dumps(dict(mode=MODES[self.mode], gear=self.gear,
                    engine_rpm=1600, ratio=self.ratio, shift_ms=self.shift_ms, ratio_rate=self.ratio_rate))))

    def pump(self, seconds, until=None):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if time.monotonic()-self.last_send >= .04:
                self.send(); self.last_send = time.monotonic()
            rclpy.spin_once(self, timeout_sec=.01)
            for _ in range(512):
                try: event = self.events.get_nowait()
                except queue.Empty: break
                self.history.append(event)
                if event['kind'] == 'network': self.ready = event['ready']
                if event['kind'] == 'status':
                    data = bytes.fromhex(event['data']); self.ready = data[1] == data[2] == 1
            if until is not None and until(): return True
        return False

    def check(self, name, value, details=None):
        self.report['cases'][name] = dict(passed=bool(value), state=self.state, details=details)
        self.save()
        print(json.dumps(dict(case=name, passed=bool(value), state=self.state)), flush=True)
        if not value: raise AssertionError(name)

    def save(self):
        self.samples.flush()
        (self.args.output/'result.json').write_text(json.dumps(self.report, indent=2)+'\n')

    def configure_auth(self, enabled):
        client = self.create_client(SetParameters, '/hil_hitch_rdk_client/set_parameters')
        try:
            if not client.wait_for_service(timeout_sec=30.): raise RuntimeError('Physical X5 client unavailable')
            future = client.call_async(SetParameters.Request(parameters=[Parameter(name, value=enabled).to_parameter_msg()
                for name in ('drive_authorized', 'work_authorized')]))
            if not self.pump(8, future.done) or not all(v.successful for v in future.result().results):
                raise RuntimeError('Authorization not acknowledged')
        finally: self.destroy_client(client)

    def capture(self, label):
        if not self.args.gui_pid: return
        errors = []
        def worker():
            try:
                ids = subprocess.check_output(['xdotool', 'search', '--onlyvisible', '--pid', str(self.args.gui_pid)], text=True).split()
                window = next(x for x in ids if subprocess.check_output(['xdotool', 'getwindowname', x], text=True).strip() == 'Gazebo')
                subprocess.run(['xdotool', 'windowactivate', '--sync', window], check=True, timeout=5)
                xwd = self.args.output/(label+'.xwd')
                subprocess.run(['xwd', '-silent', '-id', window, '-out', str(xwd)], check=True, timeout=5)
                subprocess.run(['ffmpeg', '-v', 'error', '-i', str(xwd), '-frames:v', '1', str(self.args.output/(label+'.png'))], check=True, timeout=5)
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=worker, daemon=True); thread.start()
        while thread.is_alive(): self.pump(.05)
        thread.join()
        if errors: raise errors[0]

    def neutral(self):
        self.mode = self.gear = 0; self.ratio = self.speed = 0; self.refresh = True
        if not self.pump(8, lambda: self.state is not None and abs(self.state[10]) < .02):
            raise RuntimeError('Vehicle did not stop before transmission mode change')
        self.pump(.6)

    def run(self):
        if self.args.physical:
            self.configure_auth(False)
            for name, params in (('hil_hitch_native_plant', ['physical_can', 'powertrain_force_plant']),
                                 ('hil_hitch_rdk_client', ['powertrain_enabled'])):
                client = self.create_client(GetParameters, '/'+name+'/get_parameters')
                try:
                    if not client.wait_for_service(timeout_sec=15.): raise RuntimeError(name+' unavailable')
                    future = client.call_async(GetParameters.Request(names=params))
                    if not self.pump(8, future.done) or not all(v.bool_value for v in future.result().values):
                        raise RuntimeError('Physical force-plant profile not confirmed: '+name)
                finally: self.destroy_client(client)
        self.check('01_real_feedback_and_identity' if self.args.physical else '01_sil_feedback_and_identity',
            self.pump(35, lambda: self.ready and self.state is not None and time.monotonic()-self.state_at < .2))
        if self.args.physical: self.configure_auth(True)
        self.pump(.6)
        self.hitch_mode = 'position'; self.pump(2)
        self.mode, self.gear, self.speed = 1, 1, .12
        self.check('02_powershift_first_gear', self.pump(25, lambda: self.state[13] == 1 and .07 < self.state[10] < .2))
        self.gear, self.speed = 4, .45
        self.check('03_powershift_upshift', self.pump(30, lambda: self.state[13] == 4 and self.state[10] > .34))
        self.capture('powershift_working')
        self.gear, self.speed = 1, .12
        self.check('04_powershift_downshift', self.pump(30, lambda: self.state[13] == 1 and self.state[10] < .22))
        self.speed = .3; self.pump(4)
        start = len(self.history); self.mode, self.gear = 3, 0; self.pump(.8)
        self.check('05_moving_mode_change_rejected', any(e['kind'] == 'powertrain_ack' and
            bytes.fromhex(e['data'])[1] == 6 for e in self.history[start:]))
        self.neutral()
        self.mode, self.speed = 3, .25
        self.check('06_cvt_automatic_speed', self.pump(35, lambda: self.state[0] == 3 and .18 < self.state[10] < .33))
        first_ratio = self.state[4]; self.speed = .45
        self.check('07_cvt_continuous_ratio', self.pump(35, lambda: self.state[10] > .35 and self.state[4] < first_ratio-40))
        self.load_pub.publish(Float64(data=600.)); self.pump(12)
        self.check('08_load_disturbance_recovery', self.state[10] > .30 and self.state[14] == 600)
        self.capture('cvt_loaded_working')
        self.load_pub.publish(Float64(data=0.)); self.neutral()
        self.mode, self.ratio, self.speed = 4, 450, .5
        self.check('09_cvt_manual_ratio_450', self.pump(25, lambda: self.state[0] == 4 and abs(self.state[4]-450) < 10 and self.state[10] > .12))
        self.ratio = 260
        self.check('10_cvt_manual_ratio_260', self.pump(25, lambda: abs(self.state[4]-260) < 10 and self.state[10] > .32))
        self.refresh = False; self.pump(2.)
        self.check('11_command_timeout_brakes', self.state[15] == 1 and abs(self.state[10]) < .03)
        self.neutral(); self.hitch_mode = 'hold'; self.pump(.5); self.hitch_mode = 'position'
        self.mode, self.ratio, self.speed = 3, 0, .2
        self.check('12_explicit_neutral_recovery_and_hitch', self.pump(30, lambda: self.state[0] == 3 and self.state[10] > .12 and
                    self.hitch is not None and self.hitch[2] == 1))
        self.neutral()
        self.shift_ms, self.ratio_rate = 1000, 50
        start = len(self.history); self.pump(.6)
        self.check('13_stopped_parameter_update_ack', any(e['kind'] == 'powertrain_ack' and
            bytes.fromhex(e['data'])[1] == 0 and bytes.fromhex(e['data'])[4] == 1 for e in self.history[start:]))
        self.mode, self.ratio, self.speed = 4, 600, .2
        self.check('14_manual_ratio_before_slew', self.pump(20, lambda: self.state[0] == 4 and abs(self.state[4]-600) < 5))
        self.ratio = 300; before = self.state[18]; began = time.monotonic(); self.pump(2.)
        rate = (before-self.state[18])/(time.monotonic()-began)
        self.check('15_configured_cvt_ratio_slew', 30 < rate < 65, dict(requested_ratio_per_second=50, observed=rate))
        if self.args.native_bridge_pid:
            if not self.args.physical: raise ValueError('Feedback interruption requires the physical bridge')
            command = Path(f'/proc/{self.args.native_bridge_pid}/cmdline').read_bytes().split(b'\0')
            if not any(v.endswith(b'/agribot_hil_hitch/scripts/pc_bridge.py') for v in command) or b'--powertrain' not in command:
                raise ValueError('Only the dedicated powertrain native bridge may be interrupted')
            start = len(self.history)
            began = time.monotonic()
            os.kill(self.args.native_bridge_pid, signal.SIGSTOP)
            try:
                reacted = self.pump(.5, lambda: self.state[15] == 1 and self.state[19] == 0)
                response_seconds = time.monotonic()-began
                settled = self.pump(5., lambda: abs(self.state[10]) < .03)
                settling_seconds = time.monotonic()-began
                self.pump(.3)
                interrupted = list(self.state)
                stopped = any(e['kind'] == 'powertrain_status' and bytes.fromhex(e['data'])[5] & 1
                    for e in self.history[start:]) and reacted and settled and interrupted[15] == 1 and abs(interrupted[10]) < .03
            finally: os.kill(self.args.native_bridge_pid, signal.SIGCONT)
            self.neutral()
            self.check('16_physical_sensor_loss_brakes', stopped, dict(during_interruption=interrupted,
                actuator_response_seconds=response_seconds, vehicle_settling_seconds=settling_seconds))
        self.neutral(); self.report['passed'] = True


def main(args):
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    children, files = [], []
    scripts = Path(__file__).resolve().parent
    if not args.physical:
        os.environ.update(ROS_DOMAIN_ID='74', ROS_LOCALHOST_ONLY='1', GAZEBO_MASTER_URI='http://127.0.0.1:11358',
                          GAZEBO_PLUGIN_PATH=str(args.plugins)+':'+os.environ.get('GAZEBO_PLUGIN_PATH', ''))
        def start(name, command, pipe=False):
            stream = (args.output/(name+'.log')).open('x'); files.append(stream)
            process = subprocess.Popen(command, stdout=subprocess.PIPE if pipe else stream, stderr=stream,
                stdin=subprocess.PIPE if pipe else subprocess.DEVNULL, text=True, bufsize=1, start_new_session=True)
            children.append(process); return process
        start('gazebo', ['gzserver', '--verbose', str(args.scene/'world.sdf'), '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'])
        if args.gui: args.gui_pid = start('gui', ['gzclient', '--verbose']).pid
        start('native', ['/usr/bin/python3', str(scripts/'pc_bridge.py'), '--sil', '--interface', 'vcan75', '--trace', str(args.output/'native.jsonl')])
        start('gateway', ['/usr/bin/python3', str(scripts/'sil_gateway.py'), '--endpoint', str(args.endpoint)])
        endpoint = start('client', [str(args.endpoint), 'client', 'vcan74'], True)
        bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW); bus.bind(('vcan74',))
    else:
        if os.environ.get('ROS_DOMAIN_ID') != '74': raise ValueError('Physical bench must use domain74')
        endpoint = bus = None
    rclpy.init(); node = Bench(args, endpoint, bus)
    try:
        node.run()
    except BaseException as error:
        node.report['error'] = str(error); raise
    finally:
        node.load_pub.publish(Float64(data=0.))
        try:
            try:
                node.neutral(); node.hitch_mode = 'hold'; node.pump(.3)
            finally:
                if args.physical: node.configure_auth(False)
        finally:
            for child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGINT)
                    try: child.wait(timeout=8)
                    except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=2)
            node.report['child_exit_codes'] = [p.returncode for p in children]
            if any(p.returncode not in (0, -signal.SIGINT) for p in children): node.report['passed'] = False
            node.report['events'] = node.history; node.save(); node.samples.close(); node.commands.close()
            node.destroy_node(); rclpy.try_shutdown()
            if bus: bus.close()
            for stream in files: stream.close()
    if not node.report['passed']: raise RuntimeError('Powertrain verification failed; inspect evidence')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--physical', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--plugins', type=Path)
    parser.add_argument('--endpoint', type=Path)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--gui-pid', type=int)
    parser.add_argument('--native-bridge-pid', type=int, help='Physical test only: interrupt this bridge for 0.7s and always resume it')
    args = parser.parse_args()
    if not args.physical and not all((args.scene, args.plugins, args.endpoint)):
        parser.error('SIL needs --scene, --plugins and --endpoint')
    main(args)
