"""Exercise the actual X5/6750 hitch chain; never publish actuator topics."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class Verification(Node):
    def __init__(self, output):
        super().__init__('verify_physical_hitch', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.report = dict(scope='Physical X5 + HPM6750 + two USB-CAN + Gazebo plant',
                           cases={}, samples=[], events=[], passed=False)
        self.output = output
        self.state = None
        self.state_at = -math.inf
        self.ready = False
        self.joints = []
        self.active_command = None
        self.publisher = self.create_publisher(String, '/hil_hitch/command', 1)
        self.parameters = self.create_client(SetParameters, '/hil_hitch_rdk_client/set_parameters')
        self.create_subscription(Float64MultiArray, '/hil_hitch/plant_state', self.plant,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(JointState, '/hil_hitch/joint_states',
                                 lambda msg: setattr(self, 'joints', list(msg.velocity)), 10)
        self.create_subscription(String, '/hil_hitch/isobus/events', self.event, 50)
        self.started = time.monotonic()
        self.last_send = 0.

    def plant(self, msg):
        self.state, self.state_at = list(msg.data), time.monotonic()
        self.report['samples'].append(dict(at=self.state_at-self.started, state=self.state, joints=self.joints))

    def event(self, msg):
        value = json.loads(msg.data)
        self.report['events'].append(dict(at=time.monotonic()-self.started, event=value))
        if value['kind'] == 'status':
            data = bytes.fromhex(value['data'])
            self.ready = data[1] == data[2] == 1 and data[3] == 0

    def pump(self, seconds, until=None):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if self.active_command is not None and time.monotonic()-self.last_send >= .05:
                self.publisher.publish(String(data=json.dumps(self.active_command)))
                self.last_send = time.monotonic()
            rclpy.spin_once(self, timeout_sec=.02)
            if until is not None and until():
                return True
        return False

    def wait_future(self, future, seconds=8.):
        if not self.pump(seconds, future.done):
            raise TimeoutError('Parameter request timed out')
        return future.result()

    def authorize(self, work):
        if not self.parameters.wait_for_service(timeout_sec=30.):
            raise RuntimeError('Actual X5 client is unavailable')
        request = SetParameters.Request(parameters=[
            Parameter('drive_authorized', value=False).to_parameter_msg(),
            Parameter('work_authorized', value=work).to_parameter_msg()])
        response = self.wait_future(self.parameters.call_async(request))
        if not all(item.successful for item in response.results):
            raise RuntimeError('Authorization refused')

    def command(self, mode, position=0, speed=80):
        self.active_command = dict(mode=mode, position=position, speed=speed if mode != 'hold' else 0)

    def check(self, name, condition, details=None):
        self.report['cases'][name] = dict(passed=bool(condition), details=details)
        self.output.write_text(json.dumps(self.report, indent=2)+'\n')
        print(json.dumps(dict(case=name, passed=bool(condition), details=details)), flush=True)
        if not condition:
            raise AssertionError(name)

    def run(self):
        self.authorize(False)
        ok = self.pump(15., lambda: self.ready and self.state is not None and time.monotonic()-self.state_at < .2)
        self.check('01_physical_identity_and_feedback', ok)
        self.authorize(True)
        self.pump(.6)
        initial = self.state[0]
        self.command('raise')
        ok = self.pump(40., lambda: self.state[0] > 9700)
        self.check('02_raise', ok and self.state[0] > initial+1000, self.state)
        self.pump(3.)
        self.check('03_upper_limit', self.state[0] <= 10000 and abs(self.state[1]) < 100, self.state)
        self.command('position', 5000)
        ok = self.pump(30., lambda: abs(self.state[0]-5000) < 130 and abs(self.state[1]) < 120)
        self.check('04_position_50_percent', ok, self.state)
        self.command('hold')
        before = self.state[0]
        self.pump(3.)
        self.check('05_hold', abs(self.state[0]-before) < 120, dict(before=before, after=self.state[0]))
        self.command('position', 7500)
        ok = self.pump(30., lambda: abs(self.state[0]-7500) < 130 and abs(self.state[1]) < 120)
        self.check('06_position_75_percent', ok, self.state)
        self.command('lower')
        ok = self.pump(40., lambda: self.state[0] < 250)
        self.check('07_lower', ok, self.state)
        self.pump(3.)
        self.check('08_lower_limit', self.state[0] >= 0 and abs(self.state[1]) < 100, self.state)
        self.command('raise')
        self.pump(3.)
        self.active_command = None
        self.pump(1.)
        before = self.state[0]
        self.pump(2.)
        self.check('09_command_timeout_holds', abs(self.state[0]-before) < 120 and abs(self.state[1]) < 100,
                   dict(before=before, after=self.state))
        self.command('position', 9000)
        ok = self.pump(35., lambda: abs(self.state[0]-9000) < 130 and abs(self.state[1]) < 120)
        self.check('10_new_command_recovers', ok, self.state)
        self.report['passed'] = True

    def run_position(self, target):
        self.authorize(False)
        if not self.pump(20., lambda: self.ready and self.state is not None and time.monotonic()-self.state_at < .2):
            raise RuntimeError('Fresh physical feedback required')
        self.authorize(True)
        self.pump(.6)
        self.command('position', target)
        reached = self.pump(35., lambda: abs(self.state[0]-target) < 130 and abs(self.state[1]) < 120)
        self.pump(1.)
        self.check('requested_physical_position', reached, dict(target=target, state=self.state))
        self.report['passed'] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--position', type=int, help='Check one position (0..10000) instead of the ten-case sequence')
    args = parser.parse_args()
    if args.position is not None and not 0 <= args.position <= 10000:
        parser.error('--position must be between 0 and 10000')
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Verification(args.output)
    try:
        if args.position is None:
            node.run()
        else:
            node.run_position(args.position)
    except BaseException as error:
        node.report['error'] = str(error)
        raise
    finally:
        node.command('hold')
        node.pump(.3)
        node.authorize(False)
        node.output.write_text(json.dumps(node.report, indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
