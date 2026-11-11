"""Exercise the actual pinned stack and controller on an isolated virtual CAN bus.

This is SIL evidence only. The simple plant below is not Gazebo or a real ECU.
"""
import argparse
import json
from pathlib import Path
import queue
import socket
import struct
import subprocess
import threading
import time


def crc(data):
    value = 255
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x1d if value & 128 else 0)) & 255
    return value ^ 255


def sealed(data):
    return data + bytes([crc(data)])


class Process:
    def __init__(self, executable, role, interface):
        self.process = subprocess.Popen([str(executable), role, interface], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.events = queue.Queue()
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        for line in self.process.stdout:
            self.events.put(json.loads(line))

    def write(self, text):
        self.process.stdin.write(text+'\n')
        self.process.stdin.flush()

    def close(self):
        if self.process.poll() is None:
            self.write('STOP')
            self.process.stdin.close()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=2)
        self.thread.join(timeout=1)


def run(args):
    assert crc(b'123456789') == 0x4b
    if not args.interface.startswith('vcan') or args.interface == 'vcan73':
        raise ValueError('Use a separate vcan interface, not either existing HIL bus')
    report = dict(scope='software-in-the-loop, not physical HIL', cases={}, events=[], samples=[])
    gateway = Process(args.executable, 'gateway', args.interface)
    # A late joining client must learn the gateway through the standard
    # three-byte PGN request, not just a coincident startup address claim.
    time.sleep(2.)
    client = Process(args.executable, 'client', args.interface)
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind((args.interface,))
    position, velocity, valve, counter = 4000., 0., 0., 0
    ready = False
    native_ids = []
    started = time.monotonic()

    def step(duration, command=None, feedback=True):
        nonlocal position, velocity, valve, counter, ready
        stop = time.monotonic()+duration
        last = time.monotonic()
        while time.monotonic() < stop:
            now = time.monotonic()
            dt = min(now-last, .04)
            last = now
            while not gateway.events.empty():
                event = gateway.events.get_nowait()
                report['events'].append(dict(source='gateway', at=now-started, event=event))
                if event['kind'] == 'native':
                    native_ids.append(event['id'])
                    if event['id'] == 0x630:
                        data = bytes.fromhex(event['data'])
                        assert crc(data[:7]) == data[7]
                        valve = struct.unpack('<h', data[:2])[0] if data[2] else 0
            while not client.events.empty():
                event = client.events.get_nowait()
                report['events'].append(dict(source='client', at=now-started, event=event))
                if event['kind'] == 'network':
                    ready = event['ready']
            velocity += min(1., dt*20)*(valve*3-velocity)
            position = max(0., min(10000., position+velocity*dt))
            if feedback:
                flags = int(position <= 2) | (int(position >= 9998) << 1)
                data = sealed(struct.pack('<HhBBB', round(position), round(velocity), flags, 0, counter))
                gateway.write(f'NATIVE 631 {data.hex()}')
                counter = (counter+1) & 255
            if command is not None:
                client.write(command)
            report['samples'].append([now-started, position, velocity, valve])
            time.sleep(.02)

    try:
        step(3, 'SET 0 0 0 0')
        assert ready, 'ISOBUS address claim/status did not become ready'
        report['cases']['address_claim'] = True
        step(7, 'SET 1 9000 80 1')
        assert abs(position-9000) < 100, ('raising', position)
        report['cases']['raise_position'] = True
        step(8, 'SET 1 1200 80 1')
        assert abs(position-1200) < 100, ('lowering', position)
        report['cases']['lower_position'] = True
        step(.5, 'SET 0 0 0 0')
        held = position
        step(.8, 'SET 0 0 0 0')
        assert abs(position-held) < 5 and valve == 0
        report['cases']['hold_not_lower_to_zero'] = True
        step(.8, 'SET 1 8000 80 1')
        assert valve > 0
        step(.5, 'SET 1 8000 80 1', feedback=False)
        assert valve == 0
        report['cases']['sensor_timeout_stops_valve'] = True
        step(.5)
        step(.5, 'SET 1 8000 80 1')
        assert valve == 0, 'Stale recovery resumed motion without explicit HOLD'
        report['cases']['recovery_requires_hold'] = True
        step(.3, 'SET 0 0 0 0')
        step(.6, 'SET 1 8000 80 1')
        assert valve > 0
        step(.8)
        assert valve == 0
        report['cases']['command_timeout_stops_valve'] = True
        # Real chassis IDs must not enter the new native plant protocol.
        for identity in (0x181, 0x514, 0x580):
            data = bytes(8)
            bus.send(struct.pack('=IB3x8s', identity, 8, data))
        step(.2, 'SET 0 0 0 0')
        assert not set(native_ids) & {0x181, 0x514, 0x580}
        report['cases']['real_vehicle_ids_not_forwarded'] = True
        report['passed'] = True
    finally:
        client.close()
        gateway.close()
        bus.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(report, stream)
    print(json.dumps(dict(passed=report.get('passed', False), cases=report['cases'], output=str(args.output))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--interface', default='vcan74')
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())
