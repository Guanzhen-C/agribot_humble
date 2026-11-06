"""Classic SocketCAN and the physical WHEELTEC C50C Ackermann wire format."""
import math
from pathlib import Path
import socket
import struct
import subprocess
import json
from collections import deque
import importlib.util
import os
import select
import sys
import time
from functools import reduce
from operator import xor

COMMAND_ID = 0x181
FEEDBACK_IDS = (0x101, 0x102, 0x103)
CAN_FRAME = struct.Struct('=IB3x8s')


def int16(value):
    if not math.isfinite(value):
        raise ValueError('Non-finite telemetry')
    # Match C++ std::round, including negative half-integer ties.
    rounded = math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)
    return max(-32768, min(32767, rounded))


def decode_command(payload):
    if len(payload) != 8 or payload[2:4] != b'\0\0' or payload[6:] != b'\0\0':
        raise ValueError('Expected the physical eight-byte Ackermann command')
    return (struct.unpack_from('>h', payload, 0)[0] * .001,
            struct.unpack_from('>h', payload, 4)[0] * .001)


def feedback_payload(speed=0., yaw_rate=0., acceleration=(0., 0., 9.81),
                     angular_velocity=(0., 0., 0.), voltage=24.123, stopped=True):
    values = (int16(speed * 1000), 0, int16(yaw_rate * 1000),
              *(int16(x * 1671.84) for x in acceleration),
              *(int16(x / .00026644) for x in angular_velocity))
    if len(values) != 9 or not math.isfinite(voltage) or not 0 <= voltage <= 65.535:
        raise ValueError('Invalid Ackermann feedback')
    body = struct.pack('>BB9hH', 0x7b, int(bool(stopped)), *values,
                       math.floor(voltage * 1000 + .5))
    return body + bytes((reduce(xor, body, 0), 0x7d))


def check_usb_can(interface, serial):
    if not interface.startswith('can') or not interface[3:].isdigit():
        raise ValueError('Expected a CAN interface name')
    device = (Path('/sys/class/net') / interface / 'device').resolve()
    if (device / 'driver').resolve().name != 'gs_usb':
        raise ValueError('Refusing an interface not backed by the HIL USB-CAN')
    if (device.parent / 'serial').read_text().strip() != serial:
        raise ValueError('USB-CAN serial mismatch')
    link, = json.loads(subprocess.check_output(
        ['ip', '-j', '-details', 'link', 'show', interface], text=True))
    info = link['linkinfo']['info_data']
    if ('UP' not in link['flags'] or link['mtu'] != 16 or
            info['bittiming']['bitrate'] != 1000000 or info['state'] != 'ERROR-ACTIVE'):
        raise ValueError('CAN must be UP, error-active, classic CAN, at 1 Mbps')


class CanSocket:
    def __init__(self, interface, serial, identifiers):
        check_usb_can(interface, serial)
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # CAN_ERR_FLAG in can_mask selects the kernel's error-only list.
            filters = b''.join(struct.pack('=II', ident, 0xc00007ff) for ident in identifiers)
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.socket.bind((interface,))
            self.socket.settimeout(.02)
        except BaseException:
            self.socket.close()
            raise

    def send(self, identifier, data):
        if identifier not in (COMMAND_ID, *FEEDBACK_IDS) or len(data) != 8:
            raise ValueError('Only the physical Ackermann protocol is allowed')
        self.socket.send(CAN_FRAME.pack(identifier, 8, data))

    def receive(self):
        try:
            raw = self.socket.recv(CAN_FRAME.size)
        except (socket.timeout, BlockingIOError):
            return None
        if len(raw) != CAN_FRAME.size:
            raise ValueError('Non-classic CAN frame received')
        identifier, dlc, data = CAN_FRAME.unpack(raw)
        if identifier & 0xe0000000 or dlc != 8:
            raise ValueError(f'CAN bus error or invalid frame: id=0x{identifier:08x}, dlc={dlc}')
        return identifier, data

    def close(self):
        self.socket.close()


class ZqwlCdc:
    """Use the existing physical vehicle's CDC transport, not a new CAN driver."""
    def __init__(self, port, serial, identifiers, legacy_startup=False, trace=False):
        device = (Path('/sys/class/tty') / Path(port).resolve().name / 'device').resolve()
        usb = next((p for p in (device, *device.parents) if (p / 'idVendor').is_file()), None)
        if (usb is None or (usb / 'idVendor').read_text().strip() != '3562' or
                (usb / 'idProduct').read_text().strip() != '0105' or
                (usb / 'serial').read_text().strip() != serial):
            raise ValueError('Refusing a different ZQWL USB-CAN device')
        from ament_index_python.packages import get_package_prefix
        source = Path(get_package_prefix('agribot_hardware_bringup')) / 'lib' / (
            'agribot_hardware_bringup/wheeltec_car_gui.py')
        spec = importlib.util.spec_from_file_location('hil_physical_zqwl', source)
        driver = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = driver
        spec.loader.exec_module(driver)
        self.driver = driver
        self.link = driver.CanLink(port, bitrate=None if legacy_startup else 1000000,
                                   telemetry_ids=set(identifiers))
        self.pending = deque()
        self.identifiers = set(identifiers)
        self.trace = []
        if trace:
            original_write = self.link._write_all

            def trace_write(packet):
                original_write(packet)
                chunks = bytearray()
                deadline = time.monotonic() + .15
                while time.monotonic() < deadline:
                    if select.select([self.link.fd], [], [], .01)[0]:
                        raw = os.read(self.link.fd, 4096)
                        if not raw:
                            raise OSError('ZQWL disconnected during startup')
                        chunks.extend(raw)
                        self.pending.extend(frame for frame in self.link.decoder.feed(raw)
                                            if frame[0] in self.identifiers)
                self.trace.append({'usb_tx_hex': packet.hex(), 'usb_rx_hex': chunks.hex()})

            self.link._write_all = trace_write
        if not self.link.connect():
            raise RuntimeError(self.link.last_error)

    def send(self, identifier, data):
        if identifier != COMMAND_ID or len(data) != 8:
            raise ValueError('The CDC probe may only send the physical command frame')
        if not self.link.send(data):
            raise RuntimeError(self.link.last_error)

    def receive(self):
        if not self.pending and select.select([self.link.fd], [], [], .02)[0]:
            raw = os.read(self.link.fd, 4096)
            if not raw:
                raise OSError('ZQWL USB-CAN disconnected')
            self.pending.extend(frame for frame in self.link.decoder.feed(raw)
                                if frame[0] in self.identifiers)
        return self.pending.popleft() if self.pending else None

    def close(self):
        # Disable the adapter channel without adding extra actuator frames to
        # this finite byte-count test (CanLink.close normally emits eight stops).
        try:
            if self.link.fd is not None:
                self.link._write_all(self.driver.make_zqwl_channel_control_packet(False, False))
        finally:
            self.link.close(send_stop=False)
