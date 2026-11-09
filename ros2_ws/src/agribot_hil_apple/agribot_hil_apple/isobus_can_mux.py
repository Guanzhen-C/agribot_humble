"""Single physical USB owner for legacy tracked drive and 29-bit ISOBUS frames."""
import argparse
from dataclasses import dataclass
import importlib.util
import json
from pathlib import Path
import select
import signal
import socket
import struct
import sys
import time


CAN_FRAME = struct.Struct('=IB3x8s')
EFF = 0x80000000
ALLOWED_PGNS = {0xea00, 0xee00, 0xef00, 0xec00, 0xeb00, 0xc800, 0xc700,
                0xff50, 0xff52, 0xff53, 0xff54, 0xff60, 0xff61}


@dataclass(frozen=True)
class Frame:
    identifier: int
    data: bytes
    extended: bool = False

    def __post_init__(self):
        if not 0 <= self.identifier <= (0x1fffffff if self.extended else 0x7ff):
            raise ValueError('Invalid CAN identifier')
        if not 0 <= len(self.data) <= 8:
            raise ValueError('Only classic CAN is permitted')


def pgn(identifier):
    result = (identifier >> 8) & 0x3ffff
    return result & 0x3ff00 if ((identifier >> 16) & 255) < 240 else result


def packet(frame):
    # Manufacturer CDC protocol: byte2 bit2=extended, bit6=single-shot send.
    return bytes((0x5a, len(frame.data), 0x40 | (4 if frame.extended else 0))) + \
        frame.identifier.to_bytes(4, 'big') + frame.data + b'\xa5'


def allowed(frame, outbound):
    if frame.extended:
        return pgn(frame.identifier) in ALLOWED_PGNS
    return frame.identifier in ({0x514} if outbound else {0x532, 0x533, 0x534}) and len(frame.data) == 8


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.invalid = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        if len(self.buffer) > 65536:
            raise ValueError('Unbounded CDC input')
        frames = []
        while self.buffer:
            if self.buffer[0] != 0x5a:
                del self.buffer[0]
                continue
            if len(self.buffer) < 2:
                break
            length = self.buffer[1]
            if length in (0xfe, 0xff):
                if len(self.buffer) < 32:
                    break
                if self.buffer[31] != 0xa5:
                    del self.buffer[0]; self.invalid += 1
                else:
                    del self.buffer[:32]
                continue
            if length > 8:
                del self.buffer[0]; self.invalid += 1
                continue
            if len(self.buffer) < length + 8:
                break
            raw = bytes(self.buffer[:length + 8])
            if raw[-1] != 0xa5:
                del self.buffer[0]; self.invalid += 1
                continue
            del self.buffer[:length + 8]
            # Only channel0 classic data frames; no RTR, FD or reserved flag.
            if raw[2] & 0xbb or raw[3] & 0xe0:
                self.invalid += 1
                continue
            try:
                frames.append(Frame(int.from_bytes(raw[3:7], 'big'), raw[7:-1], bool(raw[2] & 4)))
            except ValueError:
                self.invalid += 1
        return frames


def open_virtual(name):
    links = json.loads(__import__('subprocess').check_output(['ip', '-j', '-d', 'link', 'show', name]))
    if len(links) != 1 or links[0]['linkinfo']['info_kind'] != 'vcan' or 'UP' not in links[0]['flags']:
        raise ValueError('The internal interface must be an UP virtual CAN, never a physical output')
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
    bus.bind((name,))
    return bus


def open_zqwl(port, serial):
    import os
    device = (Path('/sys/class/tty')/Path(port).resolve().name/'device').resolve()
    usb = next((item for item in (device, *device.parents) if (item/'idVendor').is_file()), None)
    if usb is None or (usb/'serial').read_text().strip() != serial or \
            (usb/'idVendor').read_text().strip() != '3562' or (usb/'idProduct').read_text().strip() != '0105':
        raise ValueError('Unexpected physical USB-CAN identity')
    from ament_index_python.packages import get_package_prefix
    module_path = Path(get_package_prefix('agribot_hardware_bringup'))/'lib/agribot_hardware_bringup/wheeltec_car_gui.py'
    spec = importlib.util.spec_from_file_location('hil_shared_zqwl_driver', module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    link = module.CanLink(port, bitrate=250000, command_id=0x514, telemetry_ids={0x532, 0x533, 0x534})
    if not link.connect():
        raise RuntimeError(link.last_error)
    os.set_blocking(link.fd, False)
    return link, module


def main():
    import os
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='vcan73')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00')
    parser.add_argument('--serial', default='966960660237')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    bus = open_virtual(args.interface)
    link, driver = open_zqwl(args.port, args.serial)
    decoder = Decoder()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    start = time.monotonic()
    report = dict(scope='Single real ZQWL CDC owner; internal vcan is not a CAN bypass',
                  port=args.port, serial=args.serial, bitrate=250000,
                  sent=[], received=[], blocked=0, errors=[])
    print(json.dumps(dict(ready=True, interface=args.interface, physical_serial=args.serial)), flush=True)
    try:
        while running:
            readable, _, _ = select.select([bus, link.fd], [], [], .02)
            if link.fd in readable:
                raw = os.read(link.fd, 4096)
                if not raw:
                    raise OSError('USB-CAN disconnected')
                for frame in decoder.feed(raw):
                    if not allowed(frame, False):
                        report['blocked'] += 1
                        continue
                    bus.send(CAN_FRAME.pack(frame.identifier | (EFF if frame.extended else 0),
                                            len(frame.data), frame.data.ljust(8, b'\0')))
                    report['received'].append([time.monotonic()-start, frame.identifier,
                                                frame.extended, frame.data.hex()])
            if bus in readable:
                for _ in range(64):
                    try:
                        raw = bus.recv(CAN_FRAME.size)
                    except BlockingIOError:
                        break
                    identifier, length, data = CAN_FRAME.unpack(raw)
                    if identifier & 0x60000000 or length > 8:
                        report['blocked'] += 1
                        continue
                    frame = Frame(identifier & 0x1fffffff, data[:length], bool(identifier & EFF))
                    if not allowed(frame, True):
                        report['blocked'] += 1
                        continue
                    link._write_all(packet(frame))
                    report['sent'].append([time.monotonic()-start, frame.identifier,
                                            frame.extended, frame.data.hex()])
    except BaseException as error:
        report['errors'].append(str(error))
        raise
    finally:
        report.update(wall_seconds=time.monotonic()-start, decoder_invalid=decoder.invalid)
        try:
            link._write_all(driver.make_zqwl_channel_control_packet(False, False))
        except OSError as error:
            report['errors'].append('Channel disable after disconnect: '+str(error))
        finally:
            try:
                link.close(send_stop=False)
            finally:
                bus.close()
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
