"""Single ZQWL owner for the NEW tractor/hitch bench, never either real chassis protocol."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import struct
import time

from agribot_hil_apple.isobus_can_mux import Decoder, Frame, open_virtual, open_zqwl, packet, pgn
from wire import DRIVE, DRIVE_STATE, decode_drive, validate

FRAME = struct.Struct('=IB3x8s')
PGNS = {0xea00, 0xee00, 0xef00, *range(0xff70, 0xff7a)}


def allowed(frame, outbound):
    if frame.extended:
        return pgn(frame.identifier) in PGNS
    if frame.identifier != (DRIVE if outbound else DRIVE_STATE) or len(frame.data) != 8:
        return False
    try:
        decode_drive(frame.data) if outbound else validate(frame.data)
    except ValueError:
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='vcan74')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00')
    parser.add_argument('--serial', default='966960660237')
    parser.add_argument('--trace', type=Path, required=True)
    args = parser.parse_args()
    if args.interface != 'vcan74':
        raise ValueError('Use the dedicated hitch virtual interface, not the old vcan73')
    args.trace.parent.mkdir(parents=True, exist_ok=True)
    trace = args.trace.open('x')
    bus = open_virtual(args.interface)
    # Only reuse CDC framing, exclusive USB ownership and channel setup.
    # No native real-vehicle codec/send/stop method is called.
    link, driver = open_zqwl(args.port, args.serial)
    decoder = Decoder()
    running = True

    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    print(json.dumps(dict(ready=True, protocol='hitch-hil-0x21', serial=args.serial, bitrate=250000)), flush=True)
    try:
        while running:
            readable, _, _ = select.select([bus, link.fd], [], [], .01)
            if link.fd in readable:
                raw = os.read(link.fd, 4096)
                if not raw:
                    raise OSError('Physical USB-CAN disconnected')
                for frame in decoder.feed(raw):
                    if allowed(frame, False):
                        bus.send(FRAME.pack(frame.identifier | (0x80000000 if frame.extended else 0),
                                            len(frame.data), frame.data.ljust(8, b'\0')))
                        trace.write(json.dumps(dict(at=time.monotonic(), direction='rx', id=frame.identifier,
                                                    extended=frame.extended, data=frame.data.hex()))+'\n')
            if bus in readable:
                for _ in range(64):
                    try:
                        raw = bus.recv(FRAME.size)
                    except BlockingIOError:
                        break
                    identity, size, data = FRAME.unpack(raw)
                    if identity & 0x60000000 or size > 8:
                        continue
                    frame = Frame(identity & 0x1fffffff, data[:size], bool(identity & 0x80000000))
                    if allowed(frame, True):
                        link._write_all(packet(frame))
                        trace.write(json.dumps(dict(at=time.monotonic(), direction='tx', id=frame.identifier,
                                                    extended=frame.extended, data=frame.data.hex()))+'\n')
            trace.flush()
    finally:
        try:
            link._write_all(driver.make_zqwl_channel_control_packet(False, False))
        finally:
            link.close(send_stop=False)
            bus.close(); trace.close()


if __name__ == '__main__':
    main()
