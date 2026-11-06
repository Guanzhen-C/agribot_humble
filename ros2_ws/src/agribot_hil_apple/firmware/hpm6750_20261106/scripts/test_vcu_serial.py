#!/usr/bin/env python3
import argparse
import json
import os
import select
import struct
import termios
import time
import zlib

MAGIC = b"VCFG"
FRAME_SIZE = 24


def make_request(sequence: int, machine_id: int, protocol_id: int,
                 compute_interface_id: int) -> bytes:
    frame = bytearray(FRAME_SIZE)
    struct.pack_into("<4sBBHIHHI", frame, 0, MAGIC, 1, 0x10, 8,
                     sequence, machine_id, protocol_id, compute_interface_id)
    struct.pack_into("<I", frame, 20, zlib.crc32(frame[:20]) & 0xFFFFFFFF)
    return bytes(frame)


def configure_serial(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def read_ack(fd: int, deadline: float) -> bytes:
    received = bytearray()
    while time.monotonic() < deadline:
        ready, _, _ = select.select([fd], [], [], max(0, deadline - time.monotonic()))
        if not ready:
            break
        chunk = os.read(fd, 256)
        if not chunk:
            continue
        received.extend(chunk)
        magic_index = received.find(MAGIC)
        if magic_index >= 0 and len(received) >= magic_index + FRAME_SIZE:
            return bytes(received[magic_index:magic_index + FRAME_SIZE])
        if len(received) > 512:
            del received[:-4]
    raise TimeoutError(f"VCU ACK timeout; received={received.hex(' ')}")


def decode_ack(frame: bytes, expected_sequence: int) -> dict:
    magic, version, opcode, length, sequence = struct.unpack_from("<4sBBHI", frame, 0)
    received_crc, = struct.unpack_from("<I", frame, 20)
    if magic != MAGIC or version != 1 or opcode != 0x90 or length != 8:
        raise ValueError(f"invalid ACK header: {frame.hex(' ')}")
    if sequence != expected_sequence:
        raise ValueError(f"ACK sequence {sequence} != {expected_sequence}")
    if received_crc != (zlib.crc32(frame[:20]) & 0xFFFFFFFF):
        raise ValueError("ACK CRC mismatch")
    status, slot, generation, machine_id, protocol_id = struct.unpack_from(
        "<BBHHH", frame, 12)
    return {
        "status": status,
        "activeSlot": "A" if slot == 0 else "B",
        "activeSlotId": slot,
        "generation": generation,
        "machineId": machine_id,
        "protocolId": protocol_id,
        "frameHex": frame.hex(" ").upper(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Write and verify VCFG/1 over FireDAP VCOM")
    parser.add_argument("device", help="serial device, for example /dev/cu.wchusbserial130")
    parser.add_argument("--machine", type=lambda value: int(value, 0), default=0x0001)
    parser.add_argument("--protocol", type=lambda value: int(value, 0), default=0x0101)
    parser.add_argument(
        "--compute-interface",
        choices=("serial", "can", "socketcan"),
        default="socketcan",
    )
    parser.add_argument("--sequence", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--send-delay",
        type=float,
        default=0.0,
        help="keep VCOM open before sending (needed while a RAM image is loaded by JTAG)",
    )
    args = parser.parse_args()

    fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd)
        interface_ids = {"serial": 0, "can": 1, "socketcan": 2}
        request = make_request(
            args.sequence,
            args.machine,
            args.protocol,
            interface_ids[args.compute_interface],
        )
        if args.send_delay > 0:
            print(f"VCOM ready; sending in {args.send_delay:g}s", flush=True)
            time.sleep(args.send_delay)
            termios.tcflush(fd, termios.TCIFLUSH)
        written = 0
        while written < len(request):
            written += os.write(fd, request[written:])
        ack = decode_ack(read_ack(fd, time.monotonic() + args.timeout), args.sequence)
        ack["requestedComputeInterface"] = args.compute_interface
    finally:
        os.close(fd)

    print(json.dumps(ack, ensure_ascii=False, indent=2))
    if ack["status"] != 0:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
