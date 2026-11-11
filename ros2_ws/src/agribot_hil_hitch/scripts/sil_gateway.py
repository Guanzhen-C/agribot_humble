"""Connect the host-compiled 6750 endpoint to TWO virtual CAN buses for SIL only."""
import argparse
import json
from pathlib import Path
import queue
import select
import signal
import socket
import struct
import subprocess
import threading

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--endpoint', type=Path, required=True)
parser.add_argument('--upstream', default='vcan74')
parser.add_argument('--native', default='vcan75')
args = parser.parse_args()
if args.upstream == args.native or any(not x.startswith('vcan') or x == 'vcan73'
                                     for x in (args.upstream, args.native)):
    raise ValueError('SIL requires two dedicated virtual buses, never real CAN')
bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW | socket.SOCK_NONBLOCK, socket.CAN_RAW)
bus.bind((args.native,))
frame = struct.Struct('=IB3x8s')
process = subprocess.Popen([str(args.endpoint), 'gateway', args.upstream], stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE, text=True, bufsize=1)
events = queue.Queue()


def read():
    for line in process.stdout:
        events.put(json.loads(line))


reader = threading.Thread(target=read, daemon=True)
reader.start()
running = True


def stop(*_):
    global running
    running = False


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
try:
    while running and process.poll() is None:
        readable, _, _ = select.select([bus], [], [], .005)
        if bus in readable:
            identity, length, data = frame.unpack(bus.recv(frame.size))
            if (identity in (0x621, 0x631) or 0x642 <= identity <= 0x645) and length == 8:
                process.stdin.write(f'NATIVE {identity:x} {data.hex()}\n')
                process.stdin.flush()
        for _ in range(128):
            try:
                event = events.get_nowait()
            except queue.Empty:
                break
            if event['kind'] == 'native':
                data = bytes.fromhex(event['data'])
                bus.send(frame.pack(event['id'], len(data), data))
            else:
                print(json.dumps(event), flush=True)
except BrokenPipeError:
    if running:
        raise
finally:
    try:
        process.stdin.close()
    except BrokenPipeError:
        pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=2)
    reader.join(timeout=1)
    bus.close()
