"""Two real AgIsoStack++ endpoints on vcan; explicitly not physical HPM proof."""
import argparse
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time


def checked(payload):
    payload = bytearray(payload)
    result = 0
    for byte in payload[:7]:
        result ^= byte
    payload[7] = result
    return payload.hex()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    processes = {}
    events = []
    buffers = {}
    started = time.monotonic()
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind(('vcan73',))
    try:
        for role in ('gateway', 'client'):
            processes[role] = subprocess.Popen([str(args.executable), role, 'vcan73'],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1)
            buffers[processes[role].stdout.fileno()] = ''
        last_feedback = last_target = -1
        counter = 0
        while time.monotonic() - started < 12:
            elapsed = time.monotonic() - started
            if elapsed - last_feedback >= .4:
                last_feedback = elapsed
                frames = [(0x532, [1, 0, 0, 0, 0, 0, counter, 0])]
                frames += [(0x5b0, [common, 0, 0, 0, 0, 0, counter, 0]) for common in (0, 1)]
                frames += [(0x5b2 + i, [0, 0, 0, 24, 0, 0, counter, 0]) for i in range(3)]
                for identifier, payload in frames:
                    processes['gateway'].stdin.write(f'NATIVE {identifier:x} {checked(payload)}\n')
                processes['gateway'].stdin.flush()
                counter = (counter + 1) & 15
            if elapsed - last_target >= .08:
                last_target = elapsed
                for device in (1, 2, 3):
                    value = (device + 2 if device < 3 else 4321) if 4 <= elapsed < 7 else 0
                    # After 7 seconds renew only fan: other fields must expire.
                    if 7 <= elapsed < 9 and device != 1:
                        continue
                    if 7 <= elapsed < 9:
                        value = 3
                    processes['client'].stdin.write(f'SET {device} {value} {int(value > 0)}\n')
                processes['client'].stdin.flush()
            for role, process in processes.items():
                if process.poll() is not None:
                    raise RuntimeError(f'{role} exited: {process.stderr.read()}')
            readable, _, _ = select.select([p.stdout for p in processes.values()], [], [], .005)
            for stream in readable:
                import os
                raw = os.read(stream.fileno(), 16384).decode()
                buffers[stream.fileno()] += raw
                while '\n' in buffers[stream.fileno()]:
                    line, buffers[stream.fileno()] = buffers[stream.fileno()].split('\n', 1)
                    role = next(role for role, process in processes.items() if process.stdout is stream)
                    event = json.loads(line)
                    event.update(role=role, wall=elapsed)
                    events.append(event)
            if 5 <= elapsed < 5.05:
                payload = bytes.fromhex(checked([0, 5, 5, 0, 0, 0, 1, 0]))
                bus.send(struct.pack('=IB3x8s', 0x514, 8, payload))
        output = [e for e in events if e['role'] == 'gateway' and e['kind'] == 'native_tx' and e['id'] == 0x580]
        steady = [e for e in output if 5 <= e['wall'] < 6.8]
        expired = [e for e in output if 7.5 <= e['wall'] < 8.8]
        final = [e for e in output if e['wall'] > 10]
        checks = dict(
            both_networks_claimed=all(any(e['role'] == role and e['kind'] == 'network' and e['ready']
                                        for e in events) for role in processes),
            all_three_native_fields=bool(steady) and all(bytes.fromhex(e['data'])[:3] == b'\x43\xe1\x10' for e in steady),
            independent_command_leases=bool(expired) and all(bytes.fromhex(e['data'])[:3] == b'\x03\0\0' for e in expired),
            neutral_after_stop=bool(final) and all(bytes.fromhex(e['data'])[:3] == bytes(3) for e in final),
            legacy_drive_forwarded=any(e['role'] == 'gateway' and e['kind'] == 'native_tx' and e['id'] == 0x514
                                       for e in events),
            actual_application_ack=any(e['role'] == 'client' and e['kind'] == 'ack' and bytes.fromhex(e['data'])[3] == 0
                                      for e in events))
        report = dict(ok=all(checks.values()), checks=checks, events=events,
            scope='SIL only: actual pinned AgIsoStack++ processes and vcan. Native feedback deliberately supplied by fixture, no hardware or Gazebo claim.')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(ok=report['ok'], checks=checks, events=len(events))))
        return 0 if report['ok'] else 1
    finally:
        bus.close()
        for process in processes.values():
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
