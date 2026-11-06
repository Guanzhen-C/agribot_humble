"""Finite physical-link probe. The only command emitted is speed=0, steer=0."""
import argparse
import json
from pathlib import Path
import time

from .can_wire import COMMAND_ID, FEEDBACK_IDS, CanSocket, ZqwlCdc, feedback_payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=('pc', 'rdk'))
    parser.add_argument('--transport', choices=('socketcan', 'zqwl_cdc'), default='socketcan')
    parser.add_argument('--interface', help='SocketCAN interface, or ZQWL CDC port')
    parser.add_argument('--expected-usb-serial', required=True)
    parser.add_argument('--legacy-cdc-startup', action='store_true',
                        help='Compare the physical GUI legacy 1Mbps startup sequence')
    parser.add_argument('--trace-cdc', action='store_true',
                        help='Record CDC control/data replies, adding 150ms after each write')
    parser.add_argument('--timeout', type=float, default=8.)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--reverse-only', action='store_true',
                        help='PC sends parking feedback; X5 only receives, no command frames')
    args = parser.parse_args()
    if not 1 <= args.timeout <= 60:
        parser.error('Timeout must be 1..60 seconds')
    if not args.interface or (args.transport == 'zqwl_cdc' and args.role != 'rdk'):
        parser.error('Specify the interface/port; CDC probing is supported on the X5 side')
    if (args.legacy_cdc_startup or args.trace_cdc) and args.transport != 'zqwl_cdc':
        parser.error('Legacy CDC startup requires the ZQWL transport')
    expected = feedback_payload()
    parts = dict(zip(FEEDBACK_IDS, (expected[i:i+8] for i in range(0, 24, 8))))
    bus = None
    sent = received = 0
    received_ids = []
    error = ''
    try:
        backend = CanSocket if args.transport == 'socketcan' else ZqwlCdc
        options = {'legacy_startup': args.legacy_cdc_startup,
                   'trace': args.trace_cdc} if backend is ZqwlCdc else {}
        bus = backend(args.interface, args.expected_usb_serial,
                      [COMMAND_ID] if args.role == 'pc' else FEEDBACK_IDS, **options)
        start = time.monotonic()
        deadline = start + args.timeout
        next_send = start + .3
        complete_at = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if args.reverse_only and args.role == 'pc' and sent < 9 and now >= next_send:
                for ident, payload in parts.items():
                    bus.send(ident, payload)
                    sent += 1
                    time.sleep(.003)
                next_send = now + .1
            elif not args.reverse_only and args.role == 'rdk' and sent < 3 and now >= next_send:
                bus.send(COMMAND_ID, b'\0' * 8)
                sent += 1
                next_send = now + .1
            frame = bus.receive()
            if frame:
                identifier, data = frame
                if args.role == 'pc':
                    if args.reverse_only:
                        raise ValueError('Another command source is active during the receive-only test')
                    if identifier != COMMAND_ID or data != b'\0' * 8:
                        raise ValueError('Unexpected or nonzero command; probe aborted')
                    received += 1
                    if received > 3:
                        raise ValueError('Extra command frames: verify physical topology')
                    for ident, payload in parts.items():
                        bus.send(ident, payload)
                        sent += 1
                        time.sleep(.003)
                else:
                    if data != parts[identifier]:
                        raise ValueError('Feedback was changed in transit')
                    received += 1
                    received_ids.append(identifier)
            if args.reverse_only:
                complete = sent == 9 if args.role == 'pc' else received == 9
            else:
                complete = received == 3 if args.role == 'pc' else sent == 3 and received == 9
            if complete and complete_at is None:
                complete_at = time.monotonic()
            if complete_at is not None and time.monotonic() - complete_at > .3:
                break
        if args.reverse_only:
            ok = (sent == 9 and received == 0 if args.role == 'pc' else
                  sent == 0 and received == 9 and received_ids == list(FEEDBACK_IDS) * 3)
        else:
            ok = (received == 3 and sent == 9 if args.role == 'pc' else
                  sent == 3 and received == 9 and received_ids == list(FEEDBACK_IDS) * 3)
        if not ok:
            error = 'Timed out: no complete bidirectional byte-exact exchange'
    except Exception as exc:
        error = str(exc)
        ok = False
    finally:
        if bus is not None:
            bus.close()
    report = {'ok': ok, 'role': args.role, 'interface': args.interface,
              'transport': args.transport, 'sent': sent, 'received': received, 'error': error,
              'legacy_cdc_startup': args.legacy_cdc_startup,
              'cdc_trace': getattr(bus, 'trace', []),
              'received_feedback_ids': received_ids,
              'command_hex': '00' * 8, 'feedback_hex': expected.hex(),
              'scope': ('PC-to-X5 parking feedback only; source success alone does not prove delivery'
                        if args.reverse_only else 'Zero-motion, physical classic CAN round-trip probe')}
    text = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n')
    print(text, flush=True)
    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
