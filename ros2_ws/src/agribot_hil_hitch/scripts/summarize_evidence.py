"""Summarize recorded physical CAN traces; do not infer cross-host latency."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from wire import DRIVE, DRIVE_STATE, SENSOR, VALVE, validate


def records(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def main(args):
    pc = list(records(args.pc))
    usb = list(records(args.usb))
    invalid = []
    counts = Counter()
    native_sensor = Counter()
    native_drive = Counter()
    usb_sensor = Counter()
    usb_drive = Counter()
    for item in pc:
        if item['direction'] == 'reject':
            invalid.append(item)
            continue
        data = bytes.fromhex(item['data'])
        validate(data)
        counts[f"PC {item['direction']} 0x{item['id']:03X}"] += 1
        if item['id'] == SENSOR and item['direction'] == 'tx':
            native_sensor[item['data']] += 1
        if item['id'] == DRIVE and item['direction'] == 'rx':
            native_drive[item['data']] += 1
    for item in usb:
        identity = item['id']
        pgn = (identity >> 8) & 0x3ffff if item['extended'] else None
        if pgn is not None and (pgn >> 8) < 240:
            pgn &= 0x3ff00
        counts[f"X5 {item['direction']} "+(f'PGN 0x{pgn:04X}' if pgn is not None else f'0x{identity:03X}')] += 1
        if pgn in (0xef00, 0xff70, 0xff71, 0xff72) or identity in (DRIVE, DRIVE_STATE):
            validate(bytes.fromhex(item['data']))
        if item['direction'] == 'rx' and pgn == 0xff70:
            usb_sensor[item['data']] += 1
        if item['direction'] == 'tx' and identity == DRIVE and not item['extended']:
            usb_drive[item['data']] += 1
    samples = list(records(args.mission/'samples.jsonl'))
    path = [item['true_xy'] for item in samples if 'true_xy' in item]
    import math
    travelled = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    errors = [math.dist(item['true_xy'], item['estimated_xy']) for item in samples
              if 'true_xy' in item and 'estimated_xy' in item]
    result = dict(scope='Post-processing of actual recorded HIL traffic', counts=dict(counts), rejected=invalid,
                  native_sensor_matching_isobus_samples=sum((native_sensor & usb_sensor).values()),
                  received_isobus_sensor_samples=sum(usb_sensor.values()),
                  drive_matching_across_6750=sum((native_drive & usb_drive).values()),
                  native_drive_samples=sum(native_drive.values()),
                  actual_travel_distance_m=travelled,
                  maximum_sampled_xy_estimation_difference_m=max(errors),
                  final_actual_speed_m_s=samples[-1]['plant'][4],
                  final_hitch_position=samples[-1]['plant'][0],
                  final_engaged=samples[-1]['plant'][2],
                  note='PC/X5 wall clocks differ; no latency is inferred from these timestamps.',
                  sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (args.pc, args.usb, args.mission/'result.json', args.mission/'samples.jsonl')})
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pc', type=Path, required=True)
    parser.add_argument('--usb', type=Path, required=True)
    parser.add_argument('--mission', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    main(parser.parse_args())
