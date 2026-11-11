"""Audit actual powertrain CAN traces and plot recorded physical plant curves."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from wire import validate


def records(path):
    with path.open() as stream:
        for line in stream:
            yield json.loads(line)


def pgn(identifier):
    value = identifier >> 8 & 0x3ffff
    return value & 0x3ff00 if (value >> 8 & 255) < 240 else value


def main(args):
    if args.output.exists():
        raise FileExistsError(args.output)
    native, forwarded, commands = {}, {}, Counter()
    final_load = final_vehicle = None
    counts, rejected = Counter(), 0
    for item in records(args.pc):
        if item['direction'] == 'reject':
            rejected += 1
            continue
        data, identity = bytes.fromhex(item['data']), item['id']
        validate(data)
        counts[f"PC {item['direction']} {identity:03X}"] += 1
        if item['direction'] == 'tx' and 0x642 <= identity <= 0x645:
            native.setdefault(identity, Counter())[item['data']] += 1
            if identity == 0x645: final_load = data
            if identity == 0x644: final_vehicle = data
        if item['direction'] == 'rx' and identity in (0x640, 0x641):
            commands[f'{identity:03X}'] += 1
    for item in records(args.usb):
        if not item['extended']:
            continue
        identity = pgn(item['id'])
        if 0xff73 <= identity <= 0xff79 or identity == 0xef00:
            validate(bytes.fromhex(item['data']))
            counts[f"X5 {item['direction']} PGN {identity:04X}"] += 1
        if item['direction'] == 'rx' and 0xff74 <= identity <= 0xff77:
            forwarded.setdefault(identity-0xff74+0x642, Counter())[item['data']] += 1
    matching = {f'{identity:03X}': dict(received=sum(values.values()),
        matched=sum((values & native.get(identity, Counter())).values()))
        for identity, values in forwarded.items()}
    source = args.run/'samples.jsonl'
    all_samples = list(records(source))
    mission = any('powertrain' in s for s in all_samples)
    samples = [s for s in all_samples if s.get('powertrain') is not None] if mission else all_samples
    states = [s['powertrain'] if mission else s['state'] for s in samples]
    active = [s for s in states if s[19] == 1]
    result = dict(scope='Recorded physical HIL; engine/transmission are nominal simulated components',
        source_report=json.loads((args.run/'result.json').read_text()),
        frame_counts=dict(counts), native_actuation_frames=dict(commands), feedback_payload_matching=matching,
        rejected_native_frames=rejected, samples=len(states), active_samples=len(active),
        actual_gear_values=sorted(set(int(s[13]) for s in active)),
        active_modes=sorted(set(int(s[0]) for s in active)),
        cvt_actual_ratio_range=[min(s[4] for s in active), max(s[4] for s in active)] if active else [],
        maximum_actual_speed=max(abs(s[10]) for s in states),
        final_speed=all_samples[-1]['plant'][4] if mission else states[-1][10],
        final_enabled=final_load[4] if mission else states[-1][19],
        final_braking=final_load[3] if mission else states[-1][15],
        unplanned_off_samples=sum(s['powertrain'][0] == 0 and abs(s['powertrain'][10]) > .03
            for s in samples if s.get('commanded_transmission', 'active') != 'off') if mission else None,
        clocks='No cross-host latency calculated; payload and counter matching only',
        sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in
                (args.pc, args.usb, source, args.run/'result.json')})
    result['audit_passed'] = bool(result['source_report']['passed'] and active and len(matching) == 4 and
        all(v['received'] > 100 and v['received'] == v['matched'] for v in matching.values()) and
        all(commands[f'{identity:03X}'] > 100 for identity in (0x640, 0x641)) and not rejected and
        abs(result['final_speed']) < .03 and result['final_enabled'] == 0 and result['final_braking'] == 1 and
        (not mission or result['unplanned_off_samples'] == 0))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
        times = [s[17]-states[0][17] for s in states]
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
        plots = [(10, 'Vehicle speed (m/s)'), (1, 'Engine speed (rpm)'),
                 (4, 'Total reduction ratio'), (5, 'Clutch pressure (kPa)')]
        for ax, (index, label) in zip(axes, plots):
            ax.plot(times, [s[index] for s in states], color='#16786c', linewidth=1, label=label)
            ax.set_ylabel(label); ax.grid(alpha=.2)
        axes[1].plot(times, [s[2] for s in states], color='#c87828', linewidth=.8, label='Input shaft')
        axes[1].legend(loc='upper right')
        axes[3].plot(times, [s[6] for s in states], color='#c87828', linewidth=.8, label='Clutch B')
        axes[3].legend(loc='upper right'); axes[-1].set_xlabel('Gazebo simulation time (s)')
        fig.suptitle('Physical X5 + HPM6750 + CAN / nominal Gazebo powertrain')
        fig.savefig(args.plot, dpi=160)
    print(json.dumps({k: result[k] for k in ('audit_passed', 'feedback_payload_matching', 'active_modes', 'final_speed')}, indent=2))
    if not result['audit_passed']:
        raise SystemExit('Powertrain evidence audit failed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('pc', 'usb', 'run', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    main(parser.parse_args())
