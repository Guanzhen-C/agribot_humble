"""Check navigation continuity separately from successful end-to-end acceptance."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from .path_preflight import path_metrics


def assess(mission, recording, acceptance, stop_limit=2., trim=1.5):
    navigation = mission.get('navigation', [])
    failed = sum(entry.get('status') != 'succeeded' for entry in navigation)
    checks = {
        'complete_hil_acceptance_passed': acceptance.get('status') == 'passed',
        'mission_succeeded': mission.get('status') == 'succeeded',
        'no_failed_navigation_attempts': failed == 0,
        'one_continuous_follow_per_phase': sorted(entry.get('phase') for entry in navigation) ==
            sorted(mission.get('manifest', {}).get('navigation_phases', ['outbound', 'return'])),
    }
    windows, active = [], None
    for event in mission.get('events', []):
        if event['event'] == 'following':
            active = event
        elif (active is not None and event.get('phase') == active.get('phase') and
              event['event'] in ('navigation_completed', 'navigation_attempt_failed')):
            windows.append((active['phase'], active['sim_time']+trim, event['sim_time']-trim))
            active = None
    checks['navigation_windows_complete'] = bool(windows) and len(windows) == len(navigation) and active is None
    reports, stops = [], []
    rows = recording.get('twists', {}).get('/hil/ground_truth', [])
    for phase, begin, end in windows:
        samples = [row for row in rows if len(row) >= 7 and begin <= row[0] <= end]
        finite = all(all(math.isfinite(value) for value in row[:7]) for row in samples)
        gaps = [b[0]-a[0] for a, b in zip(samples, samples[1:])]
        complete = (end > begin and len(samples) >= 2 and finite and
                    all(0. < gap <= 1. for gap in gaps) and
                    samples[0][0]-begin <= .6 and end-samples[-1][0] <= .6)
        reports.append({'phase': phase, 'begin': begin, 'end': end,
                        'samples': len(samples), 'telemetry_complete': complete,
                        'maximum_sample_gap_sec': max(gaps, default=None)})
        first, last = None, None

        def finish():
            if first is not None and last-first >= stop_limit-1e-9:
                stops.append({'phase': phase, 'begin': first, 'end': last, 'duration_sec': last-first})

        for row in samples:
            stationary = (math.hypot(*row[1:4]) <= .02 and math.hypot(*row[4:7]) <= .02)
            if not stationary or (last is not None and row[0]-last > 1.):
                finish()
                first = None
            if stationary:
                if first is None:
                    first = row[0]
                last = row[0]
        finish()
    checks['velocity_telemetry_complete'] = bool(reports) and all(row['telemetry_complete'] for row in reports)
    checks['no_unplanned_stops_over_limit'] = not stops
    if mission.get('manifest', {}).get('forward_only'):
        plans = [entry.get('path', []) for entry in navigation]
        checks['forward_only_planned_paths'] = bool(plans) and all(
            len(path) >= 2 and path_metrics(path)['reverse_segments'] == 0 for path in plans)
        feedback = recording.get('twists', {}).get('/wheel/odometry', [])
        phase_feedback = [[row for row in feedback if len(row) >= 7 and begin <= row[0] <= end]
                          for _, begin, end in windows]
        # Native CAN wheel feedback is signed longitudinal velocity in base_link.
        checks['no_reverse_can_velocity'] = bool(phase_feedback) and all(
            len(samples) >= 2 and all(math.isfinite(row[1]) and row[1] >= -.02 for row in samples)
            for samples in phase_feedback)
    return {'status': 'passed' if all(checks.values()) else 'failed', 'checks': checks,
            'failed_navigation_attempts': failed, 'long_navigation_stops': stops, 'windows': reports,
            'stationary_threshold_mps': .02, 'stationary_threshold_radps': .02,
            'stop_limit_sim_sec': stop_limit, 'endpoint_trim_sim_sec': trim,
            'scope': 'Actual physics velocity during FollowPath only; excludes picking and endpoint ramps. '
                     'Does not replace the independent physical HIL acceptance audit.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory
    paths = {name: root/name for name in ('mission.json', 'harvest.json', 'independent-pc.json')}
    hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()}
    acceptance = json.loads((root/'independent-audit.json').read_text())
    report = assess(json.loads(paths['mission.json'].read_text()),
                    json.loads(paths['independent-pc.json'].read_text()), acceptance)
    report['checks']['acceptance_matches_actual_inputs'] = acceptance.get('input_sha256') == hashes
    report['status'] = 'passed' if all(report['checks'].values()) else 'failed'
    report['input_sha256'] = hashes
    report['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (root/'navigation-fluency.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['status'] == 'passed' else 1)


if __name__ == '__main__':
    main()
