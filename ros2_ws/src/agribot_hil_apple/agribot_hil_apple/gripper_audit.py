"""Independent physical joint-feedback check during every navigation leg."""
import argparse
import json
from pathlib import Path

import numpy as np


def joint_metrics(rows, begin, end, opening):
    samples = np.asarray([row for row in rows if begin <= row[0] <= end], dtype=float)
    if samples.ndim != 2 or len(samples) < 2 or samples.shape[1] != 9:
        return {'complete': False, 'samples': len(samples)}
    return {'complete': bool(np.isfinite(samples).all() and np.all(np.diff(samples[:, 0]) > 0.) and
                             np.max(np.diff(samples[:, 0])) < .25 and
                             samples[0, 0]-begin < .25 and end-samples[-1, 0] < .25),
            'samples': len(samples),
            'maximum_opening_error_m': float(np.max(np.abs(samples[:, 7:9]-opening))),
            'finger_ranges_m': np.ptp(samples[:, 7:9], axis=0).tolist()}


def audit(mission, harvest, recording):
    checks, legs = {}, []
    for start in [e for e in mission['events'] if e['event'] == 'following']:
        ends = [e for e in mission['events'] if e['event'] == 'navigation_completed' and
                e['phase'] == start['phase'] and e['sim_time'] > start['sim_time']]
        if not ends:
            checks[start['phase']+'_completed'] = False
            continue
        begin, end = start['sim_time']+2., ends[0]['sim_time']-2.
        metrics = joint_metrics(recording['arm'], begin, end,
                                mission['manifest']['transit_gripper_opening_m'])
        states = [row for row in recording['gripper_hold'] if begin <= row[0] <= end]
        checks[start['phase']+'_restraint_confirmed'] = bool(states) and all(row[1] is True for row in states)
        checks[start['phase']+'_feedback_complete'] = metrics['complete']
        checks[start['phase']+'_fingers_stable'] = metrics.get('maximum_opening_error_m', float('inf')) < .00025
        legs.append({'phase': start['phase'], 'begin': begin, 'end': end, **metrics})
    last_mode, contacts = None, []
    for event in harvest['events']:
        if event['event'] == 'transit_gripper_hold':
            last_mode = event['engaged']
        elif event['event'] == 'stem_contacts_confirmed':
            contacts.append(last_mode is False)
    checks['restraint_released_for_actual_stem_contacts'] = len(contacts) >= 2 and all(contacts)
    return {'status': 'passed' if checks and all(checks.values()) else 'failed',
            'checks': checks, 'legs': legs, 'maximum_allowed_error_m': .00025,
            'scope': 'Observed Gazebo joint feedback, not just target commands; original contact picking must also pass independent-audit'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    root = parser.parse_args().directory
    report = audit(*[json.loads((root/name).read_text()) for name in ('mission.json', 'harvest.json', 'independent-pc.json')])
    (root/'transit-gripper-audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['status'] == 'passed' else 1)


if __name__ == '__main__':
    main()
