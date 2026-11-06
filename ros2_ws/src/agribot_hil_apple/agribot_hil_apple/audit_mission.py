"""Offline cross-check of one continuous mission against independent PC evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from .arm_preflight import JOINTS, STOW


def audit(mission, harvest, recording):
    checks, metrics = {}, {}
    manifest = mission['manifest']
    spawn = manifest['spawn'][:2]
    finish = manifest.get('finish', spawn)[:2]
    trace = recording.get('traces', {}).get('/hil/ground_truth', [])
    end_times = [e['sim_time'] for e in mission.get('events', []) if e['event'] == 'finished']
    end_time = end_times[-1] if end_times else None
    # The recorder can outlive the finite mission. Compare its final observation
    # with the same mission time, not with later idle-world physics.
    final_sample = (min(trace, key=lambda row: abs(row[0]-end_time))
                    if trace and end_time is not None else trace[-1] if trace else None)
    checks['independent_final_sample_is_timely'] = final_sample is not None and (
        end_time is None or abs(final_sample[0]-end_time) <= .5)
    if final_sample is not None:
        metrics['independent_final_sample_sim_time'] = final_sample[0]
        metrics['mission_finish_sim_time'] = end_time
    picked = mission.get('harvested', [])
    checks['continuous_mission_succeeded'] = mission.get('status') == 'succeeded' and mission.get('executed') is True
    checks['at_least_two_distinct_fruit'] = len(set(picked)) >= 2 and len(set(picked)) == len(picked)
    checks['harvest_report_agrees'] = harvest.get('status') == 'succeeded' and picked == harvest.get('harvested')
    phases = {entry['phase'] for entry in mission.get('navigation', []) if entry['status'] == 'succeeded'}
    expected_phases = set(manifest.get('navigation_phases', ['outbound', 'return']))
    checks['both_navigation_phases_succeeded'] = expected_phases <= phases
    checks['trace_covers_original_start_and_finish'] = bool(trace) and (
        math.dist(trace[0][1:3], spawn) <= .25 and math.dist(final_sample[1:3], finish) <= .3)
    if trace:
        metrics['independent_finish_error_m'] = math.dist(final_sample[1:3], finish)
        steps = [math.dist(a[1:3], b[1:3]) for a, b in zip(trace, trace[1:])]
        metrics['maximum_observed_step_m'] = max(steps, default=0.)
    checks['continuous_observed_motion'] = len(trace) > 100 and (
        metrics['maximum_observed_step_m'] < 1. and all(b[0] > a[0] for a, b in zip(trace, trace[1:])))
    outbound_ends = [e['sim_time'] for e in mission.get('events', [])
                     if e['event'] == 'navigation_completed' and e.get('phase') == 'outbound']
    checks['outbound_completion_time_recorded'] = bool(outbound_ends)
    outbound_trace = [row for row in trace if outbound_ends and row[0] <= outbound_ends[0]+.25]
    index, via_errors = 0, []
    for via in manifest['outbound_via'][:-1]:
        remaining = [(math.dist(row[1:3], via[:2]), i) for i, row in enumerate(outbound_trace) if i >= index]
        if not remaining:
            via_errors.append(None)
            break
        error, index = min(remaining)
        via_errors.append(error)
    metrics['ordered_outbound_via_errors_m'] = via_errors
    checks['original_aisle_vias_observed_in_order'] = len(via_errors) == len(manifest['outbound_via'])-1 and all(e is not None and e < .9 for e in via_errors)
    completion_phase = 'completion' if 'completion_via' in manifest else 'return'
    completion_vias = manifest.get(completion_phase+'_via', [])
    return_starts = [e['sim_time'] for e in mission.get('events', [])
                     if e['event'] == 'following' and e.get('phase') == completion_phase]
    return_trace = [row for row in trace if return_starts and row[0] >= return_starts[0]-.25]
    index, return_errors = 0, []
    for via in completion_vias[:-1]:
        remaining = [(math.dist(row[1:3], via[:2]), i) for i, row in enumerate(return_trace) if i >= index]
        if not remaining:
            return_errors.append(None)
            break
        error, index = min(remaining)
        return_errors.append(error)
    metrics['ordered_return_via_errors_m'] = return_errors
    checks['return_circuit_vias_observed_in_order'] = bool(return_starts) and (
        completion_phase+'_via' in manifest and len(return_errors) == len(completion_vias)-1 and
        all(e is not None and e < .9 for e in return_errors))
    events = harvest.get('events', [])
    contact_errors = []
    for model in picked:
        relevant = [(i, e) for i, e in enumerate(events) if e.get('model') == model]
        contacts = [(i, e) for i, e in relevant if e['event'] == 'stem_contacts_confirmed']
        detach = [i for i, e in relevant if e['event'] == 'stem_detached_from_tree']
        deposit = [i for i, e in relevant if e['event'] == 'fruit_deposited']
        valid = False
        for i, event in contacts:
            bilateral = all(any(set(pair) == {
                model+'::fruit::grasp_stem',
                f'hil_ackermann::manipulator_finger{finger}::stem_soft_pad_collision'}
                for pair in event.get('contacts', {}).get(str(finger), [])) for finger in (1, 2))
            error = event.get('tcp_error_m', float('inf'))
            contact_errors.append(error if math.isfinite(error) else None)
            valid |= bilateral and math.isfinite(error) and error < .0007 and any(i < d < p for d in detach for p in deposit)
        checks['bilateral_stem_pick_and_deposit_'+model] = valid
    metrics['contact_tcp_errors_m'] = contact_errors
    parking = [e['engaged'] for e in events if e['event'] == 'simulation_parking_constraint']
    checks['parking_constraint_released'] = bool(parking) and parking[-1] is False
    cargo = mission.get('final_cargo', [])
    checks['all_picked_fruit_in_physical_tray'] = bool(picked) and {f['model'] for f in cargo} == set(picked) and all(
        -.15 < f['center_in_base'][0] < .03 and abs(f['center_in_base'][1]) < .14 and
        .22 < f['center_in_base'][2] < .32 and f.get('inside_tray') is True for f in cargo)
    final = mission.get('final_observed_base', [])
    checks['mission_and_independent_final_pose_agree'] = len(final) == 7 and final_sample is not None and math.dist(final[:2], final_sample[1:3]) < .05
    joints = mission.get('final_arm_positions', {})
    arm_trace = recording.get('arm', [])
    checks['arm_stowed_in_both_records'] = all(abs(joints.get(joint, float('inf'))-value) < .02 for joint, value in zip(JOINTS, STOW)) and bool(arm_trace) and all(
        abs(actual-expected) < .02 for actual, expected in zip(arm_trace[-1][1:7], STOW[:6]))
    can = recording.get('can', [])
    checks['native_can_motion_and_feedback_without_errors'] = bool(can) and all(c['errors'] == 0 and not c.get('fault') for c in can) and (
        can[-1]['nonzero_commands'] > 1000 and can[-1]['tx_feedback'] > 1000)
    counts = recording.get('counts', {})
    checks['live_fused_localization_and_can_velocity_recorded'] = all(counts.get(topic, 0) > 100 for topic in (
        '/fastlivo_rtk/odometry', '/wheel/odometry')) and counts.get('ready_true', 0) > 100
    metrics['native_can_final'] = can[-1] if can else None
    return {'status': 'passed' if all(checks.values()) else 'failed', 'checks': checks, 'metrics': metrics,
            'scope': 'One continuous HIL run; Gazebo object-pose manipulation baseline and kinematic arm actuator, not real-hardware picking acceptance'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    directory = args.directory
    report = audit(*[json.loads((directory/name).read_text()) for name in ('mission.json', 'harvest.json', 'independent-pc.json')])
    report['audit_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['input_sha256'] = {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
                              for name in ('mission.json', 'harvest.json', 'independent-pc.json')}
    (directory/'independent-audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['status'] == 'passed' else 1)


if __name__ == '__main__':
    main()
