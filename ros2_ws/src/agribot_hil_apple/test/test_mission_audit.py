import json

import pytest

from agribot_hil_apple.arm_preflight import JOINTS, STOW
from agribot_hil_apple.audit_mission import audit


def fixture():
    fruit = ['harvest_apple_1', 'harvest_apple_2']
    waypoints = [[2., 36.], [41., 36.], [41., 32.5], [28.3, 31.95],
                 [41., 32.5], [41., 36.], [2., 36.]]
    trace = []
    for a, b in zip(waypoints, waypoints[1:]):
        for i in range(100):
            trace.append([len(trace)*.2, *[x+(y-x)*i/100 for x, y in zip(a, b)], 0., 0., 0., 0., 1.])
    trace.append([len(trace)*.2, 2., 36., 0., 0., 0., 0., 1.])
    mission = dict(status='succeeded', executed=True, harvested=fruit,
        manifest={'spawn': [2., 36., .1325, 0.], 'outbound_via': [[*p, 0.] for p in waypoints[1:4]],
                  'return_via': [[*p, 0.] for p in waypoints[4:]]},
        navigation=[dict(phase=p, status='succeeded') for p in ('outbound', 'return')],
        events=[dict(event='navigation_completed', phase='outbound', sim_time=60.),
                dict(event='following', phase='return', sim_time=60.)],
        final_cargo=[dict(model=f, center_in_base=[-.05, 0., .245], inside_tray=True) for f in fruit],
        final_observed_base=[2., 36., 0., 0., 0., 0., 1.],
        final_arm_positions=dict(zip(JOINTS, STOW)))
    events = []
    for model in fruit:
        contacts = {str(i): [[model+'::fruit::grasp_stem',
            f'hil_ackermann::manipulator_finger{i}::stem_soft_pad_collision']] for i in (1, 2)}
        events += [dict(model=model, event='stem_contacts_confirmed', contacts=contacts, tcp_error_m=.00001),
                   dict(model=model, event='stem_detached_from_tree'), dict(model=model, event='fruit_deposited')]
    harvest = dict(status='succeeded', harvested=fruit,
                   events=events+[dict(event='simulation_parking_constraint', engaged=False)])
    recording = dict(traces={'/hil/ground_truth': trace}, arm=[[120., *STOW]],
        can=[dict(errors=0, nonzero_commands=2000, tx_feedback=6000, fault='')],
        counts={'/wheel/odometry': 500, '/fastlivo_rtk/odometry': 500, 'ready_true': 500})
    return mission, harvest, recording


def test_valid_continuous_evidence_passes():
    report = audit(*fixture())
    assert report['status'] == 'passed'
    json.dumps(report, allow_nan=False)


def test_final_pose_uses_mission_time_not_later_idle_recording():
    mission, harvest, recording = fixture()
    mission['events'].append(dict(event='finished', sim_time=120.))
    recording['traces']['/hil/ground_truth'].append([130., 2.2, 36., 0., 0., 0., 0., 1.])
    report = audit(mission, harvest, recording)
    assert report['status'] == 'passed'
    assert report['metrics']['independent_final_sample_sim_time'] == 120.


def test_audit_rejects_missing_telemetry_at_mission_finish():
    mission, harvest, recording = fixture()
    mission['events'].append(dict(event='finished', sim_time=125.))
    report = audit(mission, harvest, recording)
    assert report['status'] == 'failed'
    assert report['checks']['independent_final_sample_is_timely'] is False


def test_later_idle_agreement_cannot_hide_wrong_pose_at_finish():
    mission, harvest, recording = fixture()
    mission['events'].append(dict(event='finished', sim_time=120.))
    recording['traces']['/hil/ground_truth'][-1][1] += .2
    recording['traces']['/hil/ground_truth'].append([130., 2., 36., 0., 0., 0., 0., 1.])
    report = audit(mission, harvest, recording)
    assert report['status'] == 'failed'
    assert report['checks']['mission_and_independent_final_pose_agree'] is False


def test_outbound_vias_cannot_be_matched_to_closer_points_on_the_return():
    mission, harvest, recording = fixture()
    mission['manifest']['outbound_via'].insert(-1, [35., 32., 3.14])
    recording['traces']['/hil/ground_truth'][200][2] += .2
    report = audit(mission, harvest, recording)
    assert report['checks']['original_aisle_vias_observed_in_order'] is True


def test_return_vias_cannot_be_satisfied_by_outbound_passages():
    mission, harvest, recording = fixture()
    mission['manifest']['return_via'].insert(0, [20., 36., 0.])
    mission['manifest']['return_via'].insert(1, [28.3, 31.95, 0.])
    report = audit(mission, harvest, recording)
    assert report['checks']['return_circuit_vias_observed_in_order'] is False


def test_requested_finish_replaces_old_original_start_requirement():
    mission, harvest, recording = fixture()
    mission['manifest'].update(finish=[3., 36., 0.], navigation_phases=['outbound', 'completion'])
    mission['manifest']['completion_via'] = mission['manifest'].pop('return_via')
    mission['manifest']['completion_via'][-1] = [3., 36., 0.]
    mission['navigation'][1]['phase'] = 'completion'
    mission['events'][1]['phase'] = 'completion'
    mission['final_observed_base'][0] = 3.
    for i, row in enumerate(recording['traces']['/hil/ground_truth'][500:]):
        row[1] = 41.+(3.-41.)*i/100.
    report = audit(mission, harvest, recording)
    assert report['status'] == 'passed'
    assert report['metrics']['independent_finish_error_m'] == 0.


@pytest.mark.parametrize('problem', ['failed_mission', 'missing_finger', 'lost_fruit', 'teleport', 'no_recording'])
def test_audit_rejects_partial_or_inconsistent_evidence(problem):
    mission, harvest, recording = fixture()
    if problem == 'failed_mission':
        mission['status'] = 'failed'
    elif problem == 'missing_finger':
        harvest['events'][0]['contacts']['2'] = []
    elif problem == 'lost_fruit':
        mission['final_cargo'][0]['center_in_base'][2] = 0.
    elif problem == 'teleport':
        recording['traces']['/hil/ground_truth'][50][1] += 5.
    else:
        recording = {}
    report = audit(mission, harvest, recording)
    assert report['status'] == 'failed'
    json.dumps(report, allow_nan=False)
