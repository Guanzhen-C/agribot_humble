import pytest

from agribot_hil_apple.fluency import assess


def inputs():
    mission = {'status': 'succeeded', 'navigation': [
        {'phase': 'outbound', 'status': 'succeeded'}, {'phase': 'return', 'status': 'succeeded'}],
        'events': [
            {'event': 'following', 'phase': 'outbound', 'sim_time': 0.},
            {'event': 'navigation_completed', 'phase': 'outbound', 'sim_time': 20.},
            {'event': 'following', 'phase': 'return', 'sim_time': 30.},
            {'event': 'navigation_completed', 'phase': 'return', 'sim_time': 50.}]}
    recording = {'twists': {'/hil/ground_truth': [[i*.2, .2, 0., 0., 0., 0., 0.] for i in range(251)]}}
    return mission, recording, {'status': 'passed'}


def test_continuous_navigation_and_task_pickup_stop_pass():
    mission, recording, acceptance = inputs()
    for row in recording['twists']['/hil/ground_truth']:
        if 20. <= row[0] <= 30.:
            row[1] = 0.
    report = assess(mission, recording, acceptance)
    assert report['status'] == 'passed'
    assert not report['long_navigation_stops']


def test_long_stop_in_navigation_fails_even_if_task_succeeded():
    mission, recording, acceptance = inputs()
    for row in recording['twists']['/hil/ground_truth']:
        if 8. <= row[0] <= 11.:
            row[1] = 0.
    report = assess(mission, recording, acceptance)
    assert report['status'] == 'failed'
    assert report['long_navigation_stops'] == [
        {'phase': 'outbound', 'begin': 8., 'end': 11., 'duration_sec': 3.}]


def test_failure_recovery_is_not_certified_as_fluent():
    mission, recording, acceptance = inputs()
    mission['navigation'].insert(0, {'phase': 'outbound', 'status': 'failed'})
    report = assess(mission, recording, acceptance)
    assert report['status'] == 'failed'
    assert report['failed_navigation_attempts'] == 1
    assert not report['checks']['one_continuous_follow_per_phase']


@pytest.mark.parametrize('mode', ['gap', 'nan', 'missing'])
def test_incomplete_velocity_evidence_cannot_pass(mode):
    mission, recording, acceptance = inputs()
    rows = recording['twists']['/hil/ground_truth']
    if mode == 'gap':
        recording['twists']['/hil/ground_truth'] = [row for row in rows if not 7. < row[0] < 10.]
    elif mode == 'nan':
        rows[40][1] = float('nan')
    else:
        recording['twists'] = {}
    report = assess(mission, recording, acceptance)
    assert report['status'] == 'failed'
    assert not report['checks']['velocity_telemetry_complete']


def test_failed_physical_acceptance_cannot_be_replaced_by_smooth_velocities():
    mission, recording, _ = inputs()
    assert assess(mission, recording, {'status': 'failed'})['status'] == 'failed'


def test_short_endpoint_ramps_are_not_treated_as_unplanned_stops():
    mission, recording, acceptance = inputs()
    for row in recording['twists']['/hil/ground_truth']:
        if row[0] <= 1.4 or row[0] >= 48.6:
            row[1] = 0.
    assert assess(mission, recording, acceptance)['status'] == 'passed'


@pytest.mark.parametrize('problem', [None, 'reverse_plan', 'reverse_can', 'missing_can'])
def test_forward_circuit_requires_forward_plan_and_real_can_feedback(problem):
    mission, recording, acceptance = inputs()
    mission['manifest'] = {'forward_only': True}
    for entry in mission['navigation']:
        entry['path'] = [[0., 0., 0., 1.], [1., 0., 0., 1.]]
    recording['twists']['/wheel/odometry'] = [list(row) for row in recording['twists']['/hil/ground_truth']]
    if problem == 'reverse_plan':
        mission['navigation'][1]['path'][1][0] = -1.
    elif problem == 'reverse_can':
        recording['twists']['/wheel/odometry'][200][1] = -.2
    elif problem == 'missing_can':
        del recording['twists']['/wheel/odometry']
    assert (assess(mission, recording, acceptance)['status'] == 'passed') == (problem is None)
