from types import SimpleNamespace

from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

import pytest

from agribot_hil_apple.navigation import Navigator, PARAMETER_UPDATE_TIMEOUT, ViaPathProgress, advance_via_index


def test_docking_profile_uses_humble_services_and_preserves_previous_values():
    requests = []
    timeouts = []

    class Client:
        def __init__(self, service):
            self.service = service

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            requests.append(request)
            if self.service is GetParameters:
                return SimpleNamespace(values=[ParameterValue(type=3, double_value=.08)])
            assert self.service is SetParametersAtomically
            return SimpleNamespace(result=SimpleNamespace(successful=True))

    node = SimpleNamespace(
        wait_active=lambda server: requests.append(server),
        create_client=lambda kind, _name: Client(kind),
        destroy_client=lambda _client: None,
        wait=lambda result, timeout: (timeouts.append(timeout), result)[1])
    client, original = Navigator.parameter_profile(
        node, '/controller_server', {'goal_checker.xy_goal_tolerance': .015})
    assert requests[0] == 'controller_server'
    assert isinstance(requests[2], SetParametersAtomically.Request)
    assert requests[2].parameters[0].value.double_value == .015
    assert original[0].value.double_value == .08
    assert original[0].name == 'goal_checker.xy_goal_tolerance'
    assert client.service is SetParametersAtomically
    assert timeouts == [15., PARAMETER_UPDATE_TIMEOUT]
    assert PARAMETER_UPDATE_TIMEOUT >= 120.


def test_parameter_update_timeout_does_not_resubmit_and_closes_both_clients():
    clients, calls, closed = [], [], []

    class Client:
        def __init__(self, service):
            self.service = service
            clients.append(self)

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            calls.append(request)
            return request

    def wait(request, timeout):
        if isinstance(request, GetParameters.Request):
            return SimpleNamespace(values=[ParameterValue(type=3, double_value=2.)])
        assert timeout == PARAMETER_UPDATE_TIMEOUT
        raise TimeoutError('Delayed native callback')

    node = SimpleNamespace(create_client=lambda kind, _: Client(kind),
                           wait_active=lambda _: None,
                           destroy_client=closed.append, wait=wait)
    with pytest.raises(TimeoutError, match='application state is unknown'):
        Navigator.parameter_profile(node, '/planner_server', {'GridBased.reverse_penalty': 1.05})
    assert len(calls) == 2
    assert set(closed) == set(clients)


def test_rounded_corner_passage_is_preserved_for_recovery_without_retiring_final_goal():
    points = [[1., 0., 0.], [1., 1., 0.], [2., 1., 0.]]
    progress = ViaPathProgress([[0., 0.], [1., 0.], [1., 1.], [2., 1.]], points, 0)
    assert advance_via_index(points, 0, .2, .7) == 0
    assert progress.advance(0, 2.1) == 0
    assert progress.advance(0, 1.8) == 1
    assert progress.advance(1, .8) == 2
    assert progress.advance(2, 0.) == 2


def test_resumed_path_progress_uses_absolute_via_indices():
    progress = ViaPathProgress([[1., 0.], [1., 1.], [2., 1.]],
                              [[1., 0., 0.], [1., 1., 0.], [2., 1., 0.]], 1)
    assert progress.advance(1, .5) == 2
    assert progress.advance(2, 2.) == 2


def test_missing_intermediate_via_is_rejected_instead_of_silently_skipped():
    with pytest.raises(RuntimeError, match='does not include intermediate via'):
        ViaPathProgress([[0., 0.], [2., 0.]], [[1., 1., 0.], [2., 0., 0.]], 0)


@pytest.mark.parametrize('remaining', [float('nan'), float('inf'), -1.])
def test_invalid_path_feedback_cannot_advance_route(remaining):
    progress = ViaPathProgress([[0., 0.], [1., 0.], [2., 0.]], [[1., 0., 0.], [2., 0., 0.]], 0)
    assert progress.advance(0, remaining) == 0


def test_mixed_bool_and_double_profile_preserves_declared_types():
    requests = []

    class Client:
        def __init__(self, kind):
            self.kind = kind

        def wait_for_service(self, timeout_sec):
            return True

        def call_async(self, request):
            requests.append(request)
            if self.kind is GetParameters:
                return SimpleNamespace(values=[ParameterValue(type=3, double_value=.263),
                                               ParameterValue(type=1, bool_value=False)])
            return SimpleNamespace(result=SimpleNamespace(successful=True))

    node = SimpleNamespace(create_client=lambda kind, _: Client(kind),
                           wait_active=lambda _: None,
                           destroy_client=lambda _: None, wait=lambda result, _: result)
    _, original = Navigator.parameter_profile(node, '/controller_server', {
        'goal_checker.xy_goal_tolerance': .06, 'FollowPath.GoalAngleCritic.enabled': True})
    assert original[1].value.type == 1 and original[1].value.bool_value is False
    assert requests[1].parameters[1].value.type == 1
    assert requests[1].parameters[1].value.bool_value is True
