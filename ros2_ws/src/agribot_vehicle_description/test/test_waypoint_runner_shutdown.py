"""Exercise signal ordering without launching ROS or a vehicle."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import math

import pytest
import yaml


SOURCE = Path(__file__).resolve().parents[2] / "agribot_autonomy/scripts/snake_waypoint_runner.py"


def surface_helpers():
    functions = [node for node in ast.parse(SOURCE.read_text()).body
                 if isinstance(node, ast.FunctionDef) and node.name in
                 ("normalize_angle", "load_navigation_surface", "path_fits_navigation_surface")]
    namespace = {"math": math, "yaml": yaml, "Path": Path}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace


def surface_path(points):
    return SimpleNamespace(poses=[SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y), orientation=SimpleNamespace(
            x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))) for x, y, yaw in points])


def test_physical_surface_is_optional_but_rejects_invalid_configuration(tmp_path):
    functions = surface_helpers()
    path = tmp_path / "waypoints.yaml"
    path.write_text("waypoints: []\n")
    assert functions["load_navigation_surface"](str(path)) is None
    surface = dict(bounds=[-22., -15., 22., 15.], footprint=[[4., 1.6], [4., -1.6], [-3.8, -1.6]], marginM=.25)
    path.write_text(yaml.safe_dump(dict(navigationSurface=surface)))
    assert functions["load_navigation_surface"](str(path)) == surface
    surface["marginM"] = -1.
    path.write_text(yaml.safe_dump(dict(navigationSurface=surface)))
    with pytest.raises(ValueError, match="physical navigation surface"):
        functions["load_navigation_surface"](str(path))


def test_surface_checks_vehicle_corners_and_intermediate_rotation():
    check = surface_helpers()["path_fits_navigation_surface"]
    surface = dict(bounds=[-22., -15., 22., 15.],
                   footprint=[[4.2, 1.6], [4.2, -1.6], [-3.8, -1.6], [-3.8, 1.6]], marginM=.25)
    assert check(surface_path([(0., 0., 0.)]), None)
    assert not check(surface_path([(20., 0., 0.)]), surface)
    assert not check(surface_path([(18., 0., -math.pi/2), (18., 0., math.pi/2)]), surface)
    assert not check(surface_path([(float("nan"), 0., 0.)]), surface)
    assert check(surface_path([(12+6*math.sin(a), 3-6*math.cos(a), a)
                               for a in (0., math.pi/4, math.pi/2, 3*math.pi/4, math.pi)]), surface)


def test_out_of_ground_preplan_is_not_published_or_dispatched():
    cls = next(node for node in ast.parse(SOURCE.read_text()).body
               if isinstance(node, ast.ClassDef) and node.name == "SnakeWaypointRunner")
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef) and node.name == "_complete_preplanned_path")
    namespace = surface_helpers()
    namespace["NavPath"] = object
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    surface = dict(bounds=[-22., -15., 22., 15.],
                   footprint=[[4.2, 1.6], [4.2, -1.6], [-3.8, -1.6], [-3.8, 1.6]], marginM=.25)
    logger, retry, publish, create_timer = Mock(), Mock(), Mock(), Mock()
    node = SimpleNamespace(navigation_surface=surface, get_logger=lambda: logger,
                           _schedule_preplan_retry=retry, _publish_path_visuals=publish,
                           create_timer=create_timer)
    namespace[method.name](node, surface_path([(19., 0., 0.), (20., 0., 0.)]), 0., "batch")
    retry.assert_called_once_with()
    logger.error.assert_called_once()
    publish.assert_not_called()
    create_timer.assert_not_called()


def test_first_pass_through_loop_finish_keeps_goal_checker_disabled():
    cls = next(node for node in ast.parse(SOURCE.read_text()).body
               if isinstance(node, ast.ClassDef) and node.name == "SnakeWaypointRunner")
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef) and node.name == "_update_preplanned_progress")
    namespace = {"math": math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    coordinates = [(float(x), 0.) for x in range(-14, 13)]
    coordinates += [(12., float(y)) for y in range(1, 10)]
    coordinates += [(float(x), 9.) for x in range(11, -13, -1)]
    coordinates += [(-12., float(y)) for y in range(8, -1, -1)]
    coordinates += [(float(x), 0.) for x in range(-11, -7)]
    path = SimpleNamespace(poses=[SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y))) for x, y in coordinates])
    ready = Mock()
    node = SimpleNamespace(preplanned_path=path, preplan_progress_index=0,
                           preplan_completed_waypoints=0, preplan_waypoints=[1, 2, 3],
                           preplan_waypoint_path_indices=[26, 59, len(coordinates)-1],
                           robot_x=-8., robot_y=0., _set_route_goal_ready=ready)
    namespace[method.name](node)
    ready.assert_called_with(False)
    assert node.preplan_completed_waypoints == 0
    node.preplan_progress_index = len(coordinates) - 1
    namespace[method.name](node)
    ready.assert_called_with(True)


@pytest.mark.parametrize("completed", [0, 1, 3, 10])
def test_controller_abort_keeps_unvisited_waypoints_or_the_final_goal(completed):
    cls = next(node for node in ast.parse(SOURCE.read_text()).body
               if isinstance(node, ast.ClassDef) and node.name == "SnakeWaypointRunner")
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef) and node.name == "_handle_preplanned_path_failure")
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    points = [object(), object(), object()]
    logger = Mock()
    retry = Mock()
    node = SimpleNamespace(active_goal_handle=object(), preplan_completed_waypoints=completed,
                           preplan_waypoints=points, get_logger=lambda: logger,
                           _schedule_preplan_retry=retry)
    namespace[method.name](node, 6)
    assert node.preplan_waypoints == (points[completed:] if completed < 3 else points[-1:])
    assert node.preplan_completed_waypoints == 0
    assert node.active_goal_handle is None
    assert node.preplanned_path is None
    retry.assert_called_once_with()
    logger.info.assert_not_called()


@pytest.mark.parametrize("stop_signal", [2, 15])
def test_shutdown_waits_for_current_callback_before_destroying_context(stop_signal):
    events, handlers = [], {2: "old_int", 15: "old_term"}

    def install(sig, handler):
        previous = handlers[sig]
        handlers[sig] = handler
        return previous

    def spin(_node, timeout_sec):
        assert timeout_sec == .1
        handlers[stop_signal](stop_signal, None)
        assert "shutdown" not in events
        events.append("callback_return")

    def init(**kwargs):
        assert kwargs == {"signal_handler_options": "disabled"}
        events.append("init")

    main_node = next(node for node in ast.parse(SOURCE.read_text()).body
                     if isinstance(node, ast.FunctionDef) and node.name == "main")
    namespace = {
        "signal": SimpleNamespace(SIGINT=2, SIGTERM=15, signal=install),
        "SignalHandlerOptions": SimpleNamespace(NO="disabled"),
        "SnakeWaypointRunner": lambda: SimpleNamespace(destroy_node=lambda: events.append("destroy")),
        "rclpy": SimpleNamespace(init=init, ok=lambda: True, spin_once=spin,
                                  try_shutdown=lambda: events.append("shutdown")),
    }
    exec(compile(ast.Module(body=[main_node], type_ignores=[]), str(SOURCE), "exec"), namespace)
    namespace["main"]()
    assert events == ["init", "callback_return", "destroy", "shutdown"]
    assert handlers == {2: "old_int", 15: "old_term"}


def test_runtime_callback_errors_are_not_misreported_as_clean_shutdown():
    events, handlers = [], {2: None, 15: None}

    def install(sig, handler):
        previous = handlers[sig]
        handlers[sig] = handler
        return previous

    def fail(*args, **kwargs):
        raise RuntimeError("real callback error")

    main_node = next(node for node in ast.parse(SOURCE.read_text()).body
                     if isinstance(node, ast.FunctionDef) and node.name == "main")
    namespace = {
        "signal": SimpleNamespace(SIGINT=2, SIGTERM=15, signal=install),
        "SignalHandlerOptions": SimpleNamespace(NO="disabled"),
        "SnakeWaypointRunner": lambda: SimpleNamespace(destroy_node=lambda: events.append("destroy")),
        "rclpy": SimpleNamespace(init=lambda **kwargs: None, ok=lambda: True, spin_once=fail,
                                  try_shutdown=lambda: events.append("shutdown")),
    }
    exec(compile(ast.Module(body=[main_node], type_ignores=[]), str(SOURCE), "exec"), namespace)
    with pytest.raises(RuntimeError, match="real callback error"):
        namespace["main"]()
    assert events == ["destroy", "shutdown"]
    assert handlers == {2: None, 15: None}
