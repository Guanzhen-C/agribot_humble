from agribot_hil_apple.mission_view import observed_path, planned_path, waypoint_markers
from pathlib import Path
import yaml


def test_saved_native_paths_keep_coordinates_and_orientation():
    message = planned_path([[3., 32.5, 1., 0.]])
    assert message.header.frame_id == 'map'
    pose = message.poses[0].pose
    assert (pose.position.x, pose.position.y) == (3., 32.5)
    assert (pose.orientation.z, pose.orientation.w) == (1., 0.)


def test_recorded_path_uses_fused_pose_rows_not_ground_truth():
    message = observed_path([[1., 3., 32.5, .1, 0., 0., 1., 0.]])
    assert message.poses[0].pose.position.y == 32.5
    assert message.poses[0].pose.orientation.z == 1.


def test_view_does_not_insert_point_four_or_home():
    markers = waypoint_markers({'fixed_route_order': [1, 2, 3],
        'fixed_route_points': [[41., 36., 0.], [41., 32.5, 3.14], [3., 32.5, 3.14]]})
    assert [marker.text for marker in markers.markers] == ['1', '2', '3']


def test_return_to_spawn_is_labelled_start_not_original_point_four():
    markers = waypoint_markers({'fixed_route_order': [1, 2, 3], 'finish_is_spawn': True,
        'spawn': [2., 36., .1325, 0.],
        'fixed_route_points': [[41., 36., 0.], [41., 32.5, 3.14], [3., 32.5, 3.14]]})
    assert [marker.text for marker in markers.markers] == ['1', '2', '3', 'Start']
    assert markers.markers[-1].pose.position.x == 2.
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/hil_navigation.rviz').read_text())
    displays = config['Visualization Manager']['Displays']
    marker = next(display for display in displays if display['Class'] == 'rviz_default_plugins/MarkerArray')
    assert marker['Topic']['Value'] == '/hil/visualization/waypoints'
    assert displays[0]['Topic']['Value'] == '/hil/visualization/full_map'
