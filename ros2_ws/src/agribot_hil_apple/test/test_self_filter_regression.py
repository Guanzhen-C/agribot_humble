from pathlib import Path
import runpy

import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def test_measured_front_wheel_returns_are_removed_but_external_points_remain():
    source = Path(__file__).resolve().parents[2]/'agribot_vehicle_description/scripts/simulation_vehicle_cloud_filter.py'
    remove = runpy.run_path(str(source))['remove_self_returns']
    measured = [[.617249, .372518, .116345], [.539794, -.371388, .142337],
                [.550356, -.374367, .168914], [.380825, -.373814, .167562]]
    external = [[.8, 0., .2], [-.25, 0., .2], [.55, .41, .2], [.55, 0., .5]]
    message = point_cloud2.create_cloud_xyz32(Header(frame_id='base_link'), measured+external)
    original = remove(message, np.eye(3), np.zeros(3), [-.19, -.37, -.2], [.69, .37, .36])
    corrected = remove(message, np.eye(3), np.zeros(3), [-.19, -.40, -.2], [.69, .40, .36])
    assert original.width == 8
    assert corrected.width == len(external)
    points = point_cloud2.read_points(corrected, field_names=('x', 'y', 'z'))
    assert np.allclose(np.column_stack([points[k] for k in ('x', 'y', 'z')]), external)
