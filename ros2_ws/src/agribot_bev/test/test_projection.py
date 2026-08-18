import math

import numpy as np

from agribot_bev.projection import (
    CAMERA_NAMES,
    make_bev_ground_grid,
    make_occupancy_valid_mask,
    make_stereographic_map,
    quaternion_matrix,
)


def test_quaternion_matrix_identity():
    np.testing.assert_allclose(quaternion_matrix(0.0, 0.0, 0.0, 1.0), np.eye(3))


def test_front_optical_axis_maps_to_image_center():
    pitch = math.radians(25.0)
    ground_distance = 1.0 / math.tan(pitch)
    grid_x = np.array([[0.09 + ground_distance]])
    grid_y = np.array([[0.0]])
    map_x, map_y, weight = make_stereographic_map(
        "front",
        640,
        480,
        grid_x,
        grid_y,
        camera_height_m=1.0,
        camera_radius_m=0.09,
        camera_pitch_rad=pitch,
        horizontal_fov_rad=math.pi,
    )
    np.testing.assert_allclose(map_x, 320.0, atol=1.0e-4)
    np.testing.assert_allclose(map_y, 240.0, atol=1.0e-4)
    assert weight.item() > 0.9


def test_four_cameras_cover_navigation_disc():
    grid_x, grid_y = make_bev_ground_grid(10.0, 0.1)
    total_weight = np.zeros_like(grid_x)
    for name in CAMERA_NAMES:
        _, _, weight = make_stereographic_map(
            name,
            640,
            480,
            grid_x,
            grid_y,
            camera_height_m=1.0,
            camera_radius_m=0.09,
            camera_pitch_rad=math.radians(25.0),
            horizontal_fov_rad=math.pi,
        )
        total_weight += weight
    expected = make_occupancy_valid_mask(10.0, 0.1, 0.3)
    # Occupancy layout and BEV image layout are rotated/flipped, but their
    # coverage ratio over a symmetric circle must agree.
    assert np.mean(total_weight > 0.0) >= np.mean(expected) - 0.01
