"""Projection helpers for the symmetric four-camera Agribot rig."""

import math

import numpy as np


CAMERA_NAMES = ("front", "left", "rear", "right")
CAMERA_YAWS = {
    "front": 0.0,
    "left": math.pi / 2.0,
    "rear": math.pi,
    "right": -math.pi / 2.0,
}


def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64
    )


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
    )


def quaternion_matrix(x, y, z, w):
    norm = x * x + y * y + z * z + w * w
    if norm < 1.0e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


def make_bev_ground_grid(extent_m, resolution_m):
    size = int(round(2.0 * extent_m / resolution_m))
    if size <= 0:
        raise ValueError("BEV grid size must be positive")
    rows, columns = np.indices((size, size), dtype=np.float64)
    # BEV image convention: top is vehicle-forward, left is vehicle-left.
    grid_x = extent_m - (rows + 0.5) * resolution_m
    grid_y = extent_m - (columns + 0.5) * resolution_m
    return grid_x, grid_y


def make_occupancy_valid_mask(extent_m, resolution_m, min_range_m):
    size = int(round(2.0 * extent_m / resolution_m))
    rows, columns = np.indices((size, size), dtype=np.float64)
    grid_x = -extent_m + (columns + 0.5) * resolution_m
    grid_y = -extent_m + (rows + 0.5) * resolution_m
    ground_range = np.hypot(grid_x, grid_y)
    return (ground_range >= min_range_m) & (ground_range <= extent_m)


def make_stereographic_map(
    camera_name,
    image_width,
    image_height,
    grid_x,
    grid_y,
    camera_height_m,
    camera_radius_m,
    camera_pitch_rad,
    horizontal_fov_rad,
):
    if camera_name not in CAMERA_YAWS:
        raise ValueError(f"unknown camera name: {camera_name}")
    if not 0.0 < horizontal_fov_rad < 2.0 * math.pi:
        raise ValueError("horizontal FOV must be in (0, 2*pi)")

    yaw = CAMERA_YAWS[camera_name]
    camera_position = np.array(
        [
            camera_radius_m * math.cos(yaw),
            camera_radius_m * math.sin(yaw),
            camera_height_m,
        ],
        dtype=np.float64,
    )

    # REP-103 optical axes expressed in camera-link coordinates:
    # x right -> -link y, y down -> -link z, z forward -> link x.
    optical_to_link = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    optical_to_base = (
        rotation_z(yaw) @ rotation_y(camera_pitch_rad) @ optical_to_link
    )

    ground_points = np.stack(
        (grid_x, grid_y, np.zeros_like(grid_x)), axis=-1
    )
    rays_optical = (ground_points - camera_position) @ optical_to_base
    x_camera = rays_optical[..., 0]
    y_camera = rays_optical[..., 1]
    z_camera = rays_optical[..., 2]

    radial = np.hypot(x_camera, y_camera)
    theta = np.arctan2(radial, z_camera)
    half_fov = horizontal_fov_rad / 2.0
    maximum_radius = math.tan(half_fov / 2.0)
    image_radius = (image_width / 2.0) * np.tan(theta / 2.0) / maximum_radius
    scale = np.divide(
        image_radius,
        radial,
        out=np.zeros_like(image_radius),
        where=radial > 1.0e-8,
    )
    map_x = image_width / 2.0 + scale * x_camera
    map_y = image_height / 2.0 + scale * y_camera

    distance = np.sqrt(np.sum(rays_optical * rays_optical, axis=-1))
    cosine = np.divide(
        z_camera,
        distance,
        out=np.zeros_like(z_camera),
        where=distance > 1.0e-8,
    )
    valid = (
        (z_camera > 0.0)
        & (theta <= half_fov)
        & (map_x >= 0.0)
        & (map_x < image_width - 1.0)
        & (map_y >= 0.0)
        & (map_y < image_height - 1.0)
    )
    weight = np.where(valid, np.maximum(cosine, 0.0) ** 4 + 1.0e-3, 0.0)
    return map_x.astype(np.float32), map_y.astype(np.float32), weight
