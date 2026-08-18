#!/usr/bin/env python3

"""Export Agribot ROS 2 recordings as an OpenGraph/SemanticKITTI sequence."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import shutil
import sqlite3
import sys

import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import yaml


NANOSECONDS = 1_000_000_000


class ExportError(RuntimeError):
    pass


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / NANOSECONDS


def find_database(bag_directory):
    databases = sorted(bag_directory.glob("*.db3"))
    if len(databases) != 1:
        raise ExportError(
            f"expected exactly one sqlite3 bag file in {bag_directory}, found {len(databases)}"
        )
    return databases[0]


def topic_id(connection, name, expected_type):
    row = connection.execute(
        "SELECT id, type FROM topics WHERE name = ?", (name,)
    ).fetchone()
    if row is None:
        raise ExportError(f"bag does not contain required topic: {name}")
    if row[1] != expected_type:
        raise ExportError(
            f"topic {name} has type {row[1]}, expected {expected_type}"
        )
    return row[0]


def read_binary_pcd(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise ExportError(f"PCD header is incomplete: {path}")
            line = raw.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break
        if header["DATA"] != ["binary"]:
            raise ExportError("LIO-SAM transformations PCD must use binary DATA")
        fields = header.get("FIELDS", [])
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ExportError("PCD field metadata lengths do not match")
        formats = []
        for field_type, size, count in zip(types, sizes, counts):
            if count != 1:
                raise ExportError("array-valued PCD fields are not supported")
            type_map = {
                ("F", 4): "<f4",
                ("F", 8): "<f8",
                ("U", 1): "u1",
                ("U", 2): "<u2",
                ("U", 4): "<u4",
                ("I", 1): "i1",
                ("I", 2): "<i2",
                ("I", 4): "<i4",
            }
            try:
                formats.append(type_map[(field_type, size)])
            except KeyError as error:
                raise ExportError(
                    f"unsupported PCD field representation: {field_type}{size}"
                ) from error
        point_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data = np.fromfile(
            stream, dtype=np.dtype({"names": fields, "formats": formats}), count=point_count
        )
    if len(data) != point_count:
        raise ExportError(f"PCD contains {len(data)} points, expected {point_count}")
    return data


def rpy_matrix(roll, pitch, yaw):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def matrix_quaternion(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    return quaternion / np.linalg.norm(quaternion)


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def slerp(left, right, ratio):
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = left + ratio * (right - left)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    denominator = math.sin(angle)
    return (
        math.sin((1.0 - ratio) * angle) / denominator * left
        + math.sin(ratio * angle) / denominator * right
    )


class LioSamTrajectory:
    def __init__(self, transformations_pcd):
        data = read_binary_pcd(transformations_pcd)
        required = {"x", "y", "z", "roll", "pitch", "yaw", "time"}
        missing = required.difference(data.dtype.names or ())
        if missing:
            raise ExportError(f"LIO-SAM PCD is missing fields: {sorted(missing)}")
        self.times = np.asarray(data["time"], dtype=np.float64)
        self.positions = np.column_stack((data["x"], data["y"], data["z"])).astype(
            np.float64
        )
        self.quaternions = np.stack(
            [
                matrix_quaternion(rpy_matrix(roll, pitch, yaw))
                for roll, pitch, yaw in zip(data["roll"], data["pitch"], data["yaw"])
            ]
        )
        if len(self.times) < 2 or np.any(np.diff(self.times) <= 0.0):
            raise ExportError("LIO-SAM key-pose times must be strictly increasing")

    def pose(self, timestamp):
        if timestamp < self.times[0] or timestamp > self.times[-1]:
            raise ExportError("requested timestamp is outside the final LIO-SAM trajectory")
        upper = int(np.searchsorted(self.times, timestamp, side="right"))
        if upper == 0:
            upper = 1
        if upper == len(self.times):
            upper = len(self.times) - 1
        lower = upper - 1
        duration = self.times[upper] - self.times[lower]
        ratio = 0.0 if duration <= 0.0 else (timestamp - self.times[lower]) / duration
        translation = (1.0 - ratio) * self.positions[lower] + ratio * self.positions[upper]
        rotation = quaternion_matrix(
            slerp(self.quaternions[lower], self.quaternions[upper], ratio)
        )
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        return transform, lower, upper


def load_fastlivo_calibration(path):
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    parameters = document["/**"]["ros__parameters"]
    extrinsics = parameters["extrin_calib"]
    offsets = parameters["time_offset"]
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(extrinsics["Rcl"], dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(extrinsics["Pcl"], dtype=np.float64)
    return transform, float(offsets["img_time_offset"])


def point_array(message):
    if message.is_bigendian:
        raise ExportError("big-endian PointCloud2 input is unsupported")
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "z", "intensity", "time")
    missing = set(required).difference(fields)
    if missing:
        raise ExportError(f"point cloud is missing fields: {sorted(missing)}")
    for name in ("x", "y", "z", "intensity", "time"):
        if fields[name].datatype != 7 or fields[name].count != 1:
            raise ExportError(f"point field {name} must be scalar FLOAT32")
    dtype = np.dtype(
        {
            "names": required,
            "formats": ["<f4"] * len(required),
            "offsets": [fields[name].offset for name in required],
            "itemsize": message.point_step,
        }
    )
    count = int(message.width) * int(message.height)
    structured = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack(
        (structured["x"], structured["y"], structured["z"], structured["intensity"])
    ).astype(np.float32, copy=False)
    relative_times = np.asarray(structured["time"], dtype=np.float64)
    valid = np.isfinite(points).all(axis=1) & np.isfinite(relative_times)
    if not np.any(valid):
        raise ExportError("point cloud has no finite points")
    return np.ascontiguousarray(points[valid]), relative_times[valid]


def camera_parameters(message):
    intrinsic = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(message.d, dtype=np.float64)
    rectification = np.asarray(message.r, dtype=np.float64).reshape(3, 3)
    projection = np.asarray(message.p, dtype=np.float64).reshape(3, 4)
    return intrinsic, distortion, rectification, projection


def write_calibration(path, projection, camera_from_lidar):
    zero_projection = np.zeros((3, 4), dtype=np.float64)

    def line(name, matrix):
        return name + ": " + " ".join(f"{value:.12g}" for value in matrix.reshape(-1))

    path.write_text(
        "\n".join(
            [
                line("P0", zero_projection),
                line("P1", zero_projection),
                line("P2", projection),
                line("P3", zero_projection),
                line("Tr", camera_from_lidar[:3, :]),
            ]
        )
        + "\n",
        encoding="ascii",
    )


def project_overlay(image, points, projection, camera_from_lidar):
    homogeneous = np.column_stack((points[:, :3], np.ones(len(points), dtype=np.float32)))
    camera_points = (camera_from_lidar @ homogeneous.T).T
    positive = camera_points[:, 2] > 0.1
    camera_points = camera_points[positive]
    if not len(camera_points):
        return image.copy(), 0
    pixels_h = (projection @ camera_points.T).T
    pixels = pixels_h[:, :2] / pixels_h[:, 2:3]
    height, width = image.shape[:2]
    inside = (
        (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < height)
    )
    pixels = pixels[inside].astype(np.int32)
    depths = camera_points[inside, 2]
    overlay = image.copy()
    if len(pixels):
        normalized = np.clip(depths / 30.0, 0.0, 1.0)
        colors = cv2.applyColorMap(
            np.asarray((255.0 * (1.0 - normalized)), dtype=np.uint8), cv2.COLORMAP_TURBO
        ).reshape(-1, 3)
        for (u, v), color in zip(pixels[::2], colors[::2]):
            cv2.circle(overlay, (int(u), int(v)), 1, tuple(int(value) for value in color), -1)
    return cv2.addWeighted(image, 0.55, overlay, 0.45, 0.0), len(pixels)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Export synchronized camera, C16 and final LIO-SAM poses for OpenGraph"
    )
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("transformations_pcd", type=Path)
    parser.add_argument("output_sequence", type=Path)
    parser.add_argument("--lidar-topic", default="/lidar/points")
    parser.add_argument("--image-topic", default="/camera/rgb/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/rgb/camera_info")
    parser.add_argument(
        "--fastlivo-config",
        type=Path,
        default=Path("src/FAST-LIVO2/config/agribot_c16_astra.yaml"),
    )
    parser.add_argument("--lidar-step", type=int, default=1)
    parser.add_argument(
        "--semantic-stride",
        type=int,
        default=1,
        help=(
            "materialize an image every N exported scans while retaining every "
            "point cloud and pose; set OpenGraph's stride to the same value"
        ),
    )
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--maximum-image-delta", type=float, default=0.06)
    parser.add_argument("--debug-overlays", type=int, default=5)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def validate_arguments(arguments):
    if not arguments.raw_bag.is_dir():
        raise ExportError(f"raw bag directory does not exist: {arguments.raw_bag}")
    if not arguments.transformations_pcd.is_file():
        raise ExportError(
            f"LIO-SAM transformations file does not exist: {arguments.transformations_pcd}"
        )
    if not arguments.fastlivo_config.is_file():
        raise ExportError(f"calibration config does not exist: {arguments.fastlivo_config}")
    if (
        arguments.lidar_step < 1
        or arguments.semantic_stride < 1
        or arguments.max_frames < 0
    ):
        raise ExportError(
            "lidar-step and semantic-stride must be positive and max-frames "
            "must be nonnegative"
        )
    if arguments.maximum_image_delta <= 0.0 or arguments.debug_overlays < 0:
        raise ExportError("image delta must be positive and debug-overlays must be nonnegative")


def main(argv=None):
    arguments = parse_arguments(argv)
    validate_arguments(arguments)
    output = arguments.output_sequence.resolve()
    partial = output.parent / f".{output.name}.partial"
    if output.exists() and not arguments.force:
        raise ExportError(f"output already exists; choose another path or pass --force: {output}")
    if arguments.force:
        shutil.rmtree(output, ignore_errors=True)
        shutil.rmtree(partial, ignore_errors=True)
    elif partial.exists():
        raise ExportError(f"partial output already exists; remove it or pass --force: {partial}")
    partial.mkdir(parents=True)
    image_directory = partial / "image_2"
    cloud_directory = partial / "velodyne"
    overlay_directory = partial / "projection_debug"
    image_directory.mkdir()
    cloud_directory.mkdir()
    if arguments.debug_overlays:
        overlay_directory.mkdir()
    unused_image = partial / "unused_image.png"
    if arguments.semantic_stride > 1:
        if not cv2.imwrite(str(unused_image), np.zeros((1, 1, 3), dtype=np.uint8)):
            raise ExportError(f"failed to write unused image placeholder: {unused_image}")

    trajectory = LioSamTrajectory(arguments.transformations_pcd.resolve())
    camera_from_lidar, image_time_offset = load_fastlivo_calibration(
        arguments.fastlivo_config.resolve()
    )
    lidar_from_camera = np.linalg.inv(camera_from_lidar)
    database = find_database(arguments.raw_bag.resolve())
    connection = sqlite3.connect(database)
    lidar_id = topic_id(connection, arguments.lidar_topic, "sensor_msgs/msg/PointCloud2")
    image_id = topic_id(connection, arguments.image_topic, "sensor_msgs/msg/Image")
    camera_info_id = topic_id(
        connection, arguments.camera_info_topic, "sensor_msgs/msg/CameraInfo"
    )

    info_row = connection.execute(
        "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 1",
        (camera_info_id,),
    ).fetchone()
    if info_row is None:
        raise ExportError("camera info topic contains no messages")
    info = deserialize_message(info_row[0], CameraInfo)
    intrinsic, distortion, rectification, projection = camera_parameters(info)
    image_size = (int(info.width), int(info.height))
    rectification_map = cv2.initUndistortRectifyMap(
        intrinsic,
        distortion,
        rectification,
        projection[:, :3],
        image_size,
        cv2.CV_32FC1,
    )
    write_calibration(partial / "calib.txt", projection, camera_from_lidar)

    print("Indexing camera timestamps ...", flush=True)
    image_index = []
    cursor = connection.execute(
        "SELECT id, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (image_id,)
    )
    for row_id, serialized in cursor:
        image = deserialize_message(serialized, Image)
        raw_time = stamp_seconds(image)
        corrected_time = raw_time - image_time_offset
        image_index.append((corrected_time, row_id, raw_time))
    if not image_index:
        raise ExportError("image topic contains no messages")
    image_index.sort(key=lambda item: item[0])
    image_times = [item[0] for item in image_index]

    bridge = CvBridge()
    poses_camera = []
    poses_lidar = []
    associations = []
    exported = 0
    considered = 0
    overlay_count = 0
    skipped_image_delta = 0
    skipped_camera_discontinuity = 0
    lidar_cursor = connection.execute(
        "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp", (lidar_id,)
    )
    for (serialized,) in lidar_cursor:
        cloud = deserialize_message(serialized, PointCloud2)
        points, relative_times = point_array(cloud)
        scan_end = stamp_seconds(cloud)
        minimum_time = float(np.min(relative_times))
        maximum_time = float(np.max(relative_times))
        scan_start = scan_end + minimum_time - maximum_time
        scan_time = 0.5 * (scan_start + scan_end)
        if scan_time < trajectory.times[0] or scan_time > trajectory.times[-1]:
            continue
        if considered % arguments.lidar_step:
            considered += 1
            continue
        considered += 1
        insertion = bisect.bisect_left(image_times, scan_time)
        candidate_indices = []
        if insertion < len(image_index):
            candidate_indices.append(insertion)
        if insertion:
            candidate_indices.append(insertion - 1)
        image_index_position = min(
            candidate_indices, key=lambda index: abs(image_index[index][0] - scan_time)
        )
        corrected_image_time, image_row_id, raw_image_time = image_index[
            image_index_position
        ]
        neighboring_intervals = []
        if image_index_position:
            neighboring_intervals.append(
                corrected_image_time - image_index[image_index_position - 1][0]
            )
        if image_index_position + 1 < len(image_index):
            neighboring_intervals.append(
                image_index[image_index_position + 1][0] - corrected_image_time
            )
        if not neighboring_intervals or min(neighboring_intervals) > 0.1:
            skipped_camera_discontinuity += 1
            continue
        image_delta = corrected_image_time - scan_time
        if abs(image_delta) > arguments.maximum_image_delta:
            skipped_image_delta += 1
            continue
        map_from_lidar, lower, upper = trajectory.pose(scan_time)
        map_from_camera = map_from_lidar @ lidar_from_camera
        name = f"{exported:06d}"
        image_path = image_directory / f"{name}.png"
        semantic_frame = exported % arguments.semantic_stride == 0
        rectified = None
        if semantic_frame:
            image_row = connection.execute(
                "SELECT data FROM messages WHERE id = ?", (image_row_id,)
            ).fetchone()
            image_message = deserialize_message(image_row[0], Image)
            bgr = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            rectified = cv2.remap(
                bgr,
                rectification_map[0],
                rectification_map[1],
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            if not cv2.imwrite(
                str(image_path),
                rectified,
                [cv2.IMWRITE_PNG_COMPRESSION, 1],
            ):
                raise ExportError(f"failed to write image frame {name}")
        else:
            image_path.symlink_to("../unused_image.png")
        points.astype("<f4", copy=False).tofile(cloud_directory / f"{name}.bin")
        poses_camera.append(map_from_camera[:3, :].reshape(-1))
        poses_lidar.append(map_from_lidar[:3, :].reshape(-1))
        in_image = ""
        if semantic_frame and overlay_count < arguments.debug_overlays:
            overlay, in_image = project_overlay(
                rectified, points, projection, camera_from_lidar
            )
            cv2.imwrite(str(overlay_directory / f"{name}.png"), overlay)
            overlay_count += 1
        associations.append(
            [
                name,
                f"{scan_end:.9f}",
                f"{scan_start:.9f}",
                f"{scan_time:.9f}",
                f"{raw_image_time:.9f}",
                f"{corrected_image_time:.9f}",
                f"{image_delta:.9f}",
                lower,
                upper,
                len(points),
                in_image,
            ]
        )
        exported += 1
        if exported % 100 == 0:
            print(f"Exported {exported} synchronized frames ...", flush=True)
        if arguments.max_frames and exported >= arguments.max_frames:
            break
    connection.close()
    if not exported:
        raise ExportError("no synchronized frames fell inside the final LIO-SAM trajectory")

    np.savetxt(partial / "poses.txt", np.asarray(poses_camera), fmt="%.12g")
    np.savetxt(partial / "poses_lidar.txt", np.asarray(poses_lidar), fmt="%.12g")
    with (partial / "associations.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "frame",
                "raw_lidar_stamp",
                "scan_start",
                "scan_midpoint",
                "raw_image_stamp",
                "corrected_image_stamp",
                "image_minus_lidar_seconds",
                "lio_lower_key",
                "lio_upper_key",
                "point_count",
                "projected_point_count",
            ]
        )
        writer.writerows(associations)
    manifest = {
        "schema_version": 1,
        "source_raw_bag": str(arguments.raw_bag.resolve()),
        "pose_source": str(arguments.transformations_pcd.resolve()),
        "pose_policy": "final_optimized_lio_sam_keyposes_slerp",
        "frame_count": exported,
        "lidar_step": arguments.lidar_step,
        "semantic_stride": arguments.semantic_stride,
        "image_time_offset_seconds": image_time_offset,
        "image_time_correction": "corrected = raw - image_time_offset",
        "scan_pose_time": "midpoint",
        "camera_from_lidar": camera_from_lidar.tolist(),
        "rectified_projection": projection.tolist(),
        "maximum_absolute_image_delta_seconds": max(
            abs(float(row[6])) for row in associations
        ),
        "skipped_for_image_delta": skipped_image_delta,
        "skipped_for_camera_discontinuity": skipped_camera_discontinuity,
        "lio_sam_time_range": [float(trajectory.times[0]), float(trajectory.times[-1])],
    }
    (partial / "agribot_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    (partial / "times.txt").write_text(
        "\n".join(row[3] for row in associations) + "\n", encoding="ascii"
    )
    (partial / "README.txt").write_text(
        "poses.txt contains map<-camera_optical transforms. OpenGraph's dataset loader "
        "right-multiplies each pose by Tr (camera_optical<-lidar), yielding the final "
        "optimized LIO-SAM map<-lidar pose used to transform each scan. When "
        "semantic_stride is greater than one, unused image indices are placeholder "
        "links; configure OpenGraph with the same stride.\n",
        encoding="ascii",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    partial.replace(output)
    print(json.dumps(manifest, indent=2, ensure_ascii=False), flush=True)
    print(f"OpenGraph sequence exported to {output}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ExportError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
