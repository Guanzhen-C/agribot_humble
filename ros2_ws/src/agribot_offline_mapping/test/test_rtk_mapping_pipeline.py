import importlib.util
from pathlib import Path
from types import SimpleNamespace

import yaml


SCRIPT = Path(__file__).parents[1] / "scripts" / "run_rtk_mapping_pipeline.py"
SPEC = importlib.util.spec_from_file_location("run_rtk_mapping_pipeline", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_output_paths_share_one_map_prefix(tmp_path):
    map_base = tmp_path / "map_test"
    paths = MODULE.output_paths(map_base)

    assert paths["pcd"] == tmp_path / "map_test.pcd"
    assert paths["pgm"] == tmp_path / "map_test.pgm"
    assert paths["yaml"] == tmp_path / "map_test.yaml"
    assert paths["georeference"] == tmp_path / "map_test_georeference.yaml"
    assert paths["manifest"] == tmp_path / "map_test_manifest.yaml"
    assert paths["result_bag"] == tmp_path / "map_test_result"


def test_service_result_requires_explicit_success():
    assert MODULE.service_succeeded("success: true\nmessage: saved")
    assert MODULE.service_succeeded("Response(success=True, message='saved')")
    assert not MODULE.service_succeeded("success: false")


def test_indoor_playback_excludes_recorded_tf_and_rtk():
    topics = MODULE.playback_topics(without_rtk=True)

    assert topics == ("/lidar/points", "/imu/data")
    assert "/tf" not in topics
    assert "/rtk/fix" not in topics


def test_rtk_playback_still_excludes_recorded_tf():
    topics = MODULE.playback_topics(without_rtk=False)

    assert "/rtk/fix" in topics
    assert "/rtk/heading_with_covariance" in topics
    assert "/tf" not in topics


def test_manifest_records_robust_horizontal_and_gravity_constraints(tmp_path):
    map_base = tmp_path / "map_test"
    paths = MODULE.output_paths(map_base)
    arguments = MODULE.parse_arguments([str(tmp_path / "bag"), str(map_base)])
    MODULE.write_manifest(
        paths["manifest"], tmp_path / "bag", map_base,
        tmp_path / "work" / "map_test", paths, arguments
    )

    document = yaml.safe_load(paths["manifest"].read_text(encoding="utf-8"))
    assert document["pipeline"] == "lio_sam_rtk_gravity_robust_xy_v2"
    assert document["rtk_factor"]["horizontal_variance_floor_m2"] == 0.01
    assert document["rtk_factor"]["use_gps_elevation"] is False
    assert document["rtk_factor"]["huber_delta"] == 1.345
    assert document["map_leveling"] == {
        "gravity_attitude_factor": True,
        "gravity_attitude_sigma_deg": 1.0,
        "gravity_attitude_huber_delta": 1.345,
        "initial_roll_pitch_sigma_deg": 0.5,
        "initial_z_sigma_m": 0.1,
    }
    assert document["result_bag"] == str(paths["result_bag"])
    assert document["point_exclusion"]["rear_person_region_base_link"] == {
        "minimum_x_m": -4.0,
        "maximum_x_m": -0.1275,
        "half_width_m": 0.60,
    }
    assert document["point_exclusion"]["rtk_antenna_boxes_base_link"][
        "left_center_xyz_m"
    ] == [0.1425, 0.2952585, 0.78476]


def test_without_rtk_manifest_omits_georeference_artifact(tmp_path):
    map_base = tmp_path / "map_indoor"
    paths = MODULE.output_paths(map_base)
    arguments = MODULE.parse_arguments(
        [str(tmp_path / "bag"), str(map_base), "--without-rtk"]
    )

    MODULE.write_manifest(
        paths["manifest"], tmp_path / "bag", map_base,
        tmp_path / "work" / "map_indoor", paths, arguments
    )

    document = yaml.safe_load(paths["manifest"].read_text(encoding="utf-8"))
    assert document["pipeline"] == "lio_sam_gravity_indoor_v1"
    assert document["rtk_mode"] == "disabled"
    assert document["rtk_factor"]["enabled"] is False
    assert "georeference" not in document["artifacts"]


def test_differential_profile_records_measured_mounts_and_filters(tmp_path):
    map_base = tmp_path / "map_diff"
    paths = MODULE.output_paths(map_base)
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(map_base),
            "--vehicle-profile",
            "differential",
        ]
    )
    profile_paths = MODULE.resolve_profile_paths("differential")

    MODULE.write_manifest(
        paths["manifest"],
        tmp_path / "bag",
        map_base,
        tmp_path / "work" / "map_diff",
        paths,
        arguments,
        profile_paths,
    )

    document = yaml.safe_load(paths["manifest"].read_text(encoding="utf-8"))
    assert document["vehicle_profile"] == "differential"
    assert document["input_stamp_is_scan_end"] is False
    assert document["sensor_mounts"]["imu"]["xyz"] == [0.0, 0.0, 0.64]
    assert document["sensor_mounts"]["lidar"]["xyz"] == [0.47, 0.0, 0.91]
    assert document["sensor_mounts"]["rtk"]["xyz"] == [-0.48, 0.35, 0.748]
    assert document["rtk_factor"]["antenna_to_lidar_flu_m"] == [
        0.95,
        -0.35,
        0.162,
    ]
    assert document["point_exclusion"]["rear_person_region_base_link"][
        "maximum_x_m"
    ] == -0.60


def test_validate_inputs_rejects_unsafe_map_name(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    arguments = SimpleNamespace(
        bag=bag,
        map_base=tmp_path / "map has spaces",
        playback_rate=0.5,
        settle_seconds=10.0,
        save_resolution=0.1,
        domain_id=71,
    )

    try:
        MODULE.validate_inputs(arguments)
    except MODULE.PipelineError as error:
        assert "map name" in str(error)
    else:
        raise AssertionError("unsafe map name was accepted")
