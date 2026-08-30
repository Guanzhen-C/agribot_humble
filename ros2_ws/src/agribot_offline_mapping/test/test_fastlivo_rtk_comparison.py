import importlib.util
from pathlib import Path

import yaml


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "run_fastlivo_rtk_comparison.py"
)
SPEC = importlib.util.spec_from_file_location(
    "run_fastlivo_rtk_comparison", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_differential_production_launch_is_selectable(tmp_path):
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "map_diff"),
            str(tmp_path / "result"),
            "--vehicle-profile",
            "differential",
        ]
    )

    assert arguments.vehicle_profile == "differential"
    assert MODULE.VEHICLE_LAUNCHES[arguments.vehicle_profile] == (
        "differential_fastlivo_rtk_localization.launch.py"
    )


def test_fusion_replay_uses_only_raw_inputs_and_records_required_outputs():
    assert "/lidar/points" in MODULE.INPUT_TOPICS
    assert "/camera/rgb/image_raw" in MODULE.INPUT_TOPICS
    assert "/fastlivo/odometry" not in MODULE.INPUT_TOPICS
    assert "/fastlivo_rtk/odometry" in MODULE.OUTPUT_TOPICS
    assert "/localization_pose" in MODULE.OUTPUT_TOPICS
    assert "/fastlivo_rtk/fixed_active" in MODULE.OUTPUT_TOPICS


def test_default_replay_rate_prioritizes_complete_fusion_output(tmp_path):
    arguments = MODULE.parse_arguments(
        [str(tmp_path / "bag"), str(tmp_path / "map"), str(tmp_path / "out")]
    )

    assert arguments.playback_rate == 0.5


def test_visual_map_reuses_production_fusion_with_dense_color_clouds(tmp_path):
    visual_map = tmp_path / "map_visual.pcd"
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "map"),
            str(tmp_path / "out"),
            "--visual-map",
            str(visual_map),
        ]
    )
    source = SCRIPT.read_text(encoding="utf-8")

    assert arguments.visual_map == visual_map
    assert arguments.visual_map_mode == "dense"
    assert arguments.visual_voxel_size == 0.10
    assert arguments.visual_sync_tolerance_sec == 0.12
    assert arguments.rviz is False
    assert "fastlivo_dense_map:={'true' if visual_map else 'false'}" in source
    assert "fastlivo_map_sliding_en:={'false' if visual_map else 'true'}" in source
    assert 'f"map_mode:={arguments.visual_map_mode}"' in source
    assert "fastlivo_rtk_visual_mapper" in source
    assert MODULE.VISUAL_MAP_SERVICE == "/fastlivo_rtk_visual_mapper/save"


def test_visual_pcd_point_count_is_validated(tmp_path):
    pcd = tmp_path / "visual.pcd"
    pcd.write_bytes(
        b"# .PCD v0.7\nFIELDS x y z rgb\nPOINTS 1234\nDATA binary\n"
    )

    assert MODULE.pcd_point_count(pcd) == 1234


def test_visual_mapper_applies_time_varying_fused_pose_correction():
    package_root = SCRIPT.parents[1]
    source = (
        package_root / "src/fastlivo_rtk_visual_mapper.cpp"
    ).read_text(encoding="utf-8")
    cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "fused_base * local_base.inverse()" in source
    assert 'result.header.frame_id = "map"' in source
    assert 'map_mode_ == "dense"' in source
    assert "dense_points_ += mapped_cloud" in source
    assert '"/fastlivo_rtk/dense_cloud"' in source
    assert "PointXYZRGB" in source
    assert "fastlivo_rtk_visual_mapper" in cmake


def test_mapping_rviz_shows_corrected_dense_cloud_and_camera():
    rviz = (
        SCRIPT.parents[1] / "rviz" / "fastlivo_rtk_dense_mapping.rviz"
    ).read_text(encoding="utf-8")

    assert "Value: /fastlivo_rtk/dense_cloud" in rviz
    assert "Value: /rgb_img" in rviz
    assert "Value: /fastlivo_rtk/path" in rviz


def test_mapping_mode_matches_official_fastlivo2_dense_settings():
    official = yaml.safe_load(
        (
            SCRIPT.parents[2] / "FAST-LIVO2" / "config" / "avia.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    runner = SCRIPT.read_text(encoding="utf-8")

    assert official["publish"]["dense_map_en"] is True
    assert official["publish"]["pub_scan_num"] == 1
    assert official["local_map"]["map_sliding_en"] is False
    assert "fastlivo_dense_map:={'true' if visual_map else 'false'}" in runner
    assert "fastlivo_map_sliding_en:={'false' if visual_map else 'true'}" in runner
