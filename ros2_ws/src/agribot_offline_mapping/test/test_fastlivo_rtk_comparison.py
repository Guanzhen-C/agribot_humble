import importlib.util
from pathlib import Path


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
    assert arguments.visual_voxel_size == 0.10
    assert arguments.visual_sync_tolerance_sec == 0.12
    assert "fastlivo_dense_map:={'true' if visual_map else 'false'}" in source
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
    assert "PointXYZRGB" in source
    assert "fastlivo_rtk_visual_mapper" in cmake
