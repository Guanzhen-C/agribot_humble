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
