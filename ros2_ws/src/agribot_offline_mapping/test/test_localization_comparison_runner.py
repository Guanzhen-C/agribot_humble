import importlib.util
from pathlib import Path

import pytest
import yaml


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "run_localization_comparison.py"
)
SPEC = importlib.util.spec_from_file_location(
    "run_localization_comparison", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_georeference(path):
    path.write_text(
        yaml.safe_dump(
            {
                "reference": {
                    "latitude_deg": 39.9777964102,
                    "longitude_deg": 116.3258690118,
                    "altitude_m": 41.6511,
                }
            }
        ),
        encoding="utf-8",
    )


def test_matching_georeference_is_inferred_from_comparison_output(tmp_path):
    output = tmp_path / "map_casia_comparison"
    georeference = tmp_path / "map_casia_georeference.yaml"
    write_georeference(georeference)
    arguments = MODULE.parse_arguments(
        [str(tmp_path / "bag"), str(output), "--estimator", "kf_gins"]
    )

    result = MODULE.resolve_kf_reference(arguments)

    assert result[0] == georeference
    assert result[1:] == pytest.approx(
        (39.9777964102, 116.3258690118, 41.6511)
    )


def test_missing_inferred_georeference_is_rejected(tmp_path):
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "map_casia_comparison"),
            "--estimator",
            "kf_gins",
        ]
    )

    with pytest.raises(MODULE.ComparisonError, match="georeference file not found"):
        MODULE.resolve_kf_reference(arguments)


def test_nonstandard_output_keeps_legacy_automatic_reference(tmp_path):
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "standalone_result"),
            "--estimator",
            "kf_gins",
        ]
    )

    assert MODULE.resolve_kf_reference(arguments) is None


def test_default_playback_rate_prioritizes_complete_estimator_output(tmp_path):
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "standalone_result"),
            "--estimator",
            "fastlivo",
        ]
    )

    assert arguments.playback_rate == pytest.approx(0.5)
    assert arguments.fastlivo_profile == "indoor"


def test_outdoor_fastlivo_profile_is_selectable(tmp_path):
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "standalone_result"),
            "--estimator",
            "fastlivo",
            "--fastlivo-profile",
            "outdoor",
        ]
    )

    assert arguments.fastlivo_profile == "outdoor"


def test_fastlivo_inputs_and_output_are_part_of_the_comparison():
    assert (
        "/camera/rgb/image_raw"
        in MODULE.ESTIMATOR_INPUT_TOPICS["fastlivo"]
    )
    assert MODULE.FASTLIVO_TOPIC == "/comparison/fastlivo/odometry"


def test_each_replay_requires_exactly_one_estimator(tmp_path):
    with pytest.raises(SystemExit):
        MODULE.parse_arguments(
            [str(tmp_path / "bag"), str(tmp_path / "output")]
        )

    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "output"),
            "--estimator",
            "fastlio",
        ]
    )
    assert arguments.estimator == "fastlio"
    assert MODULE.ESTIMATOR_INPUT_TOPICS["fastlio"] == (
        "/lidar/points",
        "/imu/data",
    )


def test_differential_profile_layers_measured_geometry_after_shared_tuning(tmp_path):
    arguments = MODULE.parse_arguments(
        [
            str(tmp_path / "bag"),
            str(tmp_path / "output"),
            "--estimator",
            "fastlivo",
            "--vehicle-profile",
            "differential",
        ]
    )
    package_root = SCRIPT.parents[2]
    hardware = package_root / "agribot_hardware_bringup"
    fastlivo = package_root / "FAST-LIVO2"

    configs = MODULE.resolve_estimator_configs(arguments, hardware, fastlivo)

    assert configs["fastlio"] == (
        hardware / "differential" / "config" / "fast_lio_c16.yaml"
    )
    assert configs["bridge"] == (
        hardware / "differential" / "config" / "fastlio_bridge.yaml"
    )
    assert configs["kf_gins"][-1].name == "kf_gins_n300pro.yaml"
    assert configs["kf_gins"][-1].parent.name == "config"
    assert configs["fastlivo"][0].name == "agribot_c16_astra.yaml"
    assert any(
        path.name == "fastlivo_sensor_calibration.yaml"
        for path in configs["fastlivo"]
    )
    assert not any(
        path.name == "agribot_c16_astra_outdoor.yaml"
        for path in configs["fastlivo"]
    )
def test_current_hikrobot_camera_and_outdoor_override_are_used():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "fastlivo_hikrobot_mv_cu013.yaml" in source
    assert "agribot_c16_astra_outdoor.yaml" in source
    assert "agribot_astra_640.yaml" not in source


def test_comparison_does_not_run_robot_localization():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "robot_localization" not in source
