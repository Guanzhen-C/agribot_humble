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
    arguments = MODULE.parse_arguments([str(tmp_path / "bag"), str(output)])

    result = MODULE.resolve_kf_reference(arguments)

    assert result[0] == georeference
    assert result[1:] == pytest.approx(
        (39.9777964102, 116.3258690118, 41.6511)
    )


def test_missing_inferred_georeference_is_rejected(tmp_path):
    arguments = MODULE.parse_arguments(
        [str(tmp_path / "bag"), str(tmp_path / "map_casia_comparison")]
    )

    with pytest.raises(MODULE.ComparisonError, match="georeference file not found"):
        MODULE.resolve_kf_reference(arguments)


def test_nonstandard_output_keeps_legacy_automatic_reference(tmp_path):
    arguments = MODULE.parse_arguments(
        [str(tmp_path / "bag"), str(tmp_path / "standalone_result")]
    )

    assert MODULE.resolve_kf_reference(arguments) is None


def test_default_playback_rate_prioritizes_complete_estimator_output(tmp_path):
    arguments = MODULE.parse_arguments(
        [str(tmp_path / "bag"), str(tmp_path / "standalone_result")]
    )

    assert arguments.playback_rate == pytest.approx(0.5)


def test_fastlivo_inputs_and_output_are_part_of_the_comparison():
    assert "/camera/rgb/image_raw" in MODULE.RAW_TOPICS
    assert MODULE.FASTLIVO_TOPIC == "/comparison/fastlivo/odometry"
