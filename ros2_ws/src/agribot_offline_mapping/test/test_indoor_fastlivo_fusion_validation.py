import importlib.util
from pathlib import Path

import pytest
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "run_indoor_fastlivo_fusion_validation.py"
)
SPEC = importlib.util.spec_from_file_location(
    "run_indoor_fastlivo_fusion_validation", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_bag_metadata(path, counts):
    path.mkdir()
    topics = [
        {
            "topic_metadata": {
                "name": topic,
                "type": "std_msgs/msg/String",
                "serialization_format": "cdr",
                "offered_qos_profiles": "",
            },
            "message_count": count,
        }
        for topic, count in counts.items()
    ]
    (path / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "relative_file_paths": [],
                    "topics_with_message_count": topics,
                }
            }
        ),
        encoding="utf-8",
    )


def make_arguments(tmp_path, counts):
    source = tmp_path / "source"
    write_bag_metadata(source, counts)
    map_base = tmp_path / "map_indoor"
    map_base.with_suffix(".pcd").write_bytes(b"pcd")
    map_base.with_suffix(".yaml").write_text("image: map.pgm\n")
    return MODULE.parse_arguments(
        [str(source), str(map_base), str(tmp_path / "output")]
    )


def test_indoor_validation_accepts_empty_rtk_input(tmp_path):
    counts = {topic: 1 for topic in MODULE.REQUIRED_INPUT_TOPICS}
    counts["/rtk/fix"] = 0
    arguments = make_arguments(tmp_path, counts)

    source, map_base, output = MODULE.validate_inputs(arguments)

    assert source.name == "source"
    assert map_base.name == "map_indoor"
    assert output.name == "output"


def test_indoor_validation_rejects_any_rtk_position(tmp_path):
    counts = {topic: 1 for topic in MODULE.REQUIRED_INPUT_TOPICS}
    counts["/rtk/fix"] = 1
    arguments = make_arguments(tmp_path, counts)

    with pytest.raises(MODULE.ValidationError, match="contains RTK fixes"):
        MODULE.validate_inputs(arguments)


def test_indoor_validation_only_runs_fastlivo_and_fusion():
    source = SCRIPT.read_text(encoding="utf-8")

    assert "ackermann_fastlivo_rtk_localization.launch.py" in source
    assert "initialization_source:=manual" in source
    assert "allow_missing_georeference:=true" in source
    assert '"/fastlivo/odometry"' in source
    assert '"/fastlivo_rtk/odometry"' in source
    assert '"/fastlivo_rtk/path"' in source
    assert '"/fastlivo_rtk/fastlivo_path"' in source
    assert "/comparison/kf_gins/odometry" not in source
    assert "rtk_eskf_localization" not in source


def test_path_comparison_detects_an_absolute_correction():
    local_path = PathMessage()
    fused_path = PathMessage()
    local_pose = PoseStamped()
    local_pose.pose.orientation.w = 1.0
    fused_pose = PoseStamped()
    fused_pose.pose.position.x = 0.2
    fused_pose.pose.orientation.w = 1.0
    local_path.poses.append(local_pose)
    fused_path.poses.append(fused_pose)

    position_delta, orientation_delta = MODULE.compare_paths(
        fused_path, local_path
    )

    assert position_delta == pytest.approx(0.2)
    assert orientation_delta == pytest.approx(0.0)
