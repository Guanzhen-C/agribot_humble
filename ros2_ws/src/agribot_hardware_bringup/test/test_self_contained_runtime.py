import re
import xml.etree.ElementTree as element_tree
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1]
FORBIDDEN_PROJECT_PACKAGES = {
    "agribot_ackermann_mppi",
    "agribot_autonomy",
    "agribot_rl_nav",
    "scout_base",
    "scout_navigation",
}


def runtime_text_files():
    yield PACKAGE_ROOT / "CMakeLists.txt"
    yield PACKAGE_ROOT / "package.xml"
    yield from (PACKAGE_ROOT / "launch").rglob("*.py")
    yield from (PACKAGE_ROOT / "differential" / "launch").rglob("*.py")
    yield from (PACKAGE_ROOT / "ackermann" / "launch").rglob("*.py")


def test_runtime_has_no_reference_to_removed_project_packages():
    for path in runtime_text_files():
        source = path.read_text()
        for package in FORBIDDEN_PROJECT_PACKAGES:
            assert package not in source, f"{path} still references {package}"


def test_all_agribot_nodes_are_provided_by_this_package():
    node_pattern = re.compile(r'package\s*=\s*"(agribot_[^"]+)"')
    for path in (PACKAGE_ROOT / "launch").rglob("*.py"):
        for package in node_pattern.findall(path.read_text()):
            assert package == "agribot_hardware_bringup"


def test_localization_sources_are_built_without_sibling_source_paths():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "localization/navsat/src/rtk_eskf_localization_node.cpp" in cmake
    assert "localization/navsat/src/rtk_gi_engine.cpp" in cmake
    assert "localization/navsat/third_party/kf_gins/kf-gins/insmech.cpp" in cmake
    assert "../KF-GINS" not in cmake
    assert "/home/" not in cmake


def test_vehicle_launch_files_are_only_installed_at_top_level():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()
    assert "differential/launch\n  DESTINATION" not in cmake
    assert "ackermann/launch\n  DESTINATION" not in cmake
    assert "install(DIRECTORY differential/launch/" in cmake
    assert "install(DIRECTORY ackermann/launch/" in cmake


def test_migrated_runtime_resources_exist_and_parse():
    expected = (
        "localization/navsat/scripts/navsat_pose_bridge.py",
        "localization/fastlio/scripts/fastlio_odom_bridge.py",
        "ackermann/config/nav2_params_ackermann_navsat_static.yaml",
        "ackermann/config/nav2_params_ackermann_fastlio_mapped.yaml",
        "ackermann/config/nav2_params_ackermann_fastlio_local.yaml",
        "ackermann/launch/ackermann_mppi_fastlio_localization.launch.py",
        "ackermann/launch/ackermann_mppi_fastlio_mapping.launch.py",
        "config/pointcloud_to_laserscan_mapping.yaml",
        "config/slam_toolbox_mapping_c16.yaml",
        "config/slam_toolbox_localization_c16.yaml",
        "rviz/navigation_local.rviz",
        "scripts/start_wheeltec_car_gui.sh",
        "scripts/wheeltec_car_gui.py",
        "desktop/wheeltec-car-gui.desktop",
    )
    for relative_path in expected:
        assert (PACKAGE_ROOT / relative_path).is_file()

    obsolete = (
        "ackermann/config/nav2_params_ackermann_fastlio_static.yaml",
        "ackermann/launch/ackermann_mppi_fastlio.launch.py",
        "config/slam_toolbox_c16.yaml",
    )
    for relative_path in obsolete:
        assert not (PACKAGE_ROOT / relative_path).exists()

    for path in (PACKAGE_ROOT / "ackermann" / "behavior_trees").glob("*.xml"):
        element_tree.parse(path)


def test_rviz_goal_tool_sends_nav2_action_directly():
    for name in ("navigation.rviz", "navigation_local.rviz"):
        config = (PACKAGE_ROOT / "rviz" / name).read_text()
        assert "Class: nav2_rviz_plugins/GoalTool" in config
        assert "Class: nav2_rviz_plugins/Navigation 2" in config
        assert "Class: rviz_default_plugins/SetGoal" not in config


def test_simulation_orchard_map_is_not_bundled_or_defaulted():
    assert not (PACKAGE_ROOT / "maps" / "orchard_v2_map6.yaml").exists()
    assert not (PACKAGE_ROOT / "maps" / "orchard_v2_map6.pgm").exists()
    for path in runtime_text_files():
        assert "orchard_v2_map6" not in path.read_text()


def test_kf_gins_subset_keeps_its_license_and_attribution():
    third_party = PACKAGE_ROOT / "localization" / "navsat" / "third_party" / "kf_gins"
    assert (third_party / "LICENSE").is_file()
    assert "GNU GENERAL PUBLIC LICENSE" in (third_party / "LICENSE").read_text()
    assert "i2Nav-WHU/KF-GINS" in (third_party / "README.md").read_text()
