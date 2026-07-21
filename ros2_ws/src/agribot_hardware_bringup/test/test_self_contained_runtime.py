import re
import xml.etree.ElementTree as element_tree
from pathlib import Path

import yaml


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
        "ackermann/config/nav2_params_ackermann_fastlio_static.yaml",
        "maps/orchard_v2_map6.yaml",
        "maps/orchard_v2_map6.pgm",
    )
    for relative_path in expected:
        assert (PACKAGE_ROOT / relative_path).is_file()

    for path in (PACKAGE_ROOT / "ackermann" / "behavior_trees").glob("*.xml"):
        element_tree.parse(path)

    map_path = PACKAGE_ROOT / "maps" / "orchard_v2_map6.yaml"
    map_config = yaml.safe_load(map_path.read_text())
    assert (map_path.parent / map_config["image"]).is_file()


def test_kf_gins_subset_keeps_its_license_and_attribution():
    third_party = PACKAGE_ROOT / "localization" / "navsat" / "third_party" / "kf_gins"
    assert (third_party / "LICENSE").is_file()
    assert "GNU GENERAL PUBLIC LICENSE" in (third_party / "LICENSE").read_text()
    assert "i2Nav-WHU/KF-GINS" in (third_party / "README.md").read_text()
