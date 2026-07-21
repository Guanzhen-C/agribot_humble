from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_shared_can_code_uses_can_directory():
    expected = (
        "include/agribot_hardware_bringup/chassis_adapter.hpp",
        "include/agribot_hardware_bringup/chassis_can_common.hpp",
        "include/agribot_hardware_bringup/chassis_can_node.hpp",
        "src/chassis_can_common.cpp",
        "src/chassis_can_node.cpp",
    )
    can_root = PACKAGE_ROOT / "can"
    assert can_root.is_dir()
    for relative_path in expected:
        assert (can_root / relative_path).is_file()
    assert not (PACKAGE_ROOT / "common").exists()


def test_vehicle_specific_code_is_separated():
    expected = {
        "differential": (
            "README.md",
            "config/chassis_can.yaml",
            "include/agribot_hardware_bringup/differential_can_protocol.hpp",
            "launch/differential_dwb_fastlio.launch.py",
            "launch/differential_dwb_navsat.launch.py",
            "src/differential_can_protocol.cpp",
            "src/differential_chassis_adapter.cpp",
            "src/differential_chassis_main.cpp",
            "test/test_differential_can_protocol.cpp",
        ),
        "ackermann": (
            "README.md",
            "config/chassis_can.yaml",
            "include/agribot_hardware_bringup/ackermann_can_protocol.hpp",
            "launch/ackermann_mppi_fastlio.launch.py",
            "launch/ackermann_mppi_navsat.launch.py",
            "src/ackermann_can_protocol.cpp",
            "src/ackermann_chassis_adapter.cpp",
            "src/ackermann_chassis_main.cpp",
            "test/test_ackermann_can_protocol.cpp",
        ),
    }
    for vehicle, relative_paths in expected.items():
        vehicle_root = PACKAGE_ROOT / vehicle
        assert vehicle_root.is_dir()
        for relative_path in relative_paths:
            assert (vehicle_root / relative_path).is_file()


def test_obsolete_mixed_chassis_files_are_removed():
    obsolete = (
        "config/chassis_can.yaml",
        "include/agribot_hardware_bringup/chassis_can_protocol.hpp",
        "src/chassis_can_protocol.cpp",
        "src/chassis_can_node.cpp",
        "test/test_chassis_can_protocol.cpp",
    )
    for relative_path in obsolete:
        assert not (PACKAGE_ROOT / relative_path).exists()


def test_unified_launch_selects_dedicated_executables_and_configs():
    launch_source = (PACKAGE_ROOT / "launch" / "vehicle_autonomy.launch.py").read_text()
    assert 'executable="differential_chassis_can_node"' in launch_source
    assert 'executable="ackermann_chassis_can_node"' in launch_source
    assert '"differential", "config", "chassis_can.yaml"' in launch_source
    assert '"ackermann", "config", "chassis_can.yaml"' in launch_source
    assert 'executable="chassis_can_node"' not in launch_source
