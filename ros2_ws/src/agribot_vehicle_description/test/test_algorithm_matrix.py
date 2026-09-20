import importlib.util
import subprocess
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "unity_sync_and_simulate.sh"
LAUNCH_FILE = PACKAGE / "launch" / "configured_ackermann_sim.launch.py"
WORKSPACE_SRC = PACKAGE.parent


def run_script(*arguments):
    return subprocess.run(
        [str(SCRIPT), *arguments],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def load_launch_module():
    spec = importlib.util.spec_from_file_location("configured_ackermann_sim", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_component_catalog_generates_120_unique_combinations():
    components = run_script("--list-components").strip().splitlines()
    groups = {}
    for line in components:
        group, component_id, _name, _description = line.split("|", 3)
        groups.setdefault(group, []).append(component_id)

    assert {key: len(value) for key, value in groups.items()} == {
        "perception": 2,
        "localization": 4,
        "planner": 5,
        "controller": 3,
    }

    algorithms = run_script("--list-algorithms").strip().splitlines()
    algorithm_ids = [line.split("|", 1)[0] for line in algorithms]
    assert len(algorithm_ids) == 120
    assert len(set(algorithm_ids)) == 120
    assert run_script("--validate-algorithms").strip() == (
        "VALIDATED_ALGORITHM_COMBINATIONS=120"
    )


def test_independent_component_selection_and_legacy_alias():
    resolved = run_script(
        "--perception",
        "voxel",
        "--localization",
        "kiss_icp",
        "--planner",
        "theta_star",
        "--controller",
        "dwb",
        "--resolve-only",
    )
    assert "algorithm=voxel_kiss_icp_theta_star_dwb" in resolved
    assert "perception=voxel" in resolved
    assert "localization=kiss_icp" in resolved
    assert "planner=theta_star" in resolved
    assert "controller=dwb" in resolved
    assert "route=planned" in resolved

    direct = run_script(
        "--perception",
        "stvl",
        "--localization",
        "navsat",
        "--planner",
        "direct",
        "--controller",
        "rpp",
        "--resolve-only",
    )
    assert "planner=smac_hybrid" in direct
    assert "route=direct" in direct

    legacy = run_script(
        "--algorithm", "fastlivo_rtk_navfn_dwb", "--resolve-only"
    )
    assert "algorithm=stvl_fastlivo_rtk_navfn_dwb" in legacy


def test_every_nav2_plugin_profile_uses_the_unified_interfaces():
    module = load_launch_module()
    profiles = module._write_nav2_profiles(
        WORKSPACE_SRC
        / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml",
        PACKAGE / "config",
        "test",
    )
    assert len(profiles) == 24

    planner_plugins = {
        "smac_hybrid": "nav2_smac_planner/SmacPlannerHybrid",
        "navfn": "nav2_navfn_planner/NavfnPlanner",
        "theta_star": "nav2_theta_star_planner/ThetaStarPlanner",
        "smac_2d": "nav2_smac_planner/SmacPlanner2D",
    }
    controller_plugins = {
        "mppi": "nav2_mppi_controller::MPPIController",
        "rpp": (
            "nav2_regulated_pure_pursuit_controller::"
            "RegulatedPurePursuitController"
        ),
        "dwb": "dwb_core::DWBLocalPlanner",
    }
    perception_plugins = {
        "stvl": (
            "stvl_layer",
            "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer",
        ),
        "voxel": ("voxel_layer", "nav2_costmap_2d::VoxelLayer"),
    }

    for (perception, controller, planner), profile_path in profiles.items():
        with open(profile_path, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        assert (
            config["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
            == controller_plugins[controller]
        )
        assert (
            config["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
            == planner_plugins[planner]
        )
        layer_name, layer_plugin = perception_plugins[perception]
        for costmap in ("local_costmap", "global_costmap"):
            parameters = config[costmap][costmap]["ros__parameters"]
            assert layer_name in parameters["plugins"]
            assert parameters[layer_name]["plugin"] == layer_plugin


if __name__ == "__main__":
    test_component_catalog_generates_120_unique_combinations()
    test_independent_component_selection_and_legacy_alias()
    test_every_nav2_plugin_profile_uses_the_unified_interfaces()
    print("ALGORITHM_MATRIX_TESTS_PASSED=3")
