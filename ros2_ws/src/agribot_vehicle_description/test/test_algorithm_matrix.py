import importlib.util
import subprocess
from pathlib import Path

import yaml
import pytest


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


def test_component_catalog_generates_480_unique_combinations():
    components = run_script("--list-components").strip().splitlines()
    groups = {}
    for line in components:
        group, component_id, _name, _description = line.split("|", 3)
        groups.setdefault(group, []).append(component_id)

    assert {key: len(value) for key, value in groups.items()} == {
        "perception": 2,
        "vision": 4,
        "localization": 4,
        "planner": 5,
        "controller": 3,
    }

    algorithms = run_script("--list-algorithms").strip().splitlines()
    algorithm_ids = [line.split("|", 1)[0] for line in algorithms]
    assert len(algorithm_ids) == 480
    assert len(set(algorithm_ids)) == 480
    assert run_script("--validate-algorithms").strip() == (
        "VALIDATED_ALGORITHM_COMBINATIONS=480"
    )


def test_independent_component_selection_and_legacy_alias():
    resolved = run_script(
        "--perception",
        "voxel",
        "--vision",
        "instance_segmentation",
        "--localization",
        "kiss_icp",
        "--planner",
        "theta_star",
        "--controller",
        "dwb",
        "--resolve-only",
    )
    assert (
        "algorithm=voxel_instance_segmentation_kiss_icp_theta_star_dwb"
        in resolved
    )
    assert "perception=voxel" in resolved
    assert "vision=instance_segmentation" in resolved
    assert "localization=kiss_icp" in resolved
    assert "planner=theta_star" in resolved
    assert "controller=dwb" in resolved
    assert "route=planned" in resolved

    direct = run_script(
        "--perception",
        "stvl",
        "--vision",
        "pose_estimation",
        "--localization",
        "navsat",
        "--planner",
        "direct",
        "--controller",
        "rpp",
        "--resolve-only",
    )
    assert "planner=smac_hybrid" in direct
    assert "vision=pose_estimation" in direct
    assert "route=direct" in direct

    legacy = run_script(
        "--algorithm", "fastlivo_rtk_navfn_dwb", "--resolve-only"
    )
    assert "algorithm=stvl_off_fastlivo_rtk_navfn_dwb" in legacy

    previous_canonical = run_script(
        "--algorithm", "voxel_kiss_icp_theta_star_dwb", "--resolve-only"
    )
    assert "algorithm=voxel_off_kiss_icp_theta_star_dwb" in previous_canonical


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
        controller_parameters = config["controller_server"]["ros__parameters"]
        if controller == "dwb":
            assert controller_parameters["FollowPath"]["trajectory_generator_name"] == (
                "agribot_vehicle_description::AckermannTrajectoryGenerator"
            )
            assert controller_parameters["FollowPath"]["min_turning_radius"] > 0.0
        if controller == "mppi" and planner != "smac_hybrid":
            assert controller_parameters["goal_checker"]["plugin"] == (
                "nav2_controller::PositionGoalChecker"
            )
            assert not controller_parameters["FollowPath"]["PathAlignCritic"]["use_path_orientations"]
            assert not controller_parameters["FollowPath"]["GoalAngleCritic"]["enabled"]
        if controller == "mppi" and planner == "smac_hybrid":
            assert controller_parameters["FollowPath"]["time_steps"] == 60
            assert controller_parameters["FollowPath"]["PathAngleCritic"]["max_angle_to_furthest"] == 0.5
            assert controller_parameters["goal_checker"]["plugin"] == (
                "nav2_controller::SimpleGoalChecker"
            )
        layer_name, layer_plugin = perception_plugins[perception]
        for costmap in ("local_costmap", "global_costmap"):
            parameters = config[costmap][costmap]["ros__parameters"]
            assert layer_name in parameters["plugins"]
            assert parameters[layer_name]["plugin"] == layer_plugin
            marking = parameters[layer_name]["lidar_mark"]
            clearing = parameters[layer_name]["lidar_clear"]
            assert marking["topic"] == "/configured/obstacle_points"
            assert marking["marking"] and not marking["clearing"]
            assert clearing["topic"] == "/lidar/points"
            assert clearing["clearing"] and not clearing["marking"]


def test_differential_vehicle_profiles_do_not_use_ackermann_trajectory_generator(tmp_path):
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    config["controller_server"]["ros__parameters"]["FollowPath"]["motion_model"] = "DiffDrive"
    differential = tmp_path / "differential.yaml"
    differential.write_text(yaml.safe_dump(config))
    profiles = module._write_nav2_profiles(differential, PACKAGE / "config", "test_diff")
    for (_, controller, _), profile_path in profiles.items():
        config = yaml.safe_load(Path(profile_path).read_text())
        follow = config["controller_server"]["ros__parameters"]["FollowPath"]
        if controller == "dwb":
            assert follow["trajectory_generator_name"] == "dwb_plugins::StandardTrajectoryGenerator"
            assert "min_turning_radius" not in follow
            assert "RotateToGoal" in follow["critics"]
            assert follow["max_vel_y"] == 0.0
        elif controller == "rpp":
            assert follow["use_rotate_to_heading"]
            assert not follow["allow_reversing"]
        else:
            assert follow["motion_model"] == "DiffDrive"
            assert "AckermannConstraints" not in follow
            assert follow["GoalAngleCritic"]["threshold_to_consider"] <= (
                config["controller_server"]["ros__parameters"]["goal_checker"]["xy_goal_tolerance"]
            )
        assert config["controller_server"]["ros__parameters"]["goal_checker"]["plugin"] == (
            "nav2_controller::SimpleGoalChecker"
        )


@pytest.mark.parametrize("controller", ["mppi", "rpp", "dwb"])
@pytest.mark.parametrize("reverse", [False, True])
def test_tracked_controllers_preserve_unity_limits_and_template(tmp_path, controller, reverse):
    module = load_launch_module()
    source = PACKAGE / "generated/unity_car3/config/simulation/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    follow = config["controller_server"]["ros__parameters"]["FollowPath"]
    follow["vx_min"] = -follow["vx_max"] if reverse else 0.0
    input_file = tmp_path / "tracked.yaml"
    input_file.write_text(yaml.safe_dump(config))
    original = input_file.read_bytes()
    path = module._write_nav2_profile_config(
        input_file, PACKAGE / "config/rpp_controller.yaml", PACKAGE / "config/dwb_controller.yaml",
        "test_tracked", "stvl", controller, "smac_hybrid",
    )
    output = yaml.safe_load(Path(path).read_text())
    selected = output["controller_server"]["ros__parameters"]["FollowPath"]
    for name in ("global_costmap", "local_costmap"):
        expected = config[name][name]["ros__parameters"]
        actual = output[name][name]["ros__parameters"]
        assert actual["footprint"] == expected["footprint"]
        assert actual["inflation_layer"] == expected["inflation_layer"]
    assert output["planner_server"] == config["planner_server"]
    speed_key = {"mppi": "vx_max", "rpp": "desired_linear_vel", "dwb": "max_vel_x"}[controller]
    assert selected[speed_key] == follow["vx_max"]
    if controller == "rpp":
        assert selected["allow_reversing"] == reverse
        assert selected["use_rotate_to_heading"] != reverse
        assert selected["rotate_to_heading_angular_vel"] == follow["wz_max"]
    elif controller == "dwb":
        assert selected["min_vel_x"] == follow["vx_min"]
        assert selected["max_vel_theta"] == follow["wz_max"]
    else:
        assert selected["vx_min"] == follow["vx_min"]
        assert selected["wz_max"] == follow["wz_max"]
    assert input_file.read_bytes() == original


def test_simulation_time_profiles_preserve_exported_mounts_and_raw_inputs():
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/fast_lio_c16_physical_sim.yaml"
    mounts = PACKAGE / "generated/ackermann_current/config/fastlivo_extrinsics.yaml"
    original = source.read_bytes()
    parameters = yaml.safe_load(Path(module._write_fastlio_sim_config(source, mounts)).read_text())["/**"]["ros__parameters"]
    exported = yaml.safe_load(mounts.read_text())["/**"]["ros__parameters"]["extrin_calib"]
    assert parameters["common"]["lid_topic"] == "/configured/localization_points"
    assert parameters["preprocess"]["timestamp_unit"] == 0
    assert parameters["mapping"]["extrinsic_T"] == exported["extrinsic_T"]
    assert parameters["mapping"]["extrinsic_R"] == exported["extrinsic_R"]
    assert source.read_bytes() == original
    source = PACKAGE / "generated/ackermann_current/config/physical/agribot_c16_astra.yaml"
    original = source.read_bytes()
    parameters = yaml.safe_load(Path(module._write_fastlivo_sim_config(source)).read_text())["/**"]["ros__parameters"]
    assert parameters["common"]["lid_topic"] == "/configured/localization_points"
    assert parameters["preprocess"]["timestamp_unit"] == 0
    assert source.read_bytes() == original


if __name__ == "__main__":
    test_component_catalog_generates_480_unique_combinations()
    test_independent_component_selection_and_legacy_alias()
    test_every_nav2_plugin_profile_uses_the_unified_interfaces()
    test_simulation_time_profiles_preserve_exported_mounts_and_raw_inputs()
    print("ALGORITHM_MATRIX_TESTS_PASSED=4")
