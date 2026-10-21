import importlib.util
import json
import math
import subprocess
from pathlib import Path

import yaml
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "unity_sync_and_simulate.sh"
LAUNCH_FILE = PACKAGE / "launch" / "configured_ackermann_sim.launch.py"
WORKSPACE_SRC = PACKAGE.parent


def test_navsat_simulation_loads_exported_vehicle_lever_arms():
    import ast

    launch = WORKSPACE_SRC / "agribot_ackermann_mppi/launch/ackermann_waypoint_depth_collect_sim.launch.py"
    calls = [node for node in ast.walk(ast.parse(launch.read_text()))
             if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
             and node.func.id == "Node"]
    names = []
    for call in calls:
        values = {keyword.arg: keyword.value for keyword in call.keywords}
        executable = values.get("executable")
        if isinstance(executable, ast.Constant) and executable.value == "rtk_eskf_localization":
            names.append(ast.literal_eval(values["name"]))
    assert names == ["rtk_eskf_localization"]
    for vehicle in ("harvester_header", "unity_car1", "unity_car2", "unity_car4", "unity_car5"):
        params = yaml.safe_load((PACKAGE / "generated" / vehicle /
                                "config/physical/kf_gins_n300pro.yaml").read_text())
        assert names[0] in params
        assert params[names[0]]["ros__parameters"]["base_to_imu_m"] != [0.1425, 0.0, 0.143]


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


@pytest.mark.parametrize("scenario", [item["id"] for item in json.loads(
    (WORKSPACE_SRC / "agribot_farm_sim/config/scenarios.json").read_text())["scenarios"]])
def test_unity_entrypoint_accepts_every_scene_in_catalog(scenario):
    resolved = run_script("--scenario", scenario, "--resolve-only")
    assert "algorithm=" in resolved
    assert "run_waypoints=false" in resolved
    from launch.actions import DeclareLaunchArgument
    declaration = next(entity for entity in load_launch_module().generate_launch_description().entities
                       if isinstance(entity, DeclareLaunchArgument) and entity.name == "scenario")
    assert scenario in declaration.choices


def test_automatic_routes_require_explicit_opt_in():
    assert "run_waypoints=true" in run_script("--run-waypoints", "true", "--resolve-only")
    assert "run_waypoints=false" in run_script("--run-waypoints", "false", "--resolve-only")
    text = LAUNCH_FILE.read_text()
    assert 'DeclareLaunchArgument("run_waypoints", default_value="false"' in text
    assert 'LaunchConfiguration("run_waypoints", default="false")' in text


@pytest.mark.parametrize("vision_mode", ["off", "object_detection", "instance_segmentation", "pose_estimation"])
def test_unity_simulation_rviz_shows_the_published_rgb_image(vision_mode, tmp_path, monkeypatch):
    module = load_launch_module()
    monkeypatch.setattr(module.tempfile, "gettempdir", lambda: str(tmp_path))
    source = WORKSPACE_SRC / "agribot_autonomy/rviz/robot_map_global_plan_only.rviz"
    original = source.read_bytes()
    config = yaml.safe_load(Path(module._write_simulation_rviz_config(source, vision_mode)).read_text())
    assert sum(panel["Class"] == "nav2_rviz_plugins/Navigation 2" for panel in config["Panels"]) == 1
    tools = config["Visualization Manager"]["Tools"]
    assert sum(tool["Class"] == "nav2_rviz_plugins/GoalTool" for tool in tools) == 1
    assert not any(tool["Class"] == "rviz_default_plugins/SetGoal" for tool in tools)
    assert config["Window Geometry"]["Navigation 2"]["collapsed"] is False
    second = yaml.safe_load(Path(module._write_simulation_rviz_config(
        Path(tmp_path)/"configured_vehicle_camera.generated.rviz", vision_mode)).read_text())
    assert second == config
    assert all(display["Class"] != "rviz_default_plugins/Image"
               for display in config["Visualization Manager"]["Displays"])
    displays = config["Visualization Manager"]["Displays"]
    rgb = next(display for display in displays if display["Name"] == "RGB Image")
    assert rgb["Enabled"] and rgb["Value"]
    assert rgb["Queue Size"] == rgb["Topic"]["Depth"] == 1
    assert rgb["Topic"]["Value"] == (
        "/camera/rgb/image_raw" if vision_mode == "off" else "/vision/annotated_image")
    assert rgb["Topic"]["Reliability Policy"] == "Best Effort"
    previous = yaml.safe_load(original)["Visualization Manager"]["Displays"]
    for display in previous:
        if display["Class"] == "rviz_default_plugins/Image":
            display["Class"] = "scout_viz/Image"
        if (display["Class"] == "rviz_default_plugins/Path" and
                display.get("Topic", {}).get("Value") in ("/plan", "/local_plan", "/waypoint_path")):
            display["Class"] = "scout_viz/NavigationPath"
    assert [d for d in displays if d["Name"] != "RGB Image"] == [
        d for d in previous if d["Name"] != "RGB Image"]
    assert source.read_bytes() == original
    launch = LAUNCH_FILE.read_text()
    assert '"rviz_config": LaunchConfiguration("rviz_config")' in launch


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
            "agribot_vehicle_description::"
            "PositionOnlyRegulatedPurePursuitController"
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
            assert controller_parameters["FollowPath"]["time_steps"] == 40
            assert controller_parameters["FollowPath"]["PathAngleCritic"]["max_angle_to_furthest"] == 0.20
            assert controller_parameters["FollowPath"]["PathAngleCritic"]["cost_weight"] == 10.0
            assert controller_parameters["FollowPath"]["az_max"] == 0.60
            assert controller_parameters["goal_checker"]["plugin"] == (
                "nav2_controller::PositionGoalChecker"
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


def test_ordered_route_goal_checker_keeps_original_checker_and_tolerances():
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    profiles = module._write_nav2_profiles(source, PACKAGE / "config", "test_route", guard_route_progress=True)
    for path in profiles.values():
        params = yaml.safe_load(Path(path).read_text())["controller_server"]["ros__parameters"]
        original = params["goal_checker"]
        guarded = params["route_goal_checker"]
        assert guarded["plugin"] == "agribot_vehicle_description::RouteProgressGoalChecker"
        assert guarded["wrapped_plugin"] == original["plugin"]
        assert guarded["requested_goal_topic"] == "/current_goal"
        assert guarded["requested_goal_frame"] == "map"
        assert {key: value for key, value in guarded.items()
                if key not in ("plugin", "wrapped_plugin", "requested_goal_topic", "requested_goal_frame")} == {
                    key: value for key, value in original.items() if key != "plugin"}
        assert params["goal_checker_plugins"] == ["goal_checker", "route_goal_checker"]


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
            assert "RotateToGoal" not in follow["critics"]
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
            "nav2_controller::PositionGoalChecker"
        )


@pytest.mark.parametrize("controller", ("mppi", "rpp", "dwb"))
@pytest.mark.parametrize("planner", ("navfn", "theta_star", "smac_2d"))
@pytest.mark.parametrize("differential", (True, False))
def test_xy_planners_preserve_explicit_differential_goal_heading(tmp_path, controller, planner, differential):
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    config["controller_server"]["ros__parameters"]["FollowPath"]["motion_model"] = "DiffDrive" if differential else "Ackermann"
    template = tmp_path / "template.yaml"
    template.write_text(yaml.safe_dump(config))
    output = module._write_nav2_profile_config(template, PACKAGE / "config/rpp_controller.yaml",
        PACKAGE / "config/dwb_controller.yaml", "test_xy_goal", "stvl", controller, planner)
    actual = yaml.safe_load(Path(output).read_text())
    grid = actual["planner_server"]["ros__parameters"]["GridBased"]
    assert grid["use_final_approach_orientation"] is (not differential)
    goal = actual["controller_server"]["ros__parameters"]["goal_checker"]
    if differential:
        assert goal["plugin"] == "nav2_controller::PositionGoalChecker"
        assert goal["xy_goal_tolerance"] == .25
        assert "yaw_goal_tolerance" not in goal


@pytest.mark.parametrize("angular_limit,expected_std,radius", [
    (0.092, 0.046, 3.25), (0.054, 0.027, 5.56), (0.30, 0.16, 1.3)])
def test_ackermann_sampling_scales_only_when_it_exceeds_vehicle_limit(
        tmp_path, angular_limit, expected_std, radius):
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    config["controller_server"]["ros__parameters"]["FollowPath"]["wz_max"] = angular_limit
    config["controller_server"]["ros__parameters"]["FollowPath"]["AckermannConstraints"]["min_turning_r"] = radius
    input_file = tmp_path / "vehicle.yaml"
    input_file.write_text(yaml.safe_dump(config))
    original = input_file.read_bytes()
    output_file = module._write_nav2_profile_config(
        input_file, PACKAGE / "config/rpp_controller.yaml", PACKAGE / "config/dwb_controller.yaml",
        "test_sampling", "stvl", "mppi", "smac_hybrid")
    output = yaml.safe_load(Path(output_file).read_text())
    follow = output["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["wz_std"] == pytest.approx(expected_std)
    assert follow["gamma"] == pytest.approx(0.015 * (expected_std / 0.16) ** 2)
    assert follow["PathAngleCritic"]["max_angle_to_furthest"] == pytest.approx(
        min(0.20, 0.5 * angular_limit * 40 * 0.1) if angular_limit < 0.16 else 0.20)
    assert follow["wz_max"] == angular_limit
    assert follow["AckermannConstraints"]["min_turning_r"] == radius
    assert follow["ConstraintCritic"]["cost_weight"] == pytest.approx(
        4.0 * min(1.0, 1.3 / radius) if angular_limit < 0.16 else 4.0)
    assert input_file.read_bytes() == original


@pytest.mark.parametrize("angular_limit", [.1599, .16, .16022, .23])
def test_nominal_tractor_sampling_does_not_miss_near_boundary_limits(tmp_path, angular_limit):
    module = load_launch_module()
    source = PACKAGE / "generated/car4_harrow/config/simulation/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    original = config["controller_server"]["ros__parameters"]["FollowPath"]
    original["wz_max"] = angular_limit
    input_file = tmp_path / "nominal.yaml"
    input_file.write_text(yaml.safe_dump(config))
    path = module._write_nav2_profile_config(
        input_file, PACKAGE / "config/rpp_controller.yaml", PACKAGE / "config/dwb_controller.yaml",
        "test_nominal_sampling", "stvl", "mppi", "smac_hybrid", True)
    follow = yaml.safe_load(Path(path).read_text())["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["wz_std"] == pytest.approx(.5 * angular_limit)
    assert follow["gamma"] / follow["wz_std"] ** 2 == pytest.approx(original["gamma"] / .16 ** 2)
    assert follow["wz_max"] == angular_limit
    assert follow["AckermannConstraints"] == original["AckermannConstraints"]


@pytest.mark.parametrize("nominal", [False, True])
def test_large_nominal_vehicle_inflation_covers_full_footprint(tmp_path, nominal):
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    for name in ("global_costmap", "local_costmap"):
        params = config[name][name]["ros__parameters"]
        params["footprint"] = "[[4.6,2.9],[4.6,-2.9],[-2.8,-2.9],[-2.8,2.9]]"
        params["inflation_layer"].update(inflation_radius=5.0, cost_scaling_factor=3.0)
    input_file = tmp_path / "large.yaml"
    input_file.write_text(yaml.safe_dump(config))
    original = input_file.read_bytes()
    output = module._write_nav2_profile_config(
        input_file, PACKAGE / "config/rpp_controller.yaml", PACKAGE / "config/dwb_controller.yaml",
        "test_inflation", "stvl", "mppi", "smac_hybrid", nominal)
    result = yaml.safe_load(Path(output).read_text())
    for name in ("global_costmap", "local_costmap"):
        params = result[name][name]["ros__parameters"]
        inflation = params["inflation_layer"]
        assert params["footprint"] == config[name][name]["ros__parameters"]["footprint"]
        if nominal:
            circum = math.hypot(4.6, 2.9)
            assert inflation["inflation_radius"] >= circum + params["resolution"]
            assert int(252 * math.exp(-inflation["cost_scaling_factor"] * (circum - 2.8))) >= 5
        else:
            assert inflation == config[name][name]["ros__parameters"]["inflation_layer"]
    assert input_file.read_bytes() == original


def test_metric_tractors_use_size_aware_profile_without_changing_small_vehicles():
    module = load_launch_module()
    for vehicle_id in ["ackermann_current"] + [f"unity_car{index}" for index in range(1, 6)]:
        vehicle = json.loads((PACKAGE / "generated" / vehicle_id / "vehicle_config.json").read_text())
        assert module._uses_local_cad_profile(vehicle) == (vehicle_id in {
            "unity_car1", "unity_car2", "unity_car4", "unity_car5"}), vehicle_id
    assert module._uses_local_cad_profile({"metadata": {
        "calibrationState": "unity_nominal_simulation_only", "authoringTool": "inspected_local_CAD_profile"}})


@pytest.mark.parametrize("nominal", [False, True])
@pytest.mark.parametrize("differential", [False, True])
def test_large_nominal_rpp_lookahead_scales_without_changing_limits(tmp_path, nominal, differential):
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    config["controller_server"]["ros__parameters"]["FollowPath"]["motion_model"] = "DiffDrive" if differential else "Ackermann"
    config["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"] = "[[7,6],[7,-6],[-7,-6],[-7,6]]"
    input_file = tmp_path / "large_rpp.yaml"
    input_file.write_text(yaml.safe_dump(config))
    output = module._write_nav2_profile_config(
        input_file, PACKAGE / "config/rpp_controller.yaml", PACKAGE / "config/dwb_controller.yaml",
        "test_rpp_size", "stvl", "rpp", "smac_hybrid", nominal)
    result = yaml.safe_load(Path(output).read_text())["controller_server"]["ros__parameters"]
    overlay = yaml.safe_load((PACKAGE / "config/rpp_controller.yaml").read_text())
    if differential:
        diff_overlay = yaml.safe_load((PACKAGE / "config/differential_controllers.yaml").read_text())
        overlay["FollowPath"].update(diff_overlay["rpp"])
        overlay["goal_checker"] = diff_overlay["goal_checker"]
    follow = result["FollowPath"]
    for key, ratio in (("min_lookahead_dist", .15), ("lookahead_dist", .2), ("max_lookahead_dist", .3)):
        assert follow[key] == pytest.approx(ratio * 14 if nominal else overlay["FollowPath"][key])
    assert follow["use_collision_detection"] is True
    expected_goal_checker = yaml.safe_load((PACKAGE / "config/terminal_stop_simulation.yaml").read_text())["goal_checker"]
    expected_goal_checker["xy_goal_tolerance"] = .25
    assert result["goal_checker"] == expected_goal_checker
    assert follow["desired_linear_vel"] == config["controller_server"]["ros__parameters"]["FollowPath"]["vx_max"]


@pytest.mark.parametrize("nominal", [False, True])
def test_nominal_tractor_does_not_require_terminal_heading_correction(tmp_path, nominal):
    module = load_launch_module()
    source = WORKSPACE_SRC / "agribot_ackermann_mppi/config/nav2_params_ackermann_fastlio_static.yaml"
    config = yaml.safe_load(source.read_text())
    follow = config["controller_server"]["ros__parameters"]["FollowPath"]
    follow["AckermannConstraints"]["min_turning_r"] = 5.56
    follow["wz_max"] = 0.3 / 5.56
    input_file = tmp_path / "tractor.yaml"
    input_file.write_text(yaml.safe_dump(config))
    output = module._write_nav2_profile_config(
        input_file, PACKAGE / "config/rpp_controller.yaml", PACKAGE / "config/dwb_controller.yaml",
        "test_terminal", "stvl", "mppi", "smac_hybrid", nominal)
    result = yaml.safe_load(Path(output).read_text())["controller_server"]["ros__parameters"]
    assert result["FollowPath"]["GoalAngleCritic"]["threshold_to_consider"] == pytest.approx(2.78 if nominal else 0.5)
    assert not result["FollowPath"]["GoalAngleCritic"]["enabled"]
    expected_goal_checker = yaml.safe_load((PACKAGE / "config/terminal_stop_simulation.yaml").read_text())["goal_checker"]
    expected_goal_checker["xy_goal_tolerance"] = .25
    assert result["goal_checker"] == expected_goal_checker
    assert result["FollowPath"]["AckermannConstraints"]["min_turning_r"] == 5.56


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
