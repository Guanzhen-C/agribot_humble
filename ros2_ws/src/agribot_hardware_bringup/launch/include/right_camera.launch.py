import os
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_driver(context):
    driver = LaunchConfiguration("camera_driver").perform(context)
    if driver not in ("hikrobot_mvs", "usb_cam"):
        raise RuntimeError("camera_driver必须是hikrobot_mvs或usb_cam")
    return []


def _enabled(value):
    return value.lower() in ("true", "1", "yes", "on")


def _read_key_values(path):
    values = {}
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        name, value = line.split("=", 1)
        values[name] = value
    return values


def _mismatches(actual, expected, prefix):
    return [
        f"{prefix}.{name}={actual.get(name, '缺失')}(期望{value})"
        for name, value in expected.items()
        if actual.get(name) != value
    ]


def _validate_pin32_trigger(
    ready, pwm_path, edge_buffer_path, period_ns, duty_ns
):
    actual_pwm = {
        name: (pwm_path / name).read_text().strip()
        for name in ("period", "duty_cycle", "polarity", "enable")
    }
    lock_state = ready.get("pps_lock_state")
    expected_ready = {
        "backend": "pin32_pwm",
        "pwm_enable_path": str(pwm_path / "enable"),
        "pwm_period_path": str(pwm_path / "period"),
        "period_ns": str(period_ns),
        "duty_cycle_ns": str(duty_ns),
        "polarity": "normal",
        "pps_monitoring": "continuous",
        "physical_edge_capture": "pin33_gpio",
        "edge_timestamp_source": "gpio_v2_realtime",
        "edge_gpio_chip": "/dev/gpiochip5",
        "edge_gpio_offset": "10",
        "edge_buffer_path": str(edge_buffer_path),
    }
    expected_pwm = {
        "period": ready.get("applied_period_ns", "缺失"),
        "duty_cycle": str(duty_ns),
        "polarity": "normal",
        "enable": "1",
    }
    mismatches = _mismatches(ready, expected_ready, "ready") + _mismatches(
        actual_pwm, expected_pwm, "pwm"
    )
    if lock_state == "locked":
        mismatches += _mismatches(
            ready,
            {
                "pps_alignment": "every_pps",
                "pwm_phase_control": "period_adjust_each_pps",
            },
            "ready",
        )
        if int(ready.get("pps_sequence", "0")) <= 0:
            mismatches.append("ready.pps_sequence无效")
        if abs(float(ready.get("edge_phase_error_us", "inf"))) > 5000.0:
            mismatches.append("ready.edge_phase_error_us超出5ms")
    elif lock_state == "holdover":
        mismatches += _mismatches(
            ready,
            {
                "pps_alignment": "holdover",
                "pwm_phase_control": "nominal_period",
                "pps_sequence": "0",
            },
            "ready",
        )
    else:
        mismatches.append(f"ready.pps_lock_state={lock_state or '缺失'}")
    expected_edges = 1_000_000_000 // period_ns
    if int(ready.get("edges_previous_second", "-1")) != expected_edges:
        mismatches.append(
            "ready.edges_previous_second="
            f"{ready.get('edges_previous_second', '缺失')}(期望{expected_edges})"
        )
    try:
        edge_age_ns = time.time_ns() - int(ready.get("edge_timestamp_ns", "0"))
        maximum_edge_age_ns = (
            1_000_000_000 + period_ns * 3
            if lock_state == "locked"
            else period_ns * 3
        )
        if edge_age_ns < 0 or edge_age_ns > maximum_edge_age_ns:
            mismatches.append("ready.edge_timestamp_ns不是最近的物理触发沿")
    except ValueError:
        mismatches.append("ready.edge_timestamp_ns无效")
    if not edge_buffer_path.is_file():
        mismatches.append(f"物理沿缓冲不存在：{edge_buffer_path}")
    return mismatches


def _validate_j14_trigger(ready, lpwm_device_path, period_ns, duty_ns):
    if period_ns % 1000 or duty_ns % 1000:
        raise ValueError("LPWM周期和高电平必须是整数微秒")
    rows = [
        line.split()
        for line in (lpwm_device_path / "lpwm_config_info")
        .read_text()
        .splitlines()[1:]
        if line.strip()
    ]
    selected = next((row for row in rows if row[0] == "0"), None)
    if selected is None or len(selected) != 8:
        raise ValueError("RDK LPWM1通道0状态缺失或格式错误")
    actual_driver = dict(
        zip(
            (
                "core",
                "source",
                "offset",
                "period",
                "duty_time",
                "threshold",
                "adjust_step",
                "occupied",
            ),
            selected,
        )
    )
    expected_ready = {
        "backend": "j14_lpwm",
        "channel_id": "4",
        "channel": "0",
        "trigger_source": "6",
        "trigger_mode": "1",
        "period_us": str(period_ns // 1000),
        "offset_us": "10",
        "duty_us": str(duty_ns // 1000),
    }
    expected_driver = {
        "source": "6",
        "offset": "10",
        "period": str(period_ns // 1000 - 1),
        "duty_time": str(duty_ns // 1000 - 1),
        "threshold": "0",
        "adjust_step": "0",
        "occupied": "CAMSYS",
    }
    return _mismatches(ready, expected_ready, "ready") + _mismatches(
        actual_driver, expected_driver, "lpwm"
    )


def _validate_hardware_trigger(context):
    driver = LaunchConfiguration("camera_driver").perform(context)
    trigger_enabled = _enabled(
        LaunchConfiguration("hikrobot_trigger_enable").perform(context)
    )
    if driver != "hikrobot_mvs" or not trigger_enabled:
        return []

    pwm_path = Path(
        LaunchConfiguration("hikrobot_trigger_pwm_path").perform(context)
    )
    lpwm_device_path = Path(
        LaunchConfiguration("hikrobot_trigger_lpwm_device_path").perform(context)
    )
    ready_path = Path(
        LaunchConfiguration("hikrobot_trigger_ready_path").perform(context)
    )
    edge_buffer_path = Path(
        LaunchConfiguration("hikrobot_trigger_edge_buffer_path").perform(context)
    )
    requested_backend = LaunchConfiguration(
        "hikrobot_trigger_backend"
    ).perform(context)
    try:
        period_ns = int(
            LaunchConfiguration("hikrobot_trigger_period_ns").perform(context)
        )
        duty_ns = int(
            LaunchConfiguration("hikrobot_trigger_duty_cycle_ns").perform(context)
        )
        ready = _read_key_values(ready_path)
        backend = ready["backend"]
        if backend not in ("pin32_pwm", "j14_lpwm"):
            raise ValueError(f"未知触发后端{backend}")
        if requested_backend != "auto" and requested_backend != backend:
            raise ValueError(
                f"运行后端{backend}与请求后端{requested_backend}不一致"
            )
        process_id = int(ready["pid"])
        if not Path(f"/proc/{process_id}").is_dir():
            raise OSError(f"触发服务进程{process_id}不存在")
        if backend == "pin32_pwm":
            mismatches = _validate_pin32_trigger(
                ready, pwm_path, edge_buffer_path, period_ns, duty_ns
            )
        else:
            if int(ready["pps_sequence"]) <= 0:
                raise ValueError("PPS序号无效")
            mismatches = _validate_j14_trigger(
                ready, lpwm_device_path, period_ns, duty_ns
            )
    except (KeyError, OSError, UnicodeError, ValueError) as error:
        raise RuntimeError(
            "海康相机已要求Line0触发，但RDK相机触发服务未就绪："
            f"{error}；请安装并启动agribot-camera-trigger.service"
        ) from error

    if mismatches:
        raise RuntimeError(
            f"海康相机Line0触发后端{backend}未就绪："
            + ", ".join(mismatches)
            + "；请执行sudo systemctl restart agribot-camera-trigger.service"
        )
    return []


def generate_launch_description():
    hardware_share = get_package_share_directory("agribot_hardware_bringup")
    hikrobot_share = get_package_share_directory("hikrobot_mvs_ros2")
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_driver", default_value="hikrobot_mvs"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "hikrobot_camera_config",
                default_value=os.path.join(
                    hikrobot_share, "config", "mv_cu013_a0uc.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "hikrobot_camera_serial", default_value="DB0447659"
            ),
            DeclareLaunchArgument("hikrobot_trigger_enable", default_value="false"),
            DeclareLaunchArgument("hikrobot_trigger_backend", default_value="auto"),
            DeclareLaunchArgument(
                "hikrobot_trigger_pwm_path",
                default_value="/sys/class/pwm/pwmchip0/pwm0",
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_lpwm_device_path",
                default_value="/sys/class/pwm/pwmchip2/device",
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_ready_path",
                default_value="/run/agribot-camera-trigger/ready",
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_edge_buffer_path",
                default_value="/run/agribot-camera-trigger/physical_edges.bin",
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_period_ns", default_value="100000000"
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_duty_cycle_ns", default_value="1000000"
            ),
            DeclareLaunchArgument(
                "usb_camera_config",
                default_value=os.path.join(
                    hardware_share, "config", "right_camera.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "right_camera_device", default_value="/dev/agribot_right_camera"
            ),
            OpaqueFunction(function=_validate_driver),
            OpaqueFunction(function=_validate_hardware_trigger),
            Node(
                package="hikrobot_mvs_ros2",
                executable="hikrobot_mvs_camera_node",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    LaunchConfiguration("hikrobot_camera_config"),
                    {
                        "serial_number": LaunchConfiguration(
                            "hikrobot_camera_serial"
                        ),
                        "trigger_enable": LaunchConfiguration(
                            "hikrobot_trigger_enable"
                        ),
                        "trigger_edge_buffer_path": LaunchConfiguration(
                            "hikrobot_trigger_edge_buffer_path"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                condition=LaunchConfigurationEquals(
                    "camera_driver", "hikrobot_mvs"
                ),
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="agribot_right_camera",
                output="screen",
                parameters=[
                    LaunchConfiguration("usb_camera_config"),
                    {
                        "video_device": LaunchConfiguration(
                            "right_camera_device"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=[
                    ("image_raw", "/camera/rgb/image_raw"),
                    ("camera_info", "/camera/rgb/camera_info"),
                ],
                condition=LaunchConfigurationEquals("camera_driver", "usb_cam"),
            ),
        ]
    )
