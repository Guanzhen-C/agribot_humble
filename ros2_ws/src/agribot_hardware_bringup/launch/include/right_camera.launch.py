import os
from pathlib import Path

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


def _validate_hardware_trigger(context):
    driver = LaunchConfiguration("camera_driver").perform(context)
    trigger_enabled = _enabled(
        LaunchConfiguration("hikrobot_trigger_enable").perform(context)
    )
    if driver != "hikrobot_mvs" or not trigger_enabled:
        return []

    lpwm_device_path = Path(
        LaunchConfiguration("hikrobot_trigger_lpwm_device_path").perform(context)
    )
    ready_path = Path(
        LaunchConfiguration("hikrobot_trigger_ready_path").perform(context)
    )
    try:
        period_ns = int(
            LaunchConfiguration("hikrobot_trigger_period_ns").perform(context)
        )
        duty_ns = int(
            LaunchConfiguration("hikrobot_trigger_duty_cycle_ns").perform(context)
        )
        if period_ns % 1000 or duty_ns % 1000:
            raise ValueError("LPWM周期和高电平必须是整数微秒")
        ready = _read_key_values(ready_path)
        process_id = int(ready["pid"])
        os.kill(process_id, 0)
        rows = [
            line.split()
            for line in (lpwm_device_path / "lpwm_config_info")
            .read_text()
            .splitlines()[1:]
            if line.strip()
        ]
    except (KeyError, OSError, UnicodeError, ValueError) as error:
        raise RuntimeError(
            "海康相机已要求Line0硬触发，但无法读取RDK LPWM状态："
            f"{lpwm_device_path}: {error}；请先安装并启动"
            "agribot-camera-trigger.service"
        ) from error

    selected = next((row for row in rows if row[0] == "0"), None)
    if selected is None or len(selected) != 8:
        raise RuntimeError("RDK LPWM1通道0状态缺失或格式错误")

    expected_ready = {
        "channel_id": "4",
        "channel": "0",
        "trigger_source": "2",
        "trigger_mode": "1",
        "period_us": str(period_ns // 1000),
        "offset_us": "10",
        "duty_us": str(duty_ns // 1000),
    }
    expected_driver = {
        "source": "2",
        "offset": "10",
        "period": str(period_ns // 1000 - 1),
        "duty_time": str(duty_ns // 1000 - 1),
        "threshold": "0",
        "adjust_step": "0",
        "occupied": "CAMSYS",
    }
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
    mismatches = []
    mismatches.extend(
        f"ready.{name}={ready.get(name, '缺失')}(期望{value})"
        for name, value in expected_ready.items()
        if ready.get(name) != value
    )
    mismatches.extend(
        f"lpwm.{name}={actual_driver[name]}(期望{value})"
        for name, value in expected_driver.items()
        if actual_driver[name] != value
    )
    if mismatches:
        raise RuntimeError(
            "海康相机Line0硬件PPS重触发LPWM未就绪："
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
            DeclareLaunchArgument(
                "hikrobot_trigger_lpwm_device_path",
                default_value="/sys/class/pwm/pwmchip2/device",
            ),
            DeclareLaunchArgument(
                "hikrobot_trigger_ready_path",
                default_value="/run/agribot-camera-trigger/ready",
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
