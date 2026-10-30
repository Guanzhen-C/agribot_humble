"""Gazebo contact/wheel-physics test of the production saved-path runner."""
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def world_file():
    sdf = ET.Element("sdf", version="1.6")
    world = element(sdf, "world", name="default")
    ground=element(world,"model",name="ground")
    element(ground,"static","true")
    link=element(ground,"link",name="plane")
    for kind in ("collision","visual"):
        plane=element(element(element(link,kind,name="plane"),"geometry"),"plane")
        element(plane,"normal","0 0 1");element(plane,"size","100 100")
    light=element(world,"light",name="sun",type="directional")
    element(light,"direction","-0.5 -0.5 -1")
    physics = element(world, "physics", type="ode", name="physics")
    element(physics, "max_step_size", .001)
    element(physics, "real_time_update_rate", 1000)
    model = element(world, "model", name="differential_validation_vehicle")
    start_x = -1.5 if os.environ.get("SAVED_PATH_TEST_SCENARIO") == "entry" else 0
    element(model, "pose", f"{start_x} 0 0.20 0 0 0")
    base = element(model, "link", name="base_link")
    inertial = element(base, "inertial")
    element(inertial, "mass", 80)
    inertia = element(inertial, "inertia")
    for key, value in {"ixx":4,"iyy":10,"izz":12,"ixy":0,"ixz":0,"iyz":0}.items():
        element(inertia,key,value)
    for kind in ("collision", "visual"):
        part = element(base,kind,name="body")
        element(element(element(part,"geometry"),"box"),"size","1.20 0.76 0.20")
    sensor=element(base,"sensor",name="body_contact",type="contact")
    element(sensor,"always_on","true");element(sensor,"update_rate",20)
    element(element(sensor,"contact"),"collision","body")
    bumper=element(sensor,"plugin",name="body_contact_ros",filename="libgazebo_ros_bumper.so")
    element(element(bumper,"ros"),"remapping","bumper_states:=/gazebo_test/contacts")
    for side, y in (("left",.30),("right",-.30)):
        for end, x in (("front",.35),("rear",-.35)):
            name = f"{side}_{end}"
            link = element(model,"link",name=name)
            element(link,"pose",f"{x} {y} -0.05 1.57079632679 0 0")
            inertial=element(link,"inertial");element(inertial,"mass",2)
            inertia=element(inertial,"inertia")
            for key,value in {"ixx":.02,"iyy":.02,"izz":.025,"ixy":0,"ixz":0,"iyz":0}.items():
                element(inertia,key,value)
            for kind in ("collision","visual"):
                part=element(link,kind,name=name)
                cylinder=element(element(part,"geometry"),"cylinder")
                element(cylinder,"radius",.15);element(cylinder,"length",.08)
                if kind=="collision":
                    ode=element(element(element(part,"surface"),"friction"),"ode")
                    # Cylinder local Z is the axle: allow lateral skid while
                    # retaining rolling-direction traction for four-wheel turns.
                    element(ode,"fdir1","0 0 1")
                    element(ode,"mu",.10);element(ode,"mu2",1.0)
            joint=element(model,"joint",name=name+"_joint",type="revolute")
            element(joint,"parent","base_link");element(joint,"child",name)
            axis=element(joint,"axis");element(axis,"xyz","0 1 0")
            element(axis,"use_parent_model_frame","true")
    plugin=element(model,"plugin",name="drive",filename="libgazebo_ros_diff_drive.so")
    ros=element(plugin,"ros")
    element(ros,"remapping","cmd_vel:=/saved_test/cmd_vel")
    element(ros,"remapping","odom:=/gazebo_test/odom")
    for key,value in {"num_wheel_pairs":2,"update_rate":50,"max_wheel_torque":100,
                      "max_wheel_acceleration":1.0,"publish_odom":"true","publish_odom_tf":"false",
                      "odometry_source":1,"odometry_frame":"map","robot_base_frame":"base_link"}.items():
        element(plugin,key,value)
    for end in ("front","rear"):
        element(plugin,"left_joint",f"left_{end}_joint")
        element(plugin,"right_joint",f"right_{end}_joint")
        element(plugin,"wheel_separation",.60);element(plugin,"wheel_diameter",.30)
    path=Path(tempfile.gettempdir())/"agribot_saved_path_validation.world"
    ET.ElementTree(sdf).write(path,encoding="unicode")
    return str(path)


def generate_launch_description():
    if os.environ.get("ROS_DOMAIN_ID")!="81" or os.environ.get("ROS_LOCALHOST_ONLY")!="1":
        raise RuntimeError("Gazebo validation requires isolated ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1")
    hardware=Path(get_package_share_directory("agribot_hardware_bringup"))/"differential"
    sim=Path(get_package_share_directory("agribot_differential_fastlivo_mppi"))
    gazebo=Path(get_package_share_directory("gazebo_ros"))
    params=RewrittenYaml(source_file=str(hardware/"config/nav2_params_differential_fastlivo_mapped.yaml"),root_key="",
                         param_rewrites={"use_sim_time":"true","lattice_filepath":str(hardware/"config/motion_primitives/diff_5cm.json")},convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument("gui",default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/"launch/gazebo.launch.py")),
                                 launch_arguments={"world":world_file(),"gui":LaunchConfiguration("gui"),"verbose":"false"}.items()),
        Node(package="nav2_planner",executable="planner_server",name="planner_server",parameters=[params]),
        Node(package="nav2_controller",executable="controller_server",name="controller_server",parameters=[params],remappings=[("cmd_vel","/saved_test/cmd_vel")]),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager",
             parameters=[{"autostart":True,"node_names":["planner_server","controller_server"]}]),
        ExecuteProcess(cmd=["python3",str(sim/"scripts/test_saved_path_gazebo.py"),"--ros-args","-p","use_sim_time:=true"],output="screen"),
    ])
