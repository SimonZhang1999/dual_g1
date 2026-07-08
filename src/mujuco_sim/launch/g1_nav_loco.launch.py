import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


WS_ROOT = os.path.expanduser("~/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-")
BASE_LAUNCH = os.path.join(WS_ROOT, "src", "mujuco_sim", "launch", "g1_nav_sim.launch.py")
G1_CTRL_BIN = os.path.join(
    WS_ROOT,
    "src",
    "unitree_rl_mjlab",
    "deploy",
    "robots",
    "g1",
    "build",
    "g1_ctrl",
)
G1_CTRL_CWD = os.path.join(
    WS_ROOT,
    "src",
    "unitree_rl_mjlab",
    "deploy",
    "robots",
    "g1",
    "build",
)


def generate_launch_description():
    domain = LaunchConfiguration("domain")
    input_mode = LaunchConfiguration("input")
    auto_start_velocity = LaunchConfiguration("auto_start_velocity")
    start_nav_bridge = LaunchConfiguration("start_nav_bridge")
    g1_ctrl_terminal = LaunchConfiguration("g1_ctrl_terminal")
    return LaunchDescription(
        [
            DeclareLaunchArgument("domain", default_value="1"),
            DeclareLaunchArgument("input", default_value="keyboard"),
            DeclareLaunchArgument("auto_start_velocity", default_value="false"),
            DeclareLaunchArgument("start_nav_bridge", default_value="true"),
            DeclareLaunchArgument("g1_ctrl_terminal", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(BASE_LAUNCH),
                launch_arguments={
                    "domain": domain,
                    "g1_ctrl_terminal": g1_ctrl_terminal,
                    "input": input_mode,
                    "auto_start_velocity": auto_start_velocity,
                    "start_nav_bridge": start_nav_bridge,
                    "g1_ctrl_bin": G1_CTRL_BIN,
                    "g1_ctrl_cwd": G1_CTRL_CWD,
                }.items(),
            )
        ]
    )