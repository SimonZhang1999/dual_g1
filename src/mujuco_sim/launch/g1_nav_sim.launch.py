import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


WS_SRC = os.path.expanduser("~/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src")
MUJOCO_ROOT = os.path.join(WS_SRC, "unitree_mujoco")
MJLAB_ROOT = os.path.join(WS_SRC, "unitree_rl_mjlab")
MUJUCO_SIM_ROOT = os.path.join(WS_SRC, "mujuco_sim")


def generate_launch_description():
    unitree_mujoco_bin = LaunchConfiguration("unitree_mujoco_bin")
    unitree_mujoco_cwd = LaunchConfiguration("unitree_mujoco_cwd")
    g1_ctrl_bin = LaunchConfiguration("g1_ctrl_bin")
    g1_ctrl_cwd = LaunchConfiguration("g1_ctrl_cwd")
    network = LaunchConfiguration("network")
    domain = LaunchConfiguration("domain")
    input_mode = LaunchConfiguration("input")
    start_mujoco = LaunchConfiguration("start_mujoco")
    start_g1_ctrl = LaunchConfiguration("start_g1_ctrl")
    start_nav_bridge = LaunchConfiguration("start_nav_bridge")
    start_camera_bridge = LaunchConfiguration("start_camera_bridge")
    publish_robot_odom = LaunchConfiguration("publish_robot_odom")
    publish_sensor_tf_static = LaunchConfiguration("publish_sensor_tf_static")
    mujoco_terminal = LaunchConfiguration("mujoco_terminal")
    g1_ctrl_terminal = LaunchConfiguration("g1_ctrl_terminal")
    auto_start_velocity = LaunchConfiguration("auto_start_velocity")
    camera_model_path = LaunchConfiguration("camera_model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "unitree_mujoco_bin",
                default_value=os.path.join(
                    MUJOCO_ROOT, "simulate", "build", "unitree_mujoco"
                ),
            ),
            DeclareLaunchArgument(
                "unitree_mujoco_cwd",
                default_value=os.path.join(MUJOCO_ROOT, "simulate", "build"),
            ),
            DeclareLaunchArgument(
                "g1_ctrl_bin",
                default_value=os.path.join(
                    MJLAB_ROOT, "deploy", "robots", "g1", "build", "g1_ctrl"
                ),
            ),
            DeclareLaunchArgument(
                "g1_ctrl_cwd",
                default_value=os.path.join(
                    MJLAB_ROOT, "deploy", "robots", "g1", "build"
                ),
            ),
            DeclareLaunchArgument("network", default_value="auto"),
            DeclareLaunchArgument("domain", default_value="1"),
            DeclareLaunchArgument("input", default_value="keyboard"),
            DeclareLaunchArgument("start_mujoco", default_value="true"),
            DeclareLaunchArgument("start_g1_ctrl", default_value="true"),
            DeclareLaunchArgument("start_nav_bridge", default_value="true"),
            DeclareLaunchArgument("start_camera_bridge", default_value="false"),
            DeclareLaunchArgument("publish_robot_odom", default_value="true"),
            DeclareLaunchArgument("publish_sensor_tf_static", default_value="false"),
            DeclareLaunchArgument("mujoco_terminal", default_value="false"),
            DeclareLaunchArgument("g1_ctrl_terminal", default_value="true"),
            DeclareLaunchArgument("auto_start_velocity", default_value="false"),
            DeclareLaunchArgument(
                "camera_model_path",
                default_value=os.path.join(
                    MUJOCO_ROOT, "unitree_robots", "g1", "navigation_room_29dof.xml"
                ),
            ),
            SetEnvironmentVariable("ROS_DOMAIN_ID", domain),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            ExecuteProcess(
                cmd=[
                    unitree_mujoco_bin,
                    "--network",
                    network,
                    "--domain_id",
                    domain,
                ],
                cwd=unitree_mujoco_cwd,
                additional_env={
                    "ROS_DOMAIN_ID": domain,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                },
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_mujoco, "' == 'true' and '", mujoco_terminal, "' != 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--title",
                    "unitree_mujoco",
                    "--",
                    "bash",
                    "-lc",
                    PythonExpression(
                        [
                            "'export ROS_DOMAIN_ID=",
                            domain,
                            "; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
                            "; cd ",
                            unitree_mujoco_cwd,
                            " && ",
                            unitree_mujoco_bin,
                            " --network ",
                            network,
                            " --domain_id ",
                            domain,
                            "; exec bash'",
                        ]
                    ),
                ],
                additional_env={
                    "ROS_DOMAIN_ID": domain,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                },
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_mujoco, "' == 'true' and '", mujoco_terminal, "' == 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "env",
                    "-u",
                    "ROS_DOMAIN_ID",
                    "-u",
                    "CYCLONEDDS_URI",
                    "-u",
                    "RMW_IMPLEMENTATION",
                    g1_ctrl_bin,
                    "--network",
                    network,
                    "--domain",
                    domain,
                    PythonExpression(
                        [
                            "'--keyboard' if '",
                            input_mode,
                            "' == 'keyboard' else '--cmd_vel' if '",
                            input_mode,
                            "' == 'cmd_vel' else ''",
                        ]
                    ),
                ],
                cwd=g1_ctrl_cwd,
                additional_env={},
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_g1_ctrl, "' == 'true' and '", g1_ctrl_terminal, "' != 'true' and '", auto_start_velocity, "' == 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "env",
                    "-u",
                    "ROS_DOMAIN_ID",
                    "-u",
                    "CYCLONEDDS_URI",
                    "-u",
                    "RMW_IMPLEMENTATION",
                    g1_ctrl_bin,
                    "--network",
                    network,
                    "--domain",
                    domain,
                    PythonExpression(
                        [
                            "'--keyboard' if '",
                            input_mode,
                            "' == 'keyboard' else '--cmd_vel' if '",
                            input_mode,
                            "' == 'cmd_vel' else ''",
                        ]
                    ),
                    "--no_auto_velocity",
                ],
                cwd=g1_ctrl_cwd,
                additional_env={},
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_g1_ctrl, "' == 'true' and '", g1_ctrl_terminal, "' != 'true' and '", auto_start_velocity, "' != 'true'"]
                    )
                ),
            ),
            ExecuteProcess(
                cmd=[
                    "gnome-terminal",
                    "--title",
                    "g1_ctrl",
                    "--",
                    "bash",
                    "-lc",
                    PythonExpression(
                        [
                            "'export ROS_DOMAIN_ID=",
                            domain,
                            "; cd ",
                            g1_ctrl_cwd,
                            " && ",
                            g1_ctrl_bin,
                            " --network ",
                            network,
                            " --domain ",
                            domain,
                            " ' + ('--keyboard' if '",
                            input_mode,
                            "' == 'keyboard' else '--cmd_vel' if '",
                            input_mode,
                            "' == 'cmd_vel' else '') + (' --no_auto_velocity' if '",
                            auto_start_velocity,
                            "' != 'true' else '') + '; exec bash'",
                        ]
                    ),
                ],
                additional_env={"ROS_DOMAIN_ID": domain},
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", start_g1_ctrl, "' == 'true' and '", g1_ctrl_terminal, "' == 'true'"]
                    )
                ),
            ),
            Node(
                package="mujuco_sim",
                executable="unitree_mujoco_shm_bridge",
                name="unitree_mujoco_shm_bridge",
                output="screen",
                parameters=[
                    {"publish_robot_odom": publish_robot_odom},
                    {"publish_sensor_tf_static": publish_sensor_tf_static},
                    # Lidar site is ~1.25 m above ground (pelvis 0.793 + body chain + site 0.405)
                    {"laser_xyz": "0.0,0.0,1.25"},
                ],
                condition=IfCondition(start_nav_bridge),
            ),
            Node(
                package="mujuco_sim",
                executable="mujoco_camera_bridge",
                name="mujoco_camera_bridge",
                output="screen",
                parameters=[
                    {"model_path": camera_model_path},
                    {"camera_name": "head_camera"},
                    {"width": 640},
                    {"height": 480},
                    {"fps": 30.0},
                    {"camera_xyz": "0.10,0.0,0.34"},
                    {"horizontal_fov_deg": 69.4},
                ],
                condition=IfCondition(start_camera_bridge),
            ),
        ]
    )
