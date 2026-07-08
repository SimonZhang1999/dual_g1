"""
g1_nav_full.launch.py
======================
One-shot navigation launch:
  1. MuJoCo simulator (navigation_room_29dof.xml)
  2. g1_ctrl in cmd_vel mode (auto-enters Velocity when nav sends commands)
  3. unitree_mujoco_shm_bridge  -> /scan, /odom, /imu/data, TF odom->base_link
  4. map_server with g1_navigation_room map
  5. AMCL for map->odom localisation
  6. Nav2 navigation stack (MPPI holonomic controller)
  7. RViz2 for goal setting and monitoring

Usage:
  ros2 launch mujuco_sim g1_nav_full.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WS_SRC = os.path.expanduser("~/g1-mujoco-ros2-nav-sim-G1-MuJoCo-ROS2-/src")
MJLAB_ROOT = os.path.join(WS_SRC, "unitree_rl_mjlab")
DEFAULT_MAP = os.path.join(WS_SRC, "maps", "vln_navigation_room.yaml")
DEFAULT_PARAMS = os.path.join(WS_SRC, "mujuco_sim", "config", "g1_nav2_params.yaml")
DEFAULT_RVIZ = os.path.join(WS_SRC, "mujuco_sim", "map.rviz")


def generate_launch_description():
    domain = LaunchConfiguration("domain")
    network = LaunchConfiguration("network")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav_delay = LaunchConfiguration("nav_delay")

    pkg_share = FindPackageShare("mujuco_sim")
    nav2_share = FindPackageShare("nav2_bringup")

    g1_nav_sim_launch = PathJoinSubstitution(
        [pkg_share, "launch", "g1_nav_sim.launch.py"]
    )
    nav2_navigation_launch = PathJoinSubstitution(
        [nav2_share, "launch", "navigation_launch.py"]
    )

    return LaunchDescription(
        [
            # ── arguments ────────────────────────────────────────────────────
            DeclareLaunchArgument("domain",      default_value="1"),
            DeclareLaunchArgument("network",     default_value="auto"),
            DeclareLaunchArgument("map",         default_value=DEFAULT_MAP),
            DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS),
            DeclareLaunchArgument("rviz",        default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=DEFAULT_RVIZ),
            # nav_delay: how many seconds to wait before starting Nav2
            # (gives the simulator time to load and AMCL time to get a scan)
            DeclareLaunchArgument("nav_delay",   default_value="8.0"),

            # ── environment ──────────────────────────────────────────────────
            SetEnvironmentVariable("ROS_DOMAIN_ID",        domain),
            SetEnvironmentVariable("RMW_IMPLEMENTATION",   "rmw_cyclonedds_cpp"),

            # ── 1+2: simulator + g1_ctrl (cmd_vel, auto velocity) ───────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(g1_nav_sim_launch),
                launch_arguments={
                    "network":                  network,
                    "domain":                   domain,
                    "input":                    "cmd_vel",
                    "auto_start_velocity":      "true",
                    "start_nav_bridge":         "true",
                    "publish_robot_odom":       "true",
                    "publish_sensor_tf_static": "true",
                    "g1_ctrl_terminal":         "false",
                }.items(),
            ),

            # ── follower: second real g1_ctrl on isolated low-level topics ───
            ExecuteProcess(
                cmd=[
                    "env",
                    "-u",
                    "ROS_DOMAIN_ID",
                    "-u",
                    "CYCLONEDDS_URI",
                    "-u",
                    "RMW_IMPLEMENTATION",
                    os.path.join(
                        MJLAB_ROOT,
                        "deploy",
                        "robots",
                        "g1",
                        "build",
                        "g1_ctrl",
                    ),
                    "--network",
                    network,
                    "--domain",
                    domain,
                    "--cmd_vel",
                    "--cmd_vel_topic",
                    "/follower/cmd_vel",
                    "--lowcmd_topic",
                    "rt/follower/lowcmd",
                    "--lowstate_topic",
                    "rt/follower/lowstate",
                ],
                cwd=os.path.join(MJLAB_ROOT, "deploy", "robots", "g1", "build"),
                output="screen",
            ),

            # ── map→odom static TF starts immediately so RViz can render /scan ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),

            # Follower velocity target. The real follower g1_ctrl consumes this
            # and produces low-level commands for the f2_ MuJoCo robot.
            Node(
                package="mujuco_sim",
                executable="leader_follower_controller",
                name="leader_follower_controller",
                output="screen",
                parameters=[{
                    "leader_odom_topic": "/odom",
                    "follower_odom_topic": "/follower/odom",
                    "follower_cmd_topic": "/follower/cmd_vel",
                    "target_offset_x": 1.8,
                    "target_offset_y": 0.0,
                    "target_yaw_offset": 3.141592653589793,
                    "max_vx_forward": 0.30,
                    "max_vx_backward": 0.15,
                    "max_vy": 0.20,
                    "max_vyaw": 0.50,
                    "kp_xy": 1.1,
                    "kp_yaw": 1.5,
                }],
            ),

            # ── 3+4+5: Nav2 stack (delayed to let sim warm up) ───────────────
            TimerAction(
                period=nav_delay,
                actions=[
                    # map_server
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            params_file,
                            {"yaml_filename": map_file},
                        ],
                        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                    ),
                    # lifecycle manager for map_server only (no AMCL)
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        output="screen",
                        parameters=[
                            {"use_sim_time": False},
                            {"autostart": True},
                            {"node_names": ["map_server"]},
                        ],
                    ),
                    # full Nav2 navigation stack
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav2_navigation_launch),
                        launch_arguments={
                            "use_sim_time":   "false",
                            "params_file":    params_file,
                            "autostart":      "true",
                            "use_composition": "False",
                        }.items(),
                    ),
                    # RViz2
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config],
                        condition=IfCondition(rviz),
                    ),
                ],
            ),
        ]
    )
