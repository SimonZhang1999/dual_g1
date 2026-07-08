"""g1_joint_mpc.launch.py
=========================
Full navigation launch using Joint MPC as the global planner + trajectory tracker.

Architecture:
  /scan (2D) → Nav2 global_costmap → /global_costmap/costmap (OccupancyGrid)
                                          ↓
  RViz 2D Nav Goal → /goal_pose → joint_mpc_goal_manager (C++)
                                    ├── /formation/leader_reference  (green path in RViz)
                                    ├── /formation/follower_reference (blue path in RViz)
                                    └── /formation/sfc_markers        (safe corridors)
                                          ↓
  /formation/leader_reference → mpc_path_tracker → /cmd_vel → G1 robot

Usage:
  bash ~/nav_demo.sh          (includes this launch automatically)
  OR standalone:
    ros2 launch mujuco_sim g1_joint_mpc.launch.py
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
SLAM_V3_INSTALL = os.path.expanduser("~/slam_v3/install")
MJLAB_ROOT = os.path.join(WS_SRC, "unitree_rl_mjlab")
DEFAULT_MAP = os.path.join(WS_SRC, "maps", "vln_navigation_room.yaml")
DEFAULT_MPC_PARAMS = os.path.join(WS_SRC, "mujuco_sim", "config", "joint_mpc_params.yaml")
DEFAULT_NAV2_PARAMS = os.path.join(WS_SRC, "mujuco_sim", "config", "g1_joint_mpc_nav2_params.yaml")
DEFAULT_RVIZ = os.path.join(WS_SRC, "mujuco_sim", "map.rviz")


def generate_launch_description():
    domain = LaunchConfiguration("domain")
    network = LaunchConfiguration("network")
    map_file = LaunchConfiguration("map")
    mpc_params = LaunchConfiguration("mpc_params")
    nav2_params = LaunchConfiguration("nav2_params")
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

    return LaunchDescription([
        # ── arguments ────────────────────────────────────────────────
        DeclareLaunchArgument("domain",        default_value="1"),
        DeclareLaunchArgument("network",       default_value="auto"),
        DeclareLaunchArgument("map",           default_value=DEFAULT_MAP),
        DeclareLaunchArgument("mpc_params",    default_value=DEFAULT_MPC_PARAMS),
        DeclareLaunchArgument("nav2_params",   default_value=DEFAULT_NAV2_PARAMS),
        DeclareLaunchArgument("rviz",          default_value="true"),
        DeclareLaunchArgument("rviz_config",   default_value=DEFAULT_RVIZ),
        DeclareLaunchArgument("nav_delay",     default_value="10.0"),

        # ── environment ──────────────────────────────────────────────
        SetEnvironmentVariable("ROS_DOMAIN_ID",      domain),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),

        # ── simulator + g1_ctrl (cmd_vel mode, auto velocity) ────────
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

        # ── map→odom static TF (starts immediately for RViz scan) ────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),

        # ── follower g1_ctrl: real second controller on isolated DDS topics ─
        ExecuteProcess(
            cmd=[
                "env",
                "-u",
                "ROS_DOMAIN_ID",
                "-u",
                "CYCLONEDDS_URI",
                "-u",
                "RMW_IMPLEMENTATION",
                os.path.join(MJLAB_ROOT, "deploy", "robots", "g1", "build", "g1_ctrl"),
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

        # formation_sync_controller: advances sync_wp_idx when BOTH at waypoint
        Node(
            package="mujuco_sim",
            executable="formation_sync_controller",
            name="formation_sync_controller",
            output="screen",
            parameters=[{
                "wp_tolerance_xy":    0.35,
                "wp_tolerance_yaw":   0.40,
                "goal_tolerance_xy":  0.20,
                "goal_tolerance_yaw": 0.25,
                "leader_odom_topic":      "/odom",
                "follower_odom_topic":    "/follower/odom",
                "leader_ref_topic":       "/formation/leader_reference",
                "follower_ref_topic":     "/formation/follower_reference",
                "leader_goal_topic":      "/formation/leader_goal",
                "sync_idx_topic":         "/formation/sync_wp_idx",
                "goal_reached_topic":     "/formation/goal_reached",
                "status_topic":           "/formation/sync_status",
            }],
        ),

        # Leader path tracker (G1 real robot)
        Node(
            package="mujuco_sim",
            executable="mpc_path_tracker",
            name="mpc_path_tracker",
            output="screen",
            parameters=[{
                "leader_reference_topic": "/formation/leader_reference",
                "sync_wp_idx_topic":      "/formation/sync_wp_idx",
                "goal_reached_topic":     "/formation/goal_reached",
                "odom_topic":             "/odom",
                "cmd_vel_topic":          "/cmd_vel",
                "max_vx_forward":         0.30,
                "max_vx_backward":        0.15,
                "max_vy":                 0.20,
                "max_vyaw":               0.50,
                "kp_xy":                  1.2,
                "kp_yaw":                 1.5,
                "decel_radius":           0.80,
            }],
        ),

        # Follower path tracker (second G1 controller)
        Node(
            package="mujuco_sim",
            executable="follower_path_tracker",
            name="follower_path_tracker",
            output="screen",
            parameters=[{
                "follower_reference_topic": "/formation/follower_reference",
                "sync_wp_idx_topic":        "/formation/sync_wp_idx",
                "goal_reached_topic":       "/formation/goal_reached",
                "follower_odom_topic":      "/follower/odom",
                "follower_cmd_topic":       "/follower/cmd_vel",
                "max_vx_forward":           0.30,
                "max_vx_backward":          0.15,
                "max_vy":                   0.20,
                "max_vyaw":                 0.50,
                "kp_xy":                    1.2,
                "kp_yaw":                   1.5,
                "decel_radius":             0.80,
            }],
        ),

        # ── Nav2 costmap + MPC planner (delayed for sim warm-up) ─────
        TimerAction(
            period=nav_delay,
            actions=[
                # map_server
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[nav2_params, {"yaml_filename": map_file}],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                # lifecycle manager (map_server only)
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
                # Nav2 navigation stack (for global_costmap + local_costmap)
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_navigation_launch),
                    launch_arguments={
                        "use_sim_time":    "false",
                        "params_file":     nav2_params,
                        "autostart":       "true",
                        "use_composition": "False",
                    }.items(),
                ),
                # Joint MPC goal manager (C++ from slam_v3)
                Node(
                    package="g1_joint_mpc_planner",
                    executable="joint_mpc_goal_manager",
                    name="joint_mpc_goal_manager",
                    output="screen",
                    parameters=[mpc_params],
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
    ])
