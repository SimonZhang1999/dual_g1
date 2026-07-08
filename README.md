# G1 MuJoCo ROS2 Navigation Simulation

This workspace integrates Unitree G1 MuJoCo simulation, MJLab policy deployment, ROS2 Navigation, 2D SLAM, and Mid360 / FAST-LIO mapping into one reproducible workflow. The goal is to validate navigation, perception, and locomotion policies in MuJoCo before moving toward a real G1 robot.

It is strongly recommended to clone and use this complete workspace directly instead of re-cloning each upstream repository and manually stitching them together. This project contains small compatibility changes to Unitree, MJLab, FAST-LIO, Livox, and related packages. Using the full workspace avoids many version and interface mismatches.

## Features

- Start Unitree G1 MuJoCo simulation with `unitree_mujoco`.
- Deploy and run the G1 locomotion policy with `unitree_rl_mjlab/deploy/robots/g1/g1_ctrl`.
- Control the G1 from Nav2 `/cmd_vel`.
- Publish MuJoCo data to ROS2 through a bridge:
  - `/scan`
  - `/livox/lidar`
  - `/imu/data`
  - `/odom`
  - `/tf`
- Simulate a Livox Mid360 point cloud sensor mounted on the G1 head.
- Build 2D maps with `slam_toolbox`.
- Run 3D PCD mapping and localization experiments with `FAST_LIO_ROS2`.
- Run Nav2 navigation with saved 2D `.yaml/.pgm` maps.

## Data Flow

```text
Nav2 goal
  -> Nav2 planner/controller
  -> /cmd_vel
  -> g1_ctrl --cmd_vel
  -> MJLab / Unitree walking policy
  -> Unitree MuJoCo robot motion
  -> shared memory
  -> mujuco_sim bridge
  -> /scan /livox/lidar /imu/data /tf
  -> SLAM / FAST-LIO / Nav2 / RViz
```

## Workspace Layout

After creating a workspace and cloning this repository, the `src` directory should look similar to this:

```text
src/
  FAST_LIO_ROS2
  Livox-SDK2
  lidar_localization_ros2
  livox_ros_driver2
  maps
  mujuco_sim
  unitree_mujoco
  unitree_rl_mjlab
  unitree_ros2
  unitree_sdk2
```

Main directories:

```text
src/mujuco_sim/              # ROS2 launch files and MuJoCo-ROS bridge nodes
src/unitree_mujoco/          # Unitree MuJoCo simulator with nav shared memory and Mid360 simulation
src/unitree_rl_mjlab/        # MJLab G1 policy deployment and g1_ctrl
src/FAST_LIO_ROS2/           # FAST-LIO2 ROS2 package
src/Livox-SDK2/              # Livox SDK2 dependency
src/livox_ros_driver2/       # Livox ROS2 driver, mainly for real robot use
src/unitree_sdk2/            # Unitree SDK2
src/unitree_ros2/            # Unitree ROS2 examples and interfaces
src/maps/                    # 2D maps and saved PCD maps
```

## Modified Upstream Files

If you do not clone this complete workspace and instead download upstream repositories manually, at least these changes need to be carried over:

- `src/unitree_mujoco/simulate/src/main.cc`
  - Connects the navigation shared-memory bridge.
- `src/unitree_mujoco/simulate/src/shm_nav_bridge.cc`
  - Generates shared-memory data for `/scan`, Mid360 point cloud, IMU, and robot pose from MuJoCo.
- `src/unitree_mujoco/simulate/src/shm_nav_bridge.h`
  - Defines the shared-memory data structure.
- `src/unitree_mujoco/simulate/config/mid360_pattern.csv`
  - Simulated Mid360 scan pattern.
- `src/unitree_mujoco/simulate/config.yaml`
  - Selects the current simulation scene.
- `src/unitree_mujoco/unitree_robots/g1/*.xml`
  - G1 navigation scenes, VLN rooms, LiDAR site, and related MuJoCo XML changes.
- `src/unitree_mujoco/example/COLCON_IGNORE`
  - Avoids duplicate `stand_go2` package names during colcon builds.
- `src/unitree_rl_mjlab/deploy/robots/g1/main.cpp`
  - Adds keyboard, default, and `/cmd_vel` control modes.
- `src/unitree_rl_mjlab/deploy/include/input/velocity_command_source.h`
  - Unifies velocity command sources.
- `src/unitree_rl_mjlab/deploy/include/param.h`
  - Deployment parameter adaptations.
- `src/FAST_LIO_ROS2/config/mid360.yaml`
  - FAST-LIO config for simulated Mid360 plus MuJoCo IMU.
- `src/FAST_LIO_ROS2/config/mid360_real.yaml`
  - FAST-LIO config for real Mid360 experiments.
- `src/FAST_LIO_ROS2/src/laserMapping.cpp`
  - Adds the `/map_save` service for manual PCD saving.
  - The `foxy` branch includes ROS2 Foxy service callback compatibility changes.
- `src/mujuco_sim/`
  - New ROS2 package containing launch files, Nav2 params, RViz config, MuJoCo shared-memory bridge, and FAST-LIO TF bridge.

For this reason, the recommended path is to use the complete workspace from this repository. If you use upstream repositories directly, replace the files above with this repository's versions.

## Requirements

The main branch was tested mainly on Ubuntu 22.04 + ROS2 Humble. For Ubuntu 20.04 + ROS2 Foxy, use the `foxy` branch because Foxy needs extra compatibility changes for some ROS2 C++ APIs, Livox driver builds, and FAST-LIO.

Install ROS2 navigation dependencies. Humble example:

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rmw-cyclonedds-cpp
```

Foxy example:

```bash
sudo apt install \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-slam-toolbox \
  ros-foxy-rviz2 \
  ros-foxy-tf2-ros \
  ros-foxy-pcl-ros \
  ros-foxy-pcl-conversions \
  ros-foxy-rmw-cyclonedds-cpp \
  libeigen3-dev \
  libpcl-dev \
  libapr1-dev \
  libpcap-dev \
  libusb-1.0-0-dev
```

`rmw_cyclonedds_cpp` is important. The launch files use CycloneDDS by default. If the package is missing, you may see an error similar to:

```text
failed to find shared library 'rmw_cyclonedds_cpp'
```

Follow Unitree official documentation to configure Unitree MuJoCo, Unitree SDK2, and Unitree RL MJLab. This project assumes that you can already run Unitree MuJoCo and the G1 `g1_ctrl` controller independently. Full workspace builds and real LiDAR workflows also require Livox-SDK2, `livox_ros_driver2`, and FAST-LIO.

## Build Unitree MuJoCo

If building `unitree_mujoco/simulate` fails with missing MuJoCo, `glfw_adapter.h`, or `-lmujoco`, MuJoCo C/C++ SDK is not installed or not linked correctly.

Install MuJoCo 3.2.7:

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
find mujoco-3.2.7 -name "glfw_adapter.h"
find mujoco-3.2.7 -name "libmujoco.so"
```

On Ubuntu 20.04 / ROS2 Foxy, if linking fails with:

```text
undefined reference to `shm_open'
```

add `rt` to `link_libraries(...)` in `src/unitree_mujoco/simulate/CMakeLists.txt`:

```cmake
link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
  rt
)
```

Then rebuild:

```bash
cd ~/{your_workspace}/src/unitree_mujoco/simulate
rm -rf build
mkdir build
cd build
cmake .. \
  -DMUJOCO_DIR=$HOME/.mujoco/mujoco-3.2.7 \
  -DCMAKE_PREFIX_PATH=$HOME/.mujoco/mujoco-3.2.7
make -j4
```

Run:

```bash
./unitree_mujoco
```

If the G1 simulation scene appears in MuJoCo, this step is working.

## Build MJLab / G1 Controller

```bash
cd ~/{your_workspace}/src/unitree_rl_mjlab/deploy/robots/g1
mkdir -p build
cd build
cmake ..
make -j4
```

Run keyboard control:

```bash
./g1_ctrl --network=lo --domain=1 --keyboard
```

After the controller detects the robot, click reset in the MuJoCo window. The robot should enter the FixStand / Velocity flow. Use the keyboard in the `g1_ctrl` terminal:

```text
w/s: x direction
 a/d: y direction
q/e: yaw direction
```

If the robot walks in the simulation, the basic control path is working.

## Build Livox-SDK2 and livox_ros_driver2

If you plan to use a real Mid360, or if you want to build the full workspace including Livox packages, configure Livox-SDK2 first. MuJoCo simulation itself does not require the real Livox driver because the simulated `/livox/lidar` topic is published by the `mujuco_sim` bridge.

For ROS2 Foxy, build and install Livox-SDK2:

```bash
cd ~/{your_workspace}/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

Check the installation:

```bash
find /usr/local -name "livox_lidar_api.h"
ldconfig -p | grep livox
```

Build `livox_ros_driver2`:

```bash
cd ~/{your_workspace}
source /opt/ros/foxy/setup.bash
colcon build --packages-select livox_ros_driver2 \
  --cmake-args \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=/usr/local/include \
  -DLIVOX_LIDAR_SDK_LIBRARY=/usr/local/lib/liblivox_lidar_sdk_shared.so
```

If `livox_ros_driver2` reports that `package.xml` is missing, use this repository's version or copy `package_ROS2.xml` to `package.xml`. The recommended approach is still to use this complete workspace to avoid ROS distro and Livox driver compatibility issues.

## FAST-LIO Foxy Compatibility

The `foxy` branch already contains Foxy-compatible changes in `src/FAST_LIO_ROS2/src/laserMapping.cpp`. If you download FAST-LIO from upstream manually, pay attention to the `/map_save` service callback signature. Foxy should use a three-argument callback.

Recommended `create_service` code:

```cpp
map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "map_save",
    std::bind(
        &LaserMappingNode::map_save_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
```

Recommended callback code:

```cpp
void map_save_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)request_header;
    (void)req;

    RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
    if (pcd_save_en)
    {
        save_to_pcd();
        res->success = true;
        res->message = "Map saved.";
    }
    else
    {
        res->success = false;
        res->message = "Map save disabled.";
    }
}
```

Build FAST-LIO:

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
colcon build --packages-select fast_lio
source install/setup.bash
ros2 pkg prefix fast_lio
```

If `ros2 pkg prefix fast_lio` prints a path, FAST-LIO is installed correctly.

## Build the Full Workspace

On a new machine, or after copying this workspace, clean old build outputs first:

```bash
cd ~/{your_workspace}
rm -rf build install log
source /opt/ros/{your_ros_distro}/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

The launch files contain a workspace source path. Update `WS_SRC` in these files to match your workspace name:

```text
src/mujuco_sim/launch/g1_nav_sim.launch.py
src/mujuco_sim/launch/map.launch.py
src/mujuco_sim/launch/nav.launch.py
```

For example, if your workspace is named `g1_ws`:

```python
WS_SRC = os.path.expanduser("~/g1_ws/src")
```

After editing launch files, rebuild and source:

```bash
cd ~/{your_workspace}
colcon build --packages-select mujuco_sim
source install/setup.bash
```

If you see a duplicate `stand_go2` package error, make sure this file exists:

```text
src/unitree_mujoco/example/COLCON_IGNORE
```

If launch logs still reference an old workspace such as `~/yushu_ws/install`, the shell environment is polluted by an old workspace. Open a new terminal or run:

```bash
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/{your_ros_distro}/setup.bash
source ~/{your_workspace}/install/setup.bash
```

Also check `~/.bashrc` and avoid automatically sourcing an old workspace.

## Run MuJoCo + G1 Control

Keyboard control mode:

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=keyboard
```

Nav2 `/cmd_vel` control mode:

```bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=cmd_vel
```

Useful checks:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## 2D Mapping With slam_toolbox

Edit the top of `src/mujuco_sim/launch/map.launch.py`:

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = True
```

Start mapping:

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

Drive the robot with the keyboard controller, then save the 2D map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/{your_workspace}/src/maps/new_2d_map
```

## FAST-LIO PCD Mapping

Edit the top of `src/mujuco_sim/launch/map.launch.py`:

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = False
```

Start FAST-LIO mapping:

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

Drive the robot around the environment. Do not rely on `Ctrl+C` to save the PCD. Explicitly call the save service:

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

The simulated FAST-LIO PCD output path is configured in:

```text
src/FAST_LIO_ROS2/config/mid360.yaml
```

By default it saves to:

```text
./src/maps/vln_fastlio_map.pcd
```

Example FAST-LIO 3D PCD mapping result:

![FAST-LIO 3D PCD mapping result](images/pcd.png)

## Navigation

Make sure `src/mujuco_sim/launch/nav.launch.py` points to the desired 2D map. The default map is:

```text
src/maps/vln_navigation_room.yaml
```

Start navigation:

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim nav.launch.py
```

Send a `2D Goal Pose` in RViz. Nav2 publishes `/cmd_vel`, and `g1_ctrl --cmd_vel` receives it to drive the G1 in MuJoCo.

Example Nav2 simulation results:

![MuJoCo Mid360 point cloud with Nav2 map](images/mujoco_3Dcloud.png)

![FAST-LIO localization with Nav2 navigation](images/nav2_fastlio.png)

## Real Robot Notes

For a real G1 robot:

1. Configure Unitree network, SDK2, and ROS2 according to Unitree official documentation.
2. Start the real Mid360 driver and confirm that `/livox/lidar` exists.
3. If you want to build a 2D map with `slam_toolbox`, convert Mid360 `PointCloud2` to `/scan` with `pointcloud_to_laserscan`.
4. Save the `.yaml/.pgm` map and use it with Nav2.
5. Run `g1_ctrl` on the real robot network interface, for example:

```bash
cd ~/{your_workspace}/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=enp5s0
```

Validate keyboard control and policy deployment first, then switch to `/cmd_vel` mode for Nav2 integration.

## Launch Files

- `g1_nav_sim.launch.py`: starts Unitree MuJoCo, `g1_ctrl`, and the shared-memory ROS2 bridge.
- `map.launch.py`: starts mapping; switch between 2D `slam_toolbox` and FAST-LIO PCD mapping at the top of the file.
- `nav.launch.py`: starts simulation, FAST-LIO localization bridge, Nav2, and RViz.

## DDS Notes

Simulation defaults to `ROS_DOMAIN_ID=1` and uses CycloneDDS loopback config:

```text
src/mujuco_sim/config/cyclonedds_lo.xml
```

The launch files set these environment variables automatically. If `rmw_cyclonedds_cpp` is missing, install the package for your ROS distro:

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
# or
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
