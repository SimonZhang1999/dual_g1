# G1 MuJoCo ROS2 导航联合仿真

这个工作空间把 Unitree G1 MuJoCo 仿真、MJLab 策略部署、ROS2 Navigation、2D SLAM，以及 Mid360 / FAST-LIO 建图流程整合到一起。目标是在 MuJoCo 里验证导航、感知和行走策略，再逐步迁移到真实 G1 机器人。

强烈建议直接 clone 本仓库提供的完整 workspace 使用，不建议重新从各个上游仓库分别 clone 后再手动拼装。原因是本项目对 Unitree、MJLab、FAST-LIO、Livox 等开源包做过少量适配修改，直接使用完整 workspace 可以避免版本和接口兼容问题。

## 实现功能

- 使用 `unitree_mujoco` 启动 Unitree G1 MuJoCo 仿真。
- 使用 `unitree_rl_mjlab/deploy/robots/g1/g1_ctrl` 部署并运行 G1 行走策略。
- 支持 Nav2 输出 `/cmd_vel` 控制 G1 行走。
- MuJoCo 到 ROS2 的桥接节点发布：
  - `/scan`
  - `/livox/lidar`
  - `/imu/data`
  - `/odom`
  - `/tf`
- 在 G1 头部模拟 Livox Mid360 三维点云。
- 支持 `slam_toolbox` 进行 2D 建图。
- 支持 `FAST_LIO_ROS2` 进行 3D PCD 建图和定位实验。
- 支持 Nav2 读取保存好的 2D `.yaml/.pgm` 地图进行导航。

## 总体数据流

```text
Nav2 目标点
  -> Nav2 planner/controller
  -> /cmd_vel
  -> g1_ctrl --cmd_vel
  -> MJLab / Unitree 行走策略
  -> Unitree MuJoCo 机器人运动
  -> shared memory
  -> mujuco_sim bridge
  -> /scan /livox/lidar /imu/data /tf
  -> SLAM / FAST-LIO / Nav2 / RViz
```

## 目录结构

新建工作空间并 clone 本仓库后，`src` 目录结构应类似：

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

主要目录说明：

```text
src/mujuco_sim/              # ROS2 launch 文件和 MuJoCo-ROS 桥接节点
src/unitree_mujoco/          # Unitree MuJoCo 仿真器，已加入导航共享内存和 Mid360 仿真
src/unitree_rl_mjlab/        # MJLab G1 策略部署和 g1_ctrl
src/FAST_LIO_ROS2/           # FAST-LIO2 ROS2 包
src/Livox-SDK2/              # Livox SDK2 依赖
src/livox_ros_driver2/       # Livox ROS2 驱动，主要用于实机
src/unitree_sdk2/            # Unitree SDK2
src/unitree_ros2/            # Unitree ROS2 示例和接口
src/maps/                    # 2D 地图和保存的 PCD 地图
```

## 本项目修改过的上游文件

如果你没有直接 clone 本 workspace，而是自己从上游仓库重新下载，需要至少同步这些改动，否则功能可能不完整：

- `src/unitree_mujoco/simulate/src/main.cc`
  - 接入导航共享内存 bridge。
- `src/unitree_mujoco/simulate/src/shm_nav_bridge.cc`
  - 从 MuJoCo 中生成 `/scan`、Mid360 点云、IMU、位姿等共享内存数据。
- `src/unitree_mujoco/simulate/src/shm_nav_bridge.h`
  - 定义共享内存数据结构。
- `src/unitree_mujoco/simulate/config/mid360_pattern.csv`
  - Mid360 仿真扫描 pattern。
- `src/unitree_mujoco/simulate/config.yaml`
  - 配置当前仿真场景。
- `src/unitree_mujoco/unitree_robots/g1/*.xml`
  - G1 导航场景、VLN 房间、雷达 site 等 MuJoCo XML。
- `src/unitree_mujoco/example/COLCON_IGNORE`
  - 避免 `stand_go2` 重名导致 colcon 编译失败。
- `src/unitree_rl_mjlab/deploy/robots/g1/main.cpp`
  - 支持键盘、默认控制、`/cmd_vel` 控制模式。
- `src/unitree_rl_mjlab/deploy/include/input/velocity_command_source.h`
  - 统一速度命令来源。
- `src/unitree_rl_mjlab/deploy/include/param.h`
  - 与部署参数相关的适配。
- `src/FAST_LIO_ROS2/config/mid360.yaml`
  - 仿真 Mid360 + MuJoCo IMU 的 FAST-LIO 配置。
- `src/FAST_LIO_ROS2/config/mid360_real.yaml`
  - 实机 Mid360 配置。
- `src/FAST_LIO_ROS2/src/laserMapping.cpp`
  - 增加 `/map_save` 服务，用于手动保存 PCD。
  - Foxy 分支中对 service callback 做了 Foxy 兼容适配。
- `src/mujuco_sim/`
  - 本项目新增的 ROS2 包，包含 launch、Nav2 参数、RViz 配置、MuJoCo 共享内存 bridge、FAST-LIO TF bridge。

因此，推荐做法是直接使用本仓库完整 workspace；如果你只 git 上游开源包，则需要把以上文件替换为本仓库版本。

## 环境要求

主分支主要在 Ubuntu 22.04 + ROS2 Humble 下测试。Ubuntu 20.04 + ROS2 Foxy 请优先使用本仓库的 `foxy` 分支，因为 Foxy 对部分 ROS2 C++ API、Livox 驱动和 FAST-LIO 编译方式有额外适配。

安装 ROS2 导航相关依赖。Humble 示例：

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

Foxy 示例：

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

`rmw_cyclonedds_cpp` 很重要。本项目的 launch 默认使用 CycloneDDS，如果缺少对应 ROS 版本的 RMW 包，会出现类似：

```text
failed to find shared library 'rmw_cyclonedds_cpp'
```

Unitree MuJoCo、Unitree SDK2、Unitree RL MJLab 的基础环境配置请参考 Unitree 官方文档。本项目默认你已经能够单独运行 Unitree MuJoCo 和 G1 `g1_ctrl` 控制程序。完整编译和实机雷达相关功能还需要配置 Livox-SDK2、`livox_ros_driver2` 和 FAST-LIO。

## 编译 Unitree MuJoCo

如果编译 `unitree_mujoco/simulate` 时提示找不到 MuJoCo、`glfw_adapter.h` 或 `-lmujoco`，说明 MuJoCo C/C++ SDK 没有安装或没有正确链接。

安装 MuJoCo 3.2.7 示例：

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz
tar -xzf mujoco-3.2.7-linux-x86_64.tar.gz
find mujoco-3.2.7 -name "glfw_adapter.h"
find mujoco-3.2.7 -name "libmujoco.so"
```

在 Ubuntu 20.04 / ROS2 Foxy 上，如果链接时报：

```text
undefined reference to `shm_open'
```

需要修改 `src/unitree_mujoco/simulate/CMakeLists.txt`，在 `link_libraries(...)` 中加入 `rt`。例如：

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

然后重新编译：

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

编译成功后运行：

```bash
./unitree_mujoco
```

能看到 MuJoCo 中的 G1 仿真环境说明成功。

## 编译 MJLab / G1 控制器

```bash
cd ~/{your_workspace}/src/unitree_rl_mjlab/deploy/robots/g1
mkdir -p build
cd build
cmake ..
make -j4
```

运行键盘控制：

```bash
./g1_ctrl --network=lo --domain=1 --keyboard
```

等待控制器检测到机器人后，在 MuJoCo 界面点击 reset，机器人进入 FixStand / Velocity 流程。在运行 `g1_ctrl` 的终端里使用键盘控制机器人走动：

```text
w/s: x 方向
 a/d: y 方向
q/e: yaw 方向
```

能控制机器人在仿真环境中走动即说明基础控制成功。

## 编译 Livox-SDK2 和 livox_ros_driver2

如果需要实机 Mid360，或者需要编译包含 Livox 的完整工作空间，需要先配置 Livox-SDK2。MuJoCo 仿真本身不依赖真实 Livox 驱动，因为仿真的 `/livox/lidar` 由 `mujuco_sim` bridge 发布；但完整 workspace 或实机流程建议把这一步配置好。

以 ROS2 Foxy 为例，先编译并安装 Livox-SDK2：

```bash
cd ~/{your_workspace}/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

检查是否安装成功：

```bash
find /usr/local -name "livox_lidar_api.h"
ldconfig -p | grep livox
```

再编译 `livox_ros_driver2`：

```bash
cd ~/{your_workspace}
source /opt/ros/foxy/setup.bash
colcon build --packages-select livox_ros_driver2 \
  --cmake-args \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR=/usr/local/include \
  -DLIVOX_LIDAR_SDK_LIBRARY=/usr/local/lib/liblivox_lidar_sdk_shared.so
```

如果 `livox_ros_driver2` 提示 `package.xml` 不存在，请确认使用的是本仓库版本，或者将 `package_ROS2.xml` 复制为 `package.xml`。本仓库推荐直接使用完整 workspace，避免 Livox ROS2 驱动版本和 ROS 发行版不匹配。

## FAST-LIO Foxy 兼容说明

本仓库的 `foxy` 分支已经对 `src/FAST_LIO_ROS2/src/laserMapping.cpp` 做过 Foxy 兼容修改。如果你自己从 FAST-LIO 上游重新下载代码，需要特别注意 `/map_save` 服务的 callback 签名。Foxy 下应使用三参数 callback。

`create_service` 建议写法：

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

对应回调函数建议写法：

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

单独编译 FAST-LIO：

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
colcon build --packages-select fast_lio
source install/setup.bash
ros2 pkg prefix fast_lio
```

`ros2 pkg prefix fast_lio` 有输出即说明安装成功。

## 编译整个工作空间

新电脑第一次拿到这个 workspace，或者拷贝整个工作空间后，建议先清空旧的编译产物再编译：

```bash
cd ~/{your_workspace}
rm -rf build install log
source /opt/ros/{your_ros_distro}/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

注意：本仓库 launch 文件中有工作空间路径配置。请根据自己的工作空间名称修改下面三个文件中的 `WS_SRC`：

```text
src/mujuco_sim/launch/g1_nav_sim.launch.py
src/mujuco_sim/launch/map.launch.py
src/mujuco_sim/launch/nav.launch.py
```

例如你的工作空间叫 `g1_ws`：

```python
WS_SRC = os.path.expanduser("~/g1_ws/src")
```

修改 launch 文件后，需要重新编译并 source：

```bash
cd ~/{your_workspace}
colcon build --packages-select mujuco_sim
source install/setup.bash
```

如果遇到 `stand_go2` duplicate package 错误，请确认下面这个文件存在：

```text
src/unitree_mujoco/example/COLCON_IGNORE
```

如果 launch 启动后仍然搜索旧工作空间，例如日志中出现 `~/yushu_ws/install`，说明 shell 环境被旧 workspace 污染。建议新开终端并执行：

```bash
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/{your_ros_distro}/setup.bash
source ~/{your_workspace}/install/setup.bash
```

同时检查 `~/.bashrc`，不要自动 source 旧工作空间。

## 启动 MuJoCo + G1 控制

键盘控制模式：

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=keyboard
```

Nav2 `/cmd_vel` 控制模式：

```bash
ros2 launch mujuco_sim g1_nav_sim.launch.py input:=cmd_vel
```

常用检查命令：

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## 使用 slam_toolbox 进行 2D 建图

修改 `src/mujuco_sim/launch/map.launch.py` 顶部参数：

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = True
```

启动建图：

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

用键盘控制机器人走动建图，然后保存 2D 地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/{your_workspace}/src/maps/new_2d_map
```

## 使用 FAST-LIO 保存 PCD 地图

修改 `src/mujuco_sim/launch/map.launch.py` 顶部参数：

```python
MODE = "sim"
USE_SLAM_TOOLBOX_2D = False
```

启动 FAST-LIO 建图：

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim map.launch.py
```

控制机器人在环境里走动。注意：不要依赖 `Ctrl+C` 保存 PCD，必须显式调用保存服务：

```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```

仿真 FAST-LIO 的 PCD 输出路径在下面文件里配置：

```text
src/FAST_LIO_ROS2/config/mid360.yaml
```

默认保存到：

```text
./src/maps/vln_fastlio_map.pcd
```

FAST-LIO 3D PCD 建图效果示例：

![FAST-LIO 3D PCD 建图效果](images/pcd.png)

## 启动导航

确认 `src/mujuco_sim/launch/nav.launch.py` 中使用的是需要的 2D 地图。默认地图为：

```text
src/maps/vln_navigation_room.yaml
```

启动导航：

```bash
cd ~/{your_workspace}
source /opt/ros/{your_ros_distro}/setup.bash
source install/setup.bash
ros2 launch mujuco_sim nav.launch.py
```

在 RViz 中发送 `2D Goal Pose`。Nav2 会发布 `/cmd_vel`，`g1_ctrl --cmd_vel` 接收后驱动 MuJoCo 中的 G1 行走。

Nav2 联合仿真效果示例：

![MuJoCo Mid360 点云与 Nav2 地图显示](images/mujoco_3Dcloud.png)

![FAST-LIO 定位接入 Nav2 导航效果](images/nav2_fastlio.png)

## 实机部署说明

实机 G1 上的大致流程：

1. 按 Unitree 官方文档配置网络、SDK2 和 ROS2。
2. 启动真实 Mid360 驱动，确认存在 `/livox/lidar`。
3. 如果要用 `slam_toolbox` 建 2D 图，需要用 `pointcloud_to_laserscan` 把 Mid360 的 `PointCloud2` 转成 `/scan`。
4. 保存 `.yaml/.pgm` 地图后给 Nav2 使用。
5. 在真实机器人网络接口上运行 `g1_ctrl`，例如：

```bash
cd ~/{your_workspace}/src/unitree_rl_mjlab/deploy/robots/g1/build
./g1_ctrl --network=enp5s0
```

建议先验证键盘控制和策略部署没问题，再切换到 `/cmd_vel` 模式接入 Nav2。

## Launch 文件说明

- `g1_nav_sim.launch.py`：启动 Unitree MuJoCo、`g1_ctrl` 和共享内存 ROS2 bridge。
- `map.launch.py`：启动建图流程；在文件顶部切换 2D `slam_toolbox` 或 FAST-LIO PCD 建图。
- `nav.launch.py`：启动仿真、FAST-LIO 定位桥接、Nav2 和 RViz。

## DDS 说明

仿真默认使用 `ROS_DOMAIN_ID=1`，并通过下面配置让 CycloneDDS 使用本机 loopback：

```text
src/mujuco_sim/config/cyclonedds_lo.xml
```

本仓库的 launch 文件会自动设置这些环境变量。如果提示找不到 `rmw_cyclonedds_cpp`，请安装对应 ROS 版本的包，例如：

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
# 或
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
