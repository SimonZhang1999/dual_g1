#include "shm_nav_bridge.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace
{
double QuatYawWxyz(const mjtNum *q)
{
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void MatMulVec(const double mat[9], const double vec[3], mjtNum out[3])
{
  out[0] = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2];
  out[1] = mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2];
  out[2] = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2];
}

void MatTMulVec(const mjtNum mat[9], const mjtNum vec[3], double out[3])
{
  out[0] = mat[0] * vec[0] + mat[3] * vec[1] + mat[6] * vec[2];
  out[1] = mat[1] * vec[0] + mat[4] * vec[1] + mat[7] * vec[2];
  out[2] = mat[2] * vec[0] + mat[5] * vec[1] + mat[8] * vec[2];
}

void RelativeRot(const mjtNum parent[9], const mjtNum child[9], double out[9])
{
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      out[3 * row + col] =
          parent[row] * child[col] +
          parent[3 + row] * child[3 + col] +
          parent[6 + row] * child[6 + col];
    }
  }
}

void RotToRpy(const double rot[9], double rpy[3])
{
  rpy[1] = std::asin(std::clamp(-rot[6], -1.0, 1.0));
  const double cp = std::cos(rpy[1]);
  if (std::abs(cp) > 1e-6)
  {
    rpy[0] = std::atan2(rot[7], rot[8]);
    rpy[2] = std::atan2(rot[3], rot[0]);
  }
  else
  {
    rpy[0] = 0.0;
    rpy[2] = std::atan2(-rot[1], rot[4]);
  }
}
}  // namespace

ShmNavBridge::ShmNavBridge()
{
  LoadMid360Pattern();
  OpenSharedMemory();
  OpenFollowerSharedMemory();
  last_scan_update_ = std::chrono::steady_clock::now();
  last_cloud_update_ = last_scan_update_;
}

ShmNavBridge::~ShmNavBridge()
{
  CloseSharedMemory();
  if (follower_shm_)
  {
    munmap(follower_shm_, sizeof(UnitreeMujocoFollowerShmData));
    follower_shm_ = nullptr;
  }
  if (follower_shm_fd_ >= 0)
  {
    close(follower_shm_fd_);
    follower_shm_fd_ = -1;
  }
}

void ShmNavBridge::OpenSharedMemory()
{
  shm_fd_ = shm_open(kUnitreeMujocoNavShmName, O_CREAT | O_RDWR, 0666);
  if (shm_fd_ < 0)
  {
    perror("shm_open unitree_mujoco_nav");
    return;
  }

  if (ftruncate(shm_fd_, sizeof(UnitreeMujocoNavShmData)) != 0)
  {
    perror("ftruncate unitree_mujoco_nav");
    CloseSharedMemory();
    return;
  }

  void *ptr = mmap(nullptr, sizeof(UnitreeMujocoNavShmData),
                   PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
  if (ptr == MAP_FAILED)
  {
    perror("mmap unitree_mujoco_nav");
    CloseSharedMemory();
    return;
  }

  shm_ = static_cast<UnitreeMujocoNavShmData *>(ptr);
  std::memset(shm_, 0, sizeof(UnitreeMujocoNavShmData));
  shm_->magic = kUnitreeMujocoNavMagic;
  shm_->version = kUnitreeMujocoNavVersion;
  shm_->fixstand_lock_active = 1;
  shm_->num_ranges = num_lidar_rays_;
  shm_->num_points = 0;
  for (int i = 0; i < 3; ++i)
  {
    shm_->livox_xyz[i] = livox_xyz_[i];
    shm_->livox_rpy[i] = livox_rpy_[i];
  }
  for (int i = 0; i < kUnitreeMujocoNavMaxRanges; ++i)
  {
    shm_->ranges[i] = std::numeric_limits<float>::infinity();
  }

  std::cout << "Shared-memory nav bridge enabled: " << kUnitreeMujocoNavShmName
            << std::endl;
}

void ShmNavBridge::CloseSharedMemory()
{
  if (shm_)
  {
    munmap(shm_, sizeof(UnitreeMujocoNavShmData));
    shm_ = nullptr;
  }
  if (shm_fd_ >= 0)
  {
    close(shm_fd_);
    shm_fd_ = -1;
  }
}

void ShmNavBridge::OpenFollowerSharedMemory()
{
  // Open read-write so Python can create it; sim only reads the pose fields.
  follower_shm_fd_ = shm_open(kUnitreeMujocoFollowerShmName, O_CREAT | O_RDWR, 0666);
  if (follower_shm_fd_ < 0)
  {
    perror("shm_open unitree_mujoco_follower");
    return;
  }
  if (ftruncate(follower_shm_fd_, sizeof(UnitreeMujocoFollowerShmData)) != 0)
  {
    perror("ftruncate unitree_mujoco_follower");
    close(follower_shm_fd_);
    follower_shm_fd_ = -1;
    return;
  }
  void *ptr = mmap(nullptr, sizeof(UnitreeMujocoFollowerShmData),
                   PROT_READ | PROT_WRITE, MAP_SHARED, follower_shm_fd_, 0);
  if (ptr == MAP_FAILED)
  {
    perror("mmap unitree_mujoco_follower");
    close(follower_shm_fd_);
    follower_shm_fd_ = -1;
    return;
  }
  follower_shm_ = static_cast<UnitreeMujocoFollowerShmData *>(ptr);
  // Initialise face-to-face with the leader: +1.8m along leader x, yaw=pi.
  follower_shm_->magic = kUnitreeMujocoFollowerMagic;
  follower_shm_->seq   = 0;
  follower_shm_->x     =  1.8f;
  follower_shm_->y     =  0.0f;
  follower_shm_->z     =  0.793f;
  follower_shm_->qw    =  0.0f;  // 180-deg yaw: w=0,z=1
  follower_shm_->qx    =  0.0f;
  follower_shm_->qy    =  0.0f;
  follower_shm_->qz    =  1.0f;
  for (int i = 0; i < 29; ++i) follower_shm_->joints[i] = 0.0f;
  std::cout << "Follower SHM enabled: " << kUnitreeMujocoFollowerShmName << std::endl;
}

void ShmNavBridge::Update(const mjModel *model, const mjData *data)
{
  if (!shm_ || !model || !data)
  {
    return;
  }

  if (base_body_id_ < 0)
  {
    base_body_id_ = mj_name2id(model, mjOBJ_BODY, "pelvis");
    if (base_body_id_ < 0)
    {
      base_body_id_ = mj_name2id(model, mjOBJ_BODY, "torso_link");
    }
    if (base_body_id_ < 0)
    {
      return;
    }
    livox_site_id_ = mj_name2id(model, mjOBJ_SITE, "lidar");
    if (livox_site_id_ < 0)
    {
      livox_site_id_ = mj_name2id(model, mjOBJ_SITE, "livox");
    }
    lidar_body_exclude_id_ = mj_name2id(model, mjOBJ_BODY, "torso_link");
    if (lidar_body_exclude_id_ < 0)
    {
      lidar_body_exclude_id_ = base_body_id_;
    }
    int sensor_id = mj_name2id(model, mjOBJ_SENSOR, "secondary_imu_quat");
    if (sensor_id < 0)
    {
      sensor_id = mj_name2id(model, mjOBJ_SENSOR, "imu_quat");
    }
    if (sensor_id >= 0)
    {
      imu_quat_adr_ = model->sensor_adr[sensor_id];
    }
    sensor_id = mj_name2id(model, mjOBJ_SENSOR, "secondary_imu_gyro");
    if (sensor_id < 0)
    {
      sensor_id = mj_name2id(model, mjOBJ_SENSOR, "imu_gyro");
    }
    if (sensor_id >= 0)
    {
      imu_gyro_adr_ = model->sensor_adr[sensor_id];
    }
    sensor_id = mj_name2id(model, mjOBJ_SENSOR, "secondary_imu_acc");
    if (sensor_id < 0)
    {
      sensor_id = mj_name2id(model, mjOBJ_SENSOR, "imu_acc");
    }
    if (sensor_id >= 0)
    {
      imu_acc_adr_ = model->sensor_adr[sensor_id];
    }
  }

  shm_->pose[0] = data->qpos[0];
  shm_->pose[1] = data->qpos[1];
  shm_->pose[2] = data->qpos[2];
  shm_->pose[3] = data->qpos[3];
  shm_->pose[4] = data->qpos[4];
  shm_->pose[5] = data->qpos[5];
  shm_->pose[6] = data->qpos[6];

  for (int i = 0; i < 6; ++i)
  {
    shm_->qvel[i] = model->nv > i ? data->qvel[i] : 0.0;
  }

  shm_->follower_valid = 0;
  if (f2_jnt_id_ < 0)
  {
    f2_jnt_id_ = mj_name2id(model, mjOBJ_JOINT, "f2_floating_base_joint");
    if (f2_jnt_id_ >= 0)
    {
      f2_jnt_qposadr_ = model->jnt_qposadr[f2_jnt_id_];
      f2_jnt_dofadr_ = model->jnt_dofadr[f2_jnt_id_];
    }
  }
  if (f2_jnt_id_ >= 0 && f2_jnt_qposadr_ + 7 <= model->nq &&
      f2_jnt_dofadr_ + 6 <= model->nv)
  {
    shm_->follower_valid = 1;
    for (int i = 0; i < 7; ++i)
    {
      shm_->follower_pose[i] = data->qpos[f2_jnt_qposadr_ + i];
    }
    for (int i = 0; i < 6; ++i)
    {
      shm_->follower_qvel[i] = data->qvel[f2_jnt_dofadr_ + i];
    }
  }

  const double base_yaw = QuatYawWxyz(data->qpos + 3);
  if (livox_site_id_ >= 0)
  {
    const mjtNum *site_pos = data->site_xpos + 3 * livox_site_id_;
    const mjtNum *site_mat = data->site_xmat + 9 * livox_site_id_;

    // ROS navigation uses a flattened base_link at z=0. Express the real
    // MuJoCo lidar site relative to that planar base frame, otherwise the
    // 3D cloud is shifted downward by the pelvis height.
    const double c = std::cos(base_yaw);
    const double s = std::sin(base_yaw);
    const double dx = site_pos[0] - data->qpos[0];
    const double dy = site_pos[1] - data->qpos[1];
    shm_->livox_xyz[0] = c * dx + s * dy;
    shm_->livox_xyz[1] = -s * dx + c * dy;
    shm_->livox_xyz[2] = site_pos[2];

    const double yaw_mat[9] = {
        c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0,
    };
    double relative_rot[9];
    RelativeRot(yaw_mat, site_mat, relative_rot);
    RotToRpy(relative_rot, shm_->livox_rpy);
  }
  if (imu_quat_adr_ >= 0)
  {
    for (int i = 0; i < 4; ++i)
    {
      shm_->imu_quat[i] = data->sensordata[imu_quat_adr_ + i];
    }
  }
  if (imu_gyro_adr_ >= 0)
  {
    for (int i = 0; i < 3; ++i)
    {
      shm_->imu_gyro[i] = data->sensordata[imu_gyro_adr_ + i];
    }
  }
  if (imu_acc_adr_ >= 0)
  {
    for (int i = 0; i < 3; ++i)
    {
      shm_->imu_acc[i] = data->sensordata[imu_acc_adr_ + i];
    }
  }

  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_scan_update_).count() >= 1.0 / scan_hz_)
  {
    UpdateRanges(model, data);
    last_scan_update_ = now;
  }
  if (std::chrono::duration<double>(now - last_cloud_update_).count() >= 1.0 / cloud_hz_)
  {
    UpdateMid360Cloud(model, data, base_yaw);
    last_cloud_update_ = now;
  }

  shm_->sim_time = data->time;
  shm_->seq = ++seq_;
}

bool ShmNavBridge::FixStandLockActive() const
{
  return shm_ && shm_->fixstand_lock_active != 0;
}

void ShmNavBridge::TeleportFollower(const mjModel *model, mjData *data)
{
  if (!follower_shm_ || !model || !data)
    return;

  // Resolve f2_floating_base_joint once
  if (f2_jnt_id_ < 0)
  {
    f2_jnt_id_ = mj_name2id(model, mjOBJ_JOINT, "f2_floating_base_joint");
    if (f2_jnt_id_ < 0)
    {
      std::cerr << "TeleportFollower: f2_floating_base_joint not found\n";
      return;
    }
    f2_jnt_qposadr_ = model->jnt_qposadr[f2_jnt_id_];
    f2_jnt_dofadr_  = model->jnt_dofadr[f2_jnt_id_];
    std::cout << "Follower freejoint qposadr=" << f2_jnt_qposadr_
              << " dofadr=" << f2_jnt_dofadr_ << "\n";
  }

  // Set freejoint position+orientation from SHM
  data->qpos[f2_jnt_qposadr_ + 0] = follower_shm_->x;
  data->qpos[f2_jnt_qposadr_ + 1] = follower_shm_->y;
  data->qpos[f2_jnt_qposadr_ + 2] = follower_shm_->z;
  data->qpos[f2_jnt_qposadr_ + 3] = follower_shm_->qw;
  data->qpos[f2_jnt_qposadr_ + 4] = follower_shm_->qx;
  data->qpos[f2_jnt_qposadr_ + 5] = follower_shm_->qy;
  data->qpos[f2_jnt_qposadr_ + 6] = follower_shm_->qz;

  // Zero all velocities for the follower freejoint (6 DOFs)
  for (int i = 0; i < 6; ++i)
    data->qvel[f2_jnt_dofadr_ + i] = 0.0;

  // Safety: check bounds before writing body joints
  if (f2_jnt_qposadr_ + 7 + kF2Dof > model->nq ||
      f2_jnt_dofadr_  + 6 + kF2Dof > model->nv)
  {
    std::cerr << "TeleportFollower: joint index out of range "
              << "(nq=" << model->nq << " nv=" << model->nv << ")\n";
    return;
  }

  // Lock joints at standing pose (freejoint qposadr+7 is first body joint)
  for (int i = 0; i < kF2Dof; ++i)
  {
    data->qpos[f2_jnt_qposadr_ + 7 + i] = f2_stand_qpos_[i];
    // Zero joint velocities too (dofadr+6 onwards)
    data->qvel[f2_jnt_dofadr_  + 6 + i] = 0.0;
  }
}

void ShmNavBridge::UpdateRanges(const mjModel *model, const mjData *data)
{
  const mjtNum *base_pos = data->xpos + 3 * base_body_id_;
  const double base_yaw = QuatYawWxyz(data->qpos + 3);

  mjtNum lidar_pos[3] = {
      base_pos[0] + livox_xyz_[0] * std::cos(base_yaw) - livox_xyz_[1] * std::sin(base_yaw),
      base_pos[1] + livox_xyz_[0] * std::sin(base_yaw) + livox_xyz_[1] * std::cos(base_yaw),
      base_pos[2] + livox_xyz_[2],
  };
  if (livox_site_id_ >= 0)
  {
    const mjtNum *site_pos = data->site_xpos + 3 * livox_site_id_;
    lidar_pos[0] = site_pos[0];
    lidar_pos[1] = site_pos[1];
    lidar_pos[2] = site_pos[2];
  }

  const double angle_min = -M_PI;
  const double angle_increment = 2.0 * M_PI / static_cast<double>(num_lidar_rays_);

  for (int i = 0; i < num_lidar_rays_; ++i)
  {
    const double angle = base_yaw + angle_min + i * angle_increment;
    mjtNum ray_dir[3] = {std::cos(angle), std::sin(angle), 0.0};
    int geom_id = -1;
    const mjtNum dist = mj_ray(model, data, lidar_pos, ray_dir, nullptr, 1,
                               lidar_body_exclude_id_, &geom_id);

    if (dist < range_min_ || dist < 0.0)
    {
      shm_->ranges[i] = std::numeric_limits<float>::infinity();
    }
    else
    {
      shm_->ranges[i] = static_cast<float>(std::min<double>(dist, range_max_));
    }
  }
  shm_->num_ranges = num_lidar_rays_;
}

bool ShmNavBridge::LoadMid360Pattern()
{
  const char *paths[] = {
      "../config/mid360_pattern.csv",
      "config/mid360_pattern.csv",
      "/home/xkh/yushu_ws/src/unitree_mujoco/simulate/config/mid360_pattern.csv",
  };

  std::ifstream file;
  for (const char *path : paths)
  {
    file.open(path);
    if (file.good())
    {
      std::cout << "Loading Mid360 pattern: " << path << std::endl;
      break;
    }
    file.close();
  }
  if (!file.good())
  {
    std::cerr << "Mid360 pattern not found; /livox/lidar will be empty." << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string theta_text;
    std::string phi_text;
    if (!std::getline(ss, theta_text, ',') || !std::getline(ss, phi_text))
    {
      continue;
    }
    try
    {
      mid360_theta_.push_back(std::stof(theta_text));
      mid360_phi_.push_back(std::stof(phi_text));
    }
    catch (...)
    {
      continue;
    }
  }

  std::cout << "Loaded Mid360 rays: " << mid360_theta_.size() << std::endl;
  return !mid360_theta_.empty();
}

void ShmNavBridge::UpdateMid360Cloud(const mjModel *model, const mjData *data, double base_yaw)
{
  if (mid360_theta_.empty())
  {
    shm_->num_points = 0;
    return;
  }

  mjtNum lidar_pos[3] = {
      data->qpos[0] + livox_xyz_[0] * std::cos(base_yaw) - livox_xyz_[1] * std::sin(base_yaw),
      data->qpos[1] + livox_xyz_[0] * std::sin(base_yaw) + livox_xyz_[1] * std::cos(base_yaw),
      data->qpos[2] + livox_xyz_[2],
  };

  const mjtNum *site_mat = nullptr;
  if (livox_site_id_ >= 0)
  {
    const mjtNum *site_pos = data->site_xpos + 3 * livox_site_id_;
    lidar_pos[0] = site_pos[0];
    lidar_pos[1] = site_pos[1];
    lidar_pos[2] = site_pos[2];
    site_mat = data->site_xmat + 9 * livox_site_id_;
  }

  uint32_t count = 0;
  for (int i = 0; i < kUnitreeMujocoNavMaxPoints; ++i)
  {
    const size_t idx = (mid360_start_index_ + static_cast<size_t>(i)) % mid360_theta_.size();
    const double theta = mid360_theta_[idx];
    const double phi = mid360_phi_[idx];
    const double cphi = std::cos(phi);
    const double local_dir[3] = {
        cphi * std::cos(theta),
        cphi * std::sin(theta),
        std::sin(phi),
    };

    mjtNum world_dir[3];
    if (site_mat)
    {
      MatMulVec(site_mat, local_dir, world_dir);
    }
    else
    {
      world_dir[0] = local_dir[0] * std::cos(base_yaw) - local_dir[1] * std::sin(base_yaw);
      world_dir[1] = local_dir[0] * std::sin(base_yaw) + local_dir[1] * std::cos(base_yaw);
      world_dir[2] = local_dir[2];
    }

    int geom_id = -1;
    const mjtNum dist = mj_ray(model, data, lidar_pos, world_dir, nullptr, 1,
                               lidar_body_exclude_id_, &geom_id);
    if (dist < range_min_ || dist < 0.0 || dist > cloud_range_max_)
    {
      continue;
    }

    shm_->points[3 * count + 0] = static_cast<float>(local_dir[0] * dist);
    shm_->points[3 * count + 1] = static_cast<float>(local_dir[1] * dist);
    shm_->points[3 * count + 2] = static_cast<float>(local_dir[2] * dist);
    ++count;
  }

  mid360_start_index_ =
      (mid360_start_index_ + static_cast<size_t>(kUnitreeMujocoNavMaxPoints)) %
      mid360_theta_.size();
  shm_->num_points = count;
}
