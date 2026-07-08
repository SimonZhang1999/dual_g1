#pragma once

#include <chrono>
#include <cstdint>
#include <vector>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <mujoco/mujoco.h>

static constexpr const char *kUnitreeMujocoNavShmName    = "/unitree_mujoco_nav";
static constexpr const char *kUnitreeMujocoFollowerShmName = "/unitree_mujoco_follower";
static constexpr uint32_t kUnitreeMujocoNavMagic = 0x564e4a4d;  // MJNV
static constexpr uint32_t kUnitreeMujocoNavVersion = 5;
static constexpr int kUnitreeMujocoNavMaxRanges = 360;
static constexpr int kUnitreeMujocoNavMaxPoints = 24000;

#pragma pack(push, 1)
struct UnitreeMujocoNavShmData
{
  uint32_t magic;
  uint32_t version;
  uint32_t fixstand_lock_active;
  uint32_t num_ranges;
  uint32_t num_points;
  uint64_t seq;
  double sim_time;
  double pose[7];  // x y z qw qx qy qz
  double qvel[6];  // world linear xyz, angular xyz
  uint32_t follower_valid;
  double follower_pose[7];  // x y z qw qx qy qz
  double follower_qvel[6];  // world linear xyz, angular xyz
  double livox_xyz[3];
  double livox_rpy[3];
  double imu_quat[4];  // qw qx qy qz
  double imu_gyro[3];
  double imu_acc[3];
  float ranges[kUnitreeMujocoNavMaxRanges];
  float points[kUnitreeMujocoNavMaxPoints * 3];  // xyz in livox_frame
};
#pragma pack(pop)

// ── Follower pose SHM (written by Python follower_sim.py) ─────────────
#pragma pack(push, 1)
struct UnitreeMujocoFollowerShmData
{
  uint32_t magic;   // 0x464F4C57 = "FOLW"
  uint32_t seq;
  float x, y, z;
  float qw, qx, qy, qz;  // world orientation (w,x,y,z)
  float joints[29];       // standing joint angles
};
#pragma pack(pop)

static constexpr uint32_t kUnitreeMujocoFollowerMagic = 0x464F4C57;

class ShmNavBridge
{
public:
  ShmNavBridge();
  ~ShmNavBridge();

  void Update(const mjModel *model, const mjData *data);
  // Called BEFORE mj_step to teleport follower robot
  void TeleportFollower(const mjModel *model, mjData *data);
  bool FixStandLockActive() const;

private:
  void OpenSharedMemory();
  void CloseSharedMemory();
  void OpenFollowerSharedMemory();
  void UpdateRanges(const mjModel *model, const mjData *data);
  void UpdateMid360Cloud(const mjModel *model, const mjData *data, double base_yaw);
  bool LoadMid360Pattern();

  int shm_fd_ = -1;
  UnitreeMujocoNavShmData *shm_ = nullptr;
  int base_body_id_ = -1;
  int livox_site_id_ = -1;
  int lidar_body_exclude_id_ = -1;
  int imu_quat_adr_ = -1;
  int imu_gyro_adr_ = -1;
  int imu_acc_adr_ = -1;
  uint64_t seq_ = 0;
  size_t mid360_start_index_ = 0;

  std::chrono::steady_clock::time_point last_scan_update_;
  std::chrono::steady_clock::time_point last_cloud_update_;

  double scan_hz_ = 10.0;
  double cloud_hz_ = 10.0;
  int num_lidar_rays_ = 360;  // full 360° at 1° resolution
  double range_min_ = 0.10;
  double range_max_ = 8.0;
  double cloud_range_max_ = 30.0;
  double livox_xyz_[3] = {0.0, 0.0, 0.95};
  double livox_rpy_[3] = {0.0, -2.3 * 3.14159265358979323846 / 180.0, 0.0};
  std::vector<float> mid360_theta_;
  std::vector<float> mid360_phi_;

  // Follower robot SHM (read-only, written by Python)
  int follower_shm_fd_  = -1;
  UnitreeMujocoFollowerShmData *follower_shm_ = nullptr;
  // Follower robot model IDs (resolved once)
  int f2_jnt_id_      = -1;  // id of f2_floating_base_joint
  int f2_jnt_qposadr_ = -1;  // qpos start index for freejoint
  int f2_jnt_dofadr_  = -1;  // qvel start index
  // Standing joint qpos for follower (joints 7..35 after freejoint)
  static constexpr int kF2Dof = 29;
  double f2_stand_qpos_[kF2Dof] = {
    -0.20, 0.0, 0.0, 0.48, -0.30, 0.0,   // left leg
    -0.20, 0.0, 0.0, 0.48, -0.30, 0.0,   // right leg
     0.0,  0.0, 0.0,                       // waist
     0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0, // left arm
     0.35,-0.18, 0.0, 0.87, 0.0, 0.0, 0.0  // right arm
  };
};
