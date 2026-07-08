#pragma once

#include <cstdint>
#include <cstring>
#include <mutex>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace input
{

static constexpr const char *kUnitreeMujocoNavShmName = "/unitree_mujoco_nav";
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
  double pose[7];
  double qvel[6];
  uint32_t follower_valid;
  double follower_pose[7];
  double follower_qvel[6];
  double livox_xyz[3];
  double livox_rpy[3];
  double imu_quat[4];
  double imu_gyro[3];
  double imu_acc[3];
  float ranges[kUnitreeMujocoNavMaxRanges];
  float points[kUnitreeMujocoNavMaxPoints * 3];
};
#pragma pack(pop)

class FixStandLockSource
{
public:
  static void setActive(bool active)
  {
    std::lock_guard<std::mutex> lock(mutex());
    ensureOpen();
    if (shm_)
    {
      shm_->fixstand_lock_active = active ? 1u : 0u;
    }
  }

private:
  static void ensureOpen()
  {
    if (shm_)
    {
      return;
    }

    // The simulator owns this shared-memory block.  The controller must not
    // create, truncate, or clear it, otherwise nav data can be reset while
    // MuJoCo is running.
    shm_fd_ = shm_open(kUnitreeMujocoNavShmName, O_RDWR, 0);
    if (shm_fd_ < 0)
    {
      return;
    }

    void *ptr = mmap(nullptr, sizeof(UnitreeMujocoNavShmData), PROT_READ | PROT_WRITE,
                     MAP_SHARED, shm_fd_, 0);
    if (ptr == MAP_FAILED)
    {
      close(shm_fd_);
      shm_fd_ = -1;
      return;
    }

    shm_ = static_cast<UnitreeMujocoNavShmData *>(ptr);
    if (shm_->magic != kUnitreeMujocoNavMagic || shm_->version != kUnitreeMujocoNavVersion)
    {
      munmap(shm_, sizeof(UnitreeMujocoNavShmData));
      shm_ = nullptr;
      close(shm_fd_);
      shm_fd_ = -1;
    }
  }

  static std::mutex &mutex()
  {
    static std::mutex mtx;
    return mtx;
  }

  static int shm_fd_;
  static UnitreeMujocoNavShmData *shm_;
};

inline int FixStandLockSource::shm_fd_ = -1;
inline UnitreeMujocoNavShmData *FixStandLockSource::shm_ = nullptr;

} // namespace input
