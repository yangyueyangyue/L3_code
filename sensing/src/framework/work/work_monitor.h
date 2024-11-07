//
#ifndef PHOENIX_FRAMEWORK_WORK_MONITOR_H_
#define PHOENIX_FRAMEWORK_WORK_MONITOR_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "os/mutex.h"
#include "common/module_status.h"
#include "ad_msg.h"


namespace phoenix {
namespace framework {


class WorkMonitor {
public:
  WorkMonitor();
  ~WorkMonitor();

  void Initialize();

  void FeedDog_RecvGnss();
  void FeedDog_RecvImu();
  void FeedDog_RecvLaneMarkCamera();
  void FeedDog_RecvLaneCurbCamera();
  void FeedDog_RecvObstacleCamera();
  void FeedDog_RecvObstacleEsrFront();
  void FeedDog_RecvChassis();
  void FeedDog_RecvObstacleMaxieyeCamera();
  void FeedDog_RecvObstaclesLidarFront();
  void FeedDog_RecvObstacleAnngicRadarFrontLeft();
  void FeedDog_RecvObstacleAnngicRadarFrontRight();
  void FeedDog_RecvObstacleAnngicRadarRearLeft();
  void FeedDog_RecvObstacleAnngicRadarRearRight();
  void FeedDog_RecvVisualControlLaneMarkCamera();
  void FeedDog_RecvObstacleVisualControlFront();
  void FeedDog_RecvObstacleVisualControlFrontLeft();
  void FeedDog_RecvObstacleVisualControlFrontRight();
  void FeedDog_RecvObstacleVisualControlRearLeft();
  void FeedDog_RecvObstacleVisualControlRearRight();
  //void FeedDog_RecvObstacleVisualControlRear();
  void FeedDog_RecvObstacleFusionObj();
  void FeedDog_RecvMpuState();
  void FeedDog_RecvADHMIObstacleFusionObj();

  Int32_t DoWork();

private:
  class Watchdog {
   public:
    Watchdog() {
      max_foods_ = 0;
      foods_ = 0;
    }

    explicit Watchdog(Int32_t foods) {
      max_foods_ = foods;
    }

    void SetMaxFoods(Int32_t foods) { max_foods_ = foods; }
    void Feed() { foods_ = max_foods_; }
    Int32_t Walk() {
      foods_--;
      if (foods_ < 0) {
        foods_ = 0;
      }
      return (foods_);
    }
    Float32_t GetFoodsPercentage() const {
      return (static_cast<Float32_t>(foods_) /
              static_cast<Float32_t>(max_foods_));
    }

   private:
    Int32_t max_foods_;
    Int32_t foods_;
  };

  struct Monitor {
    common::os::Mutex mutex;

    Watchdog watchdog;
    common::Stopwatch timer;
    Uint32_t counter;

    ad_msg::ModuleStatus module_status;

    Monitor() {
      counter = 0;
    }

    void Initialize(Int32_t module_id, Int32_t max_food) {
      watchdog.SetMaxFoods(max_food);
      timer.Restart();
      counter = 0;

      module_status.Clear();
      module_status.sub_module_id = module_id;
    }

    void FeedDog(
        Int32_t mode,
        Int32_t status,
        Int32_t param0, Int32_t param1, Int32_t param2) {
      // Lock
      common::os::LockHelper lock(mutex);

      // Save module status
      module_status.status = status;
      module_status.param[1] = param0;
      module_status.param[2] = param1;
      module_status.param[3] = param2;

      // Analyzing Traffic
      Int64_t time_elapsed = timer.Elapsed();
      if (0 == mode) {
        // 两帧间隔
        module_status.param[0] = time_elapsed;
        // 重启定时器
        timer.Restart();
      } else if (1 == mode) {
        counter++;
        if (time_elapsed > 999) {
          // 两帧间隔
          module_status.param[0] = common::com_round(
                (Float64_t)time_elapsed / counter);
          // 帧率 FPS
          // module_status.param[1] = counter / (time_elapsed/1000.0);
          // 重启定时器
          timer.Restart();
          counter = 0;
        }
      } else {
        // 两帧间隔
        module_status.param[0] = time_elapsed;
        // 重启定时器
        timer.Restart();
      }

      // Feed dog
      watchdog.Feed();
    }

    void UpdateStatus(ad_msg::ModuleStatus* status) {
      // Lock
      common::os::LockHelper lock(mutex);

      // Get module status
      *status = module_status;
      // Timeout ?
      Int32_t food = watchdog.Walk();
      if (food > 0) {
        module_status.timeout = 0;
        status->timeout = 0;
      } else {
        module_status.timeout = 1;
        status->timeout = 1;
      }
    }

  };

  struct MonitorTable {
    Monitor recv_msg_gnss;
    Monitor recv_msg_imu;
    Monitor recv_msg_lane_mark_camera;
    Monitor recv_msg_lane_curb_camera;
    Monitor recv_msg_obstacle_camera;
    Monitor recv_msg_obstacle_esr_front;
    Monitor recv_msg_chassis;
    Monitor recv_msg_obstacle_maxieye_camera;
    Monitor recv_msg_obstacle_lidar_front;
    Monitor recv_msg_obstacle_anngic_radar_front_left;
    Monitor recv_msg_obstacle_anngic_radar_front_right;
    Monitor recv_msg_obstacle_anngic_radar_rear_left;
    Monitor recv_msg_obstacle_anngic_radar_rear_right;
    Monitor recv_msg_visual_control_lane_mark_camera;
    Monitor recv_msg_obstacle_visual_control_front;
    Monitor recv_msg_obstacle_visual_control_front_left;
    Monitor recv_msg_obstacle_visual_control_front_right;
    Monitor recv_msg_obstacle_visual_control_rear_left;
    Monitor recv_msg_obstacle_visual_control_rear_right;
    Monitor recv_msg_obstacle_visual_control_rear;
    Monitor recv_msg_obstacle_fusion;
    Monitor recv_msg_mpu_state;
    Monitor recv_msg_adhmi_obstacle_fusion;
  };

private:
  MonitorTable monitor_table_;
  ad_msg::ModuleStatusList internal_module_status_list_;
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_WORK_MONITOR_H_

