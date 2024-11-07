//
#ifndef PHOENIX_FRAMEWORK_WORK_MONITOR_H_
#define PHOENIX_FRAMEWORK_WORK_MONITOR_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "container/doubly_linked_list.h"
#include "os/mutex.h"
#include "common/module_status.h"
#include "ad_msg.h"
#include "common/message.h"
#include "planning/plan_debug.h"

namespace phoenix {
namespace framework {


class WorkMonitor {
public:
  WorkMonitor();
  ~WorkMonitor();

  void Initialize();

  void Configurate(const PlanningConfig& conf);

  void FeedDog_RecvHdMap();
  void FeedDog_RecvRouting();

  void FeedDog_RecvGnss();
  void FeedDog_RecvImu();

  void FeedDog_RecvChassis();
  void FeedDog_RecvSpecialChassisInfo();
  void FeedDog_RecvChassisCtlCmd();
  void FeedDog_RecvLaneMarkCameraList();
  void FeedDog_RecvObstacleCameraList();
  void FeedDog_RecvObstacleCamGeneralFrontList();
  void FeedDog_RecvObstacleCamGeneralLeftFrontList();
  void FeedDog_RecvObstacleCamGeneralRightFrontList();
  void FeedDog_RecvObstacleCamGeneralRearList();
  void FeedDog_RecvObstacleCamGeneralLeftRearList();
  void FeedDog_RecvObstacleCamGeneralRightRearList();
  void FeedDog_RecvObstacleRadarFrontList();
  void FeedDog_RecvObstacleRadarRearList();
  void FeedDog_RecvObstacleRadarFrontLeftList();
  void FeedDog_RecvObstacleRadarFrontRightList();
  void FeedDog_RecvObstacleRadarRearLeftList();
  void FeedDog_RecvObstacleRadarRearRightList();
  void FeedDog_RecvSrr2DetectionsFrontLeftList();
  void FeedDog_RecvSrr2DetectionsFrontRightList();
  void FeedDog_RecvSrr2DetectionsRearLeftList();
  void FeedDog_RecvSrr2DetectionsRearRightList();
  void FeedDog_RecvObstacleLidarList0();
  void FeedDog_RecvObstacleLidarList1();
  void FeedDog_RecvOutsideObstacleList();

  // adecu_debug
  void FeedDog_RecvAdecuVelPlanning();

  void FeedDog_CompletePlanning(Int32_t running_status, Int32_t running_time);

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

    void Initialize(Int32_t max_food) {
      watchdog.SetMaxFoods(max_food);
      timer.Restart();
      counter = 0;

      module_status.Clear();
      module_status.sub_module_id = ad_msg::SUB_MODULE_ID_INVALID;
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
    Monitor recv_msg_hdmap;
    Monitor recv_msg_routing;

    Monitor recv_msg_gnss;
    Monitor recv_msg_imu;
    Monitor recv_msg_chassis;
    Monitor recv_msg_special_chassis_info;
    Monitor recv_msg_chassis_ctl_cmd;
    Monitor recv_msg_lane_mark_camera_list;
    Monitor recv_msg_obstacle_camera_list;
    Monitor recv_msg_obstacle_camgeneral_front_list;
    Monitor recv_msg_obstacle_camgeneral_left_front_list;
    Monitor recv_msg_obstacle_camgeneral_right_front_list;
    Monitor recv_msg_obstacle_camgeneral_rear_list;
    Monitor recv_msg_obstacle_camgeneral_left_rear_list;
    Monitor recv_msg_obstacle_camgeneral_right_rear_list;
    Monitor recv_msg_obstacle_radar_front_list;
    Monitor recv_msg_obstacle_radar_rear_list;
    Monitor recv_msg_obstacle_radar_front_left_list;
    Monitor recv_msg_obstacle_radar_front_right_list;
    Monitor recv_msg_obstacle_radar_rear_left_list;
    Monitor recv_msg_obstacle_radar_rear_right_list;
    Monitor recv_msg_srr2_detections_front_left_list;
    Monitor recv_msg_srr2_detections_front_right_list;
    Monitor recv_msg_srr2_detections_rear_left_list;
    Monitor recv_msg_srr2_detections_rear_right_list;
    Monitor recv_msg_obstacle_lidar_list_0;
    Monitor recv_msg_obstacle_lidar_list_1;
    Monitor recv_msg_outside_obstacle_list;

    // adecu debug
    Monitor recv_msg_adecu_vel_plan_debug;

    Monitor task_planning;
  };

private:
  void UpdateModuleEventList();

private:
  // Config
  PlanningConfig config_;

  MonitorTable monitor_table_;
  ad_msg::ModuleStatusList internal_module_status_list_;

  ad_msg::SpecialChassisInfo special_chassis_info_;
  ad_msg::ModuleStatusList control_module_status_list_;

  enum { MAX_MODULE_EVENT_NUM = 20 };
  typedef common::DoublyLinkedList<ad_msg::EventReporting,
  MAX_MODULE_EVENT_NUM> module_event_list_type;
  typedef common::DataPool<module_event_list_type::node_type,
  MAX_MODULE_EVENT_NUM> module_event_pool_type;
  module_event_list_type module_event_list_;
  module_event_pool_type module_event_pool_;
  ad_msg::EventReportingList msg_event_reporing_list_;
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_WORK_MONITOR_H_

