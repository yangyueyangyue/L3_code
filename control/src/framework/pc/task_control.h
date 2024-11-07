#ifndef PHOENIX_FRAMEWORK_TASK_CONTROL_H_
#define PHOENIX_FRAMEWORK_TASK_CONTROL_H_

#include <memory>
#include <vector>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "utils/macros.h"
#include "utils/log.h"
#include "common/task.h"
#include "common/message.h"
#include "pc/task_chassis_control.h"
#include "work_c/work_lateral_control_c.h"
#include "work_c/work_vehicle_control_c.h"


namespace phoenix {
namespace framework {


class TaskControl : public Task {
 public:
  TaskControl(Task* manager, TaskChassisControl* chassis_control);
  ~TaskControl();

  bool Start();
  bool Stop();

  void StartRobotCtl();
  void StopRobotCtl();
  void EnableEps(bool enable);
  void EnableThrottleSys(bool enable);
  void EnableEbs(bool enable);
  void EnableDirectCtl(bool enable);
  void EnableRemoteCtl(bool enable);
  void TurnSteeringWheel(Float32_t angle, Float32_t speed);
  void SpeedUp(Float32_t velocity, Float32_t acc);
  void Accelerate(Float32_t value);
  void Brake(Float32_t value);
  void ChangeGear(Int8_t value);
  void SetTurnLamp(Int8_t value);
  void SetWiper(Int8_t value);
  void ReqManualInterrupt(bool enable);

 private:
  bool HandleMessage(const Message& msg, Task* sender) override;

  void ThreadLateralControl();
  void ThreadVehicleControl();

 private:
  typedef boost::unique_lock<boost::mutex> Lock;

  /* Members for Lateral Control Thread (Begin) */
  boost::atomic_bool thread_running_flag_lateral_control_;
  boost::thread thread_lateral_control_;

  WorkLatCtlInstance_t work_lat_ctl_instance_;
  /* Members for Lateral Control Thread (End) */


  /* Members for Vehicle Control Thread (Begin) */
  boost::atomic_bool thread_running_flag_vehicle_control_;
  boost::thread thread_vehicle_control_;

  WorkVehCtlInstance_t work_veh_ctl_instance_;

  TaskChassisControl* chassis_control_ = nullptr;
  /* Members for Vehicle Control Thread (End) */

  // log
  common::LogFile log_file_lat_ctl_;
  bool open_log_file_lat_ctl_ = false;
  Char_t str_buff_lat_ctl_[1024*16];

  common::LogFile log_file_veh_ctl_;
  bool open_log_file_veh_ctl_ = false;
  Char_t str_buff_veh_ctl_[1024*2];
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_TASK_CONTROL_H_
