#ifndef PHOENIX_FRAMEWORK_DF_D17_TASK_CHASSIS_CONTROL_DF_D17_H_
#define PHOENIX_FRAMEWORK_DF_D17_TASK_CHASSIS_CONTROL_DF_D17_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "utils/macros.h"
#include "utils/log.h"
#include "pc/task_chassis_control.h"
#include "pc/df_d17_b1/can_access_df_d17_b1.h"

namespace phoenix {
namespace framework {
namespace df_d17_b1 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)

class TaskChassisControlDfD17B1 : public TaskChassisControl {
 public:
  explicit TaskChassisControlDfD17B1(Task* manager);
  ~TaskChassisControlDfD17B1();

 private:
  bool StartCanAccess() override;
  bool StopCanAccess() override;

  bool DoSomethingBeforeStartRobotCtl() override;
  bool DoSomethingBeforeStopRobotCtl() override;

  Int32_t ReqStartRoboticCtl(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;
  Int32_t WaitStartRoboticCtlAck(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;
  Int32_t ReqSendRoboticCtlCmd(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;
  Int32_t ReqStopRoboticCtl(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;
  Int32_t WaitStopRoboticCtlAck(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;
    Int32_t SendAlwaysNeededCmd(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;

  bool HandleMessage(const Message& msg, Task* sender) override;

 private:
  std::unique_ptr<df_d17_b1::CanAccessDfD17B1> can_access_;

  Int32_t req_start_robotic_ctl_count_ = 0;
  Int32_t wait_start_robotic_ctl_ack_count_ = 0;
  Int32_t req_stop_robotic_ctl_count_ = 0;
  Int32_t wait_stop_robotic_ctl_ack_count_ = 0;

  Int8_t send_EHPS_cycle_ = 2;
  Int8_t send_XBL_cycle_ = 2;
  Int8_t send_ADCU_cycle_ = 2;

  SteeringControlInfo_t steering_ctl_info_;
};

#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)


}  // namespace df_d17_b1
}  // namespace canbus
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_DF_D17_TASK_CHASSIS_CONTROL_DF_D17_H_
