/*
 * @Author: xiarf xiarf@dfcv.com.cn
 * @Date: 2022-08-09 16:01:58
 * @LastEditors: xiarf xiarf@dfcv.com.cn
 * @LastEditTime: 2022-08-09 17:03:59
 * @FilePath: /plan_goyu_rpo/src/control/src/framework/pc/df_d17_b2/task_chassis_control_df_d17_b2.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PHOENIX_FRAMEWORK_DF_D17_TASK_CHASSIS_CONTROL_DF_D17_B2_H_
#define PHOENIX_FRAMEWORK_DF_D17_TASK_CHASSIS_CONTROL_DF_D17_B2_H_
//
#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "utils/macros.h"
#include "utils/log.h"
#include "pc/task_chassis_control.h"
#include "pc/df_d17_b2/can_access_df_d17_b2.h"

namespace phoenix {
namespace framework {
namespace df_d17_b2 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)

class TaskChassisControlDfD17B2 : public TaskChassisControl {
 public:
  explicit TaskChassisControlDfD17B2(Task* manager);
  ~TaskChassisControlDfD17B2();

 private:
  void DoConfigurate(const ChassisControlConfig_t& conf);

  bool StartCanAccess() override;
  bool StopCanAccess() override;

  bool DoSomethingBeforeStartRobotCtl() override;
  bool DoSomethingBeforeStopRobotCtl() override;

  Int32_t SendAlwaysNeededCmd(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) override;
  Int32_t SendCmdInManualStatus(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd) override;

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

  void SendCtlCmd(
      Int8_t ctl_mode, Int8_t enable, Int8_t engage,
      const Chassis_t& status, const ChassisCtlCmd_t& cmd);

  bool HandleMessage(const Message& msg, Task* sender) override;

 private:
  std::unique_ptr<df_d17_b2::CanAccessDfD17B2> can_access_;

  Int32_t req_start_robotic_ctl_count_ = 0;
  Int32_t wait_start_robotic_ctl_ack_count_ = 0;
  Int32_t req_stop_robotic_ctl_count_ = 0;
  Int32_t wait_stop_robotic_ctl_ack_count_ = 0;

  Int8_t ad_ecu_enable_;

  Int32_t send_count_frame_0x0C040BDC_ = 0;
  Int32_t send_count_frame_0x18FF769E_ = 0;
  Int32_t send_count_frame_0x18FF76DC_ = 0;
  Int32_t send_count_frame_0x0CFF79DC_ = 0;

  Int32_t send_count_frame_0x0CFF649E_ = 0;
  Int32_t send_count_frame_0x0CFF64DC_ = 0;
  Int32_t send_count_frame_0x0CFF659E_ = 0;
  Int32_t send_count_frame_0x0C040B9E_ = 0;
  Int32_t send_count_frame_0x0CFF799E_ = 0;
};

#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)


}  // namespace df_d17_b2
}  // namespace canbus
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_DF_D17_TASK_CHASSIS_CONTROL_DF_D17_B2_H_
