#ifndef PHOENIX_FRAMEWORK_TASK_CHASSIS_CONTROL_H_
#define PHOENIX_FRAMEWORK_TASK_CHASSIS_CONTROL_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "utils/macros.h"
#include "utils/log.h"
#include "common/task.h"
#include "common_c/chassis_control_c.h"
#include "msg_chassis_c.h"
#include "lateral_control_c.h"


namespace phoenix {
namespace framework {


class TaskChassisControl : public Task {
 public:
  explicit TaskChassisControl(Task* manager);
  virtual ~TaskChassisControl();

  /**
   * @brief 配置模块参数
   */
  void Configurate(const ChassisControlConfig_t& conf);

  bool Start();
  bool Stop();

  void StartRobotCtl();
  void StopRobotCtl();

  Int32_t GetControlStatus();

 protected:
  virtual void DoConfigurate(const ChassisControlConfig_t& conf) {
    // nothing to do
  }

  virtual bool StartCanAccess() {
    LOG_ERR << "Enter Base Class function, StartCanAccess";
    return (true);
  }

  virtual bool StopCanAccess() {
    LOG_ERR << "Enter Base Class function, StopCanAccess";
    return (true);
  }

  virtual bool DoSomethingBeforeStartRobotCtl() {
    return (true);
  }
  virtual bool DoSomethingBeforeStopRobotCtl() {
    return (true);
  }

  enum {
    REQ_OK = 0,
    REQ_ERR,
    REQ_WAIT,
    REQ_INTERRUPT
  };
  virtual Int32_t SendAlwaysNeededCmd(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);
  virtual Int32_t SendCmdInManualStatus(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);

  virtual Int32_t ReqStartRoboticCtl(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);
  virtual Int32_t WaitStartRoboticCtlAck(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);
  virtual Int32_t ReqSendRoboticCtlCmd(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);
  virtual Int32_t ReqStopRoboticCtl(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);
  virtual Int32_t WaitStopRoboticCtlAck(
      const Chassis_t& status,
      const ChassisCtlCmd_t& cmd);

  bool HandleMessage(const Message& msg, Task* sender) override;

 private:
  void ThreadChassisControl();

 private:
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::atomic_bool running_flag_chassis_control_;
  boost::thread thread_chassis_control_;

  Int32_t control_status_;
  boost::mutex lock_control_status_;

  // 底盘相关的车身状态信息
  Chassis_t chassis_info_;
  // 底盘控制指令
  ChassisCtlCmd_t chassis_ctl_cmd_;

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  Int32_t control_period_ = 10;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Int32_t control_period_ = 10;
#else
  Int32_t control_period_ = 20;
#endif
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_TASK_CHASSIS_CONTROL_H_
