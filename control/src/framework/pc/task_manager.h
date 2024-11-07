/******************************************************************************
 ** 任务管理模块
 ******************************************************************************
 *
 *  管理所有的任务(启动、停止、状态监测等)
 *
 *  @file       task_manager.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_TASK_MANAGER_H_
#define PHOENIX_FRAMEWORK_TASK_MANAGER_H_

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "utils/macros.h"
#include "utils/com_utils.h"
#include "os/mutex.h"
#include "common/task.h"
#include "util.h"
#include "common_c/message_c.h"
#include "communication_c/shared_data_c.h"
#include "work_c/work_monitor_c.h"


namespace phoenix {
namespace framework {


#if (ENABLE_ROS_NODE)
class RosNode;
#endif
#if (ENABLE_LCM_NODE)
class LcmNode;
#endif
#if (ENABLE_UDP_NODE)
class UdpNode;
#endif
class MsgReceiver;
class MsgSender;
class HmiMsgSender;

class TaskControl;
#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
namespace df_d17_b1 {
class TaskChassisControlDfD17B1;
}
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
namespace df_d17_b2 {
class TaskChassisControlDfD17B2;
}
#else
  Error: Invalid vehicle platform
#endif


class TaskManager : public Task {
 public:
  TaskManager(int argc, char** argv, const std::string& work_space);
  ~TaskManager();

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
  void ChangeGear(Uint8_t value);
  void SetTurnLamp(Uint8_t value);
  void SetWiper(Uint8_t value);
  void ReqManualInterrupt(bool enable);

  bool IsReady() const { return (thread_running_flag_check_tasks_status_); }

private:
  bool ReadConfigFromFile();

#if USING_USER_CLOCK
  void ThreadUpdateUserClock();
#endif
  void ThreadCheckTasksStatus();
  void ThreadSendDataToHmi();

  bool HandleMessage(const Message& msg, Task* sender) override;

 private:
  typedef boost::unique_lock<boost::mutex> Lock;

  ControlConfig_t config_;

#if (ENABLE_ROS_NODE)
  std::unique_ptr<RosNode> ros_node_;
#endif
#if (ENABLE_LCM_NODE)
  std::unique_ptr<LcmNode> lcm_node_;
#endif
#if (ENABLE_UDP_NODE)
  std::unique_ptr<UdpNode> udp_node_;
#endif
  std::unique_ptr<MsgReceiver> msg_receiver_;
  std::unique_ptr<MsgSender> msg_sender_;
  std::unique_ptr<HmiMsgSender> hmi_msg_sender_;

#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  std::unique_ptr<df_d17_b1::TaskChassisControlDfD17B1> task_chassis_control_;
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  std::unique_ptr<df_d17_b2::TaskChassisControlDfD17B2> task_chassis_control_;
#else
  Error: Invalid vehicle platform
#endif
  std::unique_ptr<TaskControl> task_control_;

#if USING_USER_CLOCK
  boost::atomic_bool thread_running_flag_update_user_clock_;
  boost::thread thread_update_user_clock_;
#endif

  boost::atomic_bool thread_running_flag_check_tasks_status_;
  boost::thread thread_check_tasks_status_;

  boost::atomic_bool thread_running_flag_send_data_hmi_;
  boost::thread thread_send_data_hmi_;

 private:

  // log
  std::string work_space_;
  Char_t str_buff_status_[1024*2];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_TASK_MANAGER_H_
