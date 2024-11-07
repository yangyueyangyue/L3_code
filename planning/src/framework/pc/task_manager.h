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
#include "utils/com_utils.h"
#include "os/mutex.h"
#include "common/task.h"
#include "util.h"
#include "driving_map.h"
#include "work/work_monitor.h"

#ifdef ENABLE_PACC_ADASIS
#include "task_recv_adasisv2_msg.h"
#endif

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

class TaskPlanning;
class TaskChatter;

class TaskManager : public Task {
public:
  TaskManager(Int32_t argc, Char_t** argv, const std::string& work_space);
  ~TaskManager();

  bool Start();
  bool Stop();

private:
  bool ReadConfigFromFile();

  void ThreadUpdateUserClock();
  void ThreadCheckTasksStatus();
  void ThreadSendDataToHmi();

  bool HandleMessage(const Message& msg, Task* sender) override;

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  PlanningConfig config_;

  Int32_t communication_mode_;

  std::unique_ptr<TaskPlanning> task_planning_;
  std::unique_ptr<TaskChatter> task_chatter_;

#ifdef ENABLE_PACC_ADASIS
  std::unique_ptr<adasisv2::TaskRecvADASISv2Msg> task_adasisv2_;
#endif

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

#if USING_USER_CLOCK
  boost::atomic_bool thread_running_flag_update_user_clock_;
  boost::thread thread_update_user_clock_;
#endif

  boost::atomic_bool thread_running_flag_check_tasks_status_;
  boost::thread thread_check_tasks_status_;

  boost::atomic_bool thread_running_flag_send_data_hmi_;
  boost::thread thread_send_data_hmi_;

 private:
  WorkMonitor work_monitor_;

  // log
  std::string work_space_;
  Char_t str_buff_status_[1024*2];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_TASK_MANAGER_H_
