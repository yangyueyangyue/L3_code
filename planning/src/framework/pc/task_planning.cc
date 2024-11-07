/******************************************************************************
 ** 规划任务模块
 ******************************************************************************
 *
 *  根据道路信息以及障碍物信息规划车辆局部行驶路线以及速度及加速度
 *
 *  @file       task_planning.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "task_planning.h"

#include <vector>

#include "utils/log.h"
#include "util.h"


namespace phoenix {
namespace framework {


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      manager         (in)          任务管理模块
 *
 *  @retval     None
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       None
 *
 */
TaskPlanning::TaskPlanning(Task* manager) :
  Task(TASK_ID_PLANNING, "Planning", manager) {
  thread_running_flag_planning_ = false;
}

/******************************************************************************/
/** 析构函数
 ******************************************************************************
 *  @param      none
 *
 *  @retval     None
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       析构函数
 *
 *  <Attention>
 *       None
 *
 */
TaskPlanning::~TaskPlanning() {
  Stop();
}

void TaskPlanning::Configurate(const PlanningConfig& conf) {
  work_planning_.Configurate(conf);
}

void TaskPlanning::SetPlanningStorys(const ad_msg::PlanningStoryList& storys) {
  work_planning_.SetPlanningStorys(storys);
}

/******************************************************************************/
/** 开启任务
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       开启任务
 *
 *  <Attention>
 *       None
 *
 */
bool TaskPlanning::Start() {
  // Start Planning Thread
  if (!thread_running_flag_planning_) {
    LOG_INFO(3) << "Start Planning Thread ...";
    thread_running_flag_planning_ = true;
    thread_planning_ =
        boost::thread(boost::bind(&TaskPlanning::ThreadPlanning, this));
    LOG_INFO(3) << "Start Planning Thread ... [OK]";
  }

  return (true);
}

/******************************************************************************/
/** 结束任务
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       结束任务
 *
 *  <Attention>
 *       None
 *
 */
bool TaskPlanning::Stop() {
  // Stop Planning Thread
  if (thread_running_flag_planning_) {
    LOG_INFO(3) << "Stop Planning Thread ...";

    thread_running_flag_planning_ = false;
    //cond_planning_.notify_one();

    bool ret = thread_planning_.timed_join(boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread \"Planning\" exiting.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Planning Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Planning Thread... [NG]";
    }
  }

  return (true);
}

/******************************************************************************/
/** 处理收到的消息
 ******************************************************************************
 *  @param      msg              (in)         消息
 *              sender           (in)         发送者
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       处理收到的消息
 *
 *  <Attention>
 *       None
 *
 */
bool TaskPlanning::HandleMessage(
      const Message& msg, Task* sender) {
  // Must be reentrant function

  bool ret = true;

  switch (msg.id()) {
  case (MSG_ID_REQ_DO_TASK):
  {
    // cond_calc_optimal_path_.notify_one();
    // cond_planning_.notify_one();
  }
    break;

  default:
    ret = false;
    break;
  }

  return (ret);
}

/******************************************************************************/
/** 线程函数
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       线程函数(规划)
 *
 *  <Attention>
 *       None
 *
 */
void TaskPlanning::ThreadPlanning() {
  LOG_INFO(3) << "Planning Thread ... [Started]";

  Dormancy dormancy(100);
  while (thread_running_flag_planning_) {
    /// waiting message
    // {
    //   Lock lock(lock_planning_);
    //   cond_planning_.wait(lock);
    // }
    // if (!thread_running_flag_planning_) {
    //   break;
    // }

    phoenix::common::Stopwatch performance_timer;

    Int32_t status = work_planning_.DoWork();

    Int32_t time_elapsed = performance_timer.Elapsed();

    MessagePlanningResult planning_ret(
          &(work_planning_.GetPlanningResult()), status, time_elapsed);
    Notify(planning_ret);

    // Add Planning debug BY ZQ for Ros
    MessagePlanningDebug planning_debug(
          &(work_planning_.GetPlanningDebug()), status, time_elapsed);
    Notify(planning_debug);

    // adecu_plan_debug
    MessageAdecuDebug adecu_debug(
          &(work_planning_.GetAdecuPlanDebug()), status, time_elapsed);
    Notify(adecu_debug);

    if (!work_planning_.GetConfiguration().obj_filter_config.using_outside_obj_list) {
      // TODO: Notify obstacles list.
      MessageObstacleList obstacle_list(
            &(work_planning_.GetObstacleList()));
      Notify(obstacle_list);
    }
    
#ifdef ENABLE_PACC_ADASIS
    MessageADASISV2Horion horizon(work_planning_.GetADASISv2Horizon());
    Notify(horizon);
#endif

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  if (work_planning_.IsLocalizationUpdated()) {
    MessageGnss msg_gnss(&work_planning_.GetGnssInfo());

    Notify(msg_gnss);
  }

  if (work_planning_.IsMapUpdated()) {
    MessageMapData msg_map(Nullptr_t, 0);

    Notify(msg_map);
  }
#endif

    // Sleep
    dormancy.Sleep();
  }

  thread_running_flag_planning_ = false;

  LOG_INFO(3) << "Planning Thread ... [Stopped]";
}


}  // namespace framework
}  // namespace phoenix
