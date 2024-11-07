/******************************************************************************
 ** 规划任务模块
 ******************************************************************************
 *
 *  根据道路信息以及障碍物信息规划车辆局部行驶路线以及速度及加速度
 *
 *  @file       task_planning.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_TASK_PLANNING_H_
#define PHOENIX_FRAMEWORK_TASK_PLANNING_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "utils/com_utils.h"
#include "common/task.h"

#include "work/work_planning.h"


namespace phoenix {
namespace framework {


class TaskPlanning : public Task {
private:
  typedef boost::unique_lock<boost::mutex> Lock;

public:
  explicit TaskPlanning(Task* manager);
  ~TaskPlanning();

  void Configurate(const PlanningConfig& conf);
  void SetPlanningStorys(const ad_msg::PlanningStoryList& storys);

  bool Start();
  bool Stop();

private:
  bool HandleMessage(const Message& msg, Task* sender) override;

  void ThreadPlanning();

private:
  // Thread Planning
  boost::atomic_bool thread_running_flag_planning_;
  boost::thread thread_planning_;
  boost::mutex lock_planning_;
  boost::condition_variable cond_planning_;

  WorkPlanning work_planning_;
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_TASK_PLANNING_H_
