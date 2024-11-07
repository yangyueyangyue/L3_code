/******************************************************************************
 ** 定义任务的基本操作
 ******************************************************************************
 *
 *  定义任务的基本操作
 *
 *  @file       task.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "common/task.h"
#include "utils/log.h"


namespace phoenix {
namespace framework {


/******************************************************************************/
/** 请求完成某项任务
 ******************************************************************************
 *  @param      msg             (IN)           消息
 *              sender          (IN)           消息发布者
 *
 *  @retval     true            成功
 *              false           失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       请求完成某项任务
 *
 *  <Attention>
 *       none
 *
 */
bool Task::Request(const Message& msg, Task* sender) {
  return (HandleMessage(msg, sender));
}

/******************************************************************************/
/** 通知任务状态
 ******************************************************************************
 *  @param      msg             (IN)           消息
 *
 *  @retval     true            成功
 *              false           失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       通知任务状态
 *
 *  <Attention>
 *       none
 *
 */
bool Task::Notify(const Message& msg) {
  if (Nullptr_t == manager_) {
    LOG_INFO(5) << "No manager need to notify.";
    return (false);
  }

  return (manager_->HandleMessage(msg, this));
}

/******************************************************************************/
/** 处理消息
 ******************************************************************************
 *  @param      msg             (IN)           消息
 *              sender          (IN)           消息发布者
 *
 *  @retval     true            成功
 *              false           失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       处理消息
 *
 *  <Attention>
 *       none
 *
 */
bool Task::HandleMessage(const Message& msg, Task* sender) {
  LOG_WARN << "Task[" << sender->task_name()
           <<  "] request base_task[" << task_name()
           << "] to handle message[" << msg.id() << "]";

  return (true);
}


}  // namespace framework
}  // namespace phoenix

