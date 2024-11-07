/******************************************************************************
 ** 定义任务的基本操作
 ******************************************************************************
 *
 *  定义任务的基本操作
 *
 *  @file       task.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_TASK_H_
#define PHOENIX_FRAMEWORK_TASK_H_

#include <string>
#include "common/message.h"


namespace phoenix {
namespace framework {


class Task {
public:
  explicit Task(Int32_t id, const Char_t* name, Task* manager = Nullptr_t) :
    task_id_(id), manager_(manager) {
    common::com_strncpy(task_name_, name, sizeof(task_name_)-1);
  }
  virtual ~Task() = default;

  Int32_t task_id() const { return (task_id_); }
  const Char_t* task_name() const { return (task_name_); }

  virtual bool Request(const Message& msg, Task* sender);
  virtual bool Notify(const Message& msg);

private:
  virtual bool HandleMessage(const Message& msg, Task* sender);

private:
  Int32_t task_id_;
  Char_t task_name_[128];
  Task* manager_ = Nullptr_t;
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_TASK_H_
