/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** 其它任务处理模块
 ******************************************************************************
 *
 *  处理其它琐碎的任务
 *
 *  @file       task_chatter.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_TASK_CHATTER_H_
#define PHOENIX_FRAMEWORK_TASK_CHATTER_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "common/task.h"
#include "common/message.h"
#include "communication/msg_queue.h"


namespace phoenix {
namespace framework {

using ChatterMsgQueue = MsgQueue<MessageChatterTask, 20>;

class TaskChatter : public Task {
 private:
  typedef boost::unique_lock<boost::mutex> Lock;

 public:
  explicit TaskChatter(Task* manager);
  ~TaskChatter();

  bool Start();
  bool Stop();

 private:
  bool HandleMessage(const Message& msg, Task* sender) override;
  void TheadDoTask();

 private:
  std::shared_ptr<Char_t> map_data_array_ = Nullptr_t;
  Int32_t map_data_array_size_ = 0;
  boost::mutex map_data_array_lock_;
  void SaveMapDataBuff(const Char_t* buff, Int32_t size);
  void GetMapDataBuff(std::shared_ptr<Char_t>* map_data_array,
      Int32_t* map_data_array_size);
  std::shared_ptr<Char_t> routing_data_array_ = Nullptr_t;
  Int32_t routing_data_array_size_ = 0;
  boost::mutex routing_data_array_lock_;
  void SaveRoutingDataBuff(const Char_t* buff, Int32_t size);
  void GetRoutingDataBuff(std::shared_ptr<Char_t>* routing_data_array,
      Int32_t* routing_data_array_size);

 private:
  boost::atomic_bool thread_running_flag_;
  boost::thread thread_do_task_;

  ChatterMsgQueue msg_queue_;
  boost::mutex msg_ready_lock_;
  boost::condition_variable msg_ready_cond_;
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_TASK_CHATTER_H_

