/******************************************************************************
 ** 先入先出队列
 ******************************************************************************
 *
 *  先入先出队列(主要用于多线程以及多进程通讯)
 *
 *  @file       msg_queue.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MSG_QUEUE_H_
#define PHOENIX_FRAMEWORK_MSG_QUEUE_H_

#include <string>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include "utils/log.h"
#include "container/data_list.h"

namespace phoenix {
namespace framework {


template <typename MsgType, int Count>
class MsgQueue {
 private:
  typedef MsgType  msg_type;
  typedef phoenix::common::DataList<MsgType, Count> list_type;
  typedef boost::mutex mutex_type;
  typedef boost::unique_lock<boost::mutex> lock_type;

 public:
  explicit MsgQueue(bool override_expired_msg = true);
  bool Put(const msg_type& msg);
  bool Get(msg_type* msg);
  void Clear();

  std::string DebugString(void);

 private:
  bool override_expired_msg_  = false;

  list_type list_;
  mutex_type queue_lock_;
};

template <typename MsgType, int Count>
MsgQueue<MsgType, Count>::MsgQueue(bool override_expired_msg) :
  override_expired_msg_(override_expired_msg),
  list_(override_expired_msg) {
}

template <typename MsgType, int Count>
bool MsgQueue<MsgType, Count>::Put(const msg_type& msg) {
  lock_type lock(queue_lock_);

  msg_type* msg_space = list_.AllocSpace();
  if (Nullptr_t == msg_space) {
    LOG_ERR << "Queue space is not enough.";
    return (false);
  }
  *msg_space = msg;
  list_.PushBack(msg_space);

  return (true);
}

template <typename MsgType, int Count>
bool MsgQueue<MsgType, Count>::Get(msg_type* msg) {
  lock_type lock(queue_lock_);

  msg_type* msg_space = list_.GetFront();
  if (Nullptr_t == msg_space) {
    return (false);
  }
  *msg = *msg_space;
  list_.FreeSpace(msg_space);

  return (true);
}

template <typename MsgType, int Count>
void MsgQueue<MsgType, Count>::Clear() {
  lock_type lock(queue_lock_);

  list_.Clear();
}

template <typename MsgType, int Count>
std::string MsgQueue<MsgType, Count>::DebugString(void) {
  lock_type lock(queue_lock_);

  std::ostringstream os;

  os << "-------------------- Message Queue -------------------------->"
    << std::endl;

  int i = 0;
  for (typename list_type::iterator it = list_.begin();
       it != list_.end(); ++it) {
    std::cout << "[" << i++ << "] data_addr=" << it.current()
              << std::endl;
  }

  os << "<------------------- Message Queue --------------------------"
     << std::endl;

  return (os.str());
}


}  // namespace framework
}  // namespace phoenix

#endif // PHOENIX_FRAMEWORK_MSG_QUEUE_H_

