/**
 * @file task_recv_adasisv2_msg.h
 * @author wangwh
 * @brief work thread to deal with ADASISv2 CAN/ETH message
 * @date 2023-06-08
 */

#ifndef PHOENIX_MAP_TASK_RECV_ADASISV2_MSG_H_
#define PHOENIX_MAP_TASK_RECV_ADASISV2_MSG_H_

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include "common/task.h"
#include "can_dev/can_driver.h"
#include "pcc_map/adasisv2_msg_parser.h"

namespace phoenix {
namespace adasisv2 {

class TaskRecvADASISv2Msg : public framework::Task {
public:
  explicit TaskRecvADASISv2Msg(framework::Task* manager);
  ~TaskRecvADASISv2Msg();

  bool Start();
  bool Stop();

private:
  void ThreadReceiving();

  void ParseCanFrame(const can_dev::CanFrame& frame);

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  can_dev::CanDriver* can_channel_;

  boost::atomic_bool running_flag_recv_;
  boost::thread thread_recv_;

  // ADASIS v2 原始CAN数据
  phoenix::can_dev::CanFrame can_frame_;

  // ADASIS v2 CAN消息解析后的数据结构
  ADASISv2MsgParser parser_;
};

}  // namespace map
}  // namespace phoenix


#endif  // PHOENIX_MAP_TASK_RECV_ADASISV2_MSG_H_
