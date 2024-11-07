/******************************************************************************
 ** 消息发送模块
 ******************************************************************************
 *
 *  消息发送模块(发送报文到总线上)
 *
 *  @file       msg_sender.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_HMI_MSG_SENDER_H_
#define PHOENIX_FRAMEWORK_HMI_MSG_SENDER_H_

#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include "communication/ros_node.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/task.h"
#include "common/message.h"
#include "lateral_control_c.h"


namespace phoenix {
namespace framework {


class HmiMsgSender : public Task {
public:
  HmiMsgSender(Task* manager);

#if (ENABLE_ROS_NODE)
  void SetRosNode(RosNode* node) {
    ros_node_ = node;
  }
#endif

#if (ENABLE_LCM_NODE)
  void SetLcmNode(LcmNode* node) {
    lcm_node_ = node;
  }
#endif

#if (ENABLE_UDP_NODE)
  void SetUdpNode(UdpNode* node) {
    udp_node_ = node;
  }
#endif

  bool Start();

  void SendHmiMsg();

private:
#if (ENABLE_ROS_NODE)
  RosNode* ros_node_;
#endif

#if (ENABLE_LCM_NODE)
  LcmNode* lcm_node_;
#endif

#if (ENABLE_UDP_NODE)
  UdpNode* udp_node_;
#endif

  ModuleStatusList_t module_status_list_;
  LateralControlInfo_t lateral_control_info_;

  Char_t serialization_data_buff_[100*1024];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_HMI_MSG_SENDER_H_

