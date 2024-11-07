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
#ifndef PHOENIX_FRAMEWORK_MSG_SENDER_H_
#define PHOENIX_FRAMEWORK_MSG_SENDER_H_

#include "utils/macros.h"
#include "os/mutex.h"

#if (ENABLE_ROS_NODE)
#include "communication/ros_node.h"
#include "planning/plan_debug.h"
#include "planning/CanFrame.h"
#include "planning/adasisv2_position.h"
#include "planning/adasisv2_profileshort.h"
#include "planning/adasisv2_horizon_profile.h"
#include "planning/adasisv2_horizon.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/task.h"
#include "common/message.h"


namespace phoenix {
namespace framework {


class MsgSender : public Task {
public:
  MsgSender(Task* manager);

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

  bool SendPlanningResult(const ad_msg::PlanningResult& pl_ret);
  bool SendObstacleList(const ad_msg::ObstacleList& obj_list);
  bool SendPlannigDebug(const ::planning::plan_debug &pl_debug);
  bool SendAdecuDebug(const ::planning::adecu_debug &adecu_debug);
  void SendADASISV2CanFrame(const phoenix::can_dev::CanFrame& can_frame);
  void SendADASISV2Position(const phoenix::adasisv2::PositionMessage& position);
  void SendADASISV2ProfileShort(const phoenix::adasisv2::ProfileShortMessage& profileshort);
  void SendADASISV2Horizon(const phoenix::adasisv2::Horizon& horizon);

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

  common::os::Mutex lock_serialization_data_buff_;
  Char_t serialization_data_buff_[1*1024*1024];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_SENDER_H_

