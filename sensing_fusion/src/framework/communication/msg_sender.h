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
#include "os/thread.h"

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

  void SendImuData(const ad_msg::Imu imu);
  void SendGnssData(const ad_msg::Gnss gnss);

  void SendObstacleFusionList(const ad_msg::ObstacleList& objs_list); //增加融合结果发送

  bool SendPerceptionDebug(const ::sensing_fusion::perception_debug &per_debug);


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

