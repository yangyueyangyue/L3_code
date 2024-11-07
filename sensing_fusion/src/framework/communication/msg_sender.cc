/******************************************************************************
 ** 消息发送模块
 ******************************************************************************
 *
 *  消息发送模块(发送报文到总线上)
 *
 *  @file       msg_sender.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/msg_sender.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#if (ENABLE_ROS_NODE)
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#endif

#include "utils/com_utils.h"
#include "math/math_utils.h"

#include "data_serialization.h"
#include "communication/parse_proto_msg.h"


namespace phoenix {
namespace framework {


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      manager         (IN)           任务管理模块
 *              ros_node        (IN)           ros节点模块
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
MsgSender::MsgSender(Task* manager) :
  Task(TASK_ID_MSG_SENDER, "Message Sender", manager) {
#if (ENABLE_ROS_NODE)
  ros_node_ = Nullptr_t;
#endif

#if (ENABLE_LCM_NODE)
  lcm_node_ = Nullptr_t;
#endif

#if (ENABLE_UDP_NODE)
  udp_node_ = Nullptr_t;
#endif
}

/******************************************************************************/
/** 启动消息发送模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息发送模块，注册到总线上
 *
 *  <Attention>
 *       None
 *
 */
bool MsgSender::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "localization/gnss", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"localization/gnss\".";
    }
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "localization/imu", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"localization/imu\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
                "perception/fusion", 10);
    if(false == ret){
        LOG_ERR << "Failed to add publisher with topic"
                   "\"perception/fusion\".";
    }

    ret = ros_node_->AddPublisher<::sensing_fusion::perception_debug>(
                "perception/debug", 10);
    if(false == ret){
        LOG_ERR << "Failed to add publisher with topic"
                   "\"perception/debug\".";
    }


  }
#endif

  return (ret);
}

void MsgSender::SendGnssData(const ad_msg::Gnss gnss){
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::localization::Gnss message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeGnssMessage(gnss, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "localization/gnss", msg)) {
        LOG_ERR << "Failed to send GNSS message.";
      }
    } else {
      LOG_ERR << "Failed to serialize GNSS message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::localization::Gnss message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeGnssMessage(gnss, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "localization/gnss", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send GNSS message.";
      }
    } else {
      LOG_ERR << "Failed to serialize GNSS message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::Gnss);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeGnssArray(
            serialization_data_buff_, 0, max_buff_size, &gnss, 1);
      udp_node_->Publish("localization/gnss",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}

void MsgSender::SendImuData(const ad_msg::Imu imu){
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::localization::Imu message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeImuMessage(imu, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "localization/imu", msg)) {
        LOG_ERR << "Failed to send IMU message.";
      }
    } else {
      LOG_ERR << "Failed to serialize IMU message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::localization::Imu message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeImuMessage(imu, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "localization/imu", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send IMU message.";
      }
    } else {
      LOG_ERR << "Failed to serialize IMU message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::Imu);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeImuArray(
            serialization_data_buff_, 0, max_buff_size, &imu, 1);
      udp_node_->Publish("localization/imu",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}

void MsgSender::SendObstacleFusionList(const ad_msg::ObstacleList &objs_list)
{
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::ObstacleList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleListMessage(objs_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/fusion", msg)) {
        LOG_ERR << "Failed to send obstacles fusion message.";
      }
    } else {
      LOG_ERR << "Failed to serialize obstacles fusion message.";
    }
  }
#endif
}

// Add Percetpion debug BY wzf for Ros
bool MsgSender::SendPerceptionDebug(const ::sensing_fusion::perception_debug &per_debug)
{
  bool ret = true;
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    if (!ros_node_->Publish<::sensing_fusion::perception_debug>("perception/debug", per_debug)) {
        LOG_ERR << "Failed to send message with topic \"perception/debug\".";
        ret = false;
      }
  }
#endif
  return(ret);
}


}  // namespace communication
}  // namespace phoenix

