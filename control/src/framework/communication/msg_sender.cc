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

#include "data_serialization_c.h"
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
          "control/chassis", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic \"control/chassis\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "control/special_chassis_info", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic \"control/special_chassis_info\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "control/chassis_ctl_cmd", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic \"control/chassis_ctl_cmd\".";
    }

    ret = ros_node_->AddPublisher<control::control_debug>("control/debug", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic \"control/debug\".";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    // empty
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    // empty
  }
#endif

  return (ret);
}

bool MsgSender::SendChassis(const Chassis_t& chassis) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::control::Chassis message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeChassisMessage(chassis, &message);

    // send
    int serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "control/chassis", msg)) {
        LOG_ERR << "Failed to send message with topic \"control/chassis\".";
      }
    } else {
      LOG_ERR << "Failed to serialize chassis data.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::control::Chassis message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeChassisMessage(chassis, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "control/chassis", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send message with topic"
                   " \"control/chassis\".";
      }
    } else {
      LOG_ERR << "Failed to serialize chassis.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(Chassis_t);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = Phoenix_Common_EncodeChassisArray(
            serialization_data_buff_, 0, max_buff_size, &chassis, 1);
      udp_node_->Publish("control/chassis",
                         serialization_data_buff_, data_len);
    }
  }
#endif // #if (ENABLE_UDP_NODE)

  return (true);
}

bool MsgSender::SendSpecialChassisInfo(const SpecialChassisInfo_t& chassis) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::control::SpecialChassisInfo message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeSpecialChassisInfoMessage(chassis, &message);

    // send
    int serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "control/special_chassis_info", msg)) {
        LOG_ERR << "Failed send send message with "
                   "topic \"control/special_chassis_info\".";
      }
    } else {
      LOG_ERR << "Failed to serialize special chassis info.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::control::SpecialChassisInfo message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeSpecialChassisInfoMessage(chassis, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "control/special_chassis_info", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send message with topic"
                   " \"control/special_chassis_info\".";
      }
    } else {
      LOG_ERR << "Failed to serialize chassis.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(SpecialChassisInfo_t);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = Phoenix_Common_EncodeSpecialChassisInfoArray(
            serialization_data_buff_, 0, max_buff_size, &chassis, 1);
      udp_node_->Publish("control/special_chassis_info",
                         serialization_data_buff_, data_len);
    }
  }
#endif // #if (ENABLE_UDP_NODE)

  return (true);
}

bool MsgSender::SendChassisCtlCmd(const ChassisCtlCmd_t& chassis_ctl_cmd) {
#if (ENABLE_ROS_NODE)
  if (nullptr != ros_node_) {
    msg::control::ChassisCtlCmd message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeChassisCtlCmdMessage(chassis_ctl_cmd, &message);

    // send
    int serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "control/chassis_ctl_cmd", msg)) {
        LOG_ERR << "Failed to send message with topic "
                   "\"control/chassis_ctl_cmd\".";
      }
    } else {
      LOG_ERR << "Failed to serialize chassis_ctl_cmd.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::control::ChassisCtlCmd message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeChassisCtlCmdMessage(chassis_ctl_cmd, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "control/chassis_ctl_cmd", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send message with topic"
                   " \"control/chassis_ctl_cmd\".";
      }
    } else {
      LOG_ERR << "Failed to serialize chassis.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ChassisCtlCmd_t);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = Phoenix_Common_EncodeChassisCtlCmdArray(
            serialization_data_buff_, 0, max_buff_size, &chassis_ctl_cmd, 1);
      udp_node_->Publish("control/chassis_ctl_cmd",
                         serialization_data_buff_, data_len);
    }
  }
#endif // #if (ENABLE_UDP_NODE)

  return (true);
}


bool MsgSender::SendControlDebugInfo(const control::control_debug& ctrl_debug) {
#if (ENABLE_ROS_NODE)
  if (nullptr != ros_node_) {
    if (!ros_node_->Publish<control::control_debug>("control/debug", ctrl_debug)) {
        LOG_ERR << "Failed to send message with topic \"control/debug\".";
      }
  }
#endif
  return true;
}
}  // namespace framework
}  // namespace phoenix

