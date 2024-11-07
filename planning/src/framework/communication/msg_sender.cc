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
          "planning/planning_result", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 " \"planning/planning_result\".";
    }
  }
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/fusion", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 " \"perception/fusion\".";
    }
  }
  // Add Planning debug BY ZQ for Ros
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->AddPublisher<::planning::plan_debug>(
          "planning/debug", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 " \"planning/debug\".";
    }
  }
  // adecu debug ros msg
  #if (ENTER_PLAYBACK_MODE_ADHMI)
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->AddPublisher<::planning::adecu_debug>(
          "planning/adecudebug", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 " \"planning/adecudebug\".";
    }
  }
  #endif

  if (Nullptr_t != ros_node_) {
    if (!ros_node_->AddPublisher<::planning::CanFrame>("map/adasisv2_raw", 10)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2_raw\".";
    }

    if (!ros_node_->AddPublisher<::planning::adasisv2_position>("map/adasisv2/position", 10)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2/position\".";
    }

    if (!ros_node_->AddPublisher<::planning::adasisv2_profileshort>("map/adasisv2/profileshort/slope", 10)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2/profileshort/slope\".";
    }
    
    if (!ros_node_->AddPublisher<::planning::adasisv2_profileshort>("map/adasisv2/profileshort/curvature", 10)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2/profileshort/curvature\".";
    }

    if (!ros_node_->AddPublisher<::planning::adasisv2_horizon>("map/adasisv2/horizon", 10)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2/horizon\".";
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

  return (true);
}

bool MsgSender::SendPlanningResult(const ad_msg::PlanningResult& pl_ret) {
  bool ret = false;

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::planning::PlanningResult message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodePlanningResultMessage(pl_ret, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "planning/planning_result", msg)) {
        LOG_ERR << "Failed to send message with topic"
                   " \"planning/planning_result\".";
      }
    } else {
      LOG_ERR << "Failed to serialize planning result.";
    }
  }
#endif // #if (ENABLE_ROS_NODE)

#if 0
#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::planning::PlanningResult message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodePlanningResultMessage(pl_ret, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "planning/planning_result", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send message with topic"
                   " \"planning/planning_result\".";
      }
    } else {
      LOG_ERR << "Failed to serialize planning result.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::PlanningResult);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodePlanningResultArray(
            serialization_data_buff_, 0, max_buff_size, &pl_ret, 1);
      udp_node_->Publish("planning/planning_result",
                         serialization_data_buff_, data_len);
    }
  }
#endif // #if (ENABLE_UDP_NODE)
#endif

  return (ret);
}

bool MsgSender::SendObstacleList(const ad_msg::ObstacleList &obj_list) {
  bool ret = true;
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::ObstacleList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleListMessage(obj_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/fusion", msg)) {
        ret = false;
        LOG_ERR << "Failed to send message with topic"
                   " \"perception/fusion\".";
      }
    } else {
      ret = false;
      LOG_ERR << "Failed to serialize obstacle_list.";
    }
  }
#endif
  ret = true;
  return (ret);
}

// Add Planning debug BY ZQ for Ros
bool MsgSender::SendPlannigDebug(const ::planning::plan_debug &pl_debug){
    bool ret = true;
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    if (!ros_node_->Publish<::planning::plan_debug>("planning/debug", pl_debug)) {
        LOG_ERR << "Failed to send message with topic \"planning/debug\".";
        ret = false;
      }
  }
#endif
  return(ret);
}
// adecu debug ros msg

bool MsgSender::SendAdecuDebug(const ::planning::adecu_debug &adecu_debug){
    bool ret = true;
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    if (!ros_node_->Publish<::planning::adecu_debug>("planning/adecudebug", adecu_debug)) {
        LOG_ERR << "Failed to send message with topic \"planning/adecudebug\".";
        ret = false;
      }
  }
#endif
  return(ret);
}

/// 保存原始的ADASIS v2 CAN报文，用于回放分析
void MsgSender::SendADASISV2CanFrame(const phoenix::can_dev::CanFrame& can_frame) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    ::planning::CanFrame can_ros_msg;
    can_ros_msg.time_stamp = can_frame.time_stamp;
    can_ros_msg.id = can_frame.id;
    can_ros_msg.rtr = can_frame.RTR;
    can_ros_msg.ext = can_frame.EXT;
    can_ros_msg.data_len = can_frame.data_len;
    for(Uint8_t i = 0; i < can_frame.data_len; i++) {
      can_ros_msg.data.push_back(can_frame.data[i]);
    }
    
    if (!ros_node_->Publish<::planning::CanFrame>("map/adasisv2_raw", can_ros_msg)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2_raw\".";
    }
  }
#endif
}

void MsgSender::SendADASISV2Position(const phoenix::adasisv2::PositionMessage& position) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    ::planning::adasisv2_position pos_ros_msg;
    pos_ros_msg.time_stamp = position.time_stamp;
    pos_ros_msg.type = position.type;
    pos_ros_msg.cyclic_counter = position.cyclic_counter;
    pos_ros_msg.path_index = position.path_index;
    pos_ros_msg.offset = position.offset;
    pos_ros_msg.position_index = position.position_index;
    pos_ros_msg.positioin_age = position.positioin_age;
    pos_ros_msg.slope = position.slope;
    pos_ros_msg.relative_heading = position.relative_heading;
    pos_ros_msg.position_probability = position.position_probability;
    pos_ros_msg.position_confidence = position.position_confidence;
    pos_ros_msg.current_lane = position.current_lane;

    if (!ros_node_->Publish<::planning::adasisv2_position>("map/adasisv2/position", pos_ros_msg)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2/position\".";
    }
  }
#endif
}

void MsgSender::SendADASISV2ProfileShort(const phoenix::adasisv2::ProfileShortMessage& profileshort) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    ::planning::adasisv2_profileshort profileshort_ros_msg;
    profileshort_ros_msg.time_stamp = profileshort.time_stamp;
    profileshort_ros_msg.type = profileshort.type;
    profileshort_ros_msg.cyclic_counter = profileshort.cyclic_counter;
    profileshort_ros_msg.retransmission = profileshort.retransmission;
    profileshort_ros_msg.path_index = profileshort.path_index;
    profileshort_ros_msg.update = profileshort.update;
    profileshort_ros_msg.profile_type = profileshort.profile_type;
    profileshort_ros_msg.control_point = profileshort.control_point;
    profileshort_ros_msg.offset = profileshort.offset;
    profileshort_ros_msg.value0 = profileshort.value0;
    profileshort_ros_msg.distance1 = profileshort.distance1;
    profileshort_ros_msg.value1 = profileshort.value1;
    profileshort_ros_msg.accuracy = profileshort.accuracy;

    if (phoenix::adasisv2::PROFILE_SHORT_TYPE_SLOPE_LINEAR == profileshort.profile_type) {
      if (!ros_node_->Publish<::planning::adasisv2_profileshort>("map/adasisv2/profileshort/slope", profileshort_ros_msg)) {
        LOG_ERR << "Failed to send message with topic \"map/adasisv2/profileshort/slope\".";
      }
    } else if (phoenix::adasisv2::PROFILE_SHORT_TYPE_CURVATURE == profileshort.profile_type) {
      if (!ros_node_->Publish<::planning::adasisv2_profileshort>("map/adasisv2/profileshort/curvature", profileshort_ros_msg)) {
        LOG_ERR << "Failed to send message with topic \"map/adasisv2/profileshort/curvature\".";
      }
    }
  }
#endif
}

void MsgSender::SendADASISV2Horizon(const phoenix::adasisv2::Horizon& horizon) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    ::planning::adasisv2_horizon horizon_ros_msg;
    horizon_ros_msg.position.time_stamp = horizon.position.time_stamp;
    horizon_ros_msg.position.type = horizon.position.type;
    horizon_ros_msg.position.cyclic_counter = horizon.position.cyclic_counter;
    horizon_ros_msg.position.path_index = horizon.position.path_index;
    horizon_ros_msg.position.offset = horizon.position.offset;
    horizon_ros_msg.position.position_index = horizon.position.position_index;
    horizon_ros_msg.position.positioin_age = horizon.position.positioin_age;
    horizon_ros_msg.position.slope = horizon.position.slope;
    horizon_ros_msg.position.relative_heading = horizon.position.relative_heading;
    horizon_ros_msg.position.position_probability = horizon.position.position_probability;
    horizon_ros_msg.position.position_confidence = horizon.position.position_confidence;
    horizon_ros_msg.position.current_lane = horizon.position.current_lane;
    horizon_ros_msg.position_offset_from_start = horizon.position_offset_from_start;
    horizon_ros_msg.position_offset_cyclic_num = horizon.position_offset_cyclic_num;

    horizon_ros_msg.profile_slope.time_stamp = horizon.profile_slope.time_stamp;
    horizon_ros_msg.profile_slope.type = horizon.profile_slope.type;
    horizon_ros_msg.profile_slope.cyclic_counter = horizon.profile_slope.cyclic_counter;
    horizon_ros_msg.profile_slope.retransmission = horizon.profile_slope.retransmission;
    horizon_ros_msg.profile_slope.path_index = horizon.profile_slope.path_index;
    horizon_ros_msg.profile_slope.update = horizon.profile_slope.update;
    horizon_ros_msg.profile_slope.profile_type = horizon.profile_slope.profile_type;
    horizon_ros_msg.profile_slope.control_point = horizon.profile_slope.control_point;
    horizon_ros_msg.profile_slope.offset = horizon.profile_slope.offset;
    horizon_ros_msg.profile_slope.value0 = horizon.profile_slope.value0;
    horizon_ros_msg.profile_slope.distance1 = horizon.profile_slope.distance1;
    horizon_ros_msg.profile_slope.value1 = horizon.profile_slope.value1;
    horizon_ros_msg.profile_slope.accuracy = horizon.profile_slope.accuracy;
    horizon_ros_msg.profile_slope_offset_from_start = horizon.profile_slope_offset_from_start;
    horizon_ros_msg.profile_slope_offset_cyclic_num = horizon.profile_slope_offset_cyclic_num;

    horizon_ros_msg.profile_curvatue.time_stamp = horizon.profile_curvatue.time_stamp;
    horizon_ros_msg.profile_curvatue.type = horizon.profile_curvatue.type;
    horizon_ros_msg.profile_curvatue.cyclic_counter = horizon.profile_curvatue.cyclic_counter;
    horizon_ros_msg.profile_curvatue.retransmission = horizon.profile_curvatue.retransmission;
    horizon_ros_msg.profile_curvatue.path_index = horizon.profile_curvatue.path_index;
    horizon_ros_msg.profile_curvatue.update = horizon.profile_curvatue.update;
    horizon_ros_msg.profile_curvatue.profile_type = horizon.profile_curvatue.profile_type;
    horizon_ros_msg.profile_curvatue.control_point = horizon.profile_curvatue.control_point;
    horizon_ros_msg.profile_curvatue.offset = horizon.profile_curvatue.offset;
    horizon_ros_msg.profile_curvatue.value0 = horizon.profile_curvatue.value0;
    horizon_ros_msg.profile_curvatue.distance1 = horizon.profile_curvatue.distance1;
    horizon_ros_msg.profile_curvatue.value1 = horizon.profile_curvatue.value1;
    horizon_ros_msg.profile_curvatue.accuracy = horizon.profile_curvatue.accuracy;
    horizon_ros_msg.profile_curvatue_offset_from_start = horizon.profile_curvatue_offset_from_start;
    horizon_ros_msg.profile_curvatue_offset_cyclic_num = horizon.profile_curvatue_offset_cyclic_num;


    horizon_ros_msg.is_ready = horizon.is_ready;

    horizon_ros_msg.slope_profile_num = horizon.slope_list.Size();
    horizon_ros_msg.slope_list.clear();
    // Plotjuggler最多只显示100个元素
    int slope_points = horizon_ros_msg.slope_profile_num > 100 ? 100 : horizon_ros_msg.slope_profile_num;
    ::planning::adasisv2_horizon_profile profile;
    for (Int32_t i = 0; i < slope_points; ++i) {
      profile.type = horizon.slope_list[i].type;
      profile.offset_from_start = horizon.slope_list[i].offset_from_start;
      profile.cyclic_offset = horizon.slope_list[i].cyclic_offset;
      profile.cyclic_segment_num = horizon.slope_list[i].cyclic_segment_num;
      profile.offset_from_ego = horizon.slope_list[i].offset_from_ego;
      profile.value = horizon.slope_list[i].value;
      horizon_ros_msg.slope_list.push_back(profile);
    }

    horizon_ros_msg.curvature_profile_num = horizon.curvature_list.Size();
    horizon_ros_msg.curvature_list.clear();
    // Plotjuggler最多只显示100个元素
    int curvature_points = horizon_ros_msg.curvature_profile_num > 100 ? 100 : horizon_ros_msg.curvature_profile_num;
    for (Int32_t i = 0; i < curvature_points; ++i) {
      profile.type = horizon.curvature_list[i].type;
      profile.offset_from_start = horizon.curvature_list[i].offset_from_start;
      profile.cyclic_offset = horizon.curvature_list[i].cyclic_offset;
      profile.cyclic_segment_num = horizon.curvature_list[i].cyclic_segment_num;
      profile.offset_from_ego = horizon.curvature_list[i].offset_from_ego;
      profile.value = horizon.curvature_list[i].value;
      horizon_ros_msg.curvature_list.push_back(profile);
    }

    if (!ros_node_->Publish<::planning::adasisv2_horizon>("map/adasisv2/horizon", horizon_ros_msg)) {
      LOG_ERR << "Failed to send message with topic \"map/adasisv2/horizon\".";
    }
  }
#endif
}

}  // namespace framework
}  // namespace phoenix

