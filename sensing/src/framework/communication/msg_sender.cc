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
#if (!ENTER_PLAYBACK_MODE)
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/ars_radar_can_frame", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                  "\"perception/ars_radar_can_frame\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/camera_canfd_frame", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                  "\"perception/camera_canfd_frame\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/lidar_canfd_frame", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                  "\"perception/lidar_canfd_frame\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/anngic_radar_canfd_frame", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                  "\"perception/anngic_radar_canfd_frame\".";
    }
#endif

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
          "perception/lane_mark_camera", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/lane_mark_camera\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/lane_curb_camera", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/lane_curb_camera\".";
    }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/lane_mark", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/lane_mark\".";
    // }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_maxieye_camera", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_maxieye_camera\".";
    }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/maxieye", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/maxieye\".";
    // }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/visual_control_lane_mark_camera", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/visual_control_lane_mark_camera\".";
    }
    
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_visual_control_front", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_visual_control_front\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_visual_control_front_left", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_visual_control_front_left\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_visual_control_front_right", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_visual_control_front_right\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_visual_control_rear_left", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_visual_control_rear_left\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_visual_control_rear_right", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_visual_control_rear_right\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_visual_control_rear", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_visual_control_rear\".";
    }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/visual_control_front", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/visual_control_front\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/visual_control_front_left", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/visual_control_front_left\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/visual_control_front_right", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/visual_control_front_right\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/visual_control_rear_left", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/visual_control_rear_left\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/visual_control_rear_right", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/visual_control_rear_right\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/visual_control_rear", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/visual_control_rear\".";
    // }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_ars_radar", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_ars_radar\".";
    }
    
    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/ars", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/ars\".";
    // }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_anngic_radar_front_left", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_anngic_radar_front_left\".";
    }
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_anngic_radar_front_right", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_anngic_radar_front_right\".";
    }
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_anngic_radar_rear_left", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_anngic_radar_rear_left\".";
    }
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_anngic_radar_rear_right", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_anngic_radar_rear_right\".";
    }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/anngic_radar_front_left", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/anngic_radar_front_left\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/anngic_radar_front_right", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/anngic_radar_front_right\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/anngic_radar_rear_left", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/anngic_radar_rear_left\".";
    // }

    // ret = ros_node_->AddPublisher<visualization_msgs::MarkerArray>(
    //       "visualization/anngic_radar_rear_right", 10);
    // if (false == ret) {
    //   LOG_ERR << "Failed to add publisher with topic"
    //              "\"visualization/anngic_radar_rear_right\".";
    // }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/obstacle_lidar_0", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/obstacle_lidar_0\".";
    }
    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/lidar_cloud_0", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/lidar_cloud_0\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/traffic_signal", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"perception/traffic_signal\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "mpu/state", 10);
    if (false == ret) {
      LOG_ERR << "Failed to add publisher with topic"
                 "\"mpu/state\".";
    }

    ret = ros_node_->AddPublisher<std_msgs::ByteMultiArray>(
          "perception/fusion", 10);
    if(false == ret){
        LOG_ERR << "Failed to add publisher with topic"
                   "\"perception/fusion\".";
    }

    ret = ros_node_->AddPublisher<::sensing::obstacle_fusion_list>(
          "adecu_debug/fusion", 10);
    if(false == ret){
        LOG_ERR << "Failed to add publisher with topic"
                   "\"adecu_debug/fusion\".";
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

void MsgSender::SendCanFrameList(const phoenix::can_dev::CanFrameList& can_frame_list) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    can_dev::perception::CanFrameList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeCanFrameListMessage(can_frame_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/ars_radar_can_frame", msg)) {
        LOG_ERR << "Failed to send Can Frame message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Can Frame message.";
    }
  }
#endif
}

void MsgSender::SendCanFdFrameList(const phoenix::can_dev::CanFdFrameList& canfd_frame_list, Int32_t canfd_idx) {
  std::string topic = "";
  switch (canfd_idx) {
  case (0):
    topic = "perception/camera_canfd_frame";
    break;
  case (1):
    topic = "perception/lidar_canfd_frame";
    break;
  case (2):
    topic = "perception/anngic_radar_canfd_frame";
    break;
  default:
    LOG_ERR << "Invalid radar_idx.";
    break;
  }
  if ("" == topic) {
    LOG_ERR << "Invalid topic name.";
    return;
  }

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    can_dev::perception::CanFdFrameList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeCanFdFrameListMessage(canfd_frame_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            topic, msg)) {
        LOG_ERR << "Failed to send Canfd Frame message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Canfd Frame message.";
    }
  }
#endif
}

void MsgSender::SendLaneMarkCameraList(
    const ad_msg::LaneMarkCameraList lane_mark_camera_list) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::LaneMarkCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaneMarkCameraListMessage(lane_mark_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/lane_mark_camera", msg)) {
        LOG_ERR << "Failed to send Camera Lane message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Lane message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::perception::LaneMarkCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaneMarkCameraListMessage(lane_mark_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "perception/lane_mark_camera", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send Camera Lane message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Lane message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::LaneMarkCameraList);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeLaneMarkCameraListArray(
            serialization_data_buff_, 0, max_buff_size, &lane_mark_camera_list, 1);
      udp_node_->Publish("perception/lane_mark_camera",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}

void MsgSender::SendLaneCurbCameraList(
    const ad_msg::LaneMarkCameraList lane_curb_camera_list) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::LaneMarkCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaneMarkCameraListMessage(lane_curb_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/lane_curb_camera", msg)) {
        LOG_ERR << "Failed to send Camera Lane message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Lane message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::perception::LaneMarkCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaneMarkCameraListMessage(lane_curb_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "perception/lane_curb_camera", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send Camera Lane message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Lane message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::LaneMarkCameraList);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeLaneMarkCameraListArray(
            serialization_data_buff_, 0, max_buff_size, &lane_curb_camera_list, 1);
      udp_node_->Publish("perception/lane_curb_camera",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}

void MsgSender::SendVisualControlLaneMarkCameraList(
    const ad_msg::LaneMarkCameraList lane_mark_camera_list) {
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::LaneMarkCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaneMarkCameraListMessage(lane_mark_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/visual_control_lane_mark_camera", msg)) {
        LOG_ERR << "Failed to send Camera Lane message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Lane message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::perception::LaneMarkCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLaneMarkCameraListMessage(lane_mark_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "perception/visual_control_lane_mark_camera", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send Camera Lane message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Lane message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::LaneMarkCameraList);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeLaneMarkCameraListArray(
            serialization_data_buff_, 0, max_buff_size, &lane_mark_camera_list, 1);
      udp_node_->Publish("perception/visual_control_lane_mark_camera",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}

// void MsgSender::SendVisualLaneMark(const visualization_msgs::MarkerArray &msg) {
// #if (ENABLE_ROS_NODE)
//   if (Nullptr_t != ros_node_) {
//     if (!ros_node_->Publish<visualization_msgs::MarkerArray>("visualization/lane_mark", msg)) {
//       LOG_ERR << "Failed to send lane mark.";
//     }
//   }
// #endif
// }

void MsgSender::SendObstacleCameraList(
    const ad_msg::ObstacleCameraList obstacle_camera_list, Int32_t camera_idx) {
  std::string topic = "";
  switch (camera_idx) {
  case (0):
    topic = "perception/obstacle_maxieye_camera";
    break;
  case (1):
    topic = "perception/obstacle_visual_control_front";
    break;
  case (2):
    topic = "perception/obstacle_visual_control_front_left";
    break;
  case (3):
    topic = "perception/obstacle_visual_control_front_right";
    break;
  case (4):
    topic = "perception/obstacle_visual_control_rear_left";
    break;
  case (5):
    topic = "perception/obstacle_visual_control_rear_right";
    break;
  case (6):
    topic = "perception/obstacle_visual_control_rear";
    break;
  default:
    LOG_ERR << "Invalid radar_idx.";
    break;
  }
  if ("" == topic) {
    LOG_ERR << "Invalid topic name.";
    return;
  }

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::ObstacleCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleCameraListMessage(obstacle_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            topic, msg)) {
        LOG_ERR << "Failed to send Camera Obstacles message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Obstacles message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::perception::ObstacleCameraList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleCameraListMessage(obstacle_camera_list, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "perception/obstacle_camera", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send Camera Obstacles message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Camera Obstacles message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::ObstacleCameraList);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeObstacleCameraListArray(
            serialization_data_buff_, 0, max_buff_size, &obstacle_camera_list, 1);
      udp_node_->Publish("perception/obstacle_camera",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}


void MsgSender::SendMpuStateData(
    const MpuState::MonitorMpuState& monitor_mpu_state) {
  std::string topic = "mpu/state";
  
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {

    MpuState::MonitorMpuState message = monitor_mpu_state;
    ParseProtoMsg parse_msg;
    // parse_msg.EncodeMpuStateMessage(monitor_mpu_state, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            topic, msg)) {
        LOG_ERR << "Failed to send mpu state message.";
      }
    } else {
      LOG_ERR << "Failed to serialize mpu state message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    MpuState::MonitorMpuState message = monitor_mpu_state;
    ParseProtoMsg parse_msg;
    // parse_msg.EncodeMpuStateMessage(monitor_mpu_state, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "mpu/state", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send mpu state message.";
      }
    } else {
      LOG_ERR << "Failed to serialize mpu state message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)

#endif

}



// void MsgSender::SendMaxieyeVisualMarkArray(const visualization_msgs::MarkerArray &msg) {
// #if (ENABLE_ROS_NODE)
//   if (Nullptr_t != ros_node_) {
//     if (!ros_node_->Publish<visualization_msgs::MarkerArray>("visualization/maxieye", msg)) {
//       LOG_ERR << "Failed to send Maxieye visualization message.";
//     }
//   }
// #endif
// }

void MsgSender::SendObstacleRadarList(
    const ad_msg::ObstacleRadarList& obj_list, Int32_t radar_idx) {
  std::string topic = "";
  switch (radar_idx) {
  case (0):
    topic = "perception/obstacle_ars_radar";
    break;
  case (1):
    topic = "perception/obstacle_anngic_radar_front_left";
    break;
  case (2):
    topic = "perception/obstacle_anngic_radar_front_right";
    break;
  case (3):
    topic = "perception/obstacle_anngic_radar_rear_left";
    break;
  case (4):
    topic = "perception/obstacle_anngic_radar_rear_right";
    break;
  default:
    LOG_ERR << "Invalid radar_idx.";
    break;
  }
  if ("" == topic) {
    LOG_ERR << "Invalid topic name.";
    return;
  }

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::ObstacleRadarList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleRadarListMessage(obj_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(topic, msg)) {
        LOG_ERR << "Failed to send Radar Obstacles message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Radar Obstacles message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::perception::ObstacleRadarList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleRadarListMessage(obj_list, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(topic, data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send Radar Obstacles message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Radar Obstacles message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::ObstacleRadarList);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeObstacleRadarListArray(
            serialization_data_buff_, 0, max_buff_size, &obj_list, 1);
      udp_node_->Publish(topic.c_str(), serialization_data_buff_, data_len);
    }
  }
#endif
}

// void MsgSender::SendArsVisualMarkArray(const visualization_msgs::MarkerArray &msg) {
// #if (ENABLE_ROS_NODE)
//   if (Nullptr_t != ros_node_) {
//     if (!ros_node_->Publish<visualization_msgs::MarkerArray>("visualization/ars", msg)) {
//       LOG_ERR << "Failed to send ARS visualization message.";
//     }
//   }
// #endif
// }

// void MsgSender::SendVisualControlVisualMarkArray(const visualization_msgs::MarkerArray &msg, Int32_t camera_idx) {
//   std::string topic = "";
//   switch (camera_idx) {
//   case (0):
//     topic = "visualization/visual_control_front";
//     break;
//   case (1):
//     topic = "visualization/visual_control_front_left";
//     break;
//   case (2):
//     topic = "visualization/visual_control_front_right";
//     break;
//   case (3):
//     topic = "visualization/visual_control_rear_left";
//     break;
//   case (4):
//     topic = "visualization/visual_control_rear_right";
//     break;
//   case (5):
//     topic = "visualization/visual_control_rear";
//     break;
//   default:
//     LOG_ERR << "Invalid lidar index.";
//     break;
//   }
//   if ("" == topic) {
//     LOG_ERR << "Invalid topic name.";
//     return;
//   }

// #if (ENABLE_ROS_NODE)
//   if (Nullptr_t != ros_node_) {
//     if (!ros_node_->Publish<visualization_msgs::MarkerArray>(topic, msg)) {
//       LOG_ERR << "Failed to send Visual Control visualization message.";
//     }
//   }
// #endif
// }

// void MsgSender::SendAnngicRadarVisualMarkArray(const visualization_msgs::MarkerArray &msg, Int32_t radar_idx) {
//   std::string topic = "";
//   switch (radar_idx) {
//   case (0):
//     topic = "visualization/anngic_radar_front_left";
//     break;
//   case (1):
//     topic = "visualization/anngic_radar_front_right";
//     break;
//   case (2):
//     topic = "visualization/anngic_radar_rear_left";
//     break;
//   case (3):
//     topic = "visualization/anngic_radar_rear_right";
//     break;
//   default:
//     LOG_ERR << "Invalid lidar index.";
//     break;
//   }
//   if ("" == topic) {
//     LOG_ERR << "Invalid topic name.";
//     return;
//   }

// #if (ENABLE_ROS_NODE)
//   if (Nullptr_t != ros_node_) {
//     if (!ros_node_->Publish<visualization_msgs::MarkerArray>(topic, msg)) {
//       LOG_ERR << "Failed to send Anngic Radar visualization message.";
//     }
//   }
// #endif
// }

void MsgSender::SendObstacleLidarList(
    const ad_msg::ObstacleLidarList& obj_list, Int32_t lidar_idx) {
  std::string topic = "";
  switch (lidar_idx) {
  case (0):
    topic = "perception/obstacle_lidar_0";
    break;
  default:
    LOG_ERR << "Invalid lidar index.";
    break;
  }
  if ("" == topic) {
    LOG_ERR << "Invalid topic name.";
    return;
  }

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::ObstacleLidarList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeObstacleLidarListMessage(obj_list, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(topic, msg)) {
        LOG_ERR << "Failed to send Lidar Obstacles message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Lidar Obstacles message.";
    }
  }
#endif
}

void MsgSender::SendLidarCloud(
    const ad_msg::LidarCloud& cloud, Int32_t lidar_idx) {
  std::string topic = "";
  switch (lidar_idx) {
  case (0):
    topic = "perception/lidar_cloud_0";
    break;
  default:
    LOG_ERR << "Invalid lidar index.";
    break;
  }
  if ("" == topic) {
    LOG_ERR << "Invalid topic name.";
    return;
  }

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::LidarCloud message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeLidarCloudMessage(cloud, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(topic, msg)) {
        LOG_ERR << "Failed to send Lidar Cloud message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Lidar Cloud message.";
    }
  }
#endif
}

void MsgSender::SendTrafficSignalList(
    const ad_msg::TrafficSignalList& traffic_signal) {

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    msg::perception::TrafficSignalList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeTrafficSignalListMessage(traffic_signal, &message);

    Int32_t serialize_size = message.ByteSize();
    std_msgs::ByteMultiArray msg;
    msg.data.resize(serialize_size);
    if (message.SerializeToArray(&msg.data[0], serialize_size)) {
      if (!ros_node_->Publish<std_msgs::ByteMultiArray>(
            "perception/traffic_signal", msg)) {
        LOG_ERR << "Failed to send traffic_signal message.";
      }
    } else {
      LOG_ERR << "Failed to serialize traffic_signal message.";
    }
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    msg::perception::TrafficSignalList message;
    ParseProtoMsg parse_msg;
    parse_msg.EncodeTrafficSignalListMessage(traffic_signal, &message);

    Int32_t serialize_size = message.ByteSize();
    Uint8_t* data_buff = new Uint8_t[serialize_size];
    if (message.SerializeToArray(data_buff, serialize_size)) {
      if (lcm_node_->Publish(
            "perception/traffic_signal", data_buff, serialize_size) < 0) {
        LOG_ERR << "Failed to send Traffic Signal message.";
      }
    } else {
      LOG_ERR << "Failed to serialize Traffic Signal message.";
    }
    delete [] data_buff;
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;

    // send planning result
    Int32_t data_size = sizeof(ad_msg::TrafficSignalList);
    if (data_size > max_buff_size) {
      LOG_ERR << "The size of serialization buffer is not enough.";
    } else {
      common::os::LockHelper lock(lock_serialization_data_buff_);

      Int32_t data_len = data_serial::EncodeTrafficSignalListArray(
            serialization_data_buff_, 0, max_buff_size, &traffic_signal, 1);
      udp_node_->Publish("perception/traffic_signal",
                         serialization_data_buff_, data_len);
    }
  }
#endif
}

void MsgSender::SendADHMIObstacleFusionList(const ad_msg::ObstacleList &objs_list)
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

void set_obstacle_fusion_info_value(::sensing::obstacle_fusion_info& info, const ad_msg::Obstacle& obj)
{
  info.id = obj.id;
  info.x = obj.x;
  info.y = obj.y;
  info.height = obj.height;
  info.height_to_ground = obj.height_to_ground;
  info.type = obj.type;
  info.dynamic = obj.dynamic;
  info.confidence = obj.confidence;
  info.perception_type = obj.perception_type;
  info.v_x = obj.v_x;
  info.v_y = obj.v_y;
  info.v = obj.v;
  info.a_x = obj.a_x;
  info.a_y = obj.a_y;
  info.a = obj.a;
  info.obb.x = obj.obb.x;
  info.obb.y = obj.obb.y;
  info.obb.heading = obj.obb.heading;
  info.obb.half_width = obj.obb.half_width;
  info.obb.half_length = obj.obb.half_length;
}


void MsgSender::SendADHMIObstacleDebugFusionList(const ad_msg::ObstacleList &objs_list)
{
#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    // objs_list ADHMI_FUSION_DEBUG_TARGET_ID 
    int target_id   = ADECU_FUSION_DEBUG_TARGET_ID_0;
    int target_id_1 = ADECU_FUSION_DEBUG_TARGET_ID_1;
    int target_id_2 = ADECU_FUSION_DEBUG_TARGET_ID_2;
    int target_id_3 = ADECU_FUSION_DEBUG_TARGET_ID_3;
    ::sensing::obstacle_fusion_list obstacle_fusion_list;
    for (Int32_t i = 0; i < objs_list.obstacle_num; ++i) {
    // 障碍物ID
      ::sensing::obstacle_fusion_info obstacleInfo;
      set_obstacle_fusion_info_value(obstacleInfo,  objs_list.obstacles[i]);
      obstacle_fusion_list.obstacles.push_back(obstacleInfo);
      if(target_id == objs_list.obstacles[i].id)
      {
        set_obstacle_fusion_info_value(obstacle_fusion_list.target_obstacle,  objs_list.obstacles[i]); 
      }
      if(target_id_1 == objs_list.obstacles[i].id)
      {
        set_obstacle_fusion_info_value(obstacle_fusion_list.target_obstacle_1,  objs_list.obstacles[i]); 
      }
      if(target_id_2 == objs_list.obstacles[i].id)
      {
        set_obstacle_fusion_info_value(obstacle_fusion_list.target_obstacle_2,  objs_list.obstacles[i]); 
      }
      if(target_id_3 == objs_list.obstacles[i].id)
      {
        set_obstacle_fusion_info_value(obstacle_fusion_list.target_obstacle_3,  objs_list.obstacles[i]); 
      }
    }
    if(!ros_node_->Publish<::sensing::obstacle_fusion_list>("adecu_debug/fusion", obstacle_fusion_list)) 
    {
      LOG_ERR << "Failed to send obstacles fusion message.";
    }

  }
#endif
}


}  // namespace communication
}  // namespace phoenix

