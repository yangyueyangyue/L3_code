/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/msg_receiver.h"

#include "utils/com_utils.h"
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
MsgReceiver::MsgReceiver(Task* manager) :
  Task(TASK_ID_MSG_RECEIVER, "Message Receiver", manager) {
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
/** 启动消息接收模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息接收模块，向总线订阅需要的报文
 *
 *  <Attention>
 *       None
 *
 */
bool MsgReceiver::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  /// Comunicate by ROS
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->Subscribe(
          "control/chassis", 1,
          &MsgReceiver::HandleChassisMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }

  ret = ros_node_->Subscribe(
          "perception/lane_mark_camera", 1,
                &MsgReceiver::HandleLaneMarkCameraMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/lane_mark_camera\".";
  }

  ret = ros_node_->Subscribe(
          "perception/lane_curb_camera", 1,
                &MsgReceiver::HandleLaneCurbCameraMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/lane_curb_camera\".";
  }


  ret = ros_node_->Subscribe(
          "mpu/state", 1,
                &MsgReceiver::HandleMpuStateMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"mpu/state\".";
  }




  ret = ros_node_->Subscribe(
          "perception/obstacle_maxieye_camera", 1,
                &MsgReceiver::HandleObstacleMaxieyeCameraMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_maxieye_camera\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_visual_control_front", 1,
                &MsgReceiver::HandleObstacleVisualControlFrontMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_visual_control_front\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_visual_control_front_left", 1,
                &MsgReceiver::HandleObstacleVisualControlFrontLeftMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_visual_control_front_left\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_visual_control_front_right", 1,
                &MsgReceiver::HandleObstacleVisualControlFrontRightMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_visual_control_front_right\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_visual_control_rear_left", 1,
                &MsgReceiver::HandleObstacleVisualControlRearLeftMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_visual_control_rear_left\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_visual_control_rear_right", 1,
                &MsgReceiver::HandleObstacleVisualControlRearRightMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_visual_control_rear_right\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_visual_control_rear", 1,
                &MsgReceiver::HandleObstacleVisualControlRearMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_visual_control_rear\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_ars_radar", 1,
                &MsgReceiver::HandleObstacleArsRadarMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_ars_radar\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_anngic_radar_front_left", 1,
                &MsgReceiver::HandleObstacleAnngicRadarFrontLeftMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_anngic_radar_front_left\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_anngic_radar_front_right", 1,
                &MsgReceiver::HandleObstacleAnngicRadarFrontRightMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_anngic_radar_front_right\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_anngic_radar_rear_left", 1,
                &MsgReceiver::HandleObstacleAnngicRadarRearLeftMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_anngic_radar_rear_left\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_anngic_radar_rear_right", 1,
                &MsgReceiver::HandleObstacleAnngicRadarRearRightMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_anngic_radar_rear_right\".";
  }

  ret = ros_node_->Subscribe(
          "perception/obstacle_lidar_0", 1,
                &MsgReceiver::HandleObstacleLidarFrontMessageRos,
          this);
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to \"perception/obstacle_lidar_0\".";
  }
}
#endif  // #if (ENABLE_ROS_NODE)

#if (ENABLE_LCM_NODE)
  /// Comunicate by LCM
  if (Nullptr_t != lcm_node_) {
    // empty
  }
#endif  // #if (ENABLE_LCM_NODE)

#if (ENABLE_UDP_NODE)
  /// Comunicate by UDP
  if (Nullptr_t != udp_node_) {
    // empty
  }
#endif  // #if (ENABLE_UDP_NODE)

  return (ret);
}

#if (ENABLE_ROS_NODE)
void MsgReceiver::HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_chassis_info_)) {

      MessageChassis msg_chassis(&ros_chassis_info_);
      Notify(msg_chassis);
    }
  }
}

void MsgReceiver::HandleLaneMarkCameraMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeLaneMarkCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_lane_mark_camera_list_)) {

        framework::MessageLaneMarkCameraList message(&ros_lane_mark_camera_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleLaneCurbCameraMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeLaneMarkCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_lane_curb_camera_list_)) {

        framework::MessageLaneCurbCameraList message(&ros_lane_curb_camera_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleMaxieyeCameraMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_maxieye_camera_list_)) {
        
        // 更改传感器类型  
        //printf("front:cam_type:%d\n", ros_obstacle_maxieye_camera_list_.cam_type);
        switch(ros_obstacle_maxieye_camera_list_.cam_type) {
          case (ad_msg::ObstacleCameraList::CAM_TYPE_MAXIEYE_D500):
            ros_obstacle_maxieye_camera_list_.cam_type = 1;
          break;

          default:
            ros_obstacle_maxieye_camera_list_.cam_type = -1;
          break;
        }
        //printf("rear:cam_type:%d\n", ros_obstacle_maxieye_camera_list_.cam_type);

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT, &ros_obstacle_maxieye_camera_list_);
        //printf("MaxieyeObstacleNum:%d, TimeStamp:%ld\n", ros_obstacle_maxieye_camera_list_.obstacle_num, ros_obstacle_maxieye_camera_list_.msg_head.timestamp);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleVisualControlFrontMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_visual_control_front_list_)) {

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT, &ros_obstacle_visual_control_front_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleVisualControlFrontLeftMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_visual_control_front_left_list_)) {

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT, &ros_obstacle_visual_control_front_left_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleVisualControlFrontRightMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_visual_control_front_right_list_)) {

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT, &ros_obstacle_visual_control_front_right_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleVisualControlRearLeftMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_visual_control_rear_left_list_)) {

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT, &ros_obstacle_visual_control_rear_left_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleVisualControlRearRightMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_visual_control_rear_right_list_)) {

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT, &ros_obstacle_visual_control_rear_right_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleVisualControlRearMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_visual_control_rear_list_)) {

        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR, &ros_obstacle_visual_control_rear_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleMpuStateMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeMpuStateMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &mpu_state_)) {

      framework::MessageMpuStateData msg_mpu_state(framework::MSG_ID_RECV_MPU_STATE_DATA, &mpu_state_);
      Notify(msg_mpu_state);//回放模式下不需要进行该操作，否则会陷入不必要的循环之中
    }
  }
}


void MsgReceiver::HandleObstacleArsRadarMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_ars_radar_list_)) {
        
        // 更改传感器类型  
        //printf("front:radar_type:%d\n", ros_obstacle_ars_radar_list_.radar_type);
        switch(ros_obstacle_ars_radar_list_.radar_type) {
          case (ad_msg::ObstacleRadarList::RADAR_TYPE_ARS430):
            ros_obstacle_ars_radar_list_.radar_type = 0;
          break;

          default:
            ros_obstacle_ars_radar_list_.radar_type = -1;
          break;
        }

        //printf("rear:radar_type:%d\n", ros_obstacle_ars_radar_list_.radar_type);
        framework::MessageRecvRadarData message(
                framework::MSG_ID_RECV_ESR_DATA_FRONT, &ros_obstacle_ars_radar_list_);
        
        //printf("ArsObstacleNum:%d, TimeStamp:%ld\n", ros_obstacle_ars_radar_list_.obstacle_num, ros_obstacle_ars_radar_list_.msg_head.timestamp);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleAnngicRadarFrontLeftMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_anngic_radar_front_left_list_)) {

        framework::MessageRecvRadarData message(
                framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT, &ros_obstacle_anngic_radar_front_left_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleAnngicRadarFrontRightMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_anngic_radar_front_right_list_)) {

        framework::MessageRecvRadarData message(
                framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT, &ros_obstacle_anngic_radar_front_right_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleAnngicRadarRearLeftMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_anngic_radar_rear_left_list_)) {

        framework::MessageRecvRadarData message(
                framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT, &ros_obstacle_anngic_radar_rear_left_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleAnngicRadarRearRightMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_anngic_radar_rear_right_list_)) {

        framework::MessageRecvRadarData message(
                framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT, &ros_obstacle_anngic_radar_rear_right_list_);
        Notify(message);
    }
  }
}

void MsgReceiver::HandleObstacleLidarFrontMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleLidarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_obstacle_lidar_front_list_)) {

      MessageRecvLidarObjects message(
          MSG_ID_RECV_LIDAR_OBJECTS_FRONT, &ros_obstacle_lidar_front_list_);
      Notify(message);
    }
  }
}
#endif  // #if (ENABLE_ROS_NODE)


}  // namespace communication
}  // namespace phoenix
