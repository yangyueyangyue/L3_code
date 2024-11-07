/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
#define PHOENIX_FRAMEWORK_MSG_RECEIVER_H_


#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include <std_msgs/ByteMultiArray.h>
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


class MsgReceiver : public Task {
public:
  MsgReceiver(Task* manager);

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

private:
#if (ENABLE_ROS_NODE)
  void HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg);

  void HandleLaneMarkCameraMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleLaneCurbCameraMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleMaxieyeCameraMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleVisualControlFrontMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleVisualControlFrontLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleVisualControlFrontRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleVisualControlRearLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleVisualControlRearRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleVisualControlRearMessageRos(const std_msgs::ByteMultiArray& msg);

  void HandleObstacleArsRadarMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleAnngicRadarFrontLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleAnngicRadarFrontRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleAnngicRadarRearLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleAnngicRadarRearRightMessageRos(const std_msgs::ByteMultiArray& msg);

  void HandleObstacleLidarFrontMessageRos(const std_msgs::ByteMultiArray& msg);

  void HandleMpuStateMessageRos(const std_msgs::ByteMultiArray& msg);

#endif

#if (ENABLE_LCM_NODE)
#endif

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

#if (ENABLE_ROS_NODE)
  ad_msg::Chassis ros_chassis_info_;
  MpuState::MonitorMpuState mpu_state_;

  ad_msg::LaneMarkCameraList ros_lane_mark_camera_list_;
  ad_msg::LaneMarkCameraList ros_lane_curb_camera_list_;
  ad_msg::ObstacleCameraList ros_obstacle_maxieye_camera_list_;
  ad_msg::ObstacleCameraList ros_obstacle_visual_control_front_list_;
  ad_msg::ObstacleCameraList ros_obstacle_visual_control_front_left_list_;
  ad_msg::ObstacleCameraList ros_obstacle_visual_control_front_right_list_;
  ad_msg::ObstacleCameraList ros_obstacle_visual_control_rear_left_list_;
  ad_msg::ObstacleCameraList ros_obstacle_visual_control_rear_right_list_;
  ad_msg::ObstacleCameraList ros_obstacle_visual_control_rear_list_;

  ad_msg::ObstacleRadarList ros_obstacle_ars_radar_list_;
  ad_msg::ObstacleRadarList ros_obstacle_anngic_radar_front_left_list_;
  ad_msg::ObstacleRadarList ros_obstacle_anngic_radar_front_right_list_;
  ad_msg::ObstacleRadarList ros_obstacle_anngic_radar_rear_left_list_;
  ad_msg::ObstacleRadarList ros_obstacle_anngic_radar_rear_right_list_;

  ad_msg::ObstacleLidarList ros_obstacle_lidar_front_list_;
#endif
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
