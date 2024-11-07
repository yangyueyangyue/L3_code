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
#include "planning/planning_setting.h"
#include "motion_planning.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/task.h"
#include "common/message.h"

#if (ENTER_PLAYBACK_MODE_ADASISV2)
#include "pcc_map/adasisv2_msg_parser.h"
#include "planning/CanFrame.h"
#include "planning/adasisv2_position.h"
#include "planning/adasisv2_profileshort.h"
#include "pcc_map/adasisv2_msg_parser.h"
#endif

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

  void Configurate(const PlanningConfig& conf);

  bool Start();

#if (ENABLE_UDP_NODE)
  void HandleHdMapMessageUdp(const void *buf, Int32_t buf_len);
  void HandleRoutingMessageUdp(const void *buf, Int32_t buf_len);
  void HandleGnssMessageUdp(const void *buf, Int32_t buf_len);
  void HandleImuMessageUdp(const void *buf, Int32_t buf_len);
  void HandleChassisMessageUdp(const void *buf, Int32_t buf_len);
  void HandleSpecialChassisInfoMessageUdp(const void *buf, Int32_t buf_len);
  void HandleChassisCtlCmdMessageUdp(const void *buf, Int32_t buf_len);
  void HandleLaneMarkCameraListMessageUdp(const void *buf, Int32_t buf_len);
  void HandleObstacleCameraListMessageUdp(const void *buf, Int32_t buf_len);
  void HandleObstacleRadarFrontMessageUdp(const void *buf, Int32_t buf_len);
  void HandleObstacleRadarFrontLeftMessageUdp(const void *buf, Int32_t buf_len);
  void HandleObstacleRadarFrontRightMessageUdp(const void *buf, Int32_t buf_len);
  void HandleObstacleRadarRearLeftMessageUdp(const void *buf, Int32_t buf_len);
  void HandleObstacleRadarRearRightMessageUdp(const void *buf, Int32_t buf_len);
  void HandleTrafficSignalListMessageUdp(const void *buf, Int32_t buf_len);

  void HandleRemotePlanningSettingsMsgUdp(const void *buf, Int32_t buf_len);
#endif

private:
#if (ENABLE_ROS_NODE)
  void HandleHdMapMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleRoutingMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleGnssMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleImuMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleSpecialChassisInfoMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleChassisCtlCmdMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleLaneMarkCameraListMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleCameraListMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleRadarFrontMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleRadarRearMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleRadarFrontLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleRadarFrontRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleRadarRearLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleRadarRearRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleSrr2DetectionsFrontLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleSrr2DetectionsFrontRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleSrr2DetectionsRearLeftMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleSrr2DetectionsRearRightMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleLidar0MessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleLidar1MessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleObstacleListMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleTrafficSignalListMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleTrafficLightListMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleScenceStorysMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleMapLocalizationMessageRos(const std_msgs::ByteMultiArray& msg);
  void HandleRemotePlanningSettingsMsgRos(const ::planning::planning_setting& msg);
  // adecu 回放
  void HandleADECUPlanningDebugMsgRos(const std_msgs::ByteMultiArray& msg);
#if (ENTER_PLAYBACK_MODE_ADASISV2)
  void HandleADASISV2RawCanRos(const ::planning::CanFrame& msg);
#endif

#endif

#if (ENABLE_LCM_NODE)
  void HandleHdMapMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleRoutingMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleGnssMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleImuMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleChassisMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleSpecialChassisInfoMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleChassisCtlCmdMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleLaneMarkCameraListMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleObstacleCameraListMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleObstacleRadarFrontMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleObstacleRadarFrontLeftMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleObstacleRadarFrontRightMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleObstacleRadarRearLeftMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleObstacleRadarRearRightMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void HandleTrafficSignalListMessageLcm(
      const lcm::ReceiveBuffer* rbuf, const std::string& channel);
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

  // Config
  PlanningConfig config_;

#if (ENABLE_ROS_NODE)
  ad_msg::Gnss ros_gnss_info_;
  ad_msg::Imu ros_imu_info_;
  ad_msg::Chassis ros_chassis_info_;
  ad_msg::SpecialChassisInfo ros_special_chassis_info_;
  ad_msg::ChassisCtlCmd ros_chassis_ctl_cmd_info_;

  ad_msg::LaneMarkCameraList ros_lane_mark_camera_list_;
  ad_msg::ObstacleCameraList ros_obstacle_camera_list_;
  ad_msg::ObstacleRadarList ros_obstacle_radar_front_list_;
  ad_msg::ObstacleRadarList ros_obstacle_radar_rear_list_;
  ad_msg::ObstacleRadarList ros_obstacle_radar_front_left_list_;
  ad_msg::ObstacleRadarList ros_obstacle_radar_front_right_list_;
  ad_msg::ObstacleRadarList ros_obstacle_radar_rear_left_list_;
  ad_msg::ObstacleRadarList ros_obstacle_radar_rear_right_list_;

  ad_msg::ObstacleRadarList ros_srr2_detections_front_left_list_;
  ad_msg::ObstacleRadarList ros_srr2_detections_front_right_list_;
  ad_msg::ObstacleRadarList ros_srr2_detections_rear_left_list_;
  ad_msg::ObstacleRadarList ros_srr2_detections_rear_right_list_;

  ad_msg::ObstacleLidarList ros_obstacle_lidar_list_0_;
  ad_msg::ObstacleLidarList ros_obstacle_lidar_list_1_;

  ad_msg::ObstacleList ros_obstacle_list_;
  ad_msg::TrafficSignalList ros_traffic_signal_list_;
  ad_msg::TrafficLightList ros_traffic_light_;

  ad_msg::PlanningSettings ros_planning_settings_;
  ad_msg::SceneStoryList ros_scene_story_list_;
  ad_msg::MapLocalization ros_map_localization_;
  // ADECU PLANNING DEBUG
  planning::VelocityPlanningResult ADECU_vel_planning_debug_;
#endif

#if (ENABLE_LCM_NODE)
  ad_msg::Gnss lcm_gnss_info_;
  ad_msg::Imu lcm_imu_info_;
  ad_msg::Chassis lcm_chassis_info_;
  ad_msg::SpecialChassisInfo lcm_special_chassis_info_;
  ad_msg::ChassisCtlCmd lcm_chassis_ctl_cmd_info_;

  ad_msg::LaneMarkCameraList lcm_lane_mark_camera_list_;
  ad_msg::ObstacleCameraList lcm_obstacle_camera_list_;
  ad_msg::ObstacleRadarList lcm_obstacle_radar_front_list_;
  ad_msg::ObstacleRadarList lcm_obstacle_radar_front_left_list_;
  ad_msg::ObstacleRadarList lcm_obstacle_radar_front_right_list_;
  ad_msg::ObstacleRadarList lcm_obstacle_radar_rear_left_list_;
  ad_msg::ObstacleRadarList lcm_obstacle_radar_rear_right_list_;
  ad_msg::TrafficSignalList lcm_traffic_signal_list_;

  ad_msg::PlanningSettings lcm_planning_settings_;
#endif

#if (ENABLE_UDP_NODE)
  ad_msg::Gnss udp_gnss_info_;
  ad_msg::Imu udp_imu_info_;
  ad_msg::Chassis udp_chassis_info_;
  ad_msg::SpecialChassisInfo udp_special_chassis_info_;
  ad_msg::ChassisCtlCmd udp_chassis_ctl_cmd_info_;

  ad_msg::LaneMarkCameraList udp_lane_mark_camera_list_;
  ad_msg::ObstacleCameraList udp_obstacle_camera_list_;
  ad_msg::ObstacleRadarList udp_obstacle_radar_front_list_;
  ad_msg::ObstacleRadarList udp_obstacle_radar_front_left_list_;
  ad_msg::ObstacleRadarList udp_obstacle_radar_front_right_list_;
  ad_msg::ObstacleRadarList udp_obstacle_radar_rear_left_list_;
  ad_msg::ObstacleRadarList udp_obstacle_radar_rear_right_list_;
  ad_msg::TrafficSignalList udp_traffic_signal_list_;

  ad_msg::PlanningSettings udp_planning_settings_;
#endif
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_RECEIVER_H_
