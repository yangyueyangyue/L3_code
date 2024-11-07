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
#include "driving_map.h"
#include "motion_planning.h"


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

  ad_msg::Gnss gnss_info_;
  ad_msg::Imu imu_info_;
  ad_msg::Chassis chassis_info_;
  ad_msg::SpecialChassisInfo special_chassis_info_;
  ad_msg::ChassisCtlCmd chassis_ctl_cmd_;

  ad_msg::LaneMarkCameraList lane_mark_list_;
  ad_msg::ObstacleCameraList obstacle_camera_list_;
  ad_msg::ObstacleRadarList obstacle_radar_front_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_list_;
  ad_msg::ObstacleRadarList obstacle_radar_front_left_list_;
  ad_msg::ObstacleRadarList obstacle_radar_front_right_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_left_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_right_list_;
  ad_msg::ObstacleRadarList srr2_detections_front_left_list_;
  ad_msg::ObstacleRadarList srr2_detections_front_right_list_;
  ad_msg::ObstacleRadarList srr2_detections_rear_left_list_;
  ad_msg::ObstacleRadarList srr2_detections_rear_right_list_;
  ad_msg::ObstacleLidarList obstacle_lidar_list_0_;
  ad_msg::ObstacleLidarList obstacle_lidar_list_1_;
  ad_msg::ObstacleList obstacle_list_;
  ad_msg::TrafficSignalList traffic_signal_list_;
  ad_msg::TrafficLightList traffic_light_list_;

  driv_map::DrivingMapInfo driving_map_info_;
  planning::VelocityPlanningResult velocity_planning_result_;
  planning::TrajectoryPlanningResult trajectory_planning_result_;
  planning::ActionPlanningResult action_planning_result_;
  ad_msg::RelativePosList relative_pos_list_;
  ad_msg::ObstacleTrackedInfoList obstacle_tracked_info_list_;
  ad_msg::LaneInfoCameraList lane_info_camera_list_;
  planning::TrajectoryPlanningInfo trajectory_planning_info_;
  ad_msg::PlanningResult planning_result_;
  ad_msg::LateralControlInfo lateral_control_info_;

  ad_msg::PlanningSettings curr_planning_settings_;
  ad_msg::ModuleStatusList module_status_list_;
  ad_msg::ModuleStatusList control_module_status_list_;
  ad_msg::EventReportingList event_report_list_;

  Char_t serialization_data_buff_[10*1024*1024];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_HMI_MSG_SENDER_H_

