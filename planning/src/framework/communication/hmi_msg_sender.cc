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
#include "communication/hmi_msg_sender.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#endif

#include "utils/com_utils.h"
#include "math/math_utils.h"

#include "data_serialization.h"
#include "communication/shared_data.h"


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
HmiMsgSender::HmiMsgSender(Task* manager) :
  Task(TASK_ID_HMI_MSG_SENDER, "Message Sender", manager) {
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
bool HmiMsgSender::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    // empty
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


void HmiMsgSender::SendHmiMsg() {
  SharedData *shared_data = SharedData::instance();

  Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;
  Int32_t data_size = 0;
  Int32_t data_len = 0;

#if (ENABLE_UDP_NODE)
  if (Nullptr_t == udp_node_) {
    return;
  }

  /* D001 fuyuanyi 2023-04-26 (begin) */
  /* 修改WBT index12 分支覆盖率未达标问题
     检查序列化消息大小小于分配buff大小
     以下批量修改*/
  /* D001 fuyuanyi 2023-04-26 (end) */

  // Send current planning settings
  data_size = sizeof(ad_msg::PlanningSettings);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetCurrentPlanningSettings(&curr_planning_settings_);
  data_len = data_serial::EncodePlanningSettingsArray(
        serialization_data_buff_, 0, max_buff_size, &curr_planning_settings_, 1);
  udp_node_->Publish("hmi/planning/current_settings",
                     serialization_data_buff_, data_len);

  // Send planning module status
  data_size = sizeof(ad_msg::ModuleStatusList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetModuleStatusList(&module_status_list_);
  data_len = data_serial::EncodeModuleStatusListArray(
        serialization_data_buff_, 0, max_buff_size, &module_status_list_, 1);
  udp_node_->Publish("hmi/planning/module_status_list",
                     serialization_data_buff_, data_len);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  // send control module status
  data_size = sizeof(ad_msg::ModuleStatusList);
  if (data_size > max_buff_size) {
    LOG_ERR << "The size of serialization buffer is not enough.";
  } else {
    shared_data->GetControlModuleStatusList(&control_module_status_list_);
    data_len = data_serial::EncodeModuleStatusListArray(
          serialization_data_buff_, 0,
          max_buff_size, &control_module_status_list_, 1);
    udp_node_->Publish("hmi/control/module_status_list",
                       serialization_data_buff_, data_len);
  }
#endif

  // Send event report
  data_size = sizeof(ad_msg::EventReportingList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetEventReportingList(&event_report_list_);
  data_len = data_serial::EncodeEventReportingListArray(
        serialization_data_buff_, 0, max_buff_size, &event_report_list_, 1);
  udp_node_->Publish("hmi/planning/event_report",
                     serialization_data_buff_, data_len);

  // send localizaiton gnss
  data_size = sizeof(ad_msg::Gnss);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetGnss(&gnss_info_);
  data_len = data_serial::EncodeGnssArray(
        serialization_data_buff_, 0, max_buff_size, &gnss_info_, 1);
  udp_node_->Publish("hmi/localization/gnss",
                     serialization_data_buff_, data_len);

  // send localizaiton imu
  data_size = sizeof(ad_msg::Imu);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetImu(&imu_info_);
  data_len = data_serial::EncodeImuArray(
        serialization_data_buff_, 0, max_buff_size, &imu_info_, 1);
  udp_node_->Publish("hmi/localization/imu",
                     serialization_data_buff_, data_len);

  // send Chassis message
  data_size = sizeof(ad_msg::Chassis);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetChassis(&chassis_info_);
  data_len = data_serial::EncodeChassisArray(
        serialization_data_buff_, 0, max_buff_size, &chassis_info_, 1);
  udp_node_->Publish("hmi/control/chassis",
                     serialization_data_buff_, data_len);

  // send Special Chassis message
  data_size = sizeof(ad_msg::SpecialChassisInfo);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetSpecialChassisInfo(&special_chassis_info_);
  data_len = data_serial::EncodeSpecialChassisInfoArray(
        serialization_data_buff_, 0, max_buff_size,
        &special_chassis_info_, 1);
  udp_node_->Publish("hmi/control/special_chassis_info",
                     serialization_data_buff_, data_len);

  // send ChassisCtlCmd message
  data_size = sizeof(ad_msg::ChassisCtlCmd);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetChassisCtlCmd(&chassis_ctl_cmd_);
  data_len = data_serial::EncodeChassisCtlCmdArray(
        serialization_data_buff_, 0, max_buff_size, &chassis_ctl_cmd_, 1);
  udp_node_->Publish("hmi/control/chassis_ctl_cmd",
                     serialization_data_buff_, data_len);

  // send Lane Mark CameraList
  data_size = sizeof(ad_msg::LaneMarkCameraList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetLaneMarkCameraList(&lane_mark_list_);
  data_len = data_serial::EncodeLaneMarkCameraListArray(
        serialization_data_buff_, 0, max_buff_size, &lane_mark_list_, 1);
  udp_node_->Publish("hmi/perception/lane_mark_camera",
                     serialization_data_buff_, data_len);

  //send Obstacle Camera List
  data_size = sizeof(ad_msg::ObstacleCameraList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleCameraList(&obstacle_camera_list_);
  data_len = data_serial::EncodeObstacleCameraListArray(
        serialization_data_buff_, 0, max_buff_size,
        &obstacle_camera_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_camera",
                     serialization_data_buff_, data_len);

#if 0
  //send Obstacle RadarList
  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleRadarFrontList(&obstacle_radar_front_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_radar_front_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_0",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleRadarRearList(&obstacle_radar_rear_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_radar_rear_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_9",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleRadarFrontLeftList(&obstacle_radar_front_left_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_radar_front_left_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_1",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleRadarFrontRightList(&obstacle_radar_front_right_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_radar_front_right_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_2",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleRadarRearLeftList(&obstacle_radar_rear_left_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_radar_rear_left_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_3",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleRadarRearRightList(&obstacle_radar_rear_right_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_radar_rear_right_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_4",
                     serialization_data_buff_, data_len);
#endif

#if 0
  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetSrr2DetectionsFrontLeftList(&srr2_detections_front_left_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &srr2_detections_front_left_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_5",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetSrr2DetectionsFrontRightList(&srr2_detections_front_right_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &srr2_detections_front_right_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_6",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetSrr2DetectionsRearLeftList(&srr2_detections_rear_left_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &srr2_detections_rear_left_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_7",
                     serialization_data_buff_, data_len);

  data_size = sizeof(ad_msg::ObstacleRadarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetSrr2DetectionsRearRightList(&srr2_detections_rear_right_list_);
  data_len = data_serial::EncodeObstacleRadarListArray(
        serialization_data_buff_, 0, max_buff_size, &srr2_detections_rear_right_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_radar_8",
                     serialization_data_buff_, data_len);
#endif

  // Obstacle Lidar list (0)
  data_size = sizeof(ad_msg::ObstacleLidarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleLidarList0(&obstacle_lidar_list_0_);
  data_len = data_serial::EncodeObstacleLidarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_lidar_list_0_, 1);
  udp_node_->Publish("hmi/perception/obstacle_lidar_0",
                     serialization_data_buff_, data_len);

  // Obstacle Lidar list (1)
  data_size = sizeof(ad_msg::ObstacleLidarList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleLidarList1(&obstacle_lidar_list_1_);
  data_len = data_serial::EncodeObstacleLidarListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_lidar_list_1_, 1);
  udp_node_->Publish("hmi/perception/obstacle_lidar_1",
                     serialization_data_buff_, data_len);

  // Send ObstacleList  融合后的障碍物
  data_size = sizeof(ad_msg::ObstacleList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleList(&obstacle_list_);
  data_len = data_serial::EncodeObstacleListArray(
        serialization_data_buff_, 0, max_buff_size, &obstacle_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle",
                     serialization_data_buff_, data_len);

  // Send Driving map
  data_size = sizeof(driv_map::DrivingMapInfo);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetDrivingMapInfo(&driving_map_info_);
  data_len = data_serial::EncodeDrivingMapInfoArray(
        serialization_data_buff_, 0, max_buff_size, &driving_map_info_, 1);
  udp_node_->Publish("hmi/map/driving_map",
                     serialization_data_buff_, data_len);

  // Send Velocity Planning Result
  data_size = sizeof(planning::VelocityPlanningResult);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetVelocityPlanningResult(&velocity_planning_result_);
  data_len = data_serial::EncodeVelocityPlanningResultArray(
        serialization_data_buff_, 0, max_buff_size,
        &velocity_planning_result_, 1);
  udp_node_->Publish("hmi/planning/velocity_planning_result",
                     serialization_data_buff_, data_len);

  // Send trajectory planning result
  data_size = sizeof(planning::TrajectoryPlanningResult);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetTrajectoryPlanningResult(&trajectory_planning_result_);
  data_len = data_serial::EncodeTrajectoryPlanningResultArray(
        serialization_data_buff_, 0, max_buff_size,
        &trajectory_planning_result_, 1);
  udp_node_->Publish("hmi/planning/trajectory_planning_result",
                     serialization_data_buff_, data_len);

  // Send action planning result
  data_size = sizeof(planning::ActionPlanningResult);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetActionPlanningResult(&action_planning_result_);
  data_len = data_serial::EncodeActionPlanningResultArray(
        serialization_data_buff_, 0, max_buff_size,
        &action_planning_result_, 1);
  udp_node_->Publish("hmi/planning/action_planning_reslut",
                     serialization_data_buff_, data_len);

  // Send Relative Pos List
  data_size = sizeof(ad_msg::RelativePosList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetRelativePosList(&relative_pos_list_);
  data_len = data_serial::EncodeRelativePosListArray(
        serialization_data_buff_, 0, max_buff_size, &relative_pos_list_, 1);
  udp_node_->Publish("hmi/localization/relative_pos_list",
                     serialization_data_buff_, data_len);

#if 0
  // Send Obstacle TrackedInfo List
  data_size = sizeof(ad_msg::ObstacleTrackedInfoList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetObstacleTrackedInfoList(&obstacle_tracked_info_list_);
  data_len = data_serial::EncodeObstacleTrackedInfoListArray(
        serialization_data_buff_, 0, max_buff_size,
        &obstacle_tracked_info_list_, 1);
  udp_node_->Publish("hmi/perception/obstacle_tracked_info_list",
                     serialization_data_buff_, data_len);

#endif

#if 0
  // Send LaneInfo Camera List
  data_size = sizeof(ad_msg::LaneInfoCameraList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetLaneInfoCameraList(&lane_info_camera_list_);
  data_len = data_serial::EncodeLaneInfoCameraListArray(
        serialization_data_buff_, 0, max_buff_size,
        &lane_info_camera_list_, 1);
  udp_node_->Publish("hmi/perception/lane_info_camera_list",
                     serialization_data_buff_, data_len);

#endif

  // Send Trajectory PlanningInfo
  data_size = sizeof(planning::TrajectoryPlanningInfo);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetTrajectoryPlanningInfo(&trajectory_planning_info_);
  data_len = data_serial::EncodeTrajectoryPlanningInfoArray(
        serialization_data_buff_, 0, max_buff_size,
        &trajectory_planning_info_, 1);
  udp_node_->Publish("hmi/planning/trajectory_planning_info",
                     serialization_data_buff_, data_len);

  // Send planning result
  data_size = sizeof(ad_msg::PlanningResult);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetPlanningResult(&planning_result_);
  data_len = data_serial::EncodePlanningResultArray(
        serialization_data_buff_, 0, max_buff_size, &planning_result_, 1);
  udp_node_->Publish("hmi/planning/planning_result",
                     serialization_data_buff_, data_len);

  // 交通信号标志
  data_size = sizeof(ad_msg::TrafficSignalList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetTrafficSignalList(&traffic_signal_list_);
  data_len = data_serial::EncodeTrafficSignalListArray(
        serialization_data_buff_, 0, max_buff_size, &traffic_signal_list_, 1);
  udp_node_->Publish("hmi/perception/traffic_signal",
                     serialization_data_buff_, data_len);

  // 红绿灯列表
  data_size = sizeof(ad_msg::TrafficLightList);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetTrafficLightList(&traffic_light_list_);
  data_len = data_serial::EncodeTrafficLightListArray(
        serialization_data_buff_, 0, max_buff_size, &traffic_light_list_, 1);
  udp_node_->Publish("hmi/perception/traffic_light",
                     serialization_data_buff_, data_len);

  // 横向控制调试信息
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  data_size = sizeof(ad_msg::LateralControlInfo);
  COM_CHECK(data_size < max_buff_size);
  shared_data->GetLateralControlInfo(&lateral_control_info_);
  data_len = data_serial::EncodeLateralControlInfoArray(
        serialization_data_buff_, 0, max_buff_size, &lateral_control_info_, 1);
  udp_node_->Publish("hmi/control/lat_ctl_info",
                     serialization_data_buff_, data_len);
#endif

  // Send log
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  Int32_t log_size = common::GetLogFromRingBuff(
        8*1024, serialization_data_buff_);
  // std::cout << "log_size=" << log_size << std::endl;
  if (log_size > 0) {
    udp_node_->Publish("hmi/planning/log", serialization_data_buff_, log_size);
  }
#endif

#endif  // #if (ENABLE_UDP_NODE)
}


}  // namespace framework
}  // namespace phoenix

