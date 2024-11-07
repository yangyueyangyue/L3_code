/******************************************************************************
 ** 共享数据存储
 ******************************************************************************
 *
 *  共享数据存储
 *
 *  @file       shared_data.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_SHARED_DATA_H_
#define PHOENIX_FRAMEWORK_SHARED_DATA_H_


#include "utils/macros.h"
#include "container/doubly_linked_list.h"
#include "container/ring_buffer.h"
#include "os/mutex.h"
#include "msg_event_reporting.h"
#include "pos_filter.h"
#include "driving_map.h"
#include "motion_planning.h"
#include "common/module_status.h"

#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
#include "boost/thread.hpp"
#include "modules/map/proto/map.pb.h"
#include "modules/routing/proto/routing.pb.h"
#endif

#include "pcc_map/adasis_v2.h"

namespace phoenix {
namespace framework {


class SharedData {
public:
  ~SharedData();

  // Module status
  void SetModuleStatusList(const ad_msg::ModuleStatusList& data) {
    common::os::WriteLockHelper lock(lock_module_status_list_);

    module_status_list_ = data;
  }
  void GetModuleStatusList(ad_msg::ModuleStatusList* data) {
    common::os::ReadLockHelper lock(lock_module_status_list_);

    *data = module_status_list_;
  }
  // Control Module status
  void SetControlModuleStatusList(const ad_msg::ModuleStatusList& data) {
    common::os::WriteLockHelper lock(lock_control_module_status_list_);

    control_module_status_list_ = data;
  }
  void GetControlModuleStatusList(ad_msg::ModuleStatusList* data) {
    common::os::ReadLockHelper lock(lock_control_module_status_list_);

    *data = control_module_status_list_;
  }
  void ClearControlModuleStatusList() {
    common::os::ReadLockHelper lock(lock_control_module_status_list_);

    control_module_status_list_.Clear();
  }

  // event reporting
  void AddEventReporting(
      Int32_t num, const ad_msg::EventReporting* const events) {
    common::os::WriteLockHelper lock(lock_event_reporting_buffer_);

    for (Int32_t i = 0; i < num; ++i) {
      event_reporting_buffer_.PushBackOverride(events[i]);
    }
  }
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) {
    // Must use "write lock"
    common::os::WriteLockHelper lock(lock_event_reporting_buffer_);

    Int32_t idx = 0;
    for (idx = 0; idx < num; ++idx) {
      ad_msg::EventReporting* data =
          event_reporting_buffer_.Front();
      if (Nullptr_t == data) {
        break;
      }
      events[idx] = *data;
      event_reporting_buffer_.PopFront();
    }

    return (idx);
  }
  void SetEventReportingList(const ad_msg::EventReportingList& data) {
    common::os::WriteLockHelper lock(lock_event_reporting_list_);

    event_reporting_list_ = data;
  }
  void GetEventReportingList(ad_msg::EventReportingList* data) {
    common::os::ReadLockHelper lock(lock_event_reporting_list_);

    *data = event_reporting_list_;
  }


  // Current planning setting
  void SetCurrentPlanningSettings(const ad_msg::PlanningSettings& data){
    common::os::WriteLockHelper lock(lock_curr_planning_settings_);
    curr_planning_settings_ = data;
  }
  void GetCurrentPlanningSettings(ad_msg::PlanningSettings* data){
    common::os::ReadLockHelper lock(lock_curr_planning_settings_);
    *data = curr_planning_settings_;
  }
  // Local planning setting
  void SetLocalPlanningSettings(const ad_msg::PlanningSettings& data){
    common::os::WriteLockHelper lock(lock_local_planning_settings_);
    local_planning_settings_ = data;
  }
  void GetLocalPlanningSettings(ad_msg::PlanningSettings* data){
    common::os::ReadLockHelper lock(lock_local_planning_settings_);
    *data = local_planning_settings_;
  }
  // Remote planning setting
  void SetRemotePlanningSettings(const ad_msg::PlanningSettings& data){
    common::os::WriteLockHelper lock(lock_remote_planning_settings_);
    remote_planning_settings_ = data;
  }
  void GetRemotePlanningSettings(ad_msg::PlanningSettings* data){
    common::os::ReadLockHelper lock(lock_remote_planning_settings_);
    *data = remote_planning_settings_;
  }


  // Map
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  void SetRawHDMap(const std::shared_ptr<const apollo::hdmap::Map>& data) {
    common::os::WriteLockHelper lock(lock_raw_hd_map_);

    raw_hd_map_ = data;
  }
  void GetRawHDMap(std::shared_ptr<const apollo::hdmap::Map>* data) {
    common::os::ReadLockHelper lock(lock_raw_hd_map_);

    *data = raw_hd_map_;
  }

  void SetRawRouting(const std::shared_ptr<const apollo::routing::RoutingResponse>& data) {
    common::os::WriteLockHelper lock(lock_raw_routing_);

    raw_routing_ = data;
  }
  void GetRawRouting(std::shared_ptr<const apollo::routing::RoutingResponse>* data) {
    common::os::ReadLockHelper lock(lock_raw_routing_);

    *data = raw_routing_;
  }
#endif


  // Gnss
  void SetGnss(const ad_msg::Gnss& data) {
    common::os::WriteLockHelper lock(lock_gnss_);

    gnss_ = data;
  }
  void GetGnss(ad_msg::Gnss* data) {
    common::os::ReadLockHelper lock(lock_gnss_);

    *data = gnss_;
  }
  void ClearGnss() {
    common::os::WriteLockHelper lock(lock_gnss_);

    gnss_.Clear();
  }

  // IMU
  void SetImu(const ad_msg::Imu& data) {
    common::os::WriteLockHelper lock(lock_imu_);

    imu_ = data;
  }
  void GetImu(ad_msg::Imu* data) {
    common::os::ReadLockHelper lock(lock_imu_);

    *data = imu_;
  }
  void ClearImu() {
    common::os::WriteLockHelper lock(lock_imu_);

    imu_.Clear();
  }

  // Chassis information
  void SetChassis(const ad_msg::Chassis& data) {
    common::os::WriteLockHelper lock(lock_chassis_);

    Float32_t prev_yaw_rate = chassis_.yaw_rate;
    Float32_t delta_yaw_rate = data.yaw_rate - prev_yaw_rate;

    chassis_ = data;

    if ((common::com_abs(delta_yaw_rate) > common::com_deg2rad(100.0F)) ||
        (common::com_abs(data.yaw_rate) > common::com_deg2rad(100.0F))) {
      LOG_ERR << "Detected unexpected value of yaw_rate("
              << common::com_rad2deg(data.yaw_rate)
              << " deg/s), prev=" << common::com_rad2deg(prev_yaw_rate)
              << " deg/s.";
      chassis_.yaw_rate = prev_yaw_rate;
    }
  }
  void GetChassis(ad_msg::Chassis* data) {
    common::os::ReadLockHelper lock(lock_chassis_);

    *data = chassis_;
  }

  // Special Chassis information
  void ClearSpecialChassisInfo() {
    common::os::WriteLockHelper lock(lock_special_chassis_info_);

    special_chassis_info_.Clear();
  }
  void SetSpecialChassisInfo(const ad_msg::SpecialChassisInfo& data) {
    common::os::WriteLockHelper lock(lock_special_chassis_info_);

    special_chassis_info_ = data;
  }
  void GetSpecialChassisInfo(ad_msg::SpecialChassisInfo* data) {
    common::os::ReadLockHelper lock(lock_special_chassis_info_);

    *data = special_chassis_info_;
  }

  // ChassisCtlCmd information
  void SetChassisCtlCmd(const ad_msg::ChassisCtlCmd& data) {
    common::os::WriteLockHelper lock(lock_chassis_ctl_cmd_);

    chassis_ctl_cmd_ = data;
  }
  void GetChassisCtlCmd(ad_msg::ChassisCtlCmd* data) {
    common::os::ReadLockHelper lock(lock_chassis_ctl_cmd_);

    *data = chassis_ctl_cmd_;
  }

  // Camera lane mark information
  void SetLaneMarkCameraList(const ad_msg::LaneMarkCameraList& data) {
    common::os::WriteLockHelper lock(lock_lane_mark_camera_list_);

    lane_mark_camera_list_ = data;
  }
  void GetLaneMarkCameraList(ad_msg::LaneMarkCameraList* data) {
    common::os::ReadLockHelper lock(lock_lane_mark_camera_list_);

    *data = lane_mark_camera_list_;
  }

  // camera lane line information
  void SetLaneInfoCameraList(const ad_msg::LaneInfoCameraList& data) {
    common::os::WriteLockHelper lock(lock_lane_info_camera_list_);

    lane_info_camera_list_ = data;
  }
  void GetLaneInfoCameraList(ad_msg::LaneInfoCameraList* data) {
    common::os::ReadLockHelper lock(lock_lane_info_camera_list_);

    *data = lane_info_camera_list_;
  }

  // obstacles
  void SetObstacleList(const ad_msg::ObstacleList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_list_);

    obstacle_list_ = data;
  }
  void GetObstacleList(ad_msg::ObstacleList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_list_);

    *data = obstacle_list_;
  }

  // Outside obstacle list
  void SetOutsideObstacleList(const ad_msg::ObstacleList& data) {
    common::os::WriteLockHelper lock(lock_outside_obstacle_list_);

    outside_obstacle_list_ = data;
  }
  void GetOutsideObstacleList(ad_msg::ObstacleList* data) {
    common::os::ReadLockHelper lock(lock_outside_obstacle_list_);

    *data = outside_obstacle_list_;
  }
  void ClearOutsideObstacleList() {
    common::os::WriteLockHelper lock(lock_outside_obstacle_list_);
    outside_obstacle_list_.Clear();
  }

  // camera obstacles
  void SetObstacleCameraList(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_);

    obstacle_camera_list_ = data;
  }
  void GetObstacleCameraList(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_);

    *data = obstacle_camera_list_;
  }
  void ClearObstacleCameraList() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_);
    obstacle_camera_list_.Clear();
  }

  // Vision Controller Obstacles (front)
  void SetObstacleCameraList_Front(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_front_);

    obstacle_camera_list_front_ = data;
  }
  void GetObstacleCameraList_Front(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_front_);

    *data = obstacle_camera_list_front_;
  }
  void ClearObstacleCameraList_Front() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_front_);
    obstacle_camera_list_front_.Clear();
  }

  // Vision Controller Obstacles (left front)
  void SetObstacleCameraList_Left_Front(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_left_front_);

    obstacle_camera_list_left_front_ = data;
  }
  void GetObstacleCameraList_Left_Front(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_left_front_);

    *data = obstacle_camera_list_left_front_;
  }
  void ClearObstacleCameraList_Left_Front() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_left_front_);
    obstacle_camera_list_left_front_.Clear();
  }

  // Vision Controller Obstacles (right front)
  void SetObstacleCameraList_Right_Front(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_right_front_);

    obstacle_camera_list_right_front_ = data;
  }
  void GetObstacleCameraList_Right_Front(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_right_front_);

    *data = obstacle_camera_list_right_front_;
  }
  void ClearObstacleCameraList_Right_Front() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_right_front_);
    obstacle_camera_list_right_front_.Clear();
  }

  // Vision Controller Obstacles (rear)
  void SetObstacleCameraList_Rear(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_rear_);

    obstacle_camera_list_rear_ = data;
  }
  void GetObstacleCameraList_Rear(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_rear_);

    *data = obstacle_camera_list_rear_;
  }
  void ClearObstacleCameraList_Rear() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_rear_);
    obstacle_camera_list_rear_.Clear();
  }

  // Vision Controller Obstacles (left rear)
  void SetObstacleCameraList_Left_Rear(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_left_rear_);

    obstacle_camera_list_left_rear_ = data;
  }
  void GetObstacleCameraList_Left_Rear(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_left_rear_);

    *data = obstacle_camera_list_left_rear_;
  }
  void ClearObstacleCameraList_Left_Rear() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_left_rear_);
    obstacle_camera_list_left_rear_.Clear();
  }

  // Vision Controller Obstacles (right rear)
  void SetObstacleCameraList_Right_Rear(const ad_msg::ObstacleCameraList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_right_rear_);

    obstacle_camera_list_right_rear_ = data;
  }
  void GetObstacleCameraList_Right_Rear(ad_msg::ObstacleCameraList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_right_rear_);

    *data = obstacle_camera_list_right_rear_;
  }
  void ClearObstacleCameraList_Right_Rear() {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_right_rear_);
    obstacle_camera_list_right_rear_.Clear();
  }

  // ESR obstacles (front)
  void SetObstacleRadarFrontList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_front_list_);

    obstacle_radar_front_list_ = data;
  }
  void GetObstacleRadarFrontList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_radar_front_list_);

    *data = obstacle_radar_front_list_;
  }
  void ClearObstacleRadarFrontList() {
    common::os::WriteLockHelper lock(lock_obstacle_radar_front_list_);
    obstacle_radar_front_list_.Clear();
  }

  // ESR obstacles (rear)
  void SetObstacleRadarRearList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_rear_list_);

    obstacle_radar_rear_list_ = data;
  }
  void GetObstacleRadarRearList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_radar_rear_list_);

    *data = obstacle_radar_rear_list_;
  }
  void ClearObstacleRadarRearList() {
    common::os::WriteLockHelper lock(lock_obstacle_radar_rear_list_);
    obstacle_radar_rear_list_.Clear();
  }

  // Srr2 obstacles
  void SetObstacleRadarFrontLeftList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_front_left_list_);

    obstacle_radar_front_left_list_ = data;
  }
  void GetObstacleRadarFrontLeftList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_radar_front_left_list_);

    *data = obstacle_radar_front_left_list_;
  }
  void ClearObstacleRadarFrontLeftList() {
    common::os::WriteLockHelper lock(lock_obstacle_radar_front_left_list_);
    obstacle_radar_front_left_list_.Clear();
  }

  void SetObstacleRadarFrontRightList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_front_right_list_);

    obstacle_radar_front_right_list_ = data;
  }
  void GetObstacleRadarFrontRightList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_radar_front_right_list_);

    *data = obstacle_radar_front_right_list_;
  }
  void ClearObstacleRadarFrontRightList() {
    common::os::WriteLockHelper lock(lock_obstacle_radar_front_right_list_);
    obstacle_radar_front_right_list_.Clear();
  }

  void SetObstacleRadarRearLeftList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_rear_left_list_);

    obstacle_radar_rear_left_list_ = data;
  }
  void GetObstacleRadarRearLeftList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_radar_rear_left_list_);

    *data = obstacle_radar_rear_left_list_;
  }
  void ClearObstacleRadarRearLeftList() {
    common::os::WriteLockHelper lock(lock_obstacle_radar_rear_left_list_);
    obstacle_radar_rear_left_list_.Clear();
  }

  void SetObstacleRadarRearRightList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_rear_right_list_);

    obstacle_radar_rear_right_list_ = data;
  }
  void GetObstacleRadarRearRightList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_radar_rear_right_list_);

    *data = obstacle_radar_rear_right_list_;
  }
  void ClearObstacleRadarRearRightList() {
    common::os::WriteLockHelper lock(lock_obstacle_radar_rear_right_list_);
    obstacle_radar_rear_right_list_.Clear();
  }

  // SRR2 Detections
  void SetSrr2DetectionsFrontLeftList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_srr2_detections_front_left_list_);

    srr2_detections_front_left_list_ = data;
  }
  void GetSrr2DetectionsFrontLeftList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_srr2_detections_front_left_list_);

    *data = srr2_detections_front_left_list_;
  }
  void ClearSrr2DetectionsFrontLeftList() {
    common::os::WriteLockHelper lock(lock_srr2_detections_front_left_list_);
    srr2_detections_front_left_list_.Clear();
  }

  void SetSrr2DetectionsFrontRightList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_srr2_detections_front_right_list_);

    srr2_detections_front_right_list_ = data;
  }
  void GetSrr2DetectionsFrontRightList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_srr2_detections_front_right_list_);

    *data = srr2_detections_front_right_list_;
  }
  void ClearSrr2DetectionsFrontRightList() {
    common::os::WriteLockHelper lock(lock_srr2_detections_front_right_list_);
    srr2_detections_front_right_list_.Clear();
  }

  void SetSrr2DetectionsRearLeftList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_srr2_detections_rear_left_list_);

    srr2_detections_rear_left_list_ = data;
  }
  void GetSrr2DetectionsRearLeftList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_srr2_detections_rear_left_list_);

    *data = srr2_detections_rear_left_list_;
  }
  void ClearSrr2DetectionsRearLeftList() {
    common::os::WriteLockHelper lock(lock_srr2_detections_rear_left_list_);
    srr2_detections_rear_left_list_.Clear();
  }

  void SetSrr2DetectionsRearRightList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_srr2_detections_rear_right_list_);

    srr2_detections_rear_right_list_ = data;
  }
  void GetSrr2DetectionsRearRightList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_srr2_detections_rear_right_list_);

    *data = srr2_detections_rear_right_list_;
  }
  void ClearSrr2DetectionsRearRightList() {
    common::os::WriteLockHelper lock(lock_srr2_detections_rear_right_list_);
    srr2_detections_rear_right_list_.Clear();
  }

  // Obstacles Lidar
  void SetObstacleLidarList0(const ad_msg::ObstacleLidarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_lidar_list_0_);

    obstacle_lidar_list_0_ = data;
  }
  void GetObstacleLidarList0(ad_msg::ObstacleLidarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_lidar_list_0_);

    *data = obstacle_lidar_list_0_;
  }
  void ClearObstacleLidarList0() {
    common::os::WriteLockHelper lock(lock_obstacle_lidar_list_0_);
    obstacle_lidar_list_0_.Clear();
  }

  void SetObstacleLidarList1(const ad_msg::ObstacleLidarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_lidar_list_1_);

    obstacle_lidar_list_1_ = data;
  }
  void GetObstacleLidarList1(ad_msg::ObstacleLidarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_lidar_list_1_);

    *data = obstacle_lidar_list_1_;
  }
  void ClearObstacleLidarList1() {
    common::os::WriteLockHelper lock(lock_obstacle_lidar_list_1_);
    obstacle_lidar_list_1_.Clear();
  }


  // tracked obstacles
  void SetObstacleTrackedInfoList(const ad_msg::ObstacleTrackedInfoList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_tracked_info_list_);

    obstacle_tracked_info_list_ = data;
  }
  void GetObstacleTrackedInfoList(ad_msg::ObstacleTrackedInfoList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_tracked_info_list_);

    *data = obstacle_tracked_info_list_;
  }

  void SetTrafficSignalList(const ad_msg::TrafficSignalList& data){
    common::os::WriteLockHelper lock(lock_traffic_signal_list_);
    traffic_signal_list_ = data;
  }
  void GetTrafficSignalList(ad_msg::TrafficSignalList* data){
    common::os::ReadLockHelper lock(lock_traffic_signal_list_);
    *data = traffic_signal_list_;
  }
  void ClearTrafficSignalList() {
    common::os::WriteLockHelper lock(lock_traffic_signal_list_);
    traffic_signal_list_.Clear();
  }

  void SetTrafficLightList(const ad_msg::TrafficLightList& data){
    common::os::WriteLockHelper lock(lock_traffic_light_list_);
    traffic_light_list_ = data;
  }

  void GetTrafficLightList(ad_msg::TrafficLightList* data){
    common::os::ReadLockHelper lock(lock_traffic_light_list_);
    *data = traffic_light_list_;
  }


  // Relative pos list
  void SetRelativePosList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_relative_pos_list_);

    relative_pos_list_ = data;
  }
  void GetRelativePosList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_relative_pos_list_);

    *data = relative_pos_list_;
  }

  // pos filter information
  void SetPosFilterInfo(const pos_filter::PosFilterInfo& data) {
    common::os::WriteLockHelper lock(lock_pos_filter_info_);

    pos_filter_info_ = data;
  }
  void GetPosFilterInfo(pos_filter::PosFilterInfo* data) {
    common::os::ReadLockHelper lock(lock_pos_filter_info_);

    *data = pos_filter_info_;
  }

  // gnss track list
  void SetGnssTrackList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_gnss_track_list_);

    gnss_track_list_ = data;
  }
  void GetGnssTrackList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_gnss_track_list_);

    *data = gnss_track_list_;
  }

  // gnss filtered track list
  void SetGnssFilteredTrackList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_gnss_filtered_track_list_);

    gnss_filtered_track_list_ = data;
  }
  void GetGnssFilteredTrackList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_gnss_filtered_track_list_);

    *data = gnss_filtered_track_list_;
  }

  // utm track list
  void SetUtmTrackList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_utm_track_list_);

    utm_track_list_ = data;
  }
  void GetUtmTrackList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_utm_track_list_);

    *data = utm_track_list_;
  }

  // utm filtered track list
  void SetUtmFilteredTrackList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_utm_filtered_track_list_);

    utm_filtered_track_list_ = data;
  }
  void GetUtmFilteredTrackList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_utm_filtered_track_list_);

    *data = utm_filtered_track_list_;
  }

  // odom track list
  void SetOdomTrackList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_odom_track_list_);

    odom_track_list_ = data;
  }
  void GetOdomTrackList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_odom_track_list_);

    *data = odom_track_list_;
  }

  // odom filtered track list
  void SetOdomFilteredTrackList(const ad_msg::RelativePosList& data) {
    common::os::WriteLockHelper lock(lock_odom_filtered_track_list_);

    odom_filtered_track_list_ = data;
  }
  void GetOdomFilteredTrackList(ad_msg::RelativePosList* data) {
    common::os::ReadLockHelper lock(lock_odom_filtered_track_list_);

    *data = odom_filtered_track_list_;
  }

  // Filtered Gnss
  void SetFilteredGnssInfo(const ad_msg::Gnss& data) {
    common::os::WriteLockHelper lock(lock_filtered_gnss_info_);

    filtered_gnss_info_ = data;
  }
  void GetFilteredGnssInfo(ad_msg::Gnss* data) {
    common::os::ReadLockHelper lock(lock_filtered_gnss_info_);

    *data = filtered_gnss_info_;
  }

  // Action planning
  void SetActionPlanningResult(
      const planning::ActionPlanningResult& data) {
    common::os::WriteLockHelper lock(lock_action_planning_result_);

    action_planning_result_ = data;
  }
  void GetActionPlanningResult(
      planning::ActionPlanningResult* data) {
    common::os::ReadLockHelper lock(lock_action_planning_result_);

    *data = action_planning_result_;
  }

  // Trajectory planning
  void SetTrajectoryPlanningResult(
      const planning::TrajectoryPlanningResult& data) {
    common::os::WriteLockHelper lock(lock_trajectory_planning_result_);

    trajectory_planning_result_ = data;
  }
  void GetTrajectoryPlanningResult(
      planning::TrajectoryPlanningResult* data) {
    common::os::ReadLockHelper lock(lock_trajectory_planning_result_);

    *data = trajectory_planning_result_;
  }

  // Velocity planning
  void SetVelocityPlanningResult(
      const planning::VelocityPlanningResult& data) {
    common::os::WriteLockHelper lock(lock_velocity_planning_result_);

    velocity_planning_result_ = data;
  }
  void GetVelocityPlanningResult(
      planning::VelocityPlanningResult* data) {
    common::os::ReadLockHelper lock(lock_velocity_planning_result_);

    *data = velocity_planning_result_;
  }

  void SetVelocityPlanningInternal(
      const planning::VelocityPlanningInternal& data) {
    common::os::WriteLockHelper lock(lock_velocity_planning_internal_);

    velocity_planning_internal_ = data;
  }
  void GetVelocityPlanningInternal(
      planning::VelocityPlanningInternal* data) {
    common::os::ReadLockHelper lock(lock_velocity_planning_result_);

    *data = velocity_planning_internal_;
  }

  // Planning result
  void SetPlanningResult(const ad_msg::PlanningResult& data) {
    common::os::WriteLockHelper lock(lock_planningresult_info_);

    planningresult_ = data;
  }

  void GetPlanningResult(ad_msg::PlanningResult* data) {
    common::os::ReadLockHelper lock(lock_planningresult_info_);

    *data = planningresult_;
  }


  // map for display
  void SetDrivingMapInfo(const driv_map::DrivingMapInfo& data) {
    common::os::WriteLockHelper lock(lock_driving_map_info_);

    driving_map_info_ = data;
  }
  void GetDrivingMapInfo(driv_map::DrivingMapInfo* data) {
    common::os::ReadLockHelper lock(lock_driving_map_info_);

    *data = driving_map_info_;
  }

  // Trajectory planning information
  void SetTrajectoryPlanningInfo(const planning::TrajectoryPlanningInfo& data) {
    common::os::WriteLockHelper lock(lock_trajectory_planning_info_);

    trajectory_planning_info_ = data;
  }

  void GetTrajectoryPlanningInfo(planning::TrajectoryPlanningInfo* data) {
    common::os::ReadLockHelper lock(lock_trajectory_planning_info_);

    *data = trajectory_planning_info_;
  }

  void SetSceneStoryList(const ad_msg::SceneStoryList& data) {
    common::os::WriteLockHelper lock(lock_scene_story_list_);

    scene_story_list_ = data;
  }
  void GetSceneStoryList(ad_msg::SceneStoryList* data) {
    common::os::ReadLockHelper lock(lock_scene_story_list_);

    *data = scene_story_list_;
  }

  void SetMapLocalization(const ad_msg::MapLocalization& data) {
    common::os::WriteLockHelper lock(lock_map_localization_);

    map_localization_ = data;
  }
  void GetMapLocalization(ad_msg::MapLocalization* data) {
    common::os::ReadLockHelper lock(lock_map_localization_);

    *data = map_localization_;
  }

  // adecu_debug
  void SetAdecuVelplanDebug(const planning::VelocityPlanningResult& data) {
    common::os::WriteLockHelper lock(lock_adecu_vel_plan_result_);

    adecu_vel_plan_result_ = data;
  }
  void GetAdecuVelplanDebug(planning::VelocityPlanningResult* data) {
    common::os::ReadLockHelper lock(lock_adecu_vel_plan_result_);

    *data = adecu_vel_plan_result_;
  }


  void SetADASISV2Position(const adasisv2::PositionMessage& data) {
    common::os::WriteLockHelper lock(lock_adasisv2_position_);

    adasisv2_position_ = data;
  }

  void GetADASISV2Position(adasisv2::PositionMessage* data) {
    common::os::ReadLockHelper lock(lock_adasisv2_position_);

    *data = adasisv2_position_;
  }
  
  void SetADASISV2ProfileShort(const adasisv2::ProfileShortMessage& data) {
    common::os::WriteLockHelper lock(lock_adasisv2_profileshort_);

    adasisv2_profileshort_ = data;
  }

  void GetADASISV2ProfileShort(adasisv2::ProfileShortMessage* data) {
    common::os::ReadLockHelper lock(lock_adasisv2_profileshort_);

    *data = adasisv2_profileshort_;
  }
  void SetLateralControlInfo(const ad_msg::LateralControlInfo& data) {
    common::os::WriteLockHelper lock(lock_lateral_control_info_);

    lateral_control_info_ = data;
  }
  void GetLateralControlInfo(ad_msg::LateralControlInfo* data) {
    common::os::ReadLockHelper lock(lock_lateral_control_info_);

    *data = lateral_control_info_;
  }

private:
  // Module status
  ad_msg::ModuleStatusList module_status_list_;
  common::os::ReadWriteMutex lock_module_status_list_;
  // Control Module status
  ad_msg::ModuleStatusList control_module_status_list_;
  common::os::ReadWriteMutex lock_control_module_status_list_;
  // Event reporting
  common::RingBuffer<ad_msg::EventReporting, 10> event_reporting_buffer_;
  common::os::ReadWriteMutex lock_event_reporting_buffer_;
  ad_msg::EventReportingList event_reporting_list_;
  common::os::ReadWriteMutex lock_event_reporting_list_;

  // Current planning setting
  ad_msg::PlanningSettings curr_planning_settings_;
  common::os::ReadWriteMutex lock_curr_planning_settings_;
  // Local planning setting
  ad_msg::PlanningSettings local_planning_settings_;
  common::os::ReadWriteMutex lock_local_planning_settings_;
  // Remote planning setting
  ad_msg::PlanningSettings remote_planning_settings_;
  common::os::ReadWriteMutex lock_remote_planning_settings_;

  // Map
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  std::shared_ptr<const apollo::hdmap::Map> raw_hd_map_;
  std::shared_ptr<const apollo::routing::RoutingResponse> raw_routing_;
  common::os::ReadWriteMutex lock_raw_hd_map_;
  common::os::ReadWriteMutex lock_raw_routing_;
#endif

  // Gnss
  ad_msg::Gnss gnss_;
  common::os::ReadWriteMutex lock_gnss_;
  // IMU
  ad_msg::Imu imu_;
  common::os::ReadWriteMutex lock_imu_;
  // Chassis information
  ad_msg::Chassis chassis_;
  common::os::ReadWriteMutex lock_chassis_;
  // Special Chassis information
  ad_msg::SpecialChassisInfo special_chassis_info_;
  common::os::ReadWriteMutex lock_special_chassis_info_;
  // ChassisCtlCmd information
  ad_msg::ChassisCtlCmd chassis_ctl_cmd_;
  common::os::ReadWriteMutex lock_chassis_ctl_cmd_;

  // camera lane mark
  ad_msg::LaneMarkCameraList lane_mark_camera_list_;
  common::os::ReadWriteMutex lock_lane_mark_camera_list_;
  // camera lane line
  ad_msg::LaneInfoCameraList lane_info_camera_list_;
  common::os::ReadWriteMutex lock_lane_info_camera_list_;

  // obstacles
  ad_msg::ObstacleList obstacle_list_;
  common::os::ReadWriteMutex lock_obstacle_list_;
  // Outside obstacle list
  ad_msg::ObstacleList outside_obstacle_list_;
  common::os::ReadWriteMutex lock_outside_obstacle_list_;
  // camera obstacles
  ad_msg::ObstacleCameraList obstacle_camera_list_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_;
  // visual controller
  ad_msg::ObstacleCameraList obstacle_camera_list_front_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_front_;
  ad_msg::ObstacleCameraList obstacle_camera_list_left_front_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_left_front_;
  ad_msg::ObstacleCameraList obstacle_camera_list_right_front_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_right_front_;
  ad_msg::ObstacleCameraList obstacle_camera_list_rear_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_rear_;
  ad_msg::ObstacleCameraList obstacle_camera_list_left_rear_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_left_rear_;
  ad_msg::ObstacleCameraList obstacle_camera_list_right_rear_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_right_rear_;
  // ESR obstacles (front)
  ad_msg::ObstacleRadarList obstacle_radar_front_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_front_list_;
  // ESR obstacles (rear)
  ad_msg::ObstacleRadarList obstacle_radar_rear_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_rear_list_;
  // Srr2 obstacles
  ad_msg::ObstacleRadarList obstacle_radar_front_left_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_front_left_list_;
  ad_msg::ObstacleRadarList obstacle_radar_front_right_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_front_right_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_left_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_rear_left_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_right_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_rear_right_list_;
  // Srr2 Detections
  ad_msg::ObstacleRadarList srr2_detections_front_left_list_;
  common::os::ReadWriteMutex lock_srr2_detections_front_left_list_;
  ad_msg::ObstacleRadarList srr2_detections_front_right_list_;
  common::os::ReadWriteMutex lock_srr2_detections_front_right_list_;
  ad_msg::ObstacleRadarList srr2_detections_rear_left_list_;
  common::os::ReadWriteMutex lock_srr2_detections_rear_left_list_;
  ad_msg::ObstacleRadarList srr2_detections_rear_right_list_;
  common::os::ReadWriteMutex lock_srr2_detections_rear_right_list_;
  // Lidar obstacles
  ad_msg::ObstacleLidarList obstacle_lidar_list_0_;
  common::os::ReadWriteMutex lock_obstacle_lidar_list_0_;
  ad_msg::ObstacleLidarList obstacle_lidar_list_1_;
  common::os::ReadWriteMutex lock_obstacle_lidar_list_1_;
  // tracked obstacles
  ad_msg::ObstacleTrackedInfoList obstacle_tracked_info_list_;
  common::os::ReadWriteMutex lock_obstacle_tracked_info_list_;
  // Traffic signal
  ad_msg::TrafficSignalList traffic_signal_list_;
  common::os::ReadWriteMutex lock_traffic_signal_list_;
  // Traffic light
  ad_msg::TrafficLightList traffic_light_list_;
  common::os::ReadWriteMutex lock_traffic_light_list_;

  // Relative pos list
  ad_msg::RelativePosList relative_pos_list_;
  common::os::ReadWriteMutex lock_relative_pos_list_;
  // pos filter information
  pos_filter::PosFilterInfo pos_filter_info_;
  common::os::ReadWriteMutex lock_pos_filter_info_;
  // gnss track list
  ad_msg::RelativePosList gnss_track_list_;
  common::os::ReadWriteMutex lock_gnss_track_list_;
  // gnss filtered track list
  ad_msg::RelativePosList gnss_filtered_track_list_;
  common::os::ReadWriteMutex lock_gnss_filtered_track_list_;
  // utm track list
  ad_msg::RelativePosList utm_track_list_;
  common::os::ReadWriteMutex lock_utm_track_list_;
  // utm filtered track list
  ad_msg::RelativePosList utm_filtered_track_list_;
  common::os::ReadWriteMutex lock_utm_filtered_track_list_;
  // odom track list
  ad_msg::RelativePosList odom_track_list_;
  common::os::ReadWriteMutex lock_odom_track_list_;
  // odom filtered track list
  ad_msg::RelativePosList odom_filtered_track_list_;
  common::os::ReadWriteMutex lock_odom_filtered_track_list_;
  // Filtered Gnss
  ad_msg::Gnss filtered_gnss_info_;
  common::os::ReadWriteMutex lock_filtered_gnss_info_;

  // Action planning
  planning::ActionPlanningResult action_planning_result_;
  common::os::ReadWriteMutex lock_action_planning_result_;
  // Trajectory planning
  planning::TrajectoryPlanningResult trajectory_planning_result_;
  common::os::ReadWriteMutex lock_trajectory_planning_result_;
  // Velocity planning
  planning::VelocityPlanningResult velocity_planning_result_;
  common::os::ReadWriteMutex lock_velocity_planning_result_;

  planning::VelocityPlanningInternal velocity_planning_internal_;
  common::os::ReadWriteMutex lock_velocity_planning_internal_;

  // Planning result
  ad_msg::PlanningResult planningresult_;
  common::os::ReadWriteMutex lock_planningresult_info_;

  // map for debug
  driv_map::DrivingMapInfo driving_map_info_;
  common::os::ReadWriteMutex lock_driving_map_info_;
  // trajectory planning infomation for debug
  planning::TrajectoryPlanningInfo trajectory_planning_info_;
  common::os::ReadWriteMutex lock_trajectory_planning_info_;

  // scene story list
  ad_msg::SceneStoryList scene_story_list_;
  common::os::ReadWriteMutex lock_scene_story_list_;

  // map localization
  ad_msg::MapLocalization map_localization_;
  common::os::ReadWriteMutex lock_map_localization_;

  // adecu_debug
  planning::VelocityPlanningResult adecu_vel_plan_result_;
  common::os::ReadWriteMutex lock_adecu_vel_plan_result_;

  adasisv2::PositionMessage adasisv2_position_;
  common::os::ReadWriteMutex lock_adasisv2_position_;

  adasisv2::ProfileShortMessage adasisv2_profileshort_;
  common::os::ReadWriteMutex lock_adasisv2_profileshort_;
  
  // lateral control information
  ad_msg::LateralControlInfo lateral_control_info_;
  common::os::ReadWriteMutex lock_lateral_control_info_;

private:
  DECLARE_SINGLETON(SharedData);
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_SHARED_DATA_H_

