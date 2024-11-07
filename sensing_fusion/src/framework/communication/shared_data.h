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
#include "os/mutex.h"
#include "msg_event_reporting.h"
#include "common/module_status.h"
#include "ad_msg.h"
#include "ExpWrapperComm.h"
#include "MpuState.pb.h"


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

  void SetGnssData(const ad_msg::Gnss &data) {
    common::os::WriteLockHelper lock(lock_gnss_data_);
    gnss_data_ = data;
  }
  void GetGnssData(ad_msg::Gnss *data) {
    common::os::ReadLockHelper lock(lock_gnss_data_);
    *data = gnss_data_;
  }

  void SetImuData(const ad_msg::Imu &data) {
    common::os::WriteLockHelper lock(lock_imu_data_);
    imu_data_ = data;
  }
  void GetImuData(ad_msg::Imu *data) {
    common::os::ReadLockHelper lock(lock_imu_data_);
    *data = imu_data_;
  }

  void SetLaneMarkCameraList(const ad_msg::LaneMarkCameraList& data) {
    common::os::WriteLockHelper lock(lock_lane_mark_camera_list_);
    lane_mark_camera_list_ = data;
  }
  void GetLaneMarkCameraList(ad_msg::LaneMarkCameraList* data) {
    common::os::ReadLockHelper lock(lock_lane_mark_camera_list_);
    *data = lane_mark_camera_list_;
  }

  void SetLaneCurbCameraList(const ad_msg::LaneMarkCameraList& data) {
    common::os::WriteLockHelper lock(lock_lane_curb_camera_list_);
    lane_curb_camera_list_ = data;
  }
  void GetLaneCurbCameraList(ad_msg::LaneMarkCameraList* data) {
    common::os::ReadLockHelper lock(lock_lane_curb_camera_list_);
    *data = lane_curb_camera_list_;
  }

  // D17 Maxieye Camera Obstacles
  void SetObstacleMaxieyeCameraList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_maxieye_camera_list_);
    obstacle_maxieye_camera_list_ = data;
  }
  void GetObstacleMaxieyeCameraList(ad_msg::ObstacleCameraList *data) {
    common::os::ReadLockHelper lock(lock_obstacle_maxieye_camera_list_);
    *data = obstacle_maxieye_camera_list_;
  }

  // D17 Maxieye Camera Obstacles After Preprocess
  void SetObstacleMaxieyeCameraListAfterPreprocess(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_maxieye_camera_list_after_preprocess_);
    obstacle_maxieye_camera_list_after_preprocess_ = data;
  }
  void GetObstacleMaxieyeCameraListAfterPreprocess(ad_msg::ObstacleCameraList *data) {
    common::os::ReadLockHelper lock(lock_obstacle_maxieye_camera_list_after_preprocess_);
    *data = obstacle_maxieye_camera_list_after_preprocess_;
  }

  // D17 Lidar Front obstacles
  void SetObstacleLidarFrontList(const ad_msg::ObstacleLidarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_lidar_list_);
    obstacle_lidar_list_ = data;
  }
  void GetObstacleLidarFrontList(ad_msg::ObstacleLidarList *data) {
    common::os::ReadLockHelper lock(lock_obstacle_lidar_list_);
    *data = obstacle_lidar_list_;
  }

  // Anngic Radar Preprocess Obstacles 
  void SetObstacleAnngicRadarFrontLeftList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_front_left_list_);
    obstacle_anngic_radar_front_left_list_ = data;
  }
  void GetObstacleAnngicRadarFrontLeftList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_front_left_list_);
    *data = obstacle_anngic_radar_front_left_list_;
  }
  void SetObstacleAnngicRadarFrontRightList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_front_right_list_);
    obstacle_anngic_radar_front_right_list_ = data;
  }
  void GetObstacleAnngicRadarFrontRightList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_front_right_list_);
    *data = obstacle_anngic_radar_front_right_list_;
  }

  void SetObstacleAnngicRadarRearLeftList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_rear_left_list_);
    obstacle_anngic_radar_rear_left_list_ = data;
  }
  void GetObstacleAnngicRadarRearLeftList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_rear_left_list_);
    *data = obstacle_anngic_radar_rear_left_list_;
  }

  void SetObstacleAnngicRadarRearRightList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_rear_right_list_);
    obstacle_anngic_radar_rear_right_list_ = data;
  }
  void GetObstacleAnngicRadarRearRightList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_anngic_radar_rear_right_list_);
    *data = obstacle_anngic_radar_rear_right_list_;
  }

  void SetObstacleRadarLeftFwFilterList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_left_fw_filter_list_);
    obstacle_radar_left_fw_filter_list_ = data;
  }
  void GetObstacleRadarLeftFwFilterList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_left_fw_filter_list_);
    *data = obstacle_radar_left_fw_filter_list_;
  }

  void SetObstacleRadarRightFwFilterList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_right_fw_filter_list_);
    obstacle_radar_right_fw_filter_list_ = data;
  }
  void GetObstacleRadarRightFwFilterList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_right_fw_filter_list_);
    *data = obstacle_radar_right_fw_filter_list_;
  }

  void SetObstacleRadarLeftBwFilterList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_left_bw_filter_list_);
    obstacle_radar_left_bw_filter_list_ = data;
  }
  void GetObstacleRadarLeftBwFilterList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_left_bw_filter_list_);
    *data = obstacle_radar_left_bw_filter_list_;
  }

  void SetObstacleRadarRightBwFilterList(const ad_msg::ObstacleRadarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_right_bw_filter_list_);
    obstacle_radar_right_bw_filter_list_ = data;
  }
  void GetObstacleRadarRightBwFilterList(ad_msg::ObstacleRadarList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_radar_right_bw_filter_list_);
    *data = obstacle_radar_right_bw_filter_list_;
  }


  // D17 Visual Control Obstacles
  void SetObstacleVisualControlFrontList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_front_list_);
    obstacle_visual_control_front_list_ = data;
  }
  void GetObstacleVisualControlFrontList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_front_list_);
    *data = obstacle_visual_control_front_list_;
  }

  void SetObstacleVisualControlFrontLeftList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_front_left_list_);
    obstacle_visual_control_front_left_list_ = data;
  }
  void GetObstacleVisualControlFrontLeftList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_front_left_list_);
    *data = obstacle_visual_control_front_left_list_;
  }

  void SetObstacleVisualControlFrontRightList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_front_right_list_);
    obstacle_visual_control_front_right_list_ = data;
  }
  void GetObstacleVisualControlFrontRightList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_front_right_list_);
    *data = obstacle_visual_control_front_right_list_;
  }

  void SetObstacleVisualControlRearLeftList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_rear_left_list_);
    obstacle_visual_control_rear_left_list_ = data;
  }
  void GetObstacleVisualControlRearLeftList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_rear_left_list_);
    *data = obstacle_visual_control_rear_left_list_;
  }

  void SetObstacleVisualControlRearRightList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_rear_right_list_);
    obstacle_visual_control_rear_right_list_ = data;
  }
  void GetObstacleVisualControlRearRightList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_rear_right_list_);
    *data = obstacle_visual_control_rear_right_list_;
  }

  void SetObstacleVisualControlRearList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_rear_list_);
    obstacle_visual_control_rear_list_ = data;
  }
  void GetObstacleVisualControlRearList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_visual_control_rear_list_);
    *data = obstacle_visual_control_rear_list_;
  }

  //jm Preprocess Obstacles
    void SetObstacleJMFrontList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_front_list_);
    obstacle_jm_preprocess_front_list_ = data;
  }
  void GetObstacleJMFrontList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_front_list_);
    *data = obstacle_jm_preprocess_front_list_;
  }

  void SetObstacleJMFrontLeftList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_front_left_list_);
    obstacle_jm_preprocess_front_left_list_ = data;
  }
  void GetObstacleJMFrontLeftList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_front_left_list_);
    *data = obstacle_jm_preprocess_front_left_list_;
  }

  void SetObstacleJMFrontRightList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_front_right_list_);
    obstacle_jm_preprocess_front_right_list_ = data;
  }
  void GetObstacleJMFrontRightList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_front_right_list_);
    *data = obstacle_jm_preprocess_front_right_list_;
  }

  void SetObstacleJMRearLeftList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_rear_left_list_);
    obstacle_jm_preprocess_rear_left_list_ = data;
  }
  void GetObstacleJMRearLeftList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_rear_left_list_);
    *data = obstacle_jm_preprocess_rear_left_list_;
  }

  void SetObstacleJMRearRightList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_rear_right_list_);
    obstacle_jm_preprocess_rear_right_list_ = data;
  }
  void GetObstacleJMRearRightList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_rear_right_list_);
    *data = obstacle_jm_preprocess_rear_right_list_;
  }

  void SetObstacleJMRearList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_rear_list_);
    obstacle_jm_preprocess_rear_list_ = data;
  }
  void GetObstacleJMRearList(ad_msg::ObstacleCameraList *data) {
    common::os::WriteLockHelper lock(lock_obstacle_jm_preprocess_rear_list_);
    *data = obstacle_jm_preprocess_rear_list_;
  }


  // 430雷达
  void SetObstacleEsrFrontList(const ad_msg::ObstacleRadarList& data) {
    common::os::WriteLockHelper lock(lock_obstacle_esr_front_list_);

    obstacle_esr_front_list_ = data;
  }
  void GetObstacleEsrFrontList(ad_msg::ObstacleRadarList* data) {
    common::os::ReadLockHelper lock(lock_obstacle_esr_front_list_);

    *data = obstacle_esr_front_list_;
  }

  // Traffic signal
  void SetTrafficSignalList(const ad_msg::TrafficSignalList& data){
    common::os::WriteLockHelper lock(lock_traffic_signal_list_);

    traffic_signal_list_ = data;
  }
  void GetTrafficSignalList(ad_msg::TrafficSignalList* data){
    common::os::ReadLockHelper lock(lock_traffic_signal_list_);

    *data = traffic_signal_list_;
  }

  // Traffic light
  void SetTrafficLightList(const ad_msg::TrafficLightList& data){
    common::os::WriteLockHelper lock(lock_traffic_light_list_);

    traffic_light_list_ = data;
  }
  void GetTrafficLightList(ad_msg::TrafficLightList* data){
    common::os::ReadLockHelper lock(lock_traffic_light_list_);

    *data = traffic_light_list_;
  }

  // Chassis information
  void SetChassis(const ad_msg::Chassis& data) {
    common::os::WriteLockHelper lock(lock_chassis_);

    chassis_ = data;
  }
  void GetChassis(ad_msg::Chassis* data) {
    common::os::ReadLockHelper lock(lock_chassis_);

    *data = chassis_;
  }

   //  Mpu State
  void SetMpuState(const MpuState::MonitorMpuState& data) {
    common::os::WriteLockHelper lock(lock_mpu_state_);

    mpu_state_ = data;
  }
  void GetMpuState(MpuState::MonitorMpuState* data) {
    common::os::ReadLockHelper lock(lock_mpu_state_);

    *data = mpu_state_;
  }


  // Geofence Info
  void SetMapGeofenceInfo(const exp_map_t::stMapGeofenceInfo& data) {
    common::os::WriteLockHelper lock(lock_geofence_info_);

    geofence_info_ = data;
  }
  void GetMapGeofenceInfo(exp_map_t::stMapGeofenceInfo* data) {
    common::os::ReadLockHelper lock(lock_geofence_info_);

    *data = geofence_info_;
  }


  //obstacle fusion result
  void SetObstaclesFusionList(const ad_msg::ObstacleList& data){
      common::os::WriteLockHelper lock(lock_obstacles_fusion_list_);
      obstacles_fusion_list_ = data;
  }

  void GetObstaclesFusionList(ad_msg::ObstacleList* data){
      common::os::ReadLockHelper lock(lock_obstacles_fusion_list_);
      *data = obstacles_fusion_list_;
  }

  //perception module status
  void SetPerceptionModuleStatus(const framework::PerceptionModuleStatus& data){
      common::os::WriteLockHelper lock(lock_perception_module_status_);
      perception_module_status_ = data;
  }

  void GetPerceptionModuleStatus(framework::PerceptionModuleStatus* data){
      common::os::ReadLockHelper lock(lock_perception_module_status_);
      *data = perception_module_status_;
  }


private:
  // Module status
  ad_msg::ModuleStatusList module_status_list_;
  common::os::ReadWriteMutex lock_module_status_list_;

  // GNSS
  ad_msg::Gnss gnss_data_;
  common::os::ReadWriteMutex lock_gnss_data_;

  // IMU
  ad_msg::Imu imu_data_;
  common::os::ReadWriteMutex lock_imu_data_;

  // camera lane mark
  ad_msg::LaneMarkCameraList lane_mark_camera_list_;
  common::os::ReadWriteMutex lock_lane_mark_camera_list_;

  // camera lane curb
  ad_msg::LaneMarkCameraList lane_curb_camera_list_;
  common::os::ReadWriteMutex lock_lane_curb_camera_list_;

  // ESR obstacles (front)
  ad_msg::ObstacleRadarList obstacle_esr_front_list_;
  common::os::ReadWriteMutex lock_obstacle_esr_front_list_;

  // Traffic signal
  ad_msg::TrafficSignalList traffic_signal_list_;
  common::os::ReadWriteMutex lock_traffic_signal_list_;
  // Traffic light
  ad_msg::TrafficLightList traffic_light_list_;
  common::os::ReadWriteMutex lock_traffic_light_list_;

  // Chassis information
  ad_msg::Chassis chassis_;
  common::os::ReadWriteMutex lock_chassis_;

  // Mpu State
  MpuState::MonitorMpuState mpu_state_;
  common::os::ReadWriteMutex lock_mpu_state_;

  // Geofence Info
  exp_map_t::stMapGeofenceInfo geofence_info_;
  common::os::ReadWriteMutex lock_geofence_info_;

  // Object fusion result
  ad_msg::ObstacleList obstacles_fusion_list_;
  common::os::ReadWriteMutex lock_obstacles_fusion_list_;

  // D17 Maxieye Camera Obstacles
  ad_msg::ObstacleCameraList obstacle_maxieye_camera_list_;
  common::os::ReadWriteMutex lock_obstacle_maxieye_camera_list_;


  // D17 Maxieye Camera Obstacles After Preprocess
  ad_msg::ObstacleCameraList obstacle_maxieye_camera_list_after_preprocess_;
  common::os::ReadWriteMutex lock_obstacle_maxieye_camera_list_after_preprocess_;


  // D17 Lidar Front obstacles
  ad_msg::ObstacleLidarList obstacle_lidar_list_;
  common::os::ReadWriteMutex lock_obstacle_lidar_list_;

  // D17 Anngic Radar Obstacles
  ad_msg::ObstacleRadarList obstacle_anngic_radar_front_left_list_;
  common::os::ReadWriteMutex lock_obstacle_anngic_radar_front_left_list_;

  ad_msg::ObstacleRadarList obstacle_anngic_radar_front_right_list_;
  common::os::ReadWriteMutex lock_obstacle_anngic_radar_front_right_list_;

  ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_left_list_;
  common::os::ReadWriteMutex lock_obstacle_anngic_radar_rear_left_list_;

  ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_right_list_;
  common::os::ReadWriteMutex lock_obstacle_anngic_radar_rear_right_list_;

  // Anngic Radar Pretreatment Obstacles
  ad_msg::ObstacleRadarList obstacle_radar_left_fw_filter_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_left_fw_filter_list_;

  ad_msg::ObstacleRadarList obstacle_radar_right_fw_filter_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_right_fw_filter_list_;

  ad_msg::ObstacleRadarList obstacle_radar_left_bw_filter_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_left_bw_filter_list_;

  ad_msg::ObstacleRadarList obstacle_radar_right_bw_filter_list_;
  common::os::ReadWriteMutex lock_obstacle_radar_right_bw_filter_list_;


  // D17 Visual Control Obstacles
  ad_msg::ObstacleCameraList obstacle_visual_control_front_list_;
  common::os::ReadWriteMutex lock_obstacle_visual_control_front_list_;

  ad_msg::ObstacleCameraList obstacle_visual_control_front_left_list_;
  common::os::ReadWriteMutex lock_obstacle_visual_control_front_left_list_;

  ad_msg::ObstacleCameraList obstacle_visual_control_front_right_list_;
  common::os::ReadWriteMutex lock_obstacle_visual_control_front_right_list_;

  ad_msg::ObstacleCameraList obstacle_visual_control_rear_left_list_;
  common::os::ReadWriteMutex lock_obstacle_visual_control_rear_left_list_;

  ad_msg::ObstacleCameraList obstacle_visual_control_rear_right_list_;
  common::os::ReadWriteMutex lock_obstacle_visual_control_rear_right_list_;

  ad_msg::ObstacleCameraList obstacle_visual_control_rear_list_;
  common::os::ReadWriteMutex lock_obstacle_visual_control_rear_list_;

  //jm Preprocess Obstacles
  ad_msg::ObstacleCameraList obstacle_jm_preprocess_front_list_;
  common::os::ReadWriteMutex lock_obstacle_jm_preprocess_front_list_;

  ad_msg::ObstacleCameraList obstacle_jm_preprocess_front_left_list_;
  common::os::ReadWriteMutex lock_obstacle_jm_preprocess_front_left_list_;

  ad_msg::ObstacleCameraList obstacle_jm_preprocess_front_right_list_;
  common::os::ReadWriteMutex lock_obstacle_jm_preprocess_front_right_list_;

  ad_msg::ObstacleCameraList obstacle_jm_preprocess_rear_left_list_;
  common::os::ReadWriteMutex lock_obstacle_jm_preprocess_rear_left_list_;

  ad_msg::ObstacleCameraList obstacle_jm_preprocess_rear_right_list_;
  common::os::ReadWriteMutex lock_obstacle_jm_preprocess_rear_right_list_;

  ad_msg::ObstacleCameraList obstacle_jm_preprocess_rear_list_;
  common::os::ReadWriteMutex lock_obstacle_jm_preprocess_rear_list_;

  framework::PerceptionModuleStatus perception_module_status_;
  common::os::ReadWriteMutex lock_perception_module_status_;


private:
  DECLARE_SINGLETON(SharedData);
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_SHARED_DATA_H_

