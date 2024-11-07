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
#include "can_dev/can_driver.h"
#include "MpuState.pb.h"
#include <rosgraph_msgs/Clock.h>

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

  void SetObstacleCameraList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_camera_list_);
    obstacle_camera_list_ = data;
  }
  void GetObstacleCameraList(ad_msg::ObstacleCameraList *data) {
    common::os::ReadLockHelper lock(lock_obstacle_camera_list_);
    *data = obstacle_camera_list_;
  }

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

  // D17 Maxieye Camera Obstacles
  void SetObstacleMaxieyeCameraList(const ad_msg::ObstacleCameraList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_maxieye_camera_list_);
    obstacle_maxieye_camera_list_ = data;
  }
  void GetObstacleMaxieyeCameraList(ad_msg::ObstacleCameraList *data) {
    common::os::ReadLockHelper lock(lock_obstacle_maxieye_camera_list_);
    *data = obstacle_maxieye_camera_list_;
  }

  // D17 Lidar Front obstacles
  void SetObstacleLidarFrontList(const ad_msg::ObstacleLidarList &data) {
    common::os::WriteLockHelper lock(lock_obstacle_lidar_list_);
    obstacle_lidar_list_ = data;
  }
  void GetObstaclelidarFrontList(ad_msg::ObstacleLidarList *data) {
    common::os::ReadLockHelper lock(lock_obstacle_lidar_list_);
    *data = obstacle_lidar_list_;
  }

  // D17 Anngic Radar Obstacles
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

  // D17 Visual Control Lane Mark
  void SetVisualControlLaneMarkCameraList(const ad_msg::LaneMarkCameraList& data) {
    common::os::WriteLockHelper lock(lock_visual_control_lane_mark_camera_list_);
    visual_control_lane_mark_camera_list_ = data;
  }
  void GetVisualControlLaneMarkCameraList(ad_msg::LaneMarkCameraList* data) {
    common::os::ReadLockHelper lock(lock_visual_control_lane_mark_camera_list_);
    *data = visual_control_lane_mark_camera_list_;
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

  // 发给激光雷达的车道线CanFd数据
  void SetLaneCanFdData(const can_dev::CanFdFrame &data) {
    common::os::WriteLockHelper lock(lock_lane_canfd_data_);
    lane_canfd_data_ = data;
  }
  void GetLaneCanFdData(can_dev::CanFdFrame *data) {
    common::os::WriteLockHelper lock(lock_lane_canfd_data_);
    *data = lane_canfd_data_;
  }

  // void SetCameraCanFdFrameList(const phoenix::can_dev::CanFdFrameList &data) {
  //   common::os::WriteLockHelper lock(lock_camera_canfd_frame_list_);
  //   camera_canfd_frame_list_ = data;
  // }
  // void GetCameraCanFdFrameList(phoenix::can_dev::CanFdFrameList *data) {
  //   common::os::WriteLockHelper lock(lock_camera_canfd_frame_list_);
  //   *data = camera_canfd_frame_list_;
  // }

  // void SetArsRadarCanFrameList(const phoenix::can_dev::CanFrameList &data) {
  //   common::os::WriteLockHelper lock(lock_ars_can_frame_list_);
  //   ars_can_frame_list_ = data;
  // }
  // void GetArsRadarCanFrameList(phoenix::can_dev::CanFrameList *data) {
  //   common::os::WriteLockHelper lock(lock_ars_can_frame_list_);
  //   *data = ars_can_frame_list_;
  // }

  //obstacle fusion result
  void SetObstaclesFusionList(const ad_msg::ObstacleList& data){
      common::os::WriteLockHelper lock(lock_obstacles_fusion_list_);
      obstacles_fusion_list_ = data;
  }

  void GetObstaclesFusionList(ad_msg::ObstacleList* data){
      common::os::ReadLockHelper lock(lock_obstacles_fusion_list_);
      *data = obstacles_fusion_list_;
  }

  //ADHMI obstacle fusion result
  void SetADHMIObstaclesFusionList(const ad_msg::ObstacleList& data){
      common::os::WriteLockHelper lock(lock_adhmi_obstacles_fusion_list_);
      adhmi_obstacles_fusion_list_ = data;
  }

  void GetADHMIObstaclesFusionList(ad_msg::ObstacleList* data){
      common::os::ReadLockHelper lock(lock_adhmi_obstacles_fusion_list_);
      *data = adhmi_obstacles_fusion_list_;
  }

  //ADHMI Clock
  void SetADHMIClock(const rosgraph_msgs::Clock& data){
      common::os::WriteLockHelper lock(lock_adhmi_clock_);
      adhmi_clock_ = data;
  }

  void GetADHMIClock(rosgraph_msgs::Clock* data){
      common::os::ReadLockHelper lock(lock_adhmi_clock_);
      *data = adhmi_clock_;
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
  // camera obstacles
  ad_msg::ObstacleCameraList obstacle_camera_list_;
  common::os::ReadWriteMutex lock_obstacle_camera_list_;
  // ESR obstacles (front)
  ad_msg::ObstacleRadarList obstacle_esr_front_list_;
  common::os::ReadWriteMutex lock_obstacle_esr_front_list_;

  // Traffic signal
  ad_msg::TrafficSignalList traffic_signal_list_;
  common::os::ReadWriteMutex lock_traffic_signal_list_;

  // Chassis information
  ad_msg::Chassis chassis_;
  common::os::ReadWriteMutex lock_chassis_;

  // Mpu State
  MpuState::MonitorMpuState mpu_state_;
  common::os::ReadWriteMutex lock_mpu_state_;

  // D17 Maxieye Camera Obstacles
  ad_msg::ObstacleCameraList obstacle_maxieye_camera_list_;
  common::os::ReadWriteMutex lock_obstacle_maxieye_camera_list_;

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

  // D17 Visual Control lane mark
  ad_msg::LaneMarkCameraList visual_control_lane_mark_camera_list_;
  common::os::ReadWriteMutex lock_visual_control_lane_mark_camera_list_;

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

  // 发给激光雷达的车道线数据
  can_dev::CanFdFrame lane_canfd_data_;
  common::os::ReadWriteMutex lock_lane_canfd_data_;

  // phoenix::can_dev::CanFdFrameList camera_canfd_frame_list_;
  // common::os::ReadWriteMutex lock_camera_canfd_frame_list_;

  // phoenix::can_dev::CanFrameList ars_can_frame_list_;
  // common::os::ReadWriteMutex lock_ars_can_frame_list_;
  
  // Object fusion result
  ad_msg::ObstacleList obstacles_fusion_list_;
  common::os::ReadWriteMutex lock_obstacles_fusion_list_;
  
  // AD_HMI Object fusion result
  ad_msg::ObstacleList adhmi_obstacles_fusion_list_;
  common::os::ReadWriteMutex lock_adhmi_obstacles_fusion_list_;
  
  // AD_HMI Clock
  rosgraph_msgs::Clock adhmi_clock_;
  common::os::ReadWriteMutex lock_adhmi_clock_;

private:
  DECLARE_SINGLETON(SharedData);
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_SHARED_DATA_H_

