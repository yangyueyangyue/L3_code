//
#include "work/work_monitor.h"

#include "utils/macros.h"
#include "communication/shared_data.h"


namespace phoenix {
namespace framework {


WorkMonitor::WorkMonitor() {
  Initialize();
}

WorkMonitor::~WorkMonitor() {
}

void WorkMonitor::Initialize() {
  monitor_table_.recv_msg_gnss.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_GNSS, 4);
  monitor_table_.recv_msg_imu.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_IMU, 4);
  monitor_table_.recv_msg_lane_mark_camera.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_LANE, 4);
  monitor_table_.recv_msg_lane_curb_camera.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_LANE_CURB, 4);
  monitor_table_.recv_msg_obstacle_camera.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_OBJ, 4);
  monitor_table_.recv_msg_obstacle_esr_front.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_ESR_FRONT, 4);
  monitor_table_.recv_msg_chassis.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_CHASSIS, 4);
  monitor_table_.recv_msg_obstacle_maxieye_camera.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_MAXIEYE_CAMERA, 4);
  monitor_table_.recv_msg_obstacle_lidar_front.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_LIDAR_FRONT, 4);
  monitor_table_.recv_msg_obstacle_anngic_radar_front_left.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_FRONT_LEFT, 4);
  monitor_table_.recv_msg_obstacle_anngic_radar_front_right.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_FRONT_RIGHT, 4);
  monitor_table_.recv_msg_obstacle_anngic_radar_rear_left.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_REAR_LEFT, 4);
  monitor_table_.recv_msg_obstacle_anngic_radar_rear_right.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_REAR_RIGHT, 4);
  monitor_table_.recv_msg_obstacle_visual_control_front.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT, 4);
  monitor_table_.recv_msg_obstacle_visual_control_front_left.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT_LEFT, 4);
  monitor_table_.recv_msg_obstacle_visual_control_front_right.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT_RIGHT, 4);
  monitor_table_.recv_msg_obstacle_visual_control_rear_left.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR_LEFT, 4);
  monitor_table_.recv_msg_obstacle_visual_control_rear_right.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR_RIGHT, 4);
  monitor_table_.recv_msg_obstacle_visual_control_rear.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR, 4);
  monitor_table_.recv_msg_obstacle_fusion.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_FUSION_OBJ, 4);
  monitor_table_.recv_msg_mpu_state.Initialize(
        INTERNAL_MODULE_ID_MSG_RECV_MPU_STATE, 8);
}

void WorkMonitor::FeedDog_RecvGnss() {
  monitor_table_.recv_msg_gnss.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvImu() {
  monitor_table_.recv_msg_imu.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvLaneMarkCamera() {
  monitor_table_.recv_msg_lane_mark_camera.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvLaneCurbCamera() {
  monitor_table_.recv_msg_lane_curb_camera.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamera() {
  monitor_table_.recv_msg_obstacle_camera.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleMaxieyeCamera() {
  monitor_table_.recv_msg_obstacle_maxieye_camera.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstaclesLidarFront() {
  monitor_table_.recv_msg_obstacle_lidar_front.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleAnngicRadarFrontLeft() {
  monitor_table_.recv_msg_obstacle_anngic_radar_front_left.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleAnngicRadarFrontRight() {
  monitor_table_.recv_msg_obstacle_anngic_radar_front_right.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleAnngicRadarRearLeft() {
  monitor_table_.recv_msg_obstacle_anngic_radar_rear_left.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleAnngicRadarRearRight() {
  monitor_table_.recv_msg_obstacle_anngic_radar_rear_right.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleVisualControlFront() {
  monitor_table_.recv_msg_obstacle_visual_control_front.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleVisualControlFrontLeft() {
  monitor_table_.recv_msg_obstacle_visual_control_front_left.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleVisualControlFrontRight() {
  monitor_table_.recv_msg_obstacle_visual_control_front_right.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleVisualControlRearLeft() {
  monitor_table_.recv_msg_obstacle_visual_control_rear_left.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleVisualControlRearRight() {
  monitor_table_.recv_msg_obstacle_visual_control_rear_right.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleVisualControlRear() {
  monitor_table_.recv_msg_obstacle_visual_control_rear.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleEsrFront() {
  monitor_table_.recv_msg_obstacle_esr_front.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvChassis() {
  monitor_table_.recv_msg_chassis.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleFusionObj() {
  monitor_table_.recv_msg_obstacle_fusion.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvMpuState() {
  monitor_table_.recv_msg_mpu_state.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}



Int32_t WorkMonitor::DoWork() {
  // Check tasks status
  SharedData* shared_data = SharedData::instance();

  ad_msg::ModuleStatus module_status;

  internal_module_status_list_.Clear();

  // message receive (gnss)
  monitor_table_.recv_msg_gnss.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (imu)
  monitor_table_.recv_msg_imu.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (lane mark camera)
  monitor_table_.recv_msg_lane_mark_camera.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (lane curb camera)
  monitor_table_.recv_msg_lane_curb_camera.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (obstacle camera)
  //monitor_table_.recv_msg_obstacle_camera.UpdateStatus(&module_status);
  //internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (obstacle maxieye camera)
  monitor_table_.recv_msg_obstacle_maxieye_camera.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (obstacle lidar front)
  monitor_table_.recv_msg_obstacle_lidar_front.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (obstacle anngic radar front left)
  monitor_table_.recv_msg_obstacle_anngic_radar_front_left.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle anngic radar front right)
  monitor_table_.recv_msg_obstacle_anngic_radar_front_right.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle anngic radar rear left)
  monitor_table_.recv_msg_obstacle_anngic_radar_rear_left.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle anngic radar rear right)
  monitor_table_.recv_msg_obstacle_anngic_radar_rear_right.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (obstacle visual control front)
  monitor_table_.recv_msg_obstacle_visual_control_front.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle visual control front left)
  monitor_table_.recv_msg_obstacle_visual_control_front_left.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle visual control front right
  monitor_table_.recv_msg_obstacle_visual_control_front_right.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle visual control rear left)
  monitor_table_.recv_msg_obstacle_visual_control_rear_left.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle visual control rear right)
  monitor_table_.recv_msg_obstacle_visual_control_rear_right.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  // message receive (obstacle visual control rear)
  monitor_table_.recv_msg_obstacle_visual_control_rear.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (obstacle ESR front)
  monitor_table_.recv_msg_obstacle_esr_front.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (chassis)
  monitor_table_.recv_msg_chassis.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (mpu)
  monitor_table_.recv_msg_mpu_state.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);
  
  // message receive (fusion obstacle)
  monitor_table_.recv_msg_obstacle_fusion.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // save
  shared_data->SetModuleStatusList(internal_module_status_list_);

  return (0);
}


}  // namespace framework
}  // namespace phoenix
