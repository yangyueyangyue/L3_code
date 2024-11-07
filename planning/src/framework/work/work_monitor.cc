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
  monitor_table_.recv_msg_hdmap.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_HDMAP, 20*4);
  monitor_table_.recv_msg_routing.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_ROUTING, 20*4);

  monitor_table_.recv_msg_gnss.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_GNSS, 4);
  monitor_table_.recv_msg_imu.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_IMU, 4);
  monitor_table_.recv_msg_chassis.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_CHASSIS, 4);
  monitor_table_.recv_msg_special_chassis_info.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_SPECIAL_CHASSIS_INFO, 4);
  monitor_table_.recv_msg_chassis_ctl_cmd.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_CHASSIS_CTL_CMD, 4);
  monitor_table_.recv_msg_lane_mark_camera_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_LANE_MARK_CAMERA_LIST, 4);
  monitor_table_.recv_msg_obstacle_camera_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMERA_LIST, 4);
  monitor_table_.recv_msg_obstacle_camgeneral_front_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMGENERAL_FRONT_LIST, 4);
  monitor_table_.recv_msg_obstacle_camgeneral_left_front_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMGENERAL_LEFT_FRONT_LIST, 4);
  monitor_table_.recv_msg_obstacle_camgeneral_right_front_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMGENERAL_RIGHT_FRONT_LIST, 4);
  monitor_table_.recv_msg_obstacle_camgeneral_rear_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMGENERAL_REAR_LIST, 4);
  monitor_table_.recv_msg_obstacle_camgeneral_left_rear_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMGENERAL_LEFT_REAR_LIST, 4);
  monitor_table_.recv_msg_obstacle_camgeneral_right_rear_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMGENERAL_RIGHT_REAR_LIST, 4);
  monitor_table_.recv_msg_obstacle_radar_front_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_LIST, 4);
  monitor_table_.recv_msg_obstacle_radar_rear_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_LIST, 4);
  monitor_table_.recv_msg_obstacle_radar_front_left_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_LEFT_LIST, 4);
  monitor_table_.recv_msg_obstacle_radar_front_right_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_RIGHT_LIST, 4);
  monitor_table_.recv_msg_obstacle_radar_rear_left_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_LEFT_LIST, 4);
  monitor_table_.recv_msg_obstacle_radar_rear_right_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_RIGHT_LIST, 4);
  monitor_table_.recv_msg_srr2_detections_front_left_list.Initialize(4);
  monitor_table_.recv_msg_srr2_detections_front_right_list.Initialize(4);
  monitor_table_.recv_msg_srr2_detections_rear_left_list.Initialize(4);
  monitor_table_.recv_msg_srr2_detections_rear_right_list.Initialize(4);
  monitor_table_.recv_msg_obstacle_lidar_list_0.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_LIDAR_LIST_0, 4);
  monitor_table_.recv_msg_obstacle_lidar_list_1.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_LIDAR_LIST_1, 4);
  monitor_table_.recv_msg_outside_obstacle_list.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OUTSIDE_OBSTACLE_LIST, 16);

  monitor_table_.task_planning.Initialize(
        ad_msg::SUB_MODULE_ID_MOTION_PLANNING_DO_WORK, 4);
}

void WorkMonitor::Configurate(const PlanningConfig& conf) {
  config_ = conf;
}

void WorkMonitor::FeedDog_RecvHdMap() {
  monitor_table_.recv_msg_hdmap.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvRouting() {
  monitor_table_.recv_msg_routing.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvGnss() {
  monitor_table_.recv_msg_gnss.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvImu() {
  monitor_table_.recv_msg_imu.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvChassis() {
  monitor_table_.recv_msg_chassis.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvSpecialChassisInfo() {
  monitor_table_.recv_msg_special_chassis_info.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvChassisCtlCmd() {
  monitor_table_.recv_msg_chassis_ctl_cmd.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvLaneMarkCameraList() {
  monitor_table_.recv_msg_lane_mark_camera_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCameraList() {
  monitor_table_.recv_msg_obstacle_camera_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamGeneralFrontList() {
  monitor_table_.recv_msg_obstacle_camgeneral_front_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamGeneralLeftFrontList() {
  monitor_table_.recv_msg_obstacle_camgeneral_left_front_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamGeneralRightFrontList() {
  monitor_table_.recv_msg_obstacle_camgeneral_right_front_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamGeneralRearList() {
  monitor_table_.recv_msg_obstacle_camgeneral_rear_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamGeneralLeftRearList() {
  monitor_table_.recv_msg_obstacle_camgeneral_left_rear_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleCamGeneralRightRearList() {
  monitor_table_.recv_msg_obstacle_camgeneral_right_rear_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleRadarFrontList() {
  monitor_table_.recv_msg_obstacle_radar_front_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleRadarRearList() {
  monitor_table_.recv_msg_obstacle_radar_rear_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleRadarFrontLeftList() {
  monitor_table_.recv_msg_obstacle_radar_front_left_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleRadarFrontRightList() {
  monitor_table_.recv_msg_obstacle_radar_front_right_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleRadarRearLeftList() {
  monitor_table_.recv_msg_obstacle_radar_rear_left_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleRadarRearRightList() {
  monitor_table_.recv_msg_obstacle_radar_rear_right_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvSrr2DetectionsFrontLeftList() {
  monitor_table_.recv_msg_srr2_detections_front_left_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvSrr2DetectionsFrontRightList() {
  monitor_table_.recv_msg_srr2_detections_front_right_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvSrr2DetectionsRearLeftList() {
  monitor_table_.recv_msg_srr2_detections_rear_left_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvSrr2DetectionsRearRightList() {
  monitor_table_.recv_msg_srr2_detections_rear_right_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleLidarList0() {
  monitor_table_.recv_msg_obstacle_lidar_list_0.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvObstacleLidarList1() {
  monitor_table_.recv_msg_obstacle_lidar_list_1.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_RecvOutsideObstacleList() {
  monitor_table_.recv_msg_outside_obstacle_list.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}
// adecu_debug
void WorkMonitor::FeedDog_RecvAdecuVelPlanning() {
  monitor_table_.recv_msg_adecu_vel_plan_debug.FeedDog(
        0, ad_msg::MODULE_STATUS_OK, 0, 0, 0);
}

void WorkMonitor::FeedDog_CompletePlanning(
    Int32_t running_status, Int32_t running_time) {
  monitor_table_.task_planning.FeedDog(
        0, running_status, running_time, 0, 0);
}

Int32_t WorkMonitor::DoWork() {
  // Check tasks status
  SharedData* shared_data = SharedData::instance();

  ad_msg::ModuleStatus module_status;
  bool radar_front_timeout = false;
  bool radar_rear_timeout = false;
  bool radar_front_left_timeout = false;
  bool radar_front_right_timeout = false;
  bool radar_rear_left_timeout = false;
  bool radar_rear_right_timeout = false;
  bool lidar_0_timeout = false;
  bool lidar_1_timeout = false;
  bool cam_lane_timeout = false;
  bool cam_obstacle_timeout = false;
  bool camgeneral_front_obstacle_timeout = false;
  bool camgeneral_left_front_obstacle_timeout = false;
  bool camgeneral_right_front_obstacle_timeout = false;
  bool camgeneral_rear_obstacle_timeout = false;
  bool camgeneral_left_rear_obstacle_timeout = false;
  bool camgeneral_right_rear_obstacle_timeout = false;
  bool gnss_timeout = false;
  bool imu_timeout = false;
  bool chassis_timeout = false;
  bool special_chassis_timeout = false;
  bool chassis_ctl_cmd_timeout = false;
  bool planning_timeout = false;
  bool adecu_vel_plan_debug_timeout = false;
  // 读取控制端的相关信息
  shared_data->GetSpecialChassisInfo(&special_chassis_info_);

  internal_module_status_list_.Clear();

  // message receive (hdmap)
  monitor_table_.recv_msg_hdmap.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (routing)
  monitor_table_.recv_msg_routing.UpdateStatus(&module_status);
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (gnss)
  monitor_table_.recv_msg_gnss.UpdateStatus(&module_status);
  if (module_status.timeout) {
    gnss_timeout = true;
    shared_data->ClearGnss();
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (imu)
  monitor_table_.recv_msg_imu.UpdateStatus(&module_status);
  if (module_status.timeout) {
    imu_timeout = true;
    shared_data->ClearImu();
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (chassis)
  monitor_table_.recv_msg_chassis.UpdateStatus(&module_status);
  if (module_status.timeout) {
    chassis_timeout = true;
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (special chassis info)
  monitor_table_.recv_msg_special_chassis_info.UpdateStatus(&module_status);
  if (module_status.timeout) {
    special_chassis_timeout = true;
    shared_data->ClearSpecialChassisInfo();
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // message receive (chassis command)
  monitor_table_.recv_msg_chassis_ctl_cmd.UpdateStatus(&module_status);
  if (module_status.timeout) {
    chassis_ctl_cmd_timeout = true;
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  if ((driv_map::DrivingMapConfig::INPUTTED_MAP_TYPE_CAMERA ==
       config_.driving_map_config.inputted_map_type) ||
      (driv_map::DrivingMapConfig::INPUTTED_MAP_TYPE_MIXED ==
             config_.driving_map_config.inputted_map_type)) {
    // message receive (lane mark camera)
    monitor_table_.recv_msg_lane_mark_camera_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      cam_lane_timeout = true;
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);
  }

  if (config_.obj_filter_config.using_outside_obj_list) {
    // message receive (obstacle radar)
    monitor_table_.recv_msg_outside_obstacle_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      //esr_timeout = true;
      shared_data->ClearOutsideObstacleList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);
  } else {
    // message receive (obstacle camera)
    monitor_table_.recv_msg_obstacle_camera_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      cam_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    // message receive (obstacle camgeneral front)
    monitor_table_.recv_msg_obstacle_camgeneral_front_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      camgeneral_front_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList_Front();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    // message receive (obstacle camgeneral left front)
    monitor_table_.recv_msg_obstacle_camgeneral_left_front_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      camgeneral_left_front_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList_Left_Front();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

        // message receive (obstacle camgeneral right front)
    monitor_table_.recv_msg_obstacle_camgeneral_right_front_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      camgeneral_right_front_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList_Right_Front();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);
    
    // message receive (obstacle camgeneral rear)
    monitor_table_.recv_msg_obstacle_camgeneral_rear_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      camgeneral_rear_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList_Rear();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    // message receive (obstacle camgeneral left rear)
    monitor_table_.recv_msg_obstacle_camgeneral_left_rear_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      camgeneral_left_rear_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList_Left_Rear();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    // message receive (obstacle camgeneral right rear)
    monitor_table_.recv_msg_obstacle_camgeneral_right_rear_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      camgeneral_right_rear_obstacle_timeout = true;
      shared_data->ClearObstacleCameraList_Right_Rear();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    // message receive (obstacle radar)
    monitor_table_.recv_msg_obstacle_radar_front_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_front_timeout = true;
      shared_data->ClearObstacleRadarFrontList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_obstacle_radar_rear_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_rear_timeout = true;
      shared_data->ClearObstacleRadarRearList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_obstacle_radar_front_left_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_front_left_timeout = true;
      shared_data->ClearObstacleRadarFrontLeftList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_obstacle_radar_front_right_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_front_right_timeout = true;
      shared_data->ClearObstacleRadarFrontRightList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_obstacle_radar_rear_left_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_rear_left_timeout = true;
      shared_data->ClearObstacleRadarRearLeftList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_obstacle_radar_rear_right_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_rear_right_timeout = true;
      shared_data->ClearObstacleRadarRearRightList();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_srr2_detections_front_left_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_front_left_timeout = true;
      shared_data->ClearSrr2DetectionsFrontLeftList();
    }

    monitor_table_.recv_msg_srr2_detections_front_right_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_front_right_timeout = true;
      shared_data->ClearSrr2DetectionsFrontRightList();
    }

    monitor_table_.recv_msg_srr2_detections_rear_left_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_rear_left_timeout = true;
      shared_data->ClearSrr2DetectionsRearLeftList();
    }

    monitor_table_.recv_msg_srr2_detections_rear_right_list.UpdateStatus(&module_status);
    if (module_status.timeout) {
      radar_rear_right_timeout = true;
      shared_data->ClearSrr2DetectionsRearRightList();
    }

    // message receive (obstacle lidar)
    monitor_table_.recv_msg_obstacle_lidar_list_0.UpdateStatus(&module_status);
    if (module_status.timeout) {
      lidar_0_timeout = true;
      shared_data->ClearObstacleLidarList0();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);

    monitor_table_.recv_msg_obstacle_lidar_list_1.UpdateStatus(&module_status);
    if (module_status.timeout) {
      lidar_1_timeout = true;
      shared_data->ClearObstacleLidarList1();
    }
    internal_module_status_list_.PushBackModuleStaus(module_status);
  }

    // adecu_debug
  monitor_table_.recv_msg_adecu_vel_plan_debug.UpdateStatus(&module_status);
  if (module_status.timeout) {
    adecu_vel_plan_debug_timeout = true;
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // planning
  monitor_table_.task_planning.UpdateStatus(&module_status);
  if (module_status.timeout) {
    planning_timeout = true;
  }
  internal_module_status_list_.PushBackModuleStaus(module_status);

  // save
  internal_module_status_list_.msg_head.valid = true;
  internal_module_status_list_.msg_head.src_module_id =
      ad_msg::MODULE_ID_MOTION_PLANNING;
  internal_module_status_list_.msg_head.UpdateSequenceNum();
  internal_module_status_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetModuleStatusList(internal_module_status_list_);

  bool can_timeout = false;
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  /// Check control module status
  shared_data->GetSpecialChassisInfo(&special_chassis_info_);
  control_module_status_list_.Clear();

  bool can0_timeout = false;
  bool can1_timeout = false;
  bool can2_timeout = false;
  bool can3_timeout = false;

  module_status.sub_module_id =
      ad_msg::SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_0;
  if (special_chassis_info_.cnt_stu_ctl_to_gtw &&
      special_chassis_info_.cnt_stu_ctl_to_gtw_can0 &&
      special_chassis_info_.cnt_stu_gtw_to_veh_can0) {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = false;
  } else {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = true;
    can0_timeout = true;
  }
  control_module_status_list_.PushBackModuleStaus(module_status);

  module_status.sub_module_id =
      ad_msg::SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_1;
  if (special_chassis_info_.cnt_stu_ctl_to_gtw &&
      /*special_chassis_info_.cnt_stu_ctl_to_gtw_can1 &&*/
      special_chassis_info_.cnt_stu_gtw_to_veh_can1) {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = false;
  } else {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = true;
    can1_timeout = true;
  }
  control_module_status_list_.PushBackModuleStaus(module_status);

  module_status.sub_module_id =
      ad_msg::SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_2;
  if (special_chassis_info_.cnt_stu_ctl_to_gtw &&
      special_chassis_info_.cnt_stu_ctl_to_gtw_can2 &&
      special_chassis_info_.cnt_stu_gtw_to_veh_can2) {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = false;
  } else {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = true;
    can2_timeout = true;
  }
  control_module_status_list_.PushBackModuleStaus(module_status);

  module_status.sub_module_id =
      ad_msg::SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_3;
  if (special_chassis_info_.cnt_stu_ctl_to_gtw &&
      special_chassis_info_.cnt_stu_ctl_to_gtw_can3 &&
      special_chassis_info_.cnt_stu_gtw_to_veh_can3) {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = false;
  } else {
    module_status.status = ad_msg::MODULE_STATUS_OK;
    common::com_memset(module_status.param, 0, sizeof(module_status.param));
    module_status.timeout = true;
    can3_timeout = true;
  }
  control_module_status_list_.PushBackModuleStaus(module_status);

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  can_timeout = can0_timeout || can1_timeout || can2_timeout;
#else
  can_timeout = can0_timeout;
#endif

  // save
  control_module_status_list_.msg_head.valid = true;
  control_module_status_list_.msg_head.src_module_id =
      ad_msg::MODULE_ID_CONTROL;
  control_module_status_list_.msg_head.UpdateSequenceNum();
  control_module_status_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetControlModuleStatusList(control_module_status_list_);
#endif  // #if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)

#if 0
  // 添加事件通知
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  if (imu_timeout || radar_front_timeout || cam_lane_timeout || cam_obstacle_timeout) {
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  if (gnss_timeout || imu_timeout ||
      radar_front_timeout || cam_lane_timeout || cam_obstacle_timeout) {
#else
  if (radar_front_timeout || cam_lane_timeout || cam_obstacle_timeout) {
#endif
    ad_msg::EventReporting event;
    event.event_id = ad_msg::EVENT_REPORTING_ID_SENSOR_MSG_TIMEOUT;
    event.priority = 0;
    event.lifetime = 3*1000;
    shared_data->AddEventReporting(1, &event);
  }
  if (chassis_timeout || chassis_ctl_cmd_timeout || can_timeout) {
    ad_msg::EventReporting event;
    event.event_id = ad_msg::EVNET_REPORTING_ID_CHASSIS_MSG_TIMEOUT;
    event.priority = 0;
    event.lifetime = 3*1000;
    shared_data->AddEventReporting(1, &event);
  }
  if (planning_timeout) {
    ad_msg::EventReporting event;
    event.event_id = ad_msg::EVNET_REPORTING_ID_PLANNING_TIMEOUT;
    event.priority = 0;
    event.lifetime = 3*1000;
    shared_data->AddEventReporting(1, &event);
  }
#endif

  UpdateModuleEventList();

  return (0);
}

void WorkMonitor::UpdateModuleEventList() {
  SharedData* shared_data = SharedData::instance();

  ad_msg::EventReporting event_list[10];
  Int32_t event_num = shared_data->GetEventReporting(10, event_list);

  module_event_list_type::iterator it =
      module_event_list_.begin(module_event_pool_);
  module_event_list_type::iterator it_end =
      module_event_list_.end(module_event_pool_);

  // 根据优先级添加到列表中
  for (Int32_t i = 0; i < event_num; ++i) {
    const ad_msg::EventReporting& ev = event_list[i];

    // 若已经存在列表中，则更新信息
    bool find = false;
    it = module_event_list_.begin(module_event_pool_);
    it_end = module_event_list_.end(module_event_pool_);
    for (; it != it_end; ++it) {
      if (it->event_id == ev.event_id) {
        find = true;
        *it = ev;
        break;
      }
    }

    // 若不在列表中, 则根据优先级添加
    if (!find) {
      find = false;
      it = module_event_list_.begin(module_event_pool_);
      it_end = module_event_list_.end(module_event_pool_);
      for (; it != it_end; ++it) {
        if (it->priority > ev.priority) {
          find = true;
          module_event_list_.Insert(it, ev, module_event_pool_);
          break;
        }
      }
      if (!find) {
        module_event_list_.PushBack(ev, module_event_pool_);
      }
    }
  }

  // 删除超时的信息
  it = module_event_list_.begin(module_event_pool_);
  it_end = module_event_list_.end(module_event_pool_);
  for (; it != it_end;) {
    it->lifetime -= 50;

    bool erase_flag = false;
    if (it->lifetime <= 0) {
      erase_flag = true;
    }

    if (erase_flag) {
      it = module_event_list_.Erase(it, module_event_pool_);
    } else {
      ++it;
    }
  }

  // 打印事件列表（调试）
#if 0
  Int32_t count = 0;
  std::cout << "### Event list:" << std::endl;
  it = module_event_list_.begin(module_event_pool_);
  it_end = module_event_list_.end(module_event_pool_);
  for (; it != it_end; ++it) {
    std::cout << "   Event[" << count << "]: id=" << it->event_id
              << ", life_time=" << it->lifetime
              << ", priority=" << it->priority
              << std::endl;
  }
#endif

  // 保存事件列表
  Int32_t event_count = 0;
  it = module_event_list_.begin(module_event_pool_);
  it_end = module_event_list_.end(module_event_pool_);
  for (; it != it_end; ++it) {
    if (event_count >= ad_msg::EventReportingList::MAX_EVENT_REPORTING_NUM) {
      break;
    }

    ad_msg::EventReporting& ev =
        msg_event_reporing_list_.event_reporting[event_count];
    ev = *it;

    event_count++;
  }
  msg_event_reporing_list_.event_reporting_num = event_count;

  // Save to shared memory
  msg_event_reporing_list_.msg_head.valid = true;
  msg_event_reporing_list_.msg_head.src_module_id =
      ad_msg::MODULE_ID_MOTION_PLANNING;
  msg_event_reporing_list_.msg_head.UpdateSequenceNum();
  msg_event_reporing_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetEventReportingList(msg_event_reporing_list_);
}


}  // namespace framework
}  // namespace phoenix
