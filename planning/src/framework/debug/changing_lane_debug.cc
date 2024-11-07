//
#include "debug/changing_lane_debug.h"

#include "utils/macros.h"
#include "utils/gps_tools.h"
#include "geometry/geometry_utils.h"
#include "communication/shared_data.h"


namespace phoenix {
namespace framework {


// 障碍物 { 方位: 正后方, 距离: - 80.0F/3.6F * 2.1m, 速度: 5km/h, t_gap: 2.1, ttc:  }
// 正后方障碍物 (动态)ttc =100, abs_t_gap=2.1
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_01() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = - 80.0F/3.6F * 2.1F;
    obj.y = -1.6F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 5.0F/3.6F;
    obj.v_x = 50.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}
// 障碍物 { 方位: 左后方, 距离: - 80.0F/3.6F * 2.1m, 速度: 5km/h, t_gap: 2.1, ttc: 100 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_02() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = - 80.0F/3.6F * 2.1F;
    obj.y = 3.5F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 5.0F/3.6F;
    obj.v_x = 50.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}
// 障碍物 { 方位: 正前方, 距离: 13m, 速度: 0km/h, t_gap: , ttc: 100 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_03() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = 13.0F;
    obj.y = 4.50F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 50.0F/3.6F;
    obj.v_x = 50.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 20.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 紧贴, 距离: 13m, 速度: 0km/h, t_gap: , ttc: }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_04() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = 13.0F;
    obj.y = 0.0F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 50.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 20.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}


// 障碍物 { 方位: 右前, 距离: 30m, 速度: 60km/h, t_gap:1.35 , ttc:5.4 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_11() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = 30.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 43m, 速度: 60km/h, t_gap:1.935 , ttc:7.74 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_12() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =43.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 45m, 速度: 60km/h, t_gap:2.025 , ttc:8.1 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_13() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = 45.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 50m, 速度: 60km/h, t_gap:2.25 , ttc:9 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_14() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =50.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 54m, 速度: 60km/h, t_gap:2.43 , ttc:9.72 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_15() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =54.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 57m, 速度: 60km/h, t_gap:2.565 , ttc:10.26 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_16() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =57.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 65m, 速度: 60km/h, t_gap:2.925 , ttc:11.7 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_17() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =65.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前,  静态， 距离: 225m, 速度: 60km/h, t_gap:2.925 , ttc:11.7 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_18() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =225.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = false;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前， 距离: -15m, 速度: 27km/h, t_gap:2.925 , ttc:11.7 }
// 自车 { 速度27km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_19() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -13.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 27.0F/3.6F;
    obj.v_x = 27.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 27.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 25m, 速度: 60km/h, t_gap:1.125 , ttc:4.5 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_21() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =25.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 33m, 速度: 60km/h, t_gap:1.485 , ttc:5.94 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_22() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =33.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 35m, 速度: 60km/h, t_gap:1.575 , ttc:6.3 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_23() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =35.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 40m, 速度: 60km/h, t_gap:1.8 , ttc:7.2 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_24() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =40.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}


// 障碍物 { 方位: 右前, 距离: 43m, 速度: 60km/h, t_gap:1.935 , ttc:7.74 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_25() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =43.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 45m, 速度: 60km/h, t_gap:2.025 , ttc:8.1 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_26() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =45.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前, 距离: 50m, 速度: 60km/h, t_gap:2.25 , ttc:9 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_27() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static Int32_t s_running_count = 0;
  s_running_count++;

  if (s_running_count > 50) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =50.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前,  静态， 距离: 225m, 速度: 60km/h, t_gap: , ttc: }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_28() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =225.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}


// 障碍物 { 方位: 右后, 距离: -30m, 速度: 60km/h, t_gap:1.35 , ttc:5.4 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_31() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -30.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}



// 障碍物 { 方位: 右后, 距离: -10m, 速度: 60km/h, t_gap:0.6 , ttc:0.9 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_30() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -20.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}


// 障碍物 { 方位: 右后, 距离: -43m, 速度: 60km/h, t_gap:1.935 , ttc:7.74 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_32() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -43.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -45m, 速度: 60km/h, t_gap:2.025 , ttc:8.1 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_33() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -45.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -50m, 速度: 60km/h, t_gap:2.25 , ttc:9 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_34() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -50.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -54m, 速度: 60km/h, t_gap:2.43 , ttc:9.72 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_35() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -54.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -57m, 速度: 60km/h, t_gap:2.565 , ttc:10.26 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_36() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =-57.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -65m, 速度: 60km/h, t_gap:2.925 , ttc:11.7 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_37() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -65.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右前,  静态， 距离: -105m, 速度: 60km/h, t_gap:2.925 , ttc:11.7 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_38() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -105.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = false;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 60.0F/3.6F;
    obj.v_x = 60.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -30m, 速度: 100km/h, t_gap:1.35 , ttc:5.4 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_41() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -30.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}


// 障碍物 { 方位: 右后, 距离: -43m, 速度: 100km/h, t_gap:1.935 , ttc:7.74 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_42() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -43.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -45m, 速度: 100km/h, t_gap:2.025 , ttc:8.1 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_43() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -45.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -50m, 速度: 100km/h, t_gap:2.25 , ttc:9 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_44() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -50.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -54m, 速度: 100km/h, t_gap:2.43 , ttc:9.72 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_45() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -54.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -57m, 速度: 100km/h, t_gap:2.565 , ttc:10.26 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_46() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x =-57.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -65m, 速度: 100km/h, t_gap:2.925 , ttc:11.7 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_47() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -65.0F;
    obj.y = -3.1F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 100.0F/3.6F;
    obj.v_x = 100.0F/3.6F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -25m, 速度: 100km/h, t_gap:1.125 , ttc:4.5 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_51() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 2;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];
 if (i==0) {
      obj.id = 0;
      obj.x = -25.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
 } else {
   obj.id = 1;
   obj.x =25.0F;
   obj.y = -3.1F;
   obj.obb.x = obj.x + 1.0F;
   obj.obb.y = obj.y;
   obj.obb.heading = 0.0F;
   obj.obb.half_length = 2.0F;
   obj.obb.half_width = 1.0F;
   obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
   obj.dynamic = true;
   obj.confidence = 90;
   obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
   obj.v = 60.0F/3.6F;
   obj.v_x = 60.0F/3.6F;
   obj.v_y = 0.0F;
 }
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -33m, 速度: 100km/h, t_gap:1.485 , ttc:5.94 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_52() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x =-33.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 60.0F/3.6F;
      obj.v_x = 60.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -35m, 速度: 100km/h, t_gap:1.575 , ttc:6.3 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_53() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -35.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -40m, 速度: 100km/h, t_gap:1.8 , ttc:7.2 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_54() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -40.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}


// 障碍物 { 方位: 右后, 距离: -43m, 速度: 100km/h, t_gap:1.935 , ttc:7.74 }
// 自车 { 速度80km/h }
// 预期结果 { 不允许变道 }
void ChangingLaneDebug::Debug_Case_55() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -43.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -45m, 速度: 100km/h, t_gap:2.025 , ttc:8.1 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_56() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static int count = 0;
  count ++;
  if (count > 60) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -45.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后, 距离: -50m, 速度: 100km/h, t_gap:2.25 , ttc:9 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_57() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static Int32_t s_running_count = 0;
  s_running_count++;

  if (s_running_count > 50) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -50.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 右后,  静态 距离: -105m, 速度: 100km/h, t_gap:2.25 , ttc:9 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_58() {
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.25F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.25F;
    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);

  static Int32_t s_running_count = 0;
  s_running_count++;

  if (s_running_count > 50) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -50.0F;
      obj.y = -3.1F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = false;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    shared_data->SetOutsideObstacleList(obstacle_list_);
  }



  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 障碍物 { 方位: 正后,  静态 距离: -13m, 速度: 100km/h, t_gap:1.35 , ttc:5.4 }
// 自车 { 速度80km/h }
// 预期结果 { 允许变道 }
void ChangingLaneDebug::Debug_Case_61(){
  SharedData* shared_data = SharedData::instance();

  /// 设置车道线
  lane_mark_camera_list_.lane_mark_num = 4;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.39F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -2.2109F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 5.14F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -5.97F;

    } else {
      // nothing to do
    }
  }
  // Update message head
  lane_mark_camera_list_.msg_head.valid = true;
  lane_mark_camera_list_.msg_head.UpdateSequenceNum();
  lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetLaneMarkCameraList(lane_mark_camera_list_);
  /// 设置底盘数据
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
  chassis_.v = 80.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();
  // Set to shared data
  shared_data->SetChassis(chassis_);
  static Int32_t s_running_count = 0;
  s_running_count++;

  if (s_running_count > 50) {
    /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = -30.0F;
      obj.y = 0.0F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = 0.0F;
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = false;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = 100.0F/3.6F;
      obj.v_x = 100.0F/3.6F;
      obj.v_y = 0.0F;
    }
    // Update message head
    obstacle_list_.msg_head.valid = true;
    obstacle_list_.msg_head.UpdateSequenceNum();
    obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
    //shared_data->SetOutsideObstacleList(obstacle_list_);
  }




}

void ChangingLaneDebug::Debug() {
  /// 注意: 障碍物模式设置为->使用外部障碍物

  // 障碍物 { 方位: 正后方, 距离: - 80.0F/3.6F * 2.1m, 速度: 5km/h, t_gap: 2.1, ttc: 100 }
  // 自车 { 速度80km/h }
  // 预期结果 { 允许右变道 }
  // Debug_Case_01();
  // 障碍物 { 方位: 左后方, 距离: - 80.0F/3.6F * 2.1m, 速度: 5km/h, t_gap: 2.1, ttc:  }
  // 自车 { 速度80km/h }
  // 预期结果 { 允许右变道 }
  // Debug_Case_02();
   //Debug_Case_03();
  // Debug_Case_04();
   //Debug_Case_11();
   //Debug_Case_12();
   //Debug_Case_13();
   //Debug_Case_14();
   //Debug_Case_15();
    // Debug_Case_16();
   //Debug_Case_17();
  //Debug_Case_18();
  // Debug_Case_19();
  // Debug_Case_21();
   //Debug_Case_22();
   //Debug_Case_23();
   //Debug_Case_24();
  // Debug_Case_25();
   //Debug_Case_26();
  // Debug_Case_27();
    // Debug_Case_28();
    // Debug_Case_30();
   //  Debug_Case_31();
     //Debug_Case_32();
     //Debug_Case_33();
    // Debug_Case_34();
   // Debug_Case_35();
     //Debug_Case_36();
    //Debug_Case_37();
    // Debug_Case_38();
   //Debug_Case_41();
   //Debug_Case_42();
   //Debug_Case_43();
  // Debug_Case_44();
     //Debug_Case_45();
   //Debug_Case_46();
   //Debug_Case_47();
  //  Debug_Case_51();
  Debug_Case_52();
  //Debug_Case_53();
 // Debug_Case_54();
   // Debug_Case_55();
  //Debug_Case_56();
 // Debug_Case_57();
// Debug_Case_58();
  // Debug_Case_61();

}


}  // namespace framework
}  // namespace framework
