//
#include "debug/lane_mark_filter_debug.h"

#include "utils/macros.h"
#include "utils/gps_tools.h"
#include "geometry/geometry_utils.h"
#include "communication/shared_data.h"


namespace phoenix {
namespace framework {


// 车道线左右交替跳跃
void LaneMarkFilterDebug::Debug_Case_01() {
  SharedData* shared_data = SharedData::instance();

  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode = 0;
  if (0 == (s_running_count % 5)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }

  /// 设置车道线
  if (0 == s_mode) {
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
  } else {
    lane_mark_camera_list_.lane_mark_num = 3;
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

        lane_mark.c0 = 1.75F+0.5F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F+0.5F;
      } else if (2 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -5.25F+0.5F;
      } else {
        // nothing to do
      }
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
}

// 变道
void LaneMarkFilterDebug::Debug_Case_02() {
  SharedData* shared_data = SharedData::instance();

  static Int32_t s_running_count = 0;
  s_running_count++;

//  static Int32_t s_mode = 0;
//  if (s_running_count >= 15) {
//    s_running_count = 15;
//    s_mode = 1;
//  }

  static Int32_t s_mode = 0;
  if (0 == (s_running_count % 5)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }
  //s_mode = 0;

  /// 设置车道线
  if (0 == s_mode) {
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
      lane_mark.c1 = -0.1F;
      lane_mark.c2 = 0.0F;
      lane_mark.c3 = 0.0F;

      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.75F-1.25F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F-1.25F;
      } else if (2 == i) {
        lane_mark.id = 2;

        lane_mark.c0 = 5.25F-1.25F;
      } else if (3 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -5.25F-1.25F;
      } else {
        // nothing to do
      }
    }
  } else {
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
      lane_mark.c1 = -0.1F;
      lane_mark.c2 = 0.0F;
      lane_mark.c3 = 0.0F;

      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.75F+1.25F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F+1.25F;
      } else if (2 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -5.25F+1.25F;
      } else if (3 == i) {
        lane_mark.id = 2;

        lane_mark.c0 = 5.25F+1.25F;
      } else {
        // nothing to do
      }
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
  chassis_.v = 40.0F/3.6F;
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


void LaneMarkFilterDebug::Debug() {
  // 车道线左右交替跳跃
  // Debug_Case_01();
  // 变道
  //Debug_Case_02();
}


}  // namespace framework
}  // namespace framework
