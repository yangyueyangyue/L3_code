//
#include "debug/planning_debug.h"

#include "utils/macros.h"
#include "utils/gps_tools.h"
#include "geometry/geometry_utils.h"
#include "communication/shared_data.h"


namespace phoenix {
namespace framework {


// 设置底盘数据
void PlanningDebug::Debug_Case_SetChassis() {
  SharedData* shared_data = SharedData::instance();
  static float count = 0.0F;
  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
 // chassis_.steering_wheel_angle = common::com_deg2rad(0.0F);
  chassis_.v_valid = 1;
 // chassis_.v = (40.0F+count)/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate = common::com_deg2rad(0.0F);
  chassis_.gear = ad_msg::VEH_GEAR_D;
  chassis_.trailer_status = ad_msg::VEH_TRAILER_STATUS_CONNECTED;
count++;
  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 设置车道线
void PlanningDebug::Debug_Case_SetLaneMark() {
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
    lane_mark.c2 = 0.015F;
    lane_mark.c3 = 0.0F;

    Float32_t offset = 0.0F;
    if (0 == i) {
      lane_mark.id = 1;
      lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;

      lane_mark.c0 = 1.75F + offset;
    } else if (1 == i) {
      lane_mark.id = -1;
      lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;

      lane_mark.c0 = -1.75F + offset;
    } else if (2 == i) {
      lane_mark.id = 2;
      lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;

      lane_mark.c0 = 5.25F + offset;
    } else if (3 == i) {
      lane_mark.id = -2;
      lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;

      lane_mark.c0 = -5.25F + offset;// + 1.0F;
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
}

// 设置障碍物数据
void PlanningDebug::Debug_Case_SetObstacle() {
  SharedData* shared_data = SharedData::instance();

  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;

  Float32_t obj_v = 80.0F/3.6F;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = 5.0F;//-0.5F * obj_v;
    obj.y =2.0F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = common::com_deg2rad(0.0F);
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = obj_v;
    obj.v_x = obj_v;
    obj.v_y = 0.0F;
  }

//  obstacle_list_.obstacle_num = 2;
//  ad_msg::Obstacle& obj = obstacle_list_.obstacles[obstacle_list_.obstacle_num - 1];
//  obj.id = 0;
//  obj.x = 40.0F;//-0.5F * obj_v;
//  obj.y = 0.0F;
//  obj.obb.x = obj.x + 1.0F;
//  obj.obb.y = obj.y;
//  obj.obb.heading = -common::com_deg2rad(180.0F);
//  obj.obb.half_length = 2.0F;
//  obj.obb.half_width = 1.0F;
//  obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
//  obj.dynamic = true;
//  obj.confidence = 90;
//  obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
//  obj.v = obj_v;
//  obj.v_x = obj_v;
//  obj.v_y = 0.0F;
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);
}

// 设置地理围栏数据
void PlanningDebug::Debug_Case_SetSceneStory() {
  SharedData* shared_data = SharedData::instance();

  gnss_.msg_head.valid = true;
  gnss_.msg_head.UpdateSequenceNum();
  gnss_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetGnss(gnss_);

  Int32_t story_list_idx = 0;
  scene_story_list_.Clear();

  // 在弯道中
  if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
    ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
    story_list_idx++;

    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE;
    story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
    story.area.area_type_01.valid = true;
    story.area.area_type_01.distance = 0.0F;
  }

  // 在隧道内
  if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
    ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
    story_list_idx++;

    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL;
    story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
    story.area.area_type_01.valid = true;
    story.area.area_type_01.distance = 0.0F;
  }

  // 匝道
  if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
    ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
    story_list_idx++;

    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP;
    story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
    story.area.area_type_01.valid = true;
    story.area.area_type_01.distance = 0.0F;
  }

  // 路过匝道
  if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
    ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
    story_list_idx++;

    story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
    story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
    story.area.area_type_01.valid = true;
    story.area.area_type_01.distance = 0.0F;
  }

  scene_story_list_.story_num = story_list_idx;
  scene_story_list_.msg_head.valid = true;
  scene_story_list_.msg_head.UpdateSequenceNum();
  scene_story_list_.msg_head.timestamp = common::GetClockNowMs();

  shared_data->SetSceneStoryList(scene_story_list_);
}

// 车道线交替出现/消失
void PlanningDebug::Debug_Case_01() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_lane_mark_num = 4;
  if (0 == (s_running_count % 80)) {
    s_lane_mark_num += 4;
    if (s_lane_mark_num > 4) {
      s_lane_mark_num = 0;
    }
  }
  lane_mark_camera_list_.lane_mark_num = s_lane_mark_num;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
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

      lane_mark.c0 = 2.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -0.75F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 6.5F;
    } else if (3 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -6.5F;
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


  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 30.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= 0;
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 参考线左右交替跳动
void PlanningDebug::Debug_Case_02() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode = 0;
  if (0 == (s_running_count % 20)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }
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

    Float32_t offset = -1.6F;
    if (0 == s_mode) {
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.75F + offset;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F + offset;
      } else if (2 == i) {
        lane_mark.id = 2;

        lane_mark.c0 = 1.75F+3.5F + offset;
      } else {
        // nothing to do
      }
    } else {
      Float32_t offset = 1.6F;
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.75F + offset;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F + offset;
      } else if (2 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -1.75F-3.5F + offset;
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


  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 30.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= 0;
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 车身倾斜，自动驾驶交替退出/进入
void PlanningDebug::Debug_Case_03() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode = 0;
  if (0 == (s_running_count % 50)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }

  lane_mark_camera_list_.lane_mark_num = 3;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
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

      lane_mark.c0 = 2.75F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -0.75F;
    } else if (2 == i) {
      lane_mark.id = -2;

      lane_mark.c0 = -4.5F;
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

  // For testing
  if (0 == s_mode) {
    chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;
    chassis_.eps_status = ad_msg::VEH_EPS_STATUS_MANUAL;
  } else {
    chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
    chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  }

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 30.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= 0;
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 车身倾斜，自动驾驶交替退出/进入, 参考线变更
void PlanningDebug::Debug_Case_04() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode_0 = 0;
  if (0 == (s_running_count % 50)) {
    s_mode_0 += 1;
    if (s_mode_0 > 1) {
      s_mode_0 = 0;
    }
  }
  static Int32_t s_mode_1 = 0;
  static Int32_t s_mode_count_1 = 0;
  s_mode_count_1++;
  if (0 == s_mode_0) {
    s_mode_1 = 0;
    s_mode_count_1 = 0;
  }
  if (s_mode_count_1 > 20) {
    s_mode_1 = 1;
  }

  lane_mark_camera_list_.lane_mark_num = 3;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = -0.1F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == s_mode_1) {
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 0.25F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -3.25F;
      } else if (2 == i) {
        lane_mark.id = 2;

        lane_mark.c0 = 3.75F;
      } else {
        // nothing to do
      }
    } else {
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 2.95F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -0.55F;
      } else if (2 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -4.1F;
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


  // For testing
  if (0 == s_mode_0) {
    chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;
    chassis_.eps_status = ad_msg::VEH_EPS_STATUS_MANUAL;
  } else {
    chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
    chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
  }

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 60.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= 0;
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 变道中止, 车身倾斜
void PlanningDebug::Debug_Case_05() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode_0 = 0;
  if (0 == (s_running_count % 50)) {
    s_mode_0 += 1;
    if (s_mode_0 > 1) {
      s_mode_0 = 0;
    }
  }
  static Int32_t s_mode_1 = 0;
  static Int32_t s_mode_count_1 = 0;
  s_mode_count_1++;
  if (0 == s_mode_0) {
    s_mode_1 = 0;
    s_mode_count_1 = 0;
  }
  if (s_mode_count_1 > 20) {
    s_mode_1 = 1;
  }

  local_planning_settings_.changing_lane_req = 0;
  local_planning_settings_.enable_alc = 2;
  if (0 == s_mode_0) {
    local_planning_settings_.changing_lane_req = 1;
  }
  if (1 == s_mode_1) {
    local_planning_settings_.changing_lane_req = 3;
  }
  shared_data->SetLocalPlanningSettings(local_planning_settings_);

  lane_mark_camera_list_.lane_mark_num = 3;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
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

      lane_mark.c0 = 0.25F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -3.25F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 3.75F;
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

  // config_.obj_filter_config.using_outside_obj_list = true;
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = 40.0F;
    obj.y = -6.0F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = 0.0F;
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_BICYCLE;
    obj.dynamic = false;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = 0.0F;
    obj.v_x = 0.0F;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 60.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= 0;
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 变道中止, 车身平行
void PlanningDebug::Debug_Case_06() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode_0 = 0;
  if (0 == (s_running_count % 50)) {
    s_mode_0 += 1;
    if (s_mode_0 > 1) {
      s_mode_0 = 0;
    }
  }
  static Int32_t s_mode_1 = 0;
  static Int32_t s_mode_count_1 = 0;
  s_mode_count_1++;
  if (0 == s_mode_0) {
    s_mode_1 = 0;
    s_mode_count_1 = 0;
  }
  if (s_mode_count_1 > 20) {
    s_mode_1 = 1;
  }

  local_planning_settings_.changing_lane_req = 0;
  local_planning_settings_.enable_alc = 2;
  if (0 == s_mode_0) {
    local_planning_settings_.changing_lane_req = 1;
  }
  if (1 == s_mode_1) {
    local_planning_settings_.changing_lane_req = 3;
  }
  shared_data->SetLocalPlanningSettings(local_planning_settings_);

  lane_mark_camera_list_.lane_mark_num = 3;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
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

      lane_mark.c0 = 0.25F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -3.25F;
    } else if (2 == i) {
      lane_mark.id = 2;

      lane_mark.c0 = 3.75F;
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


  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 60.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= 0;
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 参考线跳动
void PlanningDebug::Debug_Case_07() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode = 0;
  if (0 == (s_running_count % 15)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }
  lane_mark_camera_list_.lane_mark_num = 2;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0F;
    lane_mark.c3 = 0.0F;

    if (0 == s_mode) {
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.95F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.55F;
      } else {
        // nothing to do
      }
    } else {
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.67F;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.83F;
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


  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = 0;
  chassis_.v_valid = 1;
  chassis_.v = 30.0F/3.6F;
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

// 固定的车道线
void PlanningDebug::Debug_Case_08() {
  SharedData* shared_data = SharedData::instance();

  lane_mark_camera_list_.lane_mark_num = 2;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    lane_mark.mark_width = 0.1;
    lane_mark.quality = 3;
    lane_mark.view_range_valid = true;
    lane_mark.view_range_start = 0.0F;
    lane_mark.view_range_end = 80.0F;

    lane_mark.c0 = 0.0F;
    lane_mark.c1 = 0.0F;
    lane_mark.c2 = 0.0002F;
    lane_mark.c3 = 0.000001F;

    if (0 == i) {
      lane_mark.id = 1;

      lane_mark.c0 = 1.5F+0.252F;
    } else if (1 == i) {
      lane_mark.id = -1;

      lane_mark.c0 = -1.5F+0.252F;
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
}

// 低速下固定方向的角速度偏差导致车道线扭曲
void PlanningDebug::Debug_Case_09() {
  SharedData* shared_data = SharedData::instance();

  lane_mark_camera_list_.lane_mark_num = 2;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
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


  // For testing
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;

  chassis_.steering_wheel_angle_valid = 1;
  chassis_.steering_wheel_angle = common::com_deg2rad(4.0F);
  chassis_.v_valid = 1;
  chassis_.v = 4.0F/3.6F;
  chassis_.yaw_rate_valid = 1;
  chassis_.yaw_rate= common::com_deg2rad(0.5F);
  chassis_.gear = ad_msg::VEH_GEAR_D;

  // Update message head
  chassis_.msg_head.valid = true;
  chassis_.msg_head.UpdateSequenceNum();
  chassis_.msg_head.timestamp = common::GetClockNowMs();

  // Set to shared data
  shared_data->SetChassis(chassis_);
}

// 相机识别车道线异常检测
void PlanningDebug::Debug_Case_10() {
  SharedData* shared_data = SharedData::instance();

  lane_mark_camera_list_.lane_mark_num = 3;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
    ad_msg::LaneMarkCamera& lane_mark = lane_mark_camera_list_.lane_marks[i];

    lane_mark.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
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

      lane_mark.c0 = 1.75 + 2.0F*1.75F;
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


  // For testing
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

// 轨迹规划变道测试
void PlanningDebug::Debug_Case_11() {
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
  Float32_t obj_v = 80.0F/3.6F;
  obstacle_list_.Clear();
  obstacle_list_.obstacle_num = 1;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

    obj.id = 0;
    obj.x = -1.0F * obj_v;
    obj.y = -3.6F;
    obj.obb.x = obj.x + 1.0F;
    obj.obb.y = obj.y;
    obj.obb.heading = common::com_deg2rad(0.0F);
    obj.obb.half_length = 2.0F;
    obj.obb.half_width = 1.0F;
    obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    obj.dynamic = true;
    obj.confidence = 90;
    obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
    obj.v = obj_v;
    obj.v_x = obj_v;
    obj.v_y = 0.0F;
  }
  // Update message head
  obstacle_list_.msg_head.valid = true;
  obstacle_list_.msg_head.UpdateSequenceNum();
  obstacle_list_.msg_head.timestamp = common::GetClockNowMs();
  shared_data->SetOutsideObstacleList(obstacle_list_);


  /// 设置底盘数据
#if 1
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
#else
  chassis_.driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;
  chassis_.eps_status = ad_msg::VEH_EPS_STATUS_MANUAL;
#endif
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

// 弯道半径 < 500m 不变道
void PlanningDebug::Debug_Case_12() {
  SharedData* shared_data = SharedData::instance();

  // For testing
  static Int32_t s_running_count = 0;
  s_running_count++;

  static Int32_t s_mode = 0;
  if (0 == (s_running_count % 40)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }

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
    if (0 == s_mode) {
      lane_mark.c2 = 0.0011F;
    } else {
      lane_mark.c2 = 0.0F;
    }
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

// 障碍物变道抑制
void PlanningDebug::Debug_Case_13() {
  SharedData* shared_data = SharedData::instance();

  shared_data->GetTrajectoryPlanningResult(&trajectory_planning_result_);
  Int32_t changing_lane_status =
      trajectory_planning_result_.changing_lane_rsp.status();

  // For testing
  static Int32_t s_running_count = 0;
  static Int32_t s_mode = 0;

  if (planning::ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I == changing_lane_status) {
    s_running_count++;
  } else {
    s_running_count = 1;
  }

  if (0 == (s_running_count % 40)) {
    s_mode += 1;
    if (s_mode > 1) {
      s_mode = 0;
    }
  }

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

    if (0 == s_mode) {
      Float32_t offset = -1.5F;
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.75F + offset;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F + offset;
      } else if (2 == i) {
        lane_mark.id = 2;

        lane_mark.c0 = 5.25F + offset;
      } else if (3 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -5.25F + offset;
      } else {
        // nothing to do
      }
    } else {
      Float32_t offset = 1.5F;
      if (0 == i) {
        lane_mark.id = 1;

        lane_mark.c0 = 1.75F + offset;
      } else if (1 == i) {
        lane_mark.id = -1;

        lane_mark.c0 = -1.75F + offset;
      } else if (2 == i) {
        lane_mark.id = 2;

        lane_mark.c0 = 5.25F + offset;
      } else if (3 == i) {
        lane_mark.id = -2;

        lane_mark.c0 = -5.25F + offset;
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


  /// 设置障碍物数据( 注意: 障碍物模式设置为->使用外部障碍物 )
  if (planning::ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II == changing_lane_status) {
    Float32_t obj_v = 80.0F/3.6F;
    obstacle_list_.Clear();
    obstacle_list_.obstacle_num = 1;
    for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      ad_msg::Obstacle& obj = obstacle_list_.obstacles[i];

      obj.id = 0;
      obj.x = 0.0F;//-0.5F * obj_v;
      obj.y = 3.0F;//-3.6F;
      obj.obb.x = obj.x + 1.0F;
      obj.obb.y = obj.y;
      obj.obb.heading = common::com_deg2rad(0.0F);
      obj.obb.half_length = 2.0F;
      obj.obb.half_width = 1.0F;
      obj.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      obj.dynamic = true;
      obj.confidence = 90;
      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      obj.v = obj_v;
      obj.v_x = obj_v;
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

void PlanningDebug::Debug_LocalCase() {
  // 底盘数据
 // Debug_Case_SetChassis();
  // 设置车道线
 // Debug_Case_SetLaneMark();
  // 设置障碍物数据
//  Debug_Case_SetObstacle();
  // 设置地理围栏数据
 // Debug_Case_SetSceneStory();

  // 车道线交替出现/消失
  // Debug_Case_01();
  // 参考线左右交替跳动
  // Debug_Case_02();
  // 车身倾斜，自动驾驶交替退出/进入
  // Debug_Case_03();
  // 车身倾斜，自动驾驶交替退出/进入, 参考线变更
  // Debug_Case_04();
  // 变道中止, 车身倾斜
  // Debug_Case_05();
  // 变道中止, 车身平行
  // Debug_Case_06();
  // 参考线跳动
  // Debug_Case_07();
  // 固定的车道线
  // Debug_Case_08();
  // 低速下固定方向的角速度偏差导致车道线扭曲
  // Debug_Case_09();
  // 相机识别车道线异常检测
  // Debug_Case_10();
  // 轨迹规划变道测试
  // Debug_Case_11();
  // 弯道半径 < 500m 不变道
  // Debug_Case_12();
  // 障碍物变道抑制
  // Debug_Case_13();
}

void PlanningDebug::Debug() {
  // 调试本地用例
 // Debug_LocalCase();
  // 调试变道用例
  //changing_lane_debug_.Debug();
  // 调试车道线滤波
  // lane_mark_filter_debug_.Debug();
}


}  // namespace framework
}  // namespace framework
