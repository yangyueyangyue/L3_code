#ifndef PHOENIX_FRAMEWORK_PLANNING_DEBUG_H_
#define PHOENIX_FRAMEWORK_PLANNING_DEBUG_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "common/message.h"

#include "debug/changing_lane_debug.h"
#include "debug/lane_mark_filter_debug.h"


namespace phoenix {
namespace framework {


class PlanningDebug {
public:
  void Debug();

private:
  void Debug_LocalCase();

  void Debug_Case_SetChassis();
  void Debug_Case_SetLaneMark();
  void Debug_Case_SetObstacle();
  void Debug_Case_SetSceneStory();

  void Debug_Case_01();
  void Debug_Case_02();
  void Debug_Case_03();
  void Debug_Case_04();
  void Debug_Case_05();
  void Debug_Case_06();
  void Debug_Case_07();
  void Debug_Case_08();
  void Debug_Case_09();
  void Debug_Case_10();
  void Debug_Case_11();
  void Debug_Case_12();
  void Debug_Case_13();

private:
  // hmi settings
  ad_msg::PlanningSettings local_planning_settings_;
  // chassis
  ad_msg::Chassis chassis_;
  // GNSS信息
  ad_msg::Gnss gnss_;
  // Lane mark
  ad_msg::LaneMarkCameraList lane_mark_camera_list_;
  // obstacles
  ad_msg::ObstacleList obstacle_list_;
  // Scene storys
  ad_msg::SceneStoryList scene_story_list_;

  // Trajectory planning result
  planning::TrajectoryPlanningResult trajectory_planning_result_;

  // debug changing lane
  ChangingLaneDebug changing_lane_debug_;
  // debug lane mark filter
  LaneMarkFilterDebug lane_mark_filter_debug_;
};


}  // namespace framework
}  // namespace framework


#endif  // PHOENIX_FRAMEWORK_PLANNING_DEBUG_H_


