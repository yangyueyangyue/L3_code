/*******************************************************************************
 * @file pull_over_status.cc
 * @author sip (sip@goyu-ai.com)
 * @brief 靠边停车状态
 * @version 0.1
 * @date 2021-04-29
 * 
 * All Rights Reserved. Copyright(C) 2021 Kotei Co., Ltd.
 * 
 * @par 更新记录:
 * 更新内容：
 * 1.
 * 更新作者:
 * 更新时间：
 * 更新版本：
*******************************************************************************/
#include "action_planning/action_planning_scene_story.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_SCENE_STORY_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningSceneStory::ActionPlanningSceneStory(){
  // nothing to do
}

ActionPlanningSceneStory::~ActionPlanningSceneStory(){
  // nothing to do
}

void ActionPlanningSceneStory::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_SCENE_STORY_TRACE
  std::cout << "######### Action planning (Scene Story) >>>>>>>>>" << std::endl;
#endif

  if (Nullptr_t == act_planning_shared_data_) {
    LOG_ERR << "Invalid data.";
    return;
  }

  const common::StaticVector<ad_msg::SceneStory,
      ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>& story_list =
      data_source.driving_map->GetSceneStoryList();

  for (Int32_t i = 0; i < story_list.Size(); ++i) {
    const ad_msg::SceneStory& story = story_list[i];

    if (IsSatisfyingStoryCondition(story)) {
      const ad_msg::SceneStoryAction& action = story.action;

      if (ad_msg::VEH_GEAR_INVALID != action.gear) {
        planning_result->gear = action.gear;
      }

      if (ad_msg::VEH_TURN_LAMP_INVALID != action.turn_lamp) {
        planning_result->turn_lamp = action.turn_lamp;
      }
    }
  }
}

bool ActionPlanningSceneStory::IsSatisfyingStoryCondition(
    const ad_msg::SceneStory& story) {
  const ad_msg::Chassis& chassis = act_planning_shared_data_->chassis;
  const ad_msg::SceneStoryCondition& condition = story.condition;

  if (!story.action.vaild) {
    // 没有行为，返回不满足条件
    return false;
  }


  if (!condition.vaild) {
    // 没有设置条件，返回满足条件
    return true;
  }

  if (condition.valid_area) {
    if (!story.area.IsInArea()) {
      return false;
    }
  }

  if (condition.valid_speed_high) {
    if (chassis.v > condition.speed_high) {
      return false;
    }
  }
  if (condition.valid_speed_low) {
    if (chassis.v < condition.speed_low) {
      return false;
    }
  }
  if (ad_msg::VEH_GEAR_INVALID != condition.gear) {
    if (chassis.gear != condition.gear) {
      return false;
    }
  }

  return (true);
}


}  // namespace planning
}  // namespace phoenix

