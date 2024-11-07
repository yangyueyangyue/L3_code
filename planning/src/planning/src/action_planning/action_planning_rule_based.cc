
#include "action_planning/action_planning_rule_based.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_RULE_BASE_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningRuleBased::ActionPlanningRuleBased() {
  planner_lane_keeping_.set_sequence_id(0);
  planner_changing_lane_.set_sequence_id(1);
  planner_bypassing_.set_sequence_id(2);
  planner_acc_following_.set_sequence_id(3);
  planner_pulling_over_.set_sequence_id(4);
  planner_emergency_stopping_.set_sequence_id(5);
  planner_scene_story_.set_sequence_id(6);
  planner_fallback_.set_sequence_id(7);

  planner_lane_keeping_.set_next_action_planner(&planner_changing_lane_);
  planner_changing_lane_.set_next_action_planner(&planner_bypassing_);
  planner_bypassing_.set_next_action_planner(&planner_acc_following_);
  planner_acc_following_.set_next_action_planner(&planner_pulling_over_);
  planner_pulling_over_.set_next_action_planner(&planner_emergency_stopping_);
  planner_emergency_stopping_.set_next_action_planner(&planner_scene_story_);
  planner_scene_story_.set_next_action_planner(&planner_fallback_);
  planner_fallback_.set_next_action_planner(Nullptr_t);

  planner_lane_keeping_.set_shared_data(&act_planning_shared_data_);
  planner_changing_lane_.set_shared_data(&act_planning_shared_data_);
  planner_bypassing_.set_shared_data(&act_planning_shared_data_);
  planner_acc_following_.set_shared_data(&act_planning_shared_data_);
  planner_pulling_over_.set_shared_data(&act_planning_shared_data_);
  planner_emergency_stopping_.set_shared_data(&act_planning_shared_data_);
  planner_scene_story_.set_shared_data(&act_planning_shared_data_);
  planner_fallback_.set_shared_data(&act_planning_shared_data_);

  action_planner_ = &planner_lane_keeping_;

  adas_start_status_pre = false;
}

ActionPlanningRuleBased::~ActionPlanningRuleBased() {
  // noting to do
}

void ActionPlanningRuleBased::Configurate(const ActionPlanningConfig& conf) {
  act_planning_shared_data_.config = conf;
}

bool ActionPlanningRuleBased::Plan(
    const ActionPlanningDataSource& data_source) {
#if ENABLE_ACTION_PLANNING_RULE_BASE_TRACE
  std::cout << "######### Action planning (Rule Based) >>>>>>>>>" << std::endl;
#endif

  act_planning_shared_data_.UpdateTimerList();
  act_planning_shared_data_.event_reporting_list.Clear();

  if (Nullptr_t == data_source.driving_map) {
    LOG_ERR << "Invalid driving map.";
    return false;
  }

  if ((ad_msg::VEH_DRIVING_MODE_INVALID ==
       act_planning_shared_data_.chassis.driving_mode) ||
      (ad_msg::VEH_DRIVING_MODE_MANUAL ==
       act_planning_shared_data_.chassis.driving_mode)) {
    // 之前处于手动状态
    if ((ad_msg::VEH_DRIVING_MODE_INVALID !=
         data_source.chassis->driving_mode) &&
        (ad_msg::VEH_DRIVING_MODE_MANUAL !=
         data_source.chassis->driving_mode)) {
      // 进入自动模式
      act_planning_shared_data_.AddEventReporting(
            ActionPlanningSharedData::EVENT_TYPE_START_ROBOTIC);
    }
  } else {
    // 之前处于自动状态
    if ((ad_msg::VEH_DRIVING_MODE_INVALID ==
         data_source.chassis->driving_mode) ||
        (ad_msg::VEH_DRIVING_MODE_MANUAL ==
         data_source.chassis->driving_mode)) {
      // 退出自动模式
      act_planning_shared_data_.AddEventReporting(
            ActionPlanningSharedData::EVENT_TYPE_STOP_ROBOTIC);
    }
  }
  act_planning_shared_data_.chassis = *(data_source.chassis);

  result_of_planning_.driving_mode = ad_msg::VEH_DRIVING_MODE_INVALID;
  result_of_planning_.gear = ad_msg::VEH_GEAR_INVALID;

  switch (data_source.planning_settings->enable_aeb) {
  case (1):
    result_of_planning_.enable_aeb = false;
    break;
  case (2):
    result_of_planning_.enable_aeb = true;
    break;
  default:
    // 不更新
    break;
  }
  switch (data_source.planning_settings->enable_acc) {
  case (1):
    result_of_planning_.enable_acc = false;
    break;
  case (2):
    result_of_planning_.enable_acc = true;
    break;
  default:
    // 不更新
    break;
  }
  switch (data_source.planning_settings->enable_lka) {
  case (1):
    result_of_planning_.enable_lka = false;
    break;
  case (2):
    result_of_planning_.enable_lka = true;
    break;
  default:
    // 不更新
    break;
  }

  switch (data_source.planning_settings->enable_alc) {
  case (1):
    result_of_planning_.enable_alc = false;
    break;
  case (2):
    result_of_planning_.enable_alc = true;
    break;
  default:
    // 不更新
    break;
  }
  switch (data_source.planning_settings->enable_isl) {
  case (1):
    result_of_planning_.enable_isl = false;
    break;
  case (2):
    result_of_planning_.enable_isl = true;
    break;
  default:
    // 不更新
    break;
  }
  switch (data_source.planning_settings->enable_ngp) {
  case (1):
    result_of_planning_.enable_ngp = false;
    break;
  case (2):
    result_of_planning_.enable_ngp = true;
    break;
  default:
    // 不更新
    break;
  }

  switch (data_source.planning_settings->enable_fallback) {
  case (1):
    result_of_planning_.enable_fallback = false;
    break;
  case (2):
    result_of_planning_.enable_fallback = true;
    break;
  default:
    // 不更新
    break;
  }
  
  switch (data_source.planning_settings->enable_pcc) {
  case (1):
    result_of_planning_.enable_pcc = false;
    break;
  case (2):
    result_of_planning_.enable_pcc = true;
    break;
  default:
    // 不更新
    break;
  }

  if (result_of_planning_.enable_ngp) {
    Int32_t driving_map_type = data_source.driving_map->GetDrivingMapType();
    Int32_t location_status = data_source.driving_map->GetGnss().gnss_status;
    if ((driv_map::DRIVING_MAP_TYPE_HD_MAP == driving_map_type ||
         driv_map::DRIVING_MAP_TYPE_MIXED_HD_MAP == driving_map_type) &&
        ((ad_msg::Gnss::STATUS_GOOD == location_status) ||
         (ad_msg::Gnss::STATUS_CONVERGING == location_status))) {
      result_of_planning_.enable_ngp = true;
    } else {
      result_of_planning_.enable_ngp = false;
    }
  }

  result_of_planning_.v_setting =
      data_source.planning_settings->target_velocity;

  result_of_planning_.a_setting = data_source.planning_settings->target_acc;
  
  result_of_planning_.time_gap_setting =
      data_source.planning_settings->target_time_gap;

  result_of_planning_.level_setting = 
      data_source.planning_settings->target_fallback_level;
  switch (data_source.planning_settings->start_adas) {
  case (1):
  	LOG_INFO(5) << "### HMI request to stop ADAS.";
    break;
  case (2):
  	LOG_INFO(5) << "### HMI request to start ADAS.";
  	break;
  default:
    break;
  }

  switch (data_source.special_chassis_info->start_adas) {
  case (1):
  	LOG_INFO(5) << "### Chassis request to stop ADAS.";
    break;
  case (2):
  	LOG_INFO(5) << "### Chassis request to start ADAS.";
  	break;
  default:
    break;
  }

  // 进入自动驾驶
  if((2 == data_source.planning_settings->start_adas) &&
     (ad_msg::VEH_DRIVING_MODE_MANUAL == data_source.chassis->driving_mode)) {
    result_of_planning_.enable_adas = true;
    result_of_planning_.driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;

    LOG_INFO(5) << "### Planning request to start ADAS.";
  }

  // 退出自动驾驶
  if((1 == data_source.planning_settings->start_adas) &&
     (ad_msg::VEH_DRIVING_MODE_INVALID != data_source.chassis->driving_mode) &&
     (ad_msg::VEH_DRIVING_MODE_MANUAL != data_source.chassis->driving_mode)) {
    result_of_planning_.enable_adas = false;
    result_of_planning_.driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;

    LOG_INFO(5) << "### Planning request to stop ADAS.";
  }

  if((ad_msg::VEH_DRIVING_MODE_INVALID != data_source.chassis->driving_mode) &&
     (ad_msg::VEH_DRIVING_MODE_MANUAL != data_source.chassis->driving_mode) &&
     (ad_msg::VEH_GEAR_INVALID != data_source.chassis->gear) &&
     (ad_msg::VEH_GEAR_D != data_source.chassis->gear) &&
     (ad_msg::VEH_GEAR_R != data_source.chassis->gear)) {
    result_of_planning_.gear = ad_msg::VEH_GEAR_D;
  }

  // 更新车道风险信息
  UpdateLaneInfo(data_source);

  // 进入职责链模式
  action_planner_->Proceed(data_source, &result_of_planning_);
  
  return true;
}

Int32_t ActionPlanningRuleBased::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  return (act_planning_shared_data_.GetEventReporting(num, events));
}

void ActionPlanningRuleBased::UpdateLaneInfo(
    const ActionPlanningDataSource &data_source) {
  for (Int32_t i = 0; i < ActionPlanningSharedData::LANE_IDX_MAX; ++i) {
    act_planning_shared_data_.lane_info_list[i].Clear();
  }

  ActionPlanningSharedData::LaneInfo& curr_lane =
      act_planning_shared_data_.lane_info_list[ActionPlanningSharedData::LANE_IDX_CURR];
  ActionPlanningSharedData::LaneInfo& left_lane =
      act_planning_shared_data_.lane_info_list[ActionPlanningSharedData::LANE_IDX_LEFT_1];
  ActionPlanningSharedData::LaneInfo& right_lane =
      act_planning_shared_data_.lane_info_list[ActionPlanningSharedData::LANE_IDX_RIGHT_1];

  // 遍历所有参考线
  Int32_t ref_lines_num = data_source.driving_map->GetReferenceLinesNum();
  for (Int32_t i = 0; i < ref_lines_num; i++) {
    Int32_t neighbor_flag =
        data_source.driving_map->GetReferenceLineNeighborFlag(i);

    /// TODO: 需要考虑多车道合流/分流的情况
    // 只对左中右三个车道进行风险分析
    if (0 == neighbor_flag) {
      if (lane_risk_analyser_.AnalyseRisksOfLane(
            data_source, i, &(curr_lane.risk_info))) {
        curr_lane.valid = true;
      }
    } else if (-1 == neighbor_flag) {
      if (lane_risk_analyser_.AnalyseRisksOfLane(
            data_source, i, &(right_lane.risk_info))) {
        right_lane.valid = true;
      }
    } else if (1 == neighbor_flag) {
      if (lane_risk_analyser_.AnalyseRisksOfLane(
            data_source, i, &(left_lane.risk_info))) {
        left_lane.valid = true;
      }
    } else {
      // nothing to do
    }
  }
}


} // namespace planning
} // namespace phoenix

