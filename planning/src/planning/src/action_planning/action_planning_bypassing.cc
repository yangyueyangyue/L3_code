/*******************************************************************************
 * @file side_pass_status.cc
 * @author sip (sip@goyu-ai.com)
 * @brief 避障状态实现
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
#include "action_planning/action_planning_bypassing.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_BYPASSING_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningBypassing::ActionPlanningBypassing(){
  // nothing to do
}

ActionPlanningBypassing::~ActionPlanningBypassing(){
  // nothing to do
}

void ActionPlanningBypassing::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_BYPASSING_TRACE
  std::cout << "######### Action planning (Bypassing) >>>>>>>>>" << std::endl;
#endif

  const ActionPlanningSharedData::LaneInfo& curr_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_CURR];
  const ActionPlanningSharedData::LaneInfo& left_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_LEFT_1];
  const ActionPlanningSharedData::LaneInfo& right_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_RIGHT_1];

  planning_result->bypassing_req.allow_turning_left = false;
  planning_result->bypassing_req.allow_turning_right = false;

  if (!curr_lane.valid) {
    LOG_ERR << "Invalid information of current lane.";
    return;
  }

  /// 若配置参数/设置项, 不允许自主变道
  if (!act_planning_shared_data_->config.enable_auto_changing_lane) {
    return;
  }
  if (!planning_result->enable_ngp) {
    return;
  }

#if ENABLE_ACTION_PLANNING_BYPASSING_TRACE
  printf("Left Lane: valid=%d, has_risk=%d, "
         "has_uncertain=%d, lane_quality=%d, cost=%d(curr_cost=%d)"
         "\n",
         left_lane.valid, left_lane.risk_info.has_risk,
         left_lane.risk_info.has_uncertain_obj,
         left_lane.risk_info.lane_quality,
         left_lane.risk_info.cost, curr_lane.risk_info.cost);
  printf("Right Lane: valid=%d, has_risk=%d, "
         "has_uncertain=%d, lane_quality=%d, cost=%d(curr_cost=%d)"
         "\n",
         right_lane.valid, right_lane.risk_info.has_risk,
         right_lane.risk_info.has_uncertain_obj,
         right_lane.risk_info.lane_quality,
         right_lane.risk_info.cost, curr_lane.risk_info.cost);
#endif

  if (left_lane.valid &&
      /*!left_lane.risk_info.has_risk &&*/
      /*!left_lane.risk_info.has_uncertain_obj &&*/
      (left_lane.risk_info.lane_quality >= 2) /*&&
      (left_lane.risk_info.cost < curr_lane.risk_info.cost)*/) {
    // 可以向左变道避让
    planning_result->bypassing_req.allow_turning_left = true;
  }
  if (right_lane.valid &&
      /*!right_lane.risk_info.has_risk &&*/
      /*!right_lane.risk_info.has_uncertain_obj &&*/
      (right_lane.risk_info.lane_quality >= 2) /*&&
      (right_lane.risk_info.cost < curr_lane.risk_info.cost)*/) {
    // 可以向右变道避让
    planning_result->bypassing_req.allow_turning_right = true;
  }

  //planning_result->bypassing_req.allow_turning_left = true;
  //planning_result->bypassing_req.allow_turning_right = true;

#if ENABLE_ACTION_PLANNING_BYPASSING_TRACE
  printf("&&& allow_turning_left=%d, allow_turning_right=%d\n",
         planning_result->bypassing_req.allow_turning_left,
         planning_result->bypassing_req.allow_turning_right);
#endif
}


}  // namespace planning
}  // namespace phoenix

