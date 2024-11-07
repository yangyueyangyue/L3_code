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
#include "action_planning/action_planning_pulling_over.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_PULLING_OVER_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningPullingOver::ActionPlanningPullingOver(){
  // nothing to do
}

ActionPlanningPullingOver::~ActionPlanningPullingOver(){
  // nothing to do
}

void ActionPlanningPullingOver::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_PULLING_OVER_TRACE
  std::cout << "######### Action planning (Pulling over) >>>>>>>>>" << std::endl;
#endif

//  if(ad_msg::VEH_DRIVING_MODE_ROBOTIC_FULLY==planning_result.driving_mode){

//    if(0/*触发Pull Over条件*/){
//      planning_result.v_setting = 0;
//      planning_result.a_setting = -2.0;
//      planning_result.ttc_setting = 2.0;
//      //planning_result.action_requst = ActionPlanningResult::PULL_OVER_STATUS;
//    }
//    else
//    {
//      action_manager_->ActionPlanning(data_source, planning_result);
//    }
//  }
}


}  // namespace planning
}  // namespace phoenix

