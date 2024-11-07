/*******************************************************************************
 * @file emergency_stop_status.cc
 * @author sip (sip@goyu-ai.com)
 * @brief 紧急停车(AEB)
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
#include "action_planning/action_planning_emergency_stopping.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_EMERGENCY_STOPPING_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningEmergencyStopping::ActionPlanningEmergencyStopping(){
  // nothing to do
}

ActionPlanningEmergencyStopping::~ActionPlanningEmergencyStopping(){
  // nothing to do
}

void ActionPlanningEmergencyStopping::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_EMERGENCY_STOPPING_TRACE
  std::cout << "######### Action planning (Emergency Stopping) >>>>>>>>>" << std::endl;
#endif

//  if(ad_msg::VEH_DRIVING_MODE_ROBOTIC_FULLY==planning_result.driving_mode){
//    if(data_source.hmi_setting->aeb_enable==true){
//      planning_result.aeb_enable = true;
//    }
//    else{
//      planning_result.aeb_enable = false;
//    }

//    if(0/*触发AEB条件*/){
//      planning_result.v_setting = 0.0;
//      planning_result.a_setting = -5.0;
//      planning_result.ttc_setting = 2.0;
//      planning_result.action_requst = ActionPlanningResult::EMERGENCY_STOP_STATUS;
//    }
//    else{
//      action_manager_->ActionPlanning(data_source, planning_result);
//    }
//  }
}


}  // namespace planning
}  // namespace phoenix

