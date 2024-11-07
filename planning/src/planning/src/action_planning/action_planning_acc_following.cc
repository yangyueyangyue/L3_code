/*******************************************************************************
 * @file acc_follow_status.cc
 * @author sip (sip@goyu-ai.com)
 * @brief 跟车状态实现
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
#include "action_planning/action_planning_acc_following.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_ACC_FOLLOWING_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningAccFollowing::ActionPlanningAccFollowing() {
  // nothing to do
}

ActionPlanningAccFollowing::~ActionPlanningAccFollowing(){
  // nothing to do
}

//跟车条件判断
void ActionPlanningAccFollowing::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_ACC_FOLLOWING_TRACE
  std::cout << "######### Action planning (ACC Following) >>>>>>>>>" << std::endl;
#endif

//  if(ad_msg::VEH_DRIVING_MODE_ROBOTIC_FULLY==planning_result.driving_mode){
//    if(true == data_source.hmi_setting->acc_enable){
//      planning_result.acc_enable = true;
//    }
//    else{
//      planning_result.acc_enable = false;
//    }

//    if(true == planning_result.acc_enable){
//      planning_result.v_setting = data_source.hmi_setting->target_velocity;//需要判断目标物的速度与加速度
//      planning_result.a_setting = data_source.hmi_setting->target_acc;
//      planning_result.ttc_setting = data_source.hmi_setting->target_ttc;
//      planning_result.action_requst = ActionPlanningResult::ACC_FOLLOW_STATUS;
//      action_manager_->ActionPlanning(data_source, planning_result);
//    }
//    else
//    {
//      action_manager_->ActionPlanning(data_source, planning_result);
//    }
//  }
}


}  // namespace planning
}  // namespace phoenix

