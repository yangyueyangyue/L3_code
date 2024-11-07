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
#include "action_planning/action_planning_fallback.h"

#include "utils/macros.h"
#include "utils/log.h"
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  #include "Sys_InfoCom.h"//系统接口
#endif

#define ENABLE_ACTION_PLANNING_FALLBACK_TRACE (0)

namespace phoenix {
namespace planning {


ActionPlanningFallback::ActionPlanningFallback(){
  fallback_dec_status_ = 0;
  fallback_dec_level_ = REQ_FallBack_NONE;
  pre_fallback_dec_level_ = REQ_FallBack_NONE;
  fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
}

ActionPlanningFallback::~ActionPlanningFallback(){
  // nothing to do
}

void ActionPlanningFallback::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_FALLBACK_TRACE
  std::cout << "######### Action planning (Fallback) >>>>>>>>>" << std::endl;
#endif
  // clear
  planning_result->enable_fallback = false;
  planning_result->level_setting = REQ_FallBack_NONE;
  planning_result->fallback_action = REQ_FALLBACK_ACTION_NONE;

  fallback_dec_status_ = GetFallBacksReqType(data_source, planning_result);
  #if ENABLE_ACTION_PLANNING_FALLBACK_TRACE
  std::cout << "pre_fallback_dec_level_ = " << pre_fallback_dec_level_ << std::endl;
  #endif
  pre_fallback_dec_level_ = fallback_dec_status_;

  switch (data_source.planning_settings->enable_fallback) {
  case (1):
    planning_result->enable_fallback = false;
    break;
  case (2):
    planning_result->enable_fallback = true;
    break;
  default:
    // 不更新
    break;
  }
  planning_result->level_setting = fallback_dec_level_;
  planning_result->fallback_action = fallback_dec_action_;
}


void ActionPlanningFallback::ForbidAutoFallBack(const ActionPlanningDataSource& data_source) {
  Int64_t hold_time ; 
  Int8_t target_fallback_level;
  target_fallback_level = data_source.planning_settings->target_fallback_level;
  #if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    if (degrade_B == target_fallback_level) {
      hold_time = 7 * 1000;
    } else if (degrade_C == target_fallback_level) {
      hold_time = 10 * 1000;
    }
  #else
    if (2 == target_fallback_level) {
      hold_time = 7 * 1000;
    } else if (3 == target_fallback_level) {
      hold_time = 10 * 1000;
    }
  #endif

  act_planning_shared_data_->timers.forbid_auto_fallback.SetTimeout(hold_time);
  act_planning_shared_data_->timers.forbid_auto_fallback.Restart();

}

Int8_t ActionPlanningFallback::GetFallBacksReqType(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
  
  Int8_t request_type = REQ_FallBack_NONE;
  Int8_t target_fallback_level_valid;
  Int8_t target_fallback_level;

  target_fallback_level = data_source.planning_settings->target_fallback_level;
  #if ENABLE_ACTION_PLANNING_FALLBACK_TRACE
  std::cout << "target_fallback_level = " << target_fallback_level << std::endl;
  #endif

  switch (data_source.planning_settings->enable_fallback) {
  case (1):
    target_fallback_level_valid = false;
    break;
  case (2):
    target_fallback_level_valid = true;
    break;
  default:
    // 不更新
    break;
  }


  if (!(target_fallback_level_valid)) {
    
    return(request_type);
  }

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  if (degrade_A == target_fallback_level) {
    request_type = REQ_FallBack_A;
  } else if (degrade_B == target_fallback_level) {
    request_type = REQ_FallBack_B;
  } else if (degrade_C == target_fallback_level) {
    request_type = REQ_FallBack_C;
  } else if (degrade_G == target_fallback_level) {
    request_type = REQ_FallBack_D;
  }
    
  if ((target_fallback_level) != (pre_fallback_dec_level_)){
    if (degrade_A == target_fallback_level) {
      // nothing
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      return(request_type); 
    } else if (degrade_B == target_fallback_level) {
      // B 级 7 秒
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      ForbidAutoFallBack(data_source);
      return(request_type);
    } else if (degrade_C == target_fallback_level) {
      // C 级 2 秒
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      ForbidAutoFallBack(data_source);
      return(request_type);
    } else if (degrade_G == target_fallback_level) {
      // nothing
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      return(request_type);
    } else {
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    }
  } else {
    if (degrade_A == target_fallback_level) {
      // nothing
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    } else if (degrade_B == target_fallback_level) {
      // B 级 7 秒
      if (act_planning_shared_data_->timers.forbid_auto_fallback.IsActive())
      {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      } else {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_B_II;
      }
    } else if (degrade_C == target_fallback_level) {
      // C 级 2 秒
      if (act_planning_shared_data_->timers.forbid_auto_fallback.IsActive())
      {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_C_I;
      } else {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_C_II;
      }
    } else if (degrade_G == target_fallback_level) {
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    } else {
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    }
  }
  
  return(request_type);
#else
  if (1 == target_fallback_level) {
    request_type = REQ_FallBack_A;
  } else if (2 == target_fallback_level) {
    request_type = REQ_FallBack_B;
  } else if (3 == target_fallback_level) {
    request_type = REQ_FallBack_C;
  } else if (4 == target_fallback_level) {
    request_type = REQ_FallBack_D;
  }
    
  if ((target_fallback_level) != (pre_fallback_dec_level_)){
    if (1 == target_fallback_level) {
      // nothing
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      return(request_type); 
    } else if (2 == target_fallback_level) {
      // B 级 7 秒
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      ForbidAutoFallBack(data_source);
      return(request_type);
    } else if (3 == target_fallback_level) {
      // C 级 2 秒
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      ForbidAutoFallBack(data_source);
      return(request_type);
    } else if (4 == target_fallback_level) {
      // nothing
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      return(request_type);
    } else {
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    }
  } else {
    if (1 == target_fallback_level) {
      // nothing
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    } else if (2 == target_fallback_level) {
      // B 级 7 秒
      if (act_planning_shared_data_->timers.forbid_auto_fallback.IsActive())
      {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
      } else {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_B_II;
      }
    } else if (3 == target_fallback_level) {
      // C 级 2 秒
      if (act_planning_shared_data_->timers.forbid_auto_fallback.IsActive())
      {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_C_I;
      } else {
        fallback_dec_action_ = REQ_FALLBACK_ACTION_C_II;
      }
    } else if (4 == target_fallback_level) {
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    } else {
      fallback_dec_action_ = REQ_FALLBACK_ACTION_NONE;
    }
  }
  
  return(request_type);
#endif
}


}  // namespace planning
}  // namespace phoenix


