//
#include "action_planning/action_planning_base.h"

#include "utils/macros.h"
#include "utils/log.h"


#define ENABLE_ACTION_PLANNING_BASE_TRACE (0)


namespace phoenix {
namespace planning {


Int32_t ActionPlanningSharedData::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  if (num > event_reporting_list.Size()) {
    num = event_reporting_list.Size();
  }

  Int32_t idx = 0;
  for (idx = 0; idx < num; ++idx) {
    events[idx] = event_reporting_list[idx];
  }

  return (idx);
}

void ActionPlanningSharedData::AddEventReporting(
    Int32_t event_type, Int32_t param1) {
  ad_msg::EventReporting event;
  event.msg_head.valid = false;

  switch (event_type) {
  case (EVENT_TYPE_START_ROBOTIC):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_START_ROBOTIC;
    event.priority = 1;
    event.lifetime = 3*1000;
    break;
  case (EVENT_TYPE_STOP_ROBOTIC):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_STOP_ROBOTIC;
    event.priority = 1;
    event.lifetime = 3*1000;
    break;
  case (EVENT_TYPE_PREPARE_FOR_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 1;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_PREPARE_FOR_CHANGING_LEFT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_PREPARE_FOR_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_PREPARE_FOR_CHANGING_RIGHT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_START_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_LEFT;
    break;
  case (EVENT_TYPE_START_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_RIGHT;
    break;
  case (EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_LEFT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_RIGHT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_ABORT_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_LEFT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_RIGHT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_LEFT;
    break;
  case (EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_RIGHT;
    break;
  case (EVENT_TYPE_REFUSE_ABORT_CHANGING_LANE):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_ABORT_CHANGING_LANE;
    event.param[1] = param1;
    break;
  default:
    break;
  }

  if (event.msg_head.valid) {
    event_reporting_list.PushBack(event);
  }
}


ActionPlanningBase::ActionPlanningBase() {
  sequence_id_ = -1;
  next_action_planner_ = Nullptr_t;
}

ActionPlanningBase::~ActionPlanningBase(void) {
  // nothing to do
}

void ActionPlanningBase::Proceed(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
  Handle(data_source, planning_result);

  if (Nullptr_t != next_action_planner_) {
    if (next_action_planner_->sequence_id() == (sequence_id() + 1)) {
      next_action_planner_->Proceed(data_source, planning_result);
    } else {
      LOG_ERR << "Wrong order for execution, current="
              << sequence_id() << ", next="
              << next_action_planner_->sequence_id();
      COM_CHECK(next_action_planner_->sequence_id() == (sequence_id() + 1));
    }
  }
}



}  // namespace planning
}  // namespace phoenix

