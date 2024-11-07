//
#include "action_planning_wrapper.h"
#include "action_planning/action_planning_rule_based.h"


namespace phoenix {
namespace planning {


static ActionPlanningRuleBased s_action_planner;

ActionPlanningWrapper::ActionPlanningWrapper() {
  action_planner_ = &s_action_planner;
}

ActionPlanningWrapper::~ActionPlanningWrapper() {
}

void ActionPlanningWrapper::Configurate(const ActionPlanningConfig& conf) {
  return (action_planner_->Configurate(conf));
}

bool ActionPlanningWrapper::Plan(const ActionPlanningDataSource& data_source) {
  return (action_planner_->Plan(data_source));
}

const ActionPlanningResult& ActionPlanningWrapper::GetResultOfPlanning() const {
  return (action_planner_->GetResultOfPlanning());
}

Int32_t ActionPlanningWrapper::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  return (action_planner_->GetEventReporting(num, events));
}


} // namespace phoenix
} // namespace planning


