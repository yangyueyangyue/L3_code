#include "velocity_planning_wrapper.h"

#ifdef ENABLE_PACC
#include "velocity_planning/velocity_planning_polynomial_fitting.h"
#if (ENABLE_BACK_TO_BACK_TEST)
#include "velocity_planning/velocity_planning_fuzzy_pid.h"
#endif
#else
#include "velocity_planning/velocity_planning_fuzzy_pid.h"
#endif

namespace phoenix {
namespace planning {

//XXX CMakeList 中默认ENABLE_PACC 1 开
#ifdef ENABLE_PACC 
static VelocityPlanningPolynomialFitting s_velocity_planner;//默认是开
#if (ENABLE_BACK_TO_BACK_TEST)
static VelocityPlanningFuzzyPID s_velocity_planner_org;
#endif
#else
static VelocityPlanningFuzzyPID s_velocity_planner;
#endif

VelocityPlanningWrapper::VelocityPlanningWrapper() {
  velocity_planner_ = &s_velocity_planner;
}

VelocityPlanningWrapper::~VelocityPlanningWrapper() {
}

void VelocityPlanningWrapper::Configurate(const VelocityPlanningConfig& conf) {
#if (ENABLE_BACK_TO_BACK_TEST)
  s_velocity_planner_org.Configurate(conf);
#endif
  velocity_planner_->Configurate(conf);
}

bool VelocityPlanningWrapper::Plan(
    const VelocityPlanningDataSource& data_source) {
#if (ENABLE_BACK_TO_BACK_TEST)
  s_velocity_planner_org.Plan(data_source);
#endif
  return (velocity_planner_->Plan(data_source));
}

#if (ENABLE_BACK_TO_BACK_TEST)
const VelocityPlanningResult&
VelocityPlanningWrapper::GetResultOfPlanningOrg() const {
  return (s_velocity_planner_org.GetResultOfPlanning());
}
#endif

const VelocityPlanningResult&
VelocityPlanningWrapper::GetResultOfPlanning() const {
  return (velocity_planner_->GetResultOfPlanning());
}

const VelocityPlanningInternal& VelocityPlanningWrapper::GetInternalOfPlanningDebug() const {
  return velocity_planner_->GetInternalOfPlanningDebug();
}

Int32_t VelocityPlanningWrapper::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  return (velocity_planner_->GetEventReporting(num, events));
}


} // namespace phoenix
} // namespace planning


