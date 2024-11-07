//
#include "trajectory_planning_wrapper.h"
#include "trajectory_planning/trajectory_planning_state_lattice.h"


namespace phoenix {
namespace planning {


static TrajectoryPlanningStateLattice s_trajectory_planner;

Uint32_t GetTrajectoryPlanningImplSize() {
  return (sizeof(TrajectoryPlanningStateLattice));
}


TrajectoryPlanningWrapper::TrajectoryPlanningWrapper() {
  trajectory_planner_ = &s_trajectory_planner;

  std::cout << "sizeof(TrajectoryPlanningStateLattice)="
            << sizeof(TrajectoryPlanningStateLattice)
            << " bytes (" << sizeof(TrajectoryPlanningStateLattice) / 1024.0F / 1024.0F
            << " M)"
            << std::endl;
}

TrajectoryPlanningWrapper::~TrajectoryPlanningWrapper() {
}

void TrajectoryPlanningWrapper::Configurate(
    const TrajectoryPlanningConfig& conf) {
  trajectory_planner_->Configurate(conf);
}

bool TrajectoryPlanningWrapper::Plan(
    const TrajectoryPlanningDataSource& data_source) {
  return (trajectory_planner_->Plan(data_source));
}

const TrajectoryPlanningResult&
TrajectoryPlanningWrapper::GetResultOfPlanning() const {
  return (trajectory_planner_->GetResultOfPlanning());
}

void TrajectoryPlanningWrapper::GetTrajectoryPlanningInfo(
    TrajectoryPlanningInfo* trj_planning_info) const {
  trajectory_planner_->GetTrajectoryPlanningInfo(trj_planning_info);
}

Int32_t TrajectoryPlanningWrapper::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  return (trajectory_planner_->GetEventReporting(num, events));
}


} // namespace phoenix
} // namespace planning


