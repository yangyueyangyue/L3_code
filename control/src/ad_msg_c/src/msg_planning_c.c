//
#include "msg_planning_c.h"


void Phoenix_AdMsg_ClearPlanningResult(PlanningResult_t* const data) {
  Phoenix_AdMsg_ClearMsgHead(&(data->msg_head));

  phoenix_com_memset(data->planning_status, 0, sizeof(data->planning_status));

  data->tar_driving_mode = VEH_DRIVING_MODE_INVALID;
  data->enable_eps = 0;
  data->enable_throttle_sys = 0;
  data->enable_ebs = 0;

  data->hold_steering_wheel = 0;
  data->steering_wheel_speed = 0;
  data->release_throttle = 0;

  data->tar_gear = VEH_GEAR_INVALID;
  data->tar_turn_lamp = VEH_TURN_LAMP_INVALID;
  data->tar_brake_lamp = VEH_LAMP_INVALID;

  data->tar_v = 0;
  data->tar_a = 0;

  data->tar_trj.timestamp = 0;
  data->tar_trj.curr_pos.x = 0;
  data->tar_trj.curr_pos.y = 0;
  data->tar_trj.curr_pos.h = 0;
  data->tar_trj.curr_pos.c = 0;
  data->tar_trj.curr_pos.s = 0;
  data->tar_trj.curr_pos.l = 0;
  data->tar_trj.leading_pos.x = 0;
  data->tar_trj.leading_pos.y = 0;
  data->tar_trj.leading_pos.h = 0;
  data->tar_trj.leading_pos.c = 0;
  data->tar_trj.leading_pos.s = 0;
  data->tar_trj.leading_pos.l = 0;

  phoenix_com_memset(&(data->tar_trj.lat_err), 0, sizeof(data->tar_trj.lat_err));

  data->tar_trj.trj_direction = AD_MSG_TRJ_DIRECTION_FORWARD;
  data->tar_trj.points_num = 0;
  for (Int32_t i = 0; i < AD_MSG_MAX_TRAJECTORY_POINT_NUM; ++i) {
    data->tar_trj.points[i].x = 0;
    data->tar_trj.points[i].y = 0;
    data->tar_trj.points[i].h = 0;
    data->tar_trj.points[i].c = 0;
    data->tar_trj.points[i].s = 0;
  }
  data->pitch = 0.0;
  data->pitch_variance = 0.0;

    /// 油门值
  data->tar_throttle = 0.0;

  data->ramp_slope_value = 0.0;
}

Int32_t Phoenix_AdMsg_GetDrivingMapTpye(Uint32_t* const planning_status) {
  return ((planning_status[0] >> 8) & 0x0F);
}

Int32_t Phoenix_AdMsg_GetTrjPlanngingRoadTpye(Uint32_t* const planning_status) {
  return ((planning_status[0] >> 12) & 0x0F);
}

Int32_t Phoenix_AdMsg_GetTrjPlanngingTrjStatus(Uint32_t* const planning_status) {
  return ((planning_status[0] >> 16) & 0x0F);
}

Int32_t Phoenix_AdMsg_GetVelPlanngingTarType(Uint32_t* const planning_status) {
  return ((planning_status[0] >> 20) & 0x0F);
}

Int32_t Phoenix_AdMsg_GetTrjPlanngingTrjChgDirection(Int32_t planning_trj_status) {
  Int32_t dir = 0;
  if ((TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I      == planning_trj_status) ||
      (TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II     == planning_trj_status) ||
      (TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I   == planning_trj_status) ||
      (TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II  == planning_trj_status)) {
    // To left
    dir = 1;
  } else if ((TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I     == planning_trj_status) ||
             (TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II    == planning_trj_status) ||
             (TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I  == planning_trj_status) ||
             (TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == planning_trj_status)) {
    // To right
    dir = -1;
  } else {
    // No changing
    dir = 0;
  }

  return (dir);
}
