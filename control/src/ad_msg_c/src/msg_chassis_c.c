//
#include "msg_chassis_c.h"

void Phoenix_AdMsg_ClearChassis(Chassis_t* const data) {
  Phoenix_AdMsg_ClearMsgHead(&(data->msg_head));

  data->driving_mode = VEH_DRIVING_MODE_INVALID;
  data->e_stop = VEH_E_STOP_INVALID;

  data->eps_status = VEH_EPS_STATUS_INVALID;
  data->throttle_sys_status = VEH_THROTTLE_SYS_STATUS_INVALID;
  data->ebs_status = VEH_EBS_STATUS_INVALID;

  data->steering_wheel_angle_valid = 0;
  data->steering_wheel_angle = 0;
  data->steering_wheel_speed_valid = 0;
  data->steering_wheel_speed = 0;
  data->steering_wheel_torque_valid = 0;
  data->steering_wheel_torque = 0;

  data->v_valid = 0;
  data->v = 0;
  data->a_valid = 0;
  data->a = 0;
  data->yaw_rate_valid = 0;
  data->yaw_rate = 0;
  data->ax_valid = 0;
  data->ax = 0;
  data->ay_valid = 0;
  data->ay = 0;

  data->wheel_speed_fl_valid = 0;
  data->wheel_speed_fl = 0;
  data->wheel_speed_fr_valid = 0;
  data->wheel_speed_fr = 0;
  data->wheel_speed_rl_valid = 0;
  data->wheel_speed_rl = 0;
  data->wheel_speed_rr_valid = 0;
  data->wheel_speed_rr = 0;
  data->wheel_speed_rl2_valid = 0;
  data->wheel_speed_rl2 = 0;
  data->wheel_speed_rr2_valid = 0;
  data->wheel_speed_rr2 = 0;

  data->epb_status = VEH_EPB_STATUS_INVALID;
  data->gear = VEH_GEAR_INVALID;
  data->gear_number = 0;
  data->selected_gear_number = 0;

  data->signal_turning_indicator = VEH_TURNING_INDICATOR_INVALID;
  data->signal_turn_lamp = VEH_TURN_LAMP_INVALID;
  data->signal_brake_lamp = VEH_LAMP_INVALID;
  data->brake_pedal_value = 0;
  data->acc_pedal_value = 0;

  data->engine_speed_valid = 0;
  data->engine_speed = 0;
  data->engine_torque_valid = 0;
  data->engine_torque = 0;

  data->gross_weight_valid = 0;
  data->gross_weight = 0;
  data->trailer_status = 0;

  data->trailer_l = 0.0F;
  data->trailer_w = 0.0F;
  data->trailer_h = 0.0F;
}

void Phoenix_AdMsg_ClearChassisCtlCmd(ChassisCtlCmd_t* const data) {
  Phoenix_AdMsg_ClearMsgHead(&(data->msg_head));

  data->start_robotic_ctl = 0;

  data->enable_eps = 0;
  data->enable_throttle_sys = 0;
  data->enable_ebs = 0;

  data->enable_remote_ctl = 0;
  data->enable_direct_ctl = 0;
  data->enable_acc = 0;
  data->release_throttle = 0;

  data->steering_wheel_angle = 0;
  data->steering_wheel_speed = 0;
  data->steering_wheel_torque = 0;

  data->velocity = 0;
  data->acceleration = 0;
  data->acc_value = 0;
  data->brake_value = 0;

  data->gear = VEH_GEAR_INVALID;
  data->turn_lamp = VEH_TURN_LAMP_INVALID;
  data->brake_lamp = VEH_LAMP_INVALID;
  data->wiper = VEH_WIPER_INVALID;
  data->epb_status = VEH_EPB_STATUS_INVALID;

  data->ebs_mode_enable = 0;
  data->ebi_control_mode = 0;
  data->ebs_ungercy = 0;

  data->tar_type = 0;

}

void Phoenix_AdMsg_ClearChassisFtAuman(ChassisFtAuman_t* const data) {
  data->switch_hwa = 0;
  data->switch_tja = 0;
  data->switch_i_drive = 0;
}

void Phoenix_AdMsg_ClearChassisDFD17(ChassisDfD17_t* const data) {
  data->BCM_state = 0;
  data->VECU_state = 0;
  data->EPS_state = 0;
  data->CEPS1_state = 0;
  data->CEPS2_state = 0;
  data->EBS_state = 0; 
  data->CEBS_state = 0;
  data->ADCU_Mode_Sts = 0;
  data->enable_fallback = 0;
  data->target_fallback_valid = 0;
  data->AD_fallback_level = 0;
  data->AHPSFailureCode = 0;
  data->SteeringErrorEvent1ActivePS = 0;
  data->SteeringErrorEvent1PS = 0;

}

void Phoenix_AdMsg_ClearChassisDfD17adrecord(ChassisDfD17_adrecord* const data){
  data->ad_mode = 0;
  data->tar_vel_set = 0;
  data->tar_trajectory_type = 0;
  data->tar_vel_type = 0;
  data->vel_obj_dist = 0.0;
  data->vel_obj_vel = 0.0;
  data->planning_send_vel = 0.0;
  data->planning_send_ax = 0.0;
}


#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
void Phoenix_AdMsg_ClearDFCVSpecialChassisInfo(DFCV_SpecialChassisInfo_t* const data){
  data->source_address_brake_control_device = 0;
  data->vehicle_mass = 5000;
  data->brake_pressure_lf = 0;
  data->current_gear_ratio = 1;
  data->transmission_selected_gear = 0;
  data->clutch_switch = 0;
  data->nominal_fricton_troque_percent = 0;
  data->estimated_lossed_torque_percent = 0;
  data->actual_engine_torque_percent = 0;
  data->driver_damand_torque_percent = 0;
  data->source_address_engine_control_device = 0;
  data->transmission_shift_status = 0;
  data->transmission_engage_status = 0;
  data->tcu_engine_control_mode = 0;
  data->trailer_connected_status = 0;
}
#endif

void Phoenix_AdMsg_ClearSpecialChassisInfo(SpecialChassisInfo_t* const data) {
  Phoenix_AdMsg_ClearMsgHead(&(data->msg_head));

  data->start_adas = 0;
  data->enable_acc = 0;
  data->enable_lka = 0;
  data->enable_aeb = 0;
  data->enable_alc = 0;
  data->enable_isl = 0;
  data->enable_ngp = 0;
  data->target_velocity_valid = 0;
  data->target_velocity = 0;
  data->target_acc_valid = 0;
  data->target_acc = 0;
  data->target_time_gap_valid = 0;
  data->target_time_gap = 0;
  data->changing_lane_req = 0;

  // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  data->cnt_stu_frame_loss_can0 = 0;
  data->cnt_stu_frame_loss_can1 = 0;
  data->cnt_stu_frame_loss_can2 = 0;
  data->cnt_stu_frame_loss_can3 = 0;

  // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data->cnt_stu_gtw_to_veh_can0 = 0;
  data->cnt_stu_gtw_to_veh_can1 = 0;
  data->cnt_stu_gtw_to_veh_can2 = 0;
  data->cnt_stu_gtw_to_veh_can3 = 0;

  // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data->cnt_stu_ctl_to_gtw_can0 = 0;
  data->cnt_stu_ctl_to_gtw_can1 = 0;
  data->cnt_stu_ctl_to_gtw_can2 = 0;
  data->cnt_stu_ctl_to_gtw_can3 = 0;

  // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  data->cnt_stu_ctl_to_gtw = 0;

  // 具体车型的特殊信息 (FT-Auman)
  Phoenix_AdMsg_ClearChassisFtAuman(&(data->ft_auman));
  // 具体车型的特殊信息 (DF-D17)
  Phoenix_AdMsg_ClearChassisDFD17(&(data->df_d17));
}

Int32_t Phoenix_AdMsg_IsChassisInRoboticMode(Int32_t status) {
  Int32_t is_robotic = 0;

  switch (status) {
  case (VEH_CHASSIS_CTL_STATUS_MANUAL):
    is_robotic = 0;
    break;
  case (VEH_CHASSIS_CTL_STATUS_REQ_ROBOTIC_CTL):
    is_robotic = 1;
    break;
  case (VEH_CHASSIS_CTL_STATUS_WAITING_ROBOTIC_CTL_ACK):
    is_robotic = 1;
    break;
  case (VEH_CHASSIS_CTL_STATUS_ROBOTIC):
    is_robotic = 1;
    break;
  case (VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL):
    is_robotic = 0;
    break;
  case (VEH_CHASSIS_CTL_STATUS_WAITING_MANUAL_CTL_ACK):
    is_robotic = 0;
    break;
  default:
    is_robotic = 0;
    break;
  }

  return (is_robotic);
}
