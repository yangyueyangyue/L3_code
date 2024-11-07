//
#include "communication/parse_proto_msg.h"

#include "utils/log.h"
#include "utils/gps_tools.h"
#include "utils/com_clock_c.h"
#include "math/math_utils.h"


namespace phoenix {
namespace framework {


bool ParseProtoMsg::EncodeGnssMessage(
    const Gnss_t& msg,
    msg::localization::Gnss* const data_out) {
  data_out->set_latitude(msg.latitude);
  data_out->set_longitude(msg.longitude);
  data_out->set_altitude(msg.altitude);
  data_out->set_heading_gnss(msg.heading_gnss);
  data_out->set_v_e(msg.v_e);
  data_out->set_v_n(msg.v_n);
  data_out->set_v_u(msg.v_u);
  data_out->set_pitch(msg.pitch);
  data_out->set_roll(msg.roll);
  data_out->set_x_utm(msg.x_utm);
  data_out->set_y_utm(msg.y_utm);
  data_out->set_z_utm(msg.z_utm);
  data_out->set_heading_utm(msg.heading_utm);
  data_out->set_v_x_utm(msg.v_x_utm);
  data_out->set_v_y_utm(msg.v_y_utm);
  data_out->set_v_z_utm(msg.v_z_utm);
  switch (msg.gnss_status) {
  case (GNSS_STATUS_BAD):
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_BAD);
    break;
  case (GNSS_STATUS_CONVERGING):
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_CONVERGING);
    break;
  case (GNSS_STATUS_GOOD):
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_GOOD);
    break;
  default:
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_INVALID);
    break;
  }
  switch (msg.utm_status) {
  case (GNSS_STATUS_BAD):
    data_out->set_utm_status(msg::localization::Gnss::STATUS_BAD);
    break;
  case (GNSS_STATUS_CONVERGING):
    data_out->set_utm_status(msg::localization::Gnss::STATUS_CONVERGING);
    break;
  case (GNSS_STATUS_GOOD):
    data_out->set_utm_status(msg::localization::Gnss::STATUS_GOOD);
    break;
  default:
    data_out->set_utm_status(msg::localization::Gnss::STATUS_INVALID);
    break;
  }
  switch (msg.odom_status) {
  case (GNSS_STATUS_BAD):
    data_out->set_odom_status(msg::localization::Gnss::STATUS_BAD);
    break;
  case (GNSS_STATUS_CONVERGING):
    data_out->set_odom_status(msg::localization::Gnss::STATUS_CONVERGING);
    break;
  case (GNSS_STATUS_GOOD):
    data_out->set_odom_status(msg::localization::Gnss::STATUS_GOOD);
    break;
  default:
    data_out->set_odom_status(msg::localization::Gnss::STATUS_INVALID);
    break;
  }

  return true;
}

bool ParseProtoMsg::DecodeGnssMessage(
    const Char_t* msg, Int32_t msg_len,
    Gnss_t* data_out) {
  // parse
  msg::localization::Gnss gnss;
  if (!gnss.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse gnss from array.";
    return false;
  }

  data_out->latitude = gnss.latitude();
  data_out->longitude = gnss.longitude();
  data_out->altitude = gnss.altitude();
  data_out->heading_gnss = common::com_deg2rad(
        common::ConvGpsHeading(gnss.heading_gnss()));
  data_out->x_utm = gnss.x_utm();
  data_out->y_utm = gnss.y_utm();
  data_out->z_utm = gnss.z_utm();
  /// TODO: UTM定位的航向角超出了-pi到+pi的范围，后期需要修改
  data_out->heading_utm = common::NormalizeAngle(gnss.heading_utm());
  data_out->x_odom = gnss.x_odom();
  data_out->y_odom = gnss.y_odom();
  data_out->z_odom = gnss.z_odom();
  data_out->heading_odom = common::NormalizeAngle(gnss.heading_odom());
  data_out->pitch = gnss.pitch();
  data_out->roll = gnss.roll();
  data_out->v_e = gnss.v_e();
  data_out->v_n = gnss.v_n();
  data_out->v_u = gnss.v_u();
  data_out->v_x_utm = gnss.v_x_utm();
  data_out->v_y_utm = gnss.v_y_utm();
  data_out->v_z_utm = gnss.v_z_utm();
  data_out->v_x_odom = gnss.v_x_odom();
  data_out->v_y_odom = gnss.v_y_odom();
  data_out->v_z_odom = gnss.v_z_odom();

  switch (gnss.gnss_status()) {
  case msg::localization::Gnss::STATUS_BAD:
    data_out->gnss_status = GNSS_STATUS_BAD;
    break;
  case msg::localization::Gnss::STATUS_CONVERGING:
    data_out->gnss_status = GNSS_STATUS_CONVERGING;
    break;
  case msg::localization::Gnss::STATUS_GOOD:
    data_out->gnss_status = GNSS_STATUS_GOOD;
    break;
  default:
    data_out->gnss_status = GNSS_STATUS_INVALID;
    break;
  }

  switch (gnss.utm_status()) {
  case msg::localization::Gnss::STATUS_BAD:
    data_out->utm_status = GNSS_STATUS_BAD;
    break;
  case msg::localization::Gnss::STATUS_CONVERGING:
    data_out->utm_status = GNSS_STATUS_CONVERGING;
    break;
  case msg::localization::Gnss::STATUS_GOOD:
    data_out->utm_status = GNSS_STATUS_GOOD;
    break;
  default:
    data_out->utm_status = GNSS_STATUS_INVALID;
    break;
  }

  switch (gnss.odom_status()) {
  case msg::localization::Gnss::STATUS_BAD:
    data_out->odom_status = GNSS_STATUS_BAD;
    break;
  case msg::localization::Gnss::STATUS_CONVERGING:
    data_out->odom_status = GNSS_STATUS_CONVERGING;
    break;
  case msg::localization::Gnss::STATUS_GOOD:
    data_out->odom_status = GNSS_STATUS_GOOD;
    break;
  default:
    data_out->odom_status = GNSS_STATUS_INVALID;
    break;
  }

  if (com_isnan(data_out->latitude) || com_isinf(data_out->latitude) ||
      com_isnan(data_out->longitude) || com_isinf(data_out->longitude) ||
      com_isnan(data_out->heading_gnss) || com_isinf(data_out->heading_gnss)) {
    data_out->gnss_status = GNSS_STATUS_INVALID;

    data_out->latitude = 0;
    data_out->longitude = 0;
    data_out->heading_gnss = 0;
  }

  if (com_isnan(data_out->x_utm) || com_isinf(data_out->x_utm) ||
      com_isnan(data_out->y_utm) || com_isinf(data_out->y_utm) ||
      com_isnan(data_out->heading_utm) || com_isinf(data_out->heading_utm)) {
    data_out->utm_status = GNSS_STATUS_INVALID;

    data_out->x_utm = 0;
    data_out->y_utm = 0;
    data_out->heading_utm = 0;
  }

  if (com_isnan(data_out->x_odom) || com_isinf(data_out->x_odom) ||
      com_isnan(data_out->y_odom) || com_isinf(data_out->y_odom) ||
      com_isnan(data_out->heading_odom) || com_isinf(data_out->heading_odom)) {
    data_out->odom_status = GNSS_STATUS_INVALID;

    data_out->x_odom = 0;
    data_out->y_odom = 0;
    data_out->heading_odom = 0;
  }

  // Update message head
  data_out->msg_head.valid = true;
  Phoenix_AdMsg_UpdateSequenceNum(&(data_out->msg_head), 1);
  data_out->msg_head.timestamp = Phoenix_Common_GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeImuMessage(
    const Imu_t& msg,
    msg::localization::Imu* const data_out) {
  data_out->set_accel_x(msg.accel_x);
  data_out->set_accel_y(msg.accel_y);
  data_out->set_accel_z(msg.accel_z);
  data_out->set_pitch_rate(msg.pitch_rate);
  data_out->set_roll_rate(msg.roll_rate);
  data_out->set_yaw_rate(msg.yaw_rate);

  return true;
}

bool ParseProtoMsg::DecodeImuMessage(
    const Char_t* msg, Int32_t msg_len,
    Imu_t* data_out) {
  // parse
  msg::localization::Imu imu;
  if (!imu.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse imu from array.";
    return false;
  }

  data_out->yaw_rate = imu.yaw_rate();
  data_out->pitch_rate = imu.pitch_rate();
  data_out->roll_rate = imu.roll_rate();
  data_out->accel_x = imu.accel_x();
  data_out->accel_y = imu.accel_y();
  data_out->accel_z = imu.accel_z();

  // Update message head
  data_out->msg_head.valid = true;
  Phoenix_AdMsg_UpdateSequenceNum(&(data_out->msg_head), 1);
  data_out->msg_head.timestamp = Phoenix_Common_GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeChassisMessage(
    const Chassis_t& msg,
    msg::control::Chassis* const data_out) {
  // 驾驶模式
  switch (msg.driving_mode) {
  case (VEH_DRIVING_MODE_MANUAL):
    data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_MANUAL);
    break;
  case (VEH_DRIVING_MODE_ROBOTIC):
    data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_ROBOTIC);
    break;
  default:
    data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_INVALID);
    break;
  }

  // 紧急停止信号
  switch (msg.e_stop) {
  case (VEH_E_STOP_OFF):
    data_out->set_e_stop(msg::control::Chassis::E_STOP_OFF);
    break;
  case (VEH_E_STOP_ON):
    data_out->set_e_stop(msg::control::Chassis::E_STOP_ON);
    break;
  default:
    data_out->set_e_stop(msg::control::Chassis::E_STOP_INVALID);
    break;
  }

  // 方向盘控制状态
  switch (msg.eps_status) {
  case (VEH_EPS_STATUS_MANUAL):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_MANUAL);
    break;
  case (VEH_EPS_STATUS_ROBOTIC):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_ROBOTIC);
    break;
  case (VEH_EPS_STATUS_MANUAL_INTERRUPT):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_MANUAL_INTERRUPT);
    break;
  case (VEH_EPS_STATUS_ERROR):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_ERROR);
    break;
  default:
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_INVALID);
    break;
  }
  // 油门系统控制状态
  switch (msg.throttle_sys_status) {
  case (VEH_THROTTLE_SYS_STATUS_MANUAL):
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_MANUAL);
    break;
  case (VEH_THROTTLE_SYS_STATUS_ROBOTIC):
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_ROBOTIC);
    break;
  case (VEH_THROTTLE_SYS_STATUS_ERROR):
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_ERROR);
    break;
  default:
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_INVALID);
    break;
  }
  // 制动系统控制状态
  switch (msg.ebs_status) {
  case (VEH_EBS_STATUS_MANUAL):
    data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_MANUAL);
    break;
  case (VEH_EBS_STATUS_ROBOTIC):
    data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_ROBOTIC);
    break;
  case (VEH_EBS_STATUS_ERROR):
    data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_ERROR);
    break;
  default:
    data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_INVALID);
    break;
  }

  // 方向盘角度有效位
  data_out->set_steering_wheel_angle_valid(msg.steering_wheel_angle_valid);
  // 方向盘角度 (rad)
  data_out->set_steering_wheel_angle(msg.steering_wheel_angle);
  // 方向盘转速有效位
  data_out->set_steering_wheel_speed_valid(msg.steering_wheel_speed_valid);
  // 方向盘转速 (rad/s)
  data_out->set_steering_wheel_speed(msg.steering_wheel_speed);
  // 实际转向扭矩有效位
  data_out->set_steering_wheel_torque_valid(msg.steering_wheel_torque_valid);
  // 实际转向扭矩(N.m)
  data_out->set_steering_wheel_torque(msg.steering_wheel_torque);

 // 车速有效位
  data_out->set_velocity_valid(msg.v_valid);
  // 车速(m/s)
  data_out->set_velocity(msg.v);
  // 加速度有效位
  data_out->set_acceleration_valid(msg.a_valid);
  // 加速度
  data_out->set_acceleration(msg.a);
  // Yaw Rate有效位
  data_out->set_yaw_rate_valid(msg.yaw_rate_valid);
  // Yaw Rate(rad/s), PH:[-2.0943rad/s,+2.0943rad/s]
  data_out->set_yaw_rate(msg.yaw_rate);
  // AX 有效位
  data_out->set_ax_valid(msg.ax_valid);
  // AX(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->set_ax(msg.ax);
  // AY 有效位
  data_out->set_ay_valid(msg.ay_valid);
  // AY(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->set_ay(msg.ay);

  // 左前轮速有效位
  data_out->set_wheel_speed_fl_valid(msg.wheel_speed_fl_valid);
  // 左前轮速(m/s)
  data_out->set_wheel_speed_fl(msg.wheel_speed_fl);
  // 右前轮速有效位
  data_out->set_wheel_speed_fr_valid(msg.wheel_speed_fr_valid);
  // 右前轮速(m/s)
  data_out->set_wheel_speed_fr(msg.wheel_speed_fr);
  // 左后轮速有效位
  data_out->set_wheel_speed_rl_valid(msg.wheel_speed_rl_valid);
  // 左后轮速(m/s)
  data_out->set_wheel_speed_rl(msg.wheel_speed_rl);
  // 右后轮速有效位
  data_out->set_wheel_speed_rr_valid(msg.wheel_speed_rr_valid);
  // 右后轮速(m/s)
  data_out->set_wheel_speed_rr(msg.wheel_speed_rr);
  // 左后2#轮速有效位
  data_out->set_wheel_speed_rl2_valid(msg.wheel_speed_rl2_valid);
  // 左后2#轮速(m/s)
  data_out->set_wheel_speed_rl2(msg.wheel_speed_rl2);
  // 右后2#轮速有效位
  data_out->set_wheel_speed_rr2_valid(msg.wheel_speed_rr2_valid);
  // 右后2#轮速(m/s)
  data_out->set_wheel_speed_rr2(msg.wheel_speed_rr2);

  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (msg.epb_status) {
  case (VEH_EPB_STATUS_OFF):
    data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_OFF);
    break;
  case (VEH_EPB_STATUS_ON):
    data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_ON);
    break;
  default:
    data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_INVALID);
    break;
  }

  // 档位
  switch (msg.gear) {
  case (VEH_GEAR_P):
    data_out->set_gear(msg::control::Chassis::GEAR_P);
    break;
  case (VEH_GEAR_N):
    data_out->set_gear(msg::control::Chassis::GEAR_N);
    break;
  case (VEH_GEAR_R):
    data_out->set_gear(msg::control::Chassis::GEAR_R);
    break;
  case (VEH_GEAR_D):
    data_out->set_gear(msg::control::Chassis::GEAR_D);
    break;
  default:
    data_out->set_gear(msg::control::Chassis::GEAR_INVALID);
    break;
  }
  // 转向灯信号
  switch (msg.signal_turn_lamp) {
  case (VEH_TURN_LAMP_OFF):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_OFF);
    break;
  case (VEH_TURN_LAMP_RIGHT):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_RIGHT);
    break;
  case (VEH_TURN_LAMP_LEFT):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_LEFT);
    break;
  case (VEH_TURN_LAMP_EMERGENCY):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_EMERGENCY);
    break;
  default:
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_INVALID);
    break;
  }
  // 档位 Number
  data_out->set_gear_number(msg.selected_gear_number);
  // data_out->set_gear_number(msg.gear_number);
  // 转向拨杆信号
  switch (msg.signal_turning_indicator) {
  case (VEH_TURNING_INDICATOR_NONE):
    data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_NONE);
    break;
  case (VEH_TURNING_INDICATOR_LEFT):
    data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_LEFT);
    break;
  case (VEH_TURNING_INDICATOR_RIGHT):
    data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_RIGHT);
    break;
  default:
    data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_INVALID);
    break;
  }

  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (msg.signal_brake_lamp) {
  case (VEH_LAMP_OFF):
    data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_OFF);
    break;
  case (VEH_LAMP_ON):
    data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_ON);
    break;
  default:
    data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_INVALID);
    break;
  }
 
  // 制动踏板深度, PH=[百分比][0,100]
  data_out->set_brake_pedal_value(msg.brake_pedal_value);
  // 油门踏板深度, PH=[百分比][0,100]
  data_out->set_acc_pedal_value(msg.acc_pedal_value);

  // 发动机转速有效位
  data_out->set_engine_speed_valid(msg.engine_speed_valid);
  // 发动机转速
  data_out->set_engine_speed(msg.engine_speed);
  // 发动机转矩有效位
  data_out->set_engine_torque_valid(msg.engine_torque_valid);
  // 发动机转矩(N.m)
  data_out->set_engine_torque(msg.engine_torque);

  // 总质量有效位
  data_out->set_gross_weight_valid(msg.gross_weight_valid);
  // 总质量 (kg)
  data_out->set_gross_weight(msg.gross_weight);

  // 挂车状态
  switch (msg.trailer_status) {
  case (VEH_TRAILER_STATUS_NOT_CONNECTED):
    data_out->set_trailer_status(msg::control::Chassis::TRAILER_STATUS_NOT_CONNECTED);
    break;
  case (VEH_TRAILER_STATUS_CONNECTED):
    data_out->set_trailer_status(msg::control::Chassis::TRAILER_STATUS_CONNECTED);
    break;
  default:
    data_out->set_trailer_status(msg::control::Chassis::TRAILER_STATUS_NOT_CONNECTED);
    break;
  }

  return true;
}

bool ParseProtoMsg::DecodeChassisMessage(
    const Char_t* msg, Int32_t msg_len,
    Chassis_t* data_out) {
  // parse
  msg::control::Chassis chassis;
  if (!chassis.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 驾驶模式
  switch (chassis.driving_mode()) {
  case (msg::control::Chassis::DRIVING_MODE_MANUAL):
    data_out->driving_mode = VEH_DRIVING_MODE_MANUAL;
    break;
  case (msg::control::Chassis::DRIVING_MODE_ROBOTIC):
    data_out->driving_mode = VEH_DRIVING_MODE_ROBOTIC;
    break;
  default:
    data_out->driving_mode = VEH_DRIVING_MODE_INVALID;
    break;
  }

  // 紧急停止信号
  switch (chassis.e_stop()) {
  case (msg::control::Chassis::E_STOP_OFF):
    data_out->e_stop = VEH_E_STOP_OFF;
    break;
  case (msg::control::Chassis::E_STOP_ON):
    data_out->e_stop = VEH_E_STOP_ON;
    break;
  default:
    data_out->e_stop = VEH_E_STOP_INVALID;
    break;
  }

  // 方向盘控制状态
  switch (chassis.eps_status()) {
  case (msg::control::Chassis::EPS_STATUS_MANUAL):
    data_out->eps_status = VEH_EPS_STATUS_MANUAL;
    break;
  case (msg::control::Chassis::EPS_STATUS_ROBOTIC):
    data_out->eps_status = VEH_EPS_STATUS_ROBOTIC;
    break;
  case (msg::control::Chassis::EPS_STATUS_MANUAL_INTERRUPT):
    data_out->eps_status = VEH_EPS_STATUS_MANUAL_INTERRUPT;
    break;
  case (msg::control::Chassis::EPS_STATUS_ERROR):
    data_out->eps_status = VEH_EPS_STATUS_ERROR;
    break;
  default:
    data_out->eps_status = VEH_EPS_STATUS_INVALID;
  }

  // 油门系统控制状态
  switch (chassis.throttle_sys_status()) {
  case (msg::control::Chassis::THROTTLE_SYS_STATUS_MANUAL):
    data_out->throttle_sys_status = VEH_THROTTLE_SYS_STATUS_MANUAL;
    break;
  case (msg::control::Chassis::THROTTLE_SYS_STATUS_ROBOTIC):
    data_out->throttle_sys_status = VEH_THROTTLE_SYS_STATUS_ROBOTIC;
    break;
  case (msg::control::Chassis::THROTTLE_SYS_STATUS_ERROR):
    data_out->throttle_sys_status = VEH_THROTTLE_SYS_STATUS_ERROR;
    break;
  default:
    data_out->throttle_sys_status = VEH_THROTTLE_SYS_STATUS_INVALID;
  }

  // 制动系统控制状态
  switch (chassis.ebs_status()) {
  case (msg::control::Chassis::EBS_STATUS_MANUAL):
    data_out->ebs_status = VEH_EBS_STATUS_MANUAL;
    break;
  case (msg::control::Chassis::EBS_STATUS_ROBOTIC):
    data_out->ebs_status = VEH_EBS_STATUS_ROBOTIC;
    break;
  case (msg::control::Chassis::EBS_STATUS_ERROR):
    data_out->ebs_status = VEH_EBS_STATUS_ERROR;
    break;
  default:
    data_out->ebs_status = VEH_EBS_STATUS_INVALID;
  }

  // 方向盘角度有效位
  data_out->steering_wheel_angle_valid = chassis.steering_wheel_angle_valid();
  // 方向盘角度 (rad)
  data_out->steering_wheel_angle = chassis.steering_wheel_angle();
  // 方向盘转速有效位
  data_out->steering_wheel_speed_valid = chassis.steering_wheel_speed_valid();
  // 方向盘转速 (rad/s)
  data_out->steering_wheel_speed = chassis.steering_wheel_speed();
  // 实际转向扭矩有效位
  data_out->steering_wheel_torque_valid = chassis.steering_wheel_torque_valid();
  // 实际转向扭矩(N.m)
  data_out->steering_wheel_torque = chassis.steering_wheel_torque();

  // 车速有效位
  data_out->v_valid = chassis.velocity_valid();
  // 车速(m/s)
  data_out->v = chassis.velocity();
  // 加速度有效位
  data_out->a_valid = chassis.acceleration_valid();
  // 加速度
  data_out->a = chassis.acceleration();
  // Yaw Rate有效位
  data_out->yaw_rate_valid = chassis.yaw_rate_valid();
  // Yaw Rate(rad/s), PH:[-2.0943rad/s,+2.0943rad/s]
  data_out->yaw_rate = chassis.yaw_rate();
  // AX 有效位
  data_out->ax_valid = chassis.ax_valid();
  // AX(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->ax = chassis.ax();
  // AY 有效位
  data_out->ay_valid = chassis.ay_valid();
  // AY(m/s^2), PH:[-21.593m/s^2,+21.593m/s^2]
  data_out->ay = chassis.ay();

  // 左前轮速有效位
  data_out->wheel_speed_fl_valid = chassis.wheel_speed_fl_valid();
  // 左前轮速(m/s)
  data_out->wheel_speed_fl = chassis.wheel_speed_fl();
  // 右前轮速有效位
  data_out->wheel_speed_fr_valid = chassis.wheel_speed_fr_valid();
  // 右前轮速(m/s)
  data_out->wheel_speed_fr = chassis.wheel_speed_fr();
  // 左后轮速有效位
  data_out->wheel_speed_rl_valid = chassis.wheel_speed_rl_valid();
  // 左后轮速(m/s)
  data_out->wheel_speed_rl = chassis.wheel_speed_rl();
  // 右后轮速有效位
  data_out->wheel_speed_rr_valid = chassis.wheel_speed_rr_valid();
  // 右后轮速(m/s)
  data_out->wheel_speed_rr = chassis.wheel_speed_rr();
  // 左后2#轮速有效位
  data_out->wheel_speed_rl2_valid = chassis.wheel_speed_rl2_valid();
  // 左后2#轮速(m/s)
  data_out->wheel_speed_rl2 = chassis.wheel_speed_rl2();
  // 右后2#轮速有效位
  data_out->wheel_speed_rr2_valid = chassis.wheel_speed_rr2_valid();
  // 右后2#轮速(m/s)
  data_out->wheel_speed_rr2 = chassis.wheel_speed_rr2();

  // 驻车系统状态
  switch (chassis.epb_status()) {
  case (msg::control::Chassis::EPB_STATUS_OFF):
    data_out->epb_status = VEH_EPB_STATUS_OFF;
    break;
  case (msg::control::Chassis::EPB_STATUS_ON):
    data_out->epb_status = VEH_EPB_STATUS_ON;
    break;
  default:
    data_out->epb_status = VEH_EPB_STATUS_INVALID;
    break;
  }

  // 档位
  switch (chassis.gear()) {
  case (msg::control::Chassis::GEAR_P):
    data_out->gear = VEH_GEAR_P;
    break;
  case (msg::control::Chassis::GEAR_N):
    data_out->gear = VEH_GEAR_N;
    break;
  case (msg::control::Chassis::GEAR_R):
    data_out->gear = VEH_GEAR_R;
    break;
  case (msg::control::Chassis::GEAR_D):
    data_out->gear = VEH_GEAR_D;
    break;
  default:
    data_out->gear = VEH_GEAR_INVALID;
    break;
  }
  // 档位 Number
  data_out->gear_number = chassis.gear_number();

  // 转向拨杆信号
  switch (chassis.signal_turning_indicator()) {
  case (msg::control::Chassis::TURNING_INDICATOR_NONE):
    data_out->signal_turning_indicator = VEH_TURNING_INDICATOR_NONE;
    break;
  case (msg::control::Chassis::TURNING_INDICATOR_LEFT):
    data_out->signal_turning_indicator = VEH_TURNING_INDICATOR_LEFT;
    break;
  case (msg::control::Chassis::TURNING_INDICATOR_RIGHT):
    data_out->signal_turning_indicator = VEH_TURNING_INDICATOR_RIGHT;
    break;
  default:
    data_out->signal_turning_indicator = VEH_TURNING_INDICATOR_INVALID;
    break;
  }

  // 转向灯信号
  switch (chassis.signal_turn_lamp()) {
  case (msg::control::Chassis::TURN_LAMP_OFF):
    data_out->signal_turn_lamp = VEH_TURN_LAMP_OFF;
    break;
  case (msg::control::Chassis::TURN_LAMP_LEFT):
    data_out->signal_turn_lamp = VEH_TURN_LAMP_LEFT;
    break;
  case (msg::control::Chassis::TURN_LAMP_RIGHT):
    data_out->signal_turn_lamp = VEH_TURN_LAMP_RIGHT;
    break;
  case (msg::control::Chassis::TURN_LAMP_EMERGENCY):
    data_out->signal_turn_lamp = VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    data_out->signal_turn_lamp = VEH_TURN_LAMP_INVALID;
    break;
  }

  // 制动灯信号
  switch (chassis.signal_brake_lamp()) {
  case (msg::control::Chassis::LAMP_OFF):
    data_out->signal_brake_lamp = VEH_LAMP_OFF;
    break;
  case (msg::control::Chassis::LAMP_ON):
    data_out->signal_brake_lamp = VEH_LAMP_ON;
    break;
  default:
    data_out->signal_brake_lamp = VEH_LAMP_INVALID;
    break;
  }

  // 制动踏板深度, PH=[百分比][0,100]
  data_out->brake_pedal_value = chassis.brake_pedal_value();
  // 油门踏板深度, PH=[百分比][0,100]
  data_out->acc_pedal_value = chassis.acc_pedal_value();

  // 发动机转速有效位
  data_out->engine_speed_valid = chassis.engine_speed_valid();
  // 发动机转速
  data_out->engine_speed = chassis.engine_speed();
  // 发动机转矩有效位
  data_out->engine_torque_valid = chassis.engine_torque_valid();
  // 发动机转矩(N.m)
  data_out->engine_torque = chassis.engine_torque();

  // 总质量有效位
  data_out->gross_weight_valid = chassis.gross_weight_valid();
  // 总质量 (kg)
  data_out->gross_weight = chassis.gross_weight();

  // 挂车状态
  switch (chassis.trailer_status()) {
  case (msg::control::Chassis::TRAILER_STATUS_NOT_CONNECTED ):
    data_out->trailer_status = VEH_TRAILER_STATUS_NOT_CONNECTED;
    break;
  case (msg::control::Chassis::TRAILER_STATUS_CONNECTED ):
    data_out->trailer_status = VEH_TRAILER_STATUS_CONNECTED;
    break;
  default:
    data_out->trailer_status = VEH_TRAILER_STATUS_NOT_CONNECTED;
    break;
  }

  // Update message head
  data_out->msg_head.valid = true;
  Phoenix_AdMsg_UpdateSequenceNum(&(data_out->msg_head), 1);
  data_out->msg_head.timestamp = Phoenix_Common_GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeSpecialChassisInfoMessage(
    const SpecialChassisInfo_t& msg,
    msg::control::SpecialChassisInfo* const data_out) {
  // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  data_out->set_start_adas(msg.start_adas);
  // LKA功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->set_enable_lka(msg.enable_lka);
  // ACC功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->set_enable_acc(msg.enable_acc);
  // AEB功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->set_enable_aeb(msg.enable_aeb);
  // 变道功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->set_enable_alc(msg.enable_alc);
  // 智能限速功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->set_enable_isl(msg.enable_isl);
  // Navigation Guided Pilot: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->set_enable_ngp(msg.enable_ngp);

  // 目标车速[m/s]
  data_out->set_target_velocity_valid(msg.target_velocity_valid);
  data_out->set_target_velocity(msg.target_velocity);
  // 加速度[m/s^2]
  data_out->set_target_acc_valid(msg.target_acc_valid);
  data_out->set_target_acc(msg.target_acc);
  // 时距[s]
  data_out->set_target_time_gap_valid(msg.target_time_gap_valid);
  data_out->set_target_time_gap(msg.target_time_gap);
  // 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  data_out->set_changing_lane_req(msg.changing_lane_req);

  // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  data_out->set_cnt_stu_frame_loss_can0(msg.cnt_stu_frame_loss_can0);
  data_out->set_cnt_stu_frame_loss_can1(msg.cnt_stu_frame_loss_can1);
  data_out->set_cnt_stu_frame_loss_can2(msg.cnt_stu_frame_loss_can2);
  data_out->set_cnt_stu_frame_loss_can3(msg.cnt_stu_frame_loss_can3);

  // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->set_cnt_stu_gtw_to_veh_can0(msg.cnt_stu_gtw_to_veh_can0);
  data_out->set_cnt_stu_gtw_to_veh_can1(msg.cnt_stu_gtw_to_veh_can1);
  data_out->set_cnt_stu_gtw_to_veh_can2(msg.cnt_stu_gtw_to_veh_can2);
  data_out->set_cnt_stu_gtw_to_veh_can3(msg.cnt_stu_gtw_to_veh_can3);

  // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->set_cnt_stu_ctl_to_gtw_can0(msg.cnt_stu_ctl_to_gtw_can0);
  data_out->set_cnt_stu_ctl_to_gtw_can1(msg.cnt_stu_ctl_to_gtw_can1);
  data_out->set_cnt_stu_ctl_to_gtw_can2(msg.cnt_stu_ctl_to_gtw_can2);
  data_out->set_cnt_stu_ctl_to_gtw_can3(msg.cnt_stu_ctl_to_gtw_can3);

  // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  data_out->set_cnt_stu_ctl_to_gtw(msg.cnt_stu_ctl_to_gtw);

  // 具体车型的特殊信息 (FT-Auman)
  msg::control::ChassisFtAuman* ft_auman = data_out->mutable_ft_auman();
  ft_auman->set_switch_tja(msg.ft_auman.switch_tja);
  ft_auman->set_switch_hwa(msg.ft_auman.switch_hwa);
  ft_auman->set_switch_i_drive(msg.ft_auman.switch_i_drive);

  return (true);
}

bool ParseProtoMsg::DecodeSpecialChassisInfoMessage(
    const Char_t* msg, Int32_t msg_len,
    SpecialChassisInfo_t* data_out) {
  // parse
  msg::control::SpecialChassisInfo special_chassis_info;
  if (!special_chassis_info.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  data_out->start_adas = special_chassis_info.start_adas();
  // LKA功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->enable_lka = special_chassis_info.enable_lka();
  // ACC功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->enable_acc = special_chassis_info.enable_acc();
  // AEB功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->enable_aeb = special_chassis_info.enable_aeb();
  // 变道功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->enable_alc = special_chassis_info.enable_alc();
  // 智能限速功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->enable_isl = special_chassis_info.enable_isl();
  // Navigation Guided Pilot: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  data_out->enable_ngp = special_chassis_info.enable_ngp();

  // 目标车速[m/s]
  data_out->target_velocity_valid = special_chassis_info.target_velocity_valid();
  data_out->target_velocity = special_chassis_info.target_velocity();
  // 加速度[m/s^2]
  data_out->target_acc_valid = special_chassis_info.target_acc_valid();
  data_out->target_acc = special_chassis_info.target_acc();
  // 时距[s]
  data_out->target_time_gap_valid = special_chassis_info.target_time_gap_valid();
  data_out->target_time_gap = special_chassis_info.target_time_gap();
  // 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  data_out->changing_lane_req = special_chassis_info.changing_lane_req();

  // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
  data_out->cnt_stu_frame_loss_can0 =
      special_chassis_info.cnt_stu_frame_loss_can0();
  data_out->cnt_stu_frame_loss_can1 =
      special_chassis_info.cnt_stu_frame_loss_can1();
  data_out->cnt_stu_frame_loss_can2 =
      special_chassis_info.cnt_stu_frame_loss_can2();
  data_out->cnt_stu_frame_loss_can3 =
      special_chassis_info.cnt_stu_frame_loss_can3();

  // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->cnt_stu_gtw_to_veh_can0 =
      special_chassis_info.cnt_stu_gtw_to_veh_can0();
  data_out->cnt_stu_gtw_to_veh_can1 =
      special_chassis_info.cnt_stu_gtw_to_veh_can1();
  data_out->cnt_stu_gtw_to_veh_can2 =
      special_chassis_info.cnt_stu_gtw_to_veh_can2();
  data_out->cnt_stu_gtw_to_veh_can3 =
      special_chassis_info.cnt_stu_gtw_to_veh_can3();

  // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
  data_out->cnt_stu_ctl_to_gtw_can0 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can0();
  data_out->cnt_stu_ctl_to_gtw_can1 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can1();
  data_out->cnt_stu_ctl_to_gtw_can2 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can2();
  data_out->cnt_stu_ctl_to_gtw_can3 =
      special_chassis_info.cnt_stu_ctl_to_gtw_can3();

  // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
  data_out->cnt_stu_ctl_to_gtw =
      special_chassis_info.cnt_stu_ctl_to_gtw();

  // 具体车型的特殊信息 (FT-Auman)
  const msg::control::ChassisFtAuman& ft_auman = special_chassis_info.ft_auman();
  data_out->ft_auman.switch_tja = ft_auman.switch_tja();
  data_out->ft_auman.switch_hwa = ft_auman.switch_hwa();
  data_out->ft_auman.switch_i_drive = ft_auman.switch_i_drive();

  // Update message head
  data_out->msg_head.valid = true;
  Phoenix_AdMsg_UpdateSequenceNum(&(data_out->msg_head), 1);
  data_out->msg_head.timestamp = Phoenix_Common_GetClockNowMs();

  return true;
}

bool ParseProtoMsg::DecodeDFCVSpecialChassisInfoMessage(
    const Char_t* msg, Int32_t msg_len,
    DFCV_SpecialChassisInfo_t* data_out) {
  // parse
  msg::control::ChassisDFCV chassis_dfcv;
  if (!chassis_dfcv.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse dfcv chassis from array.";
    return false;
  }

  //制动设备源地址
  data_out->source_address_brake_control_device =
      chassis_dfcv.source_address_brake_control_device();
  //整车总质量（含挂车）
  data_out->vehicle_mass =
      chassis_dfcv.vehicle_mass();
  //左前轮制动压力
  data_out->brake_pressure_lf =
      chassis_dfcv.brake_pressure_lf();
  //当前档位传动比
  data_out->current_gear_ratio =
      chassis_dfcv.current_gear_ratio();
  //变速箱选择的档位
  data_out->transmission_selected_gear =
      chassis_dfcv.transmission_selected_gear();
  //离合器状态
  data_out->clutch_switch =
      chassis_dfcv.clutch_switch();
  //发动机名义摩擦扭矩百分比
  data_out->nominal_fricton_troque_percent =
      chassis_dfcv.nominal_fricton_troque_percent();
  //附件损失扭矩百分比
  data_out->estimated_lossed_torque_percent =
      chassis_dfcv.estimated_lossed_torque_percent();
  //f发动机实际扭矩百分比
  data_out->actual_engine_torque_percent =
      chassis_dfcv.actual_engine_torque_percent();
  //驾驶员请求的扭矩百分比
  data_out->driver_damand_torque_percent =
      chassis_dfcv.driver_damand_torque_percent();
  //发动机设备控制源地址.
  data_out->source_address_engine_control_device =
      chassis_dfcv.source_address_engine_control_device();
  //变速箱换档状态
  data_out->transmission_shift_status =
      chassis_dfcv.transmission_shift_status();
  //变速箱结合状态
  data_out->transmission_engage_status =
      chassis_dfcv.transmission_engage_status();
  //变速箱TSC1扭矩控制模式
  data_out->tcu_engine_control_mode =
      chassis_dfcv.tcu_engine_control_mode();
  //挂车状态
  data_out->trailer_connected_status =
      chassis_dfcv.trailer_connected_status();

  return true;
}

bool ParseProtoMsg::EncodeChassisCtlCmdMessage(
    const ChassisCtlCmd_t& msg,
    msg::control::ChassisCtlCmd* const data_out) {
  // 开始自动驾驶命令
  data_out->set_start_robotic_ctl(msg.start_robotic_ctl);
  // 使能转向控制系统
  data_out->set_enable_eps(msg.enable_eps);
  // 使能油门控制系统
  data_out->set_enable_throttle_sys(msg.enable_throttle_sys);
  // 使能制动控制系统
  data_out->set_enable_ebs(msg.enable_ebs);

  // 使能远程控制
  data_out->set_enable_remote_ctl(msg.enable_remote_ctl);
  // 使能直接控制模式
  data_out->set_enable_direct_ctl(msg.enable_direct_ctl);
  // 使能速度控制
  data_out->set_enable_acc(msg.enable_acc);
  // 释放油门控制
  data_out->set_release_throttle(msg.release_throttle);

  // 方向盘角度 (rad)
  data_out->set_steering_wheel_angle(msg.steering_wheel_angle);
  // 方向盘转速 (rad/s)
  data_out->set_steering_wheel_speed(msg.steering_wheel_speed);
  // 车速(m/s)
  data_out->set_velocity(msg.velocity);
  // 加速度
  data_out->set_acceleration(msg.acceleration);
  // 加速度量
  data_out->set_acc_value(msg.acc_value);
  // 刹车量
  data_out->set_brake_value(msg.brake_value);

  // 档位
  switch (msg.gear) {
  case (VEH_GEAR_P):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_P);
    break;
  case (VEH_GEAR_N):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_N);
    break;
  case (VEH_GEAR_R):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_R);
    break;
  case (VEH_GEAR_D):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_D);
    break;
  default:
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_INVALID);
    break;
  }
  // 转向灯信号
  switch (msg.turn_lamp) {
  case (VEH_TURN_LAMP_OFF):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_OFF);
    break;
  case (VEH_TURN_LAMP_RIGHT):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_RIGHT);
    break;
  case (VEH_TURN_LAMP_LEFT):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_LEFT);
    break;
  case (VEH_TURN_LAMP_EMERGENCY):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_EMERGENCY);
    break;
  default:
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_INVALID);
    break;
  }

  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (msg.brake_lamp) {
  case (VEH_LAMP_OFF):
    data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_OFF);
    break;
  case (VEH_LAMP_ON):
    data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_ON);
    break;
  default:
    data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_INVALID);
    break;
  }

  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (msg.epb_status) {
  case (VEH_EPB_STATUS_OFF):
    data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_OFF);
    break;
  case (VEH_EPB_STATUS_ON):
    data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_ON);
    break;
  default:
    data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_INVALID);
    break;
  }

  // 雨刮器状态, 0x0:无效，0x1:关闭, 0x2:SHORT PRESS WITH CLICK,
  //0x3:LONG PRESS WITH CLICK，0x4:INT 1, 0x5:INT 2，0x6:INT 3,
  //0x7:INT 4，0x8:LO，0x9:HI
  switch (msg.wiper) {
  case (VEH_WIPER_OFF):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_OFF);
    break;
  case (VEH_WIPER_SHORT_PRESS_WITH_CLICK):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_SHORT_PRESS_WITH_CLICK);
    break;
  case (VEH_WIPER_LONG_PRESS_WITH_CLICK):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_LONG_PRESS_WITH_CLICK);
    break;
  case (VEH_WIPER_INT_1):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_1);
    break;
  case (VEH_WIPER_INT_2):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_2);
    break;
  case (VEH_WIPER_INT_3):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_3);
    break;
  case (VEH_WIPER_INT_4):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_4);
    break;
  case (VEH_WIPER_LO):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_LO);
    break;
  case (VEH_WIPER_HI):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_HI);
    break;
  default:
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INVALID);
    break;
  }
/*
  // add debug message
  data_out->mutable_debug()->set_speed_integ_freeze_status(msg.debug.debug_speed_integ_freeze_status);
  data_out->mutable_debug()->set_speed_inieg_init_status(msg.debug.debug_speed_inieg_init_status);
  data_out->mutable_debug()->set_speed_error(msg.debug.debug_speed_error);
  
  data_out->mutable_debug()->set_speed_propo_value(msg.debug.debug_speed_propo_value);
  data_out->mutable_debug()->set_speed_integ_value(msg.debug.debug_speed_integ_value);
  data_out->mutable_debug()->set_engine_torque_raw(msg.debug.debug_engine_torque_raw);
  data_out->mutable_debug()->set_speedup_shift_status(msg.debug.debug_speedup_shift_status);
  data_out->mutable_debug()->set_tcu_torque_status(msg.debug.debug_tcu_torque_status);
  data_out->mutable_debug()->set_vehicle_max_acce(msg.debug.debug_vehicle_max_acce);
  data_out->mutable_debug()->set_drive_trm_factor(msg.debug.debug_drive_trm_factor);
  data_out->mutable_debug()->set_longitudinal_control_status(msg.debug.debug_longitudinal_control_status);
*/
  return true;
}

bool ParseProtoMsg::DecodeChassisCtlCmdMessage(
    const Char_t* msg, Int32_t msg_len,
    ChassisCtlCmd_t* data_out) {
  // parse
  msg::control::ChassisCtlCmd chassis_ctl_cmd;
  if (!chassis_ctl_cmd.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 开始自动驾驶命令
  data_out->start_robotic_ctl = chassis_ctl_cmd.start_robotic_ctl();
  // 使能转向控制系统
  data_out->enable_eps = chassis_ctl_cmd.enable_eps();
  // 使能油门控制系统
  data_out->enable_throttle_sys = chassis_ctl_cmd.enable_throttle_sys();
  // 使能制动控制系统
  data_out->enable_ebs = chassis_ctl_cmd.enable_ebs();

  // 使能远程控制
  data_out->enable_remote_ctl = chassis_ctl_cmd.enable_remote_ctl();
  // 使能直接控制模式
  data_out->enable_direct_ctl = chassis_ctl_cmd.enable_direct_ctl();
  // 使能速度控制
  data_out->enable_acc = chassis_ctl_cmd.enable_acc();
  // 释放油门控制
  data_out->release_throttle = chassis_ctl_cmd.release_throttle();

  // 方向盘角度 (rad)
  data_out->steering_wheel_angle = chassis_ctl_cmd.steering_wheel_angle();
  // 方向盘转速 (rad/s)
  data_out->steering_wheel_speed = chassis_ctl_cmd.steering_wheel_speed();
  // 车速(m/s)
  data_out->velocity = chassis_ctl_cmd.velocity();
  // 加速度
  data_out->acceleration = chassis_ctl_cmd.acceleration();
  // 加速度量
  data_out->acc_value = chassis_ctl_cmd.acc_value();
  // 刹车量
  data_out->brake_value = chassis_ctl_cmd.brake_value();

  // 档位
  switch (chassis_ctl_cmd.gear()) {
  case (msg::control::ChassisCtlCmd::GEAR_P):
    data_out->gear = VEH_GEAR_P;
    break;
  case (msg::control::ChassisCtlCmd::GEAR_N):
    data_out->gear = VEH_GEAR_N;
    break;
  case (msg::control::ChassisCtlCmd::GEAR_R):
    data_out->gear = VEH_GEAR_R;
    break;
  case (msg::control::ChassisCtlCmd::GEAR_D):
    data_out->gear = VEH_GEAR_D;
    break;
  default:
    data_out->gear = VEH_GEAR_INVALID;
    break;
  }
  // 转向灯信号
  switch (chassis_ctl_cmd.turn_lamp()) {
  case (msg::control::ChassisCtlCmd::TURN_LAMP_OFF):
    data_out->turn_lamp = VEH_TURN_LAMP_OFF;
    break;
  case (msg::control::ChassisCtlCmd::TURN_LAMP_LEFT):
    data_out->turn_lamp = VEH_TURN_LAMP_LEFT;
    break;
  case (msg::control::ChassisCtlCmd::TURN_LAMP_RIGHT):
    data_out->turn_lamp = VEH_TURN_LAMP_RIGHT;
    break;
  case (msg::control::ChassisCtlCmd::TURN_LAMP_EMERGENCY):
    data_out->turn_lamp = VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    data_out->turn_lamp = VEH_TURN_LAMP_INVALID;
    break;
  }
  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (chassis_ctl_cmd.brake_lamp()) {
  case (msg::control::ChassisCtlCmd::LAMP_OFF):
    data_out->brake_lamp = VEH_LAMP_OFF;
    break;
  case (msg::control::ChassisCtlCmd::LAMP_ON):
    data_out->brake_lamp = VEH_LAMP_ON;
    break;
  default:
    data_out->brake_lamp = VEH_LAMP_INVALID;
    break;
  }
  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (chassis_ctl_cmd.epb_status()) {
  case (msg::control::ChassisCtlCmd::EPB_STATUS_OFF):
    data_out->epb_status = VEH_EPB_STATUS_OFF;
    break;
  case (msg::control::ChassisCtlCmd::EPB_STATUS_ON):
    data_out->epb_status = VEH_EPB_STATUS_ON;
    break;
  default:
    data_out->epb_status = VEH_EPB_STATUS_INVALID;
    break;
  }
  // 雨刮器状态, 0x0:无效，0x1:关闭, 0x2:SHORT PRESS WITH CLICK,
  //0x3:LONG PRESS WITH CLICK，0x4:INT 1, 0x5:INT 2，0x6:INT 3,
  //0x7:INT 4，0x8:LO，0x9:HI
  switch (chassis_ctl_cmd.wiper()) {
  case (msg::control::ChassisCtlCmd::WIPER_OFF):
    data_out->wiper = VEH_WIPER_OFF;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_SHORT_PRESS_WITH_CLICK):
    data_out->wiper = VEH_WIPER_SHORT_PRESS_WITH_CLICK;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_LONG_PRESS_WITH_CLICK):
    data_out->wiper = VEH_WIPER_LONG_PRESS_WITH_CLICK;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_1):
    data_out->wiper = VEH_WIPER_INT_1;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_2):
    data_out->wiper = VEH_WIPER_INT_2;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_3):
    data_out->wiper = VEH_WIPER_INT_3;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_4):
    data_out->wiper = VEH_WIPER_INT_4;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_LO):
    data_out->wiper = VEH_WIPER_LO;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_HI):
    data_out->wiper = VEH_WIPER_HI;
    break;
  default:
    data_out->wiper = VEH_WIPER_INVALID;
    break;
  }

  // Update message head
  data_out->msg_head.valid = true;
  Phoenix_AdMsg_UpdateSequenceNum(&(data_out->msg_head), 1);
  data_out->msg_head.timestamp = Phoenix_Common_GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodePlanningResultMessage(
    const PlanningResult_t& msg,
    msg::planning::PlanningResult* const data_out) {
  // 当前决策模块的状态
  data_out->add_planning_status(msg.planning_status[0]);
  data_out->add_planning_status(msg.planning_status[1]);
  data_out->add_planning_status(msg.planning_status[2]);
  data_out->add_planning_status(msg.planning_status[3]);

  // 驾驶模式请求
  switch (msg.tar_driving_mode) {
  case (VEH_DRIVING_MODE_MANUAL):
    data_out->set_tar_driving_mode(
          msg::planning::PlanningResult::DRIVING_MODE_MANUAL);
    break;
  case (VEH_DRIVING_MODE_ROBOTIC):
    data_out->set_tar_driving_mode(
          msg::planning::PlanningResult::DRIVING_MODE_ROBOTIC);
    break;
  default:
    data_out->set_tar_driving_mode(
          msg::planning::PlanningResult::DRIVING_MODE_INVALID);
    break;
  }
  // 使能转向系统
  data_out->set_enable_eps(msg.enable_eps);
  // 使能油门系统
  data_out->set_enable_throttle_sys(msg.enable_throttle_sys);
  // 使能制动系统
  data_out->set_enable_ebs(msg.enable_ebs);

  // 保持当前方向盘角度不变
  data_out->set_hold_steering_wheel(msg.hold_steering_wheel);
  // 方向盘转速限制 (rad/s)
  data_out->set_steering_wheel_speed(msg.steering_wheel_speed);
  // 释放油门
  data_out->set_release_throttle(msg.release_throttle);

  // 档位请求
  switch (msg.tar_gear) {
  case (VEH_GEAR_P):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_P);
    break;
  case (VEH_GEAR_N):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_N);
    break;
  case (VEH_GEAR_R):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_R);
    break;
  case (VEH_GEAR_D):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_D);
    break;
  default:
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_INVALID);
    break;
  }
  // 转向指示灯请求
  switch (msg.tar_turn_lamp) {
  case (VEH_TURN_LAMP_OFF):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_OFF);
    break;
  case (VEH_TURN_LAMP_LEFT):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_LEFT);
    break;
  case (VEH_TURN_LAMP_RIGHT):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_RIGHT);
    break;
  case (VEH_TURN_LAMP_EMERGENCY):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_EMERGENCY);
    break;
  default:
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_INVALID);
    break;
  }
  // 制动灯请求
  switch (msg.tar_brake_lamp) {
  case (VEH_LAMP_OFF):
    data_out->set_tar_brake_lamp(msg::planning::PlanningResult::LAMP_OFF);
    break;
  case (VEH_LAMP_ON):
    data_out->set_tar_brake_lamp(msg::planning::PlanningResult::LAMP_ON);
    break;
  default:
    data_out->set_tar_brake_lamp(msg::planning::PlanningResult::LAMP_INVALID);
    break;
  }

  // 速度请求
  data_out->set_tar_v(msg.tar_v);
  // 加速度请求
  data_out->set_tar_a(msg.tar_a);

  // 目标轨迹
  msg::planning::PlanningResult::TarTrj* tar_trj = data_out->mutable_tar_trj();
  // 目标轨迹的时间戳，用于时间及空间同步
  tar_trj->set_timestamp(msg.tar_trj.timestamp);
  // 当前车辆位置
  msg::planning::PlanningResult::TarTrj::Pos* curr_pos =
      tar_trj->mutable_curr_pos();
  curr_pos->set_x(msg.tar_trj.curr_pos.x);
  curr_pos->set_y(msg.tar_trj.curr_pos.y);
  curr_pos->set_h(msg.tar_trj.curr_pos.h);
  curr_pos->set_c(msg.tar_trj.curr_pos.c);
  curr_pos->set_s(msg.tar_trj.curr_pos.s);
  curr_pos->set_l(msg.tar_trj.curr_pos.l);
  // 先导点位置
  msg::planning::PlanningResult::TarTrj::Pos* leading_pos =
      tar_trj->mutable_leading_pos();
  leading_pos->set_x(msg.tar_trj.leading_pos.x);
  leading_pos->set_y(msg.tar_trj.leading_pos.y);
  leading_pos->set_h(msg.tar_trj.leading_pos.h);
  leading_pos->set_c(msg.tar_trj.leading_pos.c);
  leading_pos->set_s(msg.tar_trj.leading_pos.s);
  leading_pos->set_l(msg.tar_trj.leading_pos.l);
  // 横向误差
  msg::planning::PlanningResult::TarTrj::LatErr* lat_err =
      tar_trj->mutable_lat_err();
  lat_err->set_moving_flag(msg.tar_trj.lat_err.moving_flag);
  msg::planning::PlanningResult::TarTrj::LatErr::Samples* lat_err_sample =
      lat_err->add_samples();
  lat_err_sample->set_lat_err(msg.tar_trj.lat_err.samples[0].lat_err);
  lat_err_sample->set_lat_err_chg_rate(msg.tar_trj.lat_err.samples[0].lat_err_chg_rate);
  lat_err_sample->set_yaw_err(msg.tar_trj.lat_err.samples[0].yaw_err);
  lat_err_sample->set_yaw_err_chg_rate(msg.tar_trj.lat_err.samples[0].yaw_err_chg_rate);
  lat_err_sample = lat_err->add_samples();
  lat_err_sample->set_lat_err(msg.tar_trj.lat_err.samples[1].lat_err);
  lat_err_sample->set_lat_err_chg_rate(msg.tar_trj.lat_err.samples[1].lat_err_chg_rate);
  lat_err_sample->set_yaw_err(msg.tar_trj.lat_err.samples[1].yaw_err);
  lat_err_sample->set_yaw_err_chg_rate(msg.tar_trj.lat_err.samples[1].yaw_err_chg_rate);
  // 轨迹的方向， 0 - 前进， 1 - 后退
  switch (msg.tar_trj.trj_direction) {
  case (AD_MSG_TRJ_DIRECTION_FORWARD):
    tar_trj->set_trj_direction(msg::planning::PlanningResult::TRJ_DIRECTION_FORWARD);
    break;
  case (AD_MSG_TRJ_DIRECTION_BACKWARD):
    tar_trj->set_trj_direction(msg::planning::PlanningResult::TRJ_DIRECTION_BACKWARD);
    break;
  default:
    tar_trj->set_trj_direction(msg::planning::PlanningResult::TRJ_DIRECTION_FORWARD);
    LOG_ERR << "Invalid trajectory direction.";
    break;
  }
  // 轨迹点的数量
  tar_trj->set_points_num(msg.tar_trj.points_num);
  // 轨迹点
  for (Int32_t i = 0; i < msg.tar_trj.points_num; ++i) {
    msg::planning::PlanningResult::TarTrj::TrjPoint* dest_p =
        tar_trj->add_points();

    dest_p->set_x(msg.tar_trj.points[i].x);
    dest_p->set_y(msg.tar_trj.points[i].y);
    dest_p->set_h(msg.tar_trj.points[i].h);
    dest_p->set_c(msg.tar_trj.points[i].c);
    dest_p->set_s(msg.tar_trj.points[i].s);
  }

  return true;
}

bool ParseProtoMsg::DecodePlanningResultMessage(
    const Char_t* msg, Int32_t msg_len,
    PlanningResult_t* data_out) {
  // parse
  msg::planning::PlanningResult planning_result;
  if (!planning_result.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse planning_result from array.";
    return false;
  }

  // 当前决策模块的状态
  data_out->planning_status[0] = planning_result.planning_status(0);
  data_out->planning_status[1] = planning_result.planning_status(1);
  data_out->planning_status[2] = planning_result.planning_status(2);
  data_out->planning_status[3] = planning_result.planning_status(3);
  // 驾驶模式请求
  switch (planning_result.tar_driving_mode()) {
  case (msg::planning::PlanningResult::DRIVING_MODE_MANUAL):
    data_out->tar_driving_mode = VEH_DRIVING_MODE_MANUAL;
    break;
  case (msg::planning::PlanningResult::DRIVING_MODE_ROBOTIC):
    data_out->tar_driving_mode = VEH_DRIVING_MODE_ROBOTIC;
    break;
  default:
    data_out->tar_driving_mode = VEH_DRIVING_MODE_INVALID;
    break;
  }
  // 使能转向系统
  data_out->enable_eps = planning_result.enable_eps();
  // 使能油门系统
  data_out->enable_throttle_sys = planning_result.enable_throttle_sys();
  // 使能制动系统
  data_out->enable_ebs = planning_result.enable_ebs();

  // 保持当前方向盘角度不变
  data_out->hold_steering_wheel = planning_result.hold_steering_wheel();
  // 方向盘转速限制 (rad/s)
  data_out->steering_wheel_speed = planning_result.steering_wheel_speed();
  // 释放油门
  data_out->release_throttle = planning_result.release_throttle();

  // 档位请求
  switch (planning_result.tar_gear()) {
  case (msg::planning::PlanningResult::GEAR_P):
    data_out->tar_gear = VEH_GEAR_P;
    break;
  case (msg::planning::PlanningResult::GEAR_N):
    data_out->tar_gear = VEH_GEAR_N;
    break;
  case (msg::planning::PlanningResult::GEAR_R):
    data_out->tar_gear = VEH_GEAR_R;
    break;
  case (msg::planning::PlanningResult::GEAR_D):
    data_out->tar_gear = VEH_GEAR_D;
    break;
  default:
    data_out->tar_gear = VEH_GEAR_INVALID;
    break;
  }
  // 转向指示灯请求
  switch (planning_result.tar_turn_lamp()) {
  case (msg::planning::PlanningResult::TURN_LAMP_OFF):
    data_out->tar_turn_lamp = VEH_TURN_LAMP_OFF;
    break;
  case (msg::planning::PlanningResult::TURN_LAMP_LEFT):
    data_out->tar_turn_lamp = VEH_TURN_LAMP_LEFT;
    break;
  case (msg::planning::PlanningResult::TURN_LAMP_RIGHT):
    data_out->tar_turn_lamp = VEH_TURN_LAMP_RIGHT;
    break;
  case (msg::planning::PlanningResult::TURN_LAMP_EMERGENCY):
    data_out->tar_turn_lamp = VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    data_out->tar_turn_lamp = VEH_TURN_LAMP_INVALID;
    break;
  }
  // 制动灯请求
  switch (planning_result.tar_brake_lamp()) {
  case (msg::planning::PlanningResult::LAMP_OFF):
    data_out->tar_brake_lamp = VEH_LAMP_OFF;
    break;
  case (msg::planning::PlanningResult::LAMP_ON):
    data_out->tar_brake_lamp = VEH_LAMP_ON;
    break;
  default:
    data_out->tar_brake_lamp = VEH_LAMP_INVALID;
    break;
  }
  // 速度请求, PH = INT * 0.1 (km/h), Range: [0.0, 200.0 km/h]
  data_out->tar_v = planning_result.tar_v();
  // 加速度请求, PH = INT * 0.01 (m/s^2), Range: [-15.00, 15.00 m/s^2]
  data_out->tar_a = planning_result.tar_a();


  data_out->ramp_slope_value = planning_result.ramp_slope_value();
  data_out->tar_throttle = planning_result.tar_throttle();
  
  // 目标轨迹
  // 目标轨迹的时间戳，用于时间及空间同步
  const msg::planning::PlanningResult::TarTrj& tar_trj =
      planning_result.tar_trj();
  data_out->tar_trj.timestamp = tar_trj.timestamp();
  // 当前车辆位置
  const msg::planning::PlanningResult::TarTrj::Pos& curr_pos =
      tar_trj.curr_pos();
  data_out->tar_trj.curr_pos.x = curr_pos.x();
  data_out->tar_trj.curr_pos.y = curr_pos.y();
  data_out->tar_trj.curr_pos.h = curr_pos.h();
  data_out->tar_trj.curr_pos.c = curr_pos.c();
  data_out->tar_trj.curr_pos.s = curr_pos.s();
  data_out->tar_trj.curr_pos.l = curr_pos.l();
  // 先导点位置
  const msg::planning::PlanningResult::TarTrj::Pos& leading_pos =
      tar_trj.leading_pos();
  data_out->tar_trj.leading_pos.x = leading_pos.x();
  data_out->tar_trj.leading_pos.y = leading_pos.y();
  data_out->tar_trj.leading_pos.h = leading_pos.h();
  data_out->tar_trj.leading_pos.c = leading_pos.c();
  data_out->tar_trj.leading_pos.s = leading_pos.s();
  data_out->tar_trj.leading_pos.l = leading_pos.l();
  // 横向误差
  const msg::planning::PlanningResult::TarTrj::LatErr& lat_err =
      tar_trj.lat_err();
  data_out->tar_trj.lat_err.moving_flag = lat_err.moving_flag();
  Int32_t lat_err_samples_num = lat_err.samples_size();
  if (2 == lat_err_samples_num) {
    const msg::planning::PlanningResult::TarTrj::LatErr::Samples& sample0 =
        lat_err.samples(0);
    data_out->tar_trj.lat_err.samples[0].lat_err = sample0.lat_err();
    data_out->tar_trj.lat_err.samples[0].lat_err_chg_rate = sample0.lat_err_chg_rate();
    data_out->tar_trj.lat_err.samples[0].yaw_err = sample0.yaw_err();
    data_out->tar_trj.lat_err.samples[0].yaw_err_chg_rate = sample0.yaw_err_chg_rate();

    const msg::planning::PlanningResult::TarTrj::LatErr::Samples& sample1 =
        lat_err.samples(1);
    data_out->tar_trj.lat_err.samples[1].lat_err = sample1.lat_err();
    data_out->tar_trj.lat_err.samples[1].lat_err_chg_rate = sample1.lat_err_chg_rate();
    data_out->tar_trj.lat_err.samples[1].yaw_err = sample1.yaw_err();
    data_out->tar_trj.lat_err.samples[1].yaw_err_chg_rate = sample1.yaw_err_chg_rate();
  } else {
    LOG_ERR << "Invalid lat err samples.";
  }
  // 轨迹的方向， 0 - 前进， 1 - 后退
  switch (tar_trj.trj_direction()) {
  case (msg::planning::PlanningResult::TRJ_DIRECTION_FORWARD):
    data_out->tar_trj.trj_direction =
        AD_MSG_TRJ_DIRECTION_FORWARD;
    break;
  case (msg::planning::PlanningResult::TRJ_DIRECTION_BACKWARD):
    data_out->tar_trj.trj_direction =
        AD_MSG_TRJ_DIRECTION_BACKWARD;
    break;
  default:
    data_out->tar_trj.trj_direction =
        AD_MSG_TRJ_DIRECTION_FORWARD;
    LOG_ERR << "Invalid trajectory direction.";
    break;
  }
  // 轨迹点的数量
  data_out->tar_trj.points_num = tar_trj.points_num();
  if (data_out->tar_trj.points_num > AD_MSG_MAX_TRAJECTORY_POINT_NUM) {
    data_out->tar_trj.points_num = AD_MSG_MAX_TRAJECTORY_POINT_NUM;
  }
  Int32_t point_index = 0;
  for (const msg::planning::PlanningResult::TarTrj::TrjPoint& p :
       tar_trj.points()) {
    data_out->tar_trj.points[point_index].x = p.x();
    data_out->tar_trj.points[point_index].y = p.y();
    data_out->tar_trj.points[point_index].h = p.h();
    data_out->tar_trj.points[point_index].c = p.c();
    data_out->tar_trj.points[point_index].s = p.s();

    point_index++;
    if (point_index >= data_out->tar_trj.points_num) {
      break;
    }
  }

#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
// 增加横向两条轨迹的数据读取
    const msg::planning::PlanningResult_TarTrj_PlanningTraj_Cv& planning_traj_p =
        tar_trj.traj_coeffs();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Cv0 = planning_traj_p.traj_cv0();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Cv1 = planning_traj_p.traj_cv1();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Cv2 = planning_traj_p.traj_cv2();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Cv3 = planning_traj_p.traj_cv3();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Cv4 = planning_traj_p.traj_cv4();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Cv5 = planning_traj_p.traj_cv5();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_Dist2PathZero = planning_traj_p.traj_dist2pathzero();
    data_out->tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid = planning_traj_p.traj_isvalid();


    for(Int32_t i = 0;i<6;i++)     {
      data_out->tar_trj.PlanningTraj_Cv.coeffs[i]  = planning_traj_p.traj_coeffs(i);
    }
 
      const msg::planning::PlanningResult::TarTrj::LaneTraj_Cv& lane_traj_p =
        tar_trj.lane_coeffs();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_Cv0 = lane_traj_p.lane_cv0();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_Cv1 = lane_traj_p.lane_cv1();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_Cv2 = lane_traj_p.lane_cv2();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_Cv3 = lane_traj_p.lane_cv3();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_Cv4 = lane_traj_p.lane_cv4();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_Cv5 = lane_traj_p.lane_cv5();
    data_out->tar_trj.LaneTraj_Cv.LaneDist2PathZero = lane_traj_p.lane_dist2path_zero();
    data_out->tar_trj.LaneTraj_Cv.LaneTraj_IsValid = lane_traj_p.lane_isvalid();
    for(Int32_t i = 0;i<6;i++)     {
      data_out->tar_trj.LaneTraj_Cv.c_center[i]  = lane_traj_p.lane_coeffs(i);
    }
#endif

  // Update message head
  data_out->msg_head.valid = true;
  Phoenix_AdMsg_UpdateSequenceNum(&(data_out->msg_head), 1);
  data_out->msg_head.timestamp = Phoenix_Common_GetClockNowMs();

  return true;
}


}  // namespace framework
}  // namespace phoenix

