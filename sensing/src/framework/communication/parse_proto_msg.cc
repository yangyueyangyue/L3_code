//
#include "communication/parse_proto_msg.h"

#include "utils/log.h"
#include "utils/gps_tools.h"
#include "math/math_utils.h"


namespace phoenix {
namespace framework {


bool ParseProtoMsg::EncodeGnssMessage(
    const ad_msg::Gnss& msg,
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
  case (ad_msg::Gnss::STATUS_BAD):
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_BAD);
    break;
  case (ad_msg::Gnss::STATUS_CONVERGING):
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_CONVERGING);
    break;
  case (msg::localization::Gnss::STATUS_GOOD):
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_GOOD);
    break;
  default:
    data_out->set_gnss_status(msg::localization::Gnss::STATUS_INVALID);
    break;
  }
  switch (msg.utm_status) {
  case (ad_msg::Gnss::STATUS_BAD):
    data_out->set_utm_status(msg::localization::Gnss::STATUS_BAD);
    break;
  case (ad_msg::Gnss::STATUS_CONVERGING):
    data_out->set_utm_status(msg::localization::Gnss::STATUS_CONVERGING);
    break;
  case (msg::localization::Gnss::STATUS_GOOD):
    data_out->set_utm_status(msg::localization::Gnss::STATUS_GOOD);
    break;
  default:
    data_out->set_utm_status(msg::localization::Gnss::STATUS_INVALID);
    break;
  }
  switch (msg.odom_status) {
  case (ad_msg::Gnss::STATUS_BAD):
    data_out->set_odom_status(msg::localization::Gnss::STATUS_BAD);
    break;
  case (ad_msg::Gnss::STATUS_CONVERGING):
    data_out->set_odom_status(msg::localization::Gnss::STATUS_CONVERGING);
    break;
  case (msg::localization::Gnss::STATUS_GOOD):
    data_out->set_odom_status(msg::localization::Gnss::STATUS_GOOD);
    break;
  default:
    data_out->set_odom_status(msg::localization::Gnss::STATUS_INVALID);
    break;
  }

  return true;
}

bool ParseProtoMsg::DecodeGnssMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::Gnss* data_out) {
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
    data_out->gnss_status = ad_msg::Gnss::STATUS_BAD;
    break;
  case msg::localization::Gnss::STATUS_CONVERGING:
    data_out->gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
    break;
  case msg::localization::Gnss::STATUS_GOOD:
    data_out->gnss_status = ad_msg::Gnss::STATUS_GOOD;
    break;
  default:
    data_out->gnss_status = ad_msg::Gnss::STATUS_INVALID;
    break;
  }

  switch (gnss.utm_status()) {
  case msg::localization::Gnss::STATUS_BAD:
    data_out->utm_status = ad_msg::Gnss::STATUS_BAD;
    break;
  case msg::localization::Gnss::STATUS_CONVERGING:
    data_out->utm_status = ad_msg::Gnss::STATUS_CONVERGING;
    break;
  case msg::localization::Gnss::STATUS_GOOD:
    data_out->utm_status = ad_msg::Gnss::STATUS_GOOD;
    break;
  default:
    data_out->utm_status = ad_msg::Gnss::STATUS_INVALID;
    break;
  }

  switch (gnss.odom_status()) {
  case msg::localization::Gnss::STATUS_BAD:
    data_out->odom_status = ad_msg::Gnss::STATUS_BAD;
    break;
  case msg::localization::Gnss::STATUS_CONVERGING:
    data_out->odom_status = ad_msg::Gnss::STATUS_CONVERGING;
    break;
  case msg::localization::Gnss::STATUS_GOOD:
    data_out->odom_status = ad_msg::Gnss::STATUS_GOOD;
    break;
  default:
    data_out->odom_status = ad_msg::Gnss::STATUS_INVALID;
    break;
  }

  if (com_isnan(data_out->latitude) || com_isinf(data_out->latitude) ||
      com_isnan(data_out->longitude) || com_isinf(data_out->longitude) ||
      com_isnan(data_out->heading_gnss) || com_isinf(data_out->heading_gnss)) {
    data_out->gnss_status = ad_msg::Gnss::STATUS_INVALID;

    data_out->latitude = 0;
    data_out->longitude = 0;
    data_out->heading_gnss = 0;
  }

  if (com_isnan(data_out->x_utm) || com_isinf(data_out->x_utm) ||
      com_isnan(data_out->y_utm) || com_isinf(data_out->y_utm) ||
      com_isnan(data_out->heading_utm) || com_isinf(data_out->heading_utm)) {
    data_out->utm_status = ad_msg::Gnss::STATUS_INVALID;

    data_out->x_utm = 0;
    data_out->y_utm = 0;
    data_out->heading_utm = 0;
  }

  if (com_isnan(data_out->x_odom) || com_isinf(data_out->x_odom) ||
      com_isnan(data_out->y_odom) || com_isinf(data_out->y_odom) ||
      com_isnan(data_out->heading_odom) || com_isinf(data_out->heading_odom)) {
    data_out->odom_status = ad_msg::Gnss::STATUS_INVALID;

    data_out->x_odom = 0;
    data_out->y_odom = 0;
    data_out->heading_odom = 0;
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeImuMessage(
    const ad_msg::Imu& msg,
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
    const Char_t* msg, Int32_t msg_len, ad_msg::Imu* data_out) {
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
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeMpuStateMessage(
    const MpuState::MonitorMpuState& msg,
      MpuState::MonitorMpuState* const data_out){

  return true;
}

bool ParseProtoMsg::DecodeMpuStateMessage(
    const Char_t* msg, Int32_t msg_len,
      MpuState::MonitorMpuState* data_out)
{
  // parse
  if(!data_out->ParseFromArray(msg, msg_len))
  {
    LOG_ERR << "Failed to parse mpu state from array.";
    return false;
  }
  return true;
}


bool ParseProtoMsg::EncodeChassisMessage(
    const ad_msg::Chassis& msg,
    msg::control::Chassis* const data_out) {
  // 驾驶模式
  switch (msg.driving_mode) {
  case (ad_msg::VEH_DRIVING_MODE_MANUAL):
    data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_MANUAL);
    break;
  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC):
    data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_ROBOTIC);
    break;
    break;
  default:
    data_out->set_driving_mode(msg::control::Chassis::DRIVING_MODE_INVALID);
    break;
  }
  // 紧急停止信号
  switch (msg.e_stop) {
  case (ad_msg::VEH_E_STOP_OFF):
    data_out->set_e_stop(msg::control::Chassis::E_STOP_OFF);
    break;
  case (ad_msg::VEH_E_STOP_ON):
    data_out->set_e_stop(msg::control::Chassis::E_STOP_ON);
    break;
  default:
    data_out->set_e_stop(msg::control::Chassis::E_STOP_INVALID);
    break;
  }

  // 方向盘控制状态
  switch (msg.eps_status) {
  case (ad_msg::VEH_EPS_STATUS_MANUAL):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_MANUAL);
    break;
  case (ad_msg::VEH_EPS_STATUS_ROBOTIC):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_ROBOTIC);
    break;
  case (ad_msg::VEH_EPS_STATUS_MANUAL_INTERRUPT):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_MANUAL_INTERRUPT);
    break;
  case (ad_msg::VEH_EPS_STATUS_ERROR):
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_ERROR);
    break;
  default:
    data_out->set_eps_status(msg::control::Chassis::EPS_STATUS_INVALID);
    break;
  }
  // 油门系统控制状态
  switch (msg.throttle_sys_status) {
  case (ad_msg::VEH_THROTTLE_SYS_STATUS_MANUAL):
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_MANUAL);
    break;
  case (ad_msg::VEH_THROTTLE_SYS_STATUS_ROBOTIC):
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_ROBOTIC);
    break;
  case (ad_msg::VEH_THROTTLE_SYS_STATUS_ERROR):
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_ERROR);
    break;
  default:
    data_out->set_throttle_sys_status(msg::control::Chassis::THROTTLE_SYS_STATUS_INVALID);
    break;
  }
  // 制动系统控制状态
  switch (msg.ebs_status) {
  case (ad_msg::VEH_EBS_STATUS_MANUAL):
    data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_MANUAL);
    break;
  case (ad_msg::VEH_EBS_STATUS_ROBOTIC):
    data_out->set_ebs_status(msg::control::Chassis::EBS_STATUS_ROBOTIC);
    break;
  case (ad_msg::VEH_EBS_STATUS_ERROR):
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
  // 制动踏板深度
  data_out->set_brake_pedal_value(msg.brake_pedal_value);
  // 油门踏板深度, PH=[百分比][0,100]
  data_out->set_acc_pedal_value(msg.acc_pedal_value);

  // 档位
  switch (msg.gear) {
  case (ad_msg::VEH_GEAR_P):
    data_out->set_gear(msg::control::Chassis::GEAR_P);
    break;
  case (ad_msg::VEH_GEAR_N):
    data_out->set_gear(msg::control::Chassis::GEAR_N);
    break;
  case (ad_msg::VEH_GEAR_R):
    data_out->set_gear(msg::control::Chassis::GEAR_R);
    break;
  case (ad_msg::VEH_GEAR_D):
    data_out->set_gear(msg::control::Chassis::GEAR_D);
    break;
  default:
    data_out->set_gear(msg::control::Chassis::GEAR_INVALID);
    break;
  }
  // 转向灯信号
  switch (msg.signal_turn_lamp) {
  case (ad_msg::VEH_TURN_LAMP_OFF):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_OFF);
    break;
  case (ad_msg::VEH_TURN_LAMP_RIGHT):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_RIGHT);
    break;
  case (ad_msg::VEH_TURN_LAMP_LEFT):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_LEFT);
    break;
  case (ad_msg::VEH_TURN_LAMP_EMERGENCY):
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_EMERGENCY);
    break;
  default:
    data_out->set_signal_turn_lamp(msg::control::Chassis::TURN_LAMP_INVALID);
    break;
  }

  switch (msg.signal_turning_indicator) {
  case (ad_msg::VEH_TURNING_INDICATOR_NONE):
    data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_NONE);
    break;
  case (ad_msg::VEH_TURNING_INDICATOR_LEFT):
    data_out->set_signal_turning_indicator(
          msg::control::Chassis::TURNING_INDICATOR_LEFT);
    break;
  case (ad_msg::VEH_TURNING_INDICATOR_RIGHT):
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
  case (ad_msg::VEH_LAMP_OFF):
    data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_OFF);
    break;
  case (ad_msg::VEH_LAMP_ON):
    data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_ON);
    break;
  default:
    data_out->set_signal_brake_lamp(msg::control::Chassis::LAMP_INVALID);
    break;
  }
  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (msg.epb_status) {
  case (ad_msg::VEH_EPB_STATUS_OFF):
    data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_OFF);
    break;
  case (ad_msg::VEH_EPB_STATUS_ON):
    data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_ON);
    break;
  default:
    data_out->set_epb_status(msg::control::Chassis::EPB_STATUS_INVALID);
    break;
  }

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

  // 发动机转速有效位
  data_out->set_engine_speed_valid(msg.engine_speed_valid);
  // 发动机转速
  data_out->set_engine_speed(msg.engine_speed);
  // 发动机转矩有效位
  data_out->set_engine_torque_valid(msg.engine_torque_valid);
  // 发动机转矩(N.m)
  data_out->set_engine_torque(msg.engine_torque);

  return true;
}

bool ParseProtoMsg::DecodeChassisMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::Chassis* data_out) {
  // parse
  msg::control::Chassis chassis;
  if (!chassis.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 驾驶模式
  switch (chassis.driving_mode()) {
  case (msg::control::Chassis::DRIVING_MODE_MANUAL):
    data_out->driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;
    break;
  case (msg::control::Chassis::DRIVING_MODE_ROBOTIC):
    data_out->driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
    break;
  default:
    data_out->driving_mode = ad_msg::VEH_DRIVING_MODE_INVALID;
    break;
  }

  // 紧急停止信号
  switch (chassis.e_stop()) {
  case (msg::control::Chassis::E_STOP_OFF):
    data_out->e_stop = ad_msg::VEH_E_STOP_OFF;
    break;
  case (msg::control::Chassis::E_STOP_ON):
    data_out->e_stop = ad_msg::VEH_E_STOP_ON;
    break;
  default:
    data_out->e_stop = ad_msg::VEH_E_STOP_INVALID;
    break;
  }

  // 方向盘控制状态
  switch (chassis.eps_status()) {
  case (msg::control::Chassis::EPS_STATUS_MANUAL):
    data_out->eps_status = ad_msg::VEH_EPS_STATUS_MANUAL;
    break;
  case (msg::control::Chassis::EPS_STATUS_ROBOTIC):
    data_out->eps_status = ad_msg::VEH_EPS_STATUS_ROBOTIC;
    break;
  case (msg::control::Chassis::EPS_STATUS_MANUAL_INTERRUPT):
    data_out->eps_status = ad_msg::VEH_EPS_STATUS_MANUAL_INTERRUPT;
    break;
  case (msg::control::Chassis::EPS_STATUS_ERROR):
    data_out->eps_status = ad_msg::VEH_EPS_STATUS_ERROR;
    break;
  default:
    data_out->eps_status = ad_msg::VEH_EPS_STATUS_INVALID;
  }
  // 油门系统控制状态
  switch (chassis.throttle_sys_status()) {
  case (msg::control::Chassis::THROTTLE_SYS_STATUS_MANUAL):
    data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_MANUAL;
    break;
  case (msg::control::Chassis::THROTTLE_SYS_STATUS_ROBOTIC):
    data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_ROBOTIC;
    break;
  case (msg::control::Chassis::THROTTLE_SYS_STATUS_ERROR):
    data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_ERROR;
    break;
  default:
    data_out->throttle_sys_status = ad_msg::VEH_THROTTLE_SYS_STATUS_INVALID;
  }
  // 制动系统控制状态
  switch (chassis.ebs_status()) {
  case (msg::control::Chassis::EBS_STATUS_MANUAL):
    data_out->ebs_status = ad_msg::VEH_EBS_STATUS_MANUAL;
    break;
  case (msg::control::Chassis::EBS_STATUS_ROBOTIC):
    data_out->ebs_status = ad_msg::VEH_EBS_STATUS_ROBOTIC;
    break;
  case (msg::control::Chassis::EBS_STATUS_ERROR):
    data_out->ebs_status = ad_msg::VEH_EBS_STATUS_ERROR;
    break;
  default:
    data_out->ebs_status = ad_msg::VEH_EBS_STATUS_INVALID;
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
    data_out->epb_status = ad_msg::VEH_EPB_STATUS_OFF;
    break;
  case (msg::control::Chassis::EPB_STATUS_ON):
    data_out->epb_status = ad_msg::VEH_EPB_STATUS_ON;
    break;
  default:
    data_out->epb_status = ad_msg::VEH_EPB_STATUS_INVALID;
    break;
  }
  // 档位
  switch (chassis.gear()) {
  case (msg::control::Chassis::GEAR_P):
    data_out->gear = ad_msg::VEH_GEAR_P;
    break;
  case (msg::control::Chassis::GEAR_N):
    data_out->gear = ad_msg::VEH_GEAR_N;
    break;
  case (msg::control::Chassis::GEAR_R):
    data_out->gear = ad_msg::VEH_GEAR_R;
    break;
  case (msg::control::Chassis::GEAR_D):
    data_out->gear = ad_msg::VEH_GEAR_D;
    break;
  default:
    data_out->gear = ad_msg::VEH_GEAR_INVALID;
    break;
  }
  // 档位 Number
  data_out->gear_number = chassis.gear_number();
  // 转向拨杆信号
  switch (chassis.signal_turning_indicator()) {
  case (msg::control::Chassis::TURNING_INDICATOR_NONE):
    data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_NONE;
    break;
  case (msg::control::Chassis::TURNING_INDICATOR_LEFT):
    data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_LEFT;
    break;
  case (msg::control::Chassis::TURNING_INDICATOR_RIGHT):
    data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_RIGHT;
    break;
  default:
    data_out->signal_turning_indicator = ad_msg::VEH_TURNING_INDICATOR_INVALID;
    break;
  }
  // 转向灯信号
  switch (chassis.signal_turn_lamp()) {
  case (msg::control::Chassis::TURN_LAMP_OFF):
    data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
    break;
  case (msg::control::Chassis::TURN_LAMP_LEFT):
    data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (msg::control::Chassis::TURN_LAMP_RIGHT):
    data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (msg::control::Chassis::TURN_LAMP_EMERGENCY):
    data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    data_out->signal_turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
    break;
  }
  // 制动灯信号
  switch (chassis.signal_brake_lamp()) {
  case (msg::control::Chassis::LAMP_OFF):
    data_out->signal_brake_lamp = ad_msg::VEH_LAMP_OFF;
    break;
  case (msg::control::Chassis::LAMP_ON):
    data_out->signal_brake_lamp = ad_msg::VEH_LAMP_ON;
    break;
  default:
    data_out->signal_brake_lamp = ad_msg::VEH_LAMP_INVALID;
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

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeSpecialChassisInfoMessage(
    const ad_msg::SpecialChassisInfo& msg,
    msg::control::SpecialChassisInfo* const data_out) {
  // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  data_out->set_start_adas(msg.start_adas);

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
    ad_msg::SpecialChassisInfo* data_out) {
  // parse
  msg::control::SpecialChassisInfo special_chassis_info;
  if (!special_chassis_info.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse chassis from array.";
    return false;
  }

  // 开始自动驾驶命令, 0 ~ 无效值, 1 ~ 退出自动驾驶, 2 ~ 进入自动驾驶
  data_out->start_adas = special_chassis_info.start_adas();

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
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeChassisCtlCmdMessage(
    const ad_msg::ChassisCtlCmd& msg,
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
  case (ad_msg::VEH_GEAR_P):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_P);
    break;
  case (ad_msg::VEH_GEAR_N):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_N);
    break;
  case (ad_msg::VEH_GEAR_R):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_R);
    break;
  case (ad_msg::VEH_GEAR_D):
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_D);
    break;
  default:
    data_out->set_gear(msg::control::ChassisCtlCmd::GEAR_INVALID);
    break;
  }
  // 转向灯信号
  switch (msg.turn_lamp) {
  case (ad_msg::VEH_TURN_LAMP_OFF):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_OFF);
    break;
  case (ad_msg::VEH_TURN_LAMP_RIGHT):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_RIGHT);
    break;
  case (ad_msg::VEH_TURN_LAMP_LEFT):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_LEFT);
    break;
  case (ad_msg::VEH_TURN_LAMP_EMERGENCY):
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_EMERGENCY);
    break;
  default:
    data_out->set_turn_lamp(msg::control::ChassisCtlCmd::TURN_LAMP_INVALID);
    break;
  }

  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (msg.brake_lamp) {
  case (ad_msg::VEH_LAMP_OFF):
    data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_OFF);
    break;
  case (ad_msg::VEH_LAMP_ON):
    data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_ON);
    break;
  default:
    data_out->set_brake_lamp(msg::control::ChassisCtlCmd::LAMP_INVALID);
    break;
  }

  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (msg.epb_status) {
  case (ad_msg::VEH_EPB_STATUS_OFF):
    data_out->set_epb_status(msg::control::ChassisCtlCmd::EPB_STATUS_OFF);
    break;
  case (ad_msg::VEH_EPB_STATUS_ON):
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
  case (ad_msg::VEH_WIPER_OFF):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_OFF);
    break;
  case (ad_msg::VEH_WIPER_SHORT_PRESS_WITH_CLICK):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_SHORT_PRESS_WITH_CLICK);
    break;
  case (ad_msg::VEH_WIPER_LONG_PRESS_WITH_CLICK):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_LONG_PRESS_WITH_CLICK);
    break;
  case (ad_msg::VEH_WIPER_INT_1):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_1);
    break;
  case (ad_msg::VEH_WIPER_INT_2):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_2);
    break;
  case (ad_msg::VEH_WIPER_INT_3):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_3);
    break;
  case (ad_msg::VEH_WIPER_INT_4):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INT_4);
    break;
  case (ad_msg::VEH_WIPER_LO):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_LO);
    break;
  case (ad_msg::VEH_WIPER_HI):
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_HI);
    break;
  default:
    data_out->set_wiper(msg::control::ChassisCtlCmd::WIPER_INVALID);
    break;
  }

  return true;
}

bool ParseProtoMsg::DecodeChassisCtlCmdMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::ChassisCtlCmd* data_out) {
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
    data_out->gear = ad_msg::VEH_GEAR_P;
    break;
  case (msg::control::ChassisCtlCmd::GEAR_N):
    data_out->gear = ad_msg::VEH_GEAR_N;
    break;
  case (msg::control::ChassisCtlCmd::GEAR_R):
    data_out->gear = ad_msg::VEH_GEAR_R;
    break;
  case (msg::control::ChassisCtlCmd::GEAR_D):
    data_out->gear = ad_msg::VEH_GEAR_D;
    break;
  default:
    data_out->gear = ad_msg::VEH_GEAR_INVALID;
    break;
  }
  // 转向灯信号
  switch (chassis_ctl_cmd.turn_lamp()) {
  case (msg::control::ChassisCtlCmd::TURN_LAMP_OFF):
    data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
    break;
  case (msg::control::ChassisCtlCmd::TURN_LAMP_LEFT):
    data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (msg::control::ChassisCtlCmd::TURN_LAMP_RIGHT):
    data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (msg::control::ChassisCtlCmd::TURN_LAMP_EMERGENCY):
    data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    data_out->turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
    break;
  }
  // 制动灯信号, 0x0:无效，0x1:OFF, 0x2:开
  switch (chassis_ctl_cmd.brake_lamp()) {
  case (msg::control::ChassisCtlCmd::LAMP_OFF):
    data_out->brake_lamp = ad_msg::VEH_LAMP_OFF;
    break;
  case (msg::control::ChassisCtlCmd::LAMP_ON):
    data_out->brake_lamp = ad_msg::VEH_LAMP_ON;
    break;
  default:
    data_out->brake_lamp = ad_msg::VEH_LAMP_INVALID;
    break;
  }
  // 驻车系统状态, 0x0:无效，0x1:驻车解除, 0x2:驻车中
  switch (chassis_ctl_cmd.epb_status()) {
  case (msg::control::ChassisCtlCmd::EPB_STATUS_OFF):
    data_out->epb_status = ad_msg::VEH_EPB_STATUS_OFF;
    break;
  case (msg::control::ChassisCtlCmd::EPB_STATUS_ON):
    data_out->epb_status = ad_msg::VEH_EPB_STATUS_ON;
    break;
  default:
    data_out->epb_status = ad_msg::VEH_EPB_STATUS_INVALID;
    break;
  }
  // 雨刮器状态, 0x0:无效，0x1:关闭, 0x2:SHORT PRESS WITH CLICK,
  //0x3:LONG PRESS WITH CLICK，0x4:INT 1, 0x5:INT 2，0x6:INT 3,
  //0x7:INT 4，0x8:LO，0x9:HI
  switch (chassis_ctl_cmd.wiper()) {
  case (msg::control::ChassisCtlCmd::WIPER_OFF):
    data_out->wiper = ad_msg::VEH_WIPER_OFF;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_SHORT_PRESS_WITH_CLICK):
    data_out->wiper = ad_msg::VEH_WIPER_SHORT_PRESS_WITH_CLICK;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_LONG_PRESS_WITH_CLICK):
    data_out->wiper = ad_msg::VEH_WIPER_LONG_PRESS_WITH_CLICK;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_1):
    data_out->wiper = ad_msg::VEH_WIPER_INT_1;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_2):
    data_out->wiper = ad_msg::VEH_WIPER_INT_2;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_3):
    data_out->wiper = ad_msg::VEH_WIPER_INT_3;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_INT_4):
    data_out->wiper = ad_msg::VEH_WIPER_INT_4;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_LO):
    data_out->wiper = ad_msg::VEH_WIPER_LO;
    break;
  case (msg::control::ChassisCtlCmd::WIPER_HI):
    data_out->wiper = ad_msg::VEH_WIPER_HI;
    break;
  default:
    data_out->wiper = ad_msg::VEH_WIPER_INVALID;
    break;
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeCanFrameListMessage(
    const can_dev::CanFrameList& msg,
    can_dev::perception::CanFrameList* const data_out) {
  data_out->set_can_frame_num(msg.can_frame_num);
  
  for (int i = 0; i < msg.can_frame_num; i++) {
    can_dev::perception::CanFrame* can_frame = data_out->add_can_frame_list();

    can_frame->set_time_stamp(msg.can_frame[i].time_stamp);
    can_frame->set_id(msg.can_frame[i].id);
    can_frame->set_rtr(msg.can_frame[i].RTR);
    can_frame->set_ext(msg.can_frame[i].EXT);
    can_frame->set_data_len(msg.can_frame[i].data_len);
    uint32_t data_1 = 0, data_2 = 0;
    for (uint8_t j = 0; j < 4; j++) {
      data_1 |= msg.can_frame[i].data[j] << (j * 8);
      data_2 |= msg.can_frame[i].data[j + 4] << (j * 8);
    }
    can_frame->set_data_1(data_1);
    can_frame->set_data_2(data_2);
  }

  return true;
}

bool ParseProtoMsg::DecodeCanFrameListMessage(
    const Char_t* msg, Int32_t msg_len,
    can_dev::CanFrameList* data_out) {
//parse
  can_dev::perception::CanFrameList can_frame_list;
  if(!can_frame_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse can frame list from array.";
    return false;
  }

  // Update each of can frame 
  Int32_t can_frame_num = can_frame_list.can_frame_list_size();
  if (can_frame_num > can_dev::CanFrameList::MAX_CAN_FRAME_NUM) {
    can_frame_num = can_dev::CanFrameList::MAX_CAN_FRAME_NUM;
  }
  for (Int32_t i = 0; i < can_frame_num; ++i) {
    const can_dev::perception::CanFrame& can_frame =
        can_frame_list.can_frame_list(i);
    can_dev::CanFrame& dst = data_out->can_frame[i];

    dst.time_stamp = can_frame.time_stamp();
    dst.id = can_frame.id();
    dst.RTR = can_frame.rtr();
    dst.EXT = can_frame.ext();
    dst.data_len = can_frame.data_len();
    uint32_t data_1 = 0, data_2 = 0;
    data_1 = can_frame.data_1();
    data_2 = can_frame.data_2();

    for (Uint8_t j = 0; j < 4; j++) {
      dst.data[j] = (data_1 >> (j * 8)) & 0xFF;
      dst.data[j + 4] = (data_2 >> (j * 8)) & 0xFF;
    }
  }

  data_out->can_frame_num = can_frame_num;

  return true;
}

bool ParseProtoMsg::EncodeCanFdFrameListMessage(
    const can_dev::CanFdFrameList& msg,
    can_dev::perception::CanFdFrameList* const data_out) {
  
  for (int i = 0; i < msg.canfd_frame_num; i++) {
    can_dev::perception::CanFdFrame* canfd_frame = data_out->add_canfd_frame_list();
    canfd_frame->set_time_stamp(msg.canfd_frame[i].time_stamp);
    canfd_frame->set_id(msg.canfd_frame[i].id);
    canfd_frame->set_fd(msg.canfd_frame[i].FD);
    canfd_frame->set_rtr(msg.canfd_frame[i].RTR);
    canfd_frame->set_ext(msg.canfd_frame[i].EXT);
    canfd_frame->set_brs(msg.canfd_frame[i].BRS);
    canfd_frame->set_esi(msg.canfd_frame[i].ESI);
    canfd_frame->set_data_len(msg.canfd_frame[i].data_len);
    uint32_t data_1 = 0, data_2 = 0, data_3 = 0, data_4 = 0, \
             data_5 = 0, data_6 = 0, data_7 = 0, data_8 = 0, \
             data_9 = 0, data_10 = 0, data_11 = 0, data_12 = 0, \
             data_13 = 0, data_14 = 0, data_15 = 0, data_16 = 0;
    for (uint8_t j = 0; j < 4; j++) {
      data_1 |= msg.canfd_frame[i].data[j] << (j * 8);
      data_2 |= msg.canfd_frame[i].data[j + 4] << (j * 8);
      data_3 |= msg.canfd_frame[i].data[j + 8] << (j * 8);
      data_4 |= msg.canfd_frame[i].data[j + 12] << (j * 8);
      data_5 |= msg.canfd_frame[i].data[j + 16] << (j * 8);
      data_6 |= msg.canfd_frame[i].data[j + 20] << (j * 8);
      data_7 |= msg.canfd_frame[i].data[j + 24] << (j * 8);
      data_8 |= msg.canfd_frame[i].data[j + 28] << (j * 8);
      data_9 |= msg.canfd_frame[i].data[j + 32] << (j * 8);
      data_10 |= msg.canfd_frame[i].data[j + 36] << (j * 8);
      data_11 |= msg.canfd_frame[i].data[j + 40] << (j * 8);
      data_12 |= msg.canfd_frame[i].data[j + 44] << (j * 8);
      data_13 |= msg.canfd_frame[i].data[j + 48] << (j * 8);
      data_14 |= msg.canfd_frame[i].data[j + 52] << (j * 8);
      data_15 |= msg.canfd_frame[i].data[j + 56] << (j * 8);
      data_16 |= msg.canfd_frame[i].data[j + 60] << (j * 8);
    }
    canfd_frame->set_data_1(data_1);
    canfd_frame->set_data_2(data_2);
    canfd_frame->set_data_3(data_3);
    canfd_frame->set_data_4(data_4);
    canfd_frame->set_data_5(data_5);
    canfd_frame->set_data_6(data_6);
    canfd_frame->set_data_7(data_7);
    canfd_frame->set_data_8(data_8);
    canfd_frame->set_data_9(data_9);
    canfd_frame->set_data_10(data_10);
    canfd_frame->set_data_11(data_11);
    canfd_frame->set_data_12(data_12);
    canfd_frame->set_data_13(data_13);
    canfd_frame->set_data_14(data_14);
    canfd_frame->set_data_15(data_15);
    canfd_frame->set_data_16(data_16);
  }

  data_out->set_canfd_frame_num(msg.canfd_frame_num);
  return true;
}

bool ParseProtoMsg::DecodeCanFdFrameListMessage(
    const Char_t* msg, Int32_t msg_len,
    can_dev::CanFdFrameList* data_out) {
  //parse
  can_dev::perception::CanFdFrameList canfd_frame_list;
  if(!canfd_frame_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse canfd frame list from array.";
    return false;
  }

  // Update each of canfd frame 
  Int32_t canfd_frame_num = canfd_frame_list.canfd_frame_list_size();
  if (canfd_frame_num > can_dev::CanFdFrameList::MAX_CANFD_FRAME_NUM) {
    canfd_frame_num = can_dev::CanFdFrameList::MAX_CANFD_FRAME_NUM;
  }
  for (Int32_t i = 0; i < canfd_frame_num; ++i) {
    const can_dev::perception::CanFdFrame& canfd_frame =
        canfd_frame_list.canfd_frame_list(i);
    can_dev::CanFdFrame& dst = data_out->canfd_frame[i];

    dst.time_stamp = canfd_frame.time_stamp();
    dst.id = canfd_frame.id();
    dst.FD = canfd_frame.fd();
    dst.RTR = canfd_frame.rtr();
    dst.EXT = canfd_frame.ext();
    dst.BRS = canfd_frame.brs();
    dst.ESI = canfd_frame.esi();
    dst.data_len = canfd_frame.data_len();
    uint32_t data_1 = 0, data_2 = 0, data_3 = 0, data_4 = 0, \
             data_5 = 0, data_6 = 0, data_7 = 0, data_8 = 0, \
             data_9 = 0, data_10 = 0, data_11 = 0, data_12 = 0, \
             data_13 = 0, data_14 = 0, data_15 = 0, data_16 = 0;
    data_1 = canfd_frame.data_1();
    data_2 = canfd_frame.data_2();
    data_3 = canfd_frame.data_3();
    data_4 = canfd_frame.data_4();
    data_5 = canfd_frame.data_5();
    data_6 = canfd_frame.data_6();
    data_7 = canfd_frame.data_7();
    data_8 = canfd_frame.data_8();
    data_9 = canfd_frame.data_9();
    data_10 = canfd_frame.data_10();
    data_11 = canfd_frame.data_11();
    data_12 = canfd_frame.data_12();
    data_13 = canfd_frame.data_13();
    data_14 = canfd_frame.data_14();
    data_15 = canfd_frame.data_15();
    data_16 = canfd_frame.data_16();

    for (Uint8_t j = 0; j < 4; j++) {
      dst.data[j] = (data_1 >> (j * 8)) & 0xFF;
      dst.data[j + 4] = (data_2 >> (j * 8)) & 0xFF;
      dst.data[j + 8] = (data_3 >> (j * 8)) & 0xFF;
      dst.data[j + 12] = (data_4 >> (j * 8)) & 0xFF;
      dst.data[j + 16] = (data_5 >> (j * 8)) & 0xFF;
      dst.data[j + 20] = (data_6 >> (j * 8)) & 0xFF;
      dst.data[j + 24] = (data_7 >> (j * 8)) & 0xFF;
      dst.data[j + 28] = (data_8 >> (j * 8)) & 0xFF;
      dst.data[j + 32] = (data_9 >> (j * 8)) & 0xFF;
      dst.data[j + 36] = (data_10 >> (j * 8)) & 0xFF;
      dst.data[j + 40] = (data_11 >> (j * 8)) & 0xFF;
      dst.data[j + 44] = (data_12 >> (j * 8)) & 0xFF;
      dst.data[j + 48] = (data_13 >> (j * 8)) & 0xFF;
      dst.data[j + 52] = (data_14 >> (j * 8)) & 0xFF;
      dst.data[j + 56] = (data_15 >> (j * 8)) & 0xFF;
      dst.data[j + 60] = (data_16 >> (j * 8)) & 0xFF;
    }
  }

  data_out->canfd_frame_num = canfd_frame_num;

  return true;
}

bool ParseProtoMsg::EncodeLaneMarkCameraListMessage(
    const ad_msg::LaneMarkCameraList& msg,
    msg::perception::LaneMarkCameraList* const data_out) {
  for (int i = 0; i < msg.lane_mark_num; i++) {
    msg::perception::LaneMarkCamera* lane_mark_camera =
        data_out->add_lane_mark_list();

    lane_mark_camera->set_id(msg.lane_marks[i].id);
    switch (msg.lane_marks[i].lane_mark_type) {
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_INVALID);
      break;
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_UNKNOWN);
      break;
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_DASHED);
      break;
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_SOLID);
      break;
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_DOUBLE_LANE_MARK);
      break;
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_BOTTS_DOTS);
      break;
    case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE:
      lane_mark_camera->set_lane_mark_type(
            msg::perception::LaneMarkCamera_LaneMarkType_LANE_MARK_TYPE_ROAD_EDGE);
      break;
    default:
      break;
    }
    lane_mark_camera->set_quality(msg.lane_marks[i].quality);
    lane_mark_camera->set_mark_width(msg.lane_marks[i].mark_width);
    lane_mark_camera->set_view_range_end(msg.lane_marks[i].view_range_end);
    lane_mark_camera->set_view_range_start(msg.lane_marks[i].view_range_start);
    lane_mark_camera->set_view_range_valid(msg.lane_marks[i].view_range_valid);

    lane_mark_camera->set_c0(msg.lane_marks[i].c0);
    lane_mark_camera->set_c1(msg.lane_marks[i].c1);
    lane_mark_camera->set_c2(msg.lane_marks[i].c2);
    lane_mark_camera->set_c3(msg.lane_marks[i].c3);
  }

  return true;
}

bool ParseProtoMsg::DecodeLaneMarkCameraListMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::LaneMarkCameraList* data_out) {
  // parse
  msg::perception::LaneMarkCameraList lane_mark_camera_list;
  if (!lane_mark_camera_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse camera lane mark list from array.";
    return false;
  }

  // Update each of lane mark information
  Int32_t lane_mark_num = lane_mark_camera_list.lane_mark_list_size();
  if (lane_mark_num > ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM) {
    lane_mark_num = ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM;
  }
  for (Int32_t i = 0; i < lane_mark_num; ++i) {
    const msg::perception::LaneMarkCamera& lane_mark =
        lane_mark_camera_list.lane_mark_list(i);
    ad_msg::LaneMarkCamera& dst = data_out->lane_marks[i];

    dst.id = lane_mark.id();
    switch (lane_mark.lane_mark_type()) {
    case msg::perception::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN:
      dst.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
      break;
    case msg::perception::LaneMarkCamera::LANE_MARK_TYPE_DASHED:
      dst.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
      break;
    case msg::perception::LaneMarkCamera::LANE_MARK_TYPE_SOLID:
      dst.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
      break;
    case msg::perception::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK:
      dst.lane_mark_type =
          ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK;
      break;
    case msg::perception::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS:
      dst.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS;
      break;
    case msg::perception::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE:
      dst.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
      break;
    default:
      dst.lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
      break;
    }
    dst.quality = lane_mark.quality();
    dst.view_range_valid = lane_mark.view_range_valid();
    dst.mark_width = lane_mark.mark_width();
    dst.view_range_start = lane_mark.view_range_start();
    dst.view_range_end = lane_mark.view_range_end();
    dst.c0 = lane_mark.c0();
    dst.c1 = lane_mark.c1();
    dst.c2 = lane_mark.c2();
    dst.c3 = lane_mark.c3();
  }
  // Update number of lanes
  data_out->lane_mark_num = lane_mark_num;

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeObstacleCameraListMessage(
    const ad_msg::ObstacleCameraList& msg,
    msg::perception::ObstacleCameraList* const data_out) {
  switch (msg.cam_type) {
  case (ad_msg::ObstacleCameraList::CAM_TYPE_MOBILEYE_Q2):
    data_out->set_cam_type(msg::perception::ObstacleCameraList::CAM_TYPE_MOBILEYE_Q2);
    break;
  case (ad_msg::ObstacleCameraList::CAM_TYPE_MAXIEYE_D500):
    data_out->set_cam_type(msg::perception::ObstacleCameraList::CAM_TYPE_MAXIEYE_D500);
    break;
  default:
    data_out->set_cam_type(msg::perception::ObstacleCameraList::CAM_TYPE_UNKNOWN);
    break;
  }

  data_out->set_obstacles_num(msg.obstacle_num);

  for (int i = 0; i < msg.obstacle_num; i++) {
    msg::perception::ObstacleCamera* obstacle = data_out->add_obstacles();
    obstacle->set_id(msg.obstacles[i].id);
    switch (msg.obstacles[i].type) {
    case ad_msg::OBJ_TYPE_UNKNOWN:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_UNKNOWN);
      break;
    case ad_msg::OBJ_TYPE_PASSENGER_VEHICLE:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_PASSENGER_VEHICLE);
      break;
    case ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_COMMERCIAL_VEHICLE);
      break;
    case ad_msg::OBJ_TYPE_SPECIAL_VEHICLE:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_SPECIAL_VEHICLE);
      break;
    case ad_msg::OBJ_TYPE_OTHER_VEHICLE:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_OTHER_VEHICLE);
      break;
    case ad_msg::OBJ_TYPE_PEDESTRIAN:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_PEDESTRIAN);
      break;
    case ad_msg::OBJ_TYPE_BICYCLE:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_BICYCLE);
      break;
    case ad_msg::OBJ_TYPE_ANIMAL:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_ANIMAL);
      break;
    case ad_msg::OBJ_TYPE_DISCARD:
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_DISCARD);
      break;
    case (ad_msg::OBJ_TYPE_CURB):
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_CURB);
      break;
    case (ad_msg::OBJ_TYPE_TRAFFIC_CONE):
      obstacle->set_type(msg::perception::ObstacleCamera::OBJ_TYPE_TRAFFIC_CONE);
      break;
    default:
      break;
    }
    switch (msg.obstacles[i].status) {
    case ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN:
      obstacle->set_status(msg::perception::ObstacleCamera_ObjStatus_OBJ_STATUS_UNKNOWN);
      break;
    case ad_msg::ObstacleCamera::OBJ_STATUS_STANDING:
      obstacle->set_status(msg::perception::ObstacleCamera_ObjStatus_OBJ_STATUS_STANDING);
      break;
    case ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED:
      obstacle->set_status(msg::perception::ObstacleCamera_ObjStatus_OBJ_STATUS_STOPPED);
      break;
    case ad_msg::ObstacleCamera::OBJ_STATUS_MOVING:
      obstacle->set_status(msg::perception::ObstacleCamera_ObjStatus_OBJ_STATUS_MOVING);
      break;
    case ad_msg::ObstacleCamera::OBJ_STATUS_ONCOMING:
      obstacle->set_status(msg::perception::ObstacleCamera_ObjStatus_OBJ_STATUS_ONCOMING);
      break;
    case ad_msg::ObstacleCamera::OBJ_STATUS_PARKED:
      obstacle->set_status(msg::perception::ObstacleCamera_ObjStatus_OBJ_STATUS_PARKED);
      break;
    default:
      break;
    }
    switch (msg.obstacles[i].cut_in) {
    case ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN:
      obstacle->set_cut_in(msg::perception::ObstacleCamera_CutInType_CUT_IN_TYPE_UNKNOWN);
      break;
    case ad_msg::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE:
      obstacle->set_cut_in(msg::perception::ObstacleCamera_CutInType_CUT_IN_TYPE_IN_HOST_LANE);
      break;
    case ad_msg::ObstacleCamera::CUT_IN_TYPE_OUT_HOST_LANE:
      obstacle->set_cut_in(msg::perception::ObstacleCamera_CutInType_CUT_IN_TYPE_OUT_HOST_LANE);
      break;
    case ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_IN:
      obstacle->set_cut_in(msg::perception::ObstacleCamera_CutInType_CUT_IN_TYPE_CUT_IN);
      break;
    case ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_OUT:
      obstacle->set_cut_in(msg::perception::ObstacleCamera_CutInType_CUT_IN_TYPE_CUT_OUT);
      break;
    default:
      break;
    }
    switch (msg.obstacles[i].blinker) {
    case ad_msg::ObstacleCamera::BLINKER_UNKNOWN:
      obstacle->set_blinker(msg::perception::ObstacleCamera_BlinkerType_BLINKER_UNKNOWN);
      break;
    case ad_msg::ObstacleCamera::BLINKER_OFF:
      obstacle->set_blinker(msg::perception::ObstacleCamera_BlinkerType_BLINKER_OFF);
      break;
    case ad_msg::ObstacleCamera::BLINKER_LEFT:
      obstacle->set_blinker(msg::perception::ObstacleCamera_BlinkerType_BLINKER_LEFT);
      break;
    case ad_msg::ObstacleCamera::BLINKER_RIGHT:
      obstacle->set_blinker(msg::perception::ObstacleCamera_BlinkerType_BLINKER_RIGHT);
      break;
    case ad_msg::ObstacleCamera::BLINKER_BOTH:
      obstacle->set_blinker(msg::perception::ObstacleCamera_BlinkerType_BLINKER_BOTH);
      break;
    default:
      break;
    }
    obstacle->set_brake_lights(msg.obstacles[i].brake_lights);
    obstacle->set_age(msg.obstacles[i].age);
    obstacle->set_lane(msg.obstacles[i].lane);
    obstacle->set_length(msg.obstacles[i].length);
    obstacle->set_width(msg.obstacles[i].width);
    obstacle->set_height(msg.obstacles[i].height);
    obstacle->set_x(msg.obstacles[i].x);
    obstacle->set_y(msg.obstacles[i].y);
    obstacle->set_heading(msg.obstacles[i].heading);
    obstacle->set_v_x(msg.obstacles[i].v_x);
    obstacle->set_v_y(msg.obstacles[i].v_y);
    obstacle->set_accel_x(msg.obstacles[i].accel_x);
    obstacle->set_accel_y(msg.obstacles[i].accel_y);
    obstacle->set_yaw_rate(msg.obstacles[i].yaw_rate);
    obstacle->set_scale_change(msg.obstacles[i].scale_change);
  }

  return true;
}

bool ParseProtoMsg::DecodeObstacleCameraListMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::ObstacleCameraList* data_out) {
  // parse
  msg::perception::ObstacleCameraList obstacle_camera_list;
  if (!obstacle_camera_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse camera obstacle list from array.";
    return false;
  }

  switch (obstacle_camera_list.cam_type()) {
  case (msg::perception::ObstacleCameraList::CAM_TYPE_MOBILEYE_Q2):
    data_out->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_MOBILEYE_Q2;
    break;
  case (msg::perception::ObstacleCameraList::CAM_TYPE_MAXIEYE_D500):
    data_out->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_MAXIEYE_D500;
    break;
  default:
    data_out->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_UNKNOWN;
    break;
  }

  // Update each of lane mark information
  Int32_t obstacle_num = obstacle_camera_list.obstacles_size();
  if (obstacle_num > ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM) {
    obstacle_num = ad_msg::ObstacleCameraList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const msg::perception::ObstacleCamera& obj =
        obstacle_camera_list.obstacles(i);
    ad_msg::ObstacleCamera& dst = data_out->obstacles[i];

    dst.id = obj.id();

    switch (obj.type()) {
    case msg::perception::ObstacleCamera::OBJ_TYPE_UNKNOWN:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      dst.length = 1.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_PASSENGER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      dst.length = 3.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_COMMERCIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      dst.length = 3.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_SPECIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      dst.length = 3.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_OTHER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      dst.length = 3.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_PEDESTRIAN:
      dst.type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      dst.length = 1.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_BICYCLE:
      dst.type = ad_msg::OBJ_TYPE_BICYCLE;
      dst.length = 2.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_ANIMAL:
      dst.type = ad_msg::OBJ_TYPE_ANIMAL;
      dst.length = 1.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_DISCARD:
      dst.type = ad_msg::OBJ_TYPE_DISCARD;
      dst.length = 1.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_CURB:
      dst.type = ad_msg::OBJ_TYPE_CURB;
      dst.length = 1.0F;
      break;
    case msg::perception::ObstacleCamera::OBJ_TYPE_TRAFFIC_CONE:
      dst.type = ad_msg::OBJ_TYPE_TRAFFIC_CONE;
      break;
    default:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      dst.length = 1.0F;
      break;
    }

    switch (obj.status()) {
    case msg::perception::ObstacleCamera::OBJ_STATUS_UNKNOWN:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
    case msg::perception::ObstacleCamera::OBJ_STATUS_STANDING:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_STANDING;
      break;
    case msg::perception::ObstacleCamera::OBJ_STATUS_STOPPED:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
      break;
    case msg::perception::ObstacleCamera::OBJ_STATUS_MOVING:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_MOVING;
      break;
    case msg::perception::ObstacleCamera::OBJ_STATUS_ONCOMING:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_ONCOMING;
      break;
    case msg::perception::ObstacleCamera::OBJ_STATUS_PARKED:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_PARKED;
      break;
    default:
      dst.status = ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
    }

    switch (obj.cut_in()) {
    case msg::perception::ObstacleCamera::CUT_IN_TYPE_UNKNOWN:
      dst.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
      break;
    case msg::perception::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE:
      dst.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE;
      break;
    case msg::perception::ObstacleCamera::CUT_IN_TYPE_OUT_HOST_LANE:
      dst.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_OUT_HOST_LANE;
      break;
    case msg::perception::ObstacleCamera::CUT_IN_TYPE_CUT_IN:
      dst.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_IN;
      break;
    case msg::perception::ObstacleCamera::CUT_IN_TYPE_CUT_OUT:
      dst.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_OUT;
      break;
    default:
      dst.cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
      break;
    }

    switch (obj.blinker()) {
    case msg::perception::ObstacleCamera::BLINKER_UNKNOWN:
      dst.blinker = ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
      break;
    case msg::perception::ObstacleCamera::BLINKER_OFF:
      dst.blinker = ad_msg::ObstacleCamera::BLINKER_OFF;
      break;
    case msg::perception::ObstacleCamera::BLINKER_LEFT:
      dst.blinker = ad_msg::ObstacleCamera::BLINKER_LEFT;
      break;
    case msg::perception::ObstacleCamera::BLINKER_RIGHT:
      dst.blinker = ad_msg::ObstacleCamera::BLINKER_RIGHT;
      break;
    case msg::perception::ObstacleCamera::BLINKER_BOTH:
      dst.blinker = ad_msg::ObstacleCamera::BLINKER_BOTH;
      break;
    default:
      dst.blinker = ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
      break;
    }

    dst.brake_lights = obj.brake_lights();
    dst.age = obj.age();
    dst.lane = obj.lane();
    // dst.length = obj.length();
    dst.width = obj.width();
    dst.height = obj.height();
    dst.x = obj.x();
    dst.y = obj.y();
    dst.heading = obj.heading();
    dst.v_x = obj.v_x();
    dst.v_y = obj.v_y();
    dst.accel_x = obj.accel_x();
    dst.accel_y = obj.accel_y();
    dst.yaw_rate = obj.yaw_rate();
    dst.scale_change = obj.scale_change();
  }

  // Update number of obstacles
  data_out->obstacle_num = obstacle_num;

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeObstacleRadarListMessage(
    const ad_msg::ObstacleRadarList& msg,
    msg::perception::ObstacleRadarList* const data_out) {
  switch (msg.radar_type) {
  case (ad_msg::ObstacleRadarList::RADAR_TYPE_ESR):
    data_out->set_radar_type(msg::perception::ObstacleRadarList::RADAR_TYPE_ESR);
    break;
  case (ad_msg::ObstacleRadarList::RADAR_TYPE_SRR2):
    data_out->set_radar_type(msg::perception::ObstacleRadarList::RADAR_TYPE_SRR2);
    break;
  case (ad_msg::ObstacleRadarList::RADAR_TYPE_RR51W):
    data_out->set_radar_type(msg::perception::ObstacleRadarList::RADAR_TYPE_RR51W);
    break;
  case (ad_msg::ObstacleRadarList::RADAR_TYPE_ARS430):
    data_out->set_radar_type(msg::perception::ObstacleRadarList::RADAR_TYPE_ARS430);
    break;
  default:
    data_out->set_radar_type(msg::perception::ObstacleRadarList::RADAR_TYPE_UNKNOWN);
    break;
  }

  data_out->set_obstacles_num(msg.obstacle_num);

  for (Int32_t i = 0; i < msg.obstacle_num; ++i) {
    msg::perception::ObstacleRadar* obj = data_out->add_obstacles();
    if (Nullptr_t != obj) {
      switch (msg.obstacles[i].type) {
      case (ad_msg::OBJ_TYPE_UNKNOWN):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_UNKNOWN);
        break;
      case (ad_msg::OBJ_TYPE_PASSENGER_VEHICLE):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_PASSENGER_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_COMMERCIAL_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_SPECIAL_VEHICLE):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_SPECIAL_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_OTHER_VEHICLE):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_OTHER_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_PEDESTRIAN):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_PEDESTRIAN);
        break;
      case (ad_msg::OBJ_TYPE_BICYCLE):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_BICYCLE);
        break;
      case (ad_msg::OBJ_TYPE_ANIMAL):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_ANIMAL);
        break;
      case (ad_msg::OBJ_TYPE_DISCARD):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_DISCARD);
        break;
      case (ad_msg::OBJ_TYPE_CURB):
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_CURB);
        break;
      default:
        obj->set_type(msg::perception::ObstacleRadar::OBJ_TYPE_UNKNOWN);
        break;
      }

      switch (msg.obstacles[i].track_status) {
      case (ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_NO_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_NEW_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_NEW_UPDATED_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_NEW_UPDATED_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_UPDATED_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_UPDATED_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_COASTED_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_COASTED_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_MERGED_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_MERGED_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_INVALID_COASTED_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_INVALID_COASTED_TARGET);
        break;
      case (ad_msg::ObstacleRadar::TRACK_STATUS_NEW_COASTED_TARGET):
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_NEW_COASTED_TARGET);
        break;
      default:
        obj->set_track_status(msg::perception::ObstacleRadar::TRACK_STATUS_NO_TARGET);
        break;
      }

      switch (msg.obstacles[i].merged_status) {
      case (ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET):
        obj->set_merged_status(msg::perception::ObstacleRadar::MERGED_STATUS_NO_TARGET);
        break;
      case (ad_msg::ObstacleRadar::MERGED_STATUS_MR_TARGET):
        obj->set_merged_status(msg::perception::ObstacleRadar::MERGED_STATUS_MR_TARGET);
        break;
      case (ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET):
        obj->set_merged_status(msg::perception::ObstacleRadar::MERGED_STATUS_LR_TARGET);
        break;
      case (ad_msg::ObstacleRadar::MERGED_STATUS_MR_LR_TARGET):
        obj->set_merged_status(msg::perception::ObstacleRadar::MERGED_STATUS_MR_LR_TARGET);
        break;
      default:
        obj->set_merged_status(msg::perception::ObstacleRadar::MERGED_STATUS_NO_TARGET);
        break;
      }
      obj->set_oncomming(msg.obstacles[i].oncomming);
      obj->set_bridge(msg.obstacles[i].bridge);
      obj->set_range(msg.obstacles[i].range);
      obj->set_angle(msg.obstacles[i].angle);
      obj->set_range_rate(msg.obstacles[i].range_rate);
      obj->set_range_acceleration(msg.obstacles[i].range_acceleration);
      obj->set_lateral_rate(msg.obstacles[i].lateral_rate);
      obj->set_width(msg.obstacles[i].width);
      obj->set_x(msg.obstacles[i].x);
      obj->set_y(msg.obstacles[i].y);
      obj->set_v_x(msg.obstacles[i].v_x);
      obj->set_v_y(msg.obstacles[i].v_y);
      obj->set_accel_x(msg.obstacles[i].accel_x);
      obj->set_accel_y(msg.obstacles[i].accel_y);
      obj->set_yaw_rate(msg.obstacles[i].yaw_rate);
      obj->set_id(msg.obstacles[i].id);
    }
  }

  return true;
}

bool ParseProtoMsg::DecodeObstacleRadarListMessage(
    const Char_t* msg, Int32_t msg_len, ad_msg::ObstacleRadarList* data_out) {
  // parse
  msg::perception::ObstacleRadarList obstacle_radar_list;
  if (!obstacle_radar_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse radar obstacle list from array.";
    return false;
  }

  switch (obstacle_radar_list.radar_type()) {
  case (msg::perception::ObstacleRadarList::RADAR_TYPE_ESR):
    data_out->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ESR;
    break;
  case (msg::perception::ObstacleRadarList::RADAR_TYPE_SRR2):
    data_out->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_SRR2;
    break;
  case (msg::perception::ObstacleRadarList::RADAR_TYPE_RR51W):
    data_out->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_RR51W;
    break;
  case (msg::perception::ObstacleRadarList::RADAR_TYPE_ARS430):
    data_out->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_ARS430;
    break;
  default:
    data_out->radar_type = ad_msg::ObstacleRadarList::RADAR_TYPE_UNKNOWN;
    break;
  }

  // Update each of lane mark information
  Int32_t obstacle_num = obstacle_radar_list.obstacles_size();
  if (obstacle_num > ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM) {
    obstacle_num = ad_msg::ObstacleRadarList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const msg::perception::ObstacleRadar& obj =
        obstacle_radar_list.obstacles(i);
    ad_msg::ObstacleRadar& dst = data_out->obstacles[i];

    dst.id = obj.id();

    switch (obj.type()) {
    case msg::perception::ObstacleRadar::OBJ_TYPE_UNKNOWN:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_PASSENGER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_COMMERCIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_SPECIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_OTHER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_PEDESTRIAN:
      dst.type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_BICYCLE:
      dst.type = ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_ANIMAL:
      dst.type = ad_msg::OBJ_TYPE_ANIMAL;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_DISCARD:
      dst.type = ad_msg::OBJ_TYPE_DISCARD;
      break;
    case msg::perception::ObstacleRadar::OBJ_TYPE_CURB:
      dst.type = ad_msg::OBJ_TYPE_CURB;
      break;
    default:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    }

    switch (obj.track_status()) {
    case msg::perception::ObstacleRadar::TRACK_STATUS_NO_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_NEW_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_NEW_UPDATED_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_UPDATED_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_UPDATED_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_UPDATED_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_COASTED_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_COASTED_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_MERGED_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_MERGED_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_INVALID_COASTED_TARGET:
      dst.track_status =
          ad_msg::ObstacleRadar::TRACK_STATUS_INVALID_COASTED_TARGET;
      break;
    case msg::perception::ObstacleRadar::TRACK_STATUS_NEW_COASTED_TARGET:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_COASTED_TARGET;
      break;
    default:
      dst.track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET;
      break;
    }

    switch (obj.merged_status()) {
    case msg::perception::ObstacleRadar::MERGED_STATUS_NO_TARGET:
      dst.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET;
      break;
    case msg::perception::ObstacleRadar::MERGED_STATUS_MR_TARGET:
      dst.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_MR_TARGET;
      break;
    case msg::perception::ObstacleRadar::MERGED_STATUS_LR_TARGET:
      dst.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET;
      break;
    case msg::perception::ObstacleRadar::MERGED_STATUS_MR_LR_TARGET:
      dst.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_MR_LR_TARGET;
      break;
    default:
      dst.merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET;
      break;
    }

    dst.oncomming = obj.oncomming();
    dst.bridge = obj.bridge();
    dst.range = obj.range();
    dst.angle = obj.angle();
    dst.range_rate = obj.range_rate();
    dst.range_acceleration = obj.range_acceleration();
    dst.lateral_rate = obj.lateral_rate();
    dst.width = obj.width();
    if (dst.width < 0.1F) {
      dst.width = 0.1F;
    }
    dst.length = dst.width;

    dst.x = obj.x();
    dst.y = obj.y();
    dst.v_x = obj.v_x();
    dst.v_y = obj.v_y();
    dst.accel_x = obj.accel_x();
    dst.accel_y = obj.accel_y();
    dst.yaw_rate = obj.yaw_rate();
  }

  // Update number of obstacles
  data_out->obstacle_num = obstacle_num;

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeObstacleLidarListMessage(
    const ad_msg::ObstacleLidarList& msg,
    msg::perception::ObstacleLidarList* const data_out) {
  switch (msg.lidar_type) {
  case (ad_msg::ObstacleLidarList::LIDAR_TYPE_IBEO_4):
    data_out->set_lidar_type(msg::perception::ObstacleLidarList::LIDAR_TYPE_IBEO_4);
    break;
  case (ad_msg::ObstacleLidarList::LIDAR_TYPE_VLP_16):
    data_out->set_lidar_type(msg::perception::ObstacleLidarList::LIDAR_TYPE_VLP_16);
    break;
  default:
    data_out->set_lidar_type(msg::perception::ObstacleLidarList::LIDAR_TYPE_UNKNOWN);
    break;
  }

  for (Int32_t i = 0; i < msg.obstacle_num; ++i) {
    const ad_msg::ObstacleLidar& src = msg.obstacles[i];
    msg::perception::ObstacleLidar* dst = data_out->add_obstacles();
    if (Nullptr_t != dst) {
      dst->set_id(src.id);

      switch (src.type) {
      case (ad_msg::OBJ_TYPE_UNKNOWN):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_UNKNOWN);
        break;
      case (ad_msg::OBJ_TYPE_PASSENGER_VEHICLE):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_PASSENGER_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_COMMERCIAL_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_SPECIAL_VEHICLE):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_SPECIAL_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_OTHER_VEHICLE):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_OTHER_VEHICLE);
        break;
      case (ad_msg::OBJ_TYPE_PEDESTRIAN):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_PEDESTRIAN);
        break;
      case (ad_msg::OBJ_TYPE_BICYCLE):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_BICYCLE);
        break;
      case (ad_msg::OBJ_TYPE_ANIMAL):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_ANIMAL);
        break;
      case (ad_msg::OBJ_TYPE_DISCARD):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_DISCARD);
        break;
      case (ad_msg::OBJ_TYPE_CURB):
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_CURB);
        break;
      default:
        dst->set_type(msg::perception::ObstacleLidar::OBJ_TYPE_UNKNOWN);
        break;
      }

      dst->set_age(src.age);
      dst->set_prediction_age(src.prediction_age);
      dst->set_x(src.x);
      dst->set_y(src.y);
      msg::perception::ObstacleLidar::OBBox* obb = dst->mutable_obb();
      if (Nullptr_t != obb) {
        obb->set_x(src.obb.x);
        obb->set_y(src.obb.y);
        obb->set_heading(src.obb.heading);
        obb->set_half_length(src.obb.half_length);
        obb->set_half_width(src.obb.half_width);
      } else {
        LOG_ERR << "Failed to add obb.";
      }
      dst->set_v_x(src.v_x);
      dst->set_v_y(src.v_y);
      dst->set_accel_x(src.accel_x);
      dst->set_accel_y(src.accel_y);
      dst->set_yaw_rate(src.yaw_rate);
    }
  }

  return true;
}

bool ParseProtoMsg::DecodeObstacleLidarListMessage(
    const Char_t* msg, Int32_t msg_len,
    ad_msg::ObstacleLidarList* data_out) {
  // parse
  msg::perception::ObstacleLidarList obstacle_lidar_list;
  if (!obstacle_lidar_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse lidar obstacle list from array.";
    return false;
  }

  switch (obstacle_lidar_list.lidar_type()) {
  case (msg::perception::ObstacleLidarList::LIDAR_TYPE_IBEO_4):
    data_out->lidar_type = ad_msg::ObstacleLidarList::LIDAR_TYPE_IBEO_4;
    break;
  case (msg::perception::ObstacleLidarList::LIDAR_TYPE_VLP_16):
    data_out->lidar_type = ad_msg::ObstacleLidarList::LIDAR_TYPE_VLP_16;
    break;
  default:
    data_out->lidar_type = ad_msg::ObstacleLidarList::LIDAR_TYPE_UNKNOWN;
    break;
  }

  Int32_t obstacle_num = obstacle_lidar_list.obstacles_size();
  if (obstacle_num > ad_msg::ObstacleLidarList::MAX_OBSTACLE_NUM) {
    obstacle_num = ad_msg::ObstacleLidarList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const msg::perception::ObstacleLidar& src =
        obstacle_lidar_list.obstacles(i);
    ad_msg::ObstacleLidar& dst = data_out->obstacles[i];

    dst.id = src.id();

    switch (src.type()) {
    case msg::perception::ObstacleLidar::OBJ_TYPE_UNKNOWN:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_PASSENGER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_COMMERCIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_SPECIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_OTHER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_PEDESTRIAN:
      dst.type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_BICYCLE:
      dst.type = ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_ANIMAL:
      dst.type = ad_msg::OBJ_TYPE_ANIMAL;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_DISCARD:
      dst.type = ad_msg::OBJ_TYPE_DISCARD;
      break;
    case msg::perception::ObstacleLidar::OBJ_TYPE_CURB:
      dst.type = ad_msg::OBJ_TYPE_CURB;
      break;
    default:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    }

    dst.age = src.age();
    dst.prediction_age = src.prediction_age();

    dst.x = src.x();
    dst.y = src.y();

    const msg::perception::ObstacleLidar::OBBox& obb = src.obb();
    dst.obb.x = obb.x();
    dst.obb.y = obb.y();
    dst.obb.heading = obb.heading();
    dst.obb.half_length = obb.half_length();
    dst.obb.half_width = obb.half_width();

    dst.v_x = src.v_x();
    dst.v_y = src.v_y();
    dst.accel_x = src.accel_x();
    dst.accel_y = src.accel_y();
    dst.yaw_rate = src.yaw_rate();
  }

  // Update number of obstacles
  data_out->obstacle_num = obstacle_num;

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeLidarCloudMessage(
    const ad_msg::LidarCloud& msg,
    msg::perception::LidarCloud* const data_out) {
  switch (msg.lidar_type) {
  case (ad_msg::ObstacleLidarList::LIDAR_TYPE_IBEO_4):
    data_out->set_lidar_type(msg::perception::LidarCloud::LIDAR_TYPE_IBEO_4);
    break;
  case (ad_msg::ObstacleLidarList::LIDAR_TYPE_VLP_16):
    data_out->set_lidar_type(msg::perception::LidarCloud::LIDAR_TYPE_VLP_16);
    break;
  default:
    data_out->set_lidar_type(msg::perception::LidarCloud::LIDAR_TYPE_UNKNOWN);
    break;
  }

  for (Int32_t i = 0; i < msg.point_num; ++i) {
    const ad_msg::LidarCloudPoint& src = msg.points[i];
    msg::perception::LidarCloud::CloudPoint* dst = data_out->add_points();
    if (Nullptr_t != dst) {
      dst->set_layer(src.layer);
      dst->set_x(src.x);
      dst->set_y(src.y);
      dst->set_z(src.z);
    }
  }

  return true;
}

bool ParseProtoMsg::DecodeLidarCloudMessage(
    const Char_t* msg, Int32_t msg_len,
    ad_msg::LidarCloud* data_out) {
  // parse
  msg::perception::LidarCloud lidar_cloud;
  if (!lidar_cloud.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse lidar cloud from array.";
    return false;
  }

  switch (lidar_cloud.lidar_type()) {
  case (msg::perception::LidarCloud::LIDAR_TYPE_IBEO_4):
    data_out->lidar_type = ad_msg::LidarCloud::LIDAR_TYPE_IBEO_4;
    break;
  case (msg::perception::LidarCloud::LIDAR_TYPE_VLP_16):
    data_out->lidar_type = ad_msg::LidarCloud::LIDAR_TYPE_VLP_16;
    break;
  default:
    data_out->lidar_type = ad_msg::LidarCloud::LIDAR_TYPE_UNKNOWN;
    break;
  }

  Int32_t point_num = lidar_cloud.points_size();
  if (point_num > ad_msg::LidarCloud::MAX_CLOUD_POINT_NUM) {
    point_num = ad_msg::LidarCloud::MAX_CLOUD_POINT_NUM;
  }
  for (Int32_t i = 0; i < point_num; ++i) {
    const msg::perception::LidarCloud::CloudPoint& src = lidar_cloud.points(i);
    ad_msg::LidarCloudPoint& dst = data_out->points[i];

    dst.layer = src.layer();
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
  }

  // Update number of points
  data_out->point_num = point_num;

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeObstacleListMessage(
    const ad_msg::ObstacleList& msg,
    msg::perception::ObstacleList* const data_out) {
  

  for(Int32_t i = 0; i < msg.obstacle_num; ++i){
      const ad_msg::Obstacle& src = msg.obstacles[i];
      msg::perception::Obstacle* dst = data_out->add_obstacles();

      if(Nullptr_t != dst){
          dst->set_id(src.id);

          switch (src.type) {
          case ad_msg::OBJ_TYPE_UNKNOWN:
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_UNKNOWN);
            break;
          case ad_msg::OBJ_TYPE_PASSENGER_VEHICLE:
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_PASSENGER_VEHICLE);
            break;
          case ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE  :
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_COMMERCIAL_VEHICLE);
            break;
          case ad_msg::OBJ_TYPE_SPECIAL_VEHICLE:
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_SPECIAL_VEHICLE);
            break;
          case ad_msg::OBJ_TYPE_OTHER_VEHICLE:
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_OTHER_VEHICLE);
            break;
          case ad_msg::OBJ_TYPE_PEDESTRIAN:
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_PEDESTRIAN );
            break;
          case ad_msg::OBJ_TYPE_BICYCLE :
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_BICYCLE);
            break;
          case ad_msg::OBJ_TYPE_ANIMAL :
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_ANIMAL);
            break;
          case ad_msg::OBJ_TYPE_DISCARD :
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_DISCARD);
            break;
          case ad_msg::OBJ_TYPE_CURB :
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_CURB);
            break;
          default:
            dst->set_type(msg::perception::Obstacle::OBJ_TYPE_UNKNOWN);
            break;
          }

          msg::perception::Obstacle::OBBox* obb = dst->mutable_obb();
          if(Nullptr_t != obb){
              obb->set_x(src.x);
              obb->set_y(src.y);
              obb->set_heading(src.obb.heading);
              obb->set_half_length(src.obb.half_length);
              obb->set_half_width(src.obb.half_width);
          }else{
              LOG_ERR << "Failed to add obb.";
          }
          dst->set_x(src.x);
          dst->set_y(src.y);
          dst->set_v_x(src.v_x);
          dst->set_v_y(src.v_y);
          dst->set_v(src.v);
          dst->set_a_x(src.a_x);
          dst->set_a_y(src.a_y);
          dst->set_a(src.a);
          dst->set_height(src.height);
          dst->set_height_to_ground(src.height_to_ground);
          dst->set_confidence(src.confidence);
          dst->set_dynamic(src.dynamic);


          // 感知类别
          switch (src.perception_type) {
          case (ad_msg::OBJ_PRCP_TYPE_RADAR):
            dst->set_perception_type(msg::perception::Obstacle::OBJ_PRCP_TYPE_RADAR);
            break;
          case (ad_msg::OBJ_PRCP_TYPE_CAMERA ):
            dst->set_perception_type(msg::perception::Obstacle::OBJ_PRCP_TYPE_CAMERA);
            break;
          case (ad_msg::OBJ_PRCP_TYPE_FUSED ):
            dst->set_perception_type(msg::perception::Obstacle::OBJ_PRCP_TYPE_FUSED);
            break;
          case (ad_msg::OBJ_PRCP_TYPE_LIDAR):
            dst->set_perception_type(msg::perception::Obstacle::OBJ_PRCP_TYPE_LIDAR);
            break;
          default:
            dst->set_perception_type(msg::perception::Obstacle::OBJ_PRCP_TYPE_UNKNOWN);
            break;
          }
          dst->set_dynamic(src.dynamic);

          //TODO: 障碍物在主参考线上的投影信息。proto 文件中缺该声明

          //预测的轨迹的数量
          //预测的轨迹中点的数量
          for(Int32_t m = 0; m < src.pred_path_num; ++m){
              msg::perception::Obstacle::PredPath* pred_path = dst->add_pred_path();
              for(Int32_t n = 0; n < src.pred_path_point_num[m]; ++n){
                  msg::perception::Obstacle::PredPathPoint* pred_path_pts = pred_path->add_points();
                  pred_path_pts->set_heading(src.pred_path[m][n].heading);
                  pred_path_pts->set_s(src.pred_path[m][n].s);
                  pred_path_pts->set_x(src.pred_path[m][n].x);
                  pred_path_pts->set_y(src.pred_path[m][n].y);
              }
          }

          // 障碍物跟踪的轨迹(障碍物历史轨迹)中点的数量
          for(Int32_t k = 0; k < src.tracked_path_point_num; ++k){
              msg::perception::Obstacle::Point2D* track_pts = dst->add_tracked_path();
              track_pts->set_x(src.tracked_path[k].x);
              track_pts->set_y(src.tracked_path[k].y);
          }

      }
  }

  return true;
}

bool ParseProtoMsg::DecodeObstacleListMessage(
    const Char_t* msg, Int32_t msg_len,
    ad_msg::ObstacleList* data_out) {
  msg::perception::ObstacleList obj_list;
  if (!obj_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse obstacle list from array.";
    return false;
  }

  data_out->obstacle_num = obj_list.obstacles_size();
  if (data_out->obstacle_num < 0) {
    data_out->obstacle_num = 0;
  }
  if (data_out->obstacle_num > ad_msg::ObstacleList::MAX_OBSTACLE_NUM) {
    data_out->obstacle_num = ad_msg::ObstacleList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < data_out->obstacle_num; ++i) {
    const msg::perception::Obstacle& obj = obj_list.obstacles(i);
    ad_msg::Obstacle& dst = data_out->obstacles[i];

    // 障碍物ID
    dst.id = obj.id();
    // 障碍物位置x坐标
    dst.x = obj.x();
    // 障碍物位置y坐标
    dst.y = obj.y();
    // 障碍物包围盒
    dst.obb.x = obj.obb().x();
    dst.obb.y = obj.obb().y();
    dst.obb.heading = obj.obb().heading();
    dst.obb.half_width = obj.obb().half_width();
    dst.obb.half_length = obj.obb().half_length();
    // 障碍物高度
    dst.height = obj.height();
    // 离地面高度
    dst.height_to_ground = obj.height_to_ground();
    // 障碍物类型
    switch (obj.type()) {
    case msg::perception::Obstacle::OBJ_TYPE_UNKNOWN:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_PASSENGER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_COMMERCIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_SPECIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_OTHER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_PEDESTRIAN:
      dst.type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_BICYCLE:
      dst.type = ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_ANIMAL:
      dst.type = ad_msg::OBJ_TYPE_ANIMAL;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_DISCARD:
      dst.type = ad_msg::OBJ_TYPE_DISCARD;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_CURB:
      dst.type = ad_msg::OBJ_TYPE_CURB;
      break;
    default:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    }
    // 是否是动态障碍物
    dst.dynamic = obj.dynamic();
    // 障碍物存在的置信度
    dst.confidence = obj.confidence();
    // 感知类别
    switch (obj.perception_type()) {
    case (msg::perception::Obstacle::OBJ_PRCP_TYPE_RADAR):
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_RADAR;
      break;
    case (msg::perception::Obstacle::OBJ_PRCP_TYPE_CAMERA):
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_CAMERA;
      break;
    case (msg::perception::Obstacle::OBJ_PRCP_TYPE_FUSED):
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      break;
    default:
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_UNKNOWN;
      break;
    }
    // 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
    dst.v_x = obj.v_x();
    // 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
    dst.v_y = obj.v_y();
    // 障碍物绝对速度，单位：米/秒
    dst.v = obj.v();
    // 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
    dst.a_x = obj.a_x();
    // 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
    dst.a_y = obj.a_y();
    // 障碍物绝对加速度，单位：米^2/秒
    dst.a = obj.a();
    // 障碍物在主参考线上的投影信息
    dst.proj_on_major_ref_line.valid = false;
    // 预测的轨迹的数量
    dst.pred_path_num = obj.pred_path_size();
    if (dst.pred_path_num < 0) {
      dst.pred_path_num = 0;
    }
    if (dst.pred_path_num > ad_msg::Obstacle::MAX_PRED_PATH_NUM) {
      dst.pred_path_num = ad_msg::Obstacle::MAX_PRED_PATH_NUM;
    }
    // 预测的轨迹中点的数量
    // 对动态障碍物预测的轨迹
    for (Int32_t j = 0; j < dst.pred_path_num; ++j) {
      dst.pred_path_point_num[j] = obj.pred_path(j).points_size();
      if (dst.pred_path_point_num[j] < 0) {
        dst.pred_path_point_num[j] = 0;
      }
      if (dst.pred_path_point_num[j] >
          ad_msg::Obstacle::MAX_PRED_PATH_POINT_NUM) {
        dst.pred_path_point_num[j] = ad_msg::Obstacle::MAX_PRED_PATH_POINT_NUM;
      }
      for (Int32_t k = 0; k < dst.pred_path_point_num[j]; ++k) {
        dst.pred_path[j][k].x = obj.pred_path(j).points(k).x();
        dst.pred_path[j][k].y = obj.pred_path(j).points(k).y();
        dst.pred_path[j][k].heading = obj.pred_path(j).points(k).heading();
        dst.pred_path[j][k].s = obj.pred_path(j).points(k).s();
      }
    }

    // 障碍物跟踪的轨迹(障碍物历史轨迹)中点的数量
    dst.tracked_path_point_num = obj.tracked_path_size();
    if (dst.tracked_path_point_num < 0) {
      dst.tracked_path_point_num = 0;
    }
    if (dst.tracked_path_point_num >
        ad_msg::Obstacle::MAX_TRACKED_PATH_POINT_NUM) {
      dst.tracked_path_point_num = ad_msg::Obstacle::MAX_TRACKED_PATH_POINT_NUM;
    }
    for (Int32_t j = 0; j < dst.tracked_path_point_num; ++j) {
      dst.tracked_path[j].x = obj.tracked_path(j).x();
      dst.tracked_path[j].y = obj.tracked_path(j).y();
    }
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

// AD_HMI rosbag
bool ParseProtoMsg::DecodeADHMIFusionObstacleListMessage(
    const Char_t* msg, Int32_t msg_len,
    ad_msg::ObstacleList* data_out) {

#if USING_NEW_ADHMI_ROSBAG_DATA
  ad_data_upload::Perception_Display_Data_message obj_list;
  if (!obj_list.ParseFromArray(msg+39, msg_len-39)) {
    LOG_ERR << "Failed to parse obstacle list from array.";
    return false;
  }
#else
  ad_data_upload::Perception_Display_Data_message obj_list;
  if (!obj_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse obstacle list from array.";
    return false;
  }
#endif

  data_out->obstacle_num = obj_list.obstacle_list().obstacles_size();
  if (data_out->obstacle_num < 0) {
    data_out->obstacle_num = 0;
  }
  if (data_out->obstacle_num > ad_msg::ObstacleList::MAX_OBSTACLE_NUM) {
    data_out->obstacle_num = ad_msg::ObstacleList::MAX_OBSTACLE_NUM;
  }
  for (Int32_t i = 0; i < data_out->obstacle_num; ++i) {
    const ad_data_upload::Obstacle_message& obj = obj_list.obstacle_list().obstacles(i);
    ad_msg::Obstacle& dst = data_out->obstacles[i];

    // 障碍物ID
    dst.id = obj.id();
    // 障碍物位置x坐标
    dst.x = obj.x();
    // 障碍物位置y坐标
    dst.y = obj.y();
    // 障碍物包围盒
    dst.obb.x = obj.obb().x();
    dst.obb.y = obj.obb().y();
    dst.obb.heading = obj.obb().heading();
    dst.obb.half_width = obj.obb().half_width();
    dst.obb.half_length = obj.obb().half_length();
    // 障碍物高度
    dst.height = obj.height();
    // 离地面高度
    dst.height_to_ground = obj.height_to_ground();
    // 障碍物类型
    switch (obj.type()) {
    case msg::perception::Obstacle::OBJ_TYPE_UNKNOWN:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_PASSENGER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_COMMERCIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_SPECIAL_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_OTHER_VEHICLE:
      dst.type = ad_msg::OBJ_TYPE_OTHER_VEHICLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_PEDESTRIAN:
      dst.type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_BICYCLE:
      dst.type = ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_ANIMAL:
      dst.type = ad_msg::OBJ_TYPE_ANIMAL;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_DISCARD:
      dst.type = ad_msg::OBJ_TYPE_DISCARD;
      break;
    case msg::perception::Obstacle::OBJ_TYPE_CURB:
      dst.type = ad_msg::OBJ_TYPE_CURB;
      break;
    default:
      dst.type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
    }
    // 是否是动态障碍物
    dst.dynamic = obj.dynamic();
    // 障碍物存在的置信度
    dst.confidence = obj.confidence();
    // 感知类别
    switch (obj.perception_type()) {
    case (msg::perception::Obstacle::OBJ_PRCP_TYPE_RADAR):
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_RADAR;
      break;
    case (msg::perception::Obstacle::OBJ_PRCP_TYPE_CAMERA):
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_CAMERA;
      break;
    case (msg::perception::Obstacle::OBJ_PRCP_TYPE_FUSED):
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      break;
    default:
      dst.perception_type = ad_msg::OBJ_PRCP_TYPE_UNKNOWN;
      break;
    }
    // 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
    dst.v_x = obj.v_x();
    // 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
    dst.v_y = obj.v_y();
    // 障碍物绝对速度，单位：米/秒
    dst.v = obj.v();
    // 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
    dst.a_x = obj.a_x();
    // 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
    dst.a_y = obj.a_y();
    // 障碍物绝对加速度，单位：米^2/秒
    dst.a = obj.a();
    // 障碍物在主参考线上的投影信息
    dst.proj_on_major_ref_line.valid = false;
    // 预测的轨迹的数量
    dst.pred_path_num = obj.pred_path_size();
    if (dst.pred_path_num < 0) {
      dst.pred_path_num = 0;
    }
    if (dst.pred_path_num > ad_msg::Obstacle::MAX_PRED_PATH_NUM) {
      dst.pred_path_num = ad_msg::Obstacle::MAX_PRED_PATH_NUM;
    }
    // 预测的轨迹中点的数量
    // 对动态障碍物预测的轨迹
    for (Int32_t j = 0; j < dst.pred_path_num; ++j) {
      dst.pred_path_point_num[j] = obj.pred_path_point_num(j);
      if (dst.pred_path_point_num[j] < 0) {
        dst.pred_path_point_num[j] = 0;
      }
      if (dst.pred_path_point_num[j] >
          ad_msg::Obstacle::MAX_PRED_PATH_POINT_NUM) {
        dst.pred_path_point_num[j] = ad_msg::Obstacle::MAX_PRED_PATH_POINT_NUM;
      }
      for (Int32_t k = 0; k < dst.pred_path_point_num[j]; ++k) {
        /*
        dst.pred_path[j][k].x = obj.pred_path(j).points(k).x();
        dst.pred_path[j][k].y = obj.pred_path(j).points(k).y();
        dst.pred_path[j][k].heading = obj.pred_path(j).points(k).heading();
        dst.pred_path[j][k].s = obj.pred_path(j).points(k).s();
        */
      }
    }

    // 障碍物跟踪的轨迹(障碍物历史轨迹)中点的数量
    dst.tracked_path_point_num = obj.tracked_path_size();
    if (dst.tracked_path_point_num < 0) {
      dst.tracked_path_point_num = 0;
    }
    if (dst.tracked_path_point_num >
        ad_msg::Obstacle::MAX_TRACKED_PATH_POINT_NUM) {
      dst.tracked_path_point_num = ad_msg::Obstacle::MAX_TRACKED_PATH_POINT_NUM;
    }
    for (Int32_t j = 0; j < dst.tracked_path_point_num; ++j) {
      dst.tracked_path[j].x = obj.tracked_path(j).x();
      dst.tracked_path[j].y = obj.tracked_path(j).y();
    }
  }

  // Update message head
  //data_out->msg_head.valid = true;
  //data_out->msg_head.sequence = obj_list.obstacle_list().msg_head().sequence();
  //data_out->msg_head.timestamp = obj_list.obstacle_list().msg_head().timestamp();

  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodeTrafficSignalBoxMessage(
    const ad_msg::TrafficSignalBox& msg,
    msg::perception::TrafficSignalBox* const data_out) {
  data_out->set_x(msg.x);
  data_out->set_y(msg.y);
  data_out->set_z(msg.z);
  data_out->set_width(msg.width);
  data_out->set_height(msg.height);
  data_out->set_camera_id(msg.camera_id);

  return true;
}

bool ParseProtoMsg::DecodeTrafficSignalBoxMessage(
    const msg::perception::TrafficSignalBox& msg,
    ad_msg::TrafficSignalBox* data_out) {
  data_out->x = msg.x();
  data_out->y = msg.y();
  data_out->z = msg.z();
  data_out->width = msg.width();
  data_out->height = msg.height();
  data_out->camera_id = msg.camera_id();

  return true;
}

bool ParseProtoMsg::EncodeTrafficSignalSpeedRestrictionMessage(
    const ad_msg::TrafficSignalSpeedRestriction& msg,
    msg::perception::TrafficSignalSpeedRestriction* const data_out) {
  // Traffic signal string-ID in the map data.
  data_out->set_id(msg.id);
  // 限速标志类型
  switch (msg.type) {
  case (ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION):
    data_out->set_type(
          msg::perception::TrafficSignalSpeedRestriction::START_RESTRICTION);
    break;
  case (ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION):
    data_out->set_type(
          msg::perception::TrafficSignalSpeedRestriction::END_RESTRICTION);
    break;
  default:
    data_out->set_type(msg::perception::TrafficSignalSpeedRestriction::UNKNOWN);
    break;
  }
  // 位置及尺寸
  EncodeTrafficSignalBoxMessage(msg.box, data_out->mutable_box());
  // 限速值 (m/s)
  data_out->set_speed(msg.speed);

  return true;
}

bool ParseProtoMsg::DecodeTrafficSignalSpeedRestrictionMessage(
    const msg::perception::TrafficSignalSpeedRestriction& msg,
    ad_msg::TrafficSignalSpeedRestriction* data_out) {
  // Traffic signal string-ID in the map data.
  data_out->id = msg.id();
  // 限速标志类型
  switch (msg.type()) {
  case (msg::perception::TrafficSignalSpeedRestriction::START_RESTRICTION):
    data_out->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
    break;
  case (msg::perception::TrafficSignalSpeedRestriction::END_RESTRICTION):
    data_out->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
    break;
  default:
    data_out->type = ad_msg::TrafficSignalSpeedRestriction::UNKNOWN;
    break;
  }
  // 位置及尺寸
  DecodeTrafficSignalBoxMessage(msg.box(), &(data_out->box));
  // 限速值 (m/s)
  data_out->speed = msg.speed();

  return true;
}

bool ParseProtoMsg::EncodeTrafficSignalListMessage(
    const ad_msg::TrafficSignalList& msg,
    msg::perception::TrafficSignalList* const data_out) {
  for (Int32_t i = 0; i < msg.speed_restriction_num; ++i) {
    msg::perception::TrafficSignalSpeedRestriction* speed_restriction =
        data_out->add_speed_restrictions();

    EncodeTrafficSignalSpeedRestrictionMessage(
          msg.speed_restrictions[i], speed_restriction);
  }

  return true;
}

bool ParseProtoMsg::DecodeTrafficSignalListMessage(
    const Char_t *msg, Int32_t msg_len, ad_msg::TrafficSignalList *data_out) {
  msg::perception::TrafficSignalList traffic_signal;
  if (!traffic_signal.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse traffic_signal from array.";
    return false;
  }

  data_out->speed_restriction_num = traffic_signal.speed_restrictions_size();
  if (data_out->speed_restriction_num >
      ad_msg::TrafficSignalList::MAX_TRAFFIC_SIGNAL_NUM) {
    data_out->speed_restriction_num =
        ad_msg::TrafficSignalList::MAX_TRAFFIC_SIGNAL_NUM;
  }
  for (Int32_t i = 0; i < data_out->speed_restriction_num; ++i) {
    DecodeTrafficSignalSpeedRestrictionMessage(
          traffic_signal.speed_restrictions(i),
          &(data_out->speed_restrictions[i]));
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}


bool ParseProtoMsg::EncodeTrafficLightListMessage(
    const ad_msg::TrafficLightList& msg,
    msg::perception::TrafficLightList* const data_out) {
  return true;
}

bool ParseProtoMsg::DecodeTrafficLightListMessage(
    const Char_t* msg, Int32_t msg_len,
    ad_msg::TrafficLightList* data_out) {
  msg::perception::TrafficLightList traffic_light_list;
  if (!traffic_light_list.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse traffic_light_list from array.";
    return false;
  }

  data_out->traffic_light_num =  traffic_light_list.traffic_lights_size();
  if (data_out->traffic_light_num >
      ad_msg::TrafficLightList::MAX_TRAFFIC_LIGHT_NUM) {
    data_out->traffic_light_num =
        ad_msg::TrafficLightList::MAX_TRAFFIC_LIGHT_NUM;
  }

  // printf("### 收到了 %d 个红绿灯信号\n", data_out->traffic_light_num);

  for (Int32_t i = 0; i < data_out->traffic_light_num; ++i) {
    const msg::perception::TrafficLight& light =
        traffic_light_list.traffic_lights(i);
    ad_msg::TrafficLight& dst = data_out->traffic_lights[i];

    // Traffic light string-ID in the map data.
  #if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    dst.id = light.id();
  #elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    dst.id = 0;
  #else
    dst.id = 0;
  #endif
    // Color of Traffic Light
    switch (light.color()) {
    case (msg::perception::TrafficLight::RED):
      dst.color = ad_msg::TrafficLight::RED;
      break;
    case (msg::perception::TrafficLight::YELLOW):
      dst.color = ad_msg::TrafficLight::YELLOW;
      break;
    case (msg::perception::TrafficLight::GREEN):
      dst.color = ad_msg::TrafficLight::GREEN;
      break;
    case (msg::perception::TrafficLight::BLACK):
      dst.color = ad_msg::TrafficLight::BLACK;
      break;
    default:
      dst.color = ad_msg::TrafficLight::UNKNOWN;
      break;
    }
    // Box of Traffic Light
    dst.box.x = light.box().x();
    dst.box.y = light.box().y();
    dst.box.width = light.box().width();
    dst.box.height = light.box().height();
    dst.box.camera_id = light.box().camera_id();
    // How confidence about the detected results, between 0 and 100.
    dst.confidence = light.confidence();
    // Duration of the traffic light since detected.
    dst.tracking_time = light.tracking_time();
    // Is traffic blinking
    dst.blink = light.blink();
    // traffic light remaining time, (unit: s, < 0 is invalid).
    dst.remaining_time = light.remaining_time();

    // printf("    红绿灯[%d]:"
    //        " color=%d"
    //        "\n",
    //        i, dst.color);
  }

  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::EncodePlanningResultMessage(
    const ad_msg::PlanningResult& msg,
    msg::planning::PlanningResult* const data_out) {
  // 当前决策模块的状态
  data_out->set_cur_status(msg.cur_status);

  // 驾驶模式请求
  switch (msg.tar_driving_mode) {
  case (ad_msg::VEH_DRIVING_MODE_MANUAL):
    data_out->set_tar_driving_mode(
          msg::planning::PlanningResult::DRIVING_MODE_MANUAL);
    break;
  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC):
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
  // 释放油门
  data_out->set_release_throttle(msg.release_throttle);

  // 档位请求
  switch (msg.tar_gear) {
  case (ad_msg::VEH_GEAR_P):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_P);
    break;
  case (ad_msg::VEH_GEAR_N):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_N);
    break;
  case (ad_msg::VEH_GEAR_R):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_R);
    break;
  case (ad_msg::VEH_GEAR_D):
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_D);
    break;
  default:
    data_out->set_tar_gear(msg::planning::PlanningResult::GEAR_INVALID);
    break;
  }
  // 转向指示灯请求
  switch (msg.tar_turn_lamp) {
  case (ad_msg::VEH_TURN_LAMP_OFF):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_OFF);
    break;
  case (ad_msg::VEH_TURN_LAMP_LEFT):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_LEFT);
    break;
  case (ad_msg::VEH_TURN_LAMP_RIGHT):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_RIGHT);
    break;
  case (ad_msg::VEH_TURN_LAMP_EMERGENCY):
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_EMERGENCY);
    break;
  default:
    data_out->set_tar_turn_lamp(msg::planning::PlanningResult::TURN_LAMP_INVALID);
    break;
  }
  // 制动灯请求
  switch (msg.tar_brake_lamp) {
  case (ad_msg::VEH_LAMP_OFF):
    data_out->set_tar_brake_lamp(msg::planning::PlanningResult::LAMP_OFF);
    break;
  case (ad_msg::VEH_LAMP_ON):
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
  case (ad_msg::PlanningResult::TRJ_DIRECTION_FORWARD):
    tar_trj->set_trj_direction(msg::planning::PlanningResult::TRJ_DIRECTION_FORWARD);
    break;
  case (ad_msg::PlanningResult::TRJ_DIRECTION_BACKWARD):
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
    ad_msg::PlanningResult* data_out) {
  // parse
  msg::planning::PlanningResult planning_result;
  if (!planning_result.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse planning_result from array.";
    return false;
  }

  // 当前决策模块的状态
  data_out->cur_status = planning_result.cur_status();
  // 驾驶模式请求
  switch (planning_result.tar_driving_mode()) {
  case (msg::planning::PlanningResult::DRIVING_MODE_MANUAL):
    data_out->tar_driving_mode = ad_msg::VEH_DRIVING_MODE_MANUAL;
    break;
  case (msg::planning::PlanningResult::DRIVING_MODE_ROBOTIC):
    data_out->tar_driving_mode = ad_msg::VEH_DRIVING_MODE_ROBOTIC;
    break;
  default:
    data_out->tar_driving_mode = ad_msg::VEH_DRIVING_MODE_INVALID;
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
  // 释放油门
  data_out->release_throttle = planning_result.release_throttle();

  // 档位请求
  switch (planning_result.tar_gear()) {
  case (msg::planning::PlanningResult::GEAR_P):
    data_out->tar_gear = ad_msg::VEH_GEAR_P;
    break;
  case (msg::planning::PlanningResult::GEAR_N):
    data_out->tar_gear = ad_msg::VEH_GEAR_N;
    break;
  case (msg::planning::PlanningResult::GEAR_R):
    data_out->tar_gear = ad_msg::VEH_GEAR_R;
    break;
  case (msg::planning::PlanningResult::GEAR_D):
    data_out->tar_gear = ad_msg::VEH_GEAR_D;
    break;
  default:
    data_out->tar_gear = ad_msg::VEH_GEAR_INVALID;
    break;
  }
  // 转向指示灯请求
  switch (planning_result.tar_turn_lamp()) {
  case (msg::planning::PlanningResult::TURN_LAMP_OFF):
    data_out->tar_turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
    break;
  case (msg::planning::PlanningResult::TURN_LAMP_LEFT):
    data_out->tar_turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (msg::planning::PlanningResult::TURN_LAMP_RIGHT):
    data_out->tar_turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (msg::planning::PlanningResult::TURN_LAMP_EMERGENCY):
    data_out->tar_turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    data_out->tar_turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
    break;
  }
  // 制动灯请求
  switch (planning_result.tar_brake_lamp()) {
  case (msg::planning::PlanningResult::LAMP_OFF):
    data_out->tar_brake_lamp = ad_msg::VEH_LAMP_OFF;
    break;
  case (msg::planning::PlanningResult::LAMP_ON):
    data_out->tar_brake_lamp = ad_msg::VEH_LAMP_ON;
    break;
  default:
    data_out->tar_brake_lamp = ad_msg::VEH_LAMP_INVALID;
    break;
  }
  // 速度请求, PH = INT * 0.1 (km/h), Range: [0.0, 200.0 km/h]
  data_out->tar_v = planning_result.tar_v();
  // 加速度请求, PH = INT * 0.01 (m/s^2), Range: [-15.00, 15.00 m/s^2]
  data_out->tar_a = planning_result.tar_a();

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
        ad_msg::PlanningResult::TRJ_DIRECTION_FORWARD;
    break;
  case (msg::planning::PlanningResult::TRJ_DIRECTION_BACKWARD):
    data_out->tar_trj.trj_direction =
        ad_msg::PlanningResult::TRJ_DIRECTION_BACKWARD;
    break;
  default:
    data_out->tar_trj.trj_direction =
        ad_msg::PlanningResult::TRJ_DIRECTION_FORWARD;
    LOG_ERR << "Invalid trajectory direction.";
    break;
  }
  // 轨迹点的数量
  data_out->tar_trj.points_num = tar_trj.points_num();
  if (data_out->tar_trj.points_num >
      ad_msg::PlanningResult::MAX_TRAJECTORY_POINT_NUM) {
    data_out->tar_trj.points_num =
        ad_msg::PlanningResult::MAX_TRAJECTORY_POINT_NUM;
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

  // data_out->tar_trj.timestamp = common::GetClockNowMs();
  // Update message head
  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return true;
}

bool ParseProtoMsg::DecodeSceneStoryControlLine(
    const msg::routing::ControlLine& line_in,
    ad_msg::SceneStoryControlLine* line_out) {
  line_out->start_point.set_x(line_in.start_point().x());
  line_out->start_point.set_y(line_in.start_point().y());
  line_out->end_point.set_x(line_in.end_point().x());
  line_out->end_point.set_y(line_in.end_point().y());

  return true;
}

bool ParseProtoMsg::DecodeSceneStoryCondition(
    const msg::routing::Condition& cond_in,
    ad_msg::SceneStoryCondition* cond_out) {
  if (cond_in.has_start_s() && cond_in.has_end_s()) {
    cond_out->valid_s = 1;
    cond_out->start_s = cond_in.start_s();
    cond_out->end_s = cond_in.end_s();
  } else {
    cond_out->valid_s = 0;
    cond_out->start_s = 0.0F;
    cond_out->end_s = 0.0F;
  }

  if (cond_in.has_speed_high()) {
    cond_out->valid_speed_high = 1;
    cond_out->speed_high = cond_in.speed_high();
  } else {
    cond_out->valid_speed_high = 0;
    cond_out->speed_high = 0.0F;
  }
  if (cond_in.has_speed_low()) {
    cond_out->valid_speed_low = 1;
    cond_out->speed_low = cond_in.speed_low();
  } else {
    cond_out->valid_speed_low = 0;
    cond_out->speed_low = 0.0F;
  }

  /// TODO: T.B.D.
  cond_out->gear = ad_msg::VEH_GEAR_INVALID;

  return true;
}

bool ParseProtoMsg::DecodeSceneStoryAction(
    const msg::routing::Action& action_in,
    ad_msg::SceneStoryAction* action_out) {
  if (action_in.has_run_time()) {
    action_out->holding_time = action_in.run_time() * 1000;
  } else {
    action_out->holding_time = -1;
  }

  if (action_in.has_speed()) {
    action_out->valid_speed = 1;
    action_out->speed = action_in.speed();
  } else {
    action_out->valid_speed = 0;
    action_out->speed = 0.0F;
  }

  if (action_in.has_acceleration()) {
    action_out->valid_acceleration = 1;
    action_out->acceleration = action_in.acceleration();
  } else {
    action_out->valid_acceleration = 0;
    action_out->acceleration = 0.0F;
  }

  if (action_in.has_gear()) {
    switch (action_in.gear()) {
    case (msg::control::Chassis::GEAR_P) :
      action_out->gear = ad_msg::VEH_GEAR_P;
      break;
    case (msg::control::Chassis::GEAR_N) :
      action_out->gear = ad_msg::VEH_GEAR_N;
      break;
    case (msg::control::Chassis::GEAR_R) :
      action_out->gear = ad_msg::VEH_GEAR_R;
      break;
    case (msg::control::Chassis::GEAR_D) :
      action_out->gear = ad_msg::VEH_GEAR_D;
      break;
    default:
      action_out->gear = ad_msg::VEH_GEAR_INVALID;
      break;
    }
  } else {
    action_out->gear = ad_msg::VEH_GEAR_INVALID;
  }

  if (action_in.has_turn_lamp()) {
    switch (action_in.turn_lamp()) {
    case (msg::control::Chassis::TURN_LAMP_OFF):
      action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
      break;
    case (msg::control::Chassis::TURN_LAMP_LEFT):
      action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
      break;
    case (msg::control::Chassis::TURN_LAMP_RIGHT):
      action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
      break;
    case (msg::control::Chassis::TURN_LAMP_EMERGENCY):
      action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
      break;
    default:
      action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
      break;
    }
  } else {
    action_out->turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
  }

  if (action_in.has_brake_lamp()) {
    switch (action_in.brake_lamp()) {
    case (msg::control::Chassis::LAMP_OFF):
      action_out->turn_lamp = ad_msg::VEH_LAMP_OFF;
      break;
    case (msg::control::Chassis::LAMP_ON):
      action_out->turn_lamp = ad_msg::VEH_LAMP_ON;
      break;
    default:
      action_out->turn_lamp = ad_msg::VEH_LAMP_INVALID;
      break;
    }
  } else {
    action_out->turn_lamp = ad_msg::VEH_LAMP_INVALID;
  }

  return true;
}

bool ParseProtoMsg::DecodeSceneStorysMessage(
    const Char_t *msg, Int32_t msg_len,
    ad_msg::SceneStoryList* data_out) {
  // parse
  msg::routing::Scene_Stories scene_storys;
  if (!scene_storys.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse scene_storys from array.";
    return false;
  }

  Int32_t story_list_idx = 0;
  Int32_t story_num = 0;
  // 接近终点
  Int32_t story_size = scene_storys.close_to_destination_size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToDestination data_in =
        scene_storys.close_to_destination(i);

    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_DESTINATION;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }

  // 接近信号灯
  story_size = scene_storys.close_to_signal().size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToSignal data_in = scene_storys.close_to_signal(i);
    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TRAFFIC_SIGNAL;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }

  // 接近弯道
  story_size = scene_storys.close_to_curve_road_size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToCurveRoad data_in =
        scene_storys.close_to_curve_road(i);
    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }
  // longjiaoy
  story_size = scene_storys.close_to_crosswalk_size();
  for (Int32_t i = 0; i < story_size; ++i) {
    if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
      break;
    }
    ad_msg::SceneStory& story = data_out->storys[story_list_idx];
    story_list_idx++;

    const msg::routing::CloseToCrosswalk data_in =
        scene_storys.close_to_crosswalk(i);
    story.id = data_in.id();
    story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CROSSWALK;
    story.distance = data_in.distance();

    DecodeSceneStoryControlLine(data_in.control_line(), &(story.control_line));

    if (data_in.has_condition()) {
      story.condition.vaild = true;
      DecodeSceneStoryCondition(data_in.condition(), &(story.condition));
    } else {
      story.condition.vaild = false;
    }

    if (data_in.has_action()) {
      story.action.vaild = true;
      DecodeSceneStoryAction(data_in.action(), &(story.action));
    } else {
      story.action.vaild = false;
    }
  }
  // lonjiaoy end

  data_out->story_num = story_list_idx;

  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return (true);
}

bool ParseProtoMsg::DecodeMapLocalizationMessage(
    const Char_t *msg, Int32_t msg_len,
    ad_msg::MapLocalization *data_out) {
  // parse
  msg::routing::MapLocalization map_localization;
  if (!map_localization.ParseFromArray(msg, msg_len)) {
    LOG_ERR << "Failed to parse scene_storys from array.";
    return false;
  }

  data_out->curr_lane_id = map_localization.point().nearest_lane_id();
  data_out->s = map_localization.point().s();
  data_out->l = map_localization.point().l();
  data_out->heading = map_localization.point().heading();

  data_out->msg_head.valid = true;
  data_out->msg_head.UpdateSequenceNum();
  data_out->msg_head.timestamp = common::GetClockNowMs();

  return (true);
}

}  // namespace framework
}  // namespace phoenix

