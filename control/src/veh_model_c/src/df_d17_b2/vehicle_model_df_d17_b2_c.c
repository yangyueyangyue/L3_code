////

#include "df_d17_b2/vehicle_model_df_d17_b2_c.h"
#include "math/math_utils_c.h"
#include"utils/linear_interpolation_c.h"


//根据速度限制方向盘角度限制表格大小
#define LMT_STEER_ANGLE_BY_SPD_TABLE_SIZE (15)
// enum { STEER_ANGLE_BY_SPD_TABLE_SIZE =15 };

static Float32_t s_max_steering_angle = 600.0F * 0.017453293F;
static Float32_t s_wheelbase = 4.05F;
static Float32_t s_wheelbase_inverse = 1.0F / 4.05F;
static Float32_t s_steering_gear_ratio = 16.0F;      //18-21
/*d001 fuyuanyi 2023-12-29 (begin) */
// 增加根据速度限制方向盘角度
// 最大方向盘转角限制
static const Float32_t  s_key_tab_lmt_steer_angle_by_spd[LMT_STEER_ANGLE_BY_SPD_TABLE_SIZE] = {
      0.0F/3.6F,       10.0F/3.6F,     20.0F/3.6F,    30.0F/3.6F,    40.0F/3.6F,  
      50.0F/3.6F,     60.0F/3.6F,    70.0F/3.6F,     80.0F/3.6F,    90.0F/3.6F,
      100.0F/3.6F,  110.0F/3.6F,  120.0F/3.6F,  130.0F/3.6F,  140.0F/3.6F
};
static const Float32_t s_value_tab_lmt_steer_angle_by_spd[LMT_STEER_ANGLE_BY_SPD_TABLE_SIZE] = {
       60.0F*0.017453293F,   60.0F*0.017453293F,   60.0F*0.017453293F,  60.0F*0.017453293F,  55.0F*0.017453293F,  
       45.0F*0.017453293F,   40.0F*0.017453293F,   40.0F*0.017453293F,  35.0F*0.017453293F,  35.0F*0.017453293F, 
       35.0F*0.017453293F,   35.0F*0.017453293F,   35.0F*0.017453293F,  35.0F*0.017453293F,  35.0F*0.017453293F
};
/* f001 fuyuanyi 2023-12-29 (end) */


static Float32_t CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v, Float32_t gear_ratio) {
  return (phoenix_com_tan_f(angle / gear_ratio) * v * s_wheelbase_inverse);
}

static Float32_t CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v, Float32_t gear_ratio) {
  return (phoenix_com_atan2_f(yaw_rate * s_wheelbase / v, 1.0F) * gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B2_GetMaxSteeringAngle() {
  return s_max_steering_angle;
}

Float32_t Phoenix_VehModel_DfD17B2_GetWheelbase() {
  return s_wheelbase;
}

Float32_t Phoenix_VehModel_DfD17B2_ClampMaxSteeringAngle(Float32_t angle) {
  if (angle > s_max_steering_angle) {
    return (s_max_steering_angle);
  } else if (angle < -s_max_steering_angle) {
    return (-s_max_steering_angle);
  } else {
    return (angle);
  }
}

Float32_t Phoenix_VehModel_DfD17B2_CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v) {
  return CalcYawRateFromSteeringAngle(angle, v, s_steering_gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B2_CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v) {
  return CalcSteeringAngleFromYawRate(
        yaw_rate, v > 0.01F ? v : 0.01F, s_steering_gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B2_CalcCurvatureFromSteeringAngle(
    Float32_t angle) {
  // curvature = yaw_rate / v
  return (phoenix_com_tan_f(angle / s_steering_gear_ratio) *
          s_wheelbase_inverse);
}

Float32_t Phoenix_VehModel_DfD17B2_CalcSteeringAngleFromCurvature(
    Float32_t curvature) {
  // curvature = yaw_rate / v
  return (phoenix_com_atan2_f(curvature * s_wheelbase, 1.0F) *
          s_steering_gear_ratio);
}


Float32_t Phoenix_VehModel_DfD17B2_ClampMaxSteeringAngleByVelocity(
    Float32_t angle, Float32_t v) {
   /*d001 fuyuanyi 2023-12-29 (begin) */
  // 插值查表
  Float32_t limited_angle = angle;
  Float32_t key_ratio_steer_angle = 0.0F;
  Float32_t max_steer_angle = 0.0F;
  Int32_t key_idx_max_steer_angle =Phoenix_Common_LerpInOrderedTable_f(
      s_key_tab_lmt_steer_angle_by_spd,
      LMT_STEER_ANGLE_BY_SPD_TABLE_SIZE,
      v,
      &key_ratio_steer_angle );

  max_steer_angle = Phoenix_Common_Lerp_f(
      s_value_tab_lmt_steer_angle_by_spd[ key_idx_max_steer_angle],
      s_value_tab_lmt_steer_angle_by_spd[ key_idx_max_steer_angle + 1],
      key_ratio_steer_angle);
  if (angle > max_steer_angle) {
    limited_angle = max_steer_angle;
  } else if (angle < -max_steer_angle) {
    limited_angle = -max_steer_angle;
  } else {
    // nothing to do
  }
  /*d001 fuyuanyi 2023-12-29 (end) */
  
//  if (common::com_abs(angle - limited_angle) >
//      common::NumLimits<Scalar>::epsilon()) {
//    LOG_WARN << "The steering angle exceeds the angle "
//                "limited by velocity factor, angle="
//             << common::com_rad2deg(angle)
//             << "deg, limited_angle=" << common::com_rad2deg(limited_angle)
//             << "deg, v=" << v*3.6F << "km/h.";
//  }

  return (limited_angle);
}


