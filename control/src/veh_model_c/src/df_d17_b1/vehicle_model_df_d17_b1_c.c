////

#include "df_d17_b1/vehicle_model_df_d17_b1_c.h"
#include "math/math_utils_c.h"


static Float32_t s_max_steering_angle = 600.0F * 0.017453293F;
static Float32_t s_wheelbase = 4.05F;
static Float32_t s_wheelbase_inverse = 1.0F / 4.05F;
static Float32_t s_steering_gear_ratio = 16.0F;      //18-21


static Float32_t CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v, Float32_t gear_ratio) {
  return (phoenix_com_tan_f(angle / gear_ratio) * v * s_wheelbase_inverse);
}

static Float32_t CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v, Float32_t gear_ratio) {
  return (phoenix_com_atan2_f(yaw_rate * s_wheelbase / v, 1.0F) * gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B1_GetMaxSteeringAngle() {
  return s_max_steering_angle;
}

Float32_t Phoenix_VehModel_DfD17B1_GetWheelbase() {
  return s_wheelbase;
}

Float32_t Phoenix_VehModel_DfD17B1_ClampMaxSteeringAngle(Float32_t angle) {
  if (angle > s_max_steering_angle) {
    return (s_max_steering_angle);
  } else if (angle < -s_max_steering_angle) {
    return (-s_max_steering_angle);
  } else {
    return (angle);
  }
}

Float32_t Phoenix_VehModel_DfD17B1_CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v) {
  return CalcYawRateFromSteeringAngle(angle, v, s_steering_gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B1_CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v) {
  return CalcSteeringAngleFromYawRate(
        yaw_rate, v > 0.01F ? v : 0.01F, s_steering_gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B1_CalcCurvatureFromSteeringAngle(
    Float32_t angle) {
  // curvature = yaw_rate / v
  return (phoenix_com_tan_f(angle / s_steering_gear_ratio) *
          s_wheelbase_inverse);
}

Float32_t Phoenix_VehModel_DfD17B1_CalcSteeringAngleFromCurvature(
    Float32_t curvature) {
  // curvature = yaw_rate / v
  return (phoenix_com_atan2_f(curvature * s_wheelbase, 1.0F) *
          s_steering_gear_ratio);
}

Float32_t Phoenix_VehModel_DfD17B1_ClampMaxSteeringAngleByVelocity(
    Float32_t angle, Float32_t v) {
  Float32_t limited_angle = angle;

  if (v > 80.0F/3.6F) {
    if (angle > phoenix_com_deg2rad_f(20.0F)) {
      limited_angle = phoenix_com_deg2rad_f(20.0F);
    } else if (angle < phoenix_com_deg2rad_f(-20.0F)) {
      limited_angle = phoenix_com_deg2rad_f(-20.0F);
    }
  } else if (v > 70.0F/3.6F) {
    if (angle > phoenix_com_deg2rad_f(20.0F)) {
      limited_angle = phoenix_com_deg2rad_f(20.0F);
    } else if (angle < phoenix_com_deg2rad_f(-20.0F)) {
      limited_angle = phoenix_com_deg2rad_f(-20.0F);
    }
  } else if (v > 60.0F/3.6F) {
    if (angle > phoenix_com_deg2rad_f(20.0F)) {
      limited_angle = phoenix_com_deg2rad_f(20.0F);
    } else if (angle < phoenix_com_deg2rad_f(-20.0F)) {
      limited_angle = phoenix_com_deg2rad_f(-20.0F);
    }
  } else if (v > 50.0F/3.6F) {
    if (angle > phoenix_com_deg2rad_f(25.0F)) {
      limited_angle = phoenix_com_deg2rad_f(25.0F);
    } else if (angle < phoenix_com_deg2rad_f(-25.0F)) {
      limited_angle = phoenix_com_deg2rad_f(-25.0F);
    }
  } else if (v > 40.0F/3.6F) {
    if (angle > phoenix_com_deg2rad_f(30.0F)) {
      limited_angle = phoenix_com_deg2rad_f(30.0F);
    } else if (angle < phoenix_com_deg2rad_f(-30.0F)) {
      limited_angle = phoenix_com_deg2rad_f(-30.0F);
    }
  }

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


