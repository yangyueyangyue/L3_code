//
#include "vehicle_model_c.h"
#include "math/math_utils_c.h"

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
#include "df_d17_b1/vehicle_model_df_d17_b1_c.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
#include "df_d17_b2/vehicle_model_df_d17_b2_c.h"
#else
  Error: Invalid vehicle platform.
#endif


Float32_t Phoenix_VehModel_GetMaxSteeringAngle() {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_GetMaxSteeringAngle());
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_GetMaxSteeringAngle());
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_GetWheelbase() {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_GetWheelbase());
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_GetWheelbase());
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_ClampMaxSteeringAngle(Float32_t angle) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_ClampMaxSteeringAngle(angle));
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_ClampMaxSteeringAngle(angle));
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_ClampMaxSteeringAngleByVelocity(
    Float32_t angle, Float32_t v) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_ClampMaxSteeringAngleByVelocity(angle, v));
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_ClampMaxSteeringAngleByVelocity(angle, v));
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_CalcYawRateFromSteeringAngle(angle, v));
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_CalcYawRateFromSteeringAngle(angle, v));
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_CalcSteeringAngleFromYawRate(yaw_rate, v));
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_CalcSteeringAngleFromYawRate(yaw_rate, v));
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_CalcCurvatureFromSteeringAngle(
    Float32_t angle) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_CalcCurvatureFromSteeringAngle(angle));
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_CalcCurvatureFromSteeringAngle(angle));
#else
  Error: Invalid vehicle platform.
#endif
}

Float32_t Phoenix_VehModel_CalcSteeringAngleFromCurvature(
    Float32_t curvature) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return (Phoenix_VehModel_DfD17B1_CalcSteeringAngleFromCurvature(curvature));
#elif  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return (Phoenix_VehModel_DfD17B2_CalcSteeringAngleFromCurvature(curvature));
#else
  Error: Invalid vehicle platform.
#endif
}

void Phoenix_VehModel_EstimateNextPos(
    const Float32_t v, const Float32_t yaw_rate, const Float32_t dt,
    const Float32_t pos[3], Float32_t next_pos[3]) {
  if ((-0.001F < yaw_rate) && (yaw_rate < 0.001F)) {
    next_pos[0] = pos[0] + v * phoenix_com_cos_f(pos[2]) * dt;
    next_pos[1] = pos[1] + v * phoenix_com_sin_f(pos[2]) * dt;
    next_pos[2] = Phoenix_Common_NormalizeAngle_f(pos[2] + yaw_rate * dt);
  } else {
    Float32_t r = v / yaw_rate;
    Float32_t heading_changed =
        Phoenix_Common_NormalizeAngle_f(pos[2] + yaw_rate * dt);
    next_pos[0] = pos[0] +
        r * (-phoenix_com_sin_f(pos[2]) + phoenix_com_sin_f(heading_changed));
    next_pos[1] = pos[1] +
        r * ( phoenix_com_cos_f(pos[2]) - phoenix_com_cos_f(heading_changed));
    next_pos[2] = heading_changed;
  }
}

