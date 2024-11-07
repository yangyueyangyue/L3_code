//
#include "vehicle_model_wrapper.h"
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
#include "qin_ev/vehicle_model_qin_ev.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
#include "ft_auman/vehicle_model_ft_auman.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
#include "df_x320/vehicle_model_df_x320.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
#include "xd_eant/vehicle_model_xd_eant.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
#include "df_d17/vehicle_model_df_d17.h"
#else
  Error: Invalid vehicle platform.
#endif


namespace phoenix {
namespace veh_model {


VehicleModelWrapper::VehicleModelWrapper() {
}

VehicleModelWrapper::~VehicleModelWrapper() {
  // nothing to do
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetVehicleLength() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetVehicleLength();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetVehicleLength();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetVehicleLength();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetVehicleLength();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetVehicleLength();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetVehicleWidth() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetVehicleWidth();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetVehicleWidth();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetVehicleWidth();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetVehicleWidth();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetVehicleWidth();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::GetDistOfLocalizationToFront() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetDistOfLocalizationToFront();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetDistOfLocalizationToFront();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetDistOfLocalizationToFront();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetDistOfLocalizationToFront();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetDistOfLocalizationToFront();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::GetDistOfLocalizationToRear() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetDistOfLocalizationToRear();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetDistOfLocalizationToRear();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetDistOfLocalizationToRear();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetDistOfLocalizationToRear();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetDistOfLocalizationToRear();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::GetDistOfLocalizationToCenter() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetDistOfLocalizationToCenter();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetDistOfLocalizationToCenter();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetDistOfLocalizationToCenter();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetDistOfLocalizationToCenter();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetDistOfLocalizationToCenter();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetMaxSteeringAngle() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetMaxSteeringAngle();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetMaxSteeringAngle();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetMaxSteeringAngle();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetMaxSteeringAngle();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetMaxSteeringAngle();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetWheelbase() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetWheelbase();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetWheelbase();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetWheelbase();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetWheelbase();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetWheelbase();
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::ClampMaxSteeringAngle(Scalar angle) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->ClampMaxSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->ClampMaxSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->ClampMaxSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->ClampMaxSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->ClampMaxSteeringAngle(angle);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::ClampMaxSteeringAngleByVelocity(
    Scalar angle, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcYawRateFromSteeringAngle(
    Scalar angle, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->CalcYawRateFromSteeringAngle(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->CalcYawRateFromSteeringAngle(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->CalcYawRateFromSteeringAngle(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->CalcYawRateFromSteeringAngle(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->CalcYawRateFromSteeringAngle(angle, v);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcSteeringAngleFromYawRate(
    Scalar yaw_rate, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcCurvatureFromSteeringAngle(
    Scalar angle) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->CalcCurvatureFromSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->CalcCurvatureFromSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->CalcCurvatureFromSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->CalcCurvatureFromSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->CalcCurvatureFromSteeringAngle(angle);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcSteeringAngleFromCurvature(
    Scalar curvature) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->CalcSteeringAngleFromCurvature(curvature);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->CalcSteeringAngleFromCurvature(curvature);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->CalcSteeringAngleFromCurvature(curvature);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->CalcSteeringAngleFromCurvature(curvature);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->CalcSteeringAngleFromCurvature(curvature);
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::EstimateNextPos(
    const Scalar v, const Scalar yaw_rate, const Scalar dt,
    const Scalar pos[3], Scalar next_pos[3]) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#else
  Error: Vehicle model have not been defined.
#endif
}

const SensorOnVehicle& VehicleModelWrapper::GetSensor() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  return VehicleModelQinEv::instance()->GetSensor();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  return VehicleModelFtAuman::instance()->GetSensor();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  return VehicleModelDfX320::instance()->GetSensor();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  return VehicleModelXdEAnt::instance()->GetSensor();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  return VehicleModelDfD17::instance()->GetSensor();
#else
  Error: Vehicle model have not been defined.
#endif
}



}  // namespace veh_model
}  // namespace phoenix

