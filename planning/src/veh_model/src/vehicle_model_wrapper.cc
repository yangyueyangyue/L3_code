//
#include "vehicle_model_wrapper.h"

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
#include "df_d17_b1/vehicle_model_df_d17_b1.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
#include "df_d17_b2/vehicle_model_df_d17_b2.h"
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
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetVehicleLength();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetVehicleLength();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetVehicleLength(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetVehicleLength(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetVehicleLength(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetVehicleWidth() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetVehicleWidth();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetVehicleWidth();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetVehicleWidth(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetVehicleWidth(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetVehicleWidth(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetVehicleHeight() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetVehicleHeight();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetVehicleHeight();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetVehicleHeight(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetVehicleHeight(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetVehicleHeight(value);
#else
  Error: Vehicle model have not been defined.
#endif
}


VehicleModelWrapper::Scalar VehicleModelWrapper::GetTrailerLength() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetTrailerLength();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetTrailerLength();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetTrailerLength(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetTrailerLength(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetTrailerLength(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetTrailerWidth() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetTrailerWidth();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetTrailerWidth();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetTrailerWidth(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetTrailerWidth(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetTrailerWidth(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetTrailerHeight() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetTrailerHeight();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetTrailerHeight();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetTrailerHeight(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetTrailerHeight(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetTrailerHeight(value);
#else
  Error: Vehicle model have not been defined.
#endif
}


VehicleModelWrapper::Scalar
VehicleModelWrapper::GetDistOfLocalizationToFront() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetDistOfLocalizationToFront();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetDistOfLocalizationToFront();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetDistOfLocalizationToFront(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetDistOfLocalizationToFront(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetDistOfLocalizationToFront(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::GetDistOfLocalizationToRear() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetDistOfLocalizationToRear();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetDistOfLocalizationToRear();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetDistOfLocalizationToRear(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetDistOfLocalizationToRear(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetDistOfLocalizationToRear(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::GetDistOfLocalizationToCenter() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetDistOfLocalizationToCenter();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetDistOfLocalizationToCenter();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetDistOfLocalizationToCenter(Scalar value) {
#if  (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetDistOfLocalizationToCenter(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetDistOfLocalizationToCenter(value);
#else
  Error: Vehicle model have not been defined.
#endif
}


VehicleModelWrapper::Scalar VehicleModelWrapper::GetMaxSteeringAngle() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetMaxSteeringAngle();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetMaxSteeringAngle();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetMaxSteeringAngle(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetMaxSteeringAngle(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetMaxSteeringAngle(value);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::GetWheelbase() const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->GetWheelbase();
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->GetWheelbase();
#else
  Error: Vehicle model have not been defined.
#endif
}

void VehicleModelWrapper::SetWheelbase(Scalar value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->SetWheelbase(value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->SetWheelbase(value);
#else
  Error: Vehicle model have not been defined.
#endif
}


VehicleModelWrapper::Scalar
VehicleModelWrapper::ClampMaxSteeringAngle(Scalar angle) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->ClampMaxSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->ClampMaxSteeringAngle(angle);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar
VehicleModelWrapper::ClampMaxSteeringAngleByVelocity(
    Scalar angle, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->ClampMaxSteeringAngleByVelocity(angle, v);
#else
  Error: Vehicle model have not been defined.
#endif
}


VehicleModelWrapper::Scalar VehicleModelWrapper::CalcYawRateFromSteeringAngle(
    Scalar angle, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->CalcYawRateFromSteeringAngle(angle, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->CalcYawRateFromSteeringAngle(angle, v);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcSteeringAngleFromYawRate(
    Scalar yaw_rate, Scalar v) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->CalcSteeringAngleFromYawRate(yaw_rate, v);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcCurvatureFromSteeringAngle(
    Scalar angle) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->CalcCurvatureFromSteeringAngle(angle);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->CalcCurvatureFromSteeringAngle(angle);
#else
  Error: Vehicle model have not been defined.
#endif
}

VehicleModelWrapper::Scalar VehicleModelWrapper::CalcSteeringAngleFromCurvature(
    Scalar curvature) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->CalcSteeringAngleFromCurvature(curvature);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->CalcSteeringAngleFromCurvature(curvature);
#else
  Error: Vehicle model have not been defined.
#endif
}


void VehicleModelWrapper::EstimateNextPos(
    const Scalar v, const Scalar yaw_rate, const Scalar dt,
    const Scalar pos[3], Scalar next_pos[3]) const {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  return VehicleModelDfD17B1::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  return VehicleModelDfD17B2::instance()->EstimateNextPos(v, yaw_rate, dt, pos, next_pos);
#else
  Error: Vehicle model have not been defined.
#endif
}


}  // namespace veh_model
}  // namespace phoenix

