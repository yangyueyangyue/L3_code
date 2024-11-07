/******************************************************************************
 ** 车身参数信息
 ******************************************************************************
 *
 *  车身参数信息(保存车身参数)
 *
 *  @file       vehicle_model.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_WRAPPER_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_WRAPPER_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "vehicle_model.h"


namespace phoenix {
namespace veh_model {


class VehicleModelWrapper {
public:
  typedef Float32_t Scalar;

public:
  VehicleModelWrapper();
  ~VehicleModelWrapper();

  Scalar GetVehicleLength() const;
  Scalar GetVehicleWidth() const;
  Scalar GetDistOfLocalizationToFront() const;
  Scalar GetDistOfLocalizationToRear() const;
  Scalar GetDistOfLocalizationToCenter() const;

  Scalar GetMaxSteeringAngle() const;

  Scalar GetWheelbase() const;

  Scalar ClampMaxSteeringAngle(Scalar angle) const;

  Scalar ClampMaxSteeringAngleByVelocity(Scalar angle, Scalar v) const;

  Scalar CalcYawRateFromSteeringAngle(
      Scalar angle, Scalar v) const;

  Scalar CalcSteeringAngleFromYawRate(
      Scalar yaw_rate, Scalar v) const;

  Scalar CalcCurvatureFromSteeringAngle(Scalar angle) const;

  Scalar CalcSteeringAngleFromCurvature(Scalar curvature) const;

  void EstimateNextPos(
      const Scalar v, const Scalar yaw_rate, const Scalar dt,
      const Scalar pos[3], Scalar next_pos[3]) const;

  const SensorOnVehicle& GetSensor() const;
};


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_WRAPPER_H_

