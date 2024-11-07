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
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_QIN_EV_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_QIN_EV_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "vehicle_model_base.h"

namespace phoenix {
namespace veh_model {


class VehicleModelQinEv : public VehicleModelBase {
public:
  typedef VehicleModelBase::Scalar Scalar;
  typedef VehicleModelBase Base;

public:
  virtual ~VehicleModelQinEv();

  inline Scalar CalcYawRateFromSteeringAngle(
      Scalar angle, Scalar v) const {
    return Base::CalcYawRateFromSteeringAngle(
          angle, v, steering_gear_ratio_);
  }

  inline Scalar CalcSteeringAngleFromYawRate(
      Scalar yaw_rate, Scalar v) const {
    return Base::CalcSteeringAngleFromYawRate(
          yaw_rate, v, steering_gear_ratio_);
  }

  inline Scalar CalcCurvatureFromSteeringAngle(Scalar angle) const {
    return (Base::CalcCurvatureFromSteeringAngle(
              angle, steering_gear_ratio_));
  }

  inline Scalar CalcSteeringAngleFromCurvature(Scalar curvature) const {
    return (Base::CalcSteeringAngleFromCurvature(
              curvature, steering_gear_ratio_));
  }

  Scalar ClampMaxSteeringAngleByVelocity(
      Scalar angle, Scalar v) const;

private:
  Scalar steering_gear_ratio_;

private:
  DECLARE_SINGLETON(VehicleModelQinEv);
};


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_QIN_EV_H_

