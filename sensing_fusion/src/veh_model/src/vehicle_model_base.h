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
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_BASE_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_BASE_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "vehicle_model.h"

namespace phoenix {
namespace veh_model {


class VehicleModelBase {
public:
  typedef Float32_t Scalar;

public:
  VehicleModelBase();
  virtual ~VehicleModelBase();


  Scalar GetVehicleLength() const {
    return (vehicle_length_);
  }
  void SetVehicleLength(Scalar value) {
    vehicle_length_ = value;
  }

  Scalar GetVehicleWidth() const {
    return (vehicle_width_);
  }
  void SetVehicleWidth(Scalar value) {
    vehicle_width_ = value;
  }

  Scalar GetDistOfLocalizationToFront() const {
    return (dist_of_localization_to_front_);
  }
  void SetDistOfLocalizationToFront(Scalar value) {
    dist_of_localization_to_front_ = value;
  }

  Scalar GetDistOfLocalizationToRear() const {
    return (dist_of_localization_to_rear_);
  }
  void SetDistOfLocalizationToRear(Scalar value) {
    dist_of_localization_to_rear_ = value;
  }

  Scalar GetDistOfLocalizationToCenter() const {
    return (dist_of_localization_to_center_);
  }
  void SetDistOfLocalizationToCenter(Scalar value) {
    dist_of_localization_to_center_ = value;
  }


  Scalar GetMaxSteeringAngle() const {
    return (max_steering_angle_);
  }

  Scalar GetWheelbase() const {
    return (wheelbase_);
  }

  void SetMaxSteeringAngle(Scalar angle) {
    max_steering_angle_ = angle;
  }

  void SetWheelbase(Scalar len) {
    wheelbase_ = len;
    wheelbase_inverse_ = 1.0F / wheelbase_;
  }

  Scalar ClampMaxSteeringAngle(Scalar angle) const {
    if (angle > max_steering_angle_) {
      return (max_steering_angle_);
    } else if (angle < -max_steering_angle_) {
      return (-max_steering_angle_);
    } else {
      return (angle);
    }
  }

  virtual Scalar ClampMaxSteeringAngleByVelocity(Scalar angle, Scalar v) const;

  inline Scalar CalcYawRateFromSteeringAngle(
      Scalar angle, Scalar v, Scalar gear_ratio) const {
    COM_CHECK(gear_ratio > common::NumLimits<Scalar>::epsilon());

    return (common::com_tan(angle / gear_ratio) * v * wheelbase_inverse_);
  }

  inline Scalar CalcSteeringAngleFromYawRate(
      Scalar yaw_rate, Scalar v, Scalar gear_ratio) const {
    COM_CHECK(v > common::NumLimits<Scalar>::epsilon());

    return (common::com_atan2(yaw_rate * wheelbase_ / v, 1.0F) * gear_ratio);
  }

  inline Scalar CalcCurvatureFromSteeringAngle(
      Scalar angle, Scalar gear_ratio) const {
    // curvature = yaw_rate / v
    return (common::com_tan(angle / gear_ratio) * wheelbase_inverse_);
  }

  inline Scalar CalcSteeringAngleFromCurvature(
      Scalar curvature, Scalar gear_ratio) const {
    // curvature = yaw_rate / v
    return (common::com_atan2(curvature * wheelbase_, 1.0F) * gear_ratio);
  }

  inline virtual Scalar CalcYawRateFromSteeringAngle(
      Scalar angle, Scalar v) const {
    LOG_WARN << "Don't call this function "
                "\"VehicleModelBase::CalcYawRateFromSteeringAngle\".";
    return (0.0F);
  }

  inline virtual Scalar CalcSteeringAngleFromYawRate(
      Scalar yaw_rate, Scalar v) const {
    LOG_WARN << "Don't call this function "
                "\"VehicleModelBase::CalcSteeringAngleFromYawRate\".";
    return (0.0F);
  }

  void EstimateNextPos(
      const Scalar v, const Scalar yaw_rate, const Scalar dt,
      const Scalar pos[3], Scalar next_pos[3]) const;

  inline const SensorOnVehicle& GetSensor() const {
    return (sensor_);
  }

protected:
  // 车长，单位：米
  Scalar vehicle_length_;
  // 车宽，单位：米
  Scalar vehicle_width_;
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  Scalar dist_of_localization_to_front_;
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  Scalar dist_of_localization_to_rear_;
  // 车辆的定位点到中心点的距离，单位：米
  Scalar dist_of_localization_to_center_;

  Scalar max_steering_angle_;
  Scalar wheelbase_;
  Scalar wheelbase_inverse_;

  SensorOnVehicle sensor_;
};


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_BASE_H_

