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


  /// 外形参数
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

  Scalar GetVehicleHeight() const {
    return (vehicle_height_);
  }
  void SetVehicleHeight(Scalar value) {
    vehicle_height_ = value;
  }


  /// 挂车参数
  Scalar GetTrailerLength() const {
    return (trailer_length_);
  }
  void SetTrailerLength(Scalar value) {
    trailer_length_ = value;
  }

  Scalar GetTrailerWidth() const {
    return (trailer_width_);
  }
  void SetTrailerWidth(Scalar value) {
    trailer_width_ = value;
  }

  Scalar GetTrailerHeight() const {
    return (trailer_height_);
  }
  void SetTrailerHeight(Scalar value) {
    trailer_height_ = value;
  }


  /// 车身坐标系参数
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


  /// 运动学参数
  Scalar GetMaxSteeringAngle() const {
    return (max_steering_angle_);
  }
  void SetMaxSteeringAngle(Scalar angle) {
    max_steering_angle_ = angle;
  }

  Scalar GetSteeringGearRatio() const {
    return (steering_gear_ratio_);
  }
  void SetSteeringGearRatio(Scalar ratio) {
    steering_gear_ratio_ = ratio;
  }

  Scalar GetWheelbase() const {
    return (wheelbase_);
  }
  void SetWheelbase(Scalar len) {
    wheelbase_ = len;
    wheelbase_inverse_ = 1.0F / wheelbase_;
  }


  /// 限制方向盘转角
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

  /// 使用方向盘转角计算角速度
  inline Scalar CalcYawRateFromSteeringAngle(
      Scalar angle, Scalar v, Scalar gear_ratio) const {
    COM_CHECK(gear_ratio > common::NumLimits<Scalar>::epsilon());

    return (common::com_tan(angle / gear_ratio) * v * wheelbase_inverse_);
  }
  inline virtual Scalar CalcYawRateFromSteeringAngle(
      Scalar angle, Scalar v) const {
    return CalcYawRateFromSteeringAngle(angle, v, steering_gear_ratio_);
  }

  /// 使用角速度计算方向盘转角
  inline Scalar CalcSteeringAngleFromYawRate(
      Scalar yaw_rate, Scalar v, Scalar gear_ratio) const {
    COM_CHECK(v > common::NumLimits<Scalar>::epsilon());

    return (common::com_atan2(yaw_rate * wheelbase_ / v, 1.0F) * gear_ratio);
  }
  inline virtual Scalar CalcSteeringAngleFromYawRate(
      Scalar yaw_rate, Scalar v) const {
    return CalcSteeringAngleFromYawRate(yaw_rate, v, steering_gear_ratio_);
  }

  /// 使用方向盘转角计算曲率
  inline Scalar CalcCurvatureFromSteeringAngle(
      Scalar angle, Scalar gear_ratio) const {
    // curvature = yaw_rate / v
    return (common::com_tan(angle / gear_ratio) * wheelbase_inverse_);
  }
  inline virtual Scalar CalcCurvatureFromSteeringAngle(Scalar angle) const {
    return (CalcCurvatureFromSteeringAngle(angle, steering_gear_ratio_));
  }

  /// 使用曲率计算方向盘转角
  inline Scalar CalcSteeringAngleFromCurvature(
      Scalar curvature, Scalar gear_ratio) const {
    // curvature = yaw_rate / v
    return (common::com_atan2(curvature * wheelbase_, 1.0F) * gear_ratio);
  }
  inline virtual Scalar CalcSteeringAngleFromCurvature(Scalar curvature) const {
    return (CalcSteeringAngleFromCurvature(curvature, steering_gear_ratio_));
  }

  /// 根据当前状态估计下一时刻的位姿
  void EstimateNextPos(
      const Scalar v, const Scalar yaw_rate, const Scalar dt,
      const Scalar pos[3], Scalar next_pos[3]) const;

protected:
  // 车长，单位：米
  Scalar vehicle_length_;
  // 车宽，单位：米
  Scalar vehicle_width_;
  // 车高，单位：米
  Scalar vehicle_height_;

  // 挂车长度，单位：米
  Scalar trailer_length_;
  // 挂车宽度，单位：米
  Scalar trailer_width_;
  // 挂车高度, 单位：米
  Scalar trailer_height_;

  // 车辆的定位点到 front of vehicle 的距离，单位：米
  Scalar dist_of_localization_to_front_;
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  Scalar dist_of_localization_to_rear_;
  // 车辆的定位点到中心点的距离，单位：米
  Scalar dist_of_localization_to_center_;

  // 最大方向盘转角
  Scalar max_steering_angle_;
  // 转向传动比
  Scalar steering_gear_ratio_;
  // 轴距
  Scalar wheelbase_;
  // 轴距的逆
  Scalar wheelbase_inverse_;
};


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_BASE_H_

