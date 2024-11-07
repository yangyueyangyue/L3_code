//

#include "vehicle_model_base.h"

namespace phoenix {
namespace veh_model {


VehicleModelBase::VehicleModelBase() {
  // 车长，单位：米
  vehicle_length_ = 5.0F;
  // 车宽，单位：米
  vehicle_width_ = 2.5F;
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  dist_of_localization_to_front_ = 2.0F;
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  dist_of_localization_to_rear_ = 0.5F;
  // 车辆的定位点到中心点的距离，单位：米
  dist_of_localization_to_center_ = 0.5F;

  max_steering_angle_ = common::com_deg2rad(500.0F);
  wheelbase_ = 2.67F;
  wheelbase_inverse_ = 1.0F / wheelbase_;
}

VehicleModelBase::~VehicleModelBase() {
  // nothing to do
}

VehicleModelBase::Scalar VehicleModelBase::ClampMaxSteeringAngleByVelocity(
    Scalar angle, Scalar v) const {
  Scalar limited_angle = angle;

  if (v > 80.0F/3.6F) {
    if (angle > common::com_deg2rad(30.0F)) {
      limited_angle = common::com_deg2rad(30.0F);
    } else if (angle < common::com_deg2rad(-30.0F)) {
      limited_angle = common::com_deg2rad(-30.0F);
    }
  } else if (v > 70.0F/3.6F) {
    if (angle > common::com_deg2rad(40.0F)) {
      limited_angle = common::com_deg2rad(40.0F);
    } else if (angle < common::com_deg2rad(-40.0F)) {
      limited_angle = common::com_deg2rad(-40.0F);
    }
  } else if (v > 60.0F/3.6F) {
    if (angle > common::com_deg2rad(50.0F)) {
      limited_angle = common::com_deg2rad(50.0F);
    } else if (angle < common::com_deg2rad(-50.0F)) {
      limited_angle = common::com_deg2rad(-50.0F);
    }
  } else if (v > 50.0F/3.6F) {
    if (angle > common::com_deg2rad(60.0F)) {
      limited_angle = common::com_deg2rad(60.0F);
    } else if (angle < common::com_deg2rad(-60.0F)) {
      limited_angle = common::com_deg2rad(-60.0F);
    }
  } else if (v > 40.0F/3.6F) {
    if (angle > common::com_deg2rad(70.0F)) {
      limited_angle = common::com_deg2rad(70.0F);
    } else if (angle < common::com_deg2rad(-70.0F)) {
      limited_angle = common::com_deg2rad(-70.0F);
    }
  }

  if (common::com_abs(angle - limited_angle) >
      common::NumLimits<Scalar>::epsilon()) {
    LOG_WARN << "The steering angle exceeds the angle "
                "limited by velocity factor, angle="
             << common::com_rad2deg(angle)
             << "deg, limited_angle=" << common::com_rad2deg(limited_angle)
             << "deg, v=" << v*3.6F << "km/h.";
  }

  return (limited_angle);
}

void VehicleModelBase::EstimateNextPos(
    const Scalar v, const Scalar yaw_rate, const Scalar dt,
    const Scalar pos[3], Scalar next_pos[3]) const {

  if (common::com_abs(yaw_rate) < 0.001F) {
    next_pos[0] = pos[0] + v * common::com_cos(pos[2]) * dt;
    next_pos[1] = pos[1] + v * common::com_sin(pos[2]) * dt;
    next_pos[2] = phoenix::common::NormalizeAngle(pos[2] + yaw_rate * dt);
  } else {
    Scalar r = v / yaw_rate;
    Scalar heading_changed = common::NormalizeAngle(pos[2] + yaw_rate * dt);
    next_pos[0] = pos[0] +
        r * (-common::com_sin(pos[2]) + common::com_sin(heading_changed));
    next_pos[1] = pos[1] +
        r * ( common::com_cos(pos[2]) - common::com_cos(heading_changed));
    next_pos[2] = heading_changed;
  }
}


}  // namespace veh_model
}  // namespace phoenix


