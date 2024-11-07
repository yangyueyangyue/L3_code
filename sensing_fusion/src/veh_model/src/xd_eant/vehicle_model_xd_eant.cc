//

#include "xd_eant/vehicle_model_xd_eant.h"

namespace phoenix {
namespace veh_model {


VehicleModelXdEAnt::VehicleModelXdEAnt() {
  Base::SetVehicleLength(4.68F);
  Base::SetVehicleWidth(1.72F);
  Base::SetDistOfLocalizationToFront(3.68F);
  Base::SetDistOfLocalizationToRear(1.0F);
  Base::SetDistOfLocalizationToCenter(1.34F);

  Base::SetMaxSteeringAngle(common::com_deg2rad(500.0F));
  Base::SetWheelbase(2.15F);

  steering_gear_ratio_ = 18.0F;

  sensor_.calibration_cam[0].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_cam[0].x_offset = 2.68F;
  sensor_.calibration_cam[0].y_offset = 0.0F;

  sensor_.calibration_radar[0].yaw_offset = common::com_deg2rad(5.0F);
  sensor_.calibration_radar[0].x_offset = 3.68F;
  sensor_.calibration_radar[0].y_offset = 0.2F;
}

VehicleModelXdEAnt::~VehicleModelXdEAnt() {
  // nothing to do
}

VehicleModelXdEAnt::Scalar VehicleModelXdEAnt::ClampMaxSteeringAngleByVelocity(
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



}  // namespace veh_model
}  // namespace phoenix


