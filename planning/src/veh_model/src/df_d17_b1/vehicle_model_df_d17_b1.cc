//

#include "df_d17_b1/vehicle_model_df_d17_b1.h"

namespace phoenix {
namespace veh_model {


VehicleModelDfD17B1::VehicleModelDfD17B1() {
  Base::SetVehicleLength(7.0F);
  Base::SetVehicleWidth(2.5F);
  Base::SetVehicleHeight(3.5F);

  Base::SetTrailerLength(13.0F);
  Base::SetTrailerWidth(2.5F);
  Base::SetTrailerHeight(3.5F);

  Base::SetDistOfLocalizationToFront(5.5F);
  Base::SetDistOfLocalizationToRear(1.5F);
  Base::SetDistOfLocalizationToCenter(2.0F);

  Base::SetMaxSteeringAngle(common::com_deg2rad(600.0F));
  Base::SetSteeringGearRatio(16.0F);
  Base::SetWheelbase(4.05F);
}

VehicleModelDfD17B1::~VehicleModelDfD17B1() {
  // nothing to do
}


VehicleModelDfD17B1::Scalar VehicleModelDfD17B1::ClampMaxSteeringAngleByVelocity(
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


