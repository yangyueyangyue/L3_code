//

#include "ft_auman/vehicle_model_ft_auman.h"

namespace phoenix {
namespace veh_model {


VehicleModelFtAuman::VehicleModelFtAuman() {
  Base::SetVehicleLength(7.0F);
  Base::SetVehicleWidth(2.5F);
  Base::SetDistOfLocalizationToFront(5.5F);
  Base::SetDistOfLocalizationToRear(1.5F);
  Base::SetDistOfLocalizationToCenter(2.0F);

  Base::SetMaxSteeringAngle(common::com_deg2rad(900.0F));
  Base::SetWheelbase(3.975F);

  steering_gear_ratio_ = 30.0F;//21.0F;

  sensor_.calibration_cam[0].yaw_offset = 0.0F;
  sensor_.calibration_cam[0].x_offset = 5.2F;
  sensor_.calibration_cam[0].y_offset = 0.0F;

  sensor_.calibration_radar[0].yaw_offset = common::com_deg2rad(2.0F);
  sensor_.calibration_radar[0].x_offset = 5.2F;
  sensor_.calibration_radar[0].y_offset = 0.26F;

#if 0
  sensor_.calibration_radar[1].yaw_offset = common::com_deg2rad(60.0F);
  sensor_.calibration_radar[1].x_offset = 5.2F;
  sensor_.calibration_radar[1].y_offset = 1.25F;

  sensor_.calibration_radar[2].yaw_offset = common::com_deg2rad(-60.0F);
  sensor_.calibration_radar[2].x_offset = 5.2F;
  sensor_.calibration_radar[2].y_offset = -1.25F;
#else
  sensor_.calibration_radar[1].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[1].x_offset = 0.0F;
  sensor_.calibration_radar[1].y_offset = 0.0F;

  sensor_.calibration_radar[2].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[2].x_offset = 0.0F;
  sensor_.calibration_radar[2].y_offset = 0.0F;
#endif

#if 0
  sensor_.calibration_radar[3].yaw_offset = common::com_deg2rad(120.0F);
  sensor_.calibration_radar[3].x_offset = 1.38F;
  sensor_.calibration_radar[3].y_offset = 1.25F;

  sensor_.calibration_radar[4].yaw_offset = common::com_deg2rad(-120.0F);
  sensor_.calibration_radar[4].x_offset = 1.38F;
  sensor_.calibration_radar[4].y_offset = -1.25F;
#else
  sensor_.calibration_radar[3].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[3].x_offset = 0.0F;
  sensor_.calibration_radar[3].y_offset = 0.0F;

  sensor_.calibration_radar[4].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[4].x_offset = 0.0F;
  sensor_.calibration_radar[4].y_offset = 0.0F;
#endif

  sensor_.calibration_radar[5].yaw_offset = common::com_deg2rad(66.0F);
  sensor_.calibration_radar[5].x_offset = 5.2F;
  sensor_.calibration_radar[5].y_offset = 1.25F;

  sensor_.calibration_radar[6].yaw_offset = common::com_deg2rad(-65.0F);
  sensor_.calibration_radar[6].x_offset = 5.2F;
  sensor_.calibration_radar[6].y_offset = -1.25F;

  sensor_.calibration_radar[7].yaw_offset = common::com_deg2rad(120.0F);
  sensor_.calibration_radar[7].x_offset = 1.38F;
  sensor_.calibration_radar[7].y_offset = 1.25F;

  sensor_.calibration_radar[8].yaw_offset = common::com_deg2rad(-120.0F);
  sensor_.calibration_radar[8].x_offset = 1.38F;
  sensor_.calibration_radar[8].y_offset = -1.25F;

  sensor_.calibration_radar[9].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[9].x_offset = -1.5F;
  sensor_.calibration_radar[9].y_offset = -0.0F;
}

VehicleModelFtAuman::~VehicleModelFtAuman() {
  // nothing to do
}


VehicleModelFtAuman::Scalar VehicleModelFtAuman::ClampMaxSteeringAngleByVelocity(
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


