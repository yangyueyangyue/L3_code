//

#include "df_d17/vehicle_model_df_d17.h"

namespace phoenix {
namespace veh_model {


VehicleModelDfD17::VehicleModelDfD17() {
  Base::SetVehicleLength(7.0F);
  Base::SetVehicleWidth(2.5F);
  Base::SetDistOfLocalizationToFront(5.5F);
  Base::SetDistOfLocalizationToRear(1.5F);
  Base::SetDistOfLocalizationToCenter(2.0F);

  Base::SetMaxSteeringAngle(common::com_deg2rad(600.0F));
  Base::SetWheelbase(4.05F);

  steering_gear_ratio_ = 16.0F;

  // 前视一体机标定参数
  sensor_.calibration_cam[0].yaw_offset = 0.0F;
  sensor_.calibration_cam[0].x_offset = 0.0F;
  //sensor_.calibration_cam[0].x_offset = 1.48F;
  sensor_.calibration_cam[0].y_offset = 0.0F;

  // 环视相机标定参数
  sensor_.calibration_cam[1].yaw_offset = 0.0F;
  sensor_.calibration_cam[1].x_offset = 0.0F;
  sensor_.calibration_cam[1].y_offset = 0.0F;

  // 前毫米波雷达标定参数
  sensor_.calibration_radar[0].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[0].x_offset = 0.0F;
  //sensor_.calibration_radar[0].x_offset = 1.48F;
  sensor_.calibration_radar[0].y_offset = 0.0F;

  // 侧向毫米波雷达标定参数
  sensor_.calibration_radar[1].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_radar[1].x_offset = 0.0F;
  sensor_.calibration_radar[1].y_offset = 0.0F;

  // 激光雷达标定参数
  sensor_.calibration_lidar[0].yaw_offset = common::com_deg2rad(0.0F);
  sensor_.calibration_lidar[0].x_offset = 0.0F;
  sensor_.calibration_lidar[0].y_offset = 0.0F;

}

VehicleModelDfD17::~VehicleModelDfD17() {
  // nothing to do
}


VehicleModelDfD17::Scalar VehicleModelDfD17::ClampMaxSteeringAngleByVelocity(
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


