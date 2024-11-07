/******************************************************************************
 ** 传感器标定参数信息
 ******************************************************************************
 *
 *  传感器标定参数信息
 *
 *  @file       vehicle_model.h
 *
 *  @author     yaoyangz
 *  @date       2022.06.09
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "sensor_calibration.h"


namespace phoenix {
namespace framework {
  
void Calibration::Clear(){
  yaw_offset = 0.0F;
  pitch_offset = 0.0F;
  roll_offset = 0.0F;
  x_offset = 0.0F;
  y_offset = 0.0F;
}

void SensorOnVehicle::Clear() {
  for (Int32_t i = 0; i < MAX_CAM_NUM; ++i) {
    calibration_cam[i].Clear();
  }
  for (Int32_t i = 0; i < MAX_RADAR_NUM; ++i) {
    calibration_radar[i].Clear();
  }
  for (Int32_t i = 0; i < MAX_LIDAR_NUM; ++i) {
    calibration_lidar[i].Clear();
  }
}

void SensorOnVehicle::SensorOnVehicleInit(){
  // 前视一体机标定参数
  calibration_cam[0].yaw_offset = 0.0F;
  calibration_cam[0].x_offset = 0.0F;
  calibration_cam[0].y_offset = 0.0F;

  // 环视相机标定参数
  calibration_cam[1].yaw_offset = 0.0F;
  calibration_cam[1].x_offset = 0.0F;
  calibration_cam[1].y_offset = 0.0F;

  // 前向毫米波雷达标定参数
  calibration_radar[0].yaw_offset = phoenix::common::com_deg2rad(0.0F);
  calibration_radar[0].x_offset = 0.0F;
  calibration_radar[0].y_offset = 0.0F;

  #if 0
  calibration_radar[1].yaw_offset = phoenix::common::com_deg2rad(60.0F);
  calibration_radar[1].x_offset = 5.2F;
  calibration_radar[1].y_offset = 1.25F;

  calibration_radar[2].yaw_offset = phoenix::common::com_deg2rad(-60.0F);
  calibration_radar[2].x_offset = 5.2F;
  calibration_radar[2].y_offset = -1.25F;
  #else
  // 侧向毫米波雷达标定参数
  calibration_radar[1].yaw_offset = phoenix::common::com_deg2rad(0.0F);
  calibration_radar[1].x_offset = 0.0F;
  calibration_radar[1].y_offset = 0.0F;

  calibration_radar[2].yaw_offset = phoenix::common::com_deg2rad(0.0F);
  calibration_radar[2].x_offset = 0.0F;
  calibration_radar[2].y_offset = 0.0F;
  #endif

  #if 0
  calibration_radar[3].yaw_offset = phoenix::common::com_deg2rad(120.0F);
  calibration_radar[3].x_offset = 1.38F;
  calibration_radar[3].y_offset = 1.25F;

  calibration_radar[4].yaw_offset = phoenix::common::com_deg2rad(-120.0F);
  calibration_radar[4].x_offset = 1.38F;
  calibration_radar[4].y_offset = -1.25F;
  #else
  calibration_radar[3].yaw_offset = phoenix::common::com_deg2rad(0.0F);
  calibration_radar[3].x_offset = 0.0F;
  calibration_radar[3].y_offset = 0.0F;

  calibration_radar[4].yaw_offset = phoenix::common::com_deg2rad(0.0F);
  calibration_radar[4].x_offset = 0.0F;
  calibration_radar[4].y_offset = 0.0F;
  #endif

  calibration_radar[5].yaw_offset = phoenix::common::com_deg2rad(66.0F);
  calibration_radar[5].x_offset = 5.2F;
  calibration_radar[5].y_offset = 1.25F;

  calibration_radar[6].yaw_offset = phoenix::common::com_deg2rad(-65.0F);
  calibration_radar[6].x_offset = 5.2F;
  calibration_radar[6].y_offset = -1.25F;

  calibration_radar[7].yaw_offset = phoenix::common::com_deg2rad(120.0F);
  calibration_radar[7].x_offset = 1.38F;
  calibration_radar[7].y_offset = 1.25F;

  calibration_radar[8].yaw_offset = phoenix::common::com_deg2rad(-120.0F);
  calibration_radar[8].x_offset = 1.38F;
  calibration_radar[8].y_offset = -1.25F;

  calibration_radar[9].yaw_offset = phoenix::common::com_deg2rad(0.0F);
  calibration_radar[9].x_offset = -1.5F;
  calibration_radar[9].y_offset = -0.0F;
}


}  // namespace framework
}  // namespace phoenix

