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
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"


namespace phoenix {
namespace veh_model {


struct SensorOnVehicle {
  enum { MAX_CAM_NUM = 2 };
  enum { MAX_RADAR_NUM = 10 };
  enum { MAX_LIDAR_NUM = 3 };

  struct Calibration {
    // 偏航角
    Float32_t yaw_offset;
    // 俯仰角
    Float32_t pitch_offset;
    // 横滚角
    Float32_t roll_offset;
    // x 偏移
    Float32_t x_offset;
    // y 偏移
    Float32_t y_offset;
    // z 偏移
    Float32_t z_offset;

    void Clear() {
      yaw_offset = 0.0F;
      pitch_offset = 0.0F;
      roll_offset = 0.0F;
      x_offset = 0.0F;
      y_offset = 0.0F;
    }

    Calibration() {
      Clear();
    }
  };

  Calibration calibration_cam[MAX_CAM_NUM];
  Calibration calibration_radar[MAX_RADAR_NUM];
  Calibration calibration_lidar[MAX_LIDAR_NUM];

  void Clear() {
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

  SensorOnVehicle() {
    Clear();
  }
};


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_H_

