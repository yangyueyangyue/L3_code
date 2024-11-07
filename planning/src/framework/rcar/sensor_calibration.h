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
#ifndef PHOENIX_FRAMEWORK_SENSOR_CALINRATIOMN_H_
#define PHOENIX_FRAMEWORK_SENSOR_CALINRATIOMN_H_

#include "utils/macros.h"
#include "math/math_utils.h"


namespace phoenix {
namespace framework {


#define MAX_CAM_NUM 2
#define MAX_RADAR_NUM 10
#define MAX_LIDAR_NUM 3


class Calibration{
public:
    Calibration(){
        Clear();
    }

    ~Calibration(){};
    void Clear();
public:
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
};

class  SensorOnVehicle {
public:
  SensorOnVehicle() {
    Clear();
  }
  ~SensorOnVehicle(){};
  void Clear();
  void SensorOnVehicleInit();
public:
  Calibration calibration_cam[MAX_CAM_NUM];
  Calibration calibration_radar[MAX_RADAR_NUM];
  Calibration calibration_lidar[MAX_LIDAR_NUM];
};


}  // namespace framework
}  // namespace phoenix
#endif// PHOENIX_FRAMEWORK_SENSOR_CALINRATIOMN_H_

