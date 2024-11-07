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

struct EnvironmentalInformation
{
  // 空气密度
  Float32_t air_density;
  // 风阻系数
  Float32_t cd;
  // 重力加速度
  Float32_t g;

  void Clear() {
    air_density = 0.0F;
    cd = 0.0F;
    g = 0.0F;
  }

  EnvironmentalInformation(){
    Clear();
  }

};

struct vehicle_fuel_comsun
{

  Float32_t L01;
  Float32_t L02;
  Float32_t L10;
  Float32_t L11;
  Float32_t L12;
  Float32_t L20;
  Float32_t L21;
  Float32_t L22;

  void Clear() {
    L01 = 0.0;
    L02 = 0.0;
    L10 = 0.0;
    L11 = 0.0;
    L12 = 0.0;
    L20 = 0.0;
    L21 = 0.0;
    L22 = 0.0;
  }

  vehicle_fuel_comsun(){
    Clear();
  }

};
// T.B.D


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_H_

