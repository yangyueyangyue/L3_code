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
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_DF_D17_B1_C_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_DF_D17_B1_C_H_


#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


Float32_t Phoenix_VehModel_DfD17B1_GetMaxSteeringAngle();

Float32_t Phoenix_VehModel_DfD17B1_GetWheelbase();

Float32_t Phoenix_VehModel_DfD17B1_ClampMaxSteeringAngle(Float32_t angle);

Float32_t Phoenix_VehModel_DfD17B1_CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v);

Float32_t Phoenix_VehModel_DfD17B1_CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v);

Float32_t Phoenix_VehModel_DfD17B1_CalcCurvatureFromSteeringAngle(
    Float32_t angle);

Float32_t Phoenix_VehModel_DfD17B1_CalcSteeringAngleFromCurvature(
    Float32_t curvature);

Float32_t Phoenix_VehModel_DfD17B1_ClampMaxSteeringAngleByVelocity(
    Float32_t angle, Float32_t v);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_DF_D17_B1_C_H_

