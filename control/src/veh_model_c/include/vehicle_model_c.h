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
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_C_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_C_H_


#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


Float32_t Phoenix_VehModel_GetMaxSteeringAngle();

Float32_t Phoenix_VehModel_GetWheelbase();

Float32_t Phoenix_VehModel_ClampMaxSteeringAngle(Float32_t angle);

Float32_t Phoenix_VehModel_ClampMaxSteeringAngleByVelocity(
    Float32_t angle, Float32_t v);

Float32_t Phoenix_VehModel_CalcYawRateFromSteeringAngle(
    Float32_t angle, Float32_t v);

Float32_t Phoenix_VehModel_CalcSteeringAngleFromYawRate(
    Float32_t yaw_rate, Float32_t v);

Float32_t Phoenix_VehModel_CalcCurvatureFromSteeringAngle(
    Float32_t angle);

Float32_t Phoenix_VehModel_CalcSteeringAngleFromCurvature(
    Float32_t curvature);

void Phoenix_VehModel_EstimateNextPos(
    const Float32_t v, const Float32_t yaw_rate, const Float32_t dt,
    const Float32_t pos[3], Float32_t next_pos[3]);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_C_H_

