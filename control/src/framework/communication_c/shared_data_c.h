/******************************************************************************
 ** 共享数据存储
 ******************************************************************************
 *
 *  共享数据存储
 *
 *  @file       shared_data.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_SHARED_DATA_C_H_
#define PHOENIX_FRAMEWORK_SHARED_DATA_C_H_

#include "ad_msg_c.h"
#include "lateral_control_c.h"
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
#include "vehicle_control_c.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


void Phoenix_SharedData_Initialize();

// IMU
void Phoenix_SharedData_SetImu(const Imu_t* data);
void Phoenix_SharedData_GetImu(Imu_t* const data);

// Planning result
void Phoenix_SharedData_SetPlanningResult(const PlanningResult_t* data);
void Phoenix_SharedData_GetPlanningResult(PlanningResult_t* const data);

// Chassis information
void Phoenix_SharedData_SetChassis(const Chassis_t* data);
void Phoenix_SharedData_GetChassis(Chassis_t* const data);

// Chassis control command
void Phoenix_SharedData_SetChassisCtlCmd(const ChassisCtlCmd_t* data);
void Phoenix_SharedData_GetChassisCtlCmd(ChassisCtlCmd_t* const data);

// Special Chassis information
void Phoenix_SharedData_SetSpecialChassisInfo(const SpecialChassisInfo_t* data);
void Phoenix_SharedData_GetSpecialChassisInfo(SpecialChassisInfo_t* const data);

void Phoenix_SharedData_SetChassisDfD17adrecordInfo(const ChassisDfD17_adrecord* data);
void Phoenix_SharedData_GetChassisDfD17adrecordInfo(ChassisDfD17_adrecord* const data);

void Phoenix_SharedData_SetChassisCtlStatus(Int32_t data);
void Phoenix_SharedData_GetChassisCtlStatus(Int32_t* const data);

void Phoenix_SharedData_SetLateralControlInfo(const LateralControlInfo_t* data);
void Phoenix_SharedData_GetLateralControlInfo(LateralControlInfo_t* const data);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
void Phoenix_SharedData_SetLongtitudeControlInfo(const LongtitudeControlInfo_t* data);
void Phoenix_SharedData_GetLongtitudeControlInfo(LongtitudeControlInfo_t* const data);
#endif

void Phoenix_SharedData_SetSteeringControlInfo(const SteeringControlInfo_t* data);
void Phoenix_SharedData_GetSteeringControlInfo(SteeringControlInfo_t* const data);

// Module status
void Phoenix_SharedData_SetModuleStatusList(const ModuleStatusList_t* data);
void Phoenix_SharedData_GetModuleStatusList(ModuleStatusList_t* const data);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
//DFCV new message
void Phoenix_SharedData_SetDFCVSpecialChassisInfo(const DFCV_SpecialChassisInfo_t* data);
void Phoenix_SharedData_GetDFCVSpecialChassisInfo(DFCV_SpecialChassisInfo_t* const data);
#endif

#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_FRAMEWORK_SHARED_DATA_C_H_

