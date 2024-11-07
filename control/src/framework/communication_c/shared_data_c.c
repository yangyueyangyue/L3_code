/******************************************************************************
 ** 共享数据存储
 ******************************************************************************
 *
 *  共享数据存储
 *
 *  @file       shared_data.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication_c/shared_data_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "math/math_utils_c.h"
#include "os/mutex_c.h"


static Imu_t s_imu_info;
static CommonReadWriteMutex_t s_lock_imu_info;

static PlanningResult_t s_planning_result;
static CommonReadWriteMutex_t s_lock_planning_result;

static Chassis_t s_chassis_info;
static CommonReadWriteMutex_t s_lock_chassis_info;

static ChassisDfD17_adrecord s_chassis_adrecord_info;
static CommonReadWriteMutex_t s_lock_chassis_adrecord_info;

static ChassisCtlCmd_t s_chassis_ctl_cmd;
static CommonReadWriteMutex_t s_lock_chassis_ctl_cmd;

static SpecialChassisInfo_t s_special_chassis_info;
static CommonReadWriteMutex_t s_lock_special_chassis_info;

#if ((CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV) || (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV))
static DFCV_SpecialChassisInfo_t s_DFCV_special_chassis_info;
static CommonReadWriteMutex_t s_lock_DFCV_special_chassis_info;
#endif

static Int32_t s_chassis_ctl_status = 0;
static CommonReadWriteMutex_t s_lock_chassis_ctl_status;

static LateralControlInfo_t s_lat_ctl_info;
static CommonReadWriteMutex_t s_lock_lat_ctl_info;

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
static LongtitudeControlInfo_t s_lon_ctl_info;
static CommonReadWriteMutex_t s_lock_lon_ctl_info;
#endif

static SteeringControlInfo_t s_steering_ctl_info;
static CommonReadWriteMutex_t s_lock_s_steering_ctl_info;

// Module status
static ModuleStatusList_t s_module_status_list;
static CommonReadWriteMutex_t s_lock_module_status_list;


void Phoenix_SharedData_Initialize() {
  Phoenix_AdMsg_ClearImu(&s_imu_info);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_imu_info);

  Phoenix_AdMsg_ClearPlanningResult(&s_planning_result);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_planning_result);

  Phoenix_AdMsg_ClearChassis(&s_chassis_info);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_chassis_info);

  Phoenix_AdMsg_ClearChassisCtlCmd(&s_chassis_ctl_cmd);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_chassis_ctl_cmd);

  Phoenix_AdMsg_ClearSpecialChassisInfo(&s_special_chassis_info);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_special_chassis_info);

  Phoenix_AdMsg_ClearChassisDfD17adrecord(&s_chassis_adrecord_info);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_chassis_adrecord_info);
  
  s_chassis_ctl_status = VEH_CHASSIS_CTL_STATUS_MANUAL;
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_chassis_ctl_status);

  phoenix_com_memset(&s_lat_ctl_info, 0, sizeof(s_lat_ctl_info));
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_lat_ctl_info);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  phoenix_com_memset(&s_lon_ctl_info, 0, sizeof(s_lon_ctl_info));
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_lon_ctl_info);
#endif

  phoenix_com_memset(&s_steering_ctl_info, 0, sizeof(s_steering_ctl_info));
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_s_steering_ctl_info);

  phoenix_com_memset(&s_module_status_list, 0, sizeof(s_module_status_list));
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_module_status_list);

#if ((CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV) || (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV))
  Phoenix_AdMsg_ClearDFCVSpecialChassisInfo(&s_DFCV_special_chassis_info);
  Phoenix_Common_Os_ReadWriteMutex_Init(&s_lock_DFCV_special_chassis_info);
#endif
}


// IMU
void Phoenix_SharedData_SetImu(const Imu_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_imu_info);

  phoenix_com_memcpy(&s_imu_info, data, sizeof(Imu_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_imu_info);
}

void Phoenix_SharedData_GetImu(Imu_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_imu_info);

  phoenix_com_memcpy(data, &s_imu_info, sizeof(Imu_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_imu_info);
}

// Planning result
void Phoenix_SharedData_SetPlanningResult(const PlanningResult_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_planning_result);

  phoenix_com_memcpy(&s_planning_result, data, sizeof(PlanningResult_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_planning_result);
}

void Phoenix_SharedData_GetPlanningResult(PlanningResult_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_planning_result);

  phoenix_com_memcpy(data, &s_planning_result, sizeof(PlanningResult_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_planning_result);
}


// Chassis information
void Phoenix_SharedData_SetChassis(const Chassis_t* data) {
  Float32_t prev_yaw_rate = s_chassis_info.yaw_rate;
  Float32_t delta_yaw_rate = data->yaw_rate - prev_yaw_rate;

  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_chassis_info);

  phoenix_com_memcpy(&s_chassis_info, data, sizeof(Chassis_t));

  if ((phoenix_com_abs_f(delta_yaw_rate) > phoenix_com_deg2rad_f(100.0F)) ||
      (phoenix_com_abs_f(data->yaw_rate) > phoenix_com_deg2rad_f(100.0F))) {
    LOG_ERR_C("[CTL][SharedData] Detected unexpected value of "
              "yaw_rate(%f deg/s), prev=%f deg/s.",
              phoenix_com_rad2deg_f(data->yaw_rate),
              phoenix_com_rad2deg_f(prev_yaw_rate));
    s_chassis_info.yaw_rate = prev_yaw_rate;
  }

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_info);
}

void Phoenix_SharedData_GetChassis(Chassis_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_chassis_info);

  phoenix_com_memcpy(data, &s_chassis_info, sizeof(Chassis_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_info);
}


// Chassis control command
void Phoenix_SharedData_SetChassisCtlCmd(const ChassisCtlCmd_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_chassis_ctl_cmd);

  phoenix_com_memcpy(&s_chassis_ctl_cmd, data, sizeof(ChassisCtlCmd_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_ctl_cmd);
}

void Phoenix_SharedData_GetChassisCtlCmd(ChassisCtlCmd_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_chassis_ctl_cmd);

  phoenix_com_memcpy(data, &s_chassis_ctl_cmd, sizeof(ChassisCtlCmd_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_ctl_cmd);
}


void Phoenix_SharedData_SetSpecialChassisInfo(const SpecialChassisInfo_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_special_chassis_info);

  phoenix_com_memcpy(&s_special_chassis_info, data, sizeof(SpecialChassisInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_special_chassis_info);
}

void Phoenix_SharedData_GetSpecialChassisInfo(SpecialChassisInfo_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_special_chassis_info);

  phoenix_com_memcpy(data, &s_special_chassis_info, sizeof(SpecialChassisInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_special_chassis_info);
}

void Phoenix_SharedData_SetChassisDfD17adrecordInfo(const ChassisDfD17_adrecord* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_chassis_adrecord_info);

  phoenix_com_memcpy(&s_chassis_adrecord_info, data, sizeof(ChassisDfD17_adrecord));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_adrecord_info);
}

void Phoenix_SharedData_GetChassisDfD17adrecordInfo(ChassisDfD17_adrecord* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_chassis_adrecord_info);

  phoenix_com_memcpy(data, &s_chassis_adrecord_info, sizeof(ChassisDfD17_adrecord));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_adrecord_info);
}




void Phoenix_SharedData_SetChassisCtlStatus(Int32_t data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_chassis_ctl_status);

  s_chassis_ctl_status = data;

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_ctl_status);
}

void Phoenix_SharedData_GetChassisCtlStatus(Int32_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_chassis_ctl_status);

  *data = s_chassis_ctl_status;

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_chassis_ctl_status);
}


void Phoenix_SharedData_SetLateralControlInfo(
    const LateralControlInfo_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_lat_ctl_info);

  phoenix_com_memcpy(&s_lat_ctl_info, data, sizeof(LateralControlInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_lat_ctl_info);
}

void Phoenix_SharedData_GetLateralControlInfo(
    LateralControlInfo_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_lat_ctl_info);

  phoenix_com_memcpy(data, &s_lat_ctl_info, sizeof(LateralControlInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_lat_ctl_info);
}

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
// Add ShereData By ZQ
void Phoenix_SharedData_SetLongtitudeControlInfo(const LongtitudeControlInfo_t* data){
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_lon_ctl_info);

  phoenix_com_memcpy(&s_lon_ctl_info, data, sizeof(LongtitudeControlInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_lon_ctl_info);
}

void Phoenix_SharedData_GetLongtitudeControlInfo(LongtitudeControlInfo_t* const data){
    // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_lon_ctl_info);

  phoenix_com_memcpy(data, &s_lon_ctl_info, sizeof(LongtitudeControlInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_lon_ctl_info);
}
#endif

void Phoenix_SharedData_SetSteeringControlInfo(
    const SteeringControlInfo_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_s_steering_ctl_info);

  phoenix_com_memcpy(&s_steering_ctl_info, data, sizeof(SteeringControlInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_s_steering_ctl_info);
}

void Phoenix_SharedData_GetSteeringControlInfo(
    SteeringControlInfo_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_s_steering_ctl_info);

  phoenix_com_memcpy(data, &s_steering_ctl_info, sizeof(SteeringControlInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_s_steering_ctl_info);
}


void Phoenix_SharedData_SetModuleStatusList(const ModuleStatusList_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_module_status_list);

  phoenix_com_memcpy(&s_module_status_list, data, sizeof(ModuleStatusList_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_module_status_list);
}

void Phoenix_SharedData_GetModuleStatusList(ModuleStatusList_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_module_status_list);

  phoenix_com_memcpy(data, &s_module_status_list, sizeof(ModuleStatusList_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_module_status_list);
}

#if ((CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV) || (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV))
void Phoenix_SharedData_SetDFCVSpecialChassisInfo(const DFCV_SpecialChassisInfo_t* data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockWrite(&s_lock_DFCV_special_chassis_info);

  phoenix_com_memcpy(&s_DFCV_special_chassis_info, data, sizeof(DFCV_SpecialChassisInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_DFCV_special_chassis_info);
}

void Phoenix_SharedData_GetDFCVSpecialChassisInfo(DFCV_SpecialChassisInfo_t* const data) {
  // Lock
  Phoenix_Common_Os_ReadWriteMutex_LockRead(&s_lock_DFCV_special_chassis_info);

  phoenix_com_memcpy(data, &s_DFCV_special_chassis_info, sizeof(DFCV_SpecialChassisInfo_t));

  // Unlock
  Phoenix_Common_Os_ReadWriteMutex_Unlock(&s_lock_DFCV_special_chassis_info);
}
#endif

