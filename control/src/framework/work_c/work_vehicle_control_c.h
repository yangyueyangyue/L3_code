/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       work_vehicle_control.h
 * @brief      车身控制
 * @details    封装了车身控制功能的接口
 *
 * @author     pengc
 * @date       2021.07.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_FRAMEWORK_WORK_VEHICLE_CONTROL_C_H_
#define PHOENIX_FRAMEWORK_WORK_VEHICLE_CONTROL_C_H_

#include "os/mutex_c.h"
#include "ad_msg_c.h"
#include "lateral_control_c.h"
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
#include "vehicle_control_c.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct WorkVehCtlInstance_t
 * @brief 保存车身控制所需要的成员变量.
 */
typedef struct _WorkVehCtlInstance_t WorkVehCtlInstance_t;
struct _WorkVehCtlInstance_t {
  /// 车身控制请求
  ChassisCtlCmd_t veh_ctl_request;

  /// 当前车身的状态
  Chassis_t chassis;

  SpecialChassisInfo_t special_chassis;
  /// 规划的结果
  PlanningResult_t planning_result;
  /// 各个模块的状态
  ModuleStatusList_t module_status_list;

  /// 车身控制指令
  ChassisCtlCmd_t chassis_ctl_cmd;
  
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
  LongtitudeControlInfo_t lon_ctl_info;
#endif

  /// 互斥锁，用于多任务访问
  CommonMutex_t lock_veh_ctl_info;
};


/**
 * @brief 初始化车身控制功能
 * @param[in] instance 成员变量
 */
void Phoenix_Work_VehicleCtl_Initialize(WorkVehCtlInstance_t* const instance);


/**
 * @brief 请求开启车身控制功能
 * @param[in] instance 成员变量
 */
void Phoenix_Work_VehicleCtl_StartRobotCtl(
    WorkVehCtlInstance_t* const instance);

/**
 * @brief 请求关闭车身控制功能
 * @param[in] instance 成员变量
 */
void Phoenix_Work_VehicleCtl_StopRobotCtl(
    WorkVehCtlInstance_t* const instance);

/**
 * @brief 允许/拒绝远程控制
 * @param[in] instance 成员变量
 * @param[in] enable true ~ 允许, false ~ 拒绝
 */
void Phoenix_Work_VehicleCtl_EnableRemoteCtl(
    WorkVehCtlInstance_t* const instance, Int8_t enable);

/**
 * @brief 请求开启/关闭直接控制车身功能
 * @param[in] instance 成员变量
 * @param[in] enable true ~ 开启, false ~ 关闭
 */
void Phoenix_Work_VehicleCtl_EnableDirectCtl(
    WorkVehCtlInstance_t* const instance, Int8_t enable);

/**
 * @brief 请求开启/关闭转向系统控制功能
 * @param[in] instance 成员变量
 * @param[in] enable true ~ 开启, false ~ 关闭
 */
void Phoenix_Work_VehicleCtl_EnableEps(
    WorkVehCtlInstance_t* const instance, Int8_t enable);

/**
 * @brief 请求开启/关闭加速系统控制功能
 * @param[in] instance 成员变量
 * @param[in] enable true ~ 开启, false ~ 关闭
 */
void Phoenix_Work_VehicleCtl_EnableThrottleSys(
    WorkVehCtlInstance_t* const instance, Int8_t enable);

/**
 * @brief 请求开启/关闭制动系统控制功能
 * @param[in] instance 成员变量
 * @param[in] enable true ~ 开启, false ~ 关闭
 */
void Phoenix_Work_VehicleCtl_EnableEbs(
    WorkVehCtlInstance_t* const instance, Int8_t enable);


/**
 * @brief 请求方向盘角度控制
 * @param[in] instance 成员变量
 * @param[in] angle 目标方向盘角度(rad)
 * @param[in] speed 目标方向盘转速(rad/s)
 */
void Phoenix_Work_VehicleCtl_TurnSteeringWheel(
    WorkVehCtlInstance_t* const instance,
    Float32_t angle, Float32_t speed);

void Phoenix_Work_VehicleCtl_TurnSteeringWheelDirectly(
    WorkVehCtlInstance_t* const instance,
    Float32_t angle, Float32_t speed);

/**
 * @brief 请求速度控制
 * @param[in] instance 成员变量
 * @param[in] spd 目标车速(m/s)
 * @param[in] acc 目标加速度(m/s^2)
 */
void Phoenix_Work_VehicleCtl_SpeedUp(
    WorkVehCtlInstance_t* const instance, Float32_t spd, Float32_t acc);

/**
 * @brief 请求加速(加油门)
 * @param[in] instance 成员变量
 * @param[in] value 加速量(油门量，0 ~ 100%)
 */
void Phoenix_Work_VehicleCtl_Accelerate(
    WorkVehCtlInstance_t* const instance, Float32_t value);

/**
 * @brief 请求制动(踩刹车)
 * @param[in] instance 成员变量
 * @param[in] value 制动量(0 ~ 100%)
 */
void Phoenix_Work_VehicleCtl_Brake(
    WorkVehCtlInstance_t* const instance, Float32_t value);

/**
 * @brief 请求变档
 * @param[in] instance 成员变量
 * @param[in] value 目标档位(P, N, R, D)
 */
void Phoenix_Work_VehicleCtl_ChangeGear(
    WorkVehCtlInstance_t* const instance, Int8_t value);

/**
 * @brief 请求转向灯控制
 * @param[in] instance 成员变量
 * @param[in] value 目标档位(OFF, LEFT, RIGHT, EMERGENCY)
 */
void Phoenix_Work_VehicleCtl_SetTurnLamp(
    WorkVehCtlInstance_t* const instance, Int8_t value);

/**
 * @brief 请求雨刮器控制
 * @param[in] instance 成员变量
 * @param[in] value 目标状态
 */
void Phoenix_Work_VehicleCtl_SetWiper(
    WorkVehCtlInstance_t* const instance, Int8_t value);


/**
 * @brief 开始一次车身控制功能（定周期调用）
 * @param[in] instance 成员变量
 */
Int32_t Phoenix_Work_VehicleCtl_DoWork(WorkVehCtlInstance_t* const instance);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_FRAMEWORK_WORK_VEHICLE_CONTROL_C_H_

