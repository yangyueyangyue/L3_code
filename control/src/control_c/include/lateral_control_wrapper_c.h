/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lateral_control_wrapper.h
 * @brief      横向控制器接口
 * @details    定义横向控制器接口
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

#ifndef PHOENIX_CONTROL_LATERAL_CONTROL_WRAPPER_C_H_
#define PHOENIX_CONTROL_LATERAL_CONTROL_WRAPPER_C_H_

#include "utils/macros.h"
#include "lateral_control_c.h"
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
#include "ad_msg_c.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化横向控制器
 */
void Phoenix_LatCtl_Initialize();

/**
 * @brief 重置横向控制器内部状态
 */
void Phoenix_LatCtl_ResetLatCtlValue(
    Float32_t tar_steering_angle);

/**
 * @brief 配置横向控制器(PID)的参数
 * @param[in] conf PID控制器的配置参数
 */
void Phoenix_LatCtl_ConfigPIDController(
    const LatCtlPidConf_t* conf);

/**
 * @brief 计算横向控制量
 * @param[in] data_source 控制器需要的数据
 * @param[out] tar_yaw_rate 目标角速度
 * @param[out] ctl_value 目标方向盘转角
 * @param[out] status_info 内部状态信息
 * @return 0 ~ 成功, others ~ 失败
 */
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
Int32_t Phoenix_LatCtl_CalcLatCtlValue(
    const LatCtlDataSource_t* data_source,
    Float32_t* tar_yaw_rate, Float32_t* ctl_value,
    LateralControlInfo_t* const status_info);
#endif

#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
/**
 * @brief 计算横向控制量
 * @param[in] dfcvctrl_src_chassis 横向控制器需要的车辆底盘数据
 * @param[in] dfcv_special_chassis 横向控制器需要的车辆底盘数据
 * @param[in] planning_result 横向控制器需要的规划结果
 * @param[in] imu 横向控制器需要的imu数据
 * @param[in] CtrlStatus 底盘工作状态
 * @param[out] tar_yaw_rate 目标角速度
 * @param[out] ctl_value 目标方向盘转角
 * @param[out] status_info 内部状态信息
 * @return 0 ~ 成功, others ~ 失败
 */
Int32_t DFCV_LatCtl_CalcLatCtlValue(Chassis_t* dfcvctrl_src_chassis, 
    DFCV_SpecialChassisInfo_t* dfcv_special_chassis,
    PlanningResult_t* planning_result,    Imu_t * imu, Int32_t* CtrlStatus, 
    Float32_t* tar_yaw_rate, Float32_t* ctl_value,    LateralControlInfo_t* const status_info);
#endif

#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_LATERAL_CONTROL_WRAPPER_C_H_
