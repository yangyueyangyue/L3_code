/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       work_lateral_control.h
 * @brief      横向控制
 * @details    封装了横向控制功能的接口
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

#ifndef PHOENIX_FRAMEWORK_WORK_LATERAL_CONTROL_C_H_
#define PHOENIX_FRAMEWORK_WORK_LATERAL_CONTROL_C_H_

#include "ad_msg_c.h"
#include "lateral_control_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct WorkLatCtlInstance_t
 * @brief 保存横向控制所需要的成员变量.
 */
typedef struct _WorkLatCtlInstance_t WorkLatCtlInstance_t;
struct _WorkLatCtlInstance_t {
  /// 惯性测量单元的测量值
  Imu_t imu;
  /// 车身状态信息
  Chassis_t chassis;
  /// 规划的结果
  PlanningResult_t planning_result;

  /// 相对于目标轨迹的车辆所在位置
  RelativePos_t curr_pos;
  /// 横向控制需要的信息
  LatCtlDataSource_t lat_ctl_data_src;

  /// 目标角速度
  Float32_t tar_yaw_rate;
  /// 目标方向盘转角
  Float32_t tar_steering_wheel_angle;
  /// 目标方向盘转速
  Float32_t tar_steering_wheel_speed;

  /// 横向控制信息
  LateralControlInfo_t lat_ctl_info;
};


/**
 * @brief 初始化横向控制功能
 * @param[in] instance 成员变量
 */
void Phoenix_Work_LateralCtl_Initialize(WorkLatCtlInstance_t* const instance);

/**
 * @brief 开始一次横向控制功能（定周期调用）
 * @param[in] instance 成员变量
 */
Int32_t Phoenix_Work_LateralCtl_DoWork(WorkLatCtlInstance_t* const instance);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_FRAMEWORK_WORK_LATERAL_CONTROL_C_H_

