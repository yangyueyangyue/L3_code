/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       longitudinal_control.h
 * @brief      数据类型定义
 * @details    车身控制所需的数据类型定义
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

#ifndef PHOENIX_CONTROL_VEHICLE_CONTROL_C_H_
#define PHOENIX_CONTROL_VEHICLE_CONTROL_C_H_

#include "utils/macros.h"
#include "ad_msg_c.h"


#ifdef __cplusplus
extern "C" {
#endif

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
typedef struct _LongtitudeControlInfo_t LongtitudeControlInfo_t;

struct _LongtitudeControlInfo_t
{
  double debug_VecDa_aCalcd_mp;                   // 本车加速度计算值
  double debug_VDC_facDrvTrmJ_mp;                 // 旋转质量换算系数 kg*m^2
  double debug_speed_integ_value;                 // unit : m/s^2 速度闭环积分反馈
  double debug_speed_propo_value;                 // unit : m/s^2 速度闭环比例反馈
  double debug_speed_value_a_req;                 // unit : m/s^2 速度闭环补偿值

  Uint8_t debug_speed_integ_freeze_status;        // 0 : unfreezed, 1 : freezed 速度闭环积分冻结状态
  Uint8_t debug_speed_inieg_init_status;          // 0 : uninitialized, 1 : initialized 速度闭环积分初始化状态
  
  double debug_speed_error;                       // unit : kph 车速偏差kph
  /** 
   *   0 ： 未激活纵向控制
   *   1 ： 触发紧急制动
   *   2 ： 常规驱动控制
   *   3 ： 常规制动控制
   *   4：起步
   *   5：停车
   *   6：临时驻车
   *   7：精准停车测试
   *   8：油门透传模式
   *   9：引擎外控测试(扭矩)
   *   10：制动系统外控测试
   *   11：引擎外控测试（油门开度）
   */
  Uint8_t debug_longitudinal_control_status;
  
  double debug_engine_torque_raw;                 // unit : % 发动机请求扭矩百分比 为处理的
  double debug_engine_torque;                     // unit : % 发动机请求扭矩百分比
  double debug_engine_torque_nfm;                 // unit : N*m 发动机请求扭矩

  double debug_Eng_rAccrPedlReq;                  // unit : % 油门开度请求
  double debug_throttle_real;                     // unit : % 真实的油门请求
  double debug_EBS_aReq;                          // unit : mps2 制动减速度请求
  double debug_vehicle_max_acce;                  // unit : m/s^2 最大允许的加速度

  Uint8_t debug_speedup_shift_status;             // 加速换挡状态
  Uint8_t debug_tcu_torque_status;                // 变速箱起步限扭状态
  Uint8_t debug_cochs_stnegtrqreq_mp;             // 负扭矩请求状态，0：驱动，1：制动

  double debug_fr_rolling_res_mp;                 // 滚动阻力
  double debug_fr_air_drag_mp;                    // 空气阻力
  double debug_fr_slp_res_mp;                     // 坡度阻力
  double debug_fr_acc_res_mp;                     // 加速阻力
  double debug_fr_cmp_mp;                         // 补偿驱动力
  double debug_fr_eng_fric_mp;                    // 发动机摩擦力

  Uint8_t debug_AD_Status;

  Uint8_t debug_Cochs_stStarting;                 // 驻车起步状态
  double debug_Vmc_aAccnGvnrD_mp;                 // 速度闭环微分反馈
  double debug_Vdc_aReqNom;                       // 名义加速度 m/s^2
  Uint8_t debug_Tra_stShiftWhtSpdUp;              // 速度限制状态
  Uint8_t debug_Tra_stTrqLimWthTcu;               // TCU限扭状态
  double debug_Tra_trqReq;                        // 变速箱需求扭矩
  double debug_acc_pid;                           // 加速PID

  // chassis info 
  double debug_SpdPln_lTrgLngErr_mp;              // 位置跟踪误差(需显示在界面)
  double debug_VehDa_nOutpShaft_mp;               // 变速箱输出轴转速
  Uint8_t debug_VehDa_stBrkReady4Rls_mp;          // 制动系统准备可以释放


  // adecu can 回放
  Uint32_t adecu_debug_ad_mode;
  Uint32_t adecu_debug_tar_vel_set;
  Uint32_t adecu_debug_tar_trajectory_type;
  Uint32_t adecu_debug_tar_vel_type;
  double adecu_debug_vel_obj_dist;
  double adecu_debug_vel_obj_vel;
  float adecu_debug_planning_send_vel;
  float adecu_debug_planning_send_ax;
  
};
#endif

/**
 * @struct VehCtlDataSource_t
 * @brief 车身控制算法所需数据
 */
typedef struct _VehCtlDataSource_t VehCtlDataSource_t;
struct _VehCtlDataSource_t {
  /// 时间戳
  Int64_t timestamp;

  /// 当前底盘控制状态
  Int32_t chassis_ctl_status;
  /// 车身控制请求
  const ChassisCtlCmd_t* request;
  /// 车身信息
  const Chassis_t* chassis;
};


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_VEHICLE_CONTROL_C_H_
