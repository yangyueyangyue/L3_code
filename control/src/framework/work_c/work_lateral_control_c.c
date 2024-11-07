/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       work_lateral_control.c
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

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "work_c/work_lateral_control_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "math/math_utils_c.h"
#include "communication_c/shared_data_c.h"

#include "vehicle_model_c.h"
#include "pos_filter_wrapper_c.h"
#include "lateral_control_wrapper_c.h"


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化横向控制功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Work_LateralCtl_Initialize(WorkLatCtlInstance_t* const instance) {
  Phoenix_AdMsg_ClearImu(&(instance->imu));
  Phoenix_AdMsg_ClearChassis(&(instance->chassis));
  Phoenix_AdMsg_ClearPlanningResult(&(instance->planning_result));

  Phoenix_AdMsg_ClearRelativePos(&(instance->curr_pos));
  phoenix_com_memset(&(instance->lat_ctl_data_src), 0,
                     sizeof(instance->lat_ctl_data_src));

  instance->tar_yaw_rate = 0.0F;
  instance->tar_steering_wheel_angle = 0.0F;
  instance->tar_steering_wheel_speed = 0.0F;

  phoenix_com_memset(&(instance->lat_ctl_info), 0,
                     sizeof(instance->lat_ctl_info));

  Phoenix_PosFilter_Initialize();
  Phoenix_LatCtl_Initialize();
}

/*
 * @brief 开始一次横向控制功能（定周期调用）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Work_LateralCtl_DoWork(WorkLatCtlInstance_t* const instance) {
  Int64_t curr_timestamp = 0;
  Int64_t time_elapsed = 0;
  Int32_t chassis_ctl_status = VEH_CHASSIS_CTL_STATUS_MANUAL;
  Int32_t pos_filter_ret = 0;
  Int32_t lat_ctl_ret = 0;
  Int32_t control_result = 0;
  Float32_t abs_angle = 0.0F;
  Float32_t prev_tar_yaw_rate = 0.0F;
  Float32_t prev_tar_steering_wheel_angle = 0.0F;
  PosFilterDataSource_t pos_filter_data_src;
 
#if ((CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV) || (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV))
 /// 增加DFCV控制需用的专用信息   20220817
  DFCV_SpecialChassisInfo_t  dfcvctrl_src_special_chassis;
  Phoenix_SharedData_GetDFCVSpecialChassisInfo(&dfcvctrl_src_special_chassis);    
#endif
  /// 读取所需的数据
  Phoenix_SharedData_GetChassisCtlStatus(&chassis_ctl_status);
  Phoenix_SharedData_GetImu(&(instance->imu));
  Phoenix_SharedData_GetChassis(&(instance->chassis));
  Phoenix_SharedData_GetPlanningResult(&(instance->planning_result));

#if (ENTER_PLAYBACK_MODE)
  chassis_ctl_status = VEH_CHASSIS_CTL_STATUS_ROBOTIC;
#endif

  /// 保存之前计算的目标角速度及方向盘目标转角
  prev_tar_yaw_rate = instance->tar_yaw_rate;
  prev_tar_steering_wheel_angle = instance->tar_steering_wheel_angle;

  /// 读取时间戳
  curr_timestamp = Phoenix_Common_GetClockNowMs();
  time_elapsed = Phoenix_Common_CalcElapsedClockMs(
          instance->planning_result.msg_head.timestamp, curr_timestamp);
  if (time_elapsed > 300) {
    instance->tar_steering_wheel_angle = instance->chassis.steering_wheel_angle;
    Phoenix_LatCtl_ResetLatCtlValue(instance->chassis.steering_wheel_angle);
    LOG_ERR_C("Receiving planning result timeout.");
  }

  if (VEH_CHASSIS_CTL_STATUS_ROBOTIC != chassis_ctl_status) {
    instance->tar_steering_wheel_angle = instance->chassis.steering_wheel_angle;
    Phoenix_LatCtl_ResetLatCtlValue(instance->chassis.steering_wheel_angle);
  }

  /// 位置及时间同步
  // 填充数据
  // 时间戳
  pos_filter_data_src.timestamp = curr_timestamp;
  // 车辆当前在目标轨迹中的相对位置
  pos_filter_data_src.rel_pos.timestamp =
      instance->planning_result.msg_head.timestamp +
      instance->planning_result.tar_trj.timestamp;
#if 0
  printf("#>>>@ 横向控制:\n");
  printf("      pos_filter_timestamp = %ld\n",
         pos_filter_data_src.timestamp);
  printf("      plan_ret_timestamp = %ld\n",
         instance->planning_result.msg_head.timestamp);
  printf("      trj_timestamp = %ld\n",
         instance->planning_result.tar_trj.timestamp);
  printf("      ref_pos_timestamp = %ld\n",
         pos_filter_data_src.rel_pos.timestamp);
#endif
  // 坐标x
  pos_filter_data_src.rel_pos.x = instance->planning_result.tar_trj.curr_pos.x;
  // 坐标y
  pos_filter_data_src.rel_pos.y = instance->planning_result.tar_trj.curr_pos.y;
  // 航向角 Range: (-pi, pi rad)
  pos_filter_data_src.rel_pos.heading =
      instance->planning_result.tar_trj.curr_pos.h;
  // 车辆姿态信息
  pos_filter_data_src.imu = &(instance->imu);
  // 车身信息（通常是车身CAN信号）
  pos_filter_data_src.chassis = &(instance->chassis);
  // 平滑/过滤当前位置，位置及时间同步
  pos_filter_ret = Phoenix_PosFilter_Update(&pos_filter_data_src);
  if (pos_filter_ret < 0) {
    instance->curr_pos.relative_time = 0;
    instance->curr_pos.x = pos_filter_data_src.rel_pos.x;
    instance->curr_pos.y = pos_filter_data_src.rel_pos.y;
    instance->curr_pos.heading = pos_filter_data_src.rel_pos.heading;
    instance->curr_pos.yaw_rate = instance->chassis.yaw_rate;
    instance->curr_pos.yaw_rate_chg_rate = 0;
    instance->curr_pos.yaw_rate_d_chg_rate = 0;
    instance->curr_pos.v = instance->chassis.v;

    control_result = -1;
    LOG_ERR_C("Failed to filter pos.");
  } else {
    const RelativePos_t* est_pos = Phoenix_PosFilter_GetEstimatedPos();
    instance->curr_pos.relative_time = est_pos->relative_time;
    instance->curr_pos.x = est_pos->x;
    instance->curr_pos.y = est_pos->y;
    instance->curr_pos.heading = est_pos->heading;
    instance->curr_pos.yaw_rate = est_pos->yaw_rate;
    instance->curr_pos.yaw_rate_chg_rate = est_pos->yaw_rate_chg_rate;
    instance->curr_pos.yaw_rate_d_chg_rate = est_pos->yaw_rate_d_chg_rate;
    instance->curr_pos.v = est_pos->v;
  }

  /// 横向控制
  // 填充数据
  // 时间戳
  instance->lat_ctl_data_src.timestamp = curr_timestamp;
  // 参数
  instance->lat_ctl_data_src.param.valid = 0;
  // 全局增益(横向误差补偿)
  instance->lat_ctl_data_src.param.p_global_gain_lat_err_feed = 1.0F;
  instance->lat_ctl_data_src.param.i_global_gain_lat_err_feed = 1.0F;
  instance->lat_ctl_data_src.param.d_global_gain_lat_err_feed = 1.0F;
  // 全局增益(角度误差补偿)
  instance->lat_ctl_data_src.param.p_global_gain_yaw_rate_err_feed = 1.0F;
  instance->lat_ctl_data_src.param.i_global_gain_yaw_rate_err_feed = 1.0F;
  instance->lat_ctl_data_src.param.d_global_gain_yaw_rate_err_feed = 1.0F;
  // 车辆当前在目标轨迹中的相对位置
  instance->lat_ctl_data_src.cur_pos.x = instance->curr_pos.x;
  instance->lat_ctl_data_src.cur_pos.y = instance->curr_pos.y;
  instance->lat_ctl_data_src.cur_pos.heading = instance->curr_pos.heading;
  instance->lat_ctl_data_src.cur_pos.v = phoenix_com_abs_f(instance->curr_pos.v);
  instance->lat_ctl_data_src.cur_pos.a = instance->chassis.a;
  instance->lat_ctl_data_src.cur_pos.yaw_rate = instance->curr_pos.yaw_rate;
  if (VEH_GEAR_R == instance->chassis.gear) {
    instance->lat_ctl_data_src.cur_pos.yaw_rate = -instance->curr_pos.yaw_rate;
  }
  instance->lat_ctl_data_src.cur_pos.yaw_rate_chg_rate =
      instance->curr_pos.yaw_rate_chg_rate;
  instance->lat_ctl_data_src.cur_pos.yaw_rate_d_chg_rate =
      instance->curr_pos.yaw_rate_d_chg_rate;
  // 横向误差
  instance->lat_ctl_data_src.lat_err.moving_flag =
      instance->planning_result.tar_trj.lat_err.moving_flag;
  instance->lat_ctl_data_src.lat_err.samples[0].lat_err =
      instance->planning_result.tar_trj.lat_err.samples[0].lat_err;
  instance->lat_ctl_data_src.lat_err.samples[0].lat_err_chg_rate =
      instance->planning_result.tar_trj.lat_err.samples[0].lat_err_chg_rate;
  instance->lat_ctl_data_src.lat_err.samples[0].yaw_err =
      instance->planning_result.tar_trj.lat_err.samples[0].yaw_err;
  instance->lat_ctl_data_src.lat_err.samples[0].yaw_err_chg_rate =
      instance->planning_result.tar_trj.lat_err.samples[0].yaw_err_chg_rate;
  instance->lat_ctl_data_src.lat_err.samples[1].lat_err =
      instance->planning_result.tar_trj.lat_err.samples[1].lat_err;
  instance->lat_ctl_data_src.lat_err.samples[1].lat_err_chg_rate =
      instance->planning_result.tar_trj.lat_err.samples[1].lat_err_chg_rate;
  instance->lat_ctl_data_src.lat_err.samples[1].yaw_err =
      instance->planning_result.tar_trj.lat_err.samples[1].yaw_err;
  instance->lat_ctl_data_src.lat_err.samples[1].yaw_err_chg_rate =
      instance->planning_result.tar_trj.lat_err.samples[1].yaw_err_chg_rate;
  // 轨迹规划生成的轨迹类型
  instance->lat_ctl_data_src.planning_trj_status =
      Phoenix_AdMsg_GetTrjPlanngingTrjStatus(instance->planning_result.planning_status);
  // 车道中心线的曲率
  instance->lat_ctl_data_src.ref_trj_curvature =
      instance->planning_result.tar_trj.leading_pos.c;
  // 目标轨迹
  if (AD_MSG_TRJ_DIRECTION_BACKWARD ==
      instance->planning_result.tar_trj.trj_direction) {
    instance->lat_ctl_data_src.tar_trj.direction =
        LAT_CTL_TRJ_DIRECTION_BACKWARD;
  } else {
    instance->lat_ctl_data_src.tar_trj.direction =
        LAT_CTL_TRJ_DIRECTION_FORWARD;
  }
  instance->lat_ctl_data_src.tar_trj.points_num =
      instance->planning_result.tar_trj.points_num;
  if (instance->lat_ctl_data_src.tar_trj.points_num >
      LAT_CTL_MAX_TRAJECTORY_POINT_NUM) {
    instance->lat_ctl_data_src.tar_trj.points_num =
        LAT_CTL_MAX_TRAJECTORY_POINT_NUM;
  }
  for (Int32_t i = 0; i < instance->lat_ctl_data_src.tar_trj.points_num; ++i) {
    // 坐标x
    instance->lat_ctl_data_src.tar_trj.points[i].x =
        instance->planning_result.tar_trj.points[i].x;
    // 坐标y
    instance->lat_ctl_data_src.tar_trj.points[i].y =
        instance->planning_result.tar_trj.points[i].y;
    // 航向角), Range: (-pi, pi rad)
    instance->lat_ctl_data_src.tar_trj.points[i].h =
        instance->planning_result.tar_trj.points[i].h;
    // 曲率
    instance->lat_ctl_data_src.tar_trj.points[i].c =
        instance->planning_result.tar_trj.points[i].c;
    // 相对于参考线的路径长
    instance->lat_ctl_data_src.tar_trj.points[i].s =
        instance->planning_result.tar_trj.points[i].s;
    // 相对于参考线的横向偏移
    instance->lat_ctl_data_src.tar_trj.points[i].l = 0.0F;
  }
  // 底盘信息
  instance->lat_ctl_data_src.chassis.eps_auto = 0;
  if (VEH_EPS_STATUS_ROBOTIC == instance->chassis.eps_status) {
    instance->lat_ctl_data_src.chassis.eps_auto = 1;
  }
  instance->lat_ctl_data_src.chassis.steering_wheel_angle =
      instance->chassis.steering_wheel_angle;
  instance->lat_ctl_data_src.chassis.gross_weight =
      instance->chassis.gross_weight;

  // 计算方向盘目标转角
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
   if (instance->planning_result.tar_trj.points_num < 2) {
#elif (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
   int32_t  Traj_IsValid =  instance->planning_result.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid ;
   int32_t  Lane_IsValid = instance->planning_result.tar_trj.LaneTraj_Cv.LaneTraj_IsValid;

   printf("LaneTraj_IsValid = %d \n",Lane_IsValid);
   printf("PlanningTraj_IsValid = %d \n",Traj_IsValid);

   if (!Traj_IsValid )  {
#endif
    // 保持之前的目标方向盘转角
    control_result = -2;
    LOG_ERR_C("Invalid trajectory, the number of points is too small.");
  } else {
    #if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_GOYU)
    lat_ctl_ret = Phoenix_LatCtl_CalcLatCtlValue(
          &(instance->lat_ctl_data_src),
          &(instance->tar_yaw_rate), &(instance->tar_steering_wheel_angle),
          &(instance->lat_ctl_info));
    #elif (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
    lat_ctl_ret = DFCV_LatCtl_CalcLatCtlValue(
          &(instance->chassis),&dfcvctrl_src_special_chassis,
          &(instance->planning_result), &(instance->imu), &chassis_ctl_status,
          &(instance->tar_yaw_rate), &(instance->tar_steering_wheel_angle),
          &(instance->lat_ctl_info));
    #endif

    if (lat_ctl_ret < 0) {
      instance->tar_yaw_rate = prev_tar_yaw_rate;
      instance->tar_steering_wheel_angle = prev_tar_steering_wheel_angle;
      control_result = -3;
      LOG_ERR_C("Failed to calculate target steering wheel angle.");
    } else {
      if (instance->planning_result.hold_steering_wheel) {
        instance->tar_yaw_rate = prev_tar_yaw_rate;
        instance->tar_steering_wheel_angle = prev_tar_steering_wheel_angle;
      }
    }
  }
  // 限制方向盘的转速
  instance->tar_steering_wheel_speed =
      instance->planning_result.steering_wheel_speed;

  // printf("instance->tar_steering_wheel_speed=%0.1f\n",
  //        phoenix_com_rad2deg_f(instance->tar_steering_wheel_speed));

  // 设置调试信息
  instance->lat_ctl_info.lat_ctl_result = control_result;
  instance->lat_ctl_info.yaw_rate_steering =
      Phoenix_PosFilter_GetYawRateFromSteeringAngle();
  instance->lat_ctl_info.yaw_rate_imu = Phoenix_PosFilter_GetYawRateFromImu();
  instance->lat_ctl_info.yaw_rate_chassis = Phoenix_PosFilter_GetYawRateFromChassis();
  instance->lat_ctl_info.yaw_rate = instance->curr_pos.yaw_rate;
  instance->lat_ctl_info.yaw_rate_chg_rate = instance->curr_pos.yaw_rate_chg_rate;
  instance->lat_ctl_info.yaw_rate_d_chg_rate = instance->curr_pos.yaw_rate_d_chg_rate;
  instance->lat_ctl_info.target_yaw_rate = instance->tar_yaw_rate;
  instance->lat_ctl_info.target_steering_wheel_angle =
      instance->tar_steering_wheel_angle;
  instance->lat_ctl_info.curr_pos.relative_time =
      instance->curr_pos.relative_time;
  instance->lat_ctl_info.curr_pos.x = instance->curr_pos.x;
  instance->lat_ctl_info.curr_pos.y = instance->curr_pos.y;
  instance->lat_ctl_info.curr_pos.heading = instance->curr_pos.heading;
  instance->lat_ctl_info.curr_pos.yaw_rate = instance->curr_pos.yaw_rate;
  instance->lat_ctl_info.curr_pos.v = instance->curr_pos.v;

  instance->lat_ctl_info.veh_spd = instance->chassis.v;
  instance->lat_ctl_info.veh_gross_weight = instance->chassis.gross_weight;
  
  instance->lat_ctl_info.lat_err[0] =
      instance->lat_ctl_data_src.lat_err.samples[0].lat_err;
  instance->lat_ctl_info.lat_err[1] =
      instance->lat_ctl_data_src.lat_err.samples[1].lat_err;
  instance->lat_ctl_info.lat_err_chg_rate[0] =
      instance->lat_ctl_data_src.lat_err.samples[0].lat_err_chg_rate;
  instance->lat_ctl_info.lat_err_chg_rate[1] =
      instance->lat_ctl_data_src.lat_err.samples[1].lat_err_chg_rate;
  instance->lat_ctl_info.yaw_err[0] =
      instance->lat_ctl_data_src.lat_err.samples[0].yaw_err;
  instance->lat_ctl_info.yaw_err[1] =
      instance->lat_ctl_data_src.lat_err.samples[1].yaw_err;
  instance->lat_ctl_info.yaw_err_chg_rate[0] =
      instance->lat_ctl_data_src.lat_err.samples[0].yaw_err_chg_rate;
  instance->lat_ctl_info.yaw_err_chg_rate[1] =
      instance->lat_ctl_data_src.lat_err.samples[1].yaw_err_chg_rate;

  // 保存调试信息
  Phoenix_SharedData_SetLateralControlInfo(&(instance->lat_ctl_info));

  return (control_result);
}

