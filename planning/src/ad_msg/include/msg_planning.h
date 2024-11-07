/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_planning.h
 * @brief      运动规划消息定义
 * @details    定义了运动规划消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_PLANNING_H_
#define PHOENIX_AD_MSG_MSG_PLANNING_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"
#include "msg_chassis.h"


namespace phoenix {
namespace ad_msg {


/**
 * @struct PlanningResult
 * @brief 规划的结果
 */
struct PlanningResult {
  /**
   * @enum MAX_TRAJECTORY_POINT_NUM
   * @brief 目标轨迹中点的最大数量
   */
  enum { MAX_TRAJECTORY_POINT_NUM = 16 };
  /**
  * @enum TrjDirection
  * @brief 轨迹方向
  */
  enum TrjDirection {
    TRJ_DIRECTION_FORWARD = 0,
    TRJ_DIRECTION_BACKWARD
  };

  /// 消息头
  MsgHead msg_head;

  /// 当前决策模块的状态
  Uint32_t planning_status[4];

  /// 驾驶模式请求
  Int8_t tar_driving_mode;
  /// 使能转向系统
  Int8_t enable_eps;
  /// 使能油门系统
  Int8_t enable_throttle_sys;
  /// 使能制动系统
  Int8_t enable_ebs;

  /// 保持当前方向盘角度不变
  Int8_t hold_steering_wheel;
  /// 方向盘转速限制 (rad/s)
  Float32_t steering_wheel_speed;
  /// 释放油门
  Int8_t release_throttle;

  /// 档位请求
  Int8_t tar_gear;
  /// 转向指示灯请求
  Int8_t tar_turn_lamp;
  /// 制动灯请求
  Int8_t tar_brake_lamp;

  /// 速度请求
  Float32_t tar_v;
  /// 加速度请求
  Float32_t tar_a;

  /// 目标轨迹
  struct {
    /// 目标轨迹的时间戳，用于时间及空间同步
    Int64_t timestamp;
    /// 当前车辆位置 & 先导点位置
    struct {
      /// 坐标x
      Float32_t x;
      /// 坐标y
      Float32_t y;
      /// 航向角, Range: (-pi, pi rad)
      Float32_t h;
      /// 曲率
      Float32_t c;
      /// 相对于参考线的路径长
      Float32_t s;
      /// 相对于参考线的横向偏移
      Float32_t l;
    } curr_pos, leading_pos;

    /// 横向误差
    struct {
      /// 目标轨迹是否处于避障或变道状态
      Int8_t moving_flag;
      struct {
        /// 横向误差
        Float32_t lat_err;
        /// 横向误差变化速率
        Float32_t lat_err_chg_rate;
        /// 角度误差 Range: (-pi, pi rad)
        Float32_t yaw_err;
        /// 角度误差变化速率
        Float32_t yaw_err_chg_rate;
      } samples[2];
    } lat_err;

    /// 轨迹的方向， 0 - 前进， 1 - 后退
    Int8_t trj_direction;
    /// 轨迹点的数量
    Int16_t points_num;
    /// 轨迹点
    struct {
      /// 坐标x
      Float32_t x;
      /// 坐标y
      Float32_t y;
      /// 航向角, Range: (-pi, pi rad)
      Float32_t h;
      /// 曲率
      Float32_t c;
      /// 沿着轨迹的长度
      Float32_t s;
    } points[MAX_TRAJECTORY_POINT_NUM];

    /* xiaranfei 2022-07-31 (begin) */
    #if (ENABLE_OUTPUT_FITTING_COEFFICIENT) // 横向控制接口：1条拟合曲线和3条车道线
    struct {
            // 轨迹曲线拟合系数
            double coeffs[6];//
            /// 拟合曲线系数
            double PlanningTraj_Cv0;
            double PlanningTraj_Cv1;
            double PlanningTraj_Cv2;
            double PlanningTraj_Cv3;
            double PlanningTraj_Cv4;
            double PlanningTraj_Cv5;
            double PlanningTraj_Dist2PathZero;
            int32_t  PlanningTraj_IsValid ;
    } PlanningTraj_Cv;

    struct {
            /// 车道中心线系数
            double c_center[6];
            double LaneTraj_Cv0;
            double LaneTraj_Cv1;
            double LaneTraj_Cv2;
            double LaneTraj_Cv3;
            double LaneTraj_Cv4;
            double LaneTraj_Cv5;
            double LaneDist2PathZero;
            int32_t LaneTraj_IsValid ;
      } LaneTraj_Cv ;

    struct {
            /// left车道线系数
            double c_left[6];
            double LeftLaneTraj_Cv0;
            double LeftLaneTraj_Cv1;
            double LeftLaneTraj_Cv2;
            double LeftLaneTraj_Cv3;
            double LeftLaneTraj_Cv4;
            double LeftLaneTraj_Cv5;
            double LeftLaneDist2PathZero;
            int32_t LeftLaneTraj_IsValid;
      } LeftLaneTraj_Cv ;
      
      struct {
           /// right车道线系数
          double c_right[6];
          double RightLaneTraj_Cv0;
          double RightLaneTraj_Cv1;
          double RightLaneTraj_Cv2;
          double RightLaneTraj_Cv3;
          double RightLaneTraj_Cv4;
          double RightLaneTraj_Cv5;
          double RightLaneDist2PathZero;
          double RightLaneTraj_IsValid;
     } RightLaneTraj_Cv ;

    #endif
    /* xiaranfei 2022-07-31 (end) */
  } tar_trj;

  double ramp_slope_value;
  double front_nearest_obj_distance;
  double front_nearest_obj_speed;

  /// 俯仰角（-PI/2～PI/2弧度）
  Float32_t pitch;
  /// 俯仰角精度
  Float32_t pitch_variance;

  /// 油门值
  Float32_t tar_throttle;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    common::com_memset(planning_status, 0, sizeof(planning_status));

    tar_driving_mode = VEH_DRIVING_MODE_INVALID;
    enable_eps = 0;
    enable_throttle_sys = 0;
    enable_ebs = 0;

    hold_steering_wheel = 0;
    steering_wheel_speed = 0.0F;
    release_throttle = 0;

    tar_gear = VEH_GEAR_INVALID;
    tar_turn_lamp = VEH_TURN_LAMP_INVALID;
    tar_brake_lamp = VEH_LAMP_INVALID;

    tar_v = 0;
    tar_a = 0;

    tar_trj.timestamp = 0;
    tar_trj.curr_pos.x = 0;
    tar_trj.curr_pos.y = 0;
    tar_trj.curr_pos.h = 0;
    tar_trj.curr_pos.c = 0;
    tar_trj.curr_pos.s = 0;
    tar_trj.curr_pos.l = 0;
    tar_trj.leading_pos.x = 0;
    tar_trj.leading_pos.y = 0;
    tar_trj.leading_pos.h = 0;
    tar_trj.leading_pos.c = 0;
    tar_trj.leading_pos.s = 0;
    tar_trj.leading_pos.l = 0;

    common::com_memset(&tar_trj.lat_err, 0, sizeof(tar_trj.lat_err));

    tar_trj.trj_direction = TRJ_DIRECTION_FORWARD;
    tar_trj.points_num = 0;
    for (Int32_t i = 0; i < MAX_TRAJECTORY_POINT_NUM; ++i) {
      tar_trj.points[i].x = 0;
      tar_trj.points[i].y = 0;
      tar_trj.points[i].h = 0;
      tar_trj.points[i].c = 0;
      tar_trj.points[i].s = 0;
    }
    
    /* xiaranfei 2022-07-31 (start) */
    //初始化  coeffs  
    #if (ENABLE_OUTPUT_FITTING_COEFFICIENT)
      common::com_memset(tar_trj.PlanningTraj_Cv.coeffs, 0,sizeof(tar_trj.PlanningTraj_Cv.coeffs));
      common::com_memset(tar_trj.LaneTraj_Cv.c_center,0, sizeof(tar_trj.LaneTraj_Cv.c_center));
      common::com_memset(tar_trj.LeftLaneTraj_Cv.c_left,0, sizeof(tar_trj.LeftLaneTraj_Cv.c_left));
      common::com_memset(tar_trj.RightLaneTraj_Cv.c_right,0, sizeof(tar_trj.RightLaneTraj_Cv.c_right));
    #endif
    /* xiaranfei 2022-07-31 (end) */
    
    pitch = 0.0F;
    pitch_variance = 0.0F;
    ramp_slope_value = 0.0F;
    front_nearest_obj_distance = 0.0F;
    front_nearest_obj_speed = 0.0F;
    tar_throttle = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  PlanningResult() {
    Clear();
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_PLANNING_H_


