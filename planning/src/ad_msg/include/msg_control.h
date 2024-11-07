/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_control.h
 * @brief      运动控制消息定义
 * @details    定义了运动控制消息类型
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

#ifndef PHOENIX_AD_MSG_MSG_CONTROL_H_
#define PHOENIX_AD_MSG_MSG_CONTROL_H_


#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @struct LateralControlPidInfo
 * @brief 横向控制模块(PID)状态信息
 */
struct LateralControlPidInfo {
  /// 轨迹前馈
  struct {
    /// 当前车辆在轨迹上的投影点, 预瞄点(近端)在轨迹上的投影点,
    /// 预瞄点(远端)在轨迹上的投影点
    struct {
      Float32_t x;
      Float32_t y;
      Float32_t h;
      Float32_t s;
    } cur_proj_on_trj, goal_point_near, goal_point_far;
    /// 预瞄距离
    Float32_t goal_dist;
    /// 预瞄距离(近端)
    Float32_t goal_dist_near;
    /// 预瞄距离(远端)
    Float32_t goal_dist_far;
    /// 轨迹前馈目标曲率(近端)
    Float32_t feed_value_near;
    /// 轨迹前馈目标曲率(远端)
    Float32_t feed_value_far;
    /// 轨迹前馈目标曲率
    Float32_t feed_value;
    /// 轨迹前馈目标曲率(平滑后)
    Float32_t feed_value_smooth;
  } trj_feed;

  /// 横向误差补偿
  struct {
    /// 横向误差
    Float32_t lat_err;
    /// 横向误差变化率
    Float32_t lat_err_spd;
    /// 角度误差
    Float32_t yaw_err;
    /// 角度误差变化率
    Float32_t yaw_err_spd;
    /// 横向误差等级
    Int32_t lat_err_lv_idx;
    /// 横向误差变化率等级
    Int32_t lat_err_spd_lv_idx;
    /// PID补偿
    struct {
      Float32_t v_ratio;
      Float32_t w_ratio;
      Float32_t k_ratio;
      Float32_t k_value;
      Float32_t feed;
    } p_feed, i_feed, d_feed;
    /// 补偿值
    Float32_t feed_value;
  } lat_err_feed;

  /// 角速度误差补偿
  struct {
    /// 角速度误差
    Float32_t yaw_rate_err;
    /// 角速度误差变化率
    Float32_t yaw_rate_spd;
    /// 角速度误差的等级索引
    Int32_t yaw_rate_err_lv_idx;
    /// 角速度误差变化率的等级索引
    Int32_t yaw_rate_spd_lv_idx;
    /// PID补偿
    struct {
      Float32_t v_ratio;
      Float32_t w_ratio;
      Float32_t k_ratio;
      Float32_t k_value;
      Float32_t feed;
    } p_feed, i_feed, d_feed;
    /// 补偿值
    Float32_t feed_value;
  } yaw_rate_err_feed;

  /// 车辆动态特性补偿
  struct {
    /// 轨迹曲率
    Float32_t trj_curvature;
    /// PID补偿
    struct {
      Float32_t v_ratio;
      Float32_t w_ratio;
      Float32_t k_ratio;
      Float32_t k_value;
      Float32_t feed;
    } p_feed, i_feed, d_feed;
    /// 补偿值
    Float32_t feed_value;
  } veh_dynamic_feed;

  /// 方向盘角度增益
  Float32_t steering_gain;

  /// 目标方向盘转角
  Float32_t tar_steering_angle;
  /// 平滑后的目标方向盘转角
  Float32_t tar_steering_angle_smooth;

  /**
   * @brief 构造函数
   */
  LateralControlPidInfo() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    common::com_memset(this, 0, sizeof(LateralControlPidInfo));
  }
};


/**
 * @struct LateralControlInfo
 * @brief 横向控制模块状态信息
 */
struct LateralControlInfo {
  /// 消息头
  MsgHead msg_head;

  /// 横向控制模块结果状态
  Int32_t lat_ctl_result;

  /// 角速度(从IMU获取)
  Float32_t yaw_rate_imu;
  /// 角速度(从车身获取)
  Float32_t yaw_rate_chassis;
  /// 角速度(从方向盘角度计算)
  Float32_t yaw_rate_steering;
  /// 角速度
  Float32_t yaw_rate;
  /// 角速度变化率
  Float32_t yaw_rate_chg_rate;
  /// 目标角速度
  Float32_t target_yaw_rate;

  /// 当前车辆位姿
  struct {
    Int32_t relative_time;
    Float32_t x;
    Float32_t y;
    Float32_t heading;
    Float32_t yaw_rate;
    Float32_t v;
  } curr_pos;

  /// 横向误差
  Float32_t lat_err[2];
  /// 横向误差变化率
  Float32_t lat_err_chg_rate[2];
  /// 角度误差
  Float32_t yaw_err[2];
  /// 角度误差变化率
  Float32_t yaw_err_chg_rate[2];

  /// 车速(m/s)
  Float32_t veh_spd;
  /// 车辆总质量 (kg)
  Float32_t veh_gross_weight;

  /// 横向控制模块(PID)状态信息
  LateralControlPidInfo lat_ctl_pid_info;

  /// 目标方向盘角度
  Float32_t target_steering_wheel_angle;
  /// 目标方向盘转速
  Float32_t target_steering_wheel_angle_speed;

  /**
   * @brief 构造函数
   */
  LateralControlInfo() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    lat_ctl_result = 0;
    yaw_rate_imu = 0.0F;
    yaw_rate_chassis = 0.0F;
    yaw_rate_steering = 0.0F;
    yaw_rate = 0.0F;
    yaw_rate_chg_rate = 0.0F;
    target_yaw_rate = 0.0F;

    common::com_memset(&curr_pos, 0, sizeof(curr_pos));

    common::com_memset(&lat_err[0], 0, sizeof(lat_err));
    common::com_memset(&lat_err_chg_rate[0], 0, sizeof(lat_err_chg_rate));
    common::com_memset(&yaw_err[0], 0, sizeof(yaw_err));
    common::com_memset(&yaw_err_chg_rate[0], 0, sizeof(yaw_err_chg_rate));

    veh_spd = 0.0F;
    veh_gross_weight = 0.0F;

    lat_ctl_pid_info.Clear();

    target_steering_wheel_angle = 0.0F;
    target_steering_wheel_angle_speed = 0.0F;
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_CONTROL_H_


