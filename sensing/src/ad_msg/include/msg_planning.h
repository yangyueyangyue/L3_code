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
  Int32_t cur_status;

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
  } tar_trj;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    cur_status = 0;

    tar_driving_mode = VEH_DRIVING_MODE_INVALID;
    enable_eps = 0;
    enable_throttle_sys = 0;
    enable_ebs = 0;

    hold_steering_wheel = 0;
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


