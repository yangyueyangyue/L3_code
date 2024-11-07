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

#ifndef PHOENIX_AD_MSG_MSG_PLANNING_C_H_
#define PHOENIX_AD_MSG_MSG_PLANNING_C_H_

#include "utils/macros.h"
#include "utils/com_utils_c.h"
#include "msg_common_c.h"
#include "msg_chassis_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @enum
 * @brief 驾驶地图类型
 */
enum {
  /// 无效的类型
  DRIVING_MAP_TYPE_INVALID = 0,
  /// 从高精度地图构建
  DRIVING_MAP_TYPE_HD_MAP,
  /// 从相机车道线构建
  DRIVING_MAP_TYPE_CAMERA_LANE,
  /// 融合了相机车道线及高精度地图
  DRIVING_MAP_TYPE_MIXED_HD_MAP,
  /// 跟随前车
  DRIVING_MAP_TYPE_FOLLOWING_PATH,
  /// 根据当前车辆状态预测的轨迹
  DRIVING_MAP_TYPE_PRED_PATH
};

/**
 * @enum
 * @brief 轨迹规划路网构建类型
 */
enum {
  /// 无效的类型
  TRJ_PLAN_ROAD_BUILED_TYPE_INVALID = 0,
  /// 从高精度地图中构建
  TRJ_PLAN_ROAD_BUILED_TYPE_HD_MAP,
  /// 从相机车道线中构建
  TRJ_PLAN_ROAD_BUILED_TYPE_CAMERA_LANE,
  /// 从融合了相机车道线及高精度地图中构建
  TRJ_PLAN_ROAD_BUILED_TYPE_MIXED_HD_MAP,
  /// 从跟随前车轨迹中构建
  TRJ_PLAN_ROAD_BUILED_TYPE_FOLLOWING,
  /// 从根据当前车辆状态预测的轨迹中构建
  TRJ_PLAN_ROAD_BUILED_TYPE_PRED_PATH
};

/**
 * @enum
 * @brief 轨迹规划生成的轨迹类型
 */
enum {
  /// 无效的类型
  TRJ_PLAN_TRJ_STATUS_INVALID = 0,
  /// 车道保持
  TRJ_PLAN_TRJ_STATUS_LKA,
  /// 向左绕行
  TRJ_PLAN_TRJ_STATUS_LKA_BYPASSING_LEFT,
  /// 向右绕行
  TRJ_PLAN_TRJ_STATUS_LKA_BYPASSING_RIGHT,
  /// 向左变道(还未变更到左侧车道)
  TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I,
  /// 向左变道(已经变更到左侧车道)
  TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II,
  /// 向右变道(还未变更到右侧车道)
  TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I,
  /// 向右变道(已经变更到右侧车道)
  TRJ_PLAN_TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II,
  /// 中止左变道，回到右车道（还未变更到右侧车道）
  TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I,
  /// 中止左变道，回到右车道（已经变更到右侧车道）
  TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II,
  /// 中止向右变道，回到左车道（还未变更到左侧车道）
  TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I,
  /// 中止向右变道，回到左车道（已经变更到左侧车道）
  TRJ_PLAN_TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II
  /// TODO: 增加靠边停车、紧急避让等类型
};

/**
 * @struct
 * @brief 速度控制目标类型
 */
enum {
  /// 无效类型
  VEL_PLAN_TARGET_TYPE_INVALID = 0,
  /// 根据用户设置控制车速
  VEL_PLAN_TARGET_TYPE_USER_SETTINGS,
  /// 根据道路曲率控制车速
  VEL_PLAN_TARGET_TYPE_CURVATURE,
  /// 根据障碍物控制车速
  VEL_PLAN_TARGET_TYPE_OBSTACLE,
  /// 跟随前车
  VEL_PLAN_TARGET_TYPE_FOLLOWING,
  /// 低速跟随
  VEL_PLAN_TARGET_TYPE_LOW_SPEED_FOLLOWING,
  /// AEB告警
  VEL_PLAN_TARGET_TYPE_AEB_WARN,
  /// 执行AEB
  VEL_PLAN_TARGET_TYPE_AEB_ACTION,
  /// 根据场景任务控制车速
  VEL_PLAN_TARGET_TYPE_SCENE_STORY,
  /// 根据交通灯控制车速
  VEL_PLAN_TARGET_TYPE_TRAFFIC_LIGHT,
  /// 根据限速牌控制车速
  VEL_PLAN_TARGET_TYPE_ISL
};

/**
 * @enum
 * @brief 目标轨迹中点的最大数量
 */
enum { AD_MSG_MAX_TRAJECTORY_POINT_NUM = 16 };

/**
* @enum
* @brief 轨迹方向
*/
enum {
  AD_MSG_TRJ_DIRECTION_FORWARD = 0,
  AD_MSG_TRJ_DIRECTION_BACKWARD
};


/**
 * @struct PlanningResult
 * @brief 规划的结果
 */
typedef struct _PlanningResult_t PlanningResult_t;
struct _PlanningResult_t {
  /// 消息头
  MsgHead_t msg_head;

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
        /// 角度误差, Range: (-pi, pi rad)
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
    } points[AD_MSG_MAX_TRAJECTORY_POINT_NUM];
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
    struct {
   
      double PlanningTraj_Cv0;
      double PlanningTraj_Cv1;
      double PlanningTraj_Cv2;
      double PlanningTraj_Cv3;
      double PlanningTraj_Cv4;
      double PlanningTraj_Cv5;

      double PlanningTraj_Dist2PathZero;
      int32_t PlanningTraj_IsValid;

         double coeffs[6];

    } PlanningTraj_Cv;
  
    struct {
 
      double LaneTraj_Cv0;
      double LaneTraj_Cv1;
      double LaneTraj_Cv2;
      double LaneTraj_Cv3;
      double LaneTraj_Cv4;
      double LaneTraj_Cv5;

      double LaneDist2PathZero;
      int32_t LaneTraj_IsValid;

      double c_center[6];

    } LaneTraj_Cv;
#endif
  } tar_trj;

  Float32_t ramp_slope_value;
  double front_nearest_obj_distance;
  double front_nearest_obj_speed;

  /// 俯仰角（-PI/2～PI/2弧度）
  Float32_t pitch;
  /// 俯仰角精度
  Float32_t pitch_variance;

  /// 油门值
  Float32_t tar_throttle;
};

/**
 * @brief 清除内部数据
 */
void Phoenix_AdMsg_ClearPlanningResult(PlanningResult_t* const data);


/**
 * @brief 获取驾驶地图类型
 * @param[in] planning_status 规划模块的状态标记
 * @return 驾驶地图类型
 */
Int32_t Phoenix_AdMsg_GetDrivingMapTpye(Uint32_t* const planning_status);

/**
 * @brief 获取轨迹规划路网构建类型
 * @param[in] planning_status 规划模块的状态标记
 * @return 轨迹规划路网构建类型
 */
Int32_t Phoenix_AdMsg_GetTrjPlanngingRoadTpye(Uint32_t* const planning_status);

/**
 * @brief 获取轨迹规划轨迹类型
 * @param[in] planning_status 规划模块的状态标记
 * @return 轨迹规划轨迹类型
 */
Int32_t Phoenix_AdMsg_GetTrjPlanngingTrjStatus(Uint32_t* const planning_status);

/**
 * @brief 获取速度规划目标类型
 * @param[in] planning_status 规划模块的状态标记
 * @return 速度规划目标类型
 */
Int32_t Phoenix_AdMsg_GetVelPlanngingTarType(Uint32_t* const planning_status);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_AD_MSG_MSG_PLANNING_C_H_


