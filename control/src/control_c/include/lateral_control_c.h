/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lateral_control.h
 * @brief      数据类型定义
 * @details    横向控制所需的数据类型定义
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

#ifndef PHOENIX_CONTROL_LATERAL_CONTROL_C_H_
#define PHOENIX_CONTROL_LATERAL_CONTROL_C_H_

#include "utils/macros.h"
#include "ad_msg_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @enum
 * @brief PID横向控制模块中线性插值表的尺寸
 */
enum { LAT_CTL_PID_LERP_TABLE_SIZE = 25 };

/**
 * @enum
 * @brief PID横向控制模块中横向误差补偿表的尺寸
 */
enum { LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE = 24 };

/**
 * @enum
 * @brief PID横向控制模块中横向速率补偿表的尺寸
 */
enum { LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE = 24 };

/**
 * @enum
 * @brief PID横向控制模块中航向角误差补偿表的尺寸
 */
enum { LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE = 8 };

/**
 * @enum
 * @brief PID横向控制模块中角速度误差补偿表的尺寸
 */
enum { LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE = 18 };

/**
 * @enum
 * @brief 目标轨迹中点的最大数量
 */
enum { LAT_CTL_MAX_TRAJECTORY_POINT_NUM = 20 };

/**
 * @enum
 * @brief 横向控制频谱图的尺寸
 */
enum { LAT_CTL_FREQUENCY_SPECTRUM_SIZE = 64 };

/**
 * @enum
 * @brief 输入输出响应关系项数量
 */
enum { LAT_CTL_ANA_TRANS_REL_ITEM_NUM = 21 };

/**
* @enum LatCtlTrjDirection
* @brief 轨迹方向
*/
enum LatCtlTrjDirection {
  LAT_CTL_TRJ_DIRECTION_FORWARD = 0,
  LAT_CTL_TRJ_DIRECTION_BACKWARD
};

/**
* @enum LatCtlCurvatureType
* @brief 轨迹曲率类型
*/
enum LatCtlCurvatureType {
  LAT_CTL_CURVATURE_TYPE_STRAIGHT = 0,
  LAT_CTL_CURVATURE_TYPE_LEFT,
  LAT_CTL_CURVATURE_TYPE_RIGHT,
  LAT_CTL_CURVATURE_TYPE_INTO_LEFT,
  LAT_CTL_CURVATURE_TYPE_INTO_RIGHT,
  LAT_CTL_CURVATURE_TYPE_OUT_LEFT,
  LAT_CTL_CURVATURE_TYPE_OUT_RIGHT,
  LAT_CTL_CURVATURE_TYPE_MAX
};


/**
 * @struct LatCtlPathPoint_t
 * @brief 轨迹点
 */
typedef struct _LatCtlPathPoint_t LatCtlPathPoint_t;
struct _LatCtlPathPoint_t {
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
};

/**
 * @struct LatCtlTrajectory_t
 * @brief 轨迹
 */
typedef struct _LatCtlTrajectory_t LatCtlTrajectory_t;
struct _LatCtlTrajectory_t {
  /// 轨迹的方向， 0 - 前进， 1 - 后退
  Int32_t direction;
  /// 轨迹点的数量
  Int32_t points_num;
  /// 轨迹点
  LatCtlPathPoint_t points[LAT_CTL_MAX_TRAJECTORY_POINT_NUM];
};

/**
 * @struct LatCtlDataSource
 * @brief 横向控制算法所需数据
 */
typedef struct _LatCtlDataSource_t LatCtlDataSource_t;
struct _LatCtlDataSource_t {
  /// 时间戳
  Int64_t timestamp;

  struct {
    Int8_t valid;
    /// 全局增益(横向误差补偿)
    Float32_t p_global_gain_lat_err_feed;
    Float32_t i_global_gain_lat_err_feed;
    Float32_t d_global_gain_lat_err_feed;
    /// 全局增益(角度误差补偿)
    Float32_t p_global_gain_yaw_rate_err_feed;
    Float32_t i_global_gain_yaw_rate_err_feed;
    Float32_t d_global_gain_yaw_rate_err_feed;
  } param;

  /// 当前车辆在目标轨迹中的位置
  struct {
    /// 坐标x
    Float32_t x;
    /// 坐标y
    Float32_t y;
    /// 航向角, Range: (-pi, pi rad)
    Float32_t heading;
    /// 车速(m/s)
    Float32_t v;
    /// 加速度(m/s^2)
    Float32_t a;
    /// 角速度(rad/s)
    Float32_t yaw_rate;
    /// 角速度变化速率(rad/s^2)
    Float32_t yaw_rate_chg_rate;
    /// 角速度变化速率的变化速率(rad/s^3)
    Float32_t yaw_rate_d_chg_rate;
  } cur_pos;

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
      /// 角度误差变化速率(rad/s^2)
      Float32_t yaw_err_chg_rate;
    } samples[2];
  } lat_err;

  /// 轨迹规划生成的轨迹类型
  Int32_t planning_trj_status;
  /// 车道中心线的曲率
  Float32_t ref_trj_curvature;
  /// 目标轨迹
  LatCtlTrajectory_t tar_trj;

  /// 底盘信息
  struct {
    /// 转向机是否处于自动状态
    Int8_t eps_auto;
    /// 方向盘角度 (rad)
    Float32_t steering_wheel_angle;
    /// 总质量 (kg)
    Float32_t gross_weight;
  } chassis;
};

/**
 * @struct LatCtlPidConf_t
 * @brief 横向控制器(PID)的配置参数
 */
typedef struct _LatCtlPidConf_t LatCtlPidConf_t;
struct _LatCtlPidConf_t {
  /// 控制器运行周期(s)
  Float32_t ctl_period;

  /// 轨迹前馈
  /// 最小预瞄距离(current, near, far)
  Float32_t min_leading_len;
  Float32_t min_near_leading_len;
  Float32_t min_far_leading_len;
  /// 通过速度修正预瞄距离(Key table & t-gap table[current, near, far])
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_crt_lead_by_spd;
  Float32_t* tgap_tab_crt_lead_by_spd;
  Float32_t* near_tgap_tab_crt_lead_by_spd;
  Float32_t* far_tgap_tab_crt_lead_by_spd;
  /// 通过曲率修正预瞄距离(Key table & gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_crt_lead_by_curvature;
  Float32_t* gain_tab_crt_lead_by_curvature;

  /// 横向误差补偿
  /// 最大横向误差补偿值(rad) P值
  Float32_t max_lat_err_feed_value_p;
  /// 最大横向误差补偿值(rad) I值
  Float32_t max_lat_err_feed_value_i;
  /// 最大横向误差补偿值(rad) D值
  Float32_t max_lat_err_feed_value_d;
  /// 最大横向误差补偿值(rad)
  Float32_t max_lat_err_feed_value;
  /// 最大航向角误差补偿值(rad) P值
  Float32_t max_yaw_err_feed_value_p;
  /// 最大航向角误差补偿值(rad) I值
  Float32_t max_yaw_err_feed_value_i;
  /// 最大航向角误差补偿值(rad) D值
  Float32_t max_yaw_err_feed_value_d;
  /// 最大航向角误差补偿值(rad)
  Float32_t max_yaw_err_feed_value;
  /// 全局增益
  Float32_t p_global_gain_lat_err_feed;
  Float32_t i_global_gain_lat_err_feed;
  Float32_t d_global_gain_lat_err_feed;
  /// 通过速度修正横向误差补偿值(Key table & p-gain table & i-gain table & d-gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_crt_lat_err_feed_by_spd;
  Float32_t* p_tab_crt_lat_err_feed_by_spd;
  Float32_t* i_tab_crt_lat_err_feed_by_spd;
  Float32_t* d_tab_crt_lat_err_feed_by_spd;
  /// 通过重量修正横向误差补偿(Key table & p-gain table & i-gain table & d-gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_crt_lat_err_feed_by_weight;
  Float32_t* p_tab_crt_lat_err_feed_by_weight;
  Float32_t* i_tab_crt_lat_err_feed_by_weight;
  Float32_t* d_tab_crt_lat_err_feed_by_weight;
  /// 横向误差补偿Fuzzy Table
  /// 误差等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE-1
  Float32_t* tab_lat_err_level;
  /// 误差变化率等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE-1
  Float32_t* tab_lat_err_spd_level;
  /// 横向误差补偿系数表
  /// 数组元素个数必须等于 LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE*LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE
  Float32_t* p_gain_tab_lat_err_feed;
  Float32_t* i_gain_tab_lat_err_feed;
  Float32_t* d_gain_tab_lat_err_feed;
  /// 航向角误差补偿Fuzzy Table
  /// 误差等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE-1
  Float32_t* tab_yaw_err_level;
  /// 误差变化率等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE-1
  Float32_t* tab_yaw_err_spd_level;
  /* k001 pengc 2023-11-07 (begin) */
  /// 横向误差补偿Fuzzy Table
  /// 误差等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE-1
  Float32_t* tab_lat_spd_err_level;
  /// 误差变化率等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE-1
  Float32_t* tab_lat_acc_level;
  /// 横向误差补偿系数表
  /// 数组元素个数必须等于 LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE*LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE
  Float32_t* p_gain_tab_lat_spd_feed;
  Float32_t* i_gain_tab_lat_spd_feed;
  Float32_t* d_gain_tab_lat_spd_feed;
  /* k001 pengc 2023-11-07 (end) */
  /// 航向角误差补偿系数表
  /// 数组元素个数必须等于 LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE*LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE
  Float32_t* p_gain_tab_yaw_err_feed;
  Float32_t* i_gain_tab_yaw_err_feed;
  Float32_t* d_gain_tab_yaw_err_feed;
  /// 不同横向距离上，横向误差变化率的限制值
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_lmt_lat_err_spd;
  Float32_t* gain_tab_lmt_lat_err_spd;
  Float32_t* base_val_tab_lmt_lat_err_spd_close;
  Float32_t* base_val_tab_lmt_lat_err_spd_far_away;

  /// 角速度误差补偿
  /// 最大角速度误差补偿值(rad) P值
  Float32_t max_yaw_rate_err_feed_value_p;
  /// 最大角速度误差补偿值(rad) I值
  Float32_t max_yaw_rate_err_feed_value_i;
  /// 最大角速度误差补偿值(rad) D值
  Float32_t max_yaw_rate_err_feed_value_d;
  /// 最大角速度误差补偿值(rad)
  Float32_t max_yaw_rate_err_feed_value;
  /// 全局增益
  Float32_t p_global_gain_yaw_rate_err_feed;
  Float32_t i_global_gain_yaw_rate_err_feed;
  Float32_t d_global_gain_yaw_rate_err_feed;
  /// 通过速度修正横向误差补偿值(Key table & p-gain table & i-gain table & d-gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_crt_yaw_rate_err_feed_by_spd;
  Float32_t* p_tab_crt_yaw_rate_err_feed_by_spd;
  Float32_t* i_tab_crt_yaw_rate_err_feed_by_spd;
  Float32_t* d_tab_crt_yaw_rate_err_feed_by_spd;
  /// 通过重量修正横向误差补偿(Key table & p-gain table & i-gain table & d-gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_crt_yaw_rate_err_feed_by_weight;
  Float32_t* p_tab_crt_yaw_rate_err_feed_by_weight;
  Float32_t* i_tab_crt_yaw_rate_err_feed_by_weight;
  Float32_t* d_tab_crt_yaw_rate_err_feed_by_weight;
  /// 角速度误差补偿Fuzzy Table
  /// 角速度误差等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE-1
  Float32_t* tab_yaw_rate_err_level;
  /// 角速度变化率等级表
  /// 数组元素个数必须等于 LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE-1
  Float32_t* tab_yaw_rate_spd_level;
  /// 角速度误差补偿系数表
  /// 数组元素个数必须等于 LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE*LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE
  Float32_t* p_gain_tab_yaw_rate_err_feed;
  Float32_t* i_gain_tab_yaw_rate_err_feed;
  Float32_t* d_gain_tab_yaw_rate_err_feed;

  /// 车辆动态特性补偿
  /// 最大动力学补值(rad)
  Float32_t max_dynamic_feed_value;
  /// 通过速度修正动态特性补偿(Key table & gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_dynamic_feed_by_spd;
  Float32_t* gain_tab_dynamic_feed_by_spd;

  /// 方向盘角度增益
  /// 通过方向盘角度修正角度增益(Key table & gain table)
  /// 数组元素个数必须等于 LAT_CTL_PID_LERP_TABLE_SIZE
  Float32_t* key_tab_steering_feed_by_angle;
  Float32_t* gain_tab_steering_feed_by_angle;
};

/**
 * @struct LatCtlAnalyseRet_t
 * @brief 横向控制分析的结果
 */
typedef struct _LatCtlAnalyseRet_t LatCtlAnalyseRet_t;
struct _LatCtlAnalyseRet_t {
  /// 方向盘角度零位偏移 (rad)
  Float32_t str_wel_ang_offset;
  Float32_t max_str_wel_ang_offset;
  /// 方向盘角度间隙 (rad)
  Float32_t str_wel_ang_gap;
  Float32_t max_str_wel_ang_gap;

  /// 角速度变化率 (rad/s^2) [底盘反馈的]
  Float32_t d1_yaw_rate_cha;
  /// 角速度变化率 (rad/s^2) [方向盘反向计算的]
  Float32_t d1_yaw_rate_str;

  /// Frequency domain
  /// 修正后的方向盘角度 (rad) [减去了零位偏移]
  Float32_t fd_low_mag_crt_str_wel_ang;
  Float32_t fd_high_mag_crt_str_wel_ang;
  /// 期望的稳态方向盘角度 (rad)
  Float32_t fd_low_mag_exp_steady_str_wel_ang;
  Float32_t fd_high_mag_exp_steady_str_wel_ang;
  /// 横向控制的目标角速度 (rad/s)
  Float32_t fd_low_mag_yaw_rate_tar;
  Float32_t fd_high_mag_yaw_rate_tar;
  /// 角速度 (rad/s) [底盘反馈的]
  Float32_t fd_low_mag_yaw_rate_cha;
  Float32_t fd_high_mag_yaw_rate_cha;

  /// 方向盘低频响应系数
  Float32_t fd_low_str_trans_ratio_list[LAT_CTL_ANA_TRANS_REL_ITEM_NUM];
  /// 方向盘低频补偿系数
  Float32_t fd_low_str_feed_ratio_list[LAT_CTL_ANA_TRANS_REL_ITEM_NUM];
};

/**
 * @struct LateralControlPidInfo_t
 * @brief 横向控制模块(PID)状态信息
 */
typedef struct _LateralControlPidInfo_t LateralControlPidInfo_t;
struct _LateralControlPidInfo_t {
  // 参考轨迹曲率信息
  struct {
    /// 曲率类型
    Int32_t curvature_type;
    /// 当前曲率
    Float32_t curr_curvature;
    /// 当前曲率变化率
    Float32_t curr_d1_curvature;
    /// 之前的曲率
    Float32_t prev_curvature;
    /// 之前曲率到当前曲率的时间差
    Float32_t prev_curvature_time_elapsed;
  } ref_trj_info;

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
    /// 平均横向误差
    Float32_t avg_lat_err;
    /// 横向误差变化率
    Float32_t lat_err_spd;
    /// 横向误差变化率的变化率
    Float32_t lat_err_acc;
    /// 目标横向误差变化率
    Float32_t tar_lat_spd;
    /// 横向误差变化率误差
    Float32_t lat_spd_err;
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

  /// 航向角误差补偿
  struct {
    /// 角度误差
    Float32_t yaw_err;
    /// 目标角度误差
    Float32_t tar_yaw_err;
    /// 修正后的角度误差
    Float32_t crt_yaw_err;
    /// 角度误差变化率
    Float32_t yaw_err_spd;
    /// 航向角误差等级
    Int32_t yaw_err_lv_idx;
    /// 航向角误差变化率等级
    Int32_t yaw_err_spd_lv_idx;
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
  } yaw_err_feed;

  /// 角速度误差补偿
  struct {
    /// 角速度误差
    Float32_t yaw_rate_err;
    /// 目标角速度变化率
    Float32_t tar_yaw_rate_spd;
    /// 角速度变化率
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

  // 相关控制量的频谱
  struct {
    struct {
      Int8_t is_hunting;
      Float32_t hunting_energy;
      Float32_t fd_low_mag_tar_yaw_rate;
      Float32_t fd_high_mag_tar_yaw_rate;
      Float32_t fd_low_mag_cur_yaw_rate;
      Float32_t fd_high_mag_cur_yaw_rate;
      Int8_t hunting_fre_idxs[LAT_CTL_FREQUENCY_SPECTRUM_SIZE];
      Float32_t tar_yaw_rate_mag[LAT_CTL_FREQUENCY_SPECTRUM_SIZE];
      Float32_t cur_yaw_rate_mag[LAT_CTL_FREQUENCY_SPECTRUM_SIZE];
    } yaw_rate;
  } frequency_spectrum;

  /// 方向盘角度增益
  Float32_t steering_gain;

  /// 目标方向盘转角
  Float32_t tar_steering_angle;
  /// 平滑后的目标方向盘转角
  Float32_t tar_steering_angle_smooth;

  /// 性能分析
  struct {
    LatCtlAnalyseRet_t ana_ret;
    Float32_t ctl_gain;
  } perf_ana;
};

/**
 * @struct LateralControlInfo_t
 * @brief 横向控制模块状态信息
 */

#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
typedef struct lateral_control_debug{
  double PrevLatDist;
  double PrevHeading;
  double PrevCurve;
  //double PrevLatDist_Path;
  //double PrevHeading_Path;
  //double PrevCurve_Path;
  double LATC_agFFSteerDeg;
  double LATC_agFBSteerDeg;
  double TarAngleSteer;
  double CurSteerAngle;
  double CurVehYawRate;

  double CalculateLatDist;
  double CalculateLatHeading;
  double CalculateLatCurve;
  double ChangeLaneFlag;

  double LATC_eleK1;//K1FormLqr;
  double LATC_eleK2;//K2FormLqr;
  double LATC_eleK3;//K3FormLqr;
  double LATC_eleK4;//K4FormLqr;

  double LATC_agFrTyreFBEle1;
  double LATC_agFrTyreFBEle2;
  double LATC_agFrTyreFBEle3;
  double LATC_agFrTyreFBEle4;

  // 状态信息
  int32_t ADmode;
  int32_t CoST_stADMoEna;
  int32_t LATC_swtPathSel_C;
  double LATC_Q1Wgt4LQR_C;
  double LATC_Q2Wgt4LQR_C;
  double LATC_Q3Wgt4LQR_C;
  double LATC_Q4Wgt4LQR_C;
  int32_t LKA_swtAgSteerOfstSel_C;
  double LATC_agSteerOfst_C;
  double LATC_agSteerZeroOfst;
  double DE_mVehMass;
  int32_t DE_stTrlr ;
  int32_t LQR_IntNum;
  double LC_lPrevCv4FF;
  double LC_lPrevHeading4FB;
  double LC_lPrevLatDist4FB;
 double LatCv4LQR;
 double LatDist4LQR;
 double LatHeading4LQR;
 double LATC_vDistErrFlt4LQR;
 double LATC_vLatSpdFlt4LQR;
 double LATC_agHeadFlt4LQR;
 double LATC_wYRErrFlt4LQR;


 }lateral_control_debug_t;

#endif

typedef struct _LateralControlInfo_t LateralControlInfo_t;
struct _LateralControlInfo_t {
  /// 消息头
  MsgHead_t msg_head;

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
  /// 角速度变化率的变化率
  Float32_t yaw_rate_d_chg_rate;
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
  LateralControlPidInfo_t lat_ctl_pid_info;
  
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
  lateral_control_debug_t lat_debug;
#endif
  /// 目标方向盘角度
  Float32_t target_steering_wheel_angle;
  /// 目标方向盘转速
  Float32_t target_steering_wheel_angle_speed;
};

/**
 * @struct SteeringControlInfo_t
 * @brief 方向盘控制模块状态信息
 */
typedef struct _SteeringControlInfo_t SteeringControlInfo_t;
struct _SteeringControlInfo_t {
  /// 目标方向盘转速
  Float32_t target_steering_speed;
};


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_LATERAL_CONTROL_C_H_
