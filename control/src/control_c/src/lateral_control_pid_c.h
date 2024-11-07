/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lateral_control_pid.h
 * @brief      横向控制器(PID)
 * @details    实现横向控制器(PID)
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

#ifndef PHOENIX_CONTROL_LATERAL_CONTROL_PID_C_H_
#define PHOENIX_CONTROL_LATERAL_CONTROL_PID_C_H_

#include "utils/macros.h"
#include "math/matrix_c.h"
#include "container/ring_buffer_c.h"
#include "utils/com_timer_c.h"
#include "utils/fuzzy_pid_c.h"
#include "lateral_control_c.h"
#include "msg_planning_c.h"
#include "lateral_control_analyse_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @enum
 * @brief 轨迹分析曲率列表的尺寸
 */
enum { LAT_CTL_REF_TRJ_CURVATURE_QUEUE_SIZE = 2*20 };

/**
 * @enum
 * @brief 轨迹前馈值列表的尺寸
 */
enum { MAX_TRJ_FEEDFORWARD_VALUE_LIST_SIZE = 32 };

/**
 * @enum
 * @brief 方向盘角度列表的尺寸
 */
enum { MAX_STEERING_WHEEL_ANGLE_LIST_SIZE = 8 };

/**
 * @enum
 * @brief 角速度误差补偿列表的尺寸
 */
enum { MAX_YAW_RATE_FEED_VALUE_LIST_SIZE = 5 };

/**
 * @enum
 * @brief FFT 缓存大小
 */
// enum { LAT_CTL_FFT_DATA_SIZE = 60 };
enum { LAT_CTL_FFT_DATA_SIZE = 100 };
enum { LAT_CTL_FFT_SIZE = 256 };


/**
 * @enum
 * @brief 横向误差平均值窗口尺寸
 */
enum { LAT_CTL_AVG_LAT_ERR_LIST_SIZE = 4*20 };

/**
 * @enum
 * @brief 横向误差2阶导滤波器窗口尺寸
 */
enum { LAT_CTL_D2_LAT_ERR_FILTER_WND_SIZE = 5 };

/**
 * @enum
 * @brief 横摆角速度误差1阶导滤波器窗口尺寸
 */
enum { LAT_CTL_D1_YAW_RATE_FILTER_WND_SIZE = 5 };

/**
 * @enum
 * @brief 横向误差速率补偿积分窗口尺寸
 */
enum { LAT_CTL_LAT_SPD_FEED_ITG_WND_SIZE = 20 };


/**
 * @struct LatCtlRCLowPassFilterInstance_t
 * @brief 横向控制RC滤波器
 */
typedef struct _LatCtlRCLowPassFilterInstance_t LatCtlRCLowPassFilterInstance_t;
struct _LatCtlRCLowPassFilterInstance_t {
  Int32_t delay_setting;
  Float32_t tar_decay_ratio;
  Float32_t cur_decay_ratio;
  Float32_t output_value;
};

/**
 * @struct LatCtlFFTInstance_t
 * @brief 横向控制FFT的成员变量
 */
typedef struct _LatCtlFFTInstance_t LatCtlFFTInstance_t;
struct _LatCtlFFTInstance_t {
  Float32_t data_buff[LAT_CTL_FFT_DATA_SIZE];
  RingBuffer_t ring_buff;
  Float32_t rex[LAT_CTL_FFT_SIZE];
  Float32_t imx[LAT_CTL_FFT_SIZE];
  Float32_t mag[LAT_CTL_FFT_SIZE/2];
  Float32_t pha[LAT_CTL_FFT_SIZE/2];
};

/**
 * @struct LatCtlLatAccInfo_t
 * @brief 定义计算横向加速度的结构体
 */
typedef struct _LatCtlLatAccInfo_t LatCtlLatAccInfo_t;
struct _LatCtlLatAccInfo_t {
  Int8_t valid;          // 标识数据是否有效
  Int64_t timestamp;     // 时间戳
  Float32_t speed;       // 当前速度
  Float32_t prev_acc;    // 上一次的加速度
  Float32_t acc_list[3]; // 加速度列表，存储三个加速度值
};

/**
 * @struct LatCtlLatAccInfo_t
 * @brief 定义计算轨迹曲率变化律的结构体
 */
typedef struct _LatCtlTrjD1CurvatureInfo_t LatCtlTrjD1CurvatureInfo_t;
struct _LatCtlTrjD1CurvatureInfo_t {
  Int8_t valid;                    // 标识数据是否有效
  Int64_t timestamp;               // 时间戳
  Float32_t curvature;             // 当前曲率
  Float32_t prev_d1_curvature;     // 上一次的曲率变化律
  Float32_t d1_curvature_list[3];  // 曲率变化率列表
};

/**
 * @struct LatCtlPidInstance_t
 * @brief 横向控制器(PID)的成员变量
 */
typedef struct _LatCtlPidInstance_t LatCtlPidInstance_t;
struct _LatCtlPidInstance_t {
  /// 控制器参数
  struct {
    /// 控制器运行周期(s)
    Float32_t ctl_period;

    /// 轨迹前馈
    /// 最小预瞄距离(current, near, far)
    Float32_t min_leading_len;
    Float32_t min_near_leading_len;
    Float32_t min_far_leading_len;
    /// 通过速度修正预瞄距离(Key table & t-gap table[current, near, far])
    Float32_t key_tab_crt_lead_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t tgap_tab_crt_lead_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t near_tgap_tab_crt_lead_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t far_tgap_tab_crt_lead_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    /// 通过曲率修正预瞄距离(Key table & gain table)
    Float32_t key_tab_crt_lead_by_curvature[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t gain_tab_crt_lead_by_curvature[LAT_CTL_PID_LERP_TABLE_SIZE];

    /// 横向误差补偿
    /// 最大横向误差补偿值 P值
    Float32_t max_lat_err_feed_value_p;
    /// 最大横向误差补偿值 I值
    Float32_t max_lat_err_feed_value_i;
    /// 最大横向误差补偿值 D值
    Float32_t max_lat_err_feed_value_d;
    /// 最大横向误差补偿值
    Float32_t max_lat_err_feed_value;
    /// 最大航向角误差补偿值 P值
    Float32_t max_yaw_err_feed_value_p;
    /// 最大航向角误差补偿值 I值
    Float32_t max_yaw_err_feed_value_i;
    /// 最大航向角误差补偿值 D值
    Float32_t max_yaw_err_feed_value_d;
    /// 最大航向角误差补偿值
    Float32_t max_yaw_err_feed_value;
    /// 全局增益
    Float32_t p_global_gain_lat_err_feed;
    Float32_t i_global_gain_lat_err_feed;
    Float32_t d_global_gain_lat_err_feed;
    /// 通过速度修正横向误差补偿值(Key table & p-gain table & i-gain table & d-gain table)
    Float32_t key_tab_crt_lat_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t p_tab_crt_lat_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t i_tab_crt_lat_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t d_tab_crt_lat_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    /// 通过重量修正横向误差补偿(Key table & p-gain table & i-gain table & d-gain table)
    Float32_t key_tab_crt_lat_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t p_tab_crt_lat_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t i_tab_crt_lat_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t d_tab_crt_lat_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    /// 横向误差补偿Fuzzy Table
    /// 误差等级表
    Float32_t tab_buf_lat_err_level[LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_lat_err_level;
    /// 误差变化率等级表
    Float32_t tab_buf_lat_err_spd_level[LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_lat_err_spd_level;
    /// 横向误差补偿系数表
    Float32_t p_gain_tab_buf_lat_err_feed[LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE];
    Matrix_t p_gain_tab_lat_err_feed;
    Float32_t i_gain_tab_buf_lat_err_feed[LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE];
    Matrix_t i_gain_tab_lat_err_feed;
    Float32_t d_gain_tab_buf_lat_err_feed[LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_LAT_ERR_FEED_TABLE_SIZE];
    Matrix_t d_gain_tab_lat_err_feed;
    /* k001 pengc 2023-11-07 (begin) */
    /// 横向误差速率补偿Fuzzy Table
    /// 误差速率等级表
    Float32_t tab_buf_lat_spd_err_level[LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE-1];
    Matrix_t tab_lat_spd_err_level;
    /// 误差速率变化率等级表
    Float32_t tab_buf_lat_acc_level[LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE-1];
    Matrix_t tab_lat_acc_level;
    /// 横向误差速率补偿系数表
    Float32_t p_gain_tab_buf_lat_spd_feed[LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE*
        LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE];
    Matrix_t p_gain_tab_lat_spd_feed;
    Float32_t i_gain_tab_buf_lat_spd_feed[LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE*
        LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE];
    Matrix_t i_gain_tab_lat_spd_feed;
    Float32_t d_gain_tab_buf_lat_spd_feed[LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE*
        LAT_CTL_PID_LAT_SPD_FEED_TABLE_SIZE];
    Matrix_t d_gain_tab_lat_spd_feed;
    /* k001 pengc 2023-11-07 (end) */
    /// 航向角误差补偿Fuzzy Table
    /// 误差等级表
    Float32_t tab_buf_yaw_err_level[LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_yaw_err_level;
    /// 误差变化率等级表
    Float32_t tab_buf_yaw_err_spd_level[LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_yaw_err_spd_level;
    /// 横向误差补偿系数表
    Float32_t p_gain_tab_buf_yaw_err_feed[LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE];
    Matrix_t p_gain_tab_yaw_err_feed;
    Float32_t i_gain_tab_buf_yaw_err_feed[LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE];
    Matrix_t i_gain_tab_yaw_err_feed;
    Float32_t d_gain_tab_buf_yaw_err_feed[LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_YAW_ERR_FEED_TABLE_SIZE];
    Matrix_t d_gain_tab_yaw_err_feed;
    /* k001 pengc 2023-11-07 (begin) */
    /// 不同横向距离上，横向误差变化率的目标值
    Float32_t key_tab_tar_lat_spd[LAT_CTL_PID_LERP_TABLE_SIZE];  // key
    Float32_t exp_tab_tar_lat_spd[LAT_CTL_PID_LERP_TABLE_SIZE];  // expected
    Float32_t dev_tab_tar_lat_spd[LAT_CTL_PID_LERP_TABLE_SIZE];  // standard deviation
    /* k001 pengc 2023-11-07 (end) */
    /* k001 pengc 2023-12-01 (begin) */
    // 通过速度修正角度误差的目标值
    Float32_t key_tab_crt_yaw_err_exp_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];   // key
    Float32_t ratio_tab_crt_yaw_err_exp_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE]; // ratio
    /// 不同横向距离上，角度误差的目标值
    Float32_t key_tab_tar_yaw_err[LAT_CTL_PID_LERP_TABLE_SIZE];  // key
    Float32_t exp_tab_tar_yaw_err[LAT_CTL_PID_LERP_TABLE_SIZE];  // expected
    Float32_t dev_tab_tar_yaw_err[LAT_CTL_PID_LERP_TABLE_SIZE];  // standard deviation
    /* k001 pengc 2023-12-01 (end) */
    /// 不同横向距离上，横向误差变化率的限制值
    Float32_t key_tab_lmt_lat_err_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t gain_tab_lmt_lat_err_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t base_val_tab_lmt_lat_err_spd_close[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t base_val_tab_lmt_lat_err_spd_far_away[LAT_CTL_PID_LERP_TABLE_SIZE];

    /// 角速度误差补偿
    /// 最大角速度误差补偿值 P值
    Float32_t max_yaw_rate_err_feed_value_p;
    /// 最大角速度误差补偿值 I值
    Float32_t max_yaw_rate_err_feed_value_i;
    /// 最大角速度误差补偿值 D值
    Float32_t max_yaw_rate_err_feed_value_d;
    /// 最大角速度误差补偿值
    Float32_t max_yaw_rate_err_feed_value;
    /// 全局增益
    Float32_t p_global_gain_yaw_rate_err_feed;
    Float32_t i_global_gain_yaw_rate_err_feed;
    Float32_t d_global_gain_yaw_rate_err_feed;
    /// 通过速度修正角速度误差补偿值(Key table & p-gain table & i-gain table & d-gain table)
    Float32_t key_tab_crt_yaw_rate_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t p_tab_crt_yaw_rate_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t i_tab_crt_yaw_rate_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t d_tab_crt_yaw_rate_err_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    /// 通过重量修正角速度误差补偿(Key table & p-gain table & i-gain table & d-gain table)
    Float32_t key_tab_crt_yaw_rate_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t p_tab_crt_yaw_rate_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t i_tab_crt_yaw_rate_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t d_tab_crt_yaw_rate_err_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    /// 角速度误差补偿Fuzzy Table
    /// 角速度误差等级表
    Float32_t tab_buf_yaw_rate_err_level[LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_yaw_rate_err_level;
    /// 角速度变化率等级表
    Float32_t tab_buf_yaw_rate_spd_level[LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_yaw_rate_spd_level;
    /// 角速度误差补偿系数表
    Float32_t p_gain_tab_buf_yaw_rate_err_feed[LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE];
    Matrix_t p_gain_tab_yaw_rate_err_feed;
    Float32_t i_gain_tab_buf_yaw_rate_err_feed[LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE];
    Matrix_t i_gain_tab_yaw_rate_err_feed;
    Float32_t d_gain_tab_buf_yaw_rate_err_feed[LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE*
        LAT_CTL_PID_YAW_RATE_ERR_FEED_TABLE_SIZE];
    Matrix_t d_gain_tab_yaw_rate_err_feed;
    /* k001 pengc 2023-12-01 (begin) */
    /// 不同横向距离上，角速度误差的目标值
    Float32_t key_tab_tar_yaw_rate_err[LAT_CTL_PID_LERP_TABLE_SIZE];  // key
    Float32_t dev_tab_tar_yaw_rate_err[LAT_CTL_PID_LERP_TABLE_SIZE];  // standard deviation
    /* k001 pengc 2023-12-01 (end) */

    /// 车辆动态特性补偿
    /// 最大动力学补值
    Float32_t max_dynamic_feed_value;
    /// 通过速度修正动态特性补偿(Key table & gain table)
    Float32_t key_tab_dynamic_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t gain_tab_dynamic_feed_by_spd[LAT_CTL_PID_LERP_TABLE_SIZE];
    /// 通过重量修正动态特性补偿(Key table & gain table)
    Float32_t key_tab_dynamic_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t gain_tab_dynamic_feed_by_weight[LAT_CTL_PID_LERP_TABLE_SIZE];

    /// 方向盘角度增益
    /// 通过方向盘角度修正角度增益(Key table & gain table)
    Float32_t key_tab_steering_feed_by_angle[LAT_CTL_PID_LERP_TABLE_SIZE];
    Float32_t gain_tab_steering_feed_by_angle[LAT_CTL_PID_LERP_TABLE_SIZE];
  } param;

  /// 时间戳
  Int64_t timestamp;

  /// 状态
  struct {
    /// 转向机的状态(之前，当前)(是否处于自动状态)
    Int8_t prev_eps_auto;
    Int8_t curr_eps_auto;
    /// 规划模块轨迹的状态(之前, 当前)
    Int8_t prev_trj_stat;
    Int8_t curr_trj_stat;
    /// 综合的横向误差/角度误差及其变化率
    Float32_t synthetic_lat_err;
    Float32_t synthetic_lat_spd;
    Float32_t synthetic_yaw_err;
    Float32_t synthetic_yaw_spd;
  } status;

  /// 定时器
  struct {
    ComTimer_t is_in_changing_lane;
    // 平滑横向误差补偿
    ComTimer_t smt_lat_err_feed;
    // 平滑横向速度补偿
    ComTimer_t smt_lat_spd_feed;
    // 平滑横向角度补偿
    ComTimer_t smt_yaw_err_feed;
    // 平滑角速度补偿
    ComTimer_t smt_yaw_rate_err_feed;
  } timers;

  // 参考轨迹曲率信息
  struct {
    // 曲率类型
    Int32_t curvature_type;
    // 当前曲率
    Float32_t curr_curvature;
    // 当前曲率变化率
    Float32_t curr_d1_curvature;
    // 之前的曲率
    Float32_t prev_curvature;
    // 之前曲率到当前曲率的时间差
    Float32_t prev_curvature_time_elapsed;

    LatCtlTrjD1CurvatureInfo_t trj_curv_info;
    struct {
      Float32_t data_buf[LAT_CTL_REF_TRJ_CURVATURE_QUEUE_SIZE];
      RingBuffer_t ring_buf;
    } curvature_queue;
  } ref_trj_info;

  /// 轨迹前馈
  struct {
    struct {
      /// 期望值
      Float32_t expectation;
      /// 方差
      Float32_t covariance;
      /// 补偿值列表
      Float32_t feed_value[MAX_TRJ_FEEDFORWARD_VALUE_LIST_SIZE];
    } filter;
  } trj_feed;

  /// 横向误差补偿器
  struct {
    /// PID控制器
    FuzzyPID_t pid_controller;
    /// 比例项补偿值
    Float32_t p_value;
    /// 积分项补偿值
    Float32_t i_value;
    /// 微分项补偿值
    Float32_t d_value;
    /// 合计的补偿值
    Float32_t feed_value;
    /// 补偿值列表
    Float32_t feed_value_list[3];

    /// 滤波
    struct {
      Int8_t valid;
      Float32_t lat_err;
      Float32_t avg_lat_err;
      struct {
        Float32_t data_buf[LAT_CTL_AVG_LAT_ERR_LIST_SIZE];
        RingBuffer_t ring_buf;
      } lat_err_queue;
    } filter;
  } lat_err_feed;

  /* k001 pengc 2023-11-07 (begin) */
  /// 横向误差变化率补偿器
  struct {
    /// PID控制器
    FuzzyPID_t pid_controller;
    /// 比例项补偿值
    Float32_t p_value;
    /// 积分项补偿值
    Float32_t i_value;
    /// 微分项补偿值
    Float32_t d_value;
    /// 合计的补偿值
    Float32_t feed_value;
    /// 补偿值列表
    Float32_t feed_value_list[3];

    /// 误差列表
    Float32_t err_list[3];

    /// 滤波
    struct {
      Int8_t valid;
      Float32_t tar_lat_spd_exp;
      Float32_t cur_lat_spd;
      Float32_t cur_lat_acc;
      struct {
        Float32_t data_buf[LAT_CTL_D2_LAT_ERR_FILTER_WND_SIZE];
        RingBuffer_t ring_buf;
      } lat_acc_queue;

      // 计算横向加速度
      LatCtlLatAccInfo_t lat_acc_info;

      Float32_t sum_tar_p_value;
      struct {
        Float32_t data_buf[LAT_CTL_LAT_SPD_FEED_ITG_WND_SIZE];
        RingBuffer_t ring_buf;
      } tar_p_value_queue;
      Float32_t sum_cur_p_value;
      struct {
        Float32_t data_buf[LAT_CTL_LAT_SPD_FEED_ITG_WND_SIZE];
        RingBuffer_t ring_buf;
      } cur_p_value_queue;
    } filter;
  } lat_spd_feed;
  /* k001 pengc 2023-11-07 (end) */

    /// 航向角误差补偿器
  struct {
    /// PID控制器
    FuzzyPID_t pid_controller;
    /// 比例项补偿值
    Float32_t p_value;
    /// 积分项补偿值
    Float32_t i_value;
    /// 微分项补偿值
    Float32_t d_value;
    /// 合计的补偿值
    Float32_t feed_value;
    /// 补偿值列表
    Float32_t feed_value_list[3];

    /// 滤波
    struct {
      Int8_t valid;
      Float32_t tar_yaw_err_exp;
    } filter;
  } yaw_err_feed;

  /// 横摆角速度误差补偿器
  struct {
    /// PID控制器
    FuzzyPID_t pid_controller;
    /// 比例项补偿值
    Float32_t p_value;
    /// 积分项补偿值
    Float32_t i_value;
    /// 微分项补偿值
    Float32_t d_value;
    /// 合计的补偿值
    Float32_t feed_value;
    /// 补偿值列表
    Float32_t feed_value_list[MAX_YAW_RATE_FEED_VALUE_LIST_SIZE];

    /// 滤波
    struct {
      Int8_t valid;
      Float32_t tar_yaw_rate;
      Float32_t tar_yaw_rate_spd;
      Float32_t yaw_rate_err;
      
      struct {
        Float32_t data_buf[LAT_CTL_D1_YAW_RATE_FILTER_WND_SIZE];
        RingBuffer_t ring_buf;
      } tar_yaw_rate_spd_queue;
    } filter;
  } yaw_rate_err_feed;

  /// 横向反馈补偿
  struct {
    /// 比例项补偿值
    Float32_t pd_value;
    /// 积分项补偿值
    Float32_t i_value;
    /// 合计的补偿值
    Float32_t feed_value;

    /// 滤波
    struct {
      Int8_t valid;
      Float32_t tar_yaw_rate_spd;
      
      struct {
        Float32_t data_buf[LAT_CTL_D1_YAW_RATE_FILTER_WND_SIZE];
        RingBuffer_t ring_buf;
      } tar_yaw_rate_spd_queue;
    } filter;
  } lat_feedback;

  /// 动态特性补偿器
  struct {
    /// PID控制器
    FuzzyPID_t pid_controller;
    /// 比例项补偿值
    Float32_t p_value;
    /// 积分项补偿值
    Float32_t i_value;
    /// 微分项补偿值
    Float32_t d_value;
    /// 合计的补偿值
    Float32_t feed_value;
    /// 补偿值列表
    Float32_t feed_value_list[3];
  } dynamic_feedforward;

  /// 目标方向盘角度值滤波器
  struct {
    /// 期望值
    Float32_t expectation;
    /// 方差
    Float32_t covariance;
    /// 方向盘角度列表列表
    Float32_t angle_list[MAX_STEERING_WHEEL_ANGLE_LIST_SIZE];
  } steering_wheel_filter;

  /// 车辆蛇行检测
  struct {
    struct {
      Int8_t is_hunting;
      Float32_t hunting_energy;
      Float32_t hunting_gain;
      Float32_t fd_low_mag_tar_yaw_rate;
      Float32_t fd_high_mag_tar_yaw_rate;
      Float32_t fd_low_mag_cur_yaw_rate;
      Float32_t fd_high_mag_cur_yaw_rate;
      LatCtlFFTInstance_t fft_tar_yaw_rate;
      LatCtlFFTInstance_t fft_cur_yaw_rate;
    } yaw_rate;
  } hunting_detector;

  /// 性能分析
  struct {
    LatCtlAnalyseInstance_t analyser;
    LatCtlAnalyseRet_t ana_ret;
  } perf_ana;
  
};

/**
 * @brief 初始化横向控制器(PID)
 * @param[in] instance 成员变量
 */
void Phoenix_LatCtlPid_Initialize(
    LatCtlPidInstance_t* const instance);

/**
 * @brief 重置横向控制器(PID)的内部状态
 * @param[in] instance 成员变量
 */
void Phoenix_LatCtlPid_ResetLatCtlValue(
    LatCtlPidInstance_t* const instance,
    Float32_t tar_steering_angle);

/**
 * @brief 配置横向控制器(PID)的参数
 * @param[in] instance 成员变量
 * @param[in] conf 配置参数
 */
void Phoenix_LatCtlPid_Configurate(
    LatCtlPidInstance_t* const instance, const LatCtlPidConf_t* conf);

/**
 * @brief 计算横向控制量(目标方向盘转角)
 * @param[in] instance 成员变量
 * @param[in] data_source 控制器需要的数据
 * @param[out] tar_yaw_rate 目标角速度
 * @param[out] ctl_value 目标方向盘转角
 * @param[out] status_info 内部状态信息
 * @return 0 ~ 成功, others ~ 失败
 */
Int32_t Phoenix_LatCtlPid_CalcLatCtlValue(
    LatCtlPidInstance_t* const instance,
    const LatCtlDataSource_t* data_source,
    Float32_t* tar_yaw_rate, Float32_t* ctl_value,
    LateralControlPidInfo_t* const status_info);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_LATERAL_CONTROL_PID_C_H_

