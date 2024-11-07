/** @copyright Copyright (c) 2018-2024 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lateral_control_analyse.h
 * @brief      横向控制分析
 * @details    实现横向控制分析
 *
 * @author     pengc
 * @date       2024.02.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_CONTROL_LATERAL_CONTROL_ANALYSE_C_H_
#define PHOENIX_CONTROL_LATERAL_CONTROL_ANALYSE_C_H_

#include "utils/macros.h"
#include "math/matrix_c.h"
#include "container/ring_buffer_c.h"
#include "utils/com_timer_c.h"
#include "lateral_control_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @enum
 * @brief 统计数据的数量
 */
enum { LAT_CTL_ANA_STATISTIC_DATA_SIZE = 200 };

/**
 * @enum
 * @brief 估计方向盘角度间隙窗口的大小
 */
enum { LAT_CTL_ANA_STR_GAP_WND_SIZE = 160 };

/**
 * @enum
 * @brief FFT 缓存大小
 */
enum { LAT_CTL_ANA_FFT_DATA_SIZE = 160 };
enum { LAT_CTL_ANA_FFT_SIZE = 256 };

/**
 * @enum
 * @brief 最大生命值
 */
enum { LAT_CTL_ANA_TRANS_REL_MAX_ITEM_LIFE = 100 };


/**
 * @struct LatCtlAnalyseConf_t
 * @brief 横向控制分析器的配置参数
 */
typedef struct _LatCtlAnalyseConf_t LatCtlAnalyseConf_t;
struct _LatCtlAnalyseConf_t {
  // 样本间隔时间
  Float32_t sample_interval;
  // 角速度响应延迟偏移量
  Int32_t yaw_rate_res_delay_offset;
};

/**
 * @struct LatCtlAnalyseConf_t
 * @brief 横向控制分析器的输入数据
 */
typedef struct _LatCtlAnalyseDataSource_t LatCtlAnalyseDataSource_t;
struct _LatCtlAnalyseDataSource_t {
  /// 时间戳
  Int64_t timestamp;

  /// 底盘信息
  struct {
    /// 转向机是否处于自动状态
    Int8_t eps_auto;
    /// 方向盘角度 (rad)
    Float32_t steering_wheel_angle;
    /// 角速度 (rad/s) [底盘反馈的]
    Float32_t yaw_rate_cha;
    Float32_t yaw_rate_chg_rate;
    /// 车速(m/s)
    Float32_t veh_speed;
    /// 总质量 (kg)
    Float32_t gross_weight;
  } chassis;

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

  /// 轨迹信息
  struct {
    /// 轨迹规划生成的轨迹类型
    Int32_t planning_trj_status;
    /// 车道中心线的曲率
    Float32_t ref_trj_curvature;

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
    } ref_trj;
  } trj_info;

  /// 横向控制信息
  struct {
    /// 横向控制的目标角速度 (rad/s)
    Float32_t yaw_rate_tar;
    /// 转向不足补偿值 (rad)
    Float32_t understeer_feed;
  } ctl_info;
};

/**
 * @struct LatCtlFFTInstance_t
 * @brief 横向控制分析器FFT的成员变量
 */
typedef struct _LatCtlAnalyseFFTInstance_t LatCtlAnalyseFFTInstance_t;
struct _LatCtlAnalyseFFTInstance_t {
  Float32_t rex[LAT_CTL_ANA_FFT_SIZE];
  Float32_t imx[LAT_CTL_ANA_FFT_SIZE];
  Float32_t mag[LAT_CTL_ANA_FFT_SIZE/2];
  Float32_t pha[LAT_CTL_ANA_FFT_SIZE/2];
};

/**
 * @struct LatCtlAnalyseTransRelItem_t
 * @brief 输入输出响应关系数据项
 */
typedef struct _LatCtlAnalyseTransRelItem_t LatCtlAnalyseTransRelItem_t;
struct _LatCtlAnalyseTransRelItem_t {
  Float32_t key;
  Int32_t life;
  Int32_t total_count;
  Int32_t count;
  Float32_t avg_y;
  Float32_t max_y;
  Float32_t trans_ratio;
  Float32_t feed_ratio;
};

/**
 * @struct LatCtlPidInstance_t
 * @brief 横向控制分析器的统计数据项
 */
typedef struct _LatCtlAnalyseStatisticDataItem_t LatCtlAnalyseStatisticDataItem_t;
struct _LatCtlAnalyseStatisticDataItem_t {
  /// 时间戳
  Int64_t timestamp;

  /// 转向机是否处于自动模式
  Int8_t eps_auto;
  /// 车速(m/s)
  Float32_t veh_speed;

  /// 方向盘角度 (rad)
  Float32_t str_wel_ang;
  /// 修正后的方向盘角度 (rad) [减去了零位偏移]
  Float32_t crt_str_wel_ang;
  /// 期望的稳态方向盘角度 (rad)
  Float32_t exp_steady_str_wel_ang;
  /// 方向盘角度变化率 (rad/s)
  Float32_t raw_d1_str_wel_ang;
  Float32_t d1_str_wel_ang;
  /// 期望的稳态方向盘角度变化率 (rad/s)
  Float32_t raw_d1_exp_steady_str_wel_ang;
  Float32_t d1_exp_steady_str_wel_ang;
  /// 方向盘角度平均值 (rad) [指定的窗口内]
  Float32_t avg_str_wel_ang;
  /// 期望的稳态方向盘角度平均值 (rad) [指定的窗口内]
  Float32_t avg_exp_steady_str_wel_ang;
  /// 方向盘角度变化率积分值 (rad) [指定的窗口内]
  Float32_t wnd_itg_d1_str_wel_ang;
  /// 期望的稳态方向盘角度变化率积分值 (rad) [指定的窗口内]
  Float32_t wnd_itg_d1_exp_steady_str_wel_ang;

  /// 横向控制的目标角速度 (rad/s)
  Float32_t yaw_rate_tar;
  /// 角速度 (rad/s) [底盘反馈的]
  Float32_t yaw_rate_cha;
  /// 角速度 (rad/s) [方向盘反向计算的]
  Float32_t yaw_rate_str;
  /// 角速度变化率 (rad/s^2) [底盘反馈的]
  Float32_t raw_d1_yaw_rate_cha;
  Float32_t d1_yaw_rate_cha;
  /// 角速度变化率 (rad/s^2) [方向盘反向计算的]
  Float32_t raw_d1_yaw_rate_str;
  Float32_t d1_yaw_rate_str;
  /// 角速度变化率积分值 (rad/s) [底盘反馈的, 指定的窗口内]
  Float32_t wnd_itg_d1_yaw_rate_cha;
  /// 角速度变化率积分值 (rad/s) [方向盘反向计算的, 指定的窗口内]
  Float32_t wnd_itg_d1_yaw_rate_str;

  /// 方向盘角度零位偏移 (rad)
  Float32_t str_wel_ang_offset;
  /// 方向盘角度间隙 (rad)
  Float32_t str_wel_ang_gap;

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
};

/**
 * @struct LatCtlPidInstance_t
 * @brief 横向控制分析器的成员变量
 */
typedef struct _LatCtlAnalyseInstance_t LatCtlAnalyseInstance_t;
struct _LatCtlAnalyseInstance_t {
  /// 时间戳
  Int64_t timestamp;

  /// 配置项
  LatCtlAnalyseConf_t conf;

    /// 定时器
  struct {
    // 刚进入自动模式
    ComTimer_t eps_auto;
  } timers;

  /// 统计数据
  struct {
    LatCtlAnalyseStatisticDataItem_t seq_data_buf[LAT_CTL_ANA_STATISTIC_DATA_SIZE];
    RingBuffer_t seq_data;

    /// 方向盘角度零位偏移 (rad)
    // Float32_t avg_str_wel_ang_offset;
    Float32_t max_str_wel_ang_offset;
    /// 方向盘角度间隙 (rad)
    // Float32_t avg_str_wel_ang_gap;
    Float32_t max_str_wel_ang_gap;
  } statistic;

  /// 估计方向盘角度间隙
  struct {
    Float32_t wnd_itg_d1_str_wel_ang;
    Float32_t max_str_wel_ang_gap_in_wnd;
    struct {
      Float32_t data_buf[LAT_CTL_ANA_STR_GAP_WND_SIZE];
      RingBuffer_t ring_buf;
    } queue;
  } est_str_gap;

  /// 估计频域内输入输出的响应关系
  struct {
    /// 修正后的方向盘角度 (rad) [减去了零位偏移]
    LatCtlAnalyseFFTInstance_t crt_str_wel_ang;
    /// 期望的稳态方向盘角度 (rad)
    LatCtlAnalyseFFTInstance_t exp_steady_str_wel_ang;
    /// 横向控制的目标角速度 (rad/s)
    LatCtlAnalyseFFTInstance_t yaw_rate_tar;
    /// 角速度 (rad/s) [底盘反馈的]
    LatCtlAnalyseFFTInstance_t yaw_rate_cha;
  } est_res_in_fre;

  /// 估计输入输出的响应关系
  struct {
    LatCtlAnalyseTransRelItem_t tab_trans_rel[LAT_CTL_ANA_TRANS_REL_ITEM_NUM];
  } est_trans_rel;

  // 蛇形抑制
  struct {
    Float32_t hunting_ratio_hig_fre;
    Float32_t hunting_ratio_d1_yaw_rate;
  } hunting_suppress;
};

/**
 * @brief 初始化横向控制分析器
 * @param[in] ins 类实例
 */
void Phoenix_LatCtlAnalyse_Initialize(
    LatCtlAnalyseInstance_t* const ins);

/**
 * @brief 重置横向控制分析器的内部状态
 * @param[in] ins 类实例
 */
void Phoenix_LatCtlAnalyse_Reset(
    LatCtlAnalyseInstance_t* const ins);

/**
 * @brief 配置横向控制分析器的参数
 * @param[in] ins 类实例
 * @param[in] conf 配置参数
 */
void Phoenix_LatCtlAnalyse_Configurate(
    LatCtlAnalyseInstance_t* const ins, 
    const LatCtlAnalyseConf_t* conf);

/**
 * @brief 更新横向控制分析器
 * @param[in] ins 类实例
 * @param[in] data_source 控制器需要的数据
 */
void Phoenix_LatCtlAnalyse_Update(
    LatCtlAnalyseInstance_t* const ins,
    const LatCtlAnalyseDataSource_t* data_source);

/**
 * @brief 获取横向控制分析的结果
 * @param[in] ins 类实例
 * @param[out] ana_ret 分析的结果
 */
void Phoenix_LatCtlAnalyse_GetAnaRet(
    LatCtlAnalyseInstance_t* const ins,
    LatCtlAnalyseRet_t* const ana_ret);

/**
 * @brief 获取方向盘低频响应补偿系数
 * @param[in] ins 类实例
 * @param[in] yaw_rate 角速度 (rad/s)
 * @param[in] veh_spd 车速(m/s)
 */
Float32_t Phoenix_LatCtlAnalyse_GetStrLowFeqResFeedRatioByYawRate(
    LatCtlAnalyseInstance_t* const ins,
    Float32_t yaw_rate, Float32_t veh_spd);

/* @brief 获取方向盘低频响应补偿系数
 * @param[in] ins 类实例
 * @param[in] str_ang 方向盘角度 (rad)
 * @param[in] veh_spd 车速(m/s)
 */
Float32_t Phoenix_LatCtlAnalyse_GetStrLowFeqResFeedRatioByStrAng(
    LatCtlAnalyseInstance_t* const ins,
    Float32_t str_ang, Float32_t veh_spd);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_LATERAL_CONTROL_ANALYSE_C_H_

