/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       longitudinal_control_ft_auman.h
 * @brief      纵向控制器(FT-Auman)
 * @details    实现纵向控制器(FT-Auman)
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

#ifndef PHOENIX_CONTROL_QINEV_LONGITUDINAL_CONTROL_DF_D17_B2_C_H_
#define PHOENIX_CONTROL_QINEV_LONGITUDINAL_CONTROL_DF_D17_B2_C_H_

#include "longitudinal_control_c.h"

#include "math/matrix_c.h"
#include "utils/fuzzy_pid_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct LonCtlDfD17B2Instance_t
 * @brief 纵向控制器(FT-Auman)的成员变量
 */
typedef struct _LonCtlDfD17B2Instance_t LonCtlDfD17B2Instance_t;
struct _LonCtlDfD17B2Instance_t {
  /// 控制器参数
  struct {
    /// 根据当前车速，限制最大油门量(Key table & value table)
    Float32_t key_tab_lmt_acc_by_spd[LON_CTL_PID_LERP_TABLE_SIZE];
    Float32_t value_tab_lmt_acc_by_spd[LON_CTL_PID_LERP_TABLE_SIZE];

    /// 通过重量修正速度误差补偿(Key table & i-gain table)
    Float32_t key_tab_crt_spd_err_feed_by_weight[LON_CTL_PID_LERP_TABLE_SIZE];
    Float32_t i_tab_crt_spd_err_feed_by_weight[LON_CTL_PID_LERP_TABLE_SIZE];

    /// 速度误差补偿Fuzzy Table
    /// 速度误差等级表
    Float32_t tab_buf_spd_err_level[LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_spd_err_level;
    /// 速度变化率等级表
    Float32_t tab_buf_spd_chg_level[LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1];
    Matrix_t tab_spd_chg_level;
    /// 速度误差补偿系数表
    Float32_t i_gain_tab_buf_spd_err_feed[LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE*
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE];
    Matrix_t i_gain_tab_spd_err_feed;
  } param;

  /// 制动控制器
  struct {
    /// 速度误差
    Float32_t errs[2];
    /// 比例项补偿值
    Float32_t value_p;
    /// 积分项补偿值
    Float32_t value_i;
    /// 微分项补偿值
    Float32_t value_d;
  } brake;

  /// 加速控制器
  struct {
    /// PID控制器
    FuzzyPID_t pid_controller;
    /// 速度误差
    Float32_t errs[2];
    /// 比例项补偿值
    Float32_t value_p;
    /// 积分项补偿值
    Float32_t value_i;
    /// 微分项补偿值
    Float32_t value_d;
  } acc;
};


/**
 * @brief 初始化纵向控制器
 * @param[in] instance 成员变量
 */
void Phoenix_LonCtl_DfD17B2_Initialize(
    LonCtlDfD17B2Instance_t* const instance);

/**
 * @brief 重置纵向控制器内部状态(加速模块)
 * @param[in] instance 成员变量
 * @param[in] value 加速量重置值
 */
void Phoenix_LonCtl_DfD17B2_ResetAccValue(
    LonCtlDfD17B2Instance_t* const instance, Float32_t value);

/**
 * @brief 重置纵向控制器内部状态(制动模块)
 * @param[in] instance 成员变量
 * @param[in] value 制动量重置值
 */
void Phoenix_LonCtl_DfD17B2_ResetBrakeValue(
    LonCtlDfD17B2Instance_t* const instance, Float32_t value);

/**
 * @brief 计算纵向控制量
 * @param[in] instance 成员变量
 * @param[in] data_source 控制器需要的数据
 * @param[out] acc_value 目标加速量
 * @param[out] brake_value 目标制动量
 * @return 0 ~ 成功, others ~ 失败
 */
Int32_t Phoenix_LonCtl_DfD17B2_CalcLonCtlValue(
    LonCtlDfD17B2Instance_t* const instance,
    const LonCtlDataSource_t* data_source,
    Float32_t* acc_value,
    Float32_t* brake_value);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_QINEV_LONGITUDINAL_CONTROL_DF_D17_B2_C_H_


