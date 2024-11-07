/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       longitudinal_control.h
 * @brief      数据类型定义
 * @details    纵向控制所需的数据类型定义
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

#ifndef PHOENIX_CONTROL_LONGITUDINAL_CONTROL_C_H_
#define PHOENIX_CONTROL_LONGITUDINAL_CONTROL_C_H_

#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @enum
 * @brief PID纵向控制模块中线性插值表的尺寸
 */
enum { LON_CTL_PID_LERP_TABLE_SIZE = 25 };

/**
 * @enum
 * @brief PID纵向控制模块中速度误差补偿表的尺寸
 */
enum { LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE = 20 };


/**
 * @struct LonCtlDataSource
 * @brief 纵向控制算法所需数据
 */
typedef struct _LonCtlDataSource_t LonCtlDataSource_t;
struct _LonCtlDataSource_t {
  /// 时间戳
  Int64_t timestamp;

  /// 当前车速(m/s)
  Float32_t cur_v;
  /// 当前加速度(m/s^2)
  Float32_t cur_a;
  /// 目标车速(m/s)
  Float32_t tar_v;
  /// 目标加速度(m/s^2)
  Float32_t tar_a;

  /// 底盘信息
  struct {
    /// 总质量 (kg)
    Float32_t gross_weight;
  } chassis;
};


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_LONGITUDINAL_CONTROL_C_H_
