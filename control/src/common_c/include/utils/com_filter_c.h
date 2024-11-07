/** @copyright Copyright (c) 2018-2024 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       com_filter.h
 * @brief      常用滤波器
 * @details    定义了一些常用滤波器
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

#ifndef PHOENIX_COMMON_COM_FILTER_C_H_
#define PHOENIX_COMMON_COM_FILTER_C_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 滑动窗口平均滤波
 * @param[in] wnd_size 窗口尺寸
 * @param[in] data_num 之前滤波器已有数据的数量
 * @param[in] prev_ret 之前的滤波值
 * @param[in] new_value 需要添加的新值
 * @param[in] old_value 需要减去的旧值
 */
Float32_t Phoenix_Com_Filter_MovingAverage_f(
    Int32_t wnd_size, Int32_t data_num, Float32_t prev_ret, Float32_t new_value, Float32_t old_value);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_COM_FILTER_C_H_
