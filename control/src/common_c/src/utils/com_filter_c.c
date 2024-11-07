/** @copyright Copyright (c) 2018-2024 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       com_filter.c
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

#include "utils/com_filter_c.h"

#include "utils/log_c.h"


/*
 * @brief 滑动窗口平均滤波
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t Phoenix_Com_Filter_MovingAverage_f(
    Int32_t wnd_size, Int32_t data_num, Float32_t prev_ret, Float32_t new_value, Float32_t old_value) {
  Float32_t filter_ret = 0.0F;
  Float32_t data_num_f = data_num;

  if (data_num < wnd_size) {
    filter_ret = (data_num_f / (data_num_f+1.0F)) * prev_ret + (1.0F / (data_num_f+1.0F)) * new_value;
  } else {
    filter_ret = prev_ret + (new_value - old_value) / wnd_size;
  }

  return (filter_ret);
}


