/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_planning_settings.h
 * @brief      决策模块设置消息定义
 * @details    定义了决策模块设置消息类型
 *
 * @author     sip
 * @date       2021.02.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/02/19  <td>1.0      <td>sip       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_PLANNING_SETTINGS_H_
#define PHOENIX_AD_MSG_MSG_PLANNING_SETTINGS_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @struct PlanningSettings
 * @brief 界面设置信息
 */
struct PlanningSettings {
  /// ADAS功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t start_adas;

  /// LKA功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_lka;
  /// ACC功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_acc;
  /// AEB功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_aeb;

  /// 变道功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_alc;
  /// 智能限速功能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_isl;

  /// Navigation Guided Pilot: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_ngp;

  /// 降级使能:  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_fallback;

  /// 节油使能： 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  Int8_t enable_pcc;
  
  /// 目标车速[m/s]
  Int8_t target_velocity_valid;
  Float32_t target_velocity;
  /// 加速度[m/s^2]
  Int8_t target_acc_valid;
  Float32_t target_acc;
  /// 时距[s]
  Int8_t target_time_gap_valid;
  Float32_t target_time_gap;
  /// 降级[-]
  Int8_t target_fallback_level_valid;
  Int8_t target_fallback_level;
  /// 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  Int32_t changing_lane_req;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    start_adas = 0;

    enable_lka = 0;
    enable_acc = 0;
    enable_aeb = 0;

    enable_alc = 0;
    enable_isl = 0;

    enable_ngp = 0;
    enable_fallback = 0;
    enable_pcc = 0;
    target_velocity_valid = 0;
    target_velocity = 0.0F;

    target_acc_valid = 0;
    target_acc = 0.0F;
    target_time_gap_valid = 0;
    target_time_gap = 0.0F;
    target_fallback_level_valid = 0;
    target_fallback_level = 0;


    changing_lane_req = 0;
  }

  /**
   * @brief 构造函数
   */
  PlanningSettings() {
    Clear();
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_PLANNING_SETTINGS_H_


