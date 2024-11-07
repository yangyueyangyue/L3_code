/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_event_reporting.h
 * @brief      事件报告
 * @details    定义了事件报告消息类型
 *
 * @author     pengc
 * @date       2021.06.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/06/15  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_EVENT_REPORTING_H_
#define PHOENIX_AD_MSG_MSG_EVENT_REPORTING_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @enum
 * @brief 事件ID
 */
enum {
  EVENT_REPORTING_ID_INVALID = 0,
  EVENT_REPORTING_ID_START_ROBOTIC,
  EVENT_REPORTING_ID_STOP_ROBOTIC,
  EVENT_REPORTING_ID_SENSOR_MSG_TIMEOUT,
  EVNET_REPORTING_ID_CHASSIS_MSG_TIMEOUT,
  EVNET_REPORTING_ID_PLANNING_TIMEOUT,
  EVENT_REPORTING_ID_CAMERA_LANE_LOST,
  EVENT_REPORTING_ID_AEB_WARNING,
  EVENT_REPORTING_ID_AEB_ACTION,
  EVENT_REPORTING_ID_UNCERTAIN_OBSTACLE,
  EVENT_REPORTING_ID_START_FOLLOWING_OBJ,
  EVENT_REPORTING_ID_STOP_FOLLOWING_OBJ,
  EVENT_REPORTING_ID_CHANGING_LANE_REQ,
  EVENT_REPORTING_ID_CHANGING_LANE_RSP
};

/**
 * @enum
 * @brief 变道事件类型
 */
enum {
  EVENT_REPORTING_CHANGING_LANE_TYPE_INVALID = 0,
  EVENT_REPORTING_CHANGING_LANE_TYPE_PREPARE_FOR_CHANGING_LEFT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_PREPARE_FOR_CHANGING_RIGHT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_START_CHANGING_LEFT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_START_CHANGING_RIGHT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_REFUSE_CHANGING_LEFT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_REFUSE_CHANGING_RIGHT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_ABORT_CHANGING_LEFT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_ABORT_CHANGING_RIGHT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_COMPLETE_CHANGING_LEFT,
  EVENT_REPORTING_CHANGING_LANE_TYPE_COMPLETE_CHANGING_RIGHT
};


/**
 * @struct EventReporting
 * @brief 事件报告
 */
struct EventReporting {
  /// 消息头
  MsgHead msg_head;

  /// 子模块的ID
  Uint32_t sub_module_id;
  /// 事件ID
  Int32_t event_id;
  /// 优先级 （0 ~ 最高， 数值越大则优先级越低）
  Int32_t priority;
  /// 此事件的生存期（单位ms, -1 ~ 无限长）
  Int32_t lifetime;
  /// 此事件的需要携带的其它信息（用户自定义）
  Int32_t param[4];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    sub_module_id = SUB_MODULE_ID_INVALID;
    event_id = EVENT_REPORTING_ID_INVALID;
    priority = 0;
    lifetime = 0;
    common::com_memset(param, 0, sizeof(param));
  }

  /**
   * @brief 构造函数
   */
  EventReporting() {
    Clear();
  }
};

/**
 * @struct EventReportingList
 * @brief 事件报告列表
 */
struct EventReportingList {
  /// 最大事件的数量
  enum { MAX_EVENT_REPORTING_NUM = 10 };

  /// 消息头
  MsgHead msg_head;

  /// 事件的数量
  Int32_t event_reporting_num;
  /// 事件报告列表
  EventReporting event_reporting[MAX_EVENT_REPORTING_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    event_reporting_num = 0;
    for (Int32_t i = 0; i < MAX_EVENT_REPORTING_NUM; ++i) {
      event_reporting[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  EventReportingList() {
    Clear();
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_EVENT_REPORTING_H_


