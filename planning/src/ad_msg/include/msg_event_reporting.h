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
  /// 无效事件
  EVENT_REPORTING_ID_INVALID = 0,
  /// 开启自动控制提醒
  EVENT_REPORTING_ID_START_ROBOTIC,
  /// 关闭自动控制提醒
  EVENT_REPORTING_ID_STOP_ROBOTIC,
  /// 传感器数据超时告警
  EVENT_REPORTING_ID_SENSOR_MSG_TIMEOUT,
  /// 底盘数据超时告警
  EVNET_REPORTING_ID_CHASSIS_MSG_TIMEOUT,
  /// 规划任务超时告警
  EVNET_REPORTING_ID_PLANNING_TIMEOUT,
  /// 车道线丢失告警
  EVENT_REPORTING_ID_CAMERA_LANE_LOST,
  /// 距离前方目标过近告警
  EVENT_REPORTING_ID_AEB_WARNING,
  /// 重制动告警
  EVENT_REPORTING_ID_AEB_ACTION,
  /// 前方检测到不确定的障碍物提醒
  EVENT_REPORTING_ID_UNCERTAIN_OBSTACLE,
  /// 开始跟随前方目标提醒
  EVENT_REPORTING_ID_START_FOLLOWING_OBJ,
  /// 停止跟随前方目标提醒
  EVENT_REPORTING_ID_STOP_FOLLOWING_OBJ,
  /// 请求开始变道提醒
  EVENT_REPORTING_ID_CHANGING_LANE_REQ,
  /// 变道状态提醒
  EVENT_REPORTING_ID_CHANGING_LANE_RSP,
  /// 隧道速度规划提醒
  EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL,
  /// 匝道速度规划提醒
  EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_RAMP,
  /// 地图通知无效
  EVENT_REPORTING_ID_MAP_NOT_VAILD,
};

/**
 * @enum
 * @brief 变道状态类型
 */
enum {
  /// 无效状态
  EVENT_REPORTING_CHANGING_LANE_STATUS_INVALID = 0,
  /// 准备向左变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_PREPARE_FOR_CHANGING_LEFT,
  /// 准备向右变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_PREPARE_FOR_CHANGING_RIGHT,
  /// 开始向左变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_LEFT,
  /// 开始向右变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_RIGHT,
  /// 拒绝向左变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_LEFT,
  /// 拒绝向右变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_RIGHT,
  /// 中止向左变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_LEFT,
  /// 中止向右变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_RIGHT,
  /// 完成向左变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_LEFT,
  /// 完成向右变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_RIGHT,
  /// 拒绝中止变道
  EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_ABORT_CHANGING_LANE
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


