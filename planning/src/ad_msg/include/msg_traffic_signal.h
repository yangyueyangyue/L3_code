/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_traffic_signal.h
 * @brief      交通信号消息定义
 * @details    定义了交通信号消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_TRAFFIC_SIGNAL_H_
#define PHOENIX_AD_MSG_MSG_TRAFFIC_SIGNAL_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"

#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
#include <string>
#endif


namespace phoenix {
namespace ad_msg {


/**
 * @struct TrafficSignalBox
 * @brief 交通标志的位置及尺寸
 */
struct TrafficSignalBox {
  Float32_t x;
  Float32_t y;
  Float32_t z;
  Float32_t width;
  Float32_t height;
  Int32_t camera_id;

  /**
   * @brief 构造函数
   */
  TrafficSignalBox() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    x = 0.0F;
    y = 0.0F;
    z = 0.0F;
    width = 0.0F;
    height = 0.0F;
    camera_id = 0;
  }
};

/**
 * @struct TrafficSignalSpeedRestriction
 * @brief 限速标志
 */
struct TrafficSignalSpeedRestriction {
  enum Type {
    UNKNOWN = 0,
    START_RESTRICTION,
    END_RESTRICTION
  };

  /// Traffic signal string-ID in the map data.
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  std::string id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  Uint64_t id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  Uint64_t id;
#else
  Uint64_t id;
#endif
  /// 限速标志类型
  Int32_t type;
  /// 位置及尺寸
  TrafficSignalBox box;
  /// 限速值 (m/s)
  Float32_t speed;

  /**
   * @brief 构造函数
   */
  TrafficSignalSpeedRestriction() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    id.clear();
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    id = 0;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
    id = 0;
#else
    id = 0;
#endif
    type = UNKNOWN;
    box.Clear();
    speed = 0.0F;
  }
};

/**
 * @struct TrafficSignalList
 * @brief 交通标志列表
 */
struct TrafficSignalList {
  /**
   * @enum MAX_TRAFFIC_SIGNAL_NUM
   * @brief 列表中最大交通标志列表的数量
   */
  enum { MAX_TRAFFIC_SIGNAL_NUM = 10 };

  /// 消息头
  MsgHead msg_head;

  /// 限速标志的数量
  Int32_t speed_restriction_num;
  /// 限速标志列表
  TrafficSignalSpeedRestriction speed_restrictions[MAX_TRAFFIC_SIGNAL_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    speed_restriction_num = 0;
    for (Int32_t i = 0; i < MAX_TRAFFIC_SIGNAL_NUM; ++i) {
      speed_restrictions[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  TrafficSignalList() {
    Clear();
  }
};


/**
 * @struct TrafficLight
 * @brief 红绿灯
 */
struct TrafficLight {
  enum Color {
    UNKNOWN = 0,
    RED,
    YELLOW,
    GREEN,
    BLACK
  };

  /// Traffic light string-ID in the map data.
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  std::string id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  Uint64_t id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  Uint64_t id;
#else
  Uint64_t id;
#endif
  /// Color of Traffic Light
  Int32_t color;
  /// Box of Traffic Light
  TrafficSignalBox box;
  /// How confidence about the detected results, between 0 and 100.
  Int32_t confidence;
  /// Duration of the traffic light since detected.
  Float32_t tracking_time;
  /// Is traffic blinking
  Int8_t blink;
  /// traffic light remaining time, (unit: s, < 0 is invalid).
  Int32_t remaining_time;

  /**
   * @brief 构造函数
   */
  TrafficLight() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    id.clear();
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    id = 0;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
    id = 0;
#else
    id = 0;
#endif
    color = UNKNOWN;
    box.Clear();
    confidence = 0;
    tracking_time = 0;
    blink = 0;
    remaining_time = -1;
  }
};

/**
 * @struct TrafficLightList
 * @brief 红绿灯列表
 */
struct TrafficLightList {
  /**
   * @enum MAX_TRAFFIC_LIGHT_NUM
   * @brief 列表中最大红绿灯的数量
   */
  enum { MAX_TRAFFIC_LIGHT_NUM = 10 };

  /// 消息头
  MsgHead msg_head;
  /// 红绿灯的数量
  Int32_t traffic_light_num;
  /// 红绿灯列表
  TrafficLight traffic_lights[MAX_TRAFFIC_LIGHT_NUM];

  /**
   * @brief 构造函数
   */
  TrafficLightList() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    traffic_light_num = 0;
    for (Int32_t i = 0; i < MAX_TRAFFIC_LIGHT_NUM; ++i) {
      traffic_lights[i].Clear();
    }
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_TRAFFIC_SIGNAL_H_


