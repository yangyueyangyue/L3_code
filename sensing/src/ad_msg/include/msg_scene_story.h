/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_scene_story.h
 * @brief      特殊场景功能消息定义
 * @details    定义了特殊场景功能消息类型
 *
 * @author     longjiaoy
 * @date       2022.12.10
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2022/12/10  <td>1.0      <td>longjiaoy  <td>First edition
 * <tr><td>2022/12/29  <td>1.1      <td>pengc      <td>修改
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_SCENE_STORY_H_
#define PHOENIX_AD_MSG_MSG_SCENE_STORY_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"
#include "geometry/vec2d.h"
#include "container/static_vector.h"
#include "msg_chassis.h"


namespace phoenix {
namespace ad_msg {


/**
 * @enum
 * @brief 场景任务的类型
 */
enum {
  /// 无效
  SCENE_STORY_TYPE_INVALID = 0,
  /// 接近终点
  SCENE_STORY_TYPE_CLOSE_TO_DESTINATION,
  /// 接近信号灯
  SCENE_STORY_TYPE_CLOSE_TO_TRAFFIC_SIGNAL,
  /// 接近停止信号
  SCENE_STORY_TYPE_CLOSE_TO_STOP_SIGN,
  /// 接近弯道
  SCENE_STORY_TYPE_CLOSE_TO_CURVE,
  /// 接近人行横道
  SCENE_STORY_TYPE_CLOSE_TO_CROSSWALK,
  /// 接近减速带
  SCENE_STORY_TYPE_CLOSE_TO_SPEED_BUMPS,
  /// 接近停车区域
  SCENE_STORY_TYPE_CLOSE_TO_PARKING_SPACE,
  /// 接近禁止停车区域
  SCENE_STORY_TYPE_CLOSE_TO_CLEAR_AREA,
  /// 接近交叉路口
  SCENE_STORY_TYPE_CLOSE_TO_JUNCTION,
  /// 接近让车标志
  SCENE_STORY_TYPE_CLOSE_TO_YIELD_SIGN
};


/**
 * @struct SceneStoryControlLine
 * @brief 场景任务的控制线
 */
struct SceneStoryControlLine {
  /// 控制线起点
  common::Vec2d start_point;
  /// 控制线终点
  common::Vec2d end_point;

  /**
   * @brief 构造函数
   */
  SceneStoryControlLine() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    start_point.Clear();
    end_point.Clear();
  }
};


/**
 * @struct SceneStoryAction
 * @brief 执行场景任务的条件(满足条件方可执行)
 */
struct SceneStoryCondition {
  /// 此条件是否有效
  Int8_t vaild;

  /// 距离条件是否有效
  Int8_t valid_s;
  /// 距离控制线的距离(Start)
  Float32_t start_s;
  /// 距离控制线的距离(End)
  Float32_t end_s;

  /// 速度上限条件是否有效
  Int8_t valid_speed_high;
  /// 速度上限
  Float32_t speed_high;
  /// 速度下限条件是否有效
  Int8_t valid_speed_low;
  /// 速度下限
  Float32_t speed_low;

  /// 档位条件
  Int8_t gear;

  /**
   * @brief 构造函数
   */
  SceneStoryCondition() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    vaild = 0;

    valid_s = 0;
    start_s = 0.0F;
    end_s = 0.0F;

    valid_speed_high = 0;
    speed_high = 0.0F;
    valid_speed_low = 0;
    speed_low = 0.0F;

    gear = VEH_GEAR_INVALID;
  }
};


/**
 * @struct SceneStoryAction
 * @brief 需要执行的场景任务
 */
struct SceneStoryAction {
  /// 此行为是否有效
  Int8_t vaild;
  /// 维持时间 (ms), <= 0 ~ 维持时间无效
  Int32_t holding_time;

  /// 目标车速是否有效
  Int8_t valid_speed;
  /// 目标车速
  Float32_t speed;

  /// 建议加速度/减速度是否有效
  Int8_t valid_acceleration;
  /// 建议加速度/减速度
  Float32_t acceleration;

  /// 目标档位
  Int8_t gear;
  /// 转向灯
  Int8_t turn_lamp;
  /// 制动灯
  Int8_t brake_lamp;

  /**
   * @brief 构造函数
   */
  SceneStoryAction() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    vaild = 0;
    holding_time = 0;

    valid_speed = 0;
    speed = 0.0F;
    valid_acceleration = 0;
    acceleration = 0.0F;

    gear = VEH_GEAR_INVALID;
    turn_lamp = VEH_TURN_LAMP_INVALID;
    brake_lamp = VEH_LAMP_INVALID;
  }
};


/**
 * @struct SceneStory
 * @brief 场景任务
 */
struct SceneStory {
  /// 任务ID
  Int32_t id;
  /// 任务类型
  Int32_t type;

  /// 是否在范围内
  Int8_t is_in_range;
  /// 离控制线的距离
  Float32_t distance;
  /// 控制线
  SceneStoryControlLine control_line;
  /// 执行任务的条件
  SceneStoryCondition condition;
  /// 需要执行的任务
  SceneStoryAction action;

  /// 任务类型相关的参数
  union {
    /// 接近弯道
    struct {
      /// 弯道的曲率
      Float64_t curvature;
    } close_to_curve;
  } extra_param;

  /**
   * @brief 构造函数
   */
  SceneStory() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = -1;
    type = SCENE_STORY_TYPE_INVALID;

    is_in_range = 0;
    distance = 0.0F;
    condition.Clear();
    action.Clear();

    common::com_memset(&extra_param, 0, sizeof(extra_param));
  }
};


/**
 * @struct SceneStoryList
 * @brief 场景任务列表
 */
struct SceneStoryList {
  /**
   * @enum MAX_SCENE_STORYS_NUM
   * @brief 列表中最大场景任务的数量
   */
  enum { MAX_SCENE_STORYS_NUM = 10 };

  /// 消息头
  MsgHead msg_head;
  /// 场景任务的数量
  Int32_t story_num;
  /// 场景任务列表
  SceneStory storys[MAX_SCENE_STORYS_NUM];

  /**
   * @brief 构造函数
   */
  SceneStoryList() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    story_num = 0;
    for (Int32_t i = 0; i < MAX_SCENE_STORYS_NUM; ++i) {
      storys[i].Clear();
    }
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_SCENE_STORY_H_


