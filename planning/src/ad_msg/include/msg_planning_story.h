/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_scene_story.h
 * @brief      Planning场景功能消息定义
 * @details    定义了Planning场景功能消息类型
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

#ifndef PHOENIX_AD_MSG_MSG_PLANNING_STORY_H_
#define PHOENIX_AD_MSG_MSG_PLANNING_STORY_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"
#include "geometry/vec2d.h"
#include "container/static_vector.h"
#include "curve/path.h"
#include "msg_chassis.h"


namespace phoenix {
namespace ad_msg {


/**
 * @struct PlanningStoryArea
 * @brief 执行场景任务的区域参考线(区域内有效)
 */
struct PlanningStoryRefLine {
  enum { MAX_POINT_NUM = 100 };
  struct Point {
    Float64_t x;
    Float64_t y;
  };

  Int32_t point_num;
  Point points[MAX_POINT_NUM];

  /**
   * @brief 构造函数
   */
  PlanningStoryRefLine() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    point_num = 0;
    common::com_memset(points, 0, sizeof(points));
  }
};


/**
 * @struct SceneStoryAction
 * @brief 执行场景任务的条件(满足条件方可执行)
 */
struct PlanningStoryCondition {
  /// 此条件是否有效
  Int8_t vaild;

  /// 执行场景任务的区域(区域内有效)
  struct {
    bool valid;
    Float32_t left_width;
    Float32_t right_width;
    Float32_t start_s;
    Float32_t end_s;

    void Clear() {
      valid = false;
      left_width = -0.1F;
      right_width = -0.1F;
      start_s = -0.1;
      end_s = -0.1;
    }
  } area;
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
  PlanningStoryCondition() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    vaild = 0;

    area.Clear();
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
struct PlanningStoryAction {
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

  Uint32_t cmd;

  /**
   * @brief 构造函数
   */
  PlanningStoryAction() {
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

    cmd = 0;
  }
};


/**
 * @struct SceneStory
 * @brief 场景任务
 */
struct PlanningStory {
  /// 任务ID
  Int32_t id;
  /// 任务类型
  Int32_t type;

  /// 是否在范围内
  Int8_t is_in_range;
  /// 区域参考线
  PlanningStoryRefLine ref_line;
  /// 执行任务的条件
  PlanningStoryCondition condition;
  /// 需要执行的任务
  PlanningStoryAction action;


  /**
   * @brief 构造函数
   */
  PlanningStory() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = -1;
    type = 0;

    is_in_range = 0;
    ref_line.Clear();
    condition.Clear();
    action.Clear();
  }
};


/**
 * @struct PlanningStoryList
 * @brief 场景任务列表
 */
struct PlanningStoryList {
  /**
   * @enum MAX_SCENE_STORYS_NUM
   * @brief 列表中最大场景任务的数量
   */
  enum { MAX_PLANNING_STORYS_NUM = 3 };

  /// 消息头
  MsgHead msg_head;
  /// 场景任务的数量
  Int32_t story_num;
  /// 场景任务列表
  PlanningStory storys[MAX_PLANNING_STORYS_NUM];

  /**
   * @brief 构造函数
   */
  PlanningStoryList() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    story_num = 0;
    for (Int32_t i = 0; i < MAX_PLANNING_STORYS_NUM; ++i) {
      storys[i].Clear();
    }
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_PLANNING_STORY_H_


