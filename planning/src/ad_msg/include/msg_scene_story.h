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

#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
#include <string>
#endif

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
  SCENE_STORY_TYPE_CLOSE_TO_YIELD_SIGN,
  /// 接近隧道
  SCENE_STORY_TYPE_CLOSE_TO_TUNNEL,
  /// 接近匝道(减速)
  SCENE_STORY_TYPE_CLOSE_TO_RAMP,
  /// 路过上/下匝道口(不减速)
  SCENE_STORY_TYPE_PASSING_RAMP,
  SCENE_STORY_TYPE_CLOSE_TO_ROAD_RIVER,
  /// 切换纯地图模式
  SCENE_STORY_TYPE_SWITCH_TO_MAP,
  /// 切换纯相机模式
  SCENE_STORY_TYPE_SWITCH_TO_CAM_LANE,

};


/**
 * @struct SceneStoryControlLine
 * @brief 场景任务的控制线
 */
struct SceneStoryControlLine {
  /**
   * @struct Point2D
   * @brief 二维点
   */
  struct Point2D {
    /// 坐标x
    Float64_t x;
    /// 坐标y
    Float64_t y;

    /**
     * @brief 构造函数
     */
    Point2D() {
      Clear();
    }

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      x = 0.0;
      y = 0.0;
    }
  };

  /// 控制线起点
  Point2D start_point;
  /// 控制线终点
  Point2D end_point;

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
 * @struct SceneStoryArea
 * @brief 场景任务的区域
 */
struct SceneStoryArea {
  /**
   * @enum
   * @brief 区域类型
   */
  enum {
    AREA_TYPE_INVALID = 0,
    AREA_TYPE_01,
    AREA_TYPE_02
  };

  struct AreaType_01 {
    /// 此区域是否有效
    bool valid;
    /// 离区域的距离
    Float32_t distance;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      valid = false;
      distance = 0.0F;
    }

    /**
     * @brief 判断是否在区域内部
     * @return true ~ 在区域内部, false ~ 不在区域内部
     */
    bool IsInArea() const {
      if (valid) {
        if ((distance < 0.1F) && (distance > -0.1F)) {
          return true;
        }
      }

      return false;
    }

    /**
     * @brief 距离区域的距离
     * @return 距离区域的距离 (>0 ~ 接近区域, 0 ~ 在区域内部, <0 ~ 远离区域)
     */
    Float32_t DistanceToArea() const {
      if (valid) {
        return (distance);
      } else {
        return (9999999.0F);
      }
    }
  };

  struct AreaType_02 {
    /// 此区域是否有效
    bool valid;
    /// 离控制线的距离
    Float32_t dist_to_ctl_line;
    /// 控制线
    SceneStoryControlLine control_line;
    /// 距离控制线的距离(Start)
    Float32_t start_s;
    /// 距离控制线的距离(End)
    Float32_t end_s;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      valid = false;
      dist_to_ctl_line = 0.0F;
      control_line.Clear();
      start_s = 0.0F;
      end_s = 0.0F;
    }

    /**
     * @brief 判断是否在区域内部
     * @return true ~ 在区域内部, false ~ 不在区域内部
     */
    bool IsInArea() const {
      if (valid) {
        if ((dist_to_ctl_line < (start_s+0.1F)) &&
            (dist_to_ctl_line > (end_s-0.1F))) {
          return true;
        }
      }

      return false;
    }

    /**
     * @brief 距离区域的距离
     * @return 距离区域的距离 (>0 ~ 接近区域, 0 ~ 在区域内部, <0 ~ 远离区域)
     */
    Float32_t DistanceToArea() const {
      Float32_t dist = 9999999.0F;
      if (valid) {
        if (dist_to_ctl_line > start_s) {
          dist = dist_to_ctl_line - start_s;
        } else if (dist_to_ctl_line < end_s) {
          dist = dist_to_ctl_line - end_s;
        } else {
          dist = 0.0F;
        }
      }

      return (dist);
    }
  };

  /// 区域类型
  Int32_t area_type;
  /// 区域(类型1)
  AreaType_01 area_type_01;
  /// 区域(类型2)
  AreaType_02 area_type_02;

  /**
   * @brief 构造函数
   */
  SceneStoryArea() {
    Clear();
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    area_type = AREA_TYPE_INVALID;
    area_type_01.Clear();
    area_type_02.Clear();
  }

  /**
   * @brief 判断区域是否有效
   * @return true ~ 有效, false ~ 无效
   */
  bool IsValid() const {
    bool ret = false;

    switch (area_type) {
    case (AREA_TYPE_01):
      ret = area_type_01.valid;
      break;
    case (AREA_TYPE_02):
      ret = area_type_02.valid;
      break;
    default:
      ret = false;
      break;
    }

    return (ret);
  }

  /**
   * @brief 判断是否在区域内部
   * @return true ~ 在区域内部, false ~ 不在区域内部
   */
  bool IsInArea() const {
    bool ret = false;

    switch (area_type) {
    case (AREA_TYPE_01):
      ret = area_type_01.IsInArea();
      break;
    case (AREA_TYPE_02):
      ret = area_type_02.IsInArea();
      break;
    default:
      ret = false;
      break;
    }

    return (ret);
  }

  /**
   * @brief 距离区域的距离
   * @return 距离区域的距离 (>0 ~ 接近区域, 0 ~ 在区域内部, <0 ~ 远离区域)
   */
  Float32_t DistanceToArea() const {
    Float32_t ret = 9999999.0F;;

    switch (area_type) {
    case (AREA_TYPE_01):
      ret = area_type_01.DistanceToArea();
      break;
    case (AREA_TYPE_02):
      ret = area_type_02.DistanceToArea();
      break;
    default:
      // nothing to do
      break;
    }

    return (ret);
  }
};


/**
 * @struct SceneStoryAction
 * @brief 执行场景任务的条件(满足条件方可执行)
 */
struct SceneStoryCondition {
  /// 此条件是否有效
  Int8_t vaild;

  /// 区域条件是否有效
  Int8_t valid_area;

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

    valid_area = 0;

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
  Int32_t hold_time;

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
    hold_time = 0;

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
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  std::string id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  Uint64_t id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  Uint64_t id;
#else
  Uint64_t id;
#endif
  /// 任务类型
  Int32_t type;

  /// 区域
  SceneStoryArea area;
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
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    id.clear();
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    id = 0;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
    id = 0;
#else
    id = 0;
#endif
    type = SCENE_STORY_TYPE_INVALID;

    area.Clear();
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


