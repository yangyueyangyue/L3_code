/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       path.h
 * @brief      路径处理
 * @details    实现了部分路径处理方法（最近点查询、相交判断，采样等）
 *
 * @author     boc
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/04/18  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_PATH_H_
#define PHOENIX_COMMON_PATH_H_

#include "utils/macros.h"
#include "utils/log.h"

#include "geometry/vec2d.h"
#include "container/static_vector.h"

namespace phoenix {
namespace common {

/**
 * @struct PathPoint
 * @brief 路径点，提供了路径上点的二维位置、航向角、曲率、沿着路径的弧长、侧向距离。
 */
struct PathPoint {
  /// 二维位置（单位：米）
  Vec2d point;
  /// 航向角（单位：弧度）
  geo_var_t heading;
  /// 曲率（单位：1/米）
  geo_var_t curvature;
  /// 沿着路径的弧长（单位：米）
  geo_var_t s;
  /// 侧向距离（单位：米），左正右负
  geo_var_t l;

  /**
   * @brief 初始化函数。
   */
  void Clear() {
    point.Clear();
    heading = 0.0f;
    curvature = 0.0f;
    s = 0.0f;
    l = 0.0f;
  }

  /**
   * @brief 构造函数。
   */
  PathPoint() {
    heading = 0.0f;
    curvature = 0.0f;
    s = 0.0f;
    l = 0.0f;
  }
};

/**
 * @struct TrajectoryPoint
 * @brief 轨迹点，除了含点的空间信息外，还包含速度、加速度等时间信息。
 */
struct TrajectoryPoint {
  /// 点的空间信息，包含位置、航向角、曲率、弧长等。
  PathPoint path_point;
  /// 速度（单位：m/s）
  geo_var_t v;
  /// 加速度（单位：m/s^2）
  geo_var_t a;
  /// 横摆角速度（单位：rad/s）
  geo_var_t yaw_rate;
  /// 相对于曲线起点已行驶的相对时间（单位：s）
  geo_var_t relative_time;

  /**
   * @brief 初始化函数。
   */
  void Clear() {
    path_point.Clear();
    v = 0.0f;
    a = 0.0f;
    yaw_rate = 0.0f;
    relative_time = 0.0f;
  }

  /**
   * @brief 构造函数。
   */
  TrajectoryPoint() {
    v = 0.0f;
    a = 0.0f;
    yaw_rate = 0.0f;
    relative_time = 0.0f;
  }
};


/**
 * @class Path
 * @brief 路径处理，提供了根据用户输入的点列构建一条路径，计算每个路径
 *        点的航向角和曲率，曲率分析等功能。
 */
class Path {
public:
  /**
   * 枚举类型
   */
  enum {
    /// 直行路段
    TYPE_STRAIGHT = 0,
    /// 左转弯路段
    TYPE_CURVING_LEFT,
    /// 右转弯路段
    TYPE_CURVING_RIGHT
  };
  /**
   * @struct CurveSegment
   * @brief 曲线段信息。曲率分析时需要用到此结构体。
   */
  struct CurveSegment {
    /// 曲线段的曲率类型，定义STRAIGHT = 0, LEFT_TURN = 1, RIGHT_TURN = 2。
    /// 定义曲率绝对值 < 0.003为直线，定义正曲率为左，定义负曲率为右。
    Int32_t type;
    /// 此曲线段在路径中的起始位置
    geo_var_t start_s;
    /// 此曲线段的长度
    geo_var_t length;
    /// 此曲线段的平均曲率
    geo_var_t ave_curvature;
    /// 此曲线段的最大曲率
    geo_var_t max_curvature;
  };

  /// 路径上点的最大个数
  static const Int32_t kMaxPathPointNum = 500;
  /// 曲线段的最大个数。直线段也认为是曲线段。
  static const Int32_t kMaxCurveSegmentNum = 20;
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_PATH_H_


