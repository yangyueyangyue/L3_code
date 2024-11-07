/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       line_segment2d.h
 * @brief      二维线段
 * @details    定义二维线段
 *
 * @author     pengc
 * @date       2020.05.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_LINE_SEGMENT2D_H_
#define PHOENIX_COMMON_LINE_SEGMENT2D_H_

#include <string>
#include "geometry/vec2d.h"


namespace phoenix {
namespace common {

/**
 * @class LineSegment2d
 * @brief 定义二维线段
 */
class LineSegment2d {
 public:
  /**
   * @brief 构造函数
   */
  LineSegment2d();

  /**
   * @brief 使用起点及终点构建线段
   * @param start 线段起点
   * @param end 线段终点
   */
  LineSegment2d(const Vec2d &start, const Vec2d &end);

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    points_[0].Clear();
    points_[1].Clear();
  }

  /**
   * @brief 获取起点
   * @return 线段起点
   */
  const Vec2d &start() const { return points_[0]; }

  /**
   * @brief 获取终点
   * @return 线段终点
   */
  const Vec2d &end() const { return points_[1]; }

  /**
   * @brief 设置线段起点
   * @param[in] start 线段起点
   */
  void set_start(const Vec2d& start) { points_[0] = start; }

  /**
   * @brief 设置线段终点
   * @param[in] end 线段终点
   */
  void set_end(const Vec2d& end) { points_[1] = end; }

 private:
  // 保存线段起点(points_[0])及终点(points_[1])
  Vec2d points_[2];
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_LINE_SEGMENT2D_H_
