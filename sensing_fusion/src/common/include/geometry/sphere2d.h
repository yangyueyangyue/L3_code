/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       sphere2d.h
 * @brief      圆
 * @details    定义二维平面上的圆
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

#ifndef PHOENIX_COMMON_SPHERE2D_H_
#define PHOENIX_COMMON_SPHERE2D_H_

#include "geometry/vec2d.h"

namespace phoenix {
namespace common {


/**
 * @class Sphere2d
 * @brief 二维平面上的圆
 */
class Sphere2d {
public:
  /**
   * @brief 构造函数
   */
  Sphere2d();

  /**
   * @brief 构造函数
   * @param[in] center 中心坐标
   * @param[in] radius 半径
   */
  Sphere2d(const Vec2d& center, geo_var_t radius);

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 获取中心坐标
   * @return 中心坐标
   */
  const Vec2d& center() const { return center_; }

  /**
   * @brief 获取半径
   * @return 半径
   */
  geo_var_t radius() const { return radius_; }

  /**
   * @brief 设置中心坐标
   * @param[in] center 中心坐标
   */
  void set_center(const Vec2d& center) { center_ = center; }

  /**
   * @brief 设置半径
   * @param[in] radius 半径
   */
  void set_radius(const geo_var_t radius) { radius_ = radius; }

private:
  // Sphere center
  Vec2d center_;
  // Sphere radius
  geo_var_t radius_;
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_SPHERE2D_H_
