/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       aabbox2d.cc
 * @brief      与坐标轴平行的矩形包围盒
 * @details    定义二维平面上的与坐标轴平行的矩形包围盒
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

#include "geometry/aabbox2d.h"
#include "utils/log.h"
#include "math/math_utils.h"

namespace phoenix {
namespace common {


AABBox2d::AABBox2d(const Vec2d &one_corner, const Vec2d &opposite_corner) {
  if (one_corner.x() < opposite_corner.x()) {
    min_.set_x(one_corner.x());
    max_.set_x(opposite_corner.x());
  } else {
    min_.set_x(opposite_corner.x());
    max_.set_x(one_corner.x());
  }

  if (one_corner.y() < opposite_corner.y()) {
    min_.set_y(one_corner.y());
    max_.set_y(opposite_corner.y());
  } else {
    min_.set_y(opposite_corner.y());
    max_.set_y(one_corner.y());
  }
}

AABBox2d::AABBox2d(
    geo_var_t one_corner_x, geo_var_t one_corner_y,
    geo_var_t opposite_corner_x, geo_var_t opposite_corner_y) {
  if (one_corner_x < opposite_corner_x) {
    min_.set_x(one_corner_x);
    max_.set_x(opposite_corner_x);
  } else {
    min_.set_x(opposite_corner_x);
    max_.set_x(one_corner_x);
  }

  if (one_corner_y < opposite_corner_y) {
    min_.set_y(one_corner_y);
    max_.set_y(opposite_corner_y);
  } else {
    min_.set_y(opposite_corner_y);
    max_.set_y(one_corner_y);
  }
}

AABBox2d::AABBox2d(const Vec2d &center,
                   const geo_var_t length, const geo_var_t width) {
  min_.set_x(center.x() - 0.5F*length);
  max_.set_x(center.x() + 0.5F*length);
  min_.set_y(center.y() - 0.5F*width);
  max_.set_y(center.y() + 0.5F*width);
}

bool AABBox2d::IsPointIn(const Vec2d &point) const {
  if ((min().x() < point.x()) && (point.x() < max().x()) &&
      (min().y() < point.y()) && (point.y() < max().y())) {
    return true;
  }

  return false;
}

bool AABBox2d::IsPointIn(geo_var_t x, geo_var_t y) const {
  if ((min().x() < x) && (x < max().x()) &&
      (min().y() < y) && (y < max().y())) {
    return true;
  }

  return false;
}


}  // namespace common
}  // namespace phoenix
