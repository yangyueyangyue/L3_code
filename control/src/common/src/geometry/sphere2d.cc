/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       sphere2d.cc
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

#include "geometry/sphere2d.h"

namespace phoenix {
namespace common {


Sphere2d::Sphere2d() {
  radius_ = 0.0F;
}

Sphere2d::Sphere2d(const Vec2d& center, geo_var_t radius) {
  center_ = center;
  radius_ = radius;
}

void Sphere2d::Clear() {
  center_.Clear();
  radius_ = 0.0F;
}


}  // namespace common
}  // namespace phoenix

