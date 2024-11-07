/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       obbox2d.cc
 * @brief      有向矩形包围盒
 * @details    定义二维平面上的有向矩形包围盒
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

#include "geometry/obbox2d.h"

namespace phoenix {
namespace common {


OBBox2d::OBBox2d() {
}

OBBox2d::OBBox2d(const Vec2d& center,
        const Vec2d& unit_direction, const Vec2d& extent) {
  set_center(center);
  set_unit_direction(unit_direction);
  set_extents(extent);
}

void OBBox2d::Clear() {
  center_.Clear();
  unit_direction_[0].Clear();
  unit_direction_[1].Clear();
  extents_.Clear();
}


}  // namespace common
}  // namespace phoenix
