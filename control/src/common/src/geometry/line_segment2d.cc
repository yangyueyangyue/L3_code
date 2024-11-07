/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       line_segment2d.cc
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

#include "geometry/line_segment2d.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <iostream>
#include <sstream>

#include "utils/log.h"
#include "math/math_utils.h"


namespace phoenix {
namespace common {


LineSegment2d::LineSegment2d() {
}

LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end) {
  points_[0] = start;
  points_[1] = end;
}


}  // namespace common
}  // namespace phoenix
