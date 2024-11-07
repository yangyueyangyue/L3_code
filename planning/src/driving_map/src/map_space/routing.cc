/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       routing.cc
 * @brief      内部导航路径
 * @details    定义了内部导航路径的数据格式及相关访问接口
 *
 * @author     boc
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "map_space/routing.h"


namespace phoenix {
namespace driv_map {


Routing::Routing() {
}

Routing::~Routing() {
}

void Routing::Clear() {
  for (Int32_t i = 0; i < routing_sections_.Size(); ++i) {
    routing_sections_[i].Clear();
  }
  routing_sections_.Clear();
}

bool Routing::IsOverlappedWithRoutingSegment(
    Int32_t lane_index, map_var_t start_s, map_var_t end_s) const {
  Int32_t segment_index = -1;
  Int32_t neighbor_index = -1;

  return IsOverlappedWithRoutingSegment(lane_index, start_s, end_s,
                                        &segment_index, &neighbor_index);
}

bool Routing::IsOverlappedWithRoutingSegment(
    Int32_t lane_index, map_var_t start_s, map_var_t end_s,
    Int32_t * const segment_index, Int32_t * const neighbor_index) const {
  *segment_index = -1;
  *neighbor_index = -1;

  for (Int32_t i = 0; i < routing_sections_.Size(); ++i) {
    for (Int32_t j = 0; j < routing_sections_[i].neighbors.Size(); ++j) {
      const RoutingSegment& seg = routing_sections_[i].neighbors[j];

      if (seg.lane_index == lane_index) {
        if (start_s > seg.end_s) {
          continue;
        }
        if (end_s < seg.start_s) {
          continue;
        }
        *segment_index = i;
        *neighbor_index = j;

        return true;
      }
    }
  }

  return false;
}


bool Routing::IsOnRoutingSegment(Int32_t lane_index, map_var_t s) const {
  for (Int32_t i = 0; i < routing_sections_.Size(); ++i) {
    for (Int32_t j = 0; j < routing_sections_[i].neighbors.Size(); ++j) {
      const RoutingSegment& seg = routing_sections_[i].neighbors[j];
      if (seg.lane_index == lane_index) {
        if (((seg.start_s-common::kGeometryEpsilon) < s) &&
            (s < (seg.end_s+common::kGeometryEpsilon))) {
          return true;
        }
      }
    }
  }

  return false;
}


}  // namespace driv_map
}  // namespace phoenix

