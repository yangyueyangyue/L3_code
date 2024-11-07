/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lane.cc
 * @brief      内部车道定义
 * @details    定义了内部车道的数据格式及相关访问接口
 *
 * @author     boc
 * @date       2020.06.03
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/03  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "map_space/lane.h"


namespace phoenix {
namespace driv_map {

/**
 * @date       2020.06.04
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/04  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Lane::Lane() {
  lane_type_ = static_cast<Int32_t>(LANE_TYPE_UNKNOWN);
  turn_type_ = static_cast<Int32_t>(LANE_TURN_TYPE_UNKNOWN);
  direction_ = static_cast<Int32_t>(LANE_DIRECTION_FORWARD);

  speed_limit_high_ = -1.0F;
  speed_limit_low_ = -1.0F;
}

/**
 * @date       2020.06.04
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/04  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void Lane::Clear() {
  id_.Clear();

  lane_type_ = static_cast<Int32_t>(LANE_TYPE_UNKNOWN);
  quality_ = LANE_QUALITY_GOOD;
  turn_type_ = static_cast<Int32_t>(LANE_TURN_TYPE_UNKNOWN);
  direction_ = static_cast<Int32_t>(LANE_DIRECTION_FORWARD);

  speed_limit_high_ = -1.0F;
  speed_limit_low_ = -1.0F;

  central_curve_.Clear();
  left_boundary_.Clear();
  right_boundary_.Clear();

  left_neighbor_forward_lanes_.Clear();
  right_neighbor_forward_lanes_.Clear();
  left_neighbor_reverse_lanes_.Clear();
  right_neighbor_reverse_lanes_.Clear();

  predecessor_lanes_.Clear();
  successor_lanes_.Clear();
  successor_lanes_start_point_.Clear();
  predecessor_lanes_start_point_.Clear();
  slope_samples_.Clear();
}

/**
 * @date       2020.06.04
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/04  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void Lane::GetWidth(
    map_var_t s, map_var_t * const left_width, map_var_t * const right_width) const {
  if (Nullptr_t != left_width) {
    *left_width = GetBoundaryWidth(left_boundary_, s);
  }
  if (Nullptr_t != right_width) {
    *right_width = GetBoundaryWidth(right_boundary_, s);
  }
}

/* k003 longjiaoy 2022-11-28 (start) */
/**
 * @brief 获取车道边线的类型
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/22  <td>1.0      <td>longjiaoy <td>First edition
 * </table>
 */
void Lane::GetBoundaryType(
    map_var_t s, Int32_t* const left_type, Int32_t* const right_type) const {
  if (Nullptr_t != left_type) {
    *left_type = GetBoundaryType(left_boundary_, s);
  }
  if (Nullptr_t != right_type) {
    *right_type = GetBoundaryType(right_boundary_, s);
  }
}
/* k003 longjiaoy 2022-11-28 (end) */

/**
 * @date       2020.06.04
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/04  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
map_var_t Lane::GetBoundaryWidth(
    const Boundary& boundary, const map_var_t s) const {
  if (boundary.boundary_width_samples.Empty()) {
    return (1.5F);
  }
  if (s <= boundary.boundary_width_samples[0].s) {
    return (boundary.boundary_width_samples[0].width);
  }
  if (s >= boundary.boundary_width_samples.Back().s) {
    return (boundary.boundary_width_samples.Back().width);
  }

  Int32_t low = 0;
  Int32_t high = boundary.boundary_width_samples.Size();
  while (low + 1 < high) {
    const Int32_t mid = (low + high) / 2;
    if (boundary.boundary_width_samples[mid].s <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }

  if (low >= boundary.boundary_width_samples.Size()) {
    LOG_ERR << "Invalid index of boundary samples.";
    return (boundary.boundary_width_samples[0].width);
  }
  const BoundaryWidthAssociation& sample1 = boundary.boundary_width_samples[low];

  if (high >= boundary.boundary_width_samples.Size()) {
    return (sample1.width);
  }

  const BoundaryWidthAssociation& sample2 = boundary.boundary_width_samples[high];
  const map_var_t ratio = (sample2.s - s) / (sample2.s - sample1.s);

  return (sample1.width * ratio + sample2.width * (1.0F - ratio));
}

/* k003 longjiaoy 2022-11-28 (start) */
/**
 * @brief 获取车道边线的类型
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/22  <td>1.0      <td>longjiaoy <td>First edition
 * </table>
 */
Int32_t Lane::GetBoundaryType(const Boundary& boundary, const map_var_t s) const {
  if (boundary.boundary_type_samples.Empty()) {
    return (static_cast<Int32_t>(LANE_BOUNDARY_TYPE_DOTTED_WHITE));
  }
  if (s <= boundary.boundary_type_samples[0].s) {
    return (boundary.boundary_type_samples[0].type);
  }
  if (s >= boundary.boundary_type_samples.Back().s) {
    return (boundary.boundary_type_samples.Back().type);
  }

  Int32_t low = 0;
  Int32_t high = boundary.boundary_type_samples.Size();
  while (low + 1 < high) {
    const Int32_t mid = (low + high) / 2;
    if (boundary.boundary_type_samples[mid].s <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }

  if (low >= boundary.boundary_type_samples.Size()) {
    LOG_ERR << "Invalid index of boundary samples.";
    return (boundary.boundary_type_samples[0].type);
  }
  const BoundaryTypeAssociation& sample1 = boundary.boundary_type_samples[low];

  if (high >= boundary.boundary_type_samples.Size()) {
    return (sample1.type);
  }

  const BoundaryTypeAssociation& sample2 = boundary.boundary_type_samples[high];
  const map_var_t ratio = (sample2.s - s) / (sample2.s - sample1.s);
  if (ratio > 0.5F) {
    return (sample1.type);
  }

  return (sample2.type);
}
/* k003 longjiaoy 2022-11-28 (end) */

/**
 * @brief 获取车道坡度
 * @param[in] s 沿着车道中心线的长度
 * @return 车道中此处位置的坡度
 */
map_var_t Lane::GetLaneSlope(map_var_t s) const {
  // 当车道不含坡道信息时，直接返回0
  if (slope_samples_.Size() < 1) {
    return 0.0F;
  }
  SlopeAssociation slope;
  slope.s = s;
  const Int32_t index = slope_samples_.LowerBound(slope, FuncCmpSlopeArcLen());
  if (index < 1) {
    return slope_samples_[0].slope;
  } else if (index >= slope_samples_.Size()) {
    return slope_samples_[slope_samples_.Size() - 1].slope;
  }
  const SlopeAssociation& slope1 = slope_samples_[index - 1];
  const SlopeAssociation& slope2 = slope_samples_[index];
  const map_var_t slope_seg_length = slope2.s - slope1.s;
  if (common::com_abs(slope_seg_length) < common::kMathEpsilonF) {
    return slope1.slope;
  }
  const map_var_t t = (slope.s - slope1.s) / slope_seg_length;
  return (1.0F - t) * slope1.slope + t * slope2.slope;
}


}  // namespace driv_map
}  // namespace phoenix


