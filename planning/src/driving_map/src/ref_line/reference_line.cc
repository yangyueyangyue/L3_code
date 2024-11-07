/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       reference_line.cc
 * @brief      参考线
 * @details    与单条参考线相关的一些变量和函数
 *
 * @author     boc
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "ref_line/reference_line.h"

namespace phoenix {
namespace driv_map {


/**
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
ReferenceLine::ReferenceLine() {
  neighbor_flag_ = 0;
  quality_ = LANE_QUALITY_GOOD;
}

/**
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
ReferenceLine::~ReferenceLine() {
}

/**
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void ReferenceLine::Clear() {
  lane_segments_.Clear();
  curve_.Clear();
  smooth_curve_.Clear();
  curve_curvature_info_.Clear();
  smooth_curve_curvature_info_.Clear();
}

/**
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t ReferenceLine::FindLaneSegment(Int32_t lane_index) const {
  for (Int32_t i = 0; i < lane_segments_.Size(); ++i) {
    if (lane_segments_[i].lane_index == lane_index) {
      return (i);
    }
  }

  return (-1);
}

/**
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t ReferenceLine::FindLaneSegment(
    map_var_t s_on_ref_line, Int32_t * const lane_index, Float32_t* const s_on_lane) const {
  const Int32_t lane_segments_size = lane_segments_.Size();
  if (lane_segments_size < 1) {
    *lane_index = -1;
    *s_on_lane = 0.0F;
    return (-1);
  }

  if (s_on_ref_line <
      (lane_segments_.Front().start_s_on_ref+common::kMathEpsilonF)) {
    *lane_index = lane_segments_.Front().lane_index;
    *s_on_lane = 0.0F;
    return (0);
  }
  if (s_on_ref_line >
      (lane_segments_.Back().end_s_on_ref-common::kMathEpsilonF)) {
    *lane_index = lane_segments_.Back().lane_index;
    *s_on_lane = lane_segments_.Back().end_s;
    return (lane_segments_size-1);
  }

  for (Int32_t i = 0; i < lane_segments_size; ++i) {
    const LaneSegment& seg = lane_segments_[i];
    if (((seg.start_s_on_ref - 0.1F) < s_on_ref_line) &&
        (s_on_ref_line < (seg.end_s_on_ref + 0.1F))) {
      *lane_index = seg.lane_index;
      *s_on_lane = seg.start_s + s_on_ref_line - seg.start_s_on_ref;
      return (i);
    }
  }

  LOG_ERR << "Failed to find lane segment by projection on reference line, "
          << " s_on_ref_line=" << s_on_ref_line
          << ", total_s=" << lane_segments_.Back().end_s_on_ref;

  *lane_index = -1;
  *s_on_lane = 0.0F;

  return (-1);
}


}  // namespace driv_map
}  // namespace phoenix
