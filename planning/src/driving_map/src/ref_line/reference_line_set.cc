/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       reference_line_set.cc
 * @brief      处理参考线列表
 * @details    实现了获取参考线集合的相关方法
 *
 * @author     boc
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "ref_line/reference_line_set.h"
#include "utils/log.h"


#define ENABLE_REFERENCE_LINE_SET_TRACE (0)
#define ENABLE_REFERENCE_LINE_SET_PERFORMANCE_TEST (0)


namespace phoenix {
namespace driv_map {


/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
ReferenceLineSet::ReferenceLineSet() {
  nearest_lane_index_ = -1;
  nearest_index_in_neighbor_lanes_of_curr_pos_ = -1;
  major_refefrence_line_index_ = -1;
  ref_lane_segment_start_index_ = 0;

  param_.backward_len = 10.0F;
  param_.forward_len = 150.0F;
  param_.anchor_point_sample_interval = 2.0F;
  param_.anchor_point_sample_cutoff_endpoint_dist = 1.5F;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
ReferenceLineSet::~ReferenceLineSet() {
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void ReferenceLineSet::Clear() {
  nearest_lane_index_ = -1;
  nearest_point_on_lane_.Clear();
  nearest_index_in_neighbor_lanes_of_curr_pos_ = -1;
  neighbor_lanes_of_curr_pos_.Clear();

  ref_lane_segment_start_index_ = -1;
  major_refefrence_line_index_ = -1;
  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    reference_lines_[i].Clear();
  }
  reference_lines_.Clear();

  proj_point_on_major_ref_line_.Clear();
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool ReferenceLineSet::Construct(
    const common::Vec2d& point,
    map_var_t heading,
    const HDMap& map) {
#if ENABLE_REFERENCE_LINE_SET_TRACE
  std::cout << "######### Construct ReferenceLine >>>>>>>>>" << std::endl;
#endif

#if ENABLE_REFERENCE_LINE_SET_PERFORMANCE_TEST
  /// Performance of constructing reference lines (Start)
  phoenix::common::Stopwatch performance_timer_constructing_reference_lines;
#endif

  // Clear
  Clear();

  if (!FindCurrentLane(point, heading, map)) {
    LOG_ERR << "Failed to find current lane.";
    return false;
  }

  if (!FindAllReferenceLaneSegments(map)) {
    LOG_ERR << "Failed to find all reference lane segments.";
    return false;
  }

#if ENABLE_REFERENCE_LINE_SET_TRACE
#if 1
  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    std::cout << "lane_segments[" << i
              << "]: ";
    for (Int32_t j = 0; j < reference_lines_[i].lane_segments().Size(); ++j) {
      std::cout << " {id="
                << reference_lines_[i].lane_segments()[j].lane_id.id
                << ", index="
                << reference_lines_[i].lane_segments()[j].lane_index
                << ", start_s="
                << reference_lines_[i].lane_segments()[j].start_s
                << ", end_s="
                << reference_lines_[i].lane_segments()[j].end_s
                << ", accu_start_s="
                << reference_lines_[i].lane_segments()[j].
                   start_s_on_ref
                << ", accu_end_s="
                << reference_lines_[i].lane_segments()[j].
                   end_s_on_ref
                << ", routing="
                << reference_lines_[i].lane_segments()[j].is_on_routing
                << "} ==>";
    }
    std::cout << " {end}" << std::endl;
    std::cout << "neighbor_flag="
              << reference_lines_[i].neighbor_flag() << std::endl;
  }
#endif
#endif

  if (!CreateAllReferenceLines(map)) {
    LOG_ERR << "Failed to create reference lines.";
    return false;
  }

  if (!FindCurrentReferenceLine(map)) {
    LOG_ERR << "Failed to find current reference line.";
    return false;
  }

  if (!reference_lines_[major_refefrence_line_index_].curve().FindProjection(
        point, &proj_point_on_major_ref_line_)) {
    LOG_ERR << "Failed to get projected point on current reference line.";
    return false;
  }

#if ENABLE_REFERENCE_LINE_SET_TRACE
  std::cout << "The index of major reference line is "
            << major_refefrence_line_index_ << std::endl;
#endif

#if ENABLE_REFERENCE_LINE_SET_PERFORMANCE_TEST
  /// Performance of constructing reference lines (End)
  std::cout << "Constructing reference lines spend "
            << performance_timer_constructing_reference_lines.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_REFERENCE_LINE_SET_TRACE
  std::cout << "<<<<<<<<< Construct ReferenceLine #########" << std::endl;
#endif

  return true;
}

/// TODO: 暂时必须传递当前的车辆位置（因为Trim函数中将更新当前车辆位置的投影点）
/// TODO: 2022-12-28 可能存在相机的当前车道线与地图的当前车道线不一致的情况，需要避免此种情况
bool ReferenceLineSet::TrimReferenceLine(
    Int32_t ref_line_index,
    const common::Path& tar_curve,
    const common::Vec2d& start_point,
    map_var_t* trim_start_s,
    map_var_t* trim_end_s) {
#if ENABLE_REFERENCE_LINE_SET_TRACE
  printf("### TrimReferenceLine (Begin) -->\n");
#endif
  *trim_start_s = 0;
  *trim_end_s = 0;
  if (!IsValidReferenceLineIndex(ref_line_index)) {
    LOG_ERR << "Invalid reference line index.";
    return false;
  }

  ReferenceLine& ref_line = reference_lines_[ref_line_index];

  Int32_t proj_on_ref_curve_idx = 0;
  common::PathPoint proj_on_ref_curve;
  if (!ref_line.curve().FindProjection(
        start_point, &proj_on_ref_curve_idx, &proj_on_ref_curve)) {
    LOG_ERR << "Failed to find projection on reference curve.";
    return false;
  }
  Int32_t proj_on_tar_curve_idx = 0;
  common::PathPoint proj_on_tar_curve;
  if (!tar_curve.FindProjection(
        start_point, &proj_on_tar_curve_idx, &proj_on_tar_curve)) {
    LOG_ERR << "Failed to find projection on target curve.";
    return false;
  }

  common::Vec2d pos_diff = proj_on_tar_curve.point - proj_on_ref_curve.point;
  map_var_t heading_diff =
      common::AngleDiff(proj_on_ref_curve.heading, proj_on_tar_curve.heading);
#if ENABLE_REFERENCE_LINE_SET_TRACE
  printf("    >>> start h_diff=%0.1f\n", common::com_rad2deg(heading_diff));
#endif
  /// Note: 避免角度旋转导致的远处偏差较大, 暂时不旋转
  heading_diff = 0.0F;
  common::Matrix<map_var_t, 2, 1> point_conv;
  common::Matrix<map_var_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<map_var_t, 2, 1> rotate_center(
        proj_on_ref_curve.point.x(), proj_on_ref_curve.point.y());
  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
      points_2d = temp_data_.sample_2d_points;
  points_2d.Clear();
  common::Rotate_2D<map_var_t>(rotate_center, heading_diff, &mat_conv);
  common::Translate_2D(pos_diff.x(), pos_diff.y(), &mat_conv);

  for (Int32_t i = 0; i < ref_line.curve().points().Size(); ++i) {
    point_conv(0) = ref_line.curve().points()[i].x();
    point_conv(1) = ref_line.curve().points()[i].y();
    common::TransformVert_2D(mat_conv, &point_conv);
    points_2d.PushBack(common::Vec2d(point_conv(0), point_conv(1)));
  }

  common::Path& ref_path = temp_data_.path_list[0];
  if (!ref_path.Construct(points_2d)) {
    LOG_ERR << "Failed to construct reference path.";
    return false;
  }
  if (!ref_path.FindProjection(
        start_point, &proj_on_ref_curve_idx, &proj_on_ref_curve)) {
    LOG_ERR << "Failed to find projection on reference curve.";
    return false;
  }

  common::Vec2d trim_end_point = tar_curve.points().Back();
  Int32_t ref_path_seg_1_start_idx = 0;
  Int32_t ref_path_seg_1_end_idx = proj_on_ref_curve_idx - 1;
  Int32_t tar_path_seg_2_start_idx = proj_on_tar_curve_idx;
  Int32_t tar_path_seg_2_end_idx = -1;
  Int32_t ref_path_seg_3_start_idx = -1;
  Int32_t ref_path_seg_3_end_idx = -1;
  common::Matrix<map_var_t, 3, 3> mat_conv_seg_3;
  mat_conv_seg_3.SetIdentity();

  if ((tar_curve.total_length() - proj_on_tar_curve.s) >
      (ref_path.total_length() - proj_on_ref_curve.s)) {
    // 输入轨迹比参考线长
    tar_path_seg_2_end_idx = tar_curve.points().Size() - 1;
    ref_path_seg_3_start_idx = -1;
    ref_path_seg_3_end_idx = -1;
  } else {
    bool find_ret = false;
    map_var_t tar_s = tar_curve.total_length();
    while (tar_s > (proj_on_tar_curve.s + 5.0F)) {
      Int32_t tar_idx = -1;
      common::PathPoint tar_point;
      if (!tar_curve.FindSmoothPoint(tar_s, &tar_idx, &tar_point)) {
        LOG_ERR << "Faied to find smooth point.";
        find_ret = false;
        break;
      }
      Int32_t ref_idx = -1;
      common::PathPoint ref_point;
      if (!ref_path.FindProjection(tar_point.point, &ref_idx, &ref_point)) {
        LOG_ERR << "Failed to find projection on target curve.";
        find_ret = false;
        break;
      }
#if ENABLE_REFERENCE_LINE_SET_TRACE
      printf("    >>> ref_point.l=%0.1f, front_s=%0.1f\n",
             ref_point.l, tar_s-proj_on_tar_curve.s);
#endif
      if (common::com_abs(ref_point.l) < 0.5F) {
        find_ret = true;
        trim_end_point = tar_point.point;

        if (tar_idx > tar_path_seg_2_start_idx) {
          tar_path_seg_2_end_idx = tar_idx;
        } else {
          tar_path_seg_2_end_idx = tar_idx + 1;
        }
        if (tar_path_seg_2_end_idx > (tar_curve.points().Size() - 1)) {
          tar_path_seg_2_end_idx = tar_curve.points().Size() - 1;
        }
        ref_path_seg_3_start_idx = ref_idx + 1;
        if (ref_path_seg_3_start_idx > ref_path.points().Size() - 1) {
          ref_path_seg_3_start_idx = ref_path.points().Size() - 1;
        }
        ref_path_seg_3_end_idx = ref_path.points().Size() - 1;

        common::Vec2d p_diff = tar_point.point - ref_point.point;
        map_var_t h_diff =
            common::AngleDiff(ref_point.heading, tar_point.heading);
#if ENABLE_REFERENCE_LINE_SET_TRACE
        printf("    >>> end h_diff=%0.1f\n", common::com_rad2deg(h_diff));
#endif
        /// Note: 角度偏差比较大，暂时不要旋转
        h_diff = 0.0F;
        rotate_center(0) = ref_point.point.x();
        rotate_center(1) = ref_point.point.y();
        common::Rotate_2D<map_var_t>(rotate_center, h_diff, &mat_conv_seg_3);
        common::Translate_2D(p_diff.x(), p_diff.y(), &mat_conv_seg_3);

        break;
      }
      tar_s -= 10.0F;
    }
    if (!find_ret) {
      LOG_ERR << "Can't find triming segment.";

      /* K001 longjiaoy 2022-04-13 修正轨迹规划查找左右相邻最近车道不一致问题 start*/
      ref_path_seg_1_end_idx = ref_path.points().Size() - 1;
      // tar_path_seg_2_end_idx = tar_curve.points().Size() - 1;
      /* K001 longjiaoy 2022-04-13 修正轨迹规划查找左右相邻最近车道不一致问题 end*/

      ref_path_seg_3_start_idx = -1;
      ref_path_seg_3_end_idx = -1;
    }
  }

  points_2d.Clear();
  for (Int32_t i = ref_path_seg_1_start_idx; i <= ref_path_seg_1_end_idx; ++i) {
    points_2d.PushBack(ref_path.points()[i]);
  }
  for (Int32_t i = tar_path_seg_2_start_idx; i <= tar_path_seg_2_end_idx; ++i) {
    points_2d.PushBack(tar_curve.points()[i]);
  }
  if (ref_path_seg_3_start_idx >= 0) {
    for (Int32_t i = ref_path_seg_3_start_idx; i <= ref_path_seg_3_end_idx; ++i) {
      point_conv(0) = ref_path.points()[i].x();
      point_conv(1) = ref_path.points()[i].y();
      common::TransformVert_2D(mat_conv_seg_3, &point_conv);
      points_2d.PushBack(common::Vec2d(point_conv(0), point_conv(1)));
    }
  }
  if (points_2d.Size() < 2) {
    LOG_ERR << "Invalid target path.";
    return false;
  }

  if (!ref_line.curve().Construct(points_2d)) {
    LOG_ERR << "Failed to construct curve of reference line.";
    return false;
  }

  ref_line.curve_curvature_info().Clear();
  if (!ref_line.curve().AnalyzeCurvature(&ref_line.curve_curvature_info())) {
    LOG_ERR << "Failed to get curvature of this reference line.";
    return false;
  }

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
      anchor_points = temp_data_.sample_points;
  anchor_points.Clear();
  if (!temp_data_.curve_fitting.Fit(
        ref_line.curve(), ref_line.curve_curvature_info(),
        &anchor_points)) {
    LOG_ERR << "Failed to fit reference line.";
    return false;
  }

  if (!ref_line.smooth_curve().Construct(anchor_points)) {
    LOG_ERR << "Failed to construct smooth reference line.";
    return false;
  }

  common::PathPoint trim_start_proj;
  common::PathPoint trim_end_proj;
  if (!ref_line.curve().FindProjection(start_point, &trim_start_proj)) {
    LOG_ERR << "Failed to find projection of reference line for trim start.";
    return false;
  }
  if (!ref_line.curve().FindProjection(trim_end_point, &trim_end_proj)) {
    LOG_ERR << "Failed to find projection of reference line for trim end.";
    return false;
  }
  *trim_start_s = trim_start_proj.s;
  *trim_end_s = trim_end_proj.s;
  ref_line.SetContinuousSegment(*trim_start_s, *trim_end_s);

  ref_line.smooth_curve_curvature_info().Clear();
  if (!ref_line.smooth_curve().AnalyzeCurvature(
        &ref_line.smooth_curve_curvature_info())) {
    LOG_ERR << "Failed to get curvature of this smooth reference line.";
    return false;
  }

  if (ref_line_index == major_refefrence_line_index_) {
#if 0
    if (!reference_lines_[major_refefrence_line_index_].curve().FindProjection(
          start_point, &proj_point_on_major_ref_line_)) {
      LOG_ERR << "Failed to get projected point on current reference line.";
      return false;
    }
#endif
    proj_point_on_major_ref_line_ = trim_start_proj;
  }

  return true;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool ReferenceLineSet::FindCurrentLane(
    const common::Vec2d& point, map_var_t heading, const HDMap& map) {
#if 0
  LOG_INFO(5) << "###### Find current lane ###### (begin)";
#endif

  if (!map.FindNearestLaneSynthetically(
        point, heading,
        &nearest_lane_index_, &nearest_point_on_lane_)) {
    LOG_ERR << "Failed to find nearest lane from map.";
    return false;
  }

  if (common::com_abs(nearest_point_on_lane_.l) > 5.0F) {
    LOG_WARN << "Too far from nearest lane in HD-Map, dist = "
             << nearest_point_on_lane_.l;
    return false;
  }

  if (!map.IsValidLaneIndex(nearest_lane_index_)) {
    LOG_ERR << "Failed to find nearest lane from map, the nearest lane index "
            << nearest_lane_index_ << " is invalid.";
    return false;
  }
  const Lane* nearest_lane = &(map.GetLane(nearest_lane_index_));

#if 0
  LOG_INFO(5) << "The nearest lane is (idx=" << nearest_lane_index_
              << ", id=" << nearest_lane->topology_id().id.id
              << ", l_diff=" << nearest_point_on_lane_.l
              << ", h_diff=" << common::com_rad2deg(
                   common::AngleDiff(nearest_point_on_lane_.heading, heading))
              << ").";
#endif

  if (!map.FindNeighbors(
        point, nearest_lane_index_,
        &neighbor_lanes_of_curr_pos_,
        &nearest_index_in_neighbor_lanes_of_curr_pos_)) {
    LOG_ERR << "Failed to find neighbors from map.";
    return (false);
  }

  if (neighbor_lanes_of_curr_pos_.Empty() ||
      nearest_index_in_neighbor_lanes_of_curr_pos_ < 0) {
    LOG_ERR << "Failed to find neighbors from map, "
               "neighbors is empty or nearest index is invalid";
    return (false);
  }

  NeighborsLaneInfo* nearest_neighbor = &(neighbor_lanes_of_curr_pos_[
      nearest_index_in_neighbor_lanes_of_curr_pos_]);
  if (!map.IsValidLaneIndex(nearest_neighbor->lane_index)) {
    LOG_ERR << "Failed to find neighbors from map,"
               " nearest neighbor lane is invalid.";
    return (false);
  }

#if ENABLE_REFERENCE_LINE_SET_TRACE
  std::cout << "Neighbor lanes of at current postion is: " << std::endl;
  for (Int32_t i = 0; i < neighbor_lanes_of_curr_pos_.Size(); ++i) {
    std::cout << "[" << i
              << "], valid_lat=" << (Int32_t)neighbor_lanes_of_curr_pos_[i].valid_lat
              << ", valid_lon=" << (Int32_t)neighbor_lanes_of_curr_pos_[i].valid_lon
              << ", index=" << neighbor_lanes_of_curr_pos_[i].lane_index
              << ", id=" << map.GetLane(neighbor_lanes_of_curr_pos_[i].
                                        lane_index).topology_id().id.id
              << ", flag=" << neighbor_lanes_of_curr_pos_[i].neighbor_flag
              << std::endl;
  }
  std::cout << "The nearest lane to current position is ["
            << nearest_neighbor->lane_id.id << "]." << std::endl;
#endif

  Int32_t correct_neighbor_index = -1;
  map_var_t min_abs_dist = 999999.0F;
  for (Int32_t i = 0; i < neighbor_lanes_of_curr_pos_.Size(); ++i) {
    const NeighborsLaneInfo& neighbor = neighbor_lanes_of_curr_pos_[i];
    if (!neighbor.valid_lat) {
      continue;
    }
    if (!neighbor.valid_lon) {
      continue;
    }
    if (common::com_abs(neighbor.proj_point.l -
                        nearest_neighbor->proj_point.l) < 0.5F) {
#if ENABLE_REFERENCE_LINE_SET_TRACE
      std::cout << "Lane[" << neighbor.lane_id.id
                << "] is close to the nearest lane["
                << nearest_neighbor->lane_id.id
                << "] at projection point." << std::endl;
#endif
      if (map.IsOnRoutingSegment(neighbor.lane_index, neighbor.proj_point.s)) {
#if ENABLE_REFERENCE_LINE_SET_TRACE
        std::cout << "And lane[" << neighbor.lane_id.id
                  << "] is on routing segment." << std::endl;
#endif
        const map_var_t abs_dist = common::com_abs(neighbor.proj_point.l);
        if (abs_dist < min_abs_dist) {
          min_abs_dist = abs_dist;
          correct_neighbor_index = i;
#if ENABLE_REFERENCE_LINE_SET_TRACE
          std::cout << "The lane[" << neighbor.lane_id.id
                    << "] is the nearest lane at this moment." << std::endl;
#endif
        }
      }
    }
  }

  if (correct_neighbor_index >= 0) {
    if (correct_neighbor_index !=
        nearest_index_in_neighbor_lanes_of_curr_pos_) {
#if ENABLE_REFERENCE_LINE_SET_TRACE
      std::cout << "!!! Correct nearest lane from ["
                << nearest_neighbor->lane_id.id
                << "] to [" << neighbor_lanes_of_curr_pos_[
                   correct_neighbor_index].lane_id.id
                << "]." << std::endl;
#endif
      nearest_index_in_neighbor_lanes_of_curr_pos_ = correct_neighbor_index;
      nearest_neighbor = &(neighbor_lanes_of_curr_pos_[correct_neighbor_index]);
      const Int32_t neareast_neighbor_flag = nearest_neighbor->neighbor_flag;

#if ENABLE_REFERENCE_LINE_SET_TRACE
      std::cout << "After correcting, the neighbor flag of nearest neighbor is="
                << neareast_neighbor_flag << std::endl;
#endif
      for (Int32_t i = 0; i < neighbor_lanes_of_curr_pos_.Size(); ++i) {
#if ENABLE_REFERENCE_LINE_SET_TRACE
        std::cout << "Correct [" << neighbor_lanes_of_curr_pos_[i].lane_id.id
                  << "]'s neighbor_flag from "
                  << neighbor_lanes_of_curr_pos_[i].neighbor_flag
                  << " to " << neighbor_lanes_of_curr_pos_[i].neighbor_flag -
                     neareast_neighbor_flag
                  << "." << std::endl;
#endif
        neighbor_lanes_of_curr_pos_[i].neighbor_flag -= neareast_neighbor_flag;
      }
    }
  }

  nearest_lane_index_ = nearest_neighbor->lane_index;
  nearest_point_on_lane_ = nearest_neighbor->proj_point;

#if 0
  LOG_INFO(5) << "After correcting nearest lane, the nearest lane is (idx=" << nearest_lane_index_
              << ", id=" << map.GetLane(nearest_lane_index_).topology_id().id.id
              << ", l_diff=" << nearest_point_on_lane_.l
              << ", h_diff=" << common::com_rad2deg(
                   common::AngleDiff(nearest_point_on_lane_.heading, heading))
              << ").";
#endif

  if (common::com_abs(nearest_point_on_lane_.l) > 5.0F) {
    LOG_WARN << "Too far from nearest lane in HD-Map, dist = "
             << nearest_point_on_lane_.l;
    return false;
  }
  if ((nearest_point_on_lane_.s < -0.1F) ||
      (nearest_point_on_lane_.s >
       (map.GetLane(nearest_lane_index_).central_curve().total_length()+0.1F))) {
    LOG_WARN << "Not in lane range, s=" << nearest_point_on_lane_.s;
    return false;
  }

  // LOG_INFO(3) << "nearest_point_on_lane_.s = " << nearest_point_on_lane_.s;

#if 0
  LOG_INFO(5) << "  Distance to nearest lane is " << nearest_point_on_lane_.l;
#endif

#if 0
  LOG_INFO(5) << "###### Find current lane ###### (end)";
#endif

  return true;
}

/**
 * @date       2020.08.28
 * @version    v1.1
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc    <td>First edition
 * <tr><td>2020/08/28  <td>1.1      <td>boc    <td>解决有时参考线前向距离与\n
 * 期望不一致的问题，问题的主要原因是内部计算时使用的是构建点到参考线终点的\n
 * 距离，所以内部计算时，前向距离需要加上从构建点到车辆当前位置的距离。
 * </table>
 */
bool ReferenceLineSet::FindAllReferenceLaneSegments(const HDMap& map) {
  // 查找参考线的起点时，从构建点向后的路径长
  map_var_t backward_len = param_.backward_len;
  // 查找参考线的终点时，从构建点向前的路径长
  map_var_t forward_len = param_.forward_len;
  // 车辆当前时刻所在位置到构建点的有向距离（向前为正，向后为负）
  map_var_t start_s_offset = 0.0F;
  // 查找起始位置处的所有相邻车道
  Int32_t nearest_index_in_neighbors = 0;
  common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM> neighbor_lanes;

  if (!SelectReferenceLineStartLanes(map,
                                     &start_s_offset,
                                     &nearest_index_in_neighbors,
                                     &neighbor_lanes)) {
    LOG_ERR << "Failed to select start lanes of reference line.";
    return false;
  }
  backward_len += start_s_offset;
  if (backward_len < 0.0F) {
    backward_len = 0.0F;
  }
  forward_len -= start_s_offset;

  Int32_t neighbors_size = neighbor_lanes.Size();
  Int32_t neighbor_index = nearest_index_in_neighbors;
  // 按照距离当前车道从小到大的顺序进行排序后的左右相邻车道在
  // neighbor_lanes中索引的列表。
  // 排列的顺序为：nearest_index_in_neighbors + {0,1,-1,2,-2,...}。
  // 这样排序的目的是防止可能的参考线个数超过最大参考线个数时，
  // 无参考线经过最近车道。
  common::StaticVector<Int32_t, MAX_NEIGHBOR_LANE_NUM> directed_neighbors;
  directed_neighbors.PushBack(nearest_index_in_neighbors);
  for (Int32_t i = 1; i < neighbors_size; ++i) {
    neighbor_index = nearest_index_in_neighbors + i;
    if (neighbor_index < neighbors_size) {
      directed_neighbors.PushBack(neighbor_index);
    }

    neighbor_index = nearest_index_in_neighbors - i;
    if (neighbor_index >= 0) {
      directed_neighbors.PushBack(neighbor_index);
    }
  }

  COM_CHECK(directed_neighbors.Size() == neighbors_size);

#if ENABLE_REFERENCE_LINE_SET_TRACE
  std::cout << "Directed neighbors(size=" << neighbors_size << ") are: ";
  for (Int32_t i = 0; i < neighbors_size; ++i) {
    std::cout << directed_neighbors[i] << " => ";
  }
  std::cout << " end." << std::endl;
#endif

  // 定义堆栈，用来在地图中搜索相连接的道路段
  TemporaryData::StackForFindingRefLanes& stack =
      temp_data_.stack_for_finding_ref_lanes;
  // 从每条相邻车道的起始位置开始，查找参考线对应的道路
  for (neighbor_index = 0; neighbor_index < neighbors_size; ++neighbor_index) {

    // std::cout << "  neighbor_index = " << neighbor_index << std::endl;

    // 起始道路段
    const NeighborsLaneInfo& start =
        neighbor_lanes[directed_neighbors[neighbor_index]];
    if (!start.valid_lat) {
      continue;
    }
    if (!start.valid_lon) {
      continue;
    }
    // 道路信息
    const Lane* lane = &(map.GetLane(start.lane_index));
    // 道路段
    LaneSegment lane_seg;
    lane_seg.lane_id = lane->topology_id().id;
    lane_seg.lane_index = start.lane_index;

    // 需要从起始位置后方某个距离开始生成参考线
    // start.proj_point为构建点在起始车道上的投影点
    // 1. 当起始车道为当前车道时，构建点为车辆当前位置；
    // 2. 当起始车道为当前车道的后方相连车道时，构建点为后方相连车道终点后面2米处的点。
    if (start.proj_point.s > backward_len) {
      lane_seg.start_s = start.proj_point.s - backward_len;
      lane_seg.start_s_on_ref = start_s_offset - backward_len;
    } else {
      lane_seg.start_s = 0.0F;
      lane_seg.start_s_on_ref = start_s_offset - start.proj_point.s;
    }

    // std::cout << "    lane_seg.start_s=" << lane_seg.start_s
    //           << ", start.proj_point.s=" << start.proj_point.s
    //           << ", backward_len=" << backward_len
    //           << ", lane_total_length=" << lane->central_curve().total_length()
    //           << std::endl;

    if ((lane->central_curve().total_length() - start.proj_point.s) >
        forward_len) {
      // 此条道路前方的长度足够长了
      lane_seg.end_s = start.proj_point.s + forward_len;
      lane_seg.end_s_on_ref = start_s_offset + forward_len;
      lane_seg.is_on_routing = map.IsOverlappedWithRoutingSegment(
            lane_seg.lane_index, lane_seg.start_s, lane_seg.end_s,
            &lane_seg.routing_association.section_index,
            &lane_seg.routing_association.neighbor_index);

      // std::cout << "    1: lane_seg.end_s=" << lane_seg.end_s << std::endl;

      // 将此车道段添加到参考线中
      ReferenceLine * const ref_line = reference_lines_.Allocate();
      if (Nullptr_t == ref_line) {
        LOG_ERR << "Can't append reference line, storage is full.";
        break;
      }
      ref_line->lane_segments().Clear();
      ref_line->lane_segments().PushBack(lane_seg);
      continue;
    } else if (lane->successor_lanes().Empty()) {
      // 此条道路没有向前连接的道路了
      lane_seg.end_s = lane->central_curve().total_length();
      lane_seg.end_s_on_ref = start_s_offset +
          lane->central_curve().total_length() - start.proj_point.s;
      lane_seg.is_on_routing = map.IsOverlappedWithRoutingSegment(
            lane_seg.lane_index, lane_seg.start_s, lane_seg.end_s,
            &lane_seg.routing_association.section_index,
            &lane_seg.routing_association.neighbor_index);

      // std::cout << "    2: lane_seg.end_s=" << lane_seg.end_s << std::endl;

      // 将此车道段添加到参考线中
      ReferenceLine * const ref_line = reference_lines_.Allocate();
      if (Nullptr_t == ref_line) {
        LOG_ERR << "Can't append reference line, storage is full.";
        break;
      }
      ref_line->lane_segments().Clear();
      ref_line->lane_segments().PushBack(lane_seg);
      continue;
    } else {
      // 此条道路前方的长度不够长，
      lane_seg.end_s = lane->central_curve().total_length();
      lane_seg.end_s_on_ref = start_s_offset +
          lane->central_curve().total_length() - start.proj_point.s;
      lane_seg.is_on_routing = map.IsOverlappedWithRoutingSegment(
            lane_seg.lane_index, lane_seg.start_s, lane_seg.end_s,
            &lane_seg.routing_association.section_index,
            &lane_seg.routing_association.neighbor_index);

      // std::cout << "    3: lane_seg.end_s=" << lane_seg.end_s << std::endl;
    }
    // 使用前需要将堆栈清空
    stack.Clear();
    // 将此道路段压入堆栈
    TemporaryData::StackForFindingRefLanes::StackNode*
        stack_node = stack.Allocate();
    if (Nullptr_t == stack_node) {
      LOG_ERR << "Storage of stack is full.";
      break;
    }
    stack_node->lanes.PushBack(lane_seg);
    stack_node->accumulated_s = lane_seg.end_s - start.proj_point.s;
    stack_node->access_index = 0;

    // 使用深度优先搜索算法，获取前后相连的车道
    bool storage_is_full = false;
    while (!(stack.Empty())) {
      // 取出栈顶元素
      stack_node = &(stack.Top());
      // 栈顶中保存的最后一段车道信息
      lane = &(map.GetLane(stack_node->lanes.Back().lane_index));

      if (lane->successor_lanes().Empty()) {
        // 此车道没有向前连接的车道了
        stack.Pop();
        continue;
      } else if (stack_node->access_index >= lane->successor_lanes().Size()) {
        // 此车道所有向前连接的车道都访问过了
        stack.Pop();
        continue;
      }
      if (!map.IsValidLaneIndex(
            lane->successor_lanes()[stack_node->access_index].index)) {
        // 道路索引无效
        stack.Pop();
        LOG_ERR << "Detected invalid lane index in successor lanes.";
        continue;
      }
      // 添加一条参考线
      ReferenceLine * const ref_line = reference_lines_.Allocate();
      if (Nullptr_t == ref_line) {
        // storage_is_full = true;
        LOG_ERR << "Can't append reference line, storage is full.";
        break;
      }
      ref_line->lane_segments().Clear();
      ref_line->lane_segments() = stack_node->lanes;

      while (lane->successor_lanes().Size() > 0) {
        // 此车道还有向前连接的道路
        if (!map.IsValidLaneIndex(
              lane->successor_lanes()[stack_node->access_index].index)) {
          // 此车道向前连接的道路索引无效
          LOG_ERR << "Detected invalid lane index in successor lanes.";
          break;
        }
        // 更新车道段为此车道向前连接的车道段
        const map_var_t accumulated_s = stack_node->accumulated_s;
        lane_seg.lane_index =
            lane->successor_lanes()[stack_node->access_index].index;
        lane_seg.start_s =
            lane->successor_lanes_start_point()[stack_node->access_index].s;
        lane_seg.start_s_on_ref =
            stack_node->lanes.Back().end_s_on_ref;
        // 此车道向前连接的车道信息
        lane = &(map.GetLane(
                   lane->successor_lanes()[stack_node->access_index].index));
        lane_seg.lane_id = lane->topology_id().id;

        //if (FindLaneSegmentById(ref_line->lane_segments(), lane_seg.lane_id) >= 0) {
        //  // 不循环重复添加
        //}
        if ((accumulated_s+lane->central_curve().total_length() -
             lane_seg.start_s) > forward_len) {
          // 加上向前连接的车道，参考线足够长了
          lane_seg.end_s = forward_len - accumulated_s + lane_seg.start_s;
          lane_seg.end_s_on_ref =
              lane_seg.start_s_on_ref +
              lane_seg.end_s - lane_seg.start_s;
          lane_seg.is_on_routing = map.IsOverlappedWithRoutingSegment(
                lane_seg.lane_index, lane_seg.start_s, lane_seg.end_s,
                &lane_seg.routing_association.section_index,
                &lane_seg.routing_association.neighbor_index);
          ref_line->lane_segments().PushBack(lane_seg);
          // 节点访问索引加1
          stack_node->access_index++;
          break;
        } else {
          // 加上向前连接的车道，参考线还不足够长，需要继续添加
          lane_seg.end_s = lane->central_curve().total_length();
          lane_seg.end_s_on_ref =
              lane_seg.start_s_on_ref +
              lane_seg.end_s - lane_seg.start_s;
          lane_seg.is_on_routing = map.IsOverlappedWithRoutingSegment(
                lane_seg.lane_index, lane_seg.start_s, lane_seg.end_s,
                &lane_seg.routing_association.section_index,
                &lane_seg.routing_association.neighbor_index);
        }
        // 添加此向前连接的车道到参考线中
        ref_line->lane_segments().PushBack(lane_seg);
        // 节点访问索引加1
        stack_node->access_index++;
        // 将此道路段压入堆栈
        stack_node = stack.Allocate();
        if (Nullptr_t == stack_node) {
          storage_is_full = true;
          LOG_ERR << "Storage of stack is full.";
          break;
        }
        stack_node->lanes = ref_line->lane_segments();
        stack_node->accumulated_s =
            accumulated_s + lane_seg.end_s - lane_seg.start_s;
        stack_node->access_index = 0;
      }
      if (storage_is_full) {
        break;
      }
    }
  }

  if (reference_lines_.Empty()) {
    LOG_ERR << "Failed to find lane segments for creating reference lines.";
    return false;
  }

  return true;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool ReferenceLineSet::SelectReferenceLineStartLanes(
    const HDMap& map,
    map_var_t * const s_offset,
    Int32_t * const nearest_index,
    common::StaticVector<NeighborsLaneInfo,
    MAX_NEIGHBOR_LANE_NUM> * const start_neighbor_lanes) {
  Int32_t start_lane_index = nearest_lane_index_;
  common::PathPoint start_point = nearest_point_on_lane_;
  const Lane& nearest_lane = map.GetLane(nearest_lane_index_);
  // *backward_len = param_.backward_len;
  *s_offset = 0.0F;
  *nearest_index = nearest_index_in_neighbor_lanes_of_curr_pos_;

  // 从当前车辆位置后方一定距离开始构建参考线
  if ((nearest_point_on_lane_.s < param_.backward_len) &&
      !map.GetLane(start_lane_index).predecessor_lanes().Empty()) {
    // 当前车道起点距离当前车辆位置小于指定的距离，当前车道后方有相连的车道，
    // 所以从后方的车道中选择参考线的起点
    const common::StaticVector<Lane::TopologyId,
        Lane::MAX_TOPOLOGY_CONNECTED_LANE_NUM>& predecessor_lanes =
        map.GetLane(start_lane_index).predecessor_lanes();
    Int32_t selected_lane_index = predecessor_lanes[0].index;
    if (map.IsValidLaneIndex(selected_lane_index)) {
      const Lane* selected_lane = &(map.GetLane(predecessor_lanes[0].index));
      common::PathPoint path_point;
      if (predecessor_lanes.Size() > 1) {
        // 选择与当前车道起点航向角度差最小的车道（最平滑）
        static const map_var_t kGetHeadingDiffBackwardLen = 5;
        map_var_t s = selected_lane->central_curve().total_length() -
            kGetHeadingDiffBackwardLen;
        selected_lane->central_curve().FindSmoothPoint(s, &path_point);
        map_var_t min_abs_angle_diff = common::com_abs(
              common::AngleDiff(path_point.heading,
                                nearest_lane.central_curve().headings()[0]));
        for (Int32_t i = 1; i < predecessor_lanes.Size(); ++i) {
          if (!map.IsValidLaneIndex(predecessor_lanes[i].index)) {
            LOG_ERR << "Detected invalid lane index in predecessor lanes.";
            continue;
          }
          const Lane& prev_lane = map.GetLane(predecessor_lanes[i].index);
          s = prev_lane.central_curve().total_length() -
              kGetHeadingDiffBackwardLen;
          prev_lane.central_curve().FindSmoothPoint(s, &path_point);
          const map_var_t abs_angle_diff = common::com_abs(
                common::AngleDiff(path_point.heading,
                                  nearest_lane.central_curve().headings()[0]));
          if (abs_angle_diff < min_abs_angle_diff) {
            min_abs_angle_diff = abs_angle_diff;
            selected_lane_index = predecessor_lanes[i].index;
            selected_lane = &prev_lane;
          }
        }
      }
      // Find successor lane
      map_var_t start_s_offset = 0.0F;
      for (Int32_t i = 0; i < selected_lane->successor_lanes().Size(); ++i) {
        if (selected_lane->successor_lanes()[i].index == nearest_lane_index_) {
          start_s_offset = selected_lane->successor_lanes_start_point()[i].s;
        }
      }
      // 更新查找参考线的起始位置
      static const map_var_t kGetStartPointBackwardLen = 2.0F;
      if (selected_lane->central_curve().FindSmoothPoint(
            selected_lane->central_curve().total_length() -
            kGetStartPointBackwardLen, &path_point)) {
//        *backward_len = param_.backward_len -
//            (nearest_point_on_lane_.s - start_s_offset) -
//            kGetStartPointBackwardLen;
//        if (*backward_len < 0.0F) {
//          *backward_len = 0.0F;
//        }
        *s_offset = -(nearest_point_on_lane_.s - start_s_offset) -
            kGetStartPointBackwardLen;
        start_lane_index = selected_lane_index;
        start_point = path_point;
      } else {
        LOG_ERR << "Failed to find smooth point on lane.";
      }
    } else {
      LOG_ERR << "Detected invalid lane index in predecessor lanes.";
    }
  }

  if (start_lane_index != nearest_lane_index_) {
    // 查找起始位置处的所有相邻车道
    Int32_t nearest_index_in_neighbors = 0;
    if (!map.FindNeighbors(
          start_point.point, start_lane_index,
          start_neighbor_lanes, &nearest_index_in_neighbors)) {
      LOG_ERR << "Failed to find neighbors from map.";
      return (false);
    }

    if (start_neighbor_lanes->Empty() || nearest_index_in_neighbors < 0) {
      LOG_ERR << "Failed to find neighbors from map, "
                 "neighbors is empty or nearest index is invalid";
      return (false);
    }

    if (!(*start_neighbor_lanes)[nearest_index_in_neighbors].valid_lat) {
      LOG_ERR << "Failed to find neighbors from map,"
                 " nearest neighbor lane is invalid (lat).";
      return (false);
    }
    if (!(*start_neighbor_lanes)[nearest_index_in_neighbors].valid_lon) {
      LOG_ERR << "Failed to find neighbors from map,"
                 " nearest neighbor lane is invalid (lon).";
      return (false);
    }
    *nearest_index = nearest_index_in_neighbors;
    ref_lane_segment_start_index_ = 1;
  } else {
    *start_neighbor_lanes = neighbor_lanes_of_curr_pos_;
    ref_lane_segment_start_index_ = 0;
  }

  return true;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool ReferenceLineSet::CreateAllReferenceLines(const HDMap& map) {
  if (reference_lines_.Empty()) {
    LOG_ERR << "There is no reference lane.";
    return false;
  }

  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    if (!CreateReferenceLine(map, &reference_lines_[i])) {
      LOG_ERR << "Failed to create the " << i << "th reference line.";
      return false;
    }
  }

  // 设置参考线的左右相邻标记
  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    bool find = false;
    for (Int32_t j = 0; j < neighbor_lanes_of_curr_pos_.Size(); ++j) {
      if (reference_lines_[i].FindLaneSegment(
            neighbor_lanes_of_curr_pos_[j].lane_index) >= 0) {
        reference_lines_[i].set_neighbor_flag(
              neighbor_lanes_of_curr_pos_[j].neighbor_flag);
        find = true;
        break;
      }
    }
    if (!find) {
      LOG_WARN << "Failed to set neighbor flag for reference[" << i << "].";
    }
  }

  return true;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool ReferenceLineSet::CreateReferenceLine(
    const HDMap& map, ReferenceLine * const ref_line) {
  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
      anchor_points = temp_data_.sample_points;
  anchor_points.Clear();

  Int32_t min_lane_quality = LANE_QUALITY_GOOD;
  for (Int32_t i = 0; i < ref_line->lane_segments().Size(); ++i) {
    const LaneSegment& node = ref_line->lane_segments()[i];

    map_var_t start_s = node.start_s +
        param_.anchor_point_sample_cutoff_endpoint_dist;
    if (0 == i) {
      start_s = node.start_s;
    }
    map_var_t end_s = node.end_s -
        param_.anchor_point_sample_cutoff_endpoint_dist;
    if ((ref_line->lane_segments().Size()-1) == i) {
      end_s = node.end_s;
    }

    if (!map.IsValidLaneIndex(node.lane_index)) {
      LOG_ERR << "Detected invalid lane index.";
      break;
    }

    // std::cout << "  Create reference line, start_s = " << start_s
    //           << ", end_s = " << end_s << std::endl;

    const Lane * const lane_info = &(map.GetLane(node.lane_index));
    if ((end_s - start_s) > param_.anchor_point_sample_cutoff_endpoint_dist) {
      lane_info->central_curve().GetSamplePoints(
            start_s, end_s, &anchor_points);
    } else {
      common::PathPoint path_point;
      lane_info->central_curve().FindSmoothPoint(start_s, &path_point);
      anchor_points.PushBack(path_point);
    }

    if (lane_info->quality() < min_lane_quality) {
      min_lane_quality = lane_info->quality();
    }
  }

  ref_line->set_quality(min_lane_quality);

#if 0
  std::cout << "Anchor points is:" << std::endl;
  for (Int32_t i = 0; i < anchor_points.Size(); ++i) {
    std::cout << i
              << "," << anchor_points[i].point.x()
              << "," << anchor_points[i].point.y()
              << std::endl;
  }
#endif

  if (anchor_points.Size() < 2) {
    LOG_ERR << "There are not enough points in reference line.";
    return false;
  }

  if (!ref_line->curve().Construct(anchor_points)) {
    LOG_ERR << "Failed to construct curve of reference line.";
    return false;
  }

  ref_line->curve_curvature_info().Clear();
  if (!ref_line->curve().AnalyzeCurvature(&ref_line->curve_curvature_info())) {
    LOG_ERR << "Failed to get curvature of this reference line.";
    return false;
  }

#if 1
  // 平滑轨迹
  anchor_points.Clear();
  if (!temp_data_.curve_fitting.Fit(
        ref_line->curve(), ref_line->curve_curvature_info(),
        &anchor_points)) {
    LOG_ERR << "Failed to fit reference line.";
    return false;
  }
#endif

  if (!ref_line->smooth_curve().Construct(anchor_points)) {
    LOG_ERR << "Failed to construct smooth reference line.";
    return false;
  }

  ref_line->smooth_curve_curvature_info().Clear();
  if (!ref_line->smooth_curve().AnalyzeCurvature(
        &ref_line->smooth_curve_curvature_info())) {
    LOG_ERR << "Failed to get curvature of this smooth reference line.";
    return false;
  }

  ref_line->SetContinuousSegment(0.0F, ref_line->curve().total_length());

  return true;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool ReferenceLineSet::FindCurrentReferenceLine(const HDMap& map) {
  if (reference_lines_.Empty()) {
    LOG_ERR << "The reference lines table is empty.";
    return false;
  }

  ListForFindingCurrRefLine::RefLineInfo ref_line_info;
  ListForFindingCurrRefLine list_for_finding;

  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    if (0 == reference_lines_[i].neighbor_flag()) {
      ref_line_info.ref_line_idx = i;
      ref_line_info.routing_count = 0;
      list_for_finding.Curr().PushBack(ref_line_info);
    }
  }

  if (list_for_finding.Curr().Empty()) {
    LOG_ERR << "These reference lines have not corrected topology"
               " of neighbor relationship.";
    return false;
  }

  if (1 == list_for_finding.Curr().Size()) {
    major_refefrence_line_index_ = list_for_finding.Curr()[0].ref_line_idx;
    return true;
  }

  // std::cout << "1 list_for_finding.Curr().Size()=" << list_for_finding.Curr().Size() << std::endl;

  // Selected if the lane segment is on routing
#if 0
  for (Int32_t i = ref_lane_segment_start_index_ + 1;
       i < MAX_LANE_SEGMENT_NUM; ++i) {
    Int32_t end_count = 0;
    const Int32_t curr_ref_line_size = list_for_finding.Curr().Size();
    for (Int32_t j = 0; j < curr_ref_line_size; ++j) {
      const ListForFindingCurrRefLine::RefLineInfo& cur_ref_info =
          list_for_finding.Curr()[j];
      const Int32_t ref_line_index = cur_ref_info.ref_line_idx;
      const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>& segs =
          reference_lines_[ref_line_index].lane_segments();
      if (i < segs.Size()) {
        if (segs[i].is_on_routing) {
          ref_line_info.ref_line_idx = ref_line_index;
          list_for_finding.Next().PushBack(ref_line_info);
        }
      } else {
        end_count++;
      }
    }
    if (end_count == curr_ref_line_size) {
      break;
    }
    if (1 == list_for_finding.Next().Size()) {
      major_refefrence_line_index_ = list_for_finding.Next()[0].ref_line_idx;
      return true;
    } else if (list_for_finding.Next().Size() > 1) {
      list_for_finding.Swap();
      list_for_finding.Next().Clear();
    } else {
      // nothing to do
    }
  }

  if (list_for_finding.Curr().Empty()) {
    LOG_ERR << "Failed to find current reference line "
               "by if the lane segment is on routing.";
    return false;
  } else if (1 == list_for_finding.Curr().Size()) {
    major_refefrence_line_index_ = list_for_finding.Curr()[0].ref_line_idx;
    return true;
  } else {
    // nothing to do
  }
#else
  // std::cout << "ref_lane_segment_start_index_=" << ref_lane_segment_start_index_ << std::endl;
  const Int32_t curr_ref_line_size = list_for_finding.Curr().Size();
  for (Int32_t i = 0; i < curr_ref_line_size; ++i) {
    ListForFindingCurrRefLine::RefLineInfo& cur_ref_info =
        list_for_finding.Curr()[i];
    const Int32_t ref_line_index = cur_ref_info.ref_line_idx;
    const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>& segs =
        reference_lines_[ref_line_index].lane_segments();
    for (Int32_t j = 0; j < segs.Size(); ++j) {
#if 0
      if (segs[j].is_on_routing) {
        std::cout << "  seg[" << j << "]: lane(idx=" << segs[j].lane_index
                  << ", id=" << segs[j].lane_id.id
                  << ") is on routings." << std::endl;
      } else {
        std::cout << "  seg[" << j << "]: lane(idx=" << segs[j].lane_index
                  << ", id=" << segs[j].lane_id.id
                  << ") is not on routings." << std::endl;
      }
#endif
      if (j == 0) {
        cur_ref_info.routing_count++;
        if (segs[j].is_on_routing) {
          cur_ref_info.routing_count++;
        }
      } else {
        if (segs[j].is_on_routing) {
          cur_ref_info.routing_count++;
        } else {
          break;
        }
      }
    }
  }
  Int32_t max_routing_count = 0;
  for (Int32_t i = 0; i < curr_ref_line_size; ++i) {
    ListForFindingCurrRefLine::RefLineInfo& cur_ref_info =
        list_for_finding.Curr()[i];
    if (cur_ref_info.routing_count > max_routing_count) {
      max_routing_count = cur_ref_info.routing_count;
    }
  }
  // std::cout << "max_routing_count=" << max_routing_count << std::endl;
  for (Int32_t i = 0; i < curr_ref_line_size; ++i) {
    ListForFindingCurrRefLine::RefLineInfo& cur_ref_info =
        list_for_finding.Curr()[i];
    if (cur_ref_info.routing_count >= max_routing_count) {
      list_for_finding.Next().PushBack(cur_ref_info);
    }
  }
  if (1 == list_for_finding.Next().Size()) {
    major_refefrence_line_index_ = list_for_finding.Next()[0].ref_line_idx;
    return true;
  } else if (list_for_finding.Next().Size() > 1) {
    list_for_finding.Swap();
    list_for_finding.Next().Clear();
  } else {
    // nothing to do
  }
  if (list_for_finding.Curr().Empty()) {
    LOG_ERR << "Failed to find current reference line "
               "by if the lane segment is on routing.";
    return false;
  } else if (1 == list_for_finding.Curr().Size()) {
    major_refefrence_line_index_ = list_for_finding.Curr()[0].ref_line_idx;
    return true;
  } else {
    // nothing to do
  }
#endif

  // std::cout << "2 list_for_finding.Curr().Size()=" << list_for_finding.Curr().Size() << std::endl;

  // Select by if the lane segment is more smooth.
  static const map_var_t kGetHeadingDiffForwardLen = 10;
  common::PathPoint start_point;
  common::PathPoint end_point;
  common::StaticVector<map_var_t, MAX_REFERENCE_LINE_NUM> list_abs_angle_diff;
  list_for_finding.Next().Clear();
  for (Int32_t i = 1; i < MAX_LANE_SEGMENT_NUM; ++i) {
    Int32_t end_count = 0;
    const Int32_t curr_ref_line_size = list_for_finding.Curr().Size();

    map_var_t min_abs_angle_diff = 999999.0F;
    list_abs_angle_diff.Clear();
    for (Int32_t j = 0; j < curr_ref_line_size; ++j) {
      const Int32_t ref_line_index = list_for_finding.Curr()[j].ref_line_idx;
      const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>& segs =
          reference_lines_[ref_line_index].lane_segments();
      if (i < segs.Size()) {
        const Lane& curr_lane = map.GetLane(segs[i].lane_index);
        const Lane& prev_lane = map.GetLane(segs[i-1].lane_index);
        prev_lane.central_curve().FindSmoothPoint(
              segs[i-1].end_s, &start_point);
        curr_lane.central_curve().FindSmoothPoint(
              segs[i].start_s+kGetHeadingDiffForwardLen, &end_point);
        const map_var_t abs_angle_diff = common::com_abs(
              common::AngleDiff(start_point.heading, end_point.heading));
        if (abs_angle_diff < min_abs_angle_diff) {
          min_abs_angle_diff = abs_angle_diff;
        }
        list_abs_angle_diff.PushBack(abs_angle_diff);
      } else {
        end_count++;
        list_abs_angle_diff.PushBack(0.0F);
      }
    }
    if (end_count == curr_ref_line_size) {
      break;
    }
    for (Int32_t j = 0; j < curr_ref_line_size; ++j) {
      const Int32_t ref_line_index = list_for_finding.Curr()[j].ref_line_idx;
      const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>& segs =
          reference_lines_[ref_line_index].lane_segments();
      if (i < segs.Size()) {
#if 1
        /* k006 pengc 2023-01-08 (begin) */
        // 解决分流口误匹配的问题
        if (segs[i].lane_index == nearest_lane_index_) {
          // std::cout << "See nearest lane." << std::endl;
          ref_line_info.ref_line_idx = ref_line_index;
          list_for_finding.Next().Clear();
          list_for_finding.Next().PushBack(ref_line_info);
          break;
        }
        /* k006 pengc 2023-01-08 (end) */
#endif
        if (common::com_abs(min_abs_angle_diff - list_abs_angle_diff[j]) <
            1.0E-5F) {
          ref_line_info.ref_line_idx = ref_line_index;
          list_for_finding.Next().PushBack(ref_line_info);
        }
      }
    }
    if (list_for_finding.Next().Empty()) {
      LOG_ERR << "Failed to find smooth reference lines, which is unexpected.";
      break;
    }
    if (1 == list_for_finding.Next().Size()) {
      major_refefrence_line_index_ = list_for_finding.Next()[0].ref_line_idx;
      return true;
    } else if (list_for_finding.Next().Size() > 1) {
      list_for_finding.Swap();
      list_for_finding.Next().Clear();
    } else {
      // nothing to do
    }
  }

  if (list_for_finding.Curr().Empty()) {
    LOG_ERR << "Failed to find current reference line "
               "by if the lane segment is more smooth.";
    return false;
  } else if (1 == list_for_finding.Curr().Size()) {
    major_refefrence_line_index_ = list_for_finding.Curr()[0].ref_line_idx;
    return true;
  } else {
    major_refefrence_line_index_ = list_for_finding.Curr()[0].ref_line_idx;
    LOG_WARN << "Find more then one reference line, which is unexpected.";
  }

  return true;
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t ReferenceLineSet::FindReferenceLine(
    const common::Vec2d& start_point,
    Int32_t start_lane_index,
    Int32_t end_lane_index) const {
  /* k001 pengc 2022-02-19 (begin) */
  // 优先使用主参考线
  common::PathPoint path_point;
  bool start_lane_exist_in_ref_line = false;
  bool end_lane_exist_in_ref_line = false;
  if (!IsValidReferenceLineIndex(major_refefrence_line_index_)) {
    LOG_ERR << "Invalid reference lines.";
    return (-1);
  }
  const ReferenceLine& major_ref = reference_lines_[major_refefrence_line_index_];
  if (major_ref.FindLaneSegment(start_lane_index) >= 0) {
    start_lane_exist_in_ref_line = true;
  }
  if (major_ref.FindLaneSegment(end_lane_index) >= 0) {
    end_lane_exist_in_ref_line = true;
  }
  if (start_lane_exist_in_ref_line && end_lane_exist_in_ref_line) {
    return (major_refefrence_line_index_);
  }
  if (end_lane_exist_in_ref_line) {
    if (major_ref.curve().FindProjection(start_point, &path_point)) {
      map_var_t distance = 0;
      if (path_point.s < 0.0F) {
        distance = start_point.DistanceTo(major_ref.curve().points().Front());
      } else if (path_point.s > major_ref.curve().total_length()) {
        distance = start_point.DistanceTo(major_ref.curve().points().Back());
      } else {
        distance = common::com_abs(path_point.l);
      }
      if (distance < 0.5F) {
        return (major_refefrence_line_index_);
      }
    }
  }
  /* k001 pengc 2022-02-19 (end) */

  start_lane_exist_in_ref_line = false;
  end_lane_exist_in_ref_line = false;
  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    const ReferenceLine& node = reference_lines_[i];
    const Int32_t start_lane_seg_index = node.FindLaneSegment(start_lane_index);
    if (start_lane_seg_index >= 0) {
      start_lane_exist_in_ref_line = true;
    }
    const Int32_t end_lane_seg_index = node.FindLaneSegment(end_lane_index);
    if (end_lane_seg_index >= 0) {
      end_lane_exist_in_ref_line = true;
    }
    if ((start_lane_seg_index >= 0) && (end_lane_seg_index >= 0)) {
      return (i);
    }
  }

  // if (!start_lane_exist_in_ref_line) {
  //   LOG_ERR << "The start lane (" << start_lane_index
  //           << ") is not contained in reference lines.";
  // }
  // if (!end_lane_exist_in_ref_line) {
  //   LOG_ERR << "The end lane (" << end_lane_index
  //           << ") is not contained in reference lines.";
  // }

  // 查找不到同时经过起始车道和终点车道的参考线，在所有经过终点车道的
  // 参考线中，以离起点位置最近的参考线作为对应的参考线。
  Int32_t index = -1;
  if (end_lane_exist_in_ref_line) {
    map_var_t min_distance = 999999.0F;
    for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
      const ReferenceLine& node = reference_lines_[i];
      if (node.FindLaneSegment(end_lane_index) >= 0) {
        if (node.curve().FindProjection(start_point, &path_point)) {
          map_var_t distance = 0;
          if (path_point.s < 0.0F) {
            distance = start_point.DistanceTo(node.curve().points().Front());
          } else if (path_point.s > node.curve().total_length()) {
            distance = start_point.DistanceTo(node.curve().points().Back());
          } else {
            distance = common::com_abs(path_point.l);
          }
          if (distance < min_distance) {
            min_distance = distance;
            index = i;
          }
        }
      }
    }
  } else {
    map_var_t min_distance = 999999.0F;
    common::PathPoint path_point;
    for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
      const ReferenceLine& node = reference_lines_[i];
      if (node.curve().FindProjection(start_point, &path_point)) {
        map_var_t distance = 0;
        if (path_point.s < 0.0F) {
          distance = start_point.DistanceTo(node.curve().points().Front());
        } else if (path_point.s > node.curve().total_length()) {
          distance = start_point.DistanceTo(node.curve().points().Back());
        } else {
          distance = common::com_abs(path_point.l);
        }
        if (distance < min_distance) {
          min_distance = distance;
          index = i;
        }
      }
    }
  }

  return (index);
}

/**
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t ReferenceLineSet::FindReferenceLine(
    Int32_t start_lane_index, Int32_t end_lane_index) const {
  /* k001 pengc 2022-02-19 (begin) */
  // 优先使用主参考线
  if (!IsValidReferenceLineIndex(major_refefrence_line_index_)) {
    LOG_ERR << "Invalid reference lines.";
    return (-1);
  }
  const ReferenceLine& major_ref = reference_lines_[major_refefrence_line_index_];
  if (major_ref.FindLaneSegment(start_lane_index) >= 0) {
    if (major_ref.FindLaneSegment(end_lane_index) >= 0) {
      return (major_refefrence_line_index_);
    }
  }
  /* k001 pengc 2022-02-19 (end) */

  for (Int32_t i = 0; i < reference_lines_.Size(); ++i) {
    const ReferenceLine& node = reference_lines_[i];
    if (node.FindLaneSegment(end_lane_index) >= 0) {
      if (node.FindLaneSegment(start_lane_index) >= 0) {
        return (i);
      }
    }
  }

  return (-1);
}

Int32_t ReferenceLineSet::FindLaneSegmentByProjOnRef(
    Int32_t ref_line_index,
    map_var_t s_on_ref_line,
    Int32_t * const lane_index,
    Float32_t* const s_on_lane) const {
  if (!IsValidReferenceLineIndex(ref_line_index)) {
    *lane_index = -1;
    *s_on_lane = 0.0F;
    return (-1);
  }

  const ReferenceLine& ref = reference_lines_[ref_line_index];
  if (ref.lane_segments().Empty()) {
    *lane_index = -1;
    *s_on_lane = 0.0F;
    return (-1);
  }

  return (ref.FindLaneSegment(
            /* Add offset to s of this reference line */
            s_on_ref_line + ref.lane_segments().Front().start_s_on_ref,
            lane_index, s_on_lane));
}


}  // namespace driv_map
}  // namespace phoenix
