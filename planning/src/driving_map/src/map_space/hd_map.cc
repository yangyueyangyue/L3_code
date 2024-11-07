/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       hd_map.cc
 * @brief      内部地图
 * @details    定义了内部地图的数据格式及相关访问接口
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
#include "map_space/hd_map.h"

// Header file for std::numeric_limits, to solve the compile warning of
// cpplint [build/include_what_you_use].
#include <limits>

#include "utils/log.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "geometry/icp_core.h"
#include "utils/linear_interpolation.h"
#include "pos_filter_wrapper.h"


#define ENABLE_HD_MAP_TRACE (0)
#define ENABLE_HD_MAP_PERFORMANCE_TEST (0)
#define OUTPUT_HD_MAP_WITH_APOLLO_TYPE (0)

#define ENABLE_CAMERA_MULTIPLE_LANE (1)


#if (OUTPUT_HD_MAP_WITH_APOLLO_TYPE)
#include <string>
#include <iostream>
#include <sstream>
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "modules/map/proto/map.pb.h"
#include "modules/routing/proto/routing.pb.h"
#endif


namespace phoenix {
namespace driv_map {


/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
HDMap::HDMap() {
  map_range_ = 200.0F;
  lane_range_backward_ = 200.0F;
  lane_range_forward_ = 200.0F;

  mat_convert_global_to_rel_coor_.SetIdentity();

  min_lane_point_interval_ = 1.0F;

  driving_direction_ = DRIVING_DIRECTION_FORWARD;

  hd_map_msg_head_.Clear();
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void HDMap::Clear() {
  for (Int32_t i = 0; i < lane_table_storage_.Size(); ++i) {
    lane_table_storage_[i].Clear();
  }
  lane_table_storage_.Clear();
  lanes_in_boundary_.Clear();
  lane_table_.Clear();
  map_traffic_light_table_.Clear();

  routing_.Clear();
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/29  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool HDMap::ConstructMapInfoFromPath(
    const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        pred_path) {
  // Clear old information.
  Clear();

  Int32_t pred_path_size = pred_path.Size();
  if (pred_path_size < 2) {
    return false;
  }

  // Get a memory space from lane table storage.
  Int32_t lane_storage_index = -1;
  Lane* lane_info = lane_table_storage_.Allocate(&lane_storage_index);
  if (Nullptr_t == lane_info) {
    LOG_ERR << "Can't save lane information anymore, storage is full.";
    return false;
  }

  common::Vec2d prev_point;
  map_var_t accumulated_s = 0.0F;
  for (Int32_t j = 0; j < pred_path_size; ++j) {
    const common::Vec2d& p = pred_path[j];

    // Lane Boundary
    if (j > 0) {
      accumulated_s += p.DistanceTo(prev_point);
    }

    Lane::BoundaryWidthAssociation* lane_boundary =
        lane_info->left_boundary().boundary_width_samples.Allocate();
    if (Nullptr_t != lane_boundary) {
      lane_boundary->s = accumulated_s;
      lane_boundary->width = 1.5F;
    }
    lane_boundary =
        lane_info->right_boundary().boundary_width_samples.Allocate();
    if (Nullptr_t != lane_boundary) {
      lane_boundary->s = accumulated_s;
      lane_boundary->width = 1.5F;
    }

    prev_point = p;
  }

  // Save central line to this lane.
  lane_info->central_curve().Clear();
  lane_info->central_curve().Construct(pred_path);
  if (lane_info->central_curve().points().Size() < 2) {
    LOG_ERR << "There are not enough points in center lane.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }

  // Lane ID
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  lane_info->topology_id().id.id = std::to_string(0);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  lane_info->topology_id().id.id = 0;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  lane_info->topology_id().id.id = 0;
#else
  lane_info->topology_id().id.id = 0;
#endif
  lane_info->topology_id().index = lane_storage_index;

  // Add to lane table
  const Int32_t* lane_index =
      lane_table_.Find(lane_info->topology_id().id.id);
  if (Nullptr_t != lane_index) {
    LOG_ERR << "Detected repeated lane id in lane table.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }

  LaneInBoundary* lane_in_boundary = lanes_in_boundary_.Allocate();
  if (Nullptr_t == lane_in_boundary) {
    LOG_ERR << "Can't save lane segment anymore, storage is full.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }
  lane_in_boundary->lane_id = lane_info->topology_id().id;
  lane_in_boundary->lane_index = lane_storage_index;
  lane_in_boundary->start_s = 0;
  lane_in_boundary->end_s = accumulated_s;

  lane_table_.Insert(lane_info->topology_id().id.id, lane_storage_index);

  UpdateLanesTopology();

  BuildLaneSegmentKDTree();

  return true;
}


/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/29  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool HDMap::ConstructMapInfoFromCamera(
    const ad_msg::LaneInfoCameraList& camera_lane_list) {
  // Clear old information.
  Clear();

  if (camera_lane_list.center_line_num < 1) {
    return false;
  }
  if (!camera_lane_list.msg_head.valid) {
    return false;
  }
  if (!rel_pos_list_.msg_head.valid) {
    return false;
  }

  for (Int32_t i = 0; i < camera_lane_list.center_line_num; ++i) {
    ConstructLaneInfoFromCamera(i, camera_lane_list, -1, 0.0F);
  }

  UpdateLanesTopology();

  BuildLaneSegmentKDTree();

  return true;
}

bool HDMap::ConstructMapInfoFromOtherMap(
    const HDMap& other_map,
    const common::Matrix<map_var_t, 3, 3>& mat_conv,
    const common::Path* cam_center_path) {
  /// Clear old information.
  Clear();

  /// Lanes
  for (Int32_t i = 0; i < other_map.lane_table_storage_.Size(); ++i) {
    const Lane& src_lane = other_map.lane_table_storage_[i];

    // Get a memory space from lane table storage.
    Int32_t lane_storage_index = -1;
    Lane* dst_lane = lane_table_storage_.Allocate(&lane_storage_index);
    if (Nullptr_t == dst_lane) {
      LOG_ERR << "Can't save lane information anymore, storage is full.";
      break;
    }

    // Convert global points to points related to vehicle coordinate.
    temp_data_.points_2d_1.Clear();
    common::Matrix<map_var_t, 2, 1> point_conv;
    Int32_t center_curve_points_size = src_lane.central_curve().points().Size();
    for (Int32_t j = 0; j < center_curve_points_size; ++j) {
      common::Vec2d* p = temp_data_.points_2d_1.Allocate();
      if (Nullptr_t == p) {
        LOG_ERR << "Can't add points, storage is full.";
        break;
      }
      const common::Vec2d& src_p = src_lane.central_curve().points()[j];
      point_conv(0) = src_p.x();
      point_conv(1) = src_p.y();
      common::TransformVert_2D(mat_conv, &point_conv);
      p->set_x(point_conv(0));
      p->set_y(point_conv(1));
    }
    // Save central line to this lane.
    dst_lane->central_curve().Clear();
    dst_lane->central_curve().Construct(temp_data_.points_2d_1);
    if (dst_lane->central_curve().points().Size() < 2) {
      LOG_ERR << "There are not enough points in path of this lane.";
      dst_lane->Clear();
      lane_table_storage_.PopBack();
      continue;
    }
    // Lane Boundarys
    Int32_t boundary_sample_size = src_lane.left_boundary().curve.Size();
    for (Int32_t j = 0; j < boundary_sample_size; ++j) {
      const common::Vec2d& src_p = src_lane.left_boundary().curve[j];
      point_conv(0) = src_p.x();
      point_conv(1) = src_p.y();
      common::TransformVert_2D(mat_conv, &point_conv);
      dst_lane->left_boundary().curve.PushBack(
            common::Vec2d(point_conv(0), point_conv(1)));
    }
    boundary_sample_size = src_lane.right_boundary().curve.Size();
    for (Int32_t j = 0; j < boundary_sample_size; ++j) {
      const common::Vec2d& src_p = src_lane.right_boundary().curve[j];
      point_conv(0) = src_p.x();
      point_conv(1) = src_p.y();
      common::TransformVert_2D(mat_conv, &point_conv);
      dst_lane->right_boundary().curve.PushBack(
            common::Vec2d(point_conv(0), point_conv(1)));
    }
    boundary_sample_size = src_lane.left_boundary().boundary_width_samples.Size();
    for (Int32_t j = 0; j < boundary_sample_size; ++j) {
      const Lane::BoundaryWidthAssociation& src_boundary =
          src_lane.left_boundary().boundary_width_samples[j];
      Lane::BoundaryWidthAssociation* dst_boundary =
          dst_lane->left_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t != dst_boundary) {
        dst_boundary->s = src_boundary.s;
        dst_boundary->width = src_boundary.width;
      }
    }
    boundary_sample_size = src_lane.right_boundary().boundary_width_samples.Size();
    for (Int32_t j = 0; j < boundary_sample_size; ++j) {
      const Lane::BoundaryWidthAssociation& src_boundary =
          src_lane.right_boundary().boundary_width_samples[j];
      Lane::BoundaryWidthAssociation* dst_boundary =
          dst_lane->right_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t != dst_boundary) {
        dst_boundary->s = src_boundary.s;
        dst_boundary->width = src_boundary.width;
      }
    }
    boundary_sample_size = src_lane.left_boundary().boundary_type_samples.Size();
    for (Int32_t j = 0; j < boundary_sample_size; ++j) {
      const Lane::BoundaryTypeAssociation& src_boundary =
          src_lane.left_boundary().boundary_type_samples[j];
      Lane::BoundaryTypeAssociation* dst_boundary =
          dst_lane->left_boundary().boundary_type_samples.Allocate();
      if (Nullptr_t != dst_boundary) {
        dst_boundary->s = src_boundary.s;
        dst_boundary->type = src_boundary.type;
      }
    }
    boundary_sample_size = src_lane.right_boundary().boundary_type_samples.Size();
    for (Int32_t j = 0; j < boundary_sample_size; ++j) {
      const Lane::BoundaryTypeAssociation& src_boundary =
          src_lane.right_boundary().boundary_type_samples[j];
      Lane::BoundaryTypeAssociation* dst_boundary =
          dst_lane->right_boundary().boundary_type_samples.Allocate();
      if (Nullptr_t != dst_boundary) {
        dst_boundary->s = src_boundary.s;
        dst_boundary->type = src_boundary.type;
      }
    }

    // Lane ID
    dst_lane->topology_id().id = src_lane.topology_id().id;
    dst_lane->topology_id().index = lane_storage_index;

    const Int32_t* lane_index =
        lane_table_.Find(dst_lane->topology_id().id.id);
    if (Nullptr_t != lane_index) {
      LOG_ERR << "Detected repeated lane id in lane table.";
      dst_lane->Clear();
      lane_table_storage_.PopBack();
      continue;
    }

    LaneInBoundary* lane_in_boundary = lanes_in_boundary_.Allocate();
    if (Nullptr_t == lane_in_boundary) {
      LOG_ERR << "Can't save lane segment anymore, storage is full.";
      dst_lane->Clear();
      lane_table_storage_.PopBack();
      continue;
    }
    Int32_t src_lane_in_boundary_idx =
        FindLaneFromLanesInBoundary(src_lane.topology_id().id);
    if (src_lane_in_boundary_idx < 0) {
      LOG_ERR << "Failed to find lanes in boundary.";
    }
    lane_in_boundary->lane_id = dst_lane->topology_id().id;
    lane_in_boundary->lane_index = lane_storage_index;
    if (src_lane_in_boundary_idx >= 0) {
      const LaneInBoundary& src_lane_in_boundary =
          other_map.lanes_in_boundary_[src_lane_in_boundary_idx];
      lane_in_boundary->start_s = src_lane_in_boundary.start_s;
      lane_in_boundary->end_s = src_lane_in_boundary.end_s;
    } else {
      lane_in_boundary->start_s = 0;
      lane_in_boundary->end_s = dst_lane->central_curve().total_length();
    }

    // Lane quality
    dst_lane->set_quality(src_lane.quality());
    // 车道类型
    dst_lane->set_lane_type(src_lane.lane_type());
    // 车道转弯类型
    dst_lane->set_turn_type(src_lane.turn_type());
    // 车道方向
    dst_lane->set_direction(src_lane.direction());
    // 车道限速
    dst_lane->set_speed_limit_high(src_lane.speed_limit_high());
    dst_lane->set_speed_limit_low(src_lane.speed_limit_low());

    //车道坡度
    dst_lane->slope_samples() = src_lane.slope_samples();

    // Topologys
    dst_lane->left_neighbor_forward_lanes() =
        src_lane.left_neighbor_forward_lanes();
    dst_lane->right_neighbor_forward_lanes() =
        src_lane.right_neighbor_forward_lanes();
    dst_lane->left_neighbor_reverse_lanes() =
        src_lane.left_neighbor_reverse_lanes();
    dst_lane->right_neighbor_reverse_lanes() =
        src_lane.right_neighbor_reverse_lanes();
    dst_lane->predecessor_lanes() = src_lane.predecessor_lanes();
    dst_lane->successor_lanes() = src_lane.successor_lanes();

    lane_table_.Insert(dst_lane->topology_id().id.id, lane_storage_index);
  }

  /* k004 pengc 2022-12-26 (begin) */
  // 使用相机车道中心线替换地图中的车道中心线
  if (Nullptr_t != cam_center_path) {
    if (cam_center_path->total_length() > 15.0F) {
      BuildLaneSegmentKDTree();
      ReplaceCenterLine(*cam_center_path);
    }
  }
  /* k004 pengc 2022-12-26 (end) */

  UpdateLanesTopology();

  BuildLaneSegmentKDTree();

  /// Traffic lights
  for (Int32_t i = 0; i < other_map.map_traffic_light_table_.Size(); ++i) {
    const MapTrafficLight& src_signal = other_map.map_traffic_light_table_[i];

    MapTrafficLight* dst_signal = map_traffic_light_table_.Allocate();
    if (Nullptr_t == dst_signal) {
      LOG_ERR << "Can't save signal information anymore, storage is full.";
      break;
    }
    // id
    dst_signal->id.id = src_signal.id.id;
    // stop line
    common::Matrix<map_var_t, 2, 1> point_conv;
    point_conv(0) = src_signal.stop_line.start().x();
    point_conv(1) = src_signal.stop_line.start().y();
    common::TransformVert_2D(mat_conv, &point_conv);
    dst_signal->stop_line.set_start(common::Vec2d(point_conv(0), point_conv(1)));
    point_conv(0) = src_signal.stop_line.end().x();
    point_conv(1) = src_signal.stop_line.end().y();
    common::TransformVert_2D(mat_conv, &point_conv);
    dst_signal->stop_line.set_end(common::Vec2d(point_conv(0), point_conv(1)));
  }

  // LOG_INFO(5) << "After construct map from other, the map is:";
  // PrintHDMap();

  return true;
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t HDMap::GetLaneIndexById(const ID& id) const {
  const Int32_t* const lane_index = lane_table_.Find(id.id);
  if (Nullptr_t != lane_index) {
    return (*lane_index);
  }

  return (-1);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::FindNearestLane(
    const common::Vec2d& point,
    Int32_t * const nearest_lane_index,
    common::PathPoint * const nearest_point) const {
  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return (false);
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of finding nearest point from map (Start)
  phoenix::common::Stopwatch performance_timer;
#endif

  *nearest_lane_index = -1;
  Int32_t obj_index = 0;
  map_var_t min_sq_distance = 0.0F;
  common::StaticVector<common::AABBoxKDTreeNodeAssociation, 32>
      tree_nodes_stack;

  const LaneSegmentObj* obj = lane_segment_kdtree_.FindNearest(
        tree_nodes_stack, point, CalcSquareDistToPt(point, *this),
        &obj_index, &min_sq_distance);
  if (Nullptr_t == obj) {
    LOG_ERR << "Failed to find nearest point from map.";
    return false;
  }

  if ((obj->lane_index < 0) ||
      (obj->lane_index >= lane_table_storage_.Size())) {
    LOG_ERR << "Detected invalid lane index " << obj->lane_index;
    return false;
  }

  const common::Path& curve =
      lane_table_storage_[obj->lane_index].central_curve();

  if ((obj->object_index < 0) ||
      (obj->object_index >= (curve.points().Size()-1))) {
    LOG_ERR << "Detected invalid object index " << obj->object_index;
    return false;
  }

  bool ret = curve.GetSmoothNearestPointOnSegment(
        point, obj->object_index, nearest_point);

  *nearest_lane_index = obj->lane_index;

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of finding nearest point from map (End)
  std::cout << "Find nearest point from map spend "
            << performance_timer.Elapsed()
            << "ms." << std::endl;
#endif

  return (ret);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::FindNearestLane(
    const common::Vec2d& point,
    Float32_t heading,
    Int32_t * const nearest_lane_index,
    common::PathPoint * const nearest_point) const {
  static const Float32_t kMaxAngleDiff = common::com_deg2rad(90.0F);

  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return (false);
  }

  *nearest_lane_index = -1;
  // map_var_t min_distance_to_lane = std::numeric_limits<map_var_t>::infinity();
  map_var_t min_distance_to_lane = 1.0E10F;
  common::PathPoint path_point;

  for (Int32_t i = 0; i < lane_table_storage_.Size(); ++i) {
    const common::Path& curve = lane_table_storage_[i].central_curve();

    if (!curve.FindNearest(point, &path_point)) {
      LOG_ERR << "Failed to find nearest point from central curve of lane.";
      continue;
    }
    const map_var_t angle_diff = common::AngleDiff(path_point.heading, heading);

    if (common::com_abs(angle_diff) < kMaxAngleDiff) {
      const map_var_t distance = common::com_abs(path_point.l);
      if (distance < min_distance_to_lane) {
        min_distance_to_lane = distance;

        *nearest_lane_index = i;
        *nearest_point = path_point;
      }
    }
  }

  if (*nearest_lane_index < 0) {
    return false;
  }

  return true;
}

/**
 * @brief 根据距离及航向综合查找最近的车道, 避免在分流口/合流口查找到不合适的最近车道
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/01/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool HDMap::FindNearestLaneSynthetically(
    const common::Vec2d& point,
    Float32_t heading,
    Int32_t * const nearest_lane_index,
    common::PathPoint * const nearest_point) const {
  static const Float32_t kMaxAngleDiff = common::com_deg2rad(90.0F);
  static const map_var_t kSearchRadius= 2.0F;
  static const map_var_t kAngleDiffRatio = 0.1F;
  static const map_var_t kRoutingCostRatio = 0.5F;

  *nearest_lane_index = -1;

  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return (false);
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of finding nearest lane from map (Start)
  phoenix::common::Stopwatch performance_timer;
#endif

  common::StaticVector<common::AABBoxKDTreeNodeAssociation, 32> tree_nodes_stack;

#if 0
  Int32_t obj_index = 0;
  map_var_t min_sq_distance = 0.0F;

  const LaneSegmentObj* obj = lane_segment_kdtree_.FindNearest(
        tree_nodes_stack, point, CalcSquareDistToPt(point, *this),
        &obj_index, &min_sq_distance);
  if (Nullptr_t == obj) {
    LOG_ERR << "Failed to find nearest point from map.";
    return false;
  }

  if ((obj->lane_index < 0) ||
      (obj->lane_index >= lane_table_storage_.Size())) {
    LOG_ERR << "Detected invalid lane index " << obj->lane_index;
    return false;
  }

  const common::Path& nearest_lane_path =
      lane_table_storage_[obj->lane_index].central_curve();
  if (nearest_lane_path.points().Size() < 2) {
    LOG_ERR << "Detected invalid lane center path.";
    return false;
  }

  if ((obj->object_index < 0) ||
      (obj->object_index >= (nearest_lane_path.points().Size()-1))) {
    LOG_ERR << "Detected invalid object index " << obj->object_index;
    return false;
  }

  if (!nearest_lane_path.GetSmoothNearestPointOnSegment(
        point, obj->object_index, nearest_point)) {
    LOG_ERR << "Failed to get smooth nearest point.";
    return false;
  }
  *nearest_lane_index = obj->lane_index;
#else
  map_var_t min_distance_to_lane = 1.0E10F;
  common::PathPoint path_point;

  for (Int32_t i = 0; i < lane_table_storage_.Size(); ++i) {
    const common::Path& curve = lane_table_storage_[i].central_curve();

    if (!curve.FindNearest(point, &path_point)) {
      LOG_ERR << "Failed to find nearest point from central curve of lane.";
      continue;
    }
    const map_var_t angle_diff = common::AngleDiff(path_point.heading, heading);

    if (common::com_abs(angle_diff) < kMaxAngleDiff) {
      const map_var_t distance = common::com_abs(path_point.l);
      if (distance < min_distance_to_lane) {
        min_distance_to_lane = distance;

        *nearest_lane_index = i;
        *nearest_point = path_point;
      }
    }
  }

  if ((*nearest_lane_index < 0) ||
      (*nearest_lane_index >= lane_table_storage_.Size())) {
    return false;
  }
  const common::Path& nearest_lane_path =
      lane_table_storage_[*nearest_lane_index].central_curve();
  if (nearest_lane_path.points().Size() < 2) {
    LOG_ERR << "Detected invalid lane center path.";
    return false;
  }
#endif

#if 0
  LOG_INFO(5) << "The original nearest lane is"
              << " (idx=" << *nearest_lane_index
              << ", id=" << lane_table_storage_[*nearest_lane_index].topology_id().id.id
              << ").";
#endif

  common::StaticVector<Int32_t, MAX_LANE_NUM> obj_idxs_in_range;
  lane_segment_kdtree_.FindObjects(
        tree_nodes_stack, nearest_point->point, kSearchRadius,
        CalcSquareDistToPt(nearest_point->point, *this), &obj_idxs_in_range);
  if (obj_idxs_in_range.Size() > 1) {
    map_var_t heading_of_lane = nearest_point->heading;
    map_var_t forward_len_of_lane =
        nearest_lane_path.total_length() - nearest_point->s;
    common::PathPoint curr_point_on_lane;
    common::PathPoint next_point_on_lane;
    if (forward_len_of_lane > 8.0F) {
      nearest_lane_path.FindSmoothPoint(
            nearest_point->s+8.0F, &next_point_on_lane);
      heading_of_lane = common::GetHeadingFromSegment(
            nearest_point->point, next_point_on_lane.point);
    } else if (forward_len_of_lane > 3.0F) {
      heading_of_lane = common::GetHeadingFromSegment(
            nearest_point->point, nearest_lane_path.points().Back());
    } else {
      heading_of_lane = nearest_point->heading;
    }

    /* k006 pengc 2023-02-01 (begin) */
    // 修改分流口最近车道匹配错误的问题
#if 1
    Int32_t routing_cost = 1;
    if (IsOnRoutingSegment(*nearest_lane_index, nearest_point->s)) {
      routing_cost = 0;
    }
#else
    Int32_t routing_cost = 0;
#endif
    /* k006 pengc 2023-02-01 (end) */

    map_var_t min_diff =
        common::com_abs(nearest_point->l) +
        kAngleDiffRatio * common::com_rad2deg(
          common::com_abs(common::AngleDiff(heading_of_lane, heading))) +
        kRoutingCostRatio * routing_cost;
    nearest_point->heading = heading_of_lane;

    for (Int32_t i = 0; i < obj_idxs_in_range.Size(); ++i) {
      const LaneSegmentObj* obj =
          lane_segment_kdtree_.GetObjectByIndex(obj_idxs_in_range[i]);
      if (Nullptr_t == obj) {
        LOG_ERR << "Invalid LaneSegmentObj";
        continue;
      }
      if ((obj->lane_index < 0) ||
          (obj->lane_index >= lane_table_storage_.Size())) {
        LOG_ERR << "Detected invalid lane index " << obj->lane_index;
        continue;
      }

      const common::Path& curve =
          lane_table_storage_[obj->lane_index].central_curve();

      // std::cout << "   *** [" << i << "]: idx=" << lane_idx_list[i]
      //           << ", id=" << lane.topology_id().id.id
      //           << std::endl;

      if (!curve.FindProjection(point, &curr_point_on_lane)) {
        LOG_ERR << "Failed to find projection.";
        continue;
      }
      if ((curr_point_on_lane.s < -0.1F) ||
          (curr_point_on_lane.s > (curve.total_length()+0.1F))) {
        // std::cout << "      not in valid range, s=" << curr_point_on_lane.s << std::endl;
        continue;
      }
      forward_len_of_lane = curve.total_length() - curr_point_on_lane.s;

      if (forward_len_of_lane > 8.0F) {
        curve.FindSmoothPoint(
              curr_point_on_lane.s+8.0F, &next_point_on_lane);
        heading_of_lane = common::GetHeadingFromSegment(
              curr_point_on_lane.point, next_point_on_lane.point);
      } else if (forward_len_of_lane > 3.0F) {
        heading_of_lane = common::GetHeadingFromSegment(
              curr_point_on_lane.point, curve.points().Back());
      } else {
        heading_of_lane = curr_point_on_lane.heading;
      }

      /* k006 pengc 2023-02-01 (begin) */
      // 修改分流口最近车道匹配错误的问题
#if 1
      routing_cost = 1;
      if (IsOnRoutingSegment(obj->lane_index, curr_point_on_lane.s)) {
        routing_cost = 0;
      }
#else
      routing_cost = 0;
#endif
      /* k006 pengc 2023-02-01 (end) */

      map_var_t diff =
          common::com_abs(curr_point_on_lane.l) +
          kAngleDiffRatio * common::com_rad2deg(
            common::com_abs(common::AngleDiff(heading_of_lane, heading))) +
          kRoutingCostRatio * routing_cost;;
#if 0
      LOG_INFO(5) << "  [" << i
                  << "]: lane(idx=" << obj->lane_index
                  << ", id=" << lane_table_storage_[obj->lane_index].topology_id().id.id
                  << ") lat_diff=" << curr_point_on_lane.l
                  << ", angle_diff=" << common::com_rad2deg(common::AngleDiff(heading_of_lane, heading))
                  << ", routing_cost=" << routing_cost
                  << ", diff=" << diff;
#endif
      if (diff < min_diff) {
        min_diff = diff;

        *nearest_lane_index = obj->lane_index;
        *nearest_point = curr_point_on_lane;
        nearest_point->heading = heading_of_lane;
      }
    }
  }

#if 0
  LOG_INFO(5) << "After searching nearest lane, the nearest lane is"
              << " (idx=" << *nearest_lane_index
              << ", id=" << lane_table_storage_[*nearest_lane_index].topology_id().id.id
              << ").";
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of finding nearest lane from map (End)
  std::cout << "Find nearest lane synthetically from map spend "
            << performance_timer.Elapsed()
            << "ms." << std::endl;
#endif

  return (true);
}

/**
 * @brief 查找查询点一定范围内的车道
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/01/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool HDMap::FindLanes(
    const common::Vec2d& point,
    map_var_t distance,
    common::StaticVector<Int32_t, MAX_LANE_NUM>* const lane_idx_list) const {
  lane_idx_list->Clear();

  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return (false);
  }

  common::StaticVector<common::AABBoxKDTreeNodeAssociation, 32>
      tree_nodes_stack;
  common::StaticVector<Int32_t, MAX_LANE_NUM> obj_idxs_in_range;

  obj_idxs_in_range.Clear();
  lane_segment_kdtree_.FindObjects(
        tree_nodes_stack, point, distance,
        CalcSquareDistToPt(point, *this), &obj_idxs_in_range);

  for (Int32_t i = 0; i < obj_idxs_in_range.Size(); ++i) {
    const LaneSegmentObj* obj =
        lane_segment_kdtree_.GetObjectByIndex(obj_idxs_in_range[i]);
    if (Nullptr_t == obj) {
      LOG_ERR << "Invalid LaneSegmentObj";
      continue;
    }
    if ((obj->lane_index < 0) ||
        (obj->lane_index >= lane_table_storage_.Size())) {
      LOG_ERR << "Detected invalid lane index " << obj->lane_index;
      continue;
    }

    lane_idx_list->PushBack(obj->lane_index);
  }

  return (true);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::FindProjOfNearestLane(
    const common::Vec2d& point,
    Int32_t* const nearest_lane_index,
    common::PathPoint* const proj_point) const {
  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return (false);
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of finding nearest point from map (Start)
  phoenix::common::Stopwatch performance_timer;
#endif

  *nearest_lane_index = -1;
  Int32_t obj_index = 0;
  map_var_t min_sq_distance = 0.0F;
  common::StaticVector<common::AABBoxKDTreeNodeAssociation, 32>
      tree_nodes_stack;

  const LaneSegmentObj* obj = lane_segment_kdtree_.FindNearest(
        tree_nodes_stack, point, CalcSquareDistToPt(point, *this),
        &obj_index, &min_sq_distance);
  if (Nullptr_t == obj) {
    LOG_ERR << "Failed to find nearest point from map.";
    return false;
  }

  if ((obj->lane_index < 0) ||
      (obj->lane_index >= lane_table_storage_.Size())) {
    LOG_ERR << "Detected invalid lane index " << obj->lane_index;
    return false;
  }

  const common::Path& curve =
      lane_table_storage_[obj->lane_index].central_curve();

  if ((obj->object_index < 0) ||
      (obj->object_index >= (curve.points().Size()-1))) {
    LOG_ERR << "Detected invalid object index " << obj->object_index;
    return false;
  }

  bool ret = curve.GetSmoothPojectivePointOnSegment(
        point, obj->object_index, proj_point);

  *nearest_lane_index = obj->lane_index;

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of finding nearest point from map (End)
  std::cout << "Find nearest point from map spend "
            << performance_timer.Elapsed()
            << "ms." << std::endl;
#endif

  return (ret);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::FindNeighbors(
    const common::Vec2d& point,
    const Int32_t lane_index,
    common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM> * const
                          sorted_neighbor_lanes,
    Int32_t * const nearest_neighbor_index) const {
  static const map_var_t kMaxAllowedLaneWidthErr = 2.0F;
  // clear neighbor lanes table
  sorted_neighbor_lanes->Clear();
  *nearest_neighbor_index = -1;
  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return false;
  }

  if ((lane_index < 0) || (lane_index >= lane_table_storage_.Size())) {
    LOG_ERR << "Invalid lane index " << lane_index;
    return false;
  }

  Int32_t tmp_lane_index = lane_index;
  common::PathPoint path_point;

  // Find all neighbor lanes of the input lane
  // Find in right side
  for (Int32_t i = 0; i < MAX_NEIGHBOR_LANE_NUM; ++i) {
    const Lane& lane = lane_table_storage_[tmp_lane_index];
    if (!lane.central_curve().FindProjection(point, &path_point)) {
      LOG_ERR << "Failed to find the projection points on lane.";
      break;
    }
    NeighborsLaneInfo* const neighbor_lane = sorted_neighbor_lanes->Allocate();
    if (Nullptr_t == neighbor_lane) {
      LOG_ERR << "Can't save neighbor lane, storage is full.";
      break;
    }
    neighbor_lane->valid_lat = false;
    neighbor_lane->valid_lon = false;
    neighbor_lane->lane_id = lane.topology_id().id;
    neighbor_lane->lane_index = tmp_lane_index;
    neighbor_lane->proj_point = path_point;

    if (lane.right_neighbor_forward_lanes().Empty()) {
      break;
    }
    tmp_lane_index = lane.right_neighbor_forward_lanes()[0].index;
    if (!IsValidLaneIndex(tmp_lane_index)) {
      LOG_ERR << "Detected invalid lane index in neighbor lanes.";
      break;
    }
    if (FindLaneFromNeighborLanesTable(
          tmp_lane_index, *sorted_neighbor_lanes)) {
      LOG_ERR << "Detected unexpected topology relationship in neighbor lanes.";
      break;
    }
  }
  // Find in left side
  if (!lane_table_storage_[lane_index].left_neighbor_forward_lanes().Empty()) {
    tmp_lane_index = lane_table_storage_[lane_index].
        left_neighbor_forward_lanes()[0].index;
    if (IsValidLaneIndex(tmp_lane_index)) {
      for (Int32_t i = 0; i < MAX_NEIGHBOR_LANE_NUM; ++i) {
        if (FindLaneFromNeighborLanesTable(
              tmp_lane_index, *sorted_neighbor_lanes)) {
          LOG_ERR << "Detected unexpected topology relationship "
                     "in neighbor lanes.";
          break;
        }
        const Lane& lane = lane_table_storage_[tmp_lane_index];
        if (!lane.central_curve().FindProjection(point, &path_point)) {
          LOG_ERR << "Failed to find the projection points on lane.";
          break;
        }
        NeighborsLaneInfo* const neighbor_lane = sorted_neighbor_lanes->Allocate();
        if (Nullptr_t == neighbor_lane) {
          LOG_ERR << "Can't save neighbor lane, storage is full.";
          break;
        }
        neighbor_lane->valid_lat = false;
        neighbor_lane->valid_lon = false;
        neighbor_lane->lane_id = lane.topology_id().id;
        neighbor_lane->lane_index = tmp_lane_index;
        neighbor_lane->proj_point = path_point;

        if (lane.left_neighbor_forward_lanes().Empty()) {
          break;
        }
        tmp_lane_index = lane.left_neighbor_forward_lanes()[0].index;
        if (!IsValidLaneIndex(tmp_lane_index)) {
          LOG_ERR << "Detected invalid lane index in neighbor lanes.";
          break;
        }
      }
    } else {
      LOG_ERR << "Detected invalid lane index in neighbor lanes.";
    }
  }

  if (sorted_neighbor_lanes->Empty()) {
    LOG_ERR << "The neighbor lanes table is empty, which is unexpected.";
    return false;
  }

  /// TODO: 需要考虑多车道合流/分流的情况
  // Sort these lanes according to from right to left
  sorted_neighbor_lanes->Sort(CmpNeighborLane());

  // Find the nearest lane to the input point
  *nearest_neighbor_index = 0;
  map_var_t min_abs_dist = common::com_abs(
        sorted_neighbor_lanes->GetData(0).proj_point.l);
  for (Int32_t i = 1; i < sorted_neighbor_lanes->Size(); ++i) {
    const NeighborsLaneInfo& neighbor = sorted_neighbor_lanes->GetData(i);
    const Lane& lane = lane_table_storage_[neighbor.lane_index];
    map_var_t abs_dist = common::com_abs(neighbor.proj_point.l);
    if (neighbor.proj_point.s < 0.0F) {
      abs_dist = point.DistanceTo(lane.central_curve().points()[0]);
    } else if (neighbor.proj_point.s > lane.central_curve().total_length()) {
      abs_dist = point.DistanceTo(lane.central_curve().points().Back());
    }
    if (abs_dist < min_abs_dist) {
      min_abs_dist = abs_dist;
      *nearest_neighbor_index = i;
    }
  }

  // Check these neighbor lanes if in the valid range.
  NeighborsLaneInfo& nearest_neighbor =
      sorted_neighbor_lanes->GetData(*nearest_neighbor_index);
  const Lane& nearest_lane = lane_table_storage_[nearest_neighbor.lane_index];
  nearest_neighbor.valid_lat = true;
  nearest_neighbor.valid_lon = true;
  nearest_neighbor.neighbor_flag = 0;
  map_var_t nearest_lane_left_width = 0;
  map_var_t nearest_lane_right_width = 0;
  nearest_lane.GetWidth(
        nearest_neighbor.proj_point.s,
        &nearest_lane_left_width, &nearest_lane_right_width);
  map_var_t curr_left_width = nearest_lane_left_width;
  map_var_t curr_right_width = nearest_lane_right_width;
  map_var_t next_left_width = 0;
  map_var_t next_right_width = 0;

  /* k001 2020-08-20 longjiaoy (begin)
   * 修改相邻车道flag设置不正确，导致查找当前参考线出错
   */
  for (Int32_t i = 0; i < sorted_neighbor_lanes->Size(); ++i) {
    NeighborsLaneInfo& neighbor = sorted_neighbor_lanes->GetData(i);
    neighbor.neighbor_flag = i - *nearest_neighbor_index;
  }
  /* k001 2020-08-20 longjiaoy (end) */

  // Check in left side
  for (Int32_t i = *nearest_neighbor_index+1;
       i < sorted_neighbor_lanes->Size(); ++i) {
    NeighborsLaneInfo& neighbor = sorted_neighbor_lanes->GetData(i);
    const Lane& lane = lane_table_storage_[neighbor.lane_index];
    lane.GetWidth(neighbor.proj_point.s, &next_left_width, &next_right_width);
    if (common::com_abs(neighbor.proj_point.l -
                        (*sorted_neighbor_lanes)[i-1].proj_point.l) >
        (curr_left_width + next_right_width + kMaxAllowedLaneWidthErr)) {
      // Invalid neighbor lane, which is too far from current lane.
      break;
    }
    neighbor.valid_lat = true;
    if ((neighbor.proj_point.s > -0.1F) &&
        (neighbor.proj_point.s < (lane.central_curve().total_length() + 0.1F))) {
      // Projection point is in this lane range.
      neighbor.valid_lon = true;
    } else {
      /* k001 pengc 2021-06-17 (begin) */
      // 不在道路范围内也继续查找
      // break;
      neighbor.valid_lon = false;
      /* k001 pengc 2021-06-17 (end) */
    }
    curr_left_width = next_left_width;
  }
  // Check in right side
  curr_right_width = nearest_lane_right_width;
  for (Int32_t i = *nearest_neighbor_index-1; i >= 0; --i) {
    NeighborsLaneInfo& neighbor = sorted_neighbor_lanes->GetData(i);
    const Lane& lane = lane_table_storage_[neighbor.lane_index];
    lane.GetWidth(neighbor.proj_point.s, &next_left_width, &next_right_width);
    if (common::com_abs(neighbor.proj_point.l -
                        (*sorted_neighbor_lanes)[i+1].proj_point.l) >
        (curr_right_width + next_left_width + kMaxAllowedLaneWidthErr)) {
      // Invalid neighbor lane, which is too far from current lane.
      break;
    }
    neighbor.valid_lat = true;
    if ((neighbor.proj_point.s > -0.1F) &&
        (neighbor.proj_point.s < (lane.central_curve().total_length() + 0.1F))) {
      // Projection point is in range of lane.
      neighbor.valid_lon = true;
    } else {
      /* k001 pengc 2021-06-17 (begin) */
      // 不在道路范围内也继续查找
      // break;
      neighbor.valid_lon = false;
      /* k001 pengc 2021-06-17 (end) */
    }
    curr_right_width = next_right_width;
  }

  return true;
}

/* k004 pengc 2022-12-26 (begin) */
// 将车道线与地图进行配准
/*
 * @brief 将轨迹与地图进行配准
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/26  <td>1.0      <td>pengc       <td>First edition
 * </table>
 */
Int32_t HDMap::RegisterPath(
    const common::StaticVector<common::PathPoint,
    common::Path::kMaxPathPointNum>& sample_points,
    map_var_t delta_pos[3]) {
//#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "### RegisterPath (Begin) ###";
//#endif

//#if ENABLE_HD_MAP_PERFORMANCE_TEST
  phoenix::common::Stopwatch performance_timer;
//#endif

  Int32_t src_points_size = 0;
  Int32_t dst_points_size = 0;

  Int32_t iter = 0;
  map_var_t residual_error = 0.0F;
  Int32_t register_quality = REGISTER_QUALITY_INVALID;
  FuncGetPoints get_src_points(temp_data_.points_2d_1);
  FuncGetPoints get_dst_points(temp_data_.points_2d_2);
  phoenix::common::Matrix<map_var_t, 3, 3> mat_rot;
  phoenix::common::Matrix<map_var_t, 3, 1> vec_tran;

  for (iter = 0; iter < 8; ++iter) {
    /// TODO：需要增加迭代停止条件(误差小于某个值，或者误差不再继续减小)
    // 根据两个车道线检测帧之间的转换关系，定义正向的平移/旋转矩阵
    common::Matrix<map_var_t, 3, 3> mat_convert;
    common::Matrix<map_var_t, 2, 1> rotate_center;
    common::Matrix<map_var_t, 2, 1> point_conv;
    mat_convert.SetIdentity();
    rotate_center.SetZeros();
    common::Rotate_2D<map_var_t>(rotate_center, delta_pos[2], &mat_convert);
    common::Translate_2D(delta_pos[0], delta_pos[1], &mat_convert);

    temp_data_.points_2d_1.Clear();
    temp_data_.points_2d_2.Clear();
    residual_error = 0.0F;

    for (Int32_t i = 0; i < sample_points.Size(); ++i) {
      // 将曲线点转换到之前帧的坐标系下
      point_conv(0) = sample_points[i].point.x();
      point_conv(1) = sample_points[i].point.y();
      common::TransformVert_2D(mat_convert, &point_conv);
      common::Vec2d new_point(point_conv(0), point_conv(1));

      // 在地图中找到匹配点（点到线的距离最近的点，投影点）
      Int32_t nearest_lane_index = -1;
      common::PathPoint nearest_point;
      if (!FindNearestLane(new_point, &nearest_lane_index, &nearest_point)) {
        LOG_ERR << "Failed to find nearest lane from map.";
        continue;
      }
      const common::Path& lane_curve =
          lane_table_storage_[nearest_lane_index].central_curve();
      map_var_t dist_error = common::com_abs(nearest_point.l);

      if ((dist_error < 1.0F) &&
          ((0.0F < nearest_point.s) &&
           (nearest_point.s < lane_curve.total_length()))) {
        // 只有在之前帧的车道线范围内的点，并且距离小于一定值
        //（过滤掉异常点）才可以算作匹配点

        // 计算残差
        residual_error += dist_error;

        // 保存匹配点到列表中
        common::Vec2d* src_point = temp_data_.points_2d_1.Allocate();
        if (Nullptr_t == src_point) {
          break;
        }
        common::Vec2d* dst_point = temp_data_.points_2d_2.Allocate();
        if (Nullptr_t == dst_point) {
          temp_data_.points_2d_1.PopBack();
          break;
        }
        *src_point = nearest_point.point;
        *dst_point = new_point;
      }
    }

    src_points_size = temp_data_.points_2d_1.Size();
    dst_points_size = temp_data_.points_2d_2.Size();

    if (src_points_size != dst_points_size) {
      register_quality = REGISTER_QUALITY_INVALID;
      break;
    }
    if (src_points_size < 2) {
      // 匹配点过少
      register_quality = REGISTER_QUALITY_INVALID;
      break;
    }

    // printf("sample_points.Size()=%d\n", sample_points.Size());
    if (src_points_size >= sample_points.Size()-1) {
      register_quality = REGISTER_QUALITY_GOOD;
    } else if (src_points_size < sample_points.Size()-2) {
      register_quality = REGISTER_QUALITY_BAD;
    } else if (src_points_size < 4) {
      register_quality = REGISTER_QUALITY_BAD;
    } else {
      register_quality = REGISTER_QUALITY_NOT_SO_GOOD;
    }
    if (residual_error > 0.8F) {
      register_quality = REGISTER_QUALITY_BAD;
    }
    // register_quality = REGISTER_QUALITY_GOOD;

#if ENABLE_HD_MAP_TRACE
    std::cout << "iter = " << iter << std::endl;
    std::cout << "residual_error = " << residual_error << std::endl;
    std::cout << "src_points_size = "
              << src_points_size
              << ", dst_points_size = "
              << dst_points_size
              << ", register_quality="
              << register_quality
              << std::endl;
#endif

    // 计算两组匹配点之间的转换关系
    bool ret = phoenix::common::CalcTfBtwCorrespondedPtsSetsQuaternionsBase_2D(
          src_points_size, get_src_points, get_dst_points, &mat_rot, &vec_tran);
    if (false == ret) {
      LOG_ERR << "Failed to calculate the transformation "
                 "matrix between the lane lines.";
      register_quality = REGISTER_QUALITY_INVALID;
      break;
    }

    // 更新位置变换关系
    map_var_t rot_angle = common::com_atan2(-mat_rot(0, 1), mat_rot(1, 1));
    delta_pos[0] -= vec_tran(0);
    delta_pos[1] -= vec_tran(1);
    delta_pos[2] -= rot_angle;

#if ENABLE_HD_MAP_TRACE
    std::cout << "mat_rot=\n" << mat_rot << std::endl;
    std::cout << "vec_tran= " << vec_tran(0) << ", " << vec_tran(1)
              << ", " << vec_tran(2) << std::endl;
    std::cout << "rotation_angle = " << common::com_rad2deg(rot_angle)
              << std::endl;
#endif
  }

//#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "iter = " << iter;
  LOG_INFO(5) << "residual_error = " << residual_error;
  LOG_INFO(5) << "src_points_size = "
              << src_points_size
              << ", dst_points_size = "
              << dst_points_size
              << ", register_quality="
              << register_quality;
//#endif

//#if ENABLE_HD_MAP_PERFORMANCE_TEST
  LOG_INFO(5) << "Registering path spend " << performance_timer.Elapsed()
              << "ms.";
//#endif

//#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "### RegisterPath (End) ###";
//#endif

  return (register_quality);
}

/*
 * @brief 使用相机车道中心线替换地图中的车道中心线
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/12/29  <td>1.0      <td>pengc       <td>First edition
 * </table>
 */
bool HDMap::ReplaceCenterLine(const common::Path& src_path) {
  const static map_var_t kMaxLatOffset = 0.2F;
  const static map_var_t kMaxHeadingOffset = common::com_deg2rad(10.0F);
  const static map_var_t kSearchStep = 5.0F;
  const static map_var_t kMinMatchedLen = 0.5F;

#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "### ReplaceCenterLine (Start) ###";
#endif

  if (lane_table_storage_.Empty()) {
    LOG_ERR << "The lane table is empty.";
    return (false);
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  phoenix::common::Stopwatch performance_timer;
#endif

  common::Vec2d cur_veh_pos(current_rel_position_.x, current_rel_position_.y);

  common::PathPoint nearest_point_on_src_path;
  if (!src_path.FindProjection(cur_veh_pos, &nearest_point_on_src_path)) {
    LOG_ERR << "Failed to find projection of current position on src path.";
    return false;
  }

  Int32_t nearest_lane_idx = -1;
  common::PathPoint nearest_point_on_lane;
  /* k006 pengc 2023-02-01 (begin) */
  // 修改分流口最近车道匹配错误的问题
#if 0
  if (!FindNearestLaneSynthetically(
        cur_veh_pos, current_rel_position_.heading,
        &nearest_lane_idx, &nearest_point_on_lane)) {
    LOG_ERR << "Failed to find nearest lane.";
    return (false);
  }
  if (common::com_abs(nearest_point_on_lane.l - nearest_point_on_src_path.l) >
      kMaxLatOffset) {
    LOG_INFO(5) << "Too far from nearest lane to src path.";
    return (false);
  }
#else
  if (!FindNearestLaneSynthetically(
        nearest_point_on_src_path.point, nearest_point_on_src_path.heading,
        &nearest_lane_idx, &nearest_point_on_lane)) {
    LOG_ERR << "Failed to find nearest lane.";
    return (false);
  }
  if (common::com_abs(nearest_point_on_lane.l) > kMaxLatOffset) {
    LOG_INFO(5) << "Too far from nearest lane to src path.";
    return (false);
  }
#endif
  /* k006 pengc 2023-02-01 (end) */
  if (!IsValidLaneIndex(nearest_lane_idx)) {
    LOG_ERR << "Invalid nearest lane index.";
    return (false);
  }
  Lane* nearest_lane = &(lane_table_storage_[nearest_lane_idx]);

#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "The nearest lane is (idx=" << nearest_lane_idx
              << ", id=" << nearest_lane->topology_id().id.id
              << ") when replace center line.";
#endif

  // Merge parameters
  common::Path::MergeParam merge_param;
  merge_param.max_lat_offset = kMaxLatOffset;
  merge_param.max_heading_offset = kMaxHeadingOffset;
  merge_param.search_step = kSearchStep;
#if 0
  merge_param.min_matched_len = kMinMatchedLen;
#else
  merge_param.min_matched_len = common::Min(
        nearest_lane->central_curve().total_length()-
        nearest_point_on_lane.s-0.01F, kMinMatchedLen);
#endif
  merge_param.max_len_for_heading = 10.0F;
  merge_param.min_len_for_heading = 3.0F;
  merge_param.points_2d = &temp_data_.points_2d_1;
  common::Path::MergeInfo merge_info_start_lane;
  merge_info_start_lane.mat_conv_start.SetIdentity();
  merge_info_start_lane.mat_conv_end.SetIdentity();
  // Merge nearest lane
  /// TODO: 车道连接处，由于可匹配的距离过短，会导致当前车道匹配失败，但前向车道实际是可匹配的
  if (nearest_lane->central_curve().Merge(
        merge_param, src_path, &merge_info_start_lane)) {
#if ENABLE_HD_MAP_TRACE
    LOG_INFO(5) << "Replace nearest lane index " << nearest_lane_idx
                << " (id=" << nearest_lane->topology_id().id.id
                << ") with source path ~ [OK]";
#endif
  } else {
#if ENABLE_HD_MAP_TRACE
    LOG_INFO(5) << "Replace nearest lane index " << nearest_lane_idx
                << " (id=" << nearest_lane->topology_id().id.id
                << ") with source path ~ [NG]";
#endif
    return false;
  }

  // 将与相机匹配上的车道设置为routing车道，避免在分流口不小心规划到分流车道
  map_var_t routing_lanes_total_len = 0.0F;
  common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM> routing_lanes;

  map_var_t backward_len = nearest_point_on_src_path.s;
  if (backward_len < 2.0F) {
    backward_len = 2.0F;
  } else if (backward_len > 50.0F) {
    backward_len = 50.0F;
  } else {
    // nothing to do
  }
  map_var_t forward_len =
      src_path.total_length() - nearest_point_on_src_path.s;
  if (forward_len < 10.0F) {
    forward_len = 10.0F;
  } else if (forward_len > 80.0F) {
    forward_len = 80.0F;
  } else {
    // nothing to do
  }

  for (Int32_t i = 0; i < 2; ++i) {
    bool search_forward = true;
    bool search_by_lane_id = true;
    map_var_t search_len = 0.0F;
    if (0 == i) {
      search_forward = true;
      search_len = forward_len;
    } else {
      search_forward = false;
      search_len = backward_len;
    }
    // 起始道路段
    Int32_t start_lane_idx = nearest_lane_idx;
    common::PathPoint start_point_on_lane = nearest_point_on_lane;
    // 道路信息
    Lane* lane = &(lane_table_storage_[start_lane_idx]);
    const common::StaticVector<Lane::TopologyId,
        Lane::MAX_TOPOLOGY_CONNECTED_LANE_NUM>* connected_lanes = Nullptr_t;
    // 道路段
    LaneSeg lane_seg;
    lane_seg.lane_id = lane->topology_id().id;
    lane_seg.lane_index = start_lane_idx;

    if (search_forward) {
      // 向前搜索
      lane_seg.start_s = start_point_on_lane.s;
      lane_seg.end_s = lane->central_curve().total_length();

      if (lane->successor_lanes().Empty()) {
        // 此条道路没有向前连接的道路了
        break;
      }
    } else {
      // 向后搜索
      lane_seg.start_s = 0.0F;
      lane_seg.end_s = start_point_on_lane.s;

      if (lane->predecessor_lanes().Empty()) {
        // 此条道路没有向后连接的道路了
        break;
      }
    }

    // 定义堆栈，用来在地图中搜索相连接的道路段
    TemporaryData::StackForFindingLanes& stack =
        temp_data_.stack_for_finding_lanes;

    // 使用前需要将堆栈清空
    stack.Clear();
    // 将此道路段压入堆栈
    TemporaryData::StackForFindingLanes::StackNode* stack_node = stack.Allocate();
    if (Nullptr_t == stack_node) {
      LOG_ERR << "Storage of stack is full.";
      break;
    }
    stack_node->lanes.PushBack(lane_seg);
    stack_node->accumulated_s = lane_seg.end_s - lane_seg.start_s;
    stack_node->access_index = 0;
    if (search_forward) {
      stack_node->mat_conv = merge_info_start_lane.mat_conv_end;
    } else {
      stack_node->mat_conv = merge_info_start_lane.mat_conv_start;
    }

#if 0
    printf("    Set mat {%0.4f, %0.4f, %0.4f,\n"
           "             %0.4f, %0.4f, %0.4f,\n"
           "             %0.4f, %0.4f, %0.4f}\n",
           stack_node->mat_conv(0,0), stack_node->mat_conv(0,1), stack_node->mat_conv(0,2),
           stack_node->mat_conv(1,0), stack_node->mat_conv(1,1), stack_node->mat_conv(1,2),
           stack_node->mat_conv(2,0), stack_node->mat_conv(2,1), stack_node->mat_conv(2,2));
#endif

    // 使用深度优先搜索算法，获取前后相连的车道
    bool storage_is_full = false;
    while (!(stack.Empty())) {
      // 取出栈顶元素
      stack_node = &(stack.Top());
      // 栈顶中保存的最后一段车道信息
      lane = &(lane_table_storage_[stack_node->lanes.Back().lane_index]);
      if (search_forward) {
        connected_lanes = &(lane->successor_lanes());
      } else {
        connected_lanes = &(lane->predecessor_lanes());
      }

      if (connected_lanes->Empty()) {
        // 此车道没有连接的车道了
        stack.Pop();
        continue;
      } else if (stack_node->access_index >= connected_lanes->Size()) {
        // 此车道所有连接的车道都访问过了
        stack.Pop();
        continue;
      } else {
        // nothing to do
      }

      Int32_t lane_idx = (*connected_lanes)[stack_node->access_index].index;
      if (search_by_lane_id) {
        const Int32_t* const tmp_lane_index =
            lane_table_.Find((*connected_lanes)[stack_node->access_index].id.id);
        if (Nullptr_t != tmp_lane_index) {
          lane_idx = *tmp_lane_index;
        } else {
          LOG_ERR << "Can't find lane index by lane id.";
        }
      }
      if (!IsValidLaneIndex(lane_idx)) {
        // 道路索引无效
        stack.Pop();
        LOG_ERR << "Detected invalid lane index in successor lanes.";
        continue;
      }

      while (connected_lanes->Size() > 0) {
        // 此车道还有连接的道路
        lane_idx = (*connected_lanes)[stack_node->access_index].index;
        if (search_by_lane_id) {
          const Int32_t* const tmp_lane_index =
              lane_table_.Find((*connected_lanes)[stack_node->access_index].id.id);
          if (Nullptr_t != tmp_lane_index) {
            lane_idx = *tmp_lane_index;
          } else {
            LOG_ERR << "Can't find lane index by lane id.";
          }
        }
        if (!IsValidLaneIndex(lane_idx)) {
          // 此车道连接的道路索引无效
          LOG_ERR << "Detected invalid lane index in successor lanes.";
          break;
        }
        // 更新车道段为此车道连接的车道段
        const map_var_t accumulated_s = stack_node->accumulated_s;
        lane_seg.lane_index = lane_idx;
        // 此车道连接的车道信息
        lane = &(lane_table_storage_[lane_idx]);
        if (search_forward) {
          connected_lanes = &(lane->successor_lanes());
        } else {
          connected_lanes = &(lane->predecessor_lanes());
        }
        lane_seg.lane_id = lane->topology_id().id;
        lane_seg.start_s = 0.0F;
        lane_seg.end_s = lane->central_curve().total_length();

        // 添加此连接的车道
        common::Matrix<map_var_t, 3, 3>* mat_conv = &(stack_node->mat_conv);
        common::Path::ConvertParam conv_param;
        conv_param.mat_conv = mat_conv;
        conv_param.points_2d = &temp_data_.points_2d_1;

        if (lane->central_curve().points().Size() < 2) {
          LOG_ERR << "Invalid lane center curve.";
          break;
        }
        common::PathPoint proj_point;
        if (search_forward) {
          if (!src_path.FindProjection(
                lane->central_curve().points().Front(), &proj_point)) {
            LOG_ERR << "Failed to find projection point.";
            break;
          }
        } else {
          if (!src_path.FindProjection(
                lane->central_curve().points().Back(), &proj_point)) {
            LOG_ERR << "Failed to find projection point.";
            break;
          }
        }
        common::Path::MergeInfo merge_info;

        if ((accumulated_s < search_len) &&
            (common::com_abs(proj_point.l) < kMaxLatOffset) &&
            (-0.1F < proj_point.s) &&
            (proj_point.s < (src_path.total_length()+0.1F))) {
          // Merge nearest lane
          if ((lane->central_curve().total_length() > 20.0F)) {
            merge_param.min_matched_len = 15.0F;
          } else {
            merge_param.min_matched_len = common::Max(
                  lane->central_curve().total_length()-5.0F, kMinMatchedLen);
          }
          // merge_param.min_matched_len = 0.5F;
          if (lane->central_curve().Merge(merge_param, src_path, &merge_info)) {
            if (search_forward) {
              mat_conv = &(merge_info.mat_conv_end);

              map_var_t total_matched_len =
                  merge_info.end_point_on_src.s - nearest_point_on_src_path.s;
              if (total_matched_len > routing_lanes_total_len) {
                routing_lanes_total_len = total_matched_len;
                routing_lanes = stack_node->lanes;
                routing_lanes.PushBack(lane_seg);
                // std::cout << "    @@@ total_matched_len=" << total_matched_len
                //           << ", add lane(idx=" << lane_seg.lane_index
                //           << ", id=" << lane_seg.lane_id.id
                //           << ")." << std::endl;
              }
            } else {
              mat_conv = &(merge_info.mat_conv_start);
            }
#if ENABLE_HD_MAP_TRACE
            LOG_INFO(5) << "Replace lane index " << lane_idx
                        << " (id=" << lane->topology_id().id.id
                        << ") with source path ~ [OK]";
#endif
#if 0
            printf("    Set mat {%0.4f, %0.4f, %0.4f,\n"
                   "             %0.4f, %0.4f, %0.4f,\n"
                   "             %0.4f, %0.4f, %0.4f}\n",
                   (*mat_conv)(0,0), (*mat_conv)(0,1), (*mat_conv)(0,2),
                   (*mat_conv)(1,0), (*mat_conv)(1,1), (*mat_conv)(1,2),
                   (*mat_conv)(2,0), (*mat_conv)(2,1), (*mat_conv)(2,2));
#endif
          } else {
            lane->central_curve().Convert(conv_param);
            LOG_INFO(5) << "Replace lane index " << lane_idx
                        << " (id=" << lane->topology_id().id.id
                        << ") with source path ~ [NG]";
#if 0
            printf("    Use mat {%0.4f, %0.4f, %0.4f,\n"
                   "             %0.4f, %0.4f, %0.4f,\n"
                   "             %0.4f, %0.4f, %0.4f}\n",
                   (*conv_param.mat_conv)(0,0), (*conv_param.mat_conv)(0,1), (*conv_param.mat_conv)(0,2),
                   (*conv_param.mat_conv)(1,0), (*conv_param.mat_conv)(1,1), (*conv_param.mat_conv)(1,2),
                   (*conv_param.mat_conv)(2,0), (*conv_param.mat_conv)(2,1), (*conv_param.mat_conv)(2,2));
#endif
          }
        } else {
          lane->central_curve().Convert(conv_param);
#if ENABLE_HD_MAP_TRACE
          LOG_INFO(5) << "The lane index " << lane_idx
                      << " (id=" << lane->topology_id().id.id
                      << ") is far from source path"
                      << ", accumulated_s=" << accumulated_s
                      << " proj_point.l=" << proj_point.l
                      << " proj_point.s=" << proj_point.s;
#endif
#if 0
          printf("    Use mat {%0.4f, %0.4f, %0.4f,\n"
                 "             %0.4f, %0.4f, %0.4f,\n"
                 "             %0.4f, %0.4f, %0.4f}\n",
                 (*conv_param.mat_conv)(0,0), (*conv_param.mat_conv)(0,1), (*conv_param.mat_conv)(0,2),
                 (*conv_param.mat_conv)(1,0), (*conv_param.mat_conv)(1,1), (*conv_param.mat_conv)(1,2),
                 (*conv_param.mat_conv)(2,0), (*conv_param.mat_conv)(2,1), (*conv_param.mat_conv)(2,2));
#endif
        }

        // lane_list->PushBack(lane_seg.lane_index);
        // std::cout << "  lane_idx=" << lane_seg.lane_index
        //           << ", lane_id=" << lane_seg.lane_id.id
        //           << std::endl;

        // 节点访问索引加1
        stack_node->access_index++;
        // 将此道路段压入堆栈
        TemporaryData::StackForFindingLanes::StackNode* old_stack_node = stack_node;
        stack_node = stack.Allocate();
        if (Nullptr_t == stack_node) {
          storage_is_full = true;
          LOG_ERR << "Storage of stack is full.";
          break;
        }
        stack_node->lanes = old_stack_node->lanes;
        stack_node->lanes.PushBack(lane_seg);
        stack_node->accumulated_s =
            accumulated_s + lane_seg.end_s - lane_seg.start_s;
        stack_node->access_index = 0;
        stack_node->mat_conv = *mat_conv;
      }
      if (storage_is_full) {
        break;
      }
    }
  }

  if (routing_lanes.Empty()) {
    LaneSeg lane_seg;
    lane_seg.lane_id = nearest_lane->topology_id().id;
    lane_seg.lane_index = nearest_lane_idx;
    routing_lanes.PushBack(lane_seg);
  }

#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "The routing lanes from camera are:";
  for (Int32_t i = 0; i < routing_lanes.Size(); ++i) {
    LOG_INFO(5) << "  [" << i
                << "]: idx=" << routing_lanes[i].lane_index
                << ", id=" << routing_lanes[i].lane_id.id;
  }
#endif

  /* k006 pengc 2023-01-08 (begin) */
  // 解决分流口误匹配的问题
  if (routing_lanes.Size() > 1) {
    for (Int32_t i = 0; i < routing_lanes.Size(); ++i) {
      RoutingSection* section = routing_.routing_sections().Allocate();
      if (Nullptr_t == section) {
        LOG_ERR << "Can't store routing segment any more, "
                   "storage is full.";
        break;
      }
      RoutingSegment* seg = section->neighbors.Allocate();
      if (Nullptr_t == seg) {
        routing_.routing_sections().PopBack();
        LOG_ERR << "Can't store routing segment any more, "
                   "storage is full.";
        break;
      }
      const Lane& lane = lane_table_storage_[routing_lanes[i].lane_index];

      seg->lane_id = routing_lanes[i].lane_id;
      seg->lane_index = routing_lanes[i].lane_index;
      seg->start_s = -0.15F;
      seg->end_s = lane.central_curve().total_length()+0.15F;

#if ENABLE_HD_MAP_TRACE
      LOG_INFO(5) << "  Add routing seg(lane_idx=" << seg->lane_index
                  << ", lane_id=" << seg->lane_id.id << ")";
#endif
    }
  }
  /* k006 pengc 2023-01-08 (end) */

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  LOG_INFO(5) << "Replacing center line spend "
              << performance_timer.Elapsed()
              << "ms.";
#endif

#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "### ReplaceCenterLine (End) ###";
#endif

  return true;
}
/* k004 pengc 2022-12-26 (end) */


Int32_t HDMap::ConstructLaneInfoFromCamera(
    Int32_t center_line_idx,
    const ad_msg::LaneInfoCameraList& camera_lane_list,
    Int32_t ref_lane_idx,
    map_var_t lat_offset) {
  const ad_msg::LaneCenterLineCamera& center_line =
      camera_lane_list.center_lines[center_line_idx];

  if (center_line.curve_point_num < 2) {
    return (-1);
  }
#if (!ENABLE_CAMERA_MULTIPLE_LANE)
  if (0 != center_line.id) {
    return (-1);
  }
#endif

  // Get a memory space from lane table storage.
  Int32_t lane_storage_index = -1;
  Lane* lane_info = lane_table_storage_.Allocate(&lane_storage_index);
  if (Nullptr_t == lane_info) {
    LOG_ERR << "Can't save lane information anymore, storage is full.";
    return (-1);
  }

  /* k003 longjiaoy 2022-11-28 (start) */
  // 获取车道边界线的类型
  Int32_t left_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
  Int32_t right_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
  if (center_line.left_boundary_index >= 0) {
    const ad_msg::LaneBoundaryLineCamera& boundary_line =
        camera_lane_list.boundary_lines[center_line.left_boundary_index];
    left_boundary_type = GetLaneBoundaryTypeFromCamera(boundary_line.type);
  }
  if (center_line.right_boundary_index >= 0) {
    const ad_msg::LaneBoundaryLineCamera& boundary_line =
        camera_lane_list.boundary_lines[center_line.right_boundary_index];
    right_boundary_type = GetLaneBoundaryTypeFromCamera(boundary_line.type);
  }
  /* k003 longjiaoy 2022-11-28 (end) */

  // center line
  common::Vec2d prev_point;
  map_var_t accumulated_s = 0.0F;
  temp_data_.points_2d_1.Clear();
  if (0 == center_line.id || !IsValidLaneIndex(ref_lane_idx)) {
    // 当前车道中心线
    for (Int32_t j = 0; j < center_line.curve_point_num; ++j) {
      common::Vec2d* p = temp_data_.points_2d_1.Allocate();
      if (Nullptr_t == p) {
        LOG_ERR << "Can't add points, storage is full.";
        break;
      }
      p->set_x(center_line.curve[j].x);
      p->set_y(center_line.curve[j].y);

      // Lane Boundary
      if (j > 0) {
        accumulated_s += p->DistanceTo(prev_point);
      }

      Lane::BoundaryWidthAssociation* lane_boundary_width =
          lane_info->left_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t != lane_boundary_width) {
        lane_boundary_width->s = accumulated_s;
        lane_boundary_width->width = center_line.curve[j].left_width;
      }
      lane_boundary_width =
          lane_info->right_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t != lane_boundary_width) {
        lane_boundary_width->s = accumulated_s;
        lane_boundary_width->width = center_line.curve[j].right_width;
      }

      prev_point = *p;
    }
    Lane::BoundaryTypeAssociation* lane_boundary_type =
        lane_info->left_boundary().boundary_type_samples.Allocate();
    if (Nullptr_t != lane_boundary_type) {
      lane_boundary_type->s = 0.0F;
      lane_boundary_type->type = left_boundary_type;
    }
    lane_boundary_type =
        lane_info->right_boundary().boundary_type_samples.Allocate();
    if (Nullptr_t != lane_boundary_type) {
      lane_boundary_type->s = 0.0F;
      lane_boundary_type->type = right_boundary_type;
    }
  } else {
    const Lane& ref_lane = GetLane(ref_lane_idx);
    ref_lane.central_curve().CalcSamplePointsByLatOffsetting(
          lat_offset, &temp_data_.points_2d_1);
    map_var_t half_width = 0.5F * common::com_abs(lat_offset);
    for (Int32_t j = 0; j < temp_data_.points_2d_1.Size(); ++j) {
      const common::Vec2d& p = temp_data_.points_2d_1[j];
      // Lane Boundary
      if (j > 0) {
        accumulated_s += p.DistanceTo(prev_point);
      }

      Lane::BoundaryWidthAssociation* lane_boundary_width =
          lane_info->left_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t != lane_boundary_width) {
        lane_boundary_width->s = accumulated_s;
        lane_boundary_width->width = half_width;
      }
      lane_boundary_width =
          lane_info->right_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t != lane_boundary_width) {
        lane_boundary_width->s = accumulated_s;
        lane_boundary_width->width = half_width;
      }

      prev_point = p;
    }
    Lane::BoundaryTypeAssociation* lane_boundary_type =
        lane_info->left_boundary().boundary_type_samples.Allocate();
    if (Nullptr_t != lane_boundary_type) {
      lane_boundary_type->s = 0.0F;
      lane_boundary_type->type = left_boundary_type;
    }
    lane_boundary_type =
        lane_info->right_boundary().boundary_type_samples.Allocate();
    if (Nullptr_t != lane_boundary_type) {
      lane_boundary_type->s = 0.0F;
      lane_boundary_type->type = right_boundary_type;
    }
  }

  // Save central line to this lane.
  lane_info->central_curve().Clear();
  lane_info->central_curve().Construct(temp_data_.points_2d_1);
  if (lane_info->central_curve().points().Size() < 2) {
    LOG_ERR << "There are not enough points in path of this lane.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return (-1);
  }
  /* k001 longjiaoy 2022-07-18 (begin) */
  // 左右车道线也延长，避免变道过程中，在两车道中间位置查找参考线时，
  // 车辆在右侧或左侧车道线的投影点位于起点之前。
  // if (0 == center_line.id) {
  if ((0 == center_line.id) ||
      (1 == center_line.id) ||
      (-1 == center_line.id)) {
  /* k001 longjiaoy 2022-07-18 (end) */
    // extend当前车道中心线
    common::PathPoint curr_proj_point;
    lane_info->central_curve().FindProjection(
          common::Vec2d(current_rel_position_.x, current_rel_position_.y),
          &curr_proj_point);
    map_var_t ex_len_backward = 0.0F;
    if (curr_proj_point.s < 2.0F) {
      ex_len_backward = 2.0F - curr_proj_point.s;
    }
    map_var_t ex_len_forward = 0.0F;
    map_var_t forward_len = lane_info->central_curve().total_length() -
        curr_proj_point.s;
    map_var_t min_forward_len =
        common::Max(20.0F, common::com_abs(current_rel_position_.v)*4.0F);
    if (forward_len < min_forward_len) {
      ex_len_forward = min_forward_len - forward_len;
    }
    ExtendPath(ex_len_backward, ex_len_forward,
               &(lane_info->central_curve()));
  }

  // Lane ID
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  lane_info->topology_id().id.id = std::to_string(center_line.id);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  lane_info->topology_id().id.id = center_line.id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  lane_info->topology_id().id.id = center_line.id;
#else
  lane_info->topology_id().id.id = center_line.id;
#endif
  lane_info->topology_id().index = lane_storage_index;

  // Add to lane table
  const Int32_t* lane_index =
      lane_table_.Find(lane_info->topology_id().id.id);
  if (Nullptr_t != lane_index) {
    LOG_ERR << "Detected repeated lane id in lane table.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return (-1);
  }

  switch (center_line.quality) {
  case (0):
    lane_info->set_quality(LANE_QUALITY_INVALID);
    break;
  case (1):
    lane_info->set_quality(LANE_QUALITY_BAD);
    break;
  case (2):
    lane_info->set_quality(LANE_QUALITY_NOT_GOOD);
    break;
  case (3):
    lane_info->set_quality(LANE_QUALITY_GOOD);
    break;
  default:
    lane_info->set_quality(LANE_QUALITY_INVALID);
    break;
  }

  LaneInBoundary* lane_in_boundary = lanes_in_boundary_.Allocate();
  if (Nullptr_t == lane_in_boundary) {
    LOG_ERR << "Can't save lane segment anymore, storage is full.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return (-1);
  }

  lane_in_boundary->lane_id = lane_info->topology_id().id;
  lane_in_boundary->lane_index = lane_storage_index;
  lane_in_boundary->start_s = 0;
  lane_in_boundary->end_s = accumulated_s;

#if ENABLE_CAMERA_MULTIPLE_LANE
  Int32_t left_center_line_index =
      camera_lane_list.FindCenterLineById(center_line.id + 1);
  if (left_center_line_index >= 0) {
    const ad_msg::LaneCenterLineCamera& left_center_line =
        camera_lane_list.center_lines[left_center_line_index];
    phoenix::driv_map::Lane::TopologyId* topology_id =
        lane_info->left_neighbor_forward_lanes().Allocate();
    if (Nullptr_t != topology_id) {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
      topology_id->id.id = std::to_string(left_center_line.id);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
      topology_id->id.id = left_center_line.id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
      topology_id->id.id = left_center_line.id;
#else
      topology_id->id.id = left_center_line.id;
#endif
    }
  }

  Int32_t right_center_line_index =
      camera_lane_list.FindCenterLineById(center_line.id - 1);
  if (right_center_line_index >= 0) {
    const ad_msg::LaneCenterLineCamera& right_center_line =
        camera_lane_list.center_lines[right_center_line_index];
    phoenix::driv_map::Lane::TopologyId* topology_id =
        lane_info->right_neighbor_forward_lanes().Allocate();
    if (Nullptr_t != topology_id) {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
      topology_id->id.id = std::to_string(right_center_line.id);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
      topology_id->id.id = right_center_line.id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
      topology_id->id.id = right_center_line.id;
#else
      topology_id->id.id = right_center_line.id;
#endif
    }
  }
#endif  // ENABLE_CAMERA_MULTIPLE_LANE

  lane_table_.Insert(lane_info->topology_id().id.id, lane_storage_index);

  return (lane_storage_index);
}

/* k003 longjiaoy 2022-11-28 (start) */
// 从相机中获取车道线类型
Int32_t HDMap::GetLaneBoundaryTypeFromCamera(
    Int32_t camera_lane_boundary_type) {
  Int32_t type = LANE_BOUNDARY_TYPE_UNKNOWN;

  switch (camera_lane_boundary_type) {
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_INVALID):
    type = LANE_BOUNDARY_TYPE_UNKNOWN;
    break;
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_UNKNOWN):
    type = LANE_BOUNDARY_TYPE_UNKNOWN;
    break;
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_DASHED):
    type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_SOLID):
    type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_DOUBLE_LANE_MARK):
    type = LANE_BOUNDARY_TYPE_DOUBLE_YELLOW;
    break;
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_BOTTS_DOTS):
    type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_ROAD_EDGE):
    type = LANE_BOUNDARY_TYPE_CURB;
    break;
  default:
    type = LANE_BOUNDARY_TYPE_UNKNOWN;
    break;
  }

  return (type);
}
/* k003 longjiaoy 2022-11-28 (end) */

void HDMap::ExtendPath(
    map_var_t ex_len_backward, map_var_t ex_len_forward, common::Path* path) {
  Int32_t path_points_num = path->points().Size();

  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>& points =
      temp_data_.points_2d_2;
  points.Clear();

  static const map_var_t kMinSegDist = 2.0F;

  if (ex_len_backward > 0.5F) {
    Int32_t next_index = 1;
    for (; next_index < (path_points_num-1); ++next_index) {
      if (path->accumulated_s()[next_index] > kMinSegDist) {
        break;
      }
    }

    map_var_t t = (-ex_len_backward) /
        (path->accumulated_s()[next_index] - path->accumulated_s()[0]);
    common::Vec2d p = (1.0F - t) * path->points()[0] +
        t * path->points()[next_index];

    points.PushBack(p);
  }

  for (Int32_t i = 0; i < path_points_num; ++i) {
    points.PushBack(path->points()[i]);
  }

  if (ex_len_forward > 0.5F) {
    Int32_t next_index = path_points_num-2;
    for (; next_index > 0; --next_index) {
      if ((path->accumulated_s()[path_points_num-1] -
           path->accumulated_s()[next_index]) > kMinSegDist) {
        break;
      }
    }

    map_var_t t =
        (ex_len_forward + path->accumulated_s()[path_points_num-1] -
         path->accumulated_s()[next_index]) /
        (path->accumulated_s()[path_points_num-1] -
        path->accumulated_s()[next_index]);
    common::Vec2d p = (1.0F - t) * path->points()[next_index] +
        t * path->points()[path_points_num-1];

    points.PushBack(p);
  }

  path->Clear();
  path->Construct(points);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::TakePointsInBoundary(
    bool is_in_boundary, bool prev_point_valid,
    const GlobalPoint& prev_point, const GlobalPoint& curr_point) {
  // Get current and next lane segment index
  const Int32_t curr_seg_index = temp_data_.take_path.curr_lane_seg_index;
  Int32_t next_seg_index = curr_seg_index + 1;
  if (next_seg_index > 1) {
    next_seg_index = 0;
  }
  // Get current and next lane segment
  TemporaryData::TakePath::LaneSeg& curr_seg =
      temp_data_.take_path.lane_seg[curr_seg_index];
  TemporaryData::TakePath::LaneSeg& next_seg =
      temp_data_.take_path.lane_seg[next_seg_index];

  if ((!is_in_boundary) && curr_seg.hold_points.Empty()) {
    // Current point is not in the range of boundary box.
    return;
  }

  // Calculate the nearest point to the path of current lane segment.
  const GlobalPoint curr_veh_position = current_global_position_;
  Float64_t curr_squared_dist_to_veh_position = 0.0;
  GlobalPoint curr_nearest_point_to_veh_position;
  if (prev_point_valid) {
    Float64_t tmp_t = 0.0;
    GetClosestPtPointToSeg(prev_point, curr_point, curr_veh_position,
                           &curr_nearest_point_to_veh_position, &tmp_t);
    curr_squared_dist_to_veh_position =
        common::Square(curr_veh_position.x -
                       curr_nearest_point_to_veh_position.x) +
        common::Square(curr_veh_position.y -
                       curr_nearest_point_to_veh_position.y);
    //curr_nearest_point_to_veh_position.heading = common::com_atan2(
    //      curr_point.y - prev_point.y, curr_point.x - prev_point.x);
    const Float64_t heading = common::com_atan2(curr_point.y - prev_point.y,
      curr_point.x - prev_point.x);
    curr_nearest_point_to_veh_position.heading =
      static_cast<Float32_t>(heading);
    curr_nearest_point_to_veh_position.s =
        prev_point.s + tmp_t * (curr_point.s - prev_point.s);
  } else {
    curr_squared_dist_to_veh_position =
        common::Square(curr_veh_position.x - curr_point.x) +
        common::Square(curr_veh_position.y - curr_point.y);
    curr_nearest_point_to_veh_position.x = curr_point.x;
    curr_nearest_point_to_veh_position.y = curr_point.y;
    curr_nearest_point_to_veh_position.heading = 0.0F;  // dummy
    curr_nearest_point_to_veh_position.s = curr_point.s;
  }
  if (curr_seg.hold_points.Empty()) {
    curr_seg.sq_dist_to_veh = curr_squared_dist_to_veh_position;
    curr_seg.nearest_point_to_veh =
        curr_nearest_point_to_veh_position;

    curr_seg.nearest_info_valid = prev_point_valid;
  } else {
    if (curr_seg.nearest_info_valid && prev_point_valid) {
      if (curr_squared_dist_to_veh_position < curr_seg.sq_dist_to_veh) {
        curr_seg.sq_dist_to_veh = curr_squared_dist_to_veh_position;
        curr_seg.nearest_point_to_veh =
            curr_nearest_point_to_veh_position;
      }
    } else {
      curr_seg.sq_dist_to_veh = curr_squared_dist_to_veh_position;
      curr_seg.nearest_point_to_veh =
          curr_nearest_point_to_veh_position;

      curr_seg.nearest_info_valid = prev_point_valid;
    }
  }

  // Push current point into buffer
  HoldPoint* point = Nullptr_t;
  if (curr_seg.hold_points.Empty() && prev_point_valid) {
    // 对于范围内的第一个点，向后多保存一个点
    point = curr_seg.hold_points.AllocateOverride();
    if (Nullptr_t == point) {
      LOG_ERR << "Unexpected, can't save points anymore.";
      return;
    } else {
      point->point = prev_point;

      point->nearest_info_valid = prev_point_valid;
      point->sq_dist_to_veh = curr_squared_dist_to_veh_position;
      point->nearest_point_to_veh = curr_nearest_point_to_veh_position;
    }
  }
  // 将范围内的点保存下来
  point = curr_seg.hold_points.AllocateOverride();
  if (Nullptr_t == point) {
    LOG_ERR << "Unexpected, can't save points anymore.";
    return;
  } else {
    point->point = curr_point;

    point->nearest_info_valid = prev_point_valid;
    point->sq_dist_to_veh = curr_squared_dist_to_veh_position;
    point->nearest_point_to_veh = curr_nearest_point_to_veh_position;
  }

#if 0
  Float64_t take_start_s = curr_seg.take_start_s;
  if (take_start_s < 0) {
    take_start_s = curr_seg.nearest_point_to_veh.s;
  }
#else
  const Float64_t take_start_s = curr_seg.nearest_point_to_veh.s;
#endif

  if ((curr_seg.hold_points.Back()->point.s - take_start_s) > map_range_) {
#if ENABLE_HD_MAP_TRACE
    std::cout << "*** Current buffer[" << curr_seg_index
              << "] have enough datas, switch to next." << std::endl;
    std::cout << "In current buffer[" << curr_seg_index
              << "], start_s=" << curr_seg.hold_points.Front()->point.s
              << ", end_s=" << curr_seg.hold_points.Back()->point.s
              << ", nearest_s=" << curr_seg.nearest_point_to_veh.s
              << ", dist_to_veh="
              << common::com_sqrt(curr_seg.sq_dist_to_veh)
              << ", total_len=" << curr_seg.hold_points.Back()->point.s -
                 curr_seg.hold_points.Front()->point.s
              << ", take_start_s=" << take_start_s
              << std::endl;
#endif

    // Switch to next segment
    if (next_seg.hold_points.Size() < 2) {
      next_seg.Clear();
      if (is_in_boundary) {
        CopyPartLaneSeg(curr_seg_index, next_seg_index);
      }

      temp_data_.take_path.curr_lane_seg_index = next_seg_index;

#if ENABLE_HD_MAP_TRACE
      std::cout << "Next buffer[" << next_seg_index
                << "] is empty, switch to it." << std::endl;
#endif
    } else {
      Int32_t better_seg_index = 0;
      SelectBetterLaneSeg(&better_seg_index);

#if ENABLE_HD_MAP_TRACE
      std::cout << "Next buffer[" << next_seg_index
                << "] is not empty, compare with it, and the better is "
                << better_seg_index << std::endl;
      std::cout << "In next buffer[" << next_seg_index
                << "], start_s=" << next_seg.hold_points.Front()->point.s
                << ", end_s=" << next_seg.hold_points.Back()->point.s
                << ", nearest_s=" << next_seg.nearest_point_to_veh.s
                << ", dist_to_veh="
                << common::com_sqrt(next_seg.sq_dist_to_veh)
                << ", total_len=" << next_seg.hold_points.Back()->point.s -
                   next_seg.hold_points.Front()->point.s
                << std::endl;
#endif

      if (curr_seg_index == better_seg_index) {
        next_seg.Clear();
        if (is_in_boundary) {
          CopyPartLaneSeg(curr_seg_index, next_seg_index);
        }
        temp_data_.take_path.curr_lane_seg_index = next_seg_index;
      } else {
        if (is_in_boundary) {
          CopyPartLaneSeg(curr_seg_index, curr_seg_index);
        } else {
          curr_seg.Clear();
        }
        temp_data_.take_path.curr_lane_seg_index = curr_seg_index;
      }
    }
  }
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::CopyPartLaneSeg(Int32_t index_from, Int32_t index_to) {
  TemporaryData::TakePath::LaneSeg& seg_from =
      temp_data_.take_path.lane_seg[index_from];
  TemporaryData::TakePath::LaneSeg& seg_to =
      temp_data_.take_path.lane_seg[index_to];

  if (seg_from.hold_points.Size() < 2) {
    LOG_ERR << "The source buffer[" << index_from << "] is empty.";
    return;
  }

#if ENABLE_HD_MAP_TRACE
  std::cout << "Copy part points from " << index_from
            << " to " << index_to << std::endl;
#endif

  common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>::const_iterator
      it = seg_from.hold_points.cbegin();
  common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>::const_iterator
      it_end = seg_from.hold_points.cend();

  if (index_from == index_to) {
    seg_from.hold_points.PopFromFront(seg_from.hold_points.Size() / 2);
  } else {
    seg_to.Clear();

    it = seg_from.hold_points.cbegin() + seg_from.hold_points.Size() / 2;
    it_end = seg_from.hold_points.cend();
    for (; it != it_end; ++it) {
      seg_to.hold_points.PushBackOverride(*it);
    }
  }

  // Update nearest point
#if 0
  // GlobalPoint curr_veh_position(position_.global_x, position_.global_y);
  Float64_t min_squared_dist_to_veh;
  GlobalPoint nearest_point_to_veh;

  if (!seg_to.hold_points.Empty()) {
    it = seg_to.hold_points.cbegin();
    it_end = seg_to.hold_points.cend();

    bool nearest_info_valid = it->nearest_info_valid;
    min_squared_dist_to_veh = it->sq_dist_to_veh;
    nearest_point_to_veh = it->nearest_point_to_veh;
    for (++it; it != it_end; ++it) {
      if (it->nearest_info_valid) {
        if (!nearest_info_valid) {
          min_squared_dist_to_veh = it->sq_dist_to_veh;
          nearest_point_to_veh = it->nearest_point_to_veh;
          nearest_info_valid = it->nearest_info_valid;
        } else {
          if (it->sq_dist_to_veh < min_squared_dist_to_veh) {
            min_squared_dist_to_veh = it->sq_dist_to_veh;
            nearest_point_to_veh = it->nearest_point_to_veh;
          }
        }
      }
    }

    seg_to.nearest_info_valid = nearest_info_valid;
    seg_to.take_start_s = seg_to.hold_points.Back()->point.s;
    seg_to.nearest_point_to_veh = nearest_point_to_veh;
    seg_to.sq_dist_to_veh = min_squared_dist_to_veh;
  }
#else
  if (!seg_to.hold_points.Empty()) {
    const HoldPoint* const back_point = seg_to.hold_points.Back();
    seg_to.nearest_info_valid = back_point->nearest_info_valid;
    seg_to.take_start_s = back_point->point.s;
    seg_to.nearest_point_to_veh = back_point->nearest_point_to_veh;
    seg_to.sq_dist_to_veh = back_point->sq_dist_to_veh;
  }
#endif

#if ENABLE_HD_MAP_TRACE
  if (seg_to.hold_points.Size() > 0) {
    std::cout << "After copy, start_s=" << seg_to.hold_points.Front()->point.s
              << ", end_s=" << seg_to.hold_points.Back()->point.s
              << ", nearest_s=" << seg_to.nearest_point_to_veh.s
              << ", dist_to_veh="
              << common::com_sqrt(seg_to.sq_dist_to_veh)
              << ", take_start_s=" << seg_to.take_start_s
              << std::endl;
  }
#endif
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::SelectBetterLaneSeg(Int32_t* const better) const {
  static const Float64_t kMaxAngleDiff = common::com_deg2rad(100.0);

  bool ret = true;
  *better = 0;

  const TemporaryData::TakePath::LaneSeg& lane_seg_0 =
      temp_data_.take_path.lane_seg[0];
  const TemporaryData::TakePath::LaneSeg& lane_seg_1 =
      temp_data_.take_path.lane_seg[1];

#if ENABLE_HD_MAP_TRACE
  std::cout << "(((((( SelectBetterLaneSeg " << std::endl;
  std::cout << "lane_seg_0.hold_points.Size()="
            << lane_seg_0.hold_points.Size() << std::endl;
  std::cout << "lane_seg_1.hold_points.Size()="
            << lane_seg_1.hold_points.Size() << std::endl;
#endif

  if ((lane_seg_0.hold_points.Size() >= 2) &&
      (lane_seg_1.hold_points.Size() < 2)) {
    *better = 0;
#if ENABLE_HD_MAP_TRACE
    std::cout << "Segment 0 isn't empty, and segment 1 is empty, so better = 0"
              << std::endl;
#endif
  } else if ((lane_seg_0.hold_points.Size() < 2) &&
             (lane_seg_1.hold_points.Size() >= 2)) {
    *better = 1;
#if ENABLE_HD_MAP_TRACE
    std::cout << "Segment 0 is empty, and segment 1 isn't empty, so better = 1"
              << std::endl;
#endif
  } else if ((lane_seg_0.hold_points.Size() < 2) &&
             (lane_seg_1.hold_points.Size() < 2)) {
    *better = 1;
    ret = false;
#if ENABLE_HD_MAP_TRACE
    LOG_WARN << "All of lane segment is empty.";
#endif
  } else {
    const Float64_t min_required_dist =
      0.6 * static_cast<Float64_t>(map_range_);

    Float32_t curr_global_heading = current_global_position_.heading;
    if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
      // For backing mode of vehicle
      curr_global_heading =
          common::NormalizeAngle(current_global_position_.heading + COM_PI);
    }
    const Float32_t abs_heading_diff_0 = common::com_abs(
          common::AngleDiff(curr_global_heading,
                            lane_seg_0.nearest_point_to_veh.heading));
    const Float32_t abs_heading_diff_1 = common::com_abs(
          common::AngleDiff(curr_global_heading,
                            lane_seg_1.nearest_point_to_veh.heading));

    if ((abs_heading_diff_0 < kMaxAngleDiff) &&
        (abs_heading_diff_1 < kMaxAngleDiff)) {
      if (common::com_abs(lane_seg_0.nearest_point_to_veh.s -
                          lane_seg_1.nearest_point_to_veh.s) < 0.01) {
        const Float64_t start_nearest_s_0 = lane_seg_0.nearest_point_to_veh.s -
            lane_seg_0.hold_points.Front()->point.s;
        const Float64_t nearest_end_s_0 = lane_seg_0.hold_points.Back()->point.s -
            lane_seg_0.nearest_point_to_veh.s;
        const Float64_t start_nearest_s_1 = lane_seg_1.nearest_point_to_veh.s -
            lane_seg_1.hold_points.Front()->point.s;
        const Float64_t nearest_end_s_1 = lane_seg_1.hold_points.Back()->point.s -
            lane_seg_1.nearest_point_to_veh.s;
        if ((nearest_end_s_0 >= min_required_dist) &&
            (nearest_end_s_1 < min_required_dist)) {
          *better = 0;
        } else if ((nearest_end_s_0 < min_required_dist) &&
                   (nearest_end_s_1 >= min_required_dist)) {
          *better = 1;
        } else if ((nearest_end_s_0 < min_required_dist) &&
                   (nearest_end_s_1 < min_required_dist)) {
          *better = (nearest_end_s_0 > nearest_end_s_1) ? static_cast<Int32_t>(0) : 1;
        } else {
          if ((start_nearest_s_0 >= min_required_dist) &&
              (start_nearest_s_1 < min_required_dist)) {
            *better = 0;
          } else if ((start_nearest_s_0 < min_required_dist) &&
                     (start_nearest_s_1 >= min_required_dist)) {
            *better = 1;
          } else if ((start_nearest_s_0 < min_required_dist) &&
                     (start_nearest_s_1 < min_required_dist)) {
            *better = (start_nearest_s_0 > start_nearest_s_1) ? static_cast<Int32_t>(0) : 1;
          } else {
            *better = (nearest_end_s_0 > nearest_end_s_1) ? static_cast<Int32_t>(0) : 1;
          }
        }
#if ENABLE_HD_MAP_TRACE
        std::cout << "!!!! Same nearest point, start_nearest_s_0="
                  << start_nearest_s_0
                  << ", nearest_end_s_0=" << nearest_end_s_0
                  << ", start_nearest_s_1=" << start_nearest_s_1
                  << ", nearest_end_s_1=" << nearest_end_s_1
                  << std::endl;
#endif
      } else {
        *better =
            (lane_seg_0.sq_dist_to_veh < lane_seg_1.sq_dist_to_veh) ? static_cast<Int32_t>(0) : 1;
      }
    } else if ((abs_heading_diff_0 < kMaxAngleDiff) &&
               (abs_heading_diff_1 >= kMaxAngleDiff)) {
      *better = 0;
    } else if ((abs_heading_diff_0 >= kMaxAngleDiff) &&
               (abs_heading_diff_1 < kMaxAngleDiff)) {
      *better = 1;
    } else {
      if (common::com_abs(lane_seg_0.nearest_point_to_veh.s -
                          lane_seg_1.nearest_point_to_veh.s) < 0.01) {
        const Float64_t start_nearest_s_0 = lane_seg_0.nearest_point_to_veh.s -
            lane_seg_0.hold_points.Front()->point.s;
        const Float64_t nearest_end_s_0 = lane_seg_0.hold_points.Back()->point.s -
            lane_seg_0.nearest_point_to_veh.s;
        const Float64_t start_nearest_s_1 = lane_seg_1.nearest_point_to_veh.s -
            lane_seg_1.hold_points.Front()->point.s;
        const Float64_t nearest_end_s_1 = lane_seg_1.hold_points.Back()->point.s -
            lane_seg_1.nearest_point_to_veh.s;
        if ((nearest_end_s_0 >= min_required_dist) &&
            (nearest_end_s_1 < min_required_dist)) {
          *better = 0;
        } else if ((nearest_end_s_0 < min_required_dist) &&
                   (nearest_end_s_1 >= min_required_dist)) {
          *better = 1;
        } else if ((nearest_end_s_0 < min_required_dist) &&
                   (nearest_end_s_1 < min_required_dist)) {
          *better = (nearest_end_s_0 > nearest_end_s_1) ? static_cast<Int32_t>(0) : 1;
        } else {
          if ((start_nearest_s_0 >= min_required_dist) &&
              (start_nearest_s_1 < min_required_dist)) {
            *better = 0;
          } else if ((start_nearest_s_0 < min_required_dist) &&
                     (start_nearest_s_1 >= min_required_dist)) {
            *better = 1;
          } else if ((start_nearest_s_0 < min_required_dist) &&
                     (start_nearest_s_1 < min_required_dist)) {
            *better = (start_nearest_s_0 > start_nearest_s_1) ? static_cast<Int32_t>(0) : 1;
          } else {
            *better = (nearest_end_s_0 > nearest_end_s_1) ? static_cast<Int32_t>(0) : 1;
          }
        }
#if ENABLE_HD_MAP_TRACE
        std::cout << "!!!! Same nearest point, start_nearest_s_0="
                  << start_nearest_s_0
                  << ", nearest_end_s_0=" << nearest_end_s_0
                  << ", start_nearest_s_1=" << start_nearest_s_1
                  << ", nearest_end_s_1=" << nearest_end_s_1
                  << std::endl;
#endif
      } else {
        *better =
            (lane_seg_0.sq_dist_to_veh < lane_seg_1.sq_dist_to_veh) ? static_cast<Int32_t>(0) : 1;
      }
    }
#if ENABLE_HD_MAP_TRACE
    std::cout << "abs_heading_diff_0 = "
              << common::com_rad2deg(abs_heading_diff_0)
              << ", dist_to_veh_0 = "
              << common::com_sqrt(lane_seg_0.sq_dist_to_veh)
              << ", abs_heading_diff_1 = "
              << common::com_rad2deg(abs_heading_diff_1)
              << ", dist_to_veh_1 = "
              << common::com_sqrt(lane_seg_1.sq_dist_to_veh)
              << ", so better = " << *better
              << std::endl;
#endif
  }

#if ENABLE_HD_MAP_TRACE
  std::cout << " ))))))" << std::endl;
#endif

  return (ret);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::LimitLaneRang(Int32_t seg_index) {
  TemporaryData::TakePath::LaneSeg& lane_seg =
      temp_data_.take_path.lane_seg[seg_index];

  if (lane_seg.hold_points.Size() < 2) {
    return;
  }
#if ENABLE_HD_MAP_TRACE
  std::cout << "(((((( LimitLaneRang" << std::endl;
  std::cout << "Before limiting, start_s="
            << lane_seg.hold_points.Front()->point.s
            << ", end_s=" << lane_seg.hold_points.Back()->point.s
            << ", total_len=" << lane_seg.hold_points.Back()->point.s -
               lane_seg.hold_points.Front()->point.s
            << std::endl;
#endif

  Float64_t min_squared_dist_to_veh;
  GlobalPoint nearest_point_to_veh;

  common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>::const_iterator
      it = lane_seg.hold_points.cbegin();
  const common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>::const_iterator
      it_end = lane_seg.hold_points.cend();

  bool nearest_info_valid = it->nearest_info_valid;
  min_squared_dist_to_veh = it->sq_dist_to_veh;
  nearest_point_to_veh = it->nearest_point_to_veh;
  for (++it; it != it_end; ++it) {
    if (it->nearest_info_valid) {
      if (!nearest_info_valid) {
        min_squared_dist_to_veh = it->sq_dist_to_veh;
        nearest_point_to_veh = it->nearest_point_to_veh;
        nearest_info_valid = it->nearest_info_valid;
      } else {
        if (it->sq_dist_to_veh < min_squared_dist_to_veh) {
          min_squared_dist_to_veh = it->sq_dist_to_veh;
          nearest_point_to_veh = it->nearest_point_to_veh;
        }
      }
    }
  }

  for (Int32_t i = 0; i < lane_seg.hold_points.Size() / 3; ++i) {
    if (lane_seg.hold_points.Size() > 3) {
      if ((nearest_point_to_veh.s - lane_seg.hold_points[1].point.s) >
          lane_range_backward_) {
        lane_seg.hold_points.PopFront();
      } else {
        break;
      }
    } else {
      break;
    }
  }

  for (Int32_t i = 0; i < lane_seg.hold_points.Size() / 3; ++i) {
    if (lane_seg.hold_points.Size() > 3) {
      if ((lane_seg.hold_points[lane_seg.hold_points.Size()-2].point.s -
           nearest_point_to_veh.s) > lane_range_forward_) {
        lane_seg.hold_points.PopBack();
      } else {
        break;
      }
    } else {
      break;
    }
  }

#if ENABLE_HD_MAP_TRACE
  std::cout << "After limiting, start_s="
            << lane_seg.hold_points.Front()->point.s
            << ", end_s=" << lane_seg.hold_points.Back()->point.s
            << ", total_len=" << lane_seg.hold_points.Back()->point.s -
               lane_seg.hold_points.Front()->point.s
            << std::endl;
  std::cout << " LimitLaneRang ))))))" << std::endl;
#endif
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::GetClosestPtPointToSeg(
    const GlobalPoint& a, const GlobalPoint& b, const GlobalPoint& p,
    GlobalPoint* const d, Float64_t* const t) const {
  GlobalPoint v_ab;
  v_ab.x = b.x - a.x;
  v_ab.y = b.y - a.y;

  GlobalPoint v_ap;
  v_ap.x = p.x - a.x;
  v_ap.y = p.y - a.y;

  // Project c onto ab, but deferring divide by Dot(ab, ab)
  *t = v_ab.x * v_ap.x + v_ab.y * v_ap.y;
  if (*t <= 0.0) {
    // c projects outside the [a,b] interval, on the a side; clamp to a
    *t = 0.0;
    *d = a;
  } else {
    // Always nonnegative since denom = ||ab||^2
    const Float64_t denom = v_ab.x * v_ab.x + v_ab.y * v_ab.y;
    if (*t >= denom) {
      // c projects outside the [a,b] interval, on the b side; clamp to b
      *t = 1.0;
      *d = b;
    } else {
      // c projects inside the [a,b] interval; must do deferred divide now
      *t = (*t) / denom;
      d->x = a.x + (*t) * v_ab.x;
      d->y = a.y + (*t) * v_ab.y;
    }
  }
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::UpdateLanesTopology() {
#if ENABLE_HD_MAP_TRACE
  LOG_INFO(5) << "\n>>>>>> Update lanes topology.";
#endif

  for (Int32_t i = 0; i < lane_table_storage_.Size(); ++i) {
    Lane& lane = lane_table_storage_[i];
#if ENABLE_HD_MAP_TRACE
    LOG_INFO(5) << "Current lane (id=" << lane.topology_id().id.id << ")";
#endif

    // Topology
    // Neighbors
    for (Int32_t j = 0; j < lane.left_neighbor_forward_lanes().Size(); ++j) {
      Lane::TopologyId& topology_id = lane.left_neighbor_forward_lanes()[j];
      const Int32_t* const lane_index = lane_table_.Find(topology_id.id.id);
      if (Nullptr_t != lane_index) {
        topology_id.index = *lane_index;
      } else {
#if ENABLE_HD_MAP_TRACE
        LOG_ERR << "Can't find left lane (id=" << topology_id.id.id << ").";
#endif
      }
    }
    for (Int32_t j = 0; j < lane.right_neighbor_forward_lanes().Size(); ++j) {
      Lane::TopologyId& topology_id = lane.right_neighbor_forward_lanes()[j];
      const Int32_t* const lane_index = lane_table_.Find(topology_id.id.id);
      if (Nullptr_t != lane_index) {
        topology_id.index = *lane_index;
      } else {
#if ENABLE_HD_MAP_TRACE
        LOG_ERR << "Can't find right lane (id=" << topology_id.id.id << ").";
#endif
      }
    }
    for (Int32_t j = 0; j < lane.left_neighbor_reverse_lanes().Size(); ++j) {
      Lane::TopologyId& topology_id = lane.left_neighbor_reverse_lanes()[j];
      const Int32_t* const lane_index = lane_table_.Find(topology_id.id.id);
      if (Nullptr_t != lane_index) {
        topology_id.index = *lane_index;
      } else {
#if ENABLE_HD_MAP_TRACE
        LOG_ERR << "Can't find reverse left lane (id=" << topology_id.id.id << ").";
#endif
      }
    }
    for (Int32_t j = 0; j < lane.right_neighbor_reverse_lanes().Size(); ++j) {
      Lane::TopologyId& topology_id = lane.right_neighbor_reverse_lanes()[j];
      const Int32_t* const lane_index = lane_table_.Find(topology_id.id.id);
      if (Nullptr_t != lane_index) {
        topology_id.index = *lane_index;
      } else {
#if ENABLE_HD_MAP_TRACE
        LOG_ERR << "Can't find reverse right lane (id=" << topology_id.id.id << ").";
#endif
      }
    }
    // Connected
    lane.predecessor_lanes().Clear();
    // for (Int32_t j = 0; j < lane.predecessor_lanes().Size(); ++j) {
    //  Lane::TopologyId& topology_id = lane.predecessor_lanes()[j];
    //  const Int32_t* lane_index = lane_table_.Find(topology_id.id.id);
    //  if (Nullptr_t != lane_index) {
    //    topology_id.index = *lane_index;
    //  }
    // }

    // 重构前向车道的目的：
    //   1. 去除前向相连车道ID中地图中不存在的车道ID；
    //   2. 去除前向相连车道ID中重复出现的车道ID。
    common::StaticVector<Lane::TopologyId,
        Lane::MAX_TOPOLOGY_CONNECTED_LANE_NUM> tmp_successor_lanes;
    lane.successor_lanes_start_point().Clear();
    for (Int32_t j = 0; j < lane.successor_lanes().Size(); ++j) {
      Lane::TopologyId& topology_id = lane.successor_lanes()[j];
      const Int32_t* const lane_index = lane_table_.Find(topology_id.id.id);

      if (Nullptr_t != lane_index) {
        if (IsValidLaneIndex(*lane_index)) {
          topology_id.index = *lane_index;
          const Lane& next_lane = lane_table_storage_[topology_id.index];
          if (next_lane.central_curve().points().Size() > 1) {
            if (!FindLaneFromConnectedLanesTable(
                  topology_id.index, tmp_successor_lanes)) {
              tmp_successor_lanes.PushBack(topology_id);
              common::PathPoint* const start_point =
                  lane.successor_lanes_start_point().Allocate();
              next_lane.central_curve().FindNearest(
                    lane.central_curve().points().Back(), start_point);
            }
          }
        }
      } else {
#if ENABLE_HD_MAP_TRACE
        LOG_ERR << "Can't find successor lane(id=" << topology_id.id.id << ").";
#endif
      }
    }
    lane.successor_lanes() = tmp_successor_lanes;
  }

  // predecessor
  for (Int32_t i = 0; i < lane_table_storage_.Size(); ++i) {
    Lane& lane = lane_table_storage_[i];
    for (Int32_t j = 0; j < lane.successor_lanes().Size(); ++j) {
      Lane::TopologyId& topology_id = lane.successor_lanes()[j];
      if (!IsValidLaneIndex(topology_id.index)) {
        continue;
      }

      Lane& next_lane = lane_table_storage_[topology_id.index];
      if (!FindLaneFromConnectedLanesTable(lane.topology_id().index,
                                           next_lane.predecessor_lanes())) {
        next_lane.predecessor_lanes().PushBack(lane.topology_id());
        common::PathPoint* const start_point =
            next_lane.predecessor_lanes_start_point().Allocate();
        lane.central_curve().FindNearest(
              next_lane.central_curve().points().Front(), start_point);
      }
    }
  }
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::IsPointInMapBoundingBox(Float64_t x, Float64_t y) {
  if ((map_bounding_box_.min_x <= x) && (x <= map_bounding_box_.max_x) &&
      (map_bounding_box_.min_y <= y) && (y <= map_bounding_box_.max_y)) {
    return true;
  }

  return false;
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::ConvertPointsToRelCoordinate(
    const common::StaticVector<GlobalPoint,
              common::Path::kMaxPathPointNum>& global_points,
    common::StaticVector<common::Vec2d,
              common::Path::kMaxPathPointNum>* const relative_points) {
  common::Matrix<Float64_t, 2, 1> point_conv;
  relative_points->Resize(global_points.Size());

  for (Int32_t i = 0; i < global_points.Size(); ++i) {
    point_conv(0) = global_points[i].x;
    point_conv(1) = global_points[i].y;

    common::TransformVert_2D(mat_convert_global_to_rel_coor_, &point_conv);

    common::Vec2d& rel_point = relative_points->GetData(i);
    rel_point.set_x(static_cast<common::geo_var_t>(point_conv(0)));
    rel_point.set_y(static_cast<common::geo_var_t>(point_conv(1)));
  }
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::ConvertPointsToRelCoordinate(
    const common::RingBuffer<HoldPoint,
        common::Path::kMaxPathPointNum>& hold_points,
    common::StaticVector<common::Vec2d,
        common::Path::kMaxPathPointNum>* const relative_points) {
  common::Matrix<Float64_t, 2, 1> point_conv;
  relative_points->Clear();

  common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>::
      const_iterator point_it = hold_points.cbegin();
  const common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>::
      const_iterator point_it_end = hold_points.cend();
  for (; point_it != point_it_end; ++point_it) {
    point_conv(0) = point_it->point.x;
    point_conv(1) = point_it->point.y;

    common::TransformVert_2D(mat_convert_global_to_rel_coor_, &point_conv);

    common::Vec2d* const rel_point = relative_points->Allocate();
    if (Nullptr_t != rel_point) {
      rel_point->set_x(static_cast<common::geo_var_t>(point_conv(0)));
      rel_point->set_y(static_cast<common::geo_var_t>(point_conv(1)));
    } else {
      break;
    }
  }
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void HDMap::BuildLaneSegmentKDTree() {
  phoenix::common::AABBoxKDTreeParams params;
  params.max_leaf_dimension = 5.0F;  // meters.
  params.max_leaf_size = 16;

  lane_segment_kdtree_.SetKDTreeParams(params);
  lane_segment_kdtree_.Clear();
  temp_data_.tree_nodes_stack.Clear();

  LaneSegmentObj obj;
  for (Int32_t lane_index = 0;
       lane_index < lane_table_storage_.Size(); ++lane_index) {
    // const Lane& lane = lane_table_storage_[lane_index];
    const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        points = lane_table_storage_[lane_index].central_curve().points();
    for (Int32_t obj_index = 0; obj_index < (points.Size()-1); ++obj_index) {
      obj.lane_index = lane_index;
      obj.object_index = obj_index;
      common::AABBox2d box(points[obj_index], points[obj_index + 1]);
      lane_segment_kdtree_.AddObject(obj, box);
    }
  }

  lane_segment_kdtree_.Partition(temp_data_.tree_nodes_stack);
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::FindLaneFromNeighborLanesTable(
    const Int32_t lane_index,
    const common::StaticVector<NeighborsLaneInfo,
        MAX_NEIGHBOR_LANE_NUM>& neighbor_lanes) const {
  if (neighbor_lanes.Empty() || (lane_index < 0)) {
    return false;
  }

  for (Int32_t i = 0; i < neighbor_lanes.Size(); ++i) {
    if (lane_index == neighbor_lanes[i].lane_index) {
      return true;
    }
  }

  return false;
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool HDMap::FindLaneFromConnectedLanesTable(
    const Int32_t lane_index,
    const common::StaticVector<Lane::TopologyId,
        Lane::MAX_TOPOLOGY_CONNECTED_LANE_NUM>& connected_lanes) const {
  if (connected_lanes.Empty() || (lane_index < 0)) {
    return false;
  }

  for (Int32_t i = 0; i < connected_lanes.Size(); ++i) {
    if (lane_index == connected_lanes[i].index) {
      return true;
    }
  }

  return false;
}

/**
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t HDMap::FindLaneFromLanesInBoundary(const ID& lane_id) const {
  for (Int32_t i = 0; i < lanes_in_boundary_.Size(); ++i) {
    if (lane_id == lanes_in_boundary_[i].lane_id) {
      return (i);
    }
  }

  return (-1);
}

/*
 * @brief 根据地图拓扑关系，搜索前后相连的车道
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/01/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void HDMap::SearchConnectedLanes(
    const SearchConnectedLanesParam& param,
    Int32_t start_lane_idx, const common::PathPoint& start_point_on_lane,
    common::StaticVector<common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>
    , MAX_LANE_SEGMENT_NUM>* const search_result) {
#if ENABLE_HD_MAP_PERFORMANCE_TEST
  phoenix::common::Stopwatch performance_timer;
#endif

  if (!IsValidLaneIndex(start_lane_idx)) {
    LOG_ERR << "Invalid start lane index.";
    return;
  }

  // 起始道路段
  // 道路信息
  const Lane* lane = &(GetLane(start_lane_idx));
  const common::StaticVector<Lane::TopologyId,
      Lane::MAX_TOPOLOGY_CONNECTED_LANE_NUM>* connected_lanes = Nullptr_t;
  // 道路段
  LaneSeg lane_seg;

  lane_seg.lane_id = lane->topology_id().id;
  lane_seg.lane_index = start_lane_idx;

  if (param.search_forward) {
    // 向前搜索
    lane_seg.start_s = start_point_on_lane.s;
    lane_seg.end_s = lane->central_curve().total_length();

    if ((lane_seg.end_s - lane_seg.start_s) >
        (param.search_len - phoenix::common::kMathEpsilonF)) {
      // 此条道路前方的长度足够长了
      common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>* const
          search_lane_segs = search_result->Allocate();
      if (Nullptr_t == search_lane_segs) {
        LOG_ERR << "Can't append lane list, storage is full.";
        return;
      }
      lane_seg.end_s = lane_seg.start_s + param.search_len;
      search_lane_segs->PushBack(lane_seg);
      return;
    }
    if (lane->successor_lanes().Empty()) {
      // 此条道路没有向前连接的道路了
      common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>* const
          search_list = search_result->Allocate();
      if (Nullptr_t == search_list) {
        LOG_ERR << "Can't append lane list, storage is full.";
        return;
      }
      search_list->PushBack(lane_seg);
      return;
    }
  } else {
    // 向后搜索
    lane_seg.start_s = 0.0F;
    lane_seg.end_s = start_point_on_lane.s;

    if ((lane_seg.end_s - lane_seg.start_s) >
        (param.search_len - phoenix::common::kMathEpsilonF)) {
      // 此条道路后方的长度足够长了
      common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>* const
          search_lane_segs = search_result->Allocate();
      if (Nullptr_t == search_lane_segs) {
        LOG_ERR << "Can't append lane list, storage is full.";
        return;
      }
      lane_seg.start_s = lane_seg.end_s - param.search_len;
      search_lane_segs->PushBack(lane_seg);
      return;
    }
    if (lane->predecessor_lanes().Empty()) {
      // 此条道路没有向后连接的道路了
      common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>* const
          search_lane_segs = search_result->Allocate();
      if (Nullptr_t == search_lane_segs) {
        LOG_ERR << "Can't append lane list, storage is full.";
        return;
      }
      search_lane_segs->PushBack(lane_seg);
      return;
    }
  }

  // 定义堆栈，用来在地图中搜索相连接的道路段
  TemporaryData::StackForFindingLanes& stack =
      temp_data_.stack_for_finding_lanes;

  // 使用前需要将堆栈清空
  stack.Clear();
  // 将此道路段压入堆栈
  TemporaryData::StackForFindingLanes::StackNode* stack_node = stack.Allocate();
  if (Nullptr_t == stack_node) {
    LOG_ERR << "Storage of stack is full.";
    return;
  }
  stack_node->lanes.PushBack(lane_seg);
  stack_node->accumulated_s = lane_seg.end_s - lane_seg.start_s;
  stack_node->access_index = 0;

  // 使用深度优先搜索算法，获取前后相连的车道
  bool storage_is_full = false;
  while (!(stack.Empty())) {
    // 取出栈顶元素
    stack_node = &(stack.Top());
    // 栈顶中保存的最后一段车道信息
    lane = &(GetLane(stack_node->lanes.Back().lane_index));
    if (param.search_forward) {
      connected_lanes = &(lane->successor_lanes());
    } else {
      connected_lanes = &(lane->predecessor_lanes());
    }

    if (connected_lanes->Empty()) {
      // 此车道没有连接的车道了
      stack.Pop();
      continue;
    } else if (stack_node->access_index >= connected_lanes->Size()) {
      // 此车道所有连接的车道都访问过了
      stack.Pop();
      continue;
    } else {
      // nothing to do
    }

    Int32_t lane_idx = (*connected_lanes)[stack_node->access_index].index;
    if (param.search_by_lane_id) {
      const Int32_t* const tmp_lane_index =
          lane_table_.Find((*connected_lanes)[stack_node->access_index].id.id);
      if (Nullptr_t != tmp_lane_index) {
        lane_idx = *tmp_lane_index;
      } else {
        LOG_ERR << "Can't find lane index by lane id.";
      }
    }
    if (!IsValidLaneIndex(lane_idx)) {
      // 道路索引无效
      stack.Pop();
      LOG_ERR << "Detected invalid lane index in successor lanes.";
      continue;
    }

    // 添加一条搜索车道序号列表
    common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>* const
        search_lane_segs = search_result->Allocate();
    if (Nullptr_t == search_lane_segs) {
      LOG_ERR << "Can't append lane list, storage is full.";
      break;
    }
    search_lane_segs->Clear();
    *search_lane_segs = stack_node->lanes;

    while (connected_lanes->Size() > 0) {
      // 此车道还有连接的道路
      lane_idx = (*connected_lanes)[stack_node->access_index].index;
      if (param.search_by_lane_id) {
        const Int32_t* const tmp_lane_index =
            lane_table_.Find((*connected_lanes)[stack_node->access_index].id.id);
        if (Nullptr_t != tmp_lane_index) {
          lane_idx = *tmp_lane_index;
        } else {
          LOG_ERR << "Can't find lane index by lane id.";
        }
      }
      if (!IsValidLaneIndex(lane_idx)) {
        // 此车道连接的道路索引无效
        LOG_ERR << "Detected invalid lane index in successor lanes.";
        break;
      }
      // 更新车道段为此车道连接的车道段
      const map_var_t accumulated_s = stack_node->accumulated_s;
      lane_seg.lane_index = lane_idx;

      if (param.search_forward) {
        lane_seg.start_s = lane->successor_lanes_start_point()
            [stack_node->access_index].s;
      } else {
        lane_seg.start_s = 0.0F;
        lane_seg.end_s = lane->predecessor_lanes_start_point()
            [stack_node->access_index].s;
      }

      // 更新连接车道
      lane = &(GetLane(lane_idx));
      if (param.search_forward) {
        connected_lanes = &(lane->successor_lanes());
      } else {
        connected_lanes = &(lane->predecessor_lanes());
      }

      lane_seg.lane_id = lane->topology_id().id;
      // lane_seg.start_s = 0.0F;

      if ((accumulated_s+lane->central_curve().total_length() -
           lane_seg.start_s) > 
	   (param.search_len - phoenix::common::kMathEpsilonF)) {
        // 加上连接的车道，足够长了
        if (param.search_forward) {
          lane_seg.end_s = param.search_len - accumulated_s + lane_seg.start_s;
        } else  {
          lane_seg.start_s = lane_seg.end_s - param.search_len + accumulated_s;
        }

        // 添加此连接的车道
        search_lane_segs->PushBack(lane_seg);

        // 节点访问索引加1
        stack_node->access_index++;
        break;
      } else {
        // 加上连接的车道，还不足够长，需要继续添加
       if (param.search_forward) {
          lane_seg.end_s = lane->central_curve().total_length();
        } else  {
          lane_seg.start_s = 0.0F;
        }
      }

      // 添加此连接的车道
      search_lane_segs->PushBack(lane_seg);

      // 节点访问索引加1
      stack_node->access_index++;
      // 将此道路段压入堆栈
      TemporaryData::StackForFindingLanes::StackNode* old_stack_node = stack_node;
      stack_node = stack.Allocate();
      if (Nullptr_t == stack_node) {
        storage_is_full = true;
        LOG_ERR << "Storage of stack is full.";
        break;
      }
      stack_node->lanes = old_stack_node->lanes;
      stack_node->lanes.PushBack(lane_seg);
      stack_node->accumulated_s =
          accumulated_s + lane_seg.end_s - lane_seg.start_s;
      stack_node->access_index = 0;
    }
    if (storage_is_full) {
      break;
    }
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  std::cout << "Searching connected lanes spend "
            << performance_timer.Elapsed()
            << "ms." << std::endl;
#endif
}

void HDMap::PrintHDMap() const {
  LOG_INFO(5) << "\n\n###### HD-Map ###### (begin)";

  Int32_t lane_num = lane_table_storage_.Size();
  for (Int32_t lane_idx = 0; lane_idx < lane_num; ++lane_idx) {
    const Lane& lane = lane_table_storage_[lane_idx];
    LOG_INFO(5) << "lane index: " << lane_idx;
    LOG_INFO(5) << "  lane id: " << lane.topology_id().id.id;

    LOG_INFO(5) << "  next_lane:";
    for (Int32_t i = 0; i < lane.successor_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.successor_lanes()[i];
      LOG_INFO(5) << "    [" << i << "]: idx=" << id.index
                  << ", id=" << id.id.id;
    }

    LOG_INFO(5) << "  left_lane:";
    for (Int32_t i = 0; i < lane.left_neighbor_forward_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.left_neighbor_forward_lanes()[i];
      LOG_INFO(5) << "    [" << i << "]: idx=" << id.index
                  << ", id=" << id.id.id;
    }

    LOG_INFO(5) << "  right_lane:";
    for (Int32_t i = 0; i < lane.right_neighbor_forward_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.right_neighbor_forward_lanes()[i];
      LOG_INFO(5) << "    [" << i << "]: idx=" << id.index
                  << ", id=" << id.id.id;
    }

#if 0
    LOG_INFO(5) << "  points:";
    for (Int32_t i = 0; i < lane.central_curve().points().Size(); ++i) {
      const common::Vec2d& p = lane.central_curve().points()[i];
      LOG_INFO(5) << "    " << p.x() << "," << p.y();
    }
#endif
  }

#if (OUTPUT_HD_MAP_WITH_APOLLO_TYPE)
  apollo::hdmap::Map map;
  for (Int32_t lane_idx = 0; lane_idx < lane_num; ++lane_idx) {
    const Lane& lane = lane_table_storage_[lane_idx];

    // Add lane to protoc
    apollo::hdmap::Lane* map_lane = map.add_lane();

    // ID
    apollo::hdmap::Id* map_lane_id = map_lane->mutable_id();
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
    map_lane_id->set_id(lane.topology_id().id.id);
#else
    std::ostringstream ostr;
    ostr << lane.topology_id().id.id;
    map_lane_id->set_id(ostr.str());
#endif

    // Length
    map_lane->set_length(lane.central_curve().total_length());

    // Topology
    // Predecessor
    for (Int32_t i = 0; i < lane.predecessor_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.predecessor_lanes()[i];

      map_lane_id = map_lane->add_predecessor_id();
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      map_lane_id->set_id(id.id.id);
#else
      std::ostringstream ostr;
      ostr << id.id.id;
      map_lane_id->set_id(ostr.str());
#endif
    }
    // Successor
    for (Int32_t i = 0; i < lane.successor_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.successor_lanes()[i];

      map_lane_id = map_lane->add_successor_id();
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      map_lane_id->set_id(id.id.id);
#else
      std::ostringstream ostr;
      ostr << id.id.id;
      map_lane_id->set_id(ostr.str());
#endif
    }
    // Left
    for (Int32_t i = 0; i < lane.left_neighbor_forward_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.left_neighbor_forward_lanes()[i];

      map_lane_id = map_lane->add_left_neighbor_forward_lane_id();
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      map_lane_id->set_id(id.id.id);
#else
      std::ostringstream ostr;
      ostr << id.id.id;
      map_lane_id->set_id(ostr.str());
#endif
    }
    // Right
    for (Int32_t i = 0; i < lane.right_neighbor_forward_lanes().Size(); ++i) {
      const Lane::TopologyId& id = lane.right_neighbor_forward_lanes()[i];

      map_lane_id = map_lane->add_right_neighbor_forward_lane_id();
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      map_lane_id->set_id(id.id.id);
#else
      std::ostringstream ostr;
      ostr << id.id.id;
      map_lane_id->set_id(ostr.str());
#endif
    }

    // Center curve
    apollo::hdmap::LineSegment* line_seg =
        map_lane->mutable_central_curve()->add_segment()->mutable_line_segment();
    for (Int32_t i = 0; i < lane.central_curve().points().Size(); ++i) {
      const common::Vec2d& p = lane.central_curve().points()[i];

      apollo::common::PointENU* p_enu = line_seg->add_point();
      p_enu->set_x(p.x());
      p_enu->set_y(p.y());
      p_enu->set_z(0.0);
    }

    // left boundary
    for (Int32_t i = 0; i < lane.left_boundary().boundary_width_samples.Size(); ++i) {
      const Lane::BoundaryWidthAssociation& sample =
          lane.left_boundary().boundary_width_samples[i];

      apollo::hdmap::LaneSampleAssociation* lane_sample =
          map_lane->add_left_sample();

      lane_sample->set_s(sample.s);
      lane_sample->set_width(sample.width);
    }

    // right boundary
    for (Int32_t i = 0; i < lane.right_boundary().boundary_width_samples.Size(); ++i) {
      const Lane::BoundaryWidthAssociation& sample =
          lane.right_boundary().boundary_width_samples[i];

      apollo::hdmap::LaneSampleAssociation* lane_sample =
          map_lane->add_right_sample();

      lane_sample->set_s(sample.s);
      lane_sample->set_width(sample.width);
    }
  }

  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::OstreamOutputStream;
  using google::protobuf::TextFormat;

  if (lane_num > 0) {
    std::cout << "\n\n#### Map (Apollo format): " << std::endl;
    OstreamOutputStream output(&std::cout);
    TextFormat::Print(map, &output);
  }
#endif  // #if (OUTPUT_HD_MAP_WITH_APOLLO_TYPE)

  LOG_INFO(5) << "\n###### HD-Map ###### (end)";
}


}  // namespace driv_map
}  // namespace phoenix


