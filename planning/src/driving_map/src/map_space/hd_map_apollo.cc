/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       hd_map_series_c.cc
 * @brief      当外部地图格式为protobuf形式发送的OpenDrive地图格式时，将接收到的\n
 *             外部地图数据解析为内部地图数据格式。
 * @details    定义了内部地图的数据格式及相关访问接口
 *
 * @author     pengc
 * @date       2020.06.06
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/06  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "map_space/hd_map.h"

#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "utils/linear_interpolation.h"
#include "pos_filter_wrapper.h"

#define ENABLE_HD_MAP_TRACE (0)
#define ENABLE_HD_MAP_PERFORMANCE_TEST (0)


namespace phoenix {
namespace driv_map {


#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO

bool HDMap::ConstructMapInfo(const apollo::hdmap::Map& map) {
#if ENABLE_HD_MAP_TRACE
  std::cout << "######### Construct HDMap >>>>>>>>>" << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of constructing hd-map (Start)
  phoenix::common::Stopwatch performance_timer_constructing_hd_map;
#endif

  // Clear old information set before.
  Clear();

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of parsing lanes in map (Start)
  phoenix::common::Stopwatch performance_timer_parse_lanes;
#endif

  // Parse all lanes in the map.
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Lane >::const_iterator
      lane_it = map.lane().begin();
  for (; lane_it != map.lane().end(); ++lane_it) {
    if (!ParseLaneInfo(*lane_it)) {
#if ENABLE_HD_MAP_TRACE
      LOG_ERR << "Failed to parse lane [" << lane_it->id().id().c_str() << "].";
#endif
      continue;
    }
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of parsing lanes in map (End)
  std::cout << "Parsing lanes in map spend "
            << performance_timer_parse_lanes.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of updating lanes topology (Start)
  phoenix::common::Stopwatch performance_timer_update_lane_topology;
#endif

  UpdateLanesTopology();

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of updating lanes topology (End)
  std::cout << "Updating lanes topology spend "
            << performance_timer_update_lane_topology.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of building lane segment kd-tree (Start)
  phoenix::common::Stopwatch performance_timer_build_lane_kd_tree;
#endif

  BuildLaneSegmentKDTree();

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of building lane segment kd-tree (End)
  std::cout << "Building lane segment kd-tree spend "
            << performance_timer_build_lane_kd_tree.Elapsed()
            << "ms." << std::endl;
#endif

  // Parse traffic ligth information in HD-Map
#if 1
  //map.signal()
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Signal >::const_iterator
      signal_it = map.signal().begin();
  for (; signal_it != map.signal().end(); ++signal_it) {
    if (!ParseMapTrafficLight(*signal_it)) {
      LOG_ERR << "Failed to parse signal [" << signal_it->id().id().c_str() << "].";
      continue;
    }
  }
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of constructing hd-map (End)
  std::cout << "Constructing hd-map spend "
            << performance_timer_constructing_hd_map.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_TRACE
  std::cout << "<<<<<<<<< Construct HDMap #########" << std::endl;
#endif

  return (true);
}

bool HDMap::ConstructRoutingInfo(const apollo::routing::RoutingResponse& routing) {
#if ENABLE_HD_MAP_TRACE
  std::cout << "######### Construct Routing >>>>>>>>>" << std::endl;
#endif

  if (lanes_in_boundary_.Empty()) {
    LOG_ERR << "The lanes table in map is empty.";
    return false;
  }

  routing_.Clear();

  ID lane_id;
  Int32_t index = -1;
  Float64_t start_s = 0;
  Float64_t end_s = 0;

#if ENABLE_HD_MAP_TRACE
  std::cout << "The original routing segments are :" << std::endl;
#endif

  Int32_t loop_count = 0;
  ::google::protobuf::RepeatedPtrField<
      ::apollo::routing::RoadSegment >::const_iterator road_it =
      routing.road().begin();
  ::google::protobuf::RepeatedPtrField<
      ::apollo::routing::RoadSegment >::const_iterator road_it_end =
      routing.road().end();
  for (; road_it != road_it_end; ++road_it) {
    // Road
    ::google::protobuf::RepeatedPtrField<
        ::apollo::routing::Passage >::const_iterator passage_it =
        road_it->passage().begin();
    ::google::protobuf::RepeatedPtrField<
        ::apollo::routing::Passage >::const_iterator passage_it_end =
        road_it->passage().end();
    for (; passage_it != passage_it_end; ++passage_it) {
      // Passage
      ::google::protobuf::RepeatedPtrField<
          ::apollo::routing::LaneSegment >::const_iterator segment_it =
          passage_it->segment().begin();
      ::google::protobuf::RepeatedPtrField<
          ::apollo::routing::LaneSegment >::const_iterator segment_it_end =
          passage_it->segment().end();
      for (; segment_it != segment_it_end; ++segment_it) {
        // Segments
        loop_count++;
        if (segment_it->has_id() &&
            segment_it->has_start_s() &&
            segment_it->has_end_s()) {
          lane_id.id = segment_it->id();
          index = FindLaneFromLanesInBoundary(lane_id);
          if (index >= 0) {
            start_s = segment_it->start_s() - lanes_in_boundary_[index].start_s;
            if (start_s < 0) {
              start_s = 0;
            }
            end_s = segment_it->end_s();
            if (end_s > lanes_in_boundary_[index].end_s) {
              end_s = lanes_in_boundary_[index].end_s -
                  lanes_in_boundary_[index].start_s;
              if (end_s < 0) {
                end_s = 0;
              }
            } else {
              end_s -= lanes_in_boundary_[index].start_s;
              if (end_s < 0) {
                end_s = 0;
              }
            }
            if (end_s - start_s > 0.1f) {
              RoutingSection* section = routing_.routing_sections().Allocate();
              if (Nullptr_t == section) {
                LOG_ERR << "Can't store routing segment any more, "
                           "storage is full.";
                return false;
              }
              RoutingSegment* seg = section->neighbors.Allocate();
              if (Nullptr_t == seg) {
                routing_.routing_sections().PopBack();
                LOG_ERR << "Can't store routing segment any more, "
                           "storage is full.";
                return false;
              }
              seg->lane_id = lane_id;
              seg->lane_index = lanes_in_boundary_[index].lane_index;
              seg->start_s = start_s;
              seg->end_s = end_s;

#if ENABLE_HD_MAP_TRACE
              std::cout << "Segment[" << loop_count - 1 << "]: ";
              std::cout << "{[" << 0
                        << "] id=" << segment_it->id()
                        << ", start_s=" << segment_it->start_s()
                        << ", end_s=" << segment_it->end_s()
                        << "} " << std::endl;
#endif
            }
          }
        } else {
          LOG_ERR << "Detected invalid routing segment.";
          return false;
        }
      }
    }
  }

#if ENABLE_HD_MAP_TRACE
  std::cout << "After parsing, the routing segments are:" << std::endl;
  for (Int32_t i = 0; i < routing_.routing_sections().Size(); ++i) {
    std::cout << "Segment[" << i << "]: ";
    for (Int32_t j = 0; j < routing_.routing_sections()[i].neighbors.Size();
         ++j) {
      std::cout << "{[" << j
                << "] id=" << routing_.routing_sections()[i].neighbors[j].
                   lane_id.id
                << ", index=" << routing_.routing_sections()[i].neighbors[j].
                   lane_index
                << ", start_s=" << routing_.routing_sections()[i].neighbors[j].
                   start_s
                << ", end_s=" << routing_.routing_sections()[i].neighbors[j].
                   end_s
                << "} ";
    }
    std::cout << std::endl;
  }
#endif

#if ENABLE_HD_MAP_TRACE
  std::cout << "<<<<<<<<< Construct Routing #########" << std::endl;
#endif

  return true;
}

bool HDMap::ParseLaneInfo(const apollo::hdmap::Lane& lane) {
#if ENABLE_HD_MAP_TRACE
  std::cout << "######### ParseLaneInfo ("
            << lane.id().id().c_str()
            << ") >>>>>>>>>" << std::endl;
#endif

  // Get a memory space from lane table storage.
  Int32_t lane_storage_index = -1;
  Lane* lane_info = lane_table_storage_.Allocate(&lane_storage_index);
  if (Nullptr_t == lane_info) {
    LOG_ERR << "Can't save lane information anymore, storage is full.";
    return false;
  }

  // Central Line
  // Get central line of lane which is in the bounding box set before.
  Int32_t loop_count = 0;
  Float64_t accumulated_s = 0;
  Float64_t distance = 0;

  bool prev_global_point_valid = false;
  GlobalPoint prev_global_point;
  GlobalPoint curr_global_point;

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of taking path in boundary (Start)
  phoenix::common::Stopwatch performance_timer_take_path;
#endif

  temp_data_.take_path.Clear();
  ::google::protobuf::RepeatedPtrField<
      ::apollo::hdmap::CurveSegment >::const_iterator curve_it =
      lane.central_curve().segment().begin();
  for (; curve_it != lane.central_curve().segment().end(); ++curve_it) {
    if (curve_it->has_line_segment()) {
      ::google::protobuf::RepeatedPtrField<
          ::apollo::common::PointENU >::const_iterator point_it =
          curve_it->line_segment().point().begin();
      ::google::protobuf::RepeatedPtrField<
          ::apollo::common::PointENU >::const_iterator point_it_end =
          curve_it->line_segment().point().end();
      for (; point_it != point_it_end; ++point_it) {
        if (loop_count > 0) {
          // 计算两点间的距离
          distance = common::com_sqrt(
                common::Square(point_it->x() - prev_global_point.x) +
                common::Square(point_it->y() - prev_global_point.y));

          // 过滤掉距离过近的点
          if (distance < min_lane_point_interval_) {
            continue;
          }
          // 计算原始地图车道中心线处每个点的轨迹长
          accumulated_s += distance;
        }

        // current point
        curr_global_point.x = point_it->x();
        curr_global_point.y = point_it->y();
        curr_global_point.s = accumulated_s;

        // 判断该点是否位于包围盒内
        bool is_in_boundary =
            IsPointInMapBoundingBox(point_it->x(), point_it->y());

        //printf("loop_count=%d, pos=(%f,%f), is_in_boundary=%d\n",
        //		loop_count, point_it->x(), point_it->y(), is_in_boundary);

        // Take points in boundary
        TakePointsInBoundary(is_in_boundary, prev_global_point_valid,
                             prev_global_point, curr_global_point);

        // 保存这个点，下个迭代会使用
        prev_global_point_valid = true;
        prev_global_point.x = point_it->x();
        prev_global_point.y = point_it->y();
        prev_global_point.s = accumulated_s;
        loop_count++;
      }
    }
  }

  // Select better segment
  Int32_t better_seg = 0;
  if (false == SelectBetterLaneSeg(&better_seg)) {
#if ENABLE_HD_MAP_TRACE
    LOG_ERR << "There are not enough points in this lane, global("
            << current_global_position_.x << "," << current_global_position_.y << ").";
#endif
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }

  LimitLaneRang(better_seg);

  if (temp_data_.take_path.lane_seg[better_seg].hold_points.Size() < 2) {
#if ENABLE_HD_MAP_TRACE
    LOG_ERR << "There are not enough points in this lane, global("
            << current_global_position_.x << "," << current_global_position_.y << ").";
#endif
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }

  Float64_t start_s = temp_data_.take_path.lane_seg[better_seg].
      hold_points.Front()->point.s;
  Float64_t end_s = temp_data_.take_path.lane_seg[better_seg].
      hold_points.Back()->point.s;

#if ENABLE_HD_MAP_TRACE
  std::cout << "After take points, The better segment is "
            << better_seg << std::endl;
  std::cout << "The number of points in better segment is "
            << temp_data_.take_path.lane_seg[better_seg].
               hold_points.Size()
            << std::endl;
  // std::cout << "The points in boundary box is :" << std::endl;
  // for (Int32_t i = 0;
  //      i < tmp_data_.lane_seg[better_seg].global_points.Size();
  //      ++i) {
  //  std::cout << "point[" << i
  //            << "] = {" << tmp_data_.lane_seg[better_seg].global_points[i].x
  //            << ", " << tmp_data_.lane_seg[better_seg].global_points[i].y
  //            << "}" << std::endl;
  // }
  std::cout << "start_s=" << start_s
            << ", end_s=" << end_s
            << ", nearest_s=" << temp_data_.take_path.lane_seg[better_seg].
               nearest_point_to_veh.s
            << ", dist_to_veh="
            << common::com_sqrt(temp_data_.take_path.lane_seg[better_seg].
                                sq_dist_to_veh)
            << ", accumulated_s=" << accumulated_s
            << ", total_len="
            << temp_data_.take_path.lane_seg[better_seg].
               hold_points.Back()->point.s -
               temp_data_.take_path.lane_seg[better_seg].
               hold_points.Front()->point.s
            << std::endl;
#endif

  // Convert global points to points related to vehicle coordinate.
  temp_data_.points_2d_1.Clear();
  //const GlobalPoint& origin, const GlobalPoint& correct,
  ConvertPointsToRelCoordinate(
        temp_data_.take_path.lane_seg[better_seg].hold_points,
        &temp_data_.points_2d_1);

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of taking path in boundary (End)
  std::cout << "Taking path in boundary spend "
            << performance_timer_take_path.Elapsed()
            << "ms." << std::endl;
#endif

  // for (Int32_t i = 0; i < temp_data_.points.Size(); ++i) {
  //  std::cout << i+1
  //            << "," << temp_data_.points[i].x()
  //            << "," << temp_data_.points[i].y()
  //            << std::endl;
  // }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of constructing path of lane (Start)
  phoenix::common::Stopwatch performance_timer_construct_path;
#endif

  // Save central line to this lane.
  lane_info->central_curve().Clear();
  lane_info->central_curve().Construct(temp_data_.points_2d_1);
  if (lane_info->central_curve().points().Size() < 2) {
    LOG_ERR << "There are not enough points in path of this lane.";
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of constructing path of lane (End)
  std::cout << "Constructing path spend "
            << performance_timer_construct_path.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of parsing other lane information (Start)
  phoenix::common::Stopwatch performance_timer_parsing_other;
#endif

  // Lane Boundary
  ParseLaneBoundary(lane, start_s, end_s, lane_info);

  // Slope
  ParseLaneSlope(lane, start_s, end_s, lane_info);

  // Topology
  ParseLaneTopology(lane, lane_info);

  // Lane ID
  lane_info->topology_id().id.id = lane.id().id();
  lane_info->topology_id().index = lane_storage_index;

  // Lane Type: TBD
  // lane_info->set_lane_type();
  // lane_info->set_turn_type();
  // lane_info->set_direction();

  // Speed limit
  if (lane.has_speed_limit()) {
    lane_info->set_speed_limit_high(lane.speed_limit());
  }

  // Add to lane table
  const Int32_t* lane_index = lane_table_.Find(lane.id().id());
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
  lane_in_boundary->start_s = start_s;
  lane_in_boundary->end_s = end_s;

  lane_table_.Insert(lane.id().id(), lane_storage_index);

#if ENABLE_HD_MAP_PERFORMANCE_TEST
  /// Performance of parsing other lane information (End)
  std::cout << "Parsing other lane info spend "
            << performance_timer_parsing_other.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_TRACE
  std::cout << "<<<<<<<<< ParseLaneInfo #########" << std::endl;
#endif

  return (true);
}

bool HDMap::ParseMapTrafficLight(const apollo::hdmap::Signal& map_signal) {
  Int32_t stop_line_num = map_signal.stop_line_size();
  if (stop_line_num < 1) {
    LOG_WARN << "There isn't stop lines in map signal.";
    return false;
  }

  for (Int32_t i = 0; i < stop_line_num; ++i) {
    const apollo::hdmap::Curve& curve = map_signal.stop_line(i);
    if (curve.segment_size() < 1) {
      LOG_WARN << "There isn't segment in curve.";
      continue;
    }
    const apollo::hdmap::CurveSegment& segment = *(curve.segment().begin());
    if (!segment.has_line_segment()) {
      LOG_WARN << "There isn't line in segment.";
      continue;
    }
    if (segment.line_segment().point_size() < 2) {
      LOG_WARN << "There isn't enough points in line.";
      continue;
    }
    const apollo::common::PointENU& start_enu_point =
        *(segment.line_segment().point().begin());
    const apollo::common::PointENU& end_enu_point =
        *(--(segment.line_segment().point().end()));
    if (com_isnan(start_enu_point.x()) || com_isinf(start_enu_point.x()) ||
        com_isnan(start_enu_point.y()) || com_isinf(start_enu_point.y()) ||
        com_isnan(end_enu_point.x()) || com_isinf(end_enu_point.x()) ||
        com_isnan(end_enu_point.y()) || com_isinf(end_enu_point.y())) {
      LOG_ERR << "Invalid stop line start(" << start_enu_point.x()
              << "," << start_enu_point.y()
              << ") -> end(" << end_enu_point.x()
              << "," << end_enu_point.y() << ").";
      continue;
    }
    Float64_t sq_dist =
        common::Square(end_enu_point.x() - start_enu_point.x()) +
        common::Square(end_enu_point.y() - start_enu_point.y());
    if (sq_dist < 0.01) {
      LOG_WARN << "Too short stop line.";
      continue;
    }
    if (!IsPointInMapBoundingBox(start_enu_point.x(), start_enu_point.y())) {
      // 停止线在范围外
      continue;
    }
    if (!IsPointInMapBoundingBox(end_enu_point.x(), end_enu_point.y())) {
      // 停止线在范围外
      continue;
    }

    common::Vec2d start_point;
    common::Vec2d end_point;
    common::Matrix<Float64_t, 2, 1> point_conv;
    // Transpose start point
    point_conv(0) = start_enu_point.x();
    point_conv(1) = start_enu_point.y();
    common::TransformVert_2D(mat_convert_global_to_rel_coor_, &point_conv);
    start_point.set_x(static_cast<common::geo_var_t>(point_conv(0)));
    start_point.set_y(static_cast<common::geo_var_t>(point_conv(1)));
    // Transpose end point
    point_conv(0) = end_enu_point.x();
    point_conv(1) = end_enu_point.y();
    common::TransformVert_2D(mat_convert_global_to_rel_coor_, &point_conv);
    end_point.set_x(static_cast<common::geo_var_t>(point_conv(0)));
    end_point.set_y(static_cast<common::geo_var_t>(point_conv(1)));

    // Add to traffic light list
    MapTrafficLight* traffic_light = map_traffic_light_table_.Allocate();
    if (Nullptr_t == traffic_light) {
      LOG_ERR << "Can't save traffic light anymore, storage is full.";
      break;
    }
    traffic_light->id.id = map_signal.id().id();
    traffic_light->stop_line.set_start(start_point);
    traffic_light->stop_line.set_end(end_point);
  }

  return (true);
}

void HDMap::ParseLaneBoundary(
    const apollo::hdmap::Lane& lane, Float64_t start_s, Float64_t end_s,
    Lane* lane_info) {
  Int32_t loop_count = 0;
  // left boundary
  ::google::protobuf::RepeatedPtrField<
      ::apollo::hdmap::LaneSampleAssociation
      >::const_iterator lane_boundary_sample_it = lane.left_sample().begin();
  ::google::protobuf::RepeatedPtrField<
      ::apollo::hdmap::LaneSampleAssociation
      >::const_iterator lane_boundary_sample_it_end = lane.left_sample().end();
  for (; lane_boundary_sample_it != lane_boundary_sample_it_end;
       ++lane_boundary_sample_it) {
    if (!lane_boundary_sample_it->has_s() ||
        !lane_boundary_sample_it->has_width()) {
      LOG_WARN << "Detected incorrect map contents about lane boundary.";
      break;
    }
    if ((start_s <= lane_boundary_sample_it->s()) &&
        (lane_boundary_sample_it->s() <= end_s)) {
      if (lane_info->left_boundary().boundary_width_samples.Empty()) {
        if (start_s > 0.1f) {
          Lane::BoundaryWidthAssociation* lane_boundary =
              lane_info->left_boundary().boundary_width_samples.Allocate();
          if (Nullptr_t == lane_boundary) {
            LOG_WARN << "Can't save boundary information anymore, "
                        "storage is full.";
            break;
          }
          lane_boundary->s = 0;
          lane_boundary->width = lane_boundary_sample_it->width();
        }
      } else {
        // 过滤掉距离过近的点
        if ((lane_boundary_sample_it->s() - start_s -
             lane_info->left_boundary().boundary_width_samples.Back().s) <
            min_lane_point_interval_) {
          continue;
        }
      }
      loop_count++;
      Lane::BoundaryWidthAssociation* lane_boundary =
          lane_info->left_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t == lane_boundary) {
        // std::cout << "loop_count=" << loop_count << std::endl;
        LOG_WARN << "Can't save boundary information anymore, storage is full.";
        break;
      }
      lane_boundary->s = lane_boundary_sample_it->s() - start_s;
      lane_boundary->width = lane_boundary_sample_it->width();
    }
  }
  // right boundary
  lane_boundary_sample_it = lane.right_sample().begin();
  lane_boundary_sample_it_end = lane.right_sample().end();
  for (; lane_boundary_sample_it != lane_boundary_sample_it_end;
       ++lane_boundary_sample_it) {
    if (!lane_boundary_sample_it->has_s() ||
        !lane_boundary_sample_it->has_width()) {
      LOG_WARN << "Detected incorrect map contents about lane boundary.";
      break;
    }
    if ((start_s <= lane_boundary_sample_it->s()) &&
        (lane_boundary_sample_it->s() <= end_s)) {
      if (lane_info->right_boundary().boundary_width_samples.Empty()) {
        if (start_s > 0.1f) {
          Lane::BoundaryWidthAssociation* lane_boundary =
              lane_info->right_boundary().boundary_width_samples.Allocate();
          if (Nullptr_t == lane_boundary) {
            LOG_WARN << "Can't save boundary information anymore, "
                        "storage is full.";
            break;
          }
          lane_boundary->s = 0;
          lane_boundary->width = lane_boundary_sample_it->width();
        }
      } else {
        // 过滤掉距离过近的点
        if ((lane_boundary_sample_it->s() - start_s -
             lane_info->right_boundary().boundary_width_samples.Back().s) <
            min_lane_point_interval_) {
          continue;
        }
      }
      Lane::BoundaryWidthAssociation* lane_boundary =
          lane_info->right_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t == lane_boundary) {
        LOG_WARN << "Can't save boundary information anymore, storage is full.";
        break;
      }
      lane_boundary->s = lane_boundary_sample_it->s() - start_s;
      lane_boundary->width = lane_boundary_sample_it->width();
    }
  }
}

void HDMap::ParseLaneSlope(
    const apollo::hdmap::Lane& lane, Float64_t start_s, Float64_t end_s,
    Lane* lane_info) {
#if 0
  ::google::protobuf::RepeatedPtrField<
      ::phoenix::map::LaneSlopeAssociation
      >::const_iterator lane_slope_sample_it = lane.slope().begin();
  for (; lane_slope_sample_it != lane.slope().end(); ++lane_slope_sample_it) {
    if (!lane_slope_sample_it->has_s() || !lane_slope_sample_it->has_slope()) {
      LOG_WARN << "Detected incorrect map contents about lane slope.";
      break;
    }
    if ((start_s <= lane_slope_sample_it->s()) &&
        (lane_slope_sample_it->s() <= end_s)) {
      if (lane_info->slope_samples().Empty()) {
        if (start_s > 0.1f) {
          Lane::SlopeAssociation* lane_slope =
              lane_info->slope_samples().Allocate();
          if (Nullptr_t == lane_slope) {
            LOG_WARN << "Can't save slope information anymore, "
                        "storage is full.";
            break;
          }
          lane_slope->s = 0;
          lane_slope->slope = lane_slope_sample_it->slope();
        }
      } else {
        // 过滤掉距离过近的点
        if ((lane_slope_sample_it->s() -
             lane_info->slope_samples().Back().s) < min_lane_point_interval_) {
          continue;
        }
      }
      Lane::SlopeAssociation* lane_slope =
          lane_info->slope_samples().Allocate();
      if (Nullptr_t == lane_slope) {
        LOG_WARN << "Can't save slope information anymore, storage is full.";
        break;
      }
      lane_slope->s = lane_slope_sample_it->s() - start_s;
      lane_slope->slope = lane_slope_sample_it->slope();
    }
  }
#endif
}

void HDMap::ParseLaneTopology(const apollo::hdmap::Lane& lane, Lane* lane_info) {
  // Neighbors
  if (!lane.left_neighbor_forward_lane_id().empty()) {
    lane_info->left_neighbor_forward_lanes().Resize(1);
    lane_info->left_neighbor_forward_lanes()[0].id.id =
        lane.left_neighbor_forward_lane_id().Get(0).id();
    lane_info->left_neighbor_forward_lanes()[0].index = -1;
  }
  if (!lane.right_neighbor_forward_lane_id().empty()) {
    lane_info->right_neighbor_forward_lanes().Resize(1);
    lane_info->right_neighbor_forward_lanes()[0].id.id =
        lane.right_neighbor_forward_lane_id().Get(0).id();
    lane_info->right_neighbor_forward_lanes()[0].index = -1;
  }
  if (!lane.left_neighbor_reverse_lane_id().empty()) {
    lane_info->left_neighbor_reverse_lanes().Resize(1);
    lane_info->left_neighbor_reverse_lanes()[0].id.id =
        lane.left_neighbor_reverse_lane_id().Get(0).id();
    lane_info->left_neighbor_reverse_lanes()[0].index = -1;
  }
  if (!lane.right_neighbor_reverse_lane_id().empty()) {
    lane_info->right_neighbor_reverse_lanes().Resize(1);
    lane_info->right_neighbor_reverse_lanes()[0].id.id =
        lane.right_neighbor_reverse_lane_id().Get(0).id();
    lane_info->right_neighbor_reverse_lanes()[0].index = -1;
  }
  // Connected
  ::google::protobuf::RepeatedPtrField< ::apollo::hdmap::Id >::const_iterator
      lane_id_it = lane.predecessor_id().begin();
  for (; lane_id_it != lane.predecessor_id().end(); ++lane_id_it) {
    Lane::TopologyId* topology_id = lane_info->predecessor_lanes().Allocate();
    if (Nullptr_t == topology_id) {
      LOG_WARN << "Can't save predecessor id anymore, storage is full.";
      break;
    }
    topology_id->id.id = lane_id_it->id();
    topology_id->index = -1;
  }
  lane_id_it = lane.successor_id().begin();
  for (; lane_id_it != lane.successor_id().end(); ++lane_id_it) {
    Lane::TopologyId* topology_id = lane_info->successor_lanes().Allocate();
    if (Nullptr_t == topology_id) {
      LOG_WARN << "Can't save successor id anymore, storage is full.";
      break;
    }
    topology_id->id.id = lane_id_it->id();
    topology_id->index = -1;
  }
}

#endif // #if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO


}  // namespace driv_map
}  // namespace phoenix
