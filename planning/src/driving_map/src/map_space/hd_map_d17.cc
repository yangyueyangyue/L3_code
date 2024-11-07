/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       hd_map_d17.cc
 * @brief      将接收到的外部地图数据解析为内部地图数据格式。
 * @details    定义了内部地图的数据格式及相关访问接口
 *
 * @author     pengc
 * @date       2020.06.06
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/07/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "map_space/hd_map.h"

#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "utils/linear_interpolation.h"
#include "pos_filter_wrapper.h"

#define ENABLE_HD_MAP_D17_TRACE (0)
#define ENABLE_HD_MAP_D17_PERFORMANCE_TEST (0)


namespace phoenix {
namespace driv_map {


#if HD_MAP_TYPE == HD_MAP_TYPE_D17

bool HDMap::ConstructMapInfo(const exp_map_t::stMapExpProfileInfo& map) {
#if ENABLE_HD_MAP_D17_TRACE
  std::cout << "######### Construct HDMap >>>>>>>>>" << std::endl;
#endif

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of constructing hd-map (Start)
  phoenix::common::Stopwatch performance_timer_constructing_hd_map;
#endif

  // Clear old information set before.
  Clear();

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of parsing lanes in map (Start)
  phoenix::common::Stopwatch performance_timer_parse_lanes;
#endif

  // Parse all lanes in the map.
#if 1
  std::vector<decision_map_t::sDecisionMapLaneItem>::const_iterator
      lane_it = map.lane_item_info.begin();
  for (; lane_it != map.lane_item_info.end(); ++lane_it) {
    if (!ParseLaneInfo(*lane_it)) {
#if ENABLE_HD_MAP_D17_TRACE
      LOG_ERR << "Failed to parse lane [" << lane_it->s_current_lane_id64 << "].";
#endif
      continue;
    }
  }
#endif

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of parsing lanes in map (End)
  std::cout << "Parsing lanes in map spend "
            << performance_timer_parse_lanes.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of updating lanes topology (Start)
  phoenix::common::Stopwatch performance_timer_update_lane_topology;
#endif

  UpdateLanesTopology();

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of updating lanes topology (End)
  std::cout << "Updating lanes topology spend "
            << performance_timer_update_lane_topology.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of building lane segment kd-tree (Start)
  phoenix::common::Stopwatch performance_timer_build_lane_kd_tree;
#endif

  BuildLaneSegmentKDTree();

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of building lane segment kd-tree (End)
  std::cout << "Building lane segment kd-tree spend "
            << performance_timer_build_lane_kd_tree.Elapsed()
            << "ms." << std::endl;
#endif

  /// TODO: Parse traffic ligth information in HD-Map

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of constructing hd-map (End)
  std::cout << "Constructing hd-map spend "
            << performance_timer_constructing_hd_map.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_D17_TRACE
  std::cout << "<<<<<<<<< Construct HDMap #########" << std::endl;
#endif

  // LOG_INFO(5) << "After construct map D17, the map is:";
  // PrintHDMap();

  return (true);
}

bool HDMap::ConstructRoutingInfo(const Uint8_t& routing) {
#if ENABLE_HD_MAP_D17_TRACE
  std::cout << "######### Construct Routing >>>>>>>>>" << std::endl;
#endif

  /// TODO: parse routings
#if 0

  if (lanes_in_boundary_.Empty()) {
    LOG_ERR << "The lanes table in map is empty.";
    return false;
  }

  ID lane_id;
  Int32_t index = -1;
  Float64_t start_s = 0;
  Float64_t end_s = 0;

#if ENABLE_HD_MAP_D17_TRACE
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

#if ENABLE_HD_MAP_D17_TRACE
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
#endif

#if ENABLE_HD_MAP_D17_TRACE
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

#if ENABLE_HD_MAP_D17_TRACE
  std::cout << "<<<<<<<<< Construct Routing #########" << std::endl;
#endif

  return true;
}

bool HDMap::ParseLaneInfo(const ::decision_map_t::sDecisionMapLaneItem& lane) {
#if ENABLE_HD_MAP_D17_TRACE
  std::cout << "######### ParseLaneInfo ("
            << lane.s_current_lane_id64
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

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of taking path in boundary (Start)
  phoenix::common::Stopwatch performance_timer_take_path;
#endif

  temp_data_.take_path.Clear();

  std::vector<Coordinate>::const_iterator point_it =
      lane.s_LaneCenterGeometry.m_points.begin();
  std::vector<Coordinate>::const_iterator point_it_end =
      lane.s_LaneCenterGeometry.m_points.end();
  for (; point_it != point_it_end; ++point_it) {
    // std::cout << "pt=(" << point_it->m_x <<", " << point_it->m_y << std::endl;

    if (loop_count > 0) {
      // 计算两点间的距离
      distance = common::com_sqrt(
            common::Square(point_it->m_x - prev_global_point.x) +
            common::Square(point_it->m_y - prev_global_point.y));

      // std::cout << "distance=" << distance << std::endl;

      // 过滤掉距离过近的点
      if (distance < min_lane_point_interval_) {
        continue;
      }
      // 计算原始地图车道中心线处每个点的轨迹长
      accumulated_s += distance;
    }

    // current point
    curr_global_point.x = point_it->m_x;
    curr_global_point.y = point_it->m_y;
    curr_global_point.s = accumulated_s;

    // 判断该点是否位于包围盒内
    bool is_in_boundary =
        IsPointInMapBoundingBox(point_it->m_x, point_it->m_y);

    //printf("loop_count=%d, pos=(%f,%f), is_in_boundary=%d\n",
    //		loop_count, point_it->x(), point_it->y(), is_in_boundary);

    // Take points in boundary
    TakePointsInBoundary(is_in_boundary, prev_global_point_valid,
                         prev_global_point, curr_global_point);

    // 保存这个点，下个迭代会使用
    prev_global_point_valid = true;
    prev_global_point.x = point_it->m_x;
    prev_global_point.y = point_it->m_y;
    prev_global_point.s = accumulated_s;
    loop_count++;
  }

  // Select better segment
  Int32_t better_seg = 0;
  if (false == SelectBetterLaneSeg(&better_seg)) {
#if ENABLE_HD_MAP_D17_TRACE
    LOG_ERR << "There are not enough points in this lane, global("
            << current_global_position_.x << "," << current_global_position_.y << ").";
#endif
    lane_info->Clear();
    lane_table_storage_.PopBack();
    return false;
  }

  LimitLaneRang(better_seg);

  if (temp_data_.take_path.lane_seg[better_seg].hold_points.Size() < 2) {
#if ENABLE_HD_MAP_D17_TRACE
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

#if ENABLE_HD_MAP_D17_TRACE
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

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
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

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
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

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of constructing path of lane (End)
  std::cout << "Constructing path spend "
            << performance_timer_construct_path.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of parsing other lane information (Start)
  phoenix::common::Stopwatch performance_timer_parsing_other;
#endif

  // Lane Boundary
  ParseLaneBoundary(lane, start_s, end_s, lane_info);

  // Slope
  ParseLaneSlope(lane, start_s, end_s, lane_info);

  // Speed limit
  if (!lane.s_LaneAttribute.s_vLaneLimitSpeedMax.empty()) {
    lane_info->set_speed_limit_high(
          lane.s_LaneAttribute.s_vLaneLimitSpeedMax.front().vehicle_limit_speed/3.6F);
  } else {
    lane_info->set_speed_limit_high(-1.0F);
  }
  if (!lane.s_LaneAttribute.s_vLaneLimitSpeedMin.empty()) {
    lane_info->set_speed_limit_low(
          lane.s_LaneAttribute.s_vLaneLimitSpeedMin.front().vehicle_limit_speed/3.6F);
  } else {
    lane_info->set_speed_limit_low(-1.0F);
  }

  // Topology
  ParseLaneTopology(lane, lane_info);

  // Lane ID
  lane_info->topology_id().id.id = lane.s_current_lane_id64;
  lane_info->topology_id().index = lane_storage_index;

  // Lane Type: TBD
  // lane_info->set_lane_type();
  // lane_info->set_turn_type();
  // lane_info->set_direction();

  /// TODO: Add Speed limit
  // lane_info->set_speed_limit();

  // Add to lane table
  const Int32_t* lane_index = lane_table_.Find(lane_info->topology_id().id.id);
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

  lane_table_.Insert(lane_info->topology_id().id.id, lane_storage_index);

#if ENABLE_HD_MAP_D17_PERFORMANCE_TEST
  /// Performance of parsing other lane information (End)
  std::cout << "Parsing other lane info spend "
            << performance_timer_parsing_other.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_D17_TRACE
  std::cout << "<<<<<<<<< ParseLaneInfo #########" << std::endl;
#endif

  return (true);
}

#if 0
void HDMap::ParseLaneBoundary(
    const ::decision_map_t::sDecisionMapLaneItem& lane,
    Float64_t start_s, Float64_t end_s, Lane* lane_info) {
  /* k003 longjiaoy 2022-12-08 (start) */
  // left Lane boundary type
  bool valid_boundary_type = false;
  Int32_t lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
  using LineControlPointsType = LineControlPoints<LineMarking, LineMarking::Unknown>;
  std::vector<LineControlPointsType::ControlPointType>::const_iterator lane_type_sample_it;
  std::vector<LineControlPointsType::ControlPointType>::const_iterator lane_type_sample_it_start;
  std::vector<LineControlPointsType::ControlPointType>::const_iterator lane_type_sample_it_end;
  if (lane.s_vLaneLineLeftId64.size() > 0) {
    std::map<LineId, LineControlPointsType>::const_iterator it =
        lane.s_LinearAttribute.s_mLaneLeftLineMarking.find(lane.s_vLaneLineLeftId64[0]);
    if (it != lane.s_LinearAttribute.s_mLaneLeftLineMarking.end()) {
      const LineControlPointsType& lane_type_sample = it->second;
      lane_type_sample_it_start = lane_type_sample.m_controlPoints.cbegin();
      lane_type_sample_it_end = lane_type_sample.m_controlPoints.cend();
      lane_type_sample_it = lane_type_sample_it_start;
      if (lane_type_sample.m_controlPoints.size() > 0) {
        valid_boundary_type = true;
        lane_boundary_type = ConvLeftLaneBoundaryType(lane_type_sample_it->m_value);
      }
    }
  }
  /* k003 longjiaoy 2022-12-08 (end) */
  // left boundary
  std::vector<OffsetFloatEntry>::const_iterator lane_width_sample_it =
      lane.s_LaneAttribute.s_vLaneWidth.begin();
  std::vector<OffsetFloatEntry>::const_iterator lane_width_sample_it_end =
      lane.s_LaneAttribute.s_vLaneWidth.end();
  for (; lane_width_sample_it != lane_width_sample_it_end;
       ++lane_width_sample_it) {
    Float64_t width_sample_s =
        (lane_width_sample_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;
    if ((start_s <= width_sample_s) && (width_sample_s <= end_s)) {
      /* k003 longjiaoy 2022-12-08 (start) */
      if (valid_boundary_type) {
        for (;lane_type_sample_it != lane_type_sample_it_end;
             ++lane_type_sample_it) {
          Float32_t type_sample_s = lane_type_sample_it->m_dist * 0.001F;
          if (width_sample_s <= type_sample_s) {
            break;
          }
        }
        if (lane_type_sample_it != lane_type_sample_it_start) {
          lane_boundary_type = ConvLeftLaneBoundaryType((lane_type_sample_it - 1)->m_value);
        }
      }
      /* k003 longjiaoy 2022-12-08 (end) */
      if (lane_info->left_boundary().boundary_samples.Empty()) {
        Lane::BoundaryAssociation* lane_boundary =
            lane_info->left_boundary().boundary_samples.Allocate();
        if (Nullptr_t == lane_boundary) {
          LOG_WARN << "Can't save boundary information anymore, "
                      "storage is full.";
          break;
        }
        lane_boundary->s = 0;
        lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
        /* k003 longjiaoy 2022-12-08 (start) */
        lane_boundary->type = lane_boundary_type;
        /* k003 longjiaoy 2022-12-08 (end) */
      }

      // 过滤掉距离过近的点
      if ((width_sample_s - start_s -
           lane_info->left_boundary().boundary_samples.Back().s) <
          min_lane_point_interval_) {
        continue;
      }

      Lane::BoundaryAssociation* lane_boundary =
          lane_info->left_boundary().boundary_samples.Allocate();
      if (Nullptr_t == lane_boundary) {

        LOG_WARN << "Can't save boundary information anymore, storage is full.";
        break;
      }
      lane_boundary->s = width_sample_s - start_s;
      lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
      /* k003 longjiaoy 2022-12-08 (start) */
      lane_boundary->type = lane_boundary_type;
      /* k003 longjiaoy 2022-12-08 (end) */
    }
  }

  /* k003 longjiaoy 2022-12-08 (start) */
  // left Lane boundary type
  valid_boundary_type = false;
  lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
  if (lane.s_vLaneLineRightId64.size() > 0) {
    std::map<LineId, LineControlPointsType>::const_iterator it =
        lane.s_LinearAttribute.s_mLaneRightLineMarking.find(lane.s_vLaneLineRightId64[0]);
    if (it != lane.s_LinearAttribute.s_mLaneRightLineMarking.end()) {
      const LineControlPointsType& lane_type_sample = it->second;
      lane_type_sample_it_start = lane_type_sample.m_controlPoints.cbegin();
      lane_type_sample_it = lane_type_sample_it_start;
      lane_type_sample_it_end = lane_type_sample.m_controlPoints.cend();
      if (lane_type_sample.m_controlPoints.size() > 0) {
        valid_boundary_type = true;
        lane_boundary_type = ConvRightLaneBoundaryType(lane_type_sample_it->m_value);
      }
    }
  }
  /* k003 longjiaoy 2022-12-08 (end) */
  // right boundary
  lane_width_sample_it = lane.s_LaneAttribute.s_vLaneWidth.begin();
  lane_width_sample_it_end = lane.s_LaneAttribute.s_vLaneWidth.end();
  for (; lane_width_sample_it != lane_width_sample_it_end;
       ++lane_width_sample_it) {
    Float64_t width_sample_s =
        (lane_width_sample_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;

    if ((start_s <= width_sample_s) && (width_sample_s <= end_s)) {
      /* k003 longjiaoy 2022-12-08 (start) */
      if (valid_boundary_type) {
        for (;lane_type_sample_it != lane_type_sample_it_end;
             ++lane_type_sample_it) {
          Float32_t type_sample_s = lane_type_sample_it->m_dist * 0.001F;
          if (width_sample_s <= type_sample_s) {
            break;
          }
        }
        if (lane_type_sample_it != lane_type_sample_it_start) {
          lane_boundary_type = ConvRightLaneBoundaryType((lane_type_sample_it - 1)->m_value);
        }
      }
      /* k003 longjiaoy 2022-12-08 (end) */
      if (lane_info->right_boundary().boundary_samples.Empty()) {
        Lane::BoundaryAssociation* lane_boundary =
            lane_info->right_boundary().boundary_samples.Allocate();
        if (Nullptr_t == lane_boundary) {
          LOG_WARN << "Can't save boundary information anymore, "
                      "storage is full.";
          break;
        }
        lane_boundary->s = 0;
        lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
        /* k003 longjiaoy 2022-12-08 (start) */
        lane_boundary->type = lane_boundary_type;
        /* k003 longjiaoy 2022-12-08 (end) */
      }

      // 过滤掉距离过近的点
      if ((width_sample_s - start_s -
           lane_info->right_boundary().boundary_samples.Back().s) <
          min_lane_point_interval_) {
        continue;
      }

      Lane::BoundaryAssociation* lane_boundary =
          lane_info->right_boundary().boundary_samples.Allocate();
      if (Nullptr_t == lane_boundary) {

        LOG_WARN << "Can't save boundary information anymore, storage is full.";
        break;
      }
      lane_boundary->s = width_sample_s - start_s;
      lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
      /* k003 longjiaoy 2022-12-08 (start) */
      lane_boundary->type = lane_boundary_type;
      /* k003 longjiaoy 2022-12-08 (end) */
    }
  }
}
#else
void HDMap::ParseLaneBoundary(
    const ::decision_map_t::sDecisionMapLaneItem& lane,
    Float64_t start_s, Float64_t end_s, Lane* lane_info) {
  /// Lane boundary width
  // left boundary
  std::vector<OffsetFloatEntry>::const_iterator lane_width_sample_it =
      lane.s_LaneAttribute.s_vLaneWidth.begin();
  std::vector<OffsetFloatEntry>::const_iterator lane_width_sample_it_end =
      lane.s_LaneAttribute.s_vLaneWidth.end();
  for (; lane_width_sample_it != lane_width_sample_it_end;
       ++lane_width_sample_it) {
    Float64_t width_sample_s =
        (lane_width_sample_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;
    if ((start_s <= width_sample_s) && (width_sample_s <= end_s)) {
      if (lane_info->left_boundary().boundary_width_samples.Empty()) {
        Lane::BoundaryWidthAssociation* lane_boundary =
            lane_info->left_boundary().boundary_width_samples.Allocate();
        if (Nullptr_t == lane_boundary) {
          LOG_WARN << "Can't save boundary information anymore, "
                      "storage is full.";
          break;
        }
        lane_boundary->s = 0;
        lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
      }

      // 过滤掉距离过近的点
      if ((width_sample_s - start_s -
           lane_info->left_boundary().boundary_width_samples.Back().s) <
          min_lane_point_interval_) {
        continue;
      }

      Lane::BoundaryWidthAssociation* lane_boundary =
          lane_info->left_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t == lane_boundary) {
        LOG_WARN << "Can't save boundary information anymore, storage is full.";
        break;
      }
      lane_boundary->s = width_sample_s - start_s;
      lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
    }
  }
  if (lane_info->left_boundary().boundary_width_samples.Empty() &&
      !lane.s_LaneAttribute.s_vLaneWidth.empty()) {
    Lane::BoundaryWidthAssociation* lane_boundary =
        lane_info->left_boundary().boundary_width_samples.Allocate();
    if (Nullptr_t == lane_boundary) {
      LOG_WARN << "Can't save boundary information anymore, storage is full.";
    } else {
      lane_boundary->s = 0.0F;
      lane_boundary->width = lane.s_LaneAttribute.s_vLaneWidth.front().m_value * 0.01F * 0.5F;
    }
  }

  // right boundary
  lane_width_sample_it = lane.s_LaneAttribute.s_vLaneWidth.begin();
  lane_width_sample_it_end = lane.s_LaneAttribute.s_vLaneWidth.end();
  for (; lane_width_sample_it != lane_width_sample_it_end;
       ++lane_width_sample_it) {
    Float64_t width_sample_s =
        (lane_width_sample_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;

    if ((start_s <= width_sample_s) && (width_sample_s <= end_s)) {
      if (lane_info->right_boundary().boundary_width_samples.Empty()) {
        Lane::BoundaryWidthAssociation* lane_boundary =
            lane_info->right_boundary().boundary_width_samples.Allocate();
        if (Nullptr_t == lane_boundary) {
          LOG_WARN << "Can't save boundary information anymore, "
                      "storage is full.";
          break;
        }
        lane_boundary->s = 0;
        lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
      }

      // 过滤掉距离过近的点
      if ((width_sample_s - start_s -
           lane_info->right_boundary().boundary_width_samples.Back().s) <
          min_lane_point_interval_) {
        continue;
      }

      Lane::BoundaryWidthAssociation* lane_boundary =
          lane_info->right_boundary().boundary_width_samples.Allocate();
      if (Nullptr_t == lane_boundary) {
        LOG_WARN << "Can't save boundary information anymore, storage is full.";
        break;
      }
      lane_boundary->s = width_sample_s - start_s;
      lane_boundary->width = lane_width_sample_it->m_value * 0.01F * 0.5F;
    }
  }
  if (lane_info->right_boundary().boundary_width_samples.Empty() &&
      !lane.s_LaneAttribute.s_vLaneWidth.empty()) {
    Lane::BoundaryWidthAssociation* lane_boundary =
        lane_info->right_boundary().boundary_width_samples.Allocate();
    if (Nullptr_t == lane_boundary) {
      LOG_WARN << "Can't save boundary information anymore, storage is full.";
    } else {
      lane_boundary->s = 0.0F;
      lane_boundary->width = lane.s_LaneAttribute.s_vLaneWidth.front().m_value * 0.01F * 0.5F;
    }
  }

  /// Lane boundary type
  // Left boundary
  using LineControlPointsType = LineControlPoints<LineMarking, LineMarking::Unknown>;
  std::vector<LineControlPointsType::ControlPointType>::const_iterator lane_type_sample_it;
  std::vector<LineControlPointsType::ControlPointType>::const_iterator lane_type_sample_it_start;
  std::vector<LineControlPointsType::ControlPointType>::const_iterator lane_type_sample_it_end;
  if (lane.s_vLaneLineLeftId64.size() > 0) {
    std::map<LineId, LineControlPointsType>::const_iterator it =
        lane.s_LinearAttribute.s_mLaneLeftLineMarking.find(lane.s_vLaneLineLeftId64[0]);
    if (it != lane.s_LinearAttribute.s_mLaneLeftLineMarking.end()) {
      const LineControlPointsType& lane_type_sample = it->second;
      lane_type_sample_it_start = lane_type_sample.m_controlPoints.cbegin();
      lane_type_sample_it_end = lane_type_sample.m_controlPoints.cend();
      lane_type_sample_it = lane_type_sample_it_start;

      for (;lane_type_sample_it != lane_type_sample_it_end;
           ++lane_type_sample_it) {
        Float32_t type_sample_s = lane_type_sample_it->m_dist * 0.001F;

        if ((start_s <= type_sample_s) && (type_sample_s <= end_s)) {
          if (lane_info->left_boundary().boundary_type_samples.Empty()) {
            Lane::BoundaryTypeAssociation* lane_boundary =
                lane_info->left_boundary().boundary_type_samples.Allocate();
            if (Nullptr_t == lane_boundary) {
              LOG_WARN << "Can't save boundary information anymore, "
                          "storage is full.";
              break;
            }
            lane_boundary->s = 0;
            lane_boundary->type = ConvLeftLaneBoundaryType(lane_type_sample_it->m_value);
          }

          // 过滤掉距离过近的点
          if ((type_sample_s - start_s -
               lane_info->left_boundary().boundary_type_samples.Back().s) <
              min_lane_point_interval_) {
            continue;
          }

          Lane::BoundaryTypeAssociation* lane_boundary =
              lane_info->left_boundary().boundary_type_samples.Allocate();
          if (Nullptr_t == lane_boundary) {
            LOG_WARN << "Can't save boundary information anymore, storage is full.";
            break;
          }
          lane_boundary->s = type_sample_s - start_s;
          lane_boundary->type = ConvLeftLaneBoundaryType(lane_type_sample_it->m_value);
        }
      }
      if (lane_info->left_boundary().boundary_type_samples.Empty() &&
          !lane_type_sample.m_controlPoints.empty()) {
        Lane::BoundaryTypeAssociation* lane_boundary =
            lane_info->left_boundary().boundary_type_samples.Allocate();
        if (Nullptr_t == lane_boundary) {
          LOG_WARN << "Can't save boundary information anymore, storage is full.";
        } else {
          lane_boundary->s = 0.0F;
          lane_boundary->type = ConvLeftLaneBoundaryType(
                lane_type_sample.m_controlPoints.front().m_value);
        }
      }
    }
  }

  // Rigth boundary
  if (lane.s_vLaneLineRightId64.size() > 0) {
    std::map<LineId, LineControlPointsType>::const_iterator it =
        lane.s_LinearAttribute.s_mLaneRightLineMarking.find(lane.s_vLaneLineRightId64[0]);
    if (it != lane.s_LinearAttribute.s_mLaneRightLineMarking.end()) {
      const LineControlPointsType& lane_type_sample = it->second;
      lane_type_sample_it_start = lane_type_sample.m_controlPoints.cbegin();
      lane_type_sample_it = lane_type_sample_it_start;
      lane_type_sample_it_end = lane_type_sample.m_controlPoints.cend();

      for (;lane_type_sample_it != lane_type_sample_it_end;
           ++lane_type_sample_it) {
        Float32_t type_sample_s = lane_type_sample_it->m_dist * 0.001F;

        if ((start_s <= type_sample_s) && (type_sample_s <= end_s)) {
          if (lane_info->right_boundary().boundary_type_samples.Empty()) {
            Lane::BoundaryTypeAssociation* lane_boundary =
                lane_info->right_boundary().boundary_type_samples.Allocate();
            if (Nullptr_t == lane_boundary) {
              LOG_WARN << "Can't save boundary information anymore, "
                          "storage is full.";
              break;
            }
            lane_boundary->s = 0;
            lane_boundary->type = ConvRightLaneBoundaryType(lane_type_sample_it->m_value);
          }

          // 过滤掉距离过近的点
          if ((type_sample_s - start_s -
               lane_info->right_boundary().boundary_type_samples.Back().s) <
              min_lane_point_interval_) {
            continue;
          }

          Lane::BoundaryTypeAssociation* lane_boundary =
              lane_info->right_boundary().boundary_type_samples.Allocate();
          if (Nullptr_t == lane_boundary) {
            LOG_WARN << "Can't save boundary information anymore, storage is full.";
            break;
          }
          lane_boundary->s = type_sample_s - start_s;
          lane_boundary->type = ConvRightLaneBoundaryType(lane_type_sample_it->m_value);
        }
      }
      if (lane_info->right_boundary().boundary_type_samples.Empty() &&
          !lane_type_sample.m_controlPoints.empty()) {
        Lane::BoundaryTypeAssociation* lane_boundary =
            lane_info->right_boundary().boundary_type_samples.Allocate();
        if (Nullptr_t == lane_boundary) {
          LOG_WARN << "Can't save boundary information anymore, storage is full.";
        } else {
          lane_boundary->s = 0.0F;
          lane_boundary->type = ConvRightLaneBoundaryType(
                lane_type_sample.m_controlPoints.front().m_value);
        }
      }
    }
  }
}
#endif

/* k003 longjiaoy 2022-12-08 (start) */
Int32_t HDMap::ConvLeftLaneBoundaryType(const LineMarking& map_lane_mark) {
  Int32_t ret = driv_map::LANE_BOUNDARY_TYPE_UNKNOWN;
  switch (map_lane_mark) {
  case (LineMarking::SolidLine):
    ret = driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  case (LineMarking::DashedLine):
    ret = driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (LineMarking::DoubleSolidLine):
    ret  = driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  case (LineMarking::DoubleDashedLine):
    ret  = driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (LineMarking::LeftSolidRightDashed):
    ret = driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (LineMarking::RightSolidLeftDashed):
    ret = driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  default:
    ret = driv_map::LANE_BOUNDARY_TYPE_UNKNOWN;
    break;
  }
  return (ret);
}

Int32_t HDMap::ConvRightLaneBoundaryType(const LineMarking& map_lane_mark) {
  Int32_t ret = driv_map::LANE_BOUNDARY_TYPE_UNKNOWN;
  switch (map_lane_mark) {
  case (LineMarking::SolidLine):
    ret = driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  case (LineMarking::DashedLine):
    ret = driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (LineMarking::DoubleSolidLine):
    ret  = driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  case (LineMarking::DoubleDashedLine):
    ret  = driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  case (LineMarking::LeftSolidRightDashed):
    ret = driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE;
    break;
  case (LineMarking::RightSolidLeftDashed):
    ret = driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE;
    break;
  default:
    ret = driv_map::LANE_BOUNDARY_TYPE_UNKNOWN;
    break;
  }
  return (ret);
}
/* k003 longjiaoy 2022-12-08 (end) */

static map_var_t ParseSlope(Int32_t value) {
  map_var_t slope = 0.0F;

  if (-126 == value) {
    slope = -30.0F;
  } else if (-125 == value) {
    slope = -25.0F;
  } else if ((-124 <= value) && (value <= 124)) {
    slope = value * 0.2F;
  } else if (125 == value) {
    slope = 25.0F;
  } else if (126 == value) {
    slope = 30.0F;
  } else {
    slope = 0.0F;
    LOG_ERR << "Invalid slope value.";
  }

  return (slope);
}

void HDMap::ParseLaneSlope(
    const ::decision_map_t::sDecisionMapLaneItem& lane,
    Float64_t start_s, Float64_t end_s, Lane* lane_info) {
  std::vector<OffsetFloatEntry>::const_iterator sample_it =
      lane.s_LaneAttribute.s_vLaneSlope.begin();
  std::vector<OffsetFloatEntry>::const_iterator sample_it_begin = sample_it;
  std::vector<OffsetFloatEntry>::const_iterator sample_it_end =
      lane.s_LaneAttribute.s_vLaneSlope.end();

  for (; sample_it != sample_it_end; ++sample_it) {
    Float64_t slope_s = (sample_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;
    map_var_t slope = ParseSlope(common::com_rintf(sample_it->m_value));

    if (lane_info->slope_samples().Empty()) {
      if ((slope_s-common::kMathEpsilonF) < start_s) {
        std::vector<OffsetFloatEntry>::const_iterator next_it = sample_it+1;
        if (next_it != sample_it_end) {
          Float64_t next_slope_s = (next_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;
          if (next_slope_s-common::kMathEpsilonF < start_s) {
            continue;
          }
          Lane::SlopeAssociation* lane_slope = lane_info->slope_samples().Allocate();
          if (Nullptr_t == lane_slope) {
            LOG_WARN << "Can't save slope information anymore, storage is full.";
            break;
          }
          lane_slope->s = 0;
          map_var_t next_slope = ParseSlope(common::com_rintf(next_it->m_value));
          if (((next_slope_s - slope_s) > 1.0F) && (start_s -common::kMathEpsilonF < next_slope_s)) {
            lane_slope->slope = common::Lerp(
                  slope, next_slope,
                  common::com_abs(start_s-slope_s) / (next_slope_s - slope_s));
          } else if (start_s -common::kMathEpsilonF > next_slope_s){
            lane_slope->slope = next_slope;
          } else {
            lane_slope->slope = slope;
          }
        } else {
          Lane::SlopeAssociation* lane_slope = lane_info->slope_samples().Allocate();
          if (Nullptr_t == lane_slope) {
            LOG_WARN << "Can't save slope information anymore, storage is full.";
            break;
          }
          lane_slope->s = 0;
          lane_slope->slope = slope;
        }
      } else if ((end_s-common::kMathEpsilonF) < slope_s) {

        Lane::SlopeAssociation* lane_slope = lane_info->slope_samples().Allocate();
        if (Nullptr_t == lane_slope) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
          break;
        }
        lane_slope->s = 0;
        lane_slope->slope = slope;
      }
    } else {
      if (((start_s+0.5F) < slope_s) && (slope_s < (end_s+common::kMathEpsilonF))) {
        Lane::SlopeAssociation* lane_slope = lane_info->slope_samples().Allocate();
        if (Nullptr_t == lane_slope) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
          break;
        }
        lane_slope->s = slope_s - start_s;
        lane_slope->slope = slope;
      } else {
        if (end_s < (slope_s+common::kMathEpsilonF)) {
          Lane::SlopeAssociation* lane_slope =
              lane_info->slope_samples().Allocate();
          if (Nullptr_t == lane_slope) {
            LOG_WARN << "Can't save slope information anymore, storage is full.";
            break;
          }
          lane_slope->s = end_s - start_s;
          if (sample_it != sample_it_begin) {
            std::vector<OffsetFloatEntry>::const_iterator prev_it = sample_it-1;
            Float64_t prev_slope_s = (prev_it->m_offset - lane.s_LaneOffsetS32) * 0.01F;
            map_var_t prev_slope = ParseSlope(common::com_rintf(prev_it->m_value));
            if (((slope_s - prev_slope_s) > 1.0F) && (prev_slope_s-common::kMathEpsilonF < end_s)) {
              lane_slope->slope = common::Lerp(
                    prev_slope, slope,
                    common::com_abs(end_s-prev_slope_s) / (slope_s - prev_slope_s));
            } else if (prev_slope_s-common::kMathEpsilonF > end_s){
              lane_slope->slope = prev_slope;
            }
          } else {
            lane_slope->slope = slope;
          }
          break;
        }
      }
    }
  }
}

void HDMap::ParseLaneTopology(
    const ::decision_map_t::sDecisionMapLaneItem& lane, Lane* lane_info) {
  // Neighbors
  if (0 != lane.s_left_lane_id64) {
    lane_info->left_neighbor_forward_lanes().Resize(1);
    lane_info->left_neighbor_forward_lanes()[0].id.id = lane.s_left_lane_id64;
    lane_info->left_neighbor_forward_lanes()[0].index = -1;
  }
  if (0 != lane.s_right_lane_id64) {
    lane_info->right_neighbor_forward_lanes().Resize(1);
    lane_info->right_neighbor_forward_lanes()[0].id.id = lane.s_right_lane_id64;
    lane_info->right_neighbor_forward_lanes()[0].index = -1;
  }

  // No reverse neighbor lane

  // Connected
  std::vector<LineId>::const_iterator lane_id_it = lane.s_vLanePreId64.begin();
  for (; lane_id_it != lane.s_vLanePreId64.end(); ++lane_id_it) {
    Lane::TopologyId* topology_id = lane_info->predecessor_lanes().Allocate();
    if (Nullptr_t == topology_id) {
      LOG_WARN << "Can't save predecessor id anymore, storage is full.";
      break;
    }
    topology_id->id.id = *lane_id_it;
    topology_id->index = -1;
  }
  lane_id_it = lane.s_vLaneNextId64.begin();
  for (; lane_id_it != lane.s_vLaneNextId64.end(); ++lane_id_it) {
    Lane::TopologyId* topology_id = lane_info->successor_lanes().Allocate();
    if (Nullptr_t == topology_id) {
      LOG_WARN << "Can't save successor id anymore, storage is full.";
      break;
    }
    topology_id->id.id = *lane_id_it;
    topology_id->index = -1;
  }
}


#endif // #if HD_MAP_TYPE == HD_MAP_TYPE_D17


}  // namespace driv_map
}  // namespace phoenix
