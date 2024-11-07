/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       hd_map_series_d600.cc
 * @brief      当外部地图格式为D600的地图格式时，将接收到的\n
 *             外部地图数据解析为内部地图数据格式。
 * @details    定义了内部地图的数据格式及相关访问接口
 *
 * @author     boc
 * @date       2020.06.30
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/30  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "map_space/hd_map.h"

#include "math/matrix.h"
#include "math/math_utils.h"
#include "geometry/geometry_utils.h"
#include "utils/linear_interpolation.h"

#define ENABLE_HD_MAP_TRACE_D600 (0)
#define ENABLE_HD_MAP_PERFORMANCE_TEST_D600 (0)

#if (ENABLE_WGS84TOUTM == 1)
#include "utils/gps_tools.h"
#endif


namespace phoenix {
namespace driv_map {


#if HD_MAP_TYPE == HD_MAP_TYPE_D600

bool HDMap::ConstructMapInfo(const MAP::MAP_VecLaneInfo& map_in,
                             const MAP::MAP_Location& map_location_in) {
#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "######### Construct HDMap >>>>>>>>>" << std::endl;
#endif

  /// TODO: Correct the function for converting coordinate !!!

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of constructing hd-map (Start)
  phoenix::common::Stopwatch performance_timer_constructing_hd_map;
#endif

  // Clear old information set before.
  Clear();

  // Set bounding box around the current postion.
  map_bounding_box_.min_x = position_.global_x - map_range_;
  map_bounding_box_.min_y = position_.global_y - map_range_;
  map_bounding_box_.max_x = position_.global_x + map_range_;
  map_bounding_box_.max_y = position_.global_y + map_range_;

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of parsing lanes in map (Start)
  phoenix::common::Stopwatch performance_timer_parse_lanes;
#endif

  // 解析所有车道信息
  for (size_t i = 0; i < map_in.data.size(); ++i) {
    const MAP::MAP_LaneInfo& lane_tmp = map_in.data[i];
    if (!ParseLaneInfo(lane_tmp, map_location_in)) {
      LOG_ERR << "Failed to parse lane [" <<
                 static_cast<Uint64_t>(lane_tmp.base_lane_id) << "].";
      continue;
    }
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of parsing lanes in map (End)
  std::cout << "Parsing lanes in map spend "
            << performance_timer_parse_lanes.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of updating lanes topology (Start)
  phoenix::common::Stopwatch performance_timer_update_lane_topology;
#endif

  UpdateLanesTopology();

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of updating lanes topology (End)
  std::cout << "Updating lanes topology spend "
            << performance_timer_update_lane_topology.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of building lane segment kd-tree (Start)
  phoenix::common::Stopwatch performance_timer_build_lane_kd_tree;
#endif

  BuildLaneSegmentKDTree();

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of building lane segment kd-tree (End)
  std::cout << "Building lane segment kd-tree spend "
            << performance_timer_build_lane_kd_tree.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of constructing hd-map (End)
  std::cout << "Constructing hd-map spend "
            << performance_timer_constructing_hd_map.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "<<<<<<<<< Construct HDMap #########" << std::endl;
#endif

  return (true);
}

bool HDMap::ConstructRoutingInfo(const MAP::MAP_Routing& routing_in) {
#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "######### Construct Routing >>>>>>>>>" << std::endl;
#endif

  if (lanes_in_boundary_.Empty()) {
    LOG_ERR << "The lanes table in map is empty.";
    return false;
  }

  ID lane_id;
  Int32_t index = -1;
  Float64_t start_s = 0;
  Float64_t end_s = 0;

#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "The original routing segments are :" << std::endl;
#endif

  Int32_t loop_count = 0;
  std::vector<MAP::Lane_ConnectivityPair>::const_iterator road_it =
      routing_in.lane_connect.begin();
  const std::vector<MAP::Lane_ConnectivityPair>::const_iterator road_it_end =
      routing_in.lane_connect.end();
  for (; road_it != road_it_end; ++road_it) {
    const MAP::Lane_ConnectivityPair& segment_it = *road_it;
    // Segments
    loop_count++;
    const uint32_t path_number = segment_it.InitialPath;
    if (path_number != 8) {
      // 暂不解析次导航路径的数据
      continue;
    }
    // int32_t lane_number = segment_it.InitialLaneNumber;
    lane_id.id = segment_it.InitialLaneNumber;
    index = FindLaneFromLanesInBoundary(lane_id);
    if (index >= 0) {
      start_s = 0.0;
      end_s = lanes_in_boundary_[index].end_s -
          lanes_in_boundary_[index].start_s;
      if (end_s - start_s > 0.1) {
        RoutingSection* const section = routing_.routing_sections().Allocate();
        if (Nullptr_t == section) {
          LOG_ERR << "Can't store routing segment any more, "
                     "storage is full.";
          return false;
        }
        RoutingSegment* const seg = section->neighbors.Allocate();
        if (Nullptr_t == seg) {
          routing_.routing_sections().PopBack();
          LOG_ERR << "Can't store routing segment any more, "
                     "storage is full.";
          return false;
        }
        seg->lane_id = lane_id;
        seg->lane_index = lanes_in_boundary_[index].lane_index;
        seg->start_s = static_cast<Float32_t>(start_s);
        seg->end_s = static_cast<Float32_t>(end_s);

#if ENABLE_HD_MAP_TRACE_D600
        std::cout << "Segment[" << loop_count - 1 << "]: ";
        std::cout << "{[" << 0
                  << "] id=" << lane_id.id
                  << ", path_number=" << segment_it.InitialPath
                  << ", lane_number=" << segment_it.InitialLaneNumber
                  << "} " << std::endl;
#endif
      }  // if (end_s - start_s ...
    }  // if (index >= 0 ...
  }  // for (; road_it != road_it_end; ...

#if ENABLE_HD_MAP_TRACE_D600
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

#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "<<<<<<<<< Construct Routing #########" << std::endl;
#endif

  return true;
}

bool HDMap::ParseLaneInfo(const MAP::MAP_LaneInfo& map_laneinfo_in,
                          const MAP::MAP_Location& map_location_in) {
#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "######### ParseLaneInfo ("
            << map_laneinfo_in.base_lane_id
            << ") >>>>>>>>>" << std::endl;
#endif

  // Get a memory space from lane table storage.
  Int32_t lane_storage_index = -1;
  Lane* const lane_info = lane_table_storage_.Allocate(&lane_storage_index);
  if (Nullptr_t == lane_info) {
    LOG_ERR << "Can't save lane information anymore, storage is full.";
    return false;
  }

  // Central Line
  // Get central line of lane which is in the bounding box set before.
  Int32_t loop_count = 0;
  Float64_t accumulated_s_tmp = 0.0;
  Float64_t distance = 0.0;

  bool prev_global_point_valid = false;
  GlobalPoint prev_global_point;
  GlobalPoint curr_global_point;

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of taking path in boundary (Start)
  phoenix::common::Stopwatch performance_timer_take_path;
#endif

  temp_data_.take_path.Clear();
  std::vector<MAP::WGS84Point>::const_iterator curve_it =
      map_laneinfo_in.center_points.begin();
  for (; curve_it != map_laneinfo_in.center_points.end(); ++curve_it) {
    const MAP::WGS84Point& point_it = *curve_it;
    Float64_t x = 0.0;
    Float64_t y = 0.0;

  #if (COORDINATE_CONVERSION_MODE == ENABLE_WGS84TOUTM)
    // 将WGS84格式的经纬度坐标转换为通用横墨卡托格网系统下的坐标
    Float64_t convergence_angle =
      phoenix::common::LLtoUTM(point_it.Latitude, point_it.Longitude, &y, &x);
    // 构建车道线时，只用到了形点信息。
    (void)convergence_angle;
  #elif (COORDINATE_CONVERSION_MODE == ENABLE_WGS84TOLOCAL)
    // 将WGS84格式的经纬度坐标转换为局部坐标
    phoenix::common::GpsPoint base_gps_in_;  // 车身GPS坐标
    phoenix::common::GpsPoint gps_coor_in_;  // 车道线GPS坐标
    phoenix::common::Vec2d local_coordinate_out_; // 输出的局部坐标
    base_gps_in_.latitude = map_location_in.location.Latitude;
    base_gps_in_.longitude = map_location_in.location.Longitude;
    gps_coor_in_.latitude = point_it.Latitude;
    gps_coor_in_.longitude = point_it.Longitude;
    // GPS转局部坐标
    phoenix::common::ConvGpsPointToLocalPoint(base_gps_in_, gps_coor_in_,
                                              &local_coordinate_out_);
    x = static_cast<Float64_t>(local_coordinate_out_.x());
    y = static_cast<Float64_t>(local_coordinate_out_.y());
  #elif (COORDINATE_CONVERSION_MODE == ENABLE_UTMORLOCAL)
    // 直接输入的为通用横墨卡托格网系统下的坐标或者局部坐标
    x = point_it.Longitude;
    y = point_it.Latitude;
  #endif

    if (loop_count > 0) {
      // 计算两点间的距离
      distance = common::com_sqrt(
            common::Square(x - prev_global_point.x) +
            common::Square(y - prev_global_point.y));

      // 过滤掉距离过近的点
      if (distance < min_lane_point_interval_) {
        continue;
      }
      // 计算原始地图车道中心线处每个点的轨迹长
      accumulated_s_tmp += distance;
    }

    // current point
    curr_global_point.x = x;
    curr_global_point.y = y;
    curr_global_point.s = accumulated_s_tmp;

    // 判断该点是否位于包围盒内
    const bool is_in_boundary = IsPointInMapBoundingBox(x, y);

    // Take points in boundary
    TakePointsInBoundary(is_in_boundary, prev_global_point_valid,
                         prev_global_point, curr_global_point);

    // 保存这个点，下个迭代会使用
    prev_global_point_valid = true;
    prev_global_point.x = x;
    prev_global_point.y = y;
    prev_global_point.s = accumulated_s_tmp;
    loop_count++;
  }

  // Select better segment
  Int32_t better_seg = 0;
  if (false == SelectBetterLaneSeg(&better_seg)) {
    LOG_ERR << "There are not enough points in this lane.";
    lane_table_storage_.PopBack();
    return false;
  }

  LimitLaneRang(better_seg);

  if (temp_data_.take_path.lane_seg[better_seg].hold_points.Size() < 2) {
    LOG_ERR << "There are not enough points in this lane.";
    lane_table_storage_.PopBack();
    return false;
  }

  const Float64_t start_s = temp_data_.take_path.lane_seg[better_seg].
      hold_points.Front()->point.s;
  const Float64_t end_s = temp_data_.take_path.lane_seg[better_seg].
      hold_points.Back()->point.s;

#if ENABLE_HD_MAP_TRACE_D600
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
            << ", accumulated_s=" << accumulated_s_tmp
            << ", total_len="
            << temp_data_.take_path.lane_seg[better_seg].
               hold_points.Back()->point.s -
               temp_data_.take_path.lane_seg[better_seg].
               hold_points.Front()->point.s
            << std::endl;
#endif

  // Convert global points to points related to vehicle coordinate.
  temp_data_.points.Clear();
  ConvertPointsToVehicleCoordinate(
        temp_data_.take_path.lane_seg[better_seg].hold_points,
        &temp_data_.points);

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
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

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of constructing path of lane (Start)
  phoenix::common::Stopwatch performance_timer_construct_path;
#endif

  // Save central line to this lane.
  lane_info->central_curve().Clear();
  if (!lane_info->central_curve().Construct(temp_data_.points)) {
    LOG_ERR << "Fail to construct this lane.";
    lane_table_storage_.PopBack();
    return false;
  }
  if (lane_info->central_curve().points().Size() < 2) {
    LOG_ERR << "There are not enough points in path of this lane.";
    lane_table_storage_.PopBack();
    return false;
  }

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of constructing path of lane (End)
  std::cout << "Constructing path spend "
            << performance_timer_construct_path.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of parsing other lane information (Start)
  phoenix::common::Stopwatch performance_timer_parsing_other;
#endif

  // Lane ID
  lane_info->topology_id().id.id = map_laneinfo_in.base_lane_id;
  lane_info->topology_id().index = lane_storage_index;

  // Lane Boundary
  ParseLaneBoundary(map_laneinfo_in, map_location_in, lane_info);

  // Slope
  ParseLaneSlope(map_laneinfo_in, start_s, end_s, lane_info);

  // Topology
  ParseLaneTopology(map_laneinfo_in, lane_info);

  // 车道类型
  switch (map_laneinfo_in.lane_type) {
    case MAP::LANE_TYPE_UNKNOWN: {
      lane_info->set_lane_type(static_cast<Int32_t>(LANE_TYPE_UNKNOWN));
      break;
    }
    case MAP::LANE_TYPE_NORMAL: {
      lane_info->set_lane_type(static_cast<Int32_t>(LANE_TYPE_CITY_DRIVING));
      break;
    }
    case MAP::LANE_TYPE_ETC:
    case MAP::LANE_TYPE_CHARGE:
    case MAP::LANE_TYPE_BUS_ONLY:
    case MAP::LANE_TYPE_DUMMY:
    case MAP::LANE_TYPE_EMERGENCY_DRIVEWAY:
    case MAP::LANE_TYPE_NA: {
      lane_info->set_lane_type(static_cast<Int32_t>(LANE_TYPE_UNKNOWN));
      break;
    }
    default: {
      lane_info->set_lane_type(static_cast<Int32_t>(LANE_TYPE_UNKNOWN));
      break;
    }
  }
  // lane_info->set_turn_type();
  // 车道方向
  switch (map_laneinfo_in.direction) {
    case MAP::None: {
      lane_info->set_direction(static_cast<Int32_t>(LANE_DIRECTION_FORWARD));
      break;
    }
    case MAP::Both: {
      lane_info->set_direction(
            static_cast<Int32_t>(LANE_DIRECTION_BIDIRECTION));
      break;
    }
    case MAP::AlongDrivingDirection: {
      lane_info->set_direction(static_cast<Int32_t>(LANE_DIRECTION_FORWARD));
      break;
    }
    case MAP::AgainstDrivingDirection: {
      lane_info->set_direction(static_cast<Int32_t>(LANE_DIRECTION_BACKWARD));
      break;
    }
    default: {
      lane_info->set_direction(static_cast<Int32_t>(LANE_DIRECTION_FORWARD));
      break;
    }
  }

  // Speed limit
  if (map_laneinfo_in.speed_limit > 0) {
    // map_laneinfo_in.speed_limit的单位是公里/小时
    lane_info->set_speed_limit(
          static_cast<map_var_t>(map_laneinfo_in.speed_limit) / 3.6F);
  }

  // Add to lane table
  const Int32_t* const lane_index = lane_table_.Find(
        map_laneinfo_in.base_lane_id);
  if (Nullptr_t != lane_index) {
    LOG_ERR << "Detected repeated lane id in lane table.";
    lane_table_storage_.PopBack();
    return false;
  }
  if (!lane_table_.Insert(map_laneinfo_in.base_lane_id, lane_storage_index)) {
    LOG_ERR << "Fail to insert lane table.";
    return false;
  }

  LaneInBoundary* const lane_in_boundary = lanes_in_boundary_.Allocate();
  if (Nullptr_t == lane_in_boundary) {
    LOG_ERR << "Can't save lane segment anymore, storage is full.";
    lane_table_storage_.PopBack();
    return false;
  }
  lane_in_boundary->lane_id = lane_info->topology_id().id;
  lane_in_boundary->lane_index = lane_storage_index;
  lane_in_boundary->start_s = start_s;
  lane_in_boundary->end_s = end_s;

#if ENABLE_HD_MAP_PERFORMANCE_TEST_D600
  /// Performance of parsing other lane information (End)
  std::cout << "Parsing other lane info spend "
            << performance_timer_parsing_other.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_HD_MAP_TRACE_D600
  std::cout << "<<<<<<<<< ParseLaneInfo #########" << std::endl;
#endif

  return (true);
}

void HDMap::ParseLaneBoundary(const MAP::MAP_LaneInfo& map_laneinfo_in,
                              const MAP::MAP_Location& map_location_in,
                              Lane* const lane_info) {
  if (lane_info->central_curve().total_length() > common::kGeometryEpsilon) {
    const map_var_t step_s = lane_info->central_curve().total_length();
    map_var_t s = 0.0F;
    // 车道左边线类型
    Int32_t left_lane_boundary_type = 0;
    // 车道右边线类型
    Int32_t right_lane_boundary_type = 0;
    switch (map_laneinfo_in.left_line_type) {
      // 未知
      case MAP::LINE_TYPE_UNKNOW: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      // 单虚线
      case MAP::LINE_TYPE_SINGLE_DOTTED: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
        break;
      }
      // 单实线
      case MAP::LINE_TYPE_SINGLE_SOLID: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
        break;
      }
      // 双实线
      case MAP::LINE_TYPE_DOUBLE_SOLID: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
        break;
      }
      // 双线（左实右虚）
      case MAP::LINE_TYPE_LEFT_SOLID_RIGHT_DOTTED: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
        break;
      }
      // 双线（左虚右实）
      case MAP::LINE_TYPE_LEFT_DOTTED_RIGHT_SOLID: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
        break;
      }
      // 双虚线
      case MAP::LINE_TYPE_DOUBLE_DOTTED: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
        break;
      }
      case MAP::LINE_TYPE_DIVERTION: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      case MAP::LINE_TYPE_INTERSECTION_VIRTUAL_DIVIDER: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      // 其他
      case MAP::LINE_TYPE_OTHER: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      default: {
        left_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
    }
    switch (map_laneinfo_in.right_line_type) {
      // 未知
      case MAP::LINE_TYPE_UNKNOW: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      // 单虚线
      case MAP::LINE_TYPE_SINGLE_DOTTED: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
        break;
      }
      // 单实线
      case MAP::LINE_TYPE_SINGLE_SOLID: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
        break;
      }
      // 双实线
      case MAP::LINE_TYPE_DOUBLE_SOLID: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
        break;
      }
      // 双线（左实右虚）
      case MAP::LINE_TYPE_LEFT_SOLID_RIGHT_DOTTED: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
        break;
      }
      // 双线（左虚右实）
      case MAP::LINE_TYPE_LEFT_DOTTED_RIGHT_SOLID: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
        break;
      }
      // 双虚线
      case MAP::LINE_TYPE_DOUBLE_DOTTED: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
        break;
      }
      case MAP::LINE_TYPE_DIVERTION: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      case MAP::LINE_TYPE_INTERSECTION_VIRTUAL_DIVIDER: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      // 其他
      case MAP::LINE_TYPE_OTHER: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
      default: {
        right_lane_boundary_type = LANE_BOUNDARY_TYPE_UNKNOWN;
        break;
      }
    }

    // 车道左边线
    for (Int32_t i = 0; i < 2; ++i) {
      Lane::BoundaryAssociation* const lane_boundary =
          lane_info->left_boundary().boundary_samples.Allocate();
      if (Nullptr_t == lane_boundary) {
        LOG_WARN << "Can't save boundary information anymore, "
                    "storage is full.";
        break;
      }
      lane_boundary->s = s;
      lane_boundary->width = map_location_in.current_lane_width / 2.0F;
      lane_boundary->type = left_lane_boundary_type;
      s += step_s;
    }

    // 车道右边线
    s = 0.0F;
    for (Int32_t i = 0; i < 2; ++i) {
      Lane::BoundaryAssociation* const lane_boundary =
          lane_info->right_boundary().boundary_samples.Allocate();
      if (Nullptr_t == lane_boundary) {
        LOG_WARN << "Can't save boundary information anymore, "
                    "storage is full.";
        break;
      }
      lane_boundary->s = s;
      lane_boundary->width = map_location_in.current_lane_width / 2.0F;
      lane_boundary->type = right_lane_boundary_type;
      s += step_s;
    }  // for (Int32_t i = 0; ...
  }  // if (lane_info->central_curve().total_length() > ...
}  // void HDMap::ParseLaneBoundary(...

void HDMap::ParseLaneSlope(
    const MAP::MAP_LaneInfo& lane_in, Float64_t start_s, Float64_t end_s,
    Lane* const lane_info) const {
  // 控制点中坡度的范围为[dist,end_dist)
  // 控制点中坡度的单位为dz/ds，dz为垂直方向的高度，ds为水平方向的距离
  std::vector<MAP::Lane_Control>::const_iterator lane_slope_sample_it =
      lane_in.slope.begin();
  for (; lane_slope_sample_it != lane_in.slope.end(); ++lane_slope_sample_it) {
    const Float64_t s = static_cast<Float64_t>(lane_slope_sample_it->dist);
    const Float64_t slope_end_s =
        static_cast<Float64_t>(lane_slope_sample_it->end_dist);
    if ((start_s <= s) && (s <= end_s)) {
      if (lane_info->slope_samples().Empty()) {
        if (start_s > 0.1) {
          Lane::SlopeAssociation* const lane_slope =
              lane_info->slope_samples().Allocate();
          if (Nullptr_t == lane_slope) {
            LOG_WARN << "Can't save slope information anymore, "
                        "storage is full.";
            break;
          }
          lane_slope->s = 0.0F;
          lane_slope->slope = lane_slope_sample_it->value;
        }
      } else {
        // 过滤掉距离过近的点
        if ((s - static_cast<Float64_t>(lane_info->slope_samples().Back().s)) <
            static_cast<Float64_t>(min_lane_point_interval_)) {
          continue;
        }
      }
      Lane::SlopeAssociation* const lane_slope =
          lane_info->slope_samples().Allocate();
      if (Nullptr_t == lane_slope) {
        LOG_WARN << "Can't save slope information anymore, storage is full.";
        break;
      }
      lane_slope->s = static_cast<map_var_t>(s - start_s);
      lane_slope->slope = lane_slope_sample_it->value;

      if ((slope_end_s + 0.1) >= end_s) {
        // 将车道的最后一个点的坡度加到坡度信息中
        Lane::SlopeAssociation* const lane_slope_end_point =
            lane_info->slope_samples().Allocate();
        if (Nullptr_t == lane_slope_end_point) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
          break;
        }
        lane_slope_end_point->s = static_cast<map_var_t>(slope_end_s - start_s);
        lane_slope_end_point->slope = lane_slope_sample_it->value;
        break;
      }
    } else if (s < start_s) {
      if (slope_end_s >= end_s) {
        // 若当前车道段被某个控制点完全包含时，
        // 将车道段起点和终点的坡度均设置为此控制点的坡度
        Lane::SlopeAssociation * const lane_slope_start_point =
            lane_info->slope_samples().Allocate();
        if (Nullptr_t == lane_slope_start_point) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
          break;
        }
        lane_slope_start_point->s = 0.0F;
        lane_slope_start_point->slope = lane_slope_sample_it->value;
        Lane::SlopeAssociation * const lane_slope_end_point =
            lane_info->slope_samples().Allocate();
        if (Nullptr_t == lane_slope_end_point) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
          break;
        }
        lane_slope_end_point->s = static_cast<map_var_t>(end_s - start_s);
        lane_slope_end_point->slope = lane_slope_sample_it->value;
        break;
      } else if (slope_end_s > start_s) {
        // 若当前车道段的起点在某个控制点上，终点不在此控制点上时。
        // 将车道段起点的坡度设置为此控制点的坡度
        Lane::SlopeAssociation * const lane_slope_start_point =
            lane_info->slope_samples().Allocate();
        if (Nullptr_t == lane_slope_start_point) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
          break;
        }
        lane_slope_start_point->s = 0.0F;
        lane_slope_start_point->slope = lane_slope_sample_it->value;
      }
    }
  }
}

void HDMap::ParseLaneTopology(const MAP::MAP_LaneInfo& map_lane,
                              Lane* const lane_output) {
  // 左右相邻车道
  if (map_lane.left_lane_id > 0) {
    lane_output->left_neighbor_forward_lanes().Resize(1);
    lane_output->left_neighbor_forward_lanes()[0].id.id = map_lane.left_lane_id;
    lane_output->left_neighbor_forward_lanes()[0].index = -1;
  }
  if (map_lane.right_lane_id > 0) {
    lane_output->right_neighbor_forward_lanes().Resize(1);
    lane_output->right_neighbor_forward_lanes()[0].id.id =
        map_lane.right_lane_id;
    lane_output->right_neighbor_forward_lanes()[0].index = -1;
  }
  // 前方相连车道
  std::vector<uint64_t>::const_iterator lane_id_it =
      map_lane.next_lane_id.begin();
  for (; lane_id_it != map_lane.next_lane_id.end(); ++lane_id_it) {
    Lane::TopologyId* const topology_id =
        lane_output->successor_lanes().Allocate();
    if (Nullptr_t == topology_id) {
      LOG_WARN << "Can't save successor id anymore, storage is full.";
      break;
    }
    topology_id->id.id = *lane_id_it;
    topology_id->index = -1;
  }
}

#endif // #if HD_MAP_TYPE == HD_MAP_TYPE_D600


}  // namespace driv_map
}  // namespace phoenix


