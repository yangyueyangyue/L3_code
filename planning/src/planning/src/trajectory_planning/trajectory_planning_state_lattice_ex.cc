
#include "trajectory_planning/trajectory_planning_state_lattice.h"
#include "utils/log.h"

#define ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE (0)
#define ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST (0)

namespace phoenix {
namespace planning {


bool TrajectoryPlanningStateLattice::CreateRoadGraph_Type1() {

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST
  phoenix::common::Stopwatch timer_create_lon_graph_samples;
#endif

  if (!CreateLonGraphSamples_Type1()) {
    LOG_ERR << "Failed to create longitudinal graph samples.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST
  std::cout << "      @ Creating longitudinal graph sameples spends "
            << timer_create_lon_graph_samples.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST
  phoenix::common::Stopwatch timer_create_graph_nodes;
#endif

  if (!CreatGraphNodes_Type1()) {
    LOG_ERR << "Failed to create graph nodes.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST
  std::cout << "      @ Creating graph nodes spends "
            << timer_create_graph_nodes.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST
  phoenix::common::Stopwatch timer_create_graph_links;
#endif

  if (!CreateGraphLinks_Type1()) {
    LOG_ERR << "Failed to create linkes of graph nodes.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_EX_PERFORMANCE_TEST
  std::cout << "      @ Creating graph links spends "
            << timer_create_graph_links.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "The road graph is (Type I):" << std::endl;
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    const GraphSection& graph_section = road_graph_[i];
    std::cout << ">>> In graph section " << i << ":" << std::endl;
    std::cout << "    lon_major=" << graph_section.lon_major
              << ", lon_minor=" << graph_section.lon_minor
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
              << ", lane_on_ref=" << driving_map_->GetLaneIdByIndex(
                   graph_section.lane_index_on_ref).id.c_str()
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
              << ", lane_on_ref=" << driving_map_->GetLaneIdByIndex(
                   graph_section.lane_index_on_ref).id
#endif
              << ", s_ref=" << graph_section.proj_on_ref.s
              << ", min_lat_major=" << graph_section.min_lat_major
              << ", max_lat_major=" << graph_section.max_lat_major
              << std::endl;

    Int32_t lane_section_size = graph_section.lane_sections.Size();
    std::cout << "    There are " << lane_section_size
              << " lane sections:" << std::endl;
    for (Int32_t j = 0; j < lane_section_size; ++j) {
      const GraphSection::LaneSection& lane_section =
          graph_section.lane_sections[j];
      std::cout << "    >>> In lane section " << j << ":" << std::endl;
      std::cout << "        lat_major=" << lane_section.lat_major
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
                << ", lane=" << driving_map_->GetLaneIdByIndex(
                     lane_section.lane_index).id.c_str()
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
                << ", lane=" << driving_map_->GetLaneIdByIndex(
                     lane_section.lane_index).id
#endif
                << ", left_width=" << lane_section.left_width
                << ", right_width=" << lane_section.right_width
                << ", s_lane=" << lane_section.proj_on_lane.s
                << ", min_lat_minor=" << lane_section.min_lat_minor
                << ", max_lat_minor=" << lane_section.max_lat_minor
                << std::endl;

      Int32_t nodes_size = lane_section.nodes.Size();
      std::cout << "        There are " << nodes_size << " nodes:" << std::endl;
      for (Int32_t k = 0; k < nodes_size; ++k) {
        const GraphSection::GraphNode& node = lane_section.nodes[k];

        std::cout << "        >>> In node " << k << ":" << std::endl;
        std::cout << "            lat_minor=" << node.lat_minor
                  << ", lat_offset=" << node.lat_offset
                  << std::endl;

        std::cout << "            >>> There are " << node.link_num
                  << " links:" << std::endl;
        common::internal::DoubleList::const_iterator<
            GraphLinkListNode, MAX_GRAPH_LINK_NUM> it =
            node.link_list.cbegin(road_graph_link_storage_);
        common::internal::DoubleList::const_iterator<
            GraphLinkListNode, MAX_GRAPH_LINK_NUM> it_end =
            node.link_list.cend(road_graph_link_storage_);
        Int32_t count = 0;
        for (; it != it_end; ++it) {
          const GraphSection::GraphNode::Link& link = it->link;

          const GraphSection& graph_section_to =
              road_graph_[link.node_to.graph_section_index];
          const GraphSection::LaneSection& lane_section_to =
              graph_section_to.lane_sections[link.node_to.lane_section_index];
          const GraphSection::GraphNode& node_to =
              lane_section_to.nodes[link.node_to.node_index];

          std::cout << "                link[" << count++
                    << "], from node[" << link.node_from.graph_section_index
                    << "," << link.node_from.lane_section_index
                    << "," << link.node_from.node_index
                    << "] to node[" << link.node_to.graph_section_index
                    << "," << link.node_to.lane_section_index
                    << "," << link.node_to.node_index
                    << "], from num[" << graph_section.lon_major
                    << "," << graph_section.lon_minor
                    << "," << lane_section.lat_major
                    << "," << node.lat_minor
                    << "] to num[" << graph_section_to.lon_major
                    << "," << graph_section_to.lon_minor
                    << "," << lane_section_to.lat_major
                    << "," << node_to.lat_minor
                    << "]"
                    << std::endl;
          std::cout << "                         abs_curvature="
                    << link.abs_curvature
                    << ", deceleration_by_curvature="
                    << link.deceleration_by_curvature
                    << ", "
                    << std::endl;
          std::cout << "                         there is risky(risk="
                    << link.collision_risk_value
                    << ") obstacle(collison_dist="
                    << link.static_collision_dist_to_risky_obj
                    << ") on curr_ref_line(relative_s="
                    << link.dist_of_risky_obj_on_curr_ref
                    << "), deceleration_by_obstacle="
                    << link.deceleration_by_obstacle
                    << ", "
                    << std::endl;
          std::cout << "                         Cost[lane:"
                    << link.cost.lane_cost
                    << ", lat_offet:" << link.cost.lateral_offset_cost
                    << ", curvature:" << link.cost.curvature_cost
                    << ", collision:" << link.cost.collision_cost
                    << "]"
                    << std::endl;
        }
      }
    }
  }
#endif

  return true;
}

bool TrajectoryPlanningStateLattice::CreateRoadGraph_Type2() {
  if (!CreateLonGraphSamples_Type2()) {
    LOG_ERR << "Failed to create longitudinal graph samples.";
    return false;
  }

  if (!CreatGraphNodes_Type2()) {
    LOG_ERR << "Failed to create graph nodes.";
    return false;
  }

  if (!CreateGraphLinks_Type2()) {
    LOG_ERR << "Failed to create linkes of graph nodes.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "The road graph is (Type II):" << std::endl;
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    const GraphSection& graph_section = road_graph_[i];
    std::cout << ">>> In graph section " << i << ":" << std::endl;
    std::cout << "    lon_major=" << graph_section.lon_major
              << ", lon_minor=" << graph_section.lon_minor
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
              << ", lane_on_ref=" << driving_map_->GetLaneIdByIndex(
                   graph_section.lane_index_on_ref).id.c_str()
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
              << ", lane_on_ref=" << driving_map_->GetLaneIdByIndex(
                   graph_section.lane_index_on_ref).id
#endif
              << ", s_ref=" << graph_section.proj_on_ref.s
              << ", min_lat_major=" << graph_section.min_lat_major
              << ", max_lat_major=" << graph_section.max_lat_major
              << std::endl;

    Int32_t lane_section_size = graph_section.lane_sections.Size();
    std::cout << "    There are " << lane_section_size
              << " lane sections:" << std::endl;
    for (Int32_t j = 0; j < lane_section_size; ++j) {
      const GraphSection::LaneSection& lane_section =
          graph_section.lane_sections[j];
      std::cout << "    >>> In lane section " << j << ":" << std::endl;
      std::cout << "        lat_major=" << lane_section.lat_major
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
                << ", lane=" << driving_map_->GetLaneIdByIndex(
                     lane_section.lane_index).id.c_str()
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
                << ", lane=" << driving_map_->GetLaneIdByIndex(
                     lane_section.lane_index).id
#endif
                << ", left_width=" << lane_section.left_width
                << ", right_width=" << lane_section.right_width
                << ", s_lane=" << lane_section.proj_on_lane.s
                << ", min_lat_minor=" << lane_section.min_lat_minor
                << ", max_lat_minor=" << lane_section.max_lat_minor
                << std::endl;

      Int32_t nodes_size = lane_section.nodes.Size();
      std::cout << "        There are " << nodes_size << " nodes:" << std::endl;
      for (Int32_t k = 0; k < nodes_size; ++k) {
        const GraphSection::GraphNode& node = lane_section.nodes[k];

        std::cout << "        >>> In node " << k << ":" << std::endl;
        std::cout << "            lat_minor=" << node.lat_minor
                  << ", lat_offset=" << node.lat_offset
                  << std::endl;

        std::cout << "            >>> There are " << node.link_num
                  << " links:" << std::endl;
        common::internal::DoubleList::const_iterator<
            GraphLinkListNode, MAX_GRAPH_LINK_NUM> it =
            node.link_list.cbegin(road_graph_link_storage_);
        common::internal::DoubleList::const_iterator<
            GraphLinkListNode, MAX_GRAPH_LINK_NUM> it_end =
            node.link_list.cend(road_graph_link_storage_);
        Int32_t count = 0;
        for (; it != it_end; ++it) {
          const GraphSection::GraphNode::Link& link = it->link;

          const GraphSection& graph_section_to =
              road_graph_[link.node_to.graph_section_index];
          const GraphSection::LaneSection& lane_section_to =
              graph_section_to.lane_sections[link.node_to.lane_section_index];
          const GraphSection::GraphNode& node_to =
              lane_section_to.nodes[link.node_to.node_index];

          std::cout << "                link[" << count++
                    << "], from node[" << link.node_from.graph_section_index
                    << "," << link.node_from.lane_section_index
                    << "," << link.node_from.node_index
                    << "] to node[" << link.node_to.graph_section_index
                    << "," << link.node_to.lane_section_index
                    << "," << link.node_to.node_index
                    << "], from num[" << graph_section.lon_major
                    << "," << graph_section.lon_minor
                    << "," << lane_section.lat_major
                    << "," << node.lat_minor
                    << "] to num[" << graph_section_to.lon_major
                    << "," << graph_section_to.lon_minor
                    << "," << lane_section_to.lat_major
                    << "," << node_to.lat_minor
                    << "]"
                    << std::endl;
          std::cout << "                         abs_curvature="
                    << link.abs_curvature
                    << ", deceleration_by_curvature="
                    << link.deceleration_by_curvature
                    << ", "
                    << std::endl;
          std::cout << "                         there is risky(risk="
                    << link.collision_risk_value
                    << ") obstacle(collison_dist="
                    << link.static_collision_dist_to_risky_obj
                    << ") on curr_ref_line(relative_s="
                    << link.dist_of_risky_obj_on_curr_ref
                    << "), deceleration_by_obstacle="
                    << link.deceleration_by_obstacle
                    << ", "
                    << std::endl;
          std::cout << "                         Cost[lane:"
                    << link.cost.lane_cost
                    << ", lat_offet:" << link.cost.lateral_offset_cost
                    << ", curvature:" << link.cost.curvature_cost
                    << ", collision:" << link.cost.collision_cost
                    << "]"
                    << std::endl;
        }
      }
    }
  }
#endif

  return true;
}

bool TrajectoryPlanningStateLattice::CreateLonGraphSamples_Type1() {
  // Clear road graph
  for (Int32_t i = 0; i < road_graph_link_storage_.Size(); ++i) {
    road_graph_link_storage_[i].Clear();
  }
  road_graph_link_storage_.Clear();
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    road_graph_[i].Clear();
  }
  road_graph_.Clear();
  lon_graph_samples_step_len_.Clear();

  // Get the first forward risky obstacle information in current reference line.
  // Which is used to shrink sample length.
  Int32_t first_forward_risky_obj_index = -1;
  plan_var_t first_forward_risky_obj_dist = 999999;
  plan_var_t min_first_forward_segment_len =
      common::Max(3.0F * curr_position_.v, param_.vehicle_length+3.0F);
  driv_map::CollisionTestResult coll_test_ret;
  driving_map_->GetRiskyObstacles(major_ref_line_index_, false, &coll_test_ret);
  for (Int32_t i = 0; i < coll_test_ret.risky_obj_list.Size(); ++i) {
    const driv_map::CollisionTestResult::ObjInfo& collision_info = coll_test_ret.risky_obj_list[i];
    if (collision_info.dynamic_distance < 1.0f) {
      if ((min_first_forward_segment_len < collision_info.collision_s) &&
          (collision_info.collision_s < first_forward_risky_obj_dist)) {
        first_forward_risky_obj_dist = collision_info.collision_s;
        first_forward_risky_obj_index = i;
      }
    }
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "first_forward_risky_obj_index = "
            << first_forward_risky_obj_index << std::endl;
  std::cout << "first_forward_risky_obj_dist = "
            << first_forward_risky_obj_dist << std::endl;
#endif

  GraphSection* sample = road_graph_.Allocate();
  if (Nullptr_t == sample) {
    LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
    return false;
  }
  sample->lon_major = 0;
  sample->lon_minor = 0;
  sample->proj_on_ref = veh_proj_on_major_ref_line_;
  driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
        major_ref_line_index_, sample->proj_on_ref.s,
        &(sample->lane_index_on_ref));
  if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
    road_graph_.PopBack();
    LOG_ERR << "Can't find valid reference line.";
    return false;
  }

  plan_var_t start_s = veh_proj_on_major_ref_line_.s;
  plan_var_t step_len = 0;
  for (Int32_t index = 1; index < MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM; ++index) {
    // Limit sampled step length by curvature of trajectory
    step_len = CalcStepLenOfLonGraphSample(start_s);

    // Limit sampled step length by obstacles
    if (1 == index) {
      // Only limit the first sampled step length
      if (first_forward_risky_obj_index >= 0) {
        if (first_forward_risky_obj_dist < step_len) {
          step_len = first_forward_risky_obj_dist;
        }
      }
    }

    // Limit min and max sampled step length
    if ((index-1) < param_.lon_sample_step_len_limit_table.Size()) {
      plan_var_t min_step_len =
          param_.lon_sample_step_len_limit_table[index-1].first;
      plan_var_t max_step_len =
          param_.lon_sample_step_len_limit_table[index-1].second;
      if (step_len < min_step_len) {
        step_len = min_step_len;
      }
      if (step_len > max_step_len) {
        step_len = max_step_len;
      }
    }

    // Limit the total sampled length less than total reference lines length
    // and the looking forward length.
    plan_var_t s_ref = start_s + step_len;
    if ((s_ref - veh_proj_on_major_ref_line_.s) > look_forward_length_) {
      s_ref = veh_proj_on_major_ref_line_.s + look_forward_length_;
      if ((s_ref - start_s) < 5.0f) {
        s_ref = start_s + 5.0f;
      }
    }
    if (major_ref_line_->total_length() > 15.0F) {
      if (s_ref > (major_ref_line_->total_length()-5.0F)) {
        s_ref = major_ref_line_->total_length()-5.0F;
      }
    } else {
      if (s_ref > (major_ref_line_->total_length()-0.5F)) {
        s_ref = major_ref_line_->total_length()-0.5F;
      }
    }
    step_len = s_ref - start_s;
    if (step_len < 2.0f) {
      break;
    }

    // Save sampled step length
    lon_graph_samples_step_len_.PushBack(step_len);

    // Get leading point on reference line for LKA
    common::PathPoint point_on_ref;
    if (1 == index) {
      if (leading_length_for_lka_ < (step_len - 2.0f)) {
        major_ref_line_->FindSmoothPoint(
              start_s+leading_length_for_lka_, &point_on_ref);
        sample = road_graph_.Allocate();
        if (Nullptr_t == sample) {
          LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
          return false;
        }
        sample->lon_major = 0;
        sample->lon_minor = -1;
        sample->proj_on_ref = point_on_ref;
        driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
              major_ref_line_index_, sample->proj_on_ref.s,
              &(sample->lane_index_on_ref));
        if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
          road_graph_.PopBack();
        }
      }
    }

    // Get minor point on feference line for segment 2
    if (2 == index) {
      if (step_len > 4.0f*2.0f) {
        plan_var_t step = step_len / 4.0f;
        for (Int32_t i = 1; i < 4; ++i) {
          major_ref_line_->FindSmoothPoint(start_s + step*i, &point_on_ref);
          sample = road_graph_.Allocate();
          if (Nullptr_t == sample) {
            LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
            return false;
          }
          sample->lon_major = index;
          sample->lon_minor = i-4;
          sample->proj_on_ref = point_on_ref;
          driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
                major_ref_line_index_, sample->proj_on_ref.s,
                &(sample->lane_index_on_ref));
          if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
            road_graph_.PopBack();
          }
        }
      } else if (step_len > 3.0f*2.0f) {
        plan_var_t step = step_len / 3.0f;
        for (Int32_t i = 1; i < 3; ++i) {
          major_ref_line_->FindSmoothPoint(start_s + step*i, &point_on_ref);
          sample = road_graph_.Allocate();
          if (Nullptr_t == sample) {
            LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
            return false;
          }
          sample->lon_major = index;
          sample->lon_minor = i-3;
          sample->proj_on_ref = point_on_ref;
          driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
                major_ref_line_index_, sample->proj_on_ref.s,
                &(sample->lane_index_on_ref));
          if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
            road_graph_.PopBack();
          }
        }
      } else if (step_len > 2.0f*2.0f) {
        plan_var_t step = step_len / 2.0f;
        for (Int32_t i = 1; i < 2; ++i) {
          major_ref_line_->FindSmoothPoint(start_s + step*i, &point_on_ref);
          sample = road_graph_.Allocate();
          if (Nullptr_t == sample) {
            LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
            return false;
          }
          sample->lon_major = index;
          sample->lon_minor = i-2;
          sample->proj_on_ref = point_on_ref;
          driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
                major_ref_line_index_, sample->proj_on_ref.s,
                &(sample->lane_index_on_ref));
          if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
            road_graph_.PopBack();
          }
        }
      } else {
        // nothing to do
      }
    }

    // Get major point on reference line
    major_ref_line_->FindSmoothPoint(s_ref, &point_on_ref);
    sample = road_graph_.Allocate();
    if (Nullptr_t == sample) {
      LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
      return false;
    }
    sample->lon_major = index;
    sample->lon_minor = 0;
    sample->proj_on_ref = point_on_ref;
    driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
          major_ref_line_index_, sample->proj_on_ref.s,
          &(sample->lane_index_on_ref));
    if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
      road_graph_.PopBack();
    }

    if ((s_ref - veh_proj_on_major_ref_line_.s) > (look_forward_length_-1.0f)) {
      break;
    }

    // Update loop variables
    start_s = s_ref;
  }

  if (road_graph_.Size() < 2) {
    LOG_ERR << "There are not enough longitudinal sampled points.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "The longitudinal graph sampled lengths are (Type I):"
            << std::endl;
  for (Int32_t i = 0; i < lon_graph_samples_step_len_.Size(); ++i) {
    std::cout << " " << lon_graph_samples_step_len_[i] << ",";
  }
  std::cout << " {end}" << std::endl;
  std::cout << "The longitudinal graph samples are: " << std::endl;
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    std::cout << "[" << i
              << "], lon_major=" << road_graph_[i].lon_major
              << ", lon_minor=" << road_graph_[i].lon_minor
              << ", s_ref=" << road_graph_[i].proj_on_ref.s
              << ", lane_index=" << road_graph_[i].lane_index_on_ref
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
              << ", lane_id=" << driving_map_->GetLaneIdByIndex(
                   road_graph_[i].lane_index_on_ref).id.c_str()
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
              << ", lane_id=" << driving_map_->GetLaneIdByIndex(
                   road_graph_[i].lane_index_on_ref).id
#endif
              << std::endl;
  }
#endif

  return true;
}

bool TrajectoryPlanningStateLattice::CreatGraphNodes_Type1() {
  Int32_t road_graph_section_size = road_graph_.Size();

  Int32_t nearest_neighbor_index = 0;
  common::StaticVector<driv_map::NeighborsLaneInfo,
      driv_map::MAX_NEIGHBOR_LANE_NUM> sorted_neighbor_lanes;
  common::StaticVector<Int32_t, driv_map::MAX_NEIGHBOR_LANE_NUM>
      directed_neighbors;

  plan_var_t lat_sample_step_len = param_.lat_sample_step_len;

  /* k002 pengc 2022-08-15 (begin) */
  /* 添加偏离车道线行驶的功能 */
  GraphSection::LaneSection* lka_lane_section = Nullptr_t;
  GraphSection::GraphNode* lka_node = Nullptr_t;
  GraphSection::GraphNode* first_off_center_node = Nullptr_t;
  /* k002 pengc 2022-08-15 (end) */
  for (Int32_t section_index = 0; section_index < road_graph_section_size;
       ++section_index) {
    GraphSection& graph_section = road_graph_[section_index];
    graph_section.max_lat_major = 0;
    graph_section.min_lat_major = 0;

    if (0 == graph_section.lon_major) {
      const common::Path& lane_curve = driving_map_->GetLaneCentralCurve(
            graph_section.lane_index_on_ref);
      common::PathPoint proj_on_lane;
      lane_curve.FindProjection(graph_section.proj_on_ref.point, &proj_on_lane);
      /* k001 pengc 2022-03-19 (begin) */
      // 确保轨迹按照参考线生成（地图及相机融合模式下，会修改当前参考线）
      proj_on_lane.point = graph_section.proj_on_ref.point;
      proj_on_lane.l = graph_section.proj_on_ref.l;
      /* k001 pengc 2022-03-19 (end) */

      GraphSection::LaneSection* lane_section =
          graph_section.lane_sections.Allocate();
      if (Nullptr_t == lane_section) {
        LOG_ERR << "Can't add lane section to graph, storage is full.";
        break;
      }
      lane_section->lat_major = 0;
      lane_section->lane_index = graph_section.lane_index_on_ref;
      driving_map_->GetLaneWidth(
            lane_section->lane_index, proj_on_lane.s,
            &(lane_section->left_width), &(lane_section->right_width));
      lane_section->proj_on_lane = proj_on_lane;
      lane_section->max_lat_minor = 0;
      lane_section->min_lat_minor = 0;

      GraphSection::GraphNode* node = lane_section->nodes.Allocate();
      if (Nullptr_t == node) {
        LOG_ERR << "Can't add graph node, storage is full.";
        break;
      }
      node->lat_minor = 0;

      if (0 == graph_section.lon_minor) {
        node->lat_offset = graph_section.proj_on_ref.l;
      } else {
        node->lat_offset = 0;

        /* k002 pengc 2022-08-15 (begin) */
        /* 添加偏离车道线行驶的功能 */
        lka_lane_section = lane_section;
        lka_node = node;
        /* k002 pengc 2022-08-15 (end) */
      }

      continue;
    }

    if (!driving_map_->FindNeighbors(
          graph_section.proj_on_ref.point,
          graph_section.lane_index_on_ref,
          &sorted_neighbor_lanes,
          &nearest_neighbor_index)) {
      LOG_ERR << "Failed to find neighbor lanes of the basis lane "
                 "in graph secton " << section_index;
      return false;
    }

#if 1
    /// TODO: 变道时检测到了这个错误，后期调查,
    /// bag: planning/log/20220326/2022-03-26-14-23-08.bag
    if (sorted_neighbor_lanes[nearest_neighbor_index].lane_index !=
        graph_section.lane_index_on_ref) {
      LOG_ERR << "Detected unexpected nearest lane ["
              << sorted_neighbor_lanes[nearest_neighbor_index].lane_index
              << "] in neighbors, which is not equal to basis lane ["
              << graph_section.lane_index_on_ref
              << "] in graph section " << section_index;
      //return false;
    }
#endif

    Int32_t sorted_neighbor_lanes_size = sorted_neighbor_lanes.Size();
    Int32_t neighbor_index = nearest_neighbor_index;
    // 按照距离当前车道从小到大的顺序进行排序后的左右相邻车道在
    // neighbor_lanes中索引的列表。
    // 排列的顺序为： nearest_neighbor_index + {0,1,-1,2,-2,...}。
    // 这样排序的目的是防止可能的Link个数超过最大Link个数时，
    // 无Link经过最近车道。
    directed_neighbors.Clear();
    directed_neighbors.PushBack(nearest_neighbor_index);
    for (Int32_t i = 1; i < sorted_neighbor_lanes_size; ++i) {
      neighbor_index = nearest_neighbor_index + i;
      if (neighbor_index < sorted_neighbor_lanes_size) {
        directed_neighbors.PushBack(neighbor_index);
      }

      neighbor_index = nearest_neighbor_index - i;
      if (neighbor_index >= 0) {
        directed_neighbors.PushBack(neighbor_index);
      }
    }

    COM_CHECK(directed_neighbors.Size() == sorted_neighbor_lanes_size);

    for (Int32_t index = 0; index < sorted_neighbor_lanes_size; ++index) {
      neighbor_index = directed_neighbors[index];
      const driv_map::NeighborsLaneInfo& neighbor =
          sorted_neighbor_lanes[neighbor_index];
      /* k001 pengc 2021-06-17 (begin) */
      // 投影点不在相邻车道长度范围内时，缩短采样长度
      if (!neighbor.valid_lat) {
        continue;
      }
      /* k001 pengc 2021-06-17 (end) */

      /* k001 pengc 2021-07-29 (begin) */
      if (!config_.enable_changing_lane) {
        if (0 != (neighbor_index - nearest_neighbor_index)) {
          // 不生成变道的轨迹
          continue;
        }
      }
      /* k001 pengc 2021-07-29 (end) */

      GraphSection::LaneSection* lane_section =
          graph_section.lane_sections.Allocate();
      if (Nullptr_t == lane_section) {
        LOG_ERR << "Can't add lane section to graph, storage is full.";
        break;
      }
      lane_section->lat_major = neighbor_index - nearest_neighbor_index;
      lane_section->lane_index = neighbor.lane_index;
      /* k001 pengc 2021-06-17 (begin) */
      // 投影点不在相邻车道长度范围内时，缩短采样长度
      if (neighbor.valid_lon) {
        driving_map_->GetLaneWidth(
              lane_section->lane_index, neighbor.proj_point.s,
              &(lane_section->left_width), &(lane_section->right_width));
        lane_section->proj_on_lane = neighbor.proj_point;
      } else {
        bool success = false;
        if (driving_map_->IsValidLaneIndex(neighbor.lane_index)) {
          const common::Path& neighbor_path =
              driving_map_->GetLaneCentralCurve(neighbor.lane_index);
          if (neighbor.proj_point.s > neighbor_path.total_length()) {
            if (section_index > 0) {
              const GraphSection& prev_graph_section =
                  road_graph_[section_index-1];
              plan_var_t step_len = graph_section.proj_on_ref.s -
                  prev_graph_section.proj_on_ref.s;
              plan_var_t diff_len =
                  neighbor.proj_point.s - neighbor_path.total_length();
              if (diff_len < (step_len - 5.0F)) {
                common::PathPoint new_point;
                if (neighbor_path.FindSmoothPoint(
                      neighbor_path.total_length()-0.5F, &new_point)) {
                  common::PathPoint proj_point;
                  if (major_ref_line_->FindProjection(
                        new_point.point, &proj_point)) {
                    lane_section->proj_on_lane = new_point;
                    lane_section->proj_on_lane.l = -proj_point.l;
                    driving_map_->GetLaneWidth(
                          lane_section->lane_index,
                          lane_section->proj_on_lane.s,
                          &(lane_section->left_width),
                          &(lane_section->right_width));
                    success = true;
                  }
                }
              }
            }
          }
        }
        if (!success) {
          graph_section.lane_sections.PopBack();
          continue;
        }
      }
      /* k001 pengc 2021-06-17 (end) */
      /* k001 pengc 2022-03-19 (begin) */
      // 确保轨迹按照参考线生成（地图及相机融合模式下，会修改当前参考线）
      if (0 == lane_section->lat_major) {
        lane_section->proj_on_lane.point = graph_section.proj_on_ref.point;
        lane_section->proj_on_lane.l = graph_section.proj_on_ref.l;
      }
      /* k001 pengc 2022-03-19 (end) */

      if (lane_section->lat_major > graph_section.max_lat_major) {
        graph_section.max_lat_major = lane_section->lat_major;
      }
      if (lane_section->lat_major < graph_section.min_lat_major) {
        graph_section.min_lat_major = lane_section->lat_major;
      }
      lane_section->max_lat_minor = 0;
      lane_section->min_lat_minor = 0;

      // Get sample range of the lane
      plan_var_t left_lat_sample_range =
          lane_section->left_width - param_.vehicle_width*0.5F;
      plan_var_t right_lat_sample_range =
          lane_section->right_width - param_.vehicle_width*0.5F;
      if (0 == neighbor_index) {
        right_lat_sample_range -=
            param_.dist_of_keeping_away_from_road_boundary;
      } else {
        if (!sorted_neighbor_lanes[neighbor_index-1].valid_lon) {
          right_lat_sample_range -=
              param_.dist_of_keeping_away_from_road_boundary;
        } else {
          /* k001 pengc 2022-02-09 (begin) */
          // 允许生成车道内避让的轨迹
          if (result_of_action_planning_.bypassing_req.allow_turning_right) {
            right_lat_sample_range -=
                param_.dist_of_keeping_away_from_lane_boundary;
            if (config_.enable_avoiding_collision_over_line) {
              if (0 == lane_section->lat_major) {
                right_lat_sample_range += 0.3F * param_.vehicle_width;
              }
            }
          } else {
            right_lat_sample_range -=
                param_.dist_of_keeping_away_from_road_boundary;
          }
          /* k001 pengc 2022-02-09 (end) */
        }
      }
      if ((sorted_neighbor_lanes_size-1) == neighbor_index) {
        left_lat_sample_range -=
            param_.dist_of_keeping_away_from_road_boundary;
      } else {
        if (!sorted_neighbor_lanes[neighbor_index+1].valid_lon) {
          left_lat_sample_range -=
              param_.dist_of_keeping_away_from_road_boundary;
        } else {
          /* k001 pengc 2022-02-09 (begin) */
          // 允许生成车道内避让的轨迹
          if (result_of_action_planning_.bypassing_req.allow_turning_left) {
            left_lat_sample_range -=
                param_.dist_of_keeping_away_from_lane_boundary;
            if (config_.enable_avoiding_collision_over_line) {
              if (0 == lane_section->lat_major) {
                left_lat_sample_range += 0.3F * param_.vehicle_width;
              }
            }
          } else {
            left_lat_sample_range -=
                param_.dist_of_keeping_away_from_road_boundary;
          }
          /* k001 pengc 2022-02-09 (end) */
        }
      }

      bool enable_avoiding_collision_in_lane =
          config_.enable_avoiding_collision_in_lane;
#if 1
      if (0 == lane_section->lat_major) {
        if (veh_proj_on_major_ref_line_.l > left_lat_sample_range) {
          enable_avoiding_collision_in_lane = true;
          left_lat_sample_range = veh_proj_on_major_ref_line_.l;
        } else if (veh_proj_on_major_ref_line_.l < -right_lat_sample_range) {
          enable_avoiding_collision_in_lane = true;
          right_lat_sample_range = -veh_proj_on_major_ref_line_.l;
        } else {
          // nothing to do
        }
      }
#endif

      Int32_t right_sample_num = 0;
      plan_var_t right_sample_step_len = 0;
      if (right_lat_sample_range > 0.1f) {
        if (right_lat_sample_range > lat_sample_step_len) {
          right_sample_num = common::com_round(
                right_lat_sample_range / lat_sample_step_len);
          right_sample_step_len = right_lat_sample_range / right_sample_num;
        } else {
          right_sample_num = 1;
          right_sample_step_len = right_lat_sample_range;
        }
      }

      Int32_t left_sample_num = 0;
      plan_var_t left_sample_step_len = 0;
      if (left_lat_sample_range > 0.1f) {
        if (left_lat_sample_range > lat_sample_step_len) {
          left_sample_num = common::com_round(
                left_lat_sample_range / lat_sample_step_len);
          left_sample_step_len = left_lat_sample_range / left_sample_num;
        } else {
          left_sample_num = 1;
          left_sample_step_len = left_lat_sample_range;
        }
      }

      GraphSection::GraphNode* node = lane_section->nodes.Allocate();
      if (Nullptr_t == node) {
        LOG_ERR << "Can't add graph node, storage is full.";
        break;
      }
      node->lat_minor = 0;
      node->lat_offset = 0;

      /* k002 pengc 2022-08-15 (begin) */
      /* 添加偏离车道线行驶的功能 */
      // 生成偏离车道中心线行驶的node
      GraphSection::GraphNode* node_off_center = Nullptr_t;
      if ((0 == lane_section->lat_major) &&
          (common::com_abs(driving_off_center_.expected_lat_offset) > 0.01F)) {
        // 只在当前车道中添加
        node_off_center = lane_section->nodes.Allocate();
        if (Nullptr_t == node_off_center) {
          LOG_ERR << "Can't add graph node, storage is full.";
          break;
        }
        if (driving_off_center_.expected_lat_offset > 0.0F) {
          node_off_center->lat_minor = 1;
        } else {
          node_off_center->lat_minor = -1;
        }
        node_off_center->lat_offset = driving_off_center_.expected_lat_offset;

        if (node_off_center->lat_minor > lane_section->max_lat_minor) {
          lane_section->max_lat_minor = node_off_center->lat_minor;
        }
        if (node_off_center->lat_minor < lane_section->min_lat_minor) {
          lane_section->min_lat_minor = node_off_center->lat_minor;
        }

        if (Nullptr_t == first_off_center_node) {
          first_off_center_node = node_off_center;
        }
      }
      /* k002 pengc 2022-08-15 (end) */

      /* k001 pengc 2021-07-29 (begin) */
      if (enable_avoiding_collision_in_lane) {
        // 生成车道内避障的轨迹

        // Left
        Int32_t lat_minor = 0;
        for (Int32_t i = 1; i <= left_sample_num; ++i) {
          lat_minor++;
          plan_var_t offset = left_sample_step_len * i;

          /* k002 pengc 2022-08-15 (begin) */
          /* 添加偏离车道线行驶的功能 */
          // modify lat_minor when add off-center node
          if (Nullptr_t != node_off_center) {
            if (lat_minor == node_off_center->lat_minor) {
              if (common::com_abs(offset-node_off_center->lat_offset) < 0.1F) {
                // std::cout << "a, lat_minor=" << lat_minor << std::endl;
                continue;
              } else if (offset < node_off_center->lat_offset) {
                node_off_center->lat_minor++;
                if (node_off_center->lat_minor > lane_section->max_lat_minor) {
                  lane_section->max_lat_minor = node_off_center->lat_minor;
                }
                // std::cout << "b, lat_minor=" << lat_minor << std::endl;
              } else {
                lat_minor++;
                // std::cout << "c, lat_minor=" << lat_minor << std::endl;
              }
            }
          }
          /* k002 pengc 2022-08-15 (end) */

          GraphSection::GraphNode* node = lane_section->nodes.Allocate();
          if (Nullptr_t == node) {
            LOG_ERR << "Can't add graph node, storage is full.";
            break;
          }
          node->lat_minor = lat_minor;
          node->lat_offset = offset;
          if (node->lat_minor > lane_section->max_lat_minor) {
            lane_section->max_lat_minor = node->lat_minor;
          }
        }

        // right
        lat_minor = 0;
        for (Int32_t i = 1; i <= right_sample_num; ++i) {
          lat_minor--;
          plan_var_t offset = -right_sample_step_len * i;

          /* k002 pengc 2022-08-15 (begin) */
          /* 添加偏离车道线行驶的功能 */
          // modify lat_minor when add off-center node
          if (Nullptr_t != node_off_center) {
            if (lat_minor == node_off_center->lat_minor) {
              if (common::com_abs(offset-node_off_center->lat_offset) < 0.1F) {
                // std::cout << "a, lat_minor=" << lat_minor << std::endl;
                continue;
              } else if (offset > node_off_center->lat_offset) {
                node_off_center->lat_minor--;
                if (node_off_center->lat_minor < lane_section->min_lat_minor) {
                  lane_section->min_lat_minor = node_off_center->lat_minor;
                }
                // std::cout << "b, lat_minor=" << lat_minor << std::endl;
              } else {
                lat_minor--;
                // std::cout << "c, lat_minor=" << lat_minor << std::endl;
              }
            }
          }
          /* k002 pengc 2022-08-15 (end) */

          GraphSection::GraphNode* node = lane_section->nodes.Allocate();
          if (Nullptr_t == node) {
            LOG_ERR << "Can't add graph node, storage is full.";
            break;
          }

          node->lat_minor = lat_minor;
          node->lat_offset = offset;
          if (node->lat_minor < lane_section->min_lat_minor) {
            lane_section->min_lat_minor = node->lat_minor;
          }
        }
      }
      /* k001 pengc 2021-07-29 (end) */
    }
  }

  /* k002 pengc 2022-08-15 (begin) */
  /* 添加偏离车道线行驶的功能 */
  if ((Nullptr_t != lka_lane_section) && (Nullptr_t != lka_node) &&
      (Nullptr_t != first_off_center_node)) {
    lka_node->lat_minor = first_off_center_node->lat_minor;
    lka_node->lat_offset = first_off_center_node->lat_offset;

    if (lka_node->lat_minor > lka_lane_section->max_lat_minor) {
      lka_lane_section->max_lat_minor = lka_node->lat_minor;
    }
    if (lka_node->lat_minor < lka_lane_section->min_lat_minor) {
      lka_lane_section->min_lat_minor = lka_node->lat_minor;
    }
    // std::cout << "lka node: "
    //           << "lat_major=" << lka_lane_section->lat_major
    //           << ", lat_minor=" << lka_node->lat_minor
    //           << ", lat_offset=" << lka_node->lat_offset
    //           << std::endl;
  }
  /* k002 pengc 2022-08-15 (end) */

  return true;
}

bool TrajectoryPlanningStateLattice::CreateGraphLinks_Type1() {
  Int32_t road_graph_section_size = road_graph_.Size();

  for (Int32_t section_index = 0; section_index < (road_graph_section_size-1);
       ++section_index) {
    if (road_graph_link_storage_.Full()) {
      LOG_ERR << "Can't add link to road graph, storage is full.";
      break;
    }

    GraphSection& graph_section = road_graph_[section_index];

    Int32_t lon_major = graph_section.lon_major;
    if (0 == lon_major) {
      if (0 == graph_section.lon_minor) {
        Int32_t next_section_index = FindGraphSection(section_index, 0, -1);
        if (next_section_index > section_index) {
          ConstructParallelLinksBetweenGraphSections(
                false, section_index, next_section_index);
        }
        next_section_index = FindGraphSection(section_index, 1, 0);
        if (next_section_index > section_index) {
          ConstructFullLinksBetweenGraphSections(
                section_index, next_section_index);
        }
        next_section_index = FindGraphSection(section_index, 2, -3);
        if (next_section_index > section_index) {
          ConstructFullLinksBetweenGraphSections(
                section_index, next_section_index);
        }
        next_section_index = FindGraphSection(section_index, 2, -2);
        if (next_section_index > section_index) {
          ConstructFullLinksBetweenGraphSections(
                section_index, next_section_index);
        }
        next_section_index = FindGraphSection(section_index, 2, -1);
        if (next_section_index > section_index) {
          ConstructFullLinksBetweenGraphSections(
                section_index, next_section_index);
        }
        next_section_index = FindGraphSection(section_index, 2, 0);
        if (next_section_index > section_index) {
          ConstructFullLinksBetweenGraphSections(
                section_index, next_section_index);
        }
      }
      if (-1 == graph_section.lon_minor) {
        Int32_t next_section_index = FindGraphSection(section_index, 1, 0);
        if (next_section_index > section_index) {
          ConstructParallelLinksBetweenGraphSections(
                false, section_index, next_section_index);
        }
      }
    } else if (1 == lon_major) {
      Int32_t next_section_index = FindGraphSection(section_index, 2, 0);
      if (next_section_index > section_index) {
        ConstructParallelLinksBetweenGraphSections(
              true, section_index, next_section_index);
      }
    } else if (2 == lon_major) {
      if (graph_section.lon_minor < 0) {
        Int32_t next_section_index = FindGraphSection(section_index, 2, 0);
        if (next_section_index > section_index) {
          ConstructParallelLinksBetweenGraphSections(
                true, section_index, next_section_index);
        }
      } else {
        Int32_t next_section_index = FindGraphSection(section_index, 3, 0);
        if (next_section_index > section_index) {
          ConstructParallelLinksBetweenGraphSections(
                true, section_index, next_section_index);
        }
      }
    } else {
      Int32_t next_section_index = FindGraphSection(
            section_index, lon_major+1, 0);
      if (next_section_index > section_index) {
        ConstructParallelLinksBetweenGraphSections(
              true, section_index, next_section_index);
      }
    }
  }

  return true;
}

bool TrajectoryPlanningStateLattice::CreateLonGraphSamples_Type2() {
  // Clear road graph
  for (Int32_t i = 0; i < road_graph_link_storage_.Size(); ++i) {
    road_graph_link_storage_[i].Clear();
  }
  road_graph_link_storage_.Clear();
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    road_graph_[i].Clear();
  }
  road_graph_.Clear();
  lon_graph_samples_step_len_.Clear();

  // Get the first forward risky obstacle information in current reference line.
  // Which is used to shrink sample length.
  Int32_t first_forward_risky_obj_index = -1;
  plan_var_t first_forward_risky_obj_dist = 999999;
  plan_var_t min_first_forward_segment_len =
      common::Max(3.0F * curr_position_.v, param_.vehicle_length+3.0F);
  driv_map::CollisionTestResult coll_test_ret;
  driving_map_->GetRiskyObstacles(major_ref_line_index_, false, &coll_test_ret);
  for (Int32_t i = 0; i < coll_test_ret.risky_obj_list.Size(); ++i) {
    const driv_map::CollisionTestResult::ObjInfo& collision_info = coll_test_ret.risky_obj_list[i];
    if (collision_info.dynamic_distance < 1.0f) {
      if ((min_first_forward_segment_len < collision_info.collision_s) &&
          (collision_info.collision_s < first_forward_risky_obj_dist)) {
        first_forward_risky_obj_dist = collision_info.collision_s;
        first_forward_risky_obj_index = i;
      }
    }
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "first_forward_risky_obj_index = "
            << first_forward_risky_obj_index << std::endl;
  std::cout << "first_forward_risky_obj_dist = "
            << first_forward_risky_obj_dist << std::endl;
#endif

  GraphSection* sample = road_graph_.Allocate();
  if (Nullptr_t == sample) {
    LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
    return false;
  }
  sample->lon_major = 0;
  sample->lon_minor = 0;
  sample->proj_on_ref = veh_proj_on_major_ref_line_;
  driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
        major_ref_line_index_, sample->proj_on_ref.s,
        &(sample->lane_index_on_ref));
  if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
    road_graph_.PopBack();
    LOG_ERR << "Can't find valid reference line.";
    return false;
  }

  plan_var_t start_s = veh_proj_on_major_ref_line_.s;
  plan_var_t step_len = 0;
  for (Int32_t index = 1; index < MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM; ++index) {
    // Limit sampled step length by curvature of trajectory
    step_len = CalcStepLenOfLonGraphSample(start_s);

    // Limit sampled step length by obstacles
    if (1 == index) {
      // Only limit the first sampled step length
      if (first_forward_risky_obj_index >= 0) {
        if (first_forward_risky_obj_dist < step_len) {
          step_len = first_forward_risky_obj_dist;
        }
      }
    }

    // Limit min and max sampled step length
    if ((index-1) < param_.lon_sample_step_len_limit_table.Size()) {
      plan_var_t min_step_len =
          param_.lon_sample_step_len_limit_table[index-1].first;
      plan_var_t max_step_len =
          param_.lon_sample_step_len_limit_table[index-1].second;
      if (step_len < min_step_len) {
        step_len = min_step_len;
      }
      if (step_len > max_step_len) {
        step_len = max_step_len;
      }
    }

    // Limit the total sampled length less than total reference lines length
    // and the looking forward length.
    plan_var_t s_ref = start_s + step_len;
    if ((s_ref - veh_proj_on_major_ref_line_.s) > look_forward_length_) {
      s_ref = veh_proj_on_major_ref_line_.s + look_forward_length_;
      if ((s_ref - start_s) < 5.0f) {
        s_ref = start_s + 5.0f;
      }
    }
    if (major_ref_line_->total_length() > 15.0F) {
      if (s_ref > (major_ref_line_->total_length()-5.0F)) {
        s_ref = major_ref_line_->total_length()-5.0F;
      }
    } else {
      if (s_ref > (major_ref_line_->total_length()-0.5F)) {
        s_ref = major_ref_line_->total_length()-0.5F;
      }
    }
    step_len = s_ref - start_s;
    if (step_len < 2.0f) {
      break;
    }

    // Save sampled step length
    lon_graph_samples_step_len_.PushBack(step_len);

    // Get leading point on reference line for LKA
    common::PathPoint point_on_ref;
    if (1 == index) {
      if (leading_length_for_lka_ < (step_len - 2.0f)) {
        major_ref_line_->FindSmoothPoint(
              start_s+leading_length_for_lka_, &point_on_ref);
        sample = road_graph_.Allocate();
        if (Nullptr_t == sample) {
          LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
          return false;
        }
        sample->lon_major = 0;
        sample->lon_minor = -1;
        sample->proj_on_ref = point_on_ref;
        driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
              major_ref_line_index_, sample->proj_on_ref.s,
              &(sample->lane_index_on_ref));
        if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
          road_graph_.PopBack();
        }
      }
    }

    // Get major point on reference line
    major_ref_line_->FindSmoothPoint(s_ref, &point_on_ref);
    sample = road_graph_.Allocate();
    if (Nullptr_t == sample) {
      LOG_ERR << "Can't add longitudinal sampled point, storage is full.";
      return false;
    }
    sample->lon_major = index;
    sample->lon_minor = 0;
    sample->proj_on_ref = point_on_ref;
    driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
          major_ref_line_index_, sample->proj_on_ref.s,
          &(sample->lane_index_on_ref));
    if (!driving_map_->IsValidLaneIndex(sample->lane_index_on_ref)) {
      road_graph_.PopBack();
    }

    if ((s_ref - veh_proj_on_major_ref_line_.s) > (look_forward_length_-1.0f)) {
      break;
    }

    // Update loop variables
    start_s = s_ref;
  }

  if (road_graph_.Size() < 2) {
    LOG_ERR << "There are not enough longitudinal sampled points.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "The longitudinal graph sampled lengths are (Type II):"
            << std::endl;
  for (Int32_t i = 0; i < lon_graph_samples_step_len_.Size(); ++i) {
    std::cout << " " << lon_graph_samples_step_len_[i] << ",";
  }
  std::cout << " {end}" << std::endl;
  std::cout << "The longitudinal graph samples are: " << std::endl;
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    std::cout << "[" << i
              << "], lon_major=" << road_graph_[i].lon_major
              << ", lon_minor=" << road_graph_[i].lon_minor
              << ", s_ref=" << road_graph_[i].proj_on_ref.s
              << ", lane_index=" << road_graph_[i].lane_index_on_ref
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
              << ", lane_id=" << driving_map_->GetLaneIdByIndex(
                   road_graph_[i].lane_index_on_ref).id.c_str()
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
              << ", lane_id=" << driving_map_->GetLaneIdByIndex(
                   road_graph_[i].lane_index_on_ref).id
#endif
              << std::endl;
  }
#endif

  return true;
}

bool TrajectoryPlanningStateLattice::CreatGraphNodes_Type2() {
  Int32_t road_graph_section_size = road_graph_.Size();

  for (Int32_t section_index = 0; section_index < road_graph_section_size;
       ++section_index) {
    GraphSection& graph_section = road_graph_[section_index];
    graph_section.max_lat_major = 0;
    graph_section.min_lat_major = 0;

    const common::Path& lane_curve = driving_map_->GetLaneCentralCurve(
          graph_section.lane_index_on_ref);
    common::PathPoint proj_on_lane;
    lane_curve.FindProjection(graph_section.proj_on_ref.point, &proj_on_lane);

    GraphSection::LaneSection* lane_section =
        graph_section.lane_sections.Allocate();
    if (Nullptr_t == lane_section) {
      LOG_ERR << "Can't add lane section to graph, storage is full.";
      break;
    }
    lane_section->lat_major = 0;
    lane_section->lane_index = graph_section.lane_index_on_ref;
    driving_map_->GetLaneWidth(
          lane_section->lane_index, proj_on_lane.s,
          &(lane_section->left_width), &(lane_section->right_width));
    lane_section->proj_on_lane = proj_on_lane;
    lane_section->max_lat_minor = 0;
    lane_section->min_lat_minor = 0;

    GraphSection::GraphNode* node = lane_section->nodes.Allocate();
    if (Nullptr_t == node) {
      LOG_ERR << "Can't add graph node, storage is full.";
      break;
    }
    node->lat_minor = 0;

    if ((0 == graph_section.lon_major) && (0 == graph_section.lon_minor)) {
      node->lat_offset = graph_section.proj_on_ref.l;
    } else {
      node->lat_offset = 0;
    }
  }
  return true;
}

bool TrajectoryPlanningStateLattice::CreateGraphLinks_Type2() {
  Int32_t road_graph_section_size = road_graph_.Size();
  Int32_t section_index = 0;
  GraphSection& graph_section = road_graph_[section_index];
  // 获取车离当前参考线的偏移距离
  plan_var_t lat_offsets = common::com_abs(
        graph_section.lane_sections[0].nodes[0].lat_offset);

  for ( ; section_index < road_graph_section_size -1; ++section_index ) {
    if (road_graph_link_storage_.Full()) {
      LOG_ERR << "Can't add link to road graph, storage is full.";
      break;
    }
    // 根据车当前横向偏移的距离查表选择合适的第一个节点来link
    if (section_index == 0) {
      //  横向偏移距离在第一个范围内，直接link(0,1)路网
      if (lat_offsets < param_.range_of_lat_offset_table[0]) {
        section_index = 1;
      } else if (lat_offsets < param_.range_of_lat_offset_table[1]) {
        Int32_t next_section_index = FindGraphSection(
              section_index, 1, 0);
        if (next_section_index > section_index) {
          // 横向偏移距离在第二个范围和第一范围之间内且(1,0)路网存在
          // 选择link的第一个路网为(1,0)
          section_index = next_section_index;
        } else {
          // 横向偏移距离在第二个范围和第一范围之间但(1,0)路网不存在
          // 选择link的第一个路网为(0,1)
          section_index = 1;
        }
      } else {
        Int32_t next_section_index = FindGraphSection(
              section_index, 2, 0);
        if (next_section_index > section_index) {
          // 横向偏移距离在大于第二个范围且(2,0)路网存在，则link(2,0)
          // 选择link的第一个路网为(2,0)
          section_index = next_section_index;
        } else {
          next_section_index = FindGraphSection(
                section_index, 1, 0);
          if (next_section_index > section_index) {
            // 横向偏移距离在大于第二个范围且(2,0)路网不存在，(1,0)路网存在
            // 选择link的第一个路网为(1,0)
            section_index = next_section_index;
          } else {
            // 横向偏移距离在大于第二个范围且(2,0)路网不存在，(1,0)路网不存在
            // 选择link的第一个路网为(0,1)
            section_index = 1;
          }
        }
      }
      // link车和第一个路网
      ConstructFullLinksBetweenGraphSections(0, section_index);
    }
    if (section_index == road_graph_section_size -1) {
      // 所有路网已link
      break;
    } else {
      // link当前路网和下个路网
      Int32_t next_section_index = section_index +1;
      ConstructFullLinksBetweenGraphSections(section_index, next_section_index);
    }
  }

  return true;
}


} // namespace planning
} // namespace phoenix
