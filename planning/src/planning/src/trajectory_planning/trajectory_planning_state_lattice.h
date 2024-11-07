/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       trajectory_planning_state_lattice.h
 * @brief      轨迹规划（状态格）
 * @details    使用状态格算法，规划局部行驶轨迹
 *
 * @author     pengc
 * @date       2020.07.10
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/07/10  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_PLANNING_TRAJECTORY_PLANNING_STATE_LATTICE_H_
#define PHOENIX_PLANNING_TRAJECTORY_PLANNING_STATE_LATTICE_H_


#include "container/static_vector.h"
#include "container/data_pair.h"
#include "geometry/aabboxkdtree2d.h"
#include "utils/linear_interpolation.h"
#include "utils/com_timer.h"
#include "curve/quintic_polynomial_curve1d.h"
#include "vehicle_model_wrapper.h"
#include "motion_planning.h"
#include "driving_map_wrapper.h"


#define ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_DEBUGGING_FILE (0)


namespace phoenix {
namespace planning {


/**
 * @struct TrajectoryPlanningStateLattice
 * @brief 轨迹规划（状态格）
 */
class TrajectoryPlanningStateLattice {
public:
  /**
   * @brief 构造函数
   */
  TrajectoryPlanningStateLattice();

  /**
   * @brief 析构函数
   */
  ~TrajectoryPlanningStateLattice();

  /**
   * @brief 设置模块参数为默认参数
   */
  void SetDefaultParam();

  /**
   * @brief 配置模块
   */
  void Configurate(const TrajectoryPlanningConfig& conf);

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 规划局部轨迹
   * @return true - 成功，false - 失败
   */
  bool Plan(const TrajectoryPlanningDataSource& data_source);

  /**
   * @brief 获取规划的结果
   * @return 规划的结果
   */
  inline const TrajectoryPlanningResult& GetResultOfPlanning() const {
    return (result_of_planning_);
  }

  /**
   * @brief 获取轨迹规划的内部信息（调试用）
   * @param[out] trj_planning_info 轨迹规划的内部信息
   */
  void GetTrajectoryPlanningInfo(
      TrajectoryPlanningInfo* trj_planning_info) const;

  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

private:
  // 最大的路网纵向段的数量
  enum { MAX_GRAPH_SECTION_NUM = 20 };
  // 每个路网段中最大车道段的数量(一般设置为最大相邻车道的数量)
  enum { MAX_LANE_SECTION_NUM_PER_GRAPH_SECTION =
         driv_map::MAX_NEIGHBOR_LANE_NUM };
  // 路网中每个车道段中最大节点的数量(每条路网车道段中最多的横向采样点的数量)
  enum { MAX_NODE_NUM_PER_LANE_SECTION = 19 };
  // link 中最大的有风险障碍物的数量
  enum { MAX_RISKY_OBJ_IN_LINK = 8 };

  enum {
    DRIVING_DIRECTION_FORWARD = 0,
    DRIVING_DIRECTION_BACKWARD
  };

  enum {
    TRJ_REQ_INVALID = 0,
    TRJ_REQ_LKA,
    TRJ_REQ_LKA_BYPASSING_LEFT,
    TRJ_REQ_LKA_BYPASSING_RIGHT,
    TRJ_REQ_CHANGING_LANE_LEFT,
    TRJ_REQ_CHANGING_LANE_RIGHT
  };

  enum {
    EVENT_TYPE_START_CHANGING_LANE_LEFT,
    EVENT_TYPE_START_CHANGING_LANE_RIGHT,
    EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT,
    EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT,
    EVENT_TYPE_ABORT_CHANGING_LANE_LEFT,
    EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT,
    EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT,
    EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT,
    EVENT_TYPE_REFUSE_ABORT_CHANGING_LANE
  };

  struct ActionManager{
    ChangingLaneReq changing_lane_req;
    ChangingLaneRsp changing_lane_rsp;

    struct {
      Int32_t req_count;
      ChangingLaneReq req;

      void Clear() {
        req_count = 0;
        req.set_request(ChangingLaneReq::REQ_CHANGING_LANE_NONE);
      }
    } req_changing_lane_automatically;

    void Clear() {
      changing_lane_req.Clear();
      changing_lane_rsp.Clear();
      req_changing_lane_automatically.Clear();
    }
  };

  // 障碍物分类信息
  struct ObjClassifyingInfo {
    // 障碍物方位
    enum {
      OBJ_POSITION_FRONT = 0,
      OBJ_POSITION_SIDE,
      OBJ_POSITION_BACK
    };
    // 障碍物方位
    Int32_t obj_position;
    // 障碍物移动方向(forward, backward, crossed)
    Int32_t obj_direction;
    // 离障碍物的距离
    plan_var_t distance;
    // 时距
    plan_var_t t_gap;
    // 碰撞时间
    plan_var_t ttc;

    ObjClassifyingInfo() {
      Clear();
    }

    void Clear() {
      obj_position = OBJ_POSITION_FRONT;
      obj_direction = driv_map::OBJ_DIRECTION_UNKNOWN;
      distance = 0.0F;
      t_gap = 0.0F;
      ttc = 0.0F;
    }
  };

  struct CollisionInfo {
    plan_var_t deceleration;
    driv_map::CollisionTestResult::ObjInfo test_ret;
  };

  struct GraphObstacleInfo {
    // 轨迹关于障碍物的风险值（碰撞分析的结果）
    Int32_t risk_value;
    // 轨迹上有风险的障碍物在当前参考线上相对当前车辆位置的距离（碰撞分析的结果）
    plan_var_t lon_dist_on_major_ref;
    // 碰撞风险测试时，轨迹上有风险的障碍物与车身之间的碰撞距离(OBB之间的距离)
    plan_var_t lat_dist;
    // 根据障碍物的碰撞风险测试结果规划的减速度，用来轨迹的cost
    plan_var_t deceleration;

    void Clear() {
      risk_value = 0;
      lon_dist_on_major_ref = 0.0F;
      lat_dist = 0.0F;
      deceleration = 0.0F;
    }

    GraphObstacleInfo() {
      Clear();
    }
  };

  /*
   * @struct GraphSection
   * @brief 路网的纵向段信息
   */
  struct GraphSection {
    // 路网纵向节点主编号
    Int32_t lon_major;
    // 路网纵向节点副编号
    Int32_t lon_minor;
    // 在当前参考线上的采样点对应的车道索引
    Int32_t lane_index_on_ref;
    // 在当前参考线上的纵向采样点在当前参考线上的投影点
    common::PathPoint proj_on_ref;

    /*
     * @struct GraphSection::GraphNode
     * @brief 路网的节点信息
     */
    struct GraphNode {
      // 路网节点横向副编号(道路中心线上的采样点此编号为0;
      // 在中心线右边，编号为负，越靠右，编号绝对值越大;
      // 在中心线左边，编号为正，越靠左，编号绝对值越大)
      Int32_t lat_minor;
      // 节点在道路中心线上的横向偏移(左正右负)
      plan_var_t lat_offset;

      /*
       * @struct GraphSection::GraphNode::Link
       * @brief 路网节点的link信息
       */
      struct Link {
        // link的存储索引
        Int32_t link_index;
        // link对应的参考线索引
        Int32_t ref_line_index;
 
        /*
         * @struct GraphSection::GraphNode::Link::Node
         * @brief link的起始/目标节点信息
         */
        struct Node {
          // 路网的纵向段索引
          Int32_t graph_section_index;
          // 路网的道路段索引
          Int32_t lane_section_index;
          // 路网的节点索引
          Int32_t node_index;

          // link的起始/目标节点的航向角(采样点)
          plan_var_t heading;
          // link的起始/目标节点的曲率(采样点)
          plan_var_t curvature;

          // link的起始/目标节点在参考线上的路径长(采样点)
          plan_var_t s;
          // link的起始/目标节点在参考线上的横向偏移(采样点)
          plan_var_t l;
          // link的起始/目标节点在参考线上的横向偏移的一阶导数(采样点)
          plan_var_t dl;
          // link的起始/目标节点在参考线上的横向偏移的二阶导数(采样点)
          plan_var_t ddl;
          // link的起始/目标节点在参考线上的投影点(采样点)
          common::PathPoint proj_on_ref;

          /*
           * @brief 清除内部数据(link的起始/目标节点信息)
           */
          void Clear() {
            graph_section_index = -1;
            lane_section_index = -1;
            node_index = -1;

            heading = 0;
            curvature = 0;

            s = 0;
            l = 0;
            dl = 0;
            ddl = 0;

            proj_on_ref.Clear();
          }
        } node_from, node_to;

        // link对应的曲线方程
        common::QuinticPolynomialCurve1d<plan_var_t> curve;
        // link对应的曲线方程的采样点
        common::StaticVector<common::PathPoint,
        common::Path::kMaxPathPointNum> sample_points;

        // link曲线的曲率
        plan_var_t abs_curvature;
        // 根据link曲线曲率规划的减速度，用来计算link的cost
        plan_var_t deceleration_by_curvature;
        // 此link曲线关于障碍物的风险（碰撞分析的结果）
        GraphObstacleInfo collision_obj;
        common::StaticVector<CollisionInfo, MAX_RISKY_OBJ_IN_LINK> risky_obj_list;
        common::StaticVector<CollisionInfo, MAX_RISKY_OBJ_IN_LINK> uncertain_list;
        /* k005 pengc 2023-01-06 (begin) */
        // 添加功能: 道路边界碰撞测试
        // 是否与道路边界有干涉
        bool is_collision_with_road_boundary;
        /* k005 pengc 2023-01-06 (end) */

        // Link的Cost
        struct {
          // 车道对应的Cost
          Int32_t lane_cost;
          // 车道内与中心线之间的横向偏差对应的Cost
          Int32_t lateral_offset_cost;
          // link曲线的曲率对应的Cost
          Int32_t curvature_cost;
          // 与障碍物之间的碰撞风险对应的Cost
          Int32_t collision_cost;

          /*
           * @brief 清除内部数据(link的Cost)
           */
          void Clear() {
            lane_cost = 0;
            lateral_offset_cost = 0;
            curvature_cost = 0;
            collision_cost = 0;
          }
        } cost;

        /*
         * @brief 清除内部数据(路网节点的link信息)
         */
        void Clear() {
          link_index = -1;
          ref_line_index = -1;
          node_from.Clear();
          node_to.Clear();
          curve.Clear();
          sample_points.Clear();
          abs_curvature = 0;
          deceleration_by_curvature = 0;
          collision_obj.Clear();
          risky_obj_list.Clear();
          uncertain_list.Clear();
          is_collision_with_road_boundary = false;

          cost.Clear();
        }
      };

      // 此路网节点中包含的link的数量
      Int32_t link_num;
      // 此路网节点中包含的link的链表
      // 注意：为了提高效率并减小存储空间，此双向链表不是足够安全的，
      // 请参考其使用说明，并小心使用。
      common::internal::DoubleList link_list;

      /*
       * @brief 清除内部数据(路网的纵向段信息)
       */
      void Clear() {
        lat_minor = 0;
        lat_offset = 0;

        link_num = 0;
        link_list.Clear();
      }
    };

    /*
     * @struct GraphSection::LaneSection
     * @brief 路网的车道段信息(每个路网段都包含一系列左右相邻的车道，\n
     *        这样的每条车道称为路网的车道段)
     */
    struct LaneSection {
      // 路网节点的横向主编号(位于当前参考线上的车道的节点的编号为0;
      // 在当前参考线右边的，编号为负，越靠右，编号绝对值越大;
      // 在当前参考线左边的，编号为正，越靠左，编号绝对值越大)
      Int32_t lat_major;
      // 此路网的车道段对应的车道索引
      Int32_t lane_index;
      // 此路网的车道段对应的车道在此节点处对应的车道的左边宽度
      plan_var_t left_width;
      // 此路网的车道段对应的车道在此节点处对应的车道的右边宽度
      plan_var_t right_width;
      // 此路网的车道段对应的车道在此节点处对应的在车道中心线上的投影点
      common::PathPoint proj_on_lane;

      // 此路网的车道段包含的最大横向副编号(对车道横向采样,最左侧的采样点的横向副编号)
      Int32_t max_lat_minor;
      // 此路网的车道段包含的最小横向副编号(对车道横向采样,最右侧的采样点的横向副编号)
      Int32_t min_lat_minor;
      // 此路网的车道段包含的节点列表
      common::StaticVector<GraphNode, MAX_NODE_NUM_PER_LANE_SECTION> nodes;

      /*
       * @brief 清除内部数据(路网的道路段信息)
       */
      void Clear() {
        lat_major = 0;
        lane_index = -1;
        left_width = 0;
        right_width = 0;
        proj_on_lane.Clear();

        max_lat_minor = 0;
        min_lat_minor = 0;
        for (Int32_t i = 0; i < nodes.Size(); ++i) {
          nodes[i].Clear();
        }
        nodes.Clear();
      }
    };

    // 路网的纵向段包含的最大横向主编号(最左侧的道路段的横向主编号)
    Int32_t max_lat_major;
    // 路网的纵向段包含的最大横向主编号(最右侧的道路段的横向主编号)
    Int32_t min_lat_major;
    // 路网的纵向段包含的车道段的列表
    common::StaticVector<LaneSection,
    MAX_LANE_SECTION_NUM_PER_GRAPH_SECTION> lane_sections;

    /*
     * @brief 清除内部数据(路网的道路段信息)
     */
    void Clear() {
      lon_major = 0;
      lon_minor = 0;
      lane_index_on_ref = -1;
      proj_on_ref.Clear();

      max_lat_major = 0;
      min_lat_major = 0;
      for (Int32_t i = 0; i < lane_sections.Size(); ++i) {
        lane_sections[i].Clear();
      }
      lane_sections.Clear();
    }
  };

  /*
   * @struct GraphNodeInfoForConstruction
   * @brief 用来生成路网节点link的信息
   */
  struct GraphNodeInfoForConstruction {
    // link对应的参考线索引
    Int32_t ref_line_index;
    // link起始节点的路网的纵向段索引
    Int32_t src_graph_sec_index;
    // link目标节点的路网的纵向段索引
    Int32_t dest_graph_sec_index;
    // link起始节点的路网的纵向段中车道段索引
    Int32_t src_lane_sec_index;
    // link目标节点的路网的纵向段中车道段索引
    Int32_t dest_lane_sec_index;
    // link起始节点的路网的纵向段中车道段中节点索引
    Int32_t src_graph_node_index;
    // link目标节点的路网的纵向段中车道段中节点索引
    Int32_t dest_graph_node_index;
    // 起始节点在link对应的参考线上的投影点
    common::PathPoint src_node_proj_on_ref;
    // 目标节点在link对应的参考线上的投影点
    common::PathPoint dest_node_proj_on_ref;

    /*
     * @brief 构造函数
     */
    GraphNodeInfoForConstruction() {
      ref_line_index = -1;
      src_graph_sec_index = -1;
      dest_graph_sec_index = -1;
      src_lane_sec_index = -1;
      dest_lane_sec_index = -1;
      src_graph_node_index = -1;
      dest_graph_node_index = -1;
    }
  };

  /*
   * @struct GraphLinkListNode
   * @brief 用来存储路网中link信息的链表的节点信息
   */
  struct GraphLinkListNode {
    // 下一个链表节点的索引
    Int32_t next;
    // 上一个链表节点的索引
    Int32_t prev;
    // 路网中link信息
    GraphSection::GraphNode::Link link;

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      next = -1;
      prev = -1;
      link.Clear();
    }
  };

  /*
   * @struct CandidateTrajectory
   * @brief 定义候选轨迹
   */
  struct CandidateTrajectory {
    /*
     * @struct CandidateTrajectory::TrjNode
     * @brief 定义候选轨迹段的节点信息
     */
    struct TrjNode {
      // 路网的纵向section索引
      Int32_t graph_section_index;
      // 路网的横向车道section索引
      Int32_t lane_section_index;
      // 路网的节点索引
      Int32_t node_index;
      // 路网的link索引
      Int32_t link_index;

      /*
       * @brief 清除内部数据(候选轨迹段的节点信息)
       */
      void Clear() {
        graph_section_index = -1;
        lane_section_index = -1;
        node_index = -1;
        link_index = -1;
      }

      /*
       * @brief 构造函数(候选轨迹段的节点信息)
       */
      TrjNode() {
        Clear();
      }
    };

    // 轨迹段列表
    common::StaticVector<TrjNode, MAX_GRAPH_SECTION_NUM> trajectory_segments;

    // 候选轨迹的的Cost
    struct {
      // 车道对应的Cost
      Int32_t lane_cost;
      // 车道内与中心线之间的横向偏差对应的Cost
      Int32_t lateral_offset_cost;
      //
      Int32_t lka_cost;
      // 候选轨迹的曲率对应的Cost
      Int32_t curvature_cost;
      // 与障碍物之间的碰撞风险对应的Cost
      Int32_t collision_cost;
      // 额外的车道内避让的cost, 不计入总的cost
      Int32_t ext_bypassing_cost;
      // 候选轨迹长度对应的Cost
      Int32_t trj_len_cost;
      // 是否与道路边界有干涉Cost
      Int32_t collision_with_road_boundary_cost;
      // Cost的合计
      Int32_t total_cost;

      /*
       * @brief 清除内部数据(轨迹的Cost)
       */
      void Clear() {
        lane_cost = 0;
        lateral_offset_cost = 0;
        lka_cost = 0;
        curvature_cost = 0;
        collision_cost = 0;
        ext_bypassing_cost = 0;

        trj_len_cost = 0;
        collision_with_road_boundary_cost = 0;

        total_cost = 0;
      }
    } cost;

    // 候选轨迹的曲率
    plan_var_t abs_curvature;
    // 根据候选轨迹曲率规划的减速度
    plan_var_t deceleration_by_curvature;
    // 障碍物的碰撞风险测试结果
    GraphObstacleInfo collision_obj;
    common::StaticVector<CollisionInfo, MAX_RISKY_OBJ_IN_LINK> risky_obj_list;
    common::StaticVector<CollisionInfo, MAX_RISKY_OBJ_IN_LINK> uncertain_list;
    /* k005 pengc 2023-01-06 (begin) */
    // 添加功能: 道路边界碰撞测试
    // 是否与道路边界有干涉
    bool is_collision_with_road_boundary;
    /* k005 pengc 2023-01-06 (end) */

    /*
     * @brief 清除内部数据(候选轨迹)
     */
    void Clear() {
      for (Int32_t i = 0; i < trajectory_segments.Size(); ++i) {
        trajectory_segments[i].Clear();
      }
      trajectory_segments.Clear();

      cost.Clear();

      abs_curvature = 0.0F;
      deceleration_by_curvature = 0.0F;
      collision_obj.Clear();
      risky_obj_list.Clear();
      uncertain_list.Clear();
      is_collision_with_road_boundary = false;
    }

    /*
     * @brief 构造函数(候选轨迹)
     */
    CandidateTrajectory() {
      Clear();
    }
  };


  class PathTrendMgr {
  public:
    PathTrendMgr() {
      max_trend_ = 6.0F;
      positive_ = 0.0F;
      negative_ = 0.0F;
      laziness_ = 3.0F;
      trend_move_velocity_ = 1.0F;
      trend_stop_velocity_ = 1.0F;

      leading_point_move_velocity_ = 0.0F;
      leading_point_move_acceleration_ = 0.01F;
      leading_point_stop_acceleration_ = 0.01F;
      leading_point_move_max_velocity_ = 0.2F;
    }

    inline void SetLaziness(plan_var_t value) { laziness_ = value; }
    inline void SetMaxTrend(plan_var_t value) { max_trend_ = value; }
    inline void SetTrendVelocity(plan_var_t move, plan_var_t stop) {
      trend_move_velocity_ = move;
      trend_stop_velocity_ = stop;
    }
    inline void SetLeadingPointMoveMaxVelocity(plan_var_t v) {
      leading_point_move_max_velocity_ = v;
    }
    inline void SetLeadingPointMoveAcceleration(
        plan_var_t move, plan_var_t stop) {
      leading_point_move_acceleration_ = move;
      leading_point_stop_acceleration_ = stop;
    }

    inline void Reset() {
      positive_ = 0.0F;
      negative_ = 0.0F;
      leading_point_move_velocity_ = 0;
    }

    void ClearPositive() {
      if (positive_ > 0.00001F) {
        positive_ = 0.0F;
        leading_point_move_velocity_ = 0.0F;
      }
    }

    void ClearNegative() {
      if (negative_ > 0.00001F) {
        negative_ = 0.0F;
        leading_point_move_velocity_ = 0.0F;
      }
    }

    void AddPositive() {
      if (negative_ > 0.00001F) {
        negative_ -= trend_stop_velocity_;
        if (negative_ < 0.00001F) {
          negative_ = 0.0F;
          leading_point_move_velocity_ = 0.0F;
        }
      } else {
        positive_ += trend_move_velocity_;
        if (positive_ > max_trend_) {
          positive_ = max_trend_;
        }
      }
    }
    void AddNegative() {
      if (positive_ > 0.00001F) {
        positive_ -= trend_stop_velocity_;
        if (positive_ < 0.00001F) {
          positive_ = 0.0F;
          leading_point_move_velocity_ = 0.0F;
        }
      } else {
        negative_ += trend_move_velocity_;
        if (negative_ > max_trend_) {
          negative_ = max_trend_;
        }
      }
    }
    void AddLaziness() {
      negative_ -= trend_stop_velocity_;
      if (negative_ < 0.00001F) {
        negative_ = 0.0F;
        leading_point_move_velocity_ = 0.0F;
      }
      positive_ -= trend_stop_velocity_;
      if (positive_ < 0.00001F) {
        positive_ = 0.0F;
        leading_point_move_velocity_ = 0.0F;
      }
    }

    inline plan_var_t GetPositiveTrend() const {
      plan_var_t trend = positive_ - laziness_;
      if (trend < 0.00001F) {
        return (0.0F);
      }
      return (trend);
    }
    inline plan_var_t GetNegativeTrend() const {
      plan_var_t trend = negative_ - laziness_;
      if (trend < 0.00001F) {
        return (0.0F);
      }
      return (trend);
    }

    plan_var_t CalcMoveDistance(plan_var_t goal_dist) {
      plan_var_t dist = 0;
      plan_var_t t_e = common::com_abs(leading_point_move_velocity_) /
          leading_point_stop_acceleration_;
      plan_var_t s_e = (common::com_abs(leading_point_move_velocity_) -
                    0.5F*leading_point_stop_acceleration_*t_e)*t_e;
      if (GetPositiveTrend() > 0.0F) {
        if (leading_point_move_velocity_ < 0.0F) {
          leading_point_move_velocity_ = 0.0F;
        }
        if (goal_dist >= 0.0F) {
          if (goal_dist < s_e) {
            leading_point_move_velocity_ -= leading_point_stop_acceleration_;
            if (leading_point_move_velocity_ < 0.001F) {
              leading_point_move_velocity_ = 0.001F;
            }
          } else {
            leading_point_move_velocity_ += leading_point_move_acceleration_;
            if (leading_point_move_velocity_ >
                leading_point_move_max_velocity_) {
              leading_point_move_velocity_ = leading_point_move_max_velocity_;
            }
          }
          if (leading_point_move_velocity_ > goal_dist) {
            leading_point_move_velocity_ = goal_dist;
          }
          dist = leading_point_move_velocity_;
        } else {
          leading_point_move_velocity_ -= leading_point_move_acceleration_;
          if (leading_point_move_velocity_ < 0) {
            leading_point_move_velocity_ = 0;
          }
          dist = 0;
        }
      } else if (GetNegativeTrend() > 0.0F) {
        if (leading_point_move_velocity_ > 0) {
          leading_point_move_velocity_ = 0;
        }
        if (goal_dist <= 0.0F) {
          if (goal_dist > -s_e) {
            leading_point_move_velocity_ += leading_point_stop_acceleration_;
            if (leading_point_move_velocity_ > -0.001F) {
              leading_point_move_velocity_ = -0.001F;
            }
          } else {
            leading_point_move_velocity_ -= leading_point_move_acceleration_;
            if (leading_point_move_velocity_ <
                -leading_point_move_max_velocity_) {
              leading_point_move_velocity_ = -leading_point_move_max_velocity_;
            }
          }
          if (leading_point_move_velocity_ < goal_dist) {
            leading_point_move_velocity_ = goal_dist;
          }
          dist = leading_point_move_velocity_;
        } else {
          leading_point_move_velocity_ += leading_point_move_acceleration_;
          if (leading_point_move_velocity_ > 0.0F) {
            leading_point_move_velocity_ = 0.0F;
          }
          dist = 0.0F;
        }
      } else {
        leading_point_move_velocity_ = 0.0F;
        dist = 0.0F;
      }

      return (dist);
    }

  private:
    plan_var_t max_trend_;
    plan_var_t positive_;
    plan_var_t negative_;
    plan_var_t laziness_;
    plan_var_t trend_move_velocity_;
    plan_var_t trend_stop_velocity_;

    plan_var_t leading_point_move_velocity_;
    plan_var_t leading_point_move_acceleration_;
    plan_var_t leading_point_stop_acceleration_;
    plan_var_t leading_point_move_max_velocity_;
  };

  class ActionSmoothing {
  public:
    ActionSmoothing() {
      max_req_count_ = 0;
      req_count_threshold_ = 0;
      time_allowed_ = 0;
      req_count_ = 0;
      time_ = 0;
    }

    void Clear() {
      req_count_ = 0;
      time_ = 0;
    }
    void SetParam(Int32_t threshold, Int32_t max_count, Int32_t max_time) {
      max_req_count_ = max_count;
      req_count_threshold_ = threshold;
      time_allowed_ = max_time;
    }

    bool Smooth(bool req) {
      if (req) {
        req_count_++;
        if (req_count_ >= req_count_threshold_) {
          if (req_count_ >= max_req_count_) {
            req_count_ = max_req_count_;
          }
          time_ = 0;
          return true;
        }
      } else {
        req_count_--;
        if (req_count_ < 0) {
          req_count_ = 0;
          time_ = 0;
        }
      }

      if (req_count_ > 0) {
        time_++;
        if (time_ >= time_allowed_) {
          req_count_ = 0;
          time_ = 0;
        }
      }

      return false;
    }

  public:
    Int32_t max_req_count_;
    Int32_t req_count_threshold_;
    Int32_t time_allowed_;
    Int32_t req_count_;
    Int32_t time_;
  };

private:
  /*
   * @brief 更新车身参数
   */
  void UpdateVehicleParameter();

  /*
   * @brief 清除历史信息
   */
  void ClearHoldingInfo();

  /*
   * @brief 更新内部定时器
   */
  void UpdateTimerList();

  /*
   * @brief 方向盘淡入
   * @param[in] prev_road_type 之前的路网构建类型
   * @param[in] curr_road_type 当前的路网构建类型
   */
  void StandByToFadeIn(Int32_t prev_road_type, Int32_t curr_road_type);

  /*
   * @brief 规划自动模式下的轨迹
   * @return false ~ 失败, true ~ 成功
   */
  bool PlanTrjInRoboticMode();

  /*
   * @brief 规划手动模式下的轨迹
   * @return false ~ 失败, true ~ 成功
   */
  bool PlanTrjInManualMode();

  /*
   * @brief 计算预瞄距离
   * @param[in] lat_offset 横向偏差
   * @return 预瞄距离
   */
  plan_var_t CalcLeadingLen(plan_var_t lat_offset);

  /*
   * @brief 根据当前参考线的曲率信息，限制每段纵向采样的长度
   * @param[in] start_s_on_ref_line 每个纵向采样段在参考线上的起始位置
   * @return 建议的纵向采样长度
   */
  plan_var_t CalcStepLenOfLonGraphSample(plan_var_t start_s_on_ref_line) const;

  /*
   * @brief 从指定的路网图section索引开始，查找指定的（lon_major和lon_minor）\n
   *        所在的图section索引
   * @param[in] start_sec_index 查找起始的图section索引
   * @param[in] target_lon_major 目标lon_major
   * @param[out] target_lon_minor 目标lon_minor
   * @return 指定的图section索引(-1 ~ 没有找到)
   */
  Int32_t FindGraphSection(
      Int32_t start_sec_index,
      Int32_t target_lon_major,
      Int32_t target_lon_minor) const;

  /*
   * @brief 在两个路网图的section之间构建全连接的link \n
   *        (每个图section的节点都和另一个图section中的每个节点相连)
   * @param[in] src_sec_index 起始图section的索引
   * @param[in] dest_sec_index 目标图section的索引
   */
  void ConstructFullLinksBetweenGraphSections(
      Int32_t src_sec_index, Int32_t dest_sec_index);

  /*
   * @brief 在两个路网图的section之间构建平行连接的link \n
   *        (每个section的节点只和另一个section中的平行的节点相连)
   * @param[in] expand 是否扩展连接(当没有相对应的平行节点时，是否可以连接到最近的节点)
   * @param[in] src_sec_index 起始图section的索引
   * @param[in] dest_sec_index 目标图section的索引
   */
  void ConstructParallelLinksBetweenGraphSections(
      bool expand, Int32_t src_sec_index, Int32_t dest_sec_index);

  /*
   * @brief 在两个车道片段的section之间构建全连接的link \n
   *        (每个车道section的节点都和另一个车道section中的每个节点相连)
   * @param[in] node_info 用于构建link的起始节点及目标节点的相关信息
   * @param[in] src_lane_section 起始车道section信息
   * @param[in] dest_lane_section 目标车道section信息
   */
  void ConstructFullLinksBetweenLaneSections(
      GraphNodeInfoForConstruction* node_info,
      GraphSection::LaneSection* src_lane_section,
      GraphSection::LaneSection* dest_lane_section);

  /*
   * @brief 在两个车道片段的section之间构建平行连接的link \n
   *        (每个车道section的节点只和另一个车道section中的平行的节点相连)
   * @param[in] expand 是否扩展连接(当没有相对应的平行节点时，是否可以连接到最近的节点)
   * @param[in] node_info 用于构建link的起始节点及目标节点的相关信息
   * @param[in] src_lane_section 起始车道section信息
   * @param[in] dest_lane_section 目标车道section信息
   */
  void ConstructParallelLinksBetweenLaneSections(
      bool expand,
      GraphNodeInfoForConstruction* node_info,
      GraphSection::LaneSection* src_lane_section,
      GraphSection::LaneSection* dest_lane_section);

  /*
   * @brief 在两个路网图节点之间构建link
   * @param[in] node_info 用于构建link的起始节点及目标节点的相关信息
   * @param[in] src_node 起始节点的信息
   * @param[in] dest_node 目标节点的信息
   */
  void ConstructLinkBetweenGraphNodes(
      GraphNodeInfoForConstruction* node_info,
      GraphSection::GraphNode* src_node,
      GraphSection::GraphNode* dest_node);

  /*
   * @brief 根据link的曲线方程生成一系列采样点
   * @param[in] step_len 采样间隔(相对于参考线)
   * @param[out] sample_points 生成的采样点序列
   */
  void SampleLinkCurve(
      const GraphSection::GraphNode::Link& link,
      plan_var_t step_len,
      common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum>* sample_points) const;

  /*
   * @brief 计算link曲线的曲率，并计算相应的减速度
   * @param[in&out] link link信息
   */
  void CalcCurvatureForLink(GraphSection::GraphNode::Link* link);

  /*
   * @brief 对link曲线进行碰撞分析，并计算相应的减速度
   * @param[in&out] link link信息
   */
  void TestCollisionForLink(GraphSection::GraphNode::Link* link);

  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
  /*
   * @brief 对link曲线与道路边界之间进行碰撞分析，避免生成道路外的轨迹
   * @param[in&out] link link信息
   */
  void TestCollisionWithRoadBoundaryForLink(GraphSection::GraphNode::Link* link);
  /* k005 pengc 2023-01-06 (end) */

  /*
   * @brief 计算link曲线的Cost
   * @param[in&out] link link信息
   */
  void CalcCostForLink(GraphSection::GraphNode::Link* link);

  /*
   * @brief 在指定位置，以指定的航向角生成车辆在此位姿下的有向矩形包围盒
   * @param[in] pos 位置
   * @param[in] heading 航向角
   * @param[out] obb 车辆在对应位姿下的有向矩形包围盒
   * @note 生成的obb用于碰撞分析。
   */
  void CreateVehOBB(
      const common::Vec2d& pos, plan_var_t heading,
      common::OBBox2d* obb) const;

  /*
   * @brief 根据曲率计算减速度
   * @param[in] s 减速距离
   * @param[in] abs_curvature 曲率的绝对值
   * @return 减速度
   */
  plan_var_t CalcDecelerationByCurvature(
      plan_var_t s, plan_var_t abs_curvature) const;

  void ClassifyObstacle(
      const common::PathPoint& sample_point,
      const driv_map::CollisionTestResult::ObjInfo& obj_test_ret,
      ObjClassifyingInfo* classify_info) const;

  /*
   * @brief 根据障碍物的碰撞分析结果计算减速度
   * @param[in] obj_test_ret 障碍物的碰撞分析结果
   * @return 减速度
   */
  plan_var_t CalcDecelerationByObstacle(
      const common::PathPoint& sample_point,
      const driv_map::CollisionTestResult::ObjInfo& obj_test_ret) const;

  plan_var_t CalcDecelerationByUncertainObstacle(
      const common::PathPoint& sample_point,
      const driv_map::CollisionTestResult::ObjInfo& obj_test_ret) const;

  /*
   * @brief 从路网中查找所有的候选轨迹
   * @return true - 成功, false - 失败
   */
  bool FindAllCandidateTrajectory();

  /*
   * @brief 生成候选轨迹的采样点
   * @param[in] candidate_trj_idx 候选轨迹的索引
   * @param[out] sample_points 生成的采样点
   * @return true - 成功, false - 失败
   */
  bool SampleCandidateTrajectory(
      Int32_t candidate_trj_idx,
      common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>*
          sample_points) const;

  /*
   * @brief 从候选轨迹中查找最优轨迹
   * @return true - 成功, false - 失败
   */
  bool FindOptimalTrajectory();

  /*
   * @brief Set parameters of path smoother
   */
  void SetPathSmootherParam();

  /*
   * @brief 计算规划的轨迹(综合考虑行为规划、安全性、舒适性，并保证时间轴上轨迹的连续性)
   * @return true - 成功, false - 失败
   */
  bool CalcTargetTrajectory();

  /*
   * @brief Smooth规划的轨迹
   * @param[in] candidate_trj_idx 候选轨迹的索引
   * @return true - 成功, false - 失败
   */
  bool SmoothTargetTrajectory(Int32_t candidate_trj_idx);

  void ConvertPrevRefLineToCurrCoorFrame();

  bool CreateTrajectoryByLeadingPoint(
      Int32_t dest_ref_line_index, const common::Vec2d& leading_point,
      common::StaticVector<GraphSection::GraphNode::Link, MAX_GRAPH_SECTION_NUM>* trj_links);

  bool SampleTrajectory(
      const common::StaticVector<GraphSection::GraphNode::Link, MAX_GRAPH_SECTION_NUM> trj_links,
      common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>* samples);

  void CalcLatErr(
      plan_var_t leading_l, plan_var_t ref_offset, plan_var_t pred_lat_offset, 
      Int32_t lat_moving_flag, plan_var_t max_leading_lat_offset, plan_var_t l_inc);
  void CalcLatErrSample(
      Int32_t sample_idx,
      const common::PathPoint& curr_point, const common::PathPoint& proj_point,
      plan_var_t leading_l, plan_var_t ref_offset, plan_var_t pred_lat_offset);

  Int32_t SelectTargetTrajectory(Int32_t* trj_req_type);

  bool CheckSafetyOfCandidateTrajectory(
      Int32_t candidate_trj_idx, plan_var_t ref_deceleration,
      bool* exist_uncertain_obj, Int32_t* unsafety_reason);

  /*
   * @brief 判断参考线是否变更了
   * @return 0 ~ unchanged, 1 ~ changed to left, 2 ~ changed to right
   *
   * @par Note:
   * @code
   *     此函数只能在 status_.prev_proj_on_ref 被更新前调用，
   *     status_.prev_proj_on_ref 在函数 UpdateInternalStatus 中被更新
   * @endcode
   */
  Int32_t IsRefLineChanged();

  void UpdateInternalStatus(Int32_t candidate_trj_idx, Int32_t trj_req_type);

  void ReqChangingLaneAutomatically(Int32_t curr_trj_idx);

  /* k006 pengc 2023-01-06 (begin) */
  // 更新轨迹坡度
  void UpdateTrajectorySlope();
  /* k006 pengc 2023-01-06 (begin) */

  void AddEventReporting(Int32_t event_type, Int32_t param1 = 0);

  template <Int32_t MaxObjNum>
  void AddCollisionInfoToList(
      const driv_map::CollisionTestResult::ObjInfo& coll_info,
      plan_var_t deceleration,
      common::StaticVector<CollisionInfo, MaxObjNum>* risky_obj_list,
      common::UnorderedMap<Int32_t, Int32_t, MaxObjNum>* risky_obj_lookup_tab) const;

  /* 注意：
   * 路网生成函数的实现在 "trajectory_planning_state_lattice_ex.cc" 文件中，
   * 若需要定义其它类型的路网，在下面声明其它类型的路网生成函数，并同样实现
   * 在 "trajectory_planning_state_lattice_ex.cc" 文件中。不要通过修改已有
   * 的函数来实现不同的路网。（设计原则：对修改封闭，对扩展开放）
   */
  /*
   * @brief 生成路网(类型I)
   * @return true - 成功, false - 失败
   *
   * @par Note:
   * @code
   *     除当前车辆位置处的节点(0列)外(0列分别与1～5列的全部节点进行连接))，
   *     其它列的节点只与下一列的平行的节点相连，下图中节点用+表示，连接没有画出)
   *      0    1    2    3    4    5                 6
   *   
   *                +    +    +    +                 +
   *     |+|        +    +    +    +                 +
   *           +    +    +    +    +                 +
   *                +    +    +    +                 +
   *                +    +    +    +                 +
   *
   * @endcode
   */
  bool CreateRoadGraph_Type1();
  /*
   * @brief 生成路网的纵向采样点（类型I）
   * @return true - 成功, false - 失败
   */
  bool CreateLonGraphSamples_Type1();
  /*
   * @brief 生成路网的节点（类型I）
   * @return true - 成功, false - 失败
   */
  bool CreatGraphNodes_Type1();
  /*
   * @brief 生成路网的link（类型I）
   * @return true - 成功, false - 失败
   */
  bool CreateGraphLinks_Type1();

  /*
   * @brief 生成路网（类型II）
   * @return true - 成功, false - 失败
   */
  bool CreateRoadGraph_Type2();
  /*
   * @brief 生成路网的纵向采样点（类型II）
   * @return true - 成功, false - 失败
   */
  bool CreateLonGraphSamples_Type2();
  /*
   * @brief 生成路网的节点（类型II）
   * @return true - 成功, false - 失败
   */
  bool CreatGraphNodes_Type2();
  /*
   * @brief 生成路网的link（类型II）
   * @return true - 成功, false - 失败
   */
  bool CreateGraphLinks_Type2();

private:
  // Config
  TrajectoryPlanningConfig config_;

  // 模块参数
  struct {
    // 车长，单位：米
    plan_var_t vehicle_length;
    // 车宽，单位：米
    plan_var_t vehicle_width;
    // 轴距，单位：米
    plan_var_t wheelbase;
    // 车辆的定位点到 front of vehicle 的距离，单位：米
    plan_var_t dist_of_localization_to_front;
    // 车辆的定位点到 rear of vehicle 的距离，单位：米
    plan_var_t dist_of_localization_to_rear;
    // 车辆的定位点到中心点的距离，单位：米
    plan_var_t dist_of_localization_to_center;
    // 车辆外接圆半径，单位：米
    plan_var_t vehicle_circumradius;
    // 挂车长度，单位：米
    plan_var_t trailer_length;

    // 规划用的最小前向速度（设置为大于0的较低速度(例如怠速的速度)，
    // 用来避免除以0,以及保障极低速度下的舒适性）
    plan_var_t min_forward_velocity_for_planning;
    plan_var_t min_forward_velocity_for_planning_backing_mode;
    // 向前看的时距(轨迹规划的长度为：当前车速*look_forward_time)
    plan_var_t look_forward_time;
    // 最小向前看的距离(当(当前车速*look_forward_time < min_look_forward_distance)时，
    // 轨迹规划的长度为 min_look_forward_distance)
    plan_var_t min_look_forward_distance;
    // 最大向前看的距离(当(当前车速*look_forward_time < max_look_forward_distance)时，
    // 轨迹规划的长度为 max_look_forward_distance)
    plan_var_t max_look_forward_distance;
    // 沿着车道中心线行驶时，生成轨迹的预瞄时距，用来减小车辆控制时与中心线之间的横向偏差
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> leading_distance_for_lka_ratio_table;
    plan_var_t min_leading_time_for_lka;
    plan_var_t max_leading_time_for_lka;
    // 沿着车道中心线行驶时，生成轨迹的最小预瞄距离
    plan_var_t min_leading_distance_for_lka;
    plan_var_t min_leading_distance_for_lka_backing_mode;
    // 沿着车道中心线行驶时，生成轨迹的最大预瞄距离
    plan_var_t max_leading_distance_for_lka;
    // 预瞄距离修正（根据参考线的曲率，曲率越大，预瞄距离越小）
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> leading_distance_correct_ratio_table_by_trj_curv;

    // 生成路网时，纵向采样时使用的最小车速(用来保障极低速度下的舒适性)
    plan_var_t min_backward_velocity_for_lon_sample;
    plan_var_t min_forward_velocity_for_lon_sample;
    // 生成路网时，纵向采样点间的时间间隔
    plan_var_t lon_sample_time_interval;
    // 生成路网时，纵向采样点间的距离限制（根据参考线的曲率，曲率越大，采样间隔越小）
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> lon_sample_step_len_limit_table_by_trj_curv;
    // 生成路网时，纵向采样点间的最大和最小距离限制表
    common::StaticVector<common::DataPair<plan_var_t, plan_var_t>,
    MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM> lon_sample_step_len_limit_table;

    // 生成路网时，横向采样点与道路边界的最小距离（保持与路沿间隔一定距离，避免轮胎擦碰）
    plan_var_t dist_of_keeping_away_from_road_boundary;
    // 生成路网时，横向采样点与车道边线的最小距离（避免轮胎压线）
    plan_var_t dist_of_keeping_away_from_lane_boundary;
    // 生成路网时，横向采样点的间隔
    plan_var_t lat_sample_step_len;
    // 生成路网时，link曲线的采样间隔
    plan_var_t sample_step_len_of_link_curve;

    // 最大允许的侧向加速度，用来计算link曲线的减速度
    plan_var_t max_allowed_lateral_acceleration;
    // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> relative_velocity_limit_table_by_dist_to_static_obj;
    // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> relative_velocity_limit_table_by_dist_to_dynamic_obj;

    // 车道对应的Cost Ratio，尽量在当前车道行驶
    // (乘以车道对应的车道相邻序号的绝对值，车道与当前车道偏差越大，车道相邻序号的绝对值越大)
    Int32_t left_lane_cost_ratio;
    Int32_t right_lane_cost_ratio;
    // Routing车道对应的Cost (按照一次规划的道路行驶)
    Int32_t routing_lane_cost;
    // 车道内与中心线之间的横向偏差对应的Cost Ratio，尽量沿着车道中心线行驶
    // (乘以车道中横向采样的序号的绝对值,横向偏差越大，序号的绝对值越大)
    Int32_t left_lateral_offset_cost_ratio;
    Int32_t right_lateral_offset_cost_ratio;
    //
    Int32_t lka_cost_ratio_major;
    Int32_t lka_cost_ratio_minor;
    // 候选轨迹的曲率对应的Cost Ratio，尽量沿着平滑的轨迹行驶
    // (乘以因为轨迹曲率导致的减速度的绝对值)
    Int32_t curvature_cost_ratio;
    // 与障碍物之间的碰撞风险对应的Cost Ratio，避免碰撞
    // (乘以因为障碍物导致的减速度的绝对值)
    Int32_t collision_cost_ratio;
    // 候选轨迹长度对应的Cost Ratio，进行选择规划的远的轨迹
    // (乘以路网纵向段索引，轨迹越长，段索引越大)
    Int32_t trj_len_cost_ratio;
    // 与道路边界有干涉的Cost
    Int32_t collision_with_road_boundary_cost;
    // 规划单条路径，车当前横向偏移范围与link个一个路网的关系表
    // 根据不同的横向范围选择合适的第一个路网
    common::StaticVector<plan_var_t,
        MAX_LAT_OFFSET_NUM_FOR_SINGLE_LANE> range_of_lat_offset_table;

    plan_var_t path_filter_laziness_lat;
    plan_var_t path_filter_max_trend_lat;
    plan_var_t path_filter_trend_move_velocity_lat;
    plan_var_t path_filter_trend_stop_velocity_lat;
    plan_var_t path_filter_leading_move_max_velocity_lat;
    plan_var_t path_filter_leading_move_accel_lat;
    plan_var_t path_filter_leading_stop_accel_lat;

    plan_var_t path_filter_laziness_lon;
    plan_var_t path_filter_max_trend_lon;
    plan_var_t path_filter_trend_move_velocity_lon;
    plan_var_t path_filter_trend_stop_velocity_lon;
    plan_var_t path_filter_leading_move_max_velocity_lon;
    plan_var_t path_filter_leading_move_accel_lon;
    plan_var_t path_filter_leading_stop_accel_lon;

    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> max_leading_lat_offset_limit_table_by_speed;

    plan_var_t changing_lane_complete_cond_lat_offset;
  } param_;

  veh_model::VehicleModelWrapper vehicle_model_;
  // Environment
  Int64_t curr_timestamp_;
  common::TrajectoryPoint curr_position_;
  Int32_t driving_direction_;
  ad_msg::Chassis chassis_status_;
  // 驾驶地图信息
  const driv_map::DrivingMapWrapper* driving_map_;
  // 行为规划的结果
  ActionPlanningResult result_of_action_planning_;
  // 规划用的当前车速（此车速必须大于0,否则将产生除0错误）
  plan_var_t veh_velocity_for_planning_;

  // Action request
  ActionManager action_manager_;

  /* k002 pengc 2022-08-15 (begin) */
  /* 添加偏离车道线行驶的功能 */
  struct {
    plan_var_t expected_lat_offset;

    void Clear() {
      expected_lat_offset = 0.0F;
    }
  } driving_off_center_;
  /* k002 pengc 2022-08-15 (end) */

  // Reference line
  // 主参考线的索引
  Int32_t major_ref_line_index_;
  // 主参考线的曲线信息
  const common::Path* major_ref_line_;
  // 当前车辆位置在主参考线上的投影点
  common::PathPoint veh_proj_on_major_ref_line_;

  // Current lane
  Int32_t curr_lane_index_;
  common::PathPoint veh_proj_on_curr_lane_;

  // Parameter in running
  // 规划轨迹向前的长度
  plan_var_t look_forward_length_;
  // 规划轨迹向前的预瞄长度（用来减小LKA时，与车道中心线间的横向偏差）
  plan_var_t leading_length_for_lka_;

  // 路网构建类型
  Int32_t road_builed_type_;
  Int32_t prev_road_builed_type_;
  // 生成路网时，保存每段的纵向采样长度
  common::StaticVector<plan_var_t, MAX_LONGITUDINAL_GRAPH_SAMPLE_NUM>
      lon_graph_samples_step_len_;
  // 用来存储路网的link节点
  common::StaticVector<GraphLinkListNode, MAX_GRAPH_LINK_NUM>
      road_graph_link_storage_;
  // 路网
  common::StaticVector<GraphSection, MAX_GRAPH_SECTION_NUM> road_graph_;

  // 候选轨迹列表
  common::StaticVector<CandidateTrajectory, MAX_CANDIDATE_TRAJECTORY_NUM>
      candidate_trj_list_;

  struct {
    Int32_t trj_idx_global;

    /* k001 pengc 2022-04-03 (begin) */
    // 避免避让的轨迹左右频繁跳动
    Int32_t trj_idx_curr_lane_left;
    Int32_t trj_idx_curr_lane_right;
    /* k001 pengc 2022-04-03 (end) */
    Int32_t trj_idx_curr_lane;
    Int32_t trj_idx_lka;
    /* k001 pengc 2022-04-10 (begin) */
    // 在一定的时间范围内，尽量保持之前的避让趋势
    Int32_t trj_idx_curr_lane_bypassing;
    /* k001 pengc 2022-04-10 (end) */

    Int32_t trj_idx_left_lane;
    Int32_t trj_idx_right_lane;

    void Clear() {
      trj_idx_global = -1;

      /* k001 pengc 2022-04-03 (begin) */
      // 避免避让的轨迹左右频繁跳动
      trj_idx_curr_lane_left = -1;
      trj_idx_curr_lane_right = -1;
      /* k001 pengc 2022-04-03 (end) */
      trj_idx_curr_lane = -1;
      trj_idx_lka = -1;
      /* k001 pengc 2022-04-10 (begin) */
      // 在一定的时间范围内，尽量保持之前的避让趋势
      trj_idx_curr_lane_bypassing = -1;
      /* k001 pengc 2022-04-10 (end) */

      trj_idx_left_lane = -1;
      trj_idx_right_lane = -1;
    }
  } optimal_trj_index_;
  // 最优轨迹的采样点
  common::StaticVector<common::PathPoint,
  common::Path::kMaxPathPointNum> optimal_trajectory_sample_points_;

  // 根据当前车辆状态预测的轨迹
  common::Path pred_path_of_vehicle_;

  // 规划的结果
  common::StaticVector<GraphSection::GraphNode::Link, MAX_GRAPH_SECTION_NUM>
      target_trajectory_link_list_;
  TrajectoryPlanningResult result_of_planning_;

  struct {
    bool valid;
    Int64_t timestamp;

    PathTrendMgr path_trend_mgr_lat;
    PathTrendMgr path_trend_mgr_lon;

    common::PathPoint prev_veh_pos;
    common::PathPoint prev_leading_point;
    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
        prev_ref_line_sample_points;
    common::Path prev_ref_line;

    void Clear() {
      valid = false;
      timestamp = 0;

      path_trend_mgr_lat.Reset();
      path_trend_mgr_lon.Reset();

      prev_veh_pos.Clear();
      prev_leading_point.Clear();

      prev_ref_line_sample_points.Clear();
      prev_ref_line.Clear();
    }
  } path_filter_;

  struct LatErr {
    enum { MAX_LATERAL_ERR_LIST_SIZE = 4 };

    struct {
      bool changed_flag;
      Int32_t changed_count;
      Float32_t ref_offset;

      void Clear() {
        changed_flag = false;
        changed_count = 0;
        ref_offset = 0.0F;
      }
    } ref_changed;

    struct LatErrFilter {
      struct {
        bool valid;
        Int64_t timestamp;
        plan_var_t lat_err;
        plan_var_t lat_err_ref;
        plan_var_t lat_err_v;
        plan_var_t lat_err_v_ref;
        plan_var_t lat_err_a;
        plan_var_t yaw_err;
        plan_var_t yaw_err_v;
        plan_var_t yaw_err_v_raw;
        plan_var_t yaw_err_a;
      } lat_err_list[MAX_LATERAL_ERR_LIST_SIZE];

      struct {
        plan_var_t expectation;
        plan_var_t covariance;
      } lat_err_filter;

      struct {
        plan_var_t expectation;
        plan_var_t covariance;
      } lat_err_ref_filter;

      struct {
        plan_var_t ref_lat_err_v;
        plan_var_t expectation;
        plan_var_t covariance;
      } lat_err_v_filter;

      struct {
        plan_var_t expectation;
        plan_var_t covariance;
      } yaw_err_filter;

      struct {
        plan_var_t ref_yaw_err_v;
        plan_var_t expectation;
        plan_var_t covariance;
      } yaw_err_v_filter;
    } filter[2];

    struct LatErrSample {
      plan_var_t lat_err;
      plan_var_t lat_err_smooth;
      plan_var_t lat_err_v;
      plan_var_t lat_err_v_smooth;
      plan_var_t yaw_err;
      plan_var_t yaw_err_smooth;
      plan_var_t yaw_err_v;
      plan_var_t yaw_err_v_smooth;
    } lat_err_sample[2];

    struct {
      plan_var_t x;
      plan_var_t y;
      plan_var_t h;
    } pred_point;

    void Clear() {
      ref_changed.Clear();
      common::com_memset(&filter[0], 0, sizeof(filter));
      common::com_memset(lat_err_sample, 0, sizeof(lat_err_sample));
      common::com_memset(&pred_point, 0, sizeof(pred_point));
    }

    LatErr() {
      Clear();
    }
  } lat_err_;

  struct {
    Int32_t trj_status;
    common::PathPoint prev_proj_on_ref;

    void Clear() {
      trj_status = TRJ_STATUS_INVALID;
      prev_proj_on_ref.Clear();
    }
  } status_;

  struct {
    ActionSmoothing abort_changing_lane_left_by_uncertain_obj;
    ActionSmoothing abort_changing_lane_right_by_uncertain_obj;
    ActionSmoothing req_bypassing_left;
    ActionSmoothing req_bypassing_right;
  } action_smoothing_;

  // 定时器
  struct {
    /* k002 pengc 2022-08-15 (begin) */
    /* 进入自动或丢失车道线然后重新识别后，平缓方向盘的修正过程 */
    bool fade_in_timer_flag;
    common::ComTimer fade_in;
    /* k002 pengc 2022-08-15 (end) */
    common::ComTimer hold_bypassing;
  } timers_;
  common::ComTimerList<10> timer_list_;

  common::StaticVector<ad_msg::EventReporting, 4> event_reporting_list_;

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>
      prev_ref_line_sample_points_for_debug_;

  /*
   * @struct TemporaryData
   * @brief 轨迹规划时需要的一些内部数据。
   */
  struct TemporaryData {
    /*
     * @struct StackForFindingCandidateTrj
     * @brief 从路网中查找候选轨迹用的栈的数据结构
     */
    struct StackForFindingCandidateTrj {
      /*
       * @struct StackNode
       * @brief 从路网中查找候选轨迹时需要的单个栈结点数据结构
       */
      struct StackNode {
        // link访问索引
        Int32_t link_access_index;
        // Trajectory segments
        common::StaticVector<CandidateTrajectory::TrjNode,
        MAX_GRAPH_SECTION_NUM> trajectory_segments;

        /*
         * @brief 清空所有成员变量的值。
         */
        void Clear() {
          link_access_index = -1;

          for (Int32_t i = 0; i < trajectory_segments.Size(); ++i) {
            trajectory_segments[i].Clear();
          }
          trajectory_segments.Clear();
        }

        /*
         * @brief 构造函数。
         */
        StackNode() {
          Clear();
        }
      };

      // 栈中的结点列表
      common::StaticVector<StackNode, MAX_GRAPH_SECTION_NUM> nodes;

      /*
       * @brief 将栈中的结点信息清空。
       */
      void Clear() {
        for (Int32_t i = 0; i < nodes.Size(); ++i) {
          nodes[i].Clear();
        }
        nodes.Clear();
      }

      /*
       * @brief 构造函数。
       */
      StackForFindingCandidateTrj() {
        Clear();
      }

      /*
       * @brief   分配一个新的栈结点。
       * @return  分配的新的栈结点。
       */
      inline StackNode* Allocate() {
        StackNode* node = nodes.Allocate();
        if (Nullptr_t != node) {
          node->Clear();
        }
        return (node);
      }

      /*
       * @brief 判断栈是否为空。
       * @return true-栈为空；false-栈不为空。
       */
      bool Empty() {
        return nodes.Empty();
      }

      /*
       * @brief 从栈中取顶上的结点。
       * @return 栈中顶上的结点。
       */
      inline StackNode& Top() {
        return nodes.Back();
      }

      /*
       * @brief 弹出栈顶的结点。
       */
      inline void Pop() {
        if (!nodes.Empty()) {
          nodes.Back().Clear();
          nodes.PopBack();
        }
      }
    } stack_for_finding_candidate_trj;

    common::UnorderedMap<Int32_t, Int32_t,
    MAX_RISKY_OBJ_IN_LINK> risky_obj_lookup_tab_1;
    common::UnorderedMap<Int32_t, Int32_t,
    MAX_RISKY_OBJ_IN_LINK> risky_obj_lookup_tab_2;

    common::StaticVector<common::PathPoint,
    common::Path::kMaxPathPointNum> sample_points_1;
    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
    sample_points_2d_1;

    driv_map::CollisionTestOnPathResult collision_test_on_path_result_1;

    /*
     * @brief 将成员变量的值清空。
     */
    void Clear() {
      stack_for_finding_candidate_trj.Clear();
      risky_obj_lookup_tab_1.Clear();
      risky_obj_lookup_tab_2.Clear();
      sample_points_1.Clear();
      sample_points_2d_1.Clear();
      collision_test_on_path_result_1.Clear();
    }
  } temp_data_;

  /// For debugging
#if (ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_DEBUGGING_FILE)
  Char_t str_buff_[1024*2];
  common::LogFile log_file_lat_err_;
#endif
};


template <Int32_t MaxObjNum>
void TrajectoryPlanningStateLattice::AddCollisionInfoToList(
    const driv_map::CollisionTestResult::ObjInfo& coll_info,
    plan_var_t deceleration,
    common::StaticVector<CollisionInfo, MaxObjNum>* risky_obj_list,
    common::UnorderedMap<Int32_t, Int32_t, MaxObjNum>* risky_obj_lookup_tab) const {
  if (coll_info.static_distance > 2.0F) {
    return;
  }

  const Int32_t* risky_obj_index =
      risky_obj_lookup_tab->Find(coll_info.obj_list_index);
  if (Nullptr_t == risky_obj_index) {
    // 还未添加到列表中
    if (risky_obj_list->Full()) {
      // 列表已经满了
      // 找到列表中风险最小的，若其风险小于当前的，则替换它
      Int32_t min_risk_obj_index = 0;
      for (Int32_t i = 1; i < risky_obj_list->Size(); ++i) {
        const CollisionInfo& cur = risky_obj_list->GetData(i);
        const CollisionInfo& cmp = risky_obj_list->GetData(min_risk_obj_index);
        if (((common::com_abs(cur.deceleration - cmp.deceleration) < 0.1F) &&
             (cur.test_ret < cmp.test_ret)) ||
            (cmp.deceleration < (cur.deceleration-0.01F))) {
          min_risk_obj_index = i;
        }
      }
      CollisionInfo& min_risk_obj = risky_obj_list->GetData(min_risk_obj_index);
      if (((common::com_abs(deceleration - min_risk_obj.deceleration) < 0.1F) &&
           (min_risk_obj.test_ret < coll_info)) ||
          (deceleration < (min_risk_obj.deceleration-0.01F))) {
        // 当前碰撞风险大
        // 从索引表中删除旧的索引
        risky_obj_lookup_tab->Erase(min_risk_obj.test_ret.obj_list_index);
        // 替换为新的障碍物信息
        min_risk_obj.deceleration = deceleration;
        min_risk_obj.test_ret = coll_info;
        // 添加新的索引信息到索引表
        risky_obj_lookup_tab->Insert(
              coll_info.obj_list_index, min_risk_obj_index);
      }
    } else {
      // 列表未满
      Int32_t obj_idx = -1;
      CollisionInfo* rsk_obj_info = risky_obj_list->Allocate(&obj_idx);
      if (Nullptr_t != rsk_obj_info) {
        // 添加新的障碍物信息
        rsk_obj_info->deceleration = deceleration;
        rsk_obj_info->test_ret = coll_info;
        // 添加新的索引信息到索引表
        risky_obj_lookup_tab->Insert(coll_info.obj_list_index, obj_idx);
      } else {
        // 这个错误是不应当发生的
        LOG_ERR << "Can't add risky obj to list.";
      }
    }
  } else {
    // 已经添加到列表中了
    CollisionInfo& rsk_obj_info = risky_obj_list->GetData(*risky_obj_index);
    // 若当前风险大，则更新为新的风险信息
    if (deceleration < rsk_obj_info.deceleration) {
      rsk_obj_info.deceleration = deceleration;
    }
    if (rsk_obj_info.test_ret < coll_info) {
      // 当前碰撞风险大
      rsk_obj_info.test_ret = coll_info;
    }
  }
}


} // planning
} // phoenix


#endif // PHOENIX_PLANNING_TRAJECTORY_PLANNING_STATE_LATTICE_H_
