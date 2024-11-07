//
#ifndef PHOENIX_PLANNING_ACTION_PLANNING_LANE_RISK_ANALYSER_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_LANE_RISK_ANALYSER_H_


#include "utils/linear_interpolation.h"
#include "driving_map_wrapper.h"
#include "motion_planning.h"


namespace phoenix {
namespace planning {


class ActionPlanningLaneRiskAnalyser {
public:
  /// 风险障碍物信息
  struct RiskyObj {
    /// 是否有效
    bool valid;
    /// 障碍物是否会切入
    bool cut_in;
    /// 是否动态障碍物
    bool dynamic;
    /// 障碍物在车辆坐标系下的位置
    Int32_t obj_pos;
    /// 障碍物移动方向(forward, backward, crossed)
    Int32_t obj_dir;
    /// 横向距离
    plan_var_t lat_dist;
    /// 纵向距离
    plan_var_t lon_dist;
    /// 障碍物速度
    plan_var_t obj_v;
    /// 时距
    plan_var_t t_gap;
    /// 碰撞时间
    plan_var_t ttc;


    RiskyObj() {
      Clear();
    }

    void Clear() {
      valid = false;
      cut_in = false;
      dynamic = false;
      obj_pos = driv_map::OBJ_POSITION_UNKNOWN;
      obj_dir = driv_map::OBJ_DIRECTION_UNKNOWN;
      lat_dist = 0.0F;
      lon_dist = 0.0F;
      obj_v = 0.0F;
      t_gap = 0.0F;
      ttc = 0.0F;
    }
  };

  struct LaneRiskInfo {
    /// 车道质量好坏
    Int32_t lane_quality;
    /// 是否存在不确定的障碍物
    bool has_uncertain_obj;
    /// 是否存在风险
    bool has_risk;
    /// 目标速度
    plan_var_t tar_v;
    /// 目标加速度
    plan_var_t tar_a;

    /* k003 longjiaoy 2022-11-28 (start) */
    /// 车道边界信息
    struct {
      // Boundary type
      bool allow_to_change_left;
      bool allow_to_change_right;
      // Lane width
      plan_var_t left_width;
      plan_var_t right_width;

      void Clear() {
        allow_to_change_left = false;
        allow_to_change_right = false;
        left_width = 0.0F;
        right_width = 0.0F;
      }
    } boundary;
    /* k003 longjiaoy 2022-11-28 (end) */

    RiskyObj nearest_obj[driv_map::OBJ_POSITION_MAX];
    RiskyObj min_ttc_obj[driv_map::OBJ_POSITION_MAX];

    LaneRiskInfo() {
      Clear();
    }

    void Clear() {
      lane_quality = 0;
      has_uncertain_obj = false;
      has_risk = false;
      tar_v = 0.0F;
      tar_a = 0.0F;

      /* k003 longjiaoy 2022-11-28 (start) */
      boundary.Clear();
      /* k003 longjiaoy 2022-11-28 (end) */

      for (Int32_t i = 0; i < driv_map::OBJ_POSITION_MAX; ++i) {
        nearest_obj[i].Clear();
        min_ttc_obj[i].Clear();
      }
    }
  };

public:
  ActionPlanningLaneRiskAnalyser();
  ~ActionPlanningLaneRiskAnalyser();

  bool AnalyseRisksOfLane(
      const ActionPlanningDataSource &data_source,
      Int32_t ref_line_idx,
      LaneRiskInfo* lane_risk_info);

private:
  struct ObjClassifyingInfo {
    // 障碍物是否会切入轨迹
    bool cut_in;
    // 障碍物与自车的横向距离（垂直于轨迹方向）
    plan_var_t static_distance;
    // 障碍物距离自车的纵向距离（沿着轨迹的方向）
    plan_var_t dist_to_tar_obj;
    // 障碍物在车辆坐标系下的位置
    Int32_t obj_position;
    // 障碍物移动方向(forward, backward, crossed)
    Int32_t obj_direction;
    // 障碍物在轨迹上的投影
    common::PathPoint tar_obj_proj;

    ObjClassifyingInfo() {
      Clear();
    }

    void Clear() {
      cut_in = false;
      static_distance = -1.0F;
      dist_to_tar_obj = 0.0F;
      obj_position = driv_map::OBJ_POSITION_UNKNOWN;
      obj_direction = driv_map::OBJ_DIRECTION_UNKNOWN;
    }
  };

  struct ObjRiskInfo {
    bool has_risk;
    plan_var_t tar_v;
    plan_var_t tar_a;

    RiskyObj risky_obj;

    ObjRiskInfo() {
      Clear();
    }

    void Clear() {
      has_risk = false;
      tar_v = 0.0F;
      tar_a = 0.0F;
      risky_obj.Clear();
    }
  };

private:
  void UpdateVehicleParameter(const ad_msg::Chassis* chassis);

  void ClassifyObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const common::Path& path,
      plan_var_t star_s_on_path,
      ObjClassifyingInfo* obj_cfy_info) const;

  void CalcRiskOfObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;

  void CalcRiskOfFrontDynamicObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfLateralFrontDynamicObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfLateralDynamicObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfLateralBackDynamicObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfBackDynamicObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfClosedDynamicObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;

  void CalcRiskOfFrontStaticObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfLateralFrontStaticObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfLateralStaticObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfLateralBackStaticObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfBackStaticObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;
  void CalcRiskOfClosedStaticObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ObjClassifyingInfo& obj_cfy_info,
      ObjRiskInfo* risk_info) const;

  /* k003 longjiaoy 2022-11-28 (start) */
  void AnalyseLaneBoundary(
      const ActionPlanningDataSource &data_source,
      Int32_t ref_line_idx, LaneRiskInfo* lane_risk_info);
  bool IsAllowToChangLaneByBoundaryType(Int32_t boundary_type);
  /* k003 longjiaoy 2022-11-28 (end) */

  void SetRiskyObj(
      const ObjRiskInfo& obj_risk_info, LaneRiskInfo* lane_risk_info);

private:
  struct {
    // 车长，单位：米
    plan_var_t vehicle_length;
    // 车宽，单位：米
    plan_var_t vehicle_width;
    // 车辆的定位点到 front of vehicle 的距离，单位：米
    plan_var_t dist_of_localization_to_front;
    // 车辆的定位点到 rear of vehicle 的距离，单位：米
    plan_var_t dist_of_localization_to_rear;
    // 车辆的定位点到中心点的距离，单位：米
    plan_var_t dist_of_localization_to_center;
    /* k007 longjiaoy 2023-03-02 (start) */
    // 车辆的挂长，单位：米
    plan_var_t trailer_length;
    /* k007 longjiaoy 2023-03-02 (end) */

    plan_var_t safe_ttc_of_front_dynamic_obj;
    plan_var_t safe_time_gap_of_front_dynamic_obj;
    plan_var_t safe_ttc_of_back_dynamic_obj;
    plan_var_t safe_time_gap_of_back_dynamic_obj;

    plan_var_t safe_ttc_of_front_static_obj;
    plan_var_t safe_time_gap_of_front_static_obj;
    plan_var_t safe_ttc_of_back_static_obj;
    plan_var_t safe_time_gap_of_back_static_obj;

    // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> relative_velocity_limit_table_by_dist_to_static_obj;
    // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> relative_velocity_limit_table_by_dist_to_dynamic_obj;
  } param_;

  struct {
    Int32_t ref_line_idx;
    Int32_t neighbor_flag;
    Int32_t lane_idx;
    plan_var_t left_lane_width;
    plan_var_t right_lane_width;

    void Clear() {
      ref_line_idx = -1;
      neighbor_flag = 0;
      lane_idx = -1;
      left_lane_width = 1.5F;
      right_lane_width = 1.5F;
    }
  } lane_info_;

  plan_var_t curr_velocity_;
};


}  // namespace planning
}  // namespace phoenix


#endif // PHOENIX_PLANNING_ACTION_PLANNING_LANE_RISK_ANALYSER_H_

