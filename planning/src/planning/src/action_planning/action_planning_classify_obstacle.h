/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       action_classify_obstacle.h
 * @brief      障碍物等级划分类
 * @details    根据障碍物等级划分来判断是否自主变道
 *
 * @author     longjiaoy (longjiaoy@goyu-ai.com)
 * @date       2022/01/07
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/05/07  <td>1.0      <td>sip       <td>First edition
 * </table>
 *
 ******************************************************************************/
//class ObjClassifying {
//  bool cut_in;
//  plan_var_t static_distance;
//  plan_var_t dist_to_tar_obj;
//  Int32_t obj_position;
//  Int32_t obj_direction;
//  common::PathPoint tar_obj_proj;

//  void Clear() {
//    cut_in = false;
//    static_distance = -1.0F;
//    dist_to_tar_obj = 0.0F;
//    obj_position = driv_map::OBJ_POSITION_UNKNOWN;
//    obj_direction = driv_map::OBJ_DIRECTION_UNKNOWN;
//  }

//  ObjClassifyingInfo() {
//    Clear();
//  }
//};
#ifndef ACTION_CLASSIFY_OBSTACLE_H_
#define ACTION_CLASSIFY_OBSTACLE_H_


#include "driving_map_wrapper.h"
#include "motion_planning.h"
#include "utils/linear_interpolation.h"
#include "action_planning/action_planning_base.h"


namespace phoenix {
namespace planning {


class ObjClassify {
public:
  ObjClassify();
  ~ObjClassify();

  struct ObjClassifyInfo {
    bool cut_in;
    plan_var_t static_distance;
    plan_var_t dist_to_tar_obj;
    Int32_t obj_position;
    Int32_t obj_direction;
    common::PathPoint tar_obj_proj;
  };

  struct ObjClassifyResult {
    //
    bool valid;
    plan_var_t deceleration;
    // 目标车速
    plan_var_t tar_velocity;
    plan_var_t obj_l;
    plan_var_t obj_s;

    void Clear() {
      deceleration = 0.0F;
      tar_velocity = 0.0F;
      obj_l = 0.0F;
      obj_s = 0.0F;
      valid = false;
    }
  };

  struct TurnLaneSorce {
    Int32_t lane_sorce ;
    void Clear() {
      lane_sorce = 0;
    }
  };

  void ClassifyObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const common::Path& path,
      plan_var_t start_s_on_path,
      ObjClassifyInfo* obj_cfy_info) const;

  bool CalcRiskofObstacle(
      const driv_map::CollisionTestResult::ObjInfo& test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);

  bool CalcRiskofObstacleOnRef(
      const ActionPlanningDataSource& data_source,
      const Int32_t& ref_index,
      const Int32_t& routing_neighbor_flag);

  void Clear() {
    curr_lane_socre_.Clear();
  }

  const TurnLaneSorce& GetTurnLaneSorce() const{
    return curr_lane_socre_;
  }

private:
  bool CalcDecelerationByFrontObstacle(
      const driv_map::CollisionTestResult::ObjInfo &test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);
  bool CalcDecelerationByLeftOrRightFrontObstacle(
      const driv_map::CollisionTestResult::ObjInfo &test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);
  bool CalcDecelerationByCloseObstacle(
      const driv_map::CollisionTestResult::ObjInfo &test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);

  bool CalcDecelerationByLeftOrRightObstacle(
      const driv_map::CollisionTestResult::ObjInfo &test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);

  bool CalcDecelerationByLeftOrRightBackObstacle(
      const driv_map::CollisionTestResult::ObjInfo &test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);

  bool CalcDecelerationByBackObstacle(
      const driv_map::CollisionTestResult::ObjInfo &test_ret,
      const ad_msg::Obstacle& tar_obj,
      const ad_msg::Chassis& chassis,
      const ObjClassifyInfo& obj_cfy_info);

  ObjClassifyResult obj_classify_result_;

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
    // 与障碍物之间的碰撞风险对应的Cost Ratio，避免碰撞
    // (乘以因为障碍物导致的减速度的绝对值)
    plan_var_t collision_deceleration_ratio;

    plan_var_t lane_offest_cost;
    //plan_var_t right_lane_cost;

    plan_var_t tar_velocity_ratio;
    plan_var_t routing_ratio;

    plan_var_t safe_ttc;
    plan_var_t safe_time_gap;

    // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> relative_velocity_limit_table_by_dist_to_static_obj;
    // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> relative_velocity_limit_table_by_dist_to_dynamic_obj;
  } param_;

  TurnLaneSorce curr_lane_socre_;
};

}
}
#endif //action_classify_obstacle
