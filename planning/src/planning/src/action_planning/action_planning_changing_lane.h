/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       action_planning_base.h
 * @brief      行为规划基类
 * @details    行为规划的基类，规划车辆行为(变道、跟车、避让、巡航等)
 *
 * @author     sip (sip@goyu-ai.com)
 * @date       2021/05/07
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/05/07  <td>1.0      <td>sip       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_PLANNING_ACTION_PLANNING_CHANGING_LANE_H_
#define PHOENIX_PLANNING_ACTION_PLANNING_CHANGING_LANE_H_

#include "driving_map_wrapper.h"
#include "action_planning/action_planning_base.h"
#include "action_planning/action_planning_lane_risk_analyser.h"


namespace phoenix {
namespace planning {


/// 变道
class ActionPlanningChangingLane : public ActionPlanningBase {
public:
  ActionPlanningChangingLane();
  ~ActionPlanningChangingLane();

private:
  struct ObjClassifyingInfo {
    bool cut_in;
    plan_var_t static_distance;
    plan_var_t dist_to_tar_obj;
    Int32_t obj_position;
    Int32_t obj_direction;
    common::PathPoint tar_obj_proj;

    void Clear() {
      cut_in = false;
      static_distance = -1.0F;
      dist_to_tar_obj = 0.0F;
      obj_position = driv_map::OBJ_POSITION_UNKNOWN;
      obj_direction = driv_map::OBJ_DIRECTION_UNKNOWN;
    }

    ObjClassifyingInfo() {
      Clear();
    }
  };

private:
  void Handle(
      const ActionPlanningDataSource &data_source,
      ActionPlanningResult *planning_result);

  void UpdateVehicleParameter();
  void UpdateChangingLaneStatus(const ActionPlanningDataSource& data_source);
  void UpdateChangingLaneStatusByRef(
      const ActionPlanningDataSource& data_source);

  Int32_t GetChangingLaneReqType(
      const ActionPlanningDataSource& data_source,
      ActionPlanningResult* planning_result);

  Int32_t ReqChangingLaneAutomatically(
      const ActionPlanningDataSource& data_source);
  void ForbidAutoChangingLane();

  bool JudgeChangingLaneConditions(
      Int32_t request_type, Int32_t* refuse_reason);
  /* k007 longjiaoy 2023-03-02 (start) */
  bool JudgeChangingLaneConditionsInChangingLane(
      Int32_t request_type, Int32_t* refuse_reason, Int32_t* event_type);
  /* k007 longjiaoy 2023-03-02 (end) */

  Int32_t JudeChangingLaneAutomaticly(
      const ActionPlanningDataSource& data_source);

  void ForbidAlcChangeLane();
  void StopHoldChangeLaneReqByRefuseReason(Int32_t refuse_reason);

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
    common::StaticVector<common::LerpTableNodeType1,
    MAX_LERP_TABLE_SIZE> dist_limit_table_for_changine_lane;
  } param_;

  const driv_map::DrivingMapWrapper* driving_map_;

  Int32_t changing_lane_status_;
  Int32_t pre_changing_lane_status_;
  plan_var_t prev_lat_offset_to_major_ref_;

  ChangingLaneReq changing_lane_req_;
  ChangingLaneReq hmi_changing_lane_req_;
};


}  // namespace planning
}  // namespace phoenix


#endif  // PHOENIX_PLANNING_ACTION_PLANNING_CHANGING_LANE_H_

