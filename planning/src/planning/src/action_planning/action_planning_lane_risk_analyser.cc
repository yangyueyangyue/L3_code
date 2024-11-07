//
#include "action_planning/action_planning_lane_risk_analyser.h"

#include "utils/macros.h"
#include "utils/log.h"
#include "geometry/geometry_utils.h"
#include "vehicle_model_wrapper.h"


#define ENABLE_ACTION_PLANNING_LANE_RISK_ANALYSER_TRACE (0)


namespace phoenix {
namespace planning {


ActionPlanningLaneRiskAnalyser::ActionPlanningLaneRiskAnalyser() {
  veh_model::VehicleModelWrapper veh_model;
  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front =
      veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear =
      veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();
  /* k007 longjiaoy 2023-03-02 (start) */
  // 车辆的挂长，单位：米
  param_.trailer_length = veh_model.GetTrailerLength();
  /* k007 longjiaoy 2023-03-02 (end) */

  param_.safe_ttc_of_front_dynamic_obj = 10.0F;
  param_.safe_time_gap_of_front_dynamic_obj = 1.5F;
  param_.safe_ttc_of_back_dynamic_obj = 10.0F;
  param_.safe_time_gap_of_back_dynamic_obj = 1.5F;

  param_.safe_ttc_of_front_static_obj = 20.0F;
  param_.safe_time_gap_of_front_static_obj = 10.0F;
  param_.safe_ttc_of_back_static_obj = 5.0F;
  param_.safe_time_gap_of_back_static_obj = 1.0F;

  // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
  param_.relative_velocity_limit_table_by_dist_to_static_obj.Clear();
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(0.0F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(0.3F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(0.5F, 15.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(0.7F, 25.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(1.0F, 80.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(1.5F, 90.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(
        common::LerpTableNodeType1(2.0F, 100.0F/3.6F));
  // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.Clear();
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(
        common::LerpTableNodeType1(0.0F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(
        common::LerpTableNodeType1(0.3F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(
        common::LerpTableNodeType1(0.9F, 20.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(
        common::LerpTableNodeType1(1.0F, 30.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(
        common::LerpTableNodeType1(1.0F, 30.0F/3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(
        common::LerpTableNodeType1(2.0F, 30.0F/3.6F));
}

ActionPlanningLaneRiskAnalyser::~ActionPlanningLaneRiskAnalyser() {
  // nothing to do
}

bool ActionPlanningLaneRiskAnalyser::AnalyseRisksOfLane(
    const ActionPlanningDataSource &data_source,
    Int32_t ref_line_idx,
    LaneRiskInfo* lane_risk_info) {
  if (Nullptr_t == data_source.chassis) {
    LOG_ERR << "Invalid chassis information.";
    return false;
  }
  if (!data_source.chassis->v_valid) {
    LOG_ERR << "Invalid velocity value of chassis.";
    return false;
  }
  const driv_map::DrivingMapWrapper* driving_map = data_source.driving_map;
  if (Nullptr_t == driving_map) {
    LOG_ERR << "Invalid driving map.";
    return false;
  }
  if (!driving_map->IsValidReferenceLineIndex(ref_line_idx)) {
    LOG_ERR << "Invalid reference line index.";
    return false;
  }

  // 更新车身参数
  UpdateVehicleParameter(data_source.chassis);

  // 根据参考线索引，获取参考线的车道线质量
  lane_risk_info->lane_quality =
      driving_map->GetReferenceLineLaneQuality(ref_line_idx);
  lane_risk_info->has_uncertain_obj = false;
  lane_risk_info->has_risk = false;
  lane_risk_info->tar_v = data_source.chassis->v;
  lane_risk_info->tar_a = 0.0F;
  /* k003 longjiaoy 2022-11-28 (start) */
  lane_risk_info->boundary.Clear();
  /* k003 longjiaoy 2022-11-28 (end) */

  curr_velocity_ = data_source.chassis->v;
  lane_info_.Clear();
  lane_info_.ref_line_idx = ref_line_idx;

  lane_info_.neighbor_flag =
      driving_map->GetReferenceLineNeighborFlag(ref_line_idx);
  plan_var_t curr_proj_s =
      driving_map->GetProjDistOfCurrPositionOnRef(ref_line_idx);
  const common::Path& ref_line =
      driving_map->GetSmoothReferenceLine(ref_line_idx);
  driving_map->FindReferenceLineLaneSegmentByProjOnRef(
        ref_line_idx, curr_proj_s, &lane_info_.lane_idx);
  if (!driving_map->IsValidLaneIndex(lane_info_.lane_idx)) {
    LOG_ERR << "Invalid lane index.";
    return false;
  }

  driving_map->GetLaneWidth(lane_info_.lane_idx, 0.0F,
                            &lane_info_.left_lane_width,
                            &lane_info_.right_lane_width);

#if ENABLE_ACTION_PLANNING_LANE_RISK_ANALYSER_TRACE
 #if 0
  printf(">>> 车道风险分析{ 参考线索引=%d,"
         ", 车道标记=%d"
         ", 车道宽度(左=%0.1f, 右=%0.1f)"
         " }"
         "\n"
         ,
         ref_line_idx,
         lane_info_.neighbor_flag,
         lane_info_.left_lane_width, lane_info_.right_lane_width);
 #else
  LOG_INFO(5) << ">>> Lane risk analyse { ref_idx=" << ref_line_idx
              << ", neighbor_flag=" << lane_info_.neighbor_flag
              << ", lane_width(left=" << lane_info_.left_lane_width
              << ", right=" << lane_info_.right_lane_width
              << ")}";
 #endif
#endif


  driv_map::CollisionTestResult result;
  driving_map->GetRiskyObstacles(ref_line_idx, true, &result);

  if (result.uncertain_list.Size() > 1) {
    // 存在不确定的障碍物
    for (Int32_t i = 0; i < result.uncertain_list.Size(); ++i) {
      const driv_map::CollisionTestResult::ObjInfo& test_ret =
          result.uncertain_list[i];
      //const ad_msg::Obstacle& tar_obj =
      //    data_source.driving_map->GetObstacle(test_ret.obj_list_index);

      //printf("uncertain_obj[%d]: s_ref=%0.1f, curr_proj_s=%0.1f\n",
      //       i, test_ret.obj_s_ref, curr_proj_s);

      plan_var_t dist_to_obj = test_ret.obj_s_ref - curr_proj_s;
      if ((-(param_.dist_of_localization_to_rear+2.0F) < dist_to_obj) &&
          (dist_to_obj < common::Max(param_.dist_of_localization_to_front+5.0F,
                                     curr_velocity_*3.0F)))

      lane_risk_info->has_uncertain_obj = true;
      break;
    }
  }

  // 确定的障碍物
  for (Int32_t i = 0; i < result.risky_obj_list.Size(); i++) {
    const driv_map::CollisionTestResult::ObjInfo& test_ret =
        result.risky_obj_list[i];
    const ad_msg::Obstacle& tar_obj =
        data_source.driving_map->GetObstacle(test_ret.obj_list_index);
    // 对障碍物分类
    ObjClassifyingInfo obj_cfy_info;
    ClassifyObstacle(test_ret, tar_obj,
                     ref_line, curr_proj_s,
                     &obj_cfy_info);
    ObjRiskInfo obj_risk_info;
    obj_risk_info.risky_obj.valid = false;
    obj_risk_info.risky_obj.cut_in = obj_cfy_info.cut_in;
    obj_risk_info.risky_obj.dynamic = tar_obj.dynamic;
    obj_risk_info.risky_obj.obj_pos = obj_cfy_info.obj_position;
    obj_risk_info.risky_obj.obj_dir = obj_cfy_info.obj_direction;
    obj_risk_info.risky_obj.lat_dist = obj_cfy_info.static_distance;
    obj_risk_info.risky_obj.lon_dist = obj_cfy_info.dist_to_tar_obj;
    obj_risk_info.risky_obj.obj_v = tar_obj.v;
    CalcRiskOfObstacle(test_ret, tar_obj, obj_cfy_info, &obj_risk_info);

    if (obj_risk_info.has_risk) {
      lane_risk_info->has_risk = true;
    }
    if (obj_risk_info.tar_v < lane_risk_info->tar_v) {
      lane_risk_info->tar_v = obj_risk_info.tar_v;
    }
    if (obj_risk_info.tar_a < lane_risk_info->tar_a) {
      lane_risk_info->tar_a = obj_risk_info.tar_a;
    }

    if (obj_risk_info.risky_obj.valid) {
      SetRiskyObj(obj_risk_info, lane_risk_info);
    }
  }

  /* k003 longjiaoy 2022-11-28 (start) */
  // 添加车道边界类型
  AnalyseLaneBoundary(data_source, ref_line_idx, lane_risk_info);
  /* k003 longjiaoy 2022-11-28 (end) */

#if ENABLE_ACTION_PLANNING_LANE_RISK_ANALYSER_TRACE
 #if 0
  printf("  车道风险信息{ 车道质量=%d"
         ", 不确定障碍=%d, 风险=%d"
         ", 目标速度=%0.1f, 目标加速度=%0.2f"
         ", 代价=%d"
         ", 前向障碍物:[ 有效=%d"
         ", 时距=%0.2f, TTC=%0.2f"
         ", 障碍物速度=%0.2f"
         " ] }"
         "\n\n"
         ,
         lane_risk_info->lane_quality,
         lane_risk_info->has_uncertain_obj, lane_risk_info->has_risk,
         lane_risk_info->tar_v*3.6F, lane_risk_info->tar_a,
         lane_risk_info->cost,
         lane_risk_info->forward_obj_info.valid,
         lane_risk_info->forward_obj_info.t_gap, lane_risk_info->forward_obj_info.ttc,
         lane_risk_info->forward_obj_info.obj_v*3.6F
         );
 #else
  LOG_INFO(5) << "  lane risk info{ lane_quality=" << lane_risk_info->lane_quality
              << ", has_uncertain_obj=" << (Int32_t)lane_risk_info->has_uncertain_obj
              << ", has_risk=" << (Int32_t)lane_risk_info->has_risk
              << ", tar_v=" << lane_risk_info->tar_v*3.6F
              << ", tar_a=" << lane_risk_info->tar_a
              << ", cost=" << lane_risk_info->cost
              << ", forward_obj:[ valid=" << (Int32_t)lane_risk_info->forward_obj_info.valid
              << ", t_gap=" << lane_risk_info->forward_obj_info.t_gap
              << ", ttc=" << lane_risk_info->forward_obj_info.ttc
              << ", obj_v=" << lane_risk_info->forward_obj_info.obj_v*3.6F
              << " ] }";
 #endif
#endif

  return true;
}

void ActionPlanningLaneRiskAnalyser::UpdateVehicleParameter(
    const ad_msg::Chassis* chassis) {
  veh_model::VehicleModelWrapper veh_model;

  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front =
      veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear =
      veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();

  param_.trailer_length = 0.0F;
  if (Nullptr_t != chassis) {
    if (ad_msg::VEH_TRAILER_STATUS_CONNECTED == chassis->trailer_status) {
      param_.vehicle_width =
          common::Max(veh_model.GetVehicleWidth(), veh_model.GetTrailerWidth());
      param_.trailer_length = veh_model.GetTrailerLength();
    }

    // LOG_INFO(5) << "Vehicle Width = " << param_.vehicle_width
    //             << ", Trailer Status = " << chassis->trailer_status;
  }
}

void ActionPlanningLaneRiskAnalyser::ClassifyObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const common::Path& path,
    plan_var_t star_s_on_path,
    ObjClassifyingInfo* obj_cfy_info) const {

  common::Vec2d tar_obj_pos(tar_obj.x, tar_obj.y);
  path.FindProjection(tar_obj_pos, &(obj_cfy_info->tar_obj_proj));
  obj_cfy_info->dist_to_tar_obj =
      obj_cfy_info->tar_obj_proj.s - star_s_on_path;

  obj_cfy_info->cut_in = false;
  plan_var_t static_dist_to_tar_obj = 0.0F;

  plan_var_t side_threshold_back =
      param_.dist_of_localization_to_rear + 2.0F*tar_obj.obb.half_length + 2.0F
      + param_.trailer_length;
  if (!tar_obj.dynamic) {
    // Static obstacle
    static_dist_to_tar_obj = test_ret.static_distance;
  } else {
    // Dynamic obstacle
    common::OBBox2d obb_check;
    obb_check.set_unit_direction(
          common::com_cos(obj_cfy_info->tar_obj_proj.heading),
          common::com_sin(obj_cfy_info->tar_obj_proj.heading));
    obb_check.set_center(
          obj_cfy_info->tar_obj_proj.point +
          param_.dist_of_localization_to_center * obb_check.unit_direction_x());
    obb_check.set_extents(0.5F*param_.vehicle_length, 0.5F*param_.vehicle_width);

    common::OBBox2d obb_tar_obj;
    obb_tar_obj.set_unit_direction(
          common::com_cos(tar_obj.obb.heading),
          common::com_sin(tar_obj.obb.heading));
    obb_tar_obj.set_center(tar_obj.obb.x, tar_obj.obb.y);
    obb_tar_obj.set_extents(tar_obj.obb.half_length, tar_obj.obb.half_width);

    // 障碍物在投影点处的静态距离
    static_dist_to_tar_obj =
        common::DistOBBToOBB_2D(obb_check, obb_tar_obj);
    if ((static_dist_to_tar_obj > 0.01F) &&
        (test_ret.static_distance < 0.01F)) {
      // 当前不与轨迹碰撞，未来会碰撞，所以属于cut-in的情况
      obj_cfy_info->cut_in = true;
    }
  }

  if (obj_cfy_info->dist_to_tar_obj >= 0.0F) {
    // 障碍物位于前方
    if (param_.dist_of_localization_to_front < obj_cfy_info->dist_to_tar_obj)  {
      if (static_dist_to_tar_obj > 0.2F) {
        if (obj_cfy_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左前方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_LEFT_FRONT;
        } else {
          // 障碍物本体位于右前方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_RIGHT_FRONT;
        }
      } else {
        // 障碍物本体位于正前方
        obj_cfy_info->obj_position = driv_map::OBJ_POSITION_FRONT;
      }
    } else {
      if (static_dist_to_tar_obj > 0.2F) {
        if (obj_cfy_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左侧方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_LEFT;
        } else {
          // 障碍物本体位于右侧方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_RIGHT;
        }
      } else {
        // 障碍物本体紧靠自车
        obj_cfy_info->obj_position = driv_map::OBJ_POSITION_CLOSE;
      }
    }
  } else {
    // 障碍物位于后方
    if ((-side_threshold_back) > obj_cfy_info->dist_to_tar_obj) {
      if (static_dist_to_tar_obj > 0.2F) {
        if (obj_cfy_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左后方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_LEFT_BACK;
        } else {
          // 障碍物本体位于右后方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_RIGHT_BACK;
        }
      } else {
        // 障碍物本体位于正后方
        obj_cfy_info->obj_position = driv_map::OBJ_POSITION_BACK;
      }
    } else {
      if (static_dist_to_tar_obj > 0.2F) {
        if (obj_cfy_info->tar_obj_proj.l > 0.0F) {
          // 障碍物本体位于左侧方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_LEFT;
        } else {
          // 障碍物本体位于右侧方
          obj_cfy_info->obj_position = driv_map::OBJ_POSITION_RIGHT;
        }
      } else {
        // 障碍物本体紧靠自车
        obj_cfy_info->obj_position = driv_map::OBJ_POSITION_CLOSE;
      }
    }
  }

  obj_cfy_info->static_distance = static_dist_to_tar_obj;

  plan_var_t angle_diff = common::com_abs(
        common::AngleDiff(tar_obj.obb.heading,
                          obj_cfy_info->tar_obj_proj.heading));
  if (angle_diff < common::com_deg2rad(60.0F)) {
    obj_cfy_info->obj_direction = driv_map::OBJ_DIRECTION_FORWARD;
  } else if (angle_diff > common::com_deg2rad(120.0F)) {
    obj_cfy_info->obj_direction = driv_map::OBJ_DIRECTION_BACKWARD;
  } else {
    obj_cfy_info->obj_direction = driv_map::OBJ_DIRECTION_CROSSED;
  }

#if ENABLE_ACTION_PLANNING_LANE_RISK_ANALYSER_TRACE
  const Char_t* str_obj_position = "Unknown";
  switch (obj_cfy_info->obj_position) {
  case (driv_map::OBJ_POSITION_FRONT):
    str_obj_position = "Front";
    break;
  case (driv_map::OBJ_POSITION_LEFT_FRONT):
    str_obj_position = "Left_Front";
    break;
  case (driv_map::OBJ_POSITION_LEFT):
    str_obj_position = "Left";
    break;
  case (driv_map::OBJ_POSITION_LEFT_BACK):
    str_obj_position = "Left_Back";
    break;
  case (driv_map::OBJ_POSITION_BACK):
    str_obj_position = "Back";
    break;
  case (driv_map::OBJ_POSITION_RIGHT_BACK):
    str_obj_position = "Right_Back";
    break;
  case (driv_map::OBJ_POSITION_RIGHT):
    str_obj_position = "Right";
    break;
  case (driv_map::OBJ_POSITION_RIGHT_FRONT):
    str_obj_position = "Right_Front";
    break;
  case (driv_map::OBJ_POSITION_CLOSE):
    str_obj_position = "Closed";
    break;
  default:
    str_obj_position = "Unknown";
    break;
  }

  const Char_t* str_obj_direction = "Unknown";
  switch (obj_cfy_info->obj_direction) {
  case (driv_map::OBJ_DIRECTION_FORWARD):
    str_obj_direction = "Forward";
    break;
  case (driv_map::OBJ_DIRECTION_BACKWARD):
    str_obj_direction = "Backward";
    break;
  case (driv_map::OBJ_DIRECTION_CROSSED):
    str_obj_direction = "Crossed";
    break;
  default:
    str_obj_direction = "Unknown";
    break;
  }

 #if 0
  printf("  障碍物分类{ ID=%d"
         ", 横向距离=%0.2f, 纵向距离=%0.2f"
         ", 区域=%s, 方向=%s"
         ", 动态=%d, cut_in=%d"
         ", 碰撞信息:[ 风险值=%d"
         ", 静态距离=%0.2f, 动态距离=%0.2f"
         ", 碰撞路径长=%0.2f"
         ", 采样点(t=%0.2f,v=%0.1f)"
         " ] }"
         "\n"
         ,
         tar_obj.id,
         obj_cfy_info->static_distance, obj_cfy_info->dist_to_tar_obj,
         str_obj_position, str_obj_direction,
         tar_obj.dynamic, obj_cfy_info->cut_in,
         test_ret.risk_value,
         test_ret.static_distance, test_ret.dynamic_distance,
         test_ret.collision_s,
         test_ret.obj_traj_point.relative_time, test_ret.obj_traj_point.v*3.6F
         );
 #else
  LOG_INFO(5) << "  Classify obj{ ID=" << tar_obj.id
              << ", lat_dist=" << obj_cfy_info->static_distance
              << ", lon_dist=" << obj_cfy_info->dist_to_tar_obj
              << ", position=" << str_obj_position
              << ", direction=" << str_obj_direction
              << ", dynamic=" << (Int32_t)tar_obj.dynamic
              << ", cut_in=" << (Int32_t)obj_cfy_info->cut_in
              << ", collision info:[ risk_value=" << test_ret.risk_value
              << ", static_dist=" << test_ret.static_distance
              << ", dynamic_dist=" << test_ret.dynamic_distance
              << ", collision_s=" << test_ret.collision_s
              << ", obj_trj_point(t=" << test_ret.obj_traj_point.relative_time
              << ", v=" << test_ret.obj_traj_point.v*3.6F
              << ") ] }";
 #endif
#endif
}

void ActionPlanningLaneRiskAnalyser::CalcRiskOfObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  if (tar_obj.dynamic) {
    // 动态
    switch (obj_cfy_info.obj_position) {
    case (driv_map::OBJ_POSITION_FRONT): {
      // 正前方障碍物
      CalcRiskOfFrontDynamicObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_LEFT_FRONT):
      // 左前方障碍物
    case (driv_map::OBJ_POSITION_RIGHT_FRONT): {
      // 右前方障碍物
      CalcRiskOfLateralFrontDynamicObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_CLOSE): {
      // 紧靠车辆的障碍物
      CalcRiskOfClosedDynamicObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_LEFT):
      // 左侧障碍物
    case (driv_map::OBJ_POSITION_RIGHT): {
      // 右侧障碍物
      // >> 侧方障碍物
      CalcRiskOfLateralDynamicObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_LEFT_BACK):
      // 左后方障碍物
    case (driv_map::OBJ_POSITION_RIGHT_BACK): {
      // 右后方障碍物
      // >> 侧后方障碍物
      CalcRiskOfLateralBackDynamicObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_BACK): {
      // 正后方障碍物
      CalcRiskOfBackDynamicObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    default:
      break;
    }
  } else {
    // 静态
    switch (obj_cfy_info.obj_position) {
    case (driv_map::OBJ_POSITION_FRONT): {
      // 正前方障碍物
      CalcRiskOfFrontStaticObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_LEFT_FRONT):
      // 左前方障碍物
    case (driv_map::OBJ_POSITION_RIGHT_FRONT): {
      // 右前方障碍物
      // >> 侧前方障碍物
      CalcRiskOfLateralFrontStaticObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_CLOSE): {
      // 紧靠车辆的障碍物
      CalcRiskOfClosedStaticObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_LEFT):
      // 左侧障碍物
    case (driv_map::OBJ_POSITION_RIGHT): {
      // 右侧障碍物
      // >> 侧方障碍物
      CalcRiskOfLateralStaticObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_LEFT_BACK):
      // 左后方障碍物
    case (driv_map::OBJ_POSITION_RIGHT_BACK): {
      // 右后方障碍物
      // >> 侧后方障碍物
      CalcRiskOfLateralBackStaticObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    case (driv_map::OBJ_POSITION_BACK): {
      // 正后方障碍物
      CalcRiskOfBackStaticObstacle(
            test_ret, tar_obj, obj_cfy_info, risk_info);
    }
      break;

    default:
      break;
    }
  }

#if ENABLE_ACTION_PLANNING_LANE_RISK_ANALYSER_TRACE
 #if 0
  printf("  风险信息{ 风险=%d"
         ", 目标速度=%0.1f, 目标加速度=%0.2f"
         ", 前向障碍物:[ 有效=%d"
         ", 时距=%0.2f, TTC=%0.2f"
         ", 障碍物速度=%0.2f"
         " ] }"
         "\n"
         ,
         risk_info->has_risk,
         risk_info->tar_v*3.6F, risk_info->tar_a,
         risk_info->forward_obj_info.valid,
         risk_info->forward_obj_info.t_gap, risk_info->forward_obj_info.ttc,
         risk_info->forward_obj_info.obj_v*3.6F
         );
 #else
  LOG_INFO(5) << "  risk info{ risk=" << (Int32_t)risk_info->has_risk
              << ", tar_v=" << risk_info->tar_v*3.6F
              << ", tar_a=" << risk_info->tar_a
              << ", forward_obj:[ valid=" << (Int32_t)risk_info->forward_obj_info.valid
              << ", t_gap=" << risk_info->forward_obj_info.t_gap
              << ", ttc=" << risk_info->forward_obj_info.ttc
              << ", obj_v=" << risk_info->forward_obj_info.obj_v*3.6F
              << " ] }";
 #endif
#endif
}

// 正前方障碍物 (动态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfFrontDynamicObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
  /* D002 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离不减去到车定位点
  /* D002 fuyuanyi 2023-11-1 (end) */
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj;
  // 时距
  plan_var_t t_gap = distance_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  plan_var_t v_diff = 0.0F;
  plan_var_t tar_v = 0.0F;
  if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
    tar_v = 0.0F;
    v_diff = -tar_obj.v - curr_v;
  } else if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_CROSSED) {
    tar_v = 0.0F;
    v_diff = -curr_v;
  } else {
    tar_v = tar_obj.v;
    v_diff = tar_obj.v - curr_v;
  }
  plan_var_t ttc = 100.0F;
  plan_var_t tar_a = 0.0F;
  if (v_diff < -0.01F) {
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
    ttc = distance_to_tar_obj / (-v_diff);
    plan_var_t t = ttc;
    if (t < 0.1F) {
      t = 0.1F;
    }
    tar_a = (tar_v - curr_v) / t;
  }

  risk_info->tar_v = tar_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
    // 正前方障碍物,逆向行驶,有风险
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  } else {
    // 正前方障碍物,同向行驶
    if ((abs_t_gap < param_.safe_time_gap_of_front_dynamic_obj) ||
        (ttc < param_.safe_ttc_of_front_dynamic_obj)) {
      // 时距小或TTC小,有风险
      risk_info->has_risk = true;
      risk_info->risky_obj.valid = true;
    } else {
      // 时距大且TTC大,无风险
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
      if (distance_to_tar_obj < 25.0F) {
        risk_info->has_risk = true;
        risk_info->risky_obj.valid = true;
      }
    }
  }
}

// 侧前方障碍物 (动态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfLateralFrontDynamicObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {
  plan_var_t lat_dist_limit = 0.5F;
  if (driv_map::OBJ_POSITION_LEFT_FRONT == obj_cfy_info.obj_position) {
    lat_dist_limit = common::Max(
          lane_info_.left_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  } else {
    lat_dist_limit = common::Max(
          lane_info_.right_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  }

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
  /* D002 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离不减去到车定位点
  /* D002 fuyuanyi 2023-11-1 (end) */
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj;
  // 时距
  plan_var_t t_gap = distance_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  plan_var_t v_diff = 0.0F;
  plan_var_t tar_v = 0.0F;
  if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
    if (obj_cfy_info.cut_in) {
      tar_v = 0.0F;
      v_diff = -tar_obj.v - v_for_plan;
    } else {
      plan_var_t ratio = 0.0F;
      Int32_t lower = common::LerpInOrderedTable(
            param_.relative_velocity_limit_table_by_dist_to_static_obj,
            test_ret.static_distance, &ratio);
      plan_var_t relative_velocity_limit = common::Lerp(
            param_.relative_velocity_limit_table_by_dist_to_static_obj[
            lower].value,
            param_.relative_velocity_limit_table_by_dist_to_static_obj[
            lower+1].value, ratio);
      tar_v = relative_velocity_limit;
      v_diff = tar_v - v_for_plan;
    }
  } else if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_CROSSED) {
    if (test_ret.risk_value > 50) {
      tar_v = 0.0F;
      v_diff = -curr_v;
    } else {
      plan_var_t ratio = 0.0F;
      Int32_t lower = common::LerpInOrderedTable(
            param_.relative_velocity_limit_table_by_dist_to_static_obj,
            test_ret.static_distance, &ratio);
      plan_var_t relative_velocity_limit = common::Lerp(
            param_.relative_velocity_limit_table_by_dist_to_static_obj[
            lower].value,
            param_.relative_velocity_limit_table_by_dist_to_static_obj[
            lower+1].value, ratio);
      tar_v = relative_velocity_limit;
      v_diff = tar_v - v_for_plan;
    }
  } else {
    if (obj_cfy_info.cut_in) {
      tar_v = tar_obj.v;
      v_diff = tar_obj.v - curr_v;
    } else {
      plan_var_t ratio = 0.0F;
      Int32_t lower = common::LerpInOrderedTable(
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
            test_ret.static_distance, &ratio);
      plan_var_t relative_velocity_limit = common::Lerp(
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
            lower].value,
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
            lower+1].value, ratio);
      tar_v = tar_obj.v + relative_velocity_limit;
      v_diff = tar_v - v_for_plan;
    }
  }

  plan_var_t ttc = 100.0F;
  plan_var_t tar_a = 0.0F;
  if (v_diff < -0.01F) {
    ttc = obj_cfy_info.dist_to_tar_obj / (-v_diff);
    plan_var_t t = ttc;
    if (t < 0.1F) {
      t = 0.1F;
    }
    tar_a = (tar_v - v_for_plan) / t;
  }

  risk_info->tar_v = curr_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
    // 侧前方障碍物,逆向行驶
    if (obj_cfy_info.cut_in) {
      // 切入车道,有风险
      risk_info->has_risk = true;
      risk_info->risky_obj.valid = true;
    } else {
      if (obj_cfy_info.static_distance < lat_dist_limit) {
        // 横向距离小,有风险
        /* k006 pengc 2022-2-12 (begin) */
        // 应该在横向距离较小时判断纵向距离是否足够大，来判断风险, bug:1285
        // 删除了在横向距离较大时判断风险
        if ((abs_t_gap < param_.safe_time_gap_of_front_dynamic_obj) ||
            (ttc < param_.safe_ttc_of_front_dynamic_obj)) {
          // 时距小或TTC小,有风险
          risk_info->has_risk = true;
          risk_info->risky_obj.valid = true;
        } else {
          // 时距大且TTC大,无风险
        }
        /* k006 pengc 2022-2-12 (end) */
      } else {
        // 横向距离大,无风险
        /* k006 pengc 2022-2-12 (begin) */
        // 应该在横向距离较小时判断纵向距离是否足够大，来判断风险, bug:1285
        // 删除了在横向距离较大时判断风险
        /* k006 pengc 2022-2-12 (end) */
      }
    }
  } else if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_CROSSED) {
    // 侧前方障碍物,横向切入车道
    if ((test_ret.risk_value > 50) && (tar_a < -1.0F) &&
        ((abs_t_gap < param_.safe_time_gap_of_front_dynamic_obj) ||
         (ttc < param_.safe_ttc_of_front_dynamic_obj))) {
      // 风险较大，有风险
      risk_info->has_risk = true;
      risk_info->risky_obj.valid = true;
    } else {
      // 风险不大，无风险
    }
  } else {
    // 侧前方障碍物,同向行驶
    if (obj_cfy_info.cut_in) {
      // 障碍物将切入车道
      if ((abs_t_gap < param_.safe_time_gap_of_front_dynamic_obj) ||
          (ttc < param_.safe_ttc_of_front_dynamic_obj)) {
        // 时距小或TTC小,有风险
        risk_info->has_risk = true;
        risk_info->risky_obj.valid = true;
      } else {
        // 时距大且TTC大,无风险
      }
    } else {
      // 障碍物不会切入车道
      if (obj_cfy_info.static_distance < lat_dist_limit) {
        // 横向距离小
        if ((tar_a < -2.0F) &&
            ((abs_t_gap < param_.safe_time_gap_of_front_dynamic_obj) ||
             (ttc < param_.safe_ttc_of_front_dynamic_obj))) {
          // 时距小或TTC小,有风险
          risk_info->has_risk = true;
          risk_info->risky_obj.valid = true;
        } else {
          // 时距大且TTC大,无风险
        }
      } else {
        // 横向距离大,无风险
      }
    }
  }
}

// 侧方障碍物 (动态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfLateralDynamicObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t lat_dist_limit = 0.5F;

  if (driv_map::OBJ_POSITION_LEFT == obj_cfy_info.obj_position) {
    lat_dist_limit = common::Max(
          lane_info_.left_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  } else {
    lat_dist_limit = common::Max(
          lane_info_.right_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  }

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  plan_var_t tar_v = 0.0F;
  if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
    plan_var_t ratio = 0.0F;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_static_obj,
          test_ret.static_distance, &ratio);
    plan_var_t relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower+1].value, ratio);
    tar_v = relative_velocity_limit;
  } else if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_CROSSED) {
    plan_var_t ratio = 0.0F;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_static_obj,
          test_ret.static_distance, &ratio);
    plan_var_t relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower+1].value, ratio);
    tar_v = relative_velocity_limit;
  } else {
    plan_var_t ratio = 0.0F;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
          test_ret.static_distance, &ratio);
    plan_var_t relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
          lower+1].value, ratio);
    tar_v = tar_obj.v + relative_velocity_limit;
  }

  plan_var_t v_diff = tar_v - v_for_plan;
  plan_var_t ttc = 100.0F;
  plan_var_t tar_a = 0.0F;
  if (v_diff < -0.01F) {
    ttc = 20.0F / (-v_diff);
    plan_var_t t = ttc;
    if (t < 0.1F) {
      t = 0.1F;
    }
    tar_a = (tar_v - v_for_plan) / t;
  }
  risk_info->tar_v = tar_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = 0.0F;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.static_distance < lat_dist_limit) {
    // 横向距离小,有风险
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  } else {
    // 横向距离大,无风险
  }
}

// 侧后方障碍物 (动态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfLateralBackDynamicObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离v3.12版本之后为前脸到前脸3.6+挂长
  /* D001 fuyuanyi 2023-11-1 (end) */
  plan_var_t side_threshold_back =
      param_.dist_of_localization_to_rear + 2.0F
      + param_.trailer_length;
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj + side_threshold_back;

  // 时距
  plan_var_t t_gap = distance_to_tar_obj / common::Max(tar_obj.v, v_for_plan);
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  plan_var_t v_diff = tar_obj.v - curr_v;
  plan_var_t ttc = 100.0F;
  if (v_diff > 0.01F) {
    ttc = -distance_to_tar_obj / v_diff;
  }

  risk_info->tar_v = curr_v;
  risk_info->tar_a = 0.0F;
  risk_info->risky_obj.t_gap = abs_t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;

  if (obj_cfy_info.static_distance < 0.5F) {
    // 横向距离小
    if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
      // 逆向行驶,无风险
    } else {
      // 同向行驶
      if ((abs_t_gap < param_.safe_time_gap_of_back_dynamic_obj) ||
          (ttc < param_.safe_ttc_of_back_dynamic_obj)) {
        // 时距小或TTC小,有风险
        risk_info->has_risk = true;
        risk_info->risky_obj.valid = true;
      } else {
        // 时距大且TTC大,无风险
      }
    }
  } else {
    // 横向距离大,无风险
  }
}

// 正后方障碍物 (动态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfBackDynamicObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离v3.12版本之后为前脸到前脸3.6+挂长
  /* D001 fuyuanyi 2023-11-1 (end) */
  plan_var_t side_threshold_back =
      param_.dist_of_localization_to_rear + 2.0F
      + param_.trailer_length;
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj + side_threshold_back;
  // 时距
  plan_var_t t_gap = distance_to_tar_obj / common::Max(tar_obj.v, v_for_plan);
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  plan_var_t v_diff = tar_obj.v - curr_v;
  plan_var_t ttc = 100.0F;
  if (v_diff > 0.01F) {
    ttc = -distance_to_tar_obj / v_diff;
  }

  risk_info->tar_v = curr_v;
  risk_info->tar_a = 0.0F;
  risk_info->risky_obj.t_gap = abs_t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
    // 逆向行驶,无风险
  } else {
    // 同向行驶
    if ((abs_t_gap < param_.safe_time_gap_of_back_dynamic_obj) ||
        (ttc < param_.safe_ttc_of_back_dynamic_obj)) {
      // 时距小或TTC小,有风险
      risk_info->has_risk = true;
      risk_info->risky_obj.valid = true;
    } else {
      // 时距大且TTC大,无风险
    }
  }
  /* k007 longjiaoy 2023-03-02 (start) */
  // 在一定范围内有障碍物不允许变道
  plan_var_t no_change_length = 15.0F;
  if (common::com_abs(distance_to_tar_obj) <= no_change_length) {
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  }
  /* k007 longjiaoy 2023-03-02 (end) */
}

// 紧靠障碍物 (动态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfClosedDynamicObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t lat_dist_limit = 0.5F;

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  plan_var_t tar_v = 0.0F;

  plan_var_t tar_a = 0.0F;
  if (tar_v < (curr_v-5.0F/3.6F)) {
    tar_a = -2.0F;
  }
  risk_info->tar_v = curr_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = 0.0F;
  risk_info->risky_obj.ttc = 0.0F;

  risk_info->has_risk = false;
  if (obj_cfy_info.static_distance < lat_dist_limit) {
    // 横向距离小,有风险
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  } else {
    // 横向距离大,无风险
  }
}

// 正前方障碍物 (静态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfFrontStaticObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
  /* D002 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离不减去到车定位点
  /* D002 fuyuanyi 2023-11-1 (end) */
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj;

  // 时距
  plan_var_t t_gap = distance_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);

  plan_var_t v_diff = tar_obj.v - curr_v;
  plan_var_t ttc = 100.0F;
  if (v_diff < -0.01F) {
    ttc = distance_to_tar_obj / -v_diff;
  }

  plan_var_t tar_v = 0.0F;
  plan_var_t t = distance_to_tar_obj / v_for_plan;
  if (t < 0.1F) {
    t = 0.1F;
  }
  plan_var_t tar_a = -curr_v / t;

  risk_info->tar_v = tar_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = abs_t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if ((abs_t_gap < param_.safe_time_gap_of_front_static_obj) ||
      (ttc < param_.safe_ttc_of_front_static_obj)) {
    // 时距小或TTC小,有风险
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  } else {
    // 时距大且TTC大,无风险
  }

  /* k007 longjiaoy 2023-03-02 (start) */
  // 在一定范围内有障碍物不允许变道
  plan_var_t no_change_length = 15.0F +  param_.dist_of_localization_to_front + 1.0F;
  if (common::com_abs(obj_cfy_info.dist_to_tar_obj) <= no_change_length) {
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  }
  /* k007 longjiaoy 2023-03-02 (end) */
}

// 侧前方障碍物 (静态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfLateralFrontStaticObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {
  plan_var_t lat_dist_limit = 0.5F;

  if (driv_map::OBJ_POSITION_LEFT_FRONT == obj_cfy_info.obj_position) {
    lat_dist_limit = common::Max(
          lane_info_.left_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  } else {
    lat_dist_limit = common::Max(
          lane_info_.right_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  }

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
  /* D002 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离不减去到车定位点
  /* D002 fuyuanyi 2023-11-1 (end) */
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj;
  // 时距
  plan_var_t t_gap = distance_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);

  plan_var_t ratio = 0.0F;
  Int32_t lower = common::LerpInOrderedTable(
        param_.relative_velocity_limit_table_by_dist_to_static_obj,
        test_ret.static_distance, &ratio);
  plan_var_t relative_velocity_limit = common::Lerp(
        param_.relative_velocity_limit_table_by_dist_to_static_obj[
        lower].value,
        param_.relative_velocity_limit_table_by_dist_to_static_obj[
        lower+1].value, ratio);
  plan_var_t tar_v = relative_velocity_limit;
  plan_var_t ttc = 100.0F;
  if (tar_v < (curr_v-1.0F/3.6F)) {
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
    ttc = distance_to_tar_obj / -(tar_v-v_for_plan);
  }
  /* D001 fuyuanyi 2023-5-12 (begin) */
  // 纵向距离减去到车定位点
  /* D001 fuyuanyi 2023-5-12 (end) */
  plan_var_t t = distance_to_tar_obj / v_for_plan;
  if (t < 0.1F) {
    t = 0.1F;
  }
  plan_var_t tar_a = (tar_v - curr_v) / t;
  if (tar_a > 0.0F) {
    tar_a = 0.0F;
  }

  risk_info->tar_v = tar_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = abs_t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.static_distance < lat_dist_limit) {
    // 横向距离小
    if ((abs_t_gap < param_.safe_time_gap_of_front_static_obj) ||
        (ttc < param_.safe_ttc_of_front_static_obj)) {
      // 时距小或TTC小,有风险
      risk_info->has_risk = true;
      risk_info->risky_obj.valid = true;
    } else {
      // 时距大且TTC大,无风险
    }
  } else {
    // 横向距离大,不受控
  }
}

// 侧方障碍物 (静态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfLateralStaticObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {
  plan_var_t lat_dist_limit = 0.5F;

  if (driv_map::OBJ_POSITION_LEFT_BACK == obj_cfy_info.obj_position) {
    lat_dist_limit = common::Max(
          lane_info_.left_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  } else {
    lat_dist_limit = common::Max(
          lane_info_.right_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  }

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  plan_var_t ratio = 0.0F;
  Int32_t lower = common::LerpInOrderedTable(
        param_.relative_velocity_limit_table_by_dist_to_static_obj,
        test_ret.static_distance, &ratio);
  plan_var_t relative_velocity_limit = common::Lerp(
        param_.relative_velocity_limit_table_by_dist_to_static_obj[
        lower].value,
        param_.relative_velocity_limit_table_by_dist_to_static_obj[
        lower+1].value, ratio);
  plan_var_t tar_v = relative_velocity_limit;
  plan_var_t v_diff = tar_v - v_for_plan;

  plan_var_t ttc = 100.0F;
  plan_var_t tar_a = 0.0F;
  if (v_diff < -0.01F) {
    ttc = 20.0F / (-v_diff);
    plan_var_t t = ttc;
    if (t < 0.1F) {
      t = 0.1F;
    }
    tar_a = (tar_v - v_for_plan) / t;
  }
  risk_info->tar_v = tar_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = 0.0F;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.static_distance < lat_dist_limit) {
    // 横向距离小,有风险
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  } else {
    // 横向距离大,无风险
  }
}

// 侧后方障碍物 (静态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfLateralBackStaticObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {
  plan_var_t lat_dist_limit = 0.5F;

  if (driv_map::OBJ_POSITION_LEFT_FRONT == obj_cfy_info.obj_position) {
    lat_dist_limit = common::Max(
          lane_info_.left_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  } else {
    lat_dist_limit = common::Max(
          lane_info_.right_lane_width - 0.5F*param_.vehicle_width, 0.4F);
  }

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离v3.12版本之后为前脸到前脸3.6+挂长
  /* D001 fuyuanyi 2023-11-1 (end) */
  plan_var_t side_threshold_back =
      param_.dist_of_localization_to_rear + 2.0F
      + param_.trailer_length;
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj + side_threshold_back;
  // 时距
  plan_var_t t_gap = distance_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  plan_var_t v_diff = tar_obj.v - curr_v;
  plan_var_t ttc = 100.0F;
  if (v_diff > 0.01F) {
    ttc = -distance_to_tar_obj / -v_diff;
  }

  risk_info->tar_v = curr_velocity_;
  risk_info->tar_a = 0.0F;
  risk_info->risky_obj.t_gap = abs_t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if (obj_cfy_info.static_distance < 0.5F) {
    // 横向距离小
    if ((abs_t_gap < param_.safe_time_gap_of_back_static_obj) ||
        (ttc < param_.safe_ttc_of_back_static_obj)) {
      // 时距小或TTC小,有风险
      risk_info->has_risk = true;
      risk_info->risky_obj.valid = true;
    } else {
      // 时距大且TTC大,无风险
    }
  } else {
    // 横向距离大,无风险
  }
}

// 正后方障碍物 (静态)
void ActionPlanningLaneRiskAnalyser::CalcRiskOfBackStaticObstacle(
    const driv_map::CollisionTestResult::ObjInfo& test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ObjClassifyingInfo& obj_cfy_info,
    ObjRiskInfo* risk_info) const {

  plan_var_t curr_v = curr_velocity_;
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }
  /* D001 fuyuanyi 2023-11-1 (begin) */
  // 纵向距离v3.12版本之后为前脸到前脸3.6+挂长
  /* D001 fuyuanyi 2023-11-1 (end) */
  plan_var_t side_threshold_back =
      param_.dist_of_localization_to_rear + 2.0F
      + param_.trailer_length;
  plan_var_t distance_to_tar_obj = obj_cfy_info.dist_to_tar_obj + side_threshold_back;
  // 时距
  plan_var_t t_gap = distance_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);

  plan_var_t v_diff = tar_obj.v - curr_v;
  plan_var_t ttc = 100.0F;
  if (v_diff > 0.01F) {
    ttc = -distance_to_tar_obj / -v_diff;
  }

  risk_info->tar_v = curr_velocity_;
  risk_info->tar_a = 0.0F;
  risk_info->risky_obj.t_gap = abs_t_gap;
  risk_info->risky_obj.ttc = ttc;

  risk_info->has_risk = false;
  if ((abs_t_gap < param_.safe_time_gap_of_back_static_obj) ||
      (ttc < param_.safe_ttc_of_back_static_obj)) {
    // 时距小或TTC小,有风险
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  } else {
    // 时距大且TTC大,无风险
  }

  /* k007 longjiaoy 2023-03-02 (start) */
  // 在一定范围内有障碍物不允许变道
  plan_var_t no_change_length = 15.0F;
  if (common::com_abs(distance_to_tar_obj) <= no_change_length) {
    risk_info->has_risk = true;
    risk_info->risky_obj.valid = true;
  }
  /* k007 longjiaoy 2023-03-02 (end) */
}

 // 紧靠障碍物(静态）
void ActionPlanningLaneRiskAnalyser::CalcRiskOfClosedStaticObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle &tar_obj,
    const ObjClassifyingInfo &obj_cfy_info, ObjRiskInfo *risk_info) const {

  plan_var_t curr_v = curr_velocity_;
  plan_var_t ratio = 0.0F;
  Int32_t lower = common::LerpInOrderedTable(
        param_.relative_velocity_limit_table_by_dist_to_static_obj,
        test_ret.static_distance, &ratio);
  plan_var_t relative_velocity_limit = common::Lerp(
        param_.relative_velocity_limit_table_by_dist_to_static_obj[
        lower].value,
        param_.relative_velocity_limit_table_by_dist_to_static_obj[
        lower+1].value, ratio);
  plan_var_t tar_v = relative_velocity_limit;
  plan_var_t tar_a = 0.0F;
  if (tar_v < (curr_v - 5.0F/3.6F)) {
    tar_a = -2.0F;
  }
  risk_info->tar_v = tar_v;
  risk_info->tar_a = tar_a;
  risk_info->risky_obj.t_gap = 0.0F;
  risk_info->risky_obj.ttc = 0.0F;

  risk_info->has_risk = true;
  risk_info->risky_obj.valid = true;
}


/* k003 longjiaoy 2022-11-28 (start) */
void ActionPlanningLaneRiskAnalyser::AnalyseLaneBoundary(
    const ActionPlanningDataSource& data_source,
    Int32_t ref_line_idx, LaneRiskInfo* lane_risk_info) {
  lane_risk_info->boundary.allow_to_change_left = true;
  lane_risk_info->boundary.allow_to_change_right = true;
  lane_risk_info->boundary.left_width = 0.0F;
  lane_risk_info->boundary.right_width = 0.0F;

  const driv_map::DrivingMapWrapper* driving_map = data_source.driving_map;
  plan_var_t curr_proj_s =
      driving_map->GetProjDistOfCurrPositionOnRef(ref_line_idx);

  for (Int32_t i = 0; i < 5; ++i) {
    Int32_t lane_idx = -1;
    Float32_t s_on_lane = 0.0F;
    driving_map->FindReferenceLineLaneSegmentByProjOnRef(
          ref_line_idx, curr_proj_s+curr_velocity_*i, &lane_idx, &s_on_lane);
    if (driving_map->IsValidLaneIndex(lane_idx)) {
      // Boundary type
      Int32_t left_boundary_type =
          static_cast<Int32_t>(driv_map::LANE_BOUNDARY_TYPE_UNKNOWN);
      Int32_t right_boundary_type =
          static_cast<Int32_t>(driv_map::LANE_BOUNDARY_TYPE_UNKNOWN);
      driving_map->GetLaneBoundaryType(
            lane_idx, s_on_lane, &left_boundary_type, &right_boundary_type);
      if (!IsAllowToChangLaneByBoundaryType(left_boundary_type)) {
        lane_risk_info->boundary.allow_to_change_left = false;
      }
      if (!IsAllowToChangLaneByBoundaryType(right_boundary_type)) {
        lane_risk_info->boundary.allow_to_change_right = false;
      }

      // Lane width
      plan_var_t left_width = 0.0F;
      plan_var_t right_width = 0.0F;
      driving_map->GetLaneWidth(lane_idx, s_on_lane, &left_width, &right_width);
      if (0 == i) {
        lane_risk_info->boundary.left_width = left_width;
        lane_risk_info->boundary.right_width = right_width;
      } else {
        if (left_width < lane_risk_info->boundary.left_width) {
          lane_risk_info->boundary.left_width = left_width;
        }
        if (right_width < lane_risk_info->boundary.right_width) {
          lane_risk_info->boundary.right_width = right_width;
        }
      }
    } else {
      LOG_ERR << "Detected invalid lane index (" << lane_idx << ")";
      lane_risk_info->boundary.allow_to_change_left = false;
      lane_risk_info->boundary.allow_to_change_right = false;
    }
  }
}

bool ActionPlanningLaneRiskAnalyser::IsAllowToChangLaneByBoundaryType(
    Int32_t boundary_type) {
  bool ret = true;

  switch (boundary_type) {
  case (driv_map::LANE_BOUNDARY_TYPE_UNKNOWN):
    ret = true;
    break;
  case (driv_map::LANE_BOUNDARY_TYPE_DOTTED_YELLOW):
    ret = true;
    break;
  case (driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE):
    ret = true;
    break;
  case (driv_map::LANE_BOUNDARY_TYPE_SOLID_YELLOW):
    ret = false;
    break;
  case (driv_map::LANE_BOUNDARY_TYPE_SOLID_WHITE):
    ret = false;
    break;
  case (driv_map::LANE_BOUNDARY_TYPE_DOUBLE_YELLOW):
    ret = false;
    break;
  case (driv_map::LANE_BOUNDARY_TYPE_CURB):
    ret = false;
    break;
  default:
    ret = true;
    break;
  }

  return (ret);
}

void ActionPlanningLaneRiskAnalyser::SetRiskyObj(
    const ObjRiskInfo& obj_risk_info, LaneRiskInfo* lane_risk_info) {
  if (!obj_risk_info.risky_obj.valid) {
    return;
  }

  Int32_t pos_idx = obj_risk_info.risky_obj.obj_pos;

  RiskyObj& nearest_obj = lane_risk_info->nearest_obj[pos_idx];
  if (nearest_obj.valid) {
    if (obj_risk_info.risky_obj.t_gap < nearest_obj.t_gap) {
      nearest_obj = obj_risk_info.risky_obj;
    }
  } else {
    nearest_obj = obj_risk_info.risky_obj;
  }

  RiskyObj& min_ttc_obj = lane_risk_info->min_ttc_obj[pos_idx];
  if (min_ttc_obj.valid) {
    if (obj_risk_info.risky_obj.ttc < min_ttc_obj.ttc) {
      min_ttc_obj = obj_risk_info.risky_obj;
    }
  } else {
    min_ttc_obj = obj_risk_info.risky_obj;
  }
}

/* k003 longjiaoy 2022-11-28 (end) */


}  // namespace planning
}  // namespace phoenix

