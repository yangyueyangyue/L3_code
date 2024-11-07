//

#include "action_planning_classify_obstacle.h"
#include "geometry/geometry_utils.h"
#include "vehicle_model_wrapper.h"

#define ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE (1)


namespace phoenix {
namespace planning {


ObjClassify ::ObjClassify () {
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
  // 与障碍物之间的碰撞风险对应的Cost Ratio，避免碰撞
  // (乘以因为障碍物导致的减速度的绝对值)
  param_.collision_deceleration_ratio = 200;
  param_.tar_velocity_ratio = 10;
  param_.routing_ratio = 10;
  param_.lane_offest_cost = -100;
  param_.safe_ttc = 10.0F;
  param_.safe_time_gap = 5.0F;

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

  obj_classify_result_.Clear();
  curr_lane_socre_.Clear();
}

ObjClassify::~ObjClassify() {

}

void ObjClassify::ClassifyObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle &tar_obj,
    const common::Path &path,
    plan_var_t start_s_on_path,
    ObjClassifyInfo* obj_cfy_info) const {

  common::Vec2d tar_obj_pos(tar_obj.x, tar_obj.y);
  path.FindProjection(tar_obj_pos, &(obj_cfy_info->tar_obj_proj));
  obj_cfy_info->dist_to_tar_obj =
      obj_cfy_info->tar_obj_proj.s - start_s_on_path;

  obj_cfy_info->cut_in = false;
  plan_var_t static_dist_to_tar_obj = 0.0F;
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

    static_dist_to_tar_obj =
        common::DistOBBToOBB_2D(obb_check, obb_tar_obj);

    if ((static_dist_to_tar_obj > 0.01F) &&
        (test_ret.static_distance < 0.01F)) {
      obj_cfy_info->cut_in = true;
    }
  }

  if (obj_cfy_info->dist_to_tar_obj >= 0.0F) {
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
    if ((-param_.dist_of_localization_to_rear) > obj_cfy_info->dist_to_tar_obj) {
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


#if (ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE)
  std::cout << "  目标障碍物ID=" << tar_obj.id << std::endl;
  std::cout << "  静态距离="
            << obj_cfy_info->static_distance << std::endl;
  std::cout << "  到目标障碍物的距离="
            << obj_cfy_info->dist_to_tar_obj << std::endl;
  std::cout << "  目标障碍物所在区域="
            << obj_cfy_info->obj_position << std::endl;
  std::cout << "  目标障碍物方向="
            << obj_cfy_info->obj_direction << std::endl;
  std::cout << "  动态="
            << (Int32_t)tar_obj.dynamic << std::endl;
  std::cout << "  cut_in=" << obj_cfy_info->cut_in << std::endl;
  std::cout << "  碰撞信息: "
            << "风险值(" << test_ret.risk_value
            << ") 静态距离(" << test_ret.static_distance
            << ") 动态距离(" << test_ret.dynamic_distance
            << ") 碰撞路径长(" << test_ret.collision_s
            << ") 采样点{t=" << test_ret.obj_traj_point.relative_time
            << ",v=" << test_ret.obj_traj_point.v*3.6F
            << "} "
            << std::endl;
#endif
}

bool ObjClassify::CalcRiskofObstacleOnRef(
    const ActionPlanningDataSource &data_source,
    const Int32_t &ref_index,
    const Int32_t& routing_neighbor_flag) {
  curr_lane_socre_.Clear();
  obj_classify_result_.Clear();

  bool has_risk = false;
  Int32_t neighbor_flag =
      data_source.driving_map->GetReferenceLineNeighborFlag(ref_index);

  plan_var_t curr_proj_s =
      data_source.driving_map->GetProjDistOfCurrPositionOnRef(ref_index);

  driv_map::CollisionTestResult result;
  data_source.driving_map->GetRiskyObstacles(ref_index, true, &result);

  if (result.uncertain_list.Size() > 0) {
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    std::cout << "$$$ 拒绝变道: 车道(" << neighbor_flag
              << ") 存在不确定的障碍物" << std::endl;
#endif
    return true;
  }
  const common::Path& ref_line =
      data_source.driving_map->GetSmoothReferenceLine(ref_index);

  if (0 == neighbor_flag) {
    param_.safe_time_gap = 1.5F;
    param_.safe_ttc = 3.0F;
  }

  if ((routing_neighbor_flag * neighbor_flag) >= 0) {
    curr_lane_socre_.lane_sorce += param_.routing_ratio;
  }
  for (Int32_t i = 0; i < result.risky_obj_list.Size(); i++) {
    const driv_map::CollisionTestResult::ObjInfo& test_ret =
        result.risky_obj_list[i];
    const ad_msg::Obstacle& tar_obj =
        data_source.driving_map->GetObstacle(test_ret.obj_list_index);
    ObjClassifyInfo obj_cfy_info;
    ClassifyObstacle(test_ret, tar_obj, ref_line, curr_proj_s, &obj_cfy_info);
    if (0 == neighbor_flag) {
      if (driv_map::OBJ_POSITION_BACK == obj_cfy_info.obj_position) {
        continue;
      }
    }
    bool risk = CalcRiskofObstacle(test_ret, tar_obj, *(data_source.chassis), obj_cfy_info);
    if (risk) {
      has_risk = true;
    }
  }

  if (!obj_classify_result_.valid) {
    obj_classify_result_.tar_velocity = data_source.chassis->v;
  }
  std::cout << "tar v = "<<obj_classify_result_.tar_velocity<<std::endl;
  std::cout << "tar deceleration = "<<obj_classify_result_.deceleration<<std::endl;
  curr_lane_socre_.lane_sorce +=
      obj_classify_result_.tar_velocity * param_.tar_velocity_ratio +
      obj_classify_result_.deceleration * param_.collision_deceleration_ratio +
      common::com_abs(neighbor_flag) * param_.lane_offest_cost;

  return (has_risk);

}

bool ObjClassify::CalcRiskofObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle &tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info) {

  bool need_ctl = false;

  // 动态
  if (tar_obj.dynamic) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
    std::cout << "         动态" << std::endl;
#endif
  } else {
    std::cout << "         静态" << std::endl;
  }

  switch (obj_cfy_info.obj_position) {
  case (driv_map::OBJ_POSITION_FRONT):
    // 前方障碍物
    need_ctl = CalcDecelerationByFrontObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_LEFT_FRONT):
    // 左前障碍物
    need_ctl = CalcDecelerationByLeftOrRightFrontObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_RIGHT_FRONT):
    // 右前障碍物
    need_ctl = CalcDecelerationByLeftOrRightFrontObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_CLOSE):
    need_ctl = CalcDecelerationByCloseObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_LEFT):
    // 左侧障碍物
    need_ctl = CalcDecelerationByLeftOrRightObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_RIGHT):
    // 右侧障碍物
    need_ctl = CalcDecelerationByLeftOrRightObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);

    break;
  case (driv_map::OBJ_POSITION_LEFT_BACK):
    // 左后方障碍物
    need_ctl = CalcDecelerationByLeftOrRightBackObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_RIGHT_BACK):
    // 右后方障碍物
    need_ctl = CalcDecelerationByLeftOrRightBackObstacle(
          test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  case (driv_map::OBJ_POSITION_BACK):
    // 后方障碍物
    need_ctl = CalcDecelerationByBackObstacle(test_ret, tar_obj, chassis, obj_cfy_info);
    break;
  default:
    break;
  }


  return (need_ctl);
}

bool ObjClassify::CalcDecelerationByFrontObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info) {
  bool need_ctl = false;

  // 根据障碍物目标速度
  plan_var_t velocity_limit = 0.0F;
  // 根据障碍物减速距离
  plan_var_t s = test_ret.collision_s;
  // 与障碍物距离相关的目标速度
  plan_var_t relative_velocity_limit = 0.0F;


  plan_var_t curr_v = 20.0F/3.6F;
  if (chassis.v_valid) {
    curr_v = chassis.v;
  }
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  // 时距
  plan_var_t t_gap = obj_cfy_info.dist_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  plan_var_t abs_angle_diff = common::com_abs(common::AngleDiff(
                                                obj_cfy_info.tar_obj_proj.heading,
                                                tar_obj.obb.heading));

  if (tar_obj.dynamic) {
    if (driv_map::OBJ_DIRECTION_BACKWARD ==
        obj_cfy_info.obj_direction) {
      //前方逆行障碍物
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << "         正前方障碍物,逆向行驶,受控" << std::endl;
#endif
      need_ctl = true;
      velocity_limit = 0.0F;
      s = common::Min(s, obj_cfy_info.dist_to_tar_obj);
    } else {
      // 正前方障碍物,同向行驶
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << "         正前方障碍物,同向行驶";
#endif  
      plan_var_t v_diff = tar_obj.v - curr_v;
      plan_var_t ttc = 100.0F;
      if (v_diff < -0.01F) {
        ttc = obj_cfy_info.dist_to_tar_obj / -v_diff;
      }

      if (abs_t_gap < param_.safe_time_gap || ttc < param_.safe_ttc) {

        need_ctl = true;
        velocity_limit = tar_obj.v * common::com_cos(abs_angle_diff);
        s = common::Min(s, obj_cfy_info.dist_to_tar_obj);

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")小 或 ttc(" << ttc
                  << ")小,受控" << std::endl;
#endif
      } else {
        plan_var_t t = 0;
        Int32_t lower = common::LerpInOrderedTable(
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
              obj_cfy_info.static_distance, &t);
        relative_velocity_limit = common::Lerp(
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
              lower].value,
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
              lower+1].value, t);
          velocity_limit =
              tar_obj.v * common::com_cos(abs_angle_diff) +
              relative_velocity_limit;

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")大 且 ttc(" << ttc
                  << ")大,不受控" << std::endl;
#endif
      }

    }
  } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
    std::cout << "         正前方障碍物";
#endif
    // 静态
    plan_var_t v_diff = tar_obj.v - curr_v;
    plan_var_t ttc = 100.0F;
    if (v_diff < -0.01F) {
      ttc = obj_cfy_info.dist_to_tar_obj / -v_diff;
    }
    if (abs_t_gap < param_.safe_time_gap || ttc < param_.safe_time_gap) {
      need_ctl = true;

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",时距("<< t_gap
                << ")小 或 ttc(" << ttc
                << ")小,受控" << std::endl;
#endif

    } else {

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",时距("<< t_gap
                << ")大 且 ttc(" << ttc
                << ")大,不受控" << std::endl;
#endif
    }

    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_static_obj,
          obj_cfy_info.static_distance, &t);
    relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower+1].value, t);
    if (v_for_plan > relative_velocity_limit) {
      velocity_limit =
          tar_obj.v +
          relative_velocity_limit;
    } else {
      velocity_limit = v_for_plan;
    }
  }

  if (velocity_limit < (2.0f/3.6f)) {
    velocity_limit = 0.0f;
  }

  plan_var_t t = 2.0f*s /
      (v_for_plan + velocity_limit);
  if (t < 1.0f) {
    t = 1.0f;
  }

  plan_var_t a = (velocity_limit - v_for_plan) / t;
  if (a > 0.0F) {
    a = 0.0F;
  }

  if (!obj_classify_result_.valid) {
    obj_classify_result_.deceleration = a;
    obj_classify_result_.tar_velocity = velocity_limit;
    obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
    obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    obj_classify_result_.valid = true;
  } else {
    if (a <= obj_classify_result_.deceleration) {
      obj_classify_result_.deceleration = a;
      obj_classify_result_.tar_velocity = velocity_limit;
      obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
      obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    }
  }

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         规划速度= "<< velocity_limit * 3.6
            << "km/h, 减速度= " << a << std::endl;
#endif

  return (need_ctl);
}

bool ObjClassify::CalcDecelerationByLeftOrRightFrontObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info) {
  bool need_ctl = false;
  // 根据障碍物目标速度
  plan_var_t velocity_limit = 0.0F;
  // 根据障碍物减速距离
  plan_var_t s = test_ret.collision_s;
  // 与障碍物距离相关的目标速度
  plan_var_t relative_velocity_limit = 0.0F;

  plan_var_t curr_v = 20.0F/3.6F;
  if (chassis.v_valid) {
    curr_v = chassis.v;
  }
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  // 时距
  plan_var_t t_gap = s / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         侧前方障碍物" ;
#endif
  if (tar_obj.dynamic) {
    s = common::Min(obj_cfy_info.dist_to_tar_obj,
                    test_ret.collision_s);

    if (obj_cfy_info.obj_direction ==
        driv_map::OBJ_DIRECTION_BACKWARD) {

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",逆向行驶" << std::endl;
#endif
      if (obj_cfy_info.cut_in) {
        need_ctl = true;
        velocity_limit = 0.0F;

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << "切入车道,受控" << std::endl;
#endif
      } else {
        if (obj_cfy_info.static_distance < 0.5F) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << "横向距离小,受控" << std::endl;
#endif
          need_ctl = true;
          velocity_limit = 0.0F;
        } else {

          velocity_limit = tar_obj.v;
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << "横向距离大,不受控" << std::endl;
#endif
        }
      }

    } else if (obj_cfy_info.obj_direction ==
               driv_map::OBJ_DIRECTION_CROSSED) {
      // 侧前方障碍物,横向切入车道
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向切入车道";
#endif
      if (test_ret.risk_value > 50) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ", 风险较大，需受控" << std::endl;
#endif
        need_ctl = true;
        velocity_limit = 0.0F;

      } else {
        velocity_limit = tar_obj.v;
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ", 风险不大，不需受控" << std::endl;
#endif
      }
    } else {
      // 侧前方障碍物,同向行驶
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",同向行驶";
#endif

      if (obj_cfy_info.cut_in) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",切入车道";
#endif
        plan_var_t v_diff = tar_obj.v - curr_v;
        plan_var_t abs_angle_diff = common::com_abs(
              common::AngleDiff(obj_cfy_info.tar_obj_proj.heading, tar_obj.obb.heading));

        plan_var_t ttc = 100.0F;
        if (v_diff < -0.01F) {
          ttc = s / -v_diff;
        }
        if (abs_t_gap < param_.safe_time_gap || ttc < param_.safe_ttc) {

          need_ctl = true;
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << ",时距("<< t_gap
                    << ")小 或 ttc(" << ttc
                    << ")小,受控" << std::endl;
#endif
        } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << ",时距("<< t_gap
                    << ")大 且 ttc(" << ttc
                    << ")大,不受控" << std::endl;
#endif
        }
        velocity_limit = tar_obj.v * common::com_cos(abs_angle_diff);

      } else {
        // 障碍物不会切入车道
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",不会切入车道";
#endif
        if (obj_cfy_info.static_distance < 0.5F) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << ",横向距离小";
#endif

          plan_var_t v_diff = tar_obj.v - curr_v;
          plan_var_t ttc = 100.0F;
          if (v_diff < -0.01F) {
            ttc = s / -v_diff;
          }
          if (abs_t_gap < param_.safe_time_gap || ttc < param_.safe_ttc) {
            need_ctl = true;


#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
            std::cout << ",时距("<< t_gap
                      << ")小 或 ttc(" << ttc
                      << ")小,受控" << std::endl;
#endif
          } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
            std::cout << ",时距("<< t_gap
                      << ")大 且 ttc(" << ttc
                      << ")大,不受控" << std::endl;
#endif
          }

        } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << ",横向距离大,不受控" << std::endl;
#endif
        }

        plan_var_t t = 0;
        Int32_t lower = common::LerpInOrderedTable(
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
              test_ret.static_distance, &t);
        plan_var_t relative_velocity_limit = common::Lerp(
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
              lower].value,
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
              lower+1].value, t);

        plan_var_t abs_angle_diff = common::com_abs(
              common::AngleDiff(obj_cfy_info.tar_obj_proj.heading, tar_obj.obb.heading));

        velocity_limit = tar_obj.v * common::com_cos(abs_angle_diff) +
            relative_velocity_limit;

      }
    }
  } else {
    // 静态
    if (obj_cfy_info.static_distance < 0.5F) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向距离小";
#endif

      plan_var_t v_diff = tar_obj.v - curr_v;
      plan_var_t ttc = 100.0F;

      if (v_diff < -0.01F) {
        ttc = obj_cfy_info.dist_to_tar_obj / -v_diff;
      }
      if (abs_t_gap < param_.safe_time_gap || ttc < param_.safe_ttc) {
        need_ctl = true;


#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")小 或 ttc(" << ttc
                  << ")小,受控" << std::endl;
#endif
      } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")大 且 ttc(" << ttc
                  << ")大,不受控" << std::endl;
#endif
      }
    } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向距离大,不受控" << std::endl;
#endif
    }


    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_static_obj,
          obj_cfy_info.static_distance, &t);
    relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower+1].value, t);
//    velocity_limit = tar_obj.v + relative_velocity_limit;
    s = obj_cfy_info.dist_to_tar_obj;

    if (v_for_plan > relative_velocity_limit) {
      velocity_limit =
          tar_obj.v  +
          relative_velocity_limit;
    } else {
      velocity_limit = v_for_plan;
    }
  }

  plan_var_t t = 2.0f*s /
      (v_for_plan + velocity_limit);
  if (t < 1.0f) {
    t = 1.0f;
  }

  plan_var_t a = (velocity_limit - v_for_plan) / t;
  if (a > 0.0F) {
    a = 0.0F;
  }

  if (!obj_classify_result_.valid) {
    obj_classify_result_.deceleration = a;
    obj_classify_result_.tar_velocity = velocity_limit;
    obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
    obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    obj_classify_result_.valid = true;
  } else {
    if (a <= obj_classify_result_.deceleration) {
      obj_classify_result_.deceleration = a;
      obj_classify_result_.tar_velocity = velocity_limit;
      obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
      obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    }
  }
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         规划速度= "<< velocity_limit * 3.6
            << "km/h, 减速度= " << a << std::endl;
#endif

  return (need_ctl);
}

bool ObjClassify::CalcDecelerationByCloseObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info) {
  bool need_ctl = true;

  // 根据障碍物目标速度
  plan_var_t velocity_limit = 0.0F;
  // 根据障碍物减速距离
  plan_var_t s = test_ret.collision_s;


  plan_var_t curr_v = 20.0F/3.6F;
  if (chassis.v_valid) {
    curr_v = chassis.v;
  }
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         紧靠车辆的障碍物,受控" << std::endl;
#endif

  plan_var_t t = 2.0f*s /
      (v_for_plan + velocity_limit);
  if (t < 1.0f) {
    t = 1.0f;
  }

  plan_var_t a = (velocity_limit - v_for_plan) / t;
  if (a > 0.0F) {
    a = 0.0F;
  }

  if (!obj_classify_result_.valid) {
    obj_classify_result_.deceleration = a;
    obj_classify_result_.tar_velocity = velocity_limit;
    obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
    obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    obj_classify_result_.valid = true;
  } else {
    if (a <= obj_classify_result_.deceleration) {
      obj_classify_result_.deceleration = a;
      obj_classify_result_.tar_velocity = velocity_limit;
      obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
      obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    }
  }
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         规划速度= "<< velocity_limit * 3.6
            << "km/h, 减速度= " << a << std::endl;
#endif
  return (need_ctl);
}

bool ObjClassify::CalcDecelerationByLeftOrRightObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info) {
  bool need_ctl = false;

  plan_var_t velocity_limit = 0.0F;
  plan_var_t s = test_ret.collision_s;
  plan_var_t curr_v = 20.0F/3.6F;
  if (chassis.v_valid) {
    curr_v = chassis.v;
  }
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         侧方障碍物";
#endif


  if (obj_cfy_info.static_distance < 0.5F) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
    std::cout << ",横向距离小,受控" << std::endl;
#endif
    need_ctl = true;
    if (tar_obj.dynamic) {
      plan_var_t abs_angle_diff = common::com_abs(
            common::AngleDiff(obj_cfy_info.tar_obj_proj.heading, tar_obj.obb.heading));
      velocity_limit = tar_obj.v * common::com_cos(abs_angle_diff);
      s = common::Min(obj_cfy_info.dist_to_tar_obj,
                      test_ret.collision_s);
    }

  } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
    std::cout << ",横向距离大,不受控" << std::endl;
#endif

  }
  if (need_ctl) {
    plan_var_t t = 2.0f*s /
        (v_for_plan + velocity_limit);
    if (t < 1.0f) {
      t = 1.0f;
    }

    plan_var_t a = (velocity_limit - v_for_plan) / t;
    if (a > 0.0F) {
      a = 0.0F;
    }

    if (!obj_classify_result_.valid) {
      obj_classify_result_.deceleration = a;
      obj_classify_result_.tar_velocity = velocity_limit;
      obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
      obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
      obj_classify_result_.valid = true;
    } else {
      if (a <= obj_classify_result_.deceleration) {
        obj_classify_result_.deceleration = a;
        obj_classify_result_.tar_velocity = velocity_limit;
        obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
        obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
      }
    }
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         规划速度= "<< velocity_limit * 3.6
            << "km/h, 减速度= " << a << std::endl;
#endif
  }
  return (need_ctl);
}

bool ObjClassify::CalcDecelerationByLeftOrRightBackObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         侧后方障碍物";
#endif
  bool need_ctl = false;
  // 根据障碍物目标速度
  plan_var_t velocity_limit = 0.0F;
  // 根据障碍物减速距离
  plan_var_t s = test_ret.collision_s;

  const plan_var_t safe_ttc = 10.0F;
  const plan_var_t safe_time_gap = 3.0F;

  plan_var_t curr_v = 20.0F/3.6F;
  if (chassis.v_valid) {
    curr_v = chassis.v;
  }
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  // 时距
  plan_var_t t_gap = obj_cfy_info.dist_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  if (tar_obj.dynamic) {

    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
          test_ret.static_distance, &t);
    plan_var_t relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
          lower+1].value, t);

    plan_var_t abs_angle_diff = common::com_abs(
          common::AngleDiff(obj_cfy_info.tar_obj_proj.heading, tar_obj.obb.heading));

    velocity_limit = tar_obj.v * common::com_cos(abs_angle_diff) +
        relative_velocity_limit;
    s = common::Min(obj_cfy_info.dist_to_tar_obj,
                    test_ret.collision_s);

    if (obj_cfy_info.static_distance < 0.5F) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向距离小";
#endif
      if (obj_cfy_info.obj_direction ==
          driv_map::OBJ_DIRECTION_BACKWARD) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",逆向行驶,不受控" << std::endl;
#endif
      } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",同向行驶" << std::endl;
#endif
        plan_var_t v_diff = tar_obj.v - curr_v;
        plan_var_t ttc = 100.0F;
        if (v_diff > 0.01F) {
          ttc = -obj_cfy_info.dist_to_tar_obj / v_diff;
        }
        plan_var_t abs_t_gap =
            -obj_cfy_info.dist_to_tar_obj / common::Max(tar_obj.v, curr_v);

        if (abs_t_gap < param_.safe_time_gap || ttc < param_.safe_ttc) {
          need_ctl = true;


#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << ",时距("<< t_gap
                    << ")小 或 ttc(" << ttc
                    << ")小,受控" << std::endl;
#endif
        } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
          std::cout << ",时距("<< t_gap
                    << ")大 且 ttc(" << ttc
                    << ")大,不受控" << std::endl;
#endif
        }

      }
    } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向距离大,不受控" << std::endl;
#endif
    }
  } else {
    // 静态
    if (obj_cfy_info.static_distance < 0.5F) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向距离小";
#endif
      plan_var_t v_diff = tar_obj.v - curr_v;
      plan_var_t ttc = 100.0F;
      if (v_diff > 0.01F) {
        ttc = -obj_cfy_info.dist_to_tar_obj / -v_diff;
      }

      if (abs_t_gap < safe_time_gap || ttc < safe_ttc) {
        need_ctl = true;

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")小 或 ttc(" << ttc
                  << ")小,受控" << std::endl;
#endif
      } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")大 且 ttc(" << ttc
                  << ")大,不受控" << std::endl;
#endif
      }

    } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",横向距离大,不受控" << std::endl;
#endif
    }
      return (need_ctl);
  }

  plan_var_t t = 2.0f*s /
      (v_for_plan + velocity_limit);
  if (t < 1.0f) {
    t = 1.0f;
  }

  plan_var_t a = (velocity_limit - v_for_plan) / t;
  if (a > 0.0F) {
    a = 0.0F;
  }
  if (!obj_classify_result_.valid) {
    obj_classify_result_.deceleration = a;
    obj_classify_result_.tar_velocity = velocity_limit;
    obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
    obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    obj_classify_result_.valid = true;
  } else {
    if (a <= obj_classify_result_.deceleration) {
      obj_classify_result_.deceleration = a;
      obj_classify_result_.tar_velocity = velocity_limit;
      obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
      obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    }
  }
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         规划速度= "<< velocity_limit * 3.6
            << "km/h, 减速度= " << a << std::endl;
#endif
  return (need_ctl);
}

bool ObjClassify::CalcDecelerationByBackObstacle(
    const driv_map::CollisionTestResult::ObjInfo &test_ret,
    const ad_msg::Obstacle& tar_obj,
    const ad_msg::Chassis &chassis,
    const ObjClassifyInfo& obj_cfy_info ) {

  bool need_ctl = false;
  // 根据障碍物目标速度
  plan_var_t velocity_limit = 0.0F;
  // 根据障碍物减速距离
  plan_var_t s = test_ret.collision_s;
  // 与障碍物距离相关的目标速度
  plan_var_t relative_velocity_limit = 0.0F;

  const plan_var_t safe_ttc = 10.0F;
  const plan_var_t safe_time_gap = 3.0F;

  plan_var_t curr_v = 20.0F/3.6F;
  if (chassis.v_valid) {
    curr_v = chassis.v;
  }
  plan_var_t v_for_plan = curr_v;
  if (v_for_plan < 20.0F/3.6F) {
    v_for_plan = 20.0F/3.6F;
  }

  // 时距
  plan_var_t t_gap = obj_cfy_info.dist_to_tar_obj / v_for_plan;
  plan_var_t abs_t_gap = common::com_abs(t_gap);
  if (tar_obj.dynamic) {
    // 正后方障碍物
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
    std::cout << "         正后方障碍物";
#endif
    if (obj_cfy_info.obj_direction ==
        driv_map::OBJ_DIRECTION_BACKWARD) {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",逆向行驶,不受控" << std::endl;
#endif
    } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",同向行驶";
#endif
      plan_var_t v_diff = tar_obj.v - curr_v;
      plan_var_t ttc = 100.0F;
      if (v_diff > 0.01F) {
        ttc = -obj_cfy_info.dist_to_tar_obj / v_diff;
      }
      plan_var_t abs_t_gap =
          -obj_cfy_info.dist_to_tar_obj / common::Max(tar_obj.v, curr_v);

      if (abs_t_gap < safe_time_gap || ttc < safe_ttc) {
        need_ctl = true;
        plan_var_t t = 0;
        Int32_t lower = common::LerpInOrderedTable(
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
              test_ret.static_distance, &t);
        plan_var_t relative_velocity_limit = common::Lerp(
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
              lower].value,
              param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
              lower+1].value, t);

        plan_var_t abs_angle_diff = common::com_abs(
              common::AngleDiff( obj_cfy_info.tar_obj_proj.heading, tar_obj.obb.heading));

        velocity_limit = tar_obj.v * common::com_cos(abs_angle_diff) +
            relative_velocity_limit;
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")小 或 ttc(" << ttc
                  << ")小,受控" << std::endl;
#endif
      } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
        std::cout << ",时距("<< t_gap
                  << ")大 且 ttc(" << ttc
                  << ")大,不受控" << std::endl;
#endif
      }
    }

  } else {
    std::cout << "         正后方障碍物";
    plan_var_t v_diff = tar_obj.v - curr_v;
    plan_var_t ttc = 100.0F;
    if (v_diff > 0.01F) {
      ttc = -obj_cfy_info.dist_to_tar_obj / -v_diff;
    }

    if (/*abs_t_gap < param_.safe_time_gap || */ttc < param_.safe_ttc) {
      need_ctl = true;

#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",时距("<< t_gap
                << ")小 或 ttc(" << ttc
                << ")小,受控" << std::endl;
#endif
    } else {
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
      std::cout << ",时距("<< t_gap
                << ")大 且 ttc(" << ttc
                << ")大,不受控" << std::endl;
#endif
    }
   return (need_ctl);
  }

  plan_var_t t = 2.0f*s /
      (v_for_plan + velocity_limit);
  if (t < 1.0f) {
    t = 1.0f;
  }

  plan_var_t a = (velocity_limit - v_for_plan) / t;
  if (a > 0.0F) {
    a = 0.0F;
  }
  if (!obj_classify_result_.valid) {
    obj_classify_result_.deceleration = a;
    obj_classify_result_.tar_velocity = velocity_limit;
    obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
    obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    obj_classify_result_.valid = true;
  } else {
    if (a <= obj_classify_result_.deceleration) {
      obj_classify_result_.deceleration = a;
      obj_classify_result_.tar_velocity = velocity_limit;
      obj_classify_result_.obj_l = obj_cfy_info.tar_obj_proj.l;
      obj_classify_result_.obj_s = obj_cfy_info.tar_obj_proj.s;
    }
  }
#if ENABLE_ACTION_CLASSIFY_OBSTACLE_TRACE
  std::cout << "         规划速度= "<< velocity_limit * 3.6
            << "km/h, 减速度= " << a << std::endl;
#endif
  return (need_ctl);
}

}
}
