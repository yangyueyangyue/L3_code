
#include "trajectory_planning/trajectory_planning_state_lattice.h"
#include "utils/log.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "vehicle_model_wrapper.h"
#include "pos_filter_wrapper.h"


#define ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE (0)
#define ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST (0)


namespace phoenix {
namespace planning {


TrajectoryPlanningStateLattice::TrajectoryPlanningStateLattice() {
  SetDefaultParam();
  Clear();

  curr_timestamp_ = 0;
  ClearHoldingInfo();

  // Adding timers
  timers_.fade_in_timer_flag = false;
  timer_list_.AddTimer(&timers_.fade_in);
  timer_list_.AddTimer(&timers_.hold_bypassing);
}

TrajectoryPlanningStateLattice::~TrajectoryPlanningStateLattice() {
  // noting to do
}

void TrajectoryPlanningStateLattice::SetDefaultParam() {
  veh_model::VehicleModelWrapper veh_model;
  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 轴距，单位：米
  param_.wheelbase = veh_model.GetWheelbase();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front =
      veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear =
      veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();
  // 车辆外接圆半径，单位：米
  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0, 0), 0, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();
  // 挂车长度，单位：米
  param_.trailer_length = veh_model.GetTrailerLength();

  // 规划用的最小前向速度（设置为大于0的较低速度(例如怠速的速度)，
  // 用来避免除以0,以及保障极低速度下的舒适性）
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  param_.min_forward_velocity_for_planning = 15.0F / 3.6F;
  param_.min_forward_velocity_for_planning_backing_mode = 5.0F / 3.6F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  param_.min_forward_velocity_for_planning = 15.0F / 3.6F;
  param_.min_forward_velocity_for_planning_backing_mode = 5.0F / 3.6F;
#else
  param_.min_forward_velocity_for_planning = 15.0F / 3.6F;
  param_.min_forward_velocity_for_planning_backing_mode = 5.0F / 3.6F;
#endif
  // 向前看的时距(轨迹规划的长度为：当前车速*look_forward_time)
  param_.look_forward_time = 8.0F;//10.0F;
  // 最小向前看的距离
  param_.min_look_forward_distance = 40.0F;
  // 最大向前看的距离
  param_.max_look_forward_distance = 200.0F;

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  // 沿着车道中心线行驶时，生成轨迹的预瞄时距，用来减小车辆控制时与中心线之间的横向偏差
  param_.min_leading_time_for_lka = 2.0F;
  param_.max_leading_time_for_lka = 4.0F;
  // 沿着车道中心线行驶时，生成轨迹的最小预瞄距离
  param_.min_leading_distance_for_lka = 15.0F;
  param_.min_leading_distance_for_lka_backing_mode = 5.0F;
  // 沿着车道中心线行驶时，生成轨迹的最大预瞄距离
  param_.max_leading_distance_for_lka = 30.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  // 沿着车道中心线行驶时，生成轨迹的预瞄时距，用来减小车辆控制时与中心线之间的横向偏差
  param_.min_leading_time_for_lka = 2.0F;
  param_.max_leading_time_for_lka = 4.0F;
  // 沿着车道中心线行驶时，生成轨迹的最小预瞄距离
  param_.min_leading_distance_for_lka = 15.0F;
  param_.min_leading_distance_for_lka_backing_mode = 5.0F;
  // 沿着车道中心线行驶时，生成轨迹的最大预瞄距离
  param_.max_leading_distance_for_lka = 30.0F;
#else
  // 沿着车道中心线行驶时，生成轨迹的预瞄时距，用来减小车辆控制时与中心线之间的横向偏差
  param_.min_leading_time_for_lka = 2.0F;
  param_.max_leading_time_for_lka = 4.0F;
  // 沿着车道中心线行驶时，生成轨迹的最小预瞄距离
  param_.min_leading_distance_for_lka = 15.0F;
  param_.min_leading_distance_for_lka_backing_mode = 8.0F;
  // 沿着车道中心线行驶时，生成轨迹的最大预瞄距离
  param_.max_leading_distance_for_lka = 30.0F;
#endif

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  param_.leading_distance_for_lka_ratio_table.Clear();
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(0.0F, 15.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(10.0F/3.6F, 15.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(20.0F/3.6F, 20.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(30.0F/3.6F, 25.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(40.0F/3.6F, 30.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(50.0F/3.6F, 35.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(60.0F/3.6F, 35.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(70.0F/3.6F, 35.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(80.0F/3.6F, 35.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(90.0F/3.6F, 35.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(100.0F/3.6F, 35.0F));
#else
  param_.leading_distance_for_lka_ratio_table.Clear();
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(0.0F, 15.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(10.0F/3.6F, 15.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(20.0F/3.6F, 18.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(30.0F/3.6F, 20.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(40.0F/3.6F, 25.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(50.0F/3.6F, 30.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(60.0F/3.6F, 30.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(70.0F/3.6F, 30.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(80.0F/3.6F, 30.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(90.0F/3.6F, 30.0F));
  param_.leading_distance_for_lka_ratio_table.PushBack(
        common::LerpTableNodeType1(100.0F/3.6F, 30.0F));
#endif

  // 预瞄距离修正（根据参考线的曲率，曲率越大，预瞄距离越小）
  param_.leading_distance_correct_ratio_table_by_trj_curv.Clear();
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.00F, 1.0F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.002F, 0.85F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.004F, 0.75F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.01F, 0.68F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.02F, 0.5F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.05F, 0.35F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.10F, 0.26F));
  param_.leading_distance_correct_ratio_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.20F, 0.21F));


  // 生成路网时，纵向采样时使用的最小车速(用来保障极低速度下的舒适性)
  param_.min_forward_velocity_for_lon_sample = 20.0F/3.6F;
  param_.min_backward_velocity_for_lon_sample = 5.0F/3.6F;
  // 生成路网时，纵向采样点间的时间间隔
  param_.lon_sample_time_interval = 4.0F;
  // 生成路网时，纵向采样点间的距离限制（根据参考线的曲率，曲率越大，采样间隔越小）
  param_.lon_sample_step_len_limit_table_by_trj_curv.Clear();
  param_.lon_sample_step_len_limit_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.0F, 50.0F));
  param_.lon_sample_step_len_limit_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.01F, 23.0F));
  param_.lon_sample_step_len_limit_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.09F, 15.0F));
  param_.lon_sample_step_len_limit_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.10F, 14.0F));
  param_.lon_sample_step_len_limit_table_by_trj_curv.PushBack(
        common::LerpTableNodeType1(0.20F, 12.0F));
  // 生成路网时，纵向采样点间的最大和最小距离限制表
  param_.lon_sample_step_len_limit_table.Clear();
  param_.lon_sample_step_len_limit_table.PushBack(
        common::DataPair<plan_var_t, plan_var_t>(10.0F, 35.0F));
  param_.lon_sample_step_len_limit_table.PushBack(
        common::DataPair<plan_var_t, plan_var_t>(10.0F, 50.0F));
  param_.lon_sample_step_len_limit_table.PushBack(
        common::DataPair<plan_var_t, plan_var_t>(10.0F, 50.0F));
  param_.lon_sample_step_len_limit_table.PushBack(
        common::DataPair<plan_var_t, plan_var_t>(10.0F, 50.0F));

  // 生成路网时，横向采样点与道路边界的最小距离（保持与路沿间隔一定距离，避免轮胎擦碰）
  param_.dist_of_keeping_away_from_road_boundary = 0.3F;
  // 生成路网时，横向采样点与车道边线的最小距离（避免轮胎压线）
  param_.dist_of_keeping_away_from_lane_boundary = 0.1F;
  // 生成路网时，横向采样点的间隔
  param_.lat_sample_step_len = 0.4F;
  // 生成路网时，link曲线的采样间隔
  param_.sample_step_len_of_link_curve = 3.0F;

  // 最大允许的侧向加速度，用来计算link曲线的减速度
  param_.max_allowed_lateral_acceleration = 0.5F;
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

  // 车道对应的Cost Ratio，尽量在当前车道行驶
  // (乘以车道对应的车道相邻序号的绝对值，车道与当前车道偏差越大，车道相邻序号的绝对值越大)
  param_.left_lane_cost_ratio = 150;
  param_.right_lane_cost_ratio = 160;
  // Routing车道对应的Cost (按照一次规划的道路行驶)
  param_.routing_lane_cost = -130;
  // 车道内与中心线之间的横向偏差对应的Cost Ratio，尽量沿着车道中心线行驶
  // (乘以车道中横向偏差的绝对值)
  param_.left_lateral_offset_cost_ratio = 100;
  param_.right_lateral_offset_cost_ratio = 120;
  // Lane cost
  param_.lka_cost_ratio_major = 30;  // 20210917: 40
  param_.lka_cost_ratio_minor = 8;   // 20210917: 10
  // 候选轨迹的曲率对应的Cost Ratio，尽量沿着平滑的轨迹行驶
  // (乘以因为轨迹曲率导致的减速度的绝对值)
  param_.curvature_cost_ratio = 15;  // 20210917: 50
  // 与障碍物之间的碰撞风险对应的Cost Ratio，避免碰撞
  // (乘以因为障碍物导致的减速度的绝对值)
  param_.collision_cost_ratio = 900; // 100, 700
  // 候选轨迹长度对应的Cost Ratio，进行选择规划的远的轨迹
  // (乘以路网纵向段索引，轨迹越长，段索引越大)
  param_.trj_len_cost_ratio = -10;
  // 与道路边界有干涉的Cost
  param_.collision_with_road_boundary_cost = 500;
  // 规划单条路径，根据车当前横向偏移范围
  // 选择link的第一个点
  param_.range_of_lat_offset_table.Clear();
  param_.range_of_lat_offset_table.PushBack(0.5f);
  param_.range_of_lat_offset_table.PushBack(25.0f);

  param_.path_filter_laziness_lat = 3.0F;
  param_.path_filter_max_trend_lat = 6.0F;
  param_.path_filter_trend_move_velocity_lat = 1.0F;
  param_.path_filter_trend_stop_velocity_lat = 1.0F;
  // param_.path_filter_leading_move_max_velocity_lat = 0.3F;
  param_.path_filter_leading_move_max_velocity_lat = 0.2F;
  param_.path_filter_leading_move_accel_lat = 0.01F;
  param_.path_filter_leading_stop_accel_lat = 0.01F;

  param_.path_filter_laziness_lon = 3.0F;
  param_.path_filter_max_trend_lon = 6.0F;
  param_.path_filter_trend_move_velocity_lon = 1.0F;
  param_.path_filter_trend_stop_velocity_lon = 1.0F;
  param_.path_filter_leading_move_max_velocity_lon = 2.0F;
  param_.path_filter_leading_move_accel_lon = 0.2F;
  param_.path_filter_leading_stop_accel_lon = 0.2F;

  param_.max_leading_lat_offset_limit_table_by_speed.Clear();
  param_.max_leading_lat_offset_limit_table_by_speed.PushBack(
        common::LerpTableNodeType1(0.0F/3.6F,   2.0F));
  param_.max_leading_lat_offset_limit_table_by_speed.PushBack(
        common::LerpTableNodeType1(20.0F/3.6F,  2.0F));
  param_.max_leading_lat_offset_limit_table_by_speed.PushBack(
        common::LerpTableNodeType1(40.0F/3.6F,  1.8F));
  param_.max_leading_lat_offset_limit_table_by_speed.PushBack(
        common::LerpTableNodeType1(60.0F/3.6F,  1.6F));
  param_.max_leading_lat_offset_limit_table_by_speed.PushBack(
        common::LerpTableNodeType1(80.0F/3.6F,  1.4F));
  param_.max_leading_lat_offset_limit_table_by_speed.PushBack(
        common::LerpTableNodeType1(100.0F/3.6F, 1.2F));

  param_.changing_lane_complete_cond_lat_offset = 0.5F;

  action_smoothing_.abort_changing_lane_left_by_uncertain_obj.SetParam(3, 5, 6);
  action_smoothing_.abort_changing_lane_right_by_uncertain_obj.SetParam(3, 5, 6);

  action_smoothing_.req_bypassing_left.SetParam(10, 13, 15);
  action_smoothing_.req_bypassing_right.SetParam(10, 13, 15);
}

void TrajectoryPlanningStateLattice::Configurate(
    const TrajectoryPlanningConfig& conf) {
  config_ = conf;
}

void TrajectoryPlanningStateLattice::Clear() {
  driving_direction_ = DRIVING_DIRECTION_FORWARD;

  result_of_action_planning_.Clear();
  major_ref_line_index_ = -1;
  major_ref_line_ = Nullptr_t;
  veh_proj_on_major_ref_line_.Clear();

  curr_lane_index_ = -1;
  veh_proj_on_curr_lane_.Clear();

  look_forward_length_ = 0;
  leading_length_for_lka_ = 0;

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

  // 清空候选轨迹
  for (Int32_t i = 0; i < candidate_trj_list_.Size(); ++i) {
    candidate_trj_list_[i].Clear();
  }
  candidate_trj_list_.Clear();

  optimal_trj_index_.Clear();
  optimal_trajectory_sample_points_.Clear();

  result_of_planning_.Clear();
}

void TrajectoryPlanningStateLattice::ClearHoldingInfo() {
  pred_path_of_vehicle_.Clear();

  action_manager_.Clear();
  /* k002 pengc 2022-08-15 (begin) */
  /* 添加偏离车道线行驶的功能 */
  driving_off_center_.Clear();
  driving_off_center_.expected_lat_offset = 0.0F;
  /* k002 pengc 2022-08-15 (end) */

  // LOG_INFO(5) << "Clear road type from " << road_builed_type_
  //             << " to " << ROAD_BUILED_TYPE_INVALID;
  road_builed_type_ = ROAD_BUILED_TYPE_INVALID;
  prev_road_builed_type_ = ROAD_BUILED_TYPE_INVALID;

  path_filter_.Clear();
  lat_err_.Clear();
  status_.Clear();
  prev_ref_line_sample_points_for_debug_.Clear();

  action_smoothing_.abort_changing_lane_left_by_uncertain_obj.Clear();
  action_smoothing_.abort_changing_lane_right_by_uncertain_obj.Clear();
  action_smoothing_.req_bypassing_left.Clear();
  action_smoothing_.req_bypassing_right.Clear();
}

bool TrajectoryPlanningStateLattice::Plan(
    const TrajectoryPlanningDataSource& data_source) {
#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "######### Trajectory planning (state lattice) >>>>>>>>>"
            << std::endl;
#endif

  event_reporting_list_.Clear();
  Clear();
  // 发生内部错误时，保持方向盘不动
  result_of_planning_.hold_steering_wheel = true;
  result_of_planning_.trj_changing.is_changing = false;
  result_of_planning_.steering_wheel_speed = common::com_deg2rad(200.0F);

  UpdateTimerList();

  if (Nullptr_t == data_source.driving_map) {
    ClearHoldingInfo();

    LOG_ERR << "Invalid driving map.";
    return false;
  }
  if (Nullptr_t == data_source.result_of_action_planning) {
    ClearHoldingInfo();

    LOG_ERR << "Invalid result of action planning.";
    return false;
  }
  if (Nullptr_t == data_source.chassis) {
    ClearHoldingInfo();

    LOG_ERR << "Invalid chassis information.";
    return false;
  }

  curr_timestamp_ = data_source.timestamp;
  chassis_status_ = *data_source.chassis;

  if (ad_msg::VEH_GEAR_R == data_source.chassis->gear) {
    driving_direction_ = DRIVING_DIRECTION_BACKWARD;
  } else {
    driving_direction_ = DRIVING_DIRECTION_FORWARD;
  }

  driving_map_ = data_source.driving_map;
  result_of_action_planning_ = *(data_source.result_of_action_planning);

  ad_msg::RelativePos current_rel_pos;
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
        data_source.timestamp, driving_map_->GetRelativePosList(),
        &current_rel_pos)) {
    ClearHoldingInfo();

    LOG_ERR << "Failed to get current posistion from relative position list.";
    return false;
  }

  // 更新车身参数
  UpdateVehicleParameter();

  veh_velocity_for_planning_ = common::com_abs(current_rel_pos.v);
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    if (veh_velocity_for_planning_ <
        param_.min_forward_velocity_for_planning_backing_mode) {
      veh_velocity_for_planning_ =
          param_.min_forward_velocity_for_planning_backing_mode;
    }
  } else {
    if (veh_velocity_for_planning_ <
        param_.min_forward_velocity_for_planning) {
      veh_velocity_for_planning_ = param_.min_forward_velocity_for_planning;
    }
  }
  // Current position
  curr_position_.path_point.point.set_x(current_rel_pos.x);
  curr_position_.path_point.point.set_y(current_rel_pos.y);
  curr_position_.path_point.heading = current_rel_pos.heading;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    curr_position_.path_point.heading = common::NormalizeAngle(
          curr_position_.path_point.heading + COM_PI);
  }
  plan_var_t cur_yaw_rate = current_rel_pos.yaw_rate;
  plan_var_t cur_yaw_rate_chg_rate = current_rel_pos.yaw_rate_chg_rate;
  //printf("current_rel_pos.yaw_rate_chg_rate=%0.3f\n",
  //       common::com_rad2deg(current_rel_pos.yaw_rate_chg_rate));
  if (current_rel_pos.v < 10.0F/3.6F) {
    veh_model::VehicleModelWrapper veh_model;
    cur_yaw_rate = veh_model.CalcYawRateFromSteeringAngle(
          data_source.chassis->steering_wheel_angle, 10.0F/3.6F);
    cur_yaw_rate_chg_rate = 0.0F;
  }
  curr_position_.path_point.curvature =
      cur_yaw_rate / veh_velocity_for_planning_;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    curr_position_.path_point.curvature = -curr_position_.path_point.curvature;
  }
  curr_position_.path_point.l = 0.0F;
  curr_position_.path_point.s = 0.0F;
  curr_position_.v = common::com_abs(current_rel_pos.v);
  curr_position_.yaw_rate = cur_yaw_rate;
  curr_position_.yaw_rate_chg_rate = cur_yaw_rate_chg_rate;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    curr_position_.yaw_rate = -curr_position_.yaw_rate;
    curr_position_.yaw_rate_chg_rate = -curr_position_.yaw_rate_chg_rate;
  }
  curr_position_.relative_time = 0.0F;


  // Check if reference line is valid
  if (driving_map_->GetReferenceLinesNum() < 1) {
    ClearHoldingInfo();

    LOG_ERR << "There are no reference line in driving map.";
    return false;
  }

  // Get current reference line
  major_ref_line_index_ = driving_map_->GetMajorReferenceLineIndex();
  if (!driving_map_->IsValidReferenceLineIndex(major_ref_line_index_)) {
    major_ref_line_index_ = -1;
    ClearHoldingInfo();

    LOG_ERR << "Invalid reference line index " << major_ref_line_index_;
    return false;
  }
  major_ref_line_ = &(driving_map_->GetSmoothReferenceLine(major_ref_line_index_));
  if (!major_ref_line_->FindProjection(curr_position_.path_point.point,
                                       &veh_proj_on_major_ref_line_)) {
    major_ref_line_index_ = -1;
    ClearHoldingInfo();

    LOG_ERR << "Failed to find projecton of current postion "
               "from current reference line.";
    return false;
  }
  curr_position_.path_point.l = veh_proj_on_major_ref_line_.l;
  curr_position_.path_point.s = veh_proj_on_major_ref_line_.s;

  if (common::com_abs(veh_proj_on_major_ref_line_.l) > 5.0F) {
    ClearHoldingInfo();

    LOG_ERR << "Too far from reference line.";
    return false;
  }

  curr_lane_index_ = -1;
  if (!driving_map_->FindNearestLaneSynthetically(
        curr_position_.path_point.point, curr_position_.path_point.heading,
        &curr_lane_index_, &veh_proj_on_curr_lane_)) {
    curr_lane_index_ = -1;
    ClearHoldingInfo();

    LOG_ERR << "Failed to find nearest lane current point.";
    return false;
  }
  if (!driving_map_->IsValidLaneIndex(curr_lane_index_)) {
    curr_lane_index_ = -1;
    ClearHoldingInfo();

    LOG_ERR << "Failed to find nearest lane of current point, "
               "because the lane index is invalid.";
    return false;
  }

#if 0
  LOG_INFO(5) << "In trajectory planning, the nearest lane is (idx="
              << curr_lane_index_ << ", id="
              << driving_map_->GetLaneIdByIndex(curr_lane_index_).id
              << ").";
#endif

  // Convert previous reference line to current coordinate frame.
  ConvertPrevRefLineToCurrCoorFrame();

  Int32_t road_type = ROAD_BUILED_TYPE_INVALID;
  Int32_t driving_map_type = driving_map_->GetDrivingMapType();
  switch (driving_map_type) {
  case (driv_map::DRIVING_MAP_TYPE_HD_MAP):
    road_type = ROAD_BUILED_TYPE_HD_MAP;
    break;
  case (driv_map::DRIVING_MAP_TYPE_CAMERA_LANE):
    road_type = ROAD_BUILED_TYPE_CAMERA_LANE;
    break;
  case (driv_map::DRIVING_MAP_TYPE_FOLLOWING_PATH):
    road_type = ROAD_BUILED_TYPE_FOLLOWING;
    /* k002 pengc 2022-08-15 (begin) */
    /* 添加偏离车道线行驶的功能 */
    driving_off_center_.Clear();
    /* k002 pengc 2022-08-15 (end) */
    break;
  case (driv_map::DRIVING_MAP_TYPE_PRED_PATH):
    road_type = ROAD_BUILED_TYPE_PRED_PATH;
    /* k002 pengc 2022-08-15 (begin) */
    /* 添加偏离车道线行驶的功能 */
    driving_off_center_.Clear();
    /* k002 pengc 2022-08-15 (end) */
    break;
  case (driv_map::DRIVING_MAP_TYPE_MIXED_HD_MAP):
    road_type = ROAD_BUILED_TYPE_MIXED_HD_MAP;
    break;
  default:
    LOG_ERR << "Invalid driving map type.";
    road_type = ROAD_BUILED_TYPE_INVALID;
    /* k002 pengc 2022-08-15 (begin) */
    /* 添加偏离车道线行驶的功能 */
    driving_off_center_.Clear();
    /* k002 pengc 2022-08-15 (end) */
    break;
  }

  // 方向盘淡入
  StandByToFadeIn(road_builed_type_, road_type);

  // 更新路网类型
  // LOG_INFO(5) << "Update road type from " << road_builed_type_
  //             << " to " << road_type;
  road_builed_type_ = road_type;

  // Get look forward length
  look_forward_length_ = veh_velocity_for_planning_ * param_.look_forward_time;
  if (look_forward_length_ < param_.min_look_forward_distance) {
    look_forward_length_ = param_.min_look_forward_distance;
  }
  if (look_forward_length_ > param_.max_look_forward_distance) {
    look_forward_length_ = param_.max_look_forward_distance;
  }
  /* k002 pengc 2022-08-15 (begin) */
  /* 添加偏离车道线行驶的功能 */
  // Get leading lenght for LKA
  // leading_length_for_lka_ = CalcLeadingLen(
  //      common::com_abs(veh_proj_on_major_ref_line_.l));
  leading_length_for_lka_ = CalcLeadingLen(
       common::com_abs(veh_proj_on_major_ref_line_.l -
                       driving_off_center_.expected_lat_offset));
  /* k002 pengc 2022-08-15 (end) */

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "look_forward_length_="
            << look_forward_length_ << std::endl;
  std::cout << "leading_length_for_lka_="
            << leading_length_for_lka_ << std::endl;
#endif

  // Update Action Request
#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "@@@ 收到变道请求="
            << result_of_action_planning_.changing_lane_req.request()
            << std::endl;
#endif
  if (result_of_action_planning_.changing_lane_req.request() !=
      ChangingLaneReq::REQ_CHANGING_LANE_NONE) {
    if (result_of_action_planning_.changing_lane_req.sequence() !=
        action_manager_.changing_lane_req.sequence()) {
      action_manager_.changing_lane_req =
          result_of_action_planning_.changing_lane_req;
      action_manager_.changing_lane_rsp.set_requset_sequence(
            result_of_action_planning_.changing_lane_req.sequence());
    }
  }

  if ((ad_msg::VEH_DRIVING_MODE_ROBOTIC == data_source.chassis->driving_mode) &&
      (ad_msg::VEH_EPS_STATUS_ROBOTIC == data_source.chassis->eps_status) &&
      (result_of_action_planning_.enable_lka)) {
    // 方向盘自动控制下

    if (!PlanTrjInRoboticMode()) {
      ClearHoldingInfo();
      LOG_ERR << "Failed to plan trajectory in robotic mode.";
      return false;
    }
  } else {
    // 方向盘非自动控制下

    if (!PlanTrjInManualMode()) {
      ClearHoldingInfo();
      LOG_ERR << "Failed to plan trajectory in manual mode.";
      return false;
    }
  }

  // 保存之前的路网构建类型
  prev_road_builed_type_ = road_builed_type_;


  /* k006 pengc 2023-01-06 (begin) */
  // 更新轨迹坡度
  UpdateTrajectorySlope();
  /* k006 pengc 2023-01-06 (begin) */

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "<<<<<<<<< Trajectory planning (state lattice) #########"
            << std::endl;
#endif

  return true;
}


void TrajectoryPlanningStateLattice::UpdateVehicleParameter() {
  veh_model::VehicleModelWrapper veh_model;
  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 轴距，单位：米
  param_.wheelbase = veh_model.GetWheelbase();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front =
      veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear =
      veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();

  if (ad_msg::VEH_TRAILER_STATUS_CONNECTED == chassis_status_.trailer_status) {
    param_.vehicle_width =
        common::Max(veh_model.GetVehicleWidth(), veh_model.GetTrailerWidth());
  }

  // 车辆外接圆半径，单位：米
  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0, 0), 0, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();
  // 挂车长度，单位：米
  param_.trailer_length = veh_model.GetTrailerLength();

  LOG_INFO(5) << "Vehicle Width = " << param_.vehicle_width
              << ", Trailer Status = " << chassis_status_.trailer_status;
}

void TrajectoryPlanningStateLattice::StandByToFadeIn(
    Int32_t prev_road_type, Int32_t curr_road_type) {
#if 1
  // Calculate expeceted latteral offset
  plan_var_t expected_lat_offset = veh_proj_on_major_ref_line_.l;
#if 0
  common::Vec2d unit_direction(
        common::com_cos(curr_position_.path_point.heading),
        common::com_sin(curr_position_.path_point.heading));
  //common::Vec2d veh_head_point =
  //    curr_position_.path_point.point + 4.0F * param_.wheelbase * unit_direction;
  // common::Vec2d veh_head_point =
  //     curr_position_.path_point.point + 20.0F * unit_direction;
  common::Vec2d veh_head_point =
      curr_position_.path_point.point +
      common::Max(1.0F, 0.5F * curr_position_.v) * unit_direction;
#else
  plan_var_t curr_pos[3] = { 0.0F };
  plan_var_t next_pos[3] = { 0.0F };
  curr_pos[0] = curr_position_.path_point.point.x();
  curr_pos[1] = curr_position_.path_point.point.y();
  curr_pos[2] = curr_position_.path_point.heading;
  vehicle_model_.EstimateNextPos(
        common::Max(30.0F/3.6F, curr_position_.v), curr_position_.yaw_rate, 1.0F, curr_pos, next_pos);
  common::Vec2d veh_head_point(next_pos[0], next_pos[1]);
#endif

  common::PathPoint veh_head_path_point;
  if (major_ref_line_->FindProjection(veh_head_point, &veh_head_path_point)) {
    expected_lat_offset = veh_head_path_point.l;
  } else {
    LOG_ERR << "Failed to find projection of vehicle head postion "
               "from major reference line.";
  }

#if 1
  if ((ROAD_BUILED_TYPE_HD_MAP != curr_road_type) &&
      (ROAD_BUILED_TYPE_CAMERA_LANE != curr_road_type) &&
      (ROAD_BUILED_TYPE_MIXED_HD_MAP != curr_road_type)) {
    driving_off_center_.expected_lat_offset = 0.0F;
    timers_.fade_in.Stop();
  }
#endif

  // std::cout << "prev_road_type=" << prev_road_type
  //           << ", curr_road_type=" << curr_road_type << std::endl;
  /* k002 pengc 2022-08-15 (begin) */
  /* 进入自动或丢失车道线然后重新识别后，平缓方向盘的修正过程 */
  if (((ROAD_BUILED_TYPE_HD_MAP != prev_road_type) &&
       (ROAD_BUILED_TYPE_CAMERA_LANE != prev_road_type) &&
       (ROAD_BUILED_TYPE_MIXED_HD_MAP != prev_road_type)) &&
      ((ROAD_BUILED_TYPE_HD_MAP == curr_road_type) ||
       (ROAD_BUILED_TYPE_CAMERA_LANE == curr_road_type) ||
       (ROAD_BUILED_TYPE_MIXED_HD_MAP == curr_road_type))) {
    // 车道线或地图丢失，然后重新识别后; 或者之前是手动模式，现在进入了自动模式
    timers_.fade_in_timer_flag = true;
    timers_.fade_in.SetTimeout(8*1000);
    timers_.fade_in.Restart();
    driving_off_center_.expected_lat_offset = expected_lat_offset;

    LOG_INFO(5) << ">>>>>> Stand by to fade in. >>>>>>";
  }
  if (timers_.fade_in.IsActive()) {
    // 限制方向盘转速
    result_of_planning_.steering_wheel_speed = common::com_deg2rad(90.0F);

    // 缓慢回到车道中心线
    plan_var_t step = 0.03F;
    if (driving_off_center_.expected_lat_offset > 0.01F) {
      driving_off_center_.expected_lat_offset -= step;
      if (driving_off_center_.expected_lat_offset < 0.01F) {
        driving_off_center_.expected_lat_offset = 0.0F;
      }
    } else if (driving_off_center_.expected_lat_offset < -0.01F) {
      driving_off_center_.expected_lat_offset += step;
      if (driving_off_center_.expected_lat_offset > -0.01F) {
        driving_off_center_.expected_lat_offset = 0.0F;
      }
    } else {
      driving_off_center_.expected_lat_offset = 0.0F;
      timers_.fade_in.Stop();
    }


    Int32_t ref_changed_flag = IsRefLineChanged();
    if (0 == ref_changed_flag) {
      if (road_builed_type_ != prev_road_builed_type_) {
        if ((veh_proj_on_major_ref_line_.l - status_.prev_proj_on_ref.l) < 0.0F) {
          ref_changed_flag = 1;
        } else {
          ref_changed_flag = 2;
        }
      }
    }

    if (0 != ref_changed_flag) {
#if 1
      if (path_filter_.valid) {
        common::PathPoint leading_path_point;
        if (major_ref_line_->FindProjection(
              path_filter_.prev_leading_point.point, &leading_path_point)) {
          driving_off_center_.expected_lat_offset = leading_path_point.l;
        } else {
          driving_off_center_.expected_lat_offset = expected_lat_offset;
        }
      } else {
        driving_off_center_.expected_lat_offset = expected_lat_offset;
      }
#else
      driving_off_center_.expected_lat_offset = expected_lat_offset;
#endif
    }

    // std::cout << "1: expected_lat_offset="
    //           << driving_off_center_.expected_lat_offset << std::endl;
  } else {
    if (timers_.fade_in_timer_flag) {
      timers_.fade_in_timer_flag = false;
      driving_off_center_.expected_lat_offset = 0.0F;
    }

    // std::cout << "2: expected_lat_offset="
    //           << driving_off_center_.expected_lat_offset << std::endl;
  }
  /* k002 pengc 2022-08-15 (end) */
#endif
}

void TrajectoryPlanningStateLattice::GetTrajectoryPlanningInfo(
    TrajectoryPlanningInfo* trj_planning_info) const {
  trj_planning_info->Clear();

  trj_planning_info->road_builed_type = road_builed_type_;

  trj_planning_info->leading_length_for_lka = leading_length_for_lka_;
  trj_planning_info->lon_graph_samples_step_len = lon_graph_samples_step_len_;

  // Get all of graph sections
  for (Int32_t i = 0; i < road_graph_.Size(); ++i) {
    const GraphSection& graph_section = road_graph_[i];

    // Get all of lane sections in graph section.
    Int32_t lane_section_size = graph_section.lane_sections.Size();
    for (Int32_t j = 0; j < lane_section_size; ++j) {
      const GraphSection::LaneSection& lane_section =
          graph_section.lane_sections[j];

      // Get all nodes in lane section
      Int32_t nodes_size = lane_section.nodes.Size();
      for (Int32_t k = 0; k < nodes_size; ++k) {
        const GraphSection::GraphNode& node = lane_section.nodes[k];

        // Get all links in nodes
        common::internal::DoubleList::const_iterator<
            GraphLinkListNode, MAX_GRAPH_LINK_NUM> it =
            node.link_list.cbegin(road_graph_link_storage_);
        common::internal::DoubleList::const_iterator<
            GraphLinkListNode, MAX_GRAPH_LINK_NUM> it_end =
            node.link_list.cend(road_graph_link_storage_);
        for (; it != it_end; ++it) {
          const GraphSection::GraphNode::Link& link = it->link;

          // Get next connected node
          const GraphSection& graph_section_to =
              road_graph_[link.node_to.graph_section_index];
          const GraphSection::LaneSection& lane_section_to =
              graph_section_to.lane_sections[link.node_to.lane_section_index];
          const GraphSection::GraphNode& node_to =
              lane_section_to.nodes[link.node_to.node_index];

          TrajectoryPlanningInfo::RoadGraph::Link* link_info_out =
              trj_planning_info->road_graph.road_graph_links.Allocate();
          if (Nullptr_t == link_info_out) {
            LOG_ERR << "Can't add link to road graph, storage is full.";
            break;
          }
          link_info_out->sample_points = link.sample_points;
          link_info_out->is_collision_with_road_boundary =
              link.is_collision_with_road_boundary;
        }
      }
    }
  }

  // Get sample points of optimal trajectory
  trj_planning_info->optimal_trajectory_sample_points =
      optimal_trajectory_sample_points_;

  trj_planning_info->prev_ref_line_sample_points =
      prev_ref_line_sample_points_for_debug_;

  // current lateral error
  trj_planning_info->lat_err_sample[0].lat_err =
      lat_err_.lat_err_sample[0].lat_err;
  trj_planning_info->lat_err_sample[0].lat_err_smooth =
      lat_err_.lat_err_sample[0].lat_err_smooth;
  trj_planning_info->lat_err_sample[0].lat_err_v =
      lat_err_.lat_err_sample[0].lat_err_v;
  trj_planning_info->lat_err_sample[0].lat_err_v_smooth =
      lat_err_.lat_err_sample[0].lat_err_v_smooth;
  trj_planning_info->lat_err_sample[0].lat_err_a =
      lat_err_.filter[0].lat_err_list[0].lat_err_a;   
  trj_planning_info->lat_err_sample[0].yaw_err =
      lat_err_.lat_err_sample[0].yaw_err;
  trj_planning_info->lat_err_sample[0].yaw_err_smooth =
      lat_err_.lat_err_sample[0].yaw_err_smooth;
  trj_planning_info->lat_err_sample[0].yaw_err_v =
      lat_err_.lat_err_sample[0].yaw_err_v;
  trj_planning_info->lat_err_sample[0].yaw_err_v_smooth =
      lat_err_.lat_err_sample[0].yaw_err_v_smooth;
  // next lateral error
  trj_planning_info->lat_err_sample[1].lat_err =
      lat_err_.lat_err_sample[1].lat_err;
  trj_planning_info->lat_err_sample[1].lat_err_smooth =
      lat_err_.lat_err_sample[1].lat_err_smooth;
  trj_planning_info->lat_err_sample[1].lat_err_v =
      lat_err_.lat_err_sample[1].lat_err_v;
  trj_planning_info->lat_err_sample[1].lat_err_v_smooth =
      lat_err_.lat_err_sample[1].lat_err_v_smooth;
  trj_planning_info->lat_err_sample[1].yaw_err =
      lat_err_.lat_err_sample[1].yaw_err;
  trj_planning_info->lat_err_sample[1].yaw_err_smooth =
      lat_err_.lat_err_sample[1].yaw_err_smooth;
  trj_planning_info->lat_err_sample[1].yaw_err_v =
      lat_err_.lat_err_sample[1].yaw_err_v;
  trj_planning_info->lat_err_sample[1].yaw_err_v_smooth =
      lat_err_.lat_err_sample[1].yaw_err_v_smooth;

  trj_planning_info->lat_err_pred_point.x = lat_err_.pred_point.x;
  trj_planning_info->lat_err_pred_point.y = lat_err_.pred_point.y;
  trj_planning_info->lat_err_pred_point.h = lat_err_.pred_point.h;
}

Int32_t TrajectoryPlanningStateLattice::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  if (num > event_reporting_list_.Size()) {
    num = event_reporting_list_.Size();
  }

  Int32_t idx = 0;
  for (idx = 0; idx < num; ++idx) {
    events[idx] = event_reporting_list_[idx];
  }

  return (idx);
}


void TrajectoryPlanningStateLattice::UpdateTimerList() {
  timer_list_.Update();
}

bool TrajectoryPlanningStateLattice::PlanTrjInRoboticMode() {
#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST
  phoenix::common::Stopwatch timer_create_road_graph;
#endif

#if 1
  // 生成路网
  if (!CreateRoadGraph_Type1()) {
    LOG_ERR << "Failed to create road graph.";
    return false;
  }
#else
  // 生成路网
  if (!CreateRoadGraph_Type2()) {
    LOG_ERR << "Failed to create road graph.";
    return false;
  }
#endif

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST
  std::cout << "    @ Create road graph spends "
            << timer_create_road_graph.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST
  phoenix::common::Stopwatch timer_find_candidate_trj;
#endif

  // 从路网中查找所有的候选轨迹
  if (!FindAllCandidateTrajectory()) {
    LOG_ERR << "Failed to find candidate trajectories.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST
  std::cout << "    @ Finding candidate trajectory spends "
            << timer_find_candidate_trj.Elapsed()
            << "ms." << std::endl;
#endif

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST
  phoenix::common::Stopwatch timer_calc_target_trj;
#endif

  // 从候选轨迹中查找最优轨迹
  if (!FindOptimalTrajectory()) {
    LOG_ERR << "Failed to find optimal trajectory.";
    return false;
  }

  // 计算目标轨迹(综合考虑行为规划、安全性、舒适性，并保证时间轴上轨迹的连续性)
  if (!CalcTargetTrajectory()) {
    LOG_ERR << "Failed to calculate target trajectory.";
    return false;
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_PERFORMANCE_TEST
  std::cout << "    @ Calculating target trajectory spends "
            << timer_calc_target_trj.Elapsed()
            << "ms." << std::endl;
#endif

  return true;
}

bool TrajectoryPlanningStateLattice::PlanTrjInManualMode() {
  /// 清除自动模式下规划的信息
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
  // 清空候选轨迹
  for (Int32_t i = 0; i < candidate_trj_list_.Size(); ++i) {
    candidate_trj_list_[i].Clear();
  }
  candidate_trj_list_.Clear();
  optimal_trj_index_.Clear();
  optimal_trajectory_sample_points_.Clear();
  result_of_planning_.Clear();
  //
  action_manager_.Clear();
  // LOG_INFO(5) << "Set road type from " << road_builed_type_
  //             << " to " << ROAD_BUILED_TYPE_PRED_PATH;
  road_builed_type_ = ROAD_BUILED_TYPE_PRED_PATH;
  path_filter_.Clear();
  lat_err_.Clear();
  status_.Clear();
  prev_ref_line_sample_points_for_debug_.Clear();
  action_smoothing_.abort_changing_lane_left_by_uncertain_obj.Clear();
  action_smoothing_.abort_changing_lane_right_by_uncertain_obj.Clear();
  action_smoothing_.req_bypassing_left.Clear();
  action_smoothing_.req_bypassing_right.Clear();

  status_.trj_status = TRJ_STATUS_LKA;
  status_.prev_proj_on_ref = veh_proj_on_major_ref_line_;


  /// 填充规划的轨迹
  result_of_planning_.msg_head = driving_map_->GetRelativePosList().msg_head;
  result_of_planning_.curr_pos.x = curr_position_.path_point.point.x();
  result_of_planning_.curr_pos.y = curr_position_.path_point.point.y();
  result_of_planning_.curr_pos.heading = curr_position_.path_point.heading;
  result_of_planning_.curr_pos.curvature = curr_position_.path_point.curvature;
  result_of_planning_.curr_pos.s = 0.0F;
  result_of_planning_.curr_pos.l = veh_proj_on_major_ref_line_.l;
  // 假的先导点
  result_of_planning_.leading_pos.x = curr_position_.path_point.point.x();
  result_of_planning_.leading_pos.y = curr_position_.path_point.point.y();
  result_of_planning_.leading_pos.heading = curr_position_.path_point.heading;
  result_of_planning_.leading_pos.curvature = curr_position_.path_point.curvature;
  result_of_planning_.leading_pos.s = 0.0F;
  result_of_planning_.leading_pos.l = 0.0F;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    result_of_planning_.trj_direction =
        TrajectoryPlanningResult::TRJ_DIRECTION_BACKWARD;
  } else {
    result_of_planning_.trj_direction =
        TrajectoryPlanningResult::TRJ_DIRECTION_FORWARD;
  }

  result_of_planning_.lat_err.moving_flag = 0;
  result_of_planning_.lat_err.samples[0].lat_err = 0.0F;
  result_of_planning_.lat_err.samples[0].lat_err_chg_rate = 0.0F;
  result_of_planning_.lat_err.samples[0].yaw_err = 0.0F;
  result_of_planning_.lat_err.samples[0].yaw_err_chg_rate = 0.0F;
  result_of_planning_.lat_err.samples[1].lat_err = 0.0F;
  result_of_planning_.lat_err.samples[1].lat_err_chg_rate = 0.0F;
  result_of_planning_.lat_err.samples[1].yaw_err = 0.0F;
  result_of_planning_.lat_err.samples[1].yaw_err_chg_rate = 0.0F;

  plan_var_t l_diff = veh_proj_on_major_ref_line_.l;
  plan_var_t h_diff = common::AngleDiff(
        curr_position_.path_point.heading, veh_proj_on_major_ref_line_.heading);
  if (common::com_abs(h_diff) < common::com_deg2rad(40.0F)) {
    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        pred_trj =temp_data_.sample_points_2d_1;
    pred_trj.Clear();
    major_ref_line_->CalcSamplePointsByLatOffsetting(l_diff, &pred_trj);
    if (!pred_path_of_vehicle_.Construct(pred_trj)) {
      LOG_ERR << "Failed to construct predicted vehicle path.";
      return false;
    }
    pred_path_of_vehicle_.GetSamplePoints(
          &(result_of_planning_.target_trajectory));
  } else {
    const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        pred_trj = driving_map_->GetPredictedPathOfVehicle();
    if (!pred_path_of_vehicle_.Construct(pred_trj)) {
      LOG_ERR << "Failed to construct predicted vehicle path.";
      return false;
    }
    pred_path_of_vehicle_.GetSamplePoints(
          &(result_of_planning_.target_trajectory));
  }

  return true;
}

plan_var_t TrajectoryPlanningStateLattice::CalcLeadingLen(
    plan_var_t lat_offset) {
  plan_var_t leading_length = 0.0F;

  plan_var_t t = 0;
  Int32_t lower = common::LerpInOrderedTable(
        param_.leading_distance_for_lka_ratio_table,
        veh_velocity_for_planning_, &t);
  leading_length = common::Lerp(
        param_.leading_distance_for_lka_ratio_table[lower].value,
        param_.leading_distance_for_lka_ratio_table[lower+1].value, t);

  if (lat_offset < 0.5F) {
#if 0
    plan_var_t ratio = lat_offset / 0.5F;
    leading_length =
        veh_velocity_for_planning_ *
        (param_.min_leading_time_for_lka +
         ratio * (param_.max_leading_time_for_lka -
                  param_.min_leading_time_for_lka));
#else
    plan_var_t dist = lat_offset - 0.2F;
    if (dist < 0) {
      dist = 0;
    }
    plan_var_t ratio = dist / 0.5F;
    leading_length += ratio * veh_velocity_for_planning_ * 2.0F;
#endif
  } else {
    leading_length =
        veh_velocity_for_planning_ * param_.max_leading_time_for_lka;
  }
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    if (leading_length < param_.min_leading_distance_for_lka_backing_mode) {
      leading_length = param_.min_leading_distance_for_lka_backing_mode;
    }
  } else {
    if (leading_length < param_.min_leading_distance_for_lka) {
      leading_length = param_.min_leading_distance_for_lka;
    }
  }
  if (leading_length > param_.max_leading_distance_for_lka) {
    leading_length = param_.max_leading_distance_for_lka;
  }

  // 曲率的影响
  plan_var_t leading_length_for_curve = leading_length;
  plan_var_t start_s = veh_proj_on_major_ref_line_.s;
  plan_var_t end_s = veh_proj_on_major_ref_line_.s + leading_length;
  const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& curve_segments =
      driving_map_->GetSmoothReferenceLineCurveInfo(major_ref_line_index_);
  for (Int32_t i = 0; i < curve_segments.Size(); ++i) {
    const common::Path::CurveSegment& curve_seg = curve_segments[i];
    if ((curve_seg.start_s + curve_seg.length) < start_s) {
      continue;
    }
    if (curve_seg.start_s > end_s) {
      continue;
    }
    plan_var_t s = curve_seg.start_s - start_s;
    if (s < 0) {
      s = 0;
    }
    plan_var_t abs_curvature = common::com_abs(curve_seg.max_curvature);

    plan_var_t ratio2 = (0.23F - abs_curvature) / 0.23F;
    if (ratio2 < 0.3F) {
      ratio2 = 0.3F;
    }

    t = 0;
    lower = common::LerpInOrderedTable(
          param_.leading_distance_correct_ratio_table_by_trj_curv,
          abs_curvature, &t);
    plan_var_t ratio = common::Lerp(
          param_.leading_distance_correct_ratio_table_by_trj_curv[lower].value,
          param_.leading_distance_correct_ratio_table_by_trj_curv[lower+1].value, t);

    // std::cout << "############################# calc leading len" << std::endl;
    // std::cout << "abs_curvature=" << abs_curvature << std::endl;
    // std::cout << "ratio=" << ratio << std::endl;
    // std::cout << "ratio2=" << ratio2 << std::endl;
    // std::cout << "limit_len=" << leading_length * ratio << std::endl;
    // std::cout << "#############################" << std::endl;

    plan_var_t limit_len = leading_length * ratio;
    plan_var_t suggest_len = leading_length;
    if (suggest_len > limit_len) {
      if (limit_len < (s - 2.0f)) {
        suggest_len = s - 2.0f;
      } else {
        suggest_len = limit_len;
      }
    }
    if (leading_length_for_curve > suggest_len) {
      leading_length_for_curve = suggest_len;
    }
  }
  leading_length = leading_length_for_curve;
  if (leading_length < 3.0F) {
    leading_length = 3.0F;
  }

  return (leading_length);
}

plan_var_t TrajectoryPlanningStateLattice::CalcStepLenOfLonGraphSample(
    plan_var_t start_s_on_ref_line) const {
  // Get current velocity of vehicle
  plan_var_t vehicle_v = curr_position_.v;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    if (vehicle_v < param_.min_backward_velocity_for_lon_sample) {
      vehicle_v = param_.min_backward_velocity_for_lon_sample;
    }
  } else {
    if (vehicle_v < param_.min_forward_velocity_for_lon_sample) {
      vehicle_v = param_.min_forward_velocity_for_lon_sample;
    }
  }

  plan_var_t nature_step_len = vehicle_v * param_.lon_sample_time_interval;
  plan_var_t step_len = nature_step_len;

  // 曲率的影响
  plan_var_t start_s = start_s_on_ref_line;
  plan_var_t end_s = start_s_on_ref_line + nature_step_len;
  const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& curve_segments =
      driving_map_->GetSmoothReferenceLineCurveInfo(major_ref_line_index_);
  for (Int32_t i = 0; i < curve_segments.Size(); ++i) {
    const common::Path::CurveSegment& curve_seg = curve_segments[i];
    if ((curve_seg.start_s + curve_seg.length) < start_s) {
      continue;
    }
    if (curve_seg.start_s > end_s) {
      continue;
    }
    plan_var_t s = curve_seg.start_s - start_s;
    if (s < 0) {
      s = 0;
    }
    plan_var_t abs_curvature = common::com_abs(curve_seg.max_curvature);

    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.lon_sample_step_len_limit_table_by_trj_curv,
          abs_curvature, &t);
    plan_var_t limit_len = common::Lerp(
          param_.lon_sample_step_len_limit_table_by_trj_curv[lower].value,
          param_.lon_sample_step_len_limit_table_by_trj_curv[lower+1].value, t);
    plan_var_t suggest_len = nature_step_len;
    if (suggest_len > limit_len) {
      if (limit_len < (s - 2.0f)) {
        suggest_len = s - 2.0f;
      } else {
        suggest_len = limit_len;
      }
    }
    if (step_len > suggest_len) {
      step_len = suggest_len;
    }
  }

  return (step_len);
}

Int32_t TrajectoryPlanningStateLattice::FindGraphSection(
    Int32_t start_sec_index,
    Int32_t target_lon_major,
    Int32_t target_lon_minor) const {
  Int32_t target_sec_index = -1;
  Int32_t road_graph_section_size = road_graph_.Size();

  for (Int32_t i = start_sec_index+1; i < road_graph_section_size; ++i) {
    const GraphSection& graph_section = road_graph_[i];
    if ((target_lon_major == graph_section.lon_major) &&
        (target_lon_minor == graph_section.lon_minor)) {
      target_sec_index = i;
      break;
    }
  }

  //if (target_sec_index < 0) {
  //  LOG_ERR << "Failed to find graph section ["
  //         << target_lon_major << ", " << target_lon_minor
  //         << "] starting from section " << start_sec_index << " .";
  //}

  return (target_sec_index);
}

void TrajectoryPlanningStateLattice::ConstructFullLinksBetweenGraphSections(
    Int32_t src_sec_index, Int32_t dest_sec_index) {
  COM_CHECK(dest_sec_index > src_sec_index);

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "     ConstructFullLinksBetweenGraphSections("
            << src_sec_index << ", " << dest_sec_index << ")" << std::endl;
#endif

  GraphSection& src_graph_section = road_graph_[src_sec_index];
  GraphSection& dest_graph_section = road_graph_[dest_sec_index];

  Int32_t src_lane_section_size = src_graph_section.lane_sections.Size();
  Int32_t dest_lane_section_size = dest_graph_section.lane_sections.Size();

  for (Int32_t i = 0; i < src_lane_section_size; ++i) {
    GraphSection::LaneSection& src_lane_section =
        src_graph_section.lane_sections[i];
    for (Int32_t j = 0; j < dest_lane_section_size; ++j) {
      GraphSection::LaneSection& dest_lane_section =
          dest_graph_section.lane_sections[j];

      Int32_t ref_line_index = driving_map_->FindReferenceLine(
            src_lane_section.proj_on_lane.point,
            src_lane_section.lane_index,
            dest_lane_section.lane_index);
      if (ref_line_index >= 0) {
        GraphNodeInfoForConstruction node_info;
        node_info.ref_line_index = ref_line_index;
        node_info.src_graph_sec_index = src_sec_index;
        node_info.dest_graph_sec_index = dest_sec_index;
        node_info.src_lane_sec_index = i;
        node_info.dest_lane_sec_index = j;
        ConstructFullLinksBetweenLaneSections(
              &node_info, &src_lane_section, &dest_lane_section);
      }
    }
  }
}

void TrajectoryPlanningStateLattice::ConstructParallelLinksBetweenGraphSections(
    bool expand, Int32_t src_sec_index, Int32_t dest_sec_index) {
  COM_CHECK(dest_sec_index > src_sec_index);

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "     ConstructParallelLinksBetweenGraphSections, from graph_sec("
            << src_sec_index << "), to graph_sec(" << dest_sec_index
            << ")" << std::endl;
#endif

  GraphSection& src_graph_section = road_graph_[src_sec_index];
  GraphSection& dest_graph_section = road_graph_[dest_sec_index];

  Int32_t src_lane_section_size = src_graph_section.lane_sections.Size();
  Int32_t dest_lane_section_size = dest_graph_section.lane_sections.Size();

  for (Int32_t i = 0; i < src_lane_section_size; ++i) {
    GraphSection::LaneSection& src_lane_section =
        src_graph_section.lane_sections[i];
    for (Int32_t j = 0; j < dest_lane_section_size; ++j) {
      GraphSection::LaneSection& dest_lane_section =
          dest_graph_section.lane_sections[j];

      bool need_constructing = false;
      if (expand) {
        if (src_lane_section.lat_major == dest_lane_section.lat_major) {
          need_constructing = true;
        } else if (src_lane_section.lat_major > dest_lane_section.lat_major) {
          if (dest_lane_section.lat_major == dest_graph_section.max_lat_major) {
            need_constructing = true;
          }
          if (src_lane_section.lat_major == src_graph_section.min_lat_major) {
            need_constructing = true;
          }
        } else {
          if (dest_lane_section.lat_major == dest_graph_section.min_lat_major) {
            need_constructing = true;
          }
          if (src_lane_section.lat_major == src_graph_section.max_lat_major) {
            need_constructing = true;
          }
        }
      } else {
        if (src_lane_section.lat_major == dest_lane_section.lat_major) {
          need_constructing = true;
        }
      }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
      std::cout << "       from lane_sec(" << i
                << ") lat_major(" << src_lane_section.lat_major
                << ") to lane_sec(" << j
                << ") lat_major(" << dest_lane_section.lat_major
                << "), construction=" << need_constructing << std::endl;
#endif

      if (need_constructing) {
        Int32_t ref_line_index = driving_map_->FindReferenceLine(
              src_lane_section.lane_index, dest_lane_section.lane_index);
        if (ref_line_index >= 0) {
          GraphNodeInfoForConstruction node_info;
          node_info.ref_line_index = ref_line_index;
          node_info.src_graph_sec_index = src_sec_index;
          node_info.dest_graph_sec_index = dest_sec_index;
          node_info.src_lane_sec_index = i;
          node_info.dest_lane_sec_index = j;
          ConstructParallelLinksBetweenLaneSections(
                expand, &node_info, &src_lane_section, &dest_lane_section);
        }
      }
    }
  }
}

void TrajectoryPlanningStateLattice::ConstructFullLinksBetweenLaneSections(
    GraphNodeInfoForConstruction* node_info,
    GraphSection::LaneSection* src_lane_section,
    GraphSection::LaneSection* dest_lane_section) {

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "     ConstructFullLinksBetweenLaneSections, from graph_sec("
            << node_info->src_graph_sec_index
            << ") lane_sec(" << node_info->src_lane_sec_index
            << ") lat_major(" << src_lane_section->lat_major
            << "), to graph_sec(" << node_info->dest_graph_sec_index
            << ") lane_sec(" << node_info->dest_lane_sec_index
            << ") lat_major(" << dest_lane_section->lat_major
            << ")" << std::endl;
#endif

  if (node_info->ref_line_index < 0) {
    node_info->ref_line_index = driving_map_->FindReferenceLine(
          src_lane_section->proj_on_lane.point,
          src_lane_section->lane_index,
          dest_lane_section->lane_index);

    if (node_info->ref_line_index < 0) {
      return;
    }
  }

  const common::Path& ref_line =
      driving_map_->GetSmoothReferenceLine(node_info->ref_line_index);
  ref_line.FindProjection(src_lane_section->proj_on_lane.point,
                          &(node_info->src_node_proj_on_ref));
  ref_line.FindProjection(dest_lane_section->proj_on_lane.point,
                          &(node_info->dest_node_proj_on_ref));

  if ((node_info->src_node_proj_on_ref.s < 0.1f) ||
      (node_info->src_node_proj_on_ref.s > (ref_line.total_length()-0.1f))) {
    return;
  }
  if ((node_info->dest_node_proj_on_ref.s < 0.1f) ||
      (node_info->dest_node_proj_on_ref.s > (ref_line.total_length()-0.1f))) {
    return;
  }
  if ((node_info->dest_node_proj_on_ref.s -
       node_info->src_node_proj_on_ref.s) < 1.0f) {
    return;
  }

  Int32_t src_nodes_size = src_lane_section->nodes.Size();
  Int32_t dest_nodes_size = dest_lane_section->nodes.Size();

  for (Int32_t i = 0; i < src_nodes_size; ++i) {
    GraphSection::GraphNode& src_node = src_lane_section->nodes[i];
    for (Int32_t j = 0; j < dest_nodes_size; ++j) {
      GraphSection::GraphNode& dest_node = dest_lane_section->nodes[j];

      node_info->src_graph_node_index = i;
      node_info->dest_graph_node_index = j;
      ConstructLinkBetweenGraphNodes(node_info, &src_node, &dest_node);
    }
  }
}

void TrajectoryPlanningStateLattice::ConstructParallelLinksBetweenLaneSections(
    bool expand,
    GraphNodeInfoForConstruction* node_info,
    GraphSection::LaneSection* src_lane_section,
    GraphSection::LaneSection* dest_lane_section) {

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "     ConstructParallelLinksBetweenLaneSections, from graph_sec("
            << node_info->src_graph_sec_index
            << ") lane_sec(" << node_info->src_lane_sec_index
            << ") lat_major(" << src_lane_section->lat_major
            << "), to graph_sec(" << node_info->dest_graph_sec_index
            << ") lane_sec(" << node_info->dest_lane_sec_index
            << ") lat_major(" << dest_lane_section->lat_major
            << ")" << std::endl;
#endif

  if (node_info->ref_line_index < 0) {
    node_info->ref_line_index = driving_map_->FindReferenceLine(
          src_lane_section->proj_on_lane.point,
          src_lane_section->lane_index,
          dest_lane_section->lane_index);

    if (node_info->ref_line_index < 0) {
      return;
    }
  }

  const common::Path& ref_line =
      driving_map_->GetSmoothReferenceLine(node_info->ref_line_index);
  ref_line.FindProjection(src_lane_section->proj_on_lane.point,
                          &(node_info->src_node_proj_on_ref));
  ref_line.FindProjection(dest_lane_section->proj_on_lane.point,
                          &(node_info->dest_node_proj_on_ref));

  if ((node_info->src_node_proj_on_ref.s < 0.1f) ||
      (node_info->src_node_proj_on_ref.s > (ref_line.total_length()-0.1f))) {
    return;
  }
  if ((node_info->dest_node_proj_on_ref.s < 0.1f) ||
      (node_info->dest_node_proj_on_ref.s > (ref_line.total_length()-0.1f))) {
    return;
  }
  if ((node_info->dest_node_proj_on_ref.s -
       node_info->src_node_proj_on_ref.s) < 1.0f) {
    return;
  }

  Int32_t src_nodes_size = src_lane_section->nodes.Size();
  Int32_t dest_nodes_size = dest_lane_section->nodes.Size();

  for (Int32_t i = 0; i < src_nodes_size; ++i) {
    GraphSection::GraphNode& src_node = src_lane_section->nodes[i];
    for (Int32_t j = 0; j < dest_nodes_size; ++j) {
      GraphSection::GraphNode& dest_node = dest_lane_section->nodes[j];

      bool need_constructing = false;
      if (expand) {
        if (src_node.lat_minor == dest_node.lat_minor) {
          need_constructing = true;
        } else if (src_node.lat_minor > dest_node.lat_minor) {
          if (dest_node.lat_minor == dest_lane_section->max_lat_minor) {
            need_constructing = true;
          }
          if (src_node.lat_minor == src_lane_section->min_lat_minor) {
            need_constructing = true;
          }
        } else {
          if (dest_node.lat_minor == dest_lane_section->min_lat_minor) {
            need_constructing = true;
          }
          if (src_node.lat_minor == src_lane_section->max_lat_minor) {
            need_constructing = true;
          }
        }
      } else {
        if (src_node.lat_minor == dest_node.lat_minor) {
          need_constructing = true;
        }
      }

      if (need_constructing) {
        node_info->src_graph_node_index = i;
        node_info->dest_graph_node_index = j;
        ConstructLinkBetweenGraphNodes(node_info, &src_node, &dest_node);
      }
    }
  }
}

void TrajectoryPlanningStateLattice::ConstructLinkBetweenGraphNodes(
    GraphNodeInfoForConstruction* node_info,
    GraphSection::GraphNode* src_node,
    GraphSection::GraphNode* dest_node) {
  Int32_t link_node_index = -1;
  GraphLinkListNode* link_node =
      road_graph_link_storage_.Allocate(&link_node_index);
  if (Nullptr_t == link_node) {
    LOG_ERR << "Can't add link to road graph, storage is full.";
    return;
  }
  src_node->link_list.PushBack(link_node_index, road_graph_link_storage_);
  src_node->link_num++;

  GraphSection::GraphNode::Link* link = &(link_node->link);
  link->link_index = link_node_index;

  link->ref_line_index = node_info->ref_line_index;

  link->node_from.graph_section_index = node_info->src_graph_sec_index;
  link->node_from.lane_section_index = node_info->src_lane_sec_index;
  link->node_from.node_index = node_info->src_graph_node_index;

  link->node_to.graph_section_index = node_info->dest_graph_sec_index;
  link->node_to.lane_section_index = node_info->dest_lane_sec_index;
  link->node_to.node_index = node_info->dest_graph_node_index;

  link->node_from.proj_on_ref = node_info->src_node_proj_on_ref;
  link->node_to.proj_on_ref = node_info->dest_node_proj_on_ref;

  link->node_from.s = link->node_from.proj_on_ref.s;
  link->node_from.l = src_node->lat_offset + link->node_from.proj_on_ref.l;
  if (0 == link->node_from.graph_section_index) {
    link->node_from.heading = curr_position_.path_point.heading;
    link->node_from.dl = common::com_tan(
          common::AngleDiff(link->node_from.proj_on_ref.heading,
                            link->node_from.heading));
    link->node_from.curvature = curr_position_.path_point.curvature;
    link->node_from.ddl = common::com_tan(
          link->node_from.curvature - link->node_from.proj_on_ref.curvature);
    //link->node_from.ddl = 0;
    //link->node_from.dl = 0;
#if 1
    plan_var_t dl_major_ref = common::com_tan(
          common::AngleDiff(veh_proj_on_major_ref_line_.heading,
                            link->node_from.heading));
    plan_var_t ddl_major_ref = common::com_tan(
          link->node_from.curvature - veh_proj_on_major_ref_line_.curvature);
    // std::cout << "  dl=" << link->node_from.dl << ", ddl=" << link->node_from.ddl
    //           << ", dl_major_ref=" << dl_major_ref << ", ddl_major_ref=" << ddl_major_ref
    //           << std::endl;
    if (common::com_abs(dl_major_ref) < common::com_abs(link->node_from.dl)) {
      link->node_from.dl = dl_major_ref;
    }
    if (common::com_abs(ddl_major_ref) < common::com_abs(link->node_from.ddl)) {
      link->node_from.ddl = ddl_major_ref;
    }
#endif
  } else {
    link->node_from.heading = link->node_from.proj_on_ref.heading;
    link->node_from.dl = 0;
    //link->node_from.curvature = link->node_from.proj_on_ref.curvature;
    plan_var_t tmp = 1.0F - link->node_from.proj_on_ref.curvature *
        (dest_node->lat_offset + link->node_to.proj_on_ref.l);
    if (common::com_abs(tmp) > 0.0001F) {
      link->node_from.curvature = link->node_from.proj_on_ref.curvature / tmp;
    } else {
      link->node_from.curvature = link->node_from.proj_on_ref.curvature;
    }

    link->node_from.ddl = 0;
  }

  link->node_to.s = link->node_to.proj_on_ref.s;
  link->node_to.l = dest_node->lat_offset + link->node_to.proj_on_ref.l;
  link->node_to.dl = 0;
  link->node_to.ddl = 0;

  // 构建link的曲线方程
  link->curve.Construct(
        link->node_from.l, link->node_from.dl, link->node_from.ddl,
        link->node_to.l, link->node_to.dl, link->node_to.ddl,
        link->node_to.s - link->node_from.s);

  // 根据link的曲线方程生成一系列采样点
  SampleLinkCurve(*link, param_.sample_step_len_of_link_curve,
                  &(link->sample_points));

  // 计算link曲线的曲率，并计算相应的减速度
  CalcCurvatureForLink(link);

  // 对link曲线进行碰撞分析，并计算相应的减速度
  TestCollisionForLink(link);

  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
  // 避免生成道路外的轨迹
  TestCollisionWithRoadBoundaryForLink(link);
  /* k005 pengc 2023-01-06 (end) */

  // 计算link曲线的Cost
  CalcCostForLink(link);
}

void TrajectoryPlanningStateLattice::SampleLinkCurve(
    const GraphSection::GraphNode::Link& link,
    plan_var_t step_len,
    common::StaticVector<common::PathPoint,
    common::Path::kMaxPathPointNum>* sample_points) const {
  const common::Path& ref_line = driving_map_->GetSmoothReferenceLine(
        link.ref_line_index);

  plan_var_t link_s = link.node_to.s - link.node_from.s;
  Int32_t sample_size = common::com_round(link_s / step_len);
  if (sample_size < 1) {
    sample_size = 1;
  }
  step_len = link_s / sample_size;

  plan_var_t s_sample = 0;
  plan_var_t l_sample = 0;
  plan_var_t dl_sample = 0;
  plan_var_t ddl_sample = 0;
  common::PathPoint path_point;
  for (Int32_t i = 0; i <= sample_size; ++i) {
    plan_var_t s = step_len * i;

    s_sample = link.node_from.s + s;
    l_sample = link.curve.Evaluate(0, s);
    dl_sample = link.curve.Evaluate(1, s);
    ddl_sample = link.curve.Evaluate(2, s);

    common::PathPoint* point = sample_points->Allocate();
    if (Nullptr_t == point) {
      LOG_ERR << "Can't add point to samples of link curve, storage is full.";
      break;
    }

    ref_line.FindSmoothPoint(s_sample, &path_point);
    point->point.set_x(path_point.point.x()
        - common::com_sin(path_point.heading) * l_sample);
    point->point.set_y(path_point.point.y()
        + common::com_cos(path_point.heading) * l_sample);
    point->heading = common::NormalizeAngle(
        path_point.heading + common::com_atan2(dl_sample, 1.0f));
    point->curvature = path_point.curvature
        + ddl_sample / ((1.0f+dl_sample*dl_sample)
          * common::com_sqrt(1.0f+dl_sample*dl_sample));
    if (0 == i) {
      point->s = 0;
    } else {
      common::PathPoint& prev_point = sample_points->GetData(i-1);
#if 0
      plan_var_t heading = common::com_atan2(
            point->point.y() - prev_point.point.y(),
            point->point.x() - prev_point.point.x());
      // printf("change heading from %0.2f to %0.2f\n",
      //        common::com_rad2deg(prev_point.heading),
      //        common::com_rad2deg(heading));
      prev_point.heading = heading;
      if (i > 1) {
        common::PathPoint& prev_point_2 = sample_points->GetData(i-2);
        plan_var_t delta_heading =
            common::NormalizeAngle(heading - prev_point_2.heading);
        // printf("change curvature from %0.3f to %0.3f\n",
        //        prev_point.curvature,
        //        delta_heading / (prev_point.s - prev_point_2.s));
        prev_point.curvature = delta_heading / (prev_point.s - prev_point_2.s);
      }
#endif
      point->s = prev_point.s + point->point.DistanceTo(prev_point.point);
    }
  }
}

void TrajectoryPlanningStateLattice::CalcCurvatureForLink(
    GraphSection::GraphNode::Link* link) {
  Int32_t sample_size = link->sample_points.Size();
  if (sample_size < 1) {
    return;
  }

  plan_var_t max_abs_curv_in_samples =
      common::com_abs(link->sample_points[0].curvature);
  for (Int32_t i = 1; i < sample_size; ++i) {
    plan_var_t abs_curv = common::com_abs(link->sample_points[i].curvature);
    if (abs_curv > max_abs_curv_in_samples) {
      max_abs_curv_in_samples = abs_curv;
    }
  }

  plan_var_t abs_curv_half = 0;
  Int32_t index_in_half = sample_size / 2;
  if (index_in_half > 0) {
    const common::PathPoint& point_in_half = link->sample_points[index_in_half];
    abs_curv_half = common::com_abs(common::AngleDiff(
                                      link->sample_points[0].heading,
                                    point_in_half.heading) / point_in_half.s);
  }

  plan_var_t abs_curv_full = 0;
  if (sample_size > 1) {
    abs_curv_full = common::com_abs(common::AngleDiff(
                                      link->sample_points.Front().heading,
                                      link->sample_points.Back().heading) /
                                    link->sample_points.Back().s);
  }

  plan_var_t max_abs_curvature = common::Max(
        max_abs_curv_in_samples,
        common::Max(abs_curv_half, abs_curv_full));

  // printf("max_abs_curv_in_samples=%0.3f, abs_curv_half=%0.3f, "
  //        "abs_curv_full=%0.3f\n",
  //        max_abs_curv_in_samples, abs_curv_half, abs_curv_full);

  link->abs_curvature = max_abs_curvature;
#if 0
  link->deceleration_by_curvature = CalcDecelerationByCurvature(
        link->node_from.s -
        driving_map_->GetProjDistOfCurrPositionOnRef(link->link_index),
        max_abs_curvature);
#else
  const GraphSection& graph_section_from =
      road_graph_[link->node_from.graph_section_index];
  link->deceleration_by_curvature = CalcDecelerationByCurvature(
        graph_section_from.proj_on_ref.s - veh_proj_on_major_ref_line_.s,
        max_abs_curvature);
#endif
}

void TrajectoryPlanningStateLattice::TestCollisionForLink(
    GraphSection::GraphNode::Link* link) {
  link->risky_obj_list.Clear();
  link->uncertain_list.Clear();

  Int32_t sample_size = link->sample_points.Size();
  if (sample_size < 1) {
    return;
  }

  common::UnorderedMap<Int32_t, Int32_t, MAX_RISKY_OBJ_IN_LINK>&
      risky_obj_lookup_tab_1 = temp_data_.risky_obj_lookup_tab_1;
  risky_obj_lookup_tab_1.Clear();
  common::UnorderedMap<Int32_t, Int32_t, MAX_RISKY_OBJ_IN_LINK>&
      risky_obj_lookup_tab_2 = temp_data_.risky_obj_lookup_tab_2;
  risky_obj_lookup_tab_2.Clear();

  const GraphSection& graph_section_from =
      road_graph_[link->node_from.graph_section_index];
  plan_var_t start_s_of_link =
        graph_section_from.proj_on_ref.s - veh_proj_on_major_ref_line_.s;

  driv_map::CollisionTestObj test_obj;
  test_obj.obb_circumradius = param_.vehicle_circumradius;
  test_obj.v = veh_velocity_for_planning_;
  if (test_obj.v < 0.1f) {
    test_obj.v = 0.1f;
  }
  driv_map::CollisionTestResult coll_test_ret;

  //link->collision_risk_value = 0;
  //link->dist_of_risky_obj_on_major_ref = -1;
  //link->static_collision_dist_to_risky_obj = -1;
  link->collision_obj.Clear();
  plan_var_t min_deceleration = 0.0f;
  Int32_t max_risk_value = 0;
  driv_map::CollisionTestParam test_param;
  for (Int32_t i = 0; i < sample_size; ++i) {
    const common::PathPoint& path_point = link->sample_points[i];

    CreateVehOBB(path_point.point, path_point.heading, &test_obj.obb);
    test_obj.s = path_point.s + start_s_of_link;
    test_obj.t = test_obj.s / test_obj.v;

    coll_test_ret.Clear();
    Int32_t risk_value = driving_map_->TestCollision(
          test_param, test_obj, &coll_test_ret);
    if (risk_value > max_risk_value) {
      max_risk_value = risk_value;
    }

    Int32_t risky_obj_list_size = coll_test_ret.risky_obj_list.Size();
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      const driv_map::CollisionTestResult::ObjInfo& test_ret =
          coll_test_ret.risky_obj_list[j];

      plan_var_t deceleration = CalcDecelerationByObstacle(
            path_point, test_ret);

      if (deceleration < min_deceleration) {
        min_deceleration = deceleration;
        //link->collision_risk_value = test_ret.risk_value;
        //link->dist_of_risky_obj_on_major_ref =
        //    test_ret.collision_s - param_.sample_step_len_of_link_curve;
        //link->static_collision_dist_to_risky_obj = test_ret.static_distance;
        link->collision_obj.risk_value = test_ret.risk_value;
        link->collision_obj.lon_dist_on_major_ref =
            test_ret.collision_s - param_.sample_step_len_of_link_curve;
        link->collision_obj.lat_dist = test_ret.static_distance;
        link->collision_obj.deceleration = deceleration;
      }

      if (deceleration < -0.01F) {
        AddCollisionInfoToList(test_ret, deceleration,
                               &(link->risky_obj_list),
                               &risky_obj_lookup_tab_1);
      }
    }

    Int32_t uncertain_list_size = coll_test_ret.uncertain_list.Size();
    for (Int32_t j = 0; j < uncertain_list_size; ++j) {
      const driv_map::CollisionTestResult::ObjInfo& test_ret =
          coll_test_ret.uncertain_list[j];
      const ad_msg::Obstacle& obj =
          driving_map_->GetObstacle(test_ret.obj_list_index);

      if (test_ret.collision_s >
          (common::Max(30.0F/3.6F, veh_velocity_for_planning_) * 2.0F)) {
        continue;
      }
      plan_var_t deceleration = CalcDecelerationByUncertainObstacle(
            path_point, test_ret);

      if ((test_ret.static_distance < 0.1F) &&
          (obj.type != ad_msg::OBJ_TYPE_CURB)) {
        AddCollisionInfoToList(test_ret, deceleration,
                               &(link->uncertain_list),
                               &risky_obj_lookup_tab_2);
      }
    }
  }

  // printf("risk_value=%d, lon_dist=%0.1f, lat_dist=%0.1f, deceleration=%0.2f\n",
  //        link->collision_obj.risk_value, link->collision_obj.lon_dist_on_major_ref,
  //        link->collision_obj.lat_dist, link->collision_obj.deceleration);

  // link->deceleration_by_obstacle = min_deceleration;
  // link->collision_obj.deceleration = min_deceleration;
}

/* k005 pengc 2023-01-06 (begin) */
// 添加功能: 道路边界碰撞测试
void TrajectoryPlanningStateLattice::TestCollisionWithRoadBoundaryForLink(
    GraphSection::GraphNode::Link* link) {
  link->is_collision_with_road_boundary = false;

  Int32_t sample_size = link->sample_points.Size();
  if (sample_size < 2) {
    return;
  }

  const GraphSection& graph_sec_from =
      road_graph_[link->node_from.graph_section_index];
  // const GraphSection::LaneSection& lane_sec_from =
  //     graph_sec_from.lane_sections[link->node_from.lane_section_index];
  // const GraphSection::GraphNode& node_from =
  //     lane_sec_from.nodes[link->node_from.node_index];

  // 只对车辆起点开始的段落进行与道路边界的碰撞分析
  if ((0 != graph_sec_from.lon_major) || (0 != graph_sec_from.lon_minor)) {
    return;
  }

  plan_var_t start_s_of_link =
        graph_sec_from.proj_on_ref.s - veh_proj_on_major_ref_line_.s;

  driv_map::CollisionTestObj test_obj;
  test_obj.obb_circumradius = param_.vehicle_circumradius;
  test_obj.v = veh_velocity_for_planning_;
  if (test_obj.v < 20.0F/3.6F) {
    test_obj.v = 20.0F/3.6F;
  }

  common::Vec2d prev_point = link->sample_points[0].point;
  bool collision_with_boundary = false;
  driv_map::CollisionTestResult coll_test_ret;
  for (Int32_t i = 1; i < sample_size; ++i) {
    const common::PathPoint& path_point = link->sample_points[i];

    plan_var_t distance = path_point.point.DistanceTo(prev_point);
    if (distance < 1.0F) {
      continue;
    }
    test_obj.obb.set_center(
          0.5F*(prev_point.x()+path_point.point.x()),
          0.5F*(prev_point.y()+path_point.point.y()));
    test_obj.obb.set_unit_direction(
          (path_point.point.x()-prev_point.x()) / distance,
          (path_point.point.y()-prev_point.y()) / distance);
    test_obj.obb.set_extents(0.5F*distance, 0.1F);
    test_obj.s = path_point.s + start_s_of_link;
    test_obj.t = test_obj.s / test_obj.v;

    coll_test_ret.Clear();
    driving_map_->TestCollisionWithRoadBoundary(test_obj, &coll_test_ret);

    Int32_t risky_obj_list_size = coll_test_ret.risky_obj_list.Size();
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      const driv_map::CollisionTestResult::ObjInfo& test_ret =
          coll_test_ret.risky_obj_list[j];

      if (test_ret.static_distance < 0.01F) {
        collision_with_boundary = true;
        break;
      }
    }

    if (collision_with_boundary) {
      break;
    }
  }


  if (collision_with_boundary) {
    link->is_collision_with_road_boundary = true;
#if 0
    const GraphSection& graph_sec_to =
        road_graph_[link->node_to.graph_section_index];
    const GraphSection::LaneSection& lane_sec_to =
        graph_sec_to.lane_sections[link->node_to.lane_section_index];
    const GraphSection::GraphNode& node_to =
        lane_sec_to.nodes[link->node_to.node_index];

    printf("link: [0,0,0,0]->[%d,%d,%d,%d] is collision with road boundary.\n",
           graph_sec_to.lon_major, graph_sec_to.lon_minor,
           lane_sec_to.lat_major, node_to.lat_minor);
#endif
  }
}
/* k005 pengc 2023-01-06 (end) */

void TrajectoryPlanningStateLattice::CalcCostForLink(
    GraphSection::GraphNode::Link* link) {
  const GraphSection& graph_section_to =
      road_graph_[link->node_to.graph_section_index];
  const GraphSection::LaneSection& lane_section_to =
      graph_section_to.lane_sections[link->node_to.lane_section_index];
  const GraphSection::GraphNode& node_to =
      lane_section_to.nodes[link->node_to.node_index];

  // 车道对应的Cost
  if (lane_section_to.lat_major > 0) {
    // Left lane
    link->cost.lane_cost =
        common::com_abs(lane_section_to.lat_major) *
        param_.left_lane_cost_ratio;
    /* k001 pengc 2022-03-18 (begin) */
    // 行为规划拒绝轨迹规划的变道请求时，轨迹规划可以向另一个车道进行规划
    if (!result_of_action_planning_.changing_lane_req.allow_auto_changing_to_left()) {
      link->cost.lane_cost += 100;
    }
    /* k001 pengc 2022-03-18 (end) */
  } else if (0 == lane_section_to.lat_major) {
    // Current lane
    link->cost.lane_cost =
        common::com_abs(lane_section_to.lat_major) *
        param_.left_lane_cost_ratio;
  } else {
    // Right lane
    link->cost.lane_cost =
        common::com_abs(lane_section_to.lat_major) *
        param_.right_lane_cost_ratio;
    /* k001 pengc 2022-03-18 (begin) */
    // 行为规划拒绝轨迹规划的变道请求时，轨迹规划可以向另一个车道进行规划
    if (!result_of_action_planning_.changing_lane_req.allow_auto_changing_to_right()) {
      link->cost.lane_cost += 100;
    }
    /* k001 pengc 2022-03-18 (end) */
  }

  // Routing车道对应的Cost
  // 由行为规划来决定是否变更到routing车道 pengc 2021.12.29
#if 0
  if (driving_map_->IsOnRoutingSegment(
        lane_section_to.lane_index, lane_section_to.proj_on_lane.s)) {
    link->cost.lane_cost += param_.routing_lane_cost;
  }
#endif

  // 车道内与中心线之间的横向偏差对应的Cost
  /* k002 pengc 2022-08-15 (begin) */
  /* 添加偏离车道线行驶的功能 */
#if 0
  if (node_to.lat_minor >= 0) {
    link->cost.lateral_offset_cost =
        common::com_abs(node_to.lat_offset) *
        param_.left_lateral_offset_cost_ratio;
  } else {
    link->cost.lateral_offset_cost =
        common::com_abs(node_to.lat_offset) *
        param_.right_lateral_offset_cost_ratio;
  }
#else
  if (0 == lane_section_to.lat_major) {
    if (node_to.lat_minor >= 0) {
      link->cost.lateral_offset_cost =
          common::com_abs(node_to.lat_offset - driving_off_center_.expected_lat_offset) *
          param_.left_lateral_offset_cost_ratio;
    } else {
      link->cost.lateral_offset_cost =
          common::com_abs(node_to.lat_offset - driving_off_center_.expected_lat_offset) *
          param_.right_lateral_offset_cost_ratio;
    }
  } else {
    if (node_to.lat_minor >= 0) {
      link->cost.lateral_offset_cost =
          common::com_abs(node_to.lat_offset) *
          param_.left_lateral_offset_cost_ratio;
    } else {
      link->cost.lateral_offset_cost =
          common::com_abs(node_to.lat_offset) *
          param_.right_lateral_offset_cost_ratio;
    }
  }
#endif
  /* k002 pengc 2022-08-15 (end) */

  // 轨迹的曲率对应的Cost
  link->cost.curvature_cost =
      common::com_abs(link->deceleration_by_curvature) *
      param_.curvature_cost_ratio;

  /// TODO: 考虑障碍物的纵向距离和横向距离
  // 与障碍物之间的碰撞风险对应的Cost
  link->cost.collision_cost =
      common::com_abs(link->collision_obj.deceleration) *
      param_.collision_cost_ratio;

#if 0
  printf("lane_cost=%d, lateral_offset_cost=%d, "
         "curvature_cost=%d(dec=%0.2f), collision_cost=%d(dec=%0.2f)\n",
         link->cost.lane_cost, link->cost.lateral_offset_cost,
         link->cost.curvature_cost, link->deceleration_by_curvature,
         link->cost.collision_cost, link->collision_obj.deceleration);
#endif
}

void TrajectoryPlanningStateLattice::CreateVehOBB(
    const common::Vec2d& pos, plan_var_t heading,
    common::OBBox2d* obb) const {
  const plan_var_t half_veh_width = param_.vehicle_width * 0.5f;
  const plan_var_t half_veh_length = param_.vehicle_length * 0.5f;

  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // for backing mode of vehicle
    heading = common::NormalizeAngle(heading+COM_PI);
  }

  obb->set_unit_direction(common::com_cos(heading), common::com_sin(heading));
  obb->set_center(pos + param_.dist_of_localization_to_center *
                  obb->unit_direction_x());
  obb->set_extents(half_veh_length, half_veh_width);
}

plan_var_t TrajectoryPlanningStateLattice::CalcDecelerationByCurvature(
    plan_var_t s, plan_var_t abs_curvature) const {
  if (abs_curvature < 0.001f) {
    return (0.0f);
  }

  plan_var_t velocity_limit = common::com_sqrt(
      param_.max_allowed_lateral_acceleration / abs_curvature);

  // printf("abs_curvature=%0.3f, v_limit=%0.1f\n",
  //        abs_curvature, velocity_limit*3.6F);

  plan_var_t curr_v = veh_velocity_for_planning_;
  if (velocity_limit > curr_v) {
    return (0.0f);
  }

  plan_var_t t = 2.0f*s / (curr_v+velocity_limit);
  if (t < 2.0f) {
    t = 2.0f;
  }

  return ((velocity_limit-curr_v) / t);
}

void TrajectoryPlanningStateLattice::ClassifyObstacle(
    const common::PathPoint& sample_point,
    const driv_map::CollisionTestResult::ObjInfo& obj_test_ret,
    ObjClassifyingInfo* classify_info) const {
  const ad_msg::Obstacle& obj =
      driving_map_->GetObstacle(obj_test_ret.obj_list_index);

  classify_info->Clear();

  plan_var_t side_threshold_front =
      param_.dist_of_localization_to_front + 1.0F + 2.0F;
  plan_var_t side_threshold_back =
      param_.dist_of_localization_to_rear + 2.0F*obj.obb.half_length + 2.0F;
  if (ad_msg::VEH_TRAILER_STATUS_CONNECTED == chassis_status_.trailer_status) {
    side_threshold_back += param_.trailer_length;
  }

  plan_var_t dist_to_obj =
      obj_test_ret.obj_s_ref - veh_proj_on_major_ref_line_.s;
  plan_var_t veh_speed = veh_velocity_for_planning_;
  if (veh_speed < 30.0F/3.6F) {
    veh_speed = 30.0F/3.6F;
  }

  //  printf("side_threshold_front=%0.1f, side_threshold_back=%0.1f, dist_to_obj=%0.1f\n",
  //         side_threshold_front, side_threshold_back, dist_to_obj);

  plan_var_t abs_angle_diff = common::com_abs(common::AngleDiff(
      sample_point.heading, obj_test_ret.obj_traj_point.path_point.heading));
  if (abs_angle_diff < common::com_deg2rad(60.0F)) {
    // 障碍物与自车同向行驶
    classify_info->obj_direction = driv_map::OBJ_DIRECTION_FORWARD;
  } else if (abs_angle_diff > common::com_deg2rad(120.0F)) {
    // 障碍物与自车逆向行驶
    classify_info->obj_direction = driv_map::OBJ_DIRECTION_BACKWARD;
  } else {
    // 障碍物横穿目标轨迹
    classify_info->obj_direction = driv_map::OBJ_DIRECTION_CROSSED;
  }

  if (dist_to_obj > side_threshold_front) {
    // printf("^^^ FRONT\n");
    // 前方
    classify_info->obj_position = ObjClassifyingInfo::OBJ_POSITION_FRONT;

    classify_info->distance = dist_to_obj - side_threshold_front;
    classify_info->t_gap = classify_info->distance / veh_speed;

    plan_var_t obj_v = 0.0F;
    if (driv_map::OBJ_DIRECTION_FORWARD == classify_info->obj_direction) {
      obj_v = obj.v * common::com_cos(abs_angle_diff);
    } else if (driv_map::OBJ_DIRECTION_BACKWARD == classify_info->obj_direction) {
      obj_v = obj.v * common::com_cos(abs_angle_diff);
    } else {
      obj_v = 0;
    }
    plan_var_t v_diff = obj_v - veh_speed;
    classify_info->ttc = 100.0F;
    if (v_diff < -0.01F) {
      classify_info->ttc = (dist_to_obj-side_threshold_front) / (-v_diff);
    }
  } else if ((-(side_threshold_back+0.1F) < dist_to_obj) &&
             (dist_to_obj < (side_threshold_front+0.1F))) {
    // printf("^^^ SIDE\n");
    // 侧方
    classify_info->obj_position = ObjClassifyingInfo::OBJ_POSITION_SIDE;

    classify_info->distance = 0.0F;
    classify_info->t_gap = 0.0F;

    classify_info->ttc = 0.0F;
  } else {
    // printf("^^^ BACK\n");
    // 后方
    classify_info->obj_position = ObjClassifyingInfo::OBJ_POSITION_BACK;

    classify_info->distance = -(dist_to_obj + side_threshold_back);
    classify_info->t_gap = classify_info->distance / veh_speed;

    if (driv_map::OBJ_DIRECTION_FORWARD == classify_info->obj_direction) {
      plan_var_t v_diff = obj.v - veh_speed;
      classify_info->ttc = 100.0F;
      if (v_diff > 0.01F) {
        classify_info->ttc = -(dist_to_obj+side_threshold_back) / v_diff;
      }
    } else if (driv_map::OBJ_DIRECTION_BACKWARD == classify_info->obj_direction) {
      classify_info->ttc = 100.0F;
    } else {
      classify_info->ttc = 100.0F;
    }
  }
}

plan_var_t TrajectoryPlanningStateLattice::CalcDecelerationByObstacle(
    const common::PathPoint& sample_point,
    const driv_map::CollisionTestResult::ObjInfo& obj_test_ret) const {
  // if (obj_test_ret.risk_value < 1) {
  //   return (0.0f);
  // }
  if (obj_test_ret.static_distance > 2.0f) {
    return (0.0f);
  }

  plan_var_t veh_speed = veh_velocity_for_planning_;
  if (veh_speed < 30.0F/3.6F) {
    veh_speed = 30.0F/3.6F;
  }

  const ad_msg::Obstacle& obj =
      driving_map_->GetObstacle(obj_test_ret.obj_list_index);

  ObjClassifyingInfo classify_info;
  ClassifyObstacle(sample_point, obj_test_ret, &classify_info);

  // bool additional_dec_req = false;
  plan_var_t s = obj_test_ret.collision_s;
  plan_var_t velocity_limit = 0.0F;
  plan_var_t relative_velocity_limit = 0.0F;
  if (!obj.dynamic) {
    // It is static obstacle
    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_static_obj,
          obj_test_ret.static_distance, &t);
    relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower+1].value, t);
    velocity_limit = relative_velocity_limit;

    if (obj_test_ret.static_distance < 0.2F) {
      if (ObjClassifyingInfo::OBJ_POSITION_FRONT == classify_info.obj_position) {
        if (classify_info.distance < 2.0F) {
          // 危险区域
          s = 0.0F;
          velocity_limit = 0.0F;
        }
      } else if (ObjClassifyingInfo::OBJ_POSITION_SIDE == classify_info.obj_position) {
        // 危险区域
        s = 0.0F;
        velocity_limit = 0.0F;
      } else {
        if (classify_info.distance < 5.0F) {
          // 危险区域
          s = 0.0F;
          velocity_limit = 0.0F;
        }
      }
    }
  } else {
    // It is dynamic obstacle
    plan_var_t abs_angle_diff = common::com_abs(common::AngleDiff(
        sample_point.heading, obj_test_ret.obj_traj_point.path_point.heading));
    if (obj_test_ret.static_distance < 0.2F) {
      // 将会碰撞，或者距离很接近
      if (driv_map::OBJ_DIRECTION_FORWARD == classify_info.obj_direction) {
        // 障碍物与自车同向行驶

        if (ObjClassifyingInfo::OBJ_POSITION_FRONT == classify_info.obj_position) {
          if (classify_info.t_gap < 0.5F) {
            // 危险区域
            s = 0.0F;
            velocity_limit = 0.0F;
          } else {
            velocity_limit = obj.v * common::com_cos(abs_angle_diff);
          }
        } else if (ObjClassifyingInfo::OBJ_POSITION_SIDE == classify_info.obj_position) {
          // 危险区域
          s = 0.0F;
          velocity_limit = 0.0F;
        } else {
          if ((classify_info.t_gap < 0.8F) || (classify_info.ttc < 5.0F)) {
            // 危险区域
            s = 0.0F;
            velocity_limit = 0.0F;
          } else {
            if (obj_test_ret.dynamic_distance < common::Max(5.0F, 0.5F*veh_speed)) {
              velocity_limit = obj.v * common::com_cos(abs_angle_diff);
              velocity_limit = common::Min(velocity_limit, curr_position_.v-10.0F/3.6F);
            } else {
              velocity_limit = veh_speed;
            }
          }
        }

#if 0
        // printf("vehicle_length=%0.1f, dist_of_localization_to_front=%0.1f"
        //        ", dist_of_localization_to_rear=%0.1f, obj_half_len=%0.1f\n",
        //        param_.vehicle_length, param_.dist_of_localization_to_front,
        //        param_.dist_of_localization_to_rear, obj.obb.half_length);
        plan_var_t dist_to_obj =
            obj_test_ret.obj_s_ref - veh_proj_on_major_ref_line_.s;
        printf("collision_s=%0.1f, dist_to_obj=%0.1f, s_dist=%0.2f, d_dist=%0.2f, pos=%d, t_gap=%0.2f, ttc=%0.2f, v_limit=%0.1f\n",
               obj_test_ret.collision_s, dist_to_obj, obj_test_ret.static_distance, obj_test_ret.dynamic_distance,
               classify_info.obj_position, classify_info.t_gap, classify_info.ttc,
               velocity_limit*3.6F);
#endif
      } else if (driv_map::OBJ_DIRECTION_BACKWARD == classify_info.obj_direction) {
        // 障碍物与自车逆向行驶
        velocity_limit = 0;
      } else {
        // 障碍物横穿目标轨迹
        velocity_limit = 0;
      }
    } else {
      // 不会碰撞
      plan_var_t t = 0;
      Int32_t lower = common::LerpInOrderedTable(
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
            obj_test_ret.static_distance, &t);
      relative_velocity_limit = common::Lerp(
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
            lower].value,
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
            lower+1].value, t);
      if (obj_test_ret.collision_s < param_.dist_of_localization_to_front) {
        // 障碍物在自车的侧方，无障碍通过
        velocity_limit = curr_position_.v;
      } else {
        // 障碍物在自车前方，需要根据与障碍物之间的距离限制相对速度
        if (driv_map::OBJ_DIRECTION_FORWARD == classify_info.obj_direction) {
          // 障碍物与自车同向行驶
          velocity_limit =
              obj.v * common::com_cos(abs_angle_diff) +
              relative_velocity_limit;
          /* longjiaoy 20231107 start */
          // 增加侧后方计算障碍物减速度加入动态距离的判断
          if (ObjClassifyingInfo::OBJ_POSITION_BACK == classify_info.obj_position) {
            if (obj_test_ret.dynamic_distance > common::Max(5.0F, 0.5F*veh_speed)) {
              velocity_limit = veh_speed;
            }
          }
          /* longjiaoy 20231107 end */
        } else if (driv_map::OBJ_DIRECTION_BACKWARD == classify_info.obj_direction) {
          // 障碍物与自车逆向行驶
          velocity_limit = curr_position_.v;
        } else {
          // 障碍物横穿目标轨迹
          velocity_limit = 0;
        }
      }
    }
  }

  if (velocity_limit < (2.0f/3.6f)) {
    velocity_limit = 0.0f;
  }

  // if (velocity_limit > veh_velocity_for_planning_) {
  //   return (0.0f);
  // }

  plan_var_t t = 2.0f*(s -
                       param_.sample_step_len_of_link_curve -
                       5.0f/*keep safe distance*/) / (veh_speed + velocity_limit);
  if (t < 0.5f) {
    t = 0.5f;
  }

  plan_var_t a = (velocity_limit - veh_speed) / t;
//  if (additional_dec_req) {
//    if (a > -2.0F) {
//      a = -2.0F;
//    }
//  }
  if (a > 0.0F) {
    a = 0.0F;
  }

  return (a);
}

plan_var_t TrajectoryPlanningStateLattice::CalcDecelerationByUncertainObstacle(
    const common::PathPoint& sample_point,
    const driv_map::CollisionTestResult::ObjInfo& obj_test_ret) const {
  if (obj_test_ret.risk_value < 1) {
    return (0.0f);
  }

  /* D001 fuyuanyi 2023-04-26 (begin) */
  /* 修改WBT index1 分支覆盖率未达标问题
     静态距离由0.01改为2.0 */
  /* D001 fuyuanyi 2023-04-26 (end) */

  if (obj_test_ret.static_distance > 2.0F) {
    return (0.0f);
  }

  const ad_msg::Obstacle& obj =
      driving_map_->GetObstacle(obj_test_ret.obj_list_index);

  plan_var_t velocity_limit = 0.0F;
  plan_var_t relative_velocity_limit = 0.0F;
  if (!obj.dynamic) {
    // Static obstacle
    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.relative_velocity_limit_table_by_dist_to_static_obj,
          obj_test_ret.static_distance, &t);
    relative_velocity_limit = common::Lerp(
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower].value,
          param_.relative_velocity_limit_table_by_dist_to_static_obj[
          lower+1].value, t);
    velocity_limit = relative_velocity_limit;
  } else {
    // Dynamic obstacle
    plan_var_t angle_diff = common::com_abs(common::AngleDiff(
        sample_point.heading, obj_test_ret.obj_traj_point.path_point.heading));
    if (obj_test_ret.static_distance < 0.2F) {
      // 将会碰撞，或者距离很接近
      if (angle_diff < common::com_deg2rad(60.0F)) {
        // 障碍物与自车同向行驶
        velocity_limit = obj.v * common::com_cos(common::com_abs(angle_diff));
      } else if (angle_diff > common::com_deg2rad(120.0F)) {
        // 障碍物与自车逆向行驶
        velocity_limit = 0;
      } else {
        // 障碍物横穿目标轨迹
        velocity_limit = 0;
      }
    } else {
      // 不会碰撞
      plan_var_t t = 0;
      Int32_t lower = common::LerpInOrderedTable(
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
            obj_test_ret.static_distance, &t);
      relative_velocity_limit = common::Lerp(
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
            lower].value,
            param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[
            lower+1].value, t);
      if (obj_test_ret.collision_s < param_.dist_of_localization_to_front) {
        // 障碍物在自车的侧方，无障碍通过
        velocity_limit = curr_position_.v;
      } else {
        // 障碍物在自车前方，需要根据与障碍物之间的距离限制相对速度
        if (angle_diff < common::com_deg2rad(60.0F)) {
          // 障碍物与自车同向行驶
          velocity_limit =
              obj.v * common::com_cos(common::com_abs(angle_diff)) +
              relative_velocity_limit;
        } else if (angle_diff > common::com_deg2rad(120.0F)) {
          // 障碍物与自车逆向行驶
          velocity_limit = curr_position_.v;
        } else {
          // 障碍物横穿目标轨迹
          velocity_limit = 0;
        }
      }
    }
  }

  if (velocity_limit < (2.0f/3.6f)) {
    velocity_limit = 0.0f;
  }

  if (velocity_limit > veh_velocity_for_planning_) {
    return (0.0f);
  }

  plan_var_t t = 2.0f*(obj_test_ret.collision_s -
                       param_.sample_step_len_of_link_curve -
                       5.0f/*keep safe distance*/) /
      (veh_velocity_for_planning_ + velocity_limit);
  if (t < 1.0f) {
    t = 1.0f;
  }

  return ((velocity_limit - veh_velocity_for_planning_) / t);
}

bool TrajectoryPlanningStateLattice::FindAllCandidateTrajectory() {
  // printf("\n###### FindAllCandidateTrajectory\n");

  // 若路网为空，则返回失败
  Int32_t graph_section_size = road_graph_.Size();
  if (graph_section_size < 1) {
    LOG_ERR << "The road graph is empty.";
    return (false);
  }

  // 清空候选轨迹
  for (Int32_t i = 0; i < candidate_trj_list_.Size(); ++i) {
    candidate_trj_list_[i].Clear();
  }
  candidate_trj_list_.Clear();

  // 定义堆栈
  TemporaryData::StackForFindingCandidateTrj& stack =
      temp_data_.stack_for_finding_candidate_trj;
  stack.Clear();

  // 将路网的头节点压入堆栈
  TemporaryData::StackForFindingCandidateTrj::StackNode*
      stack_node = stack.Allocate();
  if (Nullptr_t == stack_node) {
    LOG_ERR << "Storage of stack is full.";
    return (false);
  }
  // 将车辆当前位置的节点放入候选轨迹中
  CandidateTrajectory::TrjNode* trj_node =
      stack_node->trajectory_segments.Allocate();
  if (Nullptr_t == trj_node) {
    LOG_ERR << "Storage of trajectory in stack is full.";
    return (false);
  }
  // 填充第一个候选轨迹节点的信息
  trj_node->graph_section_index = 0;
  const GraphSection* graph_section =
      &(road_graph_[trj_node->graph_section_index]);
  if (graph_section->lane_sections.Empty()) {
    LOG_ERR << "The lane sections in graph section is empty.";
    return (false);
  }
  trj_node->lane_section_index = 0;
  const GraphSection::LaneSection* lane_section =
      &(graph_section->lane_sections[trj_node->lane_section_index]);
  if (lane_section->nodes.Empty()) {
    LOG_ERR << "The nodes in lane section is empty.";
    return (false);
  }
  trj_node->node_index = 0;
  const GraphSection::GraphNode* graph_node =
      &(lane_section->nodes[trj_node->node_index]);
  if (graph_node->link_list.Empty()) {
    LOG_ERR << "The links in graph node is empty.";
    return (false);
  }
  trj_node->link_index = graph_node->link_list.head();
  stack_node->link_access_index = trj_node->link_index;

  Int32_t loop_count = -1;
  while (!stack.Empty()) {
    loop_count++;

    // 取出栈顶节点
    stack_node = &(stack.Top());
    // 取出栈顶节点中的候选轨迹信息
    trj_node = &(stack_node->trajectory_segments.Back());
    // 取出路网的纵向段信息
    graph_section = &(road_graph_[trj_node->graph_section_index]);
    // 取出路网的车道段信息
    lane_section =
        &(graph_section->lane_sections[trj_node->lane_section_index]);
    // 取出路网的节点信息
    graph_node = &(lane_section->nodes[trj_node->node_index]);

    // std::cout << "In loop " << loop_count << ":" << std::endl;
    // for (Int32_t i = 0; i < stack_node->trajectory_segments.Size(); ++i) {
    // std::cout << " node["
    //           << stack_node->trajectory_segments[i].graph_section_index
    //           << "," << stack_node->trajectory_segments[i].lane_section_index
    //           << "," << stack_node->trajectory_segments[i].node_index
    //           << "," << stack_node->trajectory_segments[i].link_index
    //           << "], ";
    // }
    // std::cout << " [end]" << std::endl;

    if (stack_node->link_access_index < 0) {
      // 此节点的所有link都已经访问过了
      stack.Pop();
      // std::cout << "There isn't next link." << std::endl;
    } else {
      // 取出link信息
      GraphSection::GraphNode::Link* link =
          &(road_graph_link_storage_[stack_node->link_access_index].link);
      trj_node->link_index = stack_node->link_access_index;
      // 更新到下一个link的索引
      stack_node->link_access_index =
          road_graph_link_storage_[stack_node->link_access_index].next;

      // 增加一个候选轨迹
      CandidateTrajectory* trj = candidate_trj_list_.Allocate();
      if (Nullptr_t == trj) {
        LOG_ERR << "The storage of candidate trajectory list is full.";
        break;
      }
      trj->trajectory_segments = stack_node->trajectory_segments;

      // 一直遍历到最深处的节点
      while ((link->node_to.graph_section_index >= 0) &&
             (link->node_to.lane_section_index >= 0) &&
             (link->node_to.node_index >= 0)) {
        // 取出图的纵向段信息
        graph_section = &(road_graph_[link->node_to.graph_section_index]);
        // 取出图的横向车道段信息
        lane_section =
            &(graph_section->lane_sections[link->node_to.lane_section_index]);
        // 取出图的节点信息
        graph_node =&(lane_section->nodes[link->node_to.node_index]);

        // 向轨迹中增加新的轨迹段
        trj_node = trj->trajectory_segments.Allocate();
        if (Nullptr_t == trj_node) {
          LOG_ERR << "Storage of trajectory in stack is full.";
          return (false);
        }
        if (graph_node->link_list.Empty()) {
          // 已经最后一个节点了
          trj_node->graph_section_index = link->node_to.graph_section_index;
          trj_node->lane_section_index = link->node_to.lane_section_index;
          trj_node->node_index = link->node_to.node_index;
          trj_node->link_index = -1;
          // std::cout << "Get the last node." << std::endl;
          break;
        }
        // 填充新轨迹段信息
        trj_node->graph_section_index = link->node_to.graph_section_index;
        trj_node->lane_section_index = link->node_to.lane_section_index;
        trj_node->node_index = link->node_to.node_index;
        trj_node->link_index = graph_node->link_list.head();
        
        // 将这个轨迹段分支添加到栈中
        stack_node = stack.Allocate();
        if (Nullptr_t == stack_node) {
          LOG_ERR << "Storage of stack is full.";
          return (false);
        }
        stack_node->trajectory_segments = trj->trajectory_segments;
        // 当前link已经访问了，所以需要更新到下一个link的索引
        stack_node->link_access_index =
            road_graph_link_storage_[trj_node->link_index].next;

        // 更新到下一个节点
        link = &(road_graph_link_storage_[trj_node->link_index].link);
      }
    }
  }

  Int32_t candidate_trj_num = candidate_trj_list_.Size();
  if (candidate_trj_num < 1) {
    LOG_ERR << "There is not candidate trajectory in road graph.";
    return false;
  }

  // printf("candidate_trj_num=%d\n", candidate_trj_num);

  /// TODO: Add cost for the case of changing lane
  // Calculate cost for all candiate trajectories.
  for (Int32_t i = 0; i < candidate_trj_num; ++i) {
    CandidateTrajectory& trj = candidate_trj_list_[i];

    trj.risky_obj_list.Clear();
    trj.uncertain_list.Clear();
    common::UnorderedMap<Int32_t, Int32_t, MAX_RISKY_OBJ_IN_LINK>&
        risky_obj_lookup_tab_1 = temp_data_.risky_obj_lookup_tab_1;
    risky_obj_lookup_tab_1.Clear();
    common::UnorderedMap<Int32_t, Int32_t, MAX_RISKY_OBJ_IN_LINK>&
        risky_obj_lookup_tab_2 = temp_data_.risky_obj_lookup_tab_2;
    risky_obj_lookup_tab_2.Clear();

    /* k005 pengc 2023-01-06 (begin) */
    // 添加功能: 道路边界碰撞测试
    // 避免生成道路外的轨迹
    trj.is_collision_with_road_boundary = false;
    /* k005 pengc 2023-01-06 (end) */

    Int32_t trj_seg_num = trj.trajectory_segments.Size();
    if (trj_seg_num < 1) {
      trj.cost.total_cost = 99999999;
      LOG_ERR << "Detected empty segments in trajectory[" << i << "]";
      continue;
    }

    const CandidateTrajectory::TrjNode* trj_node =
        &(trj.trajectory_segments[0]);
    if (trj_node->link_index < 0) {
      trj.cost.total_cost = 99999999;
      LOG_ERR << "Detected invalid link index in trajectory[" << i << "]";
      continue;
    }

    const GraphSection::GraphNode::Link* first_link =
        &(road_graph_link_storage_[trj_node->link_index].link);
    if (first_link->node_to.graph_section_index < 0) {
      trj.cost.total_cost = 99999999;
      LOG_ERR << "Detected invalid graph section "
                 "index in trajectory[" << i << "]";
      continue;
    }
    if (first_link->node_to.lane_section_index < 0) {
      trj.cost.total_cost = 99999999;
      LOG_ERR << "Detected invalid lane section "
                 "index in trajectory[" << i << "]";
      continue;
    }
    if (first_link->node_to.node_index < 0) {
      trj.cost.total_cost = 99999999;
      LOG_ERR << "Detected invalid node "
                 "index in trajectory[" << i << "]";
      continue;
    }
    const GraphSection& dest_graph_sec =
        road_graph_[first_link->node_to.graph_section_index];
    const GraphSection::LaneSection& dest_lane_sec =
        dest_graph_sec.lane_sections[first_link->node_to.lane_section_index];
    const GraphSection::GraphNode& dest_node =
        dest_lane_sec.nodes[first_link->node_to.node_index];


    const GraphSection::GraphNode::Link* link = first_link;
    trj.cost.lane_cost = link->cost.lane_cost;
    trj.cost.lateral_offset_cost = link->cost.lateral_offset_cost;
    trj.cost.lka_cost =
        dest_graph_sec.lon_major * param_.lka_cost_ratio_major +
        dest_graph_sec.lon_minor * param_.lka_cost_ratio_minor;
    trj.cost.curvature_cost = link->cost.curvature_cost;
    trj.cost.collision_cost = link->cost.collision_cost;

    trj.abs_curvature = link->abs_curvature;
    trj.deceleration_by_curvature = link->deceleration_by_curvature;
    //trj.collision_risk_value = link->collision_risk_value;
    //trj.dist_of_risky_obj_on_major_ref = link->dist_of_risky_obj_on_major_ref;
    //trj.static_collision_dist_to_risky_obj = link->static_collision_dist_to_risky_obj;
    //trj.deceleration_by_obstacle = link->deceleration_by_obstacle;
    trj.collision_obj = link->collision_obj;

    for (Int32_t k = 0; k < link->risky_obj_list.Size(); ++k) {
      const CollisionInfo& obj = link->risky_obj_list[k];
      AddCollisionInfoToList(obj.test_ret, obj.deceleration,
                             &(trj.risky_obj_list), &risky_obj_lookup_tab_1);
    }
    for (Int32_t k = 0; k < link->uncertain_list.Size(); ++k) {
      const CollisionInfo& obj = link->uncertain_list[k];
      AddCollisionInfoToList(obj.test_ret, obj.deceleration,
                             &(trj.uncertain_list), &risky_obj_lookup_tab_2);
    }

    /* k005 pengc 2023-01-06 (begin) */
    // 添加功能: 道路边界碰撞测试
    // 避免生成道路外的轨迹
    trj.is_collision_with_road_boundary = link->is_collision_with_road_boundary;
    /* k005 pengc 2023-01-06 (end) */

    for (Int32_t j = 1; j < trj_seg_num; ++j) {
      trj_node = &(trj.trajectory_segments[j]);
      if (trj_node->link_index < 0) {
        break;
      }

      link = &(road_graph_link_storage_[trj_node->link_index].link);
      if (link->cost.curvature_cost > trj.cost.curvature_cost) {
        trj.cost.curvature_cost = link->cost.curvature_cost;
      }
      if (link->cost.collision_cost > trj.cost.collision_cost) {
        trj.cost.collision_cost = link->cost.collision_cost;
      }

      if (link->deceleration_by_curvature <
          (trj.deceleration_by_curvature-0.01F)) {
        trj.abs_curvature = link->abs_curvature;
        trj.deceleration_by_curvature = link->deceleration_by_curvature;
      } else if ((common::com_abs(trj.deceleration_by_curvature -
                                  link->deceleration_by_curvature) < 0.1F)) {
        if (link->abs_curvature > trj.abs_curvature) {
          trj.abs_curvature = link->abs_curvature;
          trj.deceleration_by_curvature = link->deceleration_by_curvature;
        }
      } else {
        // Hold previous value
      }

      if (link->collision_obj.deceleration <
          (trj.collision_obj.deceleration-0.01F)) {
        trj.collision_obj = link->collision_obj;
      } else if ((common::com_abs(trj.collision_obj.deceleration -
                                  link->collision_obj.deceleration) < 0.1F)) {
        if (link->collision_obj.risk_value > trj.collision_obj.risk_value) {
          trj.collision_obj = link->collision_obj;
        } else if (link->collision_obj.risk_value == trj.collision_obj.risk_value) {
          if (link->collision_obj.lon_dist_on_major_ref <
              (trj.collision_obj.lon_dist_on_major_ref-0.1F)) {
            trj.collision_obj = link->collision_obj;
          } else if ((common::com_abs(trj.collision_obj.lon_dist_on_major_ref -
                                      link->collision_obj.lon_dist_on_major_ref) < 0.1F)) {
            if (link->collision_obj.lat_dist < trj.collision_obj.lat_dist) {
              trj.collision_obj = link->collision_obj;
            }
          } else {
            // Hold previous value
          }
        } else {
          // Hold previous value
        }
      } else {
        // Hold previous value
      }

      for (Int32_t k = 0; k < link->risky_obj_list.Size(); ++k) {
        const CollisionInfo& obj = link->risky_obj_list[k];
        AddCollisionInfoToList(obj.test_ret, obj.deceleration,
                               &(trj.risky_obj_list), &risky_obj_lookup_tab_1);
      }
      for (Int32_t k = 0; k < link->uncertain_list.Size(); ++k) {
        const CollisionInfo& obj = link->uncertain_list[k];
        AddCollisionInfoToList(obj.test_ret, obj.deceleration,
                               &(trj.uncertain_list), &risky_obj_lookup_tab_2);
      }

      /* k005 pengc 2023-01-06 (begin) */
      // 添加功能: 道路边界碰撞测试
      // 避免生成道路外的轨迹
      if (link->is_collision_with_road_boundary) {
        trj.is_collision_with_road_boundary =
            link->is_collision_with_road_boundary;
      }
      /* k005 pengc 2023-01-06 (end) */
    }

    trj_node = &(trj.trajectory_segments.Back());
    trj.cost.trj_len_cost =
        trj_node->graph_section_index *
        param_.trj_len_cost_ratio;

    /* k005 pengc 2023-01-06 (begin) */
    // 添加功能: 道路边界碰撞测试
    trj.cost.collision_with_road_boundary_cost = 0;
    if (trj.is_collision_with_road_boundary) {
      // printf("  trj[%d] is collision with road boundary.\n", i);

      if ((0 == dest_lane_sec.lat_major) && (0 == dest_node.lat_minor) &&
          (dest_graph_sec.lon_major < 2)) {
        // 主参考线上较近的轨迹, low cost
        trj.cost.collision_with_road_boundary_cost = 0;
#if 0
        const GraphSection& src_graph_sec =
            road_graph_[first_link->node_from.graph_section_index];
        const GraphSection::LaneSection& src_lane_sec =
            src_graph_sec.lane_sections[first_link->node_from.lane_section_index];
        const GraphSection::GraphNode& src_node =
            src_lane_sec.nodes[first_link->node_from.node_index];
        printf("    >> On major reference line, from [%d,%d,%d,%d](s=%0.1f) to [%d,%d,%d,%d](s=%0.1f)\n",
               src_graph_sec.lon_major, src_graph_sec.lon_minor,
               src_lane_sec.lat_major, src_node.lat_minor,
               first_link->node_from.s,
               dest_graph_sec.lon_major, dest_graph_sec.lon_minor,
               dest_lane_sec.lat_major, dest_node.lat_minor,
               first_link->node_to.s);
#endif
      } else {
        // 主参考线上较近的轨迹, hight cost
        trj.cost.collision_with_road_boundary_cost =
            param_.collision_with_road_boundary_cost;
#if 0
        const GraphSection& src_graph_sec =
            road_graph_[first_link->node_from.graph_section_index];
        const GraphSection::LaneSection& src_lane_sec =
            src_graph_sec.lane_sections[first_link->node_from.lane_section_index];
        const GraphSection::GraphNode& src_node =
            src_lane_sec.nodes[first_link->node_from.node_index];
        printf("    >> Not on major reference line, from [%d,%d,%d,%d](s=%0.1f) to [%d,%d,%d,%d](s=%0.1f)\n",
               src_graph_sec.lon_major, src_graph_sec.lon_minor,
               src_lane_sec.lat_major, src_node.lat_minor,
               first_link->node_from.s,
               dest_graph_sec.lon_major, dest_graph_sec.lon_minor,
               dest_lane_sec.lat_major, dest_node.lat_minor,
               first_link->node_to.s);
#endif
      }
    }
    /* k005 pengc 2023-01-06 (end) */

    trj.cost.total_cost =
        trj.cost.lane_cost +
        trj.cost.lateral_offset_cost +
        trj.cost.lka_cost +
        trj.cost.curvature_cost +
        trj.cost.collision_cost +
        trj.cost.trj_len_cost +
        trj.cost.collision_with_road_boundary_cost;

    // printf("  trj[%d], dec=%0.2f, collision_cost=%d, total_cost=%d\n",
    //        i, trj.collision_obj.deceleration, trj.cost.collision_cost, trj.cost.total_cost);

    /* k001 pengc 2022-04-09 (begin) */
    // 对当前车辆位置的左右偏移进行碰撞分析，避免绕障碍物时提早回到中心车道
    trj.cost.ext_bypassing_cost = 0;
    if ((0 == dest_lane_sec.lat_major) && timers_.hold_bypassing.IsActive()) {
      common::PathPoint offset_point;
      // 假设当前偏移的位置处于前向一定位置处，用于计算减速度
      offset_point.s = veh_proj_on_major_ref_line_.s + 10.0F;
      offset_point.l = dest_node.lat_offset;
      offset_point.heading = veh_proj_on_major_ref_line_.heading;
      offset_point.point.set_x(
            veh_proj_on_major_ref_line_.point.x() -
            offset_point.l * common::com_sin(offset_point.heading));
      offset_point.point.set_y(
            veh_proj_on_major_ref_line_.point.y() +
            offset_point.l * common::com_cos(offset_point.heading));

      driv_map::CollisionTestResult coll_test_ret;
      driv_map::CollisionTestParam test_param;
      driv_map::CollisionTestObj test_obj;
      test_obj.obb_circumradius = param_.vehicle_circumradius;
      test_obj.v = veh_velocity_for_planning_;
      if (test_obj.v < 10.0F/3.6F) {
        test_obj.v = 10.0F/3.6F;
      }
      CreateVehOBB(offset_point.point, curr_position_.path_point.heading, &test_obj.obb);
      test_obj.s = offset_point.s - veh_proj_on_major_ref_line_.s;
      test_obj.t = test_obj.s / test_obj.v;
      driving_map_->TestCollision(test_param, test_obj, &coll_test_ret);

      plan_var_t min_deceleration = 0.0f;
      Int32_t risky_obj_list_size = coll_test_ret.risky_obj_list.Size();
      for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
        const driv_map::CollisionTestResult::ObjInfo& test_ret =
            coll_test_ret.risky_obj_list[j];

        plan_var_t deceleration = CalcDecelerationByObstacle(
              offset_point, test_ret);
        if (deceleration < min_deceleration) {
          min_deceleration = deceleration;
        }
      }

      trj.cost.ext_bypassing_cost =
          common::com_abs(min_deceleration) * param_.collision_cost_ratio;
#if 0
      // Uncertain objects
      Int32_t uncertain_list_size = coll_test_ret.uncertain_list.Size();
      for (Int32_t j = 0; j < uncertain_list_size; ++j) {
        const driv_map::CollisionTestResult::ObjInfo& test_ret =
            coll_test_ret.uncertain_list[j];
        const ad_msg::Obstacle& obj =
            driving_map_->GetObstacle(test_ret.obj_list_index);

        plan_var_t deceleration = CalcDecelerationByUncertainObstacle(
              offset_point, test_ret);

        if ((test_ret.static_distance < 0.1F) &&
            (obj.type != ad_msg::OBJ_TYPE_CURB)) {
        }
      }
#endif

      // 在一定的时间范围内，尽量保持之前的避让趋势
      plan_var_t prev_lat_offset = timers_.hold_bypassing.GetUserDataFloat32(0);
      // printf("    prev_lat_offset=%0.2f\n", prev_lat_offset);
      plan_var_t lat_offset = offset_point.l - prev_lat_offset;
      Int32_t lat_offset_cost = 0;
      if (prev_lat_offset < 0) {
        // bypassing to right
        if (lat_offset < 0) {
          // prio to right
          lat_offset_cost = common::com_abs(lat_offset) *
              param_.left_lateral_offset_cost_ratio;
        } else {
          lat_offset_cost = common::com_abs(lat_offset) *
              param_.right_lateral_offset_cost_ratio;
        }
      } else {
        // bypassing to left
        if (lat_offset < 0) {
          lat_offset_cost = common::com_abs(lat_offset) *
              param_.right_lateral_offset_cost_ratio;
        } else {
          // prio to left
          lat_offset_cost = common::com_abs(lat_offset) *
              param_.left_lateral_offset_cost_ratio;
        }
      }
      trj.cost.ext_bypassing_cost +=
          lat_offset_cost - trj.cost.lateral_offset_cost;

      // printf("       lat=%0.2f, offset=%0.2f, prev_offset=%0.2f, lat_cost=%d, origen_lat_cost=%d, cost=%d\n",
      //         offset_point.l, lat_offset, prev_lat_offset, lat_offset_cost,
      //         trj.cost.lateral_offset_cost, trj.cost.ext_bypassing_cost);
    }
    /* k001 pengc 2022-04-09 (end) */
  }

#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  std::cout << "There are " << candidate_trj_num
            << " candidate trajectorys in road graph."
            << std::endl;
  for (Int32_t i = 0; i < candidate_trj_num; ++i) {
    const CandidateTrajectory& trj = candidate_trj_list_[i];
    const common::StaticVector<CandidateTrajectory::TrjNode,
        MAX_GRAPH_SECTION_NUM>& trj_segs = trj.trajectory_segments;
    Int32_t trj_seg_num = trj_segs.Size();
    std::cout << ">>> Num[" << i << "], there are " << trj_seg_num
              << " segments in this trajectory." << std::endl;
    std::cout << "    lane_cost=" << trj.cost.lane_cost
              << ", lat_offset_cost=" << trj.cost.lateral_offset_cost
              << ", curvature_cost=" << trj.cost.curvature_cost
              << ", collision_cost=" << trj.cost.collision_cost
              << ", trj_len_cost=" << trj.cost.trj_len_cost
              << ", total_cost=" << trj.cost.total_cost
              << std::endl;
    for (Int32_t j = 0; j < trj_seg_num; ++j) {
      const CandidateTrajectory::TrjNode& node = trj_segs[j];
      const GraphSection& graph_sec = road_graph_[node.graph_section_index];
      const GraphSection::LaneSection& lane_sec =
          graph_sec.lane_sections[node.lane_section_index];
      const GraphSection::GraphNode& graph_nod =
          lane_sec.nodes[node.node_index];

      const GraphSection::GraphNode::Link* link = Nullptr_t;
      if (node.link_index >= 0) {
        link = &(road_graph_link_storage_[node.link_index].link);
      }

      std::cout << "           node[" << node.graph_section_index
                << "," << node.lane_section_index
                << "," << node.node_index
                << "," << node.link_index
                << ",lane_id=" << driving_map_->GetLaneIdByIndex(lane_sec.lane_index).id
                << "], \tnum[" << graph_sec.lon_major
                << "," << graph_sec.lon_minor
                << "," << lane_sec.lat_major
                << "," << graph_nod.lat_minor
                << "]";
      if (Nullptr_t != link) {
        std::cout << ", \tlink from node["
                  << link->node_from.graph_section_index
                  << "," << link->node_from.lane_section_index
                  << "," << link->node_from.node_index
                  << "], to node[" << link->node_to.graph_section_index
                  << "," << link->node_to.lane_section_index
                  << "," << link->node_to.node_index
                  << "]"
                  << std::endl;
      } else {
        std::cout << std::endl;
      }
    }
  }
#endif

  return true;
}

bool TrajectoryPlanningStateLattice::SampleCandidateTrajectory(
    Int32_t candidate_trj_idx,
    common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>*
        sample_points) const {
  sample_points->Clear();

  if ((candidate_trj_idx < 0) ||
      (candidate_trj_idx >= candidate_trj_list_.Size())) {
    // LOG_ERR << "Invalid candidate trajectory index.";
    return false;
  }

  const CandidateTrajectory& candidate_trj =
      candidate_trj_list_[candidate_trj_idx];
  const Int32_t candidate_trj_seg_num =
      candidate_trj.trajectory_segments.Size();
  if (candidate_trj_seg_num < 1) {
    LOG_ERR << "Detected invalid candidate trajectory, because the number of "
               "segments of candidate trajectory is too small.";
    return false;
  }

  for (Int32_t i = 0; i < candidate_trj_seg_num; ++i) {
    const CandidateTrajectory::TrjNode& trj_node =
        candidate_trj.trajectory_segments[i];
    if (trj_node.link_index < 0) {
      break;
    }

    const GraphSection::GraphNode::Link& link =
        road_graph_link_storage_[trj_node.link_index].link;

    SampleLinkCurve(link, 1.0f, sample_points);
    if ((candidate_trj_seg_num-1) != i) {
      sample_points->PopBack();
    }
  }

  Int32_t sample_points_num = sample_points->Size();
  if (sample_points_num < 2) {
    LOG_ERR << "Failed to sample candidate trajectory, because the number "
               "of sample points is too small";
    return false;
  }

  return true;
}


bool TrajectoryPlanningStateLattice::FindOptimalTrajectory() {
  optimal_trj_index_.Clear();

  Int32_t candidate_trj_num = candidate_trj_list_.Size();
  if (candidate_trj_num < 1) {
    LOG_ERR << "There is not candiate trajectory in road graph.";
    return false;
  }

  Int32_t min_cost_trj_idx_global = -1;
  Int32_t min_cost_global = 99999999;

  /* k001 pengc 2022-04-03 (begin) */
  // 避免避让的轨迹左右频繁跳动
  Int32_t min_cost_trj_idx_curr_lane_left = -1;
  Int32_t min_cost_curr_lane_left = 99999999;
  Int32_t min_cost_trj_idx_curr_lane_right = -1;
  Int32_t min_cost_curr_lane_right = 99999999;
  /* k001 pengc 2022-04-03 (end) */
  Int32_t min_cost_trj_idx_curr_lane = -1;
  Int32_t min_cost_curr_lane = 99999999;
  Int32_t min_cost_trj_idx_lka = -1;
  Int32_t min_cost_lka = 99999999;
  /* k001 pengc 2022-04-10 (begin) */
  // 在一定的时间范围内，尽量保持之前的避让趋势
  Int32_t min_cost_trj_idx_curr_lane_bypassing = -1;
  Int32_t min_cost_curr_lane_bypassing = 99999999;
  /* k001 pengc 2022-04-10 (end) */


  Int32_t min_cost_trj_idx_left_lane = -1;
  Int32_t min_cost_left_lane = 99999999;
  Int32_t min_cost_trj_idx_right_lane = -1;
  Int32_t min_cost_right_lane = 99999999;

  for (Int32_t i = 0; i < candidate_trj_num; ++i) {
    CandidateTrajectory& trj = candidate_trj_list_[i];

    Int32_t trj_seg_num = trj.trajectory_segments.Size();
    if (trj_seg_num < 1) {
      LOG_ERR << "Detected empty segments in trajectory[" << i << "]";
      continue;
    }

    const CandidateTrajectory::TrjNode* first_trj_node =
        &(trj.trajectory_segments[0]);
    if (first_trj_node->link_index < 0) {
      LOG_ERR << "Detected invalid link index in trajectory[" << i << "]";
      continue;
    }

    const GraphSection::GraphNode::Link* first_link =
        &(road_graph_link_storage_[first_trj_node->link_index].link);
    if (first_link->node_to.graph_section_index < 0) {
      LOG_ERR << "Detected invalid graph section "
                 "index in trajectory[" << i << "]";
      continue;
    }
    if (first_link->node_to.lane_section_index < 0) {
      LOG_ERR << "Detected invalid lane section "
                 "index in trajectory[" << i << "]";
      continue;
    }
    if (first_link->node_to.node_index < 0) {
      LOG_ERR << "Detected invalid node "
                 "index in trajectory[" << i << "]";
      continue;
    }
    const GraphSection& dest_graph_sec =
        road_graph_[first_link->node_to.graph_section_index];
    const GraphSection::LaneSection& dest_lane_sec =
        dest_graph_sec.lane_sections[first_link->node_to.lane_section_index];
    const GraphSection::GraphNode& dest_node =
        dest_lane_sec.nodes[first_link->node_to.node_index];

    if (trj.cost.total_cost < min_cost_global) {
      min_cost_global = trj.cost.total_cost;
      min_cost_trj_idx_global = i;
    }

    if (0 == dest_lane_sec.lat_major) {
      // In current lane
      if (trj.cost.total_cost < min_cost_curr_lane) {
        min_cost_curr_lane = trj.cost.total_cost;
        min_cost_trj_idx_curr_lane = i;
      }

      /* k001 pengc 2022-04-03 (begin) */
      // 避免避让的轨迹左右频繁跳动
      if (dest_node.lat_minor >= 0) {
        // left
        if (trj.cost.total_cost < min_cost_curr_lane_left) {
          min_cost_curr_lane_left = trj.cost.total_cost;
          min_cost_trj_idx_curr_lane_left = i;
        }
      } else {
        // right
        if (trj.cost.total_cost < min_cost_curr_lane_right) {
          min_cost_curr_lane_right = trj.cost.total_cost;
          min_cost_trj_idx_curr_lane_right = i;
        }
      }
      /* k001 pengc 2022-04-03 (end) */

      if (0 == dest_node.lat_minor) {
        // On centre line of current lane
        if (trj.cost.total_cost < min_cost_lka) {
          min_cost_lka = trj.cost.total_cost;
          min_cost_trj_idx_lka = i;
        }
      }

      /* k001 pengc 2022-04-10 (begin) */
      // 在一定的时间范围内，尽量保持之前的避让趋势
      Int32_t trj_cost_bypassing =
          trj.cost.total_cost + trj.cost.ext_bypassing_cost;
      if (trj_cost_bypassing < min_cost_curr_lane_bypassing) {
        min_cost_curr_lane_bypassing = trj_cost_bypassing;
        min_cost_trj_idx_curr_lane_bypassing = i;
      }
      /* k001 pengc 2022-04-10 (end) */
    }

    if (1 == dest_lane_sec.lat_major) {
      // In left lane
      if (0 == dest_node.lat_minor) {
        // On centre line of left lane
        if (trj.cost.total_cost < min_cost_left_lane) {
          min_cost_left_lane = trj.cost.total_cost;
          min_cost_trj_idx_left_lane = i;
        }
      }
    }

    if (-1 == dest_lane_sec.lat_major) {
      // In right lane
      if (0 == dest_node.lat_minor) {
        // On centre line of right lane
        if (trj.cost.total_cost < min_cost_right_lane) {
          min_cost_right_lane = trj.cost.total_cost;
          min_cost_trj_idx_right_lane = i;
        }
      }
    }
  }

  optimal_trj_index_.trj_idx_global = min_cost_trj_idx_global;

  /* k001 pengc 2022-04-03 (begin) */
  // 避免避让的轨迹左右频繁跳动
  optimal_trj_index_.trj_idx_curr_lane_left = min_cost_trj_idx_curr_lane_left;
  optimal_trj_index_.trj_idx_curr_lane_right = min_cost_trj_idx_curr_lane_right;
  /* k001 pengc 2022-04-03 (end) */
  optimal_trj_index_.trj_idx_curr_lane = min_cost_trj_idx_curr_lane;
  optimal_trj_index_.trj_idx_lka = min_cost_trj_idx_lka;
  /* k001 pengc 2022-04-10 (begin) */
  // 在一定的时间范围内，尽量保持之前的避让趋势
  optimal_trj_index_.trj_idx_curr_lane_bypassing = min_cost_trj_idx_curr_lane_bypassing;
  /* k001 pengc 2022-04-10 (end) */

  optimal_trj_index_.trj_idx_left_lane = min_cost_trj_idx_left_lane;
  optimal_trj_index_.trj_idx_right_lane = min_cost_trj_idx_right_lane;


//#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
//  std::cout << "The trajectory index which has min cost "
//               "in candiate trajectory list is "
//            << min_cost_trj_index_global
//            << std::endl;
//  std::cout << "And the min cost is " << min_cost_global << std::endl;
//#if 0
//  std::cout << "The sample points of optimal trajectory are:" << std::endl;
//  for (Int32_t i = 0; i < optimal_trajectory_sample_points_num; ++i) {
//    const common::PathPoint& path_point = optimal_trajectory_sample_points_[i];
//    std::cout << "[" << i << "], \t"
//              << "point[" << path_point.point.x()
//              << ", " << path_point.point.y()
//              << "], heading=" << path_point.heading
//              << ", curvature=" << path_point.curvature
//              << ", s=" << path_point.s
//              << std::endl;
//  }
//#endif
//#endif

  return true;
}

bool TrajectoryPlanningStateLattice::CalcTargetTrajectory() {

  /**
   * 计算输出给控制模块的规划后的轨迹
   *
   *   1、除了安全情形外(安全是第一位的，例如紧急避让等情形)，
   *      轨迹规划的结果应当符合行为规划的请求(例如，行为规划不请求变道，
   *      那么轨迹规划就不应该输出变道的轨迹)，但是若行为规划的请求存在
   *      安全问题，应当拒绝行为规划的请求(例如，行为规划请求变道，但变道
   *      存在危险，则应当拒绝此请求)
   *
   *   2、可以输出建议的行为(变道、避让等)反馈给行为规划，提高整个系统的灵活性。
   *
   *   3、应当保证规划的轨迹在时间轴上也应当是连续的。
   */

  Int32_t trj_req_type = TRJ_REQ_LKA;
  Int32_t candidate_trj_index = SelectTargetTrajectory(&trj_req_type);

  if (candidate_trj_index < 0) {
    LOG_ERR << "Invalid candiate trajectory index in candidate list.";
    return false;
  }

  if (!SampleCandidateTrajectory(candidate_trj_index,
                                 &optimal_trajectory_sample_points_)) {
    LOG_ERR << "Failed to sample candidate trajectory.";
    return false;
  }

  if (!SmoothTargetTrajectory(candidate_trj_index)) {
    path_filter_.Clear();
    LOG_ERR << "Failed to smooth target trajectory.";
    return false;
  }

  // 更新内部状态
  UpdateInternalStatus(candidate_trj_index, trj_req_type);

  // 主动请求变道
  ReqChangingLaneAutomatically(candidate_trj_index);

  // 设置状态量到规划结果中
  result_of_planning_.road_type = road_builed_type_;
  result_of_planning_.trj_status = status_.trj_status;
  // 在路网无效或者无车道线时，保持方向盘不动
  if ((ROAD_BUILED_TYPE_INVALID == road_builed_type_) ||
      (ROAD_BUILED_TYPE_PRED_PATH == road_builed_type_)) {
    result_of_planning_.hold_steering_wheel = true;
  } else {
    result_of_planning_.hold_steering_wheel = false;
  }
  // 对变道请求的响应
  result_of_planning_.changing_lane_rsp = action_manager_.changing_lane_rsp;

#if 1
  // 变道过程中, 限制方向盘转速
  if (TRJ_STATUS_LKA != status_.trj_status) {
    result_of_planning_.steering_wheel_speed =
        common::Min(result_of_planning_.steering_wheel_speed,
                    common::com_deg2rad(90.0F));
  }
#endif

  /* k001 pengc 2022-04-13 (begin) */
  // 优化变道时速度控制的问题
  // std::cout << "status_.trj_status=" << status_.trj_status << std::endl;
  // result_of_planning_.trj_changing.is_changing =
  //     ((1 == lat_moving_flag) || (2 == lat_moving_flag));

  if ((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == status_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == status_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II == status_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == status_.trj_status)) {
    result_of_planning_.trj_changing.is_changing = true;
  }

  // 期望的目标轨迹
  result_of_planning_.trj_changing.selected_trajectory =
      optimal_trajectory_sample_points_;

  // 返回原车道的轨迹
  if ((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == status_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == status_.trj_status)) {
    // 设置返回轨迹在当前车道中
    Int32_t trj_idx = optimal_trj_index_.trj_idx_curr_lane;
    if ((0 <= trj_idx) && (trj_idx < candidate_trj_list_.Size())) {
      if (!SampleCandidateTrajectory(
            trj_idx, &result_of_planning_.trj_changing.return_trajectory)) {
        result_of_planning_.trj_changing.return_trajectory.Clear();
      }
    }
  } else if ((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == status_.trj_status) ||
             (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II == status_.trj_status)) {
    // 设置返回轨迹在右车道中
    Int32_t trj_idx = optimal_trj_index_.trj_idx_right_lane;
    if ((0 <= trj_idx) && (trj_idx < candidate_trj_list_.Size())) {
      if (!SampleCandidateTrajectory(
            trj_idx, &result_of_planning_.trj_changing.return_trajectory)) {
        result_of_planning_.trj_changing.return_trajectory.Clear();
      }
    }
  } else if ((TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == status_.trj_status) ||
             (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == status_.trj_status)) {
    // 设置返回轨迹在左车道中
    Int32_t trj_idx = optimal_trj_index_.trj_idx_left_lane;
    if ((0 <= trj_idx) && (trj_idx < candidate_trj_list_.Size())) {
      if (!SampleCandidateTrajectory(
            trj_idx, &result_of_planning_.trj_changing.return_trajectory)) {
        result_of_planning_.trj_changing.return_trajectory.Clear();
      }
    }
  } else {
    // nothing to do
  }
  /* k001 pengc 2022-04-13 (end) */

  return true;
}

void TrajectoryPlanningStateLattice::SetPathSmootherParam() {
  path_filter_.path_trend_mgr_lat.SetLaziness(param_.path_filter_laziness_lat);
  path_filter_.path_trend_mgr_lat.SetMaxTrend(param_.path_filter_max_trend_lat);
  path_filter_.path_trend_mgr_lat.SetTrendVelocity(
        param_.path_filter_trend_move_velocity_lat,
        param_.path_filter_trend_stop_velocity_lat);
  path_filter_.path_trend_mgr_lat.SetLeadingPointMoveMaxVelocity(
        param_.path_filter_leading_move_max_velocity_lat);
  path_filter_.path_trend_mgr_lat.SetLeadingPointMoveAcceleration(
        param_.path_filter_leading_move_accel_lat,
        param_.path_filter_leading_stop_accel_lat);

  path_filter_.path_trend_mgr_lon.SetLaziness(param_.path_filter_laziness_lon);
  path_filter_.path_trend_mgr_lon.SetMaxTrend(param_.path_filter_max_trend_lon);
  path_filter_.path_trend_mgr_lon.SetTrendVelocity(
        param_.path_filter_trend_move_velocity_lon,
        param_.path_filter_trend_stop_velocity_lon);
  path_filter_.path_trend_mgr_lon.SetLeadingPointMoveMaxVelocity(
        param_.path_filter_leading_move_max_velocity_lon);
  path_filter_.path_trend_mgr_lon.SetLeadingPointMoveAcceleration(
        param_.path_filter_leading_move_accel_lon,
        param_.path_filter_leading_stop_accel_lon);
}


bool TrajectoryPlanningStateLattice::SmoothTargetTrajectory(
    Int32_t candidate_trj_idx) {
  if ((candidate_trj_idx < 0) ||
      (candidate_trj_idx >= candidate_trj_list_.Size())) {
    LOG_ERR << "Invalid candidate trajectory index.";
    return false;
  }
  // 获取最优候选轨迹
  const CandidateTrajectory& candidate_trj =
      candidate_trj_list_[candidate_trj_idx];
  if (candidate_trj.trajectory_segments.Size() < 1) {
    LOG_ERR << "Invalid candidate trajectory, because its size is less then 1.";
    return false;
  }

  // 获取最优轨迹第一个节点
  const CandidateTrajectory::TrjNode& first_candidate_trj_node =
      candidate_trj.trajectory_segments[0];
  if (first_candidate_trj_node.link_index < 0) {
    LOG_ERR << "Invalid candiate trajectory, "
               "because it contain invalid link index.";
    return false;
  }
  // 获取最优轨迹第一节点的link
  const GraphSection::GraphNode::Link& first_candidate_trj_link =
      road_graph_link_storage_[first_candidate_trj_node.link_index].link;
  if (first_candidate_trj_link.node_to.graph_section_index < 0) {
    LOG_ERR << "Detected invalid graph section index of candidate trajectory.";
    return false;;
  }
  if (first_candidate_trj_link.node_to.lane_section_index < 0) {
    LOG_ERR << "Detected invalid lane section index of candidate trajectory.";
    return false;;
  }
  if (first_candidate_trj_link.node_to.node_index < 0) {
    LOG_ERR << "Detected invalid node index of candidate trajectory.";
    return false;;
  }
  if (!driving_map_->IsValidReferenceLineIndex(
        first_candidate_trj_link.ref_line_index)) {
    LOG_ERR << "Invalid candiate trajectory, "
               "because it contain invalid reference line index.";
    return false;
  }
  // 获取最优轨迹第一个link所在的参考线点集
  const common::Path& candidate_trj_ref_line =
      driving_map_->GetSmoothReferenceLine(
        first_candidate_trj_link.ref_line_index);

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        lat_err_.filter[0].lat_err_list[0].timestamp, curr_timestamp_);
  plan_var_t time_elapsed_ms = time_elapsed * 0.001F;

#if 0
  const GraphSection& dest_graph_sec =
      road_graph_[first_candidate_trj_link.node_to.graph_section_index];
  const GraphSection::LaneSection& dest_lane_sec =
      dest_graph_sec.lane_sections[first_candidate_trj_link.node_to.lane_section_index];
  const GraphSection::GraphNode& dest_node =
      dest_lane_sec.nodes[first_candidate_trj_link.node_to.node_index];
  std::cout << "###### Optimal trajectory ################" << std::endl;
  std::cout << "lon_major=" << dest_graph_sec.lon_major
            << ", lon_minor=" << dest_graph_sec.lon_minor
            << ", lat_major=" << dest_lane_sec.lat_major
            << ", lat_minor=" << dest_node.lat_minor
            << ", lat_offset=" << dest_node.lat_offset
            << std::endl;
  std::cout << "##########################################" << std::endl;
#endif

  /// TODO: Consider using only leading point to smooth trajectory, not using
  /// previous reference line,
  /// and smoothing heading and curvature of leading point.
  if (!path_filter_.valid) {
    path_filter_.valid = true;
    path_filter_.timestamp = curr_timestamp_;
    path_filter_.path_trend_mgr_lat.Reset();
    path_filter_.path_trend_mgr_lon.Reset();

    path_filter_.prev_veh_pos = curr_position_.path_point;
    common::PathPoint path_point_on_candiate_trj_ref;
    candidate_trj_ref_line.FindSmoothPoint(first_candidate_trj_link.node_to.s,
                                           &path_point_on_candiate_trj_ref);
    plan_var_t leading_l = first_candidate_trj_link.node_to.l;
    path_filter_.prev_leading_point.point.set_x(
          path_point_on_candiate_trj_ref.point.x() -
          leading_l * common::com_sin(path_point_on_candiate_trj_ref.heading));
    path_filter_.prev_leading_point.point.set_y(
          path_point_on_candiate_trj_ref.point.y() +
          leading_l * common::com_cos(path_point_on_candiate_trj_ref.heading));
    path_filter_.prev_leading_point.heading =
        path_point_on_candiate_trj_ref.heading;
    plan_var_t tmp =
        1.0F - path_point_on_candiate_trj_ref.curvature * leading_l;
    if (common::com_abs(tmp) > 0.0001F) {
      path_filter_.prev_leading_point.curvature =
          path_point_on_candiate_trj_ref.curvature / tmp;
    } else {
      path_filter_.prev_leading_point.curvature =
          path_point_on_candiate_trj_ref.curvature;
    }

    common::PathPoint path_point_on_curr_ref;
    major_ref_line_->FindProjection(path_filter_.prev_leading_point.point,
                                  &path_point_on_curr_ref);
    if (path_point_on_curr_ref.s < 0.0F) {
      path_point_on_curr_ref.s = 0.0F;
    }
    if (path_point_on_curr_ref.s > major_ref_line_->total_length()) {
      path_point_on_curr_ref.s = major_ref_line_->total_length();
    }

    path_filter_.prev_leading_point.l = path_point_on_curr_ref.l;
    path_filter_.prev_leading_point.s =
        path_point_on_curr_ref.s - veh_proj_on_major_ref_line_.s;

    const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        points_in_curr_ref_line = major_ref_line_->points();
    path_filter_.prev_ref_line_sample_points.Clear();
    for (Int32_t i = 0; i < points_in_curr_ref_line.Size(); ++i) {
      path_filter_.prev_ref_line_sample_points.PushBack(
            points_in_curr_ref_line[i]);
    }
    path_filter_.prev_ref_line = *major_ref_line_;

    // std::cout<< "Init path filter: leading_l=" << leading_l
    //          << ", path_filter_.prev_leading_point.l=" << path_filter_.prev_leading_point.l
    //          << std::endl;

    result_of_planning_.Clear();
    result_of_planning_.msg_head = driving_map_->GetRelativePosList().msg_head;
    result_of_planning_.msg_head.timestamp = curr_timestamp_;
  }

  if ((path_filter_.prev_ref_line_sample_points.Size() < 2) ||
      (path_filter_.prev_ref_line.points().Size() < 2)) {
    LOG_ERR << "Invalid previous reference line, "
               "because the number of points is too small.";
    return false;
  }

#if 0
  common::Vec2d unit_direction(
        common::com_cos(curr_position_.path_point.heading),
        common::com_sin(curr_position_.path_point.heading));
  common::Vec2d veh_head_point =
      curr_position_.path_point.point +
      common::Max(1.0F, 1.5F * curr_position_.v) * unit_direction;
#else
  plan_var_t curr_pos[3] = { 0.0F };
  plan_var_t next_pos[3] = { 0.0F };
  curr_pos[0] = curr_position_.path_point.point.x();
  curr_pos[1] = curr_position_.path_point.point.y();
  curr_pos[2] = curr_position_.path_point.heading;
  vehicle_model_.EstimateNextPos(
        common::Max(30.0F/3.6F, curr_position_.v), curr_position_.yaw_rate, 1.0F, curr_pos, next_pos);
  common::Vec2d veh_head_point(next_pos[0], next_pos[1]);
#endif

  common::PathPoint veh_head_proj_on_prev_ref;
  common::PathPoint cur_proj_on_prev_ref;
  common::PathPoint tar_proj_on_prev_ref;
  if (!path_filter_.prev_ref_line.FindProjection(
        veh_head_point, &veh_head_proj_on_prev_ref)) {
    LOG_ERR << "Failed to find nearest point of vehicle head position "
               "on previous reference line.";
    return false;
  }
  if (!path_filter_.prev_ref_line.FindProjection(
        curr_position_.path_point.point, &cur_proj_on_prev_ref)) {
    LOG_ERR << "Failed to find nearest point of current position "
               "on previous reference line.";
    return false;
  }
  common::Vec2d dest_point = candidate_trj_ref_line.SLToXY(
        first_candidate_trj_link.node_to.s, first_candidate_trj_link.node_to.l);
  if (!path_filter_.prev_ref_line.FindProjection(
        dest_point, &tar_proj_on_prev_ref)) {
    LOG_ERR << "Failed to find nearest point of destination point "
               "on previous reference line.";
    return false;
  }

  common::PathPoint tar_proj_on_curr_ref;
  if (!major_ref_line_->FindProjection(dest_point, &tar_proj_on_curr_ref)) {
    LOG_ERR << "Failed to find nearest point of destination point "
               "on current reference line.";
    return false;
  }

  if (cur_proj_on_prev_ref.s < 0.0F) {
    cur_proj_on_prev_ref.s = 0.0F;
  }
  if (cur_proj_on_prev_ref.s > path_filter_.prev_ref_line.total_length()) {
    cur_proj_on_prev_ref.s = path_filter_.prev_ref_line.total_length();
  }
  if (tar_proj_on_prev_ref.s < 0.0F) {
    tar_proj_on_prev_ref.s = 0.0F;
  }
  if (tar_proj_on_prev_ref.s > path_filter_.prev_ref_line.total_length()) {
    tar_proj_on_prev_ref.s = path_filter_.prev_ref_line.total_length();
  }

  plan_var_t ref_line_offset_1 =
      veh_proj_on_major_ref_line_.l - cur_proj_on_prev_ref.l;
  plan_var_t ref_line_offset_2 =
      tar_proj_on_curr_ref.l - tar_proj_on_prev_ref.l;

  // 当前目标轨迹与之前行驶轨迹之间的横向偏差(锚点位置)
  plan_var_t l_diff =
      tar_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
  plan_var_t s_diff =
      tar_proj_on_prev_ref.s - cur_proj_on_prev_ref.s -
      path_filter_.prev_leading_point.s;
  plan_var_t l_inc = 0;
  plan_var_t s_inc = 0;

#if 0
  printf("@@@ tar_proj_on_prev_ref.l=%0.2f, "
         "path_filter_.prev_leading_point.l=%0.2f\n",
         tar_proj_on_prev_ref.l, path_filter_.prev_leading_point.l);
  printf("@@@ 1: v=%0.1f, l_diff=%0.2f\n", curr_position_.v*3.6F, l_diff);
#endif

  plan_var_t prev_veh_pos[3] = { 0.0F };
  plan_var_t pred_veh_pos[3] = { 0.0F };
  prev_veh_pos[0] = path_filter_.prev_veh_pos.point.x();
  prev_veh_pos[1] = path_filter_.prev_veh_pos.point.y();
  prev_veh_pos[2] = path_filter_.prev_veh_pos.heading;
  vehicle_model_.EstimateNextPos(
        curr_position_.v, curr_position_.yaw_rate,
        time_elapsed_ms, prev_veh_pos, pred_veh_pos);
  common::Vec2d pred_veh_point;
  pred_veh_point.set_x(pred_veh_pos[0]);
  pred_veh_point.set_y(pred_veh_pos[1]);
  common::PathPoint pred_veh_path_point;
  if (!path_filter_.prev_ref_line.FindProjection(
        pred_veh_point, &pred_veh_path_point)) {
    LOG_ERR << "Failed to find nearest point of predicted vehicle point "
               "on previous reference line.";
    return false;
  }
  plan_var_t pred_lat_offset =
      pred_veh_path_point.l - path_filter_.prev_veh_pos.l;

#if 0
  printf("curr_pos(%0.2f, %0.2f, %0.2f),"
         "prev_veh_pos(%0.2f, %0.2f, %0.2f), pred_veh_pos(%0.2f, %0.2f, %0.2f), "
         "curr_position_.v=%0.2f, curr_position_.yaw_rate=%0.2f, time_elapsed_ms=%0.3f\n",
         curr_position_.path_point.point.x(), curr_position_.path_point.point.y(), common::com_rad2deg(curr_position_.path_point.heading),
         prev_veh_pos[0], prev_veh_pos[1], common::com_rad2deg(prev_veh_pos[2]),
         pred_veh_pos[0], pred_veh_pos[1], common::com_rad2deg(pred_veh_pos[2]),
         curr_position_.v*3.6F, common::com_rad2deg(curr_position_.yaw_rate), time_elapsed_ms);
  printf("@@@ veh_proj_on_major_ref_line_.l=%0.3f, prev_veh_pos.l=%0.3f, "
         "cur_proj_on_prev_ref.l=%0.3f, pred_veh_path_point.l=%0.3f, "
         "pred_lat_offset_1=%0.3f, pred_lat_offset_2=%0.3f\n",
         veh_proj_on_major_ref_line_.l, path_filter_.prev_veh_pos.l,
         cur_proj_on_prev_ref.l, pred_veh_path_point.l,
         cur_proj_on_prev_ref.l - path_filter_.prev_veh_pos.l, pred_lat_offset);
#endif

  plan_var_t l_ref_offset = 0.0F;
  if (common::com_abs(ref_line_offset_1) < 0.2F &&
      common::com_abs(ref_line_offset_2) < 0.3F) {
    l_ref_offset = ref_line_offset_2;
  }
  l_diff += l_ref_offset;

  // printf("@@@ 2: l_diff=%0.2f, l_ref_offset=%0.2f\n", l_diff, l_ref_offset);

  SetPathSmootherParam();

  Int32_t lat_moving_flag = 0;
  if (l_diff > 0.2F) {  /// 0.01: 20210926
    // 请求向左变更轨迹
    lat_moving_flag = 1;
    // 增加向左的移动请求的强烈程度
    path_filter_.path_trend_mgr_lat.AddPositive();
    // 计算当前轨迹需要移动的横向距离
    l_inc = path_filter_.path_trend_mgr_lat.CalcMoveDistance(l_diff);
  } else if (l_diff < -0.2F) {  /// -0.01: 20210926
    // 请求向右变更轨迹
    lat_moving_flag = 2;
    // 允许向右变更轨迹
    // 增加向右的移动请求的强烈程度
    path_filter_.path_trend_mgr_lat.AddNegative();
    // 计算当前轨迹需要移动的横向距离
    l_inc = path_filter_.path_trend_mgr_lat.CalcMoveDistance(l_diff);
  } else {
    lat_moving_flag = 0;
    // 当前轨迹不需要向任何方向横向移动
    path_filter_.path_trend_mgr_lat.AddLaziness();
    l_inc = l_diff;
  }

  // printf("@@@ 3: lat_moving_flag=%d, l_inc=%0.3f\n", lat_moving_flag, l_inc);

  if (s_diff > 0.05F) {
    path_filter_.path_trend_mgr_lon.AddPositive();
    s_inc = path_filter_.path_trend_mgr_lon.CalcMoveDistance(s_diff);
  } else if (s_diff < -0.05F) {
    path_filter_.path_trend_mgr_lon.AddNegative();
    s_inc = path_filter_.path_trend_mgr_lon.CalcMoveDistance(s_diff);
  } else {
    path_filter_.path_trend_mgr_lon.AddLaziness();
    s_inc = s_diff;
  }

  plan_var_t additional_l_inc = 0.0F;
#if 1
  if ((1 == lat_moving_flag) &&
      (path_filter_.path_trend_mgr_lat.GetPositiveTrend() > 0.0F)) {
    plan_var_t tar_l_move =
        tar_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
#if 0
    plan_var_t mid_l_move =
        cur_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
    if ((tar_l_move > 0.1F) && (mid_l_move > 0.0F)) {
      additional_l_inc = common::Min(tar_l_move, mid_l_move);
    }
#else
    plan_var_t low_l_move =
        veh_head_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
    if ((tar_l_move > 0.1F) && (low_l_move > 0.0F)) {
      additional_l_inc = common::Min(tar_l_move, low_l_move);
    }
#endif

#if 0
    std::cout << "    >>> 向左移动, tar_l_move=" << tar_l_move
              << ", low_l_move=" << low_l_move
              << ", additional_l_inc=" << additional_l_inc
              << std::endl;
#endif
  } else if ((2 == lat_moving_flag) &&
             (path_filter_.path_trend_mgr_lat.GetNegativeTrend() > 0.0F)) {
    plan_var_t tar_l_move =
        tar_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
#if 0
    plan_var_t mid_l_move =
        cur_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
    if ((tar_l_move < -0.1F) && (mid_l_move < 0.0F)) {
      additional_l_inc = common::Max(tar_l_move, mid_l_move);
    }
#else
    plan_var_t low_l_move =
        veh_head_proj_on_prev_ref.l - path_filter_.prev_leading_point.l;
    if ((tar_l_move < -0.1F) && (low_l_move < 0.0F)) {
      additional_l_inc = common::Max(tar_l_move, low_l_move);
    }
#endif

#if 0
    std::cout << "    >>> 向右移动, tar_l_move=" << tar_l_move
              << ", low_l_move=" << low_l_move
              << ", additional_l_inc=" << additional_l_inc
              << std::endl;
#endif
  } else {
    additional_l_inc = 0.0F;
  }
#endif

  plan_var_t leading_point_l =
      path_filter_.prev_leading_point.l + l_inc - l_ref_offset + additional_l_inc;
  plan_var_t leading_point_s =
      path_filter_.prev_leading_point.s + s_inc + cur_proj_on_prev_ref.s;
#if 1
  // 限制变道的速率
  plan_var_t lat_diff_curr_leading = leading_point_l - cur_proj_on_prev_ref.l;
  // plan_var_t lat_diff_curr_head = veh_head_proj_on_prev_ref.l - cur_proj_on_prev_ref.l;
  plan_var_t max_leading_lat_offset = 2.0F;
  plan_var_t t = 0;
  Int32_t lower = common::LerpInOrderedTable(
        param_.max_leading_lat_offset_limit_table_by_speed,
        curr_position_.v, &t);
  max_leading_lat_offset = common::Lerp(
        param_.max_leading_lat_offset_limit_table_by_speed[
        lower].value,
        param_.max_leading_lat_offset_limit_table_by_speed[
        lower+1].value, t);

  if (lat_diff_curr_leading > max_leading_lat_offset) {
    leading_point_l = cur_proj_on_prev_ref.l + max_leading_lat_offset;
    leading_point_s = path_filter_.prev_leading_point.s + cur_proj_on_prev_ref.s;
  } else if (lat_diff_curr_leading < -max_leading_lat_offset) {
    leading_point_l = cur_proj_on_prev_ref.l - max_leading_lat_offset;
    leading_point_s = path_filter_.prev_leading_point.s + cur_proj_on_prev_ref.s;
  } else {
    // nothing to do
  }
  //if ((leading_point_l > 0.0F) && (lat_diff_curr_head > 0.0F)) {
  //} else if ((leading_point_l < 0.0F) && (lat_diff_curr_head < 0.0F)) {
  //} else {
  //  // nothing to do
  //}
#endif
  const common::Vec2d leading_point =
      path_filter_.prev_ref_line.SLToXY(leading_point_s, leading_point_l);

#if 0
  std::cout << "cur_proj_on_prev_ref.l=" << cur_proj_on_prev_ref.l
            << ", tar_proj_on_prev_ref.l=" << tar_proj_on_prev_ref.l
            << ", veh_proj_on_major_ref_line_.l=" << veh_proj_on_major_ref_line_.l
            << ", ref_line_offset_1=" << ref_line_offset_1
            << ", ref_line_offset_2=" << ref_line_offset_2
            << ", l_diff=" << l_diff
            << ", prev_leading_point.l=" << path_filter_.prev_leading_point.l
            << ", l_inc=" << l_inc
            << ", l_ref_offset=" << l_ref_offset
            << ", additional_l_inc=" << additional_l_inc
            << ", leading_point_l=" << leading_point_l
            << std::endl;
#endif

  Int32_t dest_lane_index = -1;
  common::PathPoint leading_point_on_lane;
  if (!driving_map_->FindNearestLaneSynthetically(
        leading_point, tar_proj_on_prev_ref.heading,
        &dest_lane_index, &leading_point_on_lane)) {
    LOG_ERR << "Failed to find nearest lane of leading point.";
    return false;
  }
  if (!driving_map_->IsValidLaneIndex(dest_lane_index)) {
    LOG_ERR << "Failed to find nearest lane of leading point, "
               "because the lane index is invalid.";
    return false;
  }

#if 0
  LOG_INFO(5) << "In trajectory planning, the dest lane is (idx="
              << dest_lane_index << ", id="
              << driving_map_->GetLaneIdByIndex(dest_lane_index).id
              << ").";
#endif

  Int32_t dest_ref_line_index = driving_map_->FindReferenceLine(
        curr_position_.path_point.point,
        road_graph_[first_candidate_trj_node.graph_section_index].lane_index_on_ref,
        dest_lane_index);
  if (!driving_map_->IsValidReferenceLineIndex(dest_ref_line_index)) {
    LOG_ERR << "Failed to find reference line in the direction of "
               "from current point to destination point.";
    return false;
  }

  /* k001 pengc 2022-02-19 (begin) */
  // 优先使用主参考线
  common::PathPoint leading_point_on_major_ref;
  if (!major_ref_line_->FindNearest(leading_point, &leading_point_on_major_ref)) {
    LOG_ERR << "Failed to find nearest point on major reference line.";
    return false;
  }
  if (common::com_abs(leading_point_on_major_ref.l - leading_point_on_lane.l) < 0.5F) {
    dest_ref_line_index = major_ref_line_index_;
  }
  /* k001 pengc 2022-02-19 (end) */

#if 0
  std::cout << "))))) start_lane=" << driving_map_->GetLaneIdByIndex(
               road_graph_[first_candidate_trj_node.graph_section_index].lane_index_on_ref).id
            << std::endl;
  std::cout << "))))) end_lane=" << driving_map_->GetLaneIdByIndex(dest_lane_index).id << std::endl;
  std::cout << "))))) dest_ref_line_index=" << dest_ref_line_index << std::endl;
#endif

  if (!CreateTrajectoryByLeadingPoint(
        dest_ref_line_index, leading_point, &target_trajectory_link_list_)) {
    LOG_ERR << "Failed to create target trajectory by leading point.";
    return false;
  }

  if (!SampleTrajectory(target_trajectory_link_list_,
                        &result_of_planning_.target_trajectory)) {
    LOG_ERR << "Failed to sample target trajectory.";
    return false;
  }

  common::PathPoint leading_proj_on_curr_ref;
  major_ref_line_->FindProjection(leading_point, &leading_proj_on_curr_ref);
  if (leading_proj_on_curr_ref.s < 0.0F) {
    leading_proj_on_curr_ref.s = 0.0F;
  }
  if (leading_proj_on_curr_ref.s > major_ref_line_->total_length()) {
    leading_proj_on_curr_ref.s = major_ref_line_->total_length();
  }

  leading_point_l = leading_proj_on_curr_ref.l;
  leading_point_s = leading_proj_on_curr_ref.s - veh_proj_on_major_ref_line_.s;

#if 0
  common::PathPoint prev_leading_proj_on_curr_ref;
  major_ref_line_->FindProjection(
        path_filter_.prev_leading_point.point, &prev_leading_proj_on_curr_ref);
  std::cout << "prev_leading_proj_on_curr_ref.l="
            << prev_leading_proj_on_curr_ref.l
            << ", leading_proj_on_curr_ref.l="
            << leading_proj_on_curr_ref.l
            << ", diff="
            << leading_proj_on_curr_ref.l - prev_leading_proj_on_curr_ref.l
            << std::endl;
#endif

  /// TODO 右变道时，车辆变道右车道时，切换参考线后，最优轨迹会在车道中心线右边采样，形成一条偏离车
  /// 道中心的轨迹，后续又修正.
  /// 原因分析：当leading_point_l为0.02左右时，在当前参考线采样时，leading_proj_on_curr_ref.l
  /// 有-0.25大小，分析同一个参考线（上一帧和此帧）横向偏移距离差距过大，
  /// 发现是上一帧和此帧在远处的轨迹点有偏离.
#if 0
  /* k003 longjiaoy 2022-10-24 (start) */
  /* 变道时最优轨迹选偏离车道中心线采样 */
  if(TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == status_.trj_status) {
    if (leading_proj_on_curr_ref.l > -0.2F) {
      leading_point_l = leading_proj_on_curr_ref.l;
      leading_point_s = leading_proj_on_curr_ref.s - veh_proj_on_major_ref_line_.s;
    }
  } else if (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == status_.trj_status) {
    if (leading_proj_on_curr_ref.l < 0.2F) {
      leading_point_l = leading_proj_on_curr_ref.l;
      leading_point_s = leading_proj_on_curr_ref.s - veh_proj_on_major_ref_line_.s;
    }
  } else {
    leading_point_l = leading_proj_on_curr_ref.l;
    leading_point_s = leading_proj_on_curr_ref.s - veh_proj_on_major_ref_line_.s;
  }
  /* k003 longjiaoy 2022-10-24 (end) */
#endif

  // 计算横向误差
  CalcLatErr(leading_point_l, ref_line_offset_1, pred_lat_offset, 
      lat_moving_flag, max_leading_lat_offset, l_inc);

  result_of_planning_.msg_head = driving_map_->GetRelativePosList().msg_head;
  result_of_planning_.msg_head.timestamp = curr_timestamp_;

  result_of_planning_.curr_pos.x = curr_position_.path_point.point.x();
  result_of_planning_.curr_pos.y = curr_position_.path_point.point.y();
  result_of_planning_.curr_pos.heading = curr_position_.path_point.heading;
  result_of_planning_.curr_pos.curvature = curr_position_.path_point.curvature;
  result_of_planning_.curr_pos.s = 0.0F;
  result_of_planning_.curr_pos.l = veh_proj_on_major_ref_line_.l;
  result_of_planning_.leading_pos.x = leading_point.x();
  result_of_planning_.leading_pos.y = leading_point.y();
  result_of_planning_.leading_pos.heading =
      target_trajectory_link_list_.Front().node_to.heading;
  result_of_planning_.leading_pos.curvature =
      target_trajectory_link_list_.Front().node_to.curvature;
  result_of_planning_.leading_pos.s = leading_point_s;
  result_of_planning_.leading_pos.l = leading_point_l;

  /* k003 pengc 2022-12-12 (begin) */
  // 使用当前点在主参考线的投影点，预瞄点在主参考线上的投影点，
  // 以及它们在主参考线上的中位点，计算其外接圆曲率，使用这个曲率当作预瞄点的曲率
  common::Vec2d tri_corner[3];
  tri_corner[0] = veh_proj_on_major_ref_line_.point;
  tri_corner[1] =
      major_ref_line_->SLToXY(
        veh_proj_on_major_ref_line_.s+ 0.5F*leading_point_s, 0.0F);
  tri_corner[2] = leading_proj_on_curr_ref.point;
  result_of_planning_.leading_pos.curvature =
      common::CalcCurvatureByTriCorner(tri_corner[0], tri_corner[1], tri_corner[2]);
#if 0
  printf("### trj_status=%d\n", status_.trj_status);
  printf("### CalcCurvatureByTriCorner([%0.1f,%0.1f], [%0.1f,%0.1f], [%0.1f, %0.1f]) = %0.4f\n",
         tri_corner[0].x(), tri_corner[0].y(),
         tri_corner[1].x(), tri_corner[1].y(),
         tri_corner[2].x(), tri_corner[2].y(),
         result_of_planning_.leading_pos.curvature);
#endif
  /* k003 pengc 2022-12-12 (end) */

  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    result_of_planning_.trj_direction =
        TrajectoryPlanningResult::TRJ_DIRECTION_BACKWARD;
  } else {
    result_of_planning_.trj_direction =
        TrajectoryPlanningResult::TRJ_DIRECTION_FORWARD;
  }

  result_of_planning_.lat_err.moving_flag = lat_moving_flag;
  result_of_planning_.lat_err.samples[0].lat_err =
      lat_err_.lat_err_sample[0].lat_err_smooth;
  result_of_planning_.lat_err.samples[0].lat_err_chg_rate =
      lat_err_.lat_err_sample[0].lat_err_v_smooth;
  result_of_planning_.lat_err.samples[0].yaw_err =
      lat_err_.lat_err_sample[0].yaw_err_smooth;
  result_of_planning_.lat_err.samples[0].yaw_err_chg_rate =
      lat_err_.lat_err_sample[0].yaw_err_v_smooth;
  result_of_planning_.lat_err.samples[1].lat_err =
      lat_err_.lat_err_sample[1].lat_err_smooth;
  result_of_planning_.lat_err.samples[1].lat_err_chg_rate =
      lat_err_.lat_err_sample[1].lat_err_v_smooth;
  result_of_planning_.lat_err.samples[1].yaw_err =
      lat_err_.lat_err_sample[1].yaw_err_smooth;
  result_of_planning_.lat_err.samples[1].yaw_err_chg_rate =
      lat_err_.lat_err_sample[1].yaw_err_v_smooth;


  /* k001 pengc 2022-04-13 (begin) */
  // 优化变道时速度控制的问题
  result_of_planning_.trj_changing.is_changing =
      ((1 == lat_moving_flag) || (2 == lat_moving_flag));
  result_of_planning_.trj_changing.lat_offset =
      tar_proj_on_curr_ref.l - leading_proj_on_curr_ref.l;
  /* k001 pengc 2022-04-13 (end) */

  // std::cout << "******* result_of_planning_.trj_changing.is_changing="
  //           << result_of_planning_.trj_changing.is_changing
  //           << std::endl;
  // std::cout << "&&&&&&&  trj_status = " << status_.trj_status << std::endl;

  path_filter_.valid = true;
  path_filter_.timestamp = curr_timestamp_;
  path_filter_.prev_veh_pos = curr_position_.path_point;
  path_filter_.prev_leading_point.point = leading_point;
  path_filter_.prev_leading_point.heading =
      target_trajectory_link_list_.Front().node_to.heading;
  path_filter_.prev_leading_point.curvature =
      target_trajectory_link_list_.Front().node_to.curvature;
  path_filter_.prev_leading_point.s = leading_point_s;
  path_filter_.prev_leading_point.l = leading_point_l;

#if 0
  printf("@@@ Update path_filter_.prev_leading_point.l=%0.2f\n",
         path_filter_.prev_leading_point.l);
#endif

  const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
      points_in_curr_ref_line = major_ref_line_->points();
  path_filter_.prev_ref_line_sample_points.Clear();
  for (Int32_t i = 0; i < points_in_curr_ref_line.Size(); ++i) {
    path_filter_.prev_ref_line_sample_points.PushBack(
          points_in_curr_ref_line[i]);
  }

  return true;
}

void TrajectoryPlanningStateLattice::ConvertPrevRefLineToCurrCoorFrame() {
  if (!path_filter_.valid) {
    return;
  }

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        path_filter_.timestamp, curr_timestamp_);
  if (time_elapsed > 500) {
    path_filter_.valid = false;
    LOG_ERR << "The previous reference line is too old for path filter.";
    return;
  }

  Int32_t prev_rel_line_points_num =
      path_filter_.prev_ref_line_sample_points.Size();
  if (prev_rel_line_points_num < 2) {
    path_filter_.valid = false;
    LOG_ERR << "Invalid previous reference line, "
               "because the number of points is too small.";
    return;
  }

  const ad_msg::RelativePosList& rel_pos_list =
      driving_map_->GetRelativePosList();
  ad_msg::RelativePos rel_pos;
  common::Matrix<plan_var_t, 2, 1> point_conv;
  common::Matrix<plan_var_t, 3, 3> mat_conv;
  common::Matrix<plan_var_t, 2, 1> rotate_center;
  mat_conv.SetIdentity();
  rotate_center.SetZeros();
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
      path_filter_.timestamp, rel_pos_list, &rel_pos)) {
    path_filter_.valid = false;
    LOG_ERR << "Failed to find relative position for "
               "converting previous reference line to current coordinate.";
    return;
  }
  common::Rotate_2D<plan_var_t>(rotate_center, rel_pos.heading, &mat_conv);
  common::Translate_2D(rel_pos.x, rel_pos.y, &mat_conv);

  // Convert previous vehicle position to current coordinate system.
  common::PathPoint& prev_veh_pos = path_filter_.prev_veh_pos;
  point_conv(0) = prev_veh_pos.point.x();
  point_conv(1) = prev_veh_pos.point.y();
  common::TransformVert_2D(mat_conv, &point_conv);
  prev_veh_pos.point.set_x(point_conv(0));
  prev_veh_pos.point.set_y(point_conv(1));
  prev_veh_pos.heading =
      common::NormalizeAngle(prev_veh_pos.heading+rel_pos.heading);

  // Convert previous leading point to current coordinate system.
  common::PathPoint& prev_leading_point = path_filter_.prev_leading_point;
  point_conv(0) = prev_leading_point.point.x();
  point_conv(1) = prev_leading_point.point.y();
  common::TransformVert_2D(mat_conv, &point_conv);
  prev_leading_point.point.set_x(point_conv(0));
  prev_leading_point.point.set_y(point_conv(1));
  prev_leading_point.heading =
      common::NormalizeAngle(prev_leading_point.heading+rel_pos.heading);

  // Convert previous reference line to current coordinate system.
  for (Int32_t i = 0; i < prev_rel_line_points_num; ++i) {
    common::Vec2d& p = path_filter_.prev_ref_line_sample_points[i];

    point_conv(0) = p.x();
    point_conv(1) = p.y();
    common::TransformVert_2D(mat_conv, &point_conv);
    p.set_x(point_conv(0));
    p.set_y(point_conv(1));
  }
  if (!path_filter_.prev_ref_line.Construct(
        path_filter_.prev_ref_line_sample_points)) {
    path_filter_.valid = false;
    LOG_ERR << "Failed to construct path of previous reference line.";
    return;
  }

  prev_ref_line_sample_points_for_debug_.Clear();
  path_filter_.prev_ref_line.GetSamplePoints(
        &prev_ref_line_sample_points_for_debug_);
}

bool TrajectoryPlanningStateLattice::CreateTrajectoryByLeadingPoint(
    Int32_t ref_line_index, const common::Vec2d& leading_point,
    common::StaticVector<GraphSection::GraphNode::Link, MAX_GRAPH_SECTION_NUM>* trj_links) {
  if (!driving_map_->IsValidReferenceLineIndex(ref_line_index)) {
    LOG_ERR << "Invalid reference line index.";
    return false;
  }
  const common::Path& ref_line =
      driving_map_->GetSmoothReferenceLine(ref_line_index);

  for (Int32_t i = 0; i < trj_links->Size(); ++i) {
    (*trj_links)[i].Clear();
  }
  trj_links->Clear();

  common::PathPoint curr_proj_on_ref;
  common::PathPoint leading_proj_on_ref;
  if (!ref_line.FindProjection(curr_position_.path_point.point,
                               &curr_proj_on_ref)) {
    LOG_ERR << "Failed to find projection point of current position "
               "on reference line.";
    return false;
  }
  if (!ref_line.FindProjection(leading_point, &leading_proj_on_ref)) {
    LOG_ERR << "Failed to find projection point of leading position "
               "on reference line.";
    return false;
  }
  if (lon_graph_samples_step_len_.Empty()) {
    LOG_ERR << "Detected invalid road graph.";
    return false;
  }
  if (curr_proj_on_ref.s < 0.0F) {
    curr_proj_on_ref.s = 0.0F;
  }
  if (curr_proj_on_ref.s > ref_line.total_length()) {
    curr_proj_on_ref.s = ref_line.total_length();
  }
  if (leading_proj_on_ref.s < 0.0F) {
    leading_proj_on_ref.s = 0.0F;
  }
  if (leading_proj_on_ref.s > ref_line.total_length()) {
    leading_proj_on_ref.s = ref_line.total_length();
  }

  plan_var_t leading_point_l = leading_proj_on_ref.l;
  if (common::com_abs(leading_point_l) < 0.1F) {
    leading_point_l = 0.0F;
  }
  plan_var_t leading_point_s = leading_proj_on_ref.s - curr_proj_on_ref.s;
  if (leading_point_s < leading_length_for_lka_) {
    leading_point_s = leading_length_for_lka_;
  }
  if (leading_point_s > (ref_line.total_length()-0.2F)) {
    leading_point_s = ref_line.total_length()-0.2F;
  }

  GraphSection::GraphNode::Link* dest_link = trj_links->Allocate();
  if (Nullptr_t == dest_link) {
    LOG_ERR << "Failed to allocate space for destination link.";
    return false;
  }
  dest_link->ref_line_index = ref_line_index;
  dest_link->node_from.proj_on_ref = curr_proj_on_ref;
  dest_link->node_from.s = curr_proj_on_ref.s;
  dest_link->node_from.l = curr_proj_on_ref.l;
  dest_link->node_from.heading = curr_position_.path_point.heading;
  dest_link->node_from.dl = common::com_tan(
        common::AngleDiff(curr_proj_on_ref.heading,
                          dest_link->node_from.heading));
  dest_link->node_from.curvature = curr_position_.path_point.curvature;
  dest_link->node_from.ddl = common::com_tan(
        dest_link->node_from.curvature - curr_proj_on_ref.curvature);
#if 1
  plan_var_t dl_major_ref = common::com_tan(
        common::AngleDiff(veh_proj_on_major_ref_line_.heading,
                          dest_link->node_from.heading));
  plan_var_t ddl_major_ref = common::com_tan(
        dest_link->node_from.curvature - veh_proj_on_major_ref_line_.curvature);
  if (common::com_abs(dl_major_ref) < common::com_abs(dest_link->node_from.dl)) {
    dest_link->node_from.dl = dl_major_ref;
  }
  if (common::com_abs(ddl_major_ref) < common::com_abs(dest_link->node_from.ddl)) {
    dest_link->node_from.ddl = ddl_major_ref;
  }
#endif

  plan_var_t dest_s = curr_proj_on_ref.s;
  for (Int32_t i = -1; i < lon_graph_samples_step_len_.Size(); ++i) {
    plan_var_t prev_s = dest_s;
    if (-1 == i) {
      dest_s += leading_point_s;
      if (lon_graph_samples_step_len_[0] < (leading_point_s+5.0F)) {
        i++;
      }
    } else {
      dest_s += lon_graph_samples_step_len_[i];
    }
    if (dest_s > (ref_line.total_length()-0.2F)) {
      dest_s = ref_line.total_length()-0.2F;
    }

    if (dest_s < (prev_s+2.0F)) {
      continue;
    }

    common::PathPoint path_point;
    ref_line.FindSmoothPoint(dest_s, &path_point);

    GraphSection::GraphNode::Link& prev_link = trj_links->Back();
    prev_link.node_to.proj_on_ref = path_point;
    prev_link.node_to.s = dest_s;
    prev_link.node_to.l = leading_point_l;
    prev_link.node_to.heading = path_point.heading;
    plan_var_t tmp = 1.0F - path_point.curvature * leading_point_l;
    if (common::com_abs(tmp) > 0.0001F) {
      prev_link.node_to.curvature = path_point.curvature / tmp;
    } else {
      prev_link.node_to.curvature = path_point.curvature;
    }
    prev_link.node_to.dl = 0;
    prev_link.node_to.ddl = 0;

    // 构建link的曲线方程
    prev_link.curve.Construct(
          prev_link.node_from.l, prev_link.node_from.dl,
          prev_link.node_from.ddl,
          prev_link.node_to.l, prev_link.node_to.dl, prev_link.node_to.ddl,
          prev_link.node_to.s - prev_link.node_from.s);

    dest_link = trj_links->Allocate();
    if (Nullptr_t == dest_link) {
      LOG_ERR << "Failed to allocate space for destination link.";
      break;
    }
    dest_link->ref_line_index = ref_line_index;
    dest_link->node_from.proj_on_ref = path_point;
    dest_link->node_from.s = dest_s;
    dest_link->node_from.l = leading_point_l;
    dest_link->node_from.heading = path_point.heading;
    dest_link->node_from.curvature = prev_link.node_to.curvature;
    dest_link->node_from.dl = 0;
    dest_link->node_from.ddl = 0;

    if ((dest_s+2.0F) > ref_line.total_length()) {
      break;
    }
  }

  Int32_t tar_trj_seg_num = trj_links->Size()-1;
  if (tar_trj_seg_num < 1) {
    LOG_ERR << "Failed to create trajectory, because its links size is zero.";
    return false;
  }

  return true;
}

bool TrajectoryPlanningStateLattice::SampleTrajectory(
    const common::StaticVector<GraphSection::GraphNode::Link, MAX_GRAPH_SECTION_NUM> trj_links,
    common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>* samples) {
  Int32_t trj_seg_num = trj_links.Size()-1;
  if (trj_seg_num < 1) {
    LOG_ERR << "Invalid size of trajectory links.";
    return false;
  }
  for (Int32_t i = 0; i < trj_seg_num; ++i) {
    SampleLinkCurve(trj_links[i], 1.0F, samples);
    if ((trj_seg_num-1) != i) {
      samples->PopBack();
    }
  }

  if (samples->Size() < 2) {
    LOG_ERR << "Failed to sample trajectory, "
               "because its points size is too small.";
    return false;
  }

  return true;
}

#if 1
void TrajectoryPlanningStateLattice::CalcLatErr(
    plan_var_t leading_l, plan_var_t ref_offset, plan_var_t pred_lat_offset, 
    Int32_t lat_moving_flag, plan_var_t max_leading_lat_offset, plan_var_t l_inc) {
  plan_var_t lat_err = veh_proj_on_major_ref_line_.l - leading_l;
  plan_var_t yaw_err = common::AngleDiff(veh_proj_on_major_ref_line_.heading,
                                         curr_position_.path_point.heading);

  LatErr::LatErrFilter& filter = lat_err_.filter[0];

  if (!filter.lat_err_list[0].valid) {
    lat_err_.Clear();
    filter.lat_err_filter.expectation = lat_err;
    filter.lat_err_ref_filter.expectation = veh_proj_on_major_ref_line_.l;
    filter.yaw_err_filter.expectation = yaw_err;

    filter.lat_err_list[0].valid = true;
    filter.lat_err_list[0].timestamp = curr_timestamp_;
    filter.lat_err_list[0].lat_err = lat_err;
    filter.lat_err_list[0].lat_err_ref = veh_proj_on_major_ref_line_.l;
    filter.lat_err_list[0].lat_err_v = 0.0F;
    filter.lat_err_list[0].lat_err_a = 0.0F;
    filter.lat_err_list[0].yaw_err = yaw_err;
    filter.lat_err_list[0].yaw_err_v = 0.0F;
    filter.lat_err_list[0].yaw_err_v_raw = 0.0F;
    filter.lat_err_list[0].yaw_err_a = 0.0F;

    lat_err_.lat_err_sample[0].lat_err = lat_err;
    lat_err_.lat_err_sample[0].lat_err_smooth = lat_err;
    lat_err_.lat_err_sample[0].lat_err_v = 0.0F;
    lat_err_.lat_err_sample[0].lat_err_v_smooth = 0.0F;
    lat_err_.lat_err_sample[0].yaw_err = yaw_err;
    lat_err_.lat_err_sample[0].yaw_err_smooth = yaw_err;
    lat_err_.lat_err_sample[0].yaw_err_v = 0.0F;
    lat_err_.lat_err_sample[0].yaw_err_v_smooth = 0.0F;

    return;
  }

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        filter.lat_err_list[0].timestamp, curr_timestamp_);
  plan_var_t time_elapsed_ms = time_elapsed * 0.001F;

  if ((time_elapsed > 300) || (time_elapsed < 0)) {
    lat_err_.Clear();
    filter.lat_err_filter.expectation = lat_err;
    filter.lat_err_ref_filter.expectation = veh_proj_on_major_ref_line_.l;
    filter.yaw_err_filter.expectation = yaw_err;

    filter.lat_err_list[0].valid = true;
    filter.lat_err_list[0].timestamp = curr_timestamp_;
    filter.lat_err_list[0].lat_err = lat_err;
    filter.lat_err_list[0].lat_err_ref = veh_proj_on_major_ref_line_.l;
    filter.lat_err_list[0].lat_err_v = 0.0F;
    filter.lat_err_list[0].lat_err_a = 0.0F;
    filter.lat_err_list[0].yaw_err = yaw_err;
    filter.lat_err_list[0].yaw_err_v = 0.0F;
    filter.lat_err_list[0].yaw_err_v_raw = 0.0F;
    filter.lat_err_list[0].yaw_err_a = 0.0F;

    lat_err_.lat_err_sample[0].lat_err = lat_err;
    lat_err_.lat_err_sample[0].lat_err_smooth = lat_err;
    lat_err_.lat_err_sample[0].lat_err_v = 0.0F;
    lat_err_.lat_err_sample[0].lat_err_v_smooth = 0.0F;
    lat_err_.lat_err_sample[0].yaw_err = yaw_err;
    lat_err_.lat_err_sample[0].yaw_err_smooth = yaw_err;
    lat_err_.lat_err_sample[0].yaw_err_v = 0.0F;
    lat_err_.lat_err_sample[0].yaw_err_v_smooth = 0.0F;

    return;
  }

  // 判断参考线是否变更了, 0 ~ unchanged, 1 ~ changed to left, 2 ~ changed to right
  Int32_t ref_changed_flag = IsRefLineChanged();

  plan_var_t abs_ref_offset = common::com_abs(ref_offset);
  if ((0.2F < abs_ref_offset) && (abs_ref_offset < 2.0F)) {
    lat_err_.ref_changed.ref_offset = ref_offset;
    lat_err_.ref_changed.changed_flag = true;
    lat_err_.ref_changed.changed_count = 10;
  }
  if (lat_err_.ref_changed.changed_flag) {
    lat_err_.ref_changed.changed_count--;
    if (lat_err_.ref_changed.changed_count <= 0) {
      lat_err_.ref_changed.changed_count = 0;
      lat_err_.ref_changed.changed_flag = false;
    }
  }

  /// Smooth lateral error
  filter.lat_err_filter.expectation +=
      filter.lat_err_list[0].lat_err_v * time_elapsed_ms;
  plan_var_t covariance = filter.lat_err_filter.covariance + 0.3F;
  plan_var_t gain = covariance / (covariance + 0.8F);
  plan_var_t lat_err_smooth =
      filter.lat_err_filter.expectation +
      gain * (lat_err - filter.lat_err_filter.expectation);
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.lat_err_filter.expectation = lat_err_smooth;
  filter.lat_err_filter.covariance = covariance;

  /// lateral ref error
  plan_var_t lat_err_ref = veh_proj_on_major_ref_line_.l;
  plan_var_t lat_err_ref_smooth = lat_err_ref;
  if (1 == ref_changed_flag) {
    // 参考线向左切换了, 使用之前的误差变化率
    filter.lat_err_ref_filter.expectation = lat_err_ref;
    filter.lat_err_list[0].lat_err_ref = lat_err_ref;
  } else if (2 == ref_changed_flag) {
    // 参考线向右切换了, 使用之前的误差变化率
    filter.lat_err_ref_filter.expectation = lat_err_ref;
    filter.lat_err_list[0].lat_err_ref = lat_err_ref;
  } else {
    // 参考线未发生切换
    /// Smooth reference lateral error
    filter.lat_err_ref_filter.expectation +=
        filter.lat_err_list[0].lat_err_v * time_elapsed_ms;
    covariance = filter.lat_err_ref_filter.covariance + 0.3F;
    gain = covariance / (covariance + 0.8F);
    lat_err_ref_smooth =
        filter.lat_err_ref_filter.expectation +
        gain * (lat_err_ref - filter.lat_err_ref_filter.expectation);
    covariance = (1.0F - gain) * covariance;
    if (covariance < 1e-3F) {
      covariance = 1e-3F;
    }
    filter.lat_err_ref_filter.expectation = lat_err_ref_smooth;
    filter.lat_err_ref_filter.covariance = covariance;
  }
  // std::cout << "ref_changed_flag=" << ref_changed_flag
  //           << ", lat_err_ref=" << lat_err_ref
  //           << ", expectation=" << lat_err_.lat_err_ref_filter.expectation
  //           << std::endl;
  // 根据到参考线的距离变化来估计误差变化速度
  // err                    ref
  // start  end  v          start  end  v
  // 目标在左，向左移动
  // -2     -1      1        0      1      1
  // 目标在左，向右移动
  // -2     -3     -1        0     -1     -1
  // 目标在右，向左移动
  // 2       3      1        0      1      1
  // 目标在右，向右移动
  // 2       1     -1        0     -1     -1
  plan_var_t lat_err_v_ref =
      (lat_err_ref_smooth - filter.lat_err_list[0].lat_err_ref) / time_elapsed_ms;
  plan_var_t pred_err_v_ref = pred_lat_offset / time_elapsed_ms;
  if (common::com_abs(lat_err_v_ref) > common::com_abs(pred_err_v_ref)) {
    plan_var_t max_lat_err_v_ref = 0.0F;
    plan_var_t min_lat_err_v_ref = 0.0F;
    if (pred_err_v_ref >= 0) {
      max_lat_err_v_ref = 1.3F*pred_err_v_ref + 0.1F;
      min_lat_err_v_ref = 0.7F*pred_err_v_ref - 0.1F;
    } else {
      min_lat_err_v_ref = 1.3F*pred_err_v_ref - 0.1F;
      max_lat_err_v_ref = 0.7F*pred_err_v_ref + 0.1F;
    }
    if (lat_err_v_ref > max_lat_err_v_ref) {
      lat_err_v_ref = max_lat_err_v_ref;
    } else if (lat_err_v_ref < min_lat_err_v_ref) {
      lat_err_v_ref = min_lat_err_v_ref;
    } else {
      // nothing to do
    }
  }
  if (1 == ref_changed_flag) {
    // 参考线向左切换了, 使用之前的误差变化率
    lat_err_v_ref = filter.lat_err_list[0].lat_err_v;
  } else if (2 == ref_changed_flag) {
    // 参考线向右切换了, 使用之前的误差变化率
    lat_err_v_ref = filter.lat_err_list[0].lat_err_v;
  } else {
    // 参考线未发生切换
  }

#if 0
  plan_var_t lat_err_v =
      (lat_err_smooth - lat_err_.lat_err_list_a[0].lat_err) / time_elapsed_ms;
#else
  // 根据到参考线的距离变化来估计误差变化速度
  plan_var_t lat_err_v = lat_err_v_ref;
#endif

#if 0
  printf("### ref_offset=%0.3f, lat_err=%0.3f, lat_err_smooth=%0.3f"
         ", lat_err_ref=%0.3f, lat_err_ref_smooth=%0.3f, delta_lat_err=%0.3f"
         ", lat_err_v=%0.3f\n",
         ref_offset, lat_err, lat_err_smooth,
         lat_err_ref, lat_err_ref_smooth, lat_err_ref_smooth - lat_err_.lat_err_list_a[0].lat_err_ref,
         lat_err_v);
#endif

#if 0
  if (lat_err_.ref_changed.changed_flag) {
    plan_var_t prev_v = lat_err_.lat_err_list_a[0].lat_err_v;
    plan_var_t delta_v = lat_err_v - prev_v;
    //printf("      prev_v=%0.2f, curr_v=%0.2f, delta_v=%0.2f\n", prev_v, lat_err_v, delta_v);
    if (common::com_abs(delta_v) > common::com_abs(1.0F * prev_v)) {
      plan_var_t dest_v =
          common::com_abs(lat_err_v) < common::com_abs(prev_v) ?
            lat_err_v : prev_v;
      lat_err_v = dest_v;
      //printf("      correct lat_err_v from(%0.2f) to (%0.2f)\n", lat_err_v, dest_v);
    }
  }
#endif
  if (lat_err_v > 2.0F) {
    lat_err_v = 2.0F;
  } else if (lat_err_v < -2.0F) {
    lat_err_v = -2.0F;
  } else {
    // nothing to do
  }
  plan_var_t lat_err_v_raw = lat_err_v;
  plan_var_t ref_lat_err_v =
      curr_position_.v * common::com_sin(filter.lat_err_list[0].yaw_err);
  plan_var_t lat_err_a_smooth = filter.lat_err_list[0].lat_err_a;
  filter.lat_err_v_filter.expectation += lat_err_a_smooth * time_elapsed_ms;
  covariance = filter.lat_err_v_filter.covariance + 0.1F; // 0.1
  gain = covariance / (covariance + 1.0F);  // 1.0
  plan_var_t lat_err_v_smooth =
      filter.lat_err_v_filter.expectation +
      gain * (lat_err_v - filter.lat_err_v_filter.expectation);
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.lat_err_v_filter.ref_lat_err_v = ref_lat_err_v;
  filter.lat_err_v_filter.expectation = lat_err_v_smooth;
  filter.lat_err_v_filter.covariance = covariance;

  plan_var_t lat_err_a =
      (lat_err_v_smooth - filter.lat_err_list[0].lat_err_v) / time_elapsed_ms;
  if (lat_err_a > 3.0F) {
    lat_err_a = 3.0F;
  } else if (lat_err_a < -3.0F) {
    lat_err_a = -3.0F;
  } else {
    // nothing to do
  }
  lat_err_a_smooth =
      0.20F * filter.lat_err_list[3].lat_err_a +
      0.20F * filter.lat_err_list[2].lat_err_a +
      0.20F * filter.lat_err_list[1].lat_err_a +
      0.20F * filter.lat_err_list[0].lat_err_a +
      0.20F * lat_err_a;

  /// Smooth yaw error
  filter.yaw_err_filter.expectation =
      common::NormalizeAngle(
        filter.yaw_err_filter.expectation +
        filter.lat_err_list[0].yaw_err_v * time_elapsed_ms);
  covariance = filter.yaw_err_filter.covariance + 0.4F;
  gain = covariance / (covariance + 0.8F);
  plan_var_t yaw_err_smooth = common::NormalizeAngle(
      filter.yaw_err_filter.expectation +
      gain * (yaw_err - filter.yaw_err_filter.expectation));
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.yaw_err_filter.expectation = yaw_err_smooth;
  filter.yaw_err_filter.covariance = covariance;

  // Smooth yaw error changing speed
  plan_var_t yaw_err_v =
      common::AngleDiff(filter.lat_err_list[0].yaw_err, yaw_err_smooth) /
      time_elapsed_ms;
  plan_var_t yaw_err_v_raw = yaw_err_v;
  plan_var_t ref_yaw_err_v =
      curr_position_.yaw_rate -
      veh_proj_on_major_ref_line_.curvature * curr_position_.v;
  plan_var_t yaw_err_a_smooth = filter.lat_err_list[3].yaw_err_a;
  filter.yaw_err_v_filter.expectation +=
      yaw_err_a_smooth * time_elapsed_ms;
  covariance = filter.yaw_err_v_filter.covariance + 0.2F;
  gain = covariance / (covariance + 1.0F);
  plan_var_t yaw_err_v_smooth =
      filter.yaw_err_v_filter.expectation +
      gain * (yaw_err_v - filter.yaw_err_v_filter.expectation);
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.yaw_err_v_filter.ref_yaw_err_v = ref_yaw_err_v;
  filter.yaw_err_v_filter.expectation = yaw_err_v_smooth;
  filter.yaw_err_v_filter.covariance = covariance;

  plan_var_t yaw_err_a =
      (yaw_err_v_smooth - filter.lat_err_list[0].yaw_err_v) / time_elapsed_ms;
  yaw_err_a_smooth =
      0.20F * filter.lat_err_list[3].yaw_err_a +
      0.20F * filter.lat_err_list[2].yaw_err_a +
      0.20F * filter.lat_err_list[1].yaw_err_a +
      0.20F * filter.lat_err_list[0].yaw_err_a +
      0.20F * yaw_err_a;

  for (Int32_t i = LatErr::MAX_LATERAL_ERR_LIST_SIZE-1; i > 0; --i) {
    filter.lat_err_list[i].valid = filter.lat_err_list[i-1].valid;
    filter.lat_err_list[i].timestamp = filter.lat_err_list[i-1].timestamp;
    filter.lat_err_list[i].lat_err = filter.lat_err_list[i-1].lat_err;
    filter.lat_err_list[i].lat_err_ref = filter.lat_err_list[i-1].lat_err_ref;
    filter.lat_err_list[i].lat_err_v = filter.lat_err_list[i-1].lat_err_v;
    filter.lat_err_list[i].lat_err_v_ref = filter.lat_err_list[i-1].lat_err_v_ref;
    filter.lat_err_list[i].lat_err_a = filter.lat_err_list[i-1].lat_err_a;
    filter.lat_err_list[i].yaw_err = filter.lat_err_list[i-1].yaw_err;
    filter.lat_err_list[i].yaw_err_v = filter.lat_err_list[i-1].yaw_err_v;
    filter.lat_err_list[i].yaw_err_v_raw = filter.lat_err_list[i-1].yaw_err_v_raw;
    filter.lat_err_list[i].yaw_err_a = filter.lat_err_list[i-1].yaw_err_a;
  }
  filter.lat_err_list[0].valid = 1;
  filter.lat_err_list[0].timestamp = curr_timestamp_;
  filter.lat_err_list[0].lat_err = lat_err_smooth;
  filter.lat_err_list[0].lat_err_ref = lat_err_ref_smooth;
  filter.lat_err_list[0].lat_err_v = lat_err_v_smooth;
  filter.lat_err_list[0].lat_err_v_ref = lat_err_v_ref;
  filter.lat_err_list[0].lat_err_a = lat_err_a_smooth;
  filter.lat_err_list[0].yaw_err = yaw_err_smooth;
  filter.lat_err_list[0].yaw_err_v = yaw_err_v_smooth;
  filter.lat_err_list[0].yaw_err_v_raw = yaw_err_v_raw;
  filter.lat_err_list[0].yaw_err_a = yaw_err_a_smooth;


  plan_var_t pred_time = 0.8F;
  plan_var_t pos[3] = { 0.0F };
  plan_var_t pred_pos[3] = { 0.0F };
  pos[0] = curr_position_.path_point.point.x();
  pos[1] = curr_position_.path_point.point.y();
  pos[2] = curr_position_.path_point.heading;
  plan_var_t cur_yaw_rate =
      curr_position_.yaw_rate + 0.0F*pred_time*curr_position_.yaw_rate_chg_rate;
  //printf("################## change yaw_rate from %0.3f to %0.3f, chg_rate=%0.3f\n",
  //       common::com_rad2deg(curr_position_.yaw_rate),
  //       common::com_rad2deg(cur_yaw_rate),
  //       common::com_rad2deg(curr_position_.yaw_rate_chg_rate));
  vehicle_model_.EstimateNextPos(
        curr_position_.v, cur_yaw_rate,
        pred_time, pos, pred_pos);
  common::PathPoint pred_pos_proj_on_ref;
  if (!major_ref_line_->FindProjection(common::Vec2d(pred_pos[0], pred_pos[1]),
                                       &pred_pos_proj_on_ref)) {
    LOG_ERR << "Failed to find projecton from current reference line.";
    return;
  }
  common::PathPoint pred_pos_proj_on_prev_ref;
  if (!path_filter_.prev_ref_line.FindProjection(
        common::Vec2d(pred_pos[0], pred_pos[1]), &pred_pos_proj_on_prev_ref)) {
    LOG_ERR << "Failed to find projecton from previous reference line.";
    return;
  }
  plan_var_t pred_pos_ref_offset =
      pred_pos_proj_on_ref.l - pred_pos_proj_on_prev_ref.l;


  plan_var_t pred_lat_err = pred_pos_proj_on_ref.l - leading_l;
  plan_var_t pred_yaw_err = common::AngleDiff(
        pred_pos_proj_on_ref.heading, pred_pos[2]);
  plan_var_t pred_lat_err_v =
      (pred_pos_proj_on_ref.l - veh_proj_on_major_ref_line_.l) / pred_time;
#if 0
  plan_var_t lat_err_v_diff = pred_lat_err_v - lat_err_v_smooth;
  if (lat_err_v_diff > 0.1F) {
    lat_err_v_diff = 0.1F;
  } else if (lat_err_v_diff < -0.1F) {
    lat_err_v_diff = -0.1F;
  } else {
    // nothing to do
  }
  plan_var_t pred_lat_err_v_smooth = lat_err_v_smooth + lat_err_v_diff;
#else
  plan_var_t pred_lat_err_v_smooth = pred_lat_err_v;
#endif
  plan_var_t pred_yaw_err_v =
      common::NormalizeAngle(pred_yaw_err - yaw_err) / pred_time;

  // printf("lat_err_smooth=%0.2f\n", lat_err_smooth);

  if (0 != lat_moving_flag) {
    plan_var_t leading_move_spd = l_inc / time_elapsed_ms - pred_lat_err_v_smooth;
#if 0
    if (leading_move_spd > 0.05F/* && lat_err_smooth > 0.05F*/) {
      if (leading_move_spd > lat_err_smooth) {
        leading_move_spd = lat_err_smooth;
      }
    } else if (leading_move_spd < -0.05F/* && lat_err_smooth < -0.05F*/) {
      if (leading_move_spd < lat_err_smooth) {
        leading_move_spd = lat_err_smooth;
      }
    } else {
      // nothing to do
    }
#endif
    pred_lat_err -= leading_move_spd*pred_time;
#if 1
    if (pred_lat_err > max_leading_lat_offset) {
      pred_lat_err = max_leading_lat_offset;
    } else if (pred_lat_err < -max_leading_lat_offset) {
      pred_lat_err = -max_leading_lat_offset;
    } else {
      // nothing to do
    }
#else
    plan_var_t max_lat_err = common::Min(common::com_abs(lat_err_smooth), max_leading_lat_offset);
    if (pred_lat_err > max_lat_err) {
      pred_lat_err = max_lat_err;
    } else if (pred_lat_err < -max_lat_err) {
      pred_lat_err = -max_lat_err;
    } else {
      // nothing to do
    }
#endif
  }

  printf("######################"
         "lat_err_v_smooth=%0.2f, pred_lat_err_v=%0.2f, "
         "pred_pos_ref_offset=%0.2f\n",
         lat_err_v_smooth, pred_lat_err_v, pred_pos_ref_offset);

  lat_err_.lat_err_sample[0].lat_err = lat_err;
  lat_err_.lat_err_sample[0].lat_err_smooth = lat_err_smooth;
  lat_err_.lat_err_sample[0].lat_err_v = lat_err_v_raw;
  lat_err_.lat_err_sample[0].lat_err_v_smooth = lat_err_v_smooth;
  lat_err_.lat_err_sample[0].yaw_err = yaw_err;
  lat_err_.lat_err_sample[0].yaw_err_smooth = yaw_err_smooth;
  lat_err_.lat_err_sample[0].yaw_err_v = yaw_err_v_raw;
  lat_err_.lat_err_sample[0].yaw_err_v_smooth = yaw_err_v_smooth;

  lat_err_.lat_err_sample[1].lat_err = pred_lat_err;
  lat_err_.lat_err_sample[1].lat_err_smooth = pred_lat_err;
  lat_err_.lat_err_sample[1].lat_err_v = pred_lat_err_v;
  lat_err_.lat_err_sample[1].lat_err_v_smooth = pred_lat_err_v_smooth;
  lat_err_.lat_err_sample[1].yaw_err = pred_yaw_err;
  lat_err_.lat_err_sample[1].yaw_err_smooth = pred_yaw_err;
  lat_err_.lat_err_sample[1].yaw_err_v = pred_yaw_err_v;
  lat_err_.lat_err_sample[1].yaw_err_v_smooth = pred_yaw_err_v;

  lat_err_.pred_point.x = pred_pos[0];
  lat_err_.pred_point.y = pred_pos[1];
  lat_err_.pred_point.h = pred_pos[2];

#if (ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_DEBUGGING_FILE)
  if (true) {
    if (!log_file_lat_err_.IsOpen()) {
      log_file_lat_err_.SetModuleName("lat_err");
      log_file_lat_err_.Open();
      log_file_lat_err_.Enable(true);

      std::snprintf(str_buff_, sizeof(str_buff_)-1,
                    "timestamp,time_elapsed,trj_status,  "

                    "lat_err,lat_err_smooth,"
                    "lat_err_ref,lat_err_ref_smooth,"
                    "lat_err_v_raw,lat_err_v_ref,lat_err_v,lat_err_v_smooth,"
                    "lat_err_a,lat_err_a_smooth,    "

                    "yaw_err,yaw_err_smooth,"
                    "yaw_err_v,yaw_err_v_smooth,"
                    "yaw_err_a,yaw_err_a_smooth"
                    "\n");
      log_file_lat_err_.Write(str_buff_);
    }
    std::snprintf(str_buff_, sizeof(str_buff_)-1,
                  "%ld,%ld,%d,  "

                  "%0.3f,%0.3f,"
                  "%0.3f,%0.3f,"
                  "%0.3f,%0.3f,%0.3f,%0.3f,"
                  "%0.3f,%0.3f,    "

                  "%0.1f,%0.1f,"
                  "%0.1f,%0.1f,"
                  "%0.1f,%0.1f"
                  "\n",
                  common::GetClockNowMs(), time_elapsed, status_.trj_status,

                  lat_err, lat_err_smooth,
                  veh_proj_on_major_ref_line_.l, lat_err_ref_smooth,
                  lat_err_v_raw, lat_err_v_ref,lat_err_v,lat_err_v_smooth,
                  lat_err_a, lat_err_a_smooth,

                  common::com_rad2deg(yaw_err), common::com_rad2deg(yaw_err_smooth),
                  common::com_rad2deg(yaw_err_v), common::com_rad2deg(yaw_err_v_smooth),
                  common::com_rad2deg(yaw_err_a), common::com_rad2deg(yaw_err_a_smooth));
    log_file_lat_err_.Write(str_buff_);
  }
#endif
}
#else
void TrajectoryPlanningStateLattice::CalcLatErr(
    plan_var_t leading_l, plan_var_t ref_offset, plan_var_t pred_lat_offset) {
  if (!lat_err_.filter[0].lat_err_list[0].valid) {
    lat_err_.Clear();
  }

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        lat_err_.filter[0].lat_err_list[0].timestamp, curr_timestamp_);
  // plan_var_t time_elapsed_ms = time_elapsed * 0.001F;

  if ((time_elapsed > 300) || (time_elapsed < 0)) {
    lat_err_.Clear();
  }

  CalcLatErrSample(
        0,
        curr_position_.path_point, veh_proj_on_major_ref_line_,
        leading_l, ref_offset, pred_lat_offset);

  plan_var_t pred_time = 0.8F;
  plan_var_t pos[3] = { 0.0F };
  plan_var_t pred_pos[3] = { 0.0F };
  pos[0] = curr_position_.path_point.point.x();
  pos[1] = curr_position_.path_point.point.y();
  pos[2] = curr_position_.path_point.heading;
  vehicle_model_.EstimateNextPos(
        curr_position_.v, curr_position_.yaw_rate,
        pred_time, pos, pred_pos);
  common::PathPoint pred_path_point;
  pred_path_point.point.set_x(pred_pos[0]);
  pred_path_point.point.set_y(pred_pos[1]);
  pred_path_point.heading = pred_pos[2];
  pred_path_point.curvature = curr_position_.path_point.curvature;
  common::PathPoint pred_pos_proj_on_ref;
  if (!major_ref_line_->FindProjection(pred_path_point.point,
                                       &pred_pos_proj_on_ref)) {
    LOG_ERR << "Failed to find projecton from current reference line.";
    return;
  }

  CalcLatErrSample(
        1,
        pred_path_point, pred_pos_proj_on_ref,
        leading_l, ref_offset, pred_lat_offset);

  lat_err_.pred_point.x = pred_pos[0];
  lat_err_.pred_point.y = pred_pos[1];
  lat_err_.pred_point.h = pred_pos[2];
}
#endif

void TrajectoryPlanningStateLattice::CalcLatErrSample(
    Int32_t sample_idx,
    const common::PathPoint& curr_point, const common::PathPoint& proj_point,
    plan_var_t leading_l, plan_var_t ref_offset, plan_var_t pred_lat_offset) {
  plan_var_t lat_err = proj_point.l - leading_l;
  plan_var_t yaw_err = common::AngleDiff(proj_point.heading, curr_point.heading);

  LatErr::LatErrFilter& filter = lat_err_.filter[sample_idx];
  LatErr::LatErrSample& sample = lat_err_.lat_err_sample[sample_idx];

  if (!filter.lat_err_list[0].valid) {
    filter.lat_err_filter.expectation = lat_err;
    filter.lat_err_ref_filter.expectation = proj_point.l;
    filter.yaw_err_filter.expectation = yaw_err;

    filter.lat_err_list[0].valid = true;
    filter.lat_err_list[0].timestamp = curr_timestamp_;
    filter.lat_err_list[0].lat_err = lat_err;
    filter.lat_err_list[0].lat_err_ref = proj_point.l;
    filter.lat_err_list[0].lat_err_v = 0.0F;
    filter.lat_err_list[0].lat_err_a = 0.0F;
    filter.lat_err_list[0].yaw_err = yaw_err;
    filter.lat_err_list[0].yaw_err_v = 0.0F;
    filter.lat_err_list[0].yaw_err_v_raw = 0.0F;
    filter.lat_err_list[0].yaw_err_a = 0.0F;

    sample.lat_err = lat_err;
    sample.lat_err_smooth = lat_err;
    sample.lat_err_v = 0.0F;
    sample.lat_err_v_smooth = 0.0F;
    sample.yaw_err = yaw_err;
    sample.yaw_err_smooth = yaw_err;
    sample.yaw_err_v = 0.0F;
    sample.yaw_err_v_smooth = 0.0F;

    return;
  }

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        filter.lat_err_list[0].timestamp, curr_timestamp_);
  plan_var_t time_elapsed_ms = time_elapsed * 0.001F;

  if ((time_elapsed > 300) || (time_elapsed < 0)) {
    filter.lat_err_filter.expectation = lat_err;
    filter.lat_err_ref_filter.expectation = proj_point.l;
    filter.yaw_err_filter.expectation = yaw_err;

    filter.lat_err_list[0].valid = true;
    filter.lat_err_list[0].timestamp = curr_timestamp_;
    filter.lat_err_list[0].lat_err = lat_err;
    filter.lat_err_list[0].lat_err_ref = proj_point.l;
    filter.lat_err_list[0].lat_err_v = 0.0F;
    filter.lat_err_list[0].lat_err_a = 0.0F;
    filter.lat_err_list[0].yaw_err = yaw_err;
    filter.lat_err_list[0].yaw_err_v = 0.0F;
    filter.lat_err_list[0].yaw_err_v_raw = 0.0F;
    filter.lat_err_list[0].yaw_err_a = 0.0F;

    sample.lat_err = lat_err;
    sample.lat_err_smooth = lat_err;
    sample.lat_err_v = 0.0F;
    sample.lat_err_v_smooth = 0.0F;
    sample.yaw_err = yaw_err;
    sample.yaw_err_smooth = yaw_err;
    sample.yaw_err_v = 0.0F;
    sample.yaw_err_v_smooth = 0.0F;

    return;
  }

  // 判断参考线是否变更了, 0 ~ unchanged, 1 ~ changed to left, 2 ~ changed to right
  Int32_t ref_changed_flag = IsRefLineChanged();

  plan_var_t abs_ref_offset = common::com_abs(ref_offset);
  if ((0.2F < abs_ref_offset) && (abs_ref_offset < 2.0F)) {
    lat_err_.ref_changed.ref_offset = ref_offset;
    lat_err_.ref_changed.changed_flag = true;
    lat_err_.ref_changed.changed_count = 10;
  }
  if (lat_err_.ref_changed.changed_flag) {
    lat_err_.ref_changed.changed_count--;
    if (lat_err_.ref_changed.changed_count <= 0) {
      lat_err_.ref_changed.changed_count = 0;
      lat_err_.ref_changed.changed_flag = false;
    }
  }

  /// Smooth lateral error
  filter.lat_err_filter.expectation +=
      filter.lat_err_list[0].lat_err_v * time_elapsed_ms;
  plan_var_t covariance = filter.lat_err_filter.covariance + 0.3F;
  plan_var_t gain = covariance / (covariance + 0.8F);
  plan_var_t lat_err_smooth =
      filter.lat_err_filter.expectation +
      gain * (lat_err - filter.lat_err_filter.expectation);
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.lat_err_filter.expectation = lat_err_smooth;
  filter.lat_err_filter.covariance = covariance;

  /// lateral ref error
  plan_var_t lat_err_ref = proj_point.l;
  plan_var_t lat_err_ref_smooth = lat_err_ref;
  if (1 == ref_changed_flag) {
    // 参考线向左切换了, 使用之前的误差变化率
    filter.lat_err_ref_filter.expectation = lat_err_ref;
    filter.lat_err_list[0].lat_err_ref = lat_err_ref;
  } else if (2 == ref_changed_flag) {
    // 参考线向右切换了, 使用之前的误差变化率
    filter.lat_err_ref_filter.expectation = lat_err_ref;
    filter.lat_err_list[0].lat_err_ref = lat_err_ref;
  } else {
    // 参考线未发生切换
    /// Smooth reference lateral error
    filter.lat_err_ref_filter.expectation +=
        filter.lat_err_list[0].lat_err_v * time_elapsed_ms;
    covariance = filter.lat_err_ref_filter.covariance + 0.3F;
    gain = covariance / (covariance + 0.8F);
    lat_err_ref_smooth =
        filter.lat_err_ref_filter.expectation +
        gain * (lat_err_ref - filter.lat_err_ref_filter.expectation);
    covariance = (1.0F - gain) * covariance;
    if (covariance < 1e-3F) {
      covariance = 1e-3F;
    }
    filter.lat_err_ref_filter.expectation = lat_err_ref_smooth;
    filter.lat_err_ref_filter.covariance = covariance;
  }
  // std::cout << "ref_changed_flag=" << ref_changed_flag
  //           << ", lat_err_ref=" << lat_err_ref
  //           << ", expectation=" << lat_err_.lat_err_ref_filter.expectation
  //           << std::endl;
  // 根据到参考线的距离变化来估计误差变化速度
  // err                    ref
  // start  end  v          start  end  v
  // 目标在左，向左移动
  // -2     -1      1        0      1      1
  // 目标在左，向右移动
  // -2     -3     -1        0     -1     -1
  // 目标在右，向左移动
  // 2       3      1        0      1      1
  // 目标在右，向右移动
  // 2       1     -1        0     -1     -1
  plan_var_t lat_err_v_ref =
      (lat_err_ref_smooth - filter.lat_err_list[0].lat_err_ref) / time_elapsed_ms;
  plan_var_t pred_err_v_ref = pred_lat_offset / time_elapsed_ms;
  if (common::com_abs(lat_err_v_ref) > common::com_abs(pred_err_v_ref)) {
    plan_var_t max_lat_err_v_ref = 0.0F;
    plan_var_t min_lat_err_v_ref = 0.0F;
    if (pred_err_v_ref >= 0) {
      max_lat_err_v_ref = 1.3F*pred_err_v_ref + 0.1F;
      min_lat_err_v_ref = 0.7F*pred_err_v_ref - 0.1F;
    } else {
      min_lat_err_v_ref = 1.3F*pred_err_v_ref - 0.1F;
      max_lat_err_v_ref = 0.7F*pred_err_v_ref + 0.1F;
    }
    if (lat_err_v_ref > max_lat_err_v_ref) {
      lat_err_v_ref = max_lat_err_v_ref;
    } else if (lat_err_v_ref < min_lat_err_v_ref) {
      lat_err_v_ref = min_lat_err_v_ref;
    } else {
      // nothing to do
    }
  }
  if (1 == ref_changed_flag) {
    // 参考线向左切换了, 使用之前的误差变化率
    lat_err_v_ref = filter.lat_err_list[0].lat_err_v;
  } else if (2 == ref_changed_flag) {
    // 参考线向右切换了, 使用之前的误差变化率
    lat_err_v_ref = filter.lat_err_list[0].lat_err_v;
  } else {
    // 参考线未发生切换
  }

#if 0
  plan_var_t lat_err_v =
      (lat_err_smooth - lat_err_.lat_err_list_a[0].lat_err) / time_elapsed_ms;
#else
  // 根据到参考线的距离变化来估计误差变化速度
  plan_var_t lat_err_v = lat_err_v_ref;
#endif

#if 0
  printf("### ref_offset=%0.3f, lat_err=%0.3f, lat_err_smooth=%0.3f"
         ", lat_err_ref=%0.3f, lat_err_ref_smooth=%0.3f, delta_lat_err=%0.3f"
         ", lat_err_v=%0.3f\n",
         ref_offset, lat_err, lat_err_smooth,
         lat_err_ref, lat_err_ref_smooth, lat_err_ref_smooth - lat_err_.lat_err_list_a[0].lat_err_ref,
         lat_err_v);
#endif

  if (lat_err_v > 2.0F) {
    lat_err_v = 2.0F;
  } else if (lat_err_v < -2.0F) {
    lat_err_v = -2.0F;
  } else {
    // nothing to do
  }
  plan_var_t lat_err_v_raw = lat_err_v;
  plan_var_t ref_lat_err_v =
      curr_position_.v * common::com_sin(filter.lat_err_list[0].yaw_err);
  plan_var_t lat_err_a_smooth = filter.lat_err_list[0].lat_err_a;
  filter.lat_err_v_filter.expectation += lat_err_a_smooth * time_elapsed_ms;
  covariance = filter.lat_err_v_filter.covariance + 0.1F; // 0.1
  gain = covariance / (covariance + 1.0F);  // 1.0
  plan_var_t lat_err_v_smooth =
      filter.lat_err_v_filter.expectation +
      gain * (lat_err_v - filter.lat_err_v_filter.expectation);
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.lat_err_v_filter.ref_lat_err_v = ref_lat_err_v;
  filter.lat_err_v_filter.expectation = lat_err_v_smooth;
  filter.lat_err_v_filter.covariance = covariance;

  plan_var_t lat_err_a =
      (lat_err_v_smooth - filter.lat_err_list[0].lat_err_v) / time_elapsed_ms;
  if (lat_err_a > 3.0F) {
    lat_err_a = 3.0F;
  } else if (lat_err_a < -3.0F) {
    lat_err_a = -3.0F;
  } else {
    // nothing to do
  }
  lat_err_a_smooth =
      0.20F * filter.lat_err_list[3].lat_err_a +
      0.20F * filter.lat_err_list[2].lat_err_a +
      0.20F * filter.lat_err_list[1].lat_err_a +
      0.20F * filter.lat_err_list[0].lat_err_a +
      0.20F * lat_err_a;

  /// Smooth yaw error
  filter.yaw_err_filter.expectation =
      common::NormalizeAngle(
        filter.yaw_err_filter.expectation +
        filter.lat_err_list[0].yaw_err_v * time_elapsed_ms);
  covariance = filter.yaw_err_filter.covariance + 0.4F;
  gain = covariance / (covariance + 0.8F);
  plan_var_t yaw_err_smooth = common::NormalizeAngle(
      filter.yaw_err_filter.expectation +
      gain * (yaw_err - filter.yaw_err_filter.expectation));
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.yaw_err_filter.expectation = yaw_err_smooth;
  filter.yaw_err_filter.covariance = covariance;

  // Smooth yaw error changing speed
  plan_var_t yaw_err_v =
      common::AngleDiff(filter.lat_err_list[0].yaw_err, yaw_err_smooth) /
      time_elapsed_ms;
  plan_var_t yaw_err_v_raw = yaw_err_v;
  plan_var_t ref_yaw_err_v =
      curr_position_.yaw_rate -
      veh_proj_on_major_ref_line_.curvature * curr_position_.v;
  plan_var_t yaw_err_a_smooth = filter.lat_err_list[3].yaw_err_a;
  filter.yaw_err_v_filter.expectation +=
      yaw_err_a_smooth * time_elapsed_ms;
  covariance = filter.yaw_err_v_filter.covariance + 0.2F;
  gain = covariance / (covariance + 1.0F);
  plan_var_t yaw_err_v_smooth =
      filter.yaw_err_v_filter.expectation +
      gain * (yaw_err_v - filter.yaw_err_v_filter.expectation);
  covariance = (1.0F - gain) * covariance;
  if (covariance < 1e-3F) {
    covariance = 1e-3F;
  }
  filter.yaw_err_v_filter.ref_yaw_err_v = ref_yaw_err_v;
  filter.yaw_err_v_filter.expectation = yaw_err_v_smooth;
  filter.yaw_err_v_filter.covariance = covariance;

  plan_var_t yaw_err_a =
      (yaw_err_v_smooth - filter.lat_err_list[0].yaw_err_v) / time_elapsed_ms;
  yaw_err_a_smooth =
      0.20F * filter.lat_err_list[3].yaw_err_a +
      0.20F * filter.lat_err_list[2].yaw_err_a +
      0.20F * filter.lat_err_list[1].yaw_err_a +
      0.20F * filter.lat_err_list[0].yaw_err_a +
      0.20F * yaw_err_a;

  for (Int32_t i = LatErr::MAX_LATERAL_ERR_LIST_SIZE-1; i > 0; --i) {
    filter.lat_err_list[i].valid = filter.lat_err_list[i-1].valid;
    filter.lat_err_list[i].timestamp = filter.lat_err_list[i-1].timestamp;
    filter.lat_err_list[i].lat_err = filter.lat_err_list[i-1].lat_err;
    filter.lat_err_list[i].lat_err_ref = filter.lat_err_list[i-1].lat_err_ref;
    filter.lat_err_list[i].lat_err_v = filter.lat_err_list[i-1].lat_err_v;
    filter.lat_err_list[i].lat_err_v_ref = filter.lat_err_list[i-1].lat_err_v_ref;
    filter.lat_err_list[i].lat_err_a = filter.lat_err_list[i-1].lat_err_a;
    filter.lat_err_list[i].yaw_err = filter.lat_err_list[i-1].yaw_err;
    filter.lat_err_list[i].yaw_err_v = filter.lat_err_list[i-1].yaw_err_v;
    filter.lat_err_list[i].yaw_err_v_raw = filter.lat_err_list[i-1].yaw_err_v_raw;
    filter.lat_err_list[i].yaw_err_a = filter.lat_err_list[i-1].yaw_err_a;
  }
  filter.lat_err_list[0].valid = 1;
  filter.lat_err_list[0].timestamp = curr_timestamp_;
  filter.lat_err_list[0].lat_err = lat_err_smooth;
  filter.lat_err_list[0].lat_err_ref = lat_err_ref_smooth;
  filter.lat_err_list[0].lat_err_v = lat_err_v_smooth;
  filter.lat_err_list[0].lat_err_v_ref = lat_err_v_ref;
  filter.lat_err_list[0].lat_err_a = lat_err_a_smooth;
  filter.lat_err_list[0].yaw_err = yaw_err_smooth;
  filter.lat_err_list[0].yaw_err_v = yaw_err_v_smooth;
  filter.lat_err_list[0].yaw_err_v_raw = yaw_err_v_raw;
  filter.lat_err_list[0].yaw_err_a = yaw_err_a_smooth;

  sample.lat_err = lat_err;
  sample.lat_err_smooth = lat_err_smooth;
  sample.lat_err_v = lat_err_v_raw;
  sample.lat_err_v_smooth = lat_err_v_smooth;
  sample.yaw_err = yaw_err;
  sample.yaw_err_smooth = yaw_err_smooth;
  sample.yaw_err_v = yaw_err_v_raw;
  sample.yaw_err_v_smooth = yaw_err_v_smooth;
}

Int32_t TrajectoryPlanningStateLattice::SelectTargetTrajectory(
    Int32_t* trj_req_type) {
  *trj_req_type = TRJ_REQ_LKA;

  Int32_t candidate_trj_index = optimal_trj_index_.trj_idx_lka;
  if (timers_.fade_in.IsActive()) {
    candidate_trj_index = optimal_trj_index_.trj_idx_curr_lane;
  }
  if ((candidate_trj_index < 0) ||
      (candidate_trj_index >= candidate_trj_list_.Size())) {
    LOG_ERR << "    >>> Invalid candidate trajectory index (idx="
            << candidate_trj_index << ")";
    return (-1);
  }

  const CandidateTrajectory& curr_candidate_trj =
      candidate_trj_list_[candidate_trj_index];

//#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
  LOG_INFO(5) << "### Changing lane request ="
              << action_manager_.changing_lane_req.request();
//#endif

  plan_var_t ref_deceleration = common::Max(
        -3.0F,
        common::Min(-2.0F, curr_candidate_trj.collision_obj.deceleration-0.5F));
  bool exist_uncertain_obj_in_left_lane = false;
  Int32_t unsafety_lane_reason_left = REFUSE_CHANGINHG_LANE_REASON_NONE;
  Int32_t unsafety_lane_reason_right = REFUSE_CHANGINHG_LANE_REASON_NONE;
  bool safe_left_lane = CheckSafetyOfCandidateTrajectory(
        optimal_trj_index_.trj_idx_left_lane, ref_deceleration,
        &exist_uncertain_obj_in_left_lane, &unsafety_lane_reason_left);
  bool exist_uncertain_obj_in_right_lane = false;
  bool safe_right_lane = CheckSafetyOfCandidateTrajectory(
        optimal_trj_index_.trj_idx_right_lane, ref_deceleration,
        &exist_uncertain_obj_in_right_lane, &unsafety_lane_reason_right);

  ref_deceleration = -3.0F;
  bool exist_uncertain_obj_in_curr_lane = false;
  Int32_t unsafety_lane_reason_curr = REFUSE_CHANGINHG_LANE_REASON_NONE;
  bool safe_curr_lane = CheckSafetyOfCandidateTrajectory(
        optimal_trj_index_.trj_idx_curr_lane, ref_deceleration,
        &exist_uncertain_obj_in_curr_lane, &unsafety_lane_reason_curr);

  if ((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == status_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == status_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == status_.trj_status)) {
    if (exist_uncertain_obj_in_left_lane) {
      if (action_smoothing_.abort_changing_lane_left_by_uncertain_obj.Smooth(true)) {
        safe_left_lane = false;
        unsafety_lane_reason_left = REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE;
      }
    } else {
      action_smoothing_.abort_changing_lane_left_by_uncertain_obj.Smooth(false);
    }
    if (exist_uncertain_obj_in_right_lane) {
      if (action_smoothing_.abort_changing_lane_right_by_uncertain_obj.Smooth(true)) {
        safe_right_lane = false;
        unsafety_lane_reason_right = REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE;
      }
    } else {
      action_smoothing_.abort_changing_lane_right_by_uncertain_obj.Smooth(false);
    }
  } else {
    action_smoothing_.abort_changing_lane_left_by_uncertain_obj.Smooth(false);
    action_smoothing_.abort_changing_lane_right_by_uncertain_obj.Smooth(false);

    if (exist_uncertain_obj_in_left_lane) {
      safe_left_lane = false;
      unsafety_lane_reason_left = REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE;
    }
    if (exist_uncertain_obj_in_right_lane) {
      safe_right_lane = false;
      unsafety_lane_reason_right = REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE;
    }
  }

  Float32_t left_width = 0.0F;
  Float32_t right_width = 0.0F;
  driving_map_->GetLaneWidth(curr_lane_index_, veh_proj_on_curr_lane_.s,
                             &left_width, &right_width);
  plan_var_t curr_l_ref = veh_proj_on_major_ref_line_.l;

  // 获取上个预瞄点在主参考线的横向距离
  common::PathPoint pre_leading_proj_on_curr_ref;
  major_ref_line_->FindProjection(path_filter_.prev_leading_point.point,
                                  &pre_leading_proj_on_curr_ref);
  plan_var_t pre_leading_point_l = pre_leading_proj_on_curr_ref.l;

  // plan_var_t prev_l_ref = status_.prev_proj_on_ref.l;

  Int32_t ref_line_changed_type = 0;
  ref_line_changed_type = IsRefLineChanged();

  if (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == status_.trj_status) {
    if (1 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II;
    }
  } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == status_.trj_status) {
    if (1 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II;
    }
  } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == status_.trj_status) {
    if (2 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II;
    }
  } else if (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == status_.trj_status) {
    if (2 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II;
    }
  } else if (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == status_.trj_status) {
#if 0
    if (common::com_abs(curr_l_ref) < param_.changing_lane_complete_cond_lat_offset) {
      status_.trj_status = TRJ_STATUS_LKA;
    } else {
      if (2 == ref_line_changed_type) {
        status_.trj_status = TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I;
      }
    }
#else
    if (2 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I;
    } else {
      if ((curr_l_ref > -param_.changing_lane_complete_cond_lat_offset) &&
          (pre_leading_point_l >  -param_.changing_lane_complete_cond_lat_offset)) {
        status_.trj_status = TRJ_STATUS_LKA;
      }
    }
#endif
  } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II == status_.trj_status) {
#if 0
    if (common::com_abs(curr_l_ref) < param_.changing_lane_complete_cond_lat_offset) {
      status_.trj_status = TRJ_STATUS_LKA;
    } else {
      if (2 == ref_line_changed_type) {
        status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I;
      }
    }
#else
    if (2 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I;
    } else {
      if ((curr_l_ref  > -param_.changing_lane_complete_cond_lat_offset) &&
          (pre_leading_point_l >  -param_.changing_lane_complete_cond_lat_offset)) {
        status_.trj_status = TRJ_STATUS_LKA;
      }
    }
#endif
  } else if (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == status_.trj_status) {
#if 0
    if (common::com_abs(curr_l_ref) < param_.changing_lane_complete_cond_lat_offset) {
      status_.trj_status = TRJ_STATUS_LKA;
    } else {
      if (1 == ref_line_changed_type) {
        status_.trj_status = TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I;
      }
    }
#else
    if (1 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I;
    } else {
      if ((curr_l_ref < param_.changing_lane_complete_cond_lat_offset) &&
          (pre_leading_point_l < param_.changing_lane_complete_cond_lat_offset)) {
        status_.trj_status = TRJ_STATUS_LKA;
      }
    }
#endif
  } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == status_.trj_status) {
#if 0
    if (common::com_abs(curr_l_ref) < param_.changing_lane_complete_cond_lat_offset) {
      status_.trj_status = TRJ_STATUS_LKA;
    } else {
      if (1 == ref_line_changed_type) {
        status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I;
      }
    }
#else
    if (1 == ref_line_changed_type) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I;
    } else {
      if ((curr_l_ref < param_.changing_lane_complete_cond_lat_offset) &&
          (pre_leading_point_l <  param_.changing_lane_complete_cond_lat_offset)) {
        status_.trj_status = TRJ_STATUS_LKA;
      }
    }
#endif
  } else {
    // nothing to do
  }

  if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT ==
      action_manager_.changing_lane_req.request()) {
    // 请求向左变道
    if ((TRJ_STATUS_LKA == status_.trj_status) ||
        (TRJ_STATUS_INVALID == status_.trj_status) ||
        (TRJ_STATUS_LKA_BYPASSING_LEFT == status_.trj_status) ||
        (TRJ_STATUS_LKA_BYPASSING_RIGHT == status_.trj_status)) {
      if (safe_left_lane) {
        status_.trj_status = TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I;
        action_manager_.changing_lane_rsp.set_response(
              ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
        action_manager_.changing_lane_rsp.set_refuse_reason(
              REFUSE_CHANGINHG_LANE_REASON_NONE);
      } else {
//#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
        LOG_INFO(5) << "    >>> Not safety, refuse changing to left lane.";
//#endif
        action_manager_.changing_lane_rsp.set_response(
              ChangingLaneRsp::RSP_CHANGING_LANE_REFUSE);
        action_manager_.changing_lane_rsp.set_refuse_reason(
              unsafety_lane_reason_left);
      }
    }
  } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT ==
             action_manager_.changing_lane_req.request()) {
    if ((TRJ_STATUS_LKA == status_.trj_status) ||
        (TRJ_STATUS_INVALID == status_.trj_status) ||
        (TRJ_STATUS_LKA_BYPASSING_LEFT == status_.trj_status) ||
        (TRJ_STATUS_LKA_BYPASSING_RIGHT == status_.trj_status)) {
      if (safe_right_lane) {
//#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
        LOG_INFO(5) << "    >>> Allow changing to right lane.";
//#endif
        status_.trj_status = TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I;
        action_manager_.changing_lane_rsp.set_response(
              ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
        action_manager_.changing_lane_rsp.set_refuse_reason(
              REFUSE_CHANGINHG_LANE_REASON_NONE);
      } else {
//#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
        LOG_INFO(5) << "    >>> Not safety, refuse changing to right lane.";
//#endif
        action_manager_.changing_lane_rsp.set_response(
              ChangingLaneRsp::RSP_CHANGING_LANE_REFUSE);
        action_manager_.changing_lane_rsp.set_refuse_reason(
              unsafety_lane_reason_right);
      }
    }
  } else if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT ==
             action_manager_.changing_lane_req.request()) {
    bool refuse_abort_changing_lane = false;
    Int32_t refuse_reason = REFUSE_CHANGINHG_LANE_REASON_NONE;
    if (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == status_.trj_status) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II;
      path_filter_.path_trend_mgr_lat.ClearPositive();
    } else if (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == status_.trj_status) {
      if (safe_right_lane) {
        status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I;
        path_filter_.path_trend_mgr_lat.ClearPositive();
      } else {
        // hold previous trajectory status
        refuse_abort_changing_lane = true;
        refuse_reason = unsafety_lane_reason_right;
      }
    } else if (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == status_.trj_status) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II;
      path_filter_.path_trend_mgr_lat.ClearNegative();
    } else if (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == status_.trj_status) {
      if (safe_left_lane) {
        status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I;
        path_filter_.path_trend_mgr_lat.ClearNegative();
      } else {
        // hold previous trajectory status
        refuse_abort_changing_lane = true;
        refuse_reason = unsafety_lane_reason_left;
      }
    } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == status_.trj_status) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II;
      path_filter_.path_trend_mgr_lat.ClearPositive();
    } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II == status_.trj_status) {
      if (safe_right_lane) {
        status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I;
        path_filter_.path_trend_mgr_lat.ClearPositive();
      } else {
        // hold previous trajectory status
        refuse_abort_changing_lane = true;
        refuse_reason = unsafety_lane_reason_right;
      }
    } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == status_.trj_status) {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II;
      path_filter_.path_trend_mgr_lat.ClearNegative();
    } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == status_.trj_status) {
      if (safe_left_lane) {
        status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I;
        path_filter_.path_trend_mgr_lat.ClearNegative();
      } else {
        // hold previous trajectory status
        refuse_abort_changing_lane = true;
        refuse_reason = unsafety_lane_reason_left;
      }
    }

    if (refuse_abort_changing_lane) {
      action_manager_.changing_lane_rsp.set_response(
            ChangingLaneRsp::RSP_CHANGING_LANE_REFUSE);
      action_manager_.changing_lane_rsp.set_refuse_reason(
            refuse_reason);
    } else {
      action_manager_.changing_lane_rsp.set_response(
            ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
      action_manager_.changing_lane_rsp.set_refuse_reason(
            action_manager_.changing_lane_req.refuse_reason());
    }
  }

  // 根据状态选择轨迹
  if (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == status_.trj_status) {
    if (safe_left_lane || (!safe_curr_lane)) {
      *trj_req_type = TRJ_REQ_CHANGING_LANE_LEFT;
      candidate_trj_index = optimal_trj_index_.trj_idx_left_lane;
    } else {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II;
      path_filter_.path_trend_mgr_lat.ClearPositive();

      action_manager_.changing_lane_rsp.set_response(
            ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
      action_manager_.changing_lane_rsp.set_refuse_reason(
            unsafety_lane_reason_left);
    }
  } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == status_.trj_status) {
    if (safe_left_lane || (!safe_curr_lane)) {
      *trj_req_type = TRJ_REQ_CHANGING_LANE_LEFT;
      candidate_trj_index = optimal_trj_index_.trj_idx_left_lane;
    } else {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II;
      path_filter_.path_trend_mgr_lat.ClearPositive();

      action_manager_.changing_lane_rsp.set_response(
            ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
      action_manager_.changing_lane_rsp.set_refuse_reason(
            unsafety_lane_reason_left);
    }
  } else if (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == status_.trj_status) {
    if (safe_right_lane || (!safe_curr_lane)) {
      *trj_req_type = TRJ_REQ_CHANGING_LANE_RIGHT;
      candidate_trj_index = optimal_trj_index_.trj_idx_right_lane;
    } else {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II;
      path_filter_.path_trend_mgr_lat.ClearNegative();

      action_manager_.changing_lane_rsp.set_response(
            ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
      action_manager_.changing_lane_rsp.set_refuse_reason(
            unsafety_lane_reason_right);
    }
  } else if (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == status_.trj_status) {
    if (safe_right_lane || (!safe_curr_lane)) {
      *trj_req_type = TRJ_REQ_CHANGING_LANE_RIGHT;
      candidate_trj_index = optimal_trj_index_.trj_idx_right_lane;
    } else {
      status_.trj_status = TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II;
      path_filter_.path_trend_mgr_lat.ClearNegative();

      action_manager_.changing_lane_rsp.set_response(
            ChangingLaneRsp::RSP_CHANGING_LANE_ACCEPT);
      action_manager_.changing_lane_rsp.set_refuse_reason(
            unsafety_lane_reason_right);
    }
  } else{
    // nothing to do
  }


  /* k001 pengc 2022-04-10 (begin) */
  // 在一定的时间范围内，尽量保持之前的避让趋势
  if ((TRJ_REQ_LKA == *trj_req_type) &&
      config_.enable_avoiding_collision_in_lane &&
      (DRIVING_DIRECTION_BACKWARD != driving_direction_)) {

    candidate_trj_index = optimal_trj_index_.trj_idx_curr_lane;
    /* k001 pengc 2022-04-03 (begin) */
    // 避免避让的轨迹左右频繁跳动, 优先向左避让
    if ((candidate_trj_index == optimal_trj_index_.trj_idx_curr_lane_right) &&
        (optimal_trj_index_.trj_idx_curr_lane_left >= 0)) {
      // 最优轨迹是向右避让时，判断下是否可以向左避让
      const CandidateTrajectory& trj_left =
          candidate_trj_list_[optimal_trj_index_.trj_idx_curr_lane_left];
      const CandidateTrajectory& trj_right =
          candidate_trj_list_[optimal_trj_index_.trj_idx_curr_lane_right];
      if ((common::com_abs(trj_left.collision_obj.lat_dist) > 0.1F) &&
          (common::com_abs(trj_left.collision_obj.deceleration -
                           trj_right.collision_obj.deceleration) < 1.0F)) {
        candidate_trj_index = optimal_trj_index_.trj_idx_curr_lane_left;
      }
    }
    /* k001 pengc 2022-04-03 (end) */

    if ((candidate_trj_index < 0) ||
        (candidate_trj_index >= candidate_trj_list_.Size())) {
      LOG_ERR << "    >>> Invalid candidate trajectory index (idx="
              << candidate_trj_index << ")";
      return (-1);
    }

    const CandidateTrajectory& candidate_trj =
        candidate_trj_list_[candidate_trj_index];
    const CandidateTrajectory::TrjNode* first_trj_node =
        &(candidate_trj.trajectory_segments[0]);
    if (first_trj_node->link_index < 0) {
      LOG_ERR << "Detected invalid link index in trajectory";
      return (candidate_trj_index);
    }
    const GraphSection::GraphNode::Link* first_link =
        &(road_graph_link_storage_[first_trj_node->link_index].link);
    if (first_link->node_to.graph_section_index < 0) {
      LOG_ERR << "Detected invalid graph section index in trajectory";
      return (candidate_trj_index);
    }
    if (first_link->node_to.lane_section_index < 0) {
      LOG_ERR << "Detected invalid lane section index in trajectory.";
      return (candidate_trj_index);
    }
    if (first_link->node_to.node_index < 0) {
      LOG_ERR << "Detected invalid node index in trajectory";
      return (candidate_trj_index);
    }
    const GraphSection& dest_graph_sec =
        road_graph_[first_link->node_to.graph_section_index];
    const GraphSection::LaneSection& dest_lane_sec =
        dest_graph_sec.lane_sections[first_link->node_to.lane_section_index];
    const GraphSection::GraphNode& dest_node =
        dest_lane_sec.nodes[first_link->node_to.node_index];

    if (dest_node.lat_minor > 0) {
      // bypass to left
      *trj_req_type = TRJ_REQ_LKA_BYPASSING_LEFT;
      // printf("   ---> Request bypassing to left.\n");
    } else if (dest_node.lat_minor < 0) {
      // bypass to right
      *trj_req_type = TRJ_REQ_LKA_BYPASSING_RIGHT;
      // printf("   ---> Request bypassing to right.\n");
    } else {
      // driving in center line
    }

    // printf("    timers_.hold_bypassing.IsActive()=%d\n", timers_.hold_bypassing.IsActive());
    if (timers_.hold_bypassing.IsActive()) {
      if ((0 <= optimal_trj_index_.trj_idx_curr_lane_bypassing) &&
          (optimal_trj_index_.trj_idx_curr_lane_bypassing <
           candidate_trj_list_.Size())) {
        candidate_trj_index = optimal_trj_index_.trj_idx_curr_lane_bypassing;
        // printf("    optimal bypassing trj-idx=%d\n", candidate_trj_index);
      }
    }
  }
  /* k001 pengc 2022-04-10 (end) */

  return (candidate_trj_index);
}

bool TrajectoryPlanningStateLattice::CheckSafetyOfCandidateTrajectory(
    Int32_t candidate_trj_idx, plan_var_t ref_deceleration,
    bool* exist_uncertain_obj, Int32_t* unsafety_reason) {

  *exist_uncertain_obj = false;
  *unsafety_reason = REFUSE_CHANGINHG_LANE_REASON_NONE;

  if ((candidate_trj_idx < 0) ||
      (candidate_trj_idx >= candidate_trj_list_.Size())) {
    *unsafety_reason = REFUSE_CHANGINHG_LANE_REASON_INVALID_LANE;
#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
    std::cout << "    >>> 无有效的车道 (idx="
              << candidate_trj_idx << ")"
              << std::endl;
#endif
    return false;
  }

  const CandidateTrajectory& candidate_trj =
      candidate_trj_list_[candidate_trj_idx];

  if (candidate_trj.uncertain_list.Size() > 1) {
    /*此车道包含未知类型的障碍物(可能此车道是错误识别的）*/
    *exist_uncertain_obj = true;
  }

  if ( /*有障碍物需要减速*/
      (candidate_trj.collision_obj.deceleration < ref_deceleration)) {
    // 不安全的车道
    *unsafety_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
#if ENABLE_TRAJECTORY_PLANNING_STATE_LATTICE_TRACE
    std::cout << "    >>> 不安全的车道 (idx="
              << candidate_trj_idx << ")"
              << std::endl;
#endif
    /* D001 fuyuanyi 2023-07-04 (begin) */
    /* 根据系统强制换道值取消轨迹规划抑制换道 */
    /* D001 fuyuanyi 2023-07-04 (end) */
   if(!config_.enable_trajectory_planning_force_changing_lane){
     return false;

   }
  }

  return true;
}

Int32_t TrajectoryPlanningStateLattice::IsRefLineChanged() {
  // 0 ~ unchanged, 1 ~ changed to left, 2 ~ changed to right
  Int32_t ref_changed_flag = 0;

#if 0
  if (road_builed_type_ != prev_road_builed_type_) {
    if ((veh_proj_on_major_ref_line_.l - status_.prev_proj_on_ref.l) < 0.0F) {
      ref_changed_flag = 1;
    } else {
      ref_changed_flag = 2;
    }

    return (ref_changed_flag);
  }
#endif

  //获取车道宽度
  plan_var_t left_width = 0.0F;
  plan_var_t right_width = 0.0F;
  driving_map_->GetLaneWidth(curr_lane_index_, veh_proj_on_curr_lane_.s,
                             &left_width, &right_width);
  plan_var_t curr_l_ref = veh_proj_on_major_ref_line_.l;
  plan_var_t prev_l_ref = status_.prev_proj_on_ref.l;
  if ((prev_l_ref > 0.0F) && (curr_l_ref < 0.0F) &&
      ((prev_l_ref - curr_l_ref) > (left_width - 0.1F))) {
    // 参考线向左切换了, 使用之前的误差变化率
    ref_changed_flag = 1;
  } else if ((prev_l_ref < 0.0F) && (curr_l_ref > 0.0F) &&
             ((curr_l_ref - prev_l_ref) > (right_width - 0.1F))) {
    // 参考线向右切换了, 使用之前的误差变化率
    ref_changed_flag = 2;
  } else {
    // 参考线未发生切换
    ref_changed_flag = 0;
  }

  return (ref_changed_flag);
}

void TrajectoryPlanningStateLattice::UpdateInternalStatus(
    Int32_t candidate_trj_idx, Int32_t trj_req_type) {
  if (!driving_map_->IsValidLaneIndex(curr_lane_index_)) {
    status_.Clear();

    LOG_ERR << "Invalid current lane index.";
    return;
  }

  if ((candidate_trj_idx < 0) ||
      (candidate_trj_idx >= candidate_trj_list_.Size())) {
    status_.Clear();

    LOG_ERR << "Invalid candidate trajectory index.";
    return;
  }
  // 获取候选轨迹
  const CandidateTrajectory& candidate_trj =
      candidate_trj_list_[candidate_trj_idx];
  if (candidate_trj.trajectory_segments.Size() < 1) {
    status_.Clear();

    LOG_ERR << "Invalid candidate trajectory, because its size is less then 1.";
    return;
  }
  // 获取第一个节点
  const CandidateTrajectory::TrjNode& first_candidate_trj_node =
      candidate_trj.trajectory_segments[0];
  if (first_candidate_trj_node.link_index < 0) {
    status_.Clear();

    LOG_ERR << "Invalid candiate trajectory, "
               "because it contain invalid link index.";
    return;
  }
  // 获取第一节点的link
  const GraphSection::GraphNode::Link& first_candidate_trj_link =
      road_graph_link_storage_[first_candidate_trj_node.link_index].link;

  if (!driving_map_->IsValidReferenceLineIndex(
        first_candidate_trj_link.ref_line_index)) {
    status_.Clear();

    LOG_ERR << "Invalid candiate trajectory, "
               "because it contain invalid reference line index.";
    return;
  }

  if (first_candidate_trj_link.node_to.graph_section_index < 0) {
    status_.Clear();

    LOG_ERR << "Detected invalid graph section "
               "index in candidate trajectory.";
    return;
  }
  if (first_candidate_trj_link.node_to.lane_section_index < 0) {
    status_.Clear();

    LOG_ERR << "Detected invalid lane section "
               "index in candidate trajectory.";
    return;
  }
  if (first_candidate_trj_link.node_to.node_index < 0) {
    status_.Clear();

    LOG_ERR << "Detected invalid node "
               "index in candidate trajectory.";
    return;
  }
  const GraphSection& dest_graph_sec =
      road_graph_[first_candidate_trj_link.node_to.graph_section_index];
  const GraphSection::LaneSection& dest_lane_sec =
      dest_graph_sec.lane_sections[first_candidate_trj_link.node_to.lane_section_index];
  const GraphSection::GraphNode& dest_node =
      dest_lane_sec.nodes[first_candidate_trj_link.node_to.node_index];

  // 获取最优轨迹第一个link所在的参考线点集
  const common::Path& candidate_trj_ref_line =
      driving_map_->GetSmoothReferenceLine(
        first_candidate_trj_link.ref_line_index);

  /* k001 pengc 2022-04-10 (begin) */
  // 在一定的时间范围内，尽量保持之前的避让趋势
  Int32_t bypassing_hold_time = 3*1000;
  plan_var_t veh_v = common::Max(5.0F/3.6F, curr_position_.v);
  // 至少维持的距离
  const CandidateTrajectory& lka_trj =
      candidate_trj_list_[optimal_trj_index_.trj_idx_lka];

  plan_var_t bypassing_hold_dist = 15.0F;
  if (lka_trj.collision_obj.deceleration < -0.01) {
    bypassing_hold_dist = lka_trj.collision_obj.lon_dist_on_major_ref;
  }
  if (bypassing_hold_dist < 15.0F) {
    bypassing_hold_dist = 15.0F;
  }
  bypassing_hold_time = (bypassing_hold_dist / veh_v) * 1000;
  // 至少维持的时间
  if (bypassing_hold_time < 3*1000) {
    bypassing_hold_time = 3*1000;
  }

  /* longjiaoy 2024-04-18 (start) */
  if (TRJ_STATUS_INVALID == status_.trj_status) {
    if (0 == dest_lane_sec.lat_major) {
      status_.trj_status = TRJ_STATUS_LKA;
    }
  } else if ((TRJ_STATUS_LKA_BYPASSING_LEFT == status_.trj_status) ||
             (TRJ_STATUS_LKA_BYPASSING_RIGHT == status_.trj_status)) {
    if ((0 == dest_lane_sec.lat_major) &&
        (0 == dest_node.lat_minor)) {
      status_.trj_status = TRJ_STATUS_LKA;
    }
  } else {
    // nothing to do
  }
  /* longjiaoy 2024-04-18 (end) */
  if ((TRJ_STATUS_LKA == status_.trj_status) &&
      (0 == dest_lane_sec.lat_major)) {
    if (0 == dest_node.lat_minor) {
      // printf("   ###--> status lka, lat_offset=%0.2f\n", dest_node.lat_offset);
      // 保持车道中心线行驶
      action_smoothing_.req_bypassing_left.Smooth(false);
      action_smoothing_.req_bypassing_right.Smooth(false);
    } else if (dest_node.lat_minor > 0) {
      // 向左避让
      // printf("   --> status left, req_count=%d\n",
      //        action_smoothing_.req_bypassing_left.req_count_);
      if (action_smoothing_.req_bypassing_left.Smooth(true)) {
        status_.trj_status = TRJ_STATUS_LKA_BYPASSING_LEFT;

        if (TRJ_REQ_LKA_BYPASSING_LEFT == trj_req_type) {
          timers_.hold_bypassing.SetTimeout(bypassing_hold_time);
          timers_.hold_bypassing.SetUserDataFloat32(0, dest_node.lat_offset);
          timers_.hold_bypassing.Restart();
          // printf("    -----> Left Bypassing Request, timeout=%d, lat_offset=%0.2f\n",
          //        bypassing_hold_time, dest_node.lat_offset);
        }
      }
      action_smoothing_.req_bypassing_right.Smooth(false);
    } else {
      // 向右避让
      // printf("   --> status right, req_count=%d\n",
      //        action_smoothing_.req_bypassing_right.req_count_);
      if (action_smoothing_.req_bypassing_right.Smooth(true)) {
        status_.trj_status = TRJ_STATUS_LKA_BYPASSING_RIGHT;

        if (TRJ_REQ_LKA_BYPASSING_RIGHT == trj_req_type) {
          timers_.hold_bypassing.SetTimeout(bypassing_hold_time);
          timers_.hold_bypassing.SetUserDataFloat32(0, dest_node.lat_offset);
          timers_.hold_bypassing.Restart();

          // printf("    -----> Right Bypassing Request, timeout=%d, lat_offset=%0.2f\n",
          //        bypassing_hold_time, dest_node.lat_offset);
        }
      }
      action_smoothing_.req_bypassing_left.Smooth(false);
    }
  } else {
    action_smoothing_.req_bypassing_left.Smooth(false);
    action_smoothing_.req_bypassing_right.Smooth(false);
  }
  /* k001 pengc 2022-04-10 (end) */

  status_.prev_proj_on_ref = veh_proj_on_major_ref_line_;
  action_manager_.changing_lane_req.Clear();
  switch (status_.trj_status) {
  /* D001 fuyuanyi 2023-04-26 (begin) */
  /* 修改WBT index1 分支覆盖率未达标问题
     删除不会的分支，与defult处理逻辑一样 */
  /* D001 fuyuanyi 2023-04-26 (end) */
  /* case (TRJ_STATUS_INVALID):
    action_manager_.changing_lane_req.Clear();
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_NONE);
    break;
  */
  case (TRJ_STATUS_LKA):
  case (TRJ_STATUS_LKA_BYPASSING_LEFT):
  case (TRJ_STATUS_LKA_BYPASSING_RIGHT):
    action_manager_.changing_lane_req.Clear();
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_NONE);
    break;
  case (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I);
    break;
  case (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II);
    break;
  case (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I);
    break;
  case (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II);
    break;
  case (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I);
    break;
  case (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I);
    break;
  case (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II);
    break;
  case (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II);
    break;
  default:
    action_manager_.changing_lane_req.Clear();
    action_manager_.changing_lane_rsp.set_status(
          ChangingLaneRsp::STATUS_CHANGING_LANE_NONE);
    break;
  }
}

void TrajectoryPlanningStateLattice::ReqChangingLaneAutomatically(
    Int32_t curr_trj_idx) {
  result_of_planning_.changing_lane_req.set_request(
        ChangingLaneReq::REQ_CHANGING_LANE_NONE);

  if ((TRJ_STATUS_LKA != status_.trj_status) &&
      (TRJ_STATUS_LKA_BYPASSING_LEFT != status_.trj_status) &&
      (TRJ_STATUS_LKA_BYPASSING_RIGHT != status_.trj_status)) {
    // 只有处于车道内行驶时，才允许变道
    action_manager_.req_changing_lane_automatically.Clear();
    return;
  }

  if ((curr_trj_idx < 0) || (curr_trj_idx >= candidate_trj_list_.Size())) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Invalid candidate trajectory index.";
    return;
  }
  // 获取候选轨迹
  const CandidateTrajectory& curr_trj = candidate_trj_list_[curr_trj_idx];
  if (curr_trj.trajectory_segments.Size() < 1) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Invalid candidate trajectory, because its size is less then 1.";
    return;
  }
  // 获取第一个节点
  const CandidateTrajectory::TrjNode& curr_trj_node =
      curr_trj.trajectory_segments[0];
  if (curr_trj_node.link_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Invalid candiate trajectory, "
               "because it contain invalid link index.";
    return;
  }
  // 获取第一节点的link
  const GraphSection::GraphNode::Link& curr_trj_link =
      road_graph_link_storage_[curr_trj_node.link_index].link;

  if (curr_trj_link.node_to.graph_section_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Detected invalid graph section "
               "index in candidate trajectory.";
    return;
  }
  if (curr_trj_link.node_to.lane_section_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Detected invalid lane section "
               "index in candidate trajectory.";
    return;
  }
  if (curr_trj_link.node_to.node_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Detected invalid node "
               "index in candidate trajectory.";
    return;
  }
  const GraphSection& curr_dest_graph_sec =
      road_graph_[curr_trj_link.node_to.graph_section_index];
  const GraphSection::LaneSection& curr_dest_lane_sec =
      curr_dest_graph_sec.lane_sections[curr_trj_link.node_to.lane_section_index];
  const GraphSection::GraphNode& curr_dest_node =
      curr_dest_lane_sec.nodes[curr_trj_link.node_to.node_index];

  Int32_t global_trj_idx = optimal_trj_index_.trj_idx_global;
  if ((global_trj_idx < 0) ||
      (global_trj_idx >= candidate_trj_list_.Size())) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Invalid candidate trajectory index.";
    return;
  }
  // 获取候选轨迹
  const CandidateTrajectory& global_trj = candidate_trj_list_[global_trj_idx];
  if (global_trj.trajectory_segments.Size() < 1) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Invalid candidate trajectory, because its size is less then 1.";
    return;
  }
  // 获取第一个节点
  const CandidateTrajectory::TrjNode& global_trj_node =
      global_trj.trajectory_segments[0];
  if (global_trj_node.link_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Invalid candiate trajectory, "
               "because it contain invalid link index.";
    return;
  }
  // 获取第一节点的link
  const GraphSection::GraphNode::Link& global_trj_link =
      road_graph_link_storage_[global_trj_node.link_index].link;

  if (global_trj_link.node_to.graph_section_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Detected invalid graph section "
               "index in candidate trajectory.";
    return;
  }
  if (global_trj_link.node_to.lane_section_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Detected invalid lane section "
               "index in candidate trajectory.";
    return;
  }
  if (global_trj_link.node_to.node_index < 0) {
    action_manager_.req_changing_lane_automatically.Clear();
    LOG_ERR << "Detected invalid node "
               "index in candidate trajectory.";
    return;
  }
  const GraphSection& global_dest_graph_sec =
      road_graph_[global_trj_link.node_to.graph_section_index];
  const GraphSection::LaneSection& global_dest_lane_sec =
      global_dest_graph_sec.lane_sections[global_trj_link.node_to.lane_section_index];
  const GraphSection::GraphNode& global_dest_node =
      global_dest_lane_sec.nodes[global_trj_link.node_to.node_index];

  const static Int32_t max_lane_changing_req_cout = 5;
  if (global_dest_lane_sec.lat_major != curr_dest_lane_sec.lat_major) {
    if (global_dest_lane_sec.lat_major > 0) {
      // 向左请求变道
      if ((ChangingLaneReq::REQ_CHANGING_LANE_LEFT ==
           action_manager_.req_changing_lane_automatically.req.request()) &&
          result_of_action_planning_.changing_lane_req.allow_auto_changing_to_left()) {
        action_manager_.req_changing_lane_automatically.req_count++;
        if ((action_manager_.req_changing_lane_automatically.req_count >=
             max_lane_changing_req_cout)/* ||
            result_of_action_planning_.bypassing_req.allow_turning_left*/) {
          action_manager_.req_changing_lane_automatically.req_count = 0;

          result_of_planning_.changing_lane_req.set_request(
                ChangingLaneReq::REQ_CHANGING_LANE_LEFT);
          result_of_planning_.changing_lane_req.set_sequence(
                action_manager_.req_changing_lane_automatically.req.sequence());
        }
      } else {
        action_manager_.req_changing_lane_automatically.req_count = 0;
        action_manager_.req_changing_lane_automatically.req.set_request(
              ChangingLaneReq::REQ_CHANGING_LANE_LEFT);
        action_manager_.req_changing_lane_automatically.req.set_sequence(
              action_manager_.req_changing_lane_automatically.req.sequence()+1);
      }
    } else if (global_dest_lane_sec.lat_major < 0) {
      // 向右请求变道
      if ((ChangingLaneReq::REQ_CHANGING_LANE_RIGHT ==
           action_manager_.req_changing_lane_automatically.req.request()) &&
          result_of_action_planning_.changing_lane_req.allow_auto_changing_to_right()) {
        action_manager_.req_changing_lane_automatically.req_count++;
        if ((action_manager_.req_changing_lane_automatically.req_count >=
             max_lane_changing_req_cout)/* ||
            result_of_action_planning_.bypassing_req.allow_turning_right*/) {
          action_manager_.req_changing_lane_automatically.req_count = 0;

          result_of_planning_.changing_lane_req.set_request(
                ChangingLaneReq::REQ_CHANGING_LANE_RIGHT);
          result_of_planning_.changing_lane_req.set_sequence(
                action_manager_.req_changing_lane_automatically.req.sequence());
        }
      } else {
        action_manager_.req_changing_lane_automatically.req_count = 0;
        action_manager_.req_changing_lane_automatically.req.set_request(
              ChangingLaneReq::REQ_CHANGING_LANE_RIGHT);
        action_manager_.req_changing_lane_automatically.req.set_sequence(
              action_manager_.req_changing_lane_automatically.req.sequence()+1);
      }
    } else {
      action_manager_.req_changing_lane_automatically.Clear();
    }
  } else {
    action_manager_.req_changing_lane_automatically.Clear();
  }
}

/* k006 pengc 2023-01-06 (begin) */
// 更新轨迹坡度
void TrajectoryPlanningStateLattice::UpdateTrajectorySlope() {
  static const plan_var_t kSampleStep = 5.0F;

  result_of_planning_.target_trajectory_slope.Clear();

  plan_var_t sample_len =
      major_ref_line_->total_length() - veh_proj_on_major_ref_line_.s;
  if (sample_len > look_forward_length_) {
    sample_len = look_forward_length_;
  }

  Int32_t sample_num = common::com_floor(sample_len / kSampleStep);
  for (Int32_t i = 0; i < sample_num; ++i) {
    plan_var_t s = i*kSampleStep;

    Int32_t lane_idx = -1;
    plan_var_t s_on_lane = 0.0F;
    driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
          major_ref_line_index_, veh_proj_on_major_ref_line_.s + s,
          &lane_idx, &s_on_lane);
    if (!driving_map_->IsValidLaneIndex(lane_idx)) {
      LOG_ERR << "Invalid lane index.";
      continue;
    }

    TrajectoryPlanningResult::SlopeSamplePoint slople_sample;
    slople_sample.s = s;
    slople_sample.slope = driving_map_->GetLaneSlope(lane_idx, s_on_lane);

    result_of_planning_.target_trajectory_slope.PushBack(slople_sample);
  }
}
/* k006 pengc 2023-01-06 (begin) */

void TrajectoryPlanningStateLattice::AddEventReporting(
    Int32_t event_type, Int32_t param1) {
  ad_msg::EventReporting event;
  event.msg_head.valid = false;

  switch (event_type) {
  case (EVENT_TYPE_START_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_LEFT;
    break;
  case (EVENT_TYPE_START_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_RIGHT;
    break;
  case (EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_LEFT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_RIGHT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_ABORT_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_LEFT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_RIGHT;
    event.param[1] = param1;
    break;
  case (EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_LEFT;
    break;
  case (EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_RIGHT;
    break;
  case (EVENT_TYPE_REFUSE_ABORT_CHANGING_LANE):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP;
    event.priority = 2;
    event.lifetime = 2*1000;
    event.param[0] =
        ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_ABORT_CHANGING_LANE;
    event.param[1] = param1;
    break;
  default:
    break;
  }

  if (event.msg_head.valid) {
    event_reporting_list_.PushBack(event);
  }
}


} // namespace planning
} // namespace phoenix
