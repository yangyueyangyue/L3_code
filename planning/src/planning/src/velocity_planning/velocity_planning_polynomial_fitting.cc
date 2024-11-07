#include "velocity_planning_polynomial_fitting.h"

#include "curve/cubic_polynomial_curve1d.h"
#include "curve/quintic_polynomial_curve1d.h"
#include "geometry/geometry_utils.h"
#include "math/math_utils.h"
#include "math/matrix.h"
#include "pos_filter_wrapper.h"
#include "utils/fuzzy_pid.h"
#include "utils/log.h"
#include "utils/macros.h"
#include "vehicle_model_wrapper.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  #include "Sys_InfoCom.h"//系统接口
#endif
  #include "KTMPUStateOut.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE (0)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_PERFORMANCE_TEST (0)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TAR_VELOCITY_TRACE (0)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE (0)
#else
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE (1)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_PERFORMANCE_TEST (1)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TAR_VELOCITY_TRACE (0)
#define ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE (1)
#endif


#define ENABLE_UNCERTAIN_OBSTACLE (0)
#define ENABLE_SPEED_LIMIT_FROM_DRIVER_MAP (0)

#define TM_ZF (1)
#define TM_YD (0)

#define ECO_GEAR_ACC (0)

namespace phoenix {
namespace planning {

VelocityPlanningPolynomialFitting::VelocityPlanningPolynomialFitting() {
  driving_map_ = Nullptr_t;

  SetDefaultParam();

  driving_direction_ = DRIVING_DIRECTION_FORWARD;

  ClearHoldInfo();
}

VelocityPlanningPolynomialFitting::~VelocityPlanningPolynomialFitting() {
  // noting to do
}

/**
 * @brief 设置默认/更新模块参数，算法参数，车辆相关参数
 * 
 */
void VelocityPlanningPolynomialFitting::SetDefaultParam() {
  settings_.enable_acc = true;
  settings_.enable_aeb = true;
  settings_.enable_aeb_pre_dec = true;
  settings_.enable_following = true;
  settings_.enable_low_speed_following = true;
  settings_.enable_pacc = true;
  settings_.tar_v = 10.0F / 3.6F;
  settings_.tar_a = 1.0F;
  settings_.lateral_acceleration_limit = 1.0F;
  settings_.enable_stop_by_traffic_light = false;
  settings_.time_gap_setting = 3.0F;

  veh_model::VehicleModelWrapper veh_model;
  param_.vehicle_length = veh_model.GetVehicleLength();
  param_.vehicle_width = veh_model.GetVehicleWidth();
  param_.dist_of_localization_to_front = veh_model.GetDistOfLocalizationToFront();
  param_.dist_of_localization_to_rear = veh_model.GetDistOfLocalizationToRear();
  param_.dist_of_localization_to_center = veh_model.GetDistOfLocalizationToCenter();

  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0, 0), 0, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();

  param_.max_velocity_limit = 98.0F / 3.6F;  // 最大速度限制，由90——>98，为满足时效要求，提高最大速度限制上限。
  param_.max_acceleration_limit = 1.0F;       // 最大加速度限制
  param_.max_deceleration_limit = -7.0F;      // 最大减速度限制
  param_.tunnel_velocity_limit = 80.0F / 3.6F; // 隧道速度限制
  param_.ramp_velocity_limit = 50.0F / 3.6F;  // 匝道速度限制

  param_.go_target_velocity = 1.0;
  param_.go_target_acc = 0.5;
  param_.risk_perception = 0.6;

  param_.aeb_target_acc_I = -0.5;
  param_.aeb_target_detla_v = 10 / 3.6;
  param_.aeb_target_acc_II = -4.0;

  param_.obsacle_final_dec_ax_limit = -3.0;
  param_.obsacle_final_acc_ax_limit = 2.0;


  param_.velocity_limit_table_for_stop_situation.Clear();
  param_.velocity_limit_table_for_stop_situation.PushBack(common::LerpTableNodeType1(0.0F, 0.0F / 3.6F));
  param_.velocity_limit_table_for_stop_situation.PushBack(common::LerpTableNodeType1(1.0F, 3.0F / 3.6F));
  param_.velocity_limit_table_for_stop_situation.PushBack(common::LerpTableNodeType1(3.0F, 3.0F / 3.6F));
  param_.velocity_limit_table_for_stop_situation.PushBack(common::LerpTableNodeType1(10.0F, 5.0F / 3.6F));
  param_.velocity_limit_table_for_stop_situation.PushBack(common::LerpTableNodeType1(20.0F, 10.0F / 3.6F));

  param_.velocity_limit_table_for_stop_accurately_situation.Clear();
  param_.velocity_limit_table_for_stop_accurately_situation.PushBack(common::LerpTableNodeType1(0.0F, 0.0F / 3.6F));
  param_.velocity_limit_table_for_stop_accurately_situation.PushBack(common::LerpTableNodeType1(1.0F, 3.0F / 3.6F));
  param_.velocity_limit_table_for_stop_accurately_situation.PushBack(common::LerpTableNodeType1(2.0F, 3.0F / 3.6F));
  param_.velocity_limit_table_for_stop_accurately_situation.PushBack(common::LerpTableNodeType1(10.0F, 5.0F / 3.6F));
  param_.velocity_limit_table_for_stop_accurately_situation.PushBack(common::LerpTableNodeType1(20.0F, 10.0F / 3.6F));

  param_.sample_step_len_for_collision_test = 2.0F;

  // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度
  param_.relative_velocity_limit_table_by_dist_to_static_obj.Clear();
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(0.0F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(0.2F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(0.3F, 10.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(0.4F, 15.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(0.5F, 20.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(0.7F, 25.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(1.0F, 80.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(1.5F, 90.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_static_obj.PushBack(common::LerpTableNodeType1(2.0F, 100.0F / 3.6F));
  
  // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.Clear();
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(0.0F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(0.2F, 0.0F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(0.3F, 10.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(0.4F, 20.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(0.5F, 30.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(0.9F, 40.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(1.0F, 50.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(1.0F, 60.0F / 3.6F));
  param_.relative_velocity_limit_table_by_dist_to_dynamic_obj.PushBack(common::LerpTableNodeType1(2.0F, 70.0F / 3.6F));

  // AEB启动TTC时间
  param_.aeb_action_ttc_table.Clear();
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(0.0F / 3.6F, 1.0F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(20.0F / 3.6F, 1.0F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(30.0F / 3.6F, 1.2F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(40.0F / 3.6F, 1.2F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(50.0F / 3.6F, 1.8F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(60.0F / 3.6F, 1.8F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(70.0F / 3.6F, 2.0F));

// 根据弯道半径，限制本车最大过弯速度
  param_.curvatrue_limit_velocity_table.Clear();
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(150.0F, 60.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(200.0F, 60.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(250.0F, 60.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(300.0F, 70.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(350.0F, 80.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(400.0F, 85.0F /3.6F)); 
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(500.0F, 90.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(600.0F, 95.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(700.0F, 100.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(800.0F, 105.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(900.0F, 110.0F /3.6F));
  param_.curvatrue_limit_velocity_table.PushBack(common::LerpTableNodeType1(1000.0F, 115.0F /3.6F));  


  param_.safe_distance_to_static_obstacle = 8.0F;
  param_.safe_distance_for_low_speed_following = 5.0F;
  param_.safe_distance_to_dynamic_obstacle = 8.0F;
  param_.safe_time_to_dynamic_obstacle = 2.0F;
  param_.safe_distance_in_backing_mode = 2.0F;

  param_.jitter_suppressing_acce_delay_time = 15;
  param_.jitter_suppressing_dece_delay_time = 15;
  param_.jitter_suppressing_acce_req_counter_threshold = 5;
  param_.jitter_suppressing_dece_req_counter_threshold = 5;

  // AEB之前有个hold状态：障碍物持续5帧以下就消失-3m/s2减速度，否则-4m/s2减速度
  action_smoothing_aeb_action_.set_req_count_threshold(5);
  action_smoothing_aeb_action_.set_time_allowed(10);
  action_smoothing_aeb_warning_.set_req_count_threshold(2);
  action_smoothing_aeb_warning_.set_time_allowed(5);
  action_smoothing_notify_uncertain_.set_req_count_threshold(5);
  action_smoothing_notify_uncertain_.set_time_allowed(10);
}

void VelocityPlanningPolynomialFitting::UpdateVehicleParameter() {
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
      
  if (ad_msg::VEH_TRAILER_STATUS_CONNECTED == chassis_status_.trailer_status) {
    param_.vehicle_width =
        common::Max(veh_model.GetVehicleWidth(), veh_model.GetTrailerWidth());
  }
  
  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0, 0), 0, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();
  // 车辆宽度、挂车状态、质量载荷
  LOG_INFO(5) << "Vehicle Width = " << param_.vehicle_width
              << ", Trailer Status = " << chassis_status_.trailer_status
              << ", Vehicle Weight = " << chassis_status_.gross_weight;
}

void VelocityPlanningPolynomialFitting::Configurate(const VelocityPlanningConfig &conf) {
  settings_.enable_aeb_pre_dec = conf.enable_aeb_pre_dec;
  settings_.enable_stop_by_traffic_light = conf.enable_stop_by_traffic_light;

  param_.safe_distance_to_static_obstacle = conf.safe_dist_to_static_obj;
  if (param_.safe_distance_to_static_obstacle < 1.0F) {
    param_.safe_distance_to_static_obstacle = 1.0F;
  }
  param_.safe_distance_for_low_speed_following = conf.safe_dist_for_low_speed_following;
  if (param_.safe_distance_for_low_speed_following < 1.0F) {
    param_.safe_distance_for_low_speed_following = 1.0F;
  }
  param_.safe_distance_to_dynamic_obstacle = conf.safe_dist_to_dynamic_obj;
  if (param_.safe_distance_to_dynamic_obstacle < 1.0F) {
    param_.safe_distance_to_dynamic_obstacle = 1.0F;
  }
  param_.safe_distance_in_backing_mode = conf.safe_distance_in_backing_mode;
  if (param_.safe_distance_in_backing_mode < 1.0F) {
    param_.safe_distance_in_backing_mode = 1.0F;
  }
  param_.stop_accurately.enable = conf.stop_accurately.enable;
  param_.stop_accurately.braking_distance = conf.stop_accurately.braking_distance;
  if (param_.stop_accurately.braking_distance < 0.01F) {
    param_.stop_accurately.braking_distance = 0.01F;
  }
  param_.stop_accurately.proposed_decelaration = conf.stop_accurately.proposed_decelaration;
  if (param_.stop_accurately.proposed_decelaration < 0.5F) {
    param_.stop_accurately.proposed_decelaration = 0.5F;
  }

  settings_.lateral_acceleration_limit = conf.lat_acc_limit;
  if (settings_.lateral_acceleration_limit < 0.1F) {
    settings_.lateral_acceleration_limit = 0.1F;
  }
  if (settings_.lateral_acceleration_limit > 3.0F) {
    settings_.lateral_acceleration_limit = 3.0F;
  }
}

void VelocityPlanningPolynomialFitting::ClearHoldInfo() {
  action_smoothing_aeb_action_.Clear();
  action_smoothing_aeb_warning_.Clear();
  action_smoothing_notify_uncertain_.Clear();

  status_.Clear();

  pre_adas_mode_ = false;

  valid_target_velocity_array_ = false;

  common::com_memset(array_target_velocity_, 0, sizeof(array_target_velocity_));
  common::com_memset(array_target_acceleration_, 0, sizeof(array_target_acceleration_));

  event_reporting_list_.Clear();

  obstacle_decision_num_ = 0;
  obstacle_decision_list_.Clear();

  is_pacc_valid_ = false;
  
  is_pacc_valid_pre_ = false;

  pre_vel_plan_type_ = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
}

// TODO：速度规划主函数  应返回错误码，以便细致分析问题及诊断

bool VelocityPlanningPolynomialFitting::Plan(const VelocityPlanningDataSource &data_source) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "######## Velocity planning (Polynomial Fitting) (begin) >>>>>>>>";
#endif

  event_reporting_list_.Clear();
  // 初始化debug消息
  planning_debug_.Clear();

  result_of_planning_.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
  // 发生内部错误时，释放油门

  /* 开始一系列自检：
  1：判断data_source内的数据是否存在相关信息，若为空则记录LOG_ERR；
  2：清除规划信息；3：返回为false
  */
  if (Nullptr_t == data_source.driving_map) {
    LOG_ERR << "Invalid driving map.";
    ClearHoldInfo();
    return false;
  }
  if (Nullptr_t == data_source.result_of_action_planning) {
    LOG_ERR << "Invalid result of action planning.";
    ClearHoldInfo();
    return false;
  }
  
  if (Nullptr_t == data_source.special_chassis) {
    LOG_ERR << "Invalid special chassis status.";
    ClearHoldInfo();
    return false;
  }

  if (Nullptr_t == data_source.result_of_trajectory_planning) {
    LOG_ERR << "Invalid result of trajectory planning.";
    ClearHoldInfo();
    return false;
  }
  if (Nullptr_t == data_source.chassis) {
    LOG_ERR << "Invalid chassis status.";
    ClearHoldInfo();
    return false;
  }
  if (Nullptr_t == data_source.chassis_ctl_cmd) {
    LOG_ERR << "Invalid chassis control command.";
    ClearHoldInfo();
    return false;
  }
  if (Nullptr_t == data_source.traffic_light_list) {
    traffic_light_list_.Clear();
  }
  if (Nullptr_t == data_source.traffic_signal_list) {
    traffic_signal_list_.Clear();
  }


  // 1：获取底盘和车身信息，根据档位判断是前进还是倒车；
  // 2：获取行为规划结果（ACC/AEB）；
  // 3：将巡航车速限制到0~100KM/H，加速度限制到0~2m/s^2
  chassis_status_ = *(data_source.chassis);                   // 获取底盘状态
  special_chassis_status_ = *(data_source.special_chassis);
  chassis_ctl_cmd_ = *(data_source.chassis_ctl_cmd);          // 获取底盘特定型号车辆的信息
  traffic_light_list_ = *(data_source.traffic_light_list);    // 获取traffic_light_list
  traffic_signal_list_ = *(data_source.traffic_signal_list);  // 获取traffic_signal_list

#ifdef ENABLE_PACC_ADASIS
  adasisv2_horizon_ = data_source.adasisv2_horizon;
#endif

  /// 需要将目标轨迹同步到当前坐标Need to synchronize the target trajectory to current coordinate.
  curr_timestamp_ = data_source.timestamp;                                        // 获取时间戳
  driving_map_ = data_source.driving_map;                                         // 获取驾驶地图
  result_of_action_planning_ = *(data_source.result_of_action_planning);          // 获取行为规划的结果
  result_of_trajectory_planning_ = *(data_source.result_of_trajectory_planning);  // 获取轨迹规划的结果

  obstacle_decision_list_.Clear();
  obstacle_decision_num_ = driving_map_->GetObstacleList().obstacle_num;

#if PLANNING_DEBUG
  planning_debug_.timestamp = data_source.timestamp;
  planning_debug_.planning_pos.path_point.point.set_x(result_of_trajectory_planning_.curr_pos.x);
  planning_debug_.planning_pos.path_point.point.set_y(result_of_trajectory_planning_.curr_pos.y);

  planning_debug_.planning_pos.path_point.heading = result_of_trajectory_planning_.curr_pos.heading;
  planning_debug_.planning_pos.path_point.curvature = result_of_trajectory_planning_.curr_pos.curvature;
  planning_debug_.planning_pos.path_point.s = result_of_trajectory_planning_.curr_pos.s;
  planning_debug_.planning_pos.path_point.l = result_of_trajectory_planning_.curr_pos.l;
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "驾驶地图障碍物个数, num=" << obstacle_decision_num_;
#endif
  for (Int32_t i = 0; i < obstacle_decision_num_; ++i) {
    const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(i);
    ObstacleDecision obstacle;
    obstacle.perception_obstacle = tar_obj;
    obstacle_decision_list_.PushBack(obstacle);
  }  

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  if (special_chassis_status_.dfcv_d17.ADCU_Mode_Sts == ENGAGE_MODE ||
      special_chassis_status_.dfcv_d17.ADCU_Mode_Sts == EMERGENCY_MODE_PBCR_CTL) {
      adas_mode_ = true;
  } else {
      adas_mode_ = false;
  }
#else
  if (ad_msg::VEH_DRIVING_MODE_ROBOTIC == chassis_status_.driving_mode) {
    adas_mode_ = true;
  } else {
    adas_mode_ = false;
  }
#endif

#if PLANNING_DEBUG
  planning_debug_.adas_enable_pre = pre_adas_mode_;
  planning_debug_.adas_enable = adas_mode_;
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "adas_mode_ = " << adas_mode_;
#endif
  // 获取当前控制下发的油门量
  control_acc_pedal_ = chassis_ctl_cmd_.acc_value;

  // 获取当前坡度 和 坡度轨迹的最后一个点的坡度（单位：%）
  if (!result_of_trajectory_planning_.target_trajectory_slope.Empty()) {
    slope_flag_ = true;
    curr_slope_ = result_of_trajectory_planning_.target_trajectory_slope.Front().slope;
  } else {
    slope_flag_ = false;
    curr_slope_= 0.0F;
  }

  // 获取底盘当前挡位（前进档和后退档）
  if (ad_msg::VEH_GEAR_R == data_source.chassis->gear) {
    driving_direction_ = DRIVING_DIRECTION_BACKWARD;
  } else {
    driving_direction_ = DRIVING_DIRECTION_FORWARD;
  }

  settings_.enable_acc = result_of_action_planning_.enable_acc;

  //AEB：在ADCU上需要通过标定方式关闭AEB功能
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  uint8 sys_cal_aebs = 0;
  sys_get_cal_aebs(&sys_cal_aebs);
  if (1 == sys_cal_aebs) {
    settings_.enable_aeb = false;
  } else {
    settings_.enable_aeb = result_of_action_planning_.enable_aeb;
  }
#else
  settings_.enable_aeb = result_of_action_planning_.enable_aeb;
#endif
 //PCC：在ADCU上需要通过标定方式关闭PCC功能
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  settings_.enable_pacc = sys_get_planning_pacc_switch() & 0x01; // ADCU默认关闭PCC
#else
  settings_.enable_pacc = result_of_action_planning_.enable_pcc;
#endif

  settings_.tar_v = result_of_action_planning_.v_setting;
  
 //CC：在ADCU上需要通过标定方式关闭CC功能
#if PLANNING_DEBUG
  planning_debug_.user_setting_v = result_of_action_planning_.v_setting;
#endif
  if (settings_.tar_v < 0.0F) {
    settings_.tar_v = 0.0F;
  }
  if (settings_.tar_v > param_.max_velocity_limit) {
    settings_.tar_v = param_.max_velocity_limit;
  }
  settings_.tar_a = result_of_action_planning_.a_setting;
  if (settings_.tar_a < 0.0F) {
    settings_.tar_a = 0.0F;
  }
  if (settings_.tar_a > 2.0F) {
    settings_.tar_a = 2.0F;
  }

//时距：在ADCU上需要通过标定方式关闭时距功能
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  settings_.time_gap_setting = get_acc_level() ; // ADCU
#else
  settings_.time_gap_setting = result_of_action_planning_.time_gap_setting;
#endif

  if (settings_.time_gap_setting < 2.0F) {
    settings_.time_gap_setting = 2.0F;
  } else if (settings_.time_gap_setting > 4.0F) {
    settings_.time_gap_setting = 4.0F;
  } else {
    // do nothing
  }
  if (abs(settings_.time_gap_setting - pre_time_gap_setting_) > 1.0F) {
    settings_.time_gap_setting = pre_time_gap_setting_;
  }
#if PLANNING_DEBUG
  // planning_debug_.time_gap_setting = result_of_action_planning_.time_gap_setting;
#endif

  //  退出AEB机制：根据是否有油门踏板及驾驶模式设置来抑制AEB
  if ((chassis_status_.acc_pedal_value > 5) || (ad_msg::VEH_DRIVING_MODE_MANUAL == chassis_status_.driving_mode) ||
      (ad_msg::VEH_DRIVING_MODE_INVALID == chassis_status_.driving_mode)) {
    status_.aeb_hold_flag = false;
    action_smoothing_aeb_action_.Clear();
  }

  ad_msg::RelativePos current_rel_pos;
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          data_source.timestamp, driving_map_->GetRelativePosList(), &current_rel_pos)) {
    LOG_ERR << "Failed to get current posistion from relative position list.";
    ClearHoldInfo();
    return false;
  }

  UpdateVehicleParameter();
  
  veh_velocity_for_planning_ = common::com_abs(current_rel_pos.v);
  if (veh_velocity_for_planning_ < 5.0F / 3.6F) {
    veh_velocity_for_planning_ = 5.0F / 3.6F;
  }

  // 更新当前位姿信息（TrajectoryPoint）
  curr_position_.path_point.point.set_x(current_rel_pos.x);
  curr_position_.path_point.point.set_y(current_rel_pos.y);
  curr_position_.path_point.heading = current_rel_pos.heading;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    curr_position_.path_point.heading = common::NormalizeAngle(curr_position_.path_point.heading + COM_PI);
  }
  curr_position_.path_point.curvature = current_rel_pos.yaw_rate / veh_velocity_for_planning_;  // k=w/v
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    curr_position_.path_point.curvature = -curr_position_.path_point.curvature;
  }
  curr_position_.path_point.l = 0.0F;
  curr_position_.path_point.s = 0.0F;
  curr_position_.v = common::com_abs(current_rel_pos.v);
  curr_position_.yaw_rate = current_rel_pos.yaw_rate;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    curr_position_.yaw_rate = -curr_position_.yaw_rate;
  }
  curr_position_.relative_time = 0.0F;

  // 规划的轨迹点判断小于2
  Int32_t tar_trj_point_num = result_of_trajectory_planning_.target_trajectory.Size();
  if (tar_trj_point_num < 2) {
    LOG_ERR << "There are not enough points in target trajectory.";
    ClearHoldInfo();
    return false;
  }

#if PLANNING_DEBUG
  planning_debug_.current_plan_v = curr_position_.v;
  planning_debug_.planning_pos.v = chassis_status_.v;
  planning_debug_.planning_pos.a = chassis_status_.a;
  planning_debug_.planning_pos.yaw_rate = chassis_status_.yaw_rate;
  planning_debug_.planning_pos.relative_time = current_rel_pos.relative_time;
  planning_debug_.planning_path_point_num = tar_trj_point_num;
#endif

  ad_msg::RelativePos tar_path_rel_pos;
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          data_source.result_of_trajectory_planning->msg_head.timestamp, driving_map_->GetRelativePosList(),
          &tar_path_rel_pos)) {
    LOG_ERR << "Failed to get target relative posistion in "
               "relative position list.";
    ClearHoldInfo();
    return false;
  }

  common::Matrix<plan_var_t, 2, 1> rotate_center;
  common::Matrix<plan_var_t, 3, 3> mat_conv;
  common::Matrix<plan_var_t, 2, 1> point_conv;
  rotate_center.SetZeros();
  mat_conv.SetIdentity();
  common::Rotate_2D(rotate_center, tar_path_rel_pos.heading, &mat_conv);
  common::Translate_2D(tar_path_rel_pos.x, tar_path_rel_pos.y, &mat_conv);
  for (Int32_t i = 0; i < tar_trj_point_num; ++i) {
    common::PathPoint &p = result_of_trajectory_planning_.target_trajectory[i];
    point_conv(0) = p.point.x();
    point_conv(1) = p.point.y();
    common::TransformVert_2D(mat_conv, &point_conv);
    p.point.set_x(point_conv(0));
    p.point.set_y(point_conv(1));
    p.heading = common::NormalizeAngle(p.heading + tar_path_rel_pos.heading);

#if PLANNING_DEBUG
    planning_debug_.planning_path.PushBack(p);
#endif
  }

  /* k001 pengc 2022-04-13 (begin) */
  // 优化变道时减速的问题
  Int32_t changed_trajectory_size = result_of_trajectory_planning_.trj_changing.selected_trajectory.Size();
  for (Int32_t i = 0; i < changed_trajectory_size; ++i) {
    common::PathPoint &p = result_of_trajectory_planning_.trj_changing.selected_trajectory[i];
    point_conv(0) = p.point.x();
    point_conv(1) = p.point.y();
    common::TransformVert_2D(mat_conv, &point_conv);
    p.point.set_x(point_conv(0));
    p.point.set_y(point_conv(1));
    p.heading = common::NormalizeAngle(p.heading + tar_path_rel_pos.heading);
  }
  changed_trajectory_.Construct(result_of_trajectory_planning_.trj_changing.selected_trajectory);
  changed_trajectory_size = changed_trajectory_.points().Size();
  /* k001 pengc 2022-04-13 (end) */

  // Check if reference line is valid
  if (driving_map_->GetReferenceLinesNum() < 1) {
    LOG_ERR << "There are no reference line in driving map.";
    ClearHoldInfo();
    return false;
  }

  target_trajectory_.Construct(result_of_trajectory_planning_.target_trajectory);
  if (target_trajectory_.points().Size() < 2) {
    LOG_ERR << "Invalid path of trajectory planning.";
    ClearHoldInfo();
    return false;
  }

  // 获取当前参考线，将车辆位姿投影在参考线上，计算参考线剩余长度
  Int32_t major_ref_line_index = driving_map_->GetMajorReferenceLineIndex();
  if (!driving_map_->IsValidReferenceLineIndex(major_ref_line_index)) {
    LOG_ERR << "Invalid reference line index " << major_ref_line_index;
    ClearHoldInfo();
    return false;
  }
  const common::Path &major_ref_line = driving_map_->GetSmoothReferenceLine(major_ref_line_index);
  const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum>
      &major_ref_line_curvature_info = driving_map_->GetSmoothReferenceLineCurveInfo(major_ref_line_index);
  veh_proj_on_major_ref_line_ = driving_map_->GetProjPointOnMajorRefLine();
  plan_var_t major_ref_line_forward_length = major_ref_line.total_length() - veh_proj_on_major_ref_line_.s;

  Int32_t driving_map_type = driving_map_->GetDrivingMapType();

  target_trajectory_curvature_info_.Clear();
  target_trajectory_.AnalyzeCurvature(&target_trajectory_curvature_info_);
  target_trajectory_.FindProjection(curr_position_.path_point.point, &veh_proj_on_target_trajectory_);

#if PLANNING_DEBUG
  planning_debug_.major_ref_line_point_num = internal_data_.sample_points.Size();
  for (size_t i = 0; i < internal_data_.sample_points.Size(); i++) {
    planning_debug_.major_ref_line.PushBack(internal_data_.sample_points[i]);
  }
#endif

  // 根据场景计算目标速度
  tar_v_info_list_.Clear();
  TargetVelocityInfo tar_v_info;

  // 新增根据道路规则 Planning
  TargetVelocityInfo pcc_tar_v_info;
  TargetVelocityInfo cc_tar_v_info;
  TargetVelocityUpperLimit cc_upper_limit;
  
  // A：根据道路约束计算巡航速度上限
  CalcTargetVelocityAccordingToRoadRules(&cc_upper_limit);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CC] Final CC V upper = " << cc_upper_limit.tar_v << "m/s(" << cc_upper_limit.tar_v * 3.6 << "km/h)";
#endif  

  bool cc_is_go = VelocityPlanningIsGoState(cc_upper_limit.tar_v);//定义起步状态
  cc_upper_limit_v_ = cc_upper_limit.tar_v;
#if PLANNING_DEBUG
  planning_debug_.upper_limit_v_final = cc_upper_limit_v_;
#endif

  // 根据CC目标速度规划下一帧速度及加速度
  if (cc_is_go) {//起步状态
    cc_tar_v_info.tar_type = cc_upper_limit.tar_type;
    cc_tar_v_info.tar_v = param_.go_target_velocity;
    cc_tar_v_info.tar_a = param_.go_target_acc;
  } else {
    PlanVelocityAccordingToRoadRules(cc_upper_limit, &cc_tar_v_info);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Planning][CC] tar_type = " << cc_tar_v_info.tar_type << ", tar_v = " << cc_tar_v_info.tar_v << "m/s(" << cc_tar_v_info.tar_v * 3.6 << "km/h), tar_a = " << cc_tar_v_info.tar_a << "m/s^2";
#endif
  }
#if PLANNING_DEBUG
  planning_debug_.tar_v_cruise = cc_tar_v_info.tar_v;
  planning_debug_.tar_a_cruise = cc_tar_v_info.tar_a;
#endif

  // PCC使能则开启PCC计算，而不是在CC状态才执行，以便于在跟车时也能分析PCC数据
  if (settings_.enable_pacc) {
    // 先构建PCC地图,从而给基于坡度的阶梯平滑提供当前坡度值
    is_pacc_valid_ = CheckPACCIfValid();

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    /*FIXME：不同速度源下，PCC逻辑
    * @brief 1.只在ADECU端使能此逻辑。
    *        2.当速度源为全局车速时，PCC正常使能，且采用梯度速度过渡。
    *        3.当速度源为用户设定速度时，当用户设定为减速时，则不使能PCC，否则正常使能。
    *        4.当速度源为默认速度时，PCC正常使能，且用户调速结束时，将速度源置为默认车速。
    *        尤其是全局速度分段下发的时候，前后段差异过大的情况，同时，也可适用于用户调节车速
    *        过于剧烈的情况
    * @author denghuix 
    * @date   2024-05-20 ❥(^_-)
    */
    // 最终使用速度源判断
    uint8_t sys_vel_src = sys_get_speed_src();
    uint8_t sys_vel_src = 1;
    PCC_VEL_SOURCE tem_vel_src;
    pcc_mode_change_.SetVelSource(PCC_VEL_SOURCE(sys_vel_src)); 
    switch (sys_vel_src){
    case 0:
      tem_vel_src = PCC_VEL_SOURCE::DEFAULT;
      break;
    case 1:
      tem_vel_src = PCC_VEL_SOURCE::USER_SETTING;
      break;
    case 2:
      tem_vel_src = PCC_VEL_SOURCE::GLOBAL_VEL;
      break;
    case 3:
      tem_vel_src = PCC_VEL_SOURCE::LANE_LIMIT;
      break;
    default:
      tem_vel_src = PCC_VEL_SOURCE::ERROR;
      break;
    }
    if (tem_vel_src == PCC_VEL_SOURCE::ERROR) {
      pcc_mode_change_.SetVelSource(PCC_VEL_SOURCE::DEFAULT);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[Planning][PCC_MODE] SYSTEM VELOCITY SRC ERROR !!! ";
#endif
    } else {
      pcc_mode_change_.SetVelSource(tem_vel_src);  
    }
    if (!(pcc_mode_change_.GetVelSource() == PCC_VEL_SOURCE::USER_SETTING)){ // 保证非用户设定速度源时，用户设定速度数组始终为空
      pcc_mode_change_.SetDecCount(0);
      pcc_mode_change_.ClearVelList();
    }
    // @todo 将不同速度源逻辑处理封装成函数，优化代码
    if ((pcc_mode_change_.GetVelSource() == PCC_VEL_SOURCE::GLOBAL_VEL) && (common::com_abs(cc_upper_limit_v_ - settings_.tar_v < 0.1F)) && is_pacc_valid_) { // 全局车速处理
      static plan_var_t cc_upper_limit_v_pre = cc_upper_limit_v_;

      if (common::com_abs(cc_upper_limit_v_ - cc_upper_limit_v_pre) >= 9.0 / 3.6) {//PCC前后传入的速度差异大于13km/h，将退出PCC模式，因此保守设定速度差超过9km/h将进行平滑。
        if ((cc_upper_limit_v_pre - cc_upper_limit_v_) >= 0.0) {//以每周期0.05km/h的△进行平滑
          cc_upper_limit_v_pre = cc_upper_limit_v_pre - 0.04 / 3.6; //0.05为当前调参值，在保证周期内能平滑到目标速度差范围内的前提下，可调整参数以调整减速速率
        } else {
          cc_upper_limit_v_pre = cc_upper_limit_v_pre + 1.0 / 3.6; //正值差异不会导致退出PCC，为避免加速过慢,加快平滑速率。
        }
        cc_upper_limit_v_ = cc_upper_limit_v_pre;
      } else {
        cc_upper_limit_v_pre = cc_upper_limit_v_;
      }
    } else if ((pcc_mode_change_.GetVelSource() == PCC_VEL_SOURCE::USER_SETTING) && (common::com_abs(cc_upper_limit_v_ - settings_.tar_v < 0.1F)) && is_pacc_valid_) { // 人工设置车速处理
      if (pcc_mode_change_.GetPreVelState()) { // 上一时刻是用户设置的车速
        if (settings_.tar_v < pcc_mode_change_.GetVelListBackElement()) { // 用户设置车速减小
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[Planning][PCC_MODE] 人工调速减小 ";
#endif
          pcc_mode_change_.SetDecState(true);
          pcc_mode_change_.AddElementToVelList(settings_.tar_v);
          is_pacc_valid_ = false; // 退出PCC, 启用CC进行制动降速
        } else if (common::com_abs(settings_.tar_v - pcc_mode_change_.GetVelListBackElement()) < 0.1F) { // 用户调速结束
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[Planning][PCC_MODE] 人工调速结束 ";
#endif
          if (pcc_mode_change_.GetDecState()) { // 只在用户设定车速为减速模式下才启用CC制动
                  if (chassis_status_.v- settings_.tar_v > 3.0F / 3.6F) {
                  is_pacc_valid_ = false;
                  } else {
                    pcc_mode_change_.StartDecCount();
                    if (pcc_mode_change_.GetDecCount() > pcc_mode_change_.GetMaxDecCount()) {
                      is_pacc_valid_ = true;
                      pcc_mode_change_.SetDecState(false);
                      pcc_mode_change_.SetDecCount(0);
                      pcc_mode_change_.ClearVelList();
                      pcc_mode_change_.SetVelSource(PCC_VEL_SOURCE::DEFAULT);
                    } else {
                      is_pacc_valid_ = false;
                    }
                  }
          } else {
            is_pacc_valid_ = true;
          }
        } else { // 用户设置速度增加
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[Planning][PCC_MODE] 人工调速增加 ";
#endif
          if (pcc_mode_change_.GetDecState()) { // 应对用户设置自车降速，自车速度未达到设置速度时，用户又突然增加的case，即用户上下摇摆按键调速情况 >_< !!!
            double pre_uesr_seetting_v = pcc_mode_change_.GetVelListBackElement();
            pcc_mode_change_.ClearVelList();
            pcc_mode_change_.AddElementToVelList(pre_uesr_seetting_v);
            pcc_mode_change_.AddElementToVelList(settings_.tar_v); 
            pcc_mode_change_.SetDecState(false);
          } else {
            pcc_mode_change_.AddElementToVelList(settings_.tar_v);   
          }
        }
      } else { // 上一时刻不是用户设置的车速
        pcc_mode_change_.AddElementToVelList(settings_.tar_v);
        pcc_mode_change_.SetPreVelState(true);
      }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Planning][PCC_MODE] 人工调速时,PCC状态 = " << is_pacc_valid_ ;
#endif
    } else { // 其他速度源车速不做处理
      pcc_mode_change_.SetPreVelState(false);
    }
#endif
    plan_var_t target_cc_v = cc_upper_limit_v_; // 全局速度阶梯后的巡航速度
#if PLANNING_DEBUG
  planning_debug_.pacc.cc_upper_limit_v = target_cc_v;
#endif

#if ENABLE_PCC_STEP_TARGET_CC_SPEED
    /**
     * @brief 分段阶梯调整目标巡航速度，可缓解自车低速到高速巡航过程中加速过快的问题，在如下几个场景验证
     * 1. AD开启时，起步从低速到高速巡航
     * 2. 从低速跟车切换到高速巡航
     * 3. 上坡低速到坡顶到高速巡航
     * @todo 如果PACC使能开关有更改，或者考虑模式频繁切换等工况，速度记录数组需要重置。
     */
    static plan_var_t record_chassis_v[15] = {0};
    for (size_t i = 0; i < 14; i++) {
      record_chassis_v[i] = record_chassis_v[i + 1];
    }
    record_chassis_v[14] = chassis_status_.v;

    if (is_pacc_valid_ && (pacc_map_.points.Size() > 2)) { // PCC地图有效
      float cur_slope = pacc_map_.points[0].slope * 100.0F;

      TargetVelAndEgoVelDeltaSmooth(target_cc_v, cur_slope);
      if (record_chassis_v[0] > record_chassis_v[14]) { // 判断是否在持续掉速
        target_cc_v = cc_upper_limit_v_;
      }

      if (target_cc_v > cc_upper_limit_v_) {
        target_cc_v = cc_upper_limit_v_;
      }
    }
#endif

    PlanVelocityAccordingToPredictiveControl(target_cc_v, pcc_tar_v_info);
    //FIXME: PCC有效，使用PCC速度规划结果替代CC
    // 考虑到PCC当前只支持驱动及滑行，不支持制动，需做数据分析后再替换
    /*
    if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != pcc_tar_v_info.tar_type) {
      cc_tar_v_info = pcc_tar_v_info;
    }
    */
  }

  // B：根据虚拟障碍物virtual_obs计算的规划速度，将计算出的结果存到列表中
  TargetVelocityInfo virtual_obs_tar_v_info;  
  PlanVelocityAccordingToTrafficRules(major_ref_line, veh_proj_on_major_ref_line_, &virtual_obs_tar_v_info);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != virtual_obs_tar_v_info.tar_type) {
    LOG_INFO(5) << "[Virtual Obstacles] tar_type = " << virtual_obs_tar_v_info.tar_type << ", tar_v = " << virtual_obs_tar_v_info.tar_v << "m/s(" << virtual_obs_tar_v_info.tar_v * 3.6 << "km/h), tar_a = " << virtual_obs_tar_v_info.tar_a << "m/s^2";
}
#endif

 //TODO C：虚拟障碍物（弯道曲率）
 /*弯道可以看做是区间限速： 先平缓减速到限速值，然后匀速行驶，最后加速。
   当前过弯道经常出现点刹，因为基于相机感知是相对位置，无法得到区间的稳定，可以尝试预瞄点跟踪的方式(前提是相机检测的曲率平滑)*/
#if 0
  PlanVelocityAccordingToPathCurvature(major_ref_line, major_ref_line_curvature_info, veh_proj_on_major_ref_line_, 0.0F,
                                       major_ref_line.total_length(), &tar_v_info);

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
  
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[curvature][refline] tar_type = " << tar_v_info.tar_type
            << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" ;
#endif

  // Plan velocity according to curvature of planned trajectory：输入是 target_trajectory_
  // when curvature path comes from LCA, DON'T decelerate
  if ((TRJ_STATUS_LKA == result_of_trajectory_planning_.trj_status) || 
      (TRJ_STATUS_LKA_BYPASSING_LEFT == result_of_trajectory_planning_.trj_status) ||
      (TRJ_STATUS_LKA_BYPASSING_RIGHT == result_of_trajectory_planning_.trj_status)) {
        PlanVelocityAccordingToPathCurvature(target_trajectory_, target_trajectory_curvature_info_,
                                            veh_proj_on_target_trajectory_, 0.0F, target_trajectory_.total_length(),
                                            &tar_v_info);
        if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
          tar_v_info_list_.PushBack(tar_v_info);
        }

        #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[curvature][pln-path] tar_type = " << tar_v_info.tar_type
                  << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2";
        #endif
      } 

#if PLANNING_DEBUG
  planning_debug_.tar_type_curvature = tar_v_info.tar_type;
  planning_debug_.tar_v_curvature = tar_v_info.tar_v;
  planning_debug_.tar_a_curvature = tar_v_info.tar_a;
#endif

#endif

  // D：根据实际障碍物real_obs计算的规划速度，将计算出的结果存到列表中(已经做了筛选)
  TargetVelocityInfo real_obs_tar_v_info;  
  PlanVelocityAccordingToObstacles(&real_obs_tar_v_info);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != real_obs_tar_v_info.tar_type) {
    LOG_INFO(5) << "[Real Obstacles] tar_type = " << real_obs_tar_v_info.tar_type << ", tar_v = " << real_obs_tar_v_info.tar_v << "m/s(" << real_obs_tar_v_info.tar_v * 3.6 << "km/h), tar_a = " << real_obs_tar_v_info.tar_a << "m/s^2";
  }
#endif

  // 区分虚拟障碍物和真实障碍物
  TargetVelocityInfo obs_tar_v_info;
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != real_obs_tar_v_info.tar_type && 
      VELOCITY_PLANNING_TARGET_TYPE_INVALID != virtual_obs_tar_v_info.tar_type) {
    if (real_obs_tar_v_info.tar_v <=  virtual_obs_tar_v_info.tar_v) {
      obs_tar_v_info = real_obs_tar_v_info;
    } else {
      obs_tar_v_info = virtual_obs_tar_v_info;
    }    
  } else if(VELOCITY_PLANNING_TARGET_TYPE_INVALID != real_obs_tar_v_info.tar_type) {
    obs_tar_v_info = real_obs_tar_v_info;
  } else if(VELOCITY_PLANNING_TARGET_TYPE_INVALID != virtual_obs_tar_v_info.tar_type) {
    obs_tar_v_info = virtual_obs_tar_v_info;
  } else {
     obs_tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
  }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != obs_tar_v_info.tar_type) {
    LOG_INFO(5) << "[Planning][Obstacles] tar_type = " << obs_tar_v_info.tar_type << ", tar_v = " << obs_tar_v_info.tar_v << "m/s(" << obs_tar_v_info.tar_v * 3.6 << "km/h), tar_a = " << obs_tar_v_info.tar_a << "m/s^2";
  }
#endif

  // 初始化设置：以巡航计算出的结果作为速度规划输出的初始值 
  result_of_planning_.Clear();
  result_of_planning_.msg_head = driving_map_->GetRelativePosList().msg_head;
  result_of_planning_.tar_type = cc_tar_v_info.tar_type;
  result_of_planning_.tar_v = cc_tar_v_info.tar_v;
  result_of_planning_.tar_a = cc_tar_v_info.tar_a;

  // 初始化设置：是否释放油门（滑行）
  if ((driv_map::DRIVING_MAP_TYPE_PRED_PATH == driving_map_type) ||
      (driv_map::DRIVING_MAP_TYPE_INVALID == driving_map_type)) {
    result_of_planning_.release_throttle = true;//释放油门（滑行）：条件1
  } else {
    result_of_planning_.release_throttle = cc_tar_v_info.release_throttle;
  }
  // 初始化设置：油门开度值
  result_of_planning_.tar_throttle = cc_tar_v_info.tar_throttle;

  /**
   * @brief 根据各场景规划的速度和加速度，再精细化速度决策
   * 
   */
  bool keep_cruise_v = true;
  bool overtake_release_throttle = false;
  bool is_need_dec = false;
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != obs_tar_v_info.tar_type ) {
    if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == obs_tar_v_info.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == obs_tar_v_info.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == obs_tar_v_info.tar_type) { // 真实障碍物
      // TODO 巡航，跟车，避障需要更精细决策
      if ((obs_tar_v_info.obstacle_info.obj_v - chassis_status_.v) > 3.6 / 3.6) {
        if (obs_tar_v_info.obstacle_info.dist_to_obj > 6.0f) {
          keep_cruise_v = true;
        } else {
          keep_cruise_v = false;
          overtake_release_throttle = true;
        }  
      } else {
        if ((obs_tar_v_info.obstacle_info.obj_v - cc_upper_limit_v_) > 3.6 / 3.6) {
          keep_cruise_v = true;
        } else {
          keep_cruise_v = false;
        }
      }
    } else { // 虚拟障碍物场景(接近隧道、接近匝道...)，优先减速
      keep_cruise_v = false;
    }

    if (obs_tar_v_info.tar_a < result_of_planning_.tar_a) {
      // 可能出现跟加的 加速度 小于 巡航出的加速 是否要 切跟车
      // 但是 跟车的速度 又比 巡航决策出速度大
      if (obs_tar_v_info.tar_a < 0 && 
          (obs_tar_v_info.obstacle_info.obj_v < (chassis_status_.v + 6.0 / 3.6)) &&  //（阈值过小容易不满足is_need_dec，过大容易有车高速切入就刹车）cc_upper_limit_v_(objv < setv)
          (obs_tar_v_info.tar_v < (chassis_status_.v + 3.6 / 3.6))) { //cc_upper_limit_v_(objv < setv)
          //典型问题：setv=70 < objv=75 < egov=80，以前CC控制跟随误差不大（setv=egov），
          //但PCC有速度波动（即便setv=70，egov大概率不会等于setv，永远有速度差），因此不能objv<setv,必须 objv < egov
        is_need_dec = true;
      }
    } else { 
      // obs_tar_v_info.tar_a > result_of_planning_.tar_a
      // 出现 前车速度比自车速度大的情况
      // 出现 计算的目标车速 小于 自车车速 同时 目标加速度 大于 0 
      // 同时 obs_tar_v_info.tar_a(>0) > result_of_planning_.tar_a(<0)
      // 同时 obs_tar_v_info.tar_v < result_of_planning_.tar_v
      if (obs_tar_v_info.tar_a > 0) {
        if(obs_tar_v_info.obstacle_info.obj_v >= (chassis_status_.v + 6.0 / 3.6)){
          if (obs_tar_v_info.tar_v < (chassis_status_.v + 3.6 / 3.6)) {
            float surmise_control_kp = 1.0;
            float surmise_control_target_a = surmise_control_kp * (obs_tar_v_info.tar_v - chassis_status_.v) +
                                              obs_tar_v_info.tar_a;
            if (surmise_control_target_a < 0) {
              is_need_dec = true;
            }
          }
        }else{
          if (obs_tar_v_info.tar_v < (chassis_status_.v + 3.6 / 3.6)){  
             is_need_dec = true;
          }
        }
        
      }else{
        if(obs_tar_v_info.obstacle_info.obj_v < (chassis_status_.v + 6.0 / 3.6)){
          if (obs_tar_v_info.tar_v < (chassis_status_.v + 3.6 / 3.6)){
            is_need_dec = true;
          }
        }
      }
    }
  }
  
  bool is_pcc_from_road = GetRoadEventReporting();
  
#if PLANNING_DEBUG
  planning_debug_.pacc_enable = is_pcc_from_road;
#endif
  // 细分巡航（有无坡度）和有障碍物（停车STOP）
  if (keep_cruise_v && (!is_need_dec)) {
    // TODO: 从PCC场景切换到跟车场景会出现顿挫，临时方案通过判断是巡航状态才执行PCC，最终通过PCC增加跟车模式解决
    // PCC进入的前提条件是巡航CC状态，cruise_speed 取小后的巡航上限速度 // fix by wangwh 2023.10.06.18:34
    if (settings_.enable_pacc && (!is_pcc_from_road)) { // 考虑到安全，在隧道内不执行PCC
      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != pcc_tar_v_info.tar_type) {
        result_of_planning_.tar_type = pcc_tar_v_info.tar_type;
        result_of_planning_.tar_v = pcc_tar_v_info.tar_v;
        result_of_planning_.tar_a = pcc_tar_v_info.tar_a;
        result_of_planning_.release_throttle = pcc_tar_v_info.release_throttle;
        result_of_planning_.tar_throttle = pcc_tar_v_info.tar_throttle;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Planning][PCC] tar_type = " << pcc_tar_v_info.tar_type << ", tar_v = " << pcc_tar_v_info.tar_v << "m/s(" << pcc_tar_v_info.tar_v * 3.6 << "km/h), tar_a = " << pcc_tar_v_info.tar_a << "m/s^2"
                <<  ", release_throttle = " << pcc_tar_v_info.release_throttle << ", tar_throttle = " << pcc_tar_v_info.tar_throttle;
#endif
      }
    }

    if (cc_upper_limit.valid &&
        (VELOCITY_PLANNING_TARGET_TYPE_PCC != result_of_planning_.tar_type)) {
      if (common::com_abs(result_of_planning_.tar_v - cc_upper_limit_v_) < 1.5 / 3.6F) {  //用户设定车速与界面速度统一
        result_of_planning_.release_throttle = false;
        result_of_planning_.tar_v = cc_upper_limit_v_;//接近限速值时油门抖动 cc_tar_v_info.tar_v
        result_of_planning_.tar_a = 0.0;
      }
    }

  } else {
    //有障碍物
    result_of_planning_.tar_type = obs_tar_v_info.tar_type;
    result_of_planning_.tar_v = obs_tar_v_info.tar_v;
    // Fix:防止跟车加速度超限
    // result_of_planning_.tar_a = obs_tar_v_info.tar_a;//但出现跟车不减速
    // 与CC计算的tar_a取小（tar_a＞0|<0，加速或减速均取小）fix by qiuzw 2023.10.06.11:03
    if (obs_tar_v_info.tar_a < result_of_planning_.tar_a) {
      result_of_planning_.tar_a = obs_tar_v_info.tar_a;
    }
    result_of_planning_.release_throttle = false;
    result_of_planning_.tar_throttle = 0.0;
    result_of_planning_.tar_pos.x = obs_tar_v_info.tar_pos.point.x();
    result_of_planning_.tar_pos.y = obs_tar_v_info.tar_pos.point.y();
    result_of_planning_.tar_pos.heading = obs_tar_v_info.tar_pos.heading;
    result_of_planning_.tar_pos.s = obs_tar_v_info.tar_s;

    result_of_planning_.tar_obj.valid = obs_tar_v_info.obstacle_info.valid;
    result_of_planning_.tar_obj.dist_to_obj = obs_tar_v_info.obstacle_info.dist_to_obj;
    result_of_planning_.tar_obj.time_gap = obs_tar_v_info.obstacle_info.time_gap;
    result_of_planning_.tar_obj.relative_v = obs_tar_v_info.obstacle_info.relative_v;
    result_of_planning_.tar_obj.ttc = obs_tar_v_info.obstacle_info.ttc;
    result_of_planning_.tar_obj.obj_v = obs_tar_v_info.obstacle_info.obj_v;
    result_of_planning_.tar_obj.dist_gap_level = obs_tar_v_info.obstacle_info.dist_gap_level;
    result_of_planning_.tar_obj.rel_spd_level = obs_tar_v_info.obstacle_info.rel_spd_level;

    result_of_planning_.release_throttle = obs_tar_v_info.release_throttle;
    if ((common::com_abs(result_of_planning_.tar_v - obs_tar_v_info.obstacle_info.obj_v) < 5 / 3.6F)) {
      result_of_planning_.release_throttle = false;
    }
    result_of_planning_.release_throttle = result_of_planning_.release_throttle | overtake_release_throttle;
    
    if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == result_of_planning_.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == result_of_planning_.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == result_of_planning_.tar_type) {
      
      // STOP冗余判断
      VelocityPlanningForObstacleStop(result_of_planning_.tar_type, result_of_planning_.tar_obj.obj_v, result_of_planning_.tar_v, result_of_planning_.tar_a);
    }
  }
  
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[Planning][Nosmooth] tar_type = " << result_of_planning_.tar_type << ", tar_v = " << result_of_planning_.tar_v << "m/s(" 
              << result_of_planning_.tar_v * 3.6 << "km/h), tar_a = " << result_of_planning_.tar_a << "m/s^2";
#endif  

// E 降级
  if (result_of_action_planning_.enable_fallback) {
    CalcVelocityAccordingToFallback(result_of_action_planning_, curr_position_.v, &result_of_planning_);
  } else{
    first_B_II_ = true;
    first_C_II_ = true;
  }
  
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) <<  "[Planning][Fallback] first_B_II_ = " << first_B_II_  << ", first_C_II_ = " << first_C_II_;
#endif 

// F AD启动状态下相关变量的初始化 pre_adas_mode_ adas_mode_
  VelocityPlanningStartAD();

#if PLANNING_DEBUG
  planning_debug_.tar_type_before_final = result_of_planning_.tar_type;
  planning_debug_.tar_v_before_final = result_of_planning_.tar_v;
  planning_debug_.tar_a_before_final = result_of_planning_.tar_a;
#endif

// G 对本周期规划的速度做平滑处理，AEB除外aeb_hold_flag
  if (!status_.aeb_hold_flag) {
    plan_var_t smoothed_tar_v = result_of_planning_.tar_v;
    plan_var_t smoothed_tar_a = result_of_planning_.tar_a;
    if (VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS == result_of_planning_.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT == result_of_planning_.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_TUNNEL == result_of_planning_.tar_type ||
        VELOCITY_PLANNING_TARGET_TYPE_RAMP == result_of_planning_.tar_type) {
      SmoothVelocity(result_of_planning_, &smoothed_tar_v, &smoothed_tar_a);
    } else if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == result_of_planning_.tar_type ||
               VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == result_of_planning_.tar_type ||
               VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == result_of_planning_.tar_type) {
        if (!(result_of_planning_.tar_obj.obj_v < 5.0 / 3.6)) {
           SmoothVelocity(result_of_planning_, &smoothed_tar_v, &smoothed_tar_a);
        } else {
           // Smooth_Vel
        }
    }
    result_of_planning_.tar_v = smoothed_tar_v;
    result_of_planning_.tar_a = smoothed_tar_a;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Planning][Smooth] tar_v = " << result_of_planning_.tar_v * 3.6 << result_of_planning_.tar_v << "m/s("
              << "km/h, tar_a = " << result_of_planning_.tar_a << "m/s^2";
#endif    
  }

#if PLANNING_DEBUG
  uint8_t smooth_count = 3;
  for (size_t i = 0; i < smooth_count; i++) {
    planning_debug_.tar_a_history[i] = array_target_acceleration_[i];
    planning_debug_.tar_v_history[i] = array_target_velocity_[i];
  }
  
  planning_debug_.tar_v_smoothed = result_of_planning_.tar_v;
  planning_debug_.tar_a_smoothed = result_of_planning_.tar_a;
#endif

// H1 用于限制最終的速度和加速度
  LimitFinalResult(result_of_planning_);

// H2 用于记录本周期的数据作为下一周期的参考
  RecordData(result_of_planning_);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  if (1 == sys_cal_aebs) {
    plan_var_t ratio = 0;
    Int32_t lower = common::LerpInOrderedTable(param_.aeb_action_ttc_table, curr_position_.v, &ratio);
    plan_var_t tar_ttc = common::Lerp(param_.aeb_action_ttc_table[lower].value, param_.aeb_action_ttc_table[lower + 1].value, ratio);
    if ((result_of_planning_.tar_obj.ttc < tar_ttc) && result_of_planning_.tar_obj.valid) {
      result_of_planning_.tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
    }
  }
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[Planning][Final] tar_type = " << result_of_planning_.tar_type
              << ", tar_v = " << result_of_planning_.tar_v << "m/s(" << result_of_planning_.tar_v * 3.6F << "km/h), tar_a = " << result_of_planning_.tar_a << "m/s^2 "
              << " EGO v = " << curr_position_.v * 3.6F << "km/h, Chassis V = " << chassis_status_.v * 3.6
              << " km/h, EGO a = " << curr_position_.a << "m/s^2 ";
  LOG_INFO(5) << "Chassis a = " << chassis_status_.a << "m/s^2, tar_obj.x = " << result_of_planning_.tar_obj.dist_to_obj
              << "m, tar_obj.rel_v = " << result_of_planning_.tar_obj.relative_v * 3.6
              << " km/h, tar_obj.abs_v = " << result_of_planning_.tar_obj.obj_v * 3.6 << "km/h";
  LOG_INFO(5) <<  "<<<<<<<< Velocity planning (Polynomial Fitting) (end) ########\n";
#endif

#if PLANNING_DEBUG
      planning_debug_.tar_type_final = result_of_planning_.tar_type;
      planning_debug_.tar_v_final = result_of_planning_.tar_v;
      planning_debug_.tar_a_final = result_of_planning_.tar_a;
#endif

  return true;
}


// 获取事件报告信息，输出给HMI （显示速度规划的HMI指令）
Int32_t VelocityPlanningPolynomialFitting::GetEventReporting(Int32_t num, ad_msg::EventReporting *const events) const {
  if (num > event_reporting_list_.Size()) {
    num = event_reporting_list_.Size();
  }

  Int32_t idx = 0;
  for (idx = 0; idx < num; ++idx) {
    events[idx] = event_reporting_list_[idx];
  }

  return (idx);
}

bool VelocityPlanningPolynomialFitting::GetRoadEventReporting() const {

  int event_num = event_reporting_list_.Size();
  ad_msg::EventReporting temp_event;
  int temp_event_id = 0;
  bool is_special_road_evet = false;

  for (int idx = 0; idx < event_num; ++idx) {
    temp_event = event_reporting_list_[idx];
    temp_event_id = temp_event.event_id;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
LOG_INFO(5) << "[Planning][GetRoadEventReporting] temp_event_id = " << temp_event_id << ", event_num = " << event_num;
#endif  
    if ((ad_msg::EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL == temp_event_id) &&
        ((chassis_status_.v > (param_.tunnel_velocity_limit - 2 /3.6))||(chassis_status_.v > (cc_upper_limit_v_ - 2 /3.6)))) {
        is_special_road_evet = is_special_road_evet || true;
    }

    if(ad_msg::EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_RAMP == temp_event_id) {
      is_special_road_evet = is_special_road_evet || true;
    }
    
  }


  return is_special_road_evet;
}


//泛化的五次多项式拟合V-S计算方法及实践中应对不同场景增加的补丁处理
//TODO: V-S Graph 大小可动态调整; 不同场景需要路测调参
void VelocityPlanningPolynomialFitting::CalcTarVelocityPolynomial(const ParamOfPolynomialFitting &param, const StateOfPolynomial &state, PolynomialFittingOutput& output) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "## Calc Target Velocity Polynomial (begin) >>";
#endif

  plan_var_t polynomial_initial_state_s = 0.0F;
  plan_var_t polynomial_initial_state_vx;
  if (state.initial_state_vx < 0.01F) {
    polynomial_initial_state_vx = 0;
  } else {
    polynomial_initial_state_vx = state.initial_state_vx;
  }
  plan_var_t polynomial_initial_state_ax = state.initial_state_ax;
  
  plan_var_t polynomial_final_state_s = state.final_state_distance + param.safe_distance;

  if (polynomial_final_state_s < 0.0) {
    polynomial_final_state_s = 0.0;
  }
    
  plan_var_t polynomial_final_state_vx;
  if (state.final_state_vx < 0.01F) {
    polynomial_final_state_vx = 0;
  } else {
    polynomial_final_state_vx = state.final_state_vx;
  }
  plan_var_t polynomial_final_state_ax = 0.0f;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "init-s = " << polynomial_initial_state_s << "m, init-vx = " << polynomial_initial_state_vx << "m/s(" << polynomial_initial_state_vx * 3.6 << "km/h), init_ax = " << polynomial_initial_state_ax << "m/s^2"
              << ", final_s = " << polynomial_final_state_s << "m, final_vx = " << polynomial_final_state_vx << "m/s(" << polynomial_final_state_vx * 3.6 << "km/h), final_ax = " << polynomial_final_state_ax << "m/s^2";
#endif

  plan_var_t odd_speed_limit = param_.max_velocity_limit;
  
  // 基于匀加速模型，计算规划周期内最小和最大速度范围
  plan_var_t vxf_min = common::Max(0.0F, polynomial_initial_state_vx + param.ax_min * param.tf);
  plan_var_t vxf_max = common::Min(odd_speed_limit, polynomial_initial_state_vx + param.ax_max * param.tf);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "V Sample range : " << " [" << vxf_min * 3.6F << " ~ " << vxf_max * 3.6F << "]km/h";
#endif
  // 对V均匀采样
  plan_var_t vxf_num = 7 * 8;
  plan_var_t vxf_range[7 * 8];
  linspace(vxf_min, vxf_max, vxf_num, vxf_range);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TAR_VELOCITY_TRACE
  for (Int32_t i = 0; i < vxf_num; i++) {
    printf("目标点的采样速度区间:vxf_range=%lf\n", vxf_range[i]);
  };
#endif

  // 基于匀加速模型，计算规划周期内最小和最大路径范围
  plan_var_t xf_min = (pow(vxf_min, 2) - pow(polynomial_initial_state_vx, 2)) / 2 / param.ax_min;
  plan_var_t xf_max = (pow(vxf_max, 2) - pow(polynomial_initial_state_vx, 2)) / 2 / param.ax_max;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "S Sample range : " << " [" << xf_min << " ~ " << xf_max << "]m";
#endif
  // 对S均匀采样
  Float32_t xf_num = 5 * 8;
  plan_var_t xf_range[5 * 8];
  linspace(polynomial_initial_state_s + xf_min, polynomial_initial_state_s + xf_max, xf_num, xf_range);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TAR_VELOCITY_TRACE
  for (Int32_t i = 0; i < xf_num; i++) {
    printf("目标点的采样距离区间:xf_range=%lf\n", xf_range[i]);
  };
#endif

  // 对T均匀采样
  plan_var_t tf_min = 4;
  plan_var_t tf_max = 12;

  plan_var_t tf_num = 4;
  plan_var_t tf_range[4];
  linspace(tf_min, tf_max, tf_num, tf_range);

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TAR_VELOCITY_TRACE
  for (int i = 0; i < tf_num; i++) {
    printf("目标点的采样时间区间:tf_range=%lf\n\n", tf_range[i]);
  };
#endif

  // 定义代价函数，搜素代价最小的S-V曲线
  plan_var_t t0 = 0.0F;
  plan_var_t t1 = param.tf;//{param.tf};  //  plan_var_t t1[] = { 3, 6, 8, 12 };

  plan_var_t coef5_[6];
  plan_var_t Quintic[6];
  plan_var_t coef3_[4];
  plan_var_t Cubic[4];

  // 初始化COST
  plan_var_t COST[4][40][56] = {0.0F};
  plan_var_t Jerk[4][40][56] = {0.0F};
  plan_var_t V_END[4][40][56] = {0.0F};
  plan_var_t S_END[4][40][56] = {0.0F};
  
  // Initial minimum
  plan_var_t MIN;
  plan_var_t MAX;

  Int32_t t_MIN = 0;
  Int32_t r_MIN = 0;
  Int32_t c_MIN = 0;

  plan_var_t Best_T;
  plan_var_t Best_S;
  plan_var_t Best_V;
  
  for (Int32_t Ti = 0; Ti < 1; Ti = Ti + 1) {
    for (Int32_t Si = 0; Si < 40; Si = Si + 1) {
      for (Int32_t Vi = 0; Vi < 56; Vi = Vi + 1) {
        if (param.polynomial_order_is_quintic) {
          // 选择初始状态及采样的S-V作为终状态，使用五次多项式拟合，求解系数
          QuinticComputeCoefficients(polynomial_initial_state_s, polynomial_initial_state_vx, polynomial_initial_state_ax, 
                                    xf_range[Si], vxf_range[Vi], polynomial_final_state_ax, t1, coef5_);//t1[Ti]

          // 对五次多项式系数求导
          QuinticPolynomialCurve1d(coef5_, 0, t1, Quintic);//t1[Ti]
          
          Jerk[Ti][Si][Vi] = fabs(Quintic[3]);
          V_END[Ti][Si][Vi] = fabs(Quintic[1] - polynomial_final_state_vx);
          S_END[Ti][Si][Vi] = fabs(Quintic[0] - polynomial_final_state_s);
        } else {
          CubicComputeCoefficients(polynomial_initial_state_s, polynomial_initial_state_vx, 
                                   xf_range[Si], vxf_range[Vi], t1, coef3_);//t1[Ti]
          CubicPolynomialCurve1d(coef3_, 0, t1, Cubic);//t1[Ti]

          Jerk[Ti][Si][Vi] = fabs(Cubic[3]);
          V_END[Ti][Si][Vi] = fabs(Cubic[1] - polynomial_final_state_vx);
          S_END[Ti][Si][Vi] = fabs(Cubic[0] - polynomial_final_state_s);
        }
        /*--------------------------------COST in the two-dimensional array--------------------------------*/

        COST[Ti][Si][Vi] = param.jerk_cost * Jerk[Ti][Si][Vi] + param.v_cost * V_END[Ti][Si][Vi] + 
                           param.s_cost * S_END[Ti][Si][Vi];
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TAR_VELOCITY_TRACE
  printf("COST[%d][%d][%d]=%lf\n\n ", Ti, Si, Vi, COST[Ti][Si][Vi]);
#endif
        /*--------------------------------筛选最小COST对应的位置--------------------------------*/

        if (COST[Ti][Si][Vi] < COST[t_MIN][r_MIN][c_MIN]) { 
          t_MIN = Ti;
          r_MIN = Si;
          c_MIN = Vi;
        }
      }
    }
  }

  MIN = COST[t_MIN][r_MIN][c_MIN];

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "MIN COST = " << MIN << ", t_MIN = " << t_MIN << ", r_MIN = " << r_MIN << ", c_MIN = " << c_MIN;
#endif

  // 获取最小cost对应的T-S-V采样值
  Best_T = t1;//t1[t_MIN];
  Best_S = xf_range[r_MIN];
  Best_V = vxf_range[c_MIN];

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "Best_T = " << Best_T << "s, Best_S = " << Best_S << "m, Best_V = " << Best_V * 3.6 << "km/h";
#endif

  Float32_t best_s_output;
  Float32_t best_v_output;
  Float32_t best_a_output;

  if (param.polynomial_order_is_quintic) {
    Float32_t best_coef5[6] = {0.0F};
    Float32_t best_Quintic5[6] = {0.0F};

    QuinticComputeCoefficients(polynomial_initial_state_s, polynomial_initial_state_vx, polynomial_initial_state_ax, 
                               Best_S, Best_V, polynomial_final_state_ax, Best_T, best_coef5);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "c0_ = " << best_coef5[0] << ","
                << " c1_ = " << best_coef5[1] << ","
                << " c2_ = " << best_coef5[2] << ","
                << " c3_ = " << best_coef5[3] << ","
                << " c4_ = " << best_coef5[4] << ","
                << " c5_ = " << best_coef5[5];
#endif
    QuinticPolynomialCurve1d(best_coef5, 0, param.preview_time,
                             best_Quintic5);
    best_s_output = best_Quintic5[0];
    best_v_output = best_Quintic5[1];
    best_a_output = best_Quintic5[2];

  } else {
    Float32_t best_coef3[4] = {0.0F};
    Float32_t best_Quintic3[4] = {0.0F};

    CubicComputeCoefficients(polynomial_initial_state_s, polynomial_initial_state_vx, 
                             xf_range[r_MIN], vxf_range[c_MIN], Best_T, best_coef3);
    CubicPolynomialCurve1d(best_coef3, 0, param.preview_time, best_Quintic3);

    best_s_output = best_Quintic3[0];
    best_v_output = best_Quintic3[1];
    best_a_output = best_Quintic3[2];
  }

  output.tar_v = best_v_output;
  output.tar_a = best_a_output;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<< Calc Target Velocity Polynomial (end) ## : tar_s = " << best_s_output << " m, tar_v = " << best_v_output << "m/s("
              << (best_v_output * 3.6) << "km/h), tar_a = " << best_a_output << "m/s2";
#endif
}

void VelocityPlanningPolynomialFitting::linspace(Float32_t x1, Float32_t x2, Int32_t n, Float32_t *y) {
  Float32_t d = (x2 - x1) / (n - 1);

  for (Int32_t i = 0; i < n; i++) {
    y[i] = x1 + i * d;
  }
  return;
}

void VelocityPlanningPolynomialFitting::CubicPolynomialCurve1d(const Float32_t *coef3_, const Int32_t order, const Float32_t p,
                                                      Float32_t *Cubic) {
  switch (order) {
    case 0: {
      Cubic[0] = ((coef3_[3] * p + coef3_[2]) * p + coef3_[1]) * p + coef3_[0];
    }
    case 1: {
      Cubic[1] = (3.0 * coef3_[3] * p + 2.0 * coef3_[2]) * p + coef3_[1];
    }
    case 2: {
      Cubic[2] = 6.0 * coef3_[3] * p + 2.0 * coef3_[2];
    }
    case 3: {
      Cubic[3] = 6.0 * coef3_[3];
    }
    default:
      return;
  }
  return;
}

void VelocityPlanningPolynomialFitting::CubicComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t x1,
                                                        const Float32_t dx1, const Float32_t p, Float32_t *coef3_) {
  COM_CHECK(p > 0);

  coef3_[0] = x0;
  coef3_[1] = dx0;

  const Float32_t p2 = p * p;

  const Float32_t c0 = (x1 - x0 - dx0 * p) / p2;
  const Float32_t c1 = (dx1 - dx0) / p;

  coef3_[2] = 3.0F * c0 - c1;
  coef3_[3] = (c0 - coef3_[2]) / p;
}

void VelocityPlanningPolynomialFitting::QuinticComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t ddx0,
                                                          const Float32_t x1, const Float32_t dx1, const Float32_t ddx1,
                                                          const Float32_t p, Float32_t *coef5_) {
  coef5_[0] = x0;
  coef5_[1] = dx0;
  coef5_[2] = ddx0 / 2.0;

  const Float32_t p2 = p * p;
  const Float32_t p3 = p * p2;

  const Float32_t c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
  const Float32_t c1 = (dx1 - ddx0 * p - dx0) / p2;
  const Float32_t c2 = (ddx1 - ddx0) / p;

  coef5_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
  coef5_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
  coef5_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;

  // return coef5_;		//返回到主函数
}

void VelocityPlanningPolynomialFitting::QuinticPolynomialCurve1d(const Float32_t *coef5_, const Int32_t order, const Float32_t p,
                                                        Float32_t *Quintic) {
  switch (order) {
    case 0: {
      Quintic[0] = ((((coef5_[5] * p + coef5_[4]) * p + coef5_[3]) * p + coef5_[2]) * p + coef5_[1]) * p + coef5_[0];
    }
    case 1: {
      Quintic[1] =
          (((5.0 * coef5_[5] * p + 4.0 * coef5_[4]) * p + 3.0 * coef5_[3]) * p + 2.0 * coef5_[2]) * p + coef5_[1];
    }
    case 2: {
      Quintic[2] = (((20.0 * coef5_[5] * p + 12.0 * coef5_[4]) * p) + 6.0 * coef5_[3]) * p + 2.0 * coef5_[2];
    }
    case 3: {
      Quintic[3] = (60.0 * coef5_[5] * p + 24.0 * coef5_[4]) * p + 6.0 * coef5_[3];
    }
    case 4: {
      Quintic[4] = 120.0 * coef5_[5] * p + 24.0 * coef5_[4];
    }
    case 5: {
      Quintic[5] = 120.0 * coef5_[5];
    }
    default:
      0.0;
  }
  return;
}

// 1、根据前车和自车速度计算动态安全距离；2、根据车速计算重规划阈值；3、根据载荷/挡位/坡度，计算最大和最小加速度(ax_max/ax_min)
void VelocityPlanningPolynomialFitting::UpdatePolynomialParamAccMaxMinByCurrentVehicleState(const bool &is_cc, const plan_var_t &ego_v, const plan_var_t &obj_distance, plan_var_t obj_v, 
                                                                                    plan_var_t *ax_max, plan_var_t *ax_min, plan_var_t *safe_distance,
                                                                                    plan_var_t *replan_limit_acc, plan_var_t* replan_limit_dec) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) <<"# Update Polynomial Param : ax (Begin) >";
#endif
  // 高速的时候加速不够
  plan_var_t high_and_low_speed_boundary_safe = 50 / 3.6;//高低边界速度
  plan_var_t high_and_low_speed_boundary_gear = 40 / 3.6;//高低挡位边界速度
  plan_var_t obs_low_speed_boundary = 30 / 3.6;

  plan_var_t temp_ax_max = 0.5;
  plan_var_t temp_ax_min = -0.2; 

  plan_var_t temp_safe_distance = 0.0; //即s_diff

  //安全距离（动态）：时距计算：s_diff = obj_distance + safe_distance; ts = s_diff / ego_v;
  if (!is_cc) {
    if (obj_v >= obs_low_speed_boundary) {
      if (ego_v <= high_and_low_speed_boundary_safe) {
        temp_safe_distance = -0.2 * ego_v;//TODO 需要改temp_safe_distance
      } else {
        temp_safe_distance = 0.0;
      }
    } else {
      temp_safe_distance =  -5.0;
    }
  } else {
    temp_safe_distance = 0.0;
  }
  
  //重规划阈值：
  if (ego_v < high_and_low_speed_boundary_safe) {
    *replan_limit_acc = 1.0;
    *replan_limit_dec = 1.3;
  } else {
    if (obj_distance < 100.0F) { //跟车重新规划
      *replan_limit_acc = 1.3;
      *replan_limit_dec = 1.3;
    } else {
      *replan_limit_acc = 2.0;
    }
  }
  
  //最大加速度阈值：
  if (ego_v > high_and_low_speed_boundary_gear) {
    //TODO 根据当前挡位gear_number限制最大加速度 ax_max
    UpdatePolynomialParamAccAccordingToGear(&temp_ax_max);
  } else {
    temp_ax_max = 0.5;
  }
  
  //最大减速度阈值：
  if (!is_cc) {   
    // TODO 根据纵向边界表，限制最大减速度 ax_mim   
    UpdatePolynomialParamDecAccordingToTable(ego_v, obj_distance, obj_v, temp_safe_distance, &temp_ax_min);
  } else { 
    // TODO 巡航状态下最大减速度 (平路的怠速数据) 
    temp_ax_min = -0.2;
  }

  *safe_distance = temp_safe_distance; 
  *ax_max = temp_ax_max;
  *ax_min = temp_ax_min;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "ego_v = " << ego_v << "m/s(" << ego_v * 3.6 << "km/h), obj_v = " << obj_v << "m/s(" << obj_v * 3.6 << "km/h), obj_s = " << obj_distance << "m";
  LOG_INFO(5) << "replan_acc_upper = " << *replan_limit_acc << "m/s^2, safe_distance = " << temp_safe_distance << "m, ax_min  = " << temp_ax_min << "m/s^2, ax_max = " << temp_ax_max << "m/s^2";

  LOG_INFO(5) <<"< Update Polynomial Param : ax (End) #";
#endif
}

// 根据当前挡位gear_number限制最大加速度ax_max
void VelocityPlanningPolynomialFitting::UpdatePolynomialParamAccAccordingToGear(plan_var_t *ax_max_limit) {
  int gear = (int)chassis_status_.gear_number;
#if ECO_GEAR_ACC
  plan_var_t ax_max_actual = *ax_max_limit; //实际最大加速度
  plan_var_t ax_max_theory = *ax_max_limit; //理论最大加速度
  plan_var_t Max_Torque_Eco = 0.0f; //最大经济扭矩
  plan_var_t Ff = 0.0f; //理论滚阻
  plan_var_t Fw = 0.0f; //理论风阻
  plan_var_t Fi = 0.0f; //理论坡阻
  plan_var_t Ft = 0.0f; //理论总阻力
  plan_var_t Mt = 0.0f; //理论发动机扭矩

  VehicleDynamicParam vehicle_dynamic_param;
  // 取值有效性判断
  if ((chassis_status_.gross_weight == 0.0) || (chassis_status_.v  < 0.0) || 
	  (chassis_status_.engine_speed == 0) || (chassis_ctl_cmd_.acc_value < 0.0) || 
	  (vehicle_dynamic_param.g == 0)) {
	return;
  }
  
  // 理论计算的车辆总阻力
  Ff = chassis_status_.gross_weight * vehicle_dynamic_param.g * vehicle_dynamic_param.f;
  Fw = vehicle_dynamic_param.Cd * vehicle_dynamic_param.Ad * vehicle_dynamic_param.air_density * chassis_status_.v * chassis_status_.v / 2;
  Fi = chassis_status_.gross_weight * vehicle_dynamic_param.g * curr_slope_ / 100;

  Ft = Ff + Fw + Fi;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE 
  LOG_INFO(5) << "Theory value of vehicle dynamics formula, slope = " << curr_slope_ << ", Ff = " << Ff << ", Fw = " << Fw << ", Fi = " << Fi << ", Ft = " << Ft;
#endif 
  // 根据发动机转数求出最大经济扭矩
  if ((chassis_status_.engine_speed >= 0) && (chassis_status_.engine_speed < 650)) {
	  Max_Torque_Eco = 900;
  } else if ((chassis_status_.engine_speed >= 650) && (chassis_status_.engine_speed < 700)) {
	  Max_Torque_Eco = (700 - chassis_status_.engine_speed)/(700 - 650) * 900 + (chassis_status_.engine_speed - 650)/(700 - 650) * 1503;
  } else if ((chassis_status_.engine_speed >= 700) && (chassis_status_.engine_speed < 800)) {
	  Max_Torque_Eco = (800 - chassis_status_.engine_speed)/(800 - 700) * 1503 + (chassis_status_.engine_speed - 700)/(800 - 700) * 1890;
  } else if ((chassis_status_.engine_speed >= 800) && (chassis_status_.engine_speed < 900)) {
	  Max_Torque_Eco = (900 - chassis_status_.engine_speed)/(900 - 800) * 1890 + (chassis_status_.engine_speed - 800)/(900 - 800) * 2250;
  } else if ((chassis_status_.engine_speed >= 900) && (chassis_status_.engine_speed < 960)) {
	  Max_Torque_Eco = (960 - chassis_status_.engine_speed)/(960 - 900) * 2250 + (chassis_status_.engine_speed - 900)/(960 - 900) * 2385;
  } else if ((chassis_status_.engine_speed >= 960) && (chassis_status_.engine_speed < 1500)) {
	  Max_Torque_Eco = 2385;
  } else if ((chassis_status_.engine_speed >= 1500) && (chassis_status_.engine_speed < 1600)) {
	  Max_Torque_Eco = (1600 - chassis_status_.engine_speed)/(1600 - 1500) * 2385 + (chassis_status_.engine_speed - 1500)/(1600 - 1500) * 2244.6;
  } else if ((chassis_status_.engine_speed >=1600) && (chassis_status_.engine_speed < 1700)) {
	  Max_Torque_Eco = (1700 - chassis_status_.engine_speed)/(1700 - 1600) * 2244.6 + (chassis_status_.engine_speed - 1600)/(1700 - 1600) * 2113.2;
  } else if ((chassis_status_.engine_speed >= 1700) && (chassis_status_.engine_speed < 1800)) {
	  Max_Torque_Eco = (1800 - chassis_status_.engine_speed)/(1800 - 1700) * 2113.2 + (chassis_status_.engine_speed - 1700)/(1800 - 1700) * 1995.3;
  } else if ((chassis_status_.engine_speed >= 1800) && (chassis_status_.engine_speed < 1830)) {
	  Max_Torque_Eco = (1830 - chassis_status_.engine_speed)/(1830 - 1800) * 1995.3 + (chassis_status_.engine_speed - 1800)/(1830 - 1800) * 1962.9;
  } else if ((chassis_status_.engine_speed >= 1830) && (chassis_status_.engine_speed < 2400)) {
	  Max_Torque_Eco = 1962.9;
  } else {
	  Max_Torque_Eco = 0.0f;
  }
#if (TM_YD)
  vehicle_dynamic_param.If = 3.42; 
  
  vehicle_dynamic_param.Ig.Clear();
  vehicle_dynamic_param.Ig.PushBack(0.0f);
  vehicle_dynamic_param.Ig.PushBack(14.43);
  vehicle_dynamic_param.Ig.PushBack(11.05);
  vehicle_dynamic_param.Ig.PushBack(8.44);
  vehicle_dynamic_param.Ig.PushBack(6.46);
  vehicle_dynamic_param.Ig.PushBack(4.95);
  vehicle_dynamic_param.Ig.PushBack(3.79);
  vehicle_dynamic_param.Ig.PushBack(2.91);
  vehicle_dynamic_param.Ig.PushBack(2.23);
  vehicle_dynamic_param.Ig.PushBack(1.70);
  vehicle_dynamic_param.Ig.PushBack(1.30);
  vehicle_dynamic_param.Ig.PushBack(1.0);
  vehicle_dynamic_param.Ig.PushBack(0.77);
#endif

#if (TM_ZF)
  vehicle_dynamic_param.If = 2.69;

  vehicle_dynamic_param.Ig.Clear();
  vehicle_dynamic_param.Ig.PushBack(0.0);
  vehicle_dynamic_param.Ig.PushBack(16.688);
  vehicle_dynamic_param.Ig.PushBack(12.924);
  vehicle_dynamic_param.Ig.PushBack(9.926);
  vehicle_dynamic_param.Ig.PushBack(7.688);
  vehicle_dynamic_param.Ig.PushBack(5.895);
  vehicle_dynamic_param.Ig.PushBack(4.565);
  vehicle_dynamic_param.Ig.PushBack(3.655);
  vehicle_dynamic_param.Ig.PushBack(2.831);
  vehicle_dynamic_param.Ig.PushBack(2.174);
  vehicle_dynamic_param.Ig.PushBack(1.684);
  vehicle_dynamic_param.Ig.PushBack(1.291);
  vehicle_dynamic_param.Ig.PushBack(1.0);
#endif
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE 
  LOG_INFO(5) << "vehicle_dynamic_param.eta = " << vehicle_dynamic_param.eta << "vehicle_dynamic_param.Rw = " 
			  << vehicle_dynamic_param.Rw << "vehicle_dynamic_param.If = " << vehicle_dynamic_param.If;
  LOG_INFO(5) << "Max_Torque_Eco = "<< Max_Torque_Eco << "chassis_status_.engine_torque = " << chassis_status_.engine_torque 
			  << "[Vehicle Dynamics Parameters], chassis_status_.gross_weight = " << chassis_status_.gross_weight; 
  LOG_INFO(5) << "chassis_status_.engine_speed = " << chassis_status_.engine_speed << "Current Gear = " << gear;
  for (size_t i = 0; i < 13; i++) {
	  LOG_INFO(5) << "Gear = " << i << "Transmission ratio = " << vehicle_dynamic_param.Ig[i];
  }
	  
#endif 
//	根据底实际扭矩和理论扭矩计算不同挡位对应的实际最大加速度和理论最大加速度

  switch (gear) {
    case 12:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[12] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[12] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[12] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 11:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[11] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[11] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[11] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 10:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[10] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[10] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[10] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 9:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[9] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[9] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[9] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 8:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[8] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[8] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[8] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 7:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[7] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[7] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[7] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 6:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[6] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[6] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[6] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 5:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[5] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[5] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[5] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 4:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[4] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[4] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[4] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 3:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[3] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[3] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[3] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 2:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[2] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[2] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[2] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  case 1:
	  ax_max_actual = (Max_Torque_Eco - chassis_status_.engine_torque) *	vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[1] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  Mt = Ft / vehicle_dynamic_param.If / vehicle_dynamic_param.Ig[1] * vehicle_dynamic_param.Rw;
	  ax_max_theory = (Max_Torque_Eco - Mt) *  vehicle_dynamic_param.If * vehicle_dynamic_param.Ig[1] * vehicle_dynamic_param.eta / vehicle_dynamic_param.Rw / chassis_status_.gross_weight;
	  break;
  default:
	  ax_max_actual = 0;
	  Mt = 0;
	  ax_max_theory = 0;
	  break;
  }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE 
  LOG_INFO(5) << "Theory value of vehicle dynamics formula, Mt = " << Mt << ", ax_max_actual = " << ax_max_actual << ", ax_max_theory = " << ax_max_theory;
#endif

  // 发动机扭矩为0时取理论加速度（松油门和换挡时）
  if (chassis_status_.engine_torque == 0) {
	  *ax_max_limit = ax_max_theory;
  } else if (ax_max_actual > 0 && ax_max_theory > 0) {
	  // 当实际值和理论值均大于0时，最终加速度最大限值在两者之间之间取小
	  *ax_max_limit = (ax_max_actual < ax_max_theory) ? ax_max_actual : ax_max_theory;
  } else {
	  // 当实际值和理论值存在小于0时，最终加速度最大限值在两者之间之间取大
	  *ax_max_limit = (ax_max_actual > ax_max_theory) ? ax_max_actual : ax_max_theory;
  }

  // 防止在车辆在加速过程中由于转速处于经济转速区之上而产生制动行为 
  if (*ax_max_limit <= 0 ) {
	  *ax_max_limit = 0.3;
  }
#endif

#if (TM_YD)
  // 预留根据油门开度限制加速度
  if (chassis_ctl_cmd_.acc_value > 80) {
  /* code */
  }

  if ((1 < gear) && (gear <= 10)) {
    *ax_max_limit = (*ax_max_limit <= 0.3) ? *ax_max_limit : 0.3;
  } else if ((10 < gear) && (gear <= 11)) {
    *ax_max_limit = (*ax_max_limit <= 0.25) ? *ax_max_limit : 0.25;
  } else if ( gear == 12) {
    *ax_max_limit = 0.15;
  } else {
    *ax_max_limit = 0.3;
  }
#endif
	
#if (TM_ZF)
	if ((1 < gear) && (gear <= 10)) {
	  *ax_max_limit = (*ax_max_limit <= 0.25) ? *ax_max_limit : 0.25;
	} else if ((10 < gear) && (gear <= 11)) {
	  *ax_max_limit = (*ax_max_limit <= 0.2) ? *ax_max_limit : 0.2;
	} else if ( gear == 12) {
	  *ax_max_limit = 0.13;
	} else {
	  *ax_max_limit = 0.25;
	}
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE 
  LOG_INFO(5) << "UpdatePolynomialParam by gear number : " << gear << ", *ax_max_limit = " << *ax_max_limit;
#endif 
}

// 根据纵向边界表，限制最大减速度(待更新)
void VelocityPlanningPolynomialFitting::UpdatePolynomialParamDecAccordingToTable(const plan_var_t &ego_v, const plan_var_t &obj_distance, plan_var_t obj_v, plan_var_t safe_distance,
                                                                          plan_var_t *ax_min_limit) {
  plan_var_t no_limit_ax_min_boundary = 50 / 3.6;
  
  plan_var_t ax_min = *ax_min_limit;  

  plan_var_t s_diff = obj_distance + safe_distance;
  plan_var_t v_diff = obj_v - ego_v;

  plan_var_t ts = s_diff / ego_v;

  plan_var_t follow_ttc = 100.0;

  if (ego_v > obj_v) {
    follow_ttc = -s_diff / v_diff;
  }

  if (ego_v > no_limit_ax_min_boundary && v_diff > -20 / 3.6) {
    if (ts >= 3.5) {
      ax_min = -0.2;
    } else if(ts < 3.5 && ts >= 3.0) {
      ax_min = -0.5;
    } else if(ts < 3.0 && ts >= 2.5) {
      ax_min = -0.7;
      if ((ego_v < (70 / 3.6)) && ((-10 / 3.6) <= v_diff) && (v_diff < 0)) {
        ax_min = -1.0;
        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "table ax_min = " << ax_min << std::endl;
        #endif
      } else if (ego_v < 60 / 3.6 && v_diff <= -10 / 3.6) {
        ax_min = -1.5;
        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "table ax_min = " << ax_min << std::endl;
        #endif
      }
    } else {
      ax_min = -1.0;
      if (s_diff < 30) {
        ax_min = -1.5;
        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "s_diff < 30 table ax_min = " << ax_min << std::endl;
        #endif
      }
    }
  } else {
    if (ts >= 3.5) {
      ax_min = -0.5;
    } else if(ts < 3.5 && ts >= 3.0) {
      ax_min = -1.0;
    } else if(ts < 3.0 && ts >= 2.5) {
      ax_min = -1.5;
      if (follow_ttc < 7.0) {
        ax_min = -2.0;
        #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        std::cout << "follow_ttc 7.0 ax_min = " << ax_min << std::endl;
        #endif
      } 
    } else {
      ax_min = -2.0;
      if (follow_ttc < 5.0) {
        ax_min = -2.5;
        #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        std::cout << "follow_ttc 5.0 ax_min = " << ax_min << std::endl;
        #endif
      }
    }
  }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[UpdatePolynomialParamDecAccordingToTable] ax_min = " << ax_min ;
#endif

  *ax_min_limit = ax_min;
}

// 多项式初始值更新(从MD到AD/起步计数/重规划)
void VelocityPlanningPolynomialFitting::UpdatePolynomialInitState(const ParamOfPolynomialFitting &param,
                                                                            StateOfPolynomial &polynomial_input) {
  plan_var_t curr_v = curr_position_.v;//TODO curr_position_.v
  plan_var_t curr_a = chassis_status_.a;

  if ((!pre_adas_mode_) && adas_mode_) { // 从人工驾驶到自动驾驶
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[UpdatePolynomialInputValue] START ADAS Command ! ! ";
#endif
    polynomial_input.initial_state_vx = curr_v;
    polynomial_input.initial_state_ax = curr_a;
  } else if (pre_adas_mode_ && adas_mode_) { // 一直在自动驾驶模式
    if (start_count_ <= 10) {
      polynomial_input.initial_state_vx = curr_v;
      polynomial_input.initial_state_ax = curr_a;
    } else {
      // 使用当前状态重新规划
      // TODO: 为什么要重规划？不重规划会有什么后果？ 重规划后是否同时将历史数据清除
      if ((prev_tar_v_ - curr_v > param.replan_limit_acc) ||
          (curr_v - prev_tar_v_ > param.replan_limit_dec)) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[UpdatePolynomialInputValue] Speed is Replan !! prev_tar_v_ = " << prev_tar_v_ * 3.6
                  << " km/h ,  prev_tar_a_ =  " << prev_tar_a_ << " m/s2 "
                  << ", curr_v = " << curr_v * 3.6 << "km/h";
#endif
        polynomial_input.initial_state_vx = curr_v;
        polynomial_input.initial_state_ax = curr_a;
      } else {
        polynomial_input.initial_state_vx = prev_tar_v_;
        polynomial_input.initial_state_ax = prev_tar_a_;
      }
      
    }
  } else { // 人工驾驶
    polynomial_input.initial_state_vx = curr_v;
    polynomial_input.initial_state_ax = curr_a;
  }
}

void VelocityPlanningPolynomialFitting::UpdatePolynomialFinalStateForCC(const plan_var_t &target_vel, 
                                                                            StateOfPolynomial &polynomial_output) {
    if ((polynomial_output.initial_state_vx - target_vel) >= 10 / 3.6) {
      polynomial_output.final_state_vx = curr_position_.v - 10 / 3.6; // 如果巡航车速远小于自车车速，通过设置分段目标车速来减少不舒适性
    }
}

/*  A 根据道路约束计算巡航速度上限，包含：
 * A1. 用户设定CC车速        CalculateCruiseVelocityAccordingToUserSetting
 * A2. 道路限速CC车速        CalculateCruiseVelocityAccordingToRoadLimit
 * A3. 隧道内限速CC车速      CalculateCruiseVelocityAccordingToTunnelLimit
 * A4. 匝道内限速CC车速      CalculateCruiseVelocityAccordingToRampLimit
 * A5. 下一帧曲率限速CC车速  CalculateCruiseVelocityAccordingToCurvatureLimit
 * A6. 变道场景CC车速        CalculateCruiseVelocityAccordingToLaneChange
 * ...
 */
void VelocityPlanningPolynomialFitting::CalcTargetVelocityAccordingToRoadRules(TargetVelocityUpperLimit *cc_vel_road_rules) {
  
  tar_v_upper_limit_list_.Clear();
  TargetVelocityUpperLimit target_velocity;
  TargetVelocityUpperLimit target_velocity_final;
  
  // A1. 用户设定CC车速
  CalculateCruiseVelocityAccordingToUserSetting(&target_velocity);
  if (target_velocity.valid) {
    tar_v_upper_limit_list_.PushBack(target_velocity);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CC] User Setting V = " << target_velocity.tar_v << "m/s(" << target_velocity.tar_v * 3.6 << "km/h)";
#endif    
  }
  
  //fix：隧道和道路限速调换/当道路限速->隧道限速=80km/h时，最终状态则为道路限速，进入PCC，而实际期望隧道退PCC。
  // A2. 隧道内限速CC车速  
  CalculateCruiseVelocityAccordingToTunnelLimit(&target_velocity);
  if (target_velocity.valid) {
    tar_v_upper_limit_list_.PushBack(target_velocity);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CC] Tunnel Limit V = " << target_velocity.tar_v << "m/s(" << target_velocity.tar_v * 3.6 << "km/h)";
#endif      
  }

  // A3. 道路限速CC车速
  CalculateCruiseVelocityAccordingToRoadLimit(&target_velocity);
  if (target_velocity.valid) {
    tar_v_upper_limit_list_.PushBack(target_velocity);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CC] Road Limit V = " << target_velocity.tar_v << "m/s(" << target_velocity.tar_v * 3.6 << "km/h)";
#endif      
  }

  // A4. 匝道内限速CC车速
  CalculateCruiseVelocityAccordingToRampLimit(&target_velocity);
  if (target_velocity.valid) {
    tar_v_upper_limit_list_.PushBack(target_velocity);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CC] Ramp Limit V = " << target_velocity.tar_v << "m/s(" << target_velocity.tar_v * 3.6 << "km/h)";
#endif      
  }  

  //TODO: A5. 下一帧曲率限速CC车速
  /* 屏蔽曲率限速
    CalculateCruiseVelocityAccordingToCurvatureLimit(&target_velocity);
    if (target_velocity.valid) {
      tar_v_upper_limit_list_.PushBack(target_velocity);
    } 
  */

  // A6. 变道场景CC车速
  if (!settings_.enable_pacc) { // PCC状态下，计算出的速度低于目标巡航速度，导致变道时不断掉速
    CalculateCruiseVelocityAccordingToLaneChange(&target_velocity);
    if (target_velocity.valid) {
      tar_v_upper_limit_list_.PushBack(target_velocity);
  #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[CC] LaneChange Limit V = " << target_velocity.tar_v << "m/s(" << target_velocity.tar_v * 3.6 << "km/h)";
  #endif       
    } 
  }
  // A：各种限速 => 都PushBack压入列表 tar_v_upper_limit_list_  
  Int32_t tar_v_upper_limit_list_size = tar_v_upper_limit_list_.Size();
  Int32_t tar_v_upper_limit_list_index = 0;

  // 根据上述限速约束，选择最低的限速作为巡航目标速度
  if (tar_v_upper_limit_list_size > 0) {
    target_velocity_final.valid = tar_v_upper_limit_list_[0].valid;
    target_velocity_final.tar_type = tar_v_upper_limit_list_[0].tar_type;
    target_velocity_final.tar_v = tar_v_upper_limit_list_[0].tar_v;

    for (Int32_t i = 1; i < tar_v_upper_limit_list_size; ++i) {
      if (tar_v_upper_limit_list_[i].tar_v < tar_v_upper_limit_list_[tar_v_upper_limit_list_index].tar_v) {
        tar_v_upper_limit_list_index = i;
        target_velocity_final.valid = tar_v_upper_limit_list_[tar_v_upper_limit_list_index].valid;
        target_velocity_final.tar_type = tar_v_upper_limit_list_[tar_v_upper_limit_list_index].tar_type;
        target_velocity_final.tar_v = tar_v_upper_limit_list_[tar_v_upper_limit_list_index].tar_v;
      }
    }
  }
  
  //TODO：暂时没有用 最开始想做 释放油门的状态
  int cc_action = 0;
  //cc_action = CCActionJudge(target_velocity_final.tar_v);

  //fix：针对速度不同源的问题，均统一为EBS车速源；将外围环境车速如仪表车速（A1~A6）取小后，再除以1.02

  //fix_2:仪表盘车速与实际车速统一
  plan_var_t temp_InstrumentPanel = 0;
  if(curr_position_.v * 3.6 >= 90 && curr_position_.v * 3.6 <= 95){
    temp_InstrumentPanel = 1.018;
  }else if(curr_position_.v * 3.6 > 95 && curr_position_.v * 3.6 <= 100){
    temp_InstrumentPanel = 1.015;
  }else{
    temp_InstrumentPanel = 1.02;
  }
  target_velocity_final.tar_v = target_velocity_final.tar_v / temp_InstrumentPanel;

  *cc_vel_road_rules = target_velocity_final;
  
  return;
}

//A1. 用户设定CC车速
void VelocityPlanningPolynomialFitting::CalculateCruiseVelocityAccordingToUserSetting(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();

  target_vel_result->valid = true;
  target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS;
  if(settings_.tar_v >= 95 && settings_.tar_v <= 100){
    target_vel_result->tar_v = settings_.tar_v + 2.0 / 3.6F;
  }else{
    target_vel_result->tar_v = settings_.tar_v;
  }

  return;
}

//A2. 道路限速CC车速
void VelocityPlanningPolynomialFitting::CalculateCruiseVelocityAccordingToRoadLimit(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();
//A2.1 道路限速（数据源：驾驶地图/MPU）
#if ENABLE_SPEED_LIMIT_FROM_DRIVER_MAP
  // 从DriveMap获取道路限速信息
  Int32_t curr_lane_index = -1;
  common::PathPoint veh_proj_on_curr_lane;

  if (!driving_map_->FindNearestLaneSynthetically(
          curr_position_.path_point.point, curr_position_.path_point.heading,
          &curr_lane_index, &veh_proj_on_curr_lane)) {
      curr_lane_index = -1;
      LOG_ERR << "Failed to find nearest lane to camera from Mixed HD-Map, using camera lane.";
  }

  if (!driving_map_->IsValidLaneIndex(curr_lane_index)) {
    curr_lane_index = -1;
    LOG_ERR << "Failed to find nearest lane of current point, "
                "because the lane index is invalid.";
  }

  plan_var_t lane_speed_limit = -1.0F;

  if (curr_lane_index > -1) {
    lane_speed_limit = driving_map_->GetLaneSpeedLimitHigh(curr_lane_index);
  }

  if (lane_speed_limit > 0.0F) {
    target_vel_result->valid = true;
    target_vel_result->tar_v = lane_speed_limit;
    target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT;
  }
#else
    // 从MPU获取道路限速信息
    plan_var_t lane_speed_limit = -1.0F;
    mpu_t::MpuState mpu_state;
    if (!mpu_t::GetMpuState(mpu_state)) {
      LOG_INFO(3) << "Failed to get mpu state.";
    }
    lane_speed_limit = mpu_state.MonitorState.LimitSpeedMax / 3.6F;
    if (lane_speed_limit > 0) {
      target_vel_result->valid = true;
      target_vel_result->tar_v = lane_speed_limit;
      target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT;
    }
#endif

#if PLANNING_DEBUG
    planning_debug_.lane_velocity_limit = lane_speed_limit;
    //TODO: 此变量取值位置待确认
    // planning_debug_.distance_to_lane_velocity_limit = ;
    planning_debug_.upper_limit_v_road = target_vel_result->tar_v;
    planning_debug_.tar_type_lane = result_of_planning_.tar_type;
    planning_debug_.tar_v_lane = result_of_planning_.tar_v;
    planning_debug_.tar_a_lane = result_of_planning_.tar_a;
#endif
}

//A3. 隧道内限速CC车速
void VelocityPlanningPolynomialFitting::CalculateCruiseVelocityAccordingToTunnelLimit(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();
  
  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
                                                 driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();

  plan_var_t tunnel_velocity_limit = -1.0;
  plan_var_t planning_cycle_time = 0.1; //此函数内未使用局部变量？？？

  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL != story.type) { // 非隧道场景
      continue;
    }
    if (story.area.DistanceToArea() > 0.1F) { // 如果在隧道内行驶，距离为0
      continue;
    }

    tunnel_velocity_limit = param_.tunnel_velocity_limit;
  }

  if (tunnel_velocity_limit >= 0.0) {
    target_vel_result->valid = true;
    target_vel_result->tar_v = tunnel_velocity_limit;
    target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_TUNNEL;
    AddEventReporting(EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL);
  }

#if PLANNING_DEBUG
  planning_debug_.upper_limit_v_tunnel = target_vel_result->tar_v;
#endif

} 

//A4. 匝道内限速CC车速
void VelocityPlanningPolynomialFitting::CalculateCruiseVelocityAccordingToRampLimit(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();

  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
                                                  driving_map_->GetSceneStoryList();

  plan_var_t ramp_velocity_limit = -1.0;
  plan_var_t planning_cycle_time = 0.1; //此函数内未使用局部变量？？？

  Int32_t storys_num = story_list.Size();
  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP != story.type) {  // 非匝道场景
      continue;
    }
    if (story.area.DistanceToArea() > 0.1F) { // 如果在匝道内行驶，距离为0
      continue;
    }

    ramp_velocity_limit = param_.ramp_velocity_limit; 
  }

  if (ramp_velocity_limit >= 0.0) {
    target_vel_result->valid = true;
    target_vel_result->tar_v = ramp_velocity_limit;
    target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_RAMP;
    AddEventReporting(EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_RAMP);
  }

#if PLANNING_DEBUG
    planning_debug_.upper_limit_v_ramp = target_vel_result->tar_v;
#endif
}

//A5. 下一帧曲率限速CC车速
void VelocityPlanningPolynomialFitting::CalculateCruiseVelocityAccordingToCurvatureLimit(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();

  Int32_t major_ref_line_index = driving_map_->GetMajorReferenceLineIndex();
  if (!driving_map_->IsValidReferenceLineIndex(major_ref_line_index)) {
    LOG_ERR << "Invalid reference line index " << major_ref_line_index;
    ClearHoldInfo();
    target_vel_result->valid = false; 
    return;
  }
  const common::Path &major_ref_line = driving_map_->GetSmoothReferenceLine(major_ref_line_index);

  const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum>
      &major_ref_line_curvature_info = driving_map_->GetSmoothReferenceLineCurveInfo(major_ref_line_index);

  plan_var_t major_ref_line_forward_length = major_ref_line.total_length() - veh_proj_on_major_ref_line_.s;

  plan_var_t curvature_velocity_limit = -1.0;
  plan_var_t abs_curvature = 0.0;
  Int32_t curve_seg_num = major_ref_line_curvature_info.Size();

  for (Int32_t i = 0; i < curve_seg_num; ++i) {
    const common::Path::CurveSegment &curve_seg = major_ref_line_curvature_info[i];

    if (common::Path::TYPE_STRAIGHT == curve_seg.type) {//直路
      continue;
    }

    if ((curve_seg.start_s + curve_seg.length) < veh_proj_on_major_ref_line_.s) {
      continue;
    }
    
    if ((curve_seg.start_s + curve_seg.length) > veh_proj_on_major_ref_line_.s) {
      abs_curvature = common::com_abs(curve_seg.max_curvature);
      if (abs_curvature > 0.001F) {     
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Curvature Limit] curvature < 0.001 = " << abs_curvature;
#endif
        // 向心加速度计算公式
        curvature_velocity_limit = common::com_sqrt(settings_.lateral_acceleration_limit / abs_curvature);
      }
      break;
    }

  }

  if (curvature_velocity_limit >= 0.0) {
    target_vel_result->valid = true;
    target_vel_result->tar_v = curvature_velocity_limit;
    target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_CURVATURE;

#if PLANNNING_DEBUG
    planning_debug_.upper_limit_v_curvature = target_vel_result->tar_v;
    planning_debug.target_path_max_curvature = abs_curvature;
#endif
  }

  return;
}

//A6. 变道场景CC车速
void VelocityPlanningPolynomialFitting::CalculateCruiseVelocityAccordingToLaneChange(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();

  plan_var_t lane_change_velocity_limit = -1.0;
  //if (result_of_trajectory_planning_.trj_changing.is_changing) { 
  if((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == result_of_trajectory_planning_.trj_status) || 
     (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == result_of_trajectory_planning_.trj_status) ||
     (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == result_of_trajectory_planning_.trj_status) ||
     (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == result_of_trajectory_planning_.trj_status)||
     (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == result_of_trajectory_planning_.trj_status) || 
     (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == result_of_trajectory_planning_.trj_status) ||
     (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == result_of_trajectory_planning_.trj_status) ||
     (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II == result_of_trajectory_planning_.trj_status)) {
     //lane_change_velocity_limit = curr_position_.v;//避免换道加速->fix: 针对变道过程中掉速，维持道路约束巡航车速
  } else {
  }

  if (lane_change_velocity_limit >= 0.0) {
    target_vel_result->valid = true;
    target_vel_result->tar_v = lane_change_velocity_limit;
    target_vel_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS;
  }

#if PLANNING_DEBUG_
  planning_debug_.upper_limit_v_lane_change = target_vel_result->tar_v;
#endif

  return;

}

// 根据道路约束规划巡航速度
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToRoadRules(const TargetVelocityUpperLimit &cc_vel_road_rules, TargetVelocityInfo *result) {
    
    result->Clear();
    TargetVelocityInfo tar_v_info;   
    tar_v_info.tar_type = cc_vel_road_rules.tar_type;

    ParamOfPolynomialFitting param_of_calc_tar_v_ax;
    param_of_calc_tar_v_ax.is_cc = true;//CC 巡航
    param_of_calc_tar_v_ax.tf = 3.0;
    param_of_calc_tar_v_ax.jerk_cost = 0.4;
    param_of_calc_tar_v_ax.v_cost = 0.4;
    param_of_calc_tar_v_ax.s_cost = 0.2;
    param_of_calc_tar_v_ax.preview_time = 0.2;
    param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;
    
    // 考虑载荷、挡位、坡度、重规划等，ax_max 和 ax_min 的上下限
    UpdatePolynomialParamAccMaxMinByCurrentVehicleState(true, curr_position_.v, 150.0, cc_vel_road_rules.tar_v, &(param_of_calc_tar_v_ax.ax_max), &(param_of_calc_tar_v_ax.ax_min), &(param_of_calc_tar_v_ax.safe_distance),
                                                &(param_of_calc_tar_v_ax.replan_limit_acc), &(param_of_calc_tar_v_ax.replan_limit_dec));
    if (tar_v_info.tar_type == VELOCITY_PLANNING_TARGET_TYPE_RAMP) {
      param_of_calc_tar_v_ax.ax_min = -0.7;
    }

    StateOfPolynomial param_of_polynomial_start_end_state;
    param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
    param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
    param_of_polynomial_start_end_state.final_state_vx = cc_vel_road_rules.tar_v;
    param_of_polynomial_start_end_state.final_state_distance = 150.0;

    UpdatePolynomialInitState(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state);
    UpdatePolynomialFinalStateForCC(cc_vel_road_rules.tar_v, param_of_polynomial_start_end_state);

    PolynomialFittingOutput output;
    CalcTarVelocityPolynomial(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, output);
    tar_v_info.tar_v = output.tar_v;
    tar_v_info.tar_a = output.tar_a;
    *result = tar_v_info;
  
    return;
}


//TODO：暂时没有用 最开始想做 释放油门的状态
int VelocityPlanningPolynomialFitting::CCActionJudge(plan_var_t target_cc_vel){

  plan_var_t curr_slope;
  bool slope_information_valid;

  if (!result_of_trajectory_planning_.target_trajectory.Empty()) {
    curr_slope = result_of_trajectory_planning_.target_trajectory_slope.Front().slope;
    slope_information_valid = true;
  } else {
    slope_information_valid = false;
    curr_slope = 0.0F;
  }  

  int cc_action_result = CC_ACTION_INVALID;
  /// TODO:匀速行驶 加速度波动
  if (!slope_information_valid) {
    if (((target_cc_vel - chassis_status_.v) > 5.0 / 3.6) && 
        (chassis_status_.a > 0.15)) {

      cc_action_result = CC_ACTION_ACC;

    } else if (((target_cc_vel - chassis_status_.v) < -5.0 / 3.6) && 
        (chassis_status_.a < -0.15)) {

      cc_action_result = CC_ACTION_DEC;

    } else if (((target_cc_vel - chassis_status_.v) < -5.0 / 3.6) &&
                (chassis_status_.a > 0.15)) {

      cc_action_result = CC_ACTION_ACC_TO_DEC;
      
    } else if (((target_cc_vel - chassis_status_.v) > 5.0 / 3.6) && 
                (chassis_status_.a < -0.15)) {

      cc_action_result = CC_ACTION_DEC_TO_ACC;

    } else if ((common::com_abs(target_cc_vel - chassis_status_.v)) < 5.0 / 3.6) {

      cc_action_result = CC_ACTION_HOLD;

    } else if (((target_cc_vel - chassis_status_.v) > 5.0 / 3.6) && 
                (common::com_abs(chassis_status_.a)) < 0.15){
      
      cc_action_result = CC_ACTION_NEED_ACC;
    
    } else if (((target_cc_vel - chassis_status_.v) < -5.0 / 3.6) && 
                (common::com_abs(chassis_status_.a)) < 0.15) {
      
      cc_action_result = CC_ACTION_NEED_DEC;

    }
  } else {
    /// TODO:在坡度行驶过程中
  }

  return cc_action_result;

}


/*   B 虚拟障碍物场景-根据交通规则约束，将交通对象当做虚拟障碍物，规划未来一段距离的平滑的速度，包含：
 * B1. 虚拟障碍物-交通规则（交通灯）          PlanVelocityAccordingToTrafficLight
 * B2. 虚拟障碍物-交通规则（交通标志）        PlanVelocityAccordingToTrafficSignal
 * B3. 虚拟障碍物-交通规则（交通场景）        PlanVelocityAccordingToSceneStory
 * B4. 虚拟障碍物-交通规则（接近隧道）        PlanVelocityAccordingToCloseToTunnel
 * B5. 虚拟障碍物-交通规则（接近匝道）        PlanVelocityAccordingToCloseToRamp
 * ...
 */
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToTrafficRules(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo *result) {
  TargetVelocityInfo tar_v_virtual_info_obj;
  tar_v_virtual_obstacle_list_.Clear();
  
  //B1. 虚拟障碍物-交通规则（交通灯）
  PlanVelocityAccordingToTrafficLight(path, proj_on_path, &tar_v_virtual_info_obj);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_virtual_info_obj.tar_type) {
    tar_v_virtual_obstacle_list_.PushBack(tar_v_virtual_info_obj);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[TrafficLight] tar_type =" << tar_v_virtual_info_obj.tar_type << ", tar_v = " << tar_v_virtual_info_obj.tar_v << "m/s("
                << tar_v_virtual_info_obj.tar_v * 3.6 << "km/h), tar_a = " << tar_v_virtual_info_obj.tar_a << "m/s^2";
#endif

#if PLANNING_DEBUG
    planning_debug_.tar_type_traffic_light = tar_v_virtual_info_obj.tar_type;
    planning_debug_.tar_v_traffic_light = tar_v_virtual_info_obj.tar_v;
    planning_debug_.tar_a_traffic_light = tar_v_virtual_info_obj.tar_a;
#endif
  }
  
  //B2. 虚拟障碍物-交通规则（交通标志）
  PlanVelocityAccordingToTrafficSignal(path, proj_on_path, &tar_v_virtual_info_obj);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_virtual_info_obj.tar_type) {
    tar_v_virtual_obstacle_list_.PushBack(tar_v_virtual_info_obj);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[TrafficSignal] tar_type =" << tar_v_virtual_info_obj.tar_type << ", tar_v = " << tar_v_virtual_info_obj.tar_v << "m/s("
                << tar_v_virtual_info_obj.tar_v * 3.6 << "km/h), tar_a = " << tar_v_virtual_info_obj.tar_a << "m/s^2";
#endif

#if PLANNING_DEBUG
    planning_debug_.tar_type_traffic_signal = tar_v_virtual_info_obj.tar_type;
    planning_debug_.tar_v_traffic_signal = tar_v_virtual_info_obj.tar_v;
    planning_debug_.tar_a_traffic_signal = tar_v_virtual_info_obj.tar_a;
#endif
  }

  //B3. 虚拟障碍物-交通规则（交通场景）
  PlanVelocityAccordingToSceneStory(path, proj_on_path, &tar_v_virtual_info_obj);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_virtual_info_obj.tar_type) {
    tar_v_virtual_obstacle_list_.PushBack(tar_v_virtual_info_obj);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[SceneStory] tar_type =" << tar_v_virtual_info_obj.tar_type << ", tar_v = " << tar_v_virtual_info_obj.tar_v << "m/s("
              << tar_v_virtual_info_obj.tar_v * 3.6 << "km/h), tar_a = " << tar_v_virtual_info_obj.tar_a << "m/s^2";
#endif
  }

  //B4. 虚拟障碍物-交通规则（接近隧道）
  PlanVelocityAccordingToCloseToTunnel(path, proj_on_path, &tar_v_virtual_info_obj);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_virtual_info_obj.tar_type) {
    tar_v_virtual_obstacle_list_.PushBack(tar_v_virtual_info_obj);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Tunnel] tar_type =" << tar_v_virtual_info_obj.tar_type << ", tar_v = " << tar_v_virtual_info_obj.tar_v << "m/s("
                << tar_v_virtual_info_obj.tar_v * 3.6 << "km/h), tar_a = " << tar_v_virtual_info_obj.tar_a << "m/s^2";
#endif

#if PLANNING_DEBUG
    planning_debug_.tar_type_tunnel = tar_v_virtual_info_obj.tar_type;
    planning_debug_.tar_v_tunnel = tar_v_virtual_info_obj.tar_v;
    planning_debug_.tar_a_tunnel = tar_v_virtual_info_obj.tar_a;
#endif
  }

  //B5. 虚拟障碍物-交通规则（接近匝道）
  PlanVelocityAccordingToCloseToRamp(path, proj_on_path, &tar_v_virtual_info_obj);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_virtual_info_obj.tar_type) {
    tar_v_virtual_obstacle_list_.PushBack(tar_v_virtual_info_obj);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Ramp] tar_type =" << tar_v_virtual_info_obj.tar_type << ", tar_v = " << tar_v_virtual_info_obj.tar_v << "m/s("
                << tar_v_virtual_info_obj.tar_v * 3.6 << "km/h), tar_a = " << tar_v_virtual_info_obj.tar_a << "m/s^2";
#endif
#if PLANNNING_DEBUG
    planning_debug_.tar_type_ramp = tar_v_virtual_info_obj.tar_type;
    planning_debug_.tar_v_ramp = tar_v_virtual_info_obj.tar_v;
    planning_debug_.tar_a_ramp = tar_v_virtual_info_obj.tar_a;
#endif
  }

  TargetVelocityInfo tar_v_info_obj;
  Int32_t tar_v_virtual_info_list_size = tar_v_virtual_obstacle_list_.Size();
  Int32_t tar_v_virtual_obstacle_index = 0;
  bool is_has_virtual_obstacle = false;

  if (tar_v_virtual_info_list_size > 0) {
    is_has_virtual_obstacle = true;
    for (Int32_t i = 1; i < tar_v_virtual_info_list_size; ++i) {
      if (tar_v_virtual_obstacle_list_[i].tar_v < tar_v_virtual_obstacle_list_[tar_v_virtual_obstacle_index].tar_v) {
        tar_v_virtual_obstacle_index = i;
      }
    }
  }

  if (is_has_virtual_obstacle) {
    tar_v_info_obj = tar_v_virtual_obstacle_list_[tar_v_virtual_obstacle_index];
#if PLANNING_DEBUG
    planning_debug_.tar_type_virtual_obstacle = tar_v_info_obj.tar_type;
    planning_debug_.tar_v_virtual_obstacle = tar_v_info_obj.tar_v;
    planning_debug_.tar_a_virtual_obstacle = tar_v_info_obj.tar_a;
#endif
  } else {
    tar_v_info_obj.Clear();
  }



  *result = tar_v_info_obj;
}


//B1. 虚拟障碍物-交通规则（交通灯）
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToTrafficLight(const common::Path &path,
                                                                   const common::PathPoint &proj_on_path,
                                                                   TargetVelocityInfo *result) {
  result->Clear();

  // 红绿灯颜色判断
  bool traffic_light_is_red = false;
  for (Int32_t i = 0; i < traffic_light_list_.traffic_light_num; ++i) {
    if (ad_msg::TrafficLight::RED == traffic_light_list_.traffic_lights[i].color) {
      traffic_light_is_red = true;
      break;
    } else if (ad_msg::TrafficLight::YELLOW == traffic_light_list_.traffic_lights[i].color) {
      traffic_light_is_red = true;
      break;
    } else {
      // nothing to do
    }
  }
  if (!traffic_light_is_red) {
    // 绿灯直接通过
    return;
  }

  const common::StaticVector<driv_map::MapTrafficLight, driv_map::MAX_MAP_TRAFFIC_LIGHT_NUM> &map_traffic_light_table =
      driving_map_->GetMapTrafficLightTable();
  Int32_t map_signal_num = map_traffic_light_table.Size();
  for (Int32_t i = 0; i < map_signal_num; ++i) {
    const driv_map::MapTrafficLight &map_signal = map_traffic_light_table[i];

    common::PathPoint cross_point;
    if (path.IsIntersect(map_signal.stop_line.start(), map_signal.stop_line.end(), &cross_point)) {
      if ((cross_point.s - proj_on_path.s) < 0.0) {
        // 后轮已经通过停止线了
        continue;
      }
      // 停车距离限制
      plan_var_t s_to_obj = cross_point.s - proj_on_path.s - param_.dist_of_localization_to_front;

      ParamOfPolynomialFitting param_of_calc_tar_v_ax;
      param_of_calc_tar_v_ax.is_cc = true;
      param_of_calc_tar_v_ax.tf = 3.0;
      param_of_calc_tar_v_ax.jerk_cost = 0.4;
      param_of_calc_tar_v_ax.v_cost = 0.4;
      param_of_calc_tar_v_ax.s_cost = 0.2;
      param_of_calc_tar_v_ax.preview_time = 0.2;
      param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;

      StateOfPolynomial param_of_polynomial_start_end_state;
      param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
      param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
      param_of_polynomial_start_end_state.final_state_vx = 0.0F;
      param_of_polynomial_start_end_state.final_state_distance = s_to_obj;

      PolynomialFittingOutput output;
      
      CalcTarVelocityPolynomial(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, output);

      result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_TRAFFIC_LIGHT;
      result->tar_v = output.tar_v;
      result->tar_a = output.tar_a;
      result->tar_s = s_to_obj;
      if (result->tar_s < 0.0F) {
        result->tar_s = 0.0F;
      }
      path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
    }
  }

#if PLANNING_DEBUG
  planning_debug_.traffic_light_type = result->tar_type;
  planning_debug_.distance_to_traffic_light = result->tar_s;
#endif

}

//B2. 虚拟障碍物-交通规则（交通标志）
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToTrafficSignal(const common::Path &path,
                                                                    const common::PathPoint &proj_on_path,
                                                                    TargetVelocityInfo *result) {
  // 根据交通标志信息限速
  if (result_of_action_planning_.enable_isl) {
    plan_var_t spd_signal_v = settings_.tar_v;
    if (traffic_signal_list_.msg_head.valid) {
      const ad_msg::TrafficSignalList &signal_list = traffic_signal_list_;

      Int32_t tar_idx = -1;
      for (Int32_t i = 0; i < signal_list.speed_restriction_num; ++i) {
        const ad_msg::TrafficSignalSpeedRestriction &cur_signal = signal_list.speed_restrictions[i];

        if (ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION == cur_signal.type) {
          if (tar_idx < 0) {
            tar_idx = i;
          } else {
            if (cur_signal.speed > signal_list.speed_restrictions[tar_idx].speed) {
              tar_idx = i;
            }
          }
        }
      }
      if (tar_idx >= 0) {
        plan_var_t spd_limit = signal_list.speed_restrictions[tar_idx].speed;
        if (spd_signal_v > spd_limit) {
          spd_signal_v = spd_limit;
        }
#if PLANNING_DEBUG
      planning_debug_.traffic_signal_type = signal_list.speed_restrictions[tar_idx].type;
      // TODO: 红绿灯距离使用TrafficSignalBox的x？
      planning_debug_.distance_to_traffic_signal = signal_list.speed_restrictions[tar_idx].box.x;
#endif
      }
    }

    if (spd_signal_v < settings_.tar_v) {
      result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_ISL;
      if (spd_signal_v < 6.0F / 3.6F) {
        spd_signal_v -= 1.0F / 3.6F;
      }
      result->tar_v = spd_signal_v;
      if (curr_position_.v > spd_signal_v + 3.0F / 3.6F) {
        result->tar_a = -0.8F;
      } else {
        result->tar_a = 0.0F;
      }
      result->tar_s = 0.0F;
      path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
    }
  }
}

//B3. 虚拟障碍物-交通规则（交通场景）
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToSceneStory(const common::Path &path,
                                                                 const common::PathPoint &proj_on_path,
                                                                 TargetVelocityInfo *result) {
  result->Clear();

  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
      driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();
  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (!story.area.IsInArea()) {
      // 不在受控的范围内
      continue;
    }
    if (!story.action.vaild) {
      // 没有行为
      continue;
    }
    if (!story.action.valid_speed) {
      // 不需要进行速度控制
      continue;
    }

    plan_var_t velocity_limit = story.action.speed;
    plan_var_t s_to_obj = story.area.DistanceToArea();
    if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
      s_to_obj = s_to_obj - param_.dist_of_localization_to_rear;
    }

    ParamOfPolynomialFitting param_of_calc_tar_v_ax;
    param_of_calc_tar_v_ax.is_cc = true;
    param_of_calc_tar_v_ax.tf = 3.0;
    param_of_calc_tar_v_ax.jerk_cost = 0.4;
    param_of_calc_tar_v_ax.v_cost = 0.4;
    param_of_calc_tar_v_ax.s_cost = 0.2;
    param_of_calc_tar_v_ax.preview_time = 0.2;
    param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;

    StateOfPolynomial param_of_polynomial_start_end_state;
    param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
    param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
    param_of_polynomial_start_end_state.final_state_vx = velocity_limit;
    param_of_polynomial_start_end_state.final_state_distance = s_to_obj;

    PolynomialFittingOutput output;
    
    CalcTarVelocityPolynomial(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, output);

    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_SCENE_STORY;
    result->tar_v = output.tar_v;
    result->tar_a = output.tar_a;
    result->tar_s = s_to_obj;
    if (result->tar_s < 0.0F) {
      result->tar_s = 0.0F;
    }
    path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
  }
}

//B4. 虚拟障碍物-交通规则（接近隧道）
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToCloseToTunnel(const common::Path &path,
                                                                 const common::PathPoint &proj_on_path,
                                                                 TargetVelocityInfo *result) {
  result->Clear();

  plan_var_t plan_tunnel_start_s = 300.0F;

  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
      driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();

  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL != story.type) {
      // 不在隧道场景
      continue;
    }
    plan_var_t s_to_obj = story.area.DistanceToArea();

    if (s_to_obj > plan_tunnel_start_s ) { // 距离隧道太远
      continue;
    } else if (s_to_obj < 0.1F) {  // 隧道内，由CalculateCruiseVelocityAccordingToTunnelLimit处理
      continue;
    }
    AddEventReporting(EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL);

    //fix：针对速度不同源的问题，均统一为EBS车速源，除以1.02
    plan_var_t tunnel_velocity_limit = param_.tunnel_velocity_limit / 1.02;

    if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
      s_to_obj = s_to_obj - param_.dist_of_localization_to_rear;
    }

    if (curr_position_.v < tunnel_velocity_limit) { // 隧道前加速
      continue;
    } else if (curr_position_.v < tunnel_velocity_limit + 3.0/3.6F) {
      result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_TUNNEL;
      result->tar_v = tunnel_velocity_limit;
      result->tar_a = 0.0;
      result->tar_s = s_to_obj;
      if (result->tar_s < 0.0F) {
        result->tar_s = 0.0F;
      }
      path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
      continue;
    }

    if (s_to_obj < 5.0F) {
      s_to_obj = 5.0F;
    }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CloseToTunnel] story : type = " << story.type << ", area valid = " << story.area.IsValid() << ", distance = " << story.area.DistanceToArea() 
              << ", velocity_limit = " << tunnel_velocity_limit * 3.6F;
#endif

#if PLANNING_DEBUG
    planning_debug_.tunnel_velocity_limit = tunnel_velocity_limit;
    planning_debug_.distance_to_tunnel = story.area.DistanceToArea();
#endif

    ParamOfPolynomialFitting param_of_calc_tar_v_ax;
    param_of_calc_tar_v_ax.is_cc = true;
    param_of_calc_tar_v_ax.tf = 3.0;
    param_of_calc_tar_v_ax.jerk_cost = 0.4;
    param_of_calc_tar_v_ax.v_cost = 0.4;
    param_of_calc_tar_v_ax.s_cost = 0.2;
    param_of_calc_tar_v_ax.preview_time = 0.2;
    param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;
    param_of_calc_tar_v_ax.ax_min = -0.35;
    param_of_calc_tar_v_ax.ax_max = 0.13;
    param_of_calc_tar_v_ax.replan_limit_acc = 2.0;

    StateOfPolynomial param_of_polynomial_start_end_state;
    param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
    param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
    param_of_polynomial_start_end_state.final_state_vx = tunnel_velocity_limit;
    param_of_polynomial_start_end_state.final_state_distance = s_to_obj;

    PolynomialFittingOutput output;

    UpdatePolynomialInitState(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state);
    CalcTarVelocityPolynomial(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, output);

    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_TUNNEL;
    result->tar_v = output.tar_v;
    result->tar_a = output.tar_a;
    result->tar_s = s_to_obj;
    if (result->tar_s < 0.0F) {
      result->tar_s = 0.0F;
    }

    path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));

  }
}

//B5. 虚拟障碍物-交通规则（接近匝道）
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToCloseToRamp(const common::Path &path,
                                                                 const common::PathPoint &proj_on_path,
                                                                 TargetVelocityInfo *result) {

  result->Clear();

  plan_var_t plan_ramp_start_s = 300.0F;

  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
      driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();
  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP != story.type) {
      // 不在匝道场景
      continue;
    }
    plan_var_t s_to_obj = story.area.DistanceToArea();

    if (s_to_obj > plan_ramp_start_s ) { // 距离匝道太远
      continue;
    } else if (s_to_obj < 0.1F) {  // 匝道内，由CalculateCruiseVelocityAccordingToRampLimit处理
      continue;
    }
    AddEventReporting(EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_RAMP);

    //fix：针对速度不同源的问题，均统一为EBS车速源，除以1.02
    plan_var_t ramp_velocity_limit = param_.ramp_velocity_limit / 1.02;

    if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
      s_to_obj = s_to_obj - param_.dist_of_localization_to_rear;
    }

    if (curr_position_.v < ramp_velocity_limit) {
      continue;
    } else if (curr_position_.v < ramp_velocity_limit + 3.0/3.6F) {
      result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_RAMP;
      result->tar_v = ramp_velocity_limit;
      result->tar_a = 0.0;
      result->tar_s = s_to_obj;
      if (result->tar_s < 0.0F) {
        result->tar_s = 0.0F;
      }
      path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
      continue;
    }

    if (s_to_obj < 5.0F) {
      s_to_obj = 5.0F;
    }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[CloseToRamp] story : type = " << story.type << ", area valid = " << story.area.IsValid() << ", distance = " << story.area.DistanceToArea() 
                << ", velocity_limit = " << ramp_velocity_limit * 3.6F;
#endif

#if PLANNING_DEBUG
    planning_debug_.ramp_velocity_limit = ramp_velocity_limit;
    planning_debug_.distance_to_ramp = s_to_obj;
#endif

    ParamOfPolynomialFitting param_of_calc_tar_v_ax;
    param_of_calc_tar_v_ax.is_cc = true;
    param_of_calc_tar_v_ax.tf = 3.0;
    param_of_calc_tar_v_ax.jerk_cost = 0.4;
    param_of_calc_tar_v_ax.v_cost = 0.4;
    param_of_calc_tar_v_ax.s_cost = 0.2;
    param_of_calc_tar_v_ax.preview_time = 0.2;
    param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;
    param_of_calc_tar_v_ax.ax_min = -0.6;
    param_of_calc_tar_v_ax.ax_max = 0.13;
    param_of_calc_tar_v_ax.replan_limit_acc = 2.0;

    StateOfPolynomial param_of_polynomial_start_end_state;
    param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
    param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
    param_of_polynomial_start_end_state.final_state_vx = ramp_velocity_limit;
    param_of_polynomial_start_end_state.final_state_distance = s_to_obj;

    PolynomialFittingOutput output;
    
    UpdatePolynomialInitState(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state);
    CalcTarVelocityPolynomial(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, output);

    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_RAMP;
    result->tar_v = output.tar_v;
    result->tar_a = output.tar_a;
    result->tar_s = s_to_obj;
    if (result->tar_s < 0.0F) {
      result->tar_s = 0.0F;
    }
    path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));

  }
}

// C 虚拟障碍物场景-弯道曲率的速度规划，主要考虑弯道约束场景
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToPathCurvature(
    const common::Path &path,
    const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum> &curvature_info,
    const common::PathPoint &proj_on_path, plan_var_t range_start_s, plan_var_t range_end_s,
    TargetVelocityInfo *result) {

  result->Clear();
  Int32_t curve_seg_num = curvature_info.Size();
  for (Int32_t i = 0; i < curve_seg_num; ++i) {
    const common::Path::CurveSegment &curve_seg = curvature_info[i];

    if (common::Path::TYPE_STRAIGHT == curve_seg.type) {
      continue;
    }
    if ((curve_seg.start_s + curve_seg.length) < proj_on_path.s) {
      continue;
    }

#if 0
        if ((curve_seg.start_s + curve_seg.length) < range_start_s) {
            continue;
        }
        if (curve_seg.start_s > range_end_s) {
            continue;
        }
#endif

    plan_var_t abs_curvature = common::com_abs(curve_seg.max_curvature);
    if (abs_curvature < 0.001F) {     
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[PathCurvature] abs_curvature < 0.001 = " << abs_curvature;
#endif
      continue;
    }
    plan_var_t ratio = 0;
    Int32_t lower = common::LerpInOrderedTable(param_.curvatrue_limit_velocity_table, 1 / abs_curvature, &ratio);
    plan_var_t velocity_limit = param_.curvatrue_limit_velocity_table[lower + 1].value;
 
    // plan_var_t velocity_limit = common::com_sqrt(settings_.lateral_acceleration_limit / abs_curvature);
    plan_var_t s_to_obj = curve_seg.start_s - proj_on_path.s;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[PathCurvature] velocity_limit = " << velocity_limit * 3.6
                << "km/h, abs_curvature = " << abs_curvature << " ,  R = " << 1 / abs_curvature << "m"
                << ",  s_to_obj = " << s_to_obj << "m"
                << ", ego v = " << curr_position_.v * 3.6 << "km/h";
#endif 

    ParamOfPolynomialFitting param_of_calc_tar_v_ax;
    param_of_calc_tar_v_ax.is_cc = true;
    param_of_calc_tar_v_ax.tf = 3.0;
    param_of_calc_tar_v_ax.jerk_cost = 0.4;
    param_of_calc_tar_v_ax.v_cost = 0.4;
    param_of_calc_tar_v_ax.s_cost = 0.2;
    param_of_calc_tar_v_ax.preview_time = 0.2;
    param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;

    StateOfPolynomial param_of_polynomial_start_end_state;
    param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
    param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
    param_of_polynomial_start_end_state.final_state_vx = velocity_limit;
    param_of_polynomial_start_end_state.final_state_distance = 150.0;

    PolynomialFittingOutput output;
    
    CalcTarVelocityPolynomial(param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, output);

    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_CURVATURE;
    result->tar_v = output.tar_v;
    result->tar_a = output.tar_a;
    result->tar_s = s_to_obj;
    if (result->tar_s < 0.0F) {
      result->tar_s = 0.0F;
    }
    path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
  }
}

// 考虑巡航CC时的坡度
void VelocityPlanningPolynomialFitting::ConsiderSlopeOnPruitCC(VelocityPlanningResult *resuit_vel_plan) {
	plan_var_t const_ax_max = param_.max_acceleration_limit;
  // TODO: 巡航时外围对于ax_max进一步限制取小
	UpdatePolynomialParamAccAccordingToGear(&const_ax_max);
  
#if ECO_GEAR_ACC
	if (slope_flag_) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
	  LOG_INFO(5) << "[ConsiderSlopeOnPruitCC] 有坡度！！！ ";
#endif
	  // FIX: 此处如果坡度为负则可能导致下坡加速,因此更改
	  //考虑坡度和油门开度限制ax_max
	  if (curr_slope_ < 0) {
		const_ax_max = 0.0;
	  } else if ((curr_slope_ >= 0.0) && (curr_slope_ <= 2.0) &&  (control_acc_pedal_ >= 70)) {
		const_ax_max = const_ax_max - curr_slope_ * 0.1;
	  } else if ((curr_slope_ > 2.0) && (control_acc_pedal_ >= 70)) {
		const_ax_max = const_ax_max * 0.7; //限制大油门同时保证一定动力性，临时权重
	  } else {
		/* noting to do */
	  }
	}
#endif
  
	if (const_ax_max < 0) {
	  const_ax_max = 0.0;
	}
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
	  LOG_INFO(5) << "[ConsiderSlopeOnPruitCC] 油门开度, control_acc_pedal_ = " << control_acc_pedal_;
	  LOG_INFO(5) << "[ConsiderSlopeOnPruitCC] 考虑油门开度的最大加速度限制, const_ax_max = " << const_ax_max;
#endif
  
	// TODO 1: 此处对速度进行了限制，如果设置坡度允许掉速，应在此处设置
	if (curr_position_.v  <= (cc_upper_limit_v_ - 5 / 3.6)) {
	  resuit_vel_plan->tar_v = (curr_position_.v + const_ax_max * 0.1);
	  resuit_vel_plan->tar_a = const_ax_max * 0.5;
	} else if (curr_position_.v  > cc_upper_limit_v_) {
	  resuit_vel_plan->tar_v = cc_upper_limit_v_;
	  resuit_vel_plan->tar_a = 0.0;
	} else {
	  resuit_vel_plan->tar_v = cc_upper_limit_v_;
	}
	
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
	  LOG_INFO(5) << "[ConsiderSlopeOnPruitCC] 考虑速度上限的最大加速度限制, const_ax_max = " << const_ax_max;
#endif
	return;
}


// 根据障碍物规划速度，分为3个场景：避障，跟车，紧急制动
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToObstacles(TargetVelocityInfo *result) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) <<"###### Plan Velocity AccordingTo Obstacles (Begin) >>>>>>";
#endif

  status_.exist_uncertain_obstacle = false;
  result->Clear();

  bool req_aeb_action = false;
  bool req_aeb_warning = false;
  bool req_notify_uncertain = false;
  TargetVelocityInfo tar_v_info;

  ConsideringObstaclesDebug obs_debug;
  if (!CalcVelocityConsideringObstacles(target_trajectory_, veh_proj_on_target_trajectory_, &tar_v_info,
                                        &req_aeb_action, &req_aeb_warning, &req_notify_uncertain, obs_debug)) {
    LOG_ERR << "Failed to calculate target speed considering obstacles.";
    return;
  }

#if PLANNING_DEBUG
  // 跟车场景
  common::com_memcpy(&(planning_debug_.cipv), &(obs_debug.cipv), sizeof(planning_debug_.cipv));
  planning_debug_.tar_type_following = obs_debug.tar_type_following;
  planning_debug_.tar_v_following = obs_debug.tar_v_following;
  planning_debug_.tar_a_following = obs_debug.tar_a_following;
  planning_debug_.fsm_follow_state = obs_debug.fsm_follow_state;

  // 避障场景
  common::com_memcpy(&(planning_debug_.avoidance), &(obs_debug.avoidance), sizeof(planning_debug_.avoidance));
  planning_debug_.tar_type_obstacle = obs_debug.tar_type_obstacle;
  planning_debug_.tar_v_obstacle = obs_debug.tar_v_obstacle;
  planning_debug_.tar_a_obstacle = obs_debug.tar_a_obstacle;
  planning_debug_.fsm_obstacle_state = obs_debug.fsm_obstacle_state;

  // 紧急制动场景
  common::com_memcpy(&(planning_debug_.mio), &(obs_debug.mio), sizeof(planning_debug_.mio));
  planning_debug_.tar_type_aeb = obs_debug.tar_type_aeb;
  planning_debug_.tar_v_aeb = obs_debug.tar_v_aeb;
  planning_debug_.tar_a_aeb = obs_debug.tar_a_aeb;
#endif

  obs_debug.Clear();
  //TODO: 和AEB有关的处理统一归类到AEB判断逻辑
  // 踩油门踏板退出AEB模式
  if ((chassis_status_.acc_pedal_value > 5) || (ad_msg::VEH_DRIVING_MODE_MANUAL == chassis_status_.driving_mode) ||
      (ad_msg::VEH_DRIVING_MODE_INVALID == chassis_status_.driving_mode)) {
    req_aeb_action = false;
    status_.aeb_hold_flag = false;
    action_smoothing_aeb_action_.Clear();
  }
  
  //AEB执行之前有个hold状态（aeb_hold_flag）：障碍物持续5帧以下就消失-3m/s2减速度，否则-4m/s2减速度
  if (req_aeb_action) {
    req_aeb_action = action_smoothing_aeb_action_.Smooth(true);
    if (req_aeb_action) {
      status_.aeb_hold_flag = true;
    }
  } else {
    action_smoothing_aeb_action_.Smooth(false);
  }
  if (req_aeb_warning && !status_.aeb_hold_flag) {
    req_aeb_warning = action_smoothing_aeb_warning_.Smooth(true);
    if (req_aeb_warning) {
      AddEventReporting(EVENT_TYPE_AEB_WARNING);
    }
  } else {
    action_smoothing_aeb_warning_.Smooth(false);
  }
  if (req_notify_uncertain) {
    req_notify_uncertain = action_smoothing_notify_uncertain_.Smooth(true);
    if (req_notify_uncertain) {
      status_.exist_uncertain_obstacle = true;
      if ((ad_msg::VEH_EPS_STATUS_ROBOTIC == chassis_status_.eps_status) ||
          (ad_msg::VEH_THROTTLE_SYS_STATUS_ROBOTIC == chassis_status_.throttle_sys_status)) {
        AddEventReporting(EVENT_TYPE_UNCERTAIN_OBSTACLE);
      }
    }
  } else {
    action_smoothing_notify_uncertain_.Smooth(false);
  }

  result_of_planning_.aeb_action = req_aeb_action;
  result_of_planning_.aeb_warning = req_aeb_warning;

  Int32_t changed_trajectory_size = changed_trajectory_.points().Size();
  TargetVelocityInfo corrected_tar_v_info;
  if ((result_of_trajectory_planning_.trj_changing.is_changing) && (changed_trajectory_size > 1) &&
      (!status_.aeb_hold_flag) && (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION != tar_v_info.tar_type) &&
      (tar_v_info.obstacle_info.valid) && (tar_v_info.obstacle_info.ttc > 5.0F) &&
      (tar_v_info.obstacle_info.time_gap > 1.0F)) {
    common::PathPoint proj_on_changed_trajectory;

    changed_trajectory_.FindProjection(curr_position_.path_point.point, &proj_on_changed_trajectory);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) <<"变道轨迹进行障碍物遍历 ######";
#endif
    bool req_aeb_action_2 = false;
    bool req_aeb_warning_2 = false;
    bool req_notify_uncertain_2 = false;
    if (CalcVelocityConsideringObstacles(changed_trajectory_, proj_on_changed_trajectory, &corrected_tar_v_info,
                                         &req_aeb_action_2, &req_aeb_warning_2, &req_notify_uncertain_2, obs_debug)) {

#if PLANNING_DEBUG
  // 跟车场景
  common::com_memcpy(&(planning_debug_.cipv_lca), &(obs_debug.cipv), sizeof(planning_debug_.cipv_lca));
  planning_debug_.tar_type_following_lca = obs_debug.tar_type_following;
  planning_debug_.tar_v_following_lca = obs_debug.tar_v_following;
  planning_debug_.tar_a_following_lca = obs_debug.tar_a_following;
  planning_debug_.fsm_follow_state_lca = obs_debug.fsm_follow_state;

  // 避障场景
  common::com_memcpy(&(planning_debug_.avoidance_lca), &(obs_debug.avoidance), sizeof(planning_debug_.avoidance_lca));
  planning_debug_.tar_type_obstacle_lca = obs_debug.tar_type_obstacle;
  planning_debug_.tar_v_obstacle_lca = obs_debug.tar_v_obstacle;
  planning_debug_.tar_a_obstacle_lca = obs_debug.tar_a_obstacle;
  planning_debug_.fsm_obstacle_state_lca = obs_debug.fsm_obstacle_state;

  // 紧急制动场景
  common::com_memcpy(&(planning_debug_.mio_lca), &(obs_debug.mio), sizeof(planning_debug_.mio_lca));
  planning_debug_.tar_type_aeb_lca = obs_debug.tar_type_aeb;
  planning_debug_.tar_v_aeb_lca = obs_debug.tar_v_aeb;
  planning_debug_.tar_a_aeb_lca = obs_debug.tar_a_aeb;
#endif

      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != corrected_tar_v_info.tar_type) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        printf("[LaneChange Obstacles] tar_v=%0.1f, tar_a=%0.1f, corrected: tar_v=%0.1f, tar_a=%0.1f\n", tar_v_info.tar_v * 3.6F,
               tar_v_info.tar_a, corrected_tar_v_info.tar_v * 3.6f, corrected_tar_v_info.tar_a);
#endif
        if (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION == corrected_tar_v_info.tar_type) {
          printf("[LaneChange Obstacles] 不执行变道/避让的AEB\n");
        } else {
          // 取减速度的小值
          tar_v_info.tar_v = common::Min(tar_v_info.tar_v, corrected_tar_v_info.tar_v);
          tar_v_info.tar_a = common::Min(tar_v_info.tar_a, corrected_tar_v_info.tar_a);
          printf("[LaneChange Obstacles] 使用小的减速度\n");
        }
      } else {
        // 最优轨迹不会碰撞，不用减速
        tar_v_info.Clear();
        printf("[LaneChange Obstacles] 不用减速\n");
      }

#if PLANNING_DEBUG
    //此时的轨迹为变道轨迹
    planning_debug_.changlane_path_point_num = internal_data_.sample_points.Size();
    for (size_t i = 0; i < internal_data_.sample_points.Size(); i++) {
      planning_debug_.changlane_path.PushBack(internal_data_.sample_points[i]);
    } 
#endif

    }
  }

  const plan_var_t aeb_full_tar_a = -4.0F;
  const plan_var_t aeb_half_tar_a = -3.0F;

  if (req_aeb_action) {
    if (status_.aeb_hold_flag) { // 请求AEB full brake
      *result = tar_v_info;
      result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
      result->tar_v = 0.0F;
      result->tar_a = aeb_full_tar_a;
      result->tar_s = 0.0F;
    } else { // 请求AEB half brake
      *result = tar_v_info;
      result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
      result->tar_v = 0.0F;
      result->tar_a = aeb_half_tar_a;
      result->tar_s = 0.0F;
    }
    AddEventReporting(EVENT_TYPE_AEB_ACTION);
  } else {
    *result = tar_v_info;
  }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<<<<<< Plan Velocity AccordingTo Obstacles (End) ######";
#endif

}

bool VelocityPlanningPolynomialFitting::CalcVelocityConsideringObstacles(const common::Path &path,
                                        const common::PathPoint &proj_on_path,
                                        TargetVelocityInfo *result, bool *req_aeb_action,
                                        bool *req_aeb_warning, bool *req_notify_uncertain, ConsideringObstaclesDebug& obs_debug) {

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "#### Calc Velocity Considering Obstacles (Begin) >>>>";
#endif

  result->Clear();

  *req_aeb_action = false;
  *req_aeb_warning = false;
  *req_notify_uncertain = false;

  const plan_var_t sample_step_len = param_.sample_step_len_for_collision_test;
  const plan_var_t sample_total_time = 7.0F;
  const plan_var_t sample_min_distance = 30.0F;
  const plan_var_t collision_detection_min_ego_speed = 5.0 / 3.6F; 

  const plan_var_t acc_acc_limit = 3.0F;
  const plan_var_t acc_dec_limit = -2.0F;
  const plan_var_t aeb_tar_a = -4.0F;
  const plan_var_t aeb_tar_v = 0.0F;

#if PLANNING_DEBUG
  planning_debug_.collision_test_path_point_num = internal_data_.sample_points.Size();
  for (size_t i = 0; i < internal_data_.sample_points.Size(); i++) {
    planning_debug_.collision_test_path.PushBack(internal_data_.sample_points[i]);
  }
#endif

  // 碰撞检测沿着规划路径的最小长度
  const plan_var_t min_sample_path_len = common::Max(sample_total_time * curr_position_.v, sample_min_distance);

  // 沿着规划路径等距离采样
  plan_var_t forward_path_len = path.total_length() - proj_on_path.s;
  if (forward_path_len < min_sample_path_len) {
    forward_path_len = min_sample_path_len;
  }
  Int32_t sample_num = common::com_round(forward_path_len / sample_step_len) + 1;
  internal_data_.sample_points.Clear();
  if (!path.UniformlySamplePathForward(proj_on_path.s, sample_num, sample_step_len, &internal_data_.sample_points)) {
    LOG_ERR << "Failed to sample path for collision testing.";
    return false;
  }
  Int32_t sample_size = internal_data_.sample_points.Size();

#if PLANNING_DEBUG
  planning_debug_.collision_test_samples_point_num = internal_data_.sample_points.Size();
  for (size_t i = 0; i < internal_data_.sample_points.Size(); i++) {
    planning_debug_.collision_test_samples.PushBack(internal_data_.sample_points[i]);
  }
#endif

  if (sample_size < 1) {
    LOG_ERR << "Failed to sample path for collision testing.";
    return false;
  }

  // 以自车几何形状沿规划路径按采样点计算碰撞风险信息
  driv_map::CollisionTestOnPathObj test_obj;
  test_obj.t_offset = 0.0F;
  test_obj.s_offset = 0.0F;
  test_obj.x_offset = param_.dist_of_localization_to_center;
  test_obj.obj_half_length = 0.5F * param_.vehicle_length;
  test_obj.obj_half_width = 0.5F * param_.vehicle_width;
  plan_var_t cur_v_for_plan = curr_position_.v;
  if (cur_v_for_plan < collision_detection_min_ego_speed) {
    cur_v_for_plan = collision_detection_min_ego_speed;
  }
  test_obj.obj_v = cur_v_for_plan;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    test_obj.is_backing = true;
  } else {
    test_obj.is_backing = false;
  }

  driv_map::CollisionTestOnPathResult &collison_test_ret = internal_data_.collision_test_on_path_result;
  Int32_t risk_value =
      driving_map_->TestCollisionOnPath(test_obj, path, internal_data_.sample_points, &collison_test_ret);

  // 根据障碍物关系决策速度规划
  Int32_t risky_obj_num = collison_test_ret.risky_obj_list.Size();
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "有风险的障碍物, num = " << risky_obj_num;
#endif
  
  for (Int32_t i = 0; i < risky_obj_num; ++i) {
    const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info = collison_test_ret.risky_obj_list[i];
    const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
    const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "index = " << i << ", ID = " << tar_obj.id <<", collison static distance = " << test_ret.static_distance;
#endif
  
    ClassifyTargetByObstacle(false, cur_v_for_plan, rsk_obj_info, obstacle_decision_list_[test_ret.obj_list_index]);
  }

  tar_v_obstacle_list_.Clear();
  TargetVelocityInfo tar_v_obstacle;
  // 根据障碍物决策情况(打的标签)，分别处理
  PlanVelocityAccordingToFollowing(path, proj_on_path, tar_v_obstacle, obs_debug);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_obstacle.tar_type) {
    tar_v_obstacle_list_.PushBack(tar_v_obstacle);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[Following] tar_type = " << tar_v_obstacle.tar_type << ", tar_v = " << tar_v_obstacle.tar_v << "m/s("
              << tar_v_obstacle.tar_v * 3.6 << "km/h), tar_a = " << tar_v_obstacle.tar_a << "m/s^2";
#endif    
  }

  PlanVelocityAccordingToAvoidingCollision(path, proj_on_path, tar_v_obstacle, obs_debug);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_obstacle.tar_type) {
    tar_v_obstacle_list_.PushBack(tar_v_obstacle);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[Avoiding] tar_type = " << tar_v_obstacle.tar_type << ", tar_v = " << tar_v_obstacle.tar_v << "m/s("
              << tar_v_obstacle.tar_v * 3.6 << "km/h), tar_a = " << tar_v_obstacle.tar_a << "m/s^2";
#endif    
  }

  PlanVelocityAccordingToEmergencyWarning(path, proj_on_path, tar_v_obstacle);
    if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_obstacle.tar_type) {
    tar_v_obstacle_list_.PushBack(tar_v_obstacle);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[AEB-WARNING] tar_type = " << tar_v_obstacle.tar_type << ", tar_v = " << tar_v_obstacle.tar_v << "m/s("
              << tar_v_obstacle.tar_v * 3.6 << "km/h), tar_a = " << tar_v_obstacle.tar_a << "m/s^2";
#endif    
  }

  PlanVelocityAccordingToEmergencyBrake(path, proj_on_path, tar_v_obstacle, obs_debug);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_obstacle.tar_type) {
    tar_v_obstacle_list_.PushBack(tar_v_obstacle);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[AEB] tar_type = " << tar_v_obstacle.tar_type << ", tar_v = " << tar_v_obstacle.tar_v << "m/s("
              << tar_v_obstacle.tar_v * 3.6 << "km/h), tar_a = " << tar_v_obstacle.tar_a << "m/s^2";
#endif    
  }

  // TODO: 需要完备的定义障碍物场景的处理边界
  TargetVelocityInfo tar_v_info_obj;
  Int32_t tar_v_info_list_size = tar_v_obstacle_list_.Size();
  Int32_t tar_v_obstacle_index = 0;
  bool is_has_obstacle = false;

  if (tar_v_info_list_size > 0) {
    is_has_obstacle = true;
    for (Int32_t i = 1; i < tar_v_info_list_size; ++i) {
      if (tar_v_obstacle_list_[i].tar_v < tar_v_obstacle_list_[tar_v_obstacle_index].tar_v) {
        tar_v_obstacle_index = i;
      }
    }
  }
  
  if (is_has_obstacle) {
    tar_v_info_obj = tar_v_obstacle_list_[tar_v_obstacle_index];
#if PLANNING_DEBUG
    planning_debug_.tar_type_real_obstacle = tar_v_info_obj.tar_type;
    planning_debug_.tar_v_real_obstacle = tar_v_info_obj.tar_v;
    planning_debug_.tar_a_real_obstacle = tar_v_info_obj.tar_a;
#endif

  } else {
    tar_v_info_obj.Clear();
  }


  if (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION == tar_v_info_obj.tar_type) {
    *req_aeb_action = true;
  }
  if (VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN == tar_v_info_obj.tar_type) {
    *req_aeb_warning = true;
  }

#if ENABLE_UNCERTAIN_OBSTACLE

  TargetVelocityInfo tar_v_info_uncertain;
  if (calc_result.is_uncertain) {
    UpdateTargetVelocityInfo(calc_result.tar_velocity_info, &tar_v_info_uncertain);
  }

  Int32_t uncertain_num = collison_test_ret.uncertain_list.Size();
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Calc Velocity Considering Obstacles] 不确定的障碍物, num=" << uncertain_num;
#endif
  
  for (Int32_t i = 0; i < uncertain_num; ++i) {
    const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info = collison_test_ret.uncertain_list[i];
    const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
    const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);

    if (test_ret.static_distance > 0.5) {
      continue;
    }

    ClassifyTargetByObstacle(true, cur_v_for_plan,
                                             rsk_obj_info, obstacle_decision_list_[test_ret.obj_list_index]);

    tar_v_info_uncertain.obstacle_info.dist_gap_level = tar_obj.id;
    UpdateTargetVelocityInfo(calc_result.tar_velocity_info, &tar_v_info_uncertain);
  }

  if ((ad_msg::VEH_DRIVING_MODE_ROBOTIC == chassis_status_.driving_mode) &&
      (ad_msg::VEH_EPS_STATUS_ROBOTIC == chassis_status_.eps_status) && (result_of_action_planning_.enable_lka)) {
    // 方向盘自动控制下
  } else {
    // 方向盘非自动控制下, 不处理不确定的障碍物
    tar_v_info_uncertain.Clear();
  }

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info_uncertain.tar_type) {
    if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info_obj.tar_type) {
      if ((tar_v_info_uncertain.tar_a_true < tar_v_info_obj.tar_a) &&
          (tar_v_info_uncertain.obstacle_info.dist_to_obj < (tar_v_info_obj.obstacle_info.dist_to_obj + 0.5F))) {
        *req_notify_uncertain = true;
      }
    } else {
      *req_notify_uncertain = true;
    }
  }
#endif

  if (status_.aeb_hold_flag) {
    *result = tar_v_info_obj;
    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
    result->tar_v = 0.0F;
    result->tar_a = common::Min(param_.aeb_target_acc_II, result->tar_a);
    result->tar_s = 0.0F;
    result->tar_pos = proj_on_path;
  } else {
    if (tar_v_info_obj.tar_a > param_.obsacle_final_acc_ax_limit) {
      tar_v_info_obj.tar_a = param_.obsacle_final_acc_ax_limit;
    } else if (tar_v_info_obj.tar_a < param_.obsacle_final_dec_ax_limit) {
      tar_v_info_obj.tar_a = param_.obsacle_final_dec_ax_limit;
    }
    *result = tar_v_info_obj;
  }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACEW
  LOG_INFO(5) << "<<<< Calc Velocity Considering Obstacles (End) ####";
#endif

  return true;
}

bool VelocityPlanningPolynomialFitting::IsFollowingObject(const ObstacleDecision &decision, float distance, float l_distance) {
  const plan_var_t lateral_distance_range_of_front = 2.0F;
  const plan_var_t min_obj_confidence = 80.0F;

  if (decision.obb_distance > lateral_distance_range_of_front) {
    return false;
  }
  if (decision.perception_obstacle.confidence < min_obj_confidence) {
    return false;
  }
  /*
  //TODO: 感知无法保证相机不会漏检，所以不能只考虑融合障碍物，同时导致存在跟踪误检障碍物的可能，比如雷达的ghost或分裂目标
  if (ad_msg::OBJ_PRCP_TYPE_FUSED != decision.perception_obstacle.perception_type) {
    // 只跟随融合的障碍物
    return false;
  }
  //TODO: 是否存在分类错误的情况，概率多少?
  if ((ad_msg::OBJ_TYPE_PASSENGER_VEHICLE != decision.perception_obstacle.type) &&
      (ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE != decision.perception_obstacle.type) &&
      (ad_msg::OBJ_TYPE_SPECIAL_VEHICLE != decision.perception_obstacle.type) && (ad_msg::OBJ_TYPE_OTHER_VEHICLE != decision.perception_obstacle.type) &&
      (ad_msg::OBJ_TYPE_BICYCLE != decision.perception_obstacle.type) && (ad_msg::OBJ_TYPE_PEDESTRIAN != decision.perception_obstacle.type)) {
      // 只跟随车辆
      return false;
  }
  */

  // if (driv_map::OBJ_POSITION_FRONT != decision.obj_position) {
  //   return false;
  // }

  // TODO: 驾驶地图障碍物[object_map_impl.cc]方位分类是默认感知距离是以后轴中心进行判断的
  //       在close方位下有bug
  if (driv_map::OBJ_POSITION_FRONT != decision.obj_position) {
    if (driv_map::OBJ_POSITION_CLOSE == decision.obj_position) {
      if (!(distance > 0.2 && l_distance < 1.0)) { //规划定位中心点在后轴中心
        return false;
      } else {
        // do nothing
      }  
    } else {
      return false;
    }
  }

  if (driv_map::OBJ_DIRECTION_FORWARD != decision.obj_direction) {
    return false;
  }

  return true;
}

void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToFollowing(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info, ConsideringObstaclesDebug& obs_debug) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "#### Plan Velocity AccordingTo Following (begin) ####";
#endif
  tar_v_info.Clear();
  const plan_var_t following_low_speed_threshold = 15.0F / 3.6F;

  // 从可以跟随的目标中筛选出CIPV
  Int32_t tar_following_cipv_idx = -1;
  for (Int32_t i = 0; i < obstacle_decision_num_; ++i) {
    const ObstacleDecision& decision = obstacle_decision_list_[i];
    if (decision.can_following) {
      if (tar_following_cipv_idx < 0) {
        tar_following_cipv_idx = i;
      } else {
        if (decision.dist_to_obj < obstacle_decision_list_[tar_following_cipv_idx].dist_to_obj) {
          tar_following_cipv_idx = i;
        }
      }
    }
  }

  if (-1 == tar_following_cipv_idx) { // 未找到CIPV
    tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    return;
  } else {
    ObstacleDecision& tar_following_cipv = obstacle_decision_list_[tar_following_cipv_idx];
    tar_following_cipv.is_cipv = true;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "Following CIPV index = " << tar_following_cipv_idx;
    LOG_INFO(5) << "Following : ID = " << tar_following_cipv.perception_obstacle.id 
            << ", X = " << tar_following_cipv.perception_obstacle.x
            << "m, Y = " << tar_following_cipv.perception_obstacle.y
            << "m, Vx = " << tar_following_cipv.perception_obstacle.v_x 
            << "m/s, Vy = " << tar_following_cipv.perception_obstacle.v_y
            << "m/s, AX =" << tar_following_cipv.perception_obstacle.a_x
            << "m/s^2, Ay =" << tar_following_cipv.perception_obstacle.a_y
            << "m/s^2, V =" << tar_following_cipv.perception_obstacle.v
            << "m/s, A =" << tar_following_cipv.perception_obstacle.a<< "m/s^2";
    LOG_INFO(5) << "X center to center = " << tar_following_cipv.perception_obstacle.obb.x
            << "m, Y center to center = " << tar_following_cipv.perception_obstacle.obb.y
            << "m, heading center to center= " << tar_following_cipv.perception_obstacle.obb.heading
            << "rad, obstacle width = "<< tar_following_cipv.perception_obstacle.obb.half_width * 2.0 << "m";
    LOG_INFO(5) << "obstacle length = "<< tar_following_cipv.perception_obstacle.obb.half_length * 2.0
            << "m, obstacle s on reference line = "<< tar_following_cipv.perception_obstacle.proj_on_major_ref_line.s
            << "m, obstacle l on reference line = "<< tar_following_cipv.perception_obstacle.proj_on_major_ref_line.l
            << "m, obstacle obb to obb = "<< tar_following_cipv.obb_distance;
#endif

    tar_v_info.obstacle_info.valid = true;
    tar_v_info.obstacle_info.dist_to_obj = tar_following_cipv.planning_target_s;
    tar_v_info.obstacle_info.time_gap = tar_following_cipv.time_gap;
    tar_v_info.obstacle_info.relative_v = tar_following_cipv.perception_obstacle.v - curr_position_.v;
    tar_v_info.obstacle_info.ttc = tar_following_cipv.ttc;
    tar_v_info.obstacle_info.obj_v = tar_following_cipv.perception_obstacle.v;
    tar_v_info.obstacle_info.dist_gap_level = tar_following_cipv.perception_obstacle.id;
    tar_v_info.obstacle_info.rel_spd_level = tar_following_cipv.obb_distance;
    tar_v_info.obstacle_info.velocity_limit = tar_following_cipv.perception_obstacle.v;
    tar_v_info.obstacle_info.safe_dist = tar_following_cipv.safe_dist;
    tar_v_info.tar_s = tar_v_info.obstacle_info.dist_to_obj;
    // 3.0 秒跟车时距
    path.FindSmoothPoint(proj_on_path.s + common::Max(0.0F, tar_v_info.obstacle_info.obj_v * 3.0F),
                       &(tar_v_info.tar_pos));

    //TODO: 如何定义低速跟随
    if (curr_position_.v < following_low_speed_threshold && 
        tar_following_cipv.perception_obstacle.v > -5.0F/3.6F)  {  // 低速跟车，强调快速响应，需要处理频繁的启停，切入切出
      ParamOfPolynomialFitting param_of_calc_tar_v_ax;
      param_of_calc_tar_v_ax.is_cc = false;
      param_of_calc_tar_v_ax.tf = settings_.time_gap_setting;//只在低速跟车或跟车模式下，才能使用时距调整
      param_of_calc_tar_v_ax.jerk_cost = 0.4;
      param_of_calc_tar_v_ax.v_cost = 0.4;
      param_of_calc_tar_v_ax.s_cost = 0.2;
      param_of_calc_tar_v_ax.preview_time = 0.2;
      param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;

      StateOfPolynomial param_of_polynomial_start_end_state;
      param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
      param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
      param_of_polynomial_start_end_state.final_state_vx = tar_following_cipv.planning_target_v;
      param_of_polynomial_start_end_state.final_state_distance = tar_following_cipv.planning_target_s;

      CalculateVelocityAccordingToObstacle(tar_following_cipv, param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, tar_v_info);

      tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING;

    } else { // 高速跟车，更强调舒适性，避免非预期制动，急加速，急减速
      ParamOfPolynomialFitting param_of_calc_tar_v_ax;
      param_of_calc_tar_v_ax.is_cc = false;
      param_of_calc_tar_v_ax.tf = settings_.time_gap_setting;//只在低速跟车或跟车模式下，才能使用时距调整
      param_of_calc_tar_v_ax.jerk_cost = 0.4;
      param_of_calc_tar_v_ax.v_cost = 0.4;
      param_of_calc_tar_v_ax.s_cost = 0.2;
      param_of_calc_tar_v_ax.preview_time = 0.2;
      param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;

      StateOfPolynomial param_of_polynomial_start_end_state;
      param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
      param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
      param_of_polynomial_start_end_state.final_state_vx = tar_following_cipv.planning_target_v;
      param_of_polynomial_start_end_state.final_state_distance = tar_following_cipv.planning_target_s;

      CalculateVelocityAccordingToObstacle(tar_following_cipv, param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, tar_v_info);

      tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING;
    }
    
#if PLANNING_DEBUG
    common::com_memcpy(&(obs_debug.cipv), &tar_following_cipv, sizeof(obs_debug.cipv));
    obs_debug.tar_type_following = tar_v_info.tar_type;
    obs_debug.tar_v_following = tar_v_info.tar_v;
    obs_debug.tar_a_following = tar_v_info.tar_a;
    obs_debug.fsm_follow_state = tar_v_info.obstacle_info.fsm_state;
#endif
  }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<<<< Plan Velocity AccordingTo Following (end) ####";
#endif
}

void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToAvoidingCollision(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info, ConsideringObstaclesDebug& obs_debug) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "#### Plan Velocity AccordingTo AvoidingCollision (begin) ####";
#endif
  tar_v_info.Clear();
  Int32_t tar_avoiding_mio_idx = -1;
  // 找到最紧急的需要避障的目标
  for (Int32_t i = 0; i < obstacle_decision_num_; ++i) {
    const ObstacleDecision& decision = obstacle_decision_list_[i];
    if (decision.need_avoiding) {
      if (tar_avoiding_mio_idx < 0) {
        tar_avoiding_mio_idx = i;
      } else {
        if (decision.ttc < obstacle_decision_list_[tar_avoiding_mio_idx].ttc) {
          tar_avoiding_mio_idx = i;
        }
      }
    }
  }

  if (-1 == tar_avoiding_mio_idx) { // 未找到MIO
    tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    return;
  } else {
    ObstacleDecision& tar_avoiding_mio = obstacle_decision_list_[tar_avoiding_mio_idx];
    tar_avoiding_mio.is_mio = true;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "## Avoiding mio index = " << tar_avoiding_mio_idx << ", TTC = " << tar_avoiding_mio.ttc;
    LOG_INFO(5) << "## Avoiding : ID = " << tar_avoiding_mio.perception_obstacle.id 
            << ", X = " << tar_avoiding_mio.perception_obstacle.x
            << "m, Y = " << tar_avoiding_mio.perception_obstacle.y
            << "m, Vx = " << tar_avoiding_mio.perception_obstacle.v_x 
            << "m/s, Vy = " << tar_avoiding_mio.perception_obstacle.v_y
            << "m/s, AX =" << tar_avoiding_mio.perception_obstacle.a_x
            << "m/s^2, Ay =" << tar_avoiding_mio.perception_obstacle.a_y
            << "m/s^2, V =" << tar_avoiding_mio.perception_obstacle.v
            << "m/s, A =" << tar_avoiding_mio.perception_obstacle.a<< "m/s^2";
    LOG_INFO(5) << "## X center to center = " << tar_avoiding_mio.perception_obstacle.obb.x
            << "m, Y center to center = " << tar_avoiding_mio.perception_obstacle.obb.y
            << "m, heading center to center= " << tar_avoiding_mio.perception_obstacle.obb.heading
            << "rad, obstacle width = "<< tar_avoiding_mio.perception_obstacle.obb.half_width * 2.0 << "m";
    LOG_INFO(5) << "## obstacle length = "<< tar_avoiding_mio.perception_obstacle.obb.half_length * 2.0
            << "m, obstacle s on reference line = "<< tar_avoiding_mio.perception_obstacle.proj_on_major_ref_line.s
            << "m, obstacle l on reference line = "<< tar_avoiding_mio.perception_obstacle.proj_on_major_ref_line.l
            << "m, obstacle obb to obb = "<< tar_avoiding_mio.obb_distance;
#endif
    tar_v_info.obstacle_info.valid = true;
    tar_v_info.obstacle_info.dist_to_obj = tar_avoiding_mio.planning_target_s;
    tar_v_info.obstacle_info.time_gap = tar_avoiding_mio.time_gap;
    tar_v_info.obstacle_info.relative_v = tar_avoiding_mio.perception_obstacle.v - curr_position_.v;
    tar_v_info.obstacle_info.ttc = tar_avoiding_mio.ttc;
    tar_v_info.obstacle_info.obj_v = tar_avoiding_mio.perception_obstacle.v;
    tar_v_info.obstacle_info.dist_gap_level = tar_avoiding_mio.perception_obstacle.id;
    tar_v_info.obstacle_info.rel_spd_level = tar_avoiding_mio.obb_distance;
    tar_v_info.obstacle_info.velocity_limit = tar_avoiding_mio.perception_obstacle.v;
    tar_v_info.obstacle_info.safe_dist = tar_avoiding_mio.safe_dist;

    tar_v_info.tar_s = tar_v_info.obstacle_info.dist_to_obj;
    // 3.0 秒跟车时距
    path.FindSmoothPoint(proj_on_path.s + common::Max(0.0F, tar_v_info.obstacle_info.obj_v * 3.0F), &(tar_v_info.tar_pos));

    //TODO: 如何在跟车和避障之间做严格的定义
    ParamOfPolynomialFitting param_of_calc_tar_v_ax;
    param_of_calc_tar_v_ax.is_cc = false;
    param_of_calc_tar_v_ax.tf = 3.0;
    param_of_calc_tar_v_ax.jerk_cost = 0.4;
    param_of_calc_tar_v_ax.v_cost = 0.4;
    param_of_calc_tar_v_ax.s_cost = 0.2;
    param_of_calc_tar_v_ax.preview_time = 0.2;
    param_of_calc_tar_v_ax.polynomial_order_is_quintic = true;

    StateOfPolynomial param_of_polynomial_start_end_state;
    param_of_polynomial_start_end_state.initial_state_vx = curr_position_.v;
    param_of_polynomial_start_end_state.initial_state_ax = chassis_status_.a;
    param_of_polynomial_start_end_state.final_state_vx = tar_avoiding_mio.planning_target_v;
    param_of_polynomial_start_end_state.final_state_distance = tar_avoiding_mio.planning_target_s;

    CalculateVelocityAccordingToObstacle(tar_avoiding_mio, param_of_calc_tar_v_ax, param_of_polynomial_start_end_state, tar_v_info);

    tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE;

#if PLANNING_DEBUG
    common::com_memcpy(&(obs_debug.avoidance), &tar_avoiding_mio, sizeof(obs_debug.avoidance));
    obs_debug.tar_type_obstacle = tar_v_info.tar_type;
    obs_debug.tar_v_obstacle = tar_v_info.tar_v;
    obs_debug.tar_a_obstacle = tar_v_info.tar_a;
    obs_debug.fsm_obstacle_state = tar_v_info.obstacle_info.fsm_state;
#endif

  }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<<<< Plan Velocity AccordingTo AvoidingCollision (end) ####";
#endif
}

void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToEmergencyWarning(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "#### Plan Velocity AccordingTo EmergencyWarning (begin) ####";
#endif  
  tar_v_info.Clear();
  Int32_t tar_mio_idx = -1;
  // 在所有风险障碍物中选择最紧急的需要报警的目标
  for (Int32_t i = 0; i < obstacle_decision_num_; ++i) {
    const ObstacleDecision& decision = obstacle_decision_list_[i];
    if (decision.need_aeb_warning) {
      if (tar_mio_idx < 0) {
        tar_mio_idx = i;
      } else {
        if (decision.ttc < obstacle_decision_list_[tar_mio_idx].ttc) {
          tar_mio_idx = i;
        }
      }
    }
  }
  
  if (-1 == tar_mio_idx) { // 未找到MIO
    tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    return;
  } else {
    ObstacleDecision& tar_mio = obstacle_decision_list_[tar_mio_idx];

    tar_v_info.obstacle_info.valid = true;
    tar_v_info.obstacle_info.dist_to_obj = tar_mio.planning_target_s;
    tar_v_info.obstacle_info.time_gap = tar_mio.time_gap;
    tar_v_info.obstacle_info.relative_v = tar_mio.perception_obstacle.v - curr_position_.v;
    tar_v_info.obstacle_info.ttc = tar_mio.ttc;
    tar_v_info.obstacle_info.obj_v = tar_mio.perception_obstacle.v;
    tar_v_info.obstacle_info.dist_gap_level = tar_mio.perception_obstacle.id;
    tar_v_info.obstacle_info.rel_spd_level = tar_mio.obb_distance;
    tar_v_info.obstacle_info.velocity_limit = tar_mio.perception_obstacle.v;
    tar_v_info.obstacle_info.safe_dist = tar_mio.safe_dist;

    tar_v_info.tar_s = tar_v_info.obstacle_info.dist_to_obj;
    path.FindSmoothPoint(proj_on_path.s, &(tar_v_info.tar_pos));


    if (settings_.enable_aeb_pre_dec) {
      tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN;

      tar_v_info.tar_v = chassis_status_.v - param_.aeb_target_detla_v;
      if (tar_v_info.tar_v < 10.0F / 3.6F) {
        tar_v_info.tar_v = 10.0F / 3.6F;
      }
      tar_v_info.tar_a = param_.aeb_target_acc_I;

      LOG_INFO(5) << "[Plan Velocity AccordingToE mergencyWarning] 请求AEB预减速 ttc = " << tar_v_info.obstacle_info.ttc<< ", obstacle ID = " 
      << tar_v_info.obstacle_info.dist_gap_level;

    } else {
      tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    }
  }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<<<< Plan Velocity AccordingTo EmergencyWarning (end) ####";
#endif  
}

void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToEmergencyBrake(const common::Path &path, const common::PathPoint &proj_on_path, TargetVelocityInfo &tar_v_info, ConsideringObstaclesDebug& obs_debug) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "#### Plan Velocity AccordingTo EmergencyBrake (begin) ####";
#endif
  tar_v_info.Clear();
  Int32_t tar_mio_idx = -1;
  // 在所有风险障碍物中选择最紧急的需要报警的目标
  for (Int32_t i = 0; i < obstacle_decision_num_; ++i) {
    const ObstacleDecision& decision = obstacle_decision_list_[i];
    if (decision.need_aeb_full_brake) {
      if (tar_mio_idx < 0) {
        tar_mio_idx = i;
      } else {
        if (decision.ttc < obstacle_decision_list_[tar_mio_idx].ttc) {
          tar_mio_idx = i;
        }
      }
    }
  }
  
  if (-1 == tar_mio_idx) { // 未找到MIO
    tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    return;
  } else {
    ObstacleDecision& tar_mio = obstacle_decision_list_[tar_mio_idx];

    tar_v_info.obstacle_info.valid = true;
    tar_v_info.obstacle_info.dist_to_obj = tar_mio.planning_target_s;
    tar_v_info.obstacle_info.time_gap = tar_mio.time_gap;
    tar_v_info.obstacle_info.relative_v = tar_mio.perception_obstacle.v - curr_position_.v;
    tar_v_info.obstacle_info.ttc = tar_mio.ttc;
    tar_v_info.obstacle_info.obj_v = tar_mio.perception_obstacle.v;
    tar_v_info.obstacle_info.dist_gap_level = tar_mio.perception_obstacle.id;
    tar_v_info.obstacle_info.rel_spd_level = tar_mio.obb_distance;
    tar_v_info.obstacle_info.velocity_limit = tar_mio.perception_obstacle.v;
    tar_v_info.obstacle_info.safe_dist = tar_mio.safe_dist;

    tar_v_info.tar_s = tar_v_info.obstacle_info.dist_to_obj;
    path.FindSmoothPoint(proj_on_path.s, &(tar_v_info.tar_pos));

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Plan Velocity AccordingTo EmergencyBrake] 请求AEB减速 ttc = " << tar_v_info.obstacle_info.ttc << ", obstacle ID = " 
                << tar_v_info.obstacle_info.dist_gap_level;
#endif

    tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
    tar_v_info.tar_v = 0.0F;
    tar_v_info.tar_a = param_.aeb_target_acc_II;

#if PLANNING_DEBUG
    obs_debug.tar_type_aeb = tar_v_info.tar_type;
    obs_debug.tar_v_aeb = tar_v_info.tar_v;
    obs_debug.tar_a_aeb = tar_v_info.tar_a;
    common::com_memcpy(&(obs_debug.mio), &tar_mio, sizeof(obs_debug.mio));
#endif
  }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<<<< Plan Velocity AccordingTo EmergencyBrake (end)#####";
#endif
}

// 更新规划的速度TargetVelocityInfo
bool VelocityPlanningPolynomialFitting::UpdateTargetVelocityInfo(const TargetVelocityInfo &new_info,
                                                        TargetVelocityInfo *const old_info) const {
  bool update_flag = false;

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == old_info->tar_type) {
    *old_info = new_info;
    update_flag = true;
  } else {
    plan_var_t min_v = common::Min(new_info.tar_v, old_info->tar_v);
    plan_var_t min_a = common::Min(new_info.tar_a, old_info->tar_a);
    if (new_info.tar_v < old_info->tar_v) {
      *old_info = new_info;
      update_flag = true;
    } else if (new_info.tar_a < old_info->tar_a) {
      *old_info = new_info;
      update_flag = true;
    } else {
      // nothing to do
    }
    old_info->tar_v = min_v;
    old_info->tar_a = min_a;
  }

  return (update_flag);
}

// 目标障碍物分类
void VelocityPlanningPolynomialFitting::ClassifyTargetByObstacle(bool is_uncertain, plan_var_t cur_v_for_plan, const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info, ObstacleDecision& decision) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "## 目标决策 Classify Target By Obstacle (begin) >>";
#endif
  const plan_var_t lateral_distance_need_process = 2.0F;
  const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
  const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);

  // 从碰撞检测模块获取障碍物基本数据
  decision.Clear();
  GetObstacleDecisionData(rsk_obj_info, decision);

  // 粗略筛选需要处理的障碍物
  if (is_uncertain) {
    if (test_ret.static_distance > 0.01F) {
      decision.need_process = false;
    } else {
      decision.need_process = true;
    }
  } else {
    if (test_ret.static_distance > lateral_distance_need_process) {
      decision.need_process = false;
    } else {
      decision.need_process = true;
    }
  }

  // 确定安全距离
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    decision.safe_dist = param_.safe_distance_in_backing_mode;
  } else {
    if (tar_obj.dynamic) {
      decision.safe_dist = common::Max(
          param_.safe_time_to_dynamic_obstacle * curr_position_.v, param_.safe_distance_to_dynamic_obstacle);
    } else {
      decision.safe_dist = param_.safe_distance_to_static_obstacle;
    }
  }

  if (tar_obj.dynamic) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "Step 1 : 动态";
#endif
    //如果是动态障碍物，根据目标9个方位区域，判断是否need_ctrl（障碍物分类限速）
    switch (rsk_obj_info.obj_position) {
      // 前方障碍物考虑逆向，不考虑已跟随目标前方的车辆
      case (driv_map::OBJ_POSITION_FRONT): {
        if (rsk_obj_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "Step 2 : 正前方障碍物,逆向行驶";
#endif
          /// TODO: 减小逆向车道的车辆对本车道的影响(待测试)
          if (test_ret.static_distance < 0.1F) {
            decision.need_process = true;
            decision.planning_target_v = 0.0F;
            decision.planning_target_s = common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          }
        } else { 
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "Step 2 : 正前方障碍物，同向行驶";
#endif
          decision.need_process = true;

          decision.planning_target_v = tar_obj.v;
          decision.planning_target_s = rsk_obj_info.dist_to_tar_obj;
          if (settings_.enable_following) {
            decision.can_following = true;
          }
        }
      } break;

        // 侧前方目标考虑横向切入，同向与逆向切入的；同向的不切入的，本车要限速
      case (driv_map::OBJ_POSITION_LEFT_FRONT):
        // 区域2：左前方障碍物
      case (driv_map::OBJ_POSITION_RIGHT_FRONT): {
        // 区域8：右前方障碍物
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) <<  "Step 2 : 侧前方障碍物";
#endif
        if (rsk_obj_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
          // 侧前方障碍物,逆向行驶

          if (rsk_obj_info.cut_in) {
            // 障碍物将切入车道
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) <<  "侧方障碍物切入，逆向行驶";
#endif
            decision.need_process = true;
            decision.planning_target_v = 0.0F;
            decision.planning_target_s = common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          } else {
            // 侧前方障碍物,不会切入车道,忽略
            decision.need_process = false;
          }
        } else if (rsk_obj_info.obj_direction == driv_map::OBJ_DIRECTION_CROSSED) {
          // 侧前方障碍物,横向切入车道
          if (test_ret.risk_value > 50) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) <<  "侧方障碍物，贴近行驶，风险值大于 50";
#endif   
            decision.need_process = true;
            decision.planning_target_v = 0.0F;
            decision.planning_target_s =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          } else {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "侧方障碍物，风险不大，不需受控";
#endif 
            decision.need_process = false;
          }

        } else {
          if (rsk_obj_info.cut_in) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "侧前方障碍物，同向行驶，有切入风险";
#endif
            decision.need_process = true;
            decision.planning_target_v = tar_obj.v;
            decision.planning_target_s =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          } else {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "侧前方障碍物，同向行驶，限制相对速度(同向行驶，贴线场景)";
#endif
            decision.need_process = true;
            plan_var_t t = 0;
            Int32_t lower = common::LerpInOrderedTable(param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
                                                       test_ret.static_distance, &t);
            plan_var_t relative_velocity_limit =
                common::Lerp(param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[lower].value,
                             param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[lower + 1].value, t);
            plan_var_t abs_angle_diff =
                common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
            decision.planning_target_v =
                tar_obj.v /* common::com_cos(abs_angle_diff)*/ + relative_velocity_limit;//动态：在障碍物速度基础上+抬高的相对速度
            decision.planning_target_s =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
            LOG_INFO(5) << "lat_dist=" << test_ret.static_distance << ", obj_v=" << tar_obj.v * 3.6F
                      << ", rel_v_limit=" << relative_velocity_limit * 3.6F
                      << ", abs_angle_diff=" << common::com_rad2deg(abs_angle_diff)
                      << ", cos(abs_angle_diff)=" << common::com_cos(abs_angle_diff)
                      << ", v_limit=" << decision.planning_target_v * 3.6F;
#endif
          }
        }
      } break;
      
      // 区域9：紧靠车辆的障碍物
      case (driv_map::OBJ_POSITION_CLOSE): {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 紧靠车辆的障碍物";
#endif
      // TODO: 驾驶地图障碍物[object_map_impl.cc]方位分类是默认感知距离是以后轴中心进行判断的
      //       在close方位下有bug
        //如果与紧靠的障碍物距离＞定位点与前轴距离
        if ((rsk_obj_info.dist_to_tar_obj) > 0.2F) {
          decision.need_process = true;
          decision.planning_target_v = tar_obj.v;
          decision.planning_target_s = rsk_obj_info.dist_to_tar_obj;
        } else if ((rsk_obj_info.dist_to_tar_obj) <= 0.2F &&
                   (rsk_obj_info.dist_to_tar_obj) > 0.0F ) {
          decision.need_process = true;
          decision.planning_target_v = 0.0F;
          decision.planning_target_s = 0.0F;
        } else {
          decision.need_process = false;
        }
      } break;
      // 区域3：左侧
      case (driv_map::OBJ_POSITION_LEFT): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) <<"Step 2 : 左侧障碍物，忽略";
#endif
      } break;
      // 区域7：右侧
      case (driv_map::OBJ_POSITION_RIGHT): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 右侧障碍物，忽略";
#endif
      } break;
      // 区域4：左后方
      case (driv_map::OBJ_POSITION_LEFT_BACK): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 左后方障碍物，忽略";
#endif
      } break;
      // 区域5：正后方
      case (driv_map::OBJ_POSITION_BACK): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 正后方障碍物，忽略";
#endif
      } break;
      // 区域6：右后方
      case (driv_map::OBJ_POSITION_RIGHT_BACK): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 右后方障碍物，忽略";
#endif
      } break;
      default:
        break;
    }
  } else {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "Step 1 : 静态/低速";
#endif
    switch (rsk_obj_info.obj_position) {
      case (driv_map::OBJ_POSITION_FRONT): {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 正前方障碍物";
#endif
        decision.need_process = true;
        decision.planning_target_v = tar_obj.v;
        decision.planning_target_s = rsk_obj_info.dist_to_tar_obj;
        if (settings_.enable_low_speed_following && (tar_obj.v_x > -5.0F / 3.6F)) {
          decision.can_following = true;
          decision.safe_dist = param_.safe_distance_for_low_speed_following;
          decision.planning_target_s = rsk_obj_info.dist_to_tar_obj - decision.safe_dist;
        }
      } break;
      case (driv_map::OBJ_POSITION_CLOSE): {
        // 紧靠车辆的障碍物
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 紧靠车辆的障碍物";
#endif
        // TODO: 驾驶地图障碍物[object_map_impl.cc]方位分类是默认感知距离是以后轴中心进行判断的
        //       在close方位下有bug
        if ((rsk_obj_info.dist_to_tar_obj) > 0.0F) {
          decision.need_process = true;
          decision.planning_target_v = 0.0F;
          decision.planning_target_s = rsk_obj_info.dist_to_tar_obj;
        } else {
          decision.need_process = false;
        }
      } break;
      case (driv_map::OBJ_POSITION_LEFT_FRONT):
        // 左前方障碍物
      case (driv_map::OBJ_POSITION_RIGHT_FRONT): {
        // 右前方障碍物
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 侧前方障碍物";
#endif
        decision.need_process = true;
        plan_var_t t = 0;
        Int32_t lower = common::LerpInOrderedTable(param_.relative_velocity_limit_table_by_dist_to_static_obj,
                                                   test_ret.static_distance, &t);
        plan_var_t relative_velocity_limit =
            common::Lerp(param_.relative_velocity_limit_table_by_dist_to_static_obj[lower].value,
                         param_.relative_velocity_limit_table_by_dist_to_static_obj[lower + 1].value, t);
        plan_var_t abs_angle_diff =
            common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
        decision.planning_target_v =
            tar_obj.v /* common::com_cos(abs_angle_diff)*/ + relative_velocity_limit; //静态：在障碍物速度0km/h基础上+抬高的相对速度
        decision.planning_target_s = rsk_obj_info.dist_to_tar_obj;
      } break;

      case (driv_map::OBJ_POSITION_LEFT): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 左侧障碍物，忽略";
#endif
      } break;
      case (driv_map::OBJ_POSITION_RIGHT): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 右侧障碍物，忽略";
#endif
      } break;
      case (driv_map::OBJ_POSITION_LEFT_BACK): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "Step 2 : 左后方障碍物，忽略";
#endif
      } break;
      case (driv_map::OBJ_POSITION_BACK): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
       LOG_INFO(5) << "Step 2 : 正后方障碍物，忽略";
#endif
      } break;
      case (driv_map::OBJ_POSITION_RIGHT_BACK): {
        decision.need_process = false;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
       LOG_INFO(5) << "Step 2 : 右后方障碍物，忽略";
#endif
      } break;

      default:
        break;
    }
  }

  // 更精细的筛选是否是可跟车的目标
  bool allow_following = IsFollowingObject(decision, rsk_obj_info.dist_to_tar_obj, rsk_obj_info.test_ret.obj_l_ref); // 处理正前方的静态，动态的车
  if (allow_following) {
    decision.can_following = true;
  } else {
    decision.can_following = false;

    if (rsk_obj_info.obj_position == driv_map::OBJ_POSITION_LEFT_FRONT ||
        rsk_obj_info.obj_position == driv_map::OBJ_POSITION_RIGHT_FRONT) {
      if (test_ret.static_distance < 0.5F) {
        if (settings_.enable_pacc) {
          if (rsk_obj_info.dist_to_tar_obj < 80.0F) { //  退出PCC/CC，进入跟车的纵向距离条件：＜80m
            decision.need_avoiding = true;
          }
        } else {
          decision.need_avoiding = true;
        }
      } else {
        decision.need_avoiding = false;
      }
    }
  }
  

  // 针对选择的目标计算相关判定参数（TS，TTC）
  // 由于坐标不同的原因导致的在 ttc 计算的时候 会将时间上变小 
  // 在计算的时候 再将 该距离 dist_of_localization_to_front 加回来 从而 减小 aeb 的触发频率

  decision.v_for_plan = curr_position_.v;
  decision.time_gap = decision.dist_to_obj / decision.v_for_plan;
  if (decision.time_gap < 0.0F) {
    decision.time_gap = 0.0F;
  }

  decision.ttc = 100.0F;
  plan_var_t relative_v = decision.planning_target_v - curr_position_.v;
  if (relative_v < -0.01F) {
    decision.ttc = common::Max(0.0F, (decision.dist_to_obj)) / (-relative_v);
    if (decision.ttc > 100.0F) {
      decision.ttc = 100.0F;
    }
  }

  // 对满足AEB要求的障碍物打标签
  if ((settings_.enable_aeb) &&
      (decision.ttc < 4.0F) &&
      (decision.obb_distance < 0.01F) &&
      (!is_uncertain) &&
      (decision.perception_obstacle.confidence >= 90) &&
      (decision.perception_obstacle.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED)) {
        decision.need_aeb_warning = true;
  }

  // 获取目标TTC
  plan_var_t ratio = 0;
  Int32_t lower = common::LerpInOrderedTable(param_.aeb_action_ttc_table, curr_position_.v, &ratio);
  plan_var_t tar_ttc =
      common::Lerp(param_.aeb_action_ttc_table[lower].value, param_.aeb_action_ttc_table[lower + 1].value, ratio);

  if ((settings_.enable_aeb) && 
      (decision.ttc < tar_ttc) &&
      (decision.obb_distance < 0.01F) && 
      (!is_uncertain) && 
      (decision.perception_obstacle.confidence >= 90) && 
      (decision.perception_obstacle.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED)) {
        decision.need_aeb_full_brake = true;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    printf(
        "[Classify Target By Obstacle] enable_aeb=%d, ttc=%0.1f, tar_ttc=%0.1f, static_dist=%0.1f"
        ", is_uncertain=%d, confidence=%d\n",
        settings_.enable_aeb, decision.ttc, tar_ttc, decision.obb_distance,
        is_uncertain, decision.perception_obstacle.confidence);
#endif
  }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<< 目标决策 Classify Target By Obstacle (End) ##";
#endif
}


void VelocityPlanningPolynomialFitting::GetObstacleDecisionData(const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info, 
                                                       ObstacleDecision &decision) {
  const driv_map::CollisionTestResult::ObjInfo &obj_obstacle = rsk_obj_info.test_ret;
  const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(obj_obstacle.obj_list_index);

  plan_var_t abs_angle_diff = common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
  plan_var_t velocity = tar_obj.v * common::com_cos(abs_angle_diff);
  plan_var_t distance = rsk_obj_info.dist_to_tar_obj;

  decision.is_risk = true;
  //TODO: Copy Ctr how work?
  decision.perception_obstacle = tar_obj;

  decision.obj_position = rsk_obj_info.obj_position;
  decision.obj_direction = rsk_obj_info.obj_direction;
  decision.is_cutin = rsk_obj_info.cut_in;
  decision.obb_distance = obj_obstacle.static_distance;

  decision.dist_to_obj = distance;
  decision.safe_dist = 0;
  decision.planning_target_v = velocity;
  decision.planning_target_s = distance;
}

//TODO
void VelocityPlanningPolynomialFitting::CalculateVelocityAccordingToObstacle(const ObstacleDecision &decision, ParamOfPolynomialFitting &param, StateOfPolynomial &state, TargetVelocityInfo &result) {

  plan_var_t target_v;
  plan_var_t target_a;
  plan_var_t risk_perception;
  bool release_throttle = false;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "## Calculate Velocity AccordingTo Obstacle (begin) >>";
#endif
  
  PolynomialFittingOutput output;
  // 跟车起步
  bool follow_is_go = VelocityPlanningIsGoState(decision.planning_target_v);

  if (follow_is_go) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "Follow : START to Go";
#endif  
    target_v = param_.go_target_velocity;
    target_a = param_.go_target_acc;

  } else {

    UpdatePolynomialParamAccMaxMinByCurrentVehicleState(false, curr_position_.v, decision.planning_target_s, decision.planning_target_v,
                                              &(param.ax_max), &(param.ax_min), &(param.safe_distance),
                                              &(param.replan_limit_acc), &(param.replan_limit_dec));

    UpdatePolynomialInitState(param, state);



    int calac_method = FSMPolynomialCurve1d(param, state,
                                              curr_position_.v, decision.planning_target_s, decision.planning_target_v);
    // step 1 : 更新目标车速 ，目标加速度
    switch (calac_method) {
    case (FSM_OVER_XMAX_STATIC_OBJ): {
      CalcVelForStaticObstacle(decision.planning_target_s, curr_position_.v, &target_v, &target_a);
    }
    break;

    case (FSM_CUTIN_OBJ):
    case (FSM_CUTIN_OBJ_EMERGENT):
    case (FSM_CUTIN_CROSS):
    case (FSM_CUTIN_NO_ACTION): {
      CalcVelForCutInObstacle(calac_method, param,
                                decision.planning_target_s, decision.planning_target_v,
                                curr_position_.v, &target_v, &target_a);
    }
    break;

    case (FSM_CUTIN_NORMAL): {
      if (settings_.enable_pacc) {
        if ((curr_position_.v < 30.0/3.6) && (decision.planning_target_v < 30.0/3.6)) { // 低速跟车及跟停，防止距离太远
          UpdatePolynomialInputValueFinalStateOnACC(param, curr_position_.v, decision.planning_target_s, decision.planning_target_v, state);
        }
      } else { //TODO：动力模式跟车时距需复测
          UpdatePolynomialInputValueFinalStateOnACC(param, curr_position_.v, decision.planning_target_s, decision.planning_target_v, state);
      }
      
        CalcTarVelocityPolynomial(param, state, output);
        target_v = output.tar_v;
        target_a = output.tar_a;
    }
    break;

    default:
      break;
    }
  result.obstacle_info.fsm_state = calac_method;
  }

if (settings_.enable_pacc) {
    release_throttle = CalculateVelocityRiskPerception(curr_position_.v, decision.planning_target_s, 
                                                            decision.planning_target_v, param.safe_distance, risk_perception);
}

  if (release_throttle) {
    result.release_throttle = true;//释放油门（滑行）：条件2

    result.tar_v = target_v;
    result.tar_a = target_a;
    
  } else {
  	result.tar_v = target_v;
  	result.tar_a = target_a;
  }  
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "<< Calculate Velocity AccordingTo Obstacle (end) ##";
#endif  
}

bool VelocityPlanningPolynomialFitting::CalculateVelocityRiskPerception(const plan_var_t &ego_v, const plan_var_t &obj_distance,
                                                                       plan_var_t obj_v, plan_var_t safe_distance, plan_var_t &risk_perception) {
  plan_var_t s_diff = obj_distance + safe_distance;
  plan_var_t v_diff = obj_v - ego_v;

  plan_var_t rp_ts = s_diff / ego_v;
  plan_var_t rp_ttc = 100.0;
  if (ego_v > obj_v) {
    rp_ttc = -s_diff / v_diff;
  }

  risk_perception = 1 / rp_ts + 4 / rp_ttc;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "risk_perception = " << risk_perception;
#endif

  if (risk_perception < param_.risk_perception) {
    if ((v_diff > -15.0/3.6) && (v_diff < 1.0/3.6) && (rp_ts > 2.5) && (curr_slope_ < 2.0)) { //v1-1/3.6<v0<v1+15/3.6
      return true;
    }
  }
  return false;
}

void VelocityPlanningPolynomialFitting::UpdatePolynomialInputValueFinalStateOnACC(const ParamOfPolynomialFitting &param,
                                                                                  const plan_var_t &ego_v, const plan_var_t &obj_distance,
                                                                                  const plan_var_t &obj_v,
                                                                                  StateOfPolynomial &Polynomial_output) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "# Update Polynomial On ACC (begin) >";
#endif
  const plan_var_t sample_total_time = 7.0F;

  plan_var_t polynomial_initial_state_s = 0.0F;
  plan_var_t polynomial_initial_state_vx;
  plan_var_t polynomial_final_state_vx = obj_v;

  plan_var_t odd_speed_limit  = param_.max_velocity_limit;
  plan_var_t low_speed_boundry = 5.0 / 3.6;
  plan_var_t detla_vel_low_limit = 13 / 3.6;
  plan_var_t detla_vel_upper_limit = 90 / 3.6;
  plan_var_t ts = obj_distance / ego_v;

  if (Polynomial_output.initial_state_vx < 0.01F) {
    polynomial_initial_state_vx = 0;
  } else {
    polynomial_initial_state_vx = Polynomial_output.initial_state_vx;
  }

  plan_var_t vxf_min = common::Max(0.0F, polynomial_initial_state_vx + param.ax_min * param.tf);
  plan_var_t vxf_max = common::Min(odd_speed_limit, polynomial_initial_state_vx + param.ax_max * param.tf);

  plan_var_t xf_min = (pow(vxf_min, 2) - pow(polynomial_initial_state_vx, 2)) / 2 / param.ax_min;
  plan_var_t xf_max = (pow(vxf_max, 2) - pow(polynomial_initial_state_vx, 2)) / 2 / param.ax_max;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "init-vx = " << polynomial_initial_state_vx << "m/s(" << polynomial_initial_state_vx * 3.6 << "km/h)"
              << ", ax_min = " << param.ax_min << "m/s^2" << ", ax_max = " << param.ax_max << "m/s^2" << ", tf = " << param.tf << "s";
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "V Sample range : " << " [ " << vxf_min * 3.6F << " ~ " << vxf_max * 3.6F << " ] km/h"
  << ", S Sample range : " << " [ " << xf_min << " ~ " << xf_max << " ] m";
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "init final_vx = " << polynomial_final_state_vx << "m/s(" << polynomial_final_state_vx * 3.6 << "km/h) , cc_upper_limit_v_ = " << cc_upper_limit_v_
    << "m/s(" << cc_upper_limit_v_ * 3.6 << "km/h), obj_v = " << obj_v << "m/s(" << obj_v * 3.6 << "km/h)";
#endif

  // TODO: CC 2 ACC
  if (obj_v >= low_speed_boundry) {
    if (((polynomial_initial_state_vx - obj_v) >= detla_vel_low_limit) && 
        ((polynomial_initial_state_vx - obj_v) <= detla_vel_upper_limit)) { // 自车速度远高于目标障碍物速度，比如从巡航到跟车场景
        if ((obj_distance > xf_max)) { // 自车距障碍物的距离在采样区间外，缓减速或不减速
          if ((xf_max - 5.0) > obj_v * param.tf) {
            polynomial_final_state_vx = (obj_distance - sample_total_time * ego_v) * 
                    ((ego_v - obj_v) / (sample_total_time * ego_v - (xf_max - 5.0))) + ego_v;
          } else {
            polynomial_final_state_vx = (obj_distance - sample_total_time * ego_v) * ((ego_v - obj_v) / 
                    (sample_total_time * ego_v - xf_max)) + ego_v;
          }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "obstacle far away from S Sample: dist_to_obj = " << obj_distance << " m,  xf_max = " << xf_max << " m "
                      << ", ego_v = " << ego_v << ", obj_v" << obj_v << ", final_vx = " << polynomial_final_state_vx << "m/s(" << polynomial_final_state_vx * 3.6 << "km/h)";
#endif 
        }
    } else if (common::com_abs(polynomial_initial_state_vx - obj_v) < detla_vel_low_limit) {
      if (ts > settings_.time_gap_setting + 0.5F) { // TODO: time_gap+0.5s时距内速度目标车速为前车车速，减少跟车时距附近刹车
        polynomial_final_state_vx = obj_v + 3 / 3.6;
      }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "obstacle v near ego v: obj_distance = " << obj_distance << " m,  xf_max = " << xf_max << " m "
                      << ", int_vx = " << polynomial_initial_state_vx << "m/s, obj_v" << obj_v << "m/s, final_vx = " << polynomial_final_state_vx << "m/s(" << polynomial_final_state_vx * 3.6 << "km/h)";
#endif       
    }
  } else {
    polynomial_final_state_vx = obj_v;
  }
//防止跟车速度超调，多项式终端速度polynomial_final_state_vx不能超过巡航速度上限
  if (polynomial_final_state_vx > cc_upper_limit_v_) {
    polynomial_final_state_vx = cc_upper_limit_v_;
  }
  Polynomial_output.final_state_vx = polynomial_final_state_vx;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "final final_vx = " << polynomial_final_state_vx << "m/s(" << polynomial_final_state_vx * 3.6 << "km/h)";

  LOG_INFO(5) << "< Update Polynomial On ACC (End) #";
#endif
}

int VelocityPlanningPolynomialFitting::FSMPolynomialCurve1d(const ParamOfPolynomialFitting &param, const StateOfPolynomial &state,
                                                              const plan_var_t &ego_v, const plan_var_t &obj_distance, const plan_var_t &obj_v) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "# following decision (begin) >";
#endif
  plan_var_t polynomial_initial_state_s = 0.0F;
  plan_var_t polynomial_initial_state_vx;
  plan_var_t obj_low_speed_boundary  = 5 / 3.6;//改为5/3.6
  plan_var_t distance_time_overtake_no_action = 0.6;
  plan_var_t odd_speed_limit = param_.max_velocity_limit;

  plan_var_t s_diff = obj_distance + param.safe_distance;

  if (s_diff < 0.0) {
    s_diff = 0.0;
  }
  
  int fsm_state = 0;

  if (state.initial_state_vx < 0.01F) {
    polynomial_initial_state_vx = 0;
  } else {
    polynomial_initial_state_vx = state.initial_state_vx;
  }

  plan_var_t vxf_min = common::Max(0.0F, polynomial_initial_state_vx + param.ax_min * param.tf);
  plan_var_t vxf_max = common::Min(odd_speed_limit, polynomial_initial_state_vx + param.ax_max * param.tf);

  plan_var_t xf_min = (pow(vxf_min, 2) - pow(polynomial_initial_state_vx, 2)) / 2 / param.ax_min;
  plan_var_t xf_max = (pow(vxf_max, 2) - pow(polynomial_initial_state_vx, 2)) / 2 / param.ax_max;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "init-vx = " << polynomial_initial_state_vx << "m/s(" << polynomial_initial_state_vx * 3.6 << "km/h)"
              << ", ax_min = " << param.ax_min << "m/s^2" << ", ax_max = " << param.ax_max << "m/s^2" << ", tf = " << param.tf << "s";
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "V Sample range : " << " [ " << vxf_min * 3.6F << " ~ " << vxf_max * 3.6F << " ] km/h"
  << ", S Sample range : " << " [ " << xf_min << " ~ " << xf_max << " ] m";
#endif

  if ((obj_v < obj_low_speed_boundary) && (obj_distance > xf_max)) {
    fsm_state = FSM_OVER_XMAX_STATIC_OBJ;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "type = " << fsm_state << ", obj_v = " << obj_v << ", obj_distance = " << obj_distance << ", s sample max = " << xf_max;
#endif
    return fsm_state;
  }

  if (xf_min > s_diff) { // 障碍物在规划最小距离内
    if (ego_v > obj_v) { // 自车速度高于障碍物速度
      if (obj_v > obj_low_speed_boundary) { // 障碍物速度
        fsm_state = FSM_CUTIN_OBJ;
      } else {
        fsm_state = FSM_CUTIN_OBJ_EMERGENT;
      }
      return fsm_state;
    } else {
      if ((obj_distance > 0.0) && (obj_distance < ego_v * distance_time_overtake_no_action)) {
        fsm_state = FSM_CUTIN_CROSS;
      } else {
        fsm_state = FSM_CUTIN_NO_ACTION;
      }
      return fsm_state;
    }
  }

  fsm_state = FSM_CUTIN_NORMAL;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "type = " << fsm_state << ", xf_min = " << xf_min << ", s_diff = " << s_diff;
  LOG_INFO(5) << ", ego_v = " << ego_v << ", obj_v = " << obj_v << ", obj_distance = " << obj_distance;
  LOG_INFO(5) << "< following decision (end) #";
#endif  

  return fsm_state;
}

//静止障碍物停车距离
void VelocityPlanningPolynomialFitting::CalcVelForStaticObstacle(const plan_var_t &obj_distance, const plan_var_t &ego_v,
                                                                   plan_var_t *tar_v, plan_var_t *tar_a) {
  Float32_t stop_distance = 10.0;
  Float32_t Static_Stop_Distance = obj_distance - stop_distance;
  if (Static_Stop_Distance < 0.0) {
    *tar_v = 0.0;
    *tar_a = -1.0;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Static Obstacle] avoidance static obstacle near : tar_v = " << *tar_v << "m/s(" << *tar_v * 3.6 << "km/h)"
                << ", tar_a = " << *tar_a << " m/s2 , Static_Stop_Distance = " << Static_Stop_Distance;
#endif
  } else {
      *tar_a = -1 * (ego_v * ego_v / 2 / Static_Stop_Distance);
      *tar_v = ego_v + *tar_a * 1.0;
      *tar_a = *tar_a * 0.5;

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Static Obstacle] avoidance static obstacle far : tar_v = " << *tar_v << "m/s(" << *tar_v * 3.6 << "km/h)"
                << ", tar_a = " << *tar_a << " m /s2 , Static_Stop_Distance = " << Static_Stop_Distance;
#endif
  }
}

void VelocityPlanningPolynomialFitting::CalcVelForCutInObstacle(const int &cut_state, const ParamOfPolynomialFitting &param,
                                                                   const plan_var_t &obj_distance, const plan_var_t &obj_v,
                                                                   const plan_var_t &ego_v, plan_var_t *tar_v, plan_var_t *tar_a) {
  plan_var_t s_diff = obj_distance + param.safe_distance;
 
  if (s_diff < 0.0) {
    s_diff = 0.0;
  }

  plan_var_t v_diff = obj_v - ego_v;                                                            
  switch (cut_state) {
  case (FSM_CUTIN_OBJ): {
      // 2023.8.27 在跟定的测试工况没有进入 该状态不知是否正常
      if (obj_distance > 3.0 * obj_v) { // 目标障碍物在跟车时距之外Cut In
        Float32_t dec = -(v_diff * v_diff / 2 / (s_diff - 3.0 * obj_v));
        if (dec > 0) { //修复V3.11换道中出现加速问题 qiuzw
          dec = param.ax_min;
        }
        if (dec < param.ax_min) {
          *tar_a = param.ax_min;
        } else {
          *tar_a = dec;
        } 
      } else if(obj_distance > 2.0 * obj_v) {
        if (v_diff < -10/3.6) {
          *tar_a = param.ax_min;
        } else {
          *tar_a = param.ax_min * 0.5;
        }
      } else {
        *tar_a = param.ax_min;
      }
      *tar_v = ego_v + *tar_a * 0.1;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "[Cut IN] : tar_a " << *tar_a << "m/s^2, tar_v = " << *tar_v << "m/s(" << *tar_v * 3.6 << "km/h)" << ", s_diff = " << s_diff 
                << ", obj_v = " << obj_v << ", obj_s = " << obj_distance << ", ego_v = " << ego_v;
#endif   
    }
    break;
  case (FSM_CUTIN_OBJ_EMERGENT): {
      Float32_t dec_min;
      if (ego_v <= 10 / 3.6) {
        dec_min = -1.0;
      } else if ((10 / 3.6 < ego_v) && (ego_v <= 20 / 3.6)) {
        dec_min = -1.5;
      } else if ((20 / 3.6 < ego_v) && (ego_v <= 30 / 3.6)) {
        dec_min = -1.8;
      } else if ((30 / 3.6 < ego_v) && (ego_v <= 40 / 3.6)) {
        dec_min = -2.0;
      } else if ((40 / 3.6 < ego_v) && (ego_v <= 50 / 3.6)) {
        dec_min = -2.0;
      } else if ((50 / 3.6 < ego_v) && (ego_v <= 60 / 3.6)) {
        dec_min = -2.5;
      } else if ((60 / 3.6 < ego_v) && (ego_v <= 90 / 3.6)) {
        dec_min = -2.5;
      } else {
        dec_min = -2.5;
      }

      *tar_a = dec_min;
      *tar_v = ego_v + dec_min * 1.0;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[CutIn] Stop IN scenario :  tar_a " << *tar_a << "m/s^2 , tar_v =" << *tar_v << "m/s(" << *tar_v * 3.6 << "km/h)";
#endif
    }
    break;
  case (FSM_CUTIN_CROSS): {
      *tar_a = -0.5;//近距离切入，减速不明显
      *tar_v = ego_v - 0.2;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[CutIn] Cross scenario : tar_a = " << *tar_a << "m/s2, tar_v" << *tar_v << "m/s(" << *tar_v * 3.6 << "km/h)";
#endif
    }
    break;
  case (FSM_CUTIN_NO_ACTION): {
      *tar_a = 0.0;
      *tar_v = ego_v;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[CutIn] Do Nothing : tar_a = " << *tar_a << "m/s2, tar_v" << *tar_v << "m/s(" << *tar_v * 3.6 << "km/h)";
#endif
    }
    break;

  default:
    break;
  }
}

//起步状态判断
bool VelocityPlanningPolynomialFitting::VelocityPlanningIsGoState(const float &target_v) {
    if (curr_position_.v < 0.7 && target_v > 2.0) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "VelocityPlanning : Start to Go";
#endif
      return true;
    } else {
      return false;
    }
}

//多项式的跟停/静止obs避障，采用3个周期和obj_v做冗余判断
void VelocityPlanningPolynomialFitting::VelocityPlanningForObstacleStop(const int &final_vel_type, const plan_var_t &obs_v,
                                                                  plan_var_t &final_vel, plan_var_t &final_a) {
  if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == final_vel_type ||
      VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == final_vel_type ||
      VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == final_vel_type) {
    if (final_vel < 0.8 && prev_tar_v_ < 0.8 && obs_v < 0.8) {
      if (stop_count_ > 3.0) { //3个周期
        // 防止过于平滑，导致最终规划速度无法为0；强制 tar_a=-2，tar_v=0 by 20220922
        final_a = -0.4;//最后一脚 V3.04
        final_vel = 0.0;
      }
      ++stop_count_;
    } else {
      stop_count_ = 0.0;
    }
  }
  return;
}

//H1 用于限制最終的速度和加速度
void VelocityPlanningPolynomialFitting::LimitFinalResult(VelocityPlanningResult& finalresult) {
  if (finalresult.tar_v > param_.max_velocity_limit) {
    finalresult.tar_v = param_.max_velocity_limit;
  } else if (finalresult.tar_v < 0.0F) {
    finalresult.tar_v = 0.0F;
  } else {
    // nothing to do
  }

  if (finalresult.tar_a > param_.max_acceleration_limit) {
    finalresult.tar_a = param_.max_acceleration_limit;
  } else if (finalresult.tar_a < param_.max_deceleration_limit) {
    finalresult.tar_a = param_.max_deceleration_limit;
  } else {
    // nothing to do
  }
}

//H2 用于记录本周期的数据作为下一周期的参考
void VelocityPlanningPolynomialFitting::RecordData(const VelocityPlanningResult& finalresult) {
  array_target_velocity_[2] = array_target_velocity_[1];
  array_target_velocity_[1] = array_target_velocity_[0];
  array_target_velocity_[0] = finalresult.tar_v;
  array_target_acceleration_[2] = array_target_acceleration_[1];
  array_target_acceleration_[1] = array_target_acceleration_[0];
  array_target_acceleration_[0] = finalresult.tar_a;

  prev_tar_v_ = finalresult.tar_v;
  prev_tar_a_ = finalresult.tar_a;
  pre_adas_mode_ = adas_mode_;
  pre_vel_plan_type_ = finalresult.tar_type;
  pre_time_gap_setting_ = settings_.time_gap_setting;

}

// 发出事件报告
void VelocityPlanningPolynomialFitting::AddEventReporting(Int32_t event_type) {
  ad_msg::EventReporting event;
  event.msg_head.valid = false;

  switch (event_type) {
    case (EVENT_TYPE_AEB_WARNING):
      event.msg_head.valid = true;
      event.event_id = ad_msg::EVENT_REPORTING_ID_AEB_WARNING;
      event.priority = 2;
      event.lifetime = 2 * 1000;
      break;
    case (EVENT_TYPE_AEB_ACTION):
      event.msg_head.valid = true;
      event.event_id = ad_msg::EVENT_REPORTING_ID_AEB_ACTION;
      event.priority = 2;
      event.lifetime = 2 * 1000;
      break;
    case (EVENT_TYPE_UNCERTAIN_OBSTACLE):
      event.msg_head.valid = true;
      event.event_id = ad_msg::EVENT_REPORTING_ID_UNCERTAIN_OBSTACLE;
      event.priority = 2;
      event.lifetime = 4 * 1000;
      break;
    case (EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL):
      event.msg_head.valid = true;
      event.event_id = ad_msg::EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL;
      event.priority = 2;
      event.lifetime = 4 * 1000;
      break;
    case (EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_RAMP):
      event.msg_head.valid = true;
      event.event_id = ad_msg::EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_RAMP;
      event.priority = 2;
      event.lifetime = 4 * 1000;
      break;
    default:
      break;
  }

  if (event.msg_head.valid) {
    event_reporting_list_.PushBack(event);
  }
}

/*
 * @brief 在指定位置，以指定的航向角生成车辆在此位姿下的有向矩形包围盒
 * @param[in] pos 位置
 * @param[in] heading 航向角
 * @param[out] obb 车辆在对应位姿下的有向矩形包围盒
 * @note 生成的obb用于碰撞分析。
 */
void VelocityPlanningPolynomialFitting::CreateVehOBB(const common::Vec2d &pos, plan_var_t heading, common::OBBox2d *obb) const {
  const plan_var_t half_veh_width = param_.vehicle_width * 0.5f;
  const plan_var_t half_veh_length = param_.vehicle_length * 0.5f;

  obb->set_unit_direction(common::com_cos(heading), common::com_sin(heading));
  obb->set_center(pos + param_.dist_of_localization_to_center * obb->unit_direction_x());
  obb->set_extents(half_veh_length, half_veh_width);
}

//MD到AD 起步计数的过渡，避免进入点刹
void VelocityPlanningPolynomialFitting::VelocityPlanningStartAD() {
  // 起步计数 start_count_
  if ((!pre_adas_mode_) && (adas_mode_)) {
    start_count_ = 0;
  } else if ((pre_adas_mode_) && (adas_mode_)) {
    if (start_count_ <= 10) {
      start_count_++;
    } else {
      // nothing;
    }
  } else {
    start_count_ = 0;
  }
  
  // 速度平滑有效性处理
  if ((!pre_adas_mode_) && (adas_mode_)) {
    valid_target_velocity_array_ = false;
  } else if ((!pre_adas_mode_) && (!adas_mode_)) {
    valid_target_velocity_array_ = false;
  } else {
    valid_target_velocity_array_ = true;
  }

#if PLANNING_DEBUG
  planning_debug_.tar_v_his_init = valid_target_velocity_array_;
#endif

  // 无效的目标速度向量时，用当前速度初始化array_target_velocity_
  if (!valid_target_velocity_array_) {
    array_target_velocity_[0] = curr_position_.v;
    array_target_velocity_[1] = curr_position_.v;
    array_target_velocity_[2] = curr_position_.v;
    array_target_acceleration_[0] = curr_position_.a;
    array_target_acceleration_[1] = curr_position_.a;
    array_target_acceleration_[2] = curr_position_.a;
  }
}

//SmoothVelocity
void VelocityPlanningPolynomialFitting::SmoothVelocity(VelocityPlanningResult vel_planning_result, plan_var_t *smoothed_tar_v,
                                              plan_var_t *smoothed_tar_a) {
  plan_var_t deceleration_jerk;
  if (curr_position_.v > 80.0 / 3.6) {
    deceleration_jerk = -1.0;
  } else {
    deceleration_jerk = -2.0;
  } 
  plan_var_t planning_cycle_time = 0.1;

  if (vel_planning_result.tar_a < array_target_acceleration_[0]) {
    // deceleration
    // 针对前车急停，本车减速动作延时
    *smoothed_tar_a = 0.2F * array_target_acceleration_[1] + 0.3F * array_target_acceleration_[0] + 0.5F * vel_planning_result.tar_a;

    if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == vel_planning_result.tar_type ||
    VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == vel_planning_result.tar_type ||
    VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == vel_planning_result.tar_type) {
      // TODO: 待确认3S时距变成设定时距
      if (((vel_planning_result.tar_obj.obj_v * settings_.time_gap_setting + 3.0) < vel_planning_result.tar_obj.dist_to_obj)) {
        if (((*smoothed_tar_a - array_target_acceleration_[0]) / planning_cycle_time)  < deceleration_jerk) {
          *smoothed_tar_a = array_target_acceleration_[0] +  deceleration_jerk * planning_cycle_time;
        }
      }
    }
  } else {
    // acceleration
    *smoothed_tar_a = 0.1F * array_target_acceleration_[2] + 0.2F * array_target_acceleration_[1] +
                      0.3F * array_target_acceleration_[0] + 0.4F * vel_planning_result.tar_a;
  }

  if (vel_planning_result.tar_v < array_target_velocity_[0]) {
    // deceleration
    // 针对前车急停，本车减速动作延时
    *smoothed_tar_v = 0.2F * array_target_velocity_[1] + 0.3F * array_target_velocity_[0] + 0.5F * vel_planning_result.tar_v;
    if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == vel_planning_result.tar_type ||
    VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == vel_planning_result.tar_type ||
    VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == vel_planning_result.tar_type) {
      // TODO: 待确认3S时距变成设定时距
      if (((vel_planning_result.tar_obj.obj_v * settings_.time_gap_setting + 3.0) < vel_planning_result.tar_obj.dist_to_obj)) {
        if (((*smoothed_tar_v - array_target_velocity_[0]) / planning_cycle_time) < *smoothed_tar_a) {
          *smoothed_tar_v = array_target_velocity_[0] + (*smoothed_tar_a) * planning_cycle_time; 
        }
      }
    }
  } else {
    // acceleration
    *smoothed_tar_v = 0.1F * array_target_velocity_[2] + 0.2F * array_target_velocity_[1] +
                      0.3F * array_target_velocity_[0] + 0.4F * vel_planning_result.tar_v;
    if (array_target_acceleration_[0] < -0.1F) {
      if (vel_planning_result.tar_v > curr_position_.v) {
        *smoothed_tar_v = curr_position_.v;
        if (*smoothed_tar_v < 0.01F) {
          *smoothed_tar_v = 0.0F;
        }
      }
    }
  }
}

//降级
void VelocityPlanningPolynomialFitting::CalcVelocityAccordingToFallback(ActionPlanningResult action_planning_result, 
                                                              plan_var_t curr_v,
                                                              VelocityPlanningResult *vel_planning_result ) {
  Float32_t tar_a;
  Float32_t tar_v;

  Float32_t fallback_limit_a_B_II = -2.0;
  Float32_t fallback_limit_a_C_I =  -2.0;
  Float32_t fallback_limit_a_C_II = -4.0;

  if (REQ_FALLBACK_ACTION_NONE != action_planning_result.fallback_action) { // 需要执行Fallback行为, 重置vel_planning_result数据
    vel_planning_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_FALLBACK;
    vel_planning_result->release_throttle = false;
    vel_planning_result->tar_throttle = 0.0F;
    vel_planning_result->tar_pos.x = 0.0F;
    vel_planning_result->tar_pos.y = 0.0F;
    vel_planning_result->tar_pos.heading = 0.0F;
    vel_planning_result->tar_pos.s = 0.0F;
    vel_planning_result->tar_obj.valid = false;
    vel_planning_result->tar_obj.dist_to_obj = 0.0F;
    vel_planning_result->tar_obj.time_gap = 0.0F;
    vel_planning_result->tar_obj.relative_v = 0.0F;
    vel_planning_result->tar_obj.ttc = 100.0F;
    vel_planning_result->tar_obj.obj_v = 0.0F;
    vel_planning_result->tar_obj.dist_gap_level = -1;
    vel_planning_result->tar_obj.rel_spd_level = -1;
    vel_planning_result->tar_obj.obj_dec_status = 0;
  }

  if (REQ_FALLBACK_ACTION_B_II == action_planning_result.fallback_action) {
    if (first_B_II_) {
      //TODO: -0.5以上减速度,执行器无法执行，控制会采用滑行策略，导致低速(<20km/h)B降级时会出现长时间不停车的问题 : 先使用-1定减速
      /*
      tar_a = (0 - curr_v) / 11.0;
      if (tar_a < (fallback_limit_a_B_II / 2)) {
        tar_a = fallback_limit_a_B_II / 2;
      }
      */
      tar_a = fallback_limit_a_B_II / 2;

      tar_v = curr_v + tar_a;
      dec_B_II_ = tar_a;
      first_B_II_ = false;
    } else {
      tar_v = curr_v + 0.5 * dec_B_II_;
      tar_a = dec_B_II_;
    }
    vel_planning_result->tar_v = tar_v;
    vel_planning_result->tar_a = tar_a;

    first_C_II_ = true;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "B_FallBack_Stata_II is doing !!";
#endif

  } else if (REQ_FALLBACK_ACTION_C_I == action_planning_result.fallback_action) {
    tar_a = fallback_limit_a_C_I / 2.0;
    tar_v = curr_v + 0.2 * tar_a;
    vel_planning_result->tar_v = tar_v;
    vel_planning_result->tar_a = tar_a;
    first_C_II_ = true;
    first_B_II_ = true;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) <<  "C_FallBack_Stata_I is doing !!";
#endif
  } else if (REQ_FALLBACK_ACTION_C_II == action_planning_result.fallback_action) {
    if (first_C_II_) {
      tar_a = (0 - curr_v) / 3.0;

      if (tar_a < (fallback_limit_a_C_II / 2)) {
        tar_a = fallback_limit_a_C_II / 2;
      }
      
      tar_v = curr_v + tar_a;
      dec_C_II_ = tar_a;
      first_C_II_ = false;
    } else {
      tar_v = curr_v + 0.2 * dec_C_II_;
      tar_a = dec_C_II_;
    }
    vel_planning_result->tar_v = tar_v;
    vel_planning_result->tar_a = tar_a;
  #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
    LOG_INFO(5) << "C_FallBack_Stata_II is doing !!";
  #endif
    first_B_II_ = true;
  } else {
    // nothing
  }

#if PLANNING_DEBUG
  if (REQ_FALLBACK_ACTION_NONE == action_planning_result.fallback_action) {
    planning_debug_.fallback_type = REQ_FALLBACK_ACTION_NONE;
    planning_debug_.tar_type_fallback = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    planning_debug_.tar_v_fallback = 0.0F;
    planning_debug_.tar_a_fallback = 0.0F;
  } else {
    planning_debug_.fallback_type = action_planning_result.fallback_action;
    planning_debug_.tar_type_fallback = vel_planning_result->tar_type;
    planning_debug_.tar_v_fallback = vel_planning_result->tar_v;
    planning_debug_.tar_a_fallback = vel_planning_result->tar_a;
  }
#endif

  return;
}

/**
 * @brief  PACC Speed Planning
 * 
 */
void VelocityPlanningPolynomialFitting::PlanVelocityAccordingToPredictiveControl(plan_var_t cruise_speed, TargetVelocityInfo& result) {
  result.Clear();

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_PERFORMANCE_TEST
    phoenix::common::Stopwatch performance_timer_pacc;
#endif
  
  PAccInput input;
  PAccInput input_filter;
  PAccOutput output;

  if (is_pacc_valid_) {
    if (!is_pacc_valid_pre_) { // first or pcc valid again
      pacc_.Clear();
    }
    /* 是否要初始状态？？
    if (IsStateChanged()) { // 上一时刻不是 pcc 要对其清除 状态
      pacc_.Clear();
    }
    */
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    VehicleGlobalSpeed_st temp_global_speed;
    memset(&temp_global_speed, 0x00, sizeof(VehicleGlobalSpeed_st));
    System_Get_TBOX_Planning_Vehicle_Global_Speed(&temp_global_speed);
    uint32 temp_road_type = temp_global_speed.m_RoadLength; // from 系统
    if (1 == temp_road_type) {
      road_type_ = 1; 
    } else {
      road_type_ = 0;
    }
#else
    // TODO ： 未来可能根据2000m 的道路长度 来判断 当前的路况类型
    //         (丘陵、平路、长上坡、起伏) 来进行不同的油门限制
    road_type_ = 1;
#endif

    Float32_t torq_limit = 2650.0;
#if ENABLE_PCC_SLOPE_MODEL_LIMIT_TORQUE
    RoadSlicing();

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    if (slope_nodes_.is_valid){
      int slope_nodes_num = slope_nodes_.nodes.Size();

      std::cout << "RoadSlicing number :" << slope_nodes_num << std::endl;
      for (int i = 0; i < slope_nodes_num; i++) {
        std::cout << "RoadSlicing node[ " << i << "] average_slope = " << slope_nodes_.nodes[i].average_slope <<
        ", slope_type = " << slope_nodes_.nodes[i].slope_type << ", slope_length = " << slope_nodes_.nodes[i].slope_length << 
        ", slope_end_s = " << slope_nodes_.nodes[i].end_s << std::endl;
      }
    }
#endif

    MergeSlopeNodes();

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE    
    if (slope_nodes_.is_valid){
      int slope_nodes_num = slope_nodes_.nodes.Size();

      std::cout << "MergeSlopeNodes number :" << slope_nodes_num << std::endl;
      for (int i = 0; i < slope_nodes_num; i++) {
        std::cout << "MergeSlopeNodes node[ " << i << "] average_slope = " << slope_nodes_.nodes[i].average_slope <<
        ", slope_type = " << slope_nodes_.nodes[i].slope_type << ", slope_length = " << slope_nodes_.nodes[i].slope_length << 
        ", slope_end_s = " << slope_nodes_.nodes[i].end_s << std::endl;
      }
    }
#endif
    if (!CalcuTorqueUpLimitAccordSlope(torq_limit)) {
      torq_limit = 2650.0;
    }

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    std::cout << "CalcuTorqueUpLimitAccordSlope :" << torq_limit << std::endl;
#endif

#endif

    TargetVelocityUpperLimit target_velocity; //v4.06-1道路限速8%
    CalculateCruiseVelocityAccordingToRoadLimit(&target_velocity);
    if (target_velocity.valid) {
      input.road_speed_limit = target_velocity.tar_v;
    } else {
      input.road_speed_limit = cruise_speed;
    }

    input.pcc_map_vaild = true;
    input.planning_debug_ = &planning_debug_;
    input.pacc_map = pacc_map_;
    input.cruise_speed = cruise_speed;
    input.road_type = road_type_;

    input.engine_torque_limit = torq_limit;
    
  /*  
  //fix：针对速度不同源的问题，均统一为EBS车速源；将外围环境车速如仪表车速（A1~A6）取小后，再除以1.02
  #if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
      input.cruise_speed = cruise_speed + 2.0 / 3.6;
  #endif
  */
    input.engine_speed = chassis_status_.engine_speed;
    input.engine_torque = chassis_status_.engine_torque;
    input.gear_num = chassis_status_.gear_number;
    input.brake_torque = 0; // TODO : need or not
    input.ego_speed = chassis_status_.v;
    input.ego_speed_raw = input.ego_speed;
    input.ego_accel = chassis_status_.a;
    input.ego_mass = chassis_status_.gross_weight;
    input.pre_velocity_planning_type = pre_vel_plan_type_;
    float Retarder_threshold = input.cruise_speed * 0.05;
    
#if ENABLE_PACC_BACK2BACK_TEST
    input.pcc_map_vaild = true;
    input.planning_debug_ = &planning_debug_;
    input.pacc_map = pacc_map_;
    input.cruise_speed = Float32_t(85.0 / 3.6);
    input.engine_speed = 985.750F;
    input.engine_torque = 464.240F;
    input.gear_num = 12;
    input.brake_torque = 0; // TODO : need or not
    input.ego_speed = 15.955F;
    input.ego_accel = -0.153F;
    input.ego_mass = 20000.0F;
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[PACC]input : cc_v=" << input.cruise_speed << ", ne=" << input.engine_speed << ", te=" << input.engine_torque << ", gear=" << input.gear_num
              << ", brake pedal=" << input.brake_torque << ", ego v=" << input.ego_speed << ", ego a=" << input.ego_accel << ", mass=" << input.ego_mass;
#endif

#if PLANNING_DEBUG
    if (input.pacc_map.pacc_point_num > 2) {
      planning_debug_.pacc.cur_slope_ufilter = input.pacc_map.points[0].slope;
    }
    planning_debug_.pacc.engine_speed_ufilter = input.engine_speed;
#endif

    input_filter = input;

    PCCinputfilter(input_filter);

    output = pacc_.Plan(input_filter);

#if PLANNING_DEBUG
    planning_debug_.pacc.tar_throttle_ufilter = output.tar_engine_throttle;
#endif

#if ENABLE_PCC_FILTER_OUTPUT_THROTTLE
    PCCoutputfilter(output, input.pre_velocity_planning_type);
#endif

    if (output.pcc_result_vaild && VELOCITY_PLANNING_PCC_TYPE_CRUISE_BRAKE != output.pcc_vel_plan_type) {
      result.tar_type = VELOCITY_PLANNING_TARGET_TYPE_PCC;
      if (PccIsReleaseThottle(input, output)) {
        result.release_throttle = true;//释放油门（滑行）：条件3
      }
      
      // 缓速器开启的条件：道路限速的8% 或 自车车速大于100km/h(仪表)
      if ((input.ego_speed - input.cruise_speed > Retarder_threshold) || (input.ego_speed >= 98.0 / 3.6)){ 
        result.tar_throttle = -1.0F;//开启缓速器
      } else {
        result.tar_throttle = output.tar_engine_throttle;
      }
      result.tar_v = output.tar_v;
      result.tar_a = output.tar_a;

      
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
      printf("[PACC]map src = %d, release thro = %d, tar thro = %0.2f", input.pacc_map.pacc_points_source, result.release_throttle, result.tar_throttle);
#endif
    } else {
      result.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    }
  } else {
    input.pcc_map_vaild = false;
    result.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
  }

  is_pacc_valid_pre_ = is_pacc_valid_ && result.tar_type;

#if PLANNING_DEBUG

    planning_debug_.pacc.enable = settings_.enable_pacc;

    planning_debug_.pacc.valid = is_pacc_valid_;

    planning_debug_.pacc.chassis_input.throttle_sys_status = chassis_status_.throttle_sys_status;
    planning_debug_.pacc.chassis_input.ebs_status = chassis_status_.ebs_status;
    planning_debug_.pacc.chassis_input.acc_pedal_value = chassis_status_.acc_pedal_value;
    planning_debug_.pacc.chassis_input.brake_pedal_value = chassis_status_.brake_pedal_value;
    
    planning_debug_.pacc.chassis_input.cruise_speed = input.cruise_speed;
    planning_debug_.pacc.chassis_input.engine_speed = input_filter.engine_speed;
    planning_debug_.pacc.chassis_input.engine_torque = chassis_status_.engine_torque;

    planning_debug_.pacc.chassis_input.engine_torque_limit = input.engine_torque_limit;

    planning_debug_.pacc.chassis_input.gear_number = chassis_status_.gear_number;
    planning_debug_.pacc.chassis_input.ego_speed = input_filter.ego_speed;
    planning_debug_.pacc.chassis_input.ego_accel = chassis_status_.a;
    planning_debug_.pacc.chassis_input.ego_mass = chassis_status_.gross_weight;
    
    // raw GNSS + RTK
    planning_debug_.pacc.map_input.gnss_status = driving_map_->GetGnss().gnss_status;
    planning_debug_.pacc.map_input.longitude = driving_map_->GetGnss().longitude;
    planning_debug_.pacc.map_input.latitude = driving_map_->GetGnss().latitude;
    planning_debug_.pacc.map_input.altitude = driving_map_->GetGnss().altitude;
    planning_debug_.pacc.map_input.yaw = driving_map_->GetGnss().heading_gnss;

    // localization from MPU
    planning_debug_.pacc.map_input.odom_status = driving_map_->GetGnss().odom_status;
    planning_debug_.pacc.map_input.x_odom = driving_map_->GetGnss().x_odom;
    planning_debug_.pacc.map_input.y_odom = driving_map_->GetGnss().y_odom;
    planning_debug_.pacc.map_input.z_odom = driving_map_->GetGnss().z_odom;
    planning_debug_.pacc.map_input.heading_odom = driving_map_->GetGnss().heading_odom;
    planning_debug_.pacc.map_input.pitch = driving_map_->GetGnss().pitch;
    planning_debug_.pacc.map_input.roll = driving_map_->GetGnss().roll;

    planning_debug_.pacc.road_type = road_type_;
    planning_debug_.pacc.map_input.valid = input.pcc_map_vaild;
    planning_debug_.pacc.map_input.pacc_points_source = input.pacc_map.pacc_points_source;
    planning_debug_.pacc.map_input.pacc_point_s_step = input.pacc_map.pacc_point_s_step;
    planning_debug_.pacc.map_input.pacc_point_num = input.pacc_map.pacc_point_num;
    
    planning_debug_.pacc.map_input.pacc_points.Clear();
    if (input.pacc_map.pacc_point_num > 2) {
      planning_debug_.pacc.map_input.cur_slope = input.pacc_map.points[0].slope * 100;
      planning_debug_.pacc.map_input.preview_slope = input.pacc_map.points[input.pacc_map.pacc_point_num - 1].slope * 100;
    }

    PACCPointDebug point;
    for (int i = 0; i < input.pacc_map.pacc_point_num; ++i) {
      point.s = input.pacc_map.points[i].s;
      point.slope = input.pacc_map.points[i].slope * 100;
      point.curvature = input.pacc_map.points[i].curvature;
      point.speed_limit = input.pacc_map.points[i].speed_limit;
      
      planning_debug_.pacc.map_input.pacc_points.PushBack(point);
    }

    planning_debug_.pacc.output.tar_v = output.tar_v;
    planning_debug_.pacc.output.target_unlimit_vel = output.target_unlimit_vel;
    planning_debug_.pacc.output.tar_a = output.tar_a;
    planning_debug_.pacc.output.tar_gear = output.tar_gear;
    planning_debug_.pacc.output.tar_engine_torque = output.tar_engine_torque;
    planning_debug_.pacc.output.tar_engine_ne = output.tar_engine_ne;
    planning_debug_.pacc.output.tar_drive_force = output.tar_drive_force;
    planning_debug_.pacc.output.cur_resistance = output.cur_resistance;
    planning_debug_.pacc.output.cur_Ff = output.cur_Ff;
    planning_debug_.pacc.output.cur_Fw = output.cur_Fw;
    planning_debug_.pacc.output.cur_Fi = output.cur_Fi;
    planning_debug_.pacc.output.tar_throttle = output.tar_engine_throttle;
    planning_debug_.pacc.output.dichotomy_solution_state = output.dichotomy_solution_state;
    planning_debug_.pacc.output.mpc_solution_state = output.mpc_solution_state;

    planning_debug_.pacc.output.pcc_vel_plan_type = output.pcc_vel_plan_type;

    planning_debug_.pacc.output.pcc_result_vaild = output.pcc_result_vaild;
    
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
  LOG_INFO(5) << "[PACC] output : v = " << output.tar_v << ", a = " << output.tar_a << ", Te = " << output.tar_engine_torque
              << ", Ne = " << output.tar_engine_ne << ", Throttle = " << output.tar_engine_throttle << ", gear = " << output.tar_gear;
#endif

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_PERFORMANCE_TEST
  LOG_INFO(5) << "PACC Planning spend " << performance_timer_pacc.Elapsed() << "ms.";
#endif

}

bool VelocityPlanningPolynomialFitting::CheckPACCIfValid() {
  
  // check if enabled
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  bool is_valid = true;
#else
  bool is_valid = result_of_action_planning_.enable_pcc;
#endif

  // check if gnss ready

  // 确保当前档位是D档: [1, 12]
  if (chassis_status_.gear_number >= 1 && chassis_status_.gear_number <= 12) {
    is_valid = is_valid && true;
  } else {
    is_valid = is_valid && false;
  }
  
  // TODO: PACC速度范围
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  if (adas_mode_) {
    is_valid = is_valid && true;
  } else {
    is_valid = is_valid && false;
  }

  if (pre_vel_plan_type_ != 15) { // 上一状态非 PCC 模式
    // PCC 模式进入速度约束
    if (settings_.tar_v >= 40 / 3.6F && chassis_status_.v >= 35.0 / 3.6F) {
      is_valid = is_valid && true;
    } else {
      is_valid = is_valid && false;
    }
  } else { // 上一状态为 PCC 模式
    // PCC 模式退出速度约束
    if (chassis_status_.v >= 30.0 / 3.6 && settings_.tar_v >= 40 / 3.6F) {
      is_valid = is_valid && true;
    } else {
      is_valid = is_valid && false;
    }
  }
#endif

  // check map data ready
  is_valid = is_valid && ConstructPACCMap();

  return is_valid;
}

bool VelocityPlanningPolynomialFitting::IsStateChanged() {
  if (pre_vel_plan_type_ != VELOCITY_PLANNING_TARGET_TYPE_PCC) {
    return true;
  } else {
    return false;
  }

}


bool VelocityPlanningPolynomialFitting::PccIsReleaseThottle(const PAccInput &input, const PAccOutput &output) {
  
  Float32_t cur_engine_speed = input.engine_speed;
  Float32_t pcc_target_torque = output.tar_engine_torque;

  if (pcc_target_torque <= 0) {
    return true;
  } else {
    return false;
  }
}

bool VelocityPlanningPolynomialFitting::ConstructPACCMap() {
  bool get_map_from_ADASIS_v2 = false;
  bool get_map_from_HDMap = false;
  Float32_t allow_map_length_time_gap = 2.0;
  pacc_map_.Clear();

  // try ADASIS v2 Map firstly: get map data from ADASIS v2 Horizon Reconstructor
  if (adasisv2_horizon_ && adasisv2_horizon_->is_ready) {
    Int32_t slope_sample_num = adasisv2_horizon_->slope_list.Size();
    if (slope_sample_num <= 2) {
      get_map_from_ADASIS_v2 = false;
    } else {
      Float32_t adasisv2_cur_slope = adasisv2_horizon_->slope_list[0].value;

      Float32_t mpu_cur_slope = 0.0F;
      if (!result_of_trajectory_planning_.target_trajectory_slope.Empty()) {
        mpu_cur_slope = result_of_trajectory_planning_.target_trajectory_slope.Front().slope;
      } else {
        mpu_cur_slope = 0.0F;
      }

      if (adasisv2_cur_slope * mpu_cur_slope < -1.0F) { // TBOX与MPU坡度数据相反，通过实际路测，此时更相信MPU数据
        get_map_from_ADASIS_v2 = false;
      } else {
        // TODO: 按需采样, slope, curvature, speed_limit
        // 如何采样？ 按照同一个s，还是不同的s，如何确定s间距

        for (Int32_t i = 0; i < slope_sample_num; ++i) {
          const adasisv2::Profile& sample = adasisv2_horizon_->slope_list[i];
  #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[TBOX Slope] s = " << sample.offset_from_ego << ", slope = " << sample.value * 0.01;
  #endif
          PACCPoint point;
          point.s = sample.offset_from_ego;
  #if ENABLE_PACC_BACK2BACK_TEST
          point.slope = sample.value * 0.0;
  #else      
          point.slope = sample.value * 0.01;
  #endif
          pacc_map_.points.PushBack(point);
        }
        pacc_map_.pacc_points_source = 1;
        pacc_map_.pacc_point_s_step = adasisv2_horizon_->slope_list[1].offset_from_ego - adasisv2_horizon_->slope_list[0].offset_from_ego;
        pacc_map_.pacc_point_num = slope_sample_num;
        
        /*通过地图过来的长度 与 车速乘以时距 进行比较 来限制是否进入 pcc */
        Float32_t final_pacc_adasis_map_s = pacc_map_.points.Back().s;

        if (final_pacc_adasis_map_s > (allow_map_length_time_gap * curr_position_.v)) {
  #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[TBOX Slope] end_s = " << final_pacc_adasis_map_s;
  #endif
          get_map_from_ADASIS_v2 = true;
        } else {
  #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
          LOG_INFO(5) << "[TBOX Slope]too small s = " << final_pacc_adasis_map_s;
  #endif
          pacc_map_.Clear();
          get_map_from_ADASIS_v2 = false;
        }        
      }
    }
  } else {
    get_map_from_ADASIS_v2 = false;
  }

  // try  MPU HD MAP
  if (!get_map_from_ADASIS_v2) {
    // slope, curvature, speed limit point to the same s or not?

    Int32_t slope_sample_num = result_of_trajectory_planning_.target_trajectory_slope.Size();
    if (slope_sample_num <= 2) {
      get_map_from_HDMap = false;
      return false;
    }
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[MPU Slope] slope length = " << slope_sample_num * 5 << "m, number = " << slope_sample_num;
#endif

    Int32_t driving_map_type = driving_map_->GetDrivingMapType();
    if (driv_map::DRIVING_MAP_TYPE_HD_MAP != driving_map_type && driv_map::DRIVING_MAP_TYPE_MIXED_HD_MAP != driving_map_type) {
      // get_map_from_HDMap = false;
      // return false;
    }

    for (Int32_t i = 0; i < slope_sample_num; i += 2) {  // MPU以5m作为间距，为统一输入，将间距改为10m
      const planning::TrajectoryPlanningResult::SlopeSamplePoint& sample =
          result_of_trajectory_planning_.target_trajectory_slope[i];
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[MPU Slope] s = " << sample.s << ", slope = " << sample.slope * 0.01;
#endif
      PACCPoint point;
      point.s = sample.s;
#if ENABLE_PACC_BACK2BACK_TEST
      point.slope = sample.slope * 0.0;
#else
      point.slope = sample.slope * 0.01;
#endif
      // TODO : curvature, speed_limit
      pacc_map_.points.PushBack(point);
    }
#if ENABLE_PACC_LONG_MAP
      // 使用0补齐缺失的地图数据，使得MPC有解
    if (slope_sample_num < 201) { // PCC地图默认最少2000m，自车坡度占据索引0
      Int32_t fill_start_index = 0;
      if (0 != slope_sample_num % 2) { // 因当前位置坡度占一个点，MPU地图长度是10m倍数
        fill_start_index = slope_sample_num / 2 + 1;
      } else {
        fill_start_index = slope_sample_num / 2;
      }

      for (Int32_t i = fill_start_index; i < 201; ++i) {
        PACCPoint point;
        point.s = i * 10;
        point.slope = 0.0F;
        pacc_map_.points.PushBack(point);
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
        LOG_INFO(5) << "[MPU Slope Fill] s = " << point.s << ", slope = " << point.slope;
#endif
      }
    }
#endif

    pacc_map_.pacc_points_source = 2;
    pacc_map_.pacc_point_s_step = 10;
    pacc_map_.pacc_point_num = pacc_map_.points.Size();

    Float32_t final_pacc_mpu_map_s = pacc_map_.points.Back().s;

    if (final_pacc_mpu_map_s > (allow_map_length_time_gap * curr_position_.v)) {
      get_map_from_HDMap = true;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[MPU Slope] end_s = " << final_pacc_mpu_map_s;
#endif
    } else {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_TRACE
      LOG_INFO(5) << "[MPU Slope]too small s = " << final_pacc_mpu_map_s;
#endif
      pacc_map_.Clear();
      get_map_from_HDMap = false;
    }
  }

  return get_map_from_ADASIS_v2 | get_map_from_HDMap;
}

#if ENABLE_PCC_SLOPE_MODEL_LIMIT_TORQUE
void VelocityPlanningPolynomialFitting::RoadSlicing() {
  slope_nodes_.Clear();
  
  int slope_value_type = SLOPE_VALUE_INVAILD;
  int pre_slope_value_type = SLOPE_VALUE_INVAILD;
  
  if (1 == pacc_map_.pacc_points_source) {
    SlopeNode temp_slope_node;
    temp_slope_node.start_s = 0.0;
	  Float32_t max_slope = pacc_map_.points[0].slope;
    Float32_t cur_slope = pacc_map_.points[0].slope;
    slope_value_type = DeterSlopeValues(cur_slope);
	  pre_slope_value_type = DeterSlopeValues(cur_slope);
	  Float32_t temp_sum_slope = cur_slope;
	  int star_id = 0;
	  // 从 1 开始
    for (int i = 1; i < pacc_map_.pacc_point_num; i++) {
      cur_slope = pacc_map_.points[i].slope;
		  slope_value_type = DeterSlopeValues(cur_slope);
		  if (slope_value_type == pre_slope_value_type) {

		    temp_sum_slope += cur_slope; 
		  	pre_slope_value_type = slope_value_type;

        if (max_slope < pacc_map_.points[i].slope) {
          max_slope = pacc_map_.points[i].slope;
        }
        // 如果只有一段时候，没有考虑 或者 最后一段没有push_back
        if (i == pacc_map_.pacc_point_num - 1) {
          temp_slope_node.end_s = pacc_map_.points[i].s;
          temp_slope_node.average_slope = temp_sum_slope / (i - star_id);
          temp_slope_node.max_slope = max_slope;
          temp_slope_node.slope_length = temp_slope_node.end_s - temp_slope_node.start_s;
          temp_slope_node.slope_type = pre_slope_value_type;
          slope_nodes_.nodes.PushBack(temp_slope_node);
        }  
        continue;
		  } else {
        
		    temp_slope_node.end_s = pacc_map_.points[i].s;
        temp_slope_node.average_slope = temp_sum_slope / (i - star_id);
        temp_slope_node.max_slope = max_slope;
        temp_slope_node.slope_length = temp_slope_node.end_s - temp_slope_node.start_s;
        temp_slope_node.slope_type = pre_slope_value_type;
        slope_nodes_.nodes.PushBack(temp_slope_node);
        // 状态更新
        pre_slope_value_type = slope_value_type;

        temp_slope_node.start_s = pacc_map_.points[i].s;
        max_slope = pacc_map_.points[i].slope;
        temp_sum_slope = pacc_map_.points[i].slope;
        star_id = i;
		}
   }
   Int32_t slope_nodes_num = slope_nodes_.nodes.Size();
   if (slope_nodes_num > 0) {
     slope_nodes_.is_valid = true;
   }
  }
}

void VelocityPlanningPolynomialFitting::MergeSlopeNodes() {

  if (!slope_nodes_.is_valid) {
    std::cout << "[MergeSlopeNodes]slope_nodes is invalid!!" << std::endl;
    return;
  }
  
  float slope_length_limit = 100;
	common::StaticVector<int, 250> slope_id;
	bool update = true;
  float summer_lenght = 0;
	Int32_t slope_nodes_num = slope_nodes_.nodes.Size();
	static const Uint16_t SLOPE_NODE_NUM = 300;
  common::StaticVector<SlopeNode, SLOPE_NODE_NUM> temp_slope_nodes = slope_nodes_.nodes;
  // 获取要合并的范围
	for (int i = 0; i < slope_nodes_num; i++) {
	  if (i == 0 || update) {
	    summer_lenght = slope_nodes_.nodes[i].slope_length;
		  slope_id.PushBack(i); // 存放每一段的开始的id号
	  } else {
	    summer_lenght = summer_lenght + slope_nodes_.nodes[i].slope_length;
	  }

	  if (summer_lenght < slope_length_limit) {
	    update = false;
	  } else {
	    update = true;
	  }
	}

	Int32_t slope_id_num = slope_id.Size();
	SlopeNode temp_merge_slope_node;
	if (common::com_abs(slope_id_num - slope_nodes_num) <= 2) {
	  // 如果个数相差不多进行合并
	} else {
    slope_nodes_.Clear();
    
	  for(int i = 0; i < slope_id_num - 1; ++i) {
      temp_merge_slope_node.start_s = temp_slope_nodes[slope_id[i]].start_s;
      temp_merge_slope_node.end_s = temp_slope_nodes[slope_id[i + 1] - 1].end_s;
      temp_merge_slope_node.slope_length = temp_merge_slope_node.end_s - temp_merge_slope_node.start_s;
      temp_merge_slope_node.max_slope = temp_slope_nodes[slope_id[i]].max_slope;
      int meger_segments_num = slope_id[i + 1] - slope_id[i];
      float temp_sum_slope = 0;

		  for(int j = slope_id[i]; j < slope_id[i + 1]; j++){
		    temp_sum_slope = temp_sum_slope + temp_slope_nodes[j].average_slope;
				if(temp_merge_slope_node.max_slope < temp_slope_nodes[j].max_slope) {
				  temp_merge_slope_node.max_slope = temp_slope_nodes[j].max_slope;
				}
		  }
		  temp_merge_slope_node.average_slope = temp_sum_slope / meger_segments_num;
		  temp_merge_slope_node.slope_type = DeterSlopeValues(temp_merge_slope_node.average_slope);
		  slope_nodes_.nodes.PushBack(temp_merge_slope_node);
	  }

    // 最后一段单独处理！
    temp_merge_slope_node.start_s = temp_slope_nodes[slope_id[slope_id_num - 1]].start_s;
    temp_merge_slope_node.end_s = temp_slope_nodes[slope_nodes_num - 1].end_s;
    temp_merge_slope_node.slope_length = temp_merge_slope_node.end_s - temp_merge_slope_node.start_s;
    temp_merge_slope_node.max_slope = temp_slope_nodes[slope_id[slope_id_num - 1]].max_slope;
    
    if (slope_id[slope_id_num - 1] == (slope_nodes_num - 1)) {
      temp_merge_slope_node.average_slope = temp_slope_nodes[slope_nodes_num - 1].average_slope;
    } else {
      float temp_sum_slope = 0;
      for (int i = slope_id[slope_id_num - 1]; i < slope_nodes_num; i++) {
        
        temp_sum_slope = temp_sum_slope + temp_slope_nodes[i].average_slope;
				if(temp_merge_slope_node.max_slope < temp_slope_nodes[i].max_slope) {
				  temp_merge_slope_node.max_slope = temp_slope_nodes[i].max_slope;
				}

      }
      temp_merge_slope_node.average_slope = temp_sum_slope / slope_nodes_num - slope_id[slope_id_num - 1];
    }
    temp_merge_slope_node.slope_type = DeterSlopeValues(temp_merge_slope_node.average_slope);
    slope_nodes_.nodes.PushBack(temp_merge_slope_node);
    
    Int32_t merge_slope_nodes_num = slope_nodes_.nodes.Size();
    if (merge_slope_nodes_num > 0) {
      slope_nodes_.is_valid = true;
    }
	}
}

int VelocityPlanningPolynomialFitting::DeterSlopeValues(Float32_t cur_slope) {
  int temp_slope_type = SLOPE_VALUE_INVAILD;
  if(cur_slope >= 0 && cur_slope < 0.2f / 100.0f){
    temp_slope_type = SLOPE_TYPE_FLAT_SLOPE;
	} else if(cur_slope >= 0.2f / 100.0f && cur_slope < 1.0f / 100.0f) {
	  temp_slope_type = SLOPE_VALUE_ONE;
	} else if (cur_slope >= 1.0f / 100.0f && cur_slope < 2.0f / 100.0f){
	  temp_slope_type = SLOPE_VALUE_TWO;
	} else if (cur_slope >= 2.0f / 100.0f && cur_slope < 3.0f / 100.0f){
	  temp_slope_type = SLOPE_VALUE_THREE;
	} else if (cur_slope >= 3.0f / 100.0f && cur_slope < 4.0f / 100.0f){
	  temp_slope_type = SLOPE_VALUE_FOUR;
	} else if (cur_slope >= 4.0f / 100.0f && cur_slope < 5.0f / 100.0f){
	  temp_slope_type = SLOPE_VALUE_FIVE;
	} else if (cur_slope >= 5.0f / 100.0f && cur_slope < 6.0f / 100.0f){
	  temp_slope_type = SLOPE_VALUE_SIX;
	} else if (cur_slope < 0.0f){
	  temp_slope_type = SLOPE_TYPE_DOWN_SLOPE;
	} else {
	  temp_slope_type = SLOPE_VALUE_INVAILD;
	}
  return temp_slope_type;
}

bool VelocityPlanningPolynomialFitting::CalcuTorqueUpLimitAccordSlope(Float32_t &torq_limit) {
  if (!(slope_nodes_.is_valid)) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    std::cout << "[CalcuTorqueUpLimitAccordSlope]slope_nodes is invalid!!" << std::endl;
#endif
    return false;
  }
  static uint8_t torque_limit_flag_count = 0u;
  SlopeSmoothInfo slope_smooth_info;
  CalcuSlopeInfo(slope_smooth_info);

#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
  std::cout << "[CalcuTorqueUpLimitAccordSlope] slope_smooth = " << slope_smooth_info.slope_smooth << " slope_length = " << slope_smooth_info.slope_length << " distance_to_slope = " << slope_smooth_info.distance_to_slope << std::endl;
  std::cout << "[CalcuTorqueUpLimitAccordSlope] slope_start_id = " << static_cast<int>(slope_smooth_info.slope_start_id) << " slope_end_id = " << static_cast<int>(slope_smooth_info.slope_end_id) << " uphill_type = "<< static_cast<int>(slope_smooth_info.uphill_type) <<std::endl;
#endif

  plan_var_t Ff = 0.0f; //滚阻
  plan_var_t Fw = 0.0f; //风阻
  plan_var_t Fi = 0.0f; //坡阻
  plan_var_t Ft = 0.0f; //驱动力

  // 取值有效性判断
  if ((chassis_status_.gross_weight <= 0.1f) || (chassis_status_.v  <= 0.0f) || // (chassis_ctl_cmd_.acc_value <= 0.0f) 此判断条件需考虑
	  (chassis_status_.engine_speed <= 0.1f) || (vehicle_dynamic_param_.g <= 0.1f)) {
	  return false;
  }
  
  // 计算的车辆总阻力
  Ff = chassis_status_.gross_weight * vehicle_dynamic_param.g * vehicle_dynamic_param.f;
  Fw = vehicle_dynamic_param.Cd * vehicle_dynamic_param.Ad * vehicle_dynamic_param.air_density * chassis_status_.v * chassis_status_.v / 2.0f;
  Fi = chassis_status_.gross_weight * vehicle_dynamic_param.g * slope_smooth_info.slope_smooth / 100.0f;
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
  std::cout << "[CalcuTorqueUpLimitAccordSlope] Ff = " << Ff << " Fw = " << Fw << " Fi = " << Fi << std::endl;
#endif
  // 查表获取减速度
  auto slope_dec = LookupDecTable(slope_smooth_info.slope_smooth, slope_smooth_info.slope_length);

#if PLANNING_DEBUG
    planning_debug_.pacc.slope_dec = slope_dec;
    planning_debug_.pacc.slope_length = slope_smooth_info.slope_length;
    planning_debug_.pacc.distance_to_slope = slope_smooth_info.distance_to_slope;
#endif

  if (slope_dec < 0) {
    // 暂时不考虑平路下坡的的扭矩限制，同时避免算出的torq_limit超过2650上限
    return false;
  }
  
  #if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    std::cout << "[CalcuTorqueUpLimitAccordSlope] slope_dec = " << slope_dec << " m/s^2 " << std::endl;
  #endif
  // TODO：应根据distance_to_slope，chassis_status_.v, chassis_status_.a 对车速进行优化，使到坡底的速度不低于经济车速,如85km/h。
  // 考虑是否能使PCC模型参数k1,k2为动态调节，以distance_to_slope为参考，实现合适的车速跟踪强度和惩罚力度：上坡>平路>下坡
  // 平路、下坡的扭矩、油门限制暂不考虑。
  Float32_t eco_v = 85.0f / 3.6f;
  if (slope_smooth_info.distance_to_slope <= 0.01f){
    Ft = (Ff + Fw + Fi) - (slope_dec * chassis_status_.gross_weight); // 根据减速度的符号决定驱动力的符号
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
  std::cout << "[CalcuTorqueUpLimitAccordSlope] Ft = " << Ft << std::endl;
#endif
    torq_limit = (Ft * vehicle_dynamic_param.Rw) / (vehicle_dynamic_param.Ig[(int)chassis_status_.gear_number] * vehicle_dynamic_param.If * vehicle_dynamic_param.eta);
    torq_limit = torq_limit * 1.05;// 根据坡道的扭矩限制，建议在限制后提升5%作为后备
  } else if (static_cast<uint8_t>(UphillType::UPHILL_TYPE_THREE) == static_cast<uint8_t>(slope_smooth_info.uphill_type)){ // TODO: 平路扭矩限制,应该根据到坡底的距离控制好到坡底的速度
/*  
    if (chassis_status_.v < eco_v) {
      auto flat_eco_a = (eco_v * eco_v - chassis_status_.v  * chassis_status_.v) / (2 * distance_to_slope);
      if (chassis_status_.a < flat_eco_a) {
        auto tar_a = flat_eco_a;
      } else {
        // do nothing
      }
    } else {
      // do nothing
    } 
*/  
  } else { // TODO: 下坡扭矩限制,应根据到坡底的距离控制好到坡底的速度
/*
    // 应根据distance_to_slope，chassis_status_.v, chassis_status_.a 判断下坡策略,
    // 若处于下坡滑行（也可以不考虑）根据多帧速度对比，判断下否是否在减速，如果是减速则需要看到坡底速度是否会降的太低，来判断是否为非必要滑行,
    // 若下坡非减速，但是在坡底无法加速到目标速度，也应提高加速度处理。
    static const uint8_t VEL_NUM_MAX = 5;
    common::StaticVector<Float32_t, VEL_NUM_MAX> sum_cur_vel;
    if (sum_cur_vel.Size() < VEL_NUM_MAX) {
      sum_cur_vel.PushBack(chassis_status_.v);
    } else {
      for (size_t i = 0; i < (VEL_NUM_MAX - 1); i++) {
        sum_cur_vel[i] = sum_cur_vel[i+1];
      }
      sum_cur_vel[VEL_NUM_MAX - 1] = chassis_status_.v;

      uint8_t vel_count = 0;
      for (size_t j = 0; j < VEL_NUM_MAX; j++) {
        if (sum_cur_vel[j] < sum_cur_vel[j + 1]) {
          vel_count = j + 1;
          continue;
        } else {
          break;
        }
      }
      if (vel_count == VEL_NUM_MAX) { // 连续5帧速度递减
        if (chassis_status_.v < eco_v) { // 此处如果有滑行限制开关，则直接取消滑行即可
          auto downhill_eco_a = (eco_v * eco_v - chassis_status_.v  * chassis_status_.v) / (2 * distance_to_slope);
          if (chassis_status_.a < downhill_eco_a) {
            auto tar_a = downhill_eco_a;
          } else {
            // do nothing 
          }
        } else {
          // do nothing 
        } 
      } else {
        // do nothing 
      }
    }
*/  
  } 
  /* torq_limit = f(n, throtle)*/
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
  std::cout << "[CalcuTorqueUpLimitAccordSlope] torq_limit = " << torq_limit << std::endl;
#endif 
/*
  for (uint8_t i = 0; i < 19; i++){
    torque_limit_arry[i + 1] = torque_limit_arry[i];
  }
  torque_limit_arry[0] = torq_limit;

  slope_distance_arry[0] = slope_distance_arry[1];
  slope_distance_arry[1] = slope_smooth_info.distance_to_slope;
  if ((slope_distance_arry[0] < 0.1f) && (slope_distance_arry[1]) > 0.1f) {
    torque_limit_flag_ = true;
  }

  if (torque_limit_flag_count > 19){
    torque_limit_flag_ = false;
  }

  if (torque_limit_flag_) {
    torque_limit_flag_count++;
    torq_limit = torque_limit_arry[torque_limit_flag_count];
  } else {
    torque_limit_flag_count = 0;
  }
*/ 

  if (torq_limit > 2650.0f){
    torq_limit = 2650.0f;
  }
  
  if (slope_smooth_info.distance_to_slope > 200.0f) { // 距离坡道足够远（目前定为200m）的时候不限制扭矩。
    Float32_t torq_limit_Max = 2650.0f;
    torq_limit = torq_limit_Max;
    return false;
  } else {
    return true;
  }
}

bool VelocityPlanningPolynomialFitting::CalcuSlopeInfo(SlopeSmoothInfo &slope_smooth_info) {
  if ((!(slope_nodes_.is_valid)) || ((slope_nodes_.nodes.Size()) == 0)) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    std::cout << "[CalcuSlopeInfo] slope_nodes_ error !!! " << std::endl;
#endif
    return false;
  }

  slope_smooth_info.Clear();
  //只获取路段上第一个坡道的信息
  for (uint8_t i = 0; i < slope_nodes_.nodes.Size(); i++) {  
    switch (slope_nodes_.nodes[i].slope_type) {
      case SLOPE_VALUE_INVAILD:
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
        std::cout << " [CalcuSlopeInfo] Slope Merge Error !!! " << std::endl;
#endif
        return false;
        break;
      case SLOPE_TYPE_FLAT_SLOPE:
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
        std::cout << " [CalcuSlopeInfo] Slope Merge Is FLAT Road " << std::endl;
#endif
        slope_smooth_info.slope_smooth = 0.0f;
#if PLANNING_DEBUG
    planning_debug_.pacc.slope_smooth = slope_smooth_info.slope_smooth;
#endif
        break; 
      case SLOPE_TYPE_DOWN_SLOPE:
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
        std::cout << " [CalcuSlopeInfo] Slope Merge Is Downhill Road " << std::endl;
#endif
        slope_smooth_info.slope_smooth = -1.0f;
#if PLANNING_DEBUG
    planning_debug_.pacc.slope_smooth = slope_smooth_info.slope_smooth;
#endif
        break;   
      default:
        slope_smooth_info.slope_start_id = i;
        FindOutFirstSlope(slope_smooth_info);
#if PLANNING_DEBUG
    planning_debug_.pacc.slope_smooth = slope_smooth_info.slope_smooth;
#endif
        return true;
        break;
    }  
  }
  slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_INVAILD;
  return false;
}

bool VelocityPlanningPolynomialFitting::FindOutFirstSlope(SlopeSmoothInfo &slope_smooth_info) {
  if ((!(slope_nodes_.is_valid)) || ((slope_nodes_.nodes.Size()) == 0)) {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    std::cout << "[CalcuSlopeInfo] slope_nodes_ error !!! " << std::endl;
#endif
    return false;
  }

  slope_smooth_info.slope_smooth = 0.0f; // 重新初始化 slope_smooth 的值，避免被平路或下坡的赋值影响

  for (uint8_t j = slope_smooth_info.slope_start_id; j < slope_nodes_.nodes.Size(); j++) {
    if ((slope_nodes_.nodes[j].slope_type != SLOPE_TYPE_FLAT_SLOPE) && (slope_nodes_.nodes[j].slope_type != SLOPE_TYPE_DOWN_SLOPE))
    {
      slope_smooth_info.slope_length += slope_nodes_.nodes[j].slope_length;
      slope_smooth_info.slope_end_id = j;
    } else {
      break;
    }
  }
  if ((slope_smooth_info.slope_start_id == slope_smooth_info.slope_end_id) && (slope_smooth_info.slope_start_id == 0)) { // 当前坡道由单个坡道切片组成且切片为第一段
    slope_smooth_info.slope_smooth = slope_nodes_.nodes[slope_smooth_info.slope_start_id].average_slope;
    slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_ONE;
  } else if ((slope_smooth_info.slope_start_id != slope_smooth_info.slope_end_id) && (slope_smooth_info.slope_start_id == 0)) { // 当前坡道由多个个坡道切片组成且包含第一段
    for (uint8_t k = slope_smooth_info.slope_start_id; k <= slope_smooth_info.slope_end_id; k++) {
      slope_smooth_info.slope_smooth += slope_nodes_.nodes[k].slope_length / slope_smooth_info.slope_length * slope_nodes_.nodes[k].average_slope;
    }
    slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_ONE;
  } else if ((slope_smooth_info.slope_start_id == slope_smooth_info.slope_end_id) && (slope_smooth_info.slope_start_id != 0)){ // 当前坡道由单个坡道切片组成且切片不包含第一段
    slope_smooth_info.slope_smooth = slope_nodes_.nodes[slope_smooth_info.slope_start_id].average_slope;
    if (slope_nodes_.nodes[slope_smooth_info.slope_start_id - 1].slope_type == SLOPE_TYPE_DOWN_SLOPE) {
      slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_TWO;
    } else {
      slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_THREE;
    }
  } else if ((slope_smooth_info.slope_start_id != slope_smooth_info.slope_end_id) && (slope_smooth_info.slope_start_id != 0)){ //当前坡道由多条坡道切片组成且不包含第一段
    for (uint8_t l = slope_smooth_info.slope_start_id; l <= slope_smooth_info.slope_end_id; l++) {
      slope_smooth_info.slope_smooth += slope_nodes_.nodes[l].slope_length / slope_smooth_info.slope_length * slope_nodes_.nodes[l].average_slope;
    }
    if (slope_nodes_.nodes[slope_smooth_info.slope_start_id - 1].slope_type == SLOPE_TYPE_DOWN_SLOPE) {
      slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_TWO;
    } else {
      slope_smooth_info.uphill_type = UphillType::UPHILL_TYPE_THREE;
    }
  } else {
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
    std::cout << " logic miss check " << std::endl;
#endif
    return false;
  }
  
  if (slope_smooth_info.slope_start_id != 0) {
    for (int m = 0; m < slope_smooth_info.slope_start_id; m++) {
      slope_smooth_info.distance_to_slope += slope_nodes_.nodes[m].slope_length;
    } 
  } else {
    slope_smooth_info.distance_to_slope = 0.0f;
  }
  slope_smooth_info.slope_smooth = slope_smooth_info.slope_smooth * 100; //  scaling slope unit
  return true;
}

Float32_t VelocityPlanningPolynomialFitting::LookupDecTable(const Float32_t slope_smooth, const Float32_t slope_length){
#if ENABLE_VELOCITY_PLANNING_POLYNOMIAL_SLOPE_SEGMRNT_TRACE
  if ((slope_smooth <= 0.0f) || (slope_length <= 0.0f)) {
    std::cout << "[LookupDecTable] ERROR INPUT PRARAM !!! " << std::endl;
    return -1.0f;
  }
#endif

  static const uint8_t max_slope_index = 6;
  static const uint8_t max_length_index = 15;
  static const uint8_t slope_table[max_slope_index] = {1, 2, 3, 4, 5, 6};
  static const uint32_t length_table[max_length_index] = {200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000};
  #if 0 // 根据仿真的坡长、坡底速度、坡顶速度计算的理论减速度值
  static const Float32_t dec_table[max_slope_index][max_length_index] = {
    0.070f, 0.044f,	0.033f,	0.024f,	0.020f,	0.016f,	0.014f,	0.013f,	0.011f,	0.010f,	0.009f,	0.008f,	0.008f,	0.007f,	0.007f,
    0.135f,	0.089f,	0.063f,	0.050f,	0.042f,	0.038f,	0.035f,	0.033f,	0.031f,	0.029f,	0.028f,	0.027f,	0.026f,	0.025f,	0.024f,
    0.202f,	0.150f,	0.136f,	0.130f,	0.124f,	0.116f,	0.104f,	0.095f,	0.088f,	0.083f,	0.078f,	0.074f,	0.071f,	0.067f,	0.062f,
    0.275f,	0.264f,	0.246f,	0.218f,	0.198f,	0.171f,	0.146f,	0.127f,	0.113f,	0.102f,	0.093f,	0.085f,	0.078f,	0.073f,	0.068f, 
    0.386f,	0.366f,	0.328f,	0.276f,	0.220f,	0.184f,	0.158f,	0.138f,	0.123f,	0.110f,	0.100f,	0.092f,	0.085f,	0.079f,	0.074f,
    0.476f,	0.460f,	0.384f,	0.290f,	0.232f,	0.193f,	0.165f,	0.145f,	0.129f,	0.116f,	0.105f,	0.097f,	0.089f,	0.083f,	0.077f };
  #endif

  #if 1 // 根据仿真结果和表理论减速度值对比，结合实际情况考量，所得的减速度值
  static const Float32_t dec_table[max_slope_index][max_length_index] = {
    0.055f,	0.044f,	0.033f,	0.024f,	0.020f,	0.016f,	0.014f,	0.013f,	0.011f,	0.010f,	0.009f,	0.008f,	0.008f,	0.007f,	0.007f,
    0.106f,	0.058f,	0.058f,	0.050f,	0.042f,	0.038f,	0.035f,	0.033f,	0.031f,	0.029f,	0.028f,	0.027f,	0.026f,	0.025f,	0.024f,
    0.170f,	0.150f,	0.150f,	0.130f,	0.124f,	0.116f,	0.104f,	0.095f,	0.088f,	0.083f,	0.078f,	0.030f,	0.030f,	0.030f,	0.030f,
    0.260f,	0.250f,	0.246f,	0.218f,	0.130f,	0.130f,	0.130f,	0.127f,	0.113f,	0.102f,	0.093f,	0.085f,	0.078f,	0.073f,	0.068f,
    0.340f,	0.340f,	0.328f,	0.276f,	0.220f,	0.184f,	0.158f,	0.138f,	0.123f,	0.110f,	0.100f,	0.092f,	0.085f,	0.079f,	0.074f,
    0.440f,	0.420f,	0.384f,	0.290f,	0.232f,	0.193f,	0.165f,	0.145f,	0.129f,	0.116f,	0.105f,	0.097f,	0.089f,	0.083f,	0.077f };
  #endif

  int slope_index = 0;
  int length_index = 0;
  // TODO：二分法查表提高效率
  // int index_left = 0;
  // int index_right = 0;

  // 目前往小取减速度值，避免车速掉的过快
  if (slope_length <= length_table[0]) {
    length_index = 0;
  } else if (slope_length >= length_table[max_length_index - 1]) {
    length_index = max_length_index - 1;
  } else {
    for (uint8_t i = 0; i < max_length_index - 1; i++) {
      if ((slope_length >= length_table[i]) && (slope_length < length_table[i + 1])) {
        length_index = i;
      }    
    }
  } 
  
  if (slope_smooth < 1.0f) { // TODO: 0~1% 的坡度临时渐进过度处理
    return slope_smooth * dec_table[0][length_index] * 0.2f;
  } else if (slope_smooth >= slope_table[max_slope_index - 1]) {
    slope_index = max_slope_index - 1;
  } else {
    for (uint8_t i = 0; i < max_slope_index - 1; i++) {
      if ((slope_smooth >= slope_table[i]) && (slope_smooth < slope_table[i + 1])) {
        slope_index = i;
      }    
    }
  }
  return dec_table[slope_index][length_index] * 0.2f;
}

#endif

void VelocityPlanningPolynomialFitting::TargetVelAndEgoVelDeltaSmooth(plan_var_t &tar_v, plan_var_t vel_delta_threshold, plan_var_t dec_step, plan_var_t acc_step) {
  plan_var_t tar_v_pre = chassis_status_.v;
  if (common::com_abs(tar_v - tar_v_pre) >= vel_delta_threshold) {//设定速度差超过 Delta 将进行平滑。
    if ((tar_v_pre - tar_v) >= 0.0) {
      tar_v_pre = tar_v_pre - dec_step;
    } else {
      tar_v_pre = tar_v_pre + acc_step;
    }
    tar_v = tar_v_pre;
  } else {
    /*do nothing*/
  }
}

void VelocityPlanningPolynomialFitting::TargetVelAndEgoVelDeltaSmooth(plan_var_t &tar_v, plan_var_t cur_slope) {
  const plan_var_t vel_delta_fixed = 5.0f / 3.6f;
  const plan_var_t vel_diff_threshold = 5.0f / 3.6f;

  bool use_fix_delta_to_step_cc_ = false;
  plan_var_t vel_delta = 0.0F;

  // 根据坡度细化阶梯步进值
  if (use_fix_delta_to_step_cc_) {
    vel_delta = vel_delta_fixed;
  } else {
    if (cur_slope < -1.0F) {
      return;
    } else if ((cur_slope < 0) && (cur_slope >= -1.0F)) {  // 此处坡度取值不包含0是因为避免在隧道内，实际坡度很大，而MPU坡度为0时发生掉速行为
      vel_delta = 3.5F / 3.6F;
    } else if ((cur_slope > 0) && (cur_slope <= 0.3F)) { // 上坡 (0, 1.5] -> 阶梯车速,减少低速到高速加速情况; (1.5, ~) -> 根据油门曲线限制油门开度
      vel_delta = 3.5F / 3.6F; // TODO: 泛化，需考虑载重，本质上解决还是需要计算 经济参考车速
    } else if ((cur_slope > 0.3) && (cur_slope <= 1.5F)) {
      vel_delta = cur_slope;
    } else {
      return;
    }
  }

  plan_var_t tar_v_pre = chassis_status_.v;

  if (common::com_abs(tar_v - tar_v_pre) >= vel_diff_threshold) {
    if ((tar_v_pre - tar_v) >= 0.0) {
      tar_v_pre = tar_v_pre - vel_delta; // FIXME: 存在此种场景?
    } else {
      tar_v_pre = tar_v_pre + vel_delta;
    }
    tar_v = tar_v_pre;
  } else {
    /*do nothing*/
  }
}
  
// 输入车速滤波
void VelocityPlanningPolynomialFitting::PCCinputfilter(PAccInput &pcc_input) {
#if ENABLE_PCC_FILTER_VELOCITY
  static Float32_t pre_v = pcc_input.ego_speed;
  
  Float32_t cur_v = pcc_input.ego_speed;
  Float32_t pre_v_ratio = 0.9;
  Float32_t cur_v_ratio = 0.1;
  Float32_t temp_v_filter = pcc_input.ego_speed;

  if (pcc_input.pre_velocity_planning_type != 15) {
    pre_v = pcc_input.ego_speed;
    // temp_v_filter = pre_v_ratio * pre_v + cur_v_ratio * cur_v;
    // pre_v = temp_v_filter;
    // pcc_input.ego_speed = temp_v_filter;
  } else {
    temp_v_filter = pre_v_ratio * pre_v + cur_v_ratio * cur_v;
    pre_v = temp_v_filter;
    pcc_input.ego_speed = temp_v_filter;
  }
  
  common::StaticVector<Float32_t, 14> Ig;
  Ig.Clear();
  Ig.PushBack(0.0F);
  Ig.PushBack(16.688F);
  Ig.PushBack(12.924F);
  Ig.PushBack(9.926F);
  Ig.PushBack(7.688F);
  Ig.PushBack(5.895F);
  Ig.PushBack(4.565F);
  Ig.PushBack(3.655F);
  Ig.PushBack(2.831F);
  Ig.PushBack(2.174F);
  Ig.PushBack(1.684F);
  Ig.PushBack(1.291F);
  Ig.PushBack(1.0F);
  Float32_t rw = 0.5267F;
  Float32_t If = 2.69F;
  Float32_t temp_engine_speed_filter = temp_v_filter * If * Ig[pcc_input.gear_num] * 30 / 3.14 / rw;

  if (pcc_input.engine_speed > 650) {
    pcc_input.engine_speed = temp_engine_speed_filter;
  }

#endif

  // 坡度滤波
#if ENABLE_PCC_FILTER_TBOX_SLOPES

  if (pcc_input.pacc_map.pacc_points_source != 1) {
    return;
  }
  
  static const Uint16_t PACC_MAX_POINT_NUM = 500;
  
  common::StaticVector<PACCPoint, PACC_MAX_POINT_NUM> temp_points;

  int point_size = pcc_input.pacc_map.points.Size();

  PACCPoint temp_point;

  temp_points.PushBack(pcc_input.pacc_map.points[0]);

  for (int i = 1; i < point_size; i++) {

    temp_point.slope = 0.1 * pcc_input.pacc_map.points[i].slope + 0.9 * temp_points[i-1].slope;
    temp_point.s = pcc_input.pacc_map.points[i].s;

    temp_points.PushBack(temp_point);
  }
  pcc_input.pacc_map.points.Clear();

  for (int i = 20; i < temp_points.Size(); i++) {
    temp_point.slope = temp_points[i].slope;
    temp_point.s = (i - 20) * pcc_input.pacc_map.pacc_point_s_step;
    pcc_input.pacc_map.points.PushBack(temp_point);
  }

  pcc_input.pacc_map.pacc_point_num = pcc_input.pacc_map.points.Size();

#endif

  return;
}

//PCC输出的油门开度滤波
void VelocityPlanningPolynomialFitting::PCCoutputfilter(PAccOutput &pcc_output, const int &pre_vel_plan_type) {
  
  static Float32_t pre_target_throttle = pcc_output.tar_engine_throttle;

  static bool pre_is_kickdown = false;
  
  Float32_t cur_target_throttle = pcc_output.tar_engine_throttle;
  
  Float32_t pre_target_ratio = 0.9;
  Float32_t cur_target_ratio = 0.1;
  Float32_t temp_target_filter = pcc_output.tar_engine_throttle;

  

  if (pre_vel_plan_type != 15 || pre_is_kickdown) {
    pre_target_throttle = pcc_output.tar_engine_throttle;
    // temp_target_filter = pre_target_ratio * pre_target_throttle + cur_target_ratio * cur_target_throttle;
    // pre_target_throttle = temp_target_filter;
    pre_is_kickdown = false;
    // pcc_output.tar_engine_throttle = temp_target_filter;
  } else { 
    if(cur_target_throttle > 90) {
      temp_target_filter = cur_target_throttle;

      pre_is_kickdown = true;
    } else {
      temp_target_filter = pre_target_ratio * pre_target_throttle + cur_target_ratio * cur_target_throttle;
      pre_target_throttle = temp_target_filter;
      pre_is_kickdown = false;
    }

    pcc_output.tar_engine_throttle = temp_target_filter;
  }

  return;
}


}  // namespace planning
} // namespace phoenix
