/*****************************************************************************
 * @file       velocity_planning_fuzzy_pid.cc
 * @brief      速度规划（Fuzzy PID）
 * @details    使用模糊PID算法，规划目标车速及目标加速度
 *
 * @author     longjiaoy
 * @date       2020.10.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/07/16  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "velocity_planning/velocity_planning_fuzzy_pid.h"

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
#include "Sys_InfoCom.h"
#endif
#include "KTMPUStateOut.h" //


// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <cstdint>
// #include <iostream>
// #include <vector>

#define ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE (1)
#define ENABLE_VELOCITY_PLANNING_LOG (0)
#define ENABLE_VELOCITY_PLANNING_FUZZY_PID_PERFORMANCE_TEST (0)
#define ENABLE_FUZZY_TABLE_TEST (0)

// BY ZQ
#define ENABLE_ACC_STOP_Judge (1)
#define ENABLE_CUTIN_Judge (1)
#define ENABLE_ACC_START_Judge (1)
#define ENABLE_Tar_state (1)
#define ENABLE_Cur_state (0)
#define ENABLE_Static_Obstacle_Stop (1)
#define ENABLE_CC_To_ACC (1)

#define ENABLE_CURVATURE (0)

#define ENABLE_SLOPE_MASS (1)
#define ENABLE_OBSTACLE_Decision (1)
#define ENABLE_SPEED_LIMIT_FROM_DRIVER_MAP (0)

#define ENABLE_ADECU (0)

#define TM_ZF (0)
#define TM_YD (1)


namespace phoenix {
namespace planning {

#if 1
#if (ENABLE_VELOCITY_PLANNING_DEBUGGING_FILE)
VelocityPlanningFuzzyPID::VelocityPlanningFuzzyPID()
    : log_file_speed("speed_acc")
#else
VelocityPlanningFuzzyPID::VelocityPlanningFuzzyPID()
#endif
{
  driving_map_ = Nullptr_t;

  SetDefaultParam();

  driving_direction_ = DRIVING_DIRECTION_FORWARD;

  ClearHoldInfo();
#if (ENABLE_VELOCITY_PLANNING_DEBUGGING_FILE)
  open_log_file_speed = false;
#endif
}
#endif

#if 0
    VelocityPlanningFuzzyPID::VelocityPlanningFuzzyPID()
    {
        driving_map_ = Nullptr_t;

        SetDefaultParam();

        driving_direction_ = DRIVING_DIRECTION_FORWARD;

        ClearHoldInfo();
    }
#endif

VelocityPlanningFuzzyPID::~VelocityPlanningFuzzyPID() {
  // noting to do
}

// 设置速度默认参数
void VelocityPlanningFuzzyPID::SetDefaultParam() {
  settings_.enable_acc = true;
  settings_.enable_aeb = true;
  settings_.enable_aeb_pre_dec = true;
  settings_.enable_following = true;
  settings_.enable_low_speed_following = true;
  settings_.tar_v = 10.0F / 3.6F;
  settings_.tar_a = 1.0F;
  settings_.lateral_acceleration_limit = 1.0F;
  settings_.enable_stop_by_traffic_light = false;

  veh_model::VehicleModelWrapper veh_model;
  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front = veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear = veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center = veh_model.GetDistOfLocalizationToCenter();

  /*
   * @brief 在指定位置，以指定的航向角生成车辆在此位姿下的有向矩形包围盒//区别path OBB
   * @param[in] pos 位置：每次当前时刻位置原点(0,0)
   * @param[in] heading 航向角：沿着速度方向0°
   * @param[out] obb 车辆在对应位姿下的有向矩形包围盒
   * @note 生成的obb用于碰撞分析。
   */
  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0, 0), 0, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();

  param_.max_velocity_limit = 100.0F / 3.6F;  // 最大速度限制 100km/h
  param_.max_acceleration_limit = 3.0F;       // 最大加速度限制
  param_.max_deceleration_limit = -7.0F;      // 最大减速度限制
  param_.tunnel_velocity_limit = 80.0F / 3.6F; // 隧道速度限制
  param_.ramp_velocity_limit = 50.0F / 3.6F;  // 匝道速度限制

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

  // 根据与静态障碍物之间的距离，限制与障碍物之间的最大相对速度（static 在障碍物速度0m/s基础上+抬高的相对速度，即如下表：<0.2m,0m/s;<0.5m,减速 ）
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
  // 根据与动态障碍物之间的距离，限制与障碍物之间的最大相对速度（dynamic 在障碍物速度基础上+抬高的相对速度 ）
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
#if 0
        param_.aeb_action_ttc_table.Clear();
        param_.aeb_action_ttc_table.PushBack(
                common::LerpTableNodeType1(0.0F/3.6F, 2.0F));
        param_.aeb_action_ttc_table.PushBack(
                common::LerpTableNodeType1(20.0F/3.6F, 2.0F));
        param_.aeb_action_ttc_table.PushBack(
                common::LerpTableNodeType1(30.0F/3.6F, 2.0F));
        param_.aeb_action_ttc_table.PushBack(
                common::LerpTableNodeType1(40.0F/3.6F, 2.0F));
        param_.aeb_action_ttc_table.PushBack(
                common::LerpTableNodeType1(50.0F/3.6F, 2.0F));
        param_.aeb_action_ttc_table.PushBack(
                common::LerpTableNodeType1(60.0F/3.6F, 2.0F));
#else
  param_.aeb_action_ttc_table.Clear();
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(0.0F / 3.6F, 1.0F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(20.0F / 3.6F, 1.0F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(30.0F / 3.6F, 1.2F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(40.0F / 3.6F, 1.2F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(50.0F / 3.6F, 1.8F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(60.0F / 3.6F, 1.8F));
  param_.aeb_action_ttc_table.PushBack(common::LerpTableNodeType1(70.0F / 3.6F, 2.0F));
#endif

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

//AEB之前有个hold状态：障碍物持续5帧以下就消失-3m/s2减速度，否则-5m/s2减速度
  action_smoothing_aeb_action_.set_req_count_threshold(5);
  action_smoothing_aeb_action_.set_time_allowed(10);
  action_smoothing_aeb_warning_.set_req_count_threshold(2);
  action_smoothing_aeb_warning_.set_time_allowed(5);
  action_smoothing_notify_uncertain_.set_req_count_threshold(5);
  action_smoothing_notify_uncertain_.set_time_allowed(10);
}

void VelocityPlanningFuzzyPID::UpdateVehicleParameter() {
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
  //车辆宽度、挂车状态、质量 载荷
  LOG_INFO(5) << "Vehicle Width = " << param_.vehicle_width
              << ", Trailer Status = " << chassis_status_.trailer_status
              << ", Vehicle Weight = " << chassis_status_.gross_weight;
}



void VelocityPlanningFuzzyPID::Configurate(const VelocityPlanningConfig &conf) {
  config_ = conf;

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

void VelocityPlanningFuzzyPID::ClearHoldInfo() {
  obj_jitter_suppression_.Clear();
  action_smoothing_aeb_action_.Clear();
  action_smoothing_aeb_warning_.Clear();
  action_smoothing_notify_uncertain_.Clear();

  status_.Clear();
  pre_adas_mode_ = false;
  valid_target_velocity_array_ = false;
  common::com_memset(array_target_velocity_, 0, sizeof(array_target_velocity_));
  common::com_memset(array_target_acceleration_, 0, sizeof(array_target_acceleration_));

  event_reporting_list_.Clear();
}
// PLAN MAIN
bool VelocityPlanningFuzzyPID::Plan(const VelocityPlanningDataSource &data_source) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[planning][velocity] ######### Velocity planning (Fuzzy PID) >>>>>>>>>" << std::endl;
#endif

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_PERFORMANCE_TEST
  /// Performance of velocity planning (Start)
  phoenix::common::Stopwatch performance_timer_velocity_planning;
#endif

  /// TODO:防抖抑制 obj_jitter_suppression_（防止频繁加减）
  obj_jitter_suppression_.acc_delay_time--;
  if (obj_jitter_suppression_.acc_delay_time < 0) {
    obj_jitter_suppression_.acc_delay_time = 0;
  }
  obj_jitter_suppression_.dec_delay_time--;
  if (obj_jitter_suppression_.dec_delay_time < 0) {
    obj_jitter_suppression_.dec_delay_time = 0;
  }

  event_reporting_list_.Clear();

  result_of_planning_.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
  // 发生内部错误时，释放油门

  /// XXX:开始一系列自检
  // 1：判断data_source内的数据是否存在相关信息，若为空则记录LOG_ERR；2：清除规划信息；3：返回为false

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

  // XXX: 1：获取底盘和车身信息，根据档位判断是前进还是倒车；2：获取行为规划结果（ACC/AEB）； 3：将巡航车速限制到0~120KM/H，加速度限制到0~2m/s^2
  chassis_status_ = *(data_source.chassis);                   // 获取底盘状态
  special_chassis_status_ = *(data_source.special_chassis);
  chassis_ctl_cmd_ = *(data_source.chassis_ctl_cmd);          // 获取底盘特定型号车辆的信息
  traffic_light_list_ = *(data_source.traffic_light_list);    // 获取traffic_light_list
  traffic_signal_list_ = *(data_source.traffic_signal_list);  // 获取traffic_signal_list
  // XXX: 需要将目标轨迹同步到当前坐标 Need to synchronize the target trajectory to current coordinate.
  curr_timestamp_ = data_source.timestamp;                                        // 获取时间戳
  driving_map_ = data_source.driving_map;                                         // 获取驾驶地图
  result_of_action_planning_ = *(data_source.result_of_action_planning);          // 获取行为规划的结果
  result_of_trajectory_planning_ = *(data_source.result_of_trajectory_planning);  // 获取轨迹规划的结果

#if ENABLE_ADECU
  if (special_chassis_status_.dfcv_d17.ADCU_Mode_Sts == 4 ||
      special_chassis_status_.dfcv_d17.ADCU_Mode_Sts == 5) {
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
  // 在非自动驾驶的形态对 obj_jitter_suppression_ 进行clear()
  if (!adas_mode_) {
    obj_jitter_suppression_.Clear();
  }

  // TODO：获取当前控制下发的油门量
  control_acc_pedal_ = chassis_ctl_cmd_.acc_value;

  // TODO：获取当前坡度 和 坡度轨迹的最后一个点的坡度（单位：%）
  #if ENABLE_SLOPE_MASS
    if (!result_of_trajectory_planning_.target_trajectory_slope.Empty()) {
      slope_flag_ = true;
      curr_slope_ = result_of_trajectory_planning_.target_trajectory_slope.Front().slope;
    } else {
      slope_flag_ = false;
      curr_slope_= 0.0F;
    }
  #endif

  // 获取底盘当前挡位（前进档和后退档）
  if (ad_msg::VEH_GEAR_R == data_source.chassis->gear) {
    driving_direction_ = DRIVING_DIRECTION_BACKWARD;
  } else {
    driving_direction_ = DRIVING_DIRECTION_FORWARD;
  }

#if 1  // 设定最大最小速度和加速度的限制
  settings_.enable_acc = result_of_action_planning_.enable_acc;

// 在ADCU上需要通过标定方式关闭AEB功能
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

  settings_.tar_v = result_of_action_planning_.v_setting;
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
#endif

  // 根据是否有油门踏板及驾驶模式设置来抑制AEB
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

  /// XXX: 更新当前位姿信息（TrajectoryPoint）
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

  /// XXX: 获取当前参考线，将车辆位姿投影在参考线上，计算参考线剩余长度
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

#if 0
    plan_var_t major_continuous_seg_start_s = 0.0F;
    plan_var_t major_continuous_seg_end_s = 0.0F;
    driving_map_->GetReferenceLineContinuousSegment(
            major_ref_line_index,
            &major_continuous_seg_start_s, &major_continuous_seg_end_s);
    printf("### Speed planning: major_ref_line(lenght=%0.1f)"
            ", continuous(%0.1f, %0.1f)\n",
            major_ref_line.total_length(),
            major_continuous_seg_start_s, major_continuous_seg_end_s);
#endif

  Int32_t driving_map_type = driving_map_->GetDrivingMapType();

  target_trajectory_curvature_info_.Clear();
  target_trajectory_.AnalyzeCurvature(&target_trajectory_curvature_info_);
  target_trajectory_.FindProjection(curr_position_.path_point.point, &veh_proj_on_target_trajectory_);

  tar_v_info_list_.Clear();
  TargetVelocityInfo tar_v_info;
  TargetVelocityInfo tar_v_obj_info;

//XXX: A：各种限速 => 都PushBack存入列表tar_v_info_list_  
#if 0
  //XXX: A1. Plan velocity according to Curvature
  #if (ENABLE_CURVATURE)
  plan_var_t target_velocity_curvature;
  bool Curvaturetarget_velocity_vail = TargetVelocityFromPathCurvature(major_ref_line, major_ref_line_curvature_info, veh_proj_on_major_ref_line_, &target_velocity_curvature);
   std::cout << "[planning][target_velocity][curvature]" << target_velocity_curvature * 3.6  << std::endl;
  #else
  PlanVelocityAccordingToPathCurvature(major_ref_line, major_ref_line_curvature_info, veh_proj_on_major_ref_line_, 0.0F,
                                       major_ref_line.total_length(), &tar_v_info);

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
  #endif
  
  #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[planning][velocity][curvature][refline] tar_type =" << tar_v_info.tar_type
              << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
  #endif

  // FIXME: A1.1 换道不进行曲率限速；当车道保持LKA/左让行/右让行的状态时，曲率限速（多项式）when curvature path comes from LCA, DON'T decelerate
  if ((TRJ_STATUS_LKA == result_of_trajectory_planning_.trj_status) || 
      (TRJ_STATUS_LKA_BYPASSING_LEFT == result_of_trajectory_planning_.trj_status) ||
      (TRJ_STATUS_LKA_BYPASSING_RIGHT == result_of_trajectory_planning_.trj_status)) {
        PlanVelocityAccordingToPathCurvature(target_trajectory_, target_trajectory_curvature_info_,
                                            veh_proj_on_target_trajectory_, 0.0F, target_trajectory_.total_length(),
                                            &tar_v_info);
        if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
          tar_v_info_list_.PushBack(tar_v_info);
        }

        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[planning][velocity][curvature][pln-path] tar_type =" << tar_v_info.tar_type
                  << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
        #endif
      }else{
        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "换道等状态时，不进行曲率限速"  << std::endl;
        #endif
      }
    
    //FIXME: A1.2 换道不加速 output
    
      if((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == result_of_trajectory_planning_.trj_status) || 
            (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == result_of_trajectory_planning_.trj_status) ||
            (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == result_of_trajectory_planning_.trj_status) ||
            (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == result_of_trajectory_planning_.trj_status)){
                        std::cout << " 换道不加速 " << std::endl;
            tar_v_info.tar_v = curr_v;
            tar_v_info.tar_a = 0.0;
            tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_CURVATURE ;

            tar_v_info_list_.PushBack(tar_v_info);

          #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[换道不加速 planning][velocity][curvature][pln-path] tar_type =" << tar_v_info.tar_type
                  << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
         #endif
    }
#endif 

//XXX: A2. Plan velocity according to Obstacles
#if ENABLE_OBSTACLE_Decision
  PlanVelocityAccordingToObstacles(&tar_v_info);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
  #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[planning][velocity][Obstacles] tar_type =" << tar_v_info.tar_type
              << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
  #endif
#else
  PlanVelocityAccordingToObstacles(&tar_v_obj_info);
  #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[planning][velocity][Obstacles] tar_type =" << tar_v_obj_info.tar_type
              << ", tar_v = " << tar_v_obj_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_obj_info.tar_a << "m/s^2" << std::endl;
  #endif
#endif

  //XXX: A3. Plan velocity according to traffic light
  PlanVelocityAccordingToTrafficLight(major_ref_line, veh_proj_on_major_ref_line_, &tar_v_info);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[planning][velocity][TrafficLight] tar_type =" << tar_v_info.tar_type
            << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
#endif

  //XXX: A4. Plan Velocity According To Traffic Signal object
  PlanVelocityAccordingToTrafficSignal(major_ref_line, veh_proj_on_major_ref_line_, &tar_v_info);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[planning][velocity][TrafficSignal] tar_type =" << tar_v_info.tar_type
            << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
#endif

  //XXX: A5. Plan Velocity According To Scene Story object
  PlanVelocityAccordingToSceneStory(major_ref_line, veh_proj_on_major_ref_line_, &tar_v_info);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[planning][velocity][SceneStory] tar_type =" << tar_v_info.tar_type
            << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
#endif

  //XXX: A6. Plan Velocity According To Tunnel object
#if 0
    PlanVelocityAccordingToTunnel(major_ref_line, veh_proj_on_major_ref_line_, &tar_v_info);
    if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
      tar_v_info_list_.PushBack(tar_v_info);
    }
  #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[planning][velocity][Tunnel] tar_type =" << tar_v_info.tar_type
              << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
  #endif
#endif

  //XXX: A7. Plan Velocity According To Ramp object
  PlanVelocityAccordingToRamp(major_ref_line, veh_proj_on_major_ref_line_, &tar_v_info);
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    tar_v_info_list_.PushBack(tar_v_info);
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[planning][velocity][Ramp] tar_type =" << tar_v_info.tar_type
            << ", tar_v = " << tar_v_info.tar_v * 3.6 << "km/h, tar_a = " << tar_v_info.tar_a << "m/s^2" << std::endl;
#endif

  // 初始化result_of_planning_
  result_of_planning_.msg_head = driving_map_->GetRelativePosList().msg_head;
  // tar_type类型： 用户设定巡航
  result_of_planning_.tar_type = VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS;
  // 巡航 80m
  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;
  param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
  param_of_calc_tar_v.stop_accurately = false;
  param_of_calc_tar_v.min_deceleration_time = 2.0F;

  result_of_planning_.tar_v = settings_.tar_v;  // HMI Set Speed
  result_of_planning_.tar_a = settings_.tar_a;

  if (/*(status_.exist_uncertain_obstacle) ||*/
      (driv_map::DRIVING_MAP_TYPE_PRED_PATH == driving_map_type) ||
      (driv_map::DRIVING_MAP_TYPE_INVALID == driving_map_type)) {
    result_of_planning_.release_throttle = true;
  } else {
    result_of_planning_.release_throttle = false;
  }

  result_of_planning_.tar_pos.x = 0.0F;
  result_of_planning_.tar_pos.y = 0.0F;
  result_of_planning_.tar_pos.heading = 0.0F;
  result_of_planning_.tar_pos.s = 0.0F;
  result_of_planning_.tar_obj.valid = false;
  result_of_planning_.tar_obj.dist_to_obj = 0.0F;
  result_of_planning_.tar_obj.time_gap = 0.0F;
  result_of_planning_.tar_obj.relative_v = 0.0F;
  result_of_planning_.tar_obj.ttc = 100.0F;
  result_of_planning_.tar_obj.obj_v = 0.0F;
  result_of_planning_.tar_obj.dist_gap_level = -1;
  result_of_planning_.tar_obj.rel_spd_level = -1;
  result_of_planning_.tar_obj.obj_dec_status = obj_jitter_suppression_.status;
  
  // TODO: B：对A中列表tar_v_info_list_的tar_a和tar_v各自取小
  Int32_t tar_v_info_list_size = tar_v_info_list_.Size();

  Int32_t tar_a_limit_index = 0;
  for (Int32_t i = 1; i < tar_v_info_list_size; ++i) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " vel planning list [i] = " << i << " tar_type = " << tar_v_info_list_[tar_a_limit_index].tar_type
              << "tar_a =" << tar_v_info_list_[tar_a_limit_index].tar_a << std::endl;
#endif
    if (tar_v_info_list_[i].tar_a < tar_v_info_list_[tar_a_limit_index].tar_a) {
      tar_a_limit_index = i;
    }
  }

  Int32_t tar_v_limit_index = 0;
  for (Int32_t i = 1; i < tar_v_info_list_size; ++i) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " vel planning list [i] = " << i << " tar_type = " << tar_v_info_list_[tar_v_limit_index].tar_type
              << "tar_v =" << tar_v_info_list_[tar_v_limit_index].tar_v << std::endl;
#endif
    if (tar_v_info_list_[i].tar_v < tar_v_info_list_[tar_v_limit_index].tar_v) {
      tar_v_limit_index = i;
    }
  }

// TODO: C：巡航 VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << " ===============CC ===========CalcTarVelocityQuinticPolynomialCurve1d START==========" << std::endl;
#endif

  //TODO: C1.隧道限速
  #if 1 
  TargetVelocityUpperLimit target_velocity_info;
  CalculateTargetVelocityFromTunnel(&target_velocity_info); //PlanVelocityAccordingToTunnel区别？

  if (target_velocity_info.valid) {
    if (target_velocity_info.tar_v > 0 && target_velocity_info.tar_v < settings_.tar_v){
        settings_.tar_v = target_velocity_info.tar_v;
        result_of_planning_.tar_type = target_velocity_info.tar_type;
        std::cout << " 隧道限速 lane_speed_limit :" << settings_.tar_v << std::endl;
    }
  }  
  #endif

  //TODO: C2.道路限速（数据源：驾驶地图/MPU）
  #if 1
    #if ENABLE_SPEED_LIMIT_FROM_DRIVER_MAP
    //从DRIVER_MAP获取道路限速信息
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
      #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
      std::cout << " lane_speed_limit :" << lane_speed_limit << std::endl;
      #endif
    }
    if (lane_speed_limit > 0 && lane_speed_limit < settings_.tar_v){
      settings_.tar_v = lane_speed_limit;
      result_of_planning_.tar_type = VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT;
      #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
      std::cout << " 道路限速 lane_speed_limit :" << lane_speed_limit << std::endl;
      #endif
    }

    #else
    // 从MPU获取道路限速信息
    plan_var_t lane_speed_limit = -1.0F;
    mpu_t::MpuState mpu_state;
    if (!mpu_t::GetMpuState(mpu_state)) {
      LOG_INFO(3) << "Failed to get mpu state.";
    }
    lane_speed_limit = mpu_state.MonitorState.LimitSpeedMax / 3.6F;
    #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " lane_speed_limit : " << lane_speed_limit << " settings_.tar_v = " << settings_.tar_v << std::endl;
    #endif
    if (lane_speed_limit > 0 && lane_speed_limit < settings_.tar_v){
      settings_.tar_v = lane_speed_limit;
      result_of_planning_.tar_type = VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT;
      #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
      std::cout << " 道路限速 lane_speed_limit :" << lane_speed_limit << std::endl;
      #endif
    }
    #endif

  #endif

//TODO: C3. 换道不加速 input
#if 1 
  plan_var_t settings_v = settings_.tar_v;
  //if (result_of_trajectory_planning_.trj_changing.is_changing) { 
   if((TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == result_of_trajectory_planning_.trj_status) || 
      (TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == result_of_trajectory_planning_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == result_of_trajectory_planning_.trj_status) ||
      (TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == result_of_trajectory_planning_.trj_status)||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I == result_of_trajectory_planning_.trj_status) || 
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II == result_of_trajectory_planning_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_I == result_of_trajectory_planning_.trj_status) ||
      (TRJ_STATUS_ABORT_CHANGING_LANE_TO_LEFT_II == result_of_trajectory_planning_.trj_status)) {
          #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE      
          std::cout << " is_changing " << std::endl;
          #endif
    if (settings_v > curr_position_.v) {//避免换道加速
      settings_v = curr_position_.v;
    }
  }else{
          #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << " no_changing " << std::endl;
          #endif
  }

  
  CalcTarVelocityQuinticPolynomialCurve1d(param_of_calc_tar_v, curr_position_.v, settings_v, 150.0, 0.0F, true,//settings_v
                                          &(result_of_planning_.tar_v), &(result_of_planning_.tar_a));
        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 换道 CC :"<< "tar_v =" << result_of_planning_.tar_v<< " tar_a =" <<result_of_planning_.tar_a<<  std::endl;
        #endif
  #else
  //TODO: C3. 换道直接CC2ACC
  CalcTarVelocityQuinticPolynomialCurve1d(param_of_calc_tar_v, curr_position_.v, settings_.tar_v, 150.0, 0.0F, true,
                                          &(result_of_planning_.tar_v), &(result_of_planning_.tar_a));
        #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 车道保持 CC :"<< "tar_v =" << result_of_planning_.tar_v<< " tar_a =" <<result_of_planning_.tar_a<<  std::endl;
        #endif
  #endif
  
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << " ===============CC ===========CalcTarVelocityQuinticPolynomialCurve1d END==========" << std::endl;
#endif  

#if ENABLE_OBSTACLE_Decision
// TODO: B：如果列表tar_v_info_list_不为空，则需要限速；
  if (tar_v_info_list_size > 0) { 
    //巡航CC单独和各种限速比较：
    if (tar_v_info_list_[tar_v_limit_index].tar_v < result_of_planning_.tar_v + 0.5) { //0.5m/s为了区分
      if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == tar_v_info_list_[tar_v_limit_index].tar_type ||
          VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == tar_v_info_list_[tar_v_limit_index].tar_type ||
          VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == tar_v_info_list_[tar_v_limit_index].tar_type) {
        if (tar_v_info_list_[tar_v_limit_index].obstacle_info.obj_v > settings_.tar_v 
            && tar_v_info_list_[tar_v_limit_index].obstacle_info. dist_to_obj > curr_position_.v * 0.5) {
            // NOTHING
        } else {
          result_of_planning_.tar_type = tar_v_info_list_[tar_v_limit_index].tar_type;
          result_of_planning_.tar_v = tar_v_info_list_[tar_v_limit_index].tar_v;
          result_of_planning_.tar_a = tar_v_info_list_[tar_v_limit_index].tar_a;
          result_of_planning_.tar_pos.x = tar_v_info_list_[tar_v_limit_index].tar_pos.point.x();
          result_of_planning_.tar_pos.y = tar_v_info_list_[tar_v_limit_index].tar_pos.point.y();
          result_of_planning_.tar_pos.heading = tar_v_info_list_[tar_v_limit_index].tar_pos.heading;
          result_of_planning_.tar_pos.s = tar_v_info_list_[tar_v_limit_index].tar_s;

          result_of_planning_.tar_obj.valid = tar_v_info_list_[tar_v_limit_index].obstacle_info.valid;
          result_of_planning_.tar_obj.dist_to_obj = tar_v_info_list_[tar_v_limit_index].obstacle_info.dist_to_obj;
          result_of_planning_.tar_obj.time_gap = tar_v_info_list_[tar_v_limit_index].obstacle_info.time_gap;
          result_of_planning_.tar_obj.relative_v = tar_v_info_list_[tar_v_limit_index].obstacle_info.relative_v;
          result_of_planning_.tar_obj.ttc = tar_v_info_list_[tar_v_limit_index].obstacle_info.ttc;
          result_of_planning_.tar_obj.obj_v = tar_v_info_list_[tar_v_limit_index].obstacle_info.obj_v;
          result_of_planning_.tar_obj.dist_gap_level = tar_v_info_list_[tar_v_limit_index].obstacle_info.dist_gap_level;
          result_of_planning_.tar_obj.rel_spd_level = tar_v_info_list_[tar_v_limit_index].obstacle_info.rel_spd_level;

        }
      } else {
        result_of_planning_.tar_type = tar_v_info_list_[tar_v_limit_index].tar_type;
        result_of_planning_.tar_v = tar_v_info_list_[tar_v_limit_index].tar_v;
        result_of_planning_.tar_a = tar_v_info_list_[tar_v_limit_index].tar_a;
        result_of_planning_.tar_pos.x = tar_v_info_list_[tar_v_limit_index].tar_pos.point.x();
        result_of_planning_.tar_pos.y = tar_v_info_list_[tar_v_limit_index].tar_pos.point.y();
        result_of_planning_.tar_pos.heading = tar_v_info_list_[tar_v_limit_index].tar_pos.heading;
        result_of_planning_.tar_pos.s = tar_v_info_list_[tar_v_limit_index].tar_s;

        result_of_planning_.tar_obj.valid = tar_v_info_list_[tar_v_limit_index].obstacle_info.valid;
        result_of_planning_.tar_obj.dist_to_obj = tar_v_info_list_[tar_v_limit_index].obstacle_info.dist_to_obj;
        result_of_planning_.tar_obj.time_gap = tar_v_info_list_[tar_v_limit_index].obstacle_info.time_gap;
        result_of_planning_.tar_obj.relative_v = tar_v_info_list_[tar_v_limit_index].obstacle_info.relative_v;
        result_of_planning_.tar_obj.ttc = tar_v_info_list_[tar_v_limit_index].obstacle_info.ttc;
        result_of_planning_.tar_obj.obj_v = tar_v_info_list_[tar_v_limit_index].obstacle_info.obj_v;
        result_of_planning_.tar_obj.dist_gap_level = tar_v_info_list_[tar_v_limit_index].obstacle_info.dist_gap_level;
        result_of_planning_.tar_obj.rel_spd_level = tar_v_info_list_[tar_v_limit_index].obstacle_info.rel_spd_level;
      }
    } else {
    }

    if (tar_v_info_list_[tar_a_limit_index].tar_a < result_of_planning_.tar_a) { //如果列表中规划的限制减速度 < 巡航规划的减速度
      result_of_planning_.tar_a = tar_v_info_list_[tar_a_limit_index].tar_a;
    }
  }
#else
    if (tar_v_info_list_size > 0) { 
      // 以前是前车车速>自车车速 由 巡航不会进入 跟车状态
      // 但是稳定跟车后依然速度会被带起
      // 现在的逻辑 : 目标速度列表 ： 只有任务控制车速 、交通灯控制车速 、限速牌控制车速
      // 障碍物单独拿出来
      if (tar_v_info_list_[tar_v_limit_index].tar_v < result_of_planning_.tar_v + 0.5) {
        result_of_planning_.tar_type = tar_v_info_list_[tar_v_limit_index].tar_type;
        result_of_planning_.tar_v = tar_v_info_list_[tar_v_limit_index].tar_v;
        result_of_planning_.tar_a = tar_v_info_list_[tar_v_limit_index].tar_a;
        result_of_planning_.tar_pos.x = tar_v_info_list_[tar_v_limit_index].tar_pos.point.x();
        result_of_planning_.tar_pos.y = tar_v_info_list_[tar_v_limit_index].tar_pos.point.y();
        result_of_planning_.tar_pos.heading = tar_v_info_list_[tar_v_limit_index].tar_pos.heading;
        result_of_planning_.tar_pos.s = tar_v_info_list_[tar_v_limit_index].tar_s;

        result_of_planning_.tar_obj.valid = tar_v_info_list_[tar_v_limit_index].obstacle_info.valid;
        result_of_planning_.tar_obj.dist_to_obj = tar_v_info_list_[tar_v_limit_index].obstacle_info.dist_to_obj;
        result_of_planning_.tar_obj.time_gap = tar_v_info_list_[tar_v_limit_index].obstacle_info.time_gap;
        result_of_planning_.tar_obj.relative_v = tar_v_info_list_[tar_v_limit_index].obstacle_info.relative_v;
        result_of_planning_.tar_obj.ttc = tar_v_info_list_[tar_v_limit_index].obstacle_info.ttc;
        result_of_planning_.tar_obj.obj_v = tar_v_info_list_[tar_v_limit_index].obstacle_info.obj_v;
        result_of_planning_.tar_obj.dist_gap_level = tar_v_info_list_[tar_v_limit_index].obstacle_info.dist_gap_level;
        result_of_planning_.tar_obj.rel_spd_level = tar_v_info_list_[tar_v_limit_index].obstacle_info.rel_spd_level;
      } 

      if (tar_v_info_list_[tar_a_limit_index].tar_a < result_of_planning_.tar_a) { 
        // 如果列表中规划的限制减速度 < 巡航规划的减速度
        result_of_planning_.tar_a = tar_v_info_list_[tar_a_limit_index].tar_a;
      }
  } 

  // 障碍物单独拿出来 单独基于障碍物的信息 进行 相关判断 
  // 防止在稳定跟车状态下，前车加速，速度会被带起
  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_obj_info.tar_type) {
    if (tar_v_obj_info.obstacle_info.obj_v > settings_.tar_v && 
        tar_v_obj_info.obstacle_info.dist_to_obj > curr_position_.v * 0.5) {
        /*  
                    前车车速
        ^
        |             *      
        |---------*------------    巡航车速 settings_.tar_v
        |   *
        |*
        |
        |___________________>

        1、目标车车速 > 自车车速 
           可能考虑不完全！！！ 筛选的目标车速在一定的速度范围波动
        2、目标车距离 > 自车车速 * 0.5 时距 
        3、前车处于加速
        */
       std::cout << "前车车速 > 巡航车速" << std::endl;
    } else {
      if (tar_v_obj_info.tar_v < result_of_planning_.tar_v + 0.5
          && tar_v_obj_info.tar_v < settings_.tar_v + 0.2) {
        // 由于多项式是的凸起 ，又要保证状态的稳定！！！
        // 暂时 计算出的目标车速 大于 巡航车速就不考虑
        // 由于多项式的凸起，会提前退出跟车状态 
        // 即 前车 77km/h , 设定80km/h 时候 会提前退出
        std::cout << "***********************************************************" << std::endl;
        result_of_planning_.tar_type = tar_v_obj_info.tar_type;
        result_of_planning_.tar_v = tar_v_obj_info.tar_v;
        result_of_planning_.tar_a = tar_v_obj_info.tar_a;
        result_of_planning_.tar_pos.x = tar_v_obj_info.tar_pos.point.x();
        result_of_planning_.tar_pos.y = tar_v_obj_info.tar_pos.point.y();
        result_of_planning_.tar_pos.heading = tar_v_obj_info.tar_pos.heading;
        result_of_planning_.tar_pos.s = tar_v_obj_info.tar_s;

        result_of_planning_.tar_obj.valid = tar_v_obj_info.obstacle_info.valid;
        result_of_planning_.tar_obj.dist_to_obj = tar_v_obj_info.obstacle_info.dist_to_obj;
        result_of_planning_.tar_obj.time_gap = tar_v_obj_info.obstacle_info.time_gap;
        result_of_planning_.tar_obj.relative_v = tar_v_obj_info.obstacle_info.relative_v;
        result_of_planning_.tar_obj.ttc = tar_v_obj_info.obstacle_info.ttc;
        result_of_planning_.tar_obj.obj_v = tar_v_obj_info.obstacle_info.obj_v;
        result_of_planning_.tar_obj.dist_gap_level = tar_v_obj_info.obstacle_info.dist_gap_level;
      }
    
      if (tar_v_obj_info.tar_a < result_of_planning_.tar_a) { 
        //如果列表中规划的限制减速度 < 巡航规划的减速度
        result_of_planning_.tar_a = tar_v_obj_info.tar_a;
      }
    }
  }
#endif



  // TODO: D：达到巡航速度，保持 20221021 by xiaranfei
  if (VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS == result_of_planning_.tar_type) {
    // 巡航减速过猛，考虑绝对值
    if (common::com_abs(result_of_planning_.tar_v - settings_.tar_v) < 5 / 3.6F) {
      // 在 上坡的时候 坡度值 可能 需要根据 当前 坡度的值 和 当前的 车速 进行重新规划 5/3.6 还是大了
      // TODO: D1: 不同的档位，对应不同的const_ax
      int gearnum = (int)chassis_status_.gear_number;
      float const_ax = 0.5;
      std::cout << "当前档位 : " << gearnum << std::endl;
      LOG_INFO(5)<<"当前档位 : " << gearnum;

#if TM_YD
      if (gearnum <= 10) {
        const_ax = 0.3;
      } else if (12 == gearnum) {
        std::cout << " 当前档位 12档啦!!! "<< std::endl;
        const_ax = 0.15;
      } else if (10 < gearnum <= 11) {
        const_ax = 0.25;
      } else {
        const_ax = 0.5;
      }
#endif

#if TM_ZF
    if (gearnum <= 10) {
      const_ax = 0.25;
    } else if (12 == gearnum) {
      std::cout << " 当前档位 12档啦!!! "<< std::endl;
      const_ax = 0.13;
    } else if (10 < gearnum <= 11) {
      const_ax = 0.2;
    }else {
      const_ax = 0.5;
    }
#endif

      if (slope_flag_) {
        std::cout << " 有坡度！！！ "<< std::endl;
        //如果当前坡度＜2% 且 油门开度量＞80%
        if (curr_slope_ <= 2.0 &&  control_acc_pedal_ >= 80) {
          const_ax = const_ax - curr_slope_ * 0.1;
        }
      }

      if (const_ax < 0) {
        const_ax = 0.0;
      }

      
      if (curr_position_.v  < settings_.tar_v - 3 / 3.6){ //上坡阶段
        result_of_planning_.tar_v = curr_position_.v + const_ax * 0.1;
        result_of_planning_.tar_a = const_ax * 0.5;
      } else { //下坡同以前
        result_of_planning_.tar_v = settings_.tar_v;
        result_of_planning_.tar_a = 0.0;
      }
    }
  }else if (VELOCITY_PLANNING_TARGET_TYPE_TUNNEL == result_of_planning_.tar_type) {
     if (common::com_abs(result_of_planning_.tar_v - param_.tunnel_velocity_limit) < 5 / 3.6F) {
       result_of_planning_.tar_v = param_.tunnel_velocity_limit;
       result_of_planning_.tar_a = 0.0;
     }
   } else if (VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT == result_of_planning_.tar_type) {
     if (common::com_abs(result_of_planning_.tar_v - lane_speed_limit) < 5 / 3.6F) {
       result_of_planning_.tar_v = lane_speed_limit;
       result_of_planning_.tar_a = 0.0;
     }
   } 

// TODO: D2：多项式的跟停/静止obs避障，采用3个周期和obj_v做冗余判断。
#if (ENABLE_ACC_STOP_Judge)
  if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == result_of_planning_.tar_type ||
      VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == result_of_planning_.tar_type ||
      VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == result_of_planning_.tar_type) {
    if (result_of_planning_.tar_v < 0.8 && prev_tar_v_ < 0.8 && result_of_planning_.tar_obj.obj_v < 0.8) {
      if (stop_count_ > 3.0) {
      // 防止过于平滑，导致最终规划速度无法为0；强制 tar_a=-2，tar_v=0 by 20220922
        result_of_planning_.tar_a = -0.4;//最后一脚 V3.04
        result_of_planning_.tar_v = 0.0;
      } else {
        // nothing;
      }
      ++stop_count_;
    } else {
      stop_count_ = 0.0;
    }
  } else {
    // nothing;
  }
#endif


  
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[planning][velocity][Nosmooth] tar_V = " << result_of_planning_.tar_v * 3.6
            << "km/h, tar_a = " << result_of_planning_.tar_a << "m/s^2" << std::endl;
#endif  

  if (result_of_action_planning_.enable_fallback) {
    CalcVelocityToFallback(result_of_action_planning_, curr_position_.v, &result_of_planning_);
  } else{
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "NO fallback !!! " << std::endl;
#endif  
    first_B_II_ = true;
    first_C_II_ = true;
  }
  
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "first_B_II_ = " << first_B_II_  << 
    ", first_C_II_ = " << first_C_II_ << std::endl;
#endif 
  
  // TODO: D3：起步计数 start_count_
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
  
  // TODO: D4：valid_target_velocity_array_
  if ((!pre_adas_mode_) && (adas_mode_)) {
    valid_target_velocity_array_ = false;
  } else if ((!pre_adas_mode_) && (!adas_mode_)) {
    valid_target_velocity_array_ = false;
  } else {
    valid_target_velocity_array_ = true;
  }

  // 无效的目标速度向量时，用当前速度初始化array_target_velocity_
  if (!valid_target_velocity_array_) {
    array_target_velocity_[0] = curr_position_.v;
    array_target_velocity_[1] = curr_position_.v;
    array_target_velocity_[2] = curr_position_.v;
    array_target_acceleration_[0] = curr_position_.a;
    array_target_acceleration_[1] = curr_position_.a;
    array_target_acceleration_[2] = curr_position_.a;
  }

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
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[planning][velocity][Smooth] tar_V = " << result_of_planning_.tar_v * 3.6
              << "km/h, tar_a = " << result_of_planning_.tar_a << "m/s^2" << std::endl;
#endif    
  }

  if (result_of_planning_.tar_v > param_.max_velocity_limit) {
    result_of_planning_.tar_v = param_.max_velocity_limit;
  } else if (result_of_planning_.tar_v < 0.0F) {
    result_of_planning_.tar_v = 0.0F;
  } else {
    // nothing to do
  }

  if (result_of_planning_.tar_a > param_.max_acceleration_limit) {
    result_of_planning_.tar_a = param_.max_acceleration_limit;
  } else if (result_of_planning_.tar_a < param_.max_deceleration_limit) {
    result_of_planning_.tar_a = param_.max_deceleration_limit;
  } else {
    // nothing to do
  }

  // 目标速度/加速度采样3个周期组成目标速度/加速度向量
  valid_target_velocity_array_ = true;
  array_target_velocity_[2] = array_target_velocity_[1];
  array_target_velocity_[1] = array_target_velocity_[0];
  array_target_velocity_[0] = result_of_planning_.tar_v;
  array_target_acceleration_[2] = array_target_acceleration_[1];
  array_target_acceleration_[1] = array_target_acceleration_[0];
  array_target_acceleration_[0] = result_of_planning_.tar_a;

  // 放在smooth后
  prev_tar_v_ = result_of_planning_.tar_v;
  prev_tar_a_ = result_of_planning_.tar_a;
  pre_adas_mode_ = adas_mode_;

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

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[planning][velocity] Final : tar_type= " << result_of_planning_.tar_type
            << " tar_v = " << result_of_planning_.tar_v * 3.6F << "km/h, tar_a=" << result_of_planning_.tar_a
            << "m/s^2 "
            << " EGO v = " << curr_position_.v * 3.6F << "km/h, Chassis V = " << chassis_status_.v * 3.6
            << " km/h, EGO a = " << curr_position_.a << "m/s^2 "
            << "Chassis a = " << chassis_status_.a << "m/s^2, MIO.x = " << result_of_planning_.tar_obj.dist_to_obj
            << "m, MIO.rel_v = " << result_of_planning_.tar_obj.relative_v * 3.6
            << " km/h, MIO.abs_v = " << result_of_planning_.tar_obj.obj_v * 3.6 << "km/h" << std::endl;
  std::cout << "[planning][velocity] <<<<<<<<< Velocity planning (Fuzzy PID) #########" << std::endl;
#endif 
  return true;
}

// 获取事件报告信息，输出给HMI （显示速度规划的HMI指令）
Int32_t VelocityPlanningFuzzyPID::GetEventReporting(Int32_t num, ad_msg::EventReporting *const events) const {
  if (num > event_reporting_list_.Size()) {
    num = event_reporting_list_.Size();
  }

  Int32_t idx = 0;
  for (idx = 0; idx < num; ++idx) {
    events[idx] = event_reporting_list_[idx];
  }

  return (idx);
}

// 根据轨迹曲率，规划速度
/*
 * @brief 根据轨迹曲率，规划速度
 * @param[in] path， curvature_info， proj_on_path，range_start_s，range_end_s
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToPathCurvature(
    const common::Path &path,
    const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum> &curvature_info,
    const common::PathPoint &proj_on_path, plan_var_t range_start_s, plan_var_t range_end_s,
    TargetVelocityInfo *result) {

  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;
  param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
  param_of_calc_tar_v.stop_accurately = false;
  param_of_calc_tar_v.min_deceleration_time = 2.0F;

  result->Clear();
  Int32_t curve_seg_num = curvature_info.Size();
  for (Int32_t i = 0; i < curve_seg_num; ++i) {
    const common::Path::CurveSegment &curve_seg = curvature_info[i];

    if (common::Path::TYPE_STRAIGHT == curve_seg.type) {//直路
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
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "curvature < 0.001 = " << abs_curvature << std::endl;
#endif
      continue;
    }

  plan_var_t ratio = 0;
  Int32_t lower = common::LerpInOrderedTable(param_.curvatrue_limit_velocity_table, 1 / abs_curvature, &ratio);
  plan_var_t velocity_limit = param_.curvatrue_limit_velocity_table[lower + 1].value;
  // plan_var_t velocity_limit = common::com_sqrt(settings_.lateral_acceleration_limit / abs_curvature);//向心加速度推导公式
  plan_var_t s_to_obj = curve_seg.start_s - proj_on_path.s;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[PlanVelocityAccordingToPathCurvature]velocity_limit = " << velocity_limit * 3.6
              << "km/h, abs_curvature = " << abs_curvature << " ,  R = " << 1 / abs_curvature << "m"
              << ",  s_to_obj = " << s_to_obj << "m"
              << ", ego v = " << curr_position_.v * 3.6 << "km/h" << std::endl;
#endif 
    plan_var_t tar_v = 0;
    plan_var_t tar_a = 0;
    Int32_t dist_gap_level = -1;
    Int32_t rel_spd_level = -1;

#if 0
    bool valid_tar_v = CalcTarVelocityLinearly(param_of_calc_tar_v, curr_position_.v, velocity_limit, s_to_obj, 0.0F,
                                               &tar_v, &tar_a, &dist_gap_level, &rel_spd_level);
#else
    bool valid_tar_v = CalcTarVelocityQuinticPolynomialCurve1d(param_of_calc_tar_v, curr_position_.v, velocity_limit,
                                                               150, 0.0F, true, &tar_v, &tar_a);
#endif

    if (valid_tar_v) {
      if (tar_a > 1.0F) {
        tar_a = 1.0F;
      } else if (tar_a < -2.0F) {
        tar_a = -2.0F;
      }

      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == result->tar_type) {
        result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_CURVATURE;
        result->tar_v = tar_v;
        result->tar_a = tar_a;
        result->tar_s = s_to_obj;
        if (result->tar_s < 0.0F) {
          result->tar_s = 0.0F;
        }
        path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
      } else {
        if (tar_v < result->tar_v) {
          result->tar_v = tar_v;
          result->tar_s = s_to_obj;
          if (result->tar_s < 0.0F) {
            result->tar_s = 0.0F;
          }
          path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
        }
        if (tar_a < result->tar_a) {
          result->tar_a = tar_a;
        }
      }
    }
  }
}



// 根据障碍物分类及结果，速度规划
/* k001 pengc 2022-04-13 (begin) */
// 优化变道时减速的问题
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToObstacles(TargetVelocityInfo *result) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "\n###### PlanVelocityAccordingToObstacles (Begin) ######" << std::endl;
#endif

  status_.exist_uncertain_obstacle = false;
  result->Clear();

  bool req_aeb_action = false;
  bool req_aeb_warning = false;
  bool req_notify_uncertain = false;
  TargetVelocityInfo tar_v_info;
  if (!CalcVelocityConsideringObstacles(target_trajectory_, veh_proj_on_target_trajectory_, &tar_v_info,
                                        &req_aeb_action, &req_aeb_warning, &req_notify_uncertain)) {
    LOG_ERR << "Failed to calculate target speed considering obstacles.";
    obj_jitter_suppression_.Clear();
    return;
  }

  // 踩油门踏板退出AEB模式
  if ((chassis_status_.acc_pedal_value > 5) || (ad_msg::VEH_DRIVING_MODE_MANUAL == chassis_status_.driving_mode) ||
      (ad_msg::VEH_DRIVING_MODE_INVALID == chassis_status_.driving_mode)) {
    req_aeb_action = false;
    status_.aeb_hold_flag = false;
    action_smoothing_aeb_action_.Clear();
  }
  
  //AEB执行之前有个hold状态（aeb_hold_flag）：障碍物持续5帧以下就消失-3m/s2减速度，否则-5m/s2减速度
  if (req_aeb_action) {
    req_aeb_action = action_smoothing_aeb_action_.Smooth(true);//Smooth
    if (req_aeb_action) {
      status_.aeb_hold_flag = true;
    }
  } else {
    action_smoothing_aeb_action_.Smooth(false);
  }
  //AEB警告
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

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == tar_v_info.tar_type) {
    *result = tar_v_info;
    return;
  }

  Int32_t changed_trajectory_size = changed_trajectory_.points().Size();
#if ENABLE_VELOCITY_PLANNING_LOG
  printf(
      "### trj_changing.is_changing=%d"
      ", changed_trajectory_size=%d"
      ", obstacle_info.valid=%d"
      ", obstacle_info.ttc=%0.1f"
      ", obstacle_info.time_gap=%0.1f"
      "\n",
      result_of_trajectory_planning_.trj_changing.is_changing, changed_trajectory_size, tar_v_info.obstacle_info.valid,
      tar_v_info.obstacle_info.ttc, tar_v_info.obstacle_info.time_gap);
#endif

  TargetVelocityInfo corrected_tar_v_info;
  if ((result_of_trajectory_planning_.trj_changing.is_changing) && (changed_trajectory_size > 1) &&//
      (!status_.aeb_hold_flag) && (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION != tar_v_info.tar_type) &&
      (tar_v_info.obstacle_info.valid) && (tar_v_info.obstacle_info.ttc > 5.0F) &&
      (tar_v_info.obstacle_info.time_gap > 1.0F)) {
    common::PathPoint proj_on_changed_trajectory;
    changed_trajectory_.FindProjection(curr_position_.path_point.point, &proj_on_changed_trajectory);
#if ENABLE_VELOCITY_PLANNING_LOG
    printf("\n@@@@@@@@ 计算变道的障碍物减速\n");
#endif
    bool req_aeb_action_2 = false;
    bool req_aeb_warning_2 = false;
    bool req_notify_uncertain_2 = false;
    if (CalcVelocityConsideringObstacles(changed_trajectory_, proj_on_changed_trajectory, &corrected_tar_v_info,
                                         &req_aeb_action_2, &req_aeb_warning_2, &req_notify_uncertain_2)) {
      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != corrected_tar_v_info.tar_type) {
#if ENABLE_VELOCITY_PLANNING_LOG
        printf("    tar_v=%0.1f, tar_a=%0.1f, corrected: tar_v=%0.1f, tar_a=%0.1f\n", tar_v_info.tar_v * 3.6F,
               tar_v_info.tar_a, corrected_tar_v_info.tar_v * 3.6f, corrected_tar_v_info.tar_a);
#endif
        if (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION == corrected_tar_v_info.tar_type) {
          printf("不执行变道/避让的AEB\n");
        } else {
          // 取减速度的小值
          tar_v_info.tar_v = common::Min(tar_v_info.tar_v, corrected_tar_v_info.tar_v);
          tar_v_info.tar_a = common::Min(tar_v_info.tar_a, corrected_tar_v_info.tar_a);
          printf("使用小的减速度\n");
        }
      } else {
        // 最优轨迹不会碰撞，不用减速
        tar_v_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
          printf("不用减速\n");
      }
    }
  }

//进AEB_ACTION状态，前提hold
  if (status_.aeb_hold_flag) {
    *result = tar_v_info;
    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
    result->tar_v = 0.0F;
    result->tar_a = common::Min(-4.0F, result->tar_a);
    result->tar_s = 0.0F;

    AddEventReporting(EVENT_TYPE_AEB_ACTION);

    obj_jitter_suppression_.Clear();
  } else {
#if 0
        if (tar_v_info.tar_a > 2.0F)        {
            tar_v_info.tar_a = 2.0F;
        }
        else if (tar_v_info.tar_a < -3.0F)        {
            tar_v_info.tar_a = -3.0F;
        }
#endif

#if 0
    // Suppress the jitter caused by the noise of sensors
    SuppressJitter(obj_jitter_suppression_, tar_v_info, result);
#else
    *result = tar_v_info;
#endif
  }

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "     规划结果 (障碍限速): ";
  switch (result->tar_type) {
    case (VELOCITY_PLANNING_TARGET_TYPE_INVALID):  
      std::cout << "无请求, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE): 
      std::cout << "根据障碍限速, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING):
      std::cout << "跟随, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING):
      std::cout << "低速跟随, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN):
      std::cout << "AEB_WARN ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION):
      std::cout << "AEB_ACTION ";
      break;
    default:
      std::cout << "未知请求, ";
      break;
  }
  std::cout << "tar_v=" << result->tar_v * 3.6F << "km/h, tar_a=" << result->tar_a << "m/s^2" << std::endl;
#endif

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "\n###### PlanVelocityAccordingToObstacles (End) ######" << std::endl;
#endif
}

// 根据障碍物，计算速度 
/*
 * @brief 根据障碍物，计算速度
 * @param[in] path， proj_on_path ？
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 *          req_aeb_action     req_aeb_warning    req_notify_uncertain
 */
bool VelocityPlanningFuzzyPID::CalcVelocityConsideringObstacles(const common::Path &path,
                                                                const common::PathPoint &proj_on_path,
                                                                TargetVelocityInfo *result, bool *req_aeb_action,
                                                                bool *req_aeb_warning, bool *req_notify_uncertain) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "\n###### CalcVelocityConsideringObstacles (Begin) ######" << std::endl;
#endif

  /// status_.exist_uncertain_obstacle = false;

  // parameter of calculating target velocity
  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;
  param_of_calc_tar_v.min_dist_of_ctl_range = 30.0F;
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
  param_of_calc_tar_v.stop_accurately = false;
  param_of_calc_tar_v.min_deceleration_time = 1.0F;

  const plan_var_t sample_step_len = param_.sample_step_len_for_collision_test;
  // 预描前视距离forward distance
  const plan_var_t min_forward_path_len = common::Max(7.0F * curr_position_.v, 30.0F);
  // 根据规划的轨迹，速度在轨迹上采样，末端点8.0F * curr_position_.v与障碍物包围盒碰撞 /8.0s再改回以前的6.0s

  plan_var_t cur_v_for_plan = curr_position_.v;
  if (cur_v_for_plan < 5.0F / 3.6F) {
    cur_v_for_plan = 5.0F / 3.6F;
  }

  result->Clear();
  *req_aeb_action = false;
  *req_aeb_warning = false;
  *req_notify_uncertain = false;

  plan_var_t forward_path_len = path.total_length() - proj_on_path.s;
  if (forward_path_len < min_forward_path_len) {
    forward_path_len = min_forward_path_len;
  }
  Int32_t sample_num = common::com_round(forward_path_len / sample_step_len) + 1;
  internal_data_.sample_points.Clear();
  if (!path.UniformlySamplePathForward(proj_on_path.s, sample_num, sample_step_len, &internal_data_.sample_points)) {
    LOG_ERR << "Failed to sample path for collision testing.";
    /// obj_jitter_suppression_.Clear();
    return false;
  }
  Int32_t sample_size = internal_data_.sample_points.Size();
  if (sample_size < 1) {
    LOG_ERR << "Failed to sample path for collision testing.";
    /// obj_jitter_suppression_.Clear();
    return false;
  }

  TargetVelocityInfo tar_v_info_obj;
  TargetVelocityInfo tar_v_info_flw;
  TargetVelocityInfo tar_v_info_uncertain;

  Int32_t tar_v_info_flw_obj_idx = -1;
  Int32_t tar_v_info_obj_idx = -1;


  bool UpdateVelocity_obs = false;

  driv_map::CollisionTestOnPathObj test_obj;
  test_obj.t_offset = 0.0F;
  test_obj.s_offset = 0.0F;
  test_obj.x_offset = param_.dist_of_localization_to_center;
  test_obj.obj_half_length = 0.5F * param_.vehicle_length;
  test_obj.obj_half_width = 0.5F * param_.vehicle_width;
  test_obj.obj_v = cur_v_for_plan;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    test_obj.is_backing = true;
  } else {
    test_obj.is_backing = false;
  }

  driv_map::CollisionTestOnPathResult &collison_test_ret = internal_data_.collision_test_on_path_result; //实例化collison_test_ret，其类型为driv_map::CollisionTestOnPathResult
  Int32_t risk_value =
      driving_map_->TestCollisionOnPath(test_obj, path, internal_data_.sample_points, &collison_test_ret);

  Int32_t risky_obj_num = collison_test_ret.risky_obj_list.Size();
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "\n  有风险的障碍物, num=" << risky_obj_num << std::endl;
#endif
  /* k001 pengc 2022-04-27 (begin) */
  // 多目标跟随时，过滤掉跟随目标前方的障碍物
  Int32_t tar_following_obj_idx = -1;
  for (Int32_t i = 0; i < risky_obj_num; ++i) {
    const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info = collison_test_ret.risky_obj_list[i];
    const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
    const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);

    if (test_ret.static_distance > 2.0F) {
      continue;
    }
    if (tar_obj.confidence < 80) {
      continue;
    }
    /*
    if (ad_msg::OBJ_PRCP_TYPE_FUSED != tar_obj.perception_type){
        // 只跟随融合的障碍物
        continue;
    }
    if ((ad_msg::OBJ_TYPE_PASSENGER_VEHICLE != tar_obj.type) &&
        (ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE != tar_obj.type) &&
        (ad_msg::OBJ_TYPE_SPECIAL_VEHICLE != tar_obj.type) && (ad_msg::OBJ_TYPE_OTHER_VEHICLE != tar_obj.type) &&
        (ad_msg::OBJ_TYPE_BICYCLE != tar_obj.type) && (ad_msg::OBJ_TYPE_PEDESTRIAN != tar_obj.type)){
        // 只跟随车辆
        continue;
    }
    if (!tar_obj.dynamic){
        continue;
    }*/
    if (driv_map::OBJ_POSITION_FRONT != rsk_obj_info.obj_position) {
      continue;
    }
    if (driv_map::OBJ_DIRECTION_FORWARD != rsk_obj_info.obj_direction) {
      continue;
    }

    if (tar_following_obj_idx < 0) {
      tar_following_obj_idx = i;
    } else {
      const driv_map::CollisionTestOnPathResult::ObjInfo &tar_following_obj_info =
          collison_test_ret.risky_obj_list[tar_following_obj_idx];
      if (rsk_obj_info.dist_to_tar_obj < tar_following_obj_info.dist_to_tar_obj) {
        tar_following_obj_idx = i;
      }
    }
  }
  /* k001 pengc 2022-04-27 (end) */
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << " tar_following_obj_index = " << tar_following_obj_idx << std::endl;
#endif
  

  for (Int32_t i = 0; i < risky_obj_num; ++i) {
    const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info = collison_test_ret.risky_obj_list[i];
    const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
    const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " index=" << i << " ID=" << tar_obj.id <<"risk obstacle collison static distance = " 
              << test_ret.static_distance << std::endl;
#endif
    if (test_ret.static_distance > 2.0) {
      continue;
    }

    /* k001 pengc 2022-04-27 (begin) */
    /// 多目标跟随时，过滤掉跟随目标前方的障碍物
    const driv_map::CollisionTestOnPathResult::ObjInfo *tar_following_obj = Nullptr_t;
    if (tar_following_obj_idx >= 0) {
      tar_following_obj = &(collison_test_ret.risky_obj_list[tar_following_obj_idx]);
    }
    /// TODO:2、calc_result来源于CalcTarVelocityLinearly计算的结果
    ClassifyTargetByObstacleResult calc_result; //diff
    bool need_ctl = ClassifyTargetByObstacle(param_of_calc_tar_v, path, proj_on_path, false, cur_v_for_plan,
                                             rsk_obj_info, &calc_result, tar_following_obj);//diff
    // need_ctl 表示是否障碍物限速
    if (!need_ctl) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE      
      std::cout << "不需受控" << std::endl;
#endif      
      continue;
    }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE    
    std::cout << "受控" << std::endl;
#endif    
    /* k001 pengc 2022-04-27 (end) */

    if (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION == calc_result.tar_velocity_info.tar_type) {
      *req_aeb_action = true;
    }
    if (calc_result.aeb_warning) {
      *req_aeb_warning = true;
    }
    if (calc_result.is_uncertain) {
      UpdateTargetVelocityInfo(calc_result.tar_velocity_info, &tar_v_info_uncertain);
    }
    /// TODO:3、根据tar_type在UpdateTargetVelocityInfo中取小，更新 tar_v_info_obj 和 tar_v_info_flw
    if (calc_result.req_dec) {
      switch (calc_result.tar_velocity_info.tar_type) {
        case (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE):
        case (VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN):
        case (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION):
          UpdateVelocity_obs = UpdateTargetVelocityInfo(calc_result.tar_velocity_info, &tar_v_info_obj);
          if (UpdateVelocity_obs){
              tar_v_info_obj_idx = i;
          }
          break;

        case (VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING):
        case (VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING):
          if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == tar_v_info_flw.tar_type) {
            tar_v_info_flw = calc_result.tar_velocity_info;
            tar_v_info_flw_obj_idx = i;     //   
          } else {
            if (calc_result.tar_velocity_info.obstacle_info.dist_to_obj < tar_v_info_flw.obstacle_info.dist_to_obj) {
              if (UpdateTargetVelocityInfo(tar_v_info_flw, &tar_v_info_obj)) {
                tar_v_info_obj.tar_type = VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE;
                tar_v_info_obj_idx = i;
              }
              tar_v_info_flw = calc_result.tar_velocity_info;
              tar_v_info_flw_obj_idx = i;             
            } else {
              if (UpdateTargetVelocityInfo(calc_result.tar_velocity_info, &tar_v_info_obj)) {
                tar_v_info_obj.tar_type = VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE;
                tar_v_info_obj_idx = i;
              }
            }
          }
          break;
        default:
          LOG_ERR << "Invalid velocity planning target type.";
          break;
      }
    }
  } 

  /// TODO:4、障碍物减速/跟车/低速跟车：多项式

  if (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE == tar_v_info_obj.tar_type) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout
        << " =========== OBSTACLE========CalcTarVelocityQuinticPolynomialCurve1d Start======== "<< std::endl;
#endif
    ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
    param_of_calc_tar_v.time_of_ctl_range = 4.0F;
    param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
    param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
    param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
    param_of_calc_tar_v.stop_accurately = false;
    param_of_calc_tar_v.min_deceleration_time = 2.0F;

    if (tar_v_info_obj_idx >= 0) {
      const ad_msg::Obstacle &tar_obj_obstacle = driving_map_->GetObstacle(collison_test_ret.risky_obj_list[tar_v_info_obj_idx].test_ret.obj_list_index);
      const driv_map::CollisionTestResult::ObjInfo &obj_obstacle = collison_test_ret.risky_obj_list[tar_v_info_obj_idx].test_ret;
      const ad_msg::Obstacle &tar_obj_obs = driving_map_->GetObstacle(obj_obstacle.obj_list_index);
      tar_v_info_obj.obstacle_info.dist_gap_level = tar_obj_obs.id; // 障碍物ID，借用dist_gap_level
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
      std::cout <<"id = " << tar_v_info_obj.obstacle_info.dist_gap_level <<"Obstancle X = " << tar_obj_obstacle.x << "m, Obstancle Y = " << tar_obj_obstacle.y << "m, Obstancle l =" << tar_obj_obstacle.proj_on_major_ref_line.l <<
      "m, Obstancle S =" << tar_obj_obstacle.proj_on_major_ref_line.s << "m, Obstancle 包围盒距离 = " << obj_obstacle.static_distance 
      << "m, 障碍物的宽度 = " << tar_obj_obstacle.obb.half_width * 2 << "m, 障碍物的长度 = "<< tar_obj_obstacle.obb.half_length * 2 <<"m, obs_obj_heading = "
      << tar_obj_obstacle.obb.heading << "rad "<< std::endl;
#endif
    }
    CalcTarVelocityQuinticPolynomialCurve1d(
        param_of_calc_tar_v, curr_position_.v, tar_v_info_obj.obstacle_info.velocity_limit,
        tar_v_info_obj.obstacle_info.dist_to_obj, tar_v_info_obj.obstacle_info.safe_dist, false,
        &(tar_v_info_obj.tar_v), &(tar_v_info_obj.tar_a));
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " ====== OBSTACLE========CalcTarVelocityQuinticPolynomialCurve1d END=========="<< std::endl;
#endif
  } 

  TargetVelocityInfo *tar_v_info = &tar_v_info_obj;

  if (VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING == tar_v_info_flw.tar_type) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " ======= Follow ====CalcTarVelocityQuinticPolynomialCurve1d Begin========"<< std::endl;
#endif    
    ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
    param_of_calc_tar_v.time_of_ctl_range = 4.0F;
    param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
    param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
    param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
    param_of_calc_tar_v.stop_accurately = false;
    param_of_calc_tar_v.min_deceleration_time = 2.0F;

    if (tar_v_info_flw_obj_idx >= 0) {
      const ad_msg::Obstacle &tar_obj_flw = driving_map_->GetObstacle(collison_test_ret.risky_obj_list[tar_v_info_flw_obj_idx].test_ret.obj_list_index);
      const driv_map::CollisionTestResult::ObjInfo & obj_follow = collison_test_ret.risky_obj_list[tar_v_info_flw_obj_idx].test_ret;
      const ad_msg::Obstacle &tar_obj_f = driving_map_->GetObstacle(obj_follow.obj_list_index);
      tar_v_info_flw.obstacle_info.dist_gap_level = tar_obj_f.id;//障碍物ID，借用dist_gap_level

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE      
      std::cout <<"id = "<< tar_v_info_flw.obstacle_info.dist_gap_level << " Follow X = " << tar_obj_flw.x << "m, Follow Y = " << tar_obj_flw.y << "m, Follow l =" << tar_obj_flw.proj_on_major_ref_line.l <<
      "m, Follow S =" << tar_obj_flw.proj_on_major_ref_line.s << "m, Follow 包围距离 = " << obj_follow.static_distance 
      << "m, 障碍物的宽度 = " << tar_obj_flw.obb.half_width * 2 << "m, 障碍物的长度 = "<< tar_obj_flw.obb.half_length * 2 <<"m, follow_obj_heading = "
      << tar_obj_flw.obb.heading << "rad "<< std::endl;
#endif
    } 
    
    CalcTarVelocityQuinticPolynomialCurve1d(
        param_of_calc_tar_v, curr_position_.v, tar_v_info_flw.obstacle_info.velocity_limit,
        tar_v_info_flw.obstacle_info.dist_to_obj, tar_v_info_flw.obstacle_info.safe_dist, false,
        &(tar_v_info_flw.tar_v), &(tar_v_info_flw.tar_a));
    int32_t need_ctl = true;  //

    // bool need_ctl = CalcTarVelocityForFollowing(
    //     curr_position_.v, tar_v_info_flw.obstacle_info.velocity_limit, tar_v_info_flw.obstacle_info.dist_to_obj,
    //     tar_v_info_flw.obstacle_info.safe_dist, &(tar_v_info_flw.tar_v), &(tar_v_info_flw.tar_a),
    //     &(tar_v_info_flw.obstacle_info.dist_gap_level), &(tar_v_info_flw.obstacle_info.rel_spd_level));

    // std::cout << "\n### result: tar_v_info_flw.tar_v =" << tar_v_info_flw.tar_v  <<"\n### result:
    // tar_v_info_flw.tar_a =" << tar_v_info_flw.tar_a << std::endl;

    if (!need_ctl) {
      tar_v_info_flw.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " ======= Follow ====CalcTarVelocityQuinticPolynomialCurve1d End========"<< std::endl;
#endif
  } else if (VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING == tar_v_info_flw.tar_type) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "====low follow ===CalcTarVelocityQuinticPolynomialCurve1d Begin===="<< std::endl;
#endif
    ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
    param_of_calc_tar_v.time_of_ctl_range = 4.0F;
    param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
    param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
    param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
    param_of_calc_tar_v.stop_accurately = false;
    param_of_calc_tar_v.min_deceleration_time = 2.0F;

    if (tar_v_info_flw_obj_idx >= 0)
    {
      const ad_msg::Obstacle &tar_obj_flw = driving_map_->GetObstacle(collison_test_ret.risky_obj_list[tar_v_info_flw_obj_idx].test_ret.obj_list_index);
      const driv_map::CollisionTestResult::ObjInfo & obj_follow = collison_test_ret.risky_obj_list[tar_v_info_flw_obj_idx].test_ret;
      const ad_msg::Obstacle &tar_obj_f = driving_map_->GetObstacle(obj_follow.obj_list_index);
      tar_v_info_flw.obstacle_info.dist_gap_level = tar_obj_f.id; // 障碍物ID，借用dist_gap_level
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE      
      std::cout <<"id = "<< tar_v_info_flw.obstacle_info.dist_gap_level << "Follow X = " << tar_obj_flw.x << "m, Follow Y = " << tar_obj_flw.y << "m, Follow l =" << tar_obj_flw.proj_on_major_ref_line.l <<
      "m, Follow S =" << tar_obj_flw.proj_on_major_ref_line.s << "m, Follow 包围距离 = " << obj_follow.static_distance 
      << "m, 障碍物的宽度 = " << tar_obj_flw.obb.half_width * 2 << "m, 障碍物的长度 = "<< tar_obj_flw.obb.half_length * 2 <<"m, follow_obj_heading = "
      << tar_obj_flw.obb.heading << "rad "<< std::endl;
#endif
    } 
    
    CalcTarVelocityQuinticPolynomialCurve1d(
        param_of_calc_tar_v, curr_position_.v, tar_v_info_flw.obstacle_info.velocity_limit,
        tar_v_info_flw.obstacle_info.dist_to_obj, tar_v_info_flw.obstacle_info.safe_dist, false,
        &(tar_v_info_flw.tar_v), &(tar_v_info_flw.tar_a));
    int32_t need_ctl = true;  ////////////

    // bool need_ctl = CalcTarVelocityForLowSpeedFollowing(
    //     curr_position_.v, tar_v_info_flw.obstacle_info.velocity_limit, tar_v_info_flw.obstacle_info.dist_to_obj,
    //     tar_v_info_flw.obstacle_info.safe_dist, &(tar_v_info_flw.tar_v), &(tar_v_info_flw.tar_a),
    //     &(tar_v_info_flw.obstacle_info.dist_gap_level), &(tar_v_info_flw.obstacle_info.rel_spd_level));

    if (!need_ctl) {
      tar_v_info_flw.tar_type = VELOCITY_PLANNING_TARGET_TYPE_INVALID;
    }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "====low follow ===CalcTarVelocityQuinticPolynomialCurve1d End===="<< std::endl;
#endif
  }

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == tar_v_info->tar_type) {
    tar_v_info = &tar_v_info_flw;
  } else {  // 同UpdateTargetVelocityInfo
    if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info_flw.tar_type) {
      plan_var_t min_v = common::Min(tar_v_info_flw.tar_v, tar_v_info->tar_v);
      plan_var_t min_a = common::Min(tar_v_info_flw.tar_a, tar_v_info->tar_a);
      if (tar_v_info_flw.tar_v < tar_v_info->tar_v) {
        tar_v_info = &tar_v_info_flw;
      } else if (tar_v_info_flw.tar_a < tar_v_info->tar_a) {
        tar_v_info = &tar_v_info_flw;
      }
      tar_v_info->tar_v = min_v;
      tar_v_info->tar_a = min_a;
    }
  }

  Int32_t uncertain_num = collison_test_ret.uncertain_list.Size();
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "不确定的障碍物, num=" << uncertain_num << std::endl;
#endif
  

  for (Int32_t i = 0; i < uncertain_num; ++i) {
    const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info = collison_test_ret.uncertain_list[i];
    const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
    const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);
    ClassifyTargetByObstacleResult calc_result;//diff
    if (test_ret.static_distance > 0.5) {
      continue;
    }

    bool need_ctl = ClassifyTargetByObstacle(param_of_calc_tar_v, path, proj_on_path, true, cur_v_for_plan,
                                             rsk_obj_info, &calc_result);//diff

    if (!need_ctl) {
      continue;
    }
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
    if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info->tar_type) {
      if ((tar_v_info_uncertain.tar_a_true < tar_v_info->tar_a) &&
          (tar_v_info_uncertain.obstacle_info.dist_to_obj < (tar_v_info->obstacle_info.dist_to_obj + 0.5F))) {
        *req_notify_uncertain = true;
      }
    } else {
      *req_notify_uncertain = true;
    }
  }

  // 踩油门踏板退出AEB模式
  if ((chassis_status_.acc_pedal_value > 5) || (ad_msg::VEH_DRIVING_MODE_MANUAL == chassis_status_.driving_mode) ||
      (ad_msg::VEH_DRIVING_MODE_INVALID == chassis_status_.driving_mode)) {
    *req_aeb_action = false;
  }

  if (status_.aeb_hold_flag) {
    *result = *tar_v_info;
    result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
    result->tar_v = 0.0F;
    result->tar_a = common::Min(-4.0F, result->tar_a);
    result->tar_s = 0.0F;
    result->tar_pos = proj_on_path;
  } else {
    if (tar_v_info->tar_a > 2.0F) {
      tar_v_info->tar_a = 2.0F;
    } else if (tar_v_info->tar_a < -3.0F) {
      tar_v_info->tar_a = -3.0F;
    }
    *result = *tar_v_info;
  }

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "碰撞分析结果: ";
  switch (result->tar_type) {
    case (VELOCITY_PLANNING_TARGET_TYPE_INVALID):
      std::cout << "无请求, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE):
      std::cout << "根据障碍限速, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING):
      std::cout << "跟随, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING):
      std::cout << "低速跟随, ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN):
      std::cout << "AEB_WARN ";
      break;
    case (VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION):
      std::cout << "AEB_ACTION ";
      break;
    default:
      std::cout << "未知请求, ";
      break;
  }
  std::cout << "tar_v=" << result->tar_v * 3.6F << "km/h, tar_a=" << result->tar_a << "m/s^2" << std::endl;
  std::cout << "###### CalcVelocityConsideringObstacles (End) ######" << std::endl;
#endif

  return true;
}

// 根据TrafficLight，规划速度： 区别在于对距离有限制
/*
 * @brief 根据TrafficLight，规划速度： 区别在于对距离有限制
 * @param[in] path， proj_on_path
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToTrafficLight(const common::Path &path,
                                                                   const common::PathPoint &proj_on_path,
                                                                   TargetVelocityInfo *result) {
  // parameter of calculating target velocity
  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;                    //
  param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;               //
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;   //
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;  //
  param_of_calc_tar_v.stop_accurately = param_.stop_accurately.enable;
  param_of_calc_tar_v.min_deceleration_time = 2.0F;

  result->Clear();

  // TODO: 红绿灯颜色判断
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

    // std::cout << "  Get stop_line[(" << map_signal.stop_line.start().x()
    //          << ", " << map_signal.stop_line.start().y()
    //          << "), (" << map_signal.stop_line.end().x()
    //          << ", " << map_signal.stop_line.end().y()
    //          << ")]." << std::endl;

    common::PathPoint cross_point;
    if (path.IsIntersect(map_signal.stop_line.start(), map_signal.stop_line.end(), &cross_point)) {
      if ((cross_point.s - proj_on_path.s) < 0.0) {
        // 后轮已经通过停止线了
        continue;
      }
      // 停车距离限制
      plan_var_t s_to_obj = cross_point.s - proj_on_path.s - param_.dist_of_localization_to_front;
      plan_var_t tar_v = 0;
      plan_var_t tar_a = 0;
      Int32_t dist_gap_level = -1;
      Int32_t rel_spd_level = -1;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "###### PlanVelocityAccordingToTrafficLight (CalcTarVelocityLinearly0) ######" << std::endl;
#endif

      bool valid_tar_v = CalcTarVelocityLinearly(param_of_calc_tar_v, curr_position_.v, 0.0F, s_to_obj, 0.0F, &tar_v,
                                                 &tar_a, &dist_gap_level, &rel_spd_level);
      if (valid_tar_v) {
        if (tar_a > 1.0F) {
          tar_a = 1.0F;
        } else if (tar_a < -3.0F) {
          tar_a = -3.0F;
        }

        if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == result->tar_type) {
          result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_TRAFFIC_LIGHT;
          result->tar_v = tar_v;
          result->tar_a = tar_a;
          result->tar_s = s_to_obj;
          if (result->tar_s < 0.0F) {
            result->tar_s = 0.0F;
          }
          path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
        } else {
          if (tar_v < result->tar_v) {
            result->tar_v = tar_v;
            result->tar_s = s_to_obj;
            if (result->tar_s < 0.0F) {
              result->tar_s = 0.0F;
            }
            path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
          }
          if (tar_a < result->tar_a) {
            result->tar_a = tar_a;
          }
        }
      }
    }
  }
}

// 根据TrafficSignal，规划速度
/*
 * @brief 根据TrafficSignal，规划速度
 * @param[in] path， proj_on_path
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToTrafficSignal(const common::Path &path,
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

// 根据SceneStory，规划速度
/*
 * @brief 根据SceneStory，规划速度
 * @param[in&out] path， proj_on_path
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToSceneStory(const common::Path &path,
                                                                 const common::PathPoint &proj_on_path,
                                                                 TargetVelocityInfo *result) {
  // parameter of calculating target velocity
  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;
  param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
  param_of_calc_tar_v.stop_accurately = param_.stop_accurately.enable;
  param_of_calc_tar_v.min_deceleration_time = 2.0F;

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
    plan_var_t tar_v = 0;
    plan_var_t tar_a = 0;
    Int32_t dist_gap_level = -1;
    Int32_t rel_spd_level = -1;

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "###### PlanVelocityAccordingToTrafficLight (CalcTarVelocityLinearly0) ######" << std::endl;
#endif
    bool valid_tar_v = CalcTarVelocityLinearly(param_of_calc_tar_v, curr_position_.v, velocity_limit, s_to_obj, 0.0F,
                                               &tar_v, &tar_a, &dist_gap_level, &rel_spd_level);
    if (valid_tar_v) {
      if (tar_a > 1.0F) {
        tar_a = 1.0F;
      } else if (tar_a < -3.0F) {
        tar_a = -3.0F;
      }

      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == result->tar_type) {
        result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_SCENE_STORY;
        result->tar_v = tar_v;
        result->tar_a = tar_a;
        result->tar_s = s_to_obj;
        if (result->tar_s < 0.0F) {
          result->tar_s = 0.0F;
        }
        path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
      } else {
        if (tar_v < result->tar_v) {
          result->tar_v = tar_v;
          result->tar_s = s_to_obj;
          if (result->tar_s < 0.0F) {
            result->tar_s = 0.0F;
          }
          path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
        }
        if (tar_a < result->tar_a) {
          result->tar_a = tar_a;
        }
      }
    }
  }
}

// 根据Tunnel，规划速度
/*
 * @brief 根据Tunnel，规划速度
 * @param[in&out] path， proj_on_path
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToTunnel(const common::Path &path,
                                                                 const common::PathPoint &proj_on_path,
                                                                 TargetVelocityInfo *result) {
  // parameter of calculating target velocity
  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;
  param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
  param_of_calc_tar_v.stop_accurately = param_.stop_accurately.enable;
  param_of_calc_tar_v.min_deceleration_time = 2.0F;

  result->Clear();

  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
      driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "story : num = " << storys_num << std::endl;
#endif
  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL != story.type) {
      // 不在隧道场景
      continue;
    }
    if (story.area.DistanceToArea() > 300.0F ) {
      // 距离大于300m
      continue;
    }

    plan_var_t tunnel_velocity_limit = param_.tunnel_velocity_limit;

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "story : type = " << story.type << ", area valid = " << story.area.IsValid() << ", distance = " << story.area.DistanceToArea() 
              << ", velocity_limit = " << tunnel_velocity_limit * 3.6F << std::endl;
#endif

    plan_var_t s_to_obj = story.area.DistanceToArea();
    if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
      s_to_obj = s_to_obj - param_.dist_of_localization_to_rear;
    }
    plan_var_t tar_v = 0;
    plan_var_t tar_a = 0;

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "###### PlanVelocityAccordingToTunnel (CalcTarVelocityQuinticPolynomialCurve1d) ######" << std::endl;
#endif
    bool valid_tar_v = CalcTarVelocityQuinticPolynomialCurve1d(param_of_calc_tar_v, curr_position_.v, tunnel_velocity_limit,
                                                               150.0F, 0.0F, true, &tar_v, &tar_a);
    if (valid_tar_v) {
      if (tar_a > 1.0F) {
        tar_a = 1.0F;
      } else if (tar_a < -3.0F) {
        tar_a = -3.0F;
      }

      AddEventReporting(EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL);
      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == result->tar_type) {
        result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_TUNNEL;
        result->tar_v = tar_v;
        result->tar_a = tar_a;
        result->tar_s = s_to_obj;
        if (result->tar_s < 0.0F) {
          result->tar_s = 0.0F;
        }
        path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
      } else {
        if (tar_v < result->tar_v) {
          result->tar_v = tar_v;
          result->tar_s = s_to_obj;
          if (result->tar_s < 0.0F) {
            result->tar_s = 0.0F;
          }
          path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
        }
        if (tar_a < result->tar_a) {
          result->tar_a = tar_a;
        }
      }
    }
  }
}

// 根据Ramp，规划速度
/*
 * @brief 根据Ramp，规划速度
 * @param[in&out] path， proj_on_path
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::PlanVelocityAccordingToRamp(const common::Path &path,
                                                                 const common::PathPoint &proj_on_path,
                                                                 TargetVelocityInfo *result) {
  // parameter of calculating target velocity
  ParamOfCalcTarVelocityLinearly param_of_calc_tar_v;
  param_of_calc_tar_v.time_of_ctl_range = 4.0F;
  param_of_calc_tar_v.min_dist_of_ctl_range = 20.0F;
  param_of_calc_tar_v.accelaration_of_activity_condition = 0.5F;
  param_of_calc_tar_v.deceleration_of_activity_condition = -0.5F;
  param_of_calc_tar_v.stop_accurately = param_.stop_accurately.enable;
  param_of_calc_tar_v.min_deceleration_time = 2.0F;

  result->Clear();

  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
      driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();
  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP != story.type) {
      // 不在匝道场景
      continue;
    }
    if (story.area.DistanceToArea() > 200.0F ) {
      // 距离大于200m
      continue;
    }

    plan_var_t ramp_velocity_limit = param_.ramp_velocity_limit;  // 匝道限速

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "story : type = " << story.type << ", area valid = " << story.area.IsValid() << ", distance = " << story.area.DistanceToArea() 
              << ", velocity_limit = " << ramp_velocity_limit * 3.6F << std::endl;
#endif

    plan_var_t s_to_obj = story.area.DistanceToArea();
    if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
      s_to_obj = s_to_obj - param_.dist_of_localization_to_rear;
    }
    plan_var_t tar_v = 0;
    plan_var_t tar_a = 0;

#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "###### PlanVelocityAccordingToRamp (CalcTarVelocityQuinticPolynomialCurve1d) ######" << std::endl;
#endif
    bool valid_tar_v = CalcTarVelocityQuinticPolynomialCurve1d(param_of_calc_tar_v, curr_position_.v, ramp_velocity_limit,
                                                               150.0F, 0.0F, true, &tar_v, &tar_a);
    if (valid_tar_v) {
      if (tar_a > 1.0F) {
        tar_a = 1.0F;
      } else if (tar_a < -3.0F) {
        tar_a = -3.0F;
      }

      AddEventReporting(EVENT_TYPE_VELOCITY_PLANNING_ACCORDING_TO_RAMP);
      if (VELOCITY_PLANNING_TARGET_TYPE_INVALID == result->tar_type) {
        result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_RAMP;
        result->tar_v = tar_v;
        result->tar_a = tar_a;
        result->tar_s = s_to_obj;
        if (result->tar_s < 0.0F) {
          result->tar_s = 0.0F;
        }
        path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
      } else {
        if (tar_v < result->tar_v) {
          result->tar_v = tar_v;
          result->tar_s = s_to_obj;
          if (result->tar_s < 0.0F) {
            result->tar_s = 0.0F;
          }
          path.FindSmoothPoint(proj_on_path.s + result->tar_s, &(result->tar_pos));
        }
        if (tar_a < result->tar_a) {
          result->tar_a = tar_a;
        }
      }
    }
  }
}

// 更新规划的速度TargetVelocityInfo
bool VelocityPlanningFuzzyPID::UpdateTargetVelocityInfo(const TargetVelocityInfo &new_info,
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

// 根据规划的速度结果tar_v_info），抖动抑制为了防止速度频繁加减
/*
 * @brief 根据规划的速度结果tar_v_info），抖动抑制为了防止速度频繁加减
 * @param[in] JitterSuppression& jitter_suppression
 *            TargetVelocityInfo& tar_v_info
 * @return  TargetVelocityInfo* result
 *          result->tar_type   result->tar_pos    result->tar_v       result->tar_a     result->tar_s
 */
void VelocityPlanningFuzzyPID::SuppressJitter(JitterSuppression &jitter_suppression,
                                              const TargetVelocityInfo &tar_v_info, TargetVelocityInfo *result) const {
  bool req_deceleration = false;

  if (VELOCITY_PLANNING_TARGET_TYPE_INVALID != tar_v_info.tar_type) {
    if ((tar_v_info.tar_a < -0.5F) || (tar_v_info.tar_v < (curr_position_.v - 2.0F / 3.6F))) {
      req_deceleration = true;
    } else if (tar_v_info.tar_v < 0.1F) {
      req_deceleration = true;
    } else {
      req_deceleration = false;
    }
  } else {
    req_deceleration = false;
  }

  if (req_deceleration) {
    jitter_suppression.acc_request_counter--;
    if (jitter_suppression.acc_request_counter < 0) {
      jitter_suppression.acc_request_counter = 0;
    }
    jitter_suppression.dec_request_counter++;
    if (jitter_suppression.dec_request_counter > param_.jitter_suppressing_dece_req_counter_threshold) {
      jitter_suppression.dec_request_counter = param_.jitter_suppressing_dece_req_counter_threshold;
    }
  } else {
    jitter_suppression.acc_request_counter++;
    if (jitter_suppression.acc_request_counter > param_.jitter_suppressing_acce_req_counter_threshold) {
      jitter_suppression.acc_request_counter = param_.jitter_suppressing_acce_req_counter_threshold;
    }
    jitter_suppression.dec_request_counter--;
    if (jitter_suppression.dec_request_counter < 0) {
      jitter_suppression.dec_request_counter = 0;
    }
  }

  switch (jitter_suppression.status) {
    case (JitterSuppression::STATUS_REQ_DECELERATION):
      if (req_deceleration) {
        jitter_suppression.tar_velocity_info = tar_v_info;
      }

      *result = jitter_suppression.tar_velocity_info;

      if (jitter_suppression.dec_delay_time > 0) {
        if (jitter_suppression.dec_request_counter >= param_.jitter_suppressing_dece_req_counter_threshold) {
          jitter_suppression.status = JitterSuppression::STATUS_IN_DECELERATION;
        } else {
          result->tar_v = curr_position_.v;
          result->tar_a = 0;
        }
      } else {
        jitter_suppression.status = JitterSuppression::STATUS_NO_OBSTACLE;
        result->tar_v = curr_position_.v;
        result->tar_a = 0;
      }
      break;

    case (JitterSuppression::STATUS_IN_DECELERATION):
      if (req_deceleration) {
        jitter_suppression.tar_velocity_info = tar_v_info;
      }

      *result = jitter_suppression.tar_velocity_info;

      if (req_deceleration) {
        // nothing to do
      } else {
        jitter_suppression.status = JitterSuppression::STATUS_REQ_ACCELERATION;
        jitter_suppression.acc_delay_time = param_.jitter_suppressing_acce_delay_time;
        jitter_suppression.dec_delay_time = 0;
      }
      jitter_suppression.acc_request_counter = 0;
      jitter_suppression.dec_request_counter = 0;
      break;

    case (JitterSuppression::STATUS_REQ_ACCELERATION):
      if (req_deceleration) {
        jitter_suppression.tar_velocity_info = tar_v_info;
      }

      *result = jitter_suppression.tar_velocity_info;

      if (jitter_suppression.acc_delay_time > 0) {
        if (jitter_suppression.acc_request_counter >= param_.jitter_suppressing_acce_req_counter_threshold) {
          jitter_suppression.status = JitterSuppression::STATUS_NO_OBSTACLE;
        }
      } else {
        jitter_suppression.status = JitterSuppression::STATUS_IN_DECELERATION;
      }
      break;

    default:
      if (req_deceleration) {
        *result = jitter_suppression.tar_velocity_info;

        jitter_suppression.status = JitterSuppression::STATUS_REQ_DECELERATION;
        jitter_suppression.dec_delay_time = param_.jitter_suppressing_dece_delay_time;
        jitter_suppression.acc_delay_time = 0;
        jitter_suppression.tar_velocity_info = tar_v_info;
      } else {
        *result = tar_v_info;

        jitter_suppression.tar_velocity_info = tar_v_info;
      }
      jitter_suppression.acc_request_counter = 0;
      jitter_suppression.dec_request_counter = 0;
      break;
  }
}

// 目标障碍物分类
bool VelocityPlanningFuzzyPID::ClassifyTargetByObstacle(
    const ParamOfCalcTarVelocityLinearly &param_of_calc_tar_v, const common::Path &path,
    const common::PathPoint &proj_on_path, bool is_uncertain, plan_var_t cur_v_for_plan,
    const driv_map::CollisionTestOnPathResult::ObjInfo &rsk_obj_info, ClassifyTargetByObstacleResult *const result,
    const driv_map::CollisionTestOnPathResult::ObjInfo *tar_follwing_obj) const {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "#### 目标分类 ClassifyTargetByObstacle (begin)#####"<< std::endl;
#endif
  const driv_map::CollisionTestResult::ObjInfo &test_ret = rsk_obj_info.test_ret;
  const ad_msg::Obstacle &tar_obj = driving_map_->GetObstacle(test_ret.obj_list_index);

  if (is_uncertain) {
    if (test_ret.static_distance > 0.01F) {
      return false;
    }
  } else {
    if (test_ret.static_distance > 2.0F) {
      return false;
    }
  }

  bool need_ctl = false;

  result->Clear();
  result->tar_velocity_info.obstacle_info.valid = false;
  result->tar_velocity_info.obstacle_info.dist_to_obj = common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
  result->tar_velocity_info.obstacle_info.obj_v = tar_obj.v;
  result->tar_velocity_info.obstacle_info.velocity_limit = curr_position_.v;

  // 确定安全距离
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    result->tar_velocity_info.obstacle_info.safe_dist = param_.safe_distance_in_backing_mode;//2.0m
  } else {
    if (tar_obj.dynamic) {
      result->tar_velocity_info.obstacle_info.safe_dist = common::Max(
          param_.safe_time_to_dynamic_obstacle * curr_position_.v, param_.safe_distance_to_dynamic_obstacle);//2.0s
    } else {
      result->tar_velocity_info.obstacle_info.safe_dist = param_.safe_distance_to_static_obstacle;
    }
  }
    //XXX: 判断是否允许跟随/低速跟随
  bool allow_following = false;//初始默认值
  bool allow_low_speed_following = false;//初始默认值

  if (tar_obj.dynamic) {
    std::cout << " Step 1 : 动态" << std::endl;

    //如果是动态障碍物，根据目标9个方位区域，判断是否need_ctrl（障碍物分类限速）
    switch (rsk_obj_info.obj_position) {
      case (driv_map::OBJ_POSITION_FRONT): {
        // 前方障碍物考虑逆向，不考虑已跟随目标前方的车辆
        if (rsk_obj_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "Step 1 : 正前方障碍物,逆向行驶" << std::endl;
#endif
#if ENABLE_VELOCITY_PLANNING_LOG
          std::cout << "common::com_abs(rsk_obj_info.tar_obj_proj.l)=" << common::com_abs(rsk_obj_info.tar_obj_proj.l)
                    << ", tar_obj.obb.half_width+0.5F*param_.vehicle_width="
                    << tar_obj.obb.half_width + 0.5F * param_.vehicle_width << ", rsk_obj_info.min_static_distance=" 
                    << rsk_obj_info.min_static_distance <<  ", test_ret.static_distance=" << test_ret.static_distance << std::endl;
#endif
          /// TODO: 减小逆向车道的车辆对本车道的影响(待测试)
          if (test_ret.static_distance < 0.1F) {
            need_ctl = true;
            result->tar_velocity_info.obstacle_info.velocity_limit = 0.0F;
            result->tar_velocity_info.obstacle_info.dist_to_obj =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          }
        } else { 
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "Step 1 : 正前方障碍物,同向行驶" << std::endl;
#endif
          need_ctl = true;
          plan_var_t abs_angle_diff =
              common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
          result->tar_velocity_info.obstacle_info.velocity_limit = tar_obj.v; /* common::com_cos(abs_angle_diff)*/
          result->tar_velocity_info.obstacle_info.dist_to_obj = rsk_obj_info.dist_to_tar_obj;
          if (settings_.enable_following) {
            allow_following = true;
          }

          /*k001 pengc 2022-04-27 (begin) */
          // 多目标跟随时，过滤掉跟随目标前方的障碍物
          if (Nullptr_t != tar_follwing_obj) {
            if (rsk_obj_info.dist_to_tar_obj > (tar_follwing_obj->dist_to_tar_obj + 1.0F)) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "Step 1.1 : dist_to_tar_obj > dist_to_tar_obj + 1.0F " << std::endl;
#endif
              if (is_uncertain) {
                need_ctl = false;
              }
              if (tar_obj.confidence < 80) {
                need_ctl = false;
              }
              // 只跟随融合的障碍物
              if (ad_msg::OBJ_PRCP_TYPE_FUSED != tar_obj.perception_type) {
                need_ctl = false;
              }
            }
          }
          /*k001 pengc 2022-04-27 (end) */
        }
      } break;

        // 侧前方目标考虑横向切入，同向与逆向切入的；同向的不切入的，本车要限速
      case (driv_map::OBJ_POSITION_LEFT_FRONT):
        // 区域2：左前方障碍物
      case (driv_map::OBJ_POSITION_RIGHT_FRONT): {
        // 区域8：右前方障碍物
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "Step 2 : 侧方障碍物" << std::endl;
#endif
        if (rsk_obj_info.obj_direction == driv_map::OBJ_DIRECTION_BACKWARD) {
          // 侧前方障碍物,逆向行驶

          if (rsk_obj_info.cut_in) {
            // 障碍物将切入车道

            need_ctl = true;
            result->tar_velocity_info.obstacle_info.velocity_limit = 0.0F;
            result->tar_velocity_info.obstacle_info.dist_to_obj =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          } else {
            // 侧前方障碍物,不会切入车道,忽略

            need_ctl = false;
          }
        } else if (rsk_obj_info.obj_direction == driv_map::OBJ_DIRECTION_CROSSED) {
          // 侧前方障碍物,横向靠近本车
          if (test_ret.risk_value > 50) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << "风险较大，需受控" << std::endl;
#endif
            need_ctl = true;
            result->tar_velocity_info.obstacle_info.velocity_limit = 0.0F;
            result->tar_velocity_info.obstacle_info.dist_to_obj =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          } else {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << " 风险不大，不需受控" << std::endl;
#endif 
            need_ctl = false;
          }

        } else {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << " 侧前方障碍物,同向行驶" << std::endl;
#endif
          if (rsk_obj_info.cut_in) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << " 障碍物将切入车道" << std::endl;
#endif
            need_ctl = true;
            plan_var_t abs_angle_diff =
                common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
            result->tar_velocity_info.obstacle_info.velocity_limit = tar_obj.v; /* common::com_cos(abs_angle_diff)*/
            result->tar_velocity_info.obstacle_info.dist_to_obj =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);
          } else {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << " 障碍物不会切入车道, 限制相对速度（同向行驶，贴线场景）" << std::endl;
#endif
            need_ctl = true;
            plan_var_t t = 0;
            Int32_t lower = common::LerpInOrderedTable(param_.relative_velocity_limit_table_by_dist_to_dynamic_obj,
                                                       test_ret.static_distance, &t);
            plan_var_t relative_velocity_limit =
                common::Lerp(param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[lower].value,
                             param_.relative_velocity_limit_table_by_dist_to_dynamic_obj[lower + 1].value, t);
            plan_var_t abs_angle_diff =
                common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
            result->tar_velocity_info.obstacle_info.velocity_limit =
                tar_obj.v /* common::com_cos(abs_angle_diff)*/ + relative_velocity_limit;//动态，在障碍物速度基础上+抬高的相对速度
            result->tar_velocity_info.obstacle_info.dist_to_obj =
                common::Min(rsk_obj_info.dist_to_tar_obj, test_ret.collision_s);

#if ENABLE_VELOCITY_PLANNING_LOG
            std::cout << " lat_dist=" << test_ret.static_distance << ", obj_v=" << tar_obj.v * 3.6F
                      << ", rel_v_limit=" << relative_velocity_limit * 3.6F
                      << ", abs_angle_diff=" << common::com_rad2deg(abs_angle_diff)
                      << ", cos(abs_angle_diff)=" << common::com_cos(abs_angle_diff)
                      << ", v_limit=" << result->tar_velocity_info.obstacle_info.velocity_limit * 3.6F << std::endl;
#endif
          }
        }
      } break;
      
      // 区域9：紧靠车辆的障碍物
      case (driv_map::OBJ_POSITION_CLOSE): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "Step 2 : 紧靠车辆的障碍物" << std::endl;
#endif 
        //如果与紧靠的障碍物距离＞定位点与前轴距离
        if ((rsk_obj_info.dist_to_tar_obj - param_.dist_of_localization_to_front) > 0.0F) {
          need_ctl = true;
          result->tar_velocity_info.obstacle_info.velocity_limit = 0.0F;
          result->tar_velocity_info.obstacle_info.dist_to_obj = 0.0F;
        }
      } break;
      // 区域3：左侧
      case (driv_map::OBJ_POSITION_LEFT): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 左侧障碍物，忽略" << std::endl;
#endif
      } break;
      // 区域7：右侧
      case (driv_map::OBJ_POSITION_RIGHT): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 右侧障碍物，忽略" << std::endl;
#endif
      } break;
      // 区域4：左后方
      case (driv_map::OBJ_POSITION_LEFT_BACK): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 左后方障碍物，忽略" << std::endl;
#endif
      } break;
      // 区域5：正后方
      case (driv_map::OBJ_POSITION_BACK): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 正后方障碍物，忽略" << std::endl;
#endif
      } break;
      // 区域6：右后方
      case (driv_map::OBJ_POSITION_RIGHT_BACK): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 右后方障碍物，忽略" << std::endl;
#endif
      } break;
      default:
        break;
    }
  } else {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "Step 1 : 静态" << std::endl;
#endif
    switch (rsk_obj_info.obj_position) {
      case (driv_map::OBJ_POSITION_FRONT): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "Step 2 : 正前方障碍物" << std::endl;
#endif
        // 正前方障碍物
        need_ctl = true;
        // plan_var_t abs_angle_diff = common::com_abs(common::AngleDiff(
        //    rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
        // result->tar_velocity_info.obstacle_info.velocity_limit =
        //    tar_obj.v * common::com_cos(abs_angle_diff);
        // result->tar_velocity_info.obstacle_info.velocity_limit = 0.0F;
        result->tar_velocity_info.obstacle_info.velocity_limit = tar_obj.v;
        result->tar_velocity_info.obstacle_info.dist_to_obj = rsk_obj_info.dist_to_tar_obj;
        if (settings_.enable_low_speed_following && (tar_obj.v_x > -5.0F / 3.6F)) {//
          allow_low_speed_following = true;
        }
      } break;
      case (driv_map::OBJ_POSITION_CLOSE): {
        // 紧靠车辆的障碍物
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "Step 2 : 紧靠车辆的障碍物" << std::endl;
#endif
        if ((rsk_obj_info.dist_to_tar_obj - param_.dist_of_localization_to_front) > 0.0F) {
          need_ctl = true;
          result->tar_velocity_info.obstacle_info.velocity_limit = 0.0F;
          result->tar_velocity_info.obstacle_info.dist_to_obj = rsk_obj_info.dist_to_tar_obj;
        }
      } break;
      case (driv_map::OBJ_POSITION_LEFT_FRONT):
        // 左前方障碍物
      case (driv_map::OBJ_POSITION_RIGHT_FRONT): {
        // 右前方障碍物
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " Step 2 : 侧方障碍物" << std::endl;
#endif
        need_ctl = true;
        plan_var_t t = 0;
        Int32_t lower = common::LerpInOrderedTable(param_.relative_velocity_limit_table_by_dist_to_static_obj,
                                                   test_ret.static_distance, &t);
        plan_var_t relative_velocity_limit =
            common::Lerp(param_.relative_velocity_limit_table_by_dist_to_static_obj[lower].value,
                         param_.relative_velocity_limit_table_by_dist_to_static_obj[lower + 1].value, t);
        plan_var_t abs_angle_diff =
            common::com_abs(common::AngleDiff(rsk_obj_info.tar_obj_proj.heading, tar_obj.obb.heading));
        result->tar_velocity_info.obstacle_info.velocity_limit =
            tar_obj.v /* common::com_cos(abs_angle_diff)*/ + relative_velocity_limit; //
            //动态，在障碍物速度0km/h基础上+抬高的相对速度
        result->tar_velocity_info.obstacle_info.dist_to_obj = rsk_obj_info.dist_to_tar_obj;
      } break;

      case (driv_map::OBJ_POSITION_LEFT): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 左侧障碍物，忽略" << std::endl;
#endif
      } break;
      case (driv_map::OBJ_POSITION_RIGHT): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 右侧障碍物，忽略" << std::endl;
#endif
      } break;
      case (driv_map::OBJ_POSITION_LEFT_BACK): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 左后方障碍物，忽略" << std::endl;
#endif
      } break;
      case (driv_map::OBJ_POSITION_BACK): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 正后方障碍物，忽略" << std::endl;
#endif
      } break;
      case (driv_map::OBJ_POSITION_RIGHT_BACK): {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << " 右后方障碍物，忽略" << std::endl;
#endif
      } break;

      default:
        break;
    }
  }


  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    result->tar_velocity_info.obstacle_info.dist_to_obj -= param_.dist_of_localization_to_rear;
  } else {
    result->tar_velocity_info.obstacle_info.dist_to_obj -= param_.dist_of_localization_to_front;
  }

  if (!need_ctl) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "不需受控_1" << std::endl;
#endif
    return false;
  }

  /// XXX: 以下几种情况不 （跟车/低速跟车）
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "Step 2.1 : allow_folllowing = " << allow_following << " , "
            << " allow_low_speed_following = " << allow_low_speed_following << std::endl;
#endif
  if (allow_following || allow_low_speed_following) {
    // 不跟随不确定的障碍物
    if (is_uncertain) {
      allow_following = false;
      allow_low_speed_following = false;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
      std::cout << "step 3 : is_uncertain " << std::endl;
#endif
    }
    // 不跟随置性度小于80的障碍物 
    if (tar_obj.confidence < 80) {
        allow_following           = false;
        allow_low_speed_following = false;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "step 3 : tar_obj.confidence < 80 " << std::endl;
#endif
    }
    /*
    if (ad_msg::OBJ_PRCP_TYPE_FUSED != tar_obj.perception_type) {
        allow_following           = false;
        allow_low_speed_following = false;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "step 3 : tar_obj.perception_type is not fusion" << std::endl;
#endif
    }
    if ((ad_msg::OBJ_TYPE_PASSENGER_VEHICLE != tar_obj.type) &&
        (ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE != tar_obj.type) &&
        (ad_msg::OBJ_TYPE_SPECIAL_VEHICLE != tar_obj.type) && (ad_msg::OBJ_TYPE_OTHER_VEHICLE != tar_obj.type) &&
        (ad_msg::OBJ_TYPE_BICYCLE != tar_obj.type) && (ad_msg::OBJ_TYPE_PEDESTRIAN != tar_obj.type))            {
        // 只跟随车辆 ==> 如果类型不是乘用车/商用车/特殊车/自行车，则不（跟车/低速跟车）
        allow_following           = false;
        allow_low_speed_following = false;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "step 3 : tar_obj.type is not vehicle" << std::endl;
#endif
    }
    // 当前速度过快则不允许低速跟随
    if (allow_low_speed_following) {
        if (curr_position_.v > 15.0F / 3.6F)  {
            allow_low_speed_following = false;
        }
    }
    */
  }
  // TODO: 1、CalcTarVelocityLinearly计算粗解,其结果存在ClassifyTargetByObstacle中
  /*即注意result->tar_velocity_info.tar_v 和result->tar_velocity_info.tar_a的输出，否则默认0，
    测试会出现一进入Obstacle状态为3，取小后急刹，因此不能注释*/
  /*
    std::cout << "\n###### ClassifyTargetByObstacle (CalcTarVelocityLinearly0) (Begin)######" << std::endl;
      need_ctl = CalcTarVelocityLinearly(
          param_of_calc_tar_v, curr_position_.v, result->tar_velocity_info.obstacle_info.velocity_limit,
          result->tar_velocity_info.obstacle_info.dist_to_obj, result->tar_velocity_info.obstacle_info.safe_dist,
          &(result->tar_velocity_info.tar_v), &(result->tar_velocity_info.tar_a),
          &(result->tar_velocity_info.obstacle_info.dist_gap_level),
          &(result->tar_velocity_info.obstacle_info.rel_spd_level));

  result->tar_velocity_info.tar_v_true = result->tar_velocity_info.tar_v;
  result->tar_velocity_info.tar_a_true = result->tar_velocity_info.tar_a;

  std::cout << " CalcTarVelocityLinearly " << "obstacle_info_velocity_limit = " <<
  result->tar_velocity_info.obstacle_info.velocity_limit
  << ", tar_velocity_info.tar_v  = " << result->tar_velocity_info.tar_v << ", tar_velocity_info.tar_a  = " <<
  result->tar_velocity_info.tar_a
  << std::endl;

  std::cout << " Every Obs need control = " <<  need_ctl << std::endl;
  */
  // 强制为true 障碍物分类
  need_ctl = true;

  // 根据设置参数确认tar_type
  if (!need_ctl) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "不需受控_2" << std::endl;
#endif
    return false;
  }

  // TODO：跟随和低速跟随的满足条件：HMI界面上settings_.enable_following且allow_following
  if (settings_.enable_following && allow_following) {
    result->tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "  请求跟随 step 4 : tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING " << std::endl;
#endif
  } else if (settings_.enable_low_speed_following && allow_low_speed_following) {
    result->tar_velocity_info.obstacle_info.safe_dist = param_.safe_distance_for_low_speed_following;
    result->tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << " 请求低速跟随 step 4 : tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING "
              << std::endl;
#endif
  } else {
    if (rsk_obj_info.obj_position == driv_map::OBJ_POSITION_LEFT_FRONT ||
        rsk_obj_info.obj_position == driv_map::OBJ_POSITION_RIGHT_FRONT) {
      if (test_ret.static_distance < 0.5F) {
          result->tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << " 请求速度控制 step 4 : tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE " << std::endl;
#endif
        }
      }
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << " ClassifyTargetByObstacle "
            << " , "
            << " Target_Type = " << result->tar_velocity_info.tar_type << " , "
            << " tar_velocity_info.tar_v = " << result->tar_velocity_info.tar_v_true << " , "
            << " result->tar_velocity_info.tar_a = " << result->tar_velocity_info.tar_a_true << std::endl;
#endif


  // 针对选择的目标计算相关判定参数（TS，TTC）
  // 由于坐标不同的原因导致的在 ttc 计算的时候 会将时间上变小 
  // 在计算的时候 再将 该距离 dist_of_localization_to_front 加回来 从而 减小 aeb 的触发频率
  result->tar_velocity_info.tar_s =
      result->tar_velocity_info.obstacle_info.dist_to_obj - result->tar_velocity_info.obstacle_info.safe_dist; //
  path.FindSmoothPoint(proj_on_path.s + common::Max(0.0F, result->tar_velocity_info.tar_s),
                       &(result->tar_velocity_info.tar_pos));

  result->tar_velocity_info.obstacle_info.time_gap =
      result->tar_velocity_info.obstacle_info.dist_to_obj / cur_v_for_plan;
  if (result->tar_velocity_info.obstacle_info.time_gap < 0.0F) {
    result->tar_velocity_info.obstacle_info.time_gap = 0.0F;
  }
  result->tar_velocity_info.obstacle_info.relative_v =
      result->tar_velocity_info.obstacle_info.velocity_limit - curr_position_.v;
  result->tar_velocity_info.obstacle_info.ttc = 100.0F;
  if (result->tar_velocity_info.obstacle_info.relative_v < -0.01F) {
    result->tar_velocity_info.obstacle_info.ttc =
        common::Max(0.0F, (result->tar_velocity_info.obstacle_info.dist_to_obj + param_.dist_of_localization_to_front)) /
        (-result->tar_velocity_info.obstacle_info.relative_v);
    if (result->tar_velocity_info.obstacle_info.ttc > 100.0F) {
      result->tar_velocity_info.obstacle_info.ttc = 100.0F;
    }
  }
  result->tar_velocity_info.obstacle_info.valid = true;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "Step 5 : 规划车速: tar_v = " << result->tar_velocity_info.tar_v * 3.6F
            << "km/h, tar_a = " << result->tar_velocity_info.tar_a << " m/s^2 " << std::endl;
#endif

  // 根据TTC值与查表获取的TTC阈值进行比较确认是否触发AEB不同等级
  if ((settings_.enable_aeb) && (result->tar_velocity_info.obstacle_info.ttc < 4.0F) &&
      (test_ret.static_distance < 0.01F) && (!is_uncertain) && (tar_obj.confidence >= 90)) {
    result->aeb_warning = true;
  }

  // 获取目标TTC
  plan_var_t ratio = 0;
  Int32_t lower = common::LerpInOrderedTable(param_.aeb_action_ttc_table, curr_position_.v, &ratio);
  plan_var_t tar_ttc =
      common::Lerp(param_.aeb_action_ttc_table[lower].value, param_.aeb_action_ttc_table[lower + 1].value, ratio);
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << " 当前车速: cur_v=" << curr_position_.v * 3.6F << "km/h, tar_ttc=" << tar_ttc << "s" << std::endl;
#endif
//TODO: AEB 
  if ((settings_.enable_aeb) && (result->tar_velocity_info.obstacle_info.ttc < tar_ttc) &&
      (test_ret.static_distance < 0.01F) && (!is_uncertain) && (tar_obj.confidence >= 90) && (tar_obj.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED)) {
#if ENABLE_VELOCITY_PLANNING_LOG
    printf(
        "@@@@ enable_aeb=%d, ttc=%0.1f, tar_ttc=%0.1f, static_dist=%0.1f"
        ", is_uncertain=%d, confidence=%d\n",
        settings_.enable_aeb, result->tar_velocity_info.obstacle_info.ttc, tar_ttc, test_ret.static_distance,
        is_uncertain, tar_obj.confidence);
    printf("@@@@ AEB_ACTION \n");
#endif
    result->aeb_action = true;
    result->tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION;
    result->req_dec = true;
    result->tar_velocity_info.tar_v = 0.0F;
    // result->tar_velocity_info.tar_a =
    //    common::Min(-4.0F, result->tar_velocity_info.tar_a);
    result->tar_velocity_info.tar_a = -4.0F;
    std::cout << "请求AEB" << std::endl;
  } else {
    if (settings_.enable_acc) {
      if (is_uncertain) {
        if (tar_obj.confidence < 50) {
          need_ctl = false;
        } else {
          if (result->tar_velocity_info.tar_a < -0.5F) {
            // 告警
            result->is_uncertain = true;
          }
        }
        result->req_dec = false;
      } else {
        if (tar_obj.confidence < 50) {
          need_ctl = false;
          result->req_dec = false;
        } else if (tar_obj.confidence < 60) {
          if (result->tar_velocity_info.tar_a < -0.5F) {
            // 告警
            result->is_uncertain = true;
          }
          result->req_dec = false;
        } else if (tar_obj.confidence < 70) {
          if (result->tar_velocity_info.tar_a < -0.5F) {
            // 告警
            result->is_uncertain = true;
          }
          if (result->tar_velocity_info.tar_a < -1.0F) {
            result->tar_velocity_info.tar_a = -1.0F;
          }
          result->req_dec = true;
        } else if (tar_obj.confidence < 80) {
          if (result->tar_velocity_info.tar_a < -0.5F) {
            // 告警
            result->is_uncertain = true;
          }
          if (result->tar_velocity_info.tar_a < -2.0F) {
            result->tar_velocity_info.tar_a = -2.0F;
          }
          result->req_dec = true;
        } else if (tar_obj.confidence < 90) {
          result->req_dec = true;
        } else {
          result->req_dec = true;
        }
      }
    } else if (settings_.enable_aeb_pre_dec && result->aeb_warning && !result->aeb_action) {
      result->tar_velocity_info.tar_type = VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN;
      result->req_dec = true;
      // result->tar_velocity_info.tar_v = 10.0F/3.6F;
      result->tar_velocity_info.tar_v = chassis_status_.v - 10.0F / 3.6F;  // curr_a=chassis_status_.a
      if (result->tar_velocity_info.tar_v < 10.0F / 3.6F) {
        result->tar_velocity_info.tar_v = 10.0F / 3.6F;
      }
      result->tar_velocity_info.tar_a = -0.5F;
      std::cout << " 请求AEB预减速" << std::endl;

    } else {
      need_ctl = false;
    }
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "###################### 目标分类 ClassifyTargetByObstacle (End)###########################:"
            << std::endl;
#endif
  return (need_ctl);
}

// 发出事件报告
void VelocityPlanningFuzzyPID::AddEventReporting(Int32_t event_type) {
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
void VelocityPlanningFuzzyPID::CreateVehOBB(const common::Vec2d &pos, plan_var_t heading, common::OBBox2d *obb) const {
  const plan_var_t half_veh_width = param_.vehicle_width * 0.5f;
  const plan_var_t half_veh_length = param_.vehicle_length * 0.5f;

  obb->set_unit_direction(common::com_cos(heading), common::com_sin(heading));
  obb->set_center(pos + param_.dist_of_localization_to_center * obb->unit_direction_x());
  obb->set_extents(half_veh_length, half_veh_width);
}

/*
 * @brief 平滑规划的速度
 * @param[in] tar_v ，tar_a
 * @param[out] smoothed_tar_v ，smoothed_tar_a
 */
void VelocityPlanningFuzzyPID::SmoothVelocity(VelocityPlanningResult vel_planning_result, plan_var_t *smoothed_tar_v,
                                              plan_var_t *smoothed_tar_a) {
  plan_var_t deceleration_jerk;
  if(curr_position_.v > 80.0 / 3.6){
    deceleration_jerk = -1.0;
  }else {
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
      if (((vel_planning_result.tar_obj.obj_v * 3.0 + 3.0) < vel_planning_result.tar_obj.dist_to_obj)) {
          if (((*smoothed_tar_a - array_target_acceleration_[0]) / planning_cycle_time)  < deceleration_jerk) {
            *smoothed_tar_a = array_target_acceleration_[0] +  deceleration_jerk * planning_cycle_time;
          }
      }
    }
    /*
   *smoothed_tar_a = 0.1F * array_target_acceleration_[2] + 0.2F * array_target_acceleration_[1] +
                     0.3F * array_target_acceleration_[0] + 0.4F * tar_a;
   */
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
      if (((vel_planning_result.tar_obj.obj_v * 3.0 + 3.0) < vel_planning_result.tar_obj.dist_to_obj)) {
        if (((*smoothed_tar_v - array_target_velocity_[0]) / planning_cycle_time) < *smoothed_tar_a) {
          *smoothed_tar_v = array_target_velocity_[0] + (*smoothed_tar_a) * planning_cycle_time; 
        }
      }
      
    }
    /*
     *smoothed_tar_v = 0.1F * array_target_velocity_[2] + 0.2F * array_target_velocity_[1] +
                       0.3F * array_target_velocity_[0] + 0.4F * tar_v;
     */
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

/*
 * 相对距离等级, 相对速度等级
 */

// 设置距离等级表（从负大到正大排列等级）：30m表示离跟车2s时距的距离 -15m表示超过2s跟车时距的距离
static const plan_var_t s_distance_gap_level_table[7] = {
    /*       0            1          2        3         4          5           6        */
    /*  0          1           2         3         4          5           6        7    */
    /* 负大  ][    负中    ][   负小   ][  负零  ][  正零  ][   正小   ][    正中   ][ 正大  */
    -15.0F, -10.0F, -5.0F, 0.0F, 10.0F, 20.0F, 30.0F};

// 设置前车速度与当前车速差(目标车速度 - 自车速度)的等级序表（从负大到正大排列等级）
static const plan_var_t s_relative_velocity_level_table[7] = {
    /*       0            1          2          3           4          5            6       */
    /*   0         1           2           3          4           5           6        7    */
    /* 负大  ][    负中    ][   负小   ][   负零   ][   正零   ][   正小   ][    正中   ][ 正大   */
    -15.0F / 3.6F, -10.0F / 3.6F, -5.0F / 3.6F, 0.0F / 3.6F, 5.0F / 3.6F, 10.0F / 3.6F, 20.0F / 3.6F};

/*
 * 线性控制：针对非跟车场景（停车和精准停车定减速）
 * @brief 根据目标速度及相对距离计算tar_v和tar_a
 * @details： 1：根据Fuzzy获取速度差与距离差的增益系数;2：根据增益换算计算tar_v和tar_a
 * @param[in] curr_v, obj_v, dist_to_obj, safe_dist
 * @param[out] 1、目标速度：tar_v(定减速) ； 2、目标加速度：tar_a（恒定负值）
 *             3、dist_gap_level 和rel_spd_level 观测量
 */
bool VelocityPlanningFuzzyPID::CalcTarVelocityLinearly(const ParamOfCalcTarVelocityLinearly &param, plan_var_t curr_v,
                                                       plan_var_t obj_v, plan_var_t dist_to_obj, plan_var_t safe_dist,
                                                       plan_var_t *tar_v, plan_var_t *tar_a, Int32_t *dist_gap_level,
                                                       Int32_t *rel_spd_level) const {
  plan_var_t s_diff = dist_to_obj - safe_dist;  // 相对距离=障碍物速度-安全距离
  plan_var_t v_diff = obj_v - curr_v;           // 相对速度 = 障碍物速度-本车速度

  // #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "     **** Linear dec:"
            << "\n          curr_v=" << curr_v * 3.6F << ", obj_v=" << obj_v * 3.6F << ", s_diff=" << s_diff
            << ", v_diff=" << v_diff * 3.6F << std::endl;
  // #endif

  Int32_t dist_gap_level_index = 0;
  Int32_t dist_gap_membership_index = 0;
  Float32_t dist_gap_membership_grade = 0.0F;
  Int32_t rel_spd_level_index = 0;
  Int32_t rel_spd_membership_index = 0;
  Float32_t rel_spd_membership_grade = 0.0F;
  
  /*
  dist_gap_level_index = GetFuzzyLevelIdx<plan_var_t, 8>(s_distance_gap_level_table, s_diff,
                                                           &dist_gap_membership_index, &dist_gap_membership_grade);
  // ？
  rel_spd_level_index = GetFuzzyLevelIdx<plan_var_t, 8>(s_relative_velocity_level_table, v_diff,
                                                          &rel_spd_membership_index, &rel_spd_membership_grade);
  */
 
  *dist_gap_level = dist_gap_level_index;
  *rel_spd_level = rel_spd_level_index;

// 一系列的参数检查Check parameter
#if 1
  if (curr_v < 0.01F) {
    curr_v = 0;
  }
  if (obj_v < 0.01F) {
    obj_v = 0;
  }
  plan_var_t time_of_ctl_range = param.time_of_ctl_range;  // 4s

  if (time_of_ctl_range < 0.1F) {
    time_of_ctl_range = 0.1F;
  }
  plan_var_t min_dist_of_ctl_range = param.min_dist_of_ctl_range;  // 20m
  if (min_dist_of_ctl_range < 0.1F) {
    min_dist_of_ctl_range = 0.1F;
  }
  plan_var_t accl_of_act_cond = param.accelaration_of_activity_condition;  // 0.5m/s^2
  if (accl_of_act_cond < 0.1F) {
    accl_of_act_cond = 0.1F;
  }
  plan_var_t decl_of_act_cond = param.deceleration_of_activity_condition;  //-0.5m/s^2
  if (decl_of_act_cond > -0.1F) {
    decl_of_act_cond = -0.1F;
  }
  plan_var_t min_deceleration_time = param.min_deceleration_time;  // 2s
  if (min_deceleration_time < 0.1F) {
    min_deceleration_time = 0.1F;
  }
#endif

  //
  bool need_control = false;  // 避免出现加速再减速的龙格现象，尽量以当前的速度（不超过限速）直行或过弯
  if (curr_v > obj_v)  // 本车速度 > 障碍物速度，需减速
  {
    plan_var_t act_deceleration_time = (obj_v - curr_v) / decl_of_act_cond;  // 定减速的时间：t=(obj_v - curr_v) / -0.5
    plan_var_t act_deceleration_distance =
        curr_v * act_deceleration_time + 0.5F * decl_of_act_cond * act_deceleration_time *
                                             act_deceleration_time;  // 定减速的距离：s=curr_v*t+1/2*(-0.5)*t^2

    std::cout << " curr_v > obj_v " << std::endl;
    std::cout << "s_diff =" << s_diff << " , "
              << "act_deceleration_distance =" << act_deceleration_distance << std::endl;

    if (s_diff < act_deceleration_distance) {
      need_control = true;
      std::cout << "need dec" << std::endl;
    } else {
      plan_var_t ctl_range = curr_v * time_of_ctl_range;
      std::cout << "ctl_range = " << ctl_range << " , "
                << " time_of_ctl_range = " << time_of_ctl_range << std::endl;
      if (ctl_range < min_dist_of_ctl_range) {
        ctl_range = min_dist_of_ctl_range;
      }
      std::cout << "min_dist_of_ctl_range = " << min_dist_of_ctl_range << " , "
                << " ctl_range = " << ctl_range << std::endl;
      if (s_diff < ctl_range) {
        need_control = true;
        std::cout << "need ctl" << std::endl;
      } else {
        std::cout << " not need ctl" << std::endl;
      }
    }
  } else  // 本车速度 < 障碍物速度，需加速
  {
    plan_var_t act_acceleration_time = (obj_v - curr_v) / accl_of_act_cond;
    plan_var_t act_acceleration_distance =
        curr_v * act_acceleration_time + 0.5F * accl_of_act_cond * act_acceleration_time * act_acceleration_time;

    std::cout << " curr_v < obj_v " << std::endl;
    std::cout << "s_diff =" << s_diff << " , "
              << "act_acceleration_distance =" << act_acceleration_distance << std::endl;

    if (s_diff < act_acceleration_distance) {
      need_control = true;
      std::cout << "need acc " << std::endl;
    } else {
      plan_var_t ctl_range = curr_v * time_of_ctl_range;

      std::cout << "ctl_range = " << ctl_range << " , "
                << " time_of_ctl_range = " << time_of_ctl_range << std::endl;

      if (ctl_range < min_dist_of_ctl_range) {
        ctl_range = min_dist_of_ctl_range;
      }
      std::cout << "min_dist_of_ctl_range = " << min_dist_of_ctl_range << " , "
                << " ctl_range = " << ctl_range << std::endl;

      if (s_diff < ctl_range) {
        need_control = true;
        std::cout << "need ctl" << std::endl;
      } else {
        std::cout << " not need ctl" << std::endl;
      }
    }
  }

  if (need_control) {
    if (obj_v < 1.0F / 3.6F)  // 当obj_v速度较小时
    {
      plan_var_t t = 0.0F;
      Int32_t lower = 0;
      if (param.stop_accurately)  // 配置文件逻辑
      {                           // 精准停车场景，速度限制表的索引lower
        lower = common::LerpInOrderedTable(param_.velocity_limit_table_for_stop_accurately_situation, s_diff,
                                           &t);  // 输出:插值比例t和索引lower
        obj_v = common::Lerp(param_.velocity_limit_table_for_stop_accurately_situation[lower].value,
                             param_.velocity_limit_table_for_stop_accurately_situation[lower + 1].value, t);
      } else {  // 停车场景，速度限制表的索引lower
        lower = common::LerpInOrderedTable(param_.velocity_limit_table_for_stop_situation, s_diff,
                                           &t);  // 输出:插值比例t和索引lower
        obj_v = common::Lerp(param_.velocity_limit_table_for_stop_situation[lower].value,
                             param_.velocity_limit_table_for_stop_situation[lower + 1].value, t);
      }
    }

    *tar_v = obj_v;  // 目标速度：tar_v=obj_v
    if (*tar_v < 0.1F) {
      *tar_v = 0.0F;
    }

    plan_var_t distance = s_diff;
    if (distance < 0.01F) {
      distance = 0.01F;
    }

    plan_var_t t = min_deceleration_time;  // 2s
    if ((curr_v + obj_v) > 0.01F) {
      t = 2.0F * distance / (curr_v + obj_v);  // 定减速：即在相对距离内，速度由curr_v减速到obj_v，所需要的时间
    }
    if (t < min_deceleration_time)  // 不能小于min_deceleration_time=2s
    {
      t = min_deceleration_time;
    }
    *tar_a = (obj_v - curr_v) / t;  // 目标加速度:tar_a<0

    if (obj_v < 0.1F) {
      if (curr_v < 0.1F) {
        if (s_diff < 0.3F) {
          *tar_a = -1.0F;
        }
      }
    }

    // 当处于精确停车模式时，使用固定的减速度
    if (obj_v < 1.0 / 3.6) {
      if (param.stop_accurately) {
        if (s_diff < param_.stop_accurately.braking_distance)  // s_diff <0.8m，直接以-3 m/s^2减速度刹死
        {
          *tar_a = -param_.stop_accurately.proposed_decelaration;  // 目标加速度:tar_a=-3 m/s^2
        }
      }
    }
  }

  // #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "          curr_v=" << curr_v * 3.6F << ", obj_v=" << obj_v * 3.6F << ", tar_v=" << *tar_v * 3.6F
            << ", tar_a=" << *tar_a << ", dist_to_obj=" << dist_to_obj << ", safe_dist=" << safe_dist
            << ", s_diff=" << s_diff << std::endl;
  // #endif

  return (need_control);
}

/*  xiaranfei 2022-07-31 (begin) */
/*
 * 多项式控制：针对非跟车场景（停车和精准停车平滑减速)
 * @brief 根据目标速度及相对距离计算tar_v和tar_a
 * @details：
 * @param[in] curr_v, obj_v, dist_to_obj, safe_dist
 * @param[out] 1、目标速度：tar_v ； 2、目标加速度：tar_a（从小到大）
 */
bool VelocityPlanningFuzzyPID::CalcTarVelocityQuinticPolynomialCurve1d(const ParamOfCalcTarVelocityLinearly &param,
                                                                       plan_var_t curr_v, plan_var_t obj_v,
                                                                       plan_var_t dist_to_obj, plan_var_t safe_dist,
                                                                       bool is_cc, plan_var_t *tar_v,
                                                                       plan_var_t *tar_a) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << " ******************多目标速度规划(begin)****************************** " << std::endl;
#endif
  plan_var_t s_diff;  // safe_dist = 0  可以按AEB法规计算 安全距距离离 = TTC  x  V_ego

  //XXX: 根据自车速度查表获得跟车距离，再计算s_diff
  if (obj_v >= 30 / 3.6) {
    if (curr_v <= 50 / 3.6) {
      s_diff = dist_to_obj - 0.2 * curr_v;
    } else {
      s_diff = dist_to_obj;
    }
  } else {
    s_diff = dist_to_obj - 5.0;
  }
  plan_var_t jerk_ref = 2.0;

  plan_var_t v_diff = obj_v - curr_v;    // 相对速度 = 障碍物速度-本车速度
  Float32_t curr_a = chassis_status_.a;  // 当前加速度
  Float32_t set_v = obj_v;

  Float32_t deta_dec_vx = 0.0;

  Float32_t replan_limit_acc_speed = 1.0;
  Float32_t replan_limit_dec_speed = 2.0;

  if (curr_v < 50 / 3.6) {
    replan_limit_acc_speed = 1.0;
  } else {
    replan_limit_acc_speed = 2.0;
  }
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] BEGIN >>>> curr_v = " << curr_v * 3.6
            << "km/h, curr_a = " << curr_a << "m/s^2"
            << ", dist_to_obj = " << dist_to_obj << "m, s_diff = " << s_diff << "m, obj_v = set_v = " << obj_v * 3.6
            << "km/h " << std::endl;
#endif
  Float32_t x0 = 0.0F, dx0 = curr_v, ddx0 = curr_a, x1 = s_diff, dx1 = obj_v, ddx1 = 0.0F;

// 初始值
// XXX: 重规划(包含起步)
#if (ENABLE_Tar_state)
  if ((!pre_adas_mode_) && (adas_mode_)) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] START ACC Command ! ! " << std::endl;
#endif
    dx0 = curr_v;
    ddx0 = curr_a;
  } else if ((pre_adas_mode_) && (adas_mode_)) {
    if (start_count_ <= 10) {
      dx0 = curr_v;
      ddx0 = curr_a;
    } else {
      // curr_v - 1.0  从0.6改为1.0，在plot显示换档位条件时，tar_v曲线 台阶现象改善
      // curr_v - 0.6 为满载，curr_v - 0.3为半载
      if ((prev_tar_v_ -curr_v  >  replan_limit_acc_speed) ||  //加速
          (curr_v - prev_tar_v_ >  replan_limit_dec_speed)) {  //减速
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Speed is Replan !! prev_tar_v_ = " << prev_tar_v_ * 3.6
                  << " km/h ,  prev_tar_a_ =  " << prev_tar_a_ << " m/s2 "
                  << ", curr_v = " << curr_v * 3.6 << "km/h" << std::endl;
#endif
        dx0 = curr_v;
        ddx0 = curr_a;
      } else {
        dx0 = prev_tar_v_;
        ddx0 = prev_tar_a_;
      }
    }
  } else {
    dx0 = curr_v;
    ddx0 = curr_a;
  }

#endif

#if (ENABLE_Cur_state)
  dx0 = curr_v;
  ddx0 = curr_a;
#endif

#if 0  // polyfit

        Eigen::VectorXf x(6);
        Eigen::VectorXf y(6);
        //{0,0},{0.2,0.5},{0.8,0.95},{1,1},{2,3},{4,6}
        x << 0, 0.2, 0.8, 1, 2, 4;
        y << 0, 0.5, 0.9, 1, 3, 6;

        Eigen::VectorXd xvals(6);
        Eigen::VectorXd yvals(6);
        xvals = x.cast<double>();
        yvals = y.cast<double>();
            std::cout <<  "xvals:\n" <<xvals << std::endl;

        
        //std::vector<float> a = { 1, 2, 3, 4 };
        //Eigen::VectorXf b = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(a.data(), a.size());
        //std::cout << "b:\n"<< b << std::endl;
        //Eigen::VectorXd d = b.cast<double>();
        //std::cout << "d:\n" << d << std::endl;

        Eigen::VectorXd coeffs(6);
        polyfit(xvals, yvals, 5,coeffs);
        std::cout << coeffs << std::endl;
        //std::cout << "Y(1.5)=" << polyeval(coeffs, 1.5) << std::endl;

#endif

  // 构建Polynomial的曲线方程
  // 初始定义值
  Int32_t K = 5;
  // Float32_t SpdPln_lTgtLng;
  // Float32_t SpdPln_vTgtSpd;
  // Float32_t SpdPln_aTgtAcc;
  // Float32_t SpdPln_jTgtJerk;
  // Int32_t BhvCrdn_numBhivD = 1;

#if 0
    //common::CubicPolynomialCurve1d<Float32_t>curve.Construct(0.0F, 10.0F, 100.0F,0.0F, 3.0F);
        common::CubicPolynomialCurve1d<Float32_t> curve(x0, dx0, x1,dx1, 3.0F);
        Float32_t value = curve.Evaluate(1, 0.5F);
        std::cout << "     **** Polynomial value:"      << "\n      value=" <<  value <<std::endl;

        prev_tar_v_ = value;//
        std::cout <<    "\n      prev_tar_v_=" <<  prev_tar_v_ <<std::endl;
#endif

#if 0 
    Float32_t  coef33_[4]; Float32_t  Cubic3[4];//Initial 

        CubicComputeCoefficients(0.0F, 10.0F,  100.0F, 0.0F, 2.0F,coef33_);
        
        CubicPolynomialCurve1d(coef33_, 1, 1.5F, Cubic3);

        std::cout << "     **** Polynomial coef3_:"    << "\n      coef3_[0] =" <<  coef33_[0] << " coef3_[1] =" <<  coef33_[1] << " coef3_[2] =" <<  coef33_[2] << " coef3_[3] =" <<  coef33_[3] << std::endl;
        std::cout << "     **** Polynomial Cubic:"      << "\n      Cubic[0] =" <<  Cubic3[0] << " Cubic[1] =" <<  Cubic3[1] << " Cubic[2] =" <<  Cubic3[2] << " Cubic[3] =" <<  Cubic3[3] << std::endl;
#endif

#if 1  // 一系列的参数检查 Check parameter
  if (curr_v < 0.01F) {
    curr_v = 0;
  }
  if (obj_v < 0.01F) {
    obj_v = 0;
  }
  plan_var_t time_of_ctl_range = param.time_of_ctl_range;  // 4s
  if (time_of_ctl_range < 0.1F) {
    time_of_ctl_range = 0.1F;
  }
  plan_var_t min_dist_of_ctl_range = param.min_dist_of_ctl_range;  // 20m
  if (min_dist_of_ctl_range < 0.1F) {
    min_dist_of_ctl_range = 0.1F;
  }
  plan_var_t accl_of_act_cond = param.accelaration_of_activity_condition;  // 0.5m/s^2
  if (accl_of_act_cond < 0.1F) {
    accl_of_act_cond = 0.1F;
  }
  plan_var_t decl_of_act_cond = param.deceleration_of_activity_condition;  //-0.5m/s^2
  if (decl_of_act_cond > -0.1F) {
    decl_of_act_cond = -0.1F;
  }
  plan_var_t min_deceleration_time = param.min_deceleration_time;  // 2s
  if (min_deceleration_time < 0.1F) {
    min_deceleration_time = 0.1F;
  }
#endif

#if 1
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] INPUT: x0 = " << x0 << "m, dx0 =" << dx0 * 3.6
            << " km/h , ddx0 =" << ddx0 << " m/s2, "
            << "s_diff = " << s_diff << " m, set_v =" << set_v * 3.6 << "km/h" << std::endl;
#endif
  Float32_t tF = 3.0;       // 规划的时间，跟车情况下稳定距离即=tF* set_v
  Float32_t ax_min = -2.0;  // 根据车类型和环境确定最大减速度 -5.0

  Float32_t ax_max;         // 根据车类型和环境确定最大加速度 Note:以当前车速为速度规划起点时 为 2.0m/s
                            // 以上一周期速度规划结果为规划起点时为1.5m/s
  
  if (curr_v > 40 / 3.6) {
    /*
      人工驾驶的时候 60km/h --> 90km/h 的车速的时候 
      在坡度为 2%          附近 平均加速度 0.08 m/s^2 ---> 60%  油门 --> 11 档  换挡之后 11 to 12 会有 -0.1 的掉速
      在坡度为 1.24%       附近 平均加速度 0.25 m/s^2 ---> 100% 油门 --> 11 档
      在坡度为 1.2% ~ 0.2% 附近 平均加速度 0.3m/s^2   ---> 100% 油门 --> 10 档
      需根据不同档位值进行标定 对应的加速值 随着档 上升 -> 加速度 
      0617 在 茶陵 -> 佛山 路段 对 加速度 进行标定
      60%  油门(实际：有坡度 但是 未猜到坡度值) -> 10档 0.35m/s^2   近似  0.4m/s^2
                                           -> 11档 0.269m/s^2  近似  0.3m/s^2
                                           -> 12档 0.18 m/s^2  近似  0.2m/s^2

      100% 油门(实际：有坡度 但是 未猜到坡度值) ->  10档 0.35m/s^2  近似 0.4m/s^2
                                           -> 11档 0.35m/s^2  近似 0.4m/s^2
                                           -> 12档 0.35m/s^2  近似 0.3m/s^2
    */
    //TODO：根据当前档位，限制最大加速度ax_max
    int gear = (int)chassis_status_.gear_number;
    std::cout << "当前档位 : " << gear << std::endl;
    LOG_INFO(5)<<"当前档位 : " << gear;

    #if TM_YD   
        if (gear <= 10) {
          ax_max = 0.3;
        } else if (12 == gear) {
          std::cout << " 当前档位 12档啦!!! "<< std::endl;
          ax_max = 0.15;
        } else if (10 < gear <= 11) {
          ax_max = 0.25;
        }else {
          ax_max = 0.5;
        }
    #endif

    #if TM_ZF
        if (gear <= 10) {
          ax_max = 0.25;
        } else if (12 == gear) {
          std::cout << " 当前档位 12档啦!!! "<< std::endl;
          ax_max = 0.13;
        } else if (10 < gear <= 11) {
          ax_max = 0.2;
        }else {
          ax_max = 0.5;
        }
    #endif

    std::cout << "当前控制下发的量 : " << control_acc_pedal_ 
              << "% , 当前坡度值为 : " << curr_slope_;

    // 在 2.0%的坡度以内的时候， 并不需要 100% 油门的！！!
    // 因此 在控制下发 100% 时候也 需要 通过调整最大加速来限制
    // 控制由于过于激进的 时刻的 100%
    /* 这段代码可能无效！！！！
    if (slope_flag_) {
      if (curr_slope_ <= 2.0 && 
          control_acc_pedal_ >= 80) {
        ax_max = ax_max - curr_slope_ * 0.1;
      }
    }
    if (ax_max < 0) {
        ax_max = 0.0;
    }
    */
  } else {
    ax_max = 0.5;
  }

  if (!is_cc) {   //跟车状态下最大减速度   
    #if (ENABLE_CC_To_ACC)
      plan_var_t ts = s_diff / curr_v;
      plan_var_t follow_ttc = 100.0;
      if (curr_v > obj_v) {
        follow_ttc = -s_diff / v_diff;
      }
      if (curr_v > 50 / 3.6 && v_diff > -20 / 3.6) { //拥堵情况，ax_min要放开到-2
        if (ts >= 3.5) {
          ax_min = -0.2;
        } else if(ts < 3.5 && ts >= 3.0) {
          ax_min = -0.5;
        } else if(ts < 3.0 && ts >= 2.5) {
          ax_min = -0.7;
          if (curr_v < 70 / 3.6 && -10 / 3.6 <= v_diff < 0) {
            ax_min = -1.0;
            #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << "table ax_min = " << ax_min << std::endl;
            #endif
          } else if (curr_v < 60 / 3.6 && v_diff <= -10 / 3.6) {
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
            #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << "follow_ttc 7.0 ax_min = " << ax_min << std::endl;
            #endif
          } 
        } else {
          ax_min = -2.0;
          if (follow_ttc < 5.0) {
            ax_min = -2.5;
            #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
            std::cout << "follow_ttc 5.0 ax_min = " << ax_min << std::endl;
            #endif
          }
        }
      }
      #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
      std::cout << "ax_min = " << ax_min << std::endl;
      #endif
    #endif
  } else { //巡航状态下最大减速度   
    ax_min = -0.2; // 平路的怠速数据 在上下坡的时候在 坡度值的补偿
  }


  Float32_t speed_limit = param_.max_velocity_limit;  // 交通速度限制

  Float32_t vxf_min = common::Max(0.0F, dx0 + ax_min * tF);
  Float32_t vxf_max = common::Min(speed_limit, dx0 + ax_max * tF);
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] V Sample range : "
            << " [ " << vxf_min * 3.6F << " ~ " << vxf_max * 3.6F << " ] km/h" << std::endl;
#endif
  Float32_t vxf_num = 7 * 8;
  Float32_t vxf_range[7 * 8];
  linspace(vxf_min, vxf_max, vxf_num, vxf_range);  // 目标点的采样速度区间
#if ENABLE_VELOCITY_PLANNING_LOG
  for (Int32_t i = 0; i < vxf_num; i++) {
    printf("目标点的采样速度区间:vxf_range=%lf\n", vxf_range[i]);
  };
#endif
  Float32_t axf_range = 0.0F;

  Float32_t xf_min = (pow(vxf_min, 2) - pow(dx0, 2)) / 2 / ax_min;
  Float32_t xf_max = (pow(vxf_max, 2) - pow(dx0, 2)) / 2 / ax_max;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] S Sample range : "
            << " [ " << xf_min << " ~ " << xf_max << " ] m" << std::endl;
#endif


  if (!is_cc) {
#if (ENABLE_Static_Obstacle_Stop)
    if ((obj_v < 5 / 3.6) && (dist_to_obj > xf_max)) {
      Float32_t Static_Stop_Distance = dist_to_obj - 10.0;
      if (Static_Stop_Distance < 0.0) {
        *tar_v = 0.0;
        *tar_a = -1.0;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
        std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Static_Obstacle_Stop  tar_v = " << *tar_v * 3.6
                  << "km/h "
                  << " *tar_a = " << *tar_a << " m /s2 , Static_Stop_Distance = " << Static_Stop_Distance << std::endl;
#endif
        return true;
      } else {
        /*if ((curr_v < 3 / 3.6) && (dist_to_obj > 15.0)) { //不蠕行
          *tar_v = 10 / 3.6;
          *tar_a = 0.8;
        } else {} */
          *tar_a = -1 * (curr_v * curr_v / 2 / Static_Stop_Distance);
          *tar_v = curr_v + *tar_a * 1.0;
          *tar_a = *tar_a * 0.5;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Static_Obstacle_Stop  tar_v = " << *tar_v * 3.6
                  << "km/h "
                  << " *tar_a = " << *tar_a << " m /s2" << std::endl;
#endif
        
        return true;
      }
    }
#endif

#if (ENABLE_CUTIN_Judge)
    if (xf_min > s_diff) {
      if (curr_v > obj_v) {
        if (obj_v > 15.0 / 3.6) {
          // cut in 3s时距
          if (dist_to_obj > 3.0 * obj_v) {
            Float32_t dec = -(v_diff * v_diff / 2 / (s_diff - 3.0 * obj_v));
            if (dec > 0) {
              dec = ax_min;
            }
            if (dec < ax_min) {
              *tar_a = ax_min;
            } else {
              *tar_a = dec;
            }
          } else {
            *tar_a = ax_min;
          }
          *tar_v = curr_v + *tar_a  * 0.1;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Cut_ IN is Doing !!! tar_a " << *tar_a
                    << "m/s^2, tar_v =" << *tar_v << "m/s" << std::endl;
#endif
        } else {
          // 停车 初始化
          Float32_t dec_min;
          if (curr_v <= 10 / 3.6) {
            dec_min = -1.0;
          } else if (10 / 3.6 < curr_v <= 20 / 3.6) {
            dec_min = -1.5;
          } else if (20 / 3.6 < curr_v <= 30 / 3.6) {
            dec_min = -1.8;
          } else if (30 / 3.6 < curr_v <= 40 / 3.6) {
            dec_min = -2.0;
          } else if (40 / 3.6 < curr_v <= 50 / 3.6) {
            dec_min = -2.0;
          } else if (50 / 3.6 < curr_v <= 60 / 3.6) {
            dec_min = -2.5;
          } else if (60 / 3.6 < curr_v <= 90 / 3.6) {
            dec_min = -2.5;
          } else {
            dec_min = -2.5;
          }
          
          *tar_a = dec_min;
          *tar_v = curr_v + dec_min * 1.0;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Stop_ IN is Doing !!!  tar_a " << *tar_a
                    << "m/s^2 , tar_v =" << *tar_v << "m/s" << std::endl;
#endif
          if (*tar_v < 0.0) {
            *tar_v = 0.0;
          }
        }
        return true;//不参与取小
      } else {
        // 后方车辆 超车切入的时候  前车车速 大于 自车的时候
        // 并且距离不在3s的距离规划区间
        // 暂定通过 降低 目标车速 来 拉开距离
        if ((dist_to_obj > 0.0) && (dist_to_obj < curr_v * 0.6)) { //针对v0=90km/h,v1=v0+5km/h切入20m左右，安全距离长时间不足
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] OverTakeing is doing !!! tar_a = " << -0.2
                    << "m/s2, tar_v" << (curr_v - 0.2) << "m/s" << std::endl;
#endif
          return *tar_v = curr_v - 0.2, *tar_a = -0.2;
        } else {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Cutin is not doing !!! tar_a = " << 0.0
                    << "m/s2, tar_v" << curr_v << "m/s" << std::endl;
#endif
          return *tar_v = curr_v, *tar_a = 0.0;
       }
      }
    }
#endif

#if (ENABLE_CC_To_ACC)
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] obj_v = " << obj_v * 3.6 << " km/h" << std::endl;
#endif
    if (obj_v >= 5/ 3.6) {
      if (((dx0 - obj_v) >= 13 / 3.6) && ((dx0 - obj_v) <= 90 / 3.6)) {
        if ((dist_to_obj > xf_max)) {
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
          std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] dist_to_obj = " << dist_to_obj<< " m,  xf_max = " << xf_max << " m " << std::endl;
#endif          
          if (true) {
            if ((xf_max - 5.0) > obj_v * 3.0) {
              set_v = (dist_to_obj - 7.0 * curr_v) * ((curr_v - obj_v) / (7.0 * curr_v - (xf_max - 5.0))) +
                      curr_v;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
              std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Ret Target V is doing !!! " << std::endl;
#endif
            } else {
              set_v = (dist_to_obj - 7.0 * curr_v) * ((curr_v - obj_v) / (7.0 * curr_v - xf_max)) + curr_v;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
              std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Ret Target V is doing !!! " << std::endl;
#endif
            }
          } else {
            set_v = curr_v - 5.0 / 3.6;
          }

        } else {
          // nothing
        }
      } else if (common::com_abs(dx0 - obj_v) < 13 / 3.6) {
        if (dist_to_obj > xf_max) {
          set_v = obj_v + 8 / 3.6;
        }
      } else {
      }
    }
    if (set_v > settings_.tar_v) {
      set_v = settings_.tar_v;
    }
#endif
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Ret Target V : " << set_v * 3.6 << " km/h" << std::endl;
#endif
  } else {
#if (ENABLE_CC_To_ACC)
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] obj_v = " << obj_v * 3.6 << " km/h" << std::endl;
#endif
    if ((dx0 - obj_v) >= 10 / 3.6) {
      set_v = curr_v - 10 / 3.6;
    }
#endif
  }




// XXX: 基于当前车速  ----   测试 巡航 跟车 启步 加速慢 标定 参数 0923
#if (ENABLE_ACC_START_Judge)
  if (curr_v < 0.7 && obj_v > 2.0) {
    *tar_v = 1.0;
    *tar_a = 0.5;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] START Go ACC" << std::endl;
#endif
    return true; 
  }
#endif
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] Replan V range : "
            << " [ " << vxf_min * 3.6F << " ~ " << vxf_max * 3.6F << " ] km/h" 
            << " Replan S range : "
            << " [ " << xf_min << " ~ " << xf_max << " ] m " << std::endl;
#endif


  Float32_t xf_num = 5 * 8;
  Float32_t xf_range[5 * 8];

  linspace(x0 + xf_min, x0 + xf_max, xf_num, xf_range);
#if ENABLE_VELOCITY_PLANNING_LOG
  for (Int32_t i = 0; i < xf_num; i++) {
    printf("目标点的采样距离区间:xf_range=%lf\n", xf_range[i]);
  };
#endif

  Float32_t tf_min = 3;
  Float32_t tf_max = 12;

  Float32_t tf_num = 4;
  Float32_t tf_range[4];

  linspace(tf_min, tf_max, tf_num, tf_range);  // tf_range = linspace(tf_min, tf_max, tf_num); // 目标点的采样时间区间

#if ENABLE_VELOCITY_PLANNING_LOG
  for (int i = 0; i < tf_num; i++) {
    printf("目标点的采样时间区间:tf_range=%lf\n\n", tf_range[i]);
  };
#endif

#endif

#if 1  // QuinticPolynomialPLAN

  /*******************************************************************************/

  // Float32_t  x1[] =  xf_range; //{ 100,120 };//
  // Float32_t  dx1[] =  vxf_range;//{ 10,20,30 };//{10};//
  // Float32_t  ddx1 = axf_range;

  Float32_t dt = 0.1F;
  Float32_t t0 = 0.0F;
  Float32_t t1[] = {3.0F};  //  { 3, 6, 8, 12 };

  /**********************void函数指针输出的数组定义****************************/

  Float32_t coef5_[6], Quintic[6];
  Float32_t coef4_[5], Quartic[5];
  Float32_t coef3_[4], Cubic[4], Best_coef3_[4], Best_Cubic[4];

  /**********************初始化COST****************************/

  /* for 循环执行 */
  Float32_t COST[4][5 * 8][7 * 8] = {0.0F};

  Float32_t Jerk[4][5 * 8][7 * 8] = {0.0F};
  Float32_t V_END[4][5 * 8][7 * 8] = {0.0F};
  Float32_t S_END[4][5 * 8][7 * 8] = {0.0F};
  // Initial minimum
  Float32_t MIN, MAX;
  // Float32_t 0.0F -> Int32_t 0 //PC->ADCU
  Int32_t t_MIN = 0;  // Int32_t 0.0F error
  Int32_t r_MIN = 0;
  Int32_t c_MIN = 0;

  // Float32_t MAXACC[4][5*8][7*8];

  Float32_t Best_T, Best_S, Best_V;

  for (Int32_t Ti = 0; Ti < 1; Ti = Ti + 1) {
    for (Int32_t Si = 0; Si < 5 * 8; Si = Si + 1) {
      for (Int32_t Vi = 0; Vi < 7 * 8; Vi = Vi + 1) {
        // printf("\n*************开始插值[%d][%d]*********************\n\n", Si, Vi);

#if 1  // 5次
        QuinticComputeCoefficients(x0, dx0, ddx0, xf_range[Si], vxf_range[Vi], ddx1, t1[Ti], coef5_);

        QuinticPolynomialCurve1d(coef5_, 0, t1[Ti], Quintic);

        Jerk[Ti][Si][Vi] = fabs(Quintic[3]);

        V_END[Ti][Si][Vi] = fabs(Quintic[1] - set_v);

        S_END[Ti][Si][Vi] = fabs(Quintic[0] - s_diff);
#endif

        /*--------------------------------COST in the two-dimensional array--------------------------------*/

        COST[Ti][Si][Vi] = 0.4 * Jerk[Ti][Si][Vi] + 0.4 * V_END[Ti][Si][Vi] + 0.2 * S_END[Ti][Si][Vi];

        if (COST[Ti][Si][Vi] < COST[t_MIN][r_MIN][c_MIN]) { 
          t_MIN = Ti;
          r_MIN = Si;
          c_MIN = Vi;
        }
      }
    }
  }

  /***********************Find the minimum value ,  row  and column number in COST ****************************/

  MIN = COST[t_MIN][r_MIN][c_MIN];

  printf("[CalcTarVelocityQuinticPolynomialCurve1d] MIN COST = %lf,t_MIN = %d,r_MIN = %d,c_MIN = %d\n", MIN, t_MIN,
         r_MIN, c_MIN);

  Best_T = t1[t_MIN];
  Best_S = xf_range[r_MIN];
  Best_V = vxf_range[c_MIN];

  printf("[CalcTarVelocityQuinticPolynomialCurve1d] Best_T=%lf,Best_S=%lf,Best_V=%lf km/h \n", Best_T, Best_S,
         Best_V * 3.6);

  /***********************Filter the coefficients and derivatives By the minimum COST****************************/
  /*
  //common::CubicPolynomialCurve1d<Float32_t> best_curve(x0,dx0,xf_range[r_MIN],vxf_range[c_MIN], Best_T);
  common::QuinticPolynomialCurve1d<Float32_t> best_curve(x0,dx0,ddx0,xf_range[r_MIN],vxf_range[c_MIN],ddx1, Best_T);

  Float32_t *coef_c = best_curve.getCoefficientl();
  std::cout << " c0cp = " << *coef_c << "," << " c1cp = " << *(++coef_c) << ","
            << " c2cp = " << *(++coef_c) << "," << " c3cp = " << *(++coef_c) << ","
            << " c4cp = " << *(++coef_c) << "," << " c5cp = " << *(++coef_c) << ","<<std::endl;

  Float32_t value0_c = best_curve.Evaluate(0, 0.2);
  Float32_t value1_c = best_curve.Evaluate(1, 0.2);
  Float32_t value2_c = best_curve.Evaluate(2, 0.2);
  std::cout << " value0_cp = " <<  value0_c<< "m" << "," << "value1_cp =" << (value1_c * 3.6)<< "km/h"<< "," <<
  "value2_cp =" <<  value2_c  <<"m/s2" <<std::endl;
  */

  Float32_t best_coef5[6] = {0.0F};
  Float32_t best_Quintic[6] = {0.0F};

  QuinticComputeCoefficients(x0, dx0, ddx0, Best_S, Best_V, ddx1, Best_T, best_coef5);
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] "
            << " c0_ = " << best_coef5[0] << ","
            << " c1_ = " << best_coef5[1] << ","
            << " c2_ = " << best_coef5[2] << ","
            << " c3_ = " << best_coef5[3] << ","
            << " c4_ = " << best_coef5[4] << ","
            << " c5_ = " << best_coef5[5] << "," << std::endl;
#endif


  QuinticPolynomialCurve1d(best_coef5, 0, 0.2, best_Quintic);  // 0.1s不行，0.2s /0.5s /1.0实车效果不大
  Float32_t value0 = best_Quintic[0];
  Float32_t value1 = best_Quintic[1];
  Float32_t value2 = best_Quintic[2];
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
  std::cout << "[CalcTarVelocityQuinticPolynomialCurve1d] <<< END ... OUTPUT : tar_s " << value0 << " m, tar_v "
            << (value1 * 3.6) << "km/h, tar_a = " << value2 << "m/s2" << std::endl;
#endif


  /// FIXME:5、多项式规划的结果prev_tar_v_和prev_tar_a_ //2个障碍物
  // prev_tar_v_ = value1;
  // prev_tar_a_ = value2;
  // std::cout << " prev_tar_v_ = " << prev_tar_v_ <<"m/s" << "," << "prev_tar_a_ = " << prev_tar_a_ << "m/s2"
  // <<std::endl;

#if 0
            CubicComputeCoefficients(x0,dx0,xf_range[r_MIN],vxf_range[c_MIN], Best_T,Best_coef3_);

            for (int Ki = 0; Ki < K + 1; Ki++) {
                    printf("Best_coef_[%d] =%lf\n\n", Ki, Best_coef3_[Ki]);

            }
            CubicPolynomialCurve1d(Best_coef3_, 0, dt, Best_Cubic);

            SpdPln_lTgtLng = Best_Cubic[0];
            SpdPln_vTgtSpd = Best_Cubic[1];
            SpdPln_aTgtAcc = Best_Cubic[2];

            printf("下一周期的位置:SpdPln_lTgtLng= %lf m\n\n", SpdPln_lTgtLng);
            printf("下一周期的速度:SpdPln_vTgtSpd= %lf m/s\n\n", SpdPln_vTgtSpd);
            printf("下一周期的加速度:SpdPln_aTgtAcc= %lf m/s2\n\n", SpdPln_aTgtAcc);

            printf("相对距离:s_diff= %lf m\n\n", s_diff);
#endif

#if 0  // need_control
        bool need_control = false;//？
        if (curr_v > obj_v)//本车速度 > 障碍物速度，需减速
        {
            plan_var_t act_deceleration_time = (obj_v - curr_v) / decl_of_act_cond;//定减速的时间：t=(obj_v - curr_v) / -0.5
            plan_var_t act_deceleration_distance =
                curr_v * act_deceleration_time + 0.5F * decl_of_act_cond * act_deceleration_time * act_deceleration_time;//定减速的距离：s=curr_v*t+1/2*(-0.5)*t^2
            if (s_diff < act_deceleration_distance)
            {
                need_control = true;
            }
            else
            {
                plan_var_t ctl_range = curr_v * time_of_ctl_range;
                if (ctl_range < min_dist_of_ctl_range)
                {
                    ctl_range = min_dist_of_ctl_range;
                }
                if (s_diff < ctl_range)
                {
                    need_control = true;
                }
            }
        }
        else//本车速度 < 障碍物速度，需加速
        {
            plan_var_t act_acceleration_time = (obj_v - curr_v) / accl_of_act_cond;
            plan_var_t act_acceleration_distance =
                curr_v * act_acceleration_time + 0.5F * accl_of_act_cond * act_acceleration_time * act_acceleration_time;

            if (s_diff < act_acceleration_distance)
            {
                need_control = true;
            }
            else
            {
                plan_var_t ctl_range = curr_v * time_of_ctl_range;
                if (ctl_range < min_dist_of_ctl_range)
                {
                    ctl_range = min_dist_of_ctl_range;
                }
                if (s_diff < ctl_range)
                {
                    need_control = true;
                }
            }
        }

        if (need_control)
        {
            if (obj_v < 1.0F / 3.6F)//当obj_v速度较小时
            {
                plan_var_t t     = 0.0F;
                Int32_t    lower = 0;
                if (param.stop_accurately)//配置文件逻辑
                {//精准停车场景，速度限制表的索引lower
                    lower = common::LerpInOrderedTable(param_.velocity_limit_table_for_stop_accurately_situation, s_diff, &t);//输出:插值比例t和索引lower
                    obj_v = common::Lerp(param_.velocity_limit_table_for_stop_accurately_situation[lower].value,
                                        param_.velocity_limit_table_for_stop_accurately_situation[lower + 1].value, t);
                }
                else
                {//停车场景，速度限制表的索引lower
                    lower = common::LerpInOrderedTable(param_.velocity_limit_table_for_stop_situation, s_diff, &t);//输出:插值比例t和索引lower
                    obj_v = common::Lerp(param_.velocity_limit_table_for_stop_situation[lower].value,
                                        param_.velocity_limit_table_for_stop_situation[lower + 1].value, t);
                }
            }

            *tar_v = obj_v;//目标速度：tar_v !=obj_v   //CubicPolynomialCurve1d

            if (*tar_v < 0.1F)
            {
                *tar_v = 0.0F;
            }

            plan_var_t distance = s_diff;
            if (distance < 0.01F)
            {
                distance = 0.01F;
            }

            plan_var_t t = min_deceleration_time;//2s
            if ((curr_v + obj_v) > 0.01F)
            {   
                t = 2.0F * distance / (curr_v + obj_v);//定减速：即在相对距离内，速度由curr_v减速到obj_v，所需要的时间
            }
            if (t < min_deceleration_time)//不能小于min_deceleration_time=2s
            {
                t = min_deceleration_time;
            }
            *tar_a =(obj_v - curr_v) / t;//目标加速度:tar_a<0  //CubicPolynomialCurve1d


            if (obj_v < 0.1F)
            {
                if (curr_v < 0.1F)
                {
                    if (s_diff < 0.3F)
                    {
                        *tar_a = -1.0F;
                    }
                }
            }

            // 当处于精确停车模式时，使用固定的减速度
            if (obj_v < 1.0 / 3.6)
            {
                if (param.stop_accurately)
                {
                    if (s_diff < param_.stop_accurately.braking_distance)//s_diff <0.8m，直接以-3 m/s^2减速度刹死
                    {
                        *tar_a = -param_.stop_accurately.proposed_decelaration;//目标加速度:tar_a=-3 m/s^2
                    }
                }
            }
        }
        return (need_control);
#endif

#if (ENABLE_VELOCITY_PLANNING_DEBUGGING_FILE)
  if (!open_log_file_speed) {
    open_log_file_speed = true;
    log_file_speed.Open();
    log_file_speed.Enable(true);

    std::snprintf(str_buff_, sizeof(str_buff_) - 1,
                  "timestamp,"
                  "curr_v,obj_v,set_v,dist_to_obj,safe_dist,"
                  "prev_tar_v_,prev_tar_a_,tar_v,tar_a;"
                  "\n");
    log_file_speed.Write(str_buff_);
  }

#if 1
  std::snprintf(str_buff_, sizeof(str_buff_) - 1,
                "%ld, "
                "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
                "%ld,%ld,%ld,%ld;"
                "\n",

                common::GetClockNowMs(), curr_v, obj_v, set_v, dist_to_obj, safe_dist, prev_tar_v_, prev_tar_a_, tar_v,
                tar_a);

  log_file_speed.Write(str_buff_);
#endif
}
#endif
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
std::cout << "******************************多目标速度规划(end)****************************** " << std::endl;
#endif
return *tar_v = value1, *tar_a = value2;

#endif

}

void VelocityPlanningFuzzyPID::CalcVelocityToFallback(ActionPlanningResult action_planning_result, 
                                                      plan_var_t curr_v,
                                                      VelocityPlanningResult *vel_planning_result
                                                      ) {
  Float32_t tar_a;
  Float32_t tar_v;

  Float32_t fallback_limit_a_B_II = -2.0;
  Float32_t fallback_limit_a_C_I =  -2.0;
  Float32_t fallback_limit_a_C_II = -4.0;

  if ( REQ_FALLBACK_ACTION_B_II == action_planning_result.fallback_action ) { 
    if (first_B_II_) {
      tar_a = (0 - curr_v) / 11.0;
      if (tar_a < (fallback_limit_a_B_II / 2)) {
        tar_a = fallback_limit_a_B_II / 2;
      }
      tar_v = curr_v + tar_a;
      dec_B_II_ = tar_a;
      first_B_II_ = false;
    } else {
      // TODO : 0.5 or 0.2
      tar_v = curr_v + 0.5 * dec_B_II_;
      tar_a = dec_B_II_;
    }
    vel_planning_result->tar_v = tar_v;
    vel_planning_result->tar_a = tar_a;
    vel_planning_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_FALLBACK;
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
    vel_planning_result->tar_obj.obj_dec_status = obj_jitter_suppression_.status;
    first_C_II_ = true;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "B_FallBack_Stata_II is doing !!" << std::endl;
#endif

  } else if ( REQ_FALLBACK_ACTION_C_I == action_planning_result.fallback_action ) {
    tar_a = fallback_limit_a_C_I / 2.0;
    tar_v = curr_v + 0.2 * tar_a;
    vel_planning_result->tar_v = tar_v;
    vel_planning_result->tar_a = tar_a;
    vel_planning_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_FALLBACK;
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
    vel_planning_result->tar_obj.obj_dec_status = obj_jitter_suppression_.status;
    first_C_II_ = true;
    first_B_II_ = true;
#if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "C_FallBack_Stata_I is doing !!" << std::endl;
#endif
  } else if ( REQ_FALLBACK_ACTION_C_II == action_planning_result.fallback_action ) {
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
    vel_planning_result->tar_type = VELOCITY_PLANNING_TARGET_TYPE_FALLBACK;
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
    vel_planning_result->tar_obj.obj_dec_status = obj_jitter_suppression_.status;
  #if ENABLE_VELOCITY_PLANNING_FUZZY_PID_TRACE
    std::cout << "C_FallBack_Stata_II is doing !!" << std::endl;
  #endif
    first_B_II_ = true;
  } else {
    // nothing
  }
  return;
}

bool VelocityPlanningFuzzyPID::TargetVelocityFromPathCurvature(
    const common::Path &path,
    const common::StaticVector<common::Path::CurveSegment, common::Path::kMaxCurveSegmentNum> &curvature_info,
    const common::PathPoint &proj_on_path, plan_var_t *target_velocity) {

  Int32_t curve_seg_num = curvature_info.Size();
  plan_var_t pre_s_curvature = 5.0;
  plan_var_t abs_curvature = 0.0;
  Int32_t pre_curvature_type = common::Path::TYPE_STRAIGHT;

  for (Int32_t i = 0; i < curve_seg_num; ++i) {
    const common::Path::CurveSegment &curve_seg = curvature_info[i];

    if (common::Path::TYPE_STRAIGHT == curve_seg.type) {
      continue;
    }
    if ((curve_seg.start_s + curve_seg.length) < (proj_on_path.s)) {
      continue;
    }
    
    plan_var_t temp_abs_cur;
    temp_abs_cur = common::com_abs(curve_seg.max_curvature);
    std::cout << "curvature  : " << temp_abs_cur << std::endl;
    abs_curvature = common::Max(temp_abs_cur , abs_curvature);
      
      if (abs_curvature < 0.0005F) {
        pre_curvature_type = common::Path::TYPE_STRAIGHT;
      } else {
        pre_curvature_type = curve_seg.type;
      }
   
   
    /*
    if ((curve_seg.start_s + curve_seg.length) > (proj_on_path.s + pre_s_curvature)) {

      abs_curvature = common::com_abs(curve_seg.max_curvature);
      if (abs_curvature < 0.0005F) {
        pre_curvature_type = common::Path::TYPE_STRAIGHT;
      } else {
        pre_curvature_type = curve_seg.type;
      }
      break;
    }
    */
  }
  



  if (pre_curvature_type != common::Path::TYPE_STRAIGHT) {
    plan_var_t ratio = 0;
    //Int32_t lower = common::LerpInOrderedTable(param_.curvatrue_limit_velocity_table, 1 / abs_curvature, &ratio);
    
    //plan_var_t velocity_limit = param_.curvatrue_limit_velocity_table[lower + 1].value;
     plan_var_t velocity_limit =30/3.6;
    *target_velocity = velocity_limit;
    return true;
  } else {
    return false;
  }
}

void VelocityPlanningFuzzyPID::CalculateTargetVelocityFromTunnel(TargetVelocityUpperLimit *target_vel_result) {
  
  target_vel_result->Clear();
  
  const common::StaticVector<ad_msg::SceneStory, ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM> &story_list =
                                                 driving_map_->GetSceneStoryList();

  Int32_t storys_num = story_list.Size();

  plan_var_t tunnel_velocity_limit = -1.0;

#if ENABLE_VELOCITY_PLANNING_TRACE
  std::cout << "story : num = " << storys_num << std::endl;
#endif

  for (Int32_t i = 0; i < storys_num; ++i) {
    const ad_msg::SceneStory &story = story_list[i];

    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL != story.type) {
      // 不在隧道场景
      continue;
    }
    if (story.area.DistanceToArea() > 300.0F ) {
      // 距离大于300m
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

  return;
} 

void VelocityPlanningFuzzyPID::CubicComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t x1,
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

void VelocityPlanningFuzzyPID::QuinticComputeCoefficients(const Float32_t x0, const Float32_t dx0, const Float32_t ddx0,
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

void VelocityPlanningFuzzyPID::CubicPolynomialCurve1d(const Float32_t *coef3_, const Int32_t order, const Float32_t p,
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

void VelocityPlanningFuzzyPID::QuinticPolynomialCurve1d(const Float32_t *coef5_, const Int32_t order, const Float32_t p,
                                                        Float32_t *Quintic) {
  switch (order) {
    case 0: {
      Quintic[0] = ((((coef5_[5] * p + coef5_[4]) * p + coef5_[3]) * p + coef5_[2]) * p + coef5_[1]) * p + coef5_[0];
    }
    case 1: {
      Quintic[1] = (((5.0 * coef5_[5] * p + 4.0 * coef5_[4]) * p + 3.0 * coef5_[3]) * p + 2.0 * coef5_[2]) * p + coef5_[1];
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

void VelocityPlanningFuzzyPID::linspace(Float32_t x1, Float32_t x2, Int32_t n, Float32_t *y) {
  Float32_t d = (x2 - x1) / (n - 1);

  for (Int32_t i = 0; i < n; i++) {
    y[i] = x1 + i * d;
  }
  return;
}

}  // namespace planning
} // namespace phoenix
