#include "work/work_planning.h"

#include "communication/shared_data.h"
#include "curve/path.h"
#include "msg_planning.h"
#include "utils/macros.h"
#include "utils/gps_tools.h"
#include "geometry/geometry_utils.h"
#include "vehicle_model_wrapper.h"
#include "math/matrix.h"
#include "utils/com_utils.h"

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
#include "ExpWrapper.h"
#include "KTMPUStateOut.h"
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#include "rcar/rcar_job_recv_adasisv2.h"
#endif

// 开启性能测试
#define ENABLE_WORK_PLANNING_PERFORMANCE_TEST (1)
#define PACC_MAP_DEBUG (1)

namespace phoenix {
namespace framework {

/*
    * @author    Add BY xiaranfei
    * @date         20220819
    * @details    规划轨迹点集 WORK_PLANNING PLOT
*/    
#if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
       WorkPlanning::WorkPlanning() : log_file_pointXY("pointXY")
#else
       WorkPlanning::WorkPlanning()
#endif
{
    Initialize();
    #if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
    open_log_file_pointXY = false;
    #endif
}

WorkPlanning::~WorkPlanning() {
}

void WorkPlanning::Initialize() {
  driving_map_.SetDrivingMapImpl(driv_map::GetDrivingMapImpl(0));


  curr_planning_settings_.start_adas = 0;
  curr_planning_settings_.enable_lka = 2;
  curr_planning_settings_.enable_acc = 2;
  curr_planning_settings_.enable_aeb = 2;
  curr_planning_settings_.enable_alc = 1;
  curr_planning_settings_.enable_isl = 1;
  curr_planning_settings_.enable_ngp = 1;
  curr_planning_settings_.enable_fallback = 1;
  curr_planning_settings_.enable_pcc = 1;
  curr_planning_settings_.target_velocity_valid = 1;
  curr_planning_settings_.target_velocity = 10.0F / 3.6F;
  curr_planning_settings_.target_acc_valid = 1;
  curr_planning_settings_.target_acc = 0.0F;
  curr_planning_settings_.target_time_gap_valid = 1;
  curr_planning_settings_.target_time_gap = 3.0F;
  curr_planning_settings_.target_fallback_level_valid = 1;
  curr_planning_settings_.target_fallback_level = 0;
  curr_planning_settings_.changing_lane_req = 0;
  SharedData::instance()->SetCurrentPlanningSettings(curr_planning_settings_);

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  localization_updated_flag_ = false;
  map_updated_flag_ = false;

  odom_coor_changed_flag_ = false;
  delta_heading_odom_coor_ = 0.0F;
  mat_conv_odom_coor_.SetIdentity();
  is_valid_hd_map_ = 1;
#endif

  LOG_INFO(5) << "sizeof(ad_msg::PlanningResult)= "
            << sizeof(ad_msg::PlanningResult) << " bytes";
}

void WorkPlanning::Configurate(const PlanningConfig& conf) {
  config_ = conf;

  pos_filter_.Configurate(conf.pos_filter_config);
  driving_map_.Configurate(conf.driving_map_config);
  action_planner_.Configurate(conf.act_planning_config);
  trajectory_planner_.Configurate(conf.trj_planning_config);
  velocity_planner_.Configurate(conf.vel_planning_config);
}

//多项式拟合的一个函数,返回拟合的参数曲线系数
/// 定义2*2矩阵  typedef Matrix<Float32_t, 2, 2> Matrix2f;
//void static polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, Int32_t  order,double *coeffs) 
#if (ENABLE_OUTPUT_FITTING_COEFFICIENT)
  void static polyfit(common::Matrix<double, 16, 1>  xvals, common::Matrix<double, 16, 1>  yvals, Int32_t  order,double *coeffs) {
    common::Matrix<double, 16, 6> A;

    for (Int32_t i = 0; i < 16; i++) {
        A(i, 0) = 1.0;
    }
    for (Int32_t j = 0; j < 16; j++) {
        for (Int32_t i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    common::Matrix<double, 16, 16> Q;//正交矩阵

    if (false == Mat_HouseholderQR(A,  Q)) {
      LOG_ERR << "The coefficient matrix is invalid.";
    //   return (false);
    }

    common::Matrix<double, 16, 1> result;
    //auto result= Q.solve(yvals);
    // 使用将系数矩阵分解后的QR矩阵求解多项式曲线的系数矩阵
    if (false == Mat_CalcLinearEquationFromQR(
            A, Q, yvals, result)) {
        LOG_ERR << "Failed to evaluate the coefficient a of the polynomial curve.";
        //return false;
    }

    coeffs[0] = result(0);
    coeffs[1] = result(1);
    coeffs[2] = result(2);
    coeffs[3] = result(3);
    coeffs[4] = result(4);
    coeffs[5] =result(5);
  }
#endif

Int32_t WorkPlanning::DoWork() {
  SharedData* shared_data = SharedData::instance();

  Int32_t event_num = 0;
  bool ret = false;
  Int32_t status = PLANNING_MODULE_STATUS_OK;

  LOG_INFO(5) << "Do planning work... (start).";

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_planning;
#endif

  // 当出现内部异常时，保持方向盘不动并释放油门
  planning_result_.hold_steering_wheel = 1;
  planning_result_.release_throttle = 1;

  /// 设置调试数据 (begin)
  // 注意1: 调试时，建议关闭WorkMonitor的调用，避免消息超时清除了SharedData，
  //        造成设置的调试数据被清除掉
  // 注意2: 若调试用例设置了障碍物, 将障碍物模式设置为->使用外部障碍物
  // 注意3: 正常运行时需要屏蔽此函数调用, 并打开WorkMonitor的调用
#if 0
  config_.obj_filter_config.using_outside_obj_list = true;
 //planning_info_debug_.Debug();
#endif

  /// 获取规划任务必要的数据
  // 获取GNSS数据
  shared_data->GetGnss(&gnss_info_);
  // 获取IMU数据
  shared_data->GetImu(&imu_info_);
  // 获取车身数据
  shared_data->GetChassis(&chassis_);
  // 获取车身控制指令数据
  shared_data->GetChassisCtlCmd(&chassis_ctl_cmd_);
  // 获取车辆平台相关的车身数据
  shared_data->GetSpecialChassisInfo(&special_chassis_info_);
  // 获取相机识别的车道线数据
  shared_data->GetLaneMarkCameraList(&lane_mark_camera_list_);
  // 获取相机识别的障碍物信息
  shared_data->GetObstacleCameraList(&obstacle_camera_list_);
  // 获取ESR识别的障碍物信息
  shared_data->GetObstacleRadarFrontList(&obstacle_radar_front_list_);
  shared_data->GetObstacleRadarRearList(&obstacle_radar_rear_list_);
  // 获取Srr2识别的障碍物信息
  shared_data->GetObstacleRadarFrontLeftList(&obstacle_radar_front_left_list_);
  shared_data->GetObstacleRadarFrontRightList(&obstacle_radar_front_right_list_);
  shared_data->GetObstacleRadarRearLeftList(&obstacle_radar_rear_left_list_);
  shared_data->GetObstacleRadarRearRightList(&obstacle_radar_rear_right_list_);
  // 获取Lidar识别的障碍物信息
  shared_data->GetObstacleLidarList0(&obstacle_lidar_list_0_);
  shared_data->GetObstacleLidarList1(&obstacle_lidar_list_1_);
  // 获取交通标志及交通信号数据
  shared_data->GetTrafficSignalList(&traffic_signal_list_);
  shared_data->GetTrafficLightList(&traffic_light_list_);
  // 获取场景任务
  shared_data->GetSceneStoryList(&scene_story_list_);
  // 获取车在地图上的定位信息
  shared_data->GetMapLocalization(&map_localization_);

#if (ENTER_PLAYBACK_MODE_ADHMI)
  // get_adecu_vel_plan_debug
  shared_data->GetAdecuVelplanDebug(&adecu_vel_plan_debug_);
    // adecu_plan_debug
  adecu_plan_debug_.Adecu_VelPlanning_result.obj_id = adecu_vel_plan_debug_.tar_obj.dist_gap_level;
  adecu_plan_debug_.Adecu_VelPlanning_result.tar_type = adecu_vel_plan_debug_.tar_type;
  adecu_plan_debug_.Adecu_VelPlanning_result.tar_v = adecu_vel_plan_debug_.tar_v;
  adecu_plan_debug_.Adecu_VelPlanning_result.tar_a = adecu_vel_plan_debug_.tar_a;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_dist_to_obj = adecu_vel_plan_debug_.tar_obj.dist_to_obj;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_valid = adecu_vel_plan_debug_.tar_obj.valid;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_dec_status = adecu_vel_plan_debug_.tar_obj.obj_dec_status;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_time_gap = adecu_vel_plan_debug_.tar_obj.time_gap;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_relative_v = adecu_vel_plan_debug_.tar_obj.relative_v;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_abs_v = adecu_vel_plan_debug_.tar_obj.obj_v;
  adecu_plan_debug_.Adecu_VelPlanning_result.obs_ttc = adecu_vel_plan_debug_.tar_obj.ttc;
#endif

  // 获取HMI设置项
  GetPlanningSettings();

  // 更新车身参数
  UpdateVehicleParameter();

  // 在ADCU上根据IVI设置来设定是混合地图及相机模式，还是单纯相机模式
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  PlanningConfig planning_conf;
  planning_conf.driving_map_config.inputted_map_type = special_chassis_info_.dfcv_d17.BCM_state; // 0:lane mode 1:map mode 2:mix mode
  Configurate(planning_conf);
#endif

  // 获取地图及导航数据
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
  std::shared_ptr<const apollo::hdmap::Map> ptr_raw_map;
  shared_data->GetRawHDMap(&ptr_raw_map);
  const apollo::hdmap::Map* raw_map = Nullptr_t;
  if (Nullptr_t != ptr_raw_map) {
    raw_map = ptr_raw_map.get();
  }
  std::shared_ptr<const apollo::routing::RoutingResponse> ptr_raw_routing;
  shared_data->GetRawRouting(&ptr_raw_routing);
  const apollo::routing::RoutingResponse* raw_routing = Nullptr_t;
  if (Nullptr_t != ptr_raw_routing) {
    raw_routing = ptr_raw_routing.get();
  }
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  GetMapDataFromEHR();
  shared_data->SetSceneStoryList(scene_story_list_);
#else
  // No map supported
#endif


#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_pos_filter;
#endif
  /// 对当前车辆位置进行平滑滤波
#if (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  // 更新ODOM坐标系
  if (odom_coor_changed_flag_) {
    pos_filter_.UpdateOdomCoordinate(
          mat_conv_odom_coor_, delta_heading_odom_coor_);
  }
#endif
  pos_filter::PosFilterDataSource pos_filter_data_src;
  pos_filter_data_src.timestamp = common::GetClockNowMs();
  /// TODO: ODOM定位数据滤波存在问题，需要调查
  /* k004 pengc 2022-12-26 (begin) */
  // 设置GNSS定位精度不可信任
  pos_filter_data_src.do_not_trust_gnss = true;
  /* k004 pengc 2022-12-26 (begin) */
#if 1
  pos_filter_data_src.gnss = &gnss_info_;
#else
  ad_msg::Gnss tmp_gnss_info = gnss_info_;
  tmp_gnss_info.odom_status = ad_msg::Gnss::STATUS_INVALID;
  pos_filter_data_src.gnss = &tmp_gnss_info;
#endif
  pos_filter_data_src.imu = &imu_info_;
  pos_filter_data_src.chassis = &chassis_;
  pos_filter_data_src.lane_mark_camera_list = &lane_mark_camera_list_;
  ret = pos_filter_.Update(pos_filter_data_src);
  shared_data->SetLaneInfoCameraList(pos_filter_.GetLaneInfoCameraList());
  pos_filter_.GetRelativePosList(&relative_pos_list_);
  shared_data->SetRelativePosList(relative_pos_list_);
  shared_data->SetFilteredGnssInfo(pos_filter_.GetFilteredGnssInfo());
  shared_data->GetFilteredGnssInfo(&filtered_gnss_info_);
  // 位置过滤模块的调试信息
  pos_filter_.GetPosFilterInfo(&pos_filter_info_);
  shared_data->SetPosFilterInfo(pos_filter_info_);
  pos_filter_.GetGnssTrackList(&gnss_track_list_);
  shared_data->SetGnssTrackList(gnss_track_list_);
  pos_filter_.GetGnssFilteredTrackList(&gnss_filtered_track_list_);
  shared_data->SetGnssFilteredTrackList(gnss_filtered_track_list_);
  pos_filter_.GetUtmTrackList(&utm_track_list_);
  shared_data->SetUtmTrackList(utm_track_list_);
  pos_filter_.GetUtmFilteredTrackList(&utm_filtered_track_list_);
  shared_data->SetUtmFilteredTrackList(utm_filtered_track_list_);
  pos_filter_.GetOdomTrackList(&odom_track_list_);
  shared_data->SetOdomTrackList(odom_track_list_);
  pos_filter_.GetOdomFilteredTrackList(&odom_filtered_track_list_);
  shared_data->SetOdomFilteredTrackList(odom_filtered_track_list_);
  // 判断执行结果
  if (false == ret) {
    LOG_ERR << "Failed to filter position.";
    status = PLANNING_MODULE_STATUS_ERR_POS_FILTER_FAULT;
    return (status);
  }
#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Pos Filter spends "
            << timer_pos_filter.Elapsed()
            << "ms.";
  LOG_INFO(5) << "      timestamp="
            << pos_filter_data_src.timestamp;
#endif

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_driving_map_update;
#endif
  /// 构建驾驶地图(行驶区域)
  driv_map::DrivingMapDataSource driving_map_data_source;
  driving_map_data_source.timestamp = common::GetClockNowMs();
  filtered_gnss_info_.gnss_status = gnss_info_.gnss_status;
  filtered_gnss_info_.utm_status = gnss_info_.utm_status;
  filtered_gnss_info_.odom_status = gnss_info_.odom_status;
  // driving_map_data_source.gnss = &filtered_gnss_info_;
  driving_map_data_source.gnss = &gnss_info_;
  driving_map_data_source.rel_pos_list = &relative_pos_list_;
  Int32_t changing_lane_status =
      trajectory_planning_result_.changing_lane_rsp.status();
  if (planning::ChangingLaneRsp::STATUS_CHANGING_LANE_NONE ==
      changing_lane_status) {
    driving_map_data_source.allow_removing_expired_camera_lane = true;
  } else {
    driving_map_data_source.allow_removing_expired_camera_lane = false;
  }
  driving_map_data_source.camera_lane_list =
      &pos_filter_.GetLaneInfoCameraList();
  driving_map_data_source.chassis = &chassis_;
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  driving_map_data_source.gnss_coordinate_type =
      driv_map::GNSS_COORDINATE_TYPE_UTM;
  driving_map_data_source.map = raw_map;
  driving_map_data_source.routing = raw_routing;
#elif HD_MAP_TYPE == HD_MAP_TYPE_D17
  driving_map_data_source.gnss_coordinate_type =
      driv_map::GNSS_COORDINATE_TYPE_ODOM;
  driving_map_data_source.is_vaild_hd_map = is_valid_hd_map_;
  if (hd_map_buffer_.CheckConsistent(0)) {
    // 当前地图有效
    // LOG_INFO(5) << ">>>>>>>>>>> Show new map.";
    driving_map_data_source.map = &hd_map_buffer_.GetCurrMap();
    driving_map_data_source.routing = Nullptr_t;
  } else {
    // 当前地图无效
    driving_map_data_source.map = Nullptr_t;
    driving_map_data_source.routing = Nullptr_t;
  }
#endif
  driving_map_data_source.scene_story_list = &scene_story_list_;
  driving_map_data_source.map_localization = &map_localization_;
  driving_map_data_source.planning_story_list = &planning_story_list_;

  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (start)*/
#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  mpu_t::MpuState mpu_state;
  if (mpu_t::GetMpuState(mpu_state)) {
    if ((mpu_state.MonitorState.MpuGeofenceBits >> 0) & 0x01) {
      driving_map_data_source.is_in_tunnel = (mpu_state.MonitorState.MpuGeofenceBits
                                              >> Int16_t(mpu_t::EMGBInTunnel)) & 0x01;
    } else {
      driving_map_data_source.is_in_tunnel = false;
    }
  } else {
    driving_map_data_source.is_in_tunnel = false;
  }
#endif
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (end)*/

  ret = driving_map_.Update(driving_map_data_source);
  // 判断执行结果
  if (false == ret) {
    LOG_ERR << "Failed to construct driving map.";
    status = PLANNING_MODULE_STATUS_ERR_DRIVING_MAP_FAULT;
    return (status);
  }
#if 0
  /* k004 pengc 2022-12-26 (begin) */
  // 使用相机车道线矫正地图定位
  pos_filter_.CorrectGnssCoordinate(
        3, driving_map_.GetDeltaPosOfCorrectedGnss());
  /* k004 pengc 2022-12-26 (end) */
#endif

#if 0
  /// Debug
  static common::PathPoint s_prev_porj_point;
  common::PathPoint cur_proj_point = driving_map_.GetProjPointOnMajorRefLine();
  Float32_t angle_diff = common::AngleDiff(
        s_prev_porj_point.heading, cur_proj_point.heading);
  if (common::com_abs(angle_diff) > common::com_deg2rad(5.0F)) {
    const exp_map_t::stMapExpProfileInfo& map = hd_map_buffer_.GetCurrMap();
    const exp_map_t::stMapExpLocation& loc = hd_map_buffer_.GetCurrLocalization();

    printf("\n\n#####################################################\n");
    printf("Detected unexpected localization changing (angle_diff=%f) !!!\n",
           common::com_rad2deg(angle_diff));
    printf("Previous heading between vehicle and lane is %f deg"
           ", current heading between vehicle and lane is %f deg"
           ", different angle is %f deg\n",
           common::com_rad2deg(s_prev_porj_point.heading),
           common::com_rad2deg(cur_proj_point.heading),
           common::com_rad2deg(angle_diff));
    printf("time=%ld"
           ", map_t = %ld, loc_t = %ld"
           ", x = %0.3f, y = %0.3f, heading = %0.3fdeg"
           "\n",
           common::GetClockNowMs(),
           map.profile_datum_info.m_timestamp,
           loc.location_info.s_AbsolutePosition.m_datumTimestamp,
           gnss_info_.x_odom, gnss_info_.y_odom,
           common::com_rad2deg(gnss_info_.heading_odom)
           );
    printf("#####################################################\n\n");
  }
  s_prev_porj_point = cur_proj_point;
  /// Debug
#endif

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Creating Driving Map (Lane) spends "
            << timer_driving_map_update.Elapsed()
            << "ms.";
  LOG_INFO(5) << "      timestamp="
            << driving_map_data_source.timestamp;
#endif

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_obj_filter;
#endif
  /// 添加障碍物
  shared_data->GetOutsideObstacleList(&obstacle_list_);
  shared_data->SetObstacleList(obstacle_list_);


#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_driving_map_update_obj;
#endif
  /// 更新驾驶地图中的障碍物信息
  ret = driving_map_.UpdateObstacleList(&obstacle_list_);
  // 判断执行结果
  if (false == ret) {
    LOG_ERR << "Failed to put obstacle list to driving map.";
    status = PLANNING_MODULE_STATUS_ERR_DRIVING_MAP_FAULT;
    return status;
  }
#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Creating Driving Map (Obstacles) spends "
            << timer_driving_map_update_obj.Elapsed()
            << "ms.";
#endif

  // 保存事件通知信息(驾驶地图)
  event_num = driving_map_.GetEventReporting(10, event_reporting_list_);
  if (event_num > 0) {
    shared_data->AddEventReporting(event_num, event_reporting_list_);
  }

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_action_planning;
#endif
  /// 行为规划
  planning::ActionPlanningDataSource action_planning_data_source;
  action_planning_data_source.timestamp = common::GetClockNowMs();
  action_planning_data_source.driving_map = &driving_map_;
  action_planning_data_source.chassis = &chassis_;
  action_planning_data_source.special_chassis_info = &special_chassis_info_;
  action_planning_data_source.planning_settings = &curr_planning_settings_;
  action_planning_data_source.traffic_signal_list = &traffic_signal_list_;
  action_planning_data_source.changing_lane_req_from_trj_planning =
      &(trajectory_planning_result_.changing_lane_req);
  action_planning_data_source.changing_lane_rsp =
      &(trajectory_planning_result_.changing_lane_rsp);
  action_planning_data_source.result_of_velocity_planning =
      &(velocity_planning_result_);
  ret = action_planner_.Plan(action_planning_data_source);
  action_planning_result_ = action_planner_.GetResultOfPlanning();
  shared_data->SetActionPlanningResult(action_planning_result_);
  // 判断执行结果
  if (false == ret) {
    LOG_ERR << "Failed to plan action.";
    status = PLANNING_MODULE_STATUS_ERR_ACTION_PLANNING_FAULT;
    return status;
  }
  // 保存事件通知信息(行为规划)
  event_num = action_planner_.GetEventReporting(10, event_reporting_list_);
  if (event_num > 0) {
    shared_data->AddEventReporting(event_num, event_reporting_list_);
  }
#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Action Planning spends "
            << timer_action_planning.Elapsed()
            << "ms.";
  LOG_INFO(5) << "      timestamp="
            << action_planning_data_source.timestamp;
#endif

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_trajectory_planning;
#endif
  /// 轨迹规划
  planning::TrajectoryPlanningDataSource trajectory_planning_data_source;
  trajectory_planning_data_source.timestamp = common::GetClockNowMs();
  trajectory_planning_data_source.driving_map = &driving_map_;
  trajectory_planning_data_source.result_of_action_planning =
      &(action_planning_result_);
  trajectory_planning_data_source.chassis = &chassis_;
  ret = trajectory_planner_.Plan(trajectory_planning_data_source);
  trajectory_planning_result_ = trajectory_planner_.GetResultOfPlanning();
  shared_data->SetTrajectoryPlanningResult(trajectory_planning_result_);
  // 判断执行结果
  if (false == ret) {
    LOG_ERR << "Failed to plan trajectory.";
    status = PLANNING_MODULE_STATUS_ERR_TRAJECTORY_PLANNING_FAULT;
    return status;
  }
  // 保存事件通知信息(轨迹规划)
  event_num = trajectory_planner_.GetEventReporting(10, event_reporting_list_);
  if (event_num > 0) {
    shared_data->AddEventReporting(event_num, event_reporting_list_);
  }
#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Trajectory Planning spends "
            << timer_trajectory_planning.Elapsed()
            << "ms.";
  LOG_INFO(5) << "      timestamp="
            << trajectory_planning_data_source.timestamp;
#endif

#ifdef ENABLE_PACC_ADASIS

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  RCAR_Job_RecvADASISv2_DoJob();
#endif

  // 构建当前ADASIS v2 Horizon
  // TODO : 放在Planning任务还是独立的任务?
  adasisv2::PositionMessage position;
  shared_data->GetADASISV2Position(&position);
  adasisv2::ProfileShortMessage profile_short;
  shared_data->GetADASISV2ProfileShort(&profile_short);

  av2HR_.Construct(position, profile_short);
#endif

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  common::Stopwatch timer_velocity_planning;
#endif
  /// 速度规划
  planning::VelocityPlanningDataSource velocity_planning_data_source;
  velocity_planning_data_source.timestamp = common::GetClockNowMs();
  velocity_planning_data_source.driving_map = &driving_map_;
  velocity_planning_data_source.result_of_action_planning =
      &(action_planning_result_);
  velocity_planning_data_source.result_of_trajectory_planning =
      &(trajectory_planning_result_);
  velocity_planning_data_source.chassis = &chassis_;
  velocity_planning_data_source.special_chassis = &special_chassis_info_;
  velocity_planning_data_source.chassis_ctl_cmd = &chassis_ctl_cmd_;
  velocity_planning_data_source.traffic_light_list = &traffic_light_list_;
  velocity_planning_data_source.traffic_signal_list = &traffic_signal_list_;
#if (ENABLE_ROS_NODE)
//  velocity_planning_data_source.debug_msg = &planning_debug_;
#endif

#ifdef ENABLE_PACC_ADASIS
  velocity_planning_data_source.adasisv2_horizon = av2HR_.GetHorizon();
#endif

  ret = velocity_planner_.Plan(velocity_planning_data_source);
  velocity_planning_result_ = velocity_planner_.GetResultOfPlanning();
  shared_data->SetVelocityPlanningResult(velocity_planning_result_);

  velocity_planning_internal_ = velocity_planner_.GetInternalOfPlanningDebug();
  shared_data->SetVelocityPlanningInternal(velocity_planning_internal_);

  // 判断执行结果
  if (false == ret) {
    LOG_ERR << "Failed to plan velocity.";
    status = PLANNING_MODULE_STATUS_ERR_VELOCITY_PLANNING_FAULT;
    return status;
  }
  // 保存事件通知信息(速度规划)
  event_num = velocity_planner_.GetEventReporting(10, event_reporting_list_);
  if (event_num > 0) {
    shared_data->AddEventReporting(event_num, event_reporting_list_);
  }
#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Velocity Planning spends "
            << timer_velocity_planning.Elapsed()
            << "ms.";
  LOG_INFO(5) << "      timestamp="
            << velocity_planning_data_source.timestamp;
  LOG_INFO(5) << "      trj_timestamp="
            << trajectory_planning_result_.msg_head.timestamp;
#endif

  /// 更新规划结果
  SetPlanningResult();

    /// 保存规划结果
  shared_data->SetPlanningResult(planning_result_);
    
    /// ROS debug Add BY ZQ 
#if (ENABLE_ROS_NODE)
  // 车道线输入数据
  planning_debug_.lane_input.lane_mark_num = lane_mark_camera_list_.lane_mark_num;
  planning_debug_.lane_input.lane_camera.clear();
  planning_debug_.lane_input.lane_camera.reserve(lane_mark_camera_list_.lane_mark_num);
  ::planning::laneline_info lanelineInfo;
  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {
      lanelineInfo.lane_id = lane_mark_camera_list_.lane_marks[i].id;
      lanelineInfo.c0 = lane_mark_camera_list_.lane_marks[i].c0;
      lanelineInfo.c1 = lane_mark_camera_list_.lane_marks[i].c1;
      lanelineInfo.c2 = lane_mark_camera_list_.lane_marks[i].c2;
      lanelineInfo.c3 = lane_mark_camera_list_.lane_marks[i].c3;
      planning_debug_.lane_input.lane_camera.push_back(lanelineInfo);
  }

  // 障碍物输入数据
  planning_debug_.obs_input.obstacle_num = obstacle_list_.obstacle_num;

  planning_debug_.obs_input.obstacle_fusion.clear();
  planning_debug_.obs_input.obstacle_fusion.reserve(obstacle_list_.obstacle_num);
  ::planning::obstacle_info obstacleInfo;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
      obstacleInfo.type = obstacle_list_.obstacles[i].type;
      obstacleInfo.dynamic = obstacle_list_.obstacles[i].dynamic;
      obstacleInfo.confidence = obstacle_list_.obstacles[i].confidence;
      obstacleInfo.v_x = obstacle_list_.obstacles[i].v_x;
      obstacleInfo.a = obstacle_list_.obstacles[i].a;
      planning_debug_.obs_input.obstacle_fusion.push_back(obstacleInfo);
  }

  // 行为规划结果输出
  planning_debug_.ActionPlanning_result.enable_adas = action_planning_result_.enable_adas;
  planning_debug_.ActionPlanning_result.enable_acc = action_planning_result_.enable_acc;
  planning_debug_.ActionPlanning_result.enable_aeb = action_planning_result_.enable_aeb;
  planning_debug_.ActionPlanning_result.enable_lka = action_planning_result_.enable_lka;
  planning_debug_.ActionPlanning_result.tar_v_cc = action_planning_result_.v_setting;

  // 路径规划结果输出
  planning_debug_.TrajPlanning_result.planning_change_Rsp_status = trajectory_planning_result_.changing_lane_rsp.status();
  planning_debug_.TrajPlanning_result.planning_change_Rsp_response = trajectory_planning_result_.changing_lane_rsp.response();
  planning_debug_.TrajPlanning_result.planning_change_lat_error = planning_result_.tar_trj.lat_err.samples[0].lat_err;
  #if (ENABLE_OUTPUT_FITTING_COEFFICIENT)
    planning_debug_.TrajPlanning_result.PlanningTraj_IsValid= planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;
    planning_debug_.TrajPlanning_result.coeffs_0 = planning_result_.tar_trj.PlanningTraj_Cv.coeffs[0];
    planning_debug_.TrajPlanning_result.coeffs_1 = planning_result_.tar_trj.PlanningTraj_Cv.coeffs[1];
    planning_debug_.TrajPlanning_result.coeffs_2 = planning_result_.tar_trj.PlanningTraj_Cv.coeffs[2];
    planning_debug_.TrajPlanning_result.coeffs_3 = planning_result_.tar_trj.PlanningTraj_Cv.coeffs[3];
    planning_debug_.TrajPlanning_result.coeffs_4 = planning_result_.tar_trj.PlanningTraj_Cv.coeffs[4];
    planning_debug_.TrajPlanning_result.coeffs_5 = planning_result_.tar_trj.PlanningTraj_Cv.coeffs[5];
  #endif

#if (ENABLE_BACK_TO_BACK_TEST)
  planning::VelocityPlanningResult velocity_planning_result_org = velocity_planner_.GetResultOfPlanningOrg();
  planning_debug_.VelPlanning.compare.tar_type = velocity_planning_result_org.tar_type;
  planning_debug_.VelPlanning.compare.tar_v = velocity_planning_result_org.tar_v;
  planning_debug_.VelPlanning.compare.tar_a = velocity_planning_result_org.tar_a;
  planning_debug_.VelPlanning.compare.release_throttle = velocity_planning_result_org.release_throttle;
  planning_debug_.VelPlanning.compare.aeb_warning = velocity_planning_result_org.aeb_warning;
  planning_debug_.VelPlanning.compare.aeb_action = velocity_planning_result_org.aeb_action;
  planning_debug_.VelPlanning.compare.obs_valid = velocity_planning_result_org.tar_obj.valid;
  planning_debug_.VelPlanning.compare.obs_dec_status = velocity_planning_result_org.tar_obj.obj_dec_status;
  planning_debug_.VelPlanning.compare.obs_dist_to_obj = velocity_planning_result_org.tar_obj.dist_to_obj;
  planning_debug_.VelPlanning.compare.obs_time_gap = velocity_planning_result_org.tar_obj.time_gap;
  planning_debug_.VelPlanning.compare.obs_relative_v = velocity_planning_result_org.tar_obj.relative_v;
  planning_debug_.VelPlanning.compare.obs_abs_v = velocity_planning_result_org.tar_obj.obj_v;
  planning_debug_.VelPlanning.compare.obs_ttc = velocity_planning_result_org.tar_obj.ttc;
  planning_debug_.VelPlanning.compare.obj_id  = velocity_planning_result_org.tar_obj.dist_gap_level;
#endif

  // 速度规划结果输出
  planning_debug_.VelPlanning.output.tar_type = velocity_planning_result_.tar_type;
  planning_debug_.VelPlanning.output.tar_v = velocity_planning_result_.tar_v;
  planning_debug_.VelPlanning.output.tar_a = velocity_planning_result_.tar_a;
  planning_debug_.VelPlanning.output.release_throttle = velocity_planning_result_.release_throttle;
  planning_debug_.VelPlanning.output.tar_throttle = velocity_planning_result_.tar_throttle;
  planning_debug_.VelPlanning.output.aeb_warning = velocity_planning_result_.aeb_warning;
  planning_debug_.VelPlanning.output.aeb_action = velocity_planning_result_.aeb_action;
  planning_debug_.VelPlanning.output.obs_valid = velocity_planning_result_.tar_obj.valid;
  planning_debug_.VelPlanning.output.obs_dec_status = velocity_planning_result_.tar_obj.obj_dec_status;
  planning_debug_.VelPlanning.output.obs_dist_to_obj = velocity_planning_result_.tar_obj.dist_to_obj;
  planning_debug_.VelPlanning.output.obs_time_gap = velocity_planning_result_.tar_obj.time_gap;
  planning_debug_.VelPlanning.output.obs_relative_v = velocity_planning_result_.tar_obj.relative_v;
  planning_debug_.VelPlanning.output.obs_abs_v = velocity_planning_result_.tar_obj.obj_v;
  planning_debug_.VelPlanning.output.obs_ttc = velocity_planning_result_.tar_obj.ttc;
  planning_debug_.VelPlanning.output.obj_id  = velocity_planning_result_.tar_obj.dist_gap_level;

  if (velocity_planning_internal_.pacc.map_input.pacc_points_source == 1) {
    planning_debug_.VelPlanning.output.ADASV2_cur_slope = velocity_planning_internal_.pacc.map_input.cur_slope;
  } else {
    planning_debug_.VelPlanning.output.ADASV2_cur_slope = 0.0;
  }

  if (!trajectory_planning_result_.target_trajectory_slope.Empty()) {
    planning_debug_.VelPlanning.output.mpu_cur_slope = trajectory_planning_result_.target_trajectory_slope.Front().slope;
  } else {
    planning_debug_.VelPlanning.output.mpu_cur_slope = 0.0F;
  }

  SetVelocityPlanningInternalDebug(velocity_planning_internal_);
#endif 


#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ Planning result: "
             << "      plan_ret_timestamp= "
            << planning_result_.msg_head.timestamp
            << "      trj_timestamp= "
            << trajectory_planning_result_.msg_head.timestamp
            << "      trj_timestamp_out= "
            << planning_result_.tar_trj.timestamp;
#endif

  /// 保存调试信息
  // 驾驶地图调试信息
  driving_map_.GetDrivingMapInfo(&driving_map_info_);
  shared_data->SetDrivingMapInfo(driving_map_info_);
  // 轨迹规划调试信息
  trajectory_planner_.GetTrajectoryPlanningInfo(&trj_planning_info_);
  shared_data->SetTrajectoryPlanningInfo(trj_planning_info_);

#if (ENABLE_WORK_PLANNING_PERFORMANCE_TEST)
  LOG_INFO(5) << "#>>>@ All of the Planning spends "
            << timer_planning.Elapsed()
            << "ms.\n\n";
#endif
  LOG_INFO(5) << "Do planning work... (end).";
  return (status);
}

void WorkPlanning::UpdateVehicleParameter() {
  veh_model::VehicleModelWrapper veh_model;

  if (chassis_.param.vehicle_length_valid) {
    veh_model::VehicleModelWrapper::Scalar value = veh_model.GetVehicleLength();
    if ((value < (chassis_.param.vehicle_length-0.01F)) ||
        (value > (chassis_.param.vehicle_length+0.01F))) {
      veh_model.SetVehicleLength(chassis_.param.vehicle_length);
    }
  }
  if (chassis_.param.vehicle_width_valid) {
    veh_model::VehicleModelWrapper::Scalar value = veh_model.GetVehicleWidth();
    if ((value < (chassis_.param.vehicle_width-0.01F)) ||
        (value > (chassis_.param.vehicle_width+0.01F))) {
      veh_model.SetVehicleWidth(chassis_.param.vehicle_width);
    }
  }
  if (chassis_.param.vehicle_height_valid) {
    veh_model::VehicleModelWrapper::Scalar value = veh_model.GetVehicleHeight();
    if ((value < (chassis_.param.vehicle_height-0.01F)) ||
        (value > (chassis_.param.vehicle_height+0.01F))) {
      veh_model.SetVehicleHeight(chassis_.param.vehicle_height);
    }
  }

  if (chassis_.param.trailer_length_valid) {
    veh_model::VehicleModelWrapper::Scalar value = veh_model.GetTrailerLength();
    if ((value < (chassis_.param.trailer_length-0.01F)) ||
        (value > (chassis_.param.trailer_length+0.01F))) {
      veh_model.SetTrailerLength(chassis_.param.trailer_length);
    }
  }
  if (chassis_.param.trailer_width_valid) {
    veh_model::VehicleModelWrapper::Scalar value = veh_model.GetTrailerWidth();
    if ((value < (chassis_.param.trailer_width-0.01F)) ||
        (value > (chassis_.param.trailer_width+0.01F))) {
      veh_model.SetTrailerWidth(chassis_.param.trailer_width);
    }
  }
  if (chassis_.param.trailer_height_valid) {
    veh_model::VehicleModelWrapper::Scalar value = veh_model.GetTrailerHeight();
    if ((value < (chassis_.param.trailer_height-0.01F)) ||
        (value > (chassis_.param.trailer_height+0.01F))) {
      veh_model.SetTrailerHeight(chassis_.param.trailer_height);
    }
  }
}

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
void WorkPlanning::GetMapDataFromEHR() {
  LOG_INFO(5) << "### Reading hd-map from EHR ### (begin)";

  localization_updated_flag_ = false;
  map_updated_flag_ = false;
  odom_coor_changed_flag_ = false;
  delta_heading_odom_coor_ = 0.0F;
  mat_conv_odom_coor_.SetIdentity();

  // 获取定位信息
  if (!exp_map_t::ExpGetLocation(&hd_map_buffer_.GetNextLocalization())) {
    LOG_INFO(3) << "Failed to get localization from EHR.";
    return;
  }

  // 监控定位是否更新了, 若更新了, 更新GNSS了, 并且通知monitor
  if (hd_map_buffer_.IsLocationChanged()) {
    localization_updated_flag_ = true;

    const exp_map_t::stMapExpLocation& location =
        hd_map_buffer_.GetNextLocalization();
    /// TODO: Get corrected status.
    switch (location.location_level) {
    case (exp_map_t::ExpPositionLevel::exp_position_level_null):
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_INVALID;
      break;
    case (exp_map_t::ExpPositionLevel::exp_position_level_1):
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (exp_map_t::ExpPositionLevel::exp_position_level_2):
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_BAD;
      break;
    case (exp_map_t::ExpPositionLevel::exp_position_level_3):
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_CONVERGING;
      break;
    case (exp_map_t::ExpPositionLevel::exp_position_level_4):
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    case (exp_map_t::ExpPositionLevel::exp_position_level_5):
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_GOOD;
      break;
    default:
      gnss_info_.odom_status = ad_msg::Gnss::STATUS_BAD;
      break;
    }
    gnss_info_.x_odom = location.location_info.s_AbsolutePosition.m_vehiclePosition.m_x;
    gnss_info_.y_odom = location.location_info.s_AbsolutePosition.m_vehiclePosition.m_y;
    gnss_info_.z_odom = location.location_info.s_AbsolutePosition.m_vehiclePosition.m_z;
    gnss_info_.heading_odom = common::NormalizeAngle(location.location_info.s_AbsolutePosition.m_heading);
    gnss_info_.pitch = common::NormalizeAngle(location.sensor_info.dpitch);
    gnss_info_.pitch_variance = location.sensor_info.dpitchacc;

    // Update message head
    gnss_info_.msg_head.valid = true;
    gnss_info_.msg_head.UpdateSequenceNum();
    gnss_info_.msg_head.timestamp = common::GetClockNowMs();

#if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
    if (true) {
      if (!log_file_localization_.IsOpen()) {
        log_file_localization_.SetModuleName("localization");
        log_file_localization_.Open();
        log_file_localization_.Enable(true);

        std::snprintf(log_file_buff_, sizeof(log_file_buff_)-1,
                      "timestamp,map_t,loc_t,x,y,heading"
                      "\n");
        log_file_localization_.Write(log_file_buff_);
      }
      const exp_map_t::stMapExpProfileInfo& map = hd_map_buffer_.GetNextMap();
      std::snprintf(log_file_buff_, sizeof(log_file_buff_)-1,
                    "%ld"
                    ",%ld,%ld"
                    ",%0.3f,%0.3f,%0.3f"
                    "\n",
                    common::GetClockNowMs(),
                    map.profile_datum_info.m_timestamp,
                    location.location_info.s_AbsolutePosition.m_datumTimestamp,
                    gnss_info_.x_odom, gnss_info_.y_odom,
                    common::com_rad2deg(gnss_info_.heading_odom)
                    );
      log_file_localization_.Write(log_file_buff_);
    }
#endif

    LOG_INFO(3) << ">>>>>> Localization is changed.";
  }

  // 获取地图信息
  if (!exp_map_t::ExpGetProfile(&hd_map_buffer_.GetNextMap())) {
    LOG_INFO(3) << "Failed to get hd-map from EHR.";
    return;
  }

  if (!hd_map_buffer_.CheckConsistent(1)) {
    // 新地图无效
    const exp_map_t::stMapExpProfileInfo& map = hd_map_buffer_.GetNextMap();
    const exp_map_t::stMapExpLocation& location =
        hd_map_buffer_.GetNextLocalization();
    LOG_INFO(3) << "Localization is not consistent with hd-map in new buffer"
                   ", map=" << map.profile_datum_info.m_timestamp
                << ", loc=" << location.location_info.s_AbsolutePosition.m_datumTimestamp
                << " .";
    return;
  }

  // 新地图有效

  if (hd_map_buffer_.CheckConsistent(0)) {
    // 新地图有效且旧地图也有效

    // 判断坐标系是否更新了, 若更新了, 设置 odom_coor_changed 标志
    if (hd_map_buffer_.IsCoorChanged()) {
      odom_coor_changed_flag_ = true;

      const exp_map_t::stMapExpProfileInfo& map =
          hd_map_buffer_.GetNextMap();
      common::Translate_2D(-map.profile_datum_info.m_point.m_x,
                           -map.profile_datum_info.m_point.m_y,
                           &mat_conv_odom_coor_);
      delta_heading_odom_coor_ = 0.0F;

      LOG_INFO(3) << ">>>>>> Coordinate is changed. >>>>>>>";
    }
  } else {
    LOG_INFO(3) << "Localization is not consistent with hd-map in cur buffer.";
  }

  // 监控地图是否更新了, 若更新了, 通知monitor
  if (hd_map_buffer_.IsMapChanged()) {
    map_updated_flag_ = true;

    /// TODO: Get corrected changled flags.

    LOG_INFO(3) << ">>>>>> Hd-map is changed.";
  }

  // 新地图有效，切换到新地图
  hd_map_buffer_.SwapBuffer();

  /* k007 pengc 2023-03-10 (begin) */
  // 读取地理围栏信息
  Int32_t story_list_idx = 0;
  //scene_story_list_.Clear();
 scene_story_list_.story_num = story_list_idx;
 
  exp_map_t::stMapGeofenceInfo geofence_info;
  if (!exp_map_t::ExpGetGeofence(&geofence_info)) {
    LOG_INFO(3) << "Failed to get geofence from EHR.";
  } else {
    // 在围栏内部
    // 在弯道中
    std::map<exp_map_t::Reason, uint32_t>::iterator it_curve =
        geofence_info.m_currentInReasons.find(exp_map_t::Reason::CURVATUREOVERRUN);
    if (it_curve != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! In curve.";
      }
    }
    // 在隧道内
    std::map<exp_map_t::Reason, uint32_t>::iterator it_tunnel =
        geofence_info.m_currentInReasons.find(exp_map_t::Reason::TUNNEL);
    if (it_tunnel != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! In tunnel.";
      }
    }
    // 驶入匝道
    std::map<exp_map_t::Reason, uint32_t>::iterator it_ramp =
        geofence_info.m_currentInReasons.find(exp_map_t::Reason::RAMP);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! In ramp.";
      }
    }
    // 下匝道口(无论规划是否下匝道)
    it_ramp = geofence_info.m_currentInReasons.find(exp_map_t::Reason::RampArea);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! Near ramp (down).";
      }
    }
    // 上匝道口(无论规划是否上匝道)
    it_ramp = geofence_info.m_currentInReasons.find(exp_map_t::Reason::UpRampArea);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! Near ramp (up).";
      }
    }
    // 分流口
    it_ramp = geofence_info.m_currentInReasons.find(exp_map_t::Reason::RoadDivergence);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_ROAD_RIVER;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! In ROAD RIVER.";
      }
    }
    // 汇流点
    it_ramp = geofence_info.m_currentInReasons.find(exp_map_t::Reason::RoadConfluence);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_ROAD_RIVER;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! In ROAD RIVER.";
      }
    }



    // 规划路径匝道
    it_ramp = geofence_info.m_currentInReasons.find(exp_map_t::Reason::PlanPathRampArea);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! Near ramp (down), dist=";
      }
    }
  
#if 0
    it_ramp = geofence_info.m_currentInReasons.find(exp_map_t::Reason::PlanPathUpRampArea);
    if (it_ramp != geofence_info.m_currentInReasons.end()) {
      if (story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = 0.0F;

        LOG_INFO(5) << "!!!!!!!!!! In ROAD RIVER.";
      }
    }
  #endif
  
    // 接近围栏
    std::vector<exp_map_t::Area>::iterator it_geo_other =
        geofence_info.m_reasonArea.begin();
    std::vector<exp_map_t::Area>::iterator it_geo_other_end =
        geofence_info.m_reasonArea.end();
    for (; it_geo_other != it_geo_other_end; ++it_geo_other) {
      if (story_list_idx >= ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM) {
        break;
      }

      // 接近弯道
      if (exp_map_t::Reason::CURVATUREOVERRUN == it_geo_other->m_reason) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! Near curve, dist=" <<
                       story.area.area_type_01.distance;
      }

      // 接近隧道
      if (exp_map_t::Reason::TUNNEL == it_geo_other->m_reason) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! Near tunnel, dist=" <<
                       story.area.area_type_01.distance;
      }

        if (exp_map_t::Reason::PlanPathRampArea == it_geo_other->m_reason) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! Near ramp (down), dist=" <<
                       story.area.area_type_01.distance;
      }
      // 驶入匝道
      if (exp_map_t::Reason::RAMP == it_geo_other->m_reason) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! In ramp, dist=" <<
                       story.area.area_type_01.distance;
      }
      // 下匝道口(无论规划是否下匝道)
      if (exp_map_t::Reason::RampArea == it_geo_other->m_reason) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! Near ramp (down), dist=" <<
                       story.area.area_type_01.distance;
      }
      // 上匝道口(无论规划是否上匝道)
      if (exp_map_t::Reason::UpRampArea == it_geo_other->m_reason) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_PASSING_RAMP;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! Near ramp (up), dist=" <<
                       story.area.area_type_01.distance;
      }
      // 河流口
      if ((exp_map_t::Reason::RoadDivergence == it_geo_other->m_reason) ||
          (exp_map_t::Reason::RoadConfluence == it_geo_other->m_reason)) {
        ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];
        story_list_idx++;

        story.type = ad_msg::SCENE_STORY_TYPE_CLOSE_TO_ROAD_RIVER;
        story.area.area_type = ad_msg::SceneStoryArea::AREA_TYPE_01;
        story.area.area_type_01.valid = true;
        story.area.area_type_01.distance = it_geo_other->m_offset*0.01F;

        LOG_INFO(5) << "!!!!!!!!!! Close tO road river, dist=" <<
                       story.area.area_type_01.distance;
      }

    }

    scene_story_list_.story_num = story_list_idx;
  }

  exp_map_t::stMapModelInfo mapmode_info;
  if (!exp_map_t::ExpGetMapModelInfo(mapmode_info)) {
     LOG_INFO(3) << "Failed to get mapmode_info from EHR.";
  } else {
    Int32_t map_info = mapmode_info.m_MapModelInfo;
    is_valid_hd_map_ = mapmode_info.m_GeofenceIF;
   
    if ((story_list_idx < ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM)
        && (map_info)) {
      ad_msg::SceneStory& story = scene_story_list_.storys[story_list_idx];

      switch (map_info) {
      case (0x00):
        break;
      case (0x01):
        story.type = ad_msg::SCENE_STORY_TYPE_SWITCH_TO_MAP;
        story_list_idx++;
        scene_story_list_.story_num = story_list_idx;
        break;
      case (0x02):
        story.type = ad_msg::SCENE_STORY_TYPE_SWITCH_TO_CAM_LANE;
        story_list_idx++;
        scene_story_list_.story_num = story_list_idx;
      default:
        break;
      }
    }
  }
  scene_story_list_.msg_head.valid = true;
  scene_story_list_.msg_head.UpdateSequenceNum();
  scene_story_list_.msg_head.timestamp = common::GetClockNowMs();
  /* k007 pengc 2023-03-10 (end) */

  LOG_INFO(5) << "### Reading hd-map from EHR ### (end)";
}
#endif

void WorkPlanning::SetPlanningResult() {
  planning_result_.msg_head.valid = true;
  planning_result_.msg_head.timestamp = common::GetClockNowMs();
  planning_result_.msg_head.UpdateSequenceNum();

  SetPlanningStatus();

  planning_result_.tar_driving_mode = action_planning_result_.driving_mode;
  if (action_planning_result_.enable_lka) {
    planning_result_.enable_eps = true;
  } else {
    planning_result_.enable_eps = false;
  }
  if (action_planning_result_.enable_acc) {
    planning_result_.enable_throttle_sys = true;
    planning_result_.enable_ebs = true;
  } else {
    //planning_result_.enable_throttle_sys = false;
    planning_result_.enable_throttle_sys = true;
    planning_result_.enable_ebs = false;
  }
  if (action_planning_result_.enable_aeb) {
    planning_result_.enable_ebs = true;
  }

  planning_result_.hold_steering_wheel =
      trajectory_planning_result_.hold_steering_wheel;
  if (trajectory_planning_result_.hold_steering_wheel ||
      velocity_planning_result_.release_throttle) {
    planning_result_.release_throttle = 1;
  } else {
    planning_result_.release_throttle = 0;
  }
  planning_result_.steering_wheel_speed =
      trajectory_planning_result_.steering_wheel_speed;

  planning_result_.tar_gear = action_planning_result_.gear;
  switch (action_planning_result_.turn_lamp) {
  case (ad_msg::VEH_TURN_LAMP_OFF) :
    planning_result_.tar_turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
    break;
  case (ad_msg::VEH_TURN_LAMP_LEFT) :
    planning_result_.tar_turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (ad_msg::VEH_TURN_LAMP_RIGHT) :
    planning_result_.tar_turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (ad_msg::VEH_TURN_LAMP_EMERGENCY) :
    planning_result_.tar_turn_lamp = ad_msg::VEH_TURN_LAMP_EMERGENCY;
    break;
  default:
    planning_result_.tar_turn_lamp = ad_msg::VEH_TURN_LAMP_INVALID;
  }
  planning_result_.tar_brake_lamp = ad_msg::VEH_LAMP_INVALID;

  planning_result_.tar_v = velocity_planning_result_.tar_v;
  planning_result_.tar_a = velocity_planning_result_.tar_a;
  planning_result_.tar_throttle = velocity_planning_result_.tar_throttle;
  planning_result_.tar_trj.timestamp = -common::CalcElapsedClockMs(
        trajectory_planning_result_.msg_head.timestamp,
        planning_result_.msg_head.timestamp);
  planning_result_.tar_trj.curr_pos.x =
      trajectory_planning_result_.curr_pos.x;
  planning_result_.tar_trj.curr_pos.y =
      trajectory_planning_result_.curr_pos.y;
  planning_result_.tar_trj.curr_pos.h =
      trajectory_planning_result_.curr_pos.heading;
  planning_result_.tar_trj.curr_pos.c =
      trajectory_planning_result_.curr_pos.curvature;
  planning_result_.tar_trj.curr_pos.s =
      trajectory_planning_result_.curr_pos.s;
  planning_result_.tar_trj.curr_pos.l =
      trajectory_planning_result_.curr_pos.l;
  planning_result_.tar_trj.leading_pos.x =
      trajectory_planning_result_.leading_pos.x;
  planning_result_.tar_trj.leading_pos.y =
      trajectory_planning_result_.leading_pos.y;
  planning_result_.tar_trj.leading_pos.h =
      trajectory_planning_result_.leading_pos.heading;
  planning_result_.tar_trj.leading_pos.c =
      trajectory_planning_result_.leading_pos.curvature;
  planning_result_.tar_trj.leading_pos.s =
      trajectory_planning_result_.leading_pos.s;
  planning_result_.tar_trj.leading_pos.l =
      trajectory_planning_result_.leading_pos.l;
  planning_result_.tar_trj.lat_err.moving_flag =
      trajectory_planning_result_.lat_err.moving_flag;
  planning_result_.tar_trj.lat_err.samples[0].lat_err =
      trajectory_planning_result_.lat_err.samples[0].lat_err;
  planning_result_.tar_trj.lat_err.samples[0].lat_err_chg_rate =
      trajectory_planning_result_.lat_err.samples[0].lat_err_chg_rate;
  planning_result_.tar_trj.lat_err.samples[0].yaw_err =
      trajectory_planning_result_.lat_err.samples[0].yaw_err;
  planning_result_.tar_trj.lat_err.samples[0].yaw_err_chg_rate =
      trajectory_planning_result_.lat_err.samples[0].yaw_err_chg_rate;
  planning_result_.tar_trj.lat_err.samples[1].lat_err =
      trajectory_planning_result_.lat_err.samples[1].lat_err;
  planning_result_.tar_trj.lat_err.samples[1].lat_err_chg_rate =
      trajectory_planning_result_.lat_err.samples[1].lat_err_chg_rate;
  planning_result_.tar_trj.lat_err.samples[1].yaw_err =
      trajectory_planning_result_.lat_err.samples[1].yaw_err;
  planning_result_.tar_trj.lat_err.samples[1].yaw_err_chg_rate =
      trajectory_planning_result_.lat_err.samples[1].yaw_err_chg_rate;

  switch (trajectory_planning_result_.trj_direction) {
  case (planning::TrajectoryPlanningResult::TRJ_DIRECTION_FORWARD):
    planning_result_.tar_trj.trj_direction =
        ad_msg::PlanningResult::TRJ_DIRECTION_FORWARD;
    break;
  case (planning::TrajectoryPlanningResult::TRJ_DIRECTION_BACKWARD):
    planning_result_.tar_trj.trj_direction =
        ad_msg::PlanningResult::TRJ_DIRECTION_BACKWARD;
    break;
  default:
    planning_result_.tar_trj.trj_direction =
        ad_msg::PlanningResult::TRJ_DIRECTION_FORWARD;
    LOG_ERR << "Invalid trajectory direction.";
    break;
  }
  planning_result_.tar_trj.points_num = 0;
  target_trajectory_sample_points_.Clear();
  if (target_trajectory_.Construct(
        trajectory_planning_result_.target_trajectory)) {
    Float32_t step_len = chassis_.v * 0.5F;
    if (step_len < 0.5F) {
      step_len = 0.5F;
    }
    // wm test 20201027
    // step_len = trajectory_planning_result_.target_trajectory.Back().s/ad_msg::PlanningResult::MAX_TRAJECTORY_POINT_NUM;
    target_trajectory_.UniformlySamplePathForward(0.0F, ad_msg::PlanningResult::MAX_TRAJECTORY_POINT_NUM, step_len,
                                                  &target_trajectory_sample_points_);

    /*
       * @author    Add BY xiaranfei
       * @date         20220731
       * @details    规划轨迹点集矩阵数组
      */
    /// TODO: 注意实际数量不一定为16，最大16
    #if (ENABLE_OUTPUT_FITTING_COEFFICIENT)
      common::Matrix<double, 16, 1> xx_polyfit;
      common::Matrix<double, 16, 1> yy_polyfit;
    #endif

    for (Int32_t i = 0; i < target_trajectory_sample_points_.Size(); ++i) {
      if (i >= ad_msg::PlanningResult::MAX_TRAJECTORY_POINT_NUM)  {
        break;
      }
      const common::PathPoint &p = target_trajectory_sample_points_[i];
      planning_result_.tar_trj.points[i].x = p.point.x();//输出的轨迹点x
      planning_result_.tar_trj.points[i].y = p.point.y();//输出的轨迹点y
      planning_result_.tar_trj.points[i].h = p.heading;//输出的轨迹点heading
      planning_result_.tar_trj.points[i].c = p.curvature;//输出的轨迹点curvature
      planning_result_.tar_trj.points[i].s = p.s;//输出的轨迹点s

      planning_result_.tar_trj.points_num++;

      #if (ENABLE_OUTPUT_FITTING_COEFFICIENT)
        xx_polyfit(i) = p.point.x();
        yy_polyfit(i) = p.point.y();
      #endif
    }

    //Add BY xiaranfei  20220731 曲线拟合 polyfit
#if (ENABLE_OUTPUT_FITTING_COEFFICIENT)
    int num=ad_msg::PlanningResult::MAX_TRAJECTORY_POINT_NUM;
    common::Matrix<double, 16, 1> x_polyfit;
    common::Matrix<double, 16, 1> y_polyfit;

    x_polyfit = xx_polyfit;
    y_polyfit = yy_polyfit;

    double coeffs[6] = { 0 };
    polyfit(x_polyfit, y_polyfit, 5, coeffs);

    LOG_INFO(5) << "拟合系数：";
 //   printf("Trj(x) = %f%+fx%+fx^2%+fx^3%+fx^4%+fx^5\n", coeffs[0], coeffs[1], coeffs[2], coeffs[3],coeffs[4],coeffs[5]);

    planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid = 1;
//    LOG_INFO(5)<<"PlanningTraj_IsValid= " << planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;

    #if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
      if (!open_log_file_pointXY) {
        open_log_file_pointXY = true;
        log_file_pointXY.Open();
        log_file_pointXY.Enable(true);

        std::snprintf(str_buff_, sizeof(str_buff_)-1,
                      "timestamp,"
                      "x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,"
                      "y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12,y13,y14,y15,y16;"
                      "\n");
        log_file_pointXY.Write(str_buff_);
      }
      std::snprintf(str_buff_, sizeof(str_buff_)-1, "%ld", common::GetClockNowMs());
      log_file_pointXY.Write(str_buff_);
      for (Int32_t i = 0; i < 16; ++i) {
        std::snprintf(str_buff_, sizeof(str_buff_)-1, ",%0.1f", xx_polyfit(i));
        log_file_pointXY.Write(str_buff_);
      }
      for (Int32_t i = 0; i < 16; ++i) {
        std::snprintf(str_buff_, sizeof(str_buff_)-1, ",%0.1f", yy_polyfit(i));
        log_file_pointXY.Write(str_buff_);
      }
      log_file_pointXY.Write(";\n");
    #endif

    /*
       * @author    Add BY xiaranfei
       * @date         20220818
       * @details    2条车道线quality：左边线left，右边线right
      */
    double c0_left,c1_left,c2_left,c3_left;
    double c0_right,c1_right,c2_right,c3_right;
    double c0_center,c1_center,c2_center,c3_center;
    double LeftLaneTraj_IsValid,RightLaneTraj_IsValid,LaneTraj_IsValid;

    for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {//4条车道线
      const ad_msg::LaneMarkCamera& lane = lane_mark_camera_list_.lane_marks[i];//LaneMarkCamera
      if (1 == lane.id) {
        // left
        c0_left = lane.c0;
        c1_left = lane.c1;
        c2_left = lane.c2;
        c3_left = lane.c3;

        if (0 == lane.quality || 1 == lane.quality) {
          LeftLaneTraj_IsValid = 0;
        } else {
          LeftLaneTraj_IsValid = 1;
        }

      } else if (-1 == lane.id) {
        // right
        c0_right = lane.c0;
        c1_right = lane.c1;
        c2_right = lane.c2;
        c3_right = lane.c3;

        if (0==lane.quality||1==lane.quality){
          RightLaneTraj_IsValid=0;
        } else {
          RightLaneTraj_IsValid=1;
        }

      } else {
        //TODO : set default value
  //      LOG_INFO(5) << "非正左右车道线";
      }
    }

    /*
          * @author    Add BY xiaranfei
          * @date         20220825
          * @details    三条车道线系数和有效位：左边线left，右边线right，中心线center
          */
    /************************车道左边线的系数和有效位****************************/
//    LOG_INFO(5) << "车道左边线的系数：";
    double c_left[6] = {0};
    c_left[0] = c0_left ;
    c_left[1] = c1_left ;
    c_left[2] = c2_left ;
    c_left[3] = c3_left ;
    c_left[4] = 0 ;
    c_left[5] = 0 ;

   // printf("LaneLeft(x) = %f%+fx%+fx^2%+fx^3%+fx^4%+fx^5\n", c_left[0], c_left[1], c_left[2], c_left[3],c_left[4],c_left[5]);

    planning_result_.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_IsValid = LeftLaneTraj_IsValid;
   // LOG_INFO(5) << "车道左边线是否有效:LeftLaneTraj_IsValid = " << planning_result_.tar_trj.LeftLaneTraj_Cv.LeftLaneTraj_IsValid;

    /************************车道右边线的系数和有效位****************************/
 //   LOG_INFO(5) << "车道右边线的系数：";
    double c_right[6] = {0};
    c_right[0] = c0_right;
    c_right[1] = c1_right;
    c_right[2] = c2_right;
    c_right[3] = c3_right;
    c_right[4] = 0 ;
    c_right[5] = 0 ;

 //   printf("LaneRight(x) = %f%+fx%+fx^2%+fx^3%+fx^4%+fx^5\n", c_right[0], c_right[1], c_right[2], c_right[3],c_right[4],c_right[5]);

    planning_result_.tar_trj.RightLaneTraj_Cv.RightLaneTraj_IsValid = RightLaneTraj_IsValid;
 //   LOG_INFO(5) << "车道右边线是否有效:RightLaneTraj_IsValid=" << planning_result_.tar_trj.RightLaneTraj_Cv.RightLaneTraj_IsValid;

 //   LOG_INFO(5) << "车道中心线的系数：";
    double roadwide=3.0;//可视化界面：CAM车道宽____默认车道宽3m
    double c_center[6] = {0};
    if (1 == LeftLaneTraj_IsValid && 1 == RightLaneTraj_IsValid) {//左边线和右边线都有效
      c_center[0] = (c0_left + c0_right) / 2;
      c_center[1] = (c1_left + c1_right) / 2;
      c_center[2] = (c2_left + c2_right) / 2;
      c_center[3] = (c3_left + c3_right) / 2;
      c_center[4] = 0;
      c_center[5] = 0;
      LaneTraj_IsValid = 1;
    } else if (0 == LeftLaneTraj_IsValid && 1 == RightLaneTraj_IsValid) {//左边线无效，右边线有效
      c_center[0] = c0_right + roadwide/2 ;
      c_center[1] = c1_right;
      c_center[2] = c2_right;
      c_center[3] = c3_right;
      c_center[4] = 0;
      c_center[5] = 0;
      LaneTraj_IsValid = 1;
    } else if (1 == LeftLaneTraj_IsValid && 0 == RightLaneTraj_IsValid) {//左边线有效，右边线无效
      c_center[0] = c0_left - roadwide/2 ;
      c_center[1] = c1_left;
      c_center[2] = c2_left;
      c_center[3] = c3_left;
      c_center[4] = 0;
      c_center[5] = 0;
      LaneTraj_IsValid = 1;
    }else{
      LaneTraj_IsValid = 0;//车道中心线无效
    }

 //   printf("LaneCenter(x) = %f%+fx%+fx^2%+fx^3%+fx^4%+fx^5\n", c_center[0], c_center[1], c_center[2], c_center[3],c_center[4],c_center[5]);

    planning_result_.tar_trj.LaneTraj_Cv.LaneTraj_IsValid = LaneTraj_IsValid;
 //   LOG_INFO(5) << "车道中心线是否有效:LaneTraj_IsValid=" << planning_result_.tar_trj.LaneTraj_Cv.LaneTraj_IsValid;

    /*
      * @author    Add BY xiaranfei
      * @date         20220823
      * @details    换道和跟车：车道中心线系数和规划轨迹系数的统一接口
      */
    ///TODO:ChangeLane_KeepLane
    //换道
    if (planning::TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_I == trajectory_planning_result_.trj_status
        || planning::TRJ_STATUS_CHANGING_LANE_LEFT_STAGE_II == trajectory_planning_result_.trj_status
        || planning::TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_I == trajectory_planning_result_.trj_status
        || planning::TRJ_STATUS_CHANGING_LANE_RIGHT_STAGE_II == trajectory_planning_result_.trj_status) {
 //     LOG_INFO(5) << "ChangeLane拟合系数:";
      for (Int32_t i = 0; i < 6; i++){
        planning_result_.tar_trj.PlanningTraj_Cv.coeffs[i] = coeffs[i];
        LOG_INFO(5)<< coeffs[i]<<"|";
      }

      planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid = 1;
//      LOG_INFO(5) << "规划轨迹PlanningTraj_IsValid=" << planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;
    } else {
      //车道保持
//      LOG_INFO(5) << "KeepLane拟合系数:";
      for (Int32_t i = 0; i < 6; i++){
        planning_result_.tar_trj.PlanningTraj_Cv.coeffs[i] = c_center[i];
        LOG_INFO(5)<< coeffs[i] << "|";
      }
      
      planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid = LaneTraj_IsValid;
//      LOG_INFO(5) << "车道线PlanningTraj_IsValid="<<planning_result_.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;
    }
  #endif
  }

  if (velocity_planning_result_.tar_obj.valid) {
    planning_result_.front_nearest_obj_distance = velocity_planning_result_.tar_obj.dist_to_obj;
    planning_result_.front_nearest_obj_speed = velocity_planning_result_.tar_obj.obj_v;
  } else {
    planning_result_.front_nearest_obj_distance = 255.0F;
    planning_result_.front_nearest_obj_speed = 255.0F;
  }
#if PACC_MAP_DEBUG
  planning_result_.ramp_slope_value = 0.0F;
#else
  if (!trajectory_planning_result_.target_trajectory_slope.Empty()) {
    planning_result_.ramp_slope_value = trajectory_planning_result_.target_trajectory_slope.Front().slope;
  } else {
    planning_result_.ramp_slope_value = 0.0F;
  }
#endif

  planning_result_.pitch = gnss_info_.pitch;
  planning_result_.pitch_variance = gnss_info_.pitch_variance;

  /* k006 pengc 2023-01-06 (begin) */
  // 更新轨迹坡度
#if 1
  Int32_t slope_sample_num =
      trajectory_planning_result_.target_trajectory_slope.Size();
  if (slope_sample_num > 0) {
 //   printf("@@@ The slope in current position is %0.2f\n",
 //           trajectory_planning_result_.target_trajectory_slope.Front().slope);
  }
//  printf("@@@ The slopes of target trajectory are: \n");
  for (Int32_t i = 0; i < slope_sample_num; ++i) {
    const planning::TrajectoryPlanningResult::SlopeSamplePoint& sample =
        trajectory_planning_result_.target_trajectory_slope[i];
  //  printf("    [%d]: s=%0.1f, slope=%0.1f\n", i, sample.s, sample.slope);
  }
#endif
  /* k006 pengc 2023-01-06 (end) */

  /* k006 pengc 2023-02-12 (begin) */
  // 更新轨迹俯仰角度(for d17)
#if 0
  printf("@@@ pitch = %0.2f deg\n", common::com_rad2deg(gnss_info_.pitch));
#endif
  /* k006 pengc 2023-02-12 (end) */
}

void WorkPlanning::SetPlanningStatus() {
  // Clear status
  common::com_memset(planning_result_.planning_status, 0,
                     sizeof(planning_result_.planning_status));

  // ADAS 使能
  if (action_planning_result_.enable_adas) {
    planning_result_.planning_status[0] |= 0x01;
  }
  // AEB 使能
  if (action_planning_result_.enable_aeb) {
    planning_result_.planning_status[0] |= (0x01 << 1);
  }
  // ACC 使能
  if (action_planning_result_.enable_acc) {
    planning_result_.planning_status[0] |= (0x01 << 2);
  }
  // LKA 使能
  if (action_planning_result_.enable_lka) {
    planning_result_.planning_status[0] |= (0x01 << 3);
  }
  // 变道功能 使能
  if (action_planning_result_.enable_alc) {
    planning_result_.planning_status[0] |= (0x01 << 4);
  }
  // 智能限速功能 使能
  if (action_planning_result_.enable_isl) {
    planning_result_.planning_status[0] |= (0x01 << 5);
  }
  // Navigation Guided Pilot 使能
  if (action_planning_result_.enable_ngp) {
    planning_result_.planning_status[0] |= (0x01 << 6);
  }
#if 0
  // AEB Warning状态
  if (planning::VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN ==
      velocity_planning_result_.tar_type) {
    planning_result_.cur_status |= (0x01 << 7);
  }
  // AEB状态
  if (planning::VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION ==
      velocity_planning_result_.tar_type) {
    //printf("@@@@ Send to control , AEB_ACTION, tar_v=%0.1f, tar_a=%0.1f\n",
    //       velocity_planning_result_.tar_v*3.6f, velocity_planning_result_.tar_a);
    planning_result_.cur_status |= (0x01 << 8);
  }
#endif

  // Driving map type
  Int32_t driving_map_type = driving_map_.GetDrivingMapType();
  planning_result_.planning_status[0] |= ((driving_map_type & 0x0F) << 8);
  // Road type
  planning_result_.planning_status[0] |=
      ((trajectory_planning_result_.road_type & 0x0F) << 12);
  // Trajectory status
  planning_result_.planning_status[0] |=
      ((trajectory_planning_result_.trj_status & 0x0F) << 16);
  // Deceleration target
  planning_result_.planning_status[0] |=
      ((velocity_planning_result_.tar_type & 0x0F) << 20);

}

void WorkPlanning::GetPlanningSettings() {
  SharedData* shared_data = SharedData::instance();

  // 获取HMI发送的设置信息
  shared_data->GetCurrentPlanningSettings(&curr_planning_settings_);
  shared_data->GetLocalPlanningSettings(&local_planning_settings_);
  shared_data->GetRemotePlanningSettings(&remote_planning_settings_);

  ad_msg::PlanningSettings prev_planning_settings = curr_planning_settings_;

  curr_planning_settings_.start_adas = 0;
  curr_planning_settings_.changing_lane_req = 0;
  UpdatePlanningSettings(remote_planning_settings_, &curr_planning_settings_);
  UpdatePlanningSettings(local_planning_settings_, &curr_planning_settings_);

  ad_msg::PlanningSettings chassis_planning_settings;
  chassis_planning_settings.start_adas = special_chassis_info_.start_adas;
  chassis_planning_settings.enable_lka = special_chassis_info_.enable_lka;
  chassis_planning_settings.enable_acc = special_chassis_info_.enable_acc;
  chassis_planning_settings.enable_aeb = special_chassis_info_.enable_aeb;
  chassis_planning_settings.enable_alc = special_chassis_info_.enable_alc;
  chassis_planning_settings.enable_isl = special_chassis_info_.enable_isl;
  chassis_planning_settings.enable_ngp = special_chassis_info_.enable_ngp;
  chassis_planning_settings.enable_fallback = special_chassis_info_.dfcv_d17.enable_fallback;
  chassis_planning_settings.target_acc_valid = special_chassis_info_.target_acc_valid;
  chassis_planning_settings.target_acc = special_chassis_info_.target_acc;
  chassis_planning_settings.target_time_gap_valid = special_chassis_info_.target_time_gap_valid;
  chassis_planning_settings.target_time_gap = special_chassis_info_.target_time_gap;
  chassis_planning_settings.target_velocity_valid = special_chassis_info_.target_velocity_valid;
  chassis_planning_settings.target_velocity = special_chassis_info_.target_velocity;
  chassis_planning_settings.changing_lane_req = special_chassis_info_.changing_lane_req;
  chassis_planning_settings.target_fallback_level_valid = special_chassis_info_.dfcv_d17.target_fallback_valid;
  chassis_planning_settings.target_fallback_level = special_chassis_info_.dfcv_d17.AD_fallback_level;
  // 拨杆变道
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  bool start_changing_lane_left = false;
  bool stop_changing_lane_left = false;
  bool start_changing_lane_right = false;
  bool stop_changing_lane_right = false;
  if (ad_msg::VEH_TURNING_INDICATOR_LEFT == chassis_.signal_turning_indicator) {
    button_changing_lane_req_left_.Update(
          1, &start_changing_lane_left, &stop_changing_lane_left);
    button_changing_lane_req_right_.Update(
          0, &start_changing_lane_right, &stop_changing_lane_right);
  } else if (ad_msg::VEH_TURNING_INDICATOR_RIGHT == chassis_.signal_turning_indicator) {
    button_changing_lane_req_left_.Update(
          0, &start_changing_lane_left, &stop_changing_lane_left);
    button_changing_lane_req_right_.Update(
          1, &start_changing_lane_right, &stop_changing_lane_right);
  } else {
    button_changing_lane_req_left_.Update(
          0, &start_changing_lane_left, &stop_changing_lane_left);
    button_changing_lane_req_right_.Update(
          0, &start_changing_lane_right, &stop_changing_lane_right);
  }
  if (stop_changing_lane_left) {
    chassis_planning_settings.changing_lane_req = 3;
  }
  if (stop_changing_lane_right) {
    chassis_planning_settings.changing_lane_req = 3;
  }
  if (start_changing_lane_left) {
    chassis_planning_settings.changing_lane_req = 1;
  }
  if (start_changing_lane_right) {
    chassis_planning_settings.changing_lane_req = 2;
  }
#endif
  UpdatePlanningSettings(chassis_planning_settings, &curr_planning_settings_);

  bool start_hwa = false;
  bool stop_hwa = false;
  button_start_hwa_.Update(
        special_chassis_info_.ft_auman.switch_hwa, &start_hwa, &stop_hwa);
  if (start_hwa) {
    curr_planning_settings_.enable_alc = 2;
  }
  if (stop_hwa) {
    curr_planning_settings_.enable_alc = 1;
  }
  bool start_i_drive = false;
  bool stop_i_drive = false;
  button_start_i_drive_.Update(
        special_chassis_info_.ft_auman.switch_i_drive, &start_i_drive, &stop_i_drive);
  if (start_i_drive) {
    curr_planning_settings_.enable_ngp = 2;
  }
  if (stop_i_drive) {
    curr_planning_settings_.enable_ngp = 1;
  }

  shared_data->SetCurrentPlanningSettings(curr_planning_settings_);

  /* k001 pengc 2022-04-11 (begin) */
  // 关闭ISL功能时，清除限速牌信息，便于相关功能测试
  if ((2 == prev_planning_settings.enable_isl) &&
      (1 == curr_planning_settings_.enable_isl)) {
    shared_data->ClearTrafficSignalList();
  }
  /* k001 pengc 2022-04-11 (end) */
}

void WorkPlanning::UpdatePlanningSettings(
    const ad_msg::PlanningSettings& new_settings,
    ad_msg::PlanningSettings* const cur_settings) {

  if (0 != new_settings.start_adas) {
    cur_settings->start_adas = new_settings.start_adas;
  }

  if (0 != new_settings.enable_lka) {
    cur_settings->enable_lka = new_settings.enable_lka;
  }
  if (0 != new_settings.enable_acc) {
    cur_settings->enable_acc = new_settings.enable_acc;
  }
  if (0 != new_settings.enable_aeb) {
    cur_settings->enable_aeb = new_settings.enable_aeb;
  }

  if (0 != new_settings.enable_alc) {
    cur_settings->enable_alc = new_settings.enable_alc;
  }
  if (0 != new_settings.enable_isl) {
    cur_settings->enable_isl = new_settings.enable_isl;
  }
  if (0 != new_settings.enable_ngp) {
    cur_settings->enable_ngp = new_settings.enable_ngp;
  }
  if (0 != new_settings.enable_fallback) {
      cur_settings->enable_fallback = new_settings.enable_fallback;
  }
  if (0 != new_settings.enable_pcc) {
      cur_settings->enable_pcc = new_settings.enable_pcc;
  }

  if (new_settings.target_velocity_valid) {
      cur_settings->target_velocity = new_settings.target_velocity;
  }
  if (new_settings.target_acc_valid) {
      cur_settings->target_acc = new_settings.target_acc;
  }

  if (new_settings.target_time_gap_valid) {
      cur_settings->target_time_gap = new_settings.target_time_gap;
  }

  if (new_settings.target_fallback_level_valid) {   
      cur_settings->target_fallback_level = new_settings.target_fallback_level;
  }
  if (0 != new_settings.changing_lane_req) {
      cur_settings->changing_lane_req = new_settings.changing_lane_req;
  }
}
#if (ENABLE_ROS_NODE)
void WorkPlanning::SetVelocityPlanningInternalDebug(const planning::VelocityPlanningInternal& data) {

  planning_debug_.VelPlanning.internal.timestamp = data.timestamp;

  planning_debug_.VelPlanning.internal.planning_pos.path_point.x = data.planning_pos.path_point.point.x();
  planning_debug_.VelPlanning.internal.planning_pos.path_point.y = data.planning_pos.path_point.point.y();
  planning_debug_.VelPlanning.internal.planning_pos.path_point.heading = data.planning_pos.path_point.heading;
  planning_debug_.VelPlanning.internal.planning_pos.path_point.curvature = data.planning_pos.path_point.curvature;
  planning_debug_.VelPlanning.internal.planning_pos.path_point.s = data.planning_pos.path_point.s;
  planning_debug_.VelPlanning.internal.planning_pos.path_point.l = data.planning_pos.path_point.l;

  planning_debug_.VelPlanning.internal.planning_pos.v = data.planning_pos.v;
  planning_debug_.VelPlanning.internal.planning_pos.a = data.planning_pos.a;
  planning_debug_.VelPlanning.internal.planning_pos.yaw_rate = data.planning_pos.yaw_rate;
  planning_debug_.VelPlanning.internal.planning_pos.relative_time = data.planning_pos.relative_time;

  planning_debug_.VelPlanning.internal.planning_path_point_num = data.planning_path_point_num;
  planning_debug_.VelPlanning.internal.planning_path.clear();
  planning_debug_.VelPlanning.internal.planning_path.reserve(data.planning_path_point_num);
  ::planning::path_point pathPoint;
  for (Int32_t i = 0; i < data.planning_path_point_num; ++i) {
    pathPoint.x = data.planning_path[i].point.x();
    pathPoint.y = data.planning_path[i].point.y();
    pathPoint.heading = data.planning_path[i].heading;
    pathPoint.curvature = data.planning_path[i].curvature;
    pathPoint.s = data.planning_path[i].s;
    pathPoint.l = data.planning_path[i].l;

    planning_debug_.VelPlanning.internal.planning_path.push_back(pathPoint);
  }

  planning_debug_.VelPlanning.internal.adas_enable_pre = data.adas_enable_pre;
  planning_debug_.VelPlanning.internal.adas_enable = data.adas_enable;
  
  planning_debug_.VelPlanning.internal.major_ref_line_point_num = data.major_ref_line_point_num;
  planning_debug_.VelPlanning.internal.major_ref_line.clear();
  planning_debug_.VelPlanning.internal.major_ref_line.reserve(data.major_ref_line_point_num);
  for (Int32_t i = 0; i < data.major_ref_line_point_num; ++i) {
    pathPoint.x = data.major_ref_line[i].point.x();
    pathPoint.y = data.major_ref_line[i].point.y();
    pathPoint.heading = data.major_ref_line[i].heading;
    pathPoint.curvature = data.major_ref_line[i].curvature;
    pathPoint.s = data.major_ref_line[i].s;
    pathPoint.l = data.major_ref_line[i].l;

    planning_debug_.VelPlanning.internal.major_ref_line.push_back(pathPoint);
  }

  planning_debug_.VelPlanning.internal.user_setting_v = data.user_setting_v;
  planning_debug_.VelPlanning.internal.current_plan_v = data.current_plan_v;
  planning_debug_.VelPlanning.internal.upper_limit_v_road = data.upper_limit_v_road;
  planning_debug_.VelPlanning.internal.upper_limit_v_tunnel = data.upper_limit_v_tunnel;
  planning_debug_.VelPlanning.internal.upper_limit_v_ramp = data.upper_limit_v_ramp;
  planning_debug_.VelPlanning.internal.upper_limit_v_curvature = data.upper_limit_v_curvature;
  planning_debug_.VelPlanning.internal.upper_limit_v_lane_change = data.upper_limit_v_lane_change;
  planning_debug_.VelPlanning.internal.upper_limit_v_final = data.upper_limit_v_final;

  planning_debug_.VelPlanning.internal.tar_v_cruise = data.tar_v_cruise;
  planning_debug_.VelPlanning.internal.tar_a_cruise = data.tar_a_cruise;

  planning_debug_.VelPlanning.internal.target_path_max_curvature = data.target_path_max_curvature;
  planning_debug_.VelPlanning.internal.tar_type_curvature = data.tar_type_curvature;
  planning_debug_.VelPlanning.internal.tar_v_curvature = data.tar_v_curvature;
  planning_debug_.VelPlanning.internal.tar_a_curvature = data.tar_a_curvature;

  planning_debug_.VelPlanning.internal.changlane_path_point_num = data.changlane_path_point_num;
  planning_debug_.VelPlanning.internal.changlane_path.clear();
  planning_debug_.VelPlanning.internal.changlane_path.reserve(data.changlane_path_point_num);
  for (Int32_t i = 0; i < data.changlane_path_point_num; ++i) {
    pathPoint.x = data.changlane_path[i].point.x();
    pathPoint.y = data.changlane_path[i].point.y();
    pathPoint.heading = data.changlane_path[i].heading;
    pathPoint.curvature = data.changlane_path[i].curvature;
    pathPoint.s = data.changlane_path[i].s;
    pathPoint.l = data.changlane_path[i].l;

    planning_debug_.VelPlanning.internal.changlane_path.push_back(pathPoint);
  }

  planning_debug_.VelPlanning.internal.slope = data.slope;
  planning_debug_.VelPlanning.internal.distance_to_slope = data.distance_to_slope;
  planning_debug_.VelPlanning.internal.tar_type_slope = data.tar_type_slope;
  planning_debug_.VelPlanning.internal.tar_v_slope = data.tar_v_slope;
  planning_debug_.VelPlanning.internal.tar_a_slope = data.tar_a_slope;

  planning_debug_.VelPlanning.internal.traffic_light_type = data.traffic_light_type;
  planning_debug_.VelPlanning.internal.distance_to_traffic_light = data.distance_to_traffic_light;
  planning_debug_.VelPlanning.internal.tar_type_traffic_light = data.tar_type_traffic_light;
  planning_debug_.VelPlanning.internal.tar_v_traffic_light = data.tar_v_traffic_light;
  planning_debug_.VelPlanning.internal.tar_a_traffic_light = data.tar_a_traffic_light;

  planning_debug_.VelPlanning.internal.traffic_signal_type = data.traffic_signal_type;
  planning_debug_.VelPlanning.internal.distance_to_traffic_signal = data.distance_to_traffic_signal;
  planning_debug_.VelPlanning.internal.tar_type_traffic_signal = data.tar_type_traffic_signal;
  planning_debug_.VelPlanning.internal.tar_v_traffic_signal = data.tar_v_traffic_signal;
  planning_debug_.VelPlanning.internal.tar_a_traffic_signal = data.tar_a_traffic_signal;

  planning_debug_.VelPlanning.internal.ramp_velocity_limit = data.ramp_velocity_limit;
  planning_debug_.VelPlanning.internal.distance_to_ramp = data.distance_to_ramp;
  planning_debug_.VelPlanning.internal.tar_type_ramp = data.tar_type_ramp;
  planning_debug_.VelPlanning.internal.tar_v_ramp = data.tar_v_ramp;
  planning_debug_.VelPlanning.internal.tar_a_ramp = data.tar_a_ramp;

  planning_debug_.VelPlanning.internal.tunnel_velocity_limit = data.tunnel_velocity_limit;
  planning_debug_.VelPlanning.internal.distance_to_tunnel = data.distance_to_tunnel;
  planning_debug_.VelPlanning.internal.tar_type_tunnel = data.tar_type_tunnel;
  planning_debug_.VelPlanning.internal.tar_v_tunnel = data.tar_v_tunnel;
  planning_debug_.VelPlanning.internal.tar_a_tunnel = data.tar_a_tunnel;

  planning_debug_.VelPlanning.internal.lane_velocity_limit = data.lane_velocity_limit;
  planning_debug_.VelPlanning.internal.distance_to_lane_velocity_limit = data.distance_to_lane_velocity_limit;
  planning_debug_.VelPlanning.internal.tar_type_lane = data.tar_type_lane;
  planning_debug_.VelPlanning.internal.tar_v_lane = data.tar_v_lane;
  planning_debug_.VelPlanning.internal.tar_a_lane = data.tar_a_lane;

  planning_debug_.VelPlanning.internal.tar_type_virtual_obstacle = data.tar_type_virtual_obstacle;
  planning_debug_.VelPlanning.internal.tar_v_virtual_obstacle = data.tar_v_virtual_obstacle;
  planning_debug_.VelPlanning.internal.tar_a_virtual_obstacle = data.tar_a_virtual_obstacle;

  planning_debug_.VelPlanning.internal.fallback_type = data.fallback_type;
  planning_debug_.VelPlanning.internal.tar_type_fallback = data.tar_type_fallback;
  planning_debug_.VelPlanning.internal.tar_v_fallback = data.tar_v_fallback;
  planning_debug_.VelPlanning.internal.tar_a_fallback = data.tar_a_fallback;

  ///// 车道线跟随障碍物处理 
  planning_debug_.VelPlanning.internal.collision_test_path_point_num = data.collision_test_path_point_num;
  planning_debug_.VelPlanning.internal.collision_test_path.clear();
  planning_debug_.VelPlanning.internal.collision_test_path.reserve(data.collision_test_path_point_num);
  for (Int32_t i = 0; i < data.collision_test_path_point_num; ++i) {
    pathPoint.x = data.collision_test_path[i].point.x();
    pathPoint.y = data.collision_test_path[i].point.y();
    pathPoint.heading = data.collision_test_path[i].heading;
    pathPoint.curvature = data.collision_test_path[i].curvature;
    pathPoint.s = data.collision_test_path[i].s;
    pathPoint.l = data.collision_test_path[i].l;

    planning_debug_.VelPlanning.internal.collision_test_path.push_back(pathPoint);
  }

  planning_debug_.VelPlanning.internal.collision_test_samples_point_num = data.collision_test_samples_point_num;
  planning_debug_.VelPlanning.internal.collision_test_samples.clear();
  planning_debug_.VelPlanning.internal.collision_test_samples.reserve(data.collision_test_samples_point_num);
  for (Int32_t i = 0; i < data.collision_test_samples_point_num; ++i) {
    pathPoint.x = data.collision_test_samples[i].point.x();
    pathPoint.y = data.collision_test_samples[i].point.y();
    pathPoint.heading = data.collision_test_samples[i].heading;
    pathPoint.curvature = data.collision_test_samples[i].curvature;
    pathPoint.s = data.collision_test_samples[i].s;
    pathPoint.l = data.collision_test_samples[i].l;

    planning_debug_.VelPlanning.internal.collision_test_samples.push_back(pathPoint);
  }

  // CIPV
  planning_debug_.VelPlanning.internal.cipv.is_risk = data.cipv.is_risk;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.id = data.cipv.perception_obstacle.id;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.x = data.cipv.perception_obstacle.x;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.y = data.cipv.perception_obstacle.y;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.obb.x = data.cipv.perception_obstacle.obb.x;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.obb.y = data.cipv.perception_obstacle.obb.y;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.obb.heading = data.cipv.perception_obstacle.obb.heading;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.obb.half_width = data.cipv.perception_obstacle.obb.half_width;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.obb.half_length = data.cipv.perception_obstacle.obb.half_length;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.height = data.cipv.perception_obstacle.height;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.height_to_ground = data.cipv.perception_obstacle.height_to_ground;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.type = data.cipv.perception_obstacle.type;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.dynamic = data.cipv.perception_obstacle.dynamic;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.confidence = data.cipv.perception_obstacle.confidence;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.perception_type = data.cipv.perception_obstacle.perception_type;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.v_x = data.cipv.perception_obstacle.v_x;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.v_y = data.cipv.perception_obstacle.v_y;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.v = data.cipv.perception_obstacle.v;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.a_x = data.cipv.perception_obstacle.a_x;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.a_y = data.cipv.perception_obstacle.a_y;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.a = data.cipv.perception_obstacle.a;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.valid = data.cipv.perception_obstacle.proj_on_major_ref_line.valid;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.x = data.cipv.perception_obstacle.proj_on_major_ref_line.x;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.y = data.cipv.perception_obstacle.proj_on_major_ref_line.y;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.heading = data.cipv.perception_obstacle.proj_on_major_ref_line.heading;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.curvature = data.cipv.perception_obstacle.proj_on_major_ref_line.curvature;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.s = data.cipv.perception_obstacle.proj_on_major_ref_line.s;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.proj_on_major_ref_line.l = data.cipv.perception_obstacle.proj_on_major_ref_line.l;

  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.pred_path_num = data.cipv.perception_obstacle.pred_path_num;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.pred_path_list.clear();
  ::planning::pred_point pred_point;
  for (Int32_t i = 0; i < data.cipv.perception_obstacle.pred_path_num; ++i) {

    ::planning::pred_path_1d pred_path_1d;
    pred_path_1d.pred_path_point_num = data.cipv.perception_obstacle.pred_path_point_num[i];
    for (Int32_t j = 0; j < pred_path_1d.pred_path_point_num; ++j) {
      pred_point.x = data.cipv.perception_obstacle.pred_path[i][j].x;
      pred_point.y = data.cipv.perception_obstacle.pred_path[i][j].y;
      pred_point.heading = data.cipv.perception_obstacle.pred_path[i][j].heading;
      pred_point.s = data.cipv.perception_obstacle.pred_path[i][j].s;

      planning_debug_.VelPlanning.internal.cipv.perception_obstacle.pred_path_list[i].pred_path.push_back(pred_point);
    }
    planning_debug_.VelPlanning.internal.cipv.perception_obstacle.pred_path_list.push_back(pred_path_1d);
  }

  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.tracked_path_point_num = data.cipv.perception_obstacle.tracked_path_point_num;
  planning_debug_.VelPlanning.internal.cipv.perception_obstacle.tracked_path.clear();
  ::planning::tracked_point tracked_point;
  for (Int32_t i = 0; i < data.cipv.perception_obstacle.tracked_path_point_num; ++i) {
    tracked_point.x = data.cipv.perception_obstacle.tracked_path[i].x;
    tracked_point.y = data.cipv.perception_obstacle.tracked_path[i].y;

    planning_debug_.VelPlanning.internal.cipv.perception_obstacle.tracked_path.push_back(tracked_point);
  }

  planning_debug_.VelPlanning.internal.cipv.obj_position = data.cipv.obj_position;
  planning_debug_.VelPlanning.internal.cipv.obj_direction = data.cipv.obj_direction;
  planning_debug_.VelPlanning.internal.cipv.is_cutin = data.cipv.is_cutin;
  planning_debug_.VelPlanning.internal.cipv.obb_distance = data.cipv.obb_distance;
  planning_debug_.VelPlanning.internal.cipv.need_process = data.cipv.need_process;
  planning_debug_.VelPlanning.internal.cipv.can_following = data.cipv.can_following;
  planning_debug_.VelPlanning.internal.cipv.is_cipv = data.cipv.is_cipv;
  planning_debug_.VelPlanning.internal.cipv.need_avoiding = data.cipv.need_avoiding;
  planning_debug_.VelPlanning.internal.cipv.is_mio = data.cipv.is_mio;
  planning_debug_.VelPlanning.internal.cipv.need_aeb_warning = data.cipv.need_aeb_warning;
  planning_debug_.VelPlanning.internal.cipv.is_aeb_warning_target = data.cipv.is_aeb_warning_target;
  planning_debug_.VelPlanning.internal.cipv.need_aeb_full_brake = data.cipv.need_aeb_full_brake;
  planning_debug_.VelPlanning.internal.cipv.is_aeb_full_brake_target = data.cipv.is_aeb_full_brake_target;
  planning_debug_.VelPlanning.internal.cipv.dist_to_obj = data.cipv.dist_to_obj;
  planning_debug_.VelPlanning.internal.cipv.safe_dist = data.cipv.safe_dist;
  planning_debug_.VelPlanning.internal.cipv.v_for_plan = data.cipv.v_for_plan;
  planning_debug_.VelPlanning.internal.cipv.time_gap = data.cipv.time_gap;
  planning_debug_.VelPlanning.internal.cipv.ttc = data.cipv.ttc;
  planning_debug_.VelPlanning.internal.cipv.planning_target_v = data.cipv.planning_target_v;
  planning_debug_.VelPlanning.internal.cipv.planning_target_s = data.cipv.planning_target_s;

  planning_debug_.VelPlanning.internal.tar_type_following = data.tar_type_following;
  planning_debug_.VelPlanning.internal.tar_v_following = data.tar_v_following;
  planning_debug_.VelPlanning.internal.tar_a_following = data.tar_a_following;
  planning_debug_.VelPlanning.internal.fsm_follow_state = data.fsm_follow_state;

  // obstacle
  // TODO: 复用
  planning_debug_.VelPlanning.internal.avoidance.is_risk = data.avoidance.is_risk;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.id = data.avoidance.perception_obstacle.id;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.x = data.avoidance.perception_obstacle.x;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.y = data.avoidance.perception_obstacle.y;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.obb.x = data.avoidance.perception_obstacle.obb.x;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.obb.y = data.avoidance.perception_obstacle.obb.y;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.obb.heading = data.avoidance.perception_obstacle.obb.heading;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.obb.half_width = data.avoidance.perception_obstacle.obb.half_width;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.obb.half_length = data.avoidance.perception_obstacle.obb.half_length;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.height = data.avoidance.perception_obstacle.height;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.height_to_ground = data.avoidance.perception_obstacle.height_to_ground;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.type = data.avoidance.perception_obstacle.type;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.dynamic = data.avoidance.perception_obstacle.dynamic;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.confidence = data.avoidance.perception_obstacle.confidence;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.perception_type = data.avoidance.perception_obstacle.perception_type;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.v_x = data.avoidance.perception_obstacle.v_x;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.v_y = data.avoidance.perception_obstacle.v_y;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.v = data.avoidance.perception_obstacle.v;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.a_x = data.avoidance.perception_obstacle.a_x;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.a_y = data.avoidance.perception_obstacle.a_y;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.a = data.avoidance.perception_obstacle.a;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.valid = data.avoidance.perception_obstacle.proj_on_major_ref_line.valid;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.x = data.avoidance.perception_obstacle.proj_on_major_ref_line.x;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.y = data.avoidance.perception_obstacle.proj_on_major_ref_line.y;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.heading = data.avoidance.perception_obstacle.proj_on_major_ref_line.heading;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.curvature = data.avoidance.perception_obstacle.proj_on_major_ref_line.curvature;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.s = data.avoidance.perception_obstacle.proj_on_major_ref_line.s;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.proj_on_major_ref_line.l = data.avoidance.perception_obstacle.proj_on_major_ref_line.l;

  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.pred_path_num = data.avoidance.perception_obstacle.pred_path_num;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.pred_path_list.clear();
  for (Int32_t i = 0; i < data.avoidance.perception_obstacle.pred_path_num; ++i) {

    ::planning::pred_path_1d pred_path_1d;
    pred_path_1d.pred_path_point_num = data.avoidance.perception_obstacle.pred_path_point_num[i];
    for (Int32_t j = 0; j < pred_path_1d.pred_path_point_num; ++j) {
      pred_point.x = data.avoidance.perception_obstacle.pred_path[i][j].x;
      pred_point.y = data.avoidance.perception_obstacle.pred_path[i][j].y;
      pred_point.heading = data.avoidance.perception_obstacle.pred_path[i][j].heading;
      pred_point.s = data.avoidance.perception_obstacle.pred_path[i][j].s;

      planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.pred_path_list[i].pred_path.push_back(pred_point);
    }
    planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.pred_path_list.push_back(pred_path_1d);
  }

  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.tracked_path_point_num = data.avoidance.perception_obstacle.tracked_path_point_num;
  planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.tracked_path.clear();
  for (Int32_t i = 0; i < data.avoidance.perception_obstacle.tracked_path_point_num; ++i) {
    tracked_point.x = data.avoidance.perception_obstacle.tracked_path[i].x;
    tracked_point.y = data.avoidance.perception_obstacle.tracked_path[i].y;

    planning_debug_.VelPlanning.internal.avoidance.perception_obstacle.tracked_path.push_back(tracked_point);
  }

  planning_debug_.VelPlanning.internal.avoidance.obj_position = data.avoidance.obj_position;
  planning_debug_.VelPlanning.internal.avoidance.obj_direction = data.avoidance.obj_direction;
  planning_debug_.VelPlanning.internal.avoidance.is_cutin = data.avoidance.is_cutin;
  planning_debug_.VelPlanning.internal.avoidance.obb_distance = data.avoidance.obb_distance;
  planning_debug_.VelPlanning.internal.avoidance.need_process = data.avoidance.need_process;
  planning_debug_.VelPlanning.internal.avoidance.can_following = data.avoidance.can_following;
  planning_debug_.VelPlanning.internal.avoidance.is_cipv = data.avoidance.is_cipv;
  planning_debug_.VelPlanning.internal.avoidance.need_avoiding = data.avoidance.need_avoiding;
  planning_debug_.VelPlanning.internal.avoidance.is_mio = data.avoidance.is_mio;
  planning_debug_.VelPlanning.internal.avoidance.need_aeb_warning = data.avoidance.need_aeb_warning;
  planning_debug_.VelPlanning.internal.avoidance.is_aeb_warning_target = data.avoidance.is_aeb_warning_target;
  planning_debug_.VelPlanning.internal.avoidance.need_aeb_full_brake = data.avoidance.need_aeb_full_brake;
  planning_debug_.VelPlanning.internal.avoidance.is_aeb_full_brake_target = data.avoidance.is_aeb_full_brake_target;
  planning_debug_.VelPlanning.internal.avoidance.dist_to_obj = data.avoidance.dist_to_obj;
  planning_debug_.VelPlanning.internal.avoidance.safe_dist = data.avoidance.safe_dist;
  planning_debug_.VelPlanning.internal.avoidance.v_for_plan = data.avoidance.v_for_plan;
  planning_debug_.VelPlanning.internal.avoidance.time_gap = data.avoidance.time_gap;
  planning_debug_.VelPlanning.internal.avoidance.ttc = data.avoidance.ttc;
  planning_debug_.VelPlanning.internal.avoidance.planning_target_v = data.avoidance.planning_target_v;
  planning_debug_.VelPlanning.internal.avoidance.planning_target_s = data.avoidance.planning_target_s;

  planning_debug_.VelPlanning.internal.tar_type_obstacle = data.tar_type_obstacle;
  planning_debug_.VelPlanning.internal.tar_v_obstacle = data.tar_v_obstacle;
  planning_debug_.VelPlanning.internal.tar_a_obstacle = data.tar_a_obstacle;
  planning_debug_.VelPlanning.internal.fsm_obstacle_state = data.fsm_obstacle_state;

  // AEB
  planning_debug_.VelPlanning.internal.mio.is_risk = data.mio.is_risk;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.id = data.mio.perception_obstacle.id;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.x = data.mio.perception_obstacle.x;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.y = data.mio.perception_obstacle.y;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.obb.x = data.mio.perception_obstacle.obb.x;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.obb.y = data.mio.perception_obstacle.obb.y;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.obb.heading = data.mio.perception_obstacle.obb.heading;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.obb.half_width = data.mio.perception_obstacle.obb.half_width;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.obb.half_length = data.mio.perception_obstacle.obb.half_length;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.height = data.mio.perception_obstacle.height;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.height_to_ground = data.mio.perception_obstacle.height_to_ground;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.type = data.mio.perception_obstacle.type;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.dynamic = data.mio.perception_obstacle.dynamic;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.confidence = data.mio.perception_obstacle.confidence;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.perception_type = data.mio.perception_obstacle.perception_type;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.v_x = data.mio.perception_obstacle.v_x;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.v_y = data.mio.perception_obstacle.v_y;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.v = data.mio.perception_obstacle.v;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.a_x = data.mio.perception_obstacle.a_x;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.a_y = data.mio.perception_obstacle.a_y;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.a = data.mio.perception_obstacle.a;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.valid = data.mio.perception_obstacle.proj_on_major_ref_line.valid;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.x = data.mio.perception_obstacle.proj_on_major_ref_line.x;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.y = data.mio.perception_obstacle.proj_on_major_ref_line.y;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.heading = data.mio.perception_obstacle.proj_on_major_ref_line.heading;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.curvature = data.mio.perception_obstacle.proj_on_major_ref_line.curvature;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.s = data.mio.perception_obstacle.proj_on_major_ref_line.s;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.proj_on_major_ref_line.l = data.mio.perception_obstacle.proj_on_major_ref_line.l;

  planning_debug_.VelPlanning.internal.mio.perception_obstacle.pred_path_num = data.mio.perception_obstacle.pred_path_num;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.pred_path_list.clear();

  for (Int32_t i = 0; i < data.mio.perception_obstacle.pred_path_num; ++i) {

    ::planning::pred_path_1d pred_path_1d;
    pred_path_1d.pred_path_point_num = data.mio.perception_obstacle.pred_path_point_num[i];
    for (Int32_t j = 0; j < pred_path_1d.pred_path_point_num; ++j) {
      pred_point.x = data.mio.perception_obstacle.pred_path[i][j].x;
      pred_point.y = data.mio.perception_obstacle.pred_path[i][j].y;
      pred_point.heading = data.mio.perception_obstacle.pred_path[i][j].heading;
      pred_point.s = data.mio.perception_obstacle.pred_path[i][j].s;

      planning_debug_.VelPlanning.internal.mio.perception_obstacle.pred_path_list[i].pred_path.push_back(pred_point);
    }
    planning_debug_.VelPlanning.internal.mio.perception_obstacle.pred_path_list.push_back(pred_path_1d);
  }

  planning_debug_.VelPlanning.internal.mio.perception_obstacle.tracked_path_point_num = data.mio.perception_obstacle.tracked_path_point_num;
  planning_debug_.VelPlanning.internal.mio.perception_obstacle.tracked_path.clear();
  for (Int32_t i = 0; i < data.mio.perception_obstacle.tracked_path_point_num; ++i) {
    tracked_point.x = data.mio.perception_obstacle.tracked_path[i].x;
    tracked_point.y = data.mio.perception_obstacle.tracked_path[i].y;

    planning_debug_.VelPlanning.internal.mio.perception_obstacle.tracked_path.push_back(tracked_point);
  }

  planning_debug_.VelPlanning.internal.mio.obj_position = data.mio.obj_position;
  planning_debug_.VelPlanning.internal.mio.obj_direction = data.mio.obj_direction;
  planning_debug_.VelPlanning.internal.mio.is_cutin = data.mio.is_cutin;
  planning_debug_.VelPlanning.internal.mio.obb_distance = data.mio.obb_distance;
  planning_debug_.VelPlanning.internal.mio.need_process = data.mio.need_process;
  planning_debug_.VelPlanning.internal.mio.can_following = data.mio.can_following;
  planning_debug_.VelPlanning.internal.mio.is_cipv = data.mio.is_cipv;
  planning_debug_.VelPlanning.internal.mio.need_avoiding = data.mio.need_avoiding;
  planning_debug_.VelPlanning.internal.mio.is_mio = data.mio.is_mio;
  planning_debug_.VelPlanning.internal.mio.need_aeb_warning = data.mio.need_aeb_warning;
  planning_debug_.VelPlanning.internal.mio.is_aeb_warning_target = data.mio.is_aeb_warning_target;
  planning_debug_.VelPlanning.internal.mio.need_aeb_full_brake = data.mio.need_aeb_full_brake;
  planning_debug_.VelPlanning.internal.mio.is_aeb_full_brake_target = data.mio.is_aeb_full_brake_target;
  planning_debug_.VelPlanning.internal.mio.dist_to_obj = data.mio.dist_to_obj;
  planning_debug_.VelPlanning.internal.mio.safe_dist = data.mio.safe_dist;
  planning_debug_.VelPlanning.internal.mio.v_for_plan = data.mio.v_for_plan;
  planning_debug_.VelPlanning.internal.mio.time_gap = data.mio.time_gap;
  planning_debug_.VelPlanning.internal.mio.ttc = data.mio.ttc;
  planning_debug_.VelPlanning.internal.mio.planning_target_v = data.mio.planning_target_v;
  planning_debug_.VelPlanning.internal.mio.planning_target_s = data.mio.planning_target_s;

  planning_debug_.VelPlanning.internal.tar_type_aeb = data.tar_type_aeb;
  planning_debug_.VelPlanning.internal.tar_v_aeb = data.tar_v_aeb;
  planning_debug_.VelPlanning.internal.tar_a_aeb = data.tar_a_aeb;

  ////////// 变道路径障碍物处理
  planning_debug_.VelPlanning.internal.collision_test_path_point_num_lca = data.collision_test_path_point_num_lca;
  planning_debug_.VelPlanning.internal.collision_test_path_lca.clear();
  planning_debug_.VelPlanning.internal.collision_test_path_lca.reserve(data.collision_test_path_point_num_lca);
  for (Int32_t i = 0; i < data.collision_test_path_point_num_lca; ++i) {
    pathPoint.x = data.collision_test_path_lca[i].point.x();
    pathPoint.y = data.collision_test_path_lca[i].point.y();
    pathPoint.heading = data.collision_test_path_lca[i].heading;
    pathPoint.curvature = data.collision_test_path_lca[i].curvature;
    pathPoint.s = data.collision_test_path_lca[i].s;
    pathPoint.l = data.collision_test_path_lca[i].l;

    planning_debug_.VelPlanning.internal.collision_test_path_lca.push_back(pathPoint);
  }

  planning_debug_.VelPlanning.internal.collision_test_samples_point_num_lca = data.collision_test_samples_point_num_lca;
  planning_debug_.VelPlanning.internal.collision_test_samples_lca.clear();
  planning_debug_.VelPlanning.internal.collision_test_samples_lca.reserve(data.collision_test_samples_point_num_lca);
  for (Int32_t i = 0; i < data.collision_test_samples_point_num_lca; ++i) {
    pathPoint.x = data.collision_test_samples_lca[i].point.x();
    pathPoint.y = data.collision_test_samples_lca[i].point.y();
    pathPoint.heading = data.collision_test_samples_lca[i].heading;
    pathPoint.curvature = data.collision_test_samples_lca[i].curvature;
    pathPoint.s = data.collision_test_samples_lca[i].s;
    pathPoint.l = data.collision_test_samples_lca[i].l;

    planning_debug_.VelPlanning.internal.collision_test_samples_lca.push_back(pathPoint);
  }

  // CIPV
  planning_debug_.VelPlanning.internal.cipv_lca.is_risk = data.cipv_lca.is_risk;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.id = data.cipv_lca.perception_obstacle.id;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.x = data.cipv_lca.perception_obstacle.x;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.y = data.cipv_lca.perception_obstacle.y;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.obb.x = data.cipv_lca.perception_obstacle.obb.x;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.obb.y = data.cipv_lca.perception_obstacle.obb.y;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.obb.heading = data.cipv_lca.perception_obstacle.obb.heading;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.obb.half_width = data.cipv_lca.perception_obstacle.obb.half_width;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.obb.half_length = data.cipv_lca.perception_obstacle.obb.half_length;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.height = data.cipv_lca.perception_obstacle.height;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.height_to_ground = data.cipv_lca.perception_obstacle.height_to_ground;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.type = data.cipv_lca.perception_obstacle.type;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.dynamic = data.cipv_lca.perception_obstacle.dynamic;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.confidence = data.cipv_lca.perception_obstacle.confidence;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.perception_type = data.cipv_lca.perception_obstacle.perception_type;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.v_x = data.cipv_lca.perception_obstacle.v_x;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.v_y = data.cipv_lca.perception_obstacle.v_y;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.v = data.cipv_lca.perception_obstacle.v;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.a_x = data.cipv_lca.perception_obstacle.a_x;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.a_y = data.cipv_lca.perception_obstacle.a_y;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.a = data.cipv_lca.perception_obstacle.a;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.valid = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.valid;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.x = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.x;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.y = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.y;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.heading = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.heading;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.curvature = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.curvature;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.s = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.s;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.proj_on_major_ref_line.l = data.cipv_lca.perception_obstacle.proj_on_major_ref_line.l;

  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.pred_path_num = data.cipv_lca.perception_obstacle.pred_path_num;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.pred_path_list.clear();

  for (Int32_t i = 0; i < data.cipv_lca.perception_obstacle.pred_path_num; ++i) {

    ::planning::pred_path_1d pred_path_1d;
    pred_path_1d.pred_path_point_num = data.cipv_lca.perception_obstacle.pred_path_point_num[i];
    for (Int32_t j = 0; j < pred_path_1d.pred_path_point_num; ++j) {
      pred_point.x = data.cipv_lca.perception_obstacle.pred_path[i][j].x;
      pred_point.y = data.cipv_lca.perception_obstacle.pred_path[i][j].y;
      pred_point.heading = data.cipv_lca.perception_obstacle.pred_path[i][j].heading;
      pred_point.s = data.cipv_lca.perception_obstacle.pred_path[i][j].s;

      planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.pred_path_list[i].pred_path.push_back(pred_point);
    }
    planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.pred_path_list.push_back(pred_path_1d);
  }

  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.tracked_path_point_num = data.cipv_lca.perception_obstacle.tracked_path_point_num;
  planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.tracked_path.clear();

  for (Int32_t i = 0; i < data.cipv_lca.perception_obstacle.tracked_path_point_num; ++i) {
    tracked_point.x = data.cipv_lca.perception_obstacle.tracked_path[i].x;
    tracked_point.y = data.cipv_lca.perception_obstacle.tracked_path[i].y;

    planning_debug_.VelPlanning.internal.cipv_lca.perception_obstacle.tracked_path.push_back(tracked_point);
  }

  planning_debug_.VelPlanning.internal.cipv_lca.obj_position = data.cipv_lca.obj_position;
  planning_debug_.VelPlanning.internal.cipv_lca.obj_direction = data.cipv_lca.obj_direction;
  planning_debug_.VelPlanning.internal.cipv_lca.is_cutin = data.cipv_lca.is_cutin;
  planning_debug_.VelPlanning.internal.cipv_lca.obb_distance = data.cipv_lca.obb_distance;
  planning_debug_.VelPlanning.internal.cipv_lca.need_process = data.cipv_lca.need_process;
  planning_debug_.VelPlanning.internal.cipv_lca.can_following = data.cipv_lca.can_following;
  planning_debug_.VelPlanning.internal.cipv_lca.is_cipv = data.cipv_lca.is_cipv;
  planning_debug_.VelPlanning.internal.cipv_lca.need_avoiding = data.cipv_lca.need_avoiding;
  planning_debug_.VelPlanning.internal.cipv_lca.is_mio = data.cipv_lca.is_mio;
  planning_debug_.VelPlanning.internal.cipv_lca.need_aeb_warning = data.cipv_lca.need_aeb_warning;
  planning_debug_.VelPlanning.internal.cipv_lca.is_aeb_warning_target = data.cipv_lca.is_aeb_warning_target;
  planning_debug_.VelPlanning.internal.cipv_lca.need_aeb_full_brake = data.cipv_lca.need_aeb_full_brake;
  planning_debug_.VelPlanning.internal.cipv_lca.is_aeb_full_brake_target = data.cipv_lca.is_aeb_full_brake_target;
  planning_debug_.VelPlanning.internal.cipv_lca.dist_to_obj = data.cipv_lca.dist_to_obj;
  planning_debug_.VelPlanning.internal.cipv_lca.safe_dist = data.cipv_lca.safe_dist;
  planning_debug_.VelPlanning.internal.cipv_lca.v_for_plan = data.cipv_lca.v_for_plan;
  planning_debug_.VelPlanning.internal.cipv_lca.time_gap = data.cipv_lca.time_gap;
  planning_debug_.VelPlanning.internal.cipv_lca.ttc = data.cipv_lca.ttc;
  planning_debug_.VelPlanning.internal.cipv_lca.planning_target_v = data.cipv_lca.planning_target_v;
  planning_debug_.VelPlanning.internal.cipv_lca.planning_target_s = data.cipv_lca.planning_target_s;

  planning_debug_.VelPlanning.internal.tar_type_following_lca = data.tar_type_following_lca;
  planning_debug_.VelPlanning.internal.tar_v_following_lca = data.tar_v_following_lca;
  planning_debug_.VelPlanning.internal.tar_a_following_lca = data.tar_a_following_lca;
  planning_debug_.VelPlanning.internal.fsm_follow_state_lca = data.fsm_follow_state_lca;

  // obstacle
  // TODO: 复用
  planning_debug_.VelPlanning.internal.avoidance_lca.is_risk = data.avoidance_lca.is_risk;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.id = data.avoidance_lca.perception_obstacle.id;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.x = data.avoidance_lca.perception_obstacle.x;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.y = data.avoidance_lca.perception_obstacle.y;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.obb.x = data.avoidance_lca.perception_obstacle.obb.x;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.obb.y = data.avoidance_lca.perception_obstacle.obb.y;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.obb.heading = data.avoidance_lca.perception_obstacle.obb.heading;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.obb.half_width = data.avoidance_lca.perception_obstacle.obb.half_width;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.obb.half_length = data.avoidance_lca.perception_obstacle.obb.half_length;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.height = data.avoidance_lca.perception_obstacle.height;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.height_to_ground = data.avoidance_lca.perception_obstacle.height_to_ground;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.type = data.avoidance_lca.perception_obstacle.type;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.dynamic = data.avoidance_lca.perception_obstacle.dynamic;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.confidence = data.avoidance_lca.perception_obstacle.confidence;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.perception_type = data.avoidance_lca.perception_obstacle.perception_type;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.v_x = data.avoidance_lca.perception_obstacle.v_x;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.v_y = data.avoidance_lca.perception_obstacle.v_y;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.v = data.avoidance_lca.perception_obstacle.v;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.a_x = data.avoidance_lca.perception_obstacle.a_x;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.a_y = data.avoidance_lca.perception_obstacle.a_y;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.a = data.avoidance_lca.perception_obstacle.a;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.valid = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.valid;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.x = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.x;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.y = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.y;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.heading = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.heading;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.curvature = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.curvature;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.s = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.s;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.proj_on_major_ref_line.l = data.avoidance_lca.perception_obstacle.proj_on_major_ref_line.l;

  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.pred_path_num = data.avoidance_lca.perception_obstacle.pred_path_num;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.pred_path_list.clear();
  for (Int32_t i = 0; i < data.avoidance_lca.perception_obstacle.pred_path_num; ++i) {

    ::planning::pred_path_1d pred_path_1d;
    pred_path_1d.pred_path_point_num = data.avoidance_lca.perception_obstacle.pred_path_point_num[i];
    for (Int32_t j = 0; j < pred_path_1d.pred_path_point_num; ++j) {
      pred_point.x = data.avoidance_lca.perception_obstacle.pred_path[i][j].x;
      pred_point.y = data.avoidance_lca.perception_obstacle.pred_path[i][j].y;
      pred_point.heading = data.avoidance_lca.perception_obstacle.pred_path[i][j].heading;
      pred_point.s = data.avoidance_lca.perception_obstacle.pred_path[i][j].s;

      planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.pred_path_list[i].pred_path.push_back(pred_point);
    }
    planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.pred_path_list.push_back(pred_path_1d);
  }

  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.tracked_path_point_num = data.avoidance_lca.perception_obstacle.tracked_path_point_num;
  planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.tracked_path.clear();
  for (Int32_t i = 0; i < data.avoidance_lca.perception_obstacle.tracked_path_point_num; ++i) {
    tracked_point.x = data.avoidance_lca.perception_obstacle.tracked_path[i].x;
    tracked_point.y = data.avoidance_lca.perception_obstacle.tracked_path[i].y;

    planning_debug_.VelPlanning.internal.avoidance_lca.perception_obstacle.tracked_path.push_back(tracked_point);
  }

  planning_debug_.VelPlanning.internal.avoidance_lca.obj_position = data.avoidance_lca.obj_position;
  planning_debug_.VelPlanning.internal.avoidance_lca.obj_direction = data.avoidance_lca.obj_direction;
  planning_debug_.VelPlanning.internal.avoidance_lca.is_cutin = data.avoidance_lca.is_cutin;
  planning_debug_.VelPlanning.internal.avoidance_lca.obb_distance = data.avoidance_lca.obb_distance;
  planning_debug_.VelPlanning.internal.avoidance_lca.need_process = data.avoidance_lca.need_process;
  planning_debug_.VelPlanning.internal.avoidance_lca.can_following = data.avoidance_lca.can_following;
  planning_debug_.VelPlanning.internal.avoidance_lca.is_cipv = data.avoidance_lca.is_cipv;
  planning_debug_.VelPlanning.internal.avoidance_lca.need_avoiding = data.avoidance_lca.need_avoiding;
  planning_debug_.VelPlanning.internal.avoidance_lca.is_mio = data.avoidance_lca.is_mio;
  planning_debug_.VelPlanning.internal.avoidance_lca.need_aeb_warning = data.avoidance_lca.need_aeb_warning;
  planning_debug_.VelPlanning.internal.avoidance_lca.is_aeb_warning_target = data.avoidance_lca.is_aeb_warning_target;
  planning_debug_.VelPlanning.internal.avoidance_lca.need_aeb_full_brake = data.avoidance_lca.need_aeb_full_brake;
  planning_debug_.VelPlanning.internal.avoidance_lca.is_aeb_full_brake_target = data.avoidance_lca.is_aeb_full_brake_target;
  planning_debug_.VelPlanning.internal.avoidance_lca.dist_to_obj = data.avoidance_lca.dist_to_obj;
  planning_debug_.VelPlanning.internal.avoidance_lca.safe_dist = data.avoidance_lca.safe_dist;
  planning_debug_.VelPlanning.internal.avoidance_lca.v_for_plan = data.avoidance_lca.v_for_plan;
  planning_debug_.VelPlanning.internal.avoidance_lca.time_gap = data.avoidance_lca.time_gap;
  planning_debug_.VelPlanning.internal.avoidance_lca.ttc = data.avoidance_lca.ttc;
  planning_debug_.VelPlanning.internal.avoidance_lca.planning_target_v = data.avoidance_lca.planning_target_v;
  planning_debug_.VelPlanning.internal.avoidance_lca.planning_target_s = data.avoidance_lca.planning_target_s;

  planning_debug_.VelPlanning.internal.tar_type_obstacle_lca = data.tar_type_obstacle_lca;
  planning_debug_.VelPlanning.internal.tar_v_obstacle_lca = data.tar_v_obstacle_lca;
  planning_debug_.VelPlanning.internal.tar_a_obstacle_lca = data.tar_a_obstacle_lca;
  planning_debug_.VelPlanning.internal.fsm_obstacle_state_lca = data.fsm_obstacle_state_lca;

  // AEB
  planning_debug_.VelPlanning.internal.mio_lca.is_risk = data.mio_lca.is_risk;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.id = data.mio_lca.perception_obstacle.id;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.x = data.mio_lca.perception_obstacle.x;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.y = data.mio_lca.perception_obstacle.y;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.obb.x = data.mio_lca.perception_obstacle.obb.x;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.obb.y = data.mio_lca.perception_obstacle.obb.y;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.obb.heading = data.mio_lca.perception_obstacle.obb.heading;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.obb.half_width = data.mio_lca.perception_obstacle.obb.half_width;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.obb.half_length = data.mio_lca.perception_obstacle.obb.half_length;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.height = data.mio_lca.perception_obstacle.height;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.height_to_ground = data.mio_lca.perception_obstacle.height_to_ground;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.type = data.mio_lca.perception_obstacle.type;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.dynamic = data.mio_lca.perception_obstacle.dynamic;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.confidence = data.mio_lca.perception_obstacle.confidence;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.perception_type = data.mio_lca.perception_obstacle.perception_type;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.v_x = data.mio_lca.perception_obstacle.v_x;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.v_y = data.mio_lca.perception_obstacle.v_y;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.v = data.mio_lca.perception_obstacle.v;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.a_x = data.mio_lca.perception_obstacle.a_x;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.a_y = data.mio_lca.perception_obstacle.a_y;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.a = data.mio_lca.perception_obstacle.a;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.valid = data.mio_lca.perception_obstacle.proj_on_major_ref_line.valid;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.x = data.mio_lca.perception_obstacle.proj_on_major_ref_line.x;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.y = data.mio_lca.perception_obstacle.proj_on_major_ref_line.y;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.heading = data.mio_lca.perception_obstacle.proj_on_major_ref_line.heading;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.curvature = data.mio_lca.perception_obstacle.proj_on_major_ref_line.curvature;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.s = data.mio_lca.perception_obstacle.proj_on_major_ref_line.s;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.proj_on_major_ref_line.l = data.mio_lca.perception_obstacle.proj_on_major_ref_line.l;

  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.pred_path_num = data.mio_lca.perception_obstacle.pred_path_num;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.pred_path_list.clear();

  for (Int32_t i = 0; i < data.mio_lca.perception_obstacle.pred_path_num; ++i) {

    ::planning::pred_path_1d pred_path_1d;
    pred_path_1d.pred_path_point_num = data.mio_lca.perception_obstacle.pred_path_point_num[i];
    for (Int32_t j = 0; j < pred_path_1d.pred_path_point_num; ++j) {
      pred_point.x = data.mio_lca.perception_obstacle.pred_path[i][j].x;
      pred_point.y = data.mio_lca.perception_obstacle.pred_path[i][j].y;
      pred_point.heading = data.mio_lca.perception_obstacle.pred_path[i][j].heading;
      pred_point.s = data.mio_lca.perception_obstacle.pred_path[i][j].s;

      planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.pred_path_list[i].pred_path.push_back(pred_point);
    }
    planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.pred_path_list.push_back(pred_path_1d);
  }

  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.tracked_path_point_num = data.mio_lca.perception_obstacle.tracked_path_point_num;
  planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.tracked_path.clear();
  for (Int32_t i = 0; i < data.mio_lca.perception_obstacle.tracked_path_point_num; ++i) {
    tracked_point.x = data.mio_lca.perception_obstacle.tracked_path[i].x;
    tracked_point.y = data.mio_lca.perception_obstacle.tracked_path[i].y;

    planning_debug_.VelPlanning.internal.mio_lca.perception_obstacle.tracked_path.push_back(tracked_point);
  }

  planning_debug_.VelPlanning.internal.mio_lca.obj_position = data.mio_lca.obj_position;
  planning_debug_.VelPlanning.internal.mio_lca.obj_direction = data.mio_lca.obj_direction;
  planning_debug_.VelPlanning.internal.mio_lca.is_cutin = data.mio_lca.is_cutin;
  planning_debug_.VelPlanning.internal.mio_lca.obb_distance = data.mio_lca.obb_distance;
  planning_debug_.VelPlanning.internal.mio_lca.need_process = data.mio_lca.need_process;
  planning_debug_.VelPlanning.internal.mio_lca.can_following = data.mio_lca.can_following;
  planning_debug_.VelPlanning.internal.mio_lca.is_cipv = data.mio_lca.is_cipv;
  planning_debug_.VelPlanning.internal.mio_lca.need_avoiding = data.mio_lca.need_avoiding;
  planning_debug_.VelPlanning.internal.mio_lca.is_mio = data.mio_lca.is_mio;
  planning_debug_.VelPlanning.internal.mio_lca.need_aeb_warning = data.mio_lca.need_aeb_warning;
  planning_debug_.VelPlanning.internal.mio_lca.is_aeb_warning_target = data.mio_lca.is_aeb_warning_target;
  planning_debug_.VelPlanning.internal.mio_lca.need_aeb_full_brake = data.mio_lca.need_aeb_full_brake;
  planning_debug_.VelPlanning.internal.mio_lca.is_aeb_full_brake_target = data.mio_lca.is_aeb_full_brake_target;
  planning_debug_.VelPlanning.internal.mio_lca.dist_to_obj = data.mio_lca.dist_to_obj;
  planning_debug_.VelPlanning.internal.mio_lca.safe_dist = data.mio_lca.safe_dist;
  planning_debug_.VelPlanning.internal.mio_lca.v_for_plan = data.mio_lca.v_for_plan;
  planning_debug_.VelPlanning.internal.mio_lca.time_gap = data.mio_lca.time_gap;
  planning_debug_.VelPlanning.internal.mio_lca.ttc = data.mio_lca.ttc;
  planning_debug_.VelPlanning.internal.mio_lca.planning_target_v = data.mio_lca.planning_target_v;
  planning_debug_.VelPlanning.internal.mio_lca.planning_target_s = data.mio_lca.planning_target_s;

  planning_debug_.VelPlanning.internal.tar_type_aeb_lca = data.tar_type_aeb_lca;
  planning_debug_.VelPlanning.internal.tar_v_aeb_lca = data.tar_v_aeb_lca;
  planning_debug_.VelPlanning.internal.tar_a_aeb_lca = data.tar_a_aeb_lca;

  planning_debug_.VelPlanning.internal.tar_type_real_obstacle = data.tar_type_real_obstacle;
  planning_debug_.VelPlanning.internal.tar_v_real_obstacle = data.tar_v_real_obstacle;
  planning_debug_.VelPlanning.internal.tar_a_real_obstacle = data.tar_a_real_obstacle;

  planning_debug_.VelPlanning.internal.tar_type_before_final = data.tar_type_before_final;
  planning_debug_.VelPlanning.internal.is_pcc_from_road = data.pacc_enable;
  planning_debug_.VelPlanning.internal.tar_v_before_final = data.tar_v_before_final;
  planning_debug_.VelPlanning.internal.tar_a_before_final = data.tar_a_before_final;

  // 平滑
  planning_debug_.VelPlanning.internal.tar_v_his_init = data.tar_v_his_init;
  planning_debug_.VelPlanning.internal.tar_v_history[0] = data.tar_v_history[0];
  planning_debug_.VelPlanning.internal.tar_v_history[1] = data.tar_v_history[1];
  planning_debug_.VelPlanning.internal.tar_v_history[2] = data.tar_v_history[2];
  planning_debug_.VelPlanning.internal.tar_a_history[0] = data.tar_a_history[0];
  planning_debug_.VelPlanning.internal.tar_a_history[1] = data.tar_a_history[1];
  planning_debug_.VelPlanning.internal.tar_a_history[2] = data.tar_a_history[2];
  planning_debug_.VelPlanning.internal.tar_v_smoothed = data.tar_v_smoothed;
  planning_debug_.VelPlanning.internal.tar_a_smoothed = data.tar_a_smoothed;

  planning_debug_.VelPlanning.internal.error_code = data.error_code;
  planning_debug_.VelPlanning.internal.tar_type_final = data.tar_type_final;
  planning_debug_.VelPlanning.internal.tar_v_final = data.tar_v_final;
  planning_debug_.VelPlanning.internal.tar_a_final = data.tar_a_final;

  // PCC
  planning_debug_.VelPlanning.internal.pacc.enable = data.pacc.enable;
  planning_debug_.VelPlanning.internal.pacc.valid = data.pacc.valid;
  
  planning_debug_.VelPlanning.internal.pacc.tar_throttle_ufilter = data.pacc.tar_throttle_ufilter;
  planning_debug_.VelPlanning.internal.pacc.cur_slope_ufilter = data.pacc.cur_slope_ufilter;
  planning_debug_.VelPlanning.internal.pacc.engine_speed_ufilter = data.pacc.engine_speed_ufilter;

  planning_debug_.VelPlanning.internal.pacc.slope_dec = data.pacc.slope_dec;

  planning_debug_.VelPlanning.internal.pacc.slope_smooth = data.pacc.slope_smooth;
  planning_debug_.VelPlanning.internal.pacc.slope_length = data.pacc.slope_length;
  planning_debug_.VelPlanning.internal.pacc.distance_to_slope = data.pacc.distance_to_slope;
  planning_debug_.VelPlanning.internal.pacc.cc_upper_limit_v = data.pacc.cc_upper_limit_v;

  planning_debug_.VelPlanning.internal.pacc.chassis_input.throttle_sys_status = data.pacc.chassis_input.throttle_sys_status;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.ebs_status = data.pacc.chassis_input.ebs_status;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.acc_pedal_value = data.pacc.chassis_input.acc_pedal_value;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.brake_pedal_value = data.pacc.chassis_input.brake_pedal_value;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.cruise_speed = data.pacc.chassis_input.cruise_speed;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.engine_speed = data.pacc.chassis_input.engine_speed;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.engine_torque = data.pacc.chassis_input.engine_torque;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.gear_number = data.pacc.chassis_input.gear_number;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.ego_speed = data.pacc.chassis_input.ego_speed;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.ego_accel = data.pacc.chassis_input.ego_accel;
  planning_debug_.VelPlanning.internal.pacc.chassis_input.ego_mass = data.pacc.chassis_input.ego_mass;

  planning_debug_.VelPlanning.internal.pacc.chassis_input.engine_torque_limit = data.pacc.chassis_input.engine_torque_limit;
  
  planning_debug_.VelPlanning.internal.pacc.map_input.gnss_status = data.pacc.map_input.gnss_status;
  planning_debug_.VelPlanning.internal.pacc.map_input.longitude = data.pacc.map_input.longitude;
  planning_debug_.VelPlanning.internal.pacc.map_input.latitude = data.pacc.map_input.latitude;
  planning_debug_.VelPlanning.internal.pacc.map_input.altitude = data.pacc.map_input.altitude;
  planning_debug_.VelPlanning.internal.pacc.map_input.pitch = data.pacc.map_input.pitch;
  planning_debug_.VelPlanning.internal.pacc.map_input.yaw = data.pacc.map_input.yaw;
  planning_debug_.VelPlanning.internal.pacc.map_input.roll = data.pacc.map_input.roll;
  planning_debug_.VelPlanning.internal.pacc.map_input.odom_status = data.pacc.map_input.odom_status;
  planning_debug_.VelPlanning.internal.pacc.map_input.x_odom = data.pacc.map_input.x_odom;
  planning_debug_.VelPlanning.internal.pacc.map_input.y_odom = data.pacc.map_input.y_odom;
  planning_debug_.VelPlanning.internal.pacc.map_input.z_odom = data.pacc.map_input.z_odom;
  planning_debug_.VelPlanning.internal.pacc.map_input.heading_odom = data.pacc.map_input.heading_odom;
  planning_debug_.VelPlanning.internal.pacc.map_input.valid = data.pacc.map_input.valid;
  planning_debug_.VelPlanning.internal.pacc.map_input.pacc_points_source = data.pacc.map_input.pacc_points_source;
  planning_debug_.VelPlanning.internal.pacc.map_input.pacc_point_s_step = data.pacc.map_input.pacc_point_s_step;
  planning_debug_.VelPlanning.internal.pacc.map_input.pacc_point_num = data.pacc.map_input.pacc_point_num;
  planning_debug_.VelPlanning.internal.pacc.map_input.pacc_points.clear();
  // Plotjuggler最多只显示100个元素
  int pcc_points = data.pacc.map_input.pacc_point_num > 100 ? 100 : data.pacc.map_input.pacc_point_num;
  ::planning::pcc_point point;
  for(int i = 0; i < pcc_points; ++i) {
    point.s = data.pacc.map_input.pacc_points[i].s;
    point.slope = data.pacc.map_input.pacc_points[i].slope;
    point.curvature = data.pacc.map_input.pacc_points[i].curvature;
    point.speed_limit = data.pacc.map_input.pacc_points[i].speed_limit;
    
    planning_debug_.VelPlanning.internal.pacc.map_input.pacc_points.push_back(point);
  }
  planning_debug_.VelPlanning.internal.pacc.map_input.cur_slope = data.pacc.map_input.cur_slope;
  planning_debug_.VelPlanning.internal.pacc.map_input.preview_slope = data.pacc.map_input.preview_slope;
  planning_debug_.VelPlanning.internal.pacc.map_input.prediction_map_length = data.pacc.map_input.prediction_map_length;
  planning_debug_.VelPlanning.internal.pacc.map_input.total_map_length = data.pacc.map_input.total_map_length;
  
  planning_debug_.VelPlanning.internal.pacc.internal.best_lambda = data.pacc.internal.best_lambda;
  planning_debug_.VelPlanning.internal.pacc.internal.Final_J = data.pacc.internal.Final_J;
  planning_debug_.VelPlanning.internal.pacc.internal.Final_Q = data.pacc.internal.Final_Q;
  planning_debug_.VelPlanning.internal.pacc.internal.iteration_count = data.pacc.internal.iteration_count;
  planning_debug_.VelPlanning.internal.pacc.internal.torque_max_base_cur_engine_speed = data.pacc.internal.torque_max_base_cur_engine_speed;
  planning_debug_.VelPlanning.internal.pacc.internal.torque_min_base_cur_engine_speed = data.pacc.internal.torque_min_base_cur_engine_speed;

  planning_debug_.VelPlanning.internal.pacc.internal.tar_sample_te = data.pacc.internal.tar_sample_te;
  planning_debug_.VelPlanning.internal.pacc.internal.tar_sample_ft = data.pacc.internal.tar_sample_ft;
  planning_debug_.VelPlanning.internal.pacc.internal.tar_sample_a = data.pacc.internal.tar_sample_a;
  planning_debug_.VelPlanning.internal.pacc.internal.tar_sample_v = data.pacc.internal.tar_sample_v;
  planning_debug_.VelPlanning.internal.pacc.internal.tar_sample_ne = data.pacc.internal.tar_sample_ne;
  planning_debug_.VelPlanning.internal.pacc.internal.tar_sample_throttle = data.pacc.internal.tar_sample_throttle;

  planning_debug_.VelPlanning.internal.pacc.internal.tar_throttle_from_table = data.pacc.internal.tar_throttle_from_table;

  planning_debug_.VelPlanning.internal.pacc.internal.pcc_best_point_num = data.pacc.internal.pcc_best_point_num;
  planning_debug_.VelPlanning.internal.pacc.internal.pcc_best_points.clear();
  ::planning::mpc_best_iteration_point iteration_point;
  for(int i = 0; i < data.pacc.internal.pcc_best_point_num; ++i) {
    iteration_point.station = data.pacc.internal.pcc_best_points[i].station;
    iteration_point.slope_alpha = data.pacc.internal.pcc_best_points[i].slope_alpha * 100;
    iteration_point.ft = data.pacc.internal.pcc_best_points[i].ft;
    iteration_point.velocity = data.pacc.internal.pcc_best_points[i].velocity;
    iteration_point.lambda = data.pacc.internal.pcc_best_points[i].lambda;
    
    planning_debug_.VelPlanning.internal.pacc.internal.pcc_best_points.push_back(iteration_point);
  }

  planning_debug_.VelPlanning.internal.pacc.output.tar_v = data.pacc.output.tar_v;
  planning_debug_.VelPlanning.internal.pacc.output.target_unlimit_vel = data.pacc.output.target_unlimit_vel;
  planning_debug_.VelPlanning.internal.pacc.output.tar_a = data.pacc.output.tar_a;
  planning_debug_.VelPlanning.internal.pacc.output.tar_gear = data.pacc.output.tar_gear;
  planning_debug_.VelPlanning.internal.pacc.output.tar_engine_torque = data.pacc.output.tar_engine_torque;
  planning_debug_.VelPlanning.internal.pacc.output.tar_drive_force = data.pacc.output.tar_drive_force;
  planning_debug_.VelPlanning.internal.pacc.output.cur_resistance = data.pacc.output.cur_resistance;
  planning_debug_.VelPlanning.internal.pacc.output.cur_Ff = data.pacc.output.cur_Ff;
  planning_debug_.VelPlanning.internal.pacc.output.cur_Fw = data.pacc.output.cur_Fw;
  planning_debug_.VelPlanning.internal.pacc.output.cur_Fi = data.pacc.output.cur_Fi;  
  planning_debug_.VelPlanning.internal.pacc.output.tar_engine_ne = data.pacc.output.tar_engine_ne;
  planning_debug_.VelPlanning.internal.pacc.output.tar_throttle = data.pacc.output.tar_throttle;
  planning_debug_.VelPlanning.internal.pacc.output.pcc_vel_plan_type = data.pacc.output.pcc_vel_plan_type;
  planning_debug_.VelPlanning.internal.pacc.output.dichotomy_solution_state = data.pacc.output.dichotomy_solution_state;
  planning_debug_.VelPlanning.internal.pacc.output.mpc_solution_state = data.pacc.output.mpc_solution_state;
  planning_debug_.VelPlanning.internal.pacc.output.pcc_result_vaild = data.pacc.output.pcc_result_vaild;

}
#endif
}  // namespace framework
}  // namespace phoenix
