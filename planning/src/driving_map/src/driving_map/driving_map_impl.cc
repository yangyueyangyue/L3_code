/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       driving_map_impl.cc
 * @brief      驾驶地图对外接口的实现类
 * @details    驾驶地图对外接口的具体实现，主要是通过调用内部成员变量的对应接口。
 *
 * @author     boc
 * @date       2020.06.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/19  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "driving_map/driving_map_impl.h"
#include "pos_filter_wrapper.h"
#include "vehicle_model_wrapper.h"

// 开启性能测试
#define ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST (0)

// 强制相机识别的车道线全部平行
#define ENFORCE_CAMERA_LANE_TO_PARALLEL (1)


namespace phoenix {
namespace driv_map {


DrivingMapImpl::DrivingMapImpl() {
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
  // 外接圆半径
  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0.0F, 0.0F), 0.0F, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();

  param_.pred_path_sample_time = 0.5F;
  param_.pred_path_max_sample_len = 10.0F;

  // 选取跟车目标时，目标与参考线之间的横向距离的限制表(距离越远，允许的横向距离越大)
  param_.following_target_lat_dist_limit_table.Clear();
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(0.0F, 0.8F));
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(5.0F, 1.0F));
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(10.0F, 1.0F));
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(20.0F, 1.0F));
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(50.0F, 1.0F));
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(100.0F, 1.0F));
  param_.following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(200.0F, 1.0F));

  param_.loose_following_target_lat_dist_limit_table.Clear();
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(0.0F, 1.5F));
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(5.0F, 2.0F));
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(10.0F, 2.0F));
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(20.0F, 2.0F));
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(50.0F, 2.0F));
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(100.0F, 2.0F));
  param_.loose_following_target_lat_dist_limit_table.PushBack(
        common::LerpTableNodeType1(200.0F, 2.0F));

  mat_convert_global_to_rel_coor_.SetIdentity();
  Clear();

  action_smoothing_camera_lane_lost_.set_req_count_threshold(5);
  action_smoothing_camera_lane_lost_.set_time_allowed(10);

#if 0
  std::cout << "sizeof(HDMap)="
            << sizeof(HDMap) / (1024.0*1024.0)
            << " M bytes"
            << std::endl;

  std::cout << "sizeof(ObjectMapImpl)="
            << sizeof(ObjectMapImpl) / (1024.0*1024.0)
            << " M bytes"
            << std::endl;
#endif
}

DrivingMapImpl::~DrivingMapImpl() {
}

void DrivingMapImpl::Configurate(const DrivingMapConfig& conf) {
  config_ = conf;
}

void DrivingMapImpl::Clear() {
  driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
  driving_direction_ = DRIVING_DIRECTION_FORWARD;

#if (ENABLE_MIXED_MAP_SOURCE)
  valid_raw_map_ = false;
  map_space_raw_map_.Clear();
  delta_pos_corrected_gnss_[0] = 0.0F;
  delta_pos_corrected_gnss_[1] = 0.0F;
  delta_pos_corrected_gnss_[2] = 0.0F;
  mat_convert_corrected_gnss_.SetIdentity();
  mat_convert_corrected_gnss_updated_ = false;

  map_space_pcc_map_.Clear();
  reference_line_set_pcc_map_.Clear();
#endif
  map_space_.Clear();

  sample_points_of_pred_path_.Clear();

#if (ENABLE_MIXED_MAP_SOURCE)
  reference_line_set_raw_map_.Clear();
#endif
  reference_line_set_.Clear();

  for (Int32_t i = 0; i < road_boundary_.Size(); ++i) {
    road_boundary_[i].Clear();
  }
  road_boundary_.Clear();
  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
#if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
  road_boundary_map_.Clear();
#endif
  /* k005 pengc 2023-01-06 (end) */

  obstacle_list_.Clear();
  obstacle_map_.Clear();

  for (Int32_t i = 0; i < ref_line_association_list_.Size(); ++i) {
    ref_line_association_list_[i].Clear();
  }
  ref_line_association_list_.Clear();

  scene_story_set_.Clear();
  planning_story_set_.Clear();
}

bool DrivingMapImpl::Update(const DrivingMapDataSource& data_source) {
  LOG_INFO(5) << "\n\n$$$$$$         Update driving map...";

  if (Nullptr_t == data_source.chassis) {
    return false;
  }
  if (!data_source.chassis->msg_head.valid) {
    return false;
  }
  if (Nullptr_t == data_source.rel_pos_list) {
    return false;
  }
  if (!data_source.rel_pos_list->msg_head.valid) {
    return false;
  }
  // Save gnss
  bool gnss_is_valid = false;
  if (Nullptr_t != data_source.gnss) {
    if (data_source.gnss->msg_head.valid) {
      gnss_is_valid = true;
      gnss_ = *data_source.gnss;
    }
  }
  // Save imu
  if (Nullptr_t != data_source.imu) {
    imu_ = *data_source.imu;
  }
  // Save chassis information
  if (Nullptr_t != data_source.chassis) {
    chassis_ = *data_source.chassis;
  }
  // Save traffic signal
  if (Nullptr_t != data_source.traffic_signal_list) {
    traffic_signal_list_ = *data_source.traffic_signal_list;
  }

  // 更新车身参数
  UpdateVehicleParameter();

#if (ENABLE_MIXED_MAP_SOURCE)
  valid_raw_map_ = false;
  delta_pos_corrected_gnss_[0] = 0.0F;
  delta_pos_corrected_gnss_[1] = 0.0F;
  delta_pos_corrected_gnss_[2] = 0.0F;
  mat_convert_corrected_gnss_.SetIdentity();
  mat_convert_corrected_gnss_updated_ = false;
#endif
  driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
  event_reporting_list_.Clear();
  if (ad_msg::VEH_GEAR_R == chassis_.gear) {
    // For backing mode of vehicle
    driving_direction_ = DRIVING_DIRECTION_BACKWARD;
  } else {
    driving_direction_ = DRIVING_DIRECTION_FORWARD;
  }

  // 更新位置信息
  Int32_t location_status = data_source.gnss->utm_status;
  if (GNSS_COORDINATE_TYPE_ODOM == data_source.gnss_coordinate_type) {
    location_status = data_source.gnss->odom_status;
  }
  if (!UpdatePositionInfo(data_source, gnss_is_valid)) {
    gnss_is_valid = false;
    location_status = ad_msg::Gnss::STATUS_INVALID;
  }

  // 根据当前车辆状态，预测未来的行驶轨迹
  CalcPredictedPathOfVehicle();

  // 选择地图构建类型
  driving_map_type_ = SelectDrivingMapType(data_source, location_status);
  // printf("driving_map_type_ = %d\n", driving_map_type_);
  // 构建内部地图
  if (DRIVING_MAP_TYPE_PRED_PATH == driving_map_type_) {
    // Construct internal map from predicted path
    if (!map_space_.ConstructMapInfoFromPath(sample_points_of_pred_path_)) {
      driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
      LOG_ERR << "Failed to construct map from predicted path.";
    }
  } else if (DRIVING_MAP_TYPE_FOLLOWING_PATH == driving_map_type_) {
    // Construct internal map from following path
    if(!map_space_.ConstructMapInfoFromPath(
         following_path_info_.following_path)) {
      driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
      LOG_ERR << "Failed to construct map from following path.";
    }
  } else if (DRIVING_MAP_TYPE_CAMERA_LANE == driving_map_type_) {
    // Construct internal map from camera lanes
    LOG_INFO(5) << "### Construct map using camera lanes";
    if (!map_space_.ConstructMapInfoFromCamera(cam_info_.lane_list)) {
      driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
      LOG_ERR << "Failed to construct Camera Map.";
    }
  } else if (DRIVING_MAP_TYPE_HD_MAP == driving_map_type_) {
    // Construct internal map from HDMap
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO) || \
    (HD_MAP_TYPE == HD_MAP_TYPE_D17)
 #if (ENABLE_MIXED_MAP_SOURCE)
    if (mat_convert_corrected_gnss_updated_) {
      LOG_INFO(5) << "### Construct map using Mixed HD-Map in HD-Map mode.";
      if (map_space_.ConstructMapInfoFromOtherMap(
            map_space_raw_map_, mat_convert_corrected_gnss_,
            &current_cam_center_path_)) {
        if (Nullptr_t != data_source.routing) {
          map_space_.ConstructRoutingInfo(*data_source.routing);
        }
      } else {
        // LOG_ERR << "Failed to construct HD Map.";
        driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
      }
    } else {
      LOG_INFO(5) << "### Construct map using HD-Map.";
      if (map_space_.ConstructMapInfo(*data_source.map)) {
        if (Nullptr_t != data_source.routing) {
          map_space_.ConstructRoutingInfo(*data_source.routing);
        }
      } else {
        driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
        LOG_ERR << "Failed to construct HD Map.";
      }
    }
  #else
    LOG_INFO(5) << "### Construct map using HD-Map.";
    if (map_space_.ConstructMapInfo(*data_source.map)) {
      if (Nullptr_t != data_source.routing) {
        map_space_.ConstructRoutingInfo(*data_source.routing);
      }
    } else {
      driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
      LOG_ERR << "Failed to construct HD Map.";
    }
  #endif
#else
    // ERROR: No map supported.;
    LOG_ERR << "No map supported.";
#endif
  } else if (DRIVING_MAP_TYPE_MIXED_HD_MAP == driving_map_type_) {
#if (ENABLE_MIXED_MAP_SOURCE)
    LOG_INFO(5) << "### Construct map using Mixed HD-Map.";
    if (map_space_.ConstructMapInfoFromOtherMap(
          map_space_raw_map_, mat_convert_corrected_gnss_,
          &current_cam_center_path_)) {
      if (Nullptr_t != data_source.routing) {
        map_space_.ConstructRoutingInfo(*data_source.routing);
      }
    } else {
      // LOG_ERR << "Failed to construct HD Map.";
      driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
    }
#else
    LOG_ERR << "Can't support mixed map.";
    driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
#endif
  } else {
    driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
    // LOG_ERR << "Invalid driving map type.";
  }

  if (DRIVING_MAP_TYPE_INVALID == driving_map_type_) {
    // Construct internal map from predicted path
    if (map_space_.ConstructMapInfoFromPath(sample_points_of_pred_path_)) {
      driving_map_type_ = DRIVING_MAP_TYPE_PRED_PATH;
    } else {
      driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
      LOG_ERR << "Failed to construct map from predicted path.";
    }
  }

  // Construct reference lines
  common::Vec2d veh_pos(current_rel_position_.x, current_rel_position_.y);
  map_var_t veh_heading = current_rel_position_.heading;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    veh_heading = common::NormalizeAngle(veh_heading + COM_PI);
  }
  if (DRIVING_MAP_TYPE_INVALID != driving_map_type_) {
    bool valid_ref_lines = false;
    if (reference_line_set_.Construct(veh_pos, veh_heading, map_space_)) {
      valid_ref_lines = true;
      /* k004 pengc 2022-12-26 (begin) */
      // 已经替换了地图中的车道中心线，不需要再替换参考线了
#if 0
#if (ENABLE_MIXED_MAP_SOURCE)
      if (DRIVING_MAP_TYPE_MIXED_HD_MAP == driving_map_type_) {
        map_var_t trim_start_s = 0.0F;
        map_var_t trim_end_s = 0.0F;
        /// TODO: 暂时必须传递当前的车辆位置（因为Trim函数中将更新当前车辆位置的投影点）
        /// TODO: 2022-12-28 可能存在相机的当前车道线与地图的当前车道线不一致的情况，需要避免此种情况
        if (!reference_line_set_.TrimReferenceLine(
              reference_line_set_.GetMajorRefLineIndex(),
              current_cam_center_path_, veh_pos, &trim_start_s, &trim_end_s)) {
          LOG_ERR << "Failed to trim reference line.";
          valid_ref_lines = false;
        }
      }
#endif
#endif
      /* k004 pengc 2022-12-26 (end) */

    } else {
      LOG_ERR << "Failed to construct reference lines.";
      /* longjiaoy 混合模式内部异常修复 2024-04-10 (start) */
      if ( DRIVING_MAP_TYPE_MIXED_HD_MAP == driving_map_type_) {
        if (map_space_.ConstructMapInfoFromCamera(cam_info_.lane_list)) {
          if (reference_line_set_.Construct(veh_pos, veh_heading, map_space_)) {
            valid_ref_lines = true;
            driving_map_type_ = DRIVING_MAP_TYPE_CAMERA_LANE;
          }
        }
        if (!valid_ref_lines) {
          if (valid_raw_map_) {
            if (reference_line_set_.Construct(veh_pos, veh_heading, map_space_raw_map_)) {
              valid_ref_lines = true;
              driving_map_type_ = DRIVING_MAP_TYPE_HD_MAP;
            }
          }
        }
      }
      /* longjiaoy 混合模式内部异常修复 2024-04-10 (end) */
    }
    if (!valid_ref_lines) {
      if (map_space_.ConstructMapInfoFromPath(sample_points_of_pred_path_)) {
        if (reference_line_set_.Construct(veh_pos, veh_heading, map_space_)) {
          driving_map_type_ = DRIVING_MAP_TYPE_PRED_PATH;
        } else {
          driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
          LOG_ERR << "Failed to construct reference lines.";
        }
      } else {
        driving_map_type_ = DRIVING_MAP_TYPE_INVALID;
        LOG_ERR << "Failed to construct map from predicted path.";
      }
    } else {
      if (valid_raw_map_) {
        if (DRIVING_MAP_TYPE_CAMERA_LANE == driving_map_type_) {
          // 将原始地图的部分属性设置到驾驶地图中
          SetMapAttributeFromRawMap();
        }
      }
    }
  }
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (start)*/
   if ((data_source.is_in_tunnel) &&
       (DRIVING_MAP_TYPE_CAMERA_LANE == driving_map_type_)
       && (Nullptr_t != data_source.map)) {
     if (map_space_pcc_map_.ConstructMapInfo(*data_source.map)) {
       if (reference_line_set_pcc_map_.Construct(veh_pos, veh_heading,
                                                 map_space_pcc_map_)) {
         // 将坡度信息赋值到主参考线
         SetMapSlopAttributeFromPccMap();
       }
     }
   }
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (end)*/



  if (DRIVING_MAP_TYPE_INVALID != driving_map_type_) {
    // Calculate road boundary
    CalcRoadBoundary(&road_boundary_);
    /* k005 pengc 2023-01-06 (begin) */
    // 添加功能: 道路边界碰撞测试
  #if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
    ConstructRoadBoundaryMap(road_boundary_);
  #endif
    /* k005 pengc 2023-01-06 (end) */
  }

  if (DRIVING_MAP_TYPE_INVALID == driving_map_type_) {
    map_space_.Clear();
    reference_line_set_.Clear();

    for (Int32_t i = 0; i < road_boundary_.Size(); ++i) {
      road_boundary_[i].Clear();
    }
    road_boundary_.Clear();
    /* k005 pengc 2023-01-06 (begin) */
    // 添加功能: 道路边界碰撞测试
  #if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
    road_boundary_map_.Clear();
  #endif
    /* k005 pengc 2023-01-06 (end) */

    obstacle_list_.Clear();
    obstacle_map_.Clear();

    for (Int32_t i = 0; i < ref_line_association_list_.Size(); ++i) {
      ref_line_association_list_[i].Clear();
    }
    ref_line_association_list_.Clear();
  }

  // 更新场景任务
  if ((DRIVING_MAP_TYPE_INVALID != driving_map_type_) &&
      (gnss_.msg_head.valid)) {
    scene_story_set_.set_mat_convert_utm_to_rel_coor(
          mat_convert_global_to_rel_coor_);
    scene_story_set_.UpdateSceneStorys(
          data_source.timestamp, data_source.scene_story_list,
          GetMajorSmoothReferenceLine(), GetProjPointOnMajorRefLine());

    if (Nullptr_t != data_source.planning_story_list) {
      planning_story_set_.set_mat_convert_utm_to_rel_coor(
            mat_convert_global_to_rel_coor_);
      planning_story_set_.UpdateStorys(
            *data_source.planning_story_list, current_rel_position_);
    }
  }

  return true;
}

bool DrivingMapImpl::UpdateObstacleList(const ad_msg::ObstacleList* obj_list) {

#if ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST
  /// Performance of updating obstacle list for driving map (Start)
  phoenix::common::Stopwatch performance_timer_update_obstacles_map;
#endif

  if (Nullptr_t == obj_list) {
    obstacle_list_.Clear();
    obstacle_map_.Clear();

    for (Int32_t i = 0; i < ref_line_association_list_.Size(); ++i) {
      ref_line_association_list_[i].Clear();
    }
    ref_line_association_list_.Clear();

    return true;
  }

  UpdateObstacles(*obj_list);

  /// TODO: Need to convert coordinate
  // Construct association information of reference line
  if (DRIVING_MAP_TYPE_INVALID != driving_map_type_) {
    if (!CreateRefLineAssociation()) {
      for (Int32_t i = 0; i < ref_line_association_list_.Size(); ++i) {
        ref_line_association_list_[i].Clear();
      }
      ref_line_association_list_.Clear();
      LOG_ERR << "Failed to create reference line association.";
    }
  }

#if ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST
  /// Performance of updating obstacle list for driving map (End)
  std::cout << "Update obstacle list for driving map spend "
            << performance_timer_update_obstacles_map.Elapsed()
            << "ms." << std::endl;
#endif

  return true;
}

/* k005 pengc 2023-01-06 (begin) */
// 添加功能: 道路边界碰撞测试
Int32_t DrivingMapImpl::TestCollisionWithRoadBoundary(
    const CollisionTestObj& obj, CollisionTestResult* const result) const {
#if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
  CollisionTestParam param;
  param.testing_ignored_object = false;
  param.return_single_risky_object = true;
  param.return_uncertain_object = false;
  param.max_num_of_static_obj_to_return =
      CollisionTestResult::MAX_RISKY_OBJ_NUM;
  param.max_num_of_dynamic_obj_to_return = 0;

  return road_boundary_map_.TestCollision(param, obj, result);
#else
  result->Clear();
  return (0);
#endif
}
/* k005 pengc 2023-01-06 (end) */

void DrivingMapImpl::GetAllRiskyObstacles(
    common::StaticVector<CollisionTestResult::ObjInfo,
    MAX_OBJECT_NUM_IN_OBJ_SPACE>* risky_obj_list,
    common::StaticVector<CollisionTestResult::ObjInfo,
    MAX_OBJECT_NUM_IN_OBJ_SPACE>* uncertain_list) const {
  if ((Nullptr_t == risky_obj_list) && (Nullptr_t == uncertain_list)) {
    return;
  }

  if (Nullptr_t != risky_obj_list) {
    risky_obj_list->Clear();
  }
  if (Nullptr_t != uncertain_list) {
    uncertain_list->Clear();
  }

  if (ref_line_association_list_.Size() < MAX_LANE_NUM) {
    LOG_ERR << "There is not reference line in driving map.";
    return;
  }

  for (Int32_t i = 0; i < MAX_LANE_NUM; ++i) {
    const RefLineAssociation& ass = ref_line_association_list_[i];
    if (ass.lane_index < 0 || !ass.risky_obj_list_valid) {
      continue;
    }

    if (Nullptr_t != risky_obj_list) {
      Int32_t obj_num = ass.risky_obj_list.Size();
      for (Int32_t j = 0; j < obj_num; ++j) {
        AddRiskyObjToList(ass.risky_obj_list[j], risky_obj_list);
      }
    }

    if (Nullptr_t != uncertain_list) {
      Int32_t obj_num = ass.uncertain_list.Size();
      for (Int32_t j = 0; j < obj_num; ++j) {
        AddRiskyObjToList(ass.uncertain_list[j], uncertain_list);
      }
    }
  }
}

bool DrivingMapImpl::GetRiskyObstacles(
    Int32_t ref_line_index, bool return_uncertain,
    CollisionTestResult* const result) const {
  result->Clear();
  if (!reference_line_set_.IsValidReferenceLineIndex(ref_line_index)) {
    LOG_ERR << "Invalid reference line index[" << ref_line_index << "].";
    return false;
  }

  if (ref_line_association_list_.Size() < MAX_LANE_NUM) {
    LOG_ERR << "There is not reference line in driving map.";
    return false;
  }

  const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
      lane_segs = reference_line_set_.GetLaneSegments(ref_line_index);
  Int32_t lane_segs_size = lane_segs.Size();
  for (Int32_t i = 0; i < lane_segs_size; ++i) {
    const LaneSegment& seg = lane_segs[i];
    const RefLineAssociation& ass = ref_line_association_list_[seg.lane_index];
    if (ass.lane_index < 0) {
      LOG_ERR << "Detected invalid lane index "
                 "in reference line association list.";
      continue;
    }

    if (!ass.risky_obj_list_valid) {
      // Have not tested the collision on this lane segment.
      continue;
    }

    Int32_t obj_num = ass.risky_obj_list.Size();
    for (Int32_t j = 0; j < obj_num; ++j) {
      AddRiskyObjToList(ass.risky_obj_list[j], &(result->risky_obj_list));
    }

    if (return_uncertain) {
      Int32_t uncertain_obj_num = ass.uncertain_list.Size();
      for (Int32_t j = 0; j < uncertain_obj_num; ++j) {
        AddRiskyObjToList(ass.uncertain_list[j], &(result->uncertain_list));
      }
    }
  }

  return true;
}

Int32_t DrivingMapImpl::GetEventReporting(
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

void DrivingMapImpl::GetDrivingMapInfo(
    DrivingMapInfo* const driving_map_info) const {
  driving_map_info->Clear();

  driving_map_info->msg_head = GetHDMapMsgHead();

  driving_map_info->driving_map_type = driving_map_type_;

  Int32_t nearest_lane_index = -1;
  GetNearestLaneToCurrPosition(
        &nearest_lane_index, &driving_map_info->nearest_point_to_veh_on_lane,
        Nullptr_t, Nullptr_t);

  const common::StaticVector<Lane, MAX_LANE_NUM>& lane_table = GetLaneTable();

  for (Int32_t i = 0; i < lane_table.Size(); ++i) {
    LaneInfo * const lane_info_out =
        driving_map_info->map.lane_table.Allocate();
    if (Nullptr_t == lane_info_out) {
      break;
    }
    lane_info_out->lane_id = lane_table[i].topology_id().id;
    lane_info_out->lane_index = lane_table[i].topology_id().index;
    lane_info_out->quality = lane_table[i].quality();
    const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        points = lane_table[i].central_curve().points();
    for (Int32_t j = 0; j < points.Size(); ++j) {
      if (!lane_info_out->central_curve.PushBack(points[j])) {
        break;
      }
    }

    /* k003 longjiaoy 2022-12-06 (start) */
    // 获取车道边线
#if 0
    const common::StaticVector<Lane::BoundaryAssociation,
        Lane::MAX_LANE_BOUNDARY_SAMPLE_NUM>&
        left_boundary = lane_table[i].left_boundary().boundary_samples;
    for (Int32_t j = 0; j < left_boundary.Size(); ++j) {
      LaneInfo::Boundary::BoundaryAssociation* data
          = lane_info_out->left_boundary.curve.Allocate();
      data->s = left_boundary[j].s;
      data->type = left_boundary[j].type;
      data->width = left_boundary[j].width;
    }
    const common::StaticVector<Lane::BoundaryAssociation,
        Lane::MAX_LANE_BOUNDARY_SAMPLE_NUM>&
        right_boundary = lane_table[i].right_boundary().boundary_samples;
    for (Int32_t j = 0; j < right_boundary.Size(); ++j) {
      LaneInfo::Boundary::BoundaryAssociation* data
          = lane_info_out->right_boundary.curve.Allocate();
      data->s = right_boundary[j].s;
      data->type = right_boundary[j].type;
      data->width = right_boundary[j].width;
    }
#endif
    /* k003 longjiaoy 2022-12-06 (end) */
  }
  driving_map_info->map.map_traffic_light_table = GetMapTrafficLightTable();

  driving_map_info->current_reference_line_index = GetMajorRefLineIndex();
  for (Int32_t i = 0; i < GetReferenceLinesNum(); ++i) {
    const common::Path& curve = GetReferenceLineCurve(i);
    const common::Path& smooth_curve = GetReferenceLineSmoothCurve(i);

    DrivingMapInfo::ReferenceLineInfo* ref_line_info =
        driving_map_info->reference_lines.Allocate();
    if (Nullptr_t == ref_line_info) {
      break;
    }

    ref_line_info->curve.Clear();
    curve.GetSamplePoints(&ref_line_info->curve);

    ref_line_info->smooth_curve.Clear();
    smooth_curve.GetSamplePoints(&ref_line_info->smooth_curve);
  }

  const common::PathPoint& proj_point_on_curr_ref_line =
      GetProjPointOnMajorRefLine();

  driving_map_info->road_boundary = GetRoadBoundary();
  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
#if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
  const common::StaticVector<Object, MAX_OBJECT_NUM_IN_OBJ_SPACE>&
      road_boundary_obj_list = road_boundary_map_.object_list();
  for (Int32_t i = 0; i < road_boundary_obj_list.Size(); ++i) {
    driving_map_info->road_boundary_obj_list.PushBack(
          road_boundary_obj_list[i].obb);
  }
#endif
  /* k005 pengc 2023-01-06 (end) */
  ExtractLaneBoundary(driving_map_info);

  const common::StaticVector<Object, MAX_OBJECT_NUM_IN_OBJ_SPACE>&
      obstacle_list = GetObstacleListInObjSpace();
  for (Int32_t i = 0; i < obstacle_list.Size(); ++i) {
    DrivingMapInfo::ObstacleInfo* obstacle =
        driving_map_info->obstacle_list.Allocate();
    if (Nullptr_t == obstacle) {
      break;
    }
    const Object& obj = obstacle_list[i];
    // 障碍物ID
    obstacle->id = obj.id;
    // 障碍物位置及包围盒
    obstacle->obb = obj.obb;
    // Heading
    obstacle->heading = obj.heading;
    // 障碍物类型
    obstacle->type = obj.type;
    // 是否是动态障碍物
    obstacle->dynamic = obj.dynamic;
    // 是否应当忽略此障碍物（例如道路外的静态障碍物（人、车等例外），
    // 车辆正后方的动态障碍物，车辆后方的静态障碍物等）
    obstacle->ignore = obj.ignore;
    // 标记不确定的障碍物
    obstacle->uncertain = obj.uncertain;
    // 障碍物速度
    obstacle->v = obj.v;
    // 与参考线之间的关联信息
    obstacle->s_ref = obj.ref_line.proj_on_ref.s -
        proj_point_on_curr_ref_line.s;
    obstacle->l_ref = obj.ref_line.proj_on_ref.l;
    // 保存对动态障碍物预测的轨迹
    obstacle->pred_trajectory = obj.pred_trajectory;
  }

  GetAllRiskyObstacles(
        &(driving_map_info->risky_obj_list),
        &(driving_map_info->uncertain_list));

  if (following_path_info_.valid &&
      following_path_info_.following_target.valid) {
    driving_map_info->following_target.valid = true;
    driving_map_info->following_target.obj_x =
        following_path_info_.following_target.obj_rel_x;
    driving_map_info->following_target.obj_y =
        following_path_info_.following_target.obj_rel_y;
  }

  driving_map_info->scene_storys = GetSceneStoryList();
  driving_map_info->planning_storys = GetPlanningStoryList();
}


void DrivingMapImpl::UpdateVehicleParameter() {
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

  if (ad_msg::VEH_TRAILER_STATUS_CONNECTED == chassis_.trailer_status) {
    param_.vehicle_width =
        common::Max(veh_model.GetVehicleWidth(), veh_model.GetTrailerWidth());
  }

  // 外接圆半径
  common::OBBox2d obb;
  CreateVehOBB(common::Vec2d(0.0F, 0.0F), 0.0F, &obb);
  param_.vehicle_circumradius = obb.CalcCircumradius();

  // LOG_INFO(5) << "Vehicle Width = " << param_.vehicle_width
  //             << ", Trailer Status = " << chassis_.trailer_status;
}

bool DrivingMapImpl::UpdatePositionInfo(
    const DrivingMapDataSource& data_source, bool gnss_is_valid) {
  // Get current position
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
      data_source.timestamp, *data_source.rel_pos_list, &current_rel_position_)) {
    LOG_ERR << "Failed to get current posistion from relative position list.";
    return false;
  }

  bool ret = true;
  Float64_t global_x = data_source.gnss->x_utm;
  Float64_t global_y = data_source.gnss->y_utm;
  Float64_t global_z = data_source.gnss->z_utm;
  Float32_t global_heading = data_source.gnss->heading_utm;
  if (GNSS_COORDINATE_TYPE_ODOM == data_source.gnss_coordinate_type) {
    global_x = data_source.gnss->x_odom;
    global_y = data_source.gnss->y_odom;
    global_z = data_source.gnss->z_odom;
    global_heading = data_source.gnss->heading_odom;
  }
  if (gnss_is_valid) {
    if (com_isnan(global_x) || com_isinf(global_x) ||
        com_isnan(global_y) || com_isinf(global_y) ||
        com_isnan(global_heading) || com_isinf(global_heading)) {
      gnss_is_valid = false;
      ret = false;
      LOG_ERR << "Invalid global position, global_x=" << global_x
              << ", global_y=" << global_y
              << ", global_heading=" << global_heading;
    }
    if ((common::com_abs(global_x) > 1.0e10) ||
        (common::com_abs(global_y) > 1.0e10) ||
        (common::com_abs(global_z) > 1.0e10)) {
      gnss_is_valid = false;
      ret = false;
      LOG_ERR << "Invalid global position, global_x=" << global_x
              << ", global_y=" << global_y
              << ", global_z=" << global_z;
    }
  }

  // Convert utm to Vehicle Coordinate System (VCS)
  mat_convert_global_to_rel_coor_.SetIdentity();
  if (gnss_is_valid) {
    // 时间及空间同步(对齐到时间流中)
    ad_msg::RelativePos gnss_pos_in_rel_list;
    if (pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          data_source.gnss->msg_head.timestamp, *data_source.rel_pos_list,
          &gnss_pos_in_rel_list)) {
      common::Matrix<Float64_t, 2, 1> rotate_center;
      rotate_center.SetZeros();
      common::Translate_2D(-global_x, -global_y,
                           &mat_convert_global_to_rel_coor_);
      common::Rotate_2D<Float64_t>(rotate_center, -global_heading,
                                   &mat_convert_global_to_rel_coor_);
      common::Rotate_2D<Float64_t>(rotate_center, gnss_pos_in_rel_list.heading,
                                   &mat_convert_global_to_rel_coor_);
      common::Translate_2D(static_cast<Float64_t>(gnss_pos_in_rel_list.x),
                           static_cast<Float64_t>(gnss_pos_in_rel_list.y),
                           &mat_convert_global_to_rel_coor_);
    } else {
      LOG_ERR << "Failed to get pos of gnss in reletive position list.";
    }
  }

  // Update some map space information
#if (ENABLE_MIXED_MAP_SOURCE)
  map_space_raw_map_.SetRelPosList(*data_source.rel_pos_list);
  if (gnss_is_valid) {
    map_space_raw_map_.SetGlobalPosition(
          global_x, global_y, global_z, global_heading);
  }
  map_space_raw_map_.SetMatConvertGlobalToRelCoor(mat_convert_global_to_rel_coor_);
  map_space_raw_map_.SetCurrentRelPosition(current_rel_position_);
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    map_space_raw_map_.set_driving_direction(HDMap::DRIVING_DIRECTION_BACKWARD);
  } else {
    map_space_raw_map_.set_driving_direction(HDMap::DRIVING_DIRECTION_FORWARD);
  }

  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (start)*/
  map_space_pcc_map_.SetRelPosList(*data_source.rel_pos_list);
  if (gnss_is_valid) {
    map_space_pcc_map_.SetGlobalPosition(
          global_x, global_y, global_z, global_heading);
  }
  map_space_pcc_map_.SetMatConvertGlobalToRelCoor(mat_convert_global_to_rel_coor_);
  map_space_pcc_map_.SetCurrentRelPosition(current_rel_position_);
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    map_space_pcc_map_.set_driving_direction(HDMap::DRIVING_DIRECTION_BACKWARD);
  } else {
    map_space_pcc_map_.set_driving_direction(HDMap::DRIVING_DIRECTION_FORWARD);
  }
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (end)*/

#endif
  map_space_.hd_map_msg_head().valid = true;
  map_space_.hd_map_msg_head().timestamp = data_source.timestamp;
  map_space_.hd_map_msg_head().UpdateSequenceNum();
  map_space_.SetRelPosList(*data_source.rel_pos_list);
  if (gnss_is_valid) {
    map_space_.SetGlobalPosition(
          global_x, global_y, global_z, global_heading);
  }
  map_space_.SetMatConvertGlobalToRelCoor(mat_convert_global_to_rel_coor_);
  map_space_.SetCurrentRelPosition(current_rel_position_);
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    map_space_.set_driving_direction(HDMap::DRIVING_DIRECTION_BACKWARD);
  } else {
    map_space_.set_driving_direction(HDMap::DRIVING_DIRECTION_FORWARD);
  }

  return (ret);
}

Int32_t DrivingMapImpl::SelectDrivingMapType(
    const DrivingMapDataSource& data_source, Int32_t location_status) {
  Int32_t map_type = DRIVING_MAP_TYPE_INVALID;

  // printf("### SelectDrivingMapType (Begin)->\n");

  // 校验相机识别的车道线, 获取车道线质量
  Int32_t camera_lane_quality = CheckCameraLane(data_source);
  // 获取GNSS定位质量
  Int32_t location_quality = 0;
  if (Nullptr_t != data_source.gnss) {
    if (data_source.gnss->msg_head.valid) {
      switch (location_status) {
      case (ad_msg::Gnss::STATUS_BAD):
        location_quality = 1;
        break;
      case (ad_msg::Gnss::STATUS_CONVERGING):
        location_quality = 3;
        break;
      case (ad_msg::Gnss::STATUS_GOOD):
        location_quality = 3;
        break;
      default:
        location_quality = 0;
        break;
      }
    }
  }
  if (1 != data_source.is_vaild_hd_map) {
    location_quality = 0;
    //AddEventReporting(EVENT_TYPE_MAP_IS_NOT_VAILD);
  }
  if (DrivingMapConfig::INPUTTED_MAP_TYPE_CAMERA ==
      config_.inputted_map_type) {
    // Using camera lane
    if (Nullptr_t != data_source.camera_lane_list) {
      map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
    }
  } else if (DrivingMapConfig::INPUTTED_MAP_TYPE_HDMAP ==
             config_.inputted_map_type) {
    // Using HD-Map
    if ((Nullptr_t != data_source.map) &&
        (location_quality > 0)) {
      map_type = DRIVING_MAP_TYPE_HD_MAP;
    }
  } else if (DrivingMapConfig::INPUTTED_MAP_TYPE_MIXED ==
             config_.inputted_map_type) {
#if (ENABLE_MIXED_MAP_SOURCE)
    map_type = DRIVING_MAP_TYPE_INVALID;
    if ((Nullptr_t != data_source.map) &&
        (Nullptr_t != data_source.camera_lane_list) &&
        (camera_lane_quality > 0) &&
        (location_quality > 0)) {
      map_type = ConstructMixedMap(
            data_source, camera_lane_quality, location_quality);
    }
    if (DRIVING_MAP_TYPE_MIXED_HD_MAP == map_type) {
      if ((Nullptr_t != data_source.map) &&
          (Nullptr_t != data_source.camera_lane_list) &&
          (camera_lane_quality > 2) &&
          (location_quality > 2)) {
        map_type = DRIVING_MAP_TYPE_MIXED_HD_MAP;
        LOG_INFO(5) << "### Using mixed HD-Map";
      } else {
        map_type = DRIVING_MAP_TYPE_INVALID;
      }
    }
    if (DRIVING_MAP_TYPE_INVALID == map_type) {
      if ((Nullptr_t != data_source.camera_lane_list) &&
          (camera_lane_quality > 2)) {
        map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
        LOG_INFO(5) << "### Low quality of HD-Map, switch to camera lane.";
      } else if ((Nullptr_t != data_source.map) &&
                 (location_quality > 2)) {
        map_type = DRIVING_MAP_TYPE_HD_MAP;
        LOG_INFO(5) << "### Low quality of camera lane, switch to HD-Map.";
      } else if ((Nullptr_t != data_source.camera_lane_list) &&
                 (camera_lane_quality >= 1)) {
        map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
        LOG_INFO(5) << "### Low quality of HD-Map, switch to camera lane.";
      } else if ((Nullptr_t != data_source.map) &&
                 (location_quality > 1)) {
        map_type = DRIVING_MAP_TYPE_HD_MAP;
        LOG_INFO(5) << "### Low quality of camera lane, switch to HD-Map.";
      } else {
        map_type = DRIVING_MAP_TYPE_PRED_PATH;
        LOG_INFO(5) << "### Low quality of camera lane and HD-Map, hold steering.";
      }
    }
#else
    LOG_ERR << "Not suport mixed map, switch to camera type.";
    if (Nullptr_t != data_source.camera_lane_list) {
      map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
    }
#endif
  } else {
    LOG_ERR << "Invalid inputted map tyep.";

    // Using camera lane
    if (Nullptr_t != data_source.camera_lane_list) {
      map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
    }
  }

  // printf("    camera_lane_quality=%d, "
  //        "location_quality=%d, "
  //        "map_type=%d"
  //        "\n",
  //        camera_lane_quality, location_quality, map_type);

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  if ((DrivingMapConfig::INPUTTED_MAP_TYPE_CAMERA == config_.inputted_map_type) ||
      (DrivingMapConfig::INPUTTED_MAP_TYPE_MIXED == config_.inputted_map_type)) {
#else
  if (DRIVING_MAP_TYPE_CAMERA_LANE == map_type) {
#endif
    bool camera_lane_lost = false;
    if (camera_lane_quality < 1) {
      camera_lane_lost = true;
    }
    if (1 <= camera_lane_quality && camera_lane_quality < 2) {
      if (action_smoothing_camera_lane_lost_.Smooth(true)) {
        camera_lane_lost = true;
      }
    } else {
      action_smoothing_camera_lane_lost_.Smooth(false);
    }

    if (camera_lane_lost) {
      if (DRIVING_MAP_TYPE_CAMERA_LANE == map_type) {
        map_type = DRIVING_MAP_TYPE_PRED_PATH;
      }

      if (ad_msg::VEH_EPS_STATUS_ROBOTIC == chassis_.eps_status) {
        AddEventReporting(EVENT_TYPE_CAMERA_LANE_LOST);
      }
    }
  }

  if (DRIVING_MAP_TYPE_PRED_PATH == map_type) {
    if (CalcFollowingPath(data_source)) {
      map_type = DRIVING_MAP_TYPE_FOLLOWING_PATH;
    }
  }
  if (DRIVING_MAP_TYPE_FOLLOWING_PATH != map_type) {
    following_path_info_.Clear();
  }

  // printf("    Finaly, map_type=%d\n", map_type);

  return (map_type);
}

Int32_t DrivingMapImpl::ConstructMixedMap(
    const DrivingMapDataSource& data_source,
    Int32_t camera_lane_quality, Int32_t location_quality) {
  Int32_t map_type = DRIVING_MAP_TYPE_INVALID;

#if (ENABLE_MIXED_MAP_SOURCE)

  common::Vec2d veh_pos(current_rel_position_.x, current_rel_position_.y);
  map_var_t veh_heading = current_rel_position_.heading;
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    veh_heading = common::NormalizeAngle(veh_heading + COM_PI);
  }

  Int32_t cur_cam_lane_idx = cam_info_.lane_list.FindCenterLineById(0);
  if (cur_cam_lane_idx < 0) {
    LOG_ERR << "Can't find current lane from camera lane list when construct Mixed HD-Map";
    return (map_type);
  }
  const ad_msg::LaneCenterLineCamera& cur_cam_center_line =
      cam_info_.lane_list.center_lines[cur_cam_lane_idx];
  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
      points_2d = temp_data_.sample_2d_points;
  points_2d.Clear();
#if 0
  printf("Camera center line (In Driving Map)=\n");
  for (Int32_t i = 0; i < cur_cam_center_line.curve_point_num; ++i) {
    printf("p[%d]: (%0.1f, %0.1f)\n", i,
           cur_cam_center_line.curve[i].x,
           cur_cam_center_line.curve[i].y);
  }
#endif
  for (Int32_t i = 0; i < cur_cam_center_line.curve_point_num; ++i) {
    points_2d.PushBack(common::Vec2d(cur_cam_center_line.curve[i].x,
                                     cur_cam_center_line.curve[i].y));
  }
  common::Path& cur_cam_center_path = current_cam_center_path_;
  if (!cur_cam_center_path.Construct(points_2d)) {
    LOG_ERR << "Failed to construct current camera center path when construct Mixed HD-Map.";
    return (map_type);
  }

  common::PathPoint proj_on_cam_lane;
  if (!cur_cam_center_path.FindProjection(veh_pos, &proj_on_cam_lane)) {
    LOG_ERR << "Failed to find projection on current camera center path when construct Mixed HD-Map.";
    return (map_type);
  }
  map_var_t forward_len_camera =
      cur_cam_center_path.total_length() - proj_on_cam_lane.s;
  map_var_t cam_heading_diff = common::AngleDiff(
        veh_heading, proj_on_cam_lane.heading);

  map_type = DRIVING_MAP_TYPE_CAMERA_LANE;

  // Construct internal map from HDMap
  if (!map_space_raw_map_.ConstructMapInfo(*data_source.map)) {
    LOG_ERR << "Failed to construct Mixed HD Map, using camera lane.";
    return (map_type);
  }
  if (map_space_raw_map_.GetLaneTable().Size() < 1) {
    // Empty Mixed-HD-Map
    LOG_INFO(5) << "Invalid HD-Map, using camera lane.";
    return (map_type);
  }
  if (Nullptr_t != data_source.routing) {
    map_space_raw_map_.ConstructRoutingInfo(*data_source.routing);
  }

  // Construct reference lines
  if (!reference_line_set_raw_map_.Construct(
        veh_pos, veh_heading, map_space_raw_map_)) {
    LOG_ERR << "Failed to construct reference lines when construct Mixed HD-Map.";
    return (map_type);
  }

  // 原始地图更新成功
  valid_raw_map_ = true;

  // Find nearest lane to camera lane
  Int32_t nearest_lane_index = -1;
  common::PathPoint nearest_point_on_lane;
  if (!map_space_raw_map_.FindNearestLaneSynthetically(
        proj_on_cam_lane.point, proj_on_cam_lane.heading,
        &nearest_lane_index, &nearest_point_on_lane)) {
    LOG_ERR << "Failed to find nearest lane to camera from Mixed HD-Map, using camera lane.";
    return (map_type);
  }
  if (!map_space_raw_map_.IsValidLaneIndex(nearest_lane_index)) {
    LOG_ERR << "Failed to find nearest lane to camera from hd-map, the nearest lane index "
            << nearest_lane_index << " is invalid, using camera lane.";
    return (map_type);
  }
  if (common::com_abs(nearest_point_on_lane.l) > 1.5F) {
    // Too far from nearest lane in HD-Map to camera
    LOG_INFO(5) << "Too far from camera lane to HD-Map, using camera lane.";
    return (map_type);
  }

  const Lane* nearest_lane = &(map_space_raw_map_.GetLane(nearest_lane_index));

  map_var_t lat_diff = -nearest_point_on_lane.l;
  map_var_t abs_lat_diff = common::com_abs(lat_diff);
  common::Vec2d pos_diff = nearest_point_on_lane.point - proj_on_cam_lane.point;
  map_var_t heading_diff = common::AngleDiff(
        proj_on_cam_lane.heading, nearest_point_on_lane.heading);
  map_var_t abs_heading_diff = common::com_abs(heading_diff);

  LOG_INFO(5) << "After searching nearest lane to camera, the nearest lane is"
              << " (idx=" << nearest_lane_index
              << ", id=" << nearest_lane->topology_id().id.id
              << ").";

  /* k004 pengc 2022-12-26 (begin) */
  // 使用相机车道线矫正地图定位
  LOG_INFO(5) << "The different between Camera Lane and HD-Map is:"
              << " lat_diff=" << lat_diff
              << ", pos_diff=(" << pos_diff.x() << ", " << pos_diff.y() << ")"
              << ", heading_diff=" << common::com_rad2deg(heading_diff);

  if ((abs_lat_diff < 1.0F) &&
      (abs_heading_diff < common::com_deg2rad(15.0F)) &&
      (camera_lane_quality > 1)) {
    // 使用相机的定位结果去修正GNSS的定位结果
    if (!CorrectGnss(
          pos_diff, heading_diff, proj_on_cam_lane, cur_cam_center_path,
          map_space_raw_map_)) {
      LOG_WARN << "Failed to correct gnss by camera, using camera lane.";
      return (map_type);
    }
  } else {
    LOG_WARN << "The different between camera and hd-map is too "
                "large, using camera lane."
             << " lat_diff=" << abs_lat_diff
             << ", heading_diff=" << common::com_rad2deg(abs_heading_diff);
    return (map_type);
  }
  /* k004 pengc 2022-12-26 (end) */

  Int32_t major_ref_line_idx =
       reference_line_set_raw_map_.GetMajorRefLineIndex();
  const common::PathPoint& proj_on_hd_map =
      reference_line_set_raw_map_.GetProjPointOnMajorRefLine();
  const common::Path& major_ref_line =
      reference_line_set_raw_map_.GetSmoothCurve(major_ref_line_idx);
  map_var_t forward_len_hd_map =
      major_ref_line.total_length() - proj_on_hd_map.s;

#if 0
  // 存在最近的相机车道线在左车道，但最近的地图车道线在右车道的情况。
  if ((abs_lat_diff > 2.0F) /*||
      (abs_heading_diff > common::com_deg2rad(10.0F))*/) {
    // 地图定位与相机定位偏差过大
    // 优先使用相机的结果
    std::cout << "    abs_lat_diff(" << abs_lat_diff
              << ") > 2.0, using camera."
              << " proj_on_cam_lane.l=" << proj_on_cam_lane.l
              << ", proj_on_hd_map.l=" << proj_on_hd_map.l
              << std::endl;
    map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
    return (map_type);
  }
#endif

  if (common::com_abs(proj_on_hd_map.l) > 5.0F) {
    map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
    LOG_INFO(5) << "Too far from HD-Map, using camera lane.";
    return (map_type);
  }

#if (VEHICLE_PLATFORM != VEHICLE_PLATFORM_DF_D17_B1) && \
    (VEHICLE_PLATFORM != VEHICLE_PLATFORM_DF_D17_B2)
  // 检测地图中的弯道，若曲率过大，切换到地图模式（相机暂时不能正确识别较大曲率的弯道）
  //map_var_t limited_curvature = 0.005F; // 半径 200m
  map_var_t limited_curvature = 0.01F; // 半径 100m
  map_var_t abs_curvature = 0.0F;
  bool near_curve = false;
  map_var_t start_s = proj_on_hd_map.s;
  map_var_t end_s = start_s + common::Max(20.0F, chassis_.v * 4.0F);
  const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& curve_segments =
      reference_line_set_raw_map_.GetSmoothCurveCurvatureInfo(major_ref_line_idx);
  for (Int32_t i = 0; i < curve_segments.Size(); ++i) {
    const common::Path::CurveSegment& curve_seg = curve_segments[i];
    if ((curve_seg.start_s + curve_seg.length) < start_s) {
      continue;
    }
    if (curve_seg.start_s > end_s) {
      continue;
    }

    //if (curve_seg.length)
    map_var_t s = curve_seg.start_s - start_s;
    if (s < 0) {
      s = 0;
    }
    abs_curvature = common::com_abs(curve_seg.max_curvature);
    // printf("    abs_curvature=%f, s=%0.1f[start_s=%0.1f, len=%0.1f, cur=%0.1f]\n",
    //        abs_curvature, s, curve_seg.start_s, curve_seg.length, start_s);
    if (abs_curvature > limited_curvature) {
      near_curve = true;
      break;
    }
  }
  if (near_curve) {
    // 接近大曲率弯道, 切换到地图模式
    map_type = DRIVING_MAP_TYPE_HD_MAP;
    LOG_INFO(5) << "Near curve, switch to HD-Map, abs_curvature="
                << abs_curvature;
    return (map_type);
  }
#endif

  /// TODO: 检测地图中的路口, 若接近路口, 切换到地图模式
  /* D001 longjiaoy 2023-08-16 (begin) */
  // 分流口切换成地图模式
  for (Int32_t i = 0; i < data_source.scene_story_list->story_num; ++i) {

    const ad_msg::SceneStory& story = data_source.scene_story_list->storys[i];
    if (ad_msg::SCENE_STORY_TYPE_SWITCH_TO_MAP == story.type) {
      map_type = DRIVING_MAP_TYPE_HD_MAP;
      LOG_INFO(5) << "Switch to HD-Map.";
      return (map_type);
    }
  }
  /* D001 longjiaoy 2023-08-16 (end)*/

  // 地图前向长度过短，但相机车道线较长，切换到相机车道线
  if (((forward_len_hd_map < common::Max(15.0F, 3.0F*chassis_.v)) &&
       (forward_len_hd_map < 0.8F*forward_len_camera)) ||
      (forward_len_hd_map < 0.6F*forward_len_camera)) {
    map_type = DRIVING_MAP_TYPE_CAMERA_LANE;
    LOG_INFO(5) << "The forward length of HD-Map is too short, using Camera lane."
                << " forward_len_hd_map=" << forward_len_hd_map
                << ", forward_len_camera=" << forward_len_camera;
    return (map_type);
  }

  // 车身与相机车道线之间的夹角过大，可能导致车道线识别异常，进而导致匹配异常, 切换到地图模式
  if (common::com_abs(cam_heading_diff) > common::com_deg2rad(5.0F)) {
    if (location_quality > 2) {
      map_type = DRIVING_MAP_TYPE_HD_MAP;
      LOG_INFO(5) << "The defferent heading between Camera lane and "
                     "Vehicle is larger than 5deg, switch to HD-Map.";
      return (map_type);
    }
  }

  map_type = DRIVING_MAP_TYPE_MIXED_HD_MAP;

#endif // #if (ENABLE_MIXED_MAP_SOURCE)

  return (map_type);
}

/* k004 pengc 2022-12-26 (begin) */
// 使用相机车道线矫正地图定位
bool DrivingMapImpl::CorrectGnss(
    const common::Vec2d& pos_diff, map_var_t heading_diff,
    const common::PathPoint& proj_on_cam_lane,
    const common::Path& cam_center_path,
    HDMap& hd_map) {
  const static Int32_t kMaxSampleNum = 10;
  const static map_var_t kSampleStepLen = 5.0F;

  delta_pos_corrected_gnss_[0] = 0.0F;
  delta_pos_corrected_gnss_[1] = 0.0F;
  delta_pos_corrected_gnss_[2] = 0.0F;
  mat_convert_corrected_gnss_.SetIdentity();
  mat_convert_corrected_gnss_updated_ = false;

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
      sample_points =  temp_data_.sample_points;
  sample_points.Clear();
  Int32_t sample_num =
      (cam_center_path.total_length() - proj_on_cam_lane.s) / kSampleStepLen;
  if (sample_num < 2) {
    sample_num = 2;
  } if (sample_num > kMaxSampleNum) {
    sample_num = kMaxSampleNum;
  }
  if (!cam_center_path.UniformlySamplePathForward(
        proj_on_cam_lane.s, sample_num, kSampleStepLen, &sample_points)) {
    LOG_ERR << "Failed to sample camera center path.";
    return false;
  }

  map_var_t delta_pos[3] = { 0 };
  delta_pos[0] = pos_diff.x();
  delta_pos[1] = pos_diff.y();
  delta_pos[2] = heading_diff;

  LOG_INFO(5) << "Register hd-map [in] (" << -delta_pos[0]
              << "m, " << -delta_pos[1]
              << "m, " << common::com_rad2deg(-delta_pos[2])
              << "deg).";

  Int32_t register_ret = hd_map.RegisterPath(sample_points, delta_pos);
  if (HDMap::REGISTER_QUALITY_INVALID == register_ret) {
    LOG_ERR << "Failed to register camera lane with hd-map.";
    return false;
  } else if (HDMap::REGISTER_QUALITY_BAD == register_ret) {
    LOG_ERR << "The quality of registering camera lane with hd-map is bad.";
    return false;
  } else {
    delta_pos_corrected_gnss_[0] = -delta_pos[0];
    delta_pos_corrected_gnss_[1] = -delta_pos[1];
    delta_pos_corrected_gnss_[2] = -delta_pos[2];

    common::Matrix<map_var_t, 2, 1> rotate_center;
    rotate_center.SetZeros();
    common::Rotate_2D<map_var_t>(
          rotate_center, -delta_pos[2], &mat_convert_corrected_gnss_);
    common::Translate_2D(
          -delta_pos[0], -delta_pos[1], &mat_convert_corrected_gnss_);

    mat_convert_corrected_gnss_updated_ = true;

    LOG_INFO(5) << "Register hd-map [out] (" << -delta_pos[0]
                << "m, " << -delta_pos[1]
                << "m, " << common::com_rad2deg(-delta_pos[2])
                << "deg).";
  }

  return true;
}
/* k004 pengc 2022-12-26 (end) */

void DrivingMapImpl::ExtractLaneBoundary(
    DrivingMapInfo* const driving_map_info) const {
  map_var_t step_len = 2.0F;

  common::StaticVector<LaneInfo, MAX_LANE_NUM>& lane_table =
      driving_map_info->map.lane_table;
  Int32_t lane_table_size = lane_table.Size();
  if (lane_table_size < 1) {
    return;
  }

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>
      sample_points;
  sample_points.Clear();

  Int32_t curr_ref_line_index = GetMajorRefLineIndex();
  if (!IsValidReferenceLineIndex(curr_ref_line_index)) {
    LOG_ERR << "Invalid reference line.";
    return;
  }

  const common::Path& curr_ref_line =
      GetReferenceLineCurve(curr_ref_line_index);
  if (curr_ref_line.total_length() < 1.0f) {
    LOG_ERR << "The length of reference line is too short.";
    return;
  }
  Int32_t sample_size = phoenix::common::com_round(
        curr_ref_line.total_length() / step_len);
  if (sample_size < 1) {
    sample_size = 1;
  }
  step_len = curr_ref_line.total_length() / sample_size;

  if (false == curr_ref_line.UniformlySamplePathForward(
        0, sample_size, step_len, &sample_points)) {
    LOG_ERR << "Failed to sample reference line.";
    return;
  }
  Int32_t sample_points_size = sample_points.Size();
  if (sample_points_size < 2) {
    LOG_ERR << "The number of sample points of reference line is too small.";
    return;
  }

  Int32_t nearest_neighbor_index = 0;
  common::StaticVector<NeighborsLaneInfo,
      MAX_NEIGHBOR_LANE_NUM> sorted_neighbor_lanes;

  for (Int32_t i = 0; i < sample_points_size; ++i) {
    const common::PathPoint& sample_point_on_ref = sample_points[i];
    Int32_t lane_index = 0;
    FindReferenceLineLaneSegmentByProjOnRef(
          curr_ref_line_index, sample_point_on_ref.s, &lane_index);
    if (!FindNeighbors(
          sample_point_on_ref.point,
          lane_index,
          &sorted_neighbor_lanes,
          &nearest_neighbor_index)) {
      LOG_ERR << "Failed to find neighbor lanes.";
      break;
    }

    Int32_t sorted_neighbor_lanes_num = sorted_neighbor_lanes.Size();
    for (Int32_t j = 0; j < sorted_neighbor_lanes_num; ++j) {
      const NeighborsLaneInfo& neighbor = sorted_neighbor_lanes[j];
      if (!neighbor.valid_lat || !neighbor.valid_lon) {
        continue;
      }

      map_var_t left_width = 0;
      map_var_t right_width = 0;
      GetLaneWidth(neighbor.lane_index, neighbor.proj_point.s,
                   &left_width, &right_width);

      bool left_boundary_valid = false;
      bool right_boundary_valid = false;
      Int32_t left_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
      Int32_t right_boundary_type = LANE_BOUNDARY_TYPE_SOLID_WHITE;
      if (j == nearest_neighbor_index) {
        // current lane
        left_boundary_valid = true;
        right_boundary_valid = true;

        if (j > 0) {
          if (sorted_neighbor_lanes[j-1].valid_lat &&
              sorted_neighbor_lanes[j-1].valid_lon) {
            // has lane on right
            map_var_t dist = 0.5f*common::com_abs(neighbor.proj_point.l -
                  sorted_neighbor_lanes[j-1].proj_point.l);
            right_width = dist;
            if (dist < 1.0f) {
              right_boundary_valid = false;
            } else {
              right_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
            }
          }
        }
        if (j < (sorted_neighbor_lanes_num-1)) {
          if (sorted_neighbor_lanes[j+1].valid_lat &&
              sorted_neighbor_lanes[j+1].valid_lon) {
            // has lane on left
            map_var_t dist = 0.5f*common::com_abs(neighbor.proj_point.l -
                  sorted_neighbor_lanes[j+1].proj_point.l);
            left_width = dist;
            if (dist < 1.0f) {
              left_boundary_valid = false;
            } else {
              left_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
            }
          }
        }
      } else if (j < nearest_neighbor_index) {
        right_boundary_valid = true;
        // right lane
        if (j > 0) {
          if (sorted_neighbor_lanes[j-1].valid_lat &&
              sorted_neighbor_lanes[j-1].valid_lon) {
            // has lane on right
            map_var_t dist = 0.5f*common::com_abs(neighbor.proj_point.l -
                  sorted_neighbor_lanes[j-1].proj_point.l);
            right_width = dist;
            if (dist < 1.0f) {
              right_boundary_valid = false;
            } else {
              right_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
            }
          }
        }
      } else {
        left_boundary_valid = true;
        // left lane
        if (j < (sorted_neighbor_lanes_num-1)) {
          if (sorted_neighbor_lanes[j+1].valid_lat &&
              sorted_neighbor_lanes[j+1].valid_lon) {
            // has lane on left
            map_var_t dist = 0.5f*common::com_abs(neighbor.proj_point.l -
                  sorted_neighbor_lanes[j+1].proj_point.l);
            left_width = dist;
            if (dist < 1.0f) {
              left_boundary_valid = false;
            } else {
              left_boundary_type = LANE_BOUNDARY_TYPE_DOTTED_WHITE;
            }
          }
        }
      }

      Int32_t tar_lane_index = -1;
      for (Int32_t k = 0; k < lane_table_size; ++k) {
        if (lane_table[k].lane_index == neighbor.lane_index) {
          tar_lane_index = k;
        }
      }
      if (tar_lane_index < 0) {
        continue;
      }
      LaneInfo& tar_lane = lane_table[tar_lane_index];
      common::Vec2d point;
      LaneInfo::Boundary::BoundaryAssociation* boundary_point = Nullptr_t;
      /* k003 2022-12-26 longjiaoy (start) */
      const Lane::Boundary& left_boundary =
          GetLaneTable()[neighbor.lane_index].left_boundary();
      if (left_boundary.boundary_type_samples.Size() > 0) {
        left_boundary_type = left_boundary.boundary_type_samples[0].type;
      }
      /* k003 2022-12-26 longjiaoy (end) */
      if (left_boundary_valid) {
        point.set_x(
              neighbor.proj_point.point.x() - left_width *
              common::com_sin(neighbor.proj_point.heading));
        point.set_y(
              neighbor.proj_point.point.y() + left_width *
              common::com_cos(neighbor.proj_point.heading));

        boundary_point = tar_lane.left_boundary.curve.Allocate();
        if (Nullptr_t != boundary_point) {
          boundary_point->s = sample_point_on_ref.s;
          boundary_point->width = left_width;
          boundary_point->type = left_boundary_type;
          boundary_point->point = point;
        }
      }
      /* k003 2022-12-26 longjiaoy (start) */
      const Lane::Boundary& right_boundary =
          GetLaneTable()[neighbor.lane_index].right_boundary();
      if (right_boundary.boundary_type_samples.Size() > 0) {
        right_boundary_type = right_boundary.boundary_type_samples[0].type;
      }
      /* k003 2022-12-26 longjiaoy (end) */
      if (right_boundary_valid) {
        point.set_x(
              neighbor.proj_point.point.x() + right_width *
              common::com_sin(neighbor.proj_point.heading));
        point.set_y(
              neighbor.proj_point.point.y() - right_width *
              common::com_cos(neighbor.proj_point.heading));

        boundary_point = tar_lane.right_boundary.curve.Allocate();
        if (Nullptr_t != boundary_point) {
          boundary_point->s = sample_point_on_ref.s;
          boundary_point->width = right_width;
          boundary_point->type = right_boundary_type;
          boundary_point->point = point;
        }
      }
    }
  }
}


#if 0
/// 单车道版本
Int32_t DrivingMapImpl::CheckCameraLane(
    const DrivingMapDataSource& data_source) {
  bool new_lane_list_valid = false;
  Int32_t camera_lane_quality = 0;
  if (Nullptr_t == data_source.camera_lane_list) {
    return (camera_lane_quality);
  }

  if (cam_info_.prev_msg_lane_list_head.valid) {
    if (cam_info_.prev_msg_lane_list_head.CalcSequenceDiff(
          cam_info_.prev_msg_lane_list_head.sequence,
          data_source.camera_lane_list->msg_head.sequence) > 0) {
      new_lane_list_valid = true;
    }
  }

  cam_info_.prev_msg_lane_list_head = data_source.camera_lane_list->msg_head;

  if ((data_source.camera_lane_list->quality > 2) &&
      (data_source.camera_lane_list->forward_len > 50.0F)) {
    camera_lane_quality = 3;
  } else {
    camera_lane_quality = data_source.camera_lane_list->quality;
    if (camera_lane_quality > 2) {
      camera_lane_quality = 2;
    }
  }

  // Using previous camera lane information to improve robustness.
  bool using_prev_camera_lane = false;
  if (cam_info_.lane_list.msg_head.valid &&
      ((camera_lane_quality < 2) ||
       (data_source.camera_lane_list->central_line_num < 1) ||
       !new_lane_list_valid)) {
    map_var_t time_elapsed_ms = static_cast<map_var_t>(
          common::CalcElapsedClockMs(cam_info_.lane_list.msg_head.timestamp,
                                     data_source.timestamp)) * 0.001F;
    /// TODO: Consider the situation of driving backward.
    using_prev_camera_lane = true;

    cam_info_.lane_list.forward_len -= time_elapsed_ms * common::com_abs(current_position_.v);
    if ((cam_info_.lane_list.quality > 2) &&
        (cam_info_.lane_list.forward_len > 50.0F)) {
      camera_lane_quality = 3;
    } else {
      camera_lane_quality = cam_info_.lane_list.quality;
      if (camera_lane_quality > 2) {
        camera_lane_quality = 2;
      }
    }

    //std::cout << ">>>>>>> Using previous camera lane, time_elapsed_ms="
    //          << time_elapsed_ms
    //          << std::endl;
  }

  if (!using_prev_camera_lane &&
      data_source.camera_lane_list->msg_head.valid) {
    cam_info_.lane_list = *data_source.camera_lane_list;
  } else {
    bool prev_lane_list_valid = false;
    ad_msg::LaneInfoCameraList& prev_lane_list = cam_info_.lane_list;

    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
        points_2d = temp_data_.sample_2d_points;
    common::Path& prev_cur_central_line = temp_data_.path_list[0];
    prev_cur_central_line.Clear();
    common::PathPoint veh_proj_on_prev_cur_central_line;

    ad_msg::RelativePos rel_pos;
    common::Matrix<map_var_t, 2, 1> point_conv;
    common::Matrix<map_var_t, 3, 3> mat_conv;
    mat_conv.SetIdentity();
    common::Matrix<map_var_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    Int32_t curr_central_line_idx_in_prev_lane_list = -1;

    Int32_t prev_lane_list_time_elapsed = 0;
    if (prev_lane_list.msg_head.valid) {
      prev_lane_list_time_elapsed =common::CalcElapsedClockMs(
            prev_lane_list.msg_head.timestamp, data_source.timestamp);
      if (prev_lane_list_time_elapsed > 500 || prev_lane_list_time_elapsed < 0) {
        prev_lane_list.Clear();
      } else {
        if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
              prev_lane_list.msg_head.timestamp, *data_source.rel_pos_list, &rel_pos)) {
          prev_lane_list.Clear();

          LOG_ERR << "Failed to get pos of lane lines.";
        } else {
          prev_lane_list_valid = true;

          common::Rotate_2D<map_var_t>(rotate_center, rel_pos.heading, &mat_conv);
          common::Translate_2D(rel_pos.x, rel_pos.y, &mat_conv);

          points_2d.Clear();
          for (Int32_t i = 0; i < prev_lane_list.central_line_num; ++i) {
            ad_msg::LaneCentralLineCamera& central_line =
                prev_lane_list.central_lines[i];

            for (Int32_t j = 0; j < central_line.curve_point_num; ++j) {
              point_conv(0) = central_line.curve[j].x;
              point_conv(1) = central_line.curve[j].y;
              common::TransformVert_2D(mat_conv, &point_conv);
              central_line.curve[j].x = point_conv(0);
              central_line.curve[j].y = point_conv(1);

              points_2d.PushBack(common::Vec2d(point_conv(0), point_conv(1)));
            }

            switch (central_line.id) {
            case (0):
              prev_cur_central_line.Construct(points_2d);
              curr_central_line_idx_in_prev_lane_list = i;
              break;
            case (1):
              break;
            case (-1):
              break;
            default:
              LOG_ERR << "Invalid camera central lane id("
                      << central_line.id << ").";
              break;
            }
          }

          /// TODO: 暂时不对车道边界做处理
          prev_lane_list.boundary_line_num = 0;
          ///for (Int32_t i = 0; i < prev_lane_list.lane_line_num; ++i) {
          ///  ad_msg::LaneBoundaryLineCamera& boundary_line =
          ///      prev_lane_list.boundary_lines[i];

          ///  for (Int32_t j = 0; j < boundary_line.curve_point_num; ++j) {
          ///    point_conv(0) = boundary_line.curve[j].x;
          ///    point_conv(1) = boundary_line.curve[j].y;
          ///    common::TransformVert_2D(mat_conv, &point_conv);
          ///    boundary_line.curve[j].x = point_conv(0);
          ///    boundary_line.curve[j].y = point_conv(1);
          ///  }
          ///}

          prev_lane_list.msg_head.timestamp = data_source.timestamp;

          if (prev_cur_central_line.points().Size() < 2 ||
              curr_central_line_idx_in_prev_lane_list < 0) {
            prev_lane_list.Clear();
            prev_lane_list_valid = false;

            LOG_ERR << "Invalid previous camera central line.";
          } else {
            common::Vec2d veh_pos(current_position_.x, current_position_.y);
            if (!prev_cur_central_line.FindProjection(
                  veh_pos, &veh_proj_on_prev_cur_central_line)) {
              prev_lane_list.Clear();
              prev_lane_list_valid = false;

              LOG_ERR << "Failed to find projection on current central line.";
            } else {
              if (common::com_abs(veh_proj_on_prev_cur_central_line.l) > 1.5F) {
                prev_lane_list.Clear();
                prev_lane_list_valid = false;
              }
              if (common::com_abs(common::AngleDiff(veh_proj_on_prev_cur_central_line.heading, current_position_.heading)) >
                  common::com_deg2rad(30.0F)) {
                prev_lane_list.Clear();
                prev_lane_list_valid = false;
              }
              if (common::com_abs(veh_proj_on_prev_cur_central_line.s) > prev_cur_central_line.total_length()) {
                prev_lane_list.Clear();
                prev_lane_list_valid = false;
              }
            }
          }
        }
      }
    } else {
      prev_lane_list.Clear();
    }
  }

  if (!cam_info_.lane_list.msg_head.valid) {
    camera_lane_quality = 0;
  }
  Int64_t time_elapsed = common::CalcElapsedClockMs(
        cam_info_.prev_msg_lane_list_head.timestamp, data_source.timestamp);
  if (time_elapsed >= 500) {
    camera_lane_quality = 0;
  }

  return (camera_lane_quality);
}

#else
/// 多车道版本

void DrivingMapImpl::UpdateCameraLaneAndExtractProperties(
    const DrivingMapDataSource& data_source,
    ad_msg::LaneInfoCameraList* const lane_list,
    CameraLaneProperties* const properties) {

  properties->Clear();

  if (!lane_list->msg_head.valid) {
    // LOG_ERR << "Camera lane list is invalid.";
    return;
  }
  properties->time_elapsed =common::CalcElapsedClockMs(
        lane_list->msg_head.timestamp, data_source.timestamp);
  if ((properties->time_elapsed > 300) ||
      (properties->time_elapsed < 0)) {
    // Timeout
    LOG_ERR << "Timeout to get camera lane list.";
    return;
  }

  const common::Vec2d veh_pos(current_rel_position_.x, current_rel_position_.y);
  ad_msg::RelativePos rel_pos;
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
        lane_list->msg_head.timestamp, *data_source.rel_pos_list, &rel_pos)) {
    LOG_ERR << "Failed to get pos of lane lines.";
    return;
  }

  common::Matrix<map_var_t, 2, 1> point_conv;
  common::Matrix<map_var_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<map_var_t, 2, 1> rotate_center;
  rotate_center.SetZeros();

  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
      points_2d = temp_data_.sample_2d_points;

  Int32_t proj_idx_on_cur_central_line = 0;
  Int32_t proj_idx_on_left_central_line = 0;
  Int32_t proj_idx_on_right_central_line = 0;

  common::Rotate_2D<map_var_t>(rotate_center, rel_pos.heading, &mat_conv);
  common::Translate_2D(rel_pos.x, rel_pos.y, &mat_conv);

  // std::cout << "@@@@@ Synchronize position: x=" << rel_pos.x
  //           << ", y=" << rel_pos.y
  //           << ", h=" << common::com_rad2deg(rel_pos.heading)
  //           << std::endl;

  for (Int32_t i = 0; i < lane_list->center_line_num; ++i) {
    ad_msg::LaneCenterLineCamera& central_line =
        lane_list->center_lines[i];

    points_2d.Clear();
    for (Int32_t j = 0; j < central_line.curve_point_num; ++j) {
      point_conv(0) = central_line.curve[j].x;
      point_conv(1) = central_line.curve[j].y;
      common::TransformVert_2D(mat_conv, &point_conv);
      central_line.curve[j].x = point_conv(0);
      central_line.curve[j].y = point_conv(1);

      points_2d.PushBack(common::Vec2d(point_conv(0), point_conv(1)));
    }
    switch (central_line.id) {
    case (0):
      properties->cur_central_line->Construct(points_2d);
      properties->curr_central_line_idx = i;
      break;
    case (1):
      properties->left_central_line->Construct(points_2d);
      properties->left_central_line_idx = i;
      break;
    case (-1):
      properties->right_central_line->Construct(points_2d);
      properties->right_central_line_idx = i;
      break;
    default:
      LOG_ERR << "Invalid camera central lane id("
              << central_line.id << ").";
      break;
    }

    /* k003 longjiaoy 2022-11-28 (start) */
    // 对车道边界做处理
    for (Int32_t i = 0; i < lane_list->boundary_line_num; ++i) {
      ad_msg::LaneBoundaryLineCamera& boundary_line =
          lane_list->boundary_lines[i];

      for (Int32_t j = 0; j < boundary_line.curve_point_num; ++j) {
        point_conv(0) = boundary_line.curve[j].x;
        point_conv(1) = boundary_line.curve[j].y;
        common::TransformVert_2D(mat_conv, &point_conv);
        boundary_line.curve[j].x = point_conv(0);
        boundary_line.curve[j].y = point_conv(1);
      }
    }
    /* k003 longjiaoy 2022-11-28 (end) */
  }

  // 判断道路是否有效
  if ((properties->cur_central_line->points().Size() > 1) &&
      (properties->curr_central_line_idx >= 0)) {
    if (properties->cur_central_line->FindProjection(
          veh_pos, &proj_idx_on_cur_central_line,
          &(properties->veh_proj_on_cur_central_line))) {
      if ((properties->veh_proj_on_cur_central_line.s >
           properties->cur_central_line->total_length()) ||
          (properties->veh_proj_on_cur_central_line.s < -5.0F)) {
        // 当前车辆位置超出了道路的区域
        properties->valid_curr_center_line = false;

        // std::cout << "@@@@@@ Out of lane range, s="
        //           << properties->veh_proj_on_cur_central_line.s
        //           << ", total_length="
        //           << properties->cur_central_line->total_length()
        //           << std::endl;
      } else {
        properties->valid_curr_center_line = true;
      }
    } else {
      properties->valid_curr_center_line = false;

      LOG_ERR << "Invalid camera central line, can't find projection.";
    }
  } else {
    properties->valid_curr_center_line = false;

    LOG_ERR << "Invalid camera central line, number of path points is too small.";
  }
  if (properties->valid_curr_center_line &&
      (properties->left_central_line->points().Size() > 1) &&
      (properties->left_central_line_idx >= 0)) {
    if (properties->left_central_line->FindProjection(
          veh_pos, &proj_idx_on_left_central_line,
          &(properties->veh_proj_on_left_central_line))) {
      if ((properties->veh_proj_on_left_central_line.s >
           properties->left_central_line->total_length()) ||
          (properties->veh_proj_on_left_central_line.s < -1.0F)) {
        // 当前车辆位置超出了道路的区域
      } else {
        properties->valid_left_center_line = true;

        if (data_source.allow_removing_expired_camera_lane) {
          if (lane_list->center_lines[
              properties->left_central_line_idx].age > 20) {
            properties->valid_left_center_line = false;
          }
        }
      }
    }
  }
  if (properties->valid_curr_center_line &&
      (properties->right_central_line->points().Size() > 1) &&
      (properties->right_central_line_idx >= 0)) {
    if (properties->right_central_line->FindProjection(
          veh_pos, &proj_idx_on_right_central_line,
          &(properties->veh_proj_on_right_central_line))) {
      if ((properties->veh_proj_on_right_central_line.s >
           properties->right_central_line->total_length()) ||
          (properties->veh_proj_on_right_central_line.s < -1.0F)) {
        // 当前车辆位置超出了道路的区域
      } else {
        properties->valid_right_center_line = true;

        if (data_source.allow_removing_expired_camera_lane) {
          if (lane_list->center_lines[
              properties->right_central_line_idx].age > 20) {
            properties->valid_right_center_line = false;
          }
        }
      }
    }
  }
  // 车道的左宽
  properties->left_width_of_cur_lane = lane_list->center_lines[
      properties->curr_central_line_idx].curve[
      proj_idx_on_cur_central_line].left_width;
  if (properties->valid_left_center_line) {
    map_var_t dist =
        properties->veh_proj_on_cur_central_line.l -
        properties->veh_proj_on_left_central_line.l;
    if (dist > properties->left_width_of_cur_lane) {
      properties->left_width_of_cur_lane =
          common::Max(properties->left_width_of_cur_lane, 0.5F * dist);
    } else {
      properties->valid_left_center_line = false;

      LOG_ERR << "The distance between left center line and "
                 "current center line is unexpected, dist=" << dist
              << ", and left_width=" << properties->left_width_of_cur_lane;
    }
  }
  // 车道的右宽
  properties->right_width_of_cur_lane = lane_list->center_lines[
      properties->curr_central_line_idx].curve[
      proj_idx_on_cur_central_line].right_width;
  if (properties->valid_right_center_line) {
    map_var_t dist =
        properties->veh_proj_on_right_central_line.l -
        properties->veh_proj_on_cur_central_line.l;
    if (dist > properties->right_width_of_cur_lane) {
      properties->right_width_of_cur_lane =
          common::Max(properties->right_width_of_cur_lane, 0.5F * dist);
    } else {
      properties->valid_right_center_line = false;

      LOG_ERR << "The distance between right center line and "
                 "current center line is unexpected, dist=" << dist
              << ", and right_width=" << properties->right_width_of_cur_lane;
    }
  }
}

Int32_t DrivingMapImpl::CheckCameraLane(
    const DrivingMapDataSource& data_source) {
  Int32_t camera_lane_quality = 0;

  if (Nullptr_t == data_source.camera_lane_list) {
    cam_info_.lane_list.Clear();
    return (camera_lane_quality);
  }

  cam_info_.age++;
  for (Int32_t i = 0; i < cam_info_.lane_list.center_line_num; ++i) {
    cam_info_.lane_list.center_lines[i].age++;
  }

  ad_msg::LaneInfoCameraList& prev_lane_list = cam_info_.lane_list;
  ad_msg::LaneInfoCameraList& curr_lane_list = temp_data_.lane_list[0];
  ad_msg::LaneInfoCameraList& tmp_lane_list = temp_data_.lane_list[1];
  curr_lane_list = *data_source.camera_lane_list;
  tmp_lane_list.Clear();

  CameraLaneProperties prev_lane_list_properties;
  prev_lane_list_properties.cur_central_line = &(temp_data_.path_list[0]);
  prev_lane_list_properties.left_central_line = &(temp_data_.path_list[1]);
  prev_lane_list_properties.right_central_line = &(temp_data_.path_list[2]);
  prev_lane_list_properties.cur_central_line->Clear();
  prev_lane_list_properties.left_central_line->Clear();
  prev_lane_list_properties.right_central_line->Clear();
  CameraLaneProperties curr_lane_list_properties;
  curr_lane_list_properties.cur_central_line = &(temp_data_.path_list[3]);
  curr_lane_list_properties.left_central_line = &(temp_data_.path_list[4]);
  curr_lane_list_properties.right_central_line = &(temp_data_.path_list[5]);
  curr_lane_list_properties.cur_central_line->Clear();
  curr_lane_list_properties.left_central_line->Clear();
  curr_lane_list_properties.right_central_line->Clear();

  // std::cout << "@@@@@ Get properties of previous camera lane."
  //           << std::endl;

  // 对之前保存的道路做时空同步
  UpdateCameraLaneAndExtractProperties(
        data_source, &prev_lane_list, &prev_lane_list_properties);
  if (!prev_lane_list_properties.valid_curr_center_line) {
    // std::cout << "@@@@@ Invalid previous camera center line." << std::endl;
    prev_lane_list.Clear();
  }

  // std::cout << "@@@@@ Get properties of current camera lane."
  //           << std::endl;

  // 对当前相机识别的道路做时空同步
  UpdateCameraLaneAndExtractProperties(
        data_source, &curr_lane_list, &curr_lane_list_properties);

  // 判断当前识别的车道线是否正常
  if (curr_lane_list_properties.valid_curr_center_line) {
    if ((curr_lane_list_properties.veh_proj_on_cur_central_line.l >
         (curr_lane_list_properties.left_width_of_cur_lane+0.1F))) {
      // 在相机识别的道路中，当前位置在左侧车道中（异常情况）
      LOG_ERR << "In camera lane list, current position is in left lane, "
                 "which is unexpected.";

      curr_lane_list_properties.valid_curr_center_line = false;
    } else if ((curr_lane_list_properties.veh_proj_on_cur_central_line.l <
                (-curr_lane_list_properties.right_width_of_cur_lane-0.1F))) {
      // 在相机识别的道路中，当前位置在右侧车道中（异常情况）
      LOG_ERR << "In camera lane list, current position is in right lane, "
                 "which is unexpected.";

      curr_lane_list_properties.valid_curr_center_line = false;
    } else {
      // 在相机识别的道路中，当前位置在当前车道中（正常情况）
    }
  }
  if (!curr_lane_list_properties.valid_curr_center_line) {
    // std::cout << "@@@@@ Invalid current camera center line." << std::endl;
    curr_lane_list.Clear();
  }

  if (prev_lane_list_properties.valid_curr_center_line &&
      !curr_lane_list_properties.valid_curr_center_line) {
    // 当前道路无效，之前道路有效

    if ((prev_lane_list_properties.veh_proj_on_cur_central_line.l >
         prev_lane_list_properties.left_width_of_cur_lane) &&
        prev_lane_list_properties.valid_left_center_line) {
      // 中心线发生了切换（左变道）
      tmp_lane_list.Clear();
      tmp_lane_list.msg_head = prev_lane_list.msg_head;
      tmp_lane_list.quality = prev_lane_list.quality;
      tmp_lane_list.forward_len = common::Min(
            prev_lane_list.center_lines[
            prev_lane_list_properties.left_central_line_idx].forward_len,
            prev_lane_list_properties.left_central_line->total_length() -
            prev_lane_list_properties.veh_proj_on_left_central_line.s);
      tmp_lane_list.left_width = prev_lane_list.left_width;
      tmp_lane_list.right_width = prev_lane_list.right_width;

      /* k003 longjiaoy 2022-12-08 (start) */
      // 填充车道边界线
      Int32_t left_boundary_idx = prev_lane_list.center_lines[
          prev_lane_list_properties.left_central_line_idx].left_boundary_index;
      Int32_t right_boundary_idx = prev_lane_list.center_lines[
          prev_lane_list_properties.curr_central_line_idx].left_boundary_index;
      Int32_t next_right_boundary_idx = prev_lane_list.center_lines[
          prev_lane_list_properties.curr_central_line_idx].right_boundary_index;
      if (left_boundary_idx >= 0) {
        left_boundary_idx = AddBoundaryToCameraLaneList(
              1, prev_lane_list.boundary_lines[left_boundary_idx],
              &tmp_lane_list);
      }
      if (right_boundary_idx >= 0) {
        right_boundary_idx = AddBoundaryToCameraLaneList(
              -1, prev_lane_list.boundary_lines[right_boundary_idx],
              &tmp_lane_list);
      }
      if (next_right_boundary_idx >= 0) {
        next_right_boundary_idx = AddBoundaryToCameraLaneList(
              -2, prev_lane_list.boundary_lines[next_right_boundary_idx],
              &tmp_lane_list);
      }
      /* k003 longjiaoy 2022-12-08 (end) */

      // 填充车道中心线
      // 当前车道线
      AddCenterLaneToCameraLaneList(
            0, prev_lane_list.center_lines[
            prev_lane_list_properties.left_central_line_idx],
            left_boundary_idx, right_boundary_idx,
            &tmp_lane_list);
      // 左侧车道线(没有)
      // 右侧车道线(删除最右侧车道)
      AddCenterLaneToCameraLaneList(
            -1, prev_lane_list.center_lines[
            prev_lane_list_properties.curr_central_line_idx],
            right_boundary_idx, next_right_boundary_idx,
            &tmp_lane_list);

      prev_lane_list = tmp_lane_list;
    } else if ((prev_lane_list_properties.veh_proj_on_cur_central_line.l <
                -prev_lane_list_properties.right_width_of_cur_lane) &&
               prev_lane_list_properties.valid_right_center_line) {
      // 中心线发生了切换（右变道）
      tmp_lane_list.Clear();
      tmp_lane_list.msg_head = prev_lane_list.msg_head;
      tmp_lane_list.quality = prev_lane_list.quality;
      tmp_lane_list.forward_len = common::Min(
            prev_lane_list.center_lines[
            prev_lane_list_properties.right_central_line_idx].forward_len,
            prev_lane_list_properties.right_central_line->total_length() -
            prev_lane_list_properties.veh_proj_on_right_central_line.s);
      tmp_lane_list.left_width = prev_lane_list.left_width;
      tmp_lane_list.right_width = prev_lane_list.right_width;

      /* k003 longjiaoy 2022-12-08 (start) */
      // 填充车道边界线
      Int32_t left_boundary_idx = prev_lane_list.center_lines[
          prev_lane_list_properties.curr_central_line_idx].right_boundary_index;
      Int32_t right_boundary_idx = prev_lane_list.center_lines[
          prev_lane_list_properties.right_central_line_idx].right_boundary_index;
      Int32_t next_left_boundary_idx = prev_lane_list.center_lines[
          prev_lane_list_properties.curr_central_line_idx].left_boundary_index;
      if (left_boundary_idx >= 0) {
        left_boundary_idx = AddBoundaryToCameraLaneList(
              1, prev_lane_list.boundary_lines[left_boundary_idx],
              &tmp_lane_list);
      }
      if (right_boundary_idx >= 0) {
        right_boundary_idx = AddBoundaryToCameraLaneList(
              -1, prev_lane_list.boundary_lines[right_boundary_idx],
              &tmp_lane_list);
      }
      if (next_left_boundary_idx >= 0) {
        next_left_boundary_idx = AddBoundaryToCameraLaneList(
              2, prev_lane_list.boundary_lines[next_left_boundary_idx],
              &tmp_lane_list);
      }
      /* k003 longjiaoy 2022-12-08 (end) */

      // 填充车道中心线
      // 当前车道线
      AddCenterLaneToCameraLaneList(
            0, prev_lane_list.center_lines[
            prev_lane_list_properties.right_central_line_idx],
            left_boundary_idx, right_boundary_idx,
            &tmp_lane_list);
      // 左侧车道线(删除最左侧车道)
      AddCenterLaneToCameraLaneList(
            1, prev_lane_list.center_lines[
            prev_lane_list_properties.curr_central_line_idx],
            next_left_boundary_idx, left_boundary_idx,
            &tmp_lane_list);
      // 右侧车道线(没有)

      prev_lane_list = tmp_lane_list;
    } else {
      // 维持在原车道行驶
      prev_lane_list.forward_len = common::Min(
            prev_lane_list.center_lines[
            prev_lane_list_properties.curr_central_line_idx].forward_len,
            prev_lane_list_properties.cur_central_line->total_length() -
            prev_lane_list_properties.veh_proj_on_cur_central_line.s);
      if (common::com_abs(
            prev_lane_list_properties.veh_proj_on_cur_central_line.l) > 5.0F) {
        // 车辆位置已经偏离了车道
        prev_lane_list.Clear();
        prev_lane_list_properties.valid_curr_center_line = false;
      }
      if (common::com_abs(
            common::AngleDiff(
              prev_lane_list_properties.veh_proj_on_cur_central_line.heading,
              current_rel_position_.heading)) > common::com_deg2rad(80.0F)) {

        // 车辆行驶方向已经偏离了车道
        prev_lane_list.Clear();
        prev_lane_list_properties.valid_curr_center_line = false;
      }
    }

    if (prev_lane_list_properties.valid_curr_center_line) {
      cam_info_.lane_list.msg_head.timestamp = data_source.timestamp;
      // 不是当前相机识别的车道，将车道质量设置为差
      for (Int32_t i = 0; i < cam_info_.lane_list.center_line_num; ++i) {
        cam_info_.lane_list.center_lines[i].quality = 1;
      }
      cam_info_.lane_list.quality = 1;
    }
  } else if (!prev_lane_list_properties.valid_curr_center_line &&
             curr_lane_list_properties.valid_curr_center_line) {
    // 当前道路有效，之前道路无效

    cam_info_.lane_list = curr_lane_list;
    cam_info_.age = 0;
  } else if (!prev_lane_list_properties.valid_curr_center_line &&
             !curr_lane_list_properties.valid_curr_center_line) {
    // 当前道路无效，之前道路无效

    cam_info_.age = 0;
    cam_info_.lane_list.Clear();
  } else {
    // 当前道路有效，之前道路有效

    // 当前相机识别的道路中心线与之前保存的道路中心线的偏差 (当前行驶的车道)
    map_var_t lat_diff =
        prev_lane_list_properties.veh_proj_on_cur_central_line.l -
        curr_lane_list_properties.veh_proj_on_cur_central_line.l;
    map_var_t abs_lat_diff = common::com_abs(lat_diff);

    bool lane_change = false;

    if (lat_diff > 0.0F) {
      if (/* 有足够的空间容纳两条车道 */
          (abs_lat_diff >
           0.8F*(prev_lane_list_properties.left_width_of_cur_lane +
                 curr_lane_list_properties.right_width_of_cur_lane)) &&
          /* 相对于之前保存的车道，确实发生了变道行为 */
          (prev_lane_list_properties.veh_proj_on_cur_central_line.l >
            prev_lane_list_properties.left_width_of_cur_lane)) {
        // 中心线发生了切换（左变道）
        lane_change = true;

        cam_info_.lane_list = curr_lane_list;
        cam_info_.age = 0;

        if (!curr_lane_list_properties.valid_right_center_line) {
          /* k003 longjiaoy 2022-12-08 (start) */
          // 填充车道边界线
          Int32_t right_boundary_idx = curr_lane_list.center_lines[
              curr_lane_list_properties.curr_central_line_idx].right_boundary_index;
          Int32_t next_right_boundary_idx = prev_lane_list.center_lines[
              prev_lane_list_properties.curr_central_line_idx].right_boundary_index;
          if (next_right_boundary_idx >= 0) {
            next_right_boundary_idx = AddBoundaryToCameraLaneList(
                  -2, prev_lane_list.boundary_lines[next_right_boundary_idx],
                  &cam_info_.lane_list);
          }
          /* k003 longjiaoy 2022-12-08 (end) */
          // 当前相机没有识别右边车道, 将之前保存的0车道加入到当前识别的道路的右边车道中
          Int32_t new_idx = AddCenterLaneToCameraLaneList(
                -1, prev_lane_list.center_lines[
                prev_lane_list_properties.curr_central_line_idx],
                right_boundary_idx, next_right_boundary_idx,
                &cam_info_.lane_list);
          // 不是当前相机识别的车道，将车道质量设置为差
          if (new_idx >= 0) {
            cam_info_.lane_list.center_lines[new_idx].quality = 1;
#if (ENFORCE_CAMERA_LANE_TO_PARALLEL)
            ReplaceCenterCurveOfCameraLane(
                  *curr_lane_list_properties.cur_central_line,
                  -lat_diff, 0.5F*abs_lat_diff,
                  &cam_info_.lane_list.center_lines[new_idx]);
#endif
          }
        }
      } else {
        // 维持在原车道行驶
      }
    } else {
      if (/* 有足够的空间容纳两条车道 */
          (abs_lat_diff >
           0.8F*(prev_lane_list_properties.right_width_of_cur_lane +
                 curr_lane_list_properties.left_width_of_cur_lane)) &&
          /* 相对于之前保存的车道，确实发生了变道行为 */
          (prev_lane_list_properties.veh_proj_on_cur_central_line.l <
           -prev_lane_list_properties.right_width_of_cur_lane)) {
        // 中心线发生了切换（右变道）
        lane_change = true;

        cam_info_.lane_list = curr_lane_list;
        cam_info_.age = 0;

        if (!curr_lane_list_properties.valid_left_center_line) {
          /* k003 longjiaoy 2022-12-08 (start) */
          // 填充车道边界线
          Int32_t left_boundary_idx = curr_lane_list.center_lines[
              curr_lane_list_properties.curr_central_line_idx].left_boundary_index;
          Int32_t next_left_boundary_idx = prev_lane_list.center_lines[
              prev_lane_list_properties.curr_central_line_idx].left_boundary_index;
          if (next_left_boundary_idx >= 0) {
            next_left_boundary_idx = AddBoundaryToCameraLaneList(
                  2, prev_lane_list.boundary_lines[next_left_boundary_idx],
                  &cam_info_.lane_list);
          }
          /* k003 longjiaoy 2022-12-08 (end) */
          // 当前相机没有识别左边车道, 将之前保存的0车道加入到当前识别的道路的左边车道中
          Int32_t new_idx = AddCenterLaneToCameraLaneList(
                1, prev_lane_list.center_lines[
                prev_lane_list_properties.curr_central_line_idx],
                next_left_boundary_idx, left_boundary_idx,
                &cam_info_.lane_list);
          // 不是当前相机识别的车道，将车道质量设置为差
          if (new_idx >= 0) {
            cam_info_.lane_list.center_lines[new_idx].quality = 1;
#if (ENFORCE_CAMERA_LANE_TO_PARALLEL)
            ReplaceCenterCurveOfCameraLane(
                  *curr_lane_list_properties.cur_central_line,
                  -lat_diff, 0.5F*abs_lat_diff,
                  &cam_info_.lane_list.center_lines[new_idx]);
#endif
          }
        }
      } else {
        // 维持在原车道行驶
      }
    }

    if (!lane_change) {
      tmp_lane_list.Clear();
      tmp_lane_list.msg_head = curr_lane_list.msg_head;
      tmp_lane_list.quality = curr_lane_list.quality;
      tmp_lane_list.forward_len = curr_lane_list.forward_len;
      tmp_lane_list.left_width = curr_lane_list.left_width;
      tmp_lane_list.right_width = curr_lane_list.right_width;

      /* k003 longjiaoy 2022-12-08 (start) */
      // 填充车道边界线
      Int32_t left_boundary_idx = curr_lane_list.center_lines[
          curr_lane_list_properties.curr_central_line_idx].left_boundary_index;
      Int32_t right_boundary_idx = curr_lane_list.center_lines[
          curr_lane_list_properties.curr_central_line_idx].right_boundary_index;
      if (left_boundary_idx >= 0) {
        left_boundary_idx = AddBoundaryToCameraLaneList(
              1, curr_lane_list.boundary_lines[left_boundary_idx],
              &tmp_lane_list);
      }
      if (right_boundary_idx >= 0) {
        right_boundary_idx = AddBoundaryToCameraLaneList(
              -1, curr_lane_list.boundary_lines[right_boundary_idx],
              &tmp_lane_list);
      }
      /* k003 longjiaoy 2022-12-08 (end) */

      // 填充车道中心线
      // 当前车道线
      AddCenterLaneToCameraLaneList(
            0, curr_lane_list.center_lines[
            curr_lane_list_properties.curr_central_line_idx],
            left_boundary_idx, right_boundary_idx,
            &tmp_lane_list);
      // 左侧车道线
      if (curr_lane_list_properties.valid_left_center_line) {
        /* k003 longjiaoy 2022-12-08 (start) */
        // 填充车道边界线
        Int32_t next_left_boundary_idx = curr_lane_list.center_lines[
            curr_lane_list_properties.left_central_line_idx].left_boundary_index;
        if (next_left_boundary_idx >= 0) {
          next_left_boundary_idx = AddBoundaryToCameraLaneList(
                2, curr_lane_list.boundary_lines[next_left_boundary_idx],
                &tmp_lane_list);
        }
        /* k003 longjiaoy 2022-12-08 (end) */
        // 将当前相机识别的左车道添加到车道列表中
        AddCenterLaneToCameraLaneList(
              1, curr_lane_list.center_lines[
              curr_lane_list_properties.left_central_line_idx],
              next_left_boundary_idx, left_boundary_idx,
              &tmp_lane_list);
      } else if (!curr_lane_list_properties.valid_left_center_line &&
                 prev_lane_list_properties.valid_left_center_line) {
        map_var_t lat_diff_to_prev_left_lane =
            curr_lane_list_properties.veh_proj_on_cur_central_line.l -
            prev_lane_list_properties.veh_proj_on_left_central_line.l;
        map_var_t abs_lat_diff_to_prev_left_lane =
            common::com_abs(lat_diff_to_prev_left_lane);
        if (/* 当前位置位于之前保存的车道范围内 */
            ((-0.1F <
              prev_lane_list_properties.veh_proj_on_left_central_line.s) &&
             (prev_lane_list_properties.veh_proj_on_left_central_line.s <
              prev_lane_list_properties.left_central_line->total_length())) &&
            /* 有足够的空间容纳两条车道 */
            (abs_lat_diff_to_prev_left_lane >
             0.8F*(prev_lane_list_properties.right_width_of_cur_lane +
                   curr_lane_list_properties.left_width_of_cur_lane))) {
          /* k003 longjiaoy 2022-12-08 (start) */
          // 填充车道边界线
          Int32_t next_left_boundary_idx = prev_lane_list.center_lines[
              prev_lane_list_properties.left_central_line_idx].left_boundary_index;
          if (next_left_boundary_idx >= 0) {
            next_left_boundary_idx = AddBoundaryToCameraLaneList(
                  2, prev_lane_list.boundary_lines[next_left_boundary_idx],
                  &tmp_lane_list);
          }
          /* k003 longjiaoy 2022-12-08 (end) */
          // 将之前保存的左车道添加到车道列表中
          Int32_t new_idx = AddCenterLaneToCameraLaneList(
                1, prev_lane_list.center_lines[
                prev_lane_list_properties.left_central_line_idx],
                next_left_boundary_idx, left_boundary_idx,
                &tmp_lane_list);
          // 不是当前相机识别的车道，将车道质量设置为差
          if (new_idx >= 0) {
            tmp_lane_list.center_lines[new_idx].quality = 1;
#if (ENFORCE_CAMERA_LANE_TO_PARALLEL)
            ReplaceCenterCurveOfCameraLane(
                  *curr_lane_list_properties.cur_central_line,
                  lat_diff_to_prev_left_lane,
                  0.5F*abs_lat_diff_to_prev_left_lane,
                  &tmp_lane_list.center_lines[new_idx]);
#endif
          }
        }
      } else {
        // 没有可用的左侧车道中心线
      }
      // 右侧车道线
      if (curr_lane_list_properties.valid_right_center_line) {
        /* k003 longjiaoy 2022-12-08 (start) */
        // 填充车道边界线
        Int32_t next_right_boundary_idx = curr_lane_list.center_lines[
            curr_lane_list_properties.right_central_line_idx].right_boundary_index;
        if (next_right_boundary_idx >= 0) {
          next_right_boundary_idx = AddBoundaryToCameraLaneList(
                -2, curr_lane_list.boundary_lines[next_right_boundary_idx],
                &tmp_lane_list);
        }
        /* k003 longjiaoy 2022-12-08 (end) */
        // 将当前相机识别的右车道添加到车道列表中
        AddCenterLaneToCameraLaneList(
              -1, curr_lane_list.center_lines[
              curr_lane_list_properties.right_central_line_idx],
              right_boundary_idx, next_right_boundary_idx,
              &tmp_lane_list);
      } else if (!curr_lane_list_properties.valid_right_center_line &&
                 prev_lane_list_properties.valid_right_center_line) {
        map_var_t lat_diff_to_prev_right_lane =
            curr_lane_list_properties.veh_proj_on_cur_central_line.l -
            prev_lane_list_properties.veh_proj_on_right_central_line.l;
        map_var_t abs_lat_diff_to_prev_right_lane =
            common::com_abs(lat_diff_to_prev_right_lane);
        if (/* 当前位置位于之前保存的车道范围内 */
            ((-0.1F <
             prev_lane_list_properties.veh_proj_on_right_central_line.s) &&
            (prev_lane_list_properties.veh_proj_on_right_central_line.s <
             prev_lane_list_properties.right_central_line->total_length())) &&
            /* 有足够的空间容纳两条车道 */
            (abs_lat_diff_to_prev_right_lane >
             0.8F*(prev_lane_list_properties.left_width_of_cur_lane +
                   curr_lane_list_properties.right_width_of_cur_lane))) {
          /* k003 longjiaoy 2022-12-08 (start) */
          // 填充车道边界线
          Int32_t next_right_boundary_idx = prev_lane_list.center_lines[
              prev_lane_list_properties.right_central_line_idx].right_boundary_index;
          if (next_right_boundary_idx >= 0) {
            next_right_boundary_idx = AddBoundaryToCameraLaneList(
                  -2, prev_lane_list.boundary_lines[next_right_boundary_idx],
                  &tmp_lane_list);
          }
          /* k003 longjiaoy 2022-12-08 (end) */
          // 将之前保存的右车道添加到车道列表中
          Int32_t new_idx = AddCenterLaneToCameraLaneList(
                -1, prev_lane_list.center_lines[
                prev_lane_list_properties.right_central_line_idx],
                right_boundary_idx, next_right_boundary_idx,
                &tmp_lane_list);
          // 不是当前相机识别的车道，将车道质量设置为差
          if (new_idx >= 0) {
            tmp_lane_list.center_lines[new_idx].quality = 1;
#if (ENFORCE_CAMERA_LANE_TO_PARALLEL)
            ReplaceCenterCurveOfCameraLane(
                  *curr_lane_list_properties.cur_central_line,
                  lat_diff_to_prev_right_lane,
                  0.5F*abs_lat_diff_to_prev_right_lane,
                  &tmp_lane_list.center_lines[new_idx]);
#endif
          }
        }
      } else {
        // 没有可用的右侧车道中心线
      }
      cam_info_.age = 0;
      cam_info_.lane_list = tmp_lane_list;
    }
  }

  //printf("    cam_info_.lane_list.forward_len=%0.1f\n",
  //       cam_info_.lane_list.forward_len);

  if ((cam_info_.lane_list.quality > 2) &&
      (cam_info_.lane_list.forward_len > 40.0F)) {
    camera_lane_quality = 3;
  } else {
    camera_lane_quality = cam_info_.lane_list.quality;
    if (cam_info_.lane_list.forward_len < 10.0F) {
      if (camera_lane_quality > 1) {
        camera_lane_quality = 1;
      }
    }
    if (camera_lane_quality > 2) {
      camera_lane_quality = 2;
    }
  }

  return (camera_lane_quality);
}

#endif

Int32_t DrivingMapImpl::AddCenterLaneToCameraLaneList(
    const Int32_t id,
    const ad_msg::LaneCenterLineCamera& central_line,
    Int32_t left_boundary_idx, Int32_t right_boundary_idx,
    ad_msg::LaneInfoCameraList* lane_list) const {
  Int32_t curr_line_idx = -1;
  Int32_t left_line_idx = -1;
  Int32_t right_line_idx = -1;
  Int32_t search_count =
      common::Min(lane_list->center_line_num,
                  (Int32_t)ad_msg::LaneInfoCameraList::MAX_LANE_CENTER_LINE_NUM);
  for (Int32_t i = 0; i < search_count; ++i) {
    const ad_msg::LaneCenterLineCamera& line = lane_list->center_lines[i];

    if (line.id == id) {
      curr_line_idx = i;
    } else if (line.id == (id+1)) {
      left_line_idx = i;
    } else if (line.id == (id-1)) {
      right_line_idx = i;
    } else {
      // nothing to do
    }
  }

  if (curr_line_idx < 0) {
    if (lane_list->center_line_num <
        ad_msg::LaneInfoCameraList::MAX_LANE_CENTER_LINE_NUM) {
      curr_line_idx = lane_list->center_line_num;
      lane_list->center_line_num++;
    }
  }
  if (curr_line_idx < 0) {
    LOG_ERR << "Can't add new lane to camera lane list, storage is full.";
    return (-1);
  }

  lane_list->center_lines[curr_line_idx] = central_line;
  lane_list->center_lines[curr_line_idx].id = id;
  /* k003 longjiaoy 2022-12-08 (begin)*/
  lane_list->center_lines[curr_line_idx].left_boundary_index = left_boundary_idx;
  lane_list->center_lines[curr_line_idx].right_boundary_index = right_boundary_idx;
  /* k003 longjiaoy 2022-12-08 (end)*/

  return (curr_line_idx);
}

/* k003 longjiaoy 2022-12-08 (begin)*/
Int32_t DrivingMapImpl::AddBoundaryToCameraLaneList(
    const Int32_t id,
    const ad_msg::LaneBoundaryLineCamera& boundary,
    ad_msg::LaneInfoCameraList* lane_list) const {
  Int32_t curr_line_idx = -1;

  Int32_t search_count =
      common::Min(lane_list->boundary_line_num,
                  (Int32_t)ad_msg::LaneInfoCameraList::MAX_LANE_BOUNDARY_LINE_NUM);
  for (Int32_t i = 0; i < search_count; ++i) {
    const ad_msg::LaneBoundaryLineCamera& line = lane_list->boundary_lines[i];

    if (line.id == id) {
      curr_line_idx = i;
    }
  }

  if (curr_line_idx < 0) {
    if (lane_list->boundary_line_num <
        ad_msg::LaneInfoCameraList::MAX_LANE_BOUNDARY_LINE_NUM) {
      curr_line_idx = lane_list->boundary_line_num;
      lane_list->boundary_line_num++;
    }
  }
  if (curr_line_idx < 0) {
    LOG_ERR << "Can't add new boundary to camera boundary lane list, storage is full.";
    return (-1);
  }

  lane_list->boundary_lines[curr_line_idx] = boundary;
  lane_list->boundary_lines[curr_line_idx].id = id;

  return (curr_line_idx);
}
/* k003 longjiaoy 2022-12-08 (end)*/

void DrivingMapImpl::ReplaceCenterCurveOfCameraLane(
    const common::Path& ref_path,
    map_var_t lat_offset,
    map_var_t new_half_lane_width,
    ad_msg::LaneCenterLineCamera* central_line) {
  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
      points_2d = temp_data_.sample_2d_points;
  points_2d.Clear();

  ref_path.CalcSamplePointsByLatOffsetting(lat_offset, &points_2d);
  Int32_t points_num = points_2d.Size();
  if (points_num > ad_msg::LaneCenterLineCamera::MAX_CURVE_POINT_NUM) {
    points_num = ad_msg::LaneCenterLineCamera::MAX_CURVE_POINT_NUM;
  }

  for (Int32_t i = 0; i < points_num; ++i) {
    central_line->curve[i].x = points_2d[i].x();
    central_line->curve[i].y = points_2d[i].y();
    central_line->curve[i].left_width = new_half_lane_width;
    central_line->curve[i].right_width = new_half_lane_width;
  }

  central_line->curve_point_num = points_num;
}

void DrivingMapImpl::CalcPredictedPathOfVehicle() {
  sample_points_of_pred_path_.Clear();

  map_var_t start_x = current_rel_position_.x;
  map_var_t start_y = current_rel_position_.y;
  map_var_t start_h = current_rel_position_.heading;

  map_var_t velocity_for_pred =
      common::Max(5.0F/3.6F, common::com_abs(current_rel_position_.v));
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    velocity_for_pred = -velocity_for_pred;
  }
  map_var_t yaw_rate = current_rel_position_.yaw_rate;
  if (current_rel_position_.v < 10.0F/3.6F) {
    veh_model::VehicleModelWrapper veh_model;
    yaw_rate = veh_model.CalcYawRateFromSteeringAngle(
          chassis_.steering_wheel_angle, 10.0F/3.6F);
  }

  map_var_t sin_heading = common::com_sin(current_rel_position_.heading);
  map_var_t cos_heading = common::com_cos(current_rel_position_.heading);

  map_var_t step_len = common::Max(2.0F, 0.5F*common::com_abs(velocity_for_pred));
  Int32_t loop_num = common::Max(30.0F, common::com_abs(velocity_for_pred)*6.0F) / step_len + 1;

  //if (current_rel_position_.v < -0.001F) {
  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    step_len = -step_len;
  }

  if (common::com_abs(yaw_rate) < 0.0001F /*0.005729578 degree/s*/) {
    for (Int32_t i = -1; i < loop_num; ++i) {
      map_var_t s = i * step_len;

      map_var_t x = start_x + s * cos_heading;
      map_var_t y = start_y + s * sin_heading;

      sample_points_of_pred_path_.PushBack(phoenix::common::Vec2d(x, y));
    }
  } else {
    map_var_t r = velocity_for_pred / yaw_rate;

    for (Int32_t i = -1; i < loop_num; ++i) {
      map_var_t t = i * step_len / velocity_for_pred;
      map_var_t delta_h = yaw_rate*t;
      if (common::com_abs(delta_h) > common::com_deg2rad(120.0F)) {
        break;
      }
      map_var_t heading_changed = common::NormalizeAngle(start_h + delta_h);
      map_var_t sin_heading_changed = common::com_sin(heading_changed);
      map_var_t cos_heading_changed = common::com_cos(heading_changed);
      map_var_t x = start_x - r*sin_heading + r*sin_heading_changed;
      map_var_t y = start_y + r*cos_heading - r*cos_heading_changed;

      sample_points_of_pred_path_.PushBack(phoenix::common::Vec2d(x, y));
    }
  }
}

bool DrivingMapImpl::CalcFollowingPath(
    const DrivingMapDataSource& data_source) {
  const ad_msg::ObstacleList& obj_list = obstacle_list_;

  if (!obj_list.msg_head.valid) {
    return false;
  }
  if (obj_list.obstacle_num < 1) {
    return false;
  }

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        obj_list.msg_head.timestamp, data_source.timestamp);
  if ((time_elapsed > 500) || (time_elapsed < 0)) {
    return false;
  }

  ad_msg::RelativePos rel_pos;
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
        obj_list.msg_head.timestamp, *(data_source.rel_pos_list),
        &rel_pos)) {
    LOG_ERR << "Failed to get relative pos of obstacle list.";
    return false;
  }

  common::Matrix<map_var_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<map_var_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  common::Rotate_2D<map_var_t>(rotate_center,
                               rel_pos.heading - current_rel_position_.heading,
                               &mat_conv);
  common::Translate_2D(rel_pos.x - current_rel_position_.x,
                       rel_pos.y - current_rel_position_.y,
                       &mat_conv);

  FollowingTargetInfo following_target_info;
  if (!FindFollowingTarget(obj_list, rel_pos, mat_conv,
                           &following_target_info)) {
    return false;
  }
  if (!following_target_info.valid) {
    return false;
  }

  map_var_t tar_curvature =
      2.0F * following_target_info.obj_rel_y /
      (following_target_info.obj_rel_x * following_target_info.obj_rel_x +
       following_target_info.obj_rel_y * following_target_info.obj_rel_y);

  following_path_info_.Clear();
  following_path_info_.following_target = following_target_info;
  following_path_info_.following_path_curvature = tar_curvature;

  map_var_t start_x = current_rel_position_.x;
  map_var_t start_y = current_rel_position_.y;
  map_var_t start_h = current_rel_position_.heading;
  map_var_t sin_heading = common::com_sin(current_rel_position_.heading);
  map_var_t cos_heading = common::com_cos(current_rel_position_.heading);

  map_var_t velocity_for_pred =
      common::Max(5.0F/3.6F, common::com_abs(current_rel_position_.v));
  map_var_t step_len = common::Max(2.0F, 0.5F*velocity_for_pred);
  Int32_t loop_num = common::Max(30.0F, velocity_for_pred*6.0F) / step_len + 1;

  if (common::com_abs(tar_curvature) < 0.0001F) {
    for (Int32_t i = -1; i < loop_num; ++i) {
      map_var_t s = i * step_len;
      map_var_t x = start_x + s * cos_heading;
      map_var_t y = start_y + s * sin_heading;

      following_path_info_.following_path.PushBack(
            phoenix::common::Vec2d(x, y));
    }
  } else {
    map_var_t r = 1.0F / tar_curvature;

    for (Int32_t i = -1; i < loop_num; ++i) {
      Float64_t s = i * step_len;
      Float64_t delta_h = common::NormalizeAngle(tar_curvature * s);
      if (common::com_abs(delta_h) > common::com_deg2rad(120.0F)) {
        break;
      }
      map_var_t heading_changed = common::NormalizeAngle(start_h + delta_h);
      map_var_t sin_heading_changed = common::com_sin(heading_changed);
      map_var_t cos_heading_changed = common::com_cos(heading_changed);
      map_var_t x = start_x - r*sin_heading + r*sin_heading_changed;
      map_var_t y = start_y + r*cos_heading - r*cos_heading_changed;

      following_path_info_.following_path.PushBack(
            phoenix::common::Vec2d(x, y));
    }
  }

  following_path_info_.valid = true;

  return true;
}

bool DrivingMapImpl::FindFollowingTarget(
    const ad_msg::ObstacleList& obj_list,
    const ad_msg::RelativePos& rel_pos,
    const common::Matrix<map_var_t, 3, 3>& mat_conv,
    FollowingTargetInfo* const following_target) {
  following_target->Clear();

  const map_var_t max_lon_dist_limit =
      common::Max(40.0F, common::com_abs(current_rel_position_.v) * 4.0F);

  common::Matrix<map_var_t, 2, 1> point_conv;
  FollowingTargetInfo min_cost_target_loose;
  FollowingTargetInfo min_cost_target_in_range;
  for (Int32_t i = 0; i < obj_list.obstacle_num; ++i) {
    const ad_msg::Obstacle& obj = obj_list.obstacles[i];

    point_conv(0) = obj.x;
    point_conv(1) = obj.y;
    common::TransformVert_2D(mat_conv, &point_conv);

    map_var_t lon_dist = point_conv(0);
    map_var_t lat_dist = common::com_abs(point_conv(1));

    // 只能跟随前方目标
    if (lon_dist < (param_.dist_of_localization_to_front + 1.0F)) {
      continue;
    }
    // 不跟随太远的目标
#if 0
    if (lon_dist > max_lon_dist_limit) {
      continue;
    }
#endif
    // 不跟随不确定的障碍物
    if (obj.confidence < 80) {
      continue;
    }
    // 只跟随融合的障碍物
    if (ad_msg::OBJ_PRCP_TYPE_FUSED != obj.perception_type) {
      continue;
    }
    // 只跟随车辆
    if ((ad_msg::OBJ_TYPE_PASSENGER_VEHICLE != obj.type) &&
        (ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE != obj.type) &&
        (ad_msg::OBJ_TYPE_SPECIAL_VEHICLE != obj.type) &&
        (ad_msg::OBJ_TYPE_OTHER_VEHICLE != obj.type)) {
      continue;
    }

    if (!obj.dynamic) {
      // static obstacle
      // continue;

      // 不跟随逆向行驶的车辆
      if (obj.v_x < -1.0F) {
        continue;
      }
    } else {
      // dynamic obstacle
      // 只跟随同向行驶的目标
      map_var_t angle_diff = common::com_abs(common::NormalizeAngle(
          obj.obb.heading + rel_pos.heading - current_rel_position_.heading));
      if (angle_diff < common::com_deg2rad(60.0F)) {
        // 障碍物与自车同向行驶
      } else if (angle_diff > common::com_deg2rad(120.0F)) {
        // 障碍物与自车逆向行驶
        continue;
      } else {
        // 障碍物横穿目标轨迹
        continue;
      }
    }

    FollowingTargetInfo target;
    target.valid = true;
    target.lon_dist = lon_dist;
    target.lat_dist = lat_dist;
    target.obj_rel_x = point_conv(0);
    target.obj_rel_y = point_conv(1);
    target.cost = static_cast<Int32_t>(1.0F*lon_dist + 5.0F*lat_dist);

    // 不跟随横向偏差过大的目标
    map_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.loose_following_target_lat_dist_limit_table, lon_dist, &t);
    map_var_t lat_dist_limit = common::Lerp(
          param_.loose_following_target_lat_dist_limit_table[lower].value,
          param_.loose_following_target_lat_dist_limit_table[lower+1].value,
          t);
    if (lat_dist > lat_dist_limit) {
      continue;
    }

    // 选择代价最小的目标作为跟随目标
    if (!min_cost_target_loose.valid) {
      min_cost_target_loose = target;
    } else {
      if (target < min_cost_target_loose) {
        min_cost_target_loose = target;
      }
    }

    // 不跟随横向偏差过大的目标
    t = 0;
    lower = common::LerpInOrderedTable(
          param_.following_target_lat_dist_limit_table, lon_dist, &t);
    lat_dist_limit = common::Lerp(
          param_.following_target_lat_dist_limit_table[lower].value,
          param_.following_target_lat_dist_limit_table[lower+1].value,
          t);
    if (lat_dist > lat_dist_limit) {
      continue;
    }

    // 选择代价最小的目标作为跟随目标
    if (!min_cost_target_in_range.valid) {
      min_cost_target_in_range = target;
    } else {
      if (target < min_cost_target_in_range) {
        min_cost_target_in_range = target;
      }
    }
  }

  if (min_cost_target_in_range.valid) {
    *following_target = min_cost_target_in_range;
  } else {
    if (min_cost_target_loose.valid) {
      *following_target = min_cost_target_in_range;
      if (following_target->lat_dist > 0.05F) {
        following_target->lat_dist = 0.05F;
      } else if (following_target->lat_dist < -0.05F) {
        following_target->lat_dist = -0.05F;
      } else {
        // nothing to do
      }
      if (following_target->obj_rel_y > 0.05F) {
        following_target->obj_rel_y = 0.05F;
      } else if (following_target->obj_rel_y < -0.05F) {
        following_target->obj_rel_y = -0.05F;
      } else {
        // nothing to do
      }
    } else {
      return false;
    }
  }

  return true;
}

void DrivingMapImpl::UpdateObstacles(const ad_msg::ObstacleList& obj_list) {
  obstacle_map_.Clear();

#if ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST
  /// Performance of creating obstacles map (Start)
  phoenix::common::Stopwatch performance_timer_creating_obstacles_map;
#endif

  ad_msg::RelativePos rel_pos;
  if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
        obj_list.msg_head.timestamp, GetRelativePosList(), &rel_pos)) {
    LOG_ERR << "Failed to get relative pos of obstacle list.";
    obstacle_list_.Clear();
    return;
  }

  common::Matrix<map_var_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<map_var_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  common::Rotate_2D<map_var_t>(rotate_center, rel_pos.heading, &mat_conv);
  common::Translate_2D(rel_pos.x, rel_pos.y, &mat_conv);
  common::Matrix<map_var_t, 2, 1> point_conv;

  obstacle_list_ = obj_list;
  obstacle_list_.msg_head.timestamp = GetRelativePosList().msg_head.timestamp;

  Object object;
  for (Int32_t i = 0; i < obstacle_list_.obstacle_num; ++i) {
    ad_msg::Obstacle& obstacle = obstacle_list_.obstacles[i];

    // 障碍物ID
    object.id = obstacle.id;
    // 障碍物位置
    point_conv(0) = obstacle.x;
    point_conv(1) = obstacle.y;
    common::TransformVert_2D(mat_conv, &point_conv);
    object.pos.set_x(point_conv(0));
    object.pos.set_y(point_conv(1));
    obstacle.x = point_conv(0);
    obstacle.y = point_conv(1);
    // 障碍物包围盒
    point_conv(0) = obstacle.obb.x;
    point_conv(1) = obstacle.obb.y;
    common::TransformVert_2D(mat_conv, &point_conv);
    object.obb.set_center(point_conv(0), point_conv(1));
    map_var_t corrected_heading =
        common::NormalizeAngle(obstacle.obb.heading + rel_pos.heading);
    object.obb.set_unit_direction(
          common::com_cos(corrected_heading),
          common::com_sin(corrected_heading));
    object.obb.set_extents(obstacle.obb.half_length, obstacle.obb.half_width);
    obstacle.obb.x = point_conv(0);
    obstacle.obb.y = point_conv(1);
    obstacle.obb.heading = corrected_heading;
    // Heading
    object.heading = corrected_heading;
    // 障碍物高度
    object.height = obstacle.height;
    // 离地面高度
    object.height_to_ground = obstacle.height_to_ground;
    // 障碍物类型
    object.type = obstacle.type;
    // 是否是动态障碍物
    object.dynamic = obstacle.dynamic;
    // 障碍物存在的置信度
    object.confidence = obstacle.confidence;
    // 感知类别
    object.perception_type = obstacle.perception_type;
    // 是否应当忽略此障碍物（例如道路外的静态障碍物（人、车等例外），
    // 车辆正后方的动态障碍物，车辆后方的静态障碍物等）
    object.ignore = false;
    // 障碍物速度，沿着车身纵轴的速度
    object.v_x = obstacle.v_x;
    // 障碍物速度，沿着车身横轴的速度
    object.v_y = obstacle.v_y;
    // 障碍物速度
#if 0
    object.v = common::com_sqrt(object.v_x*object.v_x + object.v_y*object.v_y);
#else
    object.v = obstacle.v;
#endif
    // 障碍物加速度，沿着车身纵轴的加速度
    object.a_x = obstacle.a_x;
    // 障碍物加速度，沿着车身横轴的加速度
    object.a_y = obstacle.a_y;
    // 障碍物加速度
#if 0
    object.a = common::com_sqrt(object.a_x*object.a_x + object.a_y*object.a_y);
#else
    object.a = obstacle.a;
#endif
    // 角速度
    object.yaw_rate = 0.0F;

    // 与参考线之间的关联信息
    UpdateRefLineInfoOfObject(&object);
    obstacle.proj_on_major_ref_line.valid = object.ref_line.valid;
    obstacle.proj_on_major_ref_line.x = object.ref_line.proj_on_ref.point.x();
    obstacle.proj_on_major_ref_line.y = object.ref_line.proj_on_ref.point.y();
    obstacle.proj_on_major_ref_line.heading = object.ref_line.proj_on_ref.heading;
    obstacle.proj_on_major_ref_line.curvature = object.ref_line.proj_on_ref.curvature;
    obstacle.proj_on_major_ref_line.s = object.ref_line.proj_on_ref.s;
    obstacle.proj_on_major_ref_line.l = object.ref_line.proj_on_ref.l;

    // 保存对动态障碍物预测的轨迹
    if (object.dynamic != 0) {
      UpdatePredictedPathOfDynamicObject(obstacle, &object);
    } else {
      object.ClearPredTrj();
    }
    if (!obstacle_map_.AddObject(object)) {
      LOG_ERR << "Failed to add object [" << i << "] to object map.";
      break;
    }
  }
  obstacle_map_.CompleteAddingObject();

  CollisionTestObj test_obj;
  CreateVehOBB(
        common::Vec2d(current_rel_position_.x, current_rel_position_.y),
        current_rel_position_.heading,
        &test_obj.obb);
  test_obj.obb_circumradius = test_obj.obb.CalcCircumradius();
  obstacle_map_.IgnoreRightBehindDynamicObject(test_obj);

#if ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST
  /// Performance of creating obstacles map (End)
  std::cout << "Creating obstacles map spend "
            << performance_timer_creating_obstacles_map.Elapsed()
            << "ms." << std::endl;
#endif
}

void DrivingMapImpl::UpdateRefLineInfoOfObject(Object* object) {
  object->ref_line.ref_line_index = GetMajorRefLineIndex();
  if (object->ref_line.ref_line_index < 0) {
    object->ref_line.valid = false;
    return;
  }

  const common::Path& ref_line_curve =
      GetReferenceLineCurve(object->ref_line.ref_line_index);

  if (!ref_line_curve.FindProjection(
        object->pos, &object->ref_line.proj_on_ref)) {
    object->ref_line.valid = false;
    return;
  }

  object->ref_line.valid = true;
}

void DrivingMapImpl::UpdatePredictedPathOfDynamicObject(
    const ad_msg::Obstacle& obstacle, Object* object) {
  object->ClearPredTrj();

  bool valid_lane = false;
  switch (driving_map_type_) {
  case (DRIVING_MAP_TYPE_HD_MAP):
    valid_lane = true;
    break;
  case (DRIVING_MAP_TYPE_CAMERA_LANE):
    valid_lane = true;
    break;
  case (DRIVING_MAP_TYPE_FOLLOWING_PATH):
    valid_lane = true;
    break;
  case (DRIVING_MAP_TYPE_PRED_PATH):
    valid_lane = false;
    break;
  case (DRIVING_MAP_TYPE_MIXED_HD_MAP):
    valid_lane = true;
    break;
  default:
    valid_lane = false;
    LOG_ERR << "Invalid driving map type.";
    break;
  }

  Int32_t path_count = obstacle.pred_path_num;
  if (path_count > ad_msg::Obstacle::MAX_PRED_PATH_NUM) {
    path_count = ad_msg::Obstacle::MAX_PRED_PATH_NUM;
  }
  Int32_t points_count = 0;

  if (path_count > 0) {
    for (Int32_t i = 0; i < path_count; ++i) {
      points_count = obstacle.pred_path_point_num[i];
      if (points_count > ad_msg::Obstacle::MAX_PRED_PATH_POINT_NUM) {
        points_count = ad_msg::Obstacle::MAX_PRED_PATH_POINT_NUM;
      }
      if (points_count > 0) {
        common::StaticVector<common::TrajectoryPoint,
            MAX_OBJ_PRED_TRAJ_POINT_NUM>* pred_path =
            object->pred_trajectory.Allocate();
        if (Nullptr_t == pred_path) {
          LOG_ERR << "Can't store predicted path any more, storage is full.";
          break;
        }
        for (Int32_t j = 0; j < points_count; ++j) {
          common::TrajectoryPoint* point = pred_path->Allocate();
          if (Nullptr_t == point) {
            LOG_ERR << "Can't store point of predicted path, storage is full.";
            break;
          }
          point->path_point.point.set_x(obstacle.pred_path[i][j].x);
          point->path_point.point.set_y(obstacle.pred_path[i][j].y);
          point->path_point.heading = obstacle.pred_path[i][j].heading;
          point->path_point.curvature = 0.0F;  // not needed, so set to 0.
          point->path_point.s = obstacle.pred_path[i][j].s;
          point->path_point.l = 0.0F;  // not needed, so set to 0.
        }
      }
    }
#if 0
    // If the predicted path from perception have no "heading" and "s",
    // these code below can be used to compute the "heading" and "s".
    path_count = object->pred_trajectory.Size();
    if (path_count <= 0) {
      LOG_ERR << "The input predicted path is empty.";
      return;
    }
    for (Int32_t i = 0; i < path_count; ++i) {
      common::StaticVector<common::TrajectoryPoint,
          Object::MAX_OBJ_PRED_TRAJ_POINT_NUM>& pred_path =
          object->pred_trajectory[i];
      points_count = pred_path.Size();
      for (Int32_t j = 0; j < points_count; ++j) {
        common::TrajectoryPoint& point = pred_path[j];
        point.path_point.l = 0;  // not needed, so set to 0.
        if (0 == j) {
          if (points_count > 1) {
            point.path_point.heading = GetHeadingFromSegment(
                  point.path_point.point, pred_path[j+1].path_point.point);
            point.path_point.curvature = 0;  // not needed, so set to 0.
            point.path_point.s = point.path_point.point.DistanceTo(
                  object->pos);
          } else {
            point.path_point.heading = object->heading;
            point.path_point.curvature = 0;  // not needed, so set to 0.
            point.path_point.s = point.path_point.point.DistanceTo(
                  object->pos);
          }
        } else if ((points_count-1) == j) {
          point.path_point.heading = pred_path[j-1].path_point.heading;
          point.path_point.curvature = 0;  // not needed, so set to 0.
          point.path_point.s = point.path_point.point.DistanceTo(
                pred_path[j-1].path_point.point) + pred_path[j-1].path_point.s;
        } else {
          point.path_point.heading = GetHeadingFromSegment(
                point.path_point.point, pred_path[j+1].path_point.point);
          point.path_point.curvature = 0;  // not needed, so set to 0.
          point.path_point.s = point.path_point.point.DistanceTo(
                pred_path[j-1].path_point.point) + pred_path[j-1].path_point.s;
        }
      }
    }
#endif
  } else {
    map_var_t sample_step_len =
        common::Max(2.0F * object->obb.extents().x(),
                    param_.pred_path_sample_time * object->v);
    if (sample_step_len > param_.pred_path_max_sample_len) {
      sample_step_len = param_.pred_path_max_sample_len;
    }

    common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
        sample_points = temp_data_.sample_points;
    sample_points.Clear();

    Int32_t sample_size = static_cast<Int32_t>(MAX_OBJ_PRED_TRAJ_POINT_NUM) - 1;

    bool find = false;
    Int32_t nearest_lane_index;
    common::PathPoint proj_point_on_lane;
    if (valid_lane &&
        map_space_.FindProjOfNearestLane(
          object->pos, &nearest_lane_index, &proj_point_on_lane)) {
      if (map_space_.IsValidLaneIndex(nearest_lane_index) &&
          (common::com_abs(proj_point_on_lane.l) < 10.0F)) {
        const Lane& nearest_lane = map_space_.GetLane(nearest_lane_index);
        if ((0.0F < proj_point_on_lane.s) &&
            (proj_point_on_lane.s <
             nearest_lane.central_curve().total_length())) {

          map_var_t abs_angle_diff = common::com_abs(
                common::AngleDiff(proj_point_on_lane.heading, object->heading));
          if (abs_angle_diff < common::com_deg2rad(60.0F)) {
            find = true;
            FindObsatclePredPathByMap(
                  true, nearest_lane_index, proj_point_on_lane,
                  sample_size, sample_step_len, &sample_points, object);

          } else if (abs_angle_diff > common::com_deg2rad(120.0F)) {
            find = true;
            FindObsatclePredPathByMap(
                  false, nearest_lane_index, proj_point_on_lane,
                  sample_size, sample_step_len, &sample_points, object);
          } else {
            // nothing to do
          }
        }
      }
    }

    if (false == find) {
#if 1
      sample_points.Clear();
      if (0.0F > proj_point_on_lane.s) {
        common::PathPoint point;
        point.point = object->pos;
        point.heading = object->heading;
        point.curvature = 0.0F;  // not needed, so set to 0.
        point.s = 0.0F;
        point.l = 0.0F;  // not needed, so set to 0.
        sample_points.PushBack(point);
        for (Int32_t i = 1; i < sample_size; ++i) {
          const common::PathPoint& prev_point = sample_points.Back();
          point.point = prev_point.point +
              sample_step_len  * object->obb.unit_direction_x();
          point.heading = object->heading;
          point.s = prev_point.s + sample_step_len;
          sample_points.PushBack(point);
        }
        common::StaticVector<common::TrajectoryPoint,
            MAX_OBJ_PRED_TRAJ_POINT_NUM>* pred_path =
            object->pred_trajectory.Allocate();
        if (Nullptr_t == pred_path) {
          LOG_ERR << "Can't store predicted path any more, storage is full.";
          return;
        }

        points_count = sample_points.Size();
        for (Int32_t i = 0; i < points_count; ++i) {
          common::TrajectoryPoint* point = pred_path->Allocate();
          if (Nullptr_t == point) {
            LOG_ERR << "Can't store point of predicted path, storage is full.";
            break;
          }
          point->path_point = sample_points[i];
          point->v = object->v;
          if (point->v < 0.1F) {
            point->v = 0.1F;
          }
          point->a = 0.0F;
          point->yaw_rate = 0.0F;
          point->relative_time = common::com_abs(point->path_point.s) / point->v;
        }
      }
    }
#endif
    }
}

// 20230425
void DrivingMapImpl::FindObsatclePredPathByMap(
    bool find_forward, const Int32_t nearest_lane_index,
    const common::PathPoint& proj_point, const Int32_t sample_size, const map_var_t sample_step_len,
    common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>* sample_points,
     Object* object) {
  HDMap::SearchConnectedLanesParam search_param;
  search_param.search_len = sample_step_len * sample_size;
  search_param.search_forward = find_forward;

  for (Int32_t i = 0; i < temp_data_.search_lane_list.Size(); ++i) {
    temp_data_.search_lane_list[i].Clear();
  }
  temp_data_.search_lane_list.Clear();

  map_space_.SearchConnectedLanes(
        search_param, nearest_lane_index,
        proj_point, &temp_data_.search_lane_list);

  Int32_t lane_list_size = temp_data_.search_lane_list.Size();
  Int32_t min_lane_list_index[2] = { -1, -1 };
  map_var_t min_lane_heading[2] = { 99.9F, 99.9F };

  for(Int32_t i = 0; i < lane_list_size; ++i) {
    const Int32_t last_seg_lane_idex =
        temp_data_.search_lane_list[i].Back().lane_index;
    const Lane& last_seg_lane = map_space_.GetLane(last_seg_lane_idex);

    common::PathPoint end_point;
    last_seg_lane.central_curve().
        FindSmoothPoint(temp_data_.search_lane_list[i].Back().end_s, &end_point);
    map_var_t heading = common::GetHeadingFromSegment(
          proj_point.point, end_point.point);
    if (0 == i) {
      min_lane_heading[0] = heading;
      min_lane_list_index[0] = i;
    } else {
      if (common::com_abs(heading) <= common::com_abs(min_lane_heading[0])) {
        min_lane_heading[1] = min_lane_heading[0];
        min_lane_heading[0] = heading;
        min_lane_list_index[1] = min_lane_list_index[0];
        min_lane_list_index[0] = i;
      } else {
        if (common::com_abs(heading) < common::com_abs(min_lane_heading[1])) {
          min_lane_heading[1] = heading;
          min_lane_list_index[1] = i;
        }
      }
    }
  }
  for (Int32_t i = 0; i < 2; ++i) {
    Int32_t lane_list_index = min_lane_list_index[i];
    if (lane_list_index < 0) {
      continue;
    }
    const common::StaticVector<HDMap::LaneSeg, MAX_LANE_SEGMENT_NUM>&
        lane_segs = temp_data_.search_lane_list[lane_list_index];
    common::Path& central_line = temp_data_.path_list[0];
    central_line.Clear();
    sample_points->Clear();
    for (Int32_t j = 0; j < lane_segs.Size(); ++j) {
      Int32_t index = j;
      if (!search_param.search_forward) {
        index = lane_segs.Size() - j - 1;
      }

      const HDMap::LaneSeg& curr_lane_seg = lane_segs[index];
      const Lane& curr_lane = map_space_.GetLane(curr_lane_seg.lane_index);

      map_var_t sample_s = curr_lane_seg.end_s - curr_lane_seg.start_s ;
      if (sample_s < phoenix::common::kMathEpsilonF) {
        common::PathPoint path_point;
        curr_lane.central_curve().FindSmoothPoint(curr_lane_seg.start_s, &path_point);
        sample_points->PushBack(path_point);
      } else {
        curr_lane.central_curve().
            GetSamplePoints(curr_lane_seg.start_s, curr_lane_seg.end_s, sample_points);
        if ((j < lane_segs.Size() - 1) && (lane_segs.Size() > 1)) {
          sample_points->PopBack();
        }
      }
    }
    central_line.Construct(*sample_points);
    sample_points->Clear();
    Int32_t obj_pred_trajectory_size =
        common::com_round(central_line.total_length() / sample_step_len);
    if (search_param.search_forward) {
      central_line.UniformlySamplePathForward(
            0, obj_pred_trajectory_size,
            sample_step_len, proj_point.l, sample_points);
    } else {
      central_line.UniformlySamplePathBackward(
            central_line.total_length(), obj_pred_trajectory_size,
            sample_step_len, proj_point.l, sample_points);
      }
      common::StaticVector<common::TrajectoryPoint,
          MAX_OBJ_PRED_TRAJ_POINT_NUM>* pred_path =
          object->pred_trajectory.Allocate();
      if (Nullptr_t == pred_path) {
        LOG_ERR << "Can't store predicted path any more, storage is full.";
        break;
      }
      for (Int32_t i = 0; i < sample_points->Size(); ++i) {
        common::TrajectoryPoint* point = pred_path->Allocate();
        if (Nullptr_t == point) {
          LOG_ERR << "Can't store point of predicted path, storage is full.";
          break;
        }
        point->path_point = sample_points->data()[i];
        if (!find_forward) {
          point->path_point.heading = common::NormalizeAngle(
                sample_points->data()[i].heading +
                static_cast<phoenix::common::geo_var_t>(COM_PI));
        }
        point->path_point.s = sample_points->data()[i].s - sample_points->Front().s;
        point->v = object->v;
        if (point->v < 0.1F) {
          point->v = 0.1F;
        }
        point->a = 0.0F;
        point->yaw_rate = 0.0F;
        point->relative_time = common::com_abs(point->path_point.s) / point->v;
      }
    }
}

void DrivingMapImpl::CreateVehOBB(
    const common::Vec2d& pos, map_var_t heading,
    common::OBBox2d* obb) const {
  const map_var_t half_veh_width = param_.vehicle_width * 0.5F;
  const map_var_t half_veh_length = param_.vehicle_length * 0.5F;

  if (DRIVING_DIRECTION_BACKWARD == driving_direction_) {
    // For backing mode of vehicle
    heading = common::NormalizeAngle(heading + COM_PI);
  }

  obb->set_unit_direction(common::com_cos(heading), common::com_sin(heading));
  obb->set_center(pos + param_.dist_of_localization_to_center *
                  obb->unit_direction_x());
  obb->set_extents(half_veh_length, half_veh_width);
}

bool DrivingMapImpl::CreateRefLineAssociation() {
  Int32_t ref_line_num = reference_line_set_.GetReferenceLinesNum();
  if (ref_line_num <= 0) {
    LOG_ERR << "There is no reference line.";
    return false;
  }

  ref_line_association_list_.Resize(static_cast<Int32_t>(MAX_LANE_NUM));
  for (Int32_t i = 0; i < static_cast<Int32_t>(MAX_LANE_NUM); ++i) {
    ref_line_association_list_[i].Clear();
  }

  for (Int32_t i = 0; i < ref_line_num; ++i) {
    const common::StaticVector<LaneSegment,
      static_cast<Int32_t>(MAX_LANE_SEGMENT_NUM)>& lane_segs =
      reference_line_set_.GetLaneSegments(i);
    Int32_t lane_segs_size = lane_segs.Size();
    for (Int32_t j = 0; j < lane_segs_size; ++j) {
      const LaneSegment& seg = lane_segs[j];
      RefLineAssociation& ass = ref_line_association_list_[seg.lane_index];
      if (ass.lane_index < 0) {
        ass.lane_index = seg.lane_index;
        ass.start_s_on_lane = seg.start_s;
        ass.end_s_on_lane = seg.end_s;
        ass.start_s_on_ref = seg.start_s_on_ref;
      } else {
        if (seg.start_s < ass.start_s_on_lane) {
          ass.start_s_on_lane = seg.start_s;
        }
        if (seg.end_s > ass.end_s_on_lane) {
          ass.end_s_on_lane = seg.end_s;
        }
        if (seg.start_s_on_ref < ass.start_s_on_ref) {
          ass.start_s_on_ref = seg.start_s_on_ref;
        }
      }
    }
  }

#if ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST
  /// Performance of analyzing risky area (Start)
  phoenix::common::Stopwatch performance_timer_analyzing_risky_area;
#endif

  for (Int32_t i = 0; i < MAX_LANE_NUM; ++i) {
    RefLineAssociation& ass = ref_line_association_list_[i];
    if (ass.lane_index < 0) {
      continue;
    }

    // Test collision on this lane segment.
    TestCollisionOnLaneSegment(&ass);
  }

#if ENABLE_DRIVING_MAP_IMPL_PERFORMANCE_TEST
  /// Performance of analyzing risky area (End)
  std::cout << "Analyzing risky area spend "
            << performance_timer_analyzing_risky_area.Elapsed()
            << "ms." << std::endl;
#endif

  return true;
}

void DrivingMapImpl::TestCollisionOnLaneSegment(RefLineAssociation* ass) {
  map_var_t sample_step_len = param_.vehicle_length;
  map_var_t start_s_ref = ass->start_s_on_ref;
  map_var_t sample_len = ass->end_s_on_lane - ass->start_s_on_lane -
      0.5F * param_.vehicle_length;
  if (sample_len < 0.5F) {
    sample_len = 0.5F;
  }
  Int32_t sample_num = common::com_floor(sample_len / sample_step_len) + 1;
  sample_step_len = sample_len / static_cast<map_var_t>(sample_num);

  const common::Path& path = map_space_.GetLane(ass->lane_index).central_curve();

  temp_data_.sample_points.Clear();

  if (!path.UniformlySamplePathForward(
        ass->start_s_on_lane, sample_num, sample_step_len,
        &temp_data_.sample_points)) {
    LOG_ERR << "Failed to sample path for testing collision.";
    return;
  }

  Int32_t sample_size = temp_data_.sample_points.Size();
  if (sample_size < 1) {
    LOG_ERR << "Failed to sample path for testing collision.";
    return;
  }

  CollisionTestObj test_obj;
  test_obj.obb_circumradius = param_.vehicle_circumradius;
  test_obj.v = common::com_abs(current_rel_position_.v);
  if (test_obj.v < 20.0F/3.6F) {
    test_obj.v = 20.0F/3.6F;
  }
  CollisionTestResult coll_test_ret;

  ass->risky_obj_list_valid = true;
  ass->risk_value = 0;
  ass->risky_obj_list.Clear();
  ass->uncertain_list.Clear();

  CollisionTestParam test_param;
  test_param.SetTestingIgnoredObj(true);
  test_param.SetReturnSingleRiskyObj(false);

  for (Int32_t i = 0; i < sample_size; ++i) {
    const common::PathPoint& path_point = temp_data_.sample_points[i];
    CreateVehOBB(path_point.point, path_point.heading, &test_obj.obb);
    test_obj.s = path_point.s - ass->start_s_on_lane + start_s_ref;
    test_obj.t = test_obj.s / test_obj.v;

    coll_test_ret.Clear();
    Int32_t risk_value = obstacle_map_.TestCollision(
          test_param, test_obj, &coll_test_ret);

    Int32_t risky_obj_list_size = coll_test_ret.risky_obj_list.Size();
    for (Int32_t j = 0; j < risky_obj_list_size; ++j) {
      AddRiskyObjToList(coll_test_ret.risky_obj_list[j], &(ass->risky_obj_list));
    }
    Int32_t uncertain_list_size = coll_test_ret.uncertain_list.Size();
    for (Int32_t j = 0; j < uncertain_list_size; ++j) {
      const driv_map::CollisionTestResult::ObjInfo& ret =
          coll_test_ret.uncertain_list[j];
      const ad_msg::Obstacle& obj = GetObstacle(ret.obj_list_index);
      if ((ret.static_distance < 0.1F) &&
          (obj.type != ad_msg::OBJ_TYPE_CURB)) {
        AddRiskyObjToList(
              coll_test_ret.uncertain_list[j], &(ass->uncertain_list));
      }
    }

    if (risk_value > ass->risk_value) {
      ass->risk_value = risk_value;
    }
  }
}

void DrivingMapImpl::CalcRoadBoundary(
    common::StaticVector<RoadBoundary,
    common::Path::kMaxPathPointNum>* const boundary_info) {
  map_var_t step_len = 2.0f;

  boundary_info->Clear();

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
      sample_points =  temp_data_.sample_points;
  sample_points.Clear();

  Int32_t curr_ref_line_index = GetMajorRefLineIndex();
  if (!IsValidReferenceLineIndex(curr_ref_line_index)) {
    LOG_ERR << "Invalid reference line.";
    return;
  }

  const common::Path& curr_ref_line = GetReferenceLineCurve(curr_ref_line_index);
  if (curr_ref_line.total_length() < 1.0f) {
    LOG_ERR << "The length of reference line is too short.";
    return;
  }
  Int32_t sample_size = phoenix::common::com_round(
        curr_ref_line.total_length() / step_len);
  if (sample_size < 1) {
    sample_size = 1;
  }
  step_len = curr_ref_line.total_length() / sample_size;

  if (false == curr_ref_line.UniformlySamplePathForward(
        0, sample_size, step_len, &sample_points)) {
    LOG_ERR << "Failed to sample reference line.";
    return;
  }
  Int32_t sample_points_size = sample_points.Size();
  if (sample_points_size < 2) {
    LOG_ERR << "The number of sample points of reference line is too small.";
    return;
  }

  Int32_t nearest_neighbor_index = 0;
  common::StaticVector<NeighborsLaneInfo,
      MAX_NEIGHBOR_LANE_NUM> sorted_neighbor_lanes;

  for (Int32_t i = 0; i < sample_points_size; ++i) {
    const common::PathPoint& sample_point_on_ref = sample_points[i];
    Int32_t lane_index = 0;
    FindReferenceLineLaneSegmentByProjOnRef(
          curr_ref_line_index, sample_point_on_ref.s, &lane_index);
    if (!FindNeighbors(
          sample_point_on_ref.point,
          lane_index,
          &sorted_neighbor_lanes,
          &nearest_neighbor_index)) {
      LOG_ERR << "Failed to find neighbor lanes.";
      break;
    }

    Int32_t sorted_neighbor_lanes_num = sorted_neighbor_lanes.Size();
    Int32_t left_boundary_neighbor_index = nearest_neighbor_index;
    Int32_t right_boundary_neighbor_index = nearest_neighbor_index;
    for (Int32_t j = (nearest_neighbor_index+1);
         j < sorted_neighbor_lanes_num; ++j) {
      if (sorted_neighbor_lanes[j].valid_lat) {
        left_boundary_neighbor_index = j;
      } else {
        break;
      }
    }
    for (Int32_t j = (nearest_neighbor_index-1); j >= 0; --j) {
      if (sorted_neighbor_lanes[j].valid_lat) {
        right_boundary_neighbor_index = j;
      } else {
        break;
      }
    }

    const NeighborsLaneInfo& left_boundary_lane =
        sorted_neighbor_lanes[left_boundary_neighbor_index];
    const NeighborsLaneInfo& right_boundary_lane =
        sorted_neighbor_lanes[right_boundary_neighbor_index];

    map_var_t left_width = 0;
    map_var_t right_width = 0;

    RoadBoundary* prev_boundary_sample = Nullptr_t;
    if (!boundary_info->Empty()) {
      prev_boundary_sample = &(boundary_info->Back());
    }
    RoadBoundary* boundary_sample = boundary_info->Allocate();
    if (Nullptr_t == boundary_sample) {
      LOG_ERR << "Can't add point to boundary, storage is full.";
      break;
    }
    boundary_sample->ref_point = sample_point_on_ref;

    GetLaneWidth(
          left_boundary_lane.lane_index, left_boundary_lane.proj_point.s,
          &left_width, &right_width);
    boundary_sample->left_boundary_point.point.set_x(
          left_boundary_lane.proj_point.point.x() - left_width *
          common::com_sin(left_boundary_lane.proj_point.heading));
    boundary_sample->left_boundary_point.point.set_y(
          left_boundary_lane.proj_point.point.y() + left_width *
          common::com_cos(left_boundary_lane.proj_point.heading));
    boundary_sample->left_boundary_point.heading =
        left_boundary_lane.proj_point.heading;
    boundary_sample->left_boundary_point.curvature =
        left_boundary_lane.proj_point.curvature;
    boundary_sample->left_boundary_point.l =
        -left_boundary_lane.proj_point.l + left_width;
    boundary_sample->left_boundary_point.s = sample_point_on_ref.s;
    boundary_sample->left_width =
        -left_boundary_lane.proj_point.l + left_width;
    if (Nullptr_t != prev_boundary_sample) {
      map_var_t boundary_heading =
          common::GetHeadingFromSegment(
            prev_boundary_sample->left_boundary_point.point,
            boundary_sample->left_boundary_point.point);
      map_var_t heading_diff =
          common::AngleDiff(
            boundary_heading, left_boundary_lane.proj_point.heading);
      if (common::com_abs(heading_diff) > common::com_deg2rad(70.0F)) {
        boundary_sample->left_boundary_point.point =
            prev_boundary_sample->left_boundary_point.point;
      }
    }

    GetLaneWidth(
          right_boundary_lane.lane_index, right_boundary_lane.proj_point.s,
          &left_width, &right_width);
    boundary_sample->right_boundary_point.point.set_x(
          right_boundary_lane.proj_point.point.x() + right_width *
          common::com_sin(right_boundary_lane.proj_point.heading));
    boundary_sample->right_boundary_point.point.set_y(
          right_boundary_lane.proj_point.point.y() - right_width *
          common::com_cos(right_boundary_lane.proj_point.heading));
    boundary_sample->right_boundary_point.heading =
        right_boundary_lane.proj_point.heading;
    boundary_sample->right_boundary_point.curvature =
        right_boundary_lane.proj_point.curvature;
    boundary_sample->right_boundary_point.l =
        -right_boundary_lane.proj_point.l - right_width;
    boundary_sample->right_boundary_point.s = sample_point_on_ref.s;
    boundary_sample->right_width =
        right_boundary_lane.proj_point.l + right_width;
    if (Nullptr_t != prev_boundary_sample) {
      map_var_t boundary_heading =
          common::GetHeadingFromSegment(
            prev_boundary_sample->right_boundary_point.point,
            boundary_sample->right_boundary_point.point);
      map_var_t heading_diff =
          common::AngleDiff(
            boundary_heading, right_boundary_lane.proj_point.heading);
      if (common::com_abs(heading_diff) > common::com_deg2rad(60.0F)) {
        boundary_sample->right_boundary_point.point =
            prev_boundary_sample->right_boundary_point.point;
      }
    }
  }

#if 0
  Int32_t boundary_point_num = boundary_info->Size();
  std::cout << "The left road boundary is:" << std::endl;
  for (Int32_t i = 0; i < boundary_point_num; ++i) {
    const RoadBoundary& sample = boundary_info->GetData(i);
    const common::PathPoint& left_point = sample.left_boundary_point;
    std::cout << "    [" << i
              << "], point[" << left_point.point.x()
              << ", " << left_point.point.y()
              << ", heading=" << left_point.heading
              << ", l=" << left_point.l
              << ", s=" << left_point.s
              << std::endl;
  }
  std::cout << "The right road boundary is:" << std::endl;
  for (Int32_t i = 0; i < boundary_point_num; ++i) {
    const RoadBoundary& sample = boundary_info->GetData(i);
    const common::PathPoint& right_point = sample.right_boundary_point;
    std::cout << "    [" << i
              << "], point[" << right_point.point.x()
              << ", " << right_point.point.y()
              << ", heading=" << right_point.heading
              << ", l=" << right_point.l
              << ", s=" << right_point.s
              << std::endl;
  }
#endif
}

/* k005 pengc 2023-01-06 (begin) */
// 添加功能: 道路边界碰撞测试
#if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
void DrivingMapImpl::ConstructRoadBoundaryMap(
    const common::StaticVector<
    RoadBoundary, common::Path::kMaxPathPointNum>& boundary) {
  road_boundary_map_.Clear();

  Int32_t boundary_point_num = boundary.Size();
  if (boundary_point_num < 2) {
    return;
  }

  Int32_t major_ref_line_idx = GetMajorRefLineIndex();
  if (!IsValidReferenceLineIndex(major_ref_line_idx)) {
    LOG_ERR << "Invalid reference line.";
    return;
  }
  const common::PathPoint& veh_proj_on_major_ref = GetProjPointOnMajorRefLine();

  Object obj;
  obj.height = 0.1F;
  obj.height_to_ground = 0.0F;
  obj.type = ad_msg::OBJ_TYPE_UNKNOWN;
  obj.dynamic = false;
  obj.confidence = 100;
  obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
  obj.ignore = false;
  obj.uncertain = false;
  obj.v_x = 0.0F;
  obj.v_y = 0.0F;
  obj.v = 0.0F;
  obj.a_x = 0.0F;
  obj.a_y = 0.0F;
  obj.a = 0.0F;
  obj.yaw_rate = 0.0F;

  bool valid_prev_boundary_point = false;
  common::Vec2d prev_left_boundary_point;
  common::Vec2d prev_right_boundary_point;
  Int32_t left_boundary_obj_num = 0;
  Int32_t right_boundary_obj_num = 0;

  for (Int32_t i = 0; i < boundary_point_num; ++i) {
    const RoadBoundary& sample = boundary[i];
    const common::PathPoint& left_point = sample.left_boundary_point;
    const common::PathPoint& right_point = sample.right_boundary_point;

    // 只对前向的道路边界进行碰撞分析
    if (sample.ref_point.s < veh_proj_on_major_ref.s) {
      continue;
    }

    if (!valid_prev_boundary_point) {
      valid_prev_boundary_point = true;
      prev_left_boundary_point = left_point.point;
      prev_right_boundary_point = right_point.point;

      continue;
    }

    map_var_t distance = left_point.point.DistanceTo(prev_left_boundary_point);
    if ((distance > 1.0F) &&
        (left_boundary_obj_num < (MAX_OBJECT_NUM_IN_OBJ_SPACE/2))) {
      obj.id = i + 1;
      obj.pos = prev_left_boundary_point;
      obj.obb.set_center(
            0.5F*(prev_left_boundary_point.x()+left_point.point.x()),
            0.5F*(prev_left_boundary_point.y()+left_point.point.y()));
      obj.obb.set_unit_direction(
            (left_point.point.x()-prev_left_boundary_point.x()) / distance,
            (left_point.point.y()-prev_left_boundary_point.y()) / distance);
      obj.obb.set_extents(0.5F*distance, 0.05F);
      obj.heading = GetHeadingFromSegment(
            prev_left_boundary_point, left_point.point);

      if (!road_boundary_map_.AddObject(obj)) {
        LOG_ERR << "Failed to add object [" << i << "] to road boundary map.";
        break;
      }

      prev_left_boundary_point = left_point.point;
      left_boundary_obj_num++;
    }

    distance = right_point.point.DistanceTo(prev_right_boundary_point);
    if ((distance > 1.0F) &&
        (right_boundary_obj_num < (MAX_OBJECT_NUM_IN_OBJ_SPACE/2))) {
      obj.id = i + 1 + MAX_OBJECT_NUM_IN_OBJ_SPACE;
      obj.pos = prev_right_boundary_point;
      obj.obb.set_center(
            0.5F*(prev_right_boundary_point.x()+right_point.point.x()),
            0.5F*(prev_right_boundary_point.y()+right_point.point.y()));
      obj.obb.set_unit_direction(
            (right_point.point.x()-prev_right_boundary_point.x()) / distance,
            (right_point.point.y()-prev_right_boundary_point.y()) / distance);
      obj.obb.set_extents(0.5F*distance, 0.05F);
      obj.heading = GetHeadingFromSegment(
            prev_right_boundary_point, right_point.point);

      if (!road_boundary_map_.AddObject(obj)) {
        LOG_ERR << "Failed to add object [" << i << "] to road boundary map.";
        break;
      }

      prev_right_boundary_point = right_point.point;
      right_boundary_obj_num++;
    }
  }
  road_boundary_map_.CompleteAddingObject();
}
#endif
/* k005 pengc 2023-01-06 (end) */

void DrivingMapImpl::SetMapAttributeFromRawMap() {
  /// TODO: 目前仅设置了当前道路参考线上的地图属性

  // LOG_INFO(5) << "### SetMapAttributeFromRawMap (begin)";

  static const map_var_t kSampleStep = 10.0F;

  Int32_t raw_major_ref_idx = reference_line_set_raw_map_.GetMajorRefLineIndex();
  if (!reference_line_set_raw_map_.IsValidReferenceLineIndex(raw_major_ref_idx)) {
    LOG_ERR << "Invalid reference line index " << raw_major_ref_idx;
    return;
  }
  const common::Path& raw_major_ref =
      reference_line_set_raw_map_.GetSmoothCurve(raw_major_ref_idx);
  const common::PathPoint& raw_start_point_on_ref =
      reference_line_set_raw_map_.GetProjPointOnMajorRefLine();

  Int32_t major_ref_idx = reference_line_set_.GetMajorRefLineIndex();
  if (!reference_line_set_.IsValidReferenceLineIndex(major_ref_idx)) {
    LOG_ERR << "Invalid reference line index " << major_ref_idx;
    return;
  }
  const common::Path& major_ref =
      reference_line_set_.GetSmoothCurve(major_ref_idx);
  const common::PathPoint& start_point_on_ref =
      reference_line_set_.GetProjPointOnMajorRefLine();

  map_var_t raw_start_s = 0.0F;
  map_var_t start_s = 0.0F;
  if (raw_start_point_on_ref.s > start_point_on_ref.s) {
    raw_start_s = raw_start_point_on_ref.s - start_point_on_ref.s;
  } else {
    start_s = start_point_on_ref.s - raw_start_point_on_ref.s;
  }
  map_var_t raw_end_s = raw_major_ref.total_length();
  if ((raw_major_ref.total_length() - raw_start_point_on_ref.s) >
      (major_ref.total_length() - start_point_on_ref.s)) {
    raw_end_s = raw_start_s + (major_ref.total_length() - start_point_on_ref.s);
  }
  map_var_t sample_len = raw_end_s - raw_start_s;
  Int32_t sample_num = common::com_floor(sample_len / kSampleStep);

  Int32_t prev_lane_idx = -1;
  for (Int32_t i = 0; i < sample_num; ++i) {
    map_var_t s = i*kSampleStep;

    Int32_t raw_lane_idx = -1;
    map_var_t s_on_raw_lane = 0.0F;
    reference_line_set_raw_map_.FindLaneSegmentByProjOnRef(
          raw_major_ref_idx, raw_start_s + s, &raw_lane_idx, &s_on_raw_lane);
    if (!map_space_raw_map_.IsValidLaneIndex(raw_lane_idx)) {
      LOG_ERR << "Invalid lane index.";
      continue;
    }
    Lane& raw_lane = map_space_raw_map_.GetLane(raw_lane_idx);

    Int32_t lane_idx = -1;
    map_var_t s_on_lane = 0.0F;
    reference_line_set_.FindLaneSegmentByProjOnRef(
          major_ref_idx, start_s + s, &lane_idx, &s_on_lane);
    if (!map_space_.IsValidLaneIndex(lane_idx)) {
      LOG_ERR << "Invalid lane index.";
      continue;
    }
    Lane& lane = map_space_.GetLane(lane_idx);

    // LOG_INFO(5) << ">>> Set attribute of lane(" << lane.topology_id().id.id
    //             << ") to lane(" << raw_lane.topology_id().id.id << ").";

    // 设置限速
    lane.set_speed_limit_high(raw_lane.speed_limit_high());
    lane.set_speed_limit_low(raw_lane.speed_limit_low());

    // LOG_INFO(5) << "    Set speed limit (high) to " << raw_lane.speed_limit_high();
    // LOG_INFO(5) << "    Set speed limit (low) to " << raw_lane.speed_limit_low();

    // 设置坡度
    if (prev_lane_idx != lane_idx) {
      lane.slope_samples().Clear();
    }
    if (s_on_lane > -0.1F) {
      map_var_t slope = raw_lane.GetLaneSlope(s_on_raw_lane);
      if (lane.slope_samples().Empty() && (s_on_lane > 0.1F)) {
        Lane::SlopeAssociation* lane_slope = lane.slope_samples().Allocate();
        if (Nullptr_t == lane_slope) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
        } else {
          lane_slope->s = 0;
          lane_slope->slope = slope;

          // LOG_INFO(5) << "      Set slope to " << slope << " on s(" << lane_slope->s << ").";
        }
      }

      Lane::SlopeAssociation* lane_slope = lane.slope_samples().Allocate();
      if (Nullptr_t == lane_slope) {
        LOG_WARN << "Can't save slope information anymore, storage is full.";
      } else {
        lane_slope->s = s_on_lane;
        lane_slope->slope = slope;

        // LOG_INFO(5) << "      Set slope to " << slope << " on s(" << lane_slope->s << ").";
      }
    }

#if 0
    /// TODO: 设置边线类型, 暂时每个车道设置为一样的类型
    if (prev_lane_idx != lane_idx) {
      Int32_t left_type = LANE_BOUNDARY_TYPE_UNKNOWN;
      Int32_t right_type = LANE_BOUNDARY_TYPE_UNKNOWN;
      raw_lane.GetBoundaryType(s_on_raw_lane, &left_type, &right_type);

      // LOG_INFO(5) << "    Set boundary type to left(" << left_type
      //             << ") right(" << right_type << ").";

      for (Int32_t i = 0; i < lane.left_boundary().boundary_samples.Size(); ++i) {
        lane.left_boundary().boundary_samples[i].type = left_type;
      }
      for (Int32_t i = 0; i < lane.right_boundary().boundary_samples.Size(); ++i) {
        lane.right_boundary().boundary_samples[i].type = right_type;
      }
    }
#endif

    // 保存之前的车道索引
    prev_lane_idx = lane_idx;
  }
}

void DrivingMapImpl::SetMapSlopAttributeFromPccMap() {
  /// TODO: 目前仅设置了当前道路参考线上的地图属性

  static const map_var_t kSampleStep = 10.0F;

  Int32_t raw_major_ref_idx = reference_line_set_pcc_map_.GetMajorRefLineIndex();
  if (!reference_line_set_pcc_map_.IsValidReferenceLineIndex(raw_major_ref_idx)) {
    LOG_ERR << "Invalid reference line index " << raw_major_ref_idx;
    return;
  }
  const common::Path& raw_major_ref =
      reference_line_set_pcc_map_.GetSmoothCurve(raw_major_ref_idx);
  const common::PathPoint& raw_start_point_on_ref =
      reference_line_set_pcc_map_.GetProjPointOnMajorRefLine();

  Int32_t major_ref_idx = reference_line_set_.GetMajorRefLineIndex();
  if (!reference_line_set_.IsValidReferenceLineIndex(major_ref_idx)) {
    LOG_ERR << "Invalid reference line index " << major_ref_idx;
    return;
  }
  const common::Path& major_ref =
      reference_line_set_.GetSmoothCurve(major_ref_idx);
  const common::PathPoint& start_point_on_ref =
      reference_line_set_.GetProjPointOnMajorRefLine();

  map_var_t raw_start_s = 0.0F;
  map_var_t start_s = 0.0F;
  if (raw_start_point_on_ref.s > start_point_on_ref.s) {
    raw_start_s = raw_start_point_on_ref.s - start_point_on_ref.s;
  } else {
    start_s = start_point_on_ref.s - raw_start_point_on_ref.s;
  }
  map_var_t raw_end_s = raw_major_ref.total_length();
  if ((raw_major_ref.total_length() - raw_start_point_on_ref.s) >
      (major_ref.total_length() - start_point_on_ref.s)) {
    raw_end_s = raw_start_s + (major_ref.total_length() - start_point_on_ref.s);
  }
  map_var_t sample_len = raw_end_s - raw_start_s;
  Int32_t sample_num = common::com_floor(sample_len / kSampleStep);

  Int32_t prev_lane_idx = -1;
  for (Int32_t i = 0; i < sample_num; ++i) {
    map_var_t s = i*kSampleStep;

    Int32_t raw_lane_idx = -1;
    map_var_t s_on_raw_lane = 0.0F;
    reference_line_set_pcc_map_.FindLaneSegmentByProjOnRef(
          raw_major_ref_idx, raw_start_s + s, &raw_lane_idx, &s_on_raw_lane);
    if (!map_space_pcc_map_.IsValidLaneIndex(raw_lane_idx)) {
      LOG_ERR << "Invalid lane index.";
      continue;
    }
    Lane& raw_lane = map_space_pcc_map_.GetLane(raw_lane_idx);

    Int32_t lane_idx = -1;
    map_var_t s_on_lane = 0.0F;
    reference_line_set_.FindLaneSegmentByProjOnRef(
          major_ref_idx, start_s + s, &lane_idx, &s_on_lane);
    if (!map_space_.IsValidLaneIndex(lane_idx)) {
      LOG_ERR << "Invalid lane index.";
      continue;
    }
    Lane& lane = map_space_.GetLane(lane_idx);

    // LOG_INFO(5) << ">>> Set attribute of lane(" << lane.topology_id().id.id
    //             << ") to lane(" << raw_lane.topology_id().id.id << ").";

    // 设置限速
  //  lane.set_speed_limit_high(raw_lane.speed_limit_high());
  //  lane.set_speed_limit_low(raw_lane.speed_limit_low());

    // LOG_INFO(5) << "    Set speed limit (high) to " << raw_lane.speed_limit_high();
    // LOG_INFO(5) << "    Set speed limit (low) to " << raw_lane.speed_limit_low();

    // 设置坡度
    if (prev_lane_idx != lane_idx) {
      lane.slope_samples().Clear();
    }
    if (s_on_lane > -0.1F) {
      map_var_t slope = raw_lane.GetLaneSlope(s_on_raw_lane);
      if (lane.slope_samples().Empty() && (s_on_lane > 0.1F)) {
        Lane::SlopeAssociation* lane_slope = lane.slope_samples().Allocate();
        if (Nullptr_t == lane_slope) {
          LOG_WARN << "Can't save slope information anymore, storage is full.";
        } else {
          lane_slope->s = 0;
          lane_slope->slope = slope;

          // LOG_INFO(5) << "      Set slope to " << slope << " on s(" << lane_slope->s << ").";
        }
      }

      Lane::SlopeAssociation* lane_slope = lane.slope_samples().Allocate();
      if (Nullptr_t == lane_slope) {
        LOG_WARN << "Can't save slope information anymore, storage is full.";
      } else {
        lane_slope->s = s_on_lane;
        lane_slope->slope = slope;

        // LOG_INFO(5) << "      Set slope to " << slope << " on s(" << lane_slope->s << ").";
      }
    }
    // 保存之前的车道索引
    prev_lane_idx = lane_idx;
  }
}
void DrivingMapImpl::AddEventReporting(Int32_t event_type) {
  ad_msg::EventReporting event;
  event.msg_head.valid = false;

  switch (event_type) {
  case (EVENT_TYPE_CAMERA_LANE_LOST):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_CAMERA_LANE_LOST;
    event.priority = 1;
    if (DRIVING_MAP_TYPE_FOLLOWING_PATH == driving_map_type_) {
      event.lifetime = 6*1000;
    } else {
      event.lifetime = 2*1000;
    }
    break;
  case (EVENT_TYPE_MAP_IS_NOT_VAILD):
    event.msg_head.valid = true;
    event.event_id = ad_msg::EVENT_REPORTING_ID_MAP_NOT_VAILD;
    event.priority = 1;
    event.lifetime = 2*1000;
    break;
  default:
    break;
  }

  if (event.msg_head.valid) {
    event_reporting_list_.PushBack(event);
  }
}


}  // namespace driv_map
}  // namespace phoenix


