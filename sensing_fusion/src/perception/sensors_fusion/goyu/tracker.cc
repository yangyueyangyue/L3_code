#include "vehicle_model_wrapper.h"
#include "utils/linear_interpolation.h"
#include "container/data_pair.h"
#include "sensors_fusion/sensors_fusion_macros.h"
#include "sensors_fusion/sensors_fusion_param_macros.h"
#include "pos_filter/include/pos_filter_wrapper.h"
#include "tracker.h"

#define  ENABLE_OBJ_TRACKER_TRACE    0

#define  ENABLE_CAM_DATA_TRACE   0

#define  ENABLE_OTHER_DEBUG_PRINTF     0


namespace phoenix{
namespace perception{
namespace sensorsfusion{

void ObjsTracker::Clear()
{
    tracked_objects_.Clear(obj_ass_pool_);
    tracked_objects_.msg_head_.valid = false;
    tracked_objects_kdtree_.Clear();
}

void ObjsTracker::SetRelativePosList(const ad_msg::RelativePosList &pos_list)
{
    rel_pos_list_ = pos_list;
}

void ObjsTracker::SetCurrentRelPos(const ad_msg::RelativePos &pos)
{
    current_rel_pos_ = pos;
}

ObjIndex ObjsTracker::AddNewObj(const Int32_t sensor_id, const ObjInfosBase &src_obj)
{
    ObjIndex tracked_obj_index(-1, -1);

    ObjsTracked* new_obj =
        tracked_objects_.objects_.Allocate(&(tracked_obj_index.obj_idx_));
    if (Nullptr_t == new_obj) {
      LOG_ERR << "[Error]Failed to add tracked obj, storage is full.";
    } else {
      new_obj->erase_flag_ = false;
      tracked_obj_index.sensor_idx_ = sensor_id;
      new_obj->list_id_ = GetListIdBySensorId(sensor_id);
      COM_CHECK(new_obj->list_id_ >= 0);
      new_obj->raw_obj_idx_ = src_obj.raw_obj_idx_;

      new_obj->track_status_ = TRACK_STATUS_NEW;
      new_obj->life_ = MAX_TRACKED_OBJ_LIFE;
      new_obj->matched_ = 0;
      new_obj->age_ = 0;
      new_obj->duration_ = 0;
      new_obj->confidence_ = 0;
      new_obj->perception_type = 0;
      new_obj->last_perception_type = 0;
      new_obj->fusion_id = 0;


      SensorsRef& sensor_ref =
          new_obj->sensor_ref_[tracked_obj_index.sensor_idx_];

      sensor_ref.valid_ = true;
      sensor_ref.sensor_type_ = src_obj.sensor_type_;
      sensor_ref.obj_id_[0].id_ = src_obj.raw_obj_id_;
      sensor_ref.obj_id_[0].age_ = 0;

      sensor_ref.obj_type_ = src_obj.obj_type_;
      sensor_ref.width_ = src_obj.width_;
      sensor_ref.height_ = src_obj.height_;
      sensor_ref.length_ = src_obj.length_;

      for (Int32_t j = 0; j < SensorsRef::MAX_POS_POINT_NUM; ++j) {
        sensor_ref.pos_[j].x_ = src_obj.pos_.x_;
        sensor_ref.pos_[j].y_ = src_obj.pos_.y_;
        sensor_ref.pos_[j].range_ = src_obj.pos_.range_;
        sensor_ref.pos_[j].angle_ = src_obj.pos_.angle_;
        //TODO:添加heading信息，待验证
        sensor_ref.pos_[j].heading_ = src_obj.pos_.heading_;
      }
      sensor_ref.aabb_ = src_obj.aabb_;
      sensor_ref.v_x_ = src_obj.v_x_;
      sensor_ref.v_y_ = src_obj.v_y_;

      sensor_ref.a_x_ = src_obj.a_x_;
      sensor_ref.a_y_ = src_obj.a_y_;

      sensor_ref.age_ = 0;

      new_obj->fused_info_.valid_ = false;
      new_obj->fused_info_.obj_type_ = ad_msg::OBJ_TYPE_UNKNOWN;
      new_obj->fused_info_.width_ = src_obj.width_;
      new_obj->fused_info_.height_ = src_obj.height_;
      new_obj->fused_info_.length_ = src_obj.length_;
      new_obj->fused_info_.pos_.x_ = src_obj.pos_.x_;
      new_obj->fused_info_.pos_.y_ = src_obj.pos_.y_;
      new_obj->fused_info_.pos_.range_ = src_obj.pos_.range_;
      new_obj->fused_info_.pos_.angle_ = src_obj.pos_.angle_;
      new_obj->fused_info_.aabb_ = src_obj.aabb_;
      new_obj->fused_info_.v_x_ = src_obj.v_x_;
      new_obj->fused_info_.v_y_ = src_obj.v_y_;

      new_obj->fused_info_.a_x_ = src_obj.a_x_;
      new_obj->fused_info_.a_y_ = src_obj.a_y_; 

      new_obj->heading_ = src_obj.pos_.heading_;
      new_obj->pos_status_(0) = src_obj.pos_.x_;
      new_obj->pos_status_(1) = src_obj.pos_.y_;
      new_obj->pos_status_(2) = src_obj.v_x_;
      new_obj->pos_status_(3) = src_obj.v_y_;
      new_obj->pos_covariance_.SetIdentity();
      new_obj->ClearTrackedTrj();

      tracked_objects_kdtree_.AddObject(tracked_obj_index, sensor_ref.aabb_);
    }

    return (tracked_obj_index);
}

bool ObjsTracker::PreProcessForAddingNewObj(const Int64_t timestamp)
{
    // 判断之前保存的障碍物跟踪列表是否有效
    bool valid_track_list = false;
    Int64_t time_elapsed_track_list = 0;
    if (tracked_objects_.msg_head_.valid) {
      time_elapsed_track_list = common::CalcElapsedClockMs(
            tracked_objects_.msg_head_.timestamp, timestamp);

      //UPPER_LIMIT_TIME_ELAPSED_FOR_TRACK_LIST
      if (time_elapsed_track_list > 200 || time_elapsed_track_list < 0) {
        // 之前保存的障碍物跟踪列表过期了，清除它
        Clear();
      } else {
        if (!tracked_objects_.objects_.Empty()) {
          // 之前保存的障碍物跟踪列表是有效的
          valid_track_list = true;
        }
      }
    }

    //将之前保存的障碍物跟踪列表进行时间及空间同步(对齐到时间流中)
    ad_msg::RelativePos pos_track_list;
    common::Matrix<Float32_t, 3, 3> mat_conv_track_list;
    mat_conv_track_list.SetIdentity();
    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    if (valid_track_list) {
      if (!pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          tracked_objects_.msg_head_.timestamp, rel_pos_list_, &pos_track_list)) {
        LOG_ERR << "[Error]Failed to get pos of tracked list.";
        return false;
      }
      common::Rotate_2D<Float32_t>(rotate_center,
                                pos_track_list.heading - current_rel_pos_.heading,
                                &mat_conv_track_list);
      common::Translate_2D(pos_track_list.x - current_rel_pos_.x,
                           pos_track_list.y - current_rel_pos_.y,
                           &mat_conv_track_list);
    }

    // 清理KD-Tree
    phoenix::common::AABBoxKDTreeParams params;
    params.max_leaf_dimension = 5.0f;  // meters. //MAX_LEAF_DIMENSION_PARAM_OF_KD_TREE
    params.max_leaf_size = 16;  //MAX_LEAF_SIZE_PARAM_OF_KD_TREE
    tracked_objects_kdtree_.SetKDTreeParams(params);
    tracked_objects_kdtree_.Clear();
    kdtree_nodes_stack_.Clear();

    // 删除需要删除的障碍物
    DeleteErasedObjFromTrackList();

    //LOG_INFO(2) << "[timestamp] last time:" << tracked_objects_.msg_head_.timestamp << "  current time:"  <<  timestamp;


    // 将之前跟踪的障碍物进行时间及空间同步(对齐到时间流中)
    UpdatePrevObjTrackedList(time_elapsed_track_list*0.001F, mat_conv_track_list);

    return true;
}

void ObjsTracker::PostProcessForAddingNewObj()
{
    // 根据跟踪列表中的障碍物, 构建KD-Tree, 加快最近障碍物的检索
    tracked_objects_kdtree_.Partition(kdtree_nodes_stack_);

  #if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "After adding sensor objs, the tracked objs are:" << std::endl;
    ShowTrackedObjectsList();
#endif
}

void ObjsTracker::MatchToTrackedObj(const MatchingParams &param, const ObjIndex &src_obj_idx, const ObjInfosBase &src_obj)
{
    if (!src_obj_idx.IsValid()) {
      //LOG_ERR << "Invalid obj index.";
      return;
    }

    common::StaticVector<Int32_t, MAX_PARTNER_NUM> obj_idxs_in_range;
    common::StaticVector<MatchingInfos, MAX_PARTNER_NUM> matched_objs_in_range;


    // if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
    //     src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
    //     src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

    //       printf("===========MatchToTrackedObj===== src_obj.sensor_type_ %d\n",src_obj.sensor_type_);

    //     }

    // 查找临近的所有障碍物（检索半径以内）
    common::Vec2d searching_center;
    if (src_obj.pos_.x_ > 0.0F) {
      searching_center.set_x(src_obj.pos_.x_ + 0.5F*src_obj.length_);
    } else {
      searching_center.set_x(src_obj.pos_.x_ - 0.5F*src_obj.length_);
    }
    searching_center.set_y(src_obj.pos_.y_);

    tracked_objects_kdtree_.FindObjects(
          kdtree_nodes_stack_,
          searching_center,
          param.searching_radius_,
          CalcSquareDistToObjTracked(src_obj.aabb_, tracked_objects_.objects_),
          &obj_idxs_in_range);

    Int32_t num_objs_in_range = obj_idxs_in_range.Size();

  #if ENABLE_OBJ_TRACKER_TRACE
    printf("  ### MatchToTrackedObj, center=(%0.1f, %0.1f), radius=%0.1f"
           ", find %d objs, they are: "
           "\n",
           searching_center.x(), searching_center.y(), param.searching_radius,
           num_objs_in_range);
  #endif

    for (Int32_t j = 0; j < num_objs_in_range; ++j) {
      const ObjIndex* idx =
          tracked_objects_kdtree_.GetObjectByIndex(obj_idxs_in_range[j]);
      if (Nullptr_t == idx) {
        LOG_ERR << "Invalid ObjIndex";
        continue;
      }
      if (!idx->IsValid()) {
        //LOG_ERR << "Invalid obj index.";
        continue;
      }

      ObjsTracked& tracked_obj = tracked_objects_.objects_[idx->obj_idx_];

  #if ENABLE_OBJ_TRACKER_TRACE
      printf("      [%d] list=%d, idx=%d, sen=%d, erase=%d"
             ", pos=(%0.1f,%0.1fd), vx=%0.1f, vy=%0.1f"
             "\n",
             j, tracked_obj.list_id_, idx->obj_idx_, idx->sensor_idx_, tracked_obj.erase_flag_,
             tracked_obj.sensor_ref_[idx->sensor_idx_].pos_[0].x_,
             tracked_obj.sensor_ref_[idx->sensor_idx_].pos_[0].y_,
             tracked_obj.sensor_ref_[idx->sensor_idx_].v_x_*3.6F,
             tracked_obj.sensor_ref_[idx->sensor_idx_].v_y_*3.6F
             );
  #endif

      // printf("      [%d] list=%d, idx=%d, sen=%d, erase=%d"
      //        ", pos=(%0.1f,%0.1fd), vx=%0.1f, vy=%0.1f"
      //        "\n",
      //        j, tracked_obj.list_id_, idx->obj_idx_, idx->sensor_idx_, tracked_obj.erase_flag_,
      //        tracked_obj.sensor_ref_[idx->sensor_idx_].pos_[0].x_,
      //        tracked_obj.sensor_ref_[idx->sensor_idx_].pos_[0].y_,
      //        tracked_obj.sensor_ref_[idx->sensor_idx_].v_x_,
      //        tracked_obj.sensor_ref_[idx->sensor_idx_].v_y_
      //        );

      if (LIST_ID_TRACKED != tracked_obj.list_id_) {
        // 此障碍物是新添加的
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("       ^此障碍物是新添加的, 忽略\n");
  #endif

  #if ENABLE_CAM_DATA_TRACE
        if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

          printf("%d ^此障碍物是新添加的, 忽略\n",src_obj.sensor_type_);

        }
  #endif

        continue;
      }

      if (tracked_obj.erase_flag_) {
        // 此障碍物被删除了
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("       ^此障碍物被删除了, 忽略\n");
  #endif

  #if ENABLE_CAM_DATA_TRACE
      if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

          printf("%d ^此障碍物被删除了, 忽略\n",src_obj.sensor_type_);

        }
  #endif

        continue;
      }
      if (!tracked_obj.sensor_ref_[idx->sensor_idx_].valid_) {
        // 不包含有效的数据
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("       ^不包含有效的数据, 忽略\n");
  #endif

  #if ENABLE_CAM_DATA_TRACE
        if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

          printf("%d ^^不包含有效的数据, 忽略\n",src_obj.sensor_type_);

        }
  #endif

        continue;
      }


      /// TODO: 考虑同个传感器的OBJ总是匹配，但与关联的其它传感器的OBJ偏差较大的情况
      if (!param.masks_[idx->sensor_idx_]) {
        // 不与此传感器匹配
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("       ^不与此传感器匹配, 忽略\n");
  #endif

  #if ENABLE_CAM_DATA_TRACE
        if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

          printf("%d ^不与此传感器匹配, 忽略\n",src_obj.sensor_type_);

        }
  #endif

        continue;
      }

      if (tracked_obj.matched_ < param.min_matched_times_) {
        // 跟踪上的次数太少
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("       ^跟踪上的次数太少, 忽略\n");
  #endif

  #if ENABLE_CAM_DATA_TRACE
        if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
        src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

          printf("printf( %d ^跟踪上的次数太少, 忽略\n",src_obj.sensor_type_);

        }
  #endif

        continue;
      }

      if (param.max_dismatch_times_ > 0) {
        Int32_t threshold_dismatch_times =
            common::Min(common::Max(2, tracked_obj.matched_/2),
                        param.max_dismatch_times_);
#if ENABLE_OTHER_DEBUG_PRINTF
        printf("id %lf,durattion %lf，age %lf dismatch_times %ld\n",tracked_obj.raw_obj_idx_,tracked_obj.duration_,tracked_obj.age_,
        threshold_dismatch_times);
#endif

        if ((tracked_obj.duration_ - tracked_obj.age_) > threshold_dismatch_times) {
          // 最近没有被跟踪上
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("       ^最近没有被跟踪上, 忽略\n");
  #endif

  #if ENABLE_CAM_DATA_TRACE
          if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
            src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
            src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){

              printf("%d ^最近没有被跟踪上, 忽略\n",src_obj.sensor_type_);

            }
  #endif
          continue;
        }
      }

      Float32_t aabb_dist = common::DistAABBToAABB_2D(
            tracked_obj.sensor_ref_[idx->sensor_idx_].aabb_, src_obj.aabb_);
      Float32_t abs_vx_diff = common::com_abs(
            src_obj.v_x_ - tracked_obj.sensor_ref_[idx->sensor_idx_].v_x_);
      Float32_t abs_vy_diff = common::com_abs(
            src_obj.v_y_ - tracked_obj.sensor_ref_[idx->sensor_idx_].v_y_);
      if (true == param.match_with_velocity_)  
      {
        if (((abs_vx_diff > param.max_vx_diff_) ||
                (abs_vy_diff > param.max_vy_diff_)) &&
                (aabb_dist > 0.01F)) {
              // 车速偏差过大 (若障碍物重合，则不判断速度偏差)
        #if ENABLE_OBJ_TRACKER_TRACE
              printf("       ^车速偏差过大, 忽略\n");
        #endif
              continue;
        }

        if(common::com_abs(src_obj.pos_.y_ - tracked_obj.sensor_ref_[idx->sensor_idx_].pos_[0].y_) > 1.7) //1.7 为半个车道宽度
        {
          if(src_obj.sensor_type_ == SensorType::SENSOR_TYPE_MAIN_FORWARD_RADAR && idx->sensor_idx_ == SensorID::SENSOR_ID_MAIN_FORWARD_CAM && \
            src_obj.obj_type_ == ad_msg::OBJ_TYPE_CURB && tracked_obj.sensor_ref_[idx->sensor_idx_].obj_type_ != ad_msg::OBJ_TYPE_CURB && \
            common::com_abs(tracked_obj.sensor_ref_[idx->sensor_idx_].v_x_) < 6.0 // 6.0的设置与前视一体机的预处理中的速度上限设置对应。
            ){ 
            // printf("[WDebug]0000\n");
            continue;
          }

          if(src_obj.sensor_type_ == SensorType::SENSOR_TYPE_MAIN_FORWARD_CAMERA && idx->sensor_idx_ == SensorID::SENSOR_ID_MAIN_FORWARD_RADAR && \
            src_obj.obj_type_ != ad_msg::OBJ_TYPE_CURB && tracked_obj.sensor_ref_[idx->sensor_idx_].obj_type_ == ad_msg::OBJ_TYPE_CURB && \
            common::com_abs(src_obj.v_x_) < 6.0 // 6.0的设置与前视一体机的预处理中的速度上限设置对应。
            ){ 
            // printf("[WDebug]0001\n");
            continue;
          }

        }

      }


      bool always_match = false;

      if (idx->sensor_idx_ == src_obj_idx.sensor_idx_) {
        if (tracked_obj.sensor_ref_[idx->sensor_idx_].FindObjId(
              src_obj.raw_obj_id_) < 0) {
          // ID不匹配
          always_match = false;
        } else {
          // ID匹配
          always_match = true;
        }
      }

  #if ENABLE_OBJ_TRACKER_TRACE
      printf("       ^可匹配，加入候选列表\n");
  #endif


      if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
            src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
            src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){
  
   #if ENABLE_CAM_DATA_TRACE
              printf("%d ^可匹配，加入候选列表\n",src_obj.sensor_type_);
   #endif

      }



      MatchingInfos match_info;
      match_info.always_match_ = always_match;
      match_info.candidate_idx_ = *idx;
      matched_objs_in_range.PushBack(match_info);

      // if(src_obj.sensor_type_==SENSOR_ID_LEFT_BACKWARD_CAM||
      //   src_obj.sensor_type_==SENSOR_ID_RIGHT_BACKWARD_CAM||
      //   src_obj.sensor_type_==SENSOR_ID_MAIN_FORWARD_CAM){
      //   printf("===========MatchToTrackedObj===== \n");
      // }
    }

    /**
     * @brief 根据基于kdtree的最邻近搜索，确定新输入的src_obj相邻的符合要求的之前的融合后的障碍物的
     * 伙伴关系
     */
    AddPartnerRelationship(
                param, src_obj_idx, src_obj, matched_objs_in_range);
}

void ObjsTracker::FindBestMatchedObj(const Int32_t sen_id, const MatchingParams &param)
{
    Int32_t list_id = GetListIdBySensorId(sen_id);
    COM_CHECK(list_id >= 0);

    // 将关联的障碍物信息按cost大小来排序
    SortObjAssoList(list_id);

    // 查找最佳的匹配
    SetBestMatchedObj(list_id);

    // 更新匹配上的obj
    UpdateMatchedObj(sen_id, list_id, param);

    // 更新未匹配上的obj
    UpdateUnmatchedObj(sen_id, list_id, param);
}

void ObjsTracker::UpdateTrackList(const ObjsFilterDataSource &data_source)
{
    // 清除过期的传感器信息
    EraseExpiredObj();

    // 合并部分障碍物
    MergeTrackedObjs();

  #if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "After merging objs, the tracked objs are:" << std::endl;
    ShowTrackedObjectsList();
  #endif

  #if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "After erasing some objs, the tracked objs are:" << std::endl;
    ShowTrackedObjectsList();
  #endif

    // 更新置信度，并且从障碍物跟踪列表中移除过期的障碍物
    UpdateObjConfidence();

    //更改挂角的单侧雷达融合置信度  
    ModifyAroundRadarObjConfidence();

    // 更新融合障碍物Id
    UpdataObjFusionId();

    // 用路沿过滤路边障碍物
    //CurbFiltObj(data_source);

    //  TODO: 缺高精度地图   // 给障碍物分组
    GroupObjs(data_source);

    //检查 life_ 值状态
    CheckObjLife();

    // 平滑前视障碍物位置
    SmoothTrackedObjs();

    //平滑左侧边的障碍物位置
    SmoothAroundLeftTrackedObjs();
    
    //平滑右侧边的障碍物位置
    SmoothAroundRightTrackedObjs();

  #if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "After filtering, the tracked objs are:" << std::endl;
    ShowTrackedObjectsList();
  #endif

    // 更新障碍物跟踪列表的时间戳信息
    tracked_objects_.msg_head_.valid = true;
    tracked_objects_.msg_head_.UpdateSequenceNum();
    tracked_objects_.msg_head_.timestamp = data_source.timestamp_;

    /**
     * @brief 更新所有obj的last_perception_type， 原因是SmoothTrackedObjs中可能会用到last_perception_type
     * 所以对其进行判断，所以需要在整个融合主流程的最后进行last_perception_type值的更新。
     */
    Int32_t obj_num = tracked_objects_.objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjsTracked& obj = tracked_objects_.objects_[i];
      if (obj.erase_flag_) {
        continue;
      }
      obj.last_perception_type = obj.perception_type;
    }

    

}

void ObjsTracker::GetObstacleTrackedInfoList(ad_msg::ObstacleTrackedInfoList *obj_list) const
{
    Int32_t obj_num = tracked_objects_.objects_.Size();

    Int32_t dst_obj_idx = 0;
    for (Int32_t i = 0; i < obj_num; ++i) {
      if (dst_obj_idx >= ad_msg::ObstacleTrackedInfoList::MAX_OBSTACLE_NUM) {
        break;
      }
      const ObjsTracked& src_obj = tracked_objects_.objects_[i];
      if (src_obj.erase_flag_) {
        continue;
      }
      ad_msg::ObstacleTrackedInfo& dst_obj = obj_list->obstacles[dst_obj_idx];
      dst_obj_idx++;

      if (LIST_ID_TRACKED == src_obj.list_id_) {
        dst_obj.obj_type = src_obj.fused_info_.obj_type_;

        dst_obj.width = src_obj.fused_info_.width_;
        dst_obj.length = src_obj.fused_info_.length_;

        dst_obj.x = src_obj.fused_info_.pos_.x_;
        dst_obj.y = src_obj.fused_info_.pos_.y_;

  #if  ENABLE_OTHER_DEBUG_PRINTF
        if (common::com_abs(dst_obj.x) < 0.5F &&
            common::com_abs(dst_obj.y) < 0.5F) {
          printf("[INFO]ERROR idx[%d], erase=%d, x=%f, y=%f\n", i, src_obj.erase_flag_, dst_obj.x, dst_obj.y);
        }
  #endif

        dst_obj.v_x = src_obj.fused_info_.v_x_;
        dst_obj.v_y = src_obj.fused_info_.v_y_;
      } else {
        Int32_t sen_id = GetSensorIdByListId(src_obj.list_id_);
        if (sen_id >= 0) {
          const SensorsRef& sen_ref = src_obj.sensor_ref_[sen_id];

          dst_obj.obj_type = sen_ref.obj_type_;

          dst_obj.width = sen_ref.width_;
          dst_obj.length = sen_ref.length_;

          dst_obj.x = sen_ref.pos_[0].x_;
          dst_obj.y = sen_ref.pos_[0].y_;

  #if  ENABLE_OTHER_DEBUG_PRINTF
          if (common::com_abs(dst_obj.x) < 0.5F &&
              common::com_abs(dst_obj.y) < 0.5F) {
            printf("[INFO]ERROR idx[%d], erase=%d, x=%f, y=%f\n", i, src_obj.erase_flag_, dst_obj.x, dst_obj.y);
          }
  #endif
          dst_obj.v_x = sen_ref.v_x_;
          dst_obj.v_y = sen_ref.v_y_;
        } else {
          LOG_ERR << "[Error]Invalid sensor id.";
        }
      }

      Float32_t half_width = 0.5F*dst_obj.width;
      Float32_t half_length = 0.5F*dst_obj.length;

      dst_obj.obb.x = dst_obj.x + half_length;
      dst_obj.obb.y = dst_obj.y;
      dst_obj.obb.heading = src_obj.heading_;
      dst_obj.obb.half_width = half_width;
      dst_obj.obb.half_length = half_length;

      dst_obj.track_status = src_obj.track_status_;
      dst_obj.age = src_obj.age_;
      dst_obj.duration = src_obj.duration_;
      dst_obj.confidence = src_obj.confidence_;
    }

    obj_list->obstacle_num = dst_obj_idx;
    obj_list->msg_head = tracked_objects_.msg_head_;
}

void ObjsTracker::GetObstaclesLists(ad_msg::ObstacleList *obj_list) const
{
#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### GetObstacleList (Start) ###" << std::endl;
#endif

  Int32_t tracked_obj_num = tracked_objects_.objects_.Size();

  Int32_t dst_obj_index = 0;

#if  ENABLE_OTHER_DEBUG_PRINTF
  LOG_ERR << "tracked_obj_num：" << tracked_obj_num;
#endif

  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    const ObjsTracked& src_obj = tracked_objects_.objects_[i];
    if (src_obj.erase_flag_) {
      continue;
    }
    
#if REMOVE_SMALL_WIDTH_OBJ
    if(src_obj.fused_info_.width_<=0.5&&src_obj.fused_info_.length_<=0.5){

      continue;
      
    }
#endif
    Int32_t obj_type = ad_msg::OBJ_TYPE_UNKNOWN;
    Int32_t perception_type = ad_msg::OBJ_PRCP_TYPE_UNKNOWN;
    Float32_t width = 0.0F;
    Float32_t length = 0.0F;
    Float32_t obj_x = src_obj.pos_status_(0);
    Float32_t obj_y = src_obj.pos_status_(1);
    Float32_t vx = 0.0F;
    Float32_t vy = 0.0F;

    Float32_t ax = 0.0F;
    Float32_t ay = 0.0F;

    if (src_obj.IsRadarValid() && src_obj.IsCameraValid()) {
      // Radar与相机融合的obj
      obj_type = src_obj.fused_info_.obj_type_;
      perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;

      width = src_obj.fused_info_.width_;
      length = src_obj.fused_info_.length_;

      obj_x = src_obj.pos_status_(0);
      // obj_y = src_obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].pos_[0].y_;
      obj_y = src_obj.pos_status_(1);

      /* w001 pengc 2021-12-16 (begin) */
      // 当障碍物在车辆侧方时，毫米波检测的是障碍物的侧面,
      // 可能造成比较靠近的障碍物被检测到当前车道范围内，导致急刹车
      if ((obj_x < 15.0F) && (obj_x > -2.0F)) {
        if ((src_obj.fused_info_.pos_.angle_ > common::com_deg2rad(10.0F)) &&
            (src_obj.fused_info_.pos_.angle_ < common::com_deg2rad(90.0F))) {
          //obj_y += 0.1F * width; // 2022-04-03: 0.25
        } else if ((src_obj.fused_info_.pos_.angle_ <
                    common::com_deg2rad(-10.0F)) &&
                   (src_obj.fused_info_.pos_.angle_ >
                    common::com_deg2rad(-90.0F))) {
          //obj_y -= 0.1F * width; // 2022-04-03: 0.25
        } else {
          // nothing to do
        }
      }
      /* w001 pengc 2021-12-16 (end) */

      vx = src_obj.fused_info_.v_x_;
      vy = src_obj.fused_info_.v_y_;

      ax = src_obj.fused_info_.a_x_;
      ay = src_obj.fused_info_.a_y_;
    } else if (!src_obj.IsRadarValid() && src_obj.IsCameraValid()) {
      // 仅相机识别的obj
      obj_type = src_obj.fused_info_.obj_type_;
      perception_type = ad_msg::OBJ_PRCP_TYPE_CAMERA;

      width = src_obj.fused_info_.width_;
      length = src_obj.fused_info_.length_;

      obj_x = src_obj.fused_info_.pos_.x_;
      obj_y = src_obj.fused_info_.pos_.y_;

      vx = src_obj.fused_info_.v_x_;
      vy = src_obj.fused_info_.v_y_;
      
      ax = src_obj.fused_info_.a_x_;
      ay = src_obj.fused_info_.a_y_;
    } else if (src_obj.IsRadarValid() && !src_obj.IsCameraValid()) {
      // 仅Radar感知的obj
      obj_type = src_obj.fused_info_.obj_type_;

      if(0==obj_type&&fabs(src_obj.fused_info_.v_x_)>5.0){

         obj_type=ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;

      }else if(0==obj_type&&fabs(src_obj.fused_info_.v_x_)>2.0&&fabs(src_obj.fused_info_.v_x_)<5.0){

          obj_type=ad_msg::OBJ_TYPE_PEDESTRIAN;

      }

      perception_type = ad_msg::OBJ_PRCP_TYPE_RADAR;

      width = src_obj.fused_info_.width_;
      length = src_obj.fused_info_.length_;

      obj_x = src_obj.pos_status_(0);
      obj_y = src_obj.pos_status_(1);

      /* w001 pengc 2021-12-16 (begin) */
      // 当障碍物在车辆侧方时，毫米波检测的是障碍物的侧面,
      // 可能造成比较靠近的障碍物被检测到当前车道范围内，导致急刹车

      //if(common::com_rad2deg(src_obj.fused_info_.pos_.angle_)>0.0 && obj_y<5.0 && obj_y>0.0)
      //printf(" only radar angle is %lf obj_x : %lf  obj_y ：%lf\n",common::com_rad2deg(src_obj.fused_info_.pos_.angle_),obj_x,obj_y);
      //if ((obj_x < 15.0F) && (obj_x > -2.0F)) {
        if ((src_obj.fused_info_.pos_.angle_ > common::com_deg2rad(10.0F)) &&
            (src_obj.fused_info_.pos_.angle_ < common::com_deg2rad(90.0F))) {
          //obj_y += 0.5F * width;
          obj_y += width;
        } else if ((src_obj.fused_info_.pos_.angle_ <
                    common::com_deg2rad(-10.0F)) &&
                   (src_obj.fused_info_.pos_.angle_ >
                    common::com_deg2rad(-90.0F))) {
          //obj_y -= 0.5F * width;
          obj_y -= width;
        } else {
          // nothing to do
        }
      //}
      /* w001 pengc 2021-12-16 (end) */

      vx = src_obj.fused_info_.v_x_;
      vy = src_obj.fused_info_.v_y_;

      ax = src_obj.fused_info_.a_x_;
      ay = src_obj.fused_info_.a_y_;

      /// TODO: 需要使用融合后的数据
      /// TODO: If the obstacle have been tracked before (v>15km/h),
      /// then it need to be preserved in obstacle list even if
      /// it's current velocity is less than 15km/h
      // if (common::com_abs(vx) < 15.0F/3.6F) {
      //   if (src_obj.group.member_num < 1) {
      //     continue;
      //   }
      //   if (src_obj.group.corrected_pos.valid) {
      //     obj_x = src_obj.group.corrected_pos.x;
      //     obj_y = src_obj.group.corrected_pos.y;
      //   }
      // }
    } else {
      LOG_ERR << "Invalid obj.";
      continue;
    }

    //if (!src_obj.IsSrr2Valid()) {
    if (src_obj.confidence_ < 60) {
      if (src_obj.matched_ < 3) {
        continue;
      }
    }
    //}
    if ((obj_x < (vehicle_params_.dist_of_localization_to_front_ + 5.0F)) &&
      (common::com_abs(obj_y) > 15.0F)) {
      continue;
    }

    Int32_t threshold_dismatch_times =
        common::Min(common::Max(5, src_obj.matched_/2), 5);

    if ((src_obj.duration_ - src_obj.age_) > threshold_dismatch_times) {
      continue;
    }

    if (dst_obj_index >= ad_msg::ObstacleList::MAX_OBSTACLE_NUM) {
      LOG_ERR << "[Error]Can't add obj to list, storage is full.";
      break;
    }
    ad_msg::Obstacle& dst_obj = obj_list->obstacles[dst_obj_index];
    dst_obj_index++;

    /// Test
    //obj_x = src_obj.avg_status.x;
    //obj_y = src_obj.avg_status.y;

    dst_obj.x = obj_x;
    dst_obj.y = obj_y;
    Float32_t half_width = 0.5F*width;
    Float32_t half_length = 0.5F*length;

    if (ad_msg::OBJ_TYPE_CURB == obj_type) {
      dst_obj.obb.x = obj_x + 1.0F;
      dst_obj.obb.y = obj_y;
      /// Warning: Fake heading
      dst_obj.obb.heading = src_obj.ref_line_.proj_on_major_ref_line_.heading;
      dst_obj.obb.half_width = 0.1F;
      dst_obj.obb.half_length = 1.0F;
    } else {
      dst_obj.obb.x = obj_x + half_length;
      dst_obj.obb.y = obj_y;
      dst_obj.obb.heading = src_obj.heading_;
      dst_obj.obb.half_width = half_width;
      dst_obj.obb.half_length = half_length;
    }

    dst_obj.v_x = vx;
#if 0
    dst_obj.v_y = src_obj.pos_status_(3);
    dst_obj.v = common::com_sqrt(common::Square(dst_obj.v_x) +
                                 common::Square(dst_obj.v_y));
#else
    dst_obj.v_y = src_obj.pos_status_(3);
    // dst_obj.v = common::com_sqrt(common::Square(dst_obj.v_x) +
    //                              common::Square(dst_obj.v_y));
    dst_obj.v = common::com_abs(vx);
#endif

    dst_obj.a_x = ax;
    dst_obj.a_y = ay;

    if (common::com_abs(dst_obj.v_x) < 15.0F/3.6F) {
      if (dst_obj.v_x < 2.0F/3.6F) {
        dst_obj.v = 0.0F;
      } else {
        dst_obj.v = dst_obj.v_x;
      }
      dst_obj.dynamic = false;
    } else {
      dst_obj.dynamic = true;
    }

    dst_obj.type = obj_type;
    dst_obj.perception_type = perception_type;
    
    dst_obj.confidence = src_obj.confidence_;

    dst_obj.proj_on_major_ref_line.valid = 0;

#if (DEBUG_FUSION_OBJ_CONFIDENCE)

    dst_obj.life = src_obj.life_;
    dst_obj.matched = src_obj.matched_;
    dst_obj.age = src_obj.age_;
    dst_obj.duration = src_obj.duration_;
    
#endif


#if DEBUG_AROUND_RADAR_ID

  // SensorID id_tmp =  SensorID::SENSOR_ID_LEFT_SIDE_RADAR;

  // if (src_obj.sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR].valid_)
  //   id_tmp = SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR;
  // if (src_obj.sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_RADAR].valid_)
  //   id_tmp = SensorID::SENSOR_ID_LEFT_SIDE_RADAR;
  // if (src_obj.sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_RADAR].valid_)
  //   id_tmp = SensorID::SENSOR_ID_MAIN_FORWARD_RADAR;
  // if (src_obj.sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_)
  //   id_tmp = SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR;
  // if (src_obj.sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_RADAR].valid_)
  //   id_tmp = SensorID::SENSOR_ID_RIGHT_SIDE_RADAR;
  
  // dst_obj.id = src_obj.sensor_ref_[id_tmp].obj_id_[0].id_;
  dst_obj.id = src_obj.fusion_id;
#endif

    common::RingBuffer<TrjPointInfos,
        MAX_TRACKED_OBJ_TRJ_POINT_NUM>::const_iterator trj_it =
        src_obj.traced_trj_[1].cbegin();
    common::RingBuffer<TrjPointInfos,
        MAX_TRACKED_OBJ_TRJ_POINT_NUM>::const_iterator trj_it_end =
        src_obj.traced_trj_[1].cend();
    Int32_t trj_pt_idx = 0;
    for (; trj_it != trj_it_end; ++trj_it) {
      const TrjPointInfos* pt = trj_it.current();
      if (trj_pt_idx >= ad_msg::Obstacle::MAX_TRACKED_PATH_POINT_NUM) {
        break;
      }
      dst_obj.tracked_path[trj_pt_idx].x = pt->x_;
      dst_obj.tracked_path[trj_pt_idx].y = pt->y_;
      trj_pt_idx++;
    }
    dst_obj.tracked_path_point_num = trj_pt_idx;

#if ENABLE_OBJ_TRACKER_TRACE
    printf("    [%d]"
           " list=%d, raw_obj=%d"
           ", pos=[%0.1f,%0.1f]"
           ", vx=%0.1fkm/h, vy=%0.1fkm/h, width=%0.1f, length=%0.1f"
           ", life=%d, matched=%d, age=%d, duration=%d, confidence=%d"
           "\n",
           i,
           src_obj.list_id, src_obj.raw_obj_idx_,
           obj_x, obj_y,
           vx*3.6F, vy*3.6F, width, length,
           src_obj.life_, src_obj.matched_, src_obj.age_,
           src_obj.duration_, src_obj.confidence_
        );
#endif
  }

  obj_list->obstacle_num = dst_obj_index;
  obj_list->msg_head = tracked_objects_.msg_head_;

  //对obj_list进行聚类

  //遍历
  #if 1
  int cur_obj_index=0;

  int next_obj_index=0;

  while(cur_obj_index<obj_list->obstacle_num){  
    ad_msg::Obstacle& cur_cluster_obj = obj_list->obstacles[cur_obj_index];
    if(cur_cluster_obj.confidence<80){
      cur_obj_index++;
      continue;
    }

    //printf("cur_cluster_obj perception_type is %d\n",cur_cluster_obj.perception_type);
    if(cur_cluster_obj.perception_type==ad_msg::OBJ_PRCP_TYPE_RADAR){
      cur_obj_index++;
      continue;

    }

    //double cur_distance =common::com_sqrt(common::Square(cur_cluster_obj.x) +
                                 //common::Square(cur_cluster_obj.y));
    next_obj_index=0;

    //查找以cur_obj为圆心，置信度>80 半径3米内的点
    while(next_obj_index<obj_list->obstacle_num){

      ad_msg::Obstacle& next_cluster_obj = obj_list->obstacles[next_obj_index];

      if(next_cluster_obj.confidence<80 || cur_obj_index==next_obj_index){

          next_obj_index++;
          continue;
      }
     // double next_distance =common::com_sqrt(common::Square(next_cluster_obj.x) +
           //                      common::Square(next_cluster_obj.y));

      if(fabs(cur_cluster_obj.x-next_cluster_obj.x)<3.0&&fabs(cur_cluster_obj.y-next_cluster_obj.y)<1.5){
      
           if(fabs(cur_cluster_obj.y)>fabs(next_cluster_obj.y)){
              
                next_cluster_obj.y=cur_cluster_obj.y;
          }

      }
      next_obj_index++;
      
    }
    cur_obj_index++;

  }
  #endif

#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### GetObstacleList (End) ###" << std::endl;
#endif
}

ObjsTracker::ObjsTracker(const EgoVehicleParams &vehicle_params)
    : vehicle_params_(vehicle_params)
{

}

Int32_t ObjsTracker::GetListIdBySensorId(const Int32_t sen_id) const
{
    Int32_t list_id = ListID::LIST_ID_INVALID;

    switch (sen_id) {
    case (SensorID::SENSOR_ID_CENTER_FORWARD_CAM):
      list_id = ListID::LIST_ID_CENTER_FORWARD_CAM;
      break;
    case (SensorID::SENSOR_ID_INSIDE_DMS_CAM):
      list_id = ListID::LIST_ID_INSIDE_DMS_CAM;
      break;
    case (SensorID::SENSOR_ID_INSIDE_OMS_CAM):
      list_id = ListID::LIST_ID_INSIDE_OMS_CAM;
      break;
    case (SensorID::SENSOR_ID_LEFT_BACKWARD_CAM):
      list_id = ListID::LIST_ID_LEFT_BACKWARD_CAM;
      break;
    case (SensorID::SENSOR_ID_LEFT_SIDE_CAM):
      list_id = ListID::LIST_ID_LEFT_SIDE_CAM;
      break;
    case (SensorID::SENSOR_ID_RIGHT_BACKWARD_CAM):
      list_id = ListID::LIST_ID_RIGHT_BACKWARD_CAM;
      break;
    case (SensorID::SENSOR_ID_RIGHT_SIDE_CAM):
      list_id = ListID::LIST_ID_RIGHT_SIDE_CAM;
      break;
    case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):
      list_id = ListID::LIST_ID_MAIN_FORWARD_CAM;
      break;

    case (SensorID::SENSOR_ID_MAIN_FORWARD_RADAR):
      list_id = ListID::LIST_ID_MAIN_FORWARD_RADAR;
      break;
    case (SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR):
      list_id = ListID::LIST_ID_LEFT_BACKWARD_RADAR;
      break;
    case (SensorID::SENSOR_ID_LEFT_SIDE_RADAR):
      list_id = ListID::LIST_ID_LEFT_SIDE_RADAR;
      break;
    case (SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR):
      list_id = ListID::LIST_ID_RIGHT_BACKWARD_RADAR;
      break;
    case (SensorID::SENSOR_ID_RIGHT_SIDE_RADAR):
      list_id = ListID::LIST_ID_RIGHT_SIDE_RADAR;
      break;

    case (SensorID::SENSOR_ID_MAIN_FORWARD_LIDAR):
      list_id = ListID::LIST_ID_MAIN_FORWARD_LIDAR;
      break;

    default:
      break;
    }

    return (list_id);
}

Int32_t ObjsTracker::GetSensorIdByListId(const Int32_t list_id) const
{
    Int32_t sen_id = SENSOR_ID_INVALID;
    switch (list_id) {
    case (ListID::LIST_ID_MAIN_FORWARD_CAM):
      sen_id = SENSOR_ID_MAIN_FORWARD_CAM;
      break;
    case (ListID::LIST_ID_CENTER_FORWARD_CAM):
      sen_id = SENSOR_ID_CENTER_FORWARD_CAM;
      break;
    case (ListID::LIST_ID_INSIDE_DMS_CAM):
      sen_id = SENSOR_ID_INSIDE_DMS_CAM;
      break;
    case (ListID::LIST_ID_INSIDE_OMS_CAM):
      sen_id = SENSOR_ID_INSIDE_OMS_CAM;
      break;
    case (ListID::LIST_ID_LEFT_BACKWARD_CAM):
      sen_id = SENSOR_ID_LEFT_BACKWARD_CAM;
      break;
    case (ListID::LIST_ID_RIGHT_BACKWARD_CAM):
      sen_id = SENSOR_ID_RIGHT_BACKWARD_CAM;
      break;
    case (ListID::LIST_ID_LEFT_SIDE_CAM):
      sen_id = SENSOR_ID_LEFT_SIDE_CAM;
      break;
    case (ListID::LIST_ID_RIGHT_SIDE_CAM):
      sen_id = SENSOR_ID_RIGHT_SIDE_CAM;
      break;

    case (ListID::LIST_ID_MAIN_FORWARD_RADAR):
      sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
      break;
    case (ListID::LIST_ID_LEFT_BACKWARD_RADAR):
      sen_id = SENSOR_ID_LEFT_BACKWARD_RADAR;
      break;
    case (ListID::LIST_ID_LEFT_SIDE_RADAR):
      sen_id = SENSOR_ID_LEFT_SIDE_RADAR;
      break;
    case (ListID::LIST_ID_RIGHT_BACKWARD_RADAR):
      sen_id = SENSOR_ID_RIGHT_BACKWARD_RADAR;
      break;
    case (ListID::LIST_ID_RIGHT_SIDE_RADAR):
      sen_id = SENSOR_ID_RIGHT_SIDE_RADAR;
      break;

    case (ListID::LIST_ID_MAIN_FORWARD_LIDAR):
      sen_id = SENSOR_ID_MAIN_FORWARD_LIDAR;
      break;

    default:
      break;
    }

    return (sen_id);
}

void ObjsTracker::DeleteErasedObjFromTrackList()
{
    Int32_t obj_num = tracked_objects_.objects_.Size();

    Int32_t last_obj_index = obj_num-1;
    for (Int32_t i = 0; i <= last_obj_index; ++i) {
      tracked_objects_.objects_[i].ClearAssoList(obj_ass_pool_);
    }

    for (Int32_t i = 0; i <= last_obj_index; ++i) {
      ObjsTracked& obj = tracked_objects_.objects_[i];

      if (obj.erase_flag_) {
        if (i != last_obj_index) {
          obj = tracked_objects_.objects_[last_obj_index];
          tracked_objects_.objects_[last_obj_index].Clear();
          last_obj_index--;
          i--;
          continue;
        } else {
          tracked_objects_.objects_[last_obj_index].Clear();
          last_obj_index--;
          break;
        }
      }
    }
    if (last_obj_index < (obj_num-1)) {
      tracked_objects_.objects_.Resize(last_obj_index+1);
    }
}

void ObjsTracker::UpdatePrevObjTrackedList(const Float32_t time_elapsed_ms, const common::Matrix<Float32_t, 3, 3> &mat_conv)
{
    Int32_t obj_nums = tracked_objects_.objects_.Size();

    //更新之前保存的障碍物列表的位置信息（时间与空间同步）
    common::Matrix<Float32_t, 2, 1> point_conv;
    for (Int32_t i = 0; i < obj_nums; ++i) {
        ObjsTracked& obj = tracked_objects_.objects_[i];

        obj.list_id_ = ListID::LIST_ID_TRACKED;
        obj.raw_obj_idx_ = -1;

        //更新跟踪状态

          obj.life_--;

        /// Don't change obj.matched
        /// Don't change obj.age
        obj.duration_++;
        int tmp = obj.duration_ - obj.age_;

        if (obj.duration_ > MAX_TRACKED_OBJ_DURATION) {
          obj.duration_ = MAX_TRACKED_OBJ_DURATION;

          obj.age_ = obj.duration_ - tmp;
        }
        /// Don't change obj.confidence

        /**
         * @brief 当前视一体机数据已经消失，融合结果需要预测一段时间后再更换融合类型，当相机消失，但是前毫米波雷达还在，相机的
         * 横向速度需要借助前向毫米波雷达的横向速度进行估算。这里用上一次的数据与毫米波雷达进行加权平均的算法进行。
         * 
         */
        if (obj.fused_info_.pos_.x_ < MIN_X_VALUE_LIMIT_FOR_FUSED_Y_VY_VALUE_CALC&&
          obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_)
        {
            Float32_t v_y_ = obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].v_y_;
            Float32_t radar_radio = 0.9;
            Float32_t camera_radio = 0.1;
            v_y_ = camera_radio*v_y_ + radar_radio*obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].v_y_;
            obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].v_y_ = v_y_/(radar_radio + camera_radio);
        }

        for (Int32_t j = 0; j < SENSOR_ID_MAX; ++j) {
            SensorsRef& sensor_ref = obj.sensor_ref_[j];

            /// Don't change sensor_ref.valid
            /// Don't change sensor_ref.obj_id
            /// Don't change sensor_ref.obj_type
            ///
            //printf("valid:%d\n", sensor_ref.valid_);
            if (!sensor_ref.valid_) {
              continue;
            }

            // 对位置进行时间及空间同步
            for (Int32_t k = 0; k < SensorsRef::MAX_POS_POINT_NUM; ++k) {
              ObjPosition& pos = sensor_ref.pos_[k];
            #if FIX_BUG_FUSION_OBJ_RECOIL
                point_conv(0) = pos.x_ + sensor_ref.v_x_ * time_elapsed_ms; 
                point_conv(1) = pos.y_ + sensor_ref.v_y_ * time_elapsed_ms;

            #else

              if ((k >= 2) && ((obj.duration_ - obj.age_) < 5)) {  //当前只维护两条轨迹
                //printf("  [%d] pos=(%0.1f, %0.1f), vx=%0.1f, vy=%0.1f, time=%0.1f\n",
                //       k, pos.x, pos.y, sensor_ref.v_x*3.6f, sensor_ref.v_y*3.6f, time_elapsed_ms);
                point_conv(0) = pos.x_ + sensor_ref.v_x_ * time_elapsed_ms; 
                point_conv(1) = pos.y_ + sensor_ref.v_y_ * time_elapsed_ms;
              } else {
                point_conv(0) = pos.x_;
                point_conv(1) = pos.y_;
              }

            #endif
              common::TransformVert_2D(mat_conv, &point_conv);

              pos.x_ = point_conv(0); 
              pos.y_ = point_conv(1);
              pos.range_ = common::com_sqrt(
                    common::Square(pos.x_) + common::Square(pos.y_));
              pos.angle_ = common::com_atan2(pos.y_, pos.x_);

              //printf("  after [%d] pos=(%0.1f, %0.1f)\n",
              //       k, pos.x, pos.y);
            }

            // 根据更新后的位置重新计算AABB包围盒
            Float32_t min_x = sensor_ref.pos_[0].x_;
            Float32_t max_x = sensor_ref.pos_[0].x_;
            Float32_t min_y = sensor_ref.pos_[0].y_;
            Float32_t max_y = sensor_ref.pos_[0].y_;
            for (Int32_t k = 1; k < SensorsRef::MAX_POS_POINT_NUM; ++k) {
              ObjPosition& pos = sensor_ref.pos_[k];
              if (pos.x_ < min_x) {
                min_x = pos.x_;
              }
              if (pos.x_ > max_x) {
                max_x = pos.x_;
              }
              if (pos.y_ < min_y) {
                min_y = pos.y_;
              }
              if (pos.y_ > max_y) {
                max_y = pos.y_;
              }
            }

            Float32_t length = 0.0F;
            Float32_t height = 0.0F;
            Float32_t width = 0.0F;
            obj.GetObjOverallDimension(j, &length, &width, &height);
            Float32_t half_width = 0.5F * width;

            if (max_x > 0.0F) {
              max_x += length;
            } else {
              min_x -= length;
            }
            min_y -= half_width;
            max_y += half_width;
            sensor_ref.aabb_.min().set_x(min_x);
            sensor_ref.aabb_.min().set_y(min_y);
            sensor_ref.aabb_.max().set_x(max_x);
            sensor_ref.aabb_.max().set_y(max_y);

            /// Don't change sensor_ref.v_x
            /// Don't change sensor_ref.v_y
            /**
             * 此处kdtree的AddObject目的是将之前所有的有效障碍物数据建立成一个搜索树。每一次执行融合周期都会重新建立
             * 依次搜索树 
            */
            ObjIndex obj_index(i, j);
            tracked_objects_kdtree_.AddObject(obj_index, sensor_ref.aabb_);

        }
        /// 融合后的信息
        /// Don't change fused_info.valid
        /// Don't change fused_info.obj_type
        /// Don't change fused_info.width
        /// Don't change fused_info.length
        point_conv(0) = obj.fused_info_.pos_.x_;
        point_conv(1) = obj.fused_info_.pos_.y_;
            
        common::TransformVert_2D(mat_conv, &point_conv);

        obj.fused_info_.pos_.x_ = point_conv(0);
        obj.fused_info_.pos_.y_ = point_conv(1);
        obj.fused_info_.pos_.range_ = common::com_sqrt(
              common::Square(obj.fused_info_.pos_.x_) +
              common::Square(obj.fused_info_.pos_.y_));
        obj.fused_info_.pos_.angle_ =
            common::com_atan2(obj.fused_info_.pos_.y_, obj.fused_info_.pos_.x_);
        // 计算包围盒
        Float32_t half_width = 0.5F * obj.fused_info_.length_;
        if (obj.fused_info_.pos_.x_ > 0.0F) {
          obj.fused_info_.aabb_.min().set_x(obj.fused_info_.pos_.x_);
          obj.fused_info_.aabb_.max().set_x(obj.fused_info_.pos_.x_ + obj.fused_info_.length_);
        } else {
          obj.fused_info_.aabb_.min().set_x(obj.fused_info_.pos_.x_ - obj.fused_info_.length_);
          obj.fused_info_.aabb_.max().set_x(obj.fused_info_.pos_.x_);
        }
        obj.fused_info_.aabb_.min().set_y(obj.fused_info_.pos_.y_ - half_width);
        obj.fused_info_.aabb_.max().set_y(obj.fused_info_.pos_.y_ + half_width);
        /// Don't change fused_info.v_x
        /// Don't change fused_info.v_y


       
        /// 更新障碍物的航向角（因为是估计计算的，所以可以不用更新）
        /// obj.heading
        // 对平滑后的位置进行时间及空间同步
        point_conv(0) = obj.pos_status_(0);
        point_conv(1) = obj.pos_status_(1);

        common::TransformVert_2D(mat_conv, &point_conv);

        obj.pos_status_(0) = point_conv(0);
        obj.pos_status_(1) = point_conv(1);
        // Don't change obj.pos_covariance
        // 对跟踪的轨迹进行时间及空间同步
        common::RingBuffer<TrjPointInfos,
            MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it =
            obj.traced_trj_[0].begin();
        common::RingBuffer<TrjPointInfos,
            MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it_end =
            obj.traced_trj_[0].end();

        for (; trj_it != trj_it_end; ++trj_it) {
          TrjPointInfos* pt = trj_it.current();
          pt->seq_num_--;
          pt->relative_time_ms_ -= time_elapsed_ms;

          point_conv(0) = pt->x_;
          point_conv(1) = pt->y_;

          common::TransformVert_2D(mat_conv, &point_conv);

          pt->x_ = point_conv(0);
          pt->y_ = point_conv(1);
        }
        trj_it = obj.traced_trj_[1].begin();
        trj_it_end = obj.traced_trj_[1].end();
        for (; trj_it != trj_it_end; ++trj_it) {
          TrjPointInfos* pt = trj_it.current();
          pt->seq_num_--;
          pt->relative_time_ms_ -= time_elapsed_ms;

          point_conv(0) = pt->x_;
          point_conv(1) = pt->y_;

          common::TransformVert_2D(mat_conv, &point_conv);

          pt->x_ = point_conv(0);
          pt->y_ = point_conv(1);
        }
    }

}

void ObjsTracker::AddPartnerRelationship(const MatchingParams &param, const ObjIndex &src_obj_idx,
                                         const ObjInfosBase &src_obj, const common::StaticVector<MatchingInfos, MAX_PARTNER_NUM> &matched_obj_idxs)
{
    if (!src_obj_idx.IsValid()) {
        LOG_ERR << "[Error]Invalid source obj index.";
        return;
    }
    // 在临近的障碍物中，查找属性相匹配的障碍物
    Int32_t num_objs = matched_obj_idxs.Size();
    for (Int32_t j = 0; j < num_objs; ++j) {
      const MatchingInfos match_info = matched_obj_idxs[j];
      if (!match_info.candidate_idx_.IsValid()) {
        LOG_ERR << "[Error]Invalid candidate obj index.";
        continue;
      }

      ObjsTracked& tracked_obj =
          tracked_objects_.objects_[match_info.candidate_idx_.obj_idx_];
      SensorsRef& sen_ref =
          tracked_obj.sensor_ref_[match_info.candidate_idx_.sensor_idx_];

      Float32_t position_diff = common::com_sqrt(
            common::Square(sen_ref.pos_[0].x_ - src_obj.pos_.x_) +
            common::Square(sen_ref.pos_[0].y_ - src_obj.pos_.y_));
      Int32_t min_dist_point_index = 0;
      for (Int32_t k = 1; k < SensorsRef::MAX_POS_POINT_NUM; ++k) {
        Float32_t dist = common::com_sqrt(
              common::Square(sen_ref.pos_[k].x_ - src_obj.pos_.x_) +
              common::Square(sen_ref.pos_[k].y_ - src_obj.pos_.y_));
        if (dist < position_diff) {
          position_diff = dist;
          min_dist_point_index = k;
        }
      }

      Float32_t x_diff = sen_ref.pos_[min_dist_point_index].x_ - src_obj.pos_.x_;
      Float32_t y_diff = sen_ref.pos_[min_dist_point_index].y_ - src_obj.pos_.y_;
      Float32_t abs_x_diff = common::com_abs(x_diff);
      Float32_t abs_y_diff = common::com_abs(y_diff);

      //Float32_t abs_range_diff = common::com_abs(
      //      src_obj.pos.range - tracked_obj.sensor_ref[sen_ref_idx].
      //      pos[min_dist_point_index].range);
      //Float32_t abs_angle_diff = common::com_abs(
      //      common::AngleDiff(src_obj.pos.angle, tracked_obj.
      //                        sensor_ref[sen_ref_idx].pos[
      //                        min_dist_point_index].angle));

      Float32_t abs_vx_diff = common::com_abs(src_obj.v_x_ - sen_ref.v_x_);
      Float32_t abs_vy_diff = common::com_abs(src_obj.v_y_ - sen_ref.v_y_);

      if(true == param.match_with_velocity_)
      {

      }
      else
      {//去掉速度影响，让速度值不参与匹配过程中其他各种值的计算
        abs_vx_diff = 0;
        abs_vy_diff = 0;
      }
      
      if (param.condition_.valid_ && !match_info.always_match_) {
              if ((abs_x_diff > param.condition_.max_x_diff_) ||
                  (abs_y_diff > param.condition_.max_y_diff_) ||
                  (abs_vx_diff > param.condition_.max_vx_diff_) ||
                  (abs_vy_diff > param.condition_.max_vy_diff_)) {
          continue;
        }
      }


      Int32_t cost = common::com_round( // COEFFICIENT0_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP
            /* 位置偏差小，代价小 */
            5.0F * position_diff +
            /* 离自车近，代价小 */
            (x_diff < -0.1F ? -10 : 0) +
            /* 速度偏差小，代价小 */
            1.0F * 3.6F * abs_vx_diff +
            /* 跟踪的次数多，代价小 */
            (tracked_obj.matched_ > 0 ? -5 : 0) +
            /* 最近刚跟踪上，代价小 */
            (((tracked_obj.duration_ - tracked_obj.age_) < 2) ? -5 : 0) +
            /* 之前刚匹配上，代价小 */
            (tracked_obj.sensor_ref_[src_obj_idx.sensor_idx_].
                FindObjId(src_obj.raw_obj_id_) >= 0 ? -5 : 0)
          );

  #if  ENABLE_OTHER_DEBUG_PRINTF
      std::cout << "          ^obj[" << tracked_obj_idx
                << "] position_diff=" << position_diff
                //<< ", abs_range_diff=" << abs_range_diff
                //<< ", abs_angle_diff=" << common::com_rad2deg(abs_angle_diff)
                << ", abs_vx_diff=" << abs_vx_diff*3.6F
                << ", abs_vy_diff=" << abs_vy_diff*3.6F
                << ", cost=" << cost
                << ", 添加到伙伴关系."
                << std::endl;
  #endif
      if (src_obj_idx.obj_idx_ < 0) {
        LOG_ERR << "[Error]Invalid tracked obj index.";
        continue;
      }

      // 添加障碍物的关联关系
      Int32_t src_list_idx = GetListIdBySensorId(src_obj_idx.sensor_idx_);
      COM_CHECK(src_list_idx >= 0);
      ObjsAssociation* ass =
          tracked_obj.objs_ass_.objs_ass_list_[src_list_idx].AddToBack(obj_ass_pool_);
      if (Nullptr_t != ass) {
        // 将关联的OBJ信息添加到tracked_obj中
        ass->mismatch_ = false;
        ass->tar_obj_index_ = src_obj_idx;
        ass->cost_ = cost;
        ass->dist_to_tar_obj_ = position_diff;

        // 将关联的tracked_obj信息添加到OBJ中
        ass = tracked_objects_.objects_[src_obj_idx.obj_idx_].objs_ass_.objs_ass_list_[LIST_ID_TRACKED].AddToBack(obj_ass_pool_);
        if (Nullptr_t != ass) {
          ass->mismatch_ = false;
          ass->tar_obj_index_ = match_info.candidate_idx_;
          ass->cost_ = cost;
          ass->dist_to_tar_obj_ = position_diff;
        } else {
          tracked_obj.objs_ass_.objs_ass_list_[src_list_idx].PopBack(obj_ass_pool_);
          LOG_ERR << "[Error]Failed to add ass, storage is full.";
        }
      } else {
        LOG_ERR << "[Error]Failed to add ass, storage is full.";
      }
    }

}

void ObjsTracker::SortObjAssoList(const Int32_t list_id)
{
#if ENABLE_OBJ_TRACKER_TRACE
  printf(">>@@@ 将关联的障碍物信息按cost大小来排序:\n");
#endif

  Int32_t tracked_obj_num = tracked_objects_.objects_.Size();
  // 将关联的障碍物信息按cost大小来排序
  FuncCmpObjAssoCost ass_cmp_func;
  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    ObjsTracked& tracked_obj = tracked_objects_.objects_[i];

    if (list_id == tracked_obj.list_id_) {
      if (!tracked_obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Empty()) {
        tracked_obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Sort(
              obj_ass_pool_, ass_cmp_func);
      }
    } else if (LIST_ID_TRACKED == tracked_obj.list_id_) {
      if (!tracked_obj.objs_ass_.objs_ass_list_[list_id].Empty()) {
        tracked_obj.objs_ass_.objs_ass_list_[list_id].Sort(
              obj_ass_pool_, ass_cmp_func);
      }
    } else {
      // nothing to do
    }

#if ENABLE_OBJ_TRACKER_TRACE
    printf("      障碍物关联信息:\n");
    if (!tracked_obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Empty() ||
        (!tracked_obj.objs_ass_.objs_ass_list_[list_id].Empty())) {
      ShowTrackedObject(i, tracked_obj);
    }
#endif
  }
}

void ObjsTracker::SetBestMatchedObj(const Int32_t list_id)
{
#if ENABLE_OBJ_TRACKER_TRACE
  printf(">>@@@ 查找最佳的匹配:\n");
#endif

  Int32_t tracked_obj_num = tracked_objects_.objects_.Size();
  // 查找最佳的匹配
  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    ObjsTracked& obj_self = tracked_objects_.objects_[i];

    if (list_id != obj_self.list_id_) {
      continue;
    }

    if (obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Empty()) {
      continue;
    }

#if ENABLE_OBJ_TRACKER_TRACE
    printf("        obj[%d]:\n", i);
#endif

    bool mismatch = true;
    FuncCmpObjAssoObjIndex func_cmp_obj_index(i);
    ObjAssoListType::iterator it =
        obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].begin(obj_ass_pool_);
    ObjAssoListType::iterator it_end =
        obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].end(obj_ass_pool_);
    for (; it != it_end; ++it) {
      ObjsAssociation* obj_ass = it.current();
      // 跟踪列表中与其相匹配的tracked_obj
      ObjsTracked& obj_partner =
          tracked_objects_.objects_[obj_ass->tar_obj_index_.obj_idx_];
      ObjsAssociation* first_ass_partner =
          obj_partner.objs_ass_.objs_ass_list_[list_id].Front(obj_ass_pool_);
      if (first_ass_partner->tar_obj_index_.obj_idx_ != i) {
        // tracked_obj与另一个obj能更好匹配，需要再找其它的
#if ENABLE_OBJ_TRACKER_TRACE
        printf("          不能与obj[%d]最佳匹配，需要再找其它的\n", obj_ass->tar_obj_index_.obj_idx_);
#endif
        // 将关联关系删除
        obj_ass->mismatch_ = true;
        ObjsAssociation* ass = obj_partner.objs_ass_.objs_ass_list_[list_id].FindData(
              obj_ass_pool_, func_cmp_obj_index);
        if (Nullptr_t != ass) {
          // 将伙伴的关联关系也删除
          ass->mismatch_ = true;
        } else {
          LOG_ERR << "[Error]Unmatched obj association.";
        }
      } else {
        // tracked_obj与自身obj能匹配上
#if ENABLE_OBJ_TRACKER_TRACE
        printf("          与obj[%d]最佳匹配\n", obj_ass->tar_obj_index_.obj_idx_);
#endif
        ObjAssoListType::iterator it2 =
            obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].begin(obj_ass_pool_);
        for (; it2 != it_end; ++it2) {
          // 删除其它的可匹配的tracked_obj的关联关系
          ObjsTracked& obj = tracked_objects_.objects_[it2->tar_obj_index_.obj_idx_];
          ObjsAssociation* ass = obj.objs_ass_.objs_ass_list_[list_id].FindData(
                obj_ass_pool_, func_cmp_obj_index);
          if (it2 == it) {
            it2->mismatch_ = false;
            if (Nullptr_t != ass) {
              ass->mismatch_ = false;
            } else {
              LOG_ERR << "[Error]Unmatched obj association.";
            }
          } else {
            // 将关联关系删除
            it2->mismatch_ = true;
            if (Nullptr_t != ass) {
              // 将伙伴的关联关系也删除
              ass->mismatch_ = true;
            } else {
              LOG_ERR << "[Error]Unmatched obj association.";
            }
          }
        }
        mismatch = false;
        break;
      }
    }
    if (mismatch) {
      // 没有最佳匹配
#if ENABLE_OBJ_TRACKER_TRACE
      printf("          没有最佳匹配\n");
#endif
      // 将匹配信息添加到最可能的跟踪上的obj中(次优匹配)
      ObjsAssociation* first_ass =
          obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Front(obj_ass_pool_);
      ObjsTracked& first_obj_partner =
          tracked_objects_.objects_[first_ass->tar_obj_index_.obj_idx_];
      ObjsAssociation* ass_partner =
          first_obj_partner.objs_ass_.objs_ass_list_[list_id].FindData(
            obj_ass_pool_, func_cmp_obj_index);
      if (Nullptr_t != ass_partner) {
        // 添加伙伴关系
#if ENABLE_OBJ_TRACKER_TRACE
        printf("            添加伙伴关系to obj[%d]\n", first_ass->tar_obj_index_.obj_idx_);
#endif
        first_ass->mismatch_ = false;
        ass_partner->mismatch_ = false;
      }
    }
  }

#if ENABLE_OBJ_TRACKER_TRACE
  printf("      障碍物关联信息:\n");
  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    ObjsTracked& tracked_obj = tracked_objects_.objects_[i];
    if (!tracked_obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Empty() ||
        (!tracked_obj.objs_ass_.objs_ass_list_[list_id].Empty())) {
      ShowTrackedObject(i, tracked_obj);
    }
  }
#endif

}

void ObjsTracker::UpdateMatchedObj(const Int32_t sen_id, const Int32_t list_id, const MatchingParams &param)
{
#if ENABLE_OBJ_TRACKER_TRACE
  printf(">>@@@ 更新跟踪列表(融合):\n");
#endif


  // if(sen_id==SENSOR_ID_LEFT_BACKWARD_CAM||
  //     sen_id==SENSOR_ID_RIGHT_BACKWARD_CAM||
  //     sen_id==SENSOR_ID_MAIN_FORWARD_CAM){

  //       printf("===========UpdateMatchedObj===== src_obj.sensor_type_ %d\n",sen_id);

  // }

  Int32_t tracked_obj_num = tracked_objects_.objects_.Size();

  // 更新跟踪列表
  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    ObjsTracked& obj_self = tracked_objects_.objects_[i];
    SensorsRef& obj_self_sen = obj_self.sensor_ref_[sen_id];

    if (LIST_ID_TRACKED == obj_self.list_id_) {
#if ENABLE_OBJ_TRACKER_TRACE
      printf("        obj[%d]:\n", i);
#endif
      // 更新跟踪上的obj信息
      if (obj_self.objs_ass_.objs_ass_list_[list_id].Empty()) {
        // 没有与之匹配的从传感器获得的obj
#if ENABLE_OBJ_TRACKER_TRACE
        printf("          没有伙伴\n");
#endif
      
#if ENABLE_CAM_DATA_TRACE
      if(sen_id==SENSOR_ID_LEFT_BACKWARD_CAM||
      sen_id==SENSOR_ID_RIGHT_BACKWARD_CAM||
      sen_id==SENSOR_ID_MAIN_FORWARD_CAM){

        //printf("%d 没有伙伴\n",sen_id);

        //printf("life_ : %d\n",obj_self.life_);

      }

#endif


        continue;
      }

      bool is_first_partner = false;
      ObjAssoListType::iterator it =
          obj_self.objs_ass_.objs_ass_list_[list_id].begin(obj_ass_pool_);
      ObjAssoListType::iterator it_end =
          obj_self.objs_ass_.objs_ass_list_[list_id].end(obj_ass_pool_);
      for (; it != it_end; ++it) {
        ObjsAssociation* ass = it.current();

        if (ass->mismatch_) {
          continue;
        }

        //有与之匹配的从传感器获得的obj,更新跟踪上的obj信息
        ObjsTracked& obj_partner =
            tracked_objects_.objects_[ass->tar_obj_index_.obj_idx_];
        SensorsRef& obj_partner_sen = obj_partner.sensor_ref_[sen_id];

        if (!is_first_partner) {
          is_first_partner = true;
          // obj_self.erase = false;

#if ENABLE_OBJ_TRACKER_TRACE
          printf("          obj[%d] 与 obj[%d] 是最佳伙伴\n",
                 i, ass->tar_obj_index_.obj_idx_);
#endif

          Float32_t x_estimation = 0.0F;
          Float32_t y_estimation = 0.0F;
          Float32_t vx_estimation = 0.0F;
          Float32_t vy_estimation = 0.0F;
          if (param.using_est_pos_) {//如果使用估计的位置
            EstimateObjPos(obj_self.duration_,
                           obj_self_sen,
                           obj_partner_sen,
                           &x_estimation, &y_estimation,
                           &vx_estimation, &vy_estimation);
          } else {
            x_estimation = obj_partner_sen.pos_[0].x_;
            y_estimation = obj_partner_sen.pos_[0].y_;
            vx_estimation = obj_partner_sen.v_x_;
            vy_estimation = obj_partner_sen.v_y_;
          }

#if ENABLE_OBJ_TRACKER_TRACE
          printf("            更新位置: From{x=%0.1f,y=%0.1f"
                 ",vx=%0.1fkm/h,vy=%0.1fkm/h}"
                 " To {x_est=%0.1f,y_est=%0.1f"
                 ",vx_est=%0.1fkm/h,vy_est=%0.1fkm/h}"
                 "\n",
                 obj_self_sen.pos_[0].x_,
                 obj_self_sen.pos_[0].y_,
                 obj_self_sen.v_x_*3.6F,
                 obj_self_sen.v_y_*3.6F,
                 x_estimation, y_estimation,
                 vx_estimation*3.6F, vy_estimation*3.6F);
#endif

          obj_self_sen.valid_ = true;

          obj_self_sen.sensor_type_ = obj_partner_sen.sensor_type_;
          obj_self_sen.obj_type_ = obj_partner_sen.obj_type_;
          obj_self_sen.width_ = obj_partner_sen.width_;
          obj_self_sen.length_ = obj_partner_sen.length_;
          

          obj_self_sen.pos_[0].x_ = x_estimation;
          obj_self_sen.pos_[0].y_ = y_estimation;
          obj_self_sen.pos_[1].x_ = obj_partner_sen.pos_[0].x_;
          obj_self_sen.pos_[1].y_ = obj_partner_sen.pos_[0].y_;
          obj_self_sen.pos_[2].x_ = x_estimation;
          obj_self_sen.pos_[2].y_ = y_estimation;
          obj_self_sen.pos_[3].x_ = obj_partner_sen.pos_[0].x_;
          obj_self_sen.pos_[3].y_ = obj_partner_sen.pos_[0].y_;
          obj_self_sen.v_x_ = vx_estimation;
          obj_self_sen.v_y_ = vy_estimation;
          obj_self_sen.age_ = obj_self.duration_;

          //加速度融合输出
          obj_self_sen.a_x_ = obj_partner_sen.a_x_;
          obj_self_sen.a_y_ = obj_partner_sen.a_y_;

          Int32_t matched_obj_id_idx =
              obj_self_sen.FindObjId(obj_partner_sen.obj_id_[0].id_);
          if (obj_self.age_ < obj_self.duration_ && matched_obj_id_idx >=0) {
            // 不重复更新状态信息
            obj_self.life_ += 1;

            if (obj_self.life_ > MAX_TRACKED_OBJ_LIFE) {
              obj_self.life_ = MAX_TRACKED_OBJ_LIFE;
              //obj_self.life_--;
            }
            obj_self.age_ = obj_self.duration_;
            obj_self.matched_++;
            if (obj_self.matched_ > MAX_TRACKED_OBJ_MATCHED_NUM) {
              obj_self.matched_ = MAX_TRACKED_OBJ_MATCHED_NUM;
            }
          }

          if (obj_partner.raw_obj_idx_ >= 0) {
            if (param.enable_add_mul_obj_id_) {
              if (obj_self_sen.AddPartnerObjId(
                    obj_partner_sen.obj_id_[0].id_, obj_self.duration_) < 0) {
                obj_self_sen.obj_id_[1].id_ = obj_partner_sen.obj_id_[0].id_;
                obj_self_sen.obj_id_[1].age_ = obj_self.duration_;
              }
            } else {
              obj_self_sen.obj_id_[0].id_ = obj_partner_sen.obj_id_[0].id_;
              obj_self_sen.obj_id_[0].age_ = obj_self.duration_;
            }
          } else {
            LOG_ERR << "Invalid raw obj index.";
          }
        } else {
          // 次优伙伴
#if ENABLE_OBJ_TRACKER_TRACE
          printf("          obj[%d] 与 obj[%d] 是次优伙伴\n",
                 i, ass->tar_obj_index_.obj_idx_);
#endif
          if (param.enable_sub_optimal_match_) {
#if ENABLE_OBJ_TRACKER_TRACE
            printf("            添加跟踪obj_id信息\n");
#endif
            // 将匹配信息添加到最可能的跟踪上的obj中(次优匹配)
            if (obj_self_sen.AddPartnerObjId(
                  obj_partner_sen.obj_id_[0].id_, obj_self.duration_) < 0) {
  #if ENABLE_OBJ_TRACKER_TRACE
              printf("              添加失败,保留传感器获得的obj\n");
  #endif
              // 添加失败,保留传感器获得的obj
              FuncCmpObjAssoObjIndex func_cmp_obj_index(i);
              ObjsAssociation* ass_partner =
                  obj_partner.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].FindData(
                    obj_ass_pool_, func_cmp_obj_index);
              if (Nullptr_t != ass_partner) {
                ass->mismatch_ = true;
                ass_partner->mismatch_ = true;
              }
              obj_partner.erase_flag_ = false;
            } else {
  #if ENABLE_OBJ_TRACKER_TRACE
              printf("              添加成功,删除传感器获得的obj\n");
  #endif
              // 添加成功,删除传感器获得的obj
              obj_partner.erase_flag_ = true;
            }
          } else {
#if ENABLE_OBJ_TRACKER_TRACE
            printf("            若存在伙伴关系，则移除它\n");
#endif
            FuncCmpObjAssoObjIndex func_cmp_obj_index(i);
            ObjsAssociation* ass_partner =
                obj_partner.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].FindData(
                  obj_ass_pool_, func_cmp_obj_index);
            if (Nullptr_t != ass_partner) {
              ass->mismatch_ = true;
              ass_partner->mismatch_ = true;
            }
            obj_partner.erase_flag_ = false;
          }
        }
      }
    }
  }

#if ENABLE_OBJ_TRACKER_TRACE
  printf("      障碍物关联信息:\n");
  for (Int32_t i = 0; i < tracked_obj_num; ++i) {
    ObjsTracked& tracked_obj = tracked_objects_.objects_[i];
    if (!tracked_obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Empty() ||
        (!tracked_obj.objs_ass_.objs_ass_list_[list_id].Empty())) {
      ShowTrackedObject(i, tracked_obj);
    }
  }
#endif
}

void ObjsTracker::UpdateUnmatchedObj(const Int32_t sen_id, const Int32_t list_id, const MatchingParams &param)
{
    // 处理未匹配上的obj
  #if ENABLE_OBJ_TRACKER_TRACE
    printf(">>@@@ 处理未匹配上的obj:\n");
  #endif
    common::StaticVector<common::DataPair<Int32_t, Int32_t>, 64> matched_objs;// VECTOR_DATA_NUM_OF_MATCHED_OBJS_FOR_UPDATE_UNMATICHED_OBJ
    common::StaticVector<Int32_t, 64> unmatched_objs; //VECTOR_DATA_NUM_OF_UNMATCHED_OBJS_FOR_UPDATE_UNMATICHED_OBJ
    FuncCmpObjAssObjMismatchFlag func_cmp_mismatch_flag(false);
    Int32_t tracked_obj_num = tracked_objects_.objects_.Size();
    for (Int32_t i = 0; i < tracked_obj_num; ++i) {
      ObjsTracked& obj_self = tracked_objects_.objects_[i];
      /* 从新加入的传感器数据中（已经放入tracked_objects_）中找出从物理上看没有匹配上的数据，如果允许然后再通过
        MatchObjAgainByIdWhenUnmatched取寻找
      */
      if (obj_self.erase_flag_ == true)
      {
        continue;
      }
      if (list_id == obj_self.list_id_) { 
  #if ENABLE_OBJ_TRACKER_TRACE
        printf("        obj[%d]:\n", i);
  #endif
        bool matched = true;
        // 若有可匹配的跟踪上的obj，则删除自身
        if (obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].Empty()) {
          // 没有可匹配的跟踪上的obj
  #if ENABLE_OBJ_TRACKER_TRACE
          printf("          没有伙伴\n");
  #endif
          matched = false;
        } else {
          ObjsAssociation* ass = obj_self.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].FindData(
                obj_ass_pool_, func_cmp_mismatch_flag);
          if (Nullptr_t == ass) {
            // 没有最佳/次优伙伴
  #if ENABLE_OBJ_TRACKER_TRACE
            printf("          没有最佳/次优伙伴\n");
  #endif
            matched = false;
          } else {
            matched_objs.PushBack(common::DataPair<Int32_t, Int32_t>(
                                    i, ass->tar_obj_index_.obj_idx_));
          }
        }

        if (matched) {
          // 有最佳匹配的跟踪上的obj，则删除自身
          obj_self.erase_flag_ = true;
  #if ENABLE_OBJ_TRACKER_TRACE
          printf("          有最佳/次优伙伴, 删除自身\n");
  #endif
        } else {
          // 若无,则保留自身
          unmatched_objs.PushBack(i);
        }
      }
    }

    if (param.match_again_by_id_if_unmatched_) {
      MatchObjAgainByIdWhenUnmatched(sen_id, param, unmatched_objs);
    }

    // 删除重复的匹配项目
    Int32_t matched_objs_num = matched_objs.Size();
    if (matched_objs_num > 0) {
      for (Int32_t i = 0; i < tracked_obj_num; ++i) {
        ObjsTracked& tracked_obj = tracked_objects_.objects_[i];

        if (LIST_ID_TRACKED != tracked_obj.list_id_) {
          continue;
        }

        if (!tracked_obj.sensor_ref_[sen_id].valid_) {
          continue;
        }

        if (tracked_obj.erase_flag_) {
          continue;
        }

        for (Int32_t j = 0; j < matched_objs_num; ++j) {
          ObjsTracked& obj_check = tracked_objects_.objects_[matched_objs[j].first];
          if (i != matched_objs[j].second) {
            if (tracked_obj.sensor_ref_[sen_id].FindObjId(
                  obj_check.sensor_ref_[sen_id].obj_id_[0].id_) >= 0) {
              if (tracked_obj.IsOhterSensorValid(sen_id)) {
                tracked_obj.sensor_ref_[sen_id].Clear();
              } else {
                tracked_obj.erase_flag_ = true;
              }
            }
          }
        }
      }
    }
}
/**
 * @brief 当通过ID匹配上之后，对sen_id类型的传感器数据partner_obj_sen，判断其是否与tracked_obj中非sen_id的传感器数据物理上匹配，
 * 
 * 
 * @param tracked_obj 
 * @param tracked_obj_sen 
 * @param partner_obj_sen 
 * @param sen_id 
 * @return true 
 * @return false 
 */
bool ObjsTracker::IsValidMatchedForObjPartnerAndObjsTracked(ObjsTracked& tracked_obj, SensorsRef& partner_obj_sen, const MatchingParams &param, const Int32_t sen_id)
{
  // bool other_sensor_valid = false;
  for (Int32_t i = 0; i < SensorID::SENSOR_ID_MAX; ++i) {
      if (i == sen_id) {
          continue;
      }
      SensorsRef & sen = tracked_obj.sensor_ref_[i];
      if (sen.valid_)
      {
        if(common::com_abs(partner_obj_sen.v_x_ - sen.v_x_) < param.max_vx_diff_for_matched_again_by_id &&
           common::com_abs(partner_obj_sen.v_y_ - sen.v_y_) < param.max_vy_diff_for_matched_again_by_id &&
           common::com_abs(partner_obj_sen.pos_[0].x_ - sen.pos_[0].x_) < param.max_x_diff_for_matched_again_by_id &&
           common::com_abs(partner_obj_sen.pos_[0].y_ - sen.pos_[0].y_) < param.max_y_diff_for_matched_again_by_id 
          ){
            continue;
          }
          else
          {
            return false;
          }
      }
  }
  return true;
}

void ObjsTracker::MatchObjAgainByIdWhenUnmatched(const Int32_t sen_id, const MatchingParams &param, const common::StaticVector<Int32_t, 64> &unmatched_objs)
{
    Int32_t tracked_obj_num = tracked_objects_.objects_.Size();
    Int32_t unmatched_objs_num = unmatched_objs.Size();

    if (unmatched_objs_num < 1) {
      return;
    }

    // std::cout << "unmatched_objs_num=" << unmatched_objs_num << std::endl;

    // 通过ID匹配
    for (Int32_t i = 0; i < tracked_obj_num; ++i) {
      ObjsTracked& tracked_obj = tracked_objects_.objects_[i];
      SensorsRef& tracked_obj_sen = tracked_obj.sensor_ref_[sen_id];

      if (LIST_ID_TRACKED != tracked_obj.list_id_) {
        continue;
      }

      if (!tracked_obj_sen.valid_) {
        continue;
      }

      if (tracked_obj.erase_flag_) {
        continue;
      }

      Int32_t obj_partner_idx = -1;
      for (Int32_t j = 0; j < unmatched_objs_num; ++j) {
        obj_partner_idx = unmatched_objs[j];
        ObjsTracked& obj_partner = tracked_objects_.objects_[obj_partner_idx];
        SensorsRef& partner_obj_sen = obj_partner.sensor_ref_[sen_id];
        Int32_t partner_obj_id_idx =
            tracked_obj_sen.FindObjId(partner_obj_sen.obj_id_[0].id_);
        if (partner_obj_id_idx < 0) {
          continue;
        }
        if (obj_partner.erase_flag_) {
          if (tracked_obj.IsOhterSensorValid(sen_id)) {
            tracked_obj_sen.Clear();
          } else {
            tracked_obj.erase_flag_ = true;
          }
          continue;
        }

        if (!IsValidMatchedForObjPartnerAndObjsTracked(tracked_obj,partner_obj_sen,param,sen_id))
        {
          continue;
        }

  #if ENABLE_OBJ_TRACKER_TRACE
        printf("            obj[%d] 与 obj[%d] 通过ID[%d]匹配.\n",
               i, obj_partner_idx,
               partner_obj_sen.obj_id_[0].id_);
  #endif

        // dist, age, v_diff ?
  #if 1
        Float32_t position_diff = common::com_sqrt(
              common::Square(tracked_obj_sen.pos_[0].x_ -
                             partner_obj_sen.pos_[0].x_) +
              common::Square(tracked_obj_sen.pos_[0].y_ -
                             partner_obj_sen.pos_[0].y_));
        Int32_t min_dist_point_index = 0;
        for (Int32_t k = 1; k < SensorsRef::MAX_POS_POINT_NUM; ++k) {
          Float32_t dist = common::com_sqrt(
                common::Square(tracked_obj_sen.pos_[k].x_ -
                               partner_obj_sen.pos_[k].x_) +
                common::Square(tracked_obj_sen.pos_[k].y_ -
                               partner_obj_sen.pos_[k].y_));
          /**
           * @brief 获取单类传感器数据中的最小值
           */
          if (dist < position_diff) {
            position_diff = dist;
            min_dist_point_index = k;
          }
        }

        Float32_t abs_x_diff = common::com_abs(
              tracked_obj_sen.pos_[min_dist_point_index].x_ -
              partner_obj_sen.pos_[0].x_);
        Float32_t abs_y_diff = common::com_abs(
              tracked_obj_sen.pos_[min_dist_point_index].y_ -
              partner_obj_sen.pos_[0].y_);
  #if 0
        Float32_t abs_range_diff = common::com_abs(
              partner_obj_sen.pos_[0].range_ -
            tracked_obj_sen.pos_[
            min_dist_point_index].range_);
        Float32_t abs_angle_diff = common::com_abs(
              common::AngleDiff(
                partner_obj_sen.pos_[0].angle_,
              tracked_obj_sen.pos_[
              min_dist_point_index].angle_));
  #endif
        Float32_t abs_vx_diff = common::com_abs(
              partner_obj_sen.v_x_ - tracked_obj_sen.v_x_);
        Float32_t abs_vy_diff = common::com_abs(
              partner_obj_sen.v_y_ - tracked_obj_sen.v_y_);

        /// TODO: 放入参数配置里面
        Float32_t obj_v = common::com_abs(partner_obj_sen.v_x_);
        Float32_t max_x_diff =
            1.0F + partner_obj_sen.pos_[0].range_*0.05F + obj_v*0.2F; //COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_WHEN_UNMATCHED
        Float32_t max_y_diff =
            0.2F + partner_obj_sen.pos_[0].range_*0.01F + obj_v*0.1F; //COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_WHEN_UNMATCHED
        Float32_t max_vx_diff = 10.0F/3.6F; //VALUE_OF_MAX_VX_DIFF_FORMULA_WHEN_UNMATCHED
        Float32_t max_vy_diff = 10.0F/3.6F; //VALUE_OF_MAX_VY_DIFF_FORMULA_WHEN_UNMATCHED

        if ((abs_x_diff > max_x_diff) || (abs_y_diff > max_y_diff) ||
            (abs_vx_diff > max_vx_diff) || (abs_vy_diff > max_vy_diff)) {
  #if ENABLE_OBJ_TRACKER_TRACE
          std::cout << "obj_v=" << obj_v*3.6F
                    << ", abs_x_diff=" << abs_x_diff
                    << ", max_x_diff=" << max_x_diff
                    << ", abs_y_diff=" << abs_y_diff
                    << ", max_y_diff=" << max_y_diff
                    << ", abs_vx_diff=" << abs_vx_diff*3.6F
                    << ", max_vx_diff=" << max_vx_diff*3.6F
                    << ", abs_vy_diff=" << abs_vy_diff*3.6F
                    << ", max_vy_diff=" << max_vy_diff*3.6F
                    << std::endl;
                printf("            ^位置或速度偏差过大，不匹配.\n");
  #endif
          continue;
        }
  #endif

        Float32_t x_estimation = 0.0F;
        Float32_t y_estimation = 0.0F;
        Float32_t vx_estimation = 0.0F;
        Float32_t vy_estimation = 0.0F;
        EstimateObjPos(tracked_obj.duration_,
                       tracked_obj_sen, partner_obj_sen,
                       &x_estimation, &y_estimation,
                       &vx_estimation, &vy_estimation);

  #if ENABLE_OBJ_TRACKER_TRACE
        printf("            更新位置: From{x=%0.1f,y=%0.1f"
               ",vx=%0.1fkm/h,vy=%0.1fkm/h}"
               " To {x_est=%0.1f,y_est=%0.1f"
               ",vx_est=%0.1fkm/h,vy_est=%0.1fkm/h}"
               "\n",
               tracked_obj_sen.pos_[0].x_,
            tracked_obj_sen.pos_[0].y_,
            tracked_obj_sen.v_x_*3.6F,
            tracked_obj_sen.v_y_*3.6F,
            x_estimation, y_estimation,
            vx_estimation*3.6F, vy_estimation*3.6F);
  #endif

        tracked_obj_sen.valid_ = true;

        tracked_obj_sen.sensor_type_ = partner_obj_sen.sensor_type_;
        tracked_obj_sen.obj_type_ = partner_obj_sen.obj_type_;
        tracked_obj_sen.width_ = partner_obj_sen.width_;
        tracked_obj_sen.length_ = partner_obj_sen.length_;

        tracked_obj_sen.pos_[0].x_ = x_estimation; 
        tracked_obj_sen.pos_[0].y_ = y_estimation;
        tracked_obj_sen.pos_[1].x_ = partner_obj_sen.pos_[0].x_;
        tracked_obj_sen.pos_[1].y_ = partner_obj_sen.pos_[0].y_;
        tracked_obj_sen.pos_[2].x_ = x_estimation;
        tracked_obj_sen.pos_[2].y_ = y_estimation;
        tracked_obj_sen.pos_[3].x_ = partner_obj_sen.pos_[0].x_;
        tracked_obj_sen.pos_[3].y_ = partner_obj_sen.pos_[0].y_;
        tracked_obj_sen.v_x_ = vx_estimation;
        tracked_obj_sen.v_y_ = vy_estimation;
        tracked_obj_sen.age_ = tracked_obj.duration_;
        if (tracked_obj.age_ < tracked_obj.duration_) {
          //不重复更新状态信息
          tracked_obj.life_ += 1;

          if (tracked_obj.life_ > MAX_TRACKED_OBJ_LIFE) {
            tracked_obj.life_ = MAX_TRACKED_OBJ_LIFE;
          }
          tracked_obj.age_ = tracked_obj.duration_;
          tracked_obj.matched_++;
          if (tracked_obj.matched_ > MAX_TRACKED_OBJ_MATCHED_NUM) {
            tracked_obj.matched_ = MAX_TRACKED_OBJ_MATCHED_NUM;
          }
        }
        tracked_obj_sen.AddPartnerObjId(
              partner_obj_sen.obj_id_[0].id_, tracked_obj.duration_);

        // 已经匹配上，删除它
        obj_partner.erase_flag_ = true;
      }
    }
}

void ObjsTracker::EstimateObjPos(const Int32_t duration, const SensorsRef &tracked_obj, const SensorsRef &sensor_obj, Float32_t *x_est, Float32_t *y_est, Float32_t *vx_est, Float32_t *vy_est)
{
    if (!tracked_obj.valid_ || ((duration - tracked_obj.age_) > 3)) {
      *x_est = sensor_obj.pos_[0].x_;
      *y_est = sensor_obj.pos_[0].y_;
      *vx_est = sensor_obj.v_x_;
      *vy_est = sensor_obj.v_y_;

  #if ENABLE_OBJ_TRACKER_TRACE
      printf("            tracked_obj is invalid,"
             " x_est=%0.1f, y_est=%0.1f"
             " vx_est=%0.1fkm/h, vy_est=%0.1fkm/h"
             "\n",
             *x_est, *y_est,
             *vx_est*3.6F, *vy_est*3.6F);
  #endif
      return;
    }

    Float32_t dist_sum = 0.0F;
    Float32_t dist[SensorsRef::MAX_POS_POINT_NUM] = { 0.0F };
    Float32_t ratio[SensorsRef::MAX_POS_POINT_NUM] = { 0.0F };
    for (Int32_t j = 0; j < SensorsRef::MAX_POS_POINT_NUM; ++j) {
      dist[j] = common::com_sqrt(
            common::Square(tracked_obj.pos_[j].x_ - sensor_obj.pos_[0].x_) +
            common::Square(tracked_obj.pos_[j].y_ - sensor_obj.pos_[0].y_));
      dist_sum += dist[j];
    }
    if (dist_sum > 0.01F) {
      Float32_t ratio_sum = 0.0F;
      for (Int32_t j = 0; j < SensorsRef::MAX_POS_POINT_NUM; ++j) {
        ratio[j] = (dist_sum - dist[j]) / dist_sum;
        ratio_sum += ratio[j];
      }
      for (Int32_t j = 0; j < SensorsRef::MAX_POS_POINT_NUM; ++j) {
        ratio[j] /= ratio_sum;
      }
    } else {
      for (Int32_t j = 0; j < SensorsRef::MAX_POS_POINT_NUM; ++j) {
        ratio[j] = 1.0F / (Float32_t)SensorsRef::MAX_POS_POINT_NUM;
      }
    }

    Float32_t pos_change_ratio = 0.8F;
    *x_est = 0.0F;
    *y_est = 0.0F;
    for (Int32_t j = 0; j < SensorsRef::MAX_POS_POINT_NUM; ++j) {
      *x_est +=
          ratio[j] *
          (pos_change_ratio * sensor_obj.pos_[0].x_ +
          (1.0F- pos_change_ratio) * tracked_obj.pos_[j].x_);
      *y_est +=
          ratio[j] *
          (pos_change_ratio * sensor_obj.pos_[0].y_ +
          (1.0F- pos_change_ratio) * tracked_obj.pos_[j].y_);
    }

    Float32_t vx_change_ratio = 0.8F;
    Float32_t vy_change_ratio = 0.8F;
    *vx_est =
        vx_change_ratio * sensor_obj.v_x_ +
        (1.0F - vx_change_ratio) * tracked_obj.v_x_;
    *vy_est =
        vy_change_ratio * sensor_obj.v_y_ +
        (1.0F - vy_change_ratio) * tracked_obj.v_y_;

  #if ENABLE_OBJ_TRACKER_TRACE
    printf("            dist=[%0.1f,%0.1f,%0.1f,%0.1f],"
           " ratio=[%0.3f,%0.3f,%0.3f,%0.3f],"
           " x_est=%0.1f, y_est=%0.1f"
           " vx_est=%0.1fkm/h, vy_est=%0.1fkm/h"
           "\n",
           dist[0], dist[1], dist[2], dist[3],
           ratio[0], ratio[1], ratio[2], ratio[3],
           *x_est, *y_est,
           *vx_est*3.6F, *vy_est*3.6F);
#endif
}

void ObjsTracker::EraseExpiredObj()
{
    Int32_t obj_num = tracked_objects_.objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjsTracked& obj = tracked_objects_.objects_[i];
      if ((obj.life_ <= 0) ||
          ((obj.duration_ - obj.age_) >
           common::Min(10, common::Max(5, obj.matched_/2)))) {
        obj.erase_flag_ = true;
      }

      if (obj.erase_flag_) {
        continue;
      }

      if (LIST_ID_TRACKED != obj.list_id_) {
        continue;
      }

      // 清除过期的传感器信息
      Int32_t max_age_sen_idx = -1;
      Int32_t max_age = 0;
      for (Int32_t j = 0; j < SENSOR_ID_MAX; ++j) {
        if (obj.sensor_ref_[j].valid_) {
          if (obj.sensor_ref_[j].age_ > max_age) {
            max_age_sen_idx = j;
            max_age = obj.sensor_ref_[j].age_;
          }
        }
      }
      if (max_age_sen_idx >= 0) {
        for (Int32_t j = 0; j < SENSOR_ID_MAX; ++j) {
          if (obj.sensor_ref_[j].valid_ && (max_age_sen_idx != j)) {
            /// TODO: 5:  UPPER_LIMIT_DURATION_DIFF_WITH_MAX_AGE_FOR_SENSOR_REFS
            if (obj.sensor_ref_[j].age_ < (max_age - 10)) {
              obj.sensor_ref_[j].Clear();
            }
          }
        }
        // 因为只对毫米波的障碍物进行轨迹跟踪，
        // 所以若毫米波的数据无效了，需要清除跟踪的轨迹
        // if (!obj.IsRadarValid())   {
        //   obj.ClearTrackedTrj()  ;
        // }
        if (!obj.IsRadarValid())   {
           obj.ClearTrackedTrj()  ;
        }
      }
    }
}

void ObjsTracker::MergeTrackedObjs()
{
#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### MergeTrackedObjs (Begin) ###" << std::endl;
#endif

  // 设置需要合并的传感器ID
  bool merge_mask[SENSOR_ID_MAX];
  merge_mask[SENSOR_ID_MAIN_FORWARD_CAM] = false;
  merge_mask[SENSOR_ID_MAIN_FORWARD_RADAR] = true;
  merge_mask[SENSOR_ID_MAIN_FORWARD_LIDAR] = true;
  merge_mask[SENSOR_ID_LEFT_SIDE_RADAR] = true;
  merge_mask[SENSOR_ID_LEFT_BACKWARD_RADAR] = true;
  merge_mask[SENSOR_ID_RIGHT_SIDE_RADAR] = true;
  merge_mask[SENSOR_ID_RIGHT_BACKWARD_RADAR] = true;
  //merge_mask[SENSOR_ID_CENTER_FORWARD_CAM] = true;
  //merge_mask[SENSOR_ID_LEFT_SIDE_CAM] = true;
  merge_mask[SENSOR_ID_LEFT_BACKWARD_CAM] = true;
  //merge_mask[SENSOR_ID_RIGHT_SIDE_CAM] = true;
  merge_mask[SENSOR_ID_RIGHT_BACKWARD_CAM] = true;

  /// TODO: 重构此函数，适应多个传感器

  Int32_t obj_num = tracked_objects_.objects_.Size();

#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << ">>@@@ 查找可匹配的obj:" << std::endl;
#endif
  // 对障碍物列表中的每个障碍物都在障碍物跟踪列表中查找与他相关的障碍物信息
  // （位置临近，属性匹配）
  for (Int32_t i = 0; i < obj_num; ++i) {
#if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "  >>> Index[" << i << "]: " << std::endl;
#endif
    ObjsTracked& src_obj = tracked_objects_.objects_[i];

    if (LIST_ID_TRACKED != src_obj.list_id_) {
      continue;
    }
    if (src_obj.erase_flag_) {
      continue;
    }
    src_obj.CalcFusedSensorObj(&src_obj.fused_info_);
    if (!src_obj.fused_info_.valid_) {
      //LOG_ERR << "[Error]Invalid fused sensor obj.";
      continue;
    }

    if (!src_obj.IsRadarValid()) {
      // 只合并毫米波雷达的obj
      continue;
    }


    Float32_t obj_v = common::com_sqrt(common::Square(src_obj.fused_info_.v_x_) +
                                    common::Square(src_obj.fused_info_.v_y_));
    // 临近障碍物的检索半径
    common::Vec2d find_pos;
    Float32_t radius_of_search_range = 1.0F;
    if (src_obj.fused_info_.pos_.x_ > 0.0F) {//wei: 为什么要判断？？？
      find_pos.set_x(src_obj.fused_info_.pos_.x_ + 0.5F*src_obj.fused_info_.length_);
    } else {
      find_pos.set_x(src_obj.fused_info_.pos_.x_ - 0.5F*src_obj.fused_info_.length_);
    }
    find_pos.set_y(src_obj.fused_info_.pos_.y_);
    radius_of_search_range = common::Max(1.0F, 0.5F*src_obj.fused_info_.length_);
    if (obj_v > 15.0F/3.6F) {
      radius_of_search_range += common::Min(3.0F, obj_v * 0.2F);
    }

    common::StaticVector<Int32_t, MAX_PARTNER_NUM> obj_indexs_in_range;
    common::StaticVector<ObjIndex, MAX_PARTNER_NUM> tracked_objs_in_range;
    // 查找临近的所有障碍物（检索半径以内）
    tracked_objects_kdtree_.FindObjects(
          kdtree_nodes_stack_,
          find_pos,
          radius_of_search_range,
          CalcSquareDistToObjTracked(src_obj.fused_info_.aabb_,
                                     tracked_objects_.objects_),
          &obj_indexs_in_range);

#if ENABLE_OBJ_TRACKER_TRACE
    Int32_t radar_sen = SENSOR_ID_MAIN_FORWARD_RADAR;
    if (src_obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
      radar_sen = SENSOR_ID_MAIN_FORWARD_RADAR;
    }
    std::cout << "    当前Tracked obj: {id="
              << src_obj.sensor_ref_[radar_sen].obj_id_[0].id_
              << " pos(" << src_obj.sensor_ref_[radar_sen].pos_[0].x_
              << "," << src_obj.sensor_ref_[radar_sen].pos_[0].y_ << ")"
              << ", range=" << src_obj.sensor_ref_[radar_sen].pos_[0].range_
              << ", angle="
              << common::com_rad2deg(src_obj.sensor_ref_[radar_sen].pos_[0].angle_)
              << ", vx=" << src_obj.fused_info_.v_x_*3.6F
              << ", vy=" << src_obj.fused_info_.v_y_*3.6F
              << ", v=" << obj_v*3.6F << "km/h"
              << "}." << std::endl;
#endif

    Int32_t num_objs_in_range = obj_indexs_in_range.Size();

#if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "    Has *" << num_objs_in_range
              << "* objects near in circle of radius *" << radius_of_search_range
              << "m*, and they are: " << std::endl;
#endif

    for (Int32_t j = 0; j < num_objs_in_range; ++j) {
#if ENABLE_OBJ_TRACKER_TRACE
    std::cout << "      第[" << j << "]个: " << std::endl;
#endif
      const ObjIndex* idx =
          tracked_objects_kdtree_.GetObjectByIndex(obj_indexs_in_range[j]);
      if (Nullptr_t == idx) {
        LOG_ERR << "Invalid ObjIndex";
        continue;
      }
      if (!idx->IsValid()) {
        //LOG_ERR << "Invalid obj index.";
        continue;
      }
      ObjsTracked& tracked_obj = tracked_objects_.objects_[idx->obj_idx_];
      SensorsRef& tracked_obj_sen =
          tracked_obj.sensor_ref_[idx->sensor_idx_];

#if ENABLE_OBJ_TRACKER_TRACE
      std::cout << "        tracke_obj[" << idx->obj_idx_ << "*" << idx->sensor_idx_ << "]"
                << ", pos=[(" << tracked_obj_sen.pos_[0].x_
                << "," << tracked_obj_sen.pos_[0].y_
                << "),(" << tracked_obj_sen.pos_[1].x_
                << "," << tracked_obj_sen.pos_[1].y_
                << "),(" << tracked_obj_sen.pos_[2].x_
                << "," << tracked_obj_sen.pos_[2].y_
                << "),(" << tracked_obj_sen.pos_[3].x_
                << "," << tracked_obj_sen.pos_[3].y_ << ")]"
                << ", range=" << tracked_obj_sen.pos_[0].range_
                << ", angle=" << common::com_rad2deg(tracked_obj_sen.pos_[0].angle_)
                << ", vx=" << tracked_obj_sen.v_x_*3.6F
                << ", vy=" << tracked_obj_sen.v_y_*3.6F
                << ", matched=" << tracked_obj.matched_
                << ", age=" << tracked_obj.age_
                << ", duration=" << tracked_obj.duration_
                << std::endl;
#endif

      if (tracked_obj.erase_flag_) {
        // 已经被删除
#if ENABLE_OBJ_TRACKER_TRACE
        std::cout << "          ^已经被删除, 忽略." << std::endl;
#endif
        continue;
      }

      if (LIST_ID_TRACKED != tracked_obj.list_id_) {
        // 此障碍物是新添加的
#if ENABLE_OBJ_TRACKER_TRACE
        std::cout << "          ^是新添加的, 忽略." << std::endl;
#endif
        continue;
      }

      if (!merge_mask[idx->sensor_idx_]) {
#if ENABLE_OBJ_TRACKER_TRACE
        std::cout << "          ^不合并此传感器, 忽略. (sen_idx="
                  << idx->sensor_idx_ << std::endl;
#endif
        continue;
      }

      if (!tracked_obj_sen.valid_) {
        // 此传感器数据无效
#if ENABLE_OBJ_TRACKER_TRACE
        std::cout << "          ^此传感器数据无效, 忽略." << std::endl;
#endif
        continue;
      }

      Float32_t x_diff = tracked_obj_sen.pos_[0].x_ - find_pos.x();
      Float32_t y_diff = tracked_obj_sen.pos_[0].y_ - find_pos.y();
      Float32_t abs_x_diff = common::com_abs(x_diff);
      Float32_t abs_y_diff = common::com_abs(y_diff);

      if (src_obj.IsCameraValid()) {
        // 若相机数据有效
        // 当纵向偏差过大时，不合并
        if (abs_x_diff > (5.0F + 0.5F*src_obj.fused_info_.length_)) {
          continue;
        }
//        if (abs_y_diff > (5.0F + 0.5F*src_obj.fused_info.length)) {
//          continue;
//        }
        // 若相机数据有效，当横向偏差大于宽度时，不合并
        if (abs_y_diff > (0.1F + 0.5F*src_obj.fused_info_.width_)) {
          continue;
        }
      } else {
        // 当纵向偏差过大时，不合并
        if (abs_x_diff > 10.0F) {
          continue;
        }
        // 当横向偏差过大时，不合并
        if (abs_y_diff > 1.0F) {
          continue;
        }
      }

      /// Note: 由于存在多个传感器，ID可能有重复的
      tracked_objs_in_range.PushBack(*idx);

#if ENABLE_OBJ_TRACKER_TRACE
      std::cout << "          ^是可匹配的" << std::endl;
#endif
    }

    num_objs_in_range = tracked_objs_in_range.Size();

    //wei: 在搜寻的结果中寻找最有可能的数据
    if (num_objs_in_range > 0) {
      ObjIndex possible_obj_idx = tracked_objs_in_range[0];
      Float32_t min_x = tracked_objects_.objects_[possible_obj_idx.obj_idx_].
          sensor_ref_[possible_obj_idx.sensor_idx_].pos_[0].x_;
      Float32_t max_x = tracked_objects_.objects_[possible_obj_idx.obj_idx_].
          sensor_ref_[possible_obj_idx.sensor_idx_].pos_[0].x_;
      Float32_t min_y = tracked_objects_.objects_[possible_obj_idx.obj_idx_].
          sensor_ref_[possible_obj_idx.sensor_idx_].pos_[0].y_;
      Float32_t max_y = tracked_objects_.objects_[possible_obj_idx.obj_idx_].
          sensor_ref_[possible_obj_idx.sensor_idx_].pos_[0].y_;
      for (Int32_t j = 0; j < num_objs_in_range; ++j) {
        ObjIndex other_obj_idx = tracked_objs_in_range[j];
        ObjsTracked& other_obj = tracked_objects_.objects_[other_obj_idx.obj_idx_];
        ObjsTracked& possible_obj = tracked_objects_.objects_[possible_obj_idx.obj_idx_];

        for (Int32_t k = 0; k < SENSOR_ID_MAX; ++k) {
          SensorsRef& other_obj_sen = other_obj.sensor_ref_[k];
          if (!other_obj_sen.valid_) {
            continue;
          }
          if (other_obj_sen.pos_[0].x_ < min_x) {
            min_x = other_obj_sen.pos_[0].x_;
          }
          if (other_obj_sen.pos_[0].x_ > max_x) {
            max_x = other_obj_sen.pos_[0].x_;
          }
          if (other_obj_sen.pos_[0].y_ < min_y) {
            min_y = other_obj_sen.pos_[0].y_;
          }
          if (other_obj_sen.pos_[0].y_ > max_y) {
            max_y = other_obj_sen.pos_[0].y_;
          }
        }

        if (other_obj.matched_ > possible_obj.matched_) {
          possible_obj_idx = other_obj_idx;
        } else if (other_obj.matched_ == possible_obj.matched_) {
          if ((other_obj.duration_ - other_obj.age_) <
              (possible_obj.duration_ - possible_obj.age_)) {
            possible_obj_idx = other_obj_idx;
          } else if ((other_obj.duration_ - other_obj.age_) ==
                     (possible_obj.duration_ - possible_obj.age_)) {


             // 取较大的毫米波车速
             //wei: 由于侧向雷达速度还存在问题，暂时考虑侧向雷达速度
            if (other_obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_ &&
                possible_obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
              if (other_obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].v_x_ >
                  possible_obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].v_x_) {
                possible_obj_idx = other_obj_idx;
              }
            }
          } else {
            // nothing to do
          }
        } else {
          // nothing to do
        }
      }

      // 修改位置，向最近的x值靠近
      ObjsTracked& possible_obj =
          tracked_objects_.objects_[possible_obj_idx.obj_idx_];
      possible_obj.fused_info_.pos_.x_ =
          0.8F * possible_obj.fused_info_.pos_.x_ +
          0.2F * min_x;
      possible_obj.fused_info_.pos_.range_ = common::com_sqrt(
            common::Square(possible_obj.fused_info_.pos_.x_) +
            common::Square(possible_obj.fused_info_.pos_.y_));
      possible_obj.fused_info_.pos_.angle_ =
          common::com_atan2(possible_obj.fused_info_.pos_.y_,
                            possible_obj.fused_info_.pos_.x_);

      // 删除合并后多余的obj
      for (Int32_t j = 0; j < num_objs_in_range; ++j) {
        ObjIndex other_obj_idx = tracked_objs_in_range[j];
        ObjsTracked& other_obj = tracked_objects_.objects_[other_obj_idx.obj_idx_];
        if (other_obj_idx.obj_idx_ != possible_obj_idx.obj_idx_
            /*&& other_obj.matched < 10 && */) {
          other_obj.erase_flag_ = true;
        } else {
          // nothing to do
        }
      }
    }
  }
}

void ObjsTracker::UpdataObjFusionId() {

#if USE_JOINT_ID_FOR_FUSION_ID_MODE //使用传感器ID拼接成融合ID的方式

  Int32_t obj_num = tracked_objects_.objects_.Size();
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];
    obj.GenerateJointModeFusionID();
  }

#else

  Int32_t obj_num = tracked_objects_.objects_.Size();
  static Int32_t max_fusion_id = 0;
  static std::vector<Int32_t> erase_objects_id;
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

    // 如果障碍物是新添加的且丢弃id容器内无丢弃id
    if (obj.list_id_ > 0 && erase_objects_id.size() == 0) {
        obj.fusion_id = max_fusion_id++;
    }

    // 如果障碍物是新添加的且丢弃id有丢弃id
    if (obj.list_id_ > 0 && erase_objects_id.size() > 0) {
      obj.fusion_id = erase_objects_id[erase_objects_id.size() - 1];
      erase_objects_id.pop_back();
    }
  }

  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];
    if (obj.erase_flag_ == 1) {
      erase_objects_id.push_back(obj.fusion_id);
    } 
  }
#endif

}

void ObjsTracker::UpdateObjConfidence()
{
#if ENABLE_OBJ_TRACKER_TRACE
  printf("### UpdateObjConfidence (begin)\n");
#endif

  Int32_t obj_num = tracked_objects_.objects_.Size();
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

#if ENABLE_OBJ_TRACKER_TRACE
    Int32_t radar_sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
    if (obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
      radar_sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
    }
    printf("    [%d] erase=%d"
           ", list=%d, radar=%d, raw_obj=%d"
           ", pos={esr(%d)[%0.1f,%0.1f] cam(%d)[%0.1f,%0.1f]}"
           ", vx=%0.1fkm/h, vy=%0.1fkm/h"
           ", life=%d, matched=%d, age=%d, duration=%d, confidence=%d"
           "\n",
           i,
           obj.erase_flag_,
           obj.list_id_, radar_sen_id, obj.raw_obj_idx_,
           obj.sensor_ref_[radar_sen_id].valid_,
           obj.sensor_ref_[radar_sen_id].pos_[0].x_,
           obj.sensor_ref_[radar_sen_id].pos_[0].y_,
           obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].valid_,
           obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].pos_[0].x_,
           obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].pos_[0].y_,
           obj.sensor_ref_[radar_sen_id].v_x_*3.6F,
           obj.sensor_ref_[radar_sen_id].v_y_*3.6F,
           obj.life_, obj.matched_, obj.age_, obj.duration_, obj.confidence_
           );
#endif

    if (obj.erase_flag_) {
      continue;
    }

    if (obj.IsRadarValid() && obj.IsCameraValid()) {
      // Radar与相机融合的obj
#if ENABLE_OBJ_TRACKER_TRACE
    printf("    ^Radar与相机融合的obj\n");
#endif

      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_FUSED;
      /*
      if (obj.perception_type != obj.last_perception_type) {
        obj.matched_ = 0;
      }
      */

      obj.confidence_ = 100;
      if (obj.matched_ < 3) {
        obj.confidence_ -= 30;
      } else if (obj.matched_ < 5) {
        obj.confidence_ -= 20;
      } else {
        // nothing to do
      }
      if ((obj.duration_ - obj.age_) >
          common::Min(common::Max(5, obj.matched_/2), 5)) {
        obj.confidence_ = 0;
      }
    } else if (!obj.IsRadarValid() && obj.IsCameraValid()) {
      // 仅相机识别的obj
#if ENABLE_OBJ_TRACKER_TRACE
    printf("    ^仅相机识别的obj\n");
#endif

      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_CAMERA;
      /*
      if (obj.perception_type != obj.last_perception_type) {
        obj.matched_ = 0;
      }
      */

      //printf("range:%f, dist_of_lacl_front+30:%f\n", obj.fused_info_.pos_.range_, vehicle_params_.dist_of_localization_to_front_ + 30.0F);
      if ((obj.fused_info_.pos_.range_ <
           (vehicle_params_.dist_of_localization_to_front_ + 30.0F))) {
        obj.confidence_ = common::Max(50, obj.confidence_);
      } else {
        obj.confidence_ = common::Max(80, obj.confidence_);
      }
      //printf("confidence_1:%d\n", obj.confidence_);

      //printf("matched:%d\n", obj.matched_);
      if (obj.matched_ < 3) {
        obj.confidence_ -= 30;
      } else if (obj.matched_ < 5) {
        obj.confidence_ -= 20;
      } else {
        // nothing to do
      }
      //printf("confidence_2:%d\n", obj.confidence_);

      //printf("matched:%d\n", obj.matched_);
      if (obj.matched_> 20) {
        obj.confidence_ += 10;
        if (obj.confidence_ > 90) {
          obj.confidence_ = 90;
        }
      }
      //printf("confidence_3:%d\n", obj.confidence_);

      //printf("dura:%d, age:%d, dura-age:%d, matched:%d, Min(Max(3, obj.matched_/2), 3):%d\n", obj.duration_, obj.age_, obj.duration_ - obj.age_, obj.matched_, common::Min(common::Max(3, obj.matched_/2), 3));
      if ((obj.duration_ - obj.age_) >
          common::Min(common::Max(3, obj.matched_/2), 3)) {
        obj.confidence_ = 0;
      }
      //printf("confidence_4:%d\n", obj.confidence_);
    } else if (obj.IsRadarValid() && !obj.IsCameraValid()) {
      // 仅Radar感知的obj
#if ENABLE_OBJ_TRACKER_TRACE
    printf("    ^仅Radar感知的obj\n");
#endif

      obj.perception_type = ad_msg::OBJ_PRCP_TYPE_RADAR;
      if (obj.perception_type != obj.last_perception_type) {
        obj.matched_ = 0;
      }


      /// TODO: 需要使用融合后的数据
      /// TODO: If the obstacle have been tracked before (v>15km/h),
      /// then it need to be preserved in obstacle list even if
      /// it's current velocity is less than 15km/h
      if (common::com_abs(obj.fused_info_.v_x_) < 15.0F/3.6F) {
#if ENABLE_OBJ_TRACKER_TRACE
        printf("    ^车速较低, <15km/h\n");
#endif
        if ((obj.fused_info_.pos_.range_ <
             (vehicle_params_.dist_of_localization_to_front_ + 10.0F)) &&
            (obj.fused_info_.pos_.angle_ < common::com_deg2rad(30.0F))) {
          obj.confidence_ = common::Max(50, obj.confidence_);
        } else if ((obj.fused_info_.pos_.range_ <
                    (vehicle_params_.dist_of_localization_to_front_ + 5.0F)) &&
                   (obj.fused_info_.pos_.angle_ < common::com_deg2rad(60.0F))) {
          obj.confidence_ = common::Max(50, obj.confidence_);
        } else {
          obj.confidence_ = common::Max(50, obj.confidence_);
        }

        if (obj.matched_ < 5) {
          obj.confidence_ -= 20;
        }
        if ((obj.duration_ - obj.age_) >
            common::Min(common::Max(3, obj.matched_/2), 3)) {
          obj.confidence_ = 0;
        }
      } else {
#if ENABLE_OBJ_TRACKER_TRACE
        printf("    ^车速较高, >15km/h\n");
#endif
        if (obj.fused_info_.pos_.angle_ < common::com_deg2rad(30.0F)) {
          obj.confidence_ = common::Max(50, obj.confidence_);
        } else {
          obj.confidence_ = common::Max(50, obj.confidence_);
        }

        if (obj.matched_ < 3) {
          obj.confidence_ -= 30;
        } else if (obj.matched_ < 5) {
          obj.confidence_ -= 20;
        } else {
          // nothing to do
        }

        if (obj.matched_ > 20) {
          obj.confidence_ += 10;
          if (obj.confidence_ > 90) {
            obj.confidence_ = 90;
          }
        }

        if ((obj.duration_ - obj.age_) >
            common::Min(common::Max(5, obj.matched_/2), 5)) {
          obj.confidence_ = 0;
        }
      }

      /// TODO: 当前车辆低速的情况，特殊处理雷达障碍物
      if (current_rel_pos_.v < 15.0F/3.6F) {
        if ((obj.fused_info_.pos_.x_ < (vehicle_params_.dist_of_localization_to_front_+5.0F)) &&
            (obj.fused_info_.pos_.x_ > (vehicle_params_.dist_of_localization_to_front_-1.0F)) &&
            (common::com_abs(obj.fused_info_.pos_.y_) < (0.5*vehicle_params_.vehicle_width_ + 5.0F))) {
          obj.confidence_ = common::Max(80, obj.confidence_);

          if (obj.matched_ < 3) {
            obj.confidence_ -= 30;
          } else if (obj.matched_ < 5) {
            obj.confidence_ -= 20;
          } else {
            // nothing to do
          }
          if ((obj.duration_ - obj.age_) >
              common::Min(common::Max(3, obj.matched_/2), 3)) {
            obj.confidence_ = 0;
          }
        }
      }
//      /// TODO: 侧向雷达特殊处理
//      if (obj.IsSrr2Valid()) {
//        if ((obj.fused_info.pos.x <
//             (param_.dist_of_localization_to_front+5.0F)) &&
//            (common::com_abs(obj.fused_info.pos.y) >
//             (0.5F*param_.vehicle_width+0.5F)) &&
//            (common::com_abs(obj.fused_info.pos.y) < 15.0F)) {
//          obj.confidence = common::Max(80, obj.confidence);
//        }
//      }
//      /// TODO: 后雷达特殊处理
//      if (obj.sensor_ref[SENSOR_ID_RADAR_5].valid) {
//        if ((obj.fused_info.pos.x < (param_.dist_of_localization_to_rear)) &&
//            (common::com_abs(obj.fused_info.pos.y) < (0.5*param_.vehicle_width + 15.0F))) {
//          obj.confidence = common::Max(80, obj.confidence);

//          if (obj.matched < 3) {
//            obj.confidence -= 30;
//          } else if (obj.matched < 5) {
//            obj.confidence -= 20;
//          } else {
//            // nothing to do
//          }
//          if ((obj.duration - obj.age) >
//              common::Min(common::Max(3, obj.matched/2), 3)) {
//            obj.confidence = 0;
//          }
//        }
//      }
    } else {
      LOG_ERR << "Invalid obj.";
    }

#if ENABLE_OBJ_TRACKER_TRACE
    printf("    ^set confidence to %d\n", obj.confidence);
#endif
  }

#if ENABLE_OBJ_TRACKER_TRACE
  printf("### UpdateObjConfidence (end)\n");
#endif
}

/**
 * @brief 检查 life_ 值状态，当单传感器融合结果时间非常长且稳定，需要尽量保证life_不会变成0，
 * 
 */
void ObjsTracker::CheckObjLife()
{
  Int32_t obj_num = tracked_objects_.objects_.Size();
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];
    if (obj.erase_flag_) {
      continue;
    }
    if (obj.perception_type ==  ad_msg::OBJ_PRCP_TYPE_CAMERA || obj.perception_type ==  ad_msg::OBJ_PRCP_TYPE_RADAR)
    {
      if(obj.life_ >= obj.last_life_)
      {
        obj.life_ref_count_++;
      } 
      else{
        obj.life_ref_count_ = 0;
      }
      /**
       * @brief 当life_ 已经变到比较小的值（这里设置为MAX_TRACKED_OBJ_LIFE/2），但是连续50个周期都没有出现
       * life_下降（也就是不匹配）的情况，life_值将重置为MAX_TRACKED_OBJ_LIFE.
       * 
       */
      if(obj.life_ref_count_ >= 50 )
      {
        obj.life_ref_count_ = 0;
        if (obj.life_ <  (MAX_TRACKED_OBJ_LIFE/2))
        {
          obj.life_ = MAX_TRACKED_OBJ_LIFE;
        }
      }

    }
    else
    {
      obj.life_ref_count_ = 0;
    }
    obj.last_life_ = obj.life_;
  }
}

void ObjsTracker::ModifyAroundRadarObjConfidence()
{
  Int32_t obj_num = tracked_objects_.objects_.Size();
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];
    if (obj.erase_flag_) {
      continue;
    }
    /**
     * @brief 处理单侧雷达传感器融合结果，针对融合挂角的区域进行
     * 横向【-5.7，5.7】 纵向 【1.2，-16】
     * 直接滤除挂角雷达障碍物
     */
    if(common::com_abs(obj.fused_info_.pos_.y_) < 5.7 &&  obj.fused_info_.pos_.x_ + DISTANCE_BETWEEN_MAINCAM_AND_FRONTWHEEL > -16 \
     && obj.fused_info_.pos_.x_ + DISTANCE_BETWEEN_MAINCAM_AND_FRONTWHEEL < 1.2 ) //横向距离：5.625
    {
      if(obj.perception_type == ad_msg::OBJ_PRCP_TYPE_RADAR && (obj.sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_ || obj.sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR].valid_))
      {
        obj.confidence_ = 20;
        obj.erase_flag_ = true;
      }
    }
  }
}


void ObjsTracker::CurbFiltObj(const ObjsFilterDataSource& data_source) {
  // 获取路沿数据
  ad_msg::LaneMarkCameraList* curb_list_cam = data_source.curb_list_cam_;
  if (curb_list_cam->lane_mark_num == 0) {
    return;
  }

  // 将路沿转化为Trenet坐标系下的路径
  std::vector<common::Path> curb_path(MAX_LANE_CURB_NUM);
  for (Uint8_t i = 0; i < curb_list_cam->lane_mark_num; i++) {
    ad_msg::LaneMarkCamera& curb_cam = curb_list_cam->lane_marks[i];

    if (curb_list_cam->lane_marks[i].id == 1) {
      common::Path& path = curb_path[0];
      if (!ConstructPath(curb_cam, path)){
        path.Clear();
      }
    } else if (curb_list_cam->lane_marks[i].id == -1) {
      common::Path& path = curb_path[1];
      if (!ConstructPath(curb_cam, path)){
        path.Clear();
      }
    }
  }

  common::Matrix<Float32_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<Float32_t, 2, 1> point_conv;

  Int32_t obj_num = tracked_objects_.objects_.Size();
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

    if (obj.erase_flag_) {
      continue;
    }

    point_conv(0) = obj.fused_info_.pos_.x_;
    point_conv(1) = obj.fused_info_.pos_.y_;

    common::TransformVert_2D(mat_conv, &point_conv);

    common::Vec2d pos(point_conv(0), point_conv(1));

    common::PathPoint pos_projecton_0; // 障碍物在左路沿的投影(左正右负)
    common::PathPoint pos_projecton_1;  // 障碍物在右路沿的投影(左正右负)

    bool path_point0_exist = false;
    bool path_point1_exist = false;

    // 找到障碍物到左侧路沿path的投影
    if (!curb_path[0].FindProjection(
          pos, &pos_projecton_0)) {
      LOG_ERR << "[Error]Failed to find projecton on major reference line.";
    } else {
      path_point0_exist = true;
    }

    // 找到障碍物到右侧路沿path的投影
    if (!curb_path[1].FindProjection(
        pos, &pos_projecton_1)) {
      LOG_ERR << "[Error]Failed to find projecton on major reference line.";
    } else {
        path_point1_exist = true;
    }

    if ((path_point0_exist == false) && (path_point1_exist == false)) {
      obj.ref_line_.valid_ = false;
    } else {
      obj.ref_line_.valid_ = true;
    }
    
    // 靠近路沿path的障碍物设为路沿
    if (obj.ref_line_.valid_ == true) {
        printf("common::com_abs(pos_projecton_0.l):%.2f\n", common::com_abs(pos_projecton_0.l));
        if ((obj.IsRadarValid()) && (!obj.IsCameraValid()) &&
        (obj.fused_info_.pos_.x_ > 0.0F) &&
        (obj.confidence_ < 60) &&
        (common::com_abs(obj.fused_info_.v_x_) < 15.0F/3.6F)) {

          if (pos_projecton_0.l > -1) {
            obj.fused_info_.obj_type_ = ad_msg::OBJ_TYPE_CURB;
          }

          if (pos_projecton_1.l < 1) {
            obj.fused_info_.obj_type_ = ad_msg::OBJ_TYPE_CURB;
          }
        }
    }
  }
}

bool ObjsTracker::ConstructPath(ad_msg::LaneMarkCamera& lane_curb, common::Path& path)
{
    
    if (lane_curb.quality < 2) {
        return false;
    }
    Float32_t lane_curb_len = lane_curb.view_range_end - lane_curb.view_range_start;
    Int32_t sample_size = 40; 
    if (lane_curb_len < 3){
        return false;
    }
    Float32_t sample_step_len = lane_curb_len / static_cast<Float32_t>(sample_size);
    if (lane_curb_len < sample_step_len || sample_step_len < 1) {
        return false;
    }

    sample_size = common::com_round(lane_curb_len / sample_step_len);
    common::CubicPolynomialCurve1d<Float64_t> curve;
    curve.SetCoefficient(lane_curb.c0, lane_curb.c1,
                         lane_curb.c2, lane_curb.c3);

    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>  point_list;

    for (Int32_t j = 0; j < sample_size; ++j) {
        Float32_t x = j * sample_step_len;
        Float32_t y = static_cast<Float32_t>(curve.Evaluate(0, x));
        point_list.PushBack(common::Vec2d(x,y));
    }
    
    path.Construct(point_list);
    return true;

}

void ObjsTracker::GroupObjs(const ObjsFilterDataSource &data_source)
{
#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### ObjFilterImpl::GroupObjs (Begin) ###" << std::endl;
#endif

  const driv_map::DrivingMapWrapper* driving_map = data_source.driving_map_;

  if (Nullptr_t == driving_map) {
    return;
  }
  if (driving_map->GetReferenceLinesNum() < 1) {
    return;
  }
  // Get major reference line
  Int32_t major_ref_line_index = driving_map->GetMajorReferenceLineIndex();
  if (!driving_map->IsValidReferenceLineIndex(major_ref_line_index)) {
    return;
  }

  ad_msg::RelativePos current_rel_pos;
  if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
        data_source.timestamp_, driving_map->GetRelativePosList(),
        &current_rel_pos)) {
    LOG_ERR << "Failed to get current posistion from relative position list.";
    return;
  }
  common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  common::Matrix<Float32_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Rotate_2D<Float32_t>(rotate_center, current_rel_pos.heading, &mat_conv);
  common::Translate_2D(current_rel_pos.x, current_rel_pos.y, &mat_conv);
  common::Matrix<Float32_t, 2, 1> point_conv;

  const common::Path& major_ref_line =
      driving_map->GetSmoothReferenceLine(major_ref_line_index);

  const common::StaticVector<driv_map::RoadBoundary,
      common::Path::kMaxPathPointNum>& road_boundary =
      driving_map->GetRoadBoundary();

  Int32_t count = 0;
  Int32_t obj_num = tracked_objects_.objects_.Size();
  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

    if (obj.erase_flag_) {
      continue;
    }

    point_conv(0) = obj.fused_info_.pos_.x_;
    point_conv(1) = obj.fused_info_.pos_.y_;

    common::TransformVert_2D(mat_conv, &point_conv);

    common::Vec2d pos(point_conv(0), point_conv(1));
    if (!major_ref_line.FindProjection(
          pos, &obj.ref_line_.proj_on_major_ref_line_)) {
      LOG_ERR << "[Error]Failed to find projecton on major reference line.";
      break;
    }
    obj.ref_line_.valid_ = true;

    // Set heading of camera obj to parall with reference line.
    if (obj.IsCameraValid() &&
        (common::com_abs(obj.fused_info_.v_x_) < 15.0F/3.6F)) {
      obj.heading_ = obj.ref_line_.proj_on_major_ref_line_.heading;
    }

    // 判断是否是路沿
    if ((obj.IsRadarValid()) && (!obj.IsCameraValid()) &&
        (obj.fused_info_.pos_.x_ > 0.0F) &&
        (obj.confidence_ < 60) &&
        (common::com_abs(obj.fused_info_.v_x_) < 15.0F/3.6F)) {
      // 仅ESR感知的静态障碍物
      // 判断障碍物体在道路中的位置
      Float32_t left_boundary_width = 0;
      Float32_t right_boundary_width = 0;
      FindRoadBoundaryWidth(
            road_boundary,
            obj.ref_line_.proj_on_major_ref_line_.s,
            &left_boundary_width, &right_boundary_width);

      Float32_t lat_offset = obj.ref_line_.proj_on_major_ref_line_.l;

      if ((lat_offset > (left_boundary_width - 1.0F)) &&
          (lat_offset < (left_boundary_width + 2.0F))) {
        // left road boundary curb
        obj.fused_info_.obj_type_ = ad_msg::OBJ_TYPE_CURB;
#if 0
        if (lat_offset < (left_boundary_width + 1.0F)) {
          common::Vec2d new_pos = major_ref_line.SLToXY(
                obj.ref_line.proj_on_major_ref_line.s,
                left_boundary_width + 1.0F);
          obj.sensor_ref_[SENSOR_REF_MAIN_FORWARD_RADAR].pos_[0].x_ = new_pos.x();
          obj.sensor_ref_[SENSOR_REF_MAIN_FORWARD_RADAR].pos_[0].y_ = new_pos.y();
        }
#endif
      } else if ((lat_offset < -(right_boundary_width - 1.0F)) &&
                 (lat_offset > -(right_boundary_width + 2.0F))) {
        // right road boundary curb
        obj.fused_info_.obj_type_ = ad_msg::OBJ_TYPE_CURB;
#if 0
        if (lat_offset > -(right_boundary_width + 1.0F)) {
          common::Vec2d new_pos = major_ref_line.SLToXY(
                obj.ref_line_.proj_on_major_ref_line_.s,
                -(right_boundary_width + 1.0F));
          obj.sensor_ref_[SENSOR_REF_MAIN_FORWARD_RADAR].pos_[0].x_ = new_pos.x();
          obj.sensor_ref_[SENSOR_REF_MAIN_FORWARD_RADAR].pos_[0].y_ = new_pos.y();
        }
#endif
      } else {
        // nothing to do
      }
    } else {
      if (ad_msg::OBJ_TYPE_CURB == obj.fused_info_.obj_type_) {
        obj.fused_info_.obj_type_ = ad_msg::OBJ_TYPE_UNKNOWN;
      }
    }

    count++;
  }

#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### ObjFilterImpl::GroupObjs (End) ###" << std::endl;
#endif
}

void ObjsTracker::FindRoadBoundaryWidth(const common::StaticVector<driv_map::RoadBoundary, common::Path::kMaxPathPointNum> &road_boundary, const Float32_t s_ref, Float32_t *left_width, Float32_t *right_width)
{
    if (road_boundary.Empty()) {
      *left_width = 1.5F;
      *right_width = 1.5F;
      return;
    }
    if (s_ref <= road_boundary[0].ref_point.s) {
      *left_width = road_boundary[0].left_width;
      *right_width = road_boundary[0].right_width;
      return;
    }
    if (s_ref >= road_boundary.Back().ref_point.s) {
      *left_width = road_boundary.Back().left_width;
      *right_width = road_boundary.Back().right_width;
      return;
    }
    Int32_t low = 0;
    Int32_t high = static_cast<Int32_t>(road_boundary.Size());
    while (low + 1 < high) {
      const Int32_t mid = (low + high) / 2;
      if (road_boundary[mid].ref_point.s <= s_ref) {
        low = mid;
      } else {
        high = mid;
      }
    }
    const driv_map::RoadBoundary &sample1 = road_boundary[low];
    const driv_map::RoadBoundary &sample2 = road_boundary[high];
    Float32_t ratio = 0.0F;
    if (sample2.ref_point.s - sample1.ref_point.s > 0.1F) {
      ratio = (sample2.ref_point.s - s_ref) /
          (sample2.ref_point.s - sample1.ref_point.s);
    }
    if (ratio > 1.0F) {
      ratio = 1.0F;
    } else if (ratio < 0.0F) {
      ratio = 0.0F;
    } else {
      // nothing to do
    }

    *left_width = sample1.left_width * ratio +
        sample2.left_width * (1.0F - ratio);
    *right_width = sample1.right_width * ratio +
        sample2.right_width * (1.0F - ratio);
}

void ObjsTracker::SmoothTrackedObjs()
{
  
  #if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### ObjFilterImpl::SmoothTrackedObjs (Begin) ###" << std::endl;
  #endif

#if ENABLE_OTHER_DEBUG_PRINTF 
  std::cout << "### ObjFilterImpl::SmoothTrackedObjs (Begin) ###" << std::endl;
#endif
  Int32_t obj_num = tracked_objects_.objects_.Size();

  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

    if (obj.erase_flag_) {
      continue;
    }

    //printf("obj.age_ : %d obj.duration_ %d\n",obj.age_,obj.duration_);
    
    if (obj.age_ < obj.duration_) {
      continue;
    }
    if (LIST_ID_TRACKED != obj.list_id_) {
      continue;                   
    }

    if (!obj.IsRadarValid()) {//
      continue;//
    }

    /*
    * 只处理前视1V1R的数据
    */
    if (!obj.IsForesightObj()) {
      continue;
    }

    /*
    * 当Radar融合结果切到多传感器融合结果时,重新设置横向距离和速度，原因是横向距离和速度的计算需要以相机数据为准
    * 为减小Radar历史数据对横向距离和速度的影响，针对上一次的横向距离和速度需要重新赋上相机的数据。
    */
    if(obj.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED  && obj.last_perception_type == ad_msg::OBJ_PRCP_TYPE_RADAR)
    {
      obj.pos_status_(1) = obj.fused_info_.pos_.y_;
      obj.pos_status_(3) = obj.fused_info_.v_y_;
      obj.avg_status_.y_ = obj.fused_info_.pos_.y_;
      obj.avg_status_.v_y_ = obj.fused_info_.v_y_;
    }

    // LOG_INFO(2)  << "### ObjFilterImpl::SmoothTrackedObjs (Begin) ###";

    TrjPointInfos* pt = Nullptr_t;

    double measurement_vx=0.0;

    double measurement_vy=0.0;

    Float32_t obj_v = common::com_sqrt(
          common::Square(obj.fused_info_.v_x_) +
          common::Square(obj.fused_info_.v_y_));
    Int32_t trj_size = obj.traced_trj_[1].Size();
    if (trj_size < 1) {
      pt = obj.traced_trj_[0].AllocateOverride();
      if (Nullptr_t != pt) {
        pt->seq_num_ = 0;
        pt->relative_time_ms_ = 0.0F;
        pt->x_ = obj.fused_info_.pos_.x_;
        pt->y_ = obj.fused_info_.pos_.y_;
        pt->vx_ = obj.fused_info_.v_x_;
        pt->vy_ = obj.fused_info_.v_y_;
      }
      obj.avg_status_.x_ = obj.fused_info_.pos_.x_;
      obj.avg_status_.y_ = obj.fused_info_.pos_.y_;
      obj.avg_status_.v_x_ = obj.fused_info_.v_x_;
      obj.avg_status_.v_y_ = obj.fused_info_.v_y_;
      obj.avg_status_.a_x_ = 0.0F;
      obj.avg_status_.a_y_ = 0.0F;
      pt = obj.traced_trj_[1].AllocateOverride();
      if (Nullptr_t != pt) {
        pt->seq_num_ = 0;
        pt->relative_time_ms_ = 0.0F;
        pt->x_ = obj.fused_info_.pos_.x_;
        pt->y_ = obj.fused_info_.pos_.y_;
        pt->vx_ = obj.fused_info_.v_x_;
        pt->vy_ = obj.fused_info_.v_y_;
      }
      obj.pos_status_(0) = obj.fused_info_.pos_.x_;
      obj.pos_status_(1) = obj.fused_info_.pos_.y_;
      obj.pos_status_(2) = obj.fused_info_.v_x_;
      obj.pos_status_(3) = obj.fused_info_.v_y_;
      obj.pos_status_(4) = 0.0F;
      obj.pos_status_(5) = 0.0F;
      obj.pos_covariance_.SetIdentity();

      obj.heading_ = 0.0F;
      // if (obj.fused_info_.v_x_ < -20.0F/3.6F) {
      //   obj.heading_ = COM_PI;
      // }

      continue;
    }
    
    pt = obj.traced_trj_[0].AllocateOverride();
    if (Nullptr_t != pt) {
      pt->seq_num_ = 0;
      pt->relative_time_ms_ = 0.0F;
      pt->x_ = obj.fused_info_.pos_.x_;
      pt->y_ = obj.fused_info_.pos_.y_;
      pt->vx_ = obj.fused_info_.v_x_;
      pt->vy_ = obj.fused_info_.v_y_;
    }

    TrjPointInfos* prev_pt = obj.traced_trj_[1].Back();
    Float32_t time_elapsed = 0.0F - prev_pt->relative_time_ms_;

    Float32_t avg_x = 0.0F;
    Float32_t avg_y = 0.0F;
    Float32_t avg_v_x = 0.0F;
    Float32_t avg_v_y = 0.0F;
    Float32_t avg_a_x = 0.0F;
    Float32_t avg_a_y = 0.0F;

    common::RingBuffer<TrjPointInfos,
        MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it =
        obj.traced_trj_[0].begin();
    common::RingBuffer<TrjPointInfos,
            MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it_end =
        obj.traced_trj_[0].end();
    Int32_t trj_num = 0;
    for (; trj_it != trj_it_end; ++trj_it) {
      TrjPointInfos* pt = trj_it.current();

      avg_x += pt->x_;
      avg_y += pt->y_;

      trj_num++;
    }
    avg_x /= trj_num;
    avg_y /= trj_num;

    avg_v_x = 0.5F*obj.fused_info_.v_x_ +
              0.5F * (avg_x - obj.avg_status_.x_) / time_elapsed;
    avg_v_y = 0.0F * obj.fused_info_.v_y_ +
              1.0F * (avg_y - obj.avg_status_.y_) / time_elapsed +
              1.3F * current_rel_pos_.yaw_rate * obj.fused_info_.pos_.range_;;
    avg_a_x = (avg_v_x - obj.avg_status_.v_x_) / time_elapsed;
    avg_a_y = (avg_v_y - obj.avg_status_.v_y_) / time_elapsed;
    // Kalman 3
    // predicted
    // x = A * x(t-1) + B * u
    common::Matrix<Float32_t, 6, 1> mat_x;

#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    mat_x(0) = obj.pos_status_(0) + obj.pos_status_(2) * time_elapsed;
    mat_x(1) = obj.pos_status_(1) + obj.pos_status_(3) * time_elapsed;
    mat_x(2) = obj.pos_status_(2);
    mat_x(3) = obj.pos_status_(3);
    mat_x(4) = 0;
    mat_x(5) = 0;
#else
    mat_x(0) = obj.pos_status_(0) + (obj.pos_status_(2) + 0.5F * obj.pos_status_(4) * time_elapsed) * time_elapsed;
    mat_x(1) = obj.pos_status_(1) + (obj.pos_status_(3) + 0.5F * obj.pos_status_(5) * time_elapsed) * time_elapsed;
    mat_x(2) = obj.pos_status_(2) + obj.pos_status_(4) * time_elapsed;
    mat_x(3) = obj.pos_status_(3) + obj.pos_status_(5) * time_elapsed;
    mat_x(4) = obj.pos_status_(4);
    mat_x(5) = obj.pos_status_(5);
#endif

    if(mat_x(0)<0){

#if ENABLE_OTHER_DEBUG_PRINTF 
    LOG_INFO(2) << "=====================================================";
    LOG_INFO(2) << "kalman time_elapsed:" << time_elapsed;
    LOG_INFO(2) << "K-1|K-1:(" << obj.pos_status_(0) << "," << obj.pos_status_(1) << ") v0:(" << obj.pos_status_(2) << ", " << obj.pos_status_(3) << ")" 
                << " a0:(" << obj.pos_status_(4) << "," << obj.pos_status_(5) << ")";


    LOG_INFO(2) << "K|K-1 s:(" << mat_x(0) << "," << mat_x(1) << ")  v(" << mat_x(2) << "," << mat_x(3)
                << ")";

    LOG_INFO(2)<<"sensor s:(" << obj.fused_info_.pos_.x_ << "," << obj.fused_info_.pos_.y_ << ")  v(" << obj.fused_info_.v_x_ << "," << obj.fused_info_.v_y_
                << ")";
#endif

    }
    
    // P = A * P_ * A.transpose() + R;
    common::Matrix<Float32_t, 6, 6> mat_P;
    mat_P = obj.pos_covariance_;

    mat_P(0, 0) += 4.0F;
    mat_P(1, 1) += 4.0F;
    mat_P(2, 2) += 1.5F ;//0.1
    mat_P(3, 3) += 1.5F ;//0.1
    mat_P(4, 4) += 4.0F;
    mat_P(5, 5) += 4.0F;

    // measurement
    // Identity
    // 计算kalman增益矩阵 [ K = P * C_t * (C * P * C_t + Q)^-1 ]
    // C * P * C_t
    common::Matrix<Float32_t, 6, 6> mat_tmp_6v6;
    common::Matrix<Float32_t, 6, 6> mat_tmp_6v6_1;
    // C * P * C_t + Q
    mat_tmp_6v6 = mat_P;
    mat_tmp_6v6(0, 0) += 4.0F + obj.fused_info_.pos_.range_*0.2F;
    mat_tmp_6v6(1, 1) += 4.0F + obj.fused_info_.pos_.range_*0.5F;
    mat_tmp_6v6(2, 2) += 1.5F;//0.1
    mat_tmp_6v6(3, 3) += 1.5F;//0.1
    mat_tmp_6v6(4, 4) += 4.0F;
    mat_tmp_6v6(5, 5) += 4.0F;

    // (C * P * C_t + Q)^-1 ]
    common::Mat_CalcPseudoInverse(mat_tmp_6v6, mat_tmp_6v6_1);
    // 增益矩阵K
    common::Matrix<Float32_t, 6, 6> mat_K;
    common::Mat_Mul(mat_P, mat_tmp_6v6_1, mat_K);
    // 使用观测量修正状态量 [ pos_status = x + K(z - C*x) ]
    obj.avg_status_.x_ = avg_x;
    obj.avg_status_.y_ = avg_y;
    obj.avg_status_.v_x_ = avg_v_x;
    obj.avg_status_.v_y_ = avg_v_y;
#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    obj.avg_status_.a_x_ = 0;
    obj.avg_status_.a_y_ = 0;
#else
    obj.avg_status_.a_x_ = avg_a_x;
    obj.avg_status_.a_y_ = avg_a_y;
#endif


    common::Matrix<Float32_t, 6, 1> mat_z;
    common::Matrix<Float32_t, 6, 1> mat_tmp_6v1;
    mat_z(0) = obj.fused_info_.pos_.x_ - mat_x(0);
    mat_z(1) = obj.fused_info_.pos_.y_ - mat_x(1);
    //mat_z(2) = avg_v_x - mat_x(2);
    //mat_z(3) = avg_v_y - mat_x(3);
    mat_z(2)=obj.fused_info_.v_x_ - mat_x(2);
    mat_z(3)=obj.fused_info_.v_y_ - mat_x(3);
#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    mat_z(4) = 0;
    mat_z(5) = 0;
#else
    mat_z(4) = avg_a_x - mat_x(4);
    mat_z(5) = avg_a_y - mat_x(5);
#endif

    common::Mat_Mul(mat_K, mat_z, mat_tmp_6v1);
    common::Mat_Add(mat_x, mat_tmp_6v1, obj.pos_status_);
    // 修正Covariance [ covariance = (I - K * C) * P ]
    common::Matrix<Float32_t, 6, 6> mat_I_6v6;
    mat_I_6v6.SetIdentity();
    common::Mat_Sub(mat_I_6v6, mat_K, mat_tmp_6v6);
    common::Mat_Mul(mat_tmp_6v6, mat_P, obj.pos_covariance_);

    pt = obj.traced_trj_[1].AllocateOverride();
    if (Nullptr_t != pt) {
      pt->seq_num_ = 0;
      pt->relative_time_ms_ = 0.0F;
      pt->x_ = obj.pos_status_(0);
      pt->y_ = obj.pos_status_(1);
      pt->vx_ = obj.fused_info_.v_x_;
      pt->vy_ = obj.fused_info_.v_y_;
    }

    Float32_t dist_start_end = common::com_sqrt(
          common::Square(obj.pos_status_(0) - obj.traced_trj_[1][0].x_) +
        common::Square(obj.pos_status_(1) - obj.traced_trj_[1][0].y_));
    //Scalar avg_v_from_pos = dist_start_end /
    //    (0.0F - obj.traced_trj[0].relative_time_ms);
    Int32_t tracked_trj_size = obj.traced_trj_[1].Size();
    if ((tracked_trj_size > 8) && (obj_v > 5.0F/3.6F) &&
        (dist_start_end > 4.0F)) {
      obj.heading_ = common::com_atan2(obj.pos_status_(1) - obj.traced_trj_[1][0].y_,
          obj.pos_status_(0) - obj.traced_trj_[1][0].x_);

        if((obj.pos_status_(0) - obj.traced_trj_[1][0].x_)>0){

            obj.heading_=obj.heading_;

        }else{

            obj.heading_=-obj.heading_;

        }

    } else {
      Float32_t est_heading = 0.0F;
      Float32_t diff0 =
          common::com_abs(common::AngleDiff(obj.heading_, 0.0F));
      Float32_t diff1 =
          common::com_abs(common::AngleDiff(obj.heading_, (Float32_t)COM_PI));
      if (diff0 > diff1) {
        est_heading = COM_PI;
      }

      //std::cout << "2. obj_heading, from "
      //          << common::com_rad2deg(obj.obj_heading)
      //          << " to "
      //          << common::com_rad2deg(est_heading)
      //          << ", ";
      obj.heading_ = common::AngleLerp(obj.heading_, est_heading, 0.1F);
      //std::cout << "ret=" << common::com_rad2deg(obj.heading_) << std::endl;
    }

     //LOG_INFO(2) <<"obj type "<<obj.fused_info_.obj_type_;

    //if(0<=fabs(obj.fused_info_.v_x_) &&fabs(obj.fused_info_.v_x_)<=5.0){

      //obj.pos_status_(2)=obj.fused_info_.v_x_;

      //obj.pos_status_(3)=obj.fused_info_.v_y_;

    //}

    if(obj.pos_status_(0)>0){

#if ENABLE_OTHER_DEBUG_PRINTF 
      LOG_INFO(2) << "K|K s:(" << obj.pos_status_(0)<< "," << obj.pos_status_(1) << ")  v(" << obj.pos_status_(2) << "," << obj.pos_status_(3)
                << ")";
      LOG_INFO(2) << "=====================================================";
#endif

    }

  }
      
  for (Int32_t i = 0; i < tracked_objects_.objects_.Size(); ++i) {

      ObjsTracked& src_obj = tracked_objects_.objects_[i];
    
      if(src_obj.confidence_>60 && src_obj.heading_<0.0 && src_obj.fused_info_.v_x_>5.0){

        src_obj.heading_=common::com_atan2(src_obj.fused_info_.v_y_,src_obj.fused_info_.v_x_);

    }

  }

#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### ObjFilterImpl::SmoothTrackedObjs (End) ###" << std::endl;
#endif
}

/**
 * @brief 针对侧雷达和环视相机的融合结果进行平滑
 * 
 */
void ObjsTracker::SmoothAroundLeftTrackedObjs()
{
  
  Int32_t obj_num = tracked_objects_.objects_.Size();

  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

    if (obj.erase_flag_) {
      continue;
    }
    if (obj.age_ < obj.duration_) {
      continue;
    }
    if (LIST_ID_TRACKED != obj.list_id_) {
      continue;                   
    }

    if (!obj.IsRadarValid()) {//
      continue;//
    }

    /*
    * 只处理左侧2V2R的数据
    */
    if (!obj.IsLeftSensorObj()) {
      continue;
    }
    /*
    * 当Radar融合结果切到多传感器融合结果时,重新设置横向距离和速度，原因是横向距离和速度的计算需要以相机数据为准
    * 为减小Radar历史数据对横向距离和速度的影响，针对上一次的横向距离和速度需要重新赋上相机的数据。
    */
    if(obj.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED  && obj.last_perception_type == ad_msg::OBJ_PRCP_TYPE_RADAR)
    {
      obj.pos_status_(1) = obj.fused_info_.pos_.y_;
      obj.pos_status_(3) = obj.fused_info_.v_y_;
      obj.avg_status_.y_ = obj.fused_info_.pos_.y_;
      obj.avg_status_.v_y_ = obj.fused_info_.v_y_;
    }


    TrjPointInfos* pt = Nullptr_t;

    double measurement_vx=0.0;

    double measurement_vy=0.0;

    Float32_t obj_v = common::com_sqrt(
          common::Square(obj.fused_info_.v_x_) +
          common::Square(obj.fused_info_.v_y_));
    Int32_t trj_size = obj.traced_trj_[1].Size();
    if (trj_size < 1) {
      pt = obj.traced_trj_[0].AllocateOverride();
      if (Nullptr_t != pt) {
        pt->seq_num_ = 0;
        pt->relative_time_ms_ = 0.0F;
        pt->x_ = obj.fused_info_.pos_.x_;
        pt->y_ = obj.fused_info_.pos_.y_;
        pt->vx_ = obj.fused_info_.v_x_;
        pt->vy_ = obj.fused_info_.v_y_;
      }
      obj.avg_status_.x_ = obj.fused_info_.pos_.x_;
      obj.avg_status_.y_ = obj.fused_info_.pos_.y_;
      obj.avg_status_.v_x_ = obj.fused_info_.v_x_;
      obj.avg_status_.v_y_ = obj.fused_info_.v_y_;
      obj.avg_status_.a_x_ = 0.0F;
      obj.avg_status_.a_y_ = 0.0F;
      pt = obj.traced_trj_[1].AllocateOverride();
      if (Nullptr_t != pt) {
        pt->seq_num_ = 0;
        pt->relative_time_ms_ = 0.0F;
        pt->x_ = obj.fused_info_.pos_.x_;
        pt->y_ = obj.fused_info_.pos_.y_;
        pt->vx_ = obj.fused_info_.v_x_;
        pt->vy_ = obj.fused_info_.v_y_;
      }
      obj.pos_status_(0) = obj.fused_info_.pos_.x_;
      obj.pos_status_(1) = obj.fused_info_.pos_.y_;
      obj.pos_status_(2) = obj.fused_info_.v_x_;
      obj.pos_status_(3) = obj.fused_info_.v_y_;
      obj.pos_status_(4) = 0.0F;
      obj.pos_status_(5) = 0.0F;
      obj.pos_covariance_.SetIdentity();

      obj.heading_ = 0.0F;
      continue;
    }
    
    pt = obj.traced_trj_[0].AllocateOverride();
    if (Nullptr_t != pt) {
      pt->seq_num_ = 0;
      pt->relative_time_ms_ = 0.0F;
      pt->x_ = obj.fused_info_.pos_.x_;
      pt->y_ = obj.fused_info_.pos_.y_;
      pt->vx_ = obj.fused_info_.v_x_;
      pt->vy_ = obj.fused_info_.v_y_;
    }

    TrjPointInfos* prev_pt = obj.traced_trj_[1].Back();
    Float32_t time_elapsed = 0.0F - prev_pt->relative_time_ms_;

    Float32_t avg_x = 0.0F;
    Float32_t avg_y = 0.0F;
    Float32_t avg_v_x = 0.0F;
    Float32_t avg_v_y = 0.0F;
    Float32_t avg_a_x = 0.0F;
    Float32_t avg_a_y = 0.0F;

    common::RingBuffer<TrjPointInfos,
        MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it =
        obj.traced_trj_[0].begin();
    common::RingBuffer<TrjPointInfos,
            MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it_end =
        obj.traced_trj_[0].end();
    Int32_t trj_num = 0;
    for (; trj_it != trj_it_end; ++trj_it) {
      TrjPointInfos* pt = trj_it.current();

      avg_x += pt->x_;
      avg_y += pt->y_;

      trj_num++;
    }
    avg_x /= trj_num;
    avg_y /= trj_num;

    avg_v_x = 0.5F*obj.fused_info_.v_x_ +
              0.5F * (avg_x - obj.avg_status_.x_) / time_elapsed;
    avg_v_y = 0.0F * obj.fused_info_.v_y_ +
              1.0F * (avg_y - obj.avg_status_.y_) / time_elapsed +
              1.3F * current_rel_pos_.yaw_rate * obj.fused_info_.pos_.range_;;
    avg_a_x = (avg_v_x - obj.avg_status_.v_x_) / time_elapsed;
    avg_a_y = (avg_v_y - obj.avg_status_.v_y_) / time_elapsed;
    // Kalman 3
    // predicted
    // x = A * x(t-1) + B * u
    common::Matrix<Float32_t, 6, 1> mat_x;

#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    mat_x(0) = obj.pos_status_(0) + obj.pos_status_(2) * time_elapsed;
    mat_x(1) = obj.pos_status_(1) + obj.pos_status_(3) * time_elapsed;
    mat_x(2) = obj.pos_status_(2);
    mat_x(3) = obj.pos_status_(3);
    mat_x(4) = 0;
    mat_x(5) = 0;
#else
    mat_x(0) = obj.pos_status_(0) + (obj.pos_status_(2) + 0.5F * obj.pos_status_(4) * time_elapsed) * time_elapsed;
    mat_x(1) = obj.pos_status_(1) + (obj.pos_status_(3) + 0.5F * obj.pos_status_(5) * time_elapsed) * time_elapsed;
    mat_x(2) = obj.pos_status_(2) + obj.pos_status_(4) * time_elapsed;
    mat_x(3) = obj.pos_status_(3) + obj.pos_status_(5) * time_elapsed;
    mat_x(4) = obj.pos_status_(4);
    mat_x(5) = obj.pos_status_(5);
#endif

    if(mat_x(0)<0){

#if ENABLE_OTHER_DEBUG_PRINTF 
    LOG_INFO(2) << "=====================================================";
    LOG_INFO(2) << "kalman time_elapsed:" << time_elapsed;
    LOG_INFO(2) << "K-1|K-1:(" << obj.pos_status_(0) << "," << obj.pos_status_(1) << ") v0:(" << obj.pos_status_(2) << ", " << obj.pos_status_(3) << ")" 
                << " a0:(" << obj.pos_status_(4) << "," << obj.pos_status_(5) << ")";
    LOG_INFO(2) << "K|K-1 s:(" << mat_x(0) << "," << mat_x(1) << ")  v(" << mat_x(2) << "," << mat_x(3)
                << ")";
    LOG_INFO(2)<<"sensor s:(" << obj.fused_info_.pos_.x_ << "," << obj.fused_info_.pos_.y_ << ")  v(" << obj.fused_info_.v_x_ << "," << obj.fused_info_.v_y_
                << ")";
#endif
    }
    
    // P = A * P_ * A.transpose() + R;
    common::Matrix<Float32_t, 6, 6> mat_P;
    mat_P = obj.pos_covariance_;

    mat_P(0, 0) += 4.0F;
    mat_P(1, 1) += 4.0F;
    mat_P(2, 2) += 1.5F ;//0.1
    mat_P(3, 3) += 1.5F ;//0.1
    mat_P(4, 4) += 4.0F;
    mat_P(5, 5) += 4.0F;

    // measurement
    // Identity
    // 计算kalman增益矩阵 [ K = P * C_t * (C * P * C_t + Q)^-1 ]
    // C * P * C_t
    common::Matrix<Float32_t, 6, 6> mat_tmp_6v6;
    common::Matrix<Float32_t, 6, 6> mat_tmp_6v6_1;
    // C * P * C_t + Q
    mat_tmp_6v6 = mat_P;
    mat_tmp_6v6(0, 0) += 4.0F + obj.fused_info_.pos_.range_*0.2F;
    mat_tmp_6v6(1, 1) += 4.0F + obj.fused_info_.pos_.range_*0.5F;
    mat_tmp_6v6(2, 2) += 1.5F;//0.1
    mat_tmp_6v6(3, 3) += 1.5F;//0.1
    mat_tmp_6v6(4, 4) += 4.0F;
    mat_tmp_6v6(5, 5) += 4.0F;

    // (C * P * C_t + Q)^-1 ]
    common::Mat_CalcPseudoInverse(mat_tmp_6v6, mat_tmp_6v6_1);
    // 增益矩阵K
    common::Matrix<Float32_t, 6, 6> mat_K;
    common::Mat_Mul(mat_P, mat_tmp_6v6_1, mat_K);
    // 使用观测量修正状态量 [ pos_status = x + K(z - C*x) ]
    obj.avg_status_.x_ = avg_x;
    obj.avg_status_.y_ = avg_y;
    obj.avg_status_.v_x_ = avg_v_x;
    obj.avg_status_.v_y_ = avg_v_y;
#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    obj.avg_status_.a_x_ = 0;
    obj.avg_status_.a_y_ = 0;
#else
    obj.avg_status_.a_x_ = avg_a_x;
    obj.avg_status_.a_y_ = avg_a_y;
#endif


    common::Matrix<Float32_t, 6, 1> mat_z;
    common::Matrix<Float32_t, 6, 1> mat_tmp_6v1;
    mat_z(0) = obj.fused_info_.pos_.x_ - mat_x(0);
    mat_z(1) = obj.fused_info_.pos_.y_ - mat_x(1);
    //mat_z(2) = avg_v_x - mat_x(2);
    //mat_z(3) = avg_v_y - mat_x(3);
    mat_z(2)=obj.fused_info_.v_x_ - mat_x(2);
    mat_z(3)=obj.fused_info_.v_y_ - mat_x(3);
#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    mat_z(4) = 0;
    mat_z(5) = 0;
#else
    mat_z(4) = avg_a_x - mat_x(4);
    mat_z(5) = avg_a_y - mat_x(5);
#endif

    common::Mat_Mul(mat_K, mat_z, mat_tmp_6v1);
    common::Mat_Add(mat_x, mat_tmp_6v1, obj.pos_status_);
    // 修正Covariance [ covariance = (I - K * C) * P ]
    common::Matrix<Float32_t, 6, 6> mat_I_6v6;
    mat_I_6v6.SetIdentity();
    common::Mat_Sub(mat_I_6v6, mat_K, mat_tmp_6v6);
    common::Mat_Mul(mat_tmp_6v6, mat_P, obj.pos_covariance_);

    pt = obj.traced_trj_[1].AllocateOverride();
    if (Nullptr_t != pt) {
      pt->seq_num_ = 0;
      pt->relative_time_ms_ = 0.0F;
      pt->x_ = obj.pos_status_(0);
      pt->y_ = obj.pos_status_(1);
      pt->vx_ = obj.fused_info_.v_x_;
      pt->vy_ = obj.fused_info_.v_y_;
    }

    Float32_t dist_start_end = common::com_sqrt(
          common::Square(obj.pos_status_(0) - obj.traced_trj_[1][0].x_) +
        common::Square(obj.pos_status_(1) - obj.traced_trj_[1][0].y_));
    //Scalar avg_v_from_pos = dist_start_end /
    //    (0.0F - obj.traced_trj[0].relative_time_ms);
    Int32_t tracked_trj_size = obj.traced_trj_[1].Size();
    if ((tracked_trj_size > 8) && (obj_v > 5.0F/3.6F) &&
        (dist_start_end > 4.0F)) {
      obj.heading_ = common::com_atan2(obj.pos_status_(1) - obj.traced_trj_[1][0].y_,
          obj.pos_status_(0) - obj.traced_trj_[1][0].x_);

        if((obj.pos_status_(0) - obj.traced_trj_[1][0].x_)>0){

            obj.heading_=obj.heading_;

        }else{

            obj.heading_=-obj.heading_;

        }

    } else {
      Float32_t est_heading = 0.0F;
      Float32_t diff0 =
          common::com_abs(common::AngleDiff(obj.heading_, 0.0F));
      Float32_t diff1 =
          common::com_abs(common::AngleDiff(obj.heading_, (Float32_t)COM_PI));
      if (diff0 > diff1) {
        est_heading = COM_PI;
      }
      obj.heading_ = common::AngleLerp(obj.heading_, est_heading, 0.1F);
    }
    if(obj.pos_status_(0)>0){

#if ENABLE_OTHER_DEBUG_PRINTF 
      LOG_INFO(2) << "K|K s:(" << obj.pos_status_(0)<< "," << obj.pos_status_(1) << ")  v(" << obj.pos_status_(2) << "," << obj.pos_status_(3)
                << ")";
      LOG_INFO(2) << "=====================================================";
#endif

    }

  }
      
  for (Int32_t i = 0; i < tracked_objects_.objects_.Size(); ++i) {

      ObjsTracked& src_obj = tracked_objects_.objects_[i];
    
      if(src_obj.confidence_>60 && src_obj.heading_<0.0 && src_obj.fused_info_.v_x_>5.0){

        src_obj.heading_=common::com_atan2(src_obj.fused_info_.v_y_,src_obj.fused_info_.v_x_);

    }

  }

#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### ObjFilterImpl::SmoothTrackedObjs (End) ###" << std::endl;
#endif
}

/**
 * @brief 针对右侧雷达和环视相机的融合结果进行平滑
 * 
 */
void ObjsTracker::SmoothAroundRightTrackedObjs()
{
  
  Int32_t obj_num = tracked_objects_.objects_.Size();

  for (Int32_t i = 0; i < obj_num; ++i) {
    ObjsTracked& obj = tracked_objects_.objects_[i];

    if (obj.erase_flag_) {
      continue;
    }
    if (obj.age_ < obj.duration_) {
      continue;
    }
    if (LIST_ID_TRACKED != obj.list_id_) {
      continue;                   
    }

    if (!obj.IsRadarValid()) {//
      continue;//
    }

   /*
    * 只处理右侧2V2R的数据
    */
    if (!obj.IsRightSensorObj()) {
      continue;
    }
    /*
    * 当Radar融合结果切到多传感器融合结果时,重新设置横向距离和速度，原因是横向距离和速度的计算需要以相机数据为准
    * 为减小Radar历史数据对横向距离和速度的影响，针对上一次的横向距离和速度需要重新赋上相机的数据。
    */
    if(obj.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED  && obj.last_perception_type == ad_msg::OBJ_PRCP_TYPE_RADAR)
    {
      obj.pos_status_(1) = obj.fused_info_.pos_.y_;
      obj.pos_status_(3) = obj.fused_info_.v_y_;
      obj.avg_status_.y_ = obj.fused_info_.pos_.y_;
      obj.avg_status_.v_y_ = obj.fused_info_.v_y_;
    }


    TrjPointInfos* pt = Nullptr_t;

    double measurement_vx=0.0;

    double measurement_vy=0.0;

    Float32_t obj_v = common::com_sqrt(
          common::Square(obj.fused_info_.v_x_) +
          common::Square(obj.fused_info_.v_y_));
    Int32_t trj_size = obj.traced_trj_[1].Size();
    if (trj_size < 1) {
      pt = obj.traced_trj_[0].AllocateOverride();
      if (Nullptr_t != pt) {
        pt->seq_num_ = 0;
        pt->relative_time_ms_ = 0.0F;
        pt->x_ = obj.fused_info_.pos_.x_;
        pt->y_ = obj.fused_info_.pos_.y_;
        pt->vx_ = obj.fused_info_.v_x_;
        pt->vy_ = obj.fused_info_.v_y_;
      }
      obj.avg_status_.x_ = obj.fused_info_.pos_.x_;
      obj.avg_status_.y_ = obj.fused_info_.pos_.y_;
      obj.avg_status_.v_x_ = obj.fused_info_.v_x_;
      obj.avg_status_.v_y_ = obj.fused_info_.v_y_;
      obj.avg_status_.a_x_ = 0.0F;
      obj.avg_status_.a_y_ = 0.0F;
      pt = obj.traced_trj_[1].AllocateOverride();
      if (Nullptr_t != pt) {
        pt->seq_num_ = 0;
        pt->relative_time_ms_ = 0.0F;
        pt->x_ = obj.fused_info_.pos_.x_;
        pt->y_ = obj.fused_info_.pos_.y_;
        pt->vx_ = obj.fused_info_.v_x_;
        pt->vy_ = obj.fused_info_.v_y_;
      }
      obj.pos_status_(0) = obj.fused_info_.pos_.x_;
      obj.pos_status_(1) = obj.fused_info_.pos_.y_;
      obj.pos_status_(2) = obj.fused_info_.v_x_;
      obj.pos_status_(3) = obj.fused_info_.v_y_;
      obj.pos_status_(4) = 0.0F;
      obj.pos_status_(5) = 0.0F;
      obj.pos_covariance_.SetIdentity();

      obj.heading_ = 0.0F;
      continue;
    }
    
    pt = obj.traced_trj_[0].AllocateOverride();
    if (Nullptr_t != pt) {
      pt->seq_num_ = 0;
      pt->relative_time_ms_ = 0.0F;
      pt->x_ = obj.fused_info_.pos_.x_;
      pt->y_ = obj.fused_info_.pos_.y_;
      pt->vx_ = obj.fused_info_.v_x_;
      pt->vy_ = obj.fused_info_.v_y_;
    }

    TrjPointInfos* prev_pt = obj.traced_trj_[1].Back();
    Float32_t time_elapsed = 0.0F - prev_pt->relative_time_ms_;

    Float32_t avg_x = 0.0F;
    Float32_t avg_y = 0.0F;
    Float32_t avg_v_x = 0.0F;
    Float32_t avg_v_y = 0.0F;
    Float32_t avg_a_x = 0.0F;
    Float32_t avg_a_y = 0.0F;

    common::RingBuffer<TrjPointInfos,
        MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it =
        obj.traced_trj_[0].begin();
    common::RingBuffer<TrjPointInfos,
            MAX_TRACKED_OBJ_TRJ_POINT_NUM>::iterator trj_it_end =
        obj.traced_trj_[0].end();
    Int32_t trj_num = 0;
    for (; trj_it != trj_it_end; ++trj_it) {
      TrjPointInfos* pt = trj_it.current();

      avg_x += pt->x_;
      avg_y += pt->y_;

      trj_num++;
    }
    avg_x /= trj_num;
    avg_y /= trj_num;

    avg_v_x = 0.5F*obj.fused_info_.v_x_ +
              0.5F * (avg_x - obj.avg_status_.x_) / time_elapsed;
    avg_v_y = 0.0F * obj.fused_info_.v_y_ +
              1.0F * (avg_y - obj.avg_status_.y_) / time_elapsed +
              1.3F * current_rel_pos_.yaw_rate * obj.fused_info_.pos_.range_;;
    avg_a_x = (avg_v_x - obj.avg_status_.v_x_) / time_elapsed;
    avg_a_y = (avg_v_y - obj.avg_status_.v_y_) / time_elapsed;
    // Kalman 3
    // predicted
    // x = A * x(t-1) + B * u
    common::Matrix<Float32_t, 6, 1> mat_x;

#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    mat_x(0) = obj.pos_status_(0) + obj.pos_status_(2) * time_elapsed;
    mat_x(1) = obj.pos_status_(1) + obj.pos_status_(3) * time_elapsed;
    mat_x(2) = obj.pos_status_(2);
    mat_x(3) = obj.pos_status_(3);
    mat_x(4) = 0;
    mat_x(5) = 0;
#else
    mat_x(0) = obj.pos_status_(0) + (obj.pos_status_(2) + 0.5F * obj.pos_status_(4) * time_elapsed) * time_elapsed;
    mat_x(1) = obj.pos_status_(1) + (obj.pos_status_(3) + 0.5F * obj.pos_status_(5) * time_elapsed) * time_elapsed;
    mat_x(2) = obj.pos_status_(2) + obj.pos_status_(4) * time_elapsed;
    mat_x(3) = obj.pos_status_(3) + obj.pos_status_(5) * time_elapsed;
    mat_x(4) = obj.pos_status_(4);
    mat_x(5) = obj.pos_status_(5);
#endif

    if(mat_x(0)<0){

#if ENABLE_OTHER_DEBUG_PRINTF 
    LOG_INFO(2) << "=====================================================";
    LOG_INFO(2) << "kalman time_elapsed:" << time_elapsed;
    LOG_INFO(2) << "K-1|K-1:(" << obj.pos_status_(0) << "," << obj.pos_status_(1) << ") v0:(" << obj.pos_status_(2) << ", " << obj.pos_status_(3) << ")" 
                << " a0:(" << obj.pos_status_(4) << "," << obj.pos_status_(5) << ")";
    LOG_INFO(2) << "K|K-1 s:(" << mat_x(0) << "," << mat_x(1) << ")  v(" << mat_x(2) << "," << mat_x(3)
                << ")";
    LOG_INFO(2)<<"sensor s:(" << obj.fused_info_.pos_.x_ << "," << obj.fused_info_.pos_.y_ << ")  v(" << obj.fused_info_.v_x_ << "," << obj.fused_info_.v_y_
                << ")";
#endif
    }
    
    // P = A * P_ * A.transpose() + R;
    common::Matrix<Float32_t, 6, 6> mat_P;
    mat_P = obj.pos_covariance_;

    mat_P(0, 0) += 4.0F;
    mat_P(1, 1) += 4.0F;
    mat_P(2, 2) += 1.5F ;//0.1
    mat_P(3, 3) += 1.5F ;//0.1
    mat_P(4, 4) += 4.0F;
    mat_P(5, 5) += 4.0F;

    // measurement
    // Identity
    // 计算kalman增益矩阵 [ K = P * C_t * (C * P * C_t + Q)^-1 ]
    // C * P * C_t
    common::Matrix<Float32_t, 6, 6> mat_tmp_6v6;
    common::Matrix<Float32_t, 6, 6> mat_tmp_6v6_1;
    // C * P * C_t + Q
    mat_tmp_6v6 = mat_P;
    mat_tmp_6v6(0, 0) += 4.0F + obj.fused_info_.pos_.range_*0.2F;
    mat_tmp_6v6(1, 1) += 4.0F + obj.fused_info_.pos_.range_*0.5F;
    mat_tmp_6v6(2, 2) += 1.5F;//0.1
    mat_tmp_6v6(3, 3) += 1.5F;//0.1
    mat_tmp_6v6(4, 4) += 4.0F;
    mat_tmp_6v6(5, 5) += 4.0F;

    // (C * P * C_t + Q)^-1 ]
    common::Mat_CalcPseudoInverse(mat_tmp_6v6, mat_tmp_6v6_1);
    // 增益矩阵K
    common::Matrix<Float32_t, 6, 6> mat_K;
    common::Mat_Mul(mat_P, mat_tmp_6v6_1, mat_K);
    // 使用观测量修正状态量 [ pos_status = x + K(z - C*x) ]
    obj.avg_status_.x_ = avg_x;
    obj.avg_status_.y_ = avg_y;
    obj.avg_status_.v_x_ = avg_v_x;
    obj.avg_status_.v_y_ = avg_v_y;
#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    obj.avg_status_.a_x_ = 0;
    obj.avg_status_.a_y_ = 0;
#else
    obj.avg_status_.a_x_ = avg_a_x;
    obj.avg_status_.a_y_ = avg_a_y;
#endif


    common::Matrix<Float32_t, 6, 1> mat_z;
    common::Matrix<Float32_t, 6, 1> mat_tmp_6v1;
    mat_z(0) = obj.fused_info_.pos_.x_ - mat_x(0);
    mat_z(1) = obj.fused_info_.pos_.y_ - mat_x(1);
    //mat_z(2) = avg_v_x - mat_x(2);
    //mat_z(3) = avg_v_y - mat_x(3);
    mat_z(2)=obj.fused_info_.v_x_ - mat_x(2);
    mat_z(3)=obj.fused_info_.v_y_ - mat_x(3);
#if REMOVE_ACCEL_CALC_OF_SMOOTH_FUN
    mat_z(4) = 0;
    mat_z(5) = 0;
#else
    mat_z(4) = avg_a_x - mat_x(4);
    mat_z(5) = avg_a_y - mat_x(5);
#endif

    common::Mat_Mul(mat_K, mat_z, mat_tmp_6v1);
    common::Mat_Add(mat_x, mat_tmp_6v1, obj.pos_status_);
    // 修正Covariance [ covariance = (I - K * C) * P ]
    common::Matrix<Float32_t, 6, 6> mat_I_6v6;
    mat_I_6v6.SetIdentity();
    common::Mat_Sub(mat_I_6v6, mat_K, mat_tmp_6v6);
    common::Mat_Mul(mat_tmp_6v6, mat_P, obj.pos_covariance_);

    pt = obj.traced_trj_[1].AllocateOverride();
    if (Nullptr_t != pt) {
      pt->seq_num_ = 0;
      pt->relative_time_ms_ = 0.0F;
      pt->x_ = obj.pos_status_(0);
      pt->y_ = obj.pos_status_(1);
      pt->vx_ = obj.fused_info_.v_x_;
      pt->vy_ = obj.fused_info_.v_y_;
    }

    Float32_t dist_start_end = common::com_sqrt(
          common::Square(obj.pos_status_(0) - obj.traced_trj_[1][0].x_) +
        common::Square(obj.pos_status_(1) - obj.traced_trj_[1][0].y_));
    //Scalar avg_v_from_pos = dist_start_end /
    //    (0.0F - obj.traced_trj[0].relative_time_ms);
    Int32_t tracked_trj_size = obj.traced_trj_[1].Size();
    if ((tracked_trj_size > 8) && (obj_v > 5.0F/3.6F) &&
        (dist_start_end > 4.0F)) {
      obj.heading_ = common::com_atan2(obj.pos_status_(1) - obj.traced_trj_[1][0].y_,
          obj.pos_status_(0) - obj.traced_trj_[1][0].x_);

        if((obj.pos_status_(0) - obj.traced_trj_[1][0].x_)>0){

            obj.heading_=obj.heading_;

        }else{

            obj.heading_=-obj.heading_;

        }

    } else {
      Float32_t est_heading = 0.0F;
      Float32_t diff0 =
          common::com_abs(common::AngleDiff(obj.heading_, 0.0F));
      Float32_t diff1 =
          common::com_abs(common::AngleDiff(obj.heading_, (Float32_t)COM_PI));
      if (diff0 > diff1) {
        est_heading = COM_PI;
      }
      obj.heading_ = common::AngleLerp(obj.heading_, est_heading, 0.1F);
    }
    if(obj.pos_status_(0)>0){

#if ENABLE_OTHER_DEBUG_PRINTF 
      LOG_INFO(2) << "K|K s:(" << obj.pos_status_(0)<< "," << obj.pos_status_(1) << ")  v(" << obj.pos_status_(2) << "," << obj.pos_status_(3)
                << ")";
      LOG_INFO(2) << "=====================================================";
#endif

    }

  }
      
  for (Int32_t i = 0; i < tracked_objects_.objects_.Size(); ++i) {

      ObjsTracked& src_obj = tracked_objects_.objects_[i];
    
      if(src_obj.confidence_>60 && src_obj.heading_<0.0 && src_obj.fused_info_.v_x_>5.0){

        src_obj.heading_=common::com_atan2(src_obj.fused_info_.v_y_,src_obj.fused_info_.v_x_);

    }

  }

#if ENABLE_OBJ_TRACKER_TRACE
  std::cout << "### ObjFilterImpl::SmoothTrackedObjs (End) ###" << std::endl;
#endif
}

Int32_t ObjsTracker::getTrackedObjectsNums() const
{
    return tracked_objects_.objects_.Size();
}

void ObjsTracker::ShowTrackedObject(const Int32_t obj_index, const ObjsTracked &obj) const
{
#if ENABLE_OBJ_TRACKER_TRACE
  Int32_t radar_sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
  if (obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
    radar_sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
  }
  printf("        obj[%d] erase=%d"
         ", list_id=%d, radar=%d, raw_obj=%d"
         ", pos={esr(%d)[%0.1f,%0.1f] cam(%d)[%0.1f,%0.1f]}"
         ", range=%0.1f, angle=%0.1fdeg"
         ", vx=%0.1fkm/h, vy=%0.1fkm/h"
         ", life=%d, matched=%d, age=%d, duration=%d, confidence=%d"
         "\n",
         obj_index, obj.erase_flag_,
         obj.list_id_, radar_sen_id, obj.raw_obj_idx_,
         obj.sensor_ref_[radar_sen_id].valid_,
         obj.sensor_ref_[radar_sen_id].pos_[0].x_,
         obj.sensor_ref_[radar_sen_id].pos_[0].y_,
         obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].valid_,
         obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].pos_[0].x_,
         obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].pos_[0].y_,
         obj.sensor_ref_[radar_sen_id].pos_[0].range_,
         common::com_rad2deg(obj.sensor_ref_[radar_sen_id].pos_[0].angle_),
         obj.sensor_ref_[radar_sen_id].v_x_*3.6F,
         obj.sensor_ref_[radar_sen_id].v_y_*3.6F,
         obj.life_, obj.matched_, obj.age_, obj.duration_, obj.confidence_
         );
  printf("          trk_list:");
  ObjAssoListType::const_iterator it =
      obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].cbegin(obj_ass_pool_);
  ObjAssoListType::const_iterator it_end =
      obj.objs_ass_.objs_ass_list_[LIST_ID_TRACKED].cend(obj_ass_pool_);
  for (; it != it_end; ++it) {
    const ObjsAssociation* obj_ass = it.current();
    printf(" {obj[%d],cost=%d,dist=%0.1f,mis=%d}",
           obj_ass->dist_to_tar_obj_, obj_ass->cost_,
           obj_ass->dist_to_tar_obj_, obj_ass->mismatch_);
  }
  printf("\n          esr_list:");
  it = obj.objs_ass_.objs_ass_list_[radar_sen_id].cbegin(obj_ass_pool_);
  it_end = obj.objs_ass_.objs_ass_list_[radar_sen_id].cend(obj_ass_pool_);
  for (; it != it_end; ++it) {
    const ObjsAssociation* obj_ass = it.current();
    printf(" {obj[%d],cost=%d,dist=%0.1f,mis=%d}",
           obj_ass->tar_obj_index_.obj_idx_, obj_ass->cost_,
           obj_ass->dist_to_tar_obj_, obj_ass->mismatch_);
  }
  printf("\n          cam_list:");
  it = obj.objs_ass_.objs_ass_list_[LIST_ID_MAIN_FORWARD_CAM].cbegin(obj_ass_pool_);
  it_end = obj.objs_ass_.objs_ass_list_[LIST_ID_MAIN_FORWARD_CAM].cend(obj_ass_pool_);
  for (; it != it_end; ++it) {
    const ObjsAssociation* obj_ass = it.current();
    printf(" {obj[%d],cost=%d,dist=%0.1f,mis=%d}",
           obj_ass->tar_obj_index_.obj_idx_, obj_ass->cost_,
           obj_ass->dist_to_tar_obj_, obj_ass->mismatch_);
  }
  printf("\n");
#endif
}

void ObjsTracker::ShowTrackedObjectsList() const
{
#if ENABLE_OBJ_TRACKER_TRACE
  Int32_t obj_num = tracked_objects_.objects_.Size();

  for (Int32_t i = 0; i < obj_num; ++i) {
    const ObjsTracked& obj = tracked_objects_.objects_[i];

    Int32_t radar_sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
    if (obj.sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
      radar_sen_id = SENSOR_ID_MAIN_FORWARD_RADAR;
    }
    printf("    [%d] erase=%d"
           ", list=%d, radar=%d, raw_obj=%d"
           ", pos={esr(%d)[%0.1f,%0.1f] cam(%d)[%0.1f,%0.1f]}"
           ", range=%0.1f, angle=%0.1fdeg"
           ", vx=%0.1fkm/h, vy=%0.1fkm/h"
           ", life=%d, matched=%d, age=%d, duration=%d, confidence=%d"
           ", shape={esr[w=%0.1f,h=%0.1f],cam[w=%0.1f,h=%0.1f]}",
           i,
           obj.erase,
           obj.list_id, radar_sen_id, obj.raw_obj_idx,
           obj.sensor_ref[radar_sen_id].valid,
           obj.sensor_ref[radar_sen_id].pos[0].x,
           obj.sensor_ref[radar_sen_id].pos[0].y,
           obj.sensor_ref[SENSOR_ID_CAM_0].valid,
           obj.sensor_ref[SENSOR_ID_CAM_0].pos[0].x,
           obj.sensor_ref[SENSOR_ID_CAM_0].pos[0].y,
           obj.sensor_ref[radar_sen_id].pos[0].range,
           common::com_rad2deg(obj.sensor_ref[radar_sen_id].pos[0].angle),
           obj.sensor_ref[radar_sen_id].v_x*3.6F,
           obj.sensor_ref[radar_sen_id].v_y*3.6F,
           obj.life, obj.matched, obj.age, obj.duration, obj.confidence,
           obj.sensor_ref[radar_sen_id].width,
           obj.sensor_ref[radar_sen_id].length,
           obj.sensor_ref[SENSOR_ID_CAM_0].width,
           obj.sensor_ref[SENSOR_ID_CAM_0].length
           );
    printf(", id=esr{");
    for (Int32_t j = 0; j < ObjTracked::SensorRef::MAX_OBJ_ID_NUM; ++j) {
      printf("%d,", obj.sensor_ref[radar_sen_id].obj_id[j].id);
    }
    printf("} cam={");
    for (Int32_t j = 0; j < ObjTracked::SensorRef::MAX_OBJ_ID_NUM; ++j) {
      printf("%d,", obj.sensor_ref[SENSOR_ID_CAM_0].obj_id[j].id);
    }
    printf("}\n");
  }
#endif
}

void ObjsTracker::GetVehicleParams(struct EgoVehicleParams& vehicle_params) 
{
  // 车长，单位：米
  vehicle_params_.vehicle_length_ = vehicle_params.vehicle_length_;
  // 车宽，单位：米
  vehicle_params_.vehicle_width_ = vehicle_params.vehicle_width_;
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  vehicle_params_.dist_of_localization_to_front_ =
      vehicle_params.dist_of_localization_to_front_;
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  vehicle_params_.dist_of_localization_to_rear_ =
      vehicle_params.dist_of_localization_to_rear_;
  // 车辆的定位点到中心点的距离，单位：米
  vehicle_params_.dist_of_localization_to_center_ =
      vehicle_params.dist_of_localization_to_center_;
}


} //sensorsfusion
} //perception
} //phoenix
