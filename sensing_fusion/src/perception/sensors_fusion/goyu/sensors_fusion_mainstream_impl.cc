#include "vehicle_model_wrapper.h"
#include "sensors_fusion_mainstream_impl.h"
#include "pos_filter/include/pos_filter_wrapper.h"

#include "perception/sensors_fusion/main_camera/main_camera_handler.h"
#include "common/module_status.h"


namespace phoenix{
namespace perception{
namespace sensorsfusion{
namespace goyu{

#define ENABLE_OBJ_FILTER_IMPL_TRACE 0

#define  ENABLE_OBJ_FILTER_IMPL_1_TRACE 0


using namespace framework;


SensorsFusionMainstreamImpl::SensorsFusionMainstreamImpl()
{
    veh_model::VehicleModelWrapper veh_model;

    // 车长，单位：米
    vehicle_params_.vehicle_length_ = veh_model.GetVehicleLength();
    // 车宽，单位：米
    vehicle_params_.vehicle_width_ = veh_model.GetVehicleWidth();
    // 车辆的定位点到 front of vehicle 的距离，单位：米
    vehicle_params_.dist_of_localization_to_front_ =
        veh_model.GetDistOfLocalizationToFront();
    // 车辆的定位点到 rear of vehicle 的距离，单位：米
    vehicle_params_.dist_of_localization_to_rear_ =
        veh_model.GetDistOfLocalizationToRear();
    // 车辆的定位点到中心点的距离，单位：米
    vehicle_params_.dist_of_localization_to_center_ =
        veh_model.GetDistOfLocalizationToCenter();

    // 将车身参数传递给obj_tracker_
    obj_tracker_.GetVehicleParams(vehicle_params_);

    Clear();

}

SensorsFusionMainstreamImpl::~SensorsFusionMainstreamImpl()
{

}

Int32_t SensorsFusionMainstreamImpl::ProcessSensorsFusionPipeline(const ObjsFilterDataSource& recv_sensors_data)
{


    return Update(recv_sensors_data);

    /*************************
     * 这两项放在taskmanager中实现，不在该模块实现
    //发送决策
    //发送给HMI
    *************************/
}

void SensorsFusionMainstreamImpl::Clear()
{
//    vehicle_params_.Clear();

    rel_pos_list_.Clear();
    current_rel_pos_.Clear();

    objs_from_main_forward_cam_.Clear();
    objs_from_center_forward_cam_.Clear();
    objs_from_left_side_cam_.Clear();
    objs_from_left_backward_cam_.Clear();
    objs_from_right_side_cam_.Clear();
    objs_from_right_backward_cam_.Clear();

    objs_from_main_forward_radar_.Clear();
    objs_from_left_side_radar_.Clear();
    objs_from_left_backward_radar_.Clear();
    objs_from_right_side_radar_.Clear();
    objs_from_right_backward_radar_.Clear();

    objs_from_main_forward_lidar_.Clear();

    obj_tracker_.Clear();
}

Int32_t SensorsFusionMainstreamImpl::Update(const ObjsFilterDataSource &data_source)
{
#if ENABLE_OBJ_FILTER_IMPL_TRACE
  std::cout << "\n### ObjFilterImpl::Update (Begin) ###" << std::endl;
#endif

#if ENABLE_OBJ_FILTER_IMPL_PERFORMANCE_TEST
  /// Performance of Obj filter (Start)
  phoenix::common::Stopwatch performance_timer_obj_filter;
#endif

  Int32_t status = PERCEPTION_MODULE_STATUS_OK;
  Int64_t timestamp = data_source.timestamp_;

  // 如果没有相对位置信息，则无法进行空间及时间同步。返回错误
  if (Nullptr_t == data_source.rel_pos_list_) {
    status = PERCEPTION_MODULE_STATUS_ERR_EMPTY_REL_POS_LIST_FAULT;
    return status;
  }
  if (data_source.rel_pos_list_->relative_pos_num < 1) {
    status = PERCEPTION_MODULE_STATUS_ERR_EMPTY_REL_POS_LIST_FAULT;
    return status;
  }

  // 获取车辆当前位置  
  if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
      data_source.timestamp_, *data_source.rel_pos_list_, &current_rel_pos_)) {
    LOG_ERR << "[Error]Failed to get current posistion from relative position list.";
    status = PERCEPTION_MODULE_STATUS_ERR_GET_CUR_POS_FAULT;
    return status;
  }

  // 保存车辆当前位置信息
  rel_pos_list_ = *(data_source.rel_pos_list_);
  obj_tracker_.SetRelativePosList(rel_pos_list_);
  obj_tracker_.SetCurrentRelPos(current_rel_pos_);

  // 更新之前跟踪的障碍物
  if (!obj_tracker_.PreProcessForAddingNewObj(timestamp)) {
    LOG_ERR << "[Error]Failed to update obj tracked list.";
    status = PERCEPTION_MODULE_STATUS_PREPROCESS_TRACKED_OBJS_FAULT;
    return status;
  }

  // 添加相机识别的障碍物列表
  Int64_t time_elapsed_cam_0 = 0;
#if (ENABLE_MAIN_CAMERA_PRE_PROCESS)
  bool valid_cam_6 = AddObjsListMainCamera_80(
        SENSOR_ID_MAIN_FORWARD_CAM, data_source, &time_elapsed_cam_0);
  bool valid_cam_7 = AddObjsListMainCamera_150(
        SENSOR_ID_MAIN_FORWARD_CAM, data_source, &time_elapsed_cam_0);
#else
  bool valid_cam_0 = AddObjsListCamera(
        SENSOR_ID_MAIN_FORWARD_CAM, data_source, &time_elapsed_cam_0);
#endif

  //添加环视处理器识别的障碍物列表
#if 1
  Int64_t time_elapsed_cam_1 = 0;
  bool valid_cam_1 = AddObjsListCamera(
        SENSOR_ID_CENTER_FORWARD_CAM, data_source, &time_elapsed_cam_1);
  Int64_t time_elapsed_cam_2 = 0;
  bool valid_cam_2 = AddObjsListCamera(
        SENSOR_ID_LEFT_BACKWARD_CAM, data_source, &time_elapsed_cam_2);
  Int64_t time_elapsed_cam_3 = 0;
  bool valid_cam_3 = AddObjsListCamera(
        SENSOR_ID_LEFT_SIDE_CAM, data_source, &time_elapsed_cam_3);
  Int64_t time_elapsed_cam_4 = 0;
  bool valid_cam_4 = AddObjsListCamera(
        SENSOR_ID_RIGHT_BACKWARD_CAM, data_source, &time_elapsed_cam_4);
  Int64_t time_elapsed_cam_5 = 0;
  bool valid_cam_5 = AddObjsListCamera(
        SENSOR_ID_RIGHT_SIDE_CAM, data_source, &time_elapsed_cam_5);

#endif

  // 添加毫米波雷达识别的障碍物列表(Radar front)
  Int64_t time_elapsed_radar_0 = 0;
  bool valid_radar_0 = AddObjsListRadar(
        SENSOR_ID_MAIN_FORWARD_RADAR, data_source, &time_elapsed_radar_0);
#if 1
  // 添加毫米波雷达识别的障碍物列表
  Int64_t time_elapsed_radar_1 = 0;
  bool valid_radar_1 = AddObjsListRadar(
        SENSOR_ID_LEFT_BACKWARD_RADAR, data_source, &time_elapsed_radar_1);
  Int64_t time_elapsed_radar_2 = 0;
  bool valid_radar_2 = AddObjsListRadar(
        SENSOR_ID_LEFT_SIDE_RADAR, data_source, &time_elapsed_radar_2);
  Int64_t time_elapsed_radar_3 = 0;
  bool valid_radar_3 = AddObjsListRadar(
        SENSOR_ID_RIGHT_BACKWARD_RADAR, data_source, &time_elapsed_radar_3);
  Int64_t time_elapsed_radar_4 = 0;
  bool valid_radar_4 = AddObjsListRadar(
        SENSOR_ID_RIGHT_SIDE_RADAR, data_source, &time_elapsed_radar_4);
#endif

  // 添加Lidar识别的障碍物列表
  Int64_t time_elapsed_lidar_0 = 0;
  bool valid_lidar_0 = AddObjsListLidar(
        SENSOR_ID_MAIN_FORWARD_LIDAR, data_source, &time_elapsed_lidar_0);

  // 完成新障碍物的添加
  obj_tracker_.PostProcessForAddingNewObj();

  /// 先对毫米波雷达进行跟踪，再将相机数据与毫米波的数据进行匹配融合
  // 使用前向毫米波雷达的障碍物列表过滤障碍物跟踪列表
  if (valid_radar_0) {
    FilterByRadar(SENSOR_ID_MAIN_FORWARD_RADAR, time_elapsed_radar_0*0.001F);
  }

  // 使侧向毫米波雷达的障碍物列表过滤障碍物跟踪列表
  if (valid_radar_1) {
    FilterByRadar(SENSOR_ID_LEFT_BACKWARD_RADAR, time_elapsed_radar_1*0.001F);
  }
  if (valid_radar_2) {
    FilterByRadar(SENSOR_ID_LEFT_SIDE_RADAR, time_elapsed_radar_2*0.001F);
  }
  if (valid_radar_3) {
    FilterByRadar(SENSOR_ID_RIGHT_BACKWARD_RADAR, time_elapsed_radar_3*0.001F);
  }
  if (valid_radar_4) {
    FilterByRadar(SENSOR_ID_RIGHT_SIDE_RADAR, time_elapsed_radar_4*0.001F);
  }


  // 使视觉处理器处理障碍物列表过滤障碍物跟踪列表
  if (valid_cam_1) {
    FilterByCamera(SENSOR_ID_CENTER_FORWARD_CAM, time_elapsed_cam_1*0.001F);
  }
  if (valid_cam_2) {
    FilterByCamera(SENSOR_ID_LEFT_BACKWARD_CAM, time_elapsed_cam_2*0.001F);
  }
  if (valid_cam_3) {
    FilterByCamera(SENSOR_ID_LEFT_SIDE_CAM, time_elapsed_cam_3*0.001F);
  }
  if (valid_cam_4) {
    FilterByCamera(SENSOR_ID_RIGHT_BACKWARD_CAM, time_elapsed_cam_4*0.001F);
  }
  if (valid_cam_5) {
    FilterByCamera(SENSOR_ID_RIGHT_SIDE_CAM, time_elapsed_cam_5*0.001F);
  }



  // 使用Cam的障碍物列表过滤障碍物跟踪列表
#if (ENABLE_MAIN_CAMERA_PRE_PROCESS)
  if (valid_cam_6) {
    FilterByMainCamera_80(SENSOR_ID_MAIN_FORWARD_CAM, time_elapsed_cam_0*0.001F);
  }
  if (valid_cam_7) {
    FilterByMainCamera_150(SENSOR_ID_MAIN_FORWARD_CAM, time_elapsed_cam_0*0.001F);
  }
#else
  if (valid_cam_0) {
    FilterByCamera(SENSOR_ID_MAIN_FORWARD_CAM, time_elapsed_cam_0*0.001F);
  }
#endif

  // 使用Lidar的障碍物列表过滤障碍物跟踪列表
  if (valid_lidar_0) {
    FilterByLidar(SENSOR_ID_MAIN_FORWARD_LIDAR, time_elapsed_lidar_0*0.001F);
  }

  // 跟新OBJ跟踪列表的内部状态
  obj_tracker_.UpdateTrackList(data_source);

#if ENABLE_OBJ_FILTER_IMPL_1_TRACE
  std::cout << "\n### ObjFilterImpl::Update (End) ###" << std::endl;
#endif

  status = PERCEPTION_MODULE_STATUS_OK;
  return status;
}

void SensorsFusionMainstreamImpl::GetObstacleTrackedInfoList(ad_msg::ObstacleTrackedInfoList *obj_list)
{
    obj_tracker_.GetObstacleTrackedInfoList(obj_list);
}


Int32_t SensorsFusionMainstreamImpl::CheckSourceDataAndTrackedInfoList(const ObjsFilterDataSource& recv_sensors_data,
    const ad_msg::ObstacleList& obj_list)
{
  Int32_t status = PERCEPTION_MODULE_STATUS_OK;

  // recv_sensors_data

  bool is_main_camera_objlist_empty = false;
  bool is_forward_radar_objlist_empty = false;
  
  bool is_left_side_radar_objlist_empty = false;
  bool is_right_side_radar_objlist_empty = false;
  bool is_left_backward_radar_objlist_empty = false;
  bool is_right_backward_radar_objlist_empty = false;

  bool is_center_forward_cam_objlist_empty = false;
  bool is_left_side_cam_objlist_empty = false;
  bool is_right_side_cam_objlist_empty = false;
  bool is_left_backward_cam_objlist_empty = false;
  bool is_right_backward_cam_objlist_empty = false;

#if (ENABLE_MAIN_CAMERA_PRE_PROCESS)
  is_main_camera_objlist_empty  = (recv_sensors_data.objs_list_main_forward_cam_80_ == Nullptr_t || recv_sensors_data.objs_list_main_forward_cam_80_->obstacle_num <= 0) && \
                                  (recv_sensors_data.objs_list_main_forward_cam_150_ == Nullptr_t || recv_sensors_data.objs_list_main_forward_cam_150_->obstacle_num <= 0);
#else
  is_main_camera_objlist_empty  = (recv_sensors_data.objs_list_main_forward_cam_ == Nullptr_t || recv_sensors_data.objs_list_main_forward_cam_->obstacle_num <= 0);
#endif

  is_forward_radar_objlist_empty =  (recv_sensors_data.objs_list_main_forward_radar_ == Nullptr_t || recv_sensors_data.objs_list_main_forward_radar_->obstacle_num <= 0);
  
  is_left_side_radar_objlist_empty =  (recv_sensors_data.objs_list_left_side_radar_ == Nullptr_t || recv_sensors_data.objs_list_left_side_radar_->obstacle_num <= 0);
  is_right_side_radar_objlist_empty =  (recv_sensors_data.objs_list_right_side_radar_ == Nullptr_t || recv_sensors_data.objs_list_right_side_radar_->obstacle_num <= 0);
  is_left_backward_radar_objlist_empty =  (recv_sensors_data.objs_list_left_backward_radar_ == Nullptr_t || recv_sensors_data.objs_list_left_backward_radar_->obstacle_num <= 0);
  is_right_backward_radar_objlist_empty =  (recv_sensors_data.objs_list_right_backward_radar_ == Nullptr_t || recv_sensors_data.objs_list_right_backward_radar_->obstacle_num <= 0);

  is_center_forward_cam_objlist_empty =  (recv_sensors_data.objs_list_center_forward_cam_ == Nullptr_t || recv_sensors_data.objs_list_center_forward_cam_->obstacle_num <= 0);
  is_left_side_cam_objlist_empty =  (recv_sensors_data.objs_list_left_side_cam_ == Nullptr_t || recv_sensors_data.objs_list_left_side_cam_->obstacle_num <= 0);
  is_right_side_cam_objlist_empty =  (recv_sensors_data.objs_list_left_backward_radar_ == Nullptr_t || recv_sensors_data.objs_list_left_backward_radar_->obstacle_num <= 0);
  is_left_backward_cam_objlist_empty =  (recv_sensors_data.objs_list_right_side_cam_ == Nullptr_t || recv_sensors_data.objs_list_right_side_cam_->obstacle_num <= 0);
  is_right_backward_cam_objlist_empty =  (recv_sensors_data.objs_list_right_backward_cam_ == Nullptr_t || recv_sensors_data.objs_list_right_backward_radar_->obstacle_num <= 0);

  if(is_main_camera_objlist_empty && is_forward_radar_objlist_empty && is_left_side_radar_objlist_empty && is_right_side_radar_objlist_empty && is_left_backward_radar_objlist_empty && \
      is_right_backward_radar_objlist_empty && is_center_forward_cam_objlist_empty && is_left_side_cam_objlist_empty && is_left_backward_cam_objlist_empty && is_right_backward_cam_objlist_empty)
      {
        status = PERCEPTION_MODULE_STATUS_ERR_NO_SENSOR_OBJ_INPUT_FAULT;
        return status;
      }

  if (obj_list.obstacle_num == 0)
  {
    status = PERCEPTION_MODULE_STATUS_ERR_NO_PERCEPTION_OBJ_RESULT_FAULT;
    return status;
  }

  if(is_main_camera_objlist_empty == false && is_forward_radar_objlist_empty == false)
  {
    bool have_per_multi_sensors_res = false;
    for(int i = 0; i < obj_list.obstacle_num; i++)
    {
      const ad_msg::Obstacle&  obstacle = obj_list.obstacles[i];
      Int8_t radar_type, camera_type, exist_lidar;
      Uint8_t radar_id, camera_id, lidar_id;
      bool ret =  ad_msg::Obstacle::DecodeFusionID(obstacle.id, radar_type, camera_type, exist_lidar,radar_id, camera_id, lidar_id);
      if (obstacle.perception_type == ad_msg::OBJ_PRCP_TYPE_FUSED && radar_type == ad_msg::Obstacle::MAIN_FRONT_RADAR_INDEX_FOR_ID \
          && camera_type == ad_msg::Obstacle::MAIN_FRONT_CAMERA_INDEX_FOR_ID)
      {
        have_per_multi_sensors_res = true;
        break;
      }
    } 
    
    if (have_per_multi_sensors_res == false)
    {
      status = PERCEPTION_MODULE_STATUS_ERR_NO_1V1R_PERCEPTION_FAULT;
      return status;
    }

  }
  return status;
}



void SensorsFusionMainstreamImpl::ObstaclesListMatches()
{
    LOG_INFO(3) << "TODO: refactor the obstacles lists matches on derived class.";
}

void SensorsFusionMainstreamImpl::ObstaclesPrediction()
{
    LOG_INFO(3) << "TODO: refactor the obstacles lists predicition on derived class.";
}

void SensorsFusionMainstreamImpl::ObstaclesTracking()
{
    LOG_INFO(3) << "TODO: refactor the obstacles lists tracking on derived class.";
}

Int32_t SensorsFusionMainstreamImpl::GetObstaclesLists(ad_msg::ObstacleList *obj_list)
{
    obj_tracker_.GetObstaclesLists(obj_list);
    return obj_tracker_.getTrackedObjectsNums();
}

bool SensorsFusionMainstreamImpl::AddObjsListCamera(const Int32_t sensor_id, const ObjsFilterDataSource &data_source, Int64_t *time_elapsed)
{
    bool valid_list = false;

    const ad_msg::ObstacleCameraList* src_obj_list = Nullptr_t;
    ObjsFromCameraList* dst_obj_list = Nullptr_t;
    bool valid_sensor = true;
    switch (sensor_id) {
    case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):
        src_obj_list = data_source.objs_list_main_forward_cam_;
        dst_obj_list = &objs_from_main_forward_cam_;
      break;
    case (SensorID::SENSOR_ID_LEFT_BACKWARD_CAM):
        src_obj_list = data_source.objs_list_left_backward_cam_;
        dst_obj_list = &objs_from_left_backward_cam_;
      break;
    case (SensorID::SENSOR_ID_LEFT_SIDE_CAM):
        src_obj_list = data_source.objs_list_left_side_cam_;
        dst_obj_list = &objs_from_left_side_cam_;
      break;
    case (SensorID::SENSOR_ID_RIGHT_BACKWARD_CAM):
        src_obj_list = data_source.objs_list_right_backward_cam_;
        dst_obj_list = &objs_from_right_backward_cam_;
      break;
    case (SensorID::SENSOR_ID_RIGHT_SIDE_CAM):
        src_obj_list = data_source.objs_list_right_side_cam_;
        dst_obj_list = &objs_from_right_side_cam_;
      break;
    case (SensorID::SENSOR_ID_CENTER_FORWARD_CAM):
        src_obj_list = data_source.objs_list_center_forward_cam_;
        dst_obj_list = &objs_from_center_forward_cam_;
      break;
    default:
        valid_sensor = false;
      break;
    }
    if (!valid_sensor) {
      LOG_ERR << "[Error]Invalid sensor ID.";
      return (valid_list);
    }

    if (Nullptr_t != src_obj_list) {
      if (src_obj_list->msg_head.valid) {
        if (src_obj_list->obstacle_num > 0) {
          *time_elapsed = common::CalcElapsedClockMs(
                src_obj_list->msg_head.timestamp, data_source.timestamp_);
          if (*time_elapsed > 500 || *time_elapsed < 0) {// UPPER_LIMIT_OF_TIME_ELAPSED_FOR_CAMERA_LIST_INPUT
            LOG_ERR << "[Error]Getting obstacles from camera is timeout";
            dst_obj_list->Clear();
          } else {
            if (!dst_obj_list->msg_head_.valid) {
              // 之前没有保存过相机识别的障碍物列表，
              // 所以当前传递进来的障碍物列表是有效的
              valid_list = true;
            } else {
              Int32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(  //两者相差不能超过10s
                    dst_obj_list->msg_head_.sequence,
                    src_obj_list->msg_head.sequence);
              if (sequence_diff > 0) {
                // 当前传递进来的障碍物列表是更新后的，所以是有效的
                valid_list = true;
              }
            }
          }
        }
      }
    }

    if (!valid_list) {
      return (valid_list);
    }

    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    // 将传递进来的相机识别的障碍物列表进行时间及空间同步(对齐到时间流中)
    dst_obj_list->objects_.Clear();
    ad_msg::RelativePos pos_sensor;
    common::Matrix<Float32_t, 3, 3> mat_conv_sensor;
    mat_conv_sensor.SetIdentity();
    if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          src_obj_list->msg_head.timestamp, rel_pos_list_, &pos_sensor)) {
      printf("[Error]Failed to get pos of camera 0x%02x.", sensor_id);
      valid_list = false;
      return (valid_list);
    } else {
      common::Rotate_2D<Float32_t>(rotate_center,
                                pos_sensor.heading - current_rel_pos_.heading,
                                &mat_conv_sensor);
      common::Translate_2D(pos_sensor.x - current_rel_pos_.x,
                           pos_sensor.y - current_rel_pos_.y,
                           &mat_conv_sensor);
    }

    common::Matrix<Float32_t, 2, 1> point_conv;
    common::Matrix<Float32_t, 3, 3> mat_conv_composite = mat_conv_sensor;

    dst_obj_list->Clear();
    dst_obj_list->msg_head_ = src_obj_list->msg_head;
    dst_obj_list->sensor_id_ = sensor_id;

    Int32_t obj_num = src_obj_list->obstacle_num;
    for (Int32_t i = 0; i < obj_num; ++i) {
      const ad_msg::ObstacleCamera& src_obj = src_obj_list->obstacles[i];
      ObjFromCamera* dst_obj = dst_obj_list->objects_.Allocate();
      if (Nullptr_t == dst_obj) {
        LOG_ERR << "[Error]Failed to add camera tracked obj, storage is full.";
        break;
      }

      dst_obj->tracked_obj_index_.Clear();

      dst_obj->raw_obj_idx_ = i;
      dst_obj->raw_obj_id_ = src_obj.id;
      switch (sensor_id) {
      case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):        //TODO: 将崔老师原始的传感器宏与sensors_fusion_macros.h中的顺序进行适配
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_MAIN_FORWARD_CAMERA;
        break;
      default:
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_UNKNOWN;
        break;
      }
      dst_obj->obj_type_ = src_obj.type;

      point_conv(0) = src_obj.x;
      point_conv(1) = src_obj.y;

      common::TransformVert_2D(mat_conv_composite, &point_conv);

      dst_obj->pos_.x_ = point_conv(0);
      dst_obj->pos_.y_ = point_conv(1);

#if ENABLE_POSITION_MAKE_UP      
      Float32_t time_diff = (*time_elapsed)*0.001;
      // dst_obj->pos_.x_ = dst_obj->pos_.x_ + (src_obj.v_x + 0.5 * src_obj.accel_x * time_diff) * time_diff;
      // dst_obj->pos_.y_ = dst_obj->pos_.y_ + (src_obj.v_y + 0.5 * src_obj.accel_y * time_diff) * time_diff;
      dst_obj->pos_.x_ = dst_obj->pos_.x_ + src_obj.v_x * time_diff;
      dst_obj->pos_.y_ = dst_obj->pos_.y_ + src_obj.v_y * time_diff;
      //LOG_INFO(2) << "make up:(" << dst_obj->pos_.x_ << "," << dst_obj->pos_.y_ << ")";
#endif




      dst_obj->pos_.range_ = common::com_sqrt(
            common::Square(dst_obj->pos_.x_) +
            common::Square(dst_obj->pos_.y_));
      dst_obj->pos_.angle_ =
          common::com_atan2(dst_obj->pos_.y_, dst_obj->pos_.x_);

      dst_obj->v_x_ = src_obj.v_x;
//      dst_obj->v_y_ = 0.0F;   //特殊处理
//      dst_obj->pos_.heading_ = 0.0F;
      dst_obj->v_y_ = src_obj.v_y;
      dst_obj->pos_.heading_ = src_obj.heading;

#if ENABLE_OUTPUT_ACCELERATION
      dst_obj->a_x_ = src_obj.accel_x;
      dst_obj->a_y_ = src_obj.accel_y;
#endif

      if (dst_obj->v_x_ < -5.0F/3.6F) {
        //dst_obj->pos_.heading_ = -COM_PI;
      }

      dst_obj->width_ = common::Max(src_obj.width, 0.5F); //TODO： 最小尺寸定为0.5米，根据传感器性能进行试验 //MIN_WIDTH_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->height_ = common::Max(src_obj.height, 0.5F);//MIN_HEIGHT_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->length_ = common::Max(src_obj.length, 0.5F);//MIN_LENGTH_OF_CAMERA_DATA_FOR_ADDING
      Float32_t half_width = 0.5F * dst_obj->width_;
      dst_obj->aabb_.min().set_x(dst_obj->pos_.x_);
      dst_obj->aabb_.min().set_y(dst_obj->pos_.y_ - half_width);
      dst_obj->aabb_.max().set_x(
            dst_obj->pos_.x_ + dst_obj->length_);
      dst_obj->aabb_.max().set_y(dst_obj->pos_.y_ + half_width);

      dst_obj->raw_data_ = src_obj;
    }

    obj_num = dst_obj_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjFromCamera& dst_obj = dst_obj_list->objects_[i];

      ObjIndex idx =
          obj_tracker_.AddNewObj(sensor_id, dst_obj);
      dst_obj.tracked_obj_index_ = idx;
    }
    if (obj_num < 1) {
      valid_list = false;
    }

    return (valid_list);
}

bool SensorsFusionMainstreamImpl::AddObjsListMainCamera_80(const Int32_t sensor_id, const ObjsFilterDataSource &data_source, Int64_t *time_elapsed)
{
    bool valid_list = false;

    const ad_msg::ObstacleCameraList* src_obj_list = Nullptr_t;
    ObjsFromCameraList* dst_obj_list = Nullptr_t;
    bool valid_sensor = true;
    switch (sensor_id) {
    case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):
        src_obj_list = data_source.objs_list_main_forward_cam_80_;
        dst_obj_list = &objs_from_main_forward_cam_80_;
      break;
    default:
        valid_sensor = false;
      break;
    }
    if (!valid_sensor) {
      LOG_ERR << "[Error]Invalid sensor ID.";
      return (valid_list);
    }

    if (Nullptr_t != src_obj_list) {
      if (src_obj_list->msg_head.valid) {
        if (src_obj_list->obstacle_num > 0) {
          *time_elapsed = common::CalcElapsedClockMs(
                src_obj_list->msg_head.timestamp, data_source.timestamp_);
          if (*time_elapsed > 500 || *time_elapsed < 0) {// UPPER_LIMIT_OF_TIME_ELAPSED_FOR_CAMERA_LIST_INPUT
            LOG_ERR << "[Error]Getting obstacles from camera is timeout";
            dst_obj_list->Clear();
          } else {
            if (!dst_obj_list->msg_head_.valid) {
              // 之前没有保存过相机识别的障碍物列表，
              // 所以当前传递进来的障碍物列表是有效的
              valid_list = true;
            } else {
              Int32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(  //两者相差不能超过10s
                    dst_obj_list->msg_head_.sequence,
                    src_obj_list->msg_head.sequence);
              if (sequence_diff > 0) {
                // 当前传递进来的障碍物列表是更新后的，所以是有效的
                valid_list = true;
              }
            }
          }
        }
      }
    }

    if (!valid_list) {
      return (valid_list);
    }

    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    // 将传递进来的相机识别的障碍物列表进行时间及空间同步(对齐到时间流中)
    dst_obj_list->objects_.Clear();
    ad_msg::RelativePos pos_sensor;
    common::Matrix<Float32_t, 3, 3> mat_conv_sensor;
    mat_conv_sensor.SetIdentity();
    if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          src_obj_list->msg_head.timestamp, rel_pos_list_, &pos_sensor)) {
      printf("[Error]Failed to get pos of camera 0x%02x.", sensor_id);
      valid_list = false;
      return (valid_list);
    } else {
      common::Rotate_2D<Float32_t>(rotate_center,
                                pos_sensor.heading - current_rel_pos_.heading,
                                &mat_conv_sensor);
      common::Translate_2D(pos_sensor.x - current_rel_pos_.x,
                           pos_sensor.y - current_rel_pos_.y,
                           &mat_conv_sensor);
    }

    common::Matrix<Float32_t, 2, 1> point_conv;
    common::Matrix<Float32_t, 3, 3> mat_conv_composite = mat_conv_sensor;

    dst_obj_list->Clear();
    dst_obj_list->msg_head_ = src_obj_list->msg_head;
    dst_obj_list->sensor_id_ = sensor_id;

    Int32_t obj_num = src_obj_list->obstacle_num;
    for (Int32_t i = 0; i < obj_num; ++i) {
      const ad_msg::ObstacleCamera& src_obj = src_obj_list->obstacles[i];
      ObjFromCamera* dst_obj = dst_obj_list->objects_.Allocate();
      if (Nullptr_t == dst_obj) {
        LOG_ERR << "[Error]Failed to add camera tracked obj, storage is full.";
        break;
      }

      dst_obj->tracked_obj_index_.Clear();

      dst_obj->raw_obj_idx_ = i;
      dst_obj->raw_obj_id_ = src_obj.id;
      switch (sensor_id) {
      case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):        //TODO: 将崔老师原始的传感器宏与sensors_fusion_macros.h中的顺序进行适配
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_MAIN_FORWARD_CAMERA;
        break;
      default:
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_UNKNOWN;
        break;
      }
      dst_obj->obj_type_ = src_obj.type;

      point_conv(0) = src_obj.x;
      point_conv(1) = src_obj.y;

      common::TransformVert_2D(mat_conv_composite, &point_conv);

      dst_obj->pos_.x_ = point_conv(0);
      dst_obj->pos_.y_ = point_conv(1);

#if ENABLE_POSITION_MAKE_UP      
      Float32_t time_diff = (*time_elapsed)*0.001;
      // dst_obj->pos_.x_ = dst_obj->pos_.x_ + (src_obj.v_x + 0.5 * src_obj.accel_x * time_diff) * time_diff;
      // dst_obj->pos_.y_ = dst_obj->pos_.y_ + (src_obj.v_y + 0.5 * src_obj.accel_y * time_diff) * time_diff;
      dst_obj->pos_.x_ = dst_obj->pos_.x_ + src_obj.v_x * time_diff;
      dst_obj->pos_.y_ = dst_obj->pos_.y_ + src_obj.v_y * time_diff;
      //LOG_INFO(2) << "make up:(" << dst_obj->pos_.x_ << "," << dst_obj->pos_.y_ << ")";
#endif

      dst_obj->pos_.range_ = common::com_sqrt(
            common::Square(dst_obj->pos_.x_) +
            common::Square(dst_obj->pos_.y_));
      dst_obj->pos_.angle_ =
          common::com_atan2(dst_obj->pos_.y_, dst_obj->pos_.x_);

      dst_obj->v_x_ = src_obj.v_x;
//      dst_obj->v_y_ = 0.0F;   //特殊处理
//      dst_obj->pos_.heading_ = 0.0F;
      dst_obj->v_y_ = src_obj.v_y;
      dst_obj->pos_.heading_ = src_obj.heading;

#if ENABLE_OUTPUT_ACCELERATION
      dst_obj->a_x_ = src_obj.accel_x;
      dst_obj->a_y_ = src_obj.accel_y;
#endif

      if (dst_obj->v_x_ < -5.0F/3.6F) {
        //dst_obj->pos_.heading_ = -COM_PI;
      }

      dst_obj->width_ = common::Max(src_obj.width, 0.5F); //TODO： 最小尺寸定为0.5米，根据传感器性能进行试验 //MIN_WIDTH_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->height_ = common::Max(src_obj.height, 0.5F);//MIN_HEIGHT_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->length_ = common::Max(src_obj.length, 0.5F);//MIN_LENGTH_OF_CAMERA_DATA_FOR_ADDING
      Float32_t half_width = 0.5F * dst_obj->width_;
      dst_obj->aabb_.min().set_x(dst_obj->pos_.x_);
      dst_obj->aabb_.min().set_y(dst_obj->pos_.y_ - half_width);
      dst_obj->aabb_.max().set_x(
            dst_obj->pos_.x_ + dst_obj->length_);
      dst_obj->aabb_.max().set_y(dst_obj->pos_.y_ + half_width);

      dst_obj->raw_data_ = src_obj;
    }

    obj_num = dst_obj_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjFromCamera& dst_obj = dst_obj_list->objects_[i];

      ObjIndex idx =
          obj_tracker_.AddNewObj(sensor_id, dst_obj);
      dst_obj.tracked_obj_index_ = idx;
    }
    if (obj_num < 1) {
      valid_list = false;
    }

    return (valid_list);
}

bool SensorsFusionMainstreamImpl::AddObjsListMainCamera_150(const Int32_t sensor_id, const ObjsFilterDataSource &data_source, Int64_t *time_elapsed)
{
    bool valid_list = false;

    const ad_msg::ObstacleCameraList* src_obj_list = Nullptr_t;
    ObjsFromCameraList* dst_obj_list = Nullptr_t;
    bool valid_sensor = true;
    switch (sensor_id) {
    case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):
        src_obj_list = data_source.objs_list_main_forward_cam_150_;
        dst_obj_list = &objs_from_main_forward_cam_150_;
      break;
    default:
        valid_sensor = false;
      break;
    }
    if (!valid_sensor) {
      LOG_ERR << "[Error]Invalid sensor ID.";
      return (valid_list);
    }

    if (Nullptr_t != src_obj_list) {
      if (src_obj_list->msg_head.valid) {
        if (src_obj_list->obstacle_num > 0) {
          *time_elapsed = common::CalcElapsedClockMs(
                src_obj_list->msg_head.timestamp, data_source.timestamp_);
          if (*time_elapsed > 500 || *time_elapsed < 0) {// UPPER_LIMIT_OF_TIME_ELAPSED_FOR_CAMERA_LIST_INPUT
            LOG_ERR << "[Error]Getting obstacles from camera is timeout";
            dst_obj_list->Clear();
          } else {
            if (!dst_obj_list->msg_head_.valid) {
              // 之前没有保存过相机识别的障碍物列表，
              // 所以当前传递进来的障碍物列表是有效的
              valid_list = true;
            } else {
              Int32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(  //两者相差不能超过10s
                    dst_obj_list->msg_head_.sequence,
                    src_obj_list->msg_head.sequence);
              if (sequence_diff > 0) {
                // 当前传递进来的障碍物列表是更新后的，所以是有效的
                valid_list = true;
              }
            }
          }
        }
      }
    }

    if (!valid_list) {
      return (valid_list);
    }

    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    // 将传递进来的相机识别的障碍物列表进行时间及空间同步(对齐到时间流中)
    dst_obj_list->objects_.Clear();
    ad_msg::RelativePos pos_sensor;
    common::Matrix<Float32_t, 3, 3> mat_conv_sensor;
    mat_conv_sensor.SetIdentity();
    if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          src_obj_list->msg_head.timestamp, rel_pos_list_, &pos_sensor)) {
      printf("[Error]Failed to get pos of camera 0x%02x.", sensor_id);
      valid_list = false;
      return (valid_list);
    } else {
      common::Rotate_2D<Float32_t>(rotate_center,
                                pos_sensor.heading - current_rel_pos_.heading,
                                &mat_conv_sensor);
      common::Translate_2D(pos_sensor.x - current_rel_pos_.x,
                           pos_sensor.y - current_rel_pos_.y,
                           &mat_conv_sensor);
    }

    common::Matrix<Float32_t, 2, 1> point_conv;
    common::Matrix<Float32_t, 3, 3> mat_conv_composite = mat_conv_sensor;

    dst_obj_list->Clear();
    dst_obj_list->msg_head_ = src_obj_list->msg_head;
    dst_obj_list->sensor_id_ = sensor_id;

    Int32_t obj_num = src_obj_list->obstacle_num;
    for (Int32_t i = 0; i < obj_num; ++i) {
      const ad_msg::ObstacleCamera& src_obj = src_obj_list->obstacles[i];
      ObjFromCamera* dst_obj = dst_obj_list->objects_.Allocate();
      if (Nullptr_t == dst_obj) {
        LOG_ERR << "[Error]Failed to add camera tracked obj, storage is full.";
        break;
      }

      dst_obj->tracked_obj_index_.Clear();

      dst_obj->raw_obj_idx_ = i;
      dst_obj->raw_obj_id_ = src_obj.id;
      switch (sensor_id) {
      case (SensorID::SENSOR_ID_MAIN_FORWARD_CAM):        //TODO: 将崔老师原始的传感器宏与sensors_fusion_macros.h中的顺序进行适配
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_MAIN_FORWARD_CAMERA;
        break;
      default:
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_UNKNOWN;
        break;
      }
      dst_obj->obj_type_ = src_obj.type;

      point_conv(0) = src_obj.x;
      point_conv(1) = src_obj.y;

      common::TransformVert_2D(mat_conv_composite, &point_conv);

      dst_obj->pos_.x_ = point_conv(0);
      dst_obj->pos_.y_ = point_conv(1);

#if ENABLE_POSITION_MAKE_UP      
      Float32_t time_diff = (*time_elapsed)*0.001;
      // dst_obj->pos_.x_ = dst_obj->pos_.x_ + (src_obj.v_x + 0.5 * src_obj.accel_x * time_diff) * time_diff;
      // dst_obj->pos_.y_ = dst_obj->pos_.y_ + (src_obj.v_y + 0.5 * src_obj.accel_y * time_diff) * time_diff;
      dst_obj->pos_.x_ = dst_obj->pos_.x_ + src_obj.v_x * time_diff;
      dst_obj->pos_.y_ = dst_obj->pos_.y_ + src_obj.v_y * time_diff;
      //LOG_INFO(2) << "make up:(" << dst_obj->pos_.x_ << "," << dst_obj->pos_.y_ << ")";
#endif

      dst_obj->pos_.range_ = common::com_sqrt(
            common::Square(dst_obj->pos_.x_) +
            common::Square(dst_obj->pos_.y_));
      dst_obj->pos_.angle_ =
          common::com_atan2(dst_obj->pos_.y_, dst_obj->pos_.x_);

      dst_obj->v_x_ = src_obj.v_x;
//      dst_obj->v_y_ = 0.0F;   //特殊处理
//      dst_obj->pos_.heading_ = 0.0F;
      dst_obj->v_y_ = src_obj.v_y;
      dst_obj->pos_.heading_ = src_obj.heading;

#if ENABLE_OUTPUT_ACCELERATION
      dst_obj->a_x_ = src_obj.accel_x;
      dst_obj->a_y_ = src_obj.accel_y;
#endif

      if (dst_obj->v_x_ < -5.0F/3.6F) {
        //dst_obj->pos_.heading_ = -COM_PI;
      }

      dst_obj->width_ = common::Max(src_obj.width, 0.5F); //TODO： 最小尺寸定为0.5米，根据传感器性能进行试验 //MIN_WIDTH_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->height_ = common::Max(src_obj.height, 0.5F);//MIN_HEIGHT_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->length_ = common::Max(src_obj.length, 0.5F);//MIN_LENGTH_OF_CAMERA_DATA_FOR_ADDING
      Float32_t half_width = 0.5F * dst_obj->width_;
      dst_obj->aabb_.min().set_x(dst_obj->pos_.x_);
      dst_obj->aabb_.min().set_y(dst_obj->pos_.y_ - half_width);
      dst_obj->aabb_.max().set_x(
            dst_obj->pos_.x_ + dst_obj->length_);
      dst_obj->aabb_.max().set_y(dst_obj->pos_.y_ + half_width);

      dst_obj->raw_data_ = src_obj;
    }

    obj_num = dst_obj_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjFromCamera& dst_obj = dst_obj_list->objects_[i];

      ObjIndex idx =
          obj_tracker_.AddNewObj(sensor_id, dst_obj);
      dst_obj.tracked_obj_index_ = idx;
    }
    if (obj_num < 1) {
      valid_list = false;
    }

    return (valid_list);
}

bool SensorsFusionMainstreamImpl::AddObjsListRadar(const Int32_t sensor_id, const ObjsFilterDataSource &data_source, Int64_t *time_elapsed)
{
    bool valid_list = false;

    const ad_msg::ObstacleRadarList* src_obj_list = Nullptr_t;
    ObjsFromRadarList* dst_obj_list = Nullptr_t;
    bool valid_sensor = true;
    bool is_radar_front = false;
    bool is_radar_around = false;

    switch (sensor_id) {
    case (SensorID::SENSOR_ID_MAIN_FORWARD_RADAR):
      is_radar_front = true;
      src_obj_list = data_source.objs_list_main_forward_radar_;
      dst_obj_list = &objs_from_main_forward_radar_;
      break;
    case (SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR):
      is_radar_around = true;
      src_obj_list = data_source.objs_list_left_backward_radar_;
      dst_obj_list = &objs_from_left_backward_radar_;
      break;
    case (SensorID::SENSOR_ID_LEFT_SIDE_RADAR):
      is_radar_around = true;
      src_obj_list = data_source.objs_list_left_side_radar_;
      dst_obj_list = &objs_from_left_side_radar_;
      break;
    case (SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR):
      is_radar_around = true;
      src_obj_list = data_source.objs_list_right_backward_radar_;
      dst_obj_list = &objs_from_right_backward_radar_;
      break;
    case (SensorID::SENSOR_ID_RIGHT_SIDE_RADAR):
      is_radar_around = true;
      src_obj_list = data_source.objs_list_right_side_radar_;
      dst_obj_list = &objs_from_right_side_radar_;
      break;
    default:
      valid_sensor = false;
      break;
    }
    if (!valid_sensor) {
      LOG_ERR << "[Error]Invalid sensor ID.";
      return (valid_list);
    }

    if (Nullptr_t != src_obj_list) {
      if (src_obj_list->msg_head.valid) {
        if (src_obj_list->obstacle_num > 0) {
          *time_elapsed = common::CalcElapsedClockMs(
                src_obj_list->msg_head.timestamp, data_source.timestamp_);
          if (*time_elapsed > 500 || *time_elapsed < 0) {  //UPPER_LIMIT_OF_TIME_ELAPSED_FOR_RADAR_LIST_INPUT
            LOG_ERR << "[Error]Getting obstacles from radar is timeout";
            dst_obj_list->Clear();
          } else {
            if (!dst_obj_list->msg_head_.valid) {
              // 之前没有保存过障碍物列表，
              // 所以当前传递进来的障碍物列表是有效的
              valid_list = true;
            } else {
              Uint32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
                    dst_obj_list->msg_head_.sequence,
                    src_obj_list->msg_head.sequence);
              if (sequence_diff > 0) {
                // 当前传递进来的障碍物列表是更新后的，所以是有效的
                valid_list = true;
              }
            }
          }
        }
      }
    }

    if (!valid_list) {
      return (valid_list);
    }

    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    // 将传递进来的障碍物列表进行时间及空间同步(对齐到时间流中)
    dst_obj_list->objects_.Clear();
    ad_msg::RelativePos pos_sensor;
    common::Matrix<Float32_t, 3, 3> mat_conv_sensor;
    mat_conv_sensor.SetIdentity();
    if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          src_obj_list->msg_head.timestamp, rel_pos_list_, &pos_sensor)) {
      LOG_ERR << "[Error]Failed to get pos of radar.";
      valid_list = false;
      return (valid_list);
    } else {
      common::Rotate_2D<Float32_t>(rotate_center,
                                pos_sensor.heading - current_rel_pos_.heading,
                                &mat_conv_sensor);
      common::Translate_2D(pos_sensor.x - current_rel_pos_.x,
                           pos_sensor.y - current_rel_pos_.y,
                           &mat_conv_sensor);
    }

    common::Matrix<Float32_t, 2, 1> point_conv;
    common::Matrix<Float32_t, 3, 3> mat_conv_composite = mat_conv_sensor;

    dst_obj_list->Clear();
    dst_obj_list->msg_head_ = src_obj_list->msg_head;
    dst_obj_list->sensor_id_ = sensor_id;

    Int32_t obj_num = src_obj_list->obstacle_num;
    for (Int32_t i = 0; i < obj_num; ++i) {
      const ad_msg::ObstacleRadar& src_obj = src_obj_list->obstacles[i];
      ObjFromRadar* dst_obj = dst_obj_list->objects_.Allocate();
      if (Nullptr_t == dst_obj) {
        LOG_ERR << "[Error]Failed to add radar tracked obj, storage is full.";
        break;
      }

      dst_obj->tracked_obj_index_.Clear();

      dst_obj->raw_obj_idx_ = i;
      dst_obj->raw_obj_id_ = src_obj.id;
      switch (sensor_id) {
      case (SensorID::SENSOR_ID_MAIN_FORWARD_RADAR):
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_MAIN_FORWARD_RADAR;
        break;
        //TODO: 添加环视毫米波雷达结果
      default:
        dst_obj->sensor_type_ = SensorType::SENSOR_TYPE_UNKNOWN;
        break;
      }

      /* 总是设置毫米波雷达识别的障碍物类型为未知 */
      if (src_obj.type == ad_msg::OBJ_TYPE_CURB)
      {
        dst_obj->obj_type_ = src_obj.type;
      }
      else
      {
        dst_obj->obj_type_ = ad_msg::OBJ_TYPE_UNKNOWN;
      }

      point_conv(0) = src_obj.x;
      point_conv(1) = src_obj.y;

      common::TransformVert_2D(mat_conv_composite, &point_conv);

      dst_obj->pos_.x_ = point_conv(0);
      dst_obj->pos_.y_ = point_conv(1);

#if ENABLE_POSITION_MAKE_UP      
      Float32_t time_diff = (*time_elapsed)*0.001;
      // dst_obj->pos_.x_ = dst_obj->pos_.x_ + (src_obj.v_x + 0.5 * src_obj.accel_x * time_diff) * time_diff;
      // dst_obj->pos_.y_ = dst_obj->pos_.y_ + (src_obj.v_y + 0.5 * src_obj.accel_y * time_diff) * time_diff;
      dst_obj->pos_.x_ = dst_obj->pos_.x_ + src_obj.v_x * time_diff;
      dst_obj->pos_.y_ = dst_obj->pos_.y_ + src_obj.v_y * time_diff;
      //LOG_INFO(2) << "make up:(" << dst_obj->pos_.x_ << "," << dst_obj->pos_.y_ << ")";
#endif

//      if (is_radar_around) {
//        if ((dst_obj->pos_.x_ > (vehicle_param_.dist_of_localization_to_front_ + 10.0F)) ||
//            (dst_obj->pos_.y_ < -(vehicle_param_.dist_of_localization_to_rear_ + 8.0F)) ||
//            ((dst_obj->pos_.x_ < (vehicle_param_.dist_of_localization_to_front_ + 0.1F)) &&
//             (dst_obj->pos_.x_ > -(vehicle_param_.dist_of_localization_to_rear_ + 0.5F)) &&
//             (common::com_abs(dst_obj->pos_.y_) < (0.5F*param_.vehicle_width_+0.1F))) ||
//            (common::com_abs(dst_obj->pos_.y_) > 8.0F)) {
//            dst_obj_list->objects_.PopBack();
//          continue;
//        }
//      }

      dst_obj->pos_.range_ = common::com_sqrt(
            common::Square(dst_obj->pos_.x_) +
            common::Square(dst_obj->pos_.y_));
      dst_obj->pos_.angle_ =
          common::com_atan2(dst_obj->pos_.y_, dst_obj->pos_.x_);


      Float32_t vx = src_obj.v_x;
      Float32_t vy = src_obj.v_y;
      dst_obj->v_x_ = vx;
      dst_obj->v_y_ = vy;

#if ENABLE_OUTPUT_ACCELERATION
      dst_obj->a_x_ = src_obj.accel_x;
      dst_obj->a_y_ = src_obj.accel_y;
#endif

//      //添加车尾雷达判断
//  #if 1
//      if (is_radar_rear) {
//        if ((common::com_abs(vx) < 5.0F/3.6F) &&
//            (dst_obj->pos.x < -(vehicle_params_.dist_of_localization_to_rear_+20.0F))) {
//          dst_obj_list->objects_.PopBack();
//          continue;
//        }
//      }
//  #endif

      dst_obj->pos_.heading_ = 0.0F;
      if (vx < -5.0F/3.6F) {
        //dst_obj->pos_.heading_ = -COM_PI;
      }

//      dst_obj->width_ = 0.5F;   //将激光雷达障碍物尺寸设置为固定长度
//      dst_obj->height_ = 0.5F;
//      dst_obj->length_ = 0.5F;

      if(is_radar_front){
        dst_obj->width_ = src_obj.width * FRONT_RADAR_USABLE_WIDTH_SCALE + FRONT_RADAR_USABLE_WIDTH_OFFSET;  //TODO：430的宽度信息可能能使用，待测试调整
      }
      else{
        dst_obj->width_ = src_obj.width;
      }
      

      dst_obj->height_ = 0.5F; //DEFAULT_HEIGHT_OF_CAMERA_DATA_FOR_ADDING
      dst_obj->length_ = common::Min(src_obj.length, 0.5F);// TODO：需430实车去测试下 //MIN_LENGTH_OF_RADAR_DATA_FOR_ADDING

      Float32_t half_width = 0.5F * dst_obj->width_;
      if (dst_obj->pos_.x_ > 0.0F) {
        dst_obj->aabb_.min().set_x(
              dst_obj->pos_.x_);
        dst_obj->aabb_.max().set_x(
              dst_obj->pos_.x_ + dst_obj->length_);
      } else {
        dst_obj->aabb_.min().set_x(
              dst_obj->pos_.x_ - dst_obj->length_);
        dst_obj->aabb_.max().set_x(
              dst_obj->pos_.x_);
      }
      dst_obj->aabb_.min().set_y(dst_obj->pos_.y_ - half_width);
      dst_obj->aabb_.max().set_y(dst_obj->pos_.y_ + half_width);

      dst_obj->raw_data_ = src_obj;
    }

    obj_num = dst_obj_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjFromRadar& dst_obj = dst_obj_list->objects_[i];

      ObjIndex idx =
          obj_tracker_.AddNewObj(sensor_id, dst_obj);
      dst_obj.tracked_obj_index_ = idx;
    }
    if (obj_num < 1) {
      valid_list = false;
    }

    return (valid_list);
}

bool SensorsFusionMainstreamImpl::AddObjsListLidar(const Int32_t sensor_id, const ObjsFilterDataSource &data_source, Int64_t *time_elapsed)
{
    bool valid_list = false;

    const ad_msg::ObstacleLidarList* src_obj_list = Nullptr_t;
    ObjsFromLidarList* dst_obj_list = Nullptr_t;
    bool valid_sensor = true;
    switch (sensor_id) {
    case (SENSOR_ID_MAIN_FORWARD_LIDAR):
      src_obj_list = data_source.objs_list_main_forward_lidar_;
      dst_obj_list = &objs_from_main_forward_lidar_;
      break;
    default:
      valid_sensor = false;
      break;
    }
    if (!valid_sensor) {
      LOG_ERR << "[Error]Invalid sensor ID.";
      return (valid_list);
    }

    if (Nullptr_t != src_obj_list) {
      if (src_obj_list->msg_head.valid) {
        if (src_obj_list->obstacle_num > 0) {
          *time_elapsed = common::CalcElapsedClockMs(
                src_obj_list->msg_head.timestamp, data_source.timestamp_);
          if (*time_elapsed > 500 || *time_elapsed < 0) {
            LOG_ERR << "[Error]Getting obstacles from lidar is timeout";
            dst_obj_list->Clear();
          } else {
            if (!dst_obj_list->msg_head_.valid) {
              // 之前没有保存过障碍物列表，
              // 所以当前传递进来的障碍物列表是有效的
              valid_list = true;
            } else {
              Uint32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
                    dst_obj_list->msg_head_.sequence,
                    src_obj_list->msg_head.sequence);
              if (sequence_diff > 0) {
                // 当前传递进来的障碍物列表是更新后的，所以是有效的
                valid_list = true;
              }
            }
          }
        }
      }
    }

    if (!valid_list) {
      return (valid_list);
    }

    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();

    // 将传递进来的障碍物列表进行时间及空间同步(对齐到时间流中)
    dst_obj_list->objects_.Clear();
    ad_msg::RelativePos pos_sensor;
    common::Matrix<Float32_t, 3, 3> mat_conv_sensor;
    mat_conv_sensor.SetIdentity();
    if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          src_obj_list->msg_head.timestamp, rel_pos_list_, &pos_sensor)) {
      LOG_ERR << "[Error]Failed to get pos of main forward lidar.";
      valid_list = false;
      return (valid_list);
    } else {
      common::Rotate_2D<Float32_t>(rotate_center,
                                pos_sensor.heading - current_rel_pos_.heading,
                                &mat_conv_sensor);
      common::Translate_2D(pos_sensor.x - current_rel_pos_.x,
                           pos_sensor.y - current_rel_pos_.y,
                           &mat_conv_sensor);
    }

    common::Matrix<Float32_t, 2, 1> point_conv;
    common::Matrix<Float32_t, 3, 3> mat_conv_composite = mat_conv_sensor;

    dst_obj_list->Clear();
    dst_obj_list->msg_head_ = src_obj_list->msg_head;
    dst_obj_list->sensor_id_ = sensor_id;

    Int32_t obj_num = src_obj_list->obstacle_num;
    for (Int32_t i = 0; i < obj_num; ++i) {
      const ad_msg::ObstacleLidar& src_obj = src_obj_list->obstacles[i];
      ObjFromLidar* dst_obj = dst_obj_list->objects_.Allocate();
      if (Nullptr_t == dst_obj) {
        LOG_ERR << "[Error]Failed to add tracked obj, storage is full.";
        break;
      }

      dst_obj->tracked_obj_index_.Clear();

      dst_obj->raw_obj_idx_ = i;
      dst_obj->raw_obj_id_ = src_obj.id;
      switch (src_obj_list->lidar_type) {
      case (ad_msg::ObstacleLidarList::LIDAR_TYPE_ROBOSENSE_M1):
        dst_obj->sensor_type_ = SENSOR_TYPE_MAIN_FORWARD_LIDAR;
        break;
      default:
        dst_obj->sensor_type_ = SENSOR_TYPE_UNKNOWN;
        break;
      }
      dst_obj->obj_type_ = src_obj.type;

      point_conv(0) = src_obj.x;
      point_conv(1) = src_obj.y;

      common::TransformVert_2D(mat_conv_composite, &point_conv);

      dst_obj->pos_.x_ = point_conv(0);
      dst_obj->pos_.y_ = point_conv(1);

#if ENABLE_POSITION_MAKE_UP      
      Float32_t time_diff = (*time_elapsed)*0.001;
      // dst_obj->pos_.x_ = dst_obj->pos_.x_ + (src_obj.v_x + 0.5 * src_obj.accel_x * time_diff) * time_diff;
      // dst_obj->pos_.y_ = dst_obj->pos_.y_ + (src_obj.v_y + 0.5 * src_obj.accel_y * time_diff) * time_diff;
      dst_obj->pos_.x_ = dst_obj->pos_.x_ + src_obj.v_x * time_diff;
      dst_obj->pos_.y_ = dst_obj->pos_.y_ + src_obj.v_y * time_diff;

      //LOG_INFO(2) << "make up:(" << dst_obj->pos_.x_ << "," << dst_obj->pos_.y_ << ")";
#endif

      dst_obj->pos_.range_ = common::com_sqrt(
            common::Square(dst_obj->pos_.x_) +
            common::Square(dst_obj->pos_.y_));
      dst_obj->pos_.angle_ =
          common::com_atan2(dst_obj->pos_.y_, dst_obj->pos_.x_);

      Float32_t vx = src_obj.v_x;
      Float32_t vy = src_obj.v_y;
      dst_obj->v_x_ = vx;
      dst_obj->v_y_ = vy;

      dst_obj->pos_.heading_ = src_obj.obb.heading;
      dst_obj->width_ = 2*src_obj.obb.half_width;
      dst_obj->length_ = 2*src_obj.obb.half_length;
      Float32_t half_width = 0.5F * dst_obj->width_;
      if (dst_obj->pos_.x_ > 0.0F) {
        dst_obj->aabb_.min().set_x(
              dst_obj->pos_.x_);
        dst_obj->aabb_.max().set_x(
              dst_obj->pos_.x_ + dst_obj->length_);
      } else {
        dst_obj->aabb_.min().set_x(
              dst_obj->pos_.x_ - dst_obj->length_);
        dst_obj->aabb_.max().set_x(
              dst_obj->pos_.x_);
      }
      dst_obj->aabb_.min().set_y(dst_obj->pos_.y_ - half_width);
      dst_obj->aabb_.max().set_y(dst_obj->pos_.y_ + half_width);

      dst_obj->raw_data_ = src_obj;
    }

    /// 跟踪融合LIDAR OBJ
  #if 0
    obj_num = dst_obj_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      ObjFromLidar& dst_obj = dst_obj_list->objects_[i];

      ObjIndex idx =
          obj_tracker_.AddNewObj(sensor_id, dst_obj);
      dst_obj.tracked_obj_index_ = idx;
    }
    if (obj_num < 1) {
      valid_list = false;
    }
  #endif

    return (valid_list);
}

void SensorsFusionMainstreamImpl::FilterByRadar(const Int32_t sensor_id, const Float32_t time_elapsed_ms)
{
    MatchingParams match_param;
    ObjsFromRadarList* src_list = Nullptr_t;
    bool is_radar_front = false;
    bool is_radar_rear = false;
    bool is_radar_around = false;
    bool valid_sensor = true;
    int postion_type = SENSOR_POSITION_NONE;

    switch (sensor_id) {
    case (SENSOR_ID_MAIN_FORWARD_RADAR):
      src_list = &objs_from_main_forward_radar_;
      is_radar_front = true;
      postion_type = SENSOR_POSITION_MAIN_FRONT;
      break;
    case (SENSOR_ID_LEFT_BACKWARD_RADAR):
      src_list = &objs_from_left_backward_radar_;
      is_radar_around = true;
      postion_type = SENSOR_POSITION_REAR_LEFT;
      break;
    case (SENSOR_ID_LEFT_SIDE_RADAR):
      src_list = &objs_from_left_side_radar_;
      is_radar_around = true;
      postion_type = SENSOR_POSITION_FRONT_LEFT;
      break;
    case (SENSOR_ID_RIGHT_BACKWARD_RADAR):
      src_list = &objs_from_right_backward_radar_;
      is_radar_around = true;
      postion_type = SENSOR_POSITION_REAR_RIGHT;
      break;
    case (SENSOR_ID_RIGHT_SIDE_RADAR):
      src_list = &objs_from_right_side_radar_;
      is_radar_around = true;
      postion_type = SENSOR_POSITION_FRONT_RIGHT;
      break;

    default:
      valid_sensor = false;
      break;
    }
    if (!valid_sensor) {
      LOG_ERR << "[Error]Invalid sensor ID.";
      return;
    }

    if (is_radar_front || is_radar_rear) {
      /**
       * @brief 前雷达匹配参数
       */
      // [粗选]只与ESR的传感器匹配
      match_param.masks_[SENSOR_ID_MAIN_FORWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_LEFT_SIDE_CAM] = true;
      match_param.masks_[SENSOR_ID_RIGHT_SIDE_CAM] = true;
      // [粗选]不判断已经匹配上的次数
      match_param.min_matched_times_ = -1;//MIN_MATCHED_TRACKING_TIMES_FOR_FRONT_RADAR_MATCH_PARAM
      // [粗选]不判断最近未匹配上的次数
      match_param.max_dismatch_times_ = -1;//MAX_DISMATCHED_TRACKING_TIMES_FOR_FRONT_RADAR_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 5.0F/3.6F; //MAX_VX_DIFF_FOR_MATCH_FRONT_RADAR_PARAM
      match_param.max_vy_diff_ = 5.0F/3.6F;//30.0F/3.6F;//MAX_VY_DIFF_FOR_MATCH_FRONT_RADAR_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true; //IS_VALID_CONDITION_FOR_MATCH_FRONT_RADAR_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 10.0F/3.6F;//MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_FRONT_RADAR_PARAM
      match_param.condition_.max_vy_diff_ = 10.0F/3.6F;//MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_FRONT_RADAR_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = true;//USING_EST_POS_FOR_MATCH_FRONT_RADAR_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = true;//ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_FRONT_RADAR_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = true;//ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_FRONT_RADAR_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = false;//MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_FRONT_RADAR_PARAM
    }

    if ((postion_type&SENSOR_POSITION_LEFT) != 0) {
      /**
       * @brief 左侧雷达匹配参数
       */
      // [粗选]只与Radar的传感器匹配
      match_param.masks_[SENSOR_ID_LEFT_SIDE_CAM] = true;
      match_param.masks_[SENSOR_ID_LEFT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_LEFT_SIDE_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_SIDE_RADAR] = true;//此处可能是个错误
      // [粗选]不判断已经匹配上的次数
      match_param.min_matched_times_ = -1; //MIN_MATCHED_TRACKING_TIMES_FOR_AROUND_RADAR_MATCH_PARAM
      // [粗选]不判断最近未匹配上的次数
      match_param.max_dismatch_times_ = -1; //MAX_DISMATCHED_TRACKING_TIMES_FOR_AROUND_RADAR_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 30.0F/3.6F;//OLD:5.0F/3.6F; //MAX_VX_DIFF_FOR_MATCH_AROUND_RADAR_PARAM
      match_param.max_vy_diff_ = 20.0F/3.6F;//OLD:30.0F/3.6F;//MAX_VY_DIFF_FOR_MATCH_AROUND_RADAR_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true; //IS_VALID_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 20.0F/3.6F;//OLD:10.0F/3.6F //MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM
      match_param.condition_.max_vy_diff_ = 15.0F/3.6F;//OLD:10.0F/3.6F //MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = true; //USING_EST_POS_FOR_MATCH_AROUND_RADAR_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = true;//ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_AROUND_RADAR_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = true;//ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_AROUND_RADAR_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true;//OLD: false//MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_AROUND_RADAR_PARAM

      match_param.match_with_velocity_ = false;//表示是在匹配过程中是否用到速度值，侧雷达这里暂时不用到速度。

    }

    if ((postion_type&SENSOR_POSITION_RIGHT) != 0) {
      /**
       * @brief 右侧雷达匹配参数
       */
      // [粗选]只与Radar的传感器匹配
      match_param.masks_[SENSOR_ID_RIGHT_SIDE_CAM] = true;
      match_param.masks_[SENSOR_ID_LEFT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_LEFT_SIDE_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_SIDE_RADAR] = true;//此处可能是个错误
      // [粗选]不判断已经匹配上的次数
      match_param.min_matched_times_ = -1; //MIN_MATCHED_TRACKING_TIMES_FOR_AROUND_RADAR_MATCH_PARAM
      // [粗选]不判断最近未匹配上的次数
      match_param.max_dismatch_times_ = -1; //MAX_DISMATCHED_TRACKING_TIMES_FOR_AROUND_RADAR_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 30.0F/3.6F;//OLD:5.0F/3.6F; //MAX_VX_DIFF_FOR_MATCH_AROUND_RADAR_PARAM
      match_param.max_vy_diff_ = 20.0F/3.6F;//OLD:30.0F/3.6F;//MAX_VY_DIFF_FOR_MATCH_AROUND_RADAR_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true; //IS_VALID_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 20.0F/3.6F;//OLD:10.0F/3.6F //MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM
      match_param.condition_.max_vy_diff_ = 15.0F/3.6F;//OLD:10.0F/3.6F //MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = true; //USING_EST_POS_FOR_MATCH_AROUND_RADAR_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = true;//ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_AROUND_RADAR_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = true;//ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_AROUND_RADAR_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true;//OLD: false//MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_AROUND_RADAR_PARAM
    
      match_param.match_with_velocity_ = false;//表示是在匹配过程中是否用到速度值，侧雷达这里暂时不用到速度。
    }


  #if ENABLE_OBJ_FILTER_IMPL_1_TRACE
    std::cout << ">>@@@  radar 查找可匹配的obj:" << std::endl;
  #endif
    // 对ESR障碍物列表中的每个障碍物都在障碍物跟踪列表中查找与他相关的障碍物信息
    // （位置临近，属性匹配）
    Int32_t obj_num = src_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      /**
       * @brief 以下代码针对不同的障碍物各自计算匹配参数，前面for循环之前match_param匹配参数属于所有同类型传感器障碍物的公共部分
       * 
       */
      ObjFromRadar& src_obj = src_list->objects_[i];

      Float32_t obj_v_x = src_obj.v_x_;
      Float32_t obj_v_y = src_obj.v_y_;
      Float32_t obj_v = common::com_sqrt(common::Square(obj_v_x) +
                                      common::Square(obj_v_y));

      // 临近障碍物的检索半径
      if (is_radar_front || is_radar_rear) {
        /**
         * @brief 前雷达匹配参数
         */
        if (obj_v > 15.0F/3.6F) {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              2.0F + src_obj.pos_.range_*0.04F +
              obj_v * time_elapsed_ms +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
              src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              2.0F + src_obj.pos_.range_*0.02F +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
              src_obj.pos_.range_;
        }

        if (obj_v > 15.0F/3.6F) {
          match_param.condition_.max_x_diff_ =
              2.0F + src_obj.pos_.range_*0.03F +
              obj_v * time_elapsed_ms;
          match_param.condition_.max_y_diff_ =
              1.0F + src_obj.pos_.range_*0.01F +
              obj_v * time_elapsed_ms;
        } else {
          match_param.condition_.max_x_diff_ =
              0.5F + src_obj.pos_.range_*0.01F;
          match_param.condition_.max_y_diff_ =
              0.5F + src_obj.pos_.range_*0.005F;
        }
      }

      if ((postion_type&SENSOR_POSITION_LEFT) != 0) {
        /**
         * @brief 左侧雷达匹配参数
         */
        if (obj_v > 15.0F/3.6F && match_param.match_with_velocity_) {//LIMIT_OF_OBJ_VELOCITY_FOR_RADAR_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              6.0F + src_obj.pos_.range_*0.05F +   //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_FOR_RADAR
              obj_v * time_elapsed_ms +
              0.3F*current_rel_pos_.yaw_rate * time_elapsed_ms *
              src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              6.0F + src_obj.pos_.range_*0.05F +
              0.3F*current_rel_pos_.yaw_rate * time_elapsed_ms *
              src_obj.pos_.range_;
        }
        if (obj_v > 15.0F/3.6F && match_param.match_with_velocity_) {
          match_param.condition_.max_x_diff_ =
              6.0F + src_obj.pos_.range_*0.04F +
              obj_v * time_elapsed_ms;
          match_param.condition_.max_y_diff_ =
              2.0F + src_obj.pos_.range_*0.01F +
              obj_v * time_elapsed_ms;
        } else {
          match_param.condition_.max_x_diff_ =
              6.0F + src_obj.pos_.range_*0.04F;
          match_param.condition_.max_y_diff_ =
              1.0F + src_obj.pos_.range_*0.009F;
        }
      }

      if ((postion_type&SENSOR_POSITION_RIGHT) != 0) {
        /**
         * @brief 右侧雷达匹配参数
         */
        if (obj_v > 15.0F/3.6F && match_param.match_with_velocity_) {//LIMIT_OF_OBJ_VELOCITY_FOR_RADAR_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              6.0F + src_obj.pos_.range_*0.05F +   //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_FOR_RADAR
              obj_v * time_elapsed_ms +
              0.3F*current_rel_pos_.yaw_rate * time_elapsed_ms *
              src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              6.0F + src_obj.pos_.range_*0.05F +
              0.3F*current_rel_pos_.yaw_rate * time_elapsed_ms *
              src_obj.pos_.range_;
        }

        if (obj_v > 15.0F/3.6F && match_param.match_with_velocity_) {
          match_param.condition_.max_x_diff_ =
              6.0F + src_obj.pos_.range_*0.04F +
              obj_v * time_elapsed_ms;
          match_param.condition_.max_y_diff_ =
              1.8F + src_obj.pos_.range_*0.01F +
              obj_v * time_elapsed_ms;
        } else {
          match_param.condition_.max_x_diff_ =
              6.0F + src_obj.pos_.range_*0.04F;
          match_param.condition_.max_y_diff_ =
              1.8F + src_obj.pos_.range_*0.009F;
        }
      }

      obj_tracker_.MatchToTrackedObj(
            match_param, src_obj.tracked_obj_index_, src_obj);
    }

    // 找到最佳匹配
    obj_tracker_.FindBestMatchedObj(sensor_id, match_param);
}

void SensorsFusionMainstreamImpl::FilterByCamera(const Int32_t sensor_id, const Float32_t time_elapsed_ms)
{
    MatchingParams match_param;
    ObjsFromCameraList* src_list = Nullptr_t;
    bool valid_sensor = true;

    bool is_main_camera = false;

    bool is_visual_control=false;

    int postion_type = 0;

    switch (sensor_id) {
    case (SENSOR_ID_MAIN_FORWARD_CAM):
      src_list = &objs_from_main_forward_cam_;
      is_main_camera=true;
      postion_type = SENSOR_POSITION_MAIN_FRONT;
      break;
      case (SENSOR_ID_CENTER_FORWARD_CAM):
      src_list = &objs_from_center_forward_cam_;
      is_visual_control=true;
      postion_type = SENSOR_POSITION_FRONT;
      break;
      case (SENSOR_ID_LEFT_BACKWARD_CAM):
      src_list = &objs_from_left_backward_cam_;
      is_visual_control=true;
      postion_type = SENSOR_POSITION_REAR_LEFT;
      break;
      case (SENSOR_ID_LEFT_SIDE_CAM):
      src_list = &objs_from_left_side_cam_;
      is_visual_control=true;
      postion_type = SENSOR_POSITION_FRONT_LEFT;
      break;
      case (SENSOR_ID_RIGHT_BACKWARD_CAM):
      src_list = &objs_from_right_backward_cam_;
      is_visual_control=true;
      postion_type = SENSOR_POSITION_REAR_RIGHT;
      break;
      case (SENSOR_ID_RIGHT_SIDE_CAM):
      src_list = &objs_from_right_side_cam_;
      is_visual_control=true;
      postion_type = SENSOR_POSITION_FRONT_RIGHT;
      break;
    default:
      valid_sensor = false;
      break;
    }

    if (!valid_sensor) {
      LOG_ERR << "Invalid sensor ID.";
      return;
    } 

    if(is_main_camera){
      /**
       * @brief 前视一体机匹配参数
       */
      // [粗选]最低已经跟踪上的次数
      match_param.min_matched_times_ = 3;//-1 //MIN_MATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM
      // [粗选]最近最多未匹配上的次数
      match_param.max_dismatch_times_ = 5;//-1 //MAX_DISMATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 10.0F/3.6F; //MAX_VX_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM
      match_param.max_vy_diff_ = 30.0F/3.6F; //MAX_VY_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true;  //IS_VALID_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 30.0F/3.6F; //MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      match_param.condition_.max_vy_diff_ = 10.0F/3.6F; //MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = false; //USING_EST_POS_FOR_MATCH_MAIN_CAMERA_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = false; //ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_MAIN_CAMERA_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = false; //ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_MAIN_CAMERA_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true ;//true //MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_MAIN_CAMERA_PARAM
      // [粗选]与环视相机，毫米波的传感器匹配
      //match_param.masks_[SENSOR_ID_MAIN_FORWARD_CAM] = true;
      match_param.masks_[SENSOR_ID_MAIN_FORWARD_RADAR] = true;
      //若match_again_by_id_if_unmatched_设置为true，以下参数才有效，并用作判断id相同的相机数据与其他传感器数据物理上的匹配阈值。
      match_param.max_x_diff_for_matched_again_by_id = 20.0F;
      match_param.max_y_diff_for_matched_again_by_id = 8.0F;
      match_param.max_vx_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vx不要小于前面的max_vx_XXX值。
      match_param.max_vy_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vy不要小于前面的max_vy_XXX值。

      //若match_again_by_id_if_unmatched_设置为true，以下参数才有效，并用作判断id相同的相机数据与其他传感器数据物理上的匹配阈值。
      match_param.max_x_diff_for_matched_again_by_id = 20.0F;
      match_param.max_y_diff_for_matched_again_by_id = 8.0F;
      match_param.max_vx_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vx不要小于前面的max_vx_XXX值。
      match_param.max_vy_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vy不要小于前面的max_vy_XXX值。


    }else if((postion_type & SENSOR_POSITION_LEFT) != 0){
      /**
       * @brief 环视左边匹配参数
       */
      // [粗选]最低已经跟踪上的次数
      match_param.min_matched_times_ = -1;//MIN_MATCHED_TRACKING_TIMES_FOR_AROUND_CAMERA_MATCH_PARAM
      // [粗选]最近最多未匹配上的次数
      match_param.max_dismatch_times_ = -1;//MAX_DISMATCHED_TRACKING_TIMES_FOR_AROUND_CAMERA_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 40.0F/3.6F;//MAX_VX_DIFF_FOR_MATCH_AROUND_CAMERA_PARAM
      match_param.max_vy_diff_ = 30.0F/3.6F;//MAX_VY_DIFF_FOR_MATCH_AROUND_CAMERA_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true;//IS_VALID_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 38.0F/3.6F;//MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM
      match_param.condition_.max_vy_diff_ = 10.0F/3.6F;//MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = false;//USING_EST_POS_FOR_MATCH_AROUND_CAMERA_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = true;//ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_AROUND_CAMERA_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = true;//ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_AROUND_CAMERA_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true;//MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_AROUND_CAMERA_PARAM
      // [粗选]与环视相机，毫米波的传感器匹配
      // match_param.masks_[SENSOR_ID_LEFT_BACKWARD_CAM] = true;
      match_param.masks_[SENSOR_ID_MAIN_FORWARD_RADAR] = true;    
      match_param.masks_[SENSOR_ID_RIGHT_BACKWARD_CAM] = true;
      match_param.masks_[SENSOR_ID_LEFT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_LEFT_SIDE_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_SIDE_RADAR] = true;//此处可能是个错误


      match_param.match_with_velocity_ = false; //表示是在匹配过程中是否用到速度值，环视这里暂时不用到速度。


    }
    else if((postion_type & SENSOR_POSITION_RIGHT) != 0){
      /**
       * @brief 环视右边匹配参数
       */
      // [粗选]最低已经跟踪上的次数
      match_param.min_matched_times_ = -1;//MIN_MATCHED_TRACKING_TIMES_FOR_AROUND_CAMERA_MATCH_PARAM
      // [粗选]最近最多未匹配上的次数
      match_param.max_dismatch_times_ = -1;//MAX_DISMATCHED_TRACKING_TIMES_FOR_AROUND_CAMERA_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 40.0F/3.6F;//MAX_VX_DIFF_FOR_MATCH_AROUND_CAMERA_PARAM
      match_param.max_vy_diff_ = 30.0F/3.6F;//MAX_VY_DIFF_FOR_MATCH_AROUND_CAMERA_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true;//IS_VALID_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 38.0F/3.6F;//MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM
      match_param.condition_.max_vy_diff_ = 10.0F/3.6F;//MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = false;//USING_EST_POS_FOR_MATCH_AROUND_CAMERA_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = true;//ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_AROUND_CAMERA_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = true;//ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_AROUND_CAMERA_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true;//MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_AROUND_CAMERA_PARAM
      // [粗选]与环视相机，毫米波的传感器匹配
      match_param.masks_[SENSOR_ID_MAIN_FORWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_LEFT_BACKWARD_CAM] = true;    
      // match_param.masks_[SENSOR_ID_RIGHT_BACKWARD_CAM] = true;
      match_param.masks_[SENSOR_ID_LEFT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_LEFT_SIDE_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_BACKWARD_RADAR] = true;
      match_param.masks_[SENSOR_ID_RIGHT_SIDE_RADAR] = true;//此处可能是个错误

      match_param.match_with_velocity_ = false; //表示是在匹配过程中是否用到速度值，环视这里暂时不用到速度。
    }


  #if ENABLE_OBJ_FILTER_IMPL_1_TRACE
    std::cout << ">>@@@ camera 查找可匹配的obj:" << std::endl;
  #endif
    // 对camera障碍物列表中的每个障碍物都在障碍物跟踪列表中查找与他相关的障碍物信息
    // （位置临近，属性匹配）
    Int32_t obj_num = src_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      /**
       * @brief 以下代码针对不同的障碍物各自计算匹配参数，前面for循环之前match_param匹配参数属于所有同类型传感器障碍物的公共部分
       */
      ObjFromCamera& src_obj = src_list->objects_[i];
      Float32_t obj_v_x = src_obj.v_x_;
      Float32_t obj_v_y = src_obj.v_y_;
      Float32_t obj_v = common::com_sqrt(common::Square(obj_v_x) +
                                      common::Square(obj_v_y));

      if(is_main_camera){
        /**
         * @brief 前视一体机匹配参数
         */
        // 临近障碍物的检索半径
        if (obj_v > 15.0F/3.6F) { // LIMIT_OF_OBJ_VELOCITY_FOR_CAMERA_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              5.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA
              obj_v * time_elapsed_ms +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              5.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        }
        match_param.condition_.max_x_diff_ =
            5.0F + src_obj.pos_.range_*0.1F + obj_v*0.2F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
        match_param.condition_.max_y_diff_ =
            0.5F + src_obj.pos_.range_*0.05F + obj_v*0.1F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA

      }
      else if((postion_type & SENSOR_POSITION_LEFT) != 0)
      {
        /**
         * @brief 环视左边的匹配参数
         */
        // 临近障碍物的检索半径
        if (obj_v > 15.0F/3.6F) { // LIMIT_OF_OBJ_VELOCITY_FOR_CAMERA_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              8.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA
              obj_v * time_elapsed_ms +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              8.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        }
        if(true == match_param.match_with_velocity_)
        {
          match_param.condition_.max_x_diff_ =
              8.0F + src_obj.pos_.range_*0.1F + obj_v*0.2F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
          match_param.condition_.max_y_diff_ =
              3.0F + src_obj.pos_.range_*0.05F + obj_v*0.1F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA
        }
        else
        {//去掉速度影响，让速度值不参与匹配过程中其他各种值的计算
          match_param.condition_.max_x_diff_ =
              8.0F + src_obj.pos_.range_*0.1F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
          match_param.condition_.max_y_diff_ =
              3.0F + src_obj.pos_.range_*0.05F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA
        }
      }
      else //if((postion_type & SENSOR_POSITION_RIGHT) != 0)
      {
        
        /**
         * @brief 环视右边的匹配参数
         */
        // 临近障碍物的检索半径
        if (obj_v > 15.0F/3.6F) { // LIMIT_OF_OBJ_VELOCITY_FOR_CAMERA_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              8.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA
              obj_v * time_elapsed_ms +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              8.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        }

        if(true == match_param.match_with_velocity_)
        {
          match_param.condition_.max_x_diff_ =
              8.0F + src_obj.pos_.range_*0.1F + obj_v*0.2F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
          match_param.condition_.max_y_diff_ =
              3.0F + src_obj.pos_.range_*0.05F + obj_v*0.1F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA
        }
        else
        {//去掉速度影响，让速度值不参与匹配过程中其他各种值的计算
          match_param.condition_.max_x_diff_ =
              8.0F + src_obj.pos_.range_*0.1F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
          match_param.condition_.max_y_diff_ =
              3.0F + src_obj.pos_.range_*0.05F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA
        }
        
      }

      obj_tracker_.MatchToTrackedObj(
            match_param, src_obj.tracked_obj_index_, src_obj);

    }

    // 找到最佳匹配
    obj_tracker_.FindBestMatchedObj(sensor_id, match_param);
}

void SensorsFusionMainstreamImpl::FilterByMainCamera_80(const Int32_t sensor_id, const Float32_t time_elapsed_ms)
{
    MatchingParams match_param;
    ObjsFromCameraList* src_list = Nullptr_t;
    bool valid_sensor = true;

    bool is_main_camera = false;

    bool is_visual_control=false;

    int postion_type = 0;

    switch (sensor_id) {
    case (SENSOR_ID_MAIN_FORWARD_CAM):
      src_list = &objs_from_main_forward_cam_80_;
      is_main_camera=true;
      postion_type = SENSOR_POSITION_MAIN_FRONT;
      break;
    default:
      valid_sensor = false;
      break;
    }

    if (!valid_sensor) {
      LOG_ERR << "Invalid sensor ID.";
      return;
    } 

    if(is_main_camera){
      /**
       * @brief 前视一体机匹配参数
       */
      // [粗选]最低已经跟踪上的次数
      match_param.min_matched_times_ = 3;//-1 //MIN_MATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM
      // [粗选]最近最多未匹配上的次数
      match_param.max_dismatch_times_ = 5;//-1 //MAX_DISMATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 10.0F/3.6F; //MAX_VX_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM
      match_param.max_vy_diff_ = 30.0F/3.6F; //MAX_VY_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true;  //IS_VALID_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 30.0F/3.6F; //MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      match_param.condition_.max_vy_diff_ = 10.0F/3.6F; //MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = false; //USING_EST_POS_FOR_MATCH_MAIN_CAMERA_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = false; //ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_MAIN_CAMERA_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = false; //ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_MAIN_CAMERA_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true ;//true //MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_MAIN_CAMERA_PARAM
      // [粗选]与环视相机，毫米波的传感器匹配
      //match_param.masks_[SENSOR_ID_MAIN_FORWARD_CAM] = true;
      match_param.masks_[SENSOR_ID_MAIN_FORWARD_RADAR] = true;
      //若match_again_by_id_if_unmatched_设置为true，以下参数才有效，并用作判断id相同的相机数据与其他传感器数据物理上的匹配阈值。
      match_param.max_x_diff_for_matched_again_by_id = 20.0F;
      match_param.max_y_diff_for_matched_again_by_id = 8.0F;
      match_param.max_vx_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vx不要小于前面的max_vx_XXX值。
      match_param.max_vy_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vy不要小于前面的max_vy_XXX值。

    }

  #if ENABLE_OBJ_FILTER_IMPL_1_TRACE
    std::cout << ">>@@@ camera 查找可匹配的obj:" << std::endl;
  #endif
    // 对camera障碍物列表中的每个障碍物都在障碍物跟踪列表中查找与他相关的障碍物信息
    // （位置临近，属性匹配）
    Int32_t obj_num = src_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      /**
       * @brief 以下代码针对不同的障碍物各自计算匹配参数，前面for循环之前match_param匹配参数属于所有同类型传感器障碍物的公共部分
       */
      ObjFromCamera& src_obj = src_list->objects_[i];
      Float32_t obj_v_x = src_obj.v_x_;
      Float32_t obj_v_y = src_obj.v_y_;
      Float32_t obj_v = common::com_sqrt(common::Square(obj_v_x) +
                                      common::Square(obj_v_y));

      if(is_main_camera){
        /**
         * @brief 前视一体机匹配参数
         */
        // 临近障碍物的检索半径
        if (obj_v > 15.0F/3.6F) { // LIMIT_OF_OBJ_VELOCITY_FOR_CAMERA_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              5.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA
              obj_v * time_elapsed_ms +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              5.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        }
        match_param.condition_.max_x_diff_ =
            5.0F + src_obj.pos_.range_*0.1F + obj_v*0.2F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
        match_param.condition_.max_y_diff_ =
            0.5F + src_obj.pos_.range_*0.05F + obj_v*0.1F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA

      }

      obj_tracker_.MatchToTrackedObj(
            match_param, src_obj.tracked_obj_index_, src_obj);

    }

    // 找到最佳匹配
    obj_tracker_.FindBestMatchedObj(sensor_id, match_param);
}

void SensorsFusionMainstreamImpl::FilterByMainCamera_150(const Int32_t sensor_id, const Float32_t time_elapsed_ms)
{
    MatchingParams match_param;
    ObjsFromCameraList* src_list = Nullptr_t;
    bool valid_sensor = true;

    bool is_main_camera = false;

    bool is_visual_control=false;

    int postion_type = 0;

    switch (sensor_id) {
    case (SENSOR_ID_MAIN_FORWARD_CAM):
      src_list = &objs_from_main_forward_cam_150_;
      is_main_camera=true;
      postion_type = SENSOR_POSITION_MAIN_FRONT;
      break;
    default:
      valid_sensor = false;
      break;
    }

    if (!valid_sensor) {
      LOG_ERR << "Invalid sensor ID.";
      return;
    } 

    if(is_main_camera){
      /**
       * @brief 前视一体机匹配参数
       */
      // [粗选]最低已经跟踪上的次数
      match_param.min_matched_times_ = 3;//-1 //MIN_MATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM
      // [粗选]最近最多未匹配上的次数
      match_param.max_dismatch_times_ = 5;//-1 //MAX_DISMATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM
      // [粗选]匹配的obj最大允许的速度差
      match_param.max_vx_diff_ = 10.0F/3.6F; //MAX_VX_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM
      match_param.max_vy_diff_ = 30.0F/3.6F; //MAX_VY_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM
      // [精选]有精选条件
      match_param.condition_.valid_ = true;  //IS_VALID_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      // [精选]匹配的obj最大允许的速度差
      match_param.condition_.max_vx_diff_ = 30.0F/3.6F; //MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      match_param.condition_.max_vy_diff_ = 10.0F/3.6F; //MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM
      // 运行根据当前状态估计位置
      match_param.using_est_pos_ = false; //USING_EST_POS_FOR_MATCH_MAIN_CAMERA_PARAM
      // 允许多个obj id进行匹配
      match_param.enable_add_mul_obj_id_ = false; //ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_MAIN_CAMERA_PARAM
      // 允许次优匹配
      match_param.enable_sub_optimal_match_ = false; //ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_MAIN_CAMERA_PARAM
      // 若根据位置未匹配上，是否根据ID重新匹配
      match_param.match_again_by_id_if_unmatched_ = true ;//true //MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_MAIN_CAMERA_PARAM
      // [粗选]与环视相机，毫米波的传感器匹配
      //match_param.masks_[SENSOR_ID_MAIN_FORWARD_CAM] = true;
      match_param.masks_[SENSOR_ID_MAIN_FORWARD_RADAR] = true;
      //若match_again_by_id_if_unmatched_设置为true，以下参数才有效，并用作判断id相同的相机数据与其他传感器数据物理上的匹配阈值。
      match_param.max_x_diff_for_matched_again_by_id = 20.0F;
      match_param.max_y_diff_for_matched_again_by_id = 8.0F;
      match_param.max_vx_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vx不要小于前面的max_vx_XXX值。
      match_param.max_vy_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vy不要小于前面的max_vy_XXX值。

      //若match_again_by_id_if_unmatched_设置为true，以下参数才有效，并用作判断id相同的相机数据与其他传感器数据物理上的匹配阈值。
      match_param.max_x_diff_for_matched_again_by_id = 20.0F;
      match_param.max_y_diff_for_matched_again_by_id = 8.0F;
      match_param.max_vx_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vx不要小于前面的max_vx_XXX值。
      match_param.max_vy_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vy不要小于前面的max_vy_XXX值。


    }

  #if ENABLE_OBJ_FILTER_IMPL_1_TRACE
    std::cout << ">>@@@ camera 查找可匹配的obj:" << std::endl;
  #endif
    // 对camera障碍物列表中的每个障碍物都在障碍物跟踪列表中查找与他相关的障碍物信息
    // （位置临近，属性匹配）
    Int32_t obj_num = src_list->objects_.Size();
    for (Int32_t i = 0; i < obj_num; ++i) {
      /**
       * @brief 以下代码针对不同的障碍物各自计算匹配参数，前面for循环之前match_param匹配参数属于所有同类型传感器障碍物的公共部分
       */
      ObjFromCamera& src_obj = src_list->objects_[i];
      Float32_t obj_v_x = src_obj.v_x_;
      Float32_t obj_v_y = src_obj.v_y_;
      Float32_t obj_v = common::com_sqrt(common::Square(obj_v_x) +
                                      common::Square(obj_v_y));

      if(is_main_camera){
        /**
         * @brief 前视一体机匹配参数
         */
        // 临近障碍物的检索半径
        if (obj_v > 15.0F/3.6F) { // LIMIT_OF_OBJ_VELOCITY_FOR_CAMERA_MATCH_SEARCH_RADIUS
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              5.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA
              obj_v * time_elapsed_ms +
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        } else {
          match_param.searching_radius_ =
              common::Max(0.5F*src_obj.length_,
                          0.5F*src_obj.width_) +
              5.0F + src_obj.pos_.range_*0.2F + //COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA
              0.2F*current_rel_pos_.yaw_rate * time_elapsed_ms *
                src_obj.pos_.range_;
        }
        match_param.condition_.max_x_diff_ =
            5.0F + src_obj.pos_.range_*0.1F + obj_v*0.2F;//COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA
        match_param.condition_.max_y_diff_ =
            0.5F + src_obj.pos_.range_*0.05F + obj_v*0.1F;//COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA

      }

      obj_tracker_.MatchToTrackedObj(
            match_param, src_obj.tracked_obj_index_, src_obj);

    }

    // 找到最佳匹配
    obj_tracker_.FindBestMatchedObj(sensor_id, match_param);
}

void SensorsFusionMainstreamImpl::FilterByLidar(const Int32_t sensor_id, const Float32_t time_elapsed_ms)
{
    ///TODO:
}

} // goyu
} // sensors fusion
} // perception
} // phoenix
