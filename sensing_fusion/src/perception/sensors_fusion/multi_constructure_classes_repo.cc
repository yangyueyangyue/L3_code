#include "vehicle_model_wrapper.h"
#include "utils/linear_interpolation.h"
#include "multi_constructure_classes_repo.h"


namespace phoenix{
namespace perception{
namespace sensorsfusion{

//bool FindRelPosFromListByTimestamp(const Int64_t timestamp, const ad_msg::RelativePosList &pos_list, ad_msg::RelativePos *pos)
//{
//    veh_model::VehicleModelWrapper vehicle_model;
//    Int32_t pos_list_size = pos_list.relative_pos_num;

//    if(pos_list_size < 1){
//        return false;
//    }

//    Int64_t relative_time = common::CalcElapsedClockMs(
//                pos_list.msg_head.timestamp, timestamp
//                );

//    bool find_flag = false;
//    for (Int32_t i = 0; i < pos_list_size; ++i) {
//        const ad_msg::RelativePos& p = pos_list.relative_pos[i];
//        if (p.relative_time <= relative_time ) {
//            Int32_t time_diff = relative_time - p.relative_time;

//            if (time_diff > 500) {
//                LOG_ERR << "[Error]Time interval is too large.";
//                return false;
//            }

//            if (0 < i) {
//               const ad_msg::RelativePos& next_p = pos_list.relative_pos[i - 1];
//               Float32_t t = static_cast<Float32_t>(time_diff)/static_cast<Float32_t>(next_p.relative_time - p.relative_time);

//               pos->relative_time = time_diff + p.relative_time;
//               pos->x = (1.0F - t) * p.x + t*next_p.x;
//               pos->y = (1.0F - t) * p.y + t*next_p.y;
//               pos->heading = common::AngleLerp(p.heading, next_p.heading, t);
//               pos->yaw_rate = (1.0F - t) * p.yaw_rate + t*next_p.yaw_rate;
//               pos->v = (1.0F - t) * p.v + t*next_p.v;

//            } else {
//                Float32_t prev_pos[3] = { 0.0F };
//                Float32_t nxt_pos[3] = { 0.0F };
//                prev_pos[0] = p.x;
//                prev_pos[1] = p.y;
//                prev_pos[2] = p.heading;
//                vehicle_model.EstimateNextPos(
//                                p.v, p.yaw_rate, 0.001F * time_diff, prev_pos, nxt_pos
//                                );
//                pos->relative_time = time_diff + p.relative_time;
//                pos->x = nxt_pos[0];
//                pos->y = nxt_pos[1];
//                pos->heading = nxt_pos[2];
//                pos->yaw_rate = p.yaw_rate;
//                pos->v = p.v;
//            }
//            find_flag = true;
//            break;
//        }
//    }

//    if(!find_flag){
//        const ad_msg::RelativePos& p = pos_list.relative_pos[pos_list_size - 1];
//        Int32_t time_diff = p.relative_time - relative_time;
//        if (time_diff > 500) {
//            LOG_ERR << "[Error]Time interval is too large.";
//            return false;
//        }

//        Float32_t prev_pos[3] = { 0.0F };
//        Float32_t next_pos[3] = { 0.0F };

//        prev_pos[0] = p.x;
//        prev_pos[1] = p.y;
//        prev_pos[2] = p.heading;
//        vehicle_model.EstimateNextPos(
//                        p.v, p.yaw_rate, -0.001F * time_diff, prev_pos, next_pos
//                        );
//        pos->relative_time = p.relative_time - time_diff;
//        pos->x = next_pos[0];
//        pos->y = next_pos[1];
//        pos->heading = next_pos[2];
//        pos->yaw_rate = p.yaw_rate;
//        pos->v = p.v;

//    }

//    return true;
//}

ObjInfosBase::ObjInfosBase()
{
    Clear();
}

void ObjInfosBase::Clear()
{
    raw_obj_idx_ = -1;
    raw_obj_id_ = -1;
    sensor_type_ = SensorType::SENSOR_TYPE_UNKNOWN;
    obj_type_ = ad_msg::OBJ_TYPE_UNKNOWN;

    width_ = 0.0F;
    height_ = 0.0F;
    length_ = 0.0F;
    v_x_ = 0.0F;
    v_y_ = 0.0F;

    aabb_.Clear();
    pos_.Clear();

}


void ObjPosition::Clear()
{
    x_ = 0.0F;
    y_ = 0.0F;
    angle_ = 0.0F;
    heading_ = 0.0F;
    range_ = 0.0F;
}

ObjPosition::ObjPosition()
{
    Clear();
}

ObjIndex::ObjIndex()
{
    obj_idx_ = -1;
    sensor_idx_ = -1;
}


void ObjFromCamera::Clear()
{
    ObjInfosBase::Clear();
    raw_data_.Clear();
    tracked_obj_index_.Clear();
}

ObjFromCamera::ObjFromCamera()
{
     Clear();
}


void ObjsFromCameraList::Clear()
{
    sensor_id_ = SENSOR_ID_INVALID;
    msg_head_.Clear();

    Int32_t objs_num = objects_.Size();
    for(Int32_t i = 0; i < objs_num; ++i){ // 不能把整个空间进行清空，static_vector中存在着限制防止vector为空时进行清空。
         objects_[i].Clear();
    }
    objects_.Clear();
}

ObjsFromCameraList::ObjsFromCameraList()
{
    Clear();
}


void ObjFromRadar::Clear()
{
    ObjInfosBase::Clear();
    raw_data_.Clear();
    tracked_obj_index_.Clear();

}

ObjFromRadar::ObjFromRadar()
{
    Clear();
}

void ObjsFromRadarList::Clear()
{
    msg_head_.Clear();    
    sensor_id_ = SENSOR_ID_INVALID;

    Int32_t objs_num = objects_.Size();
    for(Int32_t i = 0; i < objs_num; ++i){
         objects_[i].Clear();
    }
    objects_.Clear();
}

ObjsFromRadarList::ObjsFromRadarList()
{
    Clear();
}


void ObjFromLidar::Clear()
{
    ObjInfosBase::Clear();
    tracked_obj_index_.Clear();
    raw_data_.Clear();
}

ObjFromLidar::ObjFromLidar()
{
    Clear();
}

void ObjsFromLidarList::Clear()
{
    msg_head_.Clear();
    sensor_id_ = SENSOR_ID_INVALID;

    int objs_num_ = objects_.Size();
    for (Int32_t i = 0; i < objs_num_; ++i) {
        objects_[i].Clear();
    }
    objects_.Clear();
}

ObjsFromLidarList::ObjsFromLidarList()
{
    Clear();
}

ObjIndex::ObjIndex(const Int32_t obj_idx, const Int32_t sen)
{
    obj_idx_ = obj_idx;
    sensor_idx_ = sen;
}

void ObjIndex::Clear()
{
    obj_idx_ = -1;
    sensor_idx_ = -1;
}

bool ObjIndex::IsValid() const
{
    return ((obj_idx_ >= 0) && (sensor_idx_ >= 0));
}

void RelativePos::Clear()
{
    relative_time_ = 0;

    x_ = 0.0F;
    y_ = 0.0F;
    heading_ = 0.0F;
    yaw_rate_ = 0.0F;
    v_ = 0.0F;

}


ObjsFilterDataSource::ObjsFilterDataSource()
{
    timestamp_ = 0;
    rel_pos_list_ = Nullptr_t;

    objs_list_main_forward_cam_ = Nullptr_t;
    objs_list_main_forward_radar_ = Nullptr_t;
    objs_list_main_forward_lidar_ = Nullptr_t;

    objs_list_left_side_radar_ = Nullptr_t;
    objs_list_right_side_radar_ = Nullptr_t;
    objs_list_left_backward_radar_ = Nullptr_t;
    objs_list_right_backward_radar_ = Nullptr_t;

    objs_list_center_forward_cam_ = Nullptr_t;
    objs_list_left_side_cam_ = Nullptr_t;
    objs_list_right_side_cam_ = Nullptr_t;
    objs_list_left_backward_cam_ = Nullptr_t;
    objs_list_right_backward_cam_ = Nullptr_t;

    driving_map_ = Nullptr_t;

}

void RelativePosList::Clear()
{
    msg_head_.Clear();
    relative_pos_num_ = 0;
    for (Int32_t i = 0; i < RelativePosList::MAX_RELATIVE_POS_NUM; ++i) {
      relative_pos_[i].Clear();
    }

}

void SensorsRef::Clear()
{
    valid_ = false;

    for (Int32_t i = 0; i < SensorsRef::MAX_OBJ_ID_NUM; ++i) {
        obj_id_[i].Clear();
    }

    sensor_type_ = SENSOR_TYPE_UNKNOWN;
    obj_type_ = -1;
    width_ = 0.0F;
    height_ = 0.0F;
    length_ = 0.0F;
    common::com_memset(pos_, 0, sizeof(pos_));
    aabb_.Clear();
    v_x_ = 0.0F;
    v_y_ = 0.0F;
    age_ = 0;
    last_age_ = 0;
}


Int32_t SensorsRef::FindObjId(const Int32_t id) const
{
    for (Int32_t i = 0; i < SensorsRef::MAX_OBJ_ID_NUM; ++i) {
        if (obj_id_[i].id_ == id) {
            return i;
        }
    }
    return -1;
}


Int32_t SensorsRef::FetchSensorObjId() const
{
    return obj_id_[0].id_ < 0 ? obj_id_[1].id_ : obj_id_[0].id_;
}


Int32_t SensorsRef::AddPartnerObjId(const Int32_t id, const Int32_t duration)
{
    Int32_t index = FindObjId(id);

    if (index >= 0) {
        obj_id_[index].age_ = duration;
        return index;
    } else {
        Int32_t empty_id_idx = -1;
        Int32_t max_age_diff_idx = -1;
        Int32_t max_age_diff = 1;

        for (Int32_t i = 1; i < MAX_OBJ_ID_NUM; ++i) {
            if ((obj_id_[i].id_ < 0) &&
                    (empty_id_idx < 0))
                empty_id_idx = i;

            Int32_t age_diff = duration - obj_id_[i].age_;
            if (age_diff > max_age_diff) {
                max_age_diff = age_diff;
                max_age_diff_idx = i;
            }
        }

        if (empty_id_idx >= 0) {
            obj_id_[empty_id_idx].id_ = id;
            obj_id_[empty_id_idx].age_ = duration;
            return empty_id_idx;
        } else {
            if (max_age_diff_idx >= 0) {
                obj_id_[max_age_diff_idx].id_ = id;
                obj_id_[max_age_diff_idx].age_ = duration;
                return max_age_diff_idx;
            }
        }
    }

    return -1;
}

void SensorsFusionObj::Clear()
{
    valid_ = false;
    obj_type_ = ad_msg::OBJ_TYPE_UNKNOWN;
    width_ = 0.0F;
    length_ = 0.0F;
    pos_.Clear();
    aabb_.Clear();
    v_x_ = 0.0F;
    v_y_ = 0.0F;
}

TrjPointInfos::TrjPointInfos()
{
    Clear();
}

void TrjPointInfos::Clear()
{
    seq_num_ = 0;
    relative_time_ms_ = 0.0F;
    x_ = 0.0F;
    y_ = 0.0F;
    vx_ = 0.0F;
    vy_ = 0.0F;
}

ObjsTracked::ObjsTracked()
{
    Clear();
}


void ObjsTracked::Clear(ObjAssoPoolType &obj_ass_pool)
{
    Clear();
    ClearAssoList(obj_ass_pool);
}

void ObjsTracked::Clear()
{
    erase_flag_ = false;

    list_id_ = LIST_ID_INVALID;
    raw_obj_idx_ = -1;

    track_status_ = TRACK_STATUS_INVALID;
    life_ = 0;
    life_ref_count_ = 0;
    last_life_ = 0;
    matched_ = 0;
    age_ = 0;
    duration_ = 0;
    confidence_ = 0;

    for (Int32_t i = 0; i < SENSOR_ID_MAX; ++i) {
        sensor_ref_[i].Clear();
    }

    common::com_memset(&avg_status_, 0, sizeof(avg_status_));
    heading_ = 0.0F;
    pos_status_.SetZeros();
    pos_covariance_.SetIdentity();
    ClearTrackedTrj();
    ref_line_.Clear();
}

void ObjsTracked::ClearAssoList(ObjAssoPoolType &obj_ass_pool)
{
    for (Int32_t i = 0; i < LIST_ID_MAX; ++i) {
        objs_ass_.objs_ass_list_[i].Clear(obj_ass_pool);
    }
}

void ObjsTracked::ClearTrackedTrj()
{
    for (Int32_t i = 0; i < traced_trj_[0].Size(); ++i)
        traced_trj_[0][i].Clear();
    traced_trj_[0].Clear();

    for (Int32_t i = 0; i < traced_trj_[1].Size(); ++i)
        traced_trj_[1][i].Clear();
    traced_trj_[1].Clear();
}

void ObjsTracked::GetObjOverallDimension(const Int32_t sen_idx, Float32_t *length, Float32_t *width, Float32_t* height) const
{
    if (sensor_ref_[SensorID::SENSOR_ID_CENTER_FORWARD_CAM].valid_) {
        *length = sensor_ref_[SensorID::SENSOR_ID_CENTER_FORWARD_CAM].length_;
        *width = sensor_ref_[SensorID::SENSOR_ID_CENTER_FORWARD_CAM].width_;
        *height = sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_CAM].height_;
    } else {
        *length = sensor_ref_[sen_idx].length_;
        *width = sensor_ref_[sen_idx].width_;
        *height = sensor_ref_[sen_idx].height_;
    }
}

bool ObjsTracked::IsOhterSensorValid(const Int32_t sen_id) const
{
    bool other_sensor_valid = false;
    for (Int32_t i = 0; i < SensorID::SENSOR_ID_MAX; ++i) {
        if (i != sen_id) {
            if (sensor_ref_[i].valid_) {
                other_sensor_valid = true;
                break;
            }
        }
    }

    return other_sensor_valid;
}

bool ObjsTracked::IsRadarValid() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_RADAR].valid_)
        return true;

    return false;
}

bool ObjsTracked::IsCameraValid() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_CENTER_FORWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_CAM].valid_)
        return true;
//    if (sensor_ref_[SensorID::SENSOR_ID_INSIDE_DMS_CAM].valid_)   //这两个暂时不用
//        return true;
//    if (sensor_ref_[SensorID::SENSOR_ID_INSIDE_OMS_CAM].valid_)
//        return true;


    return false;
}


bool ObjsTracked::IsForesightObj() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_CENTER_FORWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_RADAR].valid_)
        return true;
    return false;
}

bool ObjsTracked::IsAroundObj() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_RADAR].valid_)
        return true;

    return false;
}


bool ObjsTracked::IsLeftSensorObj() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_BACKWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_LEFT_SIDE_RADAR].valid_)
        return true;

    return false;
}

bool ObjsTracked::IsRightSensorObj() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_CAM].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_)
        return true;
    if (sensor_ref_[SensorID::SENSOR_ID_RIGHT_SIDE_RADAR].valid_)
        return true;

    return false;
}


bool ObjsTracked::IsLidarValid() const
{
    if (sensor_ref_[SensorID::SENSOR_ID_MAIN_FORWARD_LIDAR].valid_)
        return true;

    return false;
}

/**
 * @brief 用来生成FusionID
 * 
 *
*/
void ObjsTracked::GenerateJointModeFusionID()
{
    Uint8_t radar_type = ad_msg::Obstacle::NONE_RADAR_INDEX_FOR_ID;
    Uint8_t camera_type = ad_msg::Obstacle::NONE_CAMERA_INDEX_FOR_ID;
    Uint8_t exist_lidar = 0;
    Uint8_t radar_id = 0;
    Uint8_t camera_id = 0;
    Uint8_t lidar_id = 0;
    ObjID * id_tmps;
    if (sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
        radar_type = ad_msg::Obstacle::MAIN_FRONT_RADAR_INDEX_FOR_ID;
        radar_id = sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_LEFT_SIDE_RADAR].valid_){
        radar_type = ad_msg::Obstacle::LEFT_FRONT_RADAR_INDEX_FOR_ID;
        radar_id = sensor_ref_[SENSOR_ID_LEFT_SIDE_RADAR].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_LEFT_BACKWARD_RADAR].valid_){
        radar_type = ad_msg::Obstacle::LEFT_REAR_RADAR_INDEX_FOR_ID;
        radar_id = sensor_ref_[SENSOR_ID_LEFT_BACKWARD_RADAR].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_RIGHT_SIDE_RADAR].valid_){
        radar_type = ad_msg::Obstacle::RIGHT_FRONT_RADAR_INDEX_FOR_ID;
        radar_id = sensor_ref_[SENSOR_ID_RIGHT_SIDE_RADAR].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_){
        radar_type = ad_msg::Obstacle::RIGHT_REAR_RADAR_INDEX_FOR_ID;
        radar_id = sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_RADAR].FetchSensorObjId();
    }

    if (sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].valid_) {
        camera_type = ad_msg::Obstacle::MAIN_FRONT_CAMERA_INDEX_FOR_ID;
        camera_id = sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_LEFT_SIDE_CAM].valid_){
        camera_type = ad_msg::Obstacle::LEFT_SIDE_CAMERA_INDEX_FOR_ID;
        camera_id = sensor_ref_[SENSOR_ID_LEFT_SIDE_CAM].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_LEFT_BACKWARD_CAM].valid_){
        camera_type = ad_msg::Obstacle::LEFT_REAR_CAMERA_INDEX_FOR_ID;
        camera_id = sensor_ref_[SENSOR_ID_LEFT_BACKWARD_CAM].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_RIGHT_SIDE_CAM].valid_){
        camera_type = ad_msg::Obstacle::RIGHT_SIDE_CAMERA_INDEX_FOR_ID;
        camera_id = sensor_ref_[SENSOR_ID_RIGHT_SIDE_CAM].FetchSensorObjId();
    }
    else if (sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_CAM].valid_){
        camera_type = ad_msg::Obstacle::RIGHT_REAR_CAMERA_INDEX_FOR_ID;
        camera_id = sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_CAM].FetchSensorObjId();
    }

    if (sensor_ref_[SENSOR_ID_MAIN_FORWARD_LIDAR].valid_){
        exist_lidar = 1;
    }

    Int32_t fusion_id_tmp = 0;
    bool ret = ad_msg::Obstacle::EncodeFusionID(radar_type,camera_type,exist_lidar,
                                                radar_id,camera_id,lidar_id,fusion_id_tmp);
    fusion_id = fusion_id_tmp;
}


void ObjsTracked::CalcFusedSensorObj(SensorsFusionObj *fused_obj)
{
    Int32_t mask = 0;

    fused_obj->valid_ = false;
    fused_obj->obj_type_ = ad_msg::OBJ_TYPE_UNKNOWN;
    fused_obj->width_ = 0.2F;
    fused_obj->height_ = 0.2F;
    fused_obj->length_ = 0.2F;

    Float32_t sum_pos_x = 0.0F;
    Float32_t sum_pos_y = 0.0F;
    Float32_t sum_pos_ratio = 0.0F;
    Float32_t sum_v_x = 0.0F;
    Float32_t sum_v_y = 0.0F;
    Float32_t sum_v_ratio = 0.0F;

    Float32_t radar_y = 0.0F;
    Int8_t main_y_flag = 0;

    if (sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR].valid_) {
        const SensorsRef& sen = sensor_ref_[SENSOR_ID_MAIN_FORWARD_RADAR];
        // 毫米波识别的障碍物类型不准

        // 尺寸取大值
        if (sen.width_ > fused_obj->width_) {
          fused_obj->width_ = sen.width_;
        }
        if (sen.length_ > fused_obj->length_) {
          fused_obj->length_ = sen.length_;
        }

        // 毫米波位置取加权平均值
        Float32_t pos_ratio = 1.0F;
        sum_pos_x += pos_ratio * sen.pos_[0].x_;
        sum_pos_y += pos_ratio * sen.pos_[0].y_;
        sum_pos_ratio += pos_ratio;

        radar_y = sen.pos_[0].y_;

        // 包围盒最后计算

        // 毫米波雷达车速比较准 
        Float32_t v_ratio = 1.0F;
        sum_v_x += v_ratio * sen.v_x_;
        sum_v_y += v_ratio * sen.v_y_;
        sum_v_ratio += v_ratio;

        // 使用毫米波纵向加速度 
        fused_obj->a_x_ = sen.a_x_;

        mask |= (0x1 << SENSOR_ID_MAIN_FORWARD_RADAR);
    }

    if (sensor_ref_[SENSOR_ID_LEFT_SIDE_RADAR].valid_) {
        const SensorsRef& sen = sensor_ref_[SENSOR_ID_LEFT_SIDE_RADAR];
        // 毫米波识别的障碍物类型不准

        // 尺寸取大值
        if (sen.width_ > fused_obj->width_) {
          fused_obj->width_ = sen.width_;
        }
        if (sen.length_ > fused_obj->length_) {
          fused_obj->length_ = sen.length_;
        }

        // 毫米波位置取加权平均值
        Float32_t pos_ratio = 1.0F;
        sum_pos_x += pos_ratio * sen.pos_[0].x_;
        sum_pos_y += pos_ratio * sen.pos_[0].y_;
        sum_pos_ratio += pos_ratio;

        // 包围盒最后计算

        // 毫米波雷达车速比较准 
        Float32_t v_ratio = 1.0F;
        sum_v_x += v_ratio * sen.v_x_;
        sum_v_y += v_ratio * sen.v_y_;
        sum_v_ratio += v_ratio;

        mask |= (0x1 << SENSOR_ID_LEFT_SIDE_RADAR);
    }
    if (sensor_ref_[SENSOR_ID_LEFT_BACKWARD_RADAR].valid_) {
        const SensorsRef& sen = sensor_ref_[SENSOR_ID_LEFT_BACKWARD_RADAR];
        // 毫米波识别的障碍物类型不准

        // 尺寸取大值
        if (sen.width_ > fused_obj->width_) {
          fused_obj->width_ = sen.width_;
        }
        if (sen.length_ > fused_obj->length_) {
          fused_obj->length_ = sen.length_;
        }

        // 毫米波位置取加权平均值
        Float32_t pos_ratio = 1.0F;
        sum_pos_x += pos_ratio * sen.pos_[0].x_;
        sum_pos_y += pos_ratio * sen.pos_[0].y_;
        sum_pos_ratio += pos_ratio;

        // 包围盒最后计算

        // 毫米波雷达车速比较准 
        Float32_t v_ratio = 1.0F;
        sum_v_x += v_ratio * sen.v_x_;
        sum_v_y += v_ratio * sen.v_y_;
        sum_v_ratio += v_ratio;

        mask |= (0x1 << SENSOR_ID_LEFT_BACKWARD_RADAR);
    }
    if (sensor_ref_[SENSOR_ID_RIGHT_SIDE_RADAR].valid_) {
        const SensorsRef& sen = sensor_ref_[SENSOR_ID_RIGHT_SIDE_RADAR];
        // 毫米波识别的障碍物类型不准

        // 尺寸取大值
        if (sen.width_ > fused_obj->width_) {
          fused_obj->width_ = sen.width_;
        }
        if (sen.length_ > fused_obj->length_) {
          fused_obj->length_ = sen.length_;
        }

        // 毫米波位置取加权平均值
        Float32_t pos_ratio = 1.0F;
        sum_pos_x += pos_ratio * sen.pos_[0].x_;
        sum_pos_y += pos_ratio * sen.pos_[0].y_;
        sum_pos_ratio += pos_ratio;

        // 包围盒最后计算

        // 毫米波雷达车速比较准 
        Float32_t v_ratio = 1.0F;
        sum_v_x += v_ratio * sen.v_x_;
        sum_v_y += v_ratio * sen.v_y_;
        sum_v_ratio += v_ratio;

        mask |= (0x1 << SENSOR_ID_RIGHT_SIDE_RADAR);
    }
    if (sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_RADAR].valid_) {
        const SensorsRef& sen = sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_RADAR];
        // 毫米波识别的障碍物类型不准

        // 尺寸取大值
        if (sen.width_ > fused_obj->width_) {
          fused_obj->width_ = sen.width_;
        }
        if (sen.length_ > fused_obj->length_) {
          fused_obj->length_ = sen.length_;
        }

        // 毫米波位置取加权平均值
        Float32_t pos_ratio = 1.0F;
        sum_pos_x += pos_ratio * sen.pos_[0].x_;
        sum_pos_y += pos_ratio * sen.pos_[0].y_;
        sum_pos_ratio += pos_ratio;

        // 包围盒最后计算

        // 毫米波雷达车速比较准 
        Float32_t v_ratio = 1.0F;
        sum_v_x += v_ratio * sen.v_x_;
        sum_v_y += v_ratio * sen.v_y_;
        sum_v_ratio += v_ratio;

        mask |= (0x1 << SENSOR_ID_RIGHT_BACKWARD_RADAR);
    }

    //TODO: 添加毫米波雷达传感器针对同一个目标的融合结果进行加权平均取值

    //毫米波雷达位置取加权平均值
    //毫米波雷达车速取加权平均值
    if (0 != mask) {
        fused_obj->pos_.x_ = sum_pos_x / sum_pos_ratio;
        fused_obj->pos_.y_ = sum_pos_y / sum_pos_ratio;


       // LOG_INFO(2) << "fused_obj->v_x_ :" << fused_obj->v_x_ << "  sum_v_x:"<< sum_v_x << " sum_v_ratio:" << sum_v_ratio;

        fused_obj->v_x_ = sum_v_x / sum_v_ratio;
        fused_obj->v_y_ = sum_v_y / sum_v_ratio;
    }

    //添加视觉传感器
    if (sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM].valid_) {
      SensorsRef& sen = sensor_ref_[SENSOR_ID_MAIN_FORWARD_CAM];
      // 相机识别的障碍物类型比较准
      fused_obj->obj_type_ = sen.obj_type_;

      // 尺寸取大值
      if (sen.width_ > fused_obj->width_) {
        fused_obj->width_ = sen.width_;
      }
      if (sen.length_ > fused_obj->length_) {
        fused_obj->length_ = sen.length_;
      }

      // 包围盒最后计算

      // 相机横向位置比较准

      if(0 != mask){
        /**
         * @brief （2023.4.27.之前所写注释）进行加权平均处理之前毫米波雷达的横中线比率都是“pos_ratio = 1.0”，此处横纵向进行分开处理
         * 此处使用的前视一体机是智驾的，横向距离不够准，所以需要与大陆的前向雷达进行加权平均。横纵向加权比例如下
         * 毫米波雷达与相机权重比例： 横向为 1.0 比 1.8 ，纵向为 1.0 比 0.1 。
         * 
         * 2023.4.27.改成直接使用相机的横向距离 
         */
        // Float32_t sum_pos_y_ratio = sum_pos_ratio;
        Float32_t sum_pos_x_ratio = sum_pos_ratio;
        Float32_t pos_x_ratio = 0.1F;
        sum_pos_x += pos_x_ratio * sen.pos_[0].x_;
        sum_pos_x_ratio += pos_x_ratio;
        fused_obj->pos_.x_ = sum_pos_x / sum_pos_x_ratio;
                    
        fused_obj->pos_.y_ = sen.pos_[0].y_;  //2023.4.27.改成直接使用相机的横向距离  
        fused_obj->v_y_ = sen.v_y_; 

        //使用相机横向加速度
        fused_obj->a_y_ = sen.a_y_;    

        /*
        * 当进入前视一体机的盲区时，由于感知数据的暂留，保存的瞬时速度有可能不准，需要毫米波雷达横向距离进行加权平均。
        * 而且当纵向距离较近，相机横向速度和距离值有可能不准，导致最后横向距离计算错误。
        * 这里将界限距离设置为12
        */
        if(fused_obj->pos_.x_ < MIN_X_VALUE_LIMIT_FOR_FUSED_Y_VY_VALUE_CALC)//15
        {
            Float32_t sum_v_y_ratio = sum_v_ratio;//相机横向速度
            Float32_t v_y_ratio = 0.12F;
            sum_v_y += v_y_ratio * sen.v_y_;
            sum_v_y_ratio += v_y_ratio;
            fused_obj->v_y_ = sum_v_y/sum_v_y_ratio; 


            // Float32_t sum_pos_y_ratio = sum_pos_ratio;
            // Float32_t pos_y_ratio = 15.0F;
            // sum_pos_y += pos_y_ratio * sen.pos_[0].y_;
            // sum_pos_y_ratio += pos_y_ratio;
            // fused_obj->pos_.y_ = sum_pos_y / sum_pos_y_ratio;
            
            // fused_obj->v_y_ = sen.v_y_;
            fused_obj->pos_.y_ = sen.pos_[0].y_; 
        }
        else{
                                
            fused_obj->pos_.y_ = sen.pos_[0].y_;  //2023.4.27.改成直接使用相机的横向距离
            fused_obj->v_y_ = sen.v_y_;   
        }

        // 如果前视一体机突然消失
        if (sen.last_age_ == 0) {
            sen.last_age_ = sen.age_;
            main_y_flag = 0;
        } else if (sen.last_age_ == sen.age_) {
            main_y_flag = 1;
        } else if (sen.last_age_ != sen.age_) {
            sen.last_age_ = sen.age_;
            main_y_flag = 0;
        }

        if (main_y_flag == 1) {
            fused_obj->pos_.y_ = radar_y;
        } else if (main_y_flag == 0) {
            fused_obj->pos_.y_ = sen.pos_[0].y_;
        }      
      }
      else{
        // 其它传感器无数据
        // 相机识别的纵向距离不太准
        fused_obj->pos_.x_ = sen.pos_[0].x_;
        fused_obj->pos_.y_ = sen.pos_[0].y_;
        // 相机识别的车速不太准
        fused_obj->v_x_ = sen.v_x_;
        fused_obj->v_y_ = sen.v_y_;

        //使用相机横向加速度
        fused_obj->a_y_ = sen.a_y_;   
      }

      mask |= (0x1 << SENSOR_ID_MAIN_FORWARD_CAM);
    }


    #if 1
    //前视
    /*
    if (sensor_ref_[SENSOR_ID_CENTER_FORWARD_CAM].valid_) {
      const SensorsRef& sen = sensor_ref_[SENSOR_ID_CENTER_FORWARD_CAM];
      // 相机识别的障碍物类型比较准
      fused_obj->obj_type_ = sen.obj_type_;

      // 尺寸取大值
      if (sen.width_ > fused_obj->width_) {
        fused_obj->width_ = sen.width_;
      }
      if (sen.length_ > fused_obj->length_) {
        fused_obj->length_ = sen.length_;
      }

      // 包围盒最后计算

      // 相机横向位置比较准
      fused_obj->pos_.y_ = sen.pos_[0].y_;
      if (0 == mask) {
        // 其它传感器无数据
        // 相机识别的纵向距离不太准
        fused_obj->pos_.x_ = sen.pos_[0].x_;
        // 相机识别的车速不太准
        fused_obj->v_x_ = sen.v_x_;
        fused_obj->v_y_ = sen.v_y_;
      }

      mask |= (0x1 << SENSOR_ID_CENTER_FORWARD_CAM);
    }

    //左前
    if (sensor_ref_[SENSOR_ID_LEFT_SIDE_CAM].valid_) {
      const SensorsRef& sen = sensor_ref_[SENSOR_ID_LEFT_SIDE_CAM];
      // 相机识别的障碍物类型比较准
      fused_obj->obj_type_ = sen.obj_type_;

      // 尺寸取大值
      if (sen.width_ > fused_obj->width_) {
        fused_obj->width_ = sen.width_;
      }
      if (sen.length_ > fused_obj->length_) {
        fused_obj->length_ = sen.length_;
      }

      // 包围盒最后计算

      // 相机横向位置比较准
      fused_obj->pos_.y_ = sen.pos_[0].y_;
      if (0 == mask) {
        // 其它传感器无数据
        // 相机识别的纵向距离不太准
        fused_obj->pos_.x_ = sen.pos_[0].x_;
        // 相机识别的车速不太准
        fused_obj->v_x_ = sen.v_x_;
        fused_obj->v_y_ = sen.v_y_;
      }

      mask |= (0x1 << SENSOR_ID_LEFT_SIDE_CAM);
    }

    //右前
    if (sensor_ref_[SENSOR_ID_RIGHT_SIDE_CAM].valid_) {
      const SensorsRef& sen = sensor_ref_[SENSOR_ID_RIGHT_SIDE_CAM];
      // 相机识别的障碍物类型比较准
      fused_obj->obj_type_ = sen.obj_type_;

      // 尺寸取大值
      if (sen.width_ > fused_obj->width_) {
        fused_obj->width_ = sen.width_;
      }
      if (sen.length_ > fused_obj->length_) {
        fused_obj->length_ = sen.length_;
      }

      // 包围盒最后计算

      // 相机横向位置比较准
      fused_obj->pos_.y_ = sen.pos_[0].y_;
      if (0 == mask) {
        // 其它传感器无数据
        // 相机识别的纵向距离不太准
        fused_obj->pos_.x_ = sen.pos_[0].x_;
        // 相机识别的车速不太准
        fused_obj->v_x_ = sen.v_x_;
        fused_obj->v_y_ = sen.v_y_;
      }

      mask |= (0x1 << SENSOR_ID_RIGHT_SIDE_CAM);
    }*/

    //左后
    if (sensor_ref_[SENSOR_ID_LEFT_BACKWARD_CAM].valid_) {
      const SensorsRef& sen = sensor_ref_[SENSOR_ID_LEFT_BACKWARD_CAM];
      // 相机识别的障碍物类型比较准
      fused_obj->obj_type_ = sen.obj_type_;

      // 尺寸取大值
      if (sen.width_ > fused_obj->width_) {
        fused_obj->width_ = sen.width_;
      }
      
      if (sen.length_ > fused_obj->length_) {
        fused_obj->length_ = sen.length_;
      }

      // 包围盒最后计算

      // 相机横向位置比较准
      fused_obj->pos_.y_ = sen.pos_[0].y_;
      if (0 == mask) {
        // 其它传感器无数据
        // 相机识别的纵向距离不太准
        fused_obj->pos_.x_ = sen.pos_[0].x_;
        // 相机识别的车速不太准
        fused_obj->v_x_ = sen.v_x_;
        fused_obj->v_y_ = sen.v_y_;
      }

      mask |= (0x1 << SENSOR_ID_LEFT_BACKWARD_CAM);
    }

    //右后
    if (sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_CAM].valid_) {
      const SensorsRef& sen = sensor_ref_[SENSOR_ID_RIGHT_BACKWARD_CAM];
      // 相机识别的障碍物类型比较准
      fused_obj->obj_type_ = sen.obj_type_;

      // 尺寸取大值
      if (sen.width_ > fused_obj->width_) {
        fused_obj->width_ = sen.width_;
      }
      if (sen.length_ > fused_obj->length_) {
        fused_obj->length_ = sen.length_;
      }

      // 包围盒最后计算

      // 相机横向位置比较准
      fused_obj->pos_.y_ = sen.pos_[0].y_;
      if (0 == mask) {
        // 其它传感器无数据
        // 相机识别的纵向距离不太准
        fused_obj->pos_.x_ = sen.pos_[0].x_;
        // 相机识别的车速不太准
        fused_obj->v_x_ = sen.v_x_;
        fused_obj->v_y_ = sen.v_y_;
      }

      mask |= (0x1 << SENSOR_ID_RIGHT_BACKWARD_CAM);
    }
    #endif

    if (0 != mask) {
      fused_obj->valid_ = true;

      // 计算包围盒
      Float32_t half_width = 0.5F * fused_obj->length_;
      if (fused_obj->pos_.x_ > 0.0F) {
        fused_obj->aabb_.min().set_x(fused_obj->pos_.x_);
        fused_obj->aabb_.max().set_x(fused_obj->pos_.x_ + fused_obj->length_);
      } else {
        fused_obj->aabb_.min().set_x(fused_obj->pos_.x_ - fused_obj->length_);
        fused_obj->aabb_.max().set_x(fused_obj->pos_.x_);
      }
      fused_obj->aabb_.min().set_y(fused_obj->pos_.y_ - half_width);
      fused_obj->aabb_.max().set_y(fused_obj->pos_.y_ + half_width);

      fused_obj->pos_.range_ = common::com_sqrt(
            common::Square(fused_obj->pos_.x_) + common::Square(fused_obj->pos_.y_));
      fused_obj->pos_.angle_ =
          common::com_atan2(fused_obj->pos_.y_, fused_obj->pos_.x_);
    }

    //TODO:添加激光雷达结果进行更新障碍物值
}

MatchingCondition::MatchingCondition()
{
    Clear();
}

void MatchingCondition::Clear()
{
    valid_ = false;
    max_x_diff_ = 0.0F;
    max_y_diff_ = 0.0F;
    max_vx_diff_ = 0.0F;
    max_vy_diff_ = 0.0F;

}

void MatchingParams::Clear()
{
    searching_radius_ = 0.0F;
    for (Int32_t i = 0; i < SENSOR_ID_MAX; ++i) {
      masks_[i] = false;
    }

    matched_sensors_nums_ = 0;
    min_matched_times_ = 0;
    max_dismatch_times_ = 10;
    max_vx_diff_ = 10.0F/3.6F;
    max_vy_diff_ = 100.0F/3.6F;

    condition_.Clear();

    using_est_pos_ = false;
    enable_add_mul_obj_id_ = false;
    enable_sub_optimal_match_ = false;
    match_again_by_id_if_unmatched_ = false;
    match_with_velocity_ = true;

    //若match_again_by_id_if_unmatched_设置为true，以下参数才有效，并用作判断id相同的相机数据与其他传感器数据物理上的匹配阈值。
    max_x_diff_for_matched_again_by_id = 20.0F;
    max_y_diff_for_matched_again_by_id = 8.0F;
    max_vx_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vx不要小于前面的max_vx_XXX值。
    max_vy_diff_for_matched_again_by_id = 40.0F/3.6F;//这里的max_vy不要小于前面的max_vy_XXX值。

}

void EgoVehicleParams::Clear()
{
    vehicle_length_ = 0.0F;
    vehicle_width_ = 0.0F;
    vehicle_height_ = 0.0F;
    dist_of_localization_to_center_ = 0.0F;
    dist_of_localization_to_front_ = 0.0F;
    dist_of_localization_to_rear_ = 0.0F;
}

EgoVehicleParams &EgoVehicleParams::operator =(const EgoVehicleParams &vehicle_params)
{
    vehicle_length_ = vehicle_params.vehicle_length_;
    vehicle_width_ = vehicle_params.vehicle_width_;
    vehicle_height_ = vehicle_params.vehicle_height_;
    dist_of_localization_to_front_ = vehicle_params.dist_of_localization_to_front_;
    dist_of_localization_to_rear_ = vehicle_params.dist_of_localization_to_rear_;
    dist_of_localization_to_center_ = vehicle_params.dist_of_localization_to_center_;

    return *this;

}

EgoVehicleParams::EgoVehicleParams()
{
    Clear();
}


} //sensors fusion
} //perception
} //phoenix
