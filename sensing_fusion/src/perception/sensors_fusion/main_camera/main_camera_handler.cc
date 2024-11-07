#include "main_camera_handler.h"
#include "sensors_fusion/sensors_fusion_macros.h"
#include <unordered_map>

namespace phoenix{
namespace perception{
namespace main_camera{


MainCameraHandler::MainCameraHandler(){
    rel_pos_list_ = Nullptr_t;
    geofence_info_ = Nullptr_t;
    last_geofence_timestamp_ = 0;

    last_camera_obstacle_vec_.clear();
}

MainCameraHandler::~MainCameraHandler(){


}

bool MainCameraHandler::ConstructPath(ad_msg::LaneMarkCamera& lane_curb, common::Path& path)
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

void MainCameraHandler::SetRelativePosList(ad_msg::RelativePosList* rel_pos_list)
{
    rel_pos_list_ = rel_pos_list;    
}

void MainCameraHandler::SetMapGeofenceInfo(exp_map_t::stMapGeofenceInfo * geofence_info)
{
    geofence_info_ = geofence_info;
}

/**
 * @brief 是否需要加入超时的判断？
 * 
 * @return true 
 * @return false 
 */
bool MainCameraHandler::IsInTunnel()
{
    bool have_tunnel_reason = false;
    if (last_geofence_timestamp_ != geofence_info_->m_uTimeStamp)
    {
        if (geofence_info_->m_bInGeofence == false && geofence_info_->m_currentInReasons.find(exp_map_t::Reason::TUNNEL) != geofence_info_->m_currentInReasons.end())
        {
            have_tunnel_reason = true;
        }
    }
    last_geofence_timestamp_ = geofence_info_->m_uTimeStamp;
    return have_tunnel_reason;
}



/**
 * @brief 后续会加入是否在隧道的判断！！！！！！！
 * 
 * @param camera_list 
 * @param lane_curb_list 
 */
void MainCameraHandler::UpdateMainCameraList(ad_msg::ObstacleCameraList& camera_list, ad_msg::LaneMarkCameraList& lane_curb_list)
{

    Int64_t cur_timestamp = lane_curb_list.msg_head.timestamp;
    Int64_t obj_timestamp = camera_list.msg_head.timestamp;
    std::vector<size_t> invalid_ind_vec;

    LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "lane_curb_num:" << lane_curb_list.lane_mark_num;
    LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "obstacle_num:" << camera_list.obstacle_num;
    LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "cur_timestamp:" << cur_timestamp;
    LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "obj_timestamp:" << obj_timestamp;
    
    if(!IsInTunnel())
    {
        return;
    }

    if(0 == lane_curb_list.lane_mark_num || 0 == camera_list.obstacle_num){
        return;
    }
    std::vector<common::Path> lane_curb_paths(MAX_LANE_CURB_NUM);
    for (int i = 0; i < lane_curb_list.lane_mark_num; i++)
    {
        common::Path& path = lane_curb_paths[i]; 
        ad_msg::LaneMarkCamera& lane_curb = lane_curb_list.lane_marks[i];
        
        if(!ConstructPath(lane_curb, path)){
            path.Clear();
        }

    }
    
    ad_msg::RelativePos cur_rel_pos;
    ad_msg::RelativePos obj_rel_pos;
    bool get_cur_pos_true = false;
    bool get_obj_pos_true = false;
    common::Matrix<Float32_t, 3, 3> mat_conv_sensor;
    mat_conv_sensor.SetIdentity();
    common::Matrix<Float32_t, 2, 1> rotate_center;
    rotate_center.SetZeros();
    if (phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          cur_timestamp, *rel_pos_list_, &cur_rel_pos)) 
    {
        get_cur_pos_true  = true;
    } 
    
    if (phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
          obj_timestamp, *rel_pos_list_, &obj_rel_pos)) 
    {
        get_obj_pos_true  = true;
    } 
    
    LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "get_cur_pos_true:" << get_cur_pos_true << " get_obj_pos_true:" << get_obj_pos_true;

    if(get_cur_pos_true && get_obj_pos_true) {
        common::Rotate_2D<Float32_t>(rotate_center,
                                    obj_rel_pos.heading - cur_rel_pos.heading,
                                    &mat_conv_sensor);
        common::Translate_2D(obj_rel_pos.x - cur_rel_pos.x,
                            obj_rel_pos.y - cur_rel_pos.y,
                            &mat_conv_sensor);
    }
    common::Matrix<Float32_t, 3, 3> mat_conv_composite = mat_conv_sensor;
    Float32_t time_diff = (cur_timestamp - obj_timestamp)*0.001;
    common::Matrix<Float32_t, 2, 1> point_conv;

    for(int i = 0; i < camera_list.obstacle_num; i++)
    {
        ad_msg::ObstacleCamera & obstacle =  camera_list.obstacles[i];  
        /**
         * @brief 以下代码将obstacle 与路沿进行时空同步 
         */
        if(get_cur_pos_true && get_obj_pos_true)
        {
            /**
             * @brief 坐标系进行转换
             */
            point_conv(0) = obstacle.x;
            point_conv(1) = obstacle.y;
            common::TransformVert_2D(mat_conv_composite, &point_conv);
            obstacle.x= point_conv(0);
            obstacle.y = point_conv(1);
        }

        obstacle.x = obstacle.x + obstacle.v_x * time_diff;
        obstacle.y = obstacle.y + obstacle.v_y * time_diff;

        common::Vec2d pos(obstacle.x, obstacle.y);
        common::PathPoint path_point0;
        common::PathPoint path_point1;
        bool path_point0_exist = false;
        bool path_point1_exist = false;
        path_point0.Clear();
        path_point1.Clear();
        if (!lane_curb_paths[0].FindProjection(
            pos, &path_point0)) {
            LOG_ERR << "[Error]Failed to find projecton on major reference line.";
        }
        else{
            path_point0_exist = true;
        }
        if (lane_curb_list.lane_mark_num > 1){
            if (!lane_curb_paths[1].FindProjection(
                    pos, &path_point1)) {
                    LOG_ERR << "[Error]Failed to find projecton on major reference line.";
            }
            else{
                path_point1_exist = true;
            }
        }

        LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "index:" << i << " path_point0_exist:" << path_point0_exist << " path_point1_exist:" << path_point1_exist;
        LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "lane_curb_list.lane_marks[0].id:" << lane_curb_list.lane_marks[0].id  << " lane_curb_list.lane_marks[1].id:" << lane_curb_list.lane_marks[1].id ;
        LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "path_point0.l:" << path_point0.l  << " path_point1.l:" << path_point1.l;
        LOG_INFO(CAMERA_PRE_LOG_LEVEL) << "obstacle.width:" << obstacle.width;

        if(path_point0_exist){ 
            bool is_left_lane = lane_curb_list.lane_marks[0].id > 0;
            if (is_left_lane && (path_point0.l - 0.5*obstacle.width) > -0.1){
                /**
                 * @brief 表明障碍物右边框贴近左边路沿
                 */
                invalid_ind_vec.push_back(i);
                continue;

            }
            else if (!is_left_lane && (path_point0.l + 0.5*obstacle.width) < 0.1){
                /**
                 * @brief 表明障碍物右边框贴近右边路沿
                 */
                invalid_ind_vec.push_back(i);
                continue;
            }
        }

        if(path_point1_exist){ 
            bool is_left_lane = lane_curb_list.lane_marks[1].id > 0;
            if (is_left_lane && (path_point1.l - 0.5*obstacle.width) > -0.1){
                /**
                 * @brief 表明障碍物右边框贴近左边路沿
                 */
                invalid_ind_vec.push_back(i);
            }
            else if (!is_left_lane && (path_point1.l + 0.5*obstacle.width) < 0.1){
                /**
                 * @brief 表明障碍物右边框贴近右边路沿
                 */
                invalid_ind_vec.push_back(i);

            }
        }
    }
    
    /**
     * @brief 去掉无效的数据
     */
    size_t last = camera_list.obstacle_num;
    for(int i = 0; i < (int)invalid_ind_vec.size(); i++)
    {
        size_t first = invalid_ind_vec[i];
        size_t second = last;   
        if (i+1 < (int)invalid_ind_vec.size())
        {
            second = invalid_ind_vec[i+1];
        }
        for(int j = first+1; j < (int)second; j++)
        {
            camera_list.obstacles[j-1-i] = camera_list.obstacles[j];
        }
    }
    camera_list.obstacle_num = camera_list.obstacle_num - invalid_ind_vec.size();

}


/**
 * @brief 检测前视一体机数据是否有异常，并记录状态，不改变原有数据。
 * 
 */
void MainCameraHandler::CheckCameraDataStatus(ad_msg::ObstacleCameraList& camera_list)
{
    // 获取感知数据
    framework::SharedData* shared_data = framework::SharedData::instance();
    // 获取感知模块状态值
    shared_data->GetPerceptionModuleStatus(&perception_module_status_);

    for (int i = 0; i < camera_list.obstacle_num; i++)
    {
        ad_msg::ObstacleCamera & obstacle = camera_list.obstacles[i];
        if(obstacle.x< -10|| 
           obstacle.y< -200||
           obstacle.y> 200){ 
           perception_module_status_.main_camera_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }
        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.main_camera_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }
    }
    shared_data->SetPerceptionModuleStatus(perception_module_status_);
}


void MainCameraHandler::UpdateMainCameraObjectSpeed(ad_msg::ObstacleCameraList& camera_list, ad_msg::Chassis chassis) {
    // 判断摄像头障碍物列表是否为空，如果为空则直接返回
    if (camera_list.obstacle_num == 0) {
        return;
    }

    // 判断上一帧的障碍物列表是否为空
    if (last_camera_obstacle_vec_.empty()) {
        // 如果为空则将当前帧的障碍物列表中的障碍物信息添加到上一帧的障碍物列表中，并返回
        last_camera_obstacle_vec_.reserve(camera_list.obstacle_num);
        for (Int32_t i = 0; i < camera_list.obstacle_num; i++) {
            const ad_msg::ObstacleCamera& curr_obj_ = camera_list.obstacles[i];
            if (curr_obj_.type == 10) {
                continue;
            }
            ObstacleCameraSpeedList obstacle_cam_speed_list;
            obstacle_cam_speed_list.timestamp = camera_list.msg_head.timestamp;
            obstacle_cam_speed_list.chassis_v = chassis.v;
            obstacle_cam_speed_list.camera_obstacle = curr_obj_;
            last_camera_obstacle_vec_.push_back(obstacle_cam_speed_list);
        }
        return;
    }

#if 0
    // 上一帧数据
    printf("--------Last Obstacle Camera------------\n");
    for(auto obstacle_camera_speed : last_camera_obstacle_vec_) {
        printf("id:%d, x:%f, v_x:%f, age:%d, type:%d\n", obstacle_camera_speed.camera_obstacle.id, obstacle_camera_speed.camera_obstacle.x, obstacle_camera_speed.camera_obstacle.v_x, obstacle_camera_speed.camera_obstacle.age, obstacle_camera_speed.camera_obstacle.type);
    }
    printf("----------------------------------------\n");

    // 当前帧数据
    printf("--------Current Obstacle Camera------------\n");
    for (int i = 0; i < camera_list.obstacle_num; i++) {
        ad_msg::ObstacleCamera& curr_obj = camera_list.obstacles[i];
        if (curr_obj.type != 10) {
            printf("id:%d, x:%f, v_x:%f, age:%d, type:%d\n", curr_obj.id, curr_obj.x, curr_obj.v_x, curr_obj.age, curr_obj.type);
        }
    }
    printf("----------------------------------------\n");
#endif

    // 将上一帧容器中的数据填入到map容器中
    std::unordered_map<int, size_t> obstacle_map;
    for (size_t i = 0; i < last_camera_obstacle_vec_.size(); i++) {
        obstacle_map[last_camera_obstacle_vec_[i].camera_obstacle.id] = i;
    }

    // 遍历当前帧的障碍物列表
    for (Int32_t i = 0; i < camera_list.obstacle_num; i++) {
        ad_msg::ObstacleCamera& curr_obj_ = camera_list.obstacles[i];
        // 过滤锥形桶数据
        if (curr_obj_.type == 10) {
            continue;
        }
        // 对于每一个障碍物，检查其在上一帧的障碍物列表中是否存在
        auto it = obstacle_map.find(curr_obj_.id);
        // 如果不存在，则将其添加到上一帧的障碍物列表中，并更新障碍物的时间戳、底盘速度和障碍物信息
        if (it == obstacle_map.end()) {
            ObstacleCameraSpeedList obstacle_cam_speed_list;
            obstacle_cam_speed_list.timestamp = camera_list.msg_head.timestamp;
            obstacle_cam_speed_list.chassis_v = chassis.v;
            obstacle_cam_speed_list.camera_obstacle = curr_obj_;
            last_camera_obstacle_vec_.push_back(obstacle_cam_speed_list);

            // 上一帧容器中有新增数据，所以同步更新map容器
            size_t last_num = last_camera_obstacle_vec_.size();
            obstacle_map[last_camera_obstacle_vec_[last_num - 1].camera_obstacle.id] = last_num - 1;
        } else {
            ObstacleCameraSpeedList& obstacle_cam_speed_list = last_camera_obstacle_vec_[it->second];
            // 如果存在，则比较当前帧的障碍物的age和上一帧的障碍物的age，如果当前帧的障碍物的age大于等于上一帧的障碍物的age，则计算障碍物的位置变化和速度，并更新障碍物的时间戳、底盘速度和障碍物信息
            if (curr_obj_.age >= obstacle_cam_speed_list.camera_obstacle.age) {
                // 时间戳差值
                Int64_t timestamp_diff = camera_list.msg_head.timestamp - obstacle_cam_speed_list.timestamp;
                if (timestamp_diff != 0) {
                    // 前视一体机在两帧之间移动的距离
                    Float32_t x_diff = curr_obj_.x + obstacle_cam_speed_list.chassis_v * timestamp_diff * 0.001 - obstacle_cam_speed_list.camera_obstacle.x;
                    // 根据前视一体机在两帧之间移动的距离和时间差计算速度
                    Float32_t v_x = x_diff / (timestamp_diff * 0.001);
                    // 如果计算出来的差值较大，则用计算出来的值
                    if (fabs(v_x - curr_obj_.v_x) > 7.5F && curr_obj_.v_x < 6.0F) {
                        // printf("last:id:%d, x:%f, v_x:%f\n", obstacle_cam_speed_list.camera_obstacle.id, obstacle_cam_speed_list.camera_obstacle.x, obstacle_cam_speed_list.camera_obstacle.v_x);
                        // printf("curr:id:%d, x:%f, v_x:%f\n", curr_obj_.id, curr_obj_.x, curr_obj_.v_x);
                        // printf("time_diff:%d, chass_v:%f, chass_x:%f, x_diff:%f, v_x:%f, v_diff:%f\n", timestamp_diff, obstacle_cam_speed_list.chassis_v, obstacle_cam_speed_list.chassis_v * timestamp_diff * 0.001, x_diff, v_x, v_x - curr_obj_.v_x);
                        curr_obj_.v_x = v_x;
                        // printf("curr_v_x:%f\n", curr_obj_.v_x);
                    }
                    obstacle_cam_speed_list.timestamp = camera_list.msg_head.timestamp;
                    obstacle_cam_speed_list.chassis_v = chassis.v;
                    obstacle_cam_speed_list.camera_obstacle = curr_obj_;
                }
            } else {
                // 如果当前帧的障碍物的age小于上一帧的障碍物的age，则直接更新障碍物的时间戳、底盘速度和障碍物信息
                obstacle_cam_speed_list.timestamp = camera_list.msg_head.timestamp;
                obstacle_cam_speed_list.chassis_v = chassis.v;
                obstacle_cam_speed_list.camera_obstacle = curr_obj_;
            }
            
        }
    }
}

#if 0
void MainCameraHandler::UpdataMainCameraObjectSpeed(ad_msg::ObstacleCameraList& camera_list, ad_msg::Chassis chassis) {
    // 如果无前视一体机数据
    if (camera_list.obstacle_num == 0) {
        return;
    }

    // 如果前视一体机数据容器为空，将本帧数据填入容器
    if (last_camera_obstacle_vec_.empty()) {
        last_camera_obstacle_vec_.reserve(camera_list.obstacle_num);
        // 把障碍物数据写入容器
        for (Int32_t i = 0; i < camera_list.obstacle_num; i++) {
            const ad_msg::ObstacleCamera& curr_obj_ = camera_list.obstacles[i];

            // 过滤锥形桶数据
            if (curr_obj_.type == 10) {
                continue;
            }

            // 填充数据
            ObstacleCameraSpeedList obstacle_cam_speed_list;
            obstacle_cam_speed_list.timestamp = camera_list.msg_head.timestamp;
            obstacle_cam_speed_list.chassis_v = chassis.v;
            obstacle_cam_speed_list.camera_obstacle = curr_obj_;

            // 障碍物数据
            last_camera_obstacle_vec_.push_back(obstacle_cam_speed_list);
        }
        return;
    }

#if 0
    // 上一帧数据
    printf("--------Last Obstacle Camera------------\n");
    for(auto obstacle_camera_speed : last_camera_obstacle_vec_) {
        printf("id:%d, x:%f, v_x:%f, age:%d, type:%d\n", obstacle_camera_speed.camera_obstacle.id, obstacle_camera_speed.camera_obstacle.x, obstacle_camera_speed.camera_obstacle.v_x, obstacle_camera_speed.camera_obstacle.age, obstacle_camera_speed.camera_obstacle.type);
    }
    printf("----------------------------------------\n");

    // 当前帧数据
    printf("--------Current Obstacle Camera------------\n");
    for (int i = 0; i < camera_list.obstacle_num; i++) {
        ad_msg::ObstacleCamera& curr_obj = camera_list.obstacles[i];
        if (curr_obj.type != 10) {
            printf("id:%d, x:%f, v_x:%f, age:%d, type:%d\n", curr_obj.id, curr_obj.x, curr_obj.v_x, curr_obj.age, curr_obj.type);
        }
    }
    printf("----------------------------------------\n");
#endif

    // 计算速度
    size_t last_size = last_camera_obstacle_vec_.size();
    for (Int32_t i = 0; i < camera_list.obstacle_num; i++) {
        ad_msg::ObstacleCamera& curr_obj_ = camera_list.obstacles[i];

        // 过滤锥形桶数据
        if(curr_obj_.type == 10) {
            continue;
        }

        // 将当前帧数据与上一帧数据进行比较
        for (size_t j = 0; j < last_size; j++) {
            // ad_msg::ObstacleCamera& last_obj_ = it->camera_obstacle;
            ad_msg::ObstacleCamera& last_obj_ = last_camera_obstacle_vec_[j].camera_obstacle;
            Int64_t& last_time_stamp_ = last_camera_obstacle_vec_[j].timestamp;
            Float32_t& last_chassis_v_ = last_camera_obstacle_vec_[j].chassis_v;
            // 如果当前帧有与上一帧相同id的障碍物,且生命周期大于则进行速度计算
            if ((last_obj_.id == curr_obj_.id) && (curr_obj_.age >= last_obj_.age)) {
                // 时间戳差值
                // Int64_t timestamp_diff = camera_list.msg_head.timestamp - it->timestamp;
                Int64_t timestamp_diff = camera_list.msg_head.timestamp - last_time_stamp_;
                if(timestamp_diff == 0) {
                    continue;
                }

                // 前视一体机在两帧之间移动的距离
                Float32_t x_diff = curr_obj_.x + last_chassis_v_ * timestamp_diff * 0.001 - last_obj_.x;
                // 根据前视一体机在两帧之间移动的距离和时间差计算速度
                Float32_t v_x = x_diff / (timestamp_diff * 0.001);
                // 如果计算出来的差值较大，则用计算出来的值
                if (fabs(v_x - curr_obj_.v_x) > 7.5F && curr_obj_.v_x < 1.0F) {
                    printf("last:id:%d, x:%f, v_x:%f\n", last_obj_.id, last_obj_.x, last_obj_.v_x);
                    printf("curr:id:%d, x:%f, v_x:%f\n", curr_obj_.id, curr_obj_.x, curr_obj_.v_x);
                    printf("time_diff:%d, chass_v:%f, chass_x:%f, x_diff:%f, v_x:%f, v_diff:%f\n", timestamp_diff, last_chassis_v_, last_chassis_v_ * timestamp_diff * 0.001, x_diff, v_x, v_x - curr_obj_.v_x);
                    curr_obj_.v_x = v_x;
                    printf("curr_v_x:%f\n", curr_obj_.v_x);
                }

                // 更新容器数据
                last_time_stamp_ = camera_list.msg_head.timestamp;
                last_chassis_v_ = chassis.v;
                last_obj_ = curr_obj_;
            } 
            else if ((last_obj_.id == curr_obj_.id) && (curr_obj_.age < last_obj_.age)) {
                // 如果当前帧有与上一帧相同id的障碍物，且当前帧的生命周期比上一帧的生命周期小，则说明这是两个障碍物
                // 将当前帧的数据替换掉上一帧的数据
                last_time_stamp_ = camera_list.msg_head.timestamp;
                last_chassis_v_ = chassis.v;
                last_obj_ = curr_obj_;
            }
        }
    }    

    // 如果当前帧的id在上一帧数据中找不到，则说明这个id是新增的障碍物，所以把数据写入到上一帧容器中
    // 定义哈希表容器
    std::unordered_map<int, ObstacleCameraSpeedList> obstacle_map;
    // 填充哈希表容器
    for (const auto& item : last_camera_obstacle_vec_) {
        obstacle_map[item.camera_obstacle.id] = item;
    }
    // 查找并添加数据到容器
    for (Int32_t i = 0; i < camera_list.obstacle_num; i++) {
        ad_msg::ObstacleCamera& curr_obj_ = camera_list.obstacles[i]; 
        // 过滤锥形桶数据
        if(curr_obj_.type == 10) {
            continue;
        }
        // 查找相同id的障碍物
        if (obstacle_map.count(curr_obj_.id) == 0) {
            // 在哈希表中找不到相同id的数据，将改数据添加到容器中
            // 添加到上一帧数据容器中
            ObstacleCameraSpeedList obstacle_cam_speed_list;
            obstacle_cam_speed_list.timestamp = camera_list.msg_head.timestamp;
            obstacle_cam_speed_list.chassis_v = chassis.v;
            obstacle_cam_speed_list.camera_obstacle = curr_obj_;
            last_camera_obstacle_vec_.push_back(obstacle_cam_speed_list);
        }
    }
}
#endif

// 根据纵向距离拆解前视一体机数据
Uint8_t MainCameraHandler::DisMainCameraObject(ad_msg::ObstacleCameraList& camera_list, vector<Int32_t> dis_param_vector, vector<ad_msg::ObstacleCameraList>& dis_camera_obstacle_vector_) {
    // 如果无前视一体机数据
    if (camera_list.obstacle_num == 0) {
        return 2;
    }
    
    // 如果拆解参数为0
    if (!dis_param_vector.size()) {
        return 0;
    }

    // 清空拆解容器
    if (dis_camera_obstacle_vector_.size() > 0) {
        dis_camera_obstacle_vector_.clear();
    }

    // 根据拆解参数进行拆分
    for (auto dis_param : dis_param_vector) {
        // 拆解出来的前视一体机障碍物
        ad_msg::ObstacleCameraList dis_camera_list;
        dis_camera_list.msg_head = camera_list.msg_head;
        dis_camera_list.cam_type = camera_list.cam_type;
        for (Int32_t i = 0; i < camera_list.obstacle_num; i++) {
            ad_msg::ObstacleCamera& obj = camera_list.obstacles[i];
            // 判断这个障碍物是否已经被拆解了
            if (!obj.id && !obj.age && !obj.type && !obj.status && !obj.cut_in && !obj.blinker && !obj.brake_lights && !obj.lane) {
                continue;
            }

            // 如果前视一体机障碍物生命周期小于5，则丢弃这一障碍物
            if (obj.age <= 5) {
                obj.Clear();
                continue;
            }

            // 处理近距离时，大车容易跳变到自车车道的情况
            if (obj.x < 10.0F) {
                if (obj.y > 0.0F) {
                    obj.y += 0.3F;
                } else {
                    obj.y -= 0.3F;
                }
            }

            // 根据拆解参数进行拆解
            if (obj.x < dis_param) {
                dis_camera_list.obstacles[dis_camera_list.obstacle_num++] = obj;
                // printf("all:id:%d, x:%f, age:%d\n", obj.id, obj.x, obj.age);
                // printf("%d:id:%d, x:%f, age:%d\n", dis_param, dis_camera_list.obstacles[dis_camera_list.obstacle_num-1].id, dis_camera_list.obstacles[dis_camera_list.obstacle_num-1].x, dis_camera_list.obstacles[dis_camera_list.obstacle_num-1].age);
                // 清空已经被拆解的障碍物数据
                obj.Clear();                
            }
        }

        // 将拆解出来的前视一体机障碍物填入到拆解前视一体机障碍物容器中
        dis_camera_obstacle_vector_.push_back(dis_camera_list);
    }

    // std::cout << camera_list.obstacle_num << ",";
    // for (auto diss_obstacle_list : dis_camera_obstacle_vector_) {
    //     std::cout << diss_obstacle_list.obstacle_num << ",";
    // }
    // std::cout << std::endl;

    return 1;
}


} //end of namespace phoenix
} //end of namespace perception
} //end of namespace main_camera


