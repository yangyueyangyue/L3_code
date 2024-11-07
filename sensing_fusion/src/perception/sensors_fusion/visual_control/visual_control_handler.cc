#include "visual_control_handler.h"
#include "framework/communication/shared_data.h"

namespace phoenix{
namespace perception{
namespace visual{

VisualControlHandler::VisualControlHandler()
{

}

VisualControlHandler::~VisualControlHandler()
{

}

void VisualControlHandler::UpdateCenterFwCameraList(ad_msg::ObstacleCameraList& camera_list)
{
    UpdateCameraList(camera_list, CENTER_FW_CAMERA_TYPE);   
}

void VisualControlHandler::UpdateLeftSideCameraList(ad_msg::ObstacleCameraList& camera_list)
{
    UpdateCameraList(camera_list, LEFT_SIDE_CAMERA_TYPE);
}

void VisualControlHandler::UpdateRightSideCameraList(ad_msg::ObstacleCameraList& camera_list)
{
    UpdateCameraList(camera_list, RIGHT_SIDE_CAMERA_TYPE);
}

void VisualControlHandler::UpdateLeftBwCameraList(ad_msg::ObstacleCameraList& camera_list)
{
    UpdateCameraList(camera_list, LEFT_BW_CAMERA_TYPE);
}

void VisualControlHandler::UpdateRightBwCameraList(ad_msg::ObstacleCameraList& camera_list)
{
    UpdateCameraList(camera_list, RIGHT_BW_CAMERA_TYPE);
}


void VisualControlHandler::UpdateCameraList(ad_msg::ObstacleCameraList& camera_list, VisualControlType visual_control_type)
{
    Int64_t timestamp = camera_list.msg_head.timestamp;
    std::vector<size_t> invalid_ind_vec;

    for(int i = 0; i < camera_list.obstacle_num; i++)
    {
        ad_msg::ObstacleCamera & obstacle =  camera_list.obstacles[i];  
        
        CheckCameraDataStatus(obstacle, visual_control_type);

        if (!IsCameraDataValid(obstacle, visual_control_type))
        {
            LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "UpdateCameraList";
            invalid_ind_vec.push_back(i);
            continue;//无效数据舍弃
        }

#if SET_VISUAL_CONTROL_X_V_TO_ZERO
        obstacle.v_x = 0;
#endif

#if SET_VISUAL_CONTROL_Y_V_TO_ZERO
        obstacle.v_y = 0;
#endif

    }
    /*
    去掉无效的数据
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


#if 0
    for(int i = 0; i < camera_list.obstacle_num; i++)
    {
        ad_msg::ObstacleCamera & obstacle = camera_list.obstacles[i];
        if (LEFT_BW_CAMERA_TYPE == visual_control_type){
            LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "visual control obstacle.y:" << obstacle.y << " obstacle.x:" << obstacle.x;
        }
    }
#endif

}



/**
 * @brief 检测环视相机数据是否有异常，并记录状态，不改变原有数据。
 * 
 * @param obstacle 
 * @param radar_type 
 */
void VisualControlHandler::CheckCameraDataStatus(ad_msg::ObstacleCamera & obstacle, VisualControlType visual_control_type)
{
    // 获取感知数据
    framework::SharedData* shared_data = framework::SharedData::instance();
    // 获取感知模块状态值
    shared_data->GetPerceptionModuleStatus(&perception_module_status_);
    if( CENTER_FW_CAMERA_TYPE == visual_control_type) 
    {
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "center fw camera.x:" << obstacle.x << " y:" << obstacle.y << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        if(obstacle.x< -10|| 
           obstacle.y< -200||
           obstacle.y> 200){ 
           perception_module_status_.front_center_camera_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.front_center_camera_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }

    }
    else if( LEFT_SIDE_CAMERA_TYPE == visual_control_type) 
    {
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "left side camera obstacle.x:" << obstacle.x << " y:" << obstacle.y << 
            " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        if(obstacle.x < -50|| 
           obstacle.x > 100||
           obstacle.y > 20)
        { 
           perception_module_status_.left_side_camera_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.left_side_camera_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }
    }
    else if(RIGHT_SIDE_CAMERA_TYPE == visual_control_type)
    {
        if(obstacle.x < -50|| 
           obstacle.x > 100||
           obstacle.y < -20)
        { 
           perception_module_status_.right_side_camera_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60 ){ 
           perception_module_status_.right_side_camera_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }

    }
    else if( LEFT_BW_CAMERA_TYPE == visual_control_type)
    {   
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "left bw camera obstacle.x:" << obstacle.x << " y:" << obstacle.y << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        
        if(obstacle.x < -50|| 
           obstacle.x > 100||
           obstacle.y > 20)
        { 
           perception_module_status_.left_bw_camera_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.left_bw_camera_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }

    }else if(RIGHT_BW_CAMERA_TYPE==visual_control_type){

        if(obstacle.x < -50|| 
           obstacle.x > 100||
           obstacle.y < -20)
        { 
           perception_module_status_.right_bw_camera_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60 ){ 
           perception_module_status_.right_bw_camera_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }
    }

    shared_data->SetPerceptionModuleStatus(perception_module_status_);
}


/**
 * @brief 通过传感器不同的FOV进行数据有效性判断
 * 
 * @param obstacle 
 * @param radar_type 
 * @return true 表示有效
 * @return false 表示无效
 */
bool VisualControlHandler::IsCameraDataValid(ad_msg::ObstacleCamera & obstacle, VisualControlType visual_control_type)
{
    if( CENTER_FW_CAMERA_TYPE == visual_control_type) 
    {
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "center fw camera.x:" << obstacle.x << " y:" << obstacle.y << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        
        if(obstacle.x<CENTER_FW_CAMERA_CV_X_MIN || 
           obstacle.x>CENTER_FW_CAMERA_CV_X_MAX||
           obstacle.y<CENTER_FW_CAMERA_CV_Y_MIN||
           obstacle.y>CENTER_FW_CAMERA_CV_Y_MAX){

            return false;   
        }

    }
    else if( LEFT_SIDE_CAMERA_TYPE == visual_control_type) 
    {
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "left side camera obstacle.x:" << obstacle.x << " y:" << obstacle.y << 
            " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        if(obstacle.x<LEFT_SIDE_CAMERA_CV_X_MIN || 
           obstacle.x>LEFT_SIDE_CAMERA_CV_X_MAX||
           obstacle.y<LEFT_SIDE_CAMERA_CV_Y_MIN||
           obstacle.y>LEFT_SIDE_CAMERA_CV_Y_MAX){
           
           return false;   
        }
    }
    else if(RIGHT_SIDE_CAMERA_TYPE == visual_control_type)
    {
        if(obstacle.x<RIGHT_SIDE_CAMERA_CV_X_MIN || 
           obstacle.x>RIGHT_SIDE_CAMERA_CV_X_MAX||
           obstacle.y<RIGHT_SIDE_CAMERA_CV_Y_MIN||
           obstacle.y>RIGHT_SIDE_CAMERA_CV_Y_MAX){
           
           return false;   
        }

    }
    else if( LEFT_BW_CAMERA_TYPE == visual_control_type)
    {   
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "left bw camera obstacle.x:" << obstacle.x << " y:" << obstacle.y << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        
        if(obstacle.x<LEFT_BW_CAMERA_CV_X_MIN || 
           obstacle.x>LEFT_BW_CAMERA_CV_X_MAX||
           obstacle.y<LEFT_BW_CAMERA_CV_Y_MIN||
           obstacle.y>LEFT_BW_CAMERA_CV_Y_MAX){

            LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "remove camera obstacle.x:" << obstacle.x << " y:" << obstacle.y << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
           
            return false;   
        }

    }else if(RIGHT_BW_CAMERA_TYPE==visual_control_type){

        if(obstacle.x<RIGHT_BW_CAMERA_CV_X_MIN || 
           obstacle.x>RIGHT_BW_CAMERA_CV_X_MAX||
           obstacle.y<RIGHT_BW_CAMERA_CV_Y_MIN||
           obstacle.y>RIGHT_BW_CAMERA_CV_Y_MAX){
           return false;   
        }

    }

    return true;
}

/*
void AroundRadarHandler::AssociateAndFilter(ad_msg::ObstacleRadar & obstacle, std::map<size_t, Int32_t> & idx2id_map ,std::map<Int32_t, size_t> & id2idx_map , 
          std::vector<RadarTracker> & tracker_list, AroundRadarType radar_type, Int64_t timestamp)
{
    LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "AssociateAndFilter POS00";
    std::map<Int32_t, size_t>::iterator it = id2idx_map.find(obstacle.id);
    if(it != id2idx_map.end())
    {   //找到数据
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "AssociateAndFilter POS01";
        RadarTracker & tracker = tracker_list[it->first];
        tracker.UpdateWithMeasurement(obstacle, timestamp);//更新object内部数据
    }
    else
    {   //没有相应的tracker，为此创建新的tracker
        LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "AssociateAndFilter POS02";
        RadarTracker  tracker;
        // tracker.reset(new RadarTracker());
        // tracker->Init(obstacle, timestamp);//对齐进行初始化

        tracker_list.emplace_back(tracker);
        tracker_list.back().Init(obstacle, timestamp);//对齐进行初始化

        id2idx_map[obstacle.id] = tracker_list.size() - 1;
        idx2id_map[tracker_list.size() - 1] = obstacle.id;
    }
    LOG_INFO(VISUAL_CONTROL_PRE_LOG_LEVEL) << "AssociateAndFilter POS03";

}*/

}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix




