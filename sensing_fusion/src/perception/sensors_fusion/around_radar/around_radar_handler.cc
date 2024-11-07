#include "around_radar_handler.h"

namespace phoenix{
namespace perception{
namespace radar{

void print_obstacle(ad_msg::ObstacleRadar & obstacle, AroundRadarType radar_type)
{
    // if(obstacle.id == 0){
    //     return;
    // }
    if( LEFT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        LOG_INFO(2) << "left fw:[id:" << obstacle.id <<"] (" << obstacle.x << " ," << obstacle.y << 
                    " ," << obstacle.v_x << " ," << obstacle.v_y << ")";

    }
    else if( RIGHT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        LOG_INFO(2) << "right fw:[id:" << obstacle.id <<"] (" << obstacle.x << " ," << obstacle.y << 
            " ," << obstacle.v_x << " ," << obstacle.v_y << ")";

    }
    else if(LEFT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {
        LOG_INFO(2) << "left bw:[id:" << obstacle.id <<"] (" << obstacle.x << " ," << obstacle.y << 
                    " ," << obstacle.v_x << " ," << obstacle.v_y << ")";

    }
    else if( RIGHT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {
        LOG_INFO(2) << "right bw:[id:" << obstacle.id <<"] (" << obstacle.x << " ," << obstacle.y << 
                    " ," << obstacle.v_x << " ," << obstacle.v_y << ")";
    }



}


AroundRadarHandler::AroundRadarHandler()
{
    left_fw_radar_list_.reserve(MAX_TRACKERLIST_SIZE);
    right_fw_radar_list_.reserve(MAX_TRACKERLIST_SIZE);
    left_bw_radar_list_.reserve(MAX_TRACKERLIST_SIZE);
    right_bw_radar_list_.reserve(MAX_TRACKERLIST_SIZE);

    left_fw_err_radar_list_.reserve(MAX_ERR_RADAR_LIST_SIZE);
    right_fw_err_radar_list_.reserve(MAX_ERR_RADAR_LIST_SIZE);
    left_bw_err_radar_list_.reserve(MAX_ERR_RADAR_LIST_SIZE);
    right_bw_err_radar_list_.reserve(MAX_ERR_RADAR_LIST_SIZE);

    // LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "left_fw err_radar_list:" << (int64_t)(&left_fw_err_radar_list_);
    // LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "right_fw err_radar_list:" << (int64_t)(&right_fw_err_radar_list_);
    // LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "left_bw err_radar_list:" << (int64_t)(&left_bw_err_radar_list_);
    // LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "right_bw err_radar_list:" << (int64_t)(&right_bw_err_radar_list_);

}

AroundRadarHandler::~AroundRadarHandler()
{

}

void AroundRadarHandler::SetRelativePosList(ad_msg::RelativePosList* rel_pos_list)
{
    rel_pos_list_ = rel_pos_list;    
}

/*
*/
void AroundRadarHandler::UpdateLeftFwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_)
{
    UpdateRadarList(radar_list, LEFT_FORWORD_AROUND_RADAR_TYPE, left_fw_err_radar_list_, chassis_);   
}

void AroundRadarHandler::UpdateRightFwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_)
{
    UpdateRadarList(radar_list, RIGHT_FORWORD_AROUND_RADAR_TYPE, right_fw_err_radar_list_, chassis_);
}

void AroundRadarHandler::UpdateLeftBwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_)
{
    UpdateRadarList(radar_list, LEFT_BACKWORD_AROUND_RADAR_TYPE, left_bw_err_radar_list_, chassis_);
}

void AroundRadarHandler::UpdateRightBwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_)
{
    UpdateRadarList(radar_list, RIGHT_BACKWORD_AROUND_RADAR_TYPE, right_bw_err_radar_list_, chassis_);
}

void AroundRadarHandler::UpdateRadarList(ad_msg::ObstacleRadarList& radar_list, AroundRadarType radar_type, std::vector<ErrorRadarData>& err_radar_list, ad_msg::Chassis& chassis_)
{
    Int64_t timestamp = radar_list.msg_head.timestamp;
    std::vector<size_t> invalid_ind_vec;
    ad_msg::RelativePos rel_pos;


#if ENABLE_UPDATE_ERROR_RADAR_DATA_LIST

    bool rel_pos_ret =  phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(timestamp, *rel_pos_list_, &rel_pos);

    int i = 0;
    // for (std::vector<ErrorRadarData>::iterator it = err_radar_list.begin(); it != err_radar_list.end();it++)
    // {
    //     LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << " b11 err_radar_list:" << (int64_t)(&err_radar_list) << " idx-> "<< i++ << " ptr:" << (int64_t) (&(*it)) << " id: " << it->GetId() << " need remove:" << it->NeedToRemove() << " is error:" << it->IsError() << " err_count_:" << it->err_count_ << " duration_: " << it->duration_ << " age_: " << it->age_;
    // }

    i = 0;
    for(ErrorRadarData & err_radar : err_radar_list)
    {
        err_radar.Clock();
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << " b22 err_radar_list:" << (int64_t)(&err_radar_list) << " idx-> "<< i++ << " ptr:" << (int64_t) (&(err_radar)) << " id: " << err_radar.GetId() << " need remove:" << err_radar.NeedToRemove() << " is error:" << err_radar.IsError() << " err_count_:" << err_radar.err_count_ << " duration_: " << err_radar.duration_ << " age_: " << err_radar.age_;
    }

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "current handler:" << (int64_t)this  << " radar_type:" << radar_type << " err_radar_list:" << (int64_t)(&err_radar_list) << " rel_pos_ret : " << rel_pos_ret << " radar_list size:" <<  radar_list.obstacle_num;

#endif 

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateRadarList POS00 size: " << static_cast<Int32_t>(invalid_ind_vec.size()); 

    tmp_count_ = 0;

    for(int i = 0; i < radar_list.obstacle_num; i++)
    {
        ad_msg::ObstacleRadar & obstacle =  radar_list.obstacles[i];  

        CheckRadarDataStatus(obstacle, radar_type);

#if ENABLE_AROUND_RADAR_COORDINATE_BIAS
        CorrectCoordinate(obstacle, radar_type);
#endif

        // LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateRadarList POS01 :";
        //print_obstacle(obstacle, radar_type);//print obstacle info

        if (!IsRadarDataValid(obstacle, radar_type, chassis_))
        {
            LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateRadarList POS02";
            invalid_ind_vec.push_back(i);
            continue;//无效数据舍弃
        }

#if ENABLE_UPDATE_ERROR_RADAR_DATA_LIST
        if (rel_pos_ret){
            UpdateErrRadarDataList(obstacle, invalid_ind_vec, err_radar_list, rel_pos, i);
        }
#endif

        // LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateRadarList POS03";
        /*
        通过ID完成匹配，关于匹配暂时使用ID匹配，其他匹配或者聚类算法暂时不考虑
        */
#if ENABLE_AROUND_RADAR_PRE_KF_FILTER  
        if(LEFT_FORWORD_AROUND_RADAR_TYPE == radar_type)
        {
            AssociateAndFilter(obstacle, left_fw_idx2id_, left_fw_id2idx_, left_fw_radar_list_, radar_type, timestamp);
        }
        else if(RIGHT_FORWORD_AROUND_RADAR_TYPE == radar_type)
        {
            AssociateAndFilter(obstacle, right_fw_idx2id_, right_fw_id2idx_, right_fw_radar_list_, radar_type, timestamp);
        }
        else if(LEFT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
        {
            AssociateAndFilter(obstacle, left_bw_idx2id_, left_bw_id2idx_, left_bw_radar_list_, radar_type, timestamp);
        }
        else
        {
            AssociateAndFilter(obstacle, right_bw_idx2id_, right_bw_id2idx_, right_bw_radar_list_, radar_type, timestamp);
        }
#endif


    }

    // LOG_INFO(2) << "mark --------------------------------------------------------------------------------------------------------------------------";

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "UpdateRadarList POS10 invalid_ind_vec size: " << static_cast<Int32_t>(invalid_ind_vec.size()); 
    
    /*
    去掉无效的数据
    */
    size_t last = radar_list.obstacle_num;
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
            radar_list.obstacles[j-1-i] = radar_list.obstacles[j];
        }
    }
    radar_list.obstacle_num = radar_list.obstacle_num - invalid_ind_vec.size();



/**
 * @brief 以下代码用来打印信息，不参与任何逻辑运算
 */
#if 0
    for(int i = 0; i < radar_list.obstacle_num; i++)
    {
        ad_msg::ObstacleRadar & obstacle = radar_list.obstacles[i];
        if (RIGHT_BACKWORD_AROUND_RADAR_TYPE == radar_type){
            //LOG_INFO(RADAR_PRE_LOG_LEVEL) << "radar obstacle.y:" << obstacle.y << " obstacle.x:" << obstacle.x;
        }

        if (LEFT_FORWORD_AROUND_RADAR_TYPE == radar_type && obstacle.id == 147){
            //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "id->>" << obstacle.id << " obstacle.y:" << obstacle.y << " obstacle.x:" << obstacle.x;
        }
    }
#endif

}

/*
* 针对挂角问题和带挂检测到货箱的问题做特殊处理
*/
void AroundRadarHandler::UpdateErrRadarDataList(ad_msg::ObstacleRadar & obstacle, std::vector<size_t> & invalid_ind_vec, std::vector<ErrorRadarData>& err_radar_list, ad_msg::RelativePos & rel_pos,int obj_idx)
{

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id: " << obstacle.id << " position(" << obstacle.x << " ," << obstacle.y << ") v:( " << obstacle.v_x << " ," << obstacle.v_y << ")";
    /*
    *is_in_held_up_zone 表示是否在可能出现“挂角”错误的区域
    */
    bool is_in_held_up_zone = (obstacle.x <= COMPUTE_ERROR_RADAR_ZONE_X_MAX && 
                              obstacle.x >= COMPUTE_ERROR_RADAR_ZONE_X_MIN && 
                              obstacle.y <= COMPUTE_ERROR_RADAR_ZONE_Y_MAX && 
                              obstacle.y >= COMPUTE_ERROR_RADAR_ZONE_Y_MIN );

    bool is_in_container_zone;
    #if 0
    bool is_in_container_zone = (obstacle.x <= ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_X_MAX && 
                                obstacle.x >= ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_X_MIN && 
                                obstacle.y <= ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_Y_MAX && 
                                obstacle.y >= ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_Y_MIN )||
                                (obstacle.x <= ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_X_MAX && 
                                obstacle.x >= ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_X_MIN && 
                                obstacle.y <= ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_Y_MAX && 
                                obstacle.y >= ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_Y_MIN );
    #endif
    /**
     * @brief 针对带挂出现“鬼影”的问题，这里的处理暂时进行屏蔽也就是，将is_in_container_zone设置为false
     * 经过感知组的讨论暂时使用直接确定无效区域的办法去掉“鬼影”。处理方法在IsRadarDataValid中进行添加，
     * 此处就没有必要进行进一步处理了。
     */
    is_in_container_zone = false;
    if (is_in_held_up_zone == false && is_in_container_zone == false)
    {
        //表示障碍物不在错误区域，不必处理。
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << obstacle.id << " not need compute";
        return;
    }

    bool find_data = false;
    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << obstacle.id<< " err_radar_list size:" << err_radar_list.size() << " err_radar_list:" << (int64_t)(&err_radar_list) ;
    
    int i = 0;
    // for (std::vector<ErrorRadarData>::iterator it = err_radar_list.begin(); it != err_radar_list.end();it++)
    // {
    //     LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << " before err_radar_list:" << (int64_t)(&err_radar_list) << " idx-> "<< i++ << " ptr:" << (int64_t) (&(*it)) << " id: " << it->GetId() << " need remove:" << it->NeedToRemove() << " is error:" << it->IsError() << " err_count_:" << it->err_count_ << " duration_: " << it->duration_ << " age_: " << it->age_;
    // }

    for(std::vector<ErrorRadarData>::iterator it = err_radar_list.begin(); it != err_radar_list.end(); it++)
    {
        if(it->GetId() != obstacle.id){
            //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "id " << obstacle.id << " not matich";
            continue;
        }
        else{
            find_data = true;
            if(it->IsError())
            {
                invalid_ind_vec.push_back(obj_idx);
                //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << obstacle.id << " data is error !!";
            }
            it->UpdateObstacle(&obstacle, rel_pos.v, is_in_held_up_zone, is_in_container_zone);
            break;
        }
    }    

    if(find_data != true ){
        ErrorRadarData err_data;
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "find_data:" << find_data << " obstacle.id:" << obstacle.id;
        err_data.InitObstacle(&obstacle, rel_pos.v);
        err_radar_list.push_back(err_data);
    }

    i = 0;
    // for (std::vector<ErrorRadarData>::iterator it = err_radar_list.begin(); it != err_radar_list.end();it++)
    // {
    //     LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "after err_radar_list:" << (int64_t)(&err_radar_list) << " idx-> "<< i++ << " ptr:" << (int64_t) (&(*it)) << " id: " << it->GetId() << " need remove:" << it->NeedToRemove() << " is error:" << it->IsError() << " err_count_:" << it->err_count_ << " duration_: " << it->duration_ << " age_: " << it->age_;
    // }

    /*
    * 删除需要删除过期的无用的数据
    */
    bool need_clear = true;
    for (std::vector<ErrorRadarData>::iterator it = err_radar_list.begin(); it != err_radar_list.end();it++)
    {
        if(!it->NeedToRemove()){
            need_clear = false;
        }
    }
    
    if(need_clear == true && (!err_radar_list.empty()))
    {
        // LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "clear err_radar_list";
        err_radar_list.clear();
    }
}


void AroundRadarHandler::CorrectCoordinate(ad_msg::ObstacleRadar & obstacle, AroundRadarType radar_type)
{

    if( LEFT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        obstacle.x  = obstacle.x + OBSTACLE_LEFT_FW_RADAR_X_BIAS_VALUE;
        obstacle.y  = obstacle.y + OBSTACLE_LEFT_FW_RADAR_Y_BIAS_VALUE;
    }
    else if( RIGHT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        obstacle.x  = obstacle.x + OBSTACLE_RIGHT_FW_RADAR_X_BIAS_VALUE;
        obstacle.y  = obstacle.y + OBSTACLE_RIGHT_FW_RADAR_Y_BIAS_VALUE;

    }
    else if(LEFT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {
        obstacle.x  = obstacle.x + OBSTACLE_LEFT_BW_RADAR_X_BIAS_VALUE;
        obstacle.y  = obstacle.y + OBSTACLE_LEFT_BW_RADAR_Y_BIAS_VALUE;

    }
    else if( RIGHT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {
        obstacle.x  = obstacle.x + OBSTACLE_RIGHT_BW_RADAR_X_BIAS_VALUE;
        obstacle.y  = obstacle.y + OBSTACLE_RIGHT_BW_RADAR_Y_BIAS_VALUE;
    }
}




/**
 * @brief 此方法暂时不启用，存在时序问题的，后面优化的时候可能会用到，暂时不实现
 * 
 * @param radar_list 
 * @param tracker_list 
 */
void AroundRadarHandler::CollectValidObjects(ad_msg::ObstacleRadarList& radar_list, std::vector<RadarTracker> & tracker_list)
{

}


/**
 * @brief 每一次来数据作时为一个周期，对于需要周期执行的任务放在Clock()中
 * 
 *    
 * @param timestamp 当前时间戳
 */
void AroundRadarHandler::Clock(Int64_t timestamp)
{
    HandlePeriodicTask(left_fw_idx2id_, left_fw_id2idx_, left_fw_radar_list_, LEFT_FORWORD_AROUND_RADAR_TYPE, timestamp);
    HandlePeriodicTask(right_fw_idx2id_, right_fw_id2idx_, right_fw_radar_list_, RIGHT_FORWORD_AROUND_RADAR_TYPE, timestamp);
    HandlePeriodicTask(left_bw_idx2id_, left_bw_id2idx_, left_bw_radar_list_, LEFT_BACKWORD_AROUND_RADAR_TYPE, timestamp);
    HandlePeriodicTask(right_bw_idx2id_, right_bw_id2idx_, right_bw_radar_list_, RIGHT_BACKWORD_AROUND_RADAR_TYPE, timestamp);
}
/**
 * @brief 执行周期执行的任务，处理各种列表是否过期，根据数据有效性进行更新   
 * 
 * 
 * @param idx2id_map 
 * @param id2idx_map 
 * @param tracker_list 
 * @param radar_type 
 * @param timestamp 
 */
void AroundRadarHandler::HandlePeriodicTask(std::map<size_t, Int32_t> & idx2id_map ,std::map<Int32_t, size_t> & id2idx_map , 
          std::vector<RadarTracker> & tracker_list, AroundRadarType radar_type, Int64_t timestamp)
{
    bool have_invalid_data = false;    
    /*
    * 运行心跳Clock，清除无效tracker
    */
    std::vector<RadarTracker>::iterator iter;
    for(iter = tracker_list.begin(); iter != tracker_list.end(); )
    {
        iter->Clock(timestamp);

        if(iter->IsInvalid())
        {
            iter = tracker_list.erase(iter);
            have_invalid_data = true;
        }
        else
        {
            iter++;
        }
    }

    /*
    * 更新 idx2id_map 和 id2idx_map    
    */
    if(have_invalid_data)
    {
        idx2id_map.clear();
        id2idx_map.clear();

        for(size_t i = 0; i < tracker_list.size(); i++)
        {
            idx2id_map[i] = tracker_list[i].GetObjectId();
            id2idx_map[tracker_list[i].GetObjectId()] = i;
        }

    }

}





/**
 * @brief 判断数据的有效性，检测出数据明显不对的无效数据。
 * 具体的会滤掉左后雷达检测到的最右边的数据，滤掉右后雷达检测到的最左边的数据，滤掉左前雷达以及右前雷达太靠前的数据；
 * 滤掉明显可确定是车辆自身的数据，非明显但是经过一段比较长的时间可以确定是车辆自身的数据会在后续流程中加入，带挂的情况
 * 后面在再做考虑（time:20220907）。
 * 
 * 需要注意的一点：坐标y轴指向左边，向左为大
 * 
 * @param obstacle 
 * @param radar_type 
 * @return true 表示有效
 * @return false 表示无效
 */
bool AroundRadarHandler::IsRadarDataValid(ad_msg::ObstacleRadar & obstacle, AroundRadarType radar_type, ad_msg::Chassis& chassis_)
{
    if( LEFT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        LOG_INFO(RADAR_PRE_LOG_LEVEL) << "left fw obstacle.y:" << obstacle.y << " x:" << obstacle.x << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        if (obstacle.x > LEFT_FW_RADAR_X_MAX_VALUE)
        {   //滤掉左前雷达太靠前的数据
            return false;
        }

    }
    else if( RIGHT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        LOG_INFO(RADAR_PRE_LOG_LEVEL) << "right fw obstacle.y:" << obstacle.y << " x:" << obstacle.x << 
            " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;
        if (obstacle.x >  RIGHT_FW_RADAR_X_MAX_VALUE)
        {   //滤掉右前雷达太靠前的数据
            return false;
        }
    }
    else if(LEFT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {
        // LOG_INFO(RADAR_PRE_LOG_LEVEL) << "left bw obstacle.y:" << obstacle.y << " obstacle.x:" << obstacle.x;

        if (obstacle.y < LEFT_BW_RADAR_Y_MIN_VALUE)
        {   //滤掉左后雷达检测到的最右边的数据
            return false;
        }

        if (obstacle.y > LEFT_BW_RADAR_CV_Y_MIN_VALUE && 
            obstacle.y < LEFT_BW_RADAR_CV_Y_MAX_VALUE &&
            obstacle.x > LEFT_BW_RADAR_CV_X_MIN_VALUE &&
            obstacle.x < LEFT_BW_RADAR_CV_X_MAX_VALUE
            )
        {   //障碍物为车辆自身
            return false;
        }

        //去掉因带挂而产生的误识别障碍物
        if (1 == chassis_.trailer_status){
            if (obstacle.y > 0.0F && 
                obstacle.y < ((chassis_.trailer_w/2) + TRAILER_LEFT_Y_FLUCTUATION) &&
                obstacle.x > -(chassis_.trailer_l + VEHICLE_HEAD_REAR_LENGTH) &&
                obstacle.x < 0.0F)
            {
                return false;
            }
        }

        if (obstacle.y > LEFT_BW_RADAR_DOUBTABLE_INVALID_Y_MIN_VALUE && 
            obstacle.y < LEFT_BW_RADAR_DOUBTABLE_INVALID_Y_MAX_VALUE &&
            obstacle.x > LEFT_BW_RADAR_DOUBTABLE_INVALID_X_MIN_VALUE &&
            obstacle.x < LEFT_BW_RADAR_DOUBTABLE_INVALID_X_MAX_VALUE
            )
        {   //落入可疑无效区域的障碍物滤除
            return false;
        }

    }
    else if( RIGHT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {   
        LOG_INFO(RADAR_PRE_LOG_LEVEL) << "right bw obstacle.y:" << obstacle.y << " x:" << obstacle.x << 
                    " v_x:" << obstacle.v_x << " v_y:" << obstacle.v_y;

        if (obstacle.y > RIGHT_BW_RADAR_Y_MAX_VALUE)
        {   //滤掉右后雷达检测到的最左边的数据
            return false;
        }

        if (obstacle.y > RIGHT_BW_RADAR_CV_Y_MIN_VALUE && 
            obstacle.y < RIGHT_BW_RADAR_CV_Y_MAX_VALUE &&
            obstacle.x > RIGHT_BW_RADAR_CV_X_MIN_VALUE &&
            obstacle.x < RIGHT_BW_RADAR_CV_X_MAX_VALUE 
            )
        {   //障碍物为车辆自身
            return false;
        }

        //去掉因带挂而产生的误识别障碍物
        if (1 == chassis_.trailer_status){
            
            if (obstacle.y > -((chassis_.trailer_w/2) + TRAILER_RIGHT_Y_FLUCTUATION) && 
                obstacle.y < 0.0F &&
                obstacle.x > -(chassis_.trailer_l + VEHICLE_HEAD_REAR_LENGTH) &&
                obstacle.x < 0.0F)
            {
                return false;
            }
        }

        if (obstacle.y > RIGHT_BW_RADAR_DOUBTABLE_INVALID_Y_MIN_VALUE && 
            obstacle.y < RIGHT_BW_RADAR_DOUBTABLE_INVALID_Y_MAX_VALUE &&
            obstacle.x > RIGHT_BW_RADAR_DOUBTABLE_INVALID_X_MIN_VALUE &&
            obstacle.x < RIGHT_BW_RADAR_DOUBTABLE_INVALID_X_MAX_VALUE
            )
        {   //落入可疑无效区域的障碍物滤除
            return false;
        }

    }

    return true;
}


void AroundRadarHandler::AssociateAndFilter(ad_msg::ObstacleRadar & obstacle, std::map<size_t, Int32_t> & idx2id_map ,std::map<Int32_t, size_t> & id2idx_map , 
          std::vector<RadarTracker> & tracker_list, AroundRadarType radar_type, Int64_t timestamp)
{
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "AssociateAndFilter POS00";
    std::map<Int32_t, size_t>::iterator it = id2idx_map.find(obstacle.id);
    if(it != id2idx_map.end())
    {   //找到数据
        LOG_INFO(RADAR_PRE_LOG_LEVEL) << "AssociateAndFilter POS01";
        RadarTracker & tracker = tracker_list[it->second];
        tracker.UpdateWithMeasurement(obstacle, timestamp);//更新object内部数据
    }
    else
    {   //没有相应的tracker，为此创建新的tracker
        LOG_INFO(RADAR_PRE_LOG_LEVEL) << "AssociateAndFilter POS02";
        RadarTracker  tracker;
        // tracker.reset(new RadarTracker());
        // tracker->Init(obstacle, timestamp);//对齐进行初始化

        tracker_list.emplace_back(tracker);
        tracker_list.back().Init(obstacle, timestamp);//对齐进行初始化

        id2idx_map[obstacle.id] = tracker_list.size() - 1;
        idx2id_map[tracker_list.size() - 1] = obstacle.id;
    }
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "AssociateAndFilter POS03";

}


/**
 * @brief 检测侧雷达数据是否有异常，并记录状态，不改变原有数据。
 * 
 * @param obstacle 
 * @param radar_type 
 */
void AroundRadarHandler::CheckRadarDataStatus(ad_msg::ObstacleRadar & obstacle, AroundRadarType radar_type)
{
    // 获取感知数据
    framework::SharedData* shared_data = framework::SharedData::instance();
    // 获取感知模块状态值
    shared_data->GetPerceptionModuleStatus(&perception_module_status_);

    if( LEFT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        if(obstacle.x < -50|| 
           obstacle.x > 100||
           obstacle.y > 20)
        { 
           perception_module_status_.left_fw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.left_fw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }

    }
    else if( RIGHT_FORWORD_AROUND_RADAR_TYPE == radar_type) 
    {
        if(obstacle.x < -50|| 
           obstacle.x > 100||
           obstacle.y < -20)
        { 
           perception_module_status_.right_fw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60 ){ 
           perception_module_status_.right_fw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }

    }
    else if(LEFT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {
        if(obstacle.x < -130|| 
           obstacle.x > 10||
           obstacle.y > 20)
        { 
           perception_module_status_.left_bw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.left_bw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }

    }
    else if( RIGHT_BACKWORD_AROUND_RADAR_TYPE == radar_type)
    {   
        if(obstacle.x < -130|| 
           obstacle.x > 10||
           obstacle.y < -20)
        { 
           perception_module_status_.right_bw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }

        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60 ){ 
           perception_module_status_.right_bw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }
    }

    shared_data->SetPerceptionModuleStatus(perception_module_status_);
}


/**
 * @brief 直接将侧雷达预处理的结果集成到融合结果中
 * 
 * @param left_fw_radar_list 
 * @param right_fw_radar_list 
 * @param left_bw_radar_list 
 * @param right_bw_radar_list 
 * @param objs_sensors_fusion_res 
 */
void AroundRadarHandler::CombineAroundRadarToFusedListDirectly(ad_msg::ObstacleRadarList& left_fw_radar_list,
                                                            ad_msg::ObstacleRadarList& right_fw_radar_list,
                                                            ad_msg::ObstacleRadarList& left_bw_radar_list,
                                                            ad_msg::ObstacleRadarList& right_bw_radar_list,
                                                            ad_msg::ObstacleList & objs_sensors_fusion_res
                                                            )
{
    CombineAroundRadarListToFusedList(left_fw_radar_list, objs_sensors_fusion_res);
    CombineAroundRadarListToFusedList(right_fw_radar_list, objs_sensors_fusion_res);
    CombineAroundRadarListToFusedList(left_bw_radar_list, objs_sensors_fusion_res);
    CombineAroundRadarListToFusedList(right_bw_radar_list, objs_sensors_fusion_res);
}


void AroundRadarHandler::CombineAroundRadarListToFusedList(ad_msg::ObstacleRadarList& around_radar_list,
                                       ad_msg::ObstacleList & objs_sensors_fusion_res)
{
    Int32_t dst_obj_index = objs_sensors_fusion_res.obstacle_num;

    for(int i = 0; i < around_radar_list.obstacle_num; i++)
    {
        ad_msg::ObstacleRadar& radar_obj = around_radar_list.obstacles[i];
        if (dst_obj_index >= ad_msg::ObstacleList::MAX_OBSTACLE_NUM) {
            LOG_ERR << "[Error]Can't add obj to list, storage is full.";
            break;
        }      
        ad_msg::Obstacle& dst_obj = objs_sensors_fusion_res.obstacles[dst_obj_index];
        dst_obj.Clear();
        dst_obj_index++;
        dst_obj.x = radar_obj.x;
        dst_obj.y = radar_obj.y;
        dst_obj.v_x = radar_obj.v_x;
        dst_obj.v_y = radar_obj.v_y;
        dst_obj.a_x = radar_obj.accel_x;
        dst_obj.a_y = radar_obj.accel_y;

        dst_obj.v = common::com_abs(dst_obj.v_x);
        if (common::com_abs(dst_obj.v_x) < 15.0F/3.6F) 
        {
            if (dst_obj.v_x < 2.0F/3.6F) 
            {
                dst_obj.v = 0.0F;
            } 
            else 
            {
                dst_obj.v = dst_obj.v_x;
            }
            dst_obj.dynamic = false;
        } else {
            dst_obj.dynamic = true;
        }

        Float32_t half_width = 0.5F*radar_obj.width;
        Float32_t half_length = 0.5F*radar_obj.length;

        dst_obj.obb.x = dst_obj.x + half_length;
        dst_obj.obb.y = dst_obj.y;
        dst_obj.obb.half_width = half_width;
        dst_obj.obb.half_length = half_length;

        dst_obj.perception_type = ad_msg::OBJ_PRCP_TYPE_RADAR;
        dst_obj.type = radar_obj.type;
        dst_obj.proj_on_major_ref_line.valid = 0;
        dst_obj.confidence = 90;//默认值暂时设置为90
    }
    if (dst_obj_index > around_radar_list.obstacle_num)
    {
        objs_sensors_fusion_res.obstacle_num = dst_obj_index;
        objs_sensors_fusion_res.msg_head = around_radar_list.msg_head;
    }
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "objs_sensors_fusion_res num" << objs_sensors_fusion_res.obstacle_num;
}




}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix




