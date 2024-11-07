#ifndef RADAR_TRACKER_H_
#define RADAR_TRACKER_H_
#include <memory>

#include "radar_kalman_filter.h"
#include "radar_kf_filter.h"
#include "radar_window_filter.h"
#include "msg_common.h"
#include "msg_obstacle.h"
#include "radar_tracker.h"
#include "ad_msg.h"
#include "pos_filter/include/pos_filter_wrapper.h"

namespace phoenix{
namespace perception{
namespace radar{

using namespace std;

#define  TIME_DURARION_UPPER    (1000*60*60*4)  // 4 小时


#define  GENERAL_TIME_DURARION  (1000*4)  //障碍物至少维持4秒以上

#define  VALID_DURARION_TIMES_DIFF  60 



/**
 * @brief 该宏用于控制是否针对挂角问题进行处理 
 */
#define   ENABLE_UPDATE_ERROR_RADAR_DATA_LIST    (1)
/*
* ENABLE_CHECK_CONTAINER_DATA_ERROR 表示是否开启侧雷达的“带挂”问题检测
*/
#define   ENABLE_CHECK_CONTAINER_DATA_ERROR      (1)

//如果LOG_INFO的输入level大于5表示不做任何打印
#define   ERROR_RADAR_DEBUG_LEVEL                (7)



enum AroundRadarType{
    LEFT_FORWORD_AROUND_RADAR_TYPE = 0,
    RIGHT_FORWORD_AROUND_RADAR_TYPE,
    LEFT_BACKWORD_AROUND_RADAR_TYPE,
    RIGHT_BACKWORD_AROUND_RADAR_TYPE,
};

/*
* 以下为ROI的各种限制框，参数通过调试确定
*/

#define     LEFT_BW_RADAR_Y_MIN_VALUE   0  //暂定为-2
#define     RIGHT_BW_RADAR_Y_MAX_VALUE  0  //暂定为3

#define     LEFT_FW_RADAR_X_MAX_VALUE   4 //暂定为2（m）
#define     RIGHT_FW_RADAR_X_MAX_VALUE  4 //暂定为2（m）


/*
以下四个常数确定为左后雷达确定一个框，落入该框的点，被确定为（毫米波雷达所在）车辆的自身
*/
#define     LEFT_BW_RADAR_CV_Y_MIN_VALUE   -1.5 //
#define     LEFT_BW_RADAR_CV_X_MIN_VALUE   -3 //
#define     LEFT_BW_RADAR_CV_Y_MAX_VALUE   1 //
#define     LEFT_BW_RADAR_CV_X_MAX_VALUE   4 //

/*
以下四个常数确定为右后雷达确定一个框，落入该框的点，被确定为（毫米波雷达所在）车辆的自身
*/
#define     RIGHT_BW_RADAR_CV_Y_MIN_VALUE   -1.5 //
#define     RIGHT_BW_RADAR_CV_X_MIN_VALUE   -3 //
#define     RIGHT_BW_RADAR_CV_Y_MAX_VALUE   1 //
#define     RIGHT_BW_RADAR_CV_X_MAX_VALUE   4 //

#define     AROUND_RADAR_CV_Y_MAX_V_VALUE   0.4  //限定检测到的车自身最大速度
#define     AROUND_RADAR_CV_Y_MIN_V_VALUE   -0.4 //限定检测到的车自身最小速度


/*
以下四个常数确定为左后雷达确定一个可疑框，落入该框的点，被认为可能是无效数据，也就是范围相比上面的框有所扩大
可以定位无效数据
*/
#define     LEFT_BW_RADAR_DOUBTABLE_INVALID_Y_MIN_VALUE   -2 //
#define     LEFT_BW_RADAR_DOUBTABLE_INVALID_X_MIN_VALUE   -10.5 //
#define     LEFT_BW_RADAR_DOUBTABLE_INVALID_Y_MAX_VALUE   -0.5 //
#define     LEFT_BW_RADAR_DOUBTABLE_INVALID_X_MAX_VALUE   2.0 //

/*
以下四个常数确定为右后雷达确定一个可疑框，落入该框的点，被认为可能是无效数据
*/
#define     RIGHT_BW_RADAR_DOUBTABLE_INVALID_Y_MIN_VALUE   -2 //
#define     RIGHT_BW_RADAR_DOUBTABLE_INVALID_X_MIN_VALUE   -10.5 //
#define     RIGHT_BW_RADAR_DOUBTABLE_INVALID_Y_MAX_VALUE   -0.5 //
#define     RIGHT_BW_RADAR_DOUBTABLE_INVALID_X_MAX_VALUE   2.0 //




/*
* 针对挂角问题需要进行计算的区域，其他区域默认不会出现挂角问题。
*/
#define   COMPUTE_ERROR_RADAR_ZONE_X_MIN    (-8)
#define   COMPUTE_ERROR_RADAR_ZONE_X_MAX    (4)
#define   COMPUTE_ERROR_RADAR_ZONE_Y_MIN    (-4)
#define   COMPUTE_ERROR_RADAR_ZONE_Y_MAX    (4)    


/*
* 针对“带挂”（货箱）问题相关的宏定义
*/
#define TRAILER_LEFT_Y_FLUCTUATION  (0.75)//(0.8)//(0.75) //带挂时，误识别障碍物与挂之间的横向距离波动值
#define TRAILER_RIGHT_Y_FLUCTUATION (0.75)//(0.8)//(0.75) //带挂时，误识别障碍物与挂之间的横向距离波动值


#define   VEHICLE_HEAD_REAR_LENGTH         (6.0) //从车前轴中心到车头尾部的距离
#define   VEHICLE_HEAD_WIDTH               (2.5) //车头宽度
#define   VEHICLE_CONTAINER_WIDTH          (2.5) //货箱的宽度
#define   VEHICLE_CONTAINER_LENGTH         (13.0)//货箱的长度

#define   ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_X_MIN    (-8)
#define   ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_X_MAX    (4)
#define   ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_Y_MIN    (-4)
#define   ERROR_RADAR_CONTAINER_BUG_LEFT_ZONE_Y_MAX    (4) 


#define   ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_X_MIN    (-8)
#define   ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_X_MAX    (4)
#define   ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_Y_MIN    (-4)
#define   ERROR_RADAR_CONTAINER_BUG_RIGHT_ZONE_Y_MAX    (4) 



class ErrorRadarData{
public:
ErrorRadarData()
{
    err_count_ = 0;
    duration_ = 0;
    age_ = 0;
    chassis_v_ = 0;
    need_remove_ = false;
    is_error_ = false;

    obj_id_  = 0;   
    obj_x_  = 0;    
    obj_y_  = 0;
    obj_v_x_  = 0;   
    obj_v_y_  = 0; 
    obj_length_  = 0;    
    obj_width_  = 0;  
}

~ErrorRadarData(){}

Int32_t GetId(){
    return obj_id_;
}

bool IsError(){
    return is_error_;
}

bool NeedToRemove(){
    return need_remove_;
}

/*
每个周期执行一次
*/
void UpdateObstacle(ad_msg::ObstacleRadar* object, Float32_t chassis_v, bool is_in_held_up_zone, bool is_in_container_zone){
    
    if (object->id != obj_id_){
        return;
    }
    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << object->id << " object->v_x:" << object->v_x << " chassis_v:" << chassis_v;
    
    is_in_held_up_zone_ = is_in_held_up_zone;
    is_in_container_zone_ = is_in_container_zone;
    
    if( (is_in_held_up_zone_ && common::com_abs(object->v_x - chassis_v) > 1.0)||
    (is_in_container_zone_  &&  common::com_abs(object->v_x - chassis_v) > 1.5)){
        need_remove_ = true;
        is_error_ = false;
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "v not match";
        obj_id_  = object->id;   
        obj_x_  = object->x;    
        obj_y_  = object->y;
        obj_v_x_  = object->v_x;   
        obj_v_y_  = object->v_y;        
        obj_length_  = object->length;    
        obj_width_  = object->width;    
        return;
    }
    
    bool ret_bool = false;
    if(is_in_held_up_zone_)
    {
        ret_bool = CheckHeldUpError(object);
    }
    /*
    * ret_bool 是 false表示不符合“挂角的检测条件”
    */
    if(!ret_bool && is_in_container_zone_)
    {
        ret_bool = CheckContainerError(object);
    }


    /*
    * 如果已经50*100ms内都没有出现可能错误的数据了，那么该ErrorRadarData就应该从相应列表中删除了。
    */
    if(duration_ - age_ > 50){
        need_remove_ = true;
    }
    else{
        need_remove_ = false;
    }

    obj_id_  = object->id;   
    obj_x_  = object->x;    
    obj_y_  = object->y;
    obj_v_x_  = object->v_x;   
    obj_v_y_  = object->v_y;        
    obj_length_  = object->length;    
    obj_width_  = object->width;    

}

bool CheckHeldUpError(ad_msg::ObstacleRadar* object)
{
    bool ret_bool = false;
    if(object->x == obj_x_ && object->y == obj_y_)
    {//针对“挂角”问题进行检测计数
        err_count_++;
        age_ = duration_;
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL)  << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " err_count:" << err_count_;
        ret_bool = true;
    }
    else{
        err_count_ = 0;
        is_error_ = false;
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL)  << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " is_error_ = false ";
    }

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " err_count_:" << err_count_ << " age_:" << age_ << " duration_:" << duration_;

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL)  << "obstacle id:" << object->id << " new(" << object->x << " ," << object->y << ") old(" << obj_x_ << " ," << obj_y_ << ")";
    
    /*
    * 当连续10次出现雷达检测的障碍物（x,y）值，毫无变化
    */
    if(err_count_ > 10){
        is_error_ = true;
    }
    if(is_error_){
        ret_bool = true;
    }
    return ret_bool;
}


bool CheckContainerError(ad_msg::ObstacleRadar* object)
{
    bool ret_bool = false;

    bool meet_conditions = false;
    bool meet_position_condition = false;
    bool meet_speed_condition = false;
    meet_position_condition = (common::com_abs(object->x - obj_x_) < 2.0) && 
                              (common::com_abs(object->y - obj_y_) < 2.0);
    meet_speed_condition = (common::com_abs(object->v_x - obj_v_x_) < 2.0) && 
                            (common::com_abs(object->v_y - obj_v_y_) < 2.0);


    meet_conditions = meet_speed_condition && meet_position_condition;

    if(meet_conditions) 
    {//针对“挂角”问题进行检测计数
        err_count_++;
        age_ = duration_;
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL)  << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " err_count:" << err_count_;
        ret_bool = true;
    }
    else{
        err_count_ = 0;
        is_error_ = false;
        //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL)  << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " is_error_ = false ";
    }

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " err_count_:" << err_count_ << " age_:" << age_ << " duration_:" << duration_;

    //LOG_INFO(ERROR_RADAR_DEBUG_LEVEL)  << "obstacle id:" << object->id << " new(" << object->x << " ," << object->y << ") old(" << obj_x_ << " ," << obj_y_ << ")";
    
    /*
    * 当连续20次(2s)出现雷达检测的障碍物（x,y）值，毫无变化
    */
    if(err_count_ > 20){
        is_error_ = true;
    }

    if(is_error_){
        ret_bool = true;
    }
    return ret_bool;
}



void Clock(){
    duration_++;

    /*
    * 如果已经50*100ms内都没有出现可能错误的数据了，那么该ErrorRadarData就应该从相应列表中删除了。
    */
    if(is_in_held_up_zone_)
    {
        if(duration_ - age_ > 50){
            need_remove_ = true;
        }
    }
    if ((!need_remove_) && is_in_container_zone_){
    //进行箱体检测操作，处理need_remove_的值


    }

}

void InitObstacle(ad_msg::ObstacleRadar* object, Float32_t chassis_v){
    obj_id_  = object->id;   
    obj_x_  = object->x;    
    obj_y_  = object->y;
    obj_v_x_  = object->v_x;   
    obj_v_y_  = object->v_y;        
    obj_length_  = object->length;    
    obj_width_  = object->width;  
    err_count_ = 0;
    duration_ = 0;
    age_ = 0;

    is_in_held_up_zone_ = false;
    is_in_container_zone_ = false;

    chassis_v_ = chassis_v;
    need_remove_ = false;
    is_error_ = false;
    
    //debug
    // if (GetId() == 147)
    // {
    //     LOG_INFO(ERROR_RADAR_DEBUG_LEVEL) << "obstacle id:" << object->id << " ptr_:" << (int64_t)this << " init:" << err_count_ << " age_:" << age_ << " duration_:" << duration_;
    // }
}

private:
public:
// ad_msg::ObstacleRadar * object_;

Int32_t obj_id_;
Float32_t obj_length_;
Float32_t obj_width_;
Float32_t obj_x_;
Float32_t obj_y_;
Float32_t obj_v_x_;
Float32_t obj_v_y_;
Int32_t err_count_;
Int32_t duration_;
Int32_t age_;//与duration_ 配合使用，临时保存duration_的值
Float32_t chassis_v_;
bool is_error_;
bool need_remove_;

bool is_in_held_up_zone_;
bool is_in_container_zone_;

};






class RadarTracker{

public:
    RadarTracker();
    ~RadarTracker();
    void Init(ad_msg::ObstacleRadar & object,  Int64_t timestamp);

    Int32_t GetObjectId(){ return object_.id;}

    int UpdateWithMeasurement(ad_msg::ObstacleRadar & object, Int64_t timestamp);

    int GetUpdatedValue(ad_msg::ObstacleRadar & object);

    void Clock(Int64_t timestamp);

    Int64_t GetTimeStamp(){return cur_timestamp_;}

    bool IsItself(){ return is_vehicle_itself_;}

    bool IsInvalid(){ return is_invalid_;}

private:

#if ENABLE_USING_FILTER_BASED_ON_EIGEN
    RadarKalmanFilter filter_;
#elif ENABLE_USING_WINDOW_SMOOTH_FILTER
    WindowSmoothFilter filter_;
#elif ENABLE_USING_ORIGINAL_SMOOTH_FILTER
    KalmanFilter filter_;
#endif

    ad_msg::ObstacleRadar  object_;
    Int64_t cur_timestamp_;
    Int32_t age_;
    Int32_t duration_;
    Int64_t initial_timestamp_;
    bool is_invalid_;
    bool is_vehicle_itself_;//表示车辆自身

};








}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix


#endif
