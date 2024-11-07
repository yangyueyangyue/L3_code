#include "radar_tracker.h"

namespace phoenix{
namespace perception{
namespace radar{

RadarTracker::RadarTracker()
{
    // filter_.reset(new KalmanFilter());
    age_ = 0;
    duration_ = 0;
    is_vehicle_itself_ = false;
    is_invalid_ = false;
    initial_timestamp_ = 0;
}

RadarTracker::~RadarTracker()
{

}


void RadarTracker::Init(ad_msg::ObstacleRadar & object,  Int64_t timestamp)
{
    // filter_->Init(object);
    filter_.Init(object);
    // cur_object_.object = object;
    object_ = object;
    cur_timestamp_ = timestamp;
    // cur_object_.timestamp = timestamp;
    initial_timestamp_ = timestamp;

}


int RadarTracker::UpdateWithMeasurement(ad_msg::ObstacleRadar & object, Int64_t timestamp)
{
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS00" << object.id;
    // if (cur_object_.object.id != object.id){
    //     return -1;
    // }

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS01";

    //timestamp 单位为毫米（ms）
    double time_diff = (timestamp - cur_timestamp_)/1000.0;  
    // double time_diff = (timestamp - cur_object_.timestamp)/1000.0;  

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS02 diff:" << time_diff;

    filter_.Predict(time_diff);

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS03";

    filter_.UpdateWithObject(object, time_diff);

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS04";

    filter_.GetState(object.x, object.y, object.v_x, object.v_y);

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS05";

    object_ = object;
    cur_timestamp_ = timestamp;

    if(time_diff > 0)
    {
        age_++;
    }

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithMeasurement POS06";

    return 0;
}


void RadarTracker::Clock(Int64_t timestamp)
{
    duration_++;

    if((duration_ - age_ > VALID_DURARION_TIMES_DIFF && timestamp - initial_timestamp_ >= GENERAL_TIME_DURARION)||
        (timestamp - initial_timestamp_ >= TIME_DURARION_UPPER))
    
    {
        is_invalid_ = true;
    }
}



/**
 * @brief 
 * 
 * @param object 
 * @return int 
 */
int RadarTracker::GetUpdatedValue(ad_msg::ObstacleRadar & object)
{
    // if (filter_ == Nullptr_t)
    // {
    //     return -1;
    // }
    filter_.GetState(object.x, object.y, object.v_x, object.v_y);

    return 0;
}


}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix




