#include "radar_window_filter.h"


namespace phoenix{
namespace perception{
namespace radar{

WindowSmoothFilter::WindowSmoothFilter()
{   
    history_values_.reserve(RADAR_FILTER_WINDOW_WIDTH+20);

}

WindowSmoothFilter::~WindowSmoothFilter()
{

}

void WindowSmoothFilter::Init(const ad_msg::ObstacleRadar& object)
{
    history_values_.clear();
    AddObjectToList(object);
    s_vector_(0) = object.x;
    s_vector_(1) = object.y;
    s_vector_(2) = object.v_x;
    s_vector_(3) = object.v_y;

}

void WindowSmoothFilter::Init(const ad_msg::ObstacleRadar &object, Matrix4d& p_in,
        Matrix4d& q_in, Matrix4d& r_in, Matrix4d& f_in, Matrix4d& h_in)
{
    Init(object);
}

bool WindowSmoothFilter::Predict(double time_diff)
{
    return true;
}

bool WindowSmoothFilter::UpdateWithObject(const ad_msg::ObstacleRadar& new_object,
                                double time_diff)
{
    AddObjectToList(new_object);
    CalcXValue(s_vector_(0));
    CalcYValue(s_vector_(1));
    CalcVXValue(s_vector_(2));
    CalcVYValue(s_vector_(3));
    return true;
}

void WindowSmoothFilter::GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y)
{
    x = s_vector_(0);
    y = s_vector_(1);
    v_x = s_vector_(2);
    v_y = s_vector_(3);
}

bool WindowSmoothFilter::CalcXValue(Float64_t & x)
{
    int size = history_values_.size();
    int start_index = 0;
    int count = size;
    if(history_values_.size() < RADAR_FILTER_X_WINDOW_WIDTH)
    {
        start_index = 0;
        count = size;
    }
    else{
        start_index = size - RADAR_FILTER_X_WINDOW_WIDTH;
        count = RADAR_FILTER_X_WINDOW_WIDTH;
    }
    Float64_t total = 0;
    for(int i = start_index; i < size; i++)
    {
        total += history_values_[i].x;
    }
    x = total/count;
    return true;
}

bool WindowSmoothFilter::CalcYValue(Float64_t & y)
{
    int size = history_values_.size();
    int start_index = 0;
    int count = size;
    if(history_values_.size() < RADAR_FILTER_Y_WINDOW_WIDTH)
    {
        start_index = 0;
        count = size;
    }
    else{
        start_index = size - RADAR_FILTER_Y_WINDOW_WIDTH;
        count = RADAR_FILTER_Y_WINDOW_WIDTH;
    }
    Float64_t total = 0;
    for(int i = start_index; i < size; i++)
    {
        total += history_values_[i].y;
    }
    y = total/count;
    return true;

}

bool WindowSmoothFilter::CalcVXValue(Float64_t & v_x)
{
    int size = history_values_.size();
    int start_index = 0;
    int count = size;
    if(history_values_.size() < RADAR_FILTER_VX_WINDOW_WIDTH)
    {
        start_index = 0;
        count = size;
    }
    else{
        start_index = size - RADAR_FILTER_VX_WINDOW_WIDTH;
        count = RADAR_FILTER_VX_WINDOW_WIDTH;
    }

    Float64_t total = 0;
    for(int i = start_index; i < size; i++)
    {
        total += history_values_[i].v_x;
    }
    v_x = total/count;
    return true;

}

bool WindowSmoothFilter::CalcVYValue(Float64_t & v_y)
{
    int size = history_values_.size();
    int start_index = 0;
    int count = size;
    if(history_values_.size() < RADAR_FILTER_VY_WINDOW_WIDTH)
    {
        start_index = 0;
        count = size;
    }
    else{
        start_index = size - RADAR_FILTER_VY_WINDOW_WIDTH;
        count = RADAR_FILTER_VY_WINDOW_WIDTH;
    }

    Float64_t total = 0;
    for(int i = start_index; i < size; i++)
    {
        total += history_values_[i].v_y;
    }
    v_y = total/count;
    return true;
}


void WindowSmoothFilter::AddObjectToList(const ad_msg::ObstacleRadar& object)
{
    // std::lock_guard<std::> lock(history_mutex);
    history_values_.push_back(object);
    if(history_values_.size() > RADAR_FILTER_WINDOW_WIDTH)
    {
        std::vector<ad_msg::ObstacleRadar>::iterator iter = history_values_.begin();
	    history_values_.erase(iter);
    }
}




}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix


