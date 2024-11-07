#include "radar_kf_filter.h"


namespace phoenix{
namespace perception{
namespace radar{

#if ENABLE_USING_FILTER_BASED_ON_EIGEN

RadarKalmanFilter::RadarKalmanFilter()
{

}

RadarKalmanFilter::~RadarKalmanFilter()
{

}

void RadarKalmanFilter::Init(const ad_msg::ObstacleRadar& object)
{

    Eigen::Matrix4d p_in, q_in, r_in, f_in, h_in;
    p_in.setIdentity();//
    q_in.setIdentity();
    r_in.setZero();

/*
平滑参数设置
*/
    r_in(0,0) = 0.14; //纵向 x
    r_in(1,1) = 3.33; //横向 y
    r_in(2,2) = 1.05; //纵向 v_x
    r_in(3,3) = 5.55; //纵向 v_y

    p_in(2,2) = 50;
    p_in(3,3) = 50;

    f_in.setIdentity();
    h_in.setIdentity();


    Init(object, p_in, q_in, r_in, f_in, h_in);


}


void RadarKalmanFilter::Init(const ad_msg::ObstacleRadar &object, Eigen::Matrix4d& p_in,
                Eigen::Matrix4d& q_in, Eigen::Matrix4d& r_in, Eigen::Matrix4d& f_in, Eigen::Matrix4d& h_in)
{
    s_vector_ << object.x,object.y,object.v_x,object.v_y; 
    p_matrix_ = p_in;
    q_matrix_ = q_in;
    r_matrix_ = r_in;
    f_matrix_ = f_in;
    h_matrix_ = h_in;
}

bool  RadarKalmanFilter::Predict(double time_diff)
{
    f_matrix_(0, 2) = time_diff;
    f_matrix_(1, 3) = time_diff;
    s_vector_ = f_matrix_* s_vector_;
    p_matrix_  = f_matrix_ * p_matrix_ * f_matrix_.transpose() + q_matrix_;
    return true;
}


bool  RadarKalmanFilter::UpdateWithObject(const ad_msg::ObstacleRadar& new_object,
                                    double time_diff)
{
    Eigen::Vector4d z_vector_;
    z_vector_  << new_object.x,new_object.y,new_object.v_x,new_object.v_y; 
    Eigen::MatrixXd S = h_matrix_ * p_matrix_ * h_matrix_.transpose() + r_matrix_;
    Eigen::MatrixXd K = p_matrix_ * h_matrix_.transpose() * S.inverse();
    s_vector_ = s_vector_ + K * (z_vector_ - h_matrix_ * s_vector_);
    p_matrix_ = (Eigen::MatrixXd::Identity(p_matrix_.rows(), p_matrix_.cols()) - K * h_matrix_) * p_matrix_;
    return true;
}

void RadarKalmanFilter::GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y)
{
    x  = s_vector_(0);
    y  = s_vector_(1);
    v_x  = s_vector_(2);
    v_y  = s_vector_(3);
}

#endif

}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix
