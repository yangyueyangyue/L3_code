#ifndef RADAR_KF_FILTER_H_
#define RADAR_KF_FILTER_H_
#include "radar_base_filter.h"

#if ENABLE_USING_FILTER_BASED_ON_EIGEN

#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include <cmath>


namespace phoenix{
namespace perception{
namespace radar{

class RadarKalmanFilter: public BaseFilter {


public:
    RadarKalmanFilter();
    ~RadarKalmanFilter();
    void Init(const ad_msg::ObstacleRadar& object);
    void Init(const ad_msg::ObstacleRadar &object, Eigen::Matrix4d& p_in,
                Eigen::Matrix4d& q_in, Eigen::Matrix4d& r_in, Eigen::Matrix4d& f_in, Eigen::Matrix4d& h_in);
    bool  Predict(double time_diff);
    bool  UpdateWithObject(const ad_msg::ObstacleRadar& new_object,
                                        double time_diff);
    void GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y);


private:
    //状态向量
    Eigen::Vector4d s_vector_;
    //状态协方差矩阵
    Eigen::Matrix4d p_matrix_;
    //过程噪声矩阵
    Eigen::Matrix4d q_matrix_;
    //测量噪声矩阵
    Eigen::Matrix4d r_matrix_;
    //卡尔曼增益
    Eigen::Matrix4d k_matrix_;
    //状态转移矩阵
    Eigen::Matrix4d f_matrix_;
    //测量矩阵
    Eigen::Matrix4d h_matrix_;
};




}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix



#endif














#endif
