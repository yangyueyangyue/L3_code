#ifndef RADAR_KALMAN_FILTER_H_
#define RADAR_KALMAN_FILTER_H_

#include "geometry/aabbox2d.h"
#include "geometry/geometry_utils.h"
#include "utils/log.h"
#include "math/matrix.h"
#include "math/math_utils.h"
#include "radar_base_filter.h"


namespace phoenix{
namespace perception{
namespace radar{

/**
 * @brief 卡尔曼滤波使用时假定障碍物车辆在极短时间内是匀速行驶，忽略加速度的影响，
 * 也就是运动模型基于CV模型。 
 * 
 * 
 */

class KalmanFilter: public BaseFilter {


public:
    KalmanFilter();
    ~KalmanFilter();
    void Init(const ad_msg::ObstacleRadar& object);
    void Init(const ad_msg::ObstacleRadar &object, Matrix4d& p_in,
                Matrix4d& q_in, Matrix4d& r_in, Matrix4d& f_in, Matrix4d& h_in);
    bool  Predict(double time_diff);
    bool  UpdateWithObject(const ad_msg::ObstacleRadar& new_object,
                                        double time_diff);
    void GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y);
    Matrix4d GetCovarianceMatrix();

private:
    //状态向量
    Vector4d s_vector_;
    //状态协方差矩阵
    Matrix4d p_matrix_;
    //过程噪声矩阵
    Matrix4d q_matrix_;
    //测量噪声矩阵
    Matrix4d r_matrix_;
    //卡尔曼增益
    Matrix4d k_matrix_;
    //状态转移矩阵
    Matrix4d f_matrix_;
    //测量矩阵
    Matrix4d h_matrix_;
};







}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix


#endif
