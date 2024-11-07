#ifndef RADAR_WINDOW_FILTER_H_
#define RADAR_WINDOW_FILTER_H_

#include "geometry/aabbox2d.h"
#include "geometry/geometry_utils.h"
#include "utils/log.h"
#include "math/matrix.h"
#include "math/math_utils.h"
#include "radar_base_filter.h"
#include <vector>
#include <thread>         // std::thread
#include <mutex>  

namespace phoenix{
namespace perception{
namespace radar{

/**
 * @brief 以下宏用于设置窗口大小。
 * RADAR_FILTER_WINDOW_WIDTH 表示总的list保存的值得的多少，该值必须大于以下四个窗口大小。
 * RADAR_FILTER_X_WINDOW_WIDTH，RADAR_FILTER_Y_WINDOW_WIDTH，RADAR_FILTER_VX_WINDOW_WIDTH，RADAR_FILTER_VY_WINDOW_WIDTH
 * 分别表示
 * 
 * 该算法针对不同的运动相关的值，进行设置
 * 
 */
#define  RADAR_FILTER_WINDOW_WIDTH    10 
#define  RADAR_FILTER_X_WINDOW_WIDTH  (RADAR_FILTER_WINDOW_WIDTH-8)
#define  RADAR_FILTER_Y_WINDOW_WIDTH  (RADAR_FILTER_WINDOW_WIDTH-6)
#define  RADAR_FILTER_VX_WINDOW_WIDTH  (RADAR_FILTER_WINDOW_WIDTH-8)
#define  RADAR_FILTER_VY_WINDOW_WIDTH  (RADAR_FILTER_WINDOW_WIDTH-6)


class WindowSmoothFilter: public BaseFilter {

public:
    WindowSmoothFilter();
    ~WindowSmoothFilter();
    void Init(const ad_msg::ObstacleRadar& object);
    void Init(const ad_msg::ObstacleRadar &object, Matrix4d& p_in,
                Matrix4d& q_in, Matrix4d& r_in, Matrix4d& f_in, Matrix4d& h_in);
    bool  Predict(double time_diff);
    bool  UpdateWithObject(const ad_msg::ObstacleRadar& new_object,
                                        double time_diff);
    void GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y);

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

    std::vector<ad_msg::ObstacleRadar> history_values_;
    
    void AddObjectToList(const ad_msg::ObstacleRadar& object);
    bool CalcXValue(Float64_t & x);
    bool CalcYValue(Float64_t & y);
    bool CalcVXValue(Float64_t & v_x);
    bool CalcVYValue(Float64_t & v_y);

};







}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix






#endif