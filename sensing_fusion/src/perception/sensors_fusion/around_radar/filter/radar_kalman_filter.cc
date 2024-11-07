#include "radar_kalman_filter.h"

namespace phoenix
{
namespace perception
{
namespace radar
{

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(const ad_msg::ObstacleRadar &object)
{
    Matrix4d p_in, q_in, r_in, f_in, h_in;
    p_in.SetIdentity();
    q_in.SetIdentity();
    r_in.SetZeros();
    s_vector_.SetBlock(0,0,4,1);
    p_matrix_.SetBlock(0,0,4,4);
    q_matrix_.SetBlock(0,0,4,4);
    r_matrix_.SetBlock(0,0,4,4);
    f_matrix_.SetBlock(0,0,4,4);
    h_matrix_.SetBlock(0,0,4,4);
    k_matrix_.SetBlock(0,0,4,4);

/*
平滑参数设置
*/
    r_in(0,0) = 0.14; //纵向 x
    r_in(1,1) = 3.33; //横向 y
    r_in(2,2) = 1.05; //纵向 v_x
    r_in(3,3) = 5.55; //纵向 v_y

    p_in(2,2) = 50;
    p_in(3,3) = 50;

    f_in.SetIdentity();
    h_in.SetIdentity();


    Init(object, p_in, q_in, r_in, f_in, h_in);
}

void KalmanFilter::Init(const ad_msg::ObstacleRadar &object, Matrix4d& p_in,
                        Matrix4d& q_in, Matrix4d& r_in, Matrix4d& f_in, Matrix4d& h_in)
{
    s_vector_(0) = object.x;
    s_vector_(1) = object.y;
    s_vector_(2) = object.v_x;
    s_vector_(3) = object.v_y;
    p_matrix_ = p_in;
    q_matrix_ = q_in;
    r_matrix_ = r_in;
    f_matrix_ = f_in;
    h_matrix_ = h_in;
}

/**
 * @brief 
 * 
 * @param time_diff 
 * @return true 
 * @return false 
 */
bool KalmanFilter::Predict(double time_diff)
{

    f_matrix_.SetBlock(0,0,4,4);
    s_vector_.SetBlock(0,0,4,1);
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "Predict POS00" << f_matrix_(0, 2);

    f_matrix_(0, 2) = time_diff;
    f_matrix_(1, 3) = time_diff;
    /*   
    Matrix4d s_tmp;
    common::Mat_Mul(f_matrix_, s_vector_, s_tmp);
    f_matrix_ = s_tmp;
    */
    s_vector_(0) = f_matrix_(0,0) * s_vector_(0) +  f_matrix_(0,2) * s_vector_(2);
    s_vector_(1) = f_matrix_(1,1) * s_vector_(1) +  f_matrix_(1,3) * s_vector_(3);
    /*
    更新状态协方差矩阵
    */
    Matrix4d f_p_tmp, f_t_tmp, f_p_f_tmp, p_tmp;
    f_p_tmp.SetBlock(0,0,4,4);
    f_t_tmp.SetBlock(0,0,4,4);
    f_p_f_tmp.SetBlock(0,0,4,4);
    p_tmp.SetBlock(0,0,4,4);
    p_matrix_.SetBlock(0,0,4,4);
    q_matrix_.SetBlock(0,0,4,4);
    common::Mat_Mul(f_matrix_, p_matrix_, f_p_tmp);
    common::Mat_Transpose(f_matrix_, f_t_tmp);
    common::Mat_Mul(f_p_tmp, f_t_tmp, f_p_f_tmp);
    common::Mat_Add(f_p_f_tmp, q_matrix_, p_tmp);
    p_matrix_ = p_tmp;

    return true;
}

bool KalmanFilter::UpdateWithObject(const ad_msg::ObstacleRadar &new_object,
                                                                double time_diff)
{   
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS00";
    Vector4d measure, vtmp0, vtmp1, y_tmp;
    Matrix4d tmp0, h_t_tmp, tmp1, k_tmp;
    

    measure.SetBlock(0,0,4,1);
    y_tmp.SetBlock(0,0,4,1);
    vtmp0.SetBlock(0,0,4,1);
    vtmp1.SetBlock(0,0,4,1);

    tmp0.SetBlock(0,0,4,4);
    h_t_tmp.SetBlock(0,0,4,4);
    tmp1.SetBlock(0,0,4,4);
    k_tmp.SetBlock(0,0,4,4);

    measure(0) = new_object.x;
    measure(1) = new_object.y;
    measure(2) = new_object.v_x;
    measure(3) = new_object.v_y;

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS01";

    s_vector_.SetBlock(0,0,4,1);
    h_matrix_.SetBlock(0,0,4,4);
    p_matrix_.SetBlock(0,0,4,4);
    r_matrix_.SetBlock(0,0,4,4);
    //计算y = z - H*s
    common::Mat_Mul(h_matrix_, s_vector_, vtmp0);
    common::Mat_Sub(measure, vtmp0, y_tmp);
    
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS02";

    //计算H矩阵的转置H_t
    common::Mat_Transpose(h_matrix_, h_t_tmp);
    
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS03";

    //为计算卡尔曼增益，先计算 (H * P * H_t + R)^-1 
    common::Mat_Mul(h_matrix_, p_matrix_, tmp0);
    common::Mat_Mul(tmp0, h_t_tmp, tmp1);
    common::Mat_Add(tmp1, r_matrix_, tmp0);
    common::Mat_CalcPseudoInverse(tmp0, tmp1);//算出矩阵的伪逆

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS04";

    //计算卡尔曼增益
    common::Mat_Mul(p_matrix_, h_t_tmp, tmp0);
    common::Mat_Mul(tmp0, tmp1, k_tmp);

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS05";

    //更新最后的状态矩阵
    common::Mat_Mul(k_tmp, y_tmp, vtmp0);
    common::Mat_Add(s_vector_, vtmp0, vtmp1);//此时算出来的tmp1位最后的状态矩阵
    s_vector_ = vtmp1; //更新状态矩阵
    
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS06";

    //更新状态协方差矩阵, 计算(P-K*H*P)
    common::Mat_Mul(k_tmp, h_matrix_, tmp0);
    common::Mat_Mul(tmp0, p_matrix_, tmp1);
    common::Mat_Sub(p_matrix_, tmp1, tmp0);
    p_matrix_ = tmp0; //更新状态协方差矩阵

    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "UpdateWithObject POS07";

    return true;
}

void KalmanFilter::GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y)
{   
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "GetState POS00";
    x = static_cast<Float32_t>(s_vector_(0));
    y = static_cast<Float32_t>(s_vector_(1));
    v_x = static_cast<Float32_t>(s_vector_(2));
    v_y = static_cast<Float32_t>(s_vector_(3));
    LOG_INFO(RADAR_PRE_LOG_LEVEL) << "GetState POS01";
}

Matrix4d KalmanFilter::GetCovarianceMatrix()
{
    return p_matrix_;
}

} // end of namespace radar
}     // end of namespace perception
} // end of namespace phoenix
