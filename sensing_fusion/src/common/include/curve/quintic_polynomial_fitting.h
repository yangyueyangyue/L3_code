/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       quintic_polynomial_fitting.h
 * @brief      5次多项式拟合
 * @details    根据输入的带参数（沿着曲线的长度）的二维点，使用最小二乘法拟合5次
 *             多项式曲线。
 *
 * @author     pengc
 * @date       2020.05.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_QUINTIC_POLYNOMIAL_FITTING_H_
#define PHOENIX_COMMON_QUINTIC_POLYNOMIAL_FITTING_H_

#include "utils/macros.h"
#include "utils/log.h"
#include "container/static_vector.h"
#include "math/matrix.h"
#include "curve/path.h"

// Enable to output debug information
#define ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE (0)

namespace phoenix {
namespace common {

/**
 * @class QuinticPolynomialFitting
 * @brief 使用最小二乘法实现5次多项式拟合的功能
 * @param Scalar 数据类型（float / double）
 * @param MaxPointNum 用于拟合的最多的点的数量
 */
template<typename Scalar, Int32_t MaxPointNum>
class QuinticPolynomialFitting {
public:
  /**
   * @brief 构造函数
   */
  QuinticPolynomialFitting() {
    has_constructed_ = false;
    has_start_constraint_ = false;
    start_s_ = 0;
    end_s_ = 0;
    length_ = 0;
    scale_s_ = 0;
  }

  /**
   * @brief 获取输入的待拟合的点集的起点的路径长
   * @return 输入的待拟合的点集的起点的路径长
   */
  inline Scalar start_s() const { return start_s_; }

  /**
   * @brief 获取输入的待拟合的点集的终点的路径长
   * @return 输入的待拟合的点集的终点的路径长
   */
  inline Scalar end_s() const { return end_s_; }

  /**
   * @brief 获取输入的待拟合的点集的总长
   * @return 输入的待拟合的点集的总长
   */
  inline Scalar length() const { return length_; }

  /**
   * @brief 将输入的点集使用最小二乘法拟合成5次多项式曲线(无约束)
   * @param[in] path_points 输入的待拟合的二维点集。 \n
   *           路径点中的s必须被正确设置为沿着路径的长度。
   * @return true 成功 \n
   *         false 失败
   *
   * @par Note:
   * @code
   *     1、5次多项式拟合圆会存在一定误差;
   *     2、5次多项式拟合的曲线（特别是圆形）在起点及终点处误差较大，若能够
   *        在输入的点集首尾处都增加反向及正向的延长的点，然后对拟合后的曲线
   *        去掉这些延长的点，效果会好些;
   *     3、输入点集的路径长s在此程序中，只是作为参数化曲线方程（x=fx(s), y=fy(s)）
   *        的参数，不能代表真实的沿着曲线的长度，在通过s获取曲线采样点的时候，需要特别注意。
   *
   * @endcode
   */
  bool ConstructWithoutConstraint(
      const StaticVector<PathPoint, MaxPointNum>& path_points);

  /**
   * @brief 将输入的点集使用最小二乘法拟合成5次多项式曲线(有起点约束)
   * @param[in] path_points 输入的待拟合的二维点集。 \n
   *            路径点中的s必须被正确设置为沿着路径的长度，
   *            另外第一个点的航向角及曲率必须正确设置为期望的值。
   * @return true 成功 \n
   *         false 失败
   *
   * @par Note:
   * @code
   *     1、5次多项式拟合圆会存在一定误差;
   *     2、5次多项式拟合的曲线（特别是圆形）在起点及终点处误差较大，若能够
   *        在输入的点集首尾处都增加反向及正向的延长的点，然后对拟合后的曲线
   *        去掉这些延长的点，效果会好些;
   *     3、输入点集的路径长s在此程序中，只是作为参数化曲线方程（x=fx(s), y=fy(s)）
   *        的参数，不能代表真实的沿着曲线的长度，在通过s获取曲线采样点的时候，需要特别注意。
   *
   * @endcode
   */
  bool ConstructWithStartConstraint(
      const StaticVector<PathPoint, MaxPointNum>& path_points);

  /**
   * @brief 获取路径长为s处的5次多项式上的点的信息（包含二维点的坐标、航向及曲率）
   * @param[in] s 路径长
   * @param[out] point 路径长为s处的点的信息（只填充二维点的坐标）
   * @return true 成功 \n
   *         false 失败
   *
   * @par Note:
   * @code
   *     1、输入点集的路径长s在此程序中，只是作为参数化曲线方程（x=fx(s), y=fy(s)）
   *        的参数，不能代表真实的沿着曲线的长度。
   *     2、通过这个函数返回的坐标，不是真实的沿着路径的长度上的点，可能近也可能远，
   *        通常在曲率较大的位置，采样点的间隔比较小，反之，则间隔比较大。
   *
   * @endcode
   */
  bool CalcSamplePoint(const Scalar s, Vec2d* path_point);

  /**
   * @brief 获取路径长为s处的5次多项式上的点的信息（包含二维点的坐标、航向及曲率）
   * @param[in] s 路径长
   * @param[out] path_point 路径长为s处的点的信息（只填充二维点的坐标、航向及曲率）
   * @return true 成功 \n
   *         false 失败
   *
   * @par Note:
   * @code
   *     1、输入点集的路径长s在此程序中，只是作为参数化曲线方程（x=fx(s), y=fy(s)）
   *        的参数，不能代表真实的沿着曲线的长度。
   *     2、通过这个函数返回的坐标，不是真实的沿着路径的长度上的点，可能近也可能远，
   *        通常在曲率较大的位置，采样点的间隔比较小，反之，则间隔比较大。
   *
   * @endcode
   */
  bool CalcSamplePoint(const Scalar s, PathPoint* path_point);

private:
  /*
   * @brief 内部用获取路径长为s处的5次多项式上的点的信息（二维点的坐标）
   * @param[in] s 路径长（减去起点长并且经过缩放后的路径长）
   * @param[out] point 路径长为s处的点的信息（只填充二维点的坐标）
   */
  void CalcSamplePointInteral(const Scalar s, Vec2d* point);

  /*
   * @brief 内部用获取路径长为s处的5次多项式上的点的信息（包含二维点的坐标、航向及曲率）
   * @param[in] s 路径长（减去起点长并且经过缩放后的路径长）
   * @param[out] path_point 路径长为s处的点的信息（只填充二维点的坐标、航向及曲率）
   */
  void CalcSamplePointInteral(const Scalar s, PathPoint* path_point);

  /*
   * @brief 计算路径长为s处的5次多项式曲线(参数化的曲线)处的指定阶数的导数
   * @param[in] order 导数的阶数
   * @param[in] s 路径长（减去起点长并且经过缩放后的路径长）
   * @param[out] coef 参数化曲线的的系数向量（x及y曲线是分别单独构造的）
   * @return 导数
   */
  Scalar CalcDerivative(const Uint32_t order, const Scalar s,
                        const Matrix<Scalar, MaxPointNum, 1>& coef);

private:
  // 多项式曲线的维度
  enum { PARAM_DIM = 6 };
  // 标记此5次多项式曲线是否已经被构造
  bool has_constructed_;
  bool has_start_constraint_;
  // 输入的待拟合的点集的起点的路径长
  Scalar start_s_;
  // 输入的待拟合的点集的终点的路径长
  Scalar end_s_;
  // 输入的待拟合的点集的总长
  Scalar length_;
  // 曲线长缩放因子（因为曲线中包含5次高次项目
  // 为了减小浮点运算中的舍入误差，对曲线长进行缩放）
  Scalar scale_s_;
  // 根据输入的待拟合的点集构造的系数矩阵
  Matrix<Scalar, MaxPointNum, PARAM_DIM> mat_coef_;
  // 保存对系数矩阵进行正交分解后的正交矩阵
  Matrix<Scalar, MaxPointNum, MaxPointNum> mat_orth_;
  // 参数化曲线系数向量x
  Matrix<Scalar, MaxPointNum, 1> coef_a_;
  // 参数化曲线系数向量y
  Matrix<Scalar, MaxPointNum, 1> coef_b_;
};


template<typename Scalar, Int32_t MaxPointNum>
bool QuinticPolynomialFitting<Scalar, MaxPointNum>::ConstructWithoutConstraint(
    const StaticVector<PathPoint, MaxPointNum>& path_points) {
  const Int32_t point_num = path_points.Size();

  if (point_num < PARAM_DIM) {
    LOG_ERR << "The number of points is not enough to fit the curve.";
    return (false);
  }

  start_s_ = path_points[0].s;
  end_s_ = path_points.Back().s;
  length_ = end_s_ - start_s_;
  scale_s_ = 0.2f*length_;

  if (length_ < 0.1f) {
    LOG_ERR << "The length of the input curve is too short.";
    return (false);
  }

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "########### QuinticPolynomialFitting::"
               "ConstructWithoutConstraint ############>"
            << std::endl;
  std::cout << "The input points is :" << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << i+1
              << "," << path_points[i].point.x()
              << "," << path_points[i].point.y()
              << "," << path_points[i].s
              << std::endl;
  }

  std::cout << "start_s=" << start_s_
            << ", end_s=" << end_s_
            << ", length=" << length_
            << ", scale_s=" << scale_s_
            << std::endl;
#endif

  // 构建系数矩阵
  Matrix<Scalar, MaxPointNum, 1> vec_x;
  Matrix<Scalar, MaxPointNum, 1> vec_y;
  for (Int32_t i = 0; i < point_num; ++i) {
    Scalar s = (path_points[i].s - start_s_) / scale_s_;
    for (Int32_t j = 0; j < PARAM_DIM; ++j) {
      if (0 == j) {
        mat_coef_(i, j) = 1;
      } else if (1 == j) {
        mat_coef_(i, j) = s;
      } else {
        mat_coef_(i, j) = s * mat_coef_(i, j - 1);
      }
    }
    vec_x(i) = path_points[i].point.x() / scale_s_;
    vec_y(i) = path_points[i].point.y() / scale_s_;
  }

  // 设置矩阵的行数为实际的点的数量
  mat_coef_.set_block(0, 0, point_num, PARAM_DIM);
  mat_orth_.set_block(0, 0, point_num, point_num);

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The coefficient matrix is:" << std::endl;
  std::cout << "    ";
  for (Int32_t j = 0; j < PARAM_DIM; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < PARAM_DIM; ++j) {
      std::cout << mat_coef_(i, j) << "\t";
    }
    std::cout << std::endl;
  }
#endif

  // 将系数矩阵进行QR分解
  if (false == Mat_HouseholderQR(mat_coef_, mat_orth_)) {
    LOG_ERR << "The coefficient matrix is invalid.";
    return (false);
  }

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "After Householder QR reduction:" << std::endl;
  std::cout << "The upper trapezoidal matrix is:" << std::endl;
  for (Int32_t j = 0; j < PARAM_DIM; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < PARAM_DIM; ++j) {
      std::cout << mat_coef_(i, j) << "\t";
    }
    std::cout << std::endl;
  }
  std::cout << "The orthogonal matrix is:" << std::endl;
  for (Int32_t j = 0; j < point_num; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < point_num; ++j) {
      std::cout << mat_orth_(i, j) << "\t";
    }
    std::cout << std::endl;
  }

  Matrix<Scalar, MaxPointNum, MaxPointNum> mat_orth_tran;
  Matrix<Scalar, MaxPointNum, MaxPointNum> mat_identity;
  Matrix<Scalar, MaxPointNum, PARAM_DIM> mat_recon;
  mat_orth_tran.set_block(0, 0, point_num, point_num);
  mat_identity.set_block(0, 0, point_num, point_num);
  mat_recon.set_block(0, 0, point_num, PARAM_DIM);
  Mat_Transpose(mat_orth_, mat_orth_tran);
  Mat_Mul(mat_orth_, mat_orth_tran, mat_identity);
  std::cout << "The product of multiplying Q by it's transpose is :"
            << std::endl;
  for (Int32_t j = 0; j < point_num; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < point_num; ++j) {
      std::cout << mat_identity(i, j) << "\t";
    }
    std::cout << std::endl;
  }
  Mat_Mul(mat_orth_tran, mat_coef_, mat_recon);
  std::cout << "The reconstructed matrix is :" << std::endl;
  for (Int32_t j = 0; j < PARAM_DIM; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < PARAM_DIM; ++j) {
      std::cout << mat_recon(i, j) << "\t";
    }
    std::cout << std::endl;
  }
#endif

  // 使用将系数矩阵分解后的QR矩阵求解多项式曲线的系数矩阵
  if (false == Mat_CalcLinearEquationFromQR(
        mat_coef_, mat_orth_, vec_x, coef_a_)) {
    LOG_ERR << "Failed to evaluate the coefficient a of the polynomial curve.";
    return false;
  }
  if (false == Mat_CalcLinearEquationFromQR(
        mat_coef_, mat_orth_, vec_y, coef_b_)) {
    LOG_ERR << "Failed to evaluate the coefficient b of the polynomial curve.";
    return false;
  }

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The coefficient a is: ";
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << " " << coef_a_[i];
  }
  std::cout << std::endl;
  std::cout << "The coefficient b is: ";
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << " " << coef_b_[i];
  }
  std::cout << std::endl;

  Scalar total_squared_err = 0;
  for (Int32_t i = PARAM_DIM; i < point_num; ++i) {
    total_squared_err += Square(coef_a_[i]) + Square(coef_b_[i]);
  }
  std::cout << "The total of squared error is: "
            << total_squared_err << std::endl;
  std::cout << "The avarage error is "
            << com_sqrt(total_squared_err) / point_num << std::endl;

  std::cout << "The coefficient of polynomial curve x(s) is: ";
  for (Int32_t i = 0; i < PARAM_DIM; ++i) {
    std::cout << " " << coef_a_[i];
  }
  std::cout << std::endl;
  std::cout << "The coefficient of polynomial curve y(s) is: ";
  for (Int32_t i = 0; i < PARAM_DIM; ++i) {
    std::cout << " " << coef_b_[i];
  }
  std::cout << std::endl;

  std::cout << "<########### QuinticPolynomialFitting::"
               "ConstructWithoutConstraint ############"
            << std::endl;
#endif

  has_constructed_ = true;

  return (true);
}

template<typename Scalar, Int32_t MaxPointNum>
bool QuinticPolynomialFitting<Scalar, MaxPointNum>::
    ConstructWithStartConstraint(
    const StaticVector<PathPoint, MaxPointNum>& path_points) {
  // 1 仅使用起点位置约束
  // 2 仅使用起点的位置约束和航向角约束
  // 3 起点的位置、航向角和曲率约束均使用
#define QUINTIC_FITTING_CONSTRAINT_NUM 2

  const Int32_t point_num = path_points.Size();
#if (QUINTIC_FITTING_CONSTRAINT_NUM == 1)
  const Int32_t param_dim = PARAM_DIM - 1;
#elif (QUINTIC_FITTING_CONSTRAINT_NUM == 2)
  const Int32_t param_dim = PARAM_DIM - 2;
#else
  const Int32_t param_dim = PARAM_DIM - 3;
#endif

  if (point_num < (param_dim+1)) {
    LOG_ERR << "The number of points is not enough to fit the curve.";
    return (false);
  }

  start_s_ = path_points[0].s;
  end_s_ = path_points.Back().s;
  length_ = end_s_ - start_s_;
  scale_s_ = 0.2f*length_;

  if (length_ < 0.1f) {
    LOG_ERR << "The length of the input curve is too short.";
    return (false);
  }

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "###### QuinticPolynomialFitting::"
               "ConstructWithStartConstraint #######>"
            << std::endl;
  std::cout << "The input points is :" << std::endl;
  for (Int32_t i = 0; i < point_num; ++i) {
    std::cout << i+1
              << "," << path_points[i].point.x()
              << "," << path_points[i].point.y()
              << "," << path_points[i].s
              << std::endl;
  }

  std::cout << "start_s=" << start_s_
            << ", end_s=" << end_s_
            << ", length=" << length_
            << ", scale_s=" << scale_s_
            << std::endl;
#endif

  // 根据输入的起点约束，设置参数化方程的起点约束
  Scalar x = path_points[0].point.x() / scale_s_;
  Scalar y = path_points[0].point.y() / scale_s_;
  Scalar dx = com_cos(path_points[0].heading);
  Scalar dy = com_sin(path_points[0].heading);
  Scalar ddx = 0;
  Scalar ddy = 0;

  // The method of calculating ddx and ddy from curvature is found from the
  // curvature equation that is
  // curvature = (dx*ddy - dy*ddx) / ((dx*dx+dy*dy)*sqrt(dx*dx+dy*dy))
  // where dx and dy is the first order of the parametric curve
  // { x = fx(s); y = fy(s); } , ddx and ddy is the second order
  // of the parametric curve respectively.
  const Scalar ds = 0.001f;
  Scalar dh = path_points[0].curvature * ds * scale_s_;
  Scalar nh = path_points[0].heading + dh;
  Scalar ndx = com_cos(nh);
  Scalar ndy = com_sin(nh);
//  ddx = (ndx - dx) / ds;
//  ddy = (ndy - dy) / ds;
  ddx = -com_sin(path_points[0].heading) * path_points[0].curvature * scale_s_;
  ddy =  com_cos(path_points[0].heading) * path_points[0].curvature * scale_s_;

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The constraint of start point is: "
            << "point=[" << x << ", " << y
            << "], heading=" << com_rad2deg(path_points[0].heading) << "deg"
            << ", curvature=" << path_points[0].curvature
            << std::endl;
  std::cout << "The constraint of start of parametric curve is: "
            << "x=" << x << ", y=" << y
            << ", dx=" << dx << ", dy=" << dy
            << ", ddx=" << ddx << ", ddy=" << ddy
            << std::endl;
  Scalar squared_ders_length = dx*dx + dy*dy;
  Scalar curvature_check = (dx*ddy-dy*ddx)
      / (squared_ders_length*com_sqrt(squared_ders_length));
  std::cout << "The curvature reconstructed from (dx,dy,ddx,ddy) is "
            << curvature_check << std::endl;
#endif

  // 构建系数矩阵
#if (QUINTIC_FITTING_CONSTRAINT_NUM == 1)
  Matrix<Scalar, MaxPointNum, 1> vec_x;
  Matrix<Scalar, MaxPointNum, 1> vec_y;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    Scalar s = (path_points[i+1].s - start_s_) / scale_s_;
    for (Int32_t j = 0; j < param_dim; ++j) {
      if (0 == j) {
        mat_coef_(i, j) = s;
      } else {
        mat_coef_(i, j) = s * mat_coef_(i, j - 1);
      }
    }
    vec_x(i) = path_points[i+1].point.x() / scale_s_ - x;
    vec_y(i) = path_points[i+1].point.y() / scale_s_ - y;
  }
#elif (QUINTIC_FITTING_CONSTRAINT_NUM == 2)
  Matrix<Scalar, MaxPointNum, 1> vec_x;
  Matrix<Scalar, MaxPointNum, 1> vec_y;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    Scalar s = (path_points[i+1].s - start_s_) / scale_s_;
    for (Int32_t j = 0; j < param_dim; ++j) {
      if (0 == j) {
        mat_coef_(i, j) = s * s;
      } else {
        mat_coef_(i, j) = s * mat_coef_(i, j - 1);
      }
    }
    vec_x(i) = path_points[i+1].point.x() / scale_s_ - (x + dx*s);
    vec_y(i) = path_points[i+1].point.y() / scale_s_ - (y + dy*s);
  }
#else
  Matrix<Scalar, MaxPointNum, 1> vec_x;
  Matrix<Scalar, MaxPointNum, 1> vec_y;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    Scalar s = (path_points[i+1].s - start_s_) / scale_s_;
    for (Int32_t j = 0; j < param_dim; ++j) {
      if (0 == j) {
        mat_coef_(i, j) = s * s * s;
      } else {
        mat_coef_(i, j) = s * mat_coef_(i, j - 1);
      }
    }
    vec_x(i) = path_points[i + 1].point.x() / scale_s_ -
        (x + (dx + 0.5f * ddx * s) * s);
    vec_y(i) = path_points[i + 1].point.y() / scale_s_ -
        (y + (dy + 0.5f * ddy * s) * s);
  }
#endif

  // 设置矩阵的行数为实际的点的数量
  mat_coef_.set_block(0, 0, point_num - 1, param_dim);
  mat_orth_.set_block(0, 0, point_num - 1, point_num - 1);

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The coefficient matrix is:" << std::endl;
  std::cout << "    ";
  for (Int32_t j = 0; j < param_dim; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < param_dim; ++j) {
      std::cout << mat_coef_(i, j) << "\t";
    }
    std::cout << std::endl;
  }
#endif

  // 将系数矩阵进行QR分解
  if (false == Mat_HouseholderQR(mat_coef_, mat_orth_)) {
    LOG_ERR << "The coefficient matrix is invalid.";
    return (false);
  }

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "After Householder QR reduction:" << std::endl;
  std::cout << "The upper trapezoidal matrix is:" << std::endl;
  for (Int32_t j = 0; j < param_dim; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < param_dim; ++j) {
      std::cout << mat_coef_(i, j) << "\t";
    }
    std::cout << std::endl;
  }
  std::cout << "The orthogonal matrix is:" << std::endl;
  for (Int32_t j = 0; j < (point_num-1); ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < (point_num-1); ++j) {
      std::cout << mat_orth_(i, j) << "\t";
    }
    std::cout << std::endl;
  }

  Matrix<Scalar, MaxPointNum, MaxPointNum> mat_orth_tran;
  Matrix<Scalar, MaxPointNum, MaxPointNum> mat_identity;
  Matrix<Scalar, MaxPointNum, PARAM_DIM> mat_recon;
  mat_orth_tran.set_block(0, 0, point_num - 1, point_num - 1);
  mat_identity.set_block(0, 0, point_num - 1, point_num - 1);
  mat_recon.set_block(0, 0, point_num - 1, param_dim);
  Mat_Transpose(mat_orth_, mat_orth_tran);
  Mat_Mul(mat_orth_, mat_orth_tran, mat_identity);
  std::cout << "The product of multiplying Q by it's transpose is :"
            << std::endl;
  for (Int32_t j = 0; j < (point_num-1); ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < (point_num-1); ++j) {
      std::cout << mat_identity(i, j) << "\t";
    }
    std::cout << std::endl;
  }
  Mat_Mul(mat_orth_tran, mat_coef_, mat_recon);
  std::cout << "The reconstructed matrix is :" << std::endl;
  for (Int32_t j = 0; j < param_dim; ++j) {
    std::cout << "[" << j << "]\t";
  }
  std::cout << std::endl;
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << "[" << i << "] ";
    for (Int32_t j = 0; j < param_dim; ++j) {
      std::cout << mat_recon(i, j) << "\t";
    }
    std::cout << std::endl;
  }
#endif

  // 使用将系数矩阵分解后的QR矩阵求解多项式曲线的系数矩阵
  if (false == Mat_CalcLinearEquationFromQR(
        mat_coef_, mat_orth_, vec_x, coef_a_)) {
    LOG_ERR << "Failed to evaluate the coefficient a of the polynomial curve.";
    return false;
  }
  if (false == Mat_CalcLinearEquationFromQR(
        mat_coef_, mat_orth_, vec_y, coef_b_)) {
    LOG_ERR << "Failed to evaluate the coefficient b of the polynomial curve.";
    return false;
  }

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The coefficient a is: ";
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << " " << coef_a_[i];
  }
  std::cout << std::endl;
  std::cout << "The coefficient b is: ";
  for (Int32_t i = 0; i < (point_num-1); ++i) {
    std::cout << " " << coef_b_[i];
  }
  std::cout << std::endl;

  Scalar total_squared_err = 0;
  for (Int32_t i = param_dim; i < (point_num-1); ++i) {
    total_squared_err += Square(coef_a_[i]) + Square(coef_b_[i]);
  }
  std::cout << "The total of squared error is: "
            << total_squared_err << std::endl;
  std::cout << "The avarage error is "
            << com_sqrt(total_squared_err) / (point_num-1) << std::endl;
#endif

  // 更新参数化方程的系数向量
#if (QUINTIC_FITTING_CONSTRAINT_NUM == 1)
  coef_a_[5] = coef_a_[4];
  coef_a_[4] = coef_a_[3];
  coef_a_[3] = coef_a_[2];
  coef_a_[2] = coef_a_[1];
  coef_a_[1] = coef_a_[0];
  coef_a_[0] = x;

  coef_b_[5] = coef_b_[4];
  coef_b_[4] = coef_b_[3];
  coef_b_[3] = coef_b_[2];
  coef_b_[2] = coef_b_[1];
  coef_b_[1] = coef_b_[0];
  coef_b_[0] = y;
#elif (QUINTIC_FITTING_CONSTRAINT_NUM == 2)
  coef_a_[5] = coef_a_[3];
  coef_a_[4] = coef_a_[2];
  coef_a_[3] = coef_a_[1];
  coef_a_[2] = coef_a_[0];
  coef_a_[1] = dx;
  coef_a_[0] = x;

  coef_b_[5] = coef_b_[3];
  coef_b_[4] = coef_b_[2];
  coef_b_[3] = coef_b_[1];
  coef_b_[2] = coef_b_[0];
  coef_b_[1] = dy;
  coef_b_[0] = y;
#else
  coef_a_[5] = coef_a_[2];
  coef_a_[4] = coef_a_[1];
  coef_a_[3] = coef_a_[0];
  coef_a_[2] = 0.5f*ddx;
  coef_a_[1] = dx;
  coef_a_[0] = x;

  coef_b_[5] = coef_b_[2];
  coef_b_[4] = coef_b_[1];
  coef_b_[3] = coef_b_[0];
  coef_b_[2] = 0.5f*ddy;
  coef_b_[1] = dy;
  coef_b_[0] = y;
#endif

#if ENABLE_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The coefficient of polynomial curve x(s) is: ";
  for (Int32_t i = 0; i < PARAM_DIM; ++i) {
    std::cout << " " << coef_a_[i];
  }
  std::cout << std::endl;
  std::cout << "The coefficient of polynomial curve y(s) is: ";
  for (Int32_t i = 0; i < PARAM_DIM; ++i) {
    std::cout << " " << coef_b_[i];
  }
  std::cout << std::endl;
  std::cout << "<###### QuinticPolynomialFitting::"
               "ConstructWithStartConstraint #######"
            << std::endl;
#endif

  has_constructed_ = true;

  return (true);
}

template<typename Scalar, Int32_t MaxPointNum>
bool QuinticPolynomialFitting<Scalar, MaxPointNum>::CalcSamplePoint(
    const Scalar s, Vec2d* point) {
  if (!has_constructed_) {
    LOG_ERR << "The polynomial curve has not been constructed.";
    return (false);
  }

  CalcSamplePointInteral((s - start_s_) / scale_s_, point);

  point->set_x(point->x() * scale_s_);
  point->set_y(point->y() * scale_s_);

  return (true);
}

template<typename Scalar, Int32_t MaxPointNum>
bool QuinticPolynomialFitting<Scalar, MaxPointNum>::CalcSamplePoint(
    const Scalar s, PathPoint* path_point) {
  if (!has_constructed_) {
    LOG_ERR << "The polynomial curve has not been constructed.";
    return (false);
  }

  CalcSamplePointInteral((s - start_s_) / scale_s_, path_point);

  path_point->point.set_x(path_point->point.x() * scale_s_);
  path_point->point.set_y(path_point->point.y() * scale_s_);
  path_point->curvature /= scale_s_;

  return (true);
}

template<typename Scalar, Int32_t MaxPointNum>
void QuinticPolynomialFitting<Scalar, MaxPointNum>::CalcSamplePointInteral(
    const Scalar s, Vec2d* point) {
  Scalar x = CalcDerivative(0, s, coef_a_);
  Scalar y = CalcDerivative(0, s, coef_b_);
  point->set_x(x);
  point->set_y(y);
}

template<typename Scalar, Int32_t MaxPointNum>
void QuinticPolynomialFitting<Scalar, MaxPointNum>::CalcSamplePointInteral(
    const Scalar s, PathPoint* path_point) {
  Scalar x = CalcDerivative(0, s, coef_a_);
  Scalar y = CalcDerivative(0, s, coef_b_);
  path_point->point.set_x(x);
  path_point->point.set_y(y);

  Scalar dx = CalcDerivative(1, s, coef_a_);
  Scalar dy = CalcDerivative(1, s, coef_b_);
  Scalar ddx = CalcDerivative(2, s, coef_a_);
  Scalar ddy = CalcDerivative(2, s, coef_b_);
  path_point->heading = com_atan2(dy, dx);
  Scalar squared_ders_length = dx*dx + dy*dy;
  path_point->curvature = (dx*ddy-dy*ddx)
      / (squared_ders_length*com_sqrt(squared_ders_length));
}

template<typename Scalar, Int32_t MaxPointNum>
Scalar QuinticPolynomialFitting<Scalar, MaxPointNum>::CalcDerivative(
    const Uint32_t order, const Scalar s,
    const Matrix<Scalar, MaxPointNum, 1>& coef) {
  Scalar ret = 0;

  switch (order) {
  case 0:
    ret = ((((coef[5] * s + coef[4]) * s + coef[3]) * s + coef[2]) * s +
        coef[1]) * s + coef[0];
    break;
  case 1:
    ret = (((5 * coef[5] * s + 4 * coef[4]) * s + 3 * coef[3]) * s +
        2 * coef[2]) * s + coef[1];
    break;
  case 2:
    ret = (((20 * coef[5] * s + 12 * coef[4]) * s) + 6 * coef[3]) *
        s + 2 * coef[2];
    break;
  case 3:
    ret = (60 * coef[5] * s + 24 * coef[4]) * s + 6 * coef[3];
    break;
  case 4:
    ret = 120 * coef[5] * s + 24 * coef[4];
    break;
  case 5:
    ret = 120 * coef[5];
    break;
  default:
    ret = 0;
    break;
  }

  return (ret);
}


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_QUINTIC_POLYNOMIAL_FITTING_H_
