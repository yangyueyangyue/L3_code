/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       quaternion.h
 * @brief      四元数运算
 * @details    定义四元数运算
 *
 * @author     pengc
 * @date       2020.06.01
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/01  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_COMMON_QUATERNION_H_
#define PHOENIX_COMMON_QUATERNION_H_

#include <cmath>
#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "math/matrix.h"


namespace phoenix {
namespace common {


/**
 * @class Matrix
 * @brief 定义了四元数的数据结构及相关运算
 * @param Scalar 四元数中元素的数据类型
 */
template<typename Scalar>
class Quaternion {
public:
  /**
   * @brief 构造函数
   */
  Quaternion() {
    com_memset(coeffs_, 0, sizeof(coeffs_));
  }

  /**
   * @brief 构造函数
   * @param[in] w 实部的值
   * @param[in] x 虚部坐标x的值
   * @param[in] y 虚部坐标y的值
   * @param[in] z 虚部坐标z的值
   */
  Quaternion(Scalar w, Scalar x, Scalar y, Scalar z) {
    coeffs_[0] = w;
    coeffs_[1] = x;
    coeffs_[2] = y;
    coeffs_[3] = z;
  }

  /**
   * @brief 拷贝构造函数
   * @param[in] other 另一个四元数
   */
  Quaternion(const Quaternion& other) {
    coeffs_[0] = other.coeffs_[0];
    coeffs_[1] = other.coeffs_[1];
    coeffs_[2] = other.coeffs_[2];
    coeffs_[3] = other.coeffs_[3];
  }

  /**
   * @brief 赋值运算，重载=操作符号
   * @param[in] other 另一个四元数
   */
  void operator =(const Quaternion& other) {
    coeffs_[0] = other.coeffs_[0];
    coeffs_[1] = other.coeffs_[1];
    coeffs_[2] = other.coeffs_[2];
    coeffs_[3] = other.coeffs_[3];
  }

  /**
   * @brief 获取实部的值
   * @return 实部的值
   */
  inline Scalar w() const { return (coeffs_[0]); }
  /**
   * @brief 获取实部的值
   * @return 实部的值
   */
  inline Scalar& w() { return (coeffs_[0]); }

  /**
   * @brief 获取虚部坐标x的值
   * @return 虚部坐标x的值
   */
  inline Scalar x() const { return (coeffs_[1]); }
  /**
   * @brief 获取虚部坐标y的值
   * @return 虚部坐标y的值
   */
  inline Scalar y() const { return (coeffs_[2]); }
  /**
   * @brief 获取虚部坐标z的值
   * @return 虚部坐标z的值
   */
  inline Scalar z() const { return (coeffs_[3]); }

  /**
   * @brief 获取虚部坐标x的值
   * @return 虚部坐标x的值
   */
  inline Scalar& x() { return (coeffs_[1]); }
  /**
   * @brief 获取虚部坐标y的值
   * @return 虚部坐标y的值
   */
  inline Scalar& y() { return (coeffs_[2]); }
  /**
   * @brief 获取虚部坐标z的值
   * @return 虚部坐标z的值
   */
  inline Scalar& z() { return (coeffs_[3]); }

  /**
   * @brief 获取四元数的模的平方值
   * @return 模的平方值
   */
  inline Scalar SquaredNorm() const {
    return (Square(coeffs_[0]) + Square(coeffs_[1]) +
        Square(coeffs_[2]) + Square(coeffs_[3]));
  }

  /**
   * @brief 获取四元数的模
   * @return 模
   */
  inline Scalar Norm() const {
    return (com_sqrt(SquaredNorm()));
  }

  /**
   * @brief 将当前四元数单位化
   */
  void NormalizeInplace() {
    Scalar norm = Norm();
    coeffs_[0] /= norm;
    coeffs_[1] /= norm;
    coeffs_[2] /= norm;
    coeffs_[3] /= norm;
  }

  /**
   * @brief 将当前四元数设置为不旋转
   */
  void SetIdentity() {
    coeffs_[0] = 1;
    coeffs_[1] = 0;
    coeffs_[2] = 0;
    coeffs_[3] = 0;
  }

  /**
   * @brief 从旋转角度和旋转轴构建四元数
   * @param[in] angle 需要绕旋转轴旋转的角度值
   * @param[in] axis 旋转轴
   */
  template <typename VecType>
  void ConstructFromAngleAxis(const Scalar angle, const VecType& axis) {
    Scalar ha = Scalar(0.5)*angle;
    coeffs_[0] = com_cos(ha);
    Scalar sin_ha = com_sin(ha);
    coeffs_[1] = axis(0)*sin_ha;
    coeffs_[2] = axis(1)*sin_ha;
    coeffs_[3] = axis(2)*sin_ha;
  }

  /**
   * @brief 将四元数转化为旋转角度和旋转轴
   * @param[out] angle 绕旋转轴旋转的角度值
   * @param[out] axis 旋转轴
   */
  template <typename VecType>
  void ConvertToAngleAxis(Scalar* angle, VecType* axis) const {
    Scalar n = com_sqrt(Square(coeffs_[1]) + Square(coeffs_[2]) +
        Square(coeffs_[3]));

    if (n < NumLimits<Scalar>::epsilon()) {
      *angle = Scalar(2)*com_atan2(n, com_abs(coeffs_[0]));
      if(coeffs_[0] < 0) {
        n = -n;
      }
      (*axis)(0)  = coeffs_[1] / n;
      (*axis)(1)  = coeffs_[2] / n;
      (*axis)(2)  = coeffs_[3] / n;
    } else {
      *angle = Scalar(0);
      (*axis)(0)  = Scalar(1);
      (*axis)(1)  = Scalar(0);
      (*axis)(2)  = Scalar(0);
    }
  }

  /**
   * @brief 从旋转矩阵构建四元数
   * @param[in] mat_rot 旋转矩阵
   */
  template <typename MatType>
  void ConstructFromRotationMatrix(const MatType& mat_rot) {
    COM_CHECK(mat_rot.rows() == mat_rot.cols() &&
              mat_rot.rows() == 3);

    Scalar t = mat_rot(0, 0) + mat_rot(1, 1) + mat_rot(2, 2);
    if (t > Scalar(0)) {
      t = com_sqrt(t + Scalar(1.0));
      coeffs_[0] = Scalar(0.5)*t;
      t = Scalar(0.5)/t;
      coeffs_[1] = (mat_rot(2, 1) - mat_rot(1, 2)) * t;
      coeffs_[2] = (mat_rot(0, 2) - mat_rot(2, 0)) * t;
      coeffs_[3] = (mat_rot(1, 0) - mat_rot(0, 1)) * t;
    } else {
      Int32_t i = 0;
      if (mat_rot(1, 1) > mat_rot(0, 0)) {
        i = 1;
      }
      if (mat_rot(2, 2) > mat_rot(i, i)) {
        i = 2;
      }
      Int32_t j = (i + 1) % 3;
      Int32_t k = (j + 1) % 3;

      t = com_sqrt(mat_rot(i, i) - mat_rot(j, j) - mat_rot(k, k) + Scalar(1.0));
      coeffs_[i+1] = Scalar(0.5) * t;
      t = Scalar(0.5) / t;
      coeffs_[0] = (mat_rot(k, j) - mat_rot(j, k))*t;
      coeffs_[j+1] = (mat_rot(j, i) + mat_rot(i, j))*t;
      coeffs_[k+1] = (mat_rot(k, i) + mat_rot(i, k))*t;
    }
  }

  /**
   * @brief 将四元数转化为旋转矩阵
   * @param[out] mat_rot 旋转矩阵
   */
  template <typename MatType>
  void ConvertToRotationMatrix(MatType* mat_rot) const {
    COM_CHECK(mat_rot->rows() == mat_rot->cols() &&
              mat_rot->rows() == 3);

    const Scalar tx  = Scalar(2)*this->x();
    const Scalar ty  = Scalar(2)*this->y();
    const Scalar tz  = Scalar(2)*this->z();
    const Scalar twx = tx*this->w();
    const Scalar twy = ty*this->w();
    const Scalar twz = tz*this->w();
    const Scalar txx = tx*this->x();
    const Scalar txy = ty*this->x();
    const Scalar txz = tz*this->x();
    const Scalar tyy = ty*this->y();
    const Scalar tyz = tz*this->y();
    const Scalar tzz = tz*this->z();

    (*mat_rot)(0, 0) = Scalar(1) - (tyy + tzz);
    (*mat_rot)(0, 1) = txy - twz;
    (*mat_rot)(0, 2) = txz + twy;
    (*mat_rot)(1, 0) = txy + twz;
    (*mat_rot)(1, 1) = Scalar(1) - (txx + tzz);
    (*mat_rot)(1, 2) = tyz - twx;
    (*mat_rot)(2, 0) = txz - twy;
    (*mat_rot)(2, 1) = tyz + twx;
    (*mat_rot)(2, 2) = Scalar(1) - (txx + tyy);
  }

  /**
   * @brief 获取当前四元数的共轭
   * @return 四元数的共轭
   */
  inline Quaternion Conjugate() const {
    return Quaternion(coeffs_[0], -coeffs_[1], -coeffs_[2], -coeffs_[3]);
  }

  /**
   * @brief 获取当前四元数的逆
   * @return 四元数的逆
   */
  inline Quaternion Inverse() const {
    Scalar n2 = this->SquaredNorm();
    if (n2 > NumLimits<Scalar>::epsilon()) {
      return Quaternion(coeffs_[0] / n2,
          -coeffs_[1] / n2, -coeffs_[2] / n2, -coeffs_[3] / n2);
    } else {
      // return an invalid result to flag the error
      return Quaternion(Scalar(0), Scalar(0), Scalar(0), Scalar(0));
    }
  }

  /**
   * @brief 四元数之间的加法，重载+
   * @param[in] right +号右侧的四元数
   * @return 相加后的四元数
   */
  inline Quaternion operator +(const Quaternion& right) const {
    return Quaternion(
          coeffs_[0] + right.coeffs_[0],
          coeffs_[1] + right.coeffs_[1],
          coeffs_[2] + right.coeffs_[2],
          coeffs_[3] + right.coeffs_[3]);
  }

  /**
   * @brief 四元数之间的减法，重载-
   * @param[in] right -号右侧的四元数
   * @return 相减后的四元数
   */
  inline Quaternion operator -(const Quaternion& right) const {
    return Quaternion(
          coeffs_[0]-right.coeffs_[0],
          coeffs_[1]-right.coeffs_[1],
          coeffs_[2]-right.coeffs_[2],
          coeffs_[3]-right.coeffs_[3]);
  }

  /**
   * @brief 四元数之间的乘法，重载*号(注意，四元数乘法不服从交换率)
   * @param[in] right *号右侧的四元数
   * @return 相乘后的四元数
   */
  inline Quaternion operator *(const Quaternion& right) const {
    return Product(*this, right);
  }

  /**
   * @brief 四元数之间的加法，重载+=
   * @param[in] right +=号右侧的四元数
   */
  inline void operator +=(const Quaternion& right) {
    coeffs_[0] += right.coeffs_[0];
    coeffs_[1] += right.coeffs_[1];
    coeffs_[2] += right.coeffs_[2];
    coeffs_[3] += right.coeffs_[3];
  }

  /**
   * @brief 四元数之间的减法，重载-=
   * @param[in] right -号右侧的四元数
   */
  inline void operator -=(const Quaternion& right) {
    coeffs_[0] -= right.coeffs_[0];
    coeffs_[1] -= right.coeffs_[1];
    coeffs_[2] -= right.coeffs_[2];
    coeffs_[3] -= right.coeffs_[3];
  }

  /**
   * @brief 四元数之间的乘法，重载*=号(注意，四元数乘法不服从交换率)
   * @param[in] right *=号右侧的四元数
   */
  inline void operator *=(const Quaternion& right) {
    *this = Product(*this, right);
  }

private:
  /*
   * @brief 四元数之间的乘法(注意，四元数乘法不服从交换率)
   * @param[in] a 乘号左侧的四元数
   * @param[in] b 乘号右侧的四元数
   * @return 相乘后的四元数
   */
  Quaternion Product(const Quaternion& a, const Quaternion& b) const {
    return Quaternion(
          a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
          a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
          a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
          a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x());
  }

private:
  // 存储四元数的值
  Scalar coeffs_[4];
};


} // common
} // phoenix


#endif  // PHOENIX_COMMON_QUATERNION_H_

