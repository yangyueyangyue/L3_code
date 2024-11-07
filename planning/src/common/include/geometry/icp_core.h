/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       icp_core.h
 * @brief      ICP相关基础算法
 * @details    定义ICP相关基础算法
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
#ifndef PHOENIX_COMMON_ICP_CORE_H_
#define PHOENIX_COMMON_ICP_CORE_H_

#include "utils/macros.h"
#include "utils/log.h"
#include "math/matrix.h"


// Enable to output debug information
#define ENABLE_ICP_CORE_TRACE (0)


namespace phoenix {
namespace common {


/**
 * @brief 计算两个点集之间(必须是一一对应的)的从src_points到dst_points之间 \n
 *        的变换矩阵(旋转加平移，基于4元数的方法)
 * @param[in] points_size 点集中点的数量
 * @param[in] src_points 源点的集合，函数类，需要提供根据索引访问每个点的x、y、z \n
 *                       的坐标值的方法
 * @param[in] dst_points 目标点的集合，函数类，需要提供根据索引访问每个点的x、y、z \n
 *                       的坐标值的方法
 * @param[out] mat_rot 旋转矩阵
 * @param[out] vec_tran 平移向量
 * @return true - 成功，false - 失败
 *
 * @par Note:
 * @code
 *     FuncGetPoints中需要提供根据索引访问每个点的x、y、z的坐标值的方法
 *     例如:
 *      class FuncGetPoints {
 *      public:
 *        FuncGetPoints(const std::vector<Point3D>& pts)
 *          : points_(pts) {
 *        }
 *
 *        const Scalar& x(const Int32_t index) const {
 *          return (points_(index).x);
 *        }
 *        const Scalar& y(const Int32_t index) const {
 *          return (points_(index).y);
 *        }
 *        const Scalar& z(const Int32_t index) const {
 *          return (points_(index).z);
 *        }
 *
 *      private:
 *        const std::vector<Point3D>& points_;
 *      };
 * @endcode
 */
template <typename Scalar, typename FuncGetPoints>
bool CalcTfBtwCorrespondedPtsSetsQuaternionsBase(
    const Int32_t points_size,
    const FuncGetPoints& src_points,
    const FuncGetPoints& dst_points,
    Matrix<Scalar, 3, 3>* mat_rot,
    Matrix<Scalar, 3, 1>* vec_tran) {
#if ENABLE_ICP_CORE_TRACE
  std::cout << "### CalcTfBtwCorrespondedPtsSetsQuaternionsBase (Begin) ###"
            << std::endl;
#endif

  // Calculate the center of the two points sets respectively.
  Matrix<Scalar, 3, 1> src_center;
  Matrix<Scalar, 3, 1> dst_center;
  src_center.SetZeros();
  dst_center.SetZeros();
  for (Int32_t i = 0; i < points_size; ++i) {
    src_center(0) += src_points.x(i);
    src_center(1) += src_points.y(i);
    src_center(2) += src_points.z(i);
  }
  src_center(0) /= points_size;
  src_center(1) /= points_size;
  src_center(2) /= points_size;
  for (Int32_t i = 0; i < points_size; ++i) {
    dst_center(0) += dst_points.x(i);
    dst_center(1) += dst_points.y(i);
    dst_center(2) += dst_points.z(i);
  }
  dst_center(0) /= points_size;
  dst_center(1) /= points_size;
  dst_center(2) /= points_size;

#if ENABLE_ICP_CORE_TRACE
  std::cout << "src_center= " << src_center(0) << ", " << src_center(1)
            << ", " << src_center(2) << std::endl;
  std::cout << "dst_center= " << dst_center(0) << ", " << dst_center(1)
            << ", " << dst_center(2) << std::endl;
#endif

  Scalar sxx = 0;
  Scalar syy = 0;
  Scalar szz = 0;
  Scalar sxy = 0;
  Scalar sxz = 0;
  Scalar syz = 0;
  Scalar syx = 0;
  Scalar szx = 0;
  Scalar szy = 0;
  phoenix::common::Matrix<Scalar, 4, 4> mat_coef;
  Matrix<Scalar, 4, 4> mat_eigenvec;
  for (Int32_t i = 0; i < points_size; ++i) {
    sxx += (src_points.x(i)-src_center(0)) * (dst_points.x(i)-dst_center(0));
    syy += (src_points.y(i)-src_center(1)) * (dst_points.y(i)-dst_center(1));
    szz += (src_points.z(i)-src_center(2)) * (dst_points.z(i)-dst_center(2));

    sxy += (src_points.x(i)-src_center(0)) * (dst_points.y(i)-dst_center(1));
    sxz += (src_points.x(i)-src_center(0)) * (dst_points.z(i)-dst_center(2));
    syz += (src_points.y(i)-src_center(1)) * (dst_points.z(i)-dst_center(2));

    syx += (src_points.y(i)-src_center(1)) * (dst_points.x(i)-dst_center(0));
    szx += (src_points.z(i)-src_center(2)) * (dst_points.x(i)-dst_center(0));
    szy += (src_points.z(i)-src_center(2)) * (dst_points.y(i)-dst_center(1));
  }
  mat_coef(0, 0) = sxx + syy + szz;
  mat_coef(1, 0) = syz - szy;
  mat_coef(2, 0) = szx - sxz;
  mat_coef(3, 0) = sxy - syx;
  mat_coef(0, 1) = mat_coef(1, 0);
  mat_coef(1, 1) = sxx - syy - szz;
  mat_coef(2, 1) = sxy + syx;
  mat_coef(3, 1) = szx + sxz;
  mat_coef(0, 2) = mat_coef(2, 0);
  mat_coef(1, 2) = mat_coef(2, 1);
  mat_coef(2, 2) = syy - sxx - szz;
  mat_coef(3, 2) = syz + szy;
  mat_coef(0, 3) = mat_coef(3, 0);
  mat_coef(1, 3) = mat_coef(3, 1);
  mat_coef(2, 3) = mat_coef(3, 2);
  mat_coef(3, 3) = szz - sxx - syy;

#if ENABLE_ICP_CORE_TRACE
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  // Calculate the eigenvalues and eigenvectors of the coefficient matrix
  Mat_CalcEigenValueOfSymmetricMatrix(mat_coef, mat_eigenvec);

#if ENABLE_ICP_CORE_TRACE
  std::cout << "After eigen decomposition:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_eigenvec=\n" << mat_eigenvec << std::endl;
#endif

  // Get the largest positive eigenvalue
  Scalar max_eigenval = mat_coef(0, 0);
  Int32_t max_eigenval_index = 0;
  for (Int32_t i = 1; i < 4; ++i) {
    if (mat_coef(i, i) > max_eigenval) {
      max_eigenval = mat_coef(i, i);
      max_eigenval_index = i;
    }
  }

#if ENABLE_ICP_CORE_TRACE
  Scalar angle = com_acos(mat_eigenvec(0, max_eigenval_index));
  Scalar sin_angle = com_sin(angle);
  std::cout << "rotation angle = " << com_rad2deg(Scalar(2)*angle) << std::endl;
  std::cout << "rotation axis = "
            << mat_eigenvec(1, max_eigenval_index) / sin_angle
            << ", " << mat_eigenvec(2, max_eigenval_index) / sin_angle
            << ", " << mat_eigenvec(3, max_eigenval_index) / sin_angle
            << std::endl;
#endif

  // Calculate rotation matrix and translation vector
  Scalar q00 = Square(mat_eigenvec(0, max_eigenval_index));
  Scalar q11 = Square(mat_eigenvec(1, max_eigenval_index));
  Scalar q22 = Square(mat_eigenvec(2, max_eigenval_index));
  Scalar q33 = Square(mat_eigenvec(3, max_eigenval_index));
  Scalar q01 = mat_eigenvec(0, max_eigenval_index) *
      mat_eigenvec(1, max_eigenval_index);
  Scalar q02 = mat_eigenvec(0, max_eigenval_index) *
      mat_eigenvec(2, max_eigenval_index);
  Scalar q03 = mat_eigenvec(0, max_eigenval_index) *
      mat_eigenvec(3, max_eigenval_index);
  Scalar q12 = mat_eigenvec(1, max_eigenval_index) *
      mat_eigenvec(2, max_eigenval_index);
  Scalar q13 = mat_eigenvec(1, max_eigenval_index) *
      mat_eigenvec(3, max_eigenval_index);
  Scalar q23 = mat_eigenvec(2, max_eigenval_index) *
      mat_eigenvec(3, max_eigenval_index);

  (*mat_rot)(0, 0) = q00 + q11 - q22 - q33;
  (*mat_rot)(1, 0) = Scalar(2) * (q12 + q03);
  (*mat_rot)(2, 0) = Scalar(2) * (q13 - q02);
  (*mat_rot)(0, 1) = Scalar(2) * (q12 - q03);
  (*mat_rot)(1, 1) = q00 - q11 + q22 - q33;
  (*mat_rot)(2, 1) = Scalar(2) * (q23 + q01);
  (*mat_rot)(0, 2) = Scalar(2) * (q13 + q02);
  (*mat_rot)(1, 2) = Scalar(2) * (q23 - q01);
  (*mat_rot)(2, 2) = q00 - q11 - q22 + q33;

  Mat_Mul(*mat_rot, src_center, *vec_tran);
  (*vec_tran)(0) = dst_center(0) - (*vec_tran)(0);
  (*vec_tran)(1) = dst_center(1) - (*vec_tran)(1);
  (*vec_tran)(2) = dst_center(2) - (*vec_tran)(2);

  if (max_eigenval < Scalar(0)) {
    LOG_ERR << "The largest eigenvalue is negative, "
               "which is expected to be positive.";
    return false;
  }

#if ENABLE_ICP_CORE_TRACE
  std::cout << "mat_rot=\n" << (*mat_rot) << std::endl;
  std::cout << "vec_tran= " << (*vec_tran)(0) << ", " << (*vec_tran)(1)
            << ", " << (*vec_tran)(2) << std::endl;
  std::cout << "### CalcTfBtwCorrespondedPtsSetsQuaternionsBase (End) ###"
            << std::endl;
#endif

  return true;
}

/**
 * @brief 计算两个点集(2D)之间(必须是一一对应的)的从src_points到dst_points之间 \n
 *        的变换矩阵(旋转加平移，基于4元数的方法)
 * @param[in] points_size 点集中点的数量
 * @param[in] src_points 源点的集合，函数类，需要提供根据索引访问每个点的x、y、z \n
 *                       的坐标值的方法
 * @param[in] dst_points 目标点的集合，函数类，需要提供根据索引访问每个点的x、y、z \n
 *                       的坐标值的方法
 * @param[out] mat_rot 旋转矩阵
 * @param[out] vec_tran 平移向量
 * @return true - 成功，false - 失败
 *
 * @par Note:
 * @code
 *     FuncGetPoints中需要提供根据索引访问每个点的x、y、z的坐标值的方法
 *     例如:
 *      class FuncGetPoints {
 *      public:
 *        FuncGetPoints(const std::vector<Point3D>& pts)
 *          : points_(pts) {
 *        }
 *
 *        const Scalar& x(const Int32_t index) const {
 *          return (points_(index).x);
 *        }
 *        const Scalar& y(const Int32_t index) const {
 *          return (points_(index).y);
 *        }
 *        const Scalar& z(const Int32_t index) const {
 *          return (points_(index).z);
 *        }
 *
 *      private:
 *        const std::vector<Point3D>& points_;
 *      };
 * @endcode
 */
template <typename Scalar, typename FuncGetPoints>
bool CalcTfBtwCorrespondedPtsSetsQuaternionsBase_2D(
    const Int32_t points_size,
    const FuncGetPoints& src_points,
    const FuncGetPoints& dst_points,
    Matrix<Scalar, 3, 3>* mat_rot,
    Matrix<Scalar, 3, 1>* vec_tran) {
#if ENABLE_ICP_CORE_TRACE
  std::cout << "### CalcTfBtwCorrespondedPtsSetsQuaternionsBase_2D (Begin) ###"
            << std::endl;
#endif

  const Scalar eps = NumLimits<Scalar>::epsilon();

  // Calculate the center of the two points sets respectively.
  Matrix<Scalar, 3, 1> src_center;
  Matrix<Scalar, 3, 1> dst_center;
  src_center.SetZeros();
  dst_center.SetZeros();
  for (Int32_t i = 0; i < points_size; ++i) {
    src_center(0) += src_points.x(i);
    src_center(1) += src_points.y(i);
  }
  src_center(0) /= points_size;
  src_center(1) /= points_size;
  for (Int32_t i = 0; i < points_size; ++i) {
    dst_center(0) += dst_points.x(i);
    dst_center(1) += dst_points.y(i);
  }
  dst_center(0) /= points_size;
  dst_center(1) /= points_size;

#if ENABLE_ICP_CORE_TRACE
  std::cout << "src_center= " << src_center(0) << ", " << src_center(1)
            << ", " << src_center(2) << std::endl;
  std::cout << "dst_center= " << dst_center(0) << ", " << dst_center(1)
            << ", " << dst_center(2) << std::endl;
#endif

  Scalar sxx = 0;
  Scalar syy = 0;
  Scalar sxy = 0;
  Scalar syx = 0;
  phoenix::common::Matrix<Scalar, 4, 4> mat_coef;
  Matrix<Scalar, 4, 4> mat_eigenvec;
  for (Int32_t i = 0; i < points_size; ++i) {
    sxx += (src_points.x(i)-src_center(0)) * (dst_points.x(i)-dst_center(0));
    syy += (src_points.y(i)-src_center(1)) * (dst_points.y(i)-dst_center(1));
    sxy += (src_points.x(i)-src_center(0)) * (dst_points.y(i)-dst_center(1));
    syx += (src_points.y(i)-src_center(1)) * (dst_points.x(i)-dst_center(0));
  }
  mat_coef(0, 0) = sxx + syy/* + szz*/;
  mat_coef(1, 0) = 0/*syz - szy*/;
  mat_coef(2, 0) = 0/*szx - sxz*/;
  mat_coef(3, 0) = sxy - syx;
  mat_coef(0, 1) = mat_coef(1, 0);
  mat_coef(1, 1) = sxx - syy/* - szz*/;
  mat_coef(2, 1) = sxy + syx;
  mat_coef(3, 1) = 0/*szx + sxz*/;
  mat_coef(0, 2) = mat_coef(2, 0);
  mat_coef(1, 2) = mat_coef(2, 1);
  mat_coef(2, 2) = syy - sxx/* - szz*/;
  mat_coef(3, 2) = 0/*syz + szy*/;
  mat_coef(0, 3) = mat_coef(3, 0);
  mat_coef(1, 3) = mat_coef(3, 1);
  mat_coef(2, 3) = mat_coef(3, 2);
  mat_coef(3, 3) = /*szz*/ -sxx - syy;

#if ENABLE_ICP_CORE_TRACE
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
#endif

  // Calculate the eigenvalues and eigenvectors of the coefficient matrix
  Mat_CalcEigenValueOfSymmetricMatrix(mat_coef, mat_eigenvec);

#if ENABLE_ICP_CORE_TRACE
  std::cout << "After eigen decomposition:" << std::endl;
  std::cout << "mat_coef=\n" << mat_coef << std::endl;
  std::cout << "mat_eigenvec=\n" << mat_eigenvec << std::endl;
#endif

  // Get the largest positive eigenvalue
  bool find_z_axis = false;
  Scalar max_eigenval = mat_coef(0, 0);
  Int32_t max_eigenval_index = 0;
  for (Int32_t i = 0; i < 4; ++i) {
    if ((com_abs(mat_eigenvec(1, i)) <= eps) &&
        (com_abs(mat_eigenvec(2, i)) <= eps)) {
      max_eigenval = mat_coef(0, i);
      max_eigenval_index = i;
      find_z_axis = true;
    }
  }
  for (Int32_t i = 0; i < 4; ++i) {
    if (find_z_axis) {
      if ((mat_coef(i, i) > max_eigenval) &&
          (com_abs(mat_eigenvec(1, i)) <= eps) &&
          (com_abs(mat_eigenvec(2, i)) <= eps)) {
        max_eigenval = mat_coef(i, i);
        max_eigenval_index = i;
      }
    } else {
      if (mat_coef(i, i) > max_eigenval) {
        max_eigenval = mat_coef(i, i);
        max_eigenval_index = i;
      }
    }
  }

#if ENABLE_ICP_CORE_TRACE
  Scalar angle = com_acos(mat_eigenvec(0, max_eigenval_index));
  Scalar sin_angle = com_sin(angle);
  std::cout << "rotation angle = " << com_rad2deg(Scalar(2)*angle) << std::endl;
  std::cout << "rotation axis = "
            << mat_eigenvec(1, max_eigenval_index) / sin_angle
            << ", " << mat_eigenvec(2, max_eigenval_index) / sin_angle
            << ", " << mat_eigenvec(3, max_eigenval_index) / sin_angle
            << std::endl;
#endif

  // Calculate rotation matrix and translation vector
  Scalar q00 = Square(mat_eigenvec(0, max_eigenval_index));
  Scalar q11 = Square(mat_eigenvec(1, max_eigenval_index));
  Scalar q22 = Square(mat_eigenvec(2, max_eigenval_index));
  Scalar q33 = Square(mat_eigenvec(3, max_eigenval_index));
  Scalar q01 = mat_eigenvec(0, max_eigenval_index) *
      mat_eigenvec(1, max_eigenval_index);
  Scalar q02 = mat_eigenvec(0, max_eigenval_index) *
      mat_eigenvec(2, max_eigenval_index);
  Scalar q03 = mat_eigenvec(0, max_eigenval_index) *
      mat_eigenvec(3, max_eigenval_index);
  Scalar q12 = mat_eigenvec(1, max_eigenval_index) *
      mat_eigenvec(2, max_eigenval_index);
  Scalar q13 = mat_eigenvec(1, max_eigenval_index) *
      mat_eigenvec(3, max_eigenval_index);
  Scalar q23 = mat_eigenvec(2, max_eigenval_index) *
      mat_eigenvec(3, max_eigenval_index);

  (*mat_rot)(0, 0) = q00 + q11 - q22 - q33;
  (*mat_rot)(1, 0) = Scalar(2) * (q12 + q03);
  (*mat_rot)(2, 0) = Scalar(2) * (q13 - q02);
  (*mat_rot)(0, 1) = Scalar(2) * (q12 - q03);
  (*mat_rot)(1, 1) = q00 - q11 + q22 - q33;
  (*mat_rot)(2, 1) = Scalar(2) * (q23 + q01);
  (*mat_rot)(0, 2) = Scalar(2) * (q13 + q02);
  (*mat_rot)(1, 2) = Scalar(2) * (q23 - q01);
  (*mat_rot)(2, 2) = q00 - q11 - q22 + q33;

  Mat_Mul(*mat_rot, src_center, *vec_tran);
  (*vec_tran)(0) = dst_center(0) - (*vec_tran)(0);
  (*vec_tran)(1) = dst_center(1) - (*vec_tran)(1);
  (*vec_tran)(2) = dst_center(2) - (*vec_tran)(2);

  if (max_eigenval < Scalar(0)) {
    LOG_ERR << "The largest eigenvalue is negative, "
               "which is expected to be positive.";
    return false;
  }
  if ((com_abs(mat_eigenvec(1, max_eigenval_index)) > eps) ||
      (com_abs(mat_eigenvec(2, max_eigenval_index)) > eps)) {
    LOG_ERR << "The rotation axis is not z axis.";
    return false;
  }

#if ENABLE_ICP_CORE_TRACE
  std::cout << "mat_rot=\n" << (*mat_rot) << std::endl;
  std::cout << "vec_tran= " << (*vec_tran)(0) << ", " << (*vec_tran)(1)
            << ", " << (*vec_tran)(2) << std::endl;
  std::cout << "### CalcTfBtwCorrespondedPtsSetsQuaternionsBase_2D (End) ###"
            << std::endl;
#endif

  return true;
}


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_ICP_CORE_H_

