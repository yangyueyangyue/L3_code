/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       geometry_utils.h
 * @brief      共通类基本几何算法
 * @details    定义了共通类基本几何算法
 *
 * @author     pengc
 * @date       2020.06.16
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/16  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_GEOMETRY_UTILS_H_
#define PHOENIX_COMMON_GEOMETRY_UTILS_H_

#include "geometry/vec2d.h"
#include "geometry/aabbox2d.h"
#include "geometry/obbox2d.h"
#include "geometry/sphere2d.h"
#include "math/math_utils.h"
#include "math/matrix.h"


namespace phoenix {
namespace common {


/**
 * @brief 构建二维平移复合矩阵
 * @param[in] tx 沿着x轴的移动量
 * @param[in] ty 沿着y轴的移动量
 * @param[out] mat_composite 平移后的复合矩阵
 */
template <typename T>
void Translate_2D(T tx, T ty,
                  Matrix<T, 3, 3>* mat_composite) {
  // Initialize translation matrix to identity
  Matrix<T, 3, 3> mat_transl;
  mat_transl.SetIdentity();

  mat_transl(0, 2) = tx;
  mat_transl(1, 2) = ty;

  // Concatenate matTransl with the composite matrix
  Matrix<T, 3, 3> mat_tmp;
  Mat_Mul(mat_transl, *mat_composite, mat_tmp);
  *mat_composite = mat_tmp;
}

/**
 * @brief 构建二维旋转复合矩阵
 * @param[in] pivot_pt 旋转中心点坐标
 * @param[in] theta 需要旋转的角度(弧度单位)
 * @param[out] mat_composite 旋转后的复合矩阵
 */
template <typename T>
void Rotate_2D(const Matrix<T, 2, 1>& pivot_pt,
               T theta,
               Matrix<T, 3, 3>* mat_composite) {
  T sin_theta = com_sin(theta);
  T cos_theta = com_cos(theta);
  // Initialize translation matrix to identity
  Matrix<T, 3, 3> mat_rot;
  mat_rot.SetIdentity();

  mat_rot(0, 0) = cos_theta;
  mat_rot(0, 1) = -sin_theta;
  mat_rot(0, 2) = pivot_pt(0) * (1.0f - cos_theta) + pivot_pt(1) * sin_theta;
  mat_rot(1, 0) = sin_theta;
  mat_rot(1, 1) = cos_theta;
  mat_rot(1, 2) = pivot_pt(1) * (1.0f - cos_theta) - pivot_pt(0) * sin_theta;

  // Concatenate matRot with the composite matrix
  Matrix<T, 3, 3> mat_tmp;
  Mat_Mul(mat_rot, *mat_composite, mat_tmp);
  *mat_composite = mat_tmp;
}

/**
 * @brief 使用二维复合矩阵变换二维点
 * @param[in] mat_composite 变换用的复合矩阵
 * @param[in&out] vert 待变换的二维点 & 变换后的二维点
 */
template <typename T>
void TransformVert_2D(const Matrix<T, 3, 3>& mat_composite,
                      Matrix<T, 2, 1>* vert) {
  T temp = mat_composite(0, 0) * (*vert)(0) + mat_composite(0, 1) *
      (*vert)(1) + mat_composite(0, 2);
  (*vert)(1) = mat_composite(1, 0) * (*vert)(0) + mat_composite(1, 1) *
      (*vert)(1) + mat_composite(1, 2);
  (*vert)(0) = temp;
}

/**
 * @brief 计算二维线段的航向角
 * @param[in] start 线段起点
 * @param[in] end 线段终点
 * @return 线段的航向角
 */
inline geo_var_t GetHeadingFromSegment(const Vec2d& start, const Vec2d& end) {
  return (com_atan2(end.y() - start.y(), end.x() - start.x()));
}


/**
 * @brief Returns 2 times the signed triangle area. The result is positive \n
 *        if abc is ccw, negative if abc is cw, zero if abc is degenerate.
 * @param a [in] corner of triangle abc
 * @param b [in] corner of triangle abc
 * @param c [in] corner of triangle abc
 * @return 2 times the signed triangle area
 *
 */
inline geo_var_t SignedTriArea_2D(
    const Vec2d& a, const Vec2d& b, const Vec2d& c) {
  return ((a(0)-c(0))*(b(1)-c(1)) - (a(1)-c(1))*(b(0)-c(0)));
}

bool OverlapTestSegToSeg_2D(
    const Vec2d& a, const Vec2d& b, const Vec2d& c,
    const Vec2d& d, Vec2d* p, geo_var_t* t);
bool OverlapTestAABBToAABB_2D(const AABBox2d& a, const AABBox2d& b);
geo_var_t DistAABBToAABB_2D(const AABBox2d& a, const AABBox2d& b);
bool OverlapTestSphereToSphere_2D(const Sphere2d& a, const Sphere2d& b);


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_GEOMETRY_UTILS_H_
