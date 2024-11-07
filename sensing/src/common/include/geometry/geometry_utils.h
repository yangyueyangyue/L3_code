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
 * @brief 从OBB包围盒构造AABB包围盒
 * @param[in] obb OBB包围盒
 * @param[out] aabb AABB包围盒
 */
void ConstructAABBoxFromOBBox_2D(const OBBox2d& obb, AABBox2d* const aabb);

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

/**
 * @brief 判断点P与有向线段AB之间的方位关系
 * @param[in] a 线段起点
 * @param[in] b 线段终点
 * @param[in] p 待判断的点
 * @return 0 ~ 三个点共线 \n
 *         1 ~ 点在线段左边 \n
 *         -1 ~ 点在线段右边
 */
Int32_t Orient_2D(const Vec2d& a, const Vec2d& b, const Vec2d& p);

/**
 * @brief 计算线段AB上与点P的最近点
 * @param[in] a 线段起点
 * @param[in] b 线段终点
 * @param[in] p 待判断的点
 * @param[out] d 线段AB上与P点最近的点
 * @param[out] t 线段AB上与P点最近的点在线段AB上的比例值(范围[0, 1])
 * @par Note:
 * @code
 *     Given segment AB and point P, computes closest point D on AB.
 *     Also returns t for the parametric position of D, D(t) = A + t*(B - A)
 * @endcode
 */
void ClosestPtPointToSeg_2D(const Vec2d& a, const Vec2d& b, const Vec2d& p,
    Vec2d* d, geo_var_t* t);

/**
 * @brief 计算直线AB上与点P的最近点
 * @param[in] a 直线AB上的点A
 * @param[in] b 直线AB上的点B
 * @param[in] p 待判断的点
 * @param[out] d 直线AB上与P点最近的点
 * @param[out] t 直线AB上与P点最近的点在线段AB上的比例值
 * @par Note:
 * @code
 *     Given line AB and point P, computes closest point D on AB.
 *     Also returns t for the parametric position of D, D(t) = A + t*(B - A)
 * @endcode
 */
void ClosestPtPointToLine_2D(const Vec2d& a, const Vec2d& b, const Vec2d& p,
    Vec2d* d, geo_var_t* t);

/**
 * @brief 计算点P与线段AB之间的平方距离
 * @param[in] a 直线AB上的点A
 * @param[in] b 直线AB上的点B
 * @param[in] p 待判断的点
 * @return 点P与线段AB之间的平方距离
 * @par Note:
 * @code
 *     Returns the squared distance between point P and segment AB
 *     The squared distance between a point P and a segment AB can be directly
 *     computed without explicitly computing the point D on AB closest to P.
 * @endcode
 */
geo_var_t SqDistPointToSeg_2D(const Vec2d& a, const Vec2d& b, const Vec2d& p);

/**
 * @brief 计算AABB包围盒B上与点P最近的点
 * @param[in] b AABB包围盒B
 * @param[in] p 待判断的点P
 * @param[out] q 最近的点
 * @par Note:
 * @code
 *     Given point p, return the point q on or in AABB b that is closest to p
 *     There are four cases to consider for verifying that clamping gives the
 *     desired result. If P is inside B, the clamped point is P itself, which
 *     is also the point in B closest to P. If P is in a face Voronoi region
 *     of B, the clamping operation will bring P to that face of B. The clamping
 *     corresponds to an orthogonal projection of P onto B and must therefore
 *     result in the closest point on B. When P is in a vertex Voronoi region
 *     of B, clamping P gives the vertex as a result, which again is the closest
 *     point on B. Finally, when P is in an edge Voronoi region, clamping P
 *     corresponds to an orthogonal projection onto the edge, which also must
 *     be the closest point on B to P.
 * @endcode
 */
void ClosestPtPointToAABB_2D(const AABBox2d& b, const Vec2d& p, Vec2d* q);

/**
 * @brief 计算点P与AABB包围盒之间的平方距离
 * @param[in] b AABB包围盒B
 * @param[in] p 待判断的点P
 * @return 点P与AABB包围盒之间的平方距离
 * @par Note:
 * @code
 *     Computes the square distance between a point p and an AABB b
 *     When the point Q on an AABB B closest to a given point P is computed
 *     only to determine the distance between P and Q, the distance can be
 *     calculated without explicitly obtaining Q.
 * @endcode
 */
geo_var_t SqDistPointToAABB_2D(const AABBox2d& b, const Vec2d& p);

/**
 * @brief 计算OBB包围盒B上与点P最近的点
 * @param[in] b OBB包围盒B
 * @param[in] p 待判断的点P
 * @param[out] q 最近的点
 * @par Note:
 * @code
 *     Given point p, return point q on (or in) OBB b, closest to p
 *     To compute the point R on (or in) B closest to P, the same approach as
 *     used for an AABB can be applied by expressing P in the OBB coordinate
 *     system as Q, clamping Q to the extents e0, e1, and e2, and reexpressing
 *     Q in world coordinates.
 * @endcode
 */
void ClosestPtPointToOBB_2D(const OBBox2d& b, const Vec2d& p, Vec2d* q);

/**
 * @brief 计算点P与OBB包围盒之间的平方距离
 * @param[in] b OBB包围盒B
 * @param[in] p 待判断的点P
 * @return 点P与OBB包围盒之间的平方距离
 * @par Note:
 * @code
 *     Computes the square distance between point p and OBB b.
 *     If only the squared distance and not the closest point is needed,
 *     this code can be further simplified. By projecting the vector v from
 *     the center of B to P onto each of the three OBB axes, the distance d
 *     from P to the box center along that axis is obtained. Because the axes
 *     are orthogonal, any excess amount that d is beyond the extent of the
 *     box for a given axis can be computed, squared, and added to the total
 *     squared distance of P independently of the other two axes.
 * @endcode
 */
geo_var_t SqDistPointToOBB_2D(const OBBox2d& b, const Vec2d& p);

/**
 * @brief 计算线段P1Q1与线段P2Q2之间的最近点，及最近点间的平方距离
 * @param[in] p1 线段P1Q1的起点
 * @param[in] q1 线段P1Q1的终点
 * @param[in] p2 线段P2Q2的起点
 * @param[in] q2 线段P2Q2的终点
 * @param[out] c1 线段P1Q1上的最近点
 * @param[out] c2 线段P2Q2上的最近点
 * @param[out] s 线段P1Q1上的最近点在线段P1Q1上的比例
 * @param[out] t 线段P2Q2上的最近点在线段P2Q2上的比例
 * @return 线段P1Q1与线段P2Q2之间的平方距离
 * @par Note:
 * @code
 *     Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
 *     S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
 *     distance between between S1(s) and S2(t).
 * @endcode
 */
geo_var_t ClosestPtSegToSeg_2D(
    const Vec2d& p1, const Vec2d& q1,
    const Vec2d& p2, const Vec2d& q2,
    Vec2d* c1, Vec2d* c2, geo_var_t* s, geo_var_t* t);

/**
 * @brief 计算线段P1Q1与线段P2Q2之间的平方距离
 * @param[in] p1 线段P1Q1的起点
 * @param[in] q1 线段P1Q1的终点
 * @param[in] p2 线段P2Q2的起点
 * @param[in] q2 线段P2Q2的终点
 * @return 线段P1Q1与线段P2Q2之间的平方距离
 */
geo_var_t SqDistSegToSeg_2D(const Vec2d& p1, const Vec2d& q1,
    const Vec2d& p2, const Vec2d& q2);


/**
 * @brief 判断线段AB与线段CD是否相交
 * @param[in] a 线段AB的起点
 * @param[in] b 线段AB的终点
 * @param[in] c 线段CD的起点
 * @param[in] d 线段CD的终点
 * @return true ~ 线段AB与线段CD相交 \n
 *         false ~ 线段AB与线段CD不相交，或者某条线段的端点在另一条线段上
 * @par Note:
 * @code
 *     Test if segments ab and cd overlap.
 * @endcode
 */
bool OverlapTestSegToSeg_2D(const Vec2d& a, const Vec2d& b,
    const Vec2d& c, const Vec2d& d);

/**
 * @brief 判断线段AB与线段CD是否相交
 * @param[in] a 线段AB的起点
 * @param[in] b 线段AB的终点
 * @param[in] c 线段CD的起点
 * @param[in] d 线段CD的终点
 * @param[out] p 交点
 * @param[out] t 交点在线段AB上的比例
 * @return true ~ 线段AB与线段CD相交 \n
 *         false ~ 线段AB与线段CD不相交，或者某条线段的端点在另一条线段上
 * @par Note:
 * @code
 *     Test if segments ab and cd overlap. If they do, compute and return
 *     intersection t value along ab and intersection position p
 * @endcode
 */
bool OverlapTestSegToSeg_2D(const Vec2d& a, const Vec2d& b,
    const Vec2d& c, const Vec2d& d, Vec2d* p, geo_var_t* t);

/**
 * @brief 判断包围盒A与包围盒B是否相交(AABB)
 * @param[in] a 包围盒A
 * @param[in] b 包围盒B
 * @return true ~ 相交(点或线的接触也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Overlap tests between AABBs.
 *     Two AABBs only overlap if they overlap on all axes, where their
 *     extent along each dimension is seen as an interval on the 
 *     corresponding axis.
 * @endcode
 */
bool OverlapTestAABBToAABB_2D(const AABBox2d& a, const AABBox2d& b);

/**
 * @brief 计算包围盒A与包围盒B之间的距离(AABB)
 * @param[in] a 包围盒A
 * @param[in] b 包围盒B
 * @return 两个包围盒(AABB)之间的距离
 * @par Note:
 * @code
 *     The returned distance between two AABBs is approximate, \n
 *     and may be smaller than the exact one.
 * @endcode
 */
geo_var_t DistAABBToAABB_2D(const AABBox2d& a, const AABBox2d& b);

/**
 * @brief 判断圆A与圆B是否相交
 * @param[in] a 圆A
 * @param[in] b 圆B
 * @return true ~ 相交(接触也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Overlap tests between two spheres.
 *     The Euclidean distance between the sphere centers is computed and
 *     compared against the sum of the sphere radii.
 * @endcode
 */
bool OverlapTestSphereToSphere_2D(const Sphere2d& a, const Sphere2d& b);

/**
 * @brief 判断包围盒A与包围盒B是否相交(OBB)
 * @param[in] a 包围盒A
 * @param[in] b 包围盒B
 * @return true ~ 相交(接触也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Overlap tests between OBBs.
 *     Be implemented in terms of what is known as the separating axis test.
 * @endcode
 */
bool OverlapTestOBBToOBB_2D(const OBBox2d& a, const OBBox2d& b);

/**
 * @brief 计算包围盒A与包围盒B之间的距离(OBB)
 * @param[in] a 包围盒A
 * @param[in] b 包围盒B
 * @return 两个包围盒(OBB)之间的距离
 * @par Note:
 * @code
 *     Overlap tests between OBBs.
 *     Be implemented in terms of what is known as the separating axis test.
 *     The returned distance between two OBBs is approximate, \n
 *     and may be smaller than the exact one.
 * @endcode
 */
geo_var_t DistOBBToOBB_2D(const OBBox2d& a, const OBBox2d& b);

/**
 * @brief 判断射线P与圆S是否相交，如果相交则返回交点Q及其射线参数t
 * @param[in] p 射线端点P
 * @param[in] d 射线单位方向向量
 * @param[in] s 圆S
 * @param[out] q 交点
 * @param[out] t 交点在射线上的参数t
 * @return true ~ 相交(接触也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Intersects ray r = p + td, |d| = 1, with sphere s and, if intersecting,
 *     returns t value of intersection and intersection point q.
 * @endcode
 */
bool IntersectRayToSphere_2D(const Vec2d& p, const Vec2d& d,
    const Sphere2d& s, Vec2d* q, geo_var_t* t);

/**
 * @brief 判断射线P与圆S是否相交
 * @param[in] p 射线端点P
 * @param[in] d 射线单位方向向量
 * @param[in] s 圆S
 * @return true ~ 相交(接触也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Test if ray r = p + td intersects sphere s.
 * @endcode
 */
bool OverlapTestRayToSphere_2D(const Vec2d& p, const Vec2d& d,
    const Sphere2d& s);

/**
 * @brief 判断射线P与AABB包围盒A是否相交，如果相交则返回交点Q及其射线参数t
 * @param[in] p 射线端点P
 * @param[in] d 射线单位方向向量
 * @param[in] a 包围盒A
 * @param[out] q 交点
 * @param[out] tmin 交点在射线上的参数t
 * @return true ~ 相交(接触也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Intersect ray R(t) = p + t*d against AABB a. When intersecting,
 *     return intersection distance tmin and point q of intersection.
 *     接触也算作相交,射线端点在包围盒内部也算作相交,交点为AABB框体上的较小点
 * @endcode
 */
bool IntersectRayToAABB_2D(const Vec2d& p, const Vec2d& d, const AABBox2d& a,
    Vec2d* q, geo_var_t* tmin);

/**
 * @brief 判断线段P与AABB包围盒B是否相交
 * @param[in] p0 线段起点
 * @param[in] p1 线段终点
 * @param[in] b 包围盒B
 * @return true ~ 相交(接触也算作相交,在包围盒内部也算作相交) \n
 *         false ~ 不相交
 * @par Note:
 * @code
 *     Test if segment specified by points p0 and p1 intersects AABB b.
 * @endcode
 */
bool OverlapTestSegToAABB_2D(const Vec2d& p0, const Vec2d& p1,
    const AABBox2d& b);

/**
 * @brief 判断圆S与AABB包围盒B是否相交
 * @param[in] s 圆S
 * @param[in] b 包围盒B
 * @return true ~ 相交(接触也算作相交,在包围盒内部也算作相交) \n
 *         false ~ 不相交
 */
bool OverlapTestSphereToAABB_2D(const Sphere2d& s, const AABBox2d& b);

/**
 * @brief 判断圆S与AABB包围盒B是否相交
 * @param[in] s 圆S
 * @param[in] b 包围盒B
 * @param[out] p 包围盒B上与圆心最近的点
 * @return true ~ 相交(接触也算作相交) \n
 *         false ~ 不相交
 */
bool OverlapTestSphereToAABB_2D(const Sphere2d& s, const AABBox2d& b, Vec2d* p);

/**
 * @brief 判断圆S与OBB包围盒B是否相交
 * @param[in] s 圆S
 * @param[in] b 包围盒B
 * @return true ~ 相交(接触也算作相交,在包围盒内部也算作相交) \n
 *         false ~ 不相交
 */
bool OverlapTestSphereToOBB_2D(const Sphere2d& s, const OBBox2d& b);

/**
 * @brief 判断圆S与OBB包围盒B是否相交
 * @param[in] s 圆S
 * @param[in] b 包围盒B
 * @param[out] p 包围盒B上与圆心最近的点
 * @return true ~ 相交(接触也算作相交,在包围盒内部也算作相交) \n
 *         false ~ 不相交
 */
bool OverlapTestSphereToOBB_2D(const Sphere2d& s, const OBBox2d& b, Vec2d* p);


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_GEOMETRY_UTILS_H_
