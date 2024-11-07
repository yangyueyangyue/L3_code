/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       geometry_utils.cc
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
#include "geometry/geometry_utils.h"


namespace phoenix {
namespace common {


bool OverlapTestSegToSeg_2D(
    const Vec2d& a, const Vec2d& b, const Vec2d& c,
    const Vec2d& d, Vec2d* p, geo_var_t* t) {
  // Sign of areas correspond to which side of ab points c and d are
  // Compute winding of abd (+ or -)
  geo_var_t a1 = SignedTriArea_2D(a, b, d);
  // To intersect, must have sign opposite of a1
  geo_var_t a2 = SignedTriArea_2D(a, b, c);
  // If c and d are on different sides of ab, areas have different signs
  if (a1 * a2 < 0.0F) {
    // Compute signs for a and b with respect to segment cd
    // Compute winding of cda (+ or -)
    geo_var_t a3 = SignedTriArea_2D(c, d, a);
    // Since area is constant a1 - a2 = a3 - a4, or a4 = a3 + a2 - a1
    geo_var_t a4 = a3 + a2 - a1;
    // Points a and b on different sides of cd if areas have different signs
    if (a3 * a4 < 0.0F) {
      // Segments intersect.
      // Find intersection point along L(t) = a + t * (b - a).
      // Given height h1 of an over cd and height h2 of b over cd,
      // t = h1 / (h1 - h2) = (b*h1/2) / (b*h1/2 - b*h2/2) = a3 / (a3 - a4),
      // where b (the base of the triangles cda and cdb, i.e., the length
      // of cd) cancels out.
      *t = a3 / (a3 - a4);
      *p = a + (*t) * (b - a);
      return (true);
    }
  }

  // Segments not intersecting (or collinear)
  *t = 0;
  *p = a;
  return (false);
}

bool OverlapTestAABBToAABB_2D(const AABBox2d& a, const AABBox2d& b) {
  // Exit with no intersection if separated along an axis
  if (a.max()(0) < b.min()(0) || a.min()(0) > b.max()(0)) return (false);
  if (a.max()(1) < b.min()(1) || a.min()(1) > b.max()(1)) return (false);
  // Overlapping on all axes means AABBs are intersecting
  return (true);
}

geo_var_t DistAABBToAABB_2D(const AABBox2d& a, const AABBox2d& b) {
  geo_var_t dist = 0.0F;

  if (a.max().x() < b.min().x()) {
    dist = b.min().x() - a.max().x();
  }
  if (a.min().x() > b.max().x()) {
    dist = Max(dist, a.min().x() - b.max().x());
  }
  if (a.max().y() < b.min().y()) {
    dist = Max(dist, b.min().y() - a.max().y());
  }
  if (a.min().y() > b.max().y()) {
    dist = Max(dist, a.min().y() - b.max().y());
  }

  return (dist);
}

bool OverlapTestSphereToSphere_2D(const Sphere2d& a, const Sphere2d& b) {
  // Calculate squared distance between centers
  Vec2d d = a.center() - b.center();
  geo_var_t sq_dist = d.LengthSquare();
  // Spheres intersect if squared distance is less than squared sum of radii
  geo_var_t radius_sum = a.radius() + b.radius();
  return sq_dist <= radius_sum * radius_sum;
}


}  // namespace common
}  // namespace phoenix
