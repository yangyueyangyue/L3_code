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


void ConstructAABBoxFromOBBox_2D(const OBBox2d& obb, AABBox2d* const aabb) {
  common::Vec2d delta_x = obb.extents().x() * obb.unit_direction_x();
  common::Vec2d delta_y = obb.extents().y() * obb.unit_direction_y();
  common::Vec2d corner[4];
  corner[0] = obb.center() + delta_x + delta_y;
  corner[1] = obb.center() - delta_x + delta_y;
  corner[2] = obb.center() - delta_x - delta_y;
  corner[3] = obb.center() + delta_x - delta_y;

  geo_var_t min_x = corner[0].x();
  geo_var_t max_x = corner[0].x();
  geo_var_t min_y = corner[0].y();
  geo_var_t max_y = corner[0].y();
  for (Int32_t i = 1; i < 4; ++i) {
    min_x = common::Min(min_x, corner[i].x());
    max_x = common::Max(max_x, corner[i].x());
    min_y = common::Min(min_y, corner[i].y());
    max_y = common::Max(max_y, corner[i].y());
  }

  aabb->min().set_x(min_x);
  aabb->min().set_y(min_y);
  aabb->max().set_x(max_x);
  aabb->max().set_y(max_y);
}


Int32_t Orient_2D(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
  geo_var_t orient = (a.x() - p.x()) * (b.y() - p.y())
      - (a.y() - p.y()) * (b.x() - p.x());

  if (std::abs(orient) < kGeometryEpsilon) {
    // the three points are collinear
    return (0);
  } else if (orient > 0.0F) {
    // p lies to the left of the directed line AB
    // the triangle ABC is oriented counterclockwise
    return (1);
  } else {
    // p lies to the right of the directed line AB
    // the triangle ABC is oriented clockwise
    return (-1);
  }
}

void ClosestPtPointToSeg_2D(
    const Vec2d& a, const Vec2d& b, const Vec2d& p, Vec2d* d, geo_var_t* t) {
  Vec2d v_ab = b - a;
  // Project c onto ab, but deferring divide by Dot(ab, ab)
  *t = v_ab.InnerProd(p - a);
  if (*t <= 0.0F) {
    // c projects outside the [a,b] interval, on the a side; clamp to a
    *t = 0.0F;
    *d = a;
  } else {
    // Always nonnegative since denom = ||ab||^2
    geo_var_t denom = v_ab.LengthSquare();
    if (*t >= denom) {
      // c projects outside the [a,b] interval, on the b side; clamp to b
      *t = 1.0F;
      *d = b;
    } else {
      // c projects inside the [a,b] interval; must do deferred divide now
      *t = (*t) / denom;
      *d = a + (*t) * v_ab;
    }
  }
}

void ClosestPtPointToLine_2D(
    const Vec2d& a, const Vec2d& b, const Vec2d& p, Vec2d* d, geo_var_t* t) {
  Vec2d v_ab = b - a;
  // Project c onto ab, but deferring divide by Dot(ab, ab)
  *t = v_ab.InnerProd(p - a);

  // Always nonnegative since denom = ||ab||^2
  geo_var_t denom = v_ab.LengthSquare();
  if (denom < kGeometryEpsilon) {
    *d = a;
    *t = 0;
    return;
  }

  *t = (*t) / denom;
  *d = a + (*t) * v_ab;
}

geo_var_t SqDistPointToSeg_2D(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
  Vec2d v_ab = b - a;
  Vec2d v_ap = p - a;
  Vec2d v_bp = p - b;
  geo_var_t e = v_ap.InnerProd(v_ab);
  // Handle cases where p projects outside ab
  if (e <= 0) {
    return (v_ap.LengthSquare());
  }

  geo_var_t f = v_ab.LengthSquare();
  if (e >= f) {
    return (v_bp.LengthSquare());
  }

  // Handle cases where c projects onto ab
  return (com_abs(v_ap.LengthSquare() - e * e / f));
}

void ClosestPtPointToAABB_2D(const AABBox2d& b, const Vec2d& p, Vec2d* q) {
  // For each coordinate axis, if the point coordinate value is
  // outside box, clamp it to the box, else keep it as is
  for (Uint32_t i = 0; i < 2; i++) {
    geo_var_t v = p(i);
    v = Max<geo_var_t>(v, b.min()(i));
    v = Min<geo_var_t>(v, b.max()(i));
    (*q)(i) = v;
  }
}

geo_var_t SqDistPointToAABB_2D(const AABBox2d& b, const Vec2d& p) {
  geo_var_t sq_dist = 0.0F;
  for (Uint32_t i = 0; i < 2; i++) {
    // For each axis count any excess distance outside box extents
    geo_var_t v = p(i);
    if (v < b.min()(i)) sq_dist += (b.min()(i) - v) * (b.min()(i) - v);
    if (v > b.max()(i)) sq_dist += (v - b.max()(i)) * (v - b.max()(i));
  }

  return (sq_dist);
}

void ClosestPtPointToOBB_2D(const OBBox2d& b, const Vec2d& p, Vec2d* q) {
  Vec2d d = p - b.center();
  // Start result at center of box; make steps from there
  *q = b.center();
  // For each OBB axis...
  for (Uint32_t i = 0; i < 2; ++i) {
    // ...project d onto that axis to get the distance
    // along the axis of d from the box center
    geo_var_t dist = d.InnerProd(b.unit_direction(static_cast<Int32_t>(i)));
    // If distance farther than the box extents, clamp to the box
    if (dist > b.extents()(i)) dist = b.extents()(i);
    if (dist < -b.extents()(i)) dist = -b.extents()(i);
    // Step that distance along the axis to get world coordinate
    *q += dist * b.unit_direction(static_cast<Int32_t>(i));
  }
}

geo_var_t SqDistPointToOBB_2D(const OBBox2d& b, const Vec2d& p) {
  Vec2d d = p - b.center();
  geo_var_t sq_dist = 0.0F;
  // For each OBB axis...
  for (Uint32_t i = 0; i < 2; ++i) {
    // Project vector from box center to p on each axis, getting the distance
    // of p along that axis, and count any excess distance outside box extents
    geo_var_t dist = d.InnerProd(b.unit_direction(static_cast<Int32_t>(i)));
    geo_var_t excess = 0.0F;
    // If distance farther than the box extents, clamp to the box
    if (dist > b.extents()(i)) {
      excess = dist - b.extents()(i);
    } else if (dist < -b.extents()(i)) {
      excess = dist + b.extents()(i);
    }
    sq_dist += excess * excess;
  }

  return (sq_dist);
}

geo_var_t ClosestPtSegToSeg_2D(
    const Vec2d& p1, const Vec2d& q1, const Vec2d& p2, const Vec2d& q2,
    Vec2d* c1, Vec2d* c2, geo_var_t* s, geo_var_t* t) {
  Vec2d d1 = q1 - p1;   // Direction vector of segment S1
  Vec2d d2 = q2 - p2;   // Direction vector of segment S2
  Vec2d r = p1 - p2;
  // Squared length of segment S1, always nonnegative
  geo_var_t a = d1.LengthSquare();
  // Squared length of segment S2, always nonnegative
  geo_var_t e = d2.LengthSquare();
  geo_var_t f = d2.InnerProd(r);

  // Check if either or both segments degenerate into points
  if ((a <= kGeometryEpsilon) && (e <= kGeometryEpsilon)) {
    // Both segments degenerate into points
    *s = 0.0F;
    *t = 0.0F;
    *c1 = p1;
    *c2 = p2;
    return ((*c1 - *c2).LengthSquare());
  }
  if (a <= kGeometryEpsilon) {
    // First segment degenerates into a point
    *s = 0.0F;
    // s = 0 => t = (b*s + f) / e = f / e
    *t = f / e;
    *t = Clamp<geo_var_t>(*t, 0.0F, 1.0F);
  } else {
    geo_var_t c = d1.InnerProd(r);
    if (e <= kGeometryEpsilon) {
      // Second segment degenerates into a point
      // t = 0 => s = (b*t - c) / a = -c / a
      *t = 0.0F;
      *s = Clamp<geo_var_t>(-c/a, 0.0F, 1.0F);
    } else {
      // The general nondegenerate case starts here
      geo_var_t b = d1.InnerProd(d2);
      geo_var_t denom = a * e - b * b;   // Always nonnegative

      // If segments not parallel, compute closest point on L1 to L2 and
      // clamp to segment S1. Else pick arbitrary s (here 0)
      if (denom != 0.0F) {
        *s = Clamp<geo_var_t>((b*f - c*e) / denom, 0.0F, 1.0F);
      } else {
        *s = 0.0F;
      }
      // Compute point on L2 closest to S1(s) using
      // t = Dot((P1 + D1*s) - P2,D2) / Dot(D2,D2) = (b*s + f) / e
      *t = (b*(*s) + f) / e;

      // If t in [0,1] done. Else clamp t, recompute s for the new value
      // of t using s = Dot((P2 + D2*t) - P1,D1) / Dot(D1,D1)= (t*b - c) / a
      // and clamp s to [0, 1]
      if (*t < 0.0F) {
        *t = 0.0F;
        *s = Clamp<geo_var_t>(-c / a, 0.0F, 1.0F);
      } else if (*t > 1.0F) {
        *t = 1.0F;
        *s = Clamp<geo_var_t>((b - c) / a, 0.0F, 1.0F);
      }
    }
  }

  *c1 = p1 + (*s) * d1;
  *c2 = p2 + (*t) * d2;

  return ((*c1 - *c2).LengthSquare());
}

geo_var_t SqDistSegToSeg_2D(
    const Vec2d& p1, const Vec2d& q1, const Vec2d& p2, const Vec2d& q2) {
  Vec2d c1;
  Vec2d c2;
  geo_var_t s = 0;
  geo_var_t t = 0;

  return (ClosestPtSegToSeg_2D(p1, q1, p2, q2, &c1, &c2, &s, &t));
}

bool OverlapTestSegToSeg_2D(const Vec2d& a, const Vec2d& b,
    const Vec2d& c, const Vec2d& d) {
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
      return (true);
    }
  }

  // Segments not intersecting (or collinear)
  return (false);
}

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

bool OverlapTestOBBToOBB_2D(const OBBox2d& a, const OBBox2d& b) {
  geo_var_t ra;
  geo_var_t rb;
  Matrix<geo_var_t, 2, 2> mat_r;
  Matrix<geo_var_t, 2, 2> mat_abs_r;

  // Compute rotation matrix expressing b in a’s coordinate frame
  for (Int32_t i = 0; i < 2; ++i) {
    for (Int32_t j = 0; j < 2; ++j) {
      mat_r(i, j) = a.unit_direction(i).InnerProd(b.unit_direction(j));
    }
  }

  // Compute translation vector t
  Vec2d t = b.center() - a.center();
  // Bring translation into a’s coordinate frame
  t = Vec2d(t.InnerProd(a.unit_direction(0)), t.InnerProd(a.unit_direction(1)));

  // Compute common subexpressions. Add in an epsilon term to
  // counteract arithmetic errors when two edges are parallel and
  // their cross product is (near) null (see text for details)
  for (Int32_t i = 0; i < 2; ++i) {
    for (Int32_t j = 0; j < 2; ++j) {
      mat_abs_r(i, j) = com_abs(mat_r(i, j)) + kGeometryEpsilon;
    }
  }

  // Test axes L = A0, L = A1
  for (Int32_t i = 0; i < 2; ++i) {
    ra = a.extents()(static_cast<Uint32_t>(i));
    rb = b.extents()(0) * mat_abs_r(i, 0) + b.extents()(1) * mat_abs_r(i, 1);
    if (com_abs(t(static_cast<Uint32_t>(i))) > ra + rb) return (false);
  }

  // Test axes L = B0, L = B1
  for (Int32_t i = 0; i < 2; ++i) {
    ra = a.extents()(0) * mat_abs_r(0, i) + a.extents()(1) * mat_abs_r(1, i);
    rb = b.extents()(static_cast<Uint32_t>(i));
    if (com_abs(t(0) * mat_r(0, i) + t(1) * mat_r(1, i)) > (ra + rb)) {
      return (false);
    }
  }

  // Since no separating axis is found, the OBBs must be intersecting
  return (true);
}

geo_var_t DistOBBToOBB_2D(const OBBox2d& a, const OBBox2d& b) {
  geo_var_t ra = 0;
  geo_var_t rb = 0;
  Matrix<geo_var_t, 2, 2> mat_r;
  Matrix<geo_var_t, 2, 2> mat_abs_r;
  geo_var_t obb_dist = 0;

  // Compute rotation matrix expressing b in a’s coordinate frame
  for (Int32_t i = 0; i < 2; ++i) {
    for (Int32_t j = 0; j < 2; ++j) {
      mat_r(i, j) = a.unit_direction(i).InnerProd(b.unit_direction(j));
    }
  }

  // Compute translation vector t
  Vec2d t = b.center() - a.center();
  // Bring translation into a’s coordinate frame
  t = Vec2d(t.InnerProd(a.unit_direction(0)), t.InnerProd(a.unit_direction(1)));

  // Compute common subexpressions. Add in an epsilon term to
  // counteract arithmetic errors when two edges are parallel and
  // their cross product is (near) null (see text for details)
  for (Int32_t i = 0; i < 2; ++i) {
    for (Int32_t j = 0; j < 2; ++j) {
      mat_abs_r(i, j) = com_abs(mat_r(i, j)) + kGeometryEpsilon;
    }
  }

  // Test axes L = A0, L = A1
  for (Int32_t i = 0; i < 2; ++i) {
    ra = a.extents()(static_cast<Uint32_t>(i));
    rb = b.extents()(0) * mat_abs_r(i, 0) + b.extents()(1) * mat_abs_r(i, 1);
    geo_var_t dist = com_abs(t(static_cast<Uint32_t>(i))) - ra - rb;
    if (dist > obb_dist) {
      obb_dist = dist;
    }
  }

  // Test axes L = B0, L = B1
  for (Int32_t i = 0; i < 2; ++i) {
    ra = a.extents()(0) * mat_abs_r(0, i) + a.extents()(1) * mat_abs_r(1, i);
    rb = b.extents()(static_cast<Uint32_t>(i));
    geo_var_t dist = com_abs(t(0) * mat_r(0, i) + t(1) * mat_r(1, i)) - ra - rb;
    if (dist > obb_dist) {
      obb_dist = dist;
    }
  }

  // Attention, this distance between two OBB is approximate,
  // and may be smaller than the exact one.
  return (obb_dist);
}

bool IntersectRayToSphere_2D(
    const Vec2d& p, const Vec2d& d, const Sphere2d& s,
    Vec2d* q, geo_var_t* t) {
  Vec2d m = p - s.center();
  geo_var_t b = m.InnerProd(d);
  geo_var_t c = m.LengthSquare() - s.radius() * s.radius();
  // Exit if r’s origin outside s (c > 0) and r pointing away from s (b > 0)
  if ((c > 0.0F) && (b > 0.0F)) {
    return (false);
  }
  geo_var_t disc = b*b - c;
  // A negative discriminant corresponds to ray missing sphere
  if (disc < 0.0F) {
    return (false);
  }
  // Ray now found to intersect sphere, compute smallest t value of intersection
  *t = -b - com_sqrt(disc);
  // If t is negative, ray started inside sphere so clamp t to zero
  if (*t < 0.0F) {
    *t = 0.0F;
  }
  *q = p + (*t) * d;
  return (true);
}

bool OverlapTestRayToSphere_2D(
    const Vec2d& p, const Vec2d& d, const Sphere2d& s) {
  Vec2d m = p - s.center();
  geo_var_t c = m.LengthSquare() - s.radius() * s.radius();
  // If there is definitely at least one real root,
  // there must be an intersection
  if (c <= 0.0F) {
    return (true);
  }
  geo_var_t b = m.InnerProd(d);
  // Early exit if ray origin outside sphere and ray pointing away from sphere
  if (b > 0.0F) {
    return (false);
  }
  geo_var_t disc = b*b - c;
  // A negative discriminant corresponds to ray missing sphere
  if (disc < 0.0F) {
    return (false);
  }
  // Now ray must hit sphere
  return (true);
}

bool IntersectRayToAABB_2D(
    const Vec2d& p, const Vec2d& d, const AABBox2d& a,
    Vec2d* q, geo_var_t* tmin) {
  // set to -FLT_MAX to get first hit on line
  *tmin = static_cast<geo_var_t>(-FLT_MAX);
  // set to max distance ray can travel (for segment)
  geo_var_t tmax = static_cast<geo_var_t>(FLT_MAX);

  // For all slabs
  for (Uint32_t i = 0; i < 2; i++) {
    if (std::abs(d(i)) < kGeometryEpsilon) {
      // Ray is parallel to slab. No hit if origin not within slab
      if (p(i) < a.min()(i) || p(i) > a.max()(i)) return (false);
    } else {
      // Compute intersection t value of ray with near and far plane of slab
      geo_var_t ood =  1.0F / d(i);
      geo_var_t t1 = (a.min()(i) - p(i)) * ood;
      geo_var_t t2 = (a.max()(i) - p(i)) * ood;
      // Make t1 be intersection with near plane, t2 with far plane
      if (t1 > t2) std::swap(t1, t2);
      // Compute the intersection of slab intersection intervals
      if (t1 > *tmin) *tmin = t1;
      if (t2 < tmax) tmax = t2;
      // Exit with no collision as soon as slab intersection becomes empty
      if (*tmin > tmax) return (false);
    }
  }
  // Ray intersects all slabs. Return point (q) and intersection t value (tmin)
  *q = p + (*tmin) * d;

  return (true);
}

bool OverlapTestSegToAABB_2D(
    const Vec2d& p0, const Vec2d& p1, const AABBox2d& b) {
  Vec2d c = 0.5F * (b.min() + b.max());  // Box center-point
  Vec2d e = b.max() - c;  // Box halflength extents
  Vec2d m = 0.5F * (p0 + p1);  // Segment midpoint
  Vec2d d = p1 - m;  // Segment halflength vector
  m -= c;  // Translate box and segment to origin

  // Try world coordinate axes as separating axes
  geo_var_t adx = com_abs(d(0));
  if (com_abs(m(0)) > e(0) + adx) return (false);
  geo_var_t ady = std::abs(d(1));
  if (com_abs(m(1)) > e(1) + ady) return (false);

  // No separating axis found; segment must be overlapping AABB
  return (true);
}

bool OverlapTestSphereToAABB_2D(const Sphere2d& s, const AABBox2d& b) {
  // Compute squared distance between sphere center and AABB
  geo_var_t sq_dist = SqDistPointToAABB_2D(b, s.center());
  // Sphere and AABB intersect if the (squared) distance
  // between them is less than the (squared) sphere radius
  return (sq_dist <= s.radius() * s.radius());
}

bool OverlapTestSphereToAABB_2D(
    const Sphere2d& s, const AABBox2d& b, Vec2d* p) {
  // Find point p on AABB closest to sphere center
  ClosestPtPointToAABB_2D(b, s.center(), p);
  // Sphere and AABB intersect if the (squared) distance from sphere
  // center to point p is less than the (squared) sphere radius
  Vec2d v = (*p) - s.center();
  return (v.LengthSquare() <= s.radius() * s.radius());
}

bool OverlapTestSphereToOBB_2D(const Sphere2d& s, const OBBox2d& b) {
  // Compute squared distance between sphere center and OBB
  geo_var_t sq_dist = SqDistPointToOBB_2D(b, s.center());
  // Sphere and OBB intersect if the (squared) distance
  // between them is less than the (squared) sphere radius
  // D_OUTPUT("sq_dist=%f, sq_r=%f\n", sq_dist, s.r * s.r);
  return (sq_dist <= s.radius() * s.radius());
}

bool OverlapTestSphereToOBB_2D(const Sphere2d& s, const OBBox2d& b, Vec2d* p) {
  // Find point p on AABB closest to sphere center
  ClosestPtPointToOBB_2D(b, s.center(), p);
  // Sphere and AABB intersect if the (squared) distance from sphere
  // center to point p is less than the (squared) sphere radius
  Vec2d v = *p - s.center();
  return (v.LengthSquare() <= s.radius() * s.radius());
}


}  // namespace common
}  // namespace phoenix
