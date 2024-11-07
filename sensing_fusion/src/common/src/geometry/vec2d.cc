/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       vec2d.cc
 * @brief      二维点
 * @details    定义二维点
 *
 * @author     pengc
 * @date       2020.05.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "geometry/vec2d.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include "utils/log.h"
#include "math/math_utils.h"

namespace phoenix {
namespace common {

Vec2d Vec2d::CreateUnitVec2d(const geo_var_t angle) {
  return Vec2d(com_cos(angle), com_sin(angle));
}

geo_var_t Vec2d::Length() const {
  // return std::hypot(vec_[0], vec_[1]);
  return com_sqrt(LengthSquare());
}

geo_var_t Vec2d::LengthSquare() const {
  return vec_[0] * vec_[0] + vec_[1] * vec_[1];
}

geo_var_t Vec2d::Angle() const {
  return com_atan2(vec_[1], vec_[0]);
}

void Vec2d::Normalize() {
  const geo_var_t l = Length();
  if (l > kGeometryEpsilon) {
    vec_[0] /= l;
    vec_[1] /= l;
  }
}

geo_var_t Vec2d::DistanceTo(const Vec2d &other) const {
  // return std::hypot(vec_[0] - other.vec_[0], vec_[1] - other.vec_[1]);
  return com_sqrt(DistanceSquareTo(other));
}

geo_var_t Vec2d::DistanceSquareTo(const Vec2d &other) const {
  const geo_var_t dx = vec_[0] - other.vec_[0];
  const geo_var_t dy = vec_[1] - other.vec_[1];
  return dx * dx + dy * dy;
}

geo_var_t Vec2d::CrossProd(const Vec2d &other) const {
  return vec_[0] * other.y() - vec_[1] * other.x();
}

geo_var_t Vec2d::InnerProd(const Vec2d &other) const {
  return vec_[0] * other.x() + vec_[1] * other.y();
}

Vec2d Vec2d::operator+(const Vec2d &other) const {
  return Vec2d(vec_[0] + other.x(), vec_[1] + other.y());
}

Vec2d Vec2d::operator-(const Vec2d &other) const {
  return Vec2d(vec_[0] - other.x(), vec_[1] - other.y());
}

Vec2d Vec2d::operator*(const geo_var_t ratio) const {
  return Vec2d(vec_[0] * ratio, vec_[1] * ratio);
}

Vec2d Vec2d::operator/(const geo_var_t ratio) const {
  // CHECK_GT(std::abs(ratio), kMathEpsilon);
  COM_CHECK(com_abs(ratio) > kGeometryEpsilon);
  return Vec2d(vec_[0] / ratio, vec_[1] / ratio);
}

Vec2d &Vec2d::operator+=(const Vec2d &other) {
  vec_[0] += other.x();
  vec_[1] += other.y();
  return *this;
}

Vec2d &Vec2d::operator-=(const Vec2d &other) {
  vec_[0] -= other.x();
  vec_[1] -= other.y();
  return *this;
}

Vec2d &Vec2d::operator*=(const geo_var_t ratio) {
  vec_[0] *= ratio;
  vec_[1] *= ratio;
  return *this;
}

Vec2d &Vec2d::operator/=(const geo_var_t ratio) {
  // CHECK_GT(std::abs(ratio), kMathEpsilon);
  COM_CHECK(com_abs(ratio) > kGeometryEpsilon);
  vec_[0] /= ratio;
  vec_[1] /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d &other) const {
  return (com_abs(vec_[0] - other.x()) < kGeometryEpsilon &&
      com_abs(vec_[1] - other.y()) < kGeometryEpsilon);
}

Vec2d operator*(const geo_var_t ratio, const Vec2d &vec) {
  return vec * ratio;
}

std::string Vec2d::DebugString() const {
  std::ostringstream str_info;
  str_info << "vec2d (x=" << vec_[0]
           << " y=" << vec_[1]
           << ")";
  return (str_info.str());
}

}  // namespace common
}  // namespace phoenix

