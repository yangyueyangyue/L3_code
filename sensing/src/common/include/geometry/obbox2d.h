/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       obbox2d.h
 * @brief      有向矩形包围盒
 * @details    定义二维平面上的有向矩形包围盒
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

#ifndef PHOENIX_COMMON_OBBOX2D_H_
#define PHOENIX_COMMON_OBBOX2D_H_

#include "geometry/vec2d.h"
#include "math/math_utils.h"


namespace phoenix {
namespace common {


/**
 * @class OBBox2d
 * @brief 二维平面上的有向矩形包围盒
 */
class OBBox2d {
public:
  /**
   * @brief 构造函数
   */
  OBBox2d();

  /**
   * @brief 构造函数
   * @param[in] center 包围盒中心坐标
   * @param[in] unit_direction 包围盒单位方向向量
   * @param[in] extent 包围盒的半长(沿着方向向量上的,保存在x中) \n
   *                   及半宽(沿着方向向量法向方向的,保存在y中)
   */
  OBBox2d(const Vec2d& center,
          const Vec2d& unit_direction, const Vec2d& extent);

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 获取包围盒中心坐标
   * @return 包围盒中心坐标
   */
  inline const Vec2d& center() const { return center_; }

  /**
   * @brief 获取包围盒单位方向向量
   * @return 包围盒单位方向向量
   */
  inline const Vec2d& unit_direction_x() const { return unit_direction_[0]; }

  /**
   * @brief 获取包围盒方向向量的法向方向的单位方向向量
   * @return 包围盒方向向量的法向方向的单位方向向量
   */
  inline const Vec2d& unit_direction_y() const { return unit_direction_[1]; }

  /**
   * @brief 获取包围盒的半长(沿着方向向量上的)及半宽(沿着方向向量法向方向的)
   * @return 包围盒的半长(保存在x中)及半宽(保存在y中)
   */
  inline const Vec2d& extents() const { return extents_; }

  /**
   * @brief 获取包围盒中心坐标
   * @return 包围盒中心坐标
   */
  inline Vec2d& center() { return center_; }

  /**
   * @brief 获取包围盒单位方向向量
   * @return 包围盒单位方向向量
   */
  inline Vec2d& unit_direction_x() { return unit_direction_[0]; }

  /**
   * @brief 获取包围盒方向向量的法向方向的单位方向向量
   * @return 包围盒方向向量的法向方向的单位方向向量
   */
  inline Vec2d& unit_direction_y() { return unit_direction_[1]; }

  /**
   * @brief 获取包围盒的半长(沿着方向向量上的)及半宽(沿着方向向量法向方向的)
   * @return 包围盒的半长(保存在x中)及半宽(保存在y中)
   */
  inline Vec2d& extents() { return extents_; }

  /**
   * @brief 根据索引获取包围盒单位方向向量
   * @param[in] index 方向向量的索引 \n
   *            (0-包围盒方向向量，1-包围盒方向向量的法线方向的单位方向向量)
   * @return 包围盒单位方向向量
   */
  inline const Vec2d& unit_direction(Int32_t index) const {
    COM_CHECK(0 <= index && index < 2);
    return (unit_direction_[index]);
  }

  /**
   * @brief 根据索引获取包围盒单位方向向量
   * @param[in] index 方向向量的索引 \n
   *            (0-包围盒方向向量，1-包围盒方向向量的法线方向的单位方向向量)
   * @return 包围盒单位方向向量
   */
  inline Vec2d& unit_direction(Int32_t index) {
    COM_CHECK(0 <= index && index < 2);
    return (unit_direction_[index]);
  }

  /**
   * @brief 设置包围盒中心坐标
   * @param[in] x 中心点坐标x值
   * @param[in] y 中线点坐标y值
   */
  inline void set_center(geo_var_t x, geo_var_t y) {
    center_.set_x(x);
    center_.set_y(y);
  }

  /**
   * @brief 设置包围盒中心坐标
   * @param[in] center 中心点坐标
   */
  inline void set_center(const Vec2d& center) {
    center_ = center;
  }

  /**
   * @brief 设置包围盒单位方向向量
   * @param[in] direction 方向(rad)
   */
  inline void set_unit_direction(geo_var_t direction) {
    unit_direction_[0].set_x(com_cos(direction));
    unit_direction_[0].set_y(com_sin(direction));
    unit_direction_[1].set_x(-unit_direction_[0].y());
    unit_direction_[1].set_y(unit_direction_[0].x());
  }

  /**
   * @brief 设置包围盒单位方向向量
   * @param[in] x 单位方向向量坐标x值
   * @param[in] y 单位方向向量坐标y值
   */
  inline void set_unit_direction(geo_var_t x, geo_var_t y) {
    unit_direction_[0].set_x(x);
    unit_direction_[0].set_y(y);
    unit_direction_[1].set_x(-y);
    unit_direction_[1].set_y(x);
  }

  /**
   * @brief 设置包围盒单位方向向量
   * @param[in] unit_direction 单位方向向量
   */
  inline void set_unit_direction(const Vec2d& unit_direction) {
    unit_direction_[0] = unit_direction;
    unit_direction_[1].set_x(-unit_direction_[0].y());
    unit_direction_[1].set_y(unit_direction_[0].x());
  }

  /**
   * @brief 设置包围盒的半长(沿着方向向量上的)及半宽(沿着方向向量法向方向的)
   * @param[in] x 包围盒的半长
   * @param[in] y 包围盒的半宽
   */
  inline void set_extents(geo_var_t x, geo_var_t y) {
    extents_.set_x(x);
    extents_.set_y(y);
  }

  /**
   * @brief 设置包围盒的半长(沿着方向向量上的)及半宽(沿着方向向量法向方向的)
   * @param[in] extent 包围盒的半长(保存在x中)及半宽(保存在y中)
   */
  inline void set_extents(const Vec2d& extent) {
    extents_ = extent;
  }

  inline geo_var_t CalcCircumradius() const {
    return (common::com_sqrt(common::Square(extents_.x()) +
                             common::Square(extents_.y())));
  }

private:
  // OBB center point
  Vec2d center_;
  // Local x-, y-
  Vec2d unit_direction_[2];
  // Positive halfwidth extents of OBB along each axis
  Vec2d extents_;
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_OBBOX2D_H_

