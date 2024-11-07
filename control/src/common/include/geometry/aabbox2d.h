/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       aabbox2d.h
 * @brief      与坐标轴平行的矩形包围盒
 * @details    定义二维平面上的与坐标轴平行的矩形包围盒
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

#ifndef PHOENIX_COMMON_AABBOX2D_H_
#define PHOENIX_COMMON_AABBOX2D_H_


#include "geometry/vec2d.h"


namespace phoenix {
namespace common {

/**
 * @class AABBox2d
 * @brief 二维平面上的与坐标轴平行的矩形包围盒
 */
class AABBox2d {
 public:
  /**
   * @brief Default constructor.
   * Creates an axes-aligned box with zero length and width at the origin.
   */
  AABBox2d() {
  }

  /**
   * @brief 使用不同的两个点构建包围盒
   * @param[in] one_corner 第一个点
   * @param[in] opposite_corner 第二个点
   */
  AABBox2d(const Vec2d &one_corner, const Vec2d &opposite_corner);

  /**
   * @brief 使用不同的两个点构建包围盒
   * @param[in] one_corner_x 第一个点x坐标
   * @param[in] one_corner_y 第一个点y坐标
   * @param[in] opposite_corner_x 第二个点x坐标
   * @param[in] opposite_corner_y 第二个点y坐标
   */
  AABBox2d(geo_var_t one_corner_x, geo_var_t one_corner_y,
           geo_var_t opposite_corner_x, geo_var_t opposite_corner_y);

  /**
   * @brief 使用包围盒中心点及长度和宽度构建包围盒
   * @param[in] center 包围盒中心点
   * @param[in] length 沿着x轴的长度
   * @param[in] width 沿着y轴的宽度
   */
  AABBox2d(const Vec2d &center,
           const geo_var_t length, const geo_var_t width);

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    min_.Clear();
    max_.Clear();
  }

  /**
  * @brief Returns the minimum x-coordinate of the box
  *
  * @return x-coordinate
  */
  inline geo_var_t min_x() const { return min_.x(); }

  /**
  * @brief Returns the maximum x-coordinate of the box
  *
  * @return x-coordinate
  */
  inline geo_var_t max_x() const { return max_.x(); }

  /**
  * @brief Returns the minimum y-coordinate of the box
  *
  * @return y-coordinate
  */
  inline geo_var_t min_y() const { return min_.y(); }

  /**
  * @brief Returns the maximum y-coordinate of the box
  *
  * @return y-coordinate
  */
  inline geo_var_t max_y() const { return max_.y(); }

  /**
   * @brief 获取包围盒最左下角的点
   * @return 最左下角的点
   */
  inline const Vec2d& min() const { return min_; }

  /**
   * @brief 获取包围盒最右上角的点
   * @return 最右上角的点
   */
  inline const Vec2d& max() const { return max_; }

  /**
   * @brief 获取包围盒最左下角的点
   * @return 最左下角的点
   */
  inline Vec2d& min() { return min_; }

  /**
   * @brief 获取包围盒最右上角的点
   * @return 最右上角的点
   */
  inline Vec2d& max() { return max_; }

  /**
   * @brief 判断点是否在包围盒内部
   * @param[in] point 待判断的点
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief 判断点是否在包围盒内部
   * @param[in] x 待判断的点的x坐标
   * @param[in] y 待判断的点的y坐标
   */
  bool IsPointIn(geo_var_t x, geo_var_t y) const;

 private:
  // 保存包围盒最左下角的点
  Vec2d min_;
  // 保存包围盒最右上角的点
  Vec2d max_;
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_AABBOX2D_H_

