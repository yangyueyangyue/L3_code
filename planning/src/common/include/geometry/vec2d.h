/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       vec2d.h
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

#ifndef PHOENIX_COMMON_VEC2D_H_
#define PHOENIX_COMMON_VEC2D_H_

#include <cmath>
#include <string>
#include "utils/macros.h"
#include "utils/log.h"


namespace phoenix {
namespace common {

/// 定义几何浮点型数据的精度
static const Float32_t kGeometryEpsilon = 1E-6F;

/// 定义几何浮点数据类型
typedef Float32_t geo_var_t;

/**
 * @class Vec2d
 * @brief 定义二维点
 */
class Vec2d {
public:
  /**
   * @brief 构造函数
   */
  Vec2d() {
    vec_[0] = 0;
    vec_[1] = 0;
  }

  /**
   * @brief 构造函数
   * @param[in] x 二维点的x坐标
   * @param[in] y 二维点的y坐标
   */
  Vec2d(const geo_var_t x, const geo_var_t y) {
    vec_[0] = x;
    vec_[1] = y;
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    vec_[0] = 0.0F;
    vec_[1] = 0.0F;
  }

  /**
   * @brief 从指定的角度构建二维单位方向向量
   * @param[in] angle 指定的角度
   * @return 二维单位方向向量
   */
  static Vec2d CreateUnitVec2d(const geo_var_t angle);

  /**
   * @brief 获取二维点的x坐标值
   * @return 二维点的x坐标值
   */
  const geo_var_t& x() const { return vec_[0]; }

  /**
   * @brief 获取二维点的y坐标值
   * @return 二维点的y坐标值
   */
  const geo_var_t& y() const { return vec_[1]; }

  /**
   * @brief 获取二维点的x坐标值
   * @return 二维点的x坐标值
   */
  geo_var_t& x() { return vec_[0]; }

  /**
   * @brief 获取二维点的y坐标值
   * @return 二维点的y坐标值
   */
  geo_var_t& y() { return vec_[1]; }

  /**
   * @brief 设置二维点的x坐标值
   * @param[in] x 二维点的x坐标值
   */
  void set_x(const geo_var_t x) { vec_[0] = x; }

  /**
   * @brief 设置二维点的y坐标值
   * @param[in] y 二维点的y坐标值
   */
  void set_y(const geo_var_t y) { vec_[1] = y; }

  /**
   * @brief 获取二维点构成的向量的长度
   * @return 二维点构成的向量的长度
   */
  geo_var_t Length() const;

  /**
   * @brief 获取二维点构成的向量的平方长度
   * @return 二维点构成的向量的平方长度
   */
  geo_var_t LengthSquare() const;

  /**
   * @brief 获取二维点构成的向量的角度（与x轴之间）
   * @return 二维点构成的向量的角度
   */
  geo_var_t Angle() const;

  /**
   * @brief 获取二维点构成的向量正则化（模为1）
   */
  void Normalize();

  /**
   * @brief 计算此二维点与输入的二维点之间的距离
   * @param[in] other 输入的二维点
   * @return 此二维点与输入的二维点之间的距离
   */
  geo_var_t DistanceTo(const Vec2d &other) const;

  /**
   * @brief 计算此二维点与输入的二维点之间的平方距离
   * @param[in] other 输入的二维点
   * @return 此二维点与输入的二维点之间的平方距离
   */
  geo_var_t DistanceSquareTo(const Vec2d &other) const;

  /**
   * @brief 计算此二维点与输入的二维点之间的叉乘
   * @param[in] other 输入的二维点
   * @return 此二维点与输入的二维点之间的叉乘
   */
  geo_var_t CrossProd(const Vec2d &other) const;

  /**
   * @brief 计算此二维点与输入的二维点之间的点乘
   * @param[in] other 输入的二维点
   * @return 此二维点与输入的二维点之间的点乘
   */
  geo_var_t InnerProd(const Vec2d &other) const;

  /**
   * @brief 计算向量加法
   * @param[in] other 输入的向量
   * @return 向量相加的结果
   */
  Vec2d operator+(const Vec2d &other) const;

  /**
   * @brief 计算向量减法
   * @param[in] other 输入的向量
   * @return 向量相减的结果
   */
  Vec2d operator-(const Vec2d &other) const;

  /**
   * @brief 计算标量和向量的乘法
   * @param[in] other 输入的标量
   * @return 标量和向量相乘的结果
   */
  Vec2d operator*(const geo_var_t ratio) const;

  /**
   * @brief 计算向量除以标量
   * @param[in] other 输入的标量
   * @return 向量除以标量的结果
   */
  Vec2d operator/(const geo_var_t ratio) const;

  /**
   * @brief 计算向量加法
   * @param[in] other 输入的向量
   * @return 向量相加的结果
   */
  Vec2d &operator+=(const Vec2d &other);

  /**
   * @brief 计算向量减法
   * @param[in] other 输入的向量
   * @return 向量相减的结果
   */
  Vec2d &operator-=(const Vec2d &other);

  /**
   * @brief 计算标量和向量的乘法
   * @param[in] other 输入的标量
   * @return 标量和向量相乘的结果
   */
  Vec2d &operator*=(const geo_var_t ratio);

  /**
   * @brief 计算向量除以标量
   * @param[in] other 输入的标量
   * @return 向量除以标量的结果
   */
  Vec2d &operator/=(const geo_var_t ratio);

  /**
   * @brief 判断此向量是否和输入向量相等
   * @param[in] other 输入的标量
   * @return true - 相等，false - 不相等
   */
  bool operator==(const Vec2d &other) const;

  /**
   * @brief 以索引方式获取点的坐标
   * @param[in] index 索引（必须是0或1）
   * @return true - 相等，false - 不相等
   */
  const geo_var_t& operator()(Uint32_t index) const {
    COM_CHECK(index < 2);
    return vec_[index];
  }

  /**
   * @brief 以索引方式获取点的坐标
   * @param[in] index 索引（必须是0或1）
   * @return true - 相等，false - 不相等
   */
  geo_var_t& operator()(Uint32_t index) {
    COM_CHECK(index < 2);
    return vec_[index];
  }

  /**
   * @brief 获取调试信息
   * @return 调试信息
   */
  std::string DebugString() const;

 protected:
  // 存储点的坐标
  geo_var_t vec_[2];
};

/**
 * @brief 计算标量和向量的乘法
 * @param[in] ratio 输入的标量
 * @param[in] vec 输入的向量
 * @return 标量和向量相乘的结果
 */
Vec2d operator*(const geo_var_t ratio, const Vec2d &vec);


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_VEC2D_H_
