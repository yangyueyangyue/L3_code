/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       math_utils.h
 * @brief      定义常用的数学相关的功能
 * @details    定义常用的数学相关的功能(四舍五入，开方，角度计算等)
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_MATH_UTILS_H_
#define PHOENIX_COMMON_MATH_UTILS_H_

#include <math.h>
#include <float.h>
#include <limits>
#include <utility>
#include <cmath>
#include "utils/macros.h"

/// 定义是否使用查表的方式来实现三角函数运算
/// 注意：若定义使用查表的方式来实现三角函数运算，则在进行三角函数运算之前
/// 必须调用CreateTriangleLookupTables来创建三角函数查找表
/// (只需要调用一次即可，通常在main函数中调用)
#define USING_TRIANGLE_LOOKUP_TABLE (0)

/// 定义PI
#ifndef COM_PI
  /// Copied from math.h
  #define COM_PI    (3.14159265358979323846)  // pi
  #define COM_PI_2  (1.57079632679489661923)  // pi/2
  #define COM_PI_4  (0.78539816339744830962)  // pi/4
  #define COM_PIl   (3.1415926535897932384626433832795029L)  // pi
  #define COM_PI_2l (1.5707963267948966192313216916397514L)  // pi/2
  #define COM_PI_4l (0.7853981633974483096156608458198757L)  // pi/4
#endif


#ifdef __GNUC__
  #if __cplusplus < 201103L
    /// 判断是不是NAN值（not a number非法数字）
    #define com_isnan(x)    isnan(x)
    /// 测试某个浮点数是不是有限的数
    #define com_isfinite(x) (finite(x) != 0)
    /// 测试某个浮点数是否是无限大
    #define com_isinf(x)    (finite(x) == 0)
  #else
    /// 判断是不是NAN值（not a number非法数字）
    #define com_isnan(x)    std::isnan(x)
    /// 测试某个浮点数是不是有限的数
    #define com_isfinite(x) std::isfinite(x)
    /// 测试某个浮点数是否是无限大
    #define com_isinf(x)    std::isinf(x)
  #endif
#else
  #if __cplusplus < 201103L
    /// 判断是不是NAN值（not a number非法数字）
    #define com_isnan(x)    isnan(x)
    /// 测试某个浮点数是不是有限的数
    #define com_isfinite(x) (finite(x) != 0)
    /// 测试某个浮点数是否是无限大
    #define com_isinf(x)    (finite(x) == 0)
  #else
    /// 判断是不是NAN值（not a number非法数字）
    #define com_isnan(x)    std::isnan(x)
    /// 测试某个浮点数是不是有限的数
    #define com_isfinite(x) std::isfinite(x)
    /// 测试某个浮点数是否是无限大
    #define com_isinf(x)    std::isinf(x)
  #endif
#endif


namespace phoenix {
namespace common {


/// 定义float64_t型浮点误差
static const Float64_t kMathEpsilonD = 1e-10;
/// 定义float32_t型浮点误差
static const Float32_t kMathEpsilonF = 1e-6F;

/**
 * @class NumLimits
 * @brief 定义了数据类型相关的某些常量值
 * @param T 标量数据类型
 */
template<typename T>
struct NumLimits {
  static T min() {
    return std::numeric_limits<T>::min();
  }

  static T max() {
    return std::numeric_limits<T>::max();
  }

  static T epsilon() {
    return std::numeric_limits<T>::epsilon();
  }
};

/**
 * @brief 向下取整(获得不大于number参数且最靠近参数number的整数)
 * @param[in] number 需要向下取整的值
 * @return 取整后的值
 */
inline Float64_t com_floor(Float64_t number) {
  return floor(number);
}

/**
 * @brief 向下取整(获得不大于number参数且最靠近参数number的整数)
 * @param[in] number 需要向下取整的值
 * @return 取整后的值
 */
inline Float32_t com_floor(Float32_t number) {
  return floorf(number);
}

/**
 * @brief 向上取整(获得不小于number参数且最靠近参数number的整数)
 * @param[in] number 需要向上取整的值
 * @return 取整后的值
 */
inline Float64_t com_ceil(Float64_t number) {
  return ceil(number);
}

/**
 * @brief 向上取整(获得不小于number参数且最靠近参数number的整数)
 * @param[in] number 需要向上取整的值
 * @return 取整后的值
 */
inline Float32_t com_ceil(Float32_t number) {
  return ceilf(number);
}

/**
 * @brief 四舍五入取整
 * @param[in] number 需要取整的值
 * @return 取整后的值
 */
inline Float64_t com_round(Float64_t number) {
  return (number < 0.0 ? com_ceil(number - 0.5) : com_floor(number + 0.5));
}

/**
 * @brief 四舍五入取整
 * @param[in] number 需要取整的值
 * @return 取整后的值
 */
inline Float32_t com_round(Float32_t number) {
  return (number < 0.0f ? com_ceil(number - 0.5f) : com_floor(number + 0.5f));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
inline Int32_t com_rint(Float64_t x) {
  return (static_cast<Int32_t>(com_round(x)));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
inline Int32_t com_rintf(Float32_t x) {
  return (static_cast<Int32_t>(com_round(x)));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
inline Int64_t com_lrint(Float64_t x) {
  return (static_cast<Int64_t>(com_round(x)));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
inline Int64_t com_lrintf(Float32_t x) {
  return (static_cast<Int64_t>(com_round(x)));
}

/**
 * @brief 获取某个数的绝对值
 * @param[in] number 需要求绝对值的数
 * @return 绝对值
 */
inline Float64_t com_abs(Float64_t value) {
  return std::abs(value);
}

/**
 * @brief 获取某个数的绝对值
 * @param[in] number 需要求绝对值的数
 * @return 绝对值
 */
inline Float32_t com_abs(Float32_t value) {
  return std::abs(value);
}

/**
 * @brief 获取某个数的绝对值
 * @param[in] number 需要求绝对值的数
 * @return 绝对值
 */
inline Int32_t com_abs(Int32_t value) {
  return ((value < 0) ? -value : value);
}

/**
 * @brief 求非负实数的平方根
 * @param[in] number 需要求平方跟的非负实数
 * @return 平方根
 */
inline Float64_t com_sqrt(Float64_t value) {
  return std::sqrt(value);
}

/**
 * @brief 求非负实数的平方根
 * @param[in] number 需要求平方跟的非负实数
 * @return 平方根
 */
inline Float32_t com_sqrt(Float32_t value) {
  return std::sqrt(value);
}

/**
 * @brief 求直角三角形的斜边长
 * @param[in] x 直角边1的长度
 * @param[in] y 直角边2的长度
 * @return 斜边长(=sqrt(x*x + y*y))
 */
inline Float64_t com_hypot(const Float64_t x, const Float64_t y) {
  Float64_t abs_x = com_abs(x);
  Float64_t abs_y = com_abs(y);
  Float64_t p = 0;
  Float64_t qp = 0;
  if (abs_x > abs_y) {
    p = abs_x;
    qp = abs_y / p;
  } else {
    p = abs_y;
    qp = abs_x / p;
  }
  if(p <= NumLimits<Float64_t>::min()) {
    return Float64_t(0);
  }

  return (p * com_sqrt(Float64_t(1) + qp*qp));
}

/**
 * @brief 求直角三角形的斜边长
 * @param[in] x 直角边1的长度
 * @param[in] y 直角边2的长度
 * @return 斜边长(=sqrt(x*x + y*y))
 */
inline Float32_t com_hypot(const Float32_t x, const Float32_t y) {
  Float32_t abs_x = com_abs(x);
  Float32_t abs_y = com_abs(y);
  Float32_t p = 0;
  Float32_t qp = 0;
  if (abs_x > abs_y) {
    p = abs_x;
    qp = abs_y / p;
  } else {
    p = abs_y;
    qp = abs_x / p;
  }
  if(p <= NumLimits<Float32_t>::min()) {
    return Float32_t(0);
  }

  return (p * com_sqrt(Float32_t(1) + qp*qp));
}

/**
 * @brief 将以弧度表示的角度值转换为以度表示的角度值
 * @param[in] alpha 以弧度表示的角度值
 * @return 以度表示的角度值
 */
inline Float64_t com_rad2deg(Float64_t alpha) {
  return (alpha * 57.29577951308232087685);
}

/**
 * @brief 将以弧度表示的角度值转换为以度表示的角度值
 * @param[in] alpha 以弧度表示的角度值
 * @return 以度表示的角度值
 */
inline Float32_t com_rad2deg(Float32_t alpha) {
  return (alpha * 57.29578f);
}

/**
 * @brief 将以度表示的角度值转换为以弧度表示的角度值
 * @param[in] alpha 以度表示的角度值
 * @return 以弧度表示的角度值
 */
inline Float64_t com_deg2rad(Float64_t alpha) {
  return (alpha * 0.01745329251994329577);
}

/**
 * @brief 将以度表示的角度值转换为以弧度表示的角度值
 * @param[in] alpha 以度表示的角度值
 * @return 以弧度表示的角度值
 */
inline Float32_t com_deg2rad(Float32_t alpha) {
  return (alpha * 0.017453293f);
}

/**
 * @brief 创建三角函数查找表（用来加快三角函数的运算）
 */
void CreateTriangleLookupTables();

/**
 * @brief 求正弦值(通过查表求值)
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正弦值 (-1 ~ 1)
 */
Float64_t SinLookUp(Float64_t value);

/**
 * @brief 求余弦值(通过查表求值)
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 余弦值 (-1 ~ 1)
 */
Float64_t CosLookUp(Float64_t value);

/**
 * @brief 通过反正弦函数求角度(通过查表求值)
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
Float64_t AsinLookUp(Float64_t value);

/**
 * @brief 通过反正切函数求角度（y/x）(通过查表求值)
 * @param[in] value 输入值y
 * @param[in] value 输入值x
 * @return 角度 (-PI/2 ~ PI/2)
 */
Float64_t Atan2LookUp(Float64_t y, Float64_t x);

/**
 * @brief 求正弦值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正弦值 (-1 ~ 1)
 */
inline Float64_t com_sin(Float64_t value) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (SinLookUp(value));
#else
  return (std::sin(value));
#endif
}

/**
 * @brief 求正弦值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正弦值 (-1 ~ 1)
 */
inline Float32_t com_sin(Float32_t value) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (SinLookUp(value));
#else
  return (std::sin(value));
#endif
}

/**
 * @brief 求余弦值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 余弦值 (-1 ~ 1)
 */
inline Float64_t com_cos(Float64_t value) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (CosLookUp(value));
#else
  return (std::cos(value));
#endif
}

/**
 * @brief 求余弦值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 余弦值 (-1 ~ 1)
 */
inline Float32_t com_cos(Float32_t value) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (CosLookUp(value));
#else
  return (std::cos(value));
#endif
}

/**
 * @brief 求正切值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正切值
 */
inline Float64_t com_tan(Float64_t value) {
  return (std::tan(value));
}

/**
 * @brief 求正切值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正切值
 */
inline Float32_t com_tan(Float32_t value) {
  return (std::tan(value));
}

/**
 * @brief 通过反正弦函数求角度
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
inline Float64_t com_asin(Float64_t value) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (AsinLookUp(value));
#else
  return (std::asin(value));
#endif
}

/**
 * @brief 通过反正弦函数求角度
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
inline Float32_t com_asin(Float32_t value) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (AsinLookUp(value));
#else
  return (std::asin(value));
#endif
}

/**
 * @brief 通过反余弦函数求角度
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
inline Float64_t com_acos(Float64_t value) {
  return (std::acos(value));
}

/**
 * @brief 通过反余弦函数求角度
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
inline Float32_t com_acos(Float32_t value) {
  return (std::acos(value));
}

/**
 * @brief 通过反正切函数求角度(y/x)
 * @param[in] value 输入值y
 * @param[in] value 输入值x
 * @return 角度 (-PI/2 ~ PI/2)
 */
inline Float64_t com_atan2(Float64_t y, Float64_t x) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (Atan2LookUp(y, x));
#else
  return (std::atan2(y, x));
#endif
}

/**
 * @brief 通过反正切函数求角度(y/x)
 * @param[in] value 输入值y
 * @param[in] value 输入值x
 * @return 角度 (-PI/2 ~ PI/2)
 */
inline Float32_t com_atan2(Float32_t y, Float32_t x) {
#if USING_TRIANGLE_LOOKUP_TABLE
  return (Atan2LookUp(y, x));
#else
  return (std::atan2(y, x));
#endif
}

/**
 * @brief 将角度限定到 [0, 2 * PI) 之间
 * @param[in] angle 输入的角度
 * @return 限定后的角度
 */
Float64_t WrapAngle(const Float64_t angle);

/**
 * @brief 将角度限定到 [0, 2 * PI) 之间
 * @param[in] angle 输入的角度
 * @return 限定后的角度
 */
Float32_t WrapAngle(const Float32_t angle);

/**
 * @brief 将角度正则化到 [-PI, PI) 之间
 * @param[in] angle 输入的角度
 * @return 正则化后的角度
 */
Float64_t NormalizeAngle(const Float64_t angle);

/**
 * @brief 将角度正则化到 [-PI, PI) 之间
 * @param[in] angle 输入的角度
 * @return 正则化后的角度
 */
Float32_t NormalizeAngle(const Float32_t angle);

/**
 * @brief 求两个角度之间的差值 (to - from)
 * @param[in] from 起始角度值
 * @param[in] to 结束角度值
 * @return 两个角度之间的差值 [0, PI)
 */
Float64_t AngleDiff(const Float64_t from, const Float64_t to);

/**
 * @brief 求两个角度之间的差值 (to - from)
 * @param[in] from 起始角度值
 * @param[in] to 结束角度值
 * @return 两个角度之间的差值 [0, PI)
 */
Float32_t AngleDiff(const Float32_t from, const Float32_t to);

/**
 * @brief 求输入值的平方值
 * @param[in] value 输入值
 * @return 输入值的平方值
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}

/**
 * @brief 将数n限定到范围 [min, max] 之间
 * @param[in] n 输入值
 * @param[in] min 范围(左边界值)
 * @param[in] max 范围(右边界值)
 * @return 限定后的值
 */
template <typename T>
inline T Clamp(T n, T min, T max) {
  if (n < min) return (min);
  if (n > max) return (max);
  return (n);
}

/**
 * @brief 比较两个值，返回小值
 * @param[in] n1 输入值1
 * @param[in] n2 输入值2
 * @return 两个值中的小值
 */
template <typename T>
inline T Min(T n1, T n2) {
  return (n1 < n2 ? n1 : n2);
}

/**
 * @brief 比较两个值，返回大值
 * @param[in] n1 输入值1
 * @param[in] n2 输入值2
 * @return 两个值中的大值
 */
template <typename T>
inline T Max(T n1, T n2) {
  return (n1 > n2 ? n1 : n2);
}

/**
 * @brief 交换两个数
 * @param[in] n1 输入值1
 * @param[in] n2 输入值2
 */
template <typename T>
inline void Swap(T& n1, T& n2) {
  T tmp = n1;
  n1 = n2;
  n2 = tmp;
}


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_MATH_UTILS_H_
