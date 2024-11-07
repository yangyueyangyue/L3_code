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

#ifndef PHOENIX_COMMON_MATH_UTILS_C_H_
#define PHOENIX_COMMON_MATH_UTILS_C_H_

#include <math.h>
#include <float.h>
#include "utils/macros.h"

/// 定义是否使用查表的方式来实现三角函数运算
/// 注意：若定义使用查表的方式来实现三角函数运算，则在进行三角函数运算之前
/// 必须调用CreateTriangleLookupTables来创建三角函数查找表
/// (只需要调用一次即可，通常在main函数中调用)
#define PHOENIX_USING_TRIANGLE_LOOKUP_TABLE (0)

/// 定义PI
#ifndef COM_PI
/// Copied from math.h
# define COM_PI    (3.14159265358979323846)  // pi
# define COM_PI_2  (1.57079632679489661923)  // pi/2
# define COM_PI_4  (0.78539816339744830962)  // pi/4
# define COM_PIl   (3.1415926535897932384626433832795029L)  // pi
# define COM_PI_2l (1.5707963267948966192313216916397514L)  // pi/2
# define COM_PI_4l (0.7853981633974483096156608458198757L)  // pi/4
#endif


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 向下取整(获得不大于number参数且最靠近参数number的整数)
 * @param[in] number 需要向下取整的值
 * @return 取整后的值
 */
static Float64_t phoenix_com_floor_d(Float64_t number) {
  return floor(number);
}

/**
 * @brief 向下取整(获得不大于number参数且最靠近参数number的整数)
 * @param[in] number 需要向下取整的值
 * @return 取整后的值
 */
static Float32_t phoenix_com_floor_f(Float32_t number) {
  return floorf(number);
}

/**
 * @brief 向上取整(获得不小于number参数且最靠近参数number的整数)
 * @param[in] number 需要向上取整的值
 * @return 取整后的值
 */
static Float64_t phoenix_com_ceil_d(Float64_t number) {
  return ceil(number);
}

/**
 * @brief 向上取整(获得不小于number参数且最靠近参数number的整数)
 * @param[in] number 需要向上取整的值
 * @return 取整后的值
 */
static Float32_t phoenix_com_ceil_f(Float32_t number) {
  return ceilf(number);
}

/**
 * @brief 四舍五入取整
 * @param[in] number 需要取整的值
 * @return 取整后的值
 */
static Float64_t phoenix_com_round_d(Float64_t number) {
  return (number < 0.0 ? phoenix_com_ceil_d(number - 0.5) :
                         phoenix_com_floor_d(number + 0.5));
}

/**
 * @brief 四舍五入取整
 * @param[in] number 需要取整的值
 * @return 取整后的值
 */
static Float32_t phoenix_com_round_f(Float32_t number) {
  return (number < 0.0f ? phoenix_com_ceil_f(number - 0.5f) :
                          phoenix_com_floor_f(number + 0.5f));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
static Int32_t phoenix_com_rint_d(Float64_t x) {
  return ((Int32_t)(phoenix_com_round_d(x)));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
static Int32_t phoenix_com_rint_f(Float32_t x) {
  return ((Int32_t)(phoenix_com_round_f(x)));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
static Int64_t phoenix_com_lrint_d(Float64_t x) {
  return ((Int64_t)(phoenix_com_round_d(x)));
}

/**
 * @brief 四舍五入取整
 * @param[in] x 需要取整的值
 * @return 取整后的值
 */
static Int64_t phoenix_com_lrint_f(Float32_t x) {
  return ((Int64_t)(phoenix_com_round_f(x)));
}

/**
 * @brief 获取某个数的绝对值
 * @param[in] number 需要求绝对值的数
 * @return 绝对值
 */
static Float64_t phoenix_com_abs_d(Float64_t value) {
  return (fabs(value));
}

/**
 * @brief 获取某个数的绝对值
 * @param[in] number 需要求绝对值的数
 * @return 绝对值
 */
static Float32_t phoenix_com_abs_f(Float32_t value) {
  return (fabs(value));
}

/**
 * @brief 求非负实数的平方根
 * @param[in] number 需要求平方跟的非负实数
 * @return 平方根
 */
static Float64_t phoenix_com_sqrt_d(Float64_t value) {
  return sqrt(value);
}

/**
 * @brief 求非负实数的平方根
 * @param[in] number 需要求平方跟的非负实数
 * @return 平方根
 */
static Float32_t phoenix_com_sqrt_f(Float32_t value) {
  return sqrt(value);
}

/**
 * @brief 将以弧度表示的角度值转换为以度表示的角度值
 * @param[in] alpha 以弧度表示的角度值
 * @return 以度表示的角度值
 */
static Float32_t phoenix_com_rad2deg_f(Float32_t alpha) {
  return (alpha * 57.29578f);
}

/**
 * @brief 将以度表示的角度值转换为以弧度表示的角度值
 * @param[in] alpha 以度表示的角度值
 * @return 以弧度表示的角度值
 */
static Float32_t phoenix_com_deg2rad_f(Float32_t alpha) {
  return (alpha * 0.017453293f);
}

/**
 * @brief 创建三角函数查找表（用来加快三角函数的运算）
 */
void Phoenix_Common_CreateTriangleLookupTables();

/**
 * @brief 求正弦值(通过查表求值)
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正弦值 (-1 ~ 1)
 */
Float64_t Phoenix_Common_SinLookUp(Float64_t value);

/**
 * @brief 求余弦值(通过查表求值)
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 余弦值 (-1 ~ 1)
 */
Float64_t Phoenix_Common_CosLookUp(Float64_t value);

/**
 * @brief 通过反正弦函数求角度(通过查表求值)
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
Float64_t Phoenix_Common_AsinLookUp(Float64_t value);

/**
 * @brief 通过反正切函数求角度（y/x）(通过查表求值)
 * @param[in] value 输入值y
 * @param[in] value 输入值x
 * @return 角度 (-PI/2 ~ PI/2)
 */
Float64_t Phoenix_Common_Atan2LookUp(Float64_t y, Float64_t x);

/**
 * @brief 求正弦值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正弦值 (-1 ~ 1)
 */
static Float32_t phoenix_com_sin_f(Float32_t value) {
#if PHOENIX_USING_TRIANGLE_LOOKUP_TABLE
  return (Phoenix_Common_SinLookUp(value));
#else
  return (sin(value));
#endif
}

/**
 * @brief 求余弦值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 余弦值 (-1 ~ 1)
 */
static Float32_t phoenix_com_cos_f(Float32_t value) {
#if PHOENIX_USING_TRIANGLE_LOOKUP_TABLE
  return (CosLookUp(value));
#else
  return (cos(value));
#endif
}

/**
 * @brief 求正切值
 * @param[in] value 角度值 (-PI/2 ~ PI/2)
 * @return 正切值
 */
static Float32_t phoenix_com_tan_f(Float32_t value) {
  return (tan(value));
}

/**
 * @brief 通过反正弦函数求角度
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
static Float32_t phoenix_com_asin_f(Float32_t value) {
#if PHOENIX_USING_TRIANGLE_LOOKUP_TABLE
  return (AsinLookUp(value));
#else
  return (asin(value));
#endif
}

/**
 * @brief 通过反余弦函数求角度
 * @param[in] value 输入值 (y / r)
 * @return 角度 (-PI/2 ~ PI/2)
 */
static Float32_t phoenix_com_acos_f(Float32_t value) {
  return (acos(value));
}

/**
 * @brief 通过反正切函数求角度(y/x)
 * @param[in] value 输入值y
 * @param[in] value 输入值x
 * @return 角度 (-PI/2 ~ PI/2)
 */
static Float32_t phoenix_com_atan2_f(Float32_t y, Float32_t x) {
#if PHOENIX_USING_TRIANGLE_LOOKUP_TABLE
  return (Atan2LookUp(y, x));
#else
  return (atan2(y, x));
#endif
}

/**
 * @brief 将角度限定到 [0, 2 * PI) 之间
 * @param[in] angle 输入的角度
 * @return 限定后的角度
 */
Float32_t Phoenix_Common_WrapAngle_f(const Float32_t angle);

/**
 * @brief 将角度正则化到 [-PI, PI) 之间
 * @param[in] angle 输入的角度
 * @return 正则化后的角度
 */
Float32_t Phoenix_Common_NormalizeAngle_f(const Float32_t angle);

/**
 * @brief 求两个角度之间的差值 (to - from)
 * @param[in] from 起始角度值
 * @param[in] to 结束角度值
 * @return 两个角度之间的差值 [0, PI)
 */
Float32_t Phoenix_Common_AngleDiff_f(const Float32_t from, const Float32_t to);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_MATH_UTILS_C_H_
