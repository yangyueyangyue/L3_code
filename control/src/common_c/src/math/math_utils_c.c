/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       math_utils.cc
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

#include "math/math_utils_c.h"

#define TRIANGLE_DIVISION_SIZE (10000)
#define TRIANGLE_LOOKUP_TABLE_SIZE (TRIANGLE_DIVISION_SIZE+1)
#define TRIANGLE_DIVISION_SIZE_ATAN (40000)
#define TRIANGLE_LOOKUP_TABLE_SIZE_ATAN (TRIANGLE_DIVISION_SIZE_ATAN+1)

static const Int32_t s_triangle_division_size = TRIANGLE_DIVISION_SIZE;
static const Int32_t s_triangle_lookup_table_size = TRIANGLE_LOOKUP_TABLE_SIZE;
static Float64_t s_asin_lookup_table[TRIANGLE_LOOKUP_TABLE_SIZE] = { 0.0 };
static Float64_t s_cos_lookup_table[TRIANGLE_LOOKUP_TABLE_SIZE] = { 0.0 };
static Float64_t s_sin_lookup_table[TRIANGLE_LOOKUP_TABLE_SIZE] = { 0.0 };
static const Int32_t s_triangle_division_size_atan = TRIANGLE_DIVISION_SIZE_ATAN;
static const Int32_t s_triangle_lookup_table_size_atan = TRIANGLE_LOOKUP_TABLE_SIZE_ATAN;
static Float64_t s_atan_lookup_table[TRIANGLE_LOOKUP_TABLE_SIZE_ATAN] = { 0.0 };

void Phoenix_Common_CreateTriangleLookupTables() {
  Int32_t i = 0;
  for (i = 0; i < s_triangle_lookup_table_size; ++i) {
    s_asin_lookup_table[i] =
        asin((Float64_t)i / (Float64_t)s_triangle_division_size);
  }

  for (i = 0; i < s_triangle_lookup_table_size; ++i) {
    s_cos_lookup_table[i] =
        cos(COM_PI_2*(Float64_t)i / (Float64_t)s_triangle_division_size);
  }

  for (i = 0; i < s_triangle_lookup_table_size; ++i) {
    s_sin_lookup_table[i] =
        sin(COM_PI_2*(Float64_t)i / (Float64_t)s_triangle_division_size);
  }

  for (i = 0; i < s_triangle_lookup_table_size_atan; ++i) {
    s_atan_lookup_table[i] =
        atan((Float64_t)i / (Float64_t)s_triangle_division_size_atan);
  }
}

static Float64_t LookupSinFromTab(Float64_t value) {
  if ((0 <= value) && (value <= COM_PI_2)) {
    return (s_sin_lookup_table[
            phoenix_com_rint_d(value * s_triangle_division_size / COM_PI_2)]);
  } else if ((COM_PI_2 < value) && (value <= COM_PI)) {
    return (s_sin_lookup_table[
            phoenix_com_rint_d((COM_PI-value) * s_triangle_division_size / COM_PI_2)]);
  } else if ((COM_PI < value) && (value <= 3.0*COM_PI_2)) {
    return (-s_sin_lookup_table[
            phoenix_com_rint_d((value-COM_PI) * s_triangle_division_size / COM_PI_2)]);
  } else if ((3.0*COM_PI_2 < value) && (value <= 2.0*COM_PI)) {
    return (-s_sin_lookup_table[
            phoenix_com_rint_d((2.0*COM_PI-value) * s_triangle_division_size / COM_PI_2)]);
  } else {
    return 0.0;
  }
}

Float64_t Phoenix_Common_SinLookUp(Float64_t value) {
  if ((value < -COM_PI) || (value > COM_PI)) {
    value = Phoenix_Common_NormalizeAngle_f(value);
  }

  if (value < 0.0) {
    return (-LookupSinFromTab(-value));
  } else {
    return (LookupSinFromTab(value));
  }
}

Float64_t Phoenix_Common_CosLookUp(Float64_t value) {
  if ((value < -COM_PI) || (value > COM_PI)) {
    value = Phoenix_Common_NormalizeAngle_f(value);
  }

  if (value < 0.0) {
    value = -value;
  }

  if ((0.0 <= value) && (value <= COM_PI_2)) {
    return (s_cos_lookup_table[
            phoenix_com_rint_d(value * s_triangle_division_size / COM_PI_2)]);
  } else if ((COM_PI_2 < value) && (value <= COM_PI)) {
    return (-s_cos_lookup_table[
            phoenix_com_rint_d((COM_PI-value) * s_triangle_division_size / COM_PI_2)]);
  } else if ((COM_PI < value) && (value <= 3.0*COM_PI_2)) {
    return (-s_cos_lookup_table[
            phoenix_com_rint_d((value-COM_PI) * s_triangle_division_size / COM_PI_2)]);
  } else if ((3.0*COM_PI_2 < value) && (value <= 2.0*COM_PI)) {
    return (s_cos_lookup_table[
            phoenix_com_rint_d((2.0*COM_PI-value) * s_triangle_division_size / COM_PI_2)]);
  } else {
    return 0.0;
  }
}

Float64_t Phoenix_Common_AsinLookUp(Float64_t value) {
  if (value < -(1.0-FLT_EPSILON)) {
    return (-COM_PI_2);
  } else if (value > (1.0-FLT_EPSILON)) {
    return (COM_PI_2);
  } else {
    if (value < 0.0) {
      return (-s_asin_lookup_table[
              phoenix_com_rint_d(-value * s_triangle_division_size)]);
    } else {
      return (s_asin_lookup_table[
              phoenix_com_rint_d(value * s_triangle_division_size)]);
    }
  }
}

static Float64_t LookupAtanFromTab(Float64_t abs_y, Float64_t abs_x) {
  return (s_atan_lookup_table[phoenix_com_rint_d(
      abs_y * s_triangle_division_size_atan / abs_x)]);
}

Float64_t Phoenix_Common_Atan2LookUp(Float64_t y, Float64_t x) {
  Float64_t abs_x = fabs(x);
  Float64_t abs_y = fabs(y);
  if ((abs_x < FLT_EPSILON) && (abs_y < FLT_EPSILON)) {
    return (0.0);
  }

  if ((x >= 0.0) && (y >= 0.0)) {
    if (abs_y < abs_x) {
      return (LookupAtanFromTab(abs_y, abs_x));
    } else {
      return (COM_PI_2 - LookupAtanFromTab(abs_x, abs_y));
    }
  } else if ((x < 0.0) && (y >= 0.0)) {
    if (abs_y < abs_x) {
      return (COM_PI - LookupAtanFromTab(abs_y, abs_x));
    } else {
      return (COM_PI_2 + LookupAtanFromTab(abs_x, abs_y));
    }
  } else if ((x < 0.0) && (y < 0.0)) {
    if (abs_y < abs_x) {
      return (LookupAtanFromTab(abs_y, abs_x) - COM_PI);
    } else {
      return (-COM_PI_2 - LookupAtanFromTab(abs_x, abs_y));
    }
  } else {
    if (abs_y < abs_x) {
      return (-LookupAtanFromTab(abs_y, abs_x));
    } else {
      return (-COM_PI_2 + LookupAtanFromTab(abs_x, abs_y));
    }
  }
}

Float32_t Phoenix_Common_WrapAngle_f(const Float32_t angle) {
  const Float32_t new_angle = fmod(angle, COM_PI * 2.0);
  return new_angle < 0.0F ? new_angle + (COM_PI * 2.0) : new_angle;
}

Float32_t Phoenix_Common_NormalizeAngle_f(const Float32_t angle) {
  Float32_t a = fmod(angle + COM_PI, 2.0F * COM_PI);
  if (a < 0.0F) {
    a += 2.0F * COM_PI;
  }
  return (a - COM_PI);
}

Float32_t Phoenix_Common_AngleDiff_f(const Float32_t from, const Float32_t to) {
  return Phoenix_Common_NormalizeAngle_f(to - from);
}

