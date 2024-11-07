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

#include "math/math_utils.h"

#include <utility>

namespace phoenix {
namespace common {


static const Int32_t s_triangle_division_size = 10000;
static const Int32_t s_triangle_lookup_table_size = s_triangle_division_size+1;
static Float64_t s_asin_lookup_table[s_triangle_lookup_table_size] = { 0.0 };
static Float64_t s_cos_lookup_table[s_triangle_lookup_table_size] = { 0.0 };
static Float64_t s_sin_lookup_table[s_triangle_lookup_table_size] = { 0.0 };
static const Int32_t s_triangle_division_size_atan = 40000;
static const Int32_t s_triangle_lookup_table_size_atan =
    s_triangle_division_size_atan+1;
static Float64_t s_atan_lookup_table[s_triangle_lookup_table_size_atan] = { 0.0 };

void CreateTriangleLookupTables() {
  for (Int32_t i = 0; i < s_triangle_lookup_table_size; ++i) {
    s_asin_lookup_table[i] = asin(static_cast<Float64_t>(i)
        / static_cast<Float64_t>(s_triangle_division_size));
  }

  for (Int32_t i = 0; i < s_triangle_lookup_table_size; ++i) {
    s_cos_lookup_table[i] = cos(static_cast<Float64_t>(COM_PI_2*i
        / static_cast<Float64_t>(s_triangle_division_size)));
  }

  for (Int32_t i = 0; i < s_triangle_lookup_table_size; ++i) {
    s_sin_lookup_table[i] = sin(static_cast<Float64_t>(COM_PI_2*i
        / static_cast<Float64_t>(s_triangle_division_size)));
  }

  for (Int32_t i = 0; i < s_triangle_lookup_table_size_atan; ++i) {
    s_atan_lookup_table[i] = static_cast<Float64_t>(
          atan(static_cast<Float64_t>(i)
        / static_cast<Float64_t>(s_triangle_division_size_atan)));
  }
}

Float64_t SinLookUp(Float64_t value) {
  struct LookupSin {
    Float64_t operator() (Float64_t value) {
      if ((0 <= value) && (value <= COM_PI_2)) {
        return (s_sin_lookup_table[com_rint(
            value*s_triangle_division_size/static_cast<Float64_t>(COM_PI_2))]);
      } else if ((COM_PI_2 < value) && (value <= COM_PI)) {
        return (s_sin_lookup_table[com_rint(
            (COM_PI-value)*s_triangle_division_size
            / static_cast<Float64_t>(COM_PI_2))]);
      } else if ((COM_PI < value) && (value <= 3.0*COM_PI_2)) {
        return (-s_sin_lookup_table[com_rint(
            (value-COM_PI)*s_triangle_division_size
            / static_cast<Float64_t>(COM_PI_2))]);
      } else if ((3.0*COM_PI_2 < value) && (value <= 2.0*COM_PI)) {
        return (-s_sin_lookup_table[com_rint(
            (2.0*COM_PI-value)*s_triangle_division_size
            / static_cast<Float64_t>(COM_PI_2))]);
      } else {
        return 0.0;
      }
    }
  };
  LookupSin lookupSin;

  if ((value < -COM_PI) || (value > COM_PI)) {
    value = NormalizeAngle(value);
  }

  if (value < 0.0) {
    return (-lookupSin(-value));
  } else {
    return (lookupSin(value));
  }
}

Float64_t CosLookUp(Float64_t value) {
  if ((value < -COM_PI) || (value > COM_PI)) {
    value = NormalizeAngle(value);
  }

  if (value < 0.0) {
    value = -value;
  }

  if ((0.0 <= value) && (value <= COM_PI_2)) {
    return (s_cos_lookup_table[com_rint(
        value*s_triangle_division_size/static_cast<Float64_t>(COM_PI_2))]);
  } else if ((COM_PI_2 < value) && (value <= COM_PI)) {
    return (-s_cos_lookup_table[com_rint(
        (COM_PI-value)*s_triangle_division_size/
          static_cast<Float64_t>(COM_PI_2))]);
  } else if ((COM_PI < value) && (value <= 3.0*COM_PI_2)) {
    return (-s_cos_lookup_table[com_rint(
        (value-COM_PI)*s_triangle_division_size/
          static_cast<Float64_t>(COM_PI_2))]);
  } else if ((3.0*COM_PI_2 < value) && (value <= 2.0*COM_PI)) {
    return (s_cos_lookup_table[com_rint(
        (2.0*COM_PI-value)*s_triangle_division_size
        / static_cast<Float64_t>(COM_PI_2))]);
  } else {
    return 0.0;
  }
}

Float64_t AsinLookUp(Float64_t value) {
  if (value < -static_cast<Float64_t>(1.0-FLT_EPSILON)) {
    return static_cast<Float64_t>(-COM_PI_2);
  } else if (value > static_cast<Float64_t>(1.0-FLT_EPSILON)) {
    return static_cast<Float64_t>(COM_PI_2);
  } else {
    if (value < 0.0) {
      return (-s_asin_lookup_table[
          com_rint(-value*s_triangle_division_size)]);
    } else {
      return (s_asin_lookup_table[com_rint(value*s_triangle_division_size)]);
    }
  }
}

Float64_t Atan2LookUp(Float64_t y, Float64_t x) {
  struct LookupAtan {
    Float64_t operator() (Float64_t abs_y, Float64_t abs_x) {
      return (s_atan_lookup_table[com_rint(
          abs_y*s_triangle_division_size_atan/abs_x)]);
    }
  };

  Float64_t abs_x = std::abs(x);
  Float64_t abs_y = std::abs(y);
  if ((abs_x < static_cast<Float64_t>(FLT_EPSILON)) &&
      (abs_y < static_cast<Float64_t>(FLT_EPSILON))) {
    return (0.0);
  }

  LookupAtan lookupAtan;
  if ((x >= 0.0) && (y >= 0.0)) {
    if (abs_y < abs_x) {
      return (lookupAtan(abs_y, abs_x));
    } else {
      return (static_cast<Float64_t>(COM_PI_2) - lookupAtan(abs_x, abs_y));
    }
  } else if ((x < 0.0) && (y >= 0.0)) {
    if (abs_y < abs_x) {
      return (static_cast<Float64_t>(COM_PI) - lookupAtan(abs_y, abs_x));
    } else {
      return (static_cast<Float64_t>(COM_PI_2) + lookupAtan(abs_x, abs_y));
    }
  } else if ((x < 0.0) && (y < 0.0)) {
    if (abs_y < abs_x) {
      return (lookupAtan(abs_y, abs_x) - static_cast<Float64_t>(COM_PI));
    } else {
      return (-static_cast<Float64_t>(COM_PI_2) - lookupAtan(abs_x, abs_y));
    }
  } else {
    if (abs_y < abs_x) {
      return (-lookupAtan(abs_y, abs_x));
    } else {
      return (-static_cast<Float64_t>(COM_PI_2) + lookupAtan(abs_x, abs_y));
    }
  }
}

Float64_t WrapAngle(const Float64_t angle) {
  const Float64_t new_angle = std::fmod(angle, COM_PI * 2.0);
  return new_angle < 0.0 ? new_angle + COM_PI * 2.0 : new_angle;
}

Float32_t WrapAngle(const Float32_t angle) {
  const Float32_t new_angle = std::fmod(angle,
    static_cast<Float32_t>(COM_PI * 2.0));
  return new_angle < 0.0F ? new_angle +
    static_cast<Float32_t>(COM_PI * 2.0) : new_angle;
}

Float64_t NormalizeAngle(const Float64_t angle) {
  Float64_t a = std::fmod(angle + COM_PI, 2.0 * COM_PI);
  if (a < 0.0) {
    a += (2.0 * COM_PI);
  }
  return a - COM_PI;
}

Float32_t NormalizeAngle(const Float32_t angle) {
  Float32_t a = std::fmod(angle + static_cast<Float32_t>(COM_PI),
    2.0F * static_cast<Float32_t>(COM_PI));
  if (a < 0.0F) {
    a += 2.0F * static_cast<Float32_t>(COM_PI);
  }
  return a - static_cast<Float32_t>(COM_PI);
}

Float64_t AngleDiff(const Float64_t from, const Float64_t to) {
  return NormalizeAngle(to - from);
}

Float32_t AngleDiff(const Float32_t from, const Float32_t to) {
  return NormalizeAngle(to - from);
}


}  // namespace common
}  // namespace phoenix
