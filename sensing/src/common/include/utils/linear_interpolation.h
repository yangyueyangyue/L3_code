/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       linear_interpolation.h
 * @brief      Linear interpolation functions.
 * @details
 *
 * @author     pengc
 * @date       2018.11.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/18  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_LINEAR_INTERPOLATION_H_
#define PHOENIX_COMMON_LINEAR_INTERPOLATION_H_


#include "utils/macros.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "container/static_vector.h"


namespace phoenix {
namespace common {

struct LerpTableNodeType1 {
  Float32_t key;
  Float32_t value;

  LerpTableNodeType1() : key(0), value(0) {
    // nothing to do
  }

  LerpTableNodeType1(Float32_t k, Float32_t v) : key(k), value(v) {
    // nothing to do
  }
};


/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param x1 The coordinate of the second point.
 * @param t The interpolation parameter for interpolation.
 * @return Interpolated point.
 */
template <typename T>
T Lerp(const T& x0, const T& x1, const Float64_t t) {
  return (x0 + t * (x1 - x0));
}

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param x1 The coordinate of the second point.
 * @param t The interpolation parameter for interpolation.
 * @return Interpolated point.
 */
template <typename T>
T Lerp(const T& x0, const T& x1, const Float32_t t) {
  return (x0 + t * (x1 - x0));
}

/**
 * @brief 对角度进行线性插值
 * @param[in] from 起始角度
 * @param[in] to 终止角度
 * @param[in] t 在起始角度和终止角度之间的插值比例（应当在0和1之间）
 * @return 插值后的角度
 */
inline Float64_t AngleLerp(Float64_t from, Float64_t to, Float64_t t) {
  return (NormalizeAngle(from + t * AngleDiff(from, to)));
}

/**
 * @brief 对角度进行线性插值
 * @param[in] from 起始角度
 * @param[in] to 终止角度
 * @param[in] t 在起始角度和终止角度之间的插值比例（应当在0和1之间）
 * @return 插值后的角度
 */
inline Float32_t AngleLerp(Float32_t from, Float32_t to, Float32_t t) {
  return (NormalizeAngle(from + t * AngleDiff(from, to)));
}

/**
 * @brief 根据指定的键值在有序表中进行线性插值 \n
 *        (有序表中的键值必须从小到大进行排列，且不能有相同的键值)
 * @param[in] table 有序表
 * @param[in] key 指定的键值
 * @param[out] t 插值比例
 * @return 与键值对应的有序表中的索引（这个索引对应的有序表中的键值小于指定键值）
 */
template<typename DataType, Int32_t DataNum, typename Scalar>
Int32_t LerpInOrderedTable(
    const StaticVector<DataType, DataNum>& table, Scalar key, Scalar* t) {
  if (table.Size() < 1) {
    return (-1);
  }

  Int32_t first = 0;
  Int32_t len = table.Size() - 1;
  Int32_t half = 0;
  Int32_t mid = 0;
  Int32_t lower_index = first;

  // 比第一个插值参数小，返回第一个
  if (key <= table[first].key) {
    *t = 0;
    return (lower_index);
  }

  // 比最后一个插值参数大，返回第后一个
  if (key >= table[len].key) {
    lower_index = len;
    *t = 0;
    if (len > 0) {
      lower_index = len - 1;
      *t = 1;
    }
    return (lower_index);
  }

#if 0
  Int32_t iter_count = 0;
  std::cout << "(at beginning) first=" << first
            << ", len=" << len
            << ", half=" << half
            << ", mid=" << mid
            << std::endl;
#endif

  while (len > 0) {
#if 0
    std::cout << ">>> iter_count=" << iter_count++ << std::endl;
#endif

    half = len >> 1;
    mid = first + half;

#if 0
    std::cout << "    update: half=" << half << ", mid=" << mid << std::endl;
#endif

    // 中位数大于待查找的数据, 在左半边序列中查找。
    if (table[mid].key > key) {
      len = half;

#if 0
      std::cout << "    table[" << mid
                << "].key(" << table[mid].key
                << ") > key(" << key
                << "); update: len=" << len
                << std::endl;
#endif
    } else {
      // 中位数小于等于待查找的数据, 在右半边序列中查找。
      first = mid + 1;
      len = len - half - 1;

#if 0
      std::cout << "    table[" << mid
                << "].key(" << table[mid].key
                << ") <= key(" << key
                << "); update: first=" << first
                << ", len=" << len
                << std::endl;
#endif
    }

#if 0
    std::cout << "    after update: first=" << first
              << ", len=" << len
              << ", half=" << half
              << ", mid=" << mid
              << std::endl;
#endif
  }

  if ((first == table.Size() - 1) && !(table[first].key > key)) {
    lower_index = first;
    *t = 0;
    if (first > 0) {
      lower_index = first - 1;
      *t = 1;
    }
    return (lower_index);
  }

  if (first > 0) {
    lower_index = first - 1;
    *t = (key - table[lower_index].key) /
        (table[first].key - table[lower_index].key);
  } else {
    *t = 0;
    lower_index = first;
  }

#if 0
  std::cout << "lower_index="
            << lower_index
            << ", t=" << *t
            << std::endl;
#endif

  return (lower_index);
}


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_LINEAR_INTERPOLATION_H_
