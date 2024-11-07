/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       static_vector.h
 * @brief      静态数组
 * @details    实现了部分标准库std中vector的用法，用来降低使用vector的push_back等
 *             函数时频繁分配内存的问题。
 *
 * @author     pengc,boc
 * @date       2020.05.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/15  <td>1.0      <td>pengc,boc <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_COMMON_STATIC_VECTOR_H_
#define PHOENIX_COMMON_STATIC_VECTOR_H_

#include "utils/log.h"
#include "utils/macros.h"

namespace phoenix {
namespace common {


/**
 * @class StaticVector
 * @param DataType 数据类型
 * @param DataNum 静态数组中元素的最大个数
 * @brief 静态数组
 */
template <typename DataType, Int32_t DataNum>
class StaticVector {
public:
  /**
   * @brief 构造函数
   */
  StaticVector() {
    count_ = 0;
  }

  const DataType* data() const { return (data_); }
  DataType* data() { return (data_); }

  /**
   * @brief 清除内部数据
   */
  inline void Clear() {
    count_ = 0;
  }

  /**
   * @brief 获取静态数组的元素个数
   * @return 静态数组的元素个数
   */
  inline Int32_t Size() const {
    return (count_);
  }

  /**
   * @brief 判断数组是否为空
   * @return true - 数组是空的, false - 数组不是空的
   */
  inline bool Empty() const {
    return (count_ < 1);
  }

  /**
   * @brief 判断数组是否已经存满了
   * @return true - 数组是满的, false - 数组不是满的
   */
  inline bool Full() const {
    return (count_ >= DataNum);
  }

  /**
   * @brief 更改内部元素的数量
   * @param[in] size - 需要被更改的元素的数量
   */
  inline void Resize(Int32_t size) {
    if (size > DataNum) {
      size = DataNum;
    } else if (size < 0) {
      size = 0;
    }

    /* k001 2020-05-16 pengc (begin) */
    // 之前不允许减小当前保存的数据量，修改为可以减小当前保存的数据量
    count_ = size;
    /* k001 2020-05-16 pengc (end) */
  }

  /**
   * @brief 向静态数组的数据尾端添加一个元素。
   * @param[in] data 待添加的元素
   * @return true 成功；false 失败。
   * @note 当静态数组的元素个数已经达到最大个数时，会添加失败，\n
   *       此时静态数组的元素个数会保持不变。
   */
  inline bool PushBack(const DataType& data) {
    if (count_ < DataNum) {
      data_[count_] = data;
      count_++;

      return true;
    }

    return false;
  }

  /**
   * @brief 从数组中分配一块空间
   * @return 分配的空间的地址（nullptr为分配失败）
   */
  inline DataType* Allocate() {
    if (count_ < DataNum) {
      count_++;

      return &(data_[count_-1]);
    }

    return Nullptr_t;
  }

  /**
   * @brief 从数组中分配一块空间
   * @param[out] index 分配的空间在数组中的索引
   * @return 分配的空间的地址（nullptr为分配失败）
   */
  inline DataType* Allocate(Int32_t* index) {
    if (count_ < DataNum) {
      count_++;

      *index = count_-1;
      return &(data_[count_-1]);
    }

    *index = -1;
    return Nullptr_t;
  }

  /**
   * @brief 获取数组索引所指向的数据
   * @param[in] index 数组索引
   * @return 数组索引所指向的数据
   */
  inline const DataType& operator [](Int32_t index) const {
    COM_CHECK(index >= 0);
    COM_CHECK(index < count_);

    return (data_[index]);
  }

  /**
   * @brief 获取数组索引所指向的数据
   * @param[in] index 数组索引
   * @return 数组索引所指向的数据
   */
  inline DataType& operator [](Int32_t index) {
    COM_CHECK(index >= 0);
    COM_CHECK(index < count_);

    return (data_[index]);
  }

  /**
   * @brief 获取数组索引所指向的数据
   * @param[in] index 数组索引
   * @return 数组索引所指向的数据
   */
  inline const DataType& GetData(Int32_t index) const {
    COM_CHECK(index >= 0);
    COM_CHECK(index < count_);

    return (data_[index]);
  }

  /**
   * @brief 获取数组索引所指向的数据
   * @param[in] index 数组索引
   * @return 数组索引所指向的数据
   */
  inline DataType& GetData(Int32_t index) {
    COM_CHECK(index >= 0);
    COM_CHECK(index < count_);

    return (data_[index]);
  }

  /**
   * @brief 获取数组中第一个数据
   * @return 数组中第一个数据
   */
  inline DataType& Front() {
    COM_CHECK(count_ > 0);
    return (data_[0]);
  }

  /**
   * @brief 获取数组中最后一个数据
   * @return 数组中最后一个数据
   */
  inline DataType& Back() {
    COM_CHECK(count_ > 0);
    return (data_[count_-1]);
  }

  /**
   * @brief 获取数组中第一个数据
   * @return 数组中第一个数据
   */
  inline const DataType& Front() const {
    COM_CHECK(count_ > 0);
    return (data_[0]);
  }

  /**
   * @brief 获取数组中最后一个数据
   * @return 数组中最后一个数据
   */
  inline const DataType& Back() const {
    COM_CHECK(count_ > 0);
    return (data_[count_-1]);
  }

  /**
   * @brief 移除数组中最后一个数据
   */
  inline void PopBack() {
    if (count_ > 0) {
      count_--;
    }
  }

  /**
   * @brief 返回第一个 “大于等于 data”的元素位置。
   * @param[in] data 待查找的数据
   * @return 第一个 “大于等于 data”的元素位置。
   * @note 1、当所有元素均比data小时，返回值为最后一个元素的位置。\n
   *       2、cmp_func需要定义为‘>’。
   */
  template <typename CmpFunc>
  Int32_t LowerBound(const DataType& data, const CmpFunc& cmp_func) const;
  /**
   * @brief 返回第一个 “大于 data”的元素位置。
   * @param[in] data 待查找的数据
   * @return 第一个 “大于 data”的元素位置。
   * @note 1、当所有元素均比data小时，返回值为元素个数。\n
   *       2、cmp_func需要定义为‘>’。
   */
  template <typename CmpFunc>
  Int32_t UpperBound(const DataType& data, const CmpFunc& cmp_func) const;

  /**
   * @brief 用冒泡法排序容器中的元素
   * @param[in] 比较类
   * @return true or false
   * @note cmp_func需要定义'<'
   * @par Note:
   * @code
   *     cmp_func需要重载'<'操作符, 例如：
   *     template <typename DataType, Int32_t DataNum>
   *     class CmpFunc {
   *     public:
   *       bool operator ()(const DataType& data1, const DataType& data2) const {
   *         return (data1 < data2);
   *       }
   *     };
   * @endcode
   */
  template <typename CmpFunc>
  bool Sort(const CmpFunc& cmp_func);

  /**
   * @brief 查找容器中的最小元素
   * @param[in] 比较类
   * @return true or false
   * @note cmp_func需要定义'<'
   * @par Note:
   * @code
   *     cmp_func需要重载'<'操作符, 例如：
   *     template <typename DataType, Int32_t DataNum>
   *     class CmpFunc {
   *     public:
   *       bool operator ()(const DataType& data1, const DataType& data2) const {
   *         return (data1 < data2);
   *       }
   *     };
   * @endcode
   */
  template <typename CmpFunc>
  Int32_t FindMinElement(const CmpFunc& cmp_func);

  /**
   * @brief 查找容器中的最大元素
   * @param[in] 比较类
   * @return true or false
   * @note cmp_func需要定义'<'
   * @par Note:
   * @code
   *     cmp_func需要重载'<'操作符, 例如：
   *     template <typename DataType, Int32_t DataNum>
   *     class CmpFunc {
   *     public:
   *       bool operator ()(const DataType& data1, const DataType& data2) const {
   *         return (data1 < data2);
   *       }
   *     };
   * @endcode
   */
  template <typename CmpFunc>
  Int32_t FindMaxElement(const CmpFunc& cmp_func);

private:
  /// 元素个数
  Int32_t count_;
  /// 存储的数据
  DataType data_[DataNum];
};


template <typename DataType, Int32_t DataNum>
template <typename CmpFunc>
Int32_t StaticVector<DataType, DataNum>::LowerBound(
    const DataType& data, const CmpFunc& cmp_func) const {
  Int32_t first = 0;
  Int32_t mid = 0;
  Int32_t half = 0;
  Int32_t len = count_;

  while (len > 0) {
    half = len >> 1;
    mid = first + half;
    if (cmp_func(data, data_[mid])) {
      first = mid + 1;
      // 在右边子序列中查找
      len = len - half - 1;
    } else {
      // 在左边子序列（包含mid）中查找
      len = half;
    }
  }

  return first;
}

template <typename DataType, Int32_t DataNum>
template <typename CmpFunc>
Int32_t StaticVector<DataType, DataNum>::UpperBound(
    const DataType& data, const CmpFunc& cmp_func) const {
  Int32_t first = 0;
  Int32_t len = count_ - 1;
  Int32_t half = 0;
  Int32_t mid = 0;

  while (len > 0) {
    half = len >> 1;
    mid = first + half;
    // 中位数大于待查找的数据, 在左半边序列中查找。
    if (cmp_func(data_[mid], data)) {
      len = half;
    } else {
      // 中位数小于等于待查找的数据, 在右半边序列中查找。
      first = mid + 1;
      len = len - half - 1;
    }
  }

  if ((first == count_ - 1) && (!cmp_func(data_[first], data))) {
    first++;
  }

  return first;
}

template <typename DataType, Int32_t DataNum>
template <typename CmpFunc>
bool StaticVector<DataType, DataNum>::Sort(const CmpFunc& cmp_func) {
  // 元素数量
  Int32_t n = count_;
  DataType temp;

  // 空容器返回false
  if (n < 1) {
    return false;
  }

  for (Int32_t i = 0; i < (n-1); ++i) {
    for (Int32_t j = 0; j < (n-i-1); ++j) {
      if (false == cmp_func(data_[j], data_[j+1])) {
        temp = data_[j];
        data_[j] = data_[j+1];
        data_[j+1] = temp;
      }
    }
  }

  return true;
}

template <typename DataType, Int32_t DataNum>
template <typename CmpFunc>
Int32_t StaticVector<DataType, DataNum>::FindMinElement(
    const CmpFunc& cmp_func) {
  Int32_t element_num = count_;
  if (element_num < 1) {
    return (-1);
  }

  Int32_t min_ele_index = 0;
  for (Int32_t i = 1; i < element_num; ++i) {
    if (cmp_func(data_[i], data_[min_ele_index])) {
      min_ele_index = i;
    }
  }

  return (min_ele_index);
}

template <typename DataType, Int32_t DataNum>
template <typename CmpFunc>
Int32_t StaticVector<DataType, DataNum>::FindMaxElement(
    const CmpFunc& cmp_func) {
  Int32_t element_num = count_;
  if (element_num < 1) {
    return (-1);
  }

  Int32_t max_ele_index = 0;
  for (Int32_t i = 1; i < element_num; ++i) {
    if (cmp_func(data_[max_ele_index], data_[i])) {
      max_ele_index = i;
    }
  }

  return (max_ele_index);
}


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_STATIC_VECTOR_H_


