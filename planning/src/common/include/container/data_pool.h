/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       data_pool.h
 * @brief      固定尺寸的内存池
 * @details    在静态分配的内存中动态分配用户数据的内存
 *
 * @author     pengc
 * @date       2020.05.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/19  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_DATA_POOL_H_
#define PHOENIX_COMMON_DATA_POOL_H_


#include "utils/log.h"
#include "utils/macros.h"


// Enable to output debug information
#define ENABLE_DATA_POOL_TRACE (0)


namespace phoenix {
namespace common {

/**
 * @class DataPool
 * @brief 固定尺寸的内存池
 * @param DataType 用户数据类型
 * @param DataNum 最多的数据个数
 */
template <typename DataType, Int32_t DataNum>
class DataPool {
 public:
  typedef DataType data_type;
  typedef size_t size_type;

 public:
  /**
   * @brief 构造函数
   */
  DataPool();
  /**
   * @brief 清除内部数据
   */
  void Clear(void);

  /**
   * @brief 从内存池中分配一块内存（大小为一个用户数据的大小）
   * @return 分配好的内存在内存池中的偏移量（-1表示分配失败）
   */
  Int32_t Allocate();

  /**
   * @brief 释放已经分配好的内存
   * @param[in] offset 分配好的内存在内存池中的偏移量
   * @return true - 成功, false - 失败
   */
  bool Deallocate(Int32_t offset);

  /**
   * @brief 获取已经分配好的内存的实际地址
   * @param[in] offset 分配好的内存在内存池中的偏移量
   * @return 已经分配好的内存的实际地址(nullptr表示获取失败)
   */
  const data_type* GetVirtualAddr(Int32_t offset) const;

  /**
   * @brief 获取已经分配好的内存的实际地址
   * @param[in] offset 分配好的内存在内存池中的偏移量
   * @return 已经分配好的内存的实际地址(nullptr表示获取失败)
   */
  data_type* GetVirtualAddr(Int32_t offset);

  /**
   * @brief 获取已经分配好的内存在内存池中的偏移量
   * @param[in] offset 分配好的内存的实际地址
   * @return 已经分配好的内存在内存池中的偏移量(-1表示获取失败)
   */
  Int32_t GetOffset(data_type* addr) const;

  /**
   * @brief 判断已经分配好的内存在内存池中的偏移量是否无效
   * @param[in] offset 分配好的内存在内存池中的偏移量
   * @return false - 偏移量无效, true - 偏移量有效
   */
  bool IsNull(Int32_t offset) const {
    return (offset < 0);
  }

#if ENABLE_DATA_POOL_TRACE
  /**
   * @brief 调试信息
   */
  std::string DebugString() const;
#endif

 private:
  /*
   * @brief 构建内存池
   */
  void Create(void);

  /*
   * @brief 判断某块内存是否已经被占用
   * @param[in] poffset 分配好的内存在内存池中的偏移量
   * @return false - 没有被占用, true - 已经被占用
   */
  bool IsUsed(const Uint32_t* poffset) const { return (*poffset & 0x40000000); }

  /*
   * @brief 判断某块内存是否没有被占用
   * @param[in] poffset 分配好的内存在内存池中的偏移量
   * @return false - 已经被占用, true - 没有被占用
   */
  bool IsUnused(const Uint32_t* poffset) const { return (!IsUsed(poffset)); }

  /*
   * @brief 将某块内存设置为被占用状态
   * @param[in] poffset 分配好的内存在内存池中的偏移量
   */
  void SetUsed(Uint32_t* poffset) { *poffset |= 0x40000000; }

  /*
   * @brief 将某块内存设置为没有被占用状态
   * @param[in] poffset 分配好的内存在内存池中的偏移量
   */
  void SetUnused(Uint32_t* poffset) { *poffset &= 0xBfffffff; }

  /*
   * @brief 根据内存偏移量获取内存索引
   * @param[in] offset 分配好的内存在内存池中的偏移量
   * @return 内存索引
   */
  Int32_t GetIndex(Uint32_t offset) const { return (offset & 0xBfffffff); }

 private:
  // 内存池中最大数据的个数
  enum { MAX_POOL_SIZE = 0x00FFFFFF };
  // 定义内部使用的空指针
  enum { NULL_PTR = 0xBfffffff };
  // 空闲内存块的链表头
  Int32_t free_;
  // 内存块状态信息
  Uint32_t head_[DataNum];
  // 内存块
  DataType data_[DataNum];
};


/*
* data pool, use offset to point data
*/
template <typename DataType, Int32_t DataNum>
DataPool<DataType, DataNum>::DataPool() {
  COM_CHECK((0 < DataNum) && (DataNum < MAX_POOL_SIZE));

  Create();
}

template <typename DataType, Int32_t DataNum>
void DataPool<DataType, DataNum>::Create(void) {
  Uint32_t* next_head = head_;
  for (Uint32_t i = 0; i < DataNum - 1; ++i) {
    *next_head = i + 1;
    SetUnused(next_head);
    next_head++;
  }
  *next_head = NULL_PTR;
  SetUnused(next_head);
  free_ = 0;
}

template <typename DataType, Int32_t DataNum>
void DataPool<DataType, DataNum>::Clear(void) {
  this->Create();
}

template <typename DataType, Int32_t DataNum>
Int32_t DataPool<DataType, DataNum>::Allocate() {
  if (!IsNull(free_)) {
    Int32_t offset = free_;
    Uint32_t* head_addr = head_ + offset;
    free_ = GetIndex(*head_addr);
    SetUsed(head_addr);

    return (offset);
  }

  return (-1);
}

template <typename DataType, Int32_t DataNum>
bool DataPool<DataType, DataNum>::Deallocate(Int32_t offset) {
  if (IsNull(offset)) {
    return false;
  }

  COM_CHECK(offset < DataNum);

  Uint32_t* head_addr = head_ + offset;
  if (IsUnused(head_addr)) {
    return false;
  }

  *head_addr = free_;
  SetUnused(head_addr);

  free_ = offset;

  return true;
}

template <typename DataType, Int32_t DataNum>
const typename DataPool<DataType, DataNum>::data_type*
DataPool<DataType, DataNum>::GetVirtualAddr(Int32_t offset) const {
  if (IsNull(offset)) {
    return Nullptr_t;
  }

  COM_CHECK(offset < DataNum);

  const Uint32_t* head_addr = head_ + offset;

  if (IsUnused(head_addr)) {
    return Nullptr_t;
  }

  return (data_ + offset);
}

template <typename DataType, Int32_t DataNum>
typename DataPool<DataType, DataNum>::data_type*
DataPool<DataType, DataNum>::GetVirtualAddr(Int32_t offset) {
  if (IsNull(offset)) {
    return Nullptr_t;
  }

  COM_CHECK(offset < DataNum);

  const Uint32_t* head_addr = head_ + offset;

  if (IsUnused(head_addr)) {
    LOG_ERR << "This address[offset=" << offset << "] is not been used.";
    return Nullptr_t;
  }

  return (data_ + offset);
}

template <typename DataType, Int32_t DataNum>
Int32_t DataPool<DataType, DataNum>::GetOffset(data_type* addr) const {
  if (Nullptr_t == addr) {
    return (-1);
  }

  Int32_t offset = ((size_type)addr - (size_type)data_) / sizeof(DataType);
  Int32_t offset_remainder =
      ((size_type)addr - (size_type)data_) % sizeof(DataType);
  if (0 != offset_remainder) {
    LOG_ERR << "Detected invalid addr, offset_remainder=" << offset_remainder;
    return (-1);
  }

  // COM_CHECK((0 <= offset) && (offset < DataNum));
  if ((offset < 0) || (offset >= DataNum)) {
    LOG_ERR << "Detected invalid offset(" << offset << ").";
    return (-1);
  }

  const Uint32_t* head_addr = head_ + offset;

  if (IsUnused(head_addr)) {
    return (-1);
  }

  return (offset);
}

#if ENABLE_DATA_POOL_TRACE
template <typename DataType, Int32_t DataNum>
std::string DataPool<DataType, DataNum>::DebugString() const {
  std::ostringstream os;

  os << "-------------------- Data Pool -------------------------->"
     << std::endl;

  const Uint32_t* next_head = head_;

  os << "**All:\n";
  for (Uint32_t i = 0; i < DataNum; ++i) {
    os << "**No." << i
       << " : head_addr(" << reinterpret_cast<const void*>(next_head)
       << "),data_addr("
       << reinterpret_cast<const void*>(data_ + i)
       << "),used?(" << IsUsed(next_head)
       << "),next_node_index(" << reinterpret_cast<const void*>(*next_head)
       << ")\n";
    next_head++;
  }
  os << "**Free:\n";
  Int32_t index = free_;
  Int32_t i = 0;
  while (!IsNull(index)) {
    next_head = head_ + index;
    os << "**No." << i
       << " : node_addr(" << next_head
       << "),data_addr("
       << reinterpret_cast<const void*>(data_ + index)
       << "),used?(" << IsUsed(next_head)
       << "),next_node_index(" << reinterpret_cast<const void*>(*next_head)
       << ")\n";
    index = GetIndex(*next_head);
    i++;
  }

  os << "<------------------- Data Pool ---------------------------"
     << std::endl;

  return (os.str());
}
#endif


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_DATA_POOL_H_

