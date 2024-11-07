/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       ring_buffer.h
 * @brief      环形数据缓冲区
 * @details    定义环形数据缓冲区
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

#ifndef PHOENIX_COMMON_RING_BUFFER_H_
#define PHOENIX_COMMON_RING_BUFFER_H_

#include "utils/macros.h"
#include "utils/log.h"


namespace phoenix {
namespace common {


/**
 * @class RingBuffer
 * @brief 环形数据缓冲区
 * @param DataType 需要存入缓冲区的数据类型
 * @param DataNum 环形缓冲区可以保存的数据的最大数量
 */
template<typename DataType, Int32_t DataNum>
class RingBuffer {
 public:
  /**
   * @brief 构造函数
   */
  RingBuffer();

  /**
   * @brief 析构函数
   */
  ~RingBuffer();

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 判断环形数据缓冲区是否为空
   * @return true - 空的, false - 不是空的
   */
  bool Empty() const { return (data_count_ < 1); }

  /**
   * @brief 判断环形数据缓冲区是否已经存满了
   * @return true - 满的, false - 不是满的
   */
  bool Full() const {
    return (data_count_ >= DataNum);
  }

  /**
   * @brief 获取环形数据缓冲区内已有数据的数量
   * @return 环形数据缓冲区内已有数据的数量
   */
  Int32_t Size() const { return (data_count_); }

  /**
   * @brief 向环形数据缓冲区的尾部添加一个数据（若缓冲区已满，则不添加）
   * @param[in] data 待添加的数据
   * @return true-添加成功, false-添加失败
   */
  bool PushBack(const DataType& data);

  /**
   * @brief 从环形数据缓冲区中分配一块空间（若缓冲区已满，则不分配）
   * @return 分配的空间的地址（Nullptr_t-分配失败)
   */
  DataType* Allocate();

  /**
   * @brief 向环形数据缓冲区的尾部添加一个数据（若缓冲区已满，则覆盖之前的数据）
   * @param[in] data 待添加的数据
   * @return true-添加成功, false-添加失败
   */
  bool PushBackOverride(const DataType& data);

  /**
   * @brief 从环形数据缓冲区中分配一块空间（若缓冲区已满，则覆盖之前的数据）
   * @return 分配的空间的地址
   */
  DataType* AllocateOverride();

  /**
   * @brief 获取环形数据缓冲区头部的数据，并从缓冲区中移除此数据
   * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
   */
  const DataType* PopFront();

  /**
   * @brief 从环形数据缓冲区头部开始移除指定数量的数据
   * @param[in] num 要移除的数据的数量
   */
  void PopFromFront(Int32_t num);

  /**
   * @brief 获取环形数据缓冲区尾部的数据，并从缓冲区中移除此数据
   * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
   */
  const DataType* PopBack();

  /**
   * @brief 从环形数据缓冲区尾部开始移除指定数量的数据
   * @param[in] num 要移除的数据的数量
   */
  void PopFromBack(Int32_t num);

  /**
   * @brief 获取环形数据缓冲区头部的数据
   * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
   */
  const DataType* Front() const;

  /**
   * @brief 获取环形数据缓冲区头部的数据
   * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
   */
  DataType* Front();

  /**
   * @brief 获取环形数据缓冲区尾部的数据
   * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
   */
  const DataType* Back() const;

  /**
   * @brief 获取环形数据缓冲区尾部的数据
   * @return 返回的数据, (Nullptr_t-缓冲区内无数据)
   */
  DataType* Back();

  /**
   * @brief 获取数据索引所指向的数据
   * @param[in] index 数据索引
   * @return 数据索引所指向的数据
   */
  const DataType& operator [](Int32_t index) const {
    COM_CHECK((0 <= index) && (index < Size()));

    Int32_t real_index = BoundIndex(read_position_, index);
    return data_buff_[real_index];
  }

  /**
   * @brief 获取数据索引所指向的数据
   * @param[in] index 数据索引
   * @return 数据索引所指向的数据
   */
  DataType& operator [](Int32_t index) {
    COM_CHECK((0 <= index) && (index < Size()));

    Int32_t real_index = BoundIndex(read_position_, index);
    return data_buff_[real_index];
  }

  /**
   * @class iterator
   * @brief 环形数据缓冲区的迭代器
   */
  class iterator;
  friend class iterator;
  class iterator {
   private:
    // 指向环形数据缓冲区的指针
    RingBuffer* ring_buff_;
    // 指向环形数据缓冲区的索引
    Int32_t ring_buff_index_;
    // 环形缓冲区是否已满
    bool is_ring_buff_full_;

   public:
    /**
     * @brief 构造函数
     * @param[in] ring_buff 指向环形数据缓冲区的指针
     * @param[in] index 环形数据缓冲区某个位置的索引（通常是保存的数据起始位置，或结束位置）
     * @param[in] is_full 当前环形数据缓冲区是否是满的
     */
    iterator(RingBuffer* ring_buff, Int32_t index, bool is_full)
        : ring_buff_(ring_buff)
        , ring_buff_index_(index)
        , is_ring_buff_full_(is_full) {
      // nothing to do
    }
    /**
     * @brief 拷贝构造函数
     * @param[in] rv 另一个迭代器
     */
    iterator(const iterator& rv)
        : ring_buff_(rv.ring_buff_)
        , ring_buff_index_(rv.ring_buff_index_)
        , is_ring_buff_full_(rv.is_ring_buff_full_) {
      // nothing to do
    }
    /**
     * @brief 拷贝函数
     * @param[in] rv 另一个迭代器
     */
    iterator& operator=(const iterator& rv) {
      ring_buff_ = rv.ring_buff_;
      ring_buff_index_ = rv.ring_buff_index_;
      is_ring_buff_full_ = rv.is_ring_buff_full_;
      return *this;
    }
    /**
     * @brief 迭代器自增
     * @return 自增后的迭代器
     */
    iterator& operator++() {
      ++ring_buff_index_;
      is_ring_buff_full_ = false;
      if (ring_buff_index_ >= DataNum) {
        ring_buff_index_ -= DataNum;
      }
      return *this;
    }
    /**
     * @brief 正方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    iterator operator +(Int32_t offset) const {
      iterator it = *this;

      offset %= DataNum;
      if (0 == offset || ring_buff_->Empty()) {
        return (it);
      }

      if (ring_buff_->write_position_ == ring_buff_->read_position_) {
        if (ring_buff_index_ == ring_buff_->write_position_) {
          if (offset > 0 && !is_ring_buff_full_) {
            return (it);
          }
          if (offset < 0 && is_ring_buff_full_) {
            return (it);
          }
        }
      }

      Int32_t index = ring_buff_->BoundIndex(ring_buff_index_, offset);
      if (offset < 0 && index == ring_buff_->read_position_) {
        it.is_ring_buff_full_ = ring_buff_->Full();
      } else {
        it.is_ring_buff_full_ = true;
      }

      it.ring_buff_index_ = index;

      return (it);
    }
    /**
     * @brief 负方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    iterator operator -(Int32_t offset) const {
      return this->operator +(-offset);
    }

    /**
     * @brief 正方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    iterator& operator +=(Int32_t offset) {
      offset %= DataNum;
      if (0 == offset || ring_buff_->Empty()) {
        return (*this);
      }

      if (ring_buff_->write_position_ == ring_buff_->read_position_) {
        if (ring_buff_index_ == ring_buff_->write_position_) {
          if (offset > 0 && !is_ring_buff_full_) {
            return (*this);
          }
          if (offset < 0 && is_ring_buff_full_) {
            return (*this);
          }
        }
      }

      Int32_t index = ring_buff_->BoundIndex(ring_buff_index_, offset);
      if (offset < 0 && index == ring_buff_->read_position_) {
        is_ring_buff_full_ = ring_buff_->Full();
      } else {
        is_ring_buff_full_ = true;
      }

      ring_buff_index_ = index;

      return (*this);
    }
    /**
     * @brief 负方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    iterator& operator -=(Int32_t offset) {
      return this->operator +=(-offset);
    }

    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    DataType* current() const {
      return &(ring_buff_->data_buff_[ring_buff_index_]);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    DataType& operator*() const {
      return *current();
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    DataType* operator->() const {
      return current();
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器相等
     * @param[in]  rv 另一个迭代器
     * @return true-相等, false-不相等
     */
    bool operator==(const iterator& rv) const {
      return ((ring_buff_index_ == rv.ring_buff_index_) &&
              (false == is_ring_buff_full_));
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器不相等
     * @param[in]  rv 另一个迭代器
     * @return true-不相等, false-相等
     */
    bool operator!=(const iterator& rv) const {
      return (!(*this == rv));
    }
  };

  /**
   * @brief 返回环形数据缓冲区内保存的数据的起始位置的迭代器
   * @return 环形数据缓冲区内保存的数据的起始位置的迭代器
   */
  iterator begin() {
    return iterator(this, read_position_, data_count_>= DataNum);
  }
  /**
   * @brief 返回环形数据缓冲区内保存的数据的结束位置的迭代器
   * @return 环形数据缓冲区内保存的数据的结束位置的迭代器
   */
  iterator end() {
    return iterator(this, write_position_, data_count_>= DataNum);
  }

  /**
   * @class iterator
   * @brief 环形数据缓冲区的迭代器(常量型)
   */
  class const_iterator;
  friend class const_iterator;
  class const_iterator {
   private:
    // 指向环形数据缓冲区的指针
    const RingBuffer* ring_buff_;
    // 指向环形数据缓冲区的索引
    Int32_t ring_buff_index_;
    // 环形缓冲区是否已满
    bool is_ring_buff_full_;

   public:
    /**
     * @brief 构造函数
     * @param[in] ring_buff 指向环形数据缓冲区的指针
     * @param[in] index 环形数据缓冲区某个位置的索引（通常是保存的数据起始位置，或结束位置）
     * @param[in] is_full 当前环形数据缓冲区是否是满的
     */
    const_iterator(const RingBuffer* ring_buff, Int32_t index, bool is_full)
        : ring_buff_(ring_buff)
        , ring_buff_index_(index)
        , is_ring_buff_full_(is_full) {
      // nothing to do
    }
    /**
     * @brief 拷贝构造函数
     * @param[in] rv 另一个迭代器
     */
    const_iterator(const const_iterator& rv)
        : ring_buff_(rv.ring_buff_)
        , ring_buff_index_(rv.ring_buff_index_)
        , is_ring_buff_full_(rv.is_ring_buff_full_) {
      // nothing to do
    }
    /**
     * @brief 拷贝函数
     * @param[in] rv 另一个迭代器
     */
    const_iterator& operator=(const const_iterator& rv) {
      ring_buff_ = rv.ring_buff_;
      ring_buff_index_ = rv.ring_buff_index_;
      is_ring_buff_full_ = rv.is_ring_buff_full_;
      return *this;
    }
    /**
     * @brief 迭代器自增
     * @return 自增后的迭代器
     */
    const_iterator& operator++() {
      ++ring_buff_index_;
      is_ring_buff_full_ = false;
      if (ring_buff_index_ >= DataNum) {
        ring_buff_index_ -= DataNum;
      }
      return *this;
    }
    /**
     * @brief 正方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    const_iterator operator +(Int32_t offset) const {
      const_iterator it = *this;

      offset %= DataNum;
      if (0 == offset || ring_buff_->Empty()) {
        return (it);
      }

      if (ring_buff_->write_position_ == ring_buff_->read_position_) {
        if (ring_buff_index_ == ring_buff_->write_position_) {
          if (offset > 0 && !is_ring_buff_full_) {
            return (it);
          }
          if (offset < 0 && is_ring_buff_full_) {
            return (it);
          }
        }
      }

      Int32_t index = ring_buff_->BoundIndex(ring_buff_index_, offset);
      if (offset < 0 && index == ring_buff_->read_position_) {
        it.is_ring_buff_full_ = ring_buff_->Full();
      } else {
        it.is_ring_buff_full_ = true;
      }

      it.ring_buff_index_ = index;

      return (it);
    }
    /**
     * @brief 负方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    const_iterator operator -(Int32_t offset) const {
      return this->operator +(-offset);
    }

    /**
     * @brief 正方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    const_iterator& operator +=(Int32_t offset) {
      offset %= DataNum;
      if (0 == offset || ring_buff_->Empty()) {
        return (*this);
      }

      if (ring_buff_->write_position_ == ring_buff_->read_position_) {
        if (ring_buff_index_ == ring_buff_->write_position_) {
          if (offset > 0 && !is_ring_buff_full_) {
            return (*this);
          }
          if (offset < 0 && is_ring_buff_full_) {
            return (*this);
          }
        }
      }

      Int32_t index = ring_buff_->BoundIndex(ring_buff_index_, offset);
      if (offset < 0 && index == ring_buff_->read_position_) {
        is_ring_buff_full_ = ring_buff_->Full();
      } else {
        is_ring_buff_full_ = true;
      }

      ring_buff_index_ = index;

      return (*this);
    }
    /**
     * @brief 负方向偏移迭代器
     * @param[in] offset 偏移量
     * @return 偏移后的迭代器
     */
    const_iterator& operator -=(Int32_t offset) {
      return this->operator +=(-offset);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    const DataType* current() const {
      return &(ring_buff_->data_buff_[ring_buff_index_]);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    const DataType& operator*() const {
      return *current();
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    const DataType* operator->() const {
      return current();
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器相等
     * @param[in]  rv 另一个迭代器
     * @return true-相等, false-不相等
     */
    bool operator==(const const_iterator& rv) const {
      return ((ring_buff_index_ == rv.ring_buff_index_) &&
              (false == is_ring_buff_full_));
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器不相等
     * @param[in]  rv 另一个迭代器
     * @return true-不相等, false-相等
     */
    bool operator!=(const const_iterator& rv) const {
      return (!(*this == rv));
    }
  };

  /**
   * @brief 返回环形数据缓冲区内保存的数据的起始位置的迭代器(常量型)
   * @return 环形数据缓冲区内保存的数据的起始位置的迭代器
   */
  const_iterator cbegin() const {
    return const_iterator(this, read_position_, data_count_>= DataNum);
  }
  /**
   * @brief 返回环形数据缓冲区内保存的数据的结束位置的迭代器(常量型)
   * @return 环形数据缓冲区内保存的数据的结束位置的迭代器
   */
  const_iterator cend() const {
    return const_iterator(this, write_position_, data_count_>= DataNum);
  }

private:
  /*
   * @brief Limit the ring buffer index
   * @param[in] start_real_index start real index
   * @param[in] offset offset to the start real index
   * @return real ring buffer index
   */
  Int32_t BoundIndex(Int32_t start_real_index, Int32_t offset) const;


 private:
  // 写入位置的索引
  Int32_t write_position_;
  // 读取位置的索引
  Int32_t read_position_;
  // 已保存的数据的数量
  Int32_t data_count_;
  // 用来保存数据
  DataType data_buff_[DataNum];
};

template<typename DataType, Int32_t DataNum>
RingBuffer<DataType, DataNum>::RingBuffer() {
  write_position_ = 0;
  read_position_ = 0;
  data_count_ = 0;
}

template<typename DataType, Int32_t DataNum>
RingBuffer<DataType, DataNum>::~RingBuffer() {
  // nothing to do
}

template<typename DataType, Int32_t DataNum>
void RingBuffer<DataType, DataNum>::Clear() {
  write_position_ = 0;
  read_position_ = 0;
  data_count_ = 0;
}

template<typename DataType, Int32_t DataNum>
bool RingBuffer<DataType, DataNum>::PushBack(const DataType &data) {
  bool ret = true;
  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    data_buff_[write_position_] = data;
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    ret = false;
  } else {
    data_buff_[write_position_] = data;
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  }

  return (ret);
}

template<typename DataType, Int32_t DataNum>
DataType* RingBuffer<DataType, DataNum>::Allocate() {
  DataType* data = Nullptr_t;

  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    data = &data_buff_[write_position_];
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    data = Nullptr_t;
  } else {
    data = &data_buff_[write_position_];
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
bool RingBuffer<DataType, DataNum>::PushBackOverride(const DataType &data) {
  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    data_buff_[write_position_] = data;
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    data_buff_[write_position_] = data;
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    ++read_position_;
    if (read_position_ >= DataNum) {
      read_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  } else {
    data_buff_[write_position_] = data;
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  }

  return true;
}

template<typename DataType, Int32_t DataNum>
DataType* RingBuffer<DataType, DataNum>::AllocateOverride() {
  DataType* data = Nullptr_t;

  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    data = &data_buff_[write_position_];
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    data = &data_buff_[write_position_];
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    ++read_position_;
    if (read_position_ >= DataNum) {
      read_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  } else {
    data = &data_buff_[write_position_];
    ++write_position_;
    if (write_position_ >= DataNum) {
      write_position_ -= DataNum;
    }
    data_count_++;
    if (data_count_ > DataNum) {
      data_count_ = DataNum;
    }
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
const DataType* RingBuffer<DataType, DataNum>::PopFront() {
  DataType* data = Nullptr_t;
  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    data = &(data_buff_[read_position_]);
    ++read_position_;
    if (read_position_ >= DataNum) {
      read_position_ -= DataNum;
    }
    data_count_--;
    if (data_count_ < 0) {
      data_count_ = 0;
    }
  } else {
    data = &(data_buff_[read_position_]);
    ++read_position_;
    if (read_position_ >= DataNum) {
      read_position_ -= DataNum;
    }
    data_count_--;
    if (data_count_ < 0) {
      data_count_ = 0;
    }
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
void RingBuffer<DataType, DataNum>::PopFromFront(Int32_t num) {
  if (num < 1 || Empty()) {
    return;
  }
  if (num >= data_count_) {
    read_position_ = write_position_;
    data_count_ = 0;
    return;
  }

  Int32_t index = BoundIndex(read_position_, num);
  if (index == write_position_) {
    read_position_ = write_position_;
    data_count_ = 0;
  } else {
    read_position_ = index;
    data_count_ -= num;
  }
}

template<typename DataType, Int32_t DataNum>
const DataType* RingBuffer<DataType, DataNum>::PopBack() {
  DataType* data = Nullptr_t;
  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    Int32_t read_pose = write_position_ - 1;
    if (read_pose < 0) {
      read_pose += DataNum;
    }
    data = &(data_buff_[read_pose]);
    write_position_ = read_pose;

    data_count_--;
    if (data_count_ < 0) {
      data_count_ = 0;
    }
  } else {
    Int32_t read_pose = write_position_ - 1;
    if (read_pose < 0) {
      read_pose += DataNum;
    }
    data = &(data_buff_[read_pose]);
    write_position_ = read_pose;

    data_count_--;
    if (data_count_ < 0) {
      data_count_ = 0;
    }
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
void RingBuffer<DataType, DataNum>::PopFromBack(Int32_t num) {
  if (num < 1 || Empty()) {
    return;
  }
  if (num >= data_count_) {
    write_position_ = read_position_;
    data_count_ = 0;
    return;
  }

  Int32_t index = BoundIndex(write_position_, -num);
  if (index == read_position_) {
    write_position_ = read_position_;
    data_count_ = 0;
  } else {
    write_position_ = index;
    data_count_ -= num;
  }
}

template<typename DataType, Int32_t DataNum>
const DataType* RingBuffer<DataType, DataNum>::Front() const {
  const DataType* data = Nullptr_t;

  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    data = &(data_buff_[read_position_]);
  } else {
    data = &(data_buff_[read_position_]);
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
DataType* RingBuffer<DataType, DataNum>::Front() {
  DataType* data = Nullptr_t;

  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    data = &(data_buff_[read_position_]);
  } else {
    data = &(data_buff_[read_position_]);
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
const DataType* RingBuffer<DataType, DataNum>::Back() const {
  const DataType* data = Nullptr_t;

  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    Int32_t read_pose = write_position_ - 1;
    if (read_pose < 0) {
      read_pose += DataNum;
    }
    data = &(data_buff_[read_pose]);
  } else {
    Int32_t read_pose = write_position_ - 1;
    if (read_pose < 0) {
      read_pose = DataNum -1;
    }
    data = &(data_buff_[read_pose]);
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
DataType* RingBuffer<DataType, DataNum>::Back() {
  DataType* data = Nullptr_t;

  if ((write_position_ == read_position_) && (data_count_ <= 0)) {
    // buffer is empty
    return (data);
  } else if ((write_position_ == read_position_) &&
             (data_count_ >= DataNum)) {
    // buffer is full
    Int32_t read_pose = write_position_ - 1;
    if (read_pose < 0) {
      read_pose += DataNum;
    }
    data = &(data_buff_[read_pose]);
  } else {
    Int32_t read_pose = write_position_ - 1;
    if (read_pose < 0) {
      read_pose = DataNum -1;
    }
    data = &(data_buff_[read_pose]);
  }

  return (data);
}

template<typename DataType, Int32_t DataNum>
Int32_t RingBuffer<DataType, DataNum>::BoundIndex(
    Int32_t start_real_index, Int32_t offset) const {
  Int32_t real_index = start_real_index;

  offset %= DataNum;
  if (0 == offset || Empty()) {
    return (real_index);
  }

  if (offset > 0) {
    if (write_position_ > read_position_) {
      if ((start_real_index+offset) >= write_position_) {
        real_index = write_position_;
      } else {
        real_index += offset;
      }
    } else {
      if (start_real_index >= read_position_) {
        if ((start_real_index+offset) >= DataNum) {
          real_index = start_real_index + offset - DataNum;
          if (real_index >= write_position_) {
            real_index = write_position_;
          }
        } else {
          real_index = start_real_index + offset;
        }
      } else {
        real_index = start_real_index + offset;
        if (real_index >= write_position_) {
          real_index = write_position_;
        }
      }
    }
  } else {
    if (write_position_ > read_position_) {
      if ((start_real_index+offset) <= read_position_) {
        real_index = read_position_;
      } else {
        real_index += offset;
      }
    } else {
      if (start_real_index > read_position_) {
        real_index = start_real_index + offset;
        if (real_index <= read_position_) {
          real_index = read_position_;
        }
      } else if (start_real_index == read_position_) {
        if (read_position_ == write_position_) {
          if ((start_real_index+offset) < 0) {
            real_index = start_real_index + offset + DataNum;
            if (real_index <= read_position_) {
              real_index = read_position_;
            }
          } else {
            real_index = start_real_index + offset;
          }
        } else {
          real_index = read_position_;
        }
      } else {
        if ((start_real_index+offset) < 0) {
          real_index = start_real_index + offset + DataNum;
          if (real_index <= read_position_) {
            real_index = read_position_;
          }
        } else {
          real_index = start_real_index + offset;
        }
      }
    }
  }

  return (real_index);
}


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_RING_BUFFER_H_
