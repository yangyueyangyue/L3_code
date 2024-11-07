/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       data_list.h
 * @brief      双向链表
 * @details    双向链表(主要用于多线程以及多进程通讯)
 *
 * @author     pengc
 * @date       2018.11.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2018/11/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_DATA_LIST_H_
#define PHOENIX_COMMON_DATA_LIST_H_

#include <string>
#include <iostream>
#include <sstream>
#include "utils/log.h"
#include "container/data_pool.h"


#ifndef macro_offsetof
#define macro_offsetof(TYPE, MEMBER) (reinterpret_cast<size_t>(   \
  &((reinterpret_cast<TYPE *>(0))->MEMBER)))
#endif

#define macro_container_of(ptr, type, member) (     \
  reinterpret_cast<type *>(reinterpret_cast<char *>(ptr)  \
  - macro_offsetof(type, member)))


namespace phoenix {
namespace common {

/*
* DataList, using DataPool
*/
template <typename DataType, int Count>
class DataList {
 private:
  typedef DataType data_type;
  typedef struct NodeHead {
    Int32_t head;
    Int32_t tail;
  }node_head;
  typedef struct NodeType {
    Int32_t next;
    Int32_t prev;
    DataType data;
  }node_type;
  typedef DataPool<node_type, Count> pool_type;

 public:
  explicit DataList(bool override_expired_data = false);
  bool Clear(void);

  data_type* AllocSpace();
  bool PushBack(data_type* new_data);
  data_type* GetFront();
  bool FreeSpace(data_type* del_data);

  class iterator;
  friend class iterator;
  class iterator {
   public:
    typedef DataList<DataType, Count> list_type;

   private:
    list_type& list_;
    Int32_t index_;

   public:
    explicit iterator(list_type& list)
        : list_(list), index_(list_.list_.head) {}
    iterator(list_type& list, bool)
        : list_(list), index_(-1) {}
    iterator(const iterator& rv) : list_(rv.list_), index_(rv.index_) {}
    iterator& operator=(const iterator& rv) {
      list_ = rv.list_;
      index_ = rv.index_;
      return (*this);
    }
    iterator& operator++() {
      if (!list_.pool_.IsNull(index_)) {
        typename list_type::node_type* pddr =
            list_.pool_.GetVirtualAddr(index_);
        index_ = pddr->next;
      }
      return (*this);
    }
    bool operator==(const iterator& rv) const { return (index_ == rv.index_); }
    bool operator!=(const iterator& rv) const { return (index_ != rv.index_); }
    typename list_type::data_type* current() {
      if (!list_.pool_.IsNull(index_)) {
        typename list_type::node_type* pddr =
            list_.pool_.GetVirtualAddr(index_);
        return (&(pddr->data));
      }
      return (Nullptr_t);
    }
    typename list_type::data_type& operator*() { return *current(); }
    typename list_type::data_type* operator->() { return current(); }
    typename list_type::data_type* remove() {
      if (!list_.pool_.IsNull(index_)) {
        typename list_type::node_type* pddr =
            list_.pool_.GetVirtualAddr(index_);
        list_.Erease(pddr);
        return (&(pddr->data));
      }
      return (Nullptr_t);
    }
  };
  iterator begin() { return iterator(*this); }
  iterator end() { return iterator(*this, true); }

  std::string DebugString(void) const;

 private:
  node_type* AllocNode(void);
  bool IsFreeNode(node_type* node) const {
    return ((list_.head != pool_.GetOffset(node)) &&
        (pool_.IsNull(node->next)) && (pool_.IsNull(node->prev)));
  }
  bool Insert(node_type* front, node_type* new_node);
  bool Erease(node_type* del_node);

 private:
  bool override_expired_data_flag_;  // override expired data ?
  node_head list_;
  pool_type pool_;
};

template <typename DataType, int Count>
DataList<DataType, Count>::DataList(bool override_expired_data) {
  override_expired_data_flag_ = override_expired_data;
  list_.head = -1;
  list_.tail = -1;
}

template <typename DataType, int Count>
bool DataList<DataType, Count>::Clear(void) {
  list_.head = -1;
  list_.tail = -1;
  pool_.Clear();
  return (true);
}

template <typename DataType, int Count>
typename DataList<DataType, Count>::data_type*
DataList<DataType, Count>::AllocSpace() {
  node_type* node = AllocNode();
  if (Nullptr_t != node) {
    return (&(node->data));
  }
  return (Nullptr_t);
}

template <typename DataType, int Count>
bool DataList<DataType, Count>::PushBack(data_type* new_data) {
  if (Nullptr_t == new_data) {
    return (false);
  }

  node_type* node = macro_container_of(new_data, node_type, data);

  COM_CHECK(IsFreeNode(node));
  COM_CHECK(!pool_.IsNull(pool_.GetOffset(node)));

  return (Insert(pool_.GetVirtualAddr(list_.tail), node));
}

template <typename DataType, int Count>
typename DataList<DataType, Count>::data_type*
DataList<DataType, Count>::GetFront() {
  if (!pool_.IsNull(list_.head)) {
    node_type* node = pool_.GetVirtualAddr(list_.head);
    Erease(node);
    node->next = -1;
    node->prev = -1;
    return (&(node->data));
  }

  return (Nullptr_t);
}

template <typename DataType, int Count>
bool DataList<DataType, Count>::FreeSpace(data_type* del_data) {
  if (Nullptr_t == del_data) {
    return (false);
  }

  node_type* node = macro_container_of(del_data, node_type, data);

  COM_CHECK(IsFreeNode(node));

  return (pool_.Deallocate(pool_.GetOffset(node)));
}

template <typename DataType, int Count>
typename DataList<DataType, Count>::node_type*
DataList<DataType, Count>::AllocNode(void) {
  node_type* node = Nullptr_t;
  Int32_t new_offset = pool_.Allocate();
  if (pool_.IsNull(new_offset)) {
    if (override_expired_data_flag_) {
      // get node from expired list
      if (!pool_.IsNull(list_.head)) {
        node = pool_.GetVirtualAddr(list_.head);
        Erease(node);
        node->next = -1;
        node->prev = -1;
        return (node);
      }
    }
    return (Nullptr_t);
  }

  node = pool_.GetVirtualAddr(new_offset);
  node->next = -1;
  node->prev = -1;

  return (node);
}

template <typename DataType, int Count>
bool DataList<DataType, Count>::Insert(node_type* front, node_type* new_node) {
  Int32_t new_offset = pool_.GetOffset(new_node);
  if (new_offset < 0) {
    return (false);
  }
  if (pool_.IsNull(list_.head)) {
    list_.head = new_offset;
    list_.tail = new_offset;
    new_node->prev = -1;
    new_node->next = -1;
  } else if (Nullptr_t != front) {
    Int32_t next_offset = front->next;
    if (!pool_.IsNull(next_offset)) {
      node_type* next_addr = pool_.GetVirtualAddr(next_offset);
      next_addr->prev = new_offset;
    }
    // newOne->next = next;
    new_node->next = next_offset;
    // newOne->prev = prev;
    Int32_t prev_offset = pool_.GetOffset(front);
    new_node->prev = prev_offset;
    // prev->next = newOne;
    front->next = new_offset;
    if (list_.tail == prev_offset) {
      list_.tail = new_offset;
    }
  } else {
    // next->prev = newOne;
    Int32_t next_offset = list_.head;
    if (!pool_.IsNull(next_offset)) {
      node_type* next_addr = pool_.GetVirtualAddr(next_offset);
      next_addr->prev = new_offset;
    }
    // newOne->next = next;
    new_node->next = next_offset;
    // newOne->prev = prev;
    Int32_t prev_offset = -1;
    new_node->prev = prev_offset;
    // prev->next = newOne;
    list_.head = new_offset;
  }
  return (true);
}

template <typename DataType, int Count>
bool DataList<DataType, Count>::Erease(node_type* del_node) {
  Int32_t next_offset = del_node->next;
  Int32_t prev_offset = del_node->prev;
  if (!pool_.IsNull(next_offset)) {
    node_type* next_addr = pool_.GetVirtualAddr(next_offset);
    next_addr->prev = prev_offset;
  } else {
    list_.tail = prev_offset;
  }
  if (!pool_.IsNull(prev_offset)) {
    node_type* prev_addr = pool_.GetVirtualAddr(prev_offset);
    prev_addr->next = next_offset;
  } else {
    list_.head = next_offset;
  }
  del_node->next = -1;
  del_node->prev = -1;

  return (true);
}

template <typename DataType, int Count>
std::string DataList<DataType, Count>::DebugString(void) const {
  std::ostringstream os;

  os << "-------------------- Data List -------------------------->"
     << std::endl;

  os << "**Data pool:\n";
  os << pool_.DebugString();

  os << "**list_.head=" << list_.head << ", list_.tail=" << list_.tail << "\n";

  os << "**node list (head->tail) :\n";
  Int32_t offset = list_.head;
  node_type* pddr = pool_.GetVirtualAddr(offset);
  int i = 0;
  while (!pool_.IsNull(offset)) {
    os << "**[" << i++
       << "] node_addr("<< pddr
       << "),current(" << offset
       << "),next("<< pddr->next
       << "),prev("<< pddr->prev
       << ")\n";
    offset = pddr->next;
    pddr = pool_.GetVirtualAddr(offset);
  }

  os << "**node list (tail->head) :\n";
  offset = list_.tail;
  pddr = pool_.GetVirtualAddr(offset);
  i = 0;
  while (!pool_.IsNull(offset)) {
    os << "**[" << i++
       << "] node_addr("<< pddr
       << "),current(" << offset
       << "),next("<< pddr->next
       << "),prev("<< pddr->prev
       << ")\n";
    offset = pddr->prev;
    pddr = pool_.GetVirtualAddr(offset);
  }

  os << "<------------------- Data List --------------------------"
    << std::endl;

  return (os.str());
}


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_DATA_LIST_H_
