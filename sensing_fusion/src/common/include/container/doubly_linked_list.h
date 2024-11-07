/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       doubly_linked_list.h
 * @brief      双向链表
 * @details    双向链表
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

#ifndef PHOENIX_COMMON_DOUBLY_LINKED_LIST_H_
#define PHOENIX_COMMON_DOUBLY_LINKED_LIST_H_

#include <string>
#include <iostream>
#include <sstream>
#include "utils/log.h"
#include "container/data_pool.h"


#ifndef internal_macro_offsetof
#define internal_macro_offsetof(TYPE, MEMBER) (reinterpret_cast<size_t>(   \
  &((reinterpret_cast<TYPE *>(0))->MEMBER)))
#endif

#define internal_macro_container_of(ptr, type, member) (     \
  reinterpret_cast<type *>(reinterpret_cast<char *>(ptr)  \
  - internal_macro_offsetof(type, member)))


namespace phoenix {
namespace common {

/**
 * @class DoublyLinkedListNode
 * @brief 双向链表的节点
 * @param DataType 数据类型
 */
template <typename DataType>
struct DoublyLinkedListNode {
  /// 指向下一个节点的索引
  Int32_t next;
  /// 指向上一个节点的索引
  Int32_t prev;
  /// 保存用户数据
  DataType data;
};

/**
 * @class DoublyLinkedList
 * @brief 双向链表（在外部输入的Memory Pool中构建双向链表）
 * @param DataType 数据类型
 * @param MaxDataNum Memory Pool中链表元素的最大个数
 */
template <typename DataType, Int32_t MaxDataNum>
class DoublyLinkedList {
public:
  typedef DataType data_type;
  typedef DoublyLinkedListNode<DataType> node_type;
  typedef DataPool<node_type, MaxDataNum> pool_type;

public:
  /**
   * @brief 构造函数
   */
  DoublyLinkedList() {
    head_ = -1;
    tail_ = -1;
  }

  /**
   * @brief 清除内部数据
   * @param[in] pool 链表元素存储区域的引用
   */
  void Clear(pool_type& pool);

  /**
   * @brief 判断链表是否是空的
   * @return true - 空的，false - 非空
   */
  bool Empty() const {
    return (head_ < 0);
  }

  Int32_t Size(const pool_type& pool) const {
    Int32_t count = 0;
    Int32_t offset = head_;
    const node_type* addr = pool.GetVirtualAddr(offset);
    while (!pool.IsNull(offset)) {
      count++;
      offset = addr->next;
      addr = pool.GetVirtualAddr(offset);
    }

    return (count);
  }

  /**
   * @brief 获取链表头部的数据
   * @param[in] pool 链表元素存储区域的引用
   * @return 链表头部的数据
   */
  DataType* Front(pool_type& pool) {
    if (!pool.IsNull(head_)) {
      node_type* addr = pool.GetVirtualAddr(head_);
      return (&(addr->data));
    }

    return (Nullptr_t);
  }

  /**
   * @brief 获取链表尾部的数据
   * @param[in] pool 链表元素存储区域的引用
   * @return 链表尾部的数据
   */
  DataType* Back(pool_type& pool) {
    if (!pool.IsNull(tail_)) {
      node_type* addr = pool.GetVirtualAddr(tail_);
      return (&(addr->data));
    }

    return (Nullptr_t);
  }

  /**
   * @brief 获取链表头部的数据
   * @param[in] pool 链表元素存储区域的引用
   * @return 链表头部的数据
   */
  const DataType* Front(const pool_type& pool) const {
    if (!pool.IsNull(head_)) {
      const node_type* addr = pool.GetVirtualAddr(head_);
      return (&(addr->data));
    }

    return (Nullptr_t);
  }

  /**
   * @brief 获取链表尾部的数据
   * @param[in] pool 链表元素存储区域的引用
   * @return 链表尾部的数据
   */
  const DataType* Back(const pool_type& pool) const {
    if (!pool.IsNull(tail_)) {
      const node_type* addr = pool.GetVirtualAddr(tail_);
      return (&(addr->data));
    }

    return (Nullptr_t);
  }

  /**
   * @brief 添加一个数据到链表尾部
   * @param[in] pool 链表元素存储区域的引用
   * @return 新数据的地址
   */
  data_type* AddToBack(pool_type& pool);

  /**
   * @brief 添加一个数据到链表头部
   * @param[in] pool 链表元素存储区域的引用
   * @return 新数据的地址
   */
  data_type* AddToFront(pool_type& pool);

  /**
   * @brief 添加一个数据到链表尾部
   * @param[in] new_node 新数据的索引
   * @param[in] pool 链表元素存储区域的引用
   * @return true-成功；false-失败
   */
  bool PushBack(const data_type& new_data, pool_type& pool);

  /**
   * @brief 添加一个数据到链表头部
   * @param[in] new_node 新数据的索引
   * @param[in] pool 链表元素存储区域的引用
   * @return true-成功；false-失败
   */
  bool PushFront(const data_type& new_data, pool_type& pool);

  /**
   * @brief 从链表尾部移除一个数据
   * @param[in] pool 链表元素存储区域的引用
   */
  void PopBack(pool_type& pool);

  /**
   * @brief 从链表头部移除一个数据
   * @param[in] pool 链表元素存储区域的引用
   */
  void PopFront(pool_type& pool);

  /**
   * @brief 从链表中移除一个数据
   * @param[in] del_data 需要移除的数据的地址
   * @param[in] pool 链表元素存储区域的引用
   */
  void Erase(data_type* del_data, pool_type& pool);

  /**
   * @brief 将链表中的数据排序
   * @param[in] pool 链表元素存储区域的引用
   * @param[in] cmp_func 比较函数
   * @par Note:
   * @code
   *     Using Selection-Sort algorithm
   *     Time complexity, Best: T(n) = O(n2),
   *     Worst: T(n) = O(n2), Average: T(n) = O(n2)
   *     Space complexity: S(n) = O(n2)
   * @endcode
   */
  template <typename CmpFunc>
  void Sort(pool_type& pool, const CmpFunc& cmp_func);

  /**
   * @class iterator
   * @brief 双向链表的迭代器
   */
  class iterator;
  friend class iterator;
  class iterator {
   public:
    typedef DoublyLinkedList list_type;
    typedef DoublyLinkedListNode<DataType> node_type;
    typedef DoublyLinkedList::pool_type pool_type;

   private:
    // 指向链表的指针
    list_type* list_;
    // 链表中元素的索引
    Int32_t index_;
    // 指向元素存储区域的指针
    pool_type* pool_;

   public:
    /**
     * @brief 构造函数
     * @param[in] lst 链表的引用
     * @param[in] node_pool 链表元素存储区域的引用
     */
    iterator(list_type& lst, pool_type& node_pool)
        : list_(&lst), index_(lst.head_), pool_(&node_pool) {}
    /**
     * @brief 构造函数
     * @param[in] list 链表的引用
     * @param[in] storage 链表元素存储区域的引用
     * @param[in] bool 用来构建链表尾部的索引
     */
    iterator(list_type& lst, pool_type& node_pool, Int32_t idx)
        : list_(&lst), index_(idx), pool_(&node_pool) {}

    /**
     * @brief 拷贝构造函数
     * @param[in] rv 另一个迭代器
     */
    iterator(const iterator& rv) :
      list_(rv.list_), index_(rv.index_), pool_(rv.pool_) {}
    /**
     * @brief 拷贝函数
     * @param[in] rv 另一个迭代器
     */
    inline iterator& operator=(const iterator& rv) {
      list_ = rv.list_;
      index_ = rv.index_;
      pool_ = rv.pool_;
      return (*this);
    }

    /**
     * @brief 迭代器自增
     * @return 自增后的迭代器
     */
    inline iterator& operator++() {
      if (!pool_->IsNull(index_)) {
        node_type* addr = pool_->GetVirtualAddr(index_);
        index_ = addr->next;
      }
      return (*this);
    }

    /**
     * @brief 判断当前迭代器是否与另一个迭代器相等
     * @param[in]  rv 另一个迭代器
     * @return true-相等, false-不相等
     */
    inline bool operator==(const iterator& rv) const {
      return (index_ == rv.index_);
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器不相等
     * @param[in]  rv 另一个迭代器
     * @return true-不相等, false-相等
     */
    inline bool operator!=(const iterator& rv) const {
      return (index_ != rv.index_);
    }

    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    inline DataType* current() {
      if (!pool_->IsNull(index_)) {
        node_type* addr = pool_->GetVirtualAddr(index_);
        return (&(addr->data));
      }
      return (Nullptr_t);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    inline DataType& operator*() { return *current(); }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    inline DataType* operator->() { return current(); }

    /**
     * @brief 从链表中移除当前迭代器所指向的数据
     * @return 更新后的迭代器（指向被移除的数据的下一个数据）
     */
    inline iterator& erase() {
      if (!pool_->IsNull(index_)) {
        node_type* addr = pool_->GetVirtualAddr(index_);
        index_ = addr->next;
        list_->EraseListNode(addr, *pool_);
      }
      return (*this);
    }
  };
  /**
   * @brief 返回双向链表的起始位置的迭代器
   * @return 双向链表的起始位置的迭代器
   */
  inline iterator begin(pool_type& pool) {
    return iterator(*this, pool);
  }
  /**
   * @brief 返回双向链表的结束位置的迭代器
   * @return 双向链表的结束位置的迭代器
   */
  inline iterator end(pool_type& pool) {
    return iterator(*this, pool, -1);
  }


  /**
   * @class const_iterator
   * @brief 双向链表的迭代器(常量型)
   */
  class const_iterator;
  friend class const_iterator;
  class const_iterator {
   public:
    typedef DoublyLinkedList list_type;
    typedef DoublyLinkedListNode<DataType> node_type;
    typedef DoublyLinkedList::pool_type pool_type;

   private:
    // 指向链表的指针
    const list_type* list_;
    // 链表中元素的索引
    Int32_t index_;
    // 指向元素存储区域的指针
    const pool_type* pool_;

   public:
    /**
     * @brief 构造函数
     * @param[in] lst 链表的引用
     * @param[in] node_pool 链表元素存储区域的引用
     */
    const_iterator(const list_type& lst, const pool_type& node_pool)
        : list_(&lst), index_(lst.head_), pool_(&node_pool) {}
    /**
     * @brief 构造函数
     * @param[in] list 链表的引用
     * @param[in] storage 链表元素存储区域的引用
     * @param[in] bool 用来构建链表尾部的索引
     */
    const_iterator(
        const list_type& lst, const pool_type& node_pool, Int32_t idx)
        : list_(&lst), index_(idx), pool_(&node_pool) {}

    /**
     * @brief 拷贝构造函数
     * @param[in] rv 另一个迭代器
     */
    const_iterator(const const_iterator& rv) :
      list_(rv.list_), index_(rv.index_), pool_(rv.pool_) {}
    /**
     * @brief 拷贝函数
     * @param[in] rv 另一个迭代器
     */
    inline const_iterator& operator=(const const_iterator& rv) {
      list_ = rv.list_;
      index_ = rv.index_;
      pool_ = rv.pool_;
      return (*this);
    }

    /**
     * @brief 迭代器自增
     * @return 自增后的迭代器
     */
    inline const_iterator& operator++() {
      if (!pool_->IsNull(index_)) {
        const node_type* addr = pool_->GetVirtualAddr(index_);
        index_ = addr->next;
      }
      return (*this);
    }

    /**
     * @brief 判断当前迭代器是否与另一个迭代器相等
     * @param[in]  rv 另一个迭代器
     * @return true-相等, false-不相等
     */
    inline bool operator==(const const_iterator& rv) const {
      return (index_ == rv.index_);
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器不相等
     * @param[in]  rv 另一个迭代器
     * @return true-不相等, false-相等
     */
    inline bool operator!=(const const_iterator& rv) const {
      return (index_ != rv.index_);
    }

    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    inline const DataType* current() const {
      if (!pool_->IsNull(index_)) {
        const node_type* addr = pool_->GetVirtualAddr(index_);
        return (&(addr->data));
      }
      return (Nullptr_t);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    inline const DataType& operator*() const { return *current(); }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    inline const DataType* operator->() const { return current(); }
  };
  /**
   * @brief 返回双向链表的起始位置的迭代器
   * @return 双向链表的起始位置的迭代器
   */
  inline const_iterator cbegin(const pool_type& pool) const {
    return const_iterator(*this, pool);
  }
  /**
   * @brief 返回双向链表的结束位置的迭代器
   * @return 双向链表的结束位置的迭代器
   */
  inline const_iterator cend(const pool_type& pool) const {
    return const_iterator(*this, pool, -1);
  }

  /**
   * @brief 从链表中找到指定的元素
   * @param[in] pool 链表元素存储区域的引用
   * @param[in] cmp_func 比较函数
   * @return 指定元素的迭代器
   */
  template <typename CmpFunc>
  iterator Find(pool_type& pool, const CmpFunc& cmp_func) {
    Int32_t offset = head_;
    node_type* addr = pool.GetVirtualAddr(offset);
    while (!pool.IsNull(offset)) {
      if (cmp_func(addr->data)) {
        return iterator(*this, pool, offset);
      }
      offset = addr->next;
      addr = pool.GetVirtualAddr(offset);
    }

    return iterator(*this, pool, -1);
  }

  template <typename CmpFunc>
  const_iterator Find(pool_type& pool, const CmpFunc& cmp_func) const {
    Int32_t offset = head_;
    const node_type* addr = pool.GetVirtualAddr(offset);
    while (!pool.IsNull(offset)) {
      if (cmp_func(addr->data)) {
        return const_iterator(*this, pool, offset);
      }
      offset = addr->next;
      addr = pool.GetVirtualAddr(offset);
    }

    return const_iterator(*this, pool, -1);
  }

  /**
   * @brief 从链表中找到指定的元素
   * @param[in] pool 链表元素存储区域的引用
   * @param[in] cmp_func 比较函数
   * @return 指定元素
   */
  template <typename CmpFunc>
  data_type* FindData(pool_type& pool, const CmpFunc& cmp_func) {
    Int32_t offset = head_;
    node_type* addr = pool.GetVirtualAddr(offset);
    while (!pool.IsNull(offset)) {
      if (cmp_func(addr->data)) {
        return (&(addr->data));
      }
      offset = addr->next;
      addr = pool.GetVirtualAddr(offset);
    }

    return Nullptr_t;
  }

  template <typename CmpFunc>
  const data_type* FindData(pool_type& pool, const CmpFunc& cmp_func) const {
    Int32_t offset = head_;
    const node_type* addr = pool.GetVirtualAddr(offset);
    while (!pool.IsNull(offset)) {
      if (cmp_func(addr->data)) {
        return (&(addr->data));
      }
      offset = addr->next;
      addr = pool.GetVirtualAddr(offset);
    }

    return Nullptr_t;
  }

  std::string DebugString(const pool_type& pool) const;

private:
  /*
   * @brief 从内存池中分配一个链表节点的内存空间
   * @param[in] pool 链表元素存储区域的引用
   * @return 分配好的链表节点的地址
   */
  node_type* AllocateListNode(pool_type& pool) {
    node_type* node = Nullptr_t;
    Int32_t new_offset = pool.Allocate();
    if (pool.IsNull(new_offset)) {
      return (Nullptr_t);
    }

    node = pool.GetVirtualAddr(new_offset);
    node->next = -1;
    node->prev = -1;

    return (node);
  }

  /*
   * @brief 插入一个新的链表节点到链表中
   * @param[in] front 若front不为空，则在front之后插入新节点，否则在链表头部插入新节点
   * @param[in] new_node 待插入的新的链表节点
   * @param[in] pool 链表元素存储区域的引用
   * @return true-不相等, false-相等
   */
  bool Insert(node_type* front, node_type* new_node, pool_type& pool);

  /*
   * @brief 从链表中删除一个节点
   * @param[in] del_node 待删除的链表节点
   * @param[in] pool 链表元素存储区域的引用
   */
  void EraseListNode(node_type* del_node, pool_type& pool);

  /*
   * @brief 交换两个链表节点
   * @param[in] n1 节点1
   * @param[in] n2 节点2
   * @param[in] pool 链表元素存储区域的引用
   */
  void Swap(const Int32_t n1, const Int32_t n2, pool_type& pool);

private:
  // 头节点的索引
  Int32_t head_;
  // 尾节点的索引
  Int32_t tail_;
};


template <typename DataType, Int32_t MaxDataNum>
void DoublyLinkedList<DataType, MaxDataNum>::Clear(pool_type& pool) {
  Int32_t offset = head_;
  node_type* addr = pool.GetVirtualAddr(offset);

  while (!pool.IsNull(offset)) {
    Int32_t next_offset = addr->next;

    pool.Deallocate(offset);

    offset = next_offset;
    addr = pool.GetVirtualAddr(next_offset);
  }

  head_ = -1;
  tail_ = -1;
}

template <typename DataType, Int32_t MaxDataNum>
typename DoublyLinkedList<DataType, MaxDataNum>::data_type*
DoublyLinkedList<DataType, MaxDataNum>::AddToBack(pool_type& pool) {
  node_type* node = AllocateListNode(pool);
  if (Nullptr_t != node) {
    Insert(pool.GetVirtualAddr(tail_), node, pool);
    return (&(node->data));
  }
  return (Nullptr_t);
}

template <typename DataType, Int32_t MaxDataNum>
typename DoublyLinkedList<DataType, MaxDataNum>::data_type*
DoublyLinkedList<DataType, MaxDataNum>::AddToFront(pool_type& pool) {
  node_type* node = AllocateListNode(pool);
  if (Nullptr_t != node) {
    Insert(Nullptr_t, node, pool);
    return (&(node->data));
  }
  return (Nullptr_t);
}

template <typename DataType, Int32_t MaxDataNum>
bool DoublyLinkedList<DataType, MaxDataNum>::PushBack(
    const data_type& new_data, pool_type& pool) {
  node_type* node = AllocateListNode(pool);
  if (Nullptr_t != node) {
    Insert(pool.GetVirtualAddr(tail_), node, pool);
    node->data = new_data;
    return true;
  }
  return false;
}

template <typename DataType, Int32_t MaxDataNum>
bool DoublyLinkedList<DataType, MaxDataNum>::PushFront(
    const data_type& new_data, pool_type& pool) {
  node_type* node = AllocateListNode(pool);
  if (Nullptr_t != node) {
    Insert(Nullptr_t, node, pool);
    node->data = new_data;
    return true;
  }
  return false;
}

template <typename DataType, Int32_t MaxDataNum>
void DoublyLinkedList<DataType, MaxDataNum>::PopBack(pool_type& pool) {
  if (pool.IsNull(tail_)) {
    return;
  }
  node_type* node = pool.GetVirtualAddr(tail_);
  EraseListNode(node, pool);
}

template <typename DataType, Int32_t MaxDataNum>
void DoublyLinkedList<DataType, MaxDataNum>::PopFront(pool_type& pool) {
  if (pool.IsNull(head_)) {
    return;
  }
  node_type* node = pool.GetVirtualAddr(head_);
  EraseListNode(node, pool);
}

template <typename DataType, Int32_t MaxDataNum>
void DoublyLinkedList<DataType, MaxDataNum>::Erase(
    data_type* del_data, pool_type& pool) {
  if (Nullptr_t == del_data) {
    return;
  }

  node_type* node = internal_macro_container_of(del_data, node_type, data);
  Int32_t offset = pool.GetOffset(node);
  if (pool.IsNull(offset)) {
    return;
  }
  EraseListNode(node, pool);

  return;
}

template <typename DataType, Int32_t MaxDataNum>
template <typename CmpFunc>
void DoublyLinkedList<DataType, MaxDataNum>::Sort(
    pool_type& pool, const CmpFunc& cmp_func) {
  if (Empty()) {
    return;
  }

  Int32_t first_node = head_;
  Int32_t next_node = -1;
  Int32_t tmp = 0;

  do {
    next_node = pool.GetVirtualAddr(first_node)->next;

    for (; !pool.IsNull(next_node);
         next_node = pool.GetVirtualAddr(next_node)->next) {
      if (cmp_func(pool.GetVirtualAddr(next_node)->data,
                   pool.GetVirtualAddr(first_node)->data)) {
        Swap(first_node, next_node, pool);
        tmp = first_node;
        first_node = next_node;
        next_node= tmp;
      }
    }

    first_node = pool.GetVirtualAddr(first_node)->next;

  } while (!pool.IsNull(first_node));
}

template <typename DataType, Int32_t MaxDataNum>
bool DoublyLinkedList<DataType, MaxDataNum>::Insert(
    node_type* front, node_type* new_node, pool_type& pool) {
  Int32_t new_offset = pool.GetOffset(new_node);
  if (new_offset < 0) {
    return (false);
  }
  if (pool.IsNull(head_)) {
    head_ = new_offset;
    tail_ = new_offset;
    new_node->prev = -1;
    new_node->next = -1;
  } else if (Nullptr_t != front) {
    Int32_t next_offset = front->next;
    if (!pool.IsNull(next_offset)) {
      node_type* next_addr = pool.GetVirtualAddr(next_offset);
      next_addr->prev = new_offset;
    }
    // newOne->next = next;
    new_node->next = next_offset;
    // newOne->prev = prev;
    Int32_t prev_offset = pool.GetOffset(front);
    new_node->prev = prev_offset;
    // prev->next = newOne;
    front->next = new_offset;
    if (tail_ == prev_offset) {
      tail_ = new_offset;
    }
  } else {
    // next->prev = newOne;
    Int32_t next_offset = head_;
    if (!pool.IsNull(next_offset)) {
      node_type* next_addr = pool.GetVirtualAddr(next_offset);
      next_addr->prev = new_offset;
    }
    // newOne->next = next;
    new_node->next = next_offset;
    // newOne->prev = prev;
    Int32_t prev_offset = -1;
    new_node->prev = prev_offset;
    // prev->next = newOne;
    head_ = new_offset;
  }

  return (true);
}

template <typename DataType, Int32_t MaxDataNum>
void DoublyLinkedList<DataType, MaxDataNum>::EraseListNode(
    node_type* del_node, pool_type& pool) {
  Int32_t next_offset = del_node->next;
  Int32_t prev_offset = del_node->prev;

  if (!pool.IsNull(next_offset)) {
    node_type* next_addr = pool.GetVirtualAddr(next_offset);
    next_addr->prev = prev_offset;
  } else {
    tail_ = prev_offset;
  }
  if (!pool.IsNull(prev_offset)) {
    node_type* prev_addr = pool.GetVirtualAddr(prev_offset);
    prev_addr->next = next_offset;
  } else {
    head_ = next_offset;
  }
  del_node->next = -1;
  del_node->prev = -1;

  pool.Deallocate(pool.GetOffset(del_node));
}

template <typename DataType, Int32_t MaxDataNum>
void DoublyLinkedList<DataType, MaxDataNum>::Swap(
    const Int32_t n1, const Int32_t n2, pool_type& pool) {
  COM_CHECK(n1 != n2);
  COM_CHECK(!pool.IsNull(n1) && !pool.IsNull(n1));

  node_type* node_n1 = pool.GetVirtualAddr(n1);
  node_type* node_n2 = pool.GetVirtualAddr(n2);
  COM_CHECK((Nullptr_t != node_n1) && (Nullptr_t != node_n2));

  Int32_t n1_prev = node_n1->prev;
  Int32_t n1_next = node_n1->next;
  Int32_t n2_prev = node_n2->prev;
  Int32_t n2_next = node_n2->next;

  if (n1_prev == n2 || n2_next == n1) {
    // from "n2_prev -> n2 -> n1 -> n1_next"
    //   to "n2_prev -> n1 -> n2 -> n1_next"
    // invalid: node[n1_prev].next = n2; because n1_prev == n2
    node_n2->prev = n1;
    node_n2->next = n1_next;
    if (!pool.IsNull(n1_next)) pool.GetVirtualAddr(n1_next)->prev = n2;
    if (!pool.IsNull(n2_prev)) pool.GetVirtualAddr(n2_prev)->next = n1;
    node_n1->prev = n2_prev;
    node_n1->next = n2;
    // invalid: node[n2_next].prev = n1; because n2_next == n1
  } else if (n2_prev == n1 || n1_next == n2) {
    // from "n1_prev -> n1 -> n2 -> n2_next"
    //   to "n1_prev -> n2 -> n1 -> n2_next"
    if (!pool.IsNull(n1_prev)) pool.GetVirtualAddr(n1_prev)->next = n2;
    node_n2->prev = n1_prev;
    node_n2->next = n1;
    // invalid: node[n1_next].prev = n2; because n1_next == n2
    // invalid: node[n2_prev].next = n1; because n2_prev == n1
    node_n1->prev = n2;
    node_n1->next = n2_next;
    if (!pool.IsNull(n2_next)) pool.GetVirtualAddr(n2_next)->prev = n1;
  } else {
    // from "n1_prev -> n1 -> n1_next n2_prev -> n2 -> n2_next"
    //   to "n1_prev -> n2 -> n1_next n2_prev -> n1 -> n2_next"
    // or inverse
    if (!pool.IsNull(n1_prev)) pool.GetVirtualAddr(n1_prev)->next = n2;
    node_n2->prev = n1_prev;
    node_n2->next = n1_next;
    if (!pool.IsNull(n1_next)) pool.GetVirtualAddr(n1_next)->prev = n2;
    if (!pool.IsNull(n2_prev)) pool.GetVirtualAddr(n2_prev)->next = n1;
    node_n1->prev = n2_prev;
    node_n1->next = n2_next;
    if (!pool.IsNull(n2_next)) pool.GetVirtualAddr(n2_next)->prev = n1;
  }

  if (n1 == head_ && n2 == tail_) {
    head_ = n2;
    tail_ = n1;
  } else if (n1 == tail_ && n2 == head_) {
    head_ = n1;
    tail_ = n2;
  } else if (n1 == head_ && n2 != tail_) {
    head_ = n2;
  } else if (n1 != head_ && n2 == tail_) {
    tail_ = n1;
  } else if (n1 != tail_ && n2 == head_) {
    head_ = n1;
  } else if (n1 == tail_ && n2 != head_) {
    tail_ = n2;
  } else {
    // nothing to do
  }
}

template <typename DataType, Int32_t MaxDataNum>
std::string DoublyLinkedList<DataType, MaxDataNum>::DebugString(
    const pool_type& pool) const {
  std::ostringstream os;

  os << "-------------------- DoublyLinkedList -------------------------->"
     << std::endl;

  // os << "**Data pool:\n";
  // os << pool.DebugString();

  os << "**head=" << head_ << ", tail=" << tail_ << "\n";

  os << "**node list (head->tail) :\n";
  Int32_t offset = head_;
  const node_type* pddr = pool.GetVirtualAddr(offset);
  Int32_t i = 0;
  while (!pool.IsNull(offset)) {
    os << "**[" << i++
       << "] node_addr("<< pddr
       << "),current(" << offset
       << "),next("<< pddr->next
       << "),prev("<< pddr->prev
       << ")\n";
    offset = pddr->next;
    pddr = pool.GetVirtualAddr(offset);
  }

  os << "**node list (tail->head) :\n";
  offset = tail_;
  pddr = pool.GetVirtualAddr(offset);
  i = 0;
  while (!pool.IsNull(offset)) {
    os << "**[" << i++
       << "] node_addr("<< pddr
       << "),current(" << offset
       << "),next("<< pddr->next
       << "),prev("<< pddr->prev
       << ")\n";
    offset = pddr->prev;
    pddr = pool.GetVirtualAddr(offset);
  }

  os << "<------------------- DoublyLinkedList --------------------------"
    << std::endl;

  return (os.str());
}



}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_DOUBLY_LINKED_LIST_H_
