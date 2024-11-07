/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       aabboxkdtree2d.h
 * @brief      kd-tree in 2 dimension
 * @details    Used to partition objects bounded by AABBox in 2-dimension
 *
 * @author     pengc
 * @date       2020.03.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/03/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_AABBOXKDTREE2D_H_
#define PHOENIX_COMMON_AABBOXKDTREE2D_H_

#include "utils/log.h"
#include "container/static_vector.h"
#include "geometry/aabbox2d.h"
#include "math/math_utils.h"

// Enable to output debug information
#define ENABLE_AABBOXKDTREE2D_TRACE (0)

/**
 * @name 遍历双向链表
 * @brief 定义宏用来遍历内部的双向链表
 * @{
 */
#define TRAVERSE_CONST_DOUBLE_LIST_BEGIN(ListNode, DataNum, list, storage)   \
{                                                                            \
  internal::DoubleList::const_iterator<ListNode, DataNum> it =               \
      list.cbegin(storage);                                                  \
  internal::DoubleList::const_iterator<ListNode, DataNum> it_end =           \
      list.cend(storage);                                                    \
  for (; it != it_end; ++it) {

#define ACCESS_CONST_DOUBLE_LIST_NODE it

#define TRAVERSE_CONST_DOUBLE_LIST_END  \
  }                                     \
}
/** @} 遍历双向链表 */


namespace phoenix {
namespace common {

/// 定义内部使用的功能
namespace internal {

/**
 * @class DoubleList
 * @brief 双向链表（在外部输入的数组中构建双向链表）
 */
class DoubleList {
private:
  typedef Int32_t int_type;

public:
  /**
   * @brief 构造函数
   */
  DoubleList() {
    head_ = -1;
    tail_ = -1;
  }

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    head_ = -1;
    tail_ = -1;
  }

  /**
   * @brief 获取链表头部索引
   * @return 链表头部索引（-1为无效索引）
   */
  int_type head() const { return (head_); }

  /**
   * @brief 获取链表尾部索引
   * @return 链表尾部索引（-1为无效索引）
   */
  int_type tail() const { return (tail_); }

  /**
   * @brief 判断链表是否是空的
   * @return true - 空的，false - 非空
   */
  bool Empty() const {
    return (head_ < 0);
  }

  /**
   * @class iterator
   * @brief 双向链表的迭代器
   */
  template <typename NodeType, Int32_t NodeNum>
  class iterator;
  template <typename NodeType, Int32_t NodeNum>
  friend class iterator;
  template <typename NodeType, Int32_t NodeNum>
  class iterator {
   public:
    typedef DoubleList list_type;
    typedef StaticVector<NodeType, NodeNum> storage_type;

   private:
    // 指向链表的指针
    list_type* list_;
    // 链表中元素的索引
    int_type index_;
    // 指向元素存储区域的指针
    storage_type* storage_;

   public:
    /**
     * @brief 构造函数
     * @param[in] list 链表的引用
     * @param[in] storage 元素存储区域的引用
     * @param[in] is_full 当前环形数据缓冲区是否是满的
     */
    iterator(list_type& list, storage_type& storage)
        : list_(&list), index_(list.head_), storage_(&storage) {}
    /**
     * @brief 构造函数
     * @param[in] list 链表的引用
     * @param[in] storage 元素存储区域的引用
     * @param[in] bool 用来构建链表尾部的索引
     */
    iterator(list_type& list, storage_type& storage, bool)
        : list_(&list), index_(-1), storage_(&storage) {}

    /**
     * @brief 拷贝构造函数
     * @param[in] rv 另一个迭代器
     */
    iterator(const iterator& rv) :
      list_(rv.list_), index_(rv.index_), storage_(rv.storage_) {}
    /**
     * @brief 拷贝函数
     * @param[in] rv 另一个迭代器
     */
    iterator& operator=(const iterator& rv) {
      list_ = rv.list_;
      index_ = rv.index_;
      storage_ = rv.storage_;
      return (*this);
    }

    /**
     * @brief 迭代器自增
     * @return 自增后的迭代器
     */
    iterator& operator++() {
      if (index_ >= 0) {
        index_ = (*storage_)[index_].next;
      }
      return (*this);
    }

    /**
     * @brief 判断当前迭代器是否与另一个迭代器相等
     * @param[in]  rv 另一个迭代器
     * @return true-相等, false-不相等
     */
    bool operator==(const iterator& rv) const { return (index_ == rv.index_); }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器不相等
     * @param[in]  rv 另一个迭代器
     * @return true-不相等, false-相等
     */
    bool operator!=(const iterator& rv) const { return (index_ != rv.index_); }

    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    NodeType* current() {
      if (index_ >= 0) {
        return (&((*storage_)[index_]));
      }
      return (Nullptr_t);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    NodeType& operator*() { return *current(); }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    NodeType* operator->() { return current(); }

    /**
     * @brief 从链表中移除当前迭代器所指向的数据
     * @return 被移除的数据
     */
    NodeType* remove() {
      if (index_ >=0 ) {
        int_type original = index_;
        index_ = (*storage_)[index_].next;
        list_->Erase(original, *storage_);
        return (&((*storage_)[original]));
      }
      return (Nullptr_t);
    }
    /**
     * @brief 从链表中移除当前迭代器所指向的数据
     * @return 被移除的数据的索引
     */
    int_type remove(bool) {
      if (index_ >=0 ) {
        int_type original = index_;
        index_ = (*storage_)[index_].next;
        list_->Erase(original, *storage_);
        return (original);
      }
      return (-1);
    }
    /**
     * @brief 从链表中移除当前迭代器所指向的数据
     * @return 更新后的迭代器（指向被移除的数据的下一个数据）
     */
    iterator& erase() {
      if (index_ >= 0) {
        int_type next_node = (*storage_)[index_].next;
        list_->Erase(index_, *storage_);
        index_ = next_node;
      }
      return (*this);
    }
  };
  /**
   * @brief 返回双向链表的起始位置的迭代器
   * @return 双向链表的起始位置的迭代器
   */
  template <typename NodeType, Int32_t NodeNum>
  iterator<NodeType, NodeNum> begin(
      StaticVector<NodeType, NodeNum>& storage) {
    return iterator<NodeType, NodeNum>(*this, storage);
  }
  /**
   * @brief 返回双向链表的结束位置的迭代器
   * @return 双向链表的结束位置的迭代器
   */
  template <typename NodeType, Int32_t NodeNum>
  iterator<NodeType, NodeNum> end(
      StaticVector<NodeType, NodeNum>& storage) {
    return iterator<NodeType, NodeNum>(*this, storage, true);
  }

  /**
   * @class iterator
   * @brief 双向链表的迭代器(常量型)
   */
  template <typename NodeType, Int32_t NodeNum>
  class const_iterator;
  template <typename NodeType, Int32_t NodeNum>
  friend class const_iterator;
  template <typename NodeType, Int32_t NodeNum>
  class const_iterator {
   public:
    typedef DoubleList list_type;
    typedef StaticVector<NodeType, NodeNum> storage_type;

   private:
    // 指向链表的指针
    const list_type* list_;
    // 链表中元素的索引
    int_type index_;
    // 指向元素存储区域的指针
    const storage_type* storage_;

   public:
    /**
     * @brief 构造函数
     * @param[in] list 链表的引用
     * @param[in] storage 元素存储区域的引用
     * @param[in] is_full 当前环形数据缓冲区是否是满的
     */
    const_iterator(const list_type& list, const storage_type& storage)
        : list_(&list), index_(list.head_), storage_(&storage) {}
    /**
     * @brief 构造函数
     * @param[in] list 链表的引用
     * @param[in] storage 元素存储区域的引用
     * @param[in] bool 用来构建链表尾部的索引
     */
    const_iterator(const list_type& list, const storage_type& storage, bool)
        : list_(&list), index_(-1), storage_(&storage) {}

    /**
     * @brief 拷贝构造函数
     * @param[in] rv 另一个迭代器
     */
    const_iterator(const const_iterator& rv) :
      list_(rv.list_), index_(rv.index_), storage_(rv.storage_) {}
    /**
     * @brief 拷贝函数
     * @param[in] rv 另一个迭代器
     */
    const_iterator& operator=(const const_iterator& rv) {
      list_ = rv.list_;
      index_ = rv.index_;
      storage_ = rv.storage_;
      return (*this);
    }

    /**
     * @brief 迭代器自增
     * @return 自增后的迭代器
     */
    const_iterator& operator++() {
      if (index_ >= 0) {
        index_ = (*storage_)[index_].next;
      }
      return (*this);
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器相等
     * @param[in]  rv 另一个迭代器
     * @return true-相等, false-不相等
     */
    bool operator==(const const_iterator& rv) const {
      return (index_ == rv.index_);
    }
    /**
     * @brief 判断当前迭代器是否与另一个迭代器不相等
     * @param[in]  rv 另一个迭代器
     * @return true-不相等, false-相等
     */
    bool operator!=(const const_iterator& rv) const {
      return (index_ != rv.index_);
    }

    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    const NodeType* current() const {
      if (index_ >= 0) {
        return (&((*storage_)[index_]));
      }
      return (Nullptr_t);
    }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    const NodeType& operator*() const { return *current(); }
    /**
     * @brief 返回当前迭代器所指向的数据
     * @return 当前迭代器所指向的数据
     */
    const NodeType* operator->() const { return current(); }
  };
  /**
   * @brief 返回双向链表的起始位置的迭代器
   * @return 双向链表的起始位置的迭代器
   */
  template <typename NodeType, Int32_t NodeNum>
  const_iterator<NodeType, NodeNum> cbegin(
      const StaticVector<NodeType, NodeNum>& storage) const {
    return const_iterator<NodeType, NodeNum>(*this, storage);
  }
  /**
   * @brief 返回双向链表的结束位置的迭代器
   * @return 双向链表的结束位置的迭代器
   */
  template <typename NodeType, Int32_t NodeNum>
  const_iterator<NodeType, NodeNum> cend(
      const StaticVector<NodeType, NodeNum>& storage) const {
    return const_iterator<NodeType, NodeNum>(*this, storage, true);
  }

  /**
   * @brief 在数组中构建双向链表
   * @param[in] storage 需要在内部构建双向链表的数组
   */
  template <typename NodeType, Int32_t NodeNum>
  void ConstructInVector(StaticVector<NodeType, NodeNum>& storage);

  /**
   * @brief 获取链表头部的数据
   * @param[in] storage 存储链表的数组
   * @return 链表头部的数据
   */
  template <typename NodeType, Int32_t NodeNum>
  NodeType* Front(
      StaticVector<NodeType, NodeNum>& storage) {
    if (head_ < 0) {
      return Nullptr_t;
    }

    return (&(storage[head_]));
  }

  /**
   * @brief 获取链表尾部的数据
   * @param[in] storage 存储链表的数组
   * @return 链表尾部的数据
   */
  template <typename NodeType, Int32_t NodeNum>
  NodeType* Back(
      StaticVector<NodeType, NodeNum>& storage) {
    if (tail_ < 0) {
      return Nullptr_t;
    }

    return (&(storage[tail_]));
  }

  /**
   * @brief 获取链表头部的数据
   * @param[in] storage 存储链表的数组
   * @return 链表头部的数据
   */
  template <typename NodeType, Int32_t NodeNum>
  const NodeType* Front(
      const StaticVector<NodeType, NodeNum>& storage) const {
    if (head_ < 0) {
      return Nullptr_t;
    }

    return (&(storage[head_]));
  }

  /**
   * @brief 获取链表尾部的数据
   * @param[in] storage 存储链表的数组
   * @return 链表尾部的数据
   */
  template <typename NodeType, Int32_t NodeNum>
  const NodeType* Back(
      const StaticVector<NodeType, NodeNum>& storage) const {
    if (tail_ < 0) {
      return Nullptr_t;
    }

    return (&(storage[tail_]));
  }

  /**
   * @brief 添加一个数据到链表尾部
   * @param[in] new_node 新数据的索引
   * @param[in] storage 存储链表的数组
   */
  template <typename NodeType, Int32_t NodeNum>
  void PushBack(
      const int_type new_node,
      StaticVector<NodeType, NodeNum>& storage) {
    Insert<NodeType, NodeNum>(tail_, new_node, storage);
  }

  /**
   * @brief 从链表中移除一个数据
   * @param[in] del_node 需要移除的数据的索引
   * @param[in] storage 存储链表的数组
   * @warning {不要多次删除同一个节点，否则链表将出错}
   */
  template <typename NodeType, Int32_t NodeNum>
  void Erase(
      const int_type del_node,
      StaticVector<NodeType, NodeNum>& storage);

  /**
   * @brief 从链表中移除一个数据
   * @param[in] it 需要移除的数据的迭代器
   * @return 被移除的数据的迭代器
   */
  template <typename NodeType, Int32_t NodeNum>
  iterator<NodeType, NodeNum> Erase(const iterator<NodeType, NodeNum>& it) {
    iterator<NodeType, NodeNum> tmp_it = it;
    tmp_it.erase();
    return (tmp_it);
  }

  /**
   * @brief 将链表中的数据排序
   * @param[in] storage 存储链表的数组
   * @param[in] cmp_func 比较函数
   * @par Note:
   * @code
   *     Using Selection-Sort algorithm
   *     Time complexity, Best: T(n) = O(n2),
   *     Worst: T(n) = O(n2), Average: T(n) = O(n2)
   *     Space complexity: S(n) = O(n2)
   * @endcode
   */

  template <typename NodeType, Int32_t NodeNum, typename CmpFunc>
  void Sort(StaticVector<NodeType, NodeNum>& storage, const CmpFunc& cmp_func);

#if ENABLE_AABBOXKDTREE2D_TRACE
  /**
   * @brief 获取调试信息
   * @param[in] storage 存储链表的数组
   * @return 调试信息
   */
  template <typename NodeType, Int32_t NodeNum>
  std::string DebugString(
      StaticVector<NodeType, NodeNum>& storage) const;
#endif

private:
  /**
   * @brief 向链表中插入一个数据
   * @param[in] front 链表中某个节点的索引，表示在front之后插入新数据
   * @param[in] new_node 待插入节点的索引
   * @param[in] storage 存储链表的数组
   */
  template <typename NodeType, Int32_t NodeNum>
  void Insert(
      const int_type front, const int_type new_node,
      StaticVector<NodeType, NodeNum>& storage);

  /**
   * @brief 交换两个链表节点
   * @param[in] n1 节点1
   * @param[in] n2 节点2
   * @param[in] storage 存储链表的数组
   */
  template <typename NodeType, Int32_t NodeNum>
  void Swap(int_type n1, int_type n2,
             StaticVector<NodeType, NodeNum>& storage);

private:
  // 头节点的索引
  int_type head_;
  // 尾节点的索引
  int_type tail_;
};

template <typename NodeType, Int32_t NodeNum>
void DoubleList::ConstructInVector(
    StaticVector<NodeType, NodeNum>& storage) {
  if (storage.Empty()) {
    return;
  }

  int_type nodes_size = storage.Size();
  for (int_type i = 0; i < (nodes_size-1); ++i) {
    storage[i].next = i+1;
    storage[i].prev = i-1;
  }
  storage[nodes_size-1].next = -1;
  storage[nodes_size-1].prev = nodes_size-2;

  head_ = 0;
  tail_ = nodes_size-1;
}

template <typename NodeType, Int32_t NodeNum>
void DoubleList::Erase(
    const int_type del_node,
    StaticVector<NodeType, NodeNum>& storage) {
  if (del_node < 0) {
    return;
  }

  int_type next = storage[del_node].next;
  int_type prev = storage[del_node].prev;
  if (next >= 0) {
    storage[next].prev = prev;
  } else {
    tail_ = prev;
  }
  if (prev >= 0) {
    storage[prev].next = next;
  } else {
    head_ = next;
  }
  storage[del_node].next = -1;
  storage[del_node].prev = -1;
}

template <typename NodeType, Int32_t NodeNum, typename CmpFunc>
void DoubleList::Sort(
    StaticVector<NodeType, NodeNum>& storage,
    const CmpFunc& cmp_func) {
  if (Empty()) {
    return;
  }

  int_type first_node = head_;
  int_type next_node = -1;
  int_type tmp = 0;

  do {
    next_node = storage[first_node].next;

    for (; next_node >= 0; next_node = storage[next_node].next) {
      if (cmp_func(next_node, first_node)) {
        Swap(first_node, next_node, storage);
        tmp = first_node;
        first_node = next_node;
        next_node= tmp;
      }
    }

    first_node = storage[first_node].next;

  } while (first_node >= 0);
}

template <typename NodeType, Int32_t NodeNum>
void DoubleList::Swap(
    int_type n1, int_type n2,
    StaticVector<NodeType, NodeNum>& storage) {
  COM_CHECK(n1 != n2);
  COM_CHECK(n1 >= 0 && n2 >= 0);

  int_type n1_prev = storage[n1].prev;
  int_type n1_next = storage[n1].next;
  int_type n2_prev = storage[n2].prev;
  int_type n2_next = storage[n2].next;

  if (n1_prev == n2 || n2_next == n1) {
    // from "n2_prev -> n2 -> n1 -> n1_next"
    //   to "n2_prev -> n1 -> n2 -> n1_next"
    // invalid: storage[n1_prev].next = n2; because n1_prev == n2
    storage[n2].prev = n1;
    storage[n2].next = n1_next;
    if (n1_next >= 0) storage[n1_next].prev = n2;
    if (n2_prev >= 0) storage[n2_prev].next = n1;
    storage[n1].prev = n2_prev;
    storage[n1].next = n2;
    // invalid: storage[n2_next].prev = n1; because n2_next == n1
  } else if (n2_prev == n1 || n1_next == n2) {
    // from "n1_prev -> n1 -> n2 -> n2_next"
    //   to "n1_prev -> n2 -> n1 -> n2_next"
    if (n1_prev >= 0) storage[n1_prev].next = n2;
    storage[n2].prev = n1_prev;
    storage[n2].next = n1;
    // invalid: storage[n1_next].prev = n2; because n1_next == n2
    // invalid: storage[n2_prev].next = n1; because n2_prev == n1
    storage[n1].prev = n2;
    storage[n1].next = n2_next;
    if (n2_next >= 0) storage[n2_next].prev = n1;
  } else {
    // from "n1_prev -> n1 -> n1_next n2_prev -> n2 -> n2_next"
    //   to "n1_prev -> n2 -> n1_next n2_prev -> n1 -> n2_next"
    // or inverse
    if (n1_prev >= 0) storage[n1_prev].next = n2;
    storage[n2].prev = n1_prev;
    storage[n2].next = n1_next;
    if (n1_next >= 0) storage[n1_next].prev = n2;
    if (n2_prev >= 0) storage[n2_prev].next = n1;
    storage[n1].prev = n2_prev;
    storage[n1].next = n2_next;
    if (n2_next >= 0) storage[n2_next].prev = n1;
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

template <typename NodeType, Int32_t NodeNum>
void DoubleList::Insert(
    const int_type front, const int_type new_node,
    StaticVector<NodeType, NodeNum>& storage) {
  if (new_node < 0) {
    return;
  }

  if (head_ < 0) {
    // The list is empty.
    head_ = new_node;
    tail_ = new_node;
    storage[new_node].prev = -1;
    storage[new_node].next = -1;
  } else if (front >= 0) {
    // Insert new node in the back of "front"
    int_type next_node = storage[front].next;
    if (next_node >= 0) {
      storage[next_node].prev = new_node;
    }
    storage[new_node].next = next_node;
    storage[new_node].prev = front;
    storage[front].next = new_node;
    if (tail_ == front) {
      tail_ = new_node;
    }
  } else {
    int_type next_node = head_;
    if (next_node >= 0) {
      storage[next_node].prev = new_node;
    }
    storage[new_node].next = next_node;
    storage[new_node].prev = -1;
    head_ = new_node;
  }
}

#if ENABLE_AABBOXKDTREE2D_TRACE
template <typename NodeType, Int32_t NodeNum>
std::string DoubleList::DebugString(
    StaticVector<NodeType, NodeNum>& storage) const {
  std::ostringstream os;

  os << "-------------------- Double List -------------------------->"
     << std::endl;

  os << "**head=" << head_ << ", tail=" << tail_ << "\n";

  os << "**node list (head->tail) :\n";
  int_type index = head_;
  int_type i = 0;
  while (index >= 0) {
    os << "**[" << i++
       << "] current(" << index
       << "),next(" << storage[index].next
       << "),prev(" << storage[index].prev
       << ")\n";
    index = storage[index].next;
  }

  os << "**node list (tail->head) :\n";
  index = tail_;
  i = 0;
  while (index >= 0) {
    os << "**[" << i++
       << "] current(" << index
       << "),next("<< storage[index].next
       << "),prev("<< storage[index].prev
       << ")\n";
    index = storage[index].prev;
  }

  os << "<------------------- Double List --------------------------"
    << std::endl;

  return (os.str());
}
#endif

}  // namespace internal


/**
 * @struct AABBoxKDTreeParams
 * @brief Contains parameters of axis-aligned bounding box.
 */
struct AABBoxKDTreeParams {
  /// The maximum depth of the kdtree.
  Int32_t max_depth;
  /// The maximum number of items in one leaf node.
  Int32_t max_leaf_size;
  /// The maximum dimension size of leaf node.
  geo_var_t max_leaf_dimension;

  void Clear() {
    max_depth = -1;
    max_leaf_size = -1;
    max_leaf_dimension = -1;
  }

  AABBoxKDTreeParams() {
    Clear();
  }
};

/**
 * @struct AABBoxKDTreeNodeAssociation
 * @brief Contains relation to the tree nodes. \n
 *        Used to trace the path when traverse a kd-tree in a stack.
 */
struct AABBoxKDTreeNodeAssociation {
  typedef Int32_t int_type;
  /// The index of storage of a kd-tree
  int_type tree_node_index;
  /// Used to control the branch of path when travere a kd-tree
  int_type phase;
  /// Used to store objects of the node of a tree
  /// in the partition algorithm.
  internal::DoubleList objects;

  /// Initialize
  void Clear() {
    tree_node_index = -1;
    phase = 0;
    objects.Clear();
  }

  AABBoxKDTreeNodeAssociation() {
    Clear();
  }
};


/**
 * @class AABBoxKDTree2d
 * @brief The class of KD-tree of Aligned Axis Bounding Box(AABox).
 * @param DataType The type of user's data
 * @param DataNum The max size of the storage used to store user's data.
 * @param MaxLeafSize The maximum number of items in one leaf node. \n
 *     If the number of objects belonged to one leaf node is less then \n
 *     this size, these objects will not be split again, \n
 *     and assigned to this node.
 */
template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize=16>
class AABBoxKDTree2d {
public:
  typedef Int32_t int_type;
  typedef geo_var_t float_type;

public:
  /**
   * @brief Contructor.
   */
  AABBoxKDTree2d();
  /**
   * @brief Contructor with parameters.
   * @param[in] param Parameters to build the KD-tree.
   */
  AABBoxKDTree2d(const AABBoxKDTreeParams& param);

  /**
   * @brief 设置KD树的参数
   * @param[in] param 构建KD树时需要用到的参数。
   */
  void SetKDTreeParams(const AABBoxKDTreeParams& param);

  void Clear() {
    objects_.Clear();
    object_list_storage_.Clear();
    object_list_storage_copy_.Clear();
    tree_nodes_.Clear();
  }

  /**
   * @brief Add an object to the tree.
   * @param[in] data User's data
   * @param[in] box  Aligned Axis Bounding box of User's data.
   * @return true successful \n
   *         false failed
   */
  bool AddObject(const DataType& data, const AABBox2d& box);

  /**
   * @brief Get number of objects stored in this tree.
   * @return The number of objects
   */
  int_type GetObjectsSize() const {
    return (objects_.Size());
  }

  /**
   * @brief Get bounding box stored in the tree by index of storage of objects.
   * @param[in] index Index of storage of objects.
   * @return The pointer of the bounding box stored in the tree.
   */
  const AABBox2d* GetObjectBoxByIndex(int_type index) const {
    if (0 <= index && index < objects_.Size()) {
      return (&(objects_[index].box));
    }

    return Nullptr_t;
  }

  /**
   * @brief Get user's data stored in the tree by index of storage of objects.
   * @param[in] index Index of storage of objects.
   * @return The pointer of the user's data stored in the tree.
   */
  const DataType* GetObjectByIndex(int_type index) const {
    if (0 <= index && index < objects_.Size()) {
      return (&(objects_[index].user_data));
    }

    return Nullptr_t;
  }

  /**
   * @brief Partition the objects bounded by Aligned Axis Bounding box.
   * @param[in] tree_stack The stack is used to trace the path \n
   *                   of traversing the kd-tree.
   * @warning {The size of the stack must be greater then twice as depth of the tree.}
   */
  template <Int32_t MaxStackNodesNum>
  void Partition(
      StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack);

  /**
   * @brief Find objects in the range of circle.
   * @param[in] tree_stack The stack is used to trace the \n
   *   path of traversing the kd-tree.
   * @warning {The size of the stack must be greater than the depth of the tree.}
   * @param[in] point The center point of the circle limiting the \n
   *   range of objects.
   * @param[in] distance The radius of circle limiting the range of objects.
   * @param[in] calc_sq_dist_to_pt A function operator used to calculate the \n
   *   square distance of one object to the point in the parameter list.
   * @param[out] result_objs Store objects found in the tree, \n
   *   which are in the range indicated by user.
   *
   * @par Note:
   * @code
   *     The "CalcSquareDistToPt" is a class in which an operator() needs to
   *     be impleted. The operator is used to calculate the square distance \n
   *     between an object and a point.
   *     The parameters of the operator are User's object and index
   *     of objects stored in the tree. One can use the User's object or the
   *     index of objects (Using "GetObjectByIndex" to get the User's object)
   *     to calculate the square distance to the point in the parameter list.
   *
   *     Example:
   *         class CalcSquareDistToPt {
   *         public:
   *           CalcSquareDistToPt(
   *               const Vec2d& point,
   *               const AABBoxKDTree2d& tree)
   *             : point_(point), tree_(tree) {}
   *
   *           double operator ()(
   *               const UserData& data, Int32_t tree_obj_index) const {
   *             return (data.DistanceSquareTo(point_));
   *               or:
   *             return (tree_.GetObjectByIndex(tree_obj_index)->
   *                                                DistanceSquareTo(point_));
   *           }
   *
   *         private:
   *           const Vec2d point_;
   *           const AABBoxKDTree2d& tree_;
   *         };
   *
   * @endcode
   */
  template <typename CalcSquareDistToPt, Int32_t MaxObjectsNum,
            Int32_t MaxStackNodesNum>
  void FindObjects(
      StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack,
      const Vec2d &point,
      const float_type distance,
      const CalcSquareDistToPt& calc_sq_dist_to_pt,
      StaticVector<int_type, MaxObjectsNum>* result_objs,
      bool always_check_dist = false) const;

  /**
   * @brief Find objects nearest to the point indicated by user.
   * @param[in] tree_stack The stack is used to trace the \n
   *   path of traversing the kd-tree.
   * @warning {The size of the stack must be greater then the depth of the tree.}
   * @param[in] point The point is used to find the nearest object to it in the tree.
   * @param[in] calc_sq_dist_to_pt A function operator used to calculate the \n
   *   square distance of one object to the point in the parameter list.
   * @param[out] nearest_obj_index The index of storage of the objects in the \n
   *   tree, which indicate the nearest object to the point indicated by user.
   * @param[out] min_distance_sqr The square distance between the nearest \n
   *   object and the point.
   *
   * @par Note:
   * @code
   *     The "CalcSquareDistToPt" is a class in which an operator() need to be
   *     impleted. The operator is used to calculate the square distance \n
   *     between an object and a point.
   *     See also the description of the "FindObjects".
   *
   * @endcode
   */
  template <typename CalcSquareDistToPt, Int32_t MaxStackNodesNum>
  const DataType* FindNearest(
      StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack,
      const Vec2d &point,
      const CalcSquareDistToPt& calc_sq_dist_to_pt,
      int_type* nearest_obj_index,
      float_type* min_distance_sqr) const;

#if ENABLE_AABBOXKDTREE2D_TRACE
  template <Int32_t MaxStackNodesNum>
  std::string DebugString(
      StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack);
#endif

private:
  // Define the type of the object in the tree
  struct Object {
    // Aligned Axis Bounding box of user's data
    AABBox2d box;
    // User's data input from user
    DataType user_data;

    Object() {
    }

    Object(const DataType& data, const AABBox2d& aabbox) {
      user_data = data;
      box = aabbox;
    }
  };

  // Define the type of node of the list associated with objects
  struct ListNode {
    // index referenced to next/previous node
    int_type next;
    int_type prev;
    // index of storage of objects
    int_type object_index;

    void Clear() {
      next = -1;
      prev = -1;
      object_index = -1;
    }

    ListNode() {
      Clear();
    }
  };

  // Boundary of objects of this node
  struct Boundary {
    float_type min_x;
    float_type max_x;
    float_type min_y;
    float_type max_y;
    float_type mid_x;
    float_type mid_y;

    void Clear() {
      min_x = 0;
      max_x = 0;
      min_y = 0;
      max_y = 0;
      mid_x = 0;
      mid_y = 0;
    }

    Boundary() {
      Clear();
    }
  };

  // partition type
  enum {
    PARTITION_X = 0,
    PARTITION_Y
  };

  // type of kd-tree node
  struct TreeNode {
    // number of objects
    int_type num_objects;
    // depth of this node in the whole tree
    int_type depth;

    // partition axis
    int_type partition;
    float_type partition_position;

    // boundary
    Boundary boundary;

    // list stored index of storage of objects
    internal::DoubleList objects_sorted_by_min;
    internal::DoubleList objects_sorted_by_max;

    // index referenced to subnode
    int_type left_subnode;
    int_type right_subnode;

    // Initialize member variables
    void Clear() {
      num_objects = 0;
      depth = 0;
      partition = PARTITION_X;
      partition_position = 0;

      boundary.Clear();

      objects_sorted_by_min.Clear();
      objects_sorted_by_max.Clear();

      left_subnode = -1;
      right_subnode = -1;
    }

    TreeNode() {
      Clear();
    }
  };

  // Define function operator used in sort algorithm bellow
  class ObjCmpByMinFunc {
  public:
    ObjCmpByMinFunc(
          int_type partition,
          const StaticVector<ListNode, DataNum>& storage,
          const AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>& tree)
      : partition_(partition), storage_(storage), tree_(tree) {}
    bool operator ()(int_type list_index1, int_type list_index2) const {
      int_type obj_index1 = storage_[list_index1].object_index;
      int_type obj_index2 = storage_[list_index2].object_index;
      const Object& obj1 = tree_.objects_[obj_index1];
      const Object& obj2 = tree_.objects_[obj_index2];
      return (partition_ == PARTITION_X
              ? obj1.box.min_x() < obj2.box.min_x()
              : obj1.box.min_y() < obj2.box.min_y());
    }
  private:
    int_type partition_;
    const StaticVector<ListNode, DataNum>& storage_;
    const AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>& tree_;
  };

  class ObjCmpByMaxFunc {
  public:
    ObjCmpByMaxFunc(
          int_type partition,
          const StaticVector<ListNode, DataNum>& storage,
          const AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>& tree)
      : partition_(partition), storage_(storage), tree_(tree) {}
    bool operator ()(int_type list_index1, int_type list_index2) const {
      int_type obj_index1 = storage_[list_index1].object_index;
      int_type obj_index2 = storage_[list_index2].object_index;
      const Object& obj1 = tree_.objects_[obj_index1];
      const Object& obj2 = tree_.objects_[obj_index2];
      return (partition_ == PARTITION_X
              ? obj1.box.max_x() > obj2.box.max_x()
              : obj1.box.max_y() > obj2.box.max_y());
    }
  private:
    int_type partition_;
    const StaticVector<ListNode, DataNum>& storage_;
    const AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>& tree_;
  };

private:
  /*
   * @brief Calculate Aligned Axis Bounding box of input objects.
   * @param[in] objs The list of objects.
   * @param[out] boundary The boundary of the objects input from user.
   */
  void ComputeBoundary(const internal::DoubleList& objs,
                       Boundary* boundary) const;
  /*
   * @brief Calculate partition axis and position according to \n
   *     boundary of objects.
   * @param[in] boundary The boundary of objects.
   * @param[out] partition Partition axis.\n
   *     PARTITION_X Split objects along X axis.\n
   *     PARTITION_Y Split objects along Y axis.
   * @param partition_position Partition position on partition axis.
   */
  void ComputePartition(const Boundary& boundary,
                        int_type* partition,
                        float_type* partition_position) const;
  /*
   * @brief Decide if the objects belonged to this tree node need to split again.
   * @param[in] depth The depth of this node in the tree.
   * @param[in] obj_num The number of objects of this tree node.
   * @param[in] boundary The boundary of objects of this tree node.
   * @return true - need to split these objects again. \n
   *         false - do not need to split.
   */
  bool SplitToSubNodes(const int_type depth,
                       const int_type obj_num,
                       const Boundary& boundary);
  /*
   * @brief Partion objects according to partition axis and position.
   * @param[in] partition Partition axis. \n
   *     PARTITION_X Split objects along X axis.\n
   *     PARTITION_Y Split objects along Y axis.
   * @param[in] partition_position Partition position on partition axis.
   * @param[in&out] objs As input, store objects which will be partition. \n
   *     As output, store remains of objects after partition.
   * @param[out] left_objs Store objects assigned to left subnode after \n
   *     partition.
   * @param[out] right_objs Store objects assigned to right subnode after \n
   *     partition.
   * @param[out] objs_num The number of remains of objects.
   * @param[out] left_objs_num The number of left objects.
   * @param[out] right_objs_num The number of right objects.
   */
  void PartitionObjects(const int_type partition,
                        const float_type partition_position,
                        internal::DoubleList* objs,
                        internal::DoubleList* left_objs,
                        internal::DoubleList* right_objs,
                        int_type* objs_num,
                        int_type* left_objs_num,
                        int_type* right_objs_num);
  /*
   * @brief Calculate the min distance between the point and the boundary box.
   * @param[in] point The point coordinate.
   * @param[in] boundary The boundary information.
   * @return The square distance.
   */
  float_type LowerDistanceSquareToPoint(const Vec2d &point,
                                        const Boundary& boundary) const;
  /*
   * @brief Calculate the max distance between this point and the boundary \n
   *     box. In the other words, it is the distance between the point and \n
   *     the farthest corner of the boundary box to this point.
   * @param[in] point The point coordinate.
   * @param[in] boundary The boundary information.
   * @return The square distance.
   */
  float_type UpperDistanceSquareToPoint(const Vec2d &point,
                                        const Boundary& boundary) const;

private:
  // Parameters of the tree
  AABBoxKDTreeParams params_;

  // Array stored user's objects
  StaticVector<Object, DataNum> objects_;

  // Array stored nodes of list
  StaticVector<ListNode, DataNum> object_list_storage_;
  StaticVector<ListNode, DataNum> object_list_storage_copy_;

  // Array stored tree nodes
  // The max number of tree nodes is (2*n-1), where the n is the max number
  // of nodes in the lowest level of tree. And the max depth of the tree
  // is log(2, n). ===>(provided by boc)
  // After partition the root node of the tree is tree_nodes_[0].
  StaticVector<TreeNode, 2*(DataNum+1)+1> tree_nodes_;
};


template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::AABBoxKDTree2d() {
  params_.max_leaf_size = MaxLeafSize;
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::AABBoxKDTree2d(
    const AABBoxKDTreeParams& param) {
  params_ = param;
  if (params_.max_leaf_size < MaxLeafSize) {
    params_.max_leaf_size = MaxLeafSize;
  }
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
void AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::SetKDTreeParams(
    const AABBoxKDTreeParams& param) {
  params_ = param;
  if (params_.max_leaf_size < MaxLeafSize) {
    params_.max_leaf_size = MaxLeafSize;
  }
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
bool AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::AddObject(
    const DataType& data, const AABBox2d& box) {
  if (objects_.Full()) {
    return false;
  }

  Object obj(data, box);
  objects_.PushBack(obj);

  return true;
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
template <Int32_t MaxStackNodesNum>
void AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::Partition(
    StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack) {
  if (objects_.Empty()) {
    return;
  }
#if ENABLE_AABBOXKDTREE2D_TRACE
  std::cout << "################ Partition ################>" << std::endl;
  std::cout << "Max number of tree nodes is " << 2*(DataNum+2)+1
            << std::endl;
#endif

  // Create the relations stored in the "object_list_storage_" to the objects
  int_type objects_num = objects_.Size();
  int_type left_objects_num = 0;
  int_type right_objects_num = 0;
  object_list_storage_.Resize(objects_num);
  for (int_type i = 0; i < objects_num; ++i) {
    object_list_storage_[i].object_index = i;
  }
  object_list_storage_copy_.Clear();

  // Put the relations of the objests to the list
  internal::DoubleList objects;
  objects.ConstructInVector(object_list_storage_);
  internal::DoubleList left_objects;
  internal::DoubleList right_objects;

  // Create the root node of the tree
  tree_nodes_.Clear();
  int_type tree_node_index = -1;
  TreeNode* tree_node = tree_nodes_.Allocate(&tree_node_index);
  COM_CHECK(Nullptr_t != tree_node);
  tree_node->Clear();

  // Push the root node of the tree to the stack
  tree_stack.Clear();
  AABBoxKDTreeNodeAssociation* tree_node_ass = tree_stack.Allocate();
  COM_CHECK(Nullptr_t != tree_node_ass);
  tree_node_ass->Clear();
  tree_node_ass->tree_node_index = tree_node_index;
  tree_node_ass->phase = objects_num;
  tree_node_ass->objects = objects;

#if ENABLE_AABBOXKDTREE2D_TRACE
  int_type recursion_count = 0;
#endif
  while (!(tree_stack.Empty())) {
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << std::endl << "@@@@@>recursion_count="
              << recursion_count++ << std::endl;
#endif
    // Get top node of the stack which stored associations of tree nodes
    tree_node_ass = &(tree_stack.Back());
    // Get objects would be split in the following
    objects = tree_node_ass->objects;
    if (objects.Empty()) {
      // This objects of the stack had been split before,
      // so the current node need to be skipped.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "<------This objects of the stack had been split before."
                << std::endl;
#endif
      tree_stack.PopBack();
      continue;
    }
    // Get index of tree nodes from this node of stack
    tree_node_index = tree_node_ass->tree_node_index;
    // Get numbers of objects would be split in the following
    objects_num = tree_node_ass->phase;
    // These objects stored in this node of stack will be split in the
    // following, and can not be split again,
    // so the link need to be cleared from this node of stack.
    tree_node_ass->objects.Clear();

    // Get tree node from this node of stack
    tree_node = &(tree_nodes_[tree_node_index]);

    // Compute AABB boundary of these objects.
    Boundary boundary;
    ComputeBoundary(objects, &boundary);
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << "=====>Boundary of all of the objects: min_x="
              << boundary.min_x
              << ", min_y=" << boundary.min_y
              << ", max_x=" << boundary.max_x
              << ", max_y=" << boundary.max_y
              << std::endl;
#endif

    // Compute partition information using boundary of these objects
    int_type partition = PARTITION_X;
    float_type partition_position = 0;
    ComputePartition(boundary, &partition, &partition_position);
#if ENABLE_AABBOXKDTREE2D_TRACE
    if (PARTITION_X == partition) {
      std::cout << "=====>Partition axis is X, partition position is "
                << partition_position << "." << std::endl;
    } else {
      std::cout << "=====>Partition axis is Y, partition position is "
                << partition_position << "." << std::endl;
    }
#endif

    // Decide if these objects need to be split using parameter from user
    if (SplitToSubNodes(tree_node->depth, objects_num, boundary)) {
      // Split thest objects to left node and right node respectively,
      // and leave objects intersecting with partition axis
      PartitionObjects(partition, partition_position,
                       &objects, &left_objects, &right_objects,
                       &objects_num, &left_objects_num, &right_objects_num);
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "After Partion: " << std::endl
                << "objects : " << std::endl
                << objects.DebugString(object_list_storage_)
                << "left_objects : " << std::endl
                << left_objects.DebugString(object_list_storage_)
                << "right_objects : " << std::endl
                << right_objects.DebugString(object_list_storage_)
                << std::endl;
#endif

      // Put objects intersecting with partition axis to current tree node
      // number of objects
      tree_node->num_objects = objects_num;
      // partition axis
      tree_node->partition = partition;
      tree_node->partition_position = partition_position;
      // boundary
      tree_node->boundary = boundary;
      // list stored reference of objects
      tree_node->objects_sorted_by_min = objects;
      TRAVERSE_CONST_DOUBLE_LIST_BEGIN(ListNode, DataNum,
                                       tree_node->objects_sorted_by_min,
                                       object_list_storage_)
          int_type list_index = -1;
          ListNode* list_node = object_list_storage_copy_.Allocate(&list_index);
          COM_CHECK(Nullptr_t != list_node);
          list_node->object_index = ACCESS_CONST_DOUBLE_LIST_NODE->object_index;
          tree_node->objects_sorted_by_max.PushBack(list_index,
                                                    object_list_storage_copy_);
      TRAVERSE_CONST_DOUBLE_LIST_END
      tree_node->objects_sorted_by_min.Sort(
            object_list_storage_,
            ObjCmpByMinFunc(partition, object_list_storage_, *this));
      tree_node->objects_sorted_by_max.Sort(
            object_list_storage_copy_,
            ObjCmpByMaxFunc(partition, object_list_storage_copy_, *this));
      // index of subnode
      tree_node->left_subnode = -1;
      tree_node->right_subnode = -1;
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "-------Number of objects spit into this node is "
                << objects_num << "." << std::endl;
#endif

      if (left_objects.Empty() && right_objects.Empty()) {
        // If no objects needed to split into the left or right node,
        // skip the current node
        tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "<------No objects needed to split into the "
                     "left or right node." << std::endl;
#endif
        continue;
      }

      // Split to sub-nodes.
      if (!right_objects.Empty()) {
        // Put the right objects to the right subnode of the current node
        TreeNode* new_tree_node = tree_nodes_.Allocate(&tree_node_index);
        COM_CHECK(Nullptr_t != new_tree_node);
        new_tree_node->Clear();
        // Update depth of the new node
        new_tree_node->depth = tree_node->depth + 1;

        AABBoxKDTreeNodeAssociation* new_tree_node_ass =
            tree_stack.Allocate();
        COM_CHECK(Nullptr_t != new_tree_node_ass);
        new_tree_node_ass->Clear();
        new_tree_node_ass->tree_node_index = tree_node_index;
        new_tree_node_ass->phase = right_objects_num;
        new_tree_node_ass->objects = right_objects;

        // Put the index of the new node to right subnode index
        // of the current node
        tree_node->right_subnode = tree_node_index;
      }
      if (!left_objects.Empty()) {
        // Put the left objects to the left subnode of the current node
        TreeNode* new_tree_node = tree_nodes_.Allocate(&tree_node_index);
        COM_CHECK(Nullptr_t != new_tree_node);
        new_tree_node->Clear();
        // Update depth of the new node
        new_tree_node->depth = tree_node->depth + 1;

        AABBoxKDTreeNodeAssociation* new_tree_node_ass =
            tree_stack.Allocate();
        COM_CHECK(Nullptr_t != new_tree_node_ass);
        new_tree_node_ass->Clear();
        new_tree_node_ass->tree_node_index = tree_node_index;
        new_tree_node_ass->phase = left_objects_num;
        new_tree_node_ass->objects = left_objects;

        // Put the index of the new node to left subnode index
        // of the current node
        tree_node->left_subnode = tree_node_index;
      }
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "<------Split into left or right node "
                   "(Number of left objects is "
                << left_objects_num << ", Number of right objects is "
                << right_objects_num << ")." << std::endl;
#endif
    } else {
      // Put all of these objects to current tree node

      // number of objects
      tree_node->num_objects = objects_num;
      // partition axis
      tree_node->partition = partition;
      tree_node->partition_position = partition_position;
      // boundary
      tree_node->boundary = boundary;
      // list stored reference of objects
      tree_node->objects_sorted_by_min = objects;
      TRAVERSE_CONST_DOUBLE_LIST_BEGIN(ListNode, DataNum,
                                       tree_node->objects_sorted_by_min,
                                       object_list_storage_)
          int_type list_index = -1;
          ListNode* list_node = object_list_storage_copy_.Allocate(&list_index);
          COM_CHECK(Nullptr_t != list_node);
          list_node->object_index = ACCESS_CONST_DOUBLE_LIST_NODE->object_index;
          tree_node->objects_sorted_by_max.PushBack(list_index,
                                                    object_list_storage_copy_);
      TRAVERSE_CONST_DOUBLE_LIST_END
      tree_node->objects_sorted_by_min.Sort(
            object_list_storage_,
            ObjCmpByMinFunc(partition, object_list_storage_, *this));
      tree_node->objects_sorted_by_max.Sort(
            object_list_storage_copy_,
            ObjCmpByMaxFunc(partition, object_list_storage_copy_, *this));
      // index referenced to subnode
      tree_node->left_subnode = -1;
      tree_node->right_subnode = -1;

      // This tree node don't need to be split subsequentially.
      tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "<------Reach bottom of tree (Numbers of objects is "
                << objects_num << "), and go back." << std::endl;
#endif
    }
  }

#if ENABLE_AABBOXKDTREE2D_TRACE
  std::cout << std::endl << "After partition, number of tree nodes is "
            << tree_nodes_.Size() << std::endl;
  std::cout << "<############### Partition #################" << std::endl;
#endif
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
template <typename CalcSquareDistToPt, Int32_t MaxObjectsNum,
          Int32_t MaxStackNodesNum>
void AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::FindObjects(
    StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack,
    const Vec2d &point,
    const float_type distance,
    const CalcSquareDistToPt& calc_sq_dist_to_pt,
    StaticVector<int_type, MaxObjectsNum>* result_objs,
    bool always_check_dist) const {
  result_objs->Clear();
  if (tree_nodes_.Empty()) {
    return;
  }

#if ENABLE_AABBOXKDTREE2D_TRACE
  std::cout << "################ FindObjects ################>" << std::endl;
  std::cout << "Find objects in the range of circle which center point is ["
            << point.x() << ", " << point.y()
            << "] and radius is " << distance << "." << std::endl;
#endif

  float_type distance_sqr = Square(distance);

  // Push the root node of tree to stack
  tree_stack.Clear();
  AABBoxKDTreeNodeAssociation* tree_node_ass = tree_stack.Allocate();
  COM_CHECK(Nullptr_t != tree_node_ass);
  tree_node_ass->Clear();
  tree_node_ass->tree_node_index = 0;
  tree_node_ass->phase = 0;

#if ENABLE_AABBOXKDTREE2D_TRACE
  int_type recursion_count = 0;
#endif
  while (!(tree_stack.Empty())) {
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << std::endl << "@@@@@>recursion_count="
              << recursion_count++ << std::endl;
    //if (recursion_count > 20) {
    //  std::cout << "ERROR: The number of recursion is out of range, "
    //               "and quit the function." << std::endl;
    //  break;
    //}
#endif
    // Get top node of the stack which stored associations of tree nodes
    tree_node_ass = &(tree_stack.Back());
    // Get tree node from this node of stack
    const TreeNode* tree_node = &(tree_nodes_[tree_node_ass->tree_node_index]);

    float_type lower_dist_sqr_to_p =
        LowerDistanceSquareToPoint(point, tree_node->boundary);
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << "Tree node index is " << tree_node_ass->tree_node_index
              << "." << std::endl;
    std::cout << "lower_dist_sqr_to_p=" << lower_dist_sqr_to_p << std::endl;
    std::cout << "The depth of tree node is "
              << tree_node->depth << "." << std::endl;
    std::cout << "The number of objects in this node is "
              << tree_node->num_objects << "." << std::endl;
#endif
    if (lower_dist_sqr_to_p >= distance_sqr) {
      tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "<-----The bounding box of this node is outside of "
                   "the required range, and go back."
                << std::endl;
#endif
      continue;
    }

    float_type upper_dist_sqr_to_p =
        UpperDistanceSquareToPoint(point, tree_node->boundary);

    if (0 == tree_node_ass->phase) {
      // Phase I: Get all of objects which are in the required range
      // from this node.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "=====>Phase I: Get all of the objects which are"
                   " in the requied range." << std::endl;
#endif
      tree_node_ass->phase = 1;

      if (upper_dist_sqr_to_p <= distance_sqr) {
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "------All of the objects in this node are fully in the"
                     " requied range." << std::endl;
#endif
        internal::DoubleList::const_iterator<ListNode, DataNum> it =
            tree_node->objects_sorted_by_min.cbegin(object_list_storage_);
        internal::DoubleList::const_iterator<ListNode, DataNum> it_end =
            tree_node->objects_sorted_by_min.cend(object_list_storage_);
        for (; it != it_end; ++it) {
#if ENABLE_AABBOXKDTREE2D_TRACE
          std::cout << "Get obj index " << it->object_index << std::endl;
#endif
          if (always_check_dist) {
            const Object& obj = objects_[it->object_index];
            float_type dist_sqr = calc_sq_dist_to_pt(obj.user_data,
                                                     it->object_index);
            if (dist_sqr > distance_sqr) {
              continue;
            }
          }
          int_type* obj_index = result_objs->Allocate();
          if (Nullptr_t != obj_index) {
            *obj_index = it->object_index;
          } else {
#if ENABLE_AABBOXKDTREE2D_TRACE
            std::cout << "WARNING: The number of objects exceed the required."
                      << std::endl;
#endif
            break;
          }
        }
      } else {
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "------Select objects which are in "
                     "the required range from this node." << std::endl;
#endif
        const float_type pvalue = (tree_node->partition == PARTITION_X ?
                               point.x() : point.y());

        if (pvalue < tree_node->partition_position) {
          const float_type limit = pvalue + distance;
          internal::DoubleList::const_iterator<ListNode, DataNum> it =
              tree_node->objects_sorted_by_min.cbegin(object_list_storage_);
          internal::DoubleList::const_iterator<ListNode, DataNum> it_end =
              tree_node->objects_sorted_by_min.cend(object_list_storage_);
          for (; it != it_end; ++it) {
            const Object& obj = objects_[it->object_index];
            float_type bound = tree_node->partition == PARTITION_X ?
                  obj.box.min_x() : obj.box.min_y();
            if (bound > limit) {
              break;
            }
            float_type dist_sqr = calc_sq_dist_to_pt(obj.user_data,
                                                     it->object_index);
            if (dist_sqr <= distance_sqr) {
#if ENABLE_AABBOXKDTREE2D_TRACE
              std::cout << "Get obj index " << it->object_index << std::endl;
#endif
              int_type* obj_index = result_objs->Allocate();
              if (Nullptr_t != obj_index) {
                *obj_index = it->object_index;
              } else {
#if ENABLE_AABBOXKDTREE2D_TRACE
                std::cout << "WARNING: The number of objects exceed the required."
                          << std::endl;
#endif
                break;
              }
            }
          }
        } else {
          const float_type limit = pvalue - distance;
          internal::DoubleList::const_iterator<ListNode, DataNum> it =
              tree_node->objects_sorted_by_max.cbegin(object_list_storage_copy_);
          internal::DoubleList::const_iterator<ListNode, DataNum> it_end =
              tree_node->objects_sorted_by_max.cend(object_list_storage_copy_);
          for (; it != it_end; ++it) {
            const Object& obj = objects_[it->object_index];
            float_type bound = tree_node->partition == PARTITION_X ?
                  obj.box.max_x() : obj.box.max_y();
            if (bound < limit) {
              break;
            }
            float_type dist_sqr = calc_sq_dist_to_pt(obj.user_data,
                                                         it->object_index);
            if (dist_sqr <= distance_sqr) {
#if ENABLE_AABBOXKDTREE2D_TRACE
              std::cout << "Get obj index " << it->object_index << std::endl;
#endif
              int_type* obj_index = result_objs->Allocate();
              if (Nullptr_t != obj_index) {
                *obj_index = it->object_index;
              } else {
#if ENABLE_AABBOXKDTREE2D_TRACE
                std::cout << "WARNING: The number of objects exceed the required."
                          << std::endl;
#endif
                break;
              }
            }
          }
        }
      }
    }

    if (1 == tree_node_ass->phase) {
      // Phase II: Find objects from left subnode of this node.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "=====>Phase II: Find objects from left subnode "
                   "of this node." << std::endl;
#endif

      tree_node_ass->phase = 2;

      if (tree_node->left_subnode >= 0) {
        // Go to access the left subnode

        // Push left node to the stack.
        tree_node_ass = tree_stack.Allocate();
        COM_CHECK(Nullptr_t != tree_node_ass);
        tree_node_ass->Clear();
        tree_node_ass->tree_node_index = tree_node->left_subnode;
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "<-----Access the left subnode, "
                     "the index of the subnode is "
                  << tree_node->left_subnode << "." << std::endl;
#endif
        continue;
      }
      // if (tree_node->right_subnode >= 0) {
      //   // Go to access the right subnode
      //
      //   // Push right node to the stack.
      //   tree_node_ass = tree_stack.Allocate();
      //   COM_CHECK(Nullptr_t != tree_node_ass);
      //   tree_node_ass->Clear();
      //   tree_node_ass->tree_node_index = tree_node->right_subnode;
      // #if ENABLE_AABBOXKDTREE2D_TRACE
      //   std::cout << "<-----Access the right subnode"
      //                ", the index of the subnode is "
      //             << tree_node->right_subnode << "." << std::endl;
      // #endif
      //   continue;
      // }
    }

    if (2 == tree_node_ass->phase) {
      // Phase III: Find objects from right subnode of this node.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "=====>Phase III: Find objects from right subnode "
                   "of this node." << std::endl;
#endif

      tree_node_ass->phase = 3;

      if (tree_node->right_subnode >= 0) {
        // Go to access the right subnode

        // Push right node to the stack.
        tree_node_ass = tree_stack.Allocate();
        COM_CHECK(Nullptr_t != tree_node_ass);
        tree_node_ass->Clear();
        tree_node_ass->tree_node_index = tree_node->right_subnode;
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "<-----Access the right subnode"
                     ", the index of the subnode is "
                  << tree_node->right_subnode << "." << std::endl;
#endif
        continue;
      }
    }

    tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << "<-----Complete handling this node, and go back." << std::endl;
#endif
  }

#if ENABLE_AABBOXKDTREE2D_TRACE
  std::cout << "<############### FindObjects #################" << std::endl;
#endif
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
template <typename CalcSquareDistToPt, Int32_t MaxStackNodesNum>
const DataType* AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::FindNearest(
    StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack,
    const Vec2d &point,
    const CalcSquareDistToPt& calc_sq_dist_to_pt,
    int_type* nearest_obj_index,
    float_type* min_distance_sqr) const {
  const DataType* nearest_user_obj = Nullptr_t;
  *nearest_obj_index = -1;
  *min_distance_sqr = common::NumLimits<float_type>::max();
  if (tree_nodes_.Empty()) {
    return (nearest_user_obj);
  }
#if ENABLE_AABBOXKDTREE2D_TRACE
  std::cout << "################ FindNearest ################>" << std::endl;
#endif

  // Push the root node of tree to stack
  tree_stack.Clear();
  AABBoxKDTreeNodeAssociation* tree_node_ass = tree_stack.Allocate();
  COM_CHECK(Nullptr_t != tree_node_ass);
  tree_node_ass->Clear();
  tree_node_ass->tree_node_index = 0;
  tree_node_ass->phase = 0;

#if ENABLE_AABBOXKDTREE2D_TRACE
  int_type recursion_count = 0;
#endif
  while (!(tree_stack.Empty())) {
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << std::endl << "@@@@@>recursion_count="
              << recursion_count++ << std::endl;
    //if (recursion_count > 20) {
    //  std::cout << "ERROR: The number of recursion is out of range, "
    //               "and quit the function." << std::endl;
    //  break;
    //}
#endif

    // Get top node of the stack which stored associations of tree nodes
    tree_node_ass = &(tree_stack.Back());
    // Get tree node from this node of stack
    const TreeNode* tree_node = &(tree_nodes_[tree_node_ass->tree_node_index]);

    float lower_dist_sqr_to_p =
        LowerDistanceSquareToPoint(point, tree_node->boundary);
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << "Tree node index is " << tree_node_ass->tree_node_index
              << "." << std::endl;
    std::cout << "min_distance_sqr=" << *min_distance_sqr
              << ", lower_dist_sqr_to_p=" << lower_dist_sqr_to_p << std::endl;
    std::cout << "The depth of tree node is "
              << tree_node->depth << "." << std::endl;
    std::cout << "The number of objects in this node is "
              << tree_node->num_objects << "." << std::endl;
    std::cout << "Objects stored in this node are: ";
    TRAVERSE_CONST_DOUBLE_LIST_BEGIN(ListNode, DataNum,
                                     tree_node->objects_sorted_by_min,
                                     object_list_storage_)
        std::cout << ACCESS_CONST_DOUBLE_LIST_NODE->object_index << ", ";
    TRAVERSE_CONST_DOUBLE_LIST_END
    std::cout << std::endl;
#endif

    if (lower_dist_sqr_to_p >= (*min_distance_sqr - kGeometryEpsilon)) {
      tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "<-----The distance of this bounding box to the point "
                   "is greater than min distance, and go back." << std::endl;
#endif
      continue;
    }

    const float_type pvalue = (tree_node->partition == PARTITION_X ?
                                 point.x() : point.y());
    const bool search_left_first = (pvalue < tree_node->partition_position);
#if ENABLE_AABBOXKDTREE2D_TRACE
    if (PARTITION_X == tree_node->partition) {
      std::cout << "Partition axis is X"
                << ", partition_position=" << tree_node->partition_position
                << ", pvalue=" << pvalue
                << ", search_left_first=" << search_left_first
                << "." << std::endl;
    } else {
      std::cout << "Partition axis is Y"
                << ", partition_position=" << tree_node->partition_position
                << ", pvalue=" << pvalue
                << ", search_left_first=" << search_left_first
                << "." << std::endl;
    }
#endif

    if (0 == tree_node_ass->phase) {
      // Phase I: Find the nearest and lowest node of the tree.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "=====>Phase I: Find the nearest and lowest node "
                   "of the tree." << std::endl;
#endif
      tree_node_ass->phase = 1;

      if (search_left_first) {
        if (tree_node->left_subnode >= 0) {
          // Go to access the left subnode

          // Push left node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->left_subnode;
#if ENABLE_AABBOXKDTREE2D_TRACE
          std::cout << "<-----Access the left subnode, "
                       "the index of the subnode is "
                    << tree_node->left_subnode << "." << std::endl;
#endif
          continue;
        }
      } else {
        if (tree_node->right_subnode >= 0) {
          // Go to access the right subnode

          // Push right node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->right_subnode;
#if ENABLE_AABBOXKDTREE2D_TRACE
          std::cout << "<-----Access the right subnode"
                       ", the index of the subnode is "
                    << tree_node->right_subnode << "." << std::endl;
#endif
          continue;
        }
      }
    }

    if (1 == tree_node_ass->phase) {
      // Phase II: Calculate the min distance of ojects of this node to the point.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "=====>Phase II: Calculate the min distance of ojects of"
                   " this node to the point." << std::endl;
#endif
      tree_node_ass->phase = 2;

      if (*min_distance_sqr <= kGeometryEpsilon) {
        tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "<-----The min distance of objects to the point is zero,"
                     " and go back." << std::endl;
#endif
        continue;
      }

      if (search_left_first) {
        internal::DoubleList::const_iterator<ListNode, DataNum> it =
            tree_node->objects_sorted_by_min.cbegin(object_list_storage_);
        internal::DoubleList::const_iterator<ListNode, DataNum> it_end =
            tree_node->objects_sorted_by_min.cend(object_list_storage_);
        for (; it != it_end; ++it) {
          const Object& obj = objects_[it->object_index];
          float_type bound = tree_node->partition == PARTITION_X ?
                obj.box.min_x() : obj.box.min_y();
          if ((bound > pvalue) && (Square(bound - pvalue) > *min_distance_sqr)) {
            break;
          }
          float_type distance_sqr = calc_sq_dist_to_pt(obj.user_data,
                                                       it->object_index);
          if (distance_sqr < *min_distance_sqr) {
            *min_distance_sqr = distance_sqr;
            *nearest_obj_index = it->object_index;
            nearest_user_obj = &(obj.user_data);
          }
        }
      } else {
        internal::DoubleList::const_iterator<ListNode, DataNum> it =
            tree_node->objects_sorted_by_max.cbegin(object_list_storage_copy_);
        internal::DoubleList::const_iterator<ListNode, DataNum> it_end =
            tree_node->objects_sorted_by_max.cend(object_list_storage_copy_);
        for (; it != it_end; ++it) {
          const Object& obj = objects_[it->object_index];
          float_type bound = tree_node->partition == PARTITION_X ?
                obj.box.max_x() : obj.box.max_y();
          if ((bound < pvalue) && (Square(bound - pvalue) > *min_distance_sqr)) {
            break;
          }
          float_type distance_sqr = calc_sq_dist_to_pt(obj.user_data,
                                                       it->object_index);
          if (distance_sqr < *min_distance_sqr) {
            *min_distance_sqr = distance_sqr;
            *nearest_obj_index = it->object_index;
            nearest_user_obj = &(obj.user_data);
          }
        }
      }

      if (*min_distance_sqr <= kGeometryEpsilon) {
        tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
        std::cout << "<-----After calculating the min distance of objects "
                     "of this node to the point, the min distance is zero,"
                     " and go back." << std::endl;
#endif
        continue;
      }
    }

    if (2 == tree_node_ass->phase) {
      // Phase III: Look at the other neighbor of this node.
#if ENABLE_AABBOXKDTREE2D_TRACE
      std::cout << "=====>Phase III: Look at the other neighbor of this node."
                << std::endl;
#endif
      tree_node_ass->phase = 3;

      if (search_left_first) {
        if (tree_node->right_subnode >= 0) {
          // Go to access the right subnode

          // Push right node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->right_subnode;
#if ENABLE_AABBOXKDTREE2D_TRACE
          std::cout << "<-----Access the right subnode"
                       ", the index of the subnode is "
                    << tree_node->right_subnode << "." << std::endl;
#endif

          continue;
        }
      } else {
        if (tree_node->left_subnode >= 0) {
          // Go to access the left subnode

          // Push left node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->left_subnode;
#if ENABLE_AABBOXKDTREE2D_TRACE
          std::cout << "<-----Access the left subnode, "
                       "the index of the subnode is "
                    << tree_node->left_subnode << "." << std::endl;
#endif
          continue;
        }
      }
    }

    tree_stack.PopBack();
#if ENABLE_AABBOXKDTREE2D_TRACE
    std::cout << "<-----Complete handling this node, and go back." << std::endl;
#endif
  }

#if ENABLE_AABBOXKDTREE2D_TRACE
  std::cout << "<############### FindNearest #################" << std::endl;
#endif

  return (nearest_user_obj);
}


template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
void AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::ComputeBoundary(
    const internal::DoubleList& objs, Boundary* boundary) const {
  if (objs.Empty()) {
    return;
  }

  const ListNode* front = objs.Front(object_list_storage_);
  boundary->min_x = objects_[front->object_index].box.min_x();
  boundary->min_y = objects_[front->object_index].box.min_y();
  boundary->max_x = objects_[front->object_index].box.max_x();
  boundary->max_y = objects_[front->object_index].box.max_y();

  internal::DoubleList::const_iterator<ListNode, DataNum> list_it =
      objs.cbegin(object_list_storage_);
  internal::DoubleList::const_iterator<ListNode, DataNum> list_end =
      objs.cend(object_list_storage_);
  for (++list_it; list_it != list_end; ++list_it) {
    int_type object_index = list_it->object_index;

    if (objects_[object_index].box.min_x() < boundary->min_x) {
      boundary->min_x = objects_[object_index].box.min_x();
    }
    if (objects_[object_index].box.min_y() < boundary->min_y) {
      boundary->min_y = objects_[object_index].box.min_y();
    }

    if (objects_[object_index].box.max_x() > boundary->max_x) {
      boundary->max_x = objects_[object_index].box.max_x();
    }
    if (objects_[object_index].box.max_y() > boundary->max_y) {
      boundary->max_y = objects_[object_index].box.max_y();
    }
  }

  boundary->mid_x = 0.5F * (boundary->min_x + boundary->max_x);
  boundary->mid_y = 0.5F * (boundary->min_y + boundary->max_y);
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
void AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::ComputePartition(
    const Boundary& boundary,
    int_type* partition,
    float_type* partition_position) const {
  if (boundary.max_x - boundary.min_x >= boundary.max_y - boundary.min_y) {
    *partition = PARTITION_X;
    *partition_position = 0.5F * (boundary.min_x + boundary.max_x);
  } else {
    *partition = PARTITION_Y;
    *partition_position = 0.5F * (boundary.min_y + boundary.max_y);
  }
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
bool AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::SplitToSubNodes(
    const int_type depth, const int_type obj_num, const Boundary& boundary) {
  if ((params_.max_depth >= 0) && (depth >= params_.max_depth)) {
    return false;
  }
  if (obj_num <= params_.max_leaf_size) {
    return false;
  }
  if ((params_.max_leaf_dimension >= 0.0F) &&
      (Max(boundary.max_x - boundary.min_x,
                boundary.max_y - boundary.min_y) <=
       params_.max_leaf_dimension)) {
    return false;
  }
  return true;
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
void AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::PartitionObjects(
    const int_type partition,
    const float_type partition_position,
    internal::DoubleList* objs,
    internal::DoubleList* left_objs,
    internal::DoubleList* right_objs,
    int_type* objs_num,
    int_type* left_objs_num,
    int_type* right_objs_num) {
  left_objs->Clear();
  right_objs->Clear();

  *objs_num = 0;
  *left_objs_num = 0;
  *right_objs_num = 0;

  if (objs->Empty()) {
    return;
  }

  if (PARTITION_X == partition) {
    internal::DoubleList::iterator<ListNode, DataNum> list_it =
        objs->begin(object_list_storage_);
    for (; list_it != objs->end(object_list_storage_);) {
      int_type obj_index = list_it->object_index;
      const Object& object = objects_[obj_index];

      if (object.box.max_x() <= partition_position) {
        left_objs->PushBack(list_it.remove(true), object_list_storage_);

        (*left_objs_num)++;
      } else if (object.box.min_x() >= partition_position) {
        right_objs->PushBack(list_it.remove(true), object_list_storage_);

        (*right_objs_num)++;
      } else {
        ++list_it;

        (*objs_num)++;
      }
    }
  } else {
    internal::DoubleList::iterator<ListNode, DataNum> list_it =
        objs->begin(object_list_storage_);
    for (; list_it != objs->end(object_list_storage_);) {
      int_type obj_index = list_it->object_index;
      const Object& object = objects_[obj_index];

      if (object.box.max_y() <= partition_position) {
        left_objs->PushBack(list_it.remove(true), object_list_storage_);

        (*left_objs_num)++;
      } else if (object.box.min_y() >= partition_position) {
        right_objs->PushBack(list_it.remove(true), object_list_storage_);

        (*right_objs_num)++;
      } else {
        ++list_it;

        (*objs_num)++;
      }
    }
  }
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
typename AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::float_type
AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::LowerDistanceSquareToPoint(
    const Vec2d &point, const Boundary& boundary) const {
  float_type dx = 0;
  if (point.x() < boundary.min_x) {
    dx = boundary.min_x - point.x();
  } else if (point.x() > boundary.max_x) {
    dx = point.x() - boundary.max_x;
  }

  float_type dy = 0;
  if (point.y() < boundary.min_y) {
    dy = boundary.min_y - point.y();
  } else if (point.y() > boundary.max_y) {
    dy = point.y() - boundary.max_y;
  }

  return (dx * dx + dy * dy);
}

template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
typename AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::float_type
AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::UpperDistanceSquareToPoint(
    const Vec2d &point, const Boundary& boundary) const {
  float_type dx =
      (point.x() > boundary.mid_x ?
         (point.x() - boundary.min_x) : (point.x() - boundary.max_x));
  float_type dy =
      (point.y() > boundary.mid_y ?
         (point.y() - boundary.min_y) : (point.y() - boundary.max_y));

  return (dx * dx + dy * dy);
}

#if ENABLE_AABBOXKDTREE2D_TRACE
template <typename DataType, Int32_t DataNum, Int32_t MaxLeafSize>
template <Int32_t MaxStackNodesNum>
std::string AABBoxKDTree2d<DataType, DataNum, MaxLeafSize>::DebugString(
    StaticVector<AABBoxKDTreeNodeAssociation, MaxStackNodesNum>& tree_stack) {
  // The order of access to this tree is from top-down
  // and subsequently from left-right

  //std::ostringstream os;

  std::cout << "################ AABBoxKDTree2d ################>" << std::endl;

  std::cout << "The total number of objects stored in this kd-tree is "
     << objects_.Size() << "." << std::endl;
  std::cout << "The total number of this kd-tree nodes is "
     << tree_nodes_.Size() << "." << std::endl;

  if (!tree_nodes_.Empty()) {
    tree_stack.Clear();
    AABBoxKDTreeNodeAssociation* tree_node_ass = tree_stack.Allocate();
    COM_CHECK(Nullptr_t != tree_node_ass);
    tree_node_ass->Clear();
    tree_node_ass->tree_node_index = 0;
    tree_node_ass->phase = 0;

    int_type recursion_count = 0;
    while (!(tree_stack.Empty())) {

      std::cout << std::endl << "@@@@@>recursion_count="
                << recursion_count++ << std::endl;

      // Get top node of the stack which stored associations of tree nodes
      tree_node_ass = &(tree_stack.Back());
      // Get tree node from this node of stack
      TreeNode* tree_node = &(tree_nodes_[tree_node_ass->tree_node_index]);

      std::cout << "Tree node index is " << tree_node_ass->tree_node_index
                << "." << std::endl;
      std::cout << "The number of access to this node is "
                << tree_node_ass->phase << "." << std::endl;

      if (0 == tree_node_ass->phase) {

        std::cout << "The first access." << std::endl;
        std::cout << "====>The depth of tree node is "
                  << tree_node->depth << "." << std::endl;
        std::cout << "====>The number of objects in this node is "
                  << tree_node->num_objects << "." << std::endl;

        std::cout << "====>Objects stored in this node by min are: ";
        TRAVERSE_CONST_DOUBLE_LIST_BEGIN(ListNode, DataNum,
                                         tree_node->objects_sorted_by_min,
                                         object_list_storage_)
            std::cout << ACCESS_CONST_DOUBLE_LIST_NODE->object_index << ", ";
        TRAVERSE_CONST_DOUBLE_LIST_END
        std::cout << std::endl;
        std::cout << "====>Objects stored in this node by max are: ";
        TRAVERSE_CONST_DOUBLE_LIST_BEGIN(ListNode, DataNum,
                                         tree_node->objects_sorted_by_max,
                                         object_list_storage_copy_)
            std::cout << ACCESS_CONST_DOUBLE_LIST_NODE->object_index << ", ";
        TRAVERSE_CONST_DOUBLE_LIST_END
        std::cout << std::endl;

        std::cout << "====>Boundary x[" << tree_node->boundary.min_x
                  << ", " << tree_node->boundary.max_x
                  << "]  y[" << tree_node->boundary.min_y
                  << ", " << tree_node->boundary.max_y
                  << "]" << std::endl;

        if (tree_node->left_subnode >= 0) {
          // Go to access the left subnode

          // Update the number of access to this node.
          tree_node_ass->phase = 1;

          // Push left node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->left_subnode;

          std::cout << "<-----Access the left subnode, "
                       "the index of the subnode is "
                    << tree_node->left_subnode << "." << std::endl;

        } else if (tree_node->right_subnode >= 0) {
          // This node don't have left subnode, and go to access the right subnode

          // Update the number of access to this node.
          tree_node_ass->phase = 2;

          // Push right node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->right_subnode;

          std::cout << "<-----This node have not left subnode, "
                       "go to access the right subnode"
                       ", the index of the subnode is "
                    << tree_node->right_subnode << "." << std::endl;
        } else {
          // This node is one of bottom nodes of the tree, and go back.
          tree_stack.PopBack();

          std::cout << "<-----This node is one of bottom nodes of the tree."
                    << std::endl;
        }
      } else if (1 == tree_node_ass->phase) {

        std::cout << "The second access." << std::endl;

        if (tree_node->right_subnode >= 0) {
          // Update the number of access to this node.
          tree_node_ass->phase = 2;

          // Push right node to the stack.
          tree_node_ass = tree_stack.Allocate();
          COM_CHECK(Nullptr_t != tree_node_ass);
          tree_node_ass->Clear();
          tree_node_ass->tree_node_index = tree_node->right_subnode;

          std::cout << "<-----Access the right subnode, "
                       "the index of the subnode is "
                    << tree_node->right_subnode << "." << std::endl;
        } else {
          // This node is one of bottom nodes of the tree, and go back.
          tree_stack.PopBack();

          std::cout << "<-----This node is one of bottom nodes of the tree."
                    << std::endl;
        }
      } else {
        tree_node_ass->phase++;

        std::cout << "The " << tree_node_ass->phase << "th access."
                  << std::endl;

        // All of the subnodes of this node had been accessed before,
        // and go back.
        tree_stack.PopBack();

        std::cout << "<-----All of the subnode of this node had been "
                     "accessed before, and go back." << std::endl;
      }
    }
  }

  std::cout << "<################ AABBoxKDTree2d ################" << std::endl;

  return ("");
}
#endif


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_AABBOXKDTREE2D_H_
