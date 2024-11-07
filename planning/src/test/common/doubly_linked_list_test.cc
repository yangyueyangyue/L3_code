//
#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif

#include "container/doubly_linked_list.h"

#if (ENABLE_GTEST)

namespace phoenix {
namespace common {


TEST(DoublyLinkedList, Case_01) {
  enum { DATA_NUM = 10 };

  DataPool<DoublyLinkedListNode<Int32_t>, DATA_NUM> pool;
  DoublyLinkedList<Int32_t, DATA_NUM> list;
  DoublyLinkedList<Int32_t, DATA_NUM>::iterator it(list, pool);
  DoublyLinkedList<Int32_t, DATA_NUM>::const_iterator cit(list, pool);
  Int32_t index = 0;

  std::cout << list.DebugString(pool) << std::endl;

  Int32_t* data = list.AddToBack(pool);
  *data = 0;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index);
  }
  std::cout << list.DebugString(pool) << std::endl;

  data = list.AddToBack(pool);
  *data = 1;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index);
  }
  std::cout << list.DebugString(pool) << std::endl;

  data = list.AddToBack(pool);
  *data = 2;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index);
  }
  std::cout << list.DebugString(pool) << std::endl;

  list.Clear(pool);

  std::cout << list.DebugString(pool) << std::endl;

  data = list.AddToBack(pool);
  *data = 0;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index);
  }
  std::cout << list.DebugString(pool) << std::endl;

  list.PushBack(1, pool);
  list.PushBack(2, pool);
  list.PushBack(3, pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index);
  }
  EXPECT_EQ(index, 4);
  std::cout << "index=" << index << std::endl;
  std::cout << list.DebugString(pool) << std::endl;

  std::cout << "data of list head = " << *list.Front(pool) << std::endl;
  std::cout << "data of list tail = " << *list.Back(pool) << std::endl;
  EXPECT_EQ(*list.Front(pool), 0);
  EXPECT_EQ(*list.Back(pool), 3);

  std::cout << "After erase front data:" << std::endl;
  list.Erase(list.Front(pool), pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index+1);
  }
  EXPECT_EQ(index, 3);
  std::cout << "index=" << index << std::endl;
  std::cout << list.DebugString(pool) << std::endl;

  std::cout << "After erase back data:" << std::endl;
  list.Erase(list.Back(pool), pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index+1);
  }
  EXPECT_EQ(index, 2);
  std::cout << "index=" << index << std::endl;
  std::cout << list.DebugString(pool) << std::endl;

  list.PushBack(3, pool);
  list.PushBack(4, pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    if (1 == index) {
      list.Erase(it.current(), pool);
      break;
    }
  }
  std::cout << "After erase the second data:" << std::endl;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    if (index < 1) {
      EXPECT_EQ(*it, index+1);
    } else {
      EXPECT_EQ(*it, index+2);
    }
  }
  EXPECT_EQ(index, 3);
  std::cout << "index=" << index << std::endl;

  std::cout << "Clear list and push data:" << std::endl;
  list.Clear(pool);
  list.PushBack(0, pool);
  list.PushBack(1, pool);
  list.PushBack(2, pool);
  list.PushBack(3, pool);
  list.PushBack(4, pool);
  list.PushBack(5, pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;
  }
  std::cout << list.DebugString(pool) << std::endl;


  for (it = list.begin(pool), index = 0; it != list.end(pool); ++index) {
    if (0 == index) {
      std::cout << "Erase data[" << index << "]=" << *it
                << " from list." << std::endl;
      it = it.erase();
    } else {
      std::cout << "data[" << index << "] = " << *it << std::endl;
      ++it;
    }
  }
  std::cout << "After erase the first data:" << std::endl;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index+1);
  }
  EXPECT_EQ(index, 5);
  std::cout << "index=" << index << std::endl;

  for (it = list.begin(pool), index = 0; it != list.end(pool); ++index) {
    if (4 == index) {
      std::cout << "Erase data[" << index << "]=" << *it
                << " from list." << std::endl;
      it = it.erase();
    } else {
      std::cout << "data[" << index << "] = " << *it << std::endl;
      ++it;
    }
  }
  std::cout << "After erase the last data:" << std::endl;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index+1);
  }
  EXPECT_EQ(index, 4);
  std::cout << "index=" << index << std::endl;

  for (it = list.begin(pool), index = 0; it != list.end(pool); ++index) {
    if (2 == index) {
      std::cout << "Erase data[" << index << "]=" << *it
                << " from list." << std::endl;
      it = it.erase();
    } else {
      std::cout << "data[" << index << "] = " << *it << std::endl;
      ++it;
    }
  }
  std::cout << "After erase the third data:" << std::endl;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    if (index < 2) {
      EXPECT_EQ(*it, index+1);
    } else {
      EXPECT_EQ(*it, index+2);
    }
  }
  EXPECT_EQ(index, 3);
  std::cout << "index=" << index << std::endl;


  std::cout << "Clear list and push data:" << std::endl;
  list.Clear(pool);
  list.PushBack(5, pool);
  list.PushBack(4, pool);
  list.PushBack(3, pool);
  list.PushBack(2, pool);
  list.PushBack(1, pool);
  list.PushBack(0, pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;
  }
  std::cout << list.DebugString(pool) << std::endl;
  std::cout << "After sort:" << std::endl;
  class CmpFunc {
  public:
    bool operator ()(Int32_t data1, Int32_t data2) const {
      return (data1 < data2);
    }
  };
  list.Sort(pool, CmpFunc());
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;

    EXPECT_EQ(*it, index);
  }
  EXPECT_EQ(index, 6);
  std::cout << "index=" << index << std::endl;
  std::cout << list.DebugString(pool) << std::endl;

  data = list.AddToFront(pool);
  *data = 6;
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;
  }
  EXPECT_EQ(*list.Front(pool), 6);
  EXPECT_EQ(index, 7);
  std::cout << "index=" << index << std::endl;

  list.PushFront(7, pool);
  for (it = list.begin(pool), index = 0; it != list.end(pool); ++it, ++index) {
    std::cout << "data[" << index << "] = " << *it << std::endl;
  }
  EXPECT_EQ(*list.Front(pool), 7);
  EXPECT_EQ(index, 8);
  std::cout << "index=" << index << std::endl;

  list.PushFront(8, pool);
  list.PushFront(9, pool);
  list.PushFront(10, pool);
  for (cit = list.cbegin(pool), index = 0; cit != list.cend(pool);
       ++cit, ++index) {
    std::cout << "data[" << index << "] = " << *cit << std::endl;
  }
  list.Sort(pool, CmpFunc());
  for (cit = list.cbegin(pool), index = 0; cit != list.cend(pool);
       ++cit, ++index) {
    std::cout << "data[" << index << "] = " << *cit << std::endl;

    EXPECT_EQ(*cit, index);
  }
  EXPECT_EQ(index, 10);
  std::cout << "index=" << index << std::endl;
}


}  // namespace common
}  // namespace phoenix

#endif


