#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif
#include "container/unordered_map.h"


#if (ENABLE_GTEST)

namespace phoenix {
namespace driv_map {


TEST(UNORDERED_MAP, CASE1) {
  // HDMap
  common::UnorderedMap<Int32_t, Int32_t, 5> map_test;
  map_test.Clear();
  Int32_t key = 1;
  Int32_t* m1  =  map_test.Allocate(key);
  *m1 = 99;

  key = 11;
  Int32_t* m2= map_test.Allocate(key);
  *m2 = 98;

  key = 21;
  Int32_t* m3= map_test.Allocate(key);
  *m3 = 97;

  key = 100;
  Int32_t* m4= map_test.Allocate(key);
  *m4 = 96;

  //100次测试
//  for (int i = 0; i <= 100; ++i) {
//   *map_test.Allocate(i) = i;
//    map_test.Erase(i);
//  }

  key = 11;
  EXPECT_EQ(*map_test.Find(key),98);
  key = 21;
  EXPECT_EQ(*map_test.Find(key),97);
}

TEST(UNORDERED_MAP, CASE2) {
  // HDMap
  common::UnorderedMap<Int32_t, Int32_t, 5> map_test;
  map_test.Clear();
  Int32_t key = 32;
  Int32_t* m1  =  map_test.Allocate(key);
  *m1 = 0;

  key = 45;
  Int32_t* m2= map_test.Allocate(key);
  *m2 = 1;

  key = 56;
  Int32_t* m3= map_test.Allocate(key);
  *m3 = 4;

  key = 67;
  Int32_t* m4= map_test.Allocate(key);
  *m4 =8;

  key = 32;
  EXPECT_EQ(*map_test.Find(key),0);
  key = 67;
  EXPECT_EQ(*map_test.Find(key),8);
}

TEST(UNORDERED_MAP, CASE3) {
  // HDMap
  common::UnorderedMap<Int32_t, Int32_t, 5> map_test;
  map_test.Clear();
  Int32_t key = 32;
  Int32_t* m1  =  map_test.Allocate(key);
  *m1 =100;

  key = 45;
  Int32_t* m2= map_test.Allocate(key);
  *m2 = 1;

  key = 56;
  Int32_t* m3= map_test.Allocate(key);
  *m3 = 2;

  key = 67;
  Int32_t* m4= map_test.Allocate(key);
  *m4 =8;

  key = 32;
  EXPECT_EQ(*map_test.Find(key),100);

}


} // namespace driv_map
} // phoenix

#endif

