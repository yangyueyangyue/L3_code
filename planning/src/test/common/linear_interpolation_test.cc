//
#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif

#include "container/static_vector.h"
#include "utils/linear_interpolation.h"


#if (ENABLE_GTEST)

namespace phoenix {
namespace common {

struct OrderedInterpolationTableNode {
  Float32_t key;
};

TEST(LinearInterpolation, LerpInOrderedTable) {
  OrderedInterpolationTableNode node;
  StaticVector<OrderedInterpolationTableNode, 4> table;

  node.key = 0;
  table.PushBack(node);
  node.key = 1;
  table.PushBack(node);
  node.key = 2;
  table.PushBack(node);
  node.key = 3;
  table.PushBack(node);

  Float32_t t = 0;
  Int32_t lower = LerpInOrderedTable(table, 0.0f, &t);
  EXPECT_EQ(lower, 0);
  EXPECT_FLOAT_EQ(t, 0.0f);

  lower = LerpInOrderedTable(table, -0.5f, &t);
  EXPECT_EQ(lower, 0);
  EXPECT_FLOAT_EQ(t, 0.0f);

  lower = LerpInOrderedTable(table, 0.5f, &t);
  EXPECT_EQ(lower, 0);
  EXPECT_FLOAT_EQ(t, 0.5f);

  lower = LerpInOrderedTable(table, 1.0f, &t);
  EXPECT_EQ(lower, 1);
  EXPECT_FLOAT_EQ(t, 0.0f);

  lower = LerpInOrderedTable(table, 1.5f, &t);
  EXPECT_EQ(lower, 1);
  EXPECT_FLOAT_EQ(t, 0.5f);

  lower = LerpInOrderedTable(table, 2.0f, &t);
  EXPECT_EQ(lower, 2);
  EXPECT_FLOAT_EQ(t, 0.0f);

  lower = LerpInOrderedTable(table, 2.5f, &t);
  EXPECT_EQ(lower, 2);
  EXPECT_FLOAT_EQ(t, 0.5f);

  lower = LerpInOrderedTable(table, 3.0f, &t);
  EXPECT_EQ(lower, 2);
  EXPECT_FLOAT_EQ(t, 1.0f);

  lower = LerpInOrderedTable(table, 3.5f, &t);
  EXPECT_EQ(lower, 2);
  EXPECT_FLOAT_EQ(t, 1.0f);
}


} // namespace common
} // namespace phoenix

#endif

