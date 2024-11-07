//
#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif

#include "container/data_pool.h"

#if (ENABLE_GTEST)

namespace phoenix {
namespace common {


TEST(DataPool, Create) {

  DataPool<Int32_t, 3> data_pool;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "allocate 1 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 2 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 3 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 4 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 5 = " << data_pool.Allocate() << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "Deallocate 2 ret=" << data_pool.Deallocate(2) << std::endl;
  std::cout << "Deallocate 0 ret=" << data_pool.Deallocate(0) << std::endl;
  std::cout << "Deallocate 1 ret=" << data_pool.Deallocate(1) << std::endl;
  std::cout << "Deallocate 2 ret=" << data_pool.Deallocate(2) << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "allocate 1 = " << data_pool.Allocate() << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "Deallocate 1 ret=" << data_pool.Deallocate(1) << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "allocate 1 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 2 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 3 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 4 = " << data_pool.Allocate() << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "Deallocate 0 ret=" << data_pool.Deallocate(0) << std::endl;
  std::cout << "Deallocate 2 ret=" << data_pool.Deallocate(2) << std::endl;
  std::cout << "Deallocate 1 ret=" << data_pool.Deallocate(1) << std::endl;
  std::cout << "Deallocate 1 ret=" << data_pool.Deallocate(0) << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  std::cout << "GetVirtualAddr(1) = "
            << reinterpret_cast<const void*>(data_pool.GetVirtualAddr(1))
            << std::endl;

  std::cout << "allocate 1 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 2 = " << data_pool.Allocate() << std::endl;
  std::cout << "allocate 3 = " << data_pool.Allocate() << std::endl;

#if ENABLE_DATA_POOL_TRACE
  std::cout << data_pool.DebugString() << std::endl;
#endif

  Int32_t* data_addr_1 = data_pool.GetVirtualAddr(1);
  std::cout << "GetVirtualAddr(1) = "
            << reinterpret_cast<const void*>(data_addr_1) << std::endl;

  Int32_t* data_addr_2 = data_pool.GetVirtualAddr(2);
  std::cout << "GetVirtualAddr(2) = "
            << reinterpret_cast<const void*>(data_addr_2) << std::endl;

  Int32_t* data_addr_0 = data_pool.GetVirtualAddr(0);
  std::cout << "GetVirtualAddr(0) = "
            << reinterpret_cast<const void*>(data_addr_0) << std::endl;

  std::cout << "data_pool.GetOffset(addr_1) = "
            << data_pool.GetOffset(data_addr_1) << std::endl;

  std::cout << "data_pool.GetOffset(addr_2) = "
            << data_pool.GetOffset(data_addr_2) << std::endl;

  std::cout << "data_pool.GetOffset(addr_0) = "
            << data_pool.GetOffset(data_addr_0) << std::endl;
}


} // namespace common
} // namespace phoenix

#endif

