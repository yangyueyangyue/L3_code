/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       data_pair.h
 * @brief
 * @details
 *
 * @author     pengc
 * @date       2020.06.30
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/30  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_COMMON_DATA_PAIR_H_
#define PHOENIX_COMMON_DATA_PAIR_H_

#include "utils/log.h"
#include "utils/macros.h"

namespace phoenix {
namespace common {


/**
  *  @brief Struct holding two objects of arbitrary type.
  *
  *  @tparam T1  Type of first object.
  *  @tparam T2  Type of second object.
  */
template<class T1, class T2>
struct DataPair {
  typedef T1 first_type;    /// @c first_type is the first bound type
  typedef T2 second_type;   /// @c second_type is the second bound type

  T1 first;                 /// @c first is a copy of the first object
  T2 second;                /// @c second is a copy of the second object

  /** The default constructor creates @c first and @c second using their
   *  respective default constructors.  */
  DataPair() : first(), second() { }

  /** Two objects may be passed to a @c pair constructor to be copied.  */
  DataPair(const T1& a, const T2& b) : first(a), second(b) { }
};


} // namespace common
} // namespace phoenix


#endif // PHOENIX_COMMON_DATA_PAIR_H_

