/******************************************************************************
 ** String operator
 ******************************************************************************
 *
 *  String operator
 *
 *  @file       string_util.h
 *
 *  @author     kotei
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/

/**
 * @file
 * @brief Some string util functions.
 */

#ifndef PHOENIX_FRAMEWORK_STRING_UTIL_H_
#define PHOENIX_FRAMEWORK_STRING_UTIL_H_

#include <functional>
#include <sstream>
#include <string>

#include "google/protobuf/stubs/stringprintf.h"
#include "google/protobuf/stubs/strutil.h"

/**
 * @namespace phoenix::common
 * @brief phoenix::framework
 */
namespace phoenix {
namespace framework {

// Expose some useful utils from protobuf.
using google::protobuf::StringPiece;
using google::protobuf::StrAppend;
using google::protobuf::StrCat;
using google::protobuf::StringPrintf;
using google::protobuf::Join;

/**
 * @brief Check if a string ends with a pattern.
 * @param ori The original string. To see if it ends with a specified pattern.
 * @param pat The target pattern. To see if the original string ends with it.
 * @return Whether the original string ends with the specified pattern.
 */
inline bool EndWith(const std::string& ori, const std::string& pat) {
  return StringPiece(ori).ends_with(pat);
}

template <typename T>
std::string Print(const T& val) {
  std::ostringstream oss;
  oss << val;
  return oss.str();
}

/**
 * @brief Make arrays, conatiners and iterators printable.
 *
 * Usage:
 *   vector<int> vec = {1, 2, 3};
 *   std::cout << PrintIter(vec);
 *   std::cout << PrintIter(vec, ",");
 *   std::cout << PrintIter(vec.begin(), vec.end());
 *   std::cout << PrintIter(vec.begin(), vec.end(), "|");
 *
 *   int array[] = {1, 2, 3};
 *   std::cout << PrintIter(array);
 *   std::cout << PrintIter(array, "|");
 *   std::cout << PrintIter(array + 0, array + 10, "|");
 */
template <typename Iter>
std::string PrintIter(const Iter& begin, const Iter& end,
                      const std::string& delimiter = " ") {
  std::string result;
  Join(begin, end, delimiter.c_str(), &result);
  return result;
}

template <typename Iter>
std::string PrintIter(const Iter& begin, const Iter& end,
                      const std::function<std::string(Iter)> transformer,
                      const std::string& delimiter = " ") {
  std::string result;
  if (transformer) {
    for (auto iter = begin; iter != end; ++iter) {
      if (iter == begin) {
        StrAppend(&result, transformer(*iter));
      } else {
        StrAppend(&result, delimiter, transformer(*iter));
      }
    }
  } else {
    PrintIter(begin, end, delimiter);
  }
  return result;
}

template <typename Container, typename Iter>
std::string PrintIter(const Container& container,
                      const std::function<std::string(Iter)> transformer,
                      const std::string& delimiter = " ") {
  return PrintIter(container.begin(), container.end(), transformer, delimiter);
}

template <typename Container>
std::string PrintIter(const Container& container,
                      const std::string& delimiter = " ") {
  return PrintIter(container.begin(), container.end(), delimiter);
}

template <typename T, int length>
std::string PrintIter(T (&array)[length], T* end,
                      const std::string& delimiter = " ") {
  std::string result;
  Join(array, end, delimiter.c_str(), &result);
  return result;
}

template <typename T, int length>
std::string PrintIter(T (&array)[length], const std::string& delimiter = " ") {
  return PrintIter(array, array + length, delimiter);
}

/**
 * @brief Make conatiners and iterators printable. Similar to PrintIter but
 *        output the DebugString().
 */
template <typename Iter>
std::string PrintDebugStringIter(const Iter& begin, const Iter& end,
                                 const std::string& delimiter = " ") {
  std::string result;
  for (auto iter = begin; iter != end; ++iter) {
    if (iter == begin) {
      StrAppend(&result, iter->DebugString());
    } else {
      StrAppend(&result, delimiter, iter->DebugString());
    }
  }
  return result;
}

template <typename Container>
std::string PrintDebugStringIter(const Container& container,
                                 const std::string& delimiter = " ") {
  return PrintDebugStringIter(container.begin(), container.end(), delimiter);
}

}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_STRING_UTIL_H_
