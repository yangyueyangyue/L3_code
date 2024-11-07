//
#include "utils/linear_interpolation_c.h"
#include "math/math_utils_c.h"


Float64_t Phoenix_Common_Lerp_d(
    const Float64_t x0, const Float64_t x1, const Float64_t t) {
  return (x0 + t * (x1 - x0));
}

Float32_t Phoenix_Common_Lerp_f(
    const Float32_t x0, const Float32_t x1, const Float32_t t) {
  return (x0 + t * (x1 - x0));
}


Float32_t Phoenix_Common_AngleLerp_f(Float32_t from, Float32_t to, Float32_t t) {
  return (Phoenix_Common_NormalizeAngle_f(
            from + t * Phoenix_Common_AngleDiff_f(from, to)));
}

Int32_t Phoenix_Common_LerpInOrderedTable_f(
    const Float32_t key_table[], const Int32_t key_table_size,
    Float32_t key, Float32_t* t) {
  *t = 0.0F;
  if (key_table_size < 1) {
    return (-1);
  }

  Int32_t first = 0;
  Int32_t len = key_table_size - 1;
  Int32_t half = 0;
  Int32_t mid = 0;
  Int32_t lower_index = first;

  // 比第一个插值参数小，返回第一个
  if (key <= key_table[first]) {
    *t = 0;
    return (lower_index);
  }

  // 比最后一个插值参数大，返回第后一个
  if (key >= key_table[len]) {
    lower_index = len;
    *t = 0;
    if (len > 0) {
      lower_index = len - 1;
      *t = 1;
    }
    return (lower_index);
  }

#if 0
  Int32_t iter_count = 0;
  std::cout << "(at beginning) first=" << first
            << ", len=" << len
            << ", half=" << half
            << ", mid=" << mid
            << std::endl;
#endif

  while (len > 0) {
#if 0
    std::cout << ">>> iter_count=" << iter_count++ << std::endl;
#endif

    half = len >> 1;
    mid = first + half;

#if 0
    std::cout << "    update: half=" << half << ", mid=" << mid << std::endl;
#endif

    // 中位数大于待查找的数据, 在左半边序列中查找。
    if (key_table[mid] > key) {
      len = half;

#if 0
      std::cout << "    key_table[" << mid
                << "].key(" << key_table[mid]
                << ") > key(" << key
                << "); update: len=" << len
                << std::endl;
#endif
    } else {
      // 中位数小于等于待查找的数据, 在右半边序列中查找。
      first = mid + 1;
      len = len - half - 1;

#if 0
      std::cout << "    key_table[" << mid
                << "].key(" << key_table[mid]
                << ") <= key(" << key
                << "); update: first=" << first
                << ", len=" << len
                << std::endl;
#endif
    }

#if 0
    std::cout << "    after update: first=" << first
              << ", len=" << len
              << ", half=" << half
              << ", mid=" << mid
              << std::endl;
#endif
  }

  if ((first == key_table_size - 1) && !(key_table[first] > key)) {
    lower_index = first;
    *t = 0;
    if (first > 0) {
      lower_index = first - 1;
      *t = 1;
    }
    return (lower_index);
  }

  if (first > 0) {
    lower_index = first - 1;
    *t = (key - key_table[lower_index]) /
        (key_table[first] - key_table[lower_index]);
  } else {
    *t = 0;
    lower_index = first;
  }

#if 0
  std::cout << "lower_index="
            << lower_index
            << ", t=" << *t
            << std::endl;
#endif

  return (lower_index);
}

Int32_t Phoenix_Common_LerpInOrderedVector_f(
    const Matrix_t* key_table, Float32_t key, Float32_t* t) {
  Int32_t key_table_size =
      Phoenix_Com_Matrix_GetRows(key_table) *
      Phoenix_Com_Matrix_GetCols(key_table);

  *t = 0.0F;
  if (key_table_size < 1) {
    return (-1);
  }

  Int32_t first = 0;
  Int32_t len = key_table_size - 1;
  Int32_t half = 0;
  Int32_t mid = 0;
  Int32_t lower_index = first;

  // 比第一个插值参数小，返回第一个
  if (key <= *Phoenix_Com_Matrix_GetEleByIdx_f(key_table, first)) {
    *t = 0;
    return (lower_index);
  }

  // 比最后一个插值参数大，返回第后一个
  if (key >= *Phoenix_Com_Matrix_GetEleByIdx_f(key_table, len)) {
    lower_index = len;
    *t = 0;
    if (len > 0) {
      lower_index = len - 1;
      *t = 1;
    }
    return (lower_index);
  }

#if 0
  Int32_t iter_count = 0;
  std::cout << "(at beginning) first=" << first
            << ", len=" << len
            << ", half=" << half
            << ", mid=" << mid
            << std::endl;
#endif

  while (len > 0) {
#if 0
    std::cout << ">>> iter_count=" << iter_count++ << std::endl;
#endif

    half = len >> 1;
    mid = first + half;

#if 0
    std::cout << "    update: half=" << half << ", mid=" << mid << std::endl;
#endif

    // 中位数大于待查找的数据, 在左半边序列中查找。
    if (*Phoenix_Com_Matrix_GetEleByIdx_f(key_table, mid) > key) {
      len = half;

#if 0
      std::cout << "    key_table[" << mid
                << "].key(" << *Phoenix_Com_Matrix_GetEleByIdx_f(key_table, mid)
                << ") > key(" << key
                << "); update: len=" << len
                << std::endl;
#endif
    } else {
      // 中位数小于等于待查找的数据, 在右半边序列中查找。
      first = mid + 1;
      len = len - half - 1;

#if 0
      std::cout << "    key_table[" << mid
                << "].key(" << *Phoenix_Com_Matrix_GetEleByIdx_f(key_table, mid)
                << ") <= key(" << key
                << "); update: first=" << first
                << ", len=" << len
                << std::endl;
#endif
    }

#if 0
    std::cout << "    after update: first=" << first
              << ", len=" << len
              << ", half=" << half
              << ", mid=" << mid
              << std::endl;
#endif
  }

  if ((first == key_table_size - 1) &&
      !(*Phoenix_Com_Matrix_GetEleByIdx_f(key_table, first) > key)) {
    lower_index = first;
    *t = 0;
    if (first > 0) {
      lower_index = first - 1;
      *t = 1;
    }
    return (lower_index);
  }

  if (first > 0) {
    lower_index = first - 1;
    *t = (key - *Phoenix_Com_Matrix_GetEleByIdx_f(key_table, lower_index)) /
        (*Phoenix_Com_Matrix_GetEleByIdx_f(key_table, first) -
         *Phoenix_Com_Matrix_GetEleByIdx_f(key_table, lower_index));
  } else {
    *t = 0;
    lower_index = first;
  }

#if 0
  std::cout << "lower_index="
            << lower_index
            << ", t=" << *t
            << std::endl;
#endif

  return (lower_index);
}
