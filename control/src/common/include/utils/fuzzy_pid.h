/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       fuzzy_pid.h
 * @brief      模糊PID
 * @details    提供了模糊PID的算法
 *
 * @author     pengc
 * @date       2020.05.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/15  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_FUZZY_PID_H_
#define PHOENIX_COMMON_FUZZY_PID_H_

#include "utils/macros.h"


template <typename Scalar, Int32_t FuzzyTableSize>
Int32_t GetFuzzyLevelIndex(
    const Scalar level_table[FuzzyTableSize-1],
    Scalar value, Int32_t* membership_index, Scalar* membership_grade) {
  Int32_t level_index = 0;
  for (Int32_t i = 0; i < (FuzzyTableSize-1); ++i) {
    if (value <= level_table[i]) {
      break;
    }
    level_index++;
  }

  if (level_index > 0) {
    Scalar ratio = (value - level_table[level_index-1]) /
        (level_table[level_index] - level_table[level_index-1]);
    if (ratio < 0.5F) {
      if ((FuzzyTableSize/2) == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0F;
      } else {
        *membership_index = level_index - 1;
        *membership_grade = 0.5F - ratio;
      }
    } else {
      if ((FuzzyTableSize/2-1) == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0F;
      } else {
        *membership_index = level_index + 1;
        *membership_grade = ratio - 0.5F;
      }
    }
    if (level_index >= (FuzzyTableSize-1)) {
      *membership_index = level_index;
      *membership_grade = 0.0F;
    }
  } else {
    *membership_index = level_index;
    *membership_grade = 0.0F;
  }

  return (level_index);
}

template <typename Scalar, Int32_t FuzzyTableSize>
Scalar GetFuzzyGain(
    const Scalar gain_table[FuzzyTableSize][FuzzyTableSize],
    Int32_t row_index, Int32_t col_index,
    Int32_t row_membership_index, Int32_t col_membership_index,
    Scalar row_membership_grade, Scalar col_membership_grade) {

  Scalar gain =
      (gain_table[row_index][col_index] *
           (1.0F - row_membership_grade) * (1.0F - col_membership_grade) +
       gain_table[row_membership_index][col_index] *
           row_membership_grade * (1.0F - col_membership_grade) +
       gain_table[row_index][col_membership_index] *
           (1.0F - row_membership_grade) * col_membership_grade +
       gain_table[row_membership_index][col_membership_index] *
           row_membership_grade * col_membership_grade);

  return (gain);
}


#endif // PHOENIX_COMMON_FUZZY_PID_H_

