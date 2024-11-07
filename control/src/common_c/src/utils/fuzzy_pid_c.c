/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       fuzzy_pid_c.c
 * @brief      Fuzzy PID 运算
 * @details    定义相关Fuzzy PID算法
 *
 * @author     pengc
 * @date       2022.11.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "utils/fuzzy_pid_c.h"

#include "utils/log_c.h"


/*
 * @brief 初始化模糊PID实例
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_FuzzyPID_Init(
    FuzzyPID_t* ins, Matrix_t* err_lv_tab, Matrix_t* err_spd_lv_tab) {
  Int32_t valid = -1;

  ins->err_level_table_ = err_lv_tab;
  ins->err_spd_level_table_ = err_spd_lv_tab;
  for (Int32_t i = 0; i < 3; ++i) {
    ins->gain_table_[i] = Null_t;
  }

  ins->err_level_idx_ = 0;
  ins->err_membership_idx_ = 0;
  ins->err_membership_grade_ = 0;
  ins->err_spd_level_idx_ = 0;
  ins->err_spd_membership_idx_ = 0;
  ins->err_spd_membership_grade_ = 0;

  valid = Phoenix_Com_FuzzyPID_CheckConsistent(ins);
  if (valid < 0) {
    LOG_ERR_C("The PID instance is not valid.\n");
  }

  return (valid);
}

/*
 * @brief 设置PID增益表
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_FuzzyPID_SetGainTable(
    FuzzyPID_t* ins, Int32_t idx, Matrix_t* gain_tab) {
  Int32_t valid = -1;

  if ((idx < 0) || (idx > 2)) {
    LOG_ERR_C("Invalid gain table index"
              " = %d\n", idx);
    return (valid);
  }

  ins->gain_table_[idx] = gain_tab;

  valid = Phoenix_Com_FuzzyPID_CheckConsistent(ins);
  if (valid < 0) {
    LOG_ERR_C("The PID instance is not valid.\n");
  }

  return (valid);
}

/*
 * @brief 检查此模糊PID实例是否有效
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_FuzzyPID_CheckConsistent(const FuzzyPID_t* ins) {
  Int32_t consistent = 0;
  Int32_t err_table_size = 0;
  Int32_t err_spd_table_size = 0;

  if (Null_t != ins->err_level_table_) {
    err_table_size = 1 +
        Phoenix_Com_Matrix_GetRows(ins->err_level_table_) *
        Phoenix_Com_Matrix_GetCols(ins->err_level_table_);
    if ((err_table_size < 2) || ((err_table_size-1) % 2) != 1) {
      consistent = -1;
      LOG_ERR_C("Invalid size of error level table, size=%d\n",
                err_table_size-1);
    }
  } else {
    consistent = -2;
    LOG_ERR_C("The error level table is null.\n");
  }
  if (Null_t != ins->err_spd_level_table_) {
    err_spd_table_size = 1 +
        Phoenix_Com_Matrix_GetRows(ins->err_spd_level_table_) *
        Phoenix_Com_Matrix_GetCols(ins->err_spd_level_table_);
    if ((err_spd_table_size < 2) || ((err_spd_table_size-1) % 2) != 1) {
      consistent = -3;
      LOG_ERR_C("Invalid size of error speed level table, size=%d\n",
                err_spd_table_size-1);
    }
  } else {
    consistent = -4;
    LOG_ERR_C("The error speed level table is null.\n");
  }
  if (Null_t != ins->gain_table_[0]) {
    if (err_table_size != Phoenix_Com_Matrix_GetRows(ins->gain_table_[0])) {
      consistent = -5;
      LOG_ERR_C("[ERR] Gain table 0 is not consistent, size of err_level_table is %d, "
                "but rows of table 0 is %d.\n",
                err_table_size,
                Phoenix_Com_Matrix_GetRows(ins->gain_table_[0]));
    }
    if (err_spd_table_size != Phoenix_Com_Matrix_GetCols(ins->gain_table_[0])) {
      consistent = -6;
      LOG_ERR_C("[ERR] Gain table 0 is not consistent, size of err_spd_level_table is %d, "
                "but cols of table 0 is %d.\n",
                err_spd_table_size,
                Phoenix_Com_Matrix_GetCols(ins->gain_table_[0]));
    }
  }
  if (Null_t != ins->gain_table_[1]) {
    if (err_table_size != Phoenix_Com_Matrix_GetRows(ins->gain_table_[1])) {
      consistent = -7;
      LOG_ERR_C("[ERR] Gain table 1 is not consistent, size of err_level_table is %d, "
                "but rows of table 1 is %d.\n",
                err_table_size,
                Phoenix_Com_Matrix_GetRows(ins->gain_table_[1]));
    }
    if (err_spd_table_size != Phoenix_Com_Matrix_GetCols(ins->gain_table_[1])) {
      consistent = -8;
      LOG_ERR_C("[ERR] Gain table 1 is not consistent, size of err_spd_level_table is %d, "
                "but cols of table 1 is %d.\n",
                err_spd_table_size,
                Phoenix_Com_Matrix_GetCols(ins->gain_table_[1]));
    }
  }
  if (Null_t != ins->gain_table_[2]) {
    if (err_table_size != Phoenix_Com_Matrix_GetRows(ins->gain_table_[2])) {
      consistent = -9;
      LOG_ERR_C("[ERR] Gain table 2 is not consistent, size of err_level_table is %d, "
                "but rows of table 2 is %d.\n",
                err_table_size,
                Phoenix_Com_Matrix_GetRows(ins->gain_table_[2]));
    }
    if (err_spd_table_size != Phoenix_Com_Matrix_GetCols(ins->gain_table_[2])) {
      consistent = -10;
      LOG_ERR_C("[ERR] Gain table 2 is not consistent, size of err_spd_level_table is %d, "
                "but cols of table 2 is %d.\n",
                err_spd_table_size,
                Phoenix_Com_Matrix_GetCols(ins->gain_table_[2]));
    }
  }

  return (consistent);
}

/*
 * @brief 更新当前误差等级
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_FuzzyPID_UpdateErrLevel_f(
    FuzzyPID_t* ins, Float32_t err, Float32_t err_spd) {
  Float32_t membership_grade = 0;

  ins->err_level_idx_ = Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_f(
        ins->err_level_table_, err,
        &(ins->err_membership_idx_), &membership_grade);
  ins->err_membership_grade_ = membership_grade;

  ins->err_spd_level_idx_ = Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_f(
        ins->err_spd_level_table_, err_spd,
        &(ins->err_spd_membership_idx_), &membership_grade);
  ins->err_spd_membership_grade_ = membership_grade;
}

void Phoenix_Com_FuzzyPID_UpdateErrLevel_d(
    FuzzyPID_t* ins, Float64_t err, Float64_t err_spd) {
  Float64_t membership_grade = 0;

  ins->err_level_idx_ = Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_d(
        ins->err_level_table_, err,
        &(ins->err_membership_idx_), &membership_grade);
  ins->err_membership_grade_ = membership_grade;

  ins->err_spd_level_idx_ = Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_d(
        ins->err_spd_level_table_, err_spd,
        &(ins->err_spd_membership_idx_), &membership_grade);
  ins->err_spd_membership_grade_ = membership_grade;
}

/*
 * @brief 计算当前PID增益
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t Phoenix_Com_FuzzyPID_GetGain_f(FuzzyPID_t* ins, Int32_t idx) {
  Float32_t gain = 0.0F;

  if ((idx < 0) || (idx > 2)) {
    LOG_ERR_C("Invalid idx = %d\n", idx);
    return (gain);
  }
  if (Null_t != ins->gain_table_[idx]) {
    gain = Phoenix_Com_FuzzyPID_GetFuzzyGain_f(
          ins->gain_table_[idx],
          ins->err_level_idx_, ins->err_spd_level_idx_,
          ins->err_membership_idx_, ins->err_spd_membership_idx_,
          ins->err_membership_grade_, ins->err_spd_membership_grade_);
  } else {
    LOG_ERR_C("The gain table %d is null.", idx);
  }

  return (gain);
}

Float64_t Phoenix_Com_FuzzyPID_GetGain_d(FuzzyPID_t* ins, Int32_t idx) {
  Float64_t gain = 0.0F;

  if ((idx < 0) || (idx > 2)) {
    LOG_ERR_C("Invalid idx = %d\n", idx);
    return (gain);
  }
  if (Null_t != ins->gain_table_[idx]) {
    gain = Phoenix_Com_FuzzyPID_GetFuzzyGain_d(
          ins->gain_table_[idx],
          ins->err_level_idx_, ins->err_spd_level_idx_,
          ins->err_membership_idx_, ins->err_spd_membership_idx_,
          ins->err_membership_grade_, ins->err_spd_membership_grade_);
  } else {
    LOG_ERR_C("The gain table %d is null.", idx);
  }

  return (gain);
}

/*
 * @brief 获取模糊等级的索引
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_f(
    const Matrix_t* level_table,
    Float32_t value, Int32_t* membership_index, Float32_t* membership_grade) {
  const Int32_t fuzzy_table_size = 1 +
      Phoenix_Com_Matrix_GetRows(level_table) *
      Phoenix_Com_Matrix_GetCols(level_table);
  Int32_t level_index = 0;

  for (Int32_t i = 0; i < (fuzzy_table_size-1); ++i) {
    if (value <= *Phoenix_Com_Matrix_GetEleByIdx_f(level_table, i)) {
      break;
    }
    level_index++;
  }

  if ((0 < level_index) && (level_index < (fuzzy_table_size-1))) {
    Float32_t lower_value =
        *Phoenix_Com_Matrix_GetEleByIdx_f(level_table, level_index-1);
    Float32_t upper_value =
        *Phoenix_Com_Matrix_GetEleByIdx_f(level_table, level_index);
    Float32_t ratio = (value - lower_value) / (upper_value - lower_value);
    if (ratio < 0.5F) {
      if ((fuzzy_table_size/2) == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0F;
      } else {
        *membership_index = level_index - 1;
        *membership_grade = 0.5F - ratio;
      }
    } else {
      if ((fuzzy_table_size/2-1) == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0F;
      } else {
        *membership_index = level_index + 1;
        *membership_grade = ratio - 0.5F;
      }
    }
    if (level_index >= (fuzzy_table_size-1)) {
      *membership_index = level_index;
      *membership_grade = 0.0F;
    }
  } else {
    *membership_index = level_index;
    *membership_grade = 0.0F;
  }

  return (level_index);
}

Int32_t Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_d(
    const Matrix_t* level_table,
    Float64_t value, Int32_t* membership_index, Float64_t* membership_grade) {
  const Int32_t fuzzy_table_size = 1 +
      Phoenix_Com_Matrix_GetRows(level_table) *
      Phoenix_Com_Matrix_GetCols(level_table);
  Int32_t level_index = 0;

  for (Int32_t i = 0; i < (fuzzy_table_size-1); ++i) {
    if (value <= *Phoenix_Com_Matrix_GetEleByIdx_d(level_table, i)) {
      break;
    }
    level_index++;
  }

  if (level_index > 0) {
    Float64_t lower_value =
        *Phoenix_Com_Matrix_GetEleByIdx_d(level_table, level_index-1);
    Float64_t upper_value =
        *Phoenix_Com_Matrix_GetEleByIdx_d(level_table, level_index);
    Float64_t ratio = (value - lower_value) / (upper_value - lower_value);
    if (ratio < 0.5) {
      if ((fuzzy_table_size/2) == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0;
      } else {
        *membership_index = level_index - 1;
        *membership_grade = 0.5 - ratio;
      }
    } else {
      if ((fuzzy_table_size/2-1) == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0;
      } else {
        *membership_index = level_index + 1;
        *membership_grade = ratio - 0.5;
      }
    }
    if (level_index >= (fuzzy_table_size-1)) {
      *membership_index = level_index;
      *membership_grade = 0.0;
    }
  } else {
    *membership_index = level_index;
    *membership_grade = 0.0;
  }

  return (level_index);
}

/*
 * @brief 根据模糊等级获取增益值
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t Phoenix_Com_FuzzyPID_GetFuzzyGain_f(
    const Matrix_t* gain_table,
    Int32_t row_index, Int32_t col_index,
    Int32_t row_membership_index, Int32_t col_membership_index,
    Float32_t row_membership_grade, Float32_t col_membership_grade) {

  Float32_t gain =
      ((*Phoenix_Com_Matrix_GetEle_f(gain_table, row_index, col_index)) *
           (1.0F - row_membership_grade) * (1.0F - col_membership_grade) +
       (*Phoenix_Com_Matrix_GetEle_f(gain_table, row_membership_index, col_index)) *
           row_membership_grade * (1.0F - col_membership_grade) +
       (*Phoenix_Com_Matrix_GetEle_f(gain_table, row_index, col_membership_index)) *
           (1.0F - row_membership_grade) * col_membership_grade +
       (*Phoenix_Com_Matrix_GetEle_f(gain_table, row_membership_index, col_membership_index)) *
           row_membership_grade * col_membership_grade);

  COM_CHECK_C(row_index < Phoenix_Com_Matrix_GetRows(gain_table));
  COM_CHECK_C(row_membership_index < Phoenix_Com_Matrix_GetRows(gain_table));
  COM_CHECK_C(col_index < Phoenix_Com_Matrix_GetCols(gain_table));
  COM_CHECK_C(col_membership_grade < Phoenix_Com_Matrix_GetCols(gain_table));

  return (gain);
}

Float64_t Phoenix_Com_FuzzyPID_GetFuzzyGain_d(
    const Matrix_t* gain_table,
    Int32_t row_index, Int32_t col_index,
    Int32_t row_membership_index, Int32_t col_membership_index,
    Float64_t row_membership_grade, Float64_t col_membership_grade) {

  Float64_t gain =
      ((*Phoenix_Com_Matrix_GetEle_d(gain_table, row_index, col_index)) *
           (1.0F - row_membership_grade) * (1.0F - col_membership_grade) +
       (*Phoenix_Com_Matrix_GetEle_d(gain_table, row_membership_index, col_index)) *
           row_membership_grade * (1.0F - col_membership_grade) +
       (*Phoenix_Com_Matrix_GetEle_d(gain_table, row_index, col_membership_index)) *
           (1.0F - row_membership_grade) * col_membership_grade +
       (*Phoenix_Com_Matrix_GetEle_d(gain_table, row_membership_index, col_membership_index)) *
           row_membership_grade * col_membership_grade);

  COM_CHECK_C(row_index < Phoenix_Com_Matrix_GetRows(gain_table));
  COM_CHECK_C(row_membership_index < Phoenix_Com_Matrix_GetRows(gain_table));
  COM_CHECK_C(col_index < Phoenix_Com_Matrix_GetCols(gain_table));
  COM_CHECK_C(col_membership_grade < Phoenix_Com_Matrix_GetCols(gain_table));

  return (gain);
}
