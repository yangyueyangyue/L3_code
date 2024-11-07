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
#include "math/matrix.h"


namespace phoenix {
namespace common {


/**
 * @class FuzzyPID
 * @brief 定义了模糊PID的存储结构及相应的计算方法
 * @param Scalar 用户数据类型
 * @param FuzzyTableSize 模糊表的尺寸
 */
template <typename Scalar, Int32_t FuzzyTableSize>
class FuzzyPID {
public:
  /**
   * @brief 构造函数
   */
  FuzzyPID();

  /**
   * @brief 设置误差等级表
   * @param[in] tab 以数组形式存放的误差等级表(行主序)
   */
  void SetErrLvTab(const Scalar tab[FuzzyTableSize-1]);

  /**
   * @brief 设置误差变化率等级表
   * @param[in] tab 以数组形式存放的误差变化率等级表(行主序)
   */
  void SetErrSpdLvTab(const Scalar tab[FuzzyTableSize-1]);

  /**
   * @brief 设置增益矩阵
   * @param[in] idx 需要设置的PID增益表的索引
   * @param[in] gain_matrix PID增益矩阵
   */
  void SetGainMatrix(
      Int32_t idx, Matrix<Scalar, FuzzyTableSize, FuzzyTableSize>* gain_matrix);

  /**
   * @brief 设置增益表
   * @param[in] idx 需要设置的PID增益表的索引
   * @param[in] tab 以数组形式存放的PID增益表(行主序)
   * @warning { 必须提前调用 SetGainMatrix(Int32_t, Matrix) 函数, \n
   *            设置存放增益表的矩阵 }
   */
  void SetGainTable(
      Int32_t idx, const Scalar tab[FuzzyTableSize*FuzzyTableSize]);

  /**
   * @brief 更新当前误差等级
   * @param[in] err 当前误差
   * @param[in] err_spd 当前误差变化率
   */
  void UpdateErrLevel(Scalar err, Scalar err_spd);

  /**
   * @brief 获取当前误差等级的索引
   * @return 当前误差等级的索引
   */
  Scalar GetErrLevelIdx() const {
    return (err_level_idx_);
  }

  /**
   * @brief 获取当前误差变化率等级的索引
   * @return 当前误差变化率等级的索引
   */
  Scalar GetErrSpdLevelIdx() const {
    return (err_spd_level_idx_);
  }

  /**
   * @brief 计算当前PID增益
   * @param[in] idx 需要计算的PID增益的索引
   * @return 当前PID增益
   */
  Scalar GetGain(Int32_t idx);

  /**
   * @brief 获取模糊等级的索引
   * @param[in] level_table 模糊等级表
   * @param[in] value 待查寻的值
   * @param[out] membership_index 相邻的模糊等级的索引
   * @param[out] membership_grade 模糊程度
   * @return 模糊等级的索引
   */
  Int32_t GetFuzzyLevelIdx(
      const Matrix<Scalar, FuzzyTableSize-1, 1>& level_table, Scalar value,
      Int32_t* membership_index, Scalar* membership_grade) const;

  /**
   * @brief 根据模糊等级获取增益值
   * @param[in] gain_table 增益表
   * @param[in] row_index 增益表的行索引
   * @param[in] row_index 增益表的列索引
   * @param[out] row_membership_index 相邻的模糊等级的行索引
   * @param[out] col_membership_index 相邻的模糊等级的列索引
   * @param[out] row_membership_grade 模糊程度(行)
   * @param[out] col_membership_grade 模糊程度(列)
   * @return 增益值
   */
  Scalar GetFuzzyGain(
      const Matrix<Scalar, FuzzyTableSize, FuzzyTableSize>& gain_table,
      Int32_t row_index, Int32_t col_index,
      Int32_t row_membership_index, Int32_t col_membership_index,
      Scalar row_membership_grade, Scalar col_membership_grade);

private:
  /// 最大增益表的数量
  enum { MAX_GAIN_TABLE_NUM = 3 };

  /// 误差等级表
  Matrix<Scalar, FuzzyTableSize-1, 1> err_level_table_;
  /// 误差变化率等级表
  Matrix<Scalar, FuzzyTableSize-1, 1> err_spd_level_table_;
  /// 增益表Kp, Ki, Kd
  Matrix<Scalar, FuzzyTableSize, FuzzyTableSize>* gain_table_[MAX_GAIN_TABLE_NUM];

  /// 误差等级的索引
  Int32_t err_level_idx_;
  /// 相邻的误差模糊等级的索引
  Int32_t err_membership_idx_;
  /// 误差模糊程度
  Scalar err_membership_grade_;
  /// 误差变化率等级的索引
  Int32_t err_spd_level_idx_;
  /// 相邻的误差变化率模糊等级的索引
  Int32_t err_spd_membership_idx_;
  /// 误差变化率模糊程度
  Scalar err_spd_membership_grade_;
};


template <typename Scalar, Int32_t FuzzyTableSize>
FuzzyPID<Scalar, FuzzyTableSize>::FuzzyPID() {
  for (Int32_t i = 0; i < MAX_GAIN_TABLE_NUM; ++i) {
    gain_table_[i] = Nullptr_t;
  }

  err_level_idx_ = 0;
  err_membership_idx_ = 0;
  err_membership_grade_ = 0;
  err_spd_level_idx_ = 0;
  err_spd_membership_idx_ = 0;
  err_spd_membership_grade_ = 0;
}

template <typename Scalar, Int32_t FuzzyTableSize>
void FuzzyPID<Scalar, FuzzyTableSize>::SetErrLvTab(
    const Scalar tab[FuzzyTableSize-1]) {
  for (Int32_t i = 0; i < FuzzyTableSize-1; ++i) {
    err_level_table_(i) = tab[i];
  }
}

template <typename Scalar, Int32_t FuzzyTableSize>
void FuzzyPID<Scalar, FuzzyTableSize>::SetErrSpdLvTab(
    const Scalar tab[FuzzyTableSize-1]) {
  for (Int32_t i = 0; i < FuzzyTableSize-1; ++i) {
    err_spd_level_table_(i) = tab[i];
  }
}

template <typename Scalar, Int32_t FuzzyTableSize>
void FuzzyPID<Scalar, FuzzyTableSize>::SetGainMatrix(
    Int32_t idx, Matrix<Scalar, FuzzyTableSize, FuzzyTableSize>* gain_matrix) {
  if ((idx < 0) || (idx >= MAX_GAIN_TABLE_NUM)) {
    LOG_ERR << "Invalid gain table index = " << idx;
    return;
  }

  gain_table_[idx] = gain_matrix;
}

template <typename Scalar, Int32_t FuzzyTableSize>
void FuzzyPID<Scalar, FuzzyTableSize>::SetGainTable(
    Int32_t idx, const Scalar tab[FuzzyTableSize*FuzzyTableSize]) {
  if ((idx < 0) || (idx >= MAX_GAIN_TABLE_NUM)) {
    LOG_ERR << "Invalid gain table index = " << idx;
    return;
  }

  if (Nullptr_t == gain_table_[idx]) {
    LOG_ERR << "This gain table have not been set.";
    return;
  }

  for (Int32_t i = 0; i < FuzzyTableSize; ++i) {
    for (Int32_t j = 0; j < FuzzyTableSize; ++j) {
      (*gain_table_[idx])(i, j) = tab[i*FuzzyTableSize+j];
    }
  }
}

template <typename Scalar, Int32_t FuzzyTableSize>
void FuzzyPID<Scalar, FuzzyTableSize>::UpdateErrLevel(
    Scalar err, Scalar err_spd) {
  err_level_idx_ = GetFuzzyLevelIdx(
        err_level_table_, err,
        &err_membership_idx_, &err_membership_grade_);

  err_spd_level_idx_ = GetFuzzyLevelIdx(
        err_spd_level_table_, err_spd,
        &err_spd_membership_idx_, &err_spd_membership_grade_);
}

template <typename Scalar, Int32_t FuzzyTableSize>
Scalar FuzzyPID<Scalar, FuzzyTableSize>::GetGain(Int32_t idx) {
  Scalar gain = 0.0F;

  if ((idx < 0) || (idx >= MAX_GAIN_TABLE_NUM)) {
    LOG_ERR << "Invalid idx = " << idx;
    return (gain);
  }

  if (Nullptr_t != gain_table_[idx]) {
    gain = GetFuzzyGain(
          *gain_table_[idx],
          err_level_idx_, err_spd_level_idx_,
          err_membership_idx_, err_spd_membership_idx_,
          err_membership_grade_, err_spd_membership_grade_);
  } else {
    LOG_ERR << "The gain table " << idx << " is null.";
  }

  return (gain);
}

template <typename Scalar, Int32_t FuzzyTableSize>
Int32_t FuzzyPID<Scalar, FuzzyTableSize>::GetFuzzyLevelIdx(
    const Matrix<Scalar, FuzzyTableSize-1, 1>& level_table, Scalar value,
    Int32_t* membership_index, Scalar* membership_grade) const {
  const Int32_t fuzzy_table_size = FuzzyTableSize;
  Int32_t level_index = 0;

  for (Int32_t i = 0; i < (fuzzy_table_size-1); ++i) {
    if (value <= level_table(i)) {
      break;
    }
    level_index++;
  }

  if ((0 < level_index) && (level_index < (fuzzy_table_size-1))) {
    Scalar lower_value = level_table(level_index-1);
    Scalar upper_value = level_table(level_index);
    Scalar ratio = (value - lower_value) / (upper_value - lower_value);
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

template <typename Scalar, Int32_t FuzzyTableSize>
Scalar FuzzyPID<Scalar, FuzzyTableSize>::GetFuzzyGain(
    const Matrix<Scalar, FuzzyTableSize, FuzzyTableSize>& gain_table,
    Int32_t row_index, Int32_t col_index,
    Int32_t row_membership_index, Int32_t col_membership_index,
    Scalar row_membership_grade, Scalar col_membership_grade) {
  Scalar gain =
      ((gain_table(row_index, col_index)) *
           (1.0F - row_membership_grade) * (1.0F - col_membership_grade) +
       (gain_table(row_membership_index, col_index)) *
           row_membership_grade * (1.0F - col_membership_grade) +
       (gain_table(row_index, col_membership_index)) *
           (1.0F - row_membership_grade) * col_membership_grade +
       (gain_table(row_membership_index, col_membership_index)) *
           row_membership_grade * col_membership_grade);

  COM_CHECK(row_index < FuzzyTableSize);
  COM_CHECK(row_membership_index < FuzzyTableSize);
  COM_CHECK(col_index < FuzzyTableSize);
  COM_CHECK(col_membership_grade < FuzzyTableSize);

  return (gain);
}


} // common
} // phoenix


#endif // PHOENIX_COMMON_FUZZY_PID_H_

