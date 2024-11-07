/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       fuzzy_pid_c.h
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

#ifndef PHOENIX_COMMON_UTILS_FUZZY_PID_C_H_
#define PHOENIX_COMMON_UTILS_FUZZY_PID_C_H_

#include <math.h>
#include <float.h>
#include "utils/macros.h"
#include "math/matrix_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct FuzzyPID_t
 * @brief 定义了Fuzzy PID的存储结构
 */
typedef struct _FuzzyPID_t FuzzyPID_t;
struct _FuzzyPID_t {
  /// 误差等级表
  Matrix_t* err_level_table_;
  /// 误差变化率等级表
  Matrix_t* err_spd_level_table_;
  /// 增益表Kp, Ki, Kd
  Matrix_t* gain_table_[3];

  /// 误差等级的索引
  Int32_t err_level_idx_;
  /// 相邻的误差模糊等级的索引
  Int32_t err_membership_idx_;
  /// 误差模糊程度
  Float64_t err_membership_grade_;
  /// 误差变化率等级的索引
  Int32_t err_spd_level_idx_;
  /// 相邻的误差变化率模糊等级的索引
  Int32_t err_spd_membership_idx_;
  /// 误差变化率模糊程度
  Float64_t err_spd_membership_grade_;
};


/**
 * @brief 初始化模糊PID实例
 * @param[in] ins 模糊PID实例
 * @param[in] err_lv_tab 误差等级表
 * @param[in] err_spd_lv_tab 误差变化率等级表
 * @return < 0 ~ NG, 0 ~ OK
 */
Int32_t Phoenix_Com_FuzzyPID_Init(
    FuzzyPID_t* ins, Matrix_t* err_lv_tab, Matrix_t* err_spd_lv_tab);

/**
 * @brief 设置PID增益表
 * @param[in] ins 模糊PID实例
 * @param[in] idx 需要设置的PID增益表的索引
 * @param[in] gain_tab PID增益表
 * @return <0 ~ NG, 0 ~ OK
 */
Int32_t Phoenix_Com_FuzzyPID_SetGainTable(
    FuzzyPID_t* ins, Int32_t idx, Matrix_t* gain_tab);

/**
 * @brief 检查此模糊PID实例是否有效
 * @param[in] ins 模糊PID实例
 * @return <0 ~ NG, 0 ~ OK
 */
Int32_t Phoenix_Com_FuzzyPID_CheckConsistent(const FuzzyPID_t* ins);

/**
 * @brief 更新当前误差等级
 * @param[in] ins 模糊PID实例
 * @param[in] err 当前误差
 * @param[in] err_spd 当前误差变化率
 */
void Phoenix_Com_FuzzyPID_UpdateErrLevel_f(
    FuzzyPID_t* ins, Float32_t err, Float32_t err_spd);
void Phoenix_Com_FuzzyPID_UpdateErrLevel_d(
    FuzzyPID_t* ins, Float64_t err, Float64_t err_spd);

/**
 * @brief 计算当前PID增益
 * @param[in] ins 模糊PID实例
 * @param[in] idx 需要计算的PID增益的索引
 * @return 当前PID增益
 */
Float32_t Phoenix_Com_FuzzyPID_GetGain_f(FuzzyPID_t* ins, Int32_t idx);
Float64_t Phoenix_Com_FuzzyPID_GetGain_d(FuzzyPID_t* ins, Int32_t idx);

/**
 * @brief 获取模糊等级的索引
 * @param[in] level_table 模糊等级表
 * @param[in] value 待查寻的值
 * @param[out] membership_index 相邻的模糊等级的索引
 * @param[out] membership_grade 模糊程度
 * @return 模糊等级的索引
 */
Int32_t Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_f(
    const Matrix_t* level_table,
    Float32_t value, Int32_t* membership_index, Float32_t* membership_grade);

Int32_t Phoenix_Com_FuzzyPID_GetFuzzyLevelIdx_d(
    const Matrix_t* level_table,
    Float64_t value, Int32_t* membership_index, Float64_t* membership_grade);

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
Float32_t Phoenix_Com_FuzzyPID_GetFuzzyGain_f(
    const Matrix_t* gain_table,
    Int32_t row_index, Int32_t col_index,
    Int32_t row_membership_index, Int32_t col_membership_index,
    Float32_t row_membership_grade, Float32_t col_membership_grade);

Float64_t Phoenix_Com_FuzzyPID_GetFuzzyGain_d(
    const Matrix_t* gain_table,
    Int32_t row_index, Int32_t col_index,
    Int32_t row_membership_index, Int32_t col_membership_index,
    Float64_t row_membership_grade, Float64_t col_membership_grade);



#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_UTILS_FUZZY_PID_C_H_

