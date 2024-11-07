/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       matrix_c.h
 * @brief      矩阵运算
 * @details    定义相关矩阵运算算法
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

#ifndef PHOENIX_COMMON_MATH_MATRIX_C_H_
#define PHOENIX_COMMON_MATH_MATRIX_C_H_

#include <math.h>
#include <float.h>
#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct Matrix_t
 * @brief 定义了矩阵的存储结构(列主序矩阵)
 */
typedef struct _Matrix_t Matrix_t;
struct _Matrix_t {
  /// 最大行数
  Int32_t max_rows_;
  /// 最大列数
  Int32_t max_cols_;

  /**
   * @struct _Block_t
   * @brief 矩阵分块信息
   */
  struct _Block_t {
    /// 分块矩阵的行数
    Int32_t rows;
    /// 矩阵分块的列数
    Int32_t cols;
    /// 分块矩阵在原始矩阵中起始行号(从0开始)
    Int32_t start_row;
    /// 分块矩阵在原始矩阵的起始列号(从0开始)
    Int32_t start_col;
  } block_;

  // 存储矩阵元素的数组
  void* storage_;
};


/**
 * @brief 初始化矩阵
 * @param[in] ins 矩阵实例
 * @param[in] max_rows 行号
 * @param[in] max_cols 列号
 * @param[in] buf 存储矩阵元素的数组的首地址
 */
void Phoenix_Com_Matrix_Init(
    Matrix_t* ins, Int32_t max_rows, Int32_t max_cols, void* buf);

/**
 * @brief 设置矩阵分块信息
 * @param[in] ins 矩阵实例
 * @param[in] start_row 分块矩阵的起始行号(从0开始)
 * @param[in] start_col 分块矩阵的起始列号(从0开始)
 * @param[in] rows 分块矩阵的行数
 * @param[in] cols 分块矩阵的列数
 */
void Phoenix_Com_Matrix_SetBlock(
    Matrix_t* ins, Int32_t start_row, Int32_t start_col,
    Int32_t rows, Int32_t cols);

/**
 * @brief 获取当前矩阵的行数
 * @param[in] ins 矩阵实例
 * @return 当前矩阵的行数
 */
Int32_t Phoenix_Com_Matrix_GetRows(const Matrix_t* ins);

/**
 * @brief 获取当前矩阵的列数
 * @param[in] ins 矩阵实例
 * @return 当前矩阵的列数
 */
Int32_t Phoenix_Com_Matrix_GetCols(const Matrix_t* ins);

/**
 * @brief 以向量索引的方式访问矩阵元素 \n
 *        (以列主序的顺序访问，建议矩阵定义为向量时可以使用此方式访问矩阵元素)
 * @param[in] ins 矩阵实例
 * @param[in] index 矩阵元素索引(列主序)
 * @return 矩阵元素索引所指向的数据
 */
Float32_t* Phoenix_Com_Matrix_GetEleByIdx_f(const Matrix_t* ins, Int32_t index);
Float64_t* Phoenix_Com_Matrix_GetEleByIdx_d(const Matrix_t* ins, Int32_t index);

/**
 * @brief 以矩阵行、列索引的形式访问矩阵元素
 * @param[in] ins 矩阵实例
 * @param[in] row 行号
 * @param[in] col 列号
 * @return 矩阵元素索引所指向的数据
 */
Float32_t* Phoenix_Com_Matrix_GetEle_f(
    const Matrix_t* ins, Int32_t row, Int32_t col);
Float64_t* Phoenix_Com_Matrix_GetEle_d(
    const Matrix_t* ins, Int32_t row, Int32_t col);

/**
 * @brief 将矩阵所有元素设置为0
 * @param[in] ins 矩阵实例
 */
void Phoenix_Com_Matrix_SetZeros_f(Matrix_t* ins);
void Phoenix_Com_Matrix_SetZeros_d(Matrix_t* ins);

/**
 * @brief 将矩阵所有元素设置为1
 * @param[in] ins 矩阵实例
 */
void Phoenix_Com_Matrix_SetOnes_f(Matrix_t* ins);
void Phoenix_Com_Matrix_SetOnes_d(Matrix_t* ins);

/**
 * @brief 将矩阵设置为单位矩阵
 * @param[in] ins 矩阵实例
 */
void Phoenix_Com_Matrix_SetIdentity_f(Matrix_t* ins);
void Phoenix_Com_Matrix_SetIdentity_d(Matrix_t* ins);

/**
 * @brief 将矩阵在内部转置
 * @warning {不会改变矩阵的存储结构，但必须留有足够的行和列 \n
 *           来进行转置操作，否则将导致严重错误}
 * @param[in] ins 矩阵实例
 */
void Phoenix_Com_Matrix_TransposeInPlace_f(Matrix_t* ins);
void Phoenix_Com_Matrix_TransposeInPlace_d(Matrix_t* ins);

/**
 * @brief 从数组输入数据到矩阵中
 * @param[in] ins 矩阵实例
 * @param[in] num 数据个数
 * @param[in] array 数组首地址
 */
void Phoenix_Com_Matrix_InputFromArray_f(
    Matrix_t* ins, Int32_t num, const Float32_t* array);
void Phoenix_Com_Matrix_InputFromArray_d(
    Matrix_t* ins, Int32_t num, const Float64_t* array);

/**
 * @brief 将矩阵内部数据输出到字符流
 * @param[in] ins 矩阵实例
 */
void Phoenix_Com_Matrix_Print_f(const Matrix_t* ins);
void Phoenix_Com_Matrix_Print_d(const Matrix_t* ins);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_MATH_MATRIX_C_H_

