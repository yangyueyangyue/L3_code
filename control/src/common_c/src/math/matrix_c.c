/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       matrix_c.c
 * @brief      矩阵运算
 * @details    定义相关矩阵运算算法
 *
 * @author     pengc
 * @date       2021.11.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "math/matrix_c.h"

#define ENABLE_MATRIX_C_TRACE (1)
#if ENABLE_MATRIX_C_TRACE
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#endif

#include "utils/log_c.h"


/*
 * @brief 获取矩阵元素的真实的存储索引
 * @param[in] row 行号
 * @param[in] col 列号
 * @return 矩阵元素的真实的存储索引
 */
static inline Int32_t GetStorageIndex(
    const Matrix_t* ins, Int32_t row, Int32_t col) {
  return ((ins->block_.start_col + col) * ins->max_rows_ +
          ins->block_.start_row + row);
}

/*
 * @brief 获取存储矩阵元素的地址的引用(Float32_t)
 * @param[in] row 行号
 * @param[in] col 列号
 * @return 存储矩阵元素的地址的引用
 */
static inline Float32_t* GetData_f(
    const Matrix_t* ins, Int32_t row, Int32_t col) {
  Float32_t* buf = (Float32_t*)(ins->storage_);
  return (&(buf[GetStorageIndex(ins, row, col)]));
}

/*
 * @brief 获取存储矩阵元素的地址的引用(Float64_t)
 * @param[in] row 行号
 * @param[in] col 列号
 * @return 存储矩阵元素的地址的引用
 */
static inline Float64_t* GetData_d(
    const Matrix_t* ins, Int32_t row, Int32_t col) {
  Float64_t* buf = (Float64_t*)(ins->storage_);
  return (&(buf[GetStorageIndex(ins, row, col)]));
}


/*
 * @brief 初始化矩阵
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_Init(
    Matrix_t* ins, Int32_t max_rows, Int32_t max_cols, void* buf) {
  ins->max_rows_ = max_rows;
  ins->max_cols_ = max_cols;
  ins->block_.rows = max_rows;
  ins->block_.cols = max_cols;
  ins->block_.start_row = 0;
  ins->block_.start_col = 0;
  ins->storage_ = buf;
}

/*
 * @brief 设置矩阵分块信息
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_SetBlock(
    Matrix_t* ins, Int32_t start_row, Int32_t start_col,
    Int32_t rows, Int32_t cols) {
  COM_CHECK_C((start_row >= 0) && (start_col >=0) && (rows > 0) && (cols > 0));
  COM_CHECK_C((start_row + rows) <= ins->max_rows_);
  COM_CHECK_C((start_col + cols) <= ins->max_cols_);

  ins->block_.start_row = start_row;
  ins->block_.start_col = start_col;
  ins->block_.rows = rows;
  ins->block_.cols = cols;
}

/*
 * @brief 获取当前矩阵的行数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_Matrix_GetRows(const Matrix_t* ins) {
  return (ins->block_.rows);
}

/*
 * @brief 获取当前矩阵的列数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_Com_Matrix_GetCols(const Matrix_t* ins) {
  return (ins->block_.cols);
}

/*
 * @brief 以向量索引的方式访问矩阵元素 \n
 *        (以列主序的顺序访问，建议矩阵定义为向量时可以使用此方式访问矩阵元素)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t* Phoenix_Com_Matrix_GetEleByIdx_f(
    const Matrix_t* ins, Int32_t index) {
  Float32_t* buf = (Float32_t*)(ins->storage_);
  Int32_t real_index =
      GetStorageIndex(ins, index % ins->block_.rows, index / ins->block_.rows);

  COM_CHECK_C(index >= 0);
  COM_CHECK_C(real_index < ins->max_rows_*ins->max_cols_);

  return (&(buf[real_index]));
}

Float64_t* Phoenix_Com_Matrix_GetEleByIdx_d(
    const Matrix_t* ins, Int32_t index) {
  Float64_t* buf = (Float64_t*)(ins->storage_);
  Int32_t real_index =
      GetStorageIndex(ins, index % ins->block_.rows, index / ins->block_.rows);

  COM_CHECK_C(index >= 0);
  COM_CHECK_C(real_index < ins->max_rows_*ins->max_cols_);

  return (&(buf[real_index]));
}

/*
 * @brief 以矩阵行、列索引的形式访问矩阵元素
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t* Phoenix_Com_Matrix_GetEle_f(
    const Matrix_t* ins, Int32_t row, Int32_t col) {
  Float32_t* buf = (Float32_t*)(ins->storage_);
  Int32_t real_index = GetStorageIndex(ins, row, col);

  COM_CHECK_C((row >= 0) && (col >= 0));
  COM_CHECK_C(real_index < ins->max_rows_*ins->max_cols_);

  return (&(buf[real_index]));
}

Float64_t* Phoenix_Com_Matrix_GetEle_d(
    const Matrix_t* ins, Int32_t row, Int32_t col) {
  Float64_t* buf = (Float64_t*)(ins->storage_);
  Int32_t real_index = GetStorageIndex(ins, row, col);

  COM_CHECK_C((row >= 0) && (col >= 0));
  COM_CHECK_C(real_index < ins->max_rows_*ins->max_cols_);

  return (&(buf[real_index]));
}

/*
 * @brief 将矩阵所有元素设置为0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_SetZeros_f(Matrix_t* ins) {
  Float32_t* buf = (Float32_t*)(ins->storage_);
  Int32_t real_index = 0;

  for (Int32_t j = 0; j < ins->block_.cols; ++j) {
    for (Int32_t i = 0; i < ins->block_.rows; ++i) {
      real_index = GetStorageIndex(ins, i, j);
      buf[real_index] = 0;
    }
  }
}

void Phoenix_Com_Matrix_SetZeros_d(Matrix_t* ins) {
  Float64_t* buf = (Float64_t*)(ins->storage_);
  Int32_t real_index = 0;

  for (Int32_t j = 0; j < ins->block_.cols; ++j) {
    for (Int32_t i = 0; i < ins->block_.rows; ++i) {
      real_index = GetStorageIndex(ins, i, j);
      buf[real_index] = 0;
    }
  }
}

/*
 * @brief 将矩阵所有元素设置为1
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_SetOnes_f(Matrix_t* ins) {
  Float32_t* buf = (Float32_t*)(ins->storage_);
  Int32_t real_index = 0;

  for (Int32_t j = 0; j < ins->block_.cols; ++j) {
    for (Int32_t i = 0; i < ins->block_.rows; ++i) {
      real_index = GetStorageIndex(ins, i, j);
      buf[real_index] = 1;
    }
  }
}

void Phoenix_Com_Matrix_SetOnes_d(Matrix_t* ins) {
  Float64_t* buf = (Float64_t*)(ins->storage_);
  Int32_t real_index = 0;

  for (Int32_t j = 0; j < ins->block_.cols; ++j) {
    for (Int32_t i = 0; i < ins->block_.rows; ++i) {
      real_index = GetStorageIndex(ins, i, j);
      buf[real_index] = 1;
    }
  }
}

/**
 * @brief 将矩阵设置为单位矩阵
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_SetIdentity_f(Matrix_t* ins) {
  Float32_t* buf = (Float32_t*)(ins->storage_);
  Int32_t real_index = 0;

  for (Int32_t j = 0; j < ins->block_.cols; ++j) {
    for (Int32_t i = 0; i < ins->block_.rows; ++i) {
      real_index = GetStorageIndex(ins, i, j);
      if (i == j) {
        buf[real_index] = 1;
      } else {
        buf[real_index] = 0;
      }
    }
  }
}

void Phoenix_Com_Matrix_SetIdentity_d(Matrix_t* ins) {
  Float64_t* buf = (Float64_t*)(ins->storage_);
  Int32_t real_index = 0;

  for (Int32_t j = 0; j < ins->block_.cols; ++j) {
    for (Int32_t i = 0; i < ins->block_.rows; ++i) {
      real_index = GetStorageIndex(ins, i, j);
      if (i == j) {
        buf[real_index] = 1;
      } else {
        buf[real_index] = 0;
      }
    }
  }
}

/*
 * @brief 将矩阵在内部转置
 * @warning {不会改变矩阵的存储结构，但必须留有足够的行和列 \n
 *           来进行转置操作，否则将导致严重错误}
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_TransposeInPlace_f(Matrix_t* ins) {
  Int32_t tmp_i = 0;
  Float32_t tmp_f = 0.0F;

  COM_CHECK_C((ins->block_.start_row + ins->block_.cols) <= ins->max_rows_);
  COM_CHECK_C((ins->block_.start_col + ins->block_.rows) <= ins->max_cols_);

  if (ins->block_.cols > ins->block_.rows) {
    for (Int32_t j = 1; j < ins->block_.rows; ++j) {
      for (Int32_t i = 0; i < j; ++i) {
        tmp_f = *GetData_f(ins, i, j);
        *GetData_f(ins, i, j) = *GetData_f(ins, j, i);
        *GetData_f(ins, j, i) = tmp_f;
      }
    }
    for (Int32_t j = ins->block_.rows; j < ins->block_.cols; ++j) {
      for (Int32_t i = 0; i < ins->block_.rows; ++i) {
        *GetData_f(ins, j, i) = *GetData_f(ins, i, j);
      }
    }
  } else if (ins->block_.cols < ins->block_.rows) {
    for (Int32_t j = 1; j < ins->block_.cols; ++j) {
      for (Int32_t i = 0; i < j; ++i) {
        tmp_f = *GetData_f(ins, i, j);
        *GetData_f(ins, i, j) = *GetData_f(ins, j, i);
        *GetData_f(ins, j, i) = tmp_f;
      }
    }
    for (Int32_t j = 0; j < ins->block_.cols; ++j) {
      for (Int32_t i = ins->block_.cols; i < ins->block_.rows; ++i) {
        *GetData_f(ins, j, i) = *GetData_f(ins, i, j);
      }
    }
  } else {
    for (Int32_t j = 1; j < ins->block_.rows; ++j) {
      for (Int32_t i = 0; i < j; ++i) {
        tmp_f = *GetData_f(ins, i, j);
        *GetData_f(ins, i, j) = *GetData_f(ins, j, i);
        *GetData_f(ins, j, i) = tmp_f;
      }
    }
  }

  tmp_i = ins->block_.rows;
  ins->block_.rows = ins->block_.cols;
  ins->block_.cols = tmp_i;
}

void Phoenix_Com_Matrix_TransposeInPlace_d(Matrix_t* ins) {
  Int32_t tmp_i = 0;
  Float64_t tmp_f = 0.0F;

  COM_CHECK_C((ins->block_.start_row + ins->block_.cols) <= ins->max_rows_);
  COM_CHECK_C((ins->block_.start_col + ins->block_.rows) <= ins->max_cols_);

  if (ins->block_.cols > ins->block_.rows) {
    for (Int32_t j = 1; j < ins->block_.rows; ++j) {
      for (Int32_t i = 0; i < j; ++i) {
        tmp_f = *GetData_d(ins, i, j);
        *GetData_d(ins, i, j) = *GetData_d(ins, j, i);
        *GetData_d(ins, j, i) = tmp_f;
      }
    }
    for (Int32_t j = ins->block_.rows; j < ins->block_.cols; ++j) {
      for (Int32_t i = 0; i < ins->block_.rows; ++i) {
        *GetData_d(ins, j, i) = *GetData_d(ins, i, j);
      }
    }
  } else if (ins->block_.cols < ins->block_.rows) {
    for (Int32_t j = 1; j < ins->block_.cols; ++j) {
      for (Int32_t i = 0; i < j; ++i) {
        tmp_f = *GetData_d(ins, i, j);
        *GetData_d(ins, i, j) = *GetData_d(ins, j, i);
        *GetData_d(ins, j, i) = tmp_f;
      }
    }
    for (Int32_t j = 0; j < ins->block_.cols; ++j) {
      for (Int32_t i = ins->block_.cols; i < ins->block_.rows; ++i) {
        *GetData_d(ins, j, i) = *GetData_d(ins, i, j);
      }
    }
  } else {
    for (Int32_t j = 1; j < ins->block_.rows; ++j) {
      for (Int32_t i = 0; i < j; ++i) {
        tmp_f = *GetData_d(ins, i, j);
        *GetData_d(ins, i, j) = *GetData_d(ins, j, i);
        *GetData_d(ins, j, i) = tmp_f;
      }
    }
  }

  tmp_i = ins->block_.rows;
  ins->block_.rows = ins->block_.cols;
  ins->block_.cols = tmp_i;
}

/*
 * @brief 从数组输入数据到矩阵中
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_InputFromArray_f(
    Matrix_t* ins, Int32_t num, const Float32_t* array) {
  Int32_t row_idx = 0;
  Int32_t col_idx = 0;

  for (Int32_t i = 0; i < num; ++i) {
    if (col_idx >= ins->block_.cols) {
      row_idx++;
      col_idx = 0;
      COM_CHECK_C(row_idx < ins->block_.rows);
    }
    COM_CHECK_C(col_idx < ins->block_.cols);
    *GetData_f(ins, row_idx, col_idx) = array[i];
    col_idx++;
  }
}

void Phoenix_Com_Matrix_InputFromArray_d(
    Matrix_t* ins, Int32_t num, const Float64_t* array) {
  Int32_t row_idx = 0;
  Int32_t col_idx = 0;

  for (Int32_t i = 0; i < num; ++i) {
    if (col_idx >= ins->block_.cols) {
      row_idx++;
      col_idx = 0;
      COM_CHECK_C(row_idx < ins->block_.rows);
    }
    COM_CHECK_C(col_idx < ins->block_.cols);
    *GetData_d(ins, row_idx, col_idx) = array[i];
    col_idx++;
  }
}

/*
 * @brief 将矩阵内部数据输出到字符流
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2022/11/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_Com_Matrix_Print_f(const Matrix_t* ins) {
#if ENABLE_MATRIX_C_TRACE
  for(Int32_t i = 0; i < ins->block_.rows; ++i) {
    printf("  ");
    printf("%0.6f", *GetData_f(ins, i, 0));
    for(Int32_t j = 1; j < ins->block_.cols; ++j) {
      printf(" ");
      printf("%0.6f", *GetData_f(ins, i, j));
    }
    if(i < (ins->block_.rows - 1)) {
      printf("\n");
    }
  }
  printf("\n");
#endif
}

void Phoenix_Com_Matrix_Print_d(const Matrix_t* ins) {
#if ENABLE_MATRIX_C_TRACE
  for(Int32_t i = 0; i < ins->block_.rows; ++i) {
    printf("  ");
    printf("%0.8f", *GetData_d(ins, i, 0));
    for(Int32_t j = 1; j < ins->block_.cols; ++j) {
      printf(" ");
      printf("%0.8f", *GetData_d(ins, i, j));
    }
    if(i < (ins->block_.rows - 1)) {
      printf("\n");
    }
  }
  printf("\n");
#endif
}

