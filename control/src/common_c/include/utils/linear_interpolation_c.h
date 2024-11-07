/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       linear_interpolation.h
 * @brief      Linear interpolation functions.
 * @details
 *
 * @author     pengc
 * @date       2018.11.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/18  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_LINEAR_INTERPOLATION_C_H_
#define PHOENIX_COMMON_LINEAR_INTERPOLATION_C_H_


#include "utils/macros.h"
#include "math/matrix_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param x1 The coordinate of the second point.
 * @param t The interpolation parameter for interpolation.
 * @return Interpolated point.
 */
Float64_t Phoenix_Common_Lerp_d(
    const Float64_t x0, const Float64_t x1, const Float64_t t);

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param x1 The coordinate of the second point.
 * @param t The interpolation parameter for interpolation.
 * @return Interpolated point.
 */
Float32_t Phoenix_Common_Lerp_f(
    const Float32_t x0, const Float32_t x1, const Float32_t t);

/**
 * @brief 对角度进行线性插值
 * @param[in] from 起始角度
 * @param[in] to 终止角度
 * @param[in] t 在起始角度和终止角度之间的插值比例（应当在0和1之间）
 * @return 插值后的角度
 */
Float32_t Phoenix_Common_AngleLerp_f(Float32_t from, Float32_t to, Float32_t t);

/**
 * @brief 根据指定的键值在有序表中进行线性插值 \n
 *        (有序表中的键值必须从小到大进行排列，且不能有相同的键值)
 * @param[in] table 有序表
 * @param[in] key 指定的键值
 * @param[out] t 插值比例
 * @return 与键值对应的有序表中的索引（这个索引对应的有序表中的键值小于指定键值）
 */
Int32_t Phoenix_Common_LerpInOrderedTable_f(
    const Float32_t key_table[], const Int32_t key_table_size,
    Float32_t key, Float32_t* t);

/**
 * @brief 根据指定的键值在有序向量中进行线性插值 \n
 *        (有序向量中的键值必须从小到大进行排列，且不能有相同的键值)
 * @param[in] table 有序向量
 * @param[in] key 指定的键值
 * @param[out] t 插值比例
 * @return 与键值对应的有序向量中的索引（这个索引对应的有序向量中的键值小于指定键值）
 */
Int32_t Phoenix_Common_LerpInOrderedVector_f(
    const Matrix_t* key_table, Float32_t key, Float32_t* t);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_LINEAR_INTERPOLATION_C_H_
