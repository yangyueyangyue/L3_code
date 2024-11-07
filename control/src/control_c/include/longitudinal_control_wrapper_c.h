/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       longitudinal_control_wrapper.h
 * @brief      纵向控制器接口
 * @details    定义纵向控制器接口
 *
 * @author     pengc
 * @date       2021.07.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_CONTROL_LONGITUDINAL_CONTROL_WRAPPER_C_H_
#define PHOENIX_CONTROL_LONGITUDINAL_CONTROL_WRAPPER_C_H_

#include "utils/macros.h"
#include "longitudinal_control_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化纵向控制器
 */
void Phoenix_LonCtl_Initialize();

/**
 * @brief 重置纵向控制器内部状态(加速模块)
 * @param[in] value 加速量重置值
 */
void Phoenix_LonCtl_ResetAccValue(Float32_t value);

/**
 * @brief 重置纵向控制器内部状态(制动模块)
 * @param[in] value 制动量重置值
 */
void Phoenix_LonCtl_ResetBrakeValue(Float32_t value);

/**
 * @brief 计算纵向控制量
 * @param[in] data_source 控制器需要的数据
 * @param[out] acc_value 目标加速量
 * @param[out] brake_value 目标制动量
 * @return 0 ~ 成功, others ~ 失败
 */
Int32_t Phoenix_LonCtl_CalcLonCtlValue(
    const LonCtlDataSource_t* data_source,
    Float32_t* acc_value,
    Float32_t* brake_value);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_LONGITUDINAL_CONTROL_WRAPPER_C_H_
