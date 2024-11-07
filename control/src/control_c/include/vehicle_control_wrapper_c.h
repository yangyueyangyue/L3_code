/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       vehicle_control_wrapper.h
 * @brief      车身控制器接口
 * @details    定义车身控制器接口
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

#ifndef PHOENIX_CONTROL_VEHICLE_CONTROL_WRAPPER_C_H_
#define PHOENIX_CONTROL_VEHICLE_CONTROL_WRAPPER_C_H_

#include "utils/macros.h"
#include "vehicle_control_c.h"
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
#include "Vehicle_Definiton.h"
#include "LgtCtrl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化车身控制器
 */
void Phoenix_VehCtl_Initialize();

/**
 * @brief 计算车身控制指令
 * @param[in] data_source 控制器需要的数据
 * @return 0 ~ 成功, others ~ 失败
 */
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_GOYU)
Int32_t Phoenix_VehCtl_CalcVehCtlValue(
    const VehCtlDataSource_t* data_source);
#elif (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
Int32_t Phoenix_VehCtl_CalcVehCtlValue(
    const VehCtlDataSource_t* data_source,
    LongtitudeControlInfo_t* const status_info);
#endif

/**
 * @brief 获取车身控制指令
 * @return 车身控制指令
 */
const ChassisCtlCmd_t* Phoenix_VehCtl_GetVehCtlCmd();


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_VEHICLE_CONTROL_WRAPPER_C_H_
