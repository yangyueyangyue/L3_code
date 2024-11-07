/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       vehicle_control_wrapper.c
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

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "vehicle_control_wrapper_c.h"

#include "vehicle_control_impl_c.h"

#include "utils/macros.h"

/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 车身控制器的成员变量
static VehCtlImplInstance_t veh_ctl_impl_instance;

/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
* @brief 初始化车身控制器
*
* @par Change Log:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
* </table>
*/
void Phoenix_VehCtl_Initialize() {
  Phoenix_VehCtlImpl_Initialize(&veh_ctl_impl_instance);
}

/*
* @brief 计算车身控制指令
*
* @par Change Log:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
* </table>
*/
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_GOYU)
Int32_t Phoenix_VehCtl_CalcVehCtlValue(
    const VehCtlDataSource_t* data_source) {
  Phoenix_VehCtlImpl_CalcVehCtlValue(&veh_ctl_impl_instance, data_source);
}
#elif (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
Int32_t Phoenix_VehCtl_CalcVehCtlValue(
    const VehCtlDataSource_t* data_source,
    LongtitudeControlInfo_t* const status_info) {
  Phoenix_VehCtlImpl_CalcVehCtlValue(&veh_ctl_impl_instance, data_source, status_info);
}

#endif

/*
* @brief 获取车身控制指令
*
* @par Change Log:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
* </table>
*/
const ChassisCtlCmd_t* Phoenix_VehCtl_GetVehCtlCmd() {
  return (&veh_ctl_impl_instance.ctl_cmd);
}

