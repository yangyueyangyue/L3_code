/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       vehicle_control_impl.h
 * @brief      车身控制器
 * @details    实现车身控制器
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

#ifndef PHOENIX_CONTROL_VEHICLE_CONTROL_IMPL_C_H_
#define PHOENIX_CONTROL_VEHICLE_CONTROL_IMPL_C_H_

#include "utils/macros.h"
#include "vehicle_control_c.h"
#include "msg_chassis_c.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct VehCtlImplInstance_t
 * @brief 车身控制器的成员变量
 */
typedef struct _VehCtlImplInstance_t VehCtlImplInstance_t;
struct _VehCtlImplInstance_t {
  /// 车身控制指令
  ChassisCtlCmd_t ctl_cmd;
};


/**
 * @brief 初始化车身控制器
 * @param[in] instance 成员变量
 */
void Phoenix_VehCtlImpl_Initialize(
    VehCtlImplInstance_t* const instance);

/**
 * @brief 计算车身控制指令
 * @param[in] instance 成员变量
 * @param[in] data_source 控制器需要的数据
 * @return 0 ~ 成功, others ~ 失败
 */
#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_GOYU)
Int32_t Phoenix_VehCtlImpl_CalcVehCtlValue(
    VehCtlImplInstance_t* const instance,
    const VehCtlDataSource_t* data_source);
#elif (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
Int32_t Phoenix_VehCtlImpl_CalcVehCtlValue(
    VehCtlImplInstance_t* const instance,
    const VehCtlDataSource_t* data_source,
    LongtitudeControlInfo_t* const status_info);
#endif

#ifdef __cplusplus
}
#endif


#endif // PHOENIX_CONTROL_VEHICLE_CONTROL_IMPL_C_H_
