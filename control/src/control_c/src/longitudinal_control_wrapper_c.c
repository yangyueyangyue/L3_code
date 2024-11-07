/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       longitudinal_control_wrapper.c
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

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "longitudinal_control_wrapper_c.h"

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
#include "df_d17_b1/longitudinal_control_df_d17_b1_c.h"
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
#include "df_d17_b2/longitudinal_control_df_d17_b2_c.h"
#else
  Error: Invalid vehicle platform.
#endif

/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 纵向控制器的成员变量
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
static LonCtlDfD17B1Instance_t s_lon_ctl_dfd17b1_instance;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
static LonCtlDfD17B2Instance_t s_lon_ctl_dfd17b2_instance;
#else
  Error: Invalid vehicle platform.;
#endif


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
* @brief 初始化纵向控制器
*
* @par Change Log:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
* </table>
*/
void Phoenix_LonCtl_Initialize() {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  Phoenix_LonCtl_DfD17B1_Initialize(&s_lon_ctl_dfd17b1_instance);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Phoenix_LonCtl_DfD17B2_Initialize(&s_lon_ctl_dfd17b2_instance);
#else
  Error: Invalid vehicle platform.
#endif
}

/*
 * @brief 重置纵向控制器内部状态(加速模块)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LonCtl_ResetAccValue(Float32_t value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  Phoenix_LonCtl_DfD17B1_ResetAccValue(&s_lon_ctl_dfd17b1_instance, value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Phoenix_LonCtl_DfD17B2_ResetAccValue(&s_lon_ctl_dfd17b2_instance, value);
#else
  Error: Invalid vehicle platform.
#endif
}

/*
 * @brief 重置纵向控制算法内部状态(制动模块)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LonCtl_ResetBrakeValue(Float32_t value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  Phoenix_LonCtl_DfD17B1_ResetBrakeValue(&s_lon_ctl_dfd17b1_instance, value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Phoenix_LonCtl_DfD17B2_ResetBrakeValue(&s_lon_ctl_dfd17b2_instance, value);
#else
  Error: Invalid vehicle platform.
#endif
}

/*
 * @brief 计算纵向控制量
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_LonCtl_CalcLonCtlValue(
    const LonCtlDataSource_t* data_source,
    Float32_t* acc_value,
    Float32_t* brake_value) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  Phoenix_LonCtl_DfD17B1_CalcLonCtlValue(
        &s_lon_ctl_dfd17b1_instance, data_source, acc_value, brake_value);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Phoenix_LonCtl_DfD17B2_CalcLonCtlValue(
        &s_lon_ctl_dfd17b2_instance, data_source, acc_value, brake_value);
#else
  Error: Invalid vehicle platform.
#endif
}

