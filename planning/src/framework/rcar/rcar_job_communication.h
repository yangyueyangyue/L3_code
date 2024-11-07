/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       rcar_job_communication.h
 * @brief      跟外部模块通信
 * @details    接收外部模块发送的消息，向外部模块发送消息
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

#ifndef PHOENIX_FRAMEWORK_RCAR_JOB_COMMUNICATION_H_
#define PHOENIX_FRAMEWORK_RCAR_JOB_COMMUNICATION_H_


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化外部通信模块
 */
void RCAR_Job_Communication_Initialize(void* module_monitor);

/**
 * @brief 发送规划的结果
 */
void RCAR_Job_Communication_SendPlanningResult(const void* planning_ret);

/**
 * @brief 发送模块状态
 */
void RCAR_Job_Communication_SendModuleStatus();

/**
 * @brief 发送事件通知
 */
void RCAR_Job_Communication_SendEventReporting();

/**
 * @brief 发送驾驶地图的状态
 */
void RCAR_Job_Communication_SendDrivingMapStatus();

/**
 * @brief 向HMI模块发送相关的消息（定周期调用）
 */
void RCAR_Job_Communication_SendHmiMsg();


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_FRAMEWORK_RCAR_JOB_COMMUNICATION_H_

