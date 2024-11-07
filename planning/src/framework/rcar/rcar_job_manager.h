/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       rcar_job_manager.h
 * @brief      任务管理
 * @details    封装了任务管理功能的接口
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

#ifndef PHOENIX_FRAMEWORK_RCAR_JOB_MANAGER_H_
#define PHOENIX_FRAMEWORK_RCAR_JOB_MANAGER_H_


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化任务管理功能(包含了通讯、规划等各个功能模块)
 */
void RCAR_Job_Manager_Initialize();

/**
 * @brief 更新自定义时钟（定周期调用10ms）
 */
void RCAR_Job_Manager_UpdateUserClock();

/**
 * @brief 监控模块状态（定周期调用）
 */
void RCAR_Job_Manager_CheckModuleStatus();

/**
 * @brief 开始一次运动规划功能（定周期调用）
 */
void RCAR_Job_Manager_DoPlanningWork();

/**
 * @brief 向HMI模块发送相关的消息（定周期调用）
 */
void RCAR_Job_Manager_SendHmiMsg();


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_FRAMEWORK_RCAR_JOB_MANAGER_H_
