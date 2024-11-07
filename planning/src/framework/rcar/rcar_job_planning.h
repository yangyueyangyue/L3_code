/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       rcar_job_planning.h
 * @brief      运动规划
 * @details    封装了运动规划功能的接口
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

#ifndef PHOENIX_FRAMEWORK_RCAR_JOB_PLANNING_H_
#define PHOENIX_FRAMEWORK_RCAR_JOB_PLANNING_H_


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化运动规划功能
 */
void RCAR_Job_Planning_Initialize(void* module_monitor);

/**
 * @brief 开始一次运动规划功能（定周期调用）
 */
void RCAR_Job_Planning_DoJob();


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_FRAMEWORK_RCAR_JOB_PLANNING_H_
