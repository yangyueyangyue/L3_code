/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       rcar_job_manager.cc
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

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "rcar/rcar_job_manager.h"

#include "utils/macros.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "work/work_monitor.h"
#include "rcar/rcar_job_planning.h"
#include "rcar/rcar_job_communication.h"


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 模块监控功能
static phoenix::framework::WorkMonitor s_work_monitor;


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化任务管理功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Manager_Initialize() {
  // 初始化日志功能
  phoenix::common::InitializeLogging();
  // 配置日志选项
  phoenix::common::ConfigLogging(
        5,       // 一般性日志输出等级,对于跟踪日志及一般性日志，小于等于这个等级的才输出
        true,    // 是否输出时间信息
        true,    // 是否输出文件名称
        false,    // 是否输出到控制台
        true,   // 是否输出到环形缓冲区
        false    // 是否输出到文件
        );

  // Create triangle lookup tables
  phoenix::common::CreateTriangleLookupTables();

#if USING_USER_CLOCK
  // 初始化自定义时钟
  phoenix::common::InitializeUserClock();
#endif

  // 初始化模块监控功能
  s_work_monitor.Initialize();

  // 初始化外部通信模块
  RCAR_Job_Communication_Initialize(&s_work_monitor);

  // 初始化运动规划功能
  RCAR_Job_Planning_Initialize(&s_work_monitor);
}

/*
 * @brief 更新自定义时钟（定周期调用10ms）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Manager_UpdateUserClock() {
#if USING_USER_CLOCK
  phoenix::common::UpdateUserClockUs(10*1000);
#endif
}

/*
 * @brief 监控模块状态（定周期调用）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Manager_CheckModuleStatus() {
  // 检查一次各个模块的状态
  s_work_monitor.DoWork();
}

/*
 * @brief 开始一次运动规划功能（定周期调用）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Manager_DoPlanningWork() {
  // 开始规划
  RCAR_Job_Planning_DoJob();
}

/*
 * @brief 向HMI模块发送相关的消息（定周期调用）
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Manager_SendHmiMsg() {
  // 发送HMI消息
  RCAR_Job_Communication_SendHmiMsg();
}

