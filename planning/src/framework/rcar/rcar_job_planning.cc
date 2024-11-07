/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       rcar_job_planning.cc
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

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "rcar/rcar_job_planning.h"

#include "work/work_planning.h"
#include "work/work_monitor.h"
#include "rcar/rcar_job_communication.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#ifdef __cplusplus
extern "C" {
#endif
#include "kotei_vehicle_platform_data_api.h"
#ifdef __cplusplus
}
#endif
#endif


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 运动规划功能的实现
static phoenix::framework::WorkPlanning s_work_planning;
// 模块监控
static phoenix::framework::WorkMonitor* s_module_monitor = Nullptr_t;


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化运动规划功能
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void RCAR_Job_Planning_Initialize(void* module_monitor) {

  // 设置模块监控器
  s_module_monitor = (phoenix::framework::WorkMonitor*)module_monitor;

  // 初始化运动规划模块
  s_work_planning.Initialize();
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
void RCAR_Job_Planning_DoJob() {

  phoenix::common::Stopwatch performance_timer;

  // 开始规划
  Int32_t status = s_work_planning.DoWork();

  Int32_t time_elapsed = performance_timer.Elapsed();

  // 将规划的结果发送到控制模块
  const phoenix::ad_msg::PlanningResult& planning_result =
      s_work_planning.GetPlanningResult();
  RCAR_Job_Communication_SendPlanningResult(&planning_result);

  // 更新规划模块状态
  s_module_monitor->FeedDog_CompletePlanning(status, time_elapsed);

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  // 更新定位状态
  if (s_work_planning.IsLocalizationUpdated()) {
    s_module_monitor->FeedDog_RecvGnss();
  }
  // 更新地图状态
  if (s_work_planning.IsMapUpdated()) {
    s_module_monitor->FeedDog_RecvHdMap();
  }
#endif

  // 发送模块状态
  RCAR_Job_Communication_SendModuleStatus();

  // 发送驾驶地图的状态
  RCAR_Job_Communication_SendDrivingMapStatus();

  // 发送事件通知
  RCAR_Job_Communication_SendEventReporting();
}

