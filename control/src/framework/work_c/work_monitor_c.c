/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       work_monitor.c
 * @brief      模块状态监控
 * @details    封装了模块状态监控的接口
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
#include "work_c/work_monitor_c.h"


#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "os/mutex_c.h"
#include "math/math_utils_c.h"
#include "ad_msg_c.h"
#include "communication_c/shared_data_c.h"


/******************************************************************************/
/* 类型定义                                                                    */
/******************************************************************************/
typedef struct _Watchdog_t Watchdog_t;
struct _Watchdog_t {
  Int32_t max_foods;
  Int32_t foods;
};

typedef struct _Monitor_t Monitor_t;
struct _Monitor_t {
  CommonMutex_t mutex;

  Watchdog_t watchdog;
  Int64_t timer;
  Uint32_t counter;

  ModuleStatus_t module_status;
};

typedef struct _MonitorTable_t MonitorTable_t;
struct _MonitorTable_t {
  Monitor_t recv_can_ch_0;
  Monitor_t recv_can_ch_1;
  Monitor_t recv_can_ch_2;
  Monitor_t recv_can_ch_3;
  Monitor_t recv_msg_imu;
#if ENTER_PLAYBACK_MODE
  Monitor_t recv_msg_chassis;
#endif
  Monitor_t recv_msg_planning_result;
  Monitor_t recv_msg_dfcv_special_chassis_info;
};


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
static CanConnectStatus_t s_can_connect_status;
static MonitorTable_t s_monitor_table;
static ModuleStatusList_t s_module_status_list;


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/
static void Watchdog_Feed(Watchdog_t* dog) {
  dog->foods = dog->max_foods;
}

static Int32_t Watchdog_Walk(Watchdog_t* dog) {
  dog->foods--;
  if (dog->foods < 0) {
    dog->foods = 0;
  }
  return (dog->foods);
}

static Float32_t Watchdog_GetFoodsPercentage(Watchdog_t* dog) {
  return ((Float32_t)(dog->foods) / (Float32_t)(dog->max_foods));
}

static void Monitor_Initialize(
    Monitor_t* monitor, Int32_t module_id, Int32_t max_food) {
  Phoenix_Common_Os_Mutex_Init(&(monitor->mutex));

  monitor->watchdog.foods = 0;
  monitor->watchdog.max_foods = max_food;
  monitor->timer = Phoenix_Common_GetSysClockNowMs();
  monitor->counter = 0;

  Phoenix_AdMsg_ClearModuleStatus(&(monitor->module_status));
  monitor->module_status.sub_module_id = module_id;
}

static void Monitor_FeedDog(
    Monitor_t* monitor, Int32_t mode,
    Int32_t module_status,
    Int32_t param0, Int32_t param1, Int32_t param2) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(monitor->mutex));

  // Save module status
  monitor->module_status.status = module_status;
  monitor->module_status.param[1] = param0;
  monitor->module_status.param[2] = param1;
  monitor->module_status.param[3] = param2;

  // Analyzing Traffic
  Int64_t time_now = Phoenix_Common_GetSysClockNowMs();
  Int64_t time_elapsed =
      Phoenix_Common_CalcElapsedSysClockMs(monitor->timer, time_now);
  if (0 == mode) {
    // 两帧间隔
    monitor->module_status.param[0] = time_elapsed;
    // 重启定时器
    monitor->timer = time_now;
  } else if (1 == mode) {
    monitor->counter++;
    if (time_elapsed > 999) {
      // 两帧间隔
      monitor->module_status.param[0] = phoenix_com_round_d(
            (Float64_t)time_elapsed / monitor->counter);
      // 帧率 FPS
      // monitor->module_status.param[1] = monitor->counter / (time_elapsed/1000.0);
      // 重启定时器
      monitor->timer = time_now;
      monitor->counter = 0;
    }
  } else {
    // 两帧间隔
    monitor->module_status.param[0] = time_elapsed;
    // 重启定时器
    monitor->timer = time_now;
  }

  // Feed dog
  Watchdog_Feed(&(monitor->watchdog));

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(monitor->mutex));
}

static void Monitor_UpdateStatus(
    Monitor_t* monitor, ModuleStatus_t* module_status) {
  // Lock
  Phoenix_Common_Os_Mutex_Lock(&(monitor->mutex));

  // Get module status
  phoenix_com_memcpy(module_status, &(monitor->module_status),
                     sizeof(ModuleStatus_t));
  // Timeout ?
  Int32_t food = Watchdog_Walk(&(monitor->watchdog));
  if (food > 0) {
    monitor->module_status.timeout = 0;
    module_status->timeout = 0;
  } else {
    monitor->module_status.timeout = 1;
    module_status->timeout = 1;
  }

  // Unlock
  Phoenix_Common_Os_Mutex_Unlock(&(monitor->mutex));
}


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/
void Phoenix_Work_Monitor_Initialize() {
  phoenix_com_memset(&s_can_connect_status, 0, sizeof(s_can_connect_status));

  Monitor_Initialize(&(s_monitor_table.recv_can_ch_0),
                     SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_0, 2);
  Monitor_Initialize(&(s_monitor_table.recv_can_ch_1),
                     SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_1, 2);
  Monitor_Initialize(&(s_monitor_table.recv_can_ch_2),
                     SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_2, 2);
  Monitor_Initialize(&(s_monitor_table.recv_can_ch_3),
                     SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_3, 2);

  Monitor_Initialize(&(s_monitor_table.recv_msg_imu),
                     SUB_MODULE_ID_CONTROL_RECV_MSG_IMU, 4);
#if ENTER_PLAYBACK_MODE
  Monitor_Initialize(&(s_monitor_table.recv_msg_chassis),
                     SUB_MODULE_ID_CONTROL_RECV_MSG_CHASSIS, 4);
#endif
  Monitor_Initialize(&(s_monitor_table.recv_msg_planning_result),
                     SUB_MODULE_ID_CONTROL_RECV_MSG_PLANNING_RESULT, 6);
}

Int32_t Phoenix_Work_Monitor_DoWork() {
  Int32_t can0_timeout = 0;
  Int32_t can1_timeout = 0;
  Int32_t can2_timeout = 0;
  Int32_t can3_timeout = 0;
  ModuleStatus_t module_status;
  Phoenix_AdMsg_ClearModuleStatusList(&s_module_status_list);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  // can receive (ch 0)
  Monitor_UpdateStatus(&(s_monitor_table.recv_can_ch_0), &module_status);
  if (module_status.timeout) {
    can0_timeout = 1;
  } else {
    can0_timeout = 0;
  }
  Phoenix_AdMsg_PushBackModuleStausToList(
        &module_status, &s_module_status_list);

  #if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1 || \
       VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  // can receive (ch 1)
  Monitor_UpdateStatus(&(s_monitor_table.recv_can_ch_1), &module_status);
  if (module_status.timeout) {
    can1_timeout = 1;
  } else {
    can1_timeout = 0;
  }
  Phoenix_AdMsg_PushBackModuleStausToList(
        &module_status, &s_module_status_list);

  // can receive (ch 2)
  Monitor_UpdateStatus(&(s_monitor_table.recv_can_ch_2), &module_status);
  if (module_status.timeout) {
    can2_timeout = 1;
  } else {
    can2_timeout = 0;
  }
  Phoenix_AdMsg_PushBackModuleStausToList(
        &module_status, &s_module_status_list);
  #endif

  #if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1 || \
       VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  // can receive (ch 3)
  Monitor_UpdateStatus(&(s_monitor_table.recv_can_ch_3), &module_status);
  if (module_status.timeout) {
    can3_timeout = 1;
  } else {
    can3_timeout = 0;
  }
  Phoenix_AdMsg_PushBackModuleStausToList(
        &module_status, &s_module_status_list);
  #endif

#endif


#if ENTER_PLAYBACK_MODE
  // message receive (chassis)
  Monitor_UpdateStatus(&(s_monitor_table.recv_msg_chassis), &module_status);
  Phoenix_AdMsg_PushBackModuleStausToList(
        &module_status, &s_module_status_list);
#endif

  // message receive (planning result)
  Monitor_UpdateStatus(&(s_monitor_table.recv_msg_planning_result),
                       &module_status);
  Phoenix_AdMsg_PushBackModuleStausToList(
        &module_status, &s_module_status_list);

  // save
  s_can_connect_status.cnt_timeout_can_0 = can0_timeout;
  s_can_connect_status.cnt_timeout_can_1 = can1_timeout;
  s_can_connect_status.cnt_timeout_can_2 = can2_timeout;
  s_can_connect_status.cnt_timeout_can_3 = can3_timeout;
  Phoenix_SharedData_SetModuleStatusList(&s_module_status_list);

  return (0);
}

void Phoenix_Work_Monitor_GetCanConnectStatus(
    CanConnectStatus_t* const status) {
  phoenix_com_memcpy(status, &s_can_connect_status, sizeof(CanConnectStatus_t));
}


/// 复位状态监控器
///
void Phoenix_Work_Monitor_FeedDog_RecvCanChannel0() {
  Monitor_FeedDog(&(s_monitor_table.recv_can_ch_0), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}

void Phoenix_Work_Monitor_FeedDog_RecvCanChannel1() {
  Monitor_FeedDog(&(s_monitor_table.recv_can_ch_1), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}

void Phoenix_Work_Monitor_FeedDog_RecvCanChannel2() {
  Monitor_FeedDog(&(s_monitor_table.recv_can_ch_2), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}

void Phoenix_Work_Monitor_FeedDog_RecvCanChannel3() {
  Monitor_FeedDog(&(s_monitor_table.recv_can_ch_3), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}

void Phoenix_Work_Monitor_FeedDog_RecvMsgImu() {
  Monitor_FeedDog(&(s_monitor_table.recv_msg_imu), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}

void Phoenix_Work_Monitor_FeedDog_RecvMsgChassis() {
#if ENTER_PLAYBACK_MODE
  Monitor_FeedDog(&(s_monitor_table.recv_msg_chassis), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
#endif
}

void Phoenix_Work_Monitor_FeedDog_RecvMsgPlanningResult() {
  Monitor_FeedDog(&(s_monitor_table.recv_msg_planning_result), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}

void Phoenix_Work_Monitor_FeedDog_RecvMsgDFCVSpecialChassisInfo() {
  Monitor_FeedDog(&(s_monitor_table.recv_msg_dfcv_special_chassis_info), 0,
                  MODULE_STATUS_OK, 0, 0, 0);
}