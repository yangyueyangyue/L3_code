/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       work_monitor.h
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

#ifndef PHOENIX_FRAMEWORK_WORK_MONITOR_C_H_
#define PHOENIX_FRAMEWORK_WORK_MONITOR_C_H_

#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct
 * @brief CAN线连接状态
 */
typedef struct _CanConnectStatus_t CanConnectStatus_t;
struct _CanConnectStatus_t {
  Int8_t cnt_timeout_can_0;
  Int8_t cnt_timeout_can_1;
  Int8_t cnt_timeout_can_2;
  Int8_t cnt_timeout_can_3;
};


/**
 * @brief 初始化状态监控模块
 */
void Phoenix_Work_Monitor_Initialize();

/**
 * @brief 开始一次模块监控功能（定周期调用）
 */
Int32_t Phoenix_Work_Monitor_DoWork();

/**
 * @brief 获取CAN线连接状态
 * @param[out] status ~ CAN线连接状态
 */
void Phoenix_Work_Monitor_GetCanConnectStatus(
    CanConnectStatus_t* const status);

/// 复位状态监控器
void Phoenix_Work_Monitor_FeedDog_RecvCanChannel0();
void Phoenix_Work_Monitor_FeedDog_RecvCanChannel1();
void Phoenix_Work_Monitor_FeedDog_RecvCanChannel2();
void Phoenix_Work_Monitor_FeedDog_RecvCanChannel3();
void Phoenix_Work_Monitor_FeedDog_RecvMsgImu();
void Phoenix_Work_Monitor_FeedDog_RecvMsgChassis();
void Phoenix_Work_Monitor_FeedDog_RecvMsgPlanningResult();
void Phoenix_Work_Monitor_FeedDog_RecvMsgDFCVSpecialChassisInfo();


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_FRAMEWORK_WORK_MONITOR_C_H_

