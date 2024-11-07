/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_common.h
 * @brief      共通消息定义
 * @details    定义了共通消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_COMMON_C_H_
#define PHOENIX_AD_MSG_MSG_COMMON_C_H_

#include "utils/macros.h"
#include "utils/com_utils_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/// 消息顺序号的最大值（不能超过这个值）
enum { AD_MSG_SEQUENCE_NUM_MODULO = 10000 };

/**
 * @enum
 * @brief 模块ID
 */
enum {
  MODULE_ID_INVALID = 0,
  MODULE_ID_PERCEPTION,
  MODULE_ID_LOCALIZATION,
  MODULE_ID_MOTION_PLANNING,
  MODULE_ID_CONTROL,
  MODULE_ID_MONITOR,
  MODULE_ID_HMI
};

/**
 * @enum
 * @brief 子模块ID
 */
enum {
  SUB_MODULE_ID_INVALID = 0,

  /// Motion planning
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_HDMAP,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_ROUTING,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_GNSS,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_IMU,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_CHASSIS,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_SPECIAL_CHASSIS_INFO,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_CHASSIS_CTL_CMD,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_LANE_MARK_CAMERA_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMERA_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_LEFT_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_RIGHT_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_LEFT_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_RIGHT_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_LIDAR_LIST_0,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_LIDAR_LIST_1,
  SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OUTSIDE_OBSTACLE_LIST,
  SUB_MODULE_ID_MOTION_PLANNING_DO_WORK,
  SUB_MODULE_ID_MOTION_PLANNING_POS_FILTER,
  SUB_MODULE_ID_MOTION_PLANNING_OBJ_FILTER,
  SUB_MODULE_ID_MOTION_PLANNING_DRIVING_MAP,
  SUB_MODULE_ID_MOTION_PLANNING_ACTION_PLANNING,
  SUB_MODULE_ID_MOTION_PLANNING_TRAJECTORY_PLANNING,
  SUB_MODULE_ID_MOTION_PLANNING_VELOCITY_PLANNING,

  /// Control
  SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_0,
  SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_1,
  SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_2,
  SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_3,
  SUB_MODULE_ID_CONTROL_RECV_MSG_IMU,
  SUB_MODULE_ID_CONTROL_RECV_MSG_CHASSIS,
  SUB_MODULE_ID_CONTROL_RECV_MSG_PLANNING_RESULT,
  SUB_MODULE_ID_CONTROL_LAT_CTL,
  SUB_MODULE_ID_CONTROL_VEH_CTL,
  SUB_MODULE_ID_CONTROL_CHASSIS_CTL,

  SUM_MODULE_ID_MAX
};

/**
 * @enum
 * @brief 模块状态类型
 */
enum {
  MODULE_STATUS_OK = 0,
  MODULE_STATUS_ERR
};

/**
 * @struct MsgHead
 * @brief 消息头
 */
typedef struct _MsgHead_t MsgHead_t;
struct _MsgHead_t {
  /// 此报文是否有效
  Int8_t valid;
  /// 消息的顺序号，用于检索及同步消息报文
  Uint32_t sequence;
  /// 时间戳，用于时间同步
  Int64_t timestamp;
  /// 时间偏移量，用于补偿处理延迟
  Int32_t time_offset;
  /// 发送模块的ID
  Uint32_t src_module_id;
  /// 目标模块的ID
  Uint32_t dst_module_id;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearMsgHead(MsgHead_t* const msg_head);

/**
 * @brief 更新消息顺序号
 * @param[in] step 顺序号增加的值 \n
 *            注意：如果计数器当前值加上step大于Uint32_t可以表示的最大值 \n
 *                 将产生严重错误, 但程序内部不会做检查(为了效率)，调用者需要注意这一点。
 */
void Phoenix_AdMsg_UpdateSequenceNum(MsgHead_t* const msg_head, Uint32_t step);

/**
 * @brief 计算两个消息顺序号之间的差
 * @param[in] prev 之前消息的顺序号
 * @param[in] next 下一个消息的顺序号
 * @return 两个消息顺序号之间的差
 */
Uint32_t Phoenix_AdMsg_CalcSequenceDiff(Uint32_t prev, Uint32_t next);


/**
 * @struct ModuleStatus
 * @brief 模块状态
 */
typedef struct _ModuleStatus_t ModuleStatus_t;
struct _ModuleStatus_t {
  /// 子模块ID
  Int32_t sub_module_id;
  /// 模块状态类型
  Int32_t status;
  /// 模块是否超时
  Int8_t timeout;
  /// 自定义参数
  Int32_t param[4];
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearModuleStatus(ModuleStatus_t* const msg);


/**
 * @enum AD_MSG_MAX_MODULE_STATUS_NUM
 * @brief 列表中最大模块状态的数量
 */
enum { AD_MSG_MAX_MODULE_STATUS_NUM = 40 };

/**
 * @struct ModuleStatus
 * @brief 模块状态列表
 */
typedef struct _ModuleStatusList_t ModuleStatusList_t;
struct _ModuleStatusList_t {
  /// 消息头
  MsgHead_t msg_head;
  /// 模块状态的数量
  Int32_t module_status_num;
  /// 模块状态列表
  ModuleStatus_t module_status_list[AD_MSG_MAX_MODULE_STATUS_NUM];
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearModuleStatusList(ModuleStatusList_t* const msg);

/**
 * @brief 向模块状态列表尾部添加一个模块状态
 * @param[in] status 要添加的模块状态
 * @return 0 ~ 添加成功, -1 ~ 添加失败
 */
Int32_t Phoenix_AdMsg_PushBackModuleStausToList(
    const ModuleStatus_t* status, ModuleStatusList_t* const list);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_AD_MSG_MSG_COMMON_C_H_


