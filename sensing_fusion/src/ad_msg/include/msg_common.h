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

#ifndef PHOENIX_AD_MSG_MSG_COMMON_H_
#define PHOENIX_AD_MSG_MSG_COMMON_H_

#include "utils/macros.h"
#include "utils/com_utils.h"


namespace phoenix {
namespace ad_msg {


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
  SUB_MODULE_ID_MOTION_PLANNING_CALC_PLANNING_RET,
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
 * @struct MsgHead
 * @brief 消息头
 */
struct MsgHead {
  /// 消息顺序号的最大值（不能超过这个值）
  enum { MSG_SEQUENCE_NUM_MODULO = 10000 };

  /// 此报文是否有效
  bool valid;
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

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    valid = false;
    sequence = 0;
    timestamp = 0;
    time_offset = 0;
    src_module_id = MODULE_ID_INVALID;
    dst_module_id = MODULE_ID_INVALID;
  }

  /**
   * @brief 构造函数
   */
  MsgHead() {
    Clear();
  }

  /**
   * @brief 更新消息顺序号
   * @param[in] step 顺序号增加的值 \n
   *            注意：如果计数器当前值加上step大于Uint32_t可以表示的最大值 \n
   *                 将产生严重错误, 但程序内部不会做检查(为了效率)，调用者需要注意这一点。
   */
  void UpdateSequenceNum(Uint32_t step = 1) {
    sequence += step;
    if (sequence >= MSG_SEQUENCE_NUM_MODULO) {
      sequence -= MSG_SEQUENCE_NUM_MODULO;
    }
  }

  /**
   * @brief 计算两个消息顺序号之间的差
   * @param[in] prev 之前消息的顺序号
   * @param[in] next 下一个消息的顺序号
   * @return 两个消息顺序号之间的差
   */
  static Uint32_t CalcSequenceDiff(Uint32_t prev, Uint32_t next) {
    Uint32_t diff = 0;
    if (next >= prev) {
      diff = next - prev;
    } else {
      diff = MSG_SEQUENCE_NUM_MODULO - prev + next;
    }

    return (diff);
  }
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
 * @struct ModuleStatus
 * @brief 模块状态
 */
struct ModuleStatus {
  /// 子模块ID
  Int32_t sub_module_id;
  /// 模块状态类型
  Int32_t status;
  /// 模块是否超时
  Int8_t timeout;
  /// 自定义参数
  Int32_t param[4];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    sub_module_id = MODULE_ID_INVALID;
    status = MODULE_STATUS_OK;
    common::com_memset(param, 0, sizeof(param));
    timeout = false;
  }

  /**
   * @brief 构造函数
   */
  ModuleStatus() {
    Clear();
  }
};

struct ModuleStatusList {
  /**
   * @enum MAX_MODULE_STATUS_NUM
   * @brief 列表中最大模块状态的数量
   */
  enum { MAX_MODULE_STATUS_NUM = 20 };

  /// 消息头
  MsgHead msg_head;
  /// 模块状态的数量
  Int32_t module_status_num;
  /// 模块状态列表
  ModuleStatus module_status_list[MAX_MODULE_STATUS_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    module_status_num = 0;
    for (Int32_t i = 0; i < MAX_MODULE_STATUS_NUM; ++i) {
      module_status_list[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  ModuleStatusList() {
    Clear();
  }

  /**
   * @brief 向模块状态列表尾部添加一个模块状态
   * @param[in] status 要添加的模块状态
   * @return true ~ 添加成功, false ~ 添加失败
   */
  bool PushBackModuleStaus(const ModuleStatus& status) {
    bool ret = false;
    if (module_status_num < MAX_MODULE_STATUS_NUM) {
      module_status_list[module_status_num] = status;
      module_status_num++;

      ret = true;
    }

    return (ret);
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_COMMON_H_


