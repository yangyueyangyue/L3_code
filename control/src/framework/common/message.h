/******************************************************************************
 ** 消息定义
 ******************************************************************************
 *
 *  定义各种用于通讯的消息类型
 *
 *  @file       message.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MESSAGE_H_
#define PHOENIX_FRAMEWORK_MESSAGE_H_

#include <memory>
#include <vector>
#include "utils/macros.h"
#include "utils/com_utils.h"
#include "ad_msg_c.h"


namespace phoenix {
namespace framework {


/**
 * @enum MessageId
 * @brief 消息 ID
 */
enum MessageId {
  /// 无效值
  MSG_ID_INVALID = 0,
  /// 请求执行任务
  MSG_ID_REQ_DO_TASK,
  /// 惯性测量单元信息
  MSG_ID_IMU,
  /// 车身信息
  MSG_ID_CHASSIS,
  /// 二次规划结果
  MSG_ID_PLANNING_RESULT,
  /// 车身CAN Channel 0
  MSG_ID_MODULE_STATUS_CAN_RECV_CH_0,
  /// 车身CAN Channel 1
  MSG_ID_MODULE_STATUS_CAN_RECV_CH_1,
  /// 车身CAN Channel 2
  MSG_ID_MODULE_STATUS_CAN_RECV_CH_2,
  /// 车身CAN Channel 3
  MSG_ID_MODULE_STATUS_CAN_RECV_CH_3,
  /// 车身CAN Channel 4
  MSG_ID_MODULE_STATUS_CAN_RECV_CH_4,
  /// DFCV车身信息
  MSG_ID_DFCV_SPECIAL_CHASSIS_INFO
};

/**
 * @enum MessageId
 * @brief 任务 ID
 */
enum TaskId {
  /// 无效的任务ID
  TASK_ID_INVALID = 0,
  /// 任务管理
  TASK_ID_MANAGER,
  /// 接收外部消息
  TASK_ID_MSG_RECEIVER,
  /// 发送外部消息
  TASK_ID_MSG_SENDER,
  /// 发送外部HMI消息
  TASK_ID_HMI_MSG_SENDER,
  /// CAN 收发
  TASK_ID_CAN_ACCESS,
  /// 车身底盘控制
  TASK_ID_CHASSIS_CONTROL,
  /// 车辆控制
  TASK_ID_CONTROL
};


class Message {
public:
  explicit Message(Int32_t id) : id_(id) {}
  virtual ~Message() = default;

  Message(const Message& other) {
    id_ = other.id_;
  }

  void operator =(const Message& other) {
    id_ = other.id_;
  }

  Int32_t id() const { return (id_); }

private:
  Int32_t id_;
};

class MessageModuleStatus : public Message {
public:
  explicit MessageModuleStatus(Int32_t id) : Message(id) {
    status_ = 0;
    common::com_memset(param_, 0, sizeof(param_));
  }

  void set_status(Int32_t status) { status_ = status; }
  Int32_t status() const { return (status_); }

  void set_param0(Int32_t param) { param_[0] = param; }
  Int32_t param0() const { return (param_[0]); }
  void set_param1(Int32_t param) { param_[1] = param; }
  Int32_t param1() const { return (param_[1]); }

private:
  Int32_t status_;
  Int32_t param_[4];
};

class MessageImu : public Message {
public:
  MessageImu(const Imu_t* data) :
    Message(MSG_ID_IMU),
    imu_(data) {
  }
  inline const Imu_t* imu() const { return (imu_); }

private:
  const Imu_t* imu_ = Nullptr_t;
};

class MessageChassis : public Message {
public:
  MessageChassis(const Chassis_t* data) :
    Message(MSG_ID_CHASSIS),
    chassis_(data) {
  }
  inline const Chassis_t* chassis() const { return (chassis_); }

private:
  const Chassis_t* chassis_ = Nullptr_t;
};

class MessagePlanningResult : public Message {
public:
  explicit MessagePlanningResult(
      const PlanningResult_t* pl_ret) :
    Message(MSG_ID_PLANNING_RESULT) {
    planning_result_ = pl_ret;
  }

  const PlanningResult_t* planning_result() const {
    return planning_result_;
  }

private:
  const PlanningResult_t* planning_result_;
};

class MessageDFCVSpecialChassisInfo : public Message {
public:
  MessageDFCVSpecialChassisInfo(const DFCV_SpecialChassisInfo_t* data) :
    Message(MSG_ID_DFCV_SPECIAL_CHASSIS_INFO),
    dfcv_special_chassis_info_(data) {
  }
  inline const DFCV_SpecialChassisInfo_t* dfcv_special_chassis_info() const { return (dfcv_special_chassis_info_); }

private:
  const DFCV_SpecialChassisInfo_t* dfcv_special_chassis_info_ = Nullptr_t;
};

}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_MESSAGE_H_
