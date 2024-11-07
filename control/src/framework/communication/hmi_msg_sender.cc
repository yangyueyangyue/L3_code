/******************************************************************************
 ** 消息发送模块
 ******************************************************************************
 *
 *  消息发送模块(发送报文到总线上)
 *
 *  @file       msg_sender.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/hmi_msg_sender.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "utils/macros.h"

#if (ENABLE_ROS_NODE)
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#endif

#include "utils/com_utils.h"
#include "math/math_utils.h"

#include "data_serialization_c.h"
#include "communication_c/shared_data_c.h"


namespace phoenix {
namespace framework {


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      manager         (IN)           任务管理模块
 *              ros_node        (IN)           ros节点模块
 *
 *  @retval     None
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       None
 *
 */
HmiMsgSender::HmiMsgSender(Task* manager) :
  Task(TASK_ID_HMI_MSG_SENDER, "Message Sender", manager) {
#if (ENABLE_ROS_NODE)
  ros_node_ = Nullptr_t;
#endif

#if (ENABLE_LCM_NODE)
  lcm_node_ = Nullptr_t;
#endif

#if (ENABLE_UDP_NODE)
  udp_node_ = Nullptr_t;
#endif
}

/******************************************************************************/
/** 启动消息发送模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息发送模块，注册到总线上
 *
 *  <Attention>
 *       None
 *
 */
bool HmiMsgSender::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  if (Nullptr_t != ros_node_) {
    // empty
  }
#endif

#if (ENABLE_LCM_NODE)
  if (Nullptr_t != lcm_node_) {
    // empty
  }
#endif

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    // empty
  }
#endif

  return (ret);
}

void HmiMsgSender::SendHmiMsg() {
  Int32_t max_buff_size = sizeof(serialization_data_buff_) - 1;
  Int32_t data_size = 0;
  Int32_t data_len = 0;

#if (ENABLE_UDP_NODE)
  if (Nullptr_t == udp_node_) {
    return;
  }

  // send control module status
  data_size = sizeof(ModuleStatusList_t);
  if (data_size > max_buff_size) {
    LOG_ERR << "The size of serialization buffer is not enough.";
  } else {
    Phoenix_SharedData_GetModuleStatusList(&module_status_list_);
    data_len = Phoenix_Common_EncodeModuleStatusListArray(
          serialization_data_buff_, 0,
          max_buff_size, &module_status_list_, 1);
    udp_node_->Publish("hmi/control/module_status_list",
                       serialization_data_buff_, data_len);
  }

  // send control module status
  data_size = sizeof(LateralControlInfo_t);
  if (data_size > max_buff_size) {
    LOG_ERR << "The size of serialization buffer is not enough.";
  } else {
    Phoenix_SharedData_GetLateralControlInfo(&lateral_control_info_);
    data_len = Phoenix_Common_EncodeLateralControlInfoArray(
          serialization_data_buff_, 0,
          max_buff_size, &lateral_control_info_, 1);
    udp_node_->Publish("hmi/control/lat_ctl_info",
                       serialization_data_buff_, data_len);
  }
#endif  // #if (ENABLE_UDP_NODE)
}


}  // namespace framework
}  // namespace phoenix

