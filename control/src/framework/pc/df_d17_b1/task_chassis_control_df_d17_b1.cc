//
#include "pc/df_d17_b1/task_chassis_control_df_d17_b1.h"

#include <stdlib.h>
#include <unistd.h>

#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "communication_c/shared_data_c.h"


namespace phoenix {
namespace framework {
namespace df_d17_b1 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)

TaskChassisControlDfD17B1::TaskChassisControlDfD17B1(Task* manager)
  : TaskChassisControl(manager) {
  req_start_robotic_ctl_count_ = 0;
  wait_start_robotic_ctl_ack_count_ = 0;
  req_stop_robotic_ctl_count_ = 0;
  wait_stop_robotic_ctl_ack_count_ = 0;

  send_EHPS_cycle_ = 0;
  send_XBL_cycle_ = 0;
  send_ADCU_cycle_ = 0;


  can_access_.reset(new df_d17_b1::CanAccessDfD17B1(this));

  common::com_memset(&steering_ctl_info_, 0, sizeof(steering_ctl_info_));
}

TaskChassisControlDfD17B1::~TaskChassisControlDfD17B1() {
}

bool TaskChassisControlDfD17B1::StartCanAccess() {
  return (can_access_->Start());
}

bool TaskChassisControlDfD17B1::StopCanAccess() {
  return (can_access_->Stop());
}

bool TaskChassisControlDfD17B1::DoSomethingBeforeStartRobotCtl() {
  req_start_robotic_ctl_count_ = 0;
  wait_start_robotic_ctl_ack_count_ = 0;
  req_stop_robotic_ctl_count_ = 0;
  wait_stop_robotic_ctl_ack_count_ = 0;

  return (true);
}

bool TaskChassisControlDfD17B1::DoSomethingBeforeStopRobotCtl() {
  return (true);
}

Int32_t TaskChassisControlDfD17B1::SendAlwaysNeededCmd(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;
  // 上电启动相关报文
  send_EHPS_cycle_ ++;
  if (send_EHPS_cycle_ == 2) {
    can_access_->SendFrame0x412();
  } else {
    send_EHPS_cycle_  = 1;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B1::ReqStartRoboticCtl(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;
  send_XBL_cycle_ = 0;
  send_ADCU_cycle_ = 0;

  if (req_start_robotic_ctl_count_ < 10) {
    req_start_robotic_ctl_count_++;

    ret = REQ_WAIT;
  } else {
    req_start_robotic_ctl_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B1::WaitStartRoboticCtlAck(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;

  if (wait_start_robotic_ctl_ack_count_ < 10) {
    can_access_->SendFrame0x413(cmd,0x02);
    wait_start_robotic_ctl_ack_count_++;

    ret = REQ_WAIT;
  } else {
    wait_start_robotic_ctl_ack_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B1::ReqSendRoboticCtlCmd(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Chassis_t chassis;
  Phoenix_SharedData_GetChassis(&chassis);
  Int32_t ret = REQ_OK;
  send_ADCU_cycle_ ++;
  send_XBL_cycle_ ++;

  // 上电发送报文
  if (send_ADCU_cycle_ >= 2) {
    can_access_->SendFrame0x0CFF649E(0x01, 0x01);
    send_ADCU_cycle_ = 0;
  }

  // 下发刹车时，周期为20ms，空闲时周期为200ms
  if (cmd.enable_ebs) {
    if (((cmd.brake_value > 0.5F) && (send_XBL_cycle_ >= 2)) ||
        ((cmd.brake_value <= 0.5F) && (send_XBL_cycle_ >= 20))) {
      // 制动下发报文
      can_access_->SendFrame0x0C040B9E(cmd, 0x02);
      send_XBL_cycle_ = 0;
    }
  }
  if (cmd.enable_eps) {
    // 转向角度请求时,0x413报文的4、5Byte需先发送0x0,待转向器完全开启后,在发送0x1与0x3,
    // 否则对于角度请求会产生4s延时
    // if(chassis.eps_status != VEH_EPS_STATUS_READY)
    //   can_access_->SendFrame0x413(cmd,0x02);
    // else
    can_access_->SendFrame0x413(cmd, 0x01);
  } else {
    can_access_->SendFrame0x413(cmd, 0x00);
  }

  // 油门档位下发报文
  if (cmd.enable_acc ) {
    can_access_->SendFrame0x0CFF659E(cmd, 0x01);
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B1::ReqStopRoboticCtl(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;
  send_ADCU_cycle_ ++;
  send_XBL_cycle_ ++;

  if (req_stop_robotic_ctl_count_ < 100) {
    req_stop_robotic_ctl_count_++;

    ChassisCtlCmd_t cmd_tmp = cmd;
    cmd_tmp.brake_value = 0;
    cmd_tmp.acc_value = 0;
    can_access_->SendFrame0x413(cmd,0x00);
    can_access_->SendFrame0x0CFF659E(cmd_tmp, 0x00);
    if (send_ADCU_cycle_ >= 2) {
      can_access_->SendFrame0x0CFF649E(0x01,0x01);
      send_ADCU_cycle_ = 0;
    }
    // 下发刹车时，周期为20ms，空闲时周期为200ms
    if (((cmd.brake_value > 0.5F) && (send_XBL_cycle_ >= 2)) ||
        ((cmd.brake_value <= 0.5F) && (send_XBL_cycle_ >= 20))) {
      // 制动下发报文
      can_access_->SendFrame0x0C040B9E(cmd, 0x00);
      send_XBL_cycle_ = 0;
    }
    ret = REQ_WAIT;
  } else {
    req_stop_robotic_ctl_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B1::WaitStopRoboticCtlAck(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;
  send_ADCU_cycle_++;
  if (wait_stop_robotic_ctl_ack_count_ < 10) {
    wait_stop_robotic_ctl_ack_count_++;
    if(send_ADCU_cycle_ >= 2){
      can_access_->SendFrame0x0CFF649E(0x00,0x00);
      send_ADCU_cycle_ = 0;
    }

    ret = REQ_WAIT;
  } else {
    wait_stop_robotic_ctl_ack_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

bool TaskChassisControlDfD17B1::HandleMessage(const Message& msg, Task* sender) {
  // Must be reentrant function

  bool ret = true;
  switch (msg.id()) {
  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_0): {
    Notify(msg);
  }
    break;

  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_1): {
    Notify(msg);
  }
    break;

  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_2): {
    Notify(msg);
  }
    break;

  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_3): {
    Notify(msg);
  }
    break;

  default:
    break;
  }

  return (ret);
}

#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)


}  // namespace df_d17_b1
}  // namespace framework
}  // namespace phoenix
