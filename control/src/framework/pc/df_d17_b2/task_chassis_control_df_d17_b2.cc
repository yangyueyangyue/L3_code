//
#include "pc/df_d17_b2/task_chassis_control_df_d17_b2.h"

#include <stdlib.h>
#include <unistd.h>

#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "communication_c/shared_data_c.h"


namespace phoenix {
namespace framework {
namespace df_d17_b2 {


#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)

TaskChassisControlDfD17B2::TaskChassisControlDfD17B2(Task* manager)
  : TaskChassisControl(manager) {
  req_start_robotic_ctl_count_ = 0;
  wait_start_robotic_ctl_ack_count_ = 0;
  req_stop_robotic_ctl_count_ = 0;
  wait_stop_robotic_ctl_ack_count_ = 0;
  ad_ecu_enable_ = 0x03;


  can_access_.reset(new df_d17_b2::CanAccessDfD17B2(this));
}

TaskChassisControlDfD17B2::~TaskChassisControlDfD17B2() {
}

void TaskChassisControlDfD17B2::DoConfigurate(const ChassisControlConfig_t& conf) {
  can_access_->Configurate(conf);
}

bool TaskChassisControlDfD17B2::StartCanAccess() {
  return (can_access_->Start());
}

bool TaskChassisControlDfD17B2::StopCanAccess() {
  return (can_access_->Stop());
}

bool TaskChassisControlDfD17B2::DoSomethingBeforeStartRobotCtl() {
  req_start_robotic_ctl_count_ = 0;
  wait_start_robotic_ctl_ack_count_ = 0;
  req_stop_robotic_ctl_count_ = 0;
  wait_stop_robotic_ctl_ack_count_ = 0;

  return (true);
}

bool TaskChassisControlDfD17B2::DoSomethingBeforeStopRobotCtl() {
  return (true);
}

Int32_t TaskChassisControlDfD17B2::SendAlwaysNeededCmd(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {

  if (2 == send_count_frame_0x0C040BDC_) {
    send_count_frame_0x0C040BDC_ = 0;
#if 0
    can_access_->SendFrame0x0C040BDC();
#endif
  }

  if (10 == send_count_frame_0x18FF769E_) {
    send_count_frame_0x18FF769E_ = 0;
    can_access_->SendFrame0x18FF769E();
  }

  if (10 == send_count_frame_0x18FF76DC_) {
    send_count_frame_0x18FF76DC_ = 0;
#if 0
    can_access_->SendFrame0x18FF76DC();
#endif
  }

  if (2 == send_count_frame_0x0CFF79DC_) {
    send_count_frame_0x0CFF79DC_ = 0;
#if 0
    can_access_->SendFrame0x0CFF79DC();
#endif
  }

#if 0
  can_access_->SendRetransmissionCanFrames();
#endif

  send_count_frame_0x0C040BDC_++;
  send_count_frame_0x18FF769E_++;
  send_count_frame_0x18FF76DC_++;
  send_count_frame_0x0CFF79DC_++;

  return (REQ_OK);
}

void TaskChassisControlDfD17B2::SendCtlCmd(
    Int8_t ctl_mode, Int8_t enable, Int8_t engage,
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  SpecialChassisInfo_t special_chassis;
  Phoenix_SharedData_GetSpecialChassisInfo(&special_chassis);

  if (2 == send_count_frame_0x0CFF649E_) {
    send_count_frame_0x0CFF649E_ = 0;
    can_access_->SendFrame0x0CFF649E(enable, engage);
  }
  send_count_frame_0x0CFF649E_++;

  if (2 == send_count_frame_0x0CFF64DC_) {
    send_count_frame_0x0CFF64DC_ = 0;
#if 0
    can_access_->SendFrame0x0CFF64DC(enable, engage);
#endif
  }
  send_count_frame_0x0CFF64DC_++;

  // 油门档位
  if (1 == send_count_frame_0x0CFF659E_) {
    send_count_frame_0x0CFF659E_ = 0;
    if (ctl_mode && cmd.enable_acc) {
      can_access_->SendFrame0x0CFF659E(cmd, 0x01);
    } else {
      can_access_->SendFrame0x0CFF659E(cmd, 0x00);
    }
  }
  send_count_frame_0x0CFF659E_++;

  // 制动
  // printf("send_count_frame_0x0C040B9E_ = %d,  ctl_mode=%d, enable_ebs=%d\n",
  //       send_count_frame_0x0C040B9E_, ctl_mode, cmd.enable_ebs);
  if (2 == send_count_frame_0x0C040B9E_) {
    send_count_frame_0x0C040B9E_ = 0;
    if (ctl_mode && cmd.enable_ebs) {

      can_access_->SendFrame0x0C040B9E(cmd, 0x02);
    } else {
      can_access_->SendFrame0x0C040B9E(cmd, 0x00);
    }
  }
  send_count_frame_0x0C040B9E_++;

  // 转向
  if (2 == send_count_frame_0x0CFF799E_) {
    send_count_frame_0x0CFF799E_ = 0;
    if (ctl_mode && cmd.enable_eps) {
      can_access_->SendFrame0x0CFF799E(cmd, 0x01, status, special_chassis);
    } else {
      can_access_->SendFrame0x0CFF799E(cmd, 0x00, status, special_chassis);
    }
  }
  send_count_frame_0x0CFF799E_++;
}

Int32_t TaskChassisControlDfD17B2::SendCmdInManualStatus(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  SpecialChassisInfo_t special_chassis;
  Phoenix_SharedData_GetSpecialChassisInfo(&special_chassis);
  if(((status.brake_pedal_value >= 25) && (0x03 == ad_ecu_enable_)) ||
     (0x01 == special_chassis.df_d17.EBS_state)) {
    ad_ecu_enable_ = 0x00;
  }

  SendCtlCmd(0, ad_ecu_enable_, 0x00, status, cmd);
}

Int32_t TaskChassisControlDfD17B2::ReqStartRoboticCtl(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;
  if(ad_ecu_enable_ != 0x03) {
    ad_ecu_enable_ = 0x01;
  }

  if (req_start_robotic_ctl_count_ < 10) {
    SendCtlCmd(0, ad_ecu_enable_, 0x00, status, cmd);

    req_start_robotic_ctl_count_++;

    ret = REQ_WAIT;
  } else {
    req_start_robotic_ctl_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B2::WaitStartRoboticCtlAck(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;

  if (wait_start_robotic_ctl_ack_count_ < 10) {
    SendCtlCmd(0, ad_ecu_enable_, 0x00, status, cmd);

    wait_start_robotic_ctl_ack_count_++;

    ret = REQ_WAIT;
  } else {
    wait_start_robotic_ctl_ack_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B2::ReqSendRoboticCtlCmd(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Chassis_t chassis;
  Phoenix_SharedData_GetChassis(&chassis);
  Int32_t ret = REQ_OK;

  SendCtlCmd(1, ad_ecu_enable_, 0x01, status, cmd);

  return (ret);
}

Int32_t TaskChassisControlDfD17B2::ReqStopRoboticCtl(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;

  if (req_stop_robotic_ctl_count_ < 100) {
    req_stop_robotic_ctl_count_++;

    ChassisCtlCmd_t cmd_tmp = cmd;
    cmd_tmp.brake_value = 0;
    cmd_tmp.acc_value = 0;

    SendCtlCmd(0, ad_ecu_enable_, 0x01, status, cmd);

    ret = REQ_WAIT;
  } else {
    req_stop_robotic_ctl_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

Int32_t TaskChassisControlDfD17B2::WaitStopRoboticCtlAck(
    const Chassis_t& status, const ChassisCtlCmd_t& cmd) {
  Int32_t ret = REQ_OK;

  if (wait_stop_robotic_ctl_ack_count_ < 10) {
    wait_stop_robotic_ctl_ack_count_++;

    SendCtlCmd(0, ad_ecu_enable_, 0x00, status, cmd);

    ret = REQ_WAIT;
  } else {
    wait_stop_robotic_ctl_ack_count_ = 0;
    ret = REQ_OK;
  }

  return (ret);
}

bool TaskChassisControlDfD17B2::HandleMessage(const Message& msg, Task* sender) {
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

#endif  // #if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)


}  // namespace df_d17_b2
}  // namespace framework
}  // namespace phoenix
