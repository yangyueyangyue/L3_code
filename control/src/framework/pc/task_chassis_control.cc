//
#include "pc/task_chassis_control.h"

#include <stdlib.h>
#include <unistd.h>

#include "communication_c/shared_data_c.h"
#include "pc/util.h"

namespace phoenix {
namespace framework {


TaskChassisControl::TaskChassisControl(Task* manager)
    : Task(TASK_ID_CHASSIS_CONTROL, "Chassis Control", manager) {
  running_flag_chassis_control_ = false;

  control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;

  Phoenix_AdMsg_ClearChassis(&chassis_info_);
  Phoenix_AdMsg_ClearChassisCtlCmd(&chassis_ctl_cmd_);
}

TaskChassisControl::~TaskChassisControl() {
  Stop();
}

void TaskChassisControl::Configurate(const ChassisControlConfig_t& conf) {
  DoConfigurate(conf);
}

bool TaskChassisControl::Start() {
  if (running_flag_chassis_control_) {
    return (true);
  }

  LOG_INFO(3) << "Start CAN Access ...";
  if (!StartCanAccess()) {
    LOG_ERR << "Start CAN Access ... [NG]";
    return (false);
  }
  LOG_INFO(3) << "Start CAN Access ... [OK]";

  control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;

  LOG_INFO(3) << "Start Chassis Control Thread...";
  running_flag_chassis_control_ = true;
  thread_chassis_control_ = boost::thread(
      boost::bind(&TaskChassisControl::ThreadChassisControl, this));
  LOG_INFO(3) << "Start Chassis Control Thread ... [OK]";

  return (true);
}

bool TaskChassisControl::Stop() {
  if (running_flag_chassis_control_) {
    running_flag_chassis_control_ = false;

    LOG_INFO(3) << "Stop Chasssis Control Thread ...";
    bool ret = thread_chassis_control_.timed_join(
        boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failure in waiting for thread \"Chassis Control\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Chasssis Control Thread ... [OK]";
    } else {
      LOG_INFO(3) << "Stop Chasssis Control Thread ... [NG]";
    }

    LOG_INFO(3) << "Stop CAN Access ...";
    ret = StopCanAccess();
    if (false == ret) {
      LOG_ERR << "Failure in stop CAN Access.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop CAN Access ... [OK]";
    } else {
      LOG_INFO(3) << "Stop CAN Access ... [NG]";
    }
  }

  return (true);
}

Int32_t TaskChassisControl::GetControlStatus() {
  Lock lock(lock_control_status_);

  return (control_status_);
}

void TaskChassisControl::StartRobotCtl() {
  Lock lock(lock_control_status_);

  if (VEH_CHASSIS_CTL_STATUS_MANUAL == control_status_) {
    if (DoSomethingBeforeStartRobotCtl()) {
      control_status_ = VEH_CHASSIS_CTL_STATUS_REQ_ROBOTIC_CTL;
    }
  }
}

void TaskChassisControl::StopRobotCtl() {
  Lock lock(lock_control_status_);

  if ((VEH_CHASSIS_CTL_STATUS_REQ_ROBOTIC_CTL == control_status_) ||
      (VEH_CHASSIS_CTL_STATUS_WAITING_ROBOTIC_CTL_ACK == control_status_) ||
      (VEH_CHASSIS_CTL_STATUS_ROBOTIC == control_status_)
      ) {
    if (DoSomethingBeforeStopRobotCtl()) {
      control_status_ = VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL;
    }
  }
}

void TaskChassisControl::ThreadChassisControl() {
  LOG_INFO(3) << "Chassis Control Thread ... [Started]";

  Int32_t ret = REQ_OK;

  Phoenix_AdMsg_ClearChassis(&chassis_info_);
  Phoenix_AdMsg_ClearChassisCtlCmd(&chassis_ctl_cmd_);

  Dormancy dormancy(control_period_);
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Dormancy dormancy_idle(10);
#else
  Dormancy dormancy_idle(50);
#endif

  while (running_flag_chassis_control_) {
    Phoenix_SharedData_GetChassis(&chassis_info_);
    Phoenix_SharedData_GetChassisCtlCmd(&chassis_ctl_cmd_);

    lock_control_status_.lock();

    SendAlwaysNeededCmd(chassis_info_, chassis_ctl_cmd_);

    switch (control_status_) {
    case (VEH_CHASSIS_CTL_STATUS_MANUAL):
      SendCmdInManualStatus(chassis_info_, chassis_ctl_cmd_);
      break;
    case (VEH_CHASSIS_CTL_STATUS_REQ_ROBOTIC_CTL):
      ret = ReqStartRoboticCtl(chassis_info_, chassis_ctl_cmd_);
      if (REQ_OK == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_WAITING_ROBOTIC_CTL_ACK;
      } else if (REQ_WAIT == ret) {
        // waiting
      } else if (REQ_INTERRUPT == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL;
        LOG_INFO(3) << "Interrupt, Req Auto -> Req Manually";
      } else {
        control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
        LOG_ERR << "Request start robotic failed";
      }
      break;
    case (VEH_CHASSIS_CTL_STATUS_WAITING_ROBOTIC_CTL_ACK):
      ret = WaitStartRoboticCtlAck(chassis_info_, chassis_ctl_cmd_);
      if (REQ_OK == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_ROBOTIC;
      } else if (REQ_WAIT == ret) {
        // waiting
      } else if (REQ_INTERRUPT == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL;
        LOG_INFO(3) << "Interrupt, Wait Auto Ack -> Req Manually";
      } else {
        control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
        LOG_ERR << "Request wait start robotic ack failed";
      }
      break;
    case (VEH_CHASSIS_CTL_STATUS_ROBOTIC):
      ret = ReqSendRoboticCtlCmd(chassis_info_, chassis_ctl_cmd_);
      if (REQ_OK == ret) {
        // nothing to do
      } else if (REQ_INTERRUPT == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL;
        LOG_INFO(3) << "Interrupt, Auto -> Req Manually";
      } else {
        LOG_ERR << "Send robotic command failed";
        control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
      }
      break;
    case (VEH_CHASSIS_CTL_STATUS_REQ_MANUAL_CTL):
      ret = ReqStopRoboticCtl(chassis_info_, chassis_ctl_cmd_);
      if (REQ_OK == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_WAITING_MANUAL_CTL_ACK;
      } else if (REQ_WAIT == ret) {
        // waiting
      } else {
        LOG_ERR << "Request stop robotic failed";
        control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
      }
      break;
    case (VEH_CHASSIS_CTL_STATUS_WAITING_MANUAL_CTL_ACK):
      ret = WaitStopRoboticCtlAck(chassis_info_, chassis_ctl_cmd_);
      if (REQ_OK == ret) {
        control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
      } else if (REQ_WAIT == ret) {
        // waiting
      } else {
        // 开启手动失败
        control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
        LOG_ERR << "Request wait stop robotic ack failed";
      }
      break;
    default:
      control_status_ = VEH_CHASSIS_CTL_STATUS_MANUAL;
      LOG_ERR << "Vehicle status error";
      break;
    }

    Phoenix_SharedData_SetChassisCtlStatus(control_status_);

    bool is_idle = false;
    if (VEH_CHASSIS_CTL_STATUS_MANUAL == control_status_) {
      is_idle = true;
    }
    lock_control_status_.unlock();

    // send cmd delay
    if (is_idle) {
      dormancy_idle.Sleep();
    } else {
      dormancy.Sleep();
    }
  }

  LOG_INFO(3) << "Chassis Control Thread ... [Stopped]";
}

Int32_t TaskChassisControl::SendAlwaysNeededCmd(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  return (REQ_OK);
}

Int32_t TaskChassisControl::SendCmdInManualStatus(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  return (REQ_OK);
}


Int32_t TaskChassisControl::ReqStartRoboticCtl(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  LOG_ERR << "Enter Base Class function, ReqStartRoboticCtl";
  return (REQ_OK);
}

Int32_t TaskChassisControl::WaitStartRoboticCtlAck(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  LOG_ERR << "Enter Base Class function, WaitStartRoboticCtlAck";
  return (REQ_OK);
}

Int32_t TaskChassisControl::ReqSendRoboticCtlCmd(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  LOG_ERR << "Enter Base Class function, ReqSendRoboticCtlCmd";
  return (REQ_OK);
}

Int32_t TaskChassisControl::ReqStopRoboticCtl(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  LOG_ERR << "Enter Base Class function, ReqStopRoboticCtl";
  return (REQ_OK);
}

Int32_t TaskChassisControl::WaitStopRoboticCtlAck(
    const Chassis_t& status,
    const ChassisCtlCmd_t& cmd) {
  LOG_ERR << "Enter Base Class function, WaitStopRoboticCtlAck";
  return (REQ_OK);
}


bool TaskChassisControl::HandleMessage(const Message& msg, Task* sender) {
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

  default:
    break;
  }

  return (ret);
}


}  // namespace framework
}  // namespace phoenix
