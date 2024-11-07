/******************************************************************************
 ** 任务管理模块
 ******************************************************************************
 *
 *  管理所有的任务(启动、停止、状态监测等)
 *
 *  @file       task_manager.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "task_manager.h"
#include <stdlib.h>
#include <unistd.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "boost/date_time.hpp"
#if (ENABLE_ROS_NODE)
#include "communication/ros_node.h"
#include "control/control_debug.h"
#endif
#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif
#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif
#include "communication/msg_receiver.h"
#include "communication/msg_sender.h"
#include "communication/hmi_msg_sender.h"
#include "communication_c/shared_data_c.h"
#include "utils/log.h"
#include "utils/com_utils.h"
#include "utils/gps_tools.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"

#include "task_control.h"

#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
#include "pc/df_d17_b1/task_chassis_control_df_d17_b1.h"
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
#include "pc/df_d17_b2/task_chassis_control_df_d17_b2.h"
#else
  Error: Invalid vehicle platform
#endif

namespace phoenix {
namespace framework {


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      argc            (in)           输入参数
 *              argv            (in)           输入参数
 *              work_space      (in)           工作空间目录
 *
 *  @retval     none
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
TaskManager::TaskManager(Int32_t argc, Char_t** argv,
                         const std::string &work_space)
  : Task(TASK_ID_MANAGER, "Task Manager")
  , work_space_(work_space) {
#if USING_USER_CLOCK
  thread_running_flag_update_user_clock_ = false;
#endif
  thread_running_flag_check_tasks_status_ = false;
  thread_running_flag_send_data_hmi_ = false;

#if (ENABLE_ROS_NODE)
  ros_node_.reset(new RosNode(argc, argv, "control"));
#endif
#if (ENABLE_LCM_NODE)
  lcm_node_.reset(new LcmNode("udpm://239.255.76.67:7667?ttl=1"));
#endif
#if (ENABLE_UDP_NODE)
  udp_node_.reset(new UdpNode());
#endif

  msg_receiver_.reset(new MsgReceiver(this));
  msg_sender_.reset(new MsgSender(this));
  Int32_t communication_mode = 0;
  if (0 == communication_mode) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(ros_node_.get());
    msg_sender_->SetRosNode(ros_node_.get());
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(Nullptr_t);
    msg_sender_->SetUdpNode(Nullptr_t);
#endif
  } else if (1 == communication_mode) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(Nullptr_t);
    msg_sender_->SetRosNode(Nullptr_t);
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(lcm_node_.get());
    msg_sender_->SetLcmNode(lcm_node_.get());
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(Nullptr_t);
    msg_sender_->SetUdpNode(Nullptr_t);
#endif
  } else if (2 == communication_mode) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(Nullptr_t);
    msg_sender_->SetRosNode(Nullptr_t);
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(udp_node_.get());
    msg_sender_->SetUdpNode(udp_node_.get());
#endif
  } else {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(Nullptr_t);
    msg_sender_->SetRosNode(Nullptr_t);
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(Nullptr_t);
    msg_sender_->SetUdpNode(Nullptr_t);
#endif
    LOG_ERR << "Invalid communication mode.";
  }

  hmi_msg_sender_.reset(new HmiMsgSender(this));
#if (ENABLE_ROS_NODE)
  hmi_msg_sender_->SetRosNode(Nullptr_t);
#endif
#if (ENABLE_LCM_NODE)
  hmi_msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
  hmi_msg_sender_->SetUdpNode(udp_node_.get());
#endif

#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  task_chassis_control_.reset(new df_d17_b1::TaskChassisControlDfD17B1(this));
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  task_chassis_control_.reset(new df_d17_b2::TaskChassisControlDfD17B2(this));
#else
  Error: Invalid vehicle platform
#endif

  task_control_.reset(new TaskControl(this, task_chassis_control_.get()));

  Phoenix_Common_ClearControlConfig(&config_);

  // 初始化共享内存模块
  Phoenix_SharedData_Initialize();
  // 初始化状态监控模块
  Phoenix_Work_Monitor_Initialize();
}

/******************************************************************************/
/** 析构函数
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       析构函数
 *
 *  <Attention>
 *       None
 *
 */
TaskManager::~TaskManager() {
  Stop();
}

/******************************************************************************/
/** 启动各个模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动各个模块
 *
 *  <Attention>
 *       None
 *
 */
bool TaskManager::Start() {
  bool ret = false;

  if (!ReadConfigFromFile()) {
    LOG_ERR << "Failed to read config from file.";
  }

#if USING_USER_CLOCK
  // Start Thread(Update user clock)
  if (true != thread_running_flag_update_user_clock_) {
    LOG_INFO(3) << "Start updating user clock Thread ...";
    common::InitializeUserClock();
    thread_running_flag_update_user_clock_ = true;
    thread_update_user_clock_ =
        boost::thread(boost::bind(&TaskManager::ThreadUpdateUserClock, this));
    LOG_INFO(3) << "Start update user clock Thread ... [OK]";
  }
#endif

#if (ENABLE_ROS_NODE)
  // Start ROS Node
  ret = ros_node_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start ROS node.";
    return (false);
  }
#endif

#if (ENABLE_LCM_NODE)
  // Start LCM Node
  ret = lcm_node_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start LCM node.";
    return (false);
  }
#endif

#if (ENABLE_UDP_NODE)
  // Start UDP Node
  UdpNode::UdpParam udp_param;
  udp_param.enable_recv = true;
  udp_param.enable_send = true;
#if 0
  udp_param.rcv_port = 6001;
  udp_param.snd_port = 6002;
  Int32_t mode = 0;
#else
  udp_param.rcv_port = 9500;
  udp_param.snd_port = 9500;
  Int32_t mode = 1;
#endif
  if (0 == mode) {
    udp_param.mode = UdpNode::UdpParam::MODE_UNICAST;
    strncpy(udp_param.snd_addr, "192.168.1.100", sizeof(udp_param.snd_addr)-1);
  } else if (1 == mode) {
    udp_param.mode = UdpNode::UdpParam::MODE_GROUP;
    strncpy(udp_param.snd_addr, "224.10.10.2", sizeof(udp_param.snd_addr)-1);
  } else {
    udp_param.mode = UdpNode::UdpParam::MODE_BROADCAST;
    //strncpy(udp_param.snd_addr, "192.168.10.102", sizeof(udp_param.snd_addr)-1);
    strncpy(udp_param.snd_addr, "192.168.1.255", sizeof(udp_param.snd_addr)-1);
  }
  ret = udp_node_->Start(udp_param);
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start UDP node.";
    return (false);
  }
#endif

  // Start Message receive
  ret = msg_receiver_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message receiver.";
    return false;
  }

  // Start Message send
  ret = msg_sender_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message sender.";
    return false;
  }

  // Start HMI Message send
  ret = hmi_msg_sender_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start hmi message sender.";
    return false;
  }

#if (ENABLE_UDP_NODE)
  ret = udp_node_->StartReceiving();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start UDP receiving.";
    return false;
  }
#endif

#if !ENTER_PLAYBACK_MODE
  // Start Chassis Control Task
  ret = task_chassis_control_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start chassis control task.";
    return false;
  }
#endif

  // Start Control Task
  ret = task_control_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start control task.";
    return false;
  }

  // Start Thread(Check tasks status)
  if (true != thread_running_flag_check_tasks_status_) {
    LOG_INFO(3) << "Start Check Tasks Status Thread ...";
    thread_running_flag_check_tasks_status_ = true;
    thread_check_tasks_status_ =
        boost::thread(boost::bind(&TaskManager::ThreadCheckTasksStatus, this));
    LOG_INFO(3) << "Start Check Tasks Status Thread ... [OK]";
  }

  if (true != thread_running_flag_send_data_hmi_){
    LOG_INFO(3) << "Start Send Data To HMI Thread ...";
    thread_running_flag_send_data_hmi_ = true;
    thread_send_data_hmi_ =
        boost::thread(boost::bind(&TaskManager::ThreadSendDataToHmi, this));
    LOG_INFO(3) << "Start Send Data To HMI Thread ... [OK]";
  }

  return (true);
}

/******************************************************************************/
/** 停止各个模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       停止各个模块
 *
 *  <Attention>
 *       None
 *
 */
bool TaskManager::Stop() {
  if (true == thread_running_flag_send_data_hmi_){
    LOG_INFO(3) << "Stop Send Data To HMI Thread ...";
    thread_running_flag_send_data_hmi_ = false;
    bool ret = thread_send_data_hmi_.timed_join(boost::posix_time::seconds(2));
    if(false == ret){
      LOG_ERR << "Failed to wait for thread \"Send Data To HMI\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Send Data To HMI Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Send Data To HMI Thread... [NG]";
    }
  }

  // Stop Thread(Check tasks status)
  if (true == thread_running_flag_check_tasks_status_) {
    LOG_INFO(3) << "Stop Check Tasks Status Thread ...";
    thread_running_flag_check_tasks_status_ = false;
    bool ret =
        thread_check_tasks_status_.timed_join(boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait for thread \"Check tasks status\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Check Tasks Status Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Check Tasks Status Thread... [NG]";
    }
  }

  // Stop control task
  task_control_->Stop();
#if !ENTER_PLAYBACK_MODE
  // Stop chassis control task
  task_chassis_control_->Stop();
#endif

#if (ENABLE_UDP_NODE)
  // Stop UDP Node
  udp_node_->Stop();
#endif
#if (ENABLE_LCM_NODE)
  // Stop LCM Node
  lcm_node_->Stop();
#endif
#if (ENABLE_ROS_NODE)
  // Stop ROS Node
  ros_node_->Stop();
#endif

#if USING_USER_CLOCK
  // Stop Thread(Update user clock)
  if (true == thread_running_flag_update_user_clock_){
    LOG_INFO(3) << "Stop updating user clock Thread ...";
    thread_running_flag_update_user_clock_ = false;
    bool ret = thread_update_user_clock_.timed_join(boost::posix_time::seconds(2));
    if(false == ret){
      LOG_ERR << "Failed to wait for thread \"Update user clock\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop updating user clock Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop updating user clock Thread... [NG]";
    }
  }
#endif

  return (true);
}

void TaskManager::StartRobotCtl() {
  task_control_->StartRobotCtl();
}

void TaskManager::StopRobotCtl() {
  task_control_->StopRobotCtl();
}

void TaskManager::EnableDirectCtl(bool enable) {
  task_control_->EnableDirectCtl(enable);
}

void TaskManager::EnableRemoteCtl(bool enable) {
  task_control_->EnableRemoteCtl(enable);
}

void TaskManager::EnableEps(bool enable) {
  task_control_->EnableEps(enable);
}

void TaskManager::EnableThrottleSys(bool enable) {
  task_control_->EnableThrottleSys(enable);
}

void TaskManager::EnableEbs(bool enable) {
  task_control_->EnableEbs(enable);
}

void TaskManager::TurnSteeringWheel(Float32_t angle, Float32_t speed) {
  task_control_->TurnSteeringWheel(angle, speed);
}

void TaskManager::SpeedUp(Float32_t velocity, Float32_t acc) {
  task_control_->SpeedUp(velocity, acc);
}

void TaskManager::Accelerate(Float32_t value) {
  task_control_->Accelerate(value);
}

void TaskManager::Brake(Float32_t value) {
  task_control_->Brake(value);
}

void TaskManager::ChangeGear(Uint8_t value) {
  task_control_->ChangeGear(value);
}

void TaskManager::SetTurnLamp(Uint8_t value) {
  task_control_->SetTurnLamp(value);
}

void TaskManager::SetWiper(Uint8_t value) {
  task_control_->SetWiper(value);
}

void TaskManager::ReqManualInterrupt(bool enable) {
  task_control_->ReqManualInterrupt(enable);
}

#if USING_USER_CLOCK
void TaskManager::ThreadUpdateUserClock() {
  LOG_INFO(3) << "Updating user clock Thread ... [Started]";

  Dormancy dormancy(10);
  while (true == thread_running_flag_update_user_clock_) {
    common::UpdateUserClockUs(10*1000);

    dormancy.Sleep();
  }

  LOG_INFO(3) << "Updating user clock Thread ... [Stopped]";
}
#endif


/******************************************************************************/
/** 监测各个模块的状态
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       监测各个模块的状态
 *
 *  <Attention>
 *       None
 *
 */
void TaskManager::ThreadCheckTasksStatus() {
  LOG_INFO(3) << "Check Tasks Status Thread ... [Started]";
  PlanningResult_t planning_info;
  Chassis_t chassis_info;
  SpecialChassisInfo_t special_chassis_info;
  ChassisCtlCmd_t chassis_cmd;
  CanConnectStatus_t can_connect_status;

  control::control_debug ctrl_debug;
  LateralControlInfo_t lat_ctl_info;
  Dormancy dormancy(50);
  while (true == thread_running_flag_check_tasks_status_) {
    Phoenix_SharedData_GetChassis(&chassis_info);
    Phoenix_SharedData_GetSpecialChassisInfo(&special_chassis_info);
    Phoenix_SharedData_GetChassisCtlCmd(&chassis_cmd);
    Phoenix_SharedData_GetPlanningResult(&planning_info);

#if (CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV)
    DFCV_SpecialChassisInfo_t dfcv_special_chassis;
    LongtitudeControlInfo_t longtitude_control;
    Phoenix_SharedData_GetDFCVSpecialChassisInfo(&dfcv_special_chassis);
    Phoenix_SharedData_GetLongtitudeControlInfo(&longtitude_control);
#endif
#if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
    LateralControlInfo_t lateral_control;
    Phoenix_SharedData_GetLateralControlInfo(&lateral_control);
#endif
  Phoenix_SharedData_GetLateralControlInfo(&lat_ctl_info);
    // TODO: lateral control message
    // Phoenix_SharedData_GetLateralControlInfo

    // TDDO: planning result message for control to debug
    // Phoenix_SharedData_GetPlanningResult

    Phoenix_Work_Monitor_DoWork();

    Phoenix_Work_Monitor_GetCanConnectStatus(&can_connect_status);

#if (!ENTER_PLAYBACK_MODE)

    Int32_t can_timeout = can_connect_status.cnt_timeout_can_0;

    Int32_t chassis_ctl_status = VEH_CHASSIS_CTL_STATUS_MANUAL;
    Phoenix_SharedData_GetChassisCtlStatus(&chassis_ctl_status);
    if (!can_timeout) {
      // 驾驶模式
      if (Phoenix_AdMsg_IsChassisInRoboticMode(chassis_ctl_status)) {
        if (chassis_cmd.enable_throttle_sys) {
          chassis_info.throttle_sys_status = VEH_THROTTLE_SYS_STATUS_ROBOTIC;
        } else {
          chassis_info.throttle_sys_status = VEH_THROTTLE_SYS_STATUS_MANUAL;
        }
        if (chassis_cmd.enable_ebs) {
          chassis_info.ebs_status = VEH_EBS_STATUS_ROBOTIC;
        } else {
          chassis_info.ebs_status = VEH_EBS_STATUS_MANUAL;
        }

        if (!chassis_cmd.enable_eps &&
            !chassis_cmd.enable_throttle_sys &&
            !chassis_cmd.enable_ebs) {
          chassis_info.driving_mode = VEH_DRIVING_MODE_MANUAL;
        } else {
          chassis_info.driving_mode = VEH_DRIVING_MODE_ROBOTIC;
        }
      } else {
        chassis_info.driving_mode = VEH_DRIVING_MODE_MANUAL;
        chassis_info.eps_status = VEH_EPS_STATUS_MANUAL;
        chassis_info.throttle_sys_status = VEH_THROTTLE_SYS_STATUS_MANUAL;
        chassis_info.ebs_status = VEH_EBS_STATUS_MANUAL;
      }
      msg_sender_->SendChassis(chassis_info);
      msg_sender_->SendChassisCtlCmd(chassis_cmd);

      // Get Planning Input message
      #if ((CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV) || (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV))
      ctrl_debug.planning_input.SpdPln_vTrgSpd_mp = chassis_cmd.velocity;
      ctrl_debug.planning_input.SpdPln_aTrgAcc_mp = chassis_cmd.acceleration;
      ctrl_debug.planning_input.BhvCrdn_numBhvID_mp = longtitude_control.debug_AD_Status;
      ctrl_debug.planning_input.VehDa_rSlop_mp = planning_info.ramp_slope_value;
      
      #if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
      ctrl_debug.planning_input.LaneTraj_IsValid =  planning_info.tar_trj.LaneTraj_Cv.LaneTraj_IsValid;
      ctrl_debug.planning_input.LATC_LineCv0 = planning_info.tar_trj.LaneTraj_Cv.c_center[0];
      ctrl_debug.planning_input.LATC_LineCv1 = planning_info.tar_trj.LaneTraj_Cv.c_center[1];
      ctrl_debug.planning_input.LATC_LineCv2 = planning_info.tar_trj.LaneTraj_Cv.c_center[2];
      ctrl_debug.planning_input.LATC_LineCv3 = planning_info.tar_trj.LaneTraj_Cv.c_center[3];
      ctrl_debug.planning_input.LATC_LineCv4 = 0.0;
      ctrl_debug.planning_input.LATC_LineCv5 = 0.0;

      ctrl_debug.planning_input.PlanningTraj_IsValid = planning_info.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;
      ctrl_debug.planning_input.LATC_cTgtPathCv0 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[0];
      ctrl_debug.planning_input.LATC_cTgtPathCv1 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[1];
      ctrl_debug.planning_input.LATC_cTgtPathCv2 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[2];
      ctrl_debug.planning_input.LATC_cTgtPathCv3 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[3];
      ctrl_debug.planning_input.LATC_cTgtPathCv4 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[4];
      ctrl_debug.planning_input.LATC_cTgtPathCv5 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[5];
      #endif
      // Get chassis Input message

      ctrl_debug.chassis_input.VehDa_rBrkPedl_mp = chassis_info.brake_pedal_value;
      ctrl_debug.chassis_input.VehDa_mWght_mp = dfcv_special_chassis.vehicle_mass;
      ctrl_debug.chassis_input.VehDa_vEgoSpd_mp = chassis_info.v;
      ctrl_debug.chassis_input.VehDa_aEgoAcc_mp = chassis_info.a;
      ctrl_debug.chassis_input.VehDa_stTraCurGear_mp =chassis_info.gear_number;
      ctrl_debug.chassis_input.selected_gear_number =chassis_info.selected_gear_number;
      ctrl_debug.chassis_input.VehDa_rTraCurGear_mp = dfcv_special_chassis.current_gear_ratio;
      ctrl_debug.chassis_input.VehDa_stCluSwt_mp = dfcv_special_chassis.clutch_switch;
      ctrl_debug.chassis_input.VehDa_prcTrqEngNomFric_mp = dfcv_special_chassis.nominal_fricton_troque_percent;
      ctrl_debug.chassis_input.VehDa_prcTrqEstimdLoss_mp = dfcv_special_chassis.estimated_lossed_torque_percent;
      ctrl_debug.chassis_input.VehDa_stSrcBrk_mp = dfcv_special_chassis.source_address_brake_control_device;
      ctrl_debug.chassis_input.VehDa_prcActuTrq_mp = dfcv_special_chassis.actual_engine_torque_percent;
      ctrl_debug.chassis_input.VehDa_prcDrvrDmdTrq_mp = dfcv_special_chassis.driver_damand_torque_percent;
      ctrl_debug.chassis_input.VehDa_stSrcEngCtrl_mp = dfcv_special_chassis.source_address_engine_control_device;
      ctrl_debug.chassis_input.VehDa_pFrontLeft_mp = dfcv_special_chassis.brake_pressure_lf;
      ctrl_debug.chassis_input.VehDa_nEngSpd_mp = chassis_info.engine_speed;
      ctrl_debug.chassis_input.VehDa_stTrlrCnctn_mp = dfcv_special_chassis.trailer_connected_status;
      ctrl_debug.chassis_input.VehDa_stTraSht_mp = dfcv_special_chassis.transmission_shift_status;
      ctrl_debug.chassis_input.VehDa_stTraEgd_mp = dfcv_special_chassis.transmission_engage_status;
      ctrl_debug.chassis_input.VehDa_stTraSelGear_mp = dfcv_special_chassis.transmission_selected_gear;
      ctrl_debug.chassis_input.VehDa_stTraTrqLim_mp = dfcv_special_chassis.tcu_engine_control_mode;
      ctrl_debug.chassis_input.VehDa_rAccrPedl_mp = chassis_info.acc_pedal_value;
      ctrl_debug.chassis_input.SpdPln_lTrgLngErr_mp = longtitude_control.debug_SpdPln_lTrgLngErr_mp;
      ctrl_debug.chassis_input.VehDa_nOutpShaft_mp = longtitude_control.debug_VehDa_nOutpShaft_mp;
      ctrl_debug.chassis_input.VehDa_stBrkReady4Rls_mp = longtitude_control.debug_VehDa_stBrkReady4Rls_mp;
      //  Get Longtitude Control debug
      ctrl_debug.lon_debug.control_long_VecDa_aCalcd_mp = longtitude_control.debug_VecDa_aCalcd_mp;
      ctrl_debug.lon_debug.control_long_VDC_facDrvTrmJ_mp = longtitude_control.debug_VDC_facDrvTrmJ_mp;
      ctrl_debug.lon_debug.control_long_speed_integ_value = longtitude_control.debug_speed_integ_value;
      ctrl_debug.lon_debug.control_long_speed_propo_value = longtitude_control.debug_speed_propo_value;
      ctrl_debug.lon_debug.control_long_speed_value_a_req = longtitude_control.debug_speed_value_a_req;
      ctrl_debug.lon_debug.control_long_speed_integ_freeze_status = longtitude_control.debug_speed_integ_freeze_status;
      ctrl_debug.lon_debug.control_long_speed_inieg_init_status = longtitude_control.debug_speed_inieg_init_status;
      ctrl_debug.lon_debug.control_long_speed_error = longtitude_control.debug_speed_error;
      ctrl_debug.lon_debug.control_long_longitudinal_control_status = longtitude_control.debug_longitudinal_control_status;
      ctrl_debug.lon_debug.control_long_engine_torque_raw = longtitude_control.debug_engine_torque_raw;
      ctrl_debug.lon_debug.control_long_engine_torque = longtitude_control.debug_engine_torque;
      ctrl_debug.lon_debug.control_long_engine_torque_nfm = longtitude_control.debug_engine_torque_nfm;
      ctrl_debug.lon_debug.control_long_Eng_rAccrPedlReq = longtitude_control.debug_Eng_rAccrPedlReq;
      ctrl_debug.lon_debug.control_long_EBS_aReq = longtitude_control.debug_EBS_aReq;
      ctrl_debug.lon_debug.control_long_vehicle_max_acce = longtitude_control.debug_vehicle_max_acce;
      ctrl_debug.lon_debug.control_long_speedup_shift_status = longtitude_control.debug_speedup_shift_status;
      ctrl_debug.lon_debug.control_long_tcu_torque_status = longtitude_control.debug_tcu_torque_status;
      ctrl_debug.lon_debug.control_long_cochs_stnegtrqreq_mp = longtitude_control.debug_cochs_stnegtrqreq_mp;
      ctrl_debug.lon_debug.control_long_fr_rolling_res_mp = longtitude_control.debug_fr_rolling_res_mp;
      ctrl_debug.lon_debug.control_longg_fr_air_drag_mp = longtitude_control.debug_fr_air_drag_mp;
      ctrl_debug.lon_debug.control_long_fr_slp_res_mp = longtitude_control.debug_fr_slp_res_mp;
      ctrl_debug.lon_debug.control_long_fr_acc_res_mp = longtitude_control.debug_fr_acc_res_mp;
      ctrl_debug.lon_debug.control_longg_fr_cmp_mp = longtitude_control.debug_fr_cmp_mp;
      ctrl_debug.lon_debug.control_long_fr_eng_fric_mp = longtitude_control.debug_fr_eng_fric_mp;
      ctrl_debug.lon_debug.control_long_AD_Status = longtitude_control.debug_AD_Status;
      ctrl_debug.lon_debug.control_long_Cochs_stStarting = longtitude_control.debug_Cochs_stStarting;              
      ctrl_debug.lon_debug.control_long_Vmc_aAccnGvnrD_mp = longtitude_control.debug_Vmc_aAccnGvnrD_mp;      
      ctrl_debug.lon_debug.control_long_Vdc_aReqNom = longtitude_control.debug_Vdc_aReqNom;
      ctrl_debug.lon_debug.control_long_Tra_stShiftWhtSpdUp = longtitude_control.debug_Tra_stShiftWhtSpdUp;
      ctrl_debug.lon_debug.control_long_Tra_stTrqLimWthTcu = longtitude_control.debug_Tra_stTrqLimWthTcu;           
      ctrl_debug.lon_debug.control_long_Tra_trqReq = longtitude_control.debug_Tra_trqReq;     
      ctrl_debug.lon_debug.control_long_acc_pid = longtitude_control.debug_acc_pid;
      ctrl_debug.lon_debug.control_long_Eng_throttle_real = longtitude_control.debug_throttle_real;
      #endif
      
      // Get lat Control debug
      #if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
      ctrl_debug.lat_debug.control_lat_LATC_phiSteerAngle = lateral_control.lat_debug.CurSteerAngle;
      ctrl_debug.lat_debug.control_lat_LATC_wVehYawRate = lateral_control.lat_debug.CurVehYawRate;
      ctrl_debug.lat_debug.control_lat_LCC_lPrevLatDist4LQR = lateral_control.lat_debug.PrevLatDist;
      ctrl_debug.lat_debug.control_lat_LCC_lPrevHeading4LQR = lateral_control.lat_debug.PrevHeading;
      ctrl_debug.lat_debug.control_lat_LCC_lPrevCv4LQR = lateral_control.lat_debug.PrevCurve;
      // ctrl_debug.lat_debug.control_lat_LCC_lPrevLatDist4LQR_Path = lateral_control.lat_debug.PrevLatDist_Path;
      //ctrl_debug.lat_debug.control_lat_LCC_lPrevHeading4LQR_Path = lateral_control.lat_debug.PrevHeading_Path;
      //ctrl_debug.lat_debug.control_lat_LCC_lPrevCv4LQR_Path = lateral_control.lat_debug.PrevCurve_Path;
      ctrl_debug.lat_debug.control_lat_VMC_phiTarAgSteer = lateral_control.lat_debug.TarAngleSteer;

      ctrl_debug.lat_debug.control_lat_LATC_lDist4Cmr = lateral_control.lat_debug.CalculateLatDist;
      ctrl_debug.lat_debug.control_lat_LATC_agHead4Cmr = lateral_control.lat_debug.CalculateLatHeading;
      ctrl_debug.lat_debug.control_LATC_cCv4Cmr = lateral_control.lat_debug.CalculateLatCurve;
      ctrl_debug.lat_debug.control_LC_Moving_Flag = lateral_control.lat_debug.ChangeLaneFlag;

      ctrl_debug.lat_debug.control_lat_LATC_eleK1 = lateral_control.lat_debug.LATC_eleK1;
      ctrl_debug.lat_debug.control_lat_LATC_eleK2 = lateral_control.lat_debug.LATC_eleK2;
      ctrl_debug.lat_debug.control_lat_LATC_eleK3 = lateral_control.lat_debug.LATC_eleK3;
      ctrl_debug.lat_debug.control_lat_LATC_eleK4 = lateral_control.lat_debug.LATC_eleK4;
      ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle1 = lateral_control.lat_debug.LATC_agFrTyreFBEle1;
      ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle2 = lateral_control.lat_debug.LATC_agFrTyreFBEle2;
      ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle3 = lateral_control.lat_debug.LATC_agFrTyreFBEle3;
      ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle4 = lateral_control.lat_debug.LATC_agFrTyreFBEle4;
      ctrl_debug.lat_debug.control_lat_LATC_agFFSteerDeg = lateral_control.lat_debug.LATC_agFFSteerDeg;
      ctrl_debug.lat_debug.control_lat_LATC_agFBSteerDeg = lateral_control.lat_debug.LATC_agFBSteerDeg;

 // 状态信息
      ctrl_debug.lat_debug.control_lat_ADmode =  lateral_control.lat_debug.ADmode;
      ctrl_debug.lat_debug.control_lat_CoST_stADMoEna = lateral_control.lat_debug.CoST_stADMoEna;
      ctrl_debug.lat_debug.control_lat_LATC_swtPathSel_C = lateral_control.lat_debug.LATC_swtPathSel_C;
      ctrl_debug.lat_debug.control_lat_LATC_Q1Wgt4LQR_C = lateral_control.lat_debug.LATC_Q1Wgt4LQR_C;
      ctrl_debug.lat_debug.control_lat_LATC_Q2Wgt4LQR_C = lateral_control.lat_debug.LATC_Q2Wgt4LQR_C;
      ctrl_debug.lat_debug.control_lat_LATC_Q3Wgt4LQR_C = lateral_control.lat_debug.LATC_Q3Wgt4LQR_C;
      ctrl_debug.lat_debug.control_lat_LATC_Q4Wgt4LQR_C = lateral_control.lat_debug.LATC_Q4Wgt4LQR_C;
      ctrl_debug.lat_debug.control_lat_LKA_swtAgSteerOfstSel_C = lateral_control.lat_debug.LKA_swtAgSteerOfstSel_C;
      ctrl_debug.lat_debug.control_lat_LATC_agSteerOfst_C = lateral_control.lat_debug.LATC_agSteerOfst_C;
      ctrl_debug.lat_debug.control_lat_LATC_agSteerZeroOfst = lateral_control.lat_debug.LATC_agSteerZeroOfst;

      ctrl_debug.lat_debug.control_lat_LQR_IntNum = lateral_control.lat_debug.LQR_IntNum;
      ctrl_debug.lat_debug.control_lat_LC_lPrevCv4FF = lateral_control.lat_debug.LC_lPrevCv4FF;
      ctrl_debug.lat_debug.control_lat_LC_lPrevHeading4FB =  lateral_control.lat_debug.LC_lPrevHeading4FB;
      ctrl_debug.lat_debug.control_lat_LC_lPrevLatDist4FB =  lateral_control.lat_debug.LC_lPrevLatDist4FB;
      ctrl_debug.lat_debug.control_lat_LatCv4LQR =  lateral_control.lat_debug.LatCv4LQR;
      ctrl_debug.lat_debug.control_lat_LatDist4LQR =  lateral_control.lat_debug.LatDist4LQR;
      ctrl_debug.lat_debug.control_lat_LatHeading4LQR =  lateral_control.lat_debug.LatHeading4LQR;     
      ctrl_debug.lat_debug.control_lat_DE_mVehMass = lateral_control.lat_debug.DE_mVehMass;
      ctrl_debug.lat_debug.control_lat_DE_stTrlr = lateral_control.lat_debug.DE_stTrlr;
      ctrl_debug.lat_debug.control_lat_LATC_vDistErrFlt4LQR = lateral_control.lat_debug.LATC_vDistErrFlt4LQR;
      ctrl_debug.lat_debug.control_lat_LATC_vLatSpdFlt4LQR = lateral_control.lat_debug.LATC_vLatSpdFlt4LQR;
      ctrl_debug.lat_debug.control_lat_LATC_agHeadFlt4LQR = lateral_control.lat_debug.LATC_agHeadFlt4LQR;
      ctrl_debug.lat_debug.control_lat_LATC_wYRErrFlt4LQR = lateral_control.lat_debug.LATC_wYRErrFlt4LQR;


      #endif
      // send messge for Control to debug
      msg_sender_->SendControlDebugInfo(ctrl_debug);
    }

    // CAN线连接状态：0 ~ 正常, 1 ~ 丢帧
    special_chassis_info.cnt_stu_frame_loss_can0 = 0;
    special_chassis_info.cnt_stu_frame_loss_can1 = 0;
    special_chassis_info.cnt_stu_frame_loss_can2 = 0;
    special_chassis_info.cnt_stu_frame_loss_can3 = 0;
    // 网关和车身的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
    special_chassis_info.cnt_stu_gtw_to_veh_can0 = 1;
    special_chassis_info.cnt_stu_gtw_to_veh_can1 = 1;
    special_chassis_info.cnt_stu_gtw_to_veh_can2 = 1;
    special_chassis_info.cnt_stu_gtw_to_veh_can3 = 1;
    // 控制侧和网关的CAN线连接状态：0 ~ 未连接, 1 ~ 已连接
    special_chassis_info.cnt_stu_ctl_to_gtw_can0 =
        !can_connect_status.cnt_timeout_can_0;
    special_chassis_info.cnt_stu_ctl_to_gtw_can1 =
        !can_connect_status.cnt_timeout_can_1;
    special_chassis_info.cnt_stu_ctl_to_gtw_can2 =
        !can_connect_status.cnt_timeout_can_2;
    special_chassis_info.cnt_stu_ctl_to_gtw_can3 =
        !can_connect_status.cnt_timeout_can_3;
    // 网关和控制侧的连接状态: 0 ~ 未连接, 1 ~ 已连接
    special_chassis_info.cnt_stu_ctl_to_gtw = !can_timeout;

    /// TODO: 临时使用其他变量存储蛇形标记
    special_chassis_info.cnt_stu_ctl_to_gtw = 
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.is_hunting;

    msg_sender_->SendSpecialChassisInfo(special_chassis_info);
#else
#if ENTER_PLAYBACK_MODE_ADHMI
    msg_sender_->SendChassis(chassis_info);
#endif

    msg_sender_->SendChassisCtlCmd(chassis_cmd);

    // Get Planning Input message
    #if ((CONTROLLER_LONGITUDINAL == CONTROLLER_LONGITUDINAL_DFCV) || (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV))
    ctrl_debug.planning_input.SpdPln_vTrgSpd_mp = chassis_cmd.velocity;
    ctrl_debug.planning_input.SpdPln_aTrgAcc_mp = chassis_cmd.acceleration;
    ctrl_debug.planning_input.BhvCrdn_numBhvID_mp = longtitude_control.debug_AD_Status;
      
    #if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
    ctrl_debug.planning_input.LaneTraj_IsValid =  planning_info.tar_trj.LaneTraj_Cv.LaneTraj_IsValid;
    ctrl_debug.planning_input.LATC_LineCv0 = planning_info.tar_trj.LaneTraj_Cv.c_center[0];
    ctrl_debug.planning_input.LATC_LineCv1 = planning_info.tar_trj.LaneTraj_Cv.c_center[1];
    ctrl_debug.planning_input.LATC_LineCv2 = planning_info.tar_trj.LaneTraj_Cv.c_center[2];
    ctrl_debug.planning_input.LATC_LineCv3 = planning_info.tar_trj.LaneTraj_Cv.c_center[3];
    ctrl_debug.planning_input.LATC_LineCv4 = 0.0;
    ctrl_debug.planning_input.LATC_LineCv5 = 0.0;

    ctrl_debug.planning_input.PlanningTraj_IsValid = planning_info.tar_trj.PlanningTraj_Cv.PlanningTraj_IsValid;
    ctrl_debug.planning_input.LATC_cTgtPathCv0 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[0];
    ctrl_debug.planning_input.LATC_cTgtPathCv1 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[1];
    ctrl_debug.planning_input.LATC_cTgtPathCv2 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[2];
    ctrl_debug.planning_input.LATC_cTgtPathCv3 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[3];
    ctrl_debug.planning_input.LATC_cTgtPathCv4 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[4];
    ctrl_debug.planning_input.LATC_cTgtPathCv5 = planning_info.tar_trj.PlanningTraj_Cv.coeffs[5];
    #endif

      // Get chassis Input message

      ctrl_debug.chassis_input.VehDa_rBrkPedl_mp = chassis_info.brake_pedal_value;
      ctrl_debug.chassis_input.VehDa_mWght_mp = dfcv_special_chassis.vehicle_mass;
      ctrl_debug.chassis_input.VehDa_vEgoSpd_mp = chassis_info.v;
      ctrl_debug.chassis_input.VehDa_aEgoAcc_mp = chassis_info.a;
      ctrl_debug.chassis_input.VehDa_stTraCurGear_mp =chassis_info.gear_number;
      ctrl_debug.chassis_input.VehDa_rTraCurGear_mp = dfcv_special_chassis.current_gear_ratio;
      ctrl_debug.chassis_input.VehDa_stCluSwt_mp = dfcv_special_chassis.clutch_switch;
      ctrl_debug.chassis_input.VehDa_prcTrqEngNomFric_mp = dfcv_special_chassis.nominal_fricton_troque_percent;
      ctrl_debug.chassis_input.VehDa_prcTrqEstimdLoss_mp = dfcv_special_chassis.estimated_lossed_torque_percent;
      ctrl_debug.chassis_input.VehDa_stSrcBrk_mp = dfcv_special_chassis.source_address_brake_control_device;
      ctrl_debug.chassis_input.VehDa_prcActuTrq_mp = dfcv_special_chassis.actual_engine_torque_percent;
      ctrl_debug.chassis_input.VehDa_prcDrvrDmdTrq_mp = dfcv_special_chassis.driver_damand_torque_percent;
      ctrl_debug.chassis_input.VehDa_stSrcEngCtrl_mp = dfcv_special_chassis.source_address_engine_control_device;
      ctrl_debug.chassis_input.VehDa_pFrontLeft_mp = dfcv_special_chassis.brake_pressure_lf;
      ctrl_debug.chassis_input.VehDa_nEngSpd_mp = chassis_info.engine_speed;
      ctrl_debug.chassis_input.VehDa_stTrlrCnctn_mp = dfcv_special_chassis.trailer_connected_status;
      ctrl_debug.chassis_input.VehDa_stTraSht_mp = dfcv_special_chassis.transmission_shift_status;
      ctrl_debug.chassis_input.VehDa_stTraEgd_mp = dfcv_special_chassis.transmission_engage_status;
      ctrl_debug.chassis_input.VehDa_stTraSelGear_mp = dfcv_special_chassis.transmission_selected_gear;
      ctrl_debug.chassis_input.VehDa_stTraTrqLim_mp = dfcv_special_chassis.tcu_engine_control_mode;
      ctrl_debug.chassis_input.VehDa_rAccrPedl_mp = chassis_info.acc_pedal_value;
      ctrl_debug.chassis_input.SpdPln_lTrgLngErr_mp = longtitude_control.debug_SpdPln_lTrgLngErr_mp;
      ctrl_debug.chassis_input.VehDa_nOutpShaft_mp = longtitude_control.debug_VehDa_nOutpShaft_mp;
      ctrl_debug.chassis_input.VehDa_stBrkReady4Rls_mp = longtitude_control.debug_VehDa_stBrkReady4Rls_mp;
      //  Get Longtitude Control debug
      ctrl_debug.lon_debug.control_long_VecDa_aCalcd_mp = longtitude_control.debug_VecDa_aCalcd_mp;
      ctrl_debug.lon_debug.control_long_VDC_facDrvTrmJ_mp = longtitude_control.debug_VDC_facDrvTrmJ_mp;
      ctrl_debug.lon_debug.control_long_speed_integ_value = longtitude_control.debug_speed_integ_value;
      ctrl_debug.lon_debug.control_long_speed_propo_value = longtitude_control.debug_speed_propo_value;
      ctrl_debug.lon_debug.control_long_speed_value_a_req = longtitude_control.debug_speed_value_a_req;
      ctrl_debug.lon_debug.control_long_speed_integ_freeze_status = longtitude_control.debug_speed_integ_freeze_status;
      ctrl_debug.lon_debug.control_long_speed_inieg_init_status = longtitude_control.debug_speed_inieg_init_status;
      ctrl_debug.lon_debug.control_long_speed_error = longtitude_control.debug_speed_error;
      ctrl_debug.lon_debug.control_long_longitudinal_control_status = longtitude_control.debug_longitudinal_control_status;
      ctrl_debug.lon_debug.control_long_engine_torque_raw = longtitude_control.debug_engine_torque_raw;
      ctrl_debug.lon_debug.control_long_engine_torque = longtitude_control.debug_engine_torque;
      ctrl_debug.lon_debug.control_long_engine_torque_nfm = longtitude_control.debug_engine_torque_nfm;
      ctrl_debug.lon_debug.control_long_Eng_rAccrPedlReq = longtitude_control.debug_Eng_rAccrPedlReq;
      ctrl_debug.lon_debug.control_long_EBS_aReq = longtitude_control.debug_EBS_aReq;
      ctrl_debug.lon_debug.control_long_vehicle_max_acce = longtitude_control.debug_vehicle_max_acce;
      ctrl_debug.lon_debug.control_long_speedup_shift_status = longtitude_control.debug_speedup_shift_status;
      ctrl_debug.lon_debug.control_long_tcu_torque_status = longtitude_control.debug_tcu_torque_status;
      ctrl_debug.lon_debug.control_long_cochs_stnegtrqreq_mp = longtitude_control.debug_cochs_stnegtrqreq_mp;
      ctrl_debug.lon_debug.control_long_fr_rolling_res_mp = longtitude_control.debug_fr_rolling_res_mp;
      ctrl_debug.lon_debug.control_longg_fr_air_drag_mp = longtitude_control.debug_fr_air_drag_mp;
      ctrl_debug.lon_debug.control_long_fr_slp_res_mp = longtitude_control.debug_fr_slp_res_mp;
      ctrl_debug.lon_debug.control_long_fr_acc_res_mp = longtitude_control.debug_fr_acc_res_mp;
      ctrl_debug.lon_debug.control_longg_fr_cmp_mp = longtitude_control.debug_fr_cmp_mp;
      ctrl_debug.lon_debug.control_long_fr_eng_fric_mp = longtitude_control.debug_fr_eng_fric_mp;
      ctrl_debug.lon_debug.control_long_AD_Status = longtitude_control.debug_AD_Status;
      ctrl_debug.lon_debug.control_long_Cochs_stStarting = longtitude_control.debug_Cochs_stStarting;              
      ctrl_debug.lon_debug.control_long_Vmc_aAccnGvnrD_mp = longtitude_control.debug_Vmc_aAccnGvnrD_mp;      
      ctrl_debug.lon_debug.control_long_Vdc_aReqNom = longtitude_control.debug_Vdc_aReqNom;
      ctrl_debug.lon_debug.control_long_Tra_stShiftWhtSpdUp = longtitude_control.debug_Tra_stShiftWhtSpdUp;
      ctrl_debug.lon_debug.control_long_Tra_stTrqLimWthTcu = longtitude_control.debug_Tra_stTrqLimWthTcu;           
      ctrl_debug.lon_debug.control_long_Tra_trqReq = longtitude_control.debug_Tra_trqReq;     
      ctrl_debug.lon_debug.control_long_acc_pid = longtitude_control.debug_acc_pid;
    #if (ENTER_PLAYBACK_MODE_ADHMI)
    ctrl_debug.adecu_can_debug.adecu_debug_ad_mode = longtitude_control.adecu_debug_ad_mode;
    ctrl_debug.adecu_can_debug.adecu_debug_tar_vel_set = longtitude_control.adecu_debug_tar_vel_set;
    ctrl_debug.adecu_can_debug.adecu_debug_tar_trajectory_type =longtitude_control.adecu_debug_tar_trajectory_type;
    ctrl_debug.adecu_can_debug.adecu_debug_tar_vel_type = longtitude_control.adecu_debug_tar_vel_type;
    ctrl_debug.adecu_can_debug.adecu_debug_vel_obj_dist = longtitude_control.adecu_debug_vel_obj_dist;
    ctrl_debug.adecu_can_debug.adecu_debug_vel_obj_vel = longtitude_control.adecu_debug_vel_obj_vel;
    ctrl_debug.adecu_can_debug.adecu_debug_planning_send_vel = longtitude_control.adecu_debug_planning_send_vel;
    ctrl_debug.adecu_can_debug.adecu_debug_planning_send_ax = longtitude_control.adecu_debug_planning_send_ax;
    #endif
    
    #endif
      
    // Get lat Control debug
    #if (CONTROLLER_LATERAL == CONTROLLER_LATERAL_DFCV)
    ctrl_debug.lat_debug.control_lat_LATC_phiSteerAngle = lateral_control.lat_debug.CurSteerAngle;
    ctrl_debug.lat_debug.control_lat_LATC_wVehYawRate = lateral_control.lat_debug.CurVehYawRate;
    ctrl_debug.lat_debug.control_lat_LCC_lPrevLatDist4LQR = lateral_control.lat_debug.PrevLatDist;
    ctrl_debug.lat_debug.control_lat_LCC_lPrevHeading4LQR = lateral_control.lat_debug.PrevHeading;
    ctrl_debug.lat_debug.control_lat_LCC_lPrevCv4LQR = lateral_control.lat_debug.PrevCurve;
    // ctrl_debug.lat_debug.control_lat_LCC_lPrevLatDist4LQR_Path = lateral_control.lat_debug.PrevLatDist_Path;
    //ctrl_debug.lat_debug.control_lat_LCC_lPrevHeading4LQR_Path = lateral_control.lat_debug.PrevHeading_Path;
    //ctrl_debug.lat_debug.control_lat_LCC_lPrevCv4LQR_Path = lateral_control.lat_debug.PrevCurve_Path;
    ctrl_debug.lat_debug.control_lat_VMC_phiTarAgSteer = lateral_control.lat_debug.TarAngleSteer;

    ctrl_debug.lat_debug.control_lat_LATC_lDist4Cmr = lateral_control.lat_debug.CalculateLatDist;
    ctrl_debug.lat_debug.control_lat_LATC_agHead4Cmr = lateral_control.lat_debug.CalculateLatHeading;
    ctrl_debug.lat_debug.control_LATC_cCv4Cmr = lateral_control.lat_debug.CalculateLatCurve;
    ctrl_debug.lat_debug.control_LC_Moving_Flag = lateral_control.lat_debug.ChangeLaneFlag;

    ctrl_debug.lat_debug.control_lat_LATC_eleK1 = lateral_control.lat_debug.LATC_eleK1;
    ctrl_debug.lat_debug.control_lat_LATC_eleK2 = lateral_control.lat_debug.LATC_eleK2;
    ctrl_debug.lat_debug.control_lat_LATC_eleK3 = lateral_control.lat_debug.LATC_eleK3;
    ctrl_debug.lat_debug.control_lat_LATC_eleK4 = lateral_control.lat_debug.LATC_eleK4;
    ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle1 = lateral_control.lat_debug.LATC_agFrTyreFBEle1;
    ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle2 = lateral_control.lat_debug.LATC_agFrTyreFBEle2;
    ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle3 = lateral_control.lat_debug.LATC_agFrTyreFBEle3;
    ctrl_debug.lat_debug.control_lat_LATC_agFrTyreFBEle4 = lateral_control.lat_debug.LATC_agFrTyreFBEle4;
    ctrl_debug.lat_debug.control_lat_LATC_agFFSteerDeg = lateral_control.lat_debug.LATC_agFFSteerDeg;
    ctrl_debug.lat_debug.control_lat_LATC_agFBSteerDeg = lateral_control.lat_debug.LATC_agFBSteerDeg;

    // 状态信息
    ctrl_debug.lat_debug.control_lat_ADmode =  lateral_control.lat_debug.ADmode;
    ctrl_debug.lat_debug.control_lat_CoST_stADMoEna = lateral_control.lat_debug.CoST_stADMoEna;
    ctrl_debug.lat_debug.control_lat_LATC_swtPathSel_C = lateral_control.lat_debug.LATC_swtPathSel_C;
    ctrl_debug.lat_debug.control_lat_LATC_Q1Wgt4LQR_C = lateral_control.lat_debug.LATC_Q1Wgt4LQR_C;
    ctrl_debug.lat_debug.control_lat_LATC_Q2Wgt4LQR_C = lateral_control.lat_debug.LATC_Q2Wgt4LQR_C;
    ctrl_debug.lat_debug.control_lat_LATC_Q3Wgt4LQR_C = lateral_control.lat_debug.LATC_Q3Wgt4LQR_C;
    ctrl_debug.lat_debug.control_lat_LATC_Q4Wgt4LQR_C = lateral_control.lat_debug.LATC_Q4Wgt4LQR_C;
    ctrl_debug.lat_debug.control_lat_LKA_swtAgSteerOfstSel_C = lateral_control.lat_debug.LKA_swtAgSteerOfstSel_C;
    ctrl_debug.lat_debug.control_lat_LATC_agSteerOfst_C = lateral_control.lat_debug.LATC_agSteerOfst_C;
    ctrl_debug.lat_debug.control_lat_LATC_agSteerZeroOfst = lateral_control.lat_debug.LATC_agSteerZeroOfst;

    ctrl_debug.lat_debug.control_lat_LQR_IntNum = lateral_control.lat_debug.LQR_IntNum;
    ctrl_debug.lat_debug.control_lat_LC_lPrevCv4FF = lateral_control.lat_debug.LC_lPrevCv4FF;
    ctrl_debug.lat_debug.control_lat_LC_lPrevHeading4FB =  lateral_control.lat_debug.LC_lPrevHeading4FB;
    ctrl_debug.lat_debug.control_lat_LC_lPrevLatDist4FB =  lateral_control.lat_debug.LC_lPrevLatDist4FB;
    ctrl_debug.lat_debug.control_lat_LatCv4LQR =  lateral_control.lat_debug.LatCv4LQR;
    ctrl_debug.lat_debug.control_lat_LatDist4LQR =  lateral_control.lat_debug.LatDist4LQR;
    ctrl_debug.lat_debug.control_lat_LatHeading4LQR =  lateral_control.lat_debug.LatHeading4LQR;     
    ctrl_debug.lat_debug.control_lat_DE_mVehMass = lateral_control.lat_debug.DE_mVehMass;
    ctrl_debug.lat_debug.control_lat_DE_stTrlr = lateral_control.lat_debug.DE_stTrlr;
    ctrl_debug.lat_debug.control_lat_LATC_vDistErrFlt4LQR = lateral_control.lat_debug.LATC_vDistErrFlt4LQR;
    ctrl_debug.lat_debug.control_lat_LATC_vLatSpdFlt4LQR = lateral_control.lat_debug.LATC_vLatSpdFlt4LQR;
    ctrl_debug.lat_debug.control_lat_LATC_agHeadFlt4LQR = lateral_control.lat_debug.LATC_agHeadFlt4LQR;
    ctrl_debug.lat_debug.control_lat_LATC_wYRErrFlt4LQR = lateral_control.lat_debug.LATC_wYRErrFlt4LQR;

    #endif
    // send messge for Control to debug
    msg_sender_->SendControlDebugInfo(ctrl_debug);
#endif  // #if (!ENTER_PLAYBACK_MODE)

    dormancy.Sleep();
  }

  LOG_INFO(3) << "Check Tasks Status Thread ... [Stopped]";
}

/******************************************************************************/
/** 发送相关数据到HMI界面显示
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2021.02.22   sip         新规作成
 *
 *  <Function>
 *       发送相关数据到HMI界面显示
 *
 *  <Attention>
 *       None
 *
 */
void TaskManager::ThreadSendDataToHmi() {
  LOG_INFO(3) << "Send Data To HMI Thread ... [Started]";

  Dormancy dormancy(100);
  while (true == thread_running_flag_send_data_hmi_) {

    hmi_msg_sender_->SendHmiMsg();

    dormancy.Sleep();
  }

  LOG_INFO(3) << "Send Data To HMI Thread ... [Stoped]";
}

/******************************************************************************/
/** 处理模块消息
 ******************************************************************************
 *  @param      msg             (in)           消息
 *              sender          (in)           发送者
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       处理模块消息
 *
 *  <Attention>
 *       None
 *
 */
bool TaskManager::HandleMessage(const Message& msg, Task* sender) {
  // Must be reentrant function

  bool ret = true;
  switch (msg.id()) {
  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_0): {
    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvCanChannel0();
  }
    break;

  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_1): {
    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvCanChannel1();
  }
    break;

  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_2): {
    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvCanChannel2();
  }
    break;

  case (MSG_ID_MODULE_STATUS_CAN_RECV_CH_3): {
    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvCanChannel3();
  }
    break;

  case (MSG_ID_IMU): {
    const MessageImu& message =
        dynamic_cast<const MessageImu&>(msg);

    Phoenix_SharedData_SetImu(message.imu());

    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvMsgImu();
  }
    break;

#if ENTER_PLAYBACK_MODE
  case (MSG_ID_CHASSIS): {
    const MessageChassis& message =
        dynamic_cast<const MessageChassis&>(msg);

    Phoenix_SharedData_SetChassis(message.chassis());

    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvMsgChassis();
  }
    break;
#endif

  case (MSG_ID_PLANNING_RESULT): {
    const MessagePlanningResult& message =
        dynamic_cast<const MessagePlanningResult&>(msg);

    Phoenix_SharedData_SetPlanningResult(message.planning_result());

    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvMsgPlanningResult();
  }
    break;

  case (MSG_ID_DFCV_SPECIAL_CHASSIS_INFO): {
    const MessageDFCVSpecialChassisInfo& message =
        dynamic_cast<const MessageDFCVSpecialChassisInfo&>(msg);

    Phoenix_SharedData_SetDFCVSpecialChassisInfo(message.dfcv_special_chassis_info());

    // Monitor
    Phoenix_Work_Monitor_FeedDog_RecvMsgDFCVSpecialChassisInfo();
  }
    break;

  default:
    ret = false;
    break;
  }

  return (ret);
}


}  // namespace framework
}  // namespace phoenix

