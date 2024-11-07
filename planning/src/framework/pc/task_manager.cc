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
#include "planning/plan_debug.h"
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
#include "communication/shared_data.h"
#include "utils/log.h"
#include "utils/com_utils.h"
#include "utils/gps_tools.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "task_chatter.h"
#include "task_planning.h"

#include "gnss.pb.h"
#include "imu.pb.h"
#include "chassis.pb.h"
#include "lane_mark_camera.pb.h"
#include "obstacles_camera.pb.h"
#include "obstacles_radar.pb.h"

#include "data_serialization.h"

#if (HD_MAP_TYPE == HD_MAP_TYPE_D17)
#include "ExpWrapper.h"
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


  task_planning_.reset(new TaskPlanning(this));
  task_chatter_.reset(new TaskChatter(this));

#ifdef ENABLE_PACC_ADASIS
  task_adasisv2_.reset(new adasisv2::TaskRecvADASISv2Msg(this));
#endif

  communication_mode_ = 0;

#if (ENABLE_ROS_NODE)
  ros_node_.reset(new RosNode(argc, argv, "vehicle_motion_planning"));
#endif
#if (ENABLE_LCM_NODE)
  lcm_node_.reset(new LcmNode("udpm://239.255.76.67:7667?ttl=1"));
#endif
#if (ENABLE_UDP_NODE)
  udp_node_.reset(new UdpNode());
#endif

  msg_receiver_.reset(new MsgReceiver(this));
  msg_sender_.reset(new MsgSender(this));

  if (0 == communication_mode_) {
#if (ENABLE_ROS_NODE)
    msg_receiver_->SetRosNode(ros_node_.get());
    msg_sender_->SetRosNode(ros_node_.get());
#endif
#if (ENABLE_LCM_NODE)
    msg_receiver_->SetLcmNode(Nullptr_t);
    msg_sender_->SetLcmNode(Nullptr_t);
#endif
#if (ENABLE_UDP_NODE)
    msg_receiver_->SetUdpNode(udp_node_.get());
    msg_sender_->SetUdpNode(udp_node_.get());
#endif
  } else if (1 == communication_mode_) {
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
  } else if (2 == communication_mode_) {
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
  if ((2 == communication_mode_) || (0 == communication_mode_)) {
    // Start UDP Node
    UdpNode::UdpParam udp_param;
    udp_param.enable_recv = true;
    udp_param.enable_send = true;
    udp_param.rcv_port = 9500;
    udp_param.snd_port = 9500;
    Int32_t mode = 1;
    if (0 == mode) {
      udp_param.mode = UdpNode::UdpParam::MODE_UNICAST;
      strncpy(udp_param.snd_addr, "192.168.1.100", sizeof(udp_param.snd_addr)-1);
    } else if (1 == mode) {
      udp_param.mode = UdpNode::UdpParam::MODE_GROUP;
      strncpy(udp_param.snd_addr, "224.10.10.2", sizeof(udp_param.snd_addr)-1);
    } else {
      udp_param.mode = UdpNode::UdpParam::MODE_BROADCAST;
      strncpy(udp_param.snd_addr, "192.168.10.102", sizeof(udp_param.snd_addr)-1);
    }

    ret = udp_node_->Start(udp_param);
    if (false == ret) {
      Stop();
      LOG_ERR << "Failed to start UDP node.";
      return (false);
    }
  }
#endif

  // Start Planning Task
  ret = task_planning_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start planning task.";
    return (false);
  }

  // Start Chatter Task
  ret = task_chatter_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start chatter task.";
    return false;
  }

#ifdef ENABLE_PACC_ADASIS
  ret = task_adasisv2_->Start();
  if (false == ret) {
    LOG_ERR << "Failed to start adasisv2 task.";
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
  if ((2 == communication_mode_) || (0 == communication_mode_)) {
    ret = udp_node_->StartReceiving();
    if (false == ret) {
      Stop();
      LOG_ERR << "Failed to start UDP receiving.";
      return false;
    }
  }
#endif

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  exp_map_t::ExpStart();
#endif

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

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  exp_map_t::ExpStop();
#endif

  // Stop chatter task
  task_chatter_->Stop();

  // Stop planning task
  task_planning_->Stop();

#ifdef ENABLE_PACC_ADASIS
  task_adasisv2_->Stop();
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

  Dormancy dormancy(50);
  for (Int32_t i = 0; i < 10; ++i) {
    dormancy.Sleep();
  }

  while (true == thread_running_flag_check_tasks_status_) {

    work_monitor_.DoWork();

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

  SharedData* shared_data = SharedData::instance();

  bool ret = true;
  switch (msg.id()) {
  case (MSG_ID_MAP_DATA): {
    const MessageMapData& message =
        dynamic_cast<const MessageMapData&>(msg);
    if ((Nullptr_t != message.data_buff()) && (message.buff_size() > 0)) {
      task_chatter_->Request(msg, this);
    }

    // Monitor
    work_monitor_.FeedDog_RecvHdMap();
  }
    break;

  case (MSG_ID_ROUTING_DATA): {
    task_chatter_->Request(msg, this);

    // Monitor
    work_monitor_.FeedDog_RecvRouting();
  }
    break;

  case (MSG_ID_GNSS): {
    const MessageGnss& message =
        dynamic_cast<const MessageGnss&>(msg);

    shared_data->SetGnss(*message.gnss());

    // Monitor
    work_monitor_.FeedDog_RecvGnss();
  }
    break;

  case (MSG_ID_IMU): {
    const MessageImu& message =
        dynamic_cast<const MessageImu&>(msg);

    shared_data->SetImu(*message.imu());

    // Monitor
    work_monitor_.FeedDog_RecvImu();
  }
    break;

  case (MSG_ID_CHASSIS): {
    const MessageChassis& message =
        dynamic_cast<const MessageChassis&>(msg);

    shared_data->SetChassis(*message.chassis());

    //printf("### yaw_rate_chassis = %0.1f\n",
    //       common::com_rad2deg(message.chassis()->yaw_rate));

    // Monitor
    work_monitor_.FeedDog_RecvChassis();
  }
    break;

  case (MSG_ID_SPECIAL_CHASSIS_INFO): {
    const MessageSpecialChassisInfo& message =
        dynamic_cast<const MessageSpecialChassisInfo&>(msg);

    shared_data->SetSpecialChassisInfo(*message.special_chassis_info());

    // Monitor
    work_monitor_.FeedDog_RecvSpecialChassisInfo();
  }
    break;

  case (MSG_ID_CHASSIS_CTL_CMD): {
    const MessageChassisCtlCmd& message =
        dynamic_cast<const MessageChassisCtlCmd&>(msg);

    shared_data->SetChassisCtlCmd(*message.chassis_ctl_cmd());

    // Monitor
    work_monitor_.FeedDog_RecvChassisCtlCmd();
  }
    break;

  case (MSG_ID_LANE_MARK_CAMERA_LIST): {
    const MessageLaneMarkCameraList& message =
        dynamic_cast<const MessageLaneMarkCameraList&>(msg);

    shared_data->SetLaneMarkCameraList(*message.lane_mark_camera_list());

    // Monitor
    work_monitor_.FeedDog_RecvLaneMarkCameraList();
  }
    break;

  case (MSG_ID_OBSTACLE_CAMERA_LIST): {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);

    shared_data->SetObstacleCameraList(*message.obstacle_camera_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleCameraList();
  }
    break;

  case (MSG_ID_OBSTACLE_RADAR_FRONT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetObstacleRadarFrontList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleRadarFrontList();
  }
    break;

  case (MSG_ID_OBSTACLE_RADAR_REAR_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetObstacleRadarRearList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleRadarRearList();
  }
    break;

  case (MSG_ID_OBSTACLE_RADAR_FRONT_LEFT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetObstacleRadarFrontLeftList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleRadarFrontLeftList();
  }
    break;

  case (MSG_ID_OBSTACLE_RADAR_FRONT_RIGHT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetObstacleRadarFrontRightList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleRadarFrontRightList();
  }
    break;

  case (MSG_ID_OBSTACLE_RADAR_REAR_LEFT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetObstacleRadarRearLeftList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleRadarRearLeftList();
  }
    break;

  case (MSG_ID_OBSTACLE_RADAR_REAR_RIGHT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetObstacleRadarRearRightList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleRadarRearRightList();
  }
    break;

  case (MSG_ID_SRR2_DETECTIONS_FRONT_LEFT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetSrr2DetectionsFrontLeftList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvSrr2DetectionsFrontLeftList();
  }
    break;

  case (MSG_ID_SRR2_DETECTIONS_FRONT_RIGHT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetSrr2DetectionsFrontRightList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvSrr2DetectionsFrontRightList();
  }
    break;

  case (MSG_ID_SRR2_DETECTIONS_REAR_LEFT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetSrr2DetectionsRearLeftList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvSrr2DetectionsRearLeftList();
  }
    break;

  case (MSG_ID_SRR2_DETECTIONS_REAR_RIGHT_LIST): {
    const MessageObstacleRadarList& message =
        dynamic_cast<const MessageObstacleRadarList&>(msg);

    shared_data->SetSrr2DetectionsRearRightList(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvSrr2DetectionsRearRightList();
  }
    break;

  case (MSG_ID_OBSTACLE_LIDAR_LIST_0): {
    const MessageObstacleLidarList& message =
        dynamic_cast<const MessageObstacleLidarList&>(msg);

    shared_data->SetObstacleLidarList0(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleLidarList0();
  }
    break;

  case (MSG_ID_OBSTACLE_LIDAR_LIST_1): {
    const MessageObstacleLidarList& message =
        dynamic_cast<const MessageObstacleLidarList&>(msg);

    shared_data->SetObstacleLidarList1(*message.obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleLidarList1();
  }
    break;

  case (MSG_ID_OBSTACLE_LIST): {
    if (config_.obj_filter_config.using_outside_obj_list) {
      const MessageObstacleList& message =
          dynamic_cast<const MessageObstacleList&>(msg);

      shared_data->SetOutsideObstacleList(*message.obstacle_list());

      // Monitor
      work_monitor_.FeedDog_RecvOutsideObstacleList();
    } else {
      const MessageObstacleList& message =
          dynamic_cast<const MessageObstacleList&>(msg);
      msg_sender_->SendObstacleList(*message.obstacle_list());

      // LOG_INFO(3) << "Send obstacle list.";
    }
  }
    break;

  // adecu debug
  case (MSG_ID_ADECU_VEL_PLANNING_DEBUG): {
    const MessageAdecuVelPlanningDebug& message =
        dynamic_cast<const MessageAdecuVelPlanningDebug&>(msg);

    shared_data->SetAdecuVelplanDebug(*message.adecu_planning_debug());

    // Monitor
    work_monitor_.FeedDog_RecvAdecuVelPlanning();
  }
    break;

  case (MSG_ID_PLANNING_RESULT): {
    const MessagePlanningResult& message =
        dynamic_cast<const MessagePlanningResult&>(msg);

    msg_sender_->SendPlanningResult(*(message.planning_result()));

    // Monitor
    work_monitor_.FeedDog_CompletePlanning(
          message.status(), message.time_elapsed());
  }
    break;

  // Add Planning debug BY ZQ for Ros

  case (MSG_ID_PLANNING_DEBUG):{
        const MessagePlanningDebug& message =
        dynamic_cast<const MessagePlanningDebug&>(msg);

    
    msg_sender_->SendPlannigDebug(*(message.plan_debug()));

  }
    break;
  // adecu ros debug msg
  case (MSG_ID_ADECU_PLANNING_DEBUG):{
        const MessageAdecuDebug& message = 
        dynamic_cast<const MessageAdecuDebug&>(msg);

    msg_sender_->SendAdecuDebug(*(message.adecu_debug()));
  }
    break;
    
  case (MSG_ID_TRAFFIC_SIGNAL_LIST):{
    const MessageTrafficSignalList &message =
        dynamic_cast<const MessageTrafficSignalList&>(msg);
    shared_data->SetTrafficSignalList(*message.traffic_signal_list());
  }
    break;

  case (MSG_ID_TRAFFIC_LIGHT_LIST):{
    const MessageTrafficLightList &message =
        dynamic_cast<const MessageTrafficLightList&>(msg);
    shared_data->SetTrafficLightList(*message.traffic_light_list());
  }
    break;

  case (MSG_ID_REMOTE_PLANNING_SETTINGS):{
    const MessageRemotePlanningSettings &message =
        dynamic_cast<const MessageRemotePlanningSettings&>(msg);
    shared_data->SetRemotePlanningSettings(*message.planning_settings());
  }
    break;

  case (MSG_ID_SCENE_STORY_LIST):{
    const MessageSceneStoryList &message =
        dynamic_cast<const MessageSceneStoryList&>(msg);
    shared_data->SetSceneStoryList(*message.scene_story_list());
  }
    break;

  case (MSG_ID_MAP_LOCALIZATION):{
    const MessageMapLocalization &message =
        dynamic_cast<const MessageMapLocalization&>(msg);
    shared_data->SetMapLocalization(*message.map_localization());
  }
    break;

  // 接收CANFD/ETH发送的原始ADASIS CAN 消息，发布用于保存到ROSBAG，以便回放/回灌分析
  case (MSG_ID_MAP_ADASISV2_CAN_FRAME): {
    const MessageCanFrame& message = dynamic_cast<const MessageCanFrame&>(msg);
#if (!ENTER_PLAYBACK_MODE_ADASISV2)
    msg_sender_->SendADASISV2CanFrame(*message.can_frame());
#endif
  }
  break;

  // 接收ADASIS v2 POSITION CAN message : 自车相对路径起点的偏移
  case (MSG_ID_MAP_RECV_ADASISV2_POSITION): {
    const MessageRecvADASISV2Position& msg_data = dynamic_cast<const MessageRecvADASISV2Position&>(msg);
    // save a positon message object to shared data
    shared_data->SetADASISV2Position(*msg_data.position());

    // publish POSITION ROS message
    msg_sender_->SendADASISV2Position(*msg_data.position());

    // Monitor : POSITION 100ms
    // work_monitor_.FeedDog_RecvADASISV2Position();
  }
  break;

  // 接收ADASIS v2 PROFILE SHORT CAN message : 坡度及曲率
  case (MSG_ID_MAP_RECV_ADASISV2_PROFILE_SHORT): {
    const MessageRecvADASISV2ProfileShort& msg_data =
        dynamic_cast<const MessageRecvADASISV2ProfileShort&>(msg);
    // save a profile short message object to shared data
    shared_data->SetADASISV2ProfileShort(*msg_data.profile());

    // publish PROFILE SHORT ROS message
    msg_sender_->SendADASISV2ProfileShort(*msg_data.profile());

    // Monitor : PROFILE SHORT 100ms
    // work_monitor_.FeedDog_RecvADASISV2ProfileShort();
  }
  break;

  case (MSG_ID_MAP_ADASISV2_HORIZON): {
    const MessageADASISV2Horion& msg_data =
        dynamic_cast<const MessageADASISV2Horion&>(msg);

    msg_sender_->SendADASISV2Horizon(*msg_data.horizon());
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

