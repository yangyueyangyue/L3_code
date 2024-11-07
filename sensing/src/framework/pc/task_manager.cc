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
#endif
#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif
#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif
#include "communication/msg_receiver.h"
#include "communication/msg_sender.h"
#include "communication/shared_data.h"
#include "utils/log.h"
#include "utils/com_utils.h"
#include "utils/gps_tools.h"
#include "MpuState.pb.h"


#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
#include "sensor/imu/mpsk/task_recv_gnss_data_mpsk.h"
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
#include "sensor/imu/bdstar/task_recv_gnss_data_bdstar.h"
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
#include "sensor/imu/intertial_lab/task_recv_gnss_intertial_lab.h"
#else
  // 未定义IMU设备
#endif

#include "sensor/task_recv_ars_data_front.h"
#include "sensor/task_recv_maxieye_data.h"
#include "sensor/task_recv_lidar_data_front.h"
#include "sensor/task_recv_anngic_radar_data.h"
#include "sensor/task_recv_mpu_data.h"
#include "sensor/task_save_data_to_csv.h"

#if (ENABLE_CAN_DEV_KVASER)
#include "can_dev/kvaser/can_driver_kvaser.h"
#endif
#if (ENABLE_CAN_DEV_EMUC2)
#include "can_dev/emuc2/can_driver_emuc2.h"
#endif
#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet.h"
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif
#if (ENABLE_CAN_DEV_MDC)
#include "can_dev/mdc_can/can_driver_mdc.h"
#endif

#include "radar/filter_srr2_detections.h"

#include "uisetting_manager.h"


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
TaskManager::TaskManager(int argc, char** argv, const std::string &work_space)
  : Task(TASK_ID_MANAGER, "Task Manager")
  , work_space_(work_space) {
  //, log_file_status_("status") {
  thread_running_flag_check_tasks_status_ = false;

#if (ENABLE_ROS_NODE)
  ros_node_.reset(new RosNode(argc, argv, "sensing"));
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

#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
  task_recv_gnss_data_.reset(new sensor::imu::mpsk::TaskRecvGnssDataMpsk(this));
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
  task_recv_gnss_data_.reset(new sensor::imu::bdstar::TaskRecvGnssDataBdstar(this));
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
  task_recv_gnss_data_.reset(new sensor::imu::intertial_lab::TaskRecvGnssIntertialLab(this));
#else
  // 未定义IMU设备
#endif

  task_recv_ars_data_front_.reset(new sensor::TaskRecvArsDataFront(this));
  task_recv_maxieye_data_.reset(new sensor::TaskRecvMaxieyeData(this));
  task_recv_lidar_data_front_.reset(new sensor::TaskRecvLidarDataFront(this));
  task_recv_anngic_radar_data_.reset(new sensor::TaskRecvAnngicRadarData(this));
  task_recv_mpu_data_.reset(new sensor::TaskRecvMpuData(this));

#if (ENTER_OBJ_INFO_TO_CSV)
  task_save_data_to_csv_.reset(new sensor::TaskSaveDataToCsv(this));
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
Int32_t TaskManager::Start() {
  bool ret = false;

  SharedData* shared_data = SharedData::instance();

#if (ENABLE_CAN_DEV_KVASER)
  ret = can_dev::CanDriverKvaser::OpenDevice();
  if (false == ret) {
    LOG_ERR << "Failed to open CAN Device: Kvaser.";
  }
#endif

#if (ENABLE_CAN_DEV_EMUC2)
  ret = can_dev::CanDriverEmuc2::OpenDevice();
  if (false == ret) {
    LOG_ERR << "Failed to open CAN Device: Emuc2.";
  }
#endif

#if (ENABLE_CAN_DEV_ZLGCANNET)
  ret = can_dev::CanDriverZlgCanNet::OpenDevice();
  if (false == ret) {
    LOG_ERR << "Failed to open CAN Device: ZLG CAN Net.";
  }
  ret = can_dev::CanDriverZlgCanNetFd::OpenDevice();
  if (false == ret) {
    LOG_ERR << "Failed to open CAN Device: ZLG CAN Net.";
  }
#endif

#if (ENABLE_CAN_DEV_MDC)
  ret = can_dev::CanDriverMdc::OpenDevice();
  if (false == ret) {
    LOG_ERR << "Failed to open CAN Device: MDC.";
  }
#endif

#if (ENABLE_ROS_NODE)
  // Start ROS Node
  ret = ros_node_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start ROS node.";
    return (START_ERR_FAILED_TO_START_ROS);
  }
#endif

#if (ENABLE_LCM_NODE)
  //Start LCM Node
  ret = lcm_node_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start LCM node.";
    return (START_ERR_FAILED_TO_START_LCM);
  }
#endif

#if (ENABLE_UDP_NODE)
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
    return (START_ERR_FAILED_TO_START_UDP);
  }
#endif

  // Start Message receive
  ret = msg_receiver_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message receiver.";
    return (START_ERR_FAILED_TO_START_MSG_RECV);
  }

  // Start Message send
  ret = msg_sender_->Start();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start message sender.";
    return (START_ERR_FAILED_TO_START_MSG_SEND);
  }

#if (ENTER_OBJ_INFO_TO_CSV)
  // Start Save Data To Csv
  ret = task_save_data_to_csv_->Start();
  if (false == ret) {
    //Stop();
    LOG_ERR << "Failed to start save data to csv.";
    //return (START_ERR_FAILED_TO_START_MSG_SEND);
  }
#endif

#if (!ENTER_PLAYBACK_MODE)
  // DF-D17
  ret = task_recv_ars_data_front_->Start();
  if (false == ret) {
    //Stop();
    LOG_ERR << "Failed to start receiving front ars data.";
    //return (START_ERR_FAILED_TO_START_ESR);
  }

  // DF-D17
  ret = task_recv_maxieye_data_->Start();
  if (false == ret) {
    //Stop();
    LOG_ERR << "Failed to start receiving maxieye data.";
    //return (START_ERR_FAILED_TO_START_MOBILEYE);
  }

  // DF-D17 lidar
  //ret = task_recv_lidar_data_front_->Start();
  if (false == ret) {
    LOG_ERR << "Failed to start receiving lidar front data.";
  }

  // DF-D17 Anngic Radar
  ret = task_recv_anngic_radar_data_->Start();
  if (false == ret) {
    LOG_ERR << "Failed to start receiving anngic radar data.";
  }  

#endif  // #if (!ENTER_PLAYBACK_MODE)

  // mpu data
  ret = task_recv_mpu_data_->Start();
  if (false == ret) {
    LOG_ERR << "Failed to start receiving mpu data.";
  }  

#if (DEV_IMU_TYPE != DEV_IMU_TYPE_UNDEFINED)
  ret = task_recv_gnss_data_->Start();
  if(false == ret) {
    //Stop();
    LOG_ERR<<"Failed to start receiving gnss data.";
    //return (START_ERR_FAILED_TO_START_GNSS);
  }
#endif

#if (ENABLE_UDP_NODE)
  ret = udp_node_->StartReceiving();
  if (false == ret) {
    Stop();
    LOG_ERR << "Failed to start UDP receiving.";
    return (START_ERR_FAILED_TO_START_UDP);
  }
#endif

  // Start Thread(Check tasks status)
  if (true != thread_running_flag_check_tasks_status_) {
    LOG_INFO(3) << "Start Check Tasks Status Thread ...";
    thread_running_flag_check_tasks_status_ = true;
    thread_check_tasks_status_ =
        boost::thread(boost::bind(&TaskManager::ThreadCheckTasksStatus, this));
    LOG_INFO(3) << "Start Check Tasks Status Thread ... [OK]";
  }

  return (START_OK);
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


#if (DEV_IMU_TYPE != DEV_IMU_TYPE_UNDEFINED)
  task_recv_gnss_data_->Stop();
#endif

#if (ENTER_OBJ_INFO_TO_CSV)
  task_save_data_to_csv_->Stop();
#endif

#if (!ENTER_PLAYBACK_MODE)
  task_recv_ars_data_front_->Stop();
  task_recv_maxieye_data_->Stop();
  task_recv_lidar_data_front_->Stop();
  task_recv_anngic_radar_data_->Stop();
  task_recv_mpu_data_->Stop();
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

  bool ret = false;
#if (ENABLE_CAN_DEV_KVASER)
  ret = can_dev::CanDriverKvaser::CloseDevice();
  if (can_dev::CanDriverKvaser::CloseDevice()) {
    LOG_ERR << "Failed to close CAN Device: Kvaser.";
  }
#endif
#if (ENABLE_CAN_DEV_EMUC2)
  ret = can_dev::CanDriverEmuc2::CloseDevice();
  if (false == ret) {
    LOG_ERR << "Failed to close CAN Device: Emuc2.";
  }
#endif
#if (ENABLE_CAN_DEV_ZLGCANNET)
  ret = can_dev::CanDriverZlgCanNet::CloseDevice();
  if (false == ret) {
    LOG_ERR << "Failed to close CAN Device: ZLG CAN Net.";
  }
#endif
#if (ENABLE_CAN_DEV_MDC)
  ret = can_dev::CanDriverMdc::CloseDevice();
  if (false == ret) {
    LOG_ERR << "Failed to close CAN Device: MDC.";
  }
#endif

  return (true);
}


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
  case MSG_ID_RECV_GNSS_DATA: {
    const MessageRecvGnssData &message =
        dynamic_cast<const MessageRecvGnssData &>(msg);
    shared_data->SetGnssData(*message.gnss());
    msg_sender_->SendGnssData(*message.gnss());

    // Monitor
    work_monitor_.FeedDog_RecvGnss();
  }
    break;

  case MSG_ID_RECV_IMU_DATA: {
    const MessageRecvImuData &message =
        dynamic_cast<const MessageRecvImuData &>(msg);
    shared_data->SetImuData(*message.imu());
    msg_sender_->SendImuData(*message.imu());

    // Monitor
    work_monitor_.FeedDog_RecvImu();
  }
    break;

  case MSG_ID_RECV_ARS_RADAR_CAN_FRAME: {
    const MessageCanFrameList& message = 
        dynamic_cast<const MessageCanFrameList&>(msg);
    // shared_data->SetArsRadarCanFrameList(*message.can_frame_list());
    #if (!ENTER_PLAYBACK_MODE)
    msg_sender_->SendCanFrameList(*message.can_frame_list());
    #endif
  }
    break;

  case MSG_ID_RECV_CAMERA_CANFD_FRAME: {
    const MessageCanFdFrameList& message = 
        dynamic_cast<const MessageCanFdFrameList&>(msg);
    // shared_data->SetCameraCanFdFrameList(*message.canfd_frame_list());
    #if (!ENTER_PLAYBACK_MODE)
    msg_sender_->SendCanFdFrameList(*message.canfd_frame_list(), 0);
    #endif
  }
    break;
  
  case MSG_ID_RECV_ANNGIC_RADAR_CANFD_FRAME: {
    const MessageCanFdFrameList& message = 
        dynamic_cast<const MessageCanFdFrameList&>(msg);
    #if (!ENTER_PLAYBACK_MODE)
    msg_sender_->SendCanFdFrameList(*message.canfd_frame_list(), 2);
    #endif
  }
    break;

  case MSG_ID_RECV_LIDAR_CANFD_FRAME: {
    const MessageCanFdFrameList& message = 
        dynamic_cast<const MessageCanFdFrameList&>(msg);
    #if (!ENTER_PLAYBACK_MODE)
    msg_sender_->SendCanFdFrameList(*message.canfd_frame_list(), 1);
    #endif
  }
    break;

  case MSG_ID_LANE_MARK_CAMERA_LIST: {
    const MessageLaneMarkCameraList& message =
        dynamic_cast<const MessageLaneMarkCameraList&>(msg);
    shared_data->SetLaneMarkCameraList(*message.lane_mark_camera_list());
    msg_sender_->SendLaneMarkCameraList(*message.lane_mark_camera_list());

    // Monitor
    work_monitor_.FeedDog_RecvLaneMarkCamera();
  }
    break;

  case MSG_ID_LANE_CURB_CAMERA_LIST: {
    const MessageLaneCurbCameraList& message =
        dynamic_cast<const MessageLaneCurbCameraList&>(msg);
    shared_data->SetLaneCurbCameraList(*message.lane_curb_camera_list());
    msg_sender_->SendLaneCurbCameraList(*message.lane_curb_camera_list());

    // Monitor
    work_monitor_.FeedDog_RecvLaneCurbCamera();
  }
    break;

  // case (MSG_ID_LANE_VISUAL_MARK_ARRAY) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);

  //   msg_sender_->SendVisualLaneMark(message.obstacle_list());
  // }
  //   break;

  case MSG_ID_OBSTACLE_CAMERA_LIST: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleCameraList(*message.obstacle_camera_list());
    //msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleCamera();
  }
    break;

  case MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT: {
    const MessageObstacleCameraList& message = 
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleMaxieyeCameraList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 0);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleMaxieyeCamera();
    
  }
   break;

  // case (MSG_ID_MAXIEYE_VISUAL_MARK_ARRAY) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendMaxieyeVisualMarkArray(message.obstacle_list());
  // }
  //   break;

  case MSG_ID_RECV_LIDAR_OBJECTS_FRONT: {
    const MessageRecvLidarObjects& message = 
        dynamic_cast<const MessageRecvLidarObjects&>(msg);
    shared_data->SetObstacleLidarFrontList(*message.obstacle_list());
    msg_sender_->SendObstacleLidarList(*message.obstacle_list(), 0);

    // Monitor
    work_monitor_.FeedDog_RecvObstaclesLidarFront();
  }
  break;

  case MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT: {
    const MessageRecvRadarData& message = 
        dynamic_cast<const MessageRecvRadarData&>(msg);
    shared_data->SetObstacleAnngicRadarFrontLeftList(*message.obstacle_list());
    msg_sender_->SendObstacleRadarList(*message.obstacle_list(), 1);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleAnngicRadarFrontLeft();
  }
  break;

  case MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT: {
    const MessageRecvRadarData& message = 
        dynamic_cast<const MessageRecvRadarData&>(msg);
    shared_data->SetObstacleAnngicRadarFrontRightList(*message.obstacle_list());
    msg_sender_->SendObstacleRadarList(*message.obstacle_list(), 2);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleAnngicRadarFrontRight();
  }
  break;

  case MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT: {
    const MessageRecvRadarData& message = 
        dynamic_cast<const MessageRecvRadarData&>(msg);
    shared_data->SetObstacleAnngicRadarRearLeftList(*message.obstacle_list());
    msg_sender_->SendObstacleRadarList(*message.obstacle_list(), 3);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleAnngicRadarRearLeft();
  }
  break;
  
  case MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT: {
    const MessageRecvRadarData& message = 
        dynamic_cast<const MessageRecvRadarData&>(msg);
    shared_data->SetObstacleAnngicRadarRearRightList(*message.obstacle_list());
    msg_sender_->SendObstacleRadarList(*message.obstacle_list(), 4);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleAnngicRadarRearRight();
  }
  break;
  
  // case (MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_FRONT_LEFT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendAnngicRadarVisualMarkArray(message.obstacle_list(), 0);
  // }
  //   break;

  // case (MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_FRONT_RIGHT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendAnngicRadarVisualMarkArray(message.obstacle_list(), 1);
  // }
  //   break;

  // case (MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_REAR_LEFT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendAnngicRadarVisualMarkArray(message.obstacle_list(), 2);
  // }
  //   break;

  // case (MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_REAR_RIGHT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendAnngicRadarVisualMarkArray(message.obstacle_list(), 3);
  // }
  //   break;

  case MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST: {
    const MessageVisualControlLaneMarkCameraList& message =
        dynamic_cast<const MessageVisualControlLaneMarkCameraList&>(msg);
    shared_data->SetVisualControlLaneMarkCameraList(*message.lane_mark_camera_list());
    msg_sender_->SendVisualControlLaneMarkCameraList(*message.lane_mark_camera_list());

    // Monitor
    work_monitor_.FeedDog_RecvVisualControlLaneMarkCamera();
  }
   break;

  case MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleVisualControlFrontList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 1);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleVisualControlFront();
  }
    break;
  
  case MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleVisualControlFrontLeftList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 2);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleVisualControlFrontLeft();
  }
    break;
    
  case MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleVisualControlFrontRightList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 3);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleVisualControlFrontRight();
  }
    break;

  case MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleVisualControlRearLeftList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 4);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleVisualControlRearLeft();
  }
    break;
    
  case MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleVisualControlRearRightList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 5);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleVisualControlRearRight();
  }
    break;

  case MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR: {
    const MessageObstacleCameraList& message =
        dynamic_cast<const MessageObstacleCameraList&>(msg);
    shared_data->SetObstacleVisualControlRearList(*message.obstacle_camera_list());
    msg_sender_->SendObstacleCameraList(*message.obstacle_camera_list(), 6);

    // Monitor
    // work_monitor_.FeedDog_RecvObstacleVisualControlRear();
  }
    break;
  
  // case (MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_FRONT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendVisualControlVisualMarkArray(message.obstacle_list(), 0);
  // }
  //   break;

  // case (MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_FRONT_LEFT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendVisualControlVisualMarkArray(message.obstacle_list(), 1);
  // }
  //   break;

  // case (MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_FRONT_RIGHT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendVisualControlVisualMarkArray(message.obstacle_list(), 2);
  // }
  //   break;

  // case (MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_REAR_LEFT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendVisualControlVisualMarkArray(message.obstacle_list(), 3);
  // }
  //   break;

  // case (MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_REAR_RIGHT) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendVisualControlVisualMarkArray(message.obstacle_list(), 4);
  // }
  //   break;

  // case (MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_REAR) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);
  //   msg_sender_->SendVisualControlVisualMarkArray(message.obstacle_list(), 5);
  // }
  //   break;

  case (MSG_ID_RECV_ESR_DATA_FRONT): {
    const MessageRecvRadarData& msg_data =
        dynamic_cast<const MessageRecvRadarData&>(msg);
    shared_data->SetObstacleEsrFrontList(*msg_data.obstacle_list());
    msg_sender_->SendObstacleRadarList(*msg_data.obstacle_list(), 0);

    // Monitor
    work_monitor_.FeedDog_RecvObstacleEsrFront();
  }
    break;

  // case (MSG_ID_ARS_VISUAL_MARK_ARRAY) : {
  //   const MessageVisualMarkArray& message = dynamic_cast<const MessageVisualMarkArray&>(msg);

  //   msg_sender_->SendArsVisualMarkArray(message.obstacle_list());
  // }
  //   break;

  case (MSG_ID_TRAFFIC_SIGNAL_LIST): {
    const MessageTrafficSignalList &message =
        dynamic_cast < const MessageTrafficSignalList &>(msg);
    shared_data->SetTrafficSignalList(*message.traffic_signal_list());
    msg_sender_->SendTrafficSignalList(*message.traffic_signal_list());
  }
    break;

  case (MSG_ID_CHASSIS): {
    const MessageChassis& message =
        dynamic_cast<const MessageChassis&>(msg);

    shared_data->SetChassis(*message.chassis());

    // Monitor
    work_monitor_.FeedDog_RecvChassis();
  }
    break;

  case (MSG_ID_RECV_SENSORS_FUSION_DATA): {
    const MessageRecvFusionObstacleList& message = 
        dynamic_cast<const MessageRecvFusionObstacleList&>(msg);

    shared_data->SetObstaclesFusionList(*message.fusion_obstacle_list());

    // Monitor
    work_monitor_.FeedDog_RecvObstacleFusionObj();
  }
    break;

  case (MSG_ID_RECV_MPU_STATE_DATA): {
    const MessageMpuStateData& message =
        dynamic_cast<const MessageMpuStateData&>(msg);
    shared_data->SetMpuState(*message.mpu_state());
    

    msg_sender_->SendMpuStateData(*message.mpu_state());
    
    // Monitor
    work_monitor_.FeedDog_RecvMpuState();
  }
    break;

  case (MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA): {
    const MessageRecvADHMIFusionObstacleList& message = 
        dynamic_cast<const MessageRecvADHMIFusionObstacleList&>(msg);

    shared_data->SetADHMIObstaclesFusionList(*message.adhmi_fusion_obstacle_list());

    #if (ENTER_PLAYBACK_MODE)
      // #if (ENTER_PLAYBACK_MODE_AD_HMI_FUSION)

      if(hmi::UISettingManager::instance()->GetSettingPublishFusionTopicFromAdecu())
      {
        msg_sender_->SendADHMIObstacleFusionList(*message.adhmi_fusion_obstacle_list());
      }
      // #endif
      msg_sender_->SendADHMIObstacleDebugFusionList(*message.adhmi_fusion_obstacle_list());
    #endif

    // Monitor
    work_monitor_.FeedDog_RecvADHMIObstacleFusionObj();
  }
    break;
    
  case (MSG_ID_RECV_ADHMI_CLOCK): {
    const MessageRecvADHMIClock& message = 
        dynamic_cast<const MessageRecvADHMIClock&>(msg);

    shared_data->SetADHMIClock(*message.adhmi_clock());

    // Monitor
    //work_monitor_.FeedDog_RecvADHMIObstacleFusionObj();
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

