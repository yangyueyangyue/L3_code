/******************************************************************************
 ** 消息接收模块
 ******************************************************************************
 *
 *  消息接收模块(从总线上接收报文)
 *
 *  @file       msg_receiver.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/msg_receiver.h"

#include "utils/com_utils.h"
#include "data_serialization.h"
#include "communication/parse_proto_msg.h"


namespace phoenix {
namespace framework {


#if (ENABLE_UDP_NODE)
static void UDP_RecvHdMap(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleHdMapMessageUdp(buf, buf_len);
}

static void UDP_RecvRouting(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleRoutingMessageUdp(buf, buf_len);
}

static void UDP_RecvGnss(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleGnssMessageUdp(buf, buf_len);
}

static void UDP_RecvImu(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleImuMessageUdp(buf, buf_len);
}

static void UDP_RecvChassis(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleChassisMessageUdp(buf, buf_len);
}

static void UDP_RecvSpecialChassisInfo(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleSpecialChassisInfoMessageUdp(buf, buf_len);
}

static void UDP_RecvChassisCtlCmd(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleChassisCtlCmdMessageUdp(buf, buf_len);
}

static void UDP_RecvLaneMarkCameraList(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleLaneMarkCameraListMessageUdp(buf, buf_len);
}

static void UDP_RecvObstacleCameraList(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleObstacleCameraListMessageUdp(buf, buf_len);
}

static void UDP_RecvObstacleRadarFront(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleObstacleRadarFrontMessageUdp(buf, buf_len);
}

static void UDP_RecvObstacleRadarFrontLeft(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleObstacleRadarFrontLeftMessageUdp(buf, buf_len);
}

static void UDP_RecvObstacleRadarFrontRight(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleObstacleRadarFrontRightMessageUdp(buf, buf_len);
}

static void UDP_RecvObstacleRadarRearLeft(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleObstacleRadarRearLeftMessageUdp(buf, buf_len);
}

static void UDP_RecvObstacleRadarRearRight(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleObstacleRadarRearRightMessageUdp(buf, buf_len);
}

static void UDP_RecvTrafficSignal(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleTrafficSignalListMessageUdp(buf, buf_len);
}

static void UDP_RecvRemotePlanningSettings(
    const Char_t *channel, const void *buf, Int32_t buf_len, void *user) {
  MsgReceiver* handler = (MsgReceiver*)user;
  handler->HandleRemotePlanningSettingsMsgUdp(buf, buf_len);
}
#endif

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
MsgReceiver::MsgReceiver(Task* manager) :
  Task(TASK_ID_MSG_RECEIVER, "Message Receiver", manager) {
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

void MsgReceiver::Configurate(const PlanningConfig& conf) {
  config_ = conf;
}

/******************************************************************************/
/** 启动消息接收模块
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动消息接收模块，向总线订阅需要的报文
 *
 *  <Attention>
 *       None
 *
 */
bool MsgReceiver::Start() {
  bool ret = true;

#if (ENABLE_ROS_NODE)
  /// Comunicate by ROS
  if (Nullptr_t != ros_node_) {
    ret = ros_node_->Subscribe(
          "map/hdmap", 1,
          &MsgReceiver::HandleHdMapMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"map/hdmap\".";
    }

    ret = ros_node_->Subscribe(
          "map/routing", 1,
          &MsgReceiver::HandleRoutingMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"map/routing\".";
    }

    ret = ros_node_->Subscribe(
          "localization/gnss", 1,
          &MsgReceiver::HandleGnssMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/gnss\".";
    }

    ret = ros_node_->Subscribe(
          "localization/imu", 1,
          &MsgReceiver::HandleImuMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

    ret = ros_node_->Subscribe(
          "control/chassis", 1,
          &MsgReceiver::HandleChassisMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }

    ret = ros_node_->Subscribe(
          "control/special_chassis_info", 1,
          &MsgReceiver::HandleSpecialChassisInfoMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/special_chassis_info\".";
    }

    ret = ros_node_->Subscribe(
          "control/chassis_ctl_cmd", 1,
          &MsgReceiver::HandleChassisCtlCmdMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis_ctl_cmd\".";
    }

    ret = ros_node_->Subscribe(
          "perception/lane_mark_camera", 1,
          &MsgReceiver::HandleLaneMarkCameraListMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/lane_mark_camera\".";
    }

    ret = ros_node_->Subscribe(
          "perception/obstacle_camera", 1,
          &MsgReceiver::HandleObstacleCameraListMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_camera\".";
    }

    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_0", 1,
          &MsgReceiver::HandleObstacleRadarFrontMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_0\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_1", 1,
          &MsgReceiver::HandleObstacleRadarFrontLeftMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_1\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_2", 1,
          &MsgReceiver::HandleObstacleRadarFrontRightMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_2\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_3", 1,
          &MsgReceiver::HandleObstacleRadarRearLeftMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_3\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_4", 1,
          &MsgReceiver::HandleObstacleRadarRearRightMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_4\".";
    }

    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_5", 1,
          &MsgReceiver::HandleSrr2DetectionsFrontLeftMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_5\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_6", 1,
          &MsgReceiver::HandleSrr2DetectionsFrontRightMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_6\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_7", 1,
          &MsgReceiver::HandleSrr2DetectionsRearLeftMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_7\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_8", 1,
          &MsgReceiver::HandleSrr2DetectionsRearRightMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_8\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_radar_9", 1,
          &MsgReceiver::HandleObstacleRadarRearMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_9\".";
    }

    ret = ros_node_->Subscribe(
          "perception/obstacle_lidar_0", 1,
          &MsgReceiver::HandleObstacleLidar0MessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_lidar_0\".";
    }
    ret = ros_node_->Subscribe(
          "perception/obstacle_lidar_1", 1,
          &MsgReceiver::HandleObstacleLidar1MessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_lidar_1\".";
    }

    if (config_.obj_filter_config.using_outside_obj_list) {
      ret = ros_node_->Subscribe(
            "perception/fusion", 1,
            &MsgReceiver::HandleObstacleListMessageRos,
            this);
      if (false == ret) {
        LOG_ERR << "Failed to subscribe to "
                   "\"perception/fusion\".";
      }
    }

    ret = ros_node_->Subscribe(
          "perception/traffic_signal", 1,
          &MsgReceiver::HandleTrafficSignalListMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/traffic_signal\".";
    }
    ret = ros_node_->Subscribe(
          "perception/traffic_light", 1,
          &MsgReceiver::HandleTrafficLightListMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/traffic_light\".";
    }

    ret = ros_node_->Subscribe(
          "map/map_localization", 1,
          &MsgReceiver::HandleMapLocalizationMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"map/map_localization\".";
    }

    ret = ros_node_->Subscribe(
          "map/scene_storys", 1,
          &MsgReceiver::HandleScenceStorysMessageRos,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"map/scene_storys\".";
    }

    // Add Planning Settings BY ziweiq for Ros
    ret = ros_node_->Subscribe(
          "hmi/planning/settings", 1,
          &MsgReceiver::HandleRemotePlanningSettingsMsgRos,
          this);    
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"hmi/planning/settings\"."; 
    }
#if (ENTER_PLAYBACK_MODE_ADHMI)
    // AdECU planning DeBug Result
    ret = ros_node_->Subscribe(
          "/adecu/spd_pln", 1,
          &MsgReceiver::HandleADECUPlanningDebugMsgRos,
          this);    
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"/adecu/spd_pln\"."; 
    }
#endif

#if (ENTER_PLAYBACK_MODE_ADASISV2)
  ret = ros_node_->Subscribe(
        "map/adasisv2_raw", 1,
        &MsgReceiver::HandleADASISV2RawCanRos,
        this);    
  if (false == ret) {
    LOG_ERR << "Failed to subscribe to "
                "\"/map/adasisv2_raw\"."; 
  }
#endif

  }
#endif  // #if (ENABLE_ROS_NODE)

#if (ENABLE_LCM_NODE)
  /// Comunicate by LCM
  if (Nullptr_t != lcm_node_) {
    ret = lcm_node_->Subscribe(
          "map/hdmap",
          &MsgReceiver::HandleHdMapMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"map/hdmap\".";
    }

    ret = lcm_node_->Subscribe(
          "map/routing",
          &MsgReceiver::HandleRoutingMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"map/routing\".";
    }

    ret = lcm_node_->Subscribe(
          "localization/gnss",
          &MsgReceiver::HandleGnssMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/gnss\".";
    }

    ret = lcm_node_->Subscribe(
          "localization/imu",
          &MsgReceiver::HandleImuMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

    ret = lcm_node_->Subscribe(
          "control/chassis",
          &MsgReceiver::HandleChassisMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }

    ret = lcm_node_->Subscribe(
          "control/special_chassis_info",
          &MsgReceiver::HandleSpecialChassisInfoMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/special_chassis_info\".";
    }

    ret = lcm_node_->Subscribe(
          "control/chassis_ctl_cmd",
          &MsgReceiver::HandleChassisCtlCmdMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis_ctl_cmd\".";
    }

    ret = lcm_node_->Subscribe(
          "perception/lane_mark_camera",
          &MsgReceiver::HandleLaneMarkCameraListMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/lane_mark_camera\".";
    }

    ret = lcm_node_->Subscribe(
          "perception/obstacle_camera",
          &MsgReceiver::HandleObstacleCameraListMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_camera\".";
    }

    ret = lcm_node_->Subscribe(
          "perception/obstacle_radar_0",
          &MsgReceiver::HandleObstacleRadarFrontMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_0\".";
    }
    ret = lcm_node_->Subscribe(
          "perception/obstacle_radar_1",
          &MsgReceiver::HandleObstacleRadarFrontLeftMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_1\".";
    }
    ret = lcm_node_->Subscribe(
          "perception/obstacle_radar_2",
          &MsgReceiver::HandleObstacleRadarFrontRightMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_2\".";
    }
    ret = lcm_node_->Subscribe(
          "perception/obstacle_radar_3",
          &MsgReceiver::HandleObstacleRadarRearLeftMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_3\".";
    }
    ret = lcm_node_->Subscribe(
          "perception/obstacle_radar_4",
          &MsgReceiver::HandleObstacleRadarRearRightMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_4\".";
    }

    ret = lcm_node_->Subscribe(
          "perception/traffic_signal",
          &MsgReceiver::HandleTrafficSignalListMessageLcm,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/traffic_signal\".";
    }
  }
#endif  // #if (ENABLE_LCM_NODE)

#if (ENABLE_UDP_NODE)
  if (Nullptr_t != udp_node_) {
    ret = udp_node_->Subscribe(
          "map/hdmap",
          &UDP_RecvHdMap,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"map/hdmap\".";
    }

    ret = udp_node_->Subscribe(
          "map/routing",
          &UDP_RecvRouting,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"map/routing\".";
    }

    ret = udp_node_->Subscribe(
          "localization/gnss",
          &UDP_RecvGnss,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/gnss\".";
    }

    ret = udp_node_->Subscribe(
          "localization/imu",
          &UDP_RecvImu,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"localization/imu\".";
    }

    ret = udp_node_->Subscribe(
          "control/chassis",
          &UDP_RecvChassis,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis\".";
    }

    ret = udp_node_->Subscribe(
          "control/special_chassis_info",
          &UDP_RecvSpecialChassisInfo,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/special_chassis_info\".";
    }

    ret = udp_node_->Subscribe(
          "control/chassis_ctl_cmd",
          &UDP_RecvChassisCtlCmd,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to \"control/chassis_ctl_cmd\".";
    }

    ret = udp_node_->Subscribe(
          "perception/lane_mark_camera",
          &UDP_RecvLaneMarkCameraList,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/lane_mark_camera\".";
    }

    ret = udp_node_->Subscribe(
          "perception/obstacle_camera",
          &UDP_RecvObstacleCameraList,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_camera\".";
    }

    ret = udp_node_->Subscribe(
          "perception/obstacle_radar_0",
          &UDP_RecvObstacleRadarFront,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_0\".";
    }

    ret = udp_node_->Subscribe(
          "perception/obstacle_radar_1",
          &UDP_RecvObstacleRadarFrontLeft,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_1\".";
    }

    ret = udp_node_->Subscribe(
          "perception/obstacle_radar_2",
          &UDP_RecvObstacleRadarFrontRight,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_2\".";
    }

    ret = udp_node_->Subscribe(
          "perception/obstacle_radar_3",
          &UDP_RecvObstacleRadarRearLeft,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_3\".";
    }

    ret = udp_node_->Subscribe(
          "perception/obstacle_radar_4",
          &UDP_RecvObstacleRadarRearRight,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/obstacle_radar_4\".";
    }

    ret = udp_node_->Subscribe(
          "perception/traffic_signal",
          &UDP_RecvTrafficSignal,
          this);
    if (false == ret) {
      LOG_ERR << "Failed to subscribe to "
                 "\"perception/traffic_signal\".";
    }

    ret = udp_node_->Subscribe(
          "hmi/planning/remote_settings",
          &UDP_RecvRemotePlanningSettings,
          this);
    if (!ret) {
      LOG_ERR << "Failed to subscribe \"hmi/planning/remote_settings\" from UDP Node.";
    }
  }
#endif  // #if (ENABLE_UDP_NODE)

  return (true);
}


#if (ENABLE_ROS_NODE)
/// Comunicate by ROS
void MsgReceiver::HandleHdMapMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    MessageMapData msg_map((const Char_t*)(&msg.data[0]), msg.data.size());

    Notify(msg_map);
  }
}

void MsgReceiver::HandleRoutingMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    MessageRoutingData msg_routing((const Char_t*)(&msg.data[0]), msg.data.size());

    Notify(msg_routing);
  }
}

void MsgReceiver::HandleGnssMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeGnssMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_gnss_info_)) {

      MessageGnss msg_gnss(&ros_gnss_info_);

      Notify(msg_gnss);
    }
  }
}

void MsgReceiver::HandleImuMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeImuMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_imu_info_)) {

      MessageImu msg_imu(&ros_imu_info_);

      Notify(msg_imu);
    }
  }
}

void MsgReceiver::HandleChassisMessageRos(const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(), &ros_chassis_info_)) {

      MessageChassis msg_chassis(&ros_chassis_info_);

      Notify(msg_chassis);
    }
  }
}

void MsgReceiver::HandleSpecialChassisInfoMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeSpecialChassisInfoMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_special_chassis_info_)) {

      MessageSpecialChassisInfo msg_chassis(&ros_special_chassis_info_);

      Notify(msg_chassis);
    }
  }
}

void MsgReceiver::HandleChassisCtlCmdMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisCtlCmdMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_chassis_ctl_cmd_info_)) {

      MessageChassisCtlCmd msg_chassis_ctl_cmd(&ros_chassis_ctl_cmd_info_);

      Notify(msg_chassis_ctl_cmd);
    }
  }
}

void MsgReceiver::HandleLaneMarkCameraListMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeLaneMarkCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_lane_mark_camera_list_)) {

      MessageLaneMarkCameraList msg_lane_mark(&ros_lane_mark_camera_list_);

      Notify(msg_lane_mark);
    }
  }
}

void MsgReceiver::HandleObstacleCameraListMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_camera_list_)) {

      MessageObstacleCameraList msg_obj_cam(&ros_obstacle_camera_list_);

      Notify(msg_obj_cam);
    }
  }
}

void MsgReceiver::HandleObstacleRadarFrontMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_radar_front_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_OBSTACLE_RADAR_FRONT_LIST, &ros_obstacle_radar_front_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleObstacleRadarRearMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_radar_rear_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_OBSTACLE_RADAR_REAR_LIST, &ros_obstacle_radar_rear_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleObstacleRadarFrontLeftMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_radar_front_left_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_OBSTACLE_RADAR_FRONT_LEFT_LIST,
            &ros_obstacle_radar_front_left_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleObstacleRadarFrontRightMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_radar_front_right_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_OBSTACLE_RADAR_FRONT_RIGHT_LIST,
            &ros_obstacle_radar_front_right_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleObstacleRadarRearLeftMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_radar_rear_left_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_OBSTACLE_RADAR_REAR_LEFT_LIST,
            &ros_obstacle_radar_rear_left_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleObstacleRadarRearRightMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_radar_rear_right_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_OBSTACLE_RADAR_REAR_RIGHT_LIST,
            &ros_obstacle_radar_rear_right_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleSrr2DetectionsFrontLeftMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_srr2_detections_front_left_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_SRR2_DETECTIONS_FRONT_LEFT_LIST,
            &ros_srr2_detections_front_left_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleSrr2DetectionsFrontRightMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_srr2_detections_front_right_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_SRR2_DETECTIONS_FRONT_RIGHT_LIST,
            &ros_srr2_detections_front_right_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleSrr2DetectionsRearLeftMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_srr2_detections_rear_left_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_SRR2_DETECTIONS_REAR_LEFT_LIST,
            &ros_srr2_detections_rear_left_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleSrr2DetectionsRearRightMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_srr2_detections_rear_right_list_)) {

      MessageObstacleRadarList msg_obj_radar(
            MSG_ID_SRR2_DETECTIONS_REAR_RIGHT_LIST,
            &ros_srr2_detections_rear_right_list_);

      Notify(msg_obj_radar);
    }
  }
}

void MsgReceiver::HandleObstacleLidar0MessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleLidarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_lidar_list_0_)) {

      MessageObstacleLidarList msg_obj(
            MSG_ID_OBSTACLE_LIDAR_LIST_0, &ros_obstacle_lidar_list_0_);

      Notify(msg_obj);
    }
  }
}

void MsgReceiver::HandleObstacleLidar1MessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleLidarListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_lidar_list_1_)) {

      MessageObstacleLidarList msg_obj(
            MSG_ID_OBSTACLE_LIDAR_LIST_1, &ros_obstacle_lidar_list_1_);

      Notify(msg_obj);
    }
  }
}

void MsgReceiver::HandleObstacleListMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_obstacle_list_)) {

      MessageObstacleList msg_obj(&ros_obstacle_list_);

      Notify(msg_obj);
    }
  }
}

void MsgReceiver::HandleTrafficSignalListMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeTrafficSignalListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_traffic_signal_list_)) {

      MessageTrafficSignalList  msg_traffic_signal(&ros_traffic_signal_list_);

      Notify(msg_traffic_signal);
    }
  }
}

void MsgReceiver::HandleTrafficLightListMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeTrafficLightListMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_traffic_light_)) {

      MessageTrafficLightList  msg_traffic_light(&ros_traffic_light_);

      Notify(msg_traffic_light);
    }
  }
}

void MsgReceiver::HandleScenceStorysMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeSceneStorysMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_scene_story_list_)) {

      MessageSceneStoryList msg_scene_storys(&ros_scene_story_list_);
      Notify(msg_scene_storys);
    }
  }
}

void MsgReceiver::HandleMapLocalizationMessageRos(
    const std_msgs::ByteMultiArray& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeMapLocalizationMessage(
          (const Char_t*)(&msg.data[0]), msg.data.size(),
          &ros_map_localization_)) {

      MessageMapLocalization msg_map_localization(&ros_map_localization_);
      Notify(msg_map_localization);
    }
  }
}

// Add Planning Settings BY ziweiq for Ros
void MsgReceiver::HandleRemotePlanningSettingsMsgRos(
    const ::planning::planning_setting& msg) {
    ros_planning_settings_.start_adas = msg.start_adas;
    ros_planning_settings_.enable_lka = msg.enable_lka;
    ros_planning_settings_.enable_acc = msg.enable_acc;
    ros_planning_settings_.enable_aeb = msg.enable_aeb;
    ros_planning_settings_.enable_alc = msg.enable_alc;
    ros_planning_settings_.enable_isl = msg.enable_isl;
    ros_planning_settings_.enable_ngp = msg.enable_ngp;
    ros_planning_settings_.target_velocity_valid = msg.target_velocity_valid;
    ros_planning_settings_.target_velocity = msg.target_velocity;
    ros_planning_settings_.changing_lane_req = msg.changing_lane_req;

    MessageRemotePlanningSettings msg_planning_settings(&ros_planning_settings_);
    ROS_INFO("Subcribe Settings Info: enable_lka:%d  enable_acc:%d  enable_aeb:%d  msg.enable_alc:%d  enable_isl:%d  enable_ngp:%d  target_velocity:%0.1f", 
             ros_planning_settings_.enable_lka, ros_planning_settings_.enable_acc, ros_planning_settings_.enable_aeb, ros_planning_settings_.enable_alc, 
                  ros_planning_settings_.enable_isl, ros_planning_settings_.enable_ngp, ros_planning_settings_.target_velocity * 3.6F);      
    Notify(msg_planning_settings);
}


void MsgReceiver::HandleADECUPlanningDebugMsgRos(
  const std_msgs::ByteMultiArray& msg){
  if (Nullptr_t == ros_node_) {
    return;
  }

  if (msg.data.size() > 0) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeAdecuPlanningMessage(
          (const Char_t*)(&msg.data[39]), msg.data.size()-39,
          &ADECU_vel_planning_debug_)) {

      MessageAdecuVelPlanningDebug msg_adecu_vel_planning_debug(&ADECU_vel_planning_debug_);
      Notify(msg_adecu_vel_planning_debug);
    }
  }
}

#if (ENTER_PLAYBACK_MODE_ADASISV2)
void MsgReceiver::HandleADASISV2RawCanRos(const ::planning::CanFrame& msg) {
  if (Nullptr_t == ros_node_) {
    return;
  }

  printf("[ADASIS-PLAYBACK] RAW : timestamp[%ld], id[0x%08X], data[%02X %02X %02X %02X %02X %02X %02X %02X]\n", msg.time_stamp, 
               msg.id, msg.data[0], msg.data[1], msg.data[2], msg.data[3],
            msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

  phoenix::can_dev::CanFrame can_frame;
  can_frame.time_stamp = msg.time_stamp;
  can_frame.id = msg.id;
  can_frame.RTR = msg.rtr;
  can_frame.EXT = msg.ext;
  can_frame.data_len = msg.data_len;
  for(Uint8_t i = 0; i < msg.data_len; i++) {
    can_frame.data[i] = msg.data[i];
  }

  phoenix::adasisv2::ADASISv2MsgParser parser;
  phoenix::adasisv2::MessageType msg_type = parser.ParseCanFrame(can_frame);

  if (phoenix::adasisv2::ADASIS_V2_MESSAGE_TYPE_POSITION == msg_type) {
    phoenix::adasisv2::PositionMessage position = parser.getPosition();
    LOG_INFO(3) << "[ADASIS] POSITION : " << "path = " << position.path_index << ", offset = " << position.offset << ", slope = " << position.slope;
    framework::MessageRecvADASISV2Position message(&position);
    Notify(message);
  } else if (phoenix::adasisv2::ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT == msg_type) {
    phoenix::adasisv2::ProfileShortMessage profile_short = parser.getProfileShort();
    LOG_INFO(3) << "[ADASIS] PROFILE SHORT : " << "type = " << profile_short.profile_type << ", path = " << profile_short.path_index << ", offset = " << profile_short.offset << ", value0 = " << profile_short.value0
                << ", distance1 = " << profile_short.distance1 << ", value1 = " << profile_short.value1;
    framework::MessageRecvADASISV2ProfileShort message(&profile_short);
    Notify(message);
  }
}
#endif

#endif // #if (ENABLE_ROS_NODE)


#if (ENABLE_LCM_NODE)
/// Communicate by LCM
void MsgReceiver::HandleHdMapMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    MessageMapData msg((const Char_t*)(rbuf->data), rbuf->data_size);

    Notify(msg);
  }
}

void MsgReceiver::HandleRoutingMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    MessageRoutingData msg((const Char_t*)(rbuf->data), rbuf->data_size);

    Notify(msg);
  }
}

void MsgReceiver::HandleGnssMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeGnssMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size, &lcm_gnss_info_)) {

      MessageGnss msg(&lcm_gnss_info_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleImuMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeImuMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size, &lcm_imu_info_)) {

      MessageImu msg(&lcm_imu_info_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleChassisMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size, &lcm_chassis_info_)) {

      MessageChassis msg(&lcm_chassis_info_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleSpecialChassisInfoMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeSpecialChassisInfoMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_special_chassis_info_)) {

      MessageSpecialChassisInfo msg(&lcm_special_chassis_info_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleChassisCtlCmdMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeChassisCtlCmdMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_chassis_ctl_cmd_info_)) {

      MessageChassisCtlCmd msg(&lcm_chassis_ctl_cmd_info_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleLaneMarkCameraListMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeLaneMarkCameraListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_lane_mark_camera_list_)) {

      MessageLaneMarkCameraList msg(&lcm_lane_mark_camera_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleObstacleCameraListMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleCameraListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_obstacle_camera_list_)) {

      MessageObstacleCameraList msg(&lcm_obstacle_camera_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleObstacleRadarFrontMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_obstacle_radar_front_list_)) {

      MessageObstacleRadarList msg(
            MSG_ID_OBSTACLE_RADAR_FRONT_LIST,
            &lcm_obstacle_radar_front_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleObstacleRadarFrontLeftMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_obstacle_radar_front_left_list_)) {

      MessageObstacleRadarList msg(
            MSG_ID_OBSTACLE_RADAR_FRONT_LEFT_LIST,
            &lcm_obstacle_radar_front_left_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleObstacleRadarFrontRightMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_obstacle_radar_front_right_list_)) {

      MessageObstacleRadarList msg(
            MSG_ID_OBSTACLE_RADAR_FRONT_RIGHT_LIST,
            &lcm_obstacle_radar_front_right_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleObstacleRadarRearLeftMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_obstacle_radar_rear_left_list_)) {

      MessageObstacleRadarList msg(
            MSG_ID_OBSTACLE_RADAR_REAR_LEFT_LIST,
            &lcm_obstacle_radar_rear_left_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleObstacleRadarRearRightMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeObstacleRadarListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_obstacle_radar_rear_right_list_)) {

      MessageObstacleRadarList msg(
            MSG_ID_OBSTACLE_RADAR_REAR_RIGHT_LIST,
            &lcm_obstacle_radar_rear_right_list_);

      Notify(msg);
    }
  }
}

void MsgReceiver::HandleTrafficSignalListMessageLcm(
    const lcm::ReceiveBuffer* rbuf, const std::string& channel) {
  if (Nullptr_t == lcm_node_) {
    return;
  }

  if ((Nullptr_t != rbuf->data) && (rbuf->data_size > 0)) {
    ParseProtoMsg parse_msg;
    if (parse_msg.DecodeTrafficSignalListMessage(
          (const Char_t*)(rbuf->data), rbuf->data_size,
          &lcm_traffic_signal_list_)) {

      MessageTrafficSignalList msg(&lcm_traffic_signal_list_);

      Notify(msg);
    }
  }
}

#endif // #if (ENABLE_LCM_NODE)


#if (ENABLE_UDP_NODE)
void MsgReceiver::HandleHdMapMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    MessageMapData msg((const Char_t*)(buf), buf_len);

    Notify(msg);
  }
}

void MsgReceiver::HandleRoutingMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    MessageRoutingData msg((const Char_t*)(buf), buf_len);

    Notify(msg);
  }
}

void MsgReceiver::HandleGnssMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeGnssArray(
          buf, 0, buf_len, &udp_gnss_info_, 1);

    // Update message head
    udp_gnss_info_.msg_head.valid = true;
    udp_gnss_info_.msg_head.UpdateSequenceNum();
    udp_gnss_info_.msg_head.timestamp = common::GetClockNowMs();

    MessageGnss msg(&udp_gnss_info_);
    Notify(msg);
  }
}

void MsgReceiver::HandleImuMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeImuArray(
          buf, 0, buf_len, &udp_imu_info_, 1);

    // Update message head
    udp_imu_info_.msg_head.valid = true;
    udp_imu_info_.msg_head.UpdateSequenceNum();
    udp_imu_info_.msg_head.timestamp = common::GetClockNowMs();

    MessageImu msg(&udp_imu_info_);
    Notify(msg);
  }
}

void MsgReceiver::HandleChassisMessageUdp(const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeChassisArray(
          buf, 0, buf_len, &udp_chassis_info_, 1);

    // Update message head
    udp_chassis_info_.msg_head.valid = true;
    udp_chassis_info_.msg_head.UpdateSequenceNum();
    udp_chassis_info_.msg_head.timestamp = common::GetClockNowMs();

    MessageChassis msg(&udp_chassis_info_);
    Notify(msg);
  }
}

void MsgReceiver::HandleSpecialChassisInfoMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeSpecialChassisInfoArray(
          buf, 0, buf_len, &udp_special_chassis_info_, 1);

    // Update message head
    udp_special_chassis_info_.msg_head.valid = true;
    udp_special_chassis_info_.msg_head.UpdateSequenceNum();
    udp_special_chassis_info_.msg_head.timestamp = common::GetClockNowMs();

    MessageSpecialChassisInfo msg(&udp_special_chassis_info_);
    Notify(msg);
  }
}

void MsgReceiver::HandleChassisCtlCmdMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeChassisCtlCmdArray(
          buf, 0, buf_len, &udp_chassis_ctl_cmd_info_, 1);

    // Update message head
    udp_chassis_ctl_cmd_info_.msg_head.valid = true;
    udp_chassis_ctl_cmd_info_.msg_head.UpdateSequenceNum();
    udp_chassis_ctl_cmd_info_.msg_head.timestamp = common::GetClockNowMs();

    MessageChassisCtlCmd msg(&udp_chassis_ctl_cmd_info_);
    Notify(msg);
  }
}

void MsgReceiver::HandleLaneMarkCameraListMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeLaneMarkCameraListArray(
          buf, 0, buf_len, &udp_lane_mark_camera_list_, 1);

    // Update message head
    udp_lane_mark_camera_list_.msg_head.valid = true;
    udp_lane_mark_camera_list_.msg_head.UpdateSequenceNum();
    udp_lane_mark_camera_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageLaneMarkCameraList msg(&udp_lane_mark_camera_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleObstacleCameraListMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeObstacleCameraListArray(
          buf, 0, buf_len, &udp_obstacle_camera_list_, 1);

    // Update message head
    udp_obstacle_camera_list_.msg_head.valid = true;
    udp_obstacle_camera_list_.msg_head.UpdateSequenceNum();
    udp_obstacle_camera_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageObstacleCameraList msg(&udp_obstacle_camera_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleObstacleRadarFrontMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeObstacleRadarListArray(
          buf, 0, buf_len, &udp_obstacle_radar_front_list_, 1);

    // Update message head
    udp_obstacle_radar_front_list_.msg_head.valid = true;
    udp_obstacle_radar_front_list_.msg_head.UpdateSequenceNum();
    udp_obstacle_radar_front_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageObstacleRadarList msg(
          MSG_ID_OBSTACLE_RADAR_FRONT_LIST,
          &udp_obstacle_radar_front_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleObstacleRadarFrontLeftMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeObstacleRadarListArray(
          buf, 0, buf_len, &udp_obstacle_radar_front_left_list_, 1);

    // Update message head
    udp_obstacle_radar_front_left_list_.msg_head.valid = true;
    udp_obstacle_radar_front_left_list_.msg_head.UpdateSequenceNum();
    udp_obstacle_radar_front_left_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageObstacleRadarList msg(
          MSG_ID_OBSTACLE_RADAR_FRONT_LEFT_LIST,
          &udp_obstacle_radar_front_left_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleObstacleRadarFrontRightMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeObstacleRadarListArray(
          buf, 0, buf_len, &udp_obstacle_radar_front_right_list_, 1);

    // Update message head
    udp_obstacle_radar_front_right_list_.msg_head.valid = true;
    udp_obstacle_radar_front_right_list_.msg_head.UpdateSequenceNum();
    udp_obstacle_radar_front_right_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageObstacleRadarList msg(
          MSG_ID_OBSTACLE_RADAR_FRONT_RIGHT_LIST,
          &udp_obstacle_radar_front_right_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleObstacleRadarRearLeftMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeObstacleRadarListArray(
          buf, 0, buf_len, &udp_obstacle_radar_rear_left_list_, 1);

    // Update message head
    udp_obstacle_radar_rear_left_list_.msg_head.valid = true;
    udp_obstacle_radar_rear_left_list_.msg_head.UpdateSequenceNum();
    udp_obstacle_radar_rear_left_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageObstacleRadarList msg(
          MSG_ID_OBSTACLE_RADAR_REAR_LEFT_LIST,
          &udp_obstacle_radar_rear_left_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleObstacleRadarRearRightMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeObstacleRadarListArray(
          buf, 0, buf_len, &udp_obstacle_radar_rear_right_list_, 1);

    // Update message head
    udp_obstacle_radar_rear_right_list_.msg_head.valid = true;
    udp_obstacle_radar_rear_right_list_.msg_head.UpdateSequenceNum();
    udp_obstacle_radar_rear_right_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageObstacleRadarList msg(
          MSG_ID_OBSTACLE_RADAR_REAR_RIGHT_LIST,
          &udp_obstacle_radar_rear_right_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleTrafficSignalListMessageUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodeTrafficSignalListArray(
          buf, 0, buf_len, &udp_traffic_signal_list_, 1);

    // Update message head
    udp_traffic_signal_list_.msg_head.valid = true;
    udp_traffic_signal_list_.msg_head.UpdateSequenceNum();
    udp_traffic_signal_list_.msg_head.timestamp = common::GetClockNowMs();

    MessageTrafficSignalList msg(&udp_traffic_signal_list_);
    Notify(msg);
  }
}

void MsgReceiver::HandleRemotePlanningSettingsMsgUdp(
    const void *buf, Int32_t buf_len) {
  if (Nullptr_t == udp_node_) {
    return;
  }

  if ((Nullptr_t != buf) && (buf_len > 0)) {
    data_serial::DecodePlanningSettingsArray(
          buf, 0, buf_len, &udp_planning_settings_, 1);

    // Update message head
    //udp_hmi_settings_.msg_head.valid = true;
    //udp_hmi_settings_.msg_head.UpdateSequenceNum();
    //udp_hmi_settings_.msg_head.timestamp = common::GetClockNowMs();

    MessageRemotePlanningSettings msg(&udp_planning_settings_);
    Notify(msg);
  }
}
#endif // #if (ENABLE_UDP_NODE)


}  // namespace framework
}  // namespace phoenix
