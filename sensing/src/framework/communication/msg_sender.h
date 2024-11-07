/******************************************************************************
 ** 消息发送模块
 ******************************************************************************
 *
 *  消息发送模块(发送报文到总线上)
 *
 *  @file       msg_sender.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_MSG_SENDER_H_
#define PHOENIX_FRAMEWORK_MSG_SENDER_H_

#include "utils/macros.h"
#include "os/mutex.h"
#include "os/thread.h"

#if (ENABLE_ROS_NODE)
#include "communication/ros_node.h"
#endif

#if (ENABLE_LCM_NODE)
#include "communication/lcm_node.h"
#endif

#if (ENABLE_UDP_NODE)
#include "communication/udp_node.h"
#endif

#include "common/task.h"
#include "common/message.h"
#include "src/dev_driver/can_dev/can_driver.h"


namespace phoenix {
namespace framework {


class MsgSender : public Task {
public:
  MsgSender(Task* manager);

#if (ENABLE_ROS_NODE)
  void SetRosNode(RosNode* node) {
    ros_node_ = node;
  }
#endif

#if (ENABLE_LCM_NODE)
  void SetLcmNode(LcmNode* node) {
    lcm_node_ = node;
  }
#endif

#if (ENABLE_UDP_NODE)
  void SetUdpNode(UdpNode* node) {
    udp_node_ = node;
  }
#endif

  bool Start();

  void SendImuData(const ad_msg::Imu imu);
  void SendGnssData(const ad_msg::Gnss gnss);

  void SendCanFrameList(const phoenix::can_dev::CanFrameList& can_frame_list);
  void SendCanFdFrameList(const phoenix::can_dev::CanFdFrameList& canfd_frame_list, Int32_t canfd_idx);

  void SendLaneMarkCameraList(const ad_msg::LaneMarkCameraList lane_mark_camera_list);
  void SendLaneCurbCameraList(const ad_msg::LaneMarkCameraList lane_curb_camera_list);
  void SendVisualControlLaneMarkCameraList(const ad_msg::LaneMarkCameraList lane_mark_camera_list);
  void SendObstacleCameraList(const ad_msg::ObstacleCameraList obstacle_camera_list, Int32_t camera_idx);
  void SendObstacleRadarList(const ad_msg::ObstacleRadarList& obj_list, Int32_t radar_idx);
  void SendObstacleLidarList(const ad_msg::ObstacleLidarList& obj_list, Int32_t lidar_idx);
  void SendLidarCloud(const ad_msg::LidarCloud& cloud, Int32_t lidar_idx);
  void SendTrafficSignalList(const ad_msg::TrafficSignalList& traffic_signal);
  void SendADHMIObstacleFusionList(const ad_msg::ObstacleList& objs_list); //增加融合结果发送
  void SendADHMIObstacleDebugFusionList(const ad_msg::ObstacleList& objs_list); //增加融合结果发送

  void SendVisualLaneMark(const visualization_msgs::MarkerArray &msg);
  void SendMaxieyeVisualMarkArray(const visualization_msgs::MarkerArray &msg);
  void SendArsVisualMarkArray(const visualization_msgs::MarkerArray &msg);
  void SendVisualControlVisualMarkArray(const visualization_msgs::MarkerArray &msg, Int32_t camera_idx);
  void SendAnngicRadarVisualMarkArray(const visualization_msgs::MarkerArray &msg, Int32_t radar_idx);
  void SendMpuStateData(const MpuState::MonitorMpuState& monitor_mpu_state);

private:
#if (ENABLE_ROS_NODE)
  RosNode* ros_node_;
#endif

#if (ENABLE_LCM_NODE)
  LcmNode* lcm_node_;
#endif

#if (ENABLE_UDP_NODE)
  UdpNode* udp_node_;
#endif

  common::os::Mutex lock_serialization_data_buff_;
  Char_t serialization_data_buff_[1*1024*1024];
};


}  // namespace framework
}  // namespace phoenix

#endif  // PHOENIX_FRAMEWORK_MSG_SENDER_H_

