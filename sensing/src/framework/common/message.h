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
#include "ad_msg.h"
#include "common/module_status.h"
#include "src/dev_driver/can_dev/can_driver.h"
#include <rosgraph_msgs/Clock.h>

#include <visualization_msgs/MarkerArray.h> 

#include "MpuState.pb.h"

#include "sensing/perception_debug.h"

namespace phoenix {
namespace framework {


enum MessageId {
  MSG_ID_INVALID = 0,
  MSG_ID_REQ_DO_TASK,
  MSG_ID_LANE_MARK_CAMERA_LIST,
  MSG_ID_LANE_CURB_CAMERA_LIST,
  MSG_ID_OBSTACLE_CAMERA_LIST,
  MSG_ID_TRAFFIC_SIGNAL_LIST,
  MSG_ID_RECV_ESR_DATA_FRONT,
  MSG_ID_RECV_IMU_DATA,
  MSG_ID_RECV_GNSS_DATA,
  MSG_ID_CHASSIS,

  MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT,
  MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST,
  MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT,
  MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT,
  MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT,
  MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT,
  MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT,
  MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR,
  MSG_ID_RECV_LIDAR_OBJECTS_FRONT,
  MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT,
  MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT,
  MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT,
  MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT,

  MSG_ID_RECV_CAMERA_CANFD_FRAME,
  MSG_ID_RECV_ANNGIC_RADAR_CANFD_FRAME,
  MSG_ID_RECV_LIDAR_CANFD_FRAME,
  MSG_ID_RECV_ARS_RADAR_CAN_FRAME,

  MSG_ID_RECV_SENSORS_FUSION_DATA,
  MSG_ID_LANE_VISUAL_MARK_ARRAY,
  MSG_ID_MAXIEYE_VISUAL_MARK_ARRAY,
  MSG_ID_ARS_VISUAL_MARK_ARRAY,
  MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_FRONT,
  MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_FRONT_LEFT,
  MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_FRONT_RIGHT,
  MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_REAR,
  MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_REAR_LEFT,
  MSG_ID_VISUAL_CONTROL_VISUAL_MARK_ARRAY_REAR_RIGHT,
  MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_FRONT_LEFT,
  MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_FRONT_RIGHT,
  MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_REAR_LEFT,
  MSG_ID_ANNGIC_RADAR_VISUAL_MARK_ARRAY_REAR_RIGHT,

  MSG_ID_RECV_MPU_STATE_DATA,

  MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA,
  MSG_ID_RECV_ADHMI_CLOCK
};

enum TaskId {
  TASK_ID_INVALID = 0,
  TASK_ID_MANAGER,
  TASK_ID_MSG_RECEIVER,
  TASK_ID_MSG_SENDER,
  TASK_ID_RECV_ESR_DATA_FRONT,
  TASK_ID_RECV_ARS_DATA_FRONT,
  TASK_ID_RECV_MAXIEYE_DATA,
  TASK_ID_RECV_IBEO_DATA,
  TASK_ID_RECV_IMU_DATA,
  TASK_ID_RECV_GNSS_DATA,
  TASK_ID_RECV_LIDAR_DATA_FRONT,
  TASK_ID_RECV_ANNGIC_DATA,
  TASK_ID_SAVE_DATA_TO_CSV
};

class Message {
public:
  explicit Message(int id) : id_(id) {}
  virtual ~Message() = default;

  Message(const Message& other) {
    id_ = other.id_;
  }

  void operator =(const Message& other) {
    id_ = other.id_;
  }

  int id() const { return (id_); }

private:
  int id_;
};

class MessageModuleStatus : public Message {
public:
  explicit MessageModuleStatus(int id) : Message(id) {
    status_ = ad_msg::MODULE_STATUS_OK;
    common::com_memset(param_, 0, sizeof(param_));
  }

  void set_status(int status) { status_ = status; }
  int status() const { return (status_); }

  void set_param0(int param) { param_[0] = param; }
  int param0() const { return (param_[0]); }
  void set_param1(int param) { param_[1] = param; }
  int param1() const { return (param_[1]); }

private:
  int status_;
  int param_[4];
};

class MessageCanFrameList : public Message {
public:
  MessageCanFrameList(Int32_t msg_id, const phoenix::can_dev::CanFrameList *msg) :
    Message(msg_id),
    can_frame_list_(msg) {
  }

  const phoenix::can_dev::CanFrameList* can_frame_list() const { return (can_frame_list_); }

private:
  const phoenix::can_dev::CanFrameList* can_frame_list_ =Nullptr_t;
};

class MessageCanFdFrameList : public Message {
public:
  MessageCanFdFrameList(Int32_t msg_id, const phoenix::can_dev::CanFdFrameList *msg) :
    Message(msg_id),
    canfd_frame_list_(msg) {
  }

  const phoenix::can_dev::CanFdFrameList* canfd_frame_list() const { return (canfd_frame_list_); }

private:
  const phoenix::can_dev::CanFdFrameList* canfd_frame_list_ =Nullptr_t;
};

// Sensor Data Visualization (start)
class MessageVisualMarkArray : public Message {
public:
  MessageVisualMarkArray(Int32_t msg_id, const visualization_msgs::MarkerArray &obj_list) : 
    Message(msg_id),
    obstacle_list_(obj_list) {
  }
  const visualization_msgs::MarkerArray obstacle_list() const { return obstacle_list_; } ;
private:
  const visualization_msgs::MarkerArray obstacle_list_;
};
// Sensor Data Visualization (end)

class MessageLaneMarkCameraList : public Message {
public:
  MessageLaneMarkCameraList(const ad_msg::LaneMarkCameraList* msg) :
    Message(MSG_ID_LANE_MARK_CAMERA_LIST),lane_mark_camera_list_(msg) {
  }

  const ad_msg::LaneMarkCameraList* lane_mark_camera_list() const { return (lane_mark_camera_list_); }
  //ad_msg::LaneMarkCameraList& lane_mark_camera_list() { return (lane_mark_camera_list_); }

private:
  const ad_msg::LaneMarkCameraList* lane_mark_camera_list_=Nullptr_t;
};

class MessageLaneCurbCameraList : public Message {
public:
  MessageLaneCurbCameraList(const ad_msg::LaneMarkCameraList* msg) :
    Message(MSG_ID_LANE_CURB_CAMERA_LIST),lane_curb_camera_list_(msg) {
  }

  const ad_msg::LaneMarkCameraList* lane_curb_camera_list() const { return (lane_curb_camera_list_); }
  //ad_msg::LaneMarkCameraList& lane_mark_camera_list() { return (lane_mark_camera_list_); }

private:
  const ad_msg::LaneMarkCameraList* lane_curb_camera_list_=Nullptr_t;
};

class MessageVisualControlLaneMarkCameraList : public Message {
public:
  MessageVisualControlLaneMarkCameraList(const ad_msg::LaneMarkCameraList* msg) :
    Message(MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST),lane_mark_camera_list_(msg) {
  }

  const ad_msg::LaneMarkCameraList* lane_mark_camera_list() const { return (lane_mark_camera_list_); }
  //ad_msg::LaneMarkCameraList& lane_mark_camera_list() { return (lane_mark_camera_list_); }

private:
  const ad_msg::LaneMarkCameraList* lane_mark_camera_list_=Nullptr_t;
};

class MessageObstacleCameraList : public Message {
public:
  MessageObstacleCameraList(Int32_t msg_id, const ad_msg::ObstacleCameraList *msg) :
    Message(msg_id),
    obstacle_camera_list_(msg) {
  }

  const ad_msg::ObstacleCameraList* obstacle_camera_list() const { return (obstacle_camera_list_); }
  //ad_msg::ObstacleCameraList& obstacle_camera_list() { return (obstacle_camera_list_); }

private:
  const ad_msg::ObstacleCameraList* obstacle_camera_list_ =Nullptr_t;
};

class MessageRecvRadarData : public Message {
public:
  MessageRecvRadarData(Int32_t msg_id, const ad_msg::ObstacleRadarList* obj_list) :
    Message(msg_id),
    obstacle_list_(obj_list) {
  }
  const ad_msg::ObstacleRadarList* obstacle_list() const { return (obstacle_list_); }

private:
  const ad_msg::ObstacleRadarList* obstacle_list_ = Nullptr_t;
};

class MessageRecvLidarObjects : public Message {
public:
  MessageRecvLidarObjects(Int32_t msg_id, const ad_msg::ObstacleLidarList* obj_list) :
    Message(msg_id),
    obstacle_list_(obj_list) {
  }
  const ad_msg::ObstacleLidarList* obstacle_list() const { return (obstacle_list_); }

private:
  const ad_msg::ObstacleLidarList* obstacle_list_ = Nullptr_t;
};

class MessageRecvLidarCloud : public Message {
public:
  MessageRecvLidarCloud(Int32_t msg_id, const ad_msg::LidarCloud* cloud) :
    Message(msg_id),
    lidar_cloud_(cloud) {
  }
  const ad_msg::LidarCloud* lidar_cloud() const { return (lidar_cloud_); }

private:
  const ad_msg::LidarCloud* lidar_cloud_ = Nullptr_t;
};

class MessageRecvImuData : public Message{
public:
  MessageRecvImuData(const ad_msg::Imu* imu):
    Message(MSG_ID_RECV_IMU_DATA),
    imu_(imu){
  }

  const ad_msg::Imu *imu() const { return (imu_); }
private:
  const ad_msg::Imu *imu_ = Nullptr_t;
};

class MessageRecvGnssData : public Message{
public:
  MessageRecvGnssData(const ad_msg::Gnss* gnss):
    Message(MSG_ID_RECV_GNSS_DATA),
    gnss_(gnss){

  }
  const ad_msg::Gnss *gnss() const { return (gnss_); }
private:
  const ad_msg::Gnss *gnss_ = Nullptr_t;
};

class MessageTrafficSignalList : public Message{
public:
  MessageTrafficSignalList(const ad_msg::TrafficSignalList* data):
    Message(MSG_ID_TRAFFIC_SIGNAL_LIST),
    traffic_signal_list_(data){
  }
  const ad_msg::TrafficSignalList* traffic_signal_list() const{
    return (traffic_signal_list_);
  }
private:
  const ad_msg::TrafficSignalList* traffic_signal_list_ = Nullptr_t;
};

class MessageChassis : public Message {
public:
  MessageChassis(const ad_msg::Chassis* data) :
    Message(MSG_ID_CHASSIS),
    chassis_(data) {
  }
  inline const ad_msg::Chassis* chassis() const { return (chassis_); }

private:
  const ad_msg::Chassis* chassis_ = Nullptr_t;
};

class MessageRecvFusionObstacleList : public Message {
public:
  MessageRecvFusionObstacleList(const ad_msg::ObstacleList* data) :
    Message(MSG_ID_RECV_SENSORS_FUSION_DATA),
    fusion_obstacle_list_(data) {
  }
  inline const ad_msg::ObstacleList* fusion_obstacle_list() const { return (fusion_obstacle_list_); }

private:
  const ad_msg::ObstacleList* fusion_obstacle_list_ = Nullptr_t;
};


class MessageMpuStateData : public Message {
public:
  MessageMpuStateData(Int32_t msg_id, const MpuState::MonitorMpuState* mpu_state) :
    Message(MSG_ID_RECV_MPU_STATE_DATA),
    mpu_state_(mpu_state) {
  }
  const MpuState::MonitorMpuState* mpu_state() const { return (mpu_state_); }

private:
  const MpuState::MonitorMpuState* mpu_state_ = Nullptr_t;
};

class MessageRecvADHMIFusionObstacleList : public Message {
public:
  MessageRecvADHMIFusionObstacleList(const ad_msg::ObstacleList* data) :
    Message(MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA),
    adhmi_fusion_obstacle_list_(data) {
  }
  inline const ad_msg::ObstacleList* adhmi_fusion_obstacle_list() const { return (adhmi_fusion_obstacle_list_); }

private:
  const ad_msg::ObstacleList* adhmi_fusion_obstacle_list_ = Nullptr_t;
};

class MessageRecvADHMIClock : public Message {
public:
  MessageRecvADHMIClock(const rosgraph_msgs::Clock* data) :
    Message(MSG_ID_RECV_ADHMI_CLOCK),
    adhmi_clock_(data) {
  }
  inline const rosgraph_msgs::Clock* adhmi_clock() const { return (adhmi_clock_); }

private:
  const rosgraph_msgs::Clock* adhmi_clock_ = Nullptr_t;
};
}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_MESSAGE_H_
