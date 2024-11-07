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

#include "sensors_fusion/multi_constructure_classes_repo.h"

// #include "sensing_fusion/chassis_info.h"
// #include "sensing_fusion/lane_input.h"
// #include "sensing_fusion/laneline_info.h"
// #include "sensing_fusion/obbox_info.h"
// #include "sensing_fusion/obstacle_camera.h"
// #include "sensing_fusion/perception_debug.h"
#include "sensing_fusion/perception_debug.h"
#include "MpuState.pb.h"

namespace phoenix {
namespace framework {

enum MessageId {
  MSG_ID_INVALID = 0,
  MSG_ID_REQ_DO_TASK,
  MSG_ID_LANE_MARK_CAMERA_LIST,
  MSG_ID_LANE_CURB_CAMERA_LIST,
  MSG_ID_TRAFFIC_SIGNAL_LIST,
  MSG_ID_RECV_ESR_DATA_FRONT,
  MSG_ID_RECV_IMU_DATA,
  MSG_ID_RECV_GNSS_DATA,
  MSG_ID_CHASSIS,

  MSG_ID_PLAYBACK_IBEO_OBJECTS_FRONT,
  MSG_ID_PLAYBACK_IBEO_CLOUD_FRONT,

  MSG_ID_PLAYBACK_LIDAR_OBJECTS_FRONT,
  MSG_ID_PLAYBACK_LIDAR_CLOUD_FRONT,

  MSG_ID_RECV_ROBOSENSE_M1_OBJECTS_FRONT,
  MSG_ID_RECV_ROBOSENSE_M1_CLOUD_FRONT,

  MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT,
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
  
  MSG_ID_RECV_SENSORS_FUSION_DATA,

  MSG_ID_RECV_MPU_STATE_DATA,

  MSG_ID_PERCEPTION_DEBUG=70


};

enum TaskId {
  TASK_ID_INVALID = 0,
  TASK_ID_MANAGER,
  TASK_ID_MSG_RECEIVER,
  TASK_ID_MSG_SENDER,
  TASK_ID_RECV_IMU_DATA,
  TASK_ID_RECV_GNSS_DATA,

  TASK_ID_RECV_SENSORS_FUSION_DATA
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

// Add Perception debug BY wzf for Ros
class MessagePerceptionDebug : public Message
{
public:
  explicit MessagePerceptionDebug(
      const ::sensing_fusion::perception_debug* per_ret, Int32_t st, Int32_t t_ela) :
      Message(MSG_ID_PERCEPTION_DEBUG) {
    perception_debug_ = per_ret;
    status_ = st;
    time_elapsed_ = t_ela;
  }

  const ::sensing_fusion::perception_debug* perception_debug() const {
    return perception_debug_;
  }

  Int32_t status() const { return (status_); }
  Int32_t time_elapsed() const { return (time_elapsed_); }

private:
  Int32_t status_;
  Int32_t time_elapsed_;
  const ::sensing_fusion::perception_debug* perception_debug_;
};


#if 0
class MessageSensorsFusionData : public Message {
public:
    MessageSensorsFusionData(const ad_msg::ObstacleList* data, const Int32_t st, const Int32_t t_ela ) :
     Message(MSG_ID_RECV_SENSORS_FUSION_DATA),
     objs_sensors_fusion_(data),
      status_(st), time_elapsed_(t_ela)
    {
    }

  inline const ad_msg::ObstacleList* GetObjsSensorsFusionResult() const
  {
      return objs_sensors_fusion_;
  }

  Int32_t status() const { return status_;}

  Int32_t time_elapsed() const { return time_elapsed_;}
private:
  const ad_msg::ObstacleList* objs_sensors_fusion_ = Nullptr_t;
  Int32_t status_;
  Int32_t time_elapsed_;
};
#endif

}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_MESSAGE_H_
