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
#include "pos_filter.h"
#include "obj_filter.h"
#include "driving_map.h"
#include "motion_planning.h"

#include "planning/plan_debug.h"
#include "planning/adecu_debug.h"
#include "msg_planning.h"

#include "can_dev/can_driver.h"
#include "pcc_map/adasis_v2.h"

namespace phoenix {
namespace framework {


/**
 * @enum MessageId
 * @brief 消息 ID
 */
enum MessageId {
  /// 无效值
  MSG_ID_INVALID = 0,
  /// 请求执行任务
  MSG_ID_REQ_DO_TASK,
  /// 请求执行Chatter类型任务
  MSG_ID_REQ_DO_CHATTER_TASK,
  /// 高精度地图数据
  MSG_ID_MAP_DATA,
  /// 一次规划结果
  MSG_ID_ROUTING_DATA,
  /// GNSS测量单元信息
  MSG_ID_GNSS,
  /// 惯性测量单元信息
  MSG_ID_IMU,
  /// 车身信息
  MSG_ID_CHASSIS,
  /// 从车身获取的其它信息
  MSG_ID_SPECIAL_CHASSIS_INFO,
  /// 车身控制命令信息
  MSG_ID_CHASSIS_CTL_CMD,
  /// 相机识别的车道线信息
  MSG_ID_LANE_MARK_CAMERA_LIST,
  /// 相机识别的障碍物信息
  MSG_ID_OBSTACLE_CAMERA_LIST,
  /// 毫米波雷达识别的障碍物信息
  MSG_ID_OBSTACLE_RADAR_FRONT_LIST,
  MSG_ID_OBSTACLE_RADAR_REAR_LIST,
  MSG_ID_OBSTACLE_RADAR_FRONT_LEFT_LIST,
  MSG_ID_OBSTACLE_RADAR_FRONT_RIGHT_LIST,
  MSG_ID_OBSTACLE_RADAR_REAR_LEFT_LIST,
  MSG_ID_OBSTACLE_RADAR_REAR_RIGHT_LIST,
  /// Lidar识别的障碍物信息
  MSG_ID_OBSTACLE_LIDAR_LIST_0,
  MSG_ID_OBSTACLE_LIDAR_LIST_1,
  /// SRR2 Detections
  MSG_ID_SRR2_DETECTIONS_FRONT_LEFT_LIST,
  MSG_ID_SRR2_DETECTIONS_FRONT_RIGHT_LIST,
  MSG_ID_SRR2_DETECTIONS_REAR_LEFT_LIST,
  MSG_ID_SRR2_DETECTIONS_REAR_RIGHT_LIST,
  /// 障碍物信息 from fusion module
  MSG_ID_OBSTACLE_LIST,
  /// 运动规划的结果
  MSG_ID_PLANNING_RESULT,

  /// Add Planning debug BY ZQ for Ros
  MSG_ID_PLANNING_DEBUG,
  
  /// 人机交互模块发送的设置信息
  MSG_ID_REMOTE_PLANNING_SETTINGS,
  /// 交通标志
  MSG_ID_TRAFFIC_SIGNAL_LIST,
  /// 红绿灯
  MSG_ID_TRAFFIC_LIGHT_LIST,
  /// 场景任务
  MSG_ID_SCENE_STORY_LIST,
  /// 地图定位
  MSG_ID_MAP_LOCALIZATION,
  /// adecu vel plan proto
  MSG_ID_ADECU_VEL_PLANNING_DEBUG,
  /// adecu plan debug msg ros 
  MSG_ID_ADECU_PLANNING_DEBUG,

  // ADASIS v2 RAW CAN Frame
  MSG_ID_MAP_ADASISV2_CAN_FRAME,

  // ADASIS v2 POSITION message
  MSG_ID_MAP_RECV_ADASISV2_POSITION,

  // ADASIS v2 PROFILE SHORT message
  MSG_ID_MAP_RECV_ADASISV2_PROFILE_SHORT,

  // ADASIS v2 HORIZON message
  MSG_ID_MAP_ADASISV2_HORIZON
};

/**
 * @enum MessageId
 * @brief 任务 ID
 */
enum TaskId {
  /// 无效的任务ID
  TASK_ID_INVALID = 0,
  /// 任务管理
  TASK_ID_MANAGER,
  /// Chatter类型任务
  TASK_ID_CHATTER,
  /// 外部消息接收
  TASK_ID_MSG_RECEIVER,
  /// 向外部发送消息
  TASK_ID_MSG_SENDER,
  /// 向外部发送HMI相关的消息
  TASK_ID_HMI_MSG_SENDER,
  /// 规划
  TASK_ID_PLANNING,
  /// ADASIS v2 map 数据接收任务
  TASK_ID_MAP_ADASISV2
};

struct PlanningConfig {
  pos_filter::PosFilterConfig pos_filter_config;
  obj_filter::ObjFilterConfig obj_filter_config;
  driv_map::DrivingMapConfig driving_map_config;
  planning::ActionPlanningConfig act_planning_config;
  planning::TrajectoryPlanningConfig trj_planning_config;
  planning::VelocityPlanningConfig vel_planning_config;

  PlanningConfig() {
    // nothing to do
  }
};

class Message {
 public:
  explicit Message(Int32_t id) : id_(id) {}
  virtual ~Message() = default;

  Message(const Message& other) {
    id_ = other.id_;
  }

  void operator =(const Message& other) {
    id_ = other.id_;
  }

  Int32_t id() const { return (id_); }

 private:
  Int32_t id_;
};

enum {
  CHATTER_WORK_INVALID = 0,
  CHATTER_WORK_REQ_PARSE_MAP_DATA,
  CHATTER_WORK_REQ_PARSE_ROUTING_DATA
};

class MessageChatterTask : public Message {
 public:
  MessageChatterTask() :
    Message(MSG_ID_REQ_DO_CHATTER_TASK),
    work_id_(CHATTER_WORK_INVALID) {
  }

  explicit MessageChatterTask(Int32_t work_id) :
    Message(MSG_ID_REQ_DO_CHATTER_TASK),
    work_id_(work_id) {
  }

  Int32_t work_id() const { return (work_id_); }

 private:
  Int32_t work_id_;
};

class MessageMapData : public Message {
 public:
  MessageMapData(const Char_t* data_buff, Int32_t buff_size) :
    Message(MSG_ID_MAP_DATA),
    data_buff_(data_buff),
    buff_size_(buff_size) {
  }
  inline const Char_t* data_buff() const { return (data_buff_); }
  inline Int32_t buff_size() const { return (buff_size_); }

 private:
  const Char_t*data_buff_ = Nullptr_t;
  Int32_t buff_size_ = 0;
};

class MessageRoutingData : public Message {
 public:
  MessageRoutingData(const Char_t* data_buff, Int32_t buff_size) :
    Message(MSG_ID_ROUTING_DATA),
    data_buff_(data_buff),
    buff_size_(buff_size) {
  }
  inline const Char_t* data_buff() const { return (data_buff_); }
  inline Int32_t buff_size() const { return (buff_size_); }

 private:
  const Char_t* data_buff_ = Nullptr_t;
  Int32_t buff_size_ = 0;
};

class MessageGnss : public Message {
public:
  MessageGnss(const ad_msg::Gnss* data) :
    Message(MSG_ID_GNSS),
    gnss_(data) {
  }
  inline const ad_msg::Gnss* gnss() const { return (gnss_); }

private:
  const ad_msg::Gnss* gnss_ = Nullptr_t;
};

class MessageImu : public Message {
public:
  MessageImu(const ad_msg::Imu* data) :
    Message(MSG_ID_IMU),
    imu_(data) {
  }
  inline const ad_msg::Imu* imu() const { return (imu_); }

private:
  const ad_msg::Imu* imu_ = Nullptr_t;
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

class MessageSpecialChassisInfo : public Message {
public:
  MessageSpecialChassisInfo(const ad_msg::SpecialChassisInfo* data) :
    Message(MSG_ID_SPECIAL_CHASSIS_INFO),
    special_chassis_info_(data) {
  }
  inline const ad_msg::SpecialChassisInfo* special_chassis_info() const {
    return (special_chassis_info_);
  }

private:
  const ad_msg::SpecialChassisInfo* special_chassis_info_ = Nullptr_t;
};

class MessageChassisCtlCmd : public Message {
public:
  MessageChassisCtlCmd(const ad_msg::ChassisCtlCmd* data) :
    Message(MSG_ID_CHASSIS_CTL_CMD),
    chassis_ctl_cmd_(data) {
  }
  inline const ad_msg::ChassisCtlCmd* chassis_ctl_cmd() const { return (chassis_ctl_cmd_); }

private:
  const ad_msg::ChassisCtlCmd* chassis_ctl_cmd_ = Nullptr_t;
};

class MessageLaneMarkCameraList : public Message {
public:
  MessageLaneMarkCameraList(const ad_msg::LaneMarkCameraList* data) :
    Message(MSG_ID_LANE_MARK_CAMERA_LIST),
    lane_mark_camera_list_(data) {
  }
  const ad_msg::LaneMarkCameraList* lane_mark_camera_list() const {
    return (lane_mark_camera_list_);
  }

private:
  const ad_msg::LaneMarkCameraList* lane_mark_camera_list_ = Nullptr_t;
};

class MessageObstacleCameraList : public Message {
public:
  MessageObstacleCameraList(const ad_msg::ObstacleCameraList* data) :
    Message(MSG_ID_OBSTACLE_CAMERA_LIST),
    obstacle_camera_list_(data) {
  }
  const ad_msg::ObstacleCameraList* obstacle_camera_list() const {
    return (obstacle_camera_list_);
  }

private:
  const ad_msg::ObstacleCameraList* obstacle_camera_list_ = Nullptr_t;
};

class MessageObstacleRadarList : public Message {
public:
  MessageObstacleRadarList(
      Int32_t msg_id, const ad_msg::ObstacleRadarList* data) :
    Message(msg_id),
    obstacle_list_(data) {
  }
  const ad_msg::ObstacleRadarList* obstacle_list() const {
    return (obstacle_list_);
  }

private:
  const ad_msg::ObstacleRadarList* obstacle_list_ = Nullptr_t;
};

class MessageObstacleLidarList : public Message {
public:
  MessageObstacleLidarList(
      Int32_t msg_id, const ad_msg::ObstacleLidarList* data) :
    Message(msg_id),
    obstacle_list_(data) {
  }
  const ad_msg::ObstacleLidarList* obstacle_list() const {
    return (obstacle_list_);
  }

private:
  const ad_msg::ObstacleLidarList* obstacle_list_ = Nullptr_t;
};

class MessageObstacleList : public Message {
public:
  MessageObstacleList(const ad_msg::ObstacleList* data) :
    Message(MSG_ID_OBSTACLE_LIST),
    obstacle_list_(data) {
  }
  const ad_msg::ObstacleList* obstacle_list() const {
    return (obstacle_list_);
  }

private:
  const ad_msg::ObstacleList* obstacle_list_ = Nullptr_t;
};

class MessageAdecuVelPlanningDebug : public Message {
public:
  MessageAdecuVelPlanningDebug(const planning::VelocityPlanningResult* data) :
    Message(MSG_ID_ADECU_VEL_PLANNING_DEBUG),
    adecu_vel_planning_debug_(data) {
  }
  const planning::VelocityPlanningResult* adecu_planning_debug() const {
    return (adecu_vel_planning_debug_);
  }

private:
  const planning::VelocityPlanningResult* adecu_vel_planning_debug_ = Nullptr_t;
};

class MessagePlanningResult : public Message {
 public:
  explicit MessagePlanningResult(
      const ad_msg::PlanningResult* pl_ret, Int32_t st, Int32_t t_ela) :
    Message(MSG_ID_PLANNING_RESULT) {
    planning_result_ = pl_ret;
    status_ = st;
    time_elapsed_ = t_ela;
  }

  const ad_msg::PlanningResult* planning_result() const {
    return planning_result_;
  }

  Int32_t status() const { return (status_); }
  Int32_t time_elapsed() const { return (time_elapsed_); }

 private:
  Int32_t status_;
  Int32_t time_elapsed_;
  const ad_msg::PlanningResult* planning_result_;
};


// Add Planning debug BY ZQ for Ros
class MessagePlanningDebug : public Message
{
public:
  explicit MessagePlanningDebug(
      const ::planning::plan_debug* pl_ret, Int32_t st, Int32_t t_ela) :
      Message(MSG_ID_PLANNING_DEBUG) {
      planning_debug_ = pl_ret;
    status_ = st;
    time_elapsed_ = t_ela;
  }

  const ::planning::plan_debug* plan_debug() const {
    return planning_debug_;
  }

  Int32_t status() const { return (status_); }
  Int32_t time_elapsed() const { return (time_elapsed_); }

 private:
  Int32_t status_;
  Int32_t time_elapsed_;
  const ::planning::plan_debug* planning_debug_;
};

// adecu plan debug msg ros
class MessageAdecuDebug : public Message
{
public:
  explicit MessageAdecuDebug(
      const ::planning::adecu_debug* pl_ret, Int32_t st, Int32_t t_ela) :
      Message(MSG_ID_ADECU_PLANNING_DEBUG) {
      adecu_debug_ = pl_ret;
    status_ = st;
    time_elapsed_ = t_ela;
  }

  const ::planning::adecu_debug* adecu_debug() const {
    return adecu_debug_;
  }

  Int32_t status() const { return (status_); }
  Int32_t time_elapsed() const { return (time_elapsed_); }

 private:
  Int32_t status_;
  Int32_t time_elapsed_;
  const ::planning::adecu_debug* adecu_debug_;
};



class MessageRemotePlanningSettings : public Message {
public:
  MessageRemotePlanningSettings(const ad_msg::PlanningSettings* data) :
    Message(MSG_ID_REMOTE_PLANNING_SETTINGS),
    planning_settings_(data) {
  }
  const ad_msg::PlanningSettings* planning_settings() const{
    return (planning_settings_);
  }

private:
  const ad_msg::PlanningSettings *planning_settings_ = Nullptr_t;
};

class MessageTrafficSignalList: public Message {
public:
  MessageTrafficSignalList(const ad_msg::TrafficSignalList* data) :
  Message(MSG_ID_TRAFFIC_SIGNAL_LIST),
  traffic_signal_list_(data)
 {
  }
  const ad_msg::TrafficSignalList* traffic_signal_list() const{
    return (traffic_signal_list_);
  }

private:
  const ad_msg::TrafficSignalList *traffic_signal_list_ = Nullptr_t;
};

class MessageTrafficLightList: public Message {
public:
  MessageTrafficLightList(const ad_msg::TrafficLightList* data) :
  Message(MSG_ID_TRAFFIC_LIGHT_LIST),
  traffic_light_list_(data)
 {
  }
  const ad_msg::TrafficLightList* traffic_light_list() const{
    return (traffic_light_list_);
  }

private:
  const ad_msg::TrafficLightList *traffic_light_list_ = Nullptr_t;
};

class MessageSceneStoryList: public Message {
public:
  MessageSceneStoryList(const ad_msg::SceneStoryList* data) :
    Message(MSG_ID_SCENE_STORY_LIST), scene_story_list_(data) {
  }

  const ad_msg::SceneStoryList* scene_story_list() const{
    return (scene_story_list_);
  }

private:
  const ad_msg::SceneStoryList* scene_story_list_ = Nullptr_t;
};

class MessageMapLocalization: public Message {
public:
  MessageMapLocalization(const ad_msg::MapLocalization* data) :
    Message(MSG_ID_MAP_LOCALIZATION), map_localization_(data) {
  }
  const ad_msg::MapLocalization* map_localization() const{
    return (map_localization_);
  }

private:
  const ad_msg::MapLocalization *map_localization_ = Nullptr_t;
};

class MessageCanFrame : public Message {
public:
  MessageCanFrame(const phoenix::can_dev::CanFrame *msg) :
    Message(MSG_ID_MAP_ADASISV2_CAN_FRAME),
    can_frame_(msg) {
  }

  const phoenix::can_dev::CanFrame* can_frame() const { return (can_frame_); }

private:
  const phoenix::can_dev::CanFrame* can_frame_ = Nullptr_t;
};

class MessageRecvADASISV2Position: public Message {
public:
  MessageRecvADASISV2Position(const adasisv2::PositionMessage* data) :
    Message(MSG_ID_MAP_RECV_ADASISV2_POSITION), position_(data) {
  }
  const adasisv2::PositionMessage* position() const{
    return (position_);
  }

private:
  const adasisv2::PositionMessage *position_ = Nullptr_t;
};


class MessageRecvADASISV2ProfileShort: public Message {
public:
  MessageRecvADASISV2ProfileShort(const adasisv2::ProfileShortMessage* data) :
    Message(MSG_ID_MAP_RECV_ADASISV2_PROFILE_SHORT), profile_(data) {
  }
  const adasisv2::ProfileShortMessage* profile() const{
    return (profile_);
  }

private:
  const adasisv2::ProfileShortMessage *profile_ = Nullptr_t;
};

class MessageADASISV2Horion: public Message {
public:
  MessageADASISV2Horion(const adasisv2::Horizon* data) :
    Message(MSG_ID_MAP_ADASISV2_HORIZON), horizon_(data) {
  }
  const adasisv2::Horizon* horizon() const{
    return (horizon_);
  }

private:
  const adasisv2::Horizon *horizon_ = Nullptr_t;
};

}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_MESSAGE_H_
