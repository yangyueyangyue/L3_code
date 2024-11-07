//
#ifndef PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_
#define PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_

#include "ad_msg.h"

#include "gnss.pb.h"
#include "imu.pb.h"
#include "chassis.pb.h"
#include "special_chassis_info.pb.h"
#include "chassis_ctl_cmd.pb.h"
#include "lane_mark_camera.pb.h"
#include "obstacles_camera.pb.h"
#include "obstacles_radar.pb.h"
#include "obstacles_lidar.pb.h"
#include "lidar_cloud.pb.h"
#include "obstacles.pb.h"
#include "traffic_signal.pb.h"
#include "traffic_light.pb.h"
#include "planning_result.pb.h"
#include "scene_story.pb.h"
#include "map_localization.pb.h"
#include "obstacles.pb.h"
#include "MpuState.pb.h"


namespace phoenix {
namespace framework {


class ParseProtoMsg {
public:
  bool EncodeGnssMessage(
      const ad_msg::Gnss& msg,
      msg::localization::Gnss* const data_out);
  bool DecodeGnssMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::Gnss* data_out);

  bool EncodeImuMessage(
      const ad_msg::Imu& msg,
      msg::localization::Imu* const data_out);
  bool DecodeImuMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::Imu* data_out);

  bool EncodeMpuStateMessage(
      const MpuState::MonitorMpuState& msg,
      MpuState::MonitorMpuState* const data_out);
  bool DecodeMpuStateMessage(
      const Char_t* msg, Int32_t msg_len,
      MpuState::MonitorMpuState* data_out);

  bool EncodeChassisMessage(
      const ad_msg::Chassis& msg,
      msg::control::Chassis* const data_out);
  bool DecodeChassisMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::Chassis* data_out);

  bool EncodeSpecialChassisInfoMessage(
      const ad_msg::SpecialChassisInfo& msg,
      msg::control::SpecialChassisInfo* const data_out);
  bool DecodeSpecialChassisInfoMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::SpecialChassisInfo* data_out);

  bool EncodeChassisCtlCmdMessage(
      const ad_msg::ChassisCtlCmd& msg,
      msg::control::ChassisCtlCmd* const data_out);
  bool DecodeChassisCtlCmdMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::ChassisCtlCmd* data_out);

  bool EncodeLaneMarkCameraListMessage(
      const ad_msg::LaneMarkCameraList& msg,
      msg::perception::LaneMarkCameraList* const data_out);
  bool DecodeLaneMarkCameraListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::LaneMarkCameraList* data_out);

  bool EncodeObstacleCameraListMessage(
      const ad_msg::ObstacleCameraList& msg,
      msg::perception::ObstacleCameraList* const data_out);
  bool DecodeObstacleCameraListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::ObstacleCameraList* data_out);

  bool EncodeObstacleRadarListMessage(
      const ad_msg::ObstacleRadarList& msg,
      msg::perception::ObstacleRadarList* const data_out);
  bool DecodeObstacleRadarListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::ObstacleRadarList* data_out);

  bool EncodeObstacleLidarListMessage(
      const ad_msg::ObstacleLidarList& msg,
      msg::perception::ObstacleLidarList* const data_out);
  bool DecodeObstacleLidarListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::ObstacleLidarList* data_out);

  bool EncodeLidarCloudMessage(
      const ad_msg::LidarCloud& msg,
      msg::perception::LidarCloud* const data_out);
  bool DecodeLidarCloudMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::LidarCloud* data_out);

  bool EncodeObstacleListMessage(
      const ad_msg::ObstacleList& msg,
      msg::perception::ObstacleList* const data_out);
  bool DecodeObstacleListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::ObstacleList* data_out);

  bool EncodeTrafficSignalBoxMessage(
      const ad_msg::TrafficSignalBox& msg,
      msg::perception::TrafficSignalBox* const data_out);
  bool DecodeTrafficSignalBoxMessage(
      const msg::perception::TrafficSignalBox& msg,
      ad_msg::TrafficSignalBox* data_out);

  bool EncodeTrafficSignalSpeedRestrictionMessage(
      const ad_msg::TrafficSignalSpeedRestriction& msg,
      msg::perception::TrafficSignalSpeedRestriction* const data_out);
  bool DecodeTrafficSignalSpeedRestrictionMessage(
      const msg::perception::TrafficSignalSpeedRestriction& msg,
      ad_msg::TrafficSignalSpeedRestriction* data_out);

  bool EncodeTrafficSignalListMessage(
      const ad_msg::TrafficSignalList& msg,
      msg::perception::TrafficSignalList* const data_out);
  bool DecodeTrafficSignalListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::TrafficSignalList* data_out);

  bool EncodeTrafficLightListMessage(
      const ad_msg::TrafficLightList& msg,
      msg::perception::TrafficLightList* const data_out);
  bool DecodeTrafficLightListMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::TrafficLightList* data_out);

  bool EncodePlanningResultMessage(
      const ad_msg::PlanningResult& msg,
      msg::planning::PlanningResult* const data_out);
  bool DecodePlanningResultMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::PlanningResult* data_out);

  bool DecodeSceneStoryControlLine(
      const msg::routing::ControlLine& line_in,
      ad_msg::SceneStoryControlLine* line_out);
  bool DecodeSceneStoryCondition(
      const msg::routing::Condition& cond_in,
      ad_msg::SceneStoryCondition* cond_out);
  bool DecodeSceneStoryAction(
      const msg::routing::Action& action_in,
      ad_msg::SceneStoryAction* action_out);
  bool DecodeSceneStorysMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::SceneStoryList* data_out);

  bool DecodeMapLocalizationMessage(
      const Char_t* msg, Int32_t msg_len,
      ad_msg::MapLocalization* data_out);
};


}  // namespace framework
}  // namespace phoenix


#endif // PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_
