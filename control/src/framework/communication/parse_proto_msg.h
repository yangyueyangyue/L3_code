//
#ifndef PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_
#define PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_

#include "ad_msg_c.h"

#include "gnss.pb.h"
#include "imu.pb.h"
#include "chassis.pb.h"
#include "special_chassis_info.pb.h"
#include "chassis_ctl_cmd.pb.h"
#include "planning_result.pb.h"
#include "chassis_dfcv.pb.h"


namespace phoenix {
namespace framework {


class ParseProtoMsg {
public:
  bool EncodeGnssMessage(
      const Gnss_t& msg,
      msg::localization::Gnss* const data_out);
  bool DecodeGnssMessage(
      const Char_t* msg, Int32_t msg_len,
      Gnss_t* data_out);

  bool EncodeImuMessage(
      const Imu_t& msg,
      msg::localization::Imu* const data_out);
  bool DecodeImuMessage(
      const Char_t* msg, Int32_t msg_len,
      Imu_t* data_out);

  bool EncodeChassisMessage(
      const Chassis_t& msg,
      msg::control::Chassis* const data_out);
  bool DecodeChassisMessage(
      const Char_t* msg, Int32_t msg_len,
      Chassis_t* data_out);

  bool EncodeSpecialChassisInfoMessage(
      const SpecialChassisInfo_t& msg,
      msg::control::SpecialChassisInfo* const data_out);
  bool DecodeSpecialChassisInfoMessage(
      const Char_t* msg, Int32_t msg_len,
      SpecialChassisInfo_t* data_out);

  bool DecodeDFCVSpecialChassisInfoMessage(
      const Char_t* msg, Int32_t msg_len,
      DFCV_SpecialChassisInfo_t* data_out);

  bool EncodeChassisCtlCmdMessage(
      const ChassisCtlCmd_t& msg,
      msg::control::ChassisCtlCmd* const data_out);
  bool DecodeChassisCtlCmdMessage(
      const Char_t* msg, Int32_t msg_len,
      ChassisCtlCmd_t* data_out);

  bool EncodePlanningResultMessage(
      const PlanningResult_t& msg,
      msg::planning::PlanningResult* const data_out);
  bool DecodePlanningResultMessage(
      const Char_t* msg, Int32_t msg_len,
      PlanningResult_t* data_out);
};


}  // namespace framework
}  // namespace phoenix


#endif // PHOENIX_FRAMEWORK_PARSE_PROTO_MSG_H_
