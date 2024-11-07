//
#include "serial_msg_chassis.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeChassisArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::Chassis* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"driving_mode\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].e_stop), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"e_stop\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].eps_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"eps_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].throttle_sys_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"throttle_sys_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ebs_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ebs_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_angle_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_angle_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_angle\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_speed_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_speed\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_torque_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_torque\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ax_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ax_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ax), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ax\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ay_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ay_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ay), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ay\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fl_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_fl_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_fl\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fr_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_fr_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fr), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_fr\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rl_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rl\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rr_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rr\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl2_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rl2_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rl2\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr2_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rr2_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rr2\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"epb_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear_number), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear_number\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].signal_turning_indicator), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"signal_turning_indicator\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].signal_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"signal_turn_lamp\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].signal_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"signal_brake_lamp\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_pedal_value\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acc_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"acc_pedal_value\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"engine_speed_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"engine_speed\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"engine_torque_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_torque), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"engine_torque\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeChassisArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::Chassis* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"driving_mode\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].e_stop), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"e_stop\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].eps_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"eps_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].throttle_sys_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"throttle_sys_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ebs_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ebs_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_angle_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_angle_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_angle\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_speed_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"steering_wheel_angle\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"steering_wheel_torque_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"steering_wheel_torque\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"a_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"a\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ax_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"ax_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ax), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"ax\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ay_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"ay_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].ay), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"ay\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fl_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_fl_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_fl\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fr_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_fr_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_fr), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_fr\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rl_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wheel_speed_rl\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rr_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rr\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl2_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rl2_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rl2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rl2\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr2_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rr2_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wheel_speed_rr2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wheel_speed_rr2\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"epb_status\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"gear\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear_number), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"gear_number\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].signal_turning_indicator), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"signal_turning_indicator\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].signal_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"signal_turn_lamp\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].signal_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"signal_brake_lamp\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"brake_pedal_value\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acc_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"acc_pedal_value\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"engine_speed_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"engine_speed\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"engine_torque_valid\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].engine_torque), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"engine_torque\" of Chassis["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeChassisCtlCmdArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ChassisCtlCmd* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_robotic_ctl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_robotic_ctl\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_eps\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_throttle_sys\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_ebs\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_remote_ctl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_remote_ctl\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_direct_ctl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_direct_ctl\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_acc\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"release_throttle\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_angle\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_speed\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"steering_wheel_torque\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].velocity), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"velocity\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acceleration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"acceleration\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acc_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"acc_value\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_value\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"turn_lamp\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_lamp\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wiper), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"wiper\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"epb_status\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeChassisCtlCmdArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ChassisCtlCmd* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {

    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_robotic_ctl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"start_robotic_ctl\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_eps\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_throttle_sys\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_ebs\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_remote_ctl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_remote_ctl\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_direct_ctl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_direct_ctl\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_acc\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"release_throttle\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"steering_wheel_angle\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"steering_wheel_speed\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"steering_wheel_torque\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].velocity), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"velocity\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acceleration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"acceleration\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acc_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"acc_value\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"brake_value\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"gear\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"turn_lamp\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"brake_lamp\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].wiper), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"wiper\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"epb_status\" of ChassisCtlCmd["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeSpecialChassisInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SpecialChassisInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of SpecialChassisInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_adas), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_adas\" of SpecialChassisInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_frame_loss_can0\" "
                 "of SpecialChassisInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_frame_loss_can1\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_frame_loss_can2\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_frame_loss_can3\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_gtw_to_veh_can0\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_gtw_to_veh_can1\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_gtw_to_veh_can2\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_gtw_to_veh_can3\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_ctl_to_gtw_can0\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_ctl_to_gtw_can1\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_ctl_to_gtw_can2\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_ctl_to_gtw_can3\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cnt_stu_ctl_to_gtw\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeSpecialChassisInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SpecialChassisInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of SpecialChassisInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_adas), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"start_adas\" of SpecialChassisInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_frame_loss_can0\" "
                 "of SpecialChassisInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_frame_loss_can1\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_frame_loss_can2\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_frame_loss_can3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_frame_loss_can3\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_gtw_to_veh_can0\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_gtw_to_veh_can1\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_gtw_to_veh_can2\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_gtw_to_veh_can3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_gtw_to_veh_can3\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_ctl_to_gtw_can0\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_ctl_to_gtw_can1\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_ctl_to_gtw_can2\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw_can3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_ctl_to_gtw_can3\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cnt_stu_ctl_to_gtw), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cnt_stu_ctl_to_gtw\" "
                 "of SpecialChassisInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


