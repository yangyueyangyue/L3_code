//
#include "serial_msg_chassis_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/serialization_utils_c.h"
#include "serial_msg_common_c.h"


Int32_t Phoenix_Common_EncodeChassisArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Chassis_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"msg_head\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"driving_mode\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].e_stop), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"e_stop\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].eps_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"eps_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].throttle_sys_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"throttle_sys_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].ebs_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"ebs_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_angle_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_angle_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_angle\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_speed_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_speed\" of Chassis].");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_torque_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_torque\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].v_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"v_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"v\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].a_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"a_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"a\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].yaw_rate_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"yaw_rate_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"yaw_rate\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].ax_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"ax_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].ax), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"ax\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].ay_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"ay_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].ay), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"ay\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fl_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_fl_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_fl\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fr_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_fr_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fr), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_fr\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rl_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rl\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rr_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rr\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl2_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rl2_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rl2\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr2_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rr2_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wheel_speed_rr2\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"epb_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"gear\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].gear_number), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"gear_number\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].signal_turning_indicator), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"signal_turning_indicator\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].signal_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"signal_turn_lamp\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].signal_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"signal_brake_lamp\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].brake_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"brake_pedal_value\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].acc_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"acc_pedal_value\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"engine_speed_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"engine_speed\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"engine_torque_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_torque), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"engine_torque\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeChassisArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Chassis_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"msg_head\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"driving_mode\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].e_stop), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"e_stop\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].eps_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"eps_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].throttle_sys_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"throttle_sys_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].ebs_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"ebs_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_angle_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"steering_wheel_angle_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"steering_wheel_angle\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"steering_wheel_speed_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"steering_wheel_speed\" of Chassis].");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"steering_wheel_torque_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"steering_wheel_torque\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].v_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"v_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"v\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].a_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"a_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"a\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].yaw_rate_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"yaw_rate_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"yaw_rate\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].ax_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"ax_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].ax), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"ax\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].ay_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"ay_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].ay), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"ay\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fl_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_fl_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_fl\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fr_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_fr_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_fr), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_fr\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rl_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rl\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rr_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rr\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl2_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rl2_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rl2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rl2\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr2_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rr2_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].wheel_speed_rr2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"wheel_speed_rr2\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"epb_status\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"gear\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].gear_number), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"gear_number\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].signal_turning_indicator), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"signal_turning_indicator\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].signal_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"signal_turn_lamp\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].signal_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"signal_brake_lamp\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].brake_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"brake_pedal_value\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].acc_pedal_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"acc_pedal_value\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_speed_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"engine_speed_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"engine_speed\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_torque_valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"engine_torque_valid\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].engine_torque), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"engine_torque\" of Chassis.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t Phoenix_Common_EncodeChassisCtlCmdArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ChassisCtlCmd_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"msg_head\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].start_robotic_ctl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"start_robotic_ctl\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_eps\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_throttle_sys\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_ebs\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_remote_ctl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_remote_ctl\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_direct_ctl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_direct_ctl\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_acc\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"release_throttle\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_angle\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_speed\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_torque\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].velocity), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"velocity\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].acceleration), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"acceleration\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].acc_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"acc_value\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].brake_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"brake_value\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"gear\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"turn_lamp\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"brake_lamp\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wiper), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"wiper\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"epb_status\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeChassisCtlCmdArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ChassisCtlCmd_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"msg_head\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].start_robotic_ctl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"start_robotic_ctl\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_eps\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_throttle_sys\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_ebs\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_remote_ctl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_remote_ctl\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_direct_ctl), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_direct_ctl\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_acc\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"release_throttle\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"steering_wheel_angle\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"steering_wheel_speed\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_torque), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"steering_wheel_torque\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].velocity), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"velocity\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].acceleration), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"acceleration\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].acc_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"acc_value\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].brake_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"brake_value\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"gear\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"turn_lamp\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"brake_lamp\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].wiper), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"wiper\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].epb_status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"epb_status\" of ChassisCtlCmd.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t Phoenix_Common_EncodeSpecialChassisInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const SpecialChassisInfo_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"msg_head\" of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].start_adas), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"start_adas\" of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can0), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_frame_loss_can0\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can1), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_frame_loss_can1\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_frame_loss_can2\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can3), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_frame_loss_can3\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can0), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_gtw_to_veh_can0\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can1), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_gtw_to_veh_can1\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_gtw_to_veh_can2\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can3), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_gtw_to_veh_can3\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can0), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_ctl_to_gtw_can0\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can1), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_ctl_to_gtw_can1\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_ctl_to_gtw_can2\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can3), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_ctl_to_gtw_can3\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_ctl_to_gtw\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeSpecialChassisInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    SpecialChassisInfo_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"msg_head\" of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].start_adas), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"start_adas\" of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can0), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_frame_loss_can0\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can1), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_frame_loss_can1\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_frame_loss_can2\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_frame_loss_can3), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cnt_stu_frame_loss_can3\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can0), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_gtw_to_veh_can0\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can1), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_gtw_to_veh_can1\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_gtw_to_veh_can2\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_gtw_to_veh_can3), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_gtw_to_veh_can3\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can0), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_ctl_to_gtw_can0\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can1), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_ctl_to_gtw_can1\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can2), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_ctl_to_gtw_can2\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw_can3), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_ctl_to_gtw_can3\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].cnt_stu_ctl_to_gtw), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cnt_stu_ctl_to_gtw\" "
                 "of SpecialChassisInfo.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


