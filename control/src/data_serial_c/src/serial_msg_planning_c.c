#include "serial_msg_planning_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/serialization_utils_c.h"
#include "serial_msg_common_c.h"


Int32_t Phoenix_Common_EncodePlanningResultArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const PlanningResult_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"msg_head\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].planning_status[0]), 4);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"cur_status\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_driving_mode\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_eps\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_throttle_sys\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"enable_ebs\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].hold_steering_wheel), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"hold_steering_wheel\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"steering_wheel_speed\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"release_throttle\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_gear), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_gear\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_turn_lamp\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_brake_lamp\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_v), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_v\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_a), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_a\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_EncodeInt64Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.timestamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"timestamp\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"curr_pos.x\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"curr_pos.y\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"curr_pos.h\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"curr_pos.c\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"curr_pos.s\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"curr_pos.l\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"leading_pos.x\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"leading_pos.y\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"leading_pos.h\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"leading_pos.c\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"leading_pos.s\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"leading_pos.y\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    // 横向误差
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].tar_trj.lat_err.moving_flag), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"moving_flag\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t j = 0; j < 2; ++j) {
      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"lat_err\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"lat_err_chg_rate\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"yaw_err\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"yaw_err_chg_rate\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.trj_direction), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"trj_direction\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt16Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points_num), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"tar_trj.points_num\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int16_t points_num = p[i].tar_trj.points_num;
    for (Int16_t j = 0; j < points_num; ++j) {
      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].x), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"tar_trj.points.x\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].y), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"tar_trj.points.y\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].h), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"tar_trj.points.h\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].c), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"tar_trj.points.c\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_EncodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].s), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to encode \"tar_trj.points.s\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].pitch), 1);

    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"pitch\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].pitch_variance), 1);

    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"pitch_variance\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodePlanningResultArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    PlanningResult_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"msg_head\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].planning_status[0]), 4);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"cur_status\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_driving_mode\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_eps\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_throttle_sys\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"enable_ebs\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].hold_steering_wheel), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"hold_steering_wheel\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].steering_wheel_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"steering_wheel_speed\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"release_throttle\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_gear), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_gear\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_turn_lamp\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_brake_lamp\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_v), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_v\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_a), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_a\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_DecodeInt64Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.timestamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"timestamp\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"curr_pos.x\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"curr_pos.y\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"curr_pos.h\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"curr_pos.c\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"curr_pos.s\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.curr_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"curr_pos.l\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"leading_pos.x\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"leading_pos.y\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"leading_pos.h\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"leading_pos.c\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"leading_pos.s\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.leading_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"leading_pos.y\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    // 横向误差
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].tar_trj.lat_err.moving_flag), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"moving_flag\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t j = 0; j < 2; ++j) {
      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"lat_err\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"lat_err_chg_rate\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"yaw_err\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"yaw_err_chg_rate\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.trj_direction), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"trj_direction\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt16Array(
          buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points_num), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to Decode \"tar_trj.points_num\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    Int16_t points_num = p[i].tar_trj.points_num;
    for (Int16_t j = 0; j < points_num; ++j) {
      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].x), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"tar_trj.points.x\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].y), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"tar_trj.points.y\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].h), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"tar_trj.points.h\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].c), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"tar_trj.points.c\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = Phoenix_Common_DecodeFloat32Array(
            buf, offset + pos, maxlen - pos, &(p[i].tar_trj.points[j].s), 1);

      if (thislen < 0) {
        LOG_ERR_C("Failed to Decode \"tar_trj.points.s\" of PlanningResult.");
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].pitch), 1);

    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"pitch\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].pitch_variance), 1);

    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"pitch_variance\" of PlanningResult.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

