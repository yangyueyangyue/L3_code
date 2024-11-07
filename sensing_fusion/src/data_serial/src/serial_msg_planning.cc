#include "serial_msg_planning.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodePlanningResultArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::PlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cur_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cur_status\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_driving_mode\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_eps\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_throttle_sys\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_ebs\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].hold_steering_wheel), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"hold_steering_wheel\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"release_throttle\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_gear\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_turn_lamp\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_brake_lamp\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_v\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_a\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.timestamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"timestamp\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.x\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.y\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.h\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.c\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.s\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.l\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.x\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.y\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.h\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.c\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.s\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.y\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // 横向误差
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.lat_err.moving_flag), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"moving_flag\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t j = 0; j < 2; ++j) {
      thislen = common::EncodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_chg_rate\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"yaw_err\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"yaw_err_chg_rate\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.trj_direction), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"trj_direction\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_trj.points_num\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int16_t points_num = p[i].tar_trj.points_num;
    for (Int16_t j = 0; j < points_num; ++j) {
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].x), 1);

      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"tar_trj.points.x\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].y), 1);

      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"tar_trj.points.y\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].h), 1);

      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"tar_trj.points.h\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].c), 1);

      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"tar_trj.points.c\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].s), 1);

      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"tar_trj.points.s\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}

Int32_t DecodePlanningResultArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::PlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cur_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cur_status\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tra_driving_mode\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_eps), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_eps\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_throttle_sys), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_throttle_sys\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_ebs), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_ebs\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].hold_steering_wheel), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"hold_steering_wheel\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"release_throttle\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_gear\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_turn_lamp\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_brake_lamp\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_v\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_a\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.timestamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"timestamp\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.x\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.y\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.h\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.c\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.s\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.curr_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.l\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.x\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.y\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.h\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.c), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.c\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.s\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.leading_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.l\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // 横向误差
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.lat_err.moving_flag), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"moving_flag\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t j = 0; j < 2; ++j) {
      thislen = common::DecodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].lat_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_chg_rate\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"yaw_err\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(
            buf, offset + pos, maxlen - pos,
            &(p[i].tar_trj.lat_err.samples[j].yaw_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"yaw_err_chg_rate\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_trj.trj_direction), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"trj_direction\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int16_t points_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"trj_direction\" of PlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].tar_trj.points_num = points_num;
    
    for (Int16_t j = 0; j < points_num; ++j) {
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].x), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"tar_trj.points.x\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].y), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"tar_trj.points.y\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].h), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"tar_trj.points.h\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].c), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"tar_trj.points.c\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tar_trj.points[j].s), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"tar_trj.points.s\" of PlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix
