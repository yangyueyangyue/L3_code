#include "serial_motion_planning.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "data_serialization.h"

namespace phoenix {
namespace data_serial {

Int32_t EncodeEventChangingLaneReqArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const planning::ChangingLaneReq* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int32_t request = p[i].request();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(request), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"request\" of EventChangingLaneReq[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t sequence = p[i].sequence();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(sequence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"sequence\" of EventChangingLaneReq[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeEventChangingLaneReqArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    planning::ChangingLaneReq* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    Int32_t request = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(request), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"request\" of EventChangingLaneReq[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].set_request(request);

    Int32_t sequence = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(sequence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"sequence\" of EventChangingLaneReq[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].set_sequence(sequence);
  }
  return pos;
}


Int32_t EncodeEventChangingLaneRspArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const planning::ChangingLaneRsp* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int32_t response = p[i].response();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(response), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"response_\" of EventChangingLaneRsp[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t requset_sequence = p[i].requset_sequence();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(requset_sequence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"requset_sequence\" of EventChangingLaneRsp[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t status = p[i].status();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"status\" of EventChangingLaneRsp[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}

Int32_t DecodeEventChangingLaneRspArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    planning::ChangingLaneRsp* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    Int32_t response = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(response), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"response\" of EventChangingLaneRsp[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].set_response(response);

    Int32_t requset_sequence = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(requset_sequence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"requset_sequence\" of EventChangingLaneRsp[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].set_requset_sequence(requset_sequence);

    Int32_t status = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"status\" of EventChangingLaneRsp[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].set_status(status);


  }
  return pos;
}


Int32_t EncodeActionPlanningResultArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const planning::ActionPlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;
  
  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t enable_adas = p[i].enable_adas;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_adas), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_adas\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t enable_aeb = p[i].enable_aeb;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_aeb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_aeb\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int8_t enable_acc = p[i].enable_acc;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_acc\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int8_t enable_lka = p[i].enable_lka;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_lka), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_lka\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t enable_alc = p[i].enable_alc;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_alc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_alc\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t enable_isl = p[i].enable_isl;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_isl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_isl\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_setting), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_setting\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_setting), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a_setting\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].time_gap_setting), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"time_gap_setting\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"driving_mode\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"turn_lamp\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeEventChangingLaneReqArray(buf, offset + pos, maxlen - pos,
                                              &(p[i].changing_lane_req), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"changing_lane_req\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}

Int32_t DecodeActionPlanningResultArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    planning::ActionPlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t enable_adas = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_adas), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_adas\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].enable_adas = enable_adas;

    Int8_t enable_aeb = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_aeb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_aeb\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].enable_aeb = enable_aeb;

    Int8_t enable_acc = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_acc\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].enable_acc = enable_acc;

    Int8_t enable_lka = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_lka), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_lka\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].enable_lka = enable_lka;

    Int8_t enable_alc = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_alc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_alc\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].enable_alc = enable_alc;

    Int8_t enable_isl = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(enable_isl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_isl\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].enable_isl = enable_isl;


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_setting), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_setting\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_setting), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"a_setting\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].time_gap_setting), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"time_gap_setting\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].driving_mode), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"driving_mode\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"gear\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"turn_lamp\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = DecodeEventChangingLaneReqArray(buf, offset + pos, maxlen - pos,
                                              &(p[i].changing_lane_req), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"changing_lane_req\" of ActionPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}

Int32_t EncodeTrajectoryPlanningResultArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const planning::TrajectoryPlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;
  
  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.x\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.y\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.h\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.c\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.s\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curr_pos.l\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.x\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.y\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.h\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.c\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.s\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_pos.l\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err.moving_flag), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lat_err.moving_flag\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for(Int32_t j = 0; j < 2; ++j) {
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err.lat_err.samples["<<j<<"].lat_err\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].lat_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err.lat_err.samples["<<j<<"].lat_err_chg_rate\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err.lat_err.samples["<<j<<"].yaw_err\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].yaw_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err.lat_err.samples["<<j<<"].yaw_err_chg_rate\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_direction), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"trj_direction\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t target_trajectory_sample_points_num = p[i].target_trajectory.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(target_trajectory_sample_points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_trajectory_sample_points_num\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].target_trajectory.data(),
                                   target_trajectory_sample_points_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_trajectory_sample_points\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t hold_steering_wheel = p[i].hold_steering_wheel;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(hold_steering_wheel), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"hold_steering_wheel\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeEventChangingLaneRspArray(buf, offset + pos, maxlen - pos,
                                              &(p[i].changing_lane_rsp),
                                              1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"changing_lane_rsp\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}

Int32_t DecodeTrajectoryPlanningResultArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    planning::TrajectoryPlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.x\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.y\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.h\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.c\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.s\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curr_pos.l\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.x\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.y\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.h\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.c\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.s\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_pos.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_pos.l\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err.moving_flag), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lat_err.moving_flag\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for(Int32_t j = 0; j < 2; ++j) {
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err.samples["<< j<<"].lat_err\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].lat_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err.samples["<< j<<"].lat_err_chg_rate\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err.samples["<< j<<"].yaw_err\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err.samples[j].yaw_err_chg_rate), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err.samples["<< j<<"].yaw_err_chg_rate\" of TrajectoryPlanningResult[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_direction), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"trj_direction\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t points_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"points_num\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    p[i].target_trajectory.Resize(points_num);
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].target_trajectory.data(), points_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_trajectory\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t hold_steering_wheel = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(hold_steering_wheel), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"hold_steering_wheel\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].hold_steering_wheel = hold_steering_wheel;

    thislen = DecodeEventChangingLaneRspArray(buf, offset + pos, maxlen - pos,
                                              &(p[i].changing_lane_rsp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"changing_lane_rsp\" of TrajectoryPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t EncodeVelocityPlanningResultArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const planning::VelocityPlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;
  
  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_type\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_v\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_a\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_pos.x\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_pos.y\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_pos.h\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_pos.s\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t release_throttle = p[i].release_throttle;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"release_throttle\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t valid = p[i].tar_obj.valid;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.valid\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.obj_dec_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.obj_dec_status\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.dist_to_obj), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.dist_to_obj\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.time_gap), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.time_gap\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.relative_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.relative_v\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.ttc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.ttc\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.obj_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.obj_v\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.dist_gap_level), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.dist_gap_level\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.rel_spd_level), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tar_obj.rel_spd_level\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}

Int32_t DecodeVelocityPlanningResultArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    planning::VelocityPlanningResult* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_type\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_v\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_a\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_pos.x\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_pos.y\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_pos.h\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_pos.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_pos.s\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t release_throttle = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(release_throttle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"release_throttle\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].release_throttle = release_throttle;

    Int8_t valid = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.valid\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].tar_obj.valid = valid;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.obj_dec_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.obj_dec_status\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.dist_to_obj), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.dist_to_obj\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.time_gap), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.time_gap\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.relative_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.relative_v\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.ttc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.ttc\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.obj_v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.obj_v\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.dist_gap_level), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.dist_gap_level\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_obj.rel_spd_level), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_obj.rel_spd_level\" of VelocityPlanningResult[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t EncodeTrajectoryPlanningInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const planning::TrajectoryPlanningInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;
  
  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_length_for_lka), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"leading_length_for_lka\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].road_builed_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"road_builed_type\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t road_graph_links_size = p[i].road_graph.road_graph_links.Size();
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(road_graph_links_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"road_graph_links_size\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t j = 0; j < road_graph_links_size; ++j) {
      Int32_t sample_points_size = p[i].road_graph.road_graph_links[j].sample_points.Size();
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(sample_points_size), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"sample_points_size\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
      thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                     p[i].road_graph.road_graph_links[j].sample_points.data(),
                                     sample_points_size);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"sample_points\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
    
    Int32_t optimal_trajectory_sample_points_num =
        p[i].optimal_trajectory_sample_points.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(optimal_trajectory_sample_points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"optimal_trajectory_sample_points_num\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].optimal_trajectory_sample_points.data(),
                                   optimal_trajectory_sample_points_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"optimal_trajectory_sample_points\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t prev_ref_line_sample_points_num =
        p[i].prev_ref_line_sample_points.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(prev_ref_line_sample_points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"prev_ref_line_sample_points_num\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].prev_ref_line_sample_points.data(),
                                   prev_ref_line_sample_points_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"prev_ref_line_sample_points\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t k = 0; k < 2; ++k) {
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].lat_err\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].lat_err_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err_v), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].lat_err_v\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err_v_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].lat_err_v_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }


      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].yaw_err\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].yaw_err_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err_v), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].yaw_err_v\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err_v_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"lat_err_sample["<< k <<"].yaw_err_v_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }
  return pos;
}

Int32_t DecodeTrajectoryPlanningInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    planning::TrajectoryPlanningInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].leading_length_for_lka), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"leading_length_for_lka\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].road_builed_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"road_builed_type\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t road_graph_links_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(road_graph_links_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tar_type\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].road_graph.road_graph_links.Resize(road_graph_links_size);
    for (Int32_t j = 0; j < road_graph_links_size; ++j) {
      Int32_t sample_points_size = 0;
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(sample_points_size), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"sample_points_size\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      p[i].road_graph.road_graph_links[j].sample_points.Resize(sample_points_size);
      thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                     p[i].road_graph.road_graph_links[j].sample_points.data()
                                     , sample_points_size);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"sample_points_size\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
    Int32_t optimal_trajectory_sample_points_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(optimal_trajectory_sample_points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"optimal_trajectory_sample_points_num\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].optimal_trajectory_sample_points.Resize(optimal_trajectory_sample_points_num);
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].optimal_trajectory_sample_points.data(),
                                   optimal_trajectory_sample_points_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"optimal_trajectory_sample_points\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t prev_ref_line_sample_points_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(prev_ref_line_sample_points_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"prev_ref_line_sample_points_num\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].prev_ref_line_sample_points.Resize(prev_ref_line_sample_points_num);
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].prev_ref_line_sample_points.data(),
                                   prev_ref_line_sample_points_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"prev_ref_line_sample_points\" of TrajectoryPlanningInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    for (Int32_t k = 0; k < 2; ++k) {
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].lat_err\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].lat_err_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err_v), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].lat_err_v\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].lat_err_v_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].lat_err_v_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }


      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].yaw_err\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].yaw_err_smooth\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err_v), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].yaw_err_v\" of TrajectoryPlanningInfo[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].lat_err_sample[k].yaw_err_v_smooth), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"lat_err_sample["<< k <<"].yaw_err_v_smooth\" of TrajectoryPlanningInfo[" << i << "].";
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

