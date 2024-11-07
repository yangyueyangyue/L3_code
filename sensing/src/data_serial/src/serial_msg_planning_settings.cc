#include "serial_msg_planning_settings.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodePlanningSettingsArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::PlanningSettings* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_adas), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_adas\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_lka), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_lka\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_acc\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_aeb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_aeb\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_alc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_alc\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_isl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_isl\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_ngp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"enable_ngp\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_velocity_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_velocity_valid\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_velocity), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_velocity\" "
                 "of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_acc_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_acc_valid\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_acc\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_time_gap_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_time_gap_valid\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_time_gap), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"target_time_gap\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].changing_lane_req), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"changing_lane_req\" "
                 "of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodePlanningSettingsArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::PlanningSettings* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_adas), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"start_adas\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_lka), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_lka\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_acc\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_aeb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_aeb\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_alc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_alc\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_isl), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_isl\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].enable_ngp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"enable_ngp\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_velocity_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_velocity_valid\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_velocity), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_velocity\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_acc_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_acc_valid\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_acc), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_acc\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_time_gap_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_time_gap_valid\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_time_gap), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"target_time_gap\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].changing_lane_req), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"changing_lane_req\" of HmiSettings[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix
