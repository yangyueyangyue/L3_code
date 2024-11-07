#include "pcc_map/adasisv2_msg_parser.h"

namespace phoenix {
namespace adasisv2 {

ADASISv2MsgParser::ADASISv2MsgParser() {
}


ADASISv2MsgParser::~ADASISv2MsgParser() {
}

static inline uint16_t unpack_left_shift_u16(uint8_t value, uint8_t shift, uint8_t mask) {
  return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(uint8_t value, uint8_t shift, uint8_t mask) {
  return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(uint8_t value, uint8_t shift, uint8_t mask) {
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

template<typename T> T range_limit(T value, T min, T max) {
  T limit_value;
  if (value < min) {
    limit_value = min;
  } else if (value > max) {
    limit_value = max;
  } else {
    limit_value = value;
  }
  return limit_value;
} 

MessageType ADASISv2MsgParser::ParseCanFrame(const can_dev::CanFrame &frame) {
  MessageType msg_type = ADASIS_V2_MESSAGE_TYPE_INVALID;
  switch (frame.id) {
  case (ADASIS_V2_ADAS_POSN_FRAME_ID): {  // POSITION, SEGMENT, STUB
    const Uint8_t* data = frame.data;
    
    // check message type firstly
    Uint8_t type =  range_limit<Uint8_t>(unpack_right_shift_u8(data[7], 5u, 0xe0u), 0, 7);
    switch (type) {
    case ADASIS_V2_MESSAGE_TYPE_POSITION: { // POSITION
      // decode POSITION CAN signals to structure
      adasis_v2_adas_posn_t position_raw_can;
      position_raw_can.adas_posn_rel_head = unpack_right_shift_u8(data[0], 0u, 0xffu);

      position_raw_can.adas_posn_slope = unpack_right_shift_u16(data[1], 0u, 0xffu);
      position_raw_can.adas_posn_slope |= unpack_left_shift_u16(data[2], 8u, 0x03u);

      position_raw_can.adas_posn_cur_lane = unpack_right_shift_u8(data[2], 2u, 0x0cu);

      position_raw_can.adas_posn_pos_confdc = unpack_right_shift_u8(data[2], 4u, 0x70u);

      position_raw_can.adas_posn_age = unpack_right_shift_u16(data[3], 0u, 0xffu);
      position_raw_can.adas_posn_age |= unpack_left_shift_u16(data[4], 8u, 0x01u);

      position_raw_can.adas_posn_pos_probb = unpack_right_shift_u8(data[4], 1u, 0x3eu);

      position_raw_can.adas_posn_idx = unpack_right_shift_u8(data[4], 6u, 0xc0u);

      position_raw_can.adas_posn_path_idx = unpack_right_shift_u8(data[5], 0u, 0x3fu);

      position_raw_can.adas_posn_cyc_cnt = unpack_right_shift_u8(data[5], 6u, 0xc0u);

      position_raw_can.adas_posn_offset = unpack_right_shift_u16(data[6], 0u, 0xffu);
      position_raw_can.adas_posn_offset |= unpack_left_shift_u16(data[7], 8u, 0x1fu);

      position_raw_can.adas_posn_msg_typ = unpack_right_shift_u8(data[7], 5u, 0xe0u);

      // logical value convert to physical value
      position_.time_stamp = frame.time_stamp;
      position_.type = MessageType::ADASIS_V2_MESSAGE_TYPE_POSITION;
      position_.cyclic_counter = range_limit<Uint8_t>(position_raw_can.adas_posn_cyc_cnt, 0, 3);
      position_.path_index = range_limit<Uint8_t>(position_raw_can.adas_posn_path_idx, 0, 63);
      position_.offset = range_limit<Uint16_t>(position_raw_can.adas_posn_offset, 0, 8191);
      position_.position_index = range_limit<Uint8_t>(position_raw_can.adas_posn_idx, 0, 3);
      position_.positioin_age = range_limit<Uint16_t>(position_raw_can.adas_posn_age * 5, 0, 2555);
      position_.slope = range_limit<Float32_t>((Float32_t)position_raw_can.adas_posn_slope * 0.1 - 51.1, -51.1, 51.2);
      position_.relative_heading = range_limit<Float32_t>((Float32_t)position_raw_can.adas_posn_rel_head * 1.417 - 179.959, -180, 180);
      position_.position_probability = range_limit<Float32_t>((Float32_t)position_raw_can.adas_posn_pos_probb * 3.333, 0, 100);
      position_.position_confidence = range_limit<Uint8_t>(position_raw_can.adas_posn_pos_confdc, 0, 7);
      position_.current_lane = (LaneType)range_limit<Uint8_t>(position_raw_can.adas_posn_cur_lane, 0, 7);

      msg_type = ADASIS_V2_MESSAGE_TYPE_POSITION;
    }
    break;

    case ADASIS_V2_MESSAGE_TYPE_SEGMENT: { // SEGMENT
      // LOG_INFO(3) << "[ADASIS] SEGMENT : Don't deal";
    }
    break;

    case ADASIS_V2_MESSAGE_TYPE_STUB: { // STUB
      // LOG_INFO(3) << "[ADASIS] STUB : Don't deal";
    }
    break;

    default:
    break;
  }
  }
  break;

  case (ADASIS_V2_ADAS_PROFSHORT_FRAME_ID): { // PROFILE SHORT
    const Uint8_t* data = frame.data;
    
    // check message type firstly
    Uint8_t type =  range_limit<Uint8_t>(unpack_right_shift_u8(data[7], 5u, 0xe0u), 0, 7);
    if (ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT != type) {
      break;
    }

    // decode PROFILESHORT CAN signals to structure
    adasis_v2_adas_profshort_t profileshort_raw_can;

    profileshort_raw_can.adas_profshort_value1 = unpack_right_shift_u16(data[0], 0u, 0xffu);
    profileshort_raw_can.adas_profshort_value1 |= unpack_left_shift_u16(data[1], 8u, 0x03u);

    profileshort_raw_can.adas_profshort_value0 = unpack_right_shift_u16(data[1], 2u, 0xfcu);
    profileshort_raw_can.adas_profshort_value0 |= unpack_left_shift_u16(data[2], 6u, 0x0fu);

    profileshort_raw_can.adas_profshort_dist1 = unpack_right_shift_u16(data[2], 4u, 0xf0u);
    profileshort_raw_can.adas_profshort_dist1 |= unpack_left_shift_u16(data[3], 4u, 0x3fu);

    profileshort_raw_can.adas_profshort_accur_class = unpack_right_shift_u8(data[3], 6u, 0xc0u);

    profileshort_raw_can.adas_profshort_update = unpack_right_shift_u8(data[4], 0u, 0x01u);

    profileshort_raw_can.adas_profshort_retr = unpack_right_shift_u8(data[4], 1u, 0x02u);

    profileshort_raw_can.adas_profshort_ctrl_point = unpack_right_shift_u8(data[4], 2u, 0x04u);

    profileshort_raw_can.adas_profshort_prof_type = unpack_right_shift_u8(data[4], 3u, 0xf8u);

    profileshort_raw_can.adas_profshort_path_idx = unpack_right_shift_u8(data[5], 0u, 0x3fu);

    profileshort_raw_can.adas_profshort_cyc_cnt = unpack_right_shift_u8(data[5], 6u, 0xc0u);

    profileshort_raw_can.adas_profshort_offset = unpack_right_shift_u16(data[6], 0u, 0xffu);
    profileshort_raw_can.adas_profshort_offset |= unpack_left_shift_u16(data[7], 8u, 0x1fu);

    profileshort_raw_can.adas_profshort_msg_typ = unpack_right_shift_u8(data[7], 5u, 0xe0u);

    // logical value convert to physical value
    profile_short_.time_stamp = frame.time_stamp;//common::GetClockNowMs();
    profile_short_.type = MessageType::ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT;
    profile_short_.cyclic_counter = range_limit<Uint8_t>(profileshort_raw_can.adas_profshort_cyc_cnt, 0, 3);
    profile_short_.retransmission = profileshort_raw_can.adas_profshort_retr;

    profile_short_.path_index = range_limit<Uint8_t>(profileshort_raw_can.adas_profshort_path_idx, 0, 63);

    profile_short_.offset = range_limit<Uint16_t>(profileshort_raw_can.adas_profshort_offset, 0, 8191);

    profile_short_.update = profileshort_raw_can.adas_profshort_update;

    profile_short_.profile_type = range_limit<Uint8_t>(profileshort_raw_can.adas_profshort_prof_type, 0, 31);

    profile_short_.control_point = profileshort_raw_can.adas_profshort_ctrl_point;
   
    profile_short_.distance1 = range_limit<Uint16_t>(profileshort_raw_can.adas_profshort_dist1, 0, 1023);

    profile_short_.accuracy = range_limit<Uint8_t>(profileshort_raw_can.adas_profshort_accur_class, 0, 3);

    if (PROFILE_SHORT_TYPE_SLOPE_STEP == profile_short_.profile_type ||  PROFILE_SHORT_TYPE_SLOPE_LINEAR == profile_short_.profile_type) {
      profile_short_.value0 = range_limit<Float32_t>((Float32_t)profileshort_raw_can.adas_profshort_value0 * 0.1 - 51.1, -51.1, 51.2);
      profile_short_.value1 = range_limit<Float32_t>((Float32_t)profileshort_raw_can.adas_profshort_value1 * 0.1 - 51.1, -51.1, 51.2);
    } else {
      profile_short_.value0 = range_limit<Float32_t>((Float32_t)profileshort_raw_can.adas_profshort_value0, 0, 1023);
      profile_short_.value1 = range_limit<Float32_t>((Float32_t)profileshort_raw_can.adas_profshort_value1, 0, 1023);
    }

    msg_type = ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT;
  }
  break;

  default:
    break;
  }

  return msg_type;
}


} // namespace adasisv2
} // namespace phoenix

