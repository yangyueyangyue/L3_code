#include "serial_msg_localization_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/serialization_utils_c.h"
#include "serial_msg_common_c.h"


Int32_t Phoenix_Common_EncodeImuArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const Imu_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"msg_head\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"yaw_rate\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
                                       &(p[i].pitch_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"pitch_rate\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].roll_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"roll_rate\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"accel_x\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"accel_y\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].accel_z), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"accel_z\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeImuArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    Imu_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"msg_head\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"yaw_rate\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].pitch_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"pitch_rate\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].roll_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"roll_rate\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"accel_x\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"accel_y\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos, &(p[i].accel_z), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"accel_z\" of Imu.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


