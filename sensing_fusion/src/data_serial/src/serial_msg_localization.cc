#include "serial_msg_localization.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeGnssArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::Gnss* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].latitude), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"latitude\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].longitude), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"longitude\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].altitude), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"altitude\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading_gnss), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading_gnss\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].z_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"z_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].z_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"z_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pitch), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"pitch\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].roll), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"roll\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_e), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_e\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_n), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_n\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_u), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_u\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_z_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_z_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_z_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_z_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gnss_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gnss_status\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].utm_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"utm_status\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].odom_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"odom_status\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeGnssArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::Gnss* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].latitude), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"latitude\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].longitude), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"longitude\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].altitude), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"altitude\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading_gnss), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading_gnss\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].z_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"z_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"heading_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].z_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"z_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"heading_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pitch), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"pitch\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].roll), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"roll\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_e), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_e\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_n), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_n\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_u), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_u\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_x_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_y_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_z_utm), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_z_utm\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_y_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_z_odom), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_z_odom\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gnss_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"gnss_status\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].utm_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"utm_status\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].odom_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"odom_status\" of Gnss["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

// struct Imu {
//   /// 消息头
//   MsgHead msg_head;
//   /// 航向角的角速度
//   Float32_t yaw_rate;
//   /// 俯仰角的角速度
//   Float32_t pitch_rate;
//   /// 横滚角的角速度
//   Float32_t roll_rate;
//   /// 沿着车身x轴的加速度
//   Float32_t accel_x;
//   /// 沿着车身y轴的加速度
//   Float32_t accel_y;
//   /// 沿着车身z轴的加速度
//   Float32_t accel_z;
// };
Int32_t EncodeImuArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::Imu* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pitch_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"pitch_rate\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].roll_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"roll_rate\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_x\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_y\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_z), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_z\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeImuArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::Imu* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pitch_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"pitch_rate\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].roll_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"roll_rate\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_x\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_y\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_z), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_z\" of Imu[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}
// Float32_t x;
// Float32_t y;
// Float32_t heading;
// Float32_t yaw_rate;
// Float32_t v;

Int32_t EncodeRelativePosArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::RelativePos* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen =  common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].relative_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"relative_time\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen =  common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeRelativePosArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::RelativePos* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].relative_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"relative_time\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"heading\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v\" of RelativePos[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}
// struct RelativePosList {
//   /// 最大位置点的数量
//   enum { MAX_RELATIVE_POS_NUM = 40 };

//   /// 消息头
//   MsgHead msg_head;
//   /// 相对位置点的数量
//   Int32_t relative_pos_num;
//   /// 相对位置点的列表
//   RelativePos relative_pos[MAX_RELATIVE_POS_NUM];
// };
Int32_t EncodeRelativePosListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::RelativePosList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of RelativePosList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].relative_pos_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"relative_pos_num\" of RelativePosList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t relative_pos_num = p[i].relative_pos_num;
    thislen = EncodeRelativePosArray(buf, offset + pos, maxlen - pos,
                                     &(p[i].relative_pos[0]),relative_pos_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"relative_pos\" of RelativePosList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeRelativePosListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::RelativePosList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of RelativePosList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int32_t relative_pos_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(relative_pos_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of RelativePosList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].relative_pos_num = relative_pos_num;

    thislen = DecodeRelativePosArray(buf, offset + pos, maxlen - pos,
                                     &(p[i].relative_pos[0]), relative_pos_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"relative_pos\" of RelativePosList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}


}  // namespace data_serial
}  // namespace phoenix

