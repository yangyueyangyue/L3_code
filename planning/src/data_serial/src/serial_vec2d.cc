//
#include "serial_vec2d.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeVec2dArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const common::Vec2d* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of Vec2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of Vec2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeVec2dArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    common::Vec2d* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of Vec2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of Vec2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


