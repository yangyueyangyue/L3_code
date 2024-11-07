//
#include "serial_obbox2d.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_vec2d.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeOBBox2dArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const common::OBBox2d* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].center()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"center\" of OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].unit_direction_x()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"unit_direction_x\" of "
                 "OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].unit_direction_y()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"unit_direction_y\" of "
                 "OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].extents()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"extents\" of "
                 "OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeOBBox2dArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    common::OBBox2d* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].center()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"center\" of OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].unit_direction_x()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"unit_direction_x\" of "
                 "OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].unit_direction_y()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"unit_direction_y\" of "
                 "OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].extents()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"extents\" of "
                 "OBBox2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


