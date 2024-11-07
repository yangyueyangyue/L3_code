//
#include "serial_line_segment2d.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_vec2d.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeLineSegment2dArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const common::LineSegment2d* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].start()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start\" of LineSegment2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].end()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"end\" of LineSegment2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeLineSegment2dArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    common::LineSegment2d* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].start()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"start\" of LineSegment2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].end()), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"end\" of LineSegment2d[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


