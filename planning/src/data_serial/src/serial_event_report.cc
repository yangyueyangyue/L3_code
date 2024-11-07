//
#include "serial_msg_common.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"

namespace phoenix {
namespace data_serial {


Int32_t EncodeEventReportingArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::EventReporting* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"MsgHead\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].sub_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"sub_module_id\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].event_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"event_id\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].priority), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"priority\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lifetime), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lifetime\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].param[0]), 4);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"param\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeEventReportingArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::EventReporting* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {

    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].sub_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"sub_module_id\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].event_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"event_id\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].priority), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"priority\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lifetime), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lifetime\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].param[0]), 4);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lifetime\" of EventReporting["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t EncodeEventReportingListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::EventReportingList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of EventReportingList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].event_reporting_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"relative_pos_num\" of EventReportingList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t event_reporting_num = p[i].event_reporting_num;
    thislen = EncodeEventReportingArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].event_reporting[0]),event_reporting_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"event_reporting\" of EventReportingList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeEventReportingListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::EventReportingList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of EventReportingList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t event_reporting_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(event_reporting_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of EventReportingList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].event_reporting_num = event_reporting_num;

    thislen = DecodeEventReportingArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].event_reporting[0]), event_reporting_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"event_reporting\" of EventReportingList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


