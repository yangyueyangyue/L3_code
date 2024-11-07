#include "serial_msg_traffic_signal.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeTrafficSignalBoxArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::TrafficSignalBox* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].z), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"z\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"width\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].height), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"height\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].camera_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"camera_id\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeTrafficSignalBoxArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::TrafficSignalBox* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].z), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"z\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"width\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].height), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"height\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].camera_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"camera_id\" of TrafficSignalBox["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeTrafficSignalSpeedRestrictionArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::TrafficSignalSpeedRestriction* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    /// TODO: 暂时不对id进行编码/解码
    //#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    //  std::string id;
    //#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    //  Uint64_t id;
    //#else
    //  Uint64_t id;
    //#endif
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of TrafficSignalSpeedRestriction["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeTrafficSignalBoxArray(buf, offset + pos, maxlen - pos,
                                          &(p[i].box), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"box\" of TrafficSignalSpeedRestriction["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed\" of TrafficSignalSpeedRestriction["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeTrafficSignalSpeedRestrictionArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::TrafficSignalSpeedRestriction* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    /// TODO: 暂时不对id进行编码/解码
    //#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    //  std::string id;
    //#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    //  Uint64_t id;
    //#else
    //  Uint64_t id;
    //#endif
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"type\" of TrafficSignalSpeedRestriction["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeTrafficSignalBoxArray(buf, offset + pos, maxlen - pos,
                                          &(p[i].box), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"box\" of TrafficSignalSpeedRestriction["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"speed\" of TrafficSignalSpeedRestriction["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeTrafficSignalListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::TrafficSignalList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"MsgHead\" of TrafficSignalList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed_restriction_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed_restriction_num\" of TrafficSignalList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeTrafficSignalSpeedRestrictionArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].speed_restrictions[0]), p[i].speed_restriction_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed_restrictions\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeTrafficSignalListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::TrafficSignalList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"MsgHead\" of TrafficSignalList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed_restriction_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"speed_restriction_num\" of TrafficSignalList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = DecodeTrafficSignalSpeedRestrictionArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].speed_restrictions[0]), p[i].speed_restriction_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"speed_restrictions\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeTrafficLightArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::TrafficLight* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    /// TODO: 暂时不对id进行编码/解码
    //#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    //  std::string id;
    //#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    //  Uint64_t id;
    //#else
    //  Uint64_t id;
    //#endif
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].color), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"color\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeTrafficSignalBoxArray(buf, offset + pos, maxlen - pos,
                                          &(p[i].box), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"box\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].confidence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"confidence\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tracking_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tracking_time\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].blink), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"blink\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].remaining_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"remaining_time\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeTrafficLightArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::TrafficLight* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    /// TODO: 暂时不对id进行编码/解码
    //#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    //  std::string id;
    //#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    //  Uint64_t id;
    //#else
    //  Uint64_t id;
    //#endif
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].color), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"color\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeTrafficSignalBoxArray(buf, offset + pos, maxlen - pos,
                                          &(p[i].box), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"box\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].confidence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"confidence\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tracking_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"tracking_time\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].blink), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"blink\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].remaining_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"remaining_time\" of TrafficLight["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeTrafficLightListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::TrafficLightList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"MsgHead\" of TrafficLightList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].traffic_light_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"traffic_light_num\" of TrafficLightList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeTrafficLightArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].traffic_lights[0]), p[i].traffic_light_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"traffic_lights\" of TrafficLightList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeTrafficLightListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::TrafficLightList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"MsgHead\" of TrafficLightList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].traffic_light_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"traffic_light_num\" of TrafficLightList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = DecodeTrafficLightArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].traffic_lights[0]), p[i].traffic_light_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"traffic_lights\" of TrafficLightList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix
