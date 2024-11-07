//
#include "serial_msg_scene_story.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeSceneStoryControlLineArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryControlLine * p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(p[i].start_point.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_point.x\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(p[i].start_point.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_point.y\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(p[i].end_point.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"end_point.x\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(p[i].end_point.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"end_point.y\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeSceneStoryControlLineArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryControlLine* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_point.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_point.x\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].start_point.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"start_point.y\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].end_point.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"end_point.x\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].end_point.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"end_point.y\" of "
                 "SceneStoryControlLine[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t EncodeSceneStorAreayArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryArea * p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(p[i].area_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t valid_1 = p[i].area_type_01.valid;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(valid_1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_01.valid\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].area_type_01.distance, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_01.distance\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t valid_2 = p[i].area_type_02.valid;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &(valid_2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.valid\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].area_type_02.dist_to_ctl_line, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.dist_to_ctl_line\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeSceneStoryControlLineArray(buf, offset + pos, maxlen - pos,
                               &p[i].area_type_02.control_line, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.control_line\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].area_type_02.start_s, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.start_s\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].area_type_02.end_s, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.end_s\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeSceneStoryAreaArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryArea* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t valid_1 = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(valid_1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_01.valid\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].area_type_01.valid = valid_1;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area_type_01.distance), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_01.distance\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t valid_2 = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(valid_2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.valid\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].area_type_02.valid = valid_2;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area_type_02.dist_to_ctl_line), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.dist_to_ctl_line\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeSceneStoryControlLineArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area_type_02.control_line), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.control_line\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area_type_02.start_s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.start_s\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area_type_02.end_s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area_type_02.end_s\" of "
                 "SceneStoryArea[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t EncodeSceneStoryConditionArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryCondition * p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].vaild, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"vaild\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].valid_area, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_area\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].valid_speed_high, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_speed_high\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].speed_high, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed_high\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].valid_speed_low, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_speed_low\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].speed_low, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed_low\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].gear, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeSceneStoryConditionArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryCondition* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].vaild), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].valid_area), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_area\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].valid_speed_high), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_speed_high\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed_high), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed_high\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].valid_speed_low), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_speed_low\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed_low), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed_low\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t EncodeSceneStoryActionArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryAction * p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].vaild, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"vaild\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].hold_time, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"hold_time\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].valid_speed, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_speed\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].speed, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].valid_acceleration, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_acceleration\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].acceleration, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"acceleration\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].gear, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].turn_lamp, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"turn_lamp\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].brake_lamp, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_lamp\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeSceneStoryActionArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryAction* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].vaild), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"vaild\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].hold_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"hold_time\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].valid_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_speed\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"speed\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].valid_acceleration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid_acceleration\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].acceleration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"acceleration\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].gear), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"gear\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].turn_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"turn_lamp\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].brake_lamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_lamp\" of "
                 "SceneStoryCondition[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t EncodeSceneStoryArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStory * p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    std::string id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].id, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].id, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#else
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].id, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#endif

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].type, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeSceneStorAreayArray(buf, offset + pos, maxlen - pos,
                               &p[i].area, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeSceneStoryConditionArray(buf, offset + pos, maxlen - pos,
                               &p[i].condition, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"condition\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeSceneStoryActionArray(buf, offset + pos, maxlen - pos,
                               &p[i].action, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"action\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].extra_param.close_to_curve.curvature, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"extra_param.close_to_curve.curvature\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeSceneStoryArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStory* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    std::string id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#else
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#endif

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeSceneStoryAreaArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].area), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeSceneStoryConditionArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].condition), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"area\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeSceneStoryActionArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].action), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"action\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].extra_param.close_to_curve.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"extra_param.close_to_curve.curvature\" of "
                 "SceneStory[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t EncodeSceneStoryListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::SceneStoryList * p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of "
                 "SceneStoryList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                               &p[i].story_num, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"story_num\" of "
                 "SceneStoryList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t story_num = p[i].story_num;
    thislen = EncodeSceneStoryArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].storys[0]), story_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"storys\" of "
                 "SceneStoryList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeSceneStoryListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::SceneStoryList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of "
                 "SceneStoryList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    Int32_t story_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(story_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"story_num\" of "
                 "SceneStoryList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].story_num = story_num;

    thislen = DecodeSceneStoryArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].storys[0]), story_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"storys\" of "
                 "SceneStoryList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


