#include "serial_msg_lane_camera.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeLaneMarkCameraArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LaneMarkCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t lane_mark_type = p[i].lane_mark_type;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(lane_mark_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane_mark_type\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"quality\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t view_range_valid = p[i].view_range_valid;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(view_range_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"view_range_valid\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].mark_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"mark_width\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].view_range_start), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"view_range_start\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].view_range_end), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"view_range_end\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"c0\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"c1\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"c2\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"c3\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeLaneMarkCameraArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LaneMarkCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"id\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int8_t lane_mark_type = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(lane_mark_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane_mark_type\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    switch (lane_mark_type) {
    case (0):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
      break;
    case (1):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
      break;
    case (2):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
      break;
    case (3):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
      break;
    case (4):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK;
      break;
    case (5):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS;
      break;
    case (6):
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
      break;
    default:
      p[i].lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
      break;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"quality\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int8_t view_range_valid = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(view_range_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"view_range_valid\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].view_range_valid = view_range_valid;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].mark_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"mark_width\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].view_range_start), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"view_range_start\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].view_range_end), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"view_range_end\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c0), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"c0\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c1), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"c1\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c2), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"c2\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].c3), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"c3\" of LaneMarkCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}
// struct LaneMarkCameraList {
//   /**
//    * @enum MAX_LANE_MARK_NUM
//    * @brief 列表中最大车道线的数量
//    */
//   enum { MAX_LANE_MARK_NUM = 4 };

//   /// 消息头
//   MsgHead msg_head;
//   /// 车道线的数量
//   Int8_t lane_mark_num;
//   /// 感知识别的车道线列表
//   LaneMarkCamera lane_marks[MAX_LANE_MARK_NUM];

// };
Int32_t EncodeLaneMarkCameraListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LaneMarkCameraList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of LaneMarkCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane_mark_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane_mark_num\" of LaneMarkCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int8_t size = p[i].lane_mark_num;
    thislen = EncodeLaneMarkCameraArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].lane_marks[0]), size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"LaneMarkCamera\" of LaneMarkCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}
Int32_t DecodeLaneMarkCameraListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LaneMarkCameraList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"MsgHead\" of LaneMarkCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int8_t lane_mark_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(lane_mark_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane_mark_num\" of LaneMarkCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].lane_mark_num = lane_mark_num;
    thislen = DecodeLaneMarkCameraArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].lane_marks[0]), lane_mark_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane_marks\" of TrajectorLaneMarkCameraListyPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

// struct LaneBoundaryLineCamera {
//   /**
//    * @enum MAX_CURVE_POINT_NUM
//    * @brief 车道线中点的最大数量
//    */
//   enum { MAX_CURVE_POINT_NUM = 500 };

//   /// 车道编号，左边车道线为正值，右边车道线为负数;
//   /// (数值的绝对值按照距离当前车道的远近依次增加，
//   /// 第一条左边的车道线为1，第一条右边的车道线为-1)
//   Int32_t id;
//   /// 边线类型
//   Int32_t type;
//   /// 车道线中点的数量
//   Int32_t curve_point_num;
//   /// 车道线
//   struct CurvePoint {
//     bool fake;
//     /// 形点坐标x
//     Float32_t x;
//     /// 形点坐标y
//     Float32_t y;
//   } curve[MAX_CURVE_POINT_NUM]
// };
Int32_t EncodeCurvePointArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LaneBoundaryLineCamera::CurvePoint* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int8_t fake = p[i].fake;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(fake), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"fake\" of CurvePoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of CurvePoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of CurvePoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}
Int32_t DecodeCurvePointArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LaneBoundaryLineCamera::CurvePoint* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int8_t fake = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(fake), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of CurvePoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].fake = fake;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of CurvePoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of CurvePoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}
Int32_t EncodeLaneBoundaryLineCameraArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LaneBoundaryLineCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curve_point_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curve_point_num\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = EncodeCurvePointArray(buf, offset + pos, maxlen - pos,
                                    &(p[i].curve[0]), p[i].curve_point_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curve_point\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeLaneBoundaryLineCameraArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LaneBoundaryLineCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"id\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"type\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t curve_point_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(curve_point_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curve_point_num\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    p[i].curve_point_num = curve_point_num;
    thislen = DecodeCurvePointArray(buf, offset + pos, maxlen - pos,
                                    &(p[i].curve[0]), curve_point_num);

    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curve_point\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

// struct LaneCentralLineCamera {
//   /**
//    * @enum MAX_CURVE_POINT_NUM
//    * @brief 车道线中点的最大数量
//    */
//   enum { MAX_CURVE_POINT_NUM = 500 };

//   /// 车道线识别的质量, 3 - very good, 2 - not good, 1 - bad, 0 - invalid
//   Int8_t quality;
//   /// 历史数据存在的次数
//  Int32_t age;
//   /// 从车辆当前位置开始，车道线的前向长度
//   Float32_t forward_len;
//   /// 车道中心线编号，左边车道中心线为正值，右边车道中心线为负数;
//   /// (数值的绝对值按照距离当前车道的远近依次增加，
//   /// 当前车道中心线为0,第一条左边的车道中心线为1，第一条右边的车道中心线为-1)
//   Int32_t id;
//   /// 车道左边线的索引
//   Int32_t left_boundary_index;
//   /// 车道右边线的索引
//   Int32_t right_boundary_index;
//   /// 左边车道中心线的索引
//   Int32_t left_central_line_index;
//   /// 右边车道中心线的索引
//   Int32_t right_central_line_index;
//   /// 车道中心线点的数量
//   Int32_t curve_point_num;
//   /// 车道中心线
//   struct CurvePoint {
//     /// 形点坐标x
//     Float32_t x;
//     /// 形点坐标y
//     Float32_t y;
//     /// 车道中心线到左边线的宽度
//     Float32_t left_width;
//     /// 车道中心线到右边线的宽度
//     Float32_t right_width;
//   } curve[MAX_CURVE_POINT_NUM];
// };


Int32_t EncodeLaneCenterLineCameraArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LaneCenterLineCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"quality\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"age\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].forward_len), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"forward_len\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].left_boundary_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"left_boundary_index\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].right_boundary_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"right_boundary_index\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curve_point_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curve_point_num\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t curve_point_num = p[i].curve_point_num;
    for (Int32_t j = 0; j < curve_point_num; ++j) {
      const ad_msg::LaneCenterLineCamera::CurvePoint& curve_point
          = p[i].curve[j];
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.x), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"x\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.y), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"y\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.left_width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_width\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.right_width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"right_width\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}

Int32_t DecodeLaneCenterLineCameraArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LaneCenterLineCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"quality\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"age\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].forward_len), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"forward_len\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"id\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].left_boundary_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"left_boundary_index\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].right_boundary_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"right_boundary_index\" of LaneCentralLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t curve_point_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(curve_point_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curve_point_num\" of LaneBoundaryLineCamera[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].curve_point_num = curve_point_num;
    for (Int32_t j = 0; j < curve_point_num; ++j) {

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].curve[j].x), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"x\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].curve[j].y), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"y\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].curve[j].left_width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_width\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].curve[j].right_width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"right_width\" of curve of LaneCentralLineCamera[" << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }
  return pos;
}

// struct LaneInfoCameraList {
//   /**
//    * @enum MAX_LANE_MARK_NUM
//    * @brief 列表中最大车道线的数量
//    */
//   enum { MAX_LANE_BOUNDARY_LINE_NUM = 4 };
//   /**
//    * @enum MAX_LANE_CENTRAL_LINE_NUM
//    * @brief 列表中最大车道中心线的数量
//    */
//   enum { MAX_LANE_CENTRAL_LINE_NUM = 3 };

//   /// 消息头
//   MsgHead msg_head;
//   /// 车道线识别的质量, 3 - very good, 2 - not good, 1 - bad, 0 - invalid
//   Int8_t quality;
//   /// 从车辆当前位置开始，车道线的前向长度
//   Float32_t forward_len;
// /// 车辆当前位置左边车道宽度
//Float32_t left_width;
// /// 车辆当前位置右边车道宽度
//Float32_t right_width;

//   /// 车道线的数量
//   Int32_t lane_line_num;
//   /// 感知识别的车道线列表
//   LaneBoundaryLineCamera boundary_lines[MAX_LANE_BOUNDARY_LINE_NUM];
//   /// 车道中心线的数量
//   Int32_t central_line_num;
//   /// 车道中心线列表
//   LaneCentralLineCamera central_lines[MAX_LANE_CENTRAL_LINE_NUM];
// };


Int32_t EncodeLaneInfoCameraListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LaneInfoCameraList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"quality\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].forward_len), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"forward_len\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].left_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"left_width\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].right_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"right_width\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].boundary_line_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"boundary_line_num\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int32_t boundary_line_num = p[i].boundary_line_num;
    thislen = EncodeLaneBoundaryLineCameraArray(buf, offset + pos, maxlen - pos,
                                                &(p[i].boundary_lines[0]), boundary_line_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"LaneBoundaryLineCamera\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].center_line_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"center_line_num\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t center_line_num = p[i].center_line_num;
    thislen = EncodeLaneCenterLineCameraArray(buf, offset + pos, maxlen - pos,
                                              &(p[i].center_lines[0]), center_line_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"LaneMarkCamera\" of LaneInfoCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeLaneInfoCameraListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LaneInfoCameraList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"MsgHead\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"quality\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].forward_len), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"forward_len\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].left_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"left_width\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].right_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"right_width\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int32_t boundary_line_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(boundary_line_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"boundary_line_num\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].boundary_line_num = boundary_line_num;

    thislen = DecodeLaneBoundaryLineCameraArray(buf, offset + pos, maxlen - pos,
                                                &(p[i].boundary_lines[0]), boundary_line_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"boundary_lines\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t center_line_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(center_line_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"center_line_num\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].center_line_num = center_line_num;

    thislen = DecodeLaneCenterLineCameraArray(buf, offset + pos, maxlen - pos,
                                              &(p[i].center_lines[0]), center_line_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"center_lines\" of LaneInfoCameraList["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix
