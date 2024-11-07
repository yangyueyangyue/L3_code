//
#include "serial_msg_obstacle.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


// struct OBBox {
//   /// 中心坐标x
//   Float32_t x;
//   /// 中心坐标y
//   Float32_t y;
//   /// 航向角
//   Float32_t heading;
//   /// 半宽
//   Float32_t half_width;
//   /// 半长
//   Float32_t half_length;
// };

Int32_t EncodeOBBoxArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::OBBox* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y),1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].half_width),1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"half_width\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].half_length),1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"half_length\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeOBBoxArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::OBBox* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"relative_pos\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].half_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"half_width\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].half_length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"half_length\" of OBBox[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

// struct ObstacleCamera {
//   /// 障碍物ID
//   Int32_t id;
//   /// 障碍物类型
//   Int8_t type;
//   /// 障碍物的状态
//   ObjStatus status;
//   /// 切入/切出的类型
//   CutInType cut_in;
//   /// 转向灯的状态
//   BlinkerType blinker;
//   /// 制动灯状态
//   bool brake_lights;
//   /// The age of the obstacle (in frames). This value starts at 1 when the
//   /// obstacle is first detected, and increments in 1 each frame.
//   Int32_t age;
//   /// 指示障碍物在哪个车道
//   Int32_t lane;
//   /// 障碍物长度 (m)
//   Float32_t length;
//   /// 障碍宽度 (m)
//   Float32_t width;
//   /// 障碍物位置x坐标 (m)
//   Float32_t x;
//   /// 障碍物位置y坐标 (m)
//   Float32_t y;
//   /// 障碍物的相对速度 (m/sec)
//   Float32_t rel_v_x;
//   /// 障碍物的相对速度 (m/sec)
//   Float32_t rel_v_y;
//   /// 障碍物的加速度 (m/sec^2)
//   Float32_t accel_x;
//   /// 障碍物的加速度 (m/sec^2)
//   Float32_t accel_y;
//   /// 障碍物的角速度 (rad/sec)
//   Float32_t yaw_rate;
//   /// 尺度的变化 (pix/sec)
//   Float32_t scale_change;
// };

Int32_t EncodeObstacleCameraArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    //Int8_t type = p[i].type;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t status = p[i].status;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"status\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t cut_in = p[i].cut_in;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(cut_in), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cut_in\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t blinker = p[i].blinker;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(blinker), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"blinker\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t brake_lights = p[i].brake_lights;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(brake_lights), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_lights\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"age\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"length\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"width\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_x\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_y\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].scale_change), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"scale_change\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}
Int32_t DecodeObstacleCameraArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleCamera* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t status = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"status\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    switch (status) {
    case (0):
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
    case (1):
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_STANDING;
      break;
    case (2):
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
      break;
    case (3):
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_MOVING;
      break;
    case (4):
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_ONCOMING;
      break;
    case (5):
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_PARKED;
      break;
    default:
      p[i].status = ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
    }

    Int8_t cut_in = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(cut_in), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cut_in\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    switch (cut_in) {
    case (0):
      p[i].cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
      break;
    case (1):
      p[i].cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_IN_HOST_LANE;
      break;
    case (2):
      p[i].cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_OUT_HOST_LANE;
      break;
    case (3):
      p[i].cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_IN;
      break;
    case (4):
      p[i].cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_CUT_OUT;
      break;
    default:
      p[i].cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
      break;
    }

    Int8_t blinker = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(blinker), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"blinker\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    switch (blinker) {
    case (0):
      p[i].blinker = ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
      break;
    case (1):
      p[i].blinker = ad_msg::ObstacleCamera::BLINKER_OFF;
      break;
    case (2):
      p[i].blinker = ad_msg::ObstacleCamera::BLINKER_LEFT;
      break;
    case (3):
      p[i].blinker = ad_msg::ObstacleCamera::BLINKER_RIGHT;
      break;
    case (4):
      p[i].blinker = ad_msg::ObstacleCamera::BLINKER_BOTH;
      break;
    default:
      p[i].blinker = ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
      break;
    }
    
    Int8_t brake_lights = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(brake_lights), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_lights\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].brake_lights = brake_lights;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"age\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"length\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"width\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_x\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_y\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_x\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_y\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].scale_change), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"scale_change\" of ObstacleCamera["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

// struct ObstacleCameraList {
//   /// 障碍物的最大数量
//   enum { MAX_OBSTACLE_NUM = 16 };
//   /// 消息头
//   MsgHead msg_head;
//   /// 障碍物的数量
//   Int32_t obstacle_num;
//   /// 障碍物列表
//   ObstacleCamera obstacles[MAX_OBSTACLE_NUM];
// };

Int32_t EncodeObstacleCameraListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleCameraList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cam_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cam_type\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacle_num\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t obstacle_num = p[i].obstacle_num;

    thislen = EncodeObstacleCameraArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacles\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }

  return pos;
}

Int32_t DecodeObstacleCameraListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleCameraList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].cam_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"cam_type\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int32_t obstacle_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacle_num\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].obstacle_num = obstacle_num;

    thislen = DecodeObstacleCameraArray(buf, offset + pos, maxlen - pos,
                                        &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacles\" of ObstacleCameraList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}
// struct ObstacleRadar {
//   /// 障碍物ID
//   Int32_t id;
//   /// 障碍物的类型
//   Int8_t type;
//   /// 障碍物跟踪的状态
//   TrackStatus track_status;
//   /// 障碍物融合的类型
//   MergedStatus merged_status;
//   /// 障碍物是否是迎面驶来的
//   bool oncomming;
//   /// 此障碍物是否是bridge
//   bool bridge;
//   /// 障碍物距离传感器的距离（极坐标）(m)
//   Float32_t range;
//   /// 障碍物与传感器中心纵轴之间的角度（极坐标）(rad)
//   Float32_t angle;
//   /// 障碍物距离传感器的距离的变化率（极坐标）(m/sec)
//   Float32_t range_rate;
//   /// 障碍物距离传感器的距离变化率的变化率（极坐标）(m/sec^2)
//   Float32_t range_acceleration;
//   /// 障碍物的侧向速度（左正右负）(m/sec)
//   Float32_t lateral_rate;
//   /// 障碍物长度 (m)
//   Float32_t length;
//   /// 障碍宽度 (m)
//   Float32_t width;
//   /// 障碍物位置x坐标 (m)
//   Float32_t x;
//   /// 障碍物位置y坐标 (m)
//   Float32_t y;
//   /// 障碍物的x向相对速度 (m/sec)
//   Float32_t rel_v_x;
//   /// 障碍物的y向相对速度 (m/sec)
//   Float32_t rel_v_y;
//   /// 障碍物的x向加速度 (m/sec^2)
//   Float32_t accel_x;
//   /// 障碍物的y向加速度 (m/sec^2)
//   Float32_t accel_y;
//   /// 障碍物的角速度 (rad/sec)
//   Float32_t yaw_rate;
// }

Int32_t EncodeObstacleRadarArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleRadar* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    // Int8_t type = p[i].type;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t track_status = p[i].track_status;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(track_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"status\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t merged_status = p[i].merged_status;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(merged_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cut_in\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t oncomming = p[i].oncomming;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(oncomming), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"oncomming\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t bridge = p[i].bridge;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(bridge), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"bridge\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].range), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"range\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].range_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"range_rate\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].range_acceleration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"range_acceleration\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lateral_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lateral_rate\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"length\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"width\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_x\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_y\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeObstacleRadarArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleRadar* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t track_status = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(track_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"status\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    switch (track_status) {
    case (0):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET;
      break;
    case (1):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_TARGET;
      break;
    case (2):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_UPDATED_TARGET;
      break;
    case (3):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_UPDATED_TARGET;
      break;
    case (4):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_COASTED_TARGET;
      break;
    case (5):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_MERGED_TARGET;
      break;
    case (6):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_INVALID_COASTED_TARGET;
      break;
    case (7):
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NEW_COASTED_TARGET;
      break;
    default:
      p[i].track_status = ad_msg::ObstacleRadar::TRACK_STATUS_NO_TARGET;
      break;
    }

    Int8_t merged_status = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(merged_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"cut_in\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    switch (merged_status) {
    case (0):
      p[i].merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET;
      break;
    case (1):
      p[i].merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_MR_TARGET;
      break;
    case (2):
      p[i].merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_LR_TARGET;
      break;
    case (3):
      p[i].merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_MR_LR_TARGET;
      break;
    default:
      p[i].merged_status = ad_msg::ObstacleRadar::MERGED_STATUS_NO_TARGET;
      break;
    }

    Int8_t oncomming = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(oncomming), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"oncomming\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].oncomming = oncomming;
    
    Int8_t bridge = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(bridge), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"brake_lights\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].bridge = bridge;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].range), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"range\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"angle\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].range_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"range_rate\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].range_acceleration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"range_acceleration\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lateral_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lateral_rate\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"length\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"width\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_x\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_y\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_x\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_y\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of ObstacleRadar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

// struct ObstacleRadarList {
//   /// 最大障碍物的数量
//   enum { MAX_OBSTACLE_NUM = 64 };

//   /// 消息头
//   MsgHead msg_head;
//   /// 障碍物的数量
//   Int32_t obstacle_num;
//   /// 障碍物列表
//   ObstacleRadar obstacles[MAX_OBSTACLE_NUM];
// };

Int32_t EncodeObstacleRadarListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleRadarList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].radar_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"radar_type\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacle_num\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t obstacle_num = p[i].obstacle_num;

    thislen = EncodeObstacleRadarArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacles\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeObstacleRadarListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleRadarList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].radar_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"radar_type\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int32_t obstacle_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacle_num\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].obstacle_num = obstacle_num;

    thislen = DecodeObstacleRadarArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacles\" of ObstacleRadarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}


///// 障碍物ID
//Int32_t id;
///// 障碍物的类型
//Int8_t type;
///// Number of scans this object has been tracked for.
//Int32_t age;
///// Number of scans this object has currently been predicted for without measurement update
//Int32_t prediction_age;
///// 障碍物位置x坐标 (m)
//Float32_t x;
///// 障碍物位置y坐标 (m)
//Float32_t y;
///// 障碍物包围盒
//OBBox obb;
///// 障碍物的x向速度 (m/sec)
//Float32_t v_x;
///// 障碍物的y向速度 (m/sec)
//Float32_t v_y;
///// 障碍物的x向加速度 (m/sec^2)
//Float32_t accel_x;
///// 障碍物的y向加速度 (m/sec^2)
//Float32_t accel_y;
///// 障碍物的角速度 (rad/sec)
//Float32_t yaw_rate;
Int32_t EncodeObstacleLidarArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleLidar* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"age\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].prediction_age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"prediction_age\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeOBBoxArray(buf, offset + pos, maxlen - pos,
                               &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obb\" of ObstacleLidar[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_x\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"accel_y\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeObstacleLidarArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleLidar* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"id\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"type\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"age\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].prediction_age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"prediction_age\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeOBBoxArray(buf, offset + pos, maxlen - pos,
                               &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obb\" of ObstacleLidar[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_x\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_y\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_x\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].accel_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"accel_y\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of ObstacleLidar["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


///// 消息头
//MsgHead msg_head;
///// 雷达类型
//Int32_t lidar_type;
///// 障碍物的数量
//Int32_t obstacle_num;
///// 障碍物列表
//ObstacleLidar obstacles[MAX_OBSTACLE_NUM];
Int32_t EncodeObstacleLidarListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleLidarList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lidar_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lidar_type\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacle_num\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t obstacle_num = p[i].obstacle_num;

    thislen = EncodeObstacleLidarArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacles\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeObstacleLidarListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleLidarList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lidar_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lidar_type\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacle_num\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t obstacle_num = p[i].obstacle_num;

    thislen = DecodeObstacleLidarArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacles\" of ObstacleLidarList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


// struct Obstacle {
//   /// 障碍物ID
//   Int32_t id;
//   /// 障碍物位置及包围盒
//   OBBox obb;
//   /// 障碍物高度
//   Float32_t height;
//   /// 离地面高度
//   Float32_t height_to_ground;
//   /// 障碍物类型
//   Int8_t type;
//   /// 是否是动态障碍物
//   Int8_t dynamic;
//   /// 障碍物存在的置信度
//   Int8_t confidence;
//   /// 感知类别
//   Int8_t perception_type;
//   /// 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
//   Float32_t v_x;
//   /// 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
//   Float32_t v_y;
//   /// 障碍物绝对速度，单位：米/秒
//   Float32_t v;
//   /// 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
//   Float32_t a_x;
//   /// 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
//   Float32_t a_y;
//   /// 障碍物绝对加速度，单位：米^2/秒
//   Float32_t a;
//   /// 预测的轨迹的数量
//   Int8_t pred_path_num;
//   /// 预测的轨迹中点的数量
//   Int8_t pred_path_point_num[MAX_PRED_PATH_NUM];
//   /// 对动态障碍物预测的轨迹
//   struct {
//     /// 形点坐标x
//     Float32_t x;
//     /// 形点坐标y
//     Float32_t y;
//     /// 障碍物在此点的航向
//     Float32_t heading;
//     /// 障碍物在此点沿着轨迹的长度
//     Float32_t s;
//   } pred_path[MAX_PRED_PATH_NUM][MAX_PRED_PATH_POINT_NUM];
//   /// 障碍物跟踪的轨迹(障碍物历史轨迹)中点的数量
//   Int8_t tracked_path_point_num;
//   /// 障碍物跟踪轨迹轨迹
//   struct {
//     /// 形点坐标x
//     Float32_t x;
//     /// 形点坐标y
//     Float32_t y;
//   } tracked_path[MAX_TRACKED_PATH_POINT_NUM];
// };
Int32_t EncodeObstacleArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::Obstacle* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeOBBoxArray(buf, offset + pos, maxlen - pos,
                               &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obb\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].height), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"height\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].height_to_ground), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"height_to_ground\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dynamic), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"dynamic\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].confidence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"confidence\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].perception_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"perception_type\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a_x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a_y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"aproj_on_major_ref_line.valid\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"proj_on_major_ref_line.x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"proj_on_major_ref_line.y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"proj_on_major_ref_line.heading\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"proj_on_major_ref_line.curvature\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"proj_on_major_ref_line.s\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"proj_on_major_ref_line.l\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pred_path_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"pred_path_num\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int8_t pred_path_num = p[i].pred_path_num;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pred_path_point_num[0]), pred_path_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"pred_path_num\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    for (Int8_t j = 0; j < pred_path_num; ++j) {
      for (Int8_t k = 0; k < p[i].pred_path_point_num[j]; ++k)  {
        thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].x), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to encode \"x\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }

        thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].y), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to encode \"y\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }

        thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].heading), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to encode \"heading\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }
        
        thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].s), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to encode \"s\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }
      }
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tracked_path_point_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"tracked_path_point_num\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t tracked_path_point_num = p[i].tracked_path_point_num;
    for (Int8_t j = 0; j < tracked_path_point_num; ++j) {
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tracked_path[j].x), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"x \" of tracked_path of Obstacle[" << i << "].";
      } else {
        pos += thislen;
      }
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tracked_path[j].y), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"y \" of tracked_path of Obstacle[" << i << "].";
      } else {
        pos += thislen;
      }
    }

  }
  return pos;
}

Int32_t DecodeObstacleArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::Obstacle* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"id\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = DecodeOBBoxArray(buf, offset + pos, maxlen - pos,
                               &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"obb\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].height), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"height\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].height_to_ground), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"height_to_ground\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"type\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dynamic), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"dynamic\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].confidence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"confidence\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].perception_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"perception_type\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"v_x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"v_y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"v\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"a_x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"a_y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"a\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.valid\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.x\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.y\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.heading\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.curvature\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.s\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].proj_on_major_ref_line.l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"proj_on_major_ref_line.l\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    Int8_t pred_path_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(pred_path_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"pred_path_num\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].pred_path_num = pred_path_num;
    
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].pred_path_point_num[0]), pred_path_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"pred_path_num\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    for (Int8_t j = 0; j < pred_path_num; ++j) {
      for (Int8_t k = 0; k < p[i].pred_path_point_num[j]; ++k)  {
        thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].x), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to Decode \"x\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }

        thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].y), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to Decode \"y\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }

        thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].heading), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to Decode \"heading\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }
        
        thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                           &(p[i].pred_path[j][k].s), 1);
        if (thislen < 0) {
          LOG_ERR << "Failed to Decode \"s\" of pred_path of Obstacle[" << i << "].";
          return (thislen);
        } else {
          pos += thislen;
        }
      }
    }
    Int8_t tracked_path_point_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(tracked_path_point_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to Decode \"tracked_path_point_num\" of Obstacle[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].tracked_path_point_num = tracked_path_point_num;
    
    for (Int8_t j = 0; j < tracked_path_point_num; ++j) {
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tracked_path[j].x), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to Decode \"x \" of tracked_path of Obstacle[" << i << "].";
      } else {
        pos += thislen;
      }
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].tracked_path[j].y), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to Decode \"y \" of tracked_path of Obstacle[" << i << "].";
      } else {
        pos += thislen;
      }
    }

  }
  return pos;
}

// struct ObstacleList {
//   /**
//    * @enum MAX_OBSTACLE_NUM
//    * @brief 列表中最大障碍物的数量
//    */
//   enum { MAX_OBSTACLE_NUM = 64 };

//   /// 消息头
//   MsgHead msg_head;
//   /// 障碍物的数量
//   Int32_t obstacle_num;
//   /// 障碍物列表，障碍物的位置使用车身坐标系表达
//   Obstacle obstacles[MAX_OBSTACLE_NUM];
// };

Int32_t EncodeObstacleListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacle_num\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t obstacle_num = p[i].obstacle_num;
    thislen = EncodeObstacleArray(buf, offset + pos, maxlen - pos,
                                  &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacles\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeObstacleListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    
    Int32_t obstacle_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacle_num\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].obstacle_num = obstacle_num;

    thislen = DecodeObstacleArray(buf, offset + pos, maxlen - pos,
                                  &(p[i].obstacles[0]),obstacle_num );
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacles\" of ObstacleList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}
// struct ObstacleTrackedInfo {
//   Int8_t sensor_id;
//   Int8_t obj_type;
//   Float32_t x;
//   Float32_t y;
//   Float32_t width;
//   Float32_t length;
//   OBBox obb;
//   Float32_t v_x;
//   Float32_t v_y;
//   Int32_t track_status;
//   Int32_t age;
//   Int32_t duration;
//   Int32_t confidence;
// };
Int32_t EncodeObstacleTrackedInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleTrackedInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obj_type\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"x\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"y\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"width\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"length\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeOBBoxArray(buf, offset + pos, maxlen - pos,
                               &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obb\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_x\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v_y\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].track_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"track_status\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"age\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].duration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"duration\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].confidence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"confidence\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}

Int32_t DecodeObstacleTrackedInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleTrackedInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obj_type\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"x\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"width\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].length), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"length\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeOBBoxArray(buf, offset + pos, maxlen - pos,
                               &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"sensor_id\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_x\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v_y\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].track_status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"track_status\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].age), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"age\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].duration), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"duration\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].confidence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"confidence\" of ObstacleTrackedInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

  }
  return pos;
}

// struct ObstacleTrackedInfoList {

//   MsgHead msg_head;
//   Int32_t obstacle_num;
//   ObstacleTrackedInfo obstacles[MAX_OBSTACLE_NUM];
// };
Int32_t EncodeObstacleTrackedInfoListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ObstacleTrackedInfoList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ObstacleTrackedInfoList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacle_num\" of ObstacleTrackedInfoList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t obstacle_num = p[i].obstacle_num;
    thislen = EncodeObstacleTrackedInfoArray(buf, offset + pos, maxlen - pos,
                                             &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacles\" of ObstacleTrackedInfoList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeObstacleTrackedInfoListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ObstacleTrackedInfoList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; i++) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ObstacleTrackedInfoList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t obstacle_num = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(obstacle_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"y\" of ObstacleTrackedInfoList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].obstacle_num = obstacle_num;

    thislen = DecodeObstacleTrackedInfoArray(buf, offset + pos, maxlen - pos,
                                             &(p[i].obstacles[0]), obstacle_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacles\" of ObstacleTrackedInfoList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix
