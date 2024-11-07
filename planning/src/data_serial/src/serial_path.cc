//
#include "serial_path.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_vec2d.h"


namespace phoenix {
namespace data_serial {


/// PathPoint
// 二维位置（单位：米）
// Vec2d point;
// 航向角（单位：弧度）
// geo_var_t heading;
// 曲率（单位：1/米）
// geo_var_t curvature;
// 沿着路径的弧长（单位：米）
// geo_var_t s;
// 侧向距离（单位：米），左正右负
// geo_var_t l;
Int32_t EncodePathPointArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const common::PathPoint* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"point\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curvature\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"s\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"l\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodePathPointArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    common::PathPoint* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               &(p[i].point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"point\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"heading\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curvature\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"s\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].l), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"l\" of PathPoint[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}



// struct TrajectoryPoint {
//   /// 点的空间信息，包含位置、航向角、曲率、弧长等。
//   PathPoint path_point;
//   /// 速度（单位：m/s）
//   geo_var_t v;
//   /// 加速度（单位：m/s^2）
//   geo_var_t a;
//   /// 横摆角速度（单位：rad/s）
//   geo_var_t yaw_rate;
//   /// 相对于曲线起点已行驶的相对时间（单位：s）
//   geo_var_t relative_time;
// };
Int32_t EncodeTrajectoryPointArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const common::TrajectoryPoint* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].path_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"path_point\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"a\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"yaw_rate\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].relative_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"relative_time\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeTrajectoryPointArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    common::TrajectoryPoint* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].path_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"path_point\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].a), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"a\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"yaw_rate\" of TrajectoryPoint["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].relative_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"relative_time\" of TrajectoryPoint["
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


