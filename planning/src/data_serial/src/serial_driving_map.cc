//
#include "serial_driving_map.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_vec2d.h"
#include "serial_line_segment2d.h"
#include "serial_obbox2d.h"
#include "serial_path.h"
#include "serial_msg_common.h"
#include "serial_msg_scene_story.h"

namespace phoenix {
namespace data_serial {


// LaneInfo
// 车道id
// ID lane_id;
// 车道索引
// Int32_t lane_index;
// 车道线质量
// Int32_t quality;
// 车道中心线
// common::StaticVector<common::Vec2d,
//     common::Path::kMaxPathPointNum> central_curve;
// 车道左边界
// Boundary left_boundary;
// 车道右边界
// Boundary right_boundary;
Int32_t EncodeLaneInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::LaneInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
#if (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane_id.id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane_id.id\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#endif
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane_index\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"quality\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t central_curve_size = p[i].central_curve.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &central_curve_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"central_curve\" of LaneInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                               p[i].central_curve.data(),
                               central_curve_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"central_curve\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // struct Boundary {
    //   struct BoundaryAssociation {
    //     /// 在参考线上的路径长
    //     map_var_t s;
    //     /// 宽度（车道半宽）
    //     map_var_t width;
    //     /// 边线类型
    //     Int32_t type;
    //     /// 边线型点
    //     common::Vec2d point;
    //   };
    //   /// 车道边线
    //   common::StaticVector<BoundaryAssociation,
    //   common::Path::kMaxPathPointNum> curve;
    // };
    Int32_t boundary_curve_size = p[i].left_boundary.curve.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &boundary_curve_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"left_boundary.curve\" of LaneInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    for (Int32_t j = 0; j < boundary_curve_size; ++j) {
      const driv_map::LaneInfo::Boundary::BoundaryAssociation& curve_point =
          p[i].left_boundary.curve[j];

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.s), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.type), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                                 &(curve_point.point), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    boundary_curve_size = p[i].right_boundary.curve.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &boundary_curve_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"left_boundary.curve\" of LaneInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    for (Int32_t j = 0; j < boundary_curve_size; ++j) {
      const driv_map::LaneInfo::Boundary::BoundaryAssociation& curve_point =
          p[i].right_boundary.curve[j];

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.s), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.type), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = EncodeVec2dArray(buf, offset + pos, maxlen - pos,
                                 &(curve_point.point), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}

Int32_t DecodeLaneInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::LaneInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
#if (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane_id.id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane_id.id\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
#endif
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lane_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane_index\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].quality), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"quality\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    Int32_t central_curve_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &central_curve_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"central_curve\" of LaneInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].central_curve.Resize(central_curve_size);
    thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                               p[i].central_curve.data(),
                               central_curve_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"central_curve\" of LaneInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // struct Boundary {
    //   struct BoundaryAssociation {
    //     /// 在参考线上的路径长
    //     map_var_t s;
    //     /// 宽度（车道半宽）
    //     map_var_t width;
    //     /// 边线类型
    //     Int32_t type;
    //     /// 边线型点
    //     common::Vec2d point;
    //   };
    //   /// 车道边线
    //   common::StaticVector<BoundaryAssociation,
    //   common::Path::kMaxPathPointNum> curve;
    // };
    Int32_t boundary_curve_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &boundary_curve_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"left_boundary.curve\" of LaneInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].left_boundary.curve.Resize(boundary_curve_size);
    for (Int32_t j = 0; j < boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& curve_point =
          p[i].left_boundary.curve[j];

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.s), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.type), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                                 &(curve_point.point), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }

    boundary_curve_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &boundary_curve_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"left_boundary.curve\" of LaneInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].right_boundary.curve.Resize(boundary_curve_size);
    for (Int32_t j = 0; j < boundary_curve_size; ++j) {
      driv_map::LaneInfo::Boundary::BoundaryAssociation& curve_point =
          p[i].right_boundary.curve[j];

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.s), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.width), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(curve_point.type), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }

      thislen = DecodeVec2dArray(buf, offset + pos, maxlen - pos,
                                 &(curve_point.point), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"left_boundary\" of LaneInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}


Int32_t EncodeMapTrafficLightArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::MapTrafficLight* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    /// TODO: 暂时不对ID进行编码/解码

    thislen = EncodeLineSegment2dArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].stop_line), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"stop_line\" of MapTrafficLight[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeMapTrafficLightArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::MapTrafficLight* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    /// TODO: 暂时不对ID进行编码/解码

    thislen = DecodeLineSegment2dArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].stop_line), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"stop_line\" of MapTrafficLight[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


// struct MapInfo {
//   /// Lane table
//   common::StaticVector<LaneInfo, MAX_LANE_NUM> lane_table;
// };
Int32_t EncodeMapInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::MapInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    // Lanes
    Int32_t lane_table_size = p[i].lane_table.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &lane_table_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"lane_table\" of MapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeLaneInfoArray(buf, offset + pos, maxlen - pos,
                                  p[i].lane_table.data(),
                                  lane_table_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"lane_table\" of MapInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Traffic lights
    Int32_t traffic_light_table_size = p[i].map_traffic_light_table.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &traffic_light_table_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"traffic_light_table\" of MapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeMapTrafficLightArray(buf, offset + pos, maxlen - pos,
                                         p[i].map_traffic_light_table.data(),
                                         traffic_light_table_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"traffic_light_table\" of MapInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeMapInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::MapInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    // lanes
    Int32_t lane_table_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &lane_table_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"lane_table\" of MapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].lane_table.Resize(lane_table_size);
    thislen = DecodeLaneInfoArray(buf, offset + pos, maxlen - pos,
                                  p[i].lane_table.data(),
                                  lane_table_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"lane_table\" of MapInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // traffic lights
    Int32_t traffic_light_table_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &traffic_light_table_size, 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"traffic_light_table_size\" of MapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].map_traffic_light_table.Resize(traffic_light_table_size);
    thislen = DecodeMapTrafficLightArray(buf, offset + pos, maxlen - pos,
                                         p[i].map_traffic_light_table.data(),
                                         traffic_light_table_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"traffic_light_table\" of MapInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}



// struct CollisionRiskTestResult {
//   /// 障碍物在列表中的索引
//   Int32_t obj_list_index;
//   /// 此障碍物对应的风险值
//   Int32_t risk_value;
//   /// 与障碍物之间的距离(动态距离)
//   map_var_t dynamic_distance;
//   /// 与障碍物之间的距离(静态距离)
//   map_var_t static_distance;
//   /// 障碍物在参考线上的路径长
//   map_var_t obj_s_ref;
//   /// 障碍物在与参考线上之间的横向偏差
//   map_var_t obj_l_ref;
//   /// 与此障碍物有碰撞风险的目标物体在轨迹上的路径长
//   map_var_t collision_s;
//   /// 保存了障碍物采样点相关的信息
//   /// （意味着障碍物在其这个采样点的位置，存在碰撞风险）
//   common::TrajectoryPoint obj_traj_point;
// };
Int32_t EncodeCollisionRiskTestResultObjInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::CollisionTestResult::ObjInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_list_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obj_list_index\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].risk_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"risk_value\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dynamic_distance), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"dynamic_distance\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].static_distance), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"static_distance\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_s_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obj_s_ref\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_l_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obj_l_ref\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].collision_s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"collision_s\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeTrajectoryPointArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].obj_traj_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obj_traj_point\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeCollisionRiskTestResultObjInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::CollisionTestResult::ObjInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_list_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obj_list_index\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].risk_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"risk_value\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dynamic_distance), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"dynamic_distance\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].static_distance), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"static_distance\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_s_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obj_s_ref\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].obj_l_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obj_l_ref\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].collision_s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"collision_s\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeTrajectoryPointArray(buf, offset + pos, maxlen - pos,
                                         &(p[i].obj_traj_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obj_traj_point\" of "
                 "CollisionTestResult["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}



// struct ReferenceLineInfo {
//   /// 参考线形点
//   common::StaticVector<common::PathPoint,
//       common::Path::kMaxPathPointNum> curve;
//   /// 平滑后的参考线形点
//   common::StaticVector<common::PathPoint,
//       common::Path::kMaxPathPointNum> smooth_curve;
// };
Int32_t EncodeReferenceLineInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::DrivingMapInfo::ReferenceLineInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int32_t curve_size = p[i].curve.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(curve_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].curve.data(), curve_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t smooth_curve_size = p[i].smooth_curve.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(smooth_curve_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"smooth_curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].smooth_curve.data(), smooth_curve_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"smooth_curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeReferenceLineInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::DrivingMapInfo::ReferenceLineInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int32_t curve_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(curve_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].curve.Resize(curve_size);
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].curve.data(), curve_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int32_t smooth_curve_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(smooth_curve_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"smooth_curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].smooth_curve.Resize(smooth_curve_size);
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   p[i].smooth_curve.data(), smooth_curve_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"smooth_curve\" of "
                 "DrivingMapInfo::ReferenceLineInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}



/// 障碍物信息
// struct ObstacleInfo {
//   // 障碍物ID
//   Int32_t id;
//   // 障碍物位置及包围盒
//   common::OBBox2d obb;
//   // Heading
//   map_var_t heading;
//   // 障碍物类型
//   Int8_t type;
//   // 是否是动态障碍物
//   Int8_t dynamic;
//   // 是否应当忽略此障碍物（例如道路外的静态障碍物（人、车等例外），
//   // 车辆正后方的动态障碍物，车辆后方的静态障碍物等）
//   bool ignore;
//   // 障碍物速度
//   map_var_t v;
//   // 与参考线之间的关联信息
//   map_var_t s_ref;
//   map_var_t l_ref;
//   // 保存对动态障碍物预测的轨迹
//   common::StaticVector<common::StaticVector<common::TrajectoryPoint,
//       MAX_OBJ_PRED_TRAJ_POINT_NUM>, MAX_OBJ_PRED_TRAJ_NUM> pred_trajectory;
// };
Int32_t EncodeObstacleInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::DrivingMapInfo::ObstacleInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"id\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeOBBox2dArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obb\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"heading\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dynamic), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"dynamic\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t ignore = p[i].ignore;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(ignore), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ignore\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t uncertain = p[i].uncertain;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(uncertain), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"uncertain\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"v\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].s_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"s_ref\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].l_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"l_ref\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    Int32_t pred_trajectory_size = p[i].pred_trajectory.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(pred_trajectory_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"pred_trajectory\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    for (Int32_t j = 0; j < pred_trajectory_size; ++j) {
      Int32_t points_size = p[i].pred_trajectory[j].Size();
      thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(points_size), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode size of \"pred_trajectory[" << j
                << "]\" of DrivingMapInfo::ObstacleInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
      thislen = EncodeTrajectoryPointArray(buf, offset + pos, maxlen - pos,
                                           p[i].pred_trajectory[j].data(),
                                           points_size);
      if (thislen < 0) {
        LOG_ERR << "Failed to encode \"pred_trajectory[" << j
                << "]\" of DrivingMapInfo::ObstacleInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}

Int32_t DecodeObstacleInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::DrivingMapInfo::ObstacleInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"id\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeOBBox2dArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].obb), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obb\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"heading\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"type\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dynamic), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"dynamic\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    Int8_t ignore = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(ignore), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"ignore\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].ignore = ignore;

    Int8_t uncertain = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(uncertain), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"uncertain\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].uncertain = uncertain;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"v\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].s_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"s_ref\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].l_ref), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"l_ref\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    Int32_t pred_trajectory_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(pred_trajectory_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"pred_trajectory\" of "
                 "DrivingMapInfo::ObstacleInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].pred_trajectory.Resize(pred_trajectory_size);
    for (Int32_t j = 0; j < pred_trajectory_size; ++j) {
      Int32_t points_size = 0;
      thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                         &(points_size), 1);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode size of \"pred_trajectory[" << j
                << "]\" of DrivingMapInfo::ObstacleInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
      p[i].pred_trajectory[j].Resize(points_size);
      thislen = DecodeTrajectoryPointArray(buf, offset + pos, maxlen - pos,
                                           p[i].pred_trajectory[j].data(),
                                           points_size);
      if (thislen < 0) {
        LOG_ERR << "Failed to decode \"pred_trajectory[" << j
                << "]\" of DrivingMapInfo::ObstacleInfo["
                << i << "].";
        return (thislen);
      } else {
        pos += thislen;
      }
    }
  }

  return pos;
}

Int32_t EncodeRoadBoundaryArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::RoadBoundary* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].ref_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"ref_point\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].left_boundary_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"left_boundary_point\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].right_boundary_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"right_boundary_point\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].left_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"left_width\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].right_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"type\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeRoadBoundaryArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::RoadBoundary* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].ref_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"ref_point\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].left_boundary_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"left_boundary_point\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].right_boundary_point), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"right_boundary_point\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].left_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"left_width\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].right_width), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"type\" of "
                 "RoadBoundary["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

// struct DrivingMapInfo {
//   /// 车道上距离当前车辆位置最近的点
//   common::PathPoint nearest_point_to_veh_on_lane;
//   /// 内部地图信息
//   MapInfo map;
//   /// 当前参考线的索引
//   Int32_t current_reference_line_index;
//   /// 所有参考线的集合
//   common::StaticVector<ReferenceLineInfo,
//       MAX_REFERENCE_LINE_NUM> reference_lines;
//   /// 道路左边界采样点
//   common::StaticVector<common::PathPoint,
//   common::Path::kMaxPathPointNum> left_road_boundary;
//   /// 道路右边界采样点
//   common::StaticVector<common::PathPoint,
//   common::Path::kMaxPathPointNum> right_road_boundary;
//   /// 障碍物列表
//   common::StaticVector<ObstacleInfo, MAX_OBJECT_NUM_IN_OBJ_SPACE> obstacle_list;
//   /// 风险区域分析结果(有风险的障碍物信息)
//   common::StaticVector<CollisionRiskTestResult,
//       MAX_OBJECT_NUM_IN_OBJ_SPACE> risky_obj_list;
// };
Int32_t EncodeDrivingMapInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const driv_map::DrivingMapInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Driving_map_type
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].driving_map_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"driving_map_type\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Nearest_point_to_veh_on_lane
    thislen = EncodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].nearest_point_to_veh_on_lane), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"nearest_point_to_veh_on_lane\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Map
    thislen = EncodeMapInfoArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].map), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"map\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Current_reference_line_index
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].current_reference_line_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"current_reference_line_index\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Reference lines
    Int32_t reference_lines_size = p[i].reference_lines.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(reference_lines_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"reference_lines\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeReferenceLineInfoArray(buf, offset + pos, maxlen - pos,
                                           p[i].reference_lines.data(),
                                           reference_lines_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"reference_lines\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Road boundary
    Int32_t road_boundary_size = p[i].road_boundary.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(road_boundary_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"road_boundary\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeRoadBoundaryArray(buf, offset + pos, maxlen - pos,
                                      (p[i].road_boundary.data()), road_boundary_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"road_boundary\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Road_boundary_obj_list
    Int32_t road_boundary_obj_list_size = p[i].road_boundary_obj_list.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(road_boundary_obj_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"road_boundary_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeOBBox2dArray(buf, offset + pos, maxlen - pos,
                                      (p[i].road_boundary_obj_list.data()), 
                                       road_boundary_obj_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"road_boundary_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Obstacle list
    Int32_t obstacle_list_size = p[i].obstacle_list.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(obstacle_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"obstacle_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeObstacleInfoArray(buf, offset + pos, maxlen - pos,
                                      p[i].obstacle_list.data(),
                                      obstacle_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"obstacle_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Risky object list
    Int32_t risky_obj_list_size = p[i].risky_obj_list.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(risky_obj_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"risky_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeCollisionRiskTestResultObjInfoArray(
          buf, offset + pos, maxlen - pos,
          p[i].risky_obj_list.data(),
          risky_obj_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"risky_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Uncertain_list
    Int32_t uncertain_list_size = p[i].uncertain_list.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(uncertain_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"uncertain_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = EncodeCollisionRiskTestResultObjInfoArray(
          buf, offset + pos, maxlen - pos,
          p[i].uncertain_list.data(),
          uncertain_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"uncertain_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Following_target
    Int8_t following_target_valid = p[i].following_target.valid;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(following_target_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"following_target.valid\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].following_target.obj_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"following_target.obj_x\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].following_target.obj_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"following_target.obj_y\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Scene_storys
    Int32_t scene_storys_size = p[i].scene_storys.Size();
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(scene_storys_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"scene_storys\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeSceneStoryArray(buf, offset + pos, maxlen - pos,
                                      (p[i].scene_storys.data()), 
                                       scene_storys_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode size of \"road_boundary_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeDrivingMapInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    driv_map::DrivingMapInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].driving_map_type), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"driving_map_type\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodePathPointArray(buf, offset + pos, maxlen - pos,
                                   &(p[i].nearest_point_to_veh_on_lane), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"nearest_point_to_veh_on_lane\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeMapInfoArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].map), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"map\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].current_reference_line_index), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"current_reference_line_index\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Reference lines
    Int32_t reference_lines_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(reference_lines_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"reference_lines\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].reference_lines.Resize(reference_lines_size);
    thislen = DecodeReferenceLineInfoArray(buf, offset + pos, maxlen - pos,
                                           p[i].reference_lines.data(),
                                           reference_lines_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"reference_lines\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }


    // road boundary
    Int32_t road_boundary_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(road_boundary_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"road_boundary\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].road_boundary.Resize(road_boundary_size);
    thislen = DecodeRoadBoundaryArray(buf, offset + pos, maxlen - pos,
                                      p[i].road_boundary.data(),
                                      road_boundary_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"road_boundary\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Road_boundary_obj_list 
    Int32_t road_boundary_obj_list_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(road_boundary_obj_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"road_boundary_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].road_boundary_obj_list.Resize(road_boundary_obj_list_size);
    thislen = DecodeOBBox2dArray(buf, offset + pos, maxlen - pos,
                                      p[i].road_boundary_obj_list.data(),
                                      road_boundary_obj_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"road_boundary_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Obstacle list
    Int32_t obstacle_list_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(obstacle_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"obstacle_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].obstacle_list.Resize(obstacle_list_size);
    thislen = DecodeObstacleInfoArray(buf, offset + pos, maxlen - pos,
                                      p[i].obstacle_list.data(),
                                      obstacle_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"obstacle_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Risky object list
    Int32_t risky_obj_list_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(risky_obj_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"risky_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].risky_obj_list.Resize(risky_obj_list_size);
    thislen = DecodeCollisionRiskTestResultObjInfoArray(
          buf, offset + pos, maxlen - pos,
          p[i].risky_obj_list.data(),
          risky_obj_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"risky_obj_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // uncertain_list
    Int32_t uncertain_list_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(uncertain_list_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode size of \"uncertain_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].uncertain_list.Resize(uncertain_list_size);
    thislen = DecodeCollisionRiskTestResultObjInfoArray(
          buf, offset + pos, maxlen - pos,
          p[i].uncertain_list.data(),
          uncertain_list_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"uncertain_list\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // 跟车目标信息
    Int8_t following_target_valid = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(following_target_valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"following_target.valid\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].following_target.valid = following_target_valid;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].following_target.obj_x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"following_target.obj_x\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].following_target.obj_y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"following_target.obj_y\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    // Scene Story
    Int32_t scene_storys_size = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(scene_storys_size), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"scene_storys_size\" of "
                 "DrivingMapInfo["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    p[i].scene_storys.Resize(scene_storys_size);
    thislen = DecodeSceneStoryArray(buf, offset + pos, maxlen - pos,
                                    p[i].scene_storys.data(),scene_storys_size);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"scene_storys\" of "
                 "DrivingMapInfo["
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


