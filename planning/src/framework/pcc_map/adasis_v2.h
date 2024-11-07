/**
 * @file adasis_v2.h
 * @brief 定义ADASIS v2 消息结构体 for PCC
 *        参考 ADASIS v2规范 : 200v2.0.4-D2.2-ADASIS_v2_Specification.pdf
 * @author wangwh
 * @date 2023-06-06
 * 
 */

#ifndef PHOENIX_MAP_ADASIS_V2_H_
#define PHOENIX_MAP_ADASIS_V2_H_

#include "utils/macros.h"
#include "container/static_vector.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
namespace phoenix {
namespace can_dev {

struct CanFrame {
  /// 时间戳
  Int64_t time_stamp;

  /// 报文 ID,标准帧为 11 位,扩展帧为 29 位;
  Uint32_t id;
  /// 1-远程帧, 0-数据帧
  Uint8_t RTR;
  /// 1-扩展帧, 0-标准帧
  Uint8_t EXT;
  /// 报文数据长度, 取值如下:
  /// CAN 报文: 0~8;
  Int32_t data_len;
  /// 报文数据
  /// CAN: 报文数据长度为 8 字节
  Uint8_t data[8];

  void Clear() {
    time_stamp = 0;

    id = 0;
    RTR = false;
    EXT = false;
    data_len = 0;
    common::com_memset(data, 0, sizeof(data));
  }

  CanFrame() {
    Clear();
  }
};

}
}

#endif

namespace phoenix {
namespace adasisv2 {

//TODO: SEGMENT, STUB, PROFILE LONG

/**
 * @brief ADASIS v2 message type
 * 
 */
typedef enum MessageType_t {
  ADASIS_V2_MESSAGE_TYPE_INVALID = 0,
  ADASIS_V2_MESSAGE_TYPE_POSITION = 1,
  ADASIS_V2_MESSAGE_TYPE_SEGMENT = 2,
  ADASIS_V2_MESSAGE_TYPE_STUB = 3,
  ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT = 4,
  ADASIS_V2_MESSAGE_TYPE_PROFILE_LONG = 5,
  ADASIS_V2_MESSAGE_TYPE_META_DATA = 6,
  ADASIS_V2_MESSAGE_TYPE_MAX = 7
} MessageType;

/**
 * @brief ADASIS v2是道路级地图精度，Av2HP给出的自车所在的大概的车道类型
 * 
 */
enum LaneType {
  LANE_TYPE_UNKNOWN = 0,
  /// 应急车道
  LANE_TYPE_EMERGENCY_LANE = 1,
  /// 单车道线道路
  LANE_TYPE_SINGLE_LANE_ROAD = 2,
  /// 大概率是最左边车道
  LANE_TYPE_LEFT_MOST_LANE = 3,
  /// 大概率是最右边车道
  LANE_TYPE_RIGHT_MOST_LANE = 4,
  LANE_TYPE_MIDDLE_LANE = 5,
  LANE_TYPE_RESERVED = 6,
  /// N/A
  LANE_TYPE_NA = 7
};

enum InvalidType {
  INVALID_TYPE_NORMAL = 0,
  INVALID_TYPE_MESSAGE_TYPE = ADASIS_V2_MESSAGE_TYPE_INVALID,
  INVALID_TYPE_PATH_OFFSET = 8191,
  INVALID_TYPE_PATH_DISTANCE = 1023,
  INVALID_TYPE_POSITION_AGE = 2555,
  INVALID_TYPE_PROFILE_ACCURACY = 3,
  INVALID_TYPE_POSITION_CONFIDENCE = 7,
  INVALID_TYPE_LANE_TYPE = LANE_TYPE_NA,
};


/**
 * @struct PositionMessage
 * @brief describe current position of vehicle in relation to ADASISv2 Horizon Paths
 * 
 */
struct PositionMessage {
  /// 时间戳，CAN报文时间戳
  Int64_t time_stamp;

  /// [0, 7], must be 1; ADASIS消息类型
  MessageType type;
  
  /// [0, 3] ; 消息帧循环计数，Av2HR 可据此推断是否有消息丢失
  Uint8_t cyclic_counter;
  
  /// [0, 63], 0 : Unknown; 自车所在的路径ID
  Uint8_t path_index;

  /// [0, 8191], 8191 : N/A; 自车距离path起点的位置偏移量
  Uint16_t offset;

  /// [0, 3] ; 候选position 索引
  Uint8_t position_index;

  /// [ms] [0, 511] * 5, 2555 : N/A; Av2HP计算自车位置到发送该消息所耗时间，可用于航迹推定?
  Uint16_t positioin_age;

  /// TODO: [%] [-51.1, 51.2]; 自车所在车道的纵向坡度
  Float32_t slope;

  /// TODO : [0, 358.584] [°]; 自车相对path的航向角，以path方向顺时针方向增加
  Float32_t relative_heading;

  /// [0, 100]; [%]; Av2HP 的Positioning模块计算出的自车当前位置与path的匹配概率; BAIDU 不支持该信号
  Float32_t position_probability;

  /// [0, 7] 0 : highest, 6 : lowest, 7 : N/A; Av2HP Positioning模块提供的位置匹配置信度
  Uint8_t position_confidence;

  /// @see LaneType
  LaneType current_lane;

  void Clear() {
    type = ADASIS_V2_MESSAGE_TYPE_POSITION;
    cyclic_counter = INVALID_TYPE_NORMAL;
    path_index = INVALID_TYPE_NORMAL;
    offset = INVALID_TYPE_PATH_OFFSET;
    position_index = INVALID_TYPE_NORMAL;
    positioin_age = INVALID_TYPE_POSITION_AGE;
    slope = 0.0F;
    position_probability = 0.0F;
    position_confidence = INVALID_TYPE_LANE_TYPE;
    current_lane = LANE_TYPE_NA;
  }

  PositionMessage() {
    Clear();
  }
};


enum ProfileShortType {
  PROFILE_SHORT_TYPE_UNKNOWN = 0,
  /// 曲率
  PROFILE_SHORT_TYPE_CURVATURE = 1,
  PROFILE_SHORT_TYPE_ROUTE_NUMBER_TYPE = 2,
  /// 坡度 : 离散点表征
  PROFILE_SHORT_TYPE_SLOPE_STEP = 3,
  /// 坡度 : 线性插值表征
  PROFILE_SHORT_TYPE_SLOPE_LINEAR = 4,
  PROFILE_SHORT_TYPE_ROAD_ACCESSIBILITY = 5,
  PROFILE_SHORT_TYPE_ROAD_CONDITION = 6,
  PROFILE_SHORT_TYPE_VARIABLE_SPEED_SIGN_POSITION = 7,
  PROFILE_SHORT_TYPE_HEADING_CHANGE = 8
};

/**
 * @struct ProfileShortMessage
 * @brief 
 * 
 */
struct ProfileShortMessage {
  /// 时间戳，CAN报文时间戳
  Int64_t time_stamp;

  /// [0, 7] must be 4
  MessageType type;
  
  /// [0, 3]
  Uint8_t cyclic_counter;

  bool retransmission;
  
  /// [0, 63], 0 : Unknown
  Uint8_t path_index;

  bool update;

  /// [0, 31] 1 : 曲率, 4 : 坡度; 0 : N/A
  Uint8_t profile_type;

  /// 以坡度为例，STEP和Linear插值看做0和1次多项式，通过control_point设置为true，可以通过多个PROFILE SHORT消息传递5次多项式的其余系数
  bool control_point;

  /// [0, 8191], 8191 : N/A
  Uint16_t offset;
  
  /// slope : [%] [-51.1, 51.2]; curvature : [0, 1023] 
  Float32_t value0;

  /// [0, 1023], 0 : undefined, 1023 : N/A
  Uint16_t distance1;
  
  /// [0, 1023], 1023 : N/A
  Float32_t value1;

  /// [0, 3], 3: unknown
  Uint8_t accuracy;

  void Clear() {
    type = ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT;
    cyclic_counter = INVALID_TYPE_NORMAL;
    retransmission = false;
    path_index = INVALID_TYPE_NORMAL;
    update = false;
    profile_type = PROFILE_SHORT_TYPE_UNKNOWN;
    control_point = false;
    offset = INVALID_TYPE_NORMAL;
    value0 = 0.0F;
    distance1 = INVALID_TYPE_NORMAL;
    value1 = 0.0F;
    accuracy = INVALID_TYPE_NORMAL;
  }

  ProfileShortMessage() {
    Clear();
  }
};


/**
 * @brief 描述Horizon的profile
 * 
 */
struct Profile {
  /// type
  ProfileShortType type;

  /// 距离path起点的距离，根据path起点而变化，单位：m
  Uint16_t offset_from_start;

  /// 自车与path起点的循环偏移值 [m]
  Uint16_t cyclic_offset;

  /// 循环偏移计算次数
  Uint16_t cyclic_segment_num;

  /// 距离自车的距离，根据自车位置而变化，单位：m
  int16_t offset_from_ego;

  /// 控制范围结束位置距离起点的距离，单位：m
//  Float32_t end_dist;

  /// 属性值
  Float32_t value;

  void Clear() {
    type = PROFILE_SHORT_TYPE_UNKNOWN;
    offset_from_start = INVALID_TYPE_NORMAL;
    offset_from_ego = INVALID_TYPE_NORMAL;
    cyclic_offset = 0;
    cyclic_segment_num = 0;
    value = 0.0F;
  }

  Profile() {
    Clear();
  }  
};

static const Int32_t kMaxProfilePointNum = 500;

/**
 * @brief A horizon of Av2HR from Av2HP
 * 
 */
struct Horizon {
  /// 当前时刻的POSITION
  PositionMessage position;
  /// 自车与path起点的偏移 [m]
  Uint16_t position_offset_from_start;
  /// 循环偏移计算次数
  Uint16_t position_offset_cyclic_num;

  //TODO: 简化，复用
  /// 当前时刻的Profile Short-slope
  ProfileShortMessage profile_slope;
  /// Profile Short-slope与path起点的偏移 [m]
  Uint16_t profile_slope_offset_from_start;
  /// Profile Short-slope循环偏移计算次数
  Uint16_t profile_slope_offset_cyclic_num;

  /// 当前时刻的Profile Short-curvatue
  ProfileShortMessage profile_curvatue;
  /// Profile Short-curvatue与path起点的偏移 [m]
  Uint16_t profile_curvatue_offset_from_start;
  /// Profile Short-curvatue循环偏移计算次数
  Uint16_t profile_curvatue_offset_cyclic_num;

  /// 是否满足PCC距离要求
  bool is_ready;

  /// 车道纵向坡度profiles，等长距离采样，以自车POSITION为原点
  common::StaticVector<Profile, kMaxProfilePointNum> slope_list;

  /// 车道曲率profiles，等长距离采样，以自车POSITION为原点
  common::StaticVector<Profile, kMaxProfilePointNum> curvature_list;
  
  void Clear() {
    position.Clear();
    position_offset_from_start = 0;
    position_offset_cyclic_num = 0;

    profile_slope.Clear();
    profile_slope_offset_from_start = 0;
    profile_slope_offset_cyclic_num = 0;

    profile_curvatue.Clear();
    profile_curvatue_offset_from_start = 0;
    profile_curvatue_offset_cyclic_num = 0;

    is_ready = false;

    slope_list.Clear();

    curvature_list.Clear();
  }

  Horizon() {
    Clear();
  }
};

}  // namespace map
}  // namespace phoenix
#endif // PHOENIX_MAP_ADASIS_V2_H_
