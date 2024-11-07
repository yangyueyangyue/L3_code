/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_obstacle.h
 * @brief      障碍物消息定义
 * @details    定义了障碍物消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_OBSTACLE_H_
#define PHOENIX_AD_MSG_MSG_OBSTACLE_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @enum
 * @brief 障碍物类型
 */
enum {
  OBJ_TYPE_UNKNOWN = 0,            /// 未知
  OBJ_TYPE_PASSENGER_VEHICLE,      /// 乘用车
  OBJ_TYPE_COMMERCIAL_VEHICLE,     /// 商用车
  OBJ_TYPE_SPECIAL_VEHICLE,        /// 特殊车
  OBJ_TYPE_OTHER_VEHICLE,          /// 未知类型机动车
  OBJ_TYPE_PEDESTRIAN,             /// 行人
  OBJ_TYPE_BICYCLE,                /// 非机动车(骑行者)
  OBJ_TYPE_ANIMAL,                 /// 动物
  OBJ_TYPE_DISCARD,                /// 遗撒物
  OBJ_TYPE_CURB,                   /// 路沿
  OBJ_TYPE_TRAFFIC_CONE            /// 锥形桶
};

/**
 * @enum
 * @brief 障碍物感知类别
 */
enum {
  /// 感知类别未知
  OBJ_PRCP_TYPE_UNKNOWN = 0,
  /// 单个传感器新识别出的目标(Radar)
  OBJ_PRCP_TYPE_RADAR,
  /// 单个传感器新识别出的目标(Camera)
  OBJ_PRCP_TYPE_CAMERA,
  /// 融合的目标（融合上的目标）
  OBJ_PRCP_TYPE_FUSED,
  /// 激光雷达识别出的目标
  OBJ_PRCP_TYPE_LIDAR,
};

/**
 * @struct OBBox
 * @brief 有向矩形包围盒
 */
struct OBBox {
  /// 中心坐标x
  Float32_t x;
  /// 中心坐标y
  Float32_t y;
  /// 航向角
  Float32_t heading;
  /// 半宽
  Float32_t half_width;
  /// 半长
  Float32_t half_length;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    x = 0;
    y = 0;
    heading = 0;
    half_width = 0;
    half_length = 0;
  }

  /**
   * @brief 构造函数
   */
  OBBox() {
    Clear();
  }
};

/**
 * @struct ObstacleCamera
 * @brief 从Camera识别的障碍
 */
struct ObstacleCamera {
  /// 障碍物的状态
  enum ObjStatus {
    OBJ_STATUS_UNKNOWN = 0,
    OBJ_STATUS_STANDING,
    OBJ_STATUS_STOPPED,
    OBJ_STATUS_MOVING,
    OBJ_STATUS_ONCOMING,
    OBJ_STATUS_PARKED
  };
  /// 切入/切出的类型
  enum CutInType {
    CUT_IN_TYPE_UNKNOWN = 0,
    CUT_IN_TYPE_IN_HOST_LANE,
    CUT_IN_TYPE_OUT_HOST_LANE,
    CUT_IN_TYPE_CUT_IN,
    CUT_IN_TYPE_CUT_OUT
  };
  /// 转向灯的状态
  enum BlinkerType {
    BLINKER_UNKNOWN = 0,
    BLINKER_OFF,
    BLINKER_LEFT,
    BLINKER_RIGHT,
    BLINKER_BOTH
  };

  /// 障碍物ID
  Int32_t id;
  /// 障碍物类型
  Int8_t type;
  /// 障碍物的状态
  ObjStatus status;
  /// 切入/切出的类型
  CutInType cut_in;
  /// 转向灯的状态
  BlinkerType blinker;
  /// 制动灯状态
  bool brake_lights;
  /// The age of the obstacle (in frames). This value starts at 1 when the
  /// obstacle is first detected, and increments in 1 each frame.
  Int32_t age;
  /// 指示障碍物在哪个车道
  Int32_t lane;
  /// 障碍物长度 (m)
  Float32_t length;
  /// 障碍宽度 (m)
  Float32_t width;
  /// 障碍物高度（m）
  Float32_t height;
  /// 障碍物位置x坐标 (m)
  Float32_t x;
  /// 障碍物位置y坐标 (m)
  Float32_t y;
  /// 障碍物航向 (rad)
  Float32_t heading;
  /// 障碍物的x向速度 (m/sec)
  Float32_t v_x;
  /// 障碍物的y向速度 (m/sec)
  Float32_t v_y;
  /// 障碍物的加速度 (m/sec^2)
  Float32_t accel_x;
  /// 障碍物的加速度 (m/sec^2)
  Float32_t accel_y;
  /// 障碍物的角速度 (rad/sec)
  Float32_t yaw_rate;
  /// 尺度的变化 (pix/sec)
  Float32_t scale_change;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = 0;
    type = OBJ_TYPE_UNKNOWN;
    status = OBJ_STATUS_UNKNOWN;
    cut_in = CUT_IN_TYPE_UNKNOWN;
    blinker = BLINKER_UNKNOWN;
    brake_lights = false;
    age = 0;
    lane = 0;
    length = 0.0F;
    width = 0.0F;
    height = 0.0F;
    x = 0.0F;
    y = 0.0F;
    heading = 0.0F;
    v_x = 0.0F;
    v_y = 0.0F;
    accel_x = 0.0F;
    accel_y = 0.0F;
    yaw_rate = 0.0F;
    scale_change = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  ObstacleCamera() {
    Clear();
  }
};

/**
 * @struct ObstacleCameraList
 * @brief 从Camera识别的障碍物列表
 */
struct ObstacleCameraList {
  /// 障碍物的最大数量
  enum { MAX_OBSTACLE_NUM = 32 };
  /// Camera类型定义
  enum {
    CAM_TYPE_UNKNOWN,
    CAM_TYPE_MOBILEYE_Q2,
    CAM_TYPE_MAXIEYE_D500,
    CAM_TYPE_VISUAL_CONTROL_FRONT,
    CAM_TYPE_VISUAL_CONTROL_FRONT_LEFT,
    CAM_TYPE_VISUAL_CONTROL_FRONT_RIGHT,
    CAM_TYPE_VISUAL_CONTROL_REAR,
    CAM_TYPE_VISUAL_CONTROL_REAR_LEFT,
    CAM_TYPE_VISUAL_CONTROL_REAR_RIGHT,
  };

  /// 消息头
  MsgHead msg_head;
  /// Camera类型
  Int32_t cam_type;
  /// 障碍物的数量
  Int32_t obstacle_num;
  /// 障碍物列表
  ObstacleCamera obstacles[MAX_OBSTACLE_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    cam_type = CAM_TYPE_UNKNOWN;
    obstacle_num = 0;
    for (Int32_t i = 0; i < MAX_OBSTACLE_NUM; ++i) {
      obstacles[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  ObstacleCameraList() {
    Clear();
  }
};

/**
 * @struct ObstacleRadar
 * @brief 从Radar识别的障碍物
 */
struct ObstacleRadar {
  /// 障碍物跟踪的状态
  enum TrackStatus {
    TRACK_STATUS_NO_TARGET = 0,
    TRACK_STATUS_NEW_TARGET,
    TRACK_STATUS_NEW_UPDATED_TARGET,
    TRACK_STATUS_UPDATED_TARGET,
    TRACK_STATUS_COASTED_TARGET,
    TRACK_STATUS_MERGED_TARGET,
    TRACK_STATUS_INVALID_COASTED_TARGET,
    TRACK_STATUS_NEW_COASTED_TARGET
  };
  /// 障碍物融合的类型
  enum MergedStatus {
    MERGED_STATUS_NO_TARGET = 0,
    MERGED_STATUS_MR_TARGET,
    MERGED_STATUS_LR_TARGET,
    MERGED_STATUS_MR_LR_TARGET
  };

  /// 障碍物ID
  Int32_t id;
  /// 障碍物的类型
  Int8_t type;
  /// 障碍物跟踪的状态
  TrackStatus track_status;
  /// 障碍物融合的类型
  MergedStatus merged_status;
  /// 障碍物是否是迎面驶来的
  bool oncomming;
  /// 此障碍物是否是bridge
  bool bridge;
  /// 障碍物距离传感器的距离（极坐标）(m)
  Float32_t range;
  /// 障碍物与传感器中心纵轴之间的角度（极坐标）(rad)
  Float32_t angle;
  /// 障碍物距离传感器的距离的变化率（极坐标）(m/sec)
  Float32_t range_rate;
  /// 障碍物距离传感器的距离变化率的变化率（极坐标）(m/sec^2)
  Float32_t range_acceleration;
  /// 障碍物的侧向速度（左正右负）(m/sec)
  Float32_t lateral_rate;
  /// 障碍物长度 (m)
  Float32_t length;
  /// 障碍宽度 (m)
  Float32_t width;
  /// 障碍物位置x坐标 (m)
  Float32_t x;
  /// 障碍物位置y坐标 (m)
  Float32_t y;
  /// 障碍物的x向速度 (m/sec)
  Float32_t v_x;
  /// 障碍物的y向速度 (m/sec)
  Float32_t v_y;
  /// 障碍物的x向加速度 (m/sec^2)
  Float32_t accel_x;
  /// 障碍物的y向加速度 (m/sec^2)
  Float32_t accel_y;
  /// 障碍物的角速度 (rad/sec)
  Float32_t yaw_rate;
  /// 大陆430毫米波协议字段
  /// 动态目标的置信度(0~100%)
  Uint8_t uiDynConfidence;
  /// 目标为真实目标概率(0~100%)
  Uint8_t uiProbabilityOfExistence;
  /// 高海拔目标的概率(0~100%)
  Uint8_t Obj_ucObstacleProbability;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = 0;
    type = OBJ_TYPE_UNKNOWN;
    track_status = TRACK_STATUS_NO_TARGET;
    merged_status = MERGED_STATUS_NO_TARGET;
    oncomming = false;
    bridge = false;
    range = 0.0F;
    angle = 0.0F;
    range_rate = 0.0F;
    range_acceleration = 0.0F;
    lateral_rate = 0.0F;
    length = 0.0F;
    width = 0.0F;
    x = 0.0F;
    y = 0.0F;
    v_x = 0.0F;
    v_y = 0.0F;
    accel_x = 0.0F;
    accel_y = 0.0F;
    yaw_rate = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  ObstacleRadar() {
    Clear();
  }
};

/**
 * @struct ObstacleRadarList
 * @brief 从Radar识别的障碍物列表
 */
struct ObstacleRadarList {
  /// 最大障碍物的数量
  enum { MAX_OBSTACLE_NUM = 64 };
  /// 雷达类型定义
  enum {
    RADAR_TYPE_UNKNOWN,
    RADAR_TYPE_ESR,
    RADAR_TYPE_SRR2,
    RADAR_TYPE_RR51W,
    RADAR_TYPE_ARS430,
    RADAR_TYPE_ANNGIC_FRONT_LEFT,
    RADAR_TYPE_ANNGIC_FRONT_RIGHT,
    RADAR_TYPE_ANNGIC_REAR_LEFT,
    RADAR_TYPE_ANNGIC_REAR_RIGHT
  };

  /// 消息头
  MsgHead msg_head;
  /// 雷达类型
  Int32_t radar_type;
  /// 障碍物的数量
  Int32_t obstacle_num;
  /// 障碍物列表
  ObstacleRadar obstacles[MAX_OBSTACLE_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    radar_type = RADAR_TYPE_UNKNOWN;
    obstacle_num = 0;
    for (Int32_t i = 0; i < MAX_OBSTACLE_NUM; ++i) {
      obstacles[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  ObstacleRadarList() {
    Clear();
  }
};


/**
 * @struct ObstacleLidar
 * @brief 从Lidar识别的障碍物
 */
struct ObstacleLidar {
  /// 障碍物ID
  Int32_t id;
  /// 障碍物的类型
  Int8_t type;
  /// Number of scans this object has been tracked for.
  Int32_t age;
  /// Number of scans this object has currently been predicted for without measurement update
  Int32_t prediction_age;
  /// 障碍物位置x坐标 (m)
  Float32_t x;
  /// 障碍物位置y坐标 (m)
  Float32_t y;
  /// 障碍物包围盒
  OBBox obb;
  /// 障碍物的x向速度 (m/sec)
  Float32_t v_x;
  /// 障碍物的y向速度 (m/sec)
  Float32_t v_y;
  /// 障碍物的x向加速度 (m/sec^2)
  Float32_t accel_x;
  /// 障碍物的y向加速度 (m/sec^2)
  Float32_t accel_y;
  /// 障碍物的角速度 (rad/sec)
  Float32_t yaw_rate;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = 0;
    type = OBJ_TYPE_UNKNOWN;
    age = 0;
    prediction_age = 0;
    x = 0.0F;
    y = 0.0F;
    obb.Clear();
    v_x = 0.0F;
    v_y = 0.0F;
    accel_x = 0.0F;
    accel_y = 0.0F;
    yaw_rate = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  ObstacleLidar() {
    Clear();
  }
};

/**
 * @struct ObstacleLidarList
 * @brief 从Lidar识别的障碍物列表
 */
struct ObstacleLidarList {
  /// 最大障碍物的数量
  enum { MAX_OBSTACLE_NUM = 128 };
  /// 雷达类型定义
  enum {
    LIDAR_TYPE_UNKNOWN,
    LIDAR_TYPE_IBEO_4,
    LIDAR_TYPE_VLP_16
  };

  /// 消息头
  MsgHead msg_head;
  /// 雷达类型
  Int32_t lidar_type;
  /// 障碍物的数量
  Int32_t obstacle_num;
  /// 障碍物列表
  ObstacleLidar obstacles[MAX_OBSTACLE_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    lidar_type = LIDAR_TYPE_UNKNOWN;
    obstacle_num = 0;
    common::com_memset(obstacles, 0, sizeof(obstacles));
    // for (Int32_t i = 0; i < MAX_OBSTACLE_NUM; ++i) {
    //   obstacles[i].Clear();
    // }
  }

  /**
   * @brief 构造函数
   */
  ObstacleLidarList() {
    Clear();
  }
};

struct LidarCloudPoint {
  /// Scan layer of this point (zero- based)
  Int32_t layer;
  /// 位置x坐标 (m)
  Float32_t x;
  /// 位置y坐标 (m)
  Float32_t y;
  /// 位置z坐标 (m)
  Float32_t z;

  LidarCloudPoint() {
    layer = 0;
    x = 0.0F;
    y = 0.0F;
    z = 0.0F;
  }
};

struct LidarCloud {
  /// 最大障碍物的数量
  enum { MAX_CLOUD_POINT_NUM = 1024*1024 };
  /// 雷达类型定义
  enum {
    LIDAR_TYPE_UNKNOWN,
    LIDAR_TYPE_IBEO_4,
    LIDAR_TYPE_VLP_16
  };

  /// 消息头
  MsgHead msg_head;
  /// 雷达类型
  Int32_t lidar_type;
  /// Points的数量
  Int32_t point_num;
  /// Points列表
  LidarCloudPoint points[MAX_CLOUD_POINT_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    lidar_type = LIDAR_TYPE_UNKNOWN;
    point_num = 0;
    common::com_memset(points, 0, sizeof(points));
  }

  /**
   * @brief 构造函数
   */
  LidarCloud() {
    Clear();
  }
};


/**
 * @struct ObstacleTrackedInfo
 * @brief 障碍物跟踪信息(调试用)
 */
struct ObstacleTrackedInfo {
  /// 障碍物类型
  Int8_t obj_type;
  /// 障碍物坐标x
  Float32_t x;
  /// 障碍物坐标y
  Float32_t y;
  /// 障碍物宽度
  Float32_t width;
  /// 障碍物长度
  Float32_t length;
  /// 障碍物外包围盒
  OBBox obb;

  /// 障碍物x向速度
  Float32_t v_x;
  /// 障碍物y向速度
  Float32_t v_y;

  /// 障碍物跟踪状态
  Int32_t track_status;
  /// 障碍物跟踪上的次数
  Int32_t age;
  /// 障碍物的生存期
  Int32_t duration;
  /// 障碍物的置信度
  Int32_t confidence;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    obj_type = 0;
    x = 0;
    y = 0;
    width = 0.0F;
    length = 0.0F;
    obb.Clear();

    v_x = 0.0F;
    v_y = 0.0F;

    track_status = 0;
    age = 0;
    duration = 0;
    confidence = 0;
  }

  /**
   * @brief 构造函数
   */
  ObstacleTrackedInfo() {
    Clear();
  }
};

/**
 * @struct ObstacleTrackedInfoList
 * @brief 障碍物跟踪信息列表(调试用)
 */
struct ObstacleTrackedInfoList {
  /// 障碍物跟踪列表中最大跟踪障碍物的数量
  enum { MAX_OBSTACLE_NUM = 2048 };

  /// 消息头
  MsgHead msg_head;
  /// 障碍物跟踪信息的数量
  Int32_t obstacle_num;
  /// 障碍物跟踪信息列表
  ObstacleTrackedInfo obstacles[MAX_OBSTACLE_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    obstacle_num = 0;
    for (Int32_t i = 0; i < MAX_OBSTACLE_NUM; ++i) {
      obstacles[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  ObstacleTrackedInfoList() {
    Clear();
  }
};


/**
 * @struct Obstacle
 * @brief 障碍物信息
 */
struct Obstacle {
  /**
   * @enum MAX_PRED_PATH_NUM
   * @brief 最大预测的轨迹的数量
   */
  enum { MAX_PRED_PATH_NUM = 2 };
  /**
   * @enum MAX_PRED_PATH_POINT_NUM
   * @brief 最大预测的轨迹中点的数量
   */
  enum { MAX_PRED_PATH_POINT_NUM = 10 };
  /**
   * @enum MAX_PREV_PATH_POINT_NUM
   * @brief 最大Tracked轨迹中点的数量
   */
  enum { MAX_TRACKED_PATH_POINT_NUM = 15 };
  /**
   * @brief 以下枚举专门用于融合ID管理时，专门用于编解码融合ID时使用。
   * 此处枚举专门针对Radar类型传感器进行编码，范围设置为0~7，即可以用 3 bits 来表示。
   */
  enum { MAIN_FRONT_RADAR_INDEX_FOR_ID = 0,
         LEFT_FRONT_RADAR_INDEX_FOR_ID,
         RIGHT_FRONT_RADAR_INDEX_FOR_ID,
         LEFT_REAR_RADAR_INDEX_FOR_ID,
         RIGHT_REAR_RADAR_INDEX_FOR_ID,
         NONE_RADAR_INDEX_FOR_ID = 7
        };
  /**
   * @brief 以下枚举专门用于融合ID管理时，专门用于编解码融合ID时使用。
   * 此处枚举专门针对Camera类型传感器进行编码，范围设置为0~7，即可以用 3 bits 来表示。
   */
  enum { MAIN_FRONT_CAMERA_INDEX_FOR_ID = 0,
         LEFT_SIDE_CAMERA_INDEX_FOR_ID,
         RIGHT_SIDE_CAMERA_INDEX_FOR_ID,
         LEFT_REAR_CAMERA_INDEX_FOR_ID,
         RIGHT_REAR_CAMERA_INDEX_FOR_ID,
         FRONT_CAMERA_INDEX_FOR_ID,
         NONE_CAMERA_INDEX_FOR_ID = 7
        };



  /// 障碍物ID
  Int32_t id;
  /// 障碍物位置x坐标
  Float32_t x;
  /// 障碍物位置y坐标
  Float32_t y;
  /// 障碍物包围盒
  OBBox obb;
  /// 障碍物高度
  Float32_t height;
  /// 离地面高度
  Float32_t height_to_ground;
  /// 障碍物类型
  Int8_t type;
  /// 是否是动态障碍物
  Int8_t dynamic;
  /// 障碍物存在的置信度
  Int8_t confidence;
  /// 感知类别
  Int8_t perception_type;
  /// 障碍物绝对速度，沿着车身纵轴的速度，单位：米/秒
  Float32_t v_x;
  /// 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
  Float32_t v_y;
  /// 障碍物绝对速度，单位：米/秒
  Float32_t v;
  /// 障碍物绝对加速度，沿着车身纵轴的加速度，单位：米^2/秒
  Float32_t a_x;
  /// 障碍物绝对加速度，沿着车身横轴的加速度，单位：米^2/秒
  Float32_t a_y;
  /// 障碍物绝对加速度，单位：米^2/秒
  Float32_t a;
  /// 障碍物在主参考线上的投影信息

  struct {
    /// 此信息是否有效 (0 ~ 无效, 1 ~ 有效)
    Int8_t valid;
    /// 障碍物在主参考线上的投影点的x坐标
    Float32_t x;
    /// 障碍物在主参考线上的投影点的y坐标
    Float32_t y;
    /// 障碍物在主参考线上的投影点的航向角（单位：弧度）
    Float32_t heading;
    /// 障碍物在主参考线上的投影点的曲率（单位：1/米）
    Float32_t curvature;
    /// 障碍物在主参考线上的投影点沿着路径的弧长（单位：米）
    Float32_t s;
    /// 障碍物与主参考线之间的侧向距离（单位：米），左正右负
    Float32_t l;
  } proj_on_major_ref_line;
  /// 预测的轨迹的数量
  Int8_t pred_path_num;
  /// 预测的轨迹中点的数量
  Int8_t pred_path_point_num[MAX_PRED_PATH_NUM];

  /// 对动态障碍物预测的轨迹
  struct {
    /// 形点坐标x
    Float32_t x;
    /// 形点坐标y
    Float32_t y;
    /// 障碍物在此点的航向
    Float32_t heading;
    /// 障碍物在此点沿着轨迹的长度
    Float32_t s;
  } pred_path[MAX_PRED_PATH_NUM][MAX_PRED_PATH_POINT_NUM];

  /// 障碍物跟踪的轨迹(障碍物历史轨迹)中点的数量
  Int8_t tracked_path_point_num;

  /// 障碍物跟踪轨迹轨迹
  struct {
    /// 形点坐标x
    Float32_t x;
    /// 形点坐标y
    Float32_t y;
  } tracked_path[MAX_TRACKED_PATH_POINT_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = 0;
    x = 0.0F;
    y = 0.0F;
    obb.Clear();
    height = 0.0F;
    height_to_ground = 0.0F;
    type = OBJ_TYPE_UNKNOWN;
    dynamic = 0;
    confidence = 0;
    perception_type = OBJ_PRCP_TYPE_UNKNOWN;
    v_x = 0.0F;
    v_y = 0.0F;
    v = 0.0F;
    a_x = 0.0F;
    a_y = 0.0F;
    a = 0.0F;
    proj_on_major_ref_line.valid = 0;
    proj_on_major_ref_line.x = 0.0F;
    proj_on_major_ref_line.y = 0.0F;
    proj_on_major_ref_line.heading = 0.0F;
    proj_on_major_ref_line.curvature = 0.0F;
    proj_on_major_ref_line.s = 0.0F;
    proj_on_major_ref_line.l = 0.0F;
    pred_path_num = 0;
    common::com_memset(pred_path_point_num, 0, sizeof(pred_path_point_num));
    common::com_memset(pred_path, 0, sizeof(pred_path));
    tracked_path_point_num = 0;
    common::com_memset(tracked_path, 0, sizeof(tracked_path));
  }

  /**
   * @brief 构造函数
   */
  Obstacle() {
    Clear();
  }
  /**
   * @brief 判断大小端，用来判断是否为大端
   * 
   * @return true 表示大端
   * @return false 表示小端
   */
  static bool IsBigEndian(){
    int i = 0x11223344;
    if (*(char*)(&i) == 0x44)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  /**
   * @brief 利用Radar、Camera、Lidar 障碍物ID来编码融合的ID，前提是传感器的ID值都是8 bits表示的数据。
   * 
   * @param radar_type  Radar类型，参看 MAIN_FRONT_RADAR_INDEX_FOR_ID 所对应的枚举
   * @param camera_type  Radar类型，参看 MAIN_FRONT_RADAR_INDEX_FOR_ID 所对应的枚举
   * @param exist_lidar  Lidar障碍物是否存在，Lidar只有一个，0表示无，1表示有
   * @param radar_id 
   * @param camera_id 
   * @param lidar_id 
   * @param fusion_id 
   * @return true 
   * @return false 
   */
  static bool EncodeFusionID(Int8_t radar_type, Int8_t camera_type, Int8_t exist_lidar, 
                            Uint8_t radar_id, Uint8_t camera_id, Uint8_t lidar_id, Int32_t& fusion_id)
  {
    bool is_bigendian = IsBigEndian();
    Int32_t output=0;
    Uint8_t* ptr = (Uint8_t*)(&output);
    if (is_bigendian){
      *(ptr+3)=  radar_id;  //存储Radar ID数据
      *(ptr+2) = camera_id; //存储Camera ID数据
      if (exist_lidar > 0){ //判断Lidar是否存在
        *(ptr+1) = lidar_id; //存储Lidar ID数据
      }
      *ptr = (radar_type & 0x07)|((camera_type & 0x07) << 3)|((exist_lidar & 0x01) << 6);//处理表示传感器类型的字节
    }
    else{
      *(ptr)=  radar_id;  //存储Radar ID数据
      *(ptr+1) = camera_id; //存储Camera ID数据
      if (exist_lidar > 0){ //判断Lidar是否存在
        *(ptr+2) = lidar_id; //存储Lidar ID数据
      }
      *(ptr+3) = (radar_type & 0x07)|((camera_type & 0x07) << 3)|((exist_lidar & 0x01) << 6);//处理表示传感器类型的字节
    }
    fusion_id = output;
    return true;    
  }

  /**
   * @brief 根据融合ID解码出 Radar ID、Camera ID、Lidar ID以及对应的传感器类型。
   * 
   * @param fusion_id 
   * @param radar_type  Radar类型，参看 MAIN_FRONT_RADAR_INDEX_FOR_ID 所对应的枚举
   * @param camera_type  Radar类型，参看 MAIN_FRONT_RADAR_INDEX_FOR_ID 所对应的枚举
   * @param exist_lidar  Lidar障碍物是否存在，Lidar只有一个，0表示无，1表示有
   * @param radar_id 
   * @param camera_id 
   * @param lidar_id 
   * @return true 
   * @return false 
   */
  static bool DecodeFusionID(Int32_t fusion_id, Int8_t& radar_type, Int8_t& camera_type, Int8_t& exist_lidar, 
                            Uint8_t& radar_id, Uint8_t& camera_id, Uint8_t& lidar_id)
  {
    if (fusion_id < 0){
      return false;
    }
    bool is_bigendian = IsBigEndian();
    Uint8_t* ptr = (Uint8_t*)(&fusion_id);
    if (is_bigendian){
      radar_id = *(ptr+3);  //读取Radar ID数据
      camera_id = *(ptr+2); //读取Camera ID数据
      if (exist_lidar > 0){ //判断Lidar是否存在
        lidar_id = *(ptr+1); //读取Lidar ID数据
      }
      Uint8_t status_byte = *ptr;//读取传感器类型的字节
      radar_type = status_byte & 0x07; //读取Radar类型
      camera_type = (status_byte >> 3) & 0x07;//读取Camera类型
      exist_lidar = (status_byte >> 6) & 0x01;//读取Lidar是否存在的状态
    }
    else{
      radar_id = *ptr;  //读取Radar ID数据
      camera_id = *(ptr+1); //读取Camera ID数据
      if (exist_lidar > 0){ //判断Lidar是否存在
        lidar_id = *(ptr+2); //读取Lidar ID数据
      }
      Uint8_t status_byte = *(ptr+3);//读取传感器类型的字节
      radar_type = status_byte & 0x07; //读取Radar类型
      camera_type = (status_byte >> 3) & 0x07;//读取Camera类型
      exist_lidar = (status_byte >> 6) & 0x01;//读取Lidar是否存在的状态
    }
    return true;    
  }

};

/**
 * @struct ObstacleList
 * @brief 障碍物列表
 */
struct ObstacleList {
  /**
   * @enum MAX_OBSTACLE_NUM
   * @brief 列表中最大障碍物的数量
   */
  enum { MAX_OBSTACLE_NUM = 64 };

  /// 消息头
  MsgHead msg_head;
  /// 障碍物的数量
  Int32_t obstacle_num;
  /// 障碍物列表，障碍物的位置使用车身坐标系表达
  Obstacle obstacles[MAX_OBSTACLE_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    obstacle_num = 0;
    for (Int32_t i = 0; i < MAX_OBSTACLE_NUM; ++i) {
      obstacles[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  ObstacleList() {
    Clear();
  }
};

} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_OBSTACLE_H_


