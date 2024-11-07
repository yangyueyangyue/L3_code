/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       driving_map.h
 * @brief      驾驶地图共通数据结构定义
 * @details    定义了驾驶地图共通数据类型
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_DRIVING_MAP_DRIVING_MAP_H_
#define PHOENIX_DRIVING_MAP_DRIVING_MAP_H_


#include "utils/macros.h"
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
#include <string>
#include "modules/map/proto/map.pb.h"
#include "modules/routing/proto/routing.pb.h"
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
#include "map_common.h"
#endif

#include "msg_planning_story.h"
#include "utils/com_utils.h"
#include "container/static_vector.h"
#include "container/unordered_map.h"
#include "curve/path.h"
#include "geometry/obbox2d.h"
#include "geometry/line_segment2d.h"
#include "ad_msg.h"


namespace phoenix {
namespace driv_map {


/// 驾驶地图模块基本数据类型
typedef Float32_t map_var_t;

/**
 * @enum MAX_DRIVING_MAP_IMPL_NUM
 * @brief 驾驶地图的最大数量
 */
enum { MAX_DRIVING_MAP_IMPL_NUM = 1 };

/**
 * @enum MAX_LANE_NUM
 * @brief 内部地图中可以保存的最多车道的数量
 */
enum { MAX_LANE_NUM = 20 };

/**
 * @enum MAX_MAP_TRAFFIC_LIGHT_NUM
 * @brief 内部地图中可以保存的红绿灯的数量
 */
enum { MAX_MAP_TRAFFIC_LIGHT_NUM = 10 };

/**
 * @enum MAX_REFERENCE_LINE_NUM
 * @brief 最大参考线的数量
 */
enum { MAX_REFERENCE_LINE_NUM = 10 };

/**
 * @enum MAX_NEIGHBOR_LANE_NUM
 * @brief 最大相邻车道的数量
 */
enum { MAX_NEIGHBOR_LANE_NUM = 10 };

/**
 * @enum MAX_LANE_SEGMENT_NUM
 * @brief 最大车道片段的数量
 */
enum { MAX_LANE_SEGMENT_NUM = 10 };

/**
 * @enum MAX_NEIGHBOR_ROUTING_NUM
 * @brief 最大相邻的导航车道片段数量
 */
enum { MAX_NEIGHBOR_ROUTING_NUM = 3 };

/**
 * @enum MAX_LANE_SECTION_NUM
 * @brief 最大导航车道片段数量
 */
enum { MAX_ROUTING_SECTION_NUM = 10 };

/**
 * @enum MAX_OBJ_PRED_TRAJ_POINT_NUM
 * @brief 每条动态障碍物预测的轨迹点的最大数量
 */
enum { MAX_OBJ_PRED_TRAJ_POINT_NUM = 10 };

/**
 * @enum MAX_OBJ_PRED_TRAJ_NUM
 * @brief 动态障碍物预测的轨迹的数量
 */
enum { MAX_OBJ_PRED_TRAJ_NUM = 2 };

/**
 * @enum MAX_OBJECT_NUM_IN_OBJ_SPACE
 * @brief 空间中最多可以保存的障碍物的数量
 */
enum { MAX_OBJECT_NUM_IN_OBJ_SPACE = 64 };

/**
 * @enum
 * @brief 驾驶地图类型
 */
enum {
  /// 无效的类型
  DRIVING_MAP_TYPE_INVALID = 0,
  /// 从高精度地图构建
  DRIVING_MAP_TYPE_HD_MAP,
  /// 从相机车道线构建
  DRIVING_MAP_TYPE_CAMERA_LANE,
  /// 跟随前车
  DRIVING_MAP_TYPE_FOLLOWING_PATH,
  /// 根据当前车辆状态预测的轨迹
  DRIVING_MAP_TYPE_PRED_PATH,
  /// 融合了相机车道线及高精度地图
  DRIVING_MAP_TYPE_MIXED_HD_MAP
};

/**
 * @enum
 * @brief 车道的质量
 * @warning {必须按照质量由低到高进行排列}
 */
enum {
  LANE_QUALITY_INVALID = 0,
  LANE_QUALITY_BAD,
  LANE_QUALITY_NOT_GOOD,
  LANE_QUALITY_GOOD
};

/**
 * @enum
 * @brief 车道类型
 */
enum {
  LANE_TYPE_UNKNOWN = 0,
  LANE_TYPE_CITY_DRIVING,
  LANE_TYPE_BIKING,
  LANE_TYPE_SIDEWALK,
  LANE_TYPE_PARKING
};

/**
 * @enum
 * @brief 车道弯道类型
 */
enum {
  LANE_TURN_TYPE_UNKNOWN = 0,
  LANE_TURN_TYPE_NO_TURN,
  LANE_TURN_TYPE_LEFT_TURN,
  LANE_TURN_TYPE_RIGHT_TURN,
  LANE_TURN_TYPE_U_TURN
};

/**
 * @enum
 * @brief 车道方向
 */
enum {
  LANE_DIRECTION_FORWARD = 0,
  LANE_DIRECTION_BACKWARD,
  LANE_DIRECTION_BIDIRECTION
};

/**
 * @enum
 * @brief 车道边界类型
 */
enum {
  LANE_BOUNDARY_TYPE_UNKNOWN  = 0,
  LANE_BOUNDARY_TYPE_DOTTED_YELLOW,
  LANE_BOUNDARY_TYPE_DOTTED_WHITE,
  LANE_BOUNDARY_TYPE_SOLID_YELLOW,
  LANE_BOUNDARY_TYPE_SOLID_WHITE,
  LANE_BOUNDARY_TYPE_DOUBLE_YELLOW,
  LANE_BOUNDARY_TYPE_CURB
};

/**
 * @enum
 * @brief 障碍物的方位
 */
enum {
  OBJ_POSITION_UNKNOWN = 0,
  OBJ_POSITION_FRONT = 1,
  OBJ_POSITION_LEFT_FRONT = 2,
  OBJ_POSITION_LEFT = 3,
  OBJ_POSITION_LEFT_BACK = 4,
  OBJ_POSITION_BACK = 5,
  OBJ_POSITION_RIGHT_BACK = 6,
  OBJ_POSITION_RIGHT = 7,
  OBJ_POSITION_RIGHT_FRONT = 8,
  OBJ_POSITION_CLOSE = 9
};

/**
 * @enum
 * @brief 障碍物的运动方向
 */
enum {
  OBJ_DIRECTION_UNKNOWN = 0,
  OBJ_DIRECTION_FORWARD = 1,
  OBJ_DIRECTION_BACKWARD = 2,
  OBJ_DIRECTION_CROSSED = 3
};

/**
 * @struct DrivingMapConfig
 * @brief 驾驶地图配置项
 */
struct DrivingMapConfig {
  enum {
    /// 从相机车道线构建驾驶地图
    INPUTTED_MAP_TYPE_CAMERA = 0,
    /// 从高精度地图构建驾驶地图
    INPUTTED_MAP_TYPE_HDMAP,
    /// 融合相机车道线及高精度地图
    INPUTTED_MAP_TYPE_MIXED
  };

  /// 地图输入类型
  Int32_t inputted_map_type;

  /**
   * @brief 构造函数
   */
  DrivingMapConfig() {
    inputted_map_type = INPUTTED_MAP_TYPE_CAMERA;
  }
};

/**
 * @struct DrivingMapDataSource
 * @brief 驾驶地图源数据，用于更新驾驶地图
 */
struct DrivingMapDataSource {
  /// 时间戳
  Int64_t timestamp;
  /// 定位信息
  const ad_msg::Gnss* gnss;
  const ad_msg::RelativePosList* rel_pos_list;
  /// 车辆姿态信息
  const ad_msg::Imu* imu;
  /// 车身信息（通常是车身CAN信号）
  const ad_msg::Chassis* chassis;
  /// 高精度地图及导航信息
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
  const apollo::hdmap::Map* map;
  const apollo::routing::RoutingResponse* routing;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  const MAP::MAP_VecLaneInfo* map;
  const MAP::MAP_Routing* routing;
  const MAP::MAP_Location* map_location;
#else
  // dummy
  const void* map;
  const void* routing;
#endif
  // 地图定位
  const ad_msg::MapLocalization* map_localization;
  // 场景任务
  const ad_msg::SceneStoryList* scene_story_list;

  // 是否允许删除过期的车道线
  bool allow_removing_expired_camera_lane;
  // 感知模块识别的车道线信息（通常是摄像头识别的车道线）
  const ad_msg::LaneInfoCameraList* camera_lane_list;
  // 感知识别的交通标志信息（通常是摄像头识别的交通标志）
  const ad_msg::TrafficSignalList* traffic_signal_list;

  // Planning storys
  const ad_msg::PlanningStoryList* planning_story_list;


  /**
   * @brief 构造函数
   */
  DrivingMapDataSource() {
    timestamp = 0;
    gnss = Nullptr_t;
    rel_pos_list = Nullptr_t;
    imu = Nullptr_t;
    chassis = Nullptr_t;
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
    map = Nullptr_t;
    routing = Nullptr_t;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    map = Nullptr_t;
    routing = Nullptr_t;
#else
    // dummy
    map = Nullptr_t;
    routing = Nullptr_t;
#endif
    map_localization = Nullptr_t;
    scene_story_list = Nullptr_t;

    allow_removing_expired_camera_lane = true;
    camera_lane_list = Nullptr_t;
    traffic_signal_list = Nullptr_t;

    planning_story_list = Nullptr_t;
  }
};


/**
 * @struct ID
 * @brief 定义车道及地图的ID类型
 */
struct ID {
  /// id
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  std::string id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  Uint64_t id;
#else
  Int32_t id;
#endif

  /**
   * @brief 清除内部数据
   */
  void Clear() {
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    id.clear();
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    id = 0;
#else
    id = 0;
#endif
  }

  /**
   * @brief 判断ID是否相同
   * @param[in] other 另一个ID
   */
  bool operator ==(const ID& other) const {
    return (id == other.id);
  }

  /**
   * @brief 构造函数
   */
  ID() {
    Clear();
  }
};

/**
 * @struct LaneSegment
 * @brief 车道片段
 */
struct LaneSegment {
  /// 车道id
  ID lane_id;
  /// 车道索引
  Int32_t lane_index;
  /// 车道片段在车道中心线上的起点
  Float32_t start_s;
  /// 车道片段在车道中心线上的终点
  Float32_t end_s;
  /// 注意：以当前车辆位置在参考线上的投影处作为在参考线上的起点(s=0),
  ///      不是以参考线本身的起始点作为起点(s=0)，使用时需注意存在这个偏移量
  Float32_t start_s_on_ref;
  Float32_t end_s_on_ref;
  /// 是否在地图导航规划的路径上
  bool is_on_routing;
  /// 如果在地图导航规划的路径上，则下列索引指出了它在地图导航路径列表中的位置
  struct {
    /// 导航路径片段的索引
    Int32_t section_index;
    /// 导航路径片段中车道段的索引
    Int32_t neighbor_index;
  } routing_association;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    lane_id.Clear();
    lane_index = -1;
    start_s = 0;
    end_s = 0;
    start_s_on_ref = 0;
    end_s_on_ref = 0;
    is_on_routing = false;
    routing_association.section_index = -1;
    routing_association.neighbor_index = -1;
  }

  /**
   * @brief 构造函数
   */
  LaneSegment() {
    Clear();
  }
};

/**
 * @struct RoutingSegment
 * @brief routing片段
 */
struct RoutingSegment {
  /// 车道id
  ID lane_id;
  /// 车道索引
  Int32_t lane_index;
  /// 车道片段在车道中心线上的起点
  Float32_t start_s;
  /// 车道片段在车道中心线上的终点
  Float32_t end_s;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    lane_id.Clear();
    lane_index = -1;
    start_s = 0;
    end_s = 0;
  }

  /**
   * @brief 构造函数
   */
  RoutingSegment() {
    Clear();
  }
};

/**
 * @struct RoutingSection
 * @brief 组成内部导航路径的导航道路段结构体
 */
struct RoutingSection {
  /// 组成导航道路段的左右相邻导航车道列表
  common::StaticVector<RoutingSegment, MAX_NEIGHBOR_ROUTING_NUM> neighbors;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    neighbors.Clear();
  }

  /**
   * @brief 构造函数
   */
  RoutingSection() {
    Clear();
  }
};

/**
 * @struct RoadBoundary
 * @brief 道路边界
 */
struct RoadBoundary {
  /// 在主参考线上的采样点
  common::PathPoint ref_point;
  /// 左道路边界点
  common::PathPoint left_boundary_point;
  /// 右道路边界点
  common::PathPoint right_boundary_point;
  /// 左道路边界点与主参考线之间的距离
  Float32_t left_width;
  /// 右道路边界点与主参考线之间的距离
  Float32_t right_width;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    ref_point.Clear();
    left_boundary_point.Clear();
    right_boundary_point.Clear();
    left_width = 0.0F;
    right_width = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  RoadBoundary() {
    Clear();
  }
};


/**
 * @struct LaneInfo
 * @brief 车道信息，用于获取驾驶地图内部的车道信息
 */
struct LaneInfo {
  /**
   * @struct Boundary
   * @brief 车道边线
   */
  struct Boundary {
    /**
     * @struct Boundary
     * @brief 车道边界
     */
    struct BoundaryAssociation {
      /// 在参考线上的路径长
      map_var_t s;
      /// 宽度（车道半宽）
      map_var_t width;
      /// 边线类型
      Int32_t type;
      /// 边线型点
      common::Vec2d point;

      /**
       * @brief 将成员变量的值清空。
       */
      void Clear() {
        s = 0;
        width = 0;
        type = LANE_BOUNDARY_TYPE_UNKNOWN;
      }

      /**
       * @brief 构造函数。
       */
      BoundaryAssociation() {
        Clear();
      }
    };
    /// 车道边线
    common::StaticVector<BoundaryAssociation,
    common::Path::kMaxPathPointNum> curve;

    /**
     * @brief 清除内部数据(车道边界)
     */
    void Clear() {
      curve.Clear();
    }

    /**
     * @brief 构造函数
     */
    Boundary() {
      Clear();
    }
  };

  /// 车道id
  ID lane_id;
  /// 车道索引
  Int32_t lane_index;
  /// 车道的质量
  Int32_t quality;
  /// 车道中心线
  common::StaticVector<common::Vec2d,
      common::Path::kMaxPathPointNum> central_curve;
  /// 车道左边界
  Boundary left_boundary;
  /// 车道右边界
  Boundary right_boundary;

  /**
   * @brief 清除内部数据(车道信息)
   */
  void Clear() {
    lane_id.Clear();
    lane_index = -1;
    quality = LANE_QUALITY_INVALID;
    central_curve.Clear();
    left_boundary.Clear();
    right_boundary.Clear();
  }

  /**
   * @brief 构造函数
   */
  LaneInfo() {
    Clear();
  }
};


/**
 * @struct MapTrafficLight
 * @brief 地图中红绿灯信息
 */
struct MapTrafficLight {
  /// Signal id
  ID id;
  /// Stop line
  common::LineSegment2d stop_line;

  /**
   * @brief 清除内部数据(车道信息)
   */
  void Clear() {
    id.Clear();
    stop_line.Clear();
  }

  /**
   * @brief 构造函数
   */
  MapTrafficLight() {
    Clear();
  }
};


/**
 * @struct MapInfo
 * @brief 地图信息，用于获取驾驶地图内部的地图信息
 */
struct MapInfo {
  /// 车道列表
  common::StaticVector<LaneInfo, MAX_LANE_NUM> lane_table;

  /// 地图中红绿灯信息列表
  common::StaticVector<MapTrafficLight,
      MAX_MAP_TRAFFIC_LIGHT_NUM> map_traffic_light_table;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    for (Int32_t i = 0; i < lane_table.Size(); ++i) {
      lane_table[i].Clear();
    }
    lane_table.Clear();

    for (Int32_t i = 0; i < map_traffic_light_table.Size(); ++i) {
      map_traffic_light_table[i].Clear();
    }
    map_traffic_light_table.Clear();
  }

  /**
   * @brief 构造函数
   */
  MapInfo() {
  }
};

/**
 * @struct NeighborsLaneInfo
 * @brief 相邻车道信息，用于查询相邻的车道
 */
struct NeighborsLaneInfo {
  /// 此相邻车道是否有效（是否在有效的范围内）
  bool valid_lat;
  bool valid_lon;
  /// Lane ID
  ID lane_id;
  /// 车道索引
  Int32_t lane_index;
  /// Neighbor Flag
  Int32_t neighbor_flag;
  /// 查询点在此车道上的投影点
  common::PathPoint proj_point;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    valid_lat = false;
    valid_lon = false;
    lane_id.Clear();
    lane_index = -1;
    proj_point.Clear();
  }

  /**
   * @brief 构造函数
   */
  NeighborsLaneInfo() {
    valid_lat = false;
    valid_lon = false;
    lane_index = -1;
  }
};



/**
 * @struct CollisionRiskTestObj
 * @brief 用于碰撞分析的输入的目标物信息
 */
struct CollisionTestObj {
  /// 目标物的包围盒
  common::OBBox2d obb;
  /// 目标物的包围盒的外接圆的半径
  map_var_t obb_circumradius;
  /// 目标物的速度
  map_var_t v;
  /// 目标物运动到此位置所需要的时间
  map_var_t t;
  /// 目标物运动到此位置所需要的距离
  map_var_t s;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    obb.Clear();
    obb_circumradius = 0;
    v = 0;
    t = 0;
    s = 0;
  }

  /**
   * @brief 构造函数
   */
  CollisionTestObj() {
    obb_circumradius = 0;
    v = 0;
    t = 0;
    s = 0;
  }
};

/**
 * @struct CollisionRiskTestResult
 * @brief 碰撞风险测试的结果
 */
struct CollisionTestResult {
  /**
   * @enum MAX_RISKY_OBJ_NUM
   * @brief 最大可能产生碰撞风险的障碍物的数量
   */
  enum { MAX_RISKY_OBJ_NUM = 8 };

  /// 障碍物信息
  struct ObjInfo {
    /// 障碍物在列表中的索引
    Int32_t obj_list_index;
    /// 此障碍物对应的风险值
    Int32_t risk_value;
    /// 与障碍物之间的距离(动态距离)
    map_var_t dynamic_distance;
    /// 与障碍物之间的距离(静态距离)
    map_var_t static_distance;
    /// 障碍物在参考线上的路径长
    map_var_t obj_s_ref;
    /// 障碍物在与参考线上之间的横向偏差
    map_var_t obj_l_ref;
    /// 与此障碍物有碰撞风险的目标物体在轨迹上的路径长
    map_var_t collision_s;
    /// 保存了障碍物采样点相关的信息
    /// （意味着障碍物在其这个采样点的位置，存在碰撞风险）
    common::TrajectoryPoint obj_traj_point;

    /**
     * @brief 比较两个碰撞风险测试结果中的风险大小（重载小于操作符号）
     * @param[in] right 小于操作符右侧的碰撞风险测试结果
     * @return true ~ 操作符左侧风险小，false ~ 操作符右侧的风险小
     */
    bool operator <(const ObjInfo& right) const {
      if (risk_value < right.risk_value) {
        return true;
      } else if (risk_value == right.risk_value) {
        if (right.dynamic_distance < (dynamic_distance-0.1F)) {
          return true;
        } else if ((dynamic_distance-0.11F) <= right.dynamic_distance &&
                   right.dynamic_distance <= (dynamic_distance+0.11F)) {
          if (right.static_distance < static_distance) {
            return true;
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

    /**
   * @brief 清除内部数据
   */
    void Clear() {
      obj_list_index = -1;
      risk_value = 0;
      dynamic_distance = 0;
      static_distance = 0;
      obj_s_ref = 0;
      obj_l_ref = 0;
      collision_s = 0;
      obj_traj_point.Clear();
    }

    /**
     * @brief 构造函数
     */
    ObjInfo() {
      Clear();
    }
  };

  /// 有风险的障碍物列表
  common::StaticVector<ObjInfo, MAX_RISKY_OBJ_NUM> risky_obj_list;
  /// 不确定的障碍物列表
  common::StaticVector<ObjInfo, MAX_RISKY_OBJ_NUM> uncertain_list;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    for (Int32_t i = 0; i < risky_obj_list.Size(); ++i) {
      risky_obj_list[i].Clear();
    }
    risky_obj_list.Clear();

    for (Int32_t i = 0; i < uncertain_list.Size(); ++i) {
      uncertain_list[i].Clear();
    }
    uncertain_list.Clear();
  }

  /**
   * @brief 构造函数
   */
  CollisionTestResult() {
    Clear();
  }
};

/**
* @struct CollisionTestParam
* @brief 用于碰撞分析的相关配置参数
*/
struct CollisionTestParam {
  /// 是否对可忽略障碍物进行碰撞分析
  bool testing_ignored_object;
  /// 是否只返回单个有碰撞风险的障碍物
  bool return_single_risky_object;
  /// 是否返回不确定的障碍物
  bool return_uncertain_object;
  /// 最大有碰撞风险的静态障碍物的返回个数
  Int32_t max_num_of_static_obj_to_return;
  /// 最大有碰撞风险的动态障碍物的返回个数
  Int32_t max_num_of_dynamic_obj_to_return;

  /**
   * @brief 设置为默认参数
   * @par Note:
   * @code
   *     1、不对被忽略的障碍物进行碰撞分析
   *     2、最多只返回一个最有风险的静态和动态障碍物
   *     3、最多返回的静态障碍物的数量为2,
   *        最多返回的动态障碍物的数量为(MAX_RISKY_OBJECT_NUM - 2)
   * @endcode
   */
  void SetDefaultParam() {
    testing_ignored_object = false;
    return_single_risky_object = false;
    return_uncertain_object = true;

    max_num_of_static_obj_to_return = 2;
    max_num_of_dynamic_obj_to_return =
      static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM) -
      max_num_of_static_obj_to_return;

    COM_CHECK((0 < max_num_of_static_obj_to_return) &&
              (max_num_of_static_obj_to_return < CollisionTestResult::MAX_RISKY_OBJ_NUM));
    COM_CHECK((0 < max_num_of_dynamic_obj_to_return) &&
              (max_num_of_dynamic_obj_to_return < CollisionTestResult::MAX_RISKY_OBJ_NUM));
  }

  /**
   * @brief 设置是否在碰撞分析时对设置为忽略的障碍物进行碰撞分析
   * @param[in] value true - 对被忽略的障碍物进行碰撞分析 \n
   *                  false - 不对被忽略的障碍物进行碰撞分析
   */
  void SetTestingIgnoredObj(bool value) {
    testing_ignored_object = value;
  }

  /**
   * @brief 设置是否在碰撞分析时只返回单个最有风险的障碍物
   * @param[in] return_single_risky_object \n
   *      true - 最多只返回一个最有风险的静态和动态障碍物 \n
   *      false - 可以返回多个有风险的静态及动态障碍物
   */
  void SetReturnSingleRiskyObj(bool value) {
    return_single_risky_object = value;
  }

  /**
   * @brief 设置碰撞分析时返回的有风险的障碍物列表中最大的障碍物数量
   * @param[in] max_num_of_static_obj 最多返回的静态障碍物的数量
   * @param[in] max_num_of_dynamic_obj 最多返回的动态障碍物的数量
   * @par Note:
   * @code
   *     max_num_of_static_obj 不能超过(MAX_RISKY_OBJECT_NUM - 1)
   *     max_num_of_dynamic_obj 不能超过
   *            (MAX_RISKY_OBJECT_NUM - max_num_of_static_obj)
   * @endcode
   */
  void SetMaxNumOfObjToReturn(
      Int32_t max_num_of_static_obj, Int32_t max_num_of_dynamic_obj) {
    max_num_of_static_obj_to_return = max_num_of_static_obj;
    if (max_num_of_static_obj_to_return >
        (static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM) - 1)) {
      max_num_of_static_obj_to_return =
          static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM) - 1;
    }
    max_num_of_dynamic_obj_to_return = max_num_of_dynamic_obj;
    if (max_num_of_dynamic_obj_to_return >
        (static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM) -
         max_num_of_static_obj_to_return)) {
      max_num_of_dynamic_obj_to_return =
          static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM) -
          max_num_of_static_obj_to_return;
    }

    COM_CHECK((0 < max_num_of_static_obj_to_return) &&
              (max_num_of_static_obj_to_return <
               static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM)));
    COM_CHECK((0 < max_num_of_dynamic_obj_to_return) &&
              (max_num_of_dynamic_obj_to_return <
               static_cast<Int32_t>(CollisionTestResult::MAX_RISKY_OBJ_NUM)));
  }

  /**
   * @brief 构造函数
   */
  CollisionTestParam() {
    SetDefaultParam();
  }
};

/**
 * @struct CollisionTestOnPathObj
 * @brief 对轨迹进行碰撞分析时，输入的自车信息
 */
struct CollisionTestOnPathObj {
  /// 自车相对与轨迹起点的时间偏置量
  map_var_t t_offset;
  /// 自车相对与轨迹起点的时间偏置量
  map_var_t s_offset;
  /// 自车定位点到其包围盒中心的纵向偏移量
  map_var_t x_offset;
  /// 自车半长
  map_var_t obj_half_length;
  /// 自车半宽
  map_var_t obj_half_width;
  /// 自车车速
  map_var_t obj_v;
  /// 是否倒车模式
  bool is_backing;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    t_offset = 0.0F;
    s_offset = 0.0F;
    x_offset = 0.0F;
    obj_half_length = 1.0F;
    obj_half_width = 1.0F;
    obj_v = 0.0F;
    is_backing = false;
  }

  /**
   * @brief 构造函数
   */
  CollisionTestOnPathObj() {
    Clear();
  }
};

/**
 * @struct CollisionRiskTestResult
 * @brief 对轨迹进行碰撞分析后，碰撞风险测试的结果
 */
struct CollisionTestOnPathResult {
  /**
   * @enum MAX_RISKY_OBJ_NUM
   * @brief 最大可能产生碰撞风险的障碍物的数量
   */
  enum { MAX_RISKY_OBJ_NUM = 16 };

  /// 障碍物信息
  struct ObjInfo {
    /// 是否切入目标轨迹
    bool cut_in;
    /// 与障碍物之间的最小横向距离
    map_var_t min_static_distance;
    /// 到障碍物的纵向距离(沿着轨迹)
    map_var_t dist_to_tar_obj;
    /// 障碍物的方位
    Int32_t obj_position;
    /// 障碍物运动方向
    Int32_t obj_direction;
    /// 障碍物在轨迹上的投影点
    common::PathPoint tar_obj_proj;
    /// 障碍物测试结果信息
    CollisionTestResult::ObjInfo test_ret;

    /**
     * @brief 清除内部数据
     */
    void Clear() {
      cut_in = false;
      min_static_distance = -1.0F;
      dist_to_tar_obj = 0.0F;
      obj_position = OBJ_POSITION_UNKNOWN;
      obj_direction = OBJ_DIRECTION_UNKNOWN;
      test_ret.Clear();
    }

    /**
     * @brief 构造函数
     */
    ObjInfo() {
      cut_in = false;
      min_static_distance = -1.0F;
      dist_to_tar_obj = 0.0F;
      obj_position = OBJ_POSITION_UNKNOWN;
      obj_direction = OBJ_DIRECTION_UNKNOWN;
    }
  };

  /// 有风险的障碍物列表
  common::StaticVector<ObjInfo, MAX_RISKY_OBJ_NUM> risky_obj_list;
  common::UnorderedMap<Int32_t, Int32_t, MAX_RISKY_OBJ_NUM>
      risky_obj_lookup_tab;

  /// 不确定的障碍物列表
  common::StaticVector<ObjInfo, MAX_RISKY_OBJ_NUM> uncertain_list;
  common::UnorderedMap<Int32_t, Int32_t, MAX_RISKY_OBJ_NUM>
      uncertain_lookup_tab;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    for (Int32_t i = 0; i < risky_obj_list.Size(); ++i) {
      risky_obj_list[i].Clear();
    }
    risky_obj_list.Clear();
    risky_obj_lookup_tab.Clear();

    for (Int32_t i = 0; i < uncertain_list.Size(); ++i) {
      uncertain_list[i].Clear();
    }
    uncertain_list.Clear();
    uncertain_lookup_tab.Clear();
  }

  /**
   * @brief 构造函数
   */
  CollisionTestOnPathResult() {
    Clear();
  }
};

/**
 * @struct VehicleStatus
 * @brief 当前车辆的一些状态信息（车速，等）
 */
struct VehicleStatus {
  /// 车速，单位：米/秒
  map_var_t velocity;
  /// 偏航角角速度
  map_var_t yaw_rate;
  /// 当前档位
  Int32_t gear;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    velocity = 0;
    yaw_rate = 0;
    gear = 0;
  }

  /**
   * @brief 构造函数
   */
  VehicleStatus() {
    Clear();
  }
};

/**
 * @struct DrivingMapInfo
 * @brief 内部驾驶地图的信息(调试用)
 */
struct DrivingMapInfo {
  /// Message head
  ad_msg::MsgHead msg_head;
  /// 车道上距离当前车辆位置最近的点
  common::PathPoint nearest_point_to_veh_on_lane;

  /// 内部地图信息
  MapInfo map;

  /**
   * @struct ReferenceLineInfo
   * @brief 参考线信息
   */
  struct ReferenceLineInfo {
    /// 参考线形点
    common::StaticVector<common::PathPoint,
        common::Path::kMaxPathPointNum> curve;
    /// 平滑后的参考线形点
    common::StaticVector<common::PathPoint,
        common::Path::kMaxPathPointNum> smooth_curve;

    /**
     * @brief 清除内部数据(参考线信息)
     */
    void Clear() {
      curve.Clear();
      smooth_curve.Clear();
    }

    /**
     * @brief 构造函数
     */
    ReferenceLineInfo() {
    }
  };
  /// 当前参考线的索引
  Int32_t current_reference_line_index;
  /// 所有参考线的集合
  common::StaticVector<ReferenceLineInfo,
      MAX_REFERENCE_LINE_NUM> reference_lines;

  /// 道路边界采样点
  common::StaticVector<RoadBoundary,
  common::Path::kMaxPathPointNum> road_boundary;

  /// 障碍物信息
  struct ObstacleInfo {
    /// 障碍物ID
    Int32_t id;
    /// 障碍物位置及包围盒
    common::OBBox2d obb;
    /// 障碍物朝向
    map_var_t heading;
    /// 障碍物类型
    Int8_t type;
    /// 是否是动态障碍物
    Int8_t dynamic;
    /// 是否应当忽略此障碍物（例如道路外的静态障碍物（人、车等例外），
    /// 车辆正后方的动态障碍物，车辆后方的静态障碍物等）
    bool ignore;
    /// 标记不确定的障碍物
    bool uncertain;
    /// 障碍物速度
    map_var_t v;
    /// 与参考线之间的关联信息
    /// 相对于参考线的路径长
    map_var_t s_ref;
    /// 相对于参考线的横向距离
    map_var_t l_ref;
    /// 保存对动态障碍物预测的轨迹
    common::StaticVector<common::StaticVector<common::TrajectoryPoint,
        MAX_OBJ_PRED_TRAJ_POINT_NUM>, MAX_OBJ_PRED_TRAJ_NUM> pred_trajectory;

    /**
     * @brief 清除障碍物预测轨迹
     */
    void ClearPredTrj() {
      for (Int32_t i = 0; i < pred_trajectory.Size(); ++i) {
        pred_trajectory[i].Clear();
      }
      pred_trajectory.Clear();
    }

    /**
     * @brief 清除内部数据(障碍物信息)
     */
    void Clear() {
      id = -1;
      obb.Clear();
      heading = 0;
      ignore = false;
      uncertain = false;
      v = 0;
      s_ref = 0;
      l_ref = 0;
    }

    /**
     * @brief 构造函数
     */
    ObstacleInfo() {
      Clear();
    }
  };
  /// 障碍物列表
  common::StaticVector<ObstacleInfo, MAX_OBJECT_NUM_IN_OBJ_SPACE> obstacle_list;
  /// 风险区域分析结果(有风险的障碍物信息)
  common::StaticVector<CollisionTestResult::ObjInfo,
      MAX_OBJECT_NUM_IN_OBJ_SPACE> risky_obj_list;
  common::StaticVector<CollisionTestResult::ObjInfo,
      MAX_OBJECT_NUM_IN_OBJ_SPACE> uncertain_list;

  /// 跟车目标信息
  struct FollowingTarget {
    /// 此信息是否有效
    bool valid;
    /// 跟车目标的位置x坐标
    Float32_t obj_x;
    /// 跟车目标的位置y坐标
    Float32_t obj_y;

    /**
     * @brief 清除内部数据(障碍物信息)
     */
    void Clear() {
      valid = false;
      obj_x = 0.0F;
      obj_y = 0.0F;
    }

    /**
     * @brief 构造函数
     */
    FollowingTarget() {
      Clear();
    }
  } following_target;

  /// Planning Storys
  ad_msg::PlanningStoryList planning_storys;

  /**
   * @brief 清除内部数据(内部驾驶地图的信息)
   */
  void Clear() {
    nearest_point_to_veh_on_lane.Clear();
    map.Clear();

    current_reference_line_index = -1;
    for (Int32_t i = 0; i < reference_lines.Size(); ++i) {
      reference_lines[i].Clear();
    }
    reference_lines.Clear();

    road_boundary.Clear();

    for (Int32_t i = 0; i < obstacle_list.Size(); ++i) {
      obstacle_list[i].Clear();
    }
    obstacle_list.Clear();

    for (Int32_t i = 0; i < risky_obj_list.Size(); ++i) {
      risky_obj_list[i].Clear();
    }
    risky_obj_list.Clear();
    for (Int32_t i = 0; i < uncertain_list.Size(); ++i) {
      uncertain_list[i].Clear();
    }
    uncertain_list.Clear();

    following_target.Clear();

    planning_storys.Clear();
  }

  /**
   * @brief 构造函数
   */
  DrivingMapInfo() {
    Clear();
  }
};


}  // namespace driv_map
}  // namespace phoenix

#endif  // PHOENIX_DRIVING_MAP_DRIVING_MAP_H_

