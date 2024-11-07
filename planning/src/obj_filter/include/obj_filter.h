//
#ifndef PHOENIX_OBJ_FILTER_OBJ_FILTER_H_
#define PHOENIX_OBJ_FILTER_OBJ_FILTER_H_

#include "utils/macros.h"
#include "ad_msg.h"
#include "driving_map_wrapper.h"


namespace phoenix {
namespace obj_filter {


struct ObjFilterConfig {
  /// 是否使用外部障碍物列表
  bool using_outside_obj_list;
  /// 激光雷达使用方法: 0 ~ 不使用, 1 ~ 总是使用, 2 ~ 特定路段使用
  Int32_t lidar_usage;

  ObjFilterConfig() {
    using_outside_obj_list = true;
    lidar_usage = 0;
  }
};

/**
 * @struct ObjFilterDataSource
 * @brief 障碍物Filter源数据，用于Filter障碍物
 */
struct ObjFilterDataSource {
  /// 时间戳
  Int64_t timestamp;
  /// 定位信息
  const ad_msg::RelativePosList* rel_pos_list;
  /// 车身信息（通常是车身CAN信号）
  const ad_msg::Chassis* chassis;
  /// 驾驶地图信息
  const driv_map::DrivingMapWrapper* driving_map;

  /// 传感器信息
  const ad_msg::ObstacleCameraList* obj_list_cam_0;
  const ad_msg::ObstacleRadarList* obj_list_radar_0;
  const ad_msg::ObstacleRadarList* obj_list_radar_1;
  const ad_msg::ObstacleRadarList* obj_list_radar_2;
  const ad_msg::ObstacleRadarList* obj_list_radar_3;
  const ad_msg::ObstacleRadarList* obj_list_radar_4;
  const ad_msg::ObstacleRadarList* obj_list_radar_5;
  const ad_msg::ObstacleLidarList* obj_list_lidar_0;
  const ad_msg::ObstacleLidarList* obj_list_lidar_1;


  /**
   * @brief 构造函数
   */
  ObjFilterDataSource() {
    timestamp = 0;

    rel_pos_list = Nullptr_t;
    chassis = Nullptr_t;
    driving_map = Nullptr_t;

    obj_list_cam_0 = Nullptr_t;
    obj_list_radar_0 = Nullptr_t;
    obj_list_radar_1 = Nullptr_t;
    obj_list_radar_2 = Nullptr_t;
    obj_list_radar_3 = Nullptr_t;
    obj_list_radar_4 = Nullptr_t;
    obj_list_radar_5 = Nullptr_t;
    obj_list_lidar_0 = Nullptr_t;
    obj_list_lidar_1 = Nullptr_t;
  }
};




}  // namespace obj_filter
}  // namespace phoenix


#endif // PHOENIX_OBJ_FILTER_OBJ_FILTER_H_
