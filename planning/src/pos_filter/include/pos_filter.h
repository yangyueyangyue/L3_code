//
#ifndef PHOENIX_POS_FILTER_POS_FILTER_H_
#define PHOENIX_POS_FILTER_POS_FILTER_H_

#include "utils/macros.h"
#include "ad_msg.h"


namespace phoenix {
namespace pos_filter {


struct PosFilterConfig {
  struct {
    Float32_t x_offset;
    Float32_t y_offset;
    Float32_t h_offset;
  } cam_lane;

  PosFilterConfig() {
    cam_lane.x_offset = 0.0F;
    cam_lane.y_offset = 0.0F;
    cam_lane.h_offset = 0.0F;
  }
};

/**
 * @struct PosFilterDataSource
 * @brief 定位Filter源数据，用于Filter定位
 */
struct PosFilterDataSource {
  Int64_t timestamp;
  /* k004 pengc 2022-12-26 (begin) */
  /// 设置GNSS定位精度不可信任
  bool do_not_trust_gnss;
  /* k004 pengc 2022-12-26 (end) */
  /// 定位信息
  const ad_msg::Gnss* gnss;
  /// 车辆姿态信息
  const ad_msg::Imu* imu;
  /// 车身信息（通常是车身CAN信号）
  const ad_msg::Chassis* chassis;
  /// 感知模块识别的车道线信息（通常是摄像头识别的车道线）
  const ad_msg::LaneMarkCameraList* lane_mark_camera_list;

  /**
   * @brief 构造函数
   */
  PosFilterDataSource() {
    timestamp = 0;
    /* k004 pengc 2022-12-26 (begin) */
    /// 设置GNSS定位精度不可信任
    do_not_trust_gnss = false;
    /* k004 pengc 2022-12-26 (end) */
    gnss = Nullptr_t;
    imu = Nullptr_t;
    chassis = Nullptr_t;
    lane_mark_camera_list = Nullptr_t;
  }
};


struct PosFilterInfo {
  struct {
    Float32_t yaw_rate_steering;
    Float32_t yaw_rate_imu;
    Float32_t yaw_rate_chassis;
    Float32_t yaw_rate;
    Float32_t yaw_rate_chg_rate;
  } yaw_rate_info;

  void Clear() {
    common::com_memset(&yaw_rate_info, 0, sizeof(yaw_rate_info));
  }

  PosFilterInfo() {
    Clear();
  }
};


}  // namespace pos_filter
}  // namespace phoenix


#endif // PHOENIX_POS_FILTER_POS_FILTER_H_
