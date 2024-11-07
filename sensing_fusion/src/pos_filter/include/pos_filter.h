//
#ifndef PHOENIX_POS_FILTER_POS_FILTER_H_
#define PHOENIX_POS_FILTER_POS_FILTER_H_

#include "utils/macros.h"
#include "ad_msg.h"


namespace phoenix {
namespace pos_filter {


/**
 * @struct PosFilterDataSource
 * @brief 定位Filter源数据，用于Filter定位
 */
struct PosFilterDataSource {
  Int64_t timestamp;
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
