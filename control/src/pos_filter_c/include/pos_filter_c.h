//
#ifndef PHOENIX_POS_FILTER_POS_FILTER_C_H_
#define PHOENIX_POS_FILTER_POS_FILTER_C_H_

#include "utils/macros.h"
#include "ad_msg_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct PosFilterDataSource
 * @brief 定位Filter源数据，用于Filter定位
 */
typedef struct _PosFilterDataSource_t PosFilterDataSource_t;
struct _PosFilterDataSource_t {
  /// 时间戳
  Int64_t timestamp;
  /// 车辆当前在目标轨迹中的相对位置
  struct {
    Int64_t timestamp;
    Float32_t x;
    Float32_t y;
    Float32_t heading;
  } rel_pos;
  /// 车辆姿态信息
  const Imu_t* imu;
  /// 车身信息（通常是车身CAN信号）
  const Chassis_t* chassis;
};


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_POS_FILTER_POS_FILTER_C_H_
