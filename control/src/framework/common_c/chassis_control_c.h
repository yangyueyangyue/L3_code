//
#ifndef PHOENIX_FRAMEWORK_COMMON_CHASSIS_CONTROL_C_H_
#define PHOENIX_FRAMEWORK_COMMON_CHASSIS_CONTROL_C_H_

#include "os/mutex_c.h"
#include "ad_msg_c.h"
#include "lateral_control_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct ChassisControlConfig_t
 * @brief 底盘配置项
 */
typedef struct _ChassisControlConfig_t ChassisControlConfig_t;
struct _ChassisControlConfig_t {
  /// 方向盘角度偏置 (弧度，左正右负)
  Float32_t steering_wheel_angle_offset;
};


/**
 * @brief 清除内部数据
 */
void Phoenix_Common_ClearChassisControlConfig(ChassisControlConfig_t* const data);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_FRAMEWORK_COMMON_CHASSIS_CONTROL_C_H_

