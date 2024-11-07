//
#ifndef PHOENIX_FRAMEWORK_COMMON_MESSAGE_C_H_
#define PHOENIX_FRAMEWORK_COMMON_MESSAGE_C_H_

#include "os/mutex_c.h"
#include "ad_msg_c.h"
#include "lateral_control_c.h"
#include "common_c/chassis_control_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @struct ControlConfig_t
 * @brief 模块配置项
 */
typedef struct _ControlConfig_t ControlConfig_t;
struct _ControlConfig_t {
  ChassisControlConfig_t chassis_control_config;
};


/**
 * @brief 清除内部数据
 */
void Phoenix_Common_ClearControlConfig(ControlConfig_t* const data);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_FRAMEWORK_COMMON_MESSAGE_C_H_

