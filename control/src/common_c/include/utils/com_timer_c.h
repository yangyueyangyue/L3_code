//
#ifndef PHOENIX_COMMON_COM_TIMER_C_H_
#define PHOENIX_COMMON_COM_TIMER_C_H_

#include "utils/com_utils_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/// TODO: 若使用自定义的时钟，则不能超过其最大允许的计时范围 (USER_CLOCK_US_MODULO)
enum { MAX_USER_DATA_NUM_IN_COM_TIMER = 4 };
/**
 * @struct ComTimer_t
 * @brief 定时器
 */
typedef struct _ComTimer_t ComTimer_t;
struct _ComTimer_t {
  Int8_t active_flag;
  Int64_t settings_timeout;
  Int64_t start_timestamp;

  struct {
    Int32_t int32_value[MAX_USER_DATA_NUM_IN_COM_TIMER];
    Float32_t float32_value[MAX_USER_DATA_NUM_IN_COM_TIMER];
  } user_data;
};

void Phoenix_Com_Timer_Init(ComTimer_t* ins, Int64_t timeout_ms);

void Phoenix_Com_Timer_SetTimeout(ComTimer_t* ins, Int64_t timeout_ms);

Int8_t Phoenix_Com_Timer_IsActive(const ComTimer_t* const ins);

Int64_t Phoenix_Com_Timer_Elapsed(const ComTimer_t* const ins);

void Phoenix_Com_Timer_Restart(ComTimer_t* ins);

void Phoenix_Com_Timer_Stop(ComTimer_t* ins);

Int8_t Phoenix_Com_Timer_Update(ComTimer_t* ins);

Int8_t Phoenix_Com_Timer_UpdateWithStamp(ComTimer_t* ins, Int64_t timestamp_ms);

void Phoenix_Com_Timer_SetUserDataInt32(ComTimer_t* ins, Int32_t idx, Int32_t value);

void Phoenix_Com_Timer_SetUserDataFloat32(ComTimer_t* ins, Int32_t idx, Float32_t value);

Int32_t Phoenix_Com_Timer_GetUserDataInt32(const ComTimer_t* const ins, Int32_t idx);

Float32_t Phoenix_Com_Timer_GetUserDataFloat32(const ComTimer_t* const ins, Int32_t idx);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_COMMON_COM_TIMER_C_H_

