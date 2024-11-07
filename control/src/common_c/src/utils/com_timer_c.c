//
#include "utils/com_timer_c.h"

#include "utils/log_c.h"
#include "utils/com_clock_c.h"


void Phoenix_Com_Timer_Init(ComTimer_t* ins, Int64_t timeout_ms) {
  ins->active_flag = 0;
  ins->settings_timeout = timeout_ms;
  ins->start_timestamp = Phoenix_Common_GetClockNowMs();

  phoenix_com_memset(&(ins->user_data), 0, sizeof(ins->user_data));
}

void Phoenix_Com_Timer_SetTimeout(ComTimer_t* ins, Int64_t timeout_ms) {
  ins->settings_timeout = timeout_ms;
}

Int8_t Phoenix_Com_Timer_IsActive(const ComTimer_t* const ins) {
  return ins->active_flag;
}

Int64_t Phoenix_Com_Timer_Elapsed(const ComTimer_t* const ins) {
  return (Phoenix_Common_CalcElapsedClockMs(ins->start_timestamp, Phoenix_Common_GetClockNowMs()));
}

void Phoenix_Com_Timer_Restart(ComTimer_t* ins) {
  ins->active_flag = 1;

  ins->start_timestamp = Phoenix_Common_GetClockNowMs();
}

void Phoenix_Com_Timer_Stop(ComTimer_t* ins) {
  ins->active_flag = 0;
}

Int8_t Phoenix_Com_Timer_Update(ComTimer_t* ins) {
  return (Phoenix_Com_Timer_UpdateWithStamp(ins, Phoenix_Common_GetClockNowMs()));
}

Int8_t Phoenix_Com_Timer_UpdateWithStamp(ComTimer_t* ins, Int64_t timestamp_ms) {
  // printf("update timer, timestamp=%ld\n", timestamp_ms);
  Int8_t timeout_flag = 0;
  if (!ins->active_flag) {
    return (timeout_flag);
  }

  if (Phoenix_Common_CalcElapsedClockMs(ins->start_timestamp, timestamp_ms) >= ins->settings_timeout) {
    // NotifyTimeout();
    ins->active_flag = 0;
    timeout_flag = 1;
  }

  return (timeout_flag);
}

void Phoenix_Com_Timer_SetUserDataInt32(ComTimer_t* ins, Int32_t idx, Int32_t value) {
  Int8_t valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM_IN_COM_TIMER);
  COM_CHECK_C(valid_idx);
  if (valid_idx) {
    ins->user_data.int32_value[idx] = value;
  }
}

void Phoenix_Com_Timer_SetUserDataFloat32(ComTimer_t* ins, Int32_t idx, Float32_t value) {
  Int8_t valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM_IN_COM_TIMER);
  COM_CHECK_C(valid_idx);
  if (valid_idx) {
    ins->user_data.float32_value[idx] = value;
  }
}

Int32_t Phoenix_Com_Timer_GetUserDataInt32(const ComTimer_t* const ins, Int32_t idx) {
  Int8_t valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM_IN_COM_TIMER);
  COM_CHECK_C(valid_idx);
  if (valid_idx) {
    return (ins->user_data.int32_value[idx]);
  }
  return (0);
}

Float32_t Phoenix_Com_Timer_GetUserDataFloat32(const ComTimer_t* const ins, Int32_t idx) {
  Int8_t valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM_IN_COM_TIMER);
  COM_CHECK_C(valid_idx);
  if (valid_idx) {
    return (ins->user_data.float32_value[idx]);
  }
  return (0.0F);
}

