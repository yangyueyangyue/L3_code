//
#ifndef PHOENIX_COMMON_COM_CLOCK_C_H_
#define PHOENIX_COMMON_COM_CLOCK_C_H_

#include "utils/macros.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 初始化用户时钟变量为0
 */
void Phoenix_Common_InitializeUserClock();

/**
 * @brief 增加用户时钟变量(us)
 */
void Phoenix_Common_UpdateUserClockUs(Int64_t us);

/**
 * @brief 获取当前时间(us)
 * @return 当前时间(us)
 */
Int64_t Phoenix_Common_GetClockNowUs();

/**
 * @brief 获取当前时间(ms)
 * @return 当前时间(ms)
 */
Int64_t Phoenix_Common_GetClockNowMs();

/**
 * @brief 计算两个时间之间的时间差(us)
 * @param[in] prev 之前的时间(us)
 * @param[in] next 下一个时间(us)
 * @return 两个时间之间的时间差(us)
 */
Int64_t Phoenix_Common_CalcElapsedClockUs(Int64_t prev, Int64_t next);

/**
 * @brief 计算两个时间之间的时间差(ms)
 * @param[in] prev 之前的时间(ms)
 * @param[in] next 下一个时间(ms)
 * @return 两个时间之间的时间差(ms)
 */
Int64_t Phoenix_Common_CalcElapsedClockMs(Int64_t prev, Int64_t next);


/// 系统时间
/**
 * @brief 获取当前系统时间(us)
 * @return 当前系统时间(us)
 */
Int64_t Phoenix_Common_GetSysClockNowUs();

/**
 * @brief 获取当前系统时间(ms)
 * @return 当前系统时间(ms)
 */
Int64_t Phoenix_Common_GetSysClockNowMs();

/**
 * @brief 计算两个系统时间之间的时间差(us)
 * @param[in] prev 之前的系统时间(us)
 * @param[in] next 下一个系统时间(us)
 * @return 两个系统时间之间的时间差(us)
 */
Int64_t Phoenix_Common_CalcElapsedSysClockUs(Int64_t prev, Int64_t next);

/**
 * @brief 计算两个系统时间之间的时间差(ms)
 * @param[in] prev 之前的系统时间(ms)
 * @param[in] next 下一个系统时间(ms)
 * @return 两个系统时间之间的时间差(ms)
 */
Int64_t Phoenix_Common_CalcElapsedSysClockMs(Int64_t prev, Int64_t next);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_COMMON_COM_CLOCK_C_H_

