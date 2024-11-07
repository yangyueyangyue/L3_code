/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       com_utils.cc
 * @brief      共通函数
 * @details    定义了一些共通函数
 *
 * @author     pengc
 * @date       2020.05.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/15  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/


#include "utils/com_utils.h"
#include "utils/com_clock_c.h"
#include "utils/log.h"


namespace phoenix {
namespace common {


void InitializeUserClock() {
  Phoenix_Common_InitializeUserClock();
}

void UpdateUserClockUs(Int64_t us) {
  Phoenix_Common_UpdateUserClockUs(us);
}

// 获取当前时间(us)
Int64_t GetClockNowUs() {
  return Phoenix_Common_GetClockNowUs();
}

Int64_t GetClockNowMs() {
  return Phoenix_Common_GetClockNowMs();
}

Int64_t CalcElapsedClockUs(Int64_t prev, Int64_t next) {
  return Phoenix_Common_CalcElapsedClockUs(prev, next);
}

Int64_t CalcElapsedClockMs(Int64_t prev, Int64_t next) {
  return Phoenix_Common_CalcElapsedClockMs(prev, next);
}


/// class Stopwatch
///
#define USING_SYSTEM_TIME_FOR_STOPWATCH (1)

#if (USING_SYSTEM_TIME_FOR_STOPWATCH)
Stopwatch::Stopwatch() {
  start_clock_ = Phoenix_Common_GetSysClockNowMs();
}

void Stopwatch::Restart() {
  start_clock_ = Phoenix_Common_GetSysClockNowMs();
}

Int64_t Stopwatch::Elapsed() {
  // printf("From %ld to %ld, spend %ld\n",
  //        start_clock_, GetClockNowMsFromSys(),
  //        GetClockNowMsFromSys() - start_clock_);

  return (Phoenix_Common_GetSysClockNowMs() - start_clock_);
}

#else  // #if (USING_SYSTEM_TIME_FOR_STOPWATCH)

Stopwatch::Stopwatch() {
  start_clock_ = GetClockNowMs();
}

void Stopwatch::Restart() {
  start_clock_ = GetClockNowMs();
}

Int64_t Stopwatch::Elapsed() {
  return CalcElapsedClockMs(start_clock_, GetClockNowMs());
}

#endif  // #if (USING_SYSTEM_TIME_FOR_STOPWATCH)


}  // namespace common
}  // namespace phoenix
