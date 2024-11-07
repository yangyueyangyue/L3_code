//
#include "utils/com_clock_c.h"


#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#include <sys/time.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include <windows.h>
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#include <time.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#include "common_func.h"
#include <sys/time.h>
#include <time.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
#endif  // PROJECT_PLATFORM

#if USING_USER_CLOCK
  #if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
      (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  #include <pthread.h>
  #elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  #include <windows.h>
  #elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
    #ifdef __cplusplus
    extern "C" {
    #endif  // __cplusplus
    #include <INTEGRITY.h>
    #ifdef __cplusplus
    }
    #endif  // __cplusplus
  #elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  #include "kotei_vehicle_platform_data_api.h"
  #endif  // PROJECT_PLATFORM
#endif  // USING_USER_CLOCK


#if USING_USER_CLOCK
enum { USER_CLOCK_US_MODULO = 200 * 1000 * 1000 };
enum { USER_CLOCK_MS_MODULO = USER_CLOCK_US_MODULO / 1000 };
static Int64_t s_user_clock_us = 0;


#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
static pthread_rwlock_t s_user_clock_mutex;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
static CRITICAL_SECTION s_user_clock_critical;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
static Semaphore s_user_clock_mutex;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
static Int32_t s_user_clock_mutex_fake = 0;
#else
  ERROR: Locking function has not been defined.
#endif


static void InitUserClockMutex() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_init(&s_user_clock_mutex, Nullptr_t);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  InitializeCriticalSection(&s_user_clock_critical);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  CreateBinarySemaphore(&s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  s_user_clock_mutex_fake = 0;
#else
  ERROR: Lock function has not been defined.
#endif
}

static void LockWritingUserClock() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_wrlock(&s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  EnterCriticalSection(&s_user_clock_critical);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  WaitForSemaphore(s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  s_user_clock_mutex_fake = 0;
#else
  ERROR: Locking function has not been defined.
#endif
}

static void LockReadingUserClock() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_rdlock(&s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  EnterCriticalSection(&s_user_clock_critical);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  WaitForSemaphore(s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  s_user_clock_mutex_fake = 0;
#else
  Warning: Locking function has not been defined.
#endif
}

static void UnlockRwUserClock() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  pthread_rwlock_unlock(&s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  LeaveCriticalSection(&s_user_clock_critical);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  ReleaseSemaphore(s_user_clock_mutex);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  s_user_clock_mutex_fake = 0;
#else
  ERROR: Locking function has not been defined.
#endif
}

#endif // USING_USER_CLOCK


#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
// 定义时间信息
struct timeval {
  Int32_t tv_sec, tv_usec;
};

// 读取本地时间
static Int32_t gettimeofday(struct timeval *tv, void* tz) {
#define EPOCHFILETIME (116444736000000000ULL)
  FILETIME ft;
  LARGE_INTEGER li;
  Uint64_t tt;

  GetSystemTimeAsFileTime(&ft);
  li.LowPart = ft.dwLowDateTime;
  li.HighPart = ft.dwHighDateTime;
  tt = (li.QuadPart - EPOCHFILETIME) / 10;
  tv->tv_sec = tt / 1000000;
  tv->tv_usec = tt % 1000000;

  return 0;
}
#endif


void Phoenix_Common_InitializeUserClock() {
#if USING_USER_CLOCK
  InitUserClockMutex();

  LockWritingUserClock();

  s_user_clock_us = 0;

  UnlockRwUserClock();
#endif
}

void Phoenix_Common_UpdateUserClockUs(Int64_t us) {
#if USING_USER_CLOCK
  LockWritingUserClock();

  s_user_clock_us += us;
  if (s_user_clock_us >= USER_CLOCK_US_MODULO) {
    s_user_clock_us -= USER_CLOCK_US_MODULO;
  }

  UnlockRwUserClock();
#endif
}

Int64_t Phoenix_Common_GetClockNowUs() {
#if USING_USER_CLOCK
  Int64_t clock = 0;

  LockReadingUserClock();

  clock = s_user_clock_us;

  UnlockRwUserClock();

  return (clock);
#else
  #if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
      (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
      (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  // TODO: temporary impementation - it might be too slow.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (Int64_t)((tv.tv_sec) * 1000000 + tv.tv_usec);
  #elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  return (static_cast<Int64_t>(GetTimestampUSec64bit()));
  #elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  return ((Int64_t)get_systimevalue());
  #endif
#endif
}

Int64_t Phoenix_Common_GetClockNowMs() {
  return Phoenix_Common_GetClockNowUs() / 1000;
}

Int64_t Phoenix_Common_CalcElapsedClockUs(Int64_t prev, Int64_t next) {
#if USING_USER_CLOCK
  Int64_t diff = 0;
  Int64_t diff_2 = 0;
  if (next >= prev) {
    diff = next - prev;
    diff_2 = USER_CLOCK_US_MODULO - next + prev;
    if (diff_2 < diff) {
      diff = -diff_2;
    }
  } else {
    diff = USER_CLOCK_US_MODULO - prev + next;
    diff_2 = prev - next;
    if (diff_2 < diff) {
      diff = -diff_2;
    }
  }

  return (diff);
#else
  return (next - prev);
#endif
}

Int64_t Phoenix_Common_CalcElapsedClockMs(Int64_t prev, Int64_t next) {
#if USING_USER_CLOCK
  Int64_t diff = 0;
  Int64_t diff_2 = 0;
  if (next >= prev) {
    diff = next - prev;
    diff_2 = USER_CLOCK_MS_MODULO - next + prev;
    if (diff_2 < diff) {
      diff = -diff_2;
    }
  } else {
    diff = USER_CLOCK_MS_MODULO - prev + next;
    diff_2 = prev - next;
    if (diff_2 < diff) {
      diff = -diff_2;
    }
  }

  return (diff);
#else
  return (next - prev);
#endif
}


/// 系统时间
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  #include <sys/time.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  #include <windows.h>
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  #include <time.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  #include "common_func.h"
  #include <sys/time.h>
  #include <time.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  #include "kotei_vehicle_platform_data_api.h"
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
// 定义时间信息
struct timeval {
  Int32_t tv_sec, tv_usec;
};

// 读取本地时间
static Int32_t gettimeofday(struct timeval *tv, void* tz) {
#define EPOCHFILETIME (116444736000000000ULL)
  FILETIME ft;
  LARGE_INTEGER li;
  Uint64_t tt;

  GetSystemTimeAsFileTime(&ft);
  li.LowPart = ft.dwLowDateTime;
  li.HighPart = ft.dwHighDateTime;
  tt = (li.QuadPart - EPOCHFILETIME) / 10;
  tv->tv_sec = tt / 1000000;
  tv->tv_usec = tt % 1000000;

  return 0;
}
#endif

Int64_t Phoenix_Common_GetSysClockNowUs() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  // TODO: temporary impementation - it might be too slow.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (Int64_t)(tv.tv_sec) * 1000000 + tv.tv_usec;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  return ((Int64_t)(GetTimestampUSec64bit()));
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  return ((Int64_t)(get_systimevalue()));
#else
  Error: Can not get timestamp from system.;
#endif
}

Int64_t Phoenix_Common_GetSysClockNowMs() {
  return Phoenix_Common_GetSysClockNowUs() / 1000;
}

Int64_t Phoenix_Common_CalcElapsedSysClockUs(Int64_t prev, Int64_t next) {
  return (next - prev);
}

Int64_t Phoenix_Common_CalcElapsedSysClockMs(Int64_t prev, Int64_t next) {
  return (next - prev);
}

