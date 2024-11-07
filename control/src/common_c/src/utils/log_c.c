//
#include "utils/log_c.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "utils/macros.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#include <sys/time.h>
#include <time.h>
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
#endif

#include "utils/com_utils_c.h"
#include "os/mutex_c.h"


#define ENABLE_OUTPUT_LOG (1)
#define MAX_LOG_BUFF_SIZE (512)


// 是否允许将日志输出到控制台
static Int32_t s_enable_output_to_console = 1;
// 是否允许将日志输出到环形缓冲区
static Int32_t s_enable_output_to_ring_buff = 1;
// 是否允许将日志输出到文件
static Int32_t s_enable_output_to_file = 1;
// 日志输出最大等级
static Int32_t s_log_info_max_outptu_level = 5;

// 是否输出时间信息
static Int32_t s_enable_output_time_prefix = 1;
// 是否输出文件信息
static Int32_t s_enable_output_file_prefix = 1;

// 输出日志时用到的互斥锁
static CommonMutex_t s_log_mutex;


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

// 格式化输出当地日期与时间
static struct tm* localtime_r(const time_t* timep, struct tm* result) {
  localtime_s(result, timep);
  return result;
}
#endif

// 获取当前时间
static Int64_t CycleClock_Now() {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  // TODO: temporary impementation - it might be too slow.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (Int64_t)(tv.tv_sec) * 1000000 + tv.tv_usec;
#else 
  return (0);
#endif
}

// 获取当前时间
static Float64_t WallTime_Now() {
  // Now, cycle clock is retuning microseconds since the epoch.
  return CycleClock_Now() * 0.000001;
}

// 从带目录信息的完整文件名中截取文件名称
// Get the part of filepath after the last path separator.
// (Doesn't modify filepath, contrary to basename() in libgen.h.)
static const Char_t* GetConstBasename(const Char_t* filepath) {
  const Char_t* base = strrchr(filepath, '/');
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  // Look for either path separator in Windows
  if (!base)
    base = strrchr(filepath, '\\');
#endif
  return base ? (base+1) : filepath;
}

// 输出日志
static void OutputLog(const Char_t* log, Int32_t size) {
  // 共有资源需要加锁
  Phoenix_Common_Os_Mutex_Lock(&s_log_mutex);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (s_enable_output_to_console) {
    printf("%s", log);
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  if (s_enable_output_to_console) {
    printf(log);
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  if (s_enable_output_to_console) {
    // printf(log);
  }
#else
  // not defined
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (s_enable_output_to_ring_buff) {
    // Output to ring buffer
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to ring buffer.
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  // This platform do not support outputting log to ring buffer.
#else
  // not defined
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (s_enable_output_to_file) {
    // Output to file
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to file.
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  // This platform do not support outputting log to file.
#else
  // not defined
#endif

  // 离开时解锁
  Phoenix_Common_Os_Mutex_Unlock(&s_log_mutex);
}

void Phoenix_Common_InitializeLogging() {
  // 初始化互斥锁
  Phoenix_Common_Os_Mutex_Init(&s_log_mutex);
  // 初始化环形缓冲区
  // 打开日志文件
}

void Phoenix_Common_ConfigLogging(
    int log_info_level,
    char enable_output_time_prefix,
    char enable_output_file_prefix,
    char enable_output_to_console,
    char enable_output_to_ring_buff,
    char enable_output_to_file) {
  // 共有资源需要加锁
  Phoenix_Common_Os_Mutex_Lock(&s_log_mutex);

  s_log_info_max_outptu_level = log_info_level;
  s_enable_output_time_prefix = enable_output_time_prefix;
  s_enable_output_file_prefix = enable_output_file_prefix;

  s_enable_output_to_console = enable_output_to_console;
  s_enable_output_to_ring_buff = enable_output_to_ring_buff;
  s_enable_output_to_file = enable_output_to_file;

  if (s_enable_output_to_file) {
    // 打开日志文件
    // s_log_file_.Open();
    // s_log_file_.Enable(true);
  } else {
    // s_log_file_.Enable(false);
  }

  // 离开时解锁
  Phoenix_Common_Os_Mutex_Unlock(&s_log_mutex);
}

int Phoenix_Common_GetLogFromRingBuff(char* log, int size) {
  return 0;
}

void Phoenix_Common_OutputErrLog(const char* fmt, ... ) {
#if ENABLE_OUTPUT_LOG
  int idx = 0;
  va_list argptr;
  char log_buff[MAX_LOG_BUFF_SIZE];

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  Float64_t now;
  Int32_t usecs;
  time_t timestamp;
  struct tm tm_time;
#endif

  if ( Nullptr_t == fmt ) {
    return;
  }

  // phoenix_com_memset(log_buff, 0, sizeof(log_buff));
  if (s_enable_output_time_prefix) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    now = WallTime_Now();
    timestamp = (time_t)(now);
    localtime_r(&timestamp, &tm_time);
    usecs = (Int32_t)((now - timestamp) * 1000000);
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx,
                         "[ERRO] [%02d-%02d %02d:%02d:%02d.%06d] ",
                         1+tm_time.tm_mon,
                         tm_time.tm_mday,
                         tm_time.tm_hour,
                         tm_time.tm_min,
                         tm_time.tm_sec,
                         usecs);
    idx += 31;
#else
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[ERRO] ");
    idx += 7;
#endif
  } else {
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[ERRO] ");
    idx += 7;
  }

  va_start(argptr, fmt);
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  _vsnprintf(log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, fmt, argptr);
#else
  vsnprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, fmt, argptr);
#endif
  va_end(argptr);
  idx = strlen(log_buff);
  log_buff[idx++] = '\n';
  log_buff[idx++] = 0;

  OutputLog(log_buff, idx);
#endif
}

void Phoenix_Common_OutputWarnLog(const char* fmt, ... ) {
#if ENABLE_OUTPUT_LOG
  int idx = 0;
  va_list argptr;
  char log_buff[MAX_LOG_BUFF_SIZE];

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  Float64_t now;
  Int32_t usecs;
  time_t timestamp;
  struct tm tm_time;
#endif

  if ( Nullptr_t == fmt ) {
    return;
  }

  // phoenix_com_memset(log_buff, 0, sizeof(log_buff));
  if (s_enable_output_time_prefix) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    now = WallTime_Now();
    timestamp = (time_t)(now);
    localtime_r(&timestamp, &tm_time);
    usecs = (Int32_t)((now - timestamp) * 1000000);
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx,
                         "[WARN] [%02d-%02d %02d:%02d:%02d.%06d] ",
                         1+tm_time.tm_mon,
                         tm_time.tm_mday,
                         tm_time.tm_hour,
                         tm_time.tm_min,
                         tm_time.tm_sec,
                         usecs);
    idx += 31;
#else
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[WARN] ");
    idx += 7;
#endif
  } else {
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[WARN] ");
    idx += 7;
  }

  va_start(argptr, fmt);
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  _vsnprintf(log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, fmt, argptr);
#else
  vsnprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, fmt, argptr);
#endif
  va_end(argptr);
  idx = strlen(log_buff);
  log_buff[idx++] = '\n';
  log_buff[idx++] = 0;

  OutputLog(log_buff, idx);
#endif
}

void Phoenix_Common_OutputInfoLog(const char* fmt, ... ) {
#if ENABLE_OUTPUT_LOG
  int idx = 0;
  va_list argptr;
  char log_buff[MAX_LOG_BUFF_SIZE];

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  Float64_t now;
  Int32_t usecs;
  time_t timestamp;
  struct tm tm_time;
#endif

  if ( Nullptr_t == fmt ) {
    return;
  }

  // phoenix_com_memset(log_buff, 0, sizeof(log_buff));
  if (s_enable_output_time_prefix) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    now = WallTime_Now();
    timestamp = (time_t)(now);
    localtime_r(&timestamp, &tm_time);
    usecs = (Int32_t)((now - timestamp) * 1000000);
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx,
                         "[INFO] [%02d-%02d %02d:%02d:%02d.%06d] ",
                         1+tm_time.tm_mon,
                         tm_time.tm_mday,
                         tm_time.tm_hour,
                         tm_time.tm_min,
                         tm_time.tm_sec,
                         usecs);
    idx += 31;
#else
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[INFO] ");
    idx += 7;
#endif
  } else {
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[INFO] ");
    idx += 7;
  }

  va_start(argptr, fmt);
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  _vsnprintf(log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, fmt, argptr);
#else
  vsnprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, fmt, argptr);
#endif
  va_end(argptr);
  idx = strlen(log_buff);
  log_buff[idx++] = '\n';
  log_buff[idx++] = 0;

  OutputLog(log_buff, idx);
#endif
}

void Phoenix_Common_OutputFatalLog(
    const char* file, int line, const char* info) {
#if ENABLE_OUTPUT_LOG
  int idx = 0;
  char log_buff[MAX_LOG_BUFF_SIZE];

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  Float64_t now;
  Int32_t usecs;
  time_t timestamp;
  struct tm tm_time;
#endif

  // phoenix_com_memset(log_buff, 0, sizeof(log_buff));
  if (s_enable_output_time_prefix) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    now = WallTime_Now();
    timestamp = (time_t)(now);
    localtime_r(&timestamp, &tm_time);
    usecs = (Int32_t)((now - timestamp) * 1000000);
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx,
                         "[FATAL] [%02d-%02d %02d:%02d:%02d.%06d] ",
                         1+tm_time.tm_mon,
                         tm_time.tm_mday,
                         tm_time.tm_hour,
                         tm_time.tm_min,
                         tm_time.tm_sec,
                         usecs);
    idx += 32;
#else
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[FATAL] ");
    idx += 7;
#endif
  } else {
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx, "[FATAL] ");
    idx += 7;
  }

  if (Nullptr_t != file) {
    phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx,
                         "[%s:%d] ", GetConstBasename(file), line);
    idx = strlen(log_buff);
  }

  phoenix_com_snprintf(&log_buff[idx], MAX_LOG_BUFF_SIZE-2-idx,
                       "%s", info);
  idx = strlen(log_buff);
  log_buff[idx++] = '\n';
  log_buff[idx++] = 0;

  perror(log_buff);
  exit(EXIT_FAILURE);
#endif
}
