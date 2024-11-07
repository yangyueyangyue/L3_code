/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       log.cc
 * @brief      调试日志
 * @details    定义了程序调试日志功能
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "utils/log.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#include <time.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <exception>
#include <stdexcept>
//#include <boost/thread.hpp>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#include <sys/time.h>
#include <time.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <exception>
#include <stdexcept>
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#include <sys/time.h>
#include <pthread.h>
#include <execinfo.h>
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#include <windows.h>
#endif

#if USING_GLOG
#include "glog/logging.h"
#include "glog/raw_logging.h"
#endif  // USING_GLOG

#include "container/string_ring_buffer.h"
#include "os/mutex.h"

namespace phoenix {
namespace common {


// 是否允许将日志输出到控制台
static bool s_enable_output_to_console = true;
// 是否允许将日志输出到环形缓冲区
static bool s_enable_output_to_ring_buff = true;
// 是否允许将日志输出到文件
static bool s_enable_output_to_file = true;

// 输出日志时用到的互斥锁
static os::Mutex s_log_mutex;
// 日志输出文件
static LogFile s_log_file_("planning");

// 日志输出环形缓冲区是否已经初始化
static bool s_log_ring_buff_init_flag = false;
// 日志输出环形缓冲区
common::StringRingBuf s_log_ring_buff;
// 日志输出环形缓冲区中的字符串存储区的最大尺寸
#define MAX_STRING_BUFF_SIZE_OF_RING_BUFF (1024 * 1024 * 4)
// 日志输出环形缓冲区中的字符串存储区
static Char_t s_string_buff_of_ring_buf[MAX_STRING_BUFF_SIZE_OF_RING_BUFF];


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
  // TODO: temporary impementation - it might be too slow.
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (Int64_t)(tv.tv_sec) * 1000000 + tv.tv_usec;
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

static Int32_t GetStackTrace(
    void** result, Int32_t max_depth, Int32_t skip_count) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  static const Int32_t kStackLength = 64;
  void * stack[kStackLength];
  Int32_t size = 0;

  size = backtrace(stack, kStackLength);
  skip_count++;  // we want to skip the current frame as well
  Int32_t result_count = size - skip_count;
  if (result_count < 0) {
    result_count = 0;
  }
  if (result_count > max_depth) {
    result_count = max_depth;
  }
  for (Int32_t i = 0; i < result_count; i++) {
    result[i] = stack[i + skip_count];
  }

  return (result_count);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
  if (max_depth > 64) {
    max_depth = 64;
  }
  skip_count++;  // we want to skip the current frame as well
  // This API is thread-safe (moreover it walks only the current thread).
  return CaptureStackBackTrace(skip_count, max_depth, result, NULL);
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This function has not been defined in platform R-Car-M3N
  return (0);
#else
  // This function has not been defined.
  return (0);
#endif
}

void InitializeLogging() {
  // 初始化环形缓冲区
  InitStringRingBuf(
        &s_log_ring_buff, s_string_buff_of_ring_buf,
        MAX_STRING_BUFF_SIZE_OF_RING_BUFF);
  s_log_ring_buff_init_flag = true;
}

// 配置日志输出选项
void ConfigLogging(
    Int32_t log_info_level,
    bool enable_output_time_prefix,
    bool enable_output_file_prefix,
    bool enable_output_to_console,
    bool enable_output_to_ring_buff,
    bool enable_output_to_file) {
  // 共有资源需要加锁
  s_log_mutex.Lock();

  LogMessage::SetLogOutputLevel(log_info_level);
  LogMessage::EnableOutputTimePrefex(enable_output_time_prefix);
  LogMessage::EnableOutputFilePrefex(enable_output_file_prefix);

  s_enable_output_to_console = enable_output_to_console;
  s_enable_output_to_ring_buff = enable_output_to_ring_buff;
  s_enable_output_to_file = enable_output_to_file;

  if (s_enable_output_to_file) {
    // 打开日志文件
    s_log_file_.Open();
    s_log_file_.Enable(true);
  } else {
    s_log_file_.Enable(false);
  }

  // 离开时解锁
  s_log_mutex.Unlock();
}

// 从环形缓冲区中读取日志
Int32_t GetLogFromRingBuff(const Int32_t size, Char_t* log) {
  Int32_t nRet = 0;

  // 共有资源需要加锁
  s_log_mutex.Lock();

  if (true == s_log_ring_buff_init_flag) {
    nRet = ReadFromStringRingBuf(
          &s_log_ring_buff, log, size);
  }

  // 离开时解锁
  s_log_mutex.Unlock();

  return (nRet);
}

// 输出日志到环形缓冲区
static void OutputLogToRingBuff(const Char_t* log, Int32_t size) {
  if (false == s_log_ring_buff_init_flag) {
    // 初始化环形缓冲区
    InitStringRingBuf(
          &s_log_ring_buff, s_string_buff_of_ring_buf,
          MAX_STRING_BUFF_SIZE_OF_RING_BUFF);
    s_log_ring_buff_init_flag = true;
  }
  WriteToStringRingBufOverride(
        &s_log_ring_buff, log, size);
}

// 输出日志
static void OutputLog(const Char_t* log, Int32_t size) {
  // 共有资源需要加锁
  s_log_mutex.Lock();

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (s_enable_output_to_console) {
    std::cout << log;
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to console.
  // (void)log;
  // (void)size;
  if (s_enable_output_to_console) {
    // std::cout << log;
    printf(log);
  }
#else
  // not defined
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (s_enable_output_to_ring_buff) {
    OutputLogToRingBuff(log, size);
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  if (s_enable_output_to_ring_buff) {
    OutputLogToRingBuff(log, size);
  }
#else
  // not defined
#endif

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (s_enable_output_to_file) {
    s_log_file_.Write(log);
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to file.
#else
  // not defined
#endif

  // 离开时解锁
  s_log_mutex.Unlock();
}

// 输出严重错误信息
void OutputFatal(const Char_t* file, Int32_t line, const Char_t* info) {
  LogMessage log_msg(file, line, LogMessage::LOG_TYPE_FATAL, 0);
  log_msg.Stream() << info;
  log_msg.Flush();

#if USING_GLOG
  LOG(FATAL) << "file[" << file << "] line[" << line << "] Info: " << info;
#else
  throw std::logic_error(log_msg.Stream().Str());
#endif
}


// 日志流
LogStream::LogStream() {
  count_ = 0;
  str_buff_[0] = '\0';
}

LogStream& LogStream::operator << (const Char_t* data) {
  Int32_t str_len = static_cast<Int32_t>(strlen(data));
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > str_len) {
    com_snprintf(&str_buff_[count_], static_cast<Uint32_t>(remanent_space), "%s", data);
    count_ += str_len;
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Char_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 1) {
    com_snprintf(&str_buff_[count_], static_cast<Uint32_t>(remanent_space), "%c", data);
    count_ += 1;
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Int32_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 12) {
    com_snprintf(&str_buff_[count_], static_cast<size_t>(remanent_space), "%d", data);
    count_ += static_cast<Int32_t>(strlen(&str_buff_[count_]));
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Uint32_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 12) {
    com_snprintf(&str_buff_[count_], static_cast<size_t>(remanent_space), "%u", data);
    count_ += strlen(&str_buff_[count_]);
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Int64_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 22) {
    com_snprintf(&str_buff_[count_], static_cast<Uint32_t>(remanent_space), "%ld", data);
    count_ += strlen(&str_buff_[count_]);
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Uint64_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 22) {
    com_snprintf(&str_buff_[count_], static_cast<size_t>(remanent_space), "%lu", data);
    count_ += strlen(&str_buff_[count_]);
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Float32_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 20) {
    com_snprintf(&str_buff_[count_], static_cast<size_t>(remanent_space), "%.3f", data);
    count_ += strlen(&str_buff_[count_]);
    str_buff_[count_] = '\0';
  }

  return (*this);
}

LogStream& LogStream::operator << (Float64_t data) {
  Int32_t remanent_space = static_cast<Int32_t>(MAX_BUFF_SIZE) - count_ - 1;

  if (remanent_space > 40) {
    com_snprintf(&str_buff_[count_], static_cast<size_t>(remanent_space), "%.6f", data);
    count_ += strlen(&str_buff_[count_]);
    str_buff_[count_] = '\0';
  }

  return (*this);
}


// 是否输出时间信息
bool LogMessage::s_output_time_prefix = true;
// 是否输出文件名
bool LogMessage::s_output_file_prefix = true;
// 一般性日志输出等级
Int32_t LogMessage::s_log_output_level = 5;

// 日志信息
LogMessage::LogMessage(Int32_t level) {
  has_been_flushed_ = false;

  if (level <= s_log_output_level) {
    Init(Nullptr_t, 0, static_cast<Int32_t>(LOG_TYPE_TRACE));
  }
}

LogMessage::LogMessage(const Char_t* file, Int32_t line,
                       Int32_t type, Int32_t level) {
  is_initialized_ = false;
  has_been_flushed_ = false;

  if ((LOG_TYPE_WARN == type) ||
      (LOG_TYPE_ERROR == type) ||
      (LOG_TYPE_FATAL == type) ||
      (level <= s_log_output_level)) {
    Init(file, line, type);
  }
}

LogMessage::~LogMessage() {
  Flush();
}

void LogMessage::Flush() {
  if (!has_been_flushed_ && is_initialized_) {
    log_stream_ << "\n";
    OutputLog(log_stream_.Str(), log_stream_.Size());
  }
  has_been_flushed_ = true;
}

void LogMessage::Init(const Char_t* file, Int32_t line, Int32_t type) {
  is_initialized_ = true;

  if (LOG_TYPE_WARN == type) {
    Stream() << "[WARN] ";
  } else if (LOG_TYPE_ERROR == type) {
    Stream() << "[ERRO] ";
  } else if (LOG_TYPE_FATAL == type) {
    Stream() << "[FATA] ";
  } else if (LOG_TYPE_TRACE == type) {
    Stream() << "[TRAC] ";
  } else {
    Stream() << "[INFO] ";
  }

  if (s_output_time_prefix) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    Float64_t now = WallTime_Now();
    time_t timestamp = static_cast<time_t>(now);
    struct ::tm tm_time;
    localtime_r(&timestamp, &tm_time);
    Int32_t usecs = static_cast<Int32_t>((now - timestamp) * 1000000);

    Char_t str_buff[32] = {'\0'};
    com_snprintf(str_buff, sizeof(str_buff), "[%02d-%02d %02d:%02d:%02d.%06d] ",
                 1+tm_time.tm_mon,
                 tm_time.tm_mday,
                 tm_time.tm_hour,
                 tm_time.tm_min,
                 tm_time.tm_sec,
                 usecs);
    Stream() << str_buff;
#endif
  }

  if (s_output_file_prefix) {
    if (Nullptr_t != file) {
      Stream() << "[" << GetConstBasename(file) << ':' << line << "] ";
    }
  }
}


// 日志文件的目录
Char_t LogFile::log_dir_[LogFile::MAX_LOG_DIR_SIZE] = { '\0' };

// 设置日志文件的目录
void LogFile::SetLogDir(const Char_t* log_dir) {
  com_snprintf(log_dir_, sizeof(log_dir_)-2, "%s", log_dir);
}

// 日志文件
LogFile::LogFile(const Char_t* module_name) {
  static Int32_t s_file_index = 1;

  if (Nullptr_t != module_name) {
    com_snprintf(module_name_, static_cast<Uint32_t>(MAX_MODULE_NAME_SIZE) - 1U,
                 "%s", module_name);
  } else {
    com_snprintf(module_name_, static_cast<Uint32_t>(MAX_MODULE_NAME_SIZE) - 1U,
                 "null_name_%d", s_file_index);
  }

  file_is_open_ = false;
  enable_ = false;
  s_file_index++;
}

LogFile::~LogFile() {
  Close();
}

bool LogFile::Open(void) {
  if (file_is_open_) {
    return (true);
  }

  Float64_t now = WallTime_Now();
  time_t timestamp = static_cast<time_t>(now);
  struct ::tm tm_time;
  localtime_r(&timestamp, &tm_time);
  Int32_t usecs = static_cast<Int32_t>((now - timestamp) * 1000000);

  com_snprintf(file_name_, static_cast<Uint32_t>(MAX_FILE_NAME_SIZE) - 1U,
               "%s/%s_%04d%02d%02d_%02d%02d%02d_%03d.log",
               log_dir_, module_name_,
               tm_time.tm_year+1900,
               1+tm_time.tm_mon,
               tm_time.tm_mday,
               tm_time.tm_hour,
               tm_time.tm_min,
               tm_time.tm_sec,
               usecs / 1000);

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  file_ = fopen(file_name_, "w+t");
  if (Nullptr_t == file_) {
    LOG_ERR << "Failed to open file \"" << file_name_ << "\" .";
    return (false);
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to file.
#else
  // not defined
#endif

  file_is_open_ = true;

  return (true);
}

bool LogFile::Close(void) {
  if (file_is_open_) {
    file_is_open_ = false;
    enable_ = false;

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
    if (Nullptr_t != file_) {
      fclose(static_cast<FILE*>(file_));
      file_ = Nullptr_t;
    }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to file.
#else
  // not defined
#endif
  }

  return (true);
}

void LogFile::Write(const Char_t* str) {
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  if (file_is_open_ && enable_ && Nullptr_t != str) {
    fputs(str, static_cast<FILE*>(file_));
  }
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
  // This platform do not support outputting log to file.
#else
  // not defined
#endif
}


}  // namespace common
}  // namespace phoenix




