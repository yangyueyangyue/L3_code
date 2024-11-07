/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       log.h
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

#ifndef PHOENIX_COMMON_LOG_H_
#define PHOENIX_COMMON_LOG_H_


#include "utils/macros.h"
#include "utils/com_utils.h"

/// 是否允许使用标准输入输出库
#define USING_STD_IO (1)
/// 是否允许使用GLOG库
#define USING_GLOG (1)

#if USING_STD_IO
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#endif  // USING_STD_IO

#if USING_GLOG
#include "glog/logging.h"
#include "glog/raw_logging.h"
#endif  // USING_GLOG


/// 输出跟踪日志（只有当lenvel <= 指定的等级时，才会输出此日志）
#define LOG_TRACE(level) \
  phoenix::common::LogMessage(level).Stream()
/// 输出一般性日志（只有当lenvel <= 指定的等级时，才会输出此日志）
#define LOG_INFO(level) \
  phoenix::common::LogMessage(__FILE__, __LINE__, \
    phoenix::common::LogMessage::LOG_TYPE_INFO, level).Stream()
/// 输出警告性日志
#define LOG_WARN \
  phoenix::common::LogMessage(__FILE__, __LINE__, \
    phoenix::common::LogMessage::LOG_TYPE_WARN, 0).Stream()
/// 输出错误性日志
#define LOG_ERR \
  phoenix::common::LogMessage(__FILE__, __LINE__, \
    phoenix::common::LogMessage::LOG_TYPE_ERROR, 0).Stream()
/// 若表达式cond为真则通过测试，否则将抛出异常(警告：程序将异常退出)
#define COM_CHECK(cond) \
  if (!(cond)) {        \
    phoenix::common::OutputFatal(__FILE__, __LINE__, "CHECK: " #cond) ;   \
  }


namespace phoenix {
namespace common {

/**
 * @brief 初始化日志功能
 */
void InitializeLogging();

/**
 * @brief 配置日志选项
 * @param[in] log_info_level 一般性日志输出等级
 *            （对于跟踪日志及一般性日志，小于等于这个等级的才输出）
 * @param[in] enable_output_time_prefix 是否输出时间信息
 * @param[in] enable_output_file_prefix 是否输出文件名称
 * @param[in] enable_output_to_console 是否输出到控制台
 * @param[in] enable_output_to_ring_buff 是否输出到环形缓冲区
 * @param[in] enable_output_to_file 是否输出到文件
 */
void ConfigLogging(
    Int32_t log_info_level,
    bool enable_output_time_prefix,
    bool enable_output_file_prefix,
    bool enable_output_to_console,
    bool enable_output_to_ring_buff,
    bool enable_output_to_file);

/**
 * @brief 从环形缓冲区获取日志
 * @param[in] size 需要读取的日志的最大字节数
 * @param[in] log 存储读取的日志的内存的指针
 * @return 实际读取的日志的字节数
 */
Int32_t GetLogFromRingBuff(const Int32_t size, Char_t* log);

/**
 * @brief 输出严重错误的信息，并抛出异常（警告：程序将异常退出）
 * @param[in] file 出错的文件名称
 * @param[in] line 出错的行号
 * @param[in] info 错误信息
 */
void OutputFatal(const Char_t* file, Int32_t line, const Char_t* info);


/**
 * @class LogStream
 * @brief 日志流，定义字符流输入到缓冲区功能
 */
class LogStream {
public:
  /**
   * @enum MAX_BUFF_SIZE
   * @brief 最大字符流缓冲区的尺寸
   */
  enum { MAX_BUFF_SIZE = 256 };

  /**
   * @brief 构造函数
   */
  LogStream();

  /**
   * @brief 获取缓冲区保存的字符流的数量
   * @return 缓冲区保存的字符流的数量
   */
  Int32_t Size() { return count_; }

  /**
   * @brief 获取缓冲区保存的字符流
   * @return 缓冲区保存的字符流的指针
   */
  const Char_t* Str() const { return str_buff_; }

  /**
   * @brief 将字符串型数据输入到缓冲区
   * @param[in] 字符串型数据
   * @return 日志流的引用
   */
  LogStream& operator << (const Char_t* data);

  /**
   * @brief 将字符型数据输入到缓冲区
   * @param[in] 字符型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Char_t data);

  /**
   * @brief 将Int32_t型数据输入到缓冲区
   * @param[in] Int32_t型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Int32_t data);

  /**
   * @brief 将Uint32_t型数据输入到缓冲区
   * @param[in] Uint32_t型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Uint32_t data);

  /**
   * @brief 将int64_t型数据输入到缓冲区
   * @param[in] int64_t型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Int64_t data);

  /**
   * @brief 将uint64_t型数据输入到缓冲区
   * @param[in] uint64_t型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Uint64_t data);

  /**
   * @brief 将float32_t型数据输入到缓冲区
   * @param[in] float32_t型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Float32_t data);

  /**
   * @brief 将float64_t型数据输入到缓冲区
   * @param[in] float64_t型数据
   * @return 日志流的引用
   */
  LogStream& operator << (Float64_t data);

private:
  // 缓冲区中保存的字符的数量
  Int32_t count_;
  // 字符流缓冲区
  Char_t str_buff_[MAX_BUFF_SIZE];
};


/**
 * @class LogMessage
 * @brief 日志信息，定义日志信息的输出功能
 */
class LogMessage {
public:
  /**
   * @enum
   * @brief 日志种类
   */
  enum {
    LOG_TYPE_TRACE = 0,         /// 跟踪型日志
    LOG_TYPE_INFO,              /// 一般信息型日志
    LOG_TYPE_WARN,              /// 警告型日志
    LOG_TYPE_ERROR,             /// 错误型日志
    LOG_TYPE_FATAL              /// 严重错误型日志
  };

  /**
   * @brief 设置是否运行输出时间信息
   * @param[in] enable true-允许，false-不允许
   */
  static void EnableOutputTimePrefex(bool enable) {
    s_output_time_prefix = enable;
  }

  /**
   * @brief 设置是否运行输出文件名称
   * @param[in] enable true-允许，false-不允许
   */
  static void EnableOutputFilePrefex(bool enable) {
    s_output_file_prefix = enable;
  }

  /**
   * @brief 设置一般性日志输出等级（对于跟踪日志及一般性日志，小于等于这个等级的才输出）
   * @param[in] level 一般性日志输出等级
   */
  static void SetLogOutputLevel(Int32_t level) {
    s_log_output_level = level;
  }

  /**
   * @brief 构造函数
   * @param[in] level 当前一般性日志的等级
   */
  LogMessage(Int32_t level);

  /**
   * @brief 构造函数
   * @param[in] file 调用此输出日志函数的文件名称
   * @param[in] line 调用此输出日志函数的行号
   * @param[in] type 此输出日志的种类
   * @param[in] level 此输出日志的等级（对于一般性日志及跟踪型日志有效）
   */
  LogMessage(const Char_t* file, Int32_t line, Int32_t type, Int32_t level);

  /**
   * @brief 析构函数
   */
  ~LogMessage();

  /**
   * @brief 获取日志流
   * @return 日志流的引用
   */
  LogStream& Stream() { return (log_stream_); }

  /**
   * @brief 将日志输出
   */
  void Flush();

private:
  /*
   * @brief 初始化
   * @param[in] file 调用此输出日志函数的文件名称
   * @param[in] line 调用此输出日志函数的行号
   * @param[in] type 此输出日志的种类
   */
  void Init(const Char_t* file, Int32_t line, Int32_t type);

private:
  // 是否输出时间信息
  static bool s_output_time_prefix;
  // 是否输出文件名称
  static bool s_output_file_prefix;
  // 一般性日志输出等级（对于跟踪日志及一般性日志，小于等于这个等级的才输出）
  static Int32_t s_log_output_level;
  // 是否已经将日志输出了
  bool has_been_flushed_;
  // 是否完成了初始化
  bool is_initialized_;
  // 日志流
  LogStream log_stream_;
};


/**
 * @class LogFile
 * @brief 日志文件
 */
class LogFile {
 public:
  /**
   * @brief 构造函数
   * @param[in] module_name 模块名称（在文件名称中体现）
   */
  LogFile(const Char_t* module_name);
  /**
   * @brief 析构函数
   */
  ~LogFile();

  /**
   * @brief 设置日志文件所在目录
   * @param[in] log_dir 目录名称
   */
  static void SetLogDir(const Char_t* log_dir);

  /**
   * @brief 读取是否允许输出到文件
   * @return true-允许，false-不允许
   */
  bool IsEnable(void) { return (enable_); }

  /**
   * @brief 设置是否允许输出到文件
   * @param[in] enable true-允许, false-不允许
   */
  void Enable(bool enable) {
    if (file_is_open_) {
      enable_ = enable;
    } else {
      enable_ = false;
    }
  }

  /**
   * @brief 打开文件
   * @return true-成功，false-失败
   */
  bool Open(void);

  /**
   * @brief 关闭文件
   * @return true-成功，false-失败
   */
  bool Close(void);

  /**
   * @brief 向文件写入信息
   * @param[in] str 需要写入到文件的信息
   */
  void Write(const Char_t* str);

 private:
  /*
   * @enum MAX_LOG_DIR_SIZE
   * @brief 最大目录的尺寸
   */
  enum { MAX_LOG_DIR_SIZE = 1024 };
  /*
   * @enum MAX_MODULE_NAME_SIZE
   * @brief 最大模块名称的尺寸
   */
  enum { MAX_MODULE_NAME_SIZE = 64 };
  /*
   * @enum MAX_FILE_NAME_SIZE
   * @brief 最大文件名称的尺寸
   */
  enum { MAX_FILE_NAME_SIZE = 1024 };

  // 保存目录信息
  static Char_t log_dir_[MAX_LOG_DIR_SIZE];
  // 是否允许输出到文件
  bool enable_;
  // 文件是否已经打开
  bool file_is_open_;
  // 指向文件的句柄
  void* file_;
  // 保存模块名称
  Char_t module_name_[MAX_MODULE_NAME_SIZE];
  // 保存文件名称
  Char_t file_name_[MAX_FILE_NAME_SIZE];
};


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_LOG_H_

