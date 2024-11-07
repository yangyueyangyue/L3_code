/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       com_utils.h
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

#ifndef PHOENIX_COMMON_COM_UTILS_H_
#define PHOENIX_COMMON_COM_UTILS_H_

#include <string.h>
#include "utils/macros.h"

/**
 * @brief 对格式化字符串函数的封装
 */
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#define com_snprintf std::snprintf
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#define com_snprintf _snprintf
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#define com_snprintf snprintf
#else
#define com_snprintf snprintf
#endif



namespace phoenix {
namespace common {


/**
 * @brief 对内存设置函数的封装
 * @param[in] s 待设置内存的起始地址
 * @param[in] c 需要设置的值
 * @param[in] n 待设置内存的大小
 * @return 待设置内存的起始地址
 */
inline void* com_memset(void* s, Int32_t c, Uint32_t n) {
  return (memset(s, c, n));
}

/**
 * @brief 对内存拷贝函数的封装
 * @param[in] dest 目的地址
 * @param[in] src 源地址
 * @param[in] n 需要拷贝的字节数
 * @return 目的地址
 */
inline void* com_memcpy(void* dest, const void* src, Uint32_t n) {
  return (memcpy(dest, src, n));
}

/**
 * @brief 字符串拷贝
 * @param[in] dest 目的地址
 * @param[in] src 源地址
 * @param[in] n 需要拷贝的字节数
 * @return 目的地址
 */
inline Char_t* com_strncpy(Char_t *dest, const Char_t *src, Uint32_t n) {
  return (strncpy(dest, src, n));
}


/**
 * @brief 初始化用户时钟变量为0
 */
void InitializeUserClock();

/**
 * @brief 增加用户时钟变量(us)
 */
void UpdateUserClockUs(Int64_t us);

/**
 * @brief 获取当前时间(us)
 * @return 当前时间(us)
 */
Int64_t GetClockNowUs();

/**
 * @brief 获取当前时间(ms)
 * @return 当前时间(ms)
 */
Int64_t GetClockNowMs();

/**
 * @brief 计算两个时间之间的时间差(us)
 * @param[in] prev 之前的时间(us)
 * @param[in] next 下一个时间(us)
 * @return 两个时间之间的时间差(us)
 */
Int64_t CalcElapsedClockUs(Int64_t prev, Int64_t next);

/**
 * @brief 计算两个时间之间的时间差(ms)
 * @param[in] prev 之前的时间(ms)
 * @param[in] next 下一个时间(ms)
 * @return 两个时间之间的时间差(ms)
 */
Int64_t CalcElapsedClockMs(Int64_t prev, Int64_t next);


/**
 * @struct RingCounter
 * @brief 环形计数器
 */
class RingCounter {
 public:
  /**
   * @brief 构造函数
   * @param[in] mod 模(计数器的最大值为mod-1) \n
   *            注意：mod的最大值应当距离Uint32_t可以表示的最大值保持一段距离
   */
  RingCounter(Uint32_t mod = 100000) : modulo_(mod) {
    count_ = 0;
  }

  /**
   * @brief 更新计数器
   * @param[in] step 计数器增加的值 \n
   *            注意：如果计数器当前值加上step大于Uint32_t可以表示的最大值 \n
   *                 将产生严重错误, 但程序内部不会做检查(为了效率)，调用者需要注意这一点。
   */
  void Update(Uint32_t step = 1) {
    count_ += step;
    if (count_ >= modulo_) {
      count_ -= modulo_;
    }
  }

  /**
   * @brief 获取当前计数器的值
   * @return 当前计数器的值
   */
  Uint32_t Current() {
    return (count_);
  }

  /**
   * @brief 计算当前计数器与之前计数器的值的差值
   * @param[in] prev 之前计数器的值
   * @return 当前计数器的值与之前计数器的值之间的差值
   */
  Uint32_t Elapsed(Uint32_t prev) {
    Uint32_t elap = 0;
    if (count_ >= prev) {
      elap = count_ - prev;
    } else {
      elap = modulo_ - prev + count_;
    }

    return (elap);
  }

 private:
  // mod 模(计数器的最大值为mod-1)
  Uint32_t modulo_;
  // 当前计数器的值
  Uint32_t count_;
};


/**
 * @struct Stopwatch
 * @brief 计时器
 */
class Stopwatch {
 public:
  /**
   * @brief 构造函数(构造时就会开始计时)
   */
  Stopwatch();

  /**
   * @brief 重新开始计时
   */
  void Restart();

  /**
   * @brief 获取已经运行了的时间(单位是毫秒)
   * @return 已经运行了的时间
   */
  Int64_t Elapsed();

 private:
  // 开始计时时的时间
  Int64_t start_clock_;
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_COM_UTILS_H_
