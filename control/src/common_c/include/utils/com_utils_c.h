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

#ifndef PHOENIX_COMMON_COM_UTILS_C_H_
#define PHOENIX_COMMON_COM_UTILS_C_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "utils/macros.h"


/**
 * @brief 对格式化字符串函数的封装
 */
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
#define phoenix_com_snprintf snprintf
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS)
#define phoenix_com_snprintf _snprintf
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#define phoenix_com_snprintf snprintf
#else
#define phoenix_com_snprintf snprintf
#endif


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 对内存设置函数的封装
 * @param[in] s 待设置内存的起始地址
 * @param[in] c 需要设置的值
 * @param[in] n 待设置内存的大小
 * @return 待设置内存的起始地址
 */
static void* phoenix_com_memset(void* s, Int32_t c, Uint32_t n) {
  return (memset(s, c, n));
}

/**
 * @brief 对内存拷贝函数的封装
 * @param[in] dest 目的地址
 * @param[in] src 源地址
 * @param[in] n 需要拷贝的字节数
 * @return 目的地址
 */
static void* phoenix_com_memcpy(void* dest, const void* src, Uint32_t n) {
  return (memcpy(dest, src, n));
}

/**
 * @brief 字符串拷贝
 * @param[in] dest 目的地址
 * @param[in] src 源地址
 * @param[in] n 需要拷贝的字节数
 * @return 目的地址
 */
static Char_t* phoenix_com_strncpy(Char_t *dest, const Char_t *src, Uint32_t n) {
  return (strncpy(dest, src, n));
}


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_COM_UTILS_C_H_
