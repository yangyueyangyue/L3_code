/**
 * @file SystemBase.h
 * @brief 
 * @author huic (sample@qq.com)
 * @version 1.0
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021  東風商用車技術中心
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2021-08-20 <td>1.0     <td>huic     <td>内容
 * </table>
 */

#ifndef SYSTEM_BASE_H
#define SYSTEM_BASE_H

#include <iostream>
#include <memory>
#include "KTSystemMacroCommon.h"

#if CHECK_PLATFORM_N_SIMULATE_RCAR
#include "UTYPEDEF.h"
typedef char int8;
typedef short int16;
typedef int int32;
typedef long long int64;

#define __UINT64_MAX__ 0xFFFFFFFFFFFFFFFF
#define __UINT32_MAX__ 0xFFFFFFFF
#define __UINT16_MAX__ 0xFFFF
#define __UINT8_MAX__  0xFF
#else
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

#if CHECK_PLATFORM_IMX6
typedef unsigned long long uint64;
typedef long long int int64;
#else
typedef unsigned long int uint64;
typedef long int int64;
#endif
typedef float float32;
typedef double float64;

typedef char int8;
typedef short int16;
typedef int int32;

#endif

#define SAFE_DELETE(A) \
    {                  \
        if (A != NULL) \
            delete A;  \
        A = NULL;      \
    }

#define SAFE_DELETE_ARRAY(A) \
    {                        \
        if (A != NULL)       \
            delete[] A;      \
        A = NULL;            \
    }

#define SAFE_CREATE_ARRAY(A, B, C) \
    {                              \
        if (A != NULL)             \
            delete[] A;            \
        A = new B[C];              \
    }
#define SAFE_CHECK(A) (A != NULL)
#define SAFE_CREATE(A, B) \
    {                     \
        if (A != NULL)    \
            delete A;     \
        A = new B();      \
    }

#define RE_FAIL -1
#define RE_SUC 0

#define IF_NULL_RETURN(VAR) \
    if (NULL == VAR)        \
    {                       \
        return;             \
    }

#define IF_NULL_CONTINUE(VAR) \
    if (NULL == VAR)        \
    {                       \
        continue;             \
    }

#endif