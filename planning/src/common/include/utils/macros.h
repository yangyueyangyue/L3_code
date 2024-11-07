/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       macros.h
 * @brief      共通宏定义
 * @details    定义整个工程共通的宏定义
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

#ifndef PHOENIX_COMMON_MACROS_H_
#define PHOENIX_COMMON_MACROS_H_


/// 切换运行平台
#define PROJECT_PLATFORM_UNDEFINED (0)    // 未定义平台
#define PROJECT_PLATFORM_LINUX (1)        // linux平台
#define PROJECT_PLATFORM_WINDOWS (2)      // windows平台
#define PROJECT_PLATFORM_RCAR_M3N (3)     // Rcar平台
#define PROJECT_PLATFORM_TC397_M3N (4)    // TC397平台
#define PROJECT_PLATFORM_MDC (5)          // MDC平台
#define PROJECT_PLATFORM PROJECT_PLATFORM_LINUX


/// 定义字长
#if (defined __x86_64__ && !defined __ILP32__) || \
  (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#define PROJECT_PLATFORM_WORDSIZE (64)
#else
# define PROJECT_PLATFORM_WORDSIZE (32)
#endif


/// 定义高精度地图的类型
#define HD_MAP_TYPE_UNDEFINED (0)
#define HD_MAP_TYPE_APOLLO (1)
#define HD_MAP_TYPE_D600 (2)
#define HD_MAP_TYPE_D17 (3)
#define HD_MAP_TYPE HD_MAP_TYPE_D17


/// 定义车辆平台的类型
#define VEHICLE_PLATFORM_UNDEFINED (0)    // 未定义平台
#define VEHICLE_PLATFORM_DF_D17_B1 (5)
#define VEHICLE_PLATFORM_DF_D17_B2 (6)
#define VEHICLE_PLATFORM VEHICLE_PLATFORM_DF_D17_B2

/// 定义是否进入回放模式
#define ENTER_PLAYBACK_MODE (0)

/// 定义是否进入回放模式(AD_HMI数据)
/// 订阅/adecu/path_pln 、 /adecu/spd_pln
#define ENTER_PLAYBACK_MODE_ADHMI (0)

/// 定义PCC实现开关
#define ENABLE_PACC (1)

/// 定义是否开启支持TBOX ADASISv2 地图
#define ENABLE_PACC_ADASIS (1)

/// 定义是否使能PCC长地图功能(~2000m)
#define ENABLE_PACC_LONG_MAP (1)

/// 定义是否进入ADASISV2 CAN回放模式
#define ENTER_PLAYBACK_MODE_ADASISV2 (0)

#define ENABLE_PACC_BACK2BACK_TEST (0)

/// 是否开启车速滤波
#define ENABLE_PCC_FILTER_VELOCITY (1)

/// 是否开启PCC输入坡度滤波
#define ENABLE_PCC_FILTER_TBOX_SLOPES (0)

/// 是否开启PCC输出油门开度滤波
#define ENABLE_PCC_FILTER_OUTPUT_THROTTLE (1)

/// 是否开启PCC上坡根据坡度模型限扭
#define ENABLE_PCC_SLOPE_MODEL_LIMIT_TORQUE (0)

/// 是否开启PCC阶梯目标巡航车速
#define ENABLE_PCC_STEP_TARGET_CC_SPEED (1)

/// 是否开启PCC输出油门开度上限
#define ENABLE_PCC_UPPER_LIMIT_THROTTLE (1)

/// 是否使用自定义的时钟(固定节拍计时)
#define USING_USER_CLOCK (1)


/// 使能通讯节点
#define ENABLE_ROS_NODE (1)
#define ENABLE_LCM_NODE (1)
#define ENABLE_UDP_NODE (1)


/**
 * @name c++11语法适配
 * @brief 定义部分c++11的语法，用于在不同的c++版本之间进行适配
 * @{
 */
#if __cplusplus < 201103L
  #define Nullptr_t NULL
#else
  #define Nullptr_t nullptr
#endif
/** @} c++11语法适配 */


/**
 * @name 基本数据类型定义
 * @brief 定义了基本数据类型，是为了遵守MISRA C++:2008编码规范 \n
 *        参考：MISRA C++:2008，Rule 3–9–2 (Advisory) \n
 *        typedefs that indicate size and signedness should be used in place
 *        of the basic numerical types.
 * @{
 */
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_LINUX) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_WINDOWS) || \
    (PROJECT_PLATFORM == PROJECT_PLATFORM_MDC)
  #include <stdint.h>
  typedef char          Char_t;
  typedef int8_t        Int8_t;
  typedef int16_t       Int16_t;
  typedef int32_t       Int32_t;
  typedef int64_t       Int64_t;
  typedef uint8_t       Uint8_t;
  typedef uint16_t      Uint16_t;
  typedef uint32_t      Uint32_t;
  typedef uint64_t      Uint64_t;
  typedef float         Float32_t;
  typedef double        Float64_t;
  typedef long double   Float128_t;
#elif (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N) || \
      (PROJECT_PLATFORM == PROJECT_PLATFORM_TC397_M3N)
  #include "UTYPEDEF.h"
  typedef char          Char_t;
  typedef sint8         Int8_t;
  typedef sint16        Int16_t;
  typedef sint32        Int32_t;
  typedef sint64        Int64_t;
  typedef uint8         Uint8_t;
  typedef uint16        Uint16_t;
  typedef uint32        Uint32_t;
  typedef uint64        Uint64_t;
  typedef float32       Float32_t;
  typedef float64       Float64_t;
  typedef long double   Float128_t;
#else
  typedef char                    Char_t;
  typedef signed char             Int8_t;
  typedef signed short int        Int16_t;
  typedef signed int              Int32_t;
  #if PROJECT_PLATFORM_WORDSIZE == 64
  typedef signed long int         Int64_t;
  #else
  typedef signed long long int    Int64_t;
  #endif
  typedef unsigned char           Uint8_t;
  typedef unsigned short int      Uint16_t;
  typedef unsigned int            Uint32_t;
  #if PROJECT_PLATFORM_WORDSIZE == 64
  typedef unsigned long           Uint64_t;
  #else
  typedef unsigned long long int  Uint64_t;
  #endif
  typedef float                   Float32_t;
  typedef double                  Float64_t;
  typedef long double             Float128_t;
#endif
/** @} 基本数据类型定义 */


/**
 * @name 定义单例模式
 * @brief 定义单例模式宏，用来简化单例模式类的定义过程
 * @{
 */
#define DISALLOW_COPY_AND_ASSIGN(classname) \
  private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);

#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
  private:                                         \
  classname();                                    \
  DISALLOW_COPY_AND_ASSIGN(classname);

#define DECLARE_SINGLETON(classname)        \
  public:                                    \
  static classname *instance() {            \
  static classname instance;              \
  return &instance;                       \
  }                                         \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
  private:
/** @} 定义单例模式 */


#endif  // PHOENIX_COMMON_MACROS_H_

