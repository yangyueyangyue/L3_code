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


#define ENABLE_PUBLISH_PERCEPTION_DEBUG     (1)


#define DEBUG_FUSION_OBJ_CONFIDENCE   (1)


#define DEBUG_FUSION_Y_VALUE    (1)

/// 定义字长
#if (defined __x86_64__ && !defined __ILP32__) || \
  (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#define PROJECT_PLATFORM_WORDSIZE (64)
#else
# define PROJECT_PLATFORM_WORDSIZE (32)
#endif


//#define ENABLE_CAN_DEV_KVASER (1)
//#define ENABLE_CAN_DEV_EMUC2 (1)
//#define ENABLE_CAN_DEV_MDC (1)


/// 定义车辆平台的类型
#define VEHICLE_PLATFORM_UNDEFINED (0)    // 未定义平台
#define VEHICLE_PLATFORM_QIN_EV (1)
#define VEHICLE_PLATFORM_FT_AUMAN (2)
#define VEHICLE_PLATFORM_DF_X320 (3)
#define VEHICLE_PLATFORM_XD_EANT (4)
#define VEHICLE_PLATFORM_DF_D17 (5)
#define VEHICLE_PLATFORM VEHICLE_PLATFORM_DF_D17


/// 定义IMU设备的类型
#define DEV_IMU_TYPE_UNDEFINED (0)
#define DEV_IMU_TYPE_MPSK (1)
#define DEV_IMU_TYPE_BDSTAR (2)
#define DEV_IMU_TYPE_INTERTIAL_LAB (3)
#define DEV_IMU_TYPE DEV_IMU_TYPE_BDSTAR


/// 是否使用自定义的时钟(固定节拍计时)
#define USING_USER_CLOCK (0)

/// 使能通讯节点
#define ENABLE_ROS_NODE (1)
#define ENABLE_LCM_NODE (1)
#define ENABLE_UDP_NODE (1)


/// 使能SRR2的Tracker模式
#define ENABLE_SRR2_TRACKER_MODE (0)

/// 定义是否进入回放模
#define ENTER_PLAYBACK_MODE (0)


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
 *        使用宏定义静止class的拷贝构造函数和赋值构造函数（）
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

