/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_common.h
 * @brief      定位消息定义
 * @details    定义了定位消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_LOCALIZATION_C_H_
#define PHOENIX_AD_MSG_MSG_LOCALIZATION_C_H_

#include "utils/macros.h"
#include "utils/com_utils_c.h"
#include "msg_common_c.h"


#ifdef __cplusplus
extern "C" {
#endif


enum {
  /// Invalid
  GNSS_STATUS_INVALID = 0,
  /// Do NOT use. Bad solution due to insufficient observations
  GNSS_STATUS_BAD,
  /// Use with caution. The pose may be unavailable or incorrect.
  GNSS_STATUS_CONVERGING,
  /// Safe to use. The INS has fully converged.
  GNSS_STATUS_GOOD
};


/**
 * @struct Gnss
 * @brief GNSS信息
 */
typedef struct _Gnss_t Gnss_t;
struct _Gnss_t {
  /// 消息头
  MsgHead_t msg_head;
  /// 纬度（-180～180）
  Float64_t latitude;
  /// 经度（-180～180）
  Float64_t longitude;
  /// 高度，单位（米）
  Float64_t altitude;
  /// 偏航角（-PI～PI弧度）, 正东向为0度，逆时针为正方向(右手迪卡尔坐标系)
  Float32_t heading_gnss;
  /// utm 坐标X，单位（米）
  Float64_t x_utm;
  /// utm 坐标Y，单位（米）
  Float64_t y_utm;
  /// utm 坐标Z，单位（米）
  Float64_t z_utm;
  /// utm 偏航角（-PI～PI弧度）, utm_x轴为0度，逆时针为正方向(右手迪卡尔坐标系)
  Float32_t heading_utm;
  /// odom 坐标X，单位（米）
  Float64_t x_odom;
  /// odom 坐标Y，单位（米）
  Float64_t y_odom;
  /// odom 坐标Z，单位（米）
  Float64_t z_odom;
  /// odom 偏航角（-PI～PI弧度）, odom_x轴为0度，逆时针为正方向(右手迪卡尔坐标系)
  Float32_t heading_odom;
  /// 俯仰角（-PI/2～PI/2弧度）
  Float32_t pitch;
  /// 横滚角（-PI～PI弧度）
  Float32_t roll;
  /// 东向速度，单位（米/秒）
  Float32_t v_e;
  /// 北向速度，单位（米/秒）
  Float32_t v_n;
  /// 天向速度，单位（米/秒）
  Float32_t v_u;
  /// utm x轴方向速度，单位（米/秒）
  Float32_t v_x_utm;
  /// utm y轴方向速度，单位（米/秒）
  Float32_t v_y_utm;
  /// utm z轴方向速度，单位（米/秒）
  Float32_t v_z_utm;
  /// odom x轴方向速度，单位（米/秒）
  Float32_t v_x_odom;
  /// odom y轴方向速度，单位（米/秒）
  Float32_t v_y_odom;
  /// odom z轴方向速度，单位（米/秒）
  Float32_t v_z_odom;
  /// gnss 定位状态
  Int32_t gnss_status;
  /// utm 定位状态
  Int32_t utm_status;
  /// odom 定位状态
  Int32_t odom_status;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearGnss(Gnss_t* const data);

/**
 * @struct Imu
 * @brief 姿态信息
 */
typedef struct _Imu_t Imu_t;
struct _Imu_t {
  /// 消息头
  MsgHead_t msg_head;
  /// 航向角的角速度
  Float32_t yaw_rate;
  /// 俯仰角的角速度
  Float32_t pitch_rate;
  /// 横滚角的角速度
  Float32_t roll_rate;
  /// 沿着车身x轴的加速度
  Float32_t accel_x;
  /// 沿着车身y轴的加速度
  Float32_t accel_y;
  /// 沿着车身z轴的加速度
  Float32_t accel_z;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearImu(Imu_t* const data);

/**
 * @struct RelativePos
 * @brief 相对位置(当前车身坐标系下)
 */
typedef struct _RelativePos_t RelativePos_t;
struct _RelativePos_t {
  /// 相对时间 ms
  Int32_t relative_time;

  /// 相对位置 x坐标，单位（米）
  Float32_t x;
  /// 相对位置 y坐标，单位（米）
  Float32_t y;
  /// 相对的航向角（-PI～PI弧度）
  Float32_t heading;
  /// 航向角的角速度，单位（弧度/秒）
  Float32_t yaw_rate;
  /// 航向角的角速度变化率，单位（弧度/秒^2）
  Float32_t yaw_rate_chg_rate;
  /// 航向角的角速度变化率的变化率，单位（弧度/秒^3）
  Float32_t yaw_rate_d_chg_rate;
  /// 速度，单位（米/秒）
  Float32_t v;
};

/**
 * @brief 清除数据
 */
void Phoenix_AdMsg_ClearRelativePos(RelativePos_t* const data);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_AD_MSG_MSG_LOCALIZATION_C_H_


