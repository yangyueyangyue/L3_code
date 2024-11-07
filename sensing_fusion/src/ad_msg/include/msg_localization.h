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

#ifndef PHOENIX_AD_MSG_MSG_LOCALIZATION_H_
#define PHOENIX_AD_MSG_MSG_LOCALIZATION_H_

#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
#include <string>
#endif

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @struct Gnss
 * @brief GNSS信息
 */
struct Gnss {
  enum Status {
    /// Invalid
    STATUS_INVALID = 0,
    /// Do NOT use. Bad solution due to insufficient observations
    STATUS_BAD,
    /// Use with caution. The pose may be unavailable or incorrect.
    STATUS_CONVERGING,
    /// Safe to use. The INS has fully converged.
    STATUS_GOOD
  };

  /// 消息头
  MsgHead msg_head;
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

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    latitude = 0.0;
    longitude = 0.0;
    altitude = 0.0;
    heading_gnss = 0.0F;
    x_utm = 0.0;
    y_utm = 0.0;
    z_utm = 0.0;
    heading_utm = 0.0F;
    x_odom = 0.0;
    y_odom = 0.0;
    z_odom = 0.0;
    heading_odom = 0.0F;
    pitch = 0.0F;
    roll = 0.0F;
    v_e = 0.0F;
    v_n = 0.0F;
    v_u = 0.0F;
    v_x_utm = 0.0F;
    v_y_utm = 0.0F;
    v_z_utm = 0.0F;
    v_x_odom = 0.0F;
    v_y_odom = 0.0F;
    v_z_odom = 0.0F;
    gnss_status = STATUS_INVALID;
    utm_status = STATUS_INVALID;
    odom_status = STATUS_INVALID;
  }

  /**
   * @brief 构造函数
   */
  Gnss() {
    Clear();
  }
};

/**
 * @struct Imu
 * @brief 姿态信息
 */
struct Imu {
  /// 消息头
  MsgHead msg_head;
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

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    yaw_rate = 0;
    pitch_rate = 0;
    roll_rate = 0;
    accel_x = 0;
    accel_y = 0;
    accel_z = 0;
  }

  /**
   * @brief 构造函数
   */
  Imu() {
    Clear();
  }
};

/**
 * @struct RelativePos
 * @brief 相对位置(当前车身坐标系下)
 */
struct RelativePos {
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
  /// 速度，单位（米/秒）
  Float32_t v;

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    relative_time = 0;

    x = 0.0F;
    y = 0.0F;
    heading = 0.0F;
    yaw_rate = 0.0F;
    v = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  RelativePos() {
    Clear();
  }
};

/**
 * @struct RelativePosList
 * @brief 相对位置列表
 */
struct RelativePosList {
  /// 最大位置点的数量
  enum { MAX_RELATIVE_POS_NUM = 40 };

  /// 消息头
  MsgHead msg_head;
  /// 相对位置点的数量
  Int32_t relative_pos_num;
  /// 相对位置点的列表
  RelativePos relative_pos[MAX_RELATIVE_POS_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    relative_pos_num = 0;
    for (Int32_t i = 0; i < MAX_RELATIVE_POS_NUM; ++i) {
      relative_pos[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  RelativePosList() {
    Clear();
  }
};

struct MapLocalization {
  ad_msg::MsgHead msg_head;

  /// id
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  std::string curr_lane_id;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  Uint64_t curr_lane_id;
#else
  Uint64_t curr_lane_id;
#endif

  Float32_t s ;
  Float32_t l;
  Float32_t heading;

  void Clear() {
    msg_head.Clear();
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
    curr_lane_id.clear();
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
    curr_lane_id = 0;
#else
    curr_lane_id = 0;
#endif
    s = 0.0F;
    l = 0.0F;
    heading = 0.0F;
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_LOCALIZATION_H_


