/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       gps_tools.h
 * @brief      GPS坐标系转UTM坐标系
 * @details    实现了WGS84格式的GPS坐标系与通用横墨卡托格网
 *             系统(UTM)坐标系之间互相转换的函数
 * @author     boc
 * @date       2020.07.02
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/07/02  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_GPS_TOOLS_H_
#define PHOENIX_COMMON_GPS_TOOLS_H_

#include <string>
#include "utils/macros.h"
#include "math/math_utils.h"

#include "geometry/geometry_utils.h"

namespace phoenix {
namespace common {

/**
 * @struct GPS_POINT
 * @brief GPS坐标定义
 */
struct GpsPoint{
  Float64_t latitude;  // 纬度，单位：度
  Float64_t longitude;  // 经度，单位：度

  GpsPoint() {
    latitude = 0;
    longitude = 0;
  }

  GpsPoint(Float64_t lat, Float64_t lon) {
    latitude = lat;
    longitude = lon;
  }

  bool operator == (const GpsPoint& coor_) const {
    if ((com_abs(coor_.latitude - this->latitude) < (1e-8)) &&
        (com_abs(coor_.longitude - this->longitude) < (1e-8))) {
      return true;
    } else {
      return false;
    }
  }
};

/**
 * @brief 求两个GPS坐标点之间的球面距离
 * @param point1  [in] 起点
 * @param point2  [in] 终点
 * @return  球面距离，单位：米
 */
Float64_t CalcSphericalDistance(
    const GpsPoint &point1, const GpsPoint &point2);

/**
 * @brief 两个GPS坐标构成的向量的航向角
 * @param crd1  [in] 起点
 * @param crd2  [in] 终点
 * @return  航向角(以正东向为0角度，逆时针方向为正角度方向)，单位：弧度
 */
Float64_t CalcAngle(const GpsPoint& crd1, const GpsPoint& crd2);

/**
 * @brief 给定起始GPS点和航向角，计算在这个航向角方向上距离起始GPS点距离为给定长度的GPS点坐标
 * @param pos  [in] 起点
 * @param angle  [in] 航向角
 * @param meter_dist  [in] 距离
 * @return  目标GPS坐标点
 */
GpsPoint CalcNextGpsPoint(
    const GpsPoint &pos, Float64_t angle, Float64_t meter_dist);

/**
 * @brief 转换航向角(正北为0,顺时针为正 --> 正东为0，逆时针为正)
 * @param heading  [in] 航向角，单位：度
 * @return  航向角(以正东向为0角度，逆时针方向为正角度方向)，单位：度
 */
inline Float64_t ConvGpsHeading(Float64_t heading) {
  heading = -heading + 90;
  if (heading <= -180.0) {
    heading += 360.0;
  }

  return (heading);
}

/**
 * @brief 将gps点转换为局部坐标下的点(以米为单位，以正东向为0角度，逆时针方向为正角度方向)
 * @param gps_origin[in] 局部坐标系的原点(以GPS坐标指定)
 * @param gps_point[in] 需要被转换的GPS点
 * @param local_point[out] 转换后的局部坐标系下的点
 */
Float64_t ConvGpsPointToLocalPoint(const GpsPoint& gps_origin,
                                   const GpsPoint& gps_point,
                                   Vec2d* local_point);


/**
 * @brief 获取指定纬度在通用横墨卡托格网系统下的带号。
 * @details Determine the correct UTM letter designator for the
 *          given latitude.
 * @param[in] lat       纬度
 * @return UTM带号
 *         'Z' if latitude is outside the UTM limits of 84N to 80S
 */
Char_t UTMLetterDesignator(Float64_t lat);

/**
 * @brief 获取指定纬度和经度在通用横墨卡托格网系统下的坐标。
 * @details Convert lat/long to UTM coords. Equations from USGS Bulletin 1532
 *          East Longitudes are positive, West longitudes are negative.
 *          North latitudes are positive, South latitudes are negative.
 *          Lat and Long are in fractional degrees.
 * @param[in] lat             WGS84纬度
 * @param[in] lon             WGS84经度
 * @param[out] utm_northing   UTM北向坐标值
 * @param[out] utm_easting    UTM东向坐标值
 * @return 收敛角，单位：度。
 * @note 在UTM坐标系下，车辆与UTM东向的夹角为：90 - GPS航向角 + 收敛角
 */
Float64_t LLtoUTM(const Float64_t lat, const Float64_t lon,
                  Float64_t* utm_northing, Float64_t* utm_easting);

/**
 * @brief 获取指定通用横墨卡托格网系统下的坐标对应的纬度和经度。
 * @details Converts UTM coords to lat/long. Equations from USGS Bulletin 1532
 *          East Longitudes are positive, West longitudes are negative.
 *          North latitudes are positive, South latitudes are negative
 *          Lat and Long are in fractional degrees.
 * @param[in] utm_northing   UTM北向坐标值
 * @param[in] utm_easting    UTM东向坐标值
 * @param[in] utm_zone       UTM区号
 * @param[out] lat           WGS84纬度
 * @param[out] lon           WGS84经度
 */
void UTMtoLL(const Float64_t utm_northing, const Float64_t utm_easting,
             const Char_t utm_zone[32], Float64_t* lat, Float64_t* lon);


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_GPS_TOOLS_H_


