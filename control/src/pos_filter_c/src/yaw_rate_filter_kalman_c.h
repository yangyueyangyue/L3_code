/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       yaw_rate_filter_kalman.h
 * @brief      平滑车辆当前的角速度
 * @details    使用IMU及车辆的当前状态平滑车辆当前的角速度
 *
 * @author     pengc
 * @date       2020.12.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_POS_FILTER_YAW_RATE_FILTER_KALMAN_C_H_
#define PHOENIX_POS_FILTER_YAW_RATE_FILTER_KALMAN_C_H_


#include "utils/macros.h"
#include "pos_filter_c.h"


#ifdef __cplusplus
extern "C" {
#endif


enum { MAX_YAW_RATE_LIST_SIZE = 5 };
typedef struct _YawRateFilterKalmanInstance_t YawRateFilterKalmanInstance_t;
struct _YawRateFilterKalmanInstance_t {
  struct {
    Float32_t yaw_rate_correcting_ratio_steering;

    Float32_t yaw_rate_weighted_steering;
    Float32_t yaw_rate_weighted_imu;
    Float32_t yaw_rate_weighted_chassis;

    Float32_t state_covariance;
    Float32_t measurement_covariance_steering;
    Float32_t measurement_covariance_imu;
    Float32_t measurement_covariance_chassis;
  } param;

  Int32_t initialization_flag;
  Int64_t prev_time_stamp;
  Float32_t prev_steering_wheel_angle;
  Float32_t prev_vehicle_velocity;
  Float32_t prev_yaw_rate_steering;

  Float32_t yaw_rate_expectation;
  Float32_t yaw_rate_covariance;

  Float32_t yaw_rate_from_steering;
  Float32_t yaw_rate_from_imu;
  Float32_t yaw_rate_from_chassis;

  Float32_t yaw_rate_steering_list[MAX_YAW_RATE_LIST_SIZE];
  Float32_t yaw_rate_imu_list[MAX_YAW_RATE_LIST_SIZE];
  Float32_t yaw_rate_chassis_list[MAX_YAW_RATE_LIST_SIZE];
  Float32_t yaw_rate_filtered_list[MAX_YAW_RATE_LIST_SIZE];
};


void Phoenix_YawRateFilterKalman_Initialize(
    YawRateFilterKalmanInstance_t* instance);

Int32_t Phoenix_YawRateFilterKalman_Update(
    YawRateFilterKalmanInstance_t* instance,
    Int64_t time_stamp, const PosFilterDataSource_t* data_source);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_POS_FILTER_YAW_RATE_FILTER_KALMAN_C_H_
