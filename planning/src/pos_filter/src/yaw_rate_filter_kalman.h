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
#ifndef PHOENIX_POS_FILTER_YAW_RATE_FILTER_KALMAN_H_
#define PHOENIX_POS_FILTER_YAW_RATE_FILTER_KALMAN_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "pos_filter.h"
#include "vehicle_model_wrapper.h"


namespace phoenix {
namespace pos_filter {


class YawRateFilterKalman {
public:
  typedef Float32_t Scalar;

public:
  YawRateFilterKalman();
  ~YawRateFilterKalman();

  inline Scalar GetFilteredYawRate() const {
    return (yaw_rate_expectation_);
  }

  inline Scalar GetYawRateChgRate() const {
    return (yaw_rate_list_[0].yaw_rate_chg_rate);
  }

  inline Scalar GetYawRateFromImu() const {
    return (yaw_rate_from_imu_);
  }

  inline Scalar GetYawRateFromSteeringAngle() const {
    return (yaw_rate_from_steering_);
  }

  inline Scalar GetYawRateFromChassis() const {
    return (yaw_rate_from_chassis_);
  }

  bool Update(Int64_t time_stamp, const PosFilterDataSource& data_source);

  void CorrectYawRate(Scalar corrected_yaw_rate);

private:
  struct {
    Scalar yaw_rate_correcting_ratio_steering;

    Scalar yaw_rate_weighted_steering;
    Scalar yaw_rate_weighted_imu;
    Scalar yaw_rate_weighted_chassis;

    Scalar state_covariance;
    Scalar measurement_covariance_steering;
    Scalar measurement_covariance_imu;
    Scalar measurement_covariance_chassis;
  } param_;
  veh_model::VehicleModelWrapper vehicle_model_;

  bool initialization_flag_;
  Int64_t prev_time_stamp_;
  Scalar prev_steering_wheel_angle_;
  Scalar prev_vehicle_velocity_;
  Scalar prev_yaw_rate_steering_;

  Scalar yaw_rate_expectation_;
  Scalar yaw_rate_covariance_;

  Scalar yaw_rate_from_steering_;
  Scalar yaw_rate_from_imu_;
  Scalar yaw_rate_from_chassis_;

  enum { MAX_YAW_RATE_LIST_SIZE = 5 };
  Scalar yaw_rate_steering_list_[MAX_YAW_RATE_LIST_SIZE];
  Scalar yaw_rate_imu_list_[MAX_YAW_RATE_LIST_SIZE];
  Scalar yaw_rate_chassis_list_[MAX_YAW_RATE_LIST_SIZE];
  Scalar yaw_rate_filtered_list_[MAX_YAW_RATE_LIST_SIZE];

  struct {
    Int8_t valid;
    Int64_t timestamp;
    Float32_t yaw_rate;
    Float32_t yaw_rate_chg_rate;
  } yaw_rate_list_[3];
};


}  // namespace pos_filter
}  // namespace phoenix


#endif // PHOENIX_POS_FILTER_YAW_RATE_FILTER_KALMAN_H_
