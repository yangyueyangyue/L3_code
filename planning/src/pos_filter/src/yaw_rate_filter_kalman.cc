/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       yaw_rate_filter_kalman.cc
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
#include "yaw_rate_filter_kalman.h"
#include "utils/com_utils.h"
#include "math/matrix.h"


#define ENABLE_YAW_RATE_FILTER_KALMAN_TRACE (0)


namespace phoenix {
namespace pos_filter {


YawRateFilterKalman::YawRateFilterKalman() {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  param_.yaw_rate_correcting_ratio_steering = 1.0F;//0.6F;

  param_.yaw_rate_weighted_steering = 0.2F;
  param_.yaw_rate_weighted_imu = 0.4F;
  param_.yaw_rate_weighted_chassis = 0.4F;

  param_.state_covariance = 2.0F;
  param_.measurement_covariance_steering = 4.0F;
  param_.measurement_covariance_imu = 3.0F;
  param_.measurement_covariance_chassis = 3.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
//  param_.yaw_rate_correcting_ratio_steering = 1.0F;//0.6F;

//  param_.yaw_rate_weighted_steering = 0.2F;
//  param_.yaw_rate_weighted_imu = 0.4F;
//  param_.yaw_rate_weighted_chassis = 0.4F;

//  param_.state_covariance = 2.0F;
//  param_.measurement_covariance_steering = 4.0F;
//  param_.measurement_covariance_imu = 2.0F;
//  param_.measurement_covariance_chassis = 2.0F;
  param_.yaw_rate_correcting_ratio_steering = 1.0F;//0.6F;

  param_.yaw_rate_weighted_steering = 0.2F;
  param_.yaw_rate_weighted_imu = 0.4F;
  param_.yaw_rate_weighted_chassis = 0.4F;

  param_.state_covariance = 6.0F;
  param_.measurement_covariance_steering = 20.0F;
  param_.measurement_covariance_imu = 2.0F;
  param_.measurement_covariance_chassis = 1.0F;
#else
  Error: Invalid vehicle platform.
#endif

  initialization_flag_ = false;
  prev_time_stamp_ = 0;
  prev_steering_wheel_angle_ = 0.0F;
  prev_vehicle_velocity_ = 0.0F;
  prev_yaw_rate_steering_ = 0.0F;

  yaw_rate_expectation_ = 0.0F;
  yaw_rate_covariance_ = 0.0F;

  yaw_rate_from_imu_ = 0.0F;
  yaw_rate_from_steering_ = 0.0F;
  yaw_rate_from_chassis_ = 0.0F;

  common::com_memset(yaw_rate_steering_list_, 0,
                     sizeof(yaw_rate_steering_list_));
  common::com_memset(yaw_rate_imu_list_, 0,
                     sizeof(yaw_rate_imu_list_));
  common::com_memset(yaw_rate_chassis_list_, 0,
                     sizeof(yaw_rate_chassis_list_));
  common::com_memset(yaw_rate_filtered_list_, 0,
                     sizeof(yaw_rate_filtered_list_));

  common::com_memset(yaw_rate_list_, 0, sizeof(yaw_rate_list_));
}

YawRateFilterKalman::~YawRateFilterKalman() {
  // nothing to do
}

bool YawRateFilterKalman::Update(
    Int64_t time_stamp, const PosFilterDataSource& data_source) {
#if ENABLE_YAW_RATE_FILTER_KALMAN_TRACE
  std::cout << "### YawRateFilterKalman::Update (Begin) ###" << std::endl;
#endif

  bool imu_valid = false;
  if (Nullptr_t != data_source.imu) {
    if (data_source.imu->msg_head.valid) {
      imu_valid = true;
    }
  }

  bool steering_angle_valid = false;
  bool chassis_yaw_rate_valid = false;
  if (Nullptr_t != data_source.chassis) {
    if (data_source.chassis->msg_head.valid) {
      if (data_source.chassis->steering_wheel_angle_valid &&
          data_source.chassis->v_valid) {
        steering_angle_valid = true;
      }
      if (data_source.chassis->yaw_rate_valid) {
        chassis_yaw_rate_valid = true;
      }
    }
  }
  /// TODO: This is not normal code, just for debug
  /// chassis_yaw_rate_valid must be set to false in control module
  /// if it is not valid in actuality.
  // chassis_yaw_rate_valid = false;

  if (!imu_valid && !steering_angle_valid && !chassis_yaw_rate_valid) {
    LOG_ERR << "Invalid source of data.";
    return false;
  }

  if (initialization_flag_) {
    if (common::CalcElapsedClockMs(prev_time_stamp_, time_stamp) > 200) {
      initialization_flag_ = false;
      common::com_memset(yaw_rate_steering_list_, 0,
                         sizeof(yaw_rate_steering_list_));
      common::com_memset(yaw_rate_imu_list_, 0,
                         sizeof(yaw_rate_imu_list_));
      common::com_memset(yaw_rate_chassis_list_, 0,
                         sizeof(yaw_rate_chassis_list_));
      common::com_memset(yaw_rate_filtered_list_, 0,
                         sizeof(yaw_rate_filtered_list_));
      LOG_ERR << "New data is timeout in filtering yaw rate.";
    }
  }

  Scalar yaw_rate_steering = 0.0F;
  Scalar yaw_rate_imu = 0.0F;
  Scalar yaw_rate_chassis = 0.0F;
  Scalar yaw_rate_filtered = 0.0F;
  Scalar delta_yaw_rate_steering = 0.0F;
  Scalar delta_yaw_rate_imu = 0.0F;
  Scalar delta_yaw_rate_chassis = 0.0F;
  if (imu_valid) {
    Scalar yaw_rate_sum = 0.0F;
    for (Int32_t i = (MAX_YAW_RATE_LIST_SIZE-1); i > 0; --i) {
      yaw_rate_imu_list_[i] = yaw_rate_imu_list_[i-1];
      yaw_rate_sum += yaw_rate_imu_list_[i];
    }
    yaw_rate_imu_list_[0] = data_source.imu->yaw_rate;
    yaw_rate_sum += yaw_rate_imu_list_[0];

    //yaw_rate_imu = data_source.imu->yaw_rate;
    yaw_rate_imu = yaw_rate_sum / MAX_YAW_RATE_LIST_SIZE;

    delta_yaw_rate_imu = yaw_rate_imu - yaw_rate_from_imu_;
  } else {
    common::com_memset(yaw_rate_imu_list_, 0, sizeof(yaw_rate_imu_list_));
  }
  if (chassis_yaw_rate_valid) {
    Scalar yaw_rate_sum = 0.0F;
    for (Int32_t i = (MAX_YAW_RATE_LIST_SIZE-1); i > 0; --i) {
      yaw_rate_chassis_list_[i] = yaw_rate_chassis_list_[i-1];
      yaw_rate_sum += yaw_rate_chassis_list_[i];
    }
    yaw_rate_chassis_list_[0] = data_source.chassis->yaw_rate;
    yaw_rate_sum += yaw_rate_chassis_list_[0];

    //yaw_rate_chassis = data_source.chassis->yaw_rate;
    yaw_rate_chassis = yaw_rate_sum / MAX_YAW_RATE_LIST_SIZE;

    delta_yaw_rate_chassis = yaw_rate_chassis - yaw_rate_from_chassis_;
  } else {
    common::com_memset(yaw_rate_chassis_list_, 0,
                       sizeof(yaw_rate_chassis_list_));
  }
  if (steering_angle_valid) {
    yaw_rate_steering = param_.yaw_rate_correcting_ratio_steering *
        vehicle_model_.CalcYawRateFromSteeringAngle(
          data_source.chassis->steering_wheel_angle,
          data_source.chassis->v);
    if (ad_msg::VEH_GEAR_R == data_source.chassis->gear) {
      // For backing mode of vehicle
      yaw_rate_steering = -yaw_rate_steering;
    }

    for (Int32_t i = (MAX_YAW_RATE_LIST_SIZE-1); i > 0; --i) {
      yaw_rate_steering_list_[i] = yaw_rate_steering_list_[i-1];
    }
    yaw_rate_steering_list_[0] = yaw_rate_steering;

    delta_yaw_rate_steering = yaw_rate_steering - yaw_rate_from_steering_;

#if ENABLE_YAW_RATE_FILTER_KALMAN_TRACE
    std::cout << "steering_angle = "
              << common::com_rad2deg(data_source.chassis->steering_wheel_angle)
              << "deg, v = " << data_source.chassis->v*3.6F
              << "km/h" << std::endl;
#endif
  } else {
    common::com_memset(yaw_rate_steering_list_, 0,
                       sizeof(yaw_rate_steering_list_));
  }

  if (!steering_angle_valid) {
    if (imu_valid && !chassis_yaw_rate_valid) {
      yaw_rate_filtered = yaw_rate_imu;
    } else if (!imu_valid && chassis_yaw_rate_valid) {
      yaw_rate_filtered = yaw_rate_chassis;
    } else {
      yaw_rate_filtered =
          (param_.yaw_rate_weighted_imu * yaw_rate_imu +
           param_.yaw_rate_weighted_chassis * yaw_rate_chassis) /
          (param_.yaw_rate_weighted_imu + param_.yaw_rate_weighted_chassis);
    }
  } else if (!imu_valid && !chassis_yaw_rate_valid) {
    yaw_rate_filtered = yaw_rate_steering;
  } else {
    if (!initialization_flag_) {
      if (imu_valid && !chassis_yaw_rate_valid) {
        yaw_rate_filtered =
            (param_.yaw_rate_weighted_steering * yaw_rate_steering +
             param_.yaw_rate_weighted_imu * yaw_rate_imu) /
            (param_.yaw_rate_weighted_steering + param_.yaw_rate_weighted_imu);
      } else if (!imu_valid && chassis_yaw_rate_valid) {
        yaw_rate_filtered =
            (param_.yaw_rate_weighted_steering * yaw_rate_steering +
             param_.yaw_rate_weighted_chassis * yaw_rate_chassis) /
            (param_.yaw_rate_weighted_steering +
             param_.yaw_rate_weighted_chassis);
      } else {
        yaw_rate_filtered =
            (param_.yaw_rate_weighted_steering * yaw_rate_steering +
             param_.yaw_rate_weighted_imu * yaw_rate_imu +
             param_.yaw_rate_weighted_chassis * yaw_rate_chassis) /
            (param_.yaw_rate_weighted_steering +
             param_.yaw_rate_weighted_chassis + param_.yaw_rate_weighted_imu);
      }
    } else {
      Scalar delta_yaw_rate_ref = yaw_rate_filtered_list_[0] - yaw_rate_filtered_list_[1];
      Scalar delta_yaw_rate = delta_yaw_rate_steering;
      if (imu_valid && !chassis_yaw_rate_valid) {
        if (common::com_abs(delta_yaw_rate_imu - delta_yaw_rate_ref) <
            common::com_abs(delta_yaw_rate_steering - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_imu;
        } else {
          delta_yaw_rate = delta_yaw_rate_steering;
        }
      } else if (!imu_valid && chassis_yaw_rate_valid) {
        if (common::com_abs(delta_yaw_rate_chassis - delta_yaw_rate_ref) <
            common::com_abs(delta_yaw_rate_steering - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_chassis;
        } else {
          delta_yaw_rate = delta_yaw_rate_steering;
        }
      } else {
        if (common::com_abs(delta_yaw_rate_imu - delta_yaw_rate_ref) <
            common::com_abs(delta_yaw_rate_chassis - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_imu;
        } else {
          delta_yaw_rate = delta_yaw_rate_chassis;
        }
        if (common::com_abs(delta_yaw_rate - delta_yaw_rate_ref) >
            common::com_abs(delta_yaw_rate_steering - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_chassis;
        }
      }
      yaw_rate_expectation_ += 0.8F*delta_yaw_rate + 0.2F*delta_yaw_rate_steering;
      //yaw_rate_expectation_ += delta_yaw_rate_steering;

      Scalar covariance = yaw_rate_covariance_ + param_.state_covariance;

      Scalar gain_steering = 0.0F;
      Scalar gain_imu = 0.0F;
      Scalar gain_chassis = 0.0F;
      if (imu_valid && !chassis_yaw_rate_valid) {
        common::Matrix<Float32_t, 2, 2> mat_2v2;
        common::Matrix<Float32_t, 2, 2> mat_2v2_inv;
        mat_2v2(0, 0) = covariance + param_.measurement_covariance_steering;
        mat_2v2(0, 1) = covariance;
        mat_2v2(1, 0) = covariance;
        mat_2v2(1, 1) = covariance + param_.measurement_covariance_imu;
        common::Mat_CalcPseudoInverse(mat_2v2, mat_2v2_inv);
        gain_steering = covariance * (mat_2v2_inv(0, 0) + mat_2v2_inv(1, 0));
        gain_imu = covariance * (mat_2v2_inv(0, 1) + mat_2v2_inv(1, 1));
      } else if (!imu_valid && chassis_yaw_rate_valid) {
        common::Matrix<Float32_t, 2, 2> mat_2v2;
        common::Matrix<Float32_t, 2, 2> mat_2v2_inv;
        mat_2v2(0, 0) = covariance + param_.measurement_covariance_steering;
        mat_2v2(0, 1) = covariance;
        mat_2v2(1, 0) = covariance;
        mat_2v2(1, 1) = covariance + param_.measurement_covariance_chassis;
        common::Mat_CalcPseudoInverse(mat_2v2, mat_2v2_inv);
        gain_steering = covariance * (mat_2v2_inv(0, 0) + mat_2v2_inv(1, 0));
        gain_chassis = covariance * (mat_2v2_inv(0, 1) + mat_2v2_inv(1, 1));
      } else {
        common::Matrix<Float32_t, 3, 3> mat_3v3;
        common::Matrix<Float32_t, 3, 3> mat_3v3_inv;
        mat_3v3(0, 0) = covariance + param_.measurement_covariance_steering;
        mat_3v3(0, 1) = covariance;
        mat_3v3(0, 2) = covariance;
        mat_3v3(1, 0) = covariance;
        mat_3v3(1, 1) = covariance + param_.measurement_covariance_imu;
        mat_3v3(1, 2) = covariance;
        mat_3v3(2, 0) = covariance;
        mat_3v3(2, 1) = covariance;
        mat_3v3(2, 2) = covariance + param_.measurement_covariance_chassis;
        common::Mat_CalcPseudoInverse(mat_3v3, mat_3v3_inv);
        gain_steering = covariance *
            (mat_3v3_inv(0, 0) + mat_3v3_inv(1, 0) + mat_3v3_inv(2, 0));
        gain_imu = covariance *
            (mat_3v3_inv(0, 1) + mat_3v3_inv(1, 1) + mat_3v3_inv(2, 1));
        gain_chassis = covariance *
            (mat_3v3_inv(0, 2) + mat_3v3_inv(1, 2) + mat_3v3_inv(2, 2));
      }

      yaw_rate_filtered =
          yaw_rate_expectation_ +
          gain_steering * (yaw_rate_steering - yaw_rate_expectation_) +
          gain_imu * (yaw_rate_imu - yaw_rate_expectation_) +
          gain_chassis * (yaw_rate_chassis - yaw_rate_expectation_);

      covariance = (1.0F - gain_steering - gain_imu - gain_chassis) * covariance;
      if (covariance < 1e-6F) {
        covariance = 1e-6F;
      }

      if (data_source.chassis->v_valid && data_source.chassis->v < 0.1F) {
        yaw_rate_filtered = 0.0F;
      }
      yaw_rate_expectation_ = yaw_rate_filtered;
      yaw_rate_covariance_ = covariance;

#if ENABLE_YAW_RATE_FILTER_KALMAN_TRACE
      std::cout << "delta_yaw_rate_steering = "
                << (yaw_rate_steering - prev_yaw_rate_steering_)
                << ", gain_imu = " << gain_imu
                << ", gain_chassis = " << gain_chassis
                << std::endl;
#endif
    }
  }

  initialization_flag_ = true;
  prev_time_stamp_ = time_stamp;
  if (steering_angle_valid) {
    prev_steering_wheel_angle_ = data_source.chassis->steering_wheel_angle;
    prev_vehicle_velocity_ = data_source.chassis->v;
    prev_yaw_rate_steering_ = yaw_rate_steering;
  } else {
    prev_yaw_rate_steering_ = yaw_rate_filtered;
  }
  yaw_rate_expectation_ = yaw_rate_filtered;

  yaw_rate_from_imu_ = yaw_rate_imu;
  yaw_rate_from_steering_ = yaw_rate_steering;
  yaw_rate_from_chassis_ = yaw_rate_chassis;

  for (Int32_t i = (MAX_YAW_RATE_LIST_SIZE-1); i > 0; --i) {
    yaw_rate_filtered_list_[i] = yaw_rate_filtered_list_[i-1];
  }
  yaw_rate_filtered_list_[0] = yaw_rate_filtered;

  // yaw rate change rate
  Float32_t yaw_rate_chg_rate = 0.0F;
  if (yaw_rate_list_[0].valid) {
    Float32_t time_elapsed = common::CalcElapsedClockMs(
          yaw_rate_list_[0].timestamp, time_stamp);
    Float32_t time_elapsed_ms = time_elapsed*0.001F;
    if ((time_elapsed > 200) || (time_elapsed < 0)) {
      common::com_memset(yaw_rate_list_, 0, sizeof(yaw_rate_list_));
    }
    if (time_elapsed > 40) {
      if (yaw_rate_list_[0].valid) {
        yaw_rate_chg_rate =
            (yaw_rate_filtered - yaw_rate_list_[0].yaw_rate) / time_elapsed_ms;
        yaw_rate_chg_rate =
            0.25F * yaw_rate_list_[2].yaw_rate_chg_rate +
            0.25F * yaw_rate_list_[1].yaw_rate_chg_rate +
            0.25F * yaw_rate_list_[0].yaw_rate_chg_rate +
            0.25F * yaw_rate_chg_rate;
      } else {
        yaw_rate_chg_rate = 0.0F;
      }

      for (Int32_t i = (3-1); i > 0; --i) {
        yaw_rate_list_[i].valid = yaw_rate_list_[i-1].valid;
        yaw_rate_list_[i].timestamp = yaw_rate_list_[i-1].timestamp;
        yaw_rate_list_[i].yaw_rate = yaw_rate_list_[i-1].yaw_rate;
        yaw_rate_list_[i].yaw_rate_chg_rate =
            yaw_rate_list_[i-1].yaw_rate_chg_rate;
      }
      yaw_rate_list_[0].valid = 1;
      yaw_rate_list_[0].timestamp = time_stamp;
      yaw_rate_list_[0].yaw_rate = yaw_rate_filtered;
      yaw_rate_list_[0].yaw_rate_chg_rate = yaw_rate_chg_rate;
    }
  } else {
    yaw_rate_list_[0].valid = 1;
    yaw_rate_list_[0].timestamp = time_stamp;
    yaw_rate_list_[0].yaw_rate = yaw_rate_filtered;
    yaw_rate_list_[0].yaw_rate_chg_rate = 0.0F;
  }

#if ENABLE_YAW_RATE_FILTER_KALMAN_TRACE
  std::cout << "yaw_rate_steering = " << yaw_rate_steering
            << "\nyaw_rate_imu      = " << yaw_rate_imu
            << "\nyaw_rate_chassis  = " << yaw_rate_chassis
            << "\nyaw_rate_filtered = " << yaw_rate_filtered
            << ", covariance = " << yaw_rate_covariance_
            << std::endl;


  std::cout << "### YawRateFilterKalman::Update (End) ###" << std::endl;
#endif

  return true;
}

void YawRateFilterKalman::CorrectYawRate(Scalar corrected_yaw_rate) {
  //yaw_rate_expectation_ = 0.5F*yaw_rate_expectation_ + 0.5F*corrected_yaw_rate;

  yaw_rate_from_imu_ = 0.0F*yaw_rate_expectation_ + 1.0F*corrected_yaw_rate;
}
}  // namespace pos_filter
}  // namespace phoenix

