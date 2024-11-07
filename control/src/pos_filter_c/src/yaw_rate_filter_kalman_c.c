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
#include "yaw_rate_filter_kalman_c.h"

#include "math/math_utils_c.h"
#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "vehicle_model_c.h"


#define ENABLE_YAW_RATE_FILTER_KALMAN_TRACE (0)

void Phoenix_YawRateFilterKalman_Initialize(
    YawRateFilterKalmanInstance_t* instance) {
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  //instance->param.yaw_rate_correcting_ratio_steering = 1.35F;
  instance->param.yaw_rate_correcting_ratio_steering = 0.7F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 0.1F;
  instance->param.measurement_covariance_steering = 4.0F;
  instance->param.measurement_covariance_imu = 3.0F;
  instance->param.measurement_covariance_chassis = 3.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  instance->param.yaw_rate_correcting_ratio_steering = 1.0F;//0.6F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 2.0F;
  instance->param.measurement_covariance_steering = 4.0F;
  instance->param.measurement_covariance_imu = 3.0F;
  instance->param.measurement_covariance_chassis = 3.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  instance->param.yaw_rate_correcting_ratio_steering = 1.0F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 0.5F;
  instance->param.measurement_covariance_steering = 6.0F;
  instance->param.measurement_covariance_imu = 3.0F;
  instance->param.measurement_covariance_chassis = 3.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  instance->param.yaw_rate_correcting_ratio_steering = 0.8F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 0.1F;
  instance->param.measurement_covariance_steering = 4.0F;
  instance->param.measurement_covariance_imu = 3.0F;
  instance->param.measurement_covariance_chassis = 3.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  instance->param.yaw_rate_correcting_ratio_steering = 1.0F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 0.5F;
  instance->param.measurement_covariance_steering = 6.0F;
  instance->param.measurement_covariance_imu = 3.0F;
  instance->param.measurement_covariance_chassis = 3.0F;
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
//  instance->param.yaw_rate_correcting_ratio_steering = 1.0F;

//  instance->param.yaw_rate_weighted_steering = 0.2F;
//  instance->param.yaw_rate_weighted_imu = 0.4F;
//  instance->param.yaw_rate_weighted_chassis = 0.4F;

//  instance->param.state_covariance = 0.5F;
//  instance->param.measurement_covariance_steering = 6.0F;
//  instance->param.measurement_covariance_imu = 3.0F;
//  instance->param.measurement_covariance_chassis = 3.0F;
#if 0
  instance->param.yaw_rate_correcting_ratio_steering = 1.0F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 2.0F;
  instance->param.measurement_covariance_steering = 6.0F;
  instance->param.measurement_covariance_imu = 3.0F;
  instance->param.measurement_covariance_chassis = 1.0F;
#else
  instance->param.yaw_rate_correcting_ratio_steering = 1.0F;

  instance->param.yaw_rate_weighted_steering = 0.2F;
  instance->param.yaw_rate_weighted_imu = 0.4F;
  instance->param.yaw_rate_weighted_chassis = 0.4F;

  instance->param.state_covariance = 6.0F;
  instance->param.measurement_covariance_steering = 20.0F;
  instance->param.measurement_covariance_imu = 2.0F;
  instance->param.measurement_covariance_chassis = 1.0F;
#endif
#else
  Error: Invalid vehicle platform.
#endif

  instance->initialization_flag = 0;
  instance->prev_time_stamp = 0;
  instance->prev_steering_wheel_angle = 0.0F;
  instance->prev_vehicle_velocity = 0.0F;
  instance->prev_yaw_rate_steering = 0.0F;

  instance->yaw_rate_expectation = 0.0F;
  instance->yaw_rate_covariance = 0.0F;

  instance->yaw_rate_from_imu = 0.0F;
  instance->yaw_rate_from_steering = 0.0F;
  instance->yaw_rate_from_chassis = 0.0F;

  phoenix_com_memset(instance->yaw_rate_steering_list, 0,
                     sizeof(instance->yaw_rate_steering_list));
  phoenix_com_memset(instance->yaw_rate_imu_list, 0,
                     sizeof(instance->yaw_rate_imu_list));
  phoenix_com_memset(instance->yaw_rate_chassis_list, 0,
                     sizeof(instance->yaw_rate_chassis_list));
  phoenix_com_memset(instance->yaw_rate_filtered_list, 0,
                     sizeof(instance->yaw_rate_filtered_list));
}

Int32_t Phoenix_YawRateFilterKalman_Update(
    YawRateFilterKalmanInstance_t* instance,
    Int64_t time_stamp, const PosFilterDataSource_t* data_source) {
  Int32_t index = 0;
  Int32_t imu_valid = 0;
  Int32_t steering_angle_valid = 0;
  Int32_t chassis_yaw_rate_valid = 0;
  Float32_t yaw_rate_sum = 0.0F;
  Float32_t yaw_rate_imu = 0.0F;
  Float32_t yaw_rate_steering = 0.0F;
  Float32_t yaw_rate_chassis = 0.0F;
  Float32_t yaw_rate_filtered = 0.0F;
  Float32_t delta_yaw_rate_steering = 0.0F;
  Float32_t delta_yaw_rate_imu = 0.0F;
  Float32_t delta_yaw_rate_chassis = 0.0F;
  Float32_t delta_yaw_rate_ref = 0.0F;
  Float32_t delta_yaw_rate = 0.0F;
  Float32_t covariance = 0.0F;
  Float32_t gain_steering = 0.0F;
  Float32_t gain_imu = 0.0F;
  Float32_t gain_chassis = 0.0F;
  Float32_t det = 0.0F;
  Float32_t t = 0.0F;
  Float32_t tmp = 0.0F;

  if (data_source->imu->msg_head.valid) {
    imu_valid = 1;
  }
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  /// FT-Auman项目的GNSS/INS导航单元的IMU噪音较大, 暂不使用
  imu_valid = 0;
#endif
  if (data_source->chassis->steering_wheel_angle_valid &&
      data_source->chassis->v_valid) {
    steering_angle_valid = 1;
  }
  if (data_source->chassis->yaw_rate_valid) {
    chassis_yaw_rate_valid = 1;
  }

  if (!imu_valid && !steering_angle_valid && !chassis_yaw_rate_valid) {
    LOG_ERR_C("[LAT][YawRateFilterKalman] Imu and steering_angle and chassis_yaw_rate are invalid.\n");
    return -1;
  }

  if (instance->initialization_flag) {
    if (Phoenix_Common_CalcElapsedClockMs(
          instance->prev_time_stamp, time_stamp) > 200) {
      instance->initialization_flag = 0;
      phoenix_com_memset(instance->yaw_rate_steering_list, 0,
                         sizeof(instance->yaw_rate_steering_list));
      phoenix_com_memset(instance->yaw_rate_imu_list, 0,
                         sizeof(instance->yaw_rate_imu_list));
      phoenix_com_memset(instance->yaw_rate_chassis_list, 0,
                         sizeof(instance->yaw_rate_chassis_list));
      phoenix_com_memset(instance->yaw_rate_filtered_list, 0,
                         sizeof(instance->yaw_rate_filtered_list));
      LOG_ERR_C("[LAT][YawRateFilterKalman] New data is timeout when filter yaw rate.\n");
    }
  }

  if (imu_valid) {
    yaw_rate_sum = 0.0F;
    for (index = (MAX_YAW_RATE_LIST_SIZE-1); index > 0; --index) {
      instance->yaw_rate_imu_list[index] = instance->yaw_rate_imu_list[index-1];
      yaw_rate_sum += instance->yaw_rate_imu_list[index];
    }
    instance->yaw_rate_imu_list[0] = data_source->imu->yaw_rate;
    yaw_rate_sum += instance->yaw_rate_imu_list[0];

    // yaw_rate_imu = data_source->imu.yaw_rate;
    yaw_rate_imu = yaw_rate_sum / MAX_YAW_RATE_LIST_SIZE;

    delta_yaw_rate_imu = yaw_rate_imu - instance->yaw_rate_from_imu;
  } else {
    phoenix_com_memset(instance->yaw_rate_imu_list, 0,
                       sizeof(instance->yaw_rate_imu_list));
  }
  if (chassis_yaw_rate_valid) {
    yaw_rate_sum = 0.0F;
    for (index = (MAX_YAW_RATE_LIST_SIZE-1); index > 0; --index) {
      instance->yaw_rate_chassis_list[index] = instance->yaw_rate_chassis_list[index-1];
      yaw_rate_sum += instance->yaw_rate_chassis_list[index];
    }
    instance->yaw_rate_chassis_list[0] = data_source->chassis->yaw_rate;
    yaw_rate_sum += instance->yaw_rate_chassis_list[0];

    // yaw_rate_chassis = data_source->chassis.yaw_rate;
    yaw_rate_chassis = yaw_rate_sum / MAX_YAW_RATE_LIST_SIZE;

    delta_yaw_rate_chassis = yaw_rate_chassis - instance->yaw_rate_from_chassis;
  } else {
    phoenix_com_memset(instance->yaw_rate_chassis_list, 0,
                       sizeof(instance->yaw_rate_chassis_list));
  }
  if (steering_angle_valid) {
    yaw_rate_steering = instance->param.yaw_rate_correcting_ratio_steering *
        Phoenix_VehModel_CalcYawRateFromSteeringAngle(
          data_source->chassis->steering_wheel_angle,
          data_source->chassis->v);
    if (VEH_GEAR_R == data_source->chassis->gear) {
      // For backing mode of vehicle
      yaw_rate_steering = -yaw_rate_steering;
    }

    for (index = (MAX_YAW_RATE_LIST_SIZE-1); index > 0; --index) {
      instance->yaw_rate_steering_list[index] = instance->yaw_rate_steering_list[index-1];
    }
    instance->yaw_rate_steering_list[0] = yaw_rate_steering;

    delta_yaw_rate_steering = yaw_rate_steering - instance->yaw_rate_from_steering;
  } else {
    phoenix_com_memset(instance->yaw_rate_steering_list, 0,
                       sizeof(instance->yaw_rate_steering_list));
  }

  if (!steering_angle_valid) {
    if (imu_valid && !chassis_yaw_rate_valid) {
      yaw_rate_filtered = yaw_rate_imu;
    } else if (!imu_valid && chassis_yaw_rate_valid) {
      yaw_rate_filtered = yaw_rate_chassis;
    } else {
      yaw_rate_filtered =
          (instance->param.yaw_rate_weighted_imu * yaw_rate_imu +
           instance->param.yaw_rate_weighted_chassis * yaw_rate_chassis) /
          (instance->param.yaw_rate_weighted_imu +
           instance->param.yaw_rate_weighted_chassis);
    }
  } else if (!imu_valid && !chassis_yaw_rate_valid) {
    yaw_rate_filtered = yaw_rate_steering;
  } else {
    if (!instance->initialization_flag) {
      if (imu_valid && !chassis_yaw_rate_valid) {
        yaw_rate_filtered =
            (instance->param.yaw_rate_weighted_steering * yaw_rate_steering +
             instance->param.yaw_rate_weighted_imu * yaw_rate_imu) /
            (instance->param.yaw_rate_weighted_steering +
             instance->param.yaw_rate_weighted_imu);
      } else if (!imu_valid && chassis_yaw_rate_valid) {
        yaw_rate_filtered =
            (instance->param.yaw_rate_weighted_steering * yaw_rate_steering +
             instance->param.yaw_rate_weighted_chassis * yaw_rate_chassis) /
            (instance->param.yaw_rate_weighted_steering +
             instance->param.yaw_rate_weighted_chassis);
      } else {
        yaw_rate_filtered =
            (instance->param.yaw_rate_weighted_steering * yaw_rate_steering +
             instance->param.yaw_rate_weighted_imu * yaw_rate_imu +
             instance->param.yaw_rate_weighted_chassis * yaw_rate_chassis) /
            (instance->param.yaw_rate_weighted_steering +
             instance->param.yaw_rate_weighted_chassis +
             instance->param.yaw_rate_weighted_imu);
      }
    } else {
      delta_yaw_rate_ref = instance->yaw_rate_filtered_list[0] -
          instance->yaw_rate_filtered_list[1];
      delta_yaw_rate = delta_yaw_rate_steering;
      if (imu_valid && !chassis_yaw_rate_valid) {
        if (phoenix_com_abs_f(delta_yaw_rate_imu - delta_yaw_rate_ref) <
            phoenix_com_abs_f(delta_yaw_rate_steering - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_imu;
        } else {
          delta_yaw_rate = delta_yaw_rate_steering;
        }
      } else if (!imu_valid && chassis_yaw_rate_valid) {
        if (phoenix_com_abs_f(delta_yaw_rate_chassis - delta_yaw_rate_ref) <
            phoenix_com_abs_f(delta_yaw_rate_steering - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_chassis;
        } else {
          delta_yaw_rate = delta_yaw_rate_steering;
        }
      } else {
        if (phoenix_com_abs_f(delta_yaw_rate_imu - delta_yaw_rate_ref) <
            phoenix_com_abs_f(delta_yaw_rate_chassis - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_imu;
        } else {
          delta_yaw_rate = delta_yaw_rate_chassis;
        }
        if (phoenix_com_abs_f(delta_yaw_rate - delta_yaw_rate_ref) >
            phoenix_com_abs_f(delta_yaw_rate_steering - delta_yaw_rate_ref)) {
          delta_yaw_rate = delta_yaw_rate_chassis;
        }
      }

      //instance->yaw_rate_expectation +=
      //    yaw_rate_steering - instance->prev_yaw_rate_steering;
      instance->yaw_rate_expectation += 0.8F*delta_yaw_rate + 0.2F*delta_yaw_rate_steering;

      covariance =
          instance->yaw_rate_covariance + instance->param.state_covariance;

      gain_steering = 0.0F;
      gain_imu = 0.0F;
      gain_chassis = 0.0F;
      if (imu_valid && !chassis_yaw_rate_valid) {
        det =
            covariance * instance->param.measurement_covariance_steering +
            covariance * instance->param.measurement_covariance_imu +
            instance->param.measurement_covariance_steering *
            instance->param.measurement_covariance_imu;
        if (0.01F < det) {
          gain_steering =
              covariance * instance->param.measurement_covariance_imu / det;
          gain_imu =
              covariance * instance->param.measurement_covariance_steering / det;
        } else {
          gain_steering = 0.8F;
          gain_imu = 0.1F;
        }
      } else if (!imu_valid && chassis_yaw_rate_valid) {
        det =
            covariance * instance->param.measurement_covariance_steering +
            covariance * instance->param.measurement_covariance_chassis +
            instance->param.measurement_covariance_steering *
            instance->param.measurement_covariance_chassis;
        if (0.01F < det) {
          gain_steering =
              covariance * instance->param.measurement_covariance_chassis / det;
          gain_chassis =
              covariance * instance->param.measurement_covariance_steering / det;
        } else {
          gain_steering = 0.8F;
          gain_chassis = 0.1F;
        }
      } else {
        t = instance->param.yaw_rate_weighted_imu /
            (instance->param.yaw_rate_weighted_imu +
             instance->param.yaw_rate_weighted_chassis);
        tmp = t * instance->param.measurement_covariance_imu +
            (1.0F - t) * instance->param.yaw_rate_weighted_chassis;
        det =
            covariance * instance->param.measurement_covariance_steering +
            covariance * tmp +
            instance->param.measurement_covariance_steering * tmp;
        if (0.01F < det) {
          gain_steering = covariance * tmp / det;
          tmp = covariance * instance->param.measurement_covariance_steering / det;
          gain_imu = t * tmp;
          gain_chassis = (1.0F - t) * tmp;
        } else {
          gain_steering = 0.7F;
          gain_imu = 0.1F;
          gain_chassis = 0.1F;
        }
      }

      yaw_rate_filtered =
          instance->yaw_rate_expectation +
          gain_steering * (yaw_rate_steering - instance->yaw_rate_expectation) +
          gain_imu * (yaw_rate_imu - instance->yaw_rate_expectation) +
          gain_chassis * (yaw_rate_chassis - instance->yaw_rate_expectation);

      covariance = (1.0F - gain_steering - gain_imu - gain_chassis) * covariance;
      if (covariance < 1e-6F) {
        covariance = 1e-6F;
      }

      if (data_source->chassis->v_valid && data_source->chassis->v < 0.1F) {
        yaw_rate_filtered = 0.0F;
      }
      instance->yaw_rate_expectation = yaw_rate_filtered;
      instance->yaw_rate_covariance = covariance;
    }
  }

  instance->initialization_flag = 1;
  instance->prev_time_stamp = time_stamp;
  if (steering_angle_valid) {
    instance->prev_steering_wheel_angle =
        data_source->chassis->steering_wheel_angle;
    instance->prev_vehicle_velocity = data_source->chassis->v;
    instance->prev_yaw_rate_steering = yaw_rate_steering;
  } else {
    instance->prev_yaw_rate_steering = yaw_rate_filtered;
  }
  instance->yaw_rate_expectation = yaw_rate_filtered;

  instance->yaw_rate_from_imu = yaw_rate_imu;
  instance->yaw_rate_from_steering = yaw_rate_steering;
  instance->yaw_rate_from_chassis = yaw_rate_chassis;

  for (index = (MAX_YAW_RATE_LIST_SIZE-1); index > 0; --index) {
    instance->yaw_rate_filtered_list[index] = instance->yaw_rate_filtered_list[index-1];
  }
  instance->yaw_rate_filtered_list[0] = yaw_rate_filtered;

  return (0);
}

