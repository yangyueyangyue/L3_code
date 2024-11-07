/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       pos_filter_impl.cc
 * @brief      估计车辆的相对位置序列
 * @details    使用RTK、相机识别的车道线、车辆当前状态等信息估计车辆的相对位置序列
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
#include "pos_filter_impl_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "vehicle_model_c.h"
#include "pos_filter_c.h"


#define ENABLE_POS_FILTER_IMPL_TRACE (0)


static void InitializeYawRateChgRateInfo(PosFilterYawRateChgRateInfo_t* chg_rate_info, Int64_t timestamp) {
  chg_rate_info->valid = 0;
  chg_rate_info->timestamp = timestamp;
  chg_rate_info->yaw_rate = 0;
  chg_rate_info->prev_yaw_rate_chg_rate = 0;
  phoenix_com_memset(chg_rate_info->yaw_rate_chg_rate_list, 0, sizeof(chg_rate_info->yaw_rate_chg_rate_list));
}

static Float32_t UpdateYawRateChgRateInfo(PosFilterYawRateChgRateInfo_t* chg_rate_info, Float32_t yaw_rate, Int64_t timestamp) {
  Float32_t yaw_rate_chg_rate = 0.0f;
  Int64_t elapsed = 0;
  
  if (chg_rate_info->valid) {
    elapsed = Phoenix_Common_CalcElapsedClockMs(chg_rate_info->timestamp, timestamp);
	
    if ((200 <= elapsed) && (elapsed <= 400)) {
      yaw_rate_chg_rate = (yaw_rate - chg_rate_info->yaw_rate) / elapsed * 1000;

      chg_rate_info->yaw_rate_chg_rate_list[2] = chg_rate_info->yaw_rate_chg_rate_list[1];
      chg_rate_info->yaw_rate_chg_rate_list[1] = chg_rate_info->yaw_rate_chg_rate_list[0];
      yaw_rate_chg_rate = 0.1F*chg_rate_info->yaw_rate_chg_rate_list[2] +
          0.4F*chg_rate_info->yaw_rate_chg_rate_list[1] + 0.5F*yaw_rate_chg_rate;
      //yaw_rate_chg_rate = 0.1F*chg_rate_info->yaw_rate_chg_rate_list[2] +
      //    0.2F*chg_rate_info->yaw_rate_chg_rate_list[1] + 0.7F*yaw_rate_chg_rate;
      chg_rate_info->yaw_rate_chg_rate_list[0] = yaw_rate_chg_rate;

      chg_rate_info->yaw_rate = yaw_rate;
      chg_rate_info->prev_yaw_rate_chg_rate = yaw_rate_chg_rate;
      chg_rate_info->timestamp = timestamp;
    } else if (elapsed > 400) {
      chg_rate_info->yaw_rate = yaw_rate;
      chg_rate_info->prev_yaw_rate_chg_rate = 0;
      chg_rate_info->timestamp = timestamp;
    } else if (elapsed < 0) {
      chg_rate_info->yaw_rate = yaw_rate;
      chg_rate_info->prev_yaw_rate_chg_rate = 0;
      chg_rate_info->timestamp = timestamp;
    } else {
      yaw_rate_chg_rate = chg_rate_info->prev_yaw_rate_chg_rate;
    }
  } else {
    chg_rate_info->yaw_rate = yaw_rate;
    chg_rate_info->valid = 1;
    chg_rate_info->prev_yaw_rate_chg_rate = 0;
    chg_rate_info->timestamp = timestamp;
  }

  return (yaw_rate_chg_rate);
}

void Phoenix_PosFilterImpl_Initialize(PosFilterImplInstance_t* instance) {
  instance->timestamp = 0;

  instance->estimated_pos.relative_time = 0;
  instance->estimated_pos.x = 0.0F;
  instance->estimated_pos.y = 0.0F;
  instance->estimated_pos.heading = 0.0F;
  instance->estimated_pos.yaw_rate = 0.0F;
  instance->estimated_pos.v = 0.0F;

  phoenix_com_memset(instance->yaw_rate_list, 0,
                     sizeof(instance->yaw_rate_list));
  phoenix_com_memset(instance->d_yaw_rate_list, 0,
                     sizeof(instance->d_yaw_rate_list));

  Phoenix_YawRateFilterKalman_Initialize(&(instance->yaw_rate_filter));

  InitializeYawRateChgRateInfo(&(instance->yaw_rate_chg_rate_info), instance->timestamp);
}

Int32_t Phoenix_PosFilterImpl_Update(
    PosFilterImplInstance_t* instance,
    const PosFilterDataSource_t* data_source) {

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "\n### PosFilterImpl::Update (Begin) ###" << std::endl;
#endif
  Int64_t timestamp = data_source->timestamp;
  Int64_t time_elapsed = 0;
  Float32_t time_elapsed_ms = 0.0F;
  Int32_t ret = 0;
  Int32_t chassis_valid = 0;
  Float32_t velocity = 0;
  Float32_t yaw_rate = 0.0F;
  Float32_t yaw_rate_chg_rate = 0.0F;
  Float32_t yaw_rate_d_chg_rate = 0.0F;
  Int64_t relative_time = 0;
  Float32_t prev_pos[3] = { 0.0F };
  Float32_t next_pos[3] = { 0.0F };

  ret = Phoenix_YawRateFilterKalman_Update(
        &(instance->yaw_rate_filter), timestamp, data_source);
  if (ret < 0) {
    LOG_ERR_C("[CTL][PosFilterImpl] Failed to filter yaw_rate.\n");
    return (-1);
  }

  if (data_source->chassis->v_valid) {
    chassis_valid = 1;
  }

  if (chassis_valid) {
    velocity = data_source->chassis->v;
  } else {
    LOG_ERR_C("[CTL][PosFilterImpl] Invalid velocity from chassis.\n");
    return (-2);
  }
  if (VEH_GEAR_R == data_source->chassis->gear) {
    // For backing mode of vehicle
    velocity = -velocity;
  }

  yaw_rate = instance->yaw_rate_filter.yaw_rate_expectation;

  if (instance->yaw_rate_list[0].valid) {
    time_elapsed = Phoenix_Common_CalcElapsedClockMs(
          instance->yaw_rate_list[0].timestamp, timestamp);
    time_elapsed_ms = time_elapsed*0.001F;
    if ((time_elapsed > 200) || (time_elapsed < 0)) {
      phoenix_com_memset(instance->yaw_rate_list, 0,
                         sizeof(instance->yaw_rate_list));
    }
    if (time_elapsed > 40) {
#if 1
      if (instance->yaw_rate_list[0].valid) {
        yaw_rate_chg_rate =
            (yaw_rate - instance->yaw_rate_list[0].yaw_rate) /
            time_elapsed_ms;
        yaw_rate_chg_rate =
            0.25F * instance->yaw_rate_list[2].yaw_rate_chg_rate +
            0.25F * instance->yaw_rate_list[1].yaw_rate_chg_rate +
            0.25F * instance->yaw_rate_list[0].yaw_rate_chg_rate +
            0.25F * yaw_rate_chg_rate;
      } else {
        yaw_rate_chg_rate = 0.0F;
      }
#else
      yaw_rate_chg_rate = UpdateYawRateChgRateInfo(&(instance->yaw_rate_chg_rate_info), yaw_rate, timestamp);
#endif

      instance->yaw_rate_list[2].valid = instance->yaw_rate_list[1].valid;
      instance->yaw_rate_list[2].timestamp = instance->yaw_rate_list[1].timestamp;
      instance->yaw_rate_list[2].yaw_rate = instance->yaw_rate_list[1].yaw_rate;
      instance->yaw_rate_list[2].yaw_rate_chg_rate =
          instance->yaw_rate_list[1].yaw_rate_chg_rate;
      instance->yaw_rate_list[1].valid = instance->yaw_rate_list[0].valid;
      instance->yaw_rate_list[1].timestamp = instance->yaw_rate_list[0].timestamp;
      instance->yaw_rate_list[1].yaw_rate = instance->yaw_rate_list[0].yaw_rate;
      instance->yaw_rate_list[1].yaw_rate_chg_rate =
          instance->yaw_rate_list[0].yaw_rate_chg_rate;
      instance->yaw_rate_list[0].valid = 1;
      instance->yaw_rate_list[0].timestamp = timestamp;
      instance->yaw_rate_list[0].yaw_rate = yaw_rate;
      instance->yaw_rate_list[0].yaw_rate_chg_rate = yaw_rate_chg_rate;
    }
  } else {
    instance->yaw_rate_list[0].valid = 1;
    instance->yaw_rate_list[0].timestamp = timestamp;
    instance->yaw_rate_list[0].yaw_rate = yaw_rate;
    instance->yaw_rate_list[0].yaw_rate_chg_rate = 0.0F;
  }

  if (instance->d_yaw_rate_list[0].valid) {
    time_elapsed = Phoenix_Common_CalcElapsedClockMs(
          instance->d_yaw_rate_list[0].timestamp, timestamp);
    time_elapsed_ms = time_elapsed*0.001F;
    if ((time_elapsed > 400) || (time_elapsed < 0)) {
      phoenix_com_memset(instance->d_yaw_rate_list, 0,
                         sizeof(instance->d_yaw_rate_list));
    }
    if (time_elapsed > 290) {
      if (instance->d_yaw_rate_list[0].valid) {
        yaw_rate_d_chg_rate =
            (yaw_rate_chg_rate - instance->d_yaw_rate_list[0].yaw_rate_chg_rate) /
            time_elapsed_ms;
        yaw_rate_d_chg_rate =
            0.25F * instance->d_yaw_rate_list[2].yaw_rate_d_chg_rate +
            0.25F * instance->d_yaw_rate_list[1].yaw_rate_d_chg_rate +
            0.25F * instance->d_yaw_rate_list[0].yaw_rate_d_chg_rate +
            0.25F * yaw_rate_d_chg_rate;
      } else {
        yaw_rate_d_chg_rate = 0.0F;
      }

      instance->d_yaw_rate_list[2].valid = instance->d_yaw_rate_list[1].valid;
      instance->d_yaw_rate_list[2].timestamp = instance->d_yaw_rate_list[1].timestamp;
      instance->d_yaw_rate_list[2].yaw_rate_chg_rate =
          instance->d_yaw_rate_list[1].yaw_rate_chg_rate;
      instance->d_yaw_rate_list[2].yaw_rate_d_chg_rate =
          instance->d_yaw_rate_list[1].yaw_rate_d_chg_rate;
      instance->d_yaw_rate_list[1].valid = instance->d_yaw_rate_list[0].valid;
      instance->d_yaw_rate_list[1].timestamp = instance->d_yaw_rate_list[0].timestamp;
      instance->d_yaw_rate_list[1].yaw_rate_chg_rate =
          instance->d_yaw_rate_list[0].yaw_rate_chg_rate;
      instance->d_yaw_rate_list[1].yaw_rate_d_chg_rate =
          instance->d_yaw_rate_list[0].yaw_rate_d_chg_rate;
      instance->d_yaw_rate_list[0].valid = 1;
      instance->d_yaw_rate_list[0].timestamp = timestamp;
      instance->d_yaw_rate_list[0].yaw_rate_chg_rate = yaw_rate_chg_rate;
      instance->d_yaw_rate_list[0].yaw_rate_d_chg_rate = yaw_rate_d_chg_rate;
    }
  } else {
    instance->d_yaw_rate_list[0].valid = 1;
    instance->d_yaw_rate_list[0].timestamp = timestamp;
    instance->d_yaw_rate_list[0].yaw_rate_chg_rate = yaw_rate_chg_rate;
    instance->d_yaw_rate_list[0].yaw_rate_d_chg_rate = 0.0F;
  }

  relative_time = Phoenix_Common_CalcElapsedClockMs(
        data_source->rel_pos.timestamp, timestamp);

#if 0
  printf("^^^^^ start_time=%ld, end_time=%ld, relative_time=%ld\n",
          data_source->rel_pos.timestamp, timestamp, relative_time);
#endif

  if (relative_time > 500) {
    LOG_ERR_C("[LAT][PosFilterImpl] Time interval is too large.\n");
  }

  prev_pos[0] = data_source->rel_pos.x;
  prev_pos[1] = data_source->rel_pos.y;
  prev_pos[2] = data_source->rel_pos.heading;
  Phoenix_VehModel_EstimateNextPos(
        velocity, yaw_rate, 0.001F*relative_time, prev_pos, next_pos);

  instance->timestamp = timestamp;
  instance->estimated_pos.relative_time = relative_time;
  instance->estimated_pos.x = next_pos[0];
  instance->estimated_pos.y = next_pos[1];
  instance->estimated_pos.heading = next_pos[2];
  instance->estimated_pos.yaw_rate = yaw_rate;
  instance->estimated_pos.yaw_rate_chg_rate =
      instance->yaw_rate_list[0].yaw_rate_chg_rate;
  instance->estimated_pos.yaw_rate_d_chg_rate =
      instance->d_yaw_rate_list[0].yaw_rate_d_chg_rate;
  instance->estimated_pos.v = velocity;

  printf("chg heading from %0.2f to %0.2f, diff=%0.2f, yaw_rate=%0.4f\n", 
      prev_pos[2]*57.3F, next_pos[2]*57.3F, (next_pos[2]-prev_pos[2])*57.3F, yaw_rate*57.3F);

  return (0);
}
