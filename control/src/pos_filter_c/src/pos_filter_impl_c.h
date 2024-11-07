/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       pos_filter_impl.h
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
#ifndef PHOENIX_POS_FILTER_POS_FILTER_IMPL_C_H_
#define PHOENIX_POS_FILTER_POS_FILTER_IMPL_C_H_

#include "utils/macros.h"
#include "pos_filter_c.h"
#include "yaw_rate_filter_kalman_c.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef struct _PosFilterYawRateChgRateInfo_t PosFilterYawRateChgRateInfo_t;
struct _PosFilterYawRateChgRateInfo_t {
  Int8_t valid;
  Int64_t timestamp;
  Float32_t yaw_rate;
  Float32_t prev_yaw_rate_chg_rate;
  Float32_t yaw_rate_chg_rate_list[3];
};

typedef struct _PosFilterImplInstance_t PosFilterImplInstance_t;
struct _PosFilterImplInstance_t {
  Int64_t timestamp;

  RelativePos_t estimated_pos;

  struct {
    Int8_t valid;
    Int64_t timestamp;
    Float32_t yaw_rate;
    Float32_t yaw_rate_chg_rate;
  } yaw_rate_list[3];

  PosFilterYawRateChgRateInfo_t yaw_rate_chg_rate_info;

  struct {
    Int8_t valid;
    Int64_t timestamp;
    Float32_t yaw_rate_chg_rate;
    Float32_t yaw_rate_d_chg_rate;
  } d_yaw_rate_list[3];

  YawRateFilterKalmanInstance_t yaw_rate_filter;
};

void Phoenix_PosFilterImpl_Initialize(
    PosFilterImplInstance_t* instance);

Int32_t Phoenix_PosFilterImpl_Update(
    PosFilterImplInstance_t* instance,
    const PosFilterDataSource_t* data_source);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_POS_FILTER_POS_FILTER_IMPL_C_H_
