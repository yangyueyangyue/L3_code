//
#ifndef PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_C_H_
#define PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_C_H_

#include "pos_filter_c.h"


#ifdef __cplusplus
extern "C" {
#endif


void Phoenix_PosFilter_Initialize();

Int32_t Phoenix_PosFilter_Update(
    const PosFilterDataSource_t* data_source);

const RelativePos_t* Phoenix_PosFilter_GetEstimatedPos();

Float32_t Phoenix_PosFilter_GetFilteredYawRate();

Float32_t Phoenix_PosFilter_GetYawRateFromImu();

Float32_t Phoenix_PosFilter_GetYawRateFromSteeringAngle();

Float32_t Phoenix_PosFilter_GetYawRateFromChassis();


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_C_H_
