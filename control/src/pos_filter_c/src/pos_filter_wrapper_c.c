//
#include "pos_filter_wrapper_c.h"
#include "pos_filter_impl_c.h"


static PosFilterImplInstance_t s_pos_filter_instance;

void Phoenix_PosFilter_Initialize() {
  Phoenix_PosFilterImpl_Initialize(&s_pos_filter_instance);
}

Int32_t Phoenix_PosFilter_Update(
    const PosFilterDataSource_t* data_source) {
  return Phoenix_PosFilterImpl_Update(&s_pos_filter_instance, data_source);
}

const RelativePos_t* Phoenix_PosFilter_GetEstimatedPos() {
  return (&s_pos_filter_instance.estimated_pos);
}

Float32_t Phoenix_PosFilter_GetFilteredYawRate() {
  return (s_pos_filter_instance.yaw_rate_filter.yaw_rate_expectation);
}

Float32_t Phoenix_PosFilter_GetYawRateFromImu() {
  return (s_pos_filter_instance.yaw_rate_filter.yaw_rate_from_imu);
}

Float32_t Phoenix_PosFilter_GetYawRateFromSteeringAngle() {
  return (s_pos_filter_instance.yaw_rate_filter.yaw_rate_from_steering);
}

Float32_t Phoenix_PosFilter_GetYawRateFromChassis() {
  return (s_pos_filter_instance.yaw_rate_filter.yaw_rate_from_chassis);
}



