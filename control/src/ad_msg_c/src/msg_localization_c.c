//
#include "msg_localization_c.h"

void Phoenix_AdMsg_ClearGnss(Gnss_t* const data) {
  Phoenix_AdMsg_ClearMsgHead(&(data->msg_head));

  data->latitude = 0.0;
  data->longitude = 0.0;
  data->altitude = 0.0;
  data->heading_gnss = 0.0F;
  data->x_utm = 0.0;
  data->y_utm = 0.0;
  data->z_utm = 0.0;
  data->heading_utm = 0.0F;
  data->x_odom = 0.0;
  data->y_odom = 0.0;
  data->z_odom = 0.0;
  data->heading_odom = 0.0F;
  data->pitch = 0.0F;
  data->roll = 0.0F;
  data->v_e = 0.0F;
  data->v_n = 0.0F;
  data->v_u = 0.0F;
  data->v_x_utm = 0.0F;
  data->v_y_utm = 0.0F;
  data->v_z_utm = 0.0F;
  data->v_x_odom = 0.0F;
  data->v_y_odom = 0.0F;
  data->v_z_odom = 0.0F;
  data->gnss_status = GNSS_STATUS_INVALID;
  data->utm_status = GNSS_STATUS_INVALID;
  data->odom_status = GNSS_STATUS_INVALID;
}

void Phoenix_AdMsg_ClearImu(Imu_t* const data) {
  Phoenix_AdMsg_ClearMsgHead(&(data->msg_head));

  data->yaw_rate = 0;
  data->pitch_rate = 0;
  data->roll_rate = 0;
  data->accel_x = 0;
  data->accel_y = 0;
  data->accel_z = 0;
}

void Phoenix_AdMsg_ClearRelativePos(RelativePos_t* const data) {
  data->relative_time = 0;

  data->x = 0.0F;
  data->y = 0.0F;
  data->heading = 0.0F;
  data->yaw_rate = 0.0F;
  data->yaw_rate_chg_rate = 0.0F;
  data->v = 0.0F;
}
