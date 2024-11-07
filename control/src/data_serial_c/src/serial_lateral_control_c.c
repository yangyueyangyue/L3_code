#include "serial_lateral_control_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/serialization_utils_c.h"
#include "serial_msg_common_c.h"


Int32_t Phoenix_Common_EncodeLateralControlPidInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const LateralControlPidInfo_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_dist), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_dist_near), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_dist_far), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value_near), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value_far), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value_smooth), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err_spd), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_err), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_spd), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.trj_curvature), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].steering_gain), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].tar_steering_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].tar_steering_angle_smooth), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeLateralControlPidInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    LateralControlPidInfo_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.cur_proj_on_trj.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_near.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.h), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_point_far.s), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_dist), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_dist_near), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.goal_dist_far), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value_near), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value_far), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].trj_feed.feed_value_smooth), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err_spd), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.lat_err_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_err), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_spd), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.yaw_rate_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.trj_curvature), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_dynamic_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].steering_gain), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].tar_steering_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].tar_steering_angle_smooth), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlPidInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t Phoenix_Common_EncodeLateralControlInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const LateralControlInfo_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    // 占位
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_ctl_result), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_imu), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_chassis), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_steering), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_chg_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].target_yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.relative_time), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.v), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_spd), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_gross_weight), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeLateralControlPidInfoArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_ctl_pid_info), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].target_steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].target_steering_wheel_angle_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeLateralControlInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    LateralControlInfo_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    // 占位
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_ctl_result), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_imu), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_chassis), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_steering), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_rate_chg_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].target_yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.relative_time), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].curr_pos.v), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].yaw_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_spd), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].veh_gross_weight), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeLateralControlPidInfoArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].lat_ctl_pid_info), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].target_steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeFloat32Array(
          buf, offset + pos, maxlen - pos,
          &(p[i].target_steering_wheel_angle_speed), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode member of LateralControlInfo_t.");
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

