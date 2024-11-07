#include "serial_msg_control.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"
#include "serial_msg_common.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeLateralControlPidInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LateralControlPidInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.h";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.s";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_near.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_near.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_near.h";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_near.s";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_far.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_far.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_far.h";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_point_far.s";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_dist), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_dist";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_dist_near), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_dist_near";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_dist_far), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].goal_dist_far";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value_near), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].feed_value_near";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value_far), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].feed_value_far";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value_smooth), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].feed_value_smooth";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.yaw_err), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.yaw_err";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.yaw_err_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.yaw_err_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err_lv_idx";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err_spd_lv_idx";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_err), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.yaw_rate_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.yaw_rate_err_lv_idx";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.feed_valued";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.trj_curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.trj_curvature";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }


    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_gain), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].steering_gain";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_steering_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].tar_steering_angle";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_steering_angle_smooth), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].tar_steering_angle_smooth";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeLateralControlPidInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LateralControlPidInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.h";
      return (thislen);
    } else {
      pos += thislen;
    }
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.cur_proj_on_trj.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].cur_proj_on_trj.s";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_near.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_near.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_near.h";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_near.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_near.s";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_far.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_far.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.h), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_far.h";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_point_far.s), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_point_far.s";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_dist), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_dist";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_dist_near), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_dist_near";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.goal_dist_far), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].goal_dist_far";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value_near), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].feed_value_near";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value_far), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].feed_value_far";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].trj_feed.feed_value_smooth), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].feed_value_smooth";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.yaw_err), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.yaw_err";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.yaw_err_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.yaw_err_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err_lv_idx";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.lat_err_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.lat_err_spd_lv_idx";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.p_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.i_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.d_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlPidInfo[" << i
              << "].lat_err_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_err), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.yaw_rate_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_err_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.yaw_rate_err_lv_idx";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.yaw_rate_spd_lv_idx), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.p_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.i_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.d_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_err_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].yaw_rate_err_feed.feed_valued";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.trj_curvature), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.trj_curvature";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.p_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.p_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.i_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.i_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.v_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.v_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.w_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.w_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.k_ratio), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.k_ratio";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.k_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.k_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.d_feed.feed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.d_feed.feed";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_dynamic_feed.feed_value), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].veh_dynamic_feed.feed_value";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].steering_gain), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].steering_gain";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_steering_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].tar_steering_angle";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].tar_steering_angle_smooth), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlPidInfo[" << i
              << "].tar_steering_angle_smooth";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


Int32_t EncodeLateralControlInfoArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::LateralControlInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_ctl_result), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].lat_ctl_result";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_imu), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_rate_imu";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_chassis), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_rate_chassis";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_steering), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_rate_steering";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_chg_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_rate_chg_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].target_yaw_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.relative_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].curr_pos.relative_time";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].curr_pos.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].curr_pos.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].curr_pos.heading";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].curr_pos.yaw_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].curr_pos.v";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].lat_err[0]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].lat_err_chg_rate[1]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_err[0]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].yaw_err_chg_rate[0]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].veh_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_gross_weight), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].veh_gross_weight";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeLateralControlPidInfoArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_ctl_pid_info), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].veh_gross_weight";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].target_steering_wheel_angle";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_steering_wheel_angle_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode member of LateralControlInfo[" << i
              << "].target_steering_wheel_angle_speed";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeLateralControlInfoArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::LateralControlInfo* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_ctl_result), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].lat_ctl_result";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_imu), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_rate_imu";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_chassis), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_rate_chassis";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_steering), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_rate_steering";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_rate_chg_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_rate_chg_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_rate_chg_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.relative_time), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].curr_pos.relative_time";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.x), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].curr_pos.x";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.y), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].curr_pos.y";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.heading), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].curr_pos.heading";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.yaw_rate), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].curr_pos.yaw_rate";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].curr_pos.v), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].curr_pos.v";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].lat_err[0]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].lat_err_chg_rate[1]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_err[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_err[0]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].yaw_err_chg_rate[0]), 2);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].yaw_err_chg_rate[0]";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_spd), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].veh_spd";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].veh_gross_weight), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].veh_gross_weight";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeLateralControlPidInfoArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].lat_ctl_pid_info), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].veh_gross_weight";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_steering_wheel_angle), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].target_steering_wheel_angle";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].target_steering_wheel_angle_speed), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode member of LateralControlInfo[" << i
              << "].target_steering_wheel_angle_speed";
      return (thislen);
    } else {
      pos += thislen;
    }
  }
  return pos;
}


}  // namespace data_serial
}  // namespace phoenix
