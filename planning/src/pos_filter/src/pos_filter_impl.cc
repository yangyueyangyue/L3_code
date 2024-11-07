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
#include "pos_filter_impl.h"
#include "utils/linear_interpolation.h"
#include "utils/gps_tools.h"


#define ENABLE_POS_FILTER_IMPL_TRACE (0)


namespace phoenix {
namespace pos_filter {


PosFilterImpl::PosFilterImpl() {
  curr_velocity_ = 0.0F;
  curr_yaw_rate_ = 0.0F;
  pos_filter_timeinterval_ = 0;

  prev_lane_mark_valid_ = false;
  prev_lane_mark_msg_sequence_ = 0;
  prev_lane_mark_timestamp_ = 0;

  prev_filtered_pos_valid_ = false;
  prev_filtered_pos_timestamp_ = 0;

  Reset();
}

PosFilterImpl::~PosFilterImpl() {
  // nothing to do
}

void PosFilterImpl::Configurate(const PosFilterConfig& conf) {
  lane_mark_camera_filter_.Configurate(conf);
}

void PosFilterImpl::Reset() {
  prev_lane_mark_valid_ = false;

  prev_gnss_valid_ = false;
  localization_status_ = ad_msg::Gnss::STATUS_INVALID;
  prev_localization_status_ = ad_msg::Gnss::STATUS_INVALID;

  prev_filtered_pos_valid_ = false;

  mat_covariance_.SetIdentity();

  Int32_t filtered_pos_list_size = filtered_pos_list_.Size();
  for (Int32_t i = 0; i < filtered_pos_list_size; ++i) {
    filtered_pos_list_[i].Clear();
  }
  filtered_pos_list_.Clear();

  prev_gnss_info_.Clear();
  filtered_gnss_info_.Clear();

  gnss_track_list_.Clear();
  gnss_filtered_track_list_.Clear();
  utm_track_list_.Clear();
  utm_filtered_track_list_.Clear();
  odom_track_list_.Clear();
  odom_filtered_track_list_.Clear();
}

/* k004 pengc 2022-12-26 (begin) */
// 使用相机车道线矫正地图定位
bool PosFilterImpl::CorrectGnssCoordinate(
    Int32_t type, const Float32_t delta_pos[3]) {
  /// TODO: 暂时只矫正ODOM

  common::Matrix<Float64_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<Float64_t, 2, 1> point_conv;
  point_conv.SetZeros();
  common::Matrix<Float64_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  common::Rotate_2D<Float64_t>(rotate_center, delta_pos[2], &mat_conv);
  common::Translate_2D<Float64_t>(delta_pos[0], delta_pos[1], &mat_conv);

  if (1 == type) {
    // 只矫正GPS
  } else if (2 == type) {
    // 只矫正UTM
  } else if (3 == type) {
    // 只矫正ODOM
    point_conv(0) = filtered_gnss_info_.x_odom;
    point_conv(1) = filtered_gnss_info_.y_odom;
    common::TransformVert_2D(mat_conv, &point_conv);
    filtered_gnss_info_.x_odom = point_conv(0);
    filtered_gnss_info_.y_odom = point_conv(1);
    filtered_gnss_info_.heading_odom =
        common::NormalizeAngle(filtered_gnss_info_.heading_odom + delta_pos[2]);
  } else {
    // 全部矫正
  }

  return true;
}
/* k004 pengc 2022-12-26 (end) */

bool PosFilterImpl::UpdateOdomCoordinate(
    const common::Matrix<Float64_t, 3, 3>& mat_conv, Float32_t delta_heading) {
  common::Matrix<Float64_t, 2, 1> point_conv;

  // prev_gnss_info_ odom
  point_conv(0) = prev_gnss_info_.x_odom;
  point_conv(1) = prev_gnss_info_.y_odom;
  common::TransformVert_2D(mat_conv, &point_conv);
  prev_gnss_info_.x_odom = point_conv(0);
  prev_gnss_info_.y_odom = point_conv(1);
  prev_gnss_info_.heading_odom =
      common::NormalizeAngle(prev_gnss_info_.heading_odom + delta_heading);

  // filtered_gnss_info_ odom
  point_conv(0) = filtered_gnss_info_.x_odom;
  point_conv(1) = filtered_gnss_info_.y_odom;
  common::TransformVert_2D(mat_conv, &point_conv);
  filtered_gnss_info_.x_odom = point_conv(0);
  filtered_gnss_info_.y_odom = point_conv(1);
  filtered_gnss_info_.heading_odom =
      common::NormalizeAngle(filtered_gnss_info_.heading_odom + delta_heading);

  // odom_track_list_
  for (Int32_t i = 0; i < odom_track_list_.Size(); ++i) {
    OdomPoint& pos = odom_track_list_[i];

    point_conv(0) = pos.x;
    point_conv(1) = pos.y;
    common::TransformVert_2D(mat_conv, &point_conv);
    pos.x = point_conv(0);
    pos.y = point_conv(1);
    pos.heading = common::NormalizeAngle(pos.heading + delta_heading);
  }

  // odom_filtered_track_list_
  for (Int32_t i = 0; i < odom_track_list_.Size(); ++i) {
    OdomPoint& pos = odom_filtered_track_list_[i];

    point_conv(0) = pos.x;
    point_conv(1) = pos.y;
    common::TransformVert_2D(mat_conv, &point_conv);
    pos.x = point_conv(0);
    pos.y = point_conv(1);
    pos.heading = common::NormalizeAngle(pos.heading + delta_heading);
  }

  return true;
}

bool PosFilterImpl::Update(
    const PosFilterDataSource& data_source) {

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "\n### PosFilterImpl::Update (Begin) ###" << std::endl;
#endif
  Int64_t timestamp = data_source.timestamp;

  if (prev_lane_mark_valid_) {
    Int64_t time_elapsed = common::CalcElapsedClockMs(
          prev_lane_mark_timestamp_, timestamp);
    if (time_elapsed > 500 || time_elapsed < 0) {
      // 旧的数据过时了
      prev_lane_mark_valid_ = false;
      lane_mark_camera_filter_.Reset();
    }
  }
  if (prev_gnss_valid_) {
    Int64_t time_elapsed = common::CalcElapsedClockMs(
          prev_gnss_info_.msg_head.timestamp, timestamp);
    if (time_elapsed > 500 || time_elapsed < 0) {
      // 旧的数据过时了
      prev_gnss_valid_ = false;
    }
  }
  if (prev_filtered_pos_valid_) {
    Int64_t time_elapsed = common::CalcElapsedClockMs(
          prev_filtered_pos_timestamp_, timestamp);
    if (time_elapsed > 500 || time_elapsed < 0) {
      // 旧的数据过时了
      prev_filtered_pos_valid_ = false;
    }
  }

  bool ret = yaw_rate_filter_.Update(timestamp, data_source);
  if (false == ret) {
    LOG_ERR << "Failed to filter yaw_rate.";
    return false;
  }

  bool chassis_valid = false;
  if (Nullptr_t != data_source.chassis) {
    if (data_source.chassis->msg_head.valid) {
      if (data_source.chassis->v_valid) {
        chassis_valid = true;
      }
    }
  }

  bool gnss_valid = false;
  bool utm_valid = false;
  bool odom_valid = false;
  if (Nullptr_t != data_source.gnss) {
    if (data_source.gnss->msg_head.valid) {
      if ((ad_msg::Gnss::STATUS_CONVERGING == data_source.gnss->gnss_status) ||
          (ad_msg::Gnss::STATUS_GOOD == data_source.gnss->gnss_status) ||
          (ad_msg::Gnss::STATUS_BAD == data_source.gnss->gnss_status)) {
        gnss_valid = true;

        GnssPoint* gnss_point = gnss_track_list_.AllocateOverride();
        gnss_point->latitude = data_source.gnss->latitude;
        gnss_point->longitude = data_source.gnss->longitude;
        gnss_point->heading = data_source.gnss->heading_gnss;
      }
      if ((ad_msg::Gnss::STATUS_CONVERGING == data_source.gnss->utm_status) ||
          (ad_msg::Gnss::STATUS_GOOD == data_source.gnss->utm_status) ||
          (ad_msg::Gnss::STATUS_BAD == data_source.gnss->utm_status)) {
        utm_valid = true;

        UtmPoint* utm_point = utm_track_list_.AllocateOverride();
        utm_point->x = data_source.gnss->x_utm;
        utm_point->y = data_source.gnss->y_utm;
        utm_point->heading = data_source.gnss->heading_utm;
      }
      if ((ad_msg::Gnss::STATUS_CONVERGING == data_source.gnss->odom_status) ||
          (ad_msg::Gnss::STATUS_GOOD == data_source.gnss->odom_status) ||
          (ad_msg::Gnss::STATUS_BAD == data_source.gnss->odom_status)) {
        odom_valid = true;

        OdomPoint* odom_point = odom_track_list_.AllocateOverride();
        odom_point->x = data_source.gnss->x_odom;
        odom_point->y = data_source.gnss->y_odom;
        odom_point->heading = data_source.gnss->heading_odom;
      }
    }
  }

  Scalar velocity = 0;
  if (chassis_valid) {
    velocity = data_source.chassis->v;
  } else if (gnss_valid) {
    velocity = common::com_sqrt(common::Square(data_source.gnss->v_e) +
                                common::Square(data_source.gnss->v_n));
  } else if (utm_valid) {
    velocity = common::com_sqrt(common::Square(data_source.gnss->v_x_utm) +
                                common::Square(data_source.gnss->v_y_utm));
  } else if (odom_valid) {
    velocity = common::com_sqrt(common::Square(data_source.gnss->v_x_odom) +
                                common::Square(data_source.gnss->v_y_odom));
  } else {
    LOG_ERR << "Can't get valid velocity of vehilce.";
    return false;
  }

  if (ad_msg::VEH_GEAR_R == data_source.chassis->gear) {
    // For backing mode of vehicle
    velocity = -velocity;
  }

  Scalar yaw_rate = yaw_rate_filter_.GetFilteredYawRate();
  Scalar yaw_rate_chg_rate = yaw_rate_filter_.GetYawRateChgRate();

  curr_velocity_ = velocity;
  curr_yaw_rate_ = yaw_rate;

  Int64_t filter_pos_time_elapsed =
      common::CalcElapsedClockMs(prev_filtered_pos_timestamp_, timestamp);
  if (filter_pos_time_elapsed > 200) {
    prev_filtered_pos_valid_ = false;
    Int32_t filtered_pos_list_size = filtered_pos_list_.Size();
    for (Int32_t i = 0; i < filtered_pos_list_size; ++i) {
      filtered_pos_list_[i].Clear();
    }
    filtered_pos_list_.Clear();
  }
  pos_filter_timeinterval_ = filter_pos_time_elapsed;

  bool lane_mark_valid_bel = false;
  FilterRet lane_mark_filter_ret;
  if (Nullptr_t != data_source.lane_mark_camera_list) {
    lane_mark_valid_bel = EstimatePosByLaneMarkCameraList(
          velocity, yaw_rate, *data_source.lane_mark_camera_list,
          &lane_mark_filter_ret);
    // 根据车道线的匹配结果修正角速度
    if (lane_mark_valid_bel) {
      yaw_rate_filter_.CorrectYawRate(lane_mark_filter_ret.corrected_yaw_rate);
    }
  }

  bool gnss_valid_bel = false;
  FilterRet gnss_filter_ret;
  if (gnss_valid) {
    localization_status_ = data_source.gnss->gnss_status;
    prev_localization_status_ = prev_gnss_info_.gnss_status;
    gnss_valid_bel = EstimatePosByGnss(
          velocity, yaw_rate, LOCALIZATION_TYPE_GNSS, *data_source.gnss,
          &gnss_filter_ret);
  } else if (utm_valid) {
    localization_status_ = data_source.gnss->utm_status;
    prev_localization_status_ = prev_gnss_info_.utm_status;
    gnss_valid_bel = EstimatePosByGnss(
          velocity, yaw_rate, LOCALIZATION_TYPE_UTM, *data_source.gnss,
          &gnss_filter_ret);
  } else if (odom_valid) {
    localization_status_ = data_source.gnss->odom_status;
    prev_localization_status_ = prev_gnss_info_.odom_status;
    gnss_valid_bel = EstimatePosByGnss(
          velocity, yaw_rate, LOCALIZATION_TYPE_ODOM, *data_source.gnss,
          &gnss_filter_ret);
  } else {
    // Nothing to do
  }

  /* k004 pengc 2022-12-26 (begin) */
  // 设置GNSS定位精度不可信任
  if (data_source.do_not_trust_gnss) {
    gnss_valid_bel = false;
  }
  /* k004 pengc 2022-12-26 (end) */

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "lane_mark_valid_bel=" << lane_mark_valid_bel
            << ", gnss_valid_bel=" << gnss_valid_bel
            << std::endl;
#endif

  //gnss_valid_bel = false;

  bool pos_filtered_valid = false;
  FilterRet pos_filtered;
  if (!filtered_pos_list_.Empty()) {
#if 1
    if (lane_mark_valid_bel && gnss_valid_bel) {
      // std::cout << ">>> FilterPosByLaneMarkAndGnss" << std::endl;
      FilterPosByLaneMarkAndGnss(
            lane_mark_filter_ret, gnss_filter_ret, &pos_filtered);
      pos_filtered_valid = true;
    } else if (lane_mark_valid_bel && !gnss_valid_bel) {
      // std::cout << ">>> FilterPosByLaneMark" << std::endl;
      FilterPosByLaneMark(lane_mark_filter_ret, &pos_filtered);
      pos_filtered_valid = true;
    } else if (!lane_mark_valid_bel && gnss_valid_bel) {
      // std::cout << ">>> FilterPosByGnss" << std::endl;
      FilterPosByGnss(gnss_filter_ret, &pos_filtered);
      pos_filtered_valid = true;
    } else {
      // std::cout << ">>> PredictPosByVehStatus" << std::endl;
      PredictPosByVehStatus(&pos_filtered);
      pos_filtered_valid = false;
    }
#else
//    if (lane_mark_valid_bel) {
//      FilterPosByLaneMark(lane_mark_filter_ret, &pos_filtered);
//      pos_filtered_valid = true;
//    } else {
//      PredictPosByVehStatus(&pos_filtered);
//      pos_filtered_valid = false;
//    }
    if (gnss_valid_bel) {
      FilterPosByGnss(gnss_filter_ret, &pos_filtered);
      pos_filtered_valid = true;
    } else {
      PredictPosByVehStatus(&pos_filtered);
      pos_filtered_valid = false;
    }
#endif
  }

  if (filtered_pos_list_.Empty()) {
    // 之前没有有效的相对位置列表信息
    FilteredPos* pos = filtered_pos_list_.Allocate();
    if (Nullptr_t != pos) {
      pos->relative_time = 0;
      pos->x = 0.0F;
      pos->y = 0.0F;
      pos->heading = 0.0F;
      pos->yaw_rate = yaw_rate;
      pos->yaw_rate_chg_rate = yaw_rate_chg_rate;
      pos->v = velocity;
      pos->delta_x = 0.0F;
      pos->delta_y = 0.0F;
      pos->delta_heading = 0.0F;
    }
  } else {
    // 之前存在有效的相对位置列表信息
    FilteredPos& first_pos = filtered_pos_list_.Front();

    Scalar prev_pos[3] = { 0.0F };
    Scalar curr_pos[3] = { 0.0F };
    if (pos_filtered_valid) {
#if 0
      SmoothPosBtwTimestampsInList(lane_mark_filter_ret.time_start,
                                   lane_mark_filter_ret.time_end,
                                   lane_mark_filter_ret.estimated_pos_start,
                                   lane_mark_filter_ret.corrected_delta_pos);
      Int64_t time_diff = common::CalcElapsedClockMs(
            data_source.lane_mark_camera_list->msg_head.timestamp, timestamp);


      prev_pos[0] = lane_mark_filter_ret.estimated_pos_start.x +
          lane_mark_filter_ret.corrected_delta_pos[0];
      prev_pos[1] = lane_mark_filter_ret.estimated_pos_start.y +
          lane_mark_filter_ret.corrected_delta_pos[1];
      prev_pos[2] = common::NormalizeAngle(
            lane_mark_filter_ret.estimated_pos_start.heading +
            lane_mark_filter_ret.corrected_delta_pos[2]);

      vehicle_model_.EstimateNextPos(
            lane_mark_filter_ret.estimated_pos_end.v,
            lane_mark_filter_ret.estimated_pos_end.yaw_rate,
            0.001F*time_diff, prev_pos, curr_pos);
#else
      SmoothPosBtwTimestampsInList(pos_filtered.time_start,
                                   pos_filtered.time_end,
                                   pos_filtered.estimated_pos_start,
                                   pos_filtered.corrected_delta_pos);
      Int32_t time_diff = pos_filter_timeinterval_ - pos_filtered.time_end;
      if (time_diff < 0) {
        time_diff = 0;
        LOG_ERR << "Detected unexpected timeinterval (" << time_diff << ").";
      }

#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "time_diff=" << time_diff << std::endl;
#endif

      prev_pos[0] = pos_filtered.estimated_pos_end.x;
      prev_pos[1] = pos_filtered.estimated_pos_end.y;
      prev_pos[2] = pos_filtered.estimated_pos_end.heading;

      vehicle_model_.EstimateNextPos(
            pos_filtered.estimated_pos_end.v,
            pos_filtered.estimated_pos_end.yaw_rate,
            0.001F*time_diff, prev_pos, curr_pos);
#endif
    } else {
#if 0
      Int64_t time_diff = common::CalcElapsedClockMs(
            prev_filtered_pos_timestamp_, timestamp);

      prev_pos[0] = first_pos.x;
      prev_pos[1] = first_pos.y;
      prev_pos[2] = first_pos.heading;

#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "lane mark is invalid, estimate pose by state of vehicle,"
                << "time_diff=" << time_diff
                << ", v=" << first_pos.v*3.6F
                << ", yaw_rate=" << first_pos.yaw_rate
                << std::endl;
#endif

      vehicle_model_.EstimateNextPos(
            first_pos.v, first_pos.yaw_rate,
            0.001F*time_diff, prev_pos, curr_pos);
#else
      prev_pos[0] = first_pos.x;
      prev_pos[1] = first_pos.y;
      prev_pos[2] = first_pos.heading;

      curr_pos[0] = pos_filtered.estimated_pos_end.x;
      curr_pos[1] = pos_filtered.estimated_pos_end.y;
      curr_pos[2] = pos_filtered.estimated_pos_end.heading;
#endif
    }

    common::Matrix<Scalar, 3, 3> mat_convert;
    common::Matrix<Scalar, 2, 1> rotate_center;
    common::Matrix<Scalar, 2, 1> point_conv;
    rotate_center.SetZeros();
    mat_convert.SetIdentity();
    //rotate_center(0) = curr_pos[0];
    //rotate_center(1) = curr_pos[1];
    common::Translate_2D(-curr_pos[0], -curr_pos[1], &mat_convert);
    common::Rotate_2D<Scalar>(rotate_center, -curr_pos[2], &mat_convert);

    Int32_t filtered_pos_list_size = filtered_pos_list_.Size();
    if (!filtered_pos_list_.Full()) {
      const FilteredPos& last_pos = filtered_pos_list_.Back();
      FilteredPos* pos = filtered_pos_list_.Allocate();
      if (Nullptr_t != pos) {
        point_conv(0) = last_pos.x;
        point_conv(1) = last_pos.y;
        common::TransformVert_2D(mat_convert, &point_conv);

        pos->relative_time = last_pos.relative_time - filter_pos_time_elapsed;
        pos->x = point_conv(0);
        pos->y = point_conv(1);
        pos->heading = common::NormalizeAngle(last_pos.heading - curr_pos[2]);
        pos->yaw_rate = last_pos.yaw_rate;
        pos->yaw_rate_chg_rate = last_pos.yaw_rate_chg_rate;
        pos->v = last_pos.v;
      }
    }

    for (Int32_t i = (filtered_pos_list_size-1); i >= 1; --i) {
      FilteredPos& pos = filtered_pos_list_[i];
      FilteredPos& next_pos = filtered_pos_list_[i-1];

      point_conv(0) = next_pos.x;
      point_conv(1) = next_pos.y;
      common::TransformVert_2D(mat_convert, &point_conv);

      pos.relative_time = next_pos.relative_time - filter_pos_time_elapsed;
      pos.x = point_conv(0);
      pos.y = point_conv(1);
      pos.heading = common::NormalizeAngle(next_pos.heading - curr_pos[2]);
      pos.yaw_rate = next_pos.yaw_rate;
      pos.yaw_rate_chg_rate = next_pos.yaw_rate_chg_rate;
      pos.v = next_pos.v;
    }

    first_pos.relative_time = 0;
    first_pos.x = 0;
    first_pos.y = 0;
    first_pos.heading = 0;
    first_pos.yaw_rate = yaw_rate;
    first_pos.yaw_rate_chg_rate = yaw_rate_chg_rate;
    first_pos.v = velocity;

    filtered_pos_list_size = filtered_pos_list_.Size();
    for (Int32_t i = 0; i < (filtered_pos_list_size-1); ++i) {
      FilteredPos& pos = filtered_pos_list_[i];
      const FilteredPos& next_pos = filtered_pos_list_[i+1];
      pos.delta_x = pos.x - next_pos.x;
      pos.delta_y = pos.y - next_pos.y;
      pos.delta_heading =
          common::NormalizeAngle(pos.heading - next_pos.heading);
    }
    filtered_pos_list_[filtered_pos_list_size-1].delta_x = 0.0F;
    filtered_pos_list_[filtered_pos_list_size-1].delta_y = 0.0F;
    filtered_pos_list_[filtered_pos_list_size-1].delta_heading = 0.0F;
  }

  prev_filtered_pos_valid_ = true;
  prev_filtered_pos_timestamp_ = timestamp;
  filtered_pos_list_msg_head_.valid = true;
  filtered_pos_list_msg_head_.UpdateSequenceNum();
  filtered_pos_list_msg_head_.timestamp = timestamp;

  if (Nullptr_t != data_source.lane_mark_camera_list) {
    FilterLaneMark(
          timestamp, *data_source.lane_mark_camera_list,
          lane_mark_valid_bel, lane_mark_filter_ret);
  }

  if (Nullptr_t != data_source.gnss) {
    FilterGnss(timestamp, *data_source.gnss, gnss_valid_bel);

    GnssPoint* gnss_point = gnss_filtered_track_list_.AllocateOverride();
    gnss_point->latitude = filtered_gnss_info_.latitude;
    gnss_point->longitude = filtered_gnss_info_.longitude;
    gnss_point->heading = filtered_gnss_info_.heading_gnss;

    UtmPoint* utm_point = utm_filtered_track_list_.AllocateOverride();
    utm_point->x = filtered_gnss_info_.x_utm;
    utm_point->y = filtered_gnss_info_.y_utm;
    utm_point->heading = filtered_gnss_info_.heading_utm;

    OdomPoint* odom_point = odom_filtered_track_list_.AllocateOverride();
    odom_point->x = filtered_gnss_info_.x_odom;
    odom_point->y = filtered_gnss_info_.y_odom;
    odom_point->heading = filtered_gnss_info_.heading_odom;
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "The filtered poses list is:" << std::endl;
  for (Int32_t i = 0; i < filtered_pos_list_.Size(); ++i) {
    const FilteredPos& pos = filtered_pos_list_[i];

    std::cout << "[" << i << "] t=" << pos.relative_time
              << ", x=" << pos.x << ", y=" << pos.y
              << ", h=" << common::com_rad2deg(pos.heading)
              << ", dx=" << pos.delta_x << ", dy=" << pos.delta_y
              << ", dh=" << common::com_rad2deg(pos.delta_heading)
              << std::endl;
  }

  std::cout << "### PosFilterImpl::Update (End) ###" << std::endl;
#endif

  return true;
}

void PosFilterImpl::GetRelativePosList(
    ad_msg::RelativePosList* pos_list) const {
  Int32_t pos_num = filtered_pos_list_.Size();
  if (pos_num > ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
    pos_num = ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM;
  }

  for (Int32_t i = 0; i < pos_num; ++i) {
    const FilteredPos& pos = filtered_pos_list_[i];
    ad_msg::RelativePos& pos_out = pos_list->relative_pos[i];

    pos_out.relative_time = pos.relative_time;
    pos_out.x = pos.x;
    pos_out.y = pos.y;
    pos_out.heading = pos.heading;
    pos_out.yaw_rate = pos.yaw_rate;
    pos_out.yaw_rate_chg_rate = pos.yaw_rate_chg_rate;
    pos_out.v = pos.v;
  }
  pos_list->relative_pos_num = pos_num;
  pos_list->msg_head = filtered_pos_list_msg_head_;
}

void PosFilterImpl::GetGnssTrackList(
    ad_msg::RelativePosList* track_list) const {
  if (gnss_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }
  if (gnss_filtered_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }

  //const GnssPoint* last_gnss_point = gnss_track_list_.Back();
  const GnssPoint* last_gnss_point = gnss_filtered_track_list_.Back();

  common::GpsPoint gps_origin;
  gps_origin.latitude = last_gnss_point->latitude;
  gps_origin.longitude = last_gnss_point->longitude;
  common::GpsPoint gps_point;

  common::Vec2d local_point;

  common::Matrix<Scalar, 3, 3> mat_convert;
  common::Matrix<Scalar, 2, 1> rotate_center;
  common::Matrix<Scalar, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Rotate_2D<Scalar>(
        rotate_center, -last_gnss_point->heading, &mat_convert);

  common::RingBuffer<GnssPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it = gnss_track_list_.cbegin();
  common::RingBuffer<GnssPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it_end = gnss_track_list_.cend();
  Int32_t index = 0;
  for (; it != it_end; ++it, ++index) {
    if (index >= ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
      break;
    }
    gps_point.latitude = it->latitude;
    gps_point.longitude = it->longitude;

    common::ConvGpsPointToLocalPoint(gps_origin, gps_point, &local_point);

    point_conv(0) = local_point.x();
    point_conv(1) = local_point.y();
    common::TransformVert_2D(mat_convert, &point_conv);

    track_list->relative_pos[index].x = point_conv(0);
    track_list->relative_pos[index].y = point_conv(1);
    track_list->relative_pos[index].heading =
        common::AngleDiff(last_gnss_point->heading, it->heading);
  }
  track_list->relative_pos_num = index;
}

void PosFilterImpl::GetGnssFilteredTrackList(
    ad_msg::RelativePosList* track_list) const {
  if (gnss_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }
  if (gnss_filtered_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }

  const GnssPoint* last_gnss_point = gnss_filtered_track_list_.Back();

  common::GpsPoint gps_origin;
  gps_origin.latitude = last_gnss_point->latitude;
  gps_origin.longitude = last_gnss_point->longitude;
  common::GpsPoint gps_point;

  common::Vec2d local_point;

  common::Matrix<Scalar, 3, 3> mat_convert;
  common::Matrix<Scalar, 2, 1> rotate_center;
  common::Matrix<Scalar, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Rotate_2D<Scalar>(
        rotate_center, -last_gnss_point->heading, &mat_convert);

  common::RingBuffer<GnssPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it = gnss_filtered_track_list_.cbegin();
  common::RingBuffer<GnssPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it_end = gnss_filtered_track_list_.cend();
  Int32_t index = 0;
  for (; it != it_end; ++it, ++index) {
    if (index >= ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
      break;
    }
    gps_point.latitude = it->latitude;
    gps_point.longitude = it->longitude;

    common::ConvGpsPointToLocalPoint(gps_origin, gps_point, &local_point);

    point_conv(0) = local_point.x();
    point_conv(1) = local_point.y();
    common::TransformVert_2D(mat_convert, &point_conv);

    track_list->relative_pos[index].x = point_conv(0);
    track_list->relative_pos[index].y = point_conv(1);
    track_list->relative_pos[index].heading =
        common::AngleDiff(last_gnss_point->heading, it->heading);
  }
  track_list->relative_pos_num = index;
}

void PosFilterImpl::GetUtmTrackList(
    ad_msg::RelativePosList* track_list) const {
  if (utm_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }
  if (utm_filtered_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }

  //const UtmPoint* last_utm_point = utm_track_list_.Back();
  const UtmPoint* last_utm_point = utm_filtered_track_list_.Back();

  common::Matrix<Float64_t, 3, 3> mat_convert;
  common::Matrix<Float64_t, 2, 1> rotate_center;
  common::Matrix<Float64_t, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Translate_2D<Float64_t>(
        -last_utm_point->x, -last_utm_point->y, &mat_convert);
  common::Rotate_2D<Float64_t>(
        rotate_center, -last_utm_point->heading, &mat_convert);

  common::RingBuffer<UtmPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it = utm_track_list_.cbegin();
  common::RingBuffer<UtmPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it_end = utm_track_list_.cend();
  Int32_t index = 0;
  for (; it != it_end; ++it, ++index) {
    if (index >= ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
      break;
    }

    point_conv(0) = it->x;
    point_conv(1) = it->y;
    common::TransformVert_2D(mat_convert, &point_conv);
    track_list->relative_pos[index].x = point_conv(0);
    track_list->relative_pos[index].y = point_conv(1);
    track_list->relative_pos[index].heading =
        common::AngleDiff(last_utm_point->heading, it->heading);
  }
  track_list->relative_pos_num = index;
}

void PosFilterImpl::GetUtmFilteredTrackList(
    ad_msg::RelativePosList* track_list) const {
  if (utm_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }
  if (utm_filtered_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }

  //const UtmPoint* last_utm_point = utm_track_list_.Back();
  const UtmPoint* last_utm_point = utm_filtered_track_list_.Back();

  common::Matrix<Float64_t, 3, 3> mat_convert;
  common::Matrix<Float64_t, 2, 1> rotate_center;
  common::Matrix<Float64_t, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Translate_2D<Float64_t>(
        -last_utm_point->x, -last_utm_point->y, &mat_convert);
  common::Rotate_2D<Float64_t>(
        rotate_center, -last_utm_point->heading, &mat_convert);

  common::RingBuffer<UtmPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it = utm_filtered_track_list_.cbegin();
  common::RingBuffer<UtmPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it_end = utm_filtered_track_list_.cend();
  Int32_t index = 0;
  for (; it != it_end; ++it, ++index) {
    if (index >= ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
      break;
    }

    point_conv(0) = it->x;
    point_conv(1) = it->y;
    common::TransformVert_2D(mat_convert, &point_conv);

    track_list->relative_pos[index].x = point_conv(0);
    track_list->relative_pos[index].y = point_conv(1);
    track_list->relative_pos[index].heading =
        common::AngleDiff(last_utm_point->heading, it->heading);
  }
  track_list->relative_pos_num = index;
}

void PosFilterImpl::GetOdomTrackList(
    ad_msg::RelativePosList* track_list) const {
  if (odom_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }
  if (odom_filtered_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }

  const OdomPoint* last_odom_point = odom_filtered_track_list_.Back();

  common::Matrix<Float64_t, 3, 3> mat_convert;
  common::Matrix<Float64_t, 2, 1> rotate_center;
  common::Matrix<Float64_t, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Translate_2D<Float64_t>(
        -last_odom_point->x, -last_odom_point->y, &mat_convert);
  common::Rotate_2D<Float64_t>(
        rotate_center, -last_odom_point->heading, &mat_convert);

  common::RingBuffer<OdomPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it = odom_track_list_.cbegin();
  common::RingBuffer<OdomPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it_end = odom_track_list_.cend();
  Int32_t index = 0;
  for (; it != it_end; ++it, ++index) {
    if (index >= ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
      break;
    }

    point_conv(0) = it->x;
    point_conv(1) = it->y;
    common::TransformVert_2D(mat_convert, &point_conv);
    track_list->relative_pos[index].x = point_conv(0);
    track_list->relative_pos[index].y = point_conv(1);
    track_list->relative_pos[index].heading =
        common::AngleDiff(last_odom_point->heading, it->heading);
  }
  track_list->relative_pos_num = index;
}

void PosFilterImpl::GetOdomFilteredTrackList(
    ad_msg::RelativePosList* track_list) const {
  if (odom_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }
  if (odom_filtered_track_list_.Empty()) {
    track_list->relative_pos_num = 0;
    return;
  }

  const OdomPoint* last_odom_point = odom_filtered_track_list_.Back();

  common::Matrix<Float64_t, 3, 3> mat_convert;
  common::Matrix<Float64_t, 2, 1> rotate_center;
  common::Matrix<Float64_t, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Translate_2D<Float64_t>(
        -last_odom_point->x, -last_odom_point->y, &mat_convert);
  common::Rotate_2D<Float64_t>(
        rotate_center, -last_odom_point->heading, &mat_convert);

  common::RingBuffer<OdomPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it = odom_filtered_track_list_.cbegin();
  common::RingBuffer<OdomPoint, MAX_POS_TRACK_LIST_SIZE>::const_iterator
      it_end = odom_filtered_track_list_.cend();
  Int32_t index = 0;
  for (; it != it_end; ++it, ++index) {
    if (index >= ad_msg::RelativePosList::MAX_RELATIVE_POS_NUM) {
      break;
    }

    point_conv(0) = it->x;
    point_conv(1) = it->y;
    common::TransformVert_2D(mat_convert, &point_conv);

    track_list->relative_pos[index].x = point_conv(0);
    track_list->relative_pos[index].y = point_conv(1);
    track_list->relative_pos[index].heading =
        common::AngleDiff(last_odom_point->heading, it->heading);
  }
  track_list->relative_pos_num = index;
}

void PosFilterImpl::GetPosFilterInfo(PosFilterInfo* info) const {
  info->yaw_rate_info.yaw_rate_steering =
      yaw_rate_filter_.GetYawRateFromSteeringAngle();
  info->yaw_rate_info.yaw_rate_imu = yaw_rate_filter_.GetYawRateFromImu();
  info->yaw_rate_info.yaw_rate_chassis = yaw_rate_filter_.GetYawRateFromChassis();
  info->yaw_rate_info.yaw_rate = yaw_rate_filter_.GetFilteredYawRate();
  info->yaw_rate_info.yaw_rate_chg_rate = yaw_rate_filter_.GetYawRateChgRate();
}

bool PosFilterImpl::FindRelPosFromListByTimestamp(
    Int64_t timestamp, const ad_msg::RelativePosList& pos_list,
    ad_msg::RelativePos* pos) {
  veh_model::VehicleModelWrapper vehicle_model;
  Int32_t pos_list_size = pos_list.relative_pos_num;

  if (pos_list_size < 1) {
    return false;
  }

  Int64_t relative_time = common::CalcElapsedClockMs(
        pos_list.msg_head.timestamp, timestamp);

  //std::cout << "#### pos_list.msg_head.timestamp=" << pos_list.msg_head.timestamp
  //          << ", timestamp=" << timestamp
  //          << ", relative_time=" << relative_time
  //          << std::endl;

  bool find_flag = false;
  for (Int32_t i = 0; i < pos_list_size; ++i) {
    const ad_msg::RelativePos& p = pos_list.relative_pos[i];
    if (p.relative_time <= relative_time) {
      Int32_t time_diff = relative_time - p.relative_time;
      if (time_diff > 500) {
        LOG_ERR << "Time interval is too large.";
        return false;
      }

      if (0 == i) {
        Scalar prev_pos[3] = { 0.0F };
        Scalar next_pos[3] = { 0.0F };
        prev_pos[0] = p.x;
        prev_pos[1] = p.y;
        prev_pos[2] = p.heading;
        vehicle_model.EstimateNextPos(
              p.v, p.yaw_rate, 0.001F*time_diff, prev_pos, next_pos);

        pos->relative_time = time_diff + p.relative_time;
        pos->x = next_pos[0];
        pos->y = next_pos[1];
        pos->heading = next_pos[2];
        pos->yaw_rate = p.yaw_rate;
        pos->yaw_rate_chg_rate = p.yaw_rate_chg_rate;
        //printf("1: pos->yaw_rate_chg_rate=%0.3f\n",
        //       common::com_rad2deg(pos->yaw_rate_chg_rate));
        pos->v = p.v;

#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "Find pos case 1, t=" << t << std::endl;
#endif
      } else {
        const ad_msg::RelativePos& next_p = pos_list.relative_pos[i-1];
        Scalar t = static_cast<Scalar>(time_diff) /
            static_cast<Scalar>(next_p.relative_time - p.relative_time);

        pos->relative_time = time_diff + p.relative_time;
        pos->x = (1.0F - t) * p.x + t * next_p.x;
        pos->y = (1.0F - t) * p.y + t * next_p.y;
        pos->heading = common::AngleLerp(p.heading, next_p.heading, t);
        pos->yaw_rate = (1.0F - t) * p.yaw_rate + t * next_p.yaw_rate;
        pos->yaw_rate_chg_rate =
            (1.0F - t) * p.yaw_rate_chg_rate + t * next_p.yaw_rate_chg_rate;
        //printf("2: pos->yaw_rate_chg_rate=%0.3f\n",
        //       common::com_rad2deg(pos->yaw_rate_chg_rate));
        pos->v = (1.0F - t) * p.v + t * next_p.v;
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "Find pos case 2" << std::endl;
#endif
      }
      find_flag = true;
      break;
    }
  }

  if (!find_flag) {
    const ad_msg::RelativePos& p = pos_list.relative_pos[pos_list_size-1];
    Int32_t time_diff = p.relative_time - relative_time;
    if (time_diff > 500) {
      LOG_ERR << "Time interval is too large.";
      return false;
    }
    Scalar prev_pos[3] = { 0.0F };
    Scalar next_pos[3] = { 0.0F };
    prev_pos[0] = p.x;
    prev_pos[1] = p.y;
    prev_pos[2] = p.heading;
    vehicle_model.EstimateNextPos(
          p.v, p.yaw_rate, -0.001F*time_diff, prev_pos, next_pos);

    pos->relative_time = p.relative_time - time_diff;
    pos->x = next_pos[0];
    pos->y = next_pos[1];
    pos->heading = next_pos[2];
    pos->yaw_rate = p.yaw_rate;
    pos->yaw_rate_chg_rate = p.yaw_rate_chg_rate;
    //printf("3: pos->yaw_rate_chg_rate=%0.3f\n",
    //       common::com_rad2deg(pos->yaw_rate_chg_rate));
    pos->v = p.v;

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Find pos case 3" << std::endl;
#endif
  }

  return true;
}

bool PosFilterImpl::EstimatePosByLaneMarkCameraList(
    Scalar velocity, Scalar yaw_rate,
    const ad_msg::LaneMarkCameraList& lane_mark_camera_list,
    FilterRet* filter_ret) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### EstimatePosByLaneMarkCameraList (Begin) ###" << std::endl;
#endif

  filter_ret->Clear();

  if (lane_mark_camera_list.lane_mark_num < 1) {
    return false;
  }

  // 判断相机识别的车道线是否有效
  bool lane_mark_valid = false;
  Uint32_t lane_mark_sequence_diff = 0;
  Int64_t lane_mark_time_elapsed = 0;
  if (lane_mark_camera_list.msg_head.valid) {
    if (prev_lane_mark_valid_) {
      // 保存了之前的车道线信息
      lane_mark_sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
            prev_lane_mark_msg_sequence_,
            lane_mark_camera_list.msg_head.sequence);
      if (0 != lane_mark_sequence_diff) {
        // 新的数据
        lane_mark_time_elapsed = common::CalcElapsedClockMs(
              prev_lane_mark_timestamp_,
              lane_mark_camera_list.msg_head.timestamp);
        if (9 < lane_mark_time_elapsed && lane_mark_time_elapsed < 500) {
          lane_mark_valid = true;
        }
      }
    }
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "sequence_diff=" << lane_mark_sequence_diff
            << ", time_elapsed=" << lane_mark_time_elapsed
            << std::endl;
#endif

  // 根据前后两帧之间的车道线信息，修正相对位置
  if (lane_mark_valid) {
    bool register_flag = false;
    Scalar time_elapsed_s = 0.001F*lane_mark_time_elapsed;
    Scalar delta_pos[3] = { 0.0F };
    common::Matrix<Scalar, 3, 3> mat_convert;
    common::Matrix<Scalar, 2, 1> rotate_center;
    common::Matrix<Scalar, 2, 1> point_conv;
    if (!lane_mark_camera_filter_.IsInitialized()) {
      // 车道线滤波器中没有保存有效的车道线信息
      lane_mark_valid = false;
    } else {
      // 车道线滤波器中保存了有效的车道线信息
      bool find_pos_success = false;
      if (prev_filtered_pos_valid_) {
        // 保存了有效的过滤后的相对位置列表信息
        filter_ret->time_end = common::CalcElapsedClockMs(
              prev_filtered_pos_timestamp_,
              lane_mark_camera_list.msg_head.timestamp);
        filter_ret->time_start = filter_ret->time_end - lane_mark_time_elapsed;
        if (FindPosFromListByTimestamp(filter_ret->time_end,
                                       &(filter_ret->estimated_pos_end))) {
          // 找到了正确的相对位置1
          if (FindPosFromListByTimestamp(filter_ret->time_start,
                                         &(filter_ret->estimated_pos_start))) {
            // 找到了正确的相对位置2
            find_pos_success = true;
          } else {
            LOG_ERR << "Failed to find relative pos of previous camera lane.";
          }
        } else {
          LOG_ERR << "Failed to find relative pos of current camera lane.";
        }
      }
      if (find_pos_success) {
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "Find relative pos for lane mark." << std::endl;
        std::cout << "[" << 1 << "] t="
                  << filter_ret->estimated_pos_start.relative_time
                  << ", x=" << filter_ret->estimated_pos_start.x
                  << ", y=" << filter_ret->estimated_pos_start.y
                  << ", h="
                  << common::com_rad2deg(
                       filter_ret->estimated_pos_start.heading)
                  << std::endl;
        std::cout << "[" << 2 << "] t="
                  << filter_ret->estimated_pos_end.relative_time
                  << ", x=" << filter_ret->estimated_pos_end.x
                  << ", y=" << filter_ret->estimated_pos_end.y
                  << ", h="
                  << common::com_rad2deg(filter_ret->estimated_pos_end.heading)
                  << std::endl;
#endif

        // 从过滤后的相对位置列表信息中找到了正确的相对位置
        mat_convert.SetIdentity();
        rotate_center.SetZeros();
        common::Translate_2D(-filter_ret->estimated_pos_start.x,
                             -filter_ret->estimated_pos_start.y,
                             &mat_convert);
        common::Rotate_2D<Scalar>(rotate_center,
                                  -filter_ret->estimated_pos_start.heading,
                                  &mat_convert);
        point_conv(0) = filter_ret->estimated_pos_end.x;
        point_conv(1) = filter_ret->estimated_pos_end.y;
        common::TransformVert_2D(mat_convert, &point_conv);
        delta_pos[0] = point_conv(0);
        delta_pos[1] = point_conv(1);
        delta_pos[2] = common::AngleDiff(
              filter_ret->estimated_pos_start.heading,
              filter_ret->estimated_pos_end.heading);
      } else {
        // 没有从相对位置列表中找到相对位置，从车辆当前状态估计相对位置
        Scalar pos_1[3] = { 0.0F };
        vehicle_model_.EstimateNextPos(
              velocity, yaw_rate, time_elapsed_s,
              pos_1, delta_pos);
      }
      // 前后两帧的车道线中，估计相对位置
      register_flag = lane_mark_camera_filter_.Update(
            lane_mark_camera_list, delta_pos,
            filter_ret->corrected_delta_pos);

#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Correct relative delta pos from ("
                << delta_pos[0] << "," << delta_pos[1]
                << "," << common::com_rad2deg(delta_pos[2])
                << ") to (" << filter_ret->corrected_delta_pos[0]
                << "," << filter_ret->corrected_delta_pos[1]
                << ","
                << common::com_rad2deg(filter_ret->corrected_delta_pos[2])
                << ") by lane marks." << std::endl;
#endif

      if (!register_flag || !find_pos_success) {
        // 没有正确地估计了前后两帧的车道线的相对位置，或者相对位置列表信息是无效的
        lane_mark_valid = false;
      }
    }
//    // 更新车道线列表
//    bool update_flag = lane_mark_camera_filter_.UpdateLaneLineList(
//          lane_mark_camera_list, filter_ret->corrected_delta_pos);

//    if (update_flag) {
//      // 成功地更新了车道线列表
//      prev_lane_mark_valid_ = true;
//      prev_lane_mark_msg_sequence_ =
//          lane_mark_camera_list.msg_head.sequence;
//      prev_lane_mark_timestamp_ =
//          lane_mark_camera_list.msg_head.timestamp;
//    }

    if (lane_mark_valid) {
      mat_convert.SetIdentity();
      rotate_center.SetZeros();
      common::Rotate_2D<Scalar>(rotate_center,
                                filter_ret->estimated_pos_start.heading,
                                &mat_convert);
      point_conv(0) = filter_ret->corrected_delta_pos[0];
      point_conv(1) = filter_ret->corrected_delta_pos[1];
      common::TransformVert_2D(mat_convert, &point_conv);
      filter_ret->corrected_delta_pos[0] = point_conv(0);
      filter_ret->corrected_delta_pos[1] = point_conv(1);

      filter_ret->corrected_velocity =
          common::com_sqrt(common::Square(filter_ret->corrected_delta_pos[0]) +
          common::Square(filter_ret->corrected_delta_pos[1])) / time_elapsed_s;
      if (velocity < -0.001F) {
        // For backing mode of vehicle
        filter_ret->corrected_velocity = -filter_ret->corrected_velocity;
      }

      filter_ret->corrected_yaw_rate =
          filter_ret->corrected_delta_pos[2] / time_elapsed_s;

      bool valid_bel = CalcBeliefBasedOnVelocity(
            filter_ret->estimated_pos_end.v,
            filter_ret->estimated_pos_end.yaw_rate,
            filter_ret->corrected_velocity,
            filter_ret->corrected_yaw_rate,
            &(filter_ret->belief));

      if (!valid_bel) {
        lane_mark_valid = false;
      }

#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Correct local delta pos from ("
                << filter_ret->estimated_pos_end.x -
                   filter_ret->estimated_pos_start.x
                << "," << filter_ret->estimated_pos_end.y -
                   filter_ret->estimated_pos_start.y
                << "," << common::com_rad2deg(
                     common::AngleDiff(filter_ret->estimated_pos_start.heading,
                                       filter_ret->estimated_pos_end.heading))
                << ") to (" << filter_ret->corrected_delta_pos[0]
                << "," << filter_ret->corrected_delta_pos[1]
                << ","
                << common::com_rad2deg(filter_ret->corrected_delta_pos[2])
                << ") by lane marks." << std::endl;

      std::cout << "Correct (v=" << filter_ret->estimated_pos_end.v*3.6F
                << ", yaw_rate=" << filter_ret->estimated_pos_end.yaw_rate
                << ") to (v=" << filter_ret->corrected_velocity * 3.6F
                << ", yaw_rate=" << filter_ret->corrected_yaw_rate
                << "), belief=" << filter_ret->belief
                << std::endl;
#endif
    }
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### EstimatePosByLaneMarkCameraList (End) ###" << std::endl;
#endif

  return (lane_mark_valid);
}

bool PosFilterImpl::EstimatePosByGnss(
    Scalar velocity, Scalar yaw_rate,
    Int32_t localization_type, const ad_msg::Gnss& gnss_info,
    FilterRet* filter_ret) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### EstimatePosByGnss (Begin) ###" << std::endl;
#endif

  filter_ret->Clear();

  // 判断 GNSS 是否有效
  bool gnss_valid_bel = false;
  Uint32_t sequence_diff = 0;
  Int64_t time_elapsed = 0;
  if (gnss_info.msg_head.valid && prev_gnss_valid_) {
    // 保存了之前的 GNSS 信息
    sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
          prev_gnss_info_.msg_head.sequence,
          gnss_info.msg_head.sequence);
    if (0 != sequence_diff) {
      // 新的数据
      time_elapsed = common::CalcElapsedClockMs(
            prev_gnss_info_.msg_head.timestamp, gnss_info.msg_head.timestamp);

      if (10 < time_elapsed && time_elapsed < 500) {
        if (LOCALIZATION_TYPE_GNSS == localization_type) {
          if (((ad_msg::Gnss::STATUS_GOOD == gnss_info.gnss_status) ||
               (ad_msg::Gnss::STATUS_CONVERGING == gnss_info.gnss_status)) &&
              ((ad_msg::Gnss::STATUS_GOOD == prev_gnss_info_.gnss_status) ||
               (ad_msg::Gnss::STATUS_CONVERGING == prev_gnss_info_.gnss_status))) {
            gnss_valid_bel = true;
          }
        } else if (LOCALIZATION_TYPE_UTM == localization_type) {
          if (((ad_msg::Gnss::STATUS_GOOD == gnss_info.utm_status) ||
               (ad_msg::Gnss::STATUS_CONVERGING == gnss_info.utm_status)) &&
              ((ad_msg::Gnss::STATUS_GOOD == prev_gnss_info_.utm_status) ||
               (ad_msg::Gnss::STATUS_CONVERGING == prev_gnss_info_.utm_status))) {
            gnss_valid_bel = true;
          }
        } else if (LOCALIZATION_TYPE_ODOM == localization_type) {
          if (((ad_msg::Gnss::STATUS_GOOD == gnss_info.odom_status) ||
               (ad_msg::Gnss::STATUS_CONVERGING == gnss_info.odom_status)) &&
              ((ad_msg::Gnss::STATUS_GOOD == prev_gnss_info_.odom_status) ||
               (ad_msg::Gnss::STATUS_CONVERGING == prev_gnss_info_.odom_status))) {
            gnss_valid_bel = true;
          }
        } else {
          LOG_ERR << "Invalid localization type.";
          gnss_valid_bel = false;
        }
      }
    }
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "sequence_diff=" << sequence_diff
            << ", time_elapsed=" << time_elapsed
            << std::endl;
#endif

  if (gnss_valid_bel) {
    Scalar time_elapsed_s = 0.001F*time_elapsed;
    Scalar delta_pos[3] = { 0.0F };
    bool find_pos_success = false;
    if (prev_filtered_pos_valid_) {
      // 保存了有效的过滤后的相对位置列表信息
      filter_ret->time_end = common::CalcElapsedClockMs(
            prev_filtered_pos_timestamp_, gnss_info.msg_head.timestamp);
      filter_ret->time_start = filter_ret->time_end - time_elapsed;
      if (FindPosFromListByTimestamp(filter_ret->time_end,
                                     &(filter_ret->estimated_pos_end))) {
        // 找到了正确的相对位置1
        if (FindPosFromListByTimestamp(filter_ret->time_start,
                                       &(filter_ret->estimated_pos_start))) {
          // 找到了正确的相对位置2
          find_pos_success = true;
        } else {
          LOG_ERR << "Failed to find relative pos of previous gnss.";
        }
      } else {
        LOG_ERR << "Failed to find relative pos of current gnss.";
      }
    }
    if (find_pos_success) {
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Find relative pos for lane mark." << std::endl;
      std::cout << "[" << 1 << "] t="
                << filter_ret->estimated_pos_start.relative_time
                << ", x=" << filter_ret->estimated_pos_start.x
                << ", y=" << filter_ret->estimated_pos_start.y
                << ", h="
                << common::com_rad2deg(
                     filter_ret->estimated_pos_start.heading)
                << std::endl;
      std::cout << "[" << 2 << "] t="
                << filter_ret->estimated_pos_end.relative_time
                << ", x=" << filter_ret->estimated_pos_end.x
                << ", y=" << filter_ret->estimated_pos_end.y
                << ", h="
                << common::com_rad2deg(filter_ret->estimated_pos_end.heading)
                << std::endl;
#endif
      // 从过滤后的相对位置列表信息中找到了正确的相对位置
      delta_pos[0] = filter_ret->estimated_pos_end.x -
          filter_ret->estimated_pos_start.x;
      delta_pos[1] = filter_ret->estimated_pos_end.y -
          filter_ret->estimated_pos_start.y;
      delta_pos[2] = common::AngleDiff(
            filter_ret->estimated_pos_start.heading,
            filter_ret->estimated_pos_end.heading);

      common::Matrix<Scalar, 3, 3> mat_convert;
      common::Matrix<Scalar, 2, 1> rotate_center;
      common::Matrix<Scalar, 2, 1> point_conv;
      mat_convert.SetIdentity();
      rotate_center.SetZeros();

      if (LOCALIZATION_TYPE_GNSS == localization_type) {
        common::GpsPoint gps_origin;
        gps_origin.latitude = prev_gnss_info_.latitude;
        gps_origin.longitude = prev_gnss_info_.longitude;
        common::GpsPoint gps_point;
        gps_point.latitude = gnss_info.latitude;
        gps_point.longitude = gnss_info.longitude;
        common::Vec2d local_point;
        common::ConvGpsPointToLocalPoint(gps_origin, gps_point, &local_point);

        common::Rotate_2D<Scalar>(rotate_center,
                                  common::NormalizeAngle(
                                    -prev_gnss_info_.heading_gnss +
                                    filter_ret->estimated_pos_start.heading),
                                  &mat_convert);
        point_conv(0) = local_point.x();
        point_conv(1) = local_point.y();
        common::TransformVert_2D(mat_convert, &point_conv);

        filter_ret->corrected_delta_pos[2] = common::AngleDiff(
              prev_gnss_info_.heading_gnss, gnss_info.heading_gnss);
      } else if (LOCALIZATION_TYPE_UTM == localization_type) {
        common::Rotate_2D<Scalar>(rotate_center,
                                  common::NormalizeAngle(
                                    -prev_gnss_info_.heading_utm +
                                    filter_ret->estimated_pos_start.heading),
                                  &mat_convert);
        point_conv(0) = gnss_info.x_utm - prev_gnss_info_.x_utm;
        point_conv(1) = gnss_info.y_utm - prev_gnss_info_.y_utm;
        common::TransformVert_2D(mat_convert, &point_conv);

        filter_ret->corrected_delta_pos[2] = common::AngleDiff(
              prev_gnss_info_.heading_utm, gnss_info.heading_utm);
      } else if (LOCALIZATION_TYPE_ODOM == localization_type) {
        common::Rotate_2D<Scalar>(rotate_center,
                                  common::NormalizeAngle(
                                    -prev_gnss_info_.heading_odom +
                                    filter_ret->estimated_pos_start.heading),
                                  &mat_convert);
        point_conv(0) = gnss_info.x_odom - prev_gnss_info_.x_odom;
        point_conv(1) = gnss_info.y_odom - prev_gnss_info_.y_odom;
        common::TransformVert_2D(mat_convert, &point_conv);

        filter_ret->corrected_delta_pos[2] = common::AngleDiff(
              prev_gnss_info_.heading_odom, gnss_info.heading_odom);
      } else {
        LOG_ERR << "Invalid localization type.";
        return false;
      }

      filter_ret->corrected_delta_pos[0] = point_conv(0);
      filter_ret->corrected_delta_pos[1] = point_conv(1);

      filter_ret->corrected_velocity =
          common::com_sqrt(common::Square(filter_ret->corrected_delta_pos[0]) +
          common::Square(filter_ret->corrected_delta_pos[1])) / time_elapsed_s;
      if (velocity < -0.001F) {
        // For backing mode of vehicle
        filter_ret->corrected_velocity = -filter_ret->corrected_velocity;
      }

      filter_ret->corrected_yaw_rate =
          filter_ret->corrected_delta_pos[2] / time_elapsed_s;

      bool valid_bel = CalcBeliefBasedOnVelocity(
            filter_ret->estimated_pos_end.v,
            filter_ret->estimated_pos_end.yaw_rate,
            filter_ret->corrected_velocity,
            filter_ret->corrected_yaw_rate,
            &(filter_ret->belief));

      if (!valid_bel) {
        gnss_valid_bel = false;
      }

#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Correct local delta pos from ("
                << delta_pos[0] << "," << delta_pos[1]
                << "," << common::com_rad2deg(delta_pos[2])
          << ") to (" << filter_ret->corrected_delta_pos[0]
          << "," << filter_ret->corrected_delta_pos[1]
          << ","
          << common::com_rad2deg(filter_ret->corrected_delta_pos[2])
          << ") by gnss." << std::endl;

      std::cout << "Correct (v=" << filter_ret->estimated_pos_end.v*3.6F
                << ", yaw_rate=" << filter_ret->estimated_pos_end.yaw_rate
                << ") to (v=" << filter_ret->corrected_velocity * 3.6F
                << ", yaw_rate=" << filter_ret->corrected_yaw_rate
                << "), belief=" << filter_ret->belief
                << std::endl;
#endif
    } else {
      gnss_valid_bel = false;
    }
  }

  prev_gnss_valid_ = true;
  prev_gnss_info_ = gnss_info;

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### EstimatePosByGnss (End) ###" << std::endl;
#endif

  return (gnss_valid_bel);
}

bool PosFilterImpl::CalcBeliefBasedOnVelocity(Scalar v, Scalar yaw_rate,
    Scalar estimated_v, Scalar estimated_yaw_rate, Scalar* bel) {
  bool valid_bel = true;
  Scalar belief = 1.0F;

  Scalar abs_v_diff = common::com_abs(
        estimated_v - v);
  Scalar abs_yaw_rate_diff = common::com_abs(
        estimated_yaw_rate -
        yaw_rate);
  Scalar belief_v = 1.0F;
  Scalar belief_yaw_rate = 1.0F;

  Scalar abs_v = common::com_abs(v);
  Scalar abs_yaw_rate =
      common::com_abs(yaw_rate);
  Scalar variance_v = 0.05F * abs_v + 0.1f * abs_yaw_rate;
  Scalar variance_yaw_rate = 0.001F * abs_v + 0.05f * abs_yaw_rate;

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "abs_v_diff=" << abs_v_diff * 3.6F
            << " km/h, abs_yaw_rate_diff="
            << common::com_rad2deg(abs_yaw_rate_diff) << " deg/s"
            << std::endl;
  std::cout << "variance_v=" << variance_v * 3.6F
            << " km/h, variance_yaw_rate="
            << common::com_rad2deg(variance_yaw_rate) << " deg/s"
            << std::endl;
#endif

  Scalar ratio = 0.3F;
  if (abs_v_diff < ratio*variance_v) {
    belief_v = 1.0F;
  } else if (abs_v_diff < variance_v) {
    belief_v = (variance_v - abs_v_diff) / ((1.0F-ratio)*variance_v);
  } else {
    belief_v = 0.0F;
  }

  if (abs_yaw_rate_diff < ratio*variance_yaw_rate) {
    belief_yaw_rate = 1.0F;
  } else if (abs_yaw_rate_diff < variance_yaw_rate) {
    belief_yaw_rate = (variance_yaw_rate - abs_yaw_rate_diff) /
        ((1.0F-ratio)*variance_yaw_rate);
  } else {
    belief_yaw_rate = 0.0F;
  }

  belief = belief_v * belief_yaw_rate;
  if ((abs_v_diff > 1.1F*variance_v) ||
      (abs_yaw_rate_diff > 1.1F*variance_yaw_rate)) {
    valid_bel = false;
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "belief_v=" << belief_v
            << ", belief_yaw_rate=" << belief_yaw_rate
            << ", belief=" << belief
            << std::endl;
#endif

  *bel = belief;

  return (valid_bel);
}

void PosFilterImpl::PredictPosByVehStatus(FilterRet* filter_ret) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### PredictPosByVehStatus (Start) ###" << std::endl;
#endif
  Int32_t time_start = 0;
  Int32_t time_end = 0;
  Scalar pos_last[3] = { 0.0F };
  Scalar pos_predicted[3] = { 0.0F };

  // 状态方程用的前一时刻的位置
  const FilteredPos& prev_pos = filtered_pos_list_.Front();

  time_start = prev_pos.relative_time;
  filter_ret->time_start = prev_pos.relative_time;
  filter_ret->estimated_pos_start = prev_pos;

  // 计算状态方程用的时间间隔
  Scalar delta_t = 0.001F * static_cast<Scalar>(pos_filter_timeinterval_);

  pos_last[0] = prev_pos.x;
  pos_last[1] = prev_pos.y;
  pos_last[2] = prev_pos.heading;

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "lane mark is invalid, estimate pose by state of vehicle,"
            << "time_diff=" << time_diff
            << ", v=" << first_pos.v*3.6F
            << ", yaw_rate=" << first_pos.yaw_rate
            << std::endl;
#endif

  vehicle_model_.EstimateNextPos(
        prev_pos.v, prev_pos.yaw_rate,
        delta_t, pos_last, pos_predicted);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "pos_predicted(" << pos_predicted[0]
            << "," << pos_predicted[1]
            << "," << common::com_rad2deg(pos_predicted[2])
            << ")." << std::endl;
#endif

  time_end = pos_filter_timeinterval_;
  filter_ret->time_end = pos_filter_timeinterval_;
  filter_ret->estimated_pos_end.relative_time = pos_filter_timeinterval_;
  filter_ret->estimated_pos_end.x = pos_predicted[0];
  filter_ret->estimated_pos_end.y = pos_predicted[1];
  filter_ret->estimated_pos_end.heading = pos_predicted[2];
  filter_ret->estimated_pos_end.v = curr_velocity_;
  filter_ret->estimated_pos_end.yaw_rate = curr_yaw_rate_;

  Scalar max_cov_coeff = mat_covariance_.FindMaxAbsCoeff();
  if (max_cov_coeff < 40.0F) {
    // 更新状态方程的系数矩阵
    common::Matrix<Scalar, 3, 3> mat_G;
    mat_G.SetIdentity();
    Scalar sin_prev_heading = common::com_sin(prev_pos.heading);
    Scalar cos_prev_heading = common::com_cos(prev_pos.heading);
    if (common::com_abs(prev_pos.yaw_rate) < 0.001F) {
      mat_G(0, 2) = -prev_pos.v * sin_prev_heading * delta_t;
      mat_G(1, 2) = prev_pos.v * cos_prev_heading * delta_t;
    } else {
      Scalar r = prev_pos.v / prev_pos.yaw_rate;
      Scalar heading_changed = phoenix::common::NormalizeAngle(
            prev_pos.heading + prev_pos.yaw_rate*delta_t);
      Scalar sin_heading_changed = common::com_sin(heading_changed);
      Scalar cos_heading_changed = common::com_cos(heading_changed);

      mat_G(0, 2) = -r*cos_prev_heading + r*cos_heading_changed;
      mat_G(1, 2) = -r*sin_prev_heading + r*sin_heading_changed;
    }
    common::Matrix<Scalar, 3, 3> mat_G_tran;
    common::Mat_Transpose(mat_G, mat_G_tran);

    // Covariance of the state transition noise
    StateNoise state_trans_noise;
    GetStateTransitionNoise(prev_pos.v, prev_pos.yaw_rate, &state_trans_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "predict noise = (" << state_trans_noise.x_noise
              << ", " << state_trans_noise.y_noise
              << ", " << common::com_rad2deg(state_trans_noise.heading_noise)
              << ")" << std::endl;
#endif

    // Covariance of state belief after prediction
    // P = G * P_ * G.transpose() + R;
    common::Matrix<Scalar, 3, 3> mat_P;
    common::Matrix<Scalar, 3, 3> mat_tmp_3v3;
    common::Mat_Mul(mat_G, mat_covariance_, mat_tmp_3v3);
    common::Mat_Mul(mat_tmp_3v3, mat_G_tran, mat_P);
    mat_P(0, 0) += state_trans_noise.x_noise;
    mat_P(1, 1) += state_trans_noise.y_noise;
    mat_P(2, 2) += state_trans_noise.heading_noise;

    filter_ret->corrected_delta_pos[0] =
        filter_ret->estimated_pos_end.x - filter_ret->estimated_pos_start.x;
    filter_ret->corrected_delta_pos[1] =
        filter_ret->estimated_pos_end.y - filter_ret->estimated_pos_start.y;
    filter_ret->corrected_delta_pos[2] = common::AngleDiff(
          filter_ret->estimated_pos_start.heading,
          filter_ret->estimated_pos_end.heading);

    Scalar time_elapsed_s = delta_t;
    if (time_elapsed_s < 0.005F) {
      time_elapsed_s = 0.005F;
    }
    filter_ret->corrected_velocity =
        common::com_sqrt(common::Square(filter_ret->corrected_delta_pos[0]) +
        common::Square(filter_ret->corrected_delta_pos[1])) / time_elapsed_s;
    if (curr_velocity_ < -0.001F) {
      // For backing mode of vehicle
      filter_ret->corrected_velocity = -filter_ret->corrected_velocity;
    }

    filter_ret->corrected_yaw_rate =
        filter_ret->corrected_delta_pos[2] / time_elapsed_s;

    // 修正协方差矩阵
    mat_covariance_ = mat_P;
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "After kalman filter:" << std::endl;
  std::cout << "pos_filtered = (" << filter_ret->estimated_pos_end.x
            << "," << filter_ret->estimated_pos_end.y
            << "," << common::com_rad2deg(filter_ret->estimated_pos_end.heading)
            << ")."
            << std::endl;
  std::cout << "mat_covariance_=\n" << mat_covariance_ << std::endl;

  std::cout << "Correct (v=" << filter_ret->estimated_pos_end.v*3.6F
            << " km/h, yaw_rate="
            << common::com_rad2deg(filter_ret->estimated_pos_end.yaw_rate)
            << " deg/s) to (v=" << filter_ret->corrected_velocity * 3.6F
            << " km/s, yaw_rate="
            << common::com_rad2deg(filter_ret->corrected_yaw_rate)
            << " deg/s)." << std::endl;
#endif

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### PredictPosByVehStatus (End) ###" << std::endl;
#endif
}

void PosFilterImpl::FilterPosByLaneMark(
    const FilterRet& lane_mark_filter_ret,
    FilterRet* filter_ret) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterPosByLaneMark (Start) ###" << std::endl;
#endif
  Int64_t time_start = 0;
  Int64_t time_end = 0;
  Scalar pos_predicted[3] = { 0.0F };
  Scalar pos_end_lane_mark[3] = { 0.0F };

  time_start = lane_mark_filter_ret.time_start;
  filter_ret->time_start = lane_mark_filter_ret.time_start;
  filter_ret->estimated_pos_start = lane_mark_filter_ret.estimated_pos_start;

  time_end = lane_mark_filter_ret.time_end;
  filter_ret->time_end = lane_mark_filter_ret.time_end;
  filter_ret->estimated_pos_end = lane_mark_filter_ret.estimated_pos_end;

  // 通过车身状态估计的位置（运动状态更新）
  pos_predicted[0] = lane_mark_filter_ret.estimated_pos_end.x;
  pos_predicted[1] = lane_mark_filter_ret.estimated_pos_end.y;
  pos_predicted[2] = lane_mark_filter_ret.estimated_pos_end.heading;

  // 通过相机识别的车道线估计的位置(观测量)
  pos_end_lane_mark[0] = lane_mark_filter_ret.estimated_pos_start.x +
      lane_mark_filter_ret.corrected_delta_pos[0];
  pos_end_lane_mark[1] = lane_mark_filter_ret.estimated_pos_start.y +
      lane_mark_filter_ret.corrected_delta_pos[1];
  pos_end_lane_mark[2] = common::NormalizeAngle(
        lane_mark_filter_ret.estimated_pos_start.heading +
        lane_mark_filter_ret.corrected_delta_pos[2]);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "pos_end_lane_mark("
            << pos_end_lane_mark[0]
            << "," << pos_end_lane_mark[1]
            << "," << common::com_rad2deg(pos_end_lane_mark[2])
            << "), pos_predicted(" << pos_predicted[0]
            << "," << pos_predicted[1]
            << "," << common::com_rad2deg(pos_predicted[2])
            << ")." << std::endl;

  std::cout << "time_start=" << time_start
            << ", time_end=" << time_end
            << std::endl;
#endif

  // 计算状态方程用的时间间隔
  Scalar delta_t = 0.001F * static_cast<Scalar>(
        common::Max(time_end, pos_filter_timeinterval_ / 2));
  // 状态方程用的前一时刻的位置
  const FilteredPos& prev_pos = filtered_pos_list_.Front();
  // 更新状态方程的系数矩阵
  common::Matrix<Scalar, 3, 3> mat_G;
  mat_G.SetIdentity();
  Scalar sin_prev_heading = common::com_sin(prev_pos.heading);
  Scalar cos_prev_heading = common::com_cos(prev_pos.heading);
  if (common::com_abs(prev_pos.yaw_rate) < 0.001F) {
    mat_G(0, 2) = -prev_pos.v * sin_prev_heading * delta_t;
    mat_G(1, 2) = prev_pos.v * cos_prev_heading * delta_t;
  } else {
    Scalar r = prev_pos.v / prev_pos.yaw_rate;
    Scalar heading_changed = phoenix::common::NormalizeAngle(
          prev_pos.heading + prev_pos.yaw_rate*delta_t);
    Scalar sin_heading_changed = common::com_sin(heading_changed);
    Scalar cos_heading_changed = common::com_cos(heading_changed);

    mat_G(0, 2) = -r*cos_prev_heading + r*cos_heading_changed;
    mat_G(1, 2) = -r*sin_prev_heading + r*sin_heading_changed;
  }
  common::Matrix<Scalar, 3, 3> mat_G_tran;
  common::Mat_Transpose(mat_G, mat_G_tran);

  // Covariance of the state transition noise
  StateNoise state_trans_noise;
  GetStateTransitionNoise(prev_pos.v, prev_pos.yaw_rate, &state_trans_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "predict noise = (" << state_trans_noise.x_noise
            << ", " << state_trans_noise.y_noise
            << ", " << common::com_rad2deg(state_trans_noise.heading_noise)
            << ")" << std::endl;
#endif

  // Covariance of state belief after prediction
  // P = G * P_ * G.transpose() + R;
  common::Matrix<Scalar, 3, 3> mat_P;
  common::Matrix<Scalar, 3, 3> mat_tmp_3v3;
  common::Mat_Mul(mat_G, mat_covariance_, mat_tmp_3v3);
  common::Mat_Mul(mat_tmp_3v3, mat_G_tran, mat_P);
  mat_P(0, 0) += state_trans_noise.x_noise;
  mat_P(1, 1) += state_trans_noise.y_noise;
  mat_P(2, 2) += state_trans_noise.heading_noise;

  // std::cout << "mat_P=\n" << mat_P << std::endl;

  // Covariance of observation noise
  StateNoise lane_mark_noise;
  GetMeasurementNoiseOfLaneMark(lane_mark_filter_ret.belief,
        lane_mark_filter_ret.estimated_pos_start.heading, &lane_mark_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "lane mark noise = (" << lane_mark_noise.x_noise
            << ", " << lane_mark_noise.y_noise
            << ", " << common::com_rad2deg(lane_mark_noise.heading_noise)
            << ")" << std::endl;
#endif

  // 计算kalman增益矩阵 [ K = Cov_pred * H_t * (H * Cov_pred * H_t + Q)^-1 ]
  /*
   *  H * Cov_pred * H_t
   *
   *  |        |   |   |   |        |
   *  | I(3v3) | * | P | * | I(3v3) |
   *  |        |   |   |   |        |
   *
   */
  common::Matrix<Scalar, 3, 3> mat_tmp_3v3_2;
  mat_tmp_3v3 = mat_P;

  // std::cout << "mat_tmp_3v3=\n" << mat_tmp_3v3 << std::endl;

  // H * Cov_pred * H_t + Q
  mat_tmp_3v3(0, 0) += lane_mark_noise.x_noise;
  mat_tmp_3v3(1, 1) += lane_mark_noise.y_noise;
  mat_tmp_3v3(2, 2) += lane_mark_noise.heading_noise;
  // (H * Cov_pred * H_t + Q)^-1
  common::Mat_CalcPseudoInverse(mat_tmp_3v3, mat_tmp_3v3_2);

  // std::cout << "mat_tmp_3v3_2=\n" << mat_tmp_3v3_2 << std::endl;

  /*
   *  H_t * (H * Cov_pred * H_t + Q)^-1
   *
   *  |        |   |       |
   *  | I(3v3) | * | (3v3) |
   *  |        |   |       |
   */

  // K = Cov * H_t * (H * Cov * H_t + Q)^-1
  common::Matrix<Scalar, 3, 3> mat_K;
  common::Mat_Mul(mat_P, mat_tmp_3v3_2, mat_K);

  // std::cout << "mat_K=\n" << mat_K << std::endl;

  // 使用观测量修正状态量 [ bel = bel_pred + K(z - z_pred) ]
  filter_ret->estimated_pos_end.x = pos_predicted[0] +
      mat_K(0, 0) * (pos_end_lane_mark[0] - pos_predicted[0]) +
      mat_K(0, 1) * (pos_end_lane_mark[1] - pos_predicted[1]) +
      mat_K(0, 2) * (pos_end_lane_mark[2] - pos_predicted[2]);
  filter_ret->estimated_pos_end.y = pos_predicted[1] +
      mat_K(1, 0) * (pos_end_lane_mark[0] - pos_predicted[0]) +
      mat_K(1, 1) * (pos_end_lane_mark[1] - pos_predicted[1]) +
      mat_K(1, 2) * (pos_end_lane_mark[2] - pos_predicted[2]);
  filter_ret->estimated_pos_end.heading = pos_predicted[2] +
      mat_K(2, 0) * (pos_end_lane_mark[0] - pos_predicted[0]) +
      mat_K(2, 1) * (pos_end_lane_mark[1] - pos_predicted[1]) +
      mat_K(2, 2) * (pos_end_lane_mark[2] - pos_predicted[2]);

  filter_ret->corrected_delta_pos[0] =
      filter_ret->estimated_pos_end.x - filter_ret->estimated_pos_start.x;
  filter_ret->corrected_delta_pos[1] =
      filter_ret->estimated_pos_end.y - filter_ret->estimated_pos_start.y;
  filter_ret->corrected_delta_pos[2] = common::AngleDiff(
        filter_ret->estimated_pos_start.heading,
        filter_ret->estimated_pos_end.heading);

  Scalar time_elapsed_s = 0.001F * static_cast<Scalar>(
        filter_ret->time_end - filter_ret->time_start);
  if (time_elapsed_s < 0.005F) {
    time_elapsed_s = 0.005F;
  }
  filter_ret->corrected_velocity =
      common::com_sqrt(common::Square(filter_ret->corrected_delta_pos[0]) +
      common::Square(filter_ret->corrected_delta_pos[1])) / time_elapsed_s;
  if (curr_velocity_ < -0.001F) {
    // For backing mode of vehicle
    filter_ret->corrected_velocity = -filter_ret->corrected_velocity;
  }

  filter_ret->corrected_yaw_rate =
      filter_ret->corrected_delta_pos[2] / time_elapsed_s;

  // 使用观测量修正协方差矩阵 [ Cov = (I - K * H) * Cov_pred ]
  /*
   *  I - K * H
   *
   *  |        |   |         |   |        |
   *  | I(3v3) | - | K(3v3)  | * | I(3v3) |
   *  |        |   |         |   |        |
   */
  for (Int32_t j = 0; j < 3; ++j) {
    for (Int32_t i = 0; i < 3; ++i) {
      if (i == j) {
        mat_tmp_3v3(i, j) = 1.0F - mat_K(i, j);
      } else {
        mat_tmp_3v3(i, j) = -mat_K(i, j);
      }
    }
  }

  // std::cout << "mat_tmp_3v3=\n" << mat_tmp_3v3 << std::endl;

  // (I - K * H) * Cov_pred
  common::Mat_Mul(mat_tmp_3v3, mat_P, mat_covariance_);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "After kalman filter:" << std::endl;
  std::cout << "pos_filtered = (" << filter_ret->estimated_pos_end.x
            << "," << filter_ret->estimated_pos_end.y
            << "," << common::com_rad2deg(filter_ret->estimated_pos_end.heading)
            << ")."
            << std::endl;
  std::cout << "mat_covariance_=\n" << mat_covariance_ << std::endl;

  std::cout << "Correct (v=" << filter_ret->estimated_pos_end.v*3.6F
            << " km/h, yaw_rate="
            << common::com_rad2deg(filter_ret->estimated_pos_end.yaw_rate)
            << " deg/s) to (v=" << filter_ret->corrected_velocity * 3.6F
            << " km/s, yaw_rate="
            << common::com_rad2deg(filter_ret->corrected_yaw_rate)
            << " deg/s)." << std::endl;
#endif

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterPosByLaneMark (End) ###" << std::endl;
#endif
}

void PosFilterImpl::FilterPosByLaneMarkAndGnss(
    const FilterRet& lane_mark_filter_ret,
    const FilterRet& gnss_filter_ret,
    FilterRet* filter_ret) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterPosByLaneMarkAndGnss (Start) ###" << std::endl;
#endif
  Int64_t time_start = 0;
  Int64_t time_end = 0;
  Scalar pos_predicted[3] = { 0.0F };
  Scalar pos_end_lane_mark[3] = { 0.0F };
  Scalar pos_end_gnss[3] = { 0.0F };

  // 将两种不同的数据源的时间对齐
  if (lane_mark_filter_ret.time_start < gnss_filter_ret.time_start) {
    time_start = lane_mark_filter_ret.time_start;

    filter_ret->time_start = lane_mark_filter_ret.time_start;
    filter_ret->estimated_pos_start = lane_mark_filter_ret.estimated_pos_start;

  } else {
    time_start = gnss_filter_ret.time_start;

    filter_ret->time_start = gnss_filter_ret.time_start;
    filter_ret->estimated_pos_start = gnss_filter_ret.estimated_pos_start;
  }
  Int64_t time_diff = 0;
  Scalar tmp_pos[3] = { 0.0F };
  if (lane_mark_filter_ret.time_end > gnss_filter_ret.time_end) {
    // 相机识别的车道线比GNSS要新鲜一些
    time_end = lane_mark_filter_ret.time_end;

    filter_ret->time_end = lane_mark_filter_ret.time_end;
    filter_ret->estimated_pos_end = lane_mark_filter_ret.estimated_pos_end;

    // 通过车身状态估计的位置（运动状态更新）
    pos_predicted[0] = lane_mark_filter_ret.estimated_pos_end.x;
    pos_predicted[1] = lane_mark_filter_ret.estimated_pos_end.y;
    pos_predicted[2] = lane_mark_filter_ret.estimated_pos_end.heading;

    // 通过相机识别的车道线估计的位置(观测量)
    pos_end_lane_mark[0] = lane_mark_filter_ret.estimated_pos_start.x +
        lane_mark_filter_ret.corrected_delta_pos[0];
    pos_end_lane_mark[1] = lane_mark_filter_ret.estimated_pos_start.y +
        lane_mark_filter_ret.corrected_delta_pos[1];
    pos_end_lane_mark[2] = common::NormalizeAngle(
          lane_mark_filter_ret.estimated_pos_start.heading +
          lane_mark_filter_ret.corrected_delta_pos[2]);

    // 将GNSS的位置对齐到相机识别的车道线的位置(通过车身当前状态进行估计)
    time_diff = common::CalcElapsedClockMs(
          gnss_filter_ret.time_end, lane_mark_filter_ret.time_end);
    // 通过GNSS估计的位置 + 通过车身当前状态进行估计的位置（时间差对齐）
    // = 对齐后的GNSS估计的位置(观测量)
    tmp_pos[0] = gnss_filter_ret.estimated_pos_start.x +
        gnss_filter_ret.corrected_delta_pos[0];
    tmp_pos[1] = gnss_filter_ret.estimated_pos_start.y +
        gnss_filter_ret.corrected_delta_pos[1];
    tmp_pos[2] = common::NormalizeAngle(
          gnss_filter_ret.estimated_pos_start.heading +
          gnss_filter_ret.corrected_delta_pos[2]);
    vehicle_model_.EstimateNextPos(
          gnss_filter_ret.estimated_pos_end.v,
          gnss_filter_ret.estimated_pos_end.yaw_rate,
          0.001F*time_diff, tmp_pos, pos_end_gnss);

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "lane mark first, pos_end_lane_mark("
              << pos_end_lane_mark[0]
              << "," << pos_end_lane_mark[1]
              << "," << common::com_rad2deg(pos_end_lane_mark[2])
              << "), pos_end_gnss(" << pos_end_gnss[0]
              << "," << pos_end_gnss[1]
              << "," << common::com_rad2deg(pos_end_gnss[2])
              << "), pos_predicted(" << pos_predicted[0]
              << "," << pos_predicted[1]
              << "," << common::com_rad2deg(pos_predicted[2])
              << ")." << std::endl;
#endif
  } else {
    // GNSS比相机识别的车道线要新鲜一些
    time_end = gnss_filter_ret.time_end;

    filter_ret->time_end = gnss_filter_ret.time_end;
    filter_ret->estimated_pos_end = gnss_filter_ret.estimated_pos_end;

    // 通过车身状态估计的位置（运动状态更新）
    pos_predicted[0] = gnss_filter_ret.estimated_pos_end.x;
    pos_predicted[1] = gnss_filter_ret.estimated_pos_end.y;
    pos_predicted[2] = gnss_filter_ret.estimated_pos_end.heading;

    // 通过GNSS估计的位置(观测量)
    pos_end_gnss[0] = gnss_filter_ret.estimated_pos_start.x +
        gnss_filter_ret.corrected_delta_pos[0];
    pos_end_gnss[1] = gnss_filter_ret.estimated_pos_start.y +
        gnss_filter_ret.corrected_delta_pos[1];
    pos_end_gnss[2] = common::NormalizeAngle(
          gnss_filter_ret.estimated_pos_start.heading +
          gnss_filter_ret.corrected_delta_pos[2]);

    // 将相机识别的车道线的位置对齐到GNSS的位置(通过车身当前状态进行估计)
    time_diff = common::CalcElapsedClockMs(
          lane_mark_filter_ret.time_end, gnss_filter_ret.time_end);
    // 通过相机识别的车道线估计的位置 + 通过车身当前状态进行估计的位置（时间差对齐）
    // = 对齐后的相机识别的车道线估计的位置(观测量)
    tmp_pos[0] = lane_mark_filter_ret.estimated_pos_start.x +
        lane_mark_filter_ret.corrected_delta_pos[0];
    tmp_pos[1] = lane_mark_filter_ret.estimated_pos_start.y +
        lane_mark_filter_ret.corrected_delta_pos[1];
    tmp_pos[2] = common::NormalizeAngle(
          lane_mark_filter_ret.estimated_pos_start.heading +
          lane_mark_filter_ret.corrected_delta_pos[2]);
    Scalar t_for_lane_mark = 0.5F;
    Scalar yaw_rate_for_lane_mark =
        (1.0F - t_for_lane_mark) *
        lane_mark_filter_ret.estimated_pos_end.yaw_rate +
        t_for_lane_mark * lane_mark_filter_ret.corrected_yaw_rate;
    vehicle_model_.EstimateNextPos(
          lane_mark_filter_ret.estimated_pos_end.v,
          yaw_rate_for_lane_mark,
          0.001F*time_diff, tmp_pos, pos_end_lane_mark);

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "gnss first, pos_end_lane_mark("
              << pos_end_lane_mark[0]
              << "," << pos_end_lane_mark[1]
              << "," << common::com_rad2deg(pos_end_lane_mark[2])
              << "), pos_end_gnss(" << pos_end_gnss[0]
              << "," << pos_end_gnss[1]
              << "," << common::com_rad2deg(pos_end_gnss[2])
              << "), pos_state(" << pos_predicted[0]
              << "," << pos_predicted[1]
              << "," << common::com_rad2deg(pos_predicted[2])
              << ")." << std::endl;
#endif
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "time_start=" << time_start
            << ", time_end=" << time_end
            << std::endl;
#endif

  // 计算状态方程用的时间间隔
  Scalar delta_t = 0.001F * static_cast<Scalar>(
        common::Max(time_end, pos_filter_timeinterval_ / 2));
  // 状态方程用的前一时刻的位置
  const FilteredPos& prev_pos = filtered_pos_list_.Front();
  // 更新状态方程的系数矩阵
  common::Matrix<Scalar, 3, 3> mat_G;
  mat_G.SetIdentity();
  Scalar sin_prev_heading = common::com_sin(prev_pos.heading);
  Scalar cos_prev_heading = common::com_cos(prev_pos.heading);
  if (common::com_abs(prev_pos.yaw_rate) < 0.001F) {
    mat_G(0, 2) = -prev_pos.v * sin_prev_heading * delta_t;
    mat_G(1, 2) = prev_pos.v * cos_prev_heading * delta_t;
  } else {
    Scalar r = prev_pos.v / prev_pos.yaw_rate;
    Scalar heading_changed = phoenix::common::NormalizeAngle(
          prev_pos.heading + prev_pos.yaw_rate*delta_t);
    Scalar sin_heading_changed = common::com_sin(heading_changed);
    Scalar cos_heading_changed = common::com_cos(heading_changed);

    mat_G(0, 2) = -r*cos_prev_heading + r*cos_heading_changed;
    mat_G(1, 2) = -r*sin_prev_heading + r*sin_heading_changed;
  }
  common::Matrix<Scalar, 3, 3> mat_G_tran;
  common::Mat_Transpose(mat_G, mat_G_tran);

  // Covariance of the state transition noise
  StateNoise state_trans_noise;
  GetStateTransitionNoise(prev_pos.v, prev_pos.yaw_rate, &state_trans_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "predict noise = (" << state_trans_noise.x_noise
            << ", " << state_trans_noise.y_noise
            << ", " << common::com_rad2deg(state_trans_noise.heading_noise)
            << ")" << std::endl;
#endif

  // Covariance of state belief after prediction
  // P = G * P_ * G.transpose() + R;
  common::Matrix<Scalar, 3, 3> mat_P;
  common::Matrix<Scalar, 3, 3> mat_tmp_3v3;
  common::Mat_Mul(mat_G, mat_covariance_, mat_tmp_3v3);
  common::Mat_Mul(mat_tmp_3v3, mat_G_tran, mat_P);
  mat_P(0, 0) += state_trans_noise.x_noise;
  mat_P(1, 1) += state_trans_noise.y_noise;
  mat_P(2, 2) += state_trans_noise.heading_noise;

  // std::cout << "mat_P=\n" << mat_P << std::endl;

  // Covariance of observation noise
  StateNoise lane_mark_noise;
  GetMeasurementNoiseOfLaneMark(lane_mark_filter_ret.belief,
        lane_mark_filter_ret.estimated_pos_start.heading, &lane_mark_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "lane mark noise = (" << lane_mark_noise.x_noise
            << ", " << lane_mark_noise.y_noise
            << ", " << common::com_rad2deg(lane_mark_noise.heading_noise)
            << ")" << std::endl;
#endif

  StateNoise gnss_noise;
  GetMeasurementNoiseOfGnss(gnss_filter_ret.belief,
        gnss_filter_ret.estimated_pos_start.heading, &gnss_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "gnss noise = (" << gnss_noise.x_noise
            << ", " << gnss_noise.y_noise
            << ", " << common::com_rad2deg(gnss_noise.heading_noise)
            << ")" << std::endl;
#endif

  // 计算kalman增益矩阵 [ K = Cov_pred * H_t * (H * Cov_pred * H_t + Q)^-1 ]
  /*
   *  H * Cov_pred * H_t
   *
   *  | I(3v3) |   |   |   |               |
   *  |        | * | P | * | I(3v3) I(3v3) |
   *  | I(3v3) |   |   |   |               |
   *
   */
  common::Matrix<Scalar, 6, 6> mat_tmp_6v6;
  common::Matrix<Scalar, 6, 6> mat_tmp_6v6_2;
  for (Int32_t j = 0; j < 6; ++j) {
    for (Int32_t i = 0; i < 6; ++i) {
      mat_tmp_6v6(i, j) = mat_P(i % 3, j % 3);
    }
  }

  // std::cout << "mat_tmp_6v6=\n" << mat_tmp_6v6 << std::endl;

  // H * Cov_pred * H_t + Q
  mat_tmp_6v6(0, 0) += lane_mark_noise.x_noise;
  mat_tmp_6v6(1, 1) += lane_mark_noise.y_noise;
  mat_tmp_6v6(2, 2) += lane_mark_noise.heading_noise;
  mat_tmp_6v6(3, 3) += gnss_noise.x_noise;
  mat_tmp_6v6(4, 4) += gnss_noise.y_noise;
  mat_tmp_6v6(5, 5) += gnss_noise.heading_noise;
  // (H * Cov_pred * H_t + Q)^-1
  common::Mat_CalcPseudoInverse(mat_tmp_6v6, mat_tmp_6v6_2);

  // std::cout << "mat_tmp_6v6_2=\n" << mat_tmp_6v6_2 << std::endl;

  /*
   *  H_t * (H * Cov_pred * H_t + Q)^-1
   *
   *  |               |   | part1(3v3)  part2(3v3) |
   *  | I(3v3) I(3v3) | * |                        |
   *  |               |   | part3(3v3)  part4(3v3) |
   */
  common::Matrix<Scalar, 3, 6> mat_tmp_3v6;
  for (Int32_t j = 0; j < 6; ++j) {
    for (Int32_t i = 0; i < 3; ++i) {
      mat_tmp_3v6(i, j) = mat_tmp_6v6_2(i, j) + mat_tmp_6v6_2(i+3, j);
    }
  }

  // std::cout << "mat_tmp_3v6=\n" << mat_tmp_3v6 << std::endl;

  // K = Cov * H_t * (H * Cov * H_t + Q)^-1
  common::Matrix<Scalar, 3, 6> mat_K;
  common::Mat_Mul(mat_P, mat_tmp_3v6, mat_K);

  // std::cout << "mat_K=\n" << mat_K << std::endl;

  // 使用观测量修正状态量 [ bel = bel_pred + K(z - z_pred) ]
  filter_ret->estimated_pos_end.x = pos_predicted[0] +
      mat_K(0, 0) * (pos_end_lane_mark[0] - pos_predicted[0]) +
      mat_K(0, 1) * (pos_end_lane_mark[1] - pos_predicted[1]) +
      mat_K(0, 2) * (pos_end_lane_mark[2] - pos_predicted[2]) +
      mat_K(0, 3) * (pos_end_gnss[0] - pos_predicted[0]) +
      mat_K(0, 4) * (pos_end_gnss[1] - pos_predicted[1]) +
      mat_K(0, 5) * (pos_end_gnss[2] - pos_predicted[2]);
  filter_ret->estimated_pos_end.y = pos_predicted[1] +
      mat_K(1, 0) * (pos_end_lane_mark[0] - pos_predicted[0]) +
      mat_K(1, 1) * (pos_end_lane_mark[1] - pos_predicted[1]) +
      mat_K(1, 2) * (pos_end_lane_mark[2] - pos_predicted[2]) +
      mat_K(1, 3) * (pos_end_gnss[0] - pos_predicted[0]) +
      mat_K(1, 4) * (pos_end_gnss[1] - pos_predicted[1]) +
      mat_K(1, 5) * (pos_end_gnss[2] - pos_predicted[2]);
  filter_ret->estimated_pos_end.heading = pos_predicted[2] +
      mat_K(2, 0) * (pos_end_lane_mark[0] - pos_predicted[0]) +
      mat_K(2, 1) * (pos_end_lane_mark[1] - pos_predicted[1]) +
      mat_K(2, 2) * (pos_end_lane_mark[2] - pos_predicted[2]) +
      mat_K(2, 3) * (pos_end_gnss[0] - pos_predicted[0]) +
      mat_K(2, 4) * (pos_end_gnss[1] - pos_predicted[1]) +
      mat_K(2, 5) * (pos_end_gnss[2] - pos_predicted[2]);

  filter_ret->corrected_delta_pos[0] =
      filter_ret->estimated_pos_end.x - filter_ret->estimated_pos_start.x;
  filter_ret->corrected_delta_pos[1] =
      filter_ret->estimated_pos_end.y - filter_ret->estimated_pos_start.y;
  filter_ret->corrected_delta_pos[2] = common::AngleDiff(
        filter_ret->estimated_pos_start.heading,
        filter_ret->estimated_pos_end.heading);

  Scalar time_elapsed_s = 0.001F * static_cast<Scalar>(
        filter_ret->time_end - filter_ret->time_start);
  if (time_elapsed_s < 0.005F) {
    time_elapsed_s = 0.005F;
  }
  filter_ret->corrected_velocity =
      common::com_sqrt(common::Square(filter_ret->corrected_delta_pos[0]) +
      common::Square(filter_ret->corrected_delta_pos[1])) / time_elapsed_s;
  if (curr_velocity_ < -0.001F) {
    // For backing mode of vehicle
    filter_ret->corrected_velocity = -filter_ret->corrected_velocity;
  }

  filter_ret->corrected_yaw_rate =
      filter_ret->corrected_delta_pos[2] / time_elapsed_s;

  // 使用观测量修正协方差矩阵 [ Cov = (I - K * H) * Cov_pred ]
  /*
   *  I - K * H
   *
   *  |        |   |                 |   | I(3v3) |
   *  | I(3v3) | - | K1(3v3) K2(3v3) | * |        |
   *  |        |   |                 |   | I(3v3) |
   */
  for (Int32_t j = 0; j < 3; ++j) {
    for (Int32_t i = 0; i < 3; ++i) {
      if (i == j) {
        mat_tmp_3v3(i, j) = 1.0F - mat_K(i, j) - mat_K(i, j+3);
      } else {
        mat_tmp_3v3(i, j) = -mat_K(i, j) - mat_K(i, j+3);
      }
    }
  }

  // std::cout << "mat_tmp_3v3=\n" << mat_tmp_3v3 << std::endl;

  // (I - K * H) * Cov_pred
  common::Mat_Mul(mat_tmp_3v3, mat_P, mat_covariance_);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "After kalman filter:" << std::endl;
  std::cout << "pos_filtered = (" << filter_ret->estimated_pos_end.x
            << "," << filter_ret->estimated_pos_end.y
            << "," << common::com_rad2deg(filter_ret->estimated_pos_end.heading)
            << ")."
            << std::endl;
  std::cout << "mat_covariance_=\n" << mat_covariance_ << std::endl;

  std::cout << "Correct (v=" << filter_ret->estimated_pos_end.v*3.6F
            << " km/h, yaw_rate="
            << common::com_rad2deg(filter_ret->estimated_pos_end.yaw_rate)
            << " deg/s) to (v=" << filter_ret->corrected_velocity * 3.6F
            << " km/s, yaw_rate="
            << common::com_rad2deg(filter_ret->corrected_yaw_rate)
            << " deg/s)." << std::endl;
#endif

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterPosByLaneMarkAndGnss (End) ###" << std::endl;
#endif
}

void PosFilterImpl::FilterPosByGnss(
    const FilterRet& gnss_filter_ret,
    FilterRet* filter_ret) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterPosByGnss (Start) ###" << std::endl;
#endif
  Int64_t time_start = 0;
  Int64_t time_end = 0;
  Scalar pos_predicted[3] = { 0.0F };
  Scalar pos_end_gnss[3] = { 0.0F };

  time_start = gnss_filter_ret.time_start;
  filter_ret->time_start = gnss_filter_ret.time_start;
  filter_ret->estimated_pos_start = gnss_filter_ret.estimated_pos_start;

  time_end = gnss_filter_ret.time_end;
  filter_ret->time_end = gnss_filter_ret.time_end;
  filter_ret->estimated_pos_end = gnss_filter_ret.estimated_pos_end;

  // 通过车身状态估计的位置（运动状态更新）
  pos_predicted[0] = gnss_filter_ret.estimated_pos_end.x;
  pos_predicted[1] = gnss_filter_ret.estimated_pos_end.y;
  pos_predicted[2] = gnss_filter_ret.estimated_pos_end.heading;

  // 通过相机识别的车道线估计的位置(观测量)
  pos_end_gnss[0] = gnss_filter_ret.estimated_pos_start.x +
      gnss_filter_ret.corrected_delta_pos[0];
  pos_end_gnss[1] = gnss_filter_ret.estimated_pos_start.y +
      gnss_filter_ret.corrected_delta_pos[1];
  pos_end_gnss[2] = common::NormalizeAngle(
        gnss_filter_ret.estimated_pos_start.heading +
        gnss_filter_ret.corrected_delta_pos[2]);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "pos_end_gnss("
            << pos_end_gnss[0]
            << "," << pos_end_gnss[1]
            << "," << common::com_rad2deg(pos_end_gnss[2])
      << "), pos_predicted(" << pos_predicted[0]
      << "," << pos_predicted[1]
      << "," << common::com_rad2deg(pos_predicted[2])
      << ")." << std::endl;

  std::cout << "time_start=" << time_start
            << ", time_end=" << time_end
            << std::endl;
#endif

  // 计算状态方程用的时间间隔
  Scalar delta_t = 0.001F * static_cast<Scalar>(
        common::Max(time_end, pos_filter_timeinterval_ / 2));
  // 状态方程用的前一时刻的位置
  const FilteredPos& prev_pos = filtered_pos_list_.Front();
  // 更新状态方程的系数矩阵
  common::Matrix<Scalar, 3, 3> mat_G;
  mat_G.SetIdentity();
  Scalar sin_prev_heading = common::com_sin(prev_pos.heading);
  Scalar cos_prev_heading = common::com_cos(prev_pos.heading);
  if (common::com_abs(prev_pos.yaw_rate) < 0.001F) {
    mat_G(0, 2) = -prev_pos.v * sin_prev_heading * delta_t;
    mat_G(1, 2) = prev_pos.v * cos_prev_heading * delta_t;
  } else {
    Scalar r = prev_pos.v / prev_pos.yaw_rate;
    Scalar heading_changed = phoenix::common::NormalizeAngle(
          prev_pos.heading + prev_pos.yaw_rate*delta_t);
    Scalar sin_heading_changed = common::com_sin(heading_changed);
    Scalar cos_heading_changed = common::com_cos(heading_changed);

    mat_G(0, 2) = -r*cos_prev_heading + r*cos_heading_changed;
    mat_G(1, 2) = -r*sin_prev_heading + r*sin_heading_changed;
  }
  common::Matrix<Scalar, 3, 3> mat_G_tran;
  common::Mat_Transpose(mat_G, mat_G_tran);

  // Covariance of the state transition noise
  StateNoise state_trans_noise;
  GetStateTransitionNoise(prev_pos.v, prev_pos.yaw_rate, &state_trans_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "predict noise = (" << state_trans_noise.x_noise
            << ", " << state_trans_noise.y_noise
            << ", " << common::com_rad2deg(state_trans_noise.heading_noise)
            << ")" << std::endl;
#endif

  // Covariance of state belief after prediction
  // P = G * P_ * G.transpose() + R;
  common::Matrix<Scalar, 3, 3> mat_P;
  common::Matrix<Scalar, 3, 3> mat_tmp_3v3;
  common::Mat_Mul(mat_G, mat_covariance_, mat_tmp_3v3);
  common::Mat_Mul(mat_tmp_3v3, mat_G_tran, mat_P);
  mat_P(0, 0) += state_trans_noise.x_noise;
  mat_P(1, 1) += state_trans_noise.y_noise;
  mat_P(2, 2) += state_trans_noise.heading_noise;

  // std::cout << "mat_P=\n" << mat_P << std::endl;

  // Covariance of observation noise
  StateNoise gnss_noise;
  GetMeasurementNoiseOfGnss(gnss_filter_ret.belief,
        gnss_filter_ret.estimated_pos_start.heading, &gnss_noise);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "gnss noise = (" << gnss_noise.x_noise
            << ", " << gnss_noise.y_noise
            << ", " << common::com_rad2deg(gnss_noise.heading_noise)
            << ")" << std::endl;
#endif

  // 计算kalman增益矩阵 [ K = Cov_pred * H_t * (H * Cov_pred * H_t + Q)^-1 ]
  /*
     *  H * Cov_pred * H_t
     *
     *  |        |   |   |   |        |
     *  | I(3v3) | * | P | * | I(3v3) |
     *  |        |   |   |   |        |
     *
     */
  common::Matrix<Scalar, 3, 3> mat_tmp_3v3_2;
  mat_tmp_3v3 = mat_P;

  // std::cout << "mat_tmp_3v3=\n" << mat_tmp_3v3 << std::endl;

  // H * Cov_pred * H_t + Q
  mat_tmp_3v3(0, 0) += gnss_noise.x_noise;
  mat_tmp_3v3(1, 1) += gnss_noise.y_noise;
  mat_tmp_3v3(2, 2) += gnss_noise.heading_noise;
  // (H * Cov_pred * H_t + Q)^-1
  common::Mat_CalcPseudoInverse(mat_tmp_3v3, mat_tmp_3v3_2);

  // std::cout << "mat_tmp_3v3_2=\n" << mat_tmp_3v3_2 << std::endl;

  /*
     *  H_t * (H * Cov_pred * H_t + Q)^-1
     *
     *  |        |   |       |
     *  | I(3v3) | * | (3v3) |
     *  |        |   |       |
     */

  // K = Cov * H_t * (H * Cov * H_t + Q)^-1
  common::Matrix<Scalar, 3, 3> mat_K;
  common::Mat_Mul(mat_P, mat_tmp_3v3_2, mat_K);

  // std::cout << "mat_K=\n" << mat_K << std::endl;

  // 使用观测量修正状态量 [ bel = bel_pred + K(z - z_pred) ]
  filter_ret->estimated_pos_end.x = pos_predicted[0] +
      mat_K(0, 0) * (pos_end_gnss[0] - pos_predicted[0]) +
      mat_K(0, 1) * (pos_end_gnss[1] - pos_predicted[1]) +
      mat_K(0, 2) * (pos_end_gnss[2] - pos_predicted[2]);
  filter_ret->estimated_pos_end.y = pos_predicted[1] +
      mat_K(1, 0) * (pos_end_gnss[0] - pos_predicted[0]) +
      mat_K(1, 1) * (pos_end_gnss[1] - pos_predicted[1]) +
      mat_K(1, 2) * (pos_end_gnss[2] - pos_predicted[2]);
  filter_ret->estimated_pos_end.heading = pos_predicted[2] +
      mat_K(2, 0) * (pos_end_gnss[0] - pos_predicted[0]) +
      mat_K(2, 1) * (pos_end_gnss[1] - pos_predicted[1]) +
      mat_K(2, 2) * (pos_end_gnss[2] - pos_predicted[2]);

  filter_ret->corrected_delta_pos[0] =
      filter_ret->estimated_pos_end.x - filter_ret->estimated_pos_start.x;
  filter_ret->corrected_delta_pos[1] =
      filter_ret->estimated_pos_end.y - filter_ret->estimated_pos_start.y;
  filter_ret->corrected_delta_pos[2] = common::AngleDiff(
        filter_ret->estimated_pos_start.heading,
        filter_ret->estimated_pos_end.heading);

  Scalar time_elapsed_s = 0.001F * static_cast<Scalar>(
        filter_ret->time_end - filter_ret->time_start);
  if (time_elapsed_s < 0.005F) {
    time_elapsed_s = 0.005F;
  }
  filter_ret->corrected_velocity =
      common::com_sqrt(common::Square(filter_ret->corrected_delta_pos[0]) +
      common::Square(filter_ret->corrected_delta_pos[1])) / time_elapsed_s;
  if (curr_velocity_ < -0.001F) {
    // For backing mode of vehicle
    filter_ret->corrected_velocity = -filter_ret->corrected_velocity;
  }

  filter_ret->corrected_yaw_rate =
      filter_ret->corrected_delta_pos[2] / time_elapsed_s;

  // 使用观测量修正协方差矩阵 [ Cov = (I - K * H) * Cov_pred ]
  /*
     *  I - K * H
     *
     *  |        |   |         |   |        |
     *  | I(3v3) | - | K(3v3)  | * | I(3v3) |
     *  |        |   |         |   |        |
     */
  for (Int32_t j = 0; j < 3; ++j) {
    for (Int32_t i = 0; i < 3; ++i) {
      if (i == j) {
        mat_tmp_3v3(i, j) = 1.0F - mat_K(i, j);
      } else {
        mat_tmp_3v3(i, j) = -mat_K(i, j);
      }
    }
  }

  // std::cout << "mat_tmp_3v3=\n" << mat_tmp_3v3 << std::endl;

  // (I - K * H) * Cov_pred
  common::Mat_Mul(mat_tmp_3v3, mat_P, mat_covariance_);

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "After kalman filter:" << std::endl;
  std::cout << "pos_filtered = (" << filter_ret->estimated_pos_end.x
            << "," << filter_ret->estimated_pos_end.y
            << "," << common::com_rad2deg(filter_ret->estimated_pos_end.heading)
            << ")."
            << std::endl;
  std::cout << "mat_covariance_=\n" << mat_covariance_ << std::endl;

  std::cout << "Correct (v=" << filter_ret->estimated_pos_end.v*3.6F
            << " km/h, yaw_rate="
            << common::com_rad2deg(filter_ret->estimated_pos_end.yaw_rate)
            << " deg/s) to (v=" << filter_ret->corrected_velocity * 3.6F
            << " km/s, yaw_rate="
            << common::com_rad2deg(filter_ret->corrected_yaw_rate)
            << " deg/s)." << std::endl;
#endif

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterPosByGnss (End) ###" << std::endl;
#endif
}


void PosFilterImpl::GetStateTransitionNoise(
    Scalar v, Scalar yaw_rate, StateNoise* noise) {
  noise->x_noise =
      0.02F * common::com_abs(v) +
      0.2F * common::com_abs(yaw_rate);

  noise->y_noise = noise->x_noise;

  noise->heading_noise =
      0.002F * common::com_abs(v) +
      0.02F * common::com_abs(yaw_rate);
}

void PosFilterImpl::GetMeasurementNoiseOfLaneMark(
    Scalar belief, Scalar heading, StateNoise* noise) {
  Int32_t quality = lane_mark_camera_filter_.GetFilterQuality();

  switch (quality) {
  case (LaneMarkCameraFilter::FILTER_QUALITY_GOOD): {
#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Lane Mark Filter quality is good." << std::endl;
#endif
    noise->x_noise =
        0.05F +
        0.2F * common::com_abs(common::com_cos(heading)) +
        0.2F * (1.0F - belief);;
    noise->y_noise =
        0.05F +
        0.1F * common::com_abs(common::com_sin(heading)) +
        0.2F * (1.0F - belief);;
    noise->heading_noise = 0.008F;
  }
    break;

  case (LaneMarkCameraFilter::FILTER_QUALITY_NOT_SO_GOOD): {
#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Lane Mark Filter quality is not so good." << std::endl;
#endif
    noise->x_noise =
        0.08F + 0.2F*common::com_abs(common::com_cos(heading)) +
        0.2F * (1.0F - belief);;
    noise->y_noise =
        0.08F +
        0.1F * common::com_abs(common::com_sin(heading)) +
        0.2F * (1.0F - belief);;
    noise->heading_noise = 0.01F;
  }
    break;

  case (LaneMarkCameraFilter::FILTER_QUALITY_BAD): {
#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Lane Mark Filter quality is bad." << std::endl;
#endif
    noise->x_noise =
        0.15F +
        0.2F * common::com_abs(common::com_cos(heading)) +
        0.2F * (1.0F - belief);;
    noise->y_noise =
        0.15F +
        0.1F * common::com_abs(common::com_sin(heading)) +
        0.2F * (1.0F - belief);;
    noise->heading_noise = 0.2F;
  }
    break;

  default:
#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Lane Mark Filter quality is invalid." << std::endl;
#endif
    noise->x_noise = 2.0F;
    noise->y_noise = 2.0F;
    noise->heading_noise = 1.0F;
    break;
  }

}

void PosFilterImpl::GetMeasurementNoiseOfGnss(
    Scalar belief, Scalar heading, StateNoise* noise) {
  if (prev_gnss_valid_) {
    if ((ad_msg::Gnss::STATUS_GOOD == localization_status_) &&
        (ad_msg::Gnss::STATUS_GOOD == prev_localization_status_)) {
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Gnss quality is good." << std::endl;
#endif
      noise->x_noise =
          0.1F +
          0.3F * common::com_abs(common::com_cos(heading)) +
          0.5F * (1.0F - belief);
      noise->y_noise =
          0.1F +
          0.1F*common::com_abs(common::com_sin(heading)) +
          0.3F * (1.0F - belief);
      ;
      noise->heading_noise = 0.01F;
    } else if (ad_msg::Gnss::STATUS_CONVERGING == localization_status_ &&
              ((ad_msg::Gnss::STATUS_GOOD == prev_localization_status_) ||
               (ad_msg::Gnss::STATUS_CONVERGING == prev_localization_status_))) {
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Gnss quality is not so good." << std::endl;
#endif
      noise->x_noise =
          0.2F +
          0.3F * common::com_abs(common::com_cos(heading)) +
          0.5F * (1.0F - belief);
      noise->y_noise =
          0.2F +
          0.1F * common::com_abs(common::com_sin(heading)) +
          0.3F * (1.0F - belief);
      noise->heading_noise = 0.02F;
    } else if (ad_msg::Gnss::STATUS_BAD == localization_status_ &&
              (ad_msg::Gnss::STATUS_INVALID != prev_localization_status_)) {
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Gnss quality is bad." << std::endl;
#endif
      noise->x_noise = 2.0F;
      noise->y_noise = noise->x_noise;
      noise->heading_noise = 0.2F;
    } else {
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "Gnss quality is invalid." << std::endl;
#endif
      noise->x_noise = 5.0F;
      noise->y_noise = noise->x_noise;
      noise->heading_noise = 1.0F;
    }
  } else {
#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Gnss quality is invalid." << std::endl;
#endif
    noise->x_noise = 5.0F;
    noise->y_noise = noise->x_noise;
    noise->heading_noise = 1.0F;
  }
}

bool PosFilterImpl::FindPosFromListByTimestamp(
    Int64_t relative_time, FilteredPos* pos) const {
  Int32_t filtered_pos_list_size = filtered_pos_list_.Size();

  if (filtered_pos_list_size < 1) {
    return false;
  }

  bool find_flag = false;
  for (Int32_t i = 0; i < filtered_pos_list_size; ++i) {
    const FilteredPos& p = filtered_pos_list_[i];
    if (p.relative_time <= relative_time) {
      Int32_t time_diff = relative_time - p.relative_time;
      if (time_diff > 500) {
        LOG_ERR << "Time interval is too large.";
        return false;
      }

      if (0 == i) {
        Scalar prev_pos[3] = { 0.0F };
        Scalar next_pos[3] = { 0.0F };
        prev_pos[0] = p.x;
        prev_pos[1] = p.y;
        prev_pos[2] = p.heading;
        vehicle_model_.EstimateNextPos(
              p.v, p.yaw_rate, 0.001F*time_diff, prev_pos, next_pos);

        pos->relative_time = time_diff + p.relative_time;
        pos->x = next_pos[0];
        pos->y = next_pos[1];
        pos->heading = next_pos[2];

        Scalar t = static_cast<Scalar>(time_diff) /
            static_cast<Scalar>(pos_filter_timeinterval_);
        // pos->yaw_rate = p.yaw_rate;
        // pos->v = p.v;
        pos->yaw_rate = (1.0F - t) * p.yaw_rate + t * curr_yaw_rate_;
        pos->v = (1.0F - t) * p.v + t * curr_velocity_;
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "Find pos case 1, t=" << t << std::endl;
#endif
      } else {
        const FilteredPos& next_p = filtered_pos_list_[i-1];
        Scalar t = static_cast<Scalar>(time_diff) /
            static_cast<Scalar>(next_p.relative_time - p.relative_time);

        pos->relative_time = time_diff + p.relative_time;
        pos->x = (1.0F - t) * p.x + t * next_p.x;
        pos->y = (1.0F - t) * p.y + t * next_p.y;
        pos->heading = common::AngleLerp(p.heading, next_p.heading, t);
        pos->yaw_rate = (1.0F - t) * p.yaw_rate + t * next_p.yaw_rate;
        pos->v = (1.0F - t) * p.v + t * next_p.v;
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "Find pos case 2" << std::endl;
#endif
      }
      find_flag = true;
      break;
    }
  }

  if (!find_flag) {
    const FilteredPos& p = filtered_pos_list_.Back();
    Int32_t time_diff = p.relative_time - relative_time;
    if (time_diff > 500) {
      LOG_ERR << "Time interval is too large.";
      return false;
    }
    Scalar prev_pos[3] = { 0.0F };
    Scalar next_pos[3] = { 0.0F };
    prev_pos[0] = p.x;
    prev_pos[1] = p.y;
    prev_pos[2] = p.heading;
    vehicle_model_.EstimateNextPos(
          p.v, p.yaw_rate, -0.001F*time_diff, prev_pos, next_pos);

    pos->relative_time = p.relative_time - time_diff;
    pos->x = next_pos[0];
    pos->y = next_pos[1];
    pos->heading = next_pos[2];
    pos->yaw_rate = p.yaw_rate;
    pos->v = p.v;

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Find pos case 3" << std::endl;
#endif
  }

  return true;
}

void PosFilterImpl::SmoothPosBtwTimestampsInList(
    Int32_t start_rel_time,
    Int32_t end_rel_time,
    const FilteredPos& start_pos,
    const Scalar corrected_delta_pos[3]) {
  Int32_t filtered_pos_list_size = filtered_pos_list_.Size();

  if (filtered_pos_list_size < 1) {
    return;
  }
  Int32_t timeinterval = end_rel_time - start_rel_time;
  if (timeinterval <= 0) {
    return;
  }

  Int32_t end_index = -1;
  for (Int32_t i = 0; i < filtered_pos_list_size; ++i) {
    const FilteredPos& p = filtered_pos_list_[i];
    if (p.relative_time <= end_rel_time) {
      end_index = i;
      break;
    }
  }
  Int32_t start_index = -1;
  for (Int32_t i = 0; i < filtered_pos_list_size; ++i) {
    const FilteredPos& p = filtered_pos_list_[i];
    if (p.relative_time <= start_rel_time) {
      start_index = i-1;
      if (start_index < 0) {
        start_index = 0;
      }
      break;
    }
  }
  if (end_index >= 0) {
    if (start_index < 0) {
      start_index = filtered_pos_list_size - 1;
    }
  }
  if ((end_index < 0) || (start_index < 0) || (start_index < end_index)) {
    return;
  }

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "Smooth pos list, between " << start_index
            << " and " << end_index << std::endl;
#endif

  Scalar corrected_pos[3] = { 0.0F };
  for (Int32_t i = start_index; i >= end_index; --i) {
    FilteredPos& p = filtered_pos_list_[i];
    Scalar t = static_cast<Scalar>(p.relative_time - start_rel_time) /
        static_cast<Scalar>(timeinterval);

    corrected_pos[0] = start_pos.x + t * corrected_delta_pos[0];
    corrected_pos[1] = start_pos.y + t * corrected_delta_pos[1];
    corrected_pos[2] = common::NormalizeAngle(
          start_pos.heading + t*corrected_delta_pos[2]);

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "Smooth pos[" << i << "] from ("
              << p.x << ", " << p.y << ", " << common::com_rad2deg(p.heading)
              << ") ";
#endif

    p.x = (1.0F - t) * p.x + t * corrected_pos[0];
    p.y = (1.0F - t) * p.y + t * corrected_pos[1];
    p.heading = common::AngleLerp(p.heading, corrected_pos[2], t);

#if ENABLE_POS_FILTER_IMPL_TRACE
    std::cout << "to ("
              << p.x << ", " << p.y << ", " << common::com_rad2deg(p.heading)
              << ") ." << std::endl;
#endif
  }

  if ((end_index - 1) >= 0) {
    FilteredPos& prev_p = filtered_pos_list_[end_index];
    FilteredPos& p = filtered_pos_list_[end_index - 1];
    Scalar t = static_cast<Scalar>(end_rel_time - prev_p.relative_time) /
        static_cast<Scalar>(p.relative_time - prev_p.relative_time);

    corrected_pos[0] = start_pos.x + corrected_delta_pos[0] +
        (1.0F - t) * p.delta_x;
    corrected_pos[1] = start_pos.y + corrected_delta_pos[1] +
        (1.0F - t) * p.delta_y;
    corrected_pos[2] = common::NormalizeAngle(
          start_pos.heading + corrected_delta_pos[2] +
        (1.0F - t) * p.delta_heading);

    p.x = corrected_pos[0];
    p.y = corrected_pos[1];
    p.heading = corrected_pos[2];
  }

  for (Int32_t i = (end_index-2); i >= 0; --i) {
    FilteredPos& prev_p = filtered_pos_list_[i+1];
    FilteredPos& p = filtered_pos_list_[i];

    p.x = prev_p.x + p.delta_x;
    p.y = prev_p.x + p.delta_y;
    p.heading = common::NormalizeAngle(prev_p.heading + p.delta_heading);
  }
}

PosFilterImpl::Scalar PosFilterImpl::CalcPosProbBasedOnVelocity(
    Scalar v, Scalar yaw_rate, Scalar delta_t,
    const Scalar pos_start[3], const Scalar pos_end[3]) {
  // std::cout << "### CalcPosProbBasedOnVelocity (Begin) ###" << std::endl;

  Scalar sin_angle = common::com_sin(pos_start[2]);
  Scalar cos_angle = common::com_cos(pos_start[2]);

  Scalar divisor =
      (pos_start[1] - pos_end[1]) * cos_angle -
      (pos_start[0] - pos_end[0]) * sin_angle;

  // std::cout << "From pos(" << pos_start[0] << "," << pos_start[1]
  //           << "," << common::com_rad2deg(pos_start[2])
  //           << ") to pos(" << pos_end[0] << "," << pos_end[1]
  //           << "," << common::com_rad2deg(pos_end[2])
  //           << "), v=" << v*3.6F
  //           << ", yaw_rate=" << common::com_rad2deg(yaw_rate)
  //           << ", delta_t=" << delta_t
  //           << std::endl;
  // std::cout << "divisor=" << divisor << std::endl;

  Scalar delta_angle = 0.0F;
  Scalar v_estimated = 0.0F;
  Scalar yaw_rate_estimated = 0.0F;
  Scalar yaw_rate_pert = 0.0F;
  if (common::com_abs(divisor) < 1e-4F) {
    delta_angle = 0.0F;
    Scalar dist = common::com_sqrt(common::Square(pos_start[0] - pos_end[0]) +
        common::Square(pos_start[1] - pos_end[1]));

    v_estimated = dist / delta_t;
    if (v < -0.001F) {
      // For backing mode of vehicle
      v_estimated = -v_estimated;
    }
  } else {
    Scalar ratio =
        0.5F * ((pos_start[0] - pos_end[0]) * cos_angle +
        (pos_start[1] - pos_end[1]) * sin_angle) / divisor;

    // std::cout << "ratio=" << ratio << std::endl;

    Scalar x_center = 0.5F * (pos_start[0] + pos_end[0]) +
        ratio * (pos_start[1] - pos_end[1]);
    Scalar y_center = 0.5F * (pos_start[1] + pos_end[1]) +
        ratio * (pos_end[0] - pos_start[0]);

    Scalar r = common::com_sqrt(common::Square(pos_start[0] - x_center) +
        common::Square(pos_start[1] - y_center));

    delta_angle = common::com_atan2(pos_end[1]-y_center, pos_end[0]-x_center) -
        common::com_atan2(pos_start[1]-y_center, pos_start[0]-x_center);

    // std::cout << "x_center=" << x_center << ", y_center=" << y_center
    //           << ", r=" << r << ", delta_angle="
    //           << common::com_rad2deg(delta_angle)
    //           << std::endl;

    v_estimated = common::com_abs(delta_angle) * r / delta_t;
    if (v < -0.001F) {
      // For backing mode of vehicle
      v_estimated = -v_estimated;
    }
  }

  yaw_rate_estimated = delta_angle / delta_t;
  yaw_rate_pert = (pos_end[2] - pos_start[2]) / delta_t - yaw_rate_estimated;

  // std::cout << "v_estimated=" << v_estimated*3.6F
  //           << ", yaw_rate_estimated="
  //           << common::com_rad2deg(yaw_rate_estimated)
  //           << ", yaw_rate_pert=" << common::com_rad2deg(yaw_rate_pert)
  //           << std::endl;

  Scalar abs_v = common::com_abs(v);
  Scalar abs_yaw_rate = common::com_abs(yaw_rate);

  Scalar variance_v = 0.05F * abs_v + 0.1f * abs_yaw_rate;
  Scalar variance_yaw_rate = 0.005F * abs_v + 0.05f * abs_yaw_rate;
  Scalar variance_yaw_rate_pert = 0.002F * abs_v + 0.03f * abs_yaw_rate;

  // std::cout << "variance_v=" << variance_v*3.6F
  //           << ", variance_yaw_rate=" << common::com_rad2deg(variance_yaw_rate)
  //           << ", variance_yaw_rate_pert="
  //           << common::com_rad2deg(variance_yaw_rate_pert)
  //           << std::endl;

  // std::cout << "v_diff=" << (v - v_estimated)*3.6F
  //           << ", yaw_rate_diff="
  //           << common::com_rad2deg(yaw_rate - yaw_rate_estimated)
  //           << ", yaw_rate_pert="
  //           << common::com_rad2deg(yaw_rate_pert)
  //           << std::endl;

  Scalar prob_v = CalcProbOfDistribution(v - v_estimated, variance_v);
  Scalar prob_yaw_rate = CalcProbOfDistribution(yaw_rate - yaw_rate_estimated,
                                                variance_yaw_rate);
  Scalar prob_yaw_rate_pert = CalcProbOfDistribution(yaw_rate_pert,
                                                     variance_yaw_rate_pert);

  // std::cout << "prob_v=" << prob_v
  //           << ", prob_yaw_rate=" << prob_yaw_rate
  //           << ", prob_yaw_rate_pert=" << prob_yaw_rate_pert
  //           << std::endl;

  // std::cout << "### CalcPosProbBasedOnVelocity (End) ###" << std::endl;

  return (prob_v*prob_yaw_rate*prob_yaw_rate_pert);
}

PosFilterImpl::Scalar PosFilterImpl::CalcProbOfDistribution(
    Scalar a, Scalar b) {
  if (b < 0.001F) {
    b = 0.001F;
  }

  Scalar prob = 0.0F;
#if 1
  Scalar abs_a = common::com_abs(a);
  Scalar tmp = common::com_sqrt(6.0F * b);

  if (abs_a > tmp) {
    prob = 0.0F;
  } else {
    prob = (tmp - abs_a) / (6.0F*b);
  }
#else
  prob = std::exp(-0.5F*a*a/b) / (common::com_sqrt(2*COM_PI*b));
#endif

  return (prob);
}

bool PosFilterImpl::FilterLaneMark(Int64_t curr_time_stamp,
    const ad_msg::LaneMarkCameraList& lane_mark_camera_list,
    bool lane_mark_valid_bel, const FilterRet& lane_mark_filter_ret) {
  if (!lane_mark_camera_list.msg_head.valid) {
    return false;
  }
  if (lane_mark_camera_list.lane_mark_num < 1) {
    return false;
  }

  Uint32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
        prev_lane_mark_msg_sequence_,
        lane_mark_camera_list.msg_head.sequence);
  if (0 == sequence_diff) {
    return true;
  }

  bool timeout = false;
  Int64_t time_elapsed = 0;
  if (prev_lane_mark_valid_) {
    time_elapsed = common::CalcElapsedClockMs(
          prev_lane_mark_timestamp_, lane_mark_camera_list.msg_head.timestamp);
    if (time_elapsed < 10) {
      return true;
    }
    if (time_elapsed > 500) {
      // 旧的数据过时了
      timeout = true;
      prev_lane_mark_valid_ = false;
      prev_lane_mark_msg_sequence_ =
          lane_mark_camera_list.msg_head.sequence;
      prev_lane_mark_timestamp_ =
          lane_mark_camera_list.msg_head.timestamp;
      lane_mark_camera_filter_.Reset();
      LOG_ERR << "Timeout to get camera lane.";
    }
  }

  Scalar delta_pos[3] = { 0.0F };
  Scalar heading_corrected = 0.0F;
  // 车道线滤波器中保存了有效的车道线信息
  bool find_pos_success = false;
  FilteredPos estimated_pos_start;
  FilteredPos estimated_pos_end;
  if (prev_filtered_pos_valid_ && !timeout) {
    // 保存了有效的过滤后的相对位置列表信息
    Int64_t time_end = common::CalcElapsedClockMs(
          curr_time_stamp, lane_mark_camera_list.msg_head.timestamp);
    Int64_t time_start = time_end - time_elapsed;
    if (FindPosFromListByTimestamp(time_end,
                                   &estimated_pos_end)) {
      // 找到了正确的相对位置1
      if (FindPosFromListByTimestamp(time_start,
                                     &estimated_pos_start)) {
        // 找到了正确的相对位置2
        find_pos_success = true;
      } else {
        LOG_ERR << "Failed to find relative pos of previous camera lane.";
      }
    } else {
      LOG_ERR << "Failed to find relative pos of current camera lane.";
    }
  }

  if (find_pos_success) {
    delta_pos[0] = estimated_pos_end.x - estimated_pos_start.x;
    delta_pos[1] = estimated_pos_end.y - estimated_pos_start.y;
    delta_pos[2] = common::NormalizeAngle(estimated_pos_end.heading -
                                          estimated_pos_start.heading);
    common::Matrix<Scalar, 3, 3> mat_convert;
    common::Matrix<Scalar, 2, 1> rotate_center;
    common::Matrix<Scalar, 2, 1> point_conv;
    mat_convert.SetIdentity();
    rotate_center.SetZeros();
    common::Rotate_2D<Scalar>(
          rotate_center, -estimated_pos_start.heading, &mat_convert);
    point_conv(0) = delta_pos[0];
    point_conv(1) = delta_pos[1];
    common::TransformVert_2D(mat_convert, &point_conv);
    delta_pos[0] = point_conv(0);
    delta_pos[1] = point_conv(1);

    if (lane_mark_valid_bel) {
      heading_corrected = common::NormalizeAngle(
            lane_mark_filter_ret.corrected_delta_pos[2] - delta_pos[2]);

      delta_pos[0] = lane_mark_filter_ret.corrected_delta_pos[0];
      delta_pos[1] = lane_mark_filter_ret.corrected_delta_pos[1];
      delta_pos[2] = lane_mark_filter_ret.corrected_delta_pos[2];
    }
  } else {
    lane_mark_camera_filter_.Reset();
  }

  // 更新车道线列表
  /* k006 pengc 2023-02-12 (begin) */
  // 避免低速下固定方向的角速度偏差导致车道线扭曲，进而避免导致计算出异常的目标方向盘角度
  bool match_prev_lane_mark = true;
  if (curr_velocity_ < 10.0F/3.6F) {
    match_prev_lane_mark = false;
  }
  /* k006 pengc 2023-02-12 (end) */
  bool update_flag = lane_mark_camera_filter_.UpdateLaneLineList(
        lane_mark_camera_list, delta_pos, 0.0F, curr_velocity_,
        match_prev_lane_mark);

  if (update_flag) {
    // 成功地更新了车道线列表
    prev_lane_mark_valid_ = true;
    prev_lane_mark_msg_sequence_ =
        lane_mark_camera_list.msg_head.sequence;
    prev_lane_mark_timestamp_ =
        lane_mark_camera_list.msg_head.timestamp;
  } else {
    // LOG_ERR << "Failed to update camera lane list.";
  }

  return (update_flag);
}

bool PosFilterImpl::FilterGnss(
    Int64_t curr_time_stamp,
    const ad_msg::Gnss& gnss_info, bool gnss_valid_bel) {
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterGnss (Start) ###" << std::endl;
#endif
  if (!gnss_info.msg_head.valid) {
    gnss_filter_.Clear();
    utm_filter_.Clear();
    odom_filter_.Clear();
    return false;
  }
  if (!filtered_gnss_info_.msg_head.valid) {
    filtered_gnss_info_ = gnss_info;
    gnss_filter_.Clear();
    utm_filter_.Clear();
    odom_filter_.Clear();
    return true;
  }
  if (!prev_filtered_pos_valid_) {
    filtered_gnss_info_ = gnss_info;
    gnss_filter_.Clear();
    utm_filter_.Clear();
    odom_filter_.Clear();
    return true;
  }

  Uint32_t sequence_diff = ad_msg::MsgHead::CalcSequenceDiff(
        filtered_gnss_info_.msg_head.sequence,
        gnss_info.msg_head.sequence);
  if (0 == sequence_diff) {
    return true;
  }

  Int64_t time_elapsed = common::CalcElapsedClockMs(
        filtered_gnss_info_.msg_head.timestamp, gnss_info.msg_head.timestamp);
  if (time_elapsed < 10) {
    return true;
  }
  if (time_elapsed > 500) {
    filtered_gnss_info_ = gnss_info;
    gnss_filter_.Clear();
    utm_filter_.Clear();
    odom_filter_.Clear();
    return false;
  }

  Float64_t delta_pos[3] = { 0.0F };
  bool find_pos_success = false;
  FilteredPos estimated_pos_start;
  FilteredPos estimated_pos_end;

  Int64_t time_end = common::CalcElapsedClockMs(
        curr_time_stamp, gnss_info.msg_head.timestamp);
  Int64_t time_start = time_end - time_elapsed;
  if (FindPosFromListByTimestamp(time_end, &estimated_pos_end)) {
    // 找到了正确的相对位置1
    if (FindPosFromListByTimestamp(time_start, &estimated_pos_start)) {
      // 找到了正确的相对位置2
      find_pos_success = true;
    } else {
      LOG_ERR << "Failed to find relative pos of previous gnss.";
    }
  } else {
    LOG_ERR << "Failed to find relative pos of current gnss.";
  }

  if (!find_pos_success) {
    filtered_gnss_info_ = gnss_info;
    gnss_filter_.Clear();
    utm_filter_.Clear();
    odom_filter_.Clear();
    return false;
  }

  delta_pos[0] = estimated_pos_end.x - estimated_pos_start.x;
  delta_pos[1] = estimated_pos_end.y - estimated_pos_start.y;
  delta_pos[2] = common::AngleDiff(estimated_pos_start.heading,
                                   estimated_pos_end.heading);

  common::Matrix<Float64_t, 3, 3> mat_convert;
  common::Matrix<Float64_t, 2, 1> rotate_center;
  common::Matrix<Float64_t, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Rotate_2D<Float64_t>(
        rotate_center, -estimated_pos_start.heading, &mat_convert);
  point_conv(0) = delta_pos[0];
  point_conv(1) = delta_pos[1];
  common::TransformVert_2D(mat_convert, &point_conv);
  delta_pos[0] = point_conv(0);
  delta_pos[1] = point_conv(1);

  Float64_t angle_inc = common::com_atan2(point_conv(1), point_conv(0));
  Float64_t dist_inc = common::com_sqrt(
        point_conv(0)*point_conv(0) + point_conv(1)*point_conv(1));
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "delta_pos={" << delta_pos[0]
            << ", " << delta_pos[1]
            << ", " << common::com_rad2deg(delta_pos[2])
            << "}, dist_inc=" << dist_inc
            << ", angle_inc=" << common::com_rad2deg(angle_inc)
            << std::endl;
#endif
  common::Matrix<Float64_t, 3, 1> vec_bel;
  common::Matrix<Float64_t, 3, 3> mat_covariance;
  common::Matrix<Float64_t, 3, 3> mat_q;
  mat_q.SetZeros();
  common::Matrix<Float64_t, 3, 3> mat_k;
  common::Matrix<Float64_t, 3, 3> mat_3v3_1;
  common::Matrix<Float64_t, 3, 3> mat_3v3_2;
  common::Matrix<Float64_t, 3, 1> mat_3v1_1;
  common::Matrix<Float64_t, 3, 1> mat_3v1_2;


  /// Filter GNSS
  Float64_t filtered_latitude = 0.0;
  Float64_t filtered_longitude = 0.0;
  Float64_t filtered_heading_gnss = 0.0F;
  Int32_t filtered_gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
  if ((ad_msg::Gnss::STATUS_INVALID != gnss_info.gnss_status) &&
      (ad_msg::Gnss::STATUS_INVALID != filtered_gnss_info_.gnss_status)) {
    common::GpsPoint gps_point_start(filtered_gnss_info_.latitude,
                                     filtered_gnss_info_.longitude);

    common::GpsPoint cur_gps_point(gnss_info.latitude, gnss_info.longitude);
    Float64_t dist_offset =
        common::CalcSphericalDistance(cur_gps_point, gps_point_start);
    if (dist_offset > 5.0F) {
      LOG_ERR << "Incorrect gnss coordinate, dist_offset=" << dist_offset;
      filtered_gnss_status = gnss_info.gnss_status;
      filtered_latitude = gnss_info.latitude;
      filtered_longitude = gnss_info.longitude;
      filtered_heading_gnss = gnss_info.heading_gnss;
      gnss_filter_.Clear();
    } else {
      common::GpsPoint est_gps_point = common::CalcNextGpsPoint(
            gps_point_start,
            common::NormalizeAngle(filtered_gnss_info_.heading_gnss + angle_inc),
            dist_inc);
      Float64_t est_heading = common::NormalizeAngle(
            filtered_gnss_info_.heading_gnss + delta_pos[2]);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "gnss={" << gnss_info.latitude
                << ", " << gnss_info.longitude
                << ", " << common::com_rad2deg(gnss_info.heading_gnss)
                << "}, est={" << est_gps_point.latitude
                << ", " << est_gps_point.longitude
                << ", " << common::com_rad2deg(est_heading)
                << "}" << std::endl;
#endif
      vec_bel(0, 0) = est_gps_point.latitude;
      vec_bel(1, 0) = est_gps_point.longitude;
      vec_bel(2, 0) = est_heading;
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "vec_bel={" << vec_bel(0, 0)
                << ", " << vec_bel(1, 0)
                << ", " << common::com_rad2deg(vec_bel(2, 0))
                << std::endl;
#endif
      mat_covariance = gnss_filter_.covariance;
      mat_covariance(0, 0) += 0.01F;
      mat_covariance(1, 1) += 0.01F;
      mat_covariance(2, 2) += 0.01F;
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_covariance=\n" << mat_covariance << std::endl;
#endif
      if (ad_msg::Gnss::STATUS_GOOD == gnss_info.gnss_status) {
        if (gnss_valid_bel) {
          mat_q(0, 0) = 2.0F;
          mat_q(1, 1) = 2.0F;
          mat_q(2, 2) = 2.0F;

          filtered_gnss_status = ad_msg::Gnss::STATUS_GOOD;
        } else {
          mat_q(0, 0) = 4.0F;
          mat_q(1, 1) = 4.0F;
          mat_q(2, 2) = 4.0F;

          filtered_gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
        }
      } else if (ad_msg::Gnss::STATUS_CONVERGING == gnss_info.gnss_status) {
        mat_q(0, 0) = 10.0F;
        mat_q(1, 1) = 10.0F;
        mat_q(2, 2) = 10.0F;

        filtered_gnss_status = ad_msg::Gnss::STATUS_CONVERGING;
      } else {
        mat_q(0, 0) = 20.0F;
        mat_q(1, 1) = 20.0F;
        mat_q(2, 2) = 20.0F;

        filtered_gnss_status = ad_msg::Gnss::STATUS_BAD;
      }
      common::Mat_Add(mat_covariance, mat_q, mat_3v3_1);
      common::Mat_CalcPseudoInverse(mat_3v3_1, mat_3v3_2);
      common::Mat_Mul(mat_covariance, mat_3v3_2, mat_k);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_3v3_2=\n" << mat_3v3_2 << std::endl;
      std::cout << "mat_k=\n" << mat_k << std::endl;
#endif
      mat_3v1_1(0, 0) = gnss_info.latitude - vec_bel(0, 0);
      mat_3v1_1(1, 0) = gnss_info.longitude - vec_bel(1, 0);
      mat_3v1_1(2, 0) =
          common::NormalizeAngle(gnss_info.heading_gnss - vec_bel(2, 0));
      common::Mat_Mul(mat_k, mat_3v1_1, mat_3v1_2);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_3v1_1={" << mat_3v1_1(0, 0)
                << ", " << mat_3v1_1(1, 0)
                << ", " << common::com_rad2deg(mat_3v1_1(2, 0))
                << std::endl;
      std::cout << "mat_3v1_2={" << mat_3v1_2(0, 0)
                << ", " << mat_3v1_2(1, 0)
                << ", " << common::com_rad2deg(mat_3v1_2(2, 0))
                << std::endl;
#endif
      filtered_latitude = vec_bel(0, 0) + mat_3v1_2(0, 0);
      filtered_longitude = vec_bel(1, 0) + mat_3v1_2(1, 0);
      filtered_heading_gnss =
          common::NormalizeAngle(vec_bel(2, 0) + mat_3v1_2(2, 0));
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "After filter: lat=" << filtered_latitude
                << ", lon=" << filtered_longitude
                << ", heading=" << common::com_rad2deg(filtered_heading_gnss)
                << std::endl;
#endif
      if (common::com_abs(common::AngleDiff(filtered_heading_gnss, est_heading)) > common::com_deg2rad(5.0)) {
        LOG_ERR << "Invalid filtered heading.";
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "mat_covariance=\n" << mat_covariance << std::endl;
        std::cout << "mat_k=\n" << mat_k << std::endl;
#endif
      }

      mat_3v3_1.SetIdentity();
      common::Mat_Sub(mat_3v3_1, mat_k, mat_3v3_2);
      common::Mat_Mul(mat_3v3_2, mat_covariance, gnss_filter_.covariance);
      if (gnss_filter_.covariance(0, 0) < 0.1) {
        gnss_filter_.covariance(0, 0) = 0.1;
      }
      if (gnss_filter_.covariance(1, 1) < 0.1) {
        gnss_filter_.covariance(1, 1) = 0.1;
      }
      if (gnss_filter_.covariance(2, 2) < 0.1) {
        gnss_filter_.covariance(2, 2) = 0.1;
      }
    }
  } else {
    filtered_gnss_status = gnss_info.gnss_status;
    filtered_latitude = gnss_info.latitude;
    filtered_longitude = gnss_info.longitude;
    filtered_heading_gnss = gnss_info.heading_gnss;
    gnss_filter_.Clear();
  }


  /// Filter UTM
  Float64_t filtered_x_utm = 0.0;
  Float64_t filtered_y_utm = 0.0;
  Float64_t filtered_heading_utm = 0.0F;
  Int32_t filtered_utm_status = ad_msg::Gnss::STATUS_CONVERGING;
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "gnss_info.utm_status=" << gnss_info.utm_status
            << ", filtered_gnss_info_.utm_status=" << filtered_gnss_info_.utm_status
            << std::endl;
#endif
  if ((ad_msg::Gnss::STATUS_INVALID != gnss_info.utm_status) &&
      (ad_msg::Gnss::STATUS_INVALID != filtered_gnss_info_.utm_status)) {
    Float64_t dist_offset = common::com_sqrt(
          common::Square(filtered_gnss_info_.x_utm - gnss_info.x_utm) +
          common::Square(filtered_gnss_info_.y_utm - gnss_info.y_utm));
    if (dist_offset > 5.0F) {
      LOG_ERR << "Incorrect utm coordinate filter("
    		  << filtered_gnss_info_.x_utm << "," << filtered_gnss_info_.y_utm
			  << ") curr(" << gnss_info.x_utm << "," << gnss_info.y_utm
			  << "), dist_offset=" << dist_offset;
      filtered_utm_status = gnss_info.utm_status;
#if 1
      filtered_x_utm = gnss_info.x_utm;
      filtered_y_utm = gnss_info.y_utm;
      filtered_heading_utm = gnss_info.heading_utm;
#else
      filtered_x_utm = filtered_gnss_info_.x_utm;
      filtered_y_utm = filtered_gnss_info_.y_utm;
      filtered_heading_utm = filtered_gnss_info_.heading_utm;
#endif
      utm_filter_.Clear();
    } else {
      Scalar angle = common::NormalizeAngle(
            filtered_gnss_info_.heading_utm + angle_inc);
      Float64_t est_x_utm =
          filtered_gnss_info_.x_utm + dist_inc*common::com_cos(angle);
      Float64_t est_y_utm =
          filtered_gnss_info_.y_utm + dist_inc*common::com_sin(angle);
      Float64_t est_heading = common::NormalizeAngle(
            filtered_gnss_info_.heading_utm + delta_pos[2]);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "utm={" << gnss_info.x_utm
                << ", " << gnss_info.y_utm
                << ", " << common::com_rad2deg(gnss_info.heading_utm)
                << "}, est={" << est_x_utm
                << ", " << est_y_utm
                << ", " << common::com_rad2deg(est_heading)
                << "}" << std::endl;
#endif
      vec_bel(0, 0) = est_x_utm;
      vec_bel(1, 0) = est_y_utm;
      vec_bel(2, 0) = est_heading;
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "vec_bel={" << vec_bel(0, 0)
                << ", " << vec_bel(1, 0)
                << ", " << common::com_rad2deg(vec_bel(2, 0))
                << std::endl;
#endif
      mat_covariance = utm_filter_.covariance;
      mat_covariance(0, 0) += 0.01F;
      mat_covariance(1, 1) += 0.01F;
      mat_covariance(2, 2) += 0.01F;
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_covariance=\n" << mat_covariance << std::endl;
#endif
      if (ad_msg::Gnss::STATUS_GOOD == gnss_info.utm_status) {
        if (gnss_valid_bel) {
          mat_q(0, 0) = 2.0F;
          mat_q(1, 1) = 2.0F;
          mat_q(2, 2) = 2.0F;

          filtered_utm_status = ad_msg::Gnss::STATUS_GOOD;
        } else {
          mat_q(0, 0) = 4.0F;
          mat_q(1, 1) = 4.0F;
          mat_q(2, 2) = 4.0F;

          filtered_utm_status = ad_msg::Gnss::STATUS_CONVERGING;
        }
      } else if (ad_msg::Gnss::STATUS_CONVERGING == gnss_info.gnss_status) {
        mat_q(0, 0) = 10.0F;
        mat_q(1, 1) = 10.0F;
        mat_q(2, 2) = 10.0F;

        filtered_utm_status = ad_msg::Gnss::STATUS_CONVERGING;
      } else {
        mat_q(0, 0) = 20.0F;
        mat_q(1, 1) = 20.0F;
        mat_q(2, 2) = 20.0F;

        filtered_utm_status = ad_msg::Gnss::STATUS_BAD;
      }
      common::Mat_Add(mat_covariance, mat_q, mat_3v3_1);
      common::Mat_CalcPseudoInverse(mat_3v3_1, mat_3v3_2);
      common::Mat_Mul(mat_covariance, mat_3v3_2, mat_k);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_3v3_2=\n" << mat_3v3_2 << std::endl;
      std::cout << "mat_k=\n" << mat_k << std::endl;
#endif
      mat_3v1_1(0, 0) = gnss_info.x_utm - vec_bel(0, 0);
      mat_3v1_1(1, 0) = gnss_info.y_utm - vec_bel(1, 0);
      mat_3v1_1(2, 0) =
          common::NormalizeAngle(gnss_info.heading_utm - vec_bel(2, 0));
      common::Mat_Mul(mat_k, mat_3v1_1, mat_3v1_2);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_3v1_1={" << mat_3v1_1(0, 0)
                << ", " << mat_3v1_1(1, 0)
                << ", " << common::com_rad2deg(mat_3v1_1(2, 0))
                << std::endl;
      std::cout << "mat_3v1_2={" << mat_3v1_2(0, 0)
                << ", " << mat_3v1_2(1, 0)
                << ", " << common::com_rad2deg(mat_3v1_2(2, 0))
                << std::endl;
#endif
      filtered_x_utm = vec_bel(0, 0) + mat_3v1_2(0, 0);
      filtered_y_utm = vec_bel(1, 0) + mat_3v1_2(1, 0);
      filtered_heading_utm =
          common::NormalizeAngle(vec_bel(2, 0) + mat_3v1_2(2, 0));
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "After filter: x=" << filtered_x_utm
                << ", y=" << filtered_y_utm
                << ", heading=" << common::com_rad2deg(filtered_heading_utm)
                << std::endl;
#endif
      if (common::com_abs(
            common::AngleDiff(filtered_heading_utm, est_heading)) >
          common::com_deg2rad(5.0)) {
        LOG_ERR << "Invalid filtered heading.";
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "mat_covariance=\n" << mat_covariance << std::endl;
        std::cout << "mat_k=\n" << mat_k << std::endl;
#endif
      }

      mat_3v3_1.SetIdentity();
      common::Mat_Sub(mat_3v3_1, mat_k, mat_3v3_2);
      common::Mat_Mul(mat_3v3_2, mat_covariance, utm_filter_.covariance);
      if (gnss_filter_.covariance(0, 0) < 0.1) {
        gnss_filter_.covariance(0, 0) = 0.1;
      }
      if (gnss_filter_.covariance(1, 1) < 0.1) {
        gnss_filter_.covariance(1, 1) = 0.1;
      }
      if (gnss_filter_.covariance(2, 2) < 0.1) {
        gnss_filter_.covariance(2, 2) = 0.1;
      }
    }
  } else {
    filtered_utm_status = gnss_info.utm_status;
    filtered_x_utm = gnss_info.x_utm;
    filtered_y_utm = gnss_info.y_utm;
    filtered_heading_utm = gnss_info.heading_utm;
    utm_filter_.Clear();
  }


  /// Filter ODOM
  Float64_t filtered_x_odom = 0.0;
  Float64_t filtered_y_odom = 0.0;
  Float64_t filtered_heading_odom = 0.0F;
  Int32_t filtered_odom_status = ad_msg::Gnss::STATUS_CONVERGING;
#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "gnss_info.odom_status=" << gnss_info.odom_status
            << ", filtered_gnss_info_.odom_status=" << filtered_gnss_info_.odom_status
            << std::endl;
#endif
  if ((ad_msg::Gnss::STATUS_INVALID != gnss_info.odom_status) &&
      (ad_msg::Gnss::STATUS_INVALID != filtered_gnss_info_.odom_status)) {
    Float64_t dist_offset = common::com_sqrt(
          common::Square(filtered_gnss_info_.x_odom - gnss_info.x_odom) +
          common::Square(filtered_gnss_info_.y_odom - gnss_info.y_odom));
    if (dist_offset > 5.0F) {
      LOG_ERR << "Incorrect ODOM coordinate filter("
          << filtered_gnss_info_.x_odom << "," << filtered_gnss_info_.y_odom
        << ") curr(" << gnss_info.x_odom << "," << gnss_info.y_odom
        << "), dist_offset=" << dist_offset;
      filtered_odom_status = gnss_info.odom_status;
#if 1
      filtered_x_odom = gnss_info.x_odom;
      filtered_y_odom = gnss_info.y_odom;
      filtered_heading_odom = gnss_info.heading_odom;
#else
      filtered_x_odom = filtered_gnss_info_.x_odom;
      filtered_y_odom = filtered_gnss_info_.y_odom;
      filtered_heading_odom = filtered_gnss_info_.heading_odom;
#endif
      odom_filter_.Clear();
    } else {
      Scalar angle = common::NormalizeAngle(
            filtered_gnss_info_.heading_odom + angle_inc);
      Float64_t est_x_odom =
          filtered_gnss_info_.x_odom + dist_inc*common::com_cos(angle);
      Float64_t est_y_odom =
          filtered_gnss_info_.y_odom + dist_inc*common::com_sin(angle);
      Float64_t est_heading = common::NormalizeAngle(
            filtered_gnss_info_.heading_odom + delta_pos[2]);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "odom={" << gnss_info.x_odom
                << ", " << gnss_info.y_odom
                << ", " << common::com_rad2deg(gnss_info.heading_odom)
                << "}, est={" << est_x_odom
                << ", " << est_y_odom
                << ", " << common::com_rad2deg(est_heading)
                << "}" << std::endl;
#endif
      vec_bel(0, 0) = est_x_odom;
      vec_bel(1, 0) = est_y_odom;
      vec_bel(2, 0) = est_heading;
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "vec_bel={" << vec_bel(0, 0)
                << ", " << vec_bel(1, 0)
                << ", " << common::com_rad2deg(vec_bel(2, 0))
                << std::endl;
#endif
      mat_covariance = odom_filter_.covariance;
      mat_covariance(0, 0) += 0.01F;
      mat_covariance(1, 1) += 0.01F;
      mat_covariance(2, 2) += 0.01F;
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_covariance=\n" << mat_covariance << std::endl;
#endif
      if (ad_msg::Gnss::STATUS_GOOD == gnss_info.odom_status) {
        if (gnss_valid_bel) {
          mat_q(0, 0) = 2.0F;
          mat_q(1, 1) = 2.0F;
          mat_q(2, 2) = 2.0F;

          filtered_odom_status = ad_msg::Gnss::STATUS_GOOD;
        } else {
          mat_q(0, 0) = 4.0F;
          mat_q(1, 1) = 4.0F;
          mat_q(2, 2) = 4.0F;

          filtered_odom_status = ad_msg::Gnss::STATUS_CONVERGING;
        }
      } else if (ad_msg::Gnss::STATUS_CONVERGING == gnss_info.gnss_status) {
        mat_q(0, 0) = 10.0F;
        mat_q(1, 1) = 10.0F;
        mat_q(2, 2) = 10.0F;

        filtered_odom_status = ad_msg::Gnss::STATUS_CONVERGING;
      } else {
        mat_q(0, 0) = 20.0F;
        mat_q(1, 1) = 20.0F;
        mat_q(2, 2) = 20.0F;

        filtered_odom_status = ad_msg::Gnss::STATUS_BAD;
      }
      common::Mat_Add(mat_covariance, mat_q, mat_3v3_1);
      common::Mat_CalcPseudoInverse(mat_3v3_1, mat_3v3_2);
      common::Mat_Mul(mat_covariance, mat_3v3_2, mat_k);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_3v3_2=\n" << mat_3v3_2 << std::endl;
      std::cout << "mat_k=\n" << mat_k << std::endl;
#endif
      mat_3v1_1(0, 0) = gnss_info.x_odom - vec_bel(0, 0);
      mat_3v1_1(1, 0) = gnss_info.y_odom - vec_bel(1, 0);
      mat_3v1_1(2, 0) =
          common::NormalizeAngle(gnss_info.heading_odom - vec_bel(2, 0));
      common::Mat_Mul(mat_k, mat_3v1_1, mat_3v1_2);
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "mat_3v1_1={" << mat_3v1_1(0, 0)
                << ", " << mat_3v1_1(1, 0)
                << ", " << common::com_rad2deg(mat_3v1_1(2, 0))
                << std::endl;
      std::cout << "mat_3v1_2={" << mat_3v1_2(0, 0)
                << ", " << mat_3v1_2(1, 0)
                << ", " << common::com_rad2deg(mat_3v1_2(2, 0))
                << std::endl;
#endif
      filtered_x_odom = vec_bel(0, 0) + mat_3v1_2(0, 0);
      filtered_y_odom = vec_bel(1, 0) + mat_3v1_2(1, 0);
      filtered_heading_odom =
          common::NormalizeAngle(vec_bel(2, 0) + mat_3v1_2(2, 0));
#if ENABLE_POS_FILTER_IMPL_TRACE
      std::cout << "After filter: x=" << filtered_x_odom
                << ", y=" << filtered_y_odom
                << ", heading=" << common::com_rad2deg(filtered_heading_odom)
                << std::endl;
#endif
      if (common::com_abs(
            common::AngleDiff(filtered_heading_odom, est_heading)) >
          common::com_deg2rad(5.0)) {
        LOG_ERR << "Invalid filtered heading.";
#if ENABLE_POS_FILTER_IMPL_TRACE
        std::cout << "mat_covariance=\n" << mat_covariance << std::endl;
        std::cout << "mat_k=\n" << mat_k << std::endl;
#endif
      }

      mat_3v3_1.SetIdentity();
      common::Mat_Sub(mat_3v3_1, mat_k, mat_3v3_2);
      common::Mat_Mul(mat_3v3_2, mat_covariance, odom_filter_.covariance);
      if (gnss_filter_.covariance(0, 0) < 0.1) {
        gnss_filter_.covariance(0, 0) = 0.1;
      }
      if (gnss_filter_.covariance(1, 1) < 0.1) {
        gnss_filter_.covariance(1, 1) = 0.1;
      }
      if (gnss_filter_.covariance(2, 2) < 0.1) {
        gnss_filter_.covariance(2, 2) = 0.1;
      }
    }
  } else {
    filtered_odom_status = gnss_info.odom_status;
    filtered_x_odom = gnss_info.x_odom;
    filtered_y_odom = gnss_info.y_odom;
    filtered_heading_odom = gnss_info.heading_odom;
    odom_filter_.Clear();
  }


  /// Saving
  filtered_gnss_info_ = gnss_info;

  filtered_gnss_info_.gnss_status = filtered_gnss_status;
  filtered_gnss_info_.latitude = filtered_latitude;
  filtered_gnss_info_.longitude = filtered_longitude;
  filtered_gnss_info_.heading_gnss = filtered_heading_gnss;

  filtered_gnss_info_.utm_status = filtered_utm_status;
  filtered_gnss_info_.x_utm = filtered_x_utm;
  filtered_gnss_info_.y_utm = filtered_y_utm;
  filtered_gnss_info_.heading_utm = filtered_heading_utm;

  filtered_gnss_info_.odom_status = filtered_odom_status;
  filtered_gnss_info_.x_odom = filtered_x_odom;
  filtered_gnss_info_.y_odom = filtered_y_odom;
  filtered_gnss_info_.heading_odom = filtered_heading_odom;

#if ENABLE_POS_FILTER_IMPL_TRACE
  std::cout << "### FilterGnss (End) ###" << std::endl;
#endif

  return true;
}


}  // namespace pos_filter
}  // namespace phoenix

