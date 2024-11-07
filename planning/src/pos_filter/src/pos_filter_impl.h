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
#ifndef PHOENIX_POS_FILTER_POS_FILTER_IMPL_H_
#define PHOENIX_POS_FILTER_POS_FILTER_IMPL_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "math/matrix.h"
#include "container/static_vector.h"
#include "container/ring_buffer.h"

#include "vehicle_model_wrapper.h"
#include "ad_msg.h"
#include "pos_filter.h"
#include "yaw_rate_filter_kalman.h"
#include "lane_mark_camera_filter.h"


namespace phoenix {
namespace pos_filter {


class PosFilterImpl {
public:
  typedef Float32_t Scalar;

public:
  PosFilterImpl();
  ~PosFilterImpl();

  void Configurate(const PosFilterConfig& conf);

  void Reset();

  /* k004 pengc 2022-12-26 (begin) */
  // 使用相机车道线矫正地图定位
  bool CorrectGnssCoordinate(Int32_t type, const Float32_t delta_pos[3]);
  /* k004 pengc 2022-12-26 (end) */

  bool UpdateOdomCoordinate(
      const common::Matrix<Float64_t, 3, 3>& mat_conv, Float32_t delta_heading);

  bool Update(const PosFilterDataSource& data_source);

  const ad_msg::LaneInfoCameraList& GetLaneInfoCameraList() const {
    return (lane_mark_camera_filter_.GetLaneInfoCameraList());
  }

  void GetRelativePosList(ad_msg::RelativePosList* pos_list) const;

  void GetGnssTrackList(ad_msg::RelativePosList* track_list) const;

  void GetGnssFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  void GetUtmTrackList(ad_msg::RelativePosList* track_list) const;

  void GetUtmFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  void GetOdomTrackList(ad_msg::RelativePosList* track_list) const;

  void GetOdomFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  const ad_msg::Gnss& GetFilteredGnssInfo() const {
    return (filtered_gnss_info_);
  }

  void GetPosFilterInfo(PosFilterInfo* info) const;

  static bool FindRelPosFromListByTimestamp(
      Int64_t timestamp, const ad_msg::RelativePosList& pos_list,
      ad_msg::RelativePos* pos);

private:
  enum {
    LOCALIZATION_TYPE_GNSS = 0,
    LOCALIZATION_TYPE_UTM,
    LOCALIZATION_TYPE_ODOM
  };

  struct GnssPoint {
    Float64_t latitude;
    Float64_t longitude;
    Float32_t heading;
  };

  struct UtmPoint {
    Float64_t x;
    Float64_t y;
    Float32_t heading;
  };

  struct OdomPoint {
    Float64_t x;
    Float64_t y;
    Float32_t heading;
  };

  struct FilteredPos {
    Int32_t relative_time;

    Scalar x;
    Scalar y;
    Scalar heading;
    Scalar yaw_rate;
    Scalar yaw_rate_chg_rate;
    Scalar v;

    Scalar delta_x;
    Scalar delta_y;
    Scalar delta_heading;

    void Clear() {
      relative_time = 0;

      x = 0.0F;
      y = 0.0F;
      heading = 0.0F;
      yaw_rate = 0.0F;
      yaw_rate_chg_rate = 0.0F;
      v = 0.0F;

      delta_x = 0.0F;
      delta_y = 0.0F;
      delta_heading = 0.0F;
    }

    FilteredPos() {
      Clear();
    }
  };

  struct FilterRet {
    Int64_t time_start;
    Int64_t time_end;
    FilteredPos estimated_pos_start;
    FilteredPos estimated_pos_end;
    Scalar corrected_delta_pos[3];

    Scalar corrected_velocity;
    Scalar corrected_yaw_rate;

    Scalar belief;

    void Clear() {
      time_start = 0;
      time_end = 0;
      estimated_pos_start.Clear();
      estimated_pos_end.Clear();
      corrected_delta_pos[0] = 0.0F;
      corrected_delta_pos[1] = 0.0F;
      corrected_delta_pos[2] = 0.0F;

      corrected_velocity = 0.0F;
      corrected_yaw_rate = 0.0F;

      belief = 0.0F;
    }

    FilterRet() {
      Clear();
    }
  };

  struct StateNoise {
    Scalar x_noise;
    Scalar y_noise;
    Scalar heading_noise;

    void Clear() {
      x_noise = 0.0F;
      y_noise = 0.0F;
      heading_noise = 0.0F;
    }

    StateNoise() {
      Clear();
    }
  };

private:
  bool EstimatePosByLaneMarkCameraList(
      Scalar velocity, Scalar yaw_rate,
      const ad_msg::LaneMarkCameraList& lane_mark_camera_list,
      FilterRet* filter_ret);

  bool EstimatePosByGnss(
      Scalar velocity, Scalar yaw_rate,
      Int32_t localization_type, const ad_msg::Gnss& gnss_info,
      FilterRet* filter_ret);

  bool CalcBeliefBasedOnVelocity(
      Scalar v, Scalar yaw_rate,
      Scalar estimated_v, Scalar estimated_yaw_rate,
      Scalar* bel);

  void PredictPosByVehStatus(FilterRet* filter_ret);

  void FilterPosByLaneMark(
      const FilterRet& lane_mark_filter_ret,
      FilterRet* filter_ret);

  void FilterPosByLaneMarkAndGnss(
      const FilterRet& lane_mark_filter_ret,
      const FilterRet& gnss_filter_ret,
      FilterRet* filter_ret);

  void FilterPosByGnss(
      const FilterRet& gnss_filter_ret,
      FilterRet* filter_ret);

  void GetStateTransitionNoise(Scalar v, Scalar yaw_rate, StateNoise* noise);
  void GetMeasurementNoiseOfLaneMark(
      Scalar belief, Scalar heading, StateNoise* noise);
  void GetMeasurementNoiseOfGnss(
      Scalar belief, Scalar heading, StateNoise* noise);

  bool FindPosFromListByTimestamp(
      Int64_t relative_time, FilteredPos* pos) const;

  void SmoothPosBtwTimestampsInList(
      Int32_t start_rel_time,
      Int32_t end_rel_time,
      const FilteredPos& start_pos,
      const Scalar corrected_delta_pos[3]);

  Scalar CalcPosProbBasedOnVelocity(
      Scalar v, Scalar yaw_rate, Scalar delta_t,
      const Scalar pos_start[3], const Scalar pos_end[3]);
  Scalar CalcProbOfDistribution(Scalar a, Scalar b);

  bool FilterLaneMark(
      Int64_t curr_time_stamp,
      const ad_msg::LaneMarkCameraList& lane_mark_camera_list,
      bool lane_mark_valid_bel, const FilterRet& lane_mark_filter_ret);
  bool FilterGnss(
      Int64_t curr_time_stamp,
      const ad_msg::Gnss& gnss_info,
      bool gnss_valid_bel);

private:
  veh_model::VehicleModelWrapper vehicle_model_;
  YawRateFilterKalman yaw_rate_filter_;

  Scalar curr_velocity_;
  Scalar curr_yaw_rate_;
  Int64_t pos_filter_timeinterval_;

  bool prev_lane_mark_valid_;
  Uint32_t prev_lane_mark_msg_sequence_;
  Int64_t prev_lane_mark_timestamp_;
  LaneMarkCameraFilter lane_mark_camera_filter_;

  bool prev_gnss_valid_;
  Int32_t localization_status_;
  Int32_t prev_localization_status_;
  ad_msg::Gnss prev_gnss_info_;

  bool prev_filtered_pos_valid_;
  Int64_t prev_filtered_pos_timestamp_;
  common::Matrix<Scalar, 3, 3> mat_covariance_;

  enum { MAX_FILTERED_POS_LIST_SIZE = 40 };
  common::StaticVector<FilteredPos,
      MAX_FILTERED_POS_LIST_SIZE> filtered_pos_list_;
  ad_msg::MsgHead filtered_pos_list_msg_head_;

  struct GnssFilter {
    common::Matrix<Float64_t, 3, 3> covariance;

    void Clear() {
      covariance.SetIdentity();
    }

    GnssFilter() {
      Clear();
    }
  } gnss_filter_;

  struct UtmFilter {
    common::Matrix<Float64_t, 3, 3> covariance;

    void Clear() {
      covariance.SetIdentity();
    }

    UtmFilter() {
      Clear();
    }
  } utm_filter_;

  struct OdomFilter {
    common::Matrix<Float64_t, 3, 3> covariance;

    void Clear() {
      covariance.SetIdentity();
    }

    OdomFilter() {
      Clear();
    }
  } odom_filter_;

  ad_msg::Gnss filtered_gnss_info_;

  enum { MAX_POS_TRACK_LIST_SIZE = 40 };
  common::RingBuffer<GnssPoint, MAX_POS_TRACK_LIST_SIZE>
      gnss_track_list_;
  common::RingBuffer<GnssPoint, MAX_POS_TRACK_LIST_SIZE>
      gnss_filtered_track_list_;
  common::RingBuffer<UtmPoint, MAX_POS_TRACK_LIST_SIZE>
      utm_track_list_;
  common::RingBuffer<UtmPoint, MAX_POS_TRACK_LIST_SIZE>
      utm_filtered_track_list_;
  common::RingBuffer<OdomPoint, MAX_POS_TRACK_LIST_SIZE>
      odom_track_list_;
  common::RingBuffer<OdomPoint, MAX_POS_TRACK_LIST_SIZE>
      odom_filtered_track_list_;
};


}  // namespace pos_filter
}  // namespace phoenix


#endif // PHOENIX_POS_FILTER_POS_FILTER_IMPL_H_
