//
#ifndef PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_H_
#define PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_H_

#include "pos_filter.h"
#include "math/matrix.h"


namespace phoenix {
namespace pos_filter {


class PosFilterImpl;

class PosFilterWrapper {
public:
  typedef Float32_t Scalar;

public:
  PosFilterWrapper();
  ~PosFilterWrapper();

  void Configurate(const PosFilterConfig& conf);

  /* k004 pengc 2022-12-26 (begin) */
  // 使用相机车道线矫正地图定位
  bool CorrectGnssCoordinate(Int32_t type, const Float32_t delta_pos[3]);
  /* k004 pengc 2022-12-26 (end) */

  bool UpdateOdomCoordinate(
      const common::Matrix<Float64_t, 3, 3>& mat_conv, Float32_t delta_heading);

  bool Update(const PosFilterDataSource& data_source);

  const ad_msg::LaneInfoCameraList& GetLaneInfoCameraList() const;

  void GetRelativePosList(ad_msg::RelativePosList* pos_list) const;

  void GetGnssTrackList(ad_msg::RelativePosList* track_list) const;

  void GetGnssFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  void GetUtmTrackList(ad_msg::RelativePosList* track_list) const;

  void GetUtmFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  void GetOdomTrackList(ad_msg::RelativePosList* track_list) const;

  void GetOdomFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  const ad_msg::Gnss& GetFilteredGnssInfo() const;

  void GetPosFilterInfo(PosFilterInfo* info) const;

  static bool FindRelPosFromListByTimestamp(
      Int64_t timestamp, const ad_msg::RelativePosList& pos_list,
      ad_msg::RelativePos* pos);

private:
  PosFilterImpl* pos_filter_impl_;
};


}  // namespace pos_filter
}  // namespace phoenix


#endif // PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_H_
