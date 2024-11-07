//
#ifndef PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_H_
#define PHOENIX_POS_FILTER_POS_FILTER_WRAPPER_H_

#include "pos_filter.h"


namespace phoenix {
namespace pos_filter {

class PosFilterImpl;

class PosFilterWrapper {
public:
  typedef Float32_t Scalar;

public:
  PosFilterWrapper();
  ~PosFilterWrapper();

  bool Update(const PosFilterDataSource& data_source);

  const ad_msg::LaneInfoCameraList& GetLaneInfoCameraList() const;

  void GetRelativePosList(ad_msg::RelativePosList* pos_list) const;

  void GetGnssTrackList(ad_msg::RelativePosList* track_list) const;

  void GetGnssFilteredTrackList(ad_msg::RelativePosList* track_list) const;

  void GetUtmTrackList(ad_msg::RelativePosList* track_list) const;

  void GetUtmFilteredTrackList(ad_msg::RelativePosList* track_list) const;

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
