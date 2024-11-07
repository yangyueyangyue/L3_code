//
#include "pos_filter_wrapper.h"
#include "pos_filter_impl.h"

namespace phoenix {
namespace pos_filter {


PosFilterImpl g_pos_filter_impl;


PosFilterWrapper::PosFilterWrapper() {
  pos_filter_impl_ = &g_pos_filter_impl;
}

PosFilterWrapper::~PosFilterWrapper() {
  // nothing to do
}

void PosFilterWrapper::Configurate(const PosFilterConfig& conf) {
  pos_filter_impl_->Configurate(conf);
}

/* k004 pengc 2022-12-26 (begin) */
// 使用相机车道线矫正地图定位
bool PosFilterWrapper::CorrectGnssCoordinate(
    Int32_t type, const Float32_t delta_pos[3]) {
  return pos_filter_impl_->CorrectGnssCoordinate(type, delta_pos);
}

/* k004 pengc 2022-12-26 (end) */

bool PosFilterWrapper::UpdateOdomCoordinate(
    const common::Matrix<Float64_t, 3, 3>& mat_conv, Float32_t delta_heading) {
  return pos_filter_impl_->UpdateOdomCoordinate(mat_conv, delta_heading);
}

bool PosFilterWrapper::Update(
    const PosFilterDataSource& data_source) {
  return pos_filter_impl_->Update(data_source);
}

const ad_msg::LaneInfoCameraList&
PosFilterWrapper::GetLaneInfoCameraList() const {
  return (pos_filter_impl_->GetLaneInfoCameraList());
}

void PosFilterWrapper::GetRelativePosList(
    ad_msg::RelativePosList* pos_list) const {
  return (pos_filter_impl_->GetRelativePosList(pos_list));
}

void PosFilterWrapper::GetGnssTrackList(
    ad_msg::RelativePosList* track_list) const {
  return (pos_filter_impl_->GetGnssTrackList(track_list));
}

void PosFilterWrapper::GetGnssFilteredTrackList(
    ad_msg::RelativePosList* track_list) const {
  return (pos_filter_impl_->GetGnssFilteredTrackList(track_list));
}

void PosFilterWrapper::GetUtmTrackList(
    ad_msg::RelativePosList* track_list) const {
  return (pos_filter_impl_->GetUtmTrackList(track_list));
}

void PosFilterWrapper::GetUtmFilteredTrackList(
    ad_msg::RelativePosList* track_list) const {
  return (pos_filter_impl_->GetUtmFilteredTrackList(track_list));
}

void PosFilterWrapper::GetOdomTrackList(
    ad_msg::RelativePosList* track_list) const {
  return (pos_filter_impl_->GetOdomTrackList(track_list));
}

void PosFilterWrapper::GetOdomFilteredTrackList(
    ad_msg::RelativePosList* track_list) const {
  return (pos_filter_impl_->GetOdomFilteredTrackList(track_list));
}

const ad_msg::Gnss& PosFilterWrapper::GetFilteredGnssInfo() const {
  return (pos_filter_impl_->GetFilteredGnssInfo());
}

void PosFilterWrapper::GetPosFilterInfo(PosFilterInfo* info) const {
  return (pos_filter_impl_->GetPosFilterInfo(info));
}

bool PosFilterWrapper::FindRelPosFromListByTimestamp(
    Int64_t timestamp, const ad_msg::RelativePosList& pos_list,
    ad_msg::RelativePos* pos) {
  return (PosFilterImpl::FindRelPosFromListByTimestamp(
            timestamp, pos_list, pos));
}


}  // namespace pos_filter
}  // namespace phoenix


