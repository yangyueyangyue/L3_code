/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       driving_map_wrapper.cc
 * @brief      驾驶地图接口定义
 * @details    定义了驾驶地图对外部模块的接口
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "driving_map_wrapper.h"
#include "driving_map/driving_map_impl.h"


namespace phoenix {
namespace driv_map {

static DrivingMapImpl s_driving_map_impl_array[MAX_DRIVING_MAP_IMPL_NUM];


Uint32_t GetDrivingMapImplSize() {
  return (sizeof(DrivingMapImpl));
}

void* GetDrivingMapImpl(Int32_t index) {
  if ((0 <= index) && (index < MAX_DRIVING_MAP_IMPL_NUM)) {
    return (&s_driving_map_impl_array[index]);
  }

  return Nullptr_t;
}

void CopyDrivingMapImpl(const void * const from, void * const to) {
  common::com_memcpy(to, from, sizeof(DrivingMapImpl));
}



DrivingMapWrapper::DrivingMapWrapper() {
  driving_map_ = Nullptr_t;

  std::cout << "sizeof(DrivingMapImpl)=" << sizeof(DrivingMapImpl)
            << " bytes (" << sizeof(DrivingMapImpl) / 1024.0F / 1024.0F
            << " M)"
            << std::endl;
}

DrivingMapWrapper::~DrivingMapWrapper() {
}

void* DrivingMapWrapper::GetDrivingMapImpl() {
  return (driving_map_);
}

void DrivingMapWrapper::SetDrivingMapImpl(void* instance_addr) {
  COM_CHECK(Nullptr_t != instance_addr);

  driving_map_ = static_cast<DrivingMapImpl*>(instance_addr);
}

void DrivingMapWrapper::Configurate(const DrivingMapConfig& conf) {
  driving_map_->Configurate(conf);
}

bool DrivingMapWrapper::Update(const DrivingMapDataSource& data_source) {
  return (driving_map_->Update(data_source));
}

bool DrivingMapWrapper::UpdateObstacleList(
    const ad_msg::ObstacleList* obj_list) {
  return (driving_map_->UpdateObstacleList(obj_list));
}

/* k004 pengc 2022-12-26 (begin) */
// 使用相机车道线矫正地图定位
const map_var_t* DrivingMapWrapper::GetDeltaPosOfCorrectedGnss() const {
  return (driving_map_->GetDeltaPosOfCorrectedGnss());
}
/* k004 pengc 2022-12-26 (end) */

Int32_t DrivingMapWrapper::GetDrivingMapType() const {
  return (driving_map_->GetDrivingMapType());
}

const ad_msg::Gnss& DrivingMapWrapper::GetGnss() const {
  return (driving_map_->GetGnss());
}

const ad_msg::RelativePosList& DrivingMapWrapper::GetRelativePosList() const {
  return (driving_map_->GetRelativePosList());
}

const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
DrivingMapWrapper::GetPredictedPathOfVehicle() const {
  return (driving_map_->GetPredictedPathOfVehicle());
}


/* --- 地图相关信息 --- */
bool DrivingMapWrapper::IsValidLaneIndex(Int32_t lane_index) const {
  return (driving_map_->IsValidLaneIndex(lane_index));
}

Int32_t DrivingMapWrapper::GetLaneIndexByID(
    const ID& lane_id) const {
  return driving_map_->GetLaneIndexByID(lane_id);
}

const ID& DrivingMapWrapper::GetLaneIdByIndex(Int32_t lane_index) const {
  return (driving_map_->GetLaneIdByIndex(lane_index));
}

const common::Path& DrivingMapWrapper::GetLaneCentralCurve(
    const Int32_t lane_index) const {
  return (driving_map_->GetLaneCentralCurve(lane_index));
}

Float32_t DrivingMapWrapper::GetLaneSpeedLimitHigh(
    const Int32_t lane_index) const {
   return (driving_map_->GetLaneSpeedLimitHigh(lane_index));
}

Float32_t DrivingMapWrapper::GetLaneSpeedLimitLow(
    const Int32_t lane_index) const {
   return (driving_map_->GetLaneSpeedLimitLow(lane_index));
}

void DrivingMapWrapper::GetLaneWidth(
    const Int32_t lane_index, const Float32_t s,
    Float32_t * const left_width, Float32_t * const right_width) const {
  driving_map_->GetLaneWidth(lane_index, s, left_width, right_width);
}

/* k003 longjiaoy 2022-11-28 (start) */
void DrivingMapWrapper::GetLaneBoundaryType(
    const Int32_t lane_index, const map_var_t s,
    Int32_t* const left_type, Int32_t* const right_type) const {
  driving_map_->GetLaneBoundaryType(lane_index, s, left_type, right_type);
}
/* k003 longjiaoy 2022-11-28 (end) */

Float32_t DrivingMapWrapper::GetLaneSlope(
    const Int32_t lane_index,
    const Float32_t s) const {
  return driving_map_->GetLaneSlope(lane_index, s);  // 获取车道坡度
}

bool DrivingMapWrapper::FindNearestLane(
    const common::Vec2d& point,
    Int32_t* nearest_lane_index,
    common::PathPoint* nearest_point_on_lane) const {
  return  driving_map_->FindNearestLane(
        point, nearest_lane_index, nearest_point_on_lane);
}

bool DrivingMapWrapper::FindNearestLane(
    const common::Vec2d& point,
    const Float32_t heading,
    Int32_t * const nearest_lane_index,
    common::PathPoint * const nearest_point_on_lane) const {
  return  driving_map_->FindNearestLane(
        point, heading, nearest_lane_index, nearest_point_on_lane);
}

bool DrivingMapWrapper::FindNearestLaneSynthetically(
    const common::Vec2d& point,
    Float32_t heading,
    Int32_t * const nearest_lane_index,
    common::PathPoint * const nearest_point_on_lane) const {
  return  driving_map_->FindNearestLaneSynthetically(
        point, heading, nearest_lane_index, nearest_point_on_lane);
}

bool DrivingMapWrapper::FindNeighbors(
    const common::Vec2d& point,
    const Int32_t lane_index,
    common::StaticVector<NeighborsLaneInfo,
    MAX_NEIGHBOR_LANE_NUM> * const sorted_neighbor_lanes,
    Int32_t * const nearest_neighbor_index) const {
  return  driving_map_->FindNeighbors(
        point, lane_index, sorted_neighbor_lanes, nearest_neighbor_index);
}

const common::StaticVector<RoutingSection, MAX_ROUTING_SECTION_NUM>&
DrivingMapWrapper::GetRoutingSections() const {
  return (driving_map_->GetRoutingSections());
}

bool DrivingMapWrapper::IsOverlappedWithRouting(
    const Int32_t lane_index,
    const Float32_t start_s,
    const Float32_t end_s) const {
  return driving_map_->IsOverlappedWithRoutingSegment(
        lane_index, start_s, end_s);
}

bool DrivingMapWrapper::IsOnRoutingSegment(const Int32_t lane_index,
                                           Float32_t s) const {
  return driving_map_->IsOnRoutingSegment(lane_index, s);
}

void DrivingMapWrapper::GetNearestLaneToCurrPosition(
    Int32_t* const nearest_lane_index,
    common::PathPoint* const nearest_point_on_lane,
    Int32_t* const nearest_idx_in_neighbor,
    common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM>*
    const neighbor_lanes) const {
  return driving_map_->GetNearestLaneToCurrPosition(
        nearest_lane_index, nearest_point_on_lane,
        nearest_idx_in_neighbor, neighbor_lanes);
}


/* --- 参考线相关信息 --- */
Int32_t DrivingMapWrapper::GetReferenceLinesNum() const {
  return driving_map_->GetReferenceLinesNum();
}

bool DrivingMapWrapper::IsValidReferenceLineIndex(Int32_t index) const {
  return (driving_map_->IsValidReferenceLineIndex(index));
}


Int32_t DrivingMapWrapper::GetMajorReferenceLineIndex() const {
  return driving_map_->GetMajorRefLineIndex();
}

const common::PathPoint& DrivingMapWrapper::GetProjPointOnMajorRefLine() const {
  return (driving_map_->GetProjPointOnMajorRefLine());
}

const common::Path& DrivingMapWrapper::GetMajorReferenceLine() const {
  return (driving_map_->GetMajorReferenceLine());
}

const common::Path& DrivingMapWrapper::GetMajorSmoothReferenceLine() const {
  return (driving_map_->GetMajorSmoothReferenceLine());
}

Int32_t DrivingMapWrapper::GetReferenceLineNeighborFlag(
    Int32_t ref_line_index) const {
  return (driving_map_->GetReferenceLineNeighborFlag(ref_line_index));
}

Int32_t DrivingMapWrapper::GetReferenceLineLaneQuality(
    Int32_t ref_line_index) const {
  return (driving_map_->GetReferenceLineLaneQuality(ref_line_index));
}

const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
DrivingMapWrapper::GetReferenceLaneSegments(
    const Int32_t ref_line_index) const {
  return (driving_map_->GetReferenceLineLaneSegments(ref_line_index));
}

map_var_t DrivingMapWrapper::GetProjDistOfCurrPositionOnRef(
    Int32_t index) const {
  return (driving_map_->GetProjDistOfCurrPositionOnRef(index));
}

const common::Path& DrivingMapWrapper::GetReferenceLine(
    const Int32_t ref_line_index) const {
  return (driving_map_->GetReferenceLineCurve(ref_line_index));
}

const common::Path& DrivingMapWrapper::GetSmoothReferenceLine(
    const Int32_t ref_line_index) const {
  return (driving_map_->GetReferenceLineSmoothCurve(ref_line_index));
}

const common::StaticVector<common::Path::CurveSegment,
common::Path::kMaxCurveSegmentNum>&
DrivingMapWrapper::GetReferenceLineCurveInfo(
    const Int32_t ref_line_index) const {
  return (driving_map_->GetReferenceLineCurveInfo(ref_line_index));
}

const common::StaticVector<common::Path::CurveSegment,
common::Path::kMaxCurveSegmentNum>&
DrivingMapWrapper::GetSmoothReferenceLineCurveInfo(
    const Int32_t ref_line_index) const {
  return (driving_map_->GetSmoothReferenceLineCurveInfo(ref_line_index));
}

void DrivingMapWrapper::GetReferenceLineContinuousSegment(
    Int32_t index, map_var_t* start_s, map_var_t* end_s) const {
  driving_map_->GetReferenceLineContinuousSegment(index, start_s, end_s);
}

Int32_t DrivingMapWrapper::FindReferenceLine(
    const common::Vec2d& start_point,
    const Int32_t start_lane_index,
    const Int32_t end_lane_index) const {
  return driving_map_->FindReferenceLine(
        start_point, start_lane_index, end_lane_index);
}

Int32_t DrivingMapWrapper::FindReferenceLine(
    const Int32_t start_lane_index,
    const Int32_t end_lane_index) const {
  return driving_map_->FindReferenceLine(start_lane_index, end_lane_index);
}

Int32_t DrivingMapWrapper::FindReferenceLineLaneSegmentByProjOnRef(
    Int32_t ref_line_index,
    map_var_t s_on_ref_line,
    Int32_t * const lane_index,
    Float32_t* const s_on_lane) const {
  return (driving_map_->FindReferenceLineLaneSegmentByProjOnRef(
            ref_line_index, s_on_ref_line, lane_index, s_on_lane));
}

const common::StaticVector<RoadBoundary, common::Path::kMaxPathPointNum>&
DrivingMapWrapper::GetRoadBoundary() const {
  return (driving_map_->GetRoadBoundary());
}

Int32_t DrivingMapWrapper::TestCollisionWithRoadBoundary(
    const CollisionTestObj& obj, CollisionTestResult* const result) const {
  return (driving_map_->TestCollisionWithRoadBoundary(obj, result));
}

const common::StaticVector<MapTrafficLight, MAX_MAP_TRAFFIC_LIGHT_NUM>&
DrivingMapWrapper::GetMapTrafficLightTable() const {
  return (driving_map_->GetMapTrafficLightTable());
}


/* --- 障碍物相关信息 --- */
bool DrivingMapWrapper::IsValidObstacleIndex(Int32_t obj_index) const {
  return (driving_map_->IsValidObstacleIndex(obj_index));
}

const ad_msg::Obstacle& DrivingMapWrapper::GetObstacle(
    const Int32_t obj_index) const {
  return (driving_map_->GetObstacle(obj_index));
}

const ad_msg::ObstacleList& DrivingMapWrapper::GetObstacleList() const {
  return (driving_map_->GetObstacleList());
}

bool DrivingMapWrapper::GetRiskyObstacles(
    Int32_t ref_line_index, bool return_uncertain,
    CollisionTestResult* const result) const {
  return (driving_map_->GetRiskyObstacles(
            ref_line_index, return_uncertain, result));
}

Int32_t DrivingMapWrapper::TestCollision(
    const CollisionTestParam& param,
    const CollisionTestObj& obj,
    CollisionTestResult* const result) const {
  return driving_map_->TestCollision(param, obj, result);
}

Int32_t DrivingMapWrapper::TestCollisionOnPath(
    const CollisionTestOnPathObj& obj,
    const common::Path& path,
    common::StaticVector<common::PathPoint,
    common::Path::kMaxPathPointNum> path_sample_points,
    CollisionTestOnPathResult* const result) const {
  return driving_map_->TestCollisionOnPath(
        obj, path, path_sample_points, result);
}


/**
 * @brief 获取场景任务列表
 * @return 场景任务列表
 */
const common::StaticVector<ad_msg::SceneStory,
    ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>&
DrivingMapWrapper::GetSceneStoryList() const {
  return driving_map_->GetSceneStoryList();
}
const ad_msg::PlanningStoryList&
DrivingMapWrapper::GetPlanningStoryList() const {
  return driving_map_->GetPlanningStoryList();
}


/* --- 事件通知 --- */
Int32_t DrivingMapWrapper::GetEventReporting(
    Int32_t num, ad_msg::EventReporting* const events) const {
  return (driving_map_->GetEventReporting(num, events));
}


/* --- 调试信息 --- */
void DrivingMapWrapper::GetDrivingMapInfo(
    DrivingMapInfo* const driving_map_info) const {
  driving_map_->GetDrivingMapInfo(driving_map_info);
}


}  // namespace driv_map
}  // namespace phoenix
