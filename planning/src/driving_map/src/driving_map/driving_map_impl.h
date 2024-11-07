/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       driving_map_impl.h
 * @brief      驾驶地图对外接口的实现类
 * @details    驾驶地图对外接口的具体实现，主要是通过调用内部成员变量的对应接口。
 *
 * @author     boc
 * @date       2020.06.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/19  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_DRIVING_MAP_DRIVING_MAP_IMPL_H_
#define PHOENIX_DRIVING_MAP_DRIVING_MAP_IMPL_H_

#include "utils/macros.h"
#include "utils/log.h"
#include "utils/linear_interpolation.h"
#include "map_space/hd_map.h"
#include "ref_line/reference_line_set.h"
#include "obj_space/object_map_impl.h"
#include "scene_story/scene_story_set.h"
#include "planning_story/planning_story_set.h"


#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO) || \
    (HD_MAP_TYPE == HD_MAP_TYPE_D17)
#define ENABLE_MIXED_MAP_SOURCE (1)
#else
#define ENABLE_MIXED_MAP_SOURCE (0)
#endif


namespace phoenix {
namespace driv_map {


/**
 * @class DrivingMapImpl
 * @brief 驾驶地图对外接口的实现类
 */
class DrivingMapImpl {
public:
  /**
   * @brief 构造函数
   */
  DrivingMapImpl();
  /**
   * @brief 析构函数
   */
  ~DrivingMapImpl();

  /**
   * @brief 配置模块
   */
  void Configurate(const DrivingMapConfig& conf);

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 从外部源数据更新内部驾驶地图
   * @param[in] data_source 外部源数据
   * @return true-成功, false-失败
   */
  bool Update(const DrivingMapDataSource& data_source);

  /**
   * @brief 更新障碍物列表
   * @param[in] obj_list 障碍物列表
   * @return true-成功, false-失败
   */
  bool UpdateObstacleList(const ad_msg::ObstacleList* obj_list);

  /* k004 pengc 2022-12-26 (begin) */
  // 使用相机车道线矫正地图定位
  /**
   * @brief 获取矫正后的地图定位的增量
   * @return 矫正后的地图定位的增量
   */
  const map_var_t* GetDeltaPosOfCorrectedGnss() const {
    return (delta_pos_corrected_gnss_);
  }
  /* k004 pengc 2022-12-26 (end) */

  /**
   * @brief 获取地图数据的时间戳 & 序列号
   * @return 地图数据的时间戳 & 序列号
   */
  inline const ad_msg::MsgHead& GetHDMapMsgHead() const {
    return (map_space_.GetHDMapMsgHead());
  }

  /**
   * @brief 获取驾驶地图的类型
   * @return 驾驶地图的类型
   */
  inline Int32_t GetDrivingMapType() const {
    return (driving_map_type_);
  }

  /**
   * @brief 获取根据当前车辆状态预测的轨迹
   * @return 轨迹
   */
  inline const common::StaticVector<common::Vec2d,
  common::Path::kMaxPathPointNum>& GetPredictedPathOfVehicle() const {
    return (sample_points_of_pred_path_);
  }

  /**
   * @brief 获取车道列表
   * @return 车道列表
   */
  inline const common::StaticVector<Lane, MAX_LANE_NUM>&
      GetLaneTable() const {
    return map_space_.GetLaneTable();
  }

  /**
   * @brief 获取更新驾驶地图时的全局位置信息
   * @return 更新驾驶地图时的全局位置信息
   */
  inline const ad_msg::Gnss& GetGnss() const { return (gnss_); }

  /**
   * @brief 获取驾驶地图构建时的位置信息
   * @return 驾驶地图构建时的位置信息
   */
  inline const ad_msg::RelativePosList& GetRelativePosList() const {
    return (map_space_.GetRelPosList());
  }

  /**
   * @brief 获取最近的车道
   * @param[in]  point 待查询点的位置
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] nearest_point      最近车道上与待查询点最近的点
   * @return true-获取成功, false-获取失败
   */
  inline bool FindNearestLane(
      const common::Vec2d& point,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const nearest_point) const {
    return (map_space_.FindNearestLane(
              point, nearest_lane_index, nearest_point));
  }

  /**
   * @brief 获取与待查询点方向一致的最近的车道
   * @param[in]  point   待查询点的位置
   * @param[in]  heading 待查询点的航向角
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] nearest_point      最近车道上与待查询点最近的点
   * @return true-获取成功, false-获取失败
   */
  inline bool FindNearestLane(
      const common::Vec2d& point,
      Float32_t heading,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const nearest_point) const {
    return (map_space_.FindNearestLane(
              point, heading, nearest_lane_index, nearest_point));
  }

  /**
   * @brief 根据距离及航向综合查找最近的车道, 避免在分流口/合流口查找到不合适的最近车道
   * @param[in]  point   待查询点的位置
   * @param[in]  heading 待查询点的航向角
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] nearest_point      最近车道上与待查询点最近的点
   * @return true-获取成功, false-获取失败
   */
  inline bool FindNearestLaneSynthetically(
      const common::Vec2d& point,
      Float32_t heading,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const nearest_point) const {
    return (map_space_.FindNearestLaneSynthetically(
              point, heading, nearest_lane_index, nearest_point));
  }

  /**
   * @brief 获取待查询点的所有左右相邻车道
   * @param[in]  point   待查询点的位置
   * @param[in]  lane_index   与待查询点距离最近的车道的索引
   * @param[out] sorted_neighbor_lanes   待查询点的所有左右相邻车道列表
   * @param[out] nearest_neighbor_index  与待查询点距离最近的车道在所有左右相邻\n
   *                                     车道列表中的索引
   * @return true-获取成功, false-获取失败
   */
  inline bool FindNeighbors(
      const common::Vec2d& point,
      const Int32_t lane_index,
      common::StaticVector<NeighborsLaneInfo,
      MAX_NEIGHBOR_LANE_NUM> * const sorted_neighbor_lanes,
      Int32_t * const nearest_neighbor_index) const {
    return (map_space_.FindNeighbors(
              point, lane_index, sorted_neighbor_lanes,
              nearest_neighbor_index));
  }

  /**
   * @brief 判断车道索引是否有效
   * @param[in] lane_index 车道索引
   * @return true - 索引有效, false - 索引无效
   */
  inline bool IsValidLaneIndex(Int32_t lane_index) const {
    return (map_space_.IsValidLaneIndex(lane_index));
  }

  /**
   * @brief 根据车道ID获取车道索引
   * @param[in] lane_id 车道ID
   * @return 车道索引
   */
  inline Int32_t GetLaneIndexByID(
      const ID& lane_id) const {
    return (map_space_.GetLaneIndexById(lane_id));
  }

  /**
   * @brief 根据车道索引获取车道ID
   * @param[in]  index 车道索引
   * @param[out] id    车道ID
   * @return true-获取成功, false-获取失败
   */
  inline const ID& GetLaneIdByIndex(
      const Int32_t lane_index) const {
    return (map_space_.GetLaneIdByIndex(lane_index));
  }

  /**
   * @brief 获取导航路径上的车道段信息
   * @return 导航道路段列表
   */
  inline const common::StaticVector<RoutingSection, MAX_ROUTING_SECTION_NUM>&
  GetRoutingSections() const {
    return (map_space_.GetRoutingSections());
  }


  /**
   * @brief 判断指定车道的某一段与导航路径是否有重叠
   * @param[in] lane_index 车道的索引
   * @param[in] start_s    起点的路径长
   * @param[in] end_s      终点的路径长
   * @return true-有重叠；false-没有重叠。
   */
  inline bool IsOverlappedWithRoutingSegment(
      Int32_t lane_index, map_var_t start_s, map_var_t end_s) const {
    return (map_space_.IsOverlappedWithRoutingSegment(
              lane_index, start_s, end_s));
  }

  /**
   * @brief 判断指定车道的某一个点是否在导航路径上
   * @param[in] lane_index 车道的索引
   * @param[in] s          点的路径长
   * @return true-在导航路径上；false-不在导航路径上。
   */
  inline bool IsOnRoutingSegment(Int32_t lane_index, map_var_t s) const {
    return (map_space_.IsOnRoutingSegment(lane_index, s));
  }

  /**
   * @brief 获取车辆当前位置最近车道的车道索引和最近点
   * @param[out] nearest_lane_index       车辆当前位置最近车道的车道索引
   * @param[out] nearest_point_on_lane    最近车道上离车辆当前位置的最近点
   */
  inline void GetNearestLaneToCurrPosition(
      Int32_t* const nearest_lane_index,
      common::PathPoint* const nearest_point_on_lane,
      Int32_t* const nearest_idx_in_neighbor,
      common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM>*
      const neighbor_lanes) const {
    return reference_line_set_.GetNearestLaneToCurrPosition(
          nearest_lane_index, nearest_point_on_lane,
          nearest_idx_in_neighbor, neighbor_lanes);
  }

  /**
   * @brief 获取所有参考线的个数
   * @return 所有参考线的个数
   */
  inline Int32_t GetReferenceLinesNum() const {
    return (reference_line_set_.GetReferenceLinesNum());
  }

  /**
   * @brief 判断指定的参考线索引是否有效
   * @return true-指定的参考线索引有效；false-指定的参考线索引无效。
   */
  inline bool IsValidReferenceLineIndex(Int32_t index) const {
    return (reference_line_set_.IsValidReferenceLineIndex(index));
  }

  /**
   * @brief 获取参考线的左右相邻标记。离车辆最近的参考线的左右相邻标记为0，\n
   *        过右边的第i条车道的参考线为-i，过左边的第i条车道的参考线为i。
   * @return 参考线的左右相邻标记
   */
  inline Int32_t GetReferenceLineNeighborFlag(Int32_t index) const {
    return (reference_line_set_.GetNeighborFlag(index));
  }

  /**
   * @brief 获取序号为index参考线的车道线质量。
   * @return 参考线的车道线质量
   */
  inline Int32_t GetReferenceLineLaneQuality(Int32_t index) const {
    return (reference_line_set_.GetLaneQuality(index));
  }
  /**
   * @brief 获取参考线的车道段列表。
   * @param[in] index 参考线的索引
   * @return 参考线的车道段列表
   */
  inline const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
      GetReferenceLineLaneSegments(Int32_t index) const {
    return (reference_line_set_.GetLaneSegments(index));
  }

  /**
   * @brief 获取当前车辆位置在此参考线上的投影点相对与参考线起点的距离
   * @param[in] index 参考线的索引
   * @return 当前车辆位置在此参考线上的投影点相对与参考线起点的距离
   */
  inline map_var_t GetProjDistOfCurrPositionOnRef(Int32_t index) const {
    return (reference_line_set_.GetProjDistOfCurrPositionOnRef(index));
  }


  /**
   * @brief 获取车道中心线信息
   * @param[in] lane_index 车道索引
   * @return 车道中心线信息(包含形点、航向角、曲率等信息)
   */
  inline const common::Path& GetLaneCentralCurve(
      const Int32_t lane_index) const {
     return (map_space_.GetLane(lane_index).central_curve());
  }

  /**
   * @brief 获取车道限速(最高限速)
   * @param[in] lane_index 车道索引
   * @return 限速值
   */
  inline Float32_t GetLaneSpeedLimitHigh(const Int32_t lane_index) const {
     return (map_space_.GetLane(lane_index).speed_limit_high());
  }

  /**
   * @brief 获取车道限速(最低限速)
   * @param[in] lane_index 车道索引
   * @return 限速值
   */
  inline Float32_t GetLaneSpeedLimitLow(const Int32_t lane_index) const {
     return (map_space_.GetLane(lane_index).speed_limit_low());
  }

  /**
   * @brief 获取车道宽度
   * @param[in] lane_index 车道索引
   * @param[in] s 沿着车道中心线的长度
   * @param[out] left_width 此处车道中心线左边的宽度
   * @param[out] right_width 此处车道中心线右边的宽度
   */
  inline void GetLaneWidth(
      const Int32_t lane_index, const Float32_t s,
      Float32_t * const left_width, Float32_t * const right_width) const {
    map_space_.GetLane(lane_index).GetWidth(s, left_width, right_width);
  }

  /* k003 longjiaoy 2022-11-28 (start) */
  /**
   * @brief 获取车道边线的类型
   * @param[in] lane_index 车道索引
   * @param[in] s              沿着车道中心线的长度。
   * @param[out] left_type     左边界类型
   * @param[out] left_type     右边界类型
   */
  void GetLaneBoundaryType(
      const Int32_t lane_index, const map_var_t s,
      Int32_t* const left_type, Int32_t* const right_type) const {
    map_space_.GetLane(lane_index).GetBoundaryType(s, left_type, right_type);
  }
  /* k003 longjiaoy 2022-11-28 (end) */

  /**
   * @brief 获取车道坡度
   * @param[in] lane_index 车道索引
   * @param[in] s 沿着车道中心线的长度
   * @return 车道中此处位置的坡度
   */
  inline Float32_t GetLaneSlope(const Int32_t lane_index, Float32_t s) const {
    /// 根据车道索引查找到对应的车道
    const Lane &lane = map_space_.GetLane(lane_index);
    return lane.GetLaneSlope(s);
  }

  /**
   * @brief 获取主参考线的索引
   * @return 主参考线的索引
   */
  inline Int32_t GetMajorRefLineIndex() const {
    return (reference_line_set_.GetMajorRefLineIndex());
  }

  /**
   * @brief 获取车辆位置在主参考线上的投影点。
   * @return 车辆位置在主参考线上的投影点
   */
  inline const common::PathPoint& GetProjPointOnMajorRefLine() const {
    return (reference_line_set_.GetProjPointOnMajorRefLine());
  }

  /**
   * @brief 获取主参考线信息
   * @return 主参考线的信息（包含形点、航向角、曲率等）
   */
  inline const common::Path& GetMajorReferenceLine() const {
    const Int32_t index = GetMajorRefLineIndex();
    return (GetReferenceLineCurve(index));
  }

  /**
   * @brief 获取主参考线信息
   * @return 主参考线的信息（包含形点、航向角、曲率等）
   */
  inline const common::Path& GetMajorSmoothReferenceLine() const {
    const Int32_t index = GetMajorRefLineIndex();
    return (GetReferenceLineSmoothCurve(index));
  }

  /**
   * @brief 获取参考线信息
   * @param[in] ref_line_index 参考线索引
   * @return 参考线的信息（包含形点、航向角、曲率等）
   */
  inline const common::Path& GetReferenceLineCurve(Int32_t index) const {
    return (reference_line_set_.GetCurve(index));
  }

  /**
   * @brief 获取平滑后的参考线信息
   * @param[in] ref_line_index 参考线索引
   * @return 平滑后的参考线的信息（包含形点、航向角、曲率等）
   */
  inline const common::Path& GetReferenceLineSmoothCurve(Int32_t index) const {
    return (reference_line_set_.GetSmoothCurve(index));
  }

  /**
   * @brief 获取参考线曲率信息
   * @param[in] ref_line_index 参考线索引
   * @param[out] curve_segments 按照曲率分段后的曲线信息
   */
  inline const common::StaticVector<common::Path::CurveSegment,
  common::Path::kMaxCurveSegmentNum>&
  GetReferenceLineCurveInfo(const Int32_t ref_line_index) const {
    return (reference_line_set_.GetCurveCurvatureInfo(ref_line_index));
  }

  /**
   * @brief 获取平滑后的参考线曲率信息
   * @param[in] ref_line_index 参考线索引
   * @param[out] curve_segments 按照曲率分段后的曲线信息
   */
  inline const common::StaticVector<common::Path::CurveSegment,
  common::Path::kMaxCurveSegmentNum>&
  GetSmoothReferenceLineCurveInfo(const Int32_t ref_line_index) const {
    return (reference_line_set_.GetSmoothCurveCurvatureInfo(ref_line_index));
  }

  inline void GetReferenceLineContinuousSegment(
      Int32_t index, map_var_t* start_s, map_var_t* end_s) const {
    reference_line_set_.GetContinuousSegment(index, start_s, end_s);
  }

  /**
   * @brief 若存在从start_lane到end_lane之间的参考线，则返回相应的参考线的索引， \n
   *        否则找出一条从start_point到end_lane之间的最近的参考线 \n
   *       （包含end_lane的所有参考线中与start_point距离最近的），返回相应的参考线索引。
   * @param[in] start_point 起始点坐标
   * @param[in] start_lane_index 起始车道索引
   * @param[in] end_lane_index 结束车道索引
   * @return 参考线的索引（-1 是无效的索引）
   */
  inline Int32_t FindReferenceLine(
      const common::Vec2d& start_point,
      const Int32_t start_lane_index,
      const Int32_t end_lane_index) const {
    return (reference_line_set_.FindReferenceLine(start_point, start_lane_index,
                                                  end_lane_index));
  }

  /**
   * @brief 若存在从start_lane到end_lane之间的参考线，则返回相应的参考线的索引。
   * @param[in] start_lane_index 起始车道索引
   * @param[in] end_lane_index 结束车道索引
   * @return 参考线的索引（-1 是无效的索引）
   */
  inline Int32_t FindReferenceLine(
      const Int32_t start_lane_index,
      const Int32_t end_lane_index) const {
    return (reference_line_set_.FindReferenceLine(start_lane_index,
                                                  end_lane_index));
  }

  /**
   * @brief 通过参考线上的点查找其位于参考线上的哪个车道段
   * @param[in] ref_line_index 参考线索引
   * @param[in] s_on_ref_line 参考线上的点
   * @param[out] lane_index 这个点所位于的车道索引
   * @return 参考线上的车道段的索引
   */
  inline Int32_t FindReferenceLineLaneSegmentByProjOnRef(
      Int32_t ref_line_index,
      map_var_t s_on_ref_line,
      Int32_t * const lane_index,
      Float32_t* const s_on_lane = Nullptr_t) const {
    if (Nullptr_t == s_on_lane) {
      Float32_t s_on_lane_tmp = 0.0F;
      return (reference_line_set_.FindLaneSegmentByProjOnRef(
                ref_line_index, s_on_ref_line, lane_index, &s_on_lane_tmp));
    } else {
      return (reference_line_set_.FindLaneSegmentByProjOnRef(
                ref_line_index, s_on_ref_line, lane_index, s_on_lane));
    }
  }

  /**
   * @brief 获取道路边界信息
   * @return 道路边界信息
   */
  inline const common::StaticVector<RoadBoundary,
  common::Path::kMaxPathPointNum>& GetRoadBoundary() const {
    return (road_boundary_);
  }

  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
  /**
   * @brief 测试输入的目标物是否与道路边界产生碰撞
   * @param[in] obj 输入目标物
   * @param[out] result 可能产生碰撞风险的道路边界的信息
   * @return 碰撞风险值
   */
  Int32_t TestCollisionWithRoadBoundary(
      const CollisionTestObj& obj, CollisionTestResult* const result) const;
  /* k005 pengc 2023-01-06 (end) */

  /**
   * @brief 地图内的红绿灯列表
   * @return 地图内的红绿灯列表
   */
  inline const common::StaticVector<MapTrafficLight, MAX_MAP_TRAFFIC_LIGHT_NUM>&
  GetMapTrafficLightTable() const {
    return (map_space_.GetMapTrafficLightTable());
  }

  /**
   * @brief 测试输入的目标物是否与内部的障碍物产生碰撞风险
   * @param[in] param 参数
   * @param[in] obj 输入目标物
   * @param[out] result 可能产生碰撞风险的障碍物的信息
   * @return 碰撞风险值
   */
  inline Int32_t TestCollision(
      const CollisionTestParam& param,
      const CollisionTestObj& obj,
      CollisionTestResult* const result) const {
    return obstacle_map_.TestCollision(param, obj, result);
  }

  /**
   * @brief 测试输入的目标物行驶在输入的轨迹上是否与内部的障碍物产生碰撞风险
   * @param[in] obj_in 输入的目标物
   * @param[in] path 输入的轨迹
   * @param[in] path_sample_points 输入的轨迹的采样点
   * @param[out] result 可能产生碰撞风险的障碍物的信息
   * @return 碰撞风险值
   */
  Int32_t TestCollisionOnPath(
      const CollisionTestOnPathObj& obj_in,
      const common::Path& path,
      common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum> path_sample_points,
      CollisionTestOnPathResult* const result) const {
    return obstacle_map_.TestCollisionOnPath(
          obj_in, path, path_sample_points, result);
  }

  /**
  * @brief 判断障碍物索引是否有效
  * @param[in] obj_index 障碍物索引
  * @return true - 索引有效, false - 索引无效
  */
  inline bool IsValidObstacleIndex(Int32_t obj_index) const {
    return ((0 <= obj_index) &&
            (obj_index < obstacle_list_.obstacle_num) &&
            (obj_index < ad_msg::ObstacleList::MAX_OBSTACLE_NUM));
  }

  /**
   * @brief 根据障碍物的索引返回障碍物信息
   * @param[in] obj_index 障碍物的索引
   * @return 障碍物信息
   */
  inline const ad_msg::Obstacle& GetObstacle(
      const Int32_t obj_index) const {
    COM_CHECK((0 <= obj_index) &&
              (obj_index < obstacle_list_.obstacle_num) &&
              (obj_index < ad_msg::ObstacleList::MAX_OBSTACLE_NUM));
    return (obstacle_list_.obstacles[obj_index]);
  }

  /**
   * @brief 获取障碍物列表
   * @return 障碍物列表
   */
  inline const ad_msg::ObstacleList& GetObstacleList() const {
    return (obstacle_list_);
  }

  /**
   * @brief 获取障碍物空间中的障碍物列表
   * @return 障碍物空间中的障碍物列表
   */
  inline const common::StaticVector<Object, MAX_OBJECT_NUM_IN_OBJ_SPACE>&
      GetObstacleListInObjSpace() const {
    return (obstacle_map_.object_list());
  }

  /**
   * @brief 获取可能产生碰撞的障碍物列表
   * @param[out] 可能产生碰撞的障碍物列表
   * @note 输出的障碍物为与车辆当前位置的所有参考线可能产生碰撞的障碍物。
   */
  void GetAllRiskyObstacles(
      common::StaticVector<CollisionTestResult::ObjInfo,
      MAX_OBJECT_NUM_IN_OBJ_SPACE>* risky_obj_list,
      common::StaticVector<CollisionTestResult::ObjInfo,
      MAX_OBJECT_NUM_IN_OBJ_SPACE>* uncertain_list) const;

  /**
   * @brief 获取参考线上有碰撞风险的障碍物信息
   * @param[in] ref_line_index 参考线的索引
   * @param[out] risky_obj_list 可能产生碰撞风险的障碍物的信息
   * @return true - 有可能产生碰撞风险的障碍物信息 \n
   *         false - 没有可能产生碰撞风险的障碍物信息
   */
  bool GetRiskyObstacles(
      Int32_t ref_line_index, bool return_uncertain,
      CollisionTestResult* const result) const;


  /**
   * @brief 获取场景任务列表
   * @return 场景任务列表
   */
  const common::StaticVector<ad_msg::SceneStory,
      ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>& GetSceneStoryList() const {
    return (scene_story_set_.GetSceneStoryList());
  }
  const ad_msg::PlanningStoryList& GetPlanningStoryList() const {
    return (planning_story_set_.GetStoryList());
  }


  /**
   * @brief 读取事件报告信息
   * @param[in] num 最多可以输出的事件的数量
   * @param[out] events 保存输出的事件的列表地址
   * @return 实际读取的事件的数量
   */
  Int32_t GetEventReporting(
      Int32_t num, ad_msg::EventReporting* const events) const;

  /**
   * @brief 获取内部地图的信息(调试用)
   * @param[out] driving_map_info 内部地图信息
   */
  void GetDrivingMapInfo(DrivingMapInfo* const driving_map_info) const;

private:

  enum {
    DRIVING_DIRECTION_FORWARD = 0,
    DRIVING_DIRECTION_BACKWARD
  };

  enum {
    EVENT_TYPE_CAMERA_LANE_LOST,
    EVENT_TYPE_MAP_IS_NOT_VAILD
  };

  struct CamInfo {
    Int64_t age;
    ad_msg::MsgHead prev_msg_lane_list_head;
    // 车道线信息
    ad_msg::LaneInfoCameraList lane_list;

    void Clear() {
      age = 0;
      prev_msg_lane_list_head.Clear();
      lane_list.Clear();
    }
  };

  // 跟随目标信息
  struct FollowingTargetInfo {
    bool valid;
    Int32_t cost;
    map_var_t obj_rel_x;
    map_var_t obj_rel_y;
    map_var_t lon_dist;
    map_var_t lat_dist;

    bool operator <(const FollowingTargetInfo& right) const {
      if (cost < right.cost) {
        return true;
      } else if (cost == right.cost) {
        if (lat_dist < (right.lat_dist-0.1F)) {
          return true;
        } else if ((right.lat_dist-0.11F) < lat_dist &&
                   lat_dist < (right.lat_dist+0.11F)) {
          if (lon_dist < right.lon_dist) {
            return true;
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

    void Clear() {
      valid = false;
      cost = 0;
      obj_rel_x = 0.0F;
      obj_rel_y = 0.0F;
      lon_dist = 0.0F;
      lat_dist = 0.0F;
    }

    FollowingTargetInfo() {
      Clear();
    }
  };

  // 根据前车生成的跟车轨迹及相关信息
  struct FollowingPathInfo {
    // 此轨迹是否有效
    bool valid;
    // 跟车目标信息
    FollowingTargetInfo following_target;
    // 跟车轨迹的曲率
    map_var_t following_path_curvature;
    // 跟车轨迹
    common::StaticVector<common::Vec2d,
    common::Path::kMaxPathPointNum> following_path;

    void Clear() {
      valid = false;
      following_target.Clear();
      following_path_curvature = 0.0F;
      following_path.Clear();
    }

    FollowingPathInfo() {
      Clear();
    }
  };

  /*
   * @struct RefLineAssociation
   * @brief 与参考线相关的信息。\n
   *        主要为与此参考线可能碰撞的障碍物的一些信息。
   */
  struct RefLineAssociation {
    // 与单条参考线上的单个车道片段可能产生碰撞的障碍物的最大个数
    enum { MAX_RISKY_OBJ_NUM_IN_LANE_SEG = 8 };
    // 车道索引
    Int32_t lane_index;
    // 在车道上的起点路径长
    map_var_t start_s_on_lane;
    // 在车道上的终点路径长
    map_var_t end_s_on_lane;
    // 在参考线上的起点路径长
    map_var_t start_s_on_ref;
    // 风险目标列表是否有效
    bool risky_obj_list_valid;
    // 风险值，0到100之间的值
    Int32_t risk_value;
    // 风险目标列表
    common::StaticVector<CollisionTestResult::ObjInfo,
        MAX_RISKY_OBJ_NUM_IN_LANE_SEG> risky_obj_list;
    common::StaticVector<CollisionTestResult::ObjInfo,
        MAX_RISKY_OBJ_NUM_IN_LANE_SEG> uncertain_list;

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      lane_index = -1;
      start_s_on_lane = 0.0F;
      end_s_on_lane = 0.0F;
      start_s_on_ref = 0.0F;

      risky_obj_list_valid = false;
      risk_value = 0;

      for (Int32_t i = 0; i < risky_obj_list.Size(); ++i) {
        risky_obj_list[i].Clear();
      }
      risky_obj_list.Clear();

      for (Int32_t i = 0; i < uncertain_list.Size(); ++i) {
        uncertain_list[i].Clear();
      }
      uncertain_list.Clear();
    }

    /*
     * @brief 构造函数
     */
    RefLineAssociation() {
      lane_index = -1;
      start_s_on_lane = 0.0F;
      end_s_on_lane = 0.0F;
      start_s_on_ref = 0.0F;

      risky_obj_list_valid = false;
      risk_value = 0;
    }
  };

  class ActionSmoothing {
  public:
    ActionSmoothing() {
      req_count_threshold_ = 0;
      time_allowed_ = 0;
      req_count_ = 0;
      time_ = 0;
    }

    void Clear() {
      req_count_ = 0;
      time_ = 0;
    }

    void set_req_count_threshold(Int32_t count) {
      req_count_threshold_ = count;
    }
    void set_time_allowed(Int32_t time) {
      time_allowed_ = time;
    }

    bool Smooth(bool req) {
      if (req) {
        req_count_++;
        if (req_count_ >= req_count_threshold_) {
          req_count_ = req_count_threshold_;
          time_ = 0;
          return true;
        }
      } else {
        req_count_--;
        if (req_count_ < 0) {
          req_count_ = 0;
          time_ = 0;
        }
      }

      if (req_count_ > 0) {
        time_++;
        if (time_ >= time_allowed_) {
          req_count_ = 0;
          time_ = 0;
        }
      }

      return false;
    }

  private:
    Int32_t req_count_threshold_;
    Int32_t time_allowed_;
    Int32_t req_count_;
    Int32_t time_;
  };

private:
  void UpdateVehicleParameter();
  bool UpdatePositionInfo(
      const DrivingMapDataSource& data_source, bool gnss_is_valid);

  Int32_t SelectDrivingMapType(
      const DrivingMapDataSource& data_source,
      Int32_t location_status);
  Int32_t ConstructMixedMap(
      const DrivingMapDataSource& data_source,
      Int32_t camera_lane_quality, Int32_t location_quality);
  bool CorrectGnss(
      const common::Vec2d& pos_diff, map_var_t heading_diff,
      const common::PathPoint& proj_on_cam_lane,
      const common::Path& cam_center_path,
      HDMap& hd_map);

  /**
   * @brief 从内部地图中提取车道边界线信息(调试用)
   * @param[out] driving_map_info 内部地图信息
   */
  void ExtractLaneBoundary(DrivingMapInfo* const driving_map_info) const;

  Int32_t CheckCameraLane(const DrivingMapDataSource& data_source);
  // 20230425
  void FindObsatclePredPathByMap(
      bool find_forward, const Int32_t nearest_lane_index,
      const common::PathPoint& proj_point, const Int32_t sample_size, const map_var_t sample_step_len,
      common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>* sample_points,
      Object* object);

  struct CameraLaneProperties {
    Int32_t time_elapsed;

    bool valid_curr_center_line;
    bool valid_left_center_line;
    bool valid_right_center_line;

    Int32_t curr_central_line_idx;
    Int32_t left_central_line_idx;
    Int32_t right_central_line_idx;

    common::PathPoint veh_proj_on_cur_central_line;
    common::PathPoint veh_proj_on_left_central_line;
    common::PathPoint veh_proj_on_right_central_line;

    common::Path* cur_central_line;
    common::Path* left_central_line;
    common::Path* right_central_line;

    map_var_t left_width_of_cur_lane;
    map_var_t right_width_of_cur_lane;

    void Clear() {
      time_elapsed = 0;

      valid_curr_center_line = false;
      valid_left_center_line = false;
      valid_right_center_line = false;

      curr_central_line_idx = -1;
      left_central_line_idx = -1;
      right_central_line_idx = -1;

      veh_proj_on_cur_central_line.Clear();
      veh_proj_on_left_central_line.Clear();
      veh_proj_on_right_central_line.Clear();

      left_width_of_cur_lane = 0.0F;
      right_width_of_cur_lane = 0.0F;
    }

    CameraLaneProperties() {
      Clear();

      cur_central_line = Nullptr_t;
      left_central_line = Nullptr_t;
      right_central_line = Nullptr_t;
    }
  };
  void UpdateCameraLaneAndExtractProperties(
      const DrivingMapDataSource& data_source,
      ad_msg::LaneInfoCameraList* const lane_list,
      CameraLaneProperties* const properties);

  Int32_t AddCenterLaneToCameraLaneList(
      const Int32_t id,
      const ad_msg::LaneCenterLineCamera& central_line,
      Int32_t left_boundary_idx, Int32_t right_boundary_idx,
      ad_msg::LaneInfoCameraList* lane_list) const;

  /* k003 longjiaoy 2022-12-08 (begin)*/
  Int32_t AddBoundaryToCameraLaneList(
      const Int32_t id,
      const ad_msg::LaneBoundaryLineCamera& boundary,
      ad_msg::LaneInfoCameraList* lane_list) const;
  /* k003 longjiaoy 2022-12-08 (end)*/

  void ReplaceCenterCurveOfCameraLane(
      const common::Path& ref_path,
      map_var_t lat_offset,
      map_var_t new_half_lane_width,
      ad_msg::LaneCenterLineCamera* central_line);

  /*
   * @brief 根据当前车辆状态生成预测的轨迹
   */
  void CalcPredictedPathOfVehicle();

  /*
   * @brief 根据前车生成的跟车轨迹及相关信息
   */
  bool CalcFollowingPath(const DrivingMapDataSource& data_source);

  /*
   * @brief 从障碍物列表中查找跟随目标
   * @param[out] following_target 查找到的跟随目标信息
   * @return (true ~ 找到了可用的跟随目标, false ~ 没有找到)
   */
  bool FindFollowingTarget(
      const ad_msg::ObstacleList& obj_list,
      const ad_msg::RelativePos& rel_pos,
      const common::Matrix<map_var_t, 3, 3>& mat_conv,
      FollowingTargetInfo* const following_target);

  /*
   * @brief 更新所有障碍物信息
   */
  void UpdateObstacles(const ad_msg::ObstacleList& obj_list);

  /*
   * @brief 更新障碍物的参考线信息
   * @param[in&out] object 障碍物
   */
  void UpdateRefLineInfoOfObject(Object* object);

  /*
   * @brief 更新动态障碍物的预测轨迹
   * @param[in]     obstacle    外部输入的障碍物
   * @param[in&out] object      内部使用的障碍物
   * @note 仅更新object的pred_trajectory的值，别的变量值维持不变。
   */
  void UpdatePredictedPathOfDynamicObject(
      const ad_msg::Obstacle& obstacle, Object* object);

  /*
   * @brief 在指定位置，以指定的航向角生成自动驾驶车辆在此位姿下的有向矩形包围盒
   * @param[in]     pos          位置
   * @param[in]     heading      航向角
   * @param[out]    obb          自动驾驶车辆在对应位姿下的有向矩形包围盒
   * @note 生成的obb用于碰撞分析。
   */
  void CreateVehOBB(
      const common::Vec2d& pos, map_var_t heading,
      common::OBBox2d* obb) const;

  /*
   * @brief 创建与所有参考线相关的碰撞分析信息
   */
  bool CreateRefLineAssociation();

  /*
   * @brief 对单条参考线的碰撞分析信息进行处理
   * @param[in&out] ass 参考线的碰撞分析信息
   */
  void TestCollisionOnLaneSegment(RefLineAssociation* ass);

  /*
   * @brief 将有碰撞风险的目标加入到有碰撞风险的目标列表中
   * @param[in]     risky_obj      有碰撞风险的目标
   * @param[in&out] risky_obj_list 有碰撞风险的目标列表
   * @note 1、当目标的索引与列表中某个目标的索引相同时，若目标的碰撞
   *          风险比列表中目标的碰撞风险大，则使用目标替换列表中的
   *          相同索引的目标；
   *       2、列表满了时，当输入目标的碰撞风险比列表中碰撞风险最小的
   *          目标的碰撞风险大时，才将列表中碰撞风险最小的目标替换为
   *          当前输入目标。
   */
  template <Int32_t MaxObjNum>
  void AddRiskyObjToList(
      const CollisionTestResult::ObjInfo& risky_obj,
      common::StaticVector<CollisionTestResult::ObjInfo,
      MaxObjNum>* risky_obj_list) const {
    Int32_t risky_obj_list_size = risky_obj_list->Size();
    for (Int32_t i = 0; i < risky_obj_list_size; ++i) {
      CollisionTestResult::ObjInfo& obj = risky_obj_list->GetData(i);
      // 列表中存在与输入目标索引相同的目标
      if (risky_obj.obj_list_index == obj.obj_list_index) {
        map_var_t static_dist =
            common::Min(obj.static_distance, risky_obj.static_distance);
        if (obj < risky_obj) {
          obj = risky_obj;
        }
        obj.static_distance = static_dist;
        return;
      }
    }

    if (risky_obj_list->Full()) {
      // 输入列表已满
      const CollisionTestResult::ObjInfo* min_risk = &(risky_obj_list->Front());
      Int32_t min_risky_obj_index = 0;
      for (Int32_t i = 1; i < risky_obj_list_size; ++i) {
        const CollisionTestResult::ObjInfo& risk = risky_obj_list->GetData(i);
        if (risk < (*min_risk)) {
          min_risk = &risk;
          min_risky_obj_index = i;
        }
      }
      // 当输入目标的碰撞风险比列表中碰撞风险最小的目标的碰撞风险大时，才将列表中
      // 碰撞风险最小的目标替换为当前输入目标。
      if ((*min_risk) < risky_obj) {
        risky_obj_list->GetData(min_risky_obj_index) = risky_obj;
      }
    } else {
      // 输入列表未满，直接将当前有碰撞风险的目标添加到列表中
      risky_obj_list->PushBack(risky_obj);
    }
  }

  /*
   * @brief 计算道路的边界信息
   * @param[out] left_boundary 左边道路边界的采样点信息
   * @param[out] right_boundary 右边道路边界的采样点信息
   */
  void CalcRoadBoundary(
      common::StaticVector<RoadBoundary,
      common::Path::kMaxPathPointNum>* const boundary_info);

  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
#if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
  /*
   * @brief 构建道路边界碰撞测试空间
   * @param[in] boundary 道路边界信息
   */
  void ConstructRoadBoundaryMap(
      const common::StaticVector<
      RoadBoundary, common::Path::kMaxPathPointNum>& boundary);
#endif
  /* k005 pengc 2023-01-06 (end) */

  void SetMapAttributeFromRawMap();

  void AddEventReporting(Int32_t event_type);

  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (start)*/
  void SetMapSlopAttributeFromPccMap();
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (end)*/

private:
  DrivingMapConfig config_;

  /*
   * @struct
   * @brief 与驾驶地图相关的一些配置参数信息
   */
  struct {
    // 车长，单位：米
    map_var_t vehicle_length;
    // 车宽，单位：米
    map_var_t vehicle_width;
    // 车辆的定位点到 front of vehicle 的距离，单位：米
    map_var_t dist_of_localization_to_front;
    // 车辆的定位点到 rear of vehicle 的距离，单位：米
    map_var_t dist_of_localization_to_rear;
    // 车辆的定位点到中心点的距离，单位：米
    map_var_t dist_of_localization_to_center;
    // 车辆外接圆半径，单位：米
    map_var_t vehicle_circumradius;
    // 障碍物预测时长，单位：秒
    map_var_t pred_path_sample_time;
    // 障碍物预测轨迹最大长度，单位：米
    map_var_t pred_path_max_sample_len;

    // 选取跟车目标时，目标与参考线之间的横向距离的限制表(距离越远，允许的横向距离越大)
    common::StaticVector<common::LerpTableNodeType1, 20>
    following_target_lat_dist_limit_table;
    common::StaticVector<common::LerpTableNodeType1, 20>
    loose_following_target_lat_dist_limit_table;
  } param_;

  // 驾驶地图类型
  Int32_t driving_map_type_;
  Int32_t driving_direction_;

  // current position
  ad_msg::RelativePos current_rel_position_;
  // 将UTM坐标转换到相对坐标的转换矩阵
  common::Matrix<Float64_t, 3, 3> mat_convert_global_to_rel_coor_;
  // GNSS信息
  ad_msg::Gnss gnss_;
  // 车辆姿态信息
  ad_msg::Imu imu_;
  // 车身信息
  ad_msg::Chassis chassis_;
  // 感知模块识别的车道线信息（通常是摄像头识别的车道线）
  CamInfo cam_info_;
  // 感知识别的交通标志信息（通常是摄像头识别的交通标志）
  ad_msg::TrafficSignalList traffic_signal_list_;
  // 地图及导航信息
#if (ENABLE_MIXED_MAP_SOURCE)
  bool valid_raw_map_;
  HDMap map_space_raw_map_;
  map_var_t delta_pos_corrected_gnss_[3];
  common::Matrix<map_var_t, 3, 3> mat_convert_corrected_gnss_;
  bool mat_convert_corrected_gnss_updated_;
  common::Path current_cam_center_path_;
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (start)*/
  HDMap map_space_pcc_map_;
  ReferenceLineSet reference_line_set_pcc_map_;
  /*longjiaoy pcc 在隧道内增加坡度信号 2024-04-10 (end)*/
#endif
  HDMap map_space_;
  // 根据当前车辆状态预测的轨迹
  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
      sample_points_of_pred_path_;
  // 根据前车生成的跟车轨迹及相关信息
  FollowingPathInfo following_path_info_;
  // 参考线列表
  // reference line
#if (ENABLE_MIXED_MAP_SOURCE)
  ReferenceLineSet reference_line_set_raw_map_;
#endif
  ReferenceLineSet reference_line_set_;
  // 道路边界信息
  common::StaticVector<RoadBoundary,
  common::Path::kMaxPathPointNum> road_boundary_;
  /* k005 pengc 2023-01-06 (begin) */
  // 添加功能: 道路边界碰撞测试
#if (ENABLE_ROAD_BOUNDARY_COLLISION_TEST)
  // 道路边界space
  ObjectMapImpl road_boundary_map_;
#endif
  /* k005 pengc 2023-01-06 (end) */
  // 障碍物列表
  ad_msg::ObstacleList obstacle_list_;
  // 障碍物space
  ObjectMapImpl obstacle_map_;
  // 所有参考线的相关碰撞分析信息
  common::StaticVector<RefLineAssociation, MAX_LANE_NUM>
      ref_line_association_list_;
  // Scene Storys
  SceneStorySet scene_story_set_;
  // Planning Storys
  PlanningStorySet planning_story_set_;

  ActionSmoothing action_smoothing_camera_lane_lost_;
  common::StaticVector<ad_msg::EventReporting, 4> event_reporting_list_;

  /*
   * @struct TemporaryData
   * @brief 驾驶地图类需要用到的一些内部数据
   */
  struct TemporaryData {
    ad_msg::LaneInfoCameraList lane_list[2];
    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
        sample_2d_points;
    // 路径采样点列表
    common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>
        sample_points;

    enum { MAX_PATH_LIST_SIZE = 6 };
    common::Path path_list[MAX_PATH_LIST_SIZE];

    common::StaticVector<common::StaticVector<HDMap::LaneSeg, MAX_LANE_SEGMENT_NUM>,
          MAX_LANE_SEGMENT_NUM> search_lane_list;

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      lane_list[0].Clear();
      lane_list[1].Clear();
      sample_2d_points.Clear();
      sample_points.Clear();

      for (Int32_t i = 0; i < MAX_PATH_LIST_SIZE; ++i) {
        path_list[i].Clear();
      }

      for (Int32_t i = 0; i < search_lane_list.Size(); ++i) {
        search_lane_list[i].Clear();
      }
      search_lane_list.Clear();
    }
  } temp_data_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_DRIVING_MAP_IMPL_H_


