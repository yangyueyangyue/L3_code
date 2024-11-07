/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       driving_map_wrapper.h
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

#ifndef PHOENIX_DRIVING_MAP_DRIVING_MAP_WRAPPER_H_
#define PHOENIX_DRIVING_MAP_DRIVING_MAP_WRAPPER_H_

#include "utils/macros.h"
#include "driving_map.h"
#include "container/static_vector.h"
#include "geometry/obbox2d.h"
#include "curve/path.h"


namespace phoenix {
namespace driv_map {


/**
 * @brief 获取驾驶地图实现模块所需内存空间的字节数
 * @return 驾驶地图实现模块所需内存空间的字节数
 */
Uint32_t GetDrivingMapImplSize();

/**
 * @brief 获取驾驶地图实现模块的内存空间首地址
 * @param[in] index 驾驶地图实现模块的索引
 * @return 驾驶地图实现模块的内存空间首地址
 */
void* GetDrivingMapImpl(Int32_t index);

/**
 * @brief 拷贝驾驶地图
 * @param[in] from 被拷贝的驾驶地图的源地址
 * @param[in] to 需要获取拷贝数据的驾驶地图的目标地址
 */
void CopyDrivingMapImpl(const void * const from, void * const to);


/// 驾驶地图实现模块的前向声明
class DrivingMapImpl;

/**
 * @struct DrivingMapWrapper
 * @brief 驾驶地图接口类的实现
 */
class DrivingMapWrapper {
public:
  /**
   * @brief 构造函数
   */
  DrivingMapWrapper();
  /**
   * @brief 析构函数
   */
  ~DrivingMapWrapper();

  /**
   * @brief 获取驾驶地图实现模块的内存空间首地址
   * @return 驾驶地图实现模块的内存空间首地址
   */
  void* GetDrivingMapImpl();

  /**
   * @brief 设置驾驶地图实现模块的内存空间
   * @param[in] instance_addr 用于设置驾驶地图实现模块的内存空间的首地址
   */
  void SetDrivingMapImpl(void* instance_addr);

  /**
   * @brief 配置模块
   */
  void Configurate(const DrivingMapConfig& conf);

  /**
   * @brief 更新
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
  const map_var_t* GetDeltaPosOfCorrectedGnss() const;
  /* k004 pengc 2022-12-26 (end) */

  /**
   * @brief 获取驾驶地图的类型
   * @return 驾驶地图的类型
   */
  Int32_t GetDrivingMapType() const;

  /**
   * @brief 获取更新驾驶地图时的全局位置信息
   * @return 更新驾驶地图时的全局位置信息
   */
  const ad_msg::Gnss& GetGnss() const;

  /**
   * @brief 获取驾驶地图构建时的位置信息
   * @return 驾驶地图构建时的位置信息
   */
  const ad_msg::RelativePosList& GetRelativePosList() const;

  /**
   * @brief 获取根据当前车辆状态预测的轨迹
   * @return 轨迹
   */
  const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
  GetPredictedPathOfVehicle() const;


  /**
   * @brief 判断车道索引是否有效
   * @param[in] lane_index 车道索引
   * @return true - 索引有效, false - 索引无效
   */
  bool IsValidLaneIndex(Int32_t lane_index) const;

  /**
   * @brief 根据车道ID获取车道索引
   * @param[in] lane_id 车道ID（地图中此车道的唯一ID）
   * @return 车道索引（内部使用，提高访问效率，仅当前运算周期内有效）
   */
  Int32_t GetLaneIndexByID(const ID& lane_id) const;

  /**
   * @brief 根据车道索引获取车道ID
   * @param[in]  index 车道索引
   * @param[out] id    车道ID
   * @return true-获取成功, false-获取失败
   */
  const ID& GetLaneIdByIndex(Int32_t lane_index) const;

  /**
   * @brief 获取车道中心线信息
   * @param[in] lane_index 车道索引
   * @return 车道中心线信息(包含形点、航向角、曲率等信息)
   */
  const common::Path& GetLaneCentralCurve(const Int32_t lane_index) const;

  /**
   * @brief 获取车道限速(最高限速)
   * @param[in] lane_index 车道索引
   * @return 限速值
   */
  Float32_t GetLaneSpeedLimitHigh(const Int32_t lane_index) const;

  /**
   * @brief 获取车道限速(最低限速)
   * @param[in] lane_index 车道索引
   * @return 限速值
   */
  Float32_t GetLaneSpeedLimitLow(const Int32_t lane_index) const;

  /**
   * @brief 获取车道宽度
   * @param[in] lane_index 车道索引
   * @param[in] s 沿着车道中心线的长度
   * @param[out] left_width 此处车道中心线左边的宽度
   * @param[out] right_width 此处车道中心线右边的宽度
   */
  void GetLaneWidth(
      const Int32_t lane_index, const Float32_t s,
      Float32_t * const left_width, Float32_t * const right_width) const;

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
      Int32_t* const left_type, Int32_t* const right_type) const;
  /* k003 longjiaoy 2022-11-28 (end) */

  /**
   * @brief 获取车道坡度
   * @param[in] lane_index 车道索引
   * @param[in] s 沿着车道中心线的长度
   * @return 车道中此处位置的坡度
   */
  Float32_t GetLaneSlope(const Int32_t lane_index, const Float32_t s) const;

  /**
   * @brief 查找距离某点最近的车道信息
   * @param[in] point 点的坐标
   * @param[out] nearest_lane_index 最近的车道的索引
   * @param[out] nearest_point_on_lane 车道中心线上离输入点最近的点
   * @return true successful \n
   *         false failed
   */
  bool FindNearestLane(
      const common::Vec2d& point,
      Int32_t* nearest_lane_index,
      common::PathPoint* nearest_point_on_lane) const;

  /**
   * @brief 查找距离某点最近的车道信息，并且满足车道方向与指定的航向角一致 \n
   *       （偏差在一定范围内，避免找到反向的车道）
   * @param[in] point 点的坐标
   * @param[in] heading 点的航向角
   * @param[out] nearest_lane_index 最近的车道的索引
   * @param[out] nearest_point_on_lane 车道中心线上离输入点最近的点
   * @return true successful \n
   *         false failed
   */
  bool FindNearestLane(
      const common::Vec2d& point,
      const Float32_t heading,
      Int32_t* nearest_lane_index,
      common::PathPoint* nearest_point_on_lane) const;

  /**
   * @brief 根据距离及航向综合查找最近的车道, 避免在分流口/合流口查找到不合适的最近车道
   * @param[in]  point   待查询点的位置
   * @param[in]  heading 待查询点的航向角
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] nearest_point      最近车道上与待查询点最近的点
   * @return true-获取成功, false-获取失败
   */
  bool FindNearestLaneSynthetically(
      const common::Vec2d& point,
      Float32_t heading,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const nearest_point_on_lane) const;

  /**
   * @brief 获取待查询点的所有左右相邻车道
   * @param[in]  point   待查询点的位置
   * @param[in]  lane_index   与待查询点距离最近的车道的索引
   * @param[out] sorted_neighbor_lanes   待查询点的所有左右相邻车道列表
   * @param[out] nearest_neighbor_index  与待查询点距离最近的车道在所有左右相邻\n
   *                                     车道列表中的索引
   * @return true-获取成功, false-获取失败
   */
  bool FindNeighbors(
      const common::Vec2d& point, const Int32_t lane_index,
      common::StaticVector<NeighborsLaneInfo,
      MAX_NEIGHBOR_LANE_NUM> * const sorted_neighbor_lanes,
      Int32_t * const nearest_neighbor_index) const;

  /**
   * @brief 获取导航路径上的车道段信息
   * @return 导航道路段列表
   */
  const common::StaticVector<RoutingSection, MAX_ROUTING_SECTION_NUM>&
  GetRoutingSections() const;

  /**
   * @brief 判断指定的车道的一部分是否位于导航路径上
   * @param[in] lane_index 车道索引
   * @param[in] start_s 车道段起始位置（沿着车道中心线的长度）
   * @param[in] end_s 车道段结束位置（沿着车道中心线的长度）
   * @return true - 此车道片段处于导航路径上 \n
   *         false - 此车道片段不处于导航路径上
   */
  bool IsOverlappedWithRouting(
      const Int32_t lane_index,
      const Float32_t start_s,
      const Float32_t end_s) const;

  /**
   * @brief 判断指定的车道上的点是否位于导航路径上
   * @param[in] lane_index 车道索引
   * @param[in] s 车道上的点（沿着车道中心线的长度）
   * @return true - 此车道上的点处于导航路径上 \n
   *         false - 此车道上的点不处于导航路径上
   */
  bool IsOnRoutingSegment(const Int32_t lane_index, Float32_t s) const;

  /**
   * @brief 获取车辆当前位置最近车道的车道索引和最近点
   * @param[out] nearest_lane_index       车辆当前位置最近车道的车道索引
   * @param[out] nearest_point_on_lane    最近车道上离车辆当前位置的最近点
   */
  void GetNearestLaneToCurrPosition(
      Int32_t* const nearest_lane_index,
      common::PathPoint* const nearest_point_on_lane,
      Int32_t* const nearest_idx_in_neighbor,
      common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM>*
      const neighbor_lanes) const;


  /**
   * @brief 获取参考线数量
   * @return 参考线数量
   */
  Int32_t GetReferenceLinesNum() const;

  /**
   * @brief 判断指定的参考线索引是否有效
   * @param[in] index 参考线索引
   * @return true-指定的参考线索引有效；false-指定的参考线索引无效。
   */
  bool IsValidReferenceLineIndex(Int32_t index) const;

  /**
   * @brief 获取主参考线的索引
   * @return 主参考线的索引
   */
  Int32_t GetMajorReferenceLineIndex() const;

  /**
   * @brief 获取车辆位置在主参考线上的投影点。
   * @return 车辆位置在主参考线上的投影点
   */
  const common::PathPoint& GetProjPointOnMajorRefLine() const;

  /**
   * @brief 获取主参考线信息
   * @return 主参考线的信息（包含形点、航向角、曲率等）
   */
  const common::Path& GetMajorReferenceLine() const;

  /**
   * @brief 获取主参考线信息
   * @return 主参考线的信息（包含形点、航向角、曲率等）
   */
  const common::Path& GetMajorSmoothReferenceLine() const;

  /**
   * @brief 获取参考线NeighborFlag信息
   * @return 参考线的NeighborFlag信息
   */
  Int32_t GetReferenceLineNeighborFlag(Int32_t ref_line_index) const;

  /**
   * @brief 获取当前参考线车道线质量信息
   * @return 当前参考线的车道线质量信息
   */
  Int32_t GetReferenceLineLaneQuality(Int32_t ref_line_index) const;

  /**
   * @brief 获取与参考线关联的车道片段
   * @param[in] ref_line_index 参考线索引
   * @param[out] lane_segments 车道片段列表
   */
  const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
  GetReferenceLaneSegments(const Int32_t ref_line_index) const;

  /**
   * @brief 获取当前车辆位置在此参考线上的投影点相对与参考线起点的距离
   * @param[in] index 参考线的索引
   * @return 当前车辆位置在此参考线上的投影点相对与参考线起点的距离
   */
  map_var_t GetProjDistOfCurrPositionOnRef(Int32_t index) const;

  /**
   * @brief 获取参考线信息
   * @param[in] ref_line_index 参考线索引
   * @return 参考线的信息（包含形点、航向角、曲率等）
   */
  const common::Path& GetReferenceLine(
      const Int32_t ref_line_index) const;

  /**
   * @brief 获取平滑后的参考线信息
   * @param[in] ref_line_index 参考线索引
   * @return 平滑后的参考线的信息（包含形点、航向角、曲率等）
   */
  const common::Path& GetSmoothReferenceLine(
      const Int32_t ref_line_index) const;

  /**
   * @brief 获取参考线曲率信息
   * @param[in] ref_line_index 参考线索引
   * @param[out] curve_segments 按照曲率分段后的曲线信息
   */
  const common::StaticVector<common::Path::CurveSegment,
    common::Path::kMaxCurveSegmentNum>&
  GetReferenceLineCurveInfo(const Int32_t ref_line_index) const;

  /**
   * @brief 获取平滑后的参考线曲率信息
   * @param[in] ref_line_index 参考线索引
   * @param[out] curve_segments 按照曲率分段后的曲线信息
   */
  const common::StaticVector<common::Path::CurveSegment,
  common::Path::kMaxCurveSegmentNum>&
  GetSmoothReferenceLineCurveInfo(const Int32_t ref_line_index) const;

  /**
   * @brief 获取参考线从相机融合的段落
   * @param[in] index 参考线索引
   * @param[out] start_s 起始距离
   * @param[out] end_s 结束距离
   */
  void GetReferenceLineContinuousSegment(
      Int32_t index, map_var_t* start_s, map_var_t* end_s) const;

  /**
   * @brief 若存在从start_lane到end_lane之间的参考线，则返回相应的参考线的索引， \n
   *        否则找出一条从start_point到end_lane之间的最近的参考线 \n
   *       （包含end_lane的所有参考线中与start_point距离最近的），返回相应的参考线索引。
   * @param[in] start_point 起始点坐标
   * @param[in] start_lane_index 起始车道索引
   * @param[in] end_lane_index 结束车道索引
   * @return 参考线的索引（-1 是无效的索引）
   */
  Int32_t FindReferenceLine(
      const common::Vec2d& start_point,
      const Int32_t start_lane_index,
      const Int32_t end_lane_index) const;

  /**
   * @brief 若存在从start_lane到end_lane之间的参考线，则返回相应的参考线的索引。
   * @param[in] start_lane_index 起始车道索引
   * @param[in] end_lane_index 结束车道索引
   * @return 参考线的索引（-1 是无效的索引）
   */
  Int32_t FindReferenceLine(
      const Int32_t start_lane_index,
      const Int32_t end_lane_index) const;

  /**
   * @brief 通过参考线上的点查找其位于参考线上的哪个车道段
   * @param[in] ref_line_index 参考线索引
   * @param[in] s_on_ref_line 参考线上的点
   * @param[out] lane_index 这个点所位于的车道索引
   * @return 参考线上的车道段的索引
   */
  Int32_t FindReferenceLineLaneSegmentByProjOnRef(
      Int32_t ref_line_index,
      map_var_t s_on_ref_line,
      Int32_t * const lane_index,
      Float32_t* const s_on_lane = Nullptr_t) const;

  /**
   * @brief 获取道路边界信息
   * @return 道路边界信息
   */
  const common::StaticVector<RoadBoundary,
  common::Path::kMaxPathPointNum>& GetRoadBoundary() const;

  /**
   * @brief 测试输入的目标物是否与道路边界产生碰撞
   * @param[in] obj 输入目标物
   * @param[out] result 可能产生碰撞风险的道路边界的信息
   * @return 碰撞风险值
   */
  Int32_t TestCollisionWithRoadBoundary(
      const CollisionTestObj& obj, CollisionTestResult* const result) const;

  /**
   * @brief 地图内的红绿灯列表
   * @return 地图内的红绿灯列表
   */
  const common::StaticVector<MapTrafficLight, MAX_MAP_TRAFFIC_LIGHT_NUM>&
  GetMapTrafficLightTable() const;


  /**
   * @brief 判断障碍物索引是否有效
   * @param[in] obj_index 障碍物索引
   * @return true - 索引有效, false - 索引无效
   */
  bool IsValidObstacleIndex(Int32_t obj_index) const;

  /**
   * @brief 根据障碍物的索引返回障碍物信息
   * @param[in] obj_index 障碍物的索引
   * @return 障碍物信息
   */
  const ad_msg::Obstacle& GetObstacle(const Int32_t obj_index) const;

  /**
   * @brief 获取障碍物列表
   * @return 障碍物列表
   */
  const ad_msg::ObstacleList& GetObstacleList() const;

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
   * @brief 测试输入的目标物是否与内部的障碍物产生碰撞风险
   * @param[in] param 参数
   * @param[in] obj 输入目标物
   * @param[out] result 可能产生碰撞风险的障碍物的信息
   * @return 碰撞风险值
   */
  Int32_t TestCollision(
      const CollisionTestParam& param,
      const CollisionTestObj& obj,
      CollisionTestResult* const result) const;

  /**
   * @brief 测试输入的目标物行驶在输入的轨迹上是否与内部的障碍物产生碰撞风险
   * @param[in] obj 输入的目标物
   * @param[in] path 输入的轨迹
   * @param[in] path_sample_points 输入的轨迹的采样点
   * @param[out] result 可能产生碰撞风险的障碍物的信息
   * @return 碰撞风险值
   */
  Int32_t TestCollisionOnPath(
      const CollisionTestOnPathObj& obj,
      const common::Path& path,
      common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum> path_sample_points,
      CollisionTestOnPathResult* const result) const;


  /**
   * @brief 获取场景任务列表
   * @return 场景任务列表
   */
  const common::StaticVector<ad_msg::SceneStory,
      ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>& GetSceneStoryList() const;
  const ad_msg::PlanningStoryList& GetPlanningStoryList() const;


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
  // 驾驶地图实现模块的指针
  DrivingMapImpl* driving_map_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_DRIVING_MAP_WRAPPER_H_
