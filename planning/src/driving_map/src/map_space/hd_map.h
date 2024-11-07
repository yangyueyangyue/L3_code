/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       hd_map.h
 * @brief      内部地图
 * @details    定义了内部地图的数据格式及相关访问接口
 *
 * @author     boc
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DRIVING_MAP_HD_MAP_H_
#define PHOENIX_DRIVING_MAP_HD_MAP_H_

#include "utils/macros.h"
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
// Header file for std::string, to resolve the warning of
// cpplint [build/include_what_you_use]
#include <string>
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
#include "map_common.h"
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
#include "hdi_decision_map_structs.hpp"
#endif

#include "container/static_vector.h"
#include "container/unordered_map.h"
#include "container/ring_buffer.h"
#include "geometry/aabbox2d.h"
#include "geometry/aabboxkdtree2d.h"
#include "geometry/geometry_utils.h"
#include "driving_map.h"
#include "map_space/lane.h"
#include "map_space/routing.h"


namespace phoenix {
namespace driv_map {


/**
 * @class HDMap
 * @brief 内部地图
 */
class HDMap {
public:
  enum {
    DRIVING_DIRECTION_FORWARD = 0,
    DRIVING_DIRECTION_BACKWARD
  };
  struct LaneSeg {
    // 车道id
    ID lane_id;
    // 车道索引
    Int32_t lane_index;
    // 车道片段在车道中心线上的起点
    Float32_t start_s;
    // 车道片段在车道中心线上的终点
    Float32_t end_s;

    LaneSeg() {
      lane_id.Clear();
      lane_index = -1;
      start_s = 0.0F;
      end_s = 0.0F;
    }
  };
  /*
   * @struct SearchConnectedLanesParam
   * @brief 搜索前后相连的车道功能的参数配置
   */
  struct SearchConnectedLanesParam {
    // true ~ 向前搜索, false ~ 向后搜索
    bool search_forward;
    // true ~ 通过车道ID搜索, false ~ 通过车道存储索引搜索
    bool search_by_lane_id;
    // 搜索的长度
    map_var_t search_len;

    /*
     * @brief 构造函数。
     */
    SearchConnectedLanesParam() {
      search_forward = true;
      search_by_lane_id = false;
      search_len = 0.0F;
    }
  };

public:
  /**
   * @brief 构造函数
   */
  HDMap();

  /**
   * @brief 获取内部地图的范围
   * @return 内部地图的范围
   */
  inline map_var_t map_range() const { return map_range_; }
  /**
   * @brief 设置内部地图的范围
   * @param[in] range 内部地图的范围
   */
  inline void set_map_range(map_var_t range) { map_range_ = range; }

  /**
   * @brief 设置内部Lane的范围
   * @param[in] backward 内部Lane的范围(Backward)
   * @param[in] forward 内部Lane的范围(Forward)
   */
  inline void set_lane_range(map_var_t backward, map_var_t forward) {
    lane_range_backward_ = backward;
    lane_range_forward_ = forward;
  }

  inline void set_driving_direction(Int32_t direction) {
    driving_direction_ = direction;
  }

  /**
   * @brief 设置地图数据的时间戳 & 序列号
   * @param[in] msg_head 地图数据的时间戳 & 序列号
   */
  inline void SetHDMapMsgHead(const ad_msg::MsgHead& msg_head) {
    hd_map_msg_head_ = msg_head;
  }
  /**
   * @brief 获取地图数据的时间戳 & 序列号
   * @return 地图数据的时间戳 & 序列号
   */
  inline const ad_msg::MsgHead& GetHDMapMsgHead() const {
    return (hd_map_msg_head_);
  }
  /**
   * @brief 获取地图数据的时间戳 & 序列号
   * @return 地图数据的时间戳 & 序列号
   */
  inline ad_msg::MsgHead& hd_map_msg_head() { return (hd_map_msg_head_); }

  /**
   * @brief 设置地图构建时的位置信息
   * @param[in] pos 地图构建时的位置信息
   */
  void SetGlobalPosition(Float64_t x, Float64_t y, Float64_t z, Float32_t h) {
    current_global_position_.x = x;
    current_global_position_.y = y;
    current_global_position_.heading = h;
    current_global_position_.s = 0.0;

    // Set bounding box around the current postion.
    map_bounding_box_.min_x = current_global_position_.x - map_range_;
    map_bounding_box_.min_y = current_global_position_.y - map_range_;
    map_bounding_box_.max_x = current_global_position_.x + map_range_;
    map_bounding_box_.max_y = current_global_position_.y + map_range_;
  }

  void SetMatConvertGlobalToRelCoor(
      const common::Matrix<Float64_t, 3, 3>& mat) {
    mat_convert_global_to_rel_coor_ = mat;
  }

  /**
   * @brief 获取地图构建时的位置信息
   * @return 地图构建时的位置信息
   */
  inline const ad_msg::RelativePosList& GetRelPosList() const {
    return rel_pos_list_;
  }
  /**
   * @brief 设置地图构建时的位置信息
   * @param[in] pos 地图构建时的位置信息
   */
  inline void SetRelPosList(const ad_msg::RelativePosList& pos_list) {
    rel_pos_list_ = pos_list;
  }

  inline void SetCurrentRelPosition(const ad_msg::RelativePos& pos) {
    current_rel_position_ = pos;
  }

  /**
   * @brief 清除内部信息
   */
  void Clear();

  bool ConstructMapInfoFromPath(
      const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
          pred_path);

  bool ConstructMapInfoFromCamera(
      const ad_msg::LaneInfoCameraList& camera_lane_list);

  bool ConstructMapInfoFromOtherMap(
      const HDMap& other_map,
      const common::Matrix<map_var_t, 3, 3>& mat_conv,
      const common::Path* cam_center_path = Nullptr_t);

#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  /**
   * @brief 从外部地图构建内部地图
   * @param[in] map 外部地图信息
   * @return true-成功, false-失败
   */
  bool ConstructMapInfo(const apollo::hdmap::Map& map);
  /**
   * @brief 从外部导航路径构建内部导航路径
   * @param[in] routing 外部导航路径
   * @return true-成功, false-失败
   */
  bool ConstructRoutingInfo(const apollo::routing::RoutingResponse& routing);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  /**
   * @brief 从外部地图构建内部地图
   * @param[in] map_in 外部地图信息
   * @param[in] map_location_in 外部地图中的定位消息
   * @return true-成功, false-失败
   */
  bool ConstructMapInfo(const MAP::MAP_VecLaneInfo& map_in,
                        const MAP::MAP_Location& map_location_in);
  /**
   * @brief 从外部导航路径构建内部导航路径
   * @param[in] routing_in 外部导航路径
   * @return true-成功, false-失败
   */
  bool ConstructRoutingInfo(const MAP::MAP_Routing& routing_in);
#elif HD_MAP_TYPE == HD_MAP_TYPE_D17
  /**
   * @brief 从外部地图构建内部地图
   * @param[in] map 外部地图信息
   * @return true-成功, false-失败
   */
  bool ConstructMapInfo(const exp_map_t::stMapExpProfileInfo& map);
  /**
   * @brief 从外部导航路径构建内部导航路径
   * @param[in] routing 外部导航路径
   * @return true-成功, false-失败
   */
  bool ConstructRoutingInfo(const Uint8_t& routing);
#endif

  /**
   * @brief 获取车道列表
   */
  const common::StaticVector<Lane, MAX_LANE_NUM>& GetLaneTable() const {
    return lane_table_storage_;
  }

  /**
   * @brief 判断车道索引是否有效
   * @param[in] lane_index 车道索引
   * @return true-有效, false-无效
   */
  bool IsValidLaneIndex(const Int32_t lane_index) const {
    return ((0 <= lane_index) && (lane_index < lane_table_storage_.Size()));
  }

  /**
   * @brief 根据车道ID获取车道索引
   * @param[in] id 车道ID
   * @return 车道索引
   */
  Int32_t GetLaneIndexById(const ID& id) const;

  /**
   * @brief 根据车道索引获取车道ID
   * @param[in]  index 车道索引
   * @param[out] id    车道ID
   * @return true-获取成功, false-获取失败
   */
  inline const ID& GetLaneIdByIndex(Int32_t index) const {
    COM_CHECK((0 <= index) && (index < lane_table_storage_.Size()));
    return (lane_table_storage_[index].topology_id().id);
  }

  /**
   * @brief 根据车道索引获取车道
   * @param[in]  index 车道索引
   * @return 与指定车道索引对应的车道
   */
  inline const Lane& GetLane(Int32_t index) const {
    COM_CHECK((0 <= index) && (index < lane_table_storage_.Size()));

    return (lane_table_storage_[index]);
  }
  inline Lane& GetLane(Int32_t index) {
    COM_CHECK((0 <= index) && (index < lane_table_storage_.Size()));

    return (lane_table_storage_[index]);
  }

  /**
   * @brief 获取最近的车道及在此车道上的最近点
   * @param[in]  point 待查询点的位置
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] nearest_point      最近车道上与待查询点最近的点
   * @return true-获取成功, false-获取失败
   */
  bool FindNearestLane(
      const common::Vec2d& point,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const nearest_point) const;

  /**
   * @brief 获取与待查询点方向一致的最近的车道及在此车道上的最近点
   * @param[in]  point   待查询点的位置
   * @param[in]  heading 待查询点的航向角
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] nearest_point      最近车道上与待查询点最近的点
   * @return true-获取成功, false-获取失败
   */
  bool FindNearestLane(
      const common::Vec2d& point,
      Float32_t heading,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const nearest_point) const;

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
      common::PathPoint * const nearest_point) const;

  /**
   * @brief 查找查询点一定范围内的车道
   * @param[in]  point 待查询点的位置
   * @param[out] distance 查询半径
   * @param[out] lane_idx_list  查询点一定范围内的车道
   * @return true-获取成功, false-获取失败
   */
  bool FindLanes(
      const common::Vec2d& point,
      map_var_t distance,
      common::StaticVector<Int32_t, MAX_LANE_NUM>* const lane_idx_list) const;

  /**
   * @brief 获取最近的车道及在此车道上的投影点
   * @param[in]  point 待查询点的位置
   * @param[out] nearest_lane_index 最近车道索引
   * @param[out] proj_point         待查询点在最近车道上的投影点
   * @return true-获取成功, false-获取失败
   */
  bool FindProjOfNearestLane(
      const common::Vec2d& point,
      Int32_t * const nearest_lane_index,
      common::PathPoint * const proj_point) const;

  /**
   * @brief 获取待查询点的所有左右相邻车道
   * @param[in]  point   待查询点的位置
   * @param[in]  lane_index   与待查询点距离最近的车道的索引
   * @param[out] sorted_neighbor_lanes   待查询点的所有左右相邻车道列表
   * @param[out] nearest_neighbor_index  与待查询点距离最近的车道在所有左右相邻\n
   *                                     车道列表中的索引
   * @return true-获取成功, false-获取失败
   * @note sorted_neighbor_lanes中车道的排列顺序为从右向左进行排列
   */
  bool FindNeighbors(
      const common::Vec2d& point,
      const Int32_t lane_index,
      common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM> * const
          sorted_neighbor_lanes,
      Int32_t * const nearest_neighbor_index) const;

  /**
   * @brief 判断指定车道的某一段与导航路径是否有重叠
   * @param[in] lane_index 车道的索引
   * @param[in] start_s    起点的路径长
   * @param[in] end_s      终点的路径长
   * @return true-有重叠；false-没有重叠。
   */
  inline bool IsOverlappedWithRoutingSegment(
      Int32_t lane_index, map_var_t start_s, map_var_t end_s) const {
    return (routing_.IsOverlappedWithRoutingSegment(
              lane_index, start_s, end_s));
  }

  /**
   * @brief 判断指定车道的某一段与导航路径是否有重叠。
   *        当有重叠时，获取重叠部分对应导航路径上的导航道路段索引和
   *        左右相邻导航车道索引。
   * @param[in] lane_index      车道的索引
   * @param[in] start_s         起点的路径长
   * @param[in] end_s           终点的路径长
   * @param[out] segment_index  重叠部分对应导航路径上的导航道路段索引
   * @param[out] neighbor_index 重叠部分对应导航路径上的左右相邻导航车道索引
   * @return true-有重叠；false-没有重叠。
   */
  inline bool IsOverlappedWithRoutingSegment(
      Int32_t lane_index, map_var_t start_s, map_var_t end_s,
      Int32_t * const segment_index, Int32_t * const neighbor_index) const {
    return (routing_.IsOverlappedWithRoutingSegment(
              lane_index, start_s, end_s, segment_index, neighbor_index));
  }

  /**
   * @brief 地图内的红绿灯列表
   * @return 地图内的红绿灯列表
   */
  inline const common::StaticVector<MapTrafficLight, MAX_MAP_TRAFFIC_LIGHT_NUM>&
  GetMapTrafficLightTable() const {
    return (map_traffic_light_table_);
  }

  /**
   * @brief 获取导航道路段列表
   * @return 导航道路段列表
   */
  inline const common::StaticVector<RoutingSection, MAX_ROUTING_SECTION_NUM>&
  GetRoutingSections() const {
    return (routing_.routing_sections());
  }

  /**
   * @brief 判断指定车道的某一个点是否在导航路径上
   * @param[in] lane_index 车道的索引
   * @param[in] s          点的路径长
   * @return true-在导航路径上；false-不在导航路径上。
   */
  inline bool IsOnRoutingSegment(Int32_t lane_index, map_var_t s) const {
    return (routing_.IsOnRoutingSegment(lane_index, s));
  }

  /* k004 pengc 2022-12-26 (begin) */
  // 将车道线与地图进行配准
  /**
   * @enum 位置估计的质量
   */
  enum {
    REGISTER_QUALITY_INVALID = 0,
    REGISTER_QUALITY_BAD,
    REGISTER_QUALITY_NOT_SO_GOOD,
    REGISTER_QUALITY_GOOD
  };
  /**
   * @brief 将轨迹与地图进行配准
   * @param[in] dst_path 待配准的轨迹
   * @param[in&out] delta_pos 配准前初始值 & 配准后的位置增量
   * @return 配准的质量 (0~无效, 1~质量差, 2~质量一般, 3~质量好)
   */
  Int32_t RegisterPath(
      const common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum>& sample_points,
      map_var_t delta_pos[3]);

  /**
   * @brief 使用相机车道中心线替换地图中的车道中心线
   * @param[in] src_path 相机车道中心线
   * @return true ~ OK, false ~ NG
   */
  bool ReplaceCenterLine(const common::Path& src_path);
  /* k004 pengc 2022-12-26 (end) */

  /*
   * @brief 根据地图拓扑关系，搜索前后相连的车道
   * @param[in] param 参数
   * @param[in] start_lane_idx   搜索起始车道的车道索引
   * @param[in] start_point_on_lane  起始车道上的起始点
   * @param[out] lane_list 前后连接的车道段列表
   */
  void SearchConnectedLanes(
      const SearchConnectedLanesParam& param,
      Int32_t start_lane_idx, const common::PathPoint& start_point_on_lane,
      common::StaticVector<common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM>,
      MAX_LANE_SEGMENT_NUM>* const lane_list);

private:
  /*
   * @struct GlobalPoint
   * @brief 全局坐标系二维点
   */
  struct GlobalPoint {
    // x坐标
    Float64_t x;
    // y坐标
    Float64_t y;
    // 航向角（与x方向的夹角）
    Float32_t heading;
    // 离车道起点的路径长
    Float64_t s;

    /*
     * @brief 构造函数
     */
    GlobalPoint() {
      x = 0.0;
      y = 0.0;
      heading = 0.0F;
      s = 0.0;
    }
    /*
     * @brief 构造函数
     * @param[in]  x_value   x坐标
     * @param[in]  y_value   y坐标
     */
    GlobalPoint(Float64_t x_value, Float64_t y_value) {
      x = x_value;
      y = y_value;
      heading = 0.0F;
      s = 0.0;
    }

    /*
     * @brief 清除成员变量的值
     */
    void Clear() {
      x = 0.0;
      y = 0.0;
      heading = 0.0F;
      s = 0.0;
    }
  };

  /*
   * @struct HoldPoint
   * @brief 保留下来的全局坐标系二维点
   */
  struct HoldPoint {
    // 最近点信息是否有效
    bool nearest_info_valid;
    // 点到车辆位置的平方距离
    Float64_t sq_dist_to_veh;
    // 上一点与当前点构成的线段上与车辆位置最近的点
    GlobalPoint nearest_point_to_veh;
    // 点在二维全局坐标系下的坐标
    GlobalPoint point;

    /*
     * @brief 清除成员变量的值
     */
    void Clear() {
      nearest_info_valid = false;
      sq_dist_to_veh = 0.0;
      nearest_point_to_veh.Clear();
      point.Clear();
    }

    /*
     * @brief 构造函数
     */
    HoldPoint() {
      nearest_info_valid = false;
      sq_dist_to_veh = 0.0;
    }
  };

  /*
   * @struct LaneSegmentObj
   * @brief 由车道段构成的目标，供k-d树使用。\n
   *        lane segment object for k-d tree
   */
  struct LaneSegmentObj {
    // 车道索引
    Int32_t lane_index;
    // 目标索引
    Int32_t object_index;

    /*
     * @brief 清除成员变量的值
     */
    void Clear() {
      lane_index = -1;
      object_index = -1;
    }

    /*
     * @brief 构造函数
     */
    LaneSegmentObj() {
      Clear();
    }
  };

  class CalcSquareDistToPt;
  // 声明类HDMap为类CalcSquareDistToPt的友元类，也即在类CalcSquareDistToPt中，
  // 可以访问类HDMap的私有成员变量和私有成员函数。
  friend class CalcSquareDistToPt;
  /*
   * @class CalcSquareDistToPt
   * @brief 计算点到车道段目标的平方距离
   */
  class CalcSquareDistToPt {
  public:
    /*
     * @brief 构造函数
     * @param[in]  point   待查询点
     * @param[in]  hd_map  内部地图数据
     */
    CalcSquareDistToPt(const common::Vec2d& point, const HDMap& hd_map)
      : point_(point), hd_map_(hd_map) {
      // nothing to do
    }
    /*
     * @brief 重载的'()'运算符
     * @param[in]  obj             待查询的车道段目标
     * @param[in]  tree_obj_index  待查询的车道段目标在k-d树中对应的索引
     * @return 点到车道段目标对应线段的平方最近距离。车道段目标对应线段为\n
     *         索引为obj.lane_index的车道上第obj.object_index+1个点到\n
     *         第obj.object_index+2个点所组成的线段。
     */
    map_var_t operator ()(
        const LaneSegmentObj& obj, Int32_t tree_obj_index) const {
      const common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>&
          points = hd_map_.lane_table_storage_[obj.lane_index].
                   central_curve().points();
      return (common::SqDistPointToSeg_2D(
                points[obj.object_index], points[obj.object_index + 1],
                point_));
    }

  private:
    // 待查询点
    const common::Vec2d point_;
    // 内部地图
    const HDMap& hd_map_;
  };

  /*
   * @class CmpNeighborLane
   * @brief 左右相邻车道比较类。用于将左右相邻车道从右到左进行排序。
   */
  class CmpNeighborLane {
  public:
    /*
     * @brief 重载的'()'运算符。
     * @param[in]  lane1         输入的第1个左右相邻车道的信息
     * @param[in]  lane2         输入的第2个左右相邻车道的信息
     * @return true  - lane1在lane2的右边；\n
     *         false - lane1在lane2的左边，或者lane1与lane2重合。
     * @note  1、由于proj_point.l为存储的车辆当前位置到车道上的横向距离，所以车道到
     *           车辆当前位置的横向距离为-proj_point.l，所以定义的时候是定义的'>'，
     *           这样进行排序时就是车道到车辆当前位置小的在前，车道到车辆当前位置大的
     *           在后。
     */
    bool operator ()(
        const NeighborsLaneInfo& lane1, const NeighborsLaneInfo& lane2) const {
      return (lane1.proj_point.l > lane2.proj_point.l);
    }
  };

  /*
   * @struct LaneInBoundary
   * @brief 在边界内的车道
   */
  struct LaneInBoundary {
    // 车道id
    ID lane_id;
    // 车道索引
    Int32_t lane_index;
    // 车道起点的路径长
    Float64_t start_s;
    // 车道终点的路径长
    Float64_t end_s;

    /*
     * @brief 清除成员变量的值
     */
    void Clear() {
      lane_id.Clear();
      lane_index = -1;
      start_s = 0.0;
      end_s = 0.0;
    }

    /*
     * @brief 构造函数
     */
    LaneInBoundary() {
      lane_index = -1;
      start_s = 0.0;
      end_s = 0.0;
    }
  };

  /* k004 pengc 2022-12-26 (begin) */
  // 将车道线与地图进行配准
  class FuncGetPoints {
  public:
    FuncGetPoints(const common::StaticVector<common::Vec2d,
                  common::Path::kMaxPathPointNum>& pts)
      : points_(pts) {
    }

    const map_var_t x(const Int32_t index) const { return (points_[index].x()); }
    const map_var_t y(const Int32_t index) const { return (points_[index].y()); }
    const map_var_t z(const Int32_t index) const { return (0.0F); }

  private:
    const common::StaticVector<common::Vec2d,
        common::Path::kMaxPathPointNum>& points_;
  };
  /* k004 pengc 2022-12-26 (end) */

private:
  Int32_t ConstructLaneInfoFromCamera(
      Int32_t center_line_idx,
      const ad_msg::LaneInfoCameraList& camera_lane_list,
      Int32_t ref_lane_idx,
      map_var_t lat_offset);

  /* k003 longjiaoy 2022-11-28 (start) */
  // 从相机中获取车道线类型
  Int32_t GetLaneBoundaryTypeFromCamera(Int32_t camera_lane_boundary_type);
  /* k003 longjiaoy 2022-11-28 (end) */

  void ExtendPath(
      map_var_t ex_len_backward, map_var_t ex_len_forward, common::Path* path);

#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  /*
   * @brief 解析车道信息（只解析范围内的）
   * @param[in] lane 外部地图中的车道信息
   * @return true-成功, false-失败
   */
  bool ParseLaneInfo(const apollo::hdmap::Lane& lane);

  /*
   * @brief 解析车道边界信息（只解析范围内的）
   * @param[in] lane 外部地图中的车道信息
   * @param[in] start_s 车道范围起始位置（以沿着道路中心线的长度表示）
   * @param[in] end_s 车道范围结束位置（以沿着道路中心线的长度表示）
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneBoundary(
      const apollo::hdmap::Lane& lane, Float64_t start_s, Float64_t end_s,
      Lane* lane_info);

  /*
   * @brief 解析车道坡度信息（只解析范围内的）
   * @param[in] lane 外部地图中的车道信息
   * @param[in] start_s 车道范围起始位置（以沿着道路中心线的长度表示）
   * @param[in] end_s 车道范围结束位置（以沿着道路中心线的长度表示）
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneSlope(
      const apollo::hdmap::Lane& lane, Float64_t start_s, Float64_t end_s,
      Lane* lane_info);

  /*
   * @brief 解析车道拓扑关系（相邻、相连）
   * @param[in] lane 外部地图中的车道信息
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneTopology(const apollo::hdmap::Lane& lane, Lane* lane_info);

  /*
   * @brief 解析红绿灯信息
   * @param[in] lane 外部地图中的红绿灯信息
   */
  bool ParseMapTrafficLight(const apollo::hdmap::Signal& map_signal);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  /*
   * @brief 解析车道信息（只解析范围内的）
   * @param[in] map_laneinfo_in 外部地图中的车道信息
   * @param[in] map_location_in 外部地图中的定位信息
   * @return true-成功, false-失败
   */
  bool ParseLaneInfo(const MAP::MAP_LaneInfo& map_laneinfo_in,
                     const MAP::MAP_Location& map_location_in);

  /*
   * @brief 解析车道边界信息（只解析范围内的）
   * @param[in] map_location_in 外部地图中的定位信息
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneBoundary(const MAP::MAP_LaneInfo& map_laneinfo_in,
                         const MAP::MAP_Location& map_location_in,
                         Lane* const lane_info);

  /*
   * @brief 解析车道坡度信息（只解析范围内的）
   * @param[in] lane_in 外部地图中的车道信息
   * @param[in] start_s 车道范围起始位置（以沿着道路中心线的长度表示）
   * @param[in] end_s 车道范围结束位置（以沿着道路中心线的长度表示）
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneSlope(
      const MAP::MAP_LaneInfo& lane_in, Float64_t start_s, Float64_t end_s,
      Lane* const lane_info) const;

  /*
   * @brief 解析车道拓扑关系（相邻、相连）
   * @param[in] map_lane 外部地图中的车道信息
   * @param[out] lane_output 保存解析后的信息
   */
  void ParseLaneTopology(const MAP::MAP_LaneInfo& map_lane, Lane* lane_output);
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  /*
   * @brief 解析车道信息（只解析范围内的）
   * @param[in] lane 外部地图中的车道信息
   * @return true-成功, false-失败
   */
  bool ParseLaneInfo(const ::decision_map_t::sDecisionMapLaneItem& lane);

  /*
   * @brief 解析车道边界信息（只解析范围内的）
   * @param[in] lane 外部地图中的车道信息
   * @param[in] start_s 车道范围起始位置（以沿着道路中心线的长度表示）
   * @param[in] end_s 车道范围结束位置（以沿着道路中心线的长度表示）
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneBoundary(
      const ::decision_map_t::sDecisionMapLaneItem& lane,
      Float64_t start_s, Float64_t end_s, Lane* lane_info);

  /* k003 longjiaoy 2022-12-08 (start) */
  /*
   * @brief 将地图中的车道边界类型转换为驾驶地图中的车道边界类型
   * @param[in] map_lane_mark 地图中的车道边界类型
   * @return 驾驶地图中的车道边界类型
   */
  Int32_t ConvLeftLaneBoundaryType (const LineMarking& map_lane_mark);
  Int32_t ConvRightLaneBoundaryType (const LineMarking& map_lane_mark);
  /* k003 longjiaoy 2022-12-08 (end) */

  /*
   * @brief 解析车道坡度信息（只解析范围内的）
   * @param[in] lane 外部地图中的车道信息
   * @param[in] start_s 车道范围起始位置（以沿着道路中心线的长度表示）
   * @param[in] end_s 车道范围结束位置（以沿着道路中心线的长度表示）
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneSlope(
      const ::decision_map_t::sDecisionMapLaneItem& lane,
      Float64_t start_s, Float64_t end_s, Lane* lane_info);

  /*
   * @brief 解析车道拓扑关系（相邻、相连）
   * @param[in] lane 外部地图中的车道信息
   * @param[out] lane_info 保存解析后的信息
   */
  void ParseLaneTopology(
      const ::decision_map_t::sDecisionMapLaneItem& lane, Lane* lane_info);
#endif

  /*
   * @brief 将在范围内的车道中心线形点保存下来
   * @param[in] is_in_boundary 输入点是否在范围内
   * @param[in] prev_point_valid 输入点之前的点信息是否有效
   * @param[in] prev_point 输入点之前的点
   * @param[in] curr_point 输入点
   */
  void TakePointsInBoundary(
      bool is_in_boundary, bool prev_point_valid,
      const GlobalPoint& prev_point, const GlobalPoint& curr_point);

  /*
   * @brief 将车道段拷贝到另一段
   * @param[in] index_from 源车道段索引
   * @param[in] index_to 目标车道段索引
   */
  void CopyPartLaneSeg(Int32_t index_from, Int32_t index_to);

  /*
   * @brief 选择最好的车道段（与当前车辆位置同向, 且距离最近）
   * @param[out] better 最好的车道段索引
   * @return true - 成功, false - 失败
   */
  bool SelectBetterLaneSeg(Int32_t* const better) const;

  /*
   * @brief 限制车道的范围(限制车道向前及向后的长度)
   * @param[in] seg_index 需要限制的车道段索引
   */
  void LimitLaneRang(Int32_t seg_index);

  /*
   * @brief 计算线段与输入点的最近距离
   * @param[in] a 线段起点
   * @param[in] b 线段终点
   * @param[in] p 输入点
   * @param[out] d 线段上与输入点最近的点
   * @param[out] t 线段上与输入点最近的点在线段起点及终点之间的比例
   */
  void GetClosestPtPointToSeg(
      const GlobalPoint& a, const GlobalPoint& b, const GlobalPoint& p,
      GlobalPoint* const d, Float64_t* const t) const;

  /*
   * @brief 更新道路拓扑关系(主要更新道路索引)
   */
  void UpdateLanesTopology();

  /*
   * @brief 判断点是否在道路范围内
   * @param[in] x 点的x坐标
   * @param[in] y 点的y坐标
   */
  bool IsPointInMapBoundingBox(Float64_t x, Float64_t y);

  /*
   * @brief 将全局坐标系下的轨迹转换为车身坐标系下的轨迹
   * @param[in] global_points 输入的全局坐标系下的轨迹
   * @param[in] relative_points 输出的车身坐标系下的轨迹
   */
  void ConvertPointsToRelCoordinate(
      const common::StaticVector<GlobalPoint,
                common::Path::kMaxPathPointNum>& global_points,
      common::StaticVector<common::Vec2d,
                common::Path::kMaxPathPointNum>* const relative_points);

  /*
   * @brief 将全局坐标系下的轨迹转换为车身坐标系下的轨迹
   * @param[in] global_points 输入的全局坐标系下的轨迹
   * @param[in] relative_points 输出的车身坐标系下的轨迹
   */
  void ConvertPointsToRelCoordinate(
      const common::RingBuffer<HoldPoint,
                common::Path::kMaxPathPointNum>& hold_points,
      common::StaticVector<common::Vec2d,
                common::Path::kMaxPathPointNum>* const relative_points);

  /*
   * @brief 使用所有车道的中心线构建k-d树（用来加快最近道路的查找速度）
   */
  void BuildLaneSegmentKDTree();

  /*
   * @brief 从左右相邻道路表中查找指定的车道索引
   * @param[in] lane_index      车道索引
   * @param[in] neighbor_lanes  左右相邻道路表
   * @return true - 找到了, false - 没有找到
   */
  bool FindLaneFromNeighborLanesTable(
      Int32_t lane_index,
      const common::StaticVector<NeighborsLaneInfo,
          MAX_NEIGHBOR_LANE_NUM>& neighbor_lanes) const;

  /*
   * @brief 从前后相连道路表中查找指定的车道索引
   * @param[in] lane_index       车道索引
   * @param[in] connected_lanes  前后相连道路表
   * @return true - 找到了, false - 没有找到
   */
  bool FindLaneFromConnectedLanesTable(
      const Int32_t lane_index,
      const common::StaticVector<Lane::TopologyId,
          Lane::MAX_TOPOLOGY_CONNECTED_LANE_NUM>& connected_lanes) const;

  /*
   * @brief 从边界内的道路表中查找指定的车道ID对应的车道的车道索引
   * @param[in] lane_id       车道ID
   * @return 边界内的道路表中与指定车道ID对应的车道的车道索引
   */
  Int32_t FindLaneFromLanesInBoundary(const ID& lane_id) const;


  void PrintHDMap() const;

private:
  // 内部地图的范围
  map_var_t map_range_;
  // 向后的车道长度限制
  map_var_t lane_range_backward_;
  // 向前的车道长度限制
  map_var_t lane_range_forward_;
  // 最小的车道线相邻两点之间的距离
  map_var_t min_lane_point_interval_;
  // Driving direction
  Int32_t driving_direction_;
  // 地图构建时的位置信息
  ad_msg::RelativePos current_rel_position_;
  GlobalPoint current_global_position_;
  ad_msg::RelativePosList rel_pos_list_;
  common::Matrix<Float64_t, 3, 3> mat_convert_global_to_rel_coor_;
  // 边界框，内部地图只保存边界框内部的信息
  struct {
    // 最小x坐标
    Float64_t min_x;
    // 最小y坐标
    Float64_t min_y;
    // 最大x坐标
    Float64_t max_x;
    // 最大y坐标
    Float64_t max_y;
  } map_bounding_box_;
  // 地图数据的序列号 & 地图数据的时间戳
  ad_msg::MsgHead hd_map_msg_head_;
  // 无序车道表，由车道ID与车道在车道信息列表中的索引组成。
  // lane table
#if HD_MAP_TYPE == HD_MAP_TYPE_APOLLO
  common::UnorderedMap<std::string, Int32_t, MAX_LANE_NUM> lane_table_;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D600)
  common::UnorderedMap<Uint64_t, Int32_t, MAX_LANE_NUM> lane_table_;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  common::UnorderedMap<Uint64_t, Int32_t, MAX_LANE_NUM> lane_table_;
#else
  common::UnorderedMap<Int32_t, Int32_t, MAX_LANE_NUM> lane_table_;
#endif
  // 保存车道信息的列表
  // 所有车道列表
  common::StaticVector<Lane, MAX_LANE_NUM> lane_table_storage_;
  // 边界内的车道列表
  common::StaticVector<LaneInBoundary, MAX_LANE_NUM> lanes_in_boundary_;
  // 用于加快最近路径点查询等功能的k-d树
  common::AABBoxKDTree2d<LaneSegmentObj,
      common::Path::kMaxPathPointNum * MAX_LANE_NUM, 16> lane_segment_kdtree_;

  // 地图内的红绿灯
  common::StaticVector<MapTrafficLight,
      MAX_MAP_TRAFFIC_LIGHT_NUM> map_traffic_light_table_;

  Routing routing_;
  /*
   * @struct TemporaryData
   * @brief 临时数据(程序运行过程中需要大块的临时数据)
   */
  struct TemporaryData {
    // 车道栈存储结点列表，在构建车道k-d树时需要用到的k-d树的内部变量。
    // used for k-d tree
    common::StaticVector<common::AABBoxKDTreeNodeAssociation, 64>
        tree_nodes_stack;
    /*
     * @struct TakePath
     * @brief 从车道中截取下来的车道。主要用于处理车道中心线
     */
    struct TakePath {
      // 当前车道段的索引
      Int32_t curr_lane_seg_index;

      /*
       * @struct LaneSeg
       * @brief 车道段
       */
      struct LaneSeg {
        // 车道段起点在车道上的路径长
        Float64_t take_start_s;

        // 最近点信息是否有效
        bool nearest_info_valid;
        // 车道段到车辆的平方最近距离
        Float64_t sq_dist_to_veh;
        // 车道段到车辆的最近点
        GlobalPoint nearest_point_to_veh;

        // 车道段在指定范围内截取留下来的环形点列
        common::RingBuffer<HoldPoint, common::Path::kMaxPathPointNum>
            hold_points;

        /*
         * @brief 清除成员变量的值
         */
        void Clear() {
          take_start_s = -1.0;
          nearest_info_valid = false;
          sq_dist_to_veh = 0.0;
          nearest_point_to_veh.Clear();
          hold_points.Clear();
        }
      } lane_seg[2];

      /*
       * @brief 清除成员变量的值
       */
      void Clear() {
        curr_lane_seg_index = 0;
        lane_seg[0].Clear();
        lane_seg[1].Clear();
      }
    } take_path;

    /*
     * @struct StackForFindingLanes
     * @brief 查找车道段列表时需要的栈结构体。
     */
    struct StackForFindingLanes {
      /*
       * @struct StackNode
       * @brief 查找车道段列表时需要的单个栈结点。
       */
      struct StackNode {

        // 已经存储的车道段列表
        common::StaticVector<LaneSeg, MAX_LANE_SEGMENT_NUM> lanes;
        // 已累计的路径长
        map_var_t accumulated_s;
        // 当前正在处理的前向车道在前向车道列表中的索引
        Int32_t access_index;

        // 用于车道中心线的空间转换
        common::Matrix<map_var_t, 3, 3> mat_conv;

        /*
         * @brief 清空所有成员变量的值。
         */
        void Clear() {
          lanes.Clear();
          accumulated_s = 0.0F;
          access_index = 0;
          mat_conv.SetIdentity();
        }

        /*
         * @brief 构造函数。
         */
        StackNode() {
          accumulated_s = 0.0F;
          access_index = 0;
          mat_conv.SetIdentity();
        }
      };

      // 栈中的结点列表
      common::StaticVector<StackNode, MAX_LANE_SEGMENT_NUM> nodes;

      /*
       * @brief 将栈中的结点信息清空。
       */
      void Clear() {
        for (Int32_t i = 0; i < nodes.Size(); ++i) {
          nodes[i].Clear();
        }
        nodes.Clear();
      }

      /*
       * @brief 构造函数。
       */
      StackForFindingLanes() {
        Clear();
      }

      /*
       * @brief   分配一个新的栈结点。
       * @return  分配的新的栈结点。
       */
      inline StackNode* Allocate() {
        StackNode * const node = nodes.Allocate();
        if (Nullptr_t != node) {
          node->Clear();
        }
        return (node);
      }

      /*
       * @brief 判断栈是否为空。
       * @return true-栈为空；false-栈不为空。
       */
      bool Empty() {
        return nodes.Empty();
      }

      /*
       * @brief 从栈中取顶上的结点。
       * @return 栈中顶上的结点。
       */
      inline StackNode& Top() {
        return nodes.Back();
      }

      /*
       * @brief 弹出栈顶的结点。
       */
      inline void Pop() {
        if (!nodes.Empty()) {
          nodes.Back().Clear();
          nodes.PopBack();
        }
      }
    } stack_for_finding_lanes;

    // 点列
    common::StaticVector<common::Vec2d,
        common::Path::kMaxPathPointNum> points_2d_1;
    common::StaticVector<common::Vec2d,
        common::Path::kMaxPathPointNum> points_2d_2;
  } temp_data_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_HD_MAP_H_


